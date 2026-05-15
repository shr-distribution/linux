#!/usr/bin/env python3
"""
VIDC encoder test: feed raw NV12MT frames into /dev/video7,
capture H.264 output, verify firmware loads and encoder works.

Usage: python3 vidc_enc_test.py [input.raw] [output.h264]
"""

import os, sys, fcntl, struct, mmap, ctypes, time

# ── V4L2 constants ────────────────────────────────────────────────────────────
V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  = 10
V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9
V4L2_MEMORY_MMAP   = 1
V4L2_PIX_FMT_NV12MT = 0x32314d54   # 'TM12'
V4L2_PIX_FMT_H264   = 0x34363248   # 'H264'
V4L2_FIELD_NONE     = 1

# ioctl numbers — derived from kernel uapi on 32-bit ARM.
# v4l2_buffer size = 88 bytes (0x58): __kernel_v4l2_timeval uses long long.
# v4l2_format size = 204 bytes (0xcc).
# v4l2_requestbuffers size = 20 bytes (0x14).
VIDIOC_S_FMT     = 0xc0cc5605
VIDIOC_REQBUFS   = 0xc0145608
VIDIOC_QUERYBUF  = 0xc0585609
VIDIOC_QBUF      = 0xc058560f
VIDIOC_DQBUF     = 0xc0585611
VIDIOC_STREAMON  = 0x40045612
VIDIOC_STREAMOFF = 0x40045613

# v4l2_buffer layout (88 bytes, 32-bit ARM, multiplanar):
#   index(4) type(4) bytesused(4) flags(4) field(4)
#   timestamp: tv_sec(8) tv_usec(8)          <- __kernel_v4l2_timeval
#   timecode: type(4) flags(4) frames(1) seconds(1) minutes(1) hours(1) userbits(4)
#   sequence(4) memory(4)
#   m.planes(4)                               <- pointer (32-bit)
#   length(4) reserved2(4) request_fd(4)
# Offsets:
BUF_OFF_INDEX     =  0
BUF_OFF_TYPE      =  4
BUF_OFF_BYTESUSED =  8
BUF_OFF_FLAGS     = 12
BUF_OFF_FIELD     = 16
BUF_OFF_TV_SEC    = 20   # 8 bytes
BUF_OFF_TV_USEC   = 28   # 8 bytes
BUF_OFF_TC        = 36   # 16 bytes
BUF_OFF_SEQUENCE  = 52
BUF_OFF_MEMORY    = 56
BUF_OFF_PLANES    = 60   # union m: planes pointer (4 bytes on 32-bit)
BUF_OFF_LENGTH    = 64
BUF_OFF_RESERVED2 = 68
BUF_OFF_REQFD     = 72
BUF_SIZE          = 88   # must match 0x58 in ioctl number

# v4l2_plane layout (44 bytes):
#   bytesused(4) length(4) m.mem_offset(4) data_offset(4) reserved(11*4=44)
# Total = 4+4+4+4+44 = 60? Let's use kernel definition:
#   bytesused(4) length(4) union m(4) data_offset(4) reserved[11](44) = 56 bytes
PLANE_OFF_BYTESUSED  = 0
PLANE_OFF_LENGTH     = 4
PLANE_OFF_MEM_OFFSET = 8   # union m: mem_offset
PLANE_OFF_DATA_OFF   = 12
PLANE_SIZE           = 56  # sizeof(struct v4l2_plane)

WIDTH  = 320
HEIGHT = 240
Y_STRIDE  = (WIDTH  + 127) & ~127   # 384
Y_HEIGHT  = (HEIGHT +  31) & ~31    # 256
UV_HEIGHT = (HEIGHT // 2 + 31) & ~31  # 128
FRAME_SIZE_RAW = Y_STRIDE * (Y_HEIGHT + UV_HEIGHT)   # 147456
FRAME_SIZE_ENC = max((WIDTH * HEIGHT * 2) // 8, 512 * 1024)  # 512 KB

NUM_BUFS   = 4
NUM_FRAMES = 10

DEV = "/dev/video7"

# ── ctypes structs ────────────────────────────────────────────────────────────

class v4l2_plane(ctypes.Structure):
    _fields_ = [
        ("bytesused",    ctypes.c_uint32),
        ("length",       ctypes.c_uint32),
        ("m_mem_offset", ctypes.c_uint32),   # union: mem_offset for MMAP
        ("data_offset",  ctypes.c_uint32),
        ("reserved",     ctypes.c_uint32 * 11),
    ]

class timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

class v4l2_timecode(ctypes.Structure):
    _fields_ = [
        ("type",     ctypes.c_uint32),
        ("flags",    ctypes.c_uint32),
        ("frames",   ctypes.c_uint8),
        ("seconds",  ctypes.c_uint8),
        ("minutes",  ctypes.c_uint8),
        ("hours",    ctypes.c_uint8),
        ("userbits", ctypes.c_uint8 * 4),
    ]

class v4l2_buffer_mplane(ctypes.Structure):
    """v4l2_buffer for multiplanar queues (32-bit ARM layout)"""
    _fields_ = [
        ("index",      ctypes.c_uint32),
        ("type",       ctypes.c_uint32),
        ("bytesused",  ctypes.c_uint32),
        ("flags",      ctypes.c_uint32),
        ("field",      ctypes.c_uint32),
        ("timestamp",  timeval),
        ("timecode",   v4l2_timecode),
        ("sequence",   ctypes.c_uint32),
        ("memory",     ctypes.c_uint32),
        ("planes",     ctypes.c_uint32),   # __u32 pointer on 32-bit
        ("length",     ctypes.c_uint32),
        ("reserved2",  ctypes.c_uint32),
        ("request_fd", ctypes.c_int32),
    ]


def do_ioctl(fd, req, arg):
    fcntl.ioctl(fd, req, arg)


def s_fmt(fd, buf_type, pixfmt, width, height):
    fmt = bytearray(204)
    struct.pack_into("I", fmt, 0, buf_type)
    # pix_mp starts at offset 4
    struct.pack_into("I", fmt, 4,  width)
    struct.pack_into("I", fmt, 8,  height)
    struct.pack_into("I", fmt, 12, pixfmt)
    struct.pack_into("I", fmt, 16, V4L2_FIELD_NONE)
    # plane_fmt[0] at pix_mp+20: sizeimage(4)+bytesperline(4)+reserved(16)
    if buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
        struct.pack_into("I", fmt, 4+20,   FRAME_SIZE_RAW)
        struct.pack_into("I", fmt, 4+20+4, Y_STRIDE)
    else:
        struct.pack_into("I", fmt, 4+20, FRAME_SIZE_ENC)
    do_ioctl(fd, VIDIOC_S_FMT, fmt)


def reqbufs(fd, buf_type, count):
    buf = bytearray(struct.pack("IIII4B", count, buf_type, V4L2_MEMORY_MMAP,
                                0, 0, 0, 0, 0))
    do_ioctl(fd, VIDIOC_REQBUFS, buf)
    return struct.unpack_from("I", buf)[0]


def querybuf(fd, buf_type, index):
    plane = v4l2_plane()
    buf   = v4l2_buffer_mplane()
    buf.index   = index
    buf.type    = buf_type
    buf.memory  = V4L2_MEMORY_MMAP
    buf.length  = 1
    buf.planes  = ctypes.addressof(plane)
    do_ioctl(fd, VIDIOC_QUERYBUF, buf)
    return plane.m_mem_offset, plane.length


def qbuf(fd, buf_type, index, bytesused=0):
    plane = v4l2_plane()
    plane.bytesused = bytesused
    buf = v4l2_buffer_mplane()
    buf.index   = index
    buf.type    = buf_type
    buf.memory  = V4L2_MEMORY_MMAP
    buf.length  = 1
    buf.planes  = ctypes.addressof(plane)
    do_ioctl(fd, VIDIOC_QBUF, buf)


def dqbuf(fd, buf_type):
    plane = v4l2_plane()
    buf   = v4l2_buffer_mplane()
    buf.type   = buf_type
    buf.memory = V4L2_MEMORY_MMAP
    buf.length = 1
    buf.planes = ctypes.addressof(plane)
    do_ioctl(fd, VIDIOC_DQBUF, buf)
    return buf.index, plane.bytesused


def streamon(fd, buf_type):
    do_ioctl(fd, VIDIOC_STREAMON,  bytearray(struct.pack("I", buf_type)))


def streamoff(fd, buf_type):
    do_ioctl(fd, VIDIOC_STREAMOFF, bytearray(struct.pack("I", buf_type)))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    infile  = sys.argv[1] if len(sys.argv) > 1 else "/tmp/test_nv12mt.raw"
    outfile = sys.argv[2] if len(sys.argv) > 2 else "/tmp/out.h264"

    print("=" * 60)
    print("VIDC encoder firmware + encode test")
    print("=" * 60)
    print(f"  Device : {DEV}")
    print(f"  Input  : {infile}  ({FRAME_SIZE_RAW} B/frame)")
    print(f"  Output : {outfile}")
    print(f"  Format : {WIDTH}x{HEIGHT} NV12MT → H.264, {NUM_FRAMES} frames")
    print()

    # ── Step 1: open ──────────────────────────────────────────────────────────
    fd = os.open(DEV, os.O_RDWR)
    print(f"[1] open({DEV}) = fd {fd}  OK")

    # ── Step 2: set formats ───────────────────────────────────────────────────
    try:
        s_fmt(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,  V4L2_PIX_FMT_NV12MT, WIDTH, HEIGHT)
        print(f"[2] S_FMT OUTPUT  (NV12MT {WIDTH}x{HEIGHT})  OK")
    except OSError as e:
        print(f"[2] S_FMT OUTPUT  FAILED: {e}")
        sys.exit(1)

    try:
        s_fmt(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_PIX_FMT_H264,   WIDTH, HEIGHT)
        print(f"[3] S_FMT CAPTURE (H264   {WIDTH}x{HEIGHT})  OK")
    except OSError as e:
        print(f"[3] S_FMT CAPTURE FAILED: {e}")
        sys.exit(1)

    # ── Step 3: allocate buffers ──────────────────────────────────────────────
    n_out = reqbufs(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,  NUM_BUFS)
    print(f"[4] REQBUFS OUTPUT  → {n_out} buffers")
    n_cap = reqbufs(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, NUM_BUFS)
    print(f"[5] REQBUFS CAPTURE → {n_cap} buffers")

    if n_out == 0 or n_cap == 0:
        print("\nFAIL: REQBUFS returned 0 buffers — driver rejected allocation")
        os.close(fd)
        sys.exit(1)

    # ── Step 4: mmap buffers ──────────────────────────────────────────────────
    out_maps = []
    for i in range(n_out):
        offset, length = querybuf(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, i)
        m = mmap.mmap(fd, length, mmap.MAP_SHARED,
                      mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
        out_maps.append((m, length))
    print(f"[6] mmap'd {n_out} OUTPUT  buffers ({out_maps[0][1]} B each)")

    cap_maps = []
    for i in range(n_cap):
        offset, length = querybuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, i)
        m = mmap.mmap(fd, length, mmap.MAP_SHARED,
                      mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
        cap_maps.append((m, length))
    print(f"[7] mmap'd {n_cap} CAPTURE buffers ({cap_maps[0][1]} B each)")

    # ── Step 5: STREAMON — this triggers firmware load ────────────────────────
    try:
        streamon(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
        print("[8] STREAMON OUTPUT  OK  ← firmware load + open_channel here")
    except OSError as e:
        print(f"[8] STREAMON OUTPUT  FAILED: {e}")
        print("    Check dmesg — firmware may be missing or boot timed out")
        os.close(fd)
        sys.exit(1)

    try:
        streamon(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        print("[9] STREAMON CAPTURE OK")
    except OSError as e:
        print(f"[9] STREAMON CAPTURE FAILED: {e}")
        os.close(fd)
        sys.exit(1)

    # ── Step 6: pre-queue all capture buffers ─────────────────────────────────
    for i in range(n_cap):
        qbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, i, 0)
    print(f"[10] Pre-queued {n_cap} capture buffers")

    # ── Step 7: encode loop ───────────────────────────────────────────────────
    in_data = open(infile, "rb").read()
    out_fh  = open(outfile, "wb")
    frames_encoded = 0
    total_bytes    = 0

    print(f"\n[11] Encoding {NUM_FRAMES} frames (timeout 5s/frame)...")
    for frame_idx in range(NUM_FRAMES):
        buf_idx = frame_idx % n_out

        # Fill output buffer with raw frame data
        src_off = (frame_idx * FRAME_SIZE_RAW) % max(1, len(in_data) - FRAME_SIZE_RAW)
        frame_data = in_data[src_off : src_off + FRAME_SIZE_RAW]
        if len(frame_data) < FRAME_SIZE_RAW:
            frame_data = (frame_data + in_data)[:FRAME_SIZE_RAW]

        m, length = out_maps[buf_idx]
        m.seek(0)
        m.write(frame_data[:min(FRAME_SIZE_RAW, length)])

        # Queue the raw input frame
        qbuf(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, buf_idx, FRAME_SIZE_RAW)

        # Dequeue encoded output (blocks until firmware completes)
        try:
            cap_idx, bytesused = dqbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        except OSError as e:
            print(f"  Frame {frame_idx}: DQBUF CAPTURE failed: {e}")
            break

        if bytesused > 0:
            cap_maps[cap_idx][0].seek(0)
            out_fh.write(cap_maps[cap_idx][0].read(bytesused))
            frames_encoded += 1
            total_bytes += bytesused
            print(f"  Frame {frame_idx:2d}: {bytesused:6d} B encoded → cap buf[{cap_idx}]")
        else:
            print(f"  Frame {frame_idx:2d}: bytesused=0 (firmware not producing output)")

        # Re-queue capture buffer for next frame
        qbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, cap_idx, 0)

        # Dequeue the input buffer we just consumed
        try:
            dqbuf(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
        except OSError:
            pass

    out_fh.close()

    # ── Step 8: STREAMOFF + cleanup ───────────────────────────────────────────
    streamoff(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
    streamoff(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    for m, _ in out_maps + cap_maps:
        m.close()
    os.close(fd)

    # ── Result ────────────────────────────────────────────────────────────────
    out_size = os.path.getsize(outfile)
    print()
    print("=" * 60)
    print(f"Frames encoded : {frames_encoded} / {NUM_FRAMES}")
    print(f"Output size    : {out_size} bytes  ({outfile})")
    if frames_encoded > 0:
        print("RESULT: PASS — firmware loaded, encoder produced H.264 output")
    else:
        print("RESULT: FAIL — no frames encoded, check dmesg for errors")
    print("=" * 60)
    sys.exit(0 if frames_encoded > 0 else 1)


if __name__ == "__main__":
    main()
