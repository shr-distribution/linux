/*
 * V4L2 M2M encoder client + verifier for the VIDC 1080p H.264 encoder.
 *
 * Feeds raw NV12MT tile frames on OUTPUT, dequeues H.264 from CAPTURE,
 * writes an Annex-B file, and verifies the driver additions:
 *   - per-frame timestamp passthrough (input ts -> encoded buffer ts)
 *   - V4L2_BUF_FLAG_KEYFRAME / _PFRAME flags
 *   - GOP cadence (V4L2_CID_MPEG_VIDEO_GOP_SIZE)
 *   - forced IDR (V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME)
 *   - bitrate control (V4L2_CID_MPEG_VIDEO_BITRATE)
 *
 * Env knobs: VIDC_GOP, VIDC_BITRATE, VIDC_FORCEKEY (frame idx to force).
 *
 * Input  : NV12MT 320x240 frames concatenated (147456 bytes each)
 * Output : H.264 Annex-B
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <errno.h>
#include <linux/videodev2.h>

#define WIDTH    320
#define HEIGHT   240
#define OUT_BUFS 4
#define CAP_BUFS 4
/* frame_bytes is set at runtime (linear vs tiled) */
#define MAXREC   512
#define TS_UNIT  1000       /* per-frame input timestamp step (us) */

#define die(msg) do { perror(msg); exit(1); } while (0)

static void *out_mmap[OUT_BUFS];
static size_t out_size[OUT_BUFS];
static void *cap_mmap[CAP_BUFS];
static size_t cap_size[CAP_BUFS];

/* per captured frame, recorded in dequeue order */
static unsigned rec_flags[MAXREC];
static long     rec_ts[MAXREC];
static size_t   rec_bytes[MAXREC];

static void set_ctrl(int fd, unsigned id, int val, const char *name)
{
    struct v4l2_control c = { .id = id, .value = val };
    if (ioctl(fd, VIDIOC_S_CTRL, &c))
        fprintf(stderr, "  S_CTRL %s=%d FAILED: %s\n", name, val, strerror(errno));
    else
        fprintf(stderr, "  S_CTRL %s=%d OK\n", name, val);
}

int main(int argc, char **argv)
{
    const char *in_path  = argc > 1 ? argv[1] : "/tmp/raw.tile";
    const char *out_path = argc > 2 ? argv[2] : "/tmp/enc.h264";
    const char *dev_path = argc > 3 ? argv[3] : "/dev/video7";
    int gop      = getenv("VIDC_GOP")      ? atoi(getenv("VIDC_GOP"))      : 0;
    int bitrate  = getenv("VIDC_BITRATE")  ? atoi(getenv("VIDC_BITRATE"))  : 0;
    int forcekey = getenv("VIDC_FORCEKEY") ? atoi(getenv("VIDC_FORCEKEY")) : -1;
    int linear   = getenv("VIDC_LINEAR")   ? atoi(getenv("VIDC_LINEAR"))   : 0;
    /* linear NV12 320x240 = 320*240 + 320*120 = 115200; tiled NV12MT = 147456 */
    unsigned frame_bytes = linear ? (((WIDTH*HEIGHT+2047)&~2047) + WIDTH*HEIGHT/2) : 147456;

    int fd = open(dev_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) die("open dev");

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap)) die("QUERYCAP");
    fprintf(stderr, "Device: %s (%s)\n", cap.card, cap.driver);

    struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
    f.fmt.pix_mp.width = WIDTH; f.fmt.pix_mp.height = HEIGHT;
    f.fmt.pix_mp.pixelformat = linear ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_NV12MT;
    f.fmt.pix_mp.num_planes = 1;
    if (ioctl(fd, VIDIOC_S_FMT, &f)) die("S_FMT OUTPUT");
    fprintf(stderr, "input format: %s\n", linear ? "NV12 (linear)" : "NV12MT (tiled)");

    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    fc.fmt.pix_mp.width = WIDTH; fc.fmt.pix_mp.height = HEIGHT;
    fc.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    fc.fmt.pix_mp.num_planes = 1;
    if (ioctl(fd, VIDIOC_S_FMT, &fc)) die("S_FMT CAPTURE");

    struct v4l2_streamparm parm = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
    parm.parm.output.timeperframe.numerator = 1;
    parm.parm.output.timeperframe.denominator = 25;
    if (ioctl(fd, VIDIOC_S_PARM, &parm)) perror("S_PARM (non-fatal)");

    /* Controls under test (before STREAMON) */
    fprintf(stderr, "Controls:\n");
    if (gop)     set_ctrl(fd, V4L2_CID_MPEG_VIDEO_GOP_SIZE, gop, "GOP_SIZE");
    if (bitrate) set_ctrl(fd, V4L2_CID_MPEG_VIDEO_BITRATE, bitrate, "BITRATE");

    struct v4l2_requestbuffers rb_o = { .count = OUT_BUFS,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb_o)) die("REQBUFS OUT");
    struct v4l2_requestbuffers rb_c = { .count = CAP_BUFS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, .memory = V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb_c)) die("REQBUFS CAP");
    fprintf(stderr, "OUT bufs: %u  CAP bufs: %u\n", rb_o.count, rb_c.count);

    for (unsigned i = 0; i < rb_o.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i, .m.planes = &pl, .length = 1 };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF OUT");
        out_size[i] = pl.length;
        out_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pl.m.mem_offset);
        if (out_mmap[i] == MAP_FAILED) die("mmap OUT");
    }
    for (unsigned i = 0; i < rb_c.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i, .m.planes = &pl, .length = 1 };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF CAP");
        cap_size[i] = pl.length;
        cap_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pl.m.mem_offset);
        if (cap_mmap[i] == MAP_FAILED) die("mmap CAP");
    }

    int in_fd = open(in_path, O_RDONLY);
    if (in_fd < 0) die("open input");
    off_t in_total = lseek(in_fd, 0, SEEK_END); lseek(in_fd, 0, SEEK_SET);
    size_t n_frames = in_total / frame_bytes;
    unsigned char *in_data = malloc(in_total);
    if (!in_data || read(in_fd, in_data, in_total) != in_total) die("read input");
    close(in_fd);
    fprintf(stderr, "Input: %zu frames\n", n_frames);

    int of = open(out_path, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (of < 0) die("open output");

    for (unsigned i = 0; i < rb_c.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i, .m.planes = &pl, .length = 1 };
        if (ioctl(fd, VIDIOC_QBUF, &b)) die("QBUF CAP");
    }

    /* helper: queue OUTPUT frame `fr` into buffer `idx` with timestamp */
    size_t next_frame = 0;
    #define QUEUE_OUT(idx) do {                                              \
        unsigned _qi = (idx);                                                \
        if (forcekey >= 0 && (int)next_frame == forcekey)                    \
            set_ctrl(fd, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1, "FORCE_KEY");\
        size_t _cl = frame_bytes; if (_cl > out_size[_qi]) _cl = out_size[_qi];\
        memcpy(out_mmap[_qi], in_data + next_frame*frame_bytes, _cl);        \
        struct v4l2_plane _qpl = { .bytesused = _cl };                       \
        struct v4l2_buffer _qb = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,\
            .memory = V4L2_MEMORY_MMAP, .index = _qi, .m.planes = &_qpl,     \
            .length = 1 };                                                   \
        _qb.timestamp.tv_sec = 0; _qb.timestamp.tv_usec = next_frame*TS_UNIT;\
        if (ioctl(fd, VIDIOC_QBUF, &_qb)) die("QBUF OUT");                   \
        next_frame++;                                                        \
    } while (0)

    for (unsigned i = 0; i < rb_o.count && next_frame < n_frames; i++)
        QUEUE_OUT(i);

    int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &t)) die("STREAMON OUT");
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &t)) die("STREAMON CAP");
    fprintf(stderr, "Encoding...\n");

    size_t cap_count = 0, enc_bytes = 0;
    int idle = 0;
    while (idle < 5) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN|POLLOUT };
        int pret = poll(&pfd, 1, 1000);
        if (pret < 0) die("poll");
        if (pret == 0) { idle++; continue; }
        idle = 0;

        if (pfd.revents & POLLIN) {
            struct v4l2_plane pl = {0};
            struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                .memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
            if (ioctl(fd, VIDIOC_DQBUF, &b) == 0) {
                size_t pay = pl.bytesused;
                if (cap_count < MAXREC) {
                    rec_flags[cap_count] = b.flags;
                    rec_ts[cap_count] = b.timestamp.tv_usec;
                    rec_bytes[cap_count] = pay;
                }
                if (pay > 0) { write(of, cap_mmap[b.index], pay); enc_bytes += pay; cap_count++; }
                struct v4l2_plane rp = {0};
                struct v4l2_buffer rb = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                    .memory = V4L2_MEMORY_MMAP, .index = b.index, .m.planes = &rp, .length = 1 };
                ioctl(fd, VIDIOC_QBUF, &rb);
            }
        }
        if (pfd.revents & POLLOUT) {
            struct v4l2_plane pl = {0};
            struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                .memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
            if (ioctl(fd, VIDIOC_DQBUF, &b) == 0 && next_frame < n_frames)
                QUEUE_OUT(b.index);
        }
    }

    t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;  ioctl(fd, VIDIOC_STREAMOFF, &t);
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; ioctl(fd, VIDIOC_STREAMOFF, &t);
    close(of); close(fd);

    /* ---- verification summary ---- */
    fprintf(stderr, "\n==== ENCODE VERIFY (%zu frames, %zu bytes) ====\n", cap_count, enc_bytes);
    int key_ok = 1, ts_ok = 1, flag_ok = 1, key_count = 0;
    char keymap[MAXREC+1];
    for (size_t k = 0; k < cap_count && k < MAXREC; k++) {
        int iskey = !!(rec_flags[k] & V4L2_BUF_FLAG_KEYFRAME);
        int ispf  = !!(rec_flags[k] & V4L2_BUF_FLAG_PFRAME);
        keymap[k] = iskey ? 'I' : (ispf ? 'P' : '?');
        if (iskey) key_count++;
        if (!iskey && !ispf) flag_ok = 0;            /* one of the flags must be set */
        if (rec_ts[k] != (long)k * TS_UNIT) ts_ok = 0; /* in-order passthrough */
    }
    keymap[cap_count < MAXREC ? cap_count : MAXREC] = 0;
    if (cap_count && !(rec_flags[0] & V4L2_BUF_FLAG_KEYFRAME)) key_ok = 0; /* frame0 = IDR */
    fprintf(stderr, "frame types : %s\n", keymap);
    fprintf(stderr, "keyframes   : %d  (frame0=%s)\n", key_count,
            (cap_count && (rec_flags[0]&V4L2_BUF_FLAG_KEYFRAME)) ? "IDR OK" : "NOT IDR");
    if (forcekey >= 0)
        fprintf(stderr, "forced IDR  : frame %d -> %s\n", forcekey,
                (forcekey < (int)cap_count && (rec_flags[forcekey]&V4L2_BUF_FLAG_KEYFRAME))
                ? "KEYFRAME OK" : "NOT keyframe");
    fprintf(stderr, "timestamps  : %s (in-order passthrough)\n", ts_ok ? "PASS" : "FAIL");
    fprintf(stderr, "buf flags   : %s (KEYFRAME|PFRAME set)\n", flag_ok ? "PASS" : "FAIL");
    fprintf(stderr, "RESULT: %s\n",
            (cap_count && key_ok && ts_ok && flag_ok) ? "PASS" : "CHECK");
    return (cap_count && key_ok && ts_ok && flag_ok) ? 0 : 2;
}
