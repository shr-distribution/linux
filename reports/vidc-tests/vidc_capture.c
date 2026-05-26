/*
 * Minimal V4L2 M2M stateful-decoder client for the VIDC 1080p.
 *
 * Implements the standard V4L2 stateful decoder protocol:
 *   1. Subscribe to V4L2_EVENT_SOURCE_CHANGE.
 *   2. REQBUFS+QBUF the first OUTPUT buffer (the SPS+PPS prefix).
 *   3. STREAMON OUTPUT.
 *   4. Wait for SOURCE_CHANGE (means firmware parsed SPS).
 *   5. REQBUFS+QBUF CAPTURE buffers; STREAMON CAPTURE.
 *   6. Feed remaining OUTPUT buffers; dequeue CAPTURE frames in parallel.
 *
 * Bypasses GStreamer's single-buffer behaviour so we can confirm the
 * kernel delivers every decoded frame's NV12_64Z32 data into a CAPTURE
 * buffer where userspace can collect it.
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
#include <time.h>
#include <linux/videodev2.h>

#define WIDTH    320
#define HEIGHT   240
#define OUT_BUFS 4
/* Must be >= the kernel's CAPTURE count. The VIDC stateful decoder sizes the
 * CAPTURE pool as min_dpb + headroom (up to VIDC_DPB_REG_SLOTS = 32), so an
 * 8-element array overflowed into adjacent globals (corrupting got_eos) once
 * min_dpb pushed the count past 8. Size to the kernel maximum. */
#define CAP_BUFS 32

#define die(msg) do { perror(msg); exit(1); } while (0)

static void *out_mmap[OUT_BUFS];
static size_t out_size[OUT_BUFS];
static int    out_avail[OUT_BUFS];
static void *cap_mmap[CAP_BUFS];
static size_t cap_size[CAP_BUFS];

#define MAXREC 2048
static long rec_ts[MAXREC];      /* output timestamp (us) in display order */
static size_t rec_n;
static int got_last, got_eos;

int main(int argc, char **argv)
{
    const char *in_path  = argc > 1 ? argv[1] : "/tmp/test320.h264";
    const char *out_path = argc > 2 ? argv[2] : "/tmp/raw.tile";
    const char *dev_path = argc > 3 ? argv[3] : "/dev/video6";

    int fd = open(dev_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) die("open dev");

    /* OUTPUT format */
    struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
    f.fmt.pix_mp.width        = WIDTH;
    f.fmt.pix_mp.height       = HEIGHT;
    f.fmt.pix_mp.pixelformat  = V4L2_PIX_FMT_H264;
    f.fmt.pix_mp.num_planes   = 1;
    f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
    if (ioctl(fd, VIDIOC_S_FMT, &f)) die("S_FMT OUTPUT");

    /* CAPTURE format: VIDC_LINEAR=1 -> linear NV12 (de-tiled), else tiled TM12 */
    int linear = getenv("VIDC_LINEAR") ? atoi(getenv("VIDC_LINEAR")) : 0;
    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    fc.fmt.pix_mp.width       = WIDTH;
    fc.fmt.pix_mp.height      = HEIGHT;
    fc.fmt.pix_mp.pixelformat = linear ? V4L2_PIX_FMT_NV12 :
				v4l2_fourcc('T', 'M', '1', '2');
    fc.fmt.pix_mp.num_planes  = 1;
    fprintf(stderr, "CAPTURE format: %s\n", linear ? "NV12 (linear)" : "NV12MT (tiled)");
    if (ioctl(fd, VIDIOC_S_FMT, &fc)) die("S_FMT CAPTURE");
    fprintf(stderr, "CAP fmt: %ux%u size=%u\n",
            fc.fmt.pix_mp.width, fc.fmt.pix_mp.height,
            fc.fmt.pix_mp.plane_fmt[0].sizeimage);

    /* Subscribe to SOURCE_CHANGE */
    struct v4l2_event_subscription es = {
        .type = V4L2_EVENT_SOURCE_CHANGE,
    };
    if (ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &es)) die("SUBSCRIBE_EVENT");
    struct v4l2_event_subscription ee = { .type = V4L2_EVENT_EOS };
    if (ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &ee)) perror("SUBSCRIBE EOS (non-fatal)");

    /* REQBUFS OUTPUT */
    struct v4l2_requestbuffers rb_o = {
        .count = OUT_BUFS,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb_o)) die("REQBUFS OUT");
    fprintf(stderr, "OUT bufs: %u\n", rb_o.count);

    /* Map OUTPUT */
    for (unsigned i = 0; i < rb_o.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i,
            .m.planes = &pl, .length = 1,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF OUT");
        out_size[i] = pl.length;
        out_mmap[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, pl.m.mem_offset);
        if (out_mmap[i] == MAP_FAILED) die("mmap OUT");
        out_avail[i] = 1;
    }

    /* Read input */
    int in_fd = open(in_path, O_RDONLY);
    if (in_fd < 0) die("open input");
    off_t in_total = lseek(in_fd, 0, SEEK_END);
    lseek(in_fd, 0, SEEK_SET);
    unsigned char *in_data = malloc(in_total);
    if (read(in_fd, in_data, in_total) != in_total) die("read input");
    close(in_fd);
    fprintf(stderr, "Read %lld bytes from %s\n", (long long)in_total, in_path);

    /* Feed whole file as single OUTPUT buffer 0 */
    size_t payload = in_total > (off_t)out_size[0] ? out_size[0] : (size_t)in_total;
    memcpy(out_mmap[0], in_data, payload);

    struct v4l2_plane pl0 = { .bytesused = payload };
    struct v4l2_buffer b0 = {
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
        .memory = V4L2_MEMORY_MMAP, .index = 0,
        .m.planes = &pl0, .length = 1,
    };
    if (ioctl(fd, VIDIOC_QBUF, &b0)) die("QBUF OUT[0]");
    out_avail[0] = 0;
    fprintf(stderr, "Queued OUT[0] with %zu bytes\n", payload);

    /* STREAMON OUTPUT only */
    int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &t)) die("STREAMON OUT");
    fprintf(stderr, "STREAMON OUT done, waiting for SOURCE_CHANGE\n");

    /* Wait for SOURCE_CHANGE event */
    int got_src_change = 0;
    int loops = 0;
    while (!got_src_change && loops++ < 30) {
        struct pollfd pfd = { .fd = fd, .events = POLLPRI | POLLOUT };
        if (poll(&pfd, 1, 1000) < 0) die("poll");

        if (pfd.revents & POLLPRI) {
            struct v4l2_event ev;
            while (ioctl(fd, VIDIOC_DQEVENT, &ev) == 0) {
                fprintf(stderr, "Got event type=%u\n", ev.type);
                if (ev.type == V4L2_EVENT_SOURCE_CHANGE) {
                    got_src_change = 1;
                    break;
                }
            }
        }
        /* Drain any OUTPUT buffer that came back */
        if (pfd.revents & POLLOUT) {
            struct v4l2_plane dp = {0};
            struct v4l2_buffer db = {
                .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                .memory = V4L2_MEMORY_MMAP,
                .m.planes = &dp, .length = 1,
            };
            if (ioctl(fd, VIDIOC_DQBUF, &db) == 0) {
                out_avail[db.index] = 1;
                fprintf(stderr, "Pre-SRC OUT[%u] returned\n", db.index);
            }
        }
    }
    if (!got_src_change) {
        fprintf(stderr, "Never got SOURCE_CHANGE event\n");
    }

    /* Re-query CAPTURE format (size may have updated) */
    if (ioctl(fd, VIDIOC_G_FMT, &fc) == 0) {
        fprintf(stderr, "CAP fmt after SRC_CHG: %ux%u size=%u\n",
                fc.fmt.pix_mp.width, fc.fmt.pix_mp.height,
                fc.fmt.pix_mp.plane_fmt[0].sizeimage);
    }

    /* NEW: MIN_BUFFERS_FOR_CAPTURE control (must reflect firmware min_dpb) */
    int min_cap = 5;
    {
        struct v4l2_control mc = { .id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE };
        if (ioctl(fd, VIDIOC_G_CTRL, &mc) == 0) {
            fprintf(stderr, "MIN_BUFFERS_FOR_CAPTURE = %d\n", mc.value);
            if (mc.value > 0)
                min_cap = mc.value;
        } else {
            fprintf(stderr, "MIN_BUFFERS_FOR_CAPTURE: G_CTRL FAILED: %s\n",
                    strerror(errno));
        }
    }
    /* NEW: visible crop rectangle via G_SELECTION(COMPOSE) */
    {
        struct v4l2_selection sel = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
            .target = V4L2_SEL_TGT_COMPOSE,
        };
        if (ioctl(fd, VIDIOC_G_SELECTION, &sel) == 0)
            fprintf(stderr, "G_SELECTION COMPOSE (visible) = %dx%d+%d+%d\n",
                    sel.r.width, sel.r.height, sel.r.left, sel.r.top);
        else
            fprintf(stderr, "G_SELECTION: FAILED: %s\n", strerror(errno));
    }

    /* REQBUFS CAPTURE: ask for min_dpb + a little display headroom, not the
     * whole array — over-requesting bloats both the CAPTURE pool and the
     * per-slot MV pool and exhausts the SMI carveout at higher resolutions. */
    int want_cap = min_cap + 4;
    if (want_cap > CAP_BUFS) want_cap = CAP_BUFS;
    struct v4l2_requestbuffers rb_c = {
        .count = want_cap,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb_c)) die("REQBUFS CAP");
    fprintf(stderr, "CAP bufs: %u\n", rb_c.count);
    if (rb_c.count > CAP_BUFS) {
        fprintf(stderr, "CAP count %u exceeds CAP_BUFS %d — clamping\n",
                rb_c.count, CAP_BUFS);
        rb_c.count = CAP_BUFS;
    }

    /* Map CAPTURE */
    for (unsigned i = 0; i < rb_c.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i,
            .m.planes = &pl, .length = 1,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF CAP");
        cap_size[i] = pl.length;
        cap_mmap[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, pl.m.mem_offset);
        if (cap_mmap[i] == MAP_FAILED) die("mmap CAP");
    }

    /* QBUF all CAPTURE buffers */
    for (unsigned i = 0; i < rb_c.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i,
            .m.planes = &pl, .length = 1,
        };
        if (ioctl(fd, VIDIOC_QBUF, &b)) die("QBUF CAP");
    }

    /* STREAMON CAPTURE */
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &t)) die("STREAMON CAP");
    fprintf(stderr, "STREAMON CAP done, decoding...\n");

    /*
     * Build an AU table now (each VCL NAL starts a new access unit).  We
     * feed one AU per OUTPUT buffer after SOURCE_CHANGE so the firmware
     * sees one frame per device_run.
     */
    size_t *au_off = malloc(1024 * sizeof(*au_off));
    size_t au_n = 0, au_cap = 1024;
    {
        int in_au = 0;
        for (off_t i = 0; i + 4 < in_total; i++) {
            int sc = 0;
            size_t off_nal = 0;
            if (in_data[i] == 0 && in_data[i+1] == 0 &&
                in_data[i+2] == 0 && in_data[i+3] == 1) {
                sc = 4; off_nal = i + 4;
            } else if (in_data[i] == 0 && in_data[i+1] == 0 &&
                       in_data[i+2] == 1) {
                sc = 3; off_nal = i + 3;
            }
            if (!sc) continue;
            int nal_type = in_data[off_nal] & 0x1f;
            int is_vcl = (nal_type >= 1 && nal_type <= 5);
            if (!in_au) {
                if (au_n == au_cap) {
                    au_cap *= 2;
                    au_off = realloc(au_off, au_cap * sizeof(*au_off));
                }
                au_off[au_n++] = i;
                in_au = 1;
            }
            if (is_vcl) in_au = 0;
            i += sc - 1;
        }
    }
    fprintf(stderr, "Built %zu AUs after SOURCE_CHANGE\n", au_n);
    if (au_n == 0) {
        au_off[0] = 0;
        au_n = 1;
    }

    /* Queue first few OUTPUT buffers with AUs */
    size_t next_au = 0;
    for (unsigned i = 0; i < rb_o.count && next_au < au_n; i++) {
        if (!out_avail[i]) continue;
        size_t start = au_off[next_au];
        size_t end = (next_au + 1 < au_n) ? au_off[next_au + 1] : (size_t)in_total;
        size_t len = end - start;
        if (len > out_size[i]) len = out_size[i];
        memcpy(out_mmap[i], in_data + start, len);
        struct v4l2_plane plp = { .bytesused = len };
        struct v4l2_buffer bp = {
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
            .memory = V4L2_MEMORY_MMAP, .index = i,
            .m.planes = &plp, .length = 1,
        };
        bp.timestamp.tv_usec = (next_au + 1) * 1000;  /* decode-order tag */
        if (ioctl(fd, VIDIOC_QBUF, &bp))
            perror("QBUF OUT (FRAME_DATA)");
        else {
            out_avail[i] = 0;
            fprintf(stderr, "QBUF OUT[%u] AU[%zu] %zu bytes\n", i, next_au, len);
            next_au++;
        }
    }

    /* Open output file */
    int of = open(out_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (of < 0) die("open output");

    /* Drain loop */
    size_t cap_count = 0;
    int idle_polls = 0;
    int stop_sent = 0;
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    while (idle_polls < 5 && !got_eos) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN | POLLOUT | POLLPRI };
        int pret = poll(&pfd, 1, 1000);
        if (pret < 0) die("poll");
        if (pret == 0) {
            idle_polls++;
            fprintf(stderr, "Idle %d (captured=%zu)\n", idle_polls, cap_count);
            continue;
        }
        idle_polls = 0;

        if (pfd.revents & POLLPRI) {
            struct v4l2_event ev;
            while (ioctl(fd, VIDIOC_DQEVENT, &ev) == 0) {
                if (ev.type == V4L2_EVENT_EOS) {
                    got_eos = 1;
                    fprintf(stderr, "Got V4L2_EVENT_EOS\n");
                }
            }
        }
        if (pfd.revents & POLLIN) {
            struct v4l2_plane pl = {0};
            struct v4l2_buffer b = {
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                .memory = V4L2_MEMORY_MMAP,
                .m.planes = &pl, .length = 1,
            };
            if (ioctl(fd, VIDIOC_DQBUF, &b) == 0) {
                size_t pay = pl.bytesused;
                fprintf(stderr, "CAP[%u] payload=%zu flags=0x%x seq=%u ts=%ld\n",
                        b.index, pay, b.flags, b.sequence, (long)b.timestamp.tv_usec);
                if (b.flags & V4L2_BUF_FLAG_LAST)
                    got_last = 1;
                if (pay > 0) {
                    ssize_t w = write(of, cap_mmap[b.index], pay);
                    (void)w;
                    if (rec_n < MAXREC)
                        rec_ts[rec_n++] = b.timestamp.tv_usec;
                    cap_count++;
                }
                if (!(b.flags & V4L2_BUF_FLAG_LAST)) {
                    /* requeue */
                    struct v4l2_plane rp = {0};
                    struct v4l2_buffer rb = {
                        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                        .memory = V4L2_MEMORY_MMAP, .index = b.index,
                        .m.planes = &rp, .length = 1,
                    };
                    ioctl(fd, VIDIOC_QBUF, &rb);
                }
            }
        }
        if (pfd.revents & POLLOUT) {
            struct v4l2_plane pl = {0};
            struct v4l2_buffer b = {
                .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                .memory = V4L2_MEMORY_MMAP,
                .m.planes = &pl, .length = 1,
            };
            if (ioctl(fd, VIDIOC_DQBUF, &b) == 0) {
                out_avail[b.index] = 1;
                /* Refill with next AU */
                if (next_au < au_n) {
                    size_t start = au_off[next_au];
                    size_t end = (next_au + 1 < au_n) ?
                                  au_off[next_au + 1] : (size_t)in_total;
                    size_t len = end - start;
                    if (len > out_size[b.index]) len = out_size[b.index];
                    memcpy(out_mmap[b.index], in_data + start, len);
                    struct v4l2_plane qp = { .bytesused = len };
                    struct v4l2_buffer qb = {
                        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                        .memory = V4L2_MEMORY_MMAP, .index = b.index,
                        .m.planes = &qp, .length = 1,
                    };
                    qb.timestamp.tv_usec = (next_au + 1) * 1000;
                    if (ioctl(fd, VIDIOC_QBUF, &qb) == 0) {
                        out_avail[b.index] = 0;
                        next_au++;
                    }
                }
                /* All AUs fed -> request drain (STOP) once to flush DPB + get EOS. */
                if (next_au >= au_n && !stop_sent) {
                    struct v4l2_decoder_cmd dc = { .cmd = V4L2_DEC_CMD_STOP };
                    if (ioctl(fd, VIDIOC_DECODER_CMD, &dc) == 0)
                        fprintf(stderr, "DECODER_CMD STOP sent (drain)\n");
                    stop_sent = 1;
                }
            }
        }
    }

    t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ioctl(fd, VIDIOC_STREAMOFF, &t);
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(fd, VIDIOC_STREAMOFF, &t);
    close(of);
    close(fd);

    clock_gettime(CLOCK_MONOTONIC, &t1);
    double el = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;

    /* ---- verification summary ---- */
    fprintf(stderr, "\n==== DECODE VERIFY (%zu frames) ====\n", cap_count);
    fprintf(stderr, "DECODE FPS  : %.1f  (%zu frames, %.2fs, %ux%u)\n",
            el > 0 ? cap_count / el : 0.0, cap_count, el,
            fc.fmt.pix_mp.width, fc.fmt.pix_mp.height);
    /* timestamps: every output ts must be a non-zero multiple of 1000 we fed,
     * and all distinct (proves the frame-tag table restored the right PTS,
     * even under B-frame reorder). */
    int ts_ok = (cap_count > 0);
    int reordered = 0;
    long prev = -1;
    for (size_t k = 0; k < rec_n; k++) {
        if (rec_ts[k] <= 0 || rec_ts[k] % 1000 != 0) ts_ok = 0;
        for (size_t j = 0; j < k; j++)
            if (rec_ts[j] == rec_ts[k]) ts_ok = 0;   /* duplicate */
        if (prev >= 0 && rec_ts[k] < prev) reordered = 1;
        prev = rec_ts[k];
    }
    fprintf(stderr, "output ts   : ");
    for (size_t k = 0; k < rec_n && k < 48; k++)
        fprintf(stderr, "%ld ", rec_ts[k] / 1000);
    fprintf(stderr, "\n");
    fprintf(stderr, "timestamps  : %s (distinct, from input set)%s\n",
            ts_ok ? "PASS" : "FAIL",
            reordered ? "  [reorder observed]" : "");
    fprintf(stderr, "drain       : LAST=%s EOS=%s\n",
            got_last ? "yes" : "no", got_eos ? "yes" : "no");
    fprintf(stderr, "RESULT: %s\n",
            (cap_count > 0 && ts_ok) ? "PASS" : "CHECK");
    return (cap_count > 0 && ts_ok) ? 0 : 2;
}
