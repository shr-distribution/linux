/*
 * End-to-end decode -> HW de-tile validation:
 *   /dev/video6  (qcom-vidc-dec)  H.264 -> NV12MT (tiled, firmware-mandatory)
 *   /dev/video10 (qcom-rotator)   NV12MT -> NV12 (linear) via HW de-tile
 *
 * For every decoded tiled frame we memcpy it into the rotator OUTPUT buffer,
 * run one rotator job, and collect the linear NV12 from the rotator CAPTURE
 * buffer.  Proves the real decode->rotator linear path (not standalone).
 *
 * Output: <out> = concatenated linear NV12 frames.
 *         <out>.first = first linear frame only (for PNG visualisation).
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
#define CAP_BUFS 32
#define die(m) do { perror(m); exit(1); } while (0)

static void *out_mmap[OUT_BUFS]; static size_t out_size[OUT_BUFS]; static int out_avail[OUT_BUFS];
static void *cap_mmap[CAP_BUFS]; static size_t cap_size[CAP_BUFS];

/* ---- rotator (single-plane) state ---- */
static int   rfd;
static void *rot_om, *rot_cm;
static unsigned rot_osz, rot_csz, rot_clin_w, rot_clin_h, rot_bpl;

static void rot_setup(unsigned w, unsigned h)
{
    rfd = open("/dev/video10", O_RDWR);
    if (rfd < 0) die("open rotator");
    struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT };
    f.fmt.pix.width = w; f.fmt.pix.height = h;
    f.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12MT;
    if (ioctl(rfd, VIDIOC_S_FMT, &f)) die("rot S_FMT OUT");
    rot_osz = f.fmt.pix.sizeimage;
    fprintf(stderr, "ROT OUT  NV12MT %ux%u sizeimage=%u\n",
            f.fmt.pix.width, f.fmt.pix.height, rot_osz);

    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    fc.fmt.pix.width = w; fc.fmt.pix.height = h;
    fc.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
    if (ioctl(rfd, VIDIOC_S_FMT, &fc)) die("rot S_FMT CAP");
    rot_csz = fc.fmt.pix.sizeimage; rot_bpl = fc.fmt.pix.bytesperline;
    rot_clin_w = fc.fmt.pix.width; rot_clin_h = fc.fmt.pix.height;
    fprintf(stderr, "ROT CAP  NV12   %ux%u sizeimage=%u bpl=%u\n",
            fc.fmt.pix.width, fc.fmt.pix.height, rot_csz, rot_bpl);

    struct v4l2_requestbuffers ro = { .count=1, .type=V4L2_BUF_TYPE_VIDEO_OUTPUT,  .memory=V4L2_MEMORY_MMAP };
    if (ioctl(rfd, VIDIOC_REQBUFS, &ro)) die("rot REQBUFS OUT");
    struct v4l2_requestbuffers rc = { .count=1, .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP };
    if (ioctl(rfd, VIDIOC_REQBUFS, &rc)) die("rot REQBUFS CAP");

    struct v4l2_buffer b = { .type=V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory=V4L2_MEMORY_MMAP, .index=0 };
    ioctl(rfd, VIDIOC_QUERYBUF, &b);
    rot_om = mmap(0, b.length, PROT_READ|PROT_WRITE, MAP_SHARED, rfd, b.m.offset);
    if (rot_om == MAP_FAILED) die("rot mmap OUT");
    struct v4l2_buffer bc = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP, .index=0 };
    ioctl(rfd, VIDIOC_QUERYBUF, &bc);
    rot_cm = mmap(0, bc.length, PROT_READ|PROT_WRITE, MAP_SHARED, rfd, bc.m.offset);
    if (rot_cm == MAP_FAILED) die("rot mmap CAP");

    int t = V4L2_BUF_TYPE_VIDEO_OUTPUT;  if (ioctl(rfd, VIDIOC_STREAMON, &t)) die("rot STREAMON OUT");
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE;     if (ioctl(rfd, VIDIOC_STREAMON, &t)) die("rot STREAMON CAP");
}

/* Run one de-tile job; returns linear bytes produced (0 on failure). */
static unsigned rot_detile(const void *tiled, unsigned tlen)
{
    unsigned cp = tlen < rot_osz ? tlen : rot_osz;
    memcpy(rot_om, tiled, cp);
    struct v4l2_buffer q  = { .type=V4L2_BUF_TYPE_VIDEO_OUTPUT,  .memory=V4L2_MEMORY_MMAP, .index=0, .bytesused=rot_osz };
    if (ioctl(rfd, VIDIOC_QBUF, &q))  { perror("rot QBUF OUT");  return 0; }
    struct v4l2_buffer qc = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP, .index=0 };
    if (ioctl(rfd, VIDIOC_QBUF, &qc)) { perror("rot QBUF CAP");  return 0; }
    struct pollfd pfd = { .fd=rfd, .events=POLLIN };
    if (poll(&pfd, 1, 3000) <= 0) { fprintf(stderr, "rot poll timeout\n"); return 0; }
    struct v4l2_buffer d = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=V4L2_MEMORY_MMAP };
    if (ioctl(rfd, VIDIOC_DQBUF, &d)) { perror("rot DQBUF CAP"); return 0; }
    struct v4l2_buffer dout = { .type=V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory=V4L2_MEMORY_MMAP };
    ioctl(rfd, VIDIOC_DQBUF, &dout);  /* release OUTPUT buffer for reuse */
    return rot_csz;
}

int main(int argc, char **argv)
{
    const char *in_path  = argc > 1 ? argv[1] : "/tmp/test320.h264";
    const char *out_path = argc > 2 ? argv[2] : "/tmp/e2e.nv12";

    int fd = open("/dev/video6", O_RDWR | O_NONBLOCK);
    if (fd < 0) die("open dec");

    struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
    f.fmt.pix_mp.width = WIDTH; f.fmt.pix_mp.height = HEIGHT;
    f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    f.fmt.pix_mp.num_planes = 1;
    f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
    if (ioctl(fd, VIDIOC_S_FMT, &f)) die("dec S_FMT OUTPUT");

    /* decoder CAPTURE is always tiled NV12MT (firmware-mandatory) */
    struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    fc.fmt.pix_mp.width = WIDTH; fc.fmt.pix_mp.height = HEIGHT;
    fc.fmt.pix_mp.pixelformat = v4l2_fourcc('T','M','1','2');
    fc.fmt.pix_mp.num_planes = 1;
    if (ioctl(fd, VIDIOC_S_FMT, &fc)) die("dec S_FMT CAPTURE");

    struct v4l2_event_subscription es = { .type = V4L2_EVENT_SOURCE_CHANGE };
    ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &es);

    struct v4l2_requestbuffers rb_o = { .count=OUT_BUFS, .type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory=V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb_o)) die("dec REQBUFS OUT");
    for (unsigned i = 0; i < rb_o.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = { .type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory=V4L2_MEMORY_MMAP, .index=i, .m.planes=&pl, .length=1 };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b)) die("dec QUERYBUF OUT");
        out_size[i] = pl.length;
        out_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pl.m.mem_offset);
        if (out_mmap[i] == MAP_FAILED) die("dec mmap OUT");
        out_avail[i] = 1;
    }

    int in_fd = open(in_path, O_RDONLY); if (in_fd < 0) die("open input");
    off_t in_total = lseek(in_fd, 0, SEEK_END); lseek(in_fd, 0, SEEK_SET);
    unsigned char *in_data = malloc(in_total);
    if (read(in_fd, in_data, in_total) != in_total) die("read input");
    close(in_fd);
    fprintf(stderr, "Read %lld bytes from %s\n", (long long)in_total, in_path);

    size_t payload = in_total > (off_t)out_size[0] ? out_size[0] : (size_t)in_total;
    memcpy(out_mmap[0], in_data, payload);
    struct v4l2_plane pl0 = { .bytesused = payload };
    struct v4l2_buffer b0 = { .type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory=V4L2_MEMORY_MMAP, .index=0, .m.planes=&pl0, .length=1 };
    if (ioctl(fd, VIDIOC_QBUF, &b0)) die("dec QBUF OUT[0]");
    out_avail[0] = 0;
    int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &t)) die("dec STREAMON OUT");

    int got_src = 0, loops = 0;
    while (!got_src && loops++ < 30) {
        struct pollfd pfd = { .fd=fd, .events=POLLPRI|POLLOUT };
        if (poll(&pfd, 1, 1000) < 0) die("poll");
        if (pfd.revents & POLLPRI) {
            struct v4l2_event ev;
            while (ioctl(fd, VIDIOC_DQEVENT, &ev) == 0)
                if (ev.type == V4L2_EVENT_SOURCE_CHANGE) got_src = 1;
        }
        if (pfd.revents & POLLOUT) {
            struct v4l2_plane dp = {0};
            struct v4l2_buffer db = { .type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory=V4L2_MEMORY_MMAP, .m.planes=&dp, .length=1 };
            if (ioctl(fd, VIDIOC_DQBUF, &db) == 0) out_avail[db.index] = 1;
        }
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fc) == 0)
        fprintf(stderr, "DEC CAP after SRC_CHG: %ux%u size=%u\n",
                fc.fmt.pix_mp.width, fc.fmt.pix_mp.height, fc.fmt.pix_mp.plane_fmt[0].sizeimage);
    unsigned dw = fc.fmt.pix_mp.width, dh = fc.fmt.pix_mp.height;

    struct v4l2_requestbuffers rb_c = { .count=CAP_BUFS, .type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, .memory=V4L2_MEMORY_MMAP };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb_c)) die("dec REQBUFS CAP");
    for (unsigned i = 0; i < rb_c.count; i++) {
        struct v4l2_plane pl = {0};
        struct v4l2_buffer b = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, .memory=V4L2_MEMORY_MMAP, .index=i, .m.planes=&pl, .length=1 };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b)) die("dec QUERYBUF CAP");
        cap_size[i] = pl.length;
        cap_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pl.m.mem_offset);
        if (cap_mmap[i] == MAP_FAILED) die("dec mmap CAP");
        struct v4l2_plane qpl = {0};
        struct v4l2_buffer qb = { .type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, .memory=V4L2_MEMORY_MMAP, .index=i, .m.planes=&qpl, .length=1 };
        if (ioctl(fd, VIDIOC_QBUF, &qb)) die("dec QBUF CAP");
    }
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &t)) die("dec STREAMON CAP");

    /* now bring up the rotator at the decoder's real geometry */
    rot_setup(dw, dh);
    if (rot_osz != fc.fmt.pix_mp.plane_fmt[0].sizeimage)
        fprintf(stderr, "WARN: dec tiled size=%u != rot OUT size=%u (tiling mismatch?)\n",
                fc.fmt.pix_mp.plane_fmt[0].sizeimage, rot_osz);

    /* AU table */
    size_t *au_off = malloc(1024 * sizeof(*au_off)); size_t au_n=0, au_cap=1024;
    { int in_au=0;
      for (off_t i=0; i+4<in_total; i++) {
        int sc=0; size_t off_nal=0;
        if (!in_data[i]&&!in_data[i+1]&&!in_data[i+2]&&in_data[i+3]==1){sc=4;off_nal=i+4;}
        else if (!in_data[i]&&!in_data[i+1]&&in_data[i+2]==1){sc=3;off_nal=i+3;}
        if (!sc) continue;
        int nt=in_data[off_nal]&0x1f, vcl=(nt>=1&&nt<=5);
        if (!in_au){ if(au_n==au_cap){au_cap*=2;au_off=realloc(au_off,au_cap*sizeof(*au_off));} au_off[au_n++]=i; in_au=1; }
        if (vcl) in_au=0; i+=sc-1;
      } }
    if (au_n==0){ au_off[0]=0; au_n=1; }
    fprintf(stderr, "Built %zu AUs\n", au_n);

    size_t next_au = 0;
    for (unsigned i=0; i<rb_o.count && next_au<au_n; i++) {
        if (!out_avail[i]) continue;
        size_t s=au_off[next_au], e=(next_au+1<au_n)?au_off[next_au+1]:(size_t)in_total, len=e-s;
        if (len>out_size[i]) len=out_size[i];
        memcpy(out_mmap[i], in_data+s, len);
        struct v4l2_plane plp={.bytesused=len};
        struct v4l2_buffer bp={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,.memory=V4L2_MEMORY_MMAP,.index=i,.m.planes=&plp,.length=1};
        bp.timestamp.tv_usec=(next_au+1)*1000;
        if (ioctl(fd, VIDIOC_QBUF, &bp)==0){ out_avail[i]=0; next_au++; }
    }

    int of = open(out_path, O_WRONLY|O_CREAT|O_TRUNC, 0644); if (of<0) die("open output");
    char firstp[256]; snprintf(firstp,sizeof firstp,"%s.first",out_path);
    int off1 = open(firstp, O_WRONLY|O_CREAT|O_TRUNC, 0644);

    size_t dec_frames=0, lin_frames=0, lin_nonzero=0, stop_sent=0, idle=0;
    while (idle < 6) {
        struct pollfd pfd={.fd=fd,.events=POLLIN|POLLOUT|POLLPRI};
        int pr=poll(&pfd,1,1000);
        if (pr<0) die("poll");
        if (pr==0){ idle++; continue; }
        idle=0;
        if (pfd.revents & POLLIN) {
            struct v4l2_plane pl={0};
            struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,.memory=V4L2_MEMORY_MMAP,.m.planes=&pl,.length=1};
            if (ioctl(fd, VIDIOC_DQBUF, &b)==0) {
                unsigned pay=pl.bytesused;
                int last=!!(b.flags & V4L2_BUF_FLAG_LAST);
                if (pay>0) {
                    dec_frames++;
                    /* ---- feed tiled frame through rotator de-tile ---- */
                    unsigned lin = rot_detile(cap_mmap[b.index], pay);
                    if (lin) {
                        /* check the linear frame isn't all-zero garbage */
                        const unsigned char *p=rot_cm; unsigned nz=0;
                        for (unsigned k=0;k<lin && nz<16;k+=997) if(p[k]) nz++;
                        if (nz) lin_nonzero++;
                        write(of, rot_cm, lin);
                        if (lin_frames==0 && off1>=0){ write(off1, rot_cm, lin); }
                        lin_frames++;
                        fprintf(stderr, "frame %zu: dec tiled=%u -> rot linear=%u (nz=%u)\n",
                                dec_frames, pay, lin, nz);
                    }
                }
                if (!last) {
                    struct v4l2_plane rp={0};
                    struct v4l2_buffer rq={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,.memory=V4L2_MEMORY_MMAP,.index=b.index,.m.planes=&rp,.length=1};
                    ioctl(fd, VIDIOC_QBUF, &rq);
                } else idle=6;
            }
        }
        if (pfd.revents & POLLOUT) {
            struct v4l2_plane pl={0};
            struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,.memory=V4L2_MEMORY_MMAP,.m.planes=&pl,.length=1};
            if (ioctl(fd, VIDIOC_DQBUF, &b)==0) {
                out_avail[b.index]=1;
                if (next_au<au_n) {
                    size_t s=au_off[next_au], e=(next_au+1<au_n)?au_off[next_au+1]:(size_t)in_total, len=e-s;
                    if (len>out_size[b.index]) len=out_size[b.index];
                    memcpy(out_mmap[b.index], in_data+s, len);
                    struct v4l2_plane qp={.bytesused=len};
                    struct v4l2_buffer qb={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,.memory=V4L2_MEMORY_MMAP,.index=b.index,.m.planes=&qp,.length=1};
                    qb.timestamp.tv_usec=(next_au+1)*1000;
                    if (ioctl(fd, VIDIOC_QBUF, &qb)==0){ out_avail[b.index]=0; next_au++; }
                }
                if (next_au>=au_n && !stop_sent) {
                    struct v4l2_decoder_cmd dc={.cmd=V4L2_DEC_CMD_STOP};
                    ioctl(fd, VIDIOC_DECODER_CMD, &dc); stop_sent=1;
                }
            }
        }
    }

    close(of); if(off1>=0) close(off1);
    fprintf(stderr, "\n==== E2E DECODE->ROTATOR DE-TILE ====\n");
    fprintf(stderr, "decoded tiled frames : %zu\n", dec_frames);
    fprintf(stderr, "de-tiled linear frames: %zu (size %u each, %ux%u bpl=%u)\n",
            lin_frames, rot_csz, rot_clin_w, rot_clin_h, rot_bpl);
    fprintf(stderr, "non-zero linear frames: %zu\n", lin_nonzero);
    fprintf(stderr, "RESULT: %s\n",
            (lin_frames>0 && lin_frames==dec_frames && lin_nonzero==lin_frames) ? "PASS" : "CHECK");
    return (lin_frames>0 && lin_nonzero==lin_frames) ? 0 : 2;
}
