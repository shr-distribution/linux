/*
 * vidc_play3 — VIDC tile-NV12 → rotator HW detile → linear NV12 in DDR →
 * MDP4 primary plane via DRM atomic.
 *
 * Pipeline:
 *  /dev/video6  (VIDC dec)  CAPTURE: tile NV12 in SMI (dma-buf exported)
 *      → /dev/video10 (rotator) OUTPUT: V4L2_MEMORY_DMABUF from VIDC fd
 *      → rotator CAPTURE: V4L2_MEMORY_MMAP (allocator = system CMA → DDR)
 *      → exported as dma-buf, imported into DRM as DRM_FORMAT_NV12 linear
 *      → atomic commit on the MDP4 primary plane, no Mesa, no GPU
 *
 * For 1024x768 source on the 1024x768 LVDS panel: MDP4 reads linear NV12
 * 1:1, no FIR scaler engagement, no SMI fetch contention with VIDC writes.
 * Rotator at panel-native should sustain real-time (the slow case was
 * 1080p source where per-frame clock thrash bottlenecked at ~1.7 fps).
 *
 * Build (Yocto sysroot):
 *   arm-linux-gnueabihf-gcc -O2 -Wall --sysroot=$SYSROOT \
 *     -I$SYSROOT/usr/include/libdrm \
 *     reports/vidc-tests/vidc_play3.c -o /tmp/vidc_play3 -ldrm
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#define die(m) do { perror(m); exit(1); } while (0)
#define MAX_VIDC_CAP   10
#define MAX_ROT_BUFS    6
#define MAX_DEC_OUT     4

/* ---------------------------------------------------------------- DRM */

static int drm_fd;
static uint32_t crtc_id, conn_id, plane_id;
static drmModeModeInfo dpy_mode;
static drmModeCrtc *saved_crtc;
static uint32_t p_fb, p_crtc, p_sx, p_sy, p_sw, p_sh;
static uint32_t p_cx, p_cy, p_cw, p_ch, p_rot;

static void resolve_props(uint32_t plane)
{
	drmModeObjectProperties *props =
		drmModeObjectGetProperties(drm_fd, plane, DRM_MODE_OBJECT_PLANE);
	if (!props) die("plane props");
	for (uint32_t i = 0; i < props->count_props; i++) {
		drmModePropertyRes *pr = drmModeGetProperty(drm_fd,
							    props->props[i]);
		if (!pr) continue;
		#define MAP(s, v) if (!strcmp(pr->name, s)) v = pr->prop_id
		MAP("FB_ID",    p_fb);
		MAP("CRTC_ID",  p_crtc);
		MAP("SRC_X",    p_sx);
		MAP("SRC_Y",    p_sy);
		MAP("SRC_W",    p_sw);
		MAP("SRC_H",    p_sh);
		MAP("CRTC_X",   p_cx);
		MAP("CRTC_Y",   p_cy);
		MAP("CRTC_W",   p_cw);
		MAP("CRTC_H",   p_ch);
		MAP("rotation", p_rot);
		#undef MAP
		drmModeFreeProperty(pr);
	}
	drmModeFreeObjectProperties(props);
}

static void drm_init(void)
{
	drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
	if (drm_fd < 0) die("open card0");
	drmSetClientCap(drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
	drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1);

	drmModeRes *res = drmModeGetResources(drm_fd);
	drmModeConnector *conn = NULL;
	for (int i = 0; i < res->count_connectors; i++) {
		conn = drmModeGetConnector(drm_fd, res->connectors[i]);
		if (conn && conn->connection == DRM_MODE_CONNECTED &&
		    conn->count_modes > 0)
			break;
		drmModeFreeConnector(conn);
		conn = NULL;
	}
	if (!conn) die("no connector");
	conn_id  = conn->connector_id;
	dpy_mode = conn->modes[0];
	drmModeEncoder *enc = drmModeGetEncoder(drm_fd, conn->encoder_id);
	crtc_id = enc->crtc_id;
	drmModeFreeEncoder(enc);
	int crtc_idx = 0;
	for (int i = 0; i < res->count_crtcs; i++)
		if (res->crtcs[i] == crtc_id) { crtc_idx = i; break; }
	saved_crtc = drmModeGetCrtc(drm_fd, crtc_id);

	/* find a plane that accepts NV12 on this CRTC */
	drmModePlaneRes *pr = drmModeGetPlaneResources(drm_fd);
	for (uint32_t i = 0; i < pr->count_planes; i++) {
		drmModePlane *pl = drmModeGetPlane(drm_fd, pr->planes[i]);
		if (pl && (pl->possible_crtcs & (1u << crtc_idx))) {
			for (uint32_t f = 0; f < pl->count_formats; f++)
				if (pl->formats[f] == DRM_FORMAT_NV12) {
					plane_id = pl->plane_id;
					break;
				}
			if (plane_id) { drmModeFreePlane(pl); break; }
		}
		drmModeFreePlane(pl);
	}
	drmModeFreePlaneResources(pr);
	drmModeFreeConnector(conn);
	drmModeFreeResources(res);
	if (!plane_id) die("no NV12 plane");
	resolve_props(plane_id);
	fprintf(stderr, "DRM: %dx%d@%d crtc=%u plane=%u\n",
		dpy_mode.hdisplay, dpy_mode.vdisplay, dpy_mode.vrefresh,
		crtc_id, plane_id);

	/* Set 180° rotation for upside-down LVDS panel. */
	if (p_rot)
		drmModeObjectSetProperty(drm_fd, plane_id,
		    DRM_MODE_OBJECT_PLANE, p_rot, DRM_MODE_ROTATE_180);
}

static int flip_pending;
static void flip_cb(int fd, unsigned seq, unsigned s, unsigned us,
		    unsigned cid, void *data)
{ (void)fd;(void)seq;(void)s;(void)us;(void)cid;(void)data; flip_pending=0; }

static void drain_flip(int ms)
{
	if (!flip_pending) return;
	struct pollfd pfd = { .fd = drm_fd, .events = POLLIN };
	if (poll(&pfd, 1, ms) <= 0) return;
	drmEventContext ev = { .version = 3, .page_flip_handler2 = flip_cb };
	drmHandleEvent(drm_fd, &ev);
}

static void atomic_present(uint32_t fb_id, unsigned src_w, unsigned src_h)
{
	drmModeAtomicReq *req = drmModeAtomicAlloc();
	#define ADD(p, v) drmModeAtomicAddProperty(req, plane_id, p, v)
	ADD(p_fb,   fb_id);
	ADD(p_crtc, crtc_id);
	ADD(p_sx, 0); ADD(p_sy, 0);
	ADD(p_sw, (uint64_t)src_w << 16);
	ADD(p_sh, (uint64_t)src_h << 16);
	ADD(p_cx, 0); ADD(p_cy, 0);
	ADD(p_cw, dpy_mode.hdisplay);
	ADD(p_ch, dpy_mode.vdisplay);
	if (p_rot) ADD(p_rot, DRM_MODE_ROTATE_180);
	#undef ADD
	uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT;
	int ret = drmModeAtomicCommit(drm_fd, req, flags, NULL);
	if (ret == -EBUSY) {
		drain_flip(50);
		ret = drmModeAtomicCommit(drm_fd, req, flags, NULL);
	}
	if (ret) fprintf(stderr, "AtomicCommit: %s\n", strerror(-ret));
	else     flip_pending = 1;
	drmModeAtomicFree(req);
}

/* ---------------------------------------------------------------- V4L2 */

static int vfd, rfd;
static unsigned coded_w, coded_h;
static unsigned rot_cap_bpl, rot_cap_size;

/* VIDC OUTPUT (compressed bitstream) */
static void *dec_out_mmap[MAX_DEC_OUT]; static size_t dec_out_size[MAX_DEC_OUT];
static int   dec_out_avail[MAX_DEC_OUT]; static unsigned dec_out_bufs;

/* VIDC CAPTURE (tile NV12 in SMI), exported dma-bufs fed to rotator OUTPUT */
struct vidc_slot {
	int  dmabuf_fd;
	int  in_rotator;  /* 1 = currently submitted to rotator */
};
static struct vidc_slot vidc_cap[MAX_VIDC_CAP];
static unsigned vidc_cap_count;

/* Rotator CAPTURE buffers (linear NV12 in DDR), exported as DRM FBs */
struct rot_slot {
	int      dmabuf_fd;
	uint32_t drm_handle;
	uint32_t fb_id;
};
static struct rot_slot rot_cap[MAX_ROT_BUFS];
static unsigned rot_bufs;

/* Tracks which rotator OUTPUT slot is holding which VIDC CAPTURE slot. */
static int rot_out_to_vidc[MAX_ROT_BUFS];

static void vidc_open(void)
{
	vfd = open("/dev/video6", O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (vfd < 0) die("open dec");
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	f.fmt.pix_mp.width = 320; f.fmt.pix_mp.height = 240;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f.fmt.pix_mp.num_planes = 1;
	f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
	if (ioctl(vfd, VIDIOC_S_FMT, &f)) die("dec S_FMT OUT");
	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	fc.fmt.pix_mp.width = 320; fc.fmt.pix_mp.height = 240;
	fc.fmt.pix_mp.pixelformat = v4l2_fourcc('T','M','1','2');
	fc.fmt.pix_mp.num_planes = 1;
	if (ioctl(vfd, VIDIOC_S_FMT, &fc)) die("dec S_FMT CAP");
	struct v4l2_event_subscription es = { .type = V4L2_EVENT_SOURCE_CHANGE };
	ioctl(vfd, VIDIOC_SUBSCRIBE_EVENT, &es);
	struct v4l2_requestbuffers rb = { .count = MAX_DEC_OUT,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(vfd, VIDIOC_REQBUFS, &rb)) die("dec REQBUFS OUT");
	dec_out_bufs = rb.count;
	for (unsigned i = 0; i < dec_out_bufs; i++) {
		struct v4l2_plane pl = {0};
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		if (ioctl(vfd, VIDIOC_QUERYBUF, &b)) die("dec QUERYBUF OUT");
		dec_out_size[i] = pl.length;
		dec_out_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE,
				       MAP_SHARED, vfd, pl.m.mem_offset);
		dec_out_avail[i] = 1;
	}
}

static void rot_setup(unsigned w, unsigned h)
{
	rfd = open("/dev/video10", O_RDWR | O_CLOEXEC);
	if (rfd < 0) die("open rotator");

	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT };
	f.fmt.pix.width = w; f.fmt.pix.height = h;
	f.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12MT;
	if (ioctl(rfd, VIDIOC_S_FMT, &f)) die("rot S_FMT OUT");

	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
	fc.fmt.pix.width = w; fc.fmt.pix.height = h;
	fc.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
	if (ioctl(rfd, VIDIOC_S_FMT, &fc)) die("rot S_FMT CAP");
	rot_cap_size = fc.fmt.pix.sizeimage;
	rot_cap_bpl  = fc.fmt.pix.bytesperline;
	fprintf(stderr, "ROT: OUT NV12MT %ux%u; CAP NV12 %ux%u sz=%u bpl=%u\n",
		w, h, fc.fmt.pix.width, fc.fmt.pix.height,
		rot_cap_size, rot_cap_bpl);

	/* OUTPUT side: DMABUF-imported from VIDC CAPTURE. */
	struct v4l2_requestbuffers ro = { .count = MAX_ROT_BUFS,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
		.memory = V4L2_MEMORY_DMABUF };
	if (ioctl(rfd, VIDIOC_REQBUFS, &ro)) die("rot REQBUFS OUT (DMABUF)");
	rot_bufs = ro.count;

	/* CAPTURE side: MMAP, allocated from rotator alloc dev (system CMA -> DDR). */
	struct v4l2_requestbuffers rc = { .count = MAX_ROT_BUFS,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(rfd, VIDIOC_REQBUFS, &rc)) die("rot REQBUFS CAP");
	if (rc.count < rot_bufs) rot_bufs = rc.count;

	for (unsigned i = 0; i < rot_bufs; i++) {
		struct v4l2_exportbuffer eb = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.index = i, .flags = O_CLOEXEC };
		if (ioctl(rfd, VIDIOC_EXPBUF, &eb)) die("rot EXPBUF CAP");
		rot_cap[i].dmabuf_fd = eb.fd;

		/* Import into DRM as PRIME, build linear NV12 framebuffer. */
		if (drmPrimeFDToHandle(drm_fd, eb.fd, &rot_cap[i].drm_handle))
			die("rot drmPrimeFDToHandle");
		uint32_t handles[4] = { rot_cap[i].drm_handle, rot_cap[i].drm_handle, 0, 0 };
		uint32_t strides[4] = { rot_cap_bpl, rot_cap_bpl, 0, 0 };
		/* Linear NV12: chroma plane starts at y_stride * h. */
		uint32_t offsets[4] = { 0, rot_cap_bpl * h, 0, 0 };
		if (drmModeAddFB2(drm_fd, w, h, DRM_FORMAT_NV12,
				  handles, strides, offsets, &rot_cap[i].fb_id, 0))
			die("rot AddFB2 NV12");
		fprintf(stderr, "  rot slot %u: dma_fd=%d gem=%u fb=%u\n",
			i, eb.fd, rot_cap[i].drm_handle, rot_cap[i].fb_id);

		rot_out_to_vidc[i] = -1;
	}
}

static void vidc_setup_capture_after_src_change(void)
{
	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	if (ioctl(vfd, VIDIOC_G_FMT, &fc)) die("dec G_FMT CAP");
	coded_w = fc.fmt.pix_mp.width;
	coded_h = fc.fmt.pix_mp.height;
	fprintf(stderr, "VIDC CAP %ux%u sz=%u\n",
		coded_w, coded_h, fc.fmt.pix_mp.plane_fmt[0].sizeimage);

	struct v4l2_requestbuffers rb = { .count = MAX_VIDC_CAP,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(vfd, VIDIOC_REQBUFS, &rb)) die("dec REQBUFS CAP");
	vidc_cap_count = rb.count;
	if (vidc_cap_count > MAX_VIDC_CAP) vidc_cap_count = MAX_VIDC_CAP;

	for (unsigned i = 0; i < vidc_cap_count; i++) {
		struct v4l2_exportbuffer eb = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.index = i, .plane = 0, .flags = O_CLOEXEC };
		if (ioctl(vfd, VIDIOC_EXPBUF, &eb)) die("dec EXPBUF");
		vidc_cap[i].dmabuf_fd = eb.fd;
		vidc_cap[i].in_rotator = 0;

		struct v4l2_plane qp = {0};
		struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &qp, .length = 1 };
		if (ioctl(vfd, VIDIOC_QBUF, &q)) die("dec QBUF CAP");
	}
	int t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("dec STREAMON CAP");

	rot_setup(coded_w, coded_h);

	t = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	if (ioctl(rfd, VIDIOC_STREAMON, &t)) die("rot STREAMON OUT");
	t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(rfd, VIDIOC_STREAMON, &t)) die("rot STREAMON CAP");

	/* Prime all rotator CAPTURE slots (waiting for output). */
	for (unsigned i = 0; i < rot_bufs; i++) {
		struct v4l2_buffer qc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory = V4L2_MEMORY_MMAP, .index = i };
		ioctl(rfd, VIDIOC_QBUF, &qc);
	}
}

/* Find a free rotator OUTPUT slot and submit the given VIDC CAPTURE
 * dma-buf into it. Returns the rotator OUT slot index, or -1 if none free. */
static int rot_submit_vidc(int vidc_idx)
{
	for (unsigned i = 0; i < rot_bufs; i++) {
		if (rot_out_to_vidc[i] >= 0) continue;
		struct v4l2_buffer b = {
			.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT,
			.memory = V4L2_MEMORY_DMABUF,
			.index  = i,
			.m.fd   = vidc_cap[vidc_idx].dmabuf_fd,
		};
		if (ioctl(rfd, VIDIOC_QBUF, &b))
			return -1;
		rot_out_to_vidc[i] = vidc_idx;
		vidc_cap[vidc_idx].in_rotator = 1;
		return i;
	}
	return -1;
}

/* -------------------------------------------------- AU walker (same as vidc_play) */

static size_t *au_off; static size_t au_n;
static unsigned char *in_data; static off_t in_total;

static void build_au_table(const char *path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) die("open input");
	in_total = lseek(fd, 0, SEEK_END); lseek(fd, 0, SEEK_SET);
	in_data = malloc(in_total);
	if (read(fd, in_data, in_total) != in_total) die("read input");
	close(fd);
	size_t cap = 2048;
	au_off = malloc(cap * sizeof(*au_off));
	int in_au = 0;
	for (off_t i = 0; i + 4 < in_total; i++) {
		int sc = 0; size_t off_nal = 0;
		if (!in_data[i] && !in_data[i+1] && !in_data[i+2] &&
		    in_data[i+3] == 1) { sc = 4; off_nal = i + 4; }
		else if (!in_data[i] && !in_data[i+1] && in_data[i+2] == 1)
			{ sc = 3; off_nal = i + 3; }
		if (!sc) continue;
		int nt = in_data[off_nal] & 0x1f, vcl = (nt >= 1 && nt <= 5);
		if (!in_au) {
			if (au_n == cap) { cap *= 2;
				au_off = realloc(au_off, cap * sizeof(*au_off)); }
			au_off[au_n++] = i;
			in_au = 1;
		}
		if (vcl) in_au = 0;
		i += sc - 1;
	}
	if (!au_n) { au_off[0] = 0; au_n = 1; }
	fprintf(stderr, "Built %zu AUs\n", au_n);
}

/* -------------------------------------------------- main loop */

int main(int argc, char **argv)
{
	const char *in_path = argc > 1 ? argv[1] : "/tmp/in.264";

	drm_init();
	vidc_open();
	build_au_table(in_path);

	/* Submit first AU + STREAMON VIDC OUTPUT, wait for SOURCE_CHANGE. */
	size_t next_au = 0;
	{
		size_t s = au_off[0];
		size_t e = au_n > 1 ? au_off[1] : (size_t)in_total;
		size_t len = e - s; if (len > dec_out_size[0]) len = dec_out_size[0];
		memcpy(dec_out_mmap[0], in_data + s, len);
		struct v4l2_plane pl = { .bytesused = len };
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = 0,
			.m.planes = &pl, .length = 1 };
		ioctl(vfd, VIDIOC_QBUF, &b);
		dec_out_avail[0] = 0; next_au++;
	}
	int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("dec STREAMON OUT");

	int got_src = 0, loops = 0;
	while (!got_src && loops++ < 30) {
		struct pollfd pfd = { .fd = vfd, .events = POLLPRI|POLLOUT };
		poll(&pfd, 1, 1000);
		if (pfd.revents & POLLPRI) {
			struct v4l2_event ev;
			while (ioctl(vfd, VIDIOC_DQEVENT, &ev) == 0)
				if (ev.type == V4L2_EVENT_SOURCE_CHANGE) got_src = 1;
		}
		if (pfd.revents & POLLOUT) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0)
				dec_out_avail[b.index] = 1;
		}
	}
	if (!got_src) die("no SOURCE_CHANGE");
	vidc_setup_capture_after_src_change();

	/* Bootstrap: pre-queue remaining VIDC OUTPUT slots. */
	for (unsigned i = 0; i < dec_out_bufs && next_au < au_n; i++) {
		if (!dec_out_avail[i]) continue;
		size_t s = au_off[next_au];
		size_t e = (next_au + 1 < au_n) ? au_off[next_au+1] : (size_t)in_total;
		size_t len = e - s; if (len > dec_out_size[i]) len = dec_out_size[i];
		memcpy(dec_out_mmap[i], in_data + s, len);
		struct v4l2_plane pl = { .bytesused = len };
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		if (!ioctl(vfd, VIDIOC_QBUF, &b)) {
			dec_out_avail[i] = 0; next_au++;
		}
	}

	/* Double-buffered hold: keep one slot "displayed" and one "queued"
	 * so the rotator CAPTURE buffer being scanned by MDP4 isn't re-fed
	 * to the rotator. */
	int displayed = -1, queued = -1;
	struct timespec t0, tlast;
	clock_gettime(CLOCK_MONOTONIC, &t0); tlast = t0;
	size_t frames = 0, frames_last = 0;
	unsigned idle = 0;

	while (idle < 8) {
		struct pollfd pfds[2] = {
			{ .fd = vfd, .events = POLLIN | POLLOUT | POLLPRI },
			{ .fd = rfd, .events = POLLIN | POLLOUT },
		};
		int pr = poll(pfds, 2, 500);
		if (pr < 0) die("poll");
		if (pr == 0) { idle++; continue; }
		idle = 0;

		/* VIDC produced a decoded tile-NV12 frame: forward to rotator. */
		if (pfds[0].revents & POLLIN) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0 && pl.bytesused) {
				if (rot_submit_vidc(b.index) < 0) {
					/* Rotator full: drop this frame, requeue
					 * VIDC CAPTURE slot. */
					struct v4l2_plane qp = {0};
					struct v4l2_buffer rq = {
						.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
						.memory = V4L2_MEMORY_MMAP,
						.index = b.index,
						.m.planes = &qp, .length = 1 };
					ioctl(vfd, VIDIOC_QBUF, &rq);
				}
			}
		}
		/* VIDC OUTPUT slot freed → feed next AU. */
		if (pfds[0].revents & POLLOUT) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0) {
				dec_out_avail[b.index] = 1;
				if (next_au < au_n) {
					size_t s = au_off[next_au];
					size_t e = (next_au + 1 < au_n)
						? au_off[next_au+1] : (size_t)in_total;
					size_t len = e - s;
					if (len > dec_out_size[b.index])
						len = dec_out_size[b.index];
					memcpy(dec_out_mmap[b.index],
					       in_data + s, len);
					struct v4l2_plane pl2 = { .bytesused = len };
					struct v4l2_buffer q = {
						.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
						.memory = V4L2_MEMORY_MMAP,
						.index = b.index,
						.m.planes = &pl2, .length = 1 };
					if (!ioctl(vfd, VIDIOC_QBUF, &q)) {
						dec_out_avail[b.index] = 0;
						next_au++;
					}
				}
			}
		}
		/* Rotator detiled a frame → atomic-commit it for scanout. */
		if (pfds[1].revents & POLLIN) {
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
				.memory = V4L2_MEMORY_MMAP };
			if (ioctl(rfd, VIDIOC_DQBUF, &b) == 0) {
				/* Wait for previous flip to latch, then recycle
				 * the now-no-longer-scanning slot back to the
				 * rotator CAPTURE queue. */
				if (queued >= 0) {
					int spins = 0;
					while (flip_pending && spins++ < 60)
						drain_flip(50);
					int prev = displayed;
					displayed = queued;
					queued = -1;
					if (prev >= 0) {
						struct v4l2_buffer qc = {
							.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
							.memory = V4L2_MEMORY_MMAP,
							.index = prev };
						ioctl(rfd, VIDIOC_QBUF, &qc);
					}
				}
				atomic_present(rot_cap[b.index].fb_id,
					       coded_w, coded_h);
				queued = b.index;
				frames++;

				struct timespec tn;
				clock_gettime(CLOCK_MONOTONIC, &tn);
				double dt = (tn.tv_sec - tlast.tv_sec) +
					    (tn.tv_nsec - tlast.tv_nsec)/1e9;
				if (dt >= 1.0) {
					double tdt = (tn.tv_sec - t0.tv_sec) +
						     (tn.tv_nsec - t0.tv_nsec)/1e9;
					fprintf(stderr,
					    "  [%.1fs] %.1f fps (avg %.1f)\n",
					    tdt, (frames - frames_last) / dt,
					    frames / tdt);
					tlast = tn; frames_last = frames;
				}
			}
		}
		/* Rotator finished with a VIDC CAPTURE slot → recycle to VIDC. */
		if (pfds[1].revents & POLLOUT) {
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
				.memory = V4L2_MEMORY_DMABUF };
			if (ioctl(rfd, VIDIOC_DQBUF, &b) == 0) {
				int vidc_idx = rot_out_to_vidc[b.index];
				rot_out_to_vidc[b.index] = -1;
				if (vidc_idx >= 0) {
					vidc_cap[vidc_idx].in_rotator = 0;
					struct v4l2_plane qp = {0};
					struct v4l2_buffer rq = {
						.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
						.memory = V4L2_MEMORY_MMAP,
						.index = vidc_idx,
						.m.planes = &qp, .length = 1 };
					ioctl(vfd, VIDIOC_QBUF, &rq);
				}
			}
		}
	}

	fprintf(stderr, "vidc_play3: %zu frames\n", frames);
	if (saved_crtc) {
		drmModeSetCrtc(drm_fd, saved_crtc->crtc_id, saved_crtc->buffer_id,
			       saved_crtc->x, saved_crtc->y, &conn_id, 1,
			       &saved_crtc->mode);
		drmModeFreeCrtc(saved_crtc);
	}
	close(drm_fd);
	close(rfd);
	close(vfd);
	return 0;
}
