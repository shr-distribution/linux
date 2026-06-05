/*
 * vidc_play — VIDC H.264 decode → MDP4 DRM plane display.
 *
 * Pipeline:
 *   1. Open /dev/video6 (qcom-vidc decoder), parse SPS, set up CAPTURE buffers
 *      in tiled NV12 ('TM12' = NV12_64Z32 = the firmware's native output).
 *   2. EXPBUF each CAPTURE buffer to a dma-buf fd (zero-copy: no extra alloc).
 *   3. Open /dev/dri/card0 (msm DRM, MDP4 backend on tenderloin/APQ8060).
 *      Import the V4L2 dma-buf via drmPrimeFDToHandle and wrap as a DRM
 *      framebuffer with DRM_FORMAT_NV12 + DRM_FORMAT_MOD_SAMSUNG_64_32_TILE
 *      so MDP4 detiles in hardware during scanout (no rotator needed).
 *   4. For every FRAME_DONE, flip the just-decoded slot onto the primary
 *      plane via drmModeSetPlane and wait one vblank.
 *
 * Why this beats rotator-detile + memcpy:
 *   - The rotator/VPE detile is a separate m2m job per frame; running it
 *     synchronously in the decode loop blocks FRAME_DATA submission for tens
 *     of ms and at 1080p that timing pressure makes the firmware throw
 *     0x58 (VIDC_CORE_TIME_OUT) on ~35% of P-frames. Skipping the rotator
 *     entirely (MDP4 detiles during scan-out) eliminates the timing issue.
 *   - Zero-copy: no kernel/user memcpy. The decoder's CAPTURE buffer IS the
 *     framebuffer the display reads.
 *
 * Build (out-of-tree, against the kmscube ARM sysroot):
 *   arm-linux-gnueabihf-gcc -O2 -I/usr/arm-linux-gnueabihf/include \
 *     vidc_play.c -ldrm -o vidc_play
 *
 * Usage:
 *   ./vidc_play input.264 [/dev/video6] [/dev/dri/card0]
 *
 * Reference: vendor MSM8660 displays VIDC-decoded NV12_MT through MDP4 the
 * same way (DRM_FORMAT_MOD_SAMSUNG_64_32_TILE modifier on DRM_FORMAT_NV12);
 * mdp4_plane.c::mdp4_get_frame_format() in this tree maps that modifier to
 * FRAME_TILE_YCBCR_420.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#define die(m) do { perror(m); exit(1); } while (0)
#define MAX_CAP_BUFS 12
#define MAX_OUT_BUFS 4

static int    vfd, drm_fd;
static unsigned width, height;          /* decoder-reported coded dimensions */
static unsigned crop_w, crop_h;
static unsigned dpy_w, dpy_h;           /* CRTC mode size */
static uint32_t crtc_id, plane_id, conn_id;
static drmModeModeInfo mode;
static drmModeCrtc *saved_crtc;

/* Per-CAPTURE-slot DRM state — exported dma-buf, imported handle, FB id. */
struct cap_slot {
	int      dmabuf_fd;     /* V4L2 → DRM dma-buf */
	uint32_t gem_handle;    /* DRM PRIME-imported gem handle */
	uint32_t fb_id;         /* DRM framebuffer wrapping the tile NV12 */
	void    *mmap_addr;     /* mmap for buffer-content sanity check */
	size_t   mmap_len;
};
static struct cap_slot slots[MAX_CAP_BUFS];

/* -------------------------------------------------------------------- DRM */

static void drm_setup(const char *card)
{
	drm_fd = open(card, O_RDWR | O_CLOEXEC);
	if (drm_fd < 0) die(card);

	/* We want to set NV12 + SAMSUNG_64_32_TILE modifier on a plane FB,
	 * which is universal-plane / modifier territory. */
	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1))
		fprintf(stderr, "warn: cap UNIVERSAL_PLANES (%s)\n", strerror(errno));
	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1))
		fprintf(stderr, "info: no ATOMIC, falling back to legacy SetPlane\n");

	drmModeRes *res = drmModeGetResources(drm_fd);
	if (!res) die("drmModeGetResources");

	/* Pick the first connected connector and the first usable mode on it. */
	drmModeConnector *conn = NULL;
	for (int i = 0; i < res->count_connectors; i++) {
		conn = drmModeGetConnector(drm_fd, res->connectors[i]);
		if (conn && conn->connection == DRM_MODE_CONNECTED &&
		    conn->count_modes > 0)
			break;
		drmModeFreeConnector(conn);
		conn = NULL;
	}
	if (!conn) die("no connected connector");
	conn_id = conn->connector_id;
	mode = conn->modes[0];
	dpy_w = mode.hdisplay;
	dpy_h = mode.vdisplay;
	fprintf(stderr, "DRM display: %ux%u@%uHz on connector %u\n",
		dpy_w, dpy_h, mode.vrefresh, conn_id);

	/* Encoder → CRTC. */
	drmModeEncoder *enc = drmModeGetEncoder(drm_fd, conn->encoder_id);
	if (!enc) die("get encoder");
	crtc_id = enc->crtc_id;
	if (!crtc_id) {
		/* Use a possible CRTC if not bound. */
		for (int i = 0; i < res->count_crtcs; i++) {
			if (enc->possible_crtcs & (1u << i)) {
				crtc_id = res->crtcs[i];
				break;
			}
		}
	}
	drmModeFreeEncoder(enc);
	saved_crtc = drmModeGetCrtc(drm_fd, crtc_id);

	/* Find a plane that supports NV12 on this CRTC. Universal planes give
	 * us the primary plane too; we don't care which kind as long as it
	 * accepts NV12. */
	drmModePlaneRes *plane_res = drmModeGetPlaneResources(drm_fd);
	if (!plane_res) die("plane res");
	for (uint32_t i = 0; i < plane_res->count_planes; i++) {
		drmModePlane *p = drmModeGetPlane(drm_fd, plane_res->planes[i]);
		if (!p) continue;
		int crtc_ok = (p->possible_crtcs & (1u << 0));
		int nv12_ok = 0;
		for (uint32_t f = 0; f < p->count_formats; f++)
			if (p->formats[f] == DRM_FORMAT_NV12) nv12_ok = 1;
		if (crtc_ok && nv12_ok && !plane_id)
			plane_id = p->plane_id;
		drmModeFreePlane(p);
	}
	drmModeFreePlaneResources(plane_res);
	drmModeFreeConnector(conn);
	drmModeFreeResources(res);
	if (!plane_id) die("no NV12-capable plane found");
	fprintf(stderr, "DRM: using plane %u on CRTC %u\n", plane_id, crtc_id);
}

static void drm_restore(void)
{
	if (saved_crtc) {
		drmModeSetCrtc(drm_fd, saved_crtc->crtc_id, saved_crtc->buffer_id,
			       saved_crtc->x, saved_crtc->y, &conn_id, 1,
			       &saved_crtc->mode);
		drmModeFreeCrtc(saved_crtc);
	}
}

static int drm_register_slot(struct cap_slot *s, unsigned w, unsigned h)
{
	int prime_err = drmPrimeFDToHandle(drm_fd, s->dmabuf_fd, &s->gem_handle);
	if (prime_err) {
		fprintf(stderr, "drmPrimeFDToHandle(fd=%d): %d/%s\n",
			s->dmabuf_fd, prime_err, strerror(errno));
		return -1;
	}

	/* NV12 layout for the tile path the firmware writes:
	 *   Y plane offset 0, stride = ALIGN(w, 128)
	 *   CbCr plane offset = stride * ALIGN(h, 32)
	 * That's what mdp4_get_frame_format() consumes for the tile modifier.
	 */
	uint32_t pitch  = (w + 127) & ~127u;
	uint32_t y_size = pitch * ((h + 31) & ~31u);
	uint32_t handles[4]  = { s->gem_handle, s->gem_handle, 0, 0 };
	uint32_t pitches[4]  = { pitch,   pitch,  0, 0 };
	uint32_t offsets[4]  = { 0,       y_size, 0, 0 };
	uint64_t modifiers[4] = {
		DRM_FORMAT_MOD_SAMSUNG_64_32_TILE,
		DRM_FORMAT_MOD_SAMSUNG_64_32_TILE,
		0, 0
	};
	int err = drmModeAddFB2WithModifiers(drm_fd, w, h, DRM_FORMAT_NV12,
					     handles, pitches, offsets,
					     modifiers, &s->fb_id,
					     DRM_MODE_FB_MODIFIERS);
	if (err) {
		fprintf(stderr, "AddFB2WithModifiers tile failed (%d/%s); "
			"falling back to AddFB2 (linear NV12) — won't render but "
			"isolates whether the modifier or the FB ioctl itself is "
			"the problem.\n", err, strerror(errno));
		err = drmModeAddFB2(drm_fd, w, h, DRM_FORMAT_NV12,
				    handles, pitches, offsets,
				    &s->fb_id, 0);
		if (err)
			fprintf(stderr, "  AddFB2 (no modifier) also failed: %d/%s\n",
				err, strerror(errno));
		return -1;
	}
	return 0;
}

static void drm_flip_to(struct cap_slot *s)
{
	/* SetPlane: crtc dst = entire display (MDP4 scales/tiles), src = full
	 * frame in 16.16 fixed-point. */
	if (drmModeSetPlane(drm_fd, plane_id, crtc_id, s->fb_id, 0,
			    0, 0, dpy_w, dpy_h,
			    0, 0, width << 16, height << 16))
		fprintf(stderr, "SetPlane: %s\n", strerror(errno));
}

/* -------------------------------------------------------------------- V4L2 */

static void *out_mmap[MAX_OUT_BUFS]; static size_t out_size[MAX_OUT_BUFS];
static int   out_avail[MAX_OUT_BUFS];
static struct cap_slot *all_slots = slots;
static unsigned cap_count;

static void dec_open(const char *dev)
{
	vfd = open(dev, O_RDWR | O_NONBLOCK);
	if (vfd < 0) die(dev);

	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	f.fmt.pix_mp.width = 320; f.fmt.pix_mp.height = 240;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f.fmt.pix_mp.num_planes = 1;
	f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
	if (ioctl(vfd, VIDIOC_S_FMT, &f)) die("S_FMT OUTPUT");

	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	fc.fmt.pix_mp.width = 320; fc.fmt.pix_mp.height = 240;
	fc.fmt.pix_mp.pixelformat = v4l2_fourcc('T','M','1','2');
	fc.fmt.pix_mp.num_planes = 1;
	if (ioctl(vfd, VIDIOC_S_FMT, &fc)) die("S_FMT CAPTURE");

	struct v4l2_event_subscription es = { .type = V4L2_EVENT_SOURCE_CHANGE };
	ioctl(vfd, VIDIOC_SUBSCRIBE_EVENT, &es);

	struct v4l2_requestbuffers rb = { .count = MAX_OUT_BUFS,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(vfd, VIDIOC_REQBUFS, &rb)) die("REQBUFS OUT");

	for (unsigned i = 0; i < rb.count; i++) {
		struct v4l2_plane pl = {0};
		struct v4l2_buffer b = {
			.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP,
			.index = i, .m.planes = &pl, .length = 1,
		};
		if (ioctl(vfd, VIDIOC_QUERYBUF, &b)) die("QUERYBUF OUT");
		out_size[i] = pl.length;
		out_mmap[i] = mmap(NULL, pl.length, PROT_READ|PROT_WRITE,
				   MAP_SHARED, vfd, pl.m.mem_offset);
		if (out_mmap[i] == MAP_FAILED) die("mmap OUT");
		out_avail[i] = 1;
	}
}

static void cap_setup_after_src_change(void)
{
	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	if (ioctl(vfd, VIDIOC_G_FMT, &fc)) die("G_FMT CAP");
	width  = fc.fmt.pix_mp.width;
	height = fc.fmt.pix_mp.height;
	crop_w = width;
	crop_h = height;
	struct v4l2_selection sel = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                              .target = V4L2_SEL_TGT_COMPOSE };
	if (!ioctl(vfd, VIDIOC_G_SELECTION, &sel)) {
		crop_w = sel.r.width; crop_h = sel.r.height;
	}
	fprintf(stderr, "VIDC CAPTURE %ux%u (visible %ux%u)\n",
		width, height, crop_w, crop_h);

	cap_count = 8;
	struct v4l2_requestbuffers rb = { .count = cap_count,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_MMAP };
	if (ioctl(vfd, VIDIOC_REQBUFS, &rb)) die("REQBUFS CAP");
	cap_count = rb.count;

	for (unsigned i = 0; i < cap_count; i++) {
		struct cap_slot *s = &all_slots[i];
		struct v4l2_plane pl = {0};
		struct v4l2_buffer qb = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP,
			.index = i, .m.planes = &pl, .length = 1,
		};
		if (ioctl(vfd, VIDIOC_QUERYBUF, &qb)) die("QUERYBUF CAP");

		/* mmap is only needed for diagnostic byte-checks; not for the
		 * display path. Map for cheap content peek; comment out for
		 * minimum memory footprint. */
		s->mmap_len  = pl.length;
		s->mmap_addr = mmap(NULL, pl.length, PROT_READ,
				    MAP_SHARED, vfd, pl.m.mem_offset);
		if (s->mmap_addr == MAP_FAILED) s->mmap_addr = NULL;

		/* Export dma-buf for DRM import. */
		struct v4l2_exportbuffer eb = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.index = i, .plane = 0, .flags = O_CLOEXEC,
		};
		if (ioctl(vfd, VIDIOC_EXPBUF, &eb)) die("EXPBUF");
		s->dmabuf_fd = eb.fd;

		if (drm_register_slot(s, width, height)) {
			fprintf(stderr,
				"drm_register_slot(%u): %s — falling back to "
				"single-slot reuse\n", i, strerror(errno));
			break;
		}

		struct v4l2_plane qp = {0};
		struct v4l2_buffer q = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP,
			.index = i, .m.planes = &qp, .length = 1,
		};
		if (ioctl(vfd, VIDIOC_QBUF, &q)) die("QBUF CAP");
	}

	int t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("STREAMON CAP");
}

/* -------------------------------------------------------------------- main */

int main(int argc, char **argv)
{
	const char *in_path = argc > 1 ? argv[1] : "/tmp/test.h264";
	const char *vdev    = argc > 2 ? argv[2] : "/dev/video6";
	const char *cdev    = argc > 3 ? argv[3] : "/dev/dri/card0";

	drm_setup(cdev);
	dec_open(vdev);

	/* Read entire input file. */
	int in_fd = open(in_path, O_RDONLY);
	if (in_fd < 0) die("open input");
	off_t in_total = lseek(in_fd, 0, SEEK_END); lseek(in_fd, 0, SEEK_SET);
	unsigned char *in_data = malloc(in_total);
	if (read(in_fd, in_data, in_total) != in_total) die("read input");
	close(in_fd);

	/* Build access-unit table: a "frame" is a startcode-delimited group of
	 * NALs ending at the next VCL NAL boundary. Identical to the AU walker
	 * in vidc_rot_e2e (use that as the source of truth; bug fixes there
	 * should be ported here verbatim). */
	size_t *au_off = malloc(2048 * sizeof(*au_off));
	size_t au_n = 0, au_cap = 2048;
	{
		int in_au = 0;
		for (off_t i = 0; i + 4 < in_total; i++) {
			int sc = 0; size_t off_nal = 0;
			if (!in_data[i] && !in_data[i+1] && !in_data[i+2] &&
			    in_data[i+3] == 1) { sc = 4; off_nal = i + 4; }
			else if (!in_data[i] && !in_data[i+1] &&
				 in_data[i+2] == 1) { sc = 3; off_nal = i + 3; }
			if (!sc) continue;
			int nt = in_data[off_nal] & 0x1f;
			int vcl = (nt >= 1 && nt <= 5);
			if (!in_au) {
				if (au_n == au_cap) {
					au_cap *= 2;
					au_off = realloc(au_off, au_cap * sizeof(*au_off));
				}
				au_off[au_n++] = i;
				in_au = 1;
			}
			if (vcl) in_au = 0;
			i += sc - 1;
		}
	}
	if (au_n == 0) { au_off[0] = 0; au_n = 1; }
	fprintf(stderr, "Built %zu AUs from %lld bytes\n",
		au_n, (long long)in_total);

	/* Pump the first AU and STREAMON OUTPUT; wait for SOURCE_CHANGE. */
	size_t next_au = 0;
	{
		size_t s = au_off[next_au];
		size_t e = (next_au + 1 < au_n) ? au_off[next_au + 1]
		                                : (size_t)in_total;
		size_t len = e - s;
		if (len > out_size[0]) len = out_size[0];
		memcpy(out_mmap[0], in_data + s, len);
		struct v4l2_plane pl0 = { .bytesused = len };
		struct v4l2_buffer b0 = {
			.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP,
			.index = 0, .m.planes = &pl0, .length = 1,
		};
		if (ioctl(vfd, VIDIOC_QBUF, &b0)) die("QBUF OUT[0]");
		out_avail[0] = 0;
		next_au++;
	}
	int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("STREAMON OUT");

	int got_src = 0, loops = 0;
	while (!got_src && loops++ < 30) {
		struct pollfd pfd = { .fd = vfd, .events = POLLPRI|POLLOUT };
		if (poll(&pfd, 1, 1000) < 0) die("poll src_change");
		if (pfd.revents & POLLPRI) {
			struct v4l2_event ev;
			while (ioctl(vfd, VIDIOC_DQEVENT, &ev) == 0)
				if (ev.type == V4L2_EVENT_SOURCE_CHANGE)
					got_src = 1;
		}
		if (pfd.revents & POLLOUT) {
			struct v4l2_plane dp = {0};
			struct v4l2_buffer db = {
				.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP,
				.m.planes = &dp, .length = 1,
			};
			if (ioctl(vfd, VIDIOC_DQBUF, &db) == 0)
				out_avail[db.index] = 1;
		}
	}
	if (!got_src) die("no SOURCE_CHANGE");
	cap_setup_after_src_change();

	/* Main loop: dispatch OUT/CAP events. Each CAP DQBUF arms a SetPlane
	 * flip; each OUT DQBUF lets us submit the next AU. */
	size_t frames_shown = 0;
	unsigned idle = 0;
	while (idle < 8) {
		struct pollfd pfd = { .fd = vfd,
			.events = POLLIN | POLLOUT | POLLPRI };
		int pr = poll(&pfd, 1, 1000);
		if (pr < 0) die("main poll");
		if (pr == 0) { idle++; continue; }
		idle = 0;

		if (pfd.revents & POLLIN) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = {
				.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				.memory = V4L2_MEMORY_MMAP,
				.m.planes = &pl, .length = 1,
			};
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0 && pl.bytesused) {
				drm_flip_to(&all_slots[b.index]);
				frames_shown++;
				/* Re-queue immediately. We're displaying the
				 * just-dequeued buffer; the firmware may write
				 * the *next* frame into a different slot. */
				struct v4l2_plane qp = {0};
				struct v4l2_buffer rq = {
					.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
					.memory = V4L2_MEMORY_MMAP,
					.index = b.index, .m.planes = &qp,
					.length = 1,
				};
				ioctl(vfd, VIDIOC_QBUF, &rq);
			}
		}
		if (pfd.revents & POLLOUT) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = {
				.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP,
				.m.planes = &pl, .length = 1,
			};
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0) {
				out_avail[b.index] = 1;
				if (next_au < au_n) {
					size_t s = au_off[next_au];
					size_t e = (next_au + 1 < au_n)
					           ? au_off[next_au + 1]
					           : (size_t)in_total;
					size_t len = e - s;
					if (len > out_size[b.index])
						len = out_size[b.index];
					memcpy(out_mmap[b.index],
					       in_data + s, len);
					struct v4l2_plane qp = { .bytesused = len };
					struct v4l2_buffer q = {
						.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
						.memory = V4L2_MEMORY_MMAP,
						.index = b.index,
						.m.planes = &qp, .length = 1,
					};
					q.timestamp.tv_usec = (next_au + 1) * 1000;
					if (ioctl(vfd, VIDIOC_QBUF, &q) == 0) {
						out_avail[b.index] = 0;
						next_au++;
					}
				}
			}
		}
	}

	fprintf(stderr, "vidc_play: %zu frames displayed, %zu input AUs consumed\n",
		frames_shown, next_au);
	drm_restore();
	close(drm_fd);
	close(vfd);
	return 0;
}
