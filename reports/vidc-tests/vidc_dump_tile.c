/*
 * vidc_dump_tile — decode 1 frame via VIDC, dump the raw tile-NV12 CAPTURE
 * buffer to /tmp/tile.bin.
 *
 * Then on the host: try various detile formulas in software against the
 * captured bytes and compare with the rotator ground-truth. Lets us
 * isolate the tile-layout question from anything Mesa is doing.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#define die(m) do { perror(m); exit(1); } while (0)

int main(int argc, char **argv)
{
	const char *in_path = argc > 1 ? argv[1] : "/tmp/michael_1024x768.264";
	const char *out     = argc > 2 ? argv[2] : "/tmp/tile.bin";
	int vfd = open("/dev/video6", O_RDWR | O_NONBLOCK);
	if (vfd < 0) die("open");

	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	f.fmt.pix_mp.width = 320; f.fmt.pix_mp.height = 240;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f.fmt.pix_mp.num_planes = 1;
	f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
	if (ioctl(vfd, VIDIOC_S_FMT, &f)) die("S_FMT OUT");

	struct v4l2_format fc = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	fc.fmt.pix_mp.width = 320; fc.fmt.pix_mp.height = 240;
	fc.fmt.pix_mp.pixelformat = v4l2_fourcc('T','M','1','2');
	fc.fmt.pix_mp.num_planes = 1;
	if (ioctl(vfd, VIDIOC_S_FMT, &fc)) die("S_FMT CAP");

	struct v4l2_event_subscription es = { .type = V4L2_EVENT_SOURCE_CHANGE };
	ioctl(vfd, VIDIOC_SUBSCRIBE_EVENT, &es);

	struct v4l2_requestbuffers rb = { .count = 2,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, .memory = V4L2_MEMORY_MMAP };
	ioctl(vfd, VIDIOC_REQBUFS, &rb);
	void *out_mmap[2]; size_t out_size[2];
	for (int i = 0; i < 2; i++) {
		struct v4l2_plane pl = {0};
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		ioctl(vfd, VIDIOC_QUERYBUF, &b);
		out_size[i] = pl.length;
		out_mmap[i] = mmap(0, pl.length, PROT_READ|PROT_WRITE,
				   MAP_SHARED, vfd, pl.m.mem_offset);
	}

	/* Read whole input. */
	int ifd = open(in_path, O_RDONLY); if (ifd < 0) die("open input");
	off_t total = lseek(ifd, 0, SEEK_END); lseek(ifd, 0, SEEK_SET);
	unsigned char *data = malloc(total);
	if (read(ifd, data, total) != total) die("read input");
	close(ifd);

	/* Build AU table (same as vidc_play). */
	size_t *au = malloc(2048 * sizeof(*au));
	size_t au_n = 0, au_cap = 2048; int in_au = 0;
	for (off_t i = 0; i + 4 < total; i++) {
		int sc = 0; size_t off_nal = 0;
		if (!data[i] && !data[i+1] && !data[i+2] && data[i+3] == 1) {
			sc = 4; off_nal = i + 4;
		} else if (!data[i] && !data[i+1] && data[i+2] == 1) {
			sc = 3; off_nal = i + 3;
		}
		if (!sc) continue;
		int nt = data[off_nal] & 0x1f, vcl = (nt >= 1 && nt <= 5);
		if (!in_au) {
			if (au_n == au_cap) { au_cap *= 2;
				au = realloc(au, au_cap * sizeof(*au)); }
			au[au_n++] = i;
			in_au = 1;
		}
		if (vcl) in_au = 0;
		i += sc - 1;
	}

	/* Submit AU 0. */
	size_t s = au[0], e = au_n > 1 ? au[1] : (size_t)total;
	size_t len = e - s; if (len > out_size[0]) len = out_size[0];
	memcpy(out_mmap[0], data + s, len);
	struct v4l2_plane pl0 = { .bytesused = len };
	struct v4l2_buffer b0 = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_MMAP, .index = 0,
		.m.planes = &pl0, .length = 1 };
	ioctl(vfd, VIDIOC_QBUF, &b0);
	int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("STREAMON OUT");

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
			ioctl(vfd, VIDIOC_DQBUF, &b);
		}
	}
	if (!got_src) die("no SOURCE_CHANGE");

	ioctl(vfd, VIDIOC_G_FMT, &fc);
	unsigned w = fc.fmt.pix_mp.width;
	unsigned h = fc.fmt.pix_mp.height;
	unsigned bpl = fc.fmt.pix_mp.plane_fmt[0].bytesperline;
	unsigned sz = fc.fmt.pix_mp.plane_fmt[0].sizeimage;
	fprintf(stderr, "CAP %ux%u bpl=%u size=%u\n", w, h, bpl, sz);

	struct v4l2_requestbuffers rbc = { .count = 4,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, .memory = V4L2_MEMORY_MMAP };
	ioctl(vfd, VIDIOC_REQBUFS, &rbc);
	void *cap_map[4]; size_t cap_len[4];
	for (unsigned i = 0; i < rbc.count; i++) {
		struct v4l2_plane pl = {0};
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		ioctl(vfd, VIDIOC_QUERYBUF, &b);
		cap_len[i] = pl.length;
		cap_map[i] = mmap(0, pl.length, PROT_READ, MAP_SHARED, vfd, pl.m.mem_offset);
		struct v4l2_plane qp = {0};
		struct v4l2_buffer qb = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &qp, .length = 1 };
		ioctl(vfd, VIDIOC_QBUF, &qb);
	}
	t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(vfd, VIDIOC_STREAMON, &t)) die("STREAMON CAP");

	/* Submit a few more AUs so we get a fully-decoded frame (not just the
	 * I-frame). */
	size_t next_au = 1;
	for (int i = 1; i < 2 && next_au < au_n; i++) {
		size_t ss = au[next_au];
		size_t ee = next_au + 1 < au_n ? au[next_au+1] : (size_t)total;
		size_t ll = ee - ss; if (ll > out_size[i]) ll = out_size[i];
		memcpy(out_mmap[i], data + ss, ll);
		struct v4l2_plane pl = { .bytesused = ll };
		struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP, .index = i,
			.m.planes = &pl, .length = 1 };
		ioctl(vfd, VIDIOC_QBUF, &b);
		next_au++;
	}

	/* Dequeue 30 CAPTURE buffers (skip startup, ensure cleanly-decoded
	 * frame), dump the 30th to file. */
	int frames = 0;
	while (frames < 30) {
		struct pollfd pfd = { .fd = vfd, .events = POLLIN|POLLOUT };
		poll(&pfd, 1, 1000);
		if (pfd.revents & POLLIN) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0 && pl.bytesused) {
				frames++;
				if (frames == 30) {
					int o = open(out, O_WRONLY|O_CREAT|O_TRUNC, 0644);
					ssize_t n = write(o, cap_map[b.index], cap_len[b.index]);
					(void)n;
					close(o);
					fprintf(stderr, "dumped frame 30 (size=%zu) -> %s\n",
						cap_len[b.index], out);
					break;
				}
				struct v4l2_plane qp = {0};
				struct v4l2_buffer rq = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
					.memory = V4L2_MEMORY_MMAP, .index = b.index,
					.m.planes = &qp, .length = 1 };
				ioctl(vfd, VIDIOC_QBUF, &rq);
			}
		}
		if (pfd.revents & POLLOUT) {
			struct v4l2_plane pl = {0};
			struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				.memory = V4L2_MEMORY_MMAP, .m.planes = &pl, .length = 1 };
			if (ioctl(vfd, VIDIOC_DQBUF, &b) == 0) {
				if (next_au < au_n) {
					size_t ss = au[next_au];
					size_t ee = next_au + 1 < au_n ? au[next_au+1] : (size_t)total;
					size_t ll = ee - ss; if (ll > out_size[b.index]) ll = out_size[b.index];
					memcpy(out_mmap[b.index], data + ss, ll);
					struct v4l2_plane pl2 = { .bytesused = ll };
					struct v4l2_buffer q = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
						.memory = V4L2_MEMORY_MMAP, .index = b.index,
						.m.planes = &pl2, .length = 1 };
					ioctl(vfd, VIDIOC_QBUF, &q);
					next_au++;
				}
			}
		}
	}
	close(vfd);
	return 0;
}
