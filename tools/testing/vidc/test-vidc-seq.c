/*
 * test-vidc-seq.c - Test VIDC SEQ_HEADER by queuing real SPS+PPS data
 *
 * Exercises the firmware's sequence-header decode path:
 *   STREAMON OUTPUT → QBUF(SPS+PPS) → wait for SEQ_DONE or error IRQ
 *
 * Build:
 *   arm-linux-gnueabihf-gcc -o test-vidc-seq test-vidc-seq.c -static
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define VIDC_DEC_NODE  "/dev/video6"
#define WIDTH  320
#define HEIGHT 240

/* SPS+PPS for 320x240 constrained-baseline H.264, byte-stream format */
static const unsigned char sps_pps[] = {
	0x00, 0x00, 0x00, 0x01,  /* start code */
	0x67, 0x42, 0xc0, 0x1e, 0xd9, 0x01, 0x41, 0xfb,
	0x01, 0x10, 0x00, 0x00, 0x03, 0x00, 0x10, 0x00,
	0x00, 0x03, 0x03, 0x20, 0xf1, 0x62, 0xe4, 0x80,
	/* PPS start code */
	0x00, 0x00, 0x00, 0x01,
	0x68, 0xcb, 0x83, 0xcb, 0x20,
};

int main(void)
{
	int fd, i;
	struct v4l2_format fmt = {};
	struct v4l2_requestbuffers reqbuf = {};
	struct v4l2_buffer buf = {};
	struct v4l2_plane plane = {};
	void *buf_addr;
	int type;

	fd = open(VIDC_DEC_NODE, O_RDWR);
	if (fd < 0) { perror("open"); return 1; }
	printf("opened %s\n", VIDC_DEC_NODE);

	/* OUTPUT format: H264 320x240 */
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.width  = WIDTH;
	fmt.fmt.pix_mp.height = HEIGHT;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = 524288;
	if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT OUTPUT"); return 1; }
	printf("S_FMT OUTPUT ok\n");

	/* CAPTURE format: NV12 */
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.width  = WIDTH;
	fmt.fmt.pix_mp.height = HEIGHT;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = WIDTH * HEIGHT * 3 / 2;
	if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("S_FMT CAPTURE"); return 1; }
	printf("S_FMT CAPTURE ok\n");

	/* REQBUFS OUTPUT */
	reqbuf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count  = 4;
	if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) { perror("REQBUFS OUTPUT"); return 1; }
	printf("REQBUFS OUTPUT ok: count=%d\n", reqbuf.count);

	/* REQBUFS CAPTURE */
	reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count  = 8;
	if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) { perror("REQBUFS CAPTURE"); return 1; }
	printf("REQBUFS CAPTURE ok: count=%d\n", reqbuf.count);

	/* MMAP output buffer 0 */
	buf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index  = 0;
	buf.length = 1;
	buf.m.planes = &plane;
	if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) { perror("QUERYBUF"); return 1; }
	buf_addr = mmap(NULL, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, plane.m.mem_offset);
	if (buf_addr == MAP_FAILED) { perror("mmap"); return 1; }
	printf("MMAP output buf[0] ok: length=%u\n", plane.length);

	/* Copy SPS+PPS into buffer */
	memcpy(buf_addr, sps_pps, sizeof(sps_pps));
	printf("Copied %zu bytes of SPS+PPS into buf[0]\n", sizeof(sps_pps));

	/* STREAMON OUTPUT */
	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	printf("STREAMON OUTPUT...\n"); fflush(stdout);
	if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON OUTPUT"); return 1; }
	printf("STREAMON OUTPUT ok\n");

	/* QBUF with SPS+PPS — triggers SEQ_HEADER to firmware */
	memset(&buf, 0, sizeof(buf));
	memset(&plane, 0, sizeof(plane));
	buf.type      = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.memory    = V4L2_MEMORY_MMAP;
	buf.index     = 0;
	buf.length    = 1;
	buf.m.planes  = &plane;
	plane.bytesused = sizeof(sps_pps);
	printf("QBUF OUTPUT buf[0] bytesused=%d (SPS+PPS)...\n", (int)sizeof(sps_pps));
	fflush(stdout);
	if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) { perror("QBUF OUTPUT"); return 1; }
	printf("QBUF ok — SEQ_HEADER submitted to firmware\n");

	/* Wait up to 3 seconds for firmware response */
	printf("Waiting for firmware SEQ_DONE (check dmesg)...\n");
	fflush(stdout);
	sleep(3);

	printf("Done — check dmesg for SEQ_DONE or Firmware error\n");
	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	ioctl(fd, VIDIOC_STREAMOFF, &type);
	close(fd);
	return 0;
}
