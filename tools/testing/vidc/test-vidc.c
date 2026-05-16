/*
 * test-vidc.c - Minimal VIDC firmware boot test
 *
 * Triggers firmware load by opening the decoder node and calling STREAMON.
 * Use to verify the VIDC RISC CPU executes firmware after clock/reset fixes.
 *
 * Build (cross-compile for ARM):
 *   arm-linux-gnueabihf-gcc -o test-vidc test-vidc.c -static
 *
 * Run on device:
 *   /tmp/test-vidc
 *   dmesg | grep -i vidc
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

/* VIDC decoder is video6/video7, not video0 (which is camss) */
#define VIDC_DEC_NODE  "/dev/video6"

/* Use small resolution to minimise buffer allocation */
#define WIDTH  176
#define HEIGHT 144

int main(void)
{
	int fd;
	struct v4l2_format fmt = {};
	struct v4l2_requestbuffers reqbuf = {};

	fd = open(VIDC_DEC_NODE, O_RDWR);
	if (fd < 0) {
		perror("open " VIDC_DEC_NODE);
		return 1;
	}
	printf("opened %s\n", VIDC_DEC_NODE);

	/* Set output (compressed input to decoder) format */
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.width = WIDTH;
	fmt.fmt.pix_mp.height = HEIGHT;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = 524288;
	if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
		perror("VIDIOC_S_FMT OUTPUT");
		close(fd);
		return 1;
	}
	printf("S_FMT OUTPUT ok\n");

	/* Set capture (decoded raw output) format */
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.width = WIDTH;
	fmt.fmt.pix_mp.height = HEIGHT;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = WIDTH * HEIGHT * 3 / 2;
	if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
		perror("VIDIOC_S_FMT CAPTURE");
		close(fd);
		return 1;
	}
	printf("S_FMT CAPTURE ok\n");

	/* Request output buffers */
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count = 4;
	if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
		perror("VIDIOC_REQBUFS OUTPUT");
		close(fd);
		return 1;
	}
	printf("REQBUFS OUTPUT ok: count=%d\n", reqbuf.count);

	/* Request capture buffers */
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count = 4;
	if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) < 0) {
		perror("VIDIOC_REQBUFS CAPTURE");
		close(fd);
		return 1;
	}
	printf("REQBUFS CAPTURE ok: count=%d\n", reqbuf.count);

	/* STREAMON OUTPUT - triggers pm_runtime_resume, firmware load, RISC boot */
	int type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	printf("STREAMON OUTPUT (triggers firmware load + RISC boot)...\n");
	fflush(stdout);
	if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
		perror("VIDIOC_STREAMON OUTPUT");
		printf("FAILED - check dmesg for VIDC errors\n");
		close(fd);
		return 1;
	}
	printf("STREAMON OUTPUT ok - firmware executed!\n");

	sleep(1);

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	ioctl(fd, VIDIOC_STREAMOFF, &type);
	close(fd);
	printf("Done.\n");
	return 0;
}
