/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * T4 on-device: exercise the wrong-direction guards in f_novacom.
 *
 *   read(/dev/novacom_ep_in,  ...) should issue usb_ep_set_halt on
 *     the IN endpoint and return -EBADMSG.
 *   write(/dev/novacom_ep_out, ...) should issue usb_ep_set_halt on
 *     the OUT endpoint and return -EBADMSG.
 *
 * Run after the loopback bridge is *not* using the endpoints, so the
 * driver's mutex doesn't block us out.
 *
 *   $CC -O2 -Wall -static -o wrong-dir wrong-dir.c
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int test(const char *path, int flags, int doing_read)
{
	char buf[64] = {0};
	ssize_t n;
	int fd;

	fd = open(path, flags);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		return 1;
	}

	if (doing_read)
		n = read(fd, buf, sizeof(buf));
	else
		n = write(fd, buf, sizeof(buf));

	printf("%-25s %s -> %zd errno=%d (%s)%s\n",
	       path, doing_read ? "read" : "write", n, errno,
	       strerror(errno),
	       (n < 0 && errno == EBADMSG) ? "  [PASS]" : "  [FAIL]");

	close(fd);
	return (n < 0 && errno == EBADMSG) ? 0 : 1;
}

int main(void)
{
	int fails = 0;

	fails += test("/dev/novacom_ep_in",  O_RDONLY, 1); /* read IN  */
	fails += test("/dev/novacom_ep_out", O_WRONLY, 0); /* write OUT */

	return fails ? 1 : 0;
}
