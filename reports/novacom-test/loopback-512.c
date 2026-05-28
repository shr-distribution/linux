/* SPDX-License-Identifier: GPL-2.0+ */
/* Loopback variant that reads in MPS-sized chunks (512). Forces
   USB BULK alignment so device-side URB length matches transfer length. */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BUFSZ 512

static int fd_out = -1, fd_in = -1;
static volatile sig_atomic_t stop;

static void on_signal(int s) {
	(void)s; stop = 1;
	if (fd_out >= 0) close(fd_out);
	if (fd_in >= 0) close(fd_in);
}

int main(void)
{
	unsigned char buf[BUFSZ];
	signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
	fd_out = open("/dev/novacom_ep_out", O_RDONLY);
	fd_in  = open("/dev/novacom_ep_in",  O_WRONLY);
	if (fd_out < 0 || fd_in < 0) { perror("open"); return 1; }
	fprintf(stderr, "loopback-512 ready\n");
	while (!stop) {
		ssize_t n = read(fd_out, buf, BUFSZ);
		if (n < 0) { if (errno == EINTR) continue; perror("read"); break; }
		if (n == 0) continue;
		ssize_t off = 0;
		while (off < n) {
			ssize_t w = write(fd_in, buf+off, n-off);
			if (w < 0) { if (errno == EINTR) continue; perror("write"); goto done; }
			off += w;
		}
	}
done:
	return 0;
}
