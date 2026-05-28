/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * novacom loopback bridge — runs ON the TouchPad.
 *
 * Reads bulk OUT data from /dev/novacom_ep_out and echoes it byte-for-byte
 * to /dev/novacom_ep_in. Used by the host-side tester to validate the
 * data plane without touching novacomd.
 *
 * Build (cross-compile or on-device):
 *   $CC -O2 -Wall -o loopback loopback.c
 *
 * Run:
 *   ./loopback &
 *
 * Stop with kill / Ctrl-C; both fds are closed in the signal handler so
 * any in-flight USB request is cancelled cleanly.
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define BUFSZ		(64 * 1024)
#define DEV_OUT		"/dev/novacom_ep_out"
#define DEV_IN		"/dev/novacom_ep_in"

static int fd_out = -1;
static int fd_in = -1;
static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
	if (fd_out >= 0)
		close(fd_out);
	if (fd_in >= 0)
		close(fd_in);
}

int main(void)
{
	unsigned char *buf;
	unsigned long long total_in = 0, total_out = 0;
	ssize_t n;

	buf = malloc(BUFSZ);
	if (!buf) {
		perror("malloc");
		return 1;
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	fd_out = open(DEV_OUT, O_RDONLY);
	if (fd_out < 0) {
		perror(DEV_OUT);
		return 1;
	}
	fd_in = open(DEV_IN, O_WRONLY);
	if (fd_in < 0) {
		perror(DEV_IN);
		close(fd_out);
		return 1;
	}

	fprintf(stderr, "novacom-loopback: ready, BUFSZ=%d\n", BUFSZ);

	while (!stop) {
		n = read(fd_out, buf, BUFSZ);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "read: %s\n", strerror(errno));
			break;
		}
		if (n == 0)
			continue;
		total_in += n;

		ssize_t off = 0;
		while (off < n) {
			ssize_t w = write(fd_in, buf + off, n - off);
			if (w < 0) {
				if (errno == EINTR)
					continue;
				fprintf(stderr, "write: %s\n", strerror(errno));
				goto done;
			}
			off += w;
			total_out += w;
		}
	}

done:
	fprintf(stderr, "novacom-loopback: stop, in=%llu out=%llu\n",
		total_in, total_out);
	free(buf);
	return 0;
}
