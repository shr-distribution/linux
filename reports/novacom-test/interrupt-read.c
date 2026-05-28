/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * T5 on-device: a blocking read(/dev/novacom_ep_out) interrupted by
 * SIGINT must return -EINTR cleanly. Child fork() pings SIGINT to the
 * parent after 1s.
 *
 * Side effect on dmesg: novacom: <ep> i/o interrupted line.
 *
 *   $CC -O2 -Wall -static -o interrupt-read interrupt-read.c
 *
 * IMPORTANT: stop any loopback bridge first so it does not consume
 * the OUT endpoint.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

static void sigint(int s) { (void)s; /* break read(); rely on EINTR */ }

int main(void)
{
	char buf[4096];
	struct sigaction sa = { .sa_handler = sigint };
	pid_t parent, child;
	ssize_t n;
	int fd, status;

	sigaction(SIGINT, &sa, NULL);

	fd = open("/dev/novacom_ep_out", O_RDONLY);
	if (fd < 0) { perror("open"); return 1; }

	parent = getpid();
	child = fork();
	if (child < 0) { perror("fork"); return 1; }
	if (child == 0) {
		sleep(1);
		kill(parent, SIGINT);
		return 0;
	}

	errno = 0;
	n = read(fd, buf, sizeof(buf));
	printf("read -> %zd errno=%d (%s)%s\n", n, errno,
	       strerror(errno),
	       (n < 0 && errno == EINTR) ? "  [PASS]" : "  [FAIL]");

	waitpid(child, &status, 0);
	close(fd);
	return (n < 0 && errno == EINTR) ? 0 : 1;
}
