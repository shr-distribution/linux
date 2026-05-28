/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Multi-threaded loopback bridge. Decouples ep_out reads from ep_in
 * writes via a pthread, so the device-side can keep draining the OUT
 * pipe while a previous chunk is still being written back. Necessary
 * because the f_novacom driver issues one URB per syscall (sync) --
 * the single-threaded loopback would deadlock with a sync host whose
 * bulk_IN starts only after bulk_OUT finishes.
 *
 * Build (ARM cross or on-device):
 *   $CC -O2 -Wall -pthread -static -o loopback-mt loopback-mt.c
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*
 * BUFSZ matches HS bulk MPS so each read syscall consumes one USB
 * packet. RING_BUFS sets the burst capacity -- the host can push this
 * many MPS packets before the writer thread starts draining (which
 * does not begin until the host's bulk_OUT URB finishes and bulk_IN
 * is posted). Sized to comfortably hold the largest host-tester
 * transfer (262145 / 512 = 513 packets) with margin.
 */
#define BUFSZ		512
#define RING_BUFS	768

struct slot {
	unsigned char buf[BUFSZ];
	ssize_t len;
};

static struct slot ring[RING_BUFS];
static volatile unsigned int head, tail;	/* head: writer adds, tail: reader removes */
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv_has = PTHREAD_COND_INITIALIZER;
static pthread_cond_t cv_free = PTHREAD_COND_INITIALIZER;
static int fd_out = -1, fd_in = -1;
static volatile sig_atomic_t stop;

static void on_signal(int s)
{
	(void)s;
	stop = 1;
	if (fd_out >= 0) close(fd_out);
	if (fd_in >= 0)  close(fd_in);
	pthread_cond_broadcast(&cv_has);
	pthread_cond_broadcast(&cv_free);
}

/* Producer: read ep_out -> ring */
static void *reader(void *unused)
{
	(void)unused;
	while (!stop) {
		pthread_mutex_lock(&mtx);
		while (!stop && (head - tail) >= RING_BUFS)
			pthread_cond_wait(&cv_free, &mtx);
		pthread_mutex_unlock(&mtx);
		if (stop) break;

		struct slot *s = &ring[head % RING_BUFS];
		ssize_t n = read(fd_out, s->buf, BUFSZ);
		if (n < 0) {
			if (errno == EINTR) continue;
			fprintf(stderr, "read: %s\n", strerror(errno));
			break;
		}
		if (n == 0) continue;
		s->len = n;

		pthread_mutex_lock(&mtx);
		head++;
		pthread_cond_signal(&cv_has);
		pthread_mutex_unlock(&mtx);
	}
	stop = 1;
	pthread_cond_broadcast(&cv_has);
	return NULL;
}

/* Consumer: ring -> ep_in */
static void writer(void)
{
	while (!stop) {
		pthread_mutex_lock(&mtx);
		while (!stop && head == tail)
			pthread_cond_wait(&cv_has, &mtx);
		pthread_mutex_unlock(&mtx);
		if (stop) break;

		struct slot *s = &ring[tail % RING_BUFS];
		ssize_t off = 0;
		while (off < s->len) {
			ssize_t w = write(fd_in, s->buf + off, s->len - off);
			if (w < 0) {
				if (errno == EINTR) continue;
				fprintf(stderr, "write: %s\n", strerror(errno));
				return;
			}
			off += w;
		}

		pthread_mutex_lock(&mtx);
		tail++;
		pthread_cond_signal(&cv_free);
		pthread_mutex_unlock(&mtx);
	}
}

int main(void)
{
	pthread_t rt;
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	fd_out = open("/dev/novacom_ep_out", O_RDONLY);
	fd_in  = open("/dev/novacom_ep_in",  O_WRONLY);
	if (fd_out < 0 || fd_in < 0) { perror("open"); return 1; }

	fprintf(stderr, "loopback-mt ready (BUFSZ=%d, RING=%d)\n",
		BUFSZ, RING_BUFS);

	if (pthread_create(&rt, NULL, reader, NULL) != 0) {
		perror("pthread_create");
		return 1;
	}
	writer();
	pthread_join(rt, NULL);
	fprintf(stderr, "loopback-mt stop\n");
	return 0;
}
