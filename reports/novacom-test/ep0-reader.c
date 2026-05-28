/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * novacom ep0 event reader — runs ON the TouchPad.
 *
 * Opens /dev/novacom_ep0, blocks in read(), and prints each
 * usb_gadgetfs_event record as it arrives. Used to verify T3
 * (event delivery on host connect / disconnect / set-config).
 *
 * Build:
 *   $CC -O2 -Wall -o ep0-reader ep0-reader.c
 *
 * Run on a second console or backgrounded:
 *   ./ep0-reader &
 *
 * Stop with Ctrl-C or `kill`.
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/usb/gadgetfs.h>
#include <linux/usb/ch9.h>

#define DEV_EP0		"/dev/novacom_ep0"

static int fd = -1;
static volatile sig_atomic_t stop;

static const char *evtype_str(enum usb_gadgetfs_event_type t)
{
	switch (t) {
	case GADGETFS_NOP:		return "NOP";
	case GADGETFS_CONNECT:		return "CONNECT";
	case GADGETFS_DISCONNECT:	return "DISCONNECT";
	case GADGETFS_SETUP:		return "SETUP";
	case GADGETFS_SUSPEND:		return "SUSPEND";
	default:			return "UNKNOWN";
	}
}

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
	if (fd >= 0)
		close(fd);
}

static void dump(const struct usb_gadgetfs_event *e)
{
	printf("event type=%-10s ", evtype_str(e->type));
	if (e->type == GADGETFS_SETUP) {
		printf("setup bmRT=0x%02x bReq=0x%02x wVal=0x%04x "
		       "wIdx=0x%04x wLen=%u",
		       e->u.setup.bRequestType,
		       e->u.setup.bRequest,
		       (unsigned)e->u.setup.wValue,
		       (unsigned)e->u.setup.wIndex,
		       (unsigned)e->u.setup.wLength);
	}
	printf("\n");
	fflush(stdout);
}

int main(void)
{
	struct usb_gadgetfs_event events[8];
	ssize_t n;

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	fd = open(DEV_EP0, O_RDONLY);
	if (fd < 0) {
		perror(DEV_EP0);
		return 1;
	}

	fprintf(stderr, "ep0-reader: opened %s\n", DEV_EP0);

	while (!stop) {
		n = read(fd, events, sizeof(events));
		if (n < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "read: %s\n", strerror(errno));
			break;
		}
		if (n == 0)
			continue;
		if (n % sizeof(events[0]) != 0) {
			fprintf(stderr, "short event read: %zd bytes\n", n);
			break;
		}
		for (size_t i = 0; i < (size_t)n / sizeof(events[0]); i++)
			dump(&events[i]);
	}

	fprintf(stderr, "ep0-reader: stop\n");
	return 0;
}
