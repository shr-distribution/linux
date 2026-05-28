/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * One-shot probe: send a single bulk OUT of configurable size to the
 * novacom gadget and read it back. Builds on the host workstation,
 * matches the loopback bridge's BUFSZ so we don't depend on short-packet
 * completion behaviour.
 *
 *   cc -O2 -Wall -o probe probe.c -lusb-1.0
 *   ./probe <bytes>          # default 65536
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#define SUBCLASS 0x47
#define PROTOCOL 0x11
#define TIMEOUT_MS 10000

static int find_one(libusb_device *dev, int *iface, uint8_t *ein, uint8_t *eout)
{
	struct libusb_config_descriptor *cfg;
	int r = libusb_get_active_config_descriptor(dev, &cfg);
	if (r) return r;
	*iface = -1; *ein = 0; *eout = 0;
	for (int i = 0; i < cfg->bNumInterfaces; i++)
		for (int a = 0; a < cfg->interface[i].num_altsetting; a++) {
			const struct libusb_interface_descriptor *id =
				&cfg->interface[i].altsetting[a];
			if (id->bInterfaceSubClass != SUBCLASS ||
			    id->bInterfaceProtocol != PROTOCOL) continue;
			*iface = id->bInterfaceNumber;
			for (int e = 0; e < id->bNumEndpoints; e++) {
				uint8_t ea = id->endpoint[e].bEndpointAddress;
				if (ea & 0x80) *ein = ea; else *eout = ea;
			}
		}
	libusb_free_config_descriptor(cfg);
	return (*iface >= 0 && *ein && *eout) ? 0 : -ENOENT;
}

int main(int argc, char **argv)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *h = NULL;
	libusb_device **list;
	int iface = -1, n = argc > 1 ? atoi(argv[1]) : 65536;
	uint8_t ein = 0, eout = 0;
	uint8_t *tx, *rx;
	int actual, r;
	ssize_t ndev;

	libusb_init(&ctx);
	ndev = libusb_get_device_list(ctx, &list);
	for (ssize_t i = 0; i < ndev; i++) {
		if (find_one(list[i], &iface, &ein, &eout) == 0) {
			if (libusb_open(list[i], &h) == 0) break;
			h = NULL;
		}
	}
	libusb_free_device_list(list, 1);
	if (!h) { fprintf(stderr, "no novacom\n"); return 2; }

	printf("iface=%d ep_in=0x%02x ep_out=0x%02x size=%d\n", iface, ein, eout, n);
	libusb_set_auto_detach_kernel_driver(h, 1);
	r = libusb_claim_interface(h, iface);
	if (r) { fprintf(stderr, "claim: %s\n", libusb_error_name(r)); return 3; }

	tx = malloc(n); rx = malloc(n);
	for (int i = 0; i < n; i++) tx[i] = i & 0xff;

	printf("OUT %d bytes... ", n); fflush(stdout);
	r = libusb_bulk_transfer(h, eout, tx, n, &actual, TIMEOUT_MS);
	printf("rc=%s actual=%d\n", libusb_error_name(r), actual);

	if (r == 0 && actual == n) {
		printf("IN  %d bytes... ", n); fflush(stdout);
		r = libusb_bulk_transfer(h, ein, rx, n, &actual, TIMEOUT_MS);
		printf("rc=%s actual=%d\n", libusb_error_name(r), actual);
		if (r == 0 && actual == n)
			printf("match=%s\n", memcmp(tx, rx, n) == 0 ? "OK" : "MISMATCH");
	}

	libusb_release_interface(h, iface);
	libusb_close(h);
	libusb_exit(ctx);
	free(tx); free(rx);
	return 0;
}
