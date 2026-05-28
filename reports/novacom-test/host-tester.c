/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * novacom host-side tester — runs on the developer workstation.
 *
 * Pairs with reports/novacom-test/loopback.c on the device. Drives the
 * bulk pipes with random data at increasing sizes, verifies byte-perfect
 * loopback, and exercises the wrong-direction STALL path.
 *
 * Build:
 *   cc -O2 -Wall -o host-tester host-tester.c -lusb-1.0
 *
 * Run (as root or with udev rule):
 *   ./host-tester
 *
 * Exits 0 on success, non-zero on any failed iteration.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libusb-1.0/libusb.h>

#define NOVACOM_VID		0x0830
#define NOVACOM_PID		0x8002
#define NOVACOM_IFACE		0
#define NOVACOM_SUBCLASS	0x47
#define NOVACOM_PROTOCOL	0x11

#define XFER_TIMEOUT_MS		5000

/* Sizes that exercise: a sub-MPS write (FS=64, HS=512), a few MPS, a 4KB
 * page, a typical novacomd frame, and the kernel NOVACOM_MAX_XFER (256 KiB)
 * boundary. */
static const size_t test_sizes[] = {
	48, 64, 128, 512, 513, 1024, 4096, 65536,
	256 * 1024 - 1, 256 * 1024, 256 * 1024 + 1,
};

static int find_endpoints(libusb_device *dev, uint8_t *ep_in, uint8_t *ep_out)
{
	struct libusb_config_descriptor *cfg;
	int r;

	r = libusb_get_active_config_descriptor(dev, &cfg);
	if (r != 0)
		return r;

	*ep_in = 0;
	*ep_out = 0;

	for (int i = 0; i < cfg->bNumInterfaces; i++) {
		const struct libusb_interface *iface = &cfg->interface[i];
		for (int a = 0; a < iface->num_altsetting; a++) {
			const struct libusb_interface_descriptor *id =
				&iface->altsetting[a];
			if (id->bInterfaceSubClass != NOVACOM_SUBCLASS ||
			    id->bInterfaceProtocol != NOVACOM_PROTOCOL)
				continue;
			for (int e = 0; e < id->bNumEndpoints; e++) {
				uint8_t addr = id->endpoint[e].bEndpointAddress;
				if (addr & LIBUSB_ENDPOINT_DIR_MASK)
					*ep_in = addr;
				else
					*ep_out = addr;
			}
		}
	}

	libusb_free_config_descriptor(cfg);
	return (*ep_in && *ep_out) ? 0 : -ENOENT;
}

static void fill_random(uint8_t *buf, size_t n)
{
	for (size_t i = 0; i < n; i++)
		buf[i] = (uint8_t)(rand() & 0xff);
}

static int test_roundtrip(libusb_device_handle *h, uint8_t ep_in,
			  uint8_t ep_out, size_t n)
{
	uint8_t *tx = malloc(n);
	uint8_t *rx = malloc(n);
	int actual, r;

	if (!tx || !rx) {
		free(tx); free(rx);
		fprintf(stderr, "[%zu] alloc failed\n", n);
		return -1;
	}

	fill_random(tx, n);

	r = libusb_bulk_transfer(h, ep_out, tx, n, &actual, XFER_TIMEOUT_MS);
	if (r != 0 || (size_t)actual != n) {
		fprintf(stderr, "[%zu] OUT failed: %s actual=%d\n",
			n, libusb_error_name(r), actual);
		free(tx); free(rx);
		return -1;
	}

	r = libusb_bulk_transfer(h, ep_in, rx, n, &actual, XFER_TIMEOUT_MS);
	if (r != 0 || (size_t)actual != n) {
		fprintf(stderr, "[%zu] IN  failed: %s actual=%d\n",
			n, libusb_error_name(r), actual);
		free(tx); free(rx);
		return -1;
	}

	if (memcmp(tx, rx, n) != 0) {
		fprintf(stderr, "[%zu] DATA MISMATCH\n", n);
		free(tx); free(rx);
		return -1;
	}

	printf("[%6zu] OK\n", n);
	free(tx); free(rx);
	return 0;
}

/*
 * Wrong-direction halt: try a bulk IN on the OUT endpoint address. The
 * kernel driver should STALL the endpoint; libusb maps that to
 * LIBUSB_ERROR_PIPE.
 */
static int test_wrong_dir_stall(libusb_device_handle *h, uint8_t ep_out)
{
	uint8_t scratch[64];
	int actual, r;
	uint8_t ep_out_as_in = ep_out | LIBUSB_ENDPOINT_IN;

	r = libusb_bulk_transfer(h, ep_out_as_in, scratch, sizeof(scratch),
				 &actual, XFER_TIMEOUT_MS);
	if (r == LIBUSB_ERROR_PIPE) {
		printf("[wrong-dir] OK (STALL)\n");
		/* Clear the halt for any later iterations. */
		libusb_clear_halt(h, ep_out_as_in);
		return 0;
	}

	fprintf(stderr, "[wrong-dir] expected STALL, got: %s actual=%d\n",
		libusb_error_name(r), actual);
	return -1;
}

int main(int argc, char **argv)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *h;
	libusb_device *dev;
	uint8_t ep_in = 0, ep_out = 0;
	int r, fails = 0;
	unsigned int seed = argc > 1 ? (unsigned int)atoi(argv[1]) : 1;

	srand(seed);
	printf("seed=%u\n", seed);

	r = libusb_init(&ctx);
	if (r != 0) {
		fprintf(stderr, "libusb_init: %s\n", libusb_error_name(r));
		return 1;
	}

	h = libusb_open_device_with_vid_pid(ctx, NOVACOM_VID, NOVACOM_PID);
	if (!h) {
		fprintf(stderr, "no %04x:%04x device — is the gadget bound?\n",
			NOVACOM_VID, NOVACOM_PID);
		libusb_exit(ctx);
		return 2;
	}

	dev = libusb_get_device(h);
	r = find_endpoints(dev, &ep_in, &ep_out);
	if (r != 0) {
		fprintf(stderr, "could not locate novacom endpoints (%d)\n", r);
		libusb_close(h);
		libusb_exit(ctx);
		return 3;
	}
	printf("ep_in=0x%02x ep_out=0x%02x\n", ep_in, ep_out);

	libusb_set_auto_detach_kernel_driver(h, 1);

	r = libusb_claim_interface(h, NOVACOM_IFACE);
	if (r != 0) {
		fprintf(stderr, "claim_interface: %s\n", libusb_error_name(r));
		libusb_close(h);
		libusb_exit(ctx);
		return 4;
	}

	for (size_t i = 0; i < sizeof(test_sizes) / sizeof(test_sizes[0]); i++) {
		if (test_roundtrip(h, ep_in, ep_out, test_sizes[i]) != 0)
			fails++;
	}

	if (test_wrong_dir_stall(h, ep_out) != 0)
		fails++;

	libusb_release_interface(h, NOVACOM_IFACE);
	libusb_close(h);
	libusb_exit(ctx);

	if (fails) {
		fprintf(stderr, "%d failure(s)\n", fails);
		return 5;
	}

	printf("ALL OK\n");
	return 0;
}
