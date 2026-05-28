/*
 * gemini_reg_poll — live Gemini JPEG encoder register tracer.
 *
 * Designed to run on the HP TouchPad under either webOS 3.0.6 (where the
 * stock OPAL camera stack drives the Gemini hardware correctly) or our
 * mainline Linux build (where the encoder produces valid JFIF containers
 * with garbage pixel content). Compares the two register-write sequences
 * to find what OPAL does that we don't.
 *
 * Operation:
 *   1. mmap /dev/mem at the Gemini base (0x04600000, 4 KB) and the MMCC
 *      power/clock controller (0x04000000, 64 KB).
 *   2. Snapshot the full Gemini register region at three sync points:
 *        "before" — at startup
 *        "during" — after camera activity is detected (FE_CMD or
 *                   PIPELINE_CFG go non-zero)
 *        "after"  — after activity has stopped (encoder idle for >500 ms)
 *   3. Between snapshots, poll a curated set of "interesting" registers
 *      at ~10 kHz and log every value change to a delta file.
 *
 * Output (under /tmp by default):
 *   gemini_snapshot_before.txt
 *   gemini_snapshot_during.txt
 *   gemini_snapshot_after.txt
 *   gemini_delta.log
 *
 * Build (host):
 *   arm-linux-gnueabihf-gcc -O2 -static -Wall \
 *       gemini_reg_poll.c -o gemini_reg_poll
 *
 * Run on device:
 *   ./gemini_reg_poll &
 *   # … take a photo with the Camera app …
 *   # poller exits automatically once it has captured before/during/after
 *
 * Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define GEMINI_PHYS_BASE	0x04600000UL
#define GEMINI_REGION_SIZE	0x1000	/* 4 KB */

#define MMCC_PHYS_BASE		0x04000000UL
#define MMCC_REGION_SIZE	0x10000	/* 64 KB */

/*
 * Poll interval for the tight loop, in microseconds. 100 us = 10 kHz, well
 * inside what a single CPU core can sustain via mmap'd reads, and fine
 * enough to catch most encoder transitions without dropping events.
 */
#define POLL_INTERVAL_US	100

/* Activity-watch thresholds */
#define IDLE_GRACE_NS		500000000LL	/* 500 ms idle → "after" */
#define MAX_RUNTIME_S		300		/* abort after 5 min */

struct reg_def {
	const char *name;
	uint32_t off;
	int side_effects;	/* 1 = reading the reg has side effects (skip in poll) */
};

/*
 * The full named-register set. Anything not in this list still gets dumped
 * in the snapshot files (we walk 0x000..0xFFF in 4-byte steps).
 */
static const struct reg_def gemini_regs[] = {
	{ "HW_VERSION",		0x0000, 0 },
	{ "RESET_CMD",		0x0004, 0 },
	{ "PIPELINE_CFG",	0x0008, 0 },
	{ "REALTIME_CMD",	0x000C, 0 },
	{ "IRQ_MASK",		0x0014, 0 },
	{ "IRQ_CLEAR",		0x0018, 0 },
	{ "IRQ_STATUS",		0x001C, 0 },
	{ "STOP_REQ",		0x0024, 0 },
	{ "STOP_STATUS",	0x0028, 0 },
	{ "ENCODE_OUTPUT_SIZE",	0x0034, 0 },
	{ "FE_INPUT_FORMAT",	0x0038, 0 },
	{ "FE_DIMS",		0x003C, 0 },
	{ "FE_PIPELINE_MODE",	0x0040, 0 },
	{ "OP_ENCODE_MODE",	0x0044, 0 },
	{ "OP_GEOM[0]",		0x0048, 0 },
	{ "OP_GEOM[1]",		0x004C, 0 },
	{ "OP_GEOM[2]",		0x0050, 0 },
	{ "OP_GEOM[3]",		0x0054, 0 },
	{ "OP_FORMAT_MAGIC",	0x0058, 0 },
	{ "OP_MATRIX[0]",	0x005C, 0 },
	{ "OP_MATRIX[1]",	0x0060, 0 },
	{ "OP_MATRIX[2]",	0x0064, 0 },
	{ "OP_MATRIX[3]",	0x0068, 0 },
	{ "OP_MATRIX[4]",	0x006C, 0 },
	{ "OP_MATRIX[5]",	0x0070, 0 },
	{ "OP_MATRIX[6]",	0x0074, 0 },
	{ "OP_MATRIX[7]",	0x0078, 0 },
	{ "OP_MATRIX[8]",	0x007C, 0 },
	{ "FE_BUFFER_CFG",	0x0080, 0 },
	{ "FE_Y_PING_ADDR",	0x0084, 0 },
	{ "FE_Y_PONG_ADDR",	0x0088, 0 },
	{ "FE_CBCR_PING_ADDR",	0x008C, 0 },
	{ "FE_CBCR_PONG_ADDR",	0x0090, 0 },
	{ "FE_CMD",		0x0094, 0 },
	{ "WE_CFG",		0x0098, 0 },
	{ "WE_Y_THRESHOLD",	0x00C0, 0 },
	{ "WE_CBCR_THRESHOLD",	0x00C4, 0 },
	{ "WE_Y_PING_CFG",	0x00C8, 0 },
	{ "WE_Y_PONG_CFG",	0x00CC, 0 },
	{ "WE_Y_PING_ADDR",	0x00D8, 0 },
	{ "WE_Y_PONG_ADDR",	0x00DC, 0 },
	{ "WE_Y_UB_CFG",	0x00E8, 0 },
	{ "START_KICK",		0x00F0, 0 },
	{ "DRI_INTERVAL",	0x00F4, 0 },
	{ "FSC_COUNT",		0x0110, 0 },
	{ "FSC_THRESHOLD[0]",	0x0114, 0 },
	{ "FSC_THRESHOLD[1]",	0x0118, 0 },
	{ "FSC_THRESHOLD[2]",	0x011C, 0 },
	{ "FSC_THRESHOLD[3]",	0x0120, 0 },
	{ "TABLE_SEL",		0x0124, 0 },
	{ "TABLE_INDEX",	0x0128, 0 },
	/*
	 * TABLE_DATA is auto-incrementing — reading it advances the index.
	 * Skip in the tight poll, but include in snapshots (we'll restore
	 * TABLE_INDEX afterwards).
	 */
	{ "TABLE_DATA",		0x012C, 1 },
};

#define N_GEMINI_REGS (sizeof(gemini_regs) / sizeof(gemini_regs[0]))

/* MMCC registers most relevant to Gemini */
static const struct reg_def mmcc_regs[] = {
	{ "IJPEG_CC",		0x0098, 0 },	/* IJPEG core clock control */
	{ "IJPEG_NS_REG",	0x00A0, 0 },
	{ "IJPEG_CC_FS",	0x00A4, 0 },	/* fast-switch */
	{ "IJPEG_AHB_NS_REG",	0x0270, 0 },	/* AHB clock */
	{ "IJPEG_AXI_NS",	0x0048, 0 },
	{ "AHB_NS_REG",		0x0008, 0 },
	{ "AHB_EN_REG",		0x0008, 0 },
	{ "MMSS_RESET",		0x00A8, 0 },
	{ "MISC_CC2",		0x05A0, 0 },
	{ "MMSS_FABRIC_HALT",	0x002C, 0 },
};
#define N_MMCC_REGS (sizeof(mmcc_regs) / sizeof(mmcc_regs[0]))

static volatile uint32_t *gem_base;
static volatile uint32_t *mmcc_base;

/* Ring buffer of (timestamp, name, oldval, newval) tuples for delta log. */
struct delta_entry {
	struct timespec ts;
	const char *name;
	uint32_t off;
	uint32_t old_val;
	uint32_t new_val;
};
#define DELTA_RING_SIZE	(64 * 1024)
static struct delta_entry delta_ring[DELTA_RING_SIZE];
static volatile size_t delta_head;	/* writer */
static volatile size_t delta_tail;	/* reader */

static volatile sig_atomic_t stop_requested;

static void on_signal(int sig)
{
	(void)sig;
	stop_requested = 1;
}

static long long ns_diff(struct timespec *a, struct timespec *b)
{
	return (long long)(a->tv_sec - b->tv_sec) * 1000000000LL +
	       (a->tv_nsec - b->tv_nsec);
}

static uint32_t reg_read(volatile uint32_t *base, uint32_t off)
{
	return base[off / 4];
}

static int snapshot_gemini(const char *path, const char *label)
{
	FILE *f = fopen(path, "w");
	uint32_t i;

	if (!f) {
		fprintf(stderr, "snapshot %s: %s\n", path, strerror(errno));
		return -1;
	}

	fprintf(f, "# Gemini register snapshot — %s\n", label);
	fprintf(f, "# Physical base 0x%lx, size 0x%x\n",
		GEMINI_PHYS_BASE, GEMINI_REGION_SIZE);

	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	fprintf(f, "# Time mono %lld.%09ld\n\n",
		(long long)now.tv_sec, now.tv_nsec);

	fprintf(f, "## Named registers\n\n");
	for (i = 0; i < N_GEMINI_REGS; i++) {
		const struct reg_def *r = &gemini_regs[i];

		if (r->side_effects) {
			fprintf(f, "  %-22s 0x%04x = (skipped, side effects)\n",
				r->name, r->off);
			continue;
		}
		fprintf(f, "  %-22s 0x%04x = 0x%08x\n",
			r->name, r->off, reg_read(gem_base, r->off));
	}

	fprintf(f, "\n## MMCC clock controller (selected)\n\n");
	for (i = 0; i < N_MMCC_REGS; i++) {
		const struct reg_def *r = &mmcc_regs[i];

		fprintf(f, "  %-22s MMCC+0x%04x = 0x%08x\n",
			r->name, r->off, reg_read(mmcc_base, r->off));
	}

	fprintf(f, "\n## Full Gemini region dump (0x000..0xFFF, 4-byte words, "
		   "non-zero only)\n\n");
	for (i = 0; i < GEMINI_REGION_SIZE; i += 4) {
		uint32_t v = reg_read(gem_base, i);

		if (v)
			fprintf(f, "  0x%04x = 0x%08x\n", i, v);
	}

	fclose(f);
	fprintf(stderr, "[snapshot] %s -> %s\n", label, path);
	return 0;
}

static void delta_push(const struct reg_def *r, uint32_t old_val,
		       uint32_t new_val, struct timespec *ts)
{
	size_t head = delta_head;
	size_t next = (head + 1) % DELTA_RING_SIZE;

	if (next == delta_tail) {
		/* Ring full — drop the oldest entry. */
		delta_tail = (delta_tail + 1) % DELTA_RING_SIZE;
	}
	delta_ring[head].ts = *ts;
	delta_ring[head].name = r->name;
	delta_ring[head].off = r->off;
	delta_ring[head].old_val = old_val;
	delta_ring[head].new_val = new_val;
	delta_head = next;
}

static int delta_flush(FILE *f)
{
	int n = 0;

	while (delta_tail != delta_head) {
		struct delta_entry *e = &delta_ring[delta_tail];

		fprintf(f, "%lld.%09ld  %-22s 0x%04x  0x%08x -> 0x%08x\n",
			(long long)e->ts.tv_sec, e->ts.tv_nsec,
			e->name, e->off, e->old_val, e->new_val);
		delta_tail = (delta_tail + 1) % DELTA_RING_SIZE;
		n++;
	}
	return n;
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [-o OUTDIR]\n"
		"\n"
		"Maps the Gemini JPEG register region and the MMCC clock\n"
		"controller via /dev/mem, snapshots state at three sync\n"
		"points (before/during/after a camera capture), and logs\n"
		"every register-value change in between.\n"
		"\n"
		"Run as root. Take a photo with the Camera app while\n"
		"the poller is running. The poller exits automatically\n"
		"~500 ms after the encoder goes idle following activity.\n"
		"\n"
		"  -o OUTDIR  output directory (default /tmp)\n",
		argv0);
}

int main(int argc, char **argv)
{
	const char *outdir = "/tmp";
	int opt, fd, i;
	void *gm, *mc;
	uint32_t cached[N_GEMINI_REGS];
	struct timespec last_active;
	bool seen_activity = false;
	struct timespec start;
	char path[512];
	FILE *delta_f;

	while ((opt = getopt(argc, argv, "o:h")) != -1) {
		switch (opt) {
		case 'o':
			outdir = optarg;
			break;
		case 'h':
		default:
			usage(argv[0]);
			return opt == 'h' ? 0 : 1;
		}
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("open /dev/mem (run as root, also check kernel "
		       "STRICT_DEVMEM)");
		return 1;
	}

	gm = mmap(NULL, GEMINI_REGION_SIZE, PROT_READ | PROT_WRITE,
		  MAP_SHARED, fd, GEMINI_PHYS_BASE);
	if (gm == MAP_FAILED) {
		perror("mmap gemini");
		close(fd);
		return 1;
	}
	gem_base = (volatile uint32_t *)gm;

	mc = mmap(NULL, MMCC_REGION_SIZE, PROT_READ | PROT_WRITE,
		  MAP_SHARED, fd, MMCC_PHYS_BASE);
	if (mc == MAP_FAILED) {
		perror("mmap mmcc");
		munmap(gm, GEMINI_REGION_SIZE);
		close(fd);
		return 1;
	}
	mmcc_base = (volatile uint32_t *)mc;

	close(fd);

	fprintf(stderr, "[init] Gemini @ %p (phys 0x%lx)\n",
		(void *)gem_base, GEMINI_PHYS_BASE);
	fprintf(stderr, "[init] MMCC @ %p (phys 0x%lx)\n",
		(void *)mmcc_base, MMCC_PHYS_BASE);
	fprintf(stderr, "[init] HW_VERSION = 0x%08x\n",
		reg_read(gem_base, 0x0000));

	/* Snapshot 1: before */
	snprintf(path, sizeof(path), "%s/gemini_snapshot_before.txt", outdir);
	if (snapshot_gemini(path, "before") < 0)
		goto err;

	/* Open the delta log and prime the cache. */
	snprintf(path, sizeof(path), "%s/gemini_delta.log", outdir);
	delta_f = fopen(path, "w");
	if (!delta_f) {
		perror("fopen delta log");
		goto err;
	}
	fprintf(delta_f, "# format: <mono_ts>  <name>  <off>  <old> -> <new>\n");

	for (i = 0; i < (int)N_GEMINI_REGS; i++) {
		if (gemini_regs[i].side_effects)
			cached[i] = 0;
		else
			cached[i] = reg_read(gem_base, gemini_regs[i].off);
	}

	clock_gettime(CLOCK_MONOTONIC, &start);
	last_active = start;

	fprintf(stderr,
		"[poll] watching for camera activity. Take a photo now.\n");
	fprintf(stderr,
		"[poll] poller will exit ~500 ms after encoder goes idle.\n");

	bool snapshot_during_taken = false;

	while (!stop_requested) {
		struct timespec now;
		bool any_change = false;

		clock_gettime(CLOCK_MONOTONIC, &now);

		/* Abort if we've been running too long with no activity. */
		if (now.tv_sec - start.tv_sec > MAX_RUNTIME_S) {
			fprintf(stderr,
				"[poll] timeout (%d s) — exiting\n",
				MAX_RUNTIME_S);
			break;
		}

		for (i = 0; i < (int)N_GEMINI_REGS; i++) {
			const struct reg_def *r = &gemini_regs[i];
			uint32_t v;

			if (r->side_effects)
				continue;
			v = reg_read(gem_base, r->off);
			if (v != cached[i]) {
				delta_push(r, cached[i], v, &now);
				cached[i] = v;
				any_change = true;
			}
		}

		if (any_change) {
			seen_activity = true;
			last_active = now;

			if (!snapshot_during_taken) {
				/*
				 * First sign of activity → take the "during"
				 * snapshot immediately. Subsequent activity
				 * still gets logged in the delta file.
				 */
				snprintf(path, sizeof(path),
					 "%s/gemini_snapshot_during.txt",
					 outdir);
				snapshot_gemini(path, "during");
				snapshot_during_taken = true;
			}

			delta_flush(delta_f);
			fflush(delta_f);
		} else if (seen_activity &&
			   ns_diff(&now, &last_active) > IDLE_GRACE_NS) {
			break;
		}

		usleep(POLL_INTERVAL_US);
	}

	/* Final flush. */
	delta_flush(delta_f);
	fclose(delta_f);

	/* Snapshot 3: after */
	snprintf(path, sizeof(path), "%s/gemini_snapshot_after.txt", outdir);
	snapshot_gemini(path, "after");

	if (!seen_activity)
		fprintf(stderr,
			"[poll] WARNING: no activity seen — did you take a "
			"photo? Snapshots before/after will be identical.\n");

	munmap(mc, MMCC_REGION_SIZE);
	munmap(gm, GEMINI_REGION_SIZE);
	return 0;

err:
	munmap(mc, MMCC_REGION_SIZE);
	munmap(gm, GEMINI_REGION_SIZE);
	return 1;
}
