/*
 * gemini_full_poll — exhaustive Gemini JPEG register tracer.
 *
 * Polls every 4-byte word in the Gemini register region (0x000..0xFFF)
 * at ~10 kHz and logs every value transition. Designed to catch
 * undocumented registers that OPAL writes but mainline doesn't —
 * anything we missed at offsets not in the cross-vendor doc.
 *
 * Skips known side-effect-on-read registers (TABLE_DATA at 0x012C
 * auto-increments TABLE_INDEX).
 *
 * Build (cross):
 *   arm-linux-gnueabihf-gcc -O2 -static -Wall \
 *       gemini_full_poll.c -o gemini_full_poll
 *
 * Run on either webOS (via novacom) or LuneOS (via SSH):
 *   ./gemini_full_poll &
 *   # take a photo / run test_gemini
 *   # poller exits ~500 ms after activity stops
 *
 * Output (default in /tmp):
 *   gemini_full_snapshot_before.txt
 *   gemini_full_snapshot_during.txt
 *   gemini_full_snapshot_after.txt
 *   gemini_full_delta.log
 *
 * Diff approach: compare snapshot_during between webOS-OPAL and
 * LuneOS-mainline runs. Any line that appears in OPAL but not in
 * mainline is a register OPAL writes that we don't.
 *
 * Copyright (c) 2026 Herrie (herrie.org)
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
/*
 * Only poll bank 0 (0x000..0x1FF). The encoder mirrors all writes
 * across 8 banks (verified via OPAL trace), so polling additional
 * banks just generates noise. Polling only bank 0 also dramatically
 * reduces /dev/mem read pressure during a fast encode (1280x1024
 * runs in ~13 ms — we want most poll cycles to land within that
 * window).
 */
#define N_WORDS			(0x200 / 4)	/* 128 words = bank 0 only */

#define POLL_INTERVAL_US	100	/* ~10 kHz */
#define IDLE_GRACE_NS		500000000LL
#define MAX_RUNTIME_S		300

/*
 * Registers whose READ has side effects (auto-increment, clear-on-read,
 * etc.). Skip these in the poll loop. The Gemini IP has 8 mirrored
 * banks of 0x200 bytes each, so each side-effect register is mirrored
 * at 8 offsets (0x12C, 0x32C, 0x52C, 0x72C, 0x92C, 0xB2C, 0xD2C, 0xF2C
 * for TABLE_DATA).
 *
 * Reading a side-effect register can hang the AHB bus or auto-trigger
 * encoder state — we must skip them at every bank.
 */
static bool is_skip(uint32_t word_off)
{
	uint32_t off_in_bank = (word_off * 4) & 0x1FF;

	/* TABLE_DATA at 0x12C — auto-increments TABLE_INDEX. */
	if (off_in_bank == 0x12C)
		return true;

	/* Reserved / undocumented offsets that may have side effects.
	 * Be conservative and skip the ranges we have no documentation
	 * for: 0x100..0x10F, 0x130..0x13C, 0x140..0x1FF — anything
	 * outside the cross-vendor doc's named registers.
	 *
	 * Actually — that's too aggressive. The cross-vendor reg map
	 * mentions FSC at 0x110..0x120 (has read meaning), TABLE_SEL/
	 * INDEX at 0x124/0x128 (write-only mostly), TABLE_DATA at 0x12C
	 * (already skipped). 0x13C has been read repeatedly without
	 * issue. So only skip TABLE_DATA. */
	return false;
}

/*
 * Names for known registers — used in the delta log so it's readable
 * for the offsets we already understand. Anonymous offsets get
 * "0xNNN" as their name.
 */
struct named_reg {
	const char *name;
	uint32_t off;
};
static const struct named_reg named[] = {
	{ "HW_VERSION",		0x0000 },
	{ "RESET_CMD",		0x0004 },
	{ "PIPELINE_CFG",	0x0008 },
	{ "REALTIME_CMD",	0x000C },
	{ "0x0010",		0x0010 },
	{ "IRQ_MASK",		0x0014 },
	{ "IRQ_CLEAR",		0x0018 },
	{ "IRQ_STATUS",		0x001C },
	{ "STOP_REQ",		0x0024 },
	{ "STOP_STATUS",	0x0028 },
	{ "ENCODE_OUTPUT_SIZE",	0x0034 },
	{ "FE_INPUT_FORMAT",	0x0038 },
	{ "FE_DIMS",		0x003C },
	{ "FE_PIPELINE_MODE",	0x0040 },
	{ "OP_ENCODE_MODE",	0x0044 },
	{ "OP_GEOM[0]",		0x0048 },
	{ "OP_GEOM[1]",		0x004C },
	{ "OP_GEOM[2]",		0x0050 },
	{ "OP_GEOM[3]",		0x0054 },
	{ "OP_FORMAT_MAGIC",	0x0058 },
	{ "OP_MATRIX[0]",	0x005C },
	{ "OP_MATRIX[1]",	0x0060 },
	{ "OP_MATRIX[2]",	0x0064 },
	{ "OP_MATRIX[3]",	0x0068 },
	{ "OP_MATRIX[4]",	0x006C },
	{ "OP_MATRIX[5]",	0x0070 },
	{ "OP_MATRIX[6]",	0x0074 },
	{ "OP_MATRIX[7]",	0x0078 },
	{ "OP_MATRIX[8]",	0x007C },
	{ "FE_BUFFER_CFG",	0x0080 },
	{ "FE_Y_PING_ADDR",	0x0084 },
	{ "FE_Y_PONG_ADDR",	0x0088 },
	{ "FE_CBCR_PING_ADDR",	0x008C },
	{ "FE_CBCR_PONG_ADDR",	0x0090 },
	{ "FE_CMD",		0x0094 },
	{ "WE_CFG",		0x0098 },
	{ "WE_Y_THRESHOLD",	0x00C0 },
	{ "WE_CBCR_THRESHOLD",	0x00C4 },
	{ "WE_Y_PING_CFG",	0x00C8 },
	{ "WE_Y_PONG_CFG",	0x00CC },
	{ "WE_Y_PING_ADDR",	0x00D8 },
	{ "WE_Y_PONG_ADDR",	0x00DC },
	{ "WE_Y_UB_CFG",	0x00E8 },
	{ "START_KICK",		0x00F0 },
	{ "DRI_INTERVAL",	0x00F4 },
	{ "FSC_COUNT",		0x0110 },
	{ "FSC_THRESHOLD[0]",	0x0114 },
	{ "FSC_THRESHOLD[1]",	0x0118 },
	{ "FSC_THRESHOLD[2]",	0x011C },
	{ "FSC_THRESHOLD[3]",	0x0120 },
	{ "TABLE_SEL",		0x0124 },
	{ "TABLE_INDEX",	0x0128 },
	/* TABLE_DATA at 0x012C is in skip_offsets */
	{ "0x013C",		0x013C },
};
#define N_NAMED (sizeof(named) / sizeof(named[0]))

/* Lookup table for fast name lookup — indexed by word offset. */
static const char *name_for_off[N_WORDS];

static void build_name_table(void)
{
	uint32_t i;
	static char anon_buf[N_WORDS][8];

	for (i = 0; i < N_WORDS; i++) {
		snprintf(anon_buf[i], sizeof(anon_buf[i]), "0x%03x",
			 i * 4);
		name_for_off[i] = anon_buf[i];
	}
	for (i = 0; i < N_NAMED; i++)
		name_for_off[named[i].off / 4] = named[i].name;
}

static volatile uint32_t *gem_base;

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

static int snapshot_gemini(const char *path, const char *label)
{
	FILE *f = fopen(path, "w");
	uint32_t i;
	struct timespec now;

	if (!f) {
		fprintf(stderr, "snapshot %s: %s\n", path, strerror(errno));
		return -1;
	}

	clock_gettime(CLOCK_MONOTONIC, &now);
	fprintf(f, "# Gemini full register snapshot — %s\n", label);
	fprintf(f, "# Physical base 0x%lx, size 0x%x\n",
		GEMINI_PHYS_BASE, GEMINI_REGION_SIZE);
	fprintf(f, "# Time mono %lld.%09ld\n\n",
		(long long)now.tv_sec, now.tv_nsec);

	fprintf(f, "## All non-zero words (offset = value, name)\n\n");
	for (i = 0; i < N_WORDS; i++) {
		uint32_t v;

		if (is_skip(i))
			continue;
		v = gem_base[i];
		if (v)
			fprintf(f, "  0x%03x = 0x%08x  %s\n",
				i * 4, v, name_for_off[i]);
	}

	fclose(f);
	fprintf(stderr, "[snapshot] %s -> %s\n", label, path);
	return 0;
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [-o OUTDIR]\n"
		"\n"
		"Polls EVERY 4-byte word in the Gemini register region\n"
		"(0x000..0xFFF) and logs every value transition. Catches\n"
		"undocumented registers that OPAL writes but mainline\n"
		"doesn't.\n"
		"\n"
		"Snapshots before/during/after activity, plus a full\n"
		"delta log of every register change in between.\n"
		"\n"
		"Run as root. Take a photo with the Camera app (webOS) or\n"
		"run test_gemini (LuneOS) while the poller is running.\n"
		"Poller exits automatically ~500 ms after the encoder goes\n"
		"idle.\n"
		"\n"
		"  -o OUTDIR  output directory (default /tmp)\n",
		argv0);
}

int main(int argc, char **argv)
{
	const char *outdir = "/tmp";
	int opt, fd, i;
	void *gm;
	uint32_t cached[N_WORDS];
	struct timespec last_active, start;
	bool seen_activity = false;
	bool snapshot_during_taken = false;
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

	build_name_table();

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("open /dev/mem (run as root)");
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
	close(fd);

	fprintf(stderr, "[init] Gemini @ %p (phys 0x%lx, %u words)\n",
		(void *)gem_base, GEMINI_PHYS_BASE, N_WORDS);
	fprintf(stderr, "[init] HW_VERSION = 0x%08x\n", gem_base[0]);

	/* Snapshot 1: before */
	snprintf(path, sizeof(path), "%s/gemini_full_snapshot_before.txt",
		 outdir);
	if (snapshot_gemini(path, "before") < 0)
		goto err;

	/* Open delta log and prime the cache. */
	snprintf(path, sizeof(path), "%s/gemini_full_delta.log", outdir);
	delta_f = fopen(path, "w");
	if (!delta_f) {
		perror("fopen delta log");
		goto err;
	}
	fprintf(delta_f, "# format: <mono_ts>  <name>  <off>  <old> -> <new>\n");

	for (i = 0; i < (int)N_WORDS; i++) {
		if (is_skip(i)) {
			cached[i] = 0;
			continue;
		}
		cached[i] = gem_base[i];
	}

	clock_gettime(CLOCK_MONOTONIC, &start);
	last_active = start;

	fprintf(stderr,
		"[poll] watching ALL %u words. Trigger encoder activity now.\n",
		N_WORDS);

	while (!stop_requested) {
		struct timespec now;
		bool any_change = false;

		clock_gettime(CLOCK_MONOTONIC, &now);

		if (now.tv_sec - start.tv_sec > MAX_RUNTIME_S) {
			fprintf(stderr,
				"[poll] timeout (%d s) — exiting\n",
				MAX_RUNTIME_S);
			break;
		}

		for (i = 0; i < (int)N_WORDS; i++) {
			uint32_t v;

			if (is_skip(i))
				continue;
			v = gem_base[i];
			if (v != cached[i]) {
				fprintf(delta_f,
					"%lld.%09ld  %-22s 0x%03x  0x%08x -> 0x%08x\n",
					(long long)now.tv_sec, now.tv_nsec,
					name_for_off[i], i * 4,
					cached[i], v);
				cached[i] = v;
				any_change = true;
			}
		}

		if (any_change) {
			seen_activity = true;
			last_active = now;

			if (!snapshot_during_taken) {
				snprintf(path, sizeof(path),
					 "%s/gemini_full_snapshot_during.txt",
					 outdir);
				snapshot_gemini(path, "during");
				snapshot_during_taken = true;
			}
			fflush(delta_f);
		} else if (seen_activity &&
			   ns_diff(&now, &last_active) > IDLE_GRACE_NS) {
			break;
		}

		usleep(POLL_INTERVAL_US);
	}

	fclose(delta_f);

	snprintf(path, sizeof(path), "%s/gemini_full_snapshot_after.txt",
		 outdir);
	snapshot_gemini(path, "after");

	if (!seen_activity)
		fprintf(stderr,
			"[poll] WARNING: no activity seen — did you trigger "
			"the encoder?\n");

	munmap(gm, GEMINI_REGION_SIZE);
	return 0;

err:
	munmap(gm, GEMINI_REGION_SIZE);
	return 1;
}
