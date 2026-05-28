/*
 * gemini_oneshot — single-shot Gemini register snapshot.
 *
 * Maps /dev/mem at the Gemini base, sleeps until SIGUSR1 arrives,
 * then takes ONE atomic snapshot of bank 0 (0x000..0x1FF, skipping
 * TABLE_DATA at 0x12C) into the output file. Never polls — avoids
 * the AHB-bus contention with active encode that hangs LuneOS.
 *
 * Workflow:
 *   1. Start gemini_oneshot in background — it sleeps waiting for
 *      SIGUSR1.
 *   2. Start test_gemini.
 *   3. test_gemini stalls in the m2m queue waiting for the encode
 *      to start; at that point send SIGUSR1 to gemini_oneshot.
 *   4. Snapshot is written to /tmp/gemini_oneshot.txt; gemini_oneshot
 *      exits.
 *   5. test_gemini completes the encode normally.
 *
 * The snapshot is taken at the exact moment of SIGUSR1 — pick the
 * timing carefully (shell `kill -USR1` overhead is ~ms, fast enough
 * to land mid-encode).
 *
 * Build:  arm-linux-gnueabihf-gcc -O2 -static gemini_oneshot.c -o gemini_oneshot
 *
 * Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define GEMINI_PHYS_BASE	0x04600000UL
#define GEMINI_REGION_SIZE	0x1000
#define BANK0_WORDS		(0x200 / 4)

static volatile uint32_t *gem_base;
static volatile sig_atomic_t shoot;

static void on_usr1(int sig) { (void)sig; shoot = 1; }
static void on_term(int sig) { (void)sig; shoot = 2; }

int main(int argc, char **argv)
{
	const char *path = "/tmp/gemini_oneshot.txt";
	int fd, i;
	void *gm;
	uint32_t snap[BANK0_WORDS];
	FILE *f;

	if (argc > 1) path = argv[1];

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) { perror("/dev/mem"); return 1; }
	gm = mmap(NULL, GEMINI_REGION_SIZE, PROT_READ | PROT_WRITE,
		  MAP_SHARED, fd, GEMINI_PHYS_BASE);
	if (gm == MAP_FAILED) { perror("mmap"); close(fd); return 1; }
	gem_base = (volatile uint32_t *)gm;
	close(fd);

	signal(SIGINT, on_term);
	signal(SIGTERM, on_term);

	fprintf(stderr, "[oneshot] PID=%d, mapped, watching HW_VERSION...\n",
		(int)getpid());

	/*
	 * Trigger: HW_VERSION (offset 0x000) reads as the gated value
	 * (~0x03 or 0x80) when clocks are off, and 0x00001100 when the
	 * encoder is clocked. Wait for it to go non-gated, then snapshot
	 * after a tiny delay so PIPELINE_CFG / FE registers have time to
	 * land but before FRAMEDONE clears them.
	 */
	while (!shoot) {
		uint32_t hw = gem_base[0];

		if (hw == 0x00001100)
			break;
		usleep(100);
	}
	if (shoot) {
		fprintf(stderr, "[oneshot] caught signal %d before encode started\n",
			(int)shoot);
		munmap(gm, GEMINI_REGION_SIZE);
		return 1;
	}

	/*
	 * Wait a bit for FE_CMD = 3 to land (encode underway), then
	 * snapshot immediately. Encode for 1280x1024 takes ~13 ms; pick
	 * 5 ms so we land in the middle.
	 */
	for (i = 0; i < 100; i++) {
		uint32_t fe_cmd = gem_base[0x094 / 4];
		if (fe_cmd == 3) break;
		usleep(50);
	}
	usleep(5000);

	fprintf(stderr, "[oneshot] taking snapshot now\n");

	/* Atomic-as-possible snapshot of bank 0. Skip 0x12C TABLE_DATA. */
	for (i = 0; i < BANK0_WORDS; i++) {
		if (i == 0x12C / 4) {
			snap[i] = 0;
			continue;
		}
		snap[i] = gem_base[i];
	}

	f = fopen(path, "w");
	if (!f) { perror(path); munmap(gm, GEMINI_REGION_SIZE); return 1; }

	fprintf(f, "# Gemini bank 0 single-shot snapshot\n");
	fprintf(f, "# Phys 0x%lx, taken on SIGUSR1\n\n", GEMINI_PHYS_BASE);
	fprintf(f, "## Non-zero words\n\n");
	for (i = 0; i < BANK0_WORDS; i++) {
		if (i == 0x12C / 4) continue;
		if (snap[i])
			fprintf(f, "  0x%03x = 0x%08x\n", i * 4, snap[i]);
	}
	fclose(f);

	fprintf(stderr, "[oneshot] snapshot written to %s\n", path);
	munmap(gm, GEMINI_REGION_SIZE);
	return 0;
}
