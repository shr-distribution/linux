/*
 * webos-adm-sampler — sample ADM channel state at high rate while a
 * concurrent eMMC read drives sustained traffic. Reveals:
 *
 *   - How often the CMD_PTR register updates (= how often legacy
 *     submits a new descriptor chain to the ADM controller).
 *   - Channel STATUS transitions (idle vs active windows).
 *   - RSLT register progression (= how often a chain completes).
 *   - Optionally walk the descriptor chain reachable via CMD_PTR to
 *     dump BOX descriptor layout.
 *
 * Build:
 *   /opt/PalmPDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc \
 *       -O2 -static -o webos-adm-sampler webos-adm-sampler.c
 *
 * Run on webOS:
 *   novacom put file:///tmp/webos-adm-sampler < webos-adm-sampler
 *   # Start the load in one shell:
 *   novacom run file:///bin/sh -c "dd if=/dev/mmcblk0 of=/dev/null bs=4M count=50 skip=2048 &
 *                                  /tmp/webos-adm-sampler > /tmp/sampler.txt
 *                                  wait"
 *   novacom get file:///tmp/sampler.txt > sampler.txt
 *
 * Sampling parameters (compile-time):
 *   SAMPLE_PERIOD_US = 200 us (5 kHz)
 *   SAMPLES          = 2500   (-> ~500 ms of sampling)
 *   CHANNEL          = 2      (sdcc1 eMMC) — also do 5 (sdcc4 WiFi)
 *
 * Output format: CSV-like, one line per sample with timestamp + reg values.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>

#define ADM1_PHYS	0x18420000UL
#define ADM_SIZE	0x10000UL	/* enough to cover EE=0 + EE=1 windows */

/*
 * On MSM8660/APQ8060 the SD_SIZE is 0x800. Different register classes
 * live in different EE windows on this silicon:
 *   - CMD_PTR/RSLT/STATUS/RSLT_CONF: EE=1 (SD_AARM), offsets 0x800-0xBxx
 *   - CH_CONF/CRCI_CTL: EE=0 ("hidden" master view), offsets 0x240/0x400
 *
 * Confirmed by reading EE=1 STATUS = 0x00000001 (CMD_PTR_RDY) for
 * HLOS channels, while EE=0 CH_CONF = bootloader values (0x000008D5
 * for ch2, 0x000008D6 for ch5, etc.).
 */
/* EE=1 (live AARM) — what the kernel writes/reads during operation */
#define ADM_CH_CMD_PTR(ch)	(0x800 + (ch) * 4)
#define ADM_CH_RSLT(ch)		(0x840 + (ch) * 4)
#define ADM_CH_FLUSH0(ch)	(0x880 + (ch) * 4)
#define ADM_CH_STATUS(ch)	(0xA00 + (ch) * 4)
#define ADM_CH_RSLT_CONF(ch)	(0xB00 + (ch) * 4)
#define ADM_SEC_DOMAIN_IRQ	(0xB80)
/* EE=0 (hidden, live for config) — bootloader-programmed channel/CRCI cfg */
#define ADM_CH_CONF(ch)		(0x240 + (ch) * 4)
#define ADM_CRCI_CTL(crci)	(0x400 + (crci) * 4)

/* Sampling config */
#define SAMPLE_PERIOD_US	200
#define SAMPLES			2500
#define CH_EMMC			2	/* sdcc1 */
#define CH_WIFI			5	/* sdcc4 */

static volatile uint8_t *adm_base;

static inline uint32_t r32(unsigned long off)
{
	return *(volatile uint32_t *)(adm_base + off);
}

static void busy_wait_us(long us)
{
	struct timespec t = { .tv_sec = 0, .tv_nsec = us * 1000 };
	nanosleep(&t, NULL);
}

int main(int argc, char **argv)
{
	(void)argc; (void)argv;
	int fd = open("/dev/mem", O_RDONLY | O_SYNC);
	if (fd < 0) { perror("open /dev/mem"); return 1; }

	adm_base = mmap(NULL, ADM_SIZE, PROT_READ, MAP_SHARED, fd,
			ADM1_PHYS);
	if (adm_base == MAP_FAILED) { perror("mmap"); return 1; }

	printf("# webos-adm-sampler: sampling ch2 (eMMC) and ch5 (WiFi)\n");
	printf("# Period=%d us, samples=%d, total=%d ms\n",
	       SAMPLE_PERIOD_US, SAMPLES,
	       (SAMPLE_PERIOD_US * SAMPLES) / 1000);
	printf("# Cols: sample_idx, time_us, ch2_cmd_ptr, ch2_rslt, ch2_status,"
	       " ch5_cmd_ptr, ch5_rslt, ch5_status, sd_irq_status\n");

	/*
	 * Track transitions:
	 *   - any change in cmd_ptr means a new descriptor was started
	 *   - rslt bit 31 (VALID) set + then cleared means an IRQ fired
	 *   - status bit 1 (VALID) tracks channel active vs idle
	 */
	uint32_t last_ch2_cmd = 0, last_ch5_cmd = 0;
	uint32_t ch2_cmd_changes = 0, ch5_cmd_changes = 0;
	uint32_t ch2_rslt_seen_valid = 0, ch5_rslt_seen_valid = 0;
	int ch2_was_active = 0, ch5_was_active = 0;
	uint32_t ch2_active_samples = 0, ch5_active_samples = 0;

	struct timespec t_start;
	clock_gettime(CLOCK_MONOTONIC, &t_start);

	for (int i = 0; i < SAMPLES; i++) {
		uint32_t ch2_cmd  = r32(ADM_CH_CMD_PTR(CH_EMMC));
		uint32_t ch2_rslt = r32(ADM_CH_RSLT(CH_EMMC));
		uint32_t ch2_st   = r32(ADM_CH_STATUS(CH_EMMC));
		uint32_t ch5_cmd  = r32(ADM_CH_CMD_PTR(CH_WIFI));
		uint32_t ch5_rslt = r32(ADM_CH_RSLT(CH_WIFI));
		uint32_t ch5_st   = r32(ADM_CH_STATUS(CH_WIFI));
		uint32_t irq_st   = r32(ADM_SEC_DOMAIN_IRQ);

		struct timespec t_now;
		clock_gettime(CLOCK_MONOTONIC, &t_now);
		long us = (t_now.tv_sec - t_start.tv_sec) * 1000000L +
			  (t_now.tv_nsec - t_start.tv_nsec) / 1000L;

		if (ch2_cmd != last_ch2_cmd) ch2_cmd_changes++;
		if (ch5_cmd != last_ch5_cmd) ch5_cmd_changes++;
		if (ch2_rslt & 0x80000000UL) ch2_rslt_seen_valid++;
		if (ch5_rslt & 0x80000000UL) ch5_rslt_seen_valid++;
		if (ch2_st & 0x2) ch2_active_samples++;
		if (ch5_st & 0x2) ch5_active_samples++;
		ch2_was_active = ch2_st & 0x2;
		ch5_was_active = ch5_st & 0x2;
		(void)ch2_was_active; (void)ch5_was_active;

		printf("%d,%ld,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x\n",
		       i, us, ch2_cmd, ch2_rslt, ch2_st,
		       ch5_cmd, ch5_rslt, ch5_st, irq_st);

		last_ch2_cmd = ch2_cmd;
		last_ch5_cmd = ch5_cmd;

		busy_wait_us(SAMPLE_PERIOD_US);
	}

	printf("\n# === summary ===\n");
	printf("# ch2 (eMMC): cmd_ptr changes = %u, rslt VALID seen = %u, active samples = %u/%d\n",
	       ch2_cmd_changes, ch2_rslt_seen_valid, ch2_active_samples, SAMPLES);
	printf("# ch5 (WiFi): cmd_ptr changes = %u, rslt VALID seen = %u, active samples = %u/%d\n",
	       ch5_cmd_changes, ch5_rslt_seen_valid, ch5_active_samples, SAMPLES);
	printf("# (each cmd_ptr change ~= one new descriptor chain submitted)\n");
	printf("# (each VALID seen ~= one chain completion result available)\n");

	munmap((void *)adm_base, ADM_SIZE);
	close(fd);
	return 0;
}
