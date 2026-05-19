/*
 * webos-live-dump — dump ADM/SDCC/clock state via /dev/mem on webOS.
 *
 * Build (Palm PDK toolchain):
 *   /opt/PalmPDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc \
 *       -O2 -static -o webos-live-dump webos-live-dump.c
 *
 * Deploy + run:
 *   novacom put file:///tmp/webos-live-dump < webos-live-dump
 *   novacom run file:///bin/sh -c "chmod +x /tmp/webos-live-dump && \
 *       /tmp/webos-live-dump > /tmp/webos-dump.txt"
 *   novacom get file:///tmp/webos-dump.txt > webos-dump.txt
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#define PAGE_SIZE 4096UL
#define PAGE_MASK (~(PAGE_SIZE - 1))

static int memfd = -1;

/* mmap a single page containing `phys`, return ptr to the byte at `phys`.
 * Caller should munmap with map_page_unmap(). */
struct map_ctx {
	void *page;
	off_t pa_page;
};

static int map_page(off_t phys, struct map_ctx *ctx)
{
	off_t pa = phys & PAGE_MASK;
	void *p;

	p = mmap(NULL, PAGE_SIZE, PROT_READ, MAP_SHARED, memfd, pa);
	if (p == MAP_FAILED) {
		return -errno;
	}
	ctx->page = p;
	ctx->pa_page = pa;
	return 0;
}

static void map_page_unmap(struct map_ctx *ctx)
{
	if (ctx->page) {
		munmap(ctx->page, PAGE_SIZE);
		ctx->page = NULL;
	}
}

static uint32_t read32_at(off_t phys)
{
	struct map_ctx ctx = { 0 };
	uint32_t val = 0xdeadbeef;
	if (map_page(phys, &ctx) == 0) {
		val = *(volatile uint32_t *)((uint8_t *)ctx.page + (phys - ctx.pa_page));
		map_page_unmap(&ctx);
	}
	return val;
}

static void dump_range(const char *label, off_t base, int n, int stride,
		       const char *unit, off_t format_base)
{
	int i;
	printf("\n----- %s (base=0x%lX, %d entries, stride=%d) -----\n",
	       label, (unsigned long)base, n, stride);
	for (i = 0; i < n; i++) {
		off_t phys = base + (off_t)i * stride;
		uint32_t v = read32_at(phys);
		printf("  %s[%2d] @ 0x%08lX = 0x%08X\n", unit, i,
		       (unsigned long)phys, v);
		(void)format_base;
	}
}

int main(int argc, char **argv)
{
	(void)argc; (void)argv;

	printf("==================================================================\n");
	printf("webOS live ADM/SDCC dump\n");
	printf("==================================================================\n");
	fflush(stdout);

	memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		fprintf(stderr, "open /dev/mem: %s\n", strerror(errno));
		return 1;
	}

	/*
	 * ADM1 (Tenderloin's eMMC + SDIO ADM):
	 *   PHYS base = 0x18420000
	 *   CH_CONF      live (legacy SD_MASTER, = mainline EE=1):  base + 0xA40 + ch*4
	 *   CH_CONF      alt EE=0:                                    base + 0x240 + ch*4
	 *   CH_RSLT_CONF live (EE=1):                                 base + 0xB00 + ch*4
	 *   CRCI_CTL     live (EE=1):                                 base + 0xC00 + crci*4
	 *   CRCI_CTL     alt EE=0:                                    base + 0x400 + crci*4
	 *   STATUS       live (EE=1):                                 base + 0xA00 + ch*4
	 *
	 * 4 EE windows: EE=0 at +0, EE=1 at +0x800, EE=2 at +0x1000, EE=3 at +0x1800.
	 * Reading all four for the same channel shows where bootloader programmed.
	 *
	 * Channel/CRCI mapping on Tenderloin (from legacy webOS source):
	 *   sdcc1 (eMMC) : channel 2,  CRCI 1
	 *   sdcc4 (WiFi) : channel 5,  CRCI 5
	 *   QCE crypto   : (varies; CRCI 4 = CE_IN, 5 = CE_OUT on some configs)
	 *   ch10-15      : reserved for modem (SD=3 / TrustZone) — likely read 0 from HLOS
	 */
	dump_range("ADM1 CH_CONF live  (EE=1, off 0xA40)",  0x18420A40, 16, 4, "ch",   0);
	dump_range("ADM1 CH_CONF alt   (EE=0, off 0x240)",  0x18420240, 16, 4, "ch",   0);
	dump_range("ADM1 CH_CONF EE=2  (off 0x1240)",       0x18421240, 16, 4, "ch",   0);
	dump_range("ADM1 CH_CONF EE=3  (off 0x1A40)",       0x18421A40, 16, 4, "ch",   0);

	dump_range("ADM1 CH_RSLT_CONF live (EE=1, off 0xB00)", 0x18420B00, 16, 4, "ch",   0);
	dump_range("ADM1 CH_RSLT_CONF alt  (EE=0, off 0x300)", 0x18420300, 16, 4, "ch",   0);

	dump_range("ADM1 CH_STATUS live (EE=1, off 0xA00)",    0x18420A00, 16, 4, "ch",   0);

	dump_range("ADM1 CRCI_CTL live  (EE=1, off 0xC00)",    0x18420C00, 16, 4, "crci", 0);
	dump_range("ADM1 CRCI_CTL alt   (EE=0, off 0x400)",    0x18420400, 16, 4, "crci", 0);
	dump_range("ADM1 CRCI_CTL EE=2  (off 0x1400)",         0x18421400, 16, 4, "crci", 0);
	dump_range("ADM1 CRCI_CTL EE=3  (off 0x1C00)",         0x18421C00, 16, 4, "crci", 0);

	/* CI_CONF (channel-instance config) - 12 CI entries, EE=1 at 0xB90 */
	dump_range("ADM1 CI_CONF live (EE=1, off 0xB90)",      0x18420B90, 12, 4, "ci",   0);
	dump_range("ADM1 CI_CONF alt  (EE=0, off 0x390)",      0x18420390, 12, 4, "ci",   0);

	/*
	 * ADM0 (crypto + audio path on Tenderloin):
	 *   PHYS base = 0x18320000
	 *   Same layout.
	 */
	dump_range("ADM0 CH_CONF live (EE=1, off 0xA40)",      0x18320A40, 16, 4, "ch",   0);
	dump_range("ADM0 CH_CONF alt  (EE=0, off 0x240)",      0x18320240, 16, 4, "ch",   0);
	dump_range("ADM0 CRCI_CTL live (EE=1, off 0xC00)",     0x18320C00, 16, 4, "crci", 0);
	dump_range("ADM0 CRCI_CTL alt  (EE=0, off 0x400)",     0x18320400, 16, 4, "crci", 0);

	/*
	 * SDCC PL18x register block per controller.
	 *   sdcc1 @ 0x12400000 (eMMC)
	 *   sdcc4 @ 0x121C0000 (WiFi)
	 */
	struct sdcc_def { const char *name; off_t base; };
	struct sdcc_def sdccs[] = {
		{ "SDCC1 (eMMC)",  0x12400000 },
		{ "SDCC4 (WiFi)",  0x121C0000 },
	};
	struct sdcc_reg { const char *name; off_t off; };
	struct sdcc_reg sregs[] = {
		{ "MMCIPOWER",      0x000 },
		{ "MMCICLOCK",      0x004 },
		{ "MMCIARG",        0x008 },
		{ "MMCICMD",        0x00C },
		{ "MMCIRESPCMD",    0x010 },
		{ "MMCIRESPONSE0",  0x014 },
		{ "MMCIDATATIMER",  0x024 },
		{ "MMCIDATALENGTH", 0x028 },
		{ "MMCIDATACTRL",   0x02C },
		{ "MMCIDATACNT",    0x030 },
		{ "MMCISTATUS",     0x034 },
		{ "MMCIMASK0",      0x03C },
		{ "MMCIMASK1",      0x040 },
		{ "QCOM MCI_FIFO_CNT(0x44)", 0x044 },
	};
	int si, ri;
	for (si = 0; si < (int)(sizeof(sdccs)/sizeof(sdccs[0])); si++) {
		printf("\n----- %s @ 0x%08lX -----\n",
		       sdccs[si].name, (unsigned long)sdccs[si].base);
		for (ri = 0; ri < (int)(sizeof(sregs)/sizeof(sregs[0])); ri++) {
			off_t phys = sdccs[si].base + sregs[ri].off;
			printf("  %-25s @ 0x%08lX = 0x%08X\n",
			       sregs[ri].name, (unsigned long)phys,
			       read32_at(phys));
		}
	}

	/*
	 * AR6003 SDIO function 1 mailbox status (HOST_INT_STATUS_ADDRESS=0x400).
	 * Reading this via /dev/mem isn't directly possible -- it lives on the
	 * SDIO bus, not the AHB. Skip from here; ath6kl would have to dump it.
	 */
	printf("\n----- AR6003 chip side -----\n");
	printf("  (skipped -- chip-side registers live on SDIO bus, can't reach via /dev/mem)\n");

	/*
	 * Clock controller (MSM_CLK_CTL @ 0x00900000 on MSM8660):
	 *   SDC1_NS = base + 0x0282
	 *   SDC1_MD = base + 0x0284
	 *   SDC4_NS, etc.
	 *   dfab_clk_ns / dfab_aclk_ns nearby
	 * We probe a few candidate offsets — anything that reads 0xdeadbeef
	 * means it isn't actually mapped at HLOS EE on this SoC.
	 */
	struct cc_reg { const char *name; off_t phys; };
	struct cc_reg ccregs[] = {
		/* MSM8660 GLB CLK Branch registers — best-effort guesses */
		{ "SDC1_NS_REG (guess)",         0x00903E00 },
		{ "SDC1_MD_REG (guess)",         0x00903E04 },
		{ "SDC4_NS_REG (guess)",         0x00903E40 },
		{ "SDC4_MD_REG (guess)",         0x00903E44 },
		{ "DFAB_CLK_NS (guess)",         0x009000C8 },
		{ "DFAB_CLK_CTL (guess)",        0x009000CC },
		{ "AFAB_CLK_NS (guess)",         0x009000E0 },
		{ "AHB_NS_REG (guess)",          0x00903C00 },
	};
	printf("\n----- Clock-controller-ish probes (best-effort) -----\n");
	int ci;
	for (ci = 0; ci < (int)(sizeof(ccregs)/sizeof(ccregs[0])); ci++) {
		uint32_t v = read32_at(ccregs[ci].phys);
		printf("  %-30s @ 0x%08lX = 0x%08X%s\n",
		       ccregs[ci].name, (unsigned long)ccregs[ci].phys, v,
		       v == 0xdeadbeef ? "  (unmapped / EFAULT)" : "");
	}

	close(memfd);
	printf("\n==================================================================\n");
	printf("Done.\n");
	printf("==================================================================\n");
	return 0;
}
