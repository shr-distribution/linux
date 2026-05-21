/*
 * webos-deep-dump — extended state dump for the mainline-vs-legacy
 * "why does this work on legacy but not mainline" investigation.
 *
 * Dumps registers we haven't checked before:
 *   - ADM CI_CONF (channel-instance burst sizes)
 *   - ADM GP_CTL (general control)
 *   - SDCC MMCICLOCK live state
 *   - SDCC MMCIMASK0 / MMCIMASK1
 *   - SDCC MMCIDATATIMER
 *   - SDCC DPSM-related QCOM-specific bits (registers 0x40, 0x44, 0x48 if mapped)
 *   - GCC SDC1/SDC4_NS register values (clock control)
 *
 * Build: same as webos-live-dump.
 * Run while sustained eMMC + WiFi DMA is happening for best signal.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>

#define PAGE_SIZE 4096UL
#define PAGE_MASK (~(PAGE_SIZE - 1))

static int memfd = -1;

static uint32_t r32(uint32_t phys) {
	uint32_t pa = phys & PAGE_MASK;
	void *p = mmap(NULL, PAGE_SIZE, PROT_READ, MAP_SHARED, memfd, pa);
	uint32_t v = 0xdeadbeef;
	if (p != MAP_FAILED) {
		v = *(volatile uint32_t *)((char *)p + (phys - pa));
		munmap(p, PAGE_SIZE);
	}
	return v;
}

int main(void) {
	memfd = open("/dev/mem", O_RDONLY | O_SYNC);
	if (memfd < 0) { perror("open /dev/mem"); return 1; }

	puts("====================================================");
	puts("webOS deep state dump");
	puts("====================================================");

	puts("\n=== ADM1 CI_CONF (channel-instance burst config, EE=0 off 0x390) ===");
	puts("Bits per legacy mach/dma.h:");
	puts("  RANGE_END (24-31) | RANGE_START (16-23) | MAX_BURST (0-?)");
	for (int ci = 0; ci < 12; ci++) {
		uint32_t a = 0x18420000 + 0x390 + ci * 4;
		uint32_t v = r32(a);
		printf("  CI[%2d] @ 0x%08x = 0x%08x", ci, a, v);
		printf("  (range_end=%u range_start=%u max_burst=%u)\n",
		       (v >> 24) & 0xff, (v >> 16) & 0xff, v & 0xffff);
	}

	puts("\n=== ADM1 CI_CONF at EE=1 (off 0xB90) ===");
	for (int ci = 0; ci < 12; ci++) {
		uint32_t a = 0x18420000 + 0xB90 + ci * 4;
		uint32_t v = r32(a);
		if (v) printf("  CI[%2d] @ 0x%08x = 0x%08x\n", ci, a, v);
	}

	puts("\n=== ADM1 GP_CTL (general control, off 0x3D8) ===");
	uint32_t gp = r32(0x18420000 + 0x3D8);
	printf("  GP_CTL @ 0x%08x = 0x%08x  (LP_EN=%d LP_CNT=%u)\n",
	       0x184203D8, gp, !!(gp & (1 << 12)), (gp >> 8) & 0xf);

	puts("\n=== ADM0 CI_CONF EE=0 (for comparison) ===");
	for (int ci = 0; ci < 12; ci++) {
		uint32_t a = 0x18320000 + 0x390 + ci * 4;
		uint32_t v = r32(a);
		if (v) printf("  CI[%2d] @ 0x%08x = 0x%08x\n", ci, a, v);
	}

	puts("\n=== SDCC1 (eMMC, 0x12400000) deep registers ===");
	struct { const char *n; uint32_t o; } sdcc_regs[] = {
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
		{ "MMCICLEAR",      0x038 },
		{ "MMCIMASK0",      0x03C },
		{ "MMCIMASK1",      0x040 },
		{ "QCOM_MCI_FIFO_CNT(0x44)", 0x044 },
		{ "QCOM_MCI_VERSION(0x50)",  0x050 },
		{ "QCOM_MCI_BURST_CTRL(0x60)", 0x060 },
		{ "QCOM_unknown_0x70",         0x070 },
	};
	for (int i = 0; i < (int)(sizeof(sdcc_regs)/sizeof(sdcc_regs[0])); i++) {
		uint32_t a = 0x12400000 + sdcc_regs[i].o;
		printf("  SDCC1 %-26s @ 0x%08x = 0x%08x\n",
		       sdcc_regs[i].n, a, r32(a));
	}

	puts("\n=== SDCC4 (WiFi, 0x121C0000) same regs ===");
	for (int i = 0; i < (int)(sizeof(sdcc_regs)/sizeof(sdcc_regs[0])); i++) {
		uint32_t a = 0x121C0000 + sdcc_regs[i].o;
		printf("  SDCC4 %-26s @ 0x%08x = 0x%08x\n",
		       sdcc_regs[i].n, a, r32(a));
	}

	/*
	 * GCC (Global Clock Controller) for SDCC clocks.
	 * MSM8660 GCC is at 0x00900000.
	 * SDC1_NS at +0x282C, SDC1_MD at +0x2828
	 * SDC4_NS at +0x286C, SDC4_MD at +0x2868 (guessing -- legacy header values)
	 */
	puts("\n=== GCC SDC1/SDC4 clock regs (best-effort offsets) ===");
	struct { const char *n; uint32_t a; } gcc_regs[] = {
		{ "SDC1_MD (0x2828)",  0x00902828 },
		{ "SDC1_NS (0x282C)",  0x0090282C },
		{ "SDC4_MD (0x2868)",  0x00902868 },
		{ "SDC4_NS (0x286C)",  0x0090286C },
		{ "DFAB_NS_REG (~0xC8)",     0x009000C8 },
		{ "DFAB_CLK_CTL (~0xCC)",    0x009000CC },
		{ "DFAB_BRIDGE_NS (~0xE0)",  0x009000E0 },
		{ "AHB_NS_REG (0x3C00)",     0x00903C00 },
		{ "ADM1_NS (guess)",         0x009003C0 },
	};
	for (int i = 0; i < (int)(sizeof(gcc_regs)/sizeof(gcc_regs[0])); i++) {
		uint32_t v = r32(gcc_regs[i].a);
		printf("  GCC %-30s @ 0x%08x = 0x%08x%s\n",
		       gcc_regs[i].n, gcc_regs[i].a, v,
		       v == 0xdeadbeef ? "  (unmapped)" : "");
	}

	close(memfd);
	puts("\n=== Done ===");
	return 0;
}
