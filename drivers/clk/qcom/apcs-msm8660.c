// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm APQ8060/MSM8660 APCS (Application Processor Clock System) driver
 *
 * Copyright (c) 2024, The Linux Foundation. All rights reserved.
 *
 * This driver provides the CPU clock for Scorpion-based Qualcomm SoCs.
 * It manages the clock source mux and SCPLL to provide dynamic CPU
 * frequency scaling support via cpufreq-dt.
 *
 * Clock sources:
 *   - AFAB (27 MHz) - used during power collapse
 *   - PLL_8 (384 MHz) - intermediate frequency
 *   - SCPLL (432-1512+ MHz) - high performance
 *
 * Based on webOS kernel acpuclock-8x60.c by Code Aurora Forum.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpufreq.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interconnect.h>

/*
 * Register offsets from ACC base
 * Each CPU has its own ACC at different base addresses
 */
/*
 * Per-CPU clock control / source-select registers in the ACC region.
 * Offsets verified against Palm's downstream acpuclock-8x60.c:
 *   #define SPSS0_CLK_CTL_ADDR  (MSM_ACC0_BASE + 0x04)
 *   #define SPSS0_CLK_SEL_ADDR  (MSM_ACC0_BASE + 0x08)
 * The previous values (0x00, 0x04) were off by 4 bytes and pointed
 * select_clk_source() at the divider register instead of the source
 * select register. That helper is currently unreachable from the
 * active set_rate path so this never produced a visible failure, but
 * fixing the constants before we wire up the low-freq PLL_8 source
 * switch (TODO at apcs_cpu_clk_set_rate, rate <= FREQ_PLL8).
 */
#define SPSS_CLK_CTL		0x04
#define SPSS_CLK_SEL		0x08

/*
 * NOTE: the L2 clock-source select register is *not* at ACC base + 0x38.
 * Palm's downstream acpuclock-8x60.c places it at MSM_GCC_BASE + 0x38
 * (physical 0x02082038), an entirely different functional block from
 * the per-CPU ACC region (CPU0 ACC = 0x02041000, CPU1 ACC = 0x02051000).
 * The previous quiesced stop_machine() path here mapped acc_base + 0x38
 * (0x02041038) and wrote into the wrong register; it never actually
 * touched the L2 source mux. The HP lk bootloader leaves L2 already
 * tracking SCPLL (Palm's L2-locked-to-CPU-SCPLL design), so the entire
 * mux-flip sequence was both broken *and* unnecessary, while still
 * exposing a stop_machine deadlock window vs uncached bus masters
 * (msm-dma, USB host, MMC, GPU, RPM firmware). That window is the
 * cause of the intermittent very-early-boot hangs we observed on
 * Topaz3G PVT (boot frozen between t=0.531s and the next printk, fb
 * still painting the Tux logo). See commit log for the analysis.
 */

/*
 * Source-select values used in the per-bank acpuclk_src_sel field of
 * SPSS_CLK_CTL (bits [4..7] for bank 0, [12..15] for bank 1) when the
 * core mux output is taken from the PLL divider mux.
 *
 * AFAB here is the Application Fabric clock at ~310.5 MHz (this is
 * Palm's "MAX_AXI" rate, 310500 KHz, which is also the freq HP lk
 * hands the kernel running at). NOT the 27 MHz TCXO (PXO) the previous
 * comment claimed -- PXO never reaches the CPU mux directly.
 *
 * SCPLL is selected via CLK_SEL[2:1] (the core_src_sel field), not via
 * this acpuclk_src_sel field; the value defined here is informational.
 */
#define SRC_SEL_AFAB		1	/* AFAB ~310.5 MHz */
#define SRC_SEL_PLL8		3	/* PLL_8 384 MHz */
#define SRC_SEL_SCPLL		1	/* (core_src_sel value, not acpuclk_src_sel) */

/*
 * SPSS_CLK_SEL register layout (per uber-kernel select_core_source /
 * select_clk_source_div):
 *
 *   bit 0      : active PLL-divider-mux bank (0 or 1)
 *   bits[2:1]  : core_src_sel -- selects what the CPU clock takes:
 *                  0 = output of the PLL divider mux (bank-selected)
 *                  1 = SCPLL direct
 *
 * SPSS_CLK_CTL register layout (per-bank, 8 bits each):
 *
 *   bank 0: bits[3:0] = src_div, bits[7:4] = src_sel
 *   bank 1: bits[11:8] = src_div, bits[15:12] = src_sel
 *
 *   src_sel = acpuclk_src_sel field (1=AFAB, 3=PLL_8)
 *   src_div = (N - 1) where N is the divider. div=0 -> /1, div=1 -> /2.
 *
 * Switching the mux to a new (src,div) is done glitch-free by writing
 * the *inactive* bank in SPSS_CLK_CTL, then flipping the active-bank
 * bit in SPSS_CLK_SEL with a single 32-bit write.
 */
#define CLK_SEL_BANK_BIT		BIT(0)
#define CLK_SEL_CORE_SRC_SHIFT		1
#define CLK_SEL_CORE_SRC_MASK		(0x3 << CLK_SEL_CORE_SRC_SHIFT)
#define CORE_SRC_MUX			0	/* CLK_SEL[2:1] = 0b00 */
#define CORE_SRC_SCPLL			1	/* CLK_SEL[2:1] = 0b01 */
#define CLK_CTL_BANK_FIELD_WIDTH	8	/* bits per bank in CLK_CTL */
#define CLK_CTL_SRC_SEL_SHIFT		4	/* within a bank field */
#define CLK_CTL_SRC_DIV_SHIFT		0	/* within a bank field */

/*
 * SCPLL register offsets
 */
#define SCPLL_DEBUG		0x00
#define SCPLL_CTL		0x04
#define SCPLL_CAL		0x08
#define SCPLL_STATUS		0x10
#define SCPLL_FSM_CTL_EXT	0x24
#define SCPLL_LUT(l_val)	(0x38 + (((l_val) / 4) * 4))

/* SCPLL debug modes */
#define SCPLL_DEBUG_NONE	0x0
#define SCPLL_DEBUG_FULL	0x7

/* SCPLL modes */
#define SCPLL_POWER_DOWN	0x0
#define SCPLL_BYPASS		0x1
#define SCPLL_STANDBY		0x2
#define SCPLL_FULL_CAL		0x4
#define SCPLL_HALF_CAL		0x5
#define SCPLL_STEP_CAL		0x6
#define SCPLL_NORMAL		0x7

/* FSM switch modes */
#define SCPLL_SHOT_SWITCH	4
#define SCPLL_HOP_SWITCH	5
#define SCPLL_SIMPLE_SLEW	6
#define SCPLL_COMPLEX_SLEW	7

/* L_VAL limits: freq = 2 * 27MHz * L_VAL */
#define SCPLL_L_VAL_MIN		0x08	/* 432 MHz */
#define SCPLL_L_VAL_MAX		0x22	/* 1836 MHz (uber-kernel overclock ceiling) */
#define SCPLL_RATE_FACTOR	54000000UL

/* Frequency thresholds */
#define FREQ_PLL8		384000000UL
#define FREQ_SCPLL_MIN		432000000UL
#define FREQ_SCPLL_MAX		1836000000UL

/* Maximum regulator voltage for vdd_mem/vdd_dig (PM8058 S0/S1) */
#define VDD_MEM_MAX		1350000
#define VDD_DIG_MAX		1350000

/*
 * vdd_mem and vdd_dig voltage requirements per CPU frequency.
 * Derived from legacy webOS acpuclock-8x60.c.
 * Indexed by (L_VAL - SCPLL_L_VAL_MIN).
 */
struct vdd_req {
	unsigned int vdd_mem;	/* PM8058 S0 target voltage in uV */
	unsigned int vdd_dig;	/* PM8058 S1 target voltage in uV */
};

#define VDD_MEM_PLL8		1100000
#define VDD_DIG_PLL8		1000000

static const struct vdd_req vdd_table[] = {
	[0x00] = { 1100000, 1000000 },	/* L_VAL 0x08: 432 MHz */
	[0x01] = { 1100000, 1000000 },	/* L_VAL 0x09: 486 MHz */
	[0x02] = { 1100000, 1000000 },	/* L_VAL 0x0A: 540 MHz */
	[0x03] = { 1100000, 1000000 },	/* L_VAL 0x0B: 594 MHz */
	[0x04] = { 1100000, 1100000 },	/* L_VAL 0x0C: 648 MHz */
	[0x05] = { 1100000, 1100000 },	/* L_VAL 0x0D: 702 MHz */
	[0x06] = { 1100000, 1100000 },	/* L_VAL 0x0E: 756 MHz */
	[0x07] = { 1100000, 1100000 },	/* L_VAL 0x0F: 810 MHz */
	[0x08] = { 1100000, 1100000 },	/* L_VAL 0x10: 864 MHz */
	[0x09] = { 1100000, 1100000 },	/* L_VAL 0x11: 918 MHz */
	[0x0A] = { 1100000, 1100000 },	/* L_VAL 0x12: 972 MHz */
	[0x0B] = { 1125000, 1100000 },	/* L_VAL 0x13: 1026 MHz */
	[0x0C] = { 1200000, 1100000 },	/* L_VAL 0x14: 1080 MHz */
	[0x0D] = { 1200000, 1100000 },	/* L_VAL 0x15: 1134 MHz */
	[0x0E] = { 1200000, 1200000 },	/* L_VAL 0x16: 1188 MHz */
	[0x0F] = { 1250000, 1200000 },	/* L_VAL 0x17: 1242 MHz */
	[0x10] = { 1250000, 1200000 },	/* L_VAL 0x18: 1296 MHz */
	[0x11] = { 1250000, 1200000 },	/* L_VAL 0x19: 1350 MHz */
	[0x12] = { 1250000, 1200000 },	/* L_VAL 0x1A: 1404 MHz */
	[0x13] = { 1250000, 1200000 },	/* L_VAL 0x1B: 1458 MHz */
	[0x14] = { 1250000, 1300000 },	/* L_VAL 0x1C: 1512 MHz */
	/*
	 * Overclocked OPPs above webOS / vendor-validated ceiling.
	 * Values from rwhitby/webos-uber-kernel — these are the
	 * vdd_mem/vdd_dig (PM8058 S0/S1) RPM-rail votes, NOT cpu
	 * Vcore. CPU Vcore comes from the SAW regulator
	 * (cpu0_vdd/cpu1_vdd) via the DT opp-microvolt fields and is
	 * driven by cpufreq-dt.
	 */
	[0x15] = { 1300000, 1300000 },	/* L_VAL 0x1D: 1566 MHz */
	[0x16] = { 1300000, 1300000 },	/* L_VAL 0x1E: 1620 MHz */
	[0x17] = { 1300000, 1300000 },	/* L_VAL 0x1F: 1674 MHz */
	[0x18] = { 1300000, 1300000 },	/* L_VAL 0x20: 1728 MHz */
	[0x19] = { 1300000, 1300000 },	/* L_VAL 0x21: 1782 MHz */
	[0x1A] = { 1300000, 1300000 },	/* L_VAL 0x22: 1836 MHz */
};

static const struct vdd_req *get_vdd_req(u32 l_val)
{
	unsigned int idx;

	if (l_val < SCPLL_L_VAL_MIN)
		return NULL;

	idx = l_val - SCPLL_L_VAL_MIN;
	if (idx >= ARRAY_SIZE(vdd_table))
		return &vdd_table[ARRAY_SIZE(vdd_table) - 1];

	return &vdd_table[idx];
}

/*
 * L2 cache SCPLL frequency scaling.
 * The L2 cache has its own SCPLL (same register layout as CPU SCPLLs).
 *
 * L2 frequency is locked in step with the CPU L_VAL so the CPU's
 * existing vdd_mem/vdd_dig vote always covers L2's voltage needs.
 * This matches the mainline Qualcomm pattern (Krait/Kryo drivers do
 * no L2-specific voltage voting and rely on shared CPU/L2 rails
 * being voted by the CPU path).
 *
 * The previous 3-tier mapping (LOW/MID/HIGH) jumped L2 to 1404 MHz
 * once CPU reached 1080 MHz, but the CPU vdd vote at 1080 MHz is
 * vdd_mem=1.2V/vdd_dig=1.1V — insufficient for L2 at 1404 MHz which
 * needs 1.25V/1.2V per the legacy L2 freq table. That left L2
 * undervolted in the 1080–1134 MHz CPU range.
 *
 * L2 is capped at L_VAL 0x1A (1404 MHz) because legacy validated no
 * L2 rate above that (CPU at 1512 MHz uses L2 1404 MHz per legacy
 * acpu_freq_tbl_v2[16]).
 */
#define L2_L_VAL_MAX	0x1A	/* 1404 MHz, legacy-validated L2 ceiling */

/**
 * struct apcs_cpu_clk - Per-CPU clock structure
 * @hw: clk_hw for clock framework
 * @acc_base: ACC (Application Core Cluster) register base
 * @scpll_base: SCPLL register base for this CPU
 * @vdd_mem: memory voltage regulator (PM8058 S0), optional
 * @vdd_dig: digital core voltage regulator (PM8058 S1), optional
 * @icc_cpu_mem: CPU → EBI (DDR) interconnect path, optional
 * @current_rate: current CPU frequency
 * @current_src: current core source select bits [2:1] of SPSS_CLK_SEL
 *               (CORE_SRC_MUX = 0, CORE_SRC_SCPLL = 1). Tracks where the
 *               CPU is actually taking its clock from. Distinct from the
 *               per-bank `acpuclk_src_sel` (SRC_SEL_AFAB / SRC_SEL_PLL8)
 *               which only matters when current_src == CORE_SRC_MUX.
 * @current_bw_kbps: current CPU → EBI bandwidth vote, in kBps
 * @calibrated: true if SCPLL has been calibrated
 * @lock: spinlock for register access
 */
struct apcs_cpu_clk {
	struct clk_hw hw;
	void __iomem *acc_base;
	void __iomem *scpll_base;
	struct regulator *vdd_mem;
	struct regulator *vdd_dig;
	struct icc_path *icc_cpu_mem;
	unsigned long current_rate;
	int current_src;
	unsigned int current_bw_kbps;
	bool calibrated;
	spinlock_t lock;
};

#define to_apcs_cpu_clk(_hw) container_of(_hw, struct apcs_cpu_clk, hw)

/*
 * CPU → EBI bandwidth tiers (peak ib, in kBps).
 *
 * Tiers 0-3 match legacy webOS acpuclock-8x60.c bw_level_tbl[]. RPM
 * translates the request into an EBI clock rate (the bus controller
 * picks a rate that delivers at least the requested ib).
 *
 * Tier 4 extends the table for the 1512 MHz OPP that we support but
 * webOS did not. Legacy capped at 1188 MHz so the original table
 * stopped at tier 3 (310 MHz EBI). On hardware the bootloader scales
 * ebi1_clk roughly as CPU_top_freq / 4 — empirically 384 MHz at 1512
 * MHz CPU vs 310 MHz at 1188 MHz CPU (3.94x and 3.83x respectively).
 * Tier 4 requests an ib that maps to ~378 MHz EBI so apcs's vote
 * matches the bootloader's setting rather than under-voting from
 * tier 3's webOS-era ceiling.
 *
 * Tier 5 extends further for the overclocked OPPs at 1728 MHz and
 * 1836 MHz (rwhitby/webos-uber-kernel values). EBI scales roughly as
 * CPU_freq/4 on this platform so 1728 MHz → ~432 MHz EBI and 1836
 * MHz → ~459 MHz EBI. We use a single tier 5 covering both with a
 * bandwidth target of 3680 MB/s, equivalent to ~460 MHz EBI.
 *
 * Mapping from CPU frequency to bw tier (per legacy acpu_freq_tbl_v2,
 * extended for ours):
 *   CPU <  648 MHz   → tier 0 (824 MB/s,  ~103 MHz EBI)
 *   CPU 648–864 MHz  → tier 1 (1336 MB/s, ~167 MHz EBI)
 *   CPU 918–1134 MHz → tier 2 (2008 MB/s, ~251 MHz EBI)
 *   CPU 1188–1458    → tier 3 (2480 MB/s, ~310 MHz EBI)
 *   CPU 1512 MHz     → tier 4 (3024 MB/s, ~378 MHz EBI)
 *   CPU ≥ 1728 MHz   → tier 5 (3680 MB/s, ~460 MHz EBI)
 */
#define APCS_BW_KBPS_TIER0	 824000U
#define APCS_BW_KBPS_TIER1	1336000U
#define APCS_BW_KBPS_TIER2	2008000U
#define APCS_BW_KBPS_TIER3	2480000U
#define APCS_BW_KBPS_TIER4	3024000U
#define APCS_BW_KBPS_TIER5	3680000U

static unsigned int apcs_bw_for_rate(unsigned long rate_hz)
{
	unsigned long rate_khz = rate_hz / 1000;

	if (rate_khz >= 1728000)
		return APCS_BW_KBPS_TIER5;
	if (rate_khz >= 1512000)
		return APCS_BW_KBPS_TIER4;
	if (rate_khz >= 1188000)
		return APCS_BW_KBPS_TIER3;
	if (rate_khz >= 918000)
		return APCS_BW_KBPS_TIER2;
	if (rate_khz >= 648000)
		return APCS_BW_KBPS_TIER1;
	return APCS_BW_KBPS_TIER0;
}

static int apcs_set_bus_bw(struct apcs_cpu_clk *c, unsigned int bw_kbps)
{
	int ret;

	if (!c->icc_cpu_mem || bw_kbps == c->current_bw_kbps)
		return 0;

	ret = icc_set_bw(c->icc_cpu_mem, 0, bw_kbps);
	if (ret)
		return ret;

	c->current_bw_kbps = bw_kbps;
	return 0;
}

/* L2 SCPLL shared state — initialized by first CPU to probe */
static void __iomem *l2_scpll_base;
static DEFINE_SPINLOCK(l2_lock);
static u32 l2_current_l_val;
static bool l2_initialized;
static u32 l2_vote[NR_CPUS];

/*
 * Calibrate the SCPLL for the full frequency range.
 * This must be done once before the PLL can be used.
 */
static int scpll_calibrate(void __iomem *base)
{
	u32 regval;
	int timeout;

	pr_debug("SCPLL: Starting calibration\n");

	/* Clear calibration LUT register at max frequency.
	 * LUT registers are only writable in debug mode.
	 */
	writel_relaxed(SCPLL_DEBUG_FULL, base + SCPLL_DEBUG);
	writel_relaxed(0x0, base + SCPLL_LUT(SCPLL_L_VAL_MAX));
	writel_relaxed(SCPLL_DEBUG_NONE, base + SCPLL_DEBUG);

	/* Power-up SCPLL into standby mode */
	writel_relaxed(SCPLL_STANDBY, base + SCPLL_CTL);
	mb();
	udelay(10);

	/* Set calibration range: max_l_val in bits [31:24], min in bits [23:16] */
	regval = (SCPLL_L_VAL_MAX << 24) | (SCPLL_L_VAL_MIN << 16);
	writel_relaxed(regval, base + SCPLL_CAL);

	/* Start full calibration */
	writel_relaxed(SCPLL_FULL_CAL, base + SCPLL_CTL);

	/* Wait for LUT register to be populated (proves calibration started) */
	timeout = 1000;
	while (!readl_relaxed(base + SCPLL_LUT(SCPLL_L_VAL_MAX)) && --timeout)
		cpu_relax();

	if (!timeout) {
		pr_err("SCPLL: Calibration start timeout\n");
		return -ETIMEDOUT;
	}

	/* Wait for calibration to complete (bit 1 clears when done) */
	timeout = 1000;
	while ((readl_relaxed(base + SCPLL_STATUS) & 0x2) && --timeout)
		cpu_relax();

	if (!timeout) {
		pr_err("SCPLL: Calibration completion timeout\n");
		return -ETIMEDOUT;
	}

	/* Power-down SCPLL after calibration */
	writel_relaxed(SCPLL_POWER_DOWN, base + SCPLL_CTL);

	pr_debug("SCPLL: Calibration complete\n");
	return 0;
}

static void scpll_enable_at_l_val(void __iomem *base, u32 l_val)
{
	u32 regval;

	/* First ensure SCPLL is powered down */
	writel(SCPLL_POWER_DOWN, base + SCPLL_CTL);
	mb();
	udelay(10);

	/* Power-up SCPLL into standby mode */
	writel(SCPLL_STANDBY, base + SCPLL_CTL);
	mb();
	udelay(10);

	/* Shot-switch to target frequency */
	regval = (l_val << 3) | SCPLL_SHOT_SWITCH;
	writel(regval, base + SCPLL_FSM_CTL_EXT);
	writel(SCPLL_NORMAL, base + SCPLL_CTL);
	mb();
	udelay(20);
}

static void scpll_disable(void __iomem *base)
{
	writel_relaxed(SCPLL_POWER_DOWN, base + SCPLL_CTL);
}

static int scpll_change_freq(void __iomem *base, u32 l_val)
{
	u32 regval, ctl_val, status;
	int timeout;

	/* Complex-slew switch to target frequency */
	regval = (l_val << 3) | SCPLL_COMPLEX_SLEW;
	writel(regval, base + SCPLL_FSM_CTL_EXT);
	writel(SCPLL_NORMAL, base + SCPLL_CTL);
	mb();

	/* Wait for frequency switch to start (~50us typical, 1ms timeout) */
	timeout = 1000;
	do {
		ctl_val = readl(base + SCPLL_CTL);
		if (((ctl_val >> 3) & 0x3f) == l_val)
			break;
		udelay(1);
	} while (--timeout > 0);

	if (timeout == 0)
		return -ETIMEDOUT;

	/* Wait for frequency switch to finish (~50us typical, 1ms timeout) */
	timeout = 1000;
	do {
		status = readl(base + SCPLL_STATUS);
		if (!(status & 0x1))
			break;
		udelay(1);
	} while (--timeout > 0);

	return (timeout > 0) ? 0 : -ETIMEDOUT;
}

/*
 * Increase vdd_mem/vdd_dig before frequency increase.
 * Order: vdd_mem first, then vdd_dig.
 */
static int apcs_increase_vdd(struct apcs_cpu_clk *c, unsigned int vdd_mem,
			     unsigned int vdd_dig)
{
	int ret;

	if (c->vdd_mem) {
		ret = regulator_set_voltage(c->vdd_mem, vdd_mem, VDD_MEM_MAX);
		if (ret)
			return ret;
	}
	if (c->vdd_dig) {
		ret = regulator_set_voltage(c->vdd_dig, vdd_dig, VDD_DIG_MAX);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Decrease vdd_mem/vdd_dig after frequency decrease.
 * Order: vdd_dig first, then vdd_mem (reverse of increase).
 */
static void apcs_decrease_vdd(struct apcs_cpu_clk *c, unsigned int vdd_mem,
			      unsigned int vdd_dig)
{
	if (c->vdd_dig)
		regulator_set_voltage(c->vdd_dig, vdd_dig, VDD_DIG_MAX);
	if (c->vdd_mem)
		regulator_set_voltage(c->vdd_mem, vdd_mem, VDD_MEM_MAX);
}

/*
 * Program the *inactive* PLL-divider-mux bank with (src, div), then flip
 * the active-bank bit in SPSS_CLK_SEL with a single write. The CPU mux
 * sees a glitch-free transition to the new (src, div) combination.
 *
 * Callable only when the CPU is currently consuming the mux output -- i.e.
 * core_src_sel = CORE_SRC_MUX -- or while preparing the mux for an
 * upcoming MUX-takeover (caller is responsible for not toggling the bank
 * while CPU is on SCPLL, since a bank toggle would do nothing in that case
 * but the next CORE_SRC switch must see the prepared bank).
 *
 * src: acpuclk_src_sel value to write into the bank's src_sel field
 *      (SRC_SEL_AFAB=1, SRC_SEL_PLL8=3).
 * div: divider field value, N-1 for divide-by-N. 0 = /1, 1 = /2.
 */
static void program_pll_div_mux(struct apcs_cpu_clk *c, u32 src, u32 div)
{
	u32 sel, ctl, new_bank, shift;

	sel = readl(c->acc_base + SPSS_CLK_SEL);
	new_bank = (sel & CLK_SEL_BANK_BIT) ^ 1;
	shift = new_bank * CLK_CTL_BANK_FIELD_WIDTH;

	ctl = readl(c->acc_base + SPSS_CLK_CTL);
	ctl &= ~(0xFFU << shift);
	ctl |= ((src & 0xF) << CLK_CTL_SRC_SEL_SHIFT) << shift;
	ctl |= ((div & 0xF) << CLK_CTL_SRC_DIV_SHIFT) << shift;
	writel(ctl, c->acc_base + SPSS_CLK_CTL);

	/* Activate the bank we just programmed. */
	sel ^= CLK_SEL_BANK_BIT;
	writel(sel, c->acc_base + SPSS_CLK_SEL);
}

/* Switch the CPU's core source mux. new_src is CORE_SRC_MUX or CORE_SRC_SCPLL. */
static void select_core_src(struct apcs_cpu_clk *c, u32 new_src)
{
	u32 regval = readl(c->acc_base + SPSS_CLK_SEL);

	regval &= ~CLK_SEL_CORE_SRC_MASK;
	regval |= (new_src << CLK_SEL_CORE_SRC_SHIFT) & CLK_SEL_CORE_SRC_MASK;
	writel(regval, c->acc_base + SPSS_CLK_SEL);
	c->current_src = new_src;
}

/* Read the current core source mux value (bits [2:1] of SPSS_CLK_SEL). */
static u32 read_core_src(struct apcs_cpu_clk *c)
{
	u32 regval = readl(c->acc_base + SPSS_CLK_SEL);

	return (regval & CLK_SEL_CORE_SRC_MASK) >> CLK_SEL_CORE_SRC_SHIFT;
}

/*
 * Read the mux output rate when core_src_sel = CORE_SRC_MUX, by inspecting
 * the active bank's src_sel and div fields.
 */
static unsigned long read_mux_output_rate(struct apcs_cpu_clk *c)
{
	u32 sel = readl(c->acc_base + SPSS_CLK_SEL);
	u32 active_bank = sel & CLK_SEL_BANK_BIT;
	u32 shift = active_bank * CLK_CTL_BANK_FIELD_WIDTH;
	u32 ctl = readl(c->acc_base + SPSS_CLK_CTL);
	u32 field = (ctl >> shift) & 0xFF;
	u32 src = (field >> CLK_CTL_SRC_SEL_SHIFT) & 0xF;
	u32 div = (field >> CLK_CTL_SRC_DIV_SHIFT) & 0xF;
	unsigned long parent_rate;

	switch (src) {
	case SRC_SEL_PLL8:
		parent_rate = FREQ_PLL8;
		break;
	case SRC_SEL_AFAB:
		/* HP lk boot-handoff rate; not a vendor-validated mux input. */
		parent_rate = 310500000UL;
		break;
	default:
		/* Unknown source; report current_rate as fallback. */
		return c->current_rate;
	}

	return parent_rate / (div + 1);
}

static unsigned long apcs_cpu_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct apcs_cpu_clk *c = to_apcs_cpu_clk(hw);
	u32 core_src, ctl, l_val;

	core_src = read_core_src(c);
	if (core_src == CORE_SRC_SCPLL) {
		ctl = readl(c->scpll_base + SCPLL_CTL);
		if ((ctl & 0x7) == SCPLL_NORMAL) {
			l_val = (ctl >> 3) & 0x3f;
			c->current_rate = (unsigned long)l_val * SCPLL_RATE_FACTOR;
		}
		c->current_src = CORE_SRC_SCPLL;
	} else if (core_src == CORE_SRC_MUX) {
		c->current_rate = read_mux_output_rate(c);
		c->current_src = CORE_SRC_MUX;
	}

	return c->current_rate;
}

#define FREQ_PLL8_DIV2		(FREQ_PLL8 / 2)	/* 192 MHz */

static int apcs_cpu_clk_determine_rate(struct clk_hw *hw,
				       struct clk_rate_request *req)
{
	unsigned long rate = req->rate;

	if (rate >= FREQ_SCPLL_MAX) {
		rate = FREQ_SCPLL_MAX;
	} else if (rate >= FREQ_SCPLL_MIN) {
		/* SCPLL grid: 2 * 27 MHz * L_VAL (= SCPLL_RATE_FACTOR). */
		u32 l_val = DIV_ROUND_CLOSEST(rate, SCPLL_RATE_FACTOR);

		l_val = clamp_t(u32, l_val, SCPLL_L_VAL_MIN, SCPLL_L_VAL_MAX);
		rate = (unsigned long)l_val * SCPLL_RATE_FACTOR;
	} else if (rate >= FREQ_PLL8) {
		rate = FREQ_PLL8;		/* PLL_8 direct = 384 MHz */
	} else {
		rate = FREQ_PLL8_DIV2;		/* PLL_8 / 2 = 192 MHz */
	}

	req->rate = rate;
	return 0;
}

static int apcs_cpu_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct apcs_cpu_clk *c = to_apcs_cpu_clk(hw);
	const struct vdd_req *vdd_new, *vdd_old;
	u32 ctl, l_val_cur, l_val_new, cur_src;
	unsigned long flags;
	bool freq_increasing, going_to_mux;
	int ret = 0;

	ctl = readl(c->scpll_base + SCPLL_CTL);
	l_val_cur = (ctl >> 3) & 0x3f;

	cur_src = read_core_src(c);
	c->current_src = cur_src;
	freq_increasing = rate > c->current_rate;
	going_to_mux = (rate <= FREQ_PLL8);

	if (going_to_mux) {
		/*
		 * Targets at or below PLL_8 (384 MHz) run from the PLL
		 * divider mux. 192 MHz uses div=1 (PLL_8/2); 384 MHz uses
		 * div=0 (PLL_8/1). Going "down" by definition here, so:
		 *   - reprogram the inactive mux bank with PLL_8 at the
		 *     right divider, then activate that bank,
		 *   - flip core_src_sel from SCPLL to MUX (if not already
		 *     on MUX) so the CPU actually runs from the mux output,
		 *   - drop vdd_mem/vdd_dig + EBI BW vote to PLL_8 levels.
		 */
		u32 div = (rate <= FREQ_PLL8_DIV2) ? 1 : 0;

		spin_lock_irqsave(&c->lock, flags);
		program_pll_div_mux(c, SRC_SEL_PLL8, div);
		if (cur_src != CORE_SRC_MUX)
			select_core_src(c, CORE_SRC_MUX);
		c->current_rate = (div == 1) ? FREQ_PLL8_DIV2 : FREQ_PLL8;
		spin_unlock_irqrestore(&c->lock, flags);

		apcs_set_bus_bw(c, apcs_bw_for_rate(c->current_rate));
		apcs_decrease_vdd(c, VDD_MEM_PLL8, VDD_DIG_PLL8);
		return 0;
	}

	/* SCPLL path */
	l_val_new = DIV_ROUND_CLOSEST(rate, SCPLL_RATE_FACTOR);
	l_val_new = clamp_t(u32, l_val_new, SCPLL_L_VAL_MIN, SCPLL_L_VAL_MAX);

	/* Look up target vdd_mem/vdd_dig requirements */
	vdd_new = get_vdd_req(l_val_new);

	/* Increase voltage BEFORE frequency increase */
	if (freq_increasing && vdd_new) {
		ret = apcs_increase_vdd(c, vdd_new->vdd_mem, vdd_new->vdd_dig);
		if (ret)
			return ret;
	}

	/*
	 * Increase EBI bandwidth vote BEFORE the frequency increase so DDR
	 * is at the higher rate when the CPU starts demanding it. (Legacy
	 * votes after speed-change; we vote before, which is strictly more
	 * conservative.)
	 */
	if (freq_increasing)
		apcs_set_bus_bw(c, apcs_bw_for_rate(rate));

	spin_lock_irqsave(&c->lock, flags);

	/*
	 * Slew SCPLL to the target L_VAL while the CPU is still on whatever
	 * source it's on (mux or SCPLL). If we were on MUX the CPU is
	 * unaffected by this slew; if we were on SCPLL the slew is the
	 * actual freq transition (glitch-free per the complex-slew FSM).
	 */
	if (l_val_new != l_val_cur) {
		ret = scpll_change_freq(c->scpll_base, l_val_new);
		if (ret) {
			pr_err("apcs: freq change to L=%u failed: %d\n",
			       l_val_new, ret);
			spin_unlock_irqrestore(&c->lock, flags);
			if (freq_increasing) {
				vdd_old = get_vdd_req(l_val_cur);
				if (vdd_old)
					apcs_decrease_vdd(c, vdd_old->vdd_mem,
							  vdd_old->vdd_dig);
			}
			return ret;
		}
	}

	/*
	 * If the CPU was on the PLL divider mux, flip it over to SCPLL now
	 * that SCPLL is sitting at the right L_VAL. This is a one-write
	 * mux switch -- glitch-free.
	 */
	if (cur_src != CORE_SRC_SCPLL)
		select_core_src(c, CORE_SRC_SCPLL);

	c->current_rate = (unsigned long)l_val_new * SCPLL_RATE_FACTOR;
	spin_unlock_irqrestore(&c->lock, flags);

	/* Decrease EBI bandwidth vote AFTER frequency decrease */
	if (!freq_increasing)
		apcs_set_bus_bw(c, apcs_bw_for_rate(rate));

	/* Decrease voltage AFTER frequency decrease */
	if (!freq_increasing && vdd_new)
		apcs_decrease_vdd(c, vdd_new->vdd_mem, vdd_new->vdd_dig);

	return 0;
}

static const struct clk_ops apcs_cpu_clk_ops = {
	.recalc_rate = apcs_cpu_clk_recalc_rate,
	.determine_rate = apcs_cpu_clk_determine_rate,
	.set_rate = apcs_cpu_clk_set_rate,
};

static u32 cpu_to_l2_l_val(unsigned int cpu_khz)
{
	u32 l_val;

	/* CPU below SCPLL minimum (PLL8 path): hold L2 at SCPLL floor */
	if (cpu_khz < FREQ_SCPLL_MIN / 1000)
		return SCPLL_L_VAL_MIN;

	/*
	 * Lockstep: L2 L_VAL = CPU L_VAL. This guarantees the CPU's
	 * vdd_mem/vdd_dig vote (computed from CPU L_VAL via vdd_table)
	 * also covers L2 at the same L_VAL, matching the mainline
	 * Qualcomm pattern (Krait/Kryo do no separate L2 vdd vote).
	 *
	 * Capped at L2_L_VAL_MAX (1404 MHz) because legacy validated no
	 * L2 rate above that — at CPU 1512 MHz, L2 stays at 1404 MHz.
	 */
	l_val = DIV_ROUND_CLOSEST(cpu_khz * 1000, SCPLL_RATE_FACTOR);
	return clamp_t(u32, l_val, SCPLL_L_VAL_MIN, L2_L_VAL_MAX);
}

static void l2_set_freq(u32 l_val)
{
	unsigned long flags;

	spin_lock_irqsave(&l2_lock, flags);

	if (l_val == l2_current_l_val) {
		spin_unlock_irqrestore(&l2_lock, flags);
		return;
	}

	/*
	 * L2 is already running off SCPLL — HP lk hands the kernel a chip
	 * with L2 locked to the CPU SCPLL (Palm's downstream design). SCPLL
	 * complex-slew is glitch-free by hardware design, so changing L2
	 * frequency while CPUs use L2 is safe without further quiescing.
	 */
	scpll_change_freq(l2_scpll_base, l_val);

	l2_current_l_val = l_val;
	spin_unlock_irqrestore(&l2_lock, flags);
}

static int l2_cpufreq_notifier(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct cpufreq_freqs *freqs = data;
	u32 max_l_val;
	int cpu;

	if (!l2_initialized || event != CPUFREQ_POSTCHANGE)
		return NOTIFY_DONE;

	l2_vote[freqs->policy->cpu] = cpu_to_l2_l_val(freqs->new);

	max_l_val = SCPLL_L_VAL_MIN;
	for_each_online_cpu(cpu) {
		if (l2_vote[cpu] > max_l_val)
			max_l_val = l2_vote[cpu];
	}

	l2_set_freq(max_l_val);
	return NOTIFY_OK;
}

static struct notifier_block l2_cpufreq_nb = {
	.notifier_call = l2_cpufreq_notifier,
};

static int apcs_msm8660_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apcs_cpu_clk *cpu_clk;
	struct clk_init_data init = {};
	void __iomem *acc_base, *scpll_base;
	struct resource *res;
	int ret;

	dev_info(dev, "APCS probe starting\n");

	acc_base = devm_platform_ioremap_resource_byname(pdev, "acc");
	if (IS_ERR(acc_base))
		return PTR_ERR(acc_base);

	/*
	 * The SCPLL registers are within the GCC memory region which is
	 * already claimed. Use devm_ioremap directly without requesting
	 * the region exclusively.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "scpll");
	if (!res)
		return -ENODEV;

	scpll_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!scpll_base)
		return -ENOMEM;

	cpu_clk = devm_kzalloc(dev, sizeof(*cpu_clk), GFP_KERNEL);
	if (!cpu_clk)
		return -ENOMEM;

	cpu_clk->acc_base = acc_base;
	cpu_clk->scpll_base = scpll_base;
	spin_lock_init(&cpu_clk->lock);

	/* Acquire optional vdd_mem/vdd_dig regulators for voltage co-voting */
	cpu_clk->vdd_mem = devm_regulator_get_optional(dev, "vdd-mem");
	if (IS_ERR(cpu_clk->vdd_mem)) {
		dev_info(dev, "vdd-mem regulator: error %ld\n",
			 PTR_ERR(cpu_clk->vdd_mem));
		if (PTR_ERR(cpu_clk->vdd_mem) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		cpu_clk->vdd_mem = NULL;
	}

	cpu_clk->vdd_dig = devm_regulator_get_optional(dev, "vdd-dig");
	if (IS_ERR(cpu_clk->vdd_dig)) {
		dev_info(dev, "vdd-dig regulator: error %ld\n",
			 PTR_ERR(cpu_clk->vdd_dig));
		if (PTR_ERR(cpu_clk->vdd_dig) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		cpu_clk->vdd_dig = NULL;
	}

	if (cpu_clk->vdd_mem && cpu_clk->vdd_dig)
		dev_info(dev, "vdd_mem/vdd_dig co-voting enabled\n");
	else
		dev_info(dev, "vdd_mem/vdd_dig co-voting not available\n");

	/* Acquire CPU → EBI interconnect path for bus bandwidth voting */
	cpu_clk->icc_cpu_mem = devm_of_icc_get(dev, "cpu-mem");
	if (IS_ERR(cpu_clk->icc_cpu_mem)) {
		if (PTR_ERR(cpu_clk->icc_cpu_mem) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "cpu-mem interconnect: error %ld (bandwidth voting disabled)\n",
			 PTR_ERR(cpu_clk->icc_cpu_mem));
		cpu_clk->icc_cpu_mem = NULL;
	} else {
		dev_info(dev, "CPU → EBI bandwidth voting enabled\n");
	}

	/*
	 * Skip SCPLL calibration for now - assume bootloader already did it.
	 * TODO: Add proper calibration if needed.
	 */
	cpu_clk->calibrated = true;
	dev_info(dev, "SCPLL assumed calibrated by bootloader\n");

	/*
	 * Read the actual core source select and current rate from
	 * hardware rather than assuming PLL_8. HP lk hands the kernel
	 * a chip running on AFAB at ~310.5 MHz (per webOS dmesg:
	 * `cpufreq: cpu0 init at 310500 switching to 192000`), with
	 * SCPLL already calibrated in NORMAL mode but not yet routed
	 * as the core source. cpufreq's first transition will switch
	 * us over.
	 */
	cpu_clk->current_src = read_core_src(cpu_clk);
	if (cpu_clk->current_src == CORE_SRC_SCPLL) {
		u32 ctl = readl(cpu_clk->scpll_base + SCPLL_CTL);

		if ((ctl & 0x7) == SCPLL_NORMAL)
			cpu_clk->current_rate =
				((ctl >> 3) & 0x3f) * SCPLL_RATE_FACTOR;
		else
			cpu_clk->current_rate = FREQ_PLL8;
	} else {
		cpu_clk->current_rate = read_mux_output_rate(cpu_clk);
	}

	init.name = "cpu_clk";
	if (of_property_read_string(dev->of_node, "clock-output-names",
				    &init.name))
		init.name = "cpu_clk";

	init.ops = &apcs_cpu_clk_ops;
	init.flags = CLK_GET_RATE_NOCACHE;

	cpu_clk->hw.init = &init;

	ret = devm_clk_hw_register(dev, &cpu_clk->hw);
	if (ret) {
		dev_err(dev, "Failed to register CPU clock: %d\n", ret);
		return ret;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					  &cpu_clk->hw);
	if (ret) {
		dev_err(dev, "Failed to add clock provider: %d\n", ret);
		return ret;
	}

	dev_info(dev, "CPU clock registered at %lu MHz\n",
		 cpu_clk->current_rate / 1000000);

	/*
	 * Initialize L2 SCPLL tracking — only once, by first CPU to probe.
	 *
	 * Prior code here flipped an L2 source-mux register inside a
	 * stop_machine() + flush_cache_all() block, on the assumption that
	 * HP lk handed off with L2 still on the PLL divider mux. Two
	 * problems with that:
	 *
	 *   1. The L2 CLK_SEL register on msm8660 lives at GCC + 0x38
	 *      (physical 0x02082038), not at any ACC base + 0x38. The
	 *      previous code wrote into (acc_base + 0x38) = 0x02041038,
	 *      which is in CPU0's ACC region and unrelated to the L2
	 *      source mux. The write was a no-op for L2 routing.
	 *
	 *   2. Per Palm's downstream acpuclock-8x60.c the L2 SCPLL is
	 *      already running in NORMAL mode at handoff (webOS keeps L2
	 *      locked to the CPU SCPLL), so even a *correct* write would
	 *      have set the same bits the hardware already had.
	 *
	 * Net effect: the stop_machine block accomplished nothing useful
	 * but still opened a deadlock window vs uncached bus masters
	 * (msm-dma, USB, MMC, GPU, RPM) that can hit L2 while CPUs are
	 * parked in the stopper thread. That is the cause of the
	 * intermittent very-early-boot hangs on Topaz3G PVT.
	 *
	 * What we keep:
	 *   - reading SCPLL_CTL to confirm L2 SCPLL is in NORMAL mode
	 *     (bail if not — re-enabling from a cold state would need
	 *     shot-switch + calibration that this driver doesn't implement),
	 *   - recording l2_current_l_val so the cpufreq notifier path knows
	 *     where L2 starts and can skip no-op slews,
	 *   - registering the cpufreq notifier so subsequent CPU freq
	 *     transitions can drive L2 in lockstep (those slews use the
	 *     SCPLL complex-slew FSM and are glitch-free in hardware).
	 *
	 * What we drop:
	 *   - stop_machine(l2_quiesced_clksel_switch, ...) — the source
	 *     mux flip and its deadlock window,
	 *   - the (incorrectly addressed) l2_clk_sel_base mapping,
	 *   - the pre-vote of EBI bandwidth that existed only to back the
	 *     mux flip.
	 */
	if (!l2_initialized) {
		struct resource *l2_res;
		u32 ctl;

		l2_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						      "l2-scpll");
		if (!l2_res)
			goto skip_l2;

		l2_scpll_base = devm_ioremap(dev, l2_res->start,
					     resource_size(l2_res));
		if (!l2_scpll_base) {
			dev_warn(dev, "Failed to map L2 SCPLL\n");
			goto skip_l2;
		}

		ctl = readl(l2_scpll_base + SCPLL_CTL);
		if ((ctl & 0x7) != SCPLL_NORMAL) {
			dev_warn(dev,
				 "L2 SCPLL not in NORMAL mode (ctl=0x%x), skipping\n",
				 ctl);
			goto skip_l2;
		}

		/* Record current L_VAL so the notifier path skips a no-op slew */
		l2_current_l_val = (ctl >> 3) & 0x3f;

		ret = cpufreq_register_notifier(&l2_cpufreq_nb,
						CPUFREQ_TRANSITION_NOTIFIER);
		if (ret) {
			dev_warn(dev, "L2 cpufreq notifier failed\n");
			goto skip_l2;
		}

		l2_initialized = true;
		dev_info(dev,
			 "L2 SCPLL scaling initialized at L_VAL=0x%x (%lu MHz)\n",
			 l2_current_l_val,
			 (unsigned long)l2_current_l_val * SCPLL_RATE_FACTOR /
				 1000000);
	}
skip_l2:

	return 0;
}

static const struct of_device_id apcs_msm8660_match_table[] = {
	{ .compatible = "qcom,apq8060-apcs" },
	{ .compatible = "qcom,msm8660-apcs" },
	{}
};
MODULE_DEVICE_TABLE(of, apcs_msm8660_match_table);

static struct platform_driver apcs_msm8660_driver = {
	.probe = apcs_msm8660_probe,
	.driver = {
		.name = "qcom-apcs-msm8660",
		.of_match_table = apcs_msm8660_match_table,
	},
};
module_platform_driver(apcs_msm8660_driver);

MODULE_DESCRIPTION("Qualcomm APQ8060/MSM8660 APCS Clock Driver");
MODULE_LICENSE("GPL");
