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
 * Voltage domains:
 *   - vdd_sc  - per-core voltage via SAW regulators (handled by cpufreq-dt)
 *   - vdd_mem - memory voltage (PM8058 S0), must track vdd_sc
 *   - vdd_dig - digital core voltage (PM8058 S1), tracks CPU frequency
 *
 * cpufreq-dt only manages vdd_sc via the SAW regulator. The vdd_mem and
 * vdd_dig regulators must co-vote alongside CPU frequency transitions to
 * maintain system stability, especially at frequencies above 1 GHz.
 * Voltage values are derived from the legacy webOS acpuclock-8x60.c.
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

/*
 * Register offsets from ACC base.
 * Each CPU has its own ACC at different base addresses.
 * Matches webOS kernel: CLK_CTL at ACC+0x04, CLK_SEL at ACC+0x08.
 */
#define SPSS_CLK_CTL		0x04
#define SPSS_CLK_SEL		0x08

/* L2 clock selection register */
#define SPSS_L2_CLK_SEL		0x38

/* Clock source selection values */
#define SRC_SEL_AFAB		0	/* 27 MHz */
#define SRC_SEL_PLL8		3	/* 384 MHz */
#define SRC_SEL_SCPLL		1	/* Dynamic SCPLL */

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
#define SCPLL_L_VAL_MAX		0x1c	/* 1512 MHz */
#define SCPLL_RATE_FACTOR	54000000UL

/* Frequency thresholds */
#define FREQ_PLL8		384000000UL
#define FREQ_SCPLL_MIN		432000000UL
#define FREQ_SCPLL_MAX		1512000000UL

/*
 * L2 cache SCPLL frequency scaling.
 *
 * The L2 cache has its own SCPLL (at MSM_SCPLL_BASE + 0x400) with the
 * same register layout as CPU SCPLLs. Legacy webOS kernel coupled L2
 * frequency to CPU frequency for optimal performance.
 *
 * CPU freq (kHz) -> L2 L_VAL mapping (from legacy acpu_freq_tbl_v2):
 *   192-918 MHz  -> L2 at 432 MHz  (L_VAL=0x08)
 *   972-1026 MHz -> L2 at 972 MHz  (L_VAL=0x12)
 *   1080+ MHz    -> L2 at 1404 MHz (L_VAL=0x1A)
 */
#define L2_L_VAL_LOW	0x08	/* 432 MHz */
#define L2_L_VAL_MID	0x12	/* 972 MHz */
#define L2_L_VAL_HIGH	0x1A	/* 1404 MHz */

/* CPU frequency thresholds for L2 scaling (in kHz, matching cpufreq) */
#define L2_THRESH_MID_KHZ	972000
#define L2_THRESH_HIGH_KHZ	1080000

/* Maximum regulator voltage for vdd_mem/vdd_dig (PM8058 S0/S1) */
#define VDD_MEM_MAX		1350000
#define VDD_DIG_MAX		1350000

/*
 * vdd_mem and vdd_dig voltage requirements per CPU frequency.
 *
 * Derived from legacy webOS acpuclock-8x60.c:
 *   vdd_mem = max(vdd_sc, l2_level->vdd_mem)
 *   vdd_dig = max(l2_level->vdd_dig, pll_vdd_dig)
 * where pll_vdd_dig is 1100000 if CPU or L2 freq > 594 MHz, else 1000000.
 *
 * The table is indexed by L_VAL - SCPLL_L_VAL_MIN (0x08).
 * For PLL8 (384 MHz), use the default low values.
 */
struct vdd_req {
	unsigned int vdd_mem;	/* PM8058 S0 target voltage in uV */
	unsigned int vdd_dig;	/* PM8058 S1 target voltage in uV */
};

/* Default vdd_mem/vdd_dig for PLL8 (384 MHz) and below */
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
	[0x0F] = { 1250000, 1200000 },	/* L_VAL 0x17: interpolated */
	[0x10] = { 1250000, 1200000 },	/* L_VAL 0x18: interpolated */
	[0x11] = { 1250000, 1200000 },	/* L_VAL 0x19: interpolated */
	[0x12] = { 1250000, 1200000 },	/* L_VAL 0x1A: 1404 MHz */
	[0x13] = { 1250000, 1200000 },	/* L_VAL 0x1B: interpolated */
	[0x14] = { 1250000, 1200000 },	/* L_VAL 0x1C: 1512 MHz */
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

/**
 * struct apcs_cpu_clk - Per-CPU clock structure
 * @hw: clk_hw for clock framework
 * @acc_base: ACC (Application Core Cluster) register base
 * @scpll_base: SCPLL register base for this CPU
 * @vdd_mem: memory voltage regulator (PM8058 S0), optional
 * @vdd_dig: digital core voltage regulator (PM8058 S1), optional
 * @current_rate: current CPU frequency
 * @current_src: current clock source (AFAB, PLL8, SCPLL)
 * @calibrated: true if SCPLL has been calibrated
 * @lock: spinlock for register access
 */
struct apcs_cpu_clk {
	struct clk_hw hw;
	void __iomem *acc_base;
	void __iomem *scpll_base;
	struct regulator *vdd_mem;
	struct regulator *vdd_dig;
	unsigned long current_rate;
	int current_src;
	bool calibrated;
	spinlock_t lock;
};

#define to_apcs_cpu_clk(_hw) container_of(_hw, struct apcs_cpu_clk, hw)

/*
 * L2 cache SCPLL state — shared between both CPU APCS instances.
 * Protected by l2_lock. Initialized by whichever CPU probes first.
 */
static void __iomem *l2_scpll_base;
static void __iomem *l2_clk_sel_base;
static DEFINE_SPINLOCK(l2_lock);
static u32 l2_current_l_val;
static bool l2_initialized;
static u32 l2_vote[NR_CPUS];	/* per-CPU L2 L_VAL votes */

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
 * Set vdd_mem and vdd_dig voltages for a frequency increase.
 * Must be called BEFORE the frequency switch.
 * Order: vdd_mem first (must be >= vdd_sc and vdd_dig), then vdd_dig.
 */
static int apcs_increase_vdd(struct apcs_cpu_clk *c, unsigned int vdd_mem,
			     unsigned int vdd_dig)
{
	int ret;

	if (c->vdd_mem) {
		ret = regulator_set_voltage(c->vdd_mem, vdd_mem, VDD_MEM_MAX);
		if (ret) {
			pr_err("apcs: vdd_mem increase to %u uV failed: %d\n",
			       vdd_mem, ret);
			return ret;
		}
	}

	if (c->vdd_dig) {
		ret = regulator_set_voltage(c->vdd_dig, vdd_dig, VDD_DIG_MAX);
		if (ret) {
			pr_err("apcs: vdd_dig increase to %u uV failed: %d\n",
			       vdd_dig, ret);
			return ret;
		}
	}

	return 0;
}

/*
 * Set vdd_mem and vdd_dig voltages for a frequency decrease.
 * Must be called AFTER the frequency switch.
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
 * Select the core clock source mux.
 *
 * CLK_SEL register bits [2:1] (for CPU cores, shift=1):
 *   0 = PLL divider mux (AFAB or PLL8, selected via CLK_CTL bank)
 *   1 = SCPLL
 */
static void select_core_source(struct apcs_cpu_clk *c, int src)
{
	u32 regval;

	regval = readl(c->acc_base + SPSS_CLK_SEL);
	regval &= ~(0x3 << 1);
	regval |= ((src & 0x3) << 1);
	writel(regval, c->acc_base + SPSS_CLK_SEL);

	c->current_src = src;
}

/*
 * Program the PLL divider mux and switch to it using bank toggling.
 *
 * CLK_CTL has two banks (0 and 1) of 8 bits each:
 *   Bank N at bits [8*N+7 : 8*N]:
 *     bits [7:4] = source select (3=PLL8, 1=AFAB)
 *     bits [3:0] = divider (0=div1, 1=div2, etc.)
 *
 * CLK_SEL bit 0 (SRC1N0) selects which bank is active.
 * To switch: program bank[SRC1N0], then toggle SRC1N0.
 *
 * Based on webOS select_clk_source_div().
 */
static void select_clk_source_div(struct apcs_cpu_clk *c,
				  u32 src_sel, u32 src_div)
{
	u32 reg_clksel, reg_clkctl, bank;

	reg_clksel = readl(c->acc_base + SPSS_CLK_SEL);

	/* CLK_SEL_SRC1N0 (bank select) bit */
	bank = reg_clksel & 1;

	/* Program clock source and divider (matches webOS sequence) */
	reg_clkctl = readl(c->acc_base + SPSS_CLK_CTL);
	reg_clkctl &= ~(0xFF << (8 * bank));
	reg_clkctl |= src_sel << (4 + 8 * bank);
	reg_clkctl |= src_div << (0 + 8 * bank);
	writel(reg_clkctl, c->acc_base + SPSS_CLK_CTL);

	/* Toggle clock source bank */
	reg_clksel ^= 1;
	writel(reg_clksel, c->acc_base + SPSS_CLK_SEL);
}

static unsigned long apcs_cpu_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct apcs_cpu_clk *c = to_apcs_cpu_clk(hw);
	u32 ctl, l_val;

	/* Read actual rate from hardware */
	ctl = readl(c->scpll_base + SCPLL_CTL);
	if ((ctl & 0x7) == SCPLL_NORMAL) {
		l_val = (ctl >> 3) & 0x3f;
		c->current_rate = (unsigned long)l_val * SCPLL_RATE_FACTOR;
	}

	return c->current_rate;
}

static int apcs_cpu_clk_determine_rate(struct clk_hw *hw,
				       struct clk_rate_request *req)
{
	unsigned long rate = req->rate;

	/* Clamp to supported range */
	if (rate < FREQ_PLL8)
		rate = FREQ_PLL8;
	else if (rate > FREQ_SCPLL_MAX)
		rate = FREQ_SCPLL_MAX;

	/* Round to nearest SCPLL step if using SCPLL */
	if (rate >= FREQ_SCPLL_MIN) {
		u32 l_val = DIV_ROUND_CLOSEST(rate, SCPLL_RATE_FACTOR);
		l_val = clamp_t(u32, l_val, SCPLL_L_VAL_MIN, SCPLL_L_VAL_MAX);
		rate = (unsigned long)l_val * SCPLL_RATE_FACTOR;
	} else {
		rate = FREQ_PLL8;
	}

	req->rate = rate;
	return 0;
}

static int apcs_cpu_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct apcs_cpu_clk *c = to_apcs_cpu_clk(hw);
	const struct vdd_req *vdd_new, *vdd_old;
	u32 l_val_new;
	unsigned int vdd_mem_new, vdd_dig_new;
	unsigned long flags;
	bool from_scpll = (c->current_src == SRC_SEL_SCPLL);
	bool freq_increasing;
	int ret = 0;

	if (rate <= FREQ_PLL8) {
		/*
		 * Switch to PLL8 at 384 MHz (src_sel=3, div=0).
		 * From legacy acpu_freq_tbl_v2: PLL8 384 MHz row has
		 * acpuclk_src_sel=3, acpuclk_src_div=0, core_src_sel=0.
		 */
		spin_lock_irqsave(&c->lock, flags);
		select_clk_source_div(c, SRC_SEL_PLL8, 0);
		select_core_source(c, 0);  /* 0 = PLL divider mux */
		mb();
		if (from_scpll)
			scpll_disable(c->scpll_base);
		spin_unlock_irqrestore(&c->lock, flags);

		c->current_rate = FREQ_PLL8;

		/* Decrease vdd_mem/vdd_dig after switching to PLL8 */
		apcs_decrease_vdd(c, VDD_MEM_PLL8, VDD_DIG_PLL8);

		pr_debug("apcs: switched to PLL8 at %lu Hz\n", FREQ_PLL8);
		return 0;
	}

	l_val_new = DIV_ROUND_CLOSEST(rate, SCPLL_RATE_FACTOR);
	l_val_new = clamp_t(u32, l_val_new, SCPLL_L_VAL_MIN, SCPLL_L_VAL_MAX);

	freq_increasing = rate > c->current_rate;

	/* Look up target vdd_mem/vdd_dig requirements */
	vdd_new = get_vdd_req(l_val_new);
	if (vdd_new) {
		vdd_mem_new = vdd_new->vdd_mem;
		vdd_dig_new = vdd_new->vdd_dig;
	} else {
		vdd_mem_new = VDD_MEM_PLL8;
		vdd_dig_new = VDD_DIG_PLL8;
	}

	/*
	 * Voltage sequencing for vdd_mem/vdd_dig:
	 * - Frequency increase: raise BEFORE freq change
	 * - Frequency decrease: lower AFTER freq change
	 *
	 * vdd_sc (SAW regulator) is handled by cpufreq-dt with the same
	 * ordering, so it is already at the correct level when we get here
	 * for frequency increases.
	 */
	if (freq_increasing) {
		ret = apcs_increase_vdd(c, vdd_mem_new, vdd_dig_new);
		if (ret)
			return ret;
	}

	/*
	 * Change SCPLL frequency.
	 * vdd_sc scaling is handled by cpufreq-dt via the SAW regulator
	 * (cpu-supply property in device tree).
	 */
	spin_lock_irqsave(&c->lock, flags);

	if (!from_scpll) {
		/*
		 * Switching from PLL8/AFAB to SCPLL.
		 * Enable SCPLL at target frequency, then switch core mux.
		 */
		scpll_enable_at_l_val(c->scpll_base, l_val_new);
		mb();
		select_core_source(c, SRC_SEL_SCPLL);
	} else {
		/* Already on SCPLL -- slew to new frequency */
		ret = scpll_change_freq(c->scpll_base, l_val_new);
		if (ret) {
			pr_err("apcs: freq change to L=%u failed: %d\n",
			       l_val_new, ret);
			spin_unlock_irqrestore(&c->lock, flags);
			/* Roll back voltage on failure */
			if (freq_increasing) {
				vdd_old = get_vdd_req(
					DIV_ROUND_CLOSEST(c->current_rate,
							  SCPLL_RATE_FACTOR));
				if (vdd_old)
					apcs_decrease_vdd(c, vdd_old->vdd_mem,
							  vdd_old->vdd_dig);
				else
					apcs_decrease_vdd(c, VDD_MEM_PLL8,
							  VDD_DIG_PLL8);
			}
			return ret;
		}
	}

	c->current_rate = (unsigned long)l_val_new * SCPLL_RATE_FACTOR;
	spin_unlock_irqrestore(&c->lock, flags);

	/* Decrease vdd_mem/vdd_dig after frequency decrease */
	if (!freq_increasing)
		apcs_decrease_vdd(c, vdd_mem_new, vdd_dig_new);

	pr_debug("apcs: freq changed to %lu Hz (L=%u) vdd_mem=%u vdd_dig=%u\n",
		 c->current_rate, l_val_new, vdd_mem_new, vdd_dig_new);
	return 0;
}

static const struct clk_ops apcs_cpu_clk_ops = {
	.recalc_rate = apcs_cpu_clk_recalc_rate,
	.determine_rate = apcs_cpu_clk_determine_rate,
	.set_rate = apcs_cpu_clk_set_rate,
};

/*
 * Determine the L2 SCPLL L_VAL for a given CPU frequency.
 * Uses the 3-tier mapping from legacy acpu_freq_tbl_v2:
 *   CPU <= 918 MHz  -> L2 at 432 MHz (L_VAL=0x08)
 *   CPU 972-1026 MHz -> L2 at 972 MHz (L_VAL=0x12)
 *   CPU >= 1080 MHz  -> L2 at 1404 MHz (L_VAL=0x1A)
 */
static u32 cpu_to_l2_l_val(unsigned int cpu_khz)
{
	if (cpu_khz >= L2_THRESH_HIGH_KHZ)
		return L2_L_VAL_HIGH;
	if (cpu_khz >= L2_THRESH_MID_KHZ)
		return L2_L_VAL_MID;
	return L2_L_VAL_LOW;
}

static void l2_set_freq(u32 l_val)
{
	unsigned long flags;
	u32 regval;

	spin_lock_irqsave(&l2_lock, flags);

	if (l_val == l2_current_l_val) {
		spin_unlock_irqrestore(&l2_lock, flags);
		return;
	}

	if (l2_current_l_val == 0) {
		/* L2 SCPLL not yet running — enable via shot-switch */
		scpll_enable_at_l_val(l2_scpll_base, l_val);
		mb();
		/* Select SCPLL as L2 clock source (bits [1:0], shift=0) */
		regval = readl(l2_clk_sel_base);
		regval &= ~0x3;
		regval |= SRC_SEL_SCPLL;
		writel(regval, l2_clk_sel_base);
	} else {
		/* L2 SCPLL already running — slew to new frequency */
		scpll_change_freq(l2_scpll_base, l_val);
	}

	l2_current_l_val = l_val;
	spin_unlock_irqrestore(&l2_lock, flags);

	pr_debug("apcs: L2 freq changed to %lu MHz (L=%u)\n",
		 (unsigned long)l_val * SCPLL_RATE_FACTOR / 1000000, l_val);
}

static int l2_cpufreq_notifier(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct cpufreq_freqs *freqs = data;
	u32 l_val, max_l_val;
	int cpu;

	if (!l2_initialized)
		return NOTIFY_DONE;

	if (event != CPUFREQ_POSTCHANGE)
		return NOTIFY_DONE;

	/* Update this CPU's L2 vote */
	l2_vote[freqs->policy->cpu] = cpu_to_l2_l_val(freqs->new);

	/* Find the maximum L2 vote across all CPUs */
	max_l_val = L2_L_VAL_LOW;
	for_each_online_cpu(cpu) {
		l_val = l2_vote[cpu];
		if (l_val > max_l_val)
			max_l_val = l_val;
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

	/*
	 * Acquire vdd_mem (PM8058 S0) and vdd_dig (PM8058 S1) regulators.
	 * These are optional -- if not present in DT, cpufreq will still
	 * work but without the safety of vdd_mem/vdd_dig co-voting.
	 */
	cpu_clk->vdd_mem = devm_regulator_get_optional(dev, "vdd-mem");
	if (IS_ERR(cpu_clk->vdd_mem)) {
		if (PTR_ERR(cpu_clk->vdd_mem) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "vdd-mem regulator not available, skipping co-voting\n");
		cpu_clk->vdd_mem = NULL;
	}

	cpu_clk->vdd_dig = devm_regulator_get_optional(dev, "vdd-dig");
	if (IS_ERR(cpu_clk->vdd_dig)) {
		if (PTR_ERR(cpu_clk->vdd_dig) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "vdd-dig regulator not available, skipping co-voting\n");
		cpu_clk->vdd_dig = NULL;
	}

	if (cpu_clk->vdd_mem && cpu_clk->vdd_dig)
		dev_info(dev, "vdd_mem/vdd_dig co-voting enabled\n");

	/*
	 * Skip SCPLL calibration for now - assume bootloader already did it.
	 * TODO: Add proper calibration if needed.
	 */
	cpu_clk->calibrated = true;
	dev_info(dev, "SCPLL assumed calibrated by bootloader\n");

	/*
	 * Don't change CPU clock during probe - the system may not be ready.
	 * Just record the current state (assume PLL8 at boot) and let
	 * cpufreq handle frequency changes later when the system is stable.
	 */
	cpu_clk->current_rate = FREQ_PLL8;
	cpu_clk->current_src = SRC_SEL_PLL8;

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
	 * Initialize L2 SCPLL scaling -- only once for the first CPU.
	 * The L2 SCPLL is shared between both cores and uses the same
	 * register interface as CPU SCPLLs.
	 */
	if (!l2_initialized) {
		struct resource *l2_res;

		l2_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						      "l2-scpll");
		if (l2_res) {
			l2_scpll_base = devm_ioremap(dev, l2_res->start,
						     resource_size(l2_res));
			if (!l2_scpll_base) {
				dev_warn(dev, "Failed to map L2 SCPLL\n");
				goto skip_l2;
			}

			/* L2 CLK_SEL is at ACC base + SPSS_L2_CLK_SEL */
			l2_clk_sel_base = acc_base + SPSS_L2_CLK_SEL;

			/* Calibrate L2 SCPLL */
			ret = scpll_calibrate(l2_scpll_base);
			if (ret) {
				dev_warn(dev,
					 "L2 SCPLL calibration failed: %d\n",
					 ret);
				goto skip_l2;
			}

			/* Start L2 at low frequency */
			l2_current_l_val = 0;
			l2_set_freq(L2_L_VAL_LOW);

			ret = cpufreq_register_notifier(&l2_cpufreq_nb,
						CPUFREQ_TRANSITION_NOTIFIER);
			if (ret) {
				dev_warn(dev,
					 "L2 cpufreq notifier failed: %d\n",
					 ret);
				goto skip_l2;
			}

			l2_initialized = true;
			dev_info(dev,
				 "L2 SCPLL scaling initialized at 432 MHz\n");
		} else {
			dev_info(dev,
				 "No L2 SCPLL resource, scaling disabled\n");
		}
	}
skip_l2:

	/*
	 * Program Scorpion-specific CP15 registers for L2 cache performance.
	 * These come up in an unpredictable state after reset and must be
	 * initialized on each CPU core.
	 */

	/* L2CR0: enable out-of-order bus attributes and error reporting */
	asm volatile("mcr p15, 3, %0, c15, c0, 1" : : "r" (0xC0050F0F));

	/* L2CR1: DBB (Disable Barrier Broadcast) for SMP stability */
	asm volatile("mcr p15, 3, %0, c15, c0, 3" : : "r" (0x100));

	dev_info(dev, "Scorpion L2CR0/L2CR1 initialized\n");

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
