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

/*
 * Register offsets from ACC base
 * Each CPU has its own ACC at different base addresses
 */
#define SPSS_CLK_CTL		0x00
#define SPSS_CLK_SEL		0x04

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

/*
 * L2 cache SCPLL frequency scaling.
 * The L2 cache has its own SCPLL (same register layout as CPU SCPLLs).
 * Legacy coupled L2 frequency to CPU frequency using 3 tiers.
 */
#define L2_L_VAL_LOW	0x08	/* 432 MHz */
#define L2_L_VAL_MID	0x12	/* 972 MHz */
#define L2_L_VAL_HIGH	0x1A	/* 1404 MHz */
#define L2_THRESH_MID_KHZ	972000
#define L2_THRESH_HIGH_KHZ	1080000

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

/* L2 SCPLL shared state — initialized by first CPU to probe */
static void __iomem *l2_scpll_base;
static void __iomem *l2_clk_sel_base;
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

static void select_clk_source(struct apcs_cpu_clk *c, int src)
{
	u32 regval;

	/* Read current selection */
	regval = readl(c->acc_base + SPSS_CLK_SEL);

	/*
	 * For CPU cores, source select bits are at [2:1] (shift=1).
	 * L2 would use [1:0] (shift=0) but we don't handle L2 here.
	 */
	regval &= ~(0x3 << 1);
	regval |= ((src & 0x3) << 1);

	writel(regval, c->acc_base + SPSS_CLK_SEL);

	c->current_src = src;
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
	u32 ctl, l_val_cur, l_val_new;
	unsigned long flags;
	bool freq_increasing;
	int ret = 0;

	ctl = readl(c->scpll_base + SCPLL_CTL);
	l_val_cur = (ctl >> 3) & 0x3f;

	if (rate <= FREQ_PLL8) {
		/* TODO: Switch to PLL8 source for low frequencies */
		c->current_rate = FREQ_PLL8;
		apcs_decrease_vdd(c, VDD_MEM_PLL8, VDD_DIG_PLL8);
		return 0;
	}

	l_val_new = DIV_ROUND_CLOSEST(rate, SCPLL_RATE_FACTOR);
	l_val_new = clamp_t(u32, l_val_new, SCPLL_L_VAL_MIN, SCPLL_L_VAL_MAX);

	freq_increasing = rate > c->current_rate;

	/* Look up target vdd_mem/vdd_dig requirements */
	vdd_new = get_vdd_req(l_val_new);

	/* Increase voltage BEFORE frequency increase */
	if (freq_increasing && vdd_new) {
		ret = apcs_increase_vdd(c, vdd_new->vdd_mem, vdd_new->vdd_dig);
		if (ret)
			return ret;
	}

	if (l_val_new == l_val_cur) {
		c->current_rate = (unsigned long)l_val_cur * SCPLL_RATE_FACTOR;
		goto done;
	}

	/*
	 * Change SCPLL frequency using complex slew.
	 * vdd_sc scaling is handled by cpufreq-dt via the SAW regulator.
	 */
	spin_lock_irqsave(&c->lock, flags);

	ret = scpll_change_freq(c->scpll_base, l_val_new);
	if (ret) {
		pr_err("apcs: freq change to L=%u failed: %d\n", l_val_new, ret);
		spin_unlock_irqrestore(&c->lock, flags);
		/* Roll back voltage on failure */
		if (freq_increasing) {
			vdd_old = get_vdd_req(l_val_cur);
			if (vdd_old)
				apcs_decrease_vdd(c, vdd_old->vdd_mem,
						  vdd_old->vdd_dig);
		}
		return ret;
	}

	c->current_rate = (unsigned long)l_val_new * SCPLL_RATE_FACTOR;
	spin_unlock_irqrestore(&c->lock, flags);

done:
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
		/* First time — enable L2 SCPLL via shot-switch */
		scpll_enable_at_l_val(l2_scpll_base, l_val);
		mb();
		/* Select SCPLL as L2 source (bits [1:0], shift=0) */
		regval = readl(l2_clk_sel_base);
		regval &= ~0x3;
		regval |= SRC_SEL_SCPLL;
		writel(regval, l2_clk_sel_base);
	} else {
		scpll_change_freq(l2_scpll_base, l_val);
	}

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

	max_l_val = L2_L_VAL_LOW;
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
		if (PTR_ERR(cpu_clk->vdd_mem) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		cpu_clk->vdd_mem = NULL;
	}

	cpu_clk->vdd_dig = devm_regulator_get_optional(dev, "vdd-dig");
	if (IS_ERR(cpu_clk->vdd_dig)) {
		if (PTR_ERR(cpu_clk->vdd_dig) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		cpu_clk->vdd_dig = NULL;
	}

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

	/* Initialize L2 SCPLL — only once, by first CPU to probe */
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

			l2_clk_sel_base = acc_base + SPSS_L2_CLK_SEL;

			/*
			 * Skip L2 SCPLL calibration — assume bootloader
			 * already calibrated it (same as CPU SCPLLs).
			 */

			l2_current_l_val = 0;
			l2_set_freq(L2_L_VAL_LOW);

			ret = cpufreq_register_notifier(&l2_cpufreq_nb,
						CPUFREQ_TRANSITION_NOTIFIER);
			if (ret) {
				dev_warn(dev, "L2 cpufreq notifier failed\n");
				goto skip_l2;
			}

			l2_initialized = true;
			dev_info(dev, "L2 SCPLL scaling initialized\n");
		}
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
