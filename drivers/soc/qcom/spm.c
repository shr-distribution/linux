// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 *
 * SAW power controller driver
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/smp.h>

#include <linux/regulator/driver.h>

#include <soc/qcom/spm.h>

#define FIELD_SET(current, mask, val)	\
	(((current) & ~(mask)) | FIELD_PREP((mask), (val)))

#define SPM_CTL_INDEX		0x7f
#define SPM_CTL_INDEX_SHIFT	4
#define SPM_CTL_EN		BIT(0)

/* These registers might be specific to SPM 1.1 */
#define SPM_VCTL_VLVL			GENMASK(7, 0)
#define SPM_PMIC_DATA_0_VLVL		GENMASK(7, 0)
#define SPM_PMIC_DATA_1_MIN_VSEL	GENMASK(5, 0)
#define SPM_PMIC_DATA_1_MAX_VSEL	GENMASK(21, 16)

#define SPM_1_1_AVS_CTL_AVS_ENABLED	BIT(27)
#define SPM_AVS_CTL_MAX_VLVL		GENMASK(22, 17)
#define SPM_AVS_CTL_MIN_VLVL		GENMASK(15, 10)

enum spm_reg {
	SPM_REG_CFG,
	SPM_REG_SPM_CTL,
	SPM_REG_DLY,
	SPM_REG_PMIC_DLY,
	SPM_REG_PMIC_DATA_0,
	SPM_REG_PMIC_DATA_1,
	SPM_REG_VCTL,
	SPM_REG_SEQ_ENTRY,
	SPM_REG_STS0,
	SPM_REG_STS1,
	SPM_REG_PMIC_STS,
	SPM_REG_AVS_CTL,
	SPM_REG_AVS_LIMIT,
	SPM_REG_RST,
	/* MSM8660-specific registers (SAW v1.0) */
	SPM_REG_WAKE_TMR_DLY,
	SPM_REG_SLP_CLK_EN,
	SPM_REG_SLP_HSFS_PRECLMP_EN,
	SPM_REG_SLP_HSFS_POSTCLMP_EN,
	SPM_REG_SLP_CLMP_EN,
	SPM_REG_SPM_MPM_CFG,
	SPM_REG_NR,
};

#define MAX_PMIC_DATA		2
#define MAX_SEQ_DATA		64

struct spm_reg_data {
	const u16 *reg_offset;
	u32 spm_cfg;
	u32 spm_dly;
	u32 pmic_dly;
	u32 pmic_data[MAX_PMIC_DATA];
	u32 avs_ctl;
	u32 avs_limit;
	u8 seq[MAX_SEQ_DATA];
	u8 start_index[PM_SLEEP_MODE_NR];
	bool no_seq_ram;	/* SAW v1.0: register-based mode, no sequence RAM */
	/*
	 * SAW v1.0 (MSM8660 / MSM8260 / APQ8060) power-collapses CPU cores
	 * directly via the SPM hardware sequence triggered by WFI, with no
	 * involvement from the Secure Monitor. The TouchPad's tz.mbn (and
	 * the equivalent TrustZone blobs on other MSM8x60 variants) does
	 * not implement the SCM_BOOT/TERMINATE_PC call: the SMC returns
	 * without doing anything, which cpuidle interprets as a pending-
	 * IRQ rejection and falls back to WFI. Set this flag to skip the
	 * qcom_scm_cpu_power_down() call entirely and let SPM hardware
	 * drive the collapse, matching the legacy 2.6.35-palm path
	 * (arch/arm/mach-msm/idle-v7.S: msm_pm_collapse).
	 *
	 * The warm-boot vector is still installed via the standard
	 * qcom_scm_set_warm_boot_addr() path, which IS supported by the
	 * MSM8x60 TZ.
	 */
	bool no_scm_terminate;

	/* MSM8660-specific init values (SAW v1.0 only) */
	u32 spm_ctl_init;		/* Initial SPM_CTL value */
	u32 wake_tmr_dly;		/* SAW_SPM_WAKE_TMR_DLY */
	u32 slp_clk_en;			/* SAW_SLP_CLK_EN (CPU-specific) */
	u32 slp_hsfs_preclmp_en;	/* SAW_SLP_HSFS_PRECLMP_EN */
	u32 slp_hsfs_postclmp_en;	/* SAW_SLP_HSFS_POSTCLMP_EN */
	u32 slp_clmp_en;		/* SAW_SLP_CLMP_EN */
	u32 slp_rst_en_init;		/* Initial SAW_SLP_RST_EN */
	u32 spm_mpm_cfg;		/* SAW_SPM_MPM_CFG */

	smp_call_func_t set_vdd;
	/*
	 * Optional hardware-readback for the live voltage selector. When
	 * NULL the regulator framework reports the cached drv->volt_sel,
	 * which goes stale on regulators that aren't touched by cpufreq
	 * (e.g. cpu1_vdd on a single-policy SMP where cpufreq-dt only
	 * drives one regulator). When provided, spm_get_voltage_sel
	 * refreshes drv->volt_sel from the SAW status register before
	 * returning it.
	 */
	void (*get_vdd)(struct spm_driver_data *drv);
	const struct linear_range *ranges;
	unsigned int n_ranges;
	/*
	 * For MSM8660 Band 2/Band 3 split: selectors <= band2_max_sel use
	 * Band 2 marker (0x80); selectors above it use Band 3 marker (0xC0).
	 * Only consulted when n_ranges > 1 and a band-aware set_vdd is wired.
	 */
	unsigned int band2_max_sel;
	unsigned int ramp_delay;
	unsigned int init_uV;
};

struct spm_driver_data {
	void __iomem *reg_base;
	const struct spm_reg_data *reg_data;
	struct device *dev;
	unsigned int volt_sel;
	int reg_cpu;
};

static const u16 spm_reg_offset_v4_1[SPM_REG_NR] = {
	[SPM_REG_AVS_CTL]	= 0x904,
	[SPM_REG_AVS_LIMIT]	= 0x908,
};

static const struct spm_reg_data spm_reg_660_gold_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x1010031,
	.avs_limit = 0x4580458,
};

static const struct spm_reg_data spm_reg_660_silver_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x101c031,
	.avs_limit = 0x4580458,
};

static const struct spm_reg_data spm_reg_8998_gold_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x1010031,
	.avs_limit = 0x4700470,
};

static const struct spm_reg_data spm_reg_8998_silver_l2  = {
	.reg_offset = spm_reg_offset_v4_1,
	.avs_ctl = 0x1010031,
	.avs_limit = 0x4200420,
};

static const u16 spm_reg_offset_v3_0[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_SEQ_ENTRY]	= 0x400,
};

/* SPM register data for 8909 */
static const struct spm_reg_data spm_reg_8909_cpu = {
	.reg_offset = spm_reg_offset_v3_0,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x10, 0x80, 0x30, 0x90,
		0x5B, 0x60, 0x03, 0x60, 0x76, 0x76, 0x0B, 0x94, 0x5B, 0x80,
		0x10, 0x26, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

/* SPM register data for 8916 */
static const struct spm_reg_data spm_reg_8916_cpu = {
	.reg_offset = spm_reg_offset_v3_0,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x10, 0x80, 0x30, 0x90,
		0x5B, 0x60, 0x03, 0x60, 0x3B, 0x76, 0x76, 0x0B, 0x94, 0x5B,
		0x80, 0x10, 0x26, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

static const struct spm_reg_data spm_reg_8939_cpu = {
	.reg_offset = spm_reg_offset_v3_0,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x50, 0x1B, 0x10, 0x80,
		0x30, 0x90, 0x5B, 0x60, 0x50, 0x03, 0x60, 0x76, 0x76, 0x0B,
		0x50, 0x1B, 0x94, 0x5B, 0x80, 0x10, 0x26, 0x30, 0x50, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

static const u16 spm_reg_offset_v2_3[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_PMIC_DATA_0]	= 0x40,
	[SPM_REG_PMIC_DATA_1]	= 0x44,
};

/* SPM register data for 8976 */
static const struct spm_reg_data spm_reg_8976_gold_l2 = {
	.reg_offset = spm_reg_offset_v2_3,
	.spm_cfg = 0x14,
	.spm_dly = 0x3c11840a,
	.pmic_data[0] = 0x03030080,
	.pmic_data[1] = 0x00030000,
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 3,
};

static const struct spm_reg_data spm_reg_8976_silver_l2 = {
	.reg_offset = spm_reg_offset_v2_3,
	.spm_cfg = 0x14,
	.spm_dly = 0x3c102800,
	.pmic_data[0] = 0x03030080,
	.pmic_data[1] = 0x00030000,
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 2,
};

static const u16 spm_reg_offset_v2_1[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_SPM_CTL]	= 0x30,
	[SPM_REG_DLY]		= 0x34,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

/* SPM register data for 8974, 8084 */
static const struct spm_reg_data spm_reg_8974_8084_cpu  = {
	.reg_offset = spm_reg_offset_v2_1,
	.spm_cfg = 0x1,
	.spm_dly = 0x3C102800,
	.seq = { 0x03, 0x0B, 0x0F, 0x00, 0x20, 0x80, 0x10, 0xE8, 0x5B, 0x03,
		0x3B, 0xE8, 0x5B, 0x82, 0x10, 0x0B, 0x30, 0x06, 0x26, 0x30,
		0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 3,
};

/* SPM register data for 8226 */
static const struct spm_reg_data spm_reg_8226_cpu  = {
	.reg_offset = spm_reg_offset_v2_1,
	.spm_cfg = 0x0,
	.spm_dly = 0x3C102800,
	.seq = { 0x60, 0x03, 0x60, 0x0B, 0x0F, 0x20, 0x10, 0x80, 0x30, 0x90,
		0x5B, 0x60, 0x03, 0x60, 0x3B, 0x76, 0x76, 0x0B, 0x94, 0x5B,
		0x80, 0x10, 0x26, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 5,
};

static const u16 spm_reg_offset_v1_1[SPM_REG_NR] = {
	[SPM_REG_CFG]		= 0x08,
	[SPM_REG_STS0]		= 0x0c,
	[SPM_REG_STS1]		= 0x10,
	[SPM_REG_VCTL]		= 0x14,
	[SPM_REG_AVS_CTL]	= 0x18,
	[SPM_REG_SPM_CTL]	= 0x20,
	[SPM_REG_PMIC_DLY]	= 0x24,
	[SPM_REG_PMIC_DATA_0]	= 0x28,
	[SPM_REG_PMIC_DATA_1]	= 0x2C,
	[SPM_REG_SEQ_ENTRY]	= 0x80,
};

/*
 * MSM8660/APQ8060 SAW register offsets - different from APQ8064!
 * Based on webOS kernel arch/arm/mach-msm/spm.c
 */
static const u16 spm_reg_offset_8660[SPM_REG_NR] = {
	[SPM_REG_AVS_CTL]		= 0x04,
	[SPM_REG_VCTL]			= 0x08,
	[SPM_REG_STS0]			= 0x0c,	/* SAW_STS */
	[SPM_REG_STS1]			= 0x0c,	/* Same as STS0 on 8660 */
	[SPM_REG_CFG]			= 0x10,
	[SPM_REG_SPM_CTL]		= 0x14,
	[SPM_REG_PMIC_DLY]		= 0x18,	/* SAW_SPM_SLP_TMR_DLY */
	[SPM_REG_WAKE_TMR_DLY]		= 0x1c,	/* SAW_SPM_WAKE_TMR_DLY */
	[SPM_REG_PMIC_DATA_0]		= 0x20,	/* SAW_SPM_PMIC_CTL */
	[SPM_REG_SLP_CLK_EN]		= 0x24,	/* SAW_SLP_CLK_EN */
	[SPM_REG_SLP_HSFS_PRECLMP_EN]	= 0x28,	/* SAW_SLP_HSFS_PRECLMP_EN */
	[SPM_REG_SLP_HSFS_POSTCLMP_EN]	= 0x2c,	/* SAW_SLP_HSFS_POSTCLMP_EN */
	[SPM_REG_SLP_CLMP_EN]		= 0x30,	/* SAW_SLP_CLMP_EN */
	[SPM_REG_RST]			= 0x34,	/* SLP_RST_EN */
	[SPM_REG_SPM_MPM_CFG]		= 0x38,	/* SAW_SPM_MPM_CFG */
	/* No sequence entry on 8660 - different architecture */
};

static void smp_set_vdd_v1_1(void *data);
static void smp_set_vdd_8660(void *data);
static void smp_get_vdd_8660(struct spm_driver_data *drv);

/* SPM register data for 8064 */
static const struct linear_range spm_v1_1_regulator_ranges[] = {
	REGULATOR_LINEAR_RANGE(700000, 0, 56, 12500),
};

static const struct spm_reg_data spm_reg_8064_cpu = {
	.reg_offset = spm_reg_offset_v1_1,
	.spm_cfg = 0x1F,
	.pmic_dly = 0x02020004,
	.pmic_data[0] = 0x0084009C,
	.pmic_data[1] = 0x00A4001C,
	.seq = { 0x03, 0x0F, 0x00, 0x24, 0x54, 0x10, 0x09, 0x03, 0x01,
		0x10, 0x54, 0x30, 0x0C, 0x24, 0x30, 0x0F },
	.start_index[PM_SLEEP_MODE_STBY] = 0,
	.start_index[PM_SLEEP_MODE_SPC] = 2,
	.set_vdd = smp_set_vdd_v1_1,
	.ranges = spm_v1_1_regulator_ranges,
	.n_ranges = ARRAY_SIZE(spm_v1_1_regulator_ranges),
	.init_uV = 1300000,
	.ramp_delay = 1250,
};

/*
 * SPM register data for MSM8660/APQ8060 Scorpion CPUs.
 * Based on webOS kernel board-tenderloin.c msm_spm_data.
 * Voltage range: 840mV - 1250mV from webOS saw_s0_init_data.
 * awake_vlevel = 0xA0 corresponds to ~1.1V
 *
 * IMPORTANT: Bootloader investigation (HTC TrustZone, HTC SBL3, TouchPad APPSBL)
 * shows ZERO SAW register initialization. The comment claiming "bootloader already
 * configured SPM" was incorrect. We MUST initialize SPM registers at probe, matching
 * legacy webOS kernel behavior (arch/arm/mach-msm/board-tenderloin.c lines 335-395).
 *
 * Hardware reads SPM_CTL=0x08 at boot (bootloader default), not the required 0x68.
 * Without explicit initialization, power collapse will not function correctly.
 */
/*
 * PM8901 SMPS voltage encoding (per webos-uber-kernel saw-regulator.c):
 * the 8-bit vlevel sent to SAW VCTL is `band[7:6] | local_sel[5:0]`, with
 * three valid bands:
 *
 *   Band 1: 350--650 mV, 6.25 mV step, marker 0x40 (sel 0..48)
 *   Band 2: 700--1400 mV, 12.5 mV step, marker 0x80 (sel 0..56)
 *   Band 3: 1500--3300 mV, 50 mV step, marker 0xC0 (sel 2..38)
 *
 * Two ranges exposed to the regulator framework: Band 2 (selectors
 * 0..56) and Band 3 (selectors 57..93 in the framework's continuous
 * selector space, corresponding to PMIC local selectors 2..38). Band 3
 * local selectors 0 and 1 (1.40 V, 1.45 V) sit in the Band 2/Band 3 gap
 * that the PMIC firmware rejects, so we skip them -- per uber-kernel's
 * saw-regulator.c "Round down for set points in the gaps between
 * bands" rule.
 *
 * The framework picks a continuous selector across both ranges; the
 * 8660-specific set/get helpers translate that selector into the
 * `band[7:6] | local_sel[5:0]` form the PMIC expects:
 *
 *   continuous sel <= band2_max_sel (56)
 *       -> Band 2, local_sel = continuous_sel,        vlevel = 0x80 | sel
 *   continuous sel  > band2_max_sel
 *       -> Band 3, local_sel = continuous_sel - 55,   vlevel = 0xC0 | local_sel
 *
 * (-55 because Band 3 starts at continuous sel 57 which is local sel 2;
 * 57 - 55 = 2.)
 *
 * Band 1 (350--650 mV) is intentionally not exposed: no CPU OPP in
 * either webOS or upstream lands in that range and the PMIC firmware
 * has its own minimum operating point above 650 mV.
 */
#define SPM_8660_BAND2_MAX_SEL		56
#define SPM_8660_BAND3_SEL_OFFSET	55	/* continuous -> Band 3 local */

static const struct linear_range spm_8660_regulator_ranges[] = {
	/* Band 2: 700..1400 mV, 12.5 mV step */
	REGULATOR_LINEAR_RANGE(700000, 0, 56, 12500),
	/* Band 3: 1500..3300 mV, 50 mV step (continuous sel 57..93) */
	REGULATOR_LINEAR_RANGE(1500000, 57, 93, 50000),
};

static const struct spm_reg_data spm_reg_8660_cpu = {
	.reg_offset = spm_reg_offset_8660,

	/*
	 * SPM register initialization values from legacy webOS kernel
	 * arch/arm/mach-msm/board-tenderloin.c msm_spm_data[]
	 */
	.spm_cfg = 0x1C,		/* SAW_CFG: IRQ edge sensitive, config bits */
	.pmic_dly = 0x0C0CFFFF,		/* SAW_SPM_SLP_TMR_DLY: sleep timer delay */
	.wake_tmr_dly = 0x78780FFF,	/* SAW_SPM_WAKE_TMR_DLY: wake timer delay */

	/*
	 * Initial SPM_CTL value at boot (clock gating mode).
	 * Bits [7:4] = 0x6: event output config
	 * Bits [3]   = 1:   rpm_bypass (standalone mode at boot)
	 * Bits [1:0] = 0x0: mode = CLOCK_GATING
	 */
	.spm_ctl_init = 0x68,

	/*
	 * Clock enable for different sleep modes.
	 * NOTE: CPU0 uses 0x01, CPU1 uses 0x13 (per-CPU difference!)
	 * This field will be overridden in probe based on CPU index.
	 */
	.slp_clk_en = 0x01,		/* CPU0 default, see probe for CPU1 */

	.slp_hsfs_preclmp_en = 0x07,	/* HS/FS pre-clamp enable */
	.slp_hsfs_postclmp_en = 0x00,	/* HS/FS post-clamp enable */
	.slp_clmp_en = 0x01,		/* Clamp enable */
	.slp_rst_en_init = 0x00,	/* Core reset NOT asserted at boot */
	.spm_mpm_cfg = 0x00,		/* MPM config */

	.no_seq_ram = true,
	/*
	 * MSM8660 / MSM8260 / APQ8060 TZ blob does not implement
	 * SCM_BOOT/TERMINATE_PC. Drive power-collapse via SPM hardware
	 * sequence (WFI handshake) instead of the SCM call. Matches the
	 * legacy 2.6.35-palm msm_pm_collapse() path. See the field comment
	 * in struct spm_reg_data for the full rationale.
	 */
	.no_scm_terminate = true,
	.set_vdd = smp_set_vdd_8660,
	.get_vdd = smp_get_vdd_8660,
	.ranges = spm_8660_regulator_ranges,
	.n_ranges = ARRAY_SIZE(spm_8660_regulator_ranges),
	.band2_max_sel = SPM_8660_BAND2_MAX_SEL,
	.init_uV = 1100000,		/* awake_vlevel 0xA0 = ~1.1V */
	.ramp_delay = 1250,
};

static inline void spm_register_write(struct spm_driver_data *drv,
					enum spm_reg reg, u32 val)
{
	if (drv->reg_data->reg_offset[reg])
		writel_relaxed(val, drv->reg_base +
				drv->reg_data->reg_offset[reg]);
}

/* Ensure a guaranteed write, before return */
static inline void spm_register_write_sync(struct spm_driver_data *drv,
					enum spm_reg reg, u32 val)
{
	u32 ret;

	if (!drv->reg_data->reg_offset[reg])
		return;

	do {
		writel_relaxed(val, drv->reg_base +
				drv->reg_data->reg_offset[reg]);
		ret = readl_relaxed(drv->reg_base +
				drv->reg_data->reg_offset[reg]);
		if (ret == val)
			break;
		cpu_relax();
	} while (1);
}

static inline u32 spm_register_read(struct spm_driver_data *drv,
				    enum spm_reg reg)
{
	return readl_relaxed(drv->reg_base + drv->reg_data->reg_offset[reg]);
}

/*
 * MSM8660/APQ8060 SAW v1.0 mode setting.
 *
 * SAW v1.0 has NO sequence RAM — SPM_CTL uses register-based mode selection:
 *   Bits [2:0] = mode encoding (0x00=clock gate, 0x02=retention/PC)
 *   Bit 3 = RPM bypass (1=standalone, no RPM notification)
 *   Bits [6:4] = event output selection (preserve bootloader value)
 *
 * SLP_RST_EN (offset 0x34) distinguishes retention from power collapse:
 *   0x00 = retention (no core reset)
 *   0x01 = power collapse (core reset asserts during power-down)
 */
static void spm_set_low_power_mode_8660(struct spm_driver_data *drv,
					enum pm_sleep_mode mode)
{
	u32 ctl_val;

	ctl_val = spm_register_read(drv, SPM_REG_SPM_CTL);

	/* Preserve bits [7:4] (event output + upper config), update [3:0] */
	ctl_val &= ~0x0F;

	switch (mode) {
	case PM_SLEEP_MODE_PC:
		/*
		 * Full power collapse with RPM coordination:
		 *   mode=0x02 (power collapse), rpm_bypass=0 (notify RPM),
		 *   rst=1 (core reset asserts).
		 *
		 * Clearing bit 3 (rpm_bypass) makes the SPM signal its sleep
		 * state to the RPM via the master_stat bits. If RPM sees all
		 * CPU masters signalling sleep AND no other masters are voting
		 * for shared resources, it can drop L2 / vdd_mem / vdd_dig /
		 * pxo to their sleep-set levels. Without separate RPM resource
		 * voting (rpmrs - see follow-up patch) the cluster sleep will
		 * only proceed as deep as the *default* RPM sleep-set, but
		 * that is still deeper than SPC because the L2 cache can be
		 * powered off via QCOM_SCM_CPU_PWR_DOWN_L2_OFF on the way in.
		 */
		ctl_val |= 0x02;
		spm_register_write(drv, SPM_REG_RST, 0x01);
		break;
	case PM_SLEEP_MODE_SPC:
		/* Standalone power collapse: mode=0x02, rpm_bypass=1, rst=1 */
		ctl_val |= (1 << 3) | 0x02;
		spm_register_write(drv, SPM_REG_RST, 0x01);
		break;
	case PM_SLEEP_MODE_RET:
		/* Retention: mode=0x02, rpm_bypass=1, rst=0 */
		ctl_val |= (1 << 3) | 0x02;
		spm_register_write(drv, SPM_REG_RST, 0x00);
		break;
	default:
		/* Standby / clock gating: mode=0x00, rpm_bypass=1, rst=0 */
		ctl_val |= (1 << 3) | 0x00;
		spm_register_write(drv, SPM_REG_RST, 0x00);
		break;
	}

	spm_register_write_sync(drv, SPM_REG_SPM_CTL, ctl_val);
}

/**
 * spm_get_drv_by_cpu - Get SPM driver data for a CPU
 * @cpu: CPU number
 *
 * Returns SPM driver data for the specified CPU, or NULL if not found.
 * Used by CPU hotplug path to access SPM for power collapse.
 */
struct spm_driver_data *spm_get_drv_by_cpu(unsigned int cpu)
{
	struct device_node *cpu_node, *saw_node;
	struct platform_device *pdev;
	struct spm_driver_data *drv = NULL;

	cpu_node = of_cpu_device_node_get(cpu);
	if (!cpu_node)
		return NULL;

	saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
	of_node_put(cpu_node);
	if (!saw_node)
		return NULL;

	pdev = of_find_device_by_node(saw_node);
	of_node_put(saw_node);
	if (!pdev)
		return NULL;

	drv = dev_get_drvdata(&pdev->dev);
	put_device(&pdev->dev);

	return drv;
}
EXPORT_SYMBOL(spm_get_drv_by_cpu);

/**
 * spm_collapse_via_scm() - whether to delegate power-collapse to TZ via SCM
 * @drv: SPM driver instance
 *
 * Most Qualcomm SoCs delegate the CPU power-collapse handshake to TrustZone
 * via the SCM_BOOT/TERMINATE_PC call (qcom_scm_cpu_power_down). A handful
 * of SAW v1.0 parts (MSM8660 / MSM8260 / APQ8060) ship with a TZ firmware
 * that does NOT implement that call: the SMC returns immediately without
 * powering down, which cpuidle interprets as a pending-IRQ rejection. On
 * those parts the collapse is instead driven by the SPM hardware sequence,
 * which is triggered from a raw WFI executed by the kernel.
 *
 * Callers (cpuidle, hotplug, suspend) check this and branch their
 * cpu_suspend() callback accordingly.
 *
 * Return: true if the TZ-based path should be used; false to use a raw WFI.
 */
bool spm_collapse_via_scm(struct spm_driver_data *drv)
{
	if (!drv || !drv->reg_data)
		return true;
	return !drv->reg_data->no_scm_terminate;
}
EXPORT_SYMBOL_GPL(spm_collapse_via_scm);

void spm_set_low_power_mode(struct spm_driver_data *drv,
			    enum pm_sleep_mode mode)
{
	u32 start_index;
	u32 ctl_val;

	/* MSM8660/APQ8060 SAW v1.0: register-based mode, no sequence index */
	if (drv->reg_data->no_seq_ram) {
		spm_set_low_power_mode_8660(drv, mode);
		return;
	}

	/* SAW v1.1+ with sequence RAM */
	start_index = drv->reg_data->start_index[mode];

	ctl_val = spm_register_read(drv, SPM_REG_SPM_CTL);
	ctl_val &= ~(SPM_CTL_INDEX << SPM_CTL_INDEX_SHIFT);
	ctl_val |= start_index << SPM_CTL_INDEX_SHIFT;
	ctl_val |= SPM_CTL_EN;
	spm_register_write_sync(drv, SPM_REG_SPM_CTL, ctl_val);
}
EXPORT_SYMBOL(spm_set_low_power_mode);

static int spm_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	struct spm_driver_data *drv = rdev_get_drvdata(rdev);

	drv->volt_sel = selector;

	/* Always do the SAW register writes on the corresponding CPU */
	return smp_call_function_single(drv->reg_cpu, drv->reg_data->set_vdd, drv, true);
}

static int spm_get_voltage_sel(struct regulator_dev *rdev)
{
	struct spm_driver_data *drv = rdev_get_drvdata(rdev);

	/*
	 * If the SoC variant provides a hardware readback, refresh the
	 * cached selector from the SAW status register. Reads are MMIO
	 * (readl_relaxed) and can run from any CPU — no IPI required,
	 * which also means the readback works while the target CPU is
	 * hotplugged offline.
	 */
	if (drv->reg_data->get_vdd)
		drv->reg_data->get_vdd(drv);

	return drv->volt_sel;
}

static const struct regulator_ops spm_reg_ops = {
	.set_voltage_sel	= spm_set_voltage_sel,
	.get_voltage_sel	= spm_get_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear_range,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

static void smp_set_vdd_v1_1(void *data)
{
	struct spm_driver_data *drv = data;
	unsigned int vctl, data0, data1, avs_ctl, sts;
	unsigned int vlevel, volt_sel;
	bool avs_enabled;

	volt_sel = drv->volt_sel;
	vlevel = volt_sel | 0x80; /* band */

	avs_ctl = spm_register_read(drv, SPM_REG_AVS_CTL);
	vctl = spm_register_read(drv, SPM_REG_VCTL);
	data0 = spm_register_read(drv, SPM_REG_PMIC_DATA_0);
	data1 = spm_register_read(drv, SPM_REG_PMIC_DATA_1);

	avs_enabled = avs_ctl & SPM_1_1_AVS_CTL_AVS_ENABLED;

	/* If AVS is enabled, switch it off during the voltage change */
	if (avs_enabled) {
		avs_ctl &= ~SPM_1_1_AVS_CTL_AVS_ENABLED;
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}

	/* Kick the state machine back to idle */
	spm_register_write(drv, SPM_REG_RST, 1);

	vctl = FIELD_SET(vctl, SPM_VCTL_VLVL, vlevel);
	data0 = FIELD_SET(data0, SPM_PMIC_DATA_0_VLVL, vlevel);
	data1 = FIELD_SET(data1, SPM_PMIC_DATA_1_MIN_VSEL, volt_sel);
	data1 = FIELD_SET(data1, SPM_PMIC_DATA_1_MAX_VSEL, volt_sel);

	spm_register_write(drv, SPM_REG_VCTL, vctl);
	spm_register_write(drv, SPM_REG_PMIC_DATA_0, data0);
	spm_register_write(drv, SPM_REG_PMIC_DATA_1, data1);

	if (read_poll_timeout_atomic(spm_register_read,
				     sts, sts == vlevel,
				     1, 200, false,
				     drv, SPM_REG_STS1)) {
		dev_err_ratelimited(drv->dev, "timeout setting the voltage (%x %x)!\n", sts, vlevel);
		goto enable_avs;
	}

	if (avs_enabled) {
		unsigned int max_avs = volt_sel;
		unsigned int min_avs = max(max_avs, 4U) - 4;

		avs_ctl = FIELD_SET(avs_ctl, SPM_AVS_CTL_MIN_VLVL, min_avs);
		avs_ctl = FIELD_SET(avs_ctl, SPM_AVS_CTL_MAX_VLVL, max_avs);
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}

enable_avs:
	if (avs_enabled) {
		avs_ctl |= SPM_1_1_AVS_CTL_AVS_ENABLED;
		spm_register_write(drv, SPM_REG_AVS_CTL, avs_ctl);
	}
}

/*
 * Encode the continuous regulator-framework selector for the 8660 SAW
 * into the PMIC 8-bit vlevel `band[7:6] | local_sel[5:0]`. See the
 * commentary on spm_8660_regulator_ranges for the mapping rule.
 */
static u8 spm_8660_encode_vlevel(const struct spm_reg_data *rd,
				 unsigned int volt_sel)
{
	if (volt_sel <= rd->band2_max_sel)
		return 0x80 | (volt_sel & 0x3F);
	return 0xC0 | ((volt_sel - SPM_8660_BAND3_SEL_OFFSET) & 0x3F);
}

/*
 * Decode the PMIC 8-bit vlevel read back from STS0 into the continuous
 * regulator-framework selector. Inverse of spm_8660_encode_vlevel.
 * Returns drv->volt_sel-shaped value; preserves the last cached selector
 * for unknown/empty band markers so a transient zero read doesn't snap
 * the framework's idea of the live voltage down to selector 0.
 */
static unsigned int spm_8660_decode_vlevel(struct spm_driver_data *drv,
					   u8 vlevel)
{
	u8 band = vlevel & 0xC0;
	u8 local = vlevel & 0x3F;

	if (band == 0x80)
		return local;
	if (band == 0xC0) {
		/* Band 3 local sel 0..1 are in the band gap; clamp to floor. */
		if (local < 2)
			local = 2;
		return local + SPM_8660_BAND3_SEL_OFFSET;
	}
	/* Band 1 (0x40) or unset: keep the cached value. */
	return drv->volt_sel;
}

/*
 * MSM8660/APQ8060 voltage setting function.
 * Based on webOS kernel msm_spm_set_vdd() in arch/arm/mach-msm/spm.c
 * and saw-regulator.c voltage band calculations.
 *
 * PM8901 SMPS Band 2: 700-1400 mV, 12.5 mV step, marker 0x80.
 * PM8901 SMPS Band 3: 1500-3300 mV, 50 mV step, marker 0xC0.
 * The vlevel sent to SAW VCTL is `band[7:6] | local_sel[5:0]`.
 */
static void smp_set_vdd_8660(void *data)
{
	struct spm_driver_data *drv = data;
	unsigned int vctl, sts;
	unsigned int vlevel, volt_sel;
	int timeout_us = 50;

	volt_sel = drv->volt_sel;
	vlevel = spm_8660_encode_vlevel(drv->reg_data, volt_sel);

	/* Read current VCTL and update voltage level (bits 7:0) */
	vctl = spm_register_read(drv, SPM_REG_VCTL);
	vctl &= ~0xFF;
	vctl |= vlevel;
	spm_register_write(drv, SPM_REG_VCTL, vctl);

	/* Wait for PMIC state to return to idle (bits 21:20 == 0) */
	/* and current voltage (bits 17:10) to match requested */
	do {
		sts = spm_register_read(drv, SPM_REG_STS0);
		if (((sts >> 20) & 0x3) == 0 &&	/* PMIC state idle */
		    ((sts >> 10) & 0xFF) == vlevel)	/* Voltage matches */
			return;

		if (timeout_us > 10) {
			udelay(10);
			timeout_us -= 10;
		} else {
			udelay(timeout_us);
			timeout_us = 0;
		}
	} while (timeout_us > 0);

	dev_err_ratelimited(drv->dev,
		"timeout setting voltage: sts=0x%x, requested vlevel=0x%x\n",
		sts, vlevel);
}

/*
 * MSM8660/APQ8060 voltage readback. STS0[17:10] reflects the live PMIC
 * 8-bit vlevel (the value last written to VCTL[7:0], in the
 * `band[7:6] | local_sel[5:0]` form). spm_8660_decode_vlevel translates
 * that back into the continuous selector the regulator framework uses
 * to index our two-range linear-range table.
 *
 * Why this exists: the regulator framework's regulator_get_voltage_sel
 * path returns drv->volt_sel directly, which is only updated when
 * spm_set_voltage_sel runs. On a single-cpufreq-policy SMP system the
 * inactive CPU's regulator is never written (cpufreq-dt drives just one
 * cpu_supply per policy), so its drv->volt_sel stays at init_uV=1100000
 * indefinitely and sysfs reports a stale ~1.1V even though the SAW is
 * actually programmed (by the bootloader or by the active core's
 * master-slave coupling) to the correct higher voltage.
 *
 * Reading STS0 is just readl_relaxed on the SAW's MMIO window; safe from
 * any CPU and works while the regulator's "owner" CPU is hotplugged
 * offline.
 */
static void smp_get_vdd_8660(struct spm_driver_data *drv)
{
	unsigned int sts0;
	u8 vlevel;

	sts0 = spm_register_read(drv, SPM_REG_STS0);
	vlevel = (sts0 >> 10) & 0xFF;
	drv->volt_sel = spm_8660_decode_vlevel(drv, vlevel);
}

static int spm_get_cpu(struct device *dev)
{
	int cpu;
	bool found;

	for_each_possible_cpu(cpu) {
		struct device_node *cpu_node, *saw_node;

		cpu_node = of_cpu_device_node_get(cpu);
		if (!cpu_node)
			continue;

		saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
		found = (saw_node == dev->of_node);
		of_node_put(saw_node);
		of_node_put(cpu_node);

		if (found)
			return cpu;
	}

	/* L2 SPM is not bound to any CPU, voltage setting is not supported */

	return -EOPNOTSUPP;
}

static int spm_register_regulator(struct device *dev, struct spm_driver_data *drv)
{
	struct regulator_config config = {
		.dev = dev,
		.driver_data = drv,
	};
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	int ret;
	bool found;

	if (!drv->reg_data->set_vdd)
		return 0;

	rdesc = devm_kzalloc(dev, sizeof(*rdesc), GFP_KERNEL);
	if (!rdesc)
		return -ENOMEM;

	rdesc->name = "spm";
	rdesc->of_match = of_match_ptr("regulator");
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->owner = THIS_MODULE;
	rdesc->ops = &spm_reg_ops;

	rdesc->linear_ranges = drv->reg_data->ranges;
	rdesc->n_linear_ranges = drv->reg_data->n_ranges;
	rdesc->n_voltages = rdesc->linear_ranges[rdesc->n_linear_ranges - 1].max_sel + 1;
	rdesc->ramp_delay = drv->reg_data->ramp_delay;

	ret = spm_get_cpu(dev);
	if (ret < 0)
		return ret;

	drv->reg_cpu = ret;
	dev_dbg(dev, "SAW2 bound to CPU %d\n", drv->reg_cpu);

	/*
	 * Program initial voltage, otherwise registration will also try
	 * setting the voltage, which might result in undervolting the CPU.
	 * init_uV is always inside the first range (Band 2 for 8660, the
	 * single 700-1400 mV range for 8064), so resolving against
	 * ranges[0] is sufficient.
	 */
	ret = linear_range_get_selector_high(&drv->reg_data->ranges[0],
					     drv->reg_data->init_uV,
					     &drv->volt_sel,
					     &found);
	if (ret) {
		dev_err(dev, "Initial uV value out of bounds\n");
		return ret;
	}

	/* Always do the SAW register writes on the corresponding CPU */
	smp_call_function_single(drv->reg_cpu, drv->reg_data->set_vdd, drv, true);

	rdev = devm_regulator_register(dev, rdesc, &config);
	if (IS_ERR(rdev)) {
		dev_err(dev, "failed to register regulator\n");
		return PTR_ERR(rdev);
	}

	return 0;
}

static const struct of_device_id spm_match_table[] = {
	{ .compatible = "qcom,sdm660-gold-saw2-v4.1-l2",
	  .data = &spm_reg_660_gold_l2 },
	{ .compatible = "qcom,sdm660-silver-saw2-v4.1-l2",
	  .data = &spm_reg_660_silver_l2 },
	{ .compatible = "qcom,msm8226-saw2-v2.1-cpu",
	  .data = &spm_reg_8226_cpu },
	{ .compatible = "qcom,msm8909-saw2-v3.0-cpu",
	  .data = &spm_reg_8909_cpu },
	{ .compatible = "qcom,msm8916-saw2-v3.0-cpu",
	  .data = &spm_reg_8916_cpu },
	{ .compatible = "qcom,msm8939-saw2-v3.0-cpu",
	  .data = &spm_reg_8939_cpu },
	{ .compatible = "qcom,msm8974-saw2-v2.1-cpu",
	  .data = &spm_reg_8974_8084_cpu },
	{ .compatible = "qcom,msm8976-gold-saw2-v2.3-l2",
	  .data = &spm_reg_8976_gold_l2 },
	{ .compatible = "qcom,msm8976-silver-saw2-v2.3-l2",
	  .data = &spm_reg_8976_silver_l2 },
	{ .compatible = "qcom,msm8998-gold-saw2-v4.1-l2",
	  .data = &spm_reg_8998_gold_l2 },
	{ .compatible = "qcom,msm8998-silver-saw2-v4.1-l2",
	  .data = &spm_reg_8998_silver_l2 },
	{ .compatible = "qcom,apq8084-saw2-v2.1-cpu",
	  .data = &spm_reg_8974_8084_cpu },
	{ .compatible = "qcom,apq8064-saw2-v1.1-cpu",
	  .data = &spm_reg_8064_cpu },
	{ .compatible = "qcom,msm8660-saw2-v1.1-cpu",
	  .data = &spm_reg_8660_cpu },
	{ .compatible = "qcom,apq8060-saw2-v1.1-cpu",
	  .data = &spm_reg_8660_cpu },
	{ },
};
MODULE_DEVICE_TABLE(of, spm_match_table);

static int spm_dev_probe(struct platform_device *pdev)
{
	const struct of_device_id *match_id;
	struct spm_driver_data *drv;
	void __iomem *addr;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(drv->reg_base))
		return PTR_ERR(drv->reg_base);

	match_id = of_match_node(spm_match_table, pdev->dev.of_node);
	if (!match_id)
		return -ENODEV;

	drv->reg_data = match_id->data;
	drv->dev = &pdev->dev;
	platform_set_drvdata(pdev, drv);

	/* Write the SPM sequences first (if supported by this SoC) */
	if (drv->reg_data->reg_offset[SPM_REG_SEQ_ENTRY]) {
		addr = drv->reg_base + drv->reg_data->reg_offset[SPM_REG_SEQ_ENTRY];
		__iowrite32_copy(addr, drv->reg_data->seq,
				ARRAY_SIZE(drv->reg_data->seq) / 4);
	}

	/*
	 * ..and then the control registers.
	 * On some SoC if the control registers are written first and if the
	 * CPU was held in reset, the reset signal could trigger the SPM state
	 * machine, before the sequences are completely written.
	 */
	if (drv->reg_data->spm_cfg) {
		spm_register_write(drv, SPM_REG_AVS_CTL, drv->reg_data->avs_ctl);
		spm_register_write(drv, SPM_REG_AVS_LIMIT, drv->reg_data->avs_limit);
		spm_register_write(drv, SPM_REG_CFG, drv->reg_data->spm_cfg);
		spm_register_write(drv, SPM_REG_DLY, drv->reg_data->spm_dly);
		spm_register_write(drv, SPM_REG_PMIC_DLY, drv->reg_data->pmic_dly);
		spm_register_write(drv, SPM_REG_PMIC_DATA_0,
					drv->reg_data->pmic_data[0]);
		spm_register_write(drv, SPM_REG_PMIC_DATA_1,
					drv->reg_data->pmic_data[1]);

		/*
		 * MSM8660-specific SAW v1.0 initialization.
		 * Bootloaders (HTC TrustZone, HTC SBL3, TouchPad APPSBL) do NOT
		 * initialize SPM registers. Analysis shows zero SAW base address
		 * references in any bootloader. Hardware reads SPM_CTL=0x08 at boot
		 * (default) vs required 0x68. Must explicitly initialize all 11
		 * registers matching legacy webOS kernel.
		 */
		if (drv->reg_data->no_seq_ram) {
			u32 slp_clk_en;
			u32 cpu_idx = 0;
			struct device_node *cpu_node;
			int cpu;

			/*
			 * Find which CPU node references this SAW via qcom,saw phandle.
			 * CPU nodes have reg=<0> or reg=<1> which is the actual CPU index.
			 */
			for_each_possible_cpu(cpu) {
				cpu_node = of_cpu_device_node_get(cpu);
				if (cpu_node) {
					struct device_node *saw_node;
					saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
					of_node_put(cpu_node);
					if (saw_node == pdev->dev.of_node) {
						of_node_put(saw_node);
						cpu_idx = cpu;
						break;
					}
					of_node_put(saw_node);
				}
			}

			/*
			 * CPU0 uses SLP_CLK_EN=0x01, CPU1 uses 0x13 (enables
			 * clock for different sleep modes). This is a hardware
			 * asymmetry in the platform.
			 */
			slp_clk_en = (cpu_idx == 1) ? 0x13 : drv->reg_data->slp_clk_en;

			dev_info(&pdev->dev, "SAW init: CPU%u, writing 11 registers\n", cpu_idx);

			spm_register_write(drv, SPM_REG_WAKE_TMR_DLY,
					   drv->reg_data->wake_tmr_dly);
			spm_register_write(drv, SPM_REG_SLP_CLK_EN, slp_clk_en);
			spm_register_write(drv, SPM_REG_SLP_HSFS_PRECLMP_EN,
					   drv->reg_data->slp_hsfs_preclmp_en);
			spm_register_write(drv, SPM_REG_SLP_HSFS_POSTCLMP_EN,
					   drv->reg_data->slp_hsfs_postclmp_en);
			spm_register_write(drv, SPM_REG_SLP_CLMP_EN,
					   drv->reg_data->slp_clmp_en);
			spm_register_write(drv, SPM_REG_RST,
					   drv->reg_data->slp_rst_en_init);
			spm_register_write(drv, SPM_REG_SPM_MPM_CFG,
					   drv->reg_data->spm_mpm_cfg);

			/*
			 * Write initial SPM_CTL (clock gating mode at boot).
			 * spm_set_low_power_mode() will update this at runtime.
			 */
			if (drv->reg_data->spm_ctl_init)
				spm_register_write(drv, SPM_REG_SPM_CTL,
						   drv->reg_data->spm_ctl_init);

			dev_info(&pdev->dev, "SAW init complete: SPM_CTL=0x%x SLP_CLK_EN=0x%x\n",
				 drv->reg_data->spm_ctl_init, slp_clk_en);
		}

		/* Set up Standby as the default low power mode */
		if (drv->reg_data->reg_offset[SPM_REG_SPM_CTL])
			spm_set_low_power_mode(drv, PM_SLEEP_MODE_STBY);
	}

	if (IS_ENABLED(CONFIG_REGULATOR))
		return spm_register_regulator(&pdev->dev, drv);

	return 0;
}

static struct platform_driver spm_driver = {
	.probe = spm_dev_probe,
	.driver = {
		.name = "qcom_spm",
		.of_match_table = spm_match_table,
	},
};

static int __init qcom_spm_init(void)
{
	return platform_driver_register(&spm_driver);
}
arch_initcall(qcom_spm_init);

MODULE_DESCRIPTION("Qualcomm Subsystem Power Manager (SPM)");
MODULE_LICENSE("GPL v2");
