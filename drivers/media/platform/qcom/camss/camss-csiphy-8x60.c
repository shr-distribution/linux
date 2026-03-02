// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csiphy-8x60.c
 *
 * Qualcomm MSM8660/APQ8060 Camera Subsystem - CSIPHY Module
 *
 * This driver implements the MIPI CSI PHY support for MSM8660/APQ8060.
 * The CSI controller on these SoCs has a different register layout
 * compared to newer Qualcomm chips.
 *
 * Based on webOS kernel msm_io_8x60.c implementation.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2025 Herrie (based on Code Aurora Forum VFE31 driver)
 */

#include "camss-csiphy.h"
#include "camss.h"

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

/* MSM8660 MIPI CSI Controller Register Offsets */
#define MIPI_PHY_CONTROL		0x00
#define MIPI_PROTOCOL_CONTROL		0x04
#define MIPI_INTERRUPT_STATUS		0x08
#define MIPI_INTERRUPT_MASK		0x0C
#define MIPI_CALIBRATION_CONTROL	0x18
#define MIPI_PHY_D1_CONTROL		0x20
#define MIPI_CAMERA_CNTL		0x24
#define MIPI_PHY_D2_CONTROL		0x2C
#define MIPI_PHY_D3_CONTROL		0x30
#define MIPI_PHY_D0_CONTROL		0x34
#define MIPI_PHY_D0_CONTROL2		0x38
#define MIPI_PHY_D1_CONTROL2		0x3C
#define MIPI_PHY_D2_CONTROL2		0x40
#define MIPI_PHY_D3_CONTROL2		0x44
#define MIPI_PHY_CL_CONTROL		0x48

/* MIPI_PROTOCOL_CONTROL bit definitions */
#define MIPI_PROTOCOL_CONTROL_SW_RST_BMSK			BIT(27)
#define MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK	BIT(21)
#define MIPI_PROTOCOL_CONTROL_DATA_FORMAT_BMSK			(0x3 << 19)
#define MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK			BIT(18)
#define MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK			BIT(17)

/* Shift values */
#define MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT		19
#define MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT		30

#define MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT		22
#define MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT 21
#define MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT		20
#define MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT	7

#define MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT		24
#define MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT		16
#define MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT		4
#define MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT		3

#define MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT		24
#define MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT		2

#define MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT		28
#define MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT	9
#define MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT 8

/* Default settle count for MSM8660 (from webOS kernel) */
#define MSM8660_DEFAULT_SETTLE_CNT	0x14

/*
 * csiphy_8x60_get_lane_mask - Calculate CSI2 lane mask
 * @lane_cfg: CSI2 lane configuration
 *
 * Return lane mask value for MSM8660
 */
static u8 csiphy_8x60_get_lane_mask(struct csiphy_lanes_cfg *lane_cfg)
{
	u8 lane_mask = 0;
	int i;

	/* Include clock lane (always enabled) */
	lane_mask = 1;

	/* Add data lanes */
	for (i = 0; i < lane_cfg->num_data; i++)
		lane_mask |= 1 << (lane_cfg->data[i].pos + 1);

	return lane_mask;
}

/*
 * csiphy_8x60_hw_version_read - Read CSIPHY hardware version
 * @csiphy: CSIPHY device
 * @dev: Device structure
 *
 * MSM8660 doesn't have a standard HW version register,
 * so this function just logs that it's the 8x60 variant.
 */
static void csiphy_8x60_hw_version_read(struct csiphy_device *csiphy,
					struct device *dev)
{
	dev_dbg(dev, "CSIPHY MSM8660 variant (id=%d)\n", csiphy->id);
}

/*
 * csiphy_8x60_reset - Perform software reset on CSIPHY module
 * @csiphy: CSIPHY device
 *
 * MSM8660 workaround: Cycle clocks before register access to ensure
 * the CSI hardware is in a known good state. This is required after
 * power domain transitions (GDSC enable/disable cycles).
 */
static void csiphy_8x60_reset(struct csiphy_device *csiphy)
{
	/*
	 * MSM8660: No reset operation needed here.
	 *
	 * WebOS doesn't cycle clocks or reset - it simply enables clocks
	 * in msm_camio_enable() and keeps them running. The full CSI init
	 * (PHY_CONTROL, SW_RST, config) happens in msm_camio_csi_config().
	 *
	 * We do the same: clocks are enabled in set_power, and the full
	 * initialization sequence happens in lanes_enable().
	 */
	dev_info(csiphy->camss->dev,
		 "CSIPHY%d: reset (no-op, init done in lanes_enable)\n",
		 csiphy->id);
}

/*
 * csiphy_8x60_lanes_enable - Enable CSIPHY lanes
 * @csiphy: CSIPHY device
 * @cfg: CSIPHY configuration
 * @link_freq: CSI2 link frequency
 * @lane_mask: Lane mask
 *
 * This implements the CSI initialization sequence from the webOS kernel.
 *
 * MSM8660 workaround: VFE defers CAMIF enable until this function completes.
 * This allows CSIPHY register access to work (CAMIF must not be enabled
 * while configuring CSIPHY). After configuring lanes, we call
 * vfe_enable_pending_camif() to enable the deferred CAMIF.
 */
static void csiphy_8x60_lanes_enable(struct csiphy_device *csiphy,
				     struct csiphy_config *cfg,
				     s64 link_freq, u8 lane_mask)
{
	int num_lanes;
	u8 settle_cnt = MSM8660_DEFAULT_SETTLE_CNT;
	u32 val;

	dev_info(csiphy->camss->dev, "CSIPHY%d: lanes_enable ENTER\n", csiphy->id);

	/*
	 * Clock cycling was done in reset(). Now do the full CSI init sequence
	 * following webOS msm_camio_csi_config() exactly:
	 * PHY_CONTROL -> SW_RST -> config (overwrite SW_RST, don't clear to 0)
	 */

	num_lanes = cfg->csi2->lane_cfg.num_data;

	/*
	 * Calculate settle count if link frequency is available.
	 * The settle count formula is derived from CSI2 timing requirements.
	 */
	if (link_freq > 0 && csiphy->timer_clk_rate > 0) {
		u32 ui_ps = div_u64(1000000000000ULL, link_freq) / 2;
		u32 timer_period_ps = div_u64(1000000000000ULL,
					      csiphy->timer_clk_rate);
		u32 t_hs_settle_ps;

		/* T_HS_SETTLE calculation based on MIPI D-PHY spec */
		t_hs_settle_ps = (85000 + 6 * ui_ps + 145000 + 10 * ui_ps) / 2;
		settle_cnt = t_hs_settle_ps / timer_period_ps;
		if (settle_cnt > 0)
			settle_cnt--;
	}

	dev_info(csiphy->camss->dev,
		 "CSIPHY%d: lanes_enable: lanes=%d settle_cnt=0x%02x link_freq=%lld base=%px\n",
		 csiphy->id, num_lanes, settle_cnt, link_freq, csiphy->base);

	/*
	 * Follow exact webOS msm_camio_csi_config() sequence:
	 * 1. PHY_CONTROL first (SOT_ECC_EN)
	 * 2. SW_RST to PROTOCOL_CONTROL
	 * 3. Config to PROTOCOL_CONTROL (immediately overwrites SW_RST)
	 */

	/* Step 1: PHY_CONTROL first - SOT_ECC_EN for sync data-lane */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 1 - PHY_CONTROL\n", csiphy->id);
	writel(0x4, csiphy->base + MIPI_PHY_CONTROL);

	/* Step 2: SW_RST to CSI core */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 2 - SW_RST\n", csiphy->id);
	writel(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK,
		       csiphy->base + MIPI_PROTOCOL_CONTROL);
	/* Allow reset to complete - use longer delay */
	usleep_range(10000, 15000);
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 2 - SW_RST delay done\n", csiphy->id);

	/* Step 3: PROTOCOL_CONTROL config - overwrites SW_RST */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 3 - PROTOCOL_CONTROL\n", csiphy->id);
	val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
	      MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
	      MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
	writel(val, csiphy->base + MIPI_PROTOCOL_CONTROL);

	/* Step 4: SW CAL EN - software calibration */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 4 - CALIBRATION_CONTROL\n", csiphy->id);
	val = (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT) |
	      (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT) |
	      (0x1 << MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT) |
	      (0x1 << MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT);
	writel(val, csiphy->base + MIPI_CALIBRATION_CONTROL);

	/* Step 5: Configure data lane timing (D0-D3) - settle count is speed-sensitive */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 5 - D0-D3_CONTROL2\n", csiphy->id);
	val = (settle_cnt << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
	      (0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
	      (0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
	      (0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	writel(val, csiphy->base + MIPI_PHY_D0_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D2_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D3_CONTROL2);

	/* Step 6: Configure clock lane */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 6 - CL_CONTROL\n", csiphy->id);
	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
	      (0x1 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	writel(val, csiphy->base + MIPI_PHY_CL_CONTROL);

	/* Step 7: D0 HS receiver equalization */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 7 - D0_CONTROL\n", csiphy->id);
	val = 0 << MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT;
	writel(val, csiphy->base + MIPI_PHY_D0_CONTROL);

	/* Step 8: Enable PHY - release shutdown (CLK and DATA PHY) */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 8 - D1_CONTROL (PHY enable)\n", csiphy->id);
	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
	      (0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL);

	/* Step 9: Disable unused D2/D3 lanes */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 9 - D2/D3_CONTROL\n", csiphy->id);
	writel(0x00000000, csiphy->base + MIPI_PHY_D2_CONTROL);
	writel(0x00000000, csiphy->base + MIPI_PHY_D3_CONTROL);

	/* Step 10: Configure lane count in CAMERA_CNTL */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 10 - CAMERA_CNTL\n", csiphy->id);
	switch (num_lanes) {
	case 1:
		val = 0x4; /* 1 lane */
		break;
	case 2:
		val = 0x5; /* 2 lanes */
		break;
	case 3:
		val = 0x6; /* 3 lanes */
		break;
	case 4:
		val = 0x7; /* 4 lanes */
		break;
	default:
		val = 0x4; /* Default to 1 lane */
		break;
	}
	/* Lane assignment in upper bits (usually 0) */
	writel(val, csiphy->base + MIPI_CAMERA_CNTL);

	/* Step 11: Configure interrupts */
	dev_info(csiphy->camss->dev, "CSIPHY%d: step 11 - INTERRUPT config\n", csiphy->id);
	/* Mask out ID_ERROR[19], DATA_CMM_ERR[11], CLK_CMM_ERR[10] */
	writel(0xFFF7F3FF, csiphy->base + MIPI_INTERRUPT_MASK);
	/* Clear any pending IRQ bits */
	writel(0xFFF7F3FF, csiphy->base + MIPI_INTERRUPT_STATUS);

	/* Ensure all writes are committed */
	wmb();

	dev_info(csiphy->camss->dev, "CSIPHY%d: lanes_enable complete\n", csiphy->id);
}

/*
 * csiphy_8x60_lanes_disable - Disable CSIPHY lanes
 * @csiphy: CSIPHY device
 * @cfg: CSIPHY configuration
 */
static void csiphy_8x60_lanes_disable(struct csiphy_device *csiphy,
				      struct csiphy_config *cfg)
{
	/* Disable PHY by asserting shutdown */
	writel(0, csiphy->base + MIPI_PHY_D1_CONTROL);

	/* Disable interrupts */
	writel(0, csiphy->base + MIPI_INTERRUPT_MASK);

	/* Clear interrupt status */
	writel(0xFFFFFFFF, csiphy->base + MIPI_INTERRUPT_STATUS);

	/* Reset the CSI controller */
	csiphy_8x60_reset(csiphy);
}

/*
 * csiphy_8x60_isr - CSIPHY interrupt service routine
 * @irq: Interrupt line
 * @dev: CSIPHY device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csiphy_8x60_isr(int irq, void *dev)
{
	struct csiphy_device *csiphy = dev;
	u32 status;

	status = readl_relaxed(csiphy->base + MIPI_INTERRUPT_STATUS);

	/* Clear the interrupt */
	writel(status, csiphy->base + MIPI_INTERRUPT_STATUS);

	if (status)
		dev_dbg(csiphy->camss->dev,
			"CSIPHY%d: IRQ status=0x%08x\n", csiphy->id, status);

	return IRQ_HANDLED;
}

static int csiphy_8x60_init(struct csiphy_device *csiphy)
{
	/* No special initialization needed for MSM8660 CSIPHY */
	return 0;
}

const struct csiphy_hw_ops csiphy_ops_8x60 = {
	.get_lane_mask = csiphy_8x60_get_lane_mask,
	.hw_version_read = csiphy_8x60_hw_version_read,
	.reset = csiphy_8x60_reset,
	.lanes_enable = csiphy_8x60_lanes_enable,
	.lanes_disable = csiphy_8x60_lanes_disable,
	.isr = csiphy_8x60_isr,
	.init = csiphy_8x60_init,
};
