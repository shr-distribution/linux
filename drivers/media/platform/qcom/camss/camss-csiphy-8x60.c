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
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

/*
 * Software SOF generation - disabled by default.
 *
 * When enabled, CSIPHY generates software SOF interrupts to VFE based on
 * MIPI SOT timing gaps. This is a workaround for sensors that don't send
 * MIPI Frame Start/End short packets.
 *
 * The hardware path (CSIPHY -> CSID -> VFE CAMIF) should generate proper
 * CAMIF_SOF interrupts without this. Enable only for debugging or if the
 * hardware path doesn't work.
 */
static bool software_sof_enable;
module_param(software_sof_enable, bool, 0644);
MODULE_PARM_DESC(software_sof_enable,
		 "Enable software SOF generation from CSIPHY (default: false)");

/*
 * Runtime-adjustable settle count for MIPI timing debug.
 * Valid range: 0x00-0xFF, default 0x14 (matches webOS MT9M113).
 * If ECC errors are high, try adjusting +/- 5 units.
 * - Too low: PHY samples while voltage unstable -> ECC errors
 * - Too high: PHY misses sync byte -> no valid packets
 */
static int settle_cnt_override = -1;
module_param(settle_cnt_override, int, 0644);
MODULE_PARM_DESC(settle_cnt_override,
		 "Override settle count (0x00-0xFF, -1=use calculated/default 0x14)");

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

/*
 * Default settle count for MSM8660.
 *
 * Testing results:
 *   0x10: Combined SOT+ECC errors
 *   0x12: Combined SOT+ECC errors
 *   0x14: Alternating individual SOT/ECC errors (best, webOS value)
 *   0x16: Combined SOT+ECC errors (regression)
 *
 * 0x14 is the optimal value matching webOS mt9m113_csi_params.settle_cnt.
 * MT9M113 at 96 MHz link freq: UI = 5.2ns, T-HS-SETTLE = 116-197ns
 */
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

	num_lanes = cfg->csi2->lane_cfg.num_data;

	/*
	 * Calculate settle count if link frequency is available.
	 * The settle count formula is derived from CSI2 timing requirements.
	 */
	dev_info(csiphy->camss->dev,
		 "CSIPHY%d: settle calc inputs: link_freq=%lld timer_clk_rate=%u\n",
		 csiphy->id, link_freq, csiphy->timer_clk_rate);

	if (link_freq > 0 && csiphy->timer_clk_rate > 0) {
		u32 ui_ps = div_u64(1000000000000ULL, link_freq) / 2;
		u32 timer_period_ps = div_u64(1000000000000ULL,
					      csiphy->timer_clk_rate);
		u32 t_hs_settle_ps;
		u32 t_hs_settle_ns;

		/* T_HS_SETTLE calculation based on MIPI D-PHY spec */
		t_hs_settle_ps = (85000 + 6 * ui_ps + 145000 + 10 * ui_ps) / 2;
		settle_cnt = t_hs_settle_ps / timer_period_ps;
		if (settle_cnt > 0)
			settle_cnt--;

		t_hs_settle_ns = (settle_cnt + 1) * timer_period_ps / 1000;
		dev_info(csiphy->camss->dev,
			 "CSIPHY%d: settle calc: UI=%u ps, timer_period=%u ps, "
			 "t_hs_settle=%u ps, settle_cnt=0x%02x (%u ns actual)\n",
			 csiphy->id, ui_ps, timer_period_ps, t_hs_settle_ps,
			 settle_cnt, t_hs_settle_ns);
	} else {
		dev_warn(csiphy->camss->dev,
			 "CSIPHY%d: Using default settle_cnt=0x%02x (link_freq=%lld, timer_clk=%u)\n",
			 csiphy->id, settle_cnt, link_freq, csiphy->timer_clk_rate);
	}

	/*
	 * Allow runtime override for debugging settle count issues.
	 * If ECC error rate is high, try: settle_cnt_override=0x10/0x18/0x1C
	 */
	if (settle_cnt_override >= 0 && settle_cnt_override <= 0xFF) {
		dev_info(csiphy->camss->dev,
			 "CSIPHY%d: settle_cnt OVERRIDE: 0x%02x -> 0x%02x\n",
			 csiphy->id, settle_cnt, settle_cnt_override);
		settle_cnt = settle_cnt_override;
	}

	dev_info(csiphy->camss->dev,
		 "CSIPHY%d: lanes_enable: lanes=%d settle_cnt=0x%02x link_freq=%lld base=%px\n",
		 csiphy->id, num_lanes, settle_cnt, link_freq, csiphy->base);

	/*
	 * EXACT webOS sequence from msm_io_8x60.c msm_camio_enable():
	 *
	 * 1. Enable all clocks (done in set_power)
	 * 2. msleep(10) - SINGLE delay after clock enable
	 * 3. Write D0-D3_CONTROL2 with LP_REC_EN=0
	 * 4. Write CL_CONTROL with LP_REC_EN=0
	 *
	 * webOS uses plain writel() without barriers between writes.
	 * The 10ms delay after clock enable is the ONLY required delay.
	 */

	/*
	 * Phase 1: msm_camio_enable() equivalent
	 * Single 10ms delay matching webOS exactly. Extra delays may cause
	 * the hardware to enter an unexpected state.
	 */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Phase 1 - msleep(10) then write D0-D3_CONTROL2\n", csiphy->id);
	msleep(10);

	val = (settle_cnt << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
	      (0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
	      (0x0 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
	      (0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing D0-D3_CONTROL2 = 0x%08x\n", csiphy->id, val);
	/* webOS uses plain writel() - sequential writes, no barriers between */
	writel(val, csiphy->base + MIPI_PHY_D0_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D2_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D3_CONTROL2);

	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
	      (0x0 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing CL_CONTROL = 0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_PHY_CL_CONTROL);

	dev_info(csiphy->camss->dev, "CSIPHY%d: Phase 1 complete\n", csiphy->id);

	/*
	 * Phase 2: msm_camio_csi_config() equivalent
	 * webOS calls this later, during sensor streaming setup.
	 * All writes use plain writel() with no barriers between.
	 */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Phase 2 - Config sequence\n", csiphy->id);

	/* PHY_CONTROL - SOT_ECC_EN */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing PHY_CONTROL=0x4\n", csiphy->id);
	writel(0x4, csiphy->base + MIPI_PHY_CONTROL);

	/* SW_RST to PROTOCOL_CONTROL */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing PROTOCOL_CONTROL SW_RST\n", csiphy->id);
	writel(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK, csiphy->base + MIPI_PROTOCOL_CONTROL);

	/* PROTOCOL_CONTROL with config */
	val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
	      MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
	      MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
	val |= (0x0 << MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing PROTOCOL_CONTROL=0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_PROTOCOL_CONTROL);

	/* CALIBRATION_CONTROL */
	val = (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT) |
	      (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT) |
	      (0x1 << MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT) |
	      (0x1 << MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing CALIBRATION_CONTROL=0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_CALIBRATION_CONTROL);

	/* D0-D3_CONTROL2 with LP_REC_EN=1 - overwrites Phase 1 values */
	val = (settle_cnt << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
	      (0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
	      (0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
	      (0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing D0-D3_CONTROL2 (LP_REC_EN=1) = 0x%08x\n",
		 csiphy->id, val);
	writel(val, csiphy->base + MIPI_PHY_D0_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D2_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D3_CONTROL2);
	dev_info(csiphy->camss->dev, "CSIPHY%d: D0-D3_CONTROL2 writes done\n", csiphy->id);

	/* CL_CONTROL with LP_REC_EN=1 */
	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
	      (0x1 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing CL_CONTROL=0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_PHY_CL_CONTROL);

	/* D0_CONTROL - HS receiver equalization */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing D0_CONTROL=0\n", csiphy->id);
	writel(0, csiphy->base + MIPI_PHY_D0_CONTROL);

	/* D1_CONTROL - enable PHY (release shutdown) */
	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
	      (0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing D1_CONTROL=0x%08x (PHY enable)\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL);

	/* D2_CONTROL and D3_CONTROL = 0 */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing D2/D3_CONTROL=0\n", csiphy->id);
	writel(0, csiphy->base + MIPI_PHY_D2_CONTROL);
	writel(0, csiphy->base + MIPI_PHY_D3_CONTROL);

	/*
	 * CAMERA_CNTL: Configure lane assignment and count
	 *
	 * From webOS msm_io_8x60.c msm_camio_csi_config():
	 * - lane_assign is passed from sensor driver (0xe4 for all sensors)
	 * - Bits 0-2: lane count (0x4=1 lane, 0x5=2 lanes, etc.)
	 * - Bits 8-15: lane assignment value
	 *
	 * All webOS sensors (mt9m113, mt9m114, ov7692, vx6953) use lane_assign=0xe4.
	 * Result: (0xe4 << 8) | count = 0xe404 for 1 lane
	 */
	switch (num_lanes) {
	case 1:
		val = 0xe4 << 8 | 0x4;
		break;
	case 2:
		val = 0xe4 << 8 | 0x5;
		break;
	case 3:
		val = 0xe4 << 8 | 0x6;
		break;
	case 4:
		val = 0xe4 << 8 | 0x7;
		break;
	default:
		val = 0xe4 << 8 | 0x4;
		break;
	}
	dev_info(csiphy->camss->dev, "CSIPHY%d: Writing CAMERA_CNTL=0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_CAMERA_CNTL);

	/* Configure interrupts - mask out de-featured errors */
	dev_info(csiphy->camss->dev, "CSIPHY%d: Configuring interrupts\n", csiphy->id);
	writel(0xFFF7F3FF, csiphy->base + MIPI_INTERRUPT_MASK);
	writel(0xFFF7F3FF, csiphy->base + MIPI_INTERRUPT_STATUS);

	/*
	 * Note: MSM8660 does NOT have separate CSID CID registers.
	 *
	 * Unlike later Qualcomm chips, MSM8660 has a unified CSIPHY+CSID
	 * architecture where data type filtering is automatic. WebOS kernel
	 * does not configure any CID registers.
	 *
	 * IMPORTANT: Do NOT write to offsets 0x0020 or 0x0200 here!
	 * - 0x0020 is MIPI_PHY_D1_CONTROL (already configured above)
	 * - Writing CID config there corrupts PHY lane enable bits
	 */

	dev_info(csiphy->camss->dev, "CSIPHY%d: lanes_enable complete\n", csiphy->id);

	/*
	 * Debug: Read back all configured registers to verify writes took effect.
	 * This helps diagnose issues where the hardware state doesn't match
	 * what we expect after configuration.
	 */
	{
		u32 rb_phy_control, rb_protocol, rb_camera_cntl;
		u32 rb_d0_ctrl2, rb_d1_ctrl, rb_cl_ctrl, rb_cal_ctrl;
		u32 rb_irq_mask;

		rb_phy_control = readl(csiphy->base + MIPI_PHY_CONTROL);
		rb_protocol = readl(csiphy->base + MIPI_PROTOCOL_CONTROL);
		rb_camera_cntl = readl(csiphy->base + MIPI_CAMERA_CNTL);
		rb_d0_ctrl2 = readl(csiphy->base + MIPI_PHY_D0_CONTROL2);
		rb_d1_ctrl = readl(csiphy->base + MIPI_PHY_D1_CONTROL);
		rb_cl_ctrl = readl(csiphy->base + MIPI_PHY_CL_CONTROL);
		rb_cal_ctrl = readl(csiphy->base + MIPI_CALIBRATION_CONTROL);
		rb_irq_mask = readl(csiphy->base + MIPI_INTERRUPT_MASK);

		dev_info(csiphy->camss->dev,
			 "CSIPHY%d READBACK: base=%px id=%d\n",
			 csiphy->id, csiphy->base, csiphy->id);
		dev_info(csiphy->camss->dev,
			 "  PHY_CONTROL(0x00)=0x%08x PROTOCOL(0x04)=0x%08x\n",
			 rb_phy_control, rb_protocol);
		dev_info(csiphy->camss->dev,
			 "  CAMERA_CNTL(0x24)=0x%08x (expect 0xe404 for 1 lane, 0xe405 for 2 lanes)\n",
			 rb_camera_cntl);
		dev_info(csiphy->camss->dev,
			 "  D0_CTRL2(0x38)=0x%08x D1_CTRL(0x20)=0x%08x\n",
			 rb_d0_ctrl2, rb_d1_ctrl);
		dev_info(csiphy->camss->dev,
			 "  CL_CTRL(0x48)=0x%08x CAL_CTRL(0x18)=0x%08x IRQ_MASK(0x0C)=0x%08x\n",
			 rb_cl_ctrl, rb_cal_ctrl, rb_irq_mask);

		/* Verify D1_CONTROL has PHY enabled (bits 8,9 set) */
		if (!(rb_d1_ctrl & 0x300)) {
			dev_err(csiphy->camss->dev,
				"CSIPHY%d ERROR: D1_CONTROL PHY not enabled! (0x%08x)\n",
				csiphy->id, rb_d1_ctrl);
		}

		/* Verify CAMERA_CNTL has correct lane count */
		if ((rb_camera_cntl & 0x7) != (num_lanes + 3)) {
			dev_warn(csiphy->camss->dev,
				 "CSIPHY%d WARNING: CAMERA_CNTL lane count mismatch: got 0x%x, expect 0x%x\n",
				 csiphy->id, rb_camera_cntl & 0x7, num_lanes + 3);
		}
	}
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
 * MSM8660 MIPI CSI IRQ status bit definitions
 * Based on webOS kernel msm_io_8x60.c and hardware observations.
 *
 * The MIPI_INTERRUPT_STATUS register reports various CSI-2 events:
 * - BIT(4): SOT_SYNC - Start of Transmission sync
 * - BIT(5): ECC_ERROR - ECC error detected (correctable)
 * - BIT(11): Unknown - possibly related to data reception
 * - BIT(16): FS (Frame Start) - Short packet frame start (if sensor sends it)
 * - BIT(17): FE (Frame End) - Short packet frame end
 * - BIT(22): Observed at ~6fps intervals on MSM8660/MT9M113 - purpose unclear
 * - BIT(20-23): Line count errors per lane
 *
 * Note: webOS driver doesn't use CSIPHY frame start detection.
 * Frame sync comes from VFE CAMIF internal sync, not CSIPHY IRQ bits.
 */
#define MIPI_IRQ_SOT_SYNC	BIT(4)
#define MIPI_IRQ_ECC_ERROR	BIT(5)
#define MIPI_IRQ_FRAME_START	BIT(16)
#define MIPI_IRQ_FRAME_END	BIT(17)

/*
 * MSM8660 SOF generation state - used for software SOF when sensor
 * doesn't send MIPI Frame Start/End short packets.
 *
 * Frame boundary detection heuristic:
 * - If we see MIPI_IRQ_FRAME_START, use it directly
 * - Otherwise, detect frame start by gap in SOT interrupts
 *   (vertical blanking period creates a larger gap than inter-line gaps)
 *
 * Typical timing at 30fps, 768 lines:
 * - Line time: ~43us (1/30/768)
 * - Frame gap (vertical blanking): ~500us to several ms
 * - Threshold: 200us gap indicates frame start
 */
#define CSIPHY_FRAME_GAP_THRESHOLD_NS	200000	/* 200us in nanoseconds */

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
	struct vfe_device *vfe;
	u32 status;
	ktime_t now;
	s64 gap_ns;
	bool frame_start_detected = false;
	static ktime_t last_sot_time;
	static int irq_count;
	static int sof_count;
	static u32 last_status;
	static bool first_sot = true;

	status = readl_relaxed(csiphy->base + MIPI_INTERRUPT_STATUS);

	/* Clear the interrupt */
	writel(status, csiphy->base + MIPI_INTERRUPT_STATUS);

	/* Count IRQs */
	irq_count++;

	/*
	 * Frame start detection:
	 * 1. If MIPI_IRQ_FRAME_START bit is set, use it (preferred)
	 * 2. Otherwise, use timing-based detection from SOT gaps
	 */
	if (status & MIPI_IRQ_FRAME_START) {
		frame_start_detected = true;
	} else if (status & MIPI_IRQ_SOT_SYNC) {
		now = ktime_get();

		if (first_sot) {
			/* First SOT after reset - treat as frame start */
			frame_start_detected = true;
			first_sot = false;
		} else {
			/* Check gap since last SOT */
			gap_ns = ktime_to_ns(ktime_sub(now, last_sot_time));
			if (gap_ns > CSIPHY_FRAME_GAP_THRESHOLD_NS) {
				/* Large gap - this is a new frame */
				frame_start_detected = true;
			}
		}
		last_sot_time = now;
	}

	/*
	 * Trigger software SOF if frame start detected AND software SOF is enabled.
	 *
	 * This is disabled by default because the hardware path (CSIPHY -> CSID ->
	 * VFE CAMIF) should generate proper CAMIF_SOF interrupts. Enable via:
	 *   echo 1 > /sys/module/qcom_camss/parameters/software_sof_enable
	 */
	if (software_sof_enable && frame_start_detected &&
	    csiphy->camss && csiphy->camss->vfe) {
		int line;

		vfe = &csiphy->camss->vfe[0];  /* Use first VFE */
		/* Send SOF to all VFE lines, just like VFE31 IRQ handler does */
		for (line = 0; line < vfe->res->line_num; line++)
			vfe_trigger_software_sof(vfe, line);
		sof_count++;

		/* Log SOF generation periodically */
		if ((sof_count % 30) == 1) {
			dev_info(csiphy->camss->dev,
				 "CSIPHY%d: Software SOF #%d triggered (IRQ #%d)\n",
				 csiphy->id, sof_count, irq_count);
		}
	}

	/*
	 * Log interrupt status - use dev_info for first 10 IRQs to diagnose
	 * what bits the hardware actually sets, then switch to dev_dbg.
	 */
	if (irq_count <= 10 || status != last_status || (irq_count % 100) == 0) {
		dev_info(csiphy->camss->dev,
			 "CSIPHY%d: IRQ #%d status=0x%08x [%s%s%s%s] sof_count=%d\n",
			 csiphy->id, irq_count, status,
			 (status & MIPI_IRQ_SOT_SYNC) ? "SOT " : "",
			 (status & MIPI_IRQ_ECC_ERROR) ? "ECC " : "",
			 (status & MIPI_IRQ_FRAME_START) ? "FS " : "",
			 (status & MIPI_IRQ_FRAME_END) ? "FE " : "",
			 sof_count);
		last_status = status;
	}

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
