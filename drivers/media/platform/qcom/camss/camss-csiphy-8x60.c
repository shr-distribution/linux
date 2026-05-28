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
 * Copyright (C) 2025 Herman van Hazendonk <github.com@herrie.org>
 */

#include "camss-csiphy.h"
#include "camss.h"

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
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

/*
 * MIPI_PROTOCOL_CONTROL Register (0x04) bit definitions
 *
 * This register controls CSI-2 protocol decoding and data format handling.
 *
 * Key fields:
 *   Bits [20:19] DATA_FORMAT - Controls pixel unpacking/decoding
 *                0 = 8-bit data (CSI_8BIT) - for YUV422 and RAW8
 *                1 = 10-bit data (CSI_10BIT) - for RAW10
 *                2 = 12-bit data - for RAW12
 *                3 = Reserved
 *
 *   Bit 18      DECODE_ID - Enable decode ID (purpose unclear on MSM8660)
 *   Bit 17      ECC_EN - Enable ECC error correction for packet headers
 *   Bit 21      LONG_PACKET_HEADER_CAPTURE - Capture long packet headers
 *   Bit 27      SW_RST - Software reset
 *
 * IMPORTANT: Unlike newer Qualcomm chips (MSM8974+), MSM8660 does NOT have
 * separate CSID CID_LUT registers for MIPI data type filtering. The integrated
 * CSIPHY+CSID architecture on MSM8660 passes ALL data types to VFE.
 *
 * The DATA_FORMAT field controls HOW data is decoded (8/10/12 bit), NOT which
 * MIPI data types (0x1E=YUV, 0x2A=RAW8, etc.) are accepted. All data types
 * should pass through regardless of this setting.
 *
 * From webOS msm_io_8x60.c msm_camio_csi_config():
 *   val |= (csi_params->data_format) << MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT;
 *   val |= csi_params->dpcm_scheme << MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT;
 *
 * webOS mt9m113 sensor uses: data_format=CSI_8BIT (0), settle_cnt=0x14
 */
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
	dev_dbg(csiphy->camss->dev,
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

	dev_dbg(csiphy->camss->dev, "CSIPHY%d: lanes_enable ENTER\n", csiphy->id);

	num_lanes = cfg->csi2->lane_cfg.num_data;

	/*
	 * Calculate settle count if link frequency is available.
	 * The settle count formula is derived from CSI2 timing requirements.
	 */
	dev_dbg(csiphy->camss->dev,
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
		dev_dbg(csiphy->camss->dev,
			 "CSIPHY%d: settle calc: UI=%u ps, timer_period=%u ps, "
			 "t_hs_settle=%u ps, settle_cnt=0x%02x (%u ns actual)\n",
			 csiphy->id, ui_ps, timer_period_ps, t_hs_settle_ps,
			 settle_cnt, t_hs_settle_ns);
	} else {
		dev_warn(csiphy->camss->dev,
			 "CSIPHY%d: Using default settle_cnt=0x%02x (link_freq=%lld, timer_clk=%u)\n",
			 csiphy->id, settle_cnt, link_freq, csiphy->timer_clk_rate);
	}

	dev_dbg(csiphy->camss->dev,
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
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Phase 1 - msleep(10) then write D0-D3_CONTROL2\n", csiphy->id);
	msleep(10);

	/*
	 * D0_CONTROL2 value verified against webOS: 0x140F0018
	 * - settle_cnt in bits 31:24
	 * - HS_TERM_IMP = 0x0F in bits 23:16 (can be overridden)
	 * - LP_REC_EN = 1 in bit 4 (CRITICAL for MIPI LP-to-HS detection)
	 * - ERR_SOT_HS_EN = 1 in bit 3
	 */
	{
		u8 hs_term_imp = 0x0F;  /* Default matches webOS */

		val = (settle_cnt << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
		      (hs_term_imp << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
		      (0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
		      (0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	}
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing D0-D3_CONTROL2 = 0x%08x\n", csiphy->id, val);
	/* webOS uses plain writel() - sequential writes, no barriers between */
	writel(val, csiphy->base + MIPI_PHY_D0_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D2_CONTROL2);
	writel(val, csiphy->base + MIPI_PHY_D3_CONTROL2);

	/*
	 * CL_CONTROL - webOS msm_camio_csi_config() writes:
	 * (0x0F << 24) | (0x1 << 2) = 0x0F000004
	 * - HS_TERM_IMP = 0x0F at bits [27:24]
	 * - LP_REC_EN = 0x1 at bit 2
	 */
	val = 0x0F000004;
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing CL_CONTROL = 0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_PHY_CL_CONTROL);

	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Phase 1 complete\n", csiphy->id);

	/*
	 * Phase 2: msm_camio_csi_config() equivalent
	 * webOS calls this later, during sensor streaming setup.
	 * All writes use plain writel() with no barriers between.
	 */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Phase 2 - Config sequence\n", csiphy->id);

	/* PHY_CONTROL - SOT_ECC_EN */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing PHY_CONTROL=0x4\n", csiphy->id);
	writel(0x4, csiphy->base + MIPI_PHY_CONTROL);

	/* SW_RST to PROTOCOL_CONTROL */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing PROTOCOL_CONTROL SW_RST\n", csiphy->id);
	writel(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK, csiphy->base + MIPI_PROTOCOL_CONTROL);

	/*
	 * PROTOCOL_CONTROL with config.
	 * DATA_FORMAT field (bits 20:19) must match the sensor output:
	 *   0 = 8-bit (YUV422, RAW8)
	 *   1 = 10-bit (RAW10)
	 *   2 = 12-bit (RAW12)
	 * From webOS msm_camio_csi_config():
	 *   val |= csi_params->data_format << DATA_FORMAT_SHFT
	 */
	{
		u32 data_fmt = 0; /* default 8-bit for YUV */
		struct media_pad *remote_pad;
		struct v4l2_subdev *sensor_sd;

		/* Detect sensor format to set correct data format */
		remote_pad = media_pad_remote_pad_first(&csiphy->pads[0]);
		if (remote_pad) {
			sensor_sd = media_entity_to_v4l2_subdev(remote_pad->entity);
			if (sensor_sd) {
				struct v4l2_subdev_format fmt = {
					.which = V4L2_SUBDEV_FORMAT_ACTIVE,
					.pad = remote_pad->index,
				};

				if (!v4l2_subdev_call(sensor_sd, pad, get_fmt, NULL, &fmt)) {
					switch (fmt.format.code) {
					case MEDIA_BUS_FMT_SBGGR10_1X10:
					case MEDIA_BUS_FMT_SGBRG10_1X10:
					case MEDIA_BUS_FMT_SGRBG10_1X10:
					case MEDIA_BUS_FMT_SRGGB10_1X10:
					case MEDIA_BUS_FMT_Y10_1X10:
						data_fmt = 1; /* 10-bit */
						break;
					case MEDIA_BUS_FMT_SBGGR12_1X12:
					case MEDIA_BUS_FMT_SGBRG12_1X12:
					case MEDIA_BUS_FMT_SGRBG12_1X12:
					case MEDIA_BUS_FMT_SRGGB12_1X12:
						data_fmt = 2; /* 12-bit */
						break;
					default:
						data_fmt = 0; /* 8-bit */
						break;
					}
				}
			}
		}

		val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
		      MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
		      MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
		val |= (data_fmt << MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT);
		/* Store for stream_on to re-apply after SW_RST */
		csiphy->data_format = data_fmt;
		dev_dbg(csiphy->camss->dev,
			"CSIPHY%d: PROTOCOL_CONTROL=0x%08x (data_fmt=%d)\n",
			csiphy->id, val, data_fmt);
		writel(val, csiphy->base + MIPI_PROTOCOL_CONTROL);
	}

	/* CALIBRATION_CONTROL and per-lane D0-D3_CONTROL2 configuration. */
	{
		u8 hs_term_imp = 0x0F;  /* Default matches webOS */
		int i;
		u32 cal_status;

		/*
		 * Enable software calibration with manual strength override,
		 * poll for calibration-done (bit 23), then program the per-lane
		 * settle count and HS termination impedance once calibration
		 * has completed.
		 */
		val = (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT) |
		      (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT) |
		      (0x1 << MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT) |
		      (0x1 << MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT);
		writel(val, csiphy->base + MIPI_CALIBRATION_CONTROL);

		/* Poll for calibration done (bit 23) - max 10ms */
		for (i = 0; i < 100; i++) {
			cal_status = readl(csiphy->base + MIPI_CALIBRATION_CONTROL);
			if (cal_status & BIT(23))
				break;
			udelay(100);
		}
		if (i == 100)
			dev_warn(csiphy->camss->dev,
				 "CSIPHY%d: calibration bit 23 not set after 10ms, status=0x%08x\n",
				 csiphy->id, cal_status);

		val = (settle_cnt << MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
		      (hs_term_imp << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
		      (0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
		      (0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);

		writel(val, csiphy->base + MIPI_PHY_D0_CONTROL2);
		writel(val, csiphy->base + MIPI_PHY_D1_CONTROL2);
		writel(val, csiphy->base + MIPI_PHY_D2_CONTROL2);
		writel(val, csiphy->base + MIPI_PHY_D3_CONTROL2);
		dev_dbg(csiphy->camss->dev, "CSIPHY%d: D0-D3_CONTROL2 writes done\n", csiphy->id);
	}

	/*
	 * CL_CONTROL - webOS msm_camio_csi_config() writes 0x0F000004:
	 * - HS_TERM_IMP = 0x0F at bits [27:24]
	 * - LP_REC_EN = 0x1 at bit 2
	 */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing CL_CONTROL=0x0F000004 (webOS)\n", csiphy->id);
	writel(0x0F000004, csiphy->base + MIPI_PHY_CL_CONTROL);

	/* D0_CONTROL - HS receiver equalization */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing D0_CONTROL=0\n", csiphy->id);
	writel(0, csiphy->base + MIPI_PHY_D0_CONTROL);

	/* D1_CONTROL - enable PHY (release shutdown) */
	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
	      (0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing D1_CONTROL=0x%08x (PHY enable)\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_PHY_D1_CONTROL);

	/* D2_CONTROL and D3_CONTROL = 0 */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing D2/D3_CONTROL=0\n", csiphy->id);
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
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Writing CAMERA_CNTL=0x%08x\n", csiphy->id, val);
	writel(val, csiphy->base + MIPI_CAMERA_CNTL);

	/*
	 * Configure interrupts.
	 * Enable ALL interrupt sources (0xFFFFFFFF) for debugging.
	 *
	 * webOS used 0xFFF7F3FF which masks bits 10,11,12. But polling shows
	 * BIT(11)=0x800 is the main activity without triggering IRQs.
	 * Enable all bits to see full interrupt behavior.
	 * IRQ_MASK: bit=1 means interrupt ENABLED.
	 */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: Configuring interrupts\n", csiphy->id);
	writel(0xFFFFFFFF, csiphy->base + MIPI_INTERRUPT_STATUS);  /* Clear pending */
	writel(0xFFFFFFFF, csiphy->base + MIPI_INTERRUPT_MASK);    /* Enable ALL */
	dev_dbg(csiphy->camss->dev, "CSIPHY%d: IRQ_MASK=0xFFFFFFFF (all enabled)\n", csiphy->id);

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

	dev_dbg(csiphy->camss->dev, "CSIPHY%d: lanes_enable complete\n", csiphy->id);

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

		dev_dbg(csiphy->camss->dev,
			 "CSIPHY%d READBACK: base=%px id=%d\n",
			 csiphy->id, csiphy->base, csiphy->id);
		dev_dbg(csiphy->camss->dev,
			 "  PHY_CONTROL(0x00)=0x%08x PROTOCOL(0x04)=0x%08x\n",
			 rb_phy_control, rb_protocol);
		dev_dbg(csiphy->camss->dev,
			 "  CAMERA_CNTL(0x24)=0x%08x (expect 0xe404 for 1 lane, 0xe405 for 2 lanes)\n",
			 rb_camera_cntl);
		dev_dbg(csiphy->camss->dev,
			 "  D0_CTRL2(0x38)=0x%08x D1_CTRL(0x20)=0x%08x\n",
			 rb_d0_ctrl2, rb_d1_ctrl);
		dev_dbg(csiphy->camss->dev,
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
#define MIPI_IRQ_LP_RX		BIT(3)	/* LP receive complete? */
#define MIPI_IRQ_SOT_SYNC	BIT(4)	/* Start of Transmission sync */
#define MIPI_IRQ_ECC_ERROR	BIT(5)	/* ECC error detected */
#define MIPI_IRQ_DATA_DL	BIT(11)	/* Data on data lane? */
#define MIPI_IRQ_FRAME_START	BIT(16)	/* Frame Start short packet */
#define MIPI_IRQ_FRAME_END	BIT(17)	/* Frame End short packet */
#define MIPI_IRQ_LONG_PKT	BIT(21)	/* Long packet header captured */

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

	/* Acknowledge and clear the interrupt. */
	writel(status, csiphy->base + MIPI_INTERRUPT_STATUS);

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
