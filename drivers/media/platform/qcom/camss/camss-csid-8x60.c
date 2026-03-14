// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csid-8x60.c
 *
 * Qualcomm MSM8660/APQ8060 Camera Subsystem - CSID Module
 *
 * On MSM8660/APQ8060, the CSI decoder functionality is integrated into the
 * CSIPHY hardware block. This driver provides a minimal CSID implementation
 * to satisfy the V4L2 media pipeline requirements.
 *
 * The actual CSI protocol decoding is handled by the CSIPHY driver
 * (camss-csiphy-8x60.c) via the MIPI_PROTOCOL_CONTROL register.
 *
 * Based on webOS kernel msm_io_8x60.c implementation.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2025 Herrie (based on Code Aurora Forum VFE31 driver)
 */

#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "camss-csid.h"
#include "camss-csid-gen1.h"
#include "camss.h"

/*
 * MSM8660 CSID is integrated with CSIPHY. These registers are shared
 * with the CSIPHY block and are provided here for reference.
 */
#define MIPI_INTERRUPT_STATUS		0x08
#define MIPI_INTERRUPT_MASK		0x0C

/*
 * CSID Context ID (CID) Configuration Registers
 * These configure how the CSID routes MIPI packets based on Data Type and
 * Virtual Channel to the VFE PIX or RDI interfaces.
 *
 * On MSM8660, these may be at 0x0200 or 0x0020 depending on HW revision.
 * We try both - write to the more common 0x0020 offset first.
 */
#define CSID_CID_0_CFG			0x0020
#define CSID_CID_1_CFG			0x0024
#define CSID_CID_2_CFG			0x0028
#define CSID_CID_3_CFG			0x002C

/* Alternate offsets (some HW revisions) */
#define CSID_CID_0_CFG_ALT		0x0200

/* CID configuration bit shifts */
#define CSID_CID_DT_SHIFT		24
#define CSID_CID_VC_SHIFT		22
#define CSID_CID_DECODE_FORMAT_SHIFT	0

/* MIPI CSI-2 Data Types */
#define MIPI_DT_YUV422_8BIT		0x1E
#define MIPI_DT_RAW8			0x2A
#define MIPI_DT_RAW10			0x2B

/* CSID Decode Format values */
#define CSID_DECODE_FORMAT_YUV422	0x01
#define CSID_DECODE_FORMAT_RAW8		0x02
#define CSID_DECODE_FORMAT_RAW10	0x03

/*
 * csid_8x60_configure_stream - Configure CSID stream routing
 * @csid: CSID device
 * @enable: Enable or disable stream
 *
 * On MSM8660, the CSID must map incoming MIPI packets (by Data Type and
 * Virtual Channel) to the VFE PIX interface. Without this mapping, the
 * CSID drops packets and VFE receives no data.
 */
static void csid_8x60_configure_stream(struct csid_device *csid, u8 enable)
{
	struct csid_phy_config *phy = &csid->phy;
	u32 cid_cfg_val;

	dev_info(csid->camss->dev,
		 "CSID%d: configure_stream enable=%d phy=%d lanes=%d\n",
		 csid->id, enable, phy->csiphy_id, phy->lane_cnt);

	if (!enable)
		return;

	/*
	 * Configure CID_0 to route YUV422 8-bit data from VC0 to VFE PIX.
	 * The MT9M113 outputs YUV422 8-bit (MIPI DT = 0x1E) on VC 0.
	 *
	 * Note: We access these registers even though CSIPHY clocks may not
	 * be fully enabled yet. The CSID shares register space with CSIPHY,
	 * and the base AHB clock should be sufficient for register access.
	 * If this causes issues, this configuration may need to move to
	 * the CSIPHY driver's set_stream handler.
	 */
	cid_cfg_val = (MIPI_DT_YUV422_8BIT << CSID_CID_DT_SHIFT) |
		      (0x00 << CSID_CID_VC_SHIFT) |  /* VC 0 */
		      (CSID_DECODE_FORMAT_YUV422 << CSID_CID_DECODE_FORMAT_SHIFT);

	dev_info(csid->camss->dev,
		 "CSID%d: Writing CID_0_CFG=0x%08x (DT=0x%x VC=0 decode=YUV422)\n",
		 csid->id, cid_cfg_val, MIPI_DT_YUV422_8BIT);

	/* Try primary offset first */
	if (csid->base) {
		writel_relaxed(cid_cfg_val, csid->base + CSID_CID_0_CFG);
		wmb();

		/* Verify the write */
		{
			u32 readback = readl_relaxed(csid->base + CSID_CID_0_CFG);
			dev_info(csid->camss->dev,
				 "CSID%d: CID_0_CFG@0x%03x readback=0x%08x\n",
				 csid->id, CSID_CID_0_CFG, readback);

			/* If primary offset didn't work, try alternate */
			if (readback != cid_cfg_val) {
				writel_relaxed(cid_cfg_val, csid->base + CSID_CID_0_CFG_ALT);
				wmb();
				readback = readl_relaxed(csid->base + CSID_CID_0_CFG_ALT);
				dev_info(csid->camss->dev,
					 "CSID%d: CID_0_CFG@0x%03x (alt) readback=0x%08x\n",
					 csid->id, CSID_CID_0_CFG_ALT, readback);
			}
		}
	} else {
		dev_warn(csid->camss->dev,
			 "CSID%d: No base address, cannot configure CID!\n",
			 csid->id);
	}
}

/*
 * csid_8x60_configure_testgen_pattern - Configure test generator
 * @csid: CSID device
 * @val: Test pattern value
 *
 * MSM8660 doesn't have a CSID test generator. This is a no-op.
 */
static int csid_8x60_configure_testgen_pattern(struct csid_device *csid,
					       s32 val)
{
	/* MSM8660 CSID doesn't have test generator - use VFE testgen instead */
	return 0;
}

/*
 * csid_8x60_hw_version - Read CSID hardware version
 * @csid: CSID device
 *
 * MSM8660 doesn't have a CSID version register. Return a fixed value.
 */
static u32 csid_8x60_hw_version(struct csid_device *csid)
{
	/* Return a pseudo version for MSM8660 */
	return 0x8060;
}

/*
 * csid_8x60_isr - CSID interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * On MSM8660, CSI interrupts are handled by CSIPHY. This ISR handles
 * any CSID-specific interrupts.
 */
static irqreturn_t csid_8x60_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 status;

	if (!csid->base)
		return IRQ_NONE;

	status = readl_relaxed(csid->base + MIPI_INTERRUPT_STATUS);
	if (!status)
		return IRQ_NONE;

	/* Clear the interrupt */
	writel_relaxed(status, csid->base + MIPI_INTERRUPT_STATUS);

	dev_dbg(csid->camss->dev,
		"CSID%d: IRQ status=0x%08x\n", csid->id, status);

	return IRQ_HANDLED;
}

/*
 * csid_8x60_reset - Reset CSID module
 * @csid: CSID device
 *
 * On MSM8660, reset is handled by CSIPHY. Complete immediately.
 */
static int csid_8x60_reset(struct csid_device *csid)
{
	/* Reset is handled by CSIPHY on MSM8660 */
	complete(&csid->reset_complete);
	return 0;
}

/*
 * csid_8x60_src_pad_code - Get source pad format code
 * @csid: CSID device
 * @sink_code: Sink format code
 * @match_format_idx: Index to match
 * @match_code: Code to match
 *
 * MSM8660 CSID passes format through unchanged.
 */
static u32 csid_8x60_src_pad_code(struct csid_device *csid, u32 sink_code,
				  unsigned int match_format_idx, u32 match_code)
{
	/* Pass through - format is unchanged from CSIPHY */
	if (match_code)
		return match_code == sink_code ? match_code : 0;

	return match_format_idx == 0 ? sink_code : 0;
}

/*
 * csid_8x60_subdev_init - Initialize CSID subdevice
 * @csid: CSID device
 */
static void csid_8x60_subdev_init(struct csid_device *csid)
{
	/* MSM8660 CSID doesn't have testgen modes */
	csid->testgen.modes = NULL;
	csid->testgen.nmodes = 0;
}

const struct csid_hw_ops csid_ops_8x60 = {
	.configure_stream = csid_8x60_configure_stream,
	.configure_testgen_pattern = csid_8x60_configure_testgen_pattern,
	.hw_version = csid_8x60_hw_version,
	.isr = csid_8x60_isr,
	.reset = csid_8x60_reset,
	.src_pad_code = csid_8x60_src_pad_code,
	.subdev_init = csid_8x60_subdev_init,
};

/*
 * CSID formats supported on MSM8660
 * Same as 4_1 since the CSIPHY handles decoding
 */
static const struct csid_format_info csid_formats_8x60_info[] = {
	{
		MEDIA_BUS_FMT_UYVY8_1X16,
		0x1e,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_VYUY8_1X16,
		0x1e,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_YUYV8_1X16,
		0x1e,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_YVYU8_1X16,
		0x1e,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		2,
	},
	{
		MEDIA_BUS_FMT_SBGGR8_1X8,
		0x2a,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG8_1X8,
		0x2a,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG8_1X8,
		0x2a,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB8_1X8,
		0x2a,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8,
		1,
	},
	{
		MEDIA_BUS_FMT_SBGGR10_1X10,
		0x2b,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG10_1X10,
		0x2b,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG10_1X10,
		0x2b,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB10_1X10,
		0x2b,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
	{
		MEDIA_BUS_FMT_SBGGR12_1X12,
		0x2c,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SGBRG12_1X12,
		0x2c,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SGRBG12_1X12,
		0x2c,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_SRGGB12_1X12,
		0x2c,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12,
		1,
	},
	{
		MEDIA_BUS_FMT_Y10_1X10,
		0x2b,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10,
		1,
	},
};

const struct csid_formats csid_formats_8x60 = {
	.nformats = ARRAY_SIZE(csid_formats_8x60_info),
	.formats = csid_formats_8x60_info,
};
