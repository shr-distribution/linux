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
 * csid_8x60_configure_stream - Configure CSID stream (minimal on MSM8660)
 * @csid: CSID device
 * @enable: Enable or disable stream
 *
 * On MSM8660, most configuration is done in CSIPHY. This function
 * provides minimal setup for the V4L2 pipeline.
 */
static void csid_8x60_configure_stream(struct csid_device *csid, u8 enable)
{
	struct csid_phy_config *phy = &csid->phy;

	dev_dbg(csid->camss->dev,
		"CSID%d: configure_stream enable=%d phy=%d lanes=%d\n",
		csid->id, enable, phy->csiphy_id, phy->lane_cnt);

	/*
	 * On MSM8660, the CSI decoder is integrated with CSIPHY.
	 * The CSIPHY driver has already configured the protocol
	 * control registers. Here we just need to ensure interrupts
	 * are properly configured.
	 */
	if (enable && csid->base) {
		/* Ensure interrupts are masked (CSIPHY handles them) */
		writel_relaxed(0, csid->base + MIPI_INTERRUPT_MASK);
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
