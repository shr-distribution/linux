// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-3-1.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v3.1
 *
 * Copyright (C) 2025 (based on Code Aurora Forum VFE31 driver)
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2018 Linaro Ltd.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-gen1.h"

/* VFE 3.1 Register Offsets - based on MSM8x60 VFE31 */
#define VFE_0_HW_VERSION		0x000

#define VFE_0_GLOBAL_RESET_CMD		0x004
#define VFE_0_GLOBAL_RESET_CMD_CORE	BIT(0)
#define VFE_0_GLOBAL_RESET_CMD_CAMIF	BIT(1)
#define VFE_0_GLOBAL_RESET_CMD_BUS	BIT(2)
#define VFE_0_GLOBAL_RESET_CMD_BUS_BDG	BIT(3)
#define VFE_0_GLOBAL_RESET_CMD_REGISTER	BIT(4)
#define VFE_0_GLOBAL_RESET_CMD_TIMER	BIT(5)
#define VFE_0_GLOBAL_RESET_CMD_PM	BIT(6)
#define VFE_0_GLOBAL_RESET_CMD_BUS_MISR	BIT(7)
#define VFE_0_GLOBAL_RESET_CMD_TESTGEN	BIT(8)

#define VFE_0_CGC_OVERRIDE		0x00C
#define VFE_0_CGC_OVERRIDE_1		0x00C

#define VFE_0_MODULE_CFG		0x010
#define VFE_0_MODULE_CFG_DEMUX		BIT(2)
#define VFE_0_MODULE_CFG_CHROMA_UPSAMPLE BIT(3)
#define VFE_0_MODULE_CFG_SCALE_ENC	BIT(23)
#define VFE_0_MODULE_CFG_CROP_ENC	BIT(27)

#define VFE_0_CORE_CFG			0x014
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR	0x4
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB	0x5
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY	0x6
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY	0x7

#define VFE_0_IRQ_CMD			0x018
#define VFE_0_IRQ_CMD_GLOBAL_CLEAR	BIT(0)

#define VFE_0_IRQ_MASK_0		0x01C
#define VFE_0_IRQ_MASK_0_CAMIF_SOF			BIT(0)
#define VFE_0_IRQ_MASK_0_CAMIF_EOF			BIT(1)
#define VFE_0_IRQ_MASK_0_REG_UPDATE			BIT(5)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 21)
#define VFE_0_IRQ_MASK_0_STATS_COMPOSITE		BIT(24)
#define VFE_0_IRQ_MASK_0_RESET_ACK			BIT(31)
#define VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? BIT(5) : 0)

#define VFE_0_IRQ_MASK_1		0x020
#define VFE_0_IRQ_MASK_1_CAMIF_ERROR			BIT(0)
#define VFE_0_IRQ_MASK_1_VIOLATION			BIT(7)
/*
 * VFE31 reset acknowledge is in STATUS_1 bit 22, not STATUS_0 bit 31.
 * This differs from later VFE versions.
 */
#define VFE_0_IRQ_MASK_1_RESET_ACK			BIT(22)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		BIT(23)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	BIT((n) + 9)

#define VFE_0_IRQ_CLEAR_0		0x024
#define VFE_0_IRQ_CLEAR_1		0x028

#define VFE_0_IRQ_STATUS_0		0x02C
#define VFE_0_IRQ_STATUS_0_CAMIF_SOF			BIT(0)
#define VFE_0_IRQ_STATUS_0_REG_UPDATE			BIT(5)
#define VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 21)
#define VFE_0_IRQ_STATUS_0_STATS_COMPOSITE		BIT(24)
#define VFE_0_IRQ_STATUS_0_RESET_ACK			BIT(31)
#define VFE_0_IRQ_STATUS_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? BIT(5) : 0)

#define VFE_0_IRQ_STATUS_1		0x030
#define VFE_0_IRQ_STATUS_1_VIOLATION			BIT(7)
/*
 * VFE31 reset acknowledge is in STATUS_1 bit 22, not STATUS_0 bit 31.
 * This differs from later VFE versions.
 */
#define VFE_0_IRQ_STATUS_1_RESET_ACK			BIT(22)
#define VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK		BIT(23)

#define VFE_0_IRQ_COMPOSITE_MASK_0	0x034
#define VFE_0_VIOLATION_STATUS		0x048

#define VFE_0_BUS_CMD			0x038
#define VFE_0_BUS_CMD_Mx_RLD_CMD(x)	BIT(x)

#define VFE_0_BUS_CFG			0x03C
/*
 * VFE31 BUS_CFG register bit definitions:
 * Bit 0: stripeRdPathEn
 * Bits 1-3: reserved
 * Bit 4: encYWrPathEn - Enable encoder Y write path
 * Bit 5: encCbcrWrPathEn - Enable encoder CbCr write path
 * Bit 6: viewYWrPathEn - Enable view Y write path
 * Bit 7: viewCbcrWrPathEn - Enable view CbCr write path
 * Bits 8-9: rawPixelDataSize (0=8bit, 1=10bit, 2=12bit)
 * Bits 10-11: rawWritePathSelect (0=disabled, 1=enc_cbcr, 2=view_cbcr)
 */
#define VFE_0_BUS_CFG_ENC_Y_WR_PATH_EN		BIT(4)
#define VFE_0_BUS_CFG_ENC_CBCR_WR_PATH_EN	BIT(5)
#define VFE_0_BUS_CFG_VIEW_Y_WR_PATH_EN		BIT(6)
#define VFE_0_BUS_CFG_VIEW_CBCR_WR_PATH_EN	BIT(7)
#define VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT	8
#define VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT	10
#define VFE_0_BUS_CFG_RAW_WR_PATH_DISABLED	0
#define VFE_0_BUS_CFG_RAW_WR_PATH_ENC_CBCR	1
#define VFE_0_BUS_CFG_RAW_WR_PATH_VIEW_CBCR	2

/*
 * VFE31 VFE_CFG register at 0x01C contains camif2vfeEnable and camif2busEnable
 * This enables raw passthrough mode (CAMIF -> AXI bus directly)
 */
#define VFE_0_VFE_CFG			0x01C
#define VFE_0_VFE_CFG_CAMIF_TO_VFE_EN	BIT(5)
#define VFE_0_VFE_CFG_CAMIF_TO_BUS_EN	BIT(7)

/* CAMIF configuration - VFE31 specific */
#define VFE_0_CAMIF_CMD			0x1EC
#define VFE_0_CAMIF_CMD_DISABLE_FRAME_BOUNDARY	0x0
#define VFE_0_CAMIF_CMD_ENABLE_FRAME_BOUNDARY	0x1
#define VFE_0_CAMIF_CMD_CLEAR_CAMIF_STATUS	BIT(2)
#define VFE_0_CAMIF_CMD_NO_CHANGE		0x0

#define VFE_0_CAMIF_CFG			0x1E4
#define VFE_0_CAMIF_CFG_VFE_OUTPUT_EN	BIT(6)

#define VFE_0_CAMIF_FRAME_CFG		0x1E8
#define VFE_0_CAMIF_WINDOW_WIDTH_CFG	0x1F0
#define VFE_0_CAMIF_WINDOW_HEIGHT_CFG	0x1F4
#define VFE_0_CAMIF_SUBSAMPLE_CFG_0	0x1F8
#define VFE_0_CAMIF_IRQ_SUBSAMPLE_PATTERN 0x1FC

#define VFE_0_CAMIF_STATUS		0x1E0

/* RDI configuration */
#define VFE_0_RDI_CFG_x(x)		(0x1E4 + (x) * 4)
#define VFE_0_RDI_CFG_x_MIPI_EN_BITS	0x3

/* AXI bus configuration */
#define VFE_0_AXI_CMD			0x1D8
#define VFE_0_AXI_CMD_HALT		BIT(0)

#define VFE_0_AXI_STATUS		0x1DC
#define VFE_0_AXI_STATUS_HALT_ACK	BIT(0)

#define VFE_0_BUS_PING_PONG_STATUS	0x180

/* Bus image masters - VFE31 uses different offsets */
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(n)		(0x06C + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(n)	(0x070 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(n)	(0x074 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(n)		(0x078 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(n)		(0x07C + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(n)	(0x080 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_BUFFER_CFG(n)	(0x084 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(n) (0x088 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PERIOD(n)	(0x08C + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(n) (0x090 + 0x24 * (n))

/* Demux configuration */
#define VFE_0_DEMUX_CFG			0x284
#define VFE_0_DEMUX_CFG_PERIOD		0x3
#define VFE_0_DEMUX_GAIN_0		0x288
#define VFE_0_DEMUX_GAIN_0_CH0_EVEN	(0x80 << 0)
#define VFE_0_DEMUX_GAIN_0_CH0_ODD	(0x80 << 16)
#define VFE_0_DEMUX_GAIN_1		0x28C
#define VFE_0_DEMUX_GAIN_1_CH1		(0x80 << 0)
#define VFE_0_DEMUX_GAIN_1_CH2		(0x80 << 16)

/* Chroma subsample */
#define VFE_0_CHROMA_SUBS_CFG		0x4F8

/* Scale configuration */
#define VFE_0_SCALE_ENC_Y_CFG		0x368
#define VFE_0_SCALE_ENC_CBCR_CFG	0x36C

/* Crop configuration */
#define VFE_0_CROP_ENC_Y_WIDTH		0x378
#define VFE_0_CROP_ENC_Y_HEIGHT		0x37C
#define VFE_0_CROP_ENC_CBCR_WIDTH	0x380
#define VFE_0_CROP_ENC_CBCR_HEIGHT	0x384

/* Output clamp */
#define VFE_0_CLAMP_ENC_MAX_CFG		0x524
#define VFE_0_CLAMP_ENC_MIN_CFG		0x528

/* Realign configuration */
#define VFE_0_REALIGN_BUF_CFG		0x388

/* Statistics configuration */
#define VFE_0_STATS_AE_CFG		0x534
#define VFE_0_STATS_AF_CFG		0x53C
#define VFE_0_STATS_AWB_CFG		0x54C
#define VFE_0_STATS_RS_CFG		0x56C
#define VFE_0_STATS_CS_CFG		0x574
#define VFE_0_STATS_IHIST_CFG		0x57C

/* Bus XBAR configuration */
#define VFE_0_BUS_XBAR_CFG_x(x)		(0x058 + 0x4 * ((x) / 2))

/* Register update */
#define VFE_0_REG_UPDATE_CMD		0x260
#define VFE_0_REG_UPDATE_CMD_UPDATE	BIT(0)

#define MSM_VFE_VFE0_UB_SIZE		1023
#define MSM_VFE_VFE0_UB_SIZE_RDI	127

static inline void vfe_reg_clr(struct vfe_device *vfe, u32 reg, u32 clr_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits & ~clr_bits, vfe->base + reg);
}

static inline void vfe_reg_set(struct vfe_device *vfe, u32 reg, u32 set_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits | set_bits, vfe->base + reg);
}

static u32 vfe31_hw_version(struct vfe_device *vfe)
{
	u32 hw_version = readl_relaxed(vfe->base + VFE_0_HW_VERSION);

	dev_dbg(vfe->camss->dev, "VFE HW Version = 0x%08x\n", hw_version);

	return hw_version;
}

static inline void vfe31_reg_update(struct vfe_device *vfe,
				    enum vfe_line_id line_id)
{
	/* VFE31 uses a simple register update mechanism */
	writel_relaxed(VFE_0_REG_UPDATE_CMD_UPDATE,
		       vfe->base + VFE_0_REG_UPDATE_CMD);
}

static inline void vfe31_reg_update_clear(struct vfe_device *vfe,
					  enum vfe_line_id line_id)
{
	/* VFE31 doesn't need explicit clear - auto-clears */
}

static void vfe31_global_reset(struct vfe_device *vfe)
{
	u32 reset_bits = VFE_0_GLOBAL_RESET_CMD_TESTGEN |
			 VFE_0_GLOBAL_RESET_CMD_BUS_MISR |
			 VFE_0_GLOBAL_RESET_CMD_PM |
			 VFE_0_GLOBAL_RESET_CMD_TIMER |
			 VFE_0_GLOBAL_RESET_CMD_REGISTER |
			 VFE_0_GLOBAL_RESET_CMD_BUS_BDG |
			 VFE_0_GLOBAL_RESET_CMD_BUS |
			 VFE_0_GLOBAL_RESET_CMD_CAMIF |
			 VFE_0_GLOBAL_RESET_CMD_CORE;

	/*
	 * Enable RESET_ACK interrupt before triggering reset.
	 * The vfe_reset() function waits for this interrupt to confirm
	 * the reset completed. Without enabling it first, we get a timeout.
	 *
	 * Note: VFE31 reset acknowledge is in IRQ_STATUS_1 bit 22,
	 * not IRQ_STATUS_0 bit 31 like later VFE versions.
	 */
	writel_relaxed(VFE_0_IRQ_MASK_1_RESET_ACK, vfe->base + VFE_0_IRQ_MASK_1);
	wmb();

	writel_relaxed(reset_bits, vfe->base + VFE_0_GLOBAL_RESET_CMD);
}

static void vfe31_halt_request(struct vfe_device *vfe)
{
	writel_relaxed(VFE_0_AXI_CMD_HALT, vfe->base + VFE_0_AXI_CMD);
}

static void vfe31_halt_clear(struct vfe_device *vfe)
{
	writel_relaxed(0x0, vfe->base + VFE_0_AXI_CMD);
}

static void vfe31_violation_read(struct vfe_device *vfe)
{
	u32 violation = readl_relaxed(vfe->base + VFE_0_VIOLATION_STATUS);

	dev_dbg(vfe->camss->dev, "VFE violation status: 0x%x\n", violation);
}

static void vfe31_isr_read(struct vfe_device *vfe, u32 *value0, u32 *value1)
{
	*value0 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_0);
	*value1 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_1);

	writel_relaxed(*value0, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(*value1, vfe->base + VFE_0_IRQ_CLEAR_1);

	/* Global clear */
	writel_relaxed(VFE_0_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_0_IRQ_CMD);
}

static irqreturn_t vfe31_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 value0, value1;
	int i, j;

	vfe->res->hw_ops->isr_read(vfe, &value0, &value1);

	/* Debug: log all interrupts */
	dev_info(vfe->camss->dev, "VFE IRQ status0: 0x%x, status1: 0x%x\n",
		value0, value1);

	/* VFE31 reset acknowledge is in STATUS_1 bit 22, not STATUS_0 bit 31 */
	if (value1 & VFE_0_IRQ_STATUS_1_RESET_ACK)
		vfe->isr_ops.reset_ack(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_VIOLATION)
		vfe->res->hw_ops->violation_read(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK)
		vfe->isr_ops.halt_ack(vfe);

	/*
	 * VFE31: REG_UPDATE applies to all lines since we only have one
	 * CAMIF. Notify all active lines.
	 */
	if (value0 & VFE_0_IRQ_STATUS_0_REG_UPDATE) {
		for (i = 0; i < vfe->res->line_num; i++)
			vfe->isr_ops.reg_update(vfe, i);
	}

	/*
	 * VFE31: CAMIF SOF needs to be delivered to all lines because
	 * we emulate RDI through CAMIF. Any line could be waiting for SOF.
	 */
	if (value0 & VFE_0_IRQ_STATUS_0_CAMIF_SOF) {
		for (i = 0; i < vfe->res->line_num; i++)
			vfe->isr_ops.sof(vfe, i);
	}

	for (i = 0; i < MSM_VFE_COMPOSITE_IRQ_NUM; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(i)) {
			vfe->isr_ops.comp_done(vfe, i);
			for (j = 0; j < ARRAY_SIZE(vfe->wm_output_map); j++)
				if (vfe->wm_output_map[j] == VFE_LINE_PIX)
					value0 &= ~VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(j);
		}

	for (i = 0; i < MSM_VFE_IMAGE_MASTERS_NUM; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(i))
			vfe->isr_ops.wm_done(vfe, i);

	return IRQ_HANDLED;
}

static int vfe31_halt(struct vfe_device *vfe)
{
	/* Use gen1 common halt implementation */
	return vfe_gen1_halt(vfe);
}

static int vfe31_enable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);

	dev_info(vfe->camss->dev, "VFE31 enable: line_id=%d ops_gen1=%px\n",
		 line->id, vfe->ops_gen1);
	/* Use gen1 common enable implementation */
	return vfe_gen1_enable(line);
}

static int vfe31_disable(struct vfe_line *line)
{
	/* Use gen1 common disable implementation */
	return vfe_gen1_disable(line);
}

/* Gen1-specific operations for VFE31 */
static void vfe31_enable_irq_common(struct vfe_device *vfe)
{
	/*
	 * Enable common IRQs. Note: VFE31 reset acknowledge is in
	 * STATUS_1 bit 22, not STATUS_0 bit 31 like later VFE versions.
	 */
	u32 val1 = VFE_0_IRQ_MASK_1_RESET_ACK |
		   VFE_0_IRQ_MASK_1_VIOLATION |
		   VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK;

	dev_info(vfe->camss->dev, "VFE31 enable_irq_common: mask1=0x%x\n", val1);

	writel_relaxed(0, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(val1, vfe->base + VFE_0_IRQ_MASK_1);
}

static void vfe31_set_demux_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 val, even_cfg, odd_cfg;

	writel_relaxed(VFE_0_MODULE_CFG_DEMUX,
		       vfe->base + VFE_0_MODULE_CFG);

	val = VFE_0_DEMUX_CFG_PERIOD;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_CFG);

	val = VFE_0_DEMUX_GAIN_0_CH0_EVEN | VFE_0_DEMUX_GAIN_0_CH0_ODD;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_0);

	val = VFE_0_DEMUX_GAIN_1_CH1 | VFE_0_DEMUX_GAIN_1_CH2;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_1);

	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		even_cfg = 0xc9;
		odd_cfg = 0xac;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		even_cfg = 0xa9;
		odd_cfg = 0xcc;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	default:
		even_cfg = 0x9c;
		odd_cfg = 0xca;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		even_cfg = 0xc9;
		odd_cfg = 0xac;
		break;
	}

	writel_relaxed(even_cfg << 4 | odd_cfg, vfe->base + VFE_0_DEMUX_CFG);
}

static void vfe31_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	/* VFE31 scale configuration */
	writel_relaxed(0, vfe->base + VFE_0_SCALE_ENC_Y_CFG);
	writel_relaxed(0, vfe->base + VFE_0_SCALE_ENC_CBCR_CFG);
}

static void vfe31_set_crop_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 width = line->fmt[MSM_VFE_PAD_SINK].width;
	u32 height = line->fmt[MSM_VFE_PAD_SINK].height;

	writel_relaxed((height << 16) | width,
		       vfe->base + VFE_0_CROP_ENC_Y_WIDTH);
	writel_relaxed(height, vfe->base + VFE_0_CROP_ENC_Y_HEIGHT);
	writel_relaxed((height << 16) | (width / 2),
		       vfe->base + VFE_0_CROP_ENC_CBCR_WIDTH);
	writel_relaxed(height, vfe->base + VFE_0_CROP_ENC_CBCR_HEIGHT);
}

static void vfe31_set_clamp_cfg(struct vfe_device *vfe)
{
	writel_relaxed(0x00ffffff, vfe->base + VFE_0_CLAMP_ENC_MAX_CFG);
	writel_relaxed(0x0, vfe->base + VFE_0_CLAMP_ENC_MIN_CFG);
}

static void vfe31_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable)
{
	/* VFE31 doesn't have per-WM CGC override like VFE41 */
	if (enable)
		writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CGC_OVERRIDE);
	else
		writel_relaxed(0x0, vfe->base + VFE_0_CGC_OVERRIDE);

	wmb();
}

static void vfe31_set_camif_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 val;

	dev_info(vfe->camss->dev, "VFE31 set_camif_cfg: ENTRY\n");

	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	default:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY;
		break;
	}

	writel_relaxed(val, vfe->base + VFE_0_CORE_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].width * 2;
	val |= line->fmt[MSM_VFE_PAD_SINK].height << 16;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_FRAME_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].width * 2 - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].height - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);

	val = 0xffffffff;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);

	val = 0xffffffff;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_IRQ_SUBSAMPLE_PATTERN);

	val = VFE_0_CAMIF_CFG_VFE_OUTPUT_EN;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_CFG);
}

static void vfe31_set_camif_cmd(struct vfe_device *vfe, u8 enable)
{
	u32 cmd;

	cmd = VFE_0_CAMIF_CMD_CLEAR_CAMIF_STATUS | VFE_0_CAMIF_CMD_NO_CHANGE;
	writel_relaxed(cmd, vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	if (enable)
		cmd = VFE_0_CAMIF_CMD_ENABLE_FRAME_BOUNDARY;
	else
		cmd = VFE_0_CAMIF_CMD_DISABLE_FRAME_BOUNDARY;

	writel_relaxed(cmd, vfe->base + VFE_0_CAMIF_CMD);

	/* Debug: dump CAMIF status and config registers */
	dev_info(vfe->camss->dev,
		 "VFE31 CAMIF: cmd=%s status=0x%08x cfg=0x%08x core_cfg=0x%08x frame_cfg=0x%08x\n",
		 enable ? "enable" : "disable",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS),
		 readl_relaxed(vfe->base + VFE_0_CAMIF_CFG),
		 readl_relaxed(vfe->base + VFE_0_CORE_CFG),
		 readl_relaxed(vfe->base + VFE_0_CAMIF_FRAME_CFG));
}

static void vfe31_set_module_cfg(struct vfe_device *vfe, u8 enable)
{
	u32 val = VFE_0_MODULE_CFG_DEMUX |
		  VFE_0_MODULE_CFG_CHROMA_UPSAMPLE |
		  VFE_0_MODULE_CFG_SCALE_ENC |
		  VFE_0_MODULE_CFG_CROP_ENC;

	dev_info(vfe->camss->dev, "VFE31 set_module_cfg: enable=%d val=0x%x\n",
		 enable, enable ? val : 0);

	if (enable)
		writel_relaxed(val, vfe->base + VFE_0_MODULE_CFG);
	else
		writel_relaxed(0x0, vfe->base + VFE_0_MODULE_CFG);
}

static int vfe31_camif_wait_for_stop(struct vfe_device *vfe, struct device *dev)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(vfe->base + VFE_0_CAMIF_STATUS,
				 val, (val & 0x1) == 0,
				 USEC_PER_MSEC, 100 * USEC_PER_MSEC);
	if (ret < 0)
		dev_err(dev, "CAMIF stop timeout\n");

	return ret;
}

static void vfe31_set_xbar_cfg(struct vfe_device *vfe, struct vfe_output *output,
			       u8 enable)
{
	/* VFE31 XBAR configuration */
	u32 val;

	if (output->wm_num == 1) {
		val = (output->wm_idx[0] << 8);
		writel_relaxed(val, vfe->base +
			       VFE_0_BUS_XBAR_CFG_x(output->wm_idx[0]));
	}
}

static void vfe31_set_realign_cfg(struct vfe_device *vfe, struct vfe_line *line,
				  u8 enable)
{
	/* VFE31 realign buffer configuration */
	writel_relaxed(0x0, vfe->base + VFE_0_REALIGN_BUF_CFG);
}

static void vfe31_set_rdi_cid(struct vfe_device *vfe, enum vfe_line_id id,
			      u8 cid)
{
	/*
	 * VFE31 uses CAMIF for all input, including CSI data.
	 * The CID (Channel ID) is handled at the CSI/CSID level, not VFE.
	 * The CAMIF receives data from whatever source CSI is configured for.
	 *
	 * For raw passthrough mode, ensure CAMIF is configured to pass
	 * data without modification. The actual channel selection happens
	 * in the CSI receiver hardware.
	 */
	dev_dbg(vfe->camss->dev, "VFE31: set RDI%d CID=%d (handled by CSID)\n",
		id, cid);
}

static void vfe31_set_qos(struct vfe_device *vfe)
{
	/* VFE31 QoS settings - can be left as default */
}

static void vfe31_set_ds(struct vfe_device *vfe)
{
	/* VFE31 doesn't have downscaler */
}

static u16 vfe31_get_ub_size(u8 vfe_id)
{
	/* VFE31 has 1024 bytes unified buffer */
	return MSM_VFE_VFE0_UB_SIZE;
}

static void vfe31_bus_connect_wm_to_rdi(struct vfe_device *vfe, u8 wm,
					enum vfe_line_id id)
{
	struct vfe_line *line = &vfe->line[id];
	u32 val;

	/*
	 * VFE31 doesn't have separate RDI paths like later VFEs.
	 * All data must go through CAMIF. For RDI-style output (frame-based
	 * raw passthrough), we configure:
	 * 1. CAMIF with frame dimensions (required - no separate RDI input!)
	 * 2. CAMIF to bus enable for raw passthrough
	 * 3. Raw write path selection
	 *
	 * This is the critical difference from VFE4x where RDI has separate
	 * hardware paths that don't need CAMIF.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: connect WM%d to RDI%d - configuring CAMIF for raw passthrough\n",
		 wm, id);

	/* Step 1: Configure CAMIF (normally only done for PIX path) */
	/* Set pixel pattern based on format */
	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	default:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY;
		break;
	}
	writel_relaxed(val, vfe->base + VFE_0_CORE_CFG);

	/* Set frame dimensions - width in bytes (YUV422 = 2 bytes/pixel) */
	val = line->fmt[MSM_VFE_PAD_SINK].width * 2;
	val |= line->fmt[MSM_VFE_PAD_SINK].height << 16;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_FRAME_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].width * 2 - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);

	val = line->fmt[MSM_VFE_PAD_SINK].height - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);

	writel_relaxed(0xffffffff, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);
	writel_relaxed(0xffffffff, vfe->base + VFE_0_CAMIF_IRQ_SUBSAMPLE_PATTERN);

	/* Enable VFE output in CAMIF */
	writel_relaxed(VFE_0_CAMIF_CFG_VFE_OUTPUT_EN,
		       vfe->base + VFE_0_CAMIF_CFG);

	/* Step 2: Enable CAMIF to bus (raw passthrough) in VFE_CFG */
	val = readl_relaxed(vfe->base + VFE_0_VFE_CFG);
	val |= VFE_0_VFE_CFG_CAMIF_TO_BUS_EN;
	writel_relaxed(val, vfe->base + VFE_0_VFE_CFG);

	/* Step 3: Configure BUS_CFG for raw passthrough via encoder CbCr path */
	val = readl_relaxed(vfe->base + VFE_0_BUS_CFG);
	val &= ~(0x3 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT);
	val |= (VFE_0_BUS_CFG_RAW_WR_PATH_ENC_CBCR << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT);
	val |= VFE_0_BUS_CFG_ENC_CBCR_WR_PATH_EN;
	writel_relaxed(val, vfe->base + VFE_0_BUS_CFG);

	wmb();

	/*
	 * MSM8660 workaround: Do NOT enable CAMIF here.
	 *
	 * If we enable CAMIF (CAMIF_CMD_ENABLE_FRAME_BOUNDARY) before CSIPHY
	 * is configured, the CSI register access path becomes blocked. This
	 * causes CSIPHY lanes_enable to hang when writing to CSIPHY registers.
	 *
	 * The workaround is to defer CAMIF enable until after CSIPHY
	 * lanes_enable completes. Set camif_pending flag and let CSIPHY
	 * call vfe_enable_pending_camif() after configuring its lanes.
	 */
	vfe->camif_pending = true;

	dev_info(vfe->camss->dev,
		 "VFE31: CAMIF configured (DEFERRED) - status=0x%08x cfg=0x%08x frame=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS),
		 readl_relaxed(vfe->base + VFE_0_CAMIF_CFG),
		 readl_relaxed(vfe->base + VFE_0_CAMIF_FRAME_CFG));
}

static void vfe31_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
					     enum vfe_line_id id)
{
	u32 val;

	dev_info(vfe->camss->dev, "VFE31: disconnect WM%d from RDI%d\n", wm, id);

	/* Step 1: Disable CAMIF - stop frame boundary capture */
	writel_relaxed(VFE_0_CAMIF_CMD_DISABLE_FRAME_BOUNDARY,
		       vfe->base + VFE_0_CAMIF_CMD);

	/* Step 2: Disable raw passthrough in BUS_CFG */
	val = readl_relaxed(vfe->base + VFE_0_BUS_CFG);
	val &= ~(0x3 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT);
	val |= (VFE_0_BUS_CFG_RAW_WR_PATH_DISABLED << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT);
	writel_relaxed(val, vfe->base + VFE_0_BUS_CFG);

	/* Step 3: Disable CAMIF to bus */
	val = readl_relaxed(vfe->base + VFE_0_VFE_CFG);
	val &= ~VFE_0_VFE_CFG_CAMIF_TO_BUS_EN;
	writel_relaxed(val, vfe->base + VFE_0_VFE_CFG);

	/* Step 4: Disable CAMIF output */
	writel_relaxed(0, vfe->base + VFE_0_CAMIF_CFG);

	vfe->camif_pending = false;

	wmb();
}

static void vfe31_wm_set_subsample(struct vfe_device *vfe, u8 wm)
{
	/* VFE31 WM subsample configuration */
	writel_relaxed(0x0,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(wm));
}

static void vfe31_bus_enable_wr_if(struct vfe_device *vfe, u8 enable)
{
	/*
	 * VFE31 bus write interface enable.
	 * BUS_CFG register controls which write paths are enabled:
	 * - Bit 4: encYWrPathEn
	 * - Bit 5: encCbcrWrPathEn
	 * - Bit 6: viewYWrPathEn
	 * - Bit 7: viewCbcrWrPathEn
	 *
	 * For initial enable, we set a base configuration.
	 * The specific paths are enabled/configured by connect_wm_to_rdi
	 * or set_camif_cfg depending on PIX vs RDI mode.
	 */
	if (enable) {
		/* Enable all write paths initially - specific paths configured later */
		writel_relaxed(VFE_0_BUS_CFG_ENC_Y_WR_PATH_EN |
			       VFE_0_BUS_CFG_ENC_CBCR_WR_PATH_EN |
			       VFE_0_BUS_CFG_VIEW_Y_WR_PATH_EN |
			       VFE_0_BUS_CFG_VIEW_CBCR_WR_PATH_EN,
			       vfe->base + VFE_0_BUS_CFG);
		dev_dbg(vfe->camss->dev, "VFE31: bus write interface enabled\n");
	} else {
		writel_relaxed(0x0, vfe->base + VFE_0_BUS_CFG);
	}
}

static void vfe31_bus_reload_wm(struct vfe_device *vfe, u8 wm)
{
	wmb();
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm),
		       vfe->base + VFE_0_BUS_CMD);
	wmb();
}

static void vfe31_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable)
{
	u32 val;

	/*
	 * VFE31 WM_WR_CFG register configuration for frame-based mode:
	 * Bit 0: enable - master enable
	 * Bit 1: frame_based - 1 for frame-based, 0 for line-based
	 * Other bits control burst length, etc.
	 *
	 * For raw passthrough (RDI emulation), use frame-based mode.
	 */
	if (enable) {
		/* Frame-based mode: bit 1 set, burst length default */
		val = 0x2 | BIT(0);  /* frame_based | enable */
		dev_dbg(vfe->camss->dev, "VFE31: WM%d frame-based enable\n", wm);
	} else {
		val = 0;
	}

	writel_relaxed(val, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
}

static void vfe31_wm_line_based(struct vfe_device *vfe, u32 wm,
				struct v4l2_pix_format_mplane *pix,
				u8 plane, u32 enable)
{
	/* VFE31 WM line-based mode */
	u32 val = 0x0;

	if (enable) {
		val = pix->plane_fmt[0].bytesperline *
			pix->height / 4;
	}

	writel_relaxed(val,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
}

static void vfe31_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable)
{
	/*
	 * VFE31 WM enable - bit 0 of WR_CFG enables the write master.
	 * Use read-modify-write to preserve other configuration bits.
	 */
	u32 val = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));

	if (enable)
		val |= BIT(0);
	else
		val &= ~BIT(0);

	writel_relaxed(val, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
}

static void vfe31_wm_set_ub_cfg(struct vfe_device *vfe, u8 wm,
				u16 offset, u16 depth)
{
	u32 val;

	val = (offset << 16) | depth;
	writel_relaxed(val,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
}

static void vfe31_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
}

static void vfe31_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
}

static int vfe31_wm_get_ping_pong_status(struct vfe_device *vfe, u8 wm)
{
	u32 val = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (val >> wm) & 0x1;
}

static void vfe31_wm_set_framedrop_period(struct vfe_device *vfe, u8 wm,
					  u8 per)
{
	writel_relaxed(per,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PERIOD(wm));
}

static void vfe31_wm_set_framedrop_pattern(struct vfe_device *vfe, u8 wm,
					   u32 pattern)
{
	writel_relaxed(pattern,
		       vfe->base +
		       VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(wm));
}

static void vfe31_enable_irq_pix_line(struct vfe_device *vfe, u8 comp,
				      enum vfe_line_id line_id, u8 enable)
{
	u32 val0 = VFE_0_IRQ_MASK_0_CAMIF_SOF |
		   VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(comp) |
		   VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(VFE_LINE_PIX);
	u32 val1 = VFE_0_IRQ_MASK_1_CAMIF_ERROR;

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, val0);
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, val1);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, val0);
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, val1);
	}

	dev_info(vfe->camss->dev,
		 "VFE31 IRQ pix_line: enable=%d mask0=0x%08x mask1=0x%08x\n",
		 enable,
		 readl_relaxed(vfe->base + VFE_0_IRQ_MASK_0),
		 readl_relaxed(vfe->base + VFE_0_IRQ_MASK_1));
}

static void vfe31_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
				     enum vfe_line_id line_id, u8 enable)
{
	/*
	 * VFE31: For RDI-style operation, we still need SOF interrupt
	 * because the gen1 disable code waits for SOF completion.
	 * Also enable REG_UPDATE since it's used for buffer management.
	 */
	u32 val0 = VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm) |
		   VFE_0_IRQ_MASK_0_CAMIF_SOF |
		   VFE_0_IRQ_MASK_0_REG_UPDATE;
	u32 val1 = VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(wm) |
		   VFE_0_IRQ_MASK_1_CAMIF_ERROR;

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, val0);
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, val1);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, val0);
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, val1);
	}

	dev_info(vfe->camss->dev,
		 "VFE31 IRQ wm_line: wm=%d line=%d enable=%d mask0=0x%08x mask1=0x%08x\n",
		 wm, line_id, enable,
		 readl_relaxed(vfe->base + VFE_0_IRQ_MASK_0),
		 readl_relaxed(vfe->base + VFE_0_IRQ_MASK_1));
}

static void vfe31_pm_domain_off(struct vfe_device *vfe)
{
	if (!vfe->res->has_pd)
		return;

	vfe_pm_domain_off(vfe);
}

static int vfe31_pm_domain_on(struct vfe_device *vfe)
{
	if (!vfe->res->has_pd)
		return 0;

	return vfe_pm_domain_on(vfe);
}

/* Gen1 operations structure for VFE31 */
static const struct vfe_hw_ops_gen1 vfe_ops_gen1_3_1 = {
	.bus_connect_wm_to_rdi = vfe31_bus_connect_wm_to_rdi,
	.bus_disconnect_wm_from_rdi = vfe31_bus_disconnect_wm_from_rdi,
	.bus_enable_wr_if = vfe31_bus_enable_wr_if,
	.bus_reload_wm = vfe31_bus_reload_wm,
	.camif_wait_for_stop = vfe31_camif_wait_for_stop,
	.enable_irq_common = vfe31_enable_irq_common,
	.enable_irq_pix_line = vfe31_enable_irq_pix_line,
	.enable_irq_wm_line = vfe31_enable_irq_wm_line,
	.get_ub_size = vfe31_get_ub_size,
	.halt_clear = vfe31_halt_clear,
	.halt_request = vfe31_halt_request,
	.set_camif_cfg = vfe31_set_camif_cfg,
	.set_camif_cmd = vfe31_set_camif_cmd,
	.set_cgc_override = vfe31_set_cgc_override,
	.set_clamp_cfg = vfe31_set_clamp_cfg,
	.set_crop_cfg = vfe31_set_crop_cfg,
	.set_demux_cfg = vfe31_set_demux_cfg,
	.set_ds = vfe31_set_ds,
	.set_module_cfg = vfe31_set_module_cfg,
	.set_qos = vfe31_set_qos,
	.set_rdi_cid = vfe31_set_rdi_cid,
	.set_realign_cfg = vfe31_set_realign_cfg,
	.set_scale_cfg = vfe31_set_scale_cfg,
	.set_xbar_cfg = vfe31_set_xbar_cfg,
	.wm_enable = vfe31_wm_enable,
	.wm_frame_based = vfe31_wm_frame_based,
	.wm_get_ping_pong_status = vfe31_wm_get_ping_pong_status,
	.wm_line_based = vfe31_wm_line_based,
	.wm_set_framedrop_pattern = vfe31_wm_set_framedrop_pattern,
	.wm_set_framedrop_period = vfe31_wm_set_framedrop_period,
	.wm_set_ping_addr = vfe31_wm_set_ping_addr,
	.wm_set_pong_addr = vfe31_wm_set_pong_addr,
	.wm_set_subsample = vfe31_wm_set_subsample,
	.wm_set_ub_cfg = vfe31_wm_set_ub_cfg,
};

static void vfe31_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	dev_info(dev, "VFE31 subdev_init: setting up ops_gen1\n");
	vfe->isr_ops = vfe_isr_ops_gen1;
	vfe->ops_gen1 = &vfe_ops_gen1_3_1;
	vfe->video_ops = vfe_video_ops_gen1;
	dev_info(dev, "VFE31 subdev_init: ops_gen1=%px\n", vfe->ops_gen1);
}

const struct vfe_hw_ops vfe_ops_3_1 = {
	.global_reset = vfe31_global_reset,
	.hw_version = vfe31_hw_version,
	.isr_read = vfe31_isr_read,
	.isr = vfe31_isr,
	.pm_domain_off = vfe31_pm_domain_off,
	.pm_domain_on = vfe31_pm_domain_on,
	.reg_update_clear = vfe31_reg_update_clear,
	.reg_update = vfe31_reg_update,
	.subdev_init = vfe31_subdev_init,
	.vfe_disable = vfe31_disable,
	.vfe_enable = vfe31_enable,
	.vfe_halt = vfe31_halt,
	.violation_read = vfe31_violation_read,
};
