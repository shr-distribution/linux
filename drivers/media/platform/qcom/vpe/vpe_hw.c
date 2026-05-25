// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 Video Processing Engine (VPE) - Hardware abstraction
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_vpe1 driver from webOS kernel.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include "vpe_hw.h"

/* Scale coefficient tables for different scale ratios */
static const u32 vpe_scale_coeff_table_0[] = {
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
	0x5fffc, 0x20000, 0x5fffc, 0x20000,
};

static const u32 vpe_scale_coeff_table_1[] = {
	0x5fffc, 0x20000, 0x5ffc0, 0x20040,
	0x5ff84, 0x20080, 0x5ff48, 0x200c0,
	0x5ff0c, 0x20100, 0x5fec8, 0x20148,
	0x5fe8c, 0x20188, 0x5fe48, 0x201d0,
	0x5fe08, 0x20210, 0x5fdc0, 0x20258,
	0x5fd7c, 0x20298, 0x5fd34, 0x202e0,
	0x5fcf0, 0x20320, 0x5fca8, 0x20368,
	0x5fc60, 0x203a8, 0x5fc18, 0x203f0,
};

static const u32 vpe_scale_coeff_table_2[] = {
	0x5fffc, 0x20000, 0x5ffc8, 0x20038,
	0x5ff94, 0x20070, 0x5ff60, 0x200a8,
	0x5ff2c, 0x200e0, 0x5fef8, 0x20118,
	0x5fec0, 0x20154, 0x5fe8c, 0x2018c,
	0x5fe54, 0x201c8, 0x5fe20, 0x20200,
	0x5fde8, 0x2023c, 0x5fdb0, 0x20274,
	0x5fd78, 0x202b0, 0x5fd40, 0x202e8,
	0x5fd04, 0x20324, 0x5fccc, 0x2035c,
};

static const u32 vpe_scale_coeff_table_3[] = {
	0x5fffc, 0x20000, 0x5ffd0, 0x20030,
	0x5ffa4, 0x20060, 0x5ff78, 0x20090,
	0x5ff4c, 0x200c0, 0x5ff1c, 0x200f4,
	0x5fef0, 0x20124, 0x5fec0, 0x20158,
	0x5fe94, 0x20188, 0x5fe64, 0x201bc,
	0x5fe38, 0x201ec, 0x5fe08, 0x20220,
	0x5fddc, 0x20250, 0x5fdac, 0x20284,
	0x5fd7c, 0x202b4, 0x5fd4c, 0x202e8,
};

int vpe_hw_reset(void __iomem *base)
{
	int timeout = 100;

	/* Issue software reset */
	writel(VPE_SW_RESET_VALUE, base + VPE_SW_RESET);

	/* Wait for reset to complete */
	while (timeout > 0) {
		if (!(readl(base + VPE_SW_RESET) & VPE_SW_RESET_VALUE))
			break;
		udelay(10);
		timeout--;
	}

	if (timeout == 0)
		return -ETIMEDOUT;

	/* Enable all clock gates */
	writel(VPE_CGC_ENABLE_VALUE, base + VPE_CGC_EN);

	/* Configure AXI read arbitration */
	writel(VPE_AXI_RD_ARB_CONFIG_VALUE, base + VPE_AXI_RD_ARB_CONFIG);

	/*
	 * Select command (display-list) mode ONCE here, before any frame
	 * registers are programmed -- this matches legacy msm_vpe1, which sets
	 * CMD_MODE at init and never per-frame. Setting it per-kick after the
	 * frame programming (as we did) left the engine consuming the kick but
	 * producing no output and never raising DONE.
	 */
	writel(VPE_CMD_MODE_VALUE, base + VPE_CMD_MODE);

	/* Set default operation mode */
	writel(VPE_DEFAULT_OP_MODE_VALUE, base + VPE_OP_MODE);

	/* Set default scale config */
	writel(VPE_DEFAULT_SCALE_CONFIG, base + VPE_SCALE_CONFIG);

	return 0;
}

void vpe_hw_enable_irq(void __iomem *base)
{
	writel(VPE_INTR_STATUS_DONE, base + VPE_INTR_ENABLE);
}

void vpe_hw_disable_irq(void __iomem *base)
{
	writel(0, base + VPE_INTR_ENABLE);
}

void vpe_hw_clear_irq(void __iomem *base)
{
	writel(VPE_INTR_STATUS_DONE, base + VPE_INTR_CLEAR);
}

u32 vpe_hw_get_irq_status(void __iomem *base)
{
	return readl(base + VPE_INTR_STATUS);
}

u32 vpe_hw_get_version(void __iomem *base)
{
	return readl(base + VPE_HW_VERSION);
}

void vpe_hw_start(void __iomem *base)
{
	/*
	 * CMD_MODE is established once in vpe_hw_reset (matching legacy
	 * msm_vpe1); here we only trigger the operation, as legacy vpe_start
	 * does (INTR_ENABLE is set by vpe_hw_enable_irq just before this).
	 */
	writel(1, base + VPE_DL0_START);
}

void vpe_hw_set_src_addr(void __iomem *base, dma_addr_t y_addr, dma_addr_t cbcr_addr)
{
	writel(y_addr, base + VPE_SRCP0_ADDR);
	writel(cbcr_addr, base + VPE_SRCP1_ADDR);
}

void vpe_hw_set_dst_addr(void __iomem *base, dma_addr_t y_addr, dma_addr_t cbcr_addr)
{
	writel(y_addr, base + VPE_OUTP0_ADDR);
	writel(cbcr_addr, base + VPE_OUTP1_ADDR);
}

void vpe_hw_set_src_size(void __iomem *base, u32 img_w, u32 img_h,
			 u32 crop_w, u32 crop_h, u32 stride)
{
	/* SRC_SIZE is the ROI (crop) being read; IMAGE_SIZE is the full frame */
	writel(((crop_h & 0xfff) << 16) | (crop_w & 0xfff), base + VPE_SRC_SIZE);
	writel(((img_h & 0xfff) << 16) | (img_w & 0xfff),
	       base + VPE_SRC_IMAGE_SIZE);
	writel(stride, base + VPE_SRC_YSTRIDE1);

	/*
	 * Source format + unpack pattern for NV12 (pseudo-planar Y + CbCr).
	 * These are the exact words the msm camera HAL programs for the VPE
	 * input plane (confirmed identical across the Sony nozomi, HTC and
	 * Samsung msm8660 liboemcamera.so / mm_vpe_set_input_plane). Our prior
	 * placeholder 0x2 / 0x03020100 left the engine unable to interpret the
	 * surface, so it consumed the kick but produced no output.
	 */
	writel(VPE_SRC_FORMAT_NV12, base + VPE_SRC_FORMAT);
	writel(VPE_PACK_PATTERN_NV12, base + VPE_SRC_UNPACK_PATTERN1);
}

void vpe_hw_set_dst_size(void __iomem *base, u32 width, u32 height, u32 stride)
{
	u32 size = ((height & 0xfff) << 16) | (width & 0xfff);

	writel(size, base + VPE_OUT_SIZE);
	writel(stride, base + VPE_OUT_YSTRIDE1);

	/* Output format + pack pattern for NV12, from the camera HAL's VPE
	 * output-plane config (mm_vpe_set_output_plane). Note the format word
	 * differs from the source one (byte2 = 0x09 vs 0x12). */
	writel(VPE_OUT_FORMAT_NV12, base + VPE_OUT_FORMAT);
	writel(VPE_PACK_PATTERN_NV12, base + VPE_OUT_PACK_PATTERN1);
}

static void vpe_hw_load_scale_coeff(void __iomem *base, u32 phase_step)
{
	const u32 *coeff;
	int i;

	/* Select coefficient table based on scale ratio */
	if (phase_step >= HAL_MDP_PHASE_STEP_2P50)
		coeff = vpe_scale_coeff_table_0;
	else if (phase_step >= HAL_MDP_PHASE_STEP_1P66)
		coeff = vpe_scale_coeff_table_1;
	else if (phase_step >= HAL_MDP_PHASE_STEP_1P25)
		coeff = vpe_scale_coeff_table_2;
	else
		coeff = vpe_scale_coeff_table_3;

	/* Load coefficients */
	for (i = 0; i < VPE_SCALE_COEFF_NUM; i++) {
		writel(coeff[i * 2], base + VPE_SCALE_COEFF_LSBn(i));
		writel(coeff[i * 2 + 1], base + VPE_SCALE_COEFF_MSBn(i));
	}
}

void vpe_hw_set_scale(void __iomem *base, u32 src_w, u32 src_h, u32 dst_w, u32 dst_h)
{
	u64 phase_step_x, phase_step_y;
	u32 step_x, step_y;
	u32 op_mode;

	/* Calculate phase step values (fixed point 3.29 format) */
	phase_step_x = ((u64)src_w << SCALER_PHASE_BITS) + (dst_w - 1);
	do_div(phase_step_x, dst_w);
	step_x = (u32)phase_step_x;

	phase_step_y = ((u64)src_h << SCALER_PHASE_BITS) + (dst_h - 1);
	do_div(phase_step_y, dst_h);
	step_y = (u32)phase_step_y;

	/*
	 * Enable/disable the scaler block in OP_MODE[1:0]. The phase steps
	 * and coefficients below are inert unless the scaler is enabled here:
	 * the legacy msm_vpe1 driver sets OP_MODE |= 0x3 when zooming and
	 * clears it for a 1:1 passthrough. Without this the VPE ignores the
	 * requested scale entirely.
	 */
	op_mode = readl(base + VPE_OP_MODE) & ~VPE_OP_MODE_SCALE_MASK;
	if (src_w != dst_w || src_h != dst_h)
		op_mode |= VPE_OP_MODE_SCALE_EN;
	writel(op_mode, base + VPE_OP_MODE);

	/* Load scale coefficients based on maximum phase step */
	vpe_hw_load_scale_coeff(base, max(step_x, step_y));

	/* Set phase init to 0 */
	writel(0, base + VPE_SCALE_PHASEX_INIT);
	writel(0, base + VPE_SCALE_PHASEY_INIT);

	/* Set phase step values */
	writel(step_x, base + VPE_SCALE_PHASEX_STEP);
	writel(step_y, base + VPE_SCALE_PHASEY_STEP);
}

void vpe_hw_set_roi(void __iomem *base, u32 src_x, u32 src_y,
		    u32 dst_x, u32 dst_y)
{
	/* Source crop offset and output placement offset */
	writel(((src_y & 0xfff) << 16) | (src_x & 0xfff), base + VPE_SRC_XY);
	writel(((dst_y & 0xfff) << 16) | (dst_x & 0xfff), base + VPE_OUT_XY);
}

void vpe_hw_set_rotation(void __iomem *base, int rotation)
{
	u32 op_mode = readl(base + VPE_OP_MODE);

	/* Clear rotation bits */
	op_mode &= ~VPE_OP_MODE_ROTATION_MASK;

	/* Set new rotation */
	switch (rotation) {
	case 90:
		op_mode |= VPE_OP_MODE_ROT_90;
		break;
	case 180:
		op_mode |= VPE_OP_MODE_ROT_180;
		break;
	case 270:
		op_mode |= VPE_OP_MODE_ROT_270;
		break;
	default:
		op_mode |= VPE_OP_MODE_ROT_0;
		break;
	}

	writel(op_mode, base + VPE_OP_MODE);
}

void vpe_hw_set_op_mode(void __iomem *base, u32 mode)
{
	writel(mode, base + VPE_OP_MODE);
}
