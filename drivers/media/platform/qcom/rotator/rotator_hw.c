// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 Rotator driver - Hardware abstraction
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_rotator driver from webOS kernel.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include "rotator_hw.h"

int rotator_hw_reset(void __iomem *base)
{
	/* Clear any pending interrupts */
	writel(ROTATOR_IRQ_ALL, base + ROTATOR_INTR_CLEAR);

	/* Set maximum burst size for optimal performance */
	writel(16, base + ROTATOR_MAX_BURST_SIZE);

	return 0;
}

void rotator_hw_enable_irq(void __iomem *base)
{
	writel(ROTATOR_IRQ_DONE, base + ROTATOR_INTR_ENABLE);
}

void rotator_hw_disable_irq(void __iomem *base)
{
	writel(0, base + ROTATOR_INTR_ENABLE);
}

void rotator_hw_clear_irq(void __iomem *base)
{
	writel(ROTATOR_IRQ_ALL, base + ROTATOR_INTR_CLEAR);
}

u32 rotator_hw_get_irq_status(void __iomem *base)
{
	return readl(base + ROTATOR_INTR_STATUS);
}

u32 rotator_hw_get_version(void __iomem *base)
{
	return readl(base + ROTATOR_HW_VERSION);
}

void rotator_hw_start(void __iomem *base)
{
	/* Trigger rotation */
	writel(1, base + ROTATOR_START);
}

void rotator_hw_set_src_size(void __iomem *base, u32 width, u32 height)
{
	/* SRC_SIZE: [28:16] = height-1, [12:0] = width-1 */
	writel(((height - 1) << 16) | (width - 1), base + ROTATOR_SRC_SIZE);

	/* SRC_IMAGE_SIZE: full image dimensions */
	writel((height << 16) | width, base + ROTATOR_SRC_IMAGE_SIZE);

	/* SRC_XY: crop offset (0,0 for full image) */
	writel(0, base + ROTATOR_SRC_XY);
}

void rotator_hw_set_src_addr(void __iomem *base, dma_addr_t y_addr,
			     dma_addr_t c_addr)
{
	writel(y_addr, base + ROTATOR_SRCP0_ADDR);
	writel(c_addr, base + ROTATOR_SRCP1_ADDR);
}

void rotator_hw_set_dst_addr(void __iomem *base, dma_addr_t y_addr,
			     dma_addr_t c_addr)
{
	writel(y_addr, base + ROTATOR_OUTP0_ADDR);
	writel(c_addr, base + ROTATOR_OUTP1_ADDR);
}

void rotator_hw_set_strides(void __iomem *base, u32 src_stride, u32 dst_stride)
{
	/* Y and C strides packed together */
	writel(src_stride | (src_stride << 16), base + ROTATOR_SRC_YSTRIDE1);
	writel(dst_stride | (dst_stride << 16), base + ROTATOR_OUT_YSTRIDE1);
}

void rotator_hw_set_rotation(void __iomem *base, u32 rotation, u32 chroma)
{
	u32 rot_bits = 0;
	u32 cfg;

	/* Convert rotation flags to hardware bitmask */
	if (rotation & ROTATOR_ROT_90)
		rot_bits |= 1;
	if (rotation & ROTATOR_FLIP_LR)
		rot_bits |= 2;
	if (rotation & ROTATOR_FLIP_UD)
		rot_bits |= 4;

	cfg = (chroma << ROTATOR_SUB_BLOCK_CHROMA_SHIFT) |
	      (rot_bits << ROTATOR_SUB_BLOCK_ROT_SHIFT) |
	      ROTATOR_SUB_BLOCK_ROT_EN;

	writel(cfg, base + ROTATOR_SUB_BLOCK_CFG);
}

void rotator_hw_set_format_rgb(void __iomem *base, u32 bpp, bool has_alpha)
{
	u32 fmt;
	u32 pack;

	/* RGB pack pattern: ARGB order */
	if (has_alpha)
		pack = ROTATOR_PACK_PATTERN(CLR_ALPHA, CLR_R, CLR_G, CLR_B, 8);
	else
		pack = ROTATOR_PACK_PATTERN(CLR_R, CLR_R, CLR_G, CLR_B, 8);

	writel(pack, base + ROTATOR_SRC_UNPACK_PATTERN1);
	writel(pack, base + ROTATOR_OUT_PACK_PATTERN1);

	/* Source format configuration */
	fmt = ROTATOR_SRC_FORMAT_FRAME_LINEAR |
	      ROTATOR_SRC_FORMAT_TILE_SIZE |
	      (ROTATOR_FETCH_PLANES_INTERLEAVED << ROTATOR_SRC_FORMAT_FETCH_SHIFT) |
	      ROTATOR_SRC_FORMAT_UNPACK_TIGHT |
	      ((bpp - 1) << ROTATOR_SRC_FORMAT_UNPACK_SHIFT) |
	      ((bpp - 1) << ROTATOR_SRC_FORMAT_BPP_SHIFT);

	if (has_alpha) {
		fmt |= ROTATOR_SRC_FORMAT_ALPHA |
		       (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_ALPHA_SHIFT);
	}

	if (bpp == 2) {
		/* RGB565: 5-6-5 bits */
		fmt |= (ROTATOR_COMP_5BITS << ROTATOR_SRC_FORMAT_R_SHIFT) |
		       (ROTATOR_COMP_6BITS << ROTATOR_SRC_FORMAT_G_SHIFT) |
		       (ROTATOR_COMP_5BITS << ROTATOR_SRC_FORMAT_B_SHIFT);
	} else {
		/* RGB888/ARGB8888: 8-8-8 bits */
		fmt |= (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_R_SHIFT) |
		       (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_G_SHIFT) |
		       (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_B_SHIFT);
	}

	writel(fmt, base + ROTATOR_SRC_FORMAT);
}

void rotator_hw_set_format_yuv(void __iomem *base, u32 chroma_mode, bool cbcr)
{
	u32 fmt;
	u32 pack;

	/* YUV pack pattern: CbCr or CrCb order */
	if (cbcr)
		pack = ROTATOR_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8);
	else
		pack = ROTATOR_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8);

	writel(pack, base + ROTATOR_SRC_UNPACK_PATTERN1);
	writel(pack, base + ROTATOR_OUT_PACK_PATTERN1);

	/* Source format for YUV */
	fmt = ROTATOR_SRC_FORMAT_FRAME_LINEAR |
	      ROTATOR_SRC_FORMAT_TILE_SIZE |
	      (ROTATOR_FETCH_PLANES_PSEUDO << ROTATOR_SRC_FORMAT_FETCH_SHIFT) |
	      ROTATOR_SRC_FORMAT_UNPACK_TIGHT |
	      (1 << ROTATOR_SRC_FORMAT_UNPACK_SHIFT) |	/* 2 components */
	      (0 << ROTATOR_SRC_FORMAT_BPP_SHIFT) |	/* 1 byte per component */
	      (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_R_SHIFT) |
	      (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_G_SHIFT) |
	      (ROTATOR_COMP_8BITS << ROTATOR_SRC_FORMAT_B_SHIFT);

	writel(fmt, base + ROTATOR_SRC_FORMAT);
}
