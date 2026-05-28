// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 Rotator driver - Hardware abstraction
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * Based on legacy msm_rotator driver from webOS kernel.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include "rotator_hw.h"

int rotator_hw_reset(void __iomem *base)
{
	/*
	 * Clear any pending interrupts at the start of a job. MAX_BURST_SIZE is
	 * deliberately left at its hardware default: the legacy msm_rotator only
	 * overrides it (to 0x42) on the IMEM tile-fetch path, and explicitly
	 * documents that a burst beat size of 16 hangs the rotator. We do not
	 * use IMEM, so the default burst is correct.
	 */
	writel(ROTATOR_IRQ_ALL, base + ROTATOR_INTR_CLEAR);

	return 0;
}

void rotator_hw_sw_reset(void __iomem *base)
{
	/*
	 * Soft-reset to clear a latched AXI bus error, which otherwise wedges
	 * the block so that every subsequent job times out. Matches the legacy
	 * msm_rotator AXI-error recovery: pulse SW_RESET, then mask and clear
	 * the interrupt. No poll/delay is required; the next job reconfigures
	 * the block from scratch.
	 */
	writel(1, base + ROTATOR_SW_RESET);
	writel(0, base + ROTATOR_INTR_ENABLE);
	writel(ROTATOR_IRQ_ALL, base + ROTATOR_INTR_CLEAR);
}

void rotator_hw_enable_irq(void __iomem *base)
{
	/* Enable both DONE and ERROR, as legacy msm_rotator does: an error
	 * must raise an interrupt too, otherwise it degrades to a timeout.
	 */
	writel(ROTATOR_IRQ_ALL, base + ROTATOR_INTR_ENABLE);
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

void rotator_hw_start(void __iomem *base)
{
	/* Trigger rotation */
	writel(1, base + ROTATOR_START);
}

void rotator_hw_set_src_size(void __iomem *base, u32 img_w, u32 img_h,
			     u32 crop_x, u32 crop_y, u32 crop_w, u32 crop_h)
{
	/* SRC_SIZE: [28:16] = ROI height, [12:0] = ROI width (no -1; matches
	 * legacy msm_rotator which programs the raw crop dimensions).
	 */
	writel(((crop_h & 0x1fff) << 16) | (crop_w & 0x1fff),
	       base + ROTATOR_SRC_SIZE);

	/* SRC_XY: ROI offset within the full image */
	writel(((crop_y & 0x1fff) << 16) | (crop_x & 0x1fff),
	       base + ROTATOR_SRC_XY);

	/* SRC_IMAGE_SIZE: full image dimensions */
	writel(((img_h & 0x1fff) << 16) | (img_w & 0x1fff),
	       base + ROTATOR_SRC_IMAGE_SIZE);
}

u32 rotator_tiled_chroma_offset(u32 width, u32 height)
{
	/* Two 64-wide tiles per macro-tile row, one 32-high tile per column */
	const u32 tile_w = 64 * 2;	/* 128 */
	const u32 tile_h = 32 * 1;	/* 32 */
	u32 row_num_w = (width + tile_w - 1) / tile_w;
	u32 row_num_h = (height + tile_h - 1) / tile_h;

	return ((row_num_w * row_num_h * tile_w * tile_h) + 8191) & ~8191u;
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

void rotator_hw_set_strides(void __iomem *base, u32 src_stride, u32 dst_stride,
			    u32 dst_cstride)
{
	/*
	 * YSTRIDE1 packs the luma stride [15:0] and the chroma stride [31:16].
	 * The chroma stride usually equals the luma stride, but rotating a 4:2:2
	 * (H2V1) source 90/270 transposes its horizontally-subsampled chroma into
	 * a vertically-subsampled (H1V2) plane whose rows are twice as wide — the
	 * caller passes the correct dst chroma stride for that case.
	 */
	writel(src_stride | (src_stride << 16), base + ROTATOR_SRC_YSTRIDE1);
	writel(dst_stride | (dst_cstride << 16), base + ROTATOR_OUT_YSTRIDE1);
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

	/*
	 * RGB pack pattern: ARGB order. The top (4th) component slot is the
	 * alpha channel for 32-bit formats and unused (0) otherwise — matching
	 * legacy msm_rotator, which uses GET_PACK_PATTERN(0, R, G, B) with no
	 * alpha rather than duplicating a colour component.
	 */
	if (has_alpha)
		pack = ROTATOR_PACK_PATTERN(CLR_ALPHA, CLR_R, CLR_G, CLR_B, 8);
	else
		pack = ROTATOR_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 8);

	writel(pack, base + ROTATOR_SRC_UNPACK_PATTERN1);
	writel(pack, base + ROTATOR_OUT_PACK_PATTERN1);

	/*
	 * Source format configuration. The unpack count is the number of
	 * components minus one: 3 (4 comps) with alpha, 2 (3 comps) without —
	 * it is fixed by the component layout, not derived from the byte depth
	 * (legacy uses (abits ? 3 : 2), so e.g. RGB565 must still be 2, not 1).
	 */
	fmt = ROTATOR_SRC_FORMAT_FRAME_LINEAR |
	      ROTATOR_SRC_FORMAT_TILE_SIZE |
	      (ROTATOR_FETCH_PLANES_INTERLEAVED << ROTATOR_SRC_FORMAT_FETCH_SHIFT) |
	      ROTATOR_SRC_FORMAT_UNPACK_TIGHT |
	      ((has_alpha ? 3 : 2) << ROTATOR_SRC_FORMAT_UNPACK_SHIFT) |
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

void rotator_hw_set_format_yuv(void __iomem *base, u32 chroma_mode, bool cbcr,
			       bool tiled)
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

	/*
	 * Source frame format: supertile for Samsung 64x32 tiled NV12 (the
	 * layout the VIDC decoder emits), linear otherwise. The output is
	 * always written back linear.
	 */
	fmt = (tiled ? ROTATOR_SRC_FORMAT_FRAME_SUPERTILE
		     : ROTATOR_SRC_FORMAT_FRAME_LINEAR) |
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
