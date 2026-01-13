/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm MSM8660 Rotator driver - Hardware definitions
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_rotator driver from webOS kernel.
 */

#ifndef _ROTATOR_HW_H_
#define _ROTATOR_HW_H_

#include <linux/types.h>

/* Register offsets */
#define ROTATOR_INTR_ENABLE		0x0020
#define ROTATOR_INTR_STATUS		0x0024
#define ROTATOR_INTR_CLEAR		0x0028
#define ROTATOR_START			0x0030
#define ROTATOR_MAX_BURST_SIZE		0x0050
#define ROTATOR_HW_VERSION		0x0070

/* Source configuration registers */
#define ROTATOR_SRC_SIZE		0x1108
#define ROTATOR_SRCP0_ADDR		0x110c
#define ROTATOR_SRCP1_ADDR		0x1110
#define ROTATOR_SRC_YSTRIDE1		0x111c
#define ROTATOR_SRC_YSTRIDE2		0x1120
#define ROTATOR_SRC_FORMAT		0x1124
#define ROTATOR_SRC_UNPACK_PATTERN1	0x1128

/* Sub-block configuration */
#define ROTATOR_SUB_BLOCK_CFG		0x1138

/* Output configuration registers */
#define ROTATOR_OUT_PACK_PATTERN1	0x1154
#define ROTATOR_OUTP0_ADDR		0x1168
#define ROTATOR_OUTP1_ADDR		0x116c
#define ROTATOR_OUT_YSTRIDE1		0x1178
#define ROTATOR_OUT_YSTRIDE2		0x117c

/* Crop/position registers */
#define ROTATOR_SRC_XY			0x1200
#define ROTATOR_SRC_IMAGE_SIZE		0x1208

/* Interrupt bits */
#define ROTATOR_IRQ_DONE		BIT(0)
#define ROTATOR_IRQ_ERROR		BIT(1)
#define ROTATOR_IRQ_ALL			0x3

/* Hardware limits */
#define ROTATOR_MAX_WIDTH		0x1fff
#define ROTATOR_MAX_HEIGHT		0x1fff

/* Rotation flags (same as MDP) */
#define ROTATOR_ROT_NOP			0x00
#define ROTATOR_FLIP_LR			0x01
#define ROTATOR_FLIP_UD			0x02
#define ROTATOR_ROT_90			0x04

/* Convert rotation flags to hardware bitmask (bits 9-11 in SUB_BLOCK_CFG) */
#define ROTATOR_ROT_MASK		0x07

/* Sub-block config bits */
#define ROTATOR_SUB_BLOCK_ROT_EN	BIT(8)
#define ROTATOR_SUB_BLOCK_ROT_SHIFT	9
#define ROTATOR_SUB_BLOCK_CHROMA_SHIFT	18

/* Chroma sampling modes */
#define ROTATOR_CHROMA_RGB		0
#define ROTATOR_CHROMA_H2V1		1
#define ROTATOR_CHROMA_H1V2		2
#define ROTATOR_CHROMA_H2V2		3

/* Source format register bits */
#define ROTATOR_SRC_FORMAT_FRAME_LINEAR	(0 << 29)
#define ROTATOR_SRC_FORMAT_FRAME_TILE	(1 << 29)
#define ROTATOR_SRC_FORMAT_TILE_SIZE	BIT(22)
#define ROTATOR_SRC_FORMAT_FETCH_SHIFT	19
#define ROTATOR_SRC_FORMAT_UNPACK_ALIGN	BIT(18)
#define ROTATOR_SRC_FORMAT_UNPACK_TIGHT	BIT(17)
#define ROTATOR_SRC_FORMAT_UNPACK_SHIFT	13
#define ROTATOR_SRC_FORMAT_BPP_SHIFT	9
#define ROTATOR_SRC_FORMAT_ALPHA	BIT(8)
#define ROTATOR_SRC_FORMAT_ALPHA_SHIFT	6
#define ROTATOR_SRC_FORMAT_R_SHIFT	4
#define ROTATOR_SRC_FORMAT_B_SHIFT	2
#define ROTATOR_SRC_FORMAT_G_SHIFT	0

/* Fetch plane modes */
#define ROTATOR_FETCH_PLANES_INTERLEAVED	0
#define ROTATOR_FETCH_PLANES_PLANAR		1
#define ROTATOR_FETCH_PLANES_PSEUDO		2

/* Component bit depth */
#define ROTATOR_COMP_5BITS		1
#define ROTATOR_COMP_6BITS		2
#define ROTATOR_COMP_8BITS		3

/* Pack pattern helper */
#define ROTATOR_PACK_PATTERN(a, x, y, z, bits) \
	(((a) << ((bits) * 3)) | ((x) << ((bits) * 2)) | \
	 ((y) << (bits)) | (z))

/* Component indices for pack pattern */
#define CLR_G		0
#define CLR_B		1
#define CLR_R		2
#define CLR_ALPHA	3
#define CLR_Y		CLR_G
#define CLR_CB		CLR_B
#define CLR_CR		CLR_R

/* Hardware functions */
int rotator_hw_reset(void __iomem *base);
void rotator_hw_enable_irq(void __iomem *base);
void rotator_hw_disable_irq(void __iomem *base);
void rotator_hw_clear_irq(void __iomem *base);
u32 rotator_hw_get_irq_status(void __iomem *base);
u32 rotator_hw_get_version(void __iomem *base);
void rotator_hw_start(void __iomem *base);

/* Buffer configuration */
void rotator_hw_set_src_size(void __iomem *base, u32 width, u32 height);
void rotator_hw_set_src_addr(void __iomem *base, dma_addr_t y_addr,
			     dma_addr_t c_addr);
void rotator_hw_set_dst_addr(void __iomem *base, dma_addr_t y_addr,
			     dma_addr_t c_addr);
void rotator_hw_set_strides(void __iomem *base, u32 src_stride, u32 dst_stride);
void rotator_hw_set_rotation(void __iomem *base, u32 rotation, u32 chroma);
void rotator_hw_set_format_rgb(void __iomem *base, u32 bpp, bool has_alpha);
void rotator_hw_set_format_yuv(void __iomem *base, u32 chroma_mode, bool cbcr);

#endif /* _ROTATOR_HW_H_ */
