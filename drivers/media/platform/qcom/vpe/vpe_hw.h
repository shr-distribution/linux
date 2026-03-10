/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm MSM8660 Video Processing Engine (VPE) driver
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_vpe1 driver from webOS kernel.
 */

#ifndef _VPE_HW_H_
#define _VPE_HW_H_

#include <linux/types.h>

/* VPE Register Offsets */
#define VPE_INTR_ENABLE			0x0020
#define VPE_INTR_STATUS			0x0024
#define VPE_INTR_CLEAR			0x0028
#define VPE_DL0_START			0x0030
#define VPE_HW_VERSION			0x0070
#define VPE_SW_RESET			0x0074
#define VPE_AXI_RD_ARB_CONFIG		0x0078
#define VPE_SEL_CLK_OR_HCLK_TEST_BUS	0x007C
#define VPE_CGC_EN			0x0100

/* Command and status registers */
#define VPE_CMD_STATUS			0x10008
#define VPE_PROFILE_EN			0x10010
#define VPE_PROFILE_COUNT		0x10014
#define VPE_CMD_MODE			0x10060

/* Source configuration registers */
#define VPE_SRC_SIZE			0x10108
#define VPE_SRCP0_ADDR			0x1010C
#define VPE_SRCP1_ADDR			0x10110
#define VPE_SRC_YSTRIDE1		0x1011C
#define VPE_SRC_FORMAT			0x10124
#define VPE_SRC_UNPACK_PATTERN1		0x10128

/* Operation mode and scaling registers */
#define VPE_OP_MODE			0x10138
#define VPE_SCALE_PHASEX_INIT		0x1013C
#define VPE_SCALE_PHASEY_INIT		0x10140
#define VPE_SCALE_PHASEX_STEP		0x10144
#define VPE_SCALE_PHASEY_STEP		0x10148

/* Output configuration registers */
#define VPE_OUT_FORMAT			0x10150
#define VPE_OUT_PACK_PATTERN1		0x10154
#define VPE_OUT_SIZE			0x10164
#define VPE_OUTP0_ADDR			0x10168
#define VPE_OUTP1_ADDR			0x1016C
#define VPE_OUT_YSTRIDE1		0x10178
#define VPE_OUT_XY			0x1019C

/* Source ROI registers */
#define VPE_SRC_XY			0x10200
#define VPE_SRC_IMAGE_SIZE		0x10208

/* Scale configuration */
#define VPE_SCALE_CONFIG		0x10230

/* Deinterlace registers */
#define VPE_DEINT_STATUS		0x30000
#define VPE_DEINT_DECISION		0x30004
#define VPE_DEINT_COEFF0		0x30010

/* Scale status and coefficient registers */
#define VPE_SCALE_STATUS		0x50000
#define VPE_SCALE_SVI_PARAM		0x50010
#define VPE_SCALE_SHARPEN_CFG		0x50020
#define VPE_SCALE_COEFF_LSP_0		0x50400
#define VPE_SCALE_COEFF_MSP_0		0x50404

#define VPE_SCALE_COEFF_LSBn(n)		(0x50400 + 8 * (n))
#define VPE_SCALE_COEFF_MSBn(n)		(0x50404 + 8 * (n))
#define VPE_SCALE_COEFF_NUM		32

/* Hardware version and reset values */
#define VPE_HW_VERSION_VALUE		0x00080308
#define VPE_SW_RESET_VALUE		0x00000010	/* bit 4 for PPP */
#define VPE_AXI_RD_ARB_CONFIG_VALUE	0x124924
#define VPE_CMD_MODE_VALUE		0x1
#define VPE_DEFAULT_OP_MODE_VALUE	0x40FC0004
#define VPE_CGC_ENABLE_VALUE		0xffff
#define VPE_DEFAULT_SCALE_CONFIG	0x3c

/* VPE Clock rate */
#define VPE_CLOCK_RATE			160000000

/* Interrupt status bits */
#define VPE_INTR_STATUS_DONE		BIT(0)

/* Operation mode bits */
#define VPE_OP_MODE_ROTATION_MASK	GENMASK(10, 8)
#define VPE_OP_MODE_ROT_0		(0 << 8)
#define VPE_OP_MODE_ROT_90		(1 << 8)
#define VPE_OP_MODE_ROT_180		(2 << 8)
#define VPE_OP_MODE_ROT_270		(3 << 8)

/* Scale phase constants (fixed point 3.29 format) */
#define SCALER_PHASE_BITS		29
#define HAL_MDP_PHASE_STEP_2P50		0x50000000
#define HAL_MDP_PHASE_STEP_1P66		0x35555555
#define HAL_MDP_PHASE_STEP_1P25		0x28000000

/* VPE context structure */
struct vpe_ctx;

/* Hardware functions */
int vpe_hw_reset(void __iomem *base);
void vpe_hw_enable_irq(void __iomem *base);
void vpe_hw_disable_irq(void __iomem *base);
void vpe_hw_clear_irq(void __iomem *base);
u32 vpe_hw_get_irq_status(void __iomem *base);
u32 vpe_hw_get_version(void __iomem *base);
void vpe_hw_start(void __iomem *base);

/* Configuration functions */
void vpe_hw_set_src_addr(void __iomem *base, dma_addr_t y_addr, dma_addr_t cbcr_addr);
void vpe_hw_set_dst_addr(void __iomem *base, dma_addr_t y_addr, dma_addr_t cbcr_addr);
void vpe_hw_set_src_size(void __iomem *base, u32 width, u32 height, u32 stride);
void vpe_hw_set_dst_size(void __iomem *base, u32 width, u32 height, u32 stride);
void vpe_hw_set_scale(void __iomem *base, u32 src_w, u32 src_h, u32 dst_w, u32 dst_h);
void vpe_hw_set_rotation(void __iomem *base, int rotation);
void vpe_hw_set_op_mode(void __iomem *base, u32 mode);

#endif /* _VPE_HW_H_ */
