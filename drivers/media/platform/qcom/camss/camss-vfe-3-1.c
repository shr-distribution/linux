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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>

#include "camss.h"

/*
 * AXI output mode selection for VFE31.
 * Values from webOS msm_vfe31.c vfe31_config_axi():
 *   0x01  = OUTPUT_1_AND_3 mode (default, matches webOS preview/video)
 *   0x200 = OUTPUT_2 preview mode (semi-planar Y+CbCr)
 *   0x60  = CAMIF_TO_AXI raw snapshot mode (WM0 only, no XBAR)
 *
 * WebOS uses 0x01 with XBAR CFG1=0x1a1b (VIDEO_MODE routing).
 * Can be changed at runtime via:
 *   echo 0x01 > /sys/module/qcom_camss/parameters/vfe31_axi_output_mode
 */
int vfe31_axi_output_mode = 0x01;
module_param(vfe31_axi_output_mode, int, 0644);
MODULE_PARM_DESC(vfe31_axi_output_mode,
		 "VFE31 AXI output mode (0x200=preview, 0x01=preview+video, 0x60=raw)");

/*
 * VFE31 video output enable:
 *   1 = Video output enabled (default, matches webOS - uses XBAR CFG1=0x1a1b)
 *   0 = Video output disabled (uses XBAR CFG1=0x1a03)
 *
 * WebOS uses video mode routing for preview, which routes data to WM2/WM3
 * instead of WM0/WM1, matching the XBAR CFG1 = 0x1a1b seen in registers.
 */
/*
 * VFE31 video output enable.
 * When enabled (1), WM4/WM5 are configured to mirror preview WM0/WM1.
 * WM4/WM5 IRQs are silently ignored since they're just auxiliary routing
 * paths - buffer completion is tracked via WM0/WM1 only.
 *
 * Default is 1 to match webOS behavior and enable full dual-output mode.
 */
int vfe31_video_output_enable = 1;
module_param(vfe31_video_output_enable, int, 0644);
MODULE_PARM_DESC(vfe31_video_output_enable,
		 "VFE31 video output enable (0=preview only, 1=with WM4/WM5)");

/*
 * VFE31 UV swap control for debugging color issues.
 * When enabled, swaps the DEMUX_EVEN_CFG odd byte to reverse UV channel order.
 * This converts NV16 (CbCr) output to NV61 (CrCb) order or vice versa.
 *
 * 0 = Normal (webOS default: UYVY → 0xC9CA)
 * 1 = Swap UV (UYVY → 0xC9AC, exchanges Cb and Cr)
 */
static int vfe31_swap_uv = 0;
module_param(vfe31_swap_uv, int, 0644);
MODULE_PARM_DESC(vfe31_swap_uv,
		 "VFE31 swap UV channels (0=normal CbCr, 1=swap to CrCb)");

/*
 * VFE31 XBAR_CFG1 override for testing data routing configurations.
 * Controls how Y (luma) and CbCr (chroma) are routed to Write Masters.
 *
 * Known values:
 *   0x1A03 = Y→WM0, CbCr→DISABLED (Qualcomm default - BROKEN for semi-planar!)
 *   0x1A13 = Y→WM0, CbCr→WM1 (preview-only mode - CbCr often corrupted)
 *   0x1A1B = Y→WM0+WM4, CbCr→WM1 (webOS default)
 *   0x1A9B = Y→WM0+WM4, CbCr→WM1+WM5 (full dual video mode - works correctly!)
 *
 * Default 0x1A9B for correct CbCr plane data. WM4/WM5 IRQs are silently
 * ignored - we only track buffer completion via WM0/WM1.
 * Set via: echo 0x1a1b > /sys/module/qcom_camss/parameters/vfe31_xbar_cfg1
 */
int vfe31_xbar_cfg1 = 0x1A9B;
module_param(vfe31_xbar_cfg1, int, 0644);
MODULE_PARM_DESC(vfe31_xbar_cfg1,
		 "VFE31 XBAR_CFG1 routing (0x1a13=preview, 0x1a1b=video, 0x1a9b=dual)");

/* External module parameters from camss-vfe.c */
extern int software_sof_enable;
extern int software_eof_enable;

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
#define VFE_0_GLOBAL_RESET_CMD_AXI	BIT(9)

#define VFE_0_CGC_OVERRIDE		0x00C
#define VFE_0_CGC_OVERRIDE_1		0x00C

#define VFE_0_MODULE_CFG		0x010
#define VFE_0_MODULE_CFG_DEMUX		BIT(2)
#define VFE_0_MODULE_CFG_CHROMA_UPSAMPLE BIT(3)
/* Statistics module enable bits (from Mako kernel) */
#define VFE_0_MODULE_CFG_STATS_AE_EN	BIT(5)	/* AEC/BG stats enable */
#define VFE_0_MODULE_CFG_STATS_AF_EN	BIT(6)	/* AF/BF stats enable */
#define VFE_0_MODULE_CFG_STATS_AWB_EN	BIT(7)	/* AWB stats enable */
#define VFE_0_MODULE_CFG_STATS_RS_EN	BIT(8)	/* Row Sum stats enable */
#define VFE_0_MODULE_CFG_STATS_CS_EN	BIT(9)	/* Column Sum stats enable */
#define VFE_0_MODULE_CFG_STATS_IHIST_EN	BIT(15)	/* Image Histogram enable */
#define VFE_0_MODULE_CFG_SCALE_ENC	BIT(23)
#define VFE_0_MODULE_CFG_CROP_ENC	BIT(27)

#define VFE_0_CORE_CFG			0x014
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR	0x4
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB	0x5
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY	0x6
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY	0x7
/*
 * Bit 6 of VFE_CFG_OFF: webOS always sets this (0x46 instead of 0x06).
 * Purpose unknown, but required for data path to work.
 */
#define VFE_0_CORE_CFG_INPUT_MUX_ENABLE		BIT(6)

#define VFE_0_IRQ_CMD			0x018
#define VFE_0_IRQ_CMD_GLOBAL_CLEAR	BIT(0)

/*
 * IRQ_MASK_0 / IRQ_STATUS_0 bit definitions (0x01C / 0x02C)
 *
 * Per Mako/webOS kernel msm_vfe31.h:
 *   Bit 0:  CAMIF_SOF (Start of Frame)
 *   Bit 1:  (reserved in VFE31)
 *   Bit 2:  CAMIF_EOF (End of Frame) - NOTE: bit 2, not bit 1!
 *   Bit 3:  (reserved)
 *   Bit 4:  EPOCH_IRQ_0
 *   Bit 5:  REG_UPDATE
 *   Bit 6:  RESET_ACK (also in STATUS_1 bit 22)
 *   Bit 8-14: IMAGE_MASTER_0-6_PING_PONG
 *   Bit 13: STATS_AEC
 *   Bit 14: STATS_AF
 *   Bit 15: STATS_AWB
 *   Bit 16: STATS_RS
 *   Bit 17: STATS_CS
 *   Bit 18: STATS_IHIST
 *   Bit 21-23: IMAGE_COMPOSITE_DONE_0/1/2
 *   Bit 24: STATS_COMPOSITE
 *   Bit 25-27: SYNC_TIMER_0/1/2
 *   Bit 28-31: ASYNC_TIMER_0/1/2/3
 */
#define VFE_0_IRQ_MASK_0		0x01C
#define VFE_0_IRQ_MASK_0_CAMIF_SOF			BIT(0)
#define VFE_0_IRQ_MASK_0_CAMIF_EOF			BIT(2)  /* bit 2, not 1! */
#define VFE_0_IRQ_MASK_0_EPOCH_IRQ_0			BIT(4)
#define VFE_0_IRQ_MASK_0_REG_UPDATE			BIT(5)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define VFE_0_IRQ_MASK_0_STATS_AEC			BIT(13)
#define VFE_0_IRQ_MASK_0_STATS_AF			BIT(14)
#define VFE_0_IRQ_MASK_0_STATS_AWB			BIT(15)
#define VFE_0_IRQ_MASK_0_STATS_RS			BIT(16)
#define VFE_0_IRQ_MASK_0_STATS_CS			BIT(17)
#define VFE_0_IRQ_MASK_0_STATS_IHIST			BIT(18)
#define VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 21)
#define VFE_0_IRQ_MASK_0_STATS_COMPOSITE		BIT(24)
#define VFE_0_IRQ_MASK_0_SYNC_TIMER_0			BIT(25)
#define VFE_0_IRQ_MASK_0_SYNC_TIMER_1			BIT(26)
#define VFE_0_IRQ_MASK_0_SYNC_TIMER_2			BIT(27)
#define VFE_0_IRQ_MASK_0_ASYNC_TIMER_0			BIT(28)
#define VFE_0_IRQ_MASK_0_ASYNC_TIMER_1			BIT(29)
#define VFE_0_IRQ_MASK_0_ASYNC_TIMER_2			BIT(30)
#define VFE_0_IRQ_MASK_0_ASYNC_TIMER_3			BIT(31)
#define VFE_0_IRQ_MASK_0_RESET_ACK			BIT(31)
#define VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? BIT(5) : 0)

/*
 * IRQ_MASK_1 / IRQ_STATUS_1 bit definitions (0x020 / 0x030)
 *
 * Per Mako/webOS kernel msm_vfe31.h, these are error interrupts:
 *   Bit 0:  CAMIF_ERROR
 *   Bit 1:  STATS_CS_OVERFLOW
 *   Bit 2:  STATS_IHIST_OVERFLOW
 *   Bit 3:  REALIGN_BUF_Y_OVERFLOW
 *   Bit 4:  REALIGN_BUF_CB_OVERFLOW
 *   Bit 5:  REALIGN_BUF_CR_OVERFLOW
 *   Bit 6:  VIOLATION
 *   Bit 7-13: IMAGE_MASTER_0-6_BUS_OVERFLOW
 *   Bit 14-20: STATS buffer overflows (AE, AF, AWB, RS, CS, IHIST, SKIN)
 *   Bit 21: AXI_ERROR
 *   Bit 22: RESET_ACK (VFE31-specific location!)
 *   Bit 23: BUS_BDG_HALT_ACK
 */
#define VFE_0_IRQ_MASK_1		0x020
#define VFE_0_IRQ_MASK_1_CAMIF_ERROR			BIT(0)
#define VFE_0_IRQ_MASK_1_STATS_CS_OVERFLOW		BIT(1)
#define VFE_0_IRQ_MASK_1_STATS_IHIST_OVERFLOW		BIT(2)
#define VFE_0_IRQ_MASK_1_REALIGN_BUF_Y_OVERFLOW		BIT(3)
#define VFE_0_IRQ_MASK_1_REALIGN_BUF_CB_OVERFLOW	BIT(4)
#define VFE_0_IRQ_MASK_1_REALIGN_BUF_CR_OVERFLOW	BIT(5)
#define VFE_0_IRQ_MASK_1_VIOLATION			BIT(6)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	BIT((n) + 7)
#define VFE_0_IRQ_MASK_1_STATS_AE_BUS_OVERFLOW		BIT(14)
#define VFE_0_IRQ_MASK_1_STATS_AF_BUS_OVERFLOW		BIT(15)
#define VFE_0_IRQ_MASK_1_STATS_AWB_BUS_OVERFLOW		BIT(16)
#define VFE_0_IRQ_MASK_1_STATS_RS_BUS_OVERFLOW		BIT(17)
#define VFE_0_IRQ_MASK_1_STATS_CS_BUS_OVERFLOW		BIT(18)
#define VFE_0_IRQ_MASK_1_STATS_IHIST_BUS_OVERFLOW	BIT(19)
#define VFE_0_IRQ_MASK_1_STATS_SKIN_BUS_OVERFLOW	BIT(20)
#define VFE_0_IRQ_MASK_1_AXI_ERROR			BIT(21)
/*
 * VFE31 reset acknowledge is in STATUS_1 bit 22, not STATUS_0 bit 31.
 * This differs from later VFE versions.
 */
#define VFE_0_IRQ_MASK_1_RESET_ACK			BIT(22)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		BIT(23)

/* Common error mask for all error bits in IRQ_MASK_1/STATUS_1 */
#define VFE_0_IRQ_MASK_1_ERROR_ONLY			0x003FFFFF

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
#define VFE_0_IRQ_STATUS_1_VIOLATION			BIT(6)
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
 * VFE31 BUS_CFG register at 0x03C - CRITICAL for DMA operation!
 *
 * WebOS register dumps show this register contains 0x02AAA771 during operation.
 * The lower bits enable write paths required for DMA:
 *   Bit 0: stripeRdPathEn
 *   Bit 4: encYWrPathEn - enables encoder Y write path
 *   Bit 5: encCbcrWrPathEn - enables encoder CbCr write path
 *   Bit 6: viewYWrPathEn - enables viewfinder Y write path
 *   Bit 7: viewCbcrWrPathEn - enables viewfinder CbCr write path
 *
 * Without setting this register, DMA writes never complete and the
 * ping_pong status register never toggles.
 *
 * The upper bits (0x02AAA) appear to be timing/strobe configuration.
 *
 * IMPORTANT: webOS uses 0x02AAA771 but this has bit 7 DISABLED!
 * WM1 (CbCr plane) uses the viewfinder CbCr write path, so we MUST
 * enable bit 7 (viewCbcrWrPathEn) for NV16 semi-planar output to work.
 * Without bit 7, WM1 never receives data (ping_pong bit 1 stays at 0).
 *
 * Changed from 0x02AAA771 to 0x02AAA7F1 to enable all Y+CbCr paths.
 */
#define VFE_0_BUS_CFG_WEBOS_VALUE		0x02AAA7F1
#define VFE_0_BUS_CFG_ENC_Y_WR_PATH_EN		BIT(4)
#define VFE_0_BUS_CFG_ENC_CBCR_WR_PATH_EN	BIT(5)
#define VFE_0_BUS_CFG_VIEW_Y_WR_PATH_EN		BIT(6)
#define VFE_0_BUS_CFG_VIEW_CBCR_WR_PATH_EN	BIT(7)
#define VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT	8
#define VFE_0_BUS_CFG_RAW_PIXEL_8BIT		(0 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_PIXEL_10BIT		(1 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_PIXEL_12BIT		(2 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT	10
#define VFE_0_BUS_CFG_RAW_WR_PATH_DISABLED	(0 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)
#define VFE_0_BUS_CFG_RAW_WR_PATH_ENC_CBCR	(1 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)

/*
 * ============================================================================
 * VFE31 CROSSBAR (XBAR) AND AXI OUTPUT CONFIGURATION
 * ============================================================================
 *
 * The VFE31 crossbar routes processed image data from the ISP pipeline to
 * Write Masters (WM0-WM6) for DMA transfer to memory. Unlike VFE41+ which
 * has per-WM XBAR registers at 0x058+, VFE31 uses only TWO global registers:
 *
 *   Register        Offset   Description
 *   ─────────────────────────────────────────────────────────────────────────
 *   XBAR_CFG0       0x040    AXI output mode selection
 *   XBAR_CFG1       0x044    Data routing configuration
 *
 * Source: Qualcomm msm_vfe31.h (V31_XBAR_CFG_OFF=0x40, V31_XBAR_CFG_LEN=8)
 *
 * IMPORTANT: VFE31 does NOT have per-WM XBAR registers! The address 0x058
 * in VFE31 is WM0_WR_CFG, not an XBAR register. Do not apply VFE41 XBAR
 * patterns to VFE31.
 *
 * ============================================================================
 * XBAR_CFG0 (0x040) - AXI Output Mode
 * ============================================================================
 *
 * Selects the overall output routing mode. Values from msm_vfe31.c:
 *
 *   Value   Mode                    Description
 *   ─────────────────────────────────────────────────────────────────────────
 *   0x01    OUTPUT_1_AND_3          Preview + Video (WM0/1 + WM4/5)
 *   0x60    CAMIF_TO_AXI_VIA_OUT2   Raw bypass direct to WM0
 *   0x200   OUTPUT_2                Preview only (older mode)
 *
 * webOS uses 0x01 for all modes (preview, photo capture, video recording).
 */
#define VFE_0_BUS_XBAR_CFG0			0x040
#define VFE_0_BUS_AXI_OUT_MODE_CFG		VFE_0_BUS_XBAR_CFG0

/* XBAR_CFG0 values */
#define VFE_0_BUS_XBAR_CFG0_OUTPUT_1_AND_3	0x01
#define VFE_0_BUS_XBAR_CFG0_RAW_BYPASS		0x60
#define VFE_0_BUS_XBAR_CFG0_OUTPUT_2		0x200

/* Aliases for compatibility */
#define VFE_0_BUS_XBAR_CFG0_PIX_MODE		VFE_0_BUS_XBAR_CFG0_OUTPUT_1_AND_3
#define VFE_0_BUS_AXI_OUT_MODE_RAW_WM0		VFE_0_BUS_XBAR_CFG0_RAW_BYPASS

/*
 * ============================================================================
 * XBAR_CFG1 (0x044) - Data Routing Configuration
 * ============================================================================
 *
 * Controls how Y (luma) and CbCr (chroma) data are routed to Write Masters.
 * This is the ONLY register that controls semi-planar output routing!
 *
 * Bit Layout (32-bit register):
 *
 *   Bits    Field           Description
 *   ─────────────────────────────────────────────────────────────────────────
 *   [3:0]   Y_ROUTING       Luma output destination
 *                           0x3 = WM0 only (preview Y)
 *                           0xB = WM0 + WM4 (preview + video Y)
 *
 *   [7:4]   CBCR_ROUTING    Chroma output destination
 *                           0x0 = DISABLED (no CbCr output!)
 *                           0x1 = WM1 only (preview CbCr)
 *                           0x9 = WM1 + WM5 (preview + video CbCr)
 *
 *   [15:8]  ISP_PATH_CFG    ISP pipeline routing configuration
 *                           0x1A = Standard processed output (binary: 0001_1010)
 *
 *                           Bit analysis of 0x1A (bits 8-15 of register):
 *                             Bit 9:  Set (function unknown - possibly enable processed path)
 *                             Bit 11: Set (function unknown - possibly ISP module select)
 *                             Bit 12: Set (function unknown - possibly output format)
 *
 *                           Note: Qualcomm documentation for these bits is not publicly
 *                           available. Value 0x1A is used consistently in all known
 *                           Qualcomm msm_vfe31.c sources (Android kernels, webOS, etc.)
 *                           for ISP-processed output. Do not change without testing.
 *
 *   [31:16] Reserved        Should be 0
 *
 * CRITICAL: For semi-planar formats (NV12, NV16, NV21, NV61), bits [7:4]
 * MUST be non-zero to route CbCr to a write master. If CBCR_ROUTING=0,
 * the chroma plane receives duplicate luma data!
 */
#define VFE_0_BUS_XBAR_CFG1			0x044

/* XBAR_CFG1 bit field definitions */
#define VFE_0_BUS_XBAR_CFG1_Y_ROUTING_MASK	0x0000000F
#define VFE_0_BUS_XBAR_CFG1_Y_ROUTING_SHIFT	0
#define VFE_0_BUS_XBAR_CFG1_Y_WM0		0x3
#define VFE_0_BUS_XBAR_CFG1_Y_WM0_WM4		0xB

#define VFE_0_BUS_XBAR_CFG1_CBCR_ROUTING_MASK	0x000000F0
#define VFE_0_BUS_XBAR_CFG1_CBCR_ROUTING_SHIFT	4
#define VFE_0_BUS_XBAR_CFG1_CBCR_DISABLED	0x0
#define VFE_0_BUS_XBAR_CFG1_CBCR_WM1		0x1
#define VFE_0_BUS_XBAR_CFG1_CBCR_WM1_WM5	0x9

#define VFE_0_BUS_XBAR_CFG1_ISP_PATH_MASK	0x0000FF00
#define VFE_0_BUS_XBAR_CFG1_ISP_PATH_SHIFT	8
#define VFE_0_BUS_XBAR_CFG1_ISP_PATH_STANDARD	0x1A

/*
 * Pre-computed XBAR_CFG1 values:
 *
 *   Value   Binary                  Routing
 *   ─────────────────────────────────────────────────────────────────────────
 *   0x1A03  0001_1010_0000_0011     Y→WM0, CbCr→DISABLED (BROKEN for NV16!)
 *   0x1A13  0001_1010_0001_0011     Y→WM0, CbCr→WM1 (preview only)
 *   0x1A1B  0001_1010_0001_1011     Y→WM0+WM4, CbCr→WM1 (preview + video Y)
 *   0x1A9B  0001_1010_1001_1011     Y→WM0+WM4, CbCr→WM1+WM5 (dual video)
 *
 * Sources consulted for XBAR register behavior:
 *   - freedreno/kernel-msm hp-tenderloin-3.0 branch (webOS kernel)
 *   - android.googlesource.com/kernel/msm msm_vfe31.c/msm_vfe32.c
 *   - gitlab.com/k2wl/g2_kernel msm_vfe31.h (V31_XBAR_CFG_OFF=0x40, LEN=8)
 *
 * The original Qualcomm msm_vfe31.c OUTPUT_1_AND_3 code used 0x1A03, which
 * does NOT route CbCr anywhere, causing broken semi-planar output.
 *
 * For preview-only mode: use 0x1A13 (Y→WM0, CbCr→WM1)
 * For video mode: use 0x1A1B (Y→WM0+WM4, CbCr→WM1) - requires WM4 config
 *
 * Using 0x1A1B without WM4 configured causes WM1 to not receive data!
 */
#define VFE_0_BUS_XBAR_CFG1_PIX_MODE		0x1A13
#define VFE_0_BUS_XBAR_CFG1_VIDEO_MODE		0x1A1B

/* Legacy define - Qualcomm's broken value that doesn't route CbCr */
#define VFE_0_BUS_XBAR_CFG1_BROKEN_NO_CBCR	0x1A03

#define VFE_0_BUS_CFG_RAW_WR_PATH_VIEW_CBCR	(2 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)

/*
 * ============================================================================
 * VFE31 WRITE MASTER (WM) ASSIGNMENTS
 * ============================================================================
 *
 * The VFE31 has 7 Write Masters (WM0-WM6) for DMA output. Their assignments
 * depend on the output mode configured in XBAR_CFG0/CFG1:
 *
 *   WM      Preview Mode    Video Mode      Raw Mode
 *   ─────────────────────────────────────────────────────────────────────────
 *   WM0     Y plane         Y plane         Raw data
 *   WM1     CbCr plane      CbCr plane      -
 *   WM2     -               -               -
 *   WM3     -               -               -
 *   WM4     -               Video Y         -
 *   WM5     -               Video CbCr      -
 *   WM6     -               -               -
 *
 * IRQ Composite Groups (IRQ_COMPOSITE_MASK at 0x034):
 *   - Group 0 (bit 21): WM0 + WM4 completion
 *   - Group 1 (bit 22): Snapshot WM completion
 *   - Group 2 (bit 23): WM1 + WM5 completion (requires video mode)
 *
 * webOS vfe31_config_axi() output path comments:
 *   - out0 (PT/Preview): "use wm0&4 for preview"
 *   - out2 (V/Video):    "wm1&5 for video"
 */
#define VFE31_PREVIEW_WM_Y		0
#define VFE31_PREVIEW_WM_CBCR		1
#define VFE31_VIDEO_WM_Y		4
#define VFE31_VIDEO_WM_CBCR		5

/*
 * ============================================================================
 * CONFIGURATION SUMMARY
 * ============================================================================
 *
 * For semi-planar preview (NV12/NV16/NV21/NV61):
 *   XBAR_CFG0 = 0x01   (OUTPUT_1_AND_3)
 *   XBAR_CFG1 = 0x1A1B (Y→WM0, CbCr→WM1, ISP=standard)
 *   BUS_CFG   = 0x02AAA771 (enable Y+CbCr write paths)
 *
 * For raw bypass (SRGGB8/10/12):
 *   XBAR_CFG0 = 0x60   (RAW_BYPASS)
 *   XBAR_CFG1 = 0x00   (not used)
 *   BUS_CFG   = configured for raw pixel size
 *
 * For video recording (preview + video):
 *   XBAR_CFG0 = 0x01   (OUTPUT_1_AND_3)
 *   XBAR_CFG1 = 0x1A1B (or 0x1A9B for dual video output)
 *   Configure WM4/WM5 with video buffer addresses
 *   Enable COMPOSITE_DONE_2 IRQ for video frame completion
 */

/*
 * ============================================================================
 * LEGACY WEBOS VFE31 ENUMERATIONS (from msm_vfe31.h)
 * ============================================================================
 *
 * These enumerations are preserved from the webOS kernel msm_vfe31.h for
 * reference when debugging or extending the driver:
 *
 * enum VFE_AXI_OUTPUT_MODE {
 *     VFE_AXI_OUTPUT_MODE_Output1,                     // 0
 *     VFE_AXI_OUTPUT_MODE_Output2,                     // 1
 *     VFE_AXI_OUTPUT_MODE_Output1AndOutput2,           // 2
 *     VFE_AXI_OUTPUT_MODE_CAMIFToAXIViaOutput2,        // 3 (0x60)
 *     VFE_AXI_OUTPUT_MODE_Output2AndCAMIFToAXIViaOutput1,  // 4
 *     VFE_AXI_OUTPUT_MODE_Output1AndCAMIFToAXIViaOutput2,  // 5
 *     VFE_AXI_LAST_OUTPUT_MODE_ENUM
 * };
 *
 * enum VFE_RAW_WR_PATH_SEL {
 *     VFE_RAW_OUTPUT_DISABLED,          // 0 - Raw output disabled
 *     VFE_RAW_OUTPUT_ENC_CBCR_PATH,     // 1 - Route raw to encoder CbCr WM
 *     VFE_RAW_OUTPUT_VIEW_CBCR_PATH,    // 2 - Route raw to viewfinder CbCr WM
 *     VFE_RAW_OUTPUT_PATH_INVALID
 * };
 *
 * enum VFE_START_PIXEL_PATTERN {
 *     VFE_BAYER_RGRGRG,   // 0
 *     VFE_BAYER_GRGRGR,   // 1
 *     VFE_BAYER_BGBGBG,   // 2
 *     VFE_BAYER_GBGBGB,   // 3
 *     VFE_YUV_YCbYCr,     // 4 - YCBYCR
 *     VFE_YUV_YCrYCb,     // 5 - YCRYCB
 *     VFE_YUV_CbYCrY,     // 6 - CBYCRY (webOS default for UYVY input)
 *     VFE_YUV_CrYCbY      // 7 - CRYCBY
 * };
 *
 * IRQ_STATUS_1 Error Bits (per msm_vfe31.h):
 *   Bit 0:  CAMIF_ERROR
 *   Bit 1:  STATS_CS_OVWR
 *   Bit 2:  STATS_IHIST_OVWR
 *   Bit 3:  REALIGN_BUF_Y_OVFL
 *   Bit 4:  REALIGN_BUF_CB_OVFL
 *   Bit 5:  REALIGN_BUF_CR_OVFL
 *   Bit 6:  VIOLATION
 *   Bit 7:  IMG_MAST_0_BUS_OVFL (WM0 overflow)
 *   Bit 8:  IMG_MAST_1_BUS_OVFL (WM1 overflow)
 *   ...
 *   Bit 13: IMG_MAST_6_BUS_OVFL (WM6 overflow)
 *   Bit 14-21: STATS overflow bits
 *   Bit 22: RESET_ACK (VFE31 unique location!)
 *   Bit 23: AXI_HALT_ACK
 */

/*
 * NOTE: VFE31 does NOT have a VFE_CFG register at 0x01C!
 * The 0x01C offset is VFE_IRQ_MASK_0 in VFE31.
 *
 * Unlike VFE8x which has camif2vfeEnable/camif2busEnable bits in a config
 * register, VFE31 controls CAMIF routing via the AXI output mode at 0x40.
 * Setting AXI output mode to 0x60 (CAMIF_TO_AXI_VIA_OUTPUT_2) automatically
 * enables raw passthrough from CAMIF to memory via WM0.
 */

/*
 * CAMIF configuration - VFE31 specific
 * Register layout from webOS msm_vfe31.h:
 * - VFE_CAMIF_COMMAND at 0x1E0 (write commands)
 * - CAMIF config block at 0x1E4-0x203 (32 bytes)
 * - VFE_CAMIF_STATUS at 0x204 (read status)
 */
#define VFE_0_CAMIF_CMD			0x1E0
/*
 * VFE31 CAMIF START command = 1 (BIT(0) only)
 *
 * NOTE: The webOS msm_vfe31.h header defines CAMIF_COMMAND_START = 0x5,
 * but the actual VFE31 code in vfe31_start_common() writes 1, NOT 0x5:
 *   msm_io_w(1, vfe31_ctrl->vfebase + VFE_CAMIF_COMMAND);
 *
 * The 0x5 macro is defined but NEVER USED in VFE31. It appears to be
 * legacy from VFE8x where it IS used. For VFE31, only BIT(0) is needed.
 *
 * Reference: webOS msm_vfe31.c line 1002
 */
/*
 * CAMIF_CMD_START = 0x5 per webOS (bits 0 + 2):
 * - Bit 0: Enable image data capture at frame boundary
 * - Bit 2: Clear CAMIF_STATUS register
 * Writing both together ensures clean start.
 */
/*
 * CAMIF_CMD_START: webOS header defines 0x5 but actual code writes 1.
 * Use 1 to match webOS runtime behavior (BIT(0) = enable).
 */
#define VFE_0_CAMIF_CMD_START			0x1
#define VFE_0_CAMIF_CMD_STOP_IMMEDIATELY	0x2
#define VFE_0_CAMIF_CMD_STOP_AT_FRAME_BOUNDARY	0x0
#define VFE_0_CAMIF_CMD_CLEAR_CAMIF_STATUS	BIT(2)

/*
 * VFE31 CAMIF register block layout (32 bytes at 0x1E4-0x203):
 *
 * The VFE31 driver (V31_CAMIF_CFG command) copies 32 bytes from userspace
 * to V31_CAMIF_OFF (0x1E4). The layout is defined by the vfe_camifcfg structure
 * in webOS msm_vfe8x_proc.h.
 *
 * IMPORTANT: VFE31 does NOT have a separate CAMIF_CFG register with
 * camif2vfeEnable/camif2busEnable bits like VFE8x! Those bits exist only
 * on VFE8x at offset 0x118. On VFE31, data routing is controlled entirely
 * through the AXI output mode register at 0x040.
 *
 * Correct VFE31 CAMIF register map (from vfe_camifcfg structure):
 * 0x1E4: EFS_CFG - Embedded Frame Sync codes for MIPI CSI-2
 *        [7:0]   efsEndOfLine
 *        [15:8]  efsStartOfLine
 *        [23:16] efsEndOfFrame
 *        [31:24] efsStartOfFrame
 *        For APS mode, set to 0 (EFS codes ignored)
 * 0x1E8: FRAME_CFG - frame dimensions
 *        [13:0]  pixelsPerLine (including all bytes for YUV)
 *        [29:16] linesPerFrame
 * 0x1EC: WINDOW_WIDTH_CFG - horizontal capture window
 *        [13:0]  lastPixel (0-indexed)
 *        [29:16] firstPixel (usually 0)
 * 0x1F0: WINDOW_HEIGHT_CFG - vertical capture window
 *        [13:0]  lastLine (0-indexed)
 *        [29:16] firstLine (usually 0)
 * 0x1F4: SUBSAMPLE_CFG_0 - pixel/line subsampling
 *        [15:0]  pixelSkip (0xFFFF = no skip)
 *        [31:16] lineSkip (0xFFFF = no skip)
 * 0x1F8: SUBSAMPLE_CFG_1 - frame subsampling
 * 0x1FC: EPOCH_CFG - epoch interrupt lines
 * 0x200: (padding to 32 bytes)
 */
#define VFE_0_CAMIF_EFS_CFG		0x1E4	/* EFS codes (0 for APS mode) */
#define VFE_0_CAMIF_FRAME_CFG		0x1E8	/* Frame dimensions */
#define VFE_0_CAMIF_WINDOW_WIDTH_CFG	0x1EC	/* Horizontal window */
#define VFE_0_CAMIF_WINDOW_HEIGHT_CFG	0x1F0	/* Vertical window */
#define VFE_0_CAMIF_SUBSAMPLE_CFG_0	0x1F4	/* Subsample config */
#define VFE_0_CAMIF_SUBSAMPLE_CFG_1	0x1F8	/* Frame subsample */
#define VFE_0_CAMIF_IRQ_SUBSAMPLE_PATTERN 0x1F8	/* Alias for SUBSAMPLE_CFG_1 */
#define VFE_0_CAMIF_EPOCH_CFG		0x1FC	/* Epoch interrupt */

#define VFE_0_CAMIF_STATUS		0x204

/*
 * NOTE: VFE31 does NOT have separate RDI_CFG registers like VFE41+.
 * The data path is controlled entirely through CAMIF_CFG and AXI output mode.
 * Do NOT use VFE_0_RDI_CFG_x - it was incorrectly defined at the CAMIF_CFG offset.
 */

/* AXI bus configuration */
#define VFE_0_AXI_CMD			0x1D8
#define VFE_0_AXI_CMD_HALT		BIT(0)

#define VFE_0_AXI_STATUS		0x1DC
#define VFE_0_AXI_STATUS_HALT_ACK	BIT(0)

/*
 * Bus Status and Performance Monitor Registers (from Mako kernel)
 */
#define VFE_0_BUS_PING_PONG_STATUS	0x180
#define VFE_0_BUS_OPERATION_STATUS	0x184
#define VFE_0_BUS_PM_CMD		0x188
#define VFE_0_BUS_PM_CFG		0x18C
#define VFE_0_BUS_IMAGE_MASTER_0_WR_PM_STATS_0	0x190
#define VFE_0_BUS_IMAGE_MASTER_0_WR_PM_STATS_1	0x194

/*
 * VFE31 BUS IMAGE MASTER REGISTERS
 * =================================
 *
 * VFE31 AXI output block starts at 0x38, with write masters at 0x4C.
 * Each WM block is 0x18 (24) bytes with 6 registers.
 *
 * Register Layout per Write Master (WM0-WM5):
 *   0x04C + 0x18*n: WR_CFG       - Write master enable (bit 0)
 *   0x050 + 0x18*n: WR_PING_ADDR - Ping buffer physical address
 *   0x054 + 0x18*n: WR_PONG_ADDR - Pong buffer physical address
 *   0x058 + 0x18*n: WR_ADDR_CFG  - DMA address/burst configuration
 *   0x05C + 0x18*n: WR_UB_CFG    - Micro-block configuration
 *   0x060 + 0x18*n: WR_IMAGE_SIZE - Image dimensions
 *
 * WR_ADDR_CFG Register Format (0x058 + 0x18*n):
 * ---------------------------------------------
 * Bits [15:0]  - burst_words: Number of 32-bit words per DMA burst
 *                Formula: (bytes_per_line / 4) - 17
 *                For UYVY input: bytes_per_line = width * 2
 *
 * Bits [31:16] - lines: DMA line count control
 *                WM0 (Y plane):    lines=0 means "use height from IMAGE_SIZE"
 *                WM1 (CbCr plane): lines=N means "write N lines of CbCr data"
 *
 *                IMPORTANT: For CbCr write masters (WM1, WM5), the lines field
 *                must be set to (height - 24) for proper operation. Setting
 *                lines=0 causes only ~half the UV data to be written.
 *
 *                This is NOT a crop value - it controls DMA behavior.
 *                The exact hardware reason for the -24 offset is unknown,
 *                but this matches webOS register dumps and is empirically
 *                required for correct semi-planar (NV16) output.
 *
 *                NOTE: webOS only uses 640x480, so the 24 might be fixed or
 *                might scale with resolution (5% of 480 = 24). For higher
 *                resolutions, start with fixed 24 and adjust if needed.
 *
 * Resolution Scaling Formulas (UYVY input -> NV16 output):
 * --------------------------------------------------------
 * Given: width (pixels), height (lines)
 *
 *   bytes_per_line = width * 2  (UYVY = 2 bytes/pixel)
 *   words_per_line = bytes_per_line / 4
 *   burst = words_per_line - 17
 *
 *   WM0 WR_ADDR_CFG = (0 << 16) | (burst & 0xFFFF)
 *   WM1 WR_ADDR_CFG = ((height - 24) << 16) | (burst & 0xFFFF)
 *
 * WebOS @ 640x480:
 *   bytes_per_line = 640 * 2 = 1280
 *   words_per_line = 1280 / 4 = 320
 *   burst = 320 - 17 = 303 = 0x12F
 *   WM0: 0x0000012F, WM1: 0x01C8012F (456 = 480 - 24)
 *
 * Calculated @ 1280x1024 (UNTESTED - extrapolated from webOS formulas):
 *   bytes_per_line = 1280 * 2 = 2560
 *   words_per_line = 2560 / 4 = 640
 *   burst = 640 - 17 = 623 = 0x26F
 *   WM0: 0x0000026F, WM1: 0x03E8026F (1000 = 1024 - 24)
 *
 * NOTE: webOS register dumps show it ONLY uses:
 *   - Preview (WM0/WM1): 640x480 (burst=303)
 *   - Video (WM4/WM5): ~336x240 downscaled (burst=151, height=240)
 *   - CAMIF receives 1280x1279 but VFE scales/crops to output size
 *   webOS never configures WMs for full 1280x1024 output!
 *
 * WR_UB_CFG Register Format (0x05C + 0x18*n):
 * -------------------------------------------
 * Bits [15:0]  - height_minus_1: (height - 1)
 * Bits [31:16] - ub_depth: (words_per_line / 8) - 1
 *
 * Resolution Scaling:
 *   ub_depth = (bytes_per_line / 4 / 8) - 1 = (bytes_per_line / 32) - 1
 *   WR_UB_CFG = (ub_depth << 16) | (height - 1)
 *
 * WebOS @ 640x480: 0x002701DF = (39 << 16) | 479
 *   ub_depth = (1280/32) - 1 = 39, height_minus_1 = 479
 *
 * Calculated @ 1280x1024: 0x004F03FF = (79 << 16) | 1023
 *   ub_depth = (2560/32) - 1 = 79, height_minus_1 = 1023
 *
 * WR_IMAGE_SIZE Register Format (0x060 + 0x18*n):
 * -----------------------------------------------
 * Bits [15:0]  - size_cfg: ((height - 1) << 4) | 2
 * Bits [31:16] - stride: bytes_per_line / 16
 *
 * Resolution Scaling:
 *   stride = bytes_per_line / 16
 *   size_cfg = ((height - 1) << 4) | 2
 *   WR_IMAGE_SIZE = (stride << 16) | size_cfg
 *
 * WebOS @ 640x480: 0x00501DF2 = (80 << 16) | 0x1DF2
 *   stride = 1280/16 = 80, size_cfg = (479 << 4) | 2 = 0x1DF2
 *
 * Calculated @ 1280x1024: 0x00A03FF2 = (160 << 16) | 0x3FF2
 *   stride = 2560/16 = 160, size_cfg = (1023 << 4) | 2 = 0x3FF2
 *
 * WM4/WM5 (Video Path) - webOS uses downscaled ~336x240:
 *   WM4 WR_CFG = 0x01300097 = lines=304, burst=151
 *   WM5 WR_CFG = 0x02F80097 = lines=760, burst=151
 *   burst=151 → bytes_per_line = (151+17)*4 = 672 = 336 pixels
 *   WM4 y/x_off = 0x002700EF → height_minus_1=239 (height=240)
 *
 * Sources:
 *   - webOS kernel msm_vfe31.c register dumps
 *   - Empirical testing on HP TouchPad (APQ8060/VFE31)
 *   - Linux mainline camss-vfe-4-1.c (similar but not identical)
 */
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(n)		(0x04C + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(n)	(0x050 + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(n)	(0x054 + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(n)		(0x058 + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(n)		(0x05C + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(n)	(0x060 + 0x18 * (n))
/* VFE31 doesn't have per-WM framedrop - it's at global offsets 0x504+ */
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(n) (0x060 + 0x18 * (n))

/* VFE31 global framedrop registers (not per-WM like VFE41) */
#define VFE31_FRAMEDROP_ENC_Y_CFG		0x504
#define VFE31_FRAMEDROP_ENC_CBCR_CFG		0x508
#define VFE31_FRAMEDROP_ENC_Y_PATTERN		0x50C
#define VFE31_FRAMEDROP_ENC_CBCR_PATTERN	0x510
#define VFE31_FRAMEDROP_VIEW_Y_CFG		0x514
#define VFE31_FRAMEDROP_VIEW_CBCR_CFG		0x518
#define VFE31_FRAMEDROP_VIEW_Y_PATTERN		0x51C
#define VFE31_FRAMEDROP_VIEW_CBCR_PATTERN	0x520

/* Demux configuration */
#define VFE_0_DEMUX_CFG			0x284
#define VFE_0_DEMUX_CFG_PERIOD		0x3
#define VFE_0_DEMUX_GAIN_0		0x288
#define VFE_0_DEMUX_GAIN_0_CH0_EVEN	(0x80 << 0)
#define VFE_0_DEMUX_GAIN_0_CH0_ODD	(0x80 << 16)
#define VFE_0_DEMUX_GAIN_1		0x28C
#define VFE_0_DEMUX_GAIN_1_CH1		(0x80 << 0)
#define VFE_0_DEMUX_GAIN_1_CH2		(0x80 << 16)
#define VFE_0_DEMUX_EVEN_CFG		0x290

/* Chroma subsample */
#define VFE_0_CHROMA_SUBS_CFG		0x4F8

/*
 * Scale configuration - Main Scaler block
 * VFE31 Main Scaler spans 28 bytes (0x368-0x383), followed by WB at 0x384.
 * Unlike VFE4x, VFE31 does NOT have dedicated CROP_ENC registers.
 * Output cropping in VFE31 is handled by FOV (0x360) and scaler configuration.
 */
#define VFE_0_SCALE_ENC_Y_CFG		0x368
#define VFE_0_SCALE_ENC_CBCR_CFG	0x36C
/* Main scaler output crop is configured within the scaler structure (0x368-0x383) */

/* Output clamp */
#define VFE_0_CLAMP_ENC_MAX_CFG		0x524
#define VFE_0_CLAMP_ENC_MIN_CFG		0x528

/* Realign buffer configuration (from Mako kernel - NOT at 0x388 which is Color Cor) */
#define VFE_0_REALIGN_BUF_CFG		0x52C

/* Statistics configuration */
#define VFE_0_STATS_CFG			0x530
#define VFE_0_STATS_AE_CFG		0x534
#define VFE_0_STATS_AF_CFG		0x53C
#define VFE_0_STATS_AWB_CFG		0x54C
#define VFE_0_STATS_AWB_SGW_CFG		0x554
#define VFE_0_STATS_RS_CFG		0x56C
#define VFE_0_STATS_CS_CFG		0x574
#define VFE_0_STATS_IHIST_CFG		0x57C

/*
 * DMI (Direct Memory Interface) for LUT table access
 * Used to program gamma, rolloff, and histogram lookup tables
 */
#define VFE_0_DMI_CFG			0x598
#define VFE_0_DMI_CFG_DEFAULT		0x00000100
#define VFE_0_DMI_ADDR			0x59C
#define VFE_0_DMI_DATA_LO		0x5A4

/* DMI RAM selection values for VFE_0_DMI_CFG */
#define VFE_DMI_NO_MEM_SELECTED		0x0
#define VFE_DMI_ROLLOFF_RAM		0x1
#define VFE_DMI_RGBLUT_RAM_CH0_BANK0	0x2
#define VFE_DMI_RGBLUT_RAM_CH0_BANK1	0x3
#define VFE_DMI_RGBLUT_RAM_CH1_BANK0	0x4
#define VFE_DMI_RGBLUT_RAM_CH1_BANK1	0x5
#define VFE_DMI_RGBLUT_RAM_CH2_BANK0	0x6
#define VFE_DMI_RGBLUT_RAM_CH2_BANK1	0x7
#define VFE_DMI_STATS_HIST_RAM		0x8
#define VFE_DMI_RGBLUT_CHX_BANK0	0x9
#define VFE_DMI_RGBLUT_CHX_BANK1	0xA
#define VFE_DMI_LUMA_ADAPT_LUT_BANK0	0xB
#define VFE_DMI_LUMA_ADAPT_LUT_BANK1	0xC

/*
 * Additional processing module offsets (from Mako kernel)
 * These are used for ISP processing pipeline configuration.
 */
#define VFE_0_DEMOSAIC_CFG		0x298	/* Demosaic general */
#define VFE_0_DEMOSAIC_BPC_CFG		0x29C	/* Bad pixel correction */
#define VFE_0_DEMOSAIC_ABF_CFG		0x2A4	/* Adaptive bayer filter */
#define VFE_0_BLACK_LEVEL_CFG		0x264	/* Black level config */
#define VFE_0_ROLLOFF_CFG		0x274	/* Lens rolloff */
#define VFE_0_WB_CFG			0x384	/* White balance */
#define VFE_0_COLOR_COR_CFG		0x388	/* Color correction */
#define VFE_0_GAMMA_CFG			0x3BC	/* Gamma config */
#define VFE_0_LA_CFG			0x3C0	/* Luma adaptation */
#define VFE_0_CHROMA_EN_CFG		0x3C4	/* Chroma enhancement */
#define VFE_0_CHROMA_SUP_CFG		0x3E8	/* Chroma suppression */
#define VFE_0_MCE_CFG			0x3F4	/* Memory color enhancement */
#define VFE_0_SCE_CFG			0x418	/* Skin color enhancement */
#define VFE_0_ASF_CFG			0x4A0	/* Adaptive spatial filter */
#define VFE_0_S2Y_CFG			0x4D0	/* Scaler 2 Y */
#define VFE_0_S2CBCR_CFG		0x4E4	/* Scaler 2 CbCr */
#define VFE_0_FOV_CFG			0x360	/* Field of view */
#define VFE_0_CHROMA_UP_CFG		0x35C	/* Chroma upsample */

/* Timer registers */
#define VFE_0_SYNC_TIMER_OFF		0x20C
#define VFE_0_SYNC_TIMER_POLARITY_OFF	0x234
#define VFE_0_ASYNC_TIMER_OFF		0x238
#define VFE_0_TIMER_SELECT_OFF		0x25C

/*
 * Statistics Buffer Registers (from Mako kernel msm_vfe31.h)
 *
 * VFE31 has dedicated write paths for statistics data (AEC, AF, AWB, etc.)
 * Each statistics type has its own ping/pong buffers and address config.
 * These are separate from the image write masters (WM0-WM6).
 */
#define VFE_0_BUS_STATS_AEC_WR_PING_ADDR	0x0F4
#define VFE_0_BUS_STATS_AEC_WR_PONG_ADDR	0x0F8
#define VFE_0_BUS_STATS_AEC_WR_ADDR_CFG		0x0FC
#define VFE_0_BUS_STATS_AF_WR_PING_ADDR		0x100
#define VFE_0_BUS_STATS_AF_WR_PONG_ADDR		0x104
#define VFE_0_BUS_STATS_AF_WR_ADDR_CFG		0x108
#define VFE_0_BUS_STATS_AWB_WR_PING_ADDR	0x10C
#define VFE_0_BUS_STATS_AWB_WR_PONG_ADDR	0x110
#define VFE_0_BUS_STATS_AWB_WR_ADDR_CFG		0x114
#define VFE_0_BUS_STATS_RS_WR_PING_ADDR		0x118
#define VFE_0_BUS_STATS_RS_WR_PONG_ADDR		0x11C
#define VFE_0_BUS_STATS_RS_WR_ADDR_CFG		0x120
#define VFE_0_BUS_STATS_CS_WR_PING_ADDR		0x124
#define VFE_0_BUS_STATS_CS_WR_PONG_ADDR		0x128
#define VFE_0_BUS_STATS_CS_WR_ADDR_CFG		0x12C
#define VFE_0_BUS_STATS_HIST_WR_PING_ADDR	0x130
#define VFE_0_BUS_STATS_HIST_WR_PONG_ADDR	0x134
#define VFE_0_BUS_STATS_HIST_WR_ADDR_CFG	0x138
#define VFE_0_BUS_STATS_SKIN_WR_PING_ADDR	0x13C
#define VFE_0_BUS_STATS_SKIN_WR_PONG_ADDR	0x140
#define VFE_0_BUS_STATS_SKIN_WR_ADDR_CFG	0x144

/*
 * Statistics Composite Group Config
 * Allows combining multiple statistics IRQs into a single composite IRQ
 */
#define VFE_0_STATS_COMP_GRP_CFG		0x148

/*
 * Test Pattern Generator Registers (from Mako kernel)
 * Used for internal testing without a real sensor
 */
#define VFE_0_TESTGEN_STATUS		0x158
#define VFE_0_TESTGEN_CFG		0x15C
#define VFE_0_TESTGEN_SEED_0		0x160
#define VFE_0_TESTGEN_SEED_1		0x164
#define VFE_0_TESTGEN_SEED_2		0x168
#define VFE_0_TESTGEN_SEED_3		0x16C
#define VFE_0_TESTGEN_DIMS		0x170
#define VFE_0_TESTGEN_START_PIXEL	0x174

/*
 * MISR (Multiple Input Signature Register) for debug
 */
#define VFE_0_BUS_MISR_CFG		0x178
#define VFE_0_BUS_MISR_VALUE		0x17C

/*
 * NOTE: VFE31 does NOT have per-WM XBAR registers like VFE41/47/48.
 * VFE41+ has VFE_0_BUS_XBAR_CFG_x(x) at 0x90+ for per-WM stream routing.
 * VFE31 address 0x058 is WM0_WR_CFG, not an XBAR register!
 *
 * See VFE31 CROSSBAR documentation above for XBAR_CFG0/CFG1 details.
 */

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
	u32 hw_version;

	dev_info(vfe->camss->dev, "VFE hw_version: ENTER base=%px\n", vfe->base);

	hw_version = readl_relaxed(vfe->base + VFE_0_HW_VERSION);

	dev_info(vfe->camss->dev, "VFE hw_version: read complete = 0x%08x\n", hw_version);

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

/* Forward declarations for functions used in vfe31_enable */
static void vfe31_set_demux_cfg(struct vfe_device *vfe, struct vfe_line *line);
static void vfe31_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line);
static void vfe31_set_crop_cfg(struct vfe_device *vfe, struct vfe_line *line);

static void vfe31_global_reset(struct vfe_device *vfe)
{
	u32 hw_version;

	/*
	 * VFE31 on MSM8660 reset sequence based on webOS msm_vfe31.c:
	 *
	 * In webOS, the VFE footswitch (fs_vfe) is never explicitly toggled
	 * (HP_DISABLE is defined), so the VFE stays in a "warm" state from
	 * boot. In mainline, we enable VFE_GDSC via power-domains, giving
	 * the VFE a fresh power cycle. This requires a warm-up period
	 * before the reset command can be processed.
	 *
	 * Sequence:
	 * 1. Read HW version (verify accessibility)
	 * 2. Enable internal clock gates (CGC_OVERRIDE)
	 * 3. Wait for VFE to stabilize after GDSC power-on
	 * 4. Set up default register values (like webOS does post-reset)
	 * 5. Skip actual reset command to avoid hang
	 *
	 * The actual VFE_GLOBAL_RESET_CMD write hangs, likely because the
	 * VFE state machine expects certain AXI/bus configuration that
	 * webOS sets up earlier in its initialization flow.
	 */

	/* Debug: Read HW version to verify VFE is accessible */
	hw_version = readl_relaxed(vfe->base + VFE_0_HW_VERSION);
	dev_info(vfe->camss->dev, "VFE reset: HW version=0x%08x base=%pK\n",
		 hw_version, vfe->base);

	/*
	 * Follow exact webOS vfe31_reset() sequence:
	 * 1. Enable all module clocks (MODULE_CFG = 0x3FF)
	 * 2. Disable all IRQs (write 0 to IRQ_MASK_0/1)
	 * 3. Clear all pending IRQs (write 0xFFFFFFFF to IRQ_CLEAR_0/1)
	 * 4. Enable CGC override
	 */

	/*
	 * VFE31 Reset Sequence (matching webOS exactly):
	 *
	 * 1. Disable all IRQs
	 * 2. Clear all pending IRQs
	 * 3. Issue reset command (0x3FF to VFE_GLOBAL_RESET)
	 * 4. Wait for reset to complete
	 * 5. THEN set default register values (CGC, DEMUX, FRAMEDROP, CLAMP)
	 * 6. Reload all write masters (BUS_CMD = 0x3FFF per webOS)
	 *
	 * CRITICAL: The reset command clears all VFE registers, so default
	 * values MUST be set AFTER reset completes, not before!
	 */

	/* Clear shadow registers before reset */
	vfe->irq_mask0_shadow = 0;
	vfe->irq_mask1_shadow = 0;
	vfe->irq_comp_mask_shadow = 0;

	/* Step 1: Disable all IRQs before reset */
	dev_info(vfe->camss->dev, "VFE reset: disabling IRQs (MASK=0)\n");
	writel_relaxed(0x0, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(0x0, vfe->base + VFE_0_IRQ_MASK_1);
	wmb();

	/* Step 2: Clear all pending interrupts */
	dev_info(vfe->camss->dev, "VFE reset: clearing pending IRQs\n");
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_IRQ_CLEAR_1);
	writel_relaxed(1, vfe->base + VFE_0_IRQ_CMD);  /* Acknowledge IRQ clear */
	wmb();

	/* Step 3: Issue hardware reset command */
	dev_info(vfe->camss->dev, "VFE reset: sending hardware reset cmd (0x3FF to 0x04)\n");
	writel_relaxed(0x3FF, vfe->base + 0x04);  /* VFE_GLOBAL_RESET */
	wmb();

	/* Step 4: Wait for reset to complete - webOS waits for RESET_ACK IRQ, we use delay */
	usleep_range(2000, 3000);

	dev_info(vfe->camss->dev,
		 "VFE reset: hardware reset complete, IRQ_STATUS1=0x%08x\n",
		 readl_relaxed(vfe->base + 0x30));  /* VFE_IRQ_STATUS_1 */

	/*
	 * Step 5: Set default register values AFTER reset completes.
	 * This is exactly what webOS does in vfe31_process_reset_irq() ->
	 * vfe31_set_default_reg_values().
	 */

	/* Enable all internal clock gates (CGC_OVERRIDE) */
	dev_info(vfe->camss->dev, "VFE reset: enabling CGC override (0xFFFFF)\n");
	writel_relaxed(0xFFFFF, vfe->base + VFE_0_CGC_OVERRIDE);
	wmb();

	/* DEMUX gains - webOS default values */
	dev_info(vfe->camss->dev, "VFE reset: writing DEMUX gains (0x800080)\n");
	writel_relaxed(0x800080, vfe->base + VFE_0_DEMUX_GAIN_0);
	writel_relaxed(0x800080, vfe->base + VFE_0_DEMUX_GAIN_1);

	/* Frame drop configuration - accept all frames */
	dev_info(vfe->camss->dev, "VFE reset: writing framedrop config\n");
	writel_relaxed(0x1f, vfe->base + VFE31_FRAMEDROP_ENC_Y_CFG);
	writel_relaxed(0x1f, vfe->base + VFE31_FRAMEDROP_ENC_CBCR_CFG);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE31_FRAMEDROP_ENC_Y_PATTERN);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE31_FRAMEDROP_ENC_CBCR_PATTERN);
	writel_relaxed(0x1f, vfe->base + VFE31_FRAMEDROP_VIEW_Y_CFG);
	writel_relaxed(0x1f, vfe->base + VFE31_FRAMEDROP_VIEW_CBCR_CFG);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE31_FRAMEDROP_VIEW_Y_PATTERN);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE31_FRAMEDROP_VIEW_CBCR_PATTERN);

	/* Clamp configuration - 0x524=MAX, 0x528=MIN per webOS vfe31.h */
	dev_info(vfe->camss->dev, "VFE reset: writing clamp config\n");
	writel_relaxed(0xFFFFFF, vfe->base + VFE_0_CLAMP_ENC_MAX_CFG);
	writel_relaxed(0, vfe->base + VFE_0_CLAMP_ENC_MIN_CFG);
	wmb();

	/*
	 * Step 6: Reload all write masters.
	 * webOS register dump shows BUS_CMD = 0x3FFF (14 bits set).
	 * Use exact webOS value for compatibility.
	 */
	dev_info(vfe->camss->dev, "VFE reset: reloading all write masters (BUS_CMD=0x3FFF)\n");
	writel_relaxed(0x3FFF, vfe->base + VFE_0_BUS_CMD);
	wmb();

	/*
	 * Step 7: Clear AXI halt to ensure DMA can operate.
	 * After global reset, the AXI might be in halted state.
	 * Write 0 to VFE_AXI_CMD (0x1D8) to clear any halt condition.
	 * Without this, ping_pong register never toggles and DMA hangs.
	 */
	dev_info(vfe->camss->dev, "VFE reset: clearing AXI halt (AXI_CMD=0)\n");
	writel_relaxed(0x0, vfe->base + VFE_0_AXI_CMD);
	wmb();

	dev_info(vfe->camss->dev, "VFE reset: complete, all defaults applied\n");

	/* Set flag to indicate reset done - vfe_reset() will check this */
	vfe->vfe31_reset_done = true;
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

	/*
	 * If VIOLATION_STATUS = 0, this is a spurious interrupt.
	 * webOS doesn't mask VIOLATION IRQ, so this can happen during
	 * normal operation. Only log actual violations.
	 */
	if (!violation) {
		dev_info(vfe->camss->dev, "VFE31 VIOLATION IRQ (status=0, spurious)\n");
		return;
	}

	dev_err(vfe->camss->dev, "VFE31 VIOLATION! status=0x%08x\n", violation);
	if (violation & BIT(0))
		dev_err(vfe->camss->dev, "  CAMIF_OVERFLOW\n");
	if (violation & BIT(1))
		dev_err(vfe->camss->dev, "  DEMOSAIC_OUTPUT_OVERFLOW\n");
	if (violation & BIT(2))
		dev_err(vfe->camss->dev, "  DEMUX_OUTPUT_OVERFLOW\n");
	if (violation & BIT(3))
		dev_err(vfe->camss->dev, "  CLF_OUTPUT_OVERFLOW\n");
	if (violation & BIT(4))
		dev_err(vfe->camss->dev, "  CC_OUTPUT_OVERFLOW\n");
	if (violation & BIT(5))
		dev_err(vfe->camss->dev, "  REALIGN_BUF_OVERFLOW\n");
	if (violation & BIT(6))
		dev_err(vfe->camss->dev, "  SCALE_OUTPUT_Y_OVERFLOW\n");
	if (violation & BIT(7))
		dev_err(vfe->camss->dev, "  SCALE_OUTPUT_CBCR_OVERFLOW\n");
	if (violation & BIT(8))
		dev_err(vfe->camss->dev, "  ASF_OUTPUT_OVERFLOW\n");
	if (violation & BIT(9))
		dev_err(vfe->camss->dev, "  CROP_OUTPUT_Y_OVERFLOW\n");
	if (violation & BIT(10))
		dev_err(vfe->camss->dev, "  CROP_OUTPUT_CBCR_OVERFLOW\n");
	if (violation & BIT(20))
		dev_err(vfe->camss->dev, "  AXI_WM0_FIFO_OVERFLOW\n");
	if (violation & BIT(21))
		dev_err(vfe->camss->dev, "  AXI_WM1_FIFO_OVERFLOW\n");
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
	static ktime_t first_irq_time;
	static int irq_count;
	static u32 last_ping_pong;
	ktime_t now;
	u32 value0, value1, ping_pong;
	int i, j;

	vfe->res->hw_ops->isr_read(vfe, &value0, &value1);

	/* Read ping-pong status to see if data is reaching AXI bus */
	ping_pong = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	irq_count++;
	now = ktime_get();
	if (irq_count == 1)
		first_irq_time = now;

	/* Debug: log all interrupts with timing and ping-pong status */
	dev_info(vfe->camss->dev,
		 "[TIMING] VFE IRQ #%d: status0=0x%08x status1=0x%08x ping_pong=0x%08x at %lld ns (delta=%lld ns)\n",
		 irq_count, value0, value1, ping_pong,
		 ktime_to_ns(now),
		 ktime_to_ns(now) - ktime_to_ns(first_irq_time));

	/* Log if ping-pong status changes (indicates data flow) */
	if (ping_pong != last_ping_pong) {
		dev_info(vfe->camss->dev,
			 "VFE: PING_PONG changed: 0x%08x -> 0x%08x (data flowing!)\n",
			 last_ping_pong, ping_pong);
		last_ping_pong = ping_pong;
	}

	/* Debug: dump WM0 registers on first few IRQs to verify DMA config */
	if (irq_count <= 3) {
		u32 wm0_cfg = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(0));
		u32 wm0_ping = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(0));
		u32 wm0_pong = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(0));
		u32 wm0_size = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(0));
		u32 wm0_ub = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(0));
		u32 axi_mode = readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		u32 xbar_cfg1 = readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG1);

		dev_info(vfe->camss->dev,
			 "VFE IRQ#%d WM0: CFG=0x%x PING=0x%08x PONG=0x%08x SIZE=0x%x UB=0x%x AXI=0x%x XBAR=0x%x\n",
			 irq_count, wm0_cfg, wm0_ping, wm0_pong, wm0_size, wm0_ub, axi_mode, xbar_cfg1);
	}

	/* VFE31 reset acknowledge is in STATUS_1 bit 22, not STATUS_0 bit 31 */
	if (value1 & VFE_0_IRQ_STATUS_1_RESET_ACK)
		vfe->isr_ops.reset_ack(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_VIOLATION)
		vfe->res->hw_ops->violation_read(vfe);

	if (value1 & VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK)
		vfe->isr_ops.halt_ack(vfe);

	/* Handle CAMIF_ERROR - clear status to allow next frame */
	if (value1 & VFE_0_IRQ_MASK_1_CAMIF_ERROR) {
		u32 camif_status = readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS);
		u32 window_height = readl_relaxed(vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);
		u32 window_width = readl_relaxed(vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);
		/* WINDOW_HEIGHT_CFG: [29:16]=firstLine, [13:0]=lastLine */
		u32 expected_lines = (window_height & 0x3FFF) + 1;  /* lastLine + 1 */
		u32 expected_pixels = (window_width & 0x3FFF) + 1;  /* lastPixel + 1 */
		u32 received_lines = (camif_status >> 16) & 0x3FFF;
		u32 received_pixels = camif_status & 0x3FFF;
		static int camif_error_count;

		camif_error_count++;
		if (camif_error_count <= 3) {
			dev_warn(vfe->camss->dev,
				 "CAMIF_ERROR #%d: status=0x%08x (pixels=%d/%d lines=%d/%d)\n",
				 camif_error_count, camif_status,
				 received_pixels, expected_pixels,
				 received_lines, expected_lines);
		}

		/*
		 * CAMIF_ERROR typically fires when MIPI Frame End packet is missing.
		 * The MT9M113 sensor may not send FE packets, causing this error
		 * on every frame. If we received the expected number of lines,
		 * the frame data is valid - just missing the EOF signal.
		 *
		 * CRITICAL: Clear CAMIF_STATUS AND restart CAMIF for next frame.
		 * Write 0x5 = CLEAR_STATUS (bit 2) + START (bit 0) per webOS.
		 * Without restarting, CAMIF stays halted and no more frames come.
		 */
		writel_relaxed(VFE_0_CAMIF_CMD_CLEAR_CAMIF_STATUS | VFE_0_CAMIF_CMD_START,
			       vfe->base + VFE_0_CAMIF_CMD);
		wmb();

		/* If frame data is complete, trigger frame completion */
		if (received_lines >= expected_lines - 2 && expected_lines > 0) {
			/* Issue REG_UPDATE to latch shadow registers */
			writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);
			wmb();

			/*
			 * Manually trigger frame completion for active WMs.
			 * This simulates what would happen if EOF fired normally.
			 */
			vfe->isr_ops.wm_done(vfe, 0);

			/* Notify all lines of reg_update */
			for (i = 0; i < vfe->res->line_num; i++)
				vfe->isr_ops.reg_update(vfe, i);
		}
	}

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

/*
 * vfe31_enable - Enable VFE31 streaming (direct implementation, bypassing gen1)
 *
 * VFE31 predates the gen1 framework and has different register timing
 * requirements. This function implements the webOS driver's sequence directly:
 * 1. Configure AXI output mode (0x60 for raw WM0)
 * 2. Configure WM registers (ping/pong, image_size, addr_cfg, ub_cfg)
 * 3. Configure CAMIF
 * 4. Enable IRQs
 * 5. Start CAMIF
 */
static int vfe31_enable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
	u32 ping_addr, pong_addr;
	u16 width, height, bytesperline, wpl;
	u32 val, reg;
	unsigned long flags;
	int wm_idx;
	u8 wm;

	dev_info(vfe->camss->dev, "VFE31 enable: line_id=%d (direct, not gen1)\n",
		 line->id);

	/* Setup output (inline from gen1's vfe_get_output) */
	spin_lock_irqsave(&vfe->output_lock, flags);

	if (output->state > VFE_OUTPUT_RESERVED) {
		dev_err(vfe->camss->dev, "VFE31: Output already running\n");
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return -EBUSY;
	}
	output->state = VFE_OUTPUT_RESERVED;
	output->gen1.active_buf = 0;
	output->drop_update_idx = 0;

	/*
	 * WM configuration for PIX mode with DEMUX (ISP processing):
	 * - WM0: Y (luma) channel
	 * - WM1: CbCr (chroma) channel
	 *
	 * For raw/RDI mode (AXI=0x60), only WM0 is needed.
	 * For PIX mode with DEMUX (AXI=0x01), we need both WM0 and WM1
	 * because DEMUX separates Y and CbCr internally.
	 *
	 * Note: Even for "packed" UYVY format, the VFE31 DEMUX outputs
	 * Y and CbCr to separate WMs. The CbCr is interleaved (CbYCrY).
	 */
	if (vfe31_axi_output_mode == 0x01) {
		/* PIX mode: need both WM0 (Y) and WM1 (CbCr) */
		output->wm_num = 2;
		dev_info(vfe->camss->dev, "VFE31: PIX mode - using 2 WMs (Y+CbCr)\n");

		/*
		 * IMPORTANT: VFE31 PIX mode ALWAYS outputs semi-planar format
		 * (Y to WM0, CbCr to WM1) regardless of requested pixel format.
		 * If userspace requests packed UYVY/YUYV, the data will still
		 * be semi-planar NV16 in memory, causing wrong colors when
		 * interpreted as packed format.
		 *
		 * Warn users if they request packed formats with PIX mode.
		 */
		switch (pix->pixelformat) {
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_VYUY:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
			dev_warn(vfe->camss->dev,
				 "VFE31: WARNING - Packed format requested but PIX mode outputs NV16!\n");
			dev_warn(vfe->camss->dev,
				 "VFE31: Use V4L2_PIX_FMT_NV16 for correct colors, or use RAW mode (axi=0x60) for packed output.\n");
			break;
		default:
			break;
		}
	} else {
		/* Raw/RDI mode: single WM for packed data */
		output->wm_num = 1;
		dev_info(vfe->camss->dev, "VFE31: Raw mode - using 1 WM\n");
	}

	wm_idx = vfe_reserve_wm(vfe, line->id);
	if (wm_idx < 0) {
		dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM0\n");
		output->state = VFE_OUTPUT_OFF;
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return wm_idx;
	}
	output->wm_idx[0] = wm_idx;

	/* Reserve WM1 for PIX mode */
	if (output->wm_num == 2) {
		wm_idx = vfe_reserve_wm(vfe, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM1\n");
			vfe_release_wm(vfe, output->wm_idx[0]);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[1] = wm_idx;
		dev_info(vfe->camss->dev, "VFE31: Reserved WM0=%d, WM1=%d\n",
			 output->wm_idx[0], output->wm_idx[1]);
	}

	/* Get buffers from pending queue (inline from gen1's vfe_enable_output) */
	output->buf[0] = vfe_buf_get_pending(output);
	output->buf[1] = vfe_buf_get_pending(output);

	if (!output->buf[0] && output->buf[1]) {
		output->buf[0] = output->buf[1];
		output->buf[1] = NULL;
	}

	if (output->buf[0])
		output->state = VFE_OUTPUT_SINGLE;

	if (output->buf[1])
		output->state = VFE_OUTPUT_CONTINUOUS;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	wm = output->wm_idx[0];
	width = pix->width;
	height = pix->height;

	/*
	 * CRITICAL: For VFE31 PIX mode (UYVY -> NV16), the DMA burst must be
	 * based on the UYVY input stride (width * 2), NOT the NV16 output
	 * plane bytesperline. WebOS uses 1280 bytes for 640x480, which is
	 * 640 * 2 = UYVY input line size.
	 *
	 * The output plane_fmt[0].bytesperline would be 640 (Y plane width),
	 * but the DMA needs to know the full UYVY line width to properly
	 * demux Y and CbCr bytes.
	 */
	bytesperline = width * 2;  /* UYVY input stride, not output plane */

	/* Get buffer addresses */
	if (output->buf[0])
		ping_addr = output->buf[0]->addr[0];
	else
		ping_addr = 0;

	if (output->buf[1])
		pong_addr = output->buf[1]->addr[0];
	else
		pong_addr = ping_addr;

	if (!ping_addr) {
		dev_err(vfe->camss->dev, "VFE31: No buffers available!\n");
		return -EINVAL;
	}

	dev_info(vfe->camss->dev,
		 "VFE31: WM%d %ux%u stride=%u ping=0x%08x pong=0x%08x\n",
		 wm, width, height, bytesperline, ping_addr, pong_addr);

	/*
	 * Store addresses in pending_* for vfe31_start_camif_for_rdi() which
	 * will write them to hardware after CSIPHY is configured.
	 */
	vfe->pending_ping_addr = ping_addr;
	vfe->pending_pong_addr = pong_addr;

	/*
	 * Step 1: Configure BUS_CFG, AXI output mode and XBAR
	 * Use module parameter vfe31_axi_output_mode:
	 *   0x60  = Raw/RDI mode (CAMIF_TO_AXI bypassing ISP)
	 *   0x01  = PIX/Preview mode (OUTPUT_2 with XBAR routing)
	 *
	 * CRITICAL: BUS_CFG at 0x03C must be set to 0x02AAA771 per webOS dumps.
	 * This value enables the write paths (bits 4-6) required for DMA:
	 *   - Bit 4: encYWrPathEn
	 *   - Bit 5: encCbcrWrPathEn
	 *   - Bit 6: viewYWrPathEn
	 * Without this, DMA writes don't complete and ping_pong never toggles.
	 *
	 * For PIX mode, we also need XBAR CFG1 at 0x44 = 0x1a1b
	 * This routes Y→WM0 and CbCr→WM1 for semi-planar output.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 1 - BUS_CFG=0x%08x, AXI=0x%x\n",
		 VFE_0_BUS_CFG_WEBOS_VALUE, vfe31_axi_output_mode);
	writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
	writel_relaxed(vfe31_axi_output_mode,
		       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);

	/*
	 * For PIX mode (OUTPUT_1_AND_3), configure XBAR_CFG1 to route
	 * Y to WM0 and CbCr to WM1. We always use 0x1A1B which properly
	 * routes both planes. See XBAR_CFG1 documentation above.
	 *
	 * Note: PIX_MODE and VIDEO_MODE both use 0x1A1B. The video_output_enable
	 * flag controls WM4/WM5 configuration, not the XBAR value itself.
	 */
	if (vfe31_axi_output_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		dev_info(vfe->camss->dev,
			 "VFE31: PIX mode - XBAR CFG1=0x%x (module param)\n",
			 vfe31_xbar_cfg1);
		writel_relaxed(vfe31_xbar_cfg1, vfe->base + VFE_0_BUS_XBAR_CFG1);
	}

	/*
	 * Step 1b: Configure DEMUX, scale and crop modules
	 * These must be set up before WM registers for the ISP pipeline
	 * to process data correctly. DEMUX is essential for YUV data.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 1b - Configure demux/scale/crop\n");
	vfe31_set_demux_cfg(vfe, line);
	vfe31_set_scale_cfg(vfe, line);
	vfe31_set_crop_cfg(vfe, line);

	/*
	 * Step 2: Configure WM registers
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 2 - WM%d registers\n", wm);

	/* WR_PING_ADDR */
	writel_relaxed(ping_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));

	/* WR_PONG_ADDR */
	writel_relaxed(pong_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));

	/*
	 * WR_IMAGE_SIZE - VFE31 format (from webOS register dumps):
	 * webOS WM0: 0x00501DF2 = ((80) << 16) | ((479 << 4) | 2)
	 *
	 * Upper 16 bits: bytesperline / 16 (128-bit words per line)
	 * Lower 16 bits: ((height - 1) << 4) | 2
	 *
	 * This is DIFFERENT from VFE4.x which uses (height-1) | (wpl<<16)
	 */
	reg = ((bytesperline / 16) & 0xFFFF) << 16;
	reg |= ((height - 1) << 4) | 2;
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));

	/*
	 * WR_ADDR_CFG - webOS format: (lines << 16) | burst_words
	 * webOS WM0: 0x0000012F = (lines=0 << 16) | burst=303
	 * webOS WM1: 0x01C8012F = (lines=456 << 16) | burst=303
	 *
	 * For single-plane UYVY, only WM0 is used with lines=0.
	 * burst_words = words_per_line - 1 (webOS uses wpl-1)
	 *
	 * NOTE: wpl is in 32-bit words. bytesperline is already in bytes,
	 * so divide by 4 directly (don't use vfe_word_per_line which
	 * expects pixel width and multiplies by bytes-per-pixel again).
	 *
	 * IMPORTANT: webOS uses (wpl - 17), not (wpl - 1)!
	 * For 640x480 UYVY (1280 bytes/line = 320 words):
	 *   Our old: 320 - 1 = 319
	 *   webOS:   320 - 17 = 303 = 0x12F (matches register dump!)
	 * The 16-word (64-byte) difference may be DMA alignment overhead.
	 */
	wpl = bytesperline / 4;  /* 32-bit words per line */
	reg = (wpl - 17) & 0xFFFF;  /* burst = wpl - 17 (webOS formula) */
	dev_info(vfe->camss->dev, "VFE31: WM%d WR_ADDR_CFG=0x%04x (wpl=%d, burst=%d)\n",
		 wm, reg, wpl, reg);
	/* For single-plane formats, lines=0. Multi-plane would add (height << 16) */
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	/*
	 * WR_UB_CFG - VFE31 format (from webOS register dumps):
	 * webOS WM0: 0x002701DF = ((39) << 16) | 479
	 *
	 * Upper 16 bits: (wpl / 8) - 1, where wpl is 32-bit words per line
	 *   For 1280 bytes/line: wpl = 320, (320/8)-1 = 39 = 0x27
	 * Lower 16 bits: height - 1
	 *
	 * This is DIFFERENT from VFE4.x which uses (offset << 16) | depth
	 */
	wpl = bytesperline / 4;  /* 32-bit words per line */
	reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
	reg |= (height - 1) & 0xFFFF;
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));

	/*
	 * WR_CFG - enable only (BIT(0)).
	 * Note: VFE31 does NOT have frame_based mode in WR_CFG bit 1.
	 * webOS only writes 1 here, not 3. Hardware ignores bit 1.
	 */
	writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
	wmb();

	/*
	 * Step 2b: Configure WM1 for CbCr output (PIX mode only)
	 *
	 * In PIX mode with DEMUX enabled, the VFE separates Y and CbCr:
	 * - WM0: Y (luma) channel
	 * - WM1: CbCr (chroma) channel, interleaved
	 *
	 * webOS WM1 configuration (from register dump):
	 * - WR_ADDR_CFG: 0x01C8012F = (lines=456 << 16) | burst=303
	 * - WR_UB_CFG: 0x002701DF (same as WM0)
	 * - WR_IMAGE_SIZE: 0x00501DF2 (same as WM0)
	 *
	 * For buffer address: webOS uses separate Y and CbCr buffers.
	 * Since userspace provides a single UYVY buffer, we write CbCr
	 * to an offset within the same buffer (semi-planar layout).
	 */
	if (output->wm_num == 2) {
		u8 wm1 = output->wm_idx[1];
		u32 cbcr_offset = width * height;  /* Y plane size in bytes */
		u32 wm1_ping_addr = ping_addr + cbcr_offset;
		u32 wm1_pong_addr = pong_addr + cbcr_offset;

		dev_info(vfe->camss->dev,
			 "VFE31: Configuring WM%d (CbCr) offset=0x%x ping=0x%08x\n",
			 wm1, cbcr_offset, wm1_ping_addr);

		/* WM1 PING/PONG addresses (CbCr buffer after Y) */
		writel_relaxed(wm1_ping_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm1));
		writel_relaxed(wm1_pong_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm1));

		/* WM1 IMAGE_SIZE - same as WM0 */
		reg = ((bytesperline / 16) & 0xFFFF) << 16;
		reg |= ((height - 1) << 4) | 2;
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm1));

		/*
		 * WM1 ADDR_CFG - burst and line count for CbCr write master
		 *
		 * webOS uses (456 << 16) | 303 for 640x480 = (height-24) << 16 | burst
		 * The "lines" field is NOT a crop - it appears to control DMA line
		 * counting behavior for the CbCr plane. Without it, only ~half the
		 * UV data is written.
		 *
		 * Format: (lines << 16) | burst_words
		 * Where lines = height - 24 (webOS value, reason unknown but required)
		 */
		wpl = bytesperline / 4;
		reg = ((height - 24) << 16) | ((wpl - 17) & 0xFFFF);
		dev_info(vfe->camss->dev,
			 "VFE31: WM%d WR_ADDR_CFG=0x%08x (lines=%d, burst=%d)\n",
			 wm1, reg, height - 24, (wpl - 17) & 0xFFFF);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm1));

		/* WM1 UB_CFG - same as WM0 */
		reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
		reg |= (height - 1) & 0xFFFF;
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm1));

		/* Enable WM1 */
		writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm1));
		wmb();
	}

	/* Reload WMs to apply new configuration */
	dev_info(vfe->camss->dev, "VFE31: Reloading WM%d (BUS_CMD)\n", wm);
	reg = VFE_0_BUS_CMD_Mx_RLD_CMD(wm);
	if (output->wm_num == 2)
		reg |= VFE_0_BUS_CMD_Mx_RLD_CMD(output->wm_idx[1]);
	writel_relaxed(reg, vfe->base + VFE_0_BUS_CMD);
	wmb();

	/*
	 * VFE31 testgen mode: Skip CSIPHY deferral and start testgen directly.
	 * The test generator produces data internally, no external camera needed.
	 */
	if (vfe31_use_testgen) {
		dev_info(vfe->camss->dev,
			 "VFE31: Testgen mode - starting directly (WM%d, line %d)\n",
			 wm, line->id);

		/* Configure and start the test generator */
		vfe31_configure_testgen(vfe, true, width, height);

		/*
		 * Note: Do NOT change output->state here. It was already set
		 * correctly above to VFE_OUTPUT_SINGLE or VFE_OUTPUT_CONTINUOUS
		 * based on buffer availability. Setting VFE_OUTPUT_ON would break
		 * the gen1 state machine in vfe_buf_update_wm_on_next().
		 */
		output->sequence = 0;
		output->gen1.active_buf = 0;
		vfe->camif_pending = false;

		mutex_lock(&vfe->stream_lock);
		vfe->stream_count++;
		mutex_unlock(&vfe->stream_lock);

		dev_info(vfe->camss->dev,
			 "VFE31: Testgen started, stream_count=%d\n",
			 vfe->stream_count);
		return 0;
	}

	/*
	 * MSM8660 WORKAROUND: Defer CAMIF configuration until CSIPHY is ready.
	 *
	 * On MSM8660, VFE CAMIF registers must NOT be written until after
	 * CSIPHY is configured and lanes are enabled. Writing to CAMIF
	 * registers before CSIPHY is ready causes data path issues.
	 *
	 * Set camif_pending flag here. The actual CAMIF configuration
	 * (steps 3-6) will be done by vfe_enable_pending_camif() which
	 * is called from CSIPHY set_stream after lanes are enabled.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: Deferring CAMIF config until CSIPHY ready (WM%d, line %d)\n",
		 wm, line->id);

	vfe->camif_pending = true;
	vfe->camif_pending_wm = wm;
	vfe->camif_pending_line_id = line->id;

	/*
	 * DO NOT reset output->state here! It was correctly set based on
	 * buffer availability at lines 947-951:
	 *   - SINGLE if buf[0] found
	 *   - CONTINUOUS if buf[0] and buf[1] found
	 *
	 * Previously this line incorrectly overwrote state to IDLE, causing
	 * "Next buf in wrong state! 4" errors on first frame completion.
	 */
	output->sequence = 0;
	output->gen1.active_buf = 0;

	dev_info(vfe->camss->dev,
		 "VFE31: Output state=%d (2=SINGLE, 3=CONTINUOUS) buf[0]=%px buf[1]=%px\n",
		 output->state, output->buf[0], output->buf[1]);

	dev_info(vfe->camss->dev,
		 "VFE31: WM configured, waiting for CSIPHY before CAMIF start\n");
	dev_info(vfe->camss->dev,
		 "VFE31: AXI_OUT_MODE=0x%08x WM0_CFG=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG),
		 readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(0)));

	/*
	 * Increment stream_count here to balance vfe_gen1_disable() decrement.
	 * This must be done in vfe31_enable(), not in the deferred CAMIF callback.
	 */
	mutex_lock(&vfe->stream_lock);
	vfe->stream_count++;
	mutex_unlock(&vfe->stream_lock);

	return 0;
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
	 *
	 * IMPORTANT: VFE31 IRQ_MASK_0/1 registers are WRITE-ONLY!
	 * Reading them hangs the bus. We use shadow registers to track
	 * the current mask values.
	 */
	vfe->irq_mask0_shadow = 0;
	/* IRQ_MASK_1: webOS uses only RESET_ACK (0x00400000) */
	vfe->irq_mask1_shadow = VFE_0_IRQ_MASK_1_RESET_ACK;
	vfe->irq_comp_mask_shadow = 0;  /* Clear composite mask shadow */

	dev_info(vfe->camss->dev, "VFE31 enable_irq_common: mask0=0x%x mask1=0x%x\n",
		 vfe->irq_mask0_shadow, vfe->irq_mask1_shadow);

	writel_relaxed(vfe->irq_mask0_shadow, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(vfe->irq_mask1_shadow, vfe->base + VFE_0_IRQ_MASK_1);
}

static void vfe31_set_demux_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 val, even_cfg, odd_cfg;

	/* Use webOS MODULE_CFG value (0x01c00c0c) - not just DEMUX bit */
	writel_relaxed(0x01c00c0c,
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
		/*
		 * webOS uses 0xC9CA for UYVY preview mode.
		 * This seems to be (0xC9 << 8) | 0xCA format.
		 */
		even_cfg = 0xc9;
		odd_cfg = 0xca;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		even_cfg = 0xc9;
		odd_cfg = 0xac;
		break;
	}

	/*
	 * Apply UV swap if requested via module parameter.
	 * This exchanges Cb and Cr channel routing in the DEMUX output.
	 * For UYVY: normal=0xCA, swapped=0xAC
	 */
	if (vfe31_swap_uv) {
		if (odd_cfg == 0xca)
			odd_cfg = 0xac;
		else if (odd_cfg == 0xac)
			odd_cfg = 0xca;
		dev_info(vfe->camss->dev,
			 "VFE31: UV swap enabled, odd_cfg=0x%02x\n", odd_cfg);
	}

	/*
	 * Write even/odd config to DEMUX_EVEN_CFG (0x290), NOT DEMUX_CFG!
	 * webOS register dump shows:
	 *   DEMUX_CFG (0x284) = 0x03 (period)
	 *   DEMUX_EVEN (0x290) = 0xC9CA (even << 8 | odd for UYVY)
	 */
	writel_relaxed((even_cfg << 8) | odd_cfg, vfe->base + VFE_0_DEMUX_EVEN_CFG);
}

static void vfe31_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	/* VFE31 scale configuration */
	writel_relaxed(0, vfe->base + VFE_0_SCALE_ENC_Y_CFG);
	writel_relaxed(0, vfe->base + VFE_0_SCALE_ENC_CBCR_CFG);
}

static void vfe31_set_crop_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	/*
	 * VFE31 does NOT have dedicated CROP_ENC registers like VFE4x.
	 *
	 * Per Mako kernel analysis:
	 * - Main Scaler at 0x368-0x383 (28 bytes) handles output sizing
	 * - FOV (Field of View) at 0x360 handles input region selection
	 * - 0x384 is White Balance, NOT crop (was incorrectly used before)
	 * - 0x388 is Color Correction, NOT realign buffer
	 *
	 * The previous code was writing crop values to the scaler and WB
	 * registers, which corrupted ISP processing. Output dimensions
	 * are controlled by the scaler and AXI write master configuration.
	 *
	 * TODO: If hardware cropping is needed, implement proper VFE31
	 * FOV configuration or use scaler for output size control.
	 */
	dev_dbg(vfe->camss->dev,
		"VFE31 crop: no-op (VFE31 uses FOV/scaler, not CROP_ENC regs)\n");
}

static void vfe31_set_clamp_cfg(struct vfe_device *vfe)
{
	writel_relaxed(0x00ffffff, vfe->base + VFE_0_CLAMP_ENC_MAX_CFG);
	writel_relaxed(0x0, vfe->base + VFE_0_CLAMP_ENC_MIN_CFG);
}

static void vfe31_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable)
{
	/*
	 * VFE31 CGC_OVERRIDE is already configured during global_reset().
	 * Writing to it again after other registers are configured causes hangs.
	 *
	 * Unlike VFE41 which has per-WM CGC control, VFE31 has a single global
	 * CGC_OVERRIDE that enables all internal clocks. We set it to 0xFFFFF
	 * during reset and leave it alone thereafter.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: set_cgc_override wm=%d enable=%d (NO-OP, already set in reset)\n",
		 wm, enable);
}

static void vfe31_set_camif_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 val;
	u32 width = line->fmt[MSM_VFE_PAD_SINK].width;
	u32 height = line->fmt[MSM_VFE_PAD_SINK].height;
	u32 bytes_per_line = width * 2;	/* YUV422: 2 bytes/pixel */

	dev_info(vfe->camss->dev,
		 "VFE31 set_camif_cfg: ENTRY width=%d height=%d bytes_per_line=%d\n",
		 width, height, bytes_per_line);

	/* Configure pixel pattern in CORE_CFG + bit 6 (webOS uses 0x46 for UYVY) */
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
	/* Add bit 6 - webOS always sets this (0x46 instead of 0x06) */
	val |= VFE_0_CORE_CFG_INPUT_MUX_ENABLE;
	writel_relaxed(val, vfe->base + VFE_0_CORE_CFG);

	/*
	 * Configure CAMIF registers using correct VFE31 layout:
	 *
	 * IMPORTANT: VFE31 does NOT have camif2vfeEnable/camif2busEnable bits!
	 * Those exist in VFE8x at offset 0x114, not in VFE31.
	 *
	 * VFE31 register layout:
	 * 0x1E4: EFS_CFG - Embedded Frame Sync codes (0 for APS mode with MIPI)
	 * 0x1E8: FRAME_CFG - pixelsPerLine | linesPerFrame<<16
	 * 0x1EC: WINDOW_WIDTH - lastPixel | firstPixel<<16
	 * 0x1F0: WINDOW_HEIGHT - lastLine | firstLine<<16
	 *
	 * Data routing on VFE31 is controlled solely via AXI output mode at 0x040:
	 *   - 0x200: OUTPUT_2 (PIX mode, VFE ISP processing)
	 *   - 0x60:  CAMIF_TO_AXI_VIA_OUTPUT_2 (RDI mode, raw bypass)
	 *
	 * EFS_CFG at 0x1E4: webOS uses 0x40 (bit 6 set)
	 * This enables some timing/sync feature needed for proper operation.
	 */
	writel_relaxed(0x40, vfe->base + VFE_0_CAMIF_EFS_CFG);
	dev_info(vfe->camss->dev, "VFE31: EFS_CFG=0x40 (webOS value)\n");

	/*
	 * FRAME_CFG at 0x1E8: WebOS does NOT set this register (leaves at 0).
	 * Do NOT write FRAME_CFG.
	 */

	/*
	 * VFE31 CAMIF register semantics (based on WebOS register dump):
	 *
	 * WINDOW_WIDTH_CFG (0x1EC): Frame dimensions
	 *   [29:16] = height (lines per frame)
	 *   [13:0]  = width (pixels/bytes per line)
	 *   WebOS example: 0x01E00500 = (480 << 16) | 1280
	 *
	 * WINDOW_HEIGHT_CFG (0x1F0): Last pixel index
	 *   [13:0]  = lastPixel = width - 1
	 *   WebOS example: 0x000004FF = 1279 = 1280 - 1
	 *
	 * SUBSAMPLE_CFG_0 (0x1F4): Last line index
	 *   [13:0]  = lastLine = height - 1
	 *   WebOS example: 0x000001DF = 479 = 480 - 1
	 */
	val = (height << 16) | (bytes_per_line & 0x3FFF);
	dev_info(vfe->camss->dev,
		 "VFE31: WINDOW_WIDTH_CFG=0x%08x (lines=%u, pixels=%u)\n",
		 val, height, bytes_per_line);
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);

	val = (bytes_per_line - 1) & 0x3FFF;
	dev_info(vfe->camss->dev,
		 "VFE31: WINDOW_HEIGHT_CFG=0x%08x (lastPixel=%u)\n",
		 val, bytes_per_line - 1);
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);

	/*
	 * SUBSAMPLE_CFG_0 at 0x1F4: Last line index (height - 1)
	 */
	writel_relaxed(height - 1, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);

	/*
	 * SUBSAMPLE_CFG_1 at 0x1F8: webOS uses 0xFFFFFFFF (no frame skip)
	 */
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_1);

	dev_dbg(vfe->camss->dev,
		"VFE31 set_camif_cfg: core_cfg=0x%08x frame=0x%08x\n",
		readl_relaxed(vfe->base + VFE_0_CORE_CFG),
		readl_relaxed(vfe->base + VFE_0_CAMIF_FRAME_CFG));
}

static void vfe31_set_camif_cmd(struct vfe_device *vfe, u8 enable)
{
	u32 cmd;

	/*
	 * CAMIF command values from webOS msm_vfe31.h:
	 * - START = 0x5 (bits 0 and 2: enable + clear status)
	 * - STOP_AT_FRAME_BOUNDARY = 0x0
	 * - STOP_IMMEDIATELY = 0x2
	 */
	if (enable)
		cmd = VFE_0_CAMIF_CMD_START;
	else
		cmd = VFE_0_CAMIF_CMD_STOP_AT_FRAME_BOUNDARY;

	writel_relaxed(cmd, vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	/* Debug: dump CAMIF status registers (skip CAMIF_CFG - not used for CSI) */
	dev_info(vfe->camss->dev,
		 "VFE31 CAMIF: cmd=%s status=0x%08x core_cfg=0x%08x frame_cfg=0x%08x\n",
		 enable ? "enable" : "disable",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS),
		 readl_relaxed(vfe->base + VFE_0_CORE_CFG),
		 readl_relaxed(vfe->base + VFE_0_CAMIF_FRAME_CFG));
}

static void vfe31_set_module_cfg(struct vfe_device *vfe, u8 enable)
{
	/*
	 * Use exact webOS MODULE_CFG value: 0x01c00c0c
	 * - bit 2: DEMUX
	 * - bit 3: CHROMA_UPSAMPLE
	 * - bits 10-11: Unknown but required by webOS
	 * - bit 24: Unknown but required by webOS
	 */
	u32 val = 0x01c00c0c;

	dev_info(vfe->camss->dev, "VFE31 set_module_cfg: enable=%d val=0x%x (webOS)\n",
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
	/*
	 * VFE31 does NOT have per-WM XBAR registers like VFE41/47/48.
	 * See "VFE31 CROSSBAR (XBAR)" documentation block above.
	 *
	 * VFE31 Y/CbCr routing is controlled entirely by:
	 * 1. XBAR_CFG0 at 0x040 - output mode selection
	 * 2. XBAR_CFG1 at 0x044 - Y/CbCr routing to write masters
	 * 3. DEMUX module - internally separates UYVY into Y and CbCr
	 *
	 * XBAR_CFG1 is configured in vfe31_enable(), not here.
	 * This function is intentionally empty for VFE31.
	 */
}

/*
 * vfe31_configure_video_wm - Configure video write masters (WM4/WM5)
 * @vfe: VFE device
 * @y_addr: Physical address for Y (luma) video buffer
 * @cbcr_addr: Physical address for CbCr (chroma) video buffer
 * @width: Video frame width in pixels
 * @height: Video frame height in lines
 * @stride: Bytes per line (bytesperline)
 *
 * This function configures WM4 and WM5 for video recording output.
 * Used when vfe31_video_output_enable=1 for simultaneous preview+video.
 *
 * Video output uses COMPOSITE_DONE_2 (IRQ bit 23) for frame completion.
 * This is separate from preview (COMPOSITE_DONE_0, IRQ bit 21).
 */
static void vfe31_configure_video_wm(struct vfe_device *vfe,
				     u32 y_addr, u32 cbcr_addr,
				     u16 width, u16 height, u16 stride)
{
	u16 wpl;  /* words per line */
	u32 reg;

	dev_info(vfe->camss->dev,
		 "VFE31: Configuring video WM4/5: %ux%u stride=%u Y=0x%08x CbCr=0x%08x\n",
		 width, height, stride, y_addr, cbcr_addr);

	/* Words per line (32-bit words) */
	wpl = (stride + 3) / 4;

	/*
	 * WM4 - Video Y channel
	 * Same register layout as WM0 but for video output path
	 * Using VFE31/webOS format for all WM registers
	 */
	writel_relaxed(y_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(VFE31_VIDEO_WM_Y));
	writel_relaxed(y_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(VFE31_VIDEO_WM_Y));

	/*
	 * WR_IMAGE_SIZE - VFE31 format:
	 * Upper 16 bits: stride / 16 (128-bit words per line)
	 * Lower 16 bits: ((height - 1) << 4) | 2
	 */
	reg = ((stride / 16) & 0xFFFF) << 16;
	reg |= ((height - 1) << 4) | 2;
	writel_relaxed(reg,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(VFE31_VIDEO_WM_Y));

	/*
	 * WR_ADDR_CFG - VFE31/webOS format:
	 * burst_words = words_per_line - 17 (webOS formula)
	 * For 1280 bytes/line: wpl=320, burst=320-17=303=0x12F
	 */
	reg = (wpl - 17) & 0xFFFF;
	writel_relaxed(reg,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_Y));

	/*
	 * WR_UB_CFG - VFE31 format:
	 * Upper 16 bits: (wpl / 8) - 1
	 * Lower 16 bits: height - 1
	 */
	reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
	reg |= (height - 1) & 0xFFFF;
	writel_relaxed(reg,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(VFE31_VIDEO_WM_Y));

	/* Enable WM4 (VFE31 has no frame_based bit, just enable) */
	writel_relaxed(BIT(0),
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));

	/*
	 * WM5 - Video CbCr channel (for semi-planar formats like NV12/NV16)
	 * For packed formats (UYVY/YUYV), this WM is not used.
	 */
	if (cbcr_addr) {
		writel_relaxed(cbcr_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(VFE31_VIDEO_WM_CBCR));
		writel_relaxed(cbcr_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(VFE31_VIDEO_WM_CBCR));

		/* Same size/buffer config as Y for 4:2:2 video */
		reg = ((stride / 16) & 0xFFFF) << 16;
		reg |= ((height - 1) << 4) | 2;
		writel_relaxed(reg,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(VFE31_VIDEO_WM_CBCR));

		/* CbCr ADDR_CFG - use (height-24) for lines like preview WM1 */
		reg = ((height - 24) << 16) | ((wpl - 17) & 0xFFFF);
		writel_relaxed(reg,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_CBCR));

		reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
		reg |= (height - 1) & 0xFFFF;
		writel_relaxed(reg,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(VFE31_VIDEO_WM_CBCR));

		/* Enable WM5 (VFE31 has no frame_based bit, just enable) */
		writel_relaxed(BIT(0),
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_CBCR));
	}

	wmb();
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
	 * VFE31 RDI/CID configuration - architecture note:
	 *
	 * Unlike VFE41+ which has dedicated RDI_CFG registers (at 0x2E8+)
	 * for selecting CSI streams and configuring per-RDI data types,
	 * VFE31 does NOT have separate RDI configuration registers.
	 *
	 * On MSM8660/APQ8060:
	 * - CSIPHY includes the CSI decoder (no separate CSID hardware)
	 * - CAMIF is the single input interface for all data
	 * - Data routing is controlled via AXI output mode only
	 *   (0x60 for raw snapshot via WM0)
	 * - VFE31 does NOT have camif2vfeEnable/camif2busEnable bits
	 *
	 * The CID/data type configuration happens in CSIPHY at the protocol
	 * level, not in VFE. VFE simply receives whatever CSIPHY sends.
	 */
	dev_dbg(vfe->camss->dev,
		"VFE31: set_rdi_cid RDI%d CID=%d (no-op, handled by CSIPHY on 8x60)\n",
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
	/*
	 * VFE31 RDI/raw capture setup - architecture note:
	 *
	 * On VFE41+, this function configures RDI_CFG registers (at 0x2E8+)
	 * with RDI_STREAM_SEL, RDI_EN, MIPI_EN bits to connect a write master
	 * to a specific RDI line.
	 *
	 * VFE31 does NOT have these registers. Instead, VFE31 uses:
	 * - AXI output mode 0x60 for raw snapshot capture via WM0
	 * - BUS_CFG register for raw write path configuration
	 * - CAMIF frame/window registers for dimensions
	 * Note: VFE31 has NO camif2vfeEnable/camif2busEnable bits
	 *
	 * The gen1 framework calls this function BEFORE configuring WM
	 * registers (ub_cfg, ping/pong addresses). VFE31 has strict
	 * ordering requirements - writing to CAMIF before WM is ready
	 * causes bus hangs.
	 *
	 * Solution: Set camif_pending flag here. The actual CAMIF
	 * configuration happens in vfe31_start_camif_for_rdi() which
	 * is called from vfe31_wm_enable() after all WM setup is done.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: bus_connect_wm_to_rdi WM%d to RDI%d - deferring to wm_enable\n",
		 wm, id);

	/* Store the RDI line mapping for later use */
	vfe->wm_output_map[wm] = id;

	/* Set flag to trigger CAMIF config when wm_enable is called */
	vfe->camif_pending = true;
}

static void vfe31_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
					     enum vfe_line_id id)
{
	dev_info(vfe->camss->dev, "VFE31: disconnect WM%d from RDI%d\n", wm, id);

	/* Step 1: Stop CAMIF at frame boundary */
	writel_relaxed(VFE_0_CAMIF_CMD_STOP_AT_FRAME_BOUNDARY,
		       vfe->base + VFE_0_CAMIF_CMD);

	/*
	 * Step 2: Clear AXI output mode to disable data path.
	 * Setting to 0 stops data flow to memory.
	 */
	writel_relaxed(0, vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);

	vfe->camif_pending = false;

	wmb();
}

static void vfe31_wm_set_subsample(struct vfe_device *vfe, u8 wm)
{
	/*
	 * VFE31: Skip subsample pattern write.
	 *
	 * The IRQ_SUBSAMPLE_PATTERN register at 0x060 + 0x18*wm overlaps with
	 * IMAGE_SIZE in our register map. Writing to it before WM is fully
	 * configured may cause hangs. Since we don't use subsampling,
	 * skip this entirely.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: wm_set_subsample wm=%d (NO-OP, not needed)\n", wm);
}

static void vfe31_bus_enable_wr_if(struct vfe_device *vfe, u8 enable)
{
	/*
	 * VFE31 bus write interface enable - NO-OP for VFE31!
	 *
	 * IMPORTANT: VFE31 does NOT have the same BUS_CFG register as VFE8x.
	 * The 0x03C offset is part of the 188-byte AXI output config block
	 * (starting at 0x038), and the webOS driver leaves it at 0.
	 *
	 * In VFE31, data routing is controlled ONLY by:
	 * 1. AXI output mode at 0x040 (0x60 for raw snapshot, 0x200 for preview)
	 * 2. CAMIF_CFG at 0x1E4 (camif2busEnable for raw capture)
	 *
	 * Writing VFE8x-style BUS_CFG values to 0x03C corrupts the AXI config
	 * and can cause bus errors or no data output.
	 *
	 * The AXI output mode is set in vfe31_wm_enable() when streaming starts.
	 */
	dev_dbg(vfe->camss->dev,
		"VFE31: bus_enable_wr_if(%d) - no-op (uses AXI output mode)\n",
		enable);
}

static void vfe31_bus_reload_wm(struct vfe_device *vfe, u8 wm)
{
	dev_info(vfe->camss->dev, "VFE31: bus_reload_wm wm=%d\n", wm);
	wmb();
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm),
		       vfe->base + VFE_0_BUS_CMD);
	wmb();
	dev_info(vfe->camss->dev, "VFE31: bus_reload_wm done\n");
}

static void vfe31_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable)
{
	/*
	 * VFE31: Skip WR_CFG write here - it causes hangs.
	 *
	 * The WR_CFG register at 0x04C controls both frame_based mode (bit 1)
	 * and WM enable (bit 0). Writing to it before CAMIF is configured
	 * causes system hangs.
	 *
	 * The wm_enable() function handles the actual WR_CFG configuration
	 * and is called after CAMIF setup (via camif_pending flag).
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: wm_frame_based wm=%d enable=%d (deferred to wm_enable)\n",
		 wm, enable);
}

/*
 * Helper to get WM sizes from pixel format
 */
static void vfe31_get_wm_sizes(struct v4l2_pix_format_mplane *pix, u8 plane,
			       u16 *width, u16 *height, u16 *bytesperline)
{
	*width = pix->width;
	*height = pix->height;
	*bytesperline = pix->plane_fmt[0].bytesperline;

	/* For NV12/NV21, chroma plane is half height */
	if (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
	    pix->pixelformat == V4L2_PIX_FMT_NV21)
		if (plane == 1)
			*height /= 2;
}

static void vfe31_wm_line_based(struct vfe_device *vfe, u32 wm,
				struct v4l2_pix_format_mplane *pix,
				u8 plane, u32 enable)
{
	/*
	 * VFE31: Skip IMAGE_SIZE and ADDR_CFG writes here - they cause hangs.
	 *
	 * Writing to WR_IMAGE_SIZE (0x060) and WR_ADDR_CFG (0x058) before
	 * CAMIF is configured causes system hangs.
	 *
	 * These registers are configured in vfe31_start_camif_for_rdi()
	 * which runs after CAMIF setup.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: wm_line_based wm=%d enable=%d (deferred to CAMIF start)\n",
		 wm, enable);
}

/*
 * vfe31_start_camif_for_rdi - Configure and start CAMIF after WM is ready
 *
 * This is called from wm_enable() when camif_pending is set. All WM
 * configuration must be complete before calling this.
 */
static void vfe31_start_camif_for_rdi(struct vfe_device *vfe, u8 wm)
{
	enum vfe_line_id line_id = vfe->wm_output_map[wm];
	struct vfe_line *line;
	u32 val;

	if (line_id == VFE_LINE_NONE || line_id >= vfe->res->line_num) {
		dev_err(vfe->camss->dev, "VFE31: Invalid line_id %d for WM%d\n",
			line_id, wm);
		return;
	}

	line = &vfe->line[line_id];

	dev_info(vfe->camss->dev,
		 "VFE31: Starting CAMIF for WM%d RDI%d (fmt %ux%u code=0x%x)\n",
		 wm, line_id, line->fmt[MSM_VFE_PAD_SINK].width,
		 line->fmt[MSM_VFE_PAD_SINK].height,
		 line->fmt[MSM_VFE_PAD_SINK].code);

	/*
	 * VFE31 raw capture initialization - matching webOS sequence:
	 * 1. Configure AXI output mode (0x60 for raw WM0)
	 * 2. Configure WM registers (ping/pong, image_size, etc.)
	 * 3. Configure CAMIF frame/window dimensions
	 * 4. Configure pixel pattern in CORE_CFG
	 * 5. Enable IRQs and start CAMIF
	 *
	 * NOTE: VFE31 does NOT have camif2vfeEnable/camif2busEnable bits!
	 * Data routing is controlled by AXI output mode only.
	 *
	 * Critical: AXI mode and WM addresses must be set BEFORE CAMIF starts.
	 */

	/*
	 * Step 1: Configure AXI output mode, BUS_CFG, and XBAR
	 *
	 * Two modes are supported:
	 *   0x01 (PIX): Data goes through CAMIF → DEMUX → XBAR → WM
	 *               Requires XBAR_CFG1 to route Y/CbCr to write masters
	 *               Used by webOS for all preview/video/photo capture
	 *
	 *   0x60 (RDI): Raw bypass mode, CAMIF → directly to WM0
	 *               No XBAR configuration needed (data bypasses XBAR)
	 *               For RAW8/RAW10 sensor output
	 *
	 * BUS_CFG at 0x03C must be set to enable the write paths.
	 * webOS uses 0x02AAA771 which enables view/enc Y/CbCr paths.
	 */
	if (vfe31_axi_output_mode == VFE_0_BUS_AXI_OUT_MODE_RAW_WM0) {
		/* RDI mode (0x60): Raw bypass, no XBAR needed */
		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - RDI mode: BUS_CFG=0x%08x, AXI=0x60 (raw bypass)\n",
			 VFE_0_BUS_CFG_WEBOS_VALUE);
		writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(VFE_0_BUS_AXI_OUT_MODE_RAW_WM0,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		/* No XBAR configuration for RDI mode */
	} else {
		/* PIX mode (0x01): Use XBAR to route DEMUX output to WMs */
		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - PIX mode: BUS_CFG=0x%08x, AXI=0x01, XBAR=0x%04x\n",
			 VFE_0_BUS_CFG_WEBOS_VALUE, vfe31_xbar_cfg1);
		writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(VFE_0_BUS_XBAR_CFG0_PIX_MODE,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		writel_relaxed(vfe31_xbar_cfg1, vfe->base + VFE_0_BUS_XBAR_CFG1);
	}
	wmb();

	/* Step 2: Configure WM registers (must be BEFORE CAMIF start) */
	{
		struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
		u16 width = pix->width;
		u16 height = pix->height;
		/*
		 * CRITICAL: Use UYVY input stride (width * 2), not output plane
		 * bytesperline. See comment in vfe31_enable() for details.
		 */
		u16 bytesperline = width * 2;  /* UYVY input stride */
		u16 wpl;
		u32 reg;

		dev_info(vfe->camss->dev, "VFE31: Step 2 - WM registers\n");

		/* WR_PING_ADDR */
		dev_info(vfe->camss->dev,
			 "VFE31: WM%d PING_ADDR=0x%08x\n", wm, vfe->pending_ping_addr);
		writel_relaxed(vfe->pending_ping_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));

		/* WR_PONG_ADDR */
		dev_info(vfe->camss->dev,
			 "VFE31: WM%d PONG_ADDR=0x%08x\n", wm, vfe->pending_pong_addr);
		writel_relaxed(vfe->pending_pong_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));

		/*
		 * WR_IMAGE_SIZE - VFE31 format (from webOS register dumps):
		 * webOS WM0: 0x00501DF2 = ((80) << 16) | ((479 << 4) | 2)
		 *
		 * Upper 16 bits: bytesperline / 16 (128-bit words per line)
		 * Lower 16 bits: ((height - 1) << 4) | 2
		 */
		reg = ((bytesperline / 16) & 0xFFFF) << 16;
		reg |= ((height - 1) << 4) | 2;

		dev_info(vfe->camss->dev,
			 "VFE31: WM%d IMAGE_SIZE bpl=%d height=%d reg=0x%x (webOS format)\n",
			 wm, bytesperline, height, reg);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));

		/*
		 * WR_ADDR_CFG - webOS format: (lines << 16) | burst_words
		 * webOS WM0: 0x0000012F = burst=303, lines=0
		 * webOS WM1: 0x01C8012F = burst=303, lines=456
		 *
		 * IMPORTANT: webOS uses (wpl - 17), not (wpl - 1)!
		 * For 1280 bytes/line: wpl=320, burst=320-17=303=0x12F
		 * NOTE: wpl is in 32-bit words. bytesperline is already in bytes.
		 */
		wpl = bytesperline / 4;  /* 32-bit words per line */
		reg = (wpl - 17) & 0xFFFF;  /* burst = wpl - 17 (webOS formula) */
		/* For single-plane formats, lines=0. Multi-plane would add (height << 16) */

		dev_info(vfe->camss->dev,
			 "VFE31: WM%d ADDR_CFG stride=%d wpl=%d burst=%d reg=0x%x\n",
			 wm, bytesperline, wpl, reg, reg);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

		/*
		 * WR_UB_CFG - VFE31 format (from webOS register dumps):
		 * webOS WM0: 0x002701DF = ((39) << 16) | 479
		 *
		 * Upper 16 bits: (wpl / 8) - 1, where wpl is 32-bit words per line
		 *   For 1280 bytes/line: wpl = 320, (320/8)-1 = 39 = 0x27
		 * Lower 16 bits: height - 1
		 */
		wpl = bytesperline / 4;  /* 32-bit words per line */
		reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
		reg |= (height - 1) & 0xFFFF;
		dev_info(vfe->camss->dev,
			 "VFE31: WM%d UB_CFG wpl=%d height=%d reg=0x%x (webOS format)\n",
			 wm, wpl, height, reg);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
		wmb();

		/*
		 * Step 2b: Configure WM1 for CbCr output (NV16 semi-planar)
		 *
		 * In PIX mode with DEMUX enabled, the VFE separates Y and CbCr:
		 * - WM0: Y (luma) channel
		 * - WM1: CbCr (chroma) channel, interleaved
		 *
		 * CbCr buffer starts after Y plane in the same buffer.
		 */
		if (line->output.wm_num == 2) {
			u8 wm1 = line->output.wm_idx[1];
			u32 cbcr_offset = width * height;  /* Y plane size in bytes */
			u32 wm1_ping = vfe->pending_ping_addr + cbcr_offset;
			u32 wm1_pong = vfe->pending_pong_addr + cbcr_offset;

			dev_info(vfe->camss->dev,
				 "VFE31: WM%d (CbCr) offset=0x%x PING=0x%08x PONG=0x%08x\n",
				 wm1, cbcr_offset, wm1_ping, wm1_pong);

			/* WM1 PING/PONG addresses */
			writel_relaxed(wm1_ping,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm1));
			writel_relaxed(wm1_pong,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm1));

			/* WM1 IMAGE_SIZE - same as WM0 */
			reg = ((bytesperline / 16) & 0xFFFF) << 16;
			reg |= ((height - 1) << 4) | 2;
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm1));

			/*
			 * WM1 ADDR_CFG - lines and burst for CbCr
			 * Use (height - 24) << 16 | burst (webOS value required)
			 */
			wpl = bytesperline / 4;
			reg = ((height - 24) << 16) | ((wpl - 17) & 0xFFFF);
			dev_info(vfe->camss->dev,
				 "VFE31: WM%d ADDR_CFG=0x%08x (lines=%d, burst=%d)\n",
				 wm1, reg, height - 24, (wpl - 17) & 0xFFFF);
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm1));

			/* WM1 UB_CFG - same as WM0 */
			reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
			reg |= (height - 1) & 0xFFFF;
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm1));
			wmb();
		}
	}

	/*
	 * Step 3: Configure CAMIF registers
	 *
	 * VFE31 CAMIF register layout (from webOS vfe_camifcfg structure):
	 * 0x1E4: EFS_CFG - Embedded Frame Sync codes (0 for APS mode)
	 * 0x1E8: FRAME_CFG - pixelsPerLine[13:0] | linesPerFrame[29:16]
	 * 0x1EC: WINDOW_WIDTH_CFG - lastPixel[13:0] | firstPixel[29:16]
	 * 0x1F0: WINDOW_HEIGHT_CFG - lastLine[13:0] | firstLine[29:16]
	 * 0x1F4: SUBSAMPLE_CFG_0 - pixelSkip[15:0] | lineSkip[31:16]
	 * 0x1F8: SUBSAMPLE_CFG_1 - frame subsample config
	 *
	 * NOTE: VFE31 does NOT have camif2vfeEnable/camif2busEnable bits like VFE8x!
	 * Data routing is controlled via AXI_OUT_MODE (0x040) only.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 3 - CAMIF configuration\n");

	/*
	 * EFS_CFG at 0x1E4: webOS uses 0x40 (bit 6 set)
	 * This enables some timing/sync feature needed for proper operation.
	 */
	dev_info(vfe->camss->dev, "VFE31: EFS_CFG (0x1E4) = 0x40 (webOS value)\n");
	writel_relaxed(0x40, vfe->base + VFE_0_CAMIF_EFS_CFG);

	/*
	 * VFE31 CAMIF register layout (based on WebOS register dump):
	 *
	 * FRAME_CFG at 0x1E8: WebOS does NOT set this register (leaves at 0).
	 *
	 * WINDOW_WIDTH_CFG (0x1EC): Frame dimensions
	 *   [29:16] = height (lines per frame)
	 *   [13:0]  = width (pixels/bytes per line)
	 *   WebOS example: 0x01E00500 = (480 << 16) | 1280
	 *
	 * WINDOW_HEIGHT_CFG (0x1F0): Last pixel index
	 *   [13:0]  = lastPixel = width - 1
	 *   WebOS example: 0x000004FF = 1279 = 1280 - 1
	 */
	{
		u32 width_bytes = line->fmt[MSM_VFE_PAD_SINK].width * 2;
		u32 height = line->fmt[MSM_VFE_PAD_SINK].height;

		/* WINDOW_WIDTH_CFG: (height << 16) | width_bytes */
		val = (height << 16) | (width_bytes & 0x3FFF);
		dev_info(vfe->camss->dev,
			 "VFE31: WINDOW_WIDTH_CFG (0x1EC) = 0x%08x (lines=%u, pixels=%u)\n",
			 val, height, width_bytes);
		writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);

		/* WINDOW_HEIGHT_CFG: lastPixel = width_bytes - 1 */
		val = (width_bytes - 1) & 0x3FFF;
		dev_info(vfe->camss->dev,
			 "VFE31: WINDOW_HEIGHT_CFG (0x1F0) = 0x%08x (lastPixel=%u)\n",
			 val, width_bytes - 1);
		writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);
	}

	/*
	 * SUBSAMPLE_CFG_0 at 0x1F4: webOS uses (height - 1) = 0x1DF for 480 lines
	 * This appears to be the last line number, not a skip pattern.
	 */
	val = line->fmt[MSM_VFE_PAD_SINK].height - 1;
	dev_info(vfe->camss->dev, "VFE31: SUBSAMPLE_CFG_0 (0x1F4) = 0x%08x (height-1)\n", val);
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);

	/*
	 * SUBSAMPLE_CFG_1 at 0x1F8: webOS uses 0xFFFFFFFF (no frame skip)
	 */
	dev_info(vfe->camss->dev, "VFE31: SUBSAMPLE_CFG_1 (0x1F8) = 0xFFFFFFFF (no skip)\n");
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_1);
	wmb();

	dev_info(vfe->camss->dev, "VFE31: Step 3 - CAMIF registers configured\n");

	/* Configure pixel pattern in CORE_CFG + bit 6 (webOS uses 0x46 for UYVY) */
	dev_info(vfe->camss->dev, "VFE31: Step 4b - CORE_CFG pixel pattern\n");
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
	/* Add bit 6 - webOS always sets this (0x46 instead of 0x06) */
	val |= VFE_0_CORE_CFG_INPUT_MUX_ENABLE;
	writel_relaxed(val, vfe->base + VFE_0_CORE_CFG);
	wmb();

	/*
	 * Step 4.5: Enable IRQs BEFORE starting CAMIF
	 *
	 * webOS uses IRQ_MASK_0 = 0x00EFE021 which includes:
	 * - Bit 0: SOF
	 * - Bit 5: REG_UPDATE
	 * - Bits 8-14: PING_PONG for WM0-6
	 * - etc.
	 *
	 * For RDI/raw mode we need at minimum: SOF + REG_UPDATE + PING_PONG for WM0
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 4.5 - Enable IRQs\n");
	/*
	 * CRITICAL: Configure IRQ_COMPOSITE_MASK (0x034) to map WMs to composite
	 * interrupt groups. Without this, IMAGE_COMPOSITE_DONE IRQs never fire!
	 *
	 * Bit layout:
	 * - Bits 0-7:   WMs mapped to COMPOSITE_DONE_0 (IRQ_STATUS_0 bit 21)
	 * - Bits 8-15:  WMs mapped to COMPOSITE_DONE_1 (IRQ_STATUS_0 bit 22)
	 * - Bits 16-23: WMs mapped to COMPOSITE_DONE_2 (IRQ_STATUS_0 bit 23)
	 *
	 * webOS mapping (from msm_vfe31.c):
	 * - Preview/PIX mode (OUTPUT_2, 0x200): WMs mapped to COMPOSITE_DONE_0
	 *     irq_comp_mask |= BIT(out0.ch0) | BIT(out0.ch1);  // bits 0-7
	 * - Raw snapshot mode (CAMIF_TO_AXI, 0x60): WM0 mapped to COMPOSITE_DONE_1
	 *     irq_comp_mask |= BIT(out1.ch0 + 8);  // bits 8-15
	 */
	{
		/*
		 * IRQ_COMPOSITE_MASK is write-only, use shadow register.
		 *
		 * WebOS register dump shows 0x00220011 for preview mode:
		 *   - 0x11 (bits 0-7):   WM0 + WM4 -> COMPOSITE_DONE_0
		 *   - 0x00 (bits 8-15):  nothing -> COMPOSITE_DONE_1
		 *   - 0x22 (bits 16-23): WM1 + WM5 -> COMPOSITE_DONE_2
		 *
		 * IMPORTANT: webOS mask requires WM0+WM4 AND WM1+WM5 to both
		 * complete. If WM4/WM5 aren't configured, COMPOSITE_DONE never
		 * fires! Use preview-only mask (0x00020001) when video disabled.
		 *
		 * Preview-only mask (FIXED for multi-WM support):
		 *   - 0x03 (bits 0-7):   WM0 + WM1 -> COMPOSITE_DONE_0
		 *
		 * By putting BOTH WM0 and WM1 in the same composite group,
		 * COMPOSITE_DONE_0 fires only when BOTH write masters complete.
		 * This ensures we deliver one frame per completion, not two.
		 *
		 * Previous buggy mask 0x00020001 put WM0 in group 0 and WM1
		 * in group 2, causing TWO interrupts per frame!
		 */
#define VFE31_IRQ_COMP_MASK_WEBOS		0x00220011  /* WM0+4,WM1+5 */
#define VFE31_IRQ_COMP_MASK_PREVIEW_ONLY	0x00000003  /* WM0+WM1 in group 0 */
		if (vfe31_video_output_enable) {
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_WEBOS;
			dev_info(vfe->camss->dev,
				 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (video+preview)\n",
				 vfe->irq_comp_mask_shadow);
		} else {
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PREVIEW_ONLY;
			dev_info(vfe->camss->dev,
				 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (preview-only)\n",
				 vfe->irq_comp_mask_shadow);
		}
		writel_relaxed(vfe->irq_comp_mask_shadow,
			       vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
		wmb();
	}

	/*
	 * Configure IRQ masks dynamically based on active WMs.
	 *
	 * WebOS uses IRQ_MASK_0 = 0x00EFE021 which includes:
	 *   - Bit 0: CAMIF_SOF
	 *   - Bit 5: REG_UPDATE
	 *   - Bits 8-14: PING_PONG for WM0-6
	 *   - Bits 21-23: IMAGE_COMPOSITE_DONE_0-2
	 *
	 * We build this dynamically to include PING_PONG for active WMs.
	 * For NV16 semi-planar (wm_num == 2), include both WM0 and WM1.
	 */
	vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
				VFE_0_IRQ_MASK_0_CAMIF_EOF |
				VFE_0_IRQ_MASK_0_REG_UPDATE |
				VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm) |
				VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(0) |
				VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(1) |
				VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);

	/* Add WM1 PING_PONG for NV16 semi-planar formats */
	if (line->output.wm_num == 2) {
		u8 wm1 = line->output.wm_idx[1];

		vfe->irq_mask0_shadow |= VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm1);
		dev_info(vfe->camss->dev,
			 "VFE31: Added WM%d PING_PONG to IRQ mask (NV16 mode)\n", wm1);
	}
	/*
	 * IRQ_MASK_1: webOS uses 0x00400000 (only RESET_ACK, bit 22).
	 *
	 * webOS does NOT mask VIOLATION or CAMIF_ERROR - these are considered
	 * informational and don't prevent frame capture. Masking VIOLATION
	 * causes spurious IRQs with status=0x00000000.
	 *
	 * Match webOS exactly to avoid spurious interrupts.
	 */
	vfe->irq_mask1_shadow = VFE_0_IRQ_MASK_1_RESET_ACK;

	dev_info(vfe->camss->dev,
		 "VFE31: Setting IRQ_MASK_0=0x%08x IRQ_MASK_1=0x%08x\n",
		 vfe->irq_mask0_shadow, vfe->irq_mask1_shadow);

	writel_relaxed(vfe->irq_mask0_shadow, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(vfe->irq_mask1_shadow, vfe->base + VFE_0_IRQ_MASK_1);
	wmb();

	/*
	 * REG_UPDATE + CAMIF_START sequence - match webOS exactly:
	 * 1. Write REG_UPDATE_CMD = 1 with memory barrier
	 * 2. Write CAMIF_COMMAND = 1 (start, no separate clear)
	 *
	 * webOS vfe31_start_common():
	 *   msm_io_w_mb(1, vfebase + VFE_REG_UPDATE_CMD);
	 *   msm_io_w(1, vfebase + VFE_CAMIF_COMMAND);
	 */
	dev_info(vfe->camss->dev, "VFE31: Issuing REG_UPDATE_CMD + CAMIF_START (webOS sequence)\n");
	writel(1, vfe->base + VFE_0_REG_UPDATE_CMD);
	wmb();
	writel(1, vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	/* Step 6: Enable Write Masters
	 *
	 * CRITICAL: vfe31_enable() bypasses the gen1 path which would normally
	 * call wm_enable(). We must enable WM here after CAMIF is configured
	 * and started, otherwise the WM never gets enabled and no data flows!
	 *
	 * WR_CFG bits:
	 *   Bit 0: enable
	 * Note: VFE31 does NOT have frame_based mode in bit 1. webOS writes 1.
	 *
	 * For NV16/semi-planar formats (wm_num == 2):
	 *   WM0 = Y plane (luma)
	 *   WM1 = CbCr plane (chroma)
	 * Both must be enabled for complete frame data!
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 6 - Enable Write Master WM%d\n", wm);
	writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
	wmb();

	/* Enable WM1 for semi-planar formats (NV16) */
	if (line->output.wm_num == 2) {
		u8 wm1 = line->output.wm_idx[1];

		dev_info(vfe->camss->dev,
			 "VFE31: Step 6b - Enable Write Master WM%d (CbCr plane)\n",
			 wm1);
		writel_relaxed(BIT(0),
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm1));
		wmb();
	}

	/*
	 * Step 7: Reload WM configuration via BUS_CMD
	 *
	 * After writing WM registers (ping/pong addresses, image size, etc.),
	 * we must issue a BUS_CMD reload to tell the DMA engine to read the
	 * new configuration. Without this, the DMA uses stale/zero addresses.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 7 - BUS_CMD reload WM%d\n", wm);
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm), vfe->base + VFE_0_BUS_CMD);
	wmb();

	/* Reload WM1 for semi-planar formats */
	if (line->output.wm_num == 2) {
		u8 wm1 = line->output.wm_idx[1];

		dev_info(vfe->camss->dev,
			 "VFE31: Step 7b - BUS_CMD reload WM%d (CbCr plane)\n",
			 wm1);
		writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm1),
			       vfe->base + VFE_0_BUS_CMD);
		wmb();
	}

	/*
	 * Step 8: Configure WM4/WM5 for video output (when enabled)
	 *
	 * XBAR 0x1A1B routes Y to WM0+WM4 and CbCr to WM1. We must configure
	 * WM4 to mirror WM0 even in preview-only mode, otherwise the XBAR
	 * routing fails and WM1 (CbCr) doesn't receive data.
	 *
	 * Configure WM4/WM5 with the same buffer addresses as WM0/WM1.
	 */
	if (vfe31_axi_output_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		struct v4l2_pix_format_mplane *pix =
			&line->video_out.active_fmt.fmt.pix_mp;
		u16 width = pix->width;
		u16 height = pix->height;
		/* Use UYVY input stride, not output plane bytesperline */
		u16 stride = width * 2;
		u32 cbcr_addr = 0;

		/* For semi-planar formats, CbCr is after Y plane */
		if (line->output.wm_num == 2)
			cbcr_addr = vfe->pending_ping_addr + (width * height);

		dev_info(vfe->camss->dev,
			 "VFE31: Step 8 - Configuring video WM4/WM5 (mirror preview)\n");
		vfe31_configure_video_wm(vfe, vfe->pending_ping_addr, cbcr_addr,
					 width, height, stride);

		/* Reload WM4 and WM5 */
		dev_info(vfe->camss->dev,
			 "VFE31: Step 8b - BUS_CMD reload WM4/WM5\n");
		writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(VFE31_VIDEO_WM_Y) |
			       VFE_0_BUS_CMD_Mx_RLD_CMD(VFE31_VIDEO_WM_CBCR),
			       vfe->base + VFE_0_BUS_CMD);
		wmb();
	}

	/* Debug dump of all relevant registers after CAMIF start */
	dev_info(vfe->camss->dev,
		 "VFE31: CAMIF started - comprehensive register dump:\n");
	dev_info(vfe->camss->dev,
		 "  CORE_CFG(0x014)=0x%08x  AXI_OUT_MODE(0x040)=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CORE_CFG),
		 readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG));
	dev_info(vfe->camss->dev,
		 "  EFS_CFG(0x1E4)=0x%08x  FRAME_CFG(0x1E8)=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_EFS_CFG),
		 readl_relaxed(vfe->base + VFE_0_CAMIF_FRAME_CFG));
	dev_info(vfe->camss->dev,
		 "  CAMIF_STATUS(0x204)=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS));
	dev_info(vfe->camss->dev,
		 "  IRQ_MASK_0(0x01C)=0x%08x  IRQ_MASK_1(0x020)=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_IRQ_MASK_0),
		 readl_relaxed(vfe->base + VFE_0_IRQ_MASK_1));
	dev_info(vfe->camss->dev,
		 "  IRQ_STATUS_0(0x02C)=0x%08x  IRQ_STATUS_1(0x030)=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_0),
		 readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_1));
	dev_info(vfe->camss->dev,
		 "  MODULE_CFG(0x010)=0x%08x  BUS_CFG(0x03C)=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_MODULE_CFG),
		 readl_relaxed(vfe->base + VFE_0_BUS_CFG));

	vfe->camif_pending = false;

	/*
	 * Schedule a delayed diagnostic check to verify CSI-to-VFE clock
	 * configuration after the sensor has had time to start streaming.
	 * This helps diagnose issues where data isn't reaching VFE.
	 */
	{
		void __iomem *mmcc_base;

		mmcc_base = ioremap(0x04000000, 0x200);
		if (mmcc_base) {
			u32 vfe_cc, misc_cc;

			vfe_cc = readl_relaxed(mmcc_base + 0x0104);
			misc_cc = readl_relaxed(mmcc_base + 0x0058);

			dev_info(vfe->camss->dev,
				 "VFE31: CSI-to-VFE clock state at CAMIF start:\n");
			dev_info(vfe->camss->dev,
				 "  VFE_CC_REG(0x0104)=0x%08x CSI0_VFE=%s CSI1_VFE=%s\n",
				 vfe_cc,
				 (vfe_cc & BIT(12)) ? "ON" : "off",
				 (vfe_cc & BIT(10)) ? "ON" : "off");
			dev_info(vfe->camss->dev,
				 "  MISC_CC_REG(0x0058)=0x%08x csi_pix_sel=%s csi_pix_en=%s csi_rdi_sel=%s csi_rdi_en=%s\n",
				 misc_cc,
				 (misc_cc & BIT(25)) ? "CSI1" : "CSI0",
				 (misc_cc & BIT(26)) ? "ON" : "off",
				 (misc_cc & BIT(12)) ? "CSI1" : "CSI0",
				 (misc_cc & BIT(13)) ? "ON" : "off");

			/* CRITICAL: Verify CSI1 is selected and enabled for MT9M113 */
			if (!(vfe_cc & BIT(10))) {
				dev_err(vfe->camss->dev,
					"VFE31 ERROR: CSI1_VFE_CLK not enabled in VFE_CC_REG!\n");
			}
			if (!(misc_cc & BIT(25)) || !(misc_cc & BIT(26))) {
				dev_err(vfe->camss->dev,
					"VFE31 ERROR: csi_pix not configured for CSI1! sel=%s en=%s\n",
					(misc_cc & BIT(25)) ? "CSI1" : "CSI0",
					(misc_cc & BIT(26)) ? "ON" : "off");
			}

			iounmap(mmcc_base);
		}
	}
}

static void vfe31_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable)
{
	u32 val;

	/*
	 * VFE31: Configure CAMIF FIRST if pending, before touching WR_CFG.
	 * Writing to WR_CFG before CAMIF is configured causes hangs.
	 */
	if (enable && vfe->camif_pending) {
		vfe31_start_camif_for_rdi(vfe, wm);
	}

	/*
	 * VFE31 WM enable - write complete WR_CFG value.
	 * Bit 0: enable
	 *
	 * Note: VFE31 does NOT have frame_based mode in WR_CFG bit 1.
	 * webOS only writes 1 here. Don't use read-modify-write as
	 * reading WR_CFG may cause hangs.
	 */
	if (enable)
		val = BIT(0);  /* enable only */
	else
		val = 0;

	dev_info(vfe->camss->dev, "VFE31: WM%d enable=%d reg=0x%03x val=0x%x\n",
		 wm, enable, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm), val);

	writel_relaxed(val, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
}

static void vfe31_wm_set_ub_cfg(struct vfe_device *vfe, u8 wm,
				u16 offset, u16 depth)
{
	/*
	 * VFE31: Defer UB_CFG write until after CAMIF is started.
	 * Writing to WM registers before CAMIF is configured causes bus hangs.
	 * Store the values and write them in vfe31_start_camif_for_rdi.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: wm_set_ub_cfg wm=%d offset=%d depth=%d (deferred to CAMIF start)\n",
		 wm, offset, depth);

	vfe->pending_ub_offset = offset;
	vfe->pending_ub_depth = depth;
}

static void vfe31_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	/*
	 * VFE31: If CAMIF is not yet running, defer the write.
	 * Once CAMIF is running (camif_pending=false), write directly.
	 */
	if (vfe->camif_pending) {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d ping_addr=0x%08x (deferred)\n", wm, addr);
		vfe->pending_ping_addr = addr;
	} else {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d ping_addr=0x%08x (direct write)\n", wm, addr);
		writel_relaxed(addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
	}

	/*
	 * VIDEO mode support: When video_output_enable=1, also configure
	 * WM4/WM5 with the same buffer addresses as WM0/WM1.
	 * This makes COMPOSITE_DONE fire because WM4/WM5 complete together
	 * with WM0/WM1, satisfying IRQ_COMPOSITE_MASK=0x00220011.
	 */
	if (vfe31_video_output_enable) {
		if (wm == 0) {
			/* WM0 (preview Y) -> also configure WM4 (video Y) */
			writel_relaxed(addr,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(VFE31_VIDEO_WM_Y));
			dev_dbg(vfe->camss->dev,
				"VFE31: WM4 ping_addr=0x%08x (video mirror)\n", addr);
		} else if (wm == 1) {
			/* WM1 (preview CbCr) -> also configure WM5 (video CbCr) */
			writel_relaxed(addr,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(VFE31_VIDEO_WM_CBCR));
			dev_dbg(vfe->camss->dev,
				"VFE31: WM5 ping_addr=0x%08x (video mirror)\n", addr);
		}
	}
}

static void vfe31_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	/*
	 * VFE31: If CAMIF is not yet running, defer the write.
	 * Once CAMIF is running (camif_pending=false), write directly.
	 */
	if (vfe->camif_pending) {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d pong_addr=0x%08x (deferred)\n", wm, addr);
		vfe->pending_pong_addr = addr;
	} else {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d pong_addr=0x%08x (direct write)\n", wm, addr);
		writel_relaxed(addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
	}

	/*
	 * VIDEO mode support: When video_output_enable=1, also configure
	 * WM4/WM5 with the same buffer addresses as WM0/WM1.
	 */
	if (vfe31_video_output_enable) {
		if (wm == 0) {
			/* WM0 (preview Y) -> also configure WM4 (video Y) */
			writel_relaxed(addr,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(VFE31_VIDEO_WM_Y));
			dev_dbg(vfe->camss->dev,
				"VFE31: WM4 pong_addr=0x%08x (video mirror)\n", addr);
		} else if (wm == 1) {
			/* WM1 (preview CbCr) -> also configure WM5 (video CbCr) */
			writel_relaxed(addr,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(VFE31_VIDEO_WM_CBCR));
			dev_dbg(vfe->camss->dev,
				"VFE31: WM5 pong_addr=0x%08x (video mirror)\n", addr);
		}
	}
}

static int vfe31_wm_get_ping_pong_status(struct vfe_device *vfe, u8 wm)
{
	u32 val = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (val >> wm) & 0x1;
}

static void vfe31_wm_set_framedrop_period(struct vfe_device *vfe, u8 wm,
					  u8 per)
{
	/*
	 * VFE31 uses global framedrop registers (0x504-0x520) for enc/view
	 * paths, not per-WM registers like VFE41. For raw passthrough mode
	 * (CAMIF_TO_BUS), framedrop is configured via the global registers.
	 *
	 * For WM0 (Y channel), use ENC_Y_CFG at 0x504.
	 * The period value in bits [3:0].
	 */
	if (wm == 0) {
		writel_relaxed(per, vfe->base + VFE31_FRAMEDROP_ENC_Y_CFG);
	} else if (wm == 1) {
		writel_relaxed(per, vfe->base + VFE31_FRAMEDROP_ENC_CBCR_CFG);
	}
	/* Other WMs use view path framedrop registers if needed */
}

static void vfe31_wm_set_framedrop_pattern(struct vfe_device *vfe, u8 wm,
					   u32 pattern)
{
	/*
	 * VFE31 uses global framedrop pattern registers.
	 * For WM0, use ENC_Y_PATTERN at 0x50C.
	 */
	if (wm == 0) {
		writel_relaxed(pattern, vfe->base + VFE31_FRAMEDROP_ENC_Y_PATTERN);
	} else if (wm == 1) {
		writel_relaxed(pattern, vfe->base + VFE31_FRAMEDROP_ENC_CBCR_PATTERN);
	}
}

static void vfe31_enable_irq_pix_line(struct vfe_device *vfe, u8 comp,
				      enum vfe_line_id line_id, u8 enable)
{
	/*
	 * IMPORTANT: VFE31 IRQ_MASK_0/1 registers are WRITE-ONLY!
	 * Use shadow registers instead of read-modify-write.
	 *
	 * NOTE: webOS does NOT enable CAMIF_ERROR (bit 0) in IRQ_MASK_1!
	 * webOS sets IRQ_MASK_1 = 0x00400000 (only RESET_ACK).
	 * The MT9M113 sensor doesn't send MIPI FS/FE short packets, so
	 * CAMIF_ERROR fires on every frame. Enabling it in the mask causes
	 * the ISR to process these errors, which interferes with normal
	 * frame completion via COMPOSITE_DONE.
	 *
	 * Setting val1 = 0 matches webOS behavior and allows frames to
	 * complete normally even without MIPI frame sync packets.
	 */
	u32 val0 = VFE_0_IRQ_MASK_0_CAMIF_SOF |
		   VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(comp) |
		   VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(VFE_LINE_PIX);
	u32 val1 = 0;  /* Don't enable CAMIF_ERROR - matches webOS */

	/*
	 * Configure IRQ_COMPOSITE_MASK_0 (0x034) to map WMs to composite
	 * groups. Without this, IMAGE_COMPOSITE_DONE IRQs never fire!
	 *
	 * WebOS register dump shows 0x00220011 for preview mode:
	 *   - 0x11 (bits 0-7):   WM0 + WM4 -> COMPOSITE_DONE_0
	 *   - 0x00 (bits 8-15):  nothing -> COMPOSITE_DONE_1
	 *   - 0x22 (bits 16-23): WM1 + WM5 -> COMPOSITE_DONE_2
	 *
	 * IMPORTANT: webOS mask requires WM0+WM4 AND WM1+WM5 to complete.
	 * Use preview-only mask (0x00020001) when video disabled.
	 *
	 * Also enable COMPOSITE_DONE_2 IRQ since WM1 is mapped there.
	 */
	if (enable) {
		/* Choose mask based on whether WM4/WM5 are active */
		if (vfe31_video_output_enable) {
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_WEBOS;
			/* Video mode: WM0+WM4 in group 0, WM1+WM5 in group 2 */
			val0 |= VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);
		} else {
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PREVIEW_ONLY;
			/* Preview-only: WM0+WM1 both in group 0, no group 2 needed */
		}

		dev_info(vfe->camss->dev,
			 "VFE31 pix_line: IRQ_COMPOSITE_MASK=0x%08x (%s)\n",
			 vfe->irq_comp_mask_shadow,
			 vfe31_video_output_enable ? "video+preview" : "preview-only");
		writel_relaxed(vfe->irq_comp_mask_shadow,
			       vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
	}

	if (enable) {
		vfe->irq_mask0_shadow |= val0;
		vfe->irq_mask1_shadow |= val1;
	} else {
		vfe->irq_mask0_shadow &= ~val0;
		vfe->irq_mask1_shadow &= ~val1;
	}

	writel_relaxed(vfe->irq_mask0_shadow, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(vfe->irq_mask1_shadow, vfe->base + VFE_0_IRQ_MASK_1);

	dev_info(vfe->camss->dev,
		 "VFE31 IRQ pix_line: enable=%d comp=%d mask0=0x%08x mask1=0x%08x\n",
		 enable, comp, vfe->irq_mask0_shadow, vfe->irq_mask1_shadow);
}

static void vfe31_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
				     enum vfe_line_id line_id, u8 enable)
{
	/*
	 * VFE31 RDI mode: Skip ALL IRQ mask writes here!
	 *
	 * Writing to IRQ_MASK_0 hangs because RDI bypasses CAMIF.
	 * Writing to IRQ_MASK_1 to enable BUS_OVERFLOW also hangs,
	 * likely because the WM isn't connected to RDI yet at this point
	 * (bus_connect_wm_to_rdi is called AFTER this function).
	 *
	 * The common IRQs (RESET_ACK, VIOLATION, HALT_ACK) are already
	 * enabled by enable_irq_common and should be sufficient for
	 * basic RDI operation.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31 enable_irq_wm_line: wm=%d line=%d enable=%d (NO-OP for RDI)\n",
		 wm, line_id, enable);
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
