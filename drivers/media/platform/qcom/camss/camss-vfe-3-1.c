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
 * ============================================================================
 * VFE31 WRITE MASTER (WM) PAIRING - VERIFIED FROM WEBOS KERNEL
 * ============================================================================
 *
 * Source: webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c
 *         Lines 710-722, vfe31_config_axi() OUTPUT_1_AND_3 case
 *
 * WebOS uses OFFSET-BY-4 WM PAIRING, not consecutive pairing:
 *   - Preview (out0): WM0 (Y) + WM4 (CbCr)
 *   - Video (out2):   WM1 (Y) + WM5 (CbCr)
 *
 * This is fundamentally different from what we originally assumed (WM0+WM1).
 *
 * WebOS code:
 *   case OUTPUT_1_AND_3: {
 *       // use wm0& 4 for preview, wm1&5 for video.
 *       *p++ = 0x1;      // xbar cfg0
 *       *p = 0x1a03;     // xbar cfg1
 *       vfe31_ctrl->outpath.out0.ch0 = 0;  // preview luma   → WM0
 *       vfe31_ctrl->outpath.out0.ch1 = 4;  // preview chroma → WM4
 *       vfe31_ctrl->outpath.out2.ch0 = 1;  // video luma     → WM1
 *       vfe31_ctrl->outpath.out2.ch1 = 5;  // video chroma   → WM5
 *   }
 *
 * IMPORTANT: WebOS DISABLED CbCr write masters in practice!
 *   See reports/webos-video-mode-dump.txt:
 *   - WM1_CFG_PNTR = 0x00000000 (DISABLED)
 *   - WM5_CFG_PNTR = 0x00000000 (DISABLED)
 *   WebOS only captured Y planes, never CbCr, so this pairing was untested!
 */

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
 * ============================================================================
 * VFE31 XBAR_CFG1 REGISTER - MODE-DEPENDENT ROUTING
 * ============================================================================
 *
 * XBAR_CFG1 (0x044) controls Y/CbCr routing to Write Masters.
 *
 * Bit field interpretation:
 *   Bits [3:0]  = Y routing (0x3 = WM0 only, 0xB = WM0+WM4)
 *   Bits [7:4]  = CbCr routing (0x1 = WM1 only)
 *   Bits [15:8] = 0x1A = standard ISP processing
 *
 * PIX-ONLY mode (0x1A13):
 *   - Y routes to WM0 only (avoids DMA to unconfigured WM4)
 *   - CbCr routes to WM1
 *   - Required because WM4 has no buffer in PIX-only mode
 *
 * PIX+VIDEO mode (0x1A1B):
 *   - Y routes to WM0 AND WM4 (both lines receive Y)
 *   - CbCr routes to WM1 (shared between lines)
 *   - WM4 must have buffer configured for VIDEO line
 *
 * webOS register dumps show 0x1A1B was used, but webOS disabled CbCr WMs
 * entirely (WM1_CFG_PNTR=0x00), only capturing Y planes.
 */
#define VFE31_XBAR_PIX_ONLY	0x1A13  /* Y→WM0 only, CbCr→WM1 (avoids WM4 DMA) */
#define VFE31_XBAR_PIX_VIDEO	0x1A1B  /* Y→WM0+WM4, CbCr→WM1 (VIDEO uses WM4) */

/* Module param for manual override/testing */
int vfe31_xbar_cfg1 = 0;  /* 0 = auto-select based on active lines */
module_param(vfe31_xbar_cfg1, int, 0644);
MODULE_PARM_DESC(vfe31_xbar_cfg1,
		 "VFE31 XBAR_CFG1 override (0=auto, 0x1a03=webOS default, 0x1a1b=pix+video)");

/*
 * ============================================================================
 * VFE31 IRQ COMPOSITE MASK - CORRECTED BASED ON REGISTER DUMPS
 * ============================================================================
 *
 * Controls which Write Masters trigger COMPOSITE_DONE interrupts.
 * Each bit corresponds to a Write Master (bit 0 = WM0, bit 1 = WM1, etc.)
 *
 * Group structure: bits 0-7 = Group 0, bits 8-15 = Group 1, etc.
 * CRITICAL: All WMs for a line MUST be in the same group! Mixing groups
 * causes the gen1 code to access non-existent line indices → crash.
 *
 * With XBAR 0x1A1B: Y→WM0+WM4, CbCr→WM1 only:
 *   PIX line:   WM0 (Y) + WM1 (CbCr) → bits 0,1 → 0x03
 *   VIDEO line: WM4 (Y) + WM1 (CbCr) → bits 4,1 → 0x12
 *   Combined:   WM0 + WM1 + WM4      → bits 0,1,4 → 0x13
 *
 * Note: WM1 is SHARED between PIX and VIDEO for CbCr!
 *
 * WebOS IRQ_COMPOSITE_MASK = 0x00220011 puts Y WMs in Group 0
 * and CbCr WMs in Group 2, but we use Group 0 for simplicity.
 */
#define VFE31_IRQ_COMP_MASK_PIX_ONLY	0x00000003  /* WM0+WM1 (PIX Y+CbCr) */
#define VFE31_IRQ_COMP_MASK_PIX_VIDEO	0x00000013  /* WM0+WM1+WM4 (shared WM1) */
#define VFE31_IRQ_COMP_MASK_VIDEO_ONLY	0x00000012  /* WM4+WM1 (VIDEO Y + shared CbCr) */

/* Module param for manual override/testing */
static int vfe31_irq_comp_mask = 0;  /* 0 = auto-select based on active lines */
module_param(vfe31_irq_comp_mask, int, 0644);
MODULE_PARM_DESC(vfe31_irq_comp_mask,
		 "VFE31 IRQ composite mask (0=auto, 0x03=pix, 0x13=pix+video, 0x12=video)");

/* External module parameters from camss-vfe.c */
extern int software_sof_enable;
extern int software_eof_enable;

/*
 * VFE31 XBAR routing - using webOS values:
 * - All modes: XBAR 0x1A03 with offset-by-4 WM pairing
 *   - PIX:   WM0 (Y) + WM4 (CbCr)
 *   - VIDEO: WM1 (Y) + WM5 (CbCr)
 * - Raw mode: XBAR 0x60 (CAMIF_TO_AXI) bypasses DEMUX
 *
 * Note: webOS never enabled CbCr WMs (WM4/WM5), so this routing is
 * technically untested by them. We're the first to try full NV16 output.
 */

/*
 * MSM8660 Clock Controller addresses for VFE AXI clock forcing.
 * Used by vfe31_force_enable_axi_clock() for testgen mode.
 */
#define MSM8660_MMCC_BASE		0x04000000
#define MSM8660_MMCC_SIZE		0x1000
#define MSM8660_VFE_CC_REG_OFFSET	0x0104
#define MSM8660_VFE_CC_REG_VFE_AXI_EN	BIT(1)

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
/* Input mux select (bits 4-5): 0=CAMIF, 1=TESTGEN, 2=unused, 3=AXI */
#define VFE_0_CORE_CFG_INPUT_MUX_TESTGEN	(1 << 4)
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
 * ============================================================================
 * XBAR_CFG1 VALUES - UPDATED BASED ON WEBOS ANALYSIS
 * ============================================================================
 *
 *   Value   Binary                  Our interpretation (may be wrong!)
 *   ─────────────────────────────────────────────────────────────────────────
 *   0x1A03  0001_1010_0000_0011     WebOS value: offset-by-4 WM pairing
 *   0x1A13  0001_1010_0001_0011     Our orig guess: Y→WM0, CbCr→WM1
 *   0x1A1B  0001_1010_0001_1011     Y→WM0+WM4, CbCr→WM1 (for VIDEO Y output)
 *   0x1A9B  (NOT SUPPORTED - bit 7 not writable, reads back as 0x1A1B)
 *
 * KEY INSIGHT FROM WEBOS KERNEL:
 *   - webOS used 0x1A03 with WM0 (Y) + WM4 (CbCr) for preview
 *   - webOS used 0x1A03 with WM1 (Y) + WM5 (CbCr) for video
 *   - BUT webOS NEVER ENABLED the CbCr WMs (WM4, WM5)!
 *   - So we don't actually know if 0x1A03 routes CbCr correctly
 *
 * Sources:
 *   - webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c
 *   - android.googlesource.com/kernel/msm msm_vfe31.c/msm_vfe32.c
 *   - gitlab.com/k2wl/g2_kernel msm_vfe31.h
 *
 * We use VFE31_XBAR_PIX_ONLY (0x1A03) defined at top of file, matching webOS.
 * The VFE31_XBAR_PIX_VIDEO (0x1A1B) is available for explicit VIDEO Y routing.
 */

#define VFE_0_BUS_CFG_RAW_WR_PATH_VIEW_CBCR	(2 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)

/*
 * ============================================================================
 * VFE31 WRITE MASTER (WM) ASSIGNMENTS
 * ============================================================================
 *
 * The VFE31 has 7 Write Masters (WM0-WM6) for DMA output.
 *
 * ACTUAL XBAR 0x1A1B ROUTING (from register dumps):
 *   Y data    → WM0 AND WM4 (both receive same Y!)
 *   CbCr data → WM1 ONLY (WM5 receives NOTHING!)
 *
 *   WM      PIX line        VIDEO line      Raw Mode
 *   ─────────────────────────────────────────────────────────────────────────
 *   WM0     Y plane         (Y duplicate)   Raw data
 *   WM1     CbCr plane      CbCr (shared!)  -
 *   WM2     -               -               -
 *   WM3     -               -               -
 *   WM4     (Y duplicate)   Y plane         -
 *   WM5     -               NOT USED        -
 *   WM6     -               -               -
 *
 * CRITICAL: WM1 is SHARED between PIX and VIDEO for CbCr!
 *
 * IRQ Composite Mask (bits = WM numbers):
 *   - PIX only:    0x03 = WM0 + WM1
 *   - VIDEO only:  0x12 = WM4 + WM1 (shared)
 *   - PIX + VIDEO: 0x13 = WM0 + WM1 + WM4
 */
/*
 * ============================================================================
 * WM ASSIGNMENTS - CORRECTED BASED ON ACTUAL REGISTER DUMPS
 * ============================================================================
 *
 * WebOS REGISTER DUMPS show XBAR_CFG1 = 0x1A1B (not 0x1A03!):
 *   bits [3:0] = 0xB = Y routes to WM0 AND WM4
 *   bits [7:4] = 0x1 = CbCr routes to WM1 ONLY
 *
 * This means:
 *   - Both WM0 and WM4 receive the SAME Y data from DEMUX
 *   - Only WM1 receives CbCr data (WM5 gets NOTHING!)
 *
 * The webOS code comment "wm0& 4 for preview, wm1&5 for video" is MISLEADING!
 * Those are software channel indices (outpath.out0.ch0/ch1), NOT XBAR routing.
 *
 * Correct WM assignments based on XBAR 0x1A1B:
 *   PIX:   WM0 (Y) + WM1 (CbCr)
 *   VIDEO: WM4 (Y) + WM1 (CbCr) - SHARED WM1!
 *
 * webOS worked around CbCr sharing by DISABLING CbCr WMs entirely.
 */
#define VFE31_PREVIEW_WM_Y		0
#define VFE31_PREVIEW_WM_CBCR		1  /* WM1 - only place CbCr is routed! */

/*
 * VIDEO mode: WM4 for Y, WM1 for CbCr (shared with PIX!)
 *
 * LIMITATION: PIX and VIDEO cannot both capture CbCr simultaneously
 * because they share WM1. For VIDEO-only mode, this works fine.
 * For simultaneous capture, need to disable one line's CbCr.
 */
#define VFE31_VIDEO_WM_Y		4  /* WM4 - receives Y from XBAR 0x1A1B */
#define VFE31_VIDEO_WM_CBCR		1  /* WM1 - shared with PIX (only CbCr destination!) */

/*
 * ============================================================================
 * CONFIGURATION SUMMARY (CORRECTED FROM REGISTER DUMPS)
 * ============================================================================
 *
 * For semi-planar preview (NV12/NV16/NV21/NV61):
 *   XBAR_CFG0 = 0x01   (OUTPUT_1_AND_3)
 *   XBAR_CFG1 = 0x1A1B (webOS actual: Y→WM0+WM4, CbCr→WM1 only!)
 *   BUS_CFG   = 0x02AAA771 (enable Y+CbCr write paths)
 *   WM0 = Y plane, WM1 = CbCr plane
 *
 * For video recording (VIDEO line active):
 *   XBAR_CFG0 = 0x01   (OUTPUT_1_AND_3)
 *   XBAR_CFG1 = 0x1A1B (Y duplicated to WM4, CbCr still only WM1)
 *   VIDEO: WM4 (Y) + WM1 (CbCr - SHARED with PIX!)
 *
 * For raw bypass (SRGGB8/10/12):
 *   XBAR_CFG0 = 0x60   (RAW_BYPASS / CAMIF_TO_AXI)
 *   XBAR_CFG1 = 0x00   (not used in raw mode)
 *   BUS_CFG   = configured for raw pixel size
 *
 * LIMITATION: PIX and VIDEO share WM1 for CbCr!
 * webOS disabled CbCr WMs entirely to avoid conflicts.
 * For VIDEO-only capture, WM1 is available since PIX isn't using it.
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
#define VFE_0_DEMUX_ODD_CFG		0x294

/*
 * Scale configuration - VFE31 has TWO scaler blocks:
 *
 * 1. Main Scaler (0x368-0x384): Y channel scaling
 *    VFE31 Main Scaler spans 28 bytes (7 registers), followed by WB at 0x384.
 *    Verified against Mako kernel: V31_MAIN_SCALER_OFF=0x368, V31_MAIN_SCALER_LEN=28
 *    Unlike VFE4x, VFE31 does NOT have dedicated CROP_ENC registers.
 *    Output cropping in VFE31 is handled by FOV (0x360) and scaler configuration.
 */
#define VFE_0_SCALE_Y_CFG		0x368
#define VFE_0_SCALE_Y_H_IMAGE		0x36C	/* (out << 16) | in */
#define VFE_0_SCALE_Y_H_PHASE		0x370
#define VFE_0_SCALE_Y_H_STRIPE		0x374	/* Horizontal stripe config (unused, 0) */
#define VFE_0_SCALE_Y_V_IMAGE		0x378	/* (out << 16) | in */
#define VFE_0_SCALE_Y_V_PHASE		0x37C
#define VFE_0_SCALE_Y_V_STRIPE		0x380	/* Vertical stripe config (unused, 0) */

/*
 * 2. Scaler 2 / Chroma Scale (0x4D0-0x504): S2Y and CbCr channel scaling
 *    Used for chroma subsampling (4:2:0 or 4:2:2).
 *    Verified against Mako kernel: V31_S2CbCr_OFF=0x4E4, V31_S2CbCr_LEN=20
 */
#define VFE_0_S2Y_H_IMAGE		0x4D4
#define VFE_0_S2Y_H_PHASE		0x4D8
#define VFE_0_S2Y_V_IMAGE		0x4DC
#define VFE_0_S2Y_V_PHASE		0x4E0
#define VFE_0_CHROMA_H_IMAGE		0x4E8	/* (out << 16) | in */
#define VFE_0_CHROMA_H_PHASE		0x4EC
#define VFE_0_CHROMA_V_IMAGE		0x4F0	/* (out << 16) | in */
#define VFE_0_CHROMA_V_PHASE		0x4F4

/*
 * Chroma subsample block (0x4F8-0x504): 12 bytes / 3 registers
 * Verified against Mako kernel: V31_CHROMA_SUBS_OFF=0x4F8, V31_CHROMA_SUBS_LEN=12
 */
#define VFE_0_CHROMA_SUBS_CFG		0x4F8	/* Chroma subsample config */
#define VFE_0_CHROMA_SUBS_CFG2		0x4FC	/* Additional config (unused) */
#define VFE_0_CHROMA_SUBS_CFG3		0x500	/* Additional config (unused) */

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
#define VFE_0_FOV_Y			0x360	/* FOV Y = width-1 */
#define VFE_0_FOV_CBCR			0x364	/* FOV CbCr = height-1 */
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

/* TESTGEN_CFG bit definitions */
#define VFE_0_TESTGEN_CFG_ENABLE	BIT(0)
#define VFE_0_TESTGEN_CFG_COLORBAR	(0 << 1)
#define VFE_0_TESTGEN_CFG_RANDOM	(1 << 1)
#define VFE_0_TESTGEN_CFG_CHECKER	(2 << 1)
#define VFE_0_TESTGEN_CFG_SOLID		(3 << 1)

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
		u32 window_width_cfg = readl_relaxed(vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);
		u32 subsample_cfg = readl_relaxed(vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);
		/*
		 * WINDOW_WIDTH_CFG: [29:16]=height, [13:0]=width_bytes
		 * SUBSAMPLE_CFG_0: [13:0]=lastLine (height-1)
		 * CAMIF_STATUS: [29:16]=received_lines, [13:0]=received_pixels
		 */
		u32 expected_lines = (subsample_cfg & 0x3FFF) + 1;  /* height */
		u32 expected_pixels = window_width_cfg & 0x3FFF;    /* width_bytes */
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
			/*
			 * Clear PING_PONG bits for WMs handled by comp_done.
			 * Note: VFE31 doesn't have per-WM PING_PONG IRQ bits in
			 * STATUS_0 (bits 8+ are for bus overflow errors), but we
			 * clear them anyway for compatibility with newer VFEs.
			 */
			for (j = 0; j < ARRAY_SIZE(vfe->wm_output_map); j++) {
				enum vfe_line_id line = vfe->wm_output_map[j];
				if (line == VFE_LINE_PIX ||
				    line == VFE_LINE_VIDEO ||
				    line == VFE_LINE_RDI0 ||
				    line == VFE_LINE_RDI1 ||
				    line == VFE_LINE_RDI2)
					value0 &= ~VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(j);
			}
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
	 * Determine AXI output mode based on line type:
	 * - RDI lines (RDI0, RDI1, RDI2): MUST use RAW mode (0x60) for raw bypass
	 * - PIX/VIDEO lines: Use module parameter (default 0x01 for DEMUX)
	 *
	 * VFE31 doesn't have true RDI hardware. RDI is emulated by configuring
	 * the AXI output mode for raw bypass (CAMIF_TO_AXI = 0x60).
	 */
	u32 axi_mode;
	bool is_rdi_line = (line->id == VFE_LINE_RDI0 ||
			    line->id == VFE_LINE_RDI1 ||
			    line->id == VFE_LINE_RDI2);

	if (is_rdi_line) {
		/* RDI lines always use raw bypass mode */
		axi_mode = 0x60;
		dev_info(vfe->camss->dev,
			 "VFE31: RDI line %d - forcing RAW mode (axi=0x60)\n",
			 line->id);
	} else {
		/* PIX/VIDEO lines use module parameter */
		axi_mode = vfe31_axi_output_mode;
	}

	/*
	 * WM configuration for PIX mode with DEMUX (ISP processing):
	 *
	 * Using webOS offset-by-4 WM pairing:
	 *   - PIX line:   WM0 (Y) + WM4 (CbCr)
	 *   - VIDEO line: WM1 (Y) + WM5 (CbCr)
	 *
	 * For raw/RDI mode (AXI=0x60), only WM0 is needed.
	 * For PIX mode with DEMUX (AXI=0x01), we need both Y and CbCr WMs
	 * because DEMUX separates Y and CbCr internally.
	 *
	 * Note: Even for "packed" UYVY format, the VFE31 DEMUX outputs
	 * Y and CbCr to separate WMs. The CbCr is interleaved (CbYCrY).
	 */
	if (axi_mode == 0x01) {
		/* PIX mode: need Y WM + CbCr WM (offset-by-4 pairing) */
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

	/*
	 * VFE31 WM assignment - using webOS offset-by-4 pairing:
	 * - VFE_LINE_PIX (preview): WM0 (Y) + WM4 (CbCr)
	 * - VFE_LINE_VIDEO:         WM1 (Y) + WM5 (CbCr)
	 *
	 * This allows PIX and VIDEO to capture simultaneously without
	 * sharing any write masters.
	 */
	if (line->id == VFE_LINE_VIDEO) {
		/* VIDEO line: WM1 for Y */
		wm_idx = vfe_reserve_wm_specific(vfe, VFE31_VIDEO_WM_Y, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for VIDEO Y\n",
				VFE31_VIDEO_WM_Y);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		if (output->wm_num == 2) {
			/*
			 * VIDEO line: WM1 for CbCr (shared with PIX)
			 *
			 * Note: Unlike PIX mode, VIDEO mode does NOT clear the
			 * line mapping for WM1. VIDEO uses WM4 for Y (not WM0),
			 * so when COMPOSITE_DONE fires, wm_done(WM4) is called
			 * first and processes the frame. If we also cleared WM1's
			 * mapping, the double buffer processing fix would cause
			 * VIDEO mode to fail.
			 */
			wm_idx = vfe_reserve_wm_specific(vfe, VFE31_VIDEO_WM_CBCR, line->id);
			if (wm_idx < 0) {
				dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for VIDEO CbCr\n",
					VFE31_VIDEO_WM_CBCR);
				vfe_release_wm(vfe, output->wm_idx[0]);
				output->state = VFE_OUTPUT_OFF;
				spin_unlock_irqrestore(&vfe->output_lock, flags);
				return wm_idx;
			}
			output->wm_idx[1] = wm_idx;
			/* Keep WM1 mapped to VIDEO line for buffer completion */
		}
		dev_info(vfe->camss->dev, "VFE31: VIDEO line using WM%d(Y), WM%d(CbCr)\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else if (line->id == VFE_LINE_PIX) {
		/* PIX line: WM0 for Y, WM4 for CbCr (webOS offset-by-4) */
		wm_idx = vfe_reserve_wm_specific(vfe, VFE31_PREVIEW_WM_Y, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for PIX Y\n",
				VFE31_PREVIEW_WM_Y);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		if (output->wm_num == 2) {
			/*
			 * PIX line: WM1 for CbCr (semi-planar format)
			 *
			 * IMPORTANT: Reserve WM1 but do NOT map it to this line!
			 * Both WMs share the same frame buffer. If we map WM1 to
			 * the line, vfe_isr_comp_done() will call wm_done() for
			 * BOTH WMs, causing double buffer processing and state
			 * corruption. Only WM0 (Y plane) should trigger buffer
			 * completion - WM1 just needs to be claimed so it's not
			 * used by another line.
			 */
			wm_idx = vfe_reserve_wm_specific(vfe, VFE31_PREVIEW_WM_CBCR, line->id);
			if (wm_idx < 0) {
				dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for PIX CbCr\n",
					VFE31_PREVIEW_WM_CBCR);
				vfe_release_wm(vfe, output->wm_idx[0]);
				output->state = VFE_OUTPUT_OFF;
				spin_unlock_irqrestore(&vfe->output_lock, flags);
				return wm_idx;
			}
			output->wm_idx[1] = wm_idx;
			/*
			 * Clear the line mapping for WM1 - it's claimed but shouldn't
			 * trigger buffer completion. The wm_done() handler checks
			 * wm_output_map and skips WMs mapped to VFE_LINE_NONE.
			 */
			vfe->wm_output_map[wm_idx] = VFE_LINE_NONE;
		}
		dev_info(vfe->camss->dev, "VFE31: PIX line using WM%d(Y), WM%d(CbCr, no-map)\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else {
		/* RDI lines use first available WMs */
		wm_idx = vfe_reserve_wm(vfe, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM for line %d\n", line->id);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		if (output->wm_num == 2) {
			wm_idx = vfe_reserve_wm(vfe, line->id);
			if (wm_idx < 0) {
				dev_err(vfe->camss->dev, "VFE31: Cannot reserve second WM\n");
				vfe_release_wm(vfe, output->wm_idx[0]);
				output->state = VFE_OUTPUT_OFF;
				spin_unlock_irqrestore(&vfe->output_lock, flags);
				return wm_idx;
			}
			output->wm_idx[1] = wm_idx;
		}
		dev_info(vfe->camss->dev, "VFE31: Line %d using WM%d, WM%d\n",
			 line->id, output->wm_idx[0],
			 output->wm_num == 2 ? output->wm_idx[1] : -1);
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
	 * VFE31 DMA stride must match UYVY input line size (width * 2), NOT the
	 * individual output plane size. The DEMUX separates Y and CbCr internally,
	 * but the DMA operates on the full UYVY line width for addressing.
	 *
	 * webOS always used stride=1280 for 640x480, stride=2560 for 1280x1024.
	 * Using plane_fmt[0].bytesperline (640/1280) causes data misalignment.
	 */
	bytesperline = width * 2;  /* UYVY input line size */

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
		 VFE_0_BUS_CFG_WEBOS_VALUE, axi_mode);
	writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
	writel_relaxed(axi_mode, vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);

	/*
	 * For PIX mode (OUTPUT_1_AND_3), configure XBAR_CFG1 to route
	 * Y to WM0 and CbCr to WM1.
	 *
	 * Note: This is an initial setup. The final XBAR value is set in
	 * enable_pending_camif() which uses auto-select logic based on
	 * whether VIDEO line is active. Use PIX_ONLY as default here.
	 *
	 * When vfe31_xbar_cfg1 module param is 0 (auto mode), use the
	 * PIX_ONLY default. If param is set explicitly, use that value.
	 */
	if (axi_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		u32 xbar_initial = (vfe31_xbar_cfg1 != 0) ?
				   vfe31_xbar_cfg1 : VFE31_XBAR_PIX_ONLY;
		dev_info(vfe->camss->dev,
			 "VFE31: PIX mode - XBAR CFG1=0x%x (initial, param=%d)\n",
			 xbar_initial, vfe31_xbar_cfg1);
		writel_relaxed(xbar_initial, vfe->base + VFE_0_BUS_XBAR_CFG1);
	}

	/*
	 * Step 1b: Configure DEMUX, scale and crop modules (PIX mode only)
	 * These must be set up before WM registers for the ISP pipeline
	 * to process data correctly. DEMUX is essential for YUV data.
	 *
	 * For RDI mode (axi=0x60), data bypasses the ISP entirely,
	 * so DEMUX/scale/crop configuration is not needed.
	 */
	if (axi_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		dev_info(vfe->camss->dev, "VFE31: Step 1b - Configure demux/scale/crop\n");
		vfe31_set_demux_cfg(vfe, line);
		vfe31_set_scale_cfg(vfe, line);
		vfe31_set_crop_cfg(vfe, line);
	} else {
		dev_info(vfe->camss->dev, "VFE31: Step 1b - Skip ISP config (RDI mode)\n");
	}

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
	 * For RDI lines (raw bypass mode), CAMIF is NOT used. Data goes
	 * directly from CSID to the write master without ISP processing.
	 * Skip CAMIF configuration entirely for RDI lines.
	 *
	 * Set camif_pending flag here. The actual CAMIF configuration
	 * (steps 3-6) will be done by vfe_enable_pending_camif() which
	 * is called from CSIPHY set_stream after lanes are enabled.
	 */
	if (is_rdi_line) {
		/*
		 * RDI mode: Data bypasses DEMUX/XBAR but still needs CAMIF config.
		 * VFE31 raw bypass mode (AXI=0x60) routes: CSID → CAMIF → WM0
		 *
		 * Unlike PIX mode which defers CAMIF config until CSIPHY is ready,
		 * RDI mode also needs deferral because CAMIF depends on CSIPHY timing.
		 *
		 * Set up IRQs and basic config here, but defer CAMIF to
		 * vfe31_enable_pending_camif() like PIX mode does.
		 */
		dev_info(vfe->camss->dev,
			 "VFE31: RDI line %d - deferring to pending_camif (raw bypass)\n",
			 line->id);

		/*
		 * For RDI, we still use the camif_pending path but with
		 * AXI mode 0x60 (already set above in the BUS registers).
		 * The pending_camif handler will configure CAMIF and IRQs.
		 */
		vfe->camif_pending = true;
		vfe->camif_pending_wm = wm;
		vfe->camif_pending_line_id = line->id;
	} else {
		dev_info(vfe->camss->dev,
			 "VFE31: Deferring CAMIF config until CSIPHY ready (WM%d, line %d)\n",
			 wm, line->id);

		vfe->camif_pending = true;
		vfe->camif_pending_wm = wm;
		vfe->camif_pending_line_id = line->id;
	}

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
	 * Write even/odd config to separate DEMUX registers.
	 *
	 * VFE31 DEMUX register layout (from webOS msm_vfe31.h V31_DEMUX_LEN=20):
	 *   0x284: DEMUX_CFG     (period)
	 *   0x288: DEMUX_GAIN_0  (Y gains)
	 *   0x28C: DEMUX_GAIN_1  (CbCr gains)
	 *   0x290: DEMUX_EVEN_CFG (even column routing)
	 *   0x294: DEMUX_ODD_CFG  (odd column routing)
	 *
	 * VFE4x writes the SAME pattern to both EVEN_CFG and ODD_CFG.
	 * For UYVY: both get 0xC9CA.
	 *
	 * Previous code only wrote to EVEN_CFG, leaving ODD_CFG uninitialized.
	 * This caused the DEMUX to output Y on both Y and CbCr channels.
	 */
	val = (even_cfg << 8) | odd_cfg;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_EVEN_CFG);
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_ODD_CFG);

	/* Readback to verify */
	{
		u32 cfg_rb = readl_relaxed(vfe->base + VFE_0_DEMUX_CFG);
		u32 even_rb = readl_relaxed(vfe->base + VFE_0_DEMUX_EVEN_CFG);
		u32 odd_rb = readl_relaxed(vfe->base + VFE_0_DEMUX_ODD_CFG);

		dev_info(vfe->camss->dev,
			 "VFE31: DEMUX config: EVEN=0x%04x ODD=0x%04x (readback EVEN=0x%x ODD=0x%x)\n",
			 val, val, even_rb, odd_rb);
	}
}

static void vfe31_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 width = line->fmt[MSM_VFE_PAD_SINK].width;
	u32 height = line->fmt[MSM_VFE_PAD_SINK].height;
	u32 p = line->video_out.active_fmt.fmt.pix_mp.pixelformat;

	/*
	 * VFE31 scale/FOV configuration (from webOS register dumps):
	 *
	 * FOV (Field of View) - 0x360-0x364:
	 *    - 0x360: FOV_Y = width-1
	 *    - 0x364: FOV_CBCR = height-1
	 *
	 * Main Scaler (0x368-0x37C): Y channel scaling
	 *    - 0x368: SCALE_Y_CFG (enable=0x03)
	 *    - 0x36C: SCALE_Y_H_IMAGE (out<<16 | in)
	 *    - 0x370: SCALE_Y_H_PHASE
	 *    - 0x378: SCALE_Y_V_IMAGE (out<<16 | in)
	 *    - 0x37C: SCALE_Y_V_PHASE
	 *
	 * Scaler 2 / Chroma Scale (0x4D0-0x4F8): CbCr channel scaling
	 *    - 0x4D0: S2Y_CFG (enable=0x03)
	 *    - 0x4E4: S2CBCR_CFG (enable=0x03)
	 *    - 0x4E8: CHROMA_H_IMAGE (out<<16 | in)
	 *    - 0x4EC: CHROMA_H_PHASE
	 *    - 0x4F0: CHROMA_V_IMAGE (out<<16 | in)
	 *    - 0x4F4: CHROMA_V_PHASE
	 *    - 0x4F8: CHROMA_SUBS_CFG (webOS: 0x30)
	 *
	 * Phase value 0x00310000 = 1:1 scaling
	 * Phase value 0x00320000 = 2:1 scaling
	 */

	/* FOV - Field of View (input cropping) */
	writel_relaxed(width - 1, vfe->base + VFE_0_FOV_Y);
	writel_relaxed(height - 1, vfe->base + VFE_0_FOV_CBCR);

	/* Main Scaler - Y channel (1:1 scaling) */
	writel_relaxed(0x03, vfe->base + VFE_0_SCALE_Y_CFG);
	writel_relaxed((width << 16) | width, vfe->base + VFE_0_SCALE_Y_H_IMAGE);
	writel_relaxed(0x00310000, vfe->base + VFE_0_SCALE_Y_H_PHASE);
	writel_relaxed((height << 16) | height, vfe->base + VFE_0_SCALE_Y_V_IMAGE);
	writel_relaxed(0x00310000, vfe->base + VFE_0_SCALE_Y_V_PHASE);

	/* Scaler 2 - Y pass-through */
	writel_relaxed(0x03, vfe->base + VFE_0_S2Y_CFG);
	writel_relaxed((width << 16) | width, vfe->base + VFE_0_S2Y_H_IMAGE);
	writel_relaxed(0x00310000, vfe->base + VFE_0_S2Y_H_PHASE);
	writel_relaxed((height << 16) | height, vfe->base + VFE_0_S2Y_V_IMAGE);
	writel_relaxed(0x00310000, vfe->base + VFE_0_S2Y_V_PHASE);

	/* Scaler 2 - CbCr channel (chroma subsampling) */
	writel_relaxed(0x03, vfe->base + VFE_0_S2CBCR_CFG);

	/*
	 * Chroma horizontal: always 2:1 subsample (one Cb-Cr pair per 2 pixels)
	 * Input = width, Output = width/2 (in samples, not bytes)
	 */
	writel_relaxed(((width / 2) << 16) | width, vfe->base + VFE_0_CHROMA_H_IMAGE);
	writel_relaxed(0x00320000, vfe->base + VFE_0_CHROMA_H_PHASE);

	/*
	 * Chroma vertical: depends on output format
	 * - NV12/NV21 (4:2:0): 2:1 vertical subsample
	 * - NV16/NV61 (4:2:2): 1:1 (no vertical subsample)
	 */
	if (p == V4L2_PIX_FMT_NV12 || p == V4L2_PIX_FMT_NV21) {
		/* 4:2:0: vertical 2:1 subsample */
		writel_relaxed(((height / 2) << 16) | height, vfe->base + VFE_0_CHROMA_V_IMAGE);
		writel_relaxed(0x00320000, vfe->base + VFE_0_CHROMA_V_PHASE);
	} else {
		/* 4:2:2 (NV16/NV61): no vertical subsample */
		writel_relaxed((height << 16) | height, vfe->base + VFE_0_CHROMA_V_IMAGE);
		writel_relaxed(0x00310000, vfe->base + VFE_0_CHROMA_V_PHASE);
	}

	/* Chroma subsample config - webOS uses 0x30 */
	writel_relaxed(0x30, vfe->base + VFE_0_CHROMA_SUBS_CFG);

	dev_info(vfe->camss->dev,
		 "VFE31: Scale/FOV configured: %ux%u, format=0x%x, chroma_v=%s\n",
		 width, height, p,
		 (p == V4L2_PIX_FMT_NV12 || p == V4L2_PIX_FMT_NV21) ? "2:1" : "1:1");
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
 * WARNING: This function is currently DISABLED/NOT USED because it causes
 * kernel memory corruption when WM4/WM5 are enabled with the same addresses
 * as WM0/WM1. XBAR 0x1A1B routes Y to both WM0 AND WM4, so if both are enabled
 * with the same address, two DMA engines write to the same memory.
 *
 * For future VIDEO line support, this function needs to be called with
 * SEPARATE buffer addresses (not the same as WM0/WM1 preview buffers).
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

	/*
	 * DO NOT enable WM4 here! Enabling causes DMA corruption because
	 * XBAR routes Y to both WM0 and WM4. If this function is called,
	 * the caller must enable WM4 explicitly after ensuring the address
	 * is different from WM0.
	 */

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

		/* CbCr ADDR_CFG - use (height-24) similar to preview CbCr (WM4) */
		reg = ((height - 24) << 16) | ((wpl - 17) & 0xFFFF);
		writel_relaxed(reg,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_CBCR));

		reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
		reg |= (height - 1) & 0xFFFF;
		writel_relaxed(reg,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(VFE31_VIDEO_WM_CBCR));

		/*
		 * WM5 (VIDEO CbCr) is configured but not enabled here.
		 * The caller must enable WM5 explicitly after ensuring
		 * proper buffer addresses are set.
		 */
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
		/*
		 * PIX mode (0x01): Use XBAR to route DEMUX output to WMs
		 *
		 * Using webOS offset-by-4 WM pairing:
		 *   - PIX:   WM0 (Y) + WM4 (CbCr), XBAR=0x1A03
		 *   - VIDEO: WM1 (Y) + WM5 (CbCr), XBAR=0x1A1B
		 *
		 * When VIDEO is active, we use 0x1A1B to also route Y to WM1.
		 */
		struct vfe_output *video_output = &vfe->line[VFE_LINE_VIDEO].output;
		bool video_active = (video_output->state == VFE_OUTPUT_ON ||
				     video_output->state == VFE_OUTPUT_RESERVED ||
				     video_output->state == VFE_OUTPUT_CONTINUOUS);
		u32 xbar_value;

		/* Use manual override if set, otherwise auto-select */
		if (vfe31_xbar_cfg1 != 0) {
			xbar_value = vfe31_xbar_cfg1;
		} else if (line->id == VFE_LINE_VIDEO || video_active) {
			/* VIDEO line active - route Y to WM0+WM4 */
			xbar_value = VFE31_XBAR_PIX_VIDEO;
		} else {
			/* PIX only - route Y to WM0 only, no WM4 */
			xbar_value = VFE31_XBAR_PIX_ONLY;
		}

		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - PIX mode: BUS_CFG=0x%08x, AXI=0x01, XBAR=0x%04x (%s)\n",
			 VFE_0_BUS_CFG_WEBOS_VALUE, xbar_value,
			 xbar_value == VFE31_XBAR_PIX_ONLY ? "PIX only" :
			 xbar_value == VFE31_XBAR_PIX_VIDEO ? "PIX+VIDEO" : "manual");
		writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(VFE_0_BUS_XBAR_CFG0_PIX_MODE,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		writel_relaxed(xbar_value, vfe->base + VFE_0_BUS_XBAR_CFG1);
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
		 * Using webOS offset-by-4 WM pairing:
		 *   - PIX:   WM0 (Y) + WM4 (CbCr) → mask 0x11
		 *   - VIDEO: WM1 (Y) + WM5 (CbCr) → mask 0x22
		 *   - Both:  WM0+WM1+WM4+WM5     → mask 0x33
		 *
		 * All WMs are in Group 0 (bits 0-7), so COMPOSITE_DONE_0 fires
		 * when all enabled WMs complete. This ensures we deliver one
		 * frame per completion, not multiple partial frames.
		 */
		{
			struct vfe_output *video_out = &vfe->line[VFE_LINE_VIDEO].output;
			struct vfe_output *pix_out = &vfe->line[VFE_LINE_PIX].output;
			bool video_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool pix_active = (pix_out->state == VFE_OUTPUT_ON ||
					   pix_out->state == VFE_OUTPUT_RESERVED ||
					   pix_out->state == VFE_OUTPUT_CONTINUOUS);

			/* Module param override takes priority */
			if (vfe31_irq_comp_mask != 0) {
				vfe->irq_comp_mask_shadow = vfe31_irq_comp_mask;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (module param)\n",
					 vfe->irq_comp_mask_shadow);
			} else if ((line->id == VFE_LINE_VIDEO || video_active) && !pix_active) {
				/* VIDEO-only: WM1 (Y) + WM4 (CbCr), no WM0 */
				vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (VIDEO only)\n",
					 vfe->irq_comp_mask_shadow);
			} else if ((line->id == VFE_LINE_VIDEO || video_active) && pix_active) {
				/* PIX+VIDEO: both lines active, wait for all WMs */
				vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PIX_VIDEO;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (PIX+VIDEO)\n",
					 vfe->irq_comp_mask_shadow);
			} else {
				/* PIX only */
				vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PIX_ONLY;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (PIX only)\n",
					 vfe->irq_comp_mask_shadow);
			}
			writel_relaxed(vfe->irq_comp_mask_shadow,
				       vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
			wmb();
		}
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
	 *
	 * IMPORTANT: Use line->output.wm_idx[0] for the primary WM, NOT the 'wm'
	 * parameter! The 'wm' parameter is whichever WM was enabled first, which
	 * may be WM1/WM5 (CbCr) rather than WM0/WM4 (Y). Using wm_idx[0] ensures
	 * we always enable IRQs for the correct primary write master.
	 */
	{
		u8 wm0 = line->output.wm_idx[0];

		vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
					VFE_0_IRQ_MASK_0_CAMIF_EOF |
					VFE_0_IRQ_MASK_0_REG_UPDATE |
					VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm0) |
					VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(0) |
					VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(1) |
					VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);

		dev_info(vfe->camss->dev,
			 "VFE31: IRQ_MASK_0 primary WM%d (wm_idx[0]), param wm=%d\n",
			 wm0, wm);
	}

	/* Add second WM PING_PONG for NV16 semi-planar formats */
	if (line->output.wm_num == 2) {
		u8 wm1 = line->output.wm_idx[1];

		vfe->irq_mask0_shadow |= VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm1);
		dev_info(vfe->camss->dev,
			 "VFE31: Added WM%d PING_PONG to IRQ mask (semi-planar)\n", wm1);
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
	 * IMPORTANT: Use line->output.wm_idx[] to enable the correct WMs for
	 * this line, NOT the 'wm' parameter (which may be any WM).
	 */
	{
		u8 wm0 = line->output.wm_idx[0];

		dev_info(vfe->camss->dev,
			 "VFE31: Step 6 - Enable Write Master WM%d (Y plane)\n", wm0);
		writel_relaxed(BIT(0),
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm0));
		wmb();
	}

	/* Enable second WM for semi-planar formats (NV16) */
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
	 *
	 * IMPORTANT: Use line->output.wm_idx[] to reload the correct WMs.
	 */
	{
		u8 wm0 = line->output.wm_idx[0];

		dev_info(vfe->camss->dev,
			 "VFE31: Step 7 - BUS_CMD reload WM%d (Y plane)\n", wm0);
		writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm0),
			       vfe->base + VFE_0_BUS_CMD);
		wmb();
	}

	/* Reload second WM for semi-planar formats */
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
	 * Step 8: VIDEO line WM configuration
	 *
	 * Using webOS offset-by-4 pairing:
	 *   - VIDEO line uses WM1 (Y) + WM5 (CbCr)
	 *   - When VIDEO is active, XBAR=0x1A1B routes Y to WM0+WM1
	 */
	if (vfe31_axi_output_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		if (line->id == VFE_LINE_VIDEO) {
			/*
			 * VIDEO line starting - configure WM1 (Y) and WM5 (CbCr)
			 * with VIDEO line's buffer addresses.
			 */
			struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
			u32 width = pix->width;
			u32 height = pix->height;
			u32 stride = pix->plane_fmt[0].bytesperline;
			u32 wpl = stride / 8;
			u32 reg;

			dev_info(vfe->camss->dev,
				 "VFE31: Step 8 - VIDEO line: configuring WM%d(Y)/WM%d(CbCr)\n",
				 VFE31_VIDEO_WM_Y, VFE31_VIDEO_WM_CBCR);

			/* VIDEO Y WM configuration */
			/* Addresses set via wm_set_ping_addr/wm_set_pong_addr */

			/* VIDEO Y WM IMAGE_SIZE */
			reg = ((height - 1) & 0xFFF) << 16;
			reg |= (width - 1) & 0x1FFF;
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(VFE31_VIDEO_WM_Y));

			/* VIDEO Y WM ADDR_CFG */
			reg = ((wpl - 1) & 0xFFFF) << 16;
			reg |= (height - 1) & 0xFFFF;
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_Y));

			/* VIDEO Y WM UB_CFG */
			reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
			reg |= (height - 1) & 0xFFFF;
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(VFE31_VIDEO_WM_Y));

			/* Enable VIDEO Y WM */
			writel_relaxed(BIT(0),
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
			writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(VFE31_VIDEO_WM_Y),
				       vfe->base + VFE_0_BUS_CMD);

			/*
			 * VIDEO CbCr WM (WM5 with offset-by-4 pairing)
			 */
			if (line->output.wm_num == 2) {
				/* VIDEO CbCr IMAGE_SIZE */
				reg = ((height - 1) & 0xFFF) << 16;
				reg |= (width - 1) & 0x1FFF;
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(VFE31_VIDEO_WM_CBCR));

				/* VIDEO CbCr ADDR_CFG */
				reg = ((wpl - 1) & 0xFFFF) << 16;
				reg |= (height - 1) & 0xFFFF;
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_CBCR));

				/* Enable VIDEO CbCr WM */
				writel_relaxed(BIT(0),
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_CBCR));
				writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(VFE31_VIDEO_WM_CBCR),
					       vfe->base + VFE_0_BUS_CMD);

				dev_info(vfe->camss->dev,
					 "VFE31: WM%d enabled for VIDEO CbCr\n",
					 VFE31_VIDEO_WM_CBCR);
			}
			wmb();

			dev_info(vfe->camss->dev,
				 "VFE31: VIDEO line WM%d(Y)+WM%d(CbCr): %ux%u stride=%u\n",
				 VFE31_VIDEO_WM_Y, VFE31_VIDEO_WM_CBCR, width, height, stride);
		} else {
			/*
			 * PIX line only (not VIDEO line).
			 * Disable VIDEO WMs to ensure clean state.
			 * PIX uses WM0 (Y) + WM4 (CbCr), VIDEO WMs are separate.
			 */
			dev_info(vfe->camss->dev,
				 "VFE31: Step 8 - PIX only, disabling VIDEO WM%d\n",
				 VFE31_VIDEO_WM_Y);
			writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
			wmb();
		}
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
	 *
	 * IMPORTANT: Only save WM0's address to pending_ping_addr!
	 * For semi-planar formats (NV16), WM1's address is computed later
	 * as pending_ping_addr + Y_plane_size. If we save WM1's address
	 * here, it overwrites WM0's address and breaks the offset calculation.
	 */
	if (vfe->camif_pending) {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d ping_addr=0x%08x (deferred)\n", wm, addr);
		if (wm == 0)
			vfe->pending_ping_addr = addr;
	} else {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d ping_addr=0x%08x (direct write)\n", wm, addr);
		writel_relaxed(addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
	}

	/*
	 * NOTE: WM4/WM5 addresses are NOT mirrored from WM0/WM1.
	 *
	 * When VIDEO line (VFE_LINE_VIDEO) is active, it sets WM4/WM5
	 * addresses directly via wm=4/wm=5 calls with its own buffers.
	 *
	 * When VIDEO line is NOT active, WM4 uses a dummy buffer configured
	 * in enable_pending_camif() to safely absorb duplicate Y data from
	 * XBAR 0x1A1B routing.
	 *
	 * The old video_output_enable mirror code caused DMA corruption by
	 * making WM0 and WM4 write to the same addresses simultaneously.
	 */
}

static void vfe31_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	/*
	 * VFE31: If CAMIF is not yet running, defer the write.
	 * Once CAMIF is running (camif_pending=false), write directly.
	 *
	 * IMPORTANT: Only save WM0's address to pending_pong_addr!
	 * See comment in vfe31_wm_set_ping_addr for explanation.
	 */
	if (vfe->camif_pending) {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d pong_addr=0x%08x (deferred)\n", wm, addr);
		if (wm == 0)
			vfe->pending_pong_addr = addr;
	} else {
		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d pong_addr=0x%08x (direct write)\n", wm, addr);
		writel_relaxed(addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
	}

	/* See comment in vfe31_wm_set_ping_addr about WM4/WM5 handling */
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
	 * Using webOS offset-by-4 WM pairing:
	 *   - PIX only:    0x11 = WM0+WM4
	 *   - VIDEO only:  0x22 = WM1+WM5
	 *   - PIX+VIDEO:   0x33 = WM0+WM1+WM4+WM5
	 *
	 * All WMs are in Group 0 (bits 0-7) for COMPOSITE_DONE_0.
	 */
	if (enable) {
		/* Choose mask based on which lines are active */
		struct vfe_output *video_out = &vfe->line[VFE_LINE_VIDEO].output;
		struct vfe_output *pix_out = &vfe->line[VFE_LINE_PIX].output;
		bool video_active = (video_out->state == VFE_OUTPUT_ON ||
				     video_out->state == VFE_OUTPUT_RESERVED ||
				     video_out->state == VFE_OUTPUT_CONTINUOUS);
		bool pix_active = (pix_out->state == VFE_OUTPUT_ON ||
				   pix_out->state == VFE_OUTPUT_RESERVED ||
				   pix_out->state == VFE_OUTPUT_CONTINUOUS);
		const char *mode_str;

		/* Module param override takes priority */
		if (vfe31_irq_comp_mask != 0) {
			vfe->irq_comp_mask_shadow = vfe31_irq_comp_mask;
			mode_str = "module param";
		} else if ((line_id == VFE_LINE_VIDEO || video_active) && !pix_active) {
			/* VIDEO-only: WM1 (Y) + WM5 (CbCr) */
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
			mode_str = "VIDEO only";
		} else if ((line_id == VFE_LINE_VIDEO || video_active) && pix_active) {
			/* PIX+VIDEO: both lines active */
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PIX_VIDEO;
			mode_str = "PIX+VIDEO";
		} else {
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PIX_ONLY;
			mode_str = "PIX only";
		}

		/* Enable COMPOSITE_DONE_2 if WM1 is in group 2 (VIDEO mode) */
		if (vfe->irq_comp_mask_shadow & 0x00FF0000)
			val0 |= VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);

		dev_info(vfe->camss->dev,
			 "VFE31 pix_line: IRQ_COMPOSITE_MASK=0x%08x (%s)\n",
			 vfe->irq_comp_mask_shadow, mode_str);
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

/*
 * vfe31_force_enable_axi_clock - Force enable VFE_AXI_CLK if not running
 * @dev: Device for logging
 *
 * The VFE requires the AXI clock for DMA operations. If the clock framework
 * hasn't enabled it properly, we force it on via direct register write.
 * Used primarily for testgen mode.
 */
static void vfe31_force_enable_axi_clock(struct device *dev)
{
	void __iomem *mmcc_base;
	u32 vfe_cc_reg;

	mmcc_base = ioremap(MSM8660_MMCC_BASE, MSM8660_MMCC_SIZE);
	if (!mmcc_base) {
		dev_err(dev, "VFE AXI: Failed to map MMCC\n");
		return;
	}

	vfe_cc_reg = readl_relaxed(mmcc_base + MSM8660_VFE_CC_REG_OFFSET);
	dev_dbg(dev, "VFE AXI: VFE_CC_REG before = 0x%08x\n", vfe_cc_reg);

	if (!(vfe_cc_reg & MSM8660_VFE_CC_REG_VFE_AXI_EN)) {
		dev_warn(dev, "VFE AXI: VFE_AXI_CLK not enabled, forcing on\n");
		vfe_cc_reg |= MSM8660_VFE_CC_REG_VFE_AXI_EN;
		writel_relaxed(vfe_cc_reg, mmcc_base + MSM8660_VFE_CC_REG_OFFSET);
		wmb();
	}

	iounmap(mmcc_base);
}

/*
 * vfe31_configure_testgen - Configure VFE31 internal test generator
 * @vfe: VFE device
 * @enable: true to enable, false to disable
 * @width: test pattern width in pixels
 * @height: test pattern height in lines
 *
 * The VFE31 has an internal test pattern generator that can produce
 * color bar patterns without requiring external camera input.
 *
 * The testgen bypasses CSIPHY/CSID and feeds data directly to CAMIF.
 * WM registers must already be configured by vfe31_enable() before
 * calling this function.
 */
void vfe31_configure_testgen(struct vfe_device *vfe, bool enable,
			     u16 width, u16 height)
{
	u32 cfg_val;
	u32 width_bytes = width * 2;  /* UYVY format: 2 bytes per pixel */

	dev_info(vfe->camss->dev, "VFE TESTGEN: %s test generator (%dx%d)\n",
		 enable ? "Enabling" : "Disabling", width, height);

	if (enable) {
		/* Force enable VFE_AXI_CLK if not running */
		vfe31_force_enable_axi_clock(vfe->camss->dev);

		/*
		 * Step 1: Configure DEMUX for YUV processing
		 * The DEMUX separates Y and CbCr for semi-planar output.
		 * Configure gains for unity (0x80 = 1.0x).
		 */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Configuring DEMUX\n");
		writel_relaxed(VFE_0_DEMUX_CFG_PERIOD, vfe->base + VFE_0_DEMUX_CFG);
		writel_relaxed(VFE_0_DEMUX_GAIN_0_CH0_EVEN | VFE_0_DEMUX_GAIN_0_CH0_ODD,
			       vfe->base + VFE_0_DEMUX_GAIN_0);
		writel_relaxed(VFE_0_DEMUX_GAIN_1_CH1 | VFE_0_DEMUX_GAIN_1_CH2,
			       vfe->base + VFE_0_DEMUX_GAIN_1);
		/* UYVY (CbYCrY) demux pattern: even=0xc9, odd=0xac */
		writel_relaxed(0xc9, vfe->base + VFE_0_DEMUX_EVEN_CFG);
		writel_relaxed(0xac, vfe->base + VFE_0_DEMUX_ODD_CFG);
		wmb();

		/*
		 * Step 2: Configure test generator registers
		 * TESTGEN_DIMS: [15:0]=width_bytes, [31:16]=height
		 * The testgen produces UYVY data so width is in bytes.
		 */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Configuring dimensions %ux%u (bytes=%u)\n",
			 width, height, width_bytes);
		writel_relaxed(width_bytes | ((u32)height << 16),
			       vfe->base + VFE_0_TESTGEN_DIMS);
		writel_relaxed(0, vfe->base + VFE_0_TESTGEN_START_PIXEL);

		/* Set random seeds for pattern variation */
		writel_relaxed(0xDEADBEEF, vfe->base + VFE_0_TESTGEN_SEED_0);
		writel_relaxed(0xCAFEBABE, vfe->base + VFE_0_TESTGEN_SEED_1);
		writel_relaxed(0x12345678, vfe->base + VFE_0_TESTGEN_SEED_2);
		writel_relaxed(0x87654321, vfe->base + VFE_0_TESTGEN_SEED_3);
		wmb();

		/*
		 * Step 3: Configure CAMIF for testgen input
		 * CAMIF expects frame dimensions in the same format as camera input.
		 */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Configuring CAMIF\n");
		writel_relaxed(0x40, vfe->base + VFE_0_CAMIF_EFS_CFG);
		writel_relaxed((height << 16) | (width_bytes & 0x3FFF),
			       vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);
		writel_relaxed((width_bytes - 1) & 0x3FFF,
			       vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);
		writel_relaxed(height - 1, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);
		writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_1);
		wmb();

		/*
		 * Step 4: Enable VFE pipeline modules and configure AXI/XBAR
		 */
		writel_relaxed(0x01c00c0c, vfe->base + VFE_0_MODULE_CFG);
		writel_relaxed(VFE_0_BUS_XBAR_CFG0_PIX_MODE,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		writel_relaxed((vfe31_xbar_cfg1 != 0) ? vfe31_xbar_cfg1 : VFE31_XBAR_PIX_ONLY,
			       vfe->base + VFE_0_BUS_XBAR_CFG1);
		wmb();

		/*
		 * Step 4b: Configure BUS_CFG and reload write masters
		 * This must be done for WMs to properly DMA data to memory.
		 */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Configuring BUS_CFG and reloading WMs\n");
		writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(0x3FFF, vfe->base + VFE_0_BUS_CMD);
		wmb();

		/*
		 * Step 4c: Explicitly enable WMs (WM0 for Y, WM4 for CbCr)
		 * The WM configuration was done in vfe31_enable() but the enable
		 * bit may have been cleared by BUS_CMD reload.
		 */
		writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(0));
		writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(4));
		dev_info(vfe->camss->dev, "VFE TESTGEN: Enabled WM0 and WM4\n");
		wmb();

		/*
		 * Step 5: Configure IRQ masks
		 * Use same masks as PIX mode: SOF, REG_UPDATE, PING_PONG, COMPOSITE_DONE
		 */
		vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
				       VFE_0_IRQ_MASK_0_CAMIF_EOF |
				       VFE_0_IRQ_MASK_0_REG_UPDATE |
				       VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(0) |
				       VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(1) |
				       VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(0);
		vfe->irq_mask1_shadow = 0;
		vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_PIX_ONLY;

		writel_relaxed(vfe->irq_mask0_shadow, vfe->base + VFE_0_IRQ_MASK_0);
		writel_relaxed(vfe->irq_mask1_shadow, vfe->base + VFE_0_IRQ_MASK_1);
		writel_relaxed(vfe->irq_comp_mask_shadow, vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
		wmb();

		/*
		 * Step 6: Set CORE_CFG to use testgen as input source
		 * This must be done AFTER DEMUX/CAMIF are configured.
		 */
		cfg_val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY |
			  VFE_0_CORE_CFG_INPUT_MUX_TESTGEN |
			  VFE_0_CORE_CFG_INPUT_MUX_ENABLE;
		dev_info(vfe->camss->dev, "VFE TESTGEN: CORE_CFG=0x%02x (testgen input)\n", cfg_val);
		writel_relaxed(cfg_val, vfe->base + VFE_0_CORE_CFG);
		wmb();

		/*
		 * Step 7: Enable testgen and start CAMIF
		 * Order: enable testgen -> reg update -> start CAMIF
		 */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Enabling pattern generator\n");
		writel_relaxed(VFE_0_TESTGEN_CFG_ENABLE | VFE_0_TESTGEN_CFG_COLORBAR,
			       vfe->base + VFE_0_TESTGEN_CFG);
		wmb();

		/* Issue REG_UPDATE to latch all configuration */
		writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);
		wmb();

		/* Small delay to allow testgen to start generating */
		udelay(100);

		/* Start CAMIF to begin capturing from testgen */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Starting CAMIF\n");
		writel_relaxed(VFE_0_CAMIF_CMD_START, vfe->base + VFE_0_CAMIF_CMD);
		wmb();

		dev_info(vfe->camss->dev, "VFE TESTGEN: Started successfully\n");
	} else {
		/* Stop test generator and CAMIF */
		writel_relaxed(VFE_0_CAMIF_CMD_STOP_AT_FRAME_BOUNDARY,
			       vfe->base + VFE_0_CAMIF_CMD);
		wmb();
		writel_relaxed(0, vfe->base + VFE_0_TESTGEN_CFG);
		wmb();
		dev_info(vfe->camss->dev, "VFE TESTGEN: Stopped\n");
	}
}
EXPORT_SYMBOL(vfe31_configure_testgen);

/*
 * vfe31_enable_pending_camif - Enable CAMIF that was deferred during VFE s_stream
 *
 * This is called from CSID when CSI is fully configured and ready to provide
 * data to VFE. At this point we can complete the VFE CAMIF configuration and
 * start streaming.
 *
 * This function replaces the VFE31-specific fallback code that was previously
 * in camss-vfe.c:vfe_enable_pending_camif().
 */
static void vfe31_enable_pending_camif(struct vfe_device *vfe)
{
	struct vfe_line *line;
	u32 val;
	u32 width_bytes, height;

	if (!vfe->camif_pending) {
		dev_dbg(vfe->camss->dev, "VFE31: no pending CAMIF config\n");
		return;
	}

	line = &vfe->line[vfe->camif_pending_line_id];
	width_bytes = line->fmt[MSM_VFE_PAD_SINK].width * 2;  /* YUV422: 2 bytes/pixel */
	height = line->fmt[MSM_VFE_PAD_SINK].height;

	dev_info(vfe->camss->dev,
		 "VFE31 enable_pending_camif: line=%d %ux%u stride=%u\n",
		 vfe->camif_pending_line_id,
		 line->fmt[MSM_VFE_PAD_SINK].width, height, width_bytes);

	/*
	 * Step 1: Force all VFE internal clocks on via CGC_OVERRIDE
	 */
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CGC_OVERRIDE);
	wmb();

	/*
	 * Step 2: Configure MODULE_CFG
	 * PIX mode: Enable DEMUX and processing modules (0x01c00c0c)
	 * RDI mode: Disable all modules (0) - data bypasses ISP
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (is_rdi) {
			dev_info(vfe->camss->dev,
				 "VFE31: MODULE_CFG=0x0 (RDI raw bypass)\n");
			writel_relaxed(0, vfe->base + VFE_0_MODULE_CFG);
		} else {
			dev_info(vfe->camss->dev,
				 "VFE31: MODULE_CFG=0x01c00c0c (PIX with DEMUX)\n");
			writel_relaxed(0x01c00c0c, vfe->base + VFE_0_MODULE_CFG);
		}
	}
	wmb();

	/*
	 * Step 3: Configure CORE_CFG with pixel pattern + input mux enable
	 * webOS uses 0x46 for UYVY: pixel pattern 0x6 + bit 6 (input mux)
	 * For RDI, we still need input mux enabled but no pixel pattern.
	 */
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
	val |= VFE_0_CORE_CFG_INPUT_MUX_ENABLE;
	writel_relaxed(val, vfe->base + VFE_0_CORE_CFG);

	/*
	 * Step 4: Configure DEMUX gains (PIX mode only)
	 * RDI mode bypasses DEMUX, so no gain configuration needed.
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (!is_rdi) {
			writel_relaxed(0x800080, vfe->base + VFE_0_DEMUX_GAIN_0);
			writel_relaxed(0x800080, vfe->base + VFE_0_DEMUX_GAIN_1);
		}
	}

	/*
	 * Step 5: Configure clamp values for output
	 */
	writel_relaxed(0x00ffffff, vfe->base + VFE_0_CLAMP_ENC_MAX_CFG);
	writel_relaxed(0x0, vfe->base + VFE_0_CLAMP_ENC_MIN_CFG);

	/*
	 * Step 6: Configure CAMIF registers
	 *
	 * EFS_CFG at 0x1E4: webOS uses 0x40 (bit 6 set)
	 * FRAME_CFG at 0x1E8: webOS leaves at 0 (not used)
	 * WINDOW_WIDTH_CFG at 0x1EC: (height << 16) | width_bytes
	 * WINDOW_HEIGHT_CFG at 0x1F0: width_bytes - 1
	 * SUBSAMPLE_CFG_0 at 0x1F4: height - 1
	 * SUBSAMPLE_CFG_1 at 0x1F8: 0xFFFFFFFF (no frame skip)
	 */
	writel_relaxed(0x40, vfe->base + VFE_0_CAMIF_EFS_CFG);
	writel_relaxed(0, vfe->base + VFE_0_CAMIF_FRAME_CFG);

	val = (height << 16) | (width_bytes & 0xFFFF);
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);
	dev_info(vfe->camss->dev,
		 "VFE31: WINDOW_WIDTH=0x%08x (height=%u, width_bytes=%u)\n",
		 val, height, width_bytes);

	val = width_bytes - 1;
	writel_relaxed(val, vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);

	writel_relaxed(height - 1, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_1);

	/*
	 * Step 7: Configure AXI output mode and XBAR routing
	 *
	 * RDI lines (RDI0, RDI1, RDI2) use raw bypass mode (0x60) which
	 * routes data directly from CAMIF to write master, bypassing DEMUX.
	 * PIX/VIDEO lines use the module parameter (typically 0x01 for DEMUX).
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);
		u32 axi_mode = is_rdi ? VFE_0_BUS_AXI_OUT_MODE_RAW_WM0 :
					vfe31_axi_output_mode;

		dev_info(vfe->camss->dev,
			 "VFE31: AXI_OUT_MODE=0x%x (%s)\n",
			 axi_mode, is_rdi ? "RDI raw bypass" : "PIX/DEMUX");
		writel_relaxed(axi_mode, vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);

		/* Only configure XBAR for PIX mode - RDI bypasses XBAR */
		if (!is_rdi && axi_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
			struct vfe_output *video_out = &vfe->line[VFE_LINE_VIDEO].output;
			bool video_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			u32 xbar_val;

			/*
			 * Auto-select XBAR based on active lines:
			 *   - PIX only:  0x1A03 (webOS value, offset-by-4 pairing)
			 *   - PIX+VIDEO: 0x1A1B (route Y to WM0+WM1)
			 *
			 * With offset-by-4 pairing:
			 *   - PIX uses WM0 (Y) + WM4 (CbCr)
			 *   - VIDEO uses WM1 (Y) + WM5 (CbCr)
			 */
			if (vfe31_xbar_cfg1 != 0) {
				xbar_val = vfe31_xbar_cfg1;
			} else if (line->id == VFE_LINE_VIDEO || video_active) {
				/* VIDEO line active - enable Y routing to WM1 */
				xbar_val = VFE31_XBAR_PIX_VIDEO;
			} else {
				/* PIX only - webOS default */
				xbar_val = VFE31_XBAR_PIX_ONLY;
			}
			dev_info(vfe->camss->dev,
				 "VFE31: XBAR=0x%04x (%s)\n", xbar_val,
				 xbar_val == VFE31_XBAR_PIX_ONLY ? "PIX only" :
				 xbar_val == VFE31_XBAR_PIX_VIDEO ? "PIX+VIDEO" : "manual");
			writel_relaxed(xbar_val, vfe->base + VFE_0_BUS_XBAR_CFG1);
		}
	}

	/*
	 * Step 8: Configure BUS_CFG for DMA write paths
	 */
	writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);

	/*
	 * Step 9: Reload all write masters via BUS_CMD
	 */
	writel_relaxed(0x3FFF, vfe->base + VFE_0_BUS_CMD);
	wmb();

	/*
	 * Step 10: Configure IRQ masks
	 * PIX mode: webOS values (0x00EFE021) with composite interrupts
	 * RDI mode: Use IMAGE_COMPOSITE_DONE_1 for frame completion
	 *
	 * VFE31 IRQ_STATUS_0 layout (from downstream):
	 *   Bit 0: CAMIF_SOF
	 *   Bit 5: REG_UPDATE
	 *   Bits 13-18: Stats interrupts
	 *   Bit 21: IMAGE_COMPOSITE_DONE_0 (composite group 0)
	 *   Bit 22: IMAGE_COMPOSITE_DONE_1 (composite group 1)
	 *   Bit 23: IMAGE_COMPOSITE_DONE_2 (composite group 2)
	 *
	 * IMPORTANT: Bits 8+ are NOT per-WM ping/pong interrupts in VFE31!
	 * WM completion must use COMPOSITE_DONE interrupts.
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (is_rdi) {
			/*
			 * RDI mode: Map WM0 to composite group 1, use COMPOSITE_DONE_1.
			 * This follows downstream raw snapshot configuration:
			 * - IRQ_COMP_MASK bit 8 = WM0 in composite group 1
			 * - When WM0 finishes, IMAGE_COMPOSITE_DONE_1 (bit 22) fires
			 */
			vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
						VFE_0_IRQ_MASK_0_REG_UPDATE |
						VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(1);
			dev_info(vfe->camss->dev,
				 "VFE31: RDI IRQ_MASK_0=0x%08x (COMPOSITE_DONE_1)\n",
				 vfe->irq_mask0_shadow);
		} else {
			/* PIX mode: Use webOS value with composite interrupts */
			struct vfe_output *video_out = &vfe->line[VFE_LINE_VIDEO].output;
			bool video_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool video_needs_cbcr = video_active && (video_out->wm_num == 2);

			vfe->irq_mask0_shadow = 0x00EFE021;

			/*
			 * VIDEO line with CbCr uses WM1 in group 2 (COMPOSITE_DONE_2).
			 * Add bit 23 to receive WM1 completion interrupts.
			 */
			if (line->id == VFE_LINE_VIDEO || video_needs_cbcr)
				vfe->irq_mask0_shadow |= VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);

			dev_info(vfe->camss->dev,
				 "VFE31: PIX IRQ_MASK_0=0x%08x (composite%s)\n",
				 vfe->irq_mask0_shadow,
				 (vfe->irq_mask0_shadow & 0x800000) ? "+DONE2" : "");
		}
		vfe->irq_mask1_shadow = 0x00400000;
		writel_relaxed(vfe->irq_mask0_shadow, vfe->base + VFE_0_IRQ_MASK_0);
		writel_relaxed(vfe->irq_mask1_shadow, vfe->base + VFE_0_IRQ_MASK_1);
	}

	/*
	 * Step 11: Configure composite IRQ mask for buffer completion
	 *
	 * IRQ_COMPOSITE_MASK (0x034) maps WMs to composite interrupt groups:
	 *   Bits 0-7:   WMs that trigger COMPOSITE_DONE_0 (IRQ bit 21)
	 *   Bits 8-15:  WMs that trigger COMPOSITE_DONE_1 (IRQ bit 22)
	 *   Bits 16-23: WMs that trigger COMPOSITE_DONE_2 (IRQ bit 23)
	 *
	 * Using offset-by-4 pairing (all in group 0):
	 *   PIX:   0x11 = WM0+WM4
	 *   VIDEO: 0x22 = WM1+WM5
	 *   Both:  0x33 = WM0+WM1+WM4+WM5
	 * RDI mode: WM0 in group 1 (bit 8 = 0x100)
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (is_rdi) {
			/*
			 * RDI: Map WM0 to composite group 1 (bit 8).
			 * When WM0 completes a frame, COMPOSITE_DONE_1 fires.
			 */
			u32 comp_mask = (1 << (vfe->camif_pending_wm + 8));
			dev_info(vfe->camss->dev,
				 "VFE31: RDI COMPOSITE_MASK=0x%08x (WM%d->group1)\n",
				 comp_mask, vfe->camif_pending_wm);
			writel_relaxed(comp_mask, vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
		} else {
			/* PIX/VIDEO mode - check which lines are active */
			struct vfe_output *video_out = &vfe->line[VFE_LINE_VIDEO].output;
			struct vfe_output *pix_out = &vfe->line[VFE_LINE_PIX].output;
			bool video_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool pix_active = (pix_out->state == VFE_OUTPUT_ON ||
					   pix_out->state == VFE_OUTPUT_RESERVED ||
					   pix_out->state == VFE_OUTPUT_CONTINUOUS);
			u32 comp_mask;

			/* Module param override takes priority */
			if (vfe31_irq_comp_mask != 0) {
				comp_mask = vfe31_irq_comp_mask;
			} else if ((line->id == VFE_LINE_VIDEO || video_active) && !pix_active) {
				/* VIDEO-only: WM1 (Y) + WM5 (CbCr) */
				comp_mask = VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
			} else if ((line->id == VFE_LINE_VIDEO || video_active) && pix_active) {
				/* PIX+VIDEO: both lines active */
				comp_mask = VFE31_IRQ_COMP_MASK_PIX_VIDEO;
			} else {
				/* PIX only */
				comp_mask = VFE31_IRQ_COMP_MASK_PIX_ONLY;
			}

			vfe->irq_comp_mask_shadow = comp_mask;
			dev_info(vfe->camss->dev,
				 "VFE31 enable_camif: IRQ_COMPOSITE_MASK=0x%08x\n", comp_mask);
			writel_relaxed(comp_mask, vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
		}
	}
	wmb();

	/*
	 * Step 11b: Enable Write Masters
	 *
	 * CRITICAL: Explicitly enable WMs here before CAMIF start.
	 * The WMs were configured in vfe31_enable() but the enable bit
	 * may be cleared by BUS_CMD reload or other operations.
	 *
	 * For PIX mode: Enable WM0 (Y) and WM4 (CbCr) - offset-by-4 pairing
	 * For RDI mode: Enable only WM0 (raw)
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		/* Always enable WM0 */
		writel_relaxed(BIT(0),
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(vfe->camif_pending_wm));
		dev_info(vfe->camss->dev,
			 "VFE31: Enabled WM%d (WR_CFG=0x1)\n", vfe->camif_pending_wm);

		/* For PIX mode, also enable CbCr WM (WM4 with offset-by-4 pairing) */
		if (!is_rdi && line->output.wm_num == 2) {
			u8 cbcr_wm = line->output.wm_idx[1];
			writel_relaxed(BIT(0),
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(cbcr_wm));
			dev_info(vfe->camss->dev,
				 "VFE31: Enabled WM%d (CbCr, WR_CFG=0x1)\n", cbcr_wm);
		}
		wmb();
	}

	/*
	 * Step 12: Issue REG_UPDATE command
	 * This latches all the shadow register values on the next VSYNC.
	 */
	writel(1, vfe->base + VFE_0_REG_UPDATE_CMD);
	wmb();

	/*
	 * Step 13: Start CAMIF
	 * Write 1 to CAMIF_CMD (webOS vfe31_start_common writes 1, not 0x5)
	 */
	writel(VFE_0_CAMIF_CMD_START, vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	vfe->camif_pending = false;

	dev_info(vfe->camss->dev,
		 "VFE31: CAMIF started - status=0x%08x axi=0x%08x xbar=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS),
		 readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG),
		 readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG1));
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

/*
 * vfe31_cleanup - Clean up VFE31 state during vfe_put
 *
 * This is called when powering down the VFE. It ensures CAMIF is properly
 * stopped and the pending flag is cleared. Without this, stale CAMIF state
 * can block CSIPHY register access on subsequent camera sessions.
 *
 * NOTE: The WM4 dummy buffer (vfe31_wm4_dummy_*) is NOT freed here because
 * it's a static one-time allocation that persists for the module lifetime.
 * This avoids repeated allocation/free cycles and the buffer is small (~1.3MB).
 * It will be freed automatically when the module is unloaded.
 */
static void vfe31_cleanup(struct vfe_device *vfe)
{
	/* Stop CAMIF and clear EFS config */
	writel_relaxed(0, vfe->base + VFE_0_CAMIF_CMD);
	writel_relaxed(0, vfe->base + VFE_0_CAMIF_EFS_CFG);
	vfe->camif_pending = false;

	/*
	 * Disable VIDEO line WMs to ensure they don't continue writing on next session.
	 * This prevents stale DMA activity if the previous capture was aborted.
	 * Note: WM1 (CbCr) is shared with PIX but safe to disable during full cleanup.
	 */
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_CBCR));
}

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
	.enable_pending_camif = vfe31_enable_pending_camif,
	.vfe_cleanup = vfe31_cleanup,
};
