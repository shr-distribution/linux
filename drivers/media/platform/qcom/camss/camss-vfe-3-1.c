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
int vfe31_video_output_enable = 1;
module_param(vfe31_video_output_enable, int, 0644);
MODULE_PARM_DESC(vfe31_video_output_enable,
		 "VFE31 video output enable (0=off, 1=on with WM4/WM5)");

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

#define VFE_0_IRQ_MASK_0		0x01C
#define VFE_0_IRQ_MASK_0_CAMIF_SOF			BIT(0)
#define VFE_0_IRQ_MASK_0_CAMIF_EOF			BIT(1)
#define VFE_0_IRQ_MASK_0_EPOCH_IRQ_0			BIT(2)
#define VFE_0_IRQ_MASK_0_EPOCH_IRQ_1			BIT(3)
#define VFE_0_IRQ_MASK_0_EPOCH_IRQ_2			BIT(4)
#define VFE_0_IRQ_MASK_0_REG_UPDATE			BIT(5)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 21)
#define VFE_0_IRQ_MASK_0_STATS_COMPOSITE		BIT(24)
#define VFE_0_IRQ_MASK_0_RESET_ACK			BIT(31)
#define VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? BIT(5) : 0)

#define VFE_0_IRQ_MASK_1		0x020
#define VFE_0_IRQ_MASK_1_CAMIF_ERROR			BIT(0)
#define VFE_0_IRQ_MASK_1_VIOLATION			BIT(6)
/*
 * VFE31 reset acknowledge is in STATUS_1 bit 22, not STATUS_0 bit 31.
 * This differs from later VFE versions.
 */
#define VFE_0_IRQ_MASK_1_RESET_ACK			BIT(22)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		BIT(23)
/*
 * VFE31 IRQ_MASK_1 bit layout per webOS vfe31.h:
 * Bit 6: VIOLATION
 * Bit 7-13: IMAGE_MASTER_0-6_BUS_OVERFLOW
 * Our VIOLATION define is at BIT(7) which conflicts - needs review
 */
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	BIT((n) + 7)

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
 * Use the exact webOS value 0x02AAA771 for reliable operation.
 */
#define VFE_0_BUS_CFG_WEBOS_VALUE		0x02AAA771
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
 * VFE31 AXI output mode and XBAR configuration registers
 * These configure which write masters are used and how data is routed.
 *
 * The webOS driver builds a 188-byte AXI config block starting at 0x38.
 * Key registers from that block:
 *   0x40 (ao[2]) = XBAR CFG0 / AXI output mode
 *   0x44 (ao[3]) = XBAR CFG1
 *
 * Values from legacy webOS driver:
 *   For raw mode (CAMIF_TO_AXI_VIA_OUTPUT_2):
 *     0x40 = 0x60 (raw bypass via WM0)
 *     0x44 = 0 (not configured)
 *
 *   For PIX/preview mode (OUTPUT_2):
 *     0x40 = 0x01 (xbar cfg0 - enable crossbar)
 *     0x44 = 0x1a03 (xbar cfg1 - route CAMIF to WM0/WM1)
 */
#define VFE_0_BUS_AXI_OUT_MODE_CFG		0x040
#define VFE_0_BUS_XBAR_CFG1			0x044
#define VFE_0_BUS_AXI_OUT_MODE_RAW_WM0		0x60
#define VFE_0_BUS_XBAR_CFG0_PIX_MODE		0x01
#define VFE_0_BUS_XBAR_CFG1_PIX_MODE		0x1a03
/*
 * VFE31 Video mode XBAR configuration:
 *   0x1a1b = preview + video mode (WM0/WM1 for preview, WM4/WM5 for video)
 *
 * XBAR CFG1 bit layout (per webOS msm_vfe31.c):
 *   Bits 0-3:   Y output XBAR routing (WM0 for preview, WM4 for video)
 *   Bits 4-7:   CbCr output XBAR routing (WM1 for preview, WM5 for video)
 *   Bits 8-15:  Additional routing configuration
 *
 * webOS values:
 *   0x1a03 = preview only (WM0/WM1)
 *   0x1a1b = preview + video (WM0/WM1 + WM4/WM5)
 */
#define VFE_0_BUS_XBAR_CFG1_VIDEO_MODE		0x1a1b
#define VFE_0_BUS_CFG_RAW_WR_PATH_VIEW_CBCR	(2 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)

/*
 * Video mode write master assignments:
 * - Preview: WM0 (Y) + WM1 (CbCr) → COMPOSITE_DONE_0
 * - Snapshot: WM0 (or dedicated WM) → COMPOSITE_DONE_1
 * - Video:   WM4 (Y) + WM5 (CbCr) → COMPOSITE_DONE_2
 */
#define VFE31_VIDEO_WM_Y		4
#define VFE31_VIDEO_WM_CBCR		5

/*
 * VFE31 Video Mode Configuration Summary
 * ======================================
 *
 * Video mode enables simultaneous preview and video recording using separate
 * output paths through the VFE31 bus infrastructure.
 *
 * Output Paths (per webOS msm_vfe31.c vfe31_config_axi):
 * - out0 (PT/Preview): WM0 (Y) + WM1 (CbCr) → COMPOSITE_DONE_0 (IRQ bit 21)
 * - out1 (S/Snapshot): WM (configurable) → COMPOSITE_DONE_1 (IRQ bit 22)
 * - out2 (V/Video):    WM4 (Y) + WM5 (CbCr) → COMPOSITE_DONE_2 (IRQ bit 23)
 *
 * Configuration Steps:
 * 1. Set vfe31_video_output_enable=1 (module parameter)
 * 2. Set vfe31_axi_output_mode=0x01 (preview+video mode)
 * 3. XBAR CFG1 automatically uses 0x1a1b to route to WM0/1 + WM4/5
 * 4. IRQ_COMPOSITE_MASK includes WM4/5 → COMPOSITE_DONE_2 mapping
 * 5. Configure WM4/WM5 with video buffer addresses (requires userspace)
 *
 * Note: Full video recording requires userspace to provide separate video
 * buffers and configure WM4/WM5 addresses. This infrastructure enables
 * the hardware support; actual video capture needs V4L2 multi-planar setup.
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

#define VFE_0_BUS_PING_PONG_STATUS	0x180

/*
 * Bus image masters - VFE31 layout (different from VFE41!)
 *
 * VFE31 AXI output block starts at 0x38, with write masters at 0x4C.
 * Each WM block is 0x18 (24) bytes with 6 registers.
 *
 * From webOS kernel msm_vfe31.c:
 *   #define VFE31_AXI_OFFSET 0x0050
 *   vfe31_get_ch_ping_addr(chn) = 0x0050 + 0x18 * (chn)
 *   vfe31_get_ch_pong_addr(chn) = 0x0050 + 0x18 * (chn) + 4
 *   WM enable at V31_AXI_OUT_OFF + 20 + 24*wm = 0x38 + 0x14 + 0x18*wm = 0x4C + 0x18*wm
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
	output->wm_num = 1;  /* Raw mode uses single WM */
	output->drop_update_idx = 0;

	wm_idx = vfe_reserve_wm(vfe, line->id);
	if (wm_idx < 0) {
		dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM\n");
		output->state = VFE_OUTPUT_OFF;
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return wm_idx;
	}
	output->wm_idx[0] = wm_idx;

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
	bytesperline = pix->plane_fmt[0].bytesperline;

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
	 * For PIX mode, we also need XBAR CFG1 at 0x44 = 0x1a03
	 * This configures the crossbar to route CAMIF data to WM0/WM1.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 1 - BUS_CFG=0x%08x, AXI=0x%x\n",
		 VFE_0_BUS_CFG_WEBOS_VALUE, vfe31_axi_output_mode);
	writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
	writel_relaxed(vfe31_axi_output_mode,
		       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);

	/*
	 * For PIX mode (0x01), configure XBAR CFG1 to route data to WMs.
	 * Use video mode XBAR (0x1a1b) if video output is enabled,
	 * otherwise use preview-only XBAR (0x1a03).
	 */
	if (vfe31_axi_output_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		u32 xbar_cfg1 = vfe31_video_output_enable ?
				VFE_0_BUS_XBAR_CFG1_VIDEO_MODE :
				VFE_0_BUS_XBAR_CFG1_PIX_MODE;
		dev_info(vfe->camss->dev,
			 "VFE31: PIX mode - XBAR CFG1=0x%x (video=%d)\n",
			 xbar_cfg1, vfe31_video_output_enable);
		writel_relaxed(xbar_cfg1, vfe->base + VFE_0_BUS_XBAR_CFG1);
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
	 */
	wpl = bytesperline / 4;  /* 32-bit words per line */
	reg = (wpl - 1) & 0xFFFF;  /* burst = words_per_line - 1 */
	/* For single-plane formats, lines=0. Multi-plane would add (height << 16) */
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	/*
	 * WR_UB_CFG - VFE31 format (from webOS register dumps):
	 * webOS WM0: 0x002701DF = ((39) << 16) | 479
	 *
	 * Upper 16 bits: (wpl / 8) + 1, where wpl is 32-bit words per line
	 * Lower 16 bits: height - 1
	 *
	 * This is DIFFERENT from VFE4.x which uses (offset << 16) | depth
	 */
	wpl = bytesperline / 4;  /* 32-bit words per line */
	reg = ((wpl / 8 + 1) & 0xFFFF) << 16;
	reg |= (height - 1) & 0xFFFF;
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));

	/*
	 * WR_CFG - enable only (BIT(0)).
	 * Note: VFE31 does NOT have frame_based mode in WR_CFG bit 1.
	 * webOS only writes 1 here, not 3. Hardware ignores bit 1.
	 */
	writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
	wmb();

	/* Reload WM0 to apply new configuration */
	dev_info(vfe->camss->dev, "VFE31: Reloading WM%d (BUS_CMD)\n", wm);
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm), vfe->base + VFE_0_BUS_CMD);
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

	/* Set output state - actual streaming starts after CAMIF config */
	output->state = VFE_OUTPUT_IDLE;
	output->sequence = 0;
	output->gen1.active_buf = 0;

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
	u32 p = line->video_out.active_fmt.fmt.pix_mp.pixelformat;
	u32 reg;
	u16 first, last;

	/*
	 * VFE31 crop registers use first/last pixel format:
	 * CROP_Y_WIDTH:   (first_pixel << 16) | last_pixel
	 * CROP_Y_HEIGHT:  (first_line << 16) | last_line
	 *
	 * line->crop contains the crop rectangle from pad selection.
	 */
	first = line->crop.left;
	last = line->crop.left + line->crop.width - 1;
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_Y_WIDTH);
	dev_dbg(vfe->camss->dev, "VFE31 CROP: Y_WIDTH=0x%08x (first=%d last=%d)\n",
		reg, first, last);

	first = line->crop.top;
	last = line->crop.top + line->crop.height - 1;
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_Y_HEIGHT);
	dev_dbg(vfe->camss->dev, "VFE31 CROP: Y_HEIGHT=0x%08x (first=%d last=%d)\n",
		reg, first, last);

	/* CbCr is half width for YUV422/420 */
	first = line->crop.left / 2;
	last = line->crop.left / 2 + line->crop.width / 2 - 1;
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_CBCR_WIDTH);

	first = line->crop.top;
	last = line->crop.top + line->crop.height - 1;
	if (p == V4L2_PIX_FMT_NV12 || p == V4L2_PIX_FMT_NV21) {
		first = line->crop.top / 2;
		last = line->crop.top / 2 + line->crop.height / 2 - 1;
	}
	reg = (first << 16) | last;
	writel_relaxed(reg, vfe->base + VFE_0_CROP_ENC_CBCR_HEIGHT);
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
	/* VFE31 XBAR configuration */
	u32 val;

	if (output->wm_num == 1) {
		val = (output->wm_idx[0] << 8);
		writel_relaxed(val, vfe->base +
			       VFE_0_BUS_XBAR_CFG_x(output->wm_idx[0]));
	}
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
static void __maybe_unused vfe31_configure_video_wm(struct vfe_device *vfe,
						    u32 y_addr, u32 cbcr_addr,
						    u16 width, u16 height, u16 stride)
{
	u16 wpl;  /* words per line */
	u32 reg;

	if (!vfe31_video_output_enable) {
		dev_dbg(vfe->camss->dev,
			"VFE31: video output disabled, skipping WM4/5 config\n");
		return;
	}

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
	 * WR_ADDR_CFG - VFE31 format:
	 * burst_words = words_per_line - 1
	 */
	reg = (wpl - 1) & 0xFFFF;
	writel_relaxed(reg,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_Y));

	/*
	 * WR_UB_CFG - VFE31 format:
	 * Upper 16 bits: (wpl / 8) + 1
	 * Lower 16 bits: height - 1
	 */
	reg = ((wpl / 8 + 1) & 0xFFFF) << 16;
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

		reg = (wpl - 1) & 0xFFFF;
		writel_relaxed(reg,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_CBCR));

		reg = ((wpl / 8 + 1) & 0xFFFF) << 16;
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
	 * CRITICAL: WebOS register dumps show that ALL modes (preview, video,
	 * photo capture) use AXI_OUT_MODE=0x01 and XBAR_CFG1=0x1a1b.
	 * The 0x60 "raw bypass" mode we tried doesn't work - it's not used
	 * by webOS and doesn't properly route data to the write masters.
	 *
	 * CRITICAL: BUS_CFG at 0x03C must be set to 0x02AAA771 per webOS dumps.
	 * This value enables the write paths (bits 4-6) required for DMA:
	 *   - Bit 4: encYWrPathEn
	 *   - Bit 5: encCbcrWrPathEn
	 *   - Bit 6: viewYWrPathEn
	 * Without this, DMA writes don't complete and ping_pong never toggles.
	 *
	 * Force AXI mode 0x01 and XBAR 0x1a1b regardless of module parameter.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 1 - BUS_CFG=0x%08x, AXI=0x01, XBAR=0x1a1b\n",
		 VFE_0_BUS_CFG_WEBOS_VALUE);
	writel_relaxed(VFE_0_BUS_CFG_WEBOS_VALUE, vfe->base + VFE_0_BUS_CFG);
	writel_relaxed(VFE_0_BUS_XBAR_CFG0_PIX_MODE,
		       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
	writel_relaxed(VFE_0_BUS_XBAR_CFG1_VIDEO_MODE,
		       vfe->base + VFE_0_BUS_XBAR_CFG1);
	wmb();

	/* Step 2: Configure WM registers (must be BEFORE CAMIF start) */
	{
		struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
		u16 width = pix->width;
		u16 height = pix->height;
		u16 bytesperline = pix->plane_fmt[0].bytesperline;
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
		 * burst_words = words_per_line - 1
		 * NOTE: wpl is in 32-bit words. bytesperline is already in bytes.
		 */
		wpl = bytesperline / 4;  /* 32-bit words per line */
		reg = (wpl - 1) & 0xFFFF;  /* burst = words_per_line - 1 */
		/* For single-plane formats, lines=0. Multi-plane would add (height << 16) */

		dev_info(vfe->camss->dev,
			 "VFE31: WM%d ADDR_CFG stride=%d wpl=%d burst=%d reg=0x%x\n",
			 wm, bytesperline, wpl, wpl - 1, reg);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

		/*
		 * WR_UB_CFG - VFE31 format (from webOS register dumps):
		 * webOS WM0: 0x002701DF = ((39) << 16) | 479
		 *
		 * Upper 16 bits: (wpl / 8) + 1, where wpl is 32-bit words per line
		 * Lower 16 bits: height - 1
		 */
		wpl = bytesperline / 4;  /* 32-bit words per line */
		reg = ((wpl / 8 + 1) & 0xFFFF) << 16;
		reg |= (height - 1) & 0xFFFF;
		dev_info(vfe->camss->dev,
			 "VFE31: WM%d UB_CFG wpl=%d height=%d reg=0x%x (webOS format)\n",
			 wm, wpl, height, reg);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
		wmb();
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
		 * WebOS video mode uses 0x00220011 which maps:
		 * - WM0 + WM4 -> COMPOSITE_DONE_0 (bits 0,4)
		 * - WM1 + WM5 -> COMPOSITE_DONE_2 (bits 17,21)
		 *
		 * For our dynamic configuration, map the active WM to COMP0.
		 * This follows webOS pattern where WM0 -> COMP0 (bit 0).
		 */
		vfe->irq_comp_mask_shadow |= BIT(wm);  /* Map WM to COMP0 */
		dev_info(vfe->camss->dev,
			 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (WM%d -> COMP0)\n",
			 vfe->irq_comp_mask_shadow, wm);
		writel_relaxed(vfe->irq_comp_mask_shadow,
			       vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
		wmb();
	}

	/*
	 * Configure IRQ masks dynamically based on active WM.
	 *
	 * WebOS uses IRQ_MASK_0 = 0x00EFE021 which includes:
	 *   - Bit 0: CAMIF_SOF
	 *   - Bit 5: REG_UPDATE
	 *   - Bits 9-13: PING_PONG for WM1-5
	 *   - Bits 21-23: IMAGE_COMPOSITE_DONE_0-2
	 *
	 * We build this dynamically to include PING_PONG for our active WM.
	 */
	vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
				VFE_0_IRQ_MASK_0_CAMIF_EOF |
				VFE_0_IRQ_MASK_0_REG_UPDATE |
				VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm) |
				VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(0) |
				VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(1) |
				VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);
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

	/* Step 6: Enable Write Master
	 *
	 * CRITICAL: vfe31_enable() bypasses the gen1 path which would normally
	 * call wm_enable(). We must enable WM here after CAMIF is configured
	 * and started, otherwise the WM never gets enabled and no data flows!
	 *
	 * WR_CFG bits:
	 *   Bit 0: enable
	 * Note: VFE31 does NOT have frame_based mode in bit 1. webOS writes 1.
	 */
	dev_info(vfe->camss->dev, "VFE31: Step 6 - Enable Write Master WM%d\n", wm);
	writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));
	wmb();

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
	 * VFE31 output paths (per webOS msm_vfe31.c):
	 * - out0 (Preview):  WM0/WM1 → COMPOSITE_DONE_0 (bits 0-7)
	 * - out1 (Snapshot): WM → COMPOSITE_DONE_1 (bits 8-15)
	 * - out2 (Video):    WM4/WM5 → COMPOSITE_DONE_2 (bits 16-23)
	 *
	 * Composite group mapping:
	 * - comp 0: bits 0-7 (COMPOSITE_DONE_0) - preview
	 * - comp 1: bits 8-15 (COMPOSITE_DONE_1) - snapshot
	 * - comp 2: bits 16-23 (COMPOSITE_DONE_2) - video
	 */
	if (enable) {
		u32 new_bits = BIT(comp * 8);  /* WM0 mapped to composite group */

		/*
		 * If video output is enabled, also map WM4/WM5 to COMPOSITE_DONE_2.
		 * This allows video recording to trigger IRQs independently from
		 * preview (which uses COMPOSITE_DONE_0).
		 */
		if (vfe31_video_output_enable) {
			/* Map WM4 (video Y) to COMPOSITE_DONE_2 (bits 16-23) */
			new_bits |= BIT(VFE31_VIDEO_WM_Y + 16);
			/* Map WM5 (video CbCr) to COMPOSITE_DONE_2 */
			new_bits |= BIT(VFE31_VIDEO_WM_CBCR + 16);

			/* Enable COMPOSITE_DONE_2 IRQ for video */
			val0 |= VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);
		}

		/* OR new bits into shadow and write to register */
		vfe->irq_comp_mask_shadow |= new_bits;
		dev_info(vfe->camss->dev,
			 "VFE31 pix_line: IRQ_COMPOSITE_MASK=0x%08x (added 0x%x, COMP%d%s)\n",
			 vfe->irq_comp_mask_shadow, new_bits, comp,
			 vfe31_video_output_enable ? ", WM4/5 -> COMP2" : "");
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
