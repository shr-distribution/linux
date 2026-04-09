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
#include <linux/dma-mapping.h>
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
 * WebOS uses OFFSET-BY-4 WM PAIRING in OUTPUT_1_AND_3 mode:
 *   - Preview (out0): WM0 (Y) + WM4 (CbCr)
 *   - Video (out2):   WM1 (Y) + WM5 (CbCr)
 *
 * This is controlled by XBAR_CFG1:
 *   - bits[3:0] = Y routing: 0xB = output0 + output2 (preview + video Y)
 *   - bits[7:4] = CbCr routing: 0x1 = output0 only (preview CbCr)
 *
 * XBAR ROUTING (0x1A1B):
 *   - Y goes to output0.ch0 (WM0) AND output2.ch0 (WM1)
 *   - CbCr goes to output0.ch1 (WM4) ONLY
 *   - output2.ch1 (WM5) is disabled by XBAR
 *
 * WebOS channel assignments (out0.ch1=4, out2.ch1=5) confirm:
 *   - Preview CbCr → WM4
 *   - Video CbCr → WM5 (but disabled by XBAR 0x1A1B)
 *
 * IMPORTANT: WebOS DISABLED CbCr write masters in register dumps!
 *   See reports/webos-video-mode-dump.txt. They only captured Y planes.
 *   We're attempting what webOS never actually tested.
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
/*
 * XBAR_CFG1 enables OUTPUT CHANNELS, not WMs directly!
 * OUTPUT_1_AND_3 mode WM mapping (from webOS msm_vfe31.c lines 719-722):
 *   output0.ch0 = WM0 (Preview Y)
 *   output0.ch1 = WM4 (Preview CbCr)
 *   output2.ch0 = WM1 (Video Y)
 *   output2.ch1 = WM5 (Video CbCr)
 */
#define VFE31_XBAR_PIX_ONLY	0x1A13  /* Y→output0, CbCr→output0 (WM0+WM4) */
#define VFE31_XBAR_PIX_VIDEO	0x1A1B  /* Y→output0+2, CbCr→output0 (WM0+WM1+WM4) */

/* Module param for manual override/testing */
int vfe31_xbar_cfg1 = 0;  /* 0 = auto-select based on active lines */
module_param(vfe31_xbar_cfg1, int, 0644);
MODULE_PARM_DESC(vfe31_xbar_cfg1,
		 "VFE31 XBAR_CFG1 override (0=auto, 0x1a03=webOS default, 0x1a1b=pix+video)");

/*
 * WM bytesperline override for testing stride issues.
 * 0 = auto (use V4L2 format bytesperline, e.g., 640 for NV16)
 * 1280 = UYVY input stride (webOS style)
 * 640 = NV16 output plane stride
 */
static int vfe31_bytesperline = 0;
module_param(vfe31_bytesperline, int, 0644);
MODULE_PARM_DESC(vfe31_bytesperline,
		 "VFE31 WM bytesperline override (0=auto/format, 640=NV16, 1280=UYVY)");

/*
 * DEMUX config overrides for testing different pixel format routing.
 * Default values (0) use format-specific auto-detection.
 * UYVY: even=0xC9, odd=0xCA
 * YUYV: even=0xC9, odd=0xAC
 */
static int vfe31_demux_even = 0;
module_param(vfe31_demux_even, int, 0644);
MODULE_PARM_DESC(vfe31_demux_even,
		 "VFE31 DEMUX even config (0=auto, 0xC9=UYVY/YUYV even)");

static int vfe31_demux_odd = 0;
module_param(vfe31_demux_odd, int, 0644);
MODULE_PARM_DESC(vfe31_demux_odd,
		 "VFE31 DEMUX odd config (0=auto, 0xCA=UYVY, 0xAC=YUYV)");

/*
 * WM4 (VIDEO Y) ADDR_CFG lines field override.
 * Controls the upper 16 bits of WM4_ADDR_CFG register.
 *
 * Values to try:
 *   -1 = auto (use height-24, same as CbCr WMs)
 *    0 = disabled (current default, causes ~240 lines captured)
 *  456 = height-24 for 480 height (matches CbCr formula)
 *  480 = full height
 *  544 = height+64 (extrapolated from webOS 304=240+64)
 *
 * WebOS VIDEO at 336x240 used lines=304, burst=151
 * For 640x480, try different values to find what works.
 */
static int vfe31_wm4_lines = -1;  /* -1 = auto (height-24) */
module_param(vfe31_wm4_lines, int, 0644);
MODULE_PARM_DESC(vfe31_wm4_lines,
		 "VFE31 WM4 (VIDEO Y) lines override (-1=auto/height-24, 0=disabled, N=explicit)");

/*
 * PIX CbCr WM (WM4) ADDR_CFG lines field override.
 * Controls the lines field (upper 16 bits) of WM4_ADDR_CFG register for PIX mode.
 *
 * Values:
 *   -1 = auto (use height-24, same as Y WMs)
 *   0  = use height-1 (full height)
 *   >0 = use explicit value
 *
 * WebOS at 1280 height used lines=304, but that may be subsampled output.
 * For full height NV16, we need lines = height or height-1.
 */
static int vfe31_pix_cbcr_lines = -1;  /* -1 = auto (height-24) */
module_param(vfe31_pix_cbcr_lines, int, 0644);
MODULE_PARM_DESC(vfe31_pix_cbcr_lines,
		 "VFE31 PIX CbCr WM lines (-1=auto/height-24, 0=height-1, >0=explicit)");

/*
 * PIX CbCr WM (WM4) ADDR_CFG burst field override.
 * Controls the burst field (lower 16 bits) of WM4_ADDR_CFG register for PIX mode.
 *
 * Values:
 *   -1 = auto (use (bytesperline/4) - 17)
 *   >0 = use explicit value
 *
 * WebOS used burst=151 for CbCr. Our auto calculation gives 143 for 640 width.
 * The difference (8 words = 32 bytes) might be alignment padding.
 */
static int vfe31_pix_cbcr_burst = -1;  /* -1 = auto */
module_param(vfe31_pix_cbcr_burst, int, 0644);
MODULE_PARM_DESC(vfe31_pix_cbcr_burst,
		 "VFE31 PIX CbCr WM burst (-1=auto, >0=explicit)");

/*
 * PIX CbCr WM (WM4) UB_CFG height field override.
 * Controls the lower 16 bits of WM4_UB_CFG register.
 *
 * Values:
 *   -1 = auto (use height-1)
 *   >0 = use explicit value
 */
static int vfe31_pix_cbcr_ub_height = -1;
module_param(vfe31_pix_cbcr_ub_height, int, 0644);
MODULE_PARM_DESC(vfe31_pix_cbcr_ub_height,
		 "VFE31 PIX CbCr UB_CFG height (-1=auto/height-1, >0=explicit)");

/*
 * PIX CbCr WM (WM4) IMAGE_SIZE height field override.
 * Controls the height encoding in IMAGE_SIZE register: ((height-1) << 4) | 2
 *
 * Values:
 *   -1 = auto (use height from format)
 *   >0 = use explicit height value
 */
static int vfe31_pix_cbcr_img_height = -1;
module_param(vfe31_pix_cbcr_img_height, int, 0644);
MODULE_PARM_DESC(vfe31_pix_cbcr_img_height,
		 "VFE31 PIX CbCr IMAGE_SIZE height (-1=auto, >0=explicit)");

/*
 * ============================================================================
 * Write Master selection parameters
 * ============================================================================
 *
 * These parameters allow runtime selection of which Write Master (WM)
 * receives each data type. This is critical for matching XBAR routing.
 *
 * VFE31 WM layout:
 *   WM0 = output0.ch0 (Preview Y)
 *   WM1 = output0.ch1 OR output2.ch0 (CbCr or Video Y, XBAR-dependent)
 *   WM4 = output0.ch1 OR output2.ch0 (Preview CbCr or Video Y)
 *   WM5 = output2.ch1 (Video CbCr)
 *
 * XBAR 0x1A13 (PIX only):   Y→WM0, CbCr→WM1
 * XBAR 0x1A1B (PIX+VIDEO):  Y→WM0+WM4, CbCr→WM1
 */

/* PIX Y Write Master (default: WM0) */
static int vfe31_pix_y_wm = 0;
module_param(vfe31_pix_y_wm, int, 0644);
MODULE_PARM_DESC(vfe31_pix_y_wm,
		 "VFE31 PIX Y WM selection (0=WM0/default)");

/* PIX CbCr Write Master (default: WM4, but WM1 matches XBAR) */
static int vfe31_pix_cbcr_wm = 4;
module_param(vfe31_pix_cbcr_wm, int, 0644);
MODULE_PARM_DESC(vfe31_pix_cbcr_wm,
		 "VFE31 PIX CbCr WM selection (1=WM1/XBAR-match, 4=WM4/legacy)");

/* VIDEO Y Write Master (default: WM1) */
static int vfe31_video_y_wm = 1;
module_param(vfe31_video_y_wm, int, 0644);
MODULE_PARM_DESC(vfe31_video_y_wm,
		 "VFE31 VIDEO Y WM selection (1=WM1/default, 4=WM4)");

/* VIDEO CbCr Write Master (default: WM5) */
static int vfe31_video_cbcr_wm = 5;
module_param(vfe31_video_cbcr_wm, int, 0644);
MODULE_PARM_DESC(vfe31_video_cbcr_wm,
		 "VFE31 VIDEO CbCr WM selection (5=WM5/default)");

/*
 * ============================================================================
 * PIX Y (WM0) debug parameters
 * ============================================================================
 */
static int vfe31_pix_y_lines = -1;
module_param(vfe31_pix_y_lines, int, 0644);
MODULE_PARM_DESC(vfe31_pix_y_lines,
		 "VFE31 PIX Y WM lines (-1=auto, 0=disabled, >0=explicit)");

static int vfe31_pix_y_burst = -1;
module_param(vfe31_pix_y_burst, int, 0644);
MODULE_PARM_DESC(vfe31_pix_y_burst,
		 "VFE31 PIX Y WM burst (-1=auto, >0=explicit)");

static int vfe31_pix_y_ub_height = -1;
module_param(vfe31_pix_y_ub_height, int, 0644);
MODULE_PARM_DESC(vfe31_pix_y_ub_height,
		 "VFE31 PIX Y UB_CFG height (-1=auto, >0=explicit)");

static int vfe31_pix_y_img_height = -1;
module_param(vfe31_pix_y_img_height, int, 0644);
MODULE_PARM_DESC(vfe31_pix_y_img_height,
		 "VFE31 PIX Y IMAGE_SIZE height (-1=auto, >0=explicit)");

/*
 * ============================================================================
 * VIDEO Y (WM1) debug parameters
 * ============================================================================
 */
static int vfe31_video_y_lines = -1;
module_param(vfe31_video_y_lines, int, 0644);
MODULE_PARM_DESC(vfe31_video_y_lines,
		 "VFE31 VIDEO Y WM lines (-1=auto, 0=disabled, >0=explicit)");

static int vfe31_video_y_burst = -1;
module_param(vfe31_video_y_burst, int, 0644);
MODULE_PARM_DESC(vfe31_video_y_burst,
		 "VFE31 VIDEO Y WM burst (-1=auto, >0=explicit)");

static int vfe31_video_y_ub_height = -1;
module_param(vfe31_video_y_ub_height, int, 0644);
MODULE_PARM_DESC(vfe31_video_y_ub_height,
		 "VFE31 VIDEO Y UB_CFG height (-1=auto, >0=explicit)");

static int vfe31_video_y_img_height = -1;
module_param(vfe31_video_y_img_height, int, 0644);
MODULE_PARM_DESC(vfe31_video_y_img_height,
		 "VFE31 VIDEO Y IMAGE_SIZE height (-1=auto, >0=explicit)");

/*
 * ============================================================================
 * VIDEO CbCr (WM5) debug parameters
 * ============================================================================
 */
static int vfe31_video_cbcr_lines = -1;
module_param(vfe31_video_cbcr_lines, int, 0644);
MODULE_PARM_DESC(vfe31_video_cbcr_lines,
		 "VFE31 VIDEO CbCr WM lines (-1=auto, 0=height-1, >0=explicit)");

static int vfe31_video_cbcr_burst = -1;
module_param(vfe31_video_cbcr_burst, int, 0644);
MODULE_PARM_DESC(vfe31_video_cbcr_burst,
		 "VFE31 VIDEO CbCr WM burst (-1=auto, >0=explicit)");

static int vfe31_video_cbcr_ub_height = -1;
module_param(vfe31_video_cbcr_ub_height, int, 0644);
MODULE_PARM_DESC(vfe31_video_cbcr_ub_height,
		 "VFE31 VIDEO CbCr UB_CFG height (-1=auto, >0=explicit)");

static int vfe31_video_cbcr_img_height = -1;
module_param(vfe31_video_cbcr_img_height, int, 0644);
MODULE_PARM_DESC(vfe31_video_cbcr_img_height,
		 "VFE31 VIDEO CbCr IMAGE_SIZE height (-1=auto, >0=explicit)");

/*
 * Debug: dump all WM registers after configuration.
 * Set to 1 to enable verbose register dumps in dmesg.
 */
static int vfe31_dump_wm_regs = 0;
module_param(vfe31_dump_wm_regs, int, 0644);
MODULE_PARM_DESC(vfe31_dump_wm_regs,
		 "VFE31 dump WM registers (0=off, 1=on)");

/*
 * ============================================================================
 * Chroma scale debug parameters
 * ============================================================================
 *
 * CHROMA_V_IMAGE register (0x4F0) controls vertical chroma scaling:
 *   Format: (output_height << 16) | input_height
 *   1:1 scaling: (height << 16) | height = 0x01E001E0 for 480
 *   2:1 scaling: (height/2 << 16) | height = 0x00F001E0 for 480
 *
 * WebOS used 2:1 (0x00F001E0) even for video - they used NV12 internally.
 * For NV16 (4:2:2), we need 1:1 scaling.
 */
static int vfe31_chroma_v_out = -1;  /* -1 = auto (same as input for NV16) */
module_param(vfe31_chroma_v_out, int, 0644);
MODULE_PARM_DESC(vfe31_chroma_v_out,
		 "VFE31 CHROMA_V_IMAGE output height (-1=auto, 0=half, >0=explicit)");

/*
 * CHROMA_V_PHASE register (0x4F4) controls vertical scaling phase:
 *   0x00310000 = 1:1 scaling (no subsample)
 *   0x00320000 = 2:1 scaling (vertical subsample)
 */
static int vfe31_chroma_v_phase = -1;  /* -1 = auto based on format */
module_param(vfe31_chroma_v_phase, int, 0644);
MODULE_PARM_DESC(vfe31_chroma_v_phase,
		 "VFE31 CHROMA_V_PHASE (-1=auto, 0x00310000=1:1, 0x00320000=2:1)");

/*
 * CHROMA_SUBS_CFG register (0x4F8) - chroma subsample config
 * WebOS uses 0x30. Try different values if CbCr capture is wrong.
 */
static int vfe31_chroma_subs_cfg = -1;  /* -1 = use 0x30 (webOS default) */
module_param(vfe31_chroma_subs_cfg, int, 0644);
MODULE_PARM_DESC(vfe31_chroma_subs_cfg,
		 "VFE31 CHROMA_SUBS_CFG (-1=0x30, >0=explicit)");

/*
 * ============================================================================
 * VFE31 FORMAT OVERRIDE - For testing NV16 (4:2:2) vs NV12 (4:2:0)
 * ============================================================================
 *
 * Controls how the driver interprets format for hardware configuration.
 * This affects chroma subsampling, CbCr WM height, UB height, etc.
 *
 *   0 = Auto - use actual requested format (default)
 *   1 = Force 4:2:0 - treat all semi-planar formats as NV12/NV21
 *   2 = Force 4:2:2 - treat all semi-planar formats as NV16/NV61
 *
 * Use this to test if VFE31 hardware supports 4:2:2 output.
 * When forcing 4:2:2, ensure userspace allocates NV16-sized buffers
 * (Width * Height * 2 instead of Width * Height * 1.5).
 *
 * To test NV16 properly:
 *   1. Set vfe31_force_422=2 to configure hardware for 4:2:2
 *   2. Use v4l2-ctl to request NV16 format from userspace
 *   3. Or use vfe31_force_422=2 with NV12 buffers to test HW capability
 *      (CbCr plane may overrun if HW actually outputs full height!)
 *
 * Related parameters for fine-tuning:
 *   - vfe31_chroma_v_out: Force specific chroma output height
 *   - vfe31_chroma_v_phase: Force specific scaling phase
 *   - vfe31_pix_cbcr_img_height: Force CbCr IMAGE_SIZE height
 *   - vfe31_pix_cbcr_ub_height: Force CbCr UB_CFG height
 */
static int vfe31_force_422 = 0;
module_param(vfe31_force_422, int, 0644);
MODULE_PARM_DESC(vfe31_force_422,
		 "VFE31 format mode: 0=auto, 1=force 4:2:0, 2=force 4:2:2");

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
 * With corrected WM assignments:
 *   PIX:   WM0 (Y) + WM4 (CbCr) → bits 0,4 → 0x11
 *   VIDEO: WM1 (Y) + WM5 (CbCr) → bits 1,5 → 0x22
 *   Both:  WM0 + WM1 + WM4 + WM5 → bits 0,1,4,5 → 0x33
 *
 * NOTE: With XBAR 0x1A1B, VIDEO CbCr (WM5) is disabled.
 * To enable VIDEO CbCr, XBAR bits[7:4] needs to be 0x9 (both outputs).
 *
 * WebOS IRQ_COMPOSITE_MASK = 0x00220011:
 *   Group 0 (bits 0-7):  0x11 = WM0 + WM4 (PIX Y + CbCr)
 *   Group 1 (bits 8-15): 0x00 = none
 *   Group 2 (bits 16-23): 0x22 = WM1 + WM5 (VIDEO Y + CbCr)
 *
 * CRITICAL: Must use exact webOS value - hardware may require both PIX and
 * VIDEO composite groups configured even when only using one path.
 */
#define VFE31_IRQ_COMP_MASK_PIX_ONLY	0x00220011  /* webOS exact value: PIX in G0, VIDEO in G2 */
#define VFE31_IRQ_COMP_MASK_PIX_VIDEO	0x00220011  /* Same as PIX_ONLY per webOS */
#define VFE31_IRQ_COMP_MASK_VIDEO_ONLY	0x00220011  /* Same as PIX_ONLY per webOS */

/* Module param for manual override/testing */
static int vfe31_irq_comp_mask = 0;  /* 0 = auto-select based on active lines */
module_param(vfe31_irq_comp_mask, int, 0644);
MODULE_PARM_DESC(vfe31_irq_comp_mask,
		 "VFE31 IRQ composite mask (0=auto, 0x11=pix, 0x13=pix+video, 0x02=video)");

/*
 * IMAGE_SIZE stride mode for Y WM configuration:
 * 0 = auto (input stride for PIX/VIDEO, output stride for RDI)
 * 1 = force input stride (width * 2) - matches webOS
 * 2 = force output stride (bytesperline)
 * >2 = use this value directly as stride in bytes
 *
 * NOTE: With COMPOSITE_DONE-based frame completion, output stride (mode 2)
 * should work correctly for NV12. Use vfe31_nv12_stride_fix to enable/disable
 * the legacy workaround that forces input stride for NV12 format.
 */
static int vfe31_image_stride = 0;
module_param(vfe31_image_stride, int, 0644);
MODULE_PARM_DESC(vfe31_image_stride,
		 "VFE31 Y WM IMAGE_SIZE stride (0=auto, 1=input, 2=output, >2=custom bytes)");

/*
 * IMAGE_SIZE stride mode for CbCr WM configuration:
 * 0 = auto (use bytesperline/output stride - safe default)
 * 1 = force input stride (width * 2)
 * 2 = force output stride (bytesperline)
 * >2 = use this value directly as stride in bytes
 *
 * CbCr WMs write to the output buffer, so auto uses bytesperline.
 * Using input stride causes buffer overflow and crashes!
 */
static int vfe31_cbcr_stride = 0;
module_param(vfe31_cbcr_stride, int, 0644);
MODULE_PARM_DESC(vfe31_cbcr_stride,
		 "VFE31 CbCr WM IMAGE_SIZE stride (0=auto/output, 1=input, 2=output, >2=custom bytes)");

/*
 * NV12 stride workaround control:
 *   0 = Disabled - use configured stride (test COMPOSITE_DONE fix)
 *   1 = Enabled - force input stride for NV12 (legacy workaround)
 *
 * With COMPOSITE_DONE-based frame completion, the workaround should no
 * longer be needed. Set to 0 to test if output stride (640) works correctly.
 */
static int vfe31_nv12_stride_fix = 0;
module_param(vfe31_nv12_stride_fix, int, 0644);
MODULE_PARM_DESC(vfe31_nv12_stride_fix,
		 "VFE31 NV12 stride workaround: 0=disabled (test), 1=force input stride (legacy)");

/*
 * VFE31 single-buffer mode:
 *   0 = Normal double-buffering (PING and PONG have different addresses)
 *   1 = Single-buffer mode (PONG = PING, same address for both)
 *
 * Use single-buffer mode as a workaround if PONG frames are always empty.
 * In single-buffer mode, the hardware writes all frames to the same buffer
 * address. This sacrifices double-buffering but ensures valid frame data.
 *
 * The trade-off: potential tearing if software reads while hardware writes,
 * but all frames will have valid data instead of alternating empty frames.
 */
static int vfe31_single_buffer = 0;
module_param(vfe31_single_buffer, int, 0644);
MODULE_PARM_DESC(vfe31_single_buffer,
		 "VFE31 single-buffer mode: 0=normal, 1=PONG=PING (workaround for empty PONG)");

/*
 * VFE31 ping-pong SWAP mode (webOS V4L2 workaround).
 *
 * When enabled, after each frame completion, the driver swaps the contents
 * of PING and PONG address registers. This is a workaround for VFE31 hardware
 * that may only write to the PING buffer location regardless of PP status.
 *
 * This matches the CONFIG_MSM_CAMERA_V4L2 code path in webOS msm_vfe31.c
 * (lines 2564-2574 in vfe31_process_output_path_irq_0).
 *
 * 0 = Normal mode (update one register based on PP status)
 * 1 = Swap mode (swap PING and PONG addresses after each frame)
 */
static int vfe31_swap_pingpong = 0;
module_param(vfe31_swap_pingpong, int, 0644);
MODULE_PARM_DESC(vfe31_swap_pingpong,
		 "VFE31 swap PING/PONG addresses each frame: 0=normal, 1=swap (webOS V4L2 workaround)");

/*
 * VFE31 force PING-only mode.
 *
 * When enabled, always return buf[0] (PING buffer) regardless of PP status.
 * This is a workaround for hardware that only writes to PING address even
 * though PP status toggles.
 *
 * In this mode:
 * - buf[0] is always configured as both PING and PONG
 * - All frames are written to the same physical address
 * - We always return buf[0] which contains valid data
 * - Potential for tearing if userspace reads during DMA write
 *
 * 0 = Normal ping-pong based on PP status
 * 1 = Address-based buffer matching (read PING, find matching buf)
 * 2 = Static buffer mode (always return buf[0], never rotate)
 */
static int vfe31_ping_only = 0;
module_param(vfe31_ping_only, int, 0644);
MODULE_PARM_DESC(vfe31_ping_only,
		 "VFE31 PING-only mode: 0=normal, 1=addr match, 2=static buf[0]");

/*
 * Invert PP bit interpretation.
 * 0 = normal (PP=1 means PING completed)
 * 1 = inverted (PP=1 means PONG completed)
 */
static int vfe31_invert_pp = 0;
module_param(vfe31_invert_pp, int, 0644);
MODULE_PARM_DESC(vfe31_invert_pp,
		 "VFE31 invert PP interpretation: 0=normal, 1=inverted");

/*
 * BUS_CFG register value override.
 * 0 = use default (0x02AAA771 per webOS)
 * >0 = use this value directly
 *
 * WebOS value: 0x02AAA771
 * Bits 4-7 control write paths: encY, encCbCr, viewY, viewCbCr
 */
static unsigned int vfe31_bus_cfg = 0;
module_param(vfe31_bus_cfg, uint, 0644);
MODULE_PARM_DESC(vfe31_bus_cfg,
		 "VFE31 BUS_CFG override: 0=default (0x02AAA771), >0=use value (e.g., 0x02AAA7F1)");

/*
 * BUS_CMD reload value override.
 * 0 = use default (0x7FFF per webOS code)
 * >0 = use this value directly
 *
 * WebOS CODE writes 0x7FFF (not 0x3FFF as in register dumps).
 * Bit 14 is a pingpong reload trigger that enables proper dual-buffer operation.
 * Without bit 14, hardware may only write to PING and ignore PONG addresses.
 */
static unsigned int vfe31_bus_cmd_reload = 0;
module_param(vfe31_bus_cmd_reload, uint, 0644);
MODULE_PARM_DESC(vfe31_bus_cmd_reload,
		 "VFE31 BUS_CMD reload value: 0=default (0x7FFF), >0=use value");

/*
 * Helper to get effective BUS_CFG value (module param or default).
 * Default is 0x02AAA771 per webOS register dumps.
 */
static inline u32 vfe31_get_bus_cfg(void)
{
	return vfe31_bus_cfg ? vfe31_bus_cfg : 0x02AAA771;
}

/*
 * Helper to get effective BUS_CMD reload value (module param or default).
 * Default is 0x7FFF per webOS code (includes pingpong reload bit 14).
 */
static inline u32 vfe31_get_bus_cmd_reload(void)
{
	return vfe31_bus_cmd_reload ? vfe31_bus_cmd_reload : 0x7FFF;
}

/*
 * Helper to check if format is semi-planar (NV12/NV21/NV16/NV61).
 * Semi-planar formats have separate Y and CbCr planes with 1 byte per sample.
 */
static inline bool vfe31_is_semiplanar_format(u32 pixelformat)
{
	return (pixelformat == V4L2_PIX_FMT_NV12 ||
		pixelformat == V4L2_PIX_FMT_NV21 ||
		pixelformat == V4L2_PIX_FMT_NV16 ||
		pixelformat == V4L2_PIX_FMT_NV61);
}

/*
 * Helper to calculate IMAGE_SIZE stride for Y WMs.
 * Returns stride in bytes.
 *
 * For semi-planar formats (NV12/NV16): stride = bytesperline (1 byte/pixel)
 * For packed formats (UYVY): stride = width * 2 (2 bytes/pixel)
 */
static inline u16 vfe31_calc_image_stride(u16 width, u16 bytesperline,
					  bool is_rdi, u32 pixelformat)
{
	if (vfe31_image_stride > 2)
		return (u16)vfe31_image_stride;
	else if (vfe31_image_stride == 1)
		return width * 2;  /* Force packed stride */
	else if (vfe31_image_stride == 2)
		return bytesperline;  /* Force V4L2 stride */
	else {
		/* Auto: use bytesperline for semi-planar, width*2 for packed */
		if (is_rdi)
			return bytesperline;
		return vfe31_is_semiplanar_format(pixelformat) ? bytesperline : (width * 2);
	}
}

/*
 * Helper to calculate IMAGE_SIZE stride for CbCr WMs.
 * Returns stride in bytes. Default is bytesperline (safe).
 */
static inline u16 vfe31_calc_cbcr_stride(u16 width, u16 bytesperline)
{
	if (vfe31_cbcr_stride > 2)
		return (u16)vfe31_cbcr_stride;
	else if (vfe31_cbcr_stride == 1)
		return width * 2;
	else
		/* Auto and mode 2 both use bytesperline (safe default) */
		return bytesperline;
}

/*
 * Helper to determine if format should use 4:2:0 chroma subsampling.
 * Returns true for 4:2:0 formats (NV12/NV21), false for 4:2:2 (NV16/NV61).
 *
 * When vfe31_force_422 is set:
 *   0 = auto (based on actual format)
 *   1 = force 4:2:0 (return true for all semi-planar)
 *   2 = force 4:2:2 (return false for all semi-planar)
 */
static inline bool vfe31_is_420_format(u32 pixelformat)
{
	/* Check if it's a semi-planar format at all */
	bool is_semiplanar = (pixelformat == V4L2_PIX_FMT_NV12 ||
			      pixelformat == V4L2_PIX_FMT_NV21 ||
			      pixelformat == V4L2_PIX_FMT_NV16 ||
			      pixelformat == V4L2_PIX_FMT_NV61);

	if (!is_semiplanar)
		return false;  /* Not semi-planar, doesn't apply */

	/* Check format override */
	if (vfe31_force_422 == 1)
		return true;   /* Force 4:2:0 */
	if (vfe31_force_422 == 2)
		return false;  /* Force 4:2:2 */

	/* Auto: based on actual format */
	return (pixelformat == V4L2_PIX_FMT_NV12 ||
		pixelformat == V4L2_PIX_FMT_NV21);
}

/*
 * Helper to calculate CbCr height based on format and overrides.
 * For 4:2:0: returns height / 2
 * For 4:2:2: returns height (full)
 */
static inline u16 vfe31_calc_cbcr_height(u32 pixelformat, u16 height)
{
	return vfe31_is_420_format(pixelformat) ? height / 2 : height;
}

/* External module parameters from camss-vfe.c */
extern int software_sof_enable;
extern int software_eof_enable;

/*
 * VFE31 XBAR routing - OUTPUT_1_AND_3 mode with XBAR 0x1A1B:
 *   - PIX:   WM0 (Y) + WM4 (CbCr)
 *   - VIDEO: WM1 (Y) + WM5 (CbCr) [CbCr disabled by XBAR unless 0x1A9B]
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
 * CRITICAL: Must match exact webOS value 0x02AAA771!
 * Bit 7 (viewCbcrWrPathEn) must NOT be set - enabling it may interfere
 * with ping-pong DMA causing PONG buffers to be empty.
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

/*
 * XBAR_CFG1 bit field definitions
 *
 * IMPORTANT: These route to OUTPUT CHANNELS, not WMs directly!
 * OUTPUT_1_AND_3 mode maps channels to WMs:
 *   output0.ch0 → WM0, output0.ch1 → WM4
 *   output2.ch0 → WM1, output2.ch1 → WM5
 */
#define VFE_0_BUS_XBAR_CFG1_Y_ROUTING_MASK	0x0000000F
#define VFE_0_BUS_XBAR_CFG1_Y_ROUTING_SHIFT	0
#define VFE_0_BUS_XBAR_CFG1_Y_WM0		0x3   /* Y → output0 only (WM0) */
#define VFE_0_BUS_XBAR_CFG1_Y_WM0_WM4		0xB   /* Y → output0+2 (WM0+WM1) */

#define VFE_0_BUS_XBAR_CFG1_CBCR_ROUTING_MASK	0x000000F0
#define VFE_0_BUS_XBAR_CFG1_CBCR_ROUTING_SHIFT	4
#define VFE_0_BUS_XBAR_CFG1_CBCR_DISABLED	0x0   /* CbCr disabled */
#define VFE_0_BUS_XBAR_CFG1_CBCR_WM1		0x1   /* CbCr → output0 only (WM4!) */
#define VFE_0_BUS_XBAR_CFG1_CBCR_WM1_WM5	0x9   /* CbCr → output0+2 (WM4+WM5) */

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
 * Correct WM assignments based on webOS channel assignments:
 *   Preview (output0): ch0=WM0 (Y), ch1=WM4 (CbCr)
 *   Video (output2):   ch0=WM1 (Y), ch1=WM5 (CbCr)
 *
 * XBAR_CFG1 = 0x1A1B enables:
 *   - Y to output0.ch0 (WM0) AND output2.ch0 (WM1)
 *   - CbCr to output0.ch1 (WM4) ONLY (bits[7:4]=0x1)
 *
 * NOTE: Previous code incorrectly used WM1 for preview CbCr, but
 * WM1 is output2.ch0 = VIDEO Y! That's why WM1 contained Y data.
 */
#define VFE31_PREVIEW_WM_Y		0  /* WM0 = output0.ch0 (preview Y) */
#define VFE31_PREVIEW_WM_CBCR		4  /* WM4 = output0.ch1 (preview CbCr) */

/*
 * VIDEO mode: WM1 for Y (output2.ch0), WM5 for CbCr (output2.ch1)
 *
 * NOTE: With XBAR 0x1A1B, CbCr only goes to output0.ch1 (WM4).
 * To enable VIDEO CbCr (output2.ch1/WM5), XBAR bits[7:4] would
 * need to be 0x9. Current XBAR disables VIDEO CbCr output.
 */
#define VFE31_VIDEO_WM_Y		1  /* WM1 = output2.ch0 (video Y) */
#define VFE31_VIDEO_WM_CBCR		5  /* WM5 = output2.ch1 (video CbCr) */

/*
 * ============================================================================
 * CONFIGURATION SUMMARY (CORRECTED FROM REGISTER DUMPS AND WEBOS CODE)
 * ============================================================================
 *
 * OUTPUT_1_AND_3 mode (AXI=0x01) assigns WMs to output channels:
 *   output0 (Preview): ch0=WM0 (Y), ch1=WM4 (CbCr)
 *   output2 (Video):   ch0=WM1 (Y), ch1=WM5 (CbCr)
 *
 * For semi-planar preview (NV12/NV16/NV21/NV61):
 *   XBAR_CFG0 = 0x01   (OUTPUT_1_AND_3)
 *   XBAR_CFG1 = 0x1A1B (Y→output0+output2, CbCr→output0 only)
 *   BUS_CFG   = 0x02AAA771
 *   WM0 = Preview Y, WM4 = Preview CbCr
 *
 * For video recording (VIDEO line active):
 *   Same XBAR settings, but VIDEO uses:
 *   WM1 = Video Y, WM5 = Video CbCr (needs XBAR 0x1A9B for CbCr)
 *
 * For raw bypass (SRGGB8/10/12):
 *   XBAR_CFG0 = 0x60   (RAW_BYPASS / CAMIF_TO_AXI)
 *   XBAR_CFG1 = 0x00   (not used in raw mode)
 *   BUS_CFG   = configured for raw pixel size
 *
 * NOTE: webOS only captured Y data - CbCr WMs were disabled in dumps.
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

/*
 * vfe31_get_cbcr_offset - Calculate CbCr plane offset for semi-planar formats
 * @pixelformat: V4L2 pixel format (e.g., V4L2_PIX_FMT_NV16)
 * @y_stride: Actual bytes per line used by Y WM (IMAGE_SIZE stride)
 * @height: Frame height in lines
 *
 * Returns the byte offset where the CbCr (UV) plane starts within the buffer.
 * For semi-planar formats (NV16, NV61, NV12, NV21), the UV plane follows
 * immediately after the Y plane. For packed formats (UYVY, YUYV, etc.) and
 * RAW formats, returns 0 since they use a single plane.
 *
 * IMPORTANT: y_stride must be the actual stride used by the Y write master
 * (from vfe31_calc_image_stride), NOT the V4L2 format's bytesperline.
 * For PIX/VIDEO mode, this is width*2 (UYVY input processing), which may
 * differ from the format's bytesperline (width for NV12/NV16 output).
 */
static inline u32 vfe31_get_cbcr_offset(u32 pixelformat, u32 y_stride, u16 height)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		/*
		 * NV16/NV61: Y plane = y_stride * height bytes
		 * CbCr plane starts immediately after Y plane.
		 * CbCr is interleaved (CbCrCbCr...) with full vertical resolution.
		 */
		return y_stride * height;

	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		/*
		 * NV12/NV21: Y plane = y_stride * height bytes
		 * CbCr plane starts after Y, but with half vertical resolution.
		 */
		return y_stride * height;

	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
		/* Packed formats: single interleaved plane, no offset needed */
		return 0;

	default:
		/* RAW formats and unknown: single plane */
		return 0;
	}
}

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
	 * 6. Reload all write masters with pingpong (BUS_CMD = 0x7FFF per webOS code)
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
	 * Step 6: Reload all write masters including pingpong buffers.
	 *
	 * CRITICAL: webOS CODE writes 0x7FFF (15 bits), not 0x3FFF!
	 * The register DUMP shows 0x3FFF because bit 14 is pulse-based
	 * and clears after the command executes.
	 *
	 * Bit 14 appears to be a "busPingpongReload" trigger (similar to VFE8x)
	 * that enables proper ping-pong alternation. Without it, the hardware
	 * may only write to PING buffers and ignore PONG addresses.
	 *
	 * BUS_CMD bits:
	 *   - Bits 0-13: Per-WM reload commands
	 *   - Bit 14: Pingpong reload trigger (critical for dual-buffer operation)
	 */
	dev_info(vfe->camss->dev, "VFE reset: reloading all WMs with pingpong (BUS_CMD=0x%04x)\n",
		 vfe31_get_bus_cmd_reload());
	writel_relaxed(vfe31_get_bus_cmd_reload(), vfe->base + VFE_0_BUS_CMD);
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

/*
 * vfe31_wm_done - VFE31-specific buffer completion handler
 *
 * Unlike gen1's wm_done which reads PP status from hardware, this function
 * takes the PP status as a parameter. This is critical because:
 *
 * 1. We detect PP transitions in the ISR by comparing current vs last PP
 * 2. If we read PP status AGAIN inside wm_done, it might have changed
 * 3. By passing the PP value from detection time, we process the correct buffer
 *
 * @vfe: VFE device
 * @wm: Write master index (0 for PIX, 1 for VIDEO)
 * @ping_pong: The PP status at the time the transition was detected
 */
static void vfe31_wm_done(struct vfe_device *vfe, u8 wm, u32 ping_pong)
{
	struct camss_buffer *ready_buf;
	struct vfe_output *output;
	dma_addr_t *new_addr;
	unsigned long flags;
	u32 active_index;
	u64 ts = ktime_get_ns();
	unsigned int i;

	if (vfe->wm_output_map[wm] == VFE_LINE_NONE)
		return;

	/*
	 * Use the passed ping_pong value instead of reading from hardware.
	 * This ensures we process the buffer that actually completed, not
	 * whatever the current hardware state happens to be.
	 *
	 * active_index = which buffer the hardware is CURRENTLY writing to.
	 * The completed (ready) buffer is the OTHER one: !active_index
	 *
	 * VFE31 PP bit interpretation (from webOS analysis):
	 *   PP bit=0: Hardware is writing to PING, just completed PONG
	 *   PP bit=1: Hardware is writing to PONG, just completed PING
	 *
	 * Use vfe31_invert_pp module param to test inverted interpretation.
	 */
	active_index = (ping_pong >> wm) & 1;
	if (vfe31_invert_pp)
		active_index = !active_index;

	/*
	 * Debug: Log the PP-to-buffer mapping for this completion
	 */
	dev_info(vfe->camss->dev,
		"VFE31: wm_done entry: wm=%d PP=0x%x bit%d=%d → HW writing to %s, returning %s buffer\n",
		wm, ping_pong, wm, active_index,
		active_index ? "PONG" : "PING",
		active_index ? "PING" : "PONG");

	spin_lock_irqsave(&vfe->output_lock, flags);
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	output->gen1.active_buf = active_index;

	/*
	 * Debug: On first frame, dump complete WM register state
	 */
	if (output->sequence == 0) {
		u8 y_wm = output->wm_idx[0];
		u8 cbcr_wm = (output->wm_num == 2) ? output->wm_idx[1] : 0xff;
		dev_info(vfe->camss->dev,
			"VFE31: FIRST FRAME WM%d dump:\n"
			"  Y_WM%d: PING=0x%08x PONG=0x%08x CFG=0x%08x IMG_SIZE=0x%08x UB=0x%08x\n",
			wm, y_wm,
			readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(y_wm)),
			readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(y_wm)),
			readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(y_wm)),
			readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(y_wm)),
			readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(y_wm)));
		if (cbcr_wm != 0xff) {
			dev_info(vfe->camss->dev,
				"  CbCr_WM%d: PING=0x%08x PONG=0x%08x CFG=0x%08x IMG_SIZE=0x%08x UB=0x%08x\n",
				cbcr_wm,
				readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(cbcr_wm)),
				readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(cbcr_wm)),
				readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(cbcr_wm)),
				readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(cbcr_wm)),
				readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(cbcr_wm)));
		}
		dev_info(vfe->camss->dev,
			"  BUS_CFG=0x%08x AXI_OUT=0x%08x PP_STATUS=0x%08x CAMIF_STATUS=0x%08x\n",
			readl_relaxed(vfe->base + 0x03C),
			readl_relaxed(vfe->base + 0x040),
			ping_pong,
			readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS));
		dev_info(vfe->camss->dev,
			"  buf[0]=0x%08x buf[1]=0x%08x active_index=%d\n",
			output->buf[0] ? (u32)output->buf[0]->addr[0] : 0,
			output->buf[1] ? (u32)output->buf[1]->addr[0] : 0,
			active_index);
	}

	/*
	 * Select which buffer to return to userspace and track buffer index.
	 *
	 * Normal mode: return buf[!active_index] - the buffer that just completed
	 *
	 * Ping-only mode 1 (vfe31_ping_only=1):
	 * Read actual PING address from hardware and find matching buffer.
	 *
	 * Ping-only mode 2 (vfe31_ping_only=2):
	 * Static buffer mode - always return buf[0], never rotate buffers.
	 * This ensures all frames go to the same address (potential tearing).
	 */
	{
		int ready_idx;  /* Track which buffer index we're returning */
		int skip_rotation = 0;  /* Set to 1 to keep using same buffer */

		if (vfe31_ping_only == 2) {
			/* Static buffer mode: always use buf[0], no rotation */
			ready_buf = output->buf[0];
			ready_idx = 0;
			skip_rotation = 1;
			dev_info(vfe->camss->dev,
				"VFE31: static mode: returning buf[0]=0x%08x seq=%d\n",
				ready_buf ? (u32)ready_buf->addr[0] : 0,
				output->sequence);
		} else if (vfe31_ping_only == 1) {
			u8 y_wm = output->wm_idx[0];
			u32 hw_ping = readl_relaxed(vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(y_wm));

			/* Find buffer matching PING address */
			if (output->buf[0] && (u32)output->buf[0]->addr[0] == hw_ping) {
				ready_buf = output->buf[0];
				ready_idx = 0;
				dev_info(vfe->camss->dev,
					"VFE31: ping_only: returning buf[0] (matches PING 0x%08x)\n",
					hw_ping);
			} else if (output->buf[1] && (u32)output->buf[1]->addr[0] == hw_ping) {
				ready_buf = output->buf[1];
				ready_idx = 1;
				dev_info(vfe->camss->dev,
					"VFE31: ping_only: returning buf[1] (matches PING 0x%08x)\n",
					hw_ping);
			} else {
				/* Neither buffer matches - use buf[0] as fallback */
				ready_buf = output->buf[0];
				ready_idx = 0;
				dev_warn(vfe->camss->dev,
					"VFE31: ping_only: NO MATCH! PING=0x%08x buf[0]=0x%08x buf[1]=0x%08x\n",
					hw_ping,
					output->buf[0] ? (u32)output->buf[0]->addr[0] : 0,
					output->buf[1] ? (u32)output->buf[1]->addr[0] : 0);
			}
		} else if (vfe31_single_buffer) {
			/*
			 * Single-buffer mode: PING and PONG have the same address,
			 * so we can't use PP status to determine which buffer has data.
			 * Instead, find the buffer that matches the HW PING address.
			 */
			u8 y_wm = output->wm_idx[0];
			u32 hw_ping = readl_relaxed(vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(y_wm));

			if (output->buf[0] && (u32)output->buf[0]->addr[0] == hw_ping) {
				ready_buf = output->buf[0];
				ready_idx = 0;
				dev_info(vfe->camss->dev,
					"VFE31: single_buffer: returning buf[0] (matches 0x%08x)\n",
					hw_ping);
			} else if (output->buf[1] && (u32)output->buf[1]->addr[0] == hw_ping) {
				ready_buf = output->buf[1];
				ready_idx = 1;
				dev_info(vfe->camss->dev,
					"VFE31: single_buffer: returning buf[1] (matches 0x%08x)\n",
					hw_ping);
			} else {
				/* Neither buffer matches - use buf[0] as fallback */
				ready_buf = output->buf[0];
				ready_idx = 0;
				dev_warn(vfe->camss->dev,
					"VFE31: single_buffer: NO MATCH! PING=0x%08x buf[0]=0x%08x buf[1]=0x%08x\n",
					hw_ping,
					output->buf[0] ? (u32)output->buf[0]->addr[0] : 0,
					output->buf[1] ? (u32)output->buf[1]->addr[0] : 0);
			}
		} else {
			ready_buf = output->buf[!active_index];
			ready_idx = !active_index;
		}

		if (!ready_buf) {
			dev_err_ratelimited(vfe->camss->dev,
					    "VFE31: Missing ready buf wm=%d idx=%d state=%d!\n",
					    wm, ready_idx, output->state);
			goto out_unlock;
		}

		ready_buf->vb.vb2_buf.timestamp = ts;
		ready_buf->vb.sequence = output->sequence++;

		/*
		 * Get next buffer from pending queue.
		 * In ping_only mode, replace the buffer we're returning (ready_idx).
		 * In normal mode, replace !active_index slot as before.
		 * In static mode (skip_rotation), don't rotate at all.
		 */
		if (skip_rotation) {
			/* Static mode: keep using same buffer, don't update address */
			new_addr = ready_buf->addr;
		} else {
			output->buf[ready_idx] = vfe_buf_get_pending(output);
			if (!output->buf[ready_idx]) {
				/*
				 * No next buffer available - reuse same address.
				 * Transition to SINGLE state (only one buffer active).
				 */
				new_addr = ready_buf->addr;
				if (output->state == VFE_OUTPUT_CONTINUOUS)
					output->state = VFE_OUTPUT_SINGLE;
				else if (output->state == VFE_OUTPUT_SINGLE)
					output->state = VFE_OUTPUT_STOPPING;
			} else {
				new_addr = output->buf[ready_idx]->addr;
				/* Stay in CONTINUOUS state */
			}
		}
	}

	/*
	 * Update the buffer address that just completed (was just read from).
	 * active_index tells us which buffer is currently being written to,
	 * so we update the OTHER buffer's address (!active_index).
	 *
	 * When active_index=1 (writing to PONG), PING just completed → update PING
	 * When active_index=0 (writing to PING), PONG just completed → update PONG
	 *
	 * In single-buffer mode, we update BOTH PING and PONG with the same address
	 * to ensure the hardware always has a valid write location.
	 *
	 * In ping_only mode, we always update PING since hardware only writes there.
	 */
	if (vfe31_ping_only == 2) {
		/* Static mode: don't update any addresses, keep using same buffer */
		dev_dbg(vfe->camss->dev,
			"VFE31: static mode: keeping PING/PONG at 0x%08x\n",
			(u32)new_addr[0]);
	} else if (vfe31_ping_only == 1) {
		/* Ping-only mode: always update PING (hardware only writes to PING) */
		dev_info(vfe->camss->dev,
			"VFE31: ping_only: updating PING to 0x%08x for next frame (seq=%d)\n",
			(u32)new_addr[0], output->sequence);
		for (i = 0; i < output->wm_num; i++) {
			vfe->ops_gen1->wm_set_ping_addr(vfe, output->wm_idx[i], new_addr[i]);
		}
	} else if (vfe31_single_buffer) {
		/* Single-buffer mode: update both PING and PONG with same address */
		dev_dbg(vfe->camss->dev,
			"VFE31: wm_done wm=%d single-buffer mode, updating both PING+PONG\n",
			wm);
		for (i = 0; i < output->wm_num; i++) {
			vfe->ops_gen1->wm_set_ping_addr(vfe, output->wm_idx[i], new_addr[i]);
			vfe->ops_gen1->wm_set_pong_addr(vfe, output->wm_idx[i], new_addr[i]);
		}
	} else if (active_index) {
		dev_info(vfe->camss->dev,
			"VFE31: wm_done wm=%d PP=0x%x active=%d → PING complete, updating PING with 0x%08x, seq=%d\n",
			wm, ping_pong, active_index, (u32)new_addr[0], output->sequence - 1);
		for (i = 0; i < output->wm_num; i++) {
			dev_info(vfe->camss->dev, "VFE31: wm_set_ping_addr WM%d addr=0x%08x\n",
				output->wm_idx[i], (u32)new_addr[i]);
			vfe->ops_gen1->wm_set_ping_addr(vfe, output->wm_idx[i], new_addr[i]);
			/*
			 * NOTE: Do NOT call bus_reload_wm() here!
			 * webOS doesn't reload after writing addresses during streaming.
			 * The hardware picks up new addresses automatically on next frame.
			 * Reloading may interfere with ping-pong operation.
			 */
		}
	} else {
		dev_info(vfe->camss->dev,
			"VFE31: wm_done wm=%d PP=0x%x active=%d → PONG complete, updating PONG with 0x%08x, seq=%d\n",
			wm, ping_pong, active_index, (u32)new_addr[0], output->sequence - 1);
		for (i = 0; i < output->wm_num; i++) {
			dev_info(vfe->camss->dev, "VFE31: wm_set_pong_addr WM%d addr=0x%08x\n",
				output->wm_idx[i], (u32)new_addr[i]);
			vfe->ops_gen1->wm_set_pong_addr(vfe, output->wm_idx[i], new_addr[i]);
		}
	}

	/*
	 * VFE31 SWAP mode (webOS V4L2 workaround):
	 * After updating addresses, swap the contents of PING and PONG registers.
	 *
	 * This mirrors the CONFIG_MSM_CAMERA_V4L2 code path in webOS msm_vfe31.c.
	 * If VFE31 hardware only writes to PING regardless of PP status, this
	 * swap ensures different buffers are used for each frame:
	 *   - Frame N: HW writes to PING(A)
	 *   - Swap: PING←B, PONG←A
	 *   - Frame N+1: HW writes to PING(B)
	 *   - Swap: PING←A, PONG←B
	 *   - etc.
	 */
	if (vfe31_swap_pingpong && !vfe31_single_buffer && !vfe31_ping_only) {
		for (i = 0; i < output->wm_num; i++) {
			u8 wm_idx = output->wm_idx[i];
			u32 cur_ping = readl_relaxed(vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm_idx));
			u32 cur_pong = readl_relaxed(vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm_idx));

			/* Swap: write PONG value to PING, PING value to PONG */
			writel_relaxed(cur_pong, vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm_idx));
			writel_relaxed(cur_ping, vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm_idx));

			dev_info(vfe->camss->dev,
				"VFE31: SWAP WM%d: PING 0x%08x↔PONG 0x%08x\n",
				wm_idx, cur_ping, cur_pong);
		}
	}

	/*
	 * Debug: Dump current WM register state after buffer update.
	 * This helps verify the hardware actually has the addresses we wrote.
	 */
	{
		u8 y_wm = output->wm_idx[0];
		u32 hw_ping = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(y_wm));
		u32 hw_pong = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(y_wm));
		u32 hw_pp = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);
		dev_info(vfe->camss->dev,
			"VFE31: wm_done complete: WM%d HW_PING=0x%08x HW_PONG=0x%08x PP=0x%x returned_buf=0x%08x seq=%d\n",
			y_wm, hw_ping, hw_pong, hw_pp, (u32)ready_buf->addr[0], output->sequence - 1);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	if (output->state == VFE_OUTPUT_STOPPING)
		output->last_buffer = ready_buf;
	else {
		/*
		 * VFE31 cache coherency fix:
		 * The vb2_dma_sg memops uses DMA_ATTR_SKIP_CPU_SYNC, which
		 * defers cache synchronization. While vb2_buffer_done() should
		 * call the finish memop to sync, testing shows PONG buffers
		 * still have stale cache data (devmem shows valid physical
		 * data but userspace reads zeros).
		 *
		 * Force explicit cache invalidation before returning buffer
		 * to ensure CPU sees the DMA-written data.
		 */
		struct vb2_buffer *vb = &ready_buf->vb.vb2_buf;
		unsigned int plane;

		for (plane = 0; plane < vb->num_planes; plane++) {
			dma_addr_t addr = ready_buf->addr[plane];
			size_t size = vb2_plane_size(vb, plane);

			dma_sync_single_for_cpu(vfe->camss->dev, addr, size,
						DMA_FROM_DEVICE);
		}

		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static irqreturn_t vfe31_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	static ktime_t first_irq_time;
	static int irq_count;
	static u32 last_ping_pong;  /* Track PP transitions between IRQs */
	ktime_t now;
	u32 value0, value1, ping_pong;
	int i;

	vfe->res->hw_ops->isr_read(vfe, &value0, &value1);

	/* Read ping-pong status to see if data is reaching AXI bus */
	ping_pong = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	irq_count++;
	now = ktime_get();
	if (irq_count == 1) {
		first_irq_time = now;
		last_ping_pong = ping_pong;  /* Initialize on first IRQ */
	}

	/*
	 * Note: Per-frame debug logging removed to prevent soft lockups.
	 * The console subsystem can't keep up with ~30fps dev_info() calls.
	 * Use trace events or dynamic debug for per-frame diagnostics.
	 */

	/*
	 * VFE31 frame completion: Use IMAGE_COMPOSITE_DONE interrupts.
	 *
	 * WebOS uses COMPOSITE_DONE for frame completion:
	 * - COMPOSITE_DONE_0 (bit 21): Fires when all WMs in Group 0 complete
	 * - COMPOSITE_DONE_2 (bit 23): Fires when all WMs in Group 2 complete
	 *
	 * With IRQ_COMPOSITE_MASK=0x00220011:
	 * - Group 0: WM0 + WM4 (PIX Y + CbCr) → COMPOSITE_DONE_0
	 * - Group 2: WM1 + WM5 (VIDEO Y + CbCr) → COMPOSITE_DONE_2
	 *
	 * When COMPOSITE_DONE fires, read PP status to determine which buffer
	 * (PING or PONG) just completed. This is how webOS handles it.
	 *
	 * NOTE: Previous implementation used PP transition detection which
	 * had issues with stride=640 causing half-frame capture. Switching
	 * to COMPOSITE_DONE like webOS should fix this.
	 */

	/* Handle IMAGE_COMPOSITE_DONE_0 (PIX line: WM0+WM4) */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(0)) {
		if (vfe->wm_output_map[0] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, 0, ping_pong);
	}

	/* Handle IMAGE_COMPOSITE_DONE_2 (VIDEO line: WM1+WM5) */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(2)) {
		if (vfe->wm_output_map[1] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, 1, ping_pong);
	}

	/* Debug: dump WM0 registers on first few IRQs to verify DMA config */
	if (irq_count <= 10) {
		u32 wm0_cfg = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(0));
		u32 wm0_ping = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(0));
		u32 wm0_pong = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(0));
		u32 wm0_size = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(0));
		u32 wm0_ub = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(0));
		u32 axi_mode = readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		u32 xbar_cfg1 = readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG1);

		dev_info(vfe->camss->dev,
			 "VFE IRQ#%d S0=0x%08x S1=0x%08x PP=0x%x PING=0x%08x PONG=0x%08x\n",
			 irq_count, value0, value1, ping_pong, wm0_ping, wm0_pong);
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

	/*
	 * Handle RDI mode via COMPOSITE_DONE_1.
	 * RDI uses WM0 mapped to group 1, so COMPOSITE_DONE_1 fires.
	 */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(1)) {
		/* RDI mode: WM0 in group 1 */
		if (vfe->wm_output_map[0] != VFE_LINE_NONE) {
			enum vfe_line_id line_id = vfe->wm_output_map[0];
			/* Only process if this is an RDI line */
			if (line_id >= VFE_LINE_RDI0 && line_id <= VFE_LINE_RDI2)
				vfe31_wm_done(vfe, 0, ping_pong);
		}
	}

	/* Track last PP status for debug purposes */
	last_ping_pong = ping_pong;

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
	u16 width, height, cbcr_height, bytesperline, wpl;
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
	 * VFE31 WM assignments:
	 *   - PIX line:   WM0 (Y) + WM1 (CbCr)
	 *   - VIDEO line: WM4 (Y) + WM1 (CbCr, shared)
	 *
	 * For raw/RDI mode (AXI=0x60), only WM0 is needed.
	 * For PIX mode with DEMUX (AXI=0x01), we need both Y and CbCr WMs
	 * because DEMUX separates Y and CbCr internally.
	 *
	 * Note: Even for "packed" UYVY format, the VFE31 DEMUX outputs
	 * Y and CbCr to separate WMs. The CbCr is interleaved (CbYCrY).
	 */
	/*
	 * VFE31 PIX mode uses DEMUX which separates Y and CbCr to different
	 * Write Masters. This ONLY works with semi-planar formats (NV16/NV61)
	 * where the buffer has separate Y and CbCr planes.
	 *
	 * For packed YUV formats (UYVY/YUYV/etc), the buffer is sized for a
	 * single interleaved plane. Using 2 WMs would cause buffer overflow
	 * because VFE writes Y_size + CbCr_size = 2x the buffer allocation.
	 *
	 * Solution: Force RDI mode (single WM passthrough) for packed formats.
	 * RDI mode bypasses DEMUX and writes raw sensor data directly.
	 */
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
		/*
		 * Packed format: Force RDI mode to avoid buffer overflow.
		 * PIX mode with 2 WMs would write NV16 layout (5MB at 1280x1024)
		 * but UYVY buffer is only sized for packed layout (2.5MB).
		 */
		if (axi_mode == 0x01) {
			dev_info(vfe->camss->dev,
				 "VFE31: Packed format %c%c%c%c - switching to RDI mode (single WM)\n",
				 (pix->pixelformat >> 0) & 0xff,
				 (pix->pixelformat >> 8) & 0xff,
				 (pix->pixelformat >> 16) & 0xff,
				 (pix->pixelformat >> 24) & 0xff);
			axi_mode = 0x60;  /* Force RDI passthrough */
		}
		output->wm_num = 1;
		dev_info(vfe->camss->dev, "VFE31: Packed format - using 1 WM (RDI passthrough)\n");
		break;

	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		/*
		 * Semi-planar format: PIX mode with 2 WMs.
		 * Both 0x01 (OUTPUT_1_AND_3) and 0x200 (OUTPUT_2) support 2 WMs.
		 * 0x200 is the webOS preview mode with WM0(Y)+WM1(CbCr).
		 */
		if (axi_mode == 0x01 || axi_mode == 0x200) {
			output->wm_num = 2;
			dev_info(vfe->camss->dev, "VFE31: Semi-planar NV16/NV61 - using 2 WMs (Y+CbCr) axi=0x%x\n", axi_mode);
		} else {
			/* RDI mode requested, use single WM */
			output->wm_num = 1;
			dev_info(vfe->camss->dev, "VFE31: NV16 with RDI mode - using 1 WM\n");
		}
		break;

	default:
		/* Other formats: follow axi_mode setting */
		if (axi_mode == 0x01) {
			output->wm_num = 2;
			dev_info(vfe->camss->dev, "VFE31: PIX mode - using 2 WMs (Y+CbCr)\n");
		} else {
			output->wm_num = 1;
			dev_info(vfe->camss->dev, "VFE31: Raw/RDI mode - using 1 WM\n");
		}
		break;
	}

	/*
	 * VFE31 WM assignment (from webOS msm_vfe31.c lines 719-722):
	 * - VFE_LINE_PIX:   WM0 (Y) + WM4 (CbCr)  [output0.ch0 + output0.ch1]
	 * - VFE_LINE_VIDEO: WM1 (Y) + WM5 (CbCr)  [output2.ch0 + output2.ch1]
	 *
	 * This is OUTPUT_1_AND_3 mode with "offset-by-4" CbCr channel pairing.
	 * XBAR routing determines which WMs receive Y vs CbCr data.
	 */
	if (line->id == VFE_LINE_VIDEO) {
		/* VIDEO line: Use module params for WM selection */
		u8 video_y_wm = (u8)vfe31_video_y_wm;
		u8 video_cbcr_wm = (u8)vfe31_video_cbcr_wm;

		wm_idx = vfe_reserve_wm_specific(vfe, video_y_wm, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for VIDEO Y\n",
				video_y_wm);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		if (output->wm_num == 2) {
			/*
			 * VIDEO line CbCr: Use module param vfe31_video_cbcr_wm
			 *
			 * IMPORTANT: Reserve the CbCr WM but do NOT map it to
			 * this line! Both WMs share the same frame buffer. If
			 * we map CbCr WM to the line, vfe_isr_comp_done() will
			 * call wm_done() for BOTH WMs, causing double buffer
			 * processing and corrupted CbCr data (green frames).
			 * Only the Y WM triggers buffer completion.
			 */
			wm_idx = vfe_reserve_wm_specific(vfe, video_cbcr_wm, line->id);
			if (wm_idx < 0) {
				dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for VIDEO CbCr\n",
					video_cbcr_wm);
				vfe_release_wm(vfe, output->wm_idx[0]);
				output->state = VFE_OUTPUT_OFF;
				spin_unlock_irqrestore(&vfe->output_lock, flags);
				return wm_idx;
			}
			output->wm_idx[1] = wm_idx;
			/*
			 * Clear the line mapping for CbCr WM - it's claimed but
			 * shouldn't trigger buffer completion. The wm_done()
			 * handler checks wm_output_map and skips WMs mapped to
			 * VFE_LINE_NONE.
			 */
			vfe->wm_output_map[wm_idx] = VFE_LINE_NONE;
		}
		dev_info(vfe->camss->dev, "VFE31: VIDEO line using WM%d(Y), WM%d(CbCr) [params: y=%d cbcr=%d]\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1,
			 vfe31_video_y_wm, vfe31_video_cbcr_wm);
	} else if (line->id == VFE_LINE_PIX) {
		/* PIX line: Use module params for WM selection */
		u8 pix_y_wm = (u8)vfe31_pix_y_wm;
		u8 pix_cbcr_wm = (u8)vfe31_pix_cbcr_wm;

		wm_idx = vfe_reserve_wm_specific(vfe, pix_y_wm, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for PIX Y\n",
				pix_y_wm);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		if (output->wm_num == 2) {
			/*
			 * PIX line CbCr: Use module param vfe31_pix_cbcr_wm
			 *
			 * XBAR routing determines where CbCr data goes:
			 *   - WM1 matches XBAR 0x1A13/0x1A1B CbCr routing
			 *   - WM4 was legacy assumption (doesn't match XBAR)
			 *
			 * IMPORTANT: Reserve the CbCr WM but do NOT map it to
			 * this line! Both WMs share the same frame buffer. If
			 * we map CbCr WM to the line, vfe_isr_comp_done() will
			 * call wm_done() for BOTH WMs, causing double buffer
			 * processing. Only WM0 (Y) triggers buffer completion.
			 */
			wm_idx = vfe_reserve_wm_specific(vfe, pix_cbcr_wm, line->id);
			if (wm_idx < 0) {
				dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for PIX CbCr\n",
					pix_cbcr_wm);
				vfe_release_wm(vfe, output->wm_idx[0]);
				output->state = VFE_OUTPUT_OFF;
				spin_unlock_irqrestore(&vfe->output_lock, flags);
				return wm_idx;
			}
			output->wm_idx[1] = wm_idx;
			/*
			 * Clear the line mapping for CbCr WM - it's claimed but
			 * shouldn't trigger buffer completion. The wm_done()
			 * handler checks wm_output_map and skips WMs mapped to
			 * VFE_LINE_NONE.
			 */
			vfe->wm_output_map[wm_idx] = VFE_LINE_NONE;
		}
		dev_info(vfe->camss->dev, "VFE31: PIX line using WM%d(Y), WM%d(CbCr) [params: y=%d cbcr=%d]\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1,
			 vfe31_pix_y_wm, vfe31_pix_cbcr_wm);
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
	 * CbCr height depends on format (controllable via vfe31_force_422):
	 * - 4:2:0 (NV12/NV21): CbCr height = Y height / 2
	 * - 4:2:2 (NV16/NV61): CbCr height = Y height (full)
	 */
	cbcr_height = vfe31_calc_cbcr_height(pix->pixelformat, height);
	dev_info(vfe->camss->dev,
		 "VFE31: format=0x%x cbcr_height=%d (Y height=%d) force_422=%d\n",
		 pix->pixelformat, cbcr_height, height, vfe31_force_422);

	/*
	 * Semi-planar output: DEMUX separates UYVY input into Y and CbCr planes.
	 * Each output plane has width bytes per line:
	 * - WM0 writes Y plane: 640 bytes/line @ 640x480
	 * - WM1 writes CbCr plane: 640 bytes/line @ cbcr_height
	 *
	 * Use module param if set, otherwise use V4L2 format bytesperline.
	 */
	if (vfe31_bytesperline > 0)
		bytesperline = vfe31_bytesperline;
	else
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

	/*
	 * Single-buffer mode workaround: set PONG = PING
	 * This ensures all frames have valid data at the cost of true double-buffering.
	 * Useful if PONG frames are consistently empty (hardware issue).
	 */
	if (vfe31_single_buffer) {
		dev_info(vfe->camss->dev,
			 "VFE31: Single-buffer mode enabled - PONG=PING=0x%08x\n",
			 ping_addr);
		pong_addr = ping_addr;
	}

	/* Verify addresses are valid and different (unless single-buffer mode) */
	if (!vfe31_single_buffer && ping_addr == pong_addr) {
		dev_warn(vfe->camss->dev,
			 "VFE31: WARNING - PING and PONG have same address 0x%08x!\n",
			 ping_addr);
	}

	dev_info(vfe->camss->dev,
		 "VFE31: WM%d %ux%u stride=%u ping=0x%08x pong=0x%08x%s\n",
		 wm, width, height, bytesperline, ping_addr, pong_addr,
		 vfe31_single_buffer ? " (single-buffer)" : "");

	/*
	 * Store addresses in pending_* for vfe31_start_camif_for_rdi() which
	 * will write them to hardware after CSIPHY is configured.
	 * Also store in last_y_* for runtime CbCr address calculation.
	 */
	vfe->pending_ping_addr = ping_addr;
	vfe->pending_pong_addr = pong_addr;
	vfe->last_y_ping_addr = ping_addr;
	vfe->last_y_pong_addr = pong_addr;

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
	writel_relaxed(vfe31_get_bus_cfg(), vfe->base + VFE_0_BUS_CFG);
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
	 * Both 0x01 (OUTPUT_1_AND_3) and 0x200 (OUTPUT_2) use the ISP pipeline.
	 * For RDI mode (axi=0x60), data bypasses the ISP entirely.
	 */
	if (axi_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE || axi_mode == 0x200) {
		dev_info(vfe->camss->dev, "VFE31: Step 1b - Configure demux/scale/crop (axi=0x%x)\n", axi_mode);
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
	{
		u32 readback = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
		dev_info(vfe->camss->dev, "VFE31: WM%d PING_ADDR=0x%08x (readback=0x%08x)\n",
			wm, ping_addr, readback);
	}

	/* WR_PONG_ADDR */
	writel_relaxed(pong_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
	{
		u32 readback = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
		dev_info(vfe->camss->dev, "VFE31: WM%d PONG_ADDR=0x%08x (readback=0x%08x)\n",
			wm, pong_addr, readback);
	}

	/*
	 * WR_IMAGE_SIZE - VFE31 format (from webOS register dumps):
	 * webOS WM0: 0x00501DF2 = ((80) << 16) | ((479 << 4) | 2)
	 *
	 * Upper 16 bits: stride / 16 (128-bit words per line)
	 * Lower 16 bits: ((height - 1) << 4) | 2
	 *
	 * NOTE: Previous workaround forced input stride (1280) for NV12 to avoid
	 * half-frame capture with PP transition detection. With COMPOSITE_DONE
	 * based frame completion, output stride (640) should work correctly.
	 * Use vfe31_nv12_stride_fix module param to test both modes.
	 */
	{
		u16 image_stride = vfe31_calc_image_stride(width, bytesperline,
							   is_rdi_line, pix->pixelformat);
		int img_height_val;
		bool is_nv12 = (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
				pix->pixelformat == V4L2_PIX_FMT_NV21);

		/*
		 * NV12 stride workaround (controlled by vfe31_nv12_stride_fix):
		 * When enabled, force input stride for NV12 to avoid half-frame.
		 * When disabled, use configured stride (test COMPOSITE_DONE fix).
		 */
		if (vfe31_nv12_stride_fix && is_nv12 && !is_rdi_line && image_stride < width * 2) {
			dev_info(vfe->camss->dev,
				 "VFE31: NV12 stride workaround enabled, forcing stride=%d→%d\n",
				 image_stride, width * 2);
			image_stride = width * 2;
		}

		if (vfe31_pix_y_img_height < 0)
			img_height_val = height;  /* auto */
		else
			img_height_val = vfe31_pix_y_img_height;  /* explicit */

		reg = ((image_stride / 16) & 0xFFFF) << 16;
		reg |= ((img_height_val - 1) << 4) | 2;
		dev_info(vfe->camss->dev, "VFE31: WM%d IMAGE_SIZE stride=%d height=%d (s_param=%d h_param=%d%s)\n",
			 wm, image_stride, img_height_val, vfe31_image_stride, vfe31_pix_y_img_height,
			 is_nv12 ? " NV12" : "");
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));
	}

	/*
	 * WR_ADDR_CFG - webOS format: (lines << 16) | burst_words
	 * webOS WM0: 0x0000012F = (lines=0 << 16) | burst=303
	 * webOS WM1: 0x01C8012F = (lines=456 << 16) | burst=303
	 *
	 * For single-plane UYVY, only WM0 is used with lines=0.
	 * burst_words = words_per_line - 17 (webOS formula)
	 *
	 * Use bytesperline (OUTPUT stride) to match buffer allocation.
	 * For 640x480 NV16: bytesperline = 640 bytes = 160 words, burst = 143
	 * For 1280x1024 NV16: bytesperline = 1280 bytes = 320 words, burst = 303
	 */
	{
		int lines_val, burst_val;

		wpl = bytesperline / 4;  /* 32-bit words per line from buffer stride */

		/* Lines field: 0 for single-plane, height-24 for multi-plane Y */
		if (vfe31_pix_y_lines < 0)
			lines_val = 0;  /* auto: lines=0 for Y WM (webOS behavior) */
		else
			lines_val = vfe31_pix_y_lines;  /* explicit */

		/* Burst field */
		if (vfe31_pix_y_burst < 0)
			burst_val = (wpl - 17) & 0xFFFF;  /* auto: webOS formula */
		else
			burst_val = vfe31_pix_y_burst & 0xFFFF;  /* explicit */

		reg = (lines_val << 16) | burst_val;
		dev_info(vfe->camss->dev, "VFE31: WM%d WR_ADDR_CFG=0x%08x (lines=%d, burst=%d, l_param=%d, b_param=%d)\n",
			 wm, reg, lines_val, burst_val, vfe31_pix_y_lines, vfe31_pix_y_burst);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));
	}

	/*
	 * WR_UB_CFG - VFE31 format (from webOS register dumps):
	 * webOS WM0: 0x002701DF = ((39) << 16) | 479
	 *
	 * Upper 16 bits: (wpl / 8) - 1, where wpl is 32-bit words per line
	 *   For 1280 bytes/line: wpl = 320, (320/8)-1 = 39 = 0x27
	 * Lower 16 bits: height - 1
	 *
	 * CRITICAL: UB buffers INCOMING data BEFORE DEMUX separates Y/CbCr!
	 * - PIX/VIDEO mode (UYVY input): use width * 2 (2 bytes per pixel)
	 * - RDI mode (RAW8 input): use bytesperline (1 byte per pixel)
	 *
	 * For PIX/VIDEO: Using output stride (640) instead of input stride (1280)
	 * causes UB to only buffer half the input line, resulting in half-frame
	 * capture (only 240 of 480 lines).
	 *
	 * This is DIFFERENT from VFE4.x which uses (offset << 16) | depth
	 */
	{
		u16 input_stride;
		u16 input_wpl;
		int ub_height_val;

		if (is_rdi_line) {
			/* RDI mode: RAW data, use bytesperline (1 byte per pixel for RAW8) */
			input_stride = bytesperline;
		} else {
			/* PIX/VIDEO mode: UYVY input, 2 bytes per pixel */
			input_stride = width * 2;
		}
		input_wpl = input_stride / 4;  /* 32-bit words per line */

		if (vfe31_pix_y_ub_height < 0)
			ub_height_val = height - 1;  /* auto */
		else
			ub_height_val = vfe31_pix_y_ub_height;  /* explicit */

		reg = ((input_wpl / 8 - 1) & 0xFFFF) << 16;
		reg |= ub_height_val & 0xFFFF;
		dev_info(vfe->camss->dev, "VFE31: WM%d UB_CFG=0x%08x (ub_depth=%d, ub_height=%d, %s, param=%d)\n",
			 wm, reg, (input_wpl / 8 - 1), ub_height_val,
			 is_rdi_line ? "RDI" : "PIX/VIDEO", vfe31_pix_y_ub_height);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
	}

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
		/*
		 * CbCr plane offset: Must use V4L2 bytesperline to match buffer layout.
		 * The V4L2 buffer has Y at offset 0 with stride=bytesperline, so
		 * CbCr starts at bytesperline * height.
		 *
		 * For NV12 640x480: Y=640*480=307200, CbCr starts at 307200
		 * For NV16 640x480: Y=640*480=307200, CbCr starts at 307200
		 *
		 * This is independent of VFE IMAGE_SIZE stride which controls
		 * how the hardware writes each line internally.
		 */
		u32 cbcr_offset = vfe31_get_cbcr_offset(pix->pixelformat, bytesperline, height);
		u32 wm1_ping_addr = ping_addr + cbcr_offset;
		u32 wm1_pong_addr = pong_addr + cbcr_offset;

		/* Store for runtime CbCr address calculation during streaming */
		vfe->active_cbcr_offset = cbcr_offset;

		dev_info(vfe->camss->dev,
			 "VFE31: WM%d (CbCr) offset=0x%x (bpl=%d h=%d) ping=0x%08x\n",
			 wm1, cbcr_offset, bytesperline, height, wm1_ping_addr);

		/* WM1 PING/PONG addresses (CbCr buffer after Y) */
		writel_relaxed(wm1_ping_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm1));
		writel_relaxed(wm1_pong_addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm1));

		/* CbCr WM IMAGE_SIZE - use cbcr_height for NV12 */
		{
			u16 cbcr_stride = vfe31_calc_cbcr_stride(width, bytesperline);
			int img_height_val;

			if (vfe31_pix_cbcr_img_height < 0)
				img_height_val = cbcr_height;  /* auto: use format-based height */
			else
				img_height_val = vfe31_pix_cbcr_img_height;  /* explicit */

			reg = ((cbcr_stride / 16) & 0xFFFF) << 16;
			reg |= ((img_height_val - 1) << 4) | 2;
			dev_info(vfe->camss->dev,
				 "VFE31: WM%d IMAGE_SIZE=0x%08x (stride=%d height=%d s_param=%d h_param=%d)\n",
				 wm1, reg, cbcr_stride, img_height_val, vfe31_cbcr_stride, vfe31_pix_cbcr_img_height);
			writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm1));
		}

		/*
		 * CbCr WM ADDR_CFG - burst and line count
		 *
		 * Format: (lines << 16) | burst_words
		 * Use bytesperline to match buffer allocation.
		 *
		 * Lines field controlled by vfe31_pix_cbcr_lines parameter:
		 *   -1 = auto (cbcr_height - 24 based on webOS formula)
		 *    0 = cbcr_height - 1 (full CbCr height)
		 *   >0 = explicit value
		 */
		{
			int lines_val, burst_val;

			if (vfe31_pix_cbcr_lines < 0)
				lines_val = cbcr_height - 24;  /* auto: webOS formula */
			else if (vfe31_pix_cbcr_lines == 0)
				lines_val = cbcr_height - 1;   /* full CbCr height */
			else
				lines_val = vfe31_pix_cbcr_lines;  /* explicit */

			wpl = bytesperline / 4;  /* 32-bit words per line from buffer stride */
			if (vfe31_pix_cbcr_burst < 0)
				burst_val = (wpl - 17) & 0xFFFF;  /* auto */
			else
				burst_val = vfe31_pix_cbcr_burst & 0xFFFF;  /* explicit */

			reg = (lines_val << 16) | burst_val;
			dev_info(vfe->camss->dev,
				 "VFE31: WM%d WR_ADDR_CFG=0x%08x (lines=%d, burst=%d, l_param=%d, b_param=%d)\n",
				 wm1, reg, lines_val, burst_val, vfe31_pix_cbcr_lines, vfe31_pix_cbcr_burst);
			writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm1));
		}

		/*
		 * CbCr WM UB_CFG configuration:
		 * - ub_depth: Use input stride (UYVY, same as Y WM) since UB
		 *   buffers incoming data before DEMUX separates Y and CbCr.
		 * - ub_height: Use cbcr_height (NOT Y height!) since this controls
		 *   the OUTPUT frame height for this write master.
		 *   For NV12 4:2:0, cbcr_height = Y_height/2.
		 */
		{
			u16 input_stride = width * 2;  /* UYVY input: 2 bytes per pixel */
			u16 input_wpl = input_stride / 4;  /* 32-bit words per line */
			int ub_height_val;

			/*
			 * FIX: Use cbcr_height for CbCr WM UB_CFG, not Y height!
			 * For NV12 4:2:0, cbcr_height = height/2 = 240 for 480p.
			 * Using Y height (480) caused only 120/240 UV lines to be written.
			 */
			if (vfe31_pix_cbcr_ub_height < 0)
				ub_height_val = cbcr_height - 1;  /* auto: use CbCr height */
			else
				ub_height_val = vfe31_pix_cbcr_ub_height;  /* explicit */

			reg = ((input_wpl / 8 - 1) & 0xFFFF) << 16;
			reg |= ub_height_val & 0xFFFF;
			dev_info(vfe->camss->dev,
				 "VFE31: WM%d UB_CFG=0x%08x (ub_depth=%d, ub_height=%d, cbcr_h=%d, param=%d)\n",
				 wm1, reg, (input_wpl / 8 - 1), ub_height_val, cbcr_height, vfe31_pix_cbcr_ub_height);
			writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm1));
		}

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
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	unsigned long flags;
	unsigned int i;
	bool is_rdi = (line->id == VFE_LINE_RDI0 ||
		       line->id == VFE_LINE_RDI1 ||
		       line->id == VFE_LINE_RDI2);

	/*
	 * VFE31 RDI mode: Skip SOF and REG_UPDATE waits.
	 *
	 * In RDI/raw bypass mode (AXI=0x60), data goes directly from
	 * CAMIF to memory without ISP processing. The ISP doesn't
	 * generate SOF or REG_UPDATE IRQs in this mode, so waiting
	 * for them would always timeout.
	 *
	 * For PIX/VIDEO lines, use the standard gen1 disable path.
	 */
	if (!is_rdi)
		return vfe_gen1_disable(line);

	dev_info(vfe->camss->dev,
		 "VFE31: RDI disable (skipping SOF/REG_UPDATE wait)\n");

	spin_lock_irqsave(&vfe->output_lock, flags);

	/* Disable write masters */
	for (i = 0; i < output->wm_num; i++)
		vfe->ops_gen1->wm_enable(vfe, output->wm_idx[i], 0);

	/* For RDI: disable frame-based mode and disconnect WM */
	vfe->ops_gen1->wm_frame_based(vfe, output->wm_idx[0], 0);
	vfe->ops_gen1->bus_disconnect_wm_from_rdi(vfe, output->wm_idx[0], line->id);
	vfe->ops_gen1->enable_irq_wm_line(vfe, output->wm_idx[0], line->id, 0);
	vfe->ops_gen1->set_cgc_override(vfe, output->wm_idx[0], 0);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
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
	 * Apply module param overrides if set.
	 * These allow testing different DEMUX routing without rebuilding.
	 */
	if (vfe31_demux_even > 0)
		even_cfg = vfe31_demux_even;
	if (vfe31_demux_odd > 0)
		odd_cfg = vfe31_demux_odd;

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
	/*
	 * VFE31 writes the combined 16-bit value to both EVEN and ODD registers.
	 * This was the working state at 20:04 CET - separate 8-bit values broke
	 * the capture. webOS also uses this combined format (0xC9CA for UYVY).
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
	/*
	 * Chroma vertical scaling - controllable via module params.
	 * vfe31_force_422 affects the auto mode, vfe31_chroma_v_out overrides.
	 */
	{
		u32 v_out, v_phase, subs_cfg;

		/* Determine output height */
		if (vfe31_chroma_v_out < 0) {
			/* Auto: based on format (respects vfe31_force_422) */
			v_out = vfe31_calc_cbcr_height(p, height);
		} else if (vfe31_chroma_v_out == 0) {
			v_out = height / 2;  /* Force 2:1 */
		} else {
			v_out = vfe31_chroma_v_out;  /* Explicit */
		}

		/* Determine phase */
		if (vfe31_chroma_v_phase < 0) {
			/* Auto: based on scaling ratio */
			v_phase = (v_out < height) ? 0x00320000 : 0x00310000;
		} else {
			v_phase = vfe31_chroma_v_phase;
		}

		/* Determine subs_cfg */
		subs_cfg = (vfe31_chroma_subs_cfg < 0) ? 0x30 : vfe31_chroma_subs_cfg;

		writel_relaxed((v_out << 16) | height, vfe->base + VFE_0_CHROMA_V_IMAGE);
		writel_relaxed(v_phase, vfe->base + VFE_0_CHROMA_V_PHASE);
		writel_relaxed(subs_cfg, vfe->base + VFE_0_CHROMA_SUBS_CFG);

		dev_info(vfe->camss->dev,
			 "VFE31: CHROMA_V params: v_out=%d, phase=0x%x, subs=0x%x (params: %d, %d, %d)\n",
			 v_out, v_phase, subs_cfg,
			 vfe31_chroma_v_out, vfe31_chroma_v_phase, vfe31_chroma_subs_cfg);
	}

	/* Debug: readback and log actual register values */
	{
		u32 v_image = readl_relaxed(vfe->base + VFE_0_CHROMA_V_IMAGE);
		u32 v_phase = readl_relaxed(vfe->base + VFE_0_CHROMA_V_PHASE);
		u32 h_image = readl_relaxed(vfe->base + VFE_0_CHROMA_H_IMAGE);
		u32 subs_cfg = readl_relaxed(vfe->base + VFE_0_CHROMA_SUBS_CFG);

		dev_info(vfe->camss->dev,
			 "VFE31: CHROMA_V_IMAGE=0x%08x (out=%d, in=%d), V_PHASE=0x%08x\n",
			 v_image, v_image >> 16, v_image & 0xFFFF, v_phase);
		dev_info(vfe->camss->dev,
			 "VFE31: CHROMA_H_IMAGE=0x%08x (out=%d, in=%d), SUBS_CFG=0x%08x\n",
			 h_image, h_image >> 16, h_image & 0xFFFF, subs_cfg);
	}

	dev_info(vfe->camss->dev,
		 "VFE31: Scale/FOV configured: %ux%u, format=0x%x, chroma_v=%s (force_422=%d)\n",
		 width, height, p,
		 vfe31_is_420_format(p) ? "2:1 (4:2:0)" : "1:1 (4:2:2)",
		 vfe31_force_422);
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
	wmb();
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm),
		       vfe->base + VFE_0_BUS_CMD);
	wmb();
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

	/* For CbCr plane, use format-appropriate height (respects vfe31_force_422) */
	if (plane == 1)
		*height = vfe31_calc_cbcr_height(pix->pixelformat, *height);
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
	bool is_rdi_line;
	u32 axi_mode;

	if (line_id == VFE_LINE_NONE || line_id >= vfe->res->line_num) {
		dev_err(vfe->camss->dev, "VFE31: Invalid line_id %d for WM%d\n",
			line_id, wm);
		return;
	}

	line = &vfe->line[line_id];

	/*
	 * Determine AXI mode based on line type:
	 * - RDI lines (RDI0, RDI1, RDI2) MUST use 0x60 (raw bypass)
	 * - PIX/VIDEO lines use module parameter (default 0x01 for DEMUX)
	 */
	is_rdi_line = (line_id == VFE_LINE_RDI0 ||
		       line_id == VFE_LINE_RDI1 ||
		       line_id == VFE_LINE_RDI2);

	if (is_rdi_line) {
		axi_mode = VFE_0_BUS_AXI_OUT_MODE_RAW_WM0;  /* 0x60 */
	} else {
		axi_mode = vfe31_axi_output_mode;
	}

	dev_info(vfe->camss->dev,
		 "VFE31: Starting CAMIF for WM%d line%d (fmt %ux%u code=0x%x axi=0x%x)\n",
		 wm, line_id, line->fmt[MSM_VFE_PAD_SINK].width,
		 line->fmt[MSM_VFE_PAD_SINK].height,
		 line->fmt[MSM_VFE_PAD_SINK].code, axi_mode);

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
	if (axi_mode == VFE_0_BUS_AXI_OUT_MODE_RAW_WM0) {
		/* RDI mode (0x60): Raw bypass, no XBAR needed */
		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - RDI mode: BUS_CFG=0x%08x, AXI=0x60 (raw bypass)\n",
			 VFE_0_BUS_CFG_WEBOS_VALUE);
		writel_relaxed(vfe31_get_bus_cfg(), vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(VFE_0_BUS_AXI_OUT_MODE_RAW_WM0,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		/* No XBAR configuration for RDI mode */
	} else {
		/*
		 * PIX mode (0x01): Use XBAR to route DEMUX output to WMs
		 *
		 * VFE31 WM assignments:
		 *   - PIX:   WM0 (Y) + WM1 (CbCr)
		 *   - VIDEO: WM4 (Y) + WM1 (CbCr, shared)
		 *
		 * XBAR routing:
		 *   - 0x1A13 (PIX only):  Y→WM0, CbCr→WM1
		 *   - 0x1A1B (PIX+VIDEO): Y→WM0+WM4, CbCr→WM1
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
		writel_relaxed(vfe31_get_bus_cfg(), vfe->base + VFE_0_BUS_CFG);
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
		 * NV16 output: Use module param if set, otherwise V4L2 format.
		 * DEMUX separates UYVY input into Y and CbCr planes internally.
		 */
		u16 bytesperline = (vfe31_bytesperline > 0) ?
				   vfe31_bytesperline : pix->plane_fmt[0].bytesperline;
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
		 *
		 * CRITICAL: UB buffers UYVY INPUT data (width*2), not NV16 output.
		 */
		wpl = (width * 2) / 4;  /* UYVY INPUT stride */
		reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
		reg |= (height - 1) & 0xFFFF;
		dev_info(vfe->camss->dev,
			 "VFE31: WM%d UB_CFG=0x%08x (ub_depth=%d, input_wpl=%d)\n",
			 wm, reg, (wpl / 8 - 1), wpl);
		writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
		wmb();

		/*
		 * Step 2b: Configure WM1 for CbCr output (semi-planar formats)
		 *
		 * In PIX mode with DEMUX enabled, the VFE separates Y and CbCr.
		 * Use format-aware offset calculation for portability.
		 */
		if (line->output.wm_num == 2) {
			u8 wm1 = line->output.wm_idx[1];
			/*
			 * CbCr offset: Use bytesperline to match V4L2 buffer layout.
			 * Y plane occupies bytesperline * height bytes.
			 */
			u32 cbcr_offset = vfe31_get_cbcr_offset(pix->pixelformat, bytesperline, height);
			u32 wm1_ping = vfe->pending_ping_addr + cbcr_offset;
			u32 wm1_pong = vfe->pending_pong_addr + cbcr_offset;
			u16 cbcr_height;

			/* Store for runtime CbCr address calculation */
			vfe->active_cbcr_offset = cbcr_offset;

			/*
			 * CbCr height depends on format (controllable via vfe31_force_422):
			 * - 4:2:0 (NV12/NV21): CbCr height = Y height / 2
			 * - 4:2:2 (NV16/NV61): CbCr height = Y height (full)
			 */
			cbcr_height = vfe31_calc_cbcr_height(pix->pixelformat, height);

			dev_info(vfe->camss->dev,
				 "VFE31: WM%d (CbCr) offset=0x%x PING=0x%08x cbcr_height=%d force_422=%d\n",
				 wm1, cbcr_offset, wm1_ping, cbcr_height, vfe31_force_422);

			/* WM1 PING/PONG addresses */
			writel_relaxed(wm1_ping,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm1));
			writel_relaxed(wm1_pong,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm1));

			/* CbCr WM IMAGE_SIZE - use cbcr_height for NV12 */
			{
				u16 cbcr_stride = vfe31_calc_cbcr_stride(width, bytesperline);
				reg = ((cbcr_stride / 16) & 0xFFFF) << 16;
				reg |= ((cbcr_height - 1) << 4) | 2;
				dev_info(vfe->camss->dev,
					 "VFE31: WM%d IMAGE_SIZE=0x%08x (stride=%d height=%d)\n",
					 wm1, reg, cbcr_stride, cbcr_height);
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm1));
			}

			/*
			 * WM1 ADDR_CFG - lines and burst for CbCr
			 * Use (cbcr_height - 24) << 16 | burst (webOS formula)
			 *
			 * NOTE: For NV12, cbcr_height=240, so lines=216.
			 * The -24 offset appears to be a timing/safety margin.
			 */
			wpl = bytesperline / 4;
			reg = ((cbcr_height - 24) << 16) | ((wpl - 17) & 0xFFFF);
			dev_info(vfe->camss->dev,
				 "VFE31: WM%d ADDR_CFG=0x%08x (lines=%d, burst=%d)\n",
				 wm1, reg, cbcr_height - 24, (wpl - 17) & 0xFFFF);
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm1));

			/* WM1 UB_CFG - use UYVY input stride like WM0 */
			wpl = (width * 2) / 4;  /* UYVY INPUT stride */
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
		u32 width = line->fmt[MSM_VFE_PAD_SINK].width;
		u32 height = line->fmt[MSM_VFE_PAD_SINK].height;
		u32 code = line->fmt[MSM_VFE_PAD_SINK].code;
		u32 width_bytes;

		/*
		 * Calculate bytes per line based on format:
		 * - RAW8 formats (Bayer): 1 byte per pixel
		 * - RAW10 formats (packed): 10 bits per pixel (5 bytes per 4 pixels)
		 * - YUV422 formats: 2 bytes per pixel
		 */
		switch (code) {
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SRGGB8_1X8:
			width_bytes = width;  /* 1 byte per pixel */
			break;
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			width_bytes = (width * 10 + 7) / 8;  /* 10-bit packed */
			break;
		default:
			width_bytes = width * 2;  /* YUV422: 2 bytes per pixel */
			break;
		}

		dev_info(vfe->camss->dev,
			 "VFE31: Step 3 - CAMIF config (code=0x%x width=%u bpl=%u)\n",
			 code, width, width_bytes);

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

	/*
	 * Configure pixel pattern in CORE_CFG + bit 6 (webOS uses 0x46 for UYVY)
	 *
	 * For RDI/raw bypass mode (AXI=0x60), CORE_CFG is still needed to
	 * configure the input MUX, but pixel pattern doesn't affect raw data.
	 */
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
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY;
		break;
	/* RAW Bayer formats - use default pattern (doesn't affect raw bypass) */
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		val = 0;  /* Raw bypass - pattern not used */
		dev_info(vfe->camss->dev,
			 "VFE31: RAW format detected - CORE_CFG pattern=0\n");
		break;
	default:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
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
			/* Consider the line being started as active */
			bool starting_pix = (line->id == VFE_LINE_PIX);
			bool starting_video = (line->id == VFE_LINE_VIDEO);
			bool video_state_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool pix_state_active = (pix_out->state == VFE_OUTPUT_ON ||
					   pix_out->state == VFE_OUTPUT_RESERVED ||
					   pix_out->state == VFE_OUTPUT_CONTINUOUS);
			bool video_active = starting_video || video_state_active;
			bool pix_active = starting_pix || pix_state_active;

			/* Module param override takes priority */
			if (vfe31_irq_comp_mask != 0) {
				vfe->irq_comp_mask_shadow = vfe31_irq_comp_mask;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (module param)\n",
					 vfe->irq_comp_mask_shadow);
			} else if (video_active && !pix_active) {
				/* VIDEO-only: WM1 (Y) + WM4 (CbCr), no WM0 */
				vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (VIDEO only)\n",
					 vfe->irq_comp_mask_shadow);
			} else if (video_active && pix_active) {
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
	 *   - Bits 13-19: Stats IRQs (AEC, AF, AWB, RS, CS, IHIST, SKIN)
	 *   - Bits 21-23: IMAGE_COMPOSITE_DONE_0-2
	 *
	 * NOTE: Bits 8-13 are NOT per-WM PING_PONG IRQs on VFE31!
	 * They are bus overflow errors (IMG_MAST_n_BUS_OVFL) in STATUS_1.
	 * Frame completion is signaled ONLY via IMAGE_COMPOSITE_DONE.
	 */
	{
		vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
					VFE_0_IRQ_MASK_0_CAMIF_EOF |
					VFE_0_IRQ_MASK_0_REG_UPDATE |
					VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(0) |
					VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(1) |
					VFE_0_IRQ_MASK_0_IMAGE_COMPOSITE_DONE_n(2);
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
	 * VFE31 WM assignments:
	 *   - PIX line:   WM0 (Y) + WM1 (CbCr)
	 *   - VIDEO line: WM4 (Y) + WM1 (CbCr, shared with PIX)
	 *
	 * XBAR 0x1A1B routes: Y→WM0+WM4, CbCr→WM1 only
	 *
	 * Note: This is skipped for RDI lines (axi_mode = 0x60) since they
	 * use raw bypass mode and don't need VIDEO WM configuration.
	 */
	if (axi_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		if (line->id == VFE_LINE_VIDEO) {
			/*
			 * VIDEO line starting - configure WM4 (Y) and WM1 (CbCr)
			 * with VIDEO line's buffer addresses.
			 *
			 * VIDEO uses: WM4 (Y) + WM1 (CbCr)
			 * XBAR 0x1A1B routes: Y→WM0+WM4, CbCr→WM1
			 */
			struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
			u32 width = pix->width;
			u32 height = pix->height;
			u32 cbcr_height;
			/* Use module param if set, same as Step 2 */
			u32 bytesperline = (vfe31_bytesperline > 0) ?
					   vfe31_bytesperline : pix->plane_fmt[0].bytesperline;
			u32 wpl = bytesperline / 4;  /* 32-bit words per line */
			u32 reg;

			/* Calculate CbCr height based on format (controllable via vfe31_force_422) */
			cbcr_height = vfe31_calc_cbcr_height(pix->pixelformat, height);

			dev_info(vfe->camss->dev,
				 "VFE31: Step 8 - VIDEO line: WM%d(Y)/WM%d(CbCr) cbcr_h=%d force_422=%d\n",
				 VFE31_VIDEO_WM_Y, VFE31_VIDEO_WM_CBCR, cbcr_height, vfe31_force_422);

			/* VIDEO Y WM configuration */
			/* Addresses set via wm_set_ping_addr/wm_set_pong_addr */

			/* VIDEO Y WM IMAGE_SIZE */
			{
				u16 image_stride = vfe31_calc_image_stride(width, bytesperline,
									   false, pix->pixelformat);
				int img_height_val;
				bool is_nv12 = (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
						pix->pixelformat == V4L2_PIX_FMT_NV21);

				/*
				 * NV12 stride workaround (controlled by vfe31_nv12_stride_fix).
				 * See PIX mode IMAGE_SIZE comment for detailed explanation.
				 */
				if (vfe31_nv12_stride_fix && is_nv12 && image_stride < width * 2) {
					dev_info(vfe->camss->dev,
						 "VFE31: VIDEO NV12 stride workaround, forcing stride=%d→%d\n",
						 image_stride, width * 2);
					image_stride = width * 2;
				}

				if (vfe31_video_y_img_height < 0)
					img_height_val = height;  /* auto */
				else
					img_height_val = vfe31_video_y_img_height;  /* explicit */

				reg = ((image_stride / 16) & 0xFFFF) << 16;
				reg |= ((img_height_val - 1) << 4) | 2;
				dev_info(vfe->camss->dev,
					 "VFE31: VIDEO WM1 IMAGE_SIZE stride=%d height=%d (h_param=%d%s)\n",
					 image_stride, img_height_val, vfe31_video_y_img_height,
					 is_nv12 ? " NV12" : "");
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(VFE31_VIDEO_WM_Y));
			}

			/*
			 * VIDEO Y WM ADDR_CFG - VFE31 format:
			 * Use bytesperline to match buffer allocation.
			 */
			{
				int lines_val, burst_val;
				u32 buf_wpl = bytesperline / 4;  /* words per line from buffer stride */

				if (vfe31_video_y_lines < 0)
					lines_val = height - 24;  /* auto: same as CbCr */
				else
					lines_val = vfe31_video_y_lines;

				if (vfe31_video_y_burst < 0)
					burst_val = (buf_wpl - 17) & 0xFFFF;  /* auto */
				else
					burst_val = vfe31_video_y_burst & 0xFFFF;  /* explicit */

				reg = (lines_val << 16) | burst_val;
				dev_info(vfe->camss->dev,
					 "VFE31: VIDEO WM1 ADDR_CFG=0x%08x (lines=%d, burst=%d, l_param=%d, b_param=%d)\n",
					 reg, lines_val, burst_val, vfe31_video_y_lines, vfe31_video_y_burst);
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_Y));
			}

			/*
			 * VIDEO Y WM UB_CFG - use bytesperline
			 * Upper 16 bits: (wpl / 8) - 1
			 * Lower 16 bits: height - 1
			 */
			{
				u32 buf_wpl = bytesperline / 4;  /* words per line from buffer stride */
				int ub_height_val;

				if (vfe31_video_y_ub_height < 0)
					ub_height_val = height - 1;  /* auto */
				else
					ub_height_val = vfe31_video_y_ub_height;  /* explicit */

				reg = ((buf_wpl / 8 - 1) & 0xFFFF) << 16;
				reg |= ub_height_val & 0xFFFF;
				dev_info(vfe->camss->dev,
					 "VFE31: VIDEO WM1 UB_CFG=0x%08x (ub_depth=%d, ub_height=%d, param=%d)\n",
					 reg, (buf_wpl / 8 - 1), ub_height_val, vfe31_video_y_ub_height);
			}
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(VFE31_VIDEO_WM_Y));

			/* Enable VIDEO Y WM */
			writel_relaxed(BIT(0),
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
			writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(VFE31_VIDEO_WM_Y),
				       vfe->base + VFE_0_BUS_CMD);

			/*
			 * VIDEO CbCr WM (WM1 - shared with PIX)
			 *
			 * CRITICAL: Must set PING/PONG addresses for CbCr WM!
			 * Use format-aware offset calculation for portability.
			 */
			if (line->output.wm_num == 2) {
				/*
				 * CbCr offset: Use bytesperline to match V4L2 buffer layout.
				 * Y plane occupies bytesperline * height bytes.
				 */
				u32 cbcr_offset = vfe31_get_cbcr_offset(pix->pixelformat, bytesperline, height);
				u32 cbcr_ping = vfe->pending_ping_addr + cbcr_offset;
				u32 cbcr_pong = vfe->pending_pong_addr + cbcr_offset;

				/* Store for runtime CbCr address calculation */
				vfe->active_cbcr_offset = cbcr_offset;

				dev_info(vfe->camss->dev,
					 "VFE31: VIDEO WM%d (CbCr) offset=0x%x (bpl=%d h=%d) PING=0x%08x\n",
					 VFE31_VIDEO_WM_CBCR, cbcr_offset, bytesperline, height, cbcr_ping);

				/* VIDEO CbCr PING/PONG addresses */
				writel_relaxed(cbcr_ping,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(VFE31_VIDEO_WM_CBCR));
				writel_relaxed(cbcr_pong,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(VFE31_VIDEO_WM_CBCR));

				/*
				 * VIDEO CbCr IMAGE_SIZE - use cbcr_height for NV12.
				 *
				 * CRITICAL: CbCr WMs must use bytesperline, NOT input stride!
				 * Using input stride (width*2) causes DMA to write beyond
				 * the buffer, corrupting memory and crashing the kernel.
				 */
				{
					u16 cbcr_stride = vfe31_calc_cbcr_stride(width, bytesperline);
					int img_height_val;

					if (vfe31_video_cbcr_img_height < 0)
						img_height_val = cbcr_height;  /* auto: format-based */
					else
						img_height_val = vfe31_video_cbcr_img_height;  /* explicit */

					reg = ((cbcr_stride / 16) & 0xFFFF) << 16;
					reg |= ((img_height_val - 1) << 4) | 2;
					dev_info(vfe->camss->dev,
						 "VFE31: VIDEO WM5 IMAGE_SIZE stride=%d height=%d (s_param=%d h_param=%d)\n",
						 cbcr_stride, img_height_val, vfe31_cbcr_stride, vfe31_video_cbcr_img_height);
				}
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(VFE31_VIDEO_WM_CBCR));

				/*
				 * VIDEO CbCr ADDR_CFG - VFE31 format:
				 * Upper 16 bits: cbcr_height - 24 (line count)
				 * Lower 16 bits: burst = wpl - 17
				 * Use bytesperline to match buffer allocation.
				 */
				{
					u32 buf_wpl = bytesperline / 4;  /* words per line from buffer stride */
					int lines_val, burst_val;

					if (vfe31_video_cbcr_lines < 0)
						lines_val = cbcr_height - 24;  /* auto */
					else if (vfe31_video_cbcr_lines == 0)
						lines_val = cbcr_height - 1;  /* full CbCr height */
					else
						lines_val = vfe31_video_cbcr_lines;  /* explicit */

					if (vfe31_video_cbcr_burst < 0)
						burst_val = (buf_wpl - 17) & 0xFFFF;  /* auto */
					else
						burst_val = vfe31_video_cbcr_burst & 0xFFFF;  /* explicit */

					reg = (lines_val << 16) | burst_val;
					dev_info(vfe->camss->dev,
						 "VFE31: VIDEO WM5 ADDR_CFG=0x%08x (lines=%d, burst=%d, l_param=%d, b_param=%d)\n",
						 reg, lines_val, burst_val, vfe31_video_cbcr_lines, vfe31_video_cbcr_burst);
				}
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(VFE31_VIDEO_WM_CBCR));

				/*
				 * VIDEO CbCr UB_CFG - use bytesperline
				 *
				 * FIX: Use cbcr_height for CbCr WM UB_CFG, not Y height!
				 * For NV12 4:2:0, cbcr_height = height/2 = 240 for 480p.
				 * Using Y height (480) caused only 120/240 UV lines to be written.
				 */
				{
					u32 buf_wpl = bytesperline / 4;  /* words per line from buffer stride */
					int ub_height_val;

					if (vfe31_video_cbcr_ub_height < 0)
						ub_height_val = cbcr_height - 1;  /* auto: use CbCr height */
					else
						ub_height_val = vfe31_video_cbcr_ub_height;  /* explicit */

					reg = ((buf_wpl / 8 - 1) & 0xFFFF) << 16;
					reg |= ub_height_val & 0xFFFF;
					dev_info(vfe->camss->dev,
						 "VFE31: VIDEO WM5 UB_CFG=0x%08x (ub_depth=%d, ub_height=%d, cbcr_h=%d, param=%d)\n",
						 reg, (buf_wpl / 8 - 1), ub_height_val, cbcr_height, vfe31_video_cbcr_ub_height);
				}
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(VFE31_VIDEO_WM_CBCR));

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
				 "VFE31: VIDEO line WM%d(Y)+WM%d(CbCr): %ux%u input_stride=%u\n",
				 VFE31_VIDEO_WM_Y, VFE31_VIDEO_WM_CBCR,
				 pix->width, height, width * 2);
		} else {
			/*
			 * PIX line only (not VIDEO line).
			 * Disable VIDEO Y WM (WM4) to ensure clean state.
			 * PIX uses WM0 (Y) + WM1 (CbCr).
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
	 * Save the address for the PRIMARY Y WM of the pending line:
	 *   - PIX mode (VFE_LINE_PIX): WM0
	 *   - VIDEO mode (VFE_LINE_VIDEO): WM4
	 *
	 * For semi-planar formats (NV12/NV16), WM1's address (CbCr) is computed
	 * as Y_addr + cbcr_offset. During streaming, gen1 passes addr=0 for
	 * CbCr WM because buf->addr[1] is not set for single-plane formats.
	 */
	if (vfe->camif_pending) {
		bool is_primary_wm;

		/*
		 * Check if this is the primary Y WM for the pending line.
		 * PIX uses WM0, VIDEO uses WM4.
		 */
		if (vfe->camif_pending_line_id == VFE_LINE_VIDEO)
			is_primary_wm = (wm == VFE31_VIDEO_WM_Y);
		else
			is_primary_wm = (wm == VFE31_PREVIEW_WM_Y);

		dev_info(vfe->camss->dev,
			"VFE31: WM%d ping_addr=0x%08x (DEFERRED - camif_pending=true)\n",
			wm, addr);

		if (is_primary_wm)
			vfe->pending_ping_addr = addr;
	} else {
		/*
		 * During streaming: track Y addresses and calculate CbCr.
		 *
		 * Gen1 processes WMs in order: wm_idx[0] (Y), then wm_idx[1] (CbCr).
		 * For CbCr WM, gen1 passes buf->addr[1] which is garbage for
		 * single-plane NV12. We calculate CbCr from Y + offset.
		 *
		 * Use module params for Y WM detection (allows runtime testing).
		 */
		bool is_y_wm = (wm == (u8)vfe31_pix_y_wm ||
				wm == (u8)vfe31_video_y_wm);

		if (is_y_wm) {
			vfe->last_y_ping_addr = addr;
			dev_info(vfe->camss->dev,
				"VFE31: WM%d PING write 0x%08x (Y)\n", wm, addr);
		} else if (vfe->last_y_ping_addr && vfe->active_cbcr_offset) {
			/*
			 * Non-Y WM with offset set = CbCr. Calculate address.
			 * Works regardless of which WM is used for CbCr.
			 */
			addr = vfe->last_y_ping_addr + vfe->active_cbcr_offset;
			dev_info(vfe->camss->dev,
				"VFE31: WM%d PING write 0x%08x (CbCr from Y=0x%08x)\n",
				wm, addr, vfe->last_y_ping_addr);
		} else {
			dev_info(vfe->camss->dev,
				"VFE31: WM%d PING write 0x%08x (direct)\n", wm, addr);
		}

		writel_relaxed(addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));

		/* Readback verification */
		{
			u32 readback = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
			if (readback != addr)
				dev_err(vfe->camss->dev,
					"VFE31: WM%d PING readback MISMATCH! wrote=0x%08x read=0x%08x\n",
					wm, addr, readback);
		}
	}
}

static void vfe31_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	/*
	 * VFE31: If CAMIF is not yet running, defer the write.
	 * Once CAMIF is running (camif_pending=false), write directly.
	 *
	 * Save the address for the PRIMARY Y WM of the pending line.
	 * See comment in vfe31_wm_set_ping_addr for explanation.
	 */
	if (vfe->camif_pending) {
		bool is_primary_wm;

		if (vfe->camif_pending_line_id == VFE_LINE_VIDEO)
			is_primary_wm = (wm == VFE31_VIDEO_WM_Y);
		else
			is_primary_wm = (wm == VFE31_PREVIEW_WM_Y);

		dev_dbg(vfe->camss->dev,
			"VFE31: WM%d pong_addr=0x%08x (deferred, primary=%d)\n",
			wm, addr, is_primary_wm);

		if (is_primary_wm)
			vfe->pending_pong_addr = addr;
	} else {
		/*
		 * During streaming: track Y addresses and calculate CbCr.
		 *
		 * Gen1 processes WMs in order: wm_idx[0] (Y), then wm_idx[1] (CbCr).
		 * For CbCr WM, gen1 passes buf->addr[1] which is garbage for
		 * single-plane NV12. We calculate CbCr from Y + offset.
		 *
		 * Use module params for Y WM detection (allows runtime testing).
		 */
		bool is_y_wm = (wm == (u8)vfe31_pix_y_wm ||
				wm == (u8)vfe31_video_y_wm);

		if (is_y_wm) {
			vfe->last_y_pong_addr = addr;
			dev_info(vfe->camss->dev,
				"VFE31: WM%d PONG write 0x%08x (Y)\n", wm, addr);
		} else if (vfe->last_y_pong_addr && vfe->active_cbcr_offset) {
			addr = vfe->last_y_pong_addr + vfe->active_cbcr_offset;
			dev_info(vfe->camss->dev,
				"VFE31: WM%d PONG write 0x%08x (CbCr from Y=0x%08x)\n",
				wm, addr, vfe->last_y_pong_addr);
		} else {
			dev_info(vfe->camss->dev,
				"VFE31: WM%d PONG write 0x%08x (direct)\n", wm, addr);
		}

		writel_relaxed(addr,
			       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));

		/* Readback verification */
		{
			u32 readback = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
			if (readback != addr)
				dev_err(vfe->camss->dev,
					"VFE31: WM%d PONG readback MISMATCH! wrote=0x%08x read=0x%08x\n",
					wm, addr, readback);
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
		/* Consider the line being enabled as active */
		bool starting_pix = (line_id == VFE_LINE_PIX);
		bool starting_video = (line_id == VFE_LINE_VIDEO);
		bool video_state_active = (video_out->state == VFE_OUTPUT_ON ||
				     video_out->state == VFE_OUTPUT_RESERVED ||
				     video_out->state == VFE_OUTPUT_CONTINUOUS);
		bool pix_state_active = (pix_out->state == VFE_OUTPUT_ON ||
				   pix_out->state == VFE_OUTPUT_RESERVED ||
				   pix_out->state == VFE_OUTPUT_CONTINUOUS);
		bool video_active = starting_video || video_state_active;
		bool pix_active = starting_pix || pix_state_active;
		const char *mode_str;

		/* Module param override takes priority */
		if (vfe31_irq_comp_mask != 0) {
			vfe->irq_comp_mask_shadow = vfe31_irq_comp_mask;
			mode_str = "module param";
		} else if (video_active && !pix_active) {
			/* VIDEO-only: WM1 (Y) + WM5 (CbCr) */
			vfe->irq_comp_mask_shadow = VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
			mode_str = "VIDEO only";
		} else if (video_active && pix_active) {
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
		/* UYVY demux pattern: combined 16-bit format (0xC9CA) to both registers */
		writel_relaxed(0xc9ca, vfe->base + VFE_0_DEMUX_EVEN_CFG);
		writel_relaxed(0xc9ca, vfe->base + VFE_0_DEMUX_ODD_CFG);
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
		dev_info(vfe->camss->dev, "VFE TESTGEN: Configuring BUS_CFG and reloading WMs with pingpong (0x%04x)\n",
			 vfe31_get_bus_cmd_reload());
		writel_relaxed(vfe31_get_bus_cfg(), vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(vfe31_get_bus_cmd_reload(), vfe->base + VFE_0_BUS_CMD);
		wmb();

		/*
		 * Step 4c: Explicitly enable WMs (WM0 for Y, WM1 for CbCr)
		 * For PIX mode with DEMUX, Y goes to WM0 and CbCr goes to WM1.
		 * WM4 is only used for VIDEO line (duplicate Y), not for CbCr.
		 */
		writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(0));
		writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(1));
		dev_info(vfe->camss->dev, "VFE TESTGEN: Enabled WM0 and WM1\n");
		wmb();

		/*
		 * Step 5: Configure IRQ masks
		 * Use same masks as PIX mode: SOF, REG_UPDATE, PING_PONG, COMPOSITE_DONE
		 */
		/* VFE31 uses COMP_DONE only - no per-WM PING_PONG IRQs */
		vfe->irq_mask0_shadow = VFE_0_IRQ_MASK_0_CAMIF_SOF |
				       VFE_0_IRQ_MASK_0_CAMIF_EOF |
				       VFE_0_IRQ_MASK_0_REG_UPDATE |
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
	u8 bpp;

	if (!vfe->camif_pending) {
		dev_dbg(vfe->camss->dev, "VFE31: no pending CAMIF config\n");
		return;
	}

	line = &vfe->line[vfe->camif_pending_line_id];

	/*
	 * Calculate width_bytes based on INPUT format bits-per-pixel.
	 *
	 * CRITICAL: For PIX/VIDEO mode, the CAMIF sees the full sensor input
	 * (UYVY = 16 bpp) regardless of output format. The format table's
	 * mbus_bpp reflects OUTPUT format (8 bpp per NV16 plane), not INPUT.
	 *
	 * YUV422 input: 16 bpp -> width * 2 bytes (UYVY, VYUY, YUYV, YVYU)
	 * RAW8 input:   8 bpp  -> width * 1 byte
	 * RAW10 input:  10 bpp -> width * 10/8 bytes (packed)
	 */
	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		/* YUV422 formats: 16 bits (2 bytes) per pixel */
		bpp = 16;
		break;
	default:
		/* Other formats: use format table bpp */
		bpp = camss_format_get_bpp(line->formats, line->nformats,
					   line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	}
	width_bytes = line->fmt[MSM_VFE_PAD_SINK].width * bpp / 8;
	height = line->fmt[MSM_VFE_PAD_SINK].height;

	dev_info(vfe->camss->dev,
		 "VFE31 enable_pending_camif: line=%d %ux%u stride=%u (bpp=%u code=0x%04x)\n",
		 vfe->camif_pending_line_id,
		 line->fmt[MSM_VFE_PAD_SINK].width, height, width_bytes,
		 bpp, line->fmt[MSM_VFE_PAD_SINK].code);

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
	writel_relaxed(vfe31_get_bus_cfg(), vfe->base + VFE_0_BUS_CFG);

	/*
	 * Step 9: Reload all write masters with pingpong via BUS_CMD
	 * Use 0x7FFF (not 0x3FFF) to include pingpong reload bit 14
	 */
	writel_relaxed(vfe31_get_bus_cmd_reload(), vfe->base + VFE_0_BUS_CMD);
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
			 * RDI mode: Use SAME IRQ_MASK_0 as PIX mode (0x00EFE021).
			 * webOS uses identical IRQ_MASK_0 for ALL modes including
			 * raw snapshot. The composite group 1 mapping is done via
			 * IRQ_COMPOSITE_MASK, not IRQ_MASK_0.
			 *
			 * Note: COMPOSITE_DONE_1 (bit 22) is already in 0x00EFE021.
			 */
			vfe->irq_mask0_shadow = 0x00EFE021;
			dev_info(vfe->camss->dev,
				 "VFE31: RDI IRQ_MASK_0=0x%08x (same as PIX)\n",
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
			/*
			 * CRITICAL: Consider the line we're currently enabling as active!
			 * The output state may not be updated yet when enable_camif runs.
			 */
			bool starting_pix = (line->id == VFE_LINE_PIX);
			bool starting_video = (line->id == VFE_LINE_VIDEO);
			bool video_state_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool pix_state_active = (pix_out->state == VFE_OUTPUT_ON ||
					   pix_out->state == VFE_OUTPUT_RESERVED ||
					   pix_out->state == VFE_OUTPUT_CONTINUOUS);
			bool video_active = starting_video || video_state_active;
			bool pix_active = starting_pix || pix_state_active;
			u32 comp_mask;
			const char *mode_str;

			dev_info(vfe->camss->dev,
				 "VFE31: comp_mask select: line=%d starting_pix=%d starting_video=%d pix_state=%d video_state=%d\n",
				 line->id, starting_pix, starting_video,
				 pix_out->state, video_out->state);

			/* Module param override takes priority */
			if (vfe31_irq_comp_mask != 0) {
				comp_mask = vfe31_irq_comp_mask;
				mode_str = "module param";
			} else if (video_active && !pix_active) {
				/* VIDEO-only: WM1 (Y) + WM5 (CbCr) */
				comp_mask = VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
				mode_str = "VIDEO only";
			} else if (video_active && pix_active) {
				/* PIX+VIDEO: both lines active */
				comp_mask = VFE31_IRQ_COMP_MASK_PIX_VIDEO;
				mode_str = "PIX+VIDEO";
			} else {
				/* PIX only (or no VIDEO) */
				comp_mask = VFE31_IRQ_COMP_MASK_PIX_ONLY;
				mode_str = "PIX only";
			}

			vfe->irq_comp_mask_shadow = comp_mask;
			dev_info(vfe->camss->dev,
				 "VFE31 enable_camif: IRQ_COMPOSITE_MASK=0x%08x (%s)\n",
				 comp_mask, mode_str);
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

	/*
	 * Step 14: Enable BUS power management (from webOS vfe31_capture)
	 * webOS writes these after CAMIF start with comment "for debug".
	 * Without these writes, COMPOSITE_DONE interrupts never fire.
	 */
	writel_relaxed(1, vfe->base + VFE_0_BUS_PM_CFG);
	writel_relaxed(1, vfe->base + VFE_0_BUS_PM_CMD);
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
