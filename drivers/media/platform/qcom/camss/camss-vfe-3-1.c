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
#include <linux/math64.h>
#include <linux/module.h>

#include "camss.h"

/* Forward declarations for VFE31-specific functions */
static void vfe31_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr);
static void vfe31_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr);
static void vfe31_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable);
static void vfe31_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable);
static void vfe31_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
					     enum vfe_line_id id);
static void vfe31_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
				     enum vfe_line_id line_id, u8 enable);
static void vfe31_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable);
static void vfe31_set_module_cfg(struct vfe_device *vfe, u8 enable);

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
 * ============================================================================
 * VFE31 XBAR REGISTERS (0x040-0x044) - DEMUX OUTPUT ROUTING
 * ============================================================================
 *
 * The XBAR consists of TWO registers (V31_XBAR_CFG_LEN = 8 bytes):
 *   0x040 = XBAR_CFG0 (AXI_OUT_MODE) - macro routing mode
 *   0x044 = XBAR_CFG1 - per-output data routing
 *
 * Samsung kernel supports runtime XBAR updates via V31_XBAR_CFG ioctl
 * with xbar_update_pending flag (applied at REG_UPDATE IRQ).
 *
 * XBAR_CFG0 (0x040) - AXI output mode:
 *   0x01  = OUTPUT_1_AND_3 (PIX + VIDEO, routes through DEMUX + XBAR_CFG1)
 *   0x60  = CAMIF_TO_AXI (RAW bypass, ignores XBAR_CFG1)
 *   0x200 = OUTPUT_2 (preview-only, ignores XBAR_CFG1)
 *
 * XBAR_CFG1 (0x044) - per-output routing (24-bit, 3 routing bytes):
 *
 *   Bits [7:0]   = output0 routing (preview/PIX path)
 *   Bits [15:8]  = output1 routing (video/snapshot path)
 *   Bits [23:16] = output2 routing (ZSL/third output path)
 *
 *   Each routing byte controls Y and CbCr destinations:
 *     bits[3:0] = Y routing nibble
 *     bits[7:4] = CbCr routing nibble
 *
 *   Known routing nibble values:
 *     0x0 = disabled
 *     0x1 = route to ch0 only
 *     0x3 = route to ch0 + ch1
 *     0x9 = route to ch0 + ch1 (with additional output)
 *     0xA = route to ch1 only
 *     0xB = route to ch0 + ch1 (full)
 *
 *   OUTPUT_1_AND_3 logical-to-physical WM mapping:
 *     output0: ch0 = WM0, ch1 = WM4
 *     output1: ch0 = WM1, ch1 = WM5
 *     output2: ch0 = WM2, ch1 = WM6
 *
 * Cross-vendor verified values (Samsung kernel + SII/Quincy HALs + webOS):
 *
 *   0x001B = PIX only:     out0 Y+CbCr (WM0+WM4)
 *   0x1A1B = PIX+VIDEO:    out0 Y+CbCr (WM0+WM4), out1 Y+CbCr (WM1+WM5)
 *   0x1A1B1B = PIX+VID+ZSL: + out2 Y+CbCr (WM2+WM6)
 *   0x0000 = disabled (OUTPUT_2/RAW modes bypass XBAR)
 *
 * Samsung kernel (msm_vfe31.h):
 *   V31_XBAR_CFG_OFF = 0x040, V31_XBAR_CFG_LEN = 8
 */

/* Per-output routing byte values */
#define VFE31_XBAR_ROUTE_NONE		0x00	/* Output disabled */
#define VFE31_XBAR_ROUTE_Y_CBCR	0x1B	/* Y+CbCr to ch0+ch1 */
#define VFE31_XBAR_ROUTE_Y_ONLY	0x03	/* Y to ch0+ch1, no CbCr */
#define VFE31_XBAR_ROUTE_Y_CH1_CBCR	0x1A	/* Y to ch1, CbCr to ch0 */

/* Pre-built XBAR_CFG1 values for common modes */
#define VFE31_XBAR_PIX_ONLY	(VFE31_XBAR_ROUTE_Y_CBCR)			/* 0x001B */
#define VFE31_XBAR_PIX_VIDEO	(VFE31_XBAR_ROUTE_Y_CH1_CBCR << 8 | \
				 VFE31_XBAR_ROUTE_Y_CBCR)			/* 0x1A1B */
#define VFE31_XBAR_PIX_VID_ZSL	(VFE31_XBAR_ROUTE_Y_CH1_CBCR << 16 | \
				 VFE31_XBAR_ROUTE_Y_CBCR << 8 | \
				 VFE31_XBAR_ROUTE_Y_CBCR)			/* 0x1A1B1B */

/* Default: 0x1A1B (webOS Topaz register dump, PIX+VIDEO mode) */
#define VFE31_XBAR_CFG1		VFE31_XBAR_PIX_VIDEO

/* Module param for manual override/testing */
int vfe31_xbar_cfg1 = 0;  /* 0 = auto-select based on mode */
module_param(vfe31_xbar_cfg1, int, 0644);
MODULE_PARM_DESC(vfe31_xbar_cfg1,
		 "VFE31 XBAR_CFG1 override (0=auto, 0x1a03=PIX, 0x1a1b=PIX+VIDEO)");

/*
 * ============================================================================
 * RAW-through-PIX mode (Y-Plane Hack)
 * ============================================================================
 *
 * When enabled, uses PIX mode path (AXI=0x01) but disables DEMUX processing
 * (MODULE_CFG=0). This allows capturing RAW data through the working PIX path
 * instead of the broken RDI/CAMIF_TO_AXI path.
 *
 * How it works:
 * 1. Sensor sends RAW8 pixels tagged with YUV MIPI data type (fake_yuv=1)
 * 2. CAMIF accepts the data (it filters on MIPI data type)
 * 3. MODULE_CFG=0 disables DEMUX so pixels pass through as-is
 * 4. WM0 captures the raw bytes (only Y plane used, CbCr disabled)
 *
 * Width adjustment: Since RAW8 is 8 bits/pixel (not 16 like YUV422),
 * the CAMIF width must be halved to get correct byte count:
 *   1280 input pixels at 8bpp = 1280 bytes
 *   640 "fake" pixels at 16bpp = 1280 bytes (same data)
 *
 * Values:
 *   0 = Normal PIX mode with DEMUX (default)
 *   1 = RAW-through-PIX mode (MODULE_CFG=0, WM0 only)
 */
static int vfe31_raw_pix_mode = 0;
module_param(vfe31_raw_pix_mode, int, 0644);
MODULE_PARM_DESC(vfe31_raw_pix_mode,
		 "VFE31 RAW-through-PIX mode (0=normal PIX+DEMUX, 1=bypass DEMUX for RAW)");

/*
 * VFE31 Write Master assignments (cross-vendor verified):
 *
 * webOS OUTPUT_1_AND_3 mode (all vendors agree):
 *   PIX (output0):   WM0 (Y) + WM4 (CbCr)
 *   VIDEO (output2): WM1 (Y) + WM5 (CbCr)
 *
 * Current driver: VIDEO reuses PIX WMs (WM0+WM4) since only one
 * line runs at a time. XBAR 0x1A1B routes Y to WM0+WM1 and CbCr
 * to WM4. WM1/WM5 are configured by XBAR but not enabled.
 */

/*
 * Debug: dump WM registers during first IRQs of each streaming session.
 * Set to 1 to enable verbose register dumps in dmesg.
 */
static int vfe31_dump_wm_regs = 0;
module_param(vfe31_dump_wm_regs, int, 0644);
MODULE_PARM_DESC(vfe31_dump_wm_regs,
		 "VFE31 dump WM registers (0=off, 1=on)");

/*
 * VFE31 recording state machine (from Samsung msm_vfe31.c).
 * Controls VIDEO WM enable/disable at frame boundaries via REG_UPDATE IRQ.
 */
enum vfe31_rec_state {
	VFE31_REC_IDLE = 0,
	VFE31_REC_START_REQUESTED,
	VFE31_REC_STARTED,
	VFE31_REC_STOP_REQUESTED,
	VFE31_REC_STOPPED,
};

static enum vfe31_rec_state vfe31_recording_state = VFE31_REC_IDLE;
static enum vfe31_rec_state vfe31_zsl_state = VFE31_REC_IDLE;

/*
 * Deferred PIX WM enable flag. Set after CAMIF start, cleared when
 * REG_UPDATE ISR enables WMs at the first frame boundary. This prevents
 * starting DMA mid-frame which causes progressive frame wrap at 640x480.
 */
static bool vfe31_pix_wm_pending;

/* Forward declaration - used in vfe31_wm_done, defined later */
static void vfe31_bus_reload_wm(struct vfe_device *vfe, u8 wm);

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
 */
int vfe31_force_422 = 0;
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
/*
 * IRQ_COMPOSITE_MASK values (register 0x034).
 *
 * Groups WMs for COMPOSITE_DONE interrupts:
 *   Group 0 (bits 0-7):   triggers COMPOSITE_DONE_0
 *   Group 1 (bits 8-15):  triggers COMPOSITE_DONE_1
 *   Group 2 (bits 16-23): triggers COMPOSITE_DONE_2
 *
 * webOS uses 0x00220011 for OUTPUT_1_AND_3 mode:
 *   Group 0: 0x11 = WM0 + WM4 (PIX Y + CbCr)
 *   Group 2: 0x22 = WM1 + WM5 (VIDEO Y + CbCr)
 *
 * Currently only PIX group is active (VIDEO reuses PIX WMs).
 */
#define VFE31_IRQ_COMP_MASK_PIX_ONLY	0x00000011  /* Group 0: WM0+WM4 */
#define VFE31_IRQ_COMP_MASK_PIX_VIDEO	0x00220011  /* Group 0: WM0+WM4, Group 2: WM1+WM5 */
#define VFE31_IRQ_COMP_MASK_VIDEO_ONLY	0x00220000  /* Group 2: WM1+WM5 */
#define VFE31_IRQ_COMP_MASK_ZSL_ONLY	0x00004400  /* Group 1: WM2+WM6 */
#define VFE31_IRQ_COMP_MASK_PIX_ZSL	0x00004411  /* Group 0: WM0+WM4, Group 1: WM2+WM6 */
#define VFE31_IRQ_COMP_MASK_PIX_VID_ZSL	0x00224411  /* Group 0: WM0+WM4, Group 1: WM2+WM6, Group 2: WM1+WM5 */

/* Module param for manual override/testing */
static int vfe31_irq_comp_mask = 0;  /* 0 = auto-select based on active lines */
module_param(vfe31_irq_comp_mask, int, 0644);
MODULE_PARM_DESC(vfe31_irq_comp_mask,
		 "VFE31 IRQ composite mask (0=auto, 0x11=pix, 0x13=pix+video, 0x02=video)");

/*
 * RDI/raw mode EFS_CFG override.
 * -1 = use default (0x40, same as PIX mode)
 *  0 = APS mode (EFS codes ignored, CAMIF counts lines internally)
 * >0 = use this value directly
 *
 * If RDI mode doesn't count lines properly, try setting this to 0.
 * EFS_CFG 0x40 (bit 6) enables some timing/sync feature from webOS.
 * For raw capture without MIPI embedded sync, 0 (APS mode) might work better.
 */
static int vfe31_rdi_efs_cfg = -1;
module_param(vfe31_rdi_efs_cfg, int, 0644);
MODULE_PARM_DESC(vfe31_rdi_efs_cfg,
		 "VFE31 RDI EFS_CFG: -1=default (0x40), 0=APS mode, >0=use value");

/*
 * RDI mode force 16bpp input.
 *
 * Some sensors (like MT9M113 with IFP) always output 2 bytes per pixel
 * even when configured for "Processed Bayer" mode. The MIPI data type
 * might be RAW8 (0x2A) but the actual data width is still 16 bits.
 *
 * When enabled, the CAMIF is configured to expect 16 bpp input for RDI
 * regardless of the mbus format. This allows capture of 1280 bytes for
 * 640 pixels instead of the expected 640 bytes.
 *
 * 0 = use format's actual bpp (8 for RAW8, 10 for RAW10, etc.)
 * 1 = force 16 bpp for all RDI formats
 */
static int vfe31_rdi_force_16bpp = 0;
module_param(vfe31_rdi_force_16bpp, int, 0644);
MODULE_PARM_DESC(vfe31_rdi_force_16bpp,
		 "VFE31 RDI force 16bpp: 0=use format bpp, 1=force 16bpp (for MT9M113 IFP)");

/*
 * BUS_CFG default value per webOS register dumps.
 */
static inline u32 vfe31_get_bus_cfg(void)
{
	return 0x02AAA771;
}

/*
 * BUS_CMD reload default value per webOS code (includes pingpong reload bit 14).
 */
static inline u32 vfe31_get_bus_cmd_reload(void)
{
	return 0x7FFF;
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
	bool is_native_420 = (pixelformat == V4L2_PIX_FMT_NV12 ||
			      pixelformat == V4L2_PIX_FMT_NV21);

	if (!is_semiplanar)
		return false;  /* Not semi-planar, doesn't apply */

	/*
	 * Check format override - but NEVER allow force_422=2 (4:2:2) on
	 * a 4:2:0 format (NV12/NV21) because the buffer is sized for half-height
	 * CbCr and writing full-height would overflow. This prevents crashes
	 * when switching formats at runtime.
	 *
	 * force_422=1 (force 4:2:0) is always safe - it reduces CbCr height.
	 * force_422=2 (force 4:2:2) is only safe on NV16/NV61 buffers.
	 */
	if (vfe31_force_422 == 1)
		return true;   /* Force 4:2:0 - always safe */
	if (vfe31_force_422 == 2 && !is_native_420)
		return false;  /* Force 4:2:2 - only safe on NV16/NV61 */

	/* Auto or force_422=2 on NV12: based on actual format */
	return is_native_420;
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

/*
 * ============================================================================
 * VFE31 CENTRALIZED CONFIGURATION STRUCTURES
 * ============================================================================
 *
 * These structures provide deterministic, pre-calculated configuration for
 * VFE31 Write Masters. This replaces the scattered inline calculations and
 * numerous module parameters with a single source of truth.
 *
 * Key insights from reference implementations (webOS, Mako, LG G2):
 *
 * 1. UB_CFG and IMAGE_SIZE stride sit BEFORE DEMUX → use INPUT stride (width×2)
 * 2. ADDR_CFG burst sits AFTER DEMUX → use OUTPUT stride for Y, (width/4) for CbCr
 * 3. CbCr offset uses OUTPUT stride (bytesperline × height), not input stride
 * 4. For 4:2:0, CbCr WM lines = cbcr_height + 64 (pipeline headroom)
 * 5. For 4:2:2, CbCr WM lines = cbcr_height (no headroom needed)
 */

/**
 * struct vfe31_wm_config - Pre-calculated Write Master configuration
 * @ub_depth:       Unified Buffer depth: (input_stride/32) - 1 (pre-DEMUX)
 * @ub_height:      UB height value: output_height - 1
 * @image_stride:   IMAGE_SIZE stride: input_stride / 16 (pre-DEMUX, 128-bit units)
 * @image_height:   IMAGE_SIZE height: output_height - 1
 * @burst_words:    ADDR_CFG burst: (output_stride/4) - 17 for Y, (width/4) - 9 for CbCr
 * @burst_lines:    ADDR_CFG lines: 0 for Y, cbcr_height(+64) for CbCr
 * @ping_addr:      PING buffer address (set at runtime)
 * @pong_addr:      PONG buffer address (set at runtime)
 */
struct vfe31_wm_config {
	u16 ub_depth;
	u16 ub_height;
	u16 image_stride;
	u16 image_height;
	u16 burst_words;
	u16 burst_lines;
	u32 ping_addr;
	u32 pong_addr;
};

/**
 * struct vfe31_line_config - Complete line configuration
 * @path:           VFE_LINE_RDI, VFE_LINE_PIX, or VFE_LINE_VIDEO
 * @pixelformat:    V4L2 pixel format (NV12, NV16, etc.)
 * @width:          Frame width in pixels
 * @height:         Frame height in lines
 * @input_stride:   Bytes per line at VFE input (width×2 for UYVY)
 * @output_stride:  Bytes per line at DMA output (bytesperline)
 * @cbcr_height:    CbCr plane height (height/2 for NV12, height for NV16)
 * @y_plane_size:   Y plane size in bytes (output_stride × height)
 * @cbcr_offset:    CbCr plane offset from buffer start (= y_plane_size)
 * @axi_mode:       AXI output mode (0x01, 0x60, 0x200)
 * @xbar_value:     XBAR_CFG1 routing value
 * @chroma_v_out:   CHROMA_V_IMAGE output height
 * @chroma_subs_cfg: CHROMA_SUBS_CFG value (0x10 or 0x30)
 * @y_wm:           Y Write Master config
 * @cbcr_wm:        CbCr Write Master config (if semi-planar)
 * @has_cbcr:       True if format uses separate CbCr WM
 */
struct vfe31_line_config {
	enum vfe_line_id path;
	u32 pixelformat;
	u16 width;
	u16 height;
	u16 input_stride;
	u16 output_stride;
	u16 cbcr_height;
	u32 y_plane_size;
	u32 cbcr_offset;
	u32 axi_mode;
	u16 xbar_value;
	u16 chroma_v_out;
	u8 chroma_subs_cfg;
	struct vfe31_wm_config y_wm;
	struct vfe31_wm_config cbcr_wm;
	bool has_cbcr;
};

/**
 * vfe31_calc_rdi_config - Calculate config for RDI/RAW bypass mode
 * @cfg: Configuration structure to fill
 * @width: Frame width in pixels
 * @height: Frame height in lines
 * @bpp: Bits per pixel (8, 10, or 12)
 *
 * RDI mode bypasses DEMUX and writes raw sensor data directly.
 * Only the Y WM is used, and stride is simply width * (bpp/8).
 */
static void vfe31_calc_rdi_config(struct vfe31_line_config *cfg,
				  u16 width, u16 height, u8 bpp)
{
	u16 stride = width * bpp / 8;

	cfg->input_stride = stride;
	cfg->output_stride = stride;
	cfg->cbcr_height = 0;
	cfg->y_plane_size = stride * height;
	cfg->cbcr_offset = 0;
	cfg->has_cbcr = false;
	cfg->axi_mode = 0x60;  /* CAMIF_TO_AXI raw bypass */
	cfg->xbar_value = 0;   /* XBAR not used in RDI mode */
	cfg->chroma_v_out = 0;
	cfg->chroma_subs_cfg = 0;

	/*
	 * Y WM config for RDI/RAW.
	 *
	 * UB_CFG: same stride-based depth as PIX.
	 * IMAGE_SIZE: stride in 16-byte units, same formula as PIX.
	 * ADDR_CFG (UB allocation): single WM gets full UB budget (911).
	 *   Samsung raw snapshot uses 0x397 (919), Opal uses 0x38F (911).
	 *   We use 911 matching Opal (same APQ8060).
	 */
	cfg->y_wm.ub_depth = (stride / 32) - 1;
	cfg->y_wm.ub_height = height - 1;
	cfg->y_wm.image_stride = stride / 16;
	cfg->y_wm.image_height = height - 1;
	cfg->y_wm.burst_words = 0x38F;  /* Full UB budget for single WM */
	cfg->y_wm.burst_lines = 0;      /* UB start = 0 */
}

/**
 * vfe31_calc_pix_config - Calculate config for PIX/VIDEO mode (UYVY → NV12/NV16)
 * @cfg: Configuration structure to fill
 * @width: Frame width in pixels
 * @height: Frame height in lines
 * @bytesperline: Output bytes per line from V4L2 format
 * @pixelformat: V4L2 pixel format (NV12, NV16, etc.)
 *
 * PIX mode uses DEMUX to separate UYVY input into Y and CbCr planes.
 * UB_CFG and IMAGE_SIZE use INPUT stride (width*2 for VFE timing).
 * ADDR_CFG burst uses OUTPUT stride (actual DMA writes).
 *
 * From Gemini validation:
 * - CHROMA_V_IMAGE does vertical scaling (480→240 for NV12)
 * - CHROMA_SUBS_CFG tells CbCr DMA how many lines to expect
 * - Both must be configured consistently for the hardware to work
 */
static void vfe31_calc_pix_config(struct vfe31_line_config *cfg,
				  u16 width, u16 height, u16 bytesperline,
				  u32 pixelformat)
{
	u16 input_stride = width * 2;           /* UYVY input (VFE pipeline timing) */
	u16 output_stride = bytesperline;       /* DMA output stride */
	u16 cbcr_height;
	bool is_420;

	cfg->pixelformat = pixelformat;
	cfg->width = width;
	cfg->height = height;
	cfg->input_stride = input_stride;
	cfg->output_stride = output_stride;
	cfg->has_cbcr = true;
	cfg->axi_mode = 0x01;  /* OUTPUT_1_AND_3 mode */

	/* Determine if 4:2:0 or 4:2:2 format */
	is_420 = vfe31_is_420_format(pixelformat);
	cbcr_height = is_420 ? height / 2 : height;
	cfg->cbcr_height = cbcr_height;

	/*
	 * Y plane size for CbCr offset calculation.
	 *
	 * IMPORTANT: VFE31 Y WM writes at COMPACT stride (width bytes per line),
	 * NOT input stride (width*2). This was verified by analyzing raw capture
	 * data (2026-04-18):
	 *   - Data at offset 1023*1280 = 1,309,440: present (full Y plane)
	 *   - Data at offset 512*2560 = 1,310,720: zeros (not at 2x stride)
	 *   - CbCr at offset 0x280000 (input_stride*h): all zeros (wrong offset)
	 *
	 * The ADDR_CFG burst uses OUTPUT stride (width), not input stride.
	 * UB_CFG and IMAGE_SIZE use INPUT stride for pipeline timing, but
	 * the DMA writes compactly at output stride.
	 *
	 * For 1280x1024:
	 *   Y plane = width * height = 1280 * 1024 = 1,310,720 bytes
	 *   CbCr starts at offset 1,310,720 (0x140000)
	 */
	cfg->y_plane_size = width * height;  /* Y at compact output stride */
	cfg->cbcr_offset = cfg->y_plane_size;

	/*
	 * Chroma scaling configuration (from Gemini validation):
	 * - CHROMA_V_IMAGE: (output_height << 16) | input_height
	 * - CHROMA_SUBS_CFG: 0x30 for NV12 (Enable + vsubSample), 0x10 for NV16
	 */
	cfg->chroma_v_out = cbcr_height;
	cfg->chroma_subs_cfg = is_420 ? 0x30 : 0x10;

	/*
	 * WR_ADDR_CFG = (UB_start << 16) | UB_depth
	 *
	 * CORRECTED 2026-04-20: WR_ADDR_CFG is UB SRAM allocation, NOT DMA burst.
	 * Proven by Opal (APQ8060, same SoC) decompiled HAL using constant 0x390
	 * (912 = total UB entries for image WMs) which reproduces exact webOS
	 * register dump values.
	 *
	 * UB proportional allocation formula (all vendors agree):
	 *   UB_depth = (plane_pixels * 912) / total_bandwidth - 1
	 *   UB_start = previous_WM_end + 1  (sequential stacking)
	 *
	 * For single-output 640x480 NV12:
	 *   total_bw = width * height * 1.5 = 460,800
	 *   Y_depth  = (640*480*912) / 460800 - 1 = 607
	 *   Cb_depth = (320*480*912) / 460800 - 1 = 303
	 *
	 * For dual-output 640x480 NV12 (OUTPUT_1_AND_3):
	 *   total_bw = 2 * width * height * 1.5 = 921,600
	 *   Y_depth  = (640*480*912) / 921600 - 1 = 303  (matches webOS 0x012F)
	 *   Cb_depth = (320*480*912) / 921600 - 1 = 151  (matches webOS 0x0097)
	 */
	{
		/*
		 * Proportional UB SRAM allocation (cross-vendor verified).
		 *
		 * UB_depth = (plane_pixels * 912) / total_bw - 1
		 * 912 = total UB entries for image WMs (from Opal HAL, same APQ8060)
		 *
		 * Total bandwidth depends on format (HTC/Sony verified):
		 *   NV12 (4:2:0): total_bw = 2 * width * height * 1.5 = y_pixels * 3
		 *   NV16 (4:2:2): total_bw = 2 * width * height * 2.0 = y_pixels * 4
		 *
		 * CbCr UB depth depends on format (HTC/Sony verified):
		 *   NV12: CbCr depth computed separately (half-width pixels)
		 *   NV16: CbCr depth = same as Y depth (equal plane sizes)
		 */
		u32 y_pixels = width * height;
		u32 total_bw = is_420 ? (y_pixels * 3) : (y_pixels * 4);
		u32 y_depth = div_u64((u64)y_pixels * 912, total_bw);
		u32 cbcr_depth;

		if (is_420) {
			/* NV12: CbCr at half-width data rate */
			u32 cbcr_pixels = (width / 2) * height;
			cbcr_depth = div_u64((u64)cbcr_pixels * 912, total_bw);
		} else {
			/* NV16: CbCr same depth as Y (HTC/Sony verified) */
			cbcr_depth = y_depth;
		}

		if (y_depth > 0)
			y_depth--;
		if (cbcr_depth > 0)
			cbcr_depth--;
		if (y_depth < 1)
			y_depth = 1;
		if (cbcr_depth < 1)
			cbcr_depth = 1;

		cfg->y_wm.burst_words = y_depth & 0x3ff;
		cfg->y_wm.burst_lines = 0;  /* UB start = 0 (first WM) */

		cfg->cbcr_wm.burst_words = cbcr_depth & 0x3ff;
		cfg->cbcr_wm.burst_lines = (y_depth + 1) & 0x3ff;  /* UB start after Y */
	}

	/*
	 * Y WM UB_CFG and IMAGE_SIZE (cross-vendor verified):
	 *   UB_CFG:     (depth << 16) | (height - 1), depth = (input_stride/32)-1
	 *   IMAGE_SIZE: (stride << 16) | ((height-1) << 4) | 2, stride = input_stride/16
	 */
	cfg->y_wm.ub_depth = (input_stride / 32) - 1;
	cfg->y_wm.ub_height = height - 1;
	cfg->y_wm.image_stride = input_stride / 16;
	cfg->y_wm.image_height = height - 1;

	/*
	 * CbCr WM UB_CFG and IMAGE_SIZE: same depth/stride as Y, height varies.
	 */
	cfg->cbcr_wm.ub_depth = (input_stride / 32) - 1;
	cfg->cbcr_wm.ub_height = cbcr_height - 1;
	cfg->cbcr_wm.image_stride = input_stride / 16;
	cfg->cbcr_wm.image_height = cbcr_height - 1;
}

/**
 * vfe31_dump_line_config - Debug helper to log calculated configuration
 * @dev: Device for logging
 * @cfg: Configuration to dump
 */
static void vfe31_dump_line_config(struct device *dev,
				   const struct vfe31_line_config *cfg)
{
	dev_info(dev, "VFE31 config: path=%d w=%u h=%u in_stride=%u out_stride=%u\n",
		 cfg->path, cfg->width, cfg->height,
		 cfg->input_stride, cfg->output_stride);
	dev_info(dev, "  Y WM: ub_depth=%u ub_height=%u img_stride=%u img_height=%u\n",
		 cfg->y_wm.ub_depth, cfg->y_wm.ub_height,
		 cfg->y_wm.image_stride, cfg->y_wm.image_height);
	dev_info(dev, "  Y WM: burst=%u lines=%u\n",
		 cfg->y_wm.burst_words, cfg->y_wm.burst_lines);
	if (cfg->has_cbcr) {
		dev_info(dev, "  CbCr WM: ub_depth=%u ub_height=%u img_stride=%u img_height=%u\n",
			 cfg->cbcr_wm.ub_depth, cfg->cbcr_wm.ub_height,
			 cfg->cbcr_wm.image_stride, cfg->cbcr_wm.image_height);
		dev_info(dev, "  CbCr WM: burst=%u lines=%u offset=0x%x\n",
			 cfg->cbcr_wm.burst_words, cfg->cbcr_wm.burst_lines,
			 cfg->cbcr_offset);
		dev_info(dev, "  Chroma: v_out=%u subs_cfg=0x%02x\n",
			 cfg->chroma_v_out, cfg->chroma_subs_cfg);
	}
}

/* VFE31 BUS Write Master register offsets (WM0-WM6, stride 0x18) */
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(n)		(0x04C + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(n)	(0x050 + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(n)	(0x054 + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(n)		(0x058 + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(n)		(0x05C + 0x18 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(n)	(0x060 + 0x18 * (n))

/**
 * vfe31_apply_wm_config - Write WM configuration to hardware registers
 * @vfe: VFE device
 * @wm: Write Master index (0-7)
 * @cfg: Pre-calculated WM configuration
 *
 * Writes all WM registers using the pre-calculated values from cfg.
 * This replaces scattered inline calculations with a single function
 * that applies deterministic values.
 *
 * Register format (VFE31):
 * - IMAGE_SIZE: (stride/16) << 16 | ((height-1) << 4) | 2
 * - ADDR_CFG:   (lines << 16) | burst_words
 * - UB_CFG:     (depth << 16) | height
 */
static void vfe31_apply_wm_config(struct vfe_device *vfe, u8 wm,
				  const struct vfe31_wm_config *cfg)
{
	u32 reg;

	/* WR_PING_ADDR */
	writel_relaxed(cfg->ping_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));

	/* WR_PONG_ADDR */
	writel_relaxed(cfg->pong_addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));

	/* WR_IMAGE_SIZE: (stride/16) << 16 | ((height-1) << 4) | 2 */
	reg = ((cfg->image_stride) & 0xFFFF) << 16;
	reg |= ((cfg->image_height) << 4) | 2;
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));

	/* WR_ADDR_CFG: (lines << 16) | burst_words */
	reg = ((cfg->burst_lines) << 16) | (cfg->burst_words & 0xFFFF);
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	/* WR_UB_CFG: (depth << 16) | height */
	reg = ((cfg->ub_depth) << 16) | (cfg->ub_height & 0xFFFF);
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));

	/* WR_CFG: enable only (BIT(0)) - VFE31 doesn't use frame_based in bit 1 */
	writel_relaxed(BIT(0), vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm));

	wmb();
}

/**
 * vfe31_apply_line_config - Apply complete line configuration to hardware
 * @vfe: VFE device
 * @cfg: Pre-calculated line configuration
 * @y_wm: Y plane Write Master index
 * @cbcr_wm: CbCr plane Write Master index (ignored if !has_cbcr)
 *
 * Applies the calculated configuration to all relevant WMs.
 * Call vfe31_calc_pix_config() or vfe31_calc_rdi_config() first
 * to populate the cfg structure.
 */
static void vfe31_apply_line_config(struct vfe_device *vfe,
				    const struct vfe31_line_config *cfg,
				    u8 y_wm, u8 cbcr_wm)
{
	/* Apply Y WM configuration */
	vfe31_apply_wm_config(vfe, y_wm, &cfg->y_wm);

	/* Apply CbCr WM configuration if semi-planar format and valid WM */
	if (cfg->has_cbcr && cbcr_wm != 0xff)
		vfe31_apply_wm_config(vfe, cbcr_wm, &cfg->cbcr_wm);
}

/**
 * vfe31_calc_xbar - Calculate XBAR_CFG1 routing value dynamically
 * @pix_active: True if PIX line is active
 * @video_active: True if VIDEO line is active
 * @video_cbcr: True if VIDEO CbCr (WM5) is enabled
 *
 * Constructs XBAR_CFG1 from building blocks based on active WMs:
 *   PIX only:      Y→WM0,      CbCr→WM4       = 0x1A13
 *   PIX+VIDEO Y:   Y→WM0+WM1,  CbCr→WM4       = 0x1A1B (webOS default)
 *   Full dual:     Y→WM0+WM1,  CbCr→WM4+WM5   = 0x1A9B
 */
static u32 vfe31_calc_xbar(bool pix_active, bool video_active, bool zsl_active)
{
	/*
	 * Build XBAR_CFG1 from per-output routing bytes:
	 *   byte0 = output0 (PIX/preview)
	 *   byte1 = output1 (VIDEO/snapshot)
	 *   byte2 = output2 (ZSL)
	 *
	 * webOS Topaz uses 0x1A1B even for PIX-only (out1 routing
	 * is harmless when WM1/WM5 are not enabled).
	 */
	u32 xbar = 0;

	if (pix_active)
		xbar |= VFE31_XBAR_ROUTE_Y_CBCR;		/* out0: WM0+WM4 */

	if (video_active || pix_active)
		xbar |= (u32)VFE31_XBAR_ROUTE_Y_CH1_CBCR << 8;/* out1: WM1+WM5 */

	if (zsl_active)
		xbar |= (u32)VFE31_XBAR_ROUTE_Y_CH1_CBCR << 16;/* out2: WM2+WM6 */

	return xbar;
}

/* External module parameters from camss-vfe.c */
extern int software_sof_enable;
extern int software_eof_enable;

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

/*
 * ============================================================================
 * VFE31 CONFIGURATION REFERENCE - HTC CAMERA BINARY ANALYSIS
 * ============================================================================
 *
 * This section documents configuration values discovered through decompilation
 * of HTC camera binaries from MSM8660-based devices. These values complement
 * the webOS kernel sources and provide additional insight into raw capture
 * and advanced configurations that webOS never used.
 *
 * SOURCES:
 *   - HTC liboemcamera.so (MSM8660 camera HAL library, decompiled with Ghidra)
 *   - HP webOS msm_vfe31.c/msm_vfe31.h (TouchPad kernel)
 *   - Qualcomm CAF kernels (Mako, Sony, LG G2)
 *
 * Full analysis available in:
 *   - reports/htc-camera-decompiled/RAW_MODE_ANALYSIS.md
 *   - reports/vfe31-htc-vs-mainline-comparison.md
 *   - reports/vfe31-video-path-nv16-analysis.md
 *
 * ============================================================================
 * RAW CAPTURE MODE (from HTC axi_raw_snapshot_config)
 * ============================================================================
 *
 * HTC binary reveals RAW bit depth configuration at BUS_CFG bits 2-3:
 *
 *   Bit Depth   BUS_CFG Value   Bits 2-3   Burst Divisor
 *   ─────────────────────────────────────────────────────────
 *   8-bit       0x2AAA771       00         8
 *   10-bit      0x2AAA775       01         6
 *   12-bit      0x2AAA779       10         5
 *
 * The RAW pixel data size field is at bits 2-3 (shift = 2), NOT bits 8-9.
 * This was discovered from HTC axi_raw_snapshot_config decompilation.
 *
 * RAW mode also uses:
 *   - AXI_OUT_MODE = 0x60 (CAMIF_TO_AXI_VIA_OUTPUT_2)
 *   - Frame drop pattern = 0x3FFF (no frame drops)
 *   - WM0 only (single write master, raw bypass)
 *   - IRQ COMPOSITE_MASK: WM0 in group 1 (bit 8)
 *
 * ============================================================================
 * CHROMA SUBSAMPLE CONFIGURATION (from HTC vfe_chroma_subsample_config)
 * ============================================================================
 *
 * Register: CHROMA_SUBS_CFG at 0x4F8 (12 bytes total)
 *
 * Configuration byte (offset 0x4F8) bit layout:
 *
 *   Bit   Name              NV12 (4:2:0)   NV16 (4:2:2)
 *   ─────────────────────────────────────────────────────────
 *   0-1   Reserved          0              0
 *   2     Format select     varies         varies
 *   4     Enable            1              1
 *   5     vsubSampleEnable  1 (2:1 vert)   0 (no vert sub)
 *
 * For NV16 (4:2:2 full chroma):
 *   - Set bit 4 = 1 (enable)
 *   - Set bit 5 = 0 (no vertical subsampling)
 *   - Result: CHROMA_SUBS_CFG[0] = 0x10
 *
 * For NV12 (4:2:0 half chroma):
 *   - Set bit 4 = 1 (enable)
 *   - Set bit 5 = 1 (2:1 vertical subsampling)
 *   - Result: CHROMA_SUBS_CFG[0] = 0x30
 *
 * Current mainline only writes 1 byte (0x30 for NV12). Full 12-byte
 * configuration may be needed for proper NV16 support.
 *
 * ============================================================================
 * AXI OUTPUT MODE VALUES (from HTC axi_vfe_config)
 * ============================================================================
 *
 *   Format Mask   Output Mode   Format Type
 *   ─────────────────────────────────────────────
 *   0x86          0x200         YUV420 (NV12/NV21)
 *   0x41          0x1A00        YUV422 (NV16/NV61)
 *   0x20          0x204000      Other format
 *   RAW           0x60          Raw bypass
 *
 * NOTE: NV16/YUV422 may use different output mode (0x1A00) than NV12 (0x200).
 * This is NOT yet implemented in mainline - we use 0x01 for all YUV modes.
 *
 * ============================================================================
 * VIDEO PATH (WM1/WM5) CONFIGURATION
 * ============================================================================
 *
 * For simultaneous preview + video capture using WM4/WM5:
 *
 *   XBAR_CFG1 = 0x1A9B needed for CbCr to reach both WM1 and WM5:
 *     - bits [3:0] = 0xB = Y routing to WM0 + WM4
 *     - bits [7:4] = 0x9 = CbCr routing to WM1 + WM5
 *     - bits [15:8] = 0x1A = output1 routing byte
 *
 *   Current XBAR values:
 *     0x1A1B: CbCr → WM1 only (WM5 receives nothing!)
 *     0x1A9B: CbCr → WM1 + WM5 (required for VIDEO CbCr)
 *
 * WebOS observation: CbCr write masters (WM1, WM4, WM5) were DISABLED
 * during preview/video! Only Y plane was captured. CbCr was only enabled
 * during picture capture mode. This means webOS never actually tested
 * full NV16 output during streaming.
 *
 * ============================================================================
 * VFE COMMAND CODES (from HTC vfe_util_write_hw_cmd)
 * ============================================================================
 *
 *   Command   Value   Description
 *   ─────────────────────────────────────
 *   GET_HW_VERSION     0x42   Get VFE hardware version
 *   MODULE_CFG         0x71   Module configuration
 *   DEMUX              0x0B   Demux config
 *   DEMUX_UPDATE       0x21   Demux update
 *   CHROMA_SS          0x19   Chroma subsample config (12 bytes)
 *   GAMMA              0x10   Gamma config
 *   COLOR_CORRECT      0x0F   Color correction config
 *   WB                 0x0E   White balance config
 *   STATS_AEC          0x56   AEC stats config
 *   STATS_AF           0x54   AF stats config
 *
 * ============================================================================
 */

/*
 * ============================================================================
 * VFE 3.1 REGISTER REFERENCE - MSM8660/APQ8060
 * ============================================================================
 *
 * This documentation is derived from analysis of HP webOS msm_vfe31.c kernel
 * source (primary reference for HP TouchPad) and cross-verified against
 * decompiled vendor camera HAL binaries from HTC, Samsung, and Sony devices
 * using MSM8660/MSM8960 SoCs. All four vendors use identical register formulas.
 *
 * VFE31 HARDWARE PIPELINE:
 *
 *   Sensor (UYVY) → CSIPHY → CSID → CAMIF → DEMUX → Scaler → XBAR → WMs → DDR
 *                                     ↓
 *                              [Y and CbCr separation]
 *
 * WRITE MASTER ASSIGNMENT (OUTPUT_1_AND_3 mode):
 *   WM0 = PIX Y        (output0.ch0)
 *   WM1 = VIDEO Y      (output2.ch0)
 *   WM4 = PIX CbCr     (output0.ch1)
 *   WM5 = VIDEO CbCr   (output2.ch1)
 *
 * KEY REGISTER FORMULAS (cross-verified HTC/Samsung/Sony/webOS):
 *
 *   IMAGE_SIZE register:
 *     stride_field = ((width + 15) / 16) - 1
 *     height_field = height - 1
 *     value = (stride_field << 16) | (height_field << 4) | 0x2
 *
 *   UB_CFG register:
 *     ub_depth = (calculated_depth + 64) & 0x3FF   // +64 headroom required
 *     value = (ub_depth << 16) | (height - 1)
 *
 *   ADDR_CFG register:
 *     y_burst = (input_stride / 4) - 17
 *     cbcr_burst = (width / 4) - 9
 *     cbcr_lines = cbcr_height + 64   // +64 for pipeline flush
 *
 *   DEMUX for UYVY (CbYCrY):
 *     DEMUX_EVEN_CFG = DEMUX_ODD_CFG = 0xC9CA   // 16-bit value
 *     DEMUX_CFG bits[2:0] = 3 (YUV mode)
 *
 * CROSS-VENDOR VERIFICATION SUMMARY (HTC, Samsung, Sony, HP webOS):
 *   ┌─────────────────────┬─────────┬─────────┬─────────┬─────────┬──────────┐
 *   │ Register/Formula    │   HTC   │ Samsung │  Sony   │ webOS   │ Verified │
 *   ├─────────────────────┼─────────┼─────────┼─────────┼─────────┼──────────┤
 *   │ IMAGE_SIZE stride   │(w+15)/16-1│ same  │  same   │  same   │    ✓     │
 *   │ UB_CFG +64 headroom │  +0x40  │  +0x40  │  +0x40  │  +0x40  │    ✓     │
 *   │ DEMUX UYVY value    │ 0xC9CA  │ 0xC9CA  │ 0xC9CA  │ 0xC9CA  │    ✓     │
 *   │ CbCr lines +64      │   +64   │   +64   │   +64   │   +64   │    ✓     │
 *   │ XBAR_CFG1 VIDEO     │ 0x1A1B  │    -    │    -    │ 0x1A1B  │    ✓     │
 *   └─────────────────────┴─────────┴─────────┴─────────┴─────────┴──────────┘
 *
 * XBAR_CFG1 ROUTING (0x044):
 *   bits[3:0]  = Y routing:    0x3=WM0, 0xB=WM0+WM1
 *   bits[7:4]  = CbCr routing: 0x1=WM4, 0x9=WM4+WM5
 *   bits[15:8] = 0x1A (output1 routing byte)
 *   Common values: 0x1A1B (PIX+VIDEO Y, PIX CbCr only)
 *                  0x1A9B (PIX+VIDEO Y and CbCr)
 *
 * TESTGEN: NOT FUNCTIONAL on VFE31
 *   The test pattern generator hardware was removed in MSM8660-era silicon.
 *   INPUT_MUX=0x03 (TESTGEN) exists in CORE_CFG but leads to no hardware.
 *   TESTGEN_CFG (0x15C) writes don't stick. VFE8x addresses (0x364+) are
 *   repurposed for FOV/SCALER/WB modules in VFE31.
 *
 * ============================================================================
 */

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
/*
 * CORE_CFG pixel pattern field (bits 0-2):
 *   0-3 = Bayer patterns (for RAW sensor input)
 *   4-7 = YUV patterns (for YUV sensor input)
 *
 * From Samsung/HTC VFE31 binaries analysis:
 *   - Bayer patterns MUST be set correctly for RAW mode
 *   - Setting pattern=0 for all RAW formats causes CAMIF to not recognize input
 *
 * Pattern to media bus format mapping:
 *   VFE_BAYER_RGRGRG (0) → SRGGB (first pixel R, pattern starts R-Gr-R-Gr...)
 *   VFE_BAYER_GRGRGR (1) → SGRBG (first pixel Gr, pattern starts Gr-R-Gr-R...)
 *   VFE_BAYER_BGBGBG (2) → SBGGR (first pixel B, pattern starts B-Gb-B-Gb...)
 *   VFE_BAYER_GBGBGB (3) → SGBRG (first pixel Gb, pattern starts Gb-B-Gb-B...)
 */
#define VFE_0_CORE_CFG_PIXEL_PATTERN_RGRGRG	0x0	/* SRGGB */
#define VFE_0_CORE_CFG_PIXEL_PATTERN_GRGRGR	0x1	/* SGRBG */
#define VFE_0_CORE_CFG_PIXEL_PATTERN_BGBGBG	0x2	/* SBGGR */
#define VFE_0_CORE_CFG_PIXEL_PATTERN_GBGBGB	0x3	/* SGBRG */
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCBYCR	0x4
#define VFE_0_CORE_CFG_PIXEL_PATTERN_YCRYCB	0x5
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY	0x6
#define VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY	0x7
/*
 * Input mux select (bits 5:4, 2-bit field): 0=CAMIF, 1=TESTGEN, 2=unused, 3=AXI
 * Bit 6: Input mux enable (must be set for data to flow)
 *
 * Examples: CAMIF+UYVY = 0x46 (bit6=1, bits[5:4]=00, bits[2:0]=110)
 *           TESTGEN+UYVY = 0x56 (bit6=1, bits[5:4]=01, bits[2:0]=110)
 */
#define VFE_0_CORE_CFG_INPUT_MUX_TESTGEN	(1 << 4)
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

/*
 * IRQ_COMPOSITE_MASK_0 (0x034) - Maps Write Masters to Composite IRQ Groups
 *
 * This register controls which Write Masters (WM0-WM6) trigger which
 * IMAGE_COMPOSITE_DONE_n interrupt. Essential for frame completion detection.
 *
 * Bit Layout (32-bit register):
 *   Bits [7:0]   Group 0 → IMAGE_COMPOSITE_DONE_0 (IRQ_STATUS_0 bit 21)
 *   Bits [15:8]  Group 1 → IMAGE_COMPOSITE_DONE_1 (IRQ_STATUS_0 bit 22)
 *   Bits [23:16] Group 2 → IMAGE_COMPOSITE_DONE_2 (IRQ_STATUS_0 bit 23)
 *
 * Each bit within a group corresponds to a WM:
 *   Bit 0/8/16  = WM0    Bit 4/12/20 = WM4
 *   Bit 1/9/17  = WM1    Bit 5/13/21 = WM5
 *   Bit 2/10/18 = WM2    Bit 6/14/22 = WM6
 *   Bit 3/11/19 = WM3
 *
 * PIX/YUV Mode Configuration (webOS value: 0x00220011):
 *   Group 0: 0x11 = WM0 + WM4 (PIX Y + CbCr)
 *   Group 1: 0x00 = none
 *   Group 2: 0x22 = WM1 + WM5 (VIDEO Y + CbCr)
 *   Result: COMPOSITE_DONE_0 fires when both WM0 AND WM4 complete
 *
 * Raw/RDI Mode Configuration (from webOS msm_vfe31.c):
 *   Group 1: bit 8 = WM0 (raw data)
 *   Formula: irq_comp_mask |= (0x1 << (out1.ch0 + 8))
 *            = (0x1 << (0 + 8)) = 0x100
 *   Result: COMPOSITE_DONE_1 fires when WM0 completes
 *
 * IMPORTANT: Raw mode uses COMPOSITE_DONE_1 (group 1), not COMPOSITE_DONE_0!
 * This is because raw snapshot is considered a separate output path from
 * the normal PIX preview/video modes.
 */
#define VFE_0_IRQ_COMPOSITE_MASK_0	0x034

/*
 * VIOLATION_STATUS (0x048) - Reports ISP pipeline violations
 *
 * NOTE: This register may NOT exist on VFE31 (MSM8660). WebOS header
 * defines it but the hardware may not implement it. Reads often return 0
 * even when VIOLATION IRQ fires, indicating spurious interrupt or
 * different violation reporting mechanism on this SoC.
 */
#define VFE_0_VIOLATION_STATUS		0x048

#define VFE_0_BUS_CMD			0x038
#define VFE_0_BUS_CMD_Mx_RLD_CMD(x)	BIT(x)

#define VFE_0_BUS_CFG			0x03C
/*
 * BUS_CFG (0x03C) = 0x02AAA771 for all NV12/NV16 modes.
 *
 * Confirmed bits (from VFE31 vendor evidence):
 *   bits 3:2  = rawPixelDataSize (00=8bit, 01=10bit, 10=12bit)
 *   bits 11:10 = rawWritePathSelect (VFE_RAW_WR_PATH_SEL enum)
 *
 * Remaining bits are unconfirmed (VFE8x names may not apply).
 * See vfe31-register-reference.md for full bit field analysis.
 *
 * RAW mode variants: 8-bit=0x2AAA771, 10-bit=0x2AAA775, 12-bit=0x2AAA779
 */
#define VFE_0_BUS_CFG_WEBOS_VALUE		0x02AAA771
#define VFE_0_MODULE_CFG_WEBOS_VALUE		0x01C00C0C
#define VFE_0_BUS_CFG_ENC_Y_WR_PATH_EN		BIT(4)
#define VFE_0_BUS_CFG_ENC_CBCR_WR_PATH_EN	BIT(5)
#define VFE_0_BUS_CFG_VIEW_Y_WR_PATH_EN		BIT(6)
#define VFE_0_BUS_CFG_VIEW_CBCR_WR_PATH_EN	BIT(7)
/*
 * RAW pixel data size field is at bits 2-3 (shift = 2), discovered from
 * HTC liboemcamera.so axi_raw_snapshot_config decompilation:
 *   8-bit:  0x2aaa771 (bits 2-3 = 00)
 *   10-bit: 0x2aaa775 (bits 2-3 = 01)
 *   12-bit: 0x2aaa779 (bits 2-3 = 10)
 */
#define VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT	2
#define VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_MASK	(0x3 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_PIXEL_8BIT		(0 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_PIXEL_10BIT		(1 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_PIXEL_12BIT		(2 << VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_SHFT)
#define VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT	10
#define VFE_0_BUS_CFG_RAW_WR_PATH_DISABLED	(0 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)
#define VFE_0_BUS_CFG_RAW_WR_PATH_ENC_CBCR	(1 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)

/*
 * Get BUS_CFG value adjusted for RAW pixel bit depth.
 * @raw_bpp: bits per pixel (8, 10, or 12 for RAW; 0 for non-RAW)
 *
 * From HTC liboemcamera.so axi_raw_snapshot_config decompilation:
 *   8-bit RAW:  BUS_CFG = 0x2aaa771 (bits 2-3 = 00)
 *   10-bit RAW: BUS_CFG = 0x2aaa775 (bits 2-3 = 01)
 *   12-bit RAW: BUS_CFG = 0x2aaa779 (bits 2-3 = 10)
 *
 * The RAW pixel data size field is at bits 2-3 (shift = 2).
 */
static inline u32 vfe31_get_bus_cfg_for_raw(u8 raw_bpp)
{
	u32 base_cfg = VFE_0_BUS_CFG_WEBOS_VALUE;
	u32 raw_size_bits;

	/* Clear existing RAW pixel size bits (2-3) */
	base_cfg &= ~VFE_0_BUS_CFG_RAW_PIXEL_DATA_SIZE_MASK;

	switch (raw_bpp) {
	case 10:
		raw_size_bits = VFE_0_BUS_CFG_RAW_PIXEL_10BIT;
		break;
	case 12:
		raw_size_bits = VFE_0_BUS_CFG_RAW_PIXEL_12BIT;
		break;
	case 8:
	default:
		raw_size_bits = VFE_0_BUS_CFG_RAW_PIXEL_8BIT;
		break;
	}

	return base_cfg | raw_size_bits;
}

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
 * XBAR_CFG0 (0x040) - AXI Output Mode Selection
 * ============================================================================
 *
 * Selects the overall output routing mode. This is the PRIMARY register that
 * determines whether data flows through the ISP pipeline or bypasses it.
 *
 * Values from webOS msm_vfe31.c enum VFE_AXI_OUTPUT_MODE:
 *
 *   Value   Mode                           Description
 *   ─────────────────────────────────────────────────────────────────────────
 *   0x01    OUTPUT_1_AND_3                 Preview + Video with ISP processing
 *                                          Data path: CAMIF → DEMUX → XBAR → WM
 *                                          Uses: WM0+WM4 (Y), WM1+WM5 (CbCr)
 *
 *   0x60    CAMIF_TO_AXI_VIA_OUTPUT_2      Raw bypass direct to WM0
 *                                          Data path: CAMIF → WM0 (bypasses ISP)
 *                                          Uses: WM0 only (raw sensor data)
 *                                          ISP modules (DEMUX, scale, etc) disabled
 *
 *   0x200   OUTPUT_2                       Preview only (older/alternate mode)
 *                                          Data path: Similar to 0x01 but limited
 *
 * ============================================================================
 * Raw/RDI Mode Configuration (AXI_OUT_MODE = 0x60)
 * ============================================================================
 *
 * Raw snapshot mode (CAMIF_TO_AXI_VIA_OUTPUT_2) routes sensor data directly
 * from CAMIF to memory via WM0, bypassing all ISP processing modules.
 *
 * Required configuration for raw mode:
 *   1. AXI_OUT_MODE = 0x60 (this register)
 *   2. MODULE_CFG = 0x00 (disable all ISP modules)
 *   3. CORE_CFG = 0x40 (input mux enable only, no pixel pattern)
 *   4. WM0 configured with raw buffer addresses
 *   5. IRQ_COMPOSITE_MASK: bit 8 set (WM0 → COMPOSITE_DONE_1)
 *   6. XBAR_CFG1 = 0x00 (not used in raw mode)
 *
 * Key differences from YUV mode (0x01):
 *   - Single WM (WM0) vs dual WM (WM0+WM4 for Y, WM1+WM5 for CbCr)
 *   - No DEMUX processing (data passes through unchanged)
 *   - No chroma subsampling or color conversion
 *   - IRQ uses COMPOSITE_DONE_1 (group 1) vs COMPOSITE_DONE_0 (group 0)
 *
 * From webOS msm_vfe31.c raw snapshot configuration:
 *   case CAMIF_TO_AXI_VIA_OUTPUT_2:
 *       *p = 0x60;  // raw snapshot with wm0
 *       vfe31_ctrl->outpath.out1.ch0 = 0;  // raw uses ch0
 *       vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_S;
 *       // IRQ: irq_comp_mask |= (0x1 << (out1.ch0 + 8)) = bit 8
 *
 * IMPORTANT: webOS never validated raw mode on TouchPad hardware. The code
 * exists but was not used in production. Raw capture may not work due to:
 *   - CSIPHY data type filtering (may only pass YUV data types)
 *   - Missing register configuration not present in webOS code
 *   - Hardware limitations on APQ8060 VFE31 silicon
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
 *   [15:8]  OUTPUT1_ROUTING Output1 routing byte
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

#define VFE_0_BUS_XBAR_CFG1_OUT1_ROUTING_MASK	0x0000FF00
#define VFE_0_BUS_XBAR_CFG1_OUT1_ROUTING_SHIFT	8
#define VFE_0_BUS_XBAR_CFG1_OUT1_ROUTING_STANDARD	0x1A

/*
 * ============================================================================
 * XBAR_CFG1 VALUES - UPDATED BASED ON WEBOS + HTC ANALYSIS
 * ============================================================================
 *
 * Analysis of XBAR_CFG1 values and their CbCr routing:
 *
 *   Value   bits[7:4]  CbCr Routing                     Use Case
 *   ─────────────────────────────────────────────────────────────────────────
 *   0x1A03  0x0        CbCr DISABLED                    webOS default (Y only!)
 *   0x1A1B  0x1        CbCr → WM1 only (output0.ch1)    PIX with CbCr
 *   0x1A9B  0x9        CbCr → WM1 + WM5 (both outputs)  PIX + VIDEO CbCr
 *
 * CRITICAL DISCOVERY FROM WEBOS REGISTER DUMPS:
 *
 * WebOS captured register state shows CbCr WMs (WM1, WM4, WM5) were DISABLED
 * during preview/video streaming! Only during PICTURE CAPTURE were CbCr WMs
 * enabled. This means:
 *
 *   1. WebOS never actually streamed CbCr data during preview/video
 *   2. The XBAR routing for CbCr was never validated in practice
 *   3. Color preview was likely reconstructed from raw UYVY input in software
 *
 * From webOS register dumps:
 *   PREVIEW/VIDEO MODE:
 *     WM0 (Y)    = ENABLED
 *     WM1 (CbCr) = DISABLED!
 *     WM4 (Y)    = ENABLED
 *     WM5 (CbCr) = DISABLED!
 *
 *   PICTURE CAPTURE MODE:
 *     WM0 (Y)    = ENABLED
 *     WM1 (CbCr) = ENABLED!
 *     WM4 (Y)    = ENABLED
 *     WM5 (CbCr) = ENABLED!
 *
 * For VIDEO path (WM4/WM5) to receive CbCr, XBAR must be 0x1A9B:
 *   - bits[7:4] = 0x9 routes CbCr to BOTH output0 (WM1) AND output2 (WM5)
 *   - 0x1A1B only routes to output0 (WM1), leaving WM5 without CbCr data
 *
 * Sources:
 *   - webos-linux-kernel-touchpad/drivers/media/video/msm/msm_vfe31.c
 *   - reports/webos-vfe31-analysis-summary.txt (register dump analysis)
 *   - reports/vfe31-video-path-nv16-analysis.md (detailed path analysis)
 *
 * We use VFE31_XBAR_CFG1 (0x1A1B) defined at top of file, matching webOS.
 * VFE31_XBAR_VIDEO_DUAL (0x1A9B) would be needed for VIDEO CbCr output.
 */

#define VFE_0_BUS_CFG_RAW_WR_PATH_VIEW_CBCR	(2 << VFE_0_BUS_CFG_RAW_WR_PATH_SEL_SHFT)

/*
 * VFE31 has 8 Write Masters (WM0-WM7), WM7 at 0x0F4 is unused by all vendors.
 *
 * WM assignments in OUTPUT_1_AND_3 mode (webOS channel mapping):
 *   Preview (output0): ch0=WM0 (Y), ch1=WM4 (CbCr)
 *   Video   (output2): ch0=WM1 (Y), ch1=WM5 (CbCr)
 *
 * XBAR_CFG1 controls which outputs receive data from the DEMUX:
 *   0x1A1B: Y to WM0+WM1, CbCr to WM4 only (WM5 disabled)
 *   0x1A9B: Y to WM0+WM1, CbCr to WM4+WM5 (full dual output)
 *
 * See vfe31-register-reference.md for full WM register map.
 */
#define VFE31_PREVIEW_WM_Y		0  /* WM0 = output0.ch0 (preview Y) */
#define VFE31_PREVIEW_WM_CBCR		4  /* WM4 = output0.ch1 (preview CbCr) */

/*
 * VIDEO Write Master assignments per webOS msm_vfe31.c:
 *   output2.ch0 = WM1 (VIDEO Y)
 *   output2.ch1 = WM5 (VIDEO CbCr)
 *
 * NOTE: Currently VIDEO reuses PIX WMs (WM0+WM4) since only one
 * line runs at a time. These defines document the webOS hardware
 * assignments for future simultaneous PIX+VIDEO support.
 */
#define VFE31_VIDEO_WM_Y		1
#define VFE31_VIDEO_WM_CBCR		5

/*
 * ZSL/Snapshot Write Master assignments per webOS msm_vfe31.c:
 *   output2.ch0 = WM2 (ZSL Y)
 *   output2.ch1 = WM6 (ZSL CbCr)
 *
 * ZSL (Zero Shutter Lag) captures full-resolution snapshot frames
 * while preview continues on WM0+WM4. XBAR byte2 (bits[23:16])
 * routes data to output2 channels (WM2+WM6).
 */
#define VFE31_ZSL_WM_Y			2  /* WM2 = output2.ch0 (snapshot Y) */
#define VFE31_ZSL_WM_CBCR		6  /* WM6 = output2.ch1 (snapshot CbCr) */

/*
 * VFE31 Dummy Buffer for Unused Write Masters
 *
 * The VFE31 XBAR can route data to any of the 7 WMs (WM0-WM6) based on mode.
 * If a WM receives data but has no valid DMA address configured, the system
 * may crash or hang. This dummy buffer acts as a "bit bucket" - all unused
 * WMs point here so any errant DMA writes are safely absorbed.
 *
 * Size calculation:
 *   - Maximum resolution: 1280x1024
 *   - VFE31 Sparse DMA writes at INPUT stride (width*2 = 2560 bytes/line)
 *   - Maximum Y plane: 2560 * 1024 = 2,621,440 bytes (~2.5MB)
 *   - Round up to 3MB to provide safety margin
 *
 * Note: webOS didn't use dummy buffers - they relied on proper WM disable
 * and XBAR configuration. We use this as a safety net because XBAR routing
 * and WM enable/disable may not be perfectly synchronized during mode changes.
 */
#define VFE31_DUMMY_BUF_SIZE		(3 * 1024 * 1024)  /* 3MB */

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
 * Chroma subsample block (0x4F8-0x503): 12 bytes / 3 registers
 * Verified against Mako kernel: V31_CHROMA_SUBS_OFF=0x4F8, V31_CHROMA_SUBS_LEN=12
 *
 * From HTC liboemcamera.so vfe_chroma_subsample_config decompilation:
 *
 *   Offset   Size   Description
 *   ──────────────────────────────────────────────────────────────
 *   0x4F8    1      Config byte:
 *                     Bit 0-1: Reserved (cleared)
 *                     Bit 2:   Format select (mode-dependent)
 *                     Bit 4:   Enable (always set)
 *                     Bit 5:   vsubSampleEnable (1=NV12, 0=NV16)
 *   0x4F9    1      Crop enable, etc.
 *   0x4FA    2      Crop width first pixel (16-bit)
 *   0x4FC    2      Crop width last pixel (16-bit)
 *   0x4FE    2      Crop height first line (16-bit)
 *   0x500    2      Crop height last line (16-bit)
 *   0x502    2      (extends block to 12 bytes)
 *
 * HTC writes command 0x19 with 12 bytes (0xC) to configure this block.
 * Current mainline only writes the config byte at 0x4F8.
 *
 * Values for config byte:
 *   0x30 = NV12 mode (4:2:0): bit 4 (enable) + bit 5 (vsub)
 *   0x10 = NV16 mode (4:2:2): bit 4 (enable) only
 */
#define VFE_0_CHROMA_SUBS_CFG		0x4F8	/* Chroma subsample config */
#define VFE_0_CHROMA_SUBS_CFG2		0x4FC	/* Crop width last pixel */
#define VFE_0_CHROMA_SUBS_CFG3		0x500	/* Crop height last line */

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
 *
 * NOTE: TESTGEN IS NOT FUNCTIONAL ON APQ8060/MSM8660
 *
 * Investigation confirmed that the testgen hardware block does not exist
 * or is not implemented in silicon on APQ8060 VFE31:
 *
 * - TESTGEN_CFG (0x15C) reads back 0x00 after writing - writes don't stick
 * - Adjacent registers (CORE_CFG, MODULE_CFG) work correctly
 * - WebOS VFE31 code has testgen command placeholder but no implementation
 * - VFE32 kernels (Mako, G2) show same pattern - placeholder without code
 * - VFE8x (older) has working testgen with different register layout (0x364+)
 *
 * The register definitions are kept for reference but vfe31_configure_testgen()
 * will not produce any output on APQ8060. The testgen feature was likely
 * removed or never implemented in the MSM8660-era VFE31 silicon.
 */
#define VFE_0_TESTGEN_STATUS		0x158
#define VFE_0_TESTGEN_CFG		0x15C
#define VFE_0_TESTGEN_SEED_0		0x160
#define VFE_0_TESTGEN_SEED_1		0x164
#define VFE_0_TESTGEN_SEED_2		0x168
#define VFE_0_TESTGEN_SEED_3		0x16C
#define VFE_0_TESTGEN_DIMS		0x170
#define VFE_0_TESTGEN_START_PIXEL	0x174

/* TESTGEN_CFG bit definitions (non-functional on APQ8060) */
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
 * (width*2 for PIX/VIDEO mode), NOT the V4L2 format's bytesperline.
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
	writel_relaxed(0x3FF, vfe->base + VFE_0_GLOBAL_RESET_CMD);
	wmb();

	/* Step 4: Wait for reset to complete - webOS waits for RESET_ACK IRQ, we use delay */
	usleep_range(2000, 3000);

	dev_info(vfe->camss->dev,
		 "VFE reset: hardware reset complete, IRQ_STATUS1=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_1));

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
	 * Step 5b: Point ALL Write Masters to dummy buffer.
	 *
	 * This ensures any WM that receives data has a valid DMA address,
	 * preventing crashes if XBAR routes data unexpectedly. The dummy
	 * buffer acts as a bit bucket - data written there is discarded.
	 *
	 * Active WMs will get real buffers assigned during vfe_enable().
	 */
	if (vfe->dummy_buf_addr) {
		int wm;
		u32 dummy_addr = (u32)vfe->dummy_buf_addr;

		dev_info(vfe->camss->dev,
			 "VFE reset: pointing all 7 WMs to dummy buffer 0x%08x\n",
			 dummy_addr);

		for (wm = 0; wm < MSM_VFE_IMAGE_MASTERS_NUM; wm++) {
			writel_relaxed(dummy_addr,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
			writel_relaxed(dummy_addr,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
		}
		wmb();
	}

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

/*
 * vfe31_dump_axi_wm_debug - Dump AXI and WM status for debugging
 * @vfe: VFE device
 *
 * Dumps comprehensive AXI bus and Write Master status for debugging
 * data flow issues, especially in RDI mode where CAMIF shows no data.
 */
static void vfe31_dump_axi_wm_debug(struct vfe_device *vfe)
{
	u32 axi_status, bus_op_status, pp_status, camif_status;
	u32 wm0_cfg, wm0_ping, wm0_pong, wm0_addr_cfg, wm0_ub_cfg, wm0_img_size;
	u32 axi_mode, bus_cfg, core_cfg, module_cfg;
	static int dump_count;

	/* Rate limit to first 5 dumps and every 100th after */
	if (dump_count > 5 && (dump_count % 100) != 0) {
		dump_count++;
		return;
	}
	dump_count++;

	/* Read AXI and bus status registers */
	axi_status = readl_relaxed(vfe->base + VFE_0_AXI_STATUS);
	bus_op_status = readl_relaxed(vfe->base + VFE_0_BUS_OPERATION_STATUS);
	pp_status = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);
	camif_status = readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS);

	/* Read core configuration */
	axi_mode = readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG0);
	bus_cfg = readl_relaxed(vfe->base + VFE_0_BUS_CFG);
	core_cfg = readl_relaxed(vfe->base + VFE_0_CORE_CFG);
	module_cfg = readl_relaxed(vfe->base + VFE_0_MODULE_CFG);

	/* Read WM0 configuration */
	wm0_cfg = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(0));
	wm0_ping = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(0));
	wm0_pong = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(0));
	wm0_addr_cfg = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(0));
	wm0_ub_cfg = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(0));
	wm0_img_size = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(0));

	dev_info(vfe->camss->dev,
		 "VFE31 DEBUG DUMP #%d:\n", dump_count);
	dev_info(vfe->camss->dev,
		 "  AXI_STATUS(0x1DC)=0x%08x BUS_OP_STATUS(0x184)=0x%08x PP_STATUS(0x180)=0x%08x\n",
		 axi_status, bus_op_status, pp_status);
	dev_info(vfe->camss->dev,
		 "  CAMIF_STATUS(0x204)=0x%08x [lines=%d pixels=%d active=%d]\n",
		 camif_status,
		 (camif_status >> 16) & 0x3FFF,
		 camif_status & 0x3FFF,
		 (camif_status >> 31) & 1);
	dev_info(vfe->camss->dev,
		 "  AXI_MODE(0x40)=0x%08x BUS_CFG(0x3C)=0x%08x CORE_CFG(0x14)=0x%08x MODULE_CFG(0x10)=0x%08x\n",
		 axi_mode, bus_cfg, core_cfg, module_cfg);
	dev_info(vfe->camss->dev,
		 "  WM0: CFG=0x%08x PING=0x%08x PONG=0x%08x\n",
		 wm0_cfg, wm0_ping, wm0_pong);
	dev_info(vfe->camss->dev,
		 "  WM0: ADDR_CFG=0x%08x [ub_start=%d ub_depth=%d] UB_CFG=0x%08x [depth=%d height=%d]\n",
		 wm0_addr_cfg,
		 (wm0_addr_cfg >> 16) & 0x3FF, wm0_addr_cfg & 0x3FF,
		 wm0_ub_cfg,
		 (wm0_ub_cfg >> 16) & 0x3FF, (wm0_ub_cfg & 0xFFFF) + 1);
	dev_info(vfe->camss->dev,
		 "  WM0: IMG_SIZE=0x%08x [stride=%d height=%d flags=0x%x]\n",
		 wm0_img_size,
		 ((wm0_img_size >> 16) & 0xFFFF) * 16,
		 ((wm0_img_size >> 4) & 0xFFF) + 1,
		 wm0_img_size & 0xF);
}

static void vfe31_violation_read(struct vfe_device *vfe)
{
	u32 violation = readl_relaxed(vfe->base + VFE_0_VIOLATION_STATUS);
	static int spurious_count;

	/*
	 * If VIOLATION_STATUS = 0, this is a spurious interrupt.
	 * webOS doesn't mask VIOLATION IRQ, so this can happen during
	 * normal operation. Only log actual violations.
	 *
	 * For RDI mode debugging: Dump AXI/WM status on spurious violations
	 * since these fire continuously and indicate data path issues.
	 */
	if (!violation) {
		spurious_count++;
		/* Log first few and every 100th spurious violation */
		if (spurious_count <= 3 || (spurious_count % 100) == 0) {
			dev_info(vfe->camss->dev,
				 "VFE31 VIOLATION IRQ #%d (status=0, spurious)\n",
				 spurious_count);
			/* Dump debug info to help diagnose RDI issues */
			vfe31_dump_axi_wm_debug(vfe);
		}
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
	int return_buffer = 1;  /* Set to 0 if buffer should not be returned to userspace */

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
	 */
	spin_lock_irqsave(&vfe->output_lock, flags);
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	/*
	 * VFE31 PP bit quirk: For some WMs (like WM1 VIDEO Y), the PP bit
	 * never toggles in the status register. However, the CbCr WM's bit
	 * (WM5 for VIDEO, WM4 for PIX) DOES toggle.
	 *
	 * When using 2 WMs (semi-planar Y+CbCr), check the CbCr WM's PP bit
	 * instead of the Y WM's bit, since they complete together via
	 * COMPOSITE_DONE and should have the same state.
	 */
	{
		u8 pp_wm = output->wm_idx[0];  /* Default: Y plane WM */
		if (output->wm_num == 2) {
			/* Use CbCr plane WM - its PP bit actually toggles */
			pp_wm = output->wm_idx[1];
		}
		active_index = (ping_pong >> pp_wm) & 1;

		dev_dbg(vfe->camss->dev,
			"VFE31: wm_done entry: wm=%d PP=0x%x using_bit%d=%d → HW writing to %s, returning %s buffer\n",
			wm, ping_pong, pp_wm, active_index,
			active_index ? "PONG" : "PING",
			active_index ? "PING" : "PONG");
	}

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
			readl_relaxed(vfe->base + VFE_0_BUS_CFG),
			readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG),
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
	 */
	{
		int ready_idx;  /* Track which buffer index we're returning */

		{
			/*
			 * Normal double-buffer mode:
			 * - active_index=1 (PONG active): HW writing to PONG, PING just completed
			 * - active_index=0 (PING active): HW writing to PING, PONG just completed
			 *
			 * Return the buffer from the COMPLETED slot (!active_index).
			 * The buf[] array tracks: buf[0]=PING slot, buf[1]=PONG slot.
			 */
			ready_idx = !active_index;
			ready_buf = output->buf[ready_idx];

			dev_dbg(vfe->camss->dev,
				"VFE31: double-buffer: active=%d returning buf[%d]=0x%08x\n",
				active_index, ready_idx,
				ready_buf ? (u32)ready_buf->addr[0] : 0);
		}

		/*
		 * DEBUG: Verify buffer address matches where hardware wrote.
		 * active_index=1: PONG active, PING just completed → expect hw_ping
		 * active_index=0: PING active, PONG just completed → expect hw_pong
		 */
		{
			u8 y_wm = output->wm_idx[0];
			u32 hw_ping = readl_relaxed(vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(y_wm));
			u32 hw_pong = readl_relaxed(vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(y_wm));
			u32 expected_addr = active_index ? hw_ping : hw_pong;
			u32 returning_addr = ready_buf ? (u32)ready_buf->addr[0] : 0;
			bool addr_match = (returning_addr == expected_addr);

			dev_dbg(vfe->camss->dev,
				"VFE31: buf_verify seq=%d: PING=0x%08x PONG=0x%08x "
				"active=%d expect_%s=0x%08x returning=0x%08x %s\n",
				output->sequence, hw_ping, hw_pong,
				active_index, active_index ? "PING" : "PONG",
				expected_addr, returning_addr,
				addr_match ? "MATCH" : "MISMATCH!");
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
		 * Replace the completed buffer slot with a fresh one.
		 */
		output->buf[ready_idx] = vfe_buf_get_pending(output);
		if (!output->buf[ready_idx]) {
			/*
			 * No next buffer available - reuse same address.
			 * CRITICAL: Don't return ready_buf to userspace since
			 * we're still using its address for DMA. Returning it
			 * would cause list corruption when the next frame
			 * completes and we try to return it again.
			 */
			output->buf[ready_idx] = ready_buf;  /* Keep buffer in slot */
			new_addr = ready_buf->addr;
			return_buffer = 0;
			if (output->state == VFE_OUTPUT_CONTINUOUS)
				output->state = VFE_OUTPUT_SINGLE;
			else if (output->state == VFE_OUTPUT_SINGLE)
				output->state = VFE_OUTPUT_STOPPING;
			dev_dbg(vfe->camss->dev,
				"VFE31: frame drop wm=%d seq=%d: no pending buffer, reusing 0x%08x\n",
				wm, output->sequence, (u32)new_addr[0]);
		} else {
			new_addr = output->buf[ready_idx]->addr;
			/* Stay in CONTINUOUS state */
		}
	}

	/*
	 * Update the buffer address that just completed (was just read from).
	 * active_index tells us which buffer is currently being written to,
	 * so we update the OTHER buffer's address (!active_index).
	 *
	 * When active_index=1 (writing to PONG), PING just completed → update PING
	 * When active_index=0 (writing to PING), PONG just completed → update PONG
	 */
	{
		/*
		 * Normal double-buffer mode:
		 * Update only the INACTIVE register (the one that just completed).
		 * The ACTIVE register is currently being written to by hardware.
		 *
		 * - active_index=1 (PONG active): update PING (just completed)
		 * - active_index=0 (PING active): update PONG (just completed)
		 *
		 * This maintains proper double-buffering with alternating buffers.
		 */
		dev_dbg(vfe->camss->dev,
			"VFE31: wm_done wm=%d PP=0x%x active=%d → updating %s with 0x%08x, seq=%d\n",
			wm, ping_pong, active_index,
			active_index ? "PING" : "PONG",
			(u32)new_addr[0], output->sequence - 1);

		/*
		 * Debug: For 2-WM mode (NV16), log addr[1] to help trace issues.
		 * The CbCr address should be addr[0] + y_plane_size.
		 */
		if (output->wm_num == 2) {
			dev_dbg(vfe->camss->dev,
				"VFE31: 2-WM mode: addr[0]=0x%08x addr[1]=0x%08x cbcr_offset=0x%x\n",
				(u32)new_addr[0], (u32)new_addr[1],
				vfe->active_cbcr_offset);
		}

		/*
		 * Update ALL WM addresses first, then reload ALL at once.
		 * Reloading WM0 before WM4's address is updated causes
		 * Y/CbCr desync and frame shift on every other frame.
		 */
		for (i = 0; i < output->wm_num; i++) {
			if (active_index)
				vfe31_wm_set_ping_addr(vfe, output->wm_idx[i], new_addr[i]);
			else
				vfe31_wm_set_pong_addr(vfe, output->wm_idx[i], new_addr[i]);
		}
		wmb();
		for (i = 0; i < output->wm_num; i++)
			vfe31_bus_reload_wm(vfe, output->wm_idx[i]);
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
		dev_dbg(vfe->camss->dev,
			"VFE31: wm_done complete: WM%d HW_PING=0x%08x HW_PONG=0x%08x PP=0x%x returned_buf=0x%08x seq=%d\n",
			y_wm, hw_ping, hw_pong, hw_pp, (u32)ready_buf->addr[0], output->sequence - 1);
	}

	/*
	 * Capture output state while holding the lock.
	 * The vb2_buffer_done() call must happen outside the lock
	 * (it can trigger other locks), but we need to check state
	 * and update last_buffer atomically.
	 */
	if (output->state == VFE_OUTPUT_STOPPING) {
		output->last_buffer = ready_buf;
		return_buffer = 0;  /* Don't return stopping buffer */
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	if (return_buffer && ready_buf) {
		/*
		 * Return buffer to userspace via vb2_buffer_done().
		 *
		 * Cache sync is handled by vb2's finish() memop - we do NOT
		 * perform manual dma_sync here, matching other VFE implementations
		 * (gen1, 17x, etc.) which also rely on vb2's built-in sync.
		 *
		 * Previous attempts at manual sync caused issues:
		 * - dma_sync_single_for_cpu() crashed (SG buffers non-contiguous)
		 * - dma_sync_sgtable_for_cpu() caused hangs
		 */
		vb2_buffer_done(&ready_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
	/* else: buffer stays with driver for continued DMA use */

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static irqreturn_t vfe31_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	static ktime_t first_irq_time;
	static int irq_count;
	static int camif_error_count;
	static u32 last_ping_pong;  /* Track PP transitions between IRQs */
	ktime_t now;
	u32 value0, value1, ping_pong;
	int i;

	vfe->res->hw_ops->isr_read(vfe, &value0, &value1);

	/* Read ping-pong status to see if data is reaching AXI bus */
	ping_pong = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	now = ktime_get();

	/*
	 * Detect new streaming session: if >1 second since last IRQ,
	 * reset all per-session counters. This ensures debug dumps and
	 * CAMIF error warnings work correctly across stream restarts.
	 */
	if (irq_count == 0 || ktime_ms_delta(now, first_irq_time) > 1000 * (irq_count + 1)) {
		irq_count = 0;
		camif_error_count = 0;
	}

	irq_count++;
	if (irq_count == 1) {
		first_irq_time = now;
		last_ping_pong = ping_pong;
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

	/* Handle IMAGE_COMPOSITE_DONE_0 (PIX line: WM0+WM4 in Group 0) */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(0)) {
		if (vfe->wm_output_map[0] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, 0, ping_pong);
	}

	/*
	 * Handle IMAGE_COMPOSITE_DONE_1 (RDI line: WM0 in Group 1).
	 *
	 * RDI raw bypass (AXI=0x60) uses WM0 routed to COMPOSITE group 1.
	 * This is separate from PIX's group 0 so RDI and PIX can coexist.
	 */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(1)) {
		if (vfe->wm_output_map[0] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, 0, ping_pong);
	}

	/*
	 * Handle IMAGE_COMPOSITE_DONE_2 (VIDEO line: WM1+WM5 in Group 2).
	 *
	 * Currently inactive: VIDEO reuses PIX WMs (WM0+WM4), so VIDEO
	 * frames complete via COMPOSITE_DONE_0 above. When simultaneous
	 * PIX+VIDEO is implemented with separate WMs, this handler will
	 * process WM1 (VIDEO Y) completions.
	 */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(2)) {
		if (vfe->wm_output_map[VFE31_VIDEO_WM_Y] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, VFE31_VIDEO_WM_Y, ping_pong);
	}

	/*
	 * Handle WM0 PING_PONG interrupt (RDI raw bypass fallback).
	 *
	 * In raw bypass mode (AXI=0x60), COMPOSITE_DONE may not fire because
	 * the ISP pipeline is disabled. The individual WM0 PING_PONG interrupt
	 * fires when WM0 completes writing a frame. Only process this if
	 * COMPOSITE_DONE_0 and COMPOSITE_DONE_1 didn't already handle it.
	 */
	if ((value0 & VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(0)) &&
	    !(value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(0)) &&
	    !(value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(1))) {
		if (vfe->wm_output_map[0] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, 0, ping_pong);
	}

	/* Debug: dump WM0 registers on first few IRQs to verify DMA config */
	if (vfe31_dump_wm_regs && irq_count <= 10) {
		u32 wm0_ping = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(0));
		u32 wm0_pong = readl_relaxed(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(0));

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
			 * Use vfe31_wm_done() directly with the ping_pong
			 * snapshot to avoid re-reading stale HW state.
			 */
			vfe31_wm_done(vfe, 0, ping_pong);

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
		/*
		 * Deferred PIX WM enable: enable WM0+WM4 at first frame
		 * boundary after CAMIF start. This prevents frame wrap
		 * artifacts caused by starting DMA mid-frame.
		 *
		 * Samsung vfe31_start_common() does the same: CAMIF starts
		 * but WMs are enabled later via the recording state machine.
		 */
		if (vfe31_pix_wm_pending) {
			struct vfe_output *out = &vfe->line[VFE_LINE_PIX].output;

			writel_relaxed(BIT(0), vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(out->wm_idx[0]));
			if (out->wm_num == 2)
				writel_relaxed(BIT(0), vfe->base +
					VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(out->wm_idx[1]));
			vfe31_pix_wm_pending = false;
			writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);
			dev_info(vfe->camss->dev,
				 "VFE31: PIX WMs enabled at frame boundary (WM%d+WM%d)\n",
				 out->wm_idx[0],
				 out->wm_num == 2 ? out->wm_idx[1] : -1);
		}

		/* Process recording state machine at frame boundary */
		if (vfe31_recording_state == VFE31_REC_START_REQUESTED) {
			/* Enable VIDEO WMs at frame boundary */
			writel_relaxed(BIT(0), vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
			writel_relaxed(BIT(0), vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_CBCR));
			vfe31_recording_state = VFE31_REC_STARTED;
			writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);
			dev_info(vfe->camss->dev, "VFE31: VIDEO recording started (WM%d+WM%d enabled)\n",
				 VFE31_VIDEO_WM_Y, VFE31_VIDEO_WM_CBCR);
		} else if (vfe31_recording_state == VFE31_REC_STOP_REQUESTED) {
			/* Disable VIDEO WMs at frame boundary */
			writel_relaxed(0, vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
			writel_relaxed(0, vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_CBCR));
			vfe31_recording_state = VFE31_REC_STOPPED;
			dev_info(vfe->camss->dev, "VFE31: VIDEO recording stopped\n");
		} else if (vfe31_recording_state == VFE31_REC_STOPPED) {
			vfe31_recording_state = VFE31_REC_IDLE;
		}

		/* Process ZSL state machine at frame boundary */
		if (vfe31_zsl_state == VFE31_REC_START_REQUESTED) {
			/* Enable ZSL WMs at frame boundary */
			writel_relaxed(BIT(0), vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_ZSL_WM_Y));
			writel_relaxed(BIT(0), vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_ZSL_WM_CBCR));
			vfe31_zsl_state = VFE31_REC_STARTED;
			writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);
			dev_info(vfe->camss->dev, "VFE31: ZSL started (WM%d+WM%d enabled)\n",
				 VFE31_ZSL_WM_Y, VFE31_ZSL_WM_CBCR);
		} else if (vfe31_zsl_state == VFE31_REC_STOP_REQUESTED) {
			/* Disable ZSL WMs at frame boundary */
			writel_relaxed(0, vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_ZSL_WM_Y));
			writel_relaxed(0, vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_ZSL_WM_CBCR));
			vfe31_zsl_state = VFE31_REC_STOPPED;
			dev_info(vfe->camss->dev, "VFE31: ZSL stopped\n");
		} else if (vfe31_zsl_state == VFE31_REC_STOPPED) {
			vfe31_zsl_state = VFE31_REC_IDLE;
		}

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
	 * Handle RDI/RAW mode via COMPOSITE_DONE_1.
	 *
	 * VFE31 raw bypass (AXI=0x60) routes data to WM0, same as PIX.
	 * The IRQ_COMPOSITE_MASK maps WM0 to Group 1 for RDI mode
	 * (bit 8 = WM0 in Group 1), so COMPOSITE_DONE_1 fires.
	 *
	 * From webOS: raw snapshot IRQ_COMP_MASK = 0x00000100
	 * = bit 8 = WM0 in Group 1 → COMPOSITE_DONE_1
	 */
	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_COMPOSITE_DONE_n(1)) {
		/* ZSL: WM2+WM6 are in Group 1 */
		if (vfe->wm_output_map[VFE31_ZSL_WM_Y] != VFE_LINE_NONE)
			vfe31_wm_done(vfe, VFE31_ZSL_WM_Y, ping_pong);

		/* RDI: WM0 is in Group 1 for raw bypass mode */
		if (vfe->wm_output_map[0] != VFE_LINE_NONE) {
			enum vfe_line_id lid = vfe->wm_output_map[0];

			if (lid >= VFE_LINE_RDI0 && lid <= VFE_LINE_RDI2)
				vfe31_wm_done(vfe, 0, ping_pong);
		}
	}

	/* Track last PP status for debug purposes */
	last_ping_pong = ping_pong;

	return IRQ_HANDLED;
}

static int vfe31_halt(struct vfe_device *vfe)
{
	unsigned long time;

	/*
	 * Proper AXI halt sequence (from Samsung msm_vfe31.c vfe31_stop):
	 * 1. Stop CAMIF immediately
	 * 2. Disable all IRQs
	 * 3. Clear all pending IRQs
	 * 4. Latch IRQ clear (write 1 to IRQ_CMD)
	 * 5. Enable only AXI_HALT_ACK IRQ
	 * 6. Issue AXI halt command
	 * 7. Wait for halt acknowledgment
	 */

	/* Step 1: Disable all IRQs before stopping CAMIF */
	writel_relaxed(0x0, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(0x0, vfe->base + VFE_0_IRQ_MASK_1);
	wmb();

	/* Step 2: Stop CAMIF immediately */
	writel_relaxed(VFE_0_CAMIF_CMD_STOP_IMMEDIATELY,
		       vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	/* Step 3: Clear all pending IRQs */
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_IRQ_CLEAR_1);

	/* Step 4: Latch IRQ clear */
	writel_relaxed(1, vfe->base + VFE_0_IRQ_CMD);
	wmb();

	/* Step 5: Enable only RESET_ACK in MASK_1 for halt completion */
	writel_relaxed(VFE_0_IRQ_STATUS_1_RESET_ACK,
		       vfe->base + VFE_0_IRQ_MASK_1);

	/* Step 6: Issue AXI halt */
	reinit_completion(&vfe->halt_complete);
	writel_relaxed(VFE_0_AXI_CMD_HALT, vfe->base + VFE_0_AXI_CMD);
	wmb();

	/* Step 7: Wait for halt ACK */
	time = wait_for_completion_timeout(&vfe->halt_complete,
					   msecs_to_jiffies(500));
	if (!time)
		dev_err(vfe->camss->dev, "VFE31: AXI halt timeout\n");

	return 0;
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
	u32 ping_addr = 0, pong_addr = 0;
	u16 width, height, cbcr_height, bytesperline;
	u32 reg;
	unsigned long flags;
	int wm_idx;
	u8 y_wm;  /* Y plane write master (WM0) */

	vfe31_recording_state = VFE31_REC_IDLE;
	vfe31_pix_wm_pending = false;

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
		/* VIDEO line: WM1 (Y) + WM5 (CbCr) - separate from PIX WMs */
		u8 video_y_wm = VFE31_VIDEO_WM_Y;	/* WM1 */
		u8 video_cbcr_wm = VFE31_VIDEO_WM_CBCR;/* WM5 */

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
		dev_info(vfe->camss->dev, "VFE31: VIDEO line using WM%d(Y), WM%d(CbCr)\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else if (line->id == VFE_LINE_ZSL) {
		/* ZSL/Snapshot line: WM2 (Y) + WM6 (CbCr) */
		u8 zsl_y_wm = VFE31_ZSL_WM_Y;		/* WM2 */
		u8 zsl_cbcr_wm = VFE31_ZSL_WM_CBCR;	/* WM6 */

		wm_idx = vfe_reserve_wm_specific(vfe, zsl_y_wm, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for ZSL Y\n",
				zsl_y_wm);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		if (output->wm_num == 2) {
			/*
			 * Reserve CbCr WM but do NOT map to this line.
			 * Both WMs share the same frame buffer. Only the
			 * Y WM triggers buffer completion.
			 */
			wm_idx = vfe_reserve_wm_specific(vfe, zsl_cbcr_wm, line->id);
			if (wm_idx < 0) {
				dev_err(vfe->camss->dev, "VFE31: Cannot reserve WM%d for ZSL CbCr\n",
					zsl_cbcr_wm);
				vfe_release_wm(vfe, output->wm_idx[0]);
				output->state = VFE_OUTPUT_OFF;
				spin_unlock_irqrestore(&vfe->output_lock, flags);
				return wm_idx;
			}
			output->wm_idx[1] = wm_idx;
			vfe->wm_output_map[wm_idx] = VFE_LINE_NONE;
		}
		dev_info(vfe->camss->dev, "VFE31: ZSL line using WM%d(Y), WM%d(CbCr)\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else if (line->id == VFE_LINE_PIX) {
		/* PIX line: WM0 (Y) + WM4 (CbCr) */
		u8 pix_y_wm = 0;
		u8 pix_cbcr_wm = 4;

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
			 * XBAR routing determines where CbCr data goes:
			 *   - WM4 matches XBAR 0x1A1B CbCr routing
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
		dev_info(vfe->camss->dev, "VFE31: PIX line using WM%d(Y), WM%d(CbCr)\n",
			 output->wm_idx[0], output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else {
		/*
		 * RDI/RAW lines use WM0 on VFE31.
		 *
		 * VFE31 has no true RDI hardware. Raw capture uses
		 * CAMIF_TO_AXI_VIA_OUTPUT_2 (AXI=0x60) which routes
		 * sensor data directly to WM0, bypassing DEMUX/XBAR.
		 *
		 * From webOS msm_vfe31.c (CAMIF_TO_AXI_VIA_OUTPUT_2):
		 *   *p = 0x60;        // raw snapshot with wm0
		 *   out1.ch0 = 0;     // channel 0 = WM0
		 *   p1 = ao + 6;      // wm0 for y
		 *
		 * Samsung kernel confirms: "use wm0 only" comment.
		 *
		 * NOTE: Only one RDI line can be active at a time since
		 * they all share WM0. This matches VFE31 hardware which
		 * has a single CAMIF → AXI bypass path.
		 */
		wm_idx = vfe_reserve_wm(vfe, line->id);
		if (wm_idx < 0) {
			dev_err(vfe->camss->dev,
				"VFE31: Cannot reserve WM for RDI line %d\n",
				line->id);
			output->state = VFE_OUTPUT_OFF;
			spin_unlock_irqrestore(&vfe->output_lock, flags);
			return wm_idx;
		}
		output->wm_idx[0] = wm_idx;

		dev_info(vfe->camss->dev,
			 "VFE31: RDI line %d using WM%d (CAMIF_TO_AXI bypass)\n",
			 line->id, output->wm_idx[0]);
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

	y_wm = output->wm_idx[0];  /* Y WM (WM0) */
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
	 */
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
		spin_lock_irqsave(&vfe->output_lock, flags);
		for (wm_idx = 0; wm_idx < output->wm_num; wm_idx++)
			vfe_release_wm(vfe, output->wm_idx[wm_idx]);
		output->state = VFE_OUTPUT_OFF;
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return -EINVAL;
	}

	/* Verify addresses are valid and different */
	if (ping_addr == pong_addr) {
		dev_warn(vfe->camss->dev,
			 "VFE31: WARNING - PING and PONG have same address 0x%08x!\n",
			 ping_addr);
	}

	dev_info(vfe->camss->dev,
		 "VFE31: Y WM%d %ux%u stride=%u ping=0x%08x pong=0x%08x\n",
		 y_wm, width, height, bytesperline, ping_addr, pong_addr);

	/*
	 * Store addresses in pending_* for vfe31_configure_pending_camif() which
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
	 *
	 * For RDI/RAW mode (0x60), use format-aware BUS_CFG that sets the
	 * RAW pixel size bits (2-3) based on the actual format bit depth.
	 */
	if (axi_mode == VFE_0_BUS_AXI_OUT_MODE_RAW_WM0) {
		/* RDI mode: use format-aware BUS_CFG */
		u8 raw_bpp = camss_format_get_bpp(line->formats, line->nformats,
						  line->fmt[MSM_VFE_PAD_SINK].code);
		u32 bus_cfg = vfe31_get_bus_cfg_for_raw(raw_bpp);

		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - RDI mode: BUS_CFG=0x%08x (bpp=%u), AXI=0x%x\n",
			 bus_cfg, raw_bpp, axi_mode);
		writel_relaxed(bus_cfg, vfe->base + VFE_0_BUS_CFG);
	} else {
		/* PIX mode: use base BUS_CFG */
		dev_info(vfe->camss->dev, "VFE31: Step 1 - PIX mode: BUS_CFG=0x%08x, AXI=0x%x\n",
			 VFE_0_BUS_CFG_WEBOS_VALUE, axi_mode);
		writel_relaxed(vfe31_get_bus_cfg(), vfe->base + VFE_0_BUS_CFG);
	}
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
				   vfe31_xbar_cfg1 :
				   vfe31_calc_xbar(true, false, false);
		dev_info(vfe->camss->dev,
			 "VFE31: PIX mode - XBAR CFG1=0x%04x\n", xbar_initial);
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
	 * Step 2: Configure Write Masters using centralized config
	 *
	 * Use vfe31_calc_pix_config() or vfe31_calc_rdi_config() to calculate
	 * all register values deterministically, then apply with vfe31_apply_wm_config().
	 * Module param overrides are checked after calculation for debugging.
	 */
	{
		struct vfe31_line_config cfg = {0};
		u8 cbcr_wm = (output->wm_num == 2) ? output->wm_idx[1] : 0xff;

		/* Calculate configuration based on line type */
		if (is_rdi_line) {
			u8 bpp = camss_format_get_bpp(line->formats, line->nformats,
						      line->fmt[MSM_VFE_PAD_SINK].code);
			vfe31_calc_rdi_config(&cfg, width, height, bpp);
			cfg.path = line->id;
		} else {
			vfe31_calc_pix_config(&cfg, width, height, bytesperline,
					      pix->pixelformat);
			cfg.path = line->id;
		}

		/* Set buffer addresses */
		cfg.y_wm.ping_addr = ping_addr;
		cfg.y_wm.pong_addr = pong_addr;
		if (cfg.has_cbcr) {
			cfg.cbcr_wm.ping_addr = ping_addr + cfg.cbcr_offset;
			cfg.cbcr_wm.pong_addr = pong_addr + cfg.cbcr_offset;
			vfe->active_cbcr_offset = cfg.cbcr_offset;
		}

		/* Log calculated configuration */
		vfe31_dump_line_config(vfe->camss->dev, &cfg);

		/* Apply Y and CbCr WM configuration */
		dev_info(vfe->camss->dev, "VFE31: Step 2 - Applying WM config (Y=WM%d, CbCr=WM%d)\n",
			 y_wm, cbcr_wm);
		vfe31_apply_line_config(vfe, &cfg, y_wm, cbcr_wm);
	}

	/* Reload WMs to apply new configuration */
	dev_info(vfe->camss->dev, "VFE31: Reloading Y WM%d (BUS_CMD)\n", y_wm);
	reg = VFE_0_BUS_CMD_Mx_RLD_CMD(y_wm);
	if (output->wm_num == 2)
		reg |= VFE_0_BUS_CMD_Mx_RLD_CMD(output->wm_idx[1]);  /* CbCr WM */
	writel_relaxed(reg, vfe->base + VFE_0_BUS_CMD);
	wmb();

	/*
	 * VFE31 testgen mode: Skip CSIPHY deferral and start testgen directly.
	 * The test generator produces data internally, no external camera needed.
	 */
	if (vfe31_use_testgen) {
		dev_info(vfe->camss->dev,
			 "VFE31: Testgen mode - starting directly (Y WM%d, line %d)\n",
			 y_wm, line->id);

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
		vfe->camif_pending_wm = y_wm;
		vfe->camif_pending_line_id = line->id;
	} else if (line->id == VFE_LINE_VIDEO && vfe->stream_count > 0) {
		/*
		 * VIDEO joining already-running PIX stream.
		 *
		 * Samsung approach (vfe31_start_recording):
		 * 1. AXI config pre-configures WM1+WM5 (addresses, sizes)
		 * 2. start_recording() sets state = START_REQUESTED
		 * 3. REG_UPDATE IRQ enables WM1+WM5 at frame boundary
		 *
		 * We do the same: configure WM1+WM5 directly (CAMIF is
		 * already running from PIX), then let the recording state
		 * machine enable them at the next frame boundary.
		 *
		 * DO NOT touch: CAMIF, DEMUX, scale, MODULE_CFG, AXI mode
		 * (all already configured by PIX).
		 */
		dev_info(vfe->camss->dev,
			 "VFE31: VIDEO joining active PIX stream (stream_count=%d)\n",
			 vfe->stream_count);

		/* Configure WM1+WM5 registers directly */
		{
			struct vfe31_line_config cfg = {0};
			u8 cbcr_wm = (output->wm_num == 2) ?
				      output->wm_idx[1] : 0xff;

			vfe31_calc_pix_config(&cfg, width, height,
					     bytesperline,
					     pix->pixelformat);

			/* Set buffer addresses */
			cfg.y_wm.ping_addr = ping_addr;
			cfg.y_wm.pong_addr = pong_addr;
			if (output->wm_num == 2 && cfg.has_cbcr) {
				cfg.cbcr_wm.ping_addr = ping_addr +
							cfg.cbcr_offset;
				cfg.cbcr_wm.pong_addr = pong_addr +
							cfg.cbcr_offset;
			}

			vfe31_dump_line_config(vfe->camss->dev, &cfg);
			vfe31_apply_line_config(vfe, &cfg, y_wm, cbcr_wm);

			/* Update XBAR to include VIDEO routing */
			writel_relaxed(vfe31_calc_xbar(true, true, false),
				       vfe->base + VFE_0_BUS_XBAR_CFG1);

			/* Update IRQ comp mask to include Group 2 (VIDEO) */
			vfe->irq_comp_mask_shadow =
				VFE31_IRQ_COMP_MASK_PIX_VIDEO;
			writel_relaxed(vfe->irq_comp_mask_shadow,
				       vfe->base +
				       VFE_0_IRQ_COMPOSITE_MASK_0);
			wmb();
		}

		/* Recording state machine enables WMs at frame boundary */
		vfe31_recording_state = VFE31_REC_START_REQUESTED;
		writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);

		dev_info(vfe->camss->dev,
			 "VFE31: VIDEO WM%d+WM%d configured, waiting for REG_UPDATE to enable\n",
			 y_wm, output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else if (line->id == VFE_LINE_ZSL && vfe->stream_count > 0) {
		/*
		 * ZSL joining already-running PIX stream.
		 *
		 * Same approach as VIDEO-joins-PIX: configure WM2+WM6 directly
		 * (CAMIF is already running from PIX), then let the ZSL state
		 * machine enable them at the next frame boundary.
		 *
		 * DO NOT touch: CAMIF, DEMUX, scale, MODULE_CFG, AXI mode
		 * (all already configured by PIX).
		 */
		dev_info(vfe->camss->dev,
			 "VFE31: ZSL joining active PIX stream (stream_count=%d)\n",
			 vfe->stream_count);

		/* Configure WM2+WM6 registers directly */
		{
			struct vfe31_line_config cfg = {0};
			u8 cbcr_wm = (output->wm_num == 2) ?
				      output->wm_idx[1] : 0xff;
			bool video_active =
				(vfe->line[VFE_LINE_VIDEO].output.state == VFE_OUTPUT_ON ||
				 vfe->line[VFE_LINE_VIDEO].output.state == VFE_OUTPUT_CONTINUOUS);

			vfe31_calc_pix_config(&cfg, width, height,
					     bytesperline,
					     pix->pixelformat);

			/* Set buffer addresses */
			cfg.y_wm.ping_addr = ping_addr;
			cfg.y_wm.pong_addr = pong_addr;
			if (output->wm_num == 2 && cfg.has_cbcr) {
				cfg.cbcr_wm.ping_addr = ping_addr +
							cfg.cbcr_offset;
				cfg.cbcr_wm.pong_addr = pong_addr +
							cfg.cbcr_offset;
			}

			vfe31_dump_line_config(vfe->camss->dev, &cfg);
			vfe31_apply_line_config(vfe, &cfg, y_wm, cbcr_wm);

			/* Update XBAR to include ZSL routing */
			writel_relaxed(vfe31_calc_xbar(true, video_active, true),
				       vfe->base + VFE_0_BUS_XBAR_CFG1);

			/* Update IRQ comp mask to include Group 1 (ZSL) */
			if (video_active)
				vfe->irq_comp_mask_shadow =
					VFE31_IRQ_COMP_MASK_PIX_VID_ZSL;
			else
				vfe->irq_comp_mask_shadow =
					VFE31_IRQ_COMP_MASK_PIX_ZSL;
			writel_relaxed(vfe->irq_comp_mask_shadow,
				       vfe->base +
				       VFE_0_IRQ_COMPOSITE_MASK_0);
			wmb();
		}

		/* ZSL state machine enables WMs at frame boundary */
		vfe31_zsl_state = VFE31_REC_START_REQUESTED;
		writel_relaxed(1, vfe->base + VFE_0_REG_UPDATE_CMD);

		dev_info(vfe->camss->dev,
			 "VFE31: ZSL WM%d+WM%d configured, waiting for REG_UPDATE to enable\n",
			 y_wm, output->wm_num == 2 ? output->wm_idx[1] : -1);
	} else {
		/*
		 * First stream (PIX or VIDEO alone): full CAMIF setup.
		 * Defer to vfe31_configure_pending_camif() which runs
		 * after CSIPHY is ready.
		 */
		dev_info(vfe->camss->dev,
			 "VFE31: Deferring CAMIF config until CSIPHY ready (Y WM%d, line %d)\n",
			 y_wm, line->id);

		vfe->camif_pending = true;
		vfe->camif_pending_wm = y_wm;
		vfe->camif_pending_line_id = line->id;

		/* VIDEO/ZSL starting alone also uses recording state */
		if (line->id == VFE_LINE_VIDEO)
			vfe31_recording_state = VFE31_REC_START_REQUESTED;
		if (line->id == VFE_LINE_ZSL)
			vfe31_zsl_state = VFE31_REC_START_REQUESTED;
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
	/*
	 * VFE31-specific disable sequence (replaces vfe_gen1_disable).
	 * Based on Samsung msm_vfe31.c stop sequence.
	 *
	 * For VIDEO line: request recording stop at frame boundary
	 * via the recording state machine (WMs disabled at REG_UPDATE).
	 *
	 * For all lines: stop CAMIF immediately (not at frame boundary
	 * like gen1 does), then disable WMs and clean up.
	 */
	dev_info(vfe->camss->dev, "VFE31: disable line %d (%s)\n",
		 line->id, is_rdi ? "RDI" : "PIX/VIDEO");

	/* Request recording stop for VIDEO line */
	if (line->id == VFE_LINE_VIDEO &&
	    vfe31_recording_state == VFE31_REC_STARTED)
		vfe31_recording_state = VFE31_REC_STOP_REQUESTED;

	/* Request ZSL stop */
	if (line->id == VFE_LINE_ZSL &&
	    vfe31_zsl_state == VFE31_REC_STARTED)
		vfe31_zsl_state = VFE31_REC_STOP_REQUESTED;

	/*
	 * Disable IRQs before stopping CAMIF to prevent spurious
	 * CAMIF_ERROR from the partial frame caused by STOP_IMMEDIATELY.
	 */
	writel_relaxed(0, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(0, vfe->base + VFE_0_IRQ_MASK_1);
	wmb();

	/* Stop CAMIF immediately (Samsung: CAMIF_COMMAND_STOP_IMMEDIATELY) */
	writel_relaxed(VFE_0_CAMIF_CMD_STOP_IMMEDIATELY,
		       vfe->base + VFE_0_CAMIF_CMD);
	wmb();

	spin_lock_irqsave(&vfe->output_lock, flags);

	/* Disable write masters for this output */
	for (i = 0; i < output->wm_num; i++)
		vfe31_wm_enable(vfe, output->wm_idx[i], 0);

	if (is_rdi) {
		/* RDI: disconnect WM from RDI path */
		vfe31_wm_frame_based(vfe, output->wm_idx[0], 0);
		vfe31_bus_disconnect_wm_from_rdi(vfe, output->wm_idx[0], line->id);
		vfe31_enable_irq_wm_line(vfe, output->wm_idx[0], line->id, 0);
		vfe31_set_cgc_override(vfe, output->wm_idx[0], 0);
	} else {
		/* PIX/VIDEO: disable ISP modules and XBAR */
		for (i = 0; i < output->wm_num; i++)
			vfe31_set_cgc_override(vfe, output->wm_idx[i], 0);
		vfe31_set_module_cfg(vfe, 0);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	/* Release output buffers and WM reservations */
	vfe_put_output(line);

	/* Track stream count (last stream disables bus write interface) */
	mutex_lock(&vfe->stream_lock);
	if (vfe->stream_count == 1)
		vfe->ops_gen1->bus_enable_wr_if(vfe, 0);
	vfe->stream_count--;
	mutex_unlock(&vfe->stream_lock);

	/* Clear CAMIF pending state */
	vfe->camif_pending = false;
	vfe31_recording_state = VFE31_REC_IDLE;
	vfe31_zsl_state = VFE31_REC_IDLE;
	vfe31_pix_wm_pending = false;

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

	/* Use webOS MODULE_CFG value - not just DEMUX bit */
	writel_relaxed(VFE_0_MODULE_CFG_WEBOS_VALUE,
		       vfe->base + VFE_0_MODULE_CFG);

	val = VFE_0_DEMUX_CFG_PERIOD;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_CFG);

	val = VFE_0_DEMUX_GAIN_0_CH0_EVEN | VFE_0_DEMUX_GAIN_0_CH0_ODD;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_0);

	val = VFE_0_DEMUX_GAIN_1_CH1 | VFE_0_DEMUX_GAIN_1_CH2;
	writel_relaxed(val, vfe->base + VFE_0_DEMUX_GAIN_1);

	/*
	 * DEMUX configuration for YUV patterns.
	 * Values from HTC vfe_demux_set_cfg_parms() binary analysis:
	 *
	 * | Pattern | Case | 16-bit Value | Notes |
	 * |---------|------|--------------|-------|
	 * | YUYV    | 4    | 0x9CAC       | YCbYCr |
	 * | YVYU    | 5    | 0xAC9C       | YCrYCb |
	 * | UYVY    | 6    | 0xC9CA       | CbYCrY (webOS default) |
	 * | VYUY    | 7    | 0xCAC9       | CrYCbY |
	 *
	 * The 16-bit value is written to both DEMUX_EVEN_CFG and DEMUX_ODD_CFG.
	 * even_cfg = upper byte, odd_cfg = lower byte
	 */
	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
		/* YUYV (YCbYCr) → 0x9CAC */
		even_cfg = 0x9c;
		odd_cfg = 0xac;
		break;
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		/* YVYU (YCrYCb) → 0xAC9C */
		even_cfg = 0xac;
		odd_cfg = 0x9c;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	default:
		/*
		 * DEMUX config 0xC9CA for UYVY (CbYCrY) input.
		 *
		 * CROSS-VENDOR VERIFIED (HTC/Samsung/Sony/webOS):
		 * All four implementations use identical DEMUX_EVEN_CFG=DEMUX_ODD_CFG=0xC9CA
		 * for UYVY input. This produces proper NV12/NV16 output with CbCr order.
		 *
		 * Source verification:
		 *   HTC: vfe_demux_set_cfg_parms() case 6 (CbYCrY/UYVY)
		 *   Samsung: liboemcamera.so line 56150 - 0xC9CA constant
		 *   Sony: liboemcamera.so same pattern confirmed
		 *   webOS: msm_vfe31.c vfe31_config_demux()
		 *
		 * Note: Previous 0xCAC9 config was inverted and produced NV21/NV61 output.
		 */
		even_cfg = 0xc9;
		odd_cfg = 0xca;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		/* VYUY (CrYCbY) → 0xCAC9 */
		even_cfg = 0xca;
		odd_cfg = 0xc9;
		break;
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
	dev_info(vfe->camss->dev,
		 "VFE31: SCALE_Y: CFG=0x03 H=0x%08x V=0x%08x (%dx%d)\n",
		 (width << 16) | width, (height << 16) | height,
		 width, height);

	/* Scaler 2 - Y pass-through */
	writel_relaxed(0x03, vfe->base + VFE_0_S2Y_CFG);
	writel_relaxed((width << 16) | width, vfe->base + VFE_0_S2Y_H_IMAGE);
	writel_relaxed(0x00310000, vfe->base + VFE_0_S2Y_H_PHASE);
	writel_relaxed((height << 16) | height, vfe->base + VFE_0_S2Y_V_IMAGE);
	writel_relaxed(0x00310000, vfe->base + VFE_0_S2Y_V_PHASE);
	dev_info(vfe->camss->dev,
		 "VFE31: S2Y: CFG=0x03 H=0x%08x V=0x%08x (%dx%d)\n",
		 (width << 16) | width, (height << 16) | height,
		 width, height);

	/* Scaler 2 - CbCr channel (chroma subsampling) */
	writel_relaxed(0x03, vfe->base + VFE_0_S2CBCR_CFG);
	dev_info(vfe->camss->dev, "VFE31: S2CBCR: CFG=0x03\n");

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
	 * Chroma vertical scaling - auto-detect based on format.
	 * vfe31_force_422 affects the format detection.
	 */
	{
		u32 v_out, v_phase, subs_cfg;

		/* Auto: based on format (respects vfe31_force_422) */
		v_out = vfe31_calc_cbcr_height(p, height);

		/* Auto: based on scaling ratio */
		v_phase = (v_out < height) ? 0x00320000 : 0x00310000;

		/*
		 * CHROMA_SUBS_CFG (0x4F8) - Chroma subsampling block control.
		 *
		 * Bit layout (from webOS HTC analysis and Gemini validation):
		 *   Bit 4: Enable - MUST always be set (0x10)
		 *   Bit 5: vsubSampleEnable - controls vertical chroma subsampling
		 *
		 * Values:
		 *   0x10 = Enable only - NV16 (4:2:2, no vertical chroma subsampling)
		 *   0x30 = Enable + vsubSample - NV12 (4:2:0, vertical 2:1 subsampling)
		 *
		 * IMPORTANT: CHROMA_V_IMAGE scaler and CHROMA_SUBS_CFG work TOGETHER:
		 *   - CHROMA_V_IMAGE: Controls the chroma scaler (height to height/2)
		 *   - CHROMA_SUBS_CFG: Enables the subsampler and tells CbCr DMA
		 *     engine to expect half as many lines (for NV12)
		 *
		 * For NV12: CHROMA_V_IMAGE scales 480->240, CHROMA_SUBS_CFG=0x30
		 *           tells hardware to expect 240 CbCr lines.
		 * For NV16: CHROMA_V_IMAGE passes 480->480, CHROMA_SUBS_CFG=0x10
		 *           tells hardware to expect 480 CbCr lines.
		 */
		if (vfe31_is_420_format(p))
			subs_cfg = 0x30;  /* NV12: Enable + vsubSample */
		else
			subs_cfg = 0x10;  /* NV16: Enable only */

		writel_relaxed((v_out << 16) | height, vfe->base + VFE_0_CHROMA_V_IMAGE);
		writel_relaxed(v_phase, vfe->base + VFE_0_CHROMA_V_PHASE);
		writel_relaxed(subs_cfg, vfe->base + VFE_0_CHROMA_SUBS_CFG);

		dev_info(vfe->camss->dev,
			 "VFE31: CHROMA_V: v_out=%d, phase=0x%x, subs=0x%x\n",
			 v_out, v_phase, subs_cfg);
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

	/*
	 * Configure pixel pattern in CORE_CFG + bit 6 (webOS uses 0x46 for UYVY)
	 *
	 * IMPORTANT: Samsung/HTC analysis shows Bayer pixel pattern must be set
	 * correctly even for RAW bypass mode (AXI=0x60).
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
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
		break;
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CRYCBY;
		break;
	/* RAW Bayer SBGGR formats - pattern 2 (BGBGBG) */
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_BGBGBG;
		break;
	/* RAW Bayer SGBRG formats - pattern 3 (GBGBGB) */
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GBGBGB;
		break;
	/* RAW Bayer SGRBG formats - pattern 1 (GRGRGR) */
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GRGRGR;
		break;
	/* RAW Bayer SRGGB formats - pattern 0 (RGRGRG) */
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_RGRGRG;
		break;
	/* Monochrome */
	case MEDIA_BUS_FMT_Y10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_RGRGRG;
		break;
	default:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
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
	 * Use exact webOS MODULE_CFG value (VFE_0_MODULE_CFG_WEBOS_VALUE)
	 * - bit 2: DEMUX
	 * - bit 3: CHROMA_UPSAMPLE
	 * - bits 10-11: Unknown but required by webOS
	 * - bit 24: Unknown but required by webOS
	 */
	u32 val = VFE_0_MODULE_CFG_WEBOS_VALUE;

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
	 * configuration happens in vfe31_configure_pending_camif() which
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
	/*
	 * BUS_CMD has separate PING (bits 0-6) and PONG (bits 7-13) reload
	 * bits for each WM. Reload BOTH to ensure the DMA picks up the new
	 * address regardless of which buffer is currently active.
	 * BIT(wm) = PING reload, BIT(wm+7) = PONG reload.
	 */
	wmb();
	writel_relaxed(BIT(wm) | BIT(wm + 7),
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
	 * These registers are configured in vfe31_configure_pending_camif()
	 * which runs after CAMIF setup.
	 */
	dev_info(vfe->camss->dev,
		 "VFE31: wm_line_based wm=%d enable=%d (deferred to CAMIF start)\n",
		 wm, enable);
}

/*
 * vfe31_configure_pending_camif - Configure and start CAMIF after WM is ready
 *
 * This is called from wm_enable() when camif_pending is set. All WM
 * configuration must be complete before calling this.
 */
static void vfe31_configure_pending_camif(struct vfe_device *vfe, u8 wm)
{
	enum vfe_line_id line_id = vfe->wm_output_map[wm];
	struct vfe_line *line;
	u32 val;
	bool is_rdi_line;
	bool rdi_use_16bpp;
	bool fmt_is_420;
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

	/*
	 * Determine if we should use 16bpp stride for this RDI line.
	 * When vfe31_rdi_force_16bpp is set, RDI mode uses 2 bytes/pixel
	 * instead of the format's actual bpp. This is needed for sensors
	 * like MT9M113 that output 2 bytes/pixel even in "Bayer" mode.
	 */
	rdi_use_16bpp = is_rdi_line && vfe31_rdi_force_16bpp;

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
		/*
		 * RDI mode (0x60): Raw bypass, configure based on format bit depth.
		 * BUS_CFG RAW pixel size field (bits 2-3) must match the format:
		 *   8-bit:  BUS_CFG = 0x2aaa771
		 *   10-bit: BUS_CFG = 0x2aaa775
		 *   12-bit: BUS_CFG = 0x2aaa779
		 */
		u8 raw_bpp = camss_format_get_bpp(line->formats, line->nformats,
						  line->fmt[MSM_VFE_PAD_SINK].code);
		u32 bus_cfg = vfe31_get_bus_cfg_for_raw(raw_bpp);

		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - RDI mode: BUS_CFG=0x%08x (bpp=%u), AXI=0x60 (raw bypass)\n",
			 bus_cfg, raw_bpp);
		writel_relaxed(bus_cfg, vfe->base + VFE_0_BUS_CFG);
		writel_relaxed(VFE_0_BUS_AXI_OUT_MODE_RAW_WM0,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		/* No XBAR configuration for RDI mode */
	} else {
		/*
		 * PIX mode (0x01): Use XBAR to route DEMUX output to WMs
		 *
		 * Both PIX and VIDEO modes use:
		 *   - WM0 (Y) + WM4 (CbCr)
		 *
		 * XBAR routing options:
		 *   - 0x1A03: CbCr→WM4 (correct for our WM assignment)
		 *   - 0x1A1B: CbCr→WM1 (wrong, causes Cb/Cr swap)
		 */
		u32 xbar_value;

		/*
		 * Dynamic XBAR: build from active WM configuration.
		 * Currently single-line (PIX or VIDEO), no simultaneous.
		 * Module param overrides for testing.
		 */
		if (vfe31_xbar_cfg1 != 0) {
			xbar_value = vfe31_xbar_cfg1;
		} else {
			bool video_active = (line_id == VFE_LINE_VIDEO);
			xbar_value = vfe31_calc_xbar(true, video_active, false);
		}

		dev_info(vfe->camss->dev,
			 "VFE31: Step 1 - PIX mode: BUS_CFG=0x%08x, AXI=0x01, XBAR=0x%04x\n",
			 VFE_0_BUS_CFG_WEBOS_VALUE, xbar_value);
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
		 * Upper 16 bits: stride / 16 (128-bit words per line)
		 * Lower 16 bits: ((height - 1) << 4) | 2
		 *
		 * CRITICAL: Use INPUT stride, not output bytesperline!
		 * - UYVY (PIX/VIDEO): 2 bytes/pixel -> width * 2
		 * - RAW8 (RDI): 1 byte/pixel -> width * 1
		 * - RDI with force_16bpp: 2 bytes/pixel -> width * 2
		 * Using output stride causes half-frame capture.
		 */
		{
			u16 image_stride = (is_rdi_line && !rdi_use_16bpp) ? width : (width * 2);
			reg = ((image_stride / 16) & 0xFFFF) << 16;
			reg |= ((height - 1) << 4) | 2;

			dev_info(vfe->camss->dev,
				 "VFE31: WM%d IMAGE_SIZE stride=%d height=%d reg=0x%x\n",
				 wm, image_stride, height, reg);
			writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(wm));
		}

		/*
		 * WR_ADDR_CFG = (UB_start << 16) | UB_depth
		 *
		 * CORRECTED 2026-04-20: This register controls UB SRAM allocation,
		 * NOT DMA burst configuration. Proven by Opal HAL decompilation.
		 *
		 * UB_depth = (plane_pixels * 912) / total_bw - 1
		 * UB_start = 0 for first WM (Y), Y_depth + 1 for CbCr WM
		 *
		 * For RDI: single WM gets full UB budget (0x397 = 911).
		 *
		 * webOS: WM0 = 0x0000012F (start=0, depth=303)
		 */
		{
			u32 ub_depth;

			fmt_is_420 = vfe31_is_420_format(
				line->video_out.active_fmt.fmt.pix_mp.pixelformat);

			if (is_rdi_line) {
				/* RDI: single WM, full UB budget */
				ub_depth = 0x397;  /* 911, matches Samsung raw snapshot */
			} else {
				/*
				 * PIX/VIDEO: proportional UB allocation.
				 * NV12: total_bw = y_pixels * 3 (= 2 * w*h*1.5)
				 * NV16: total_bw = y_pixels * 4 (= 2 * w*h*2.0)
				 */
				u32 y_pixels = width * height;
				u32 total_bw = fmt_is_420 ?
					(y_pixels * 3) : (y_pixels * 4);
				ub_depth = div_u64((u64)y_pixels * 912, total_bw);
				if (ub_depth > 0)
					ub_depth--;
				if (ub_depth < 1)
					ub_depth = 1;
			}
			reg = ub_depth & 0x3ff;  /* UB_start=0 for Y WM */

			dev_info(vfe->camss->dev,
				 "VFE31: WM%d ADDR_CFG=0x%08x (UB start=0, depth=%d)\n",
				 wm, reg, ub_depth);
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
		 * Use INPUT stride based on mode:
		 * - UYVY (PIX/VIDEO): width * 2
		 * - RAW8 (RDI): width * 1
		 * - RDI with force_16bpp: width * 2
		 */
		wpl = ((is_rdi_line && !rdi_use_16bpp) ? width : (width * 2)) / 4;
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
			u8 cbcr_wm = line->output.wm_idx[1];  /* CbCr WM (WM4) */
			u32 y_plane_size;
			u32 cbcr_offset;
			u32 cbcr_ping, cbcr_pong;
			u16 cbcr_height;

			/*
			 * CbCr plane offset = Y plane size in memory.
			 *
			 * VFE31 writes Y compactly at width stride, not bytesperline.
			 * Use width * height for correct CbCr offset.
			 */
			y_plane_size = width * height;
			cbcr_offset = y_plane_size;
			cbcr_ping = vfe->pending_ping_addr + cbcr_offset;
			cbcr_pong = vfe->pending_pong_addr + cbcr_offset;

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
				 cbcr_wm, cbcr_offset, cbcr_ping, cbcr_height, vfe31_force_422);

			/* CbCr WM PING/PONG addresses */
			writel_relaxed(cbcr_ping,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(cbcr_wm));
			writel_relaxed(cbcr_pong,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(cbcr_wm));

			/*
			 * CbCr WM IMAGE_SIZE - MUST use INPUT stride (same as Y WM)
			 * webOS uses stride=1280 for both Y and CbCr IMAGE_SIZE.
			 */
			{
				u16 input_stride = width * 2;  /* UYVY input stride */
				reg = ((input_stride / 16) & 0xFFFF) << 16;
				reg |= ((cbcr_height - 1) << 4) | 2;
				dev_info(vfe->camss->dev,
					 "VFE31: WM%d IMAGE_SIZE=0x%08x (stride=%d height=%d)\n",
					 cbcr_wm, reg, input_stride, cbcr_height);
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IMAGE_SIZE(cbcr_wm));
			}

			/*
			 * CbCr WM ADDR_CFG = (UB_start << 16) | UB_depth
			 *
			 * UB start = Y_depth + 1 (sequential after Y WM).
			 * CbCr depth depends on format (HTC/Sony verified):
			 *   NV12: half-width pixels, separate computation
			 *   NV16: same depth as Y (equal plane sizes)
			 *
			 * webOS NV12: WM4 = 0x01300097 (start=304, depth=151)
			 */
			{
				u32 y_pixels = width * height;
				u32 total_bw = fmt_is_420 ?
					(y_pixels * 3) : (y_pixels * 4);
				u32 y_depth = div_u64((u64)y_pixels * 912, total_bw);
				u32 cb_depth;
				u32 ub_start;

				if (fmt_is_420) {
					u32 cbcr_pixels = (width / 2) * height;
					cb_depth = div_u64((u64)cbcr_pixels * 912, total_bw);
				} else {
					cb_depth = y_depth;
				}

				if (y_depth > 0)
					y_depth--;
				if (cb_depth > 0)
					cb_depth--;
				if (y_depth < 1)
					y_depth = 1;
				if (cb_depth < 1)
					cb_depth = 1;

				ub_start = (y_depth + 1) & 0x3ff;
				reg = (ub_start << 16) | (cb_depth & 0x3ff);
				dev_info(vfe->camss->dev,
					 "VFE31: WM%d ADDR_CFG=0x%08x (UB start=%d, depth=%d)\n",
					 cbcr_wm, reg, ub_start, cb_depth);
				writel_relaxed(reg,
					       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(cbcr_wm));
			}

			/* CbCr WM UB_CFG - MUST use INPUT stride (same as Y WM) */
			{
				u16 input_stride = width * 2;  /* UYVY input stride */
				wpl = input_stride / 4;  /* Same as Y WM */
			}
			reg = ((wpl / 8 - 1) & 0xFFFF) << 16;
			reg |= (cbcr_height - 1) & 0xFFFF;  /* Use cbcr_height, not height */
			writel_relaxed(reg,
				       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(cbcr_wm));
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
	 * IMPORTANT: Samsung/HTC analysis shows that Bayer pixel pattern MUST
	 * be set correctly even for RAW bypass mode (AXI=0x60). Setting val=0
	 * for all RAW formats caused CAMIF to not recognize input data.
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
	/* RAW Bayer SBGGR formats - pattern 2 (BGBGBG) */
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_BGBGBG;
		dev_info(vfe->camss->dev,
			 "VFE31: RAW SBGGR - CORE_CFG pattern=2\n");
		break;
	/* RAW Bayer SGBRG formats - pattern 3 (GBGBGB) */
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GBGBGB;
		dev_info(vfe->camss->dev,
			 "VFE31: RAW SGBRG - CORE_CFG pattern=3\n");
		break;
	/* RAW Bayer SGRBG formats - pattern 1 (GRGRGR) */
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GRGRGR;
		dev_info(vfe->camss->dev,
			 "VFE31: RAW SGRBG - CORE_CFG pattern=1\n");
		break;
	/* RAW Bayer SRGGB formats - pattern 0 (RGRGRG) */
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_RGRGRG;
		dev_info(vfe->camss->dev,
			 "VFE31: RAW SRGGB - CORE_CFG pattern=0\n");
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
			struct vfe_output *zsl_out = &vfe->line[VFE_LINE_ZSL].output;
			/* Consider the line being started as active */
			bool starting_pix = (line->id == VFE_LINE_PIX);
			bool starting_video = (line->id == VFE_LINE_VIDEO);
			bool starting_zsl = (line->id == VFE_LINE_ZSL);
			bool video_state_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool pix_state_active = (pix_out->state == VFE_OUTPUT_ON ||
					   pix_out->state == VFE_OUTPUT_RESERVED ||
					   pix_out->state == VFE_OUTPUT_CONTINUOUS);
			bool zsl_state_active = (zsl_out->state == VFE_OUTPUT_ON ||
					   zsl_out->state == VFE_OUTPUT_RESERVED ||
					   zsl_out->state == VFE_OUTPUT_CONTINUOUS);
			bool video_active = starting_video || video_state_active;
			bool pix_active = starting_pix || pix_state_active;
			bool zsl_active = starting_zsl || zsl_state_active;

			/* Module param override takes priority */
			if (vfe31_irq_comp_mask != 0) {
				vfe->irq_comp_mask_shadow = vfe31_irq_comp_mask;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (module param)\n",
					 vfe->irq_comp_mask_shadow);
			} else if (is_rdi_line) {
				/*
				 * RDI mode: Map WM to composite group 1 (bits 8-15).
				 * COMPOSITE_DONE_1 (IRQ bit 22) fires when WM completes.
				 * This matches webOS raw snapshot mode.
				 */
				u8 wm0 = line->output.wm_idx[0];
				vfe->irq_comp_mask_shadow = (1 << (wm0 + 8));
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (RDI WM%d->group1)\n",
					 vfe->irq_comp_mask_shadow, wm0);
			} else {
				/*
				 * Build composite mask from active lines:
				 *   PIX:   Group 0 (WM0+WM4)
				 *   ZSL:   Group 1 (WM2+WM6)
				 *   VIDEO: Group 2 (WM1+WM5)
				 */
				u32 mask = 0;
				const char *mode_str;

				if (pix_active)
					mask |= VFE31_IRQ_COMP_MASK_PIX_ONLY;
				if (zsl_active)
					mask |= VFE31_IRQ_COMP_MASK_ZSL_ONLY;
				if (video_active)
					mask |= VFE31_IRQ_COMP_MASK_VIDEO_ONLY;

				if (!mask)
					mask = VFE31_IRQ_COMP_MASK_PIX_ONLY;

				if (pix_active && video_active && zsl_active)
					mode_str = "PIX+VIDEO+ZSL";
				else if (pix_active && zsl_active)
					mode_str = "PIX+ZSL";
				else if (pix_active && video_active)
					mode_str = "PIX+VIDEO";
				else if (video_active)
					mode_str = "VIDEO only";
				else if (zsl_active)
					mode_str = "ZSL only";
				else
					mode_str = "PIX only";

				vfe->irq_comp_mask_shadow = mask;
				dev_info(vfe->camss->dev,
					 "VFE31: IRQ_COMPOSITE_MASK=0x%08x (%s)\n",
					 vfe->irq_comp_mask_shadow, mode_str);
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
	 * Issue REG_UPDATE to latch shadow registers.
	 *
	 * Do NOT start CAMIF here. CAMIF must only be started by
	 * enable_pending_camif() which runs AFTER the sensor starts
	 * streaming. Starting CAMIF before the sensor is ready causes
	 * the VFE to run with no input data, corrupting IRQ state.
	 *
	 * webOS sequence: REG_UPDATE + CAMIF_START happen together in
	 * vfe31_start_common(), but that's called AFTER sensor s_stream.
	 * Our configure_pending_camif runs BEFORE sensor s_stream.
	 */
	dev_info(vfe->camss->dev, "VFE31: Issuing REG_UPDATE_CMD (CAMIF deferred to enable_pending)\n");
	writel(1, vfe->base + VFE_0_REG_UPDATE_CMD);
	wmb();

	/*
	 * Step 6: Store WM index for enable_pending_camif.
	 *
	 * Samsung/webOS enable PIX WMs BEFORE CAMIF_START in vfe31_start().
	 * WM enable happens in enable_pending_camif() (Step 11b) which runs
	 * after the sensor starts streaming but before CAMIF_CMD_START.
	 * CAMIF then waits for the next sensor SOF, so WMs are ready before
	 * the first frame arrives. No deferred enable needed for PIX.
	 *
	 * VIDEO/ZSL WMs use the recording state machine (deferred to
	 * REG_UPDATE) since they join an already-running CAMIF.
	 */
	vfe->camif_pending_wm = line->output.wm_idx[0];
	dev_info(vfe->camss->dev,
		 "VFE31: Step 6 - WM%d+WM%d will be enabled before CAMIF start\n",
		 line->output.wm_idx[0],
		 line->output.wm_num == 2 ? line->output.wm_idx[1] : -1);

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

	/* Reload CbCr WM for semi-planar formats */
	if (line->output.wm_num == 2) {
		u8 cbcr_wm = line->output.wm_idx[1];  /* CbCr WM (WM4) */

		dev_info(vfe->camss->dev,
			 "VFE31: Step 7b - BUS_CMD reload WM%d (CbCr plane)\n",
			 cbcr_wm);
		writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(cbcr_wm),
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
	 *
	 * Also skip when VIDEO uses the same WMs as PIX (WM0+WM4), since
	 * the WM configuration was already done in Steps 2-6 above.
	 */
	if (axi_mode == VFE_0_BUS_XBAR_CFG0_PIX_MODE) {
		{
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
		vfe31_configure_pending_camif(vfe, wm);
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
	 * Store the values and write them in vfe31_configure_pending_camif.
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
		 * PIX uses WM0, VIDEO uses WM1, ZSL uses WM2.
		 */
		if (vfe->camif_pending_line_id == VFE_LINE_VIDEO)
			is_primary_wm = (wm == VFE31_VIDEO_WM_Y);
		else if (vfe->camif_pending_line_id == VFE_LINE_ZSL)
			is_primary_wm = (wm == VFE31_ZSL_WM_Y);
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
		 * WM0 is always the Y plane WM for both PIX and VIDEO.
		 */
		bool is_y_wm = (wm == VFE31_PREVIEW_WM_Y || wm == VFE31_VIDEO_WM_Y || wm == VFE31_ZSL_WM_Y);

		if (is_y_wm) {
			vfe->last_y_ping_addr = addr;
			dev_dbg(vfe->camss->dev,
				"VFE31: WM%d PING write 0x%08x (Y)\n", wm, addr);
		} else if (vfe->last_y_ping_addr && vfe->active_cbcr_offset) {
			/*
			 * Non-Y WM with offset set = CbCr. Calculate address.
			 * Works regardless of which WM is used for CbCr.
			 */
			addr = vfe->last_y_ping_addr + vfe->active_cbcr_offset;
			dev_dbg(vfe->camss->dev,
				"VFE31: WM%d PING write 0x%08x (CbCr from Y=0x%08x)\n",
				wm, addr, vfe->last_y_ping_addr);
		} else {
			dev_dbg(vfe->camss->dev,
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
		else if (vfe->camif_pending_line_id == VFE_LINE_ZSL)
			is_primary_wm = (wm == VFE31_ZSL_WM_Y);
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
		 * WM0 is always the Y plane WM for both PIX and VIDEO.
		 */
		bool is_y_wm = (wm == VFE31_PREVIEW_WM_Y || wm == VFE31_VIDEO_WM_Y || wm == VFE31_ZSL_WM_Y);

		if (is_y_wm) {
			vfe->last_y_pong_addr = addr;
			dev_dbg(vfe->camss->dev,
				"VFE31: WM%d PONG write 0x%08x (Y)\n", wm, addr);
		} else if (vfe->last_y_pong_addr && vfe->active_cbcr_offset) {
			addr = vfe->last_y_pong_addr + vfe->active_cbcr_offset;
			dev_dbg(vfe->camss->dev,
				"VFE31: WM%d PONG write 0x%08x (CbCr from Y=0x%08x)\n",
				wm, addr, vfe->last_y_pong_addr);
		} else {
			dev_dbg(vfe->camss->dev,
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
 * WARNING: TESTGEN IS NOT FUNCTIONAL ON APQ8060/MSM8660
 *
 * Investigation confirmed that the testgen hardware block does not exist
 * in APQ8060 silicon. Writes to TESTGEN_CFG do not stick (read back 0x00).
 * This function is kept for potential use on other VFE31 variants but
 * will NOT produce any output on HP TouchPad or other APQ8060 devices.
 *
 * The testgen was designed to bypass CSIPHY/CSID and feed data directly
 * to CAMIF for testing without a real camera sensor.
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
		 * TESTGEN_DIMS: [15:0]=width dimension, [31:16]=height
		 *
		 * The format depends on hardware - some expect pixel count,
		 * others expect byte count. Use module parameter to control.
		 */
		{
			u32 dim_width = vfe31_testgen_pixel_dims ? width : width_bytes;
			dev_info(vfe->camss->dev, "VFE TESTGEN: TESTGEN_DIMS=%ux%u (%s)\n",
				 dim_width, height,
				 vfe31_testgen_pixel_dims ? "pixels" : "bytes");
			writel_relaxed(dim_width | ((u32)height << 16),
				       vfe->base + VFE_0_TESTGEN_DIMS);
		}
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
		 * For testgen, we also need FRAME_CFG to help CAMIF count lines.
		 */
		dev_info(vfe->camss->dev, "VFE TESTGEN: Configuring CAMIF %ux%u (bytes=%u)\n",
			 width, height, width_bytes);
		writel_relaxed(0x40, vfe->base + VFE_0_CAMIF_EFS_CFG);
		/* FRAME_CFG: [13:0]=pixelsPerLine, [29:16]=linesPerFrame */
		writel_relaxed((height << 16) | (width_bytes & 0x3FFF),
			       vfe->base + VFE_0_CAMIF_FRAME_CFG);
		/* WINDOW_WIDTH_CFG: same format */
		writel_relaxed((height << 16) | (width_bytes & 0x3FFF),
			       vfe->base + VFE_0_CAMIF_WINDOW_WIDTH_CFG);
		/* WINDOW_HEIGHT_CFG: last pixel offset */
		writel_relaxed((width_bytes - 1) & 0x3FFF,
			       vfe->base + VFE_0_CAMIF_WINDOW_HEIGHT_CFG);
		/* SUBSAMPLE: line count minus 1, no skip */
		writel_relaxed(height - 1, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_0);
		writel_relaxed(0xFFFFFFFF, vfe->base + VFE_0_CAMIF_SUBSAMPLE_CFG_1);
		wmb();

		/*
		 * Step 4: Enable VFE pipeline modules and configure AXI/XBAR
		 */
		writel_relaxed(VFE_0_MODULE_CFG_WEBOS_VALUE, vfe->base + VFE_0_MODULE_CFG);
		writel_relaxed(VFE_0_BUS_XBAR_CFG0_PIX_MODE,
			       vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG);
		writel_relaxed((vfe31_xbar_cfg1 != 0) ? vfe31_xbar_cfg1 :
			       vfe31_calc_xbar(true, false, false),
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

		/* Wait and read back status registers for debug */
		udelay(500);
		dev_info(vfe->camss->dev, "VFE TESTGEN: TESTGEN_STATUS=0x%08x TESTGEN_CFG=0x%08x\n",
			 readl_relaxed(vfe->base + VFE_0_TESTGEN_STATUS),
			 readl_relaxed(vfe->base + VFE_0_TESTGEN_CFG));
		dev_info(vfe->camss->dev, "VFE TESTGEN: CAMIF_STATUS=0x%08x CORE_CFG=0x%08x\n",
			 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS),
			 readl_relaxed(vfe->base + VFE_0_CORE_CFG));
		dev_info(vfe->camss->dev, "VFE TESTGEN: MODULE_CFG=0x%08x AXI_OUT_MODE=0x%08x\n",
			 readl_relaxed(vfe->base + VFE_0_MODULE_CFG),
			 readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG));
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
	/*
	 * Check if this is an RDI line for format-specific handling.
	 */
	{
		bool is_rdi = (line->id == VFE_LINE_RDI0 ||
			       line->id == VFE_LINE_RDI1 ||
			       line->id == VFE_LINE_RDI2);

		/*
		 * Force 16 bpp for RDI mode if vfe31_rdi_force_16bpp is set.
		 * This is needed for sensors like MT9M113 where the IFP always
		 * outputs 2 bytes per pixel even in "Processed Bayer" mode.
		 */
		if (is_rdi && vfe31_rdi_force_16bpp) {
			bpp = 16;
			dev_info(vfe->camss->dev,
				 "VFE31: RDI force 16bpp enabled (actual format bpp ignored)\n");
		} else {
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
		}
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
	 * PIX mode: Enable DEMUX and processing modules (VFE_0_MODULE_CFG_WEBOS_VALUE)
	 * RDI mode: Disable all modules (0) - data bypasses ISP
	 * RAW-through-PIX mode: Use PIX path but disable DEMUX (0)
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (is_rdi) {
			/*
			 * RDI raw bypass: MODULE_CFG=0 (disable all ISP modules).
			 *
			 * Samsung raw snapshot (operation_mode=3) sets MODULE_CFG
			 * to ~0 with minimal bits. Data bypasses ISP entirely
			 * via AXI=0x60 (CAMIF_TO_AXI). DEMUX must NOT be enabled
			 * for raw bypass - it would try to split raw data as YUV.
			 */
			dev_info(vfe->camss->dev,
				 "VFE31: MODULE_CFG=0x0 (RDI raw bypass)\n");
			writel_relaxed(0, vfe->base + VFE_0_MODULE_CFG);
		} else if (vfe31_raw_pix_mode) {
			/*
			 * RAW-through-PIX mode: Use PIX path but disable DEMUX.
			 * This allows RAW data to pass through without
			 * interpretation as YUV. Only WM0 (Y plane) will
			 * receive data - CbCr is meaningless for RAW.
			 */
			dev_info(vfe->camss->dev,
				 "VFE31: MODULE_CFG=0x0 (RAW-through-PIX, DEMUX disabled)\n");
			writel_relaxed(0, vfe->base + VFE_0_MODULE_CFG);
		} else {
			dev_info(vfe->camss->dev,
				 "VFE31: MODULE_CFG=0x%08x (PIX with DEMUX)\n",
				 VFE_0_MODULE_CFG_WEBOS_VALUE);
			writel_relaxed(VFE_0_MODULE_CFG_WEBOS_VALUE, vfe->base + VFE_0_MODULE_CFG);
		}
	}
	wmb();

	/*
	 * Step 3: Configure CORE_CFG with pixel pattern + input mux enable
	 * webOS uses 0x46 for UYVY: pixel pattern 0x6 + bit 6 (input mux)
	 *
	 * IMPORTANT: Samsung/HTC analysis shows that Bayer pixel pattern MUST
	 * be set correctly even for RAW bypass mode (AXI=0x60). Setting val=0
	 * for all RAW formats caused CAMIF to not recognize input data.
	 *
	 * Bayer pattern mapping:
	 *   SBGGR (B-Gb-Gr-R) → pattern 2 (BGBGBG)
	 *   SGBRG (Gb-B-R-Gr) → pattern 3 (GBGBGB)
	 *   SGRBG (Gr-R-B-Gb) → pattern 1 (GRGRGR)
	 *   SRGGB (R-Gr-Gb-B) → pattern 0 (RGRGRG)
	 */
	switch (line->fmt[MSM_VFE_PAD_SINK].code) {
	/* RAW Bayer SBGGR formats - pattern 2 (BGBGBG) */
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_BGBGBG;
		dev_info(vfe->camss->dev,
			 "VFE31: CORE_CFG RAW SBGGR (code=0x%04x, pattern=2)\n",
			 line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	/* RAW Bayer SGBRG formats - pattern 3 (GBGBGB) */
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GBGBGB;
		dev_info(vfe->camss->dev,
			 "VFE31: CORE_CFG RAW SGBRG (code=0x%04x, pattern=3)\n",
			 line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	/* RAW Bayer SGRBG formats - pattern 1 (GRGRGR) */
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_GRGRGR;
		dev_info(vfe->camss->dev,
			 "VFE31: CORE_CFG RAW SGRBG (code=0x%04x, pattern=1)\n",
			 line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	/* RAW Bayer SRGGB formats - pattern 0 (RGRGRG) */
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_RGRGRG;
		dev_info(vfe->camss->dev,
			 "VFE31: CORE_CFG RAW SRGGB (code=0x%04x, pattern=0)\n",
			 line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	/* Monochrome Y10 - use pattern 0 (arbitrary, CAMIF just needs valid pattern) */
	case MEDIA_BUS_FMT_Y10_1X10:
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_RGRGRG;
		dev_info(vfe->camss->dev,
			 "VFE31: CORE_CFG RAW Y10 (code=0x%04x, pattern=0)\n",
			 line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	/* YUV formats - set appropriate pixel pattern */
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
	default:
		/* Unknown format - fall back to UYVY pattern */
		val = VFE_0_CORE_CFG_PIXEL_PATTERN_CBYCRY;
		dev_warn(vfe->camss->dev,
			 "VFE31: unknown format code=0x%04x, using UYVY pattern\n",
			 line->fmt[MSM_VFE_PAD_SINK].code);
		break;
	}
	/*
	 * Input mux enable (bit 6) routes data to DEMUX. Samsung raw
	 * snapshot sets CORE_CFG=0x01 (pattern only, NO mux enable).
	 * Only enable mux for PIX/VIDEO which use DEMUX processing.
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (!is_rdi)
			val |= VFE_0_CORE_CFG_INPUT_MUX_ENABLE;
	}
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
	 * FRAME_CFG at 0x1E8: Frame dimensions for raw mode
	 *   - webOS leaves at 0 for PIX mode (not needed with DEMUX)
	 *   - For RDI/raw mode, set frame dimensions to help CAMIF count lines
	 * WINDOW_WIDTH_CFG at 0x1EC: (height << 16) | width_bytes
	 * WINDOW_HEIGHT_CFG at 0x1F0: width_bytes - 1
	 * SUBSAMPLE_CFG_0 at 0x1F4: height - 1
	 * SUBSAMPLE_CFG_1 at 0x1F8: 0xFFFFFFFF (no frame skip)
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);
		u32 efs_cfg;

		if (is_rdi && vfe31_rdi_efs_cfg >= 0) {
			/* Use module parameter for RDI EFS_CFG */
			efs_cfg = vfe31_rdi_efs_cfg;
			dev_info(vfe->camss->dev,
				 "VFE31: RDI EFS_CFG=0x%02x (module param)\n", efs_cfg);
		} else {
			/* Default: 0x40 (webOS value) */
			efs_cfg = 0x40;
			dev_info(vfe->camss->dev,
				 "VFE31: EFS_CFG=0x%02x (webOS default)\n", efs_cfg);
		}
		writel_relaxed(efs_cfg, vfe->base + VFE_0_CAMIF_EFS_CFG);
	}

	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (is_rdi) {
			/*
			 * RDI/raw mode: Set FRAME_CFG with frame dimensions.
			 * This tells CAMIF the exact pixelsPerLine and linesPerFrame
			 * so it can count lines even without embedded sync codes.
			 *
			 * FRAME_CFG format:
			 *   [13:0]  = pixelsPerLine (width in bytes)
			 *   [29:16] = linesPerFrame (height)
			 *
			 * webOS never used raw mode, so leaving FRAME_CFG=0 was OK
			 * for their PIX/VIDEO modes. For raw mode we need it set.
			 */
			val = (height << 16) | (width_bytes & 0x3FFF);
			writel_relaxed(val, vfe->base + VFE_0_CAMIF_FRAME_CFG);
			dev_info(vfe->camss->dev,
				 "VFE31: RDI FRAME_CFG=0x%08x (lines=%u, pixels=%u)\n",
				 val, height, width_bytes);
		} else {
			/* PIX/VIDEO mode: webOS leaves FRAME_CFG at 0 */
			writel_relaxed(0, vfe->base + VFE_0_CAMIF_FRAME_CFG);
		}
	}

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
			u32 xbar_val;

			/*
			 * Auto-select XBAR based on CbCr WM assignment:
			 *   - WM4 for CbCr: 0x1A03 (routes CbCr to WM4)
			 *   - WM1 for CbCr: 0x1A1B (routes CbCr to WM1)
			 *
			 * Both PIX and VIDEO modes currently use WM0(Y)+WM4(CbCr),
			 * so they should both use 0x1A03 for correct Cb/Cr order.
			 *
			 * The old logic selected 0x1A1B for VIDEO line, which
			 * routes CbCr to WM1, causing Cb/Cr swap when WM4 is used.
			 */
			if (vfe31_xbar_cfg1 != 0) {
				xbar_val = vfe31_xbar_cfg1;
			} else {
				bool vid = (line->id == VFE_LINE_VIDEO);
				xbar_val = vfe31_calc_xbar(true, vid, false);
			}
			dev_info(vfe->camss->dev,
				 "VFE31: XBAR=0x%04x\n", xbar_val);
			writel_relaxed(xbar_val, vfe->base + VFE_0_BUS_XBAR_CFG1);
		}
	}

	/*
	 * Step 8: Configure BUS_CFG for DMA write paths
	 *
	 * For RDI mode, use format-aware BUS_CFG that sets the correct
	 * RAW pixel data size bits based on bit depth (8/10/12-bit).
	 * For PIX/VIDEO mode, use the base configuration.
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);
		u32 bus_cfg;

		if (is_rdi) {
			u8 raw_bpp = camss_format_get_bpp(line->formats,
							  line->nformats,
							  line->fmt[MSM_VFE_PAD_SINK].code);
			bus_cfg = vfe31_get_bus_cfg_for_raw(raw_bpp);
			dev_info(vfe->camss->dev,
				 "VFE31: BUS_CFG=0x%08x (RDI, bpp=%u)\n",
				 bus_cfg, raw_bpp);
		} else {
			bus_cfg = vfe31_get_bus_cfg();
			dev_info(vfe->camss->dev,
				 "VFE31: BUS_CFG=0x%08x (PIX/VIDEO)\n", bus_cfg);
		}
		writel_relaxed(bus_cfg, vfe->base + VFE_0_BUS_CFG);
	}

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
			 * RDI mode: Use PIX IRQ_MASK_0 plus WM0 PING_PONG.
			 *
			 * In raw bypass (AXI=0x60, MODULE_CFG=0), the ISP pipeline
			 * is disabled so COMPOSITE_DONE may not fire. Enable the
			 * individual WM0 PING_PONG interrupt (bit 8) as fallback.
			 * Both COMPOSITE_DONE_1 and WM0_PING_PONG are enabled so
			 * whichever fires will trigger frame completion.
			 */
			vfe->irq_mask0_shadow = 0x00EFE021 |
				VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(0);
			dev_info(vfe->camss->dev,
				 "VFE31: RDI IRQ_MASK_0=0x%08x (COMPOSITE+WM0_PP)\n",
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
	 * RDI mode: RDI WM (WM2/WM3/WM6) in group 1
	 */
	{
		bool is_rdi = (vfe->camif_pending_line_id == VFE_LINE_RDI0 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI1 ||
			       vfe->camif_pending_line_id == VFE_LINE_RDI2);

		if (is_rdi) {
			/*
			 * RDI: Map RDI WM to composite group 1.
			 * RDI0→WM2, RDI1→WM3, RDI2→WM6.
			 * COMPOSITE_DONE_1 (IRQ bit 22) fires when WM completes.
			 */
			u32 comp_mask = (1 << (vfe->camif_pending_wm + 8));
			dev_info(vfe->camss->dev,
				 "VFE31: RDI COMPOSITE_MASK=0x%08x (WM%d->group1)\n",
				 comp_mask, vfe->camif_pending_wm);
			writel_relaxed(comp_mask, vfe->base + VFE_0_IRQ_COMPOSITE_MASK_0);
		} else {
			/* PIX/VIDEO/ZSL mode - check which lines are active */
			struct vfe_output *video_out = &vfe->line[VFE_LINE_VIDEO].output;
			struct vfe_output *pix_out = &vfe->line[VFE_LINE_PIX].output;
			struct vfe_output *zsl_out = &vfe->line[VFE_LINE_ZSL].output;
			/*
			 * CRITICAL: Consider the line we're currently enabling as active!
			 * The output state may not be updated yet when enable_camif runs.
			 */
			bool starting_pix = (line->id == VFE_LINE_PIX);
			bool starting_video = (line->id == VFE_LINE_VIDEO);
			bool starting_zsl = (line->id == VFE_LINE_ZSL);
			bool video_state_active = (video_out->state == VFE_OUTPUT_ON ||
					     video_out->state == VFE_OUTPUT_RESERVED ||
					     video_out->state == VFE_OUTPUT_CONTINUOUS);
			bool pix_state_active = (pix_out->state == VFE_OUTPUT_ON ||
					   pix_out->state == VFE_OUTPUT_RESERVED ||
					   pix_out->state == VFE_OUTPUT_CONTINUOUS);
			bool zsl_state_active = (zsl_out->state == VFE_OUTPUT_ON ||
					   zsl_out->state == VFE_OUTPUT_RESERVED ||
					   zsl_out->state == VFE_OUTPUT_CONTINUOUS);
			bool video_active = starting_video || video_state_active;
			bool pix_active = starting_pix || pix_state_active;
			bool zsl_active = starting_zsl || zsl_state_active;
			u32 comp_mask;
			const char *mode_str;

			dev_info(vfe->camss->dev,
				 "VFE31: comp_mask select: line=%d starting_pix=%d starting_video=%d starting_zsl=%d\n",
				 line->id, starting_pix, starting_video, starting_zsl);

			/* Module param override takes priority */
			if (vfe31_irq_comp_mask != 0) {
				comp_mask = vfe31_irq_comp_mask;
				mode_str = "module param";
			} else {
				/*
				 * Build composite mask from active lines:
				 *   PIX:   Group 0 (WM0+WM4)
				 *   ZSL:   Group 1 (WM2+WM6)
				 *   VIDEO: Group 2 (WM1+WM5)
				 */
				comp_mask = 0;
				if (pix_active)
					comp_mask |= VFE31_IRQ_COMP_MASK_PIX_ONLY;
				if (zsl_active)
					comp_mask |= VFE31_IRQ_COMP_MASK_ZSL_ONLY;
				if (video_active)
					comp_mask |= VFE31_IRQ_COMP_MASK_VIDEO_ONLY;
				if (!comp_mask)
					comp_mask = VFE31_IRQ_COMP_MASK_PIX_ONLY;

				if (pix_active && video_active && zsl_active)
					mode_str = "PIX+VIDEO+ZSL";
				else if (pix_active && zsl_active)
					mode_str = "PIX+ZSL";
				else if (pix_active && video_active)
					mode_str = "PIX+VIDEO";
				else if (video_active)
					mode_str = "VIDEO only";
				else if (zsl_active)
					mode_str = "ZSL only";
				else
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
	 * Samsung/webOS enable PIX WMs BEFORE CAMIF starts. CAMIF waits for
	 * the next sensor SOF before capturing data, so enabling WMs early
	 * is safe - they won't see data until SOF arrives. This prevents
	 * frame wrap caused by enabling WMs after CAMIF is already running.
	 *
	 * The deferred WM enable (via recording state machine) is only used
	 * for VIDEO/ZSL WMs that join an already-running CAMIF stream.
	 *
	 * RDI mode: Also enable immediately (no REG_UPDATE in raw bypass).
	 */
	{
		struct vfe_line *line = &vfe->line[vfe->camif_pending_line_id];
		struct vfe_output *out = &line->output;
		unsigned int i;

		for (i = 0; i < out->wm_num; i++) {
			writel_relaxed(BIT(0), vfe->base +
				VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(out->wm_idx[i]));
		}
		wmb();
		vfe31_pix_wm_pending = false;

		dev_info(vfe->camss->dev,
			 "VFE31: WMs enabled before CAMIF start (WM%d%s)\n",
			 out->wm_idx[0],
			 out->wm_num == 2 ? "+WM4" : "");
	}

	/*
	 * Step 12: Issue REG_UPDATE command
	 * This latches all the shadow register values on the next VSYNC.
	 */
	writel(1, vfe->base + VFE_0_REG_UPDATE_CMD);
	wmb();

	/*
	 * Step 13: Start CAMIF
	 *
	 * configure_pending_camif() no longer starts CAMIF (it only does
	 * REG_UPDATE), so we always start it here. This is the correct
	 * timing: sensor is streaming, all registers are configured.
	 *
	 * Note: CAMIF_STATUS bit 31 reads as 0x80000000 after VFE reset
	 * even when CAMIF is not running - it is NOT a reliable "active"
	 * indicator. Always issue CAMIF_CMD_START unconditionally.
	 */
	dev_info(vfe->camss->dev, "VFE31: Pre-start CAMIF_STATUS=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS));

	vfe31_dump_axi_wm_debug(vfe);
	writel(VFE_0_CAMIF_CMD_START, vfe->base + VFE_0_CAMIF_CMD);
	wmb();
	dev_info(vfe->camss->dev, "VFE31: CAMIF started\n");

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
		 "VFE31: CAMIF status=0x%08x axi=0x%08x xbar=0x%08x\n",
		 readl_relaxed(vfe->base + VFE_0_CAMIF_STATUS),
		 readl_relaxed(vfe->base + VFE_0_BUS_AXI_OUT_MODE_CFG),
		 readl_relaxed(vfe->base + VFE_0_BUS_XBAR_CFG1));

	/*
	 * NOTE: Do NOT call vfe31_dump_axi_wm_debug() after CAMIF starts!
	 * Reading VFE_0_AXI_STATUS (0x1DC) while CAMIF is active causes
	 * a bus hang on VFE31. The pre-CAMIF dump is sufficient for debugging.
	 */
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
 * NOTE: The dummy buffer (vfe->dummy_buf_*) is NOT freed here because
 * it's allocated once in subdev_init and persists for the device lifetime.
 * This avoids repeated allocation/free cycles. The 3MB buffer will
 * be freed automatically when the device is removed/module unloaded.
 */
static void vfe31_cleanup(struct vfe_device *vfe)
{
	vfe31_recording_state = VFE31_REC_IDLE;
	vfe31_zsl_state = VFE31_REC_IDLE;
	vfe31_pix_wm_pending = false;

	/* Stop CAMIF and clear EFS config */
	writel_relaxed(0, vfe->base + VFE_0_CAMIF_CMD);
	writel_relaxed(0, vfe->base + VFE_0_CAMIF_EFS_CFG);
	vfe->camif_pending = false;

	/*
	 * Disable ALL WMs to ensure they don't continue writing on next session.
	 * This prevents stale DMA activity if the previous capture was aborted.
	 */
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_PREVIEW_WM_Y));
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_PREVIEW_WM_CBCR));
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_Y));
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_VIDEO_WM_CBCR));
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_ZSL_WM_Y));
	writel_relaxed(0, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(VFE31_ZSL_WM_CBCR));
}

static void vfe31_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	dev_info(dev, "VFE31 subdev_init: setting up ops_gen1\n");
	vfe->isr_ops = vfe_isr_ops_gen1;
	vfe->ops_gen1 = &vfe_ops_gen1_3_1;
	vfe->video_ops = vfe_video_ops_gen1;

	/*
	 * Allocate dummy buffer for unused Write Masters.
	 * All WMs point here by default until real buffers are assigned.
	 * This prevents crashes if XBAR routes data to an unconfigured WM.
	 */
	if (!vfe->dummy_buf_vaddr) {
		vfe->dummy_buf_vaddr = dma_alloc_coherent(dev, VFE31_DUMMY_BUF_SIZE,
							  &vfe->dummy_buf_addr,
							  GFP_KERNEL);
		if (vfe->dummy_buf_vaddr) {
			dev_info(dev, "VFE31: Allocated dummy buffer at DMA 0x%pad\n",
				 &vfe->dummy_buf_addr);
		} else {
			dev_warn(dev, "VFE31: Failed to allocate dummy buffer\n");
		}
	}

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
