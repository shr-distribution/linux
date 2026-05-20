/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm VIDC 1080p Video Codec driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Linux-SHR Project
 */

#ifndef __VIDC_CORE_H__
#define __VIDC_CORE_H__

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

/* VIDC Register offsets */
#define VIDC_REG_SW_RESET		0x0000
#define VIDC_REG_INTERRUPT		0x0008
#define VIDC_REG_HOST2RISC_CMD		0x0030
#define VIDC_REG_HOST2RISC_ARG1		0x0034
#define VIDC_REG_HOST2RISC_ARG2		0x0038
#define VIDC_REG_HOST2RISC_ARG3		0x003c
#define VIDC_REG_HOST2RISC_ARG4		0x0040
#define VIDC_REG_RISC2HOST_CMD		0x0044
#define VIDC_REG_RISC2HOST_ARG1		0x0048
#define VIDC_REG_RISC2HOST_ARG2		0x004c
#define VIDC_REG_RISC2HOST_ARG3		0x0050
#define VIDC_REG_RISC2HOST_ARG4		0x0054
#define VIDC_REG_FW_VERSION		0x0058
#define VIDC_REG_FW_STATUS		0x0080
#define VIDC_REG_DRAM_BASE_A		0x0508
#define VIDC_REG_DRAM_BASE_B		0x050c
#define VIDC_REG_AXI_CTRL		0x80014  /* MGEN2MAXI base + 0x14 */
#define VIDC_REG_AXI_STATUS		0x80018  /* MGEN2MAXI base + 0x18 */
#define VIDC_REG_BURST_CONFIG		0x8003c  /* MGEN2MAXI base + 0x3c */
#define VIDC_REG_RETURNED_CH_INST_ID	0x2000
#define VIDC_REG_CH0_INST_ID		0x2040
#define VIDC_REG_CH1_INST_ID		0x2080

/* SW Reset bits */
#define VIDC_RESET_RISC			BIT(0)
#define VIDC_RESET_MC			BIT(1)
#define VIDC_RESET_VIDCCORE		BIT(2)
#define VIDC_RESET_VI			BIT(3)
#define VIDC_RESET_DMX			BIT(4)
#define VIDC_RESET_COMMON		BIT(5)
#define VIDC_RESET_H264			BIT(6)
#define VIDC_RESET_VC1			BIT(7)
#define VIDC_RESET_MPEG4		BIT(8)
#define VIDC_RESET_MPEG2		BIT(9)
#define VIDC_RESET_ALL			0x000
#define VIDC_RESET_NONE			0x3ff

/* HOST2RISC commands */
#define VIDC_CMD_EMPTY			0
#define VIDC_CMD_OPEN_CH		1
#define VIDC_CMD_CLOSE_CH		2
#define VIDC_CMD_SYS_INIT		3
#define VIDC_CMD_FLUSH			4
#define VIDC_CMD_CONTINUE_ENC		7
#define VIDC_CMD_ABORT_ENC		8

/* RISC2HOST responses */
#define VIDC_RESP_EMPTY			0
#define VIDC_RESP_OPEN_CH		1
#define VIDC_RESP_CLOSE_CH		2
#define VIDC_RESP_SEQ_DONE		4
#define VIDC_RESP_FRAME_DONE		5
#define VIDC_RESP_ENC_COMPLETE		7
#define VIDC_RESP_SYS_INIT		8
#define VIDC_RESP_FW_STATUS		9
#define VIDC_RESP_FLUSH_DONE		12
#define VIDC_RESP_ABORT_DONE		13
#define VIDC_RESP_INIT_BUFFERS		15
#define VIDC_RESP_ERROR			32

/* Decode/Encode operation types (OR'd with instance ID) */
#define VIDC_OP_SEQ_HEADER		0x00010000
#define VIDC_OP_FRAME_DATA		0x00020000
#define VIDC_OP_LAST_FRAME		0x00030000
#define VIDC_OP_INIT_BUFFERS		0x00040000

/*
 * INIT_CH sentinel: written to VIDC_REG_CH0_INST_ID before every
 * command to put the channel in a known idle state (DDL requirement).
 */
#define VIDC_INIT_CH_INST_ID		0x0000ffff

/* Address shift for hardware registers */
#define VIDC_ADDR_SHIFT			11

/* Pixel-cache control values for OPEN_CH arg2 */
#define VIDC_PCACHE_DEC_DISABLE		3
#define VIDC_PCACHE_ENC_ENABLE		0

/*
 * Per-instance context memory size: H.264 decode requires 800 KB per
 * channel (DDL_FW_H264DEC_CONTEXT_SPACE_SIZE). The firmware enforces
 * this minimum and returns VIDC_1080P_ERROR_INSUFFICIENT_CONTEXT_SIZE
 * (25) if the OPEN_CH arg4 is smaller.
 */
#define VIDC_CTXT_MEM_SIZE		(800 * SZ_1K)

/*
 * Concurrent instance ceiling. With 800 KB per channel the layout is:
 *   fw (~592 KB) + 2×800 KB ctx + 128 KB desc + 4 KB shm ≈ 2.3 MB
 * which fits within the 3 MB vidc_fw_mem SMI reservation.
 */
#define VIDC_MAX_INSTANCES		2

/*
 * DPB (display picture buffer) sizing for the 1080p tile-NV12 format
 * the hardware writes into. Mirrors legacy DDL helpers in
 * webos-linux-kernel-touchpad/drivers/video/msm/vidc/1080p/ddl/
 * vcd_ddl_helper.c (ddl_get_yuv_buf_size) + vcd_ddl_core.h.
 */
#define VIDC_DPB_TILE_ALIGN_WIDTH	128
#define VIDC_DPB_TILE_ALIGN_HEIGHT	32

/*
 * Per-DPB-slot motion-vector buffer size, conservative upper bound for
 * H.264 at 1080p. Legacy computes this from the slice/macroblock count
 * dynamically; using a fixed 32 KB per slot is wasteful (~half a megabyte
 * total at min_dpb=8) but safe and simple for the initial port.
 */
#define VIDC_DPB_MV_SIZE		SZ_32K

/* Hardware DPB slot count cap (matches VIDC_MAX_DPB_BUFFERS below) */
#define VIDC_DPB_REG_SLOTS		32

/*
 * Per-channel scratch ("descriptor") buffer the firmware uses to
 * record per-frame state during SEQ_HEADER parse and FRAME_DATA
 * decode. Legacy vcd_ddl_helper.c:585 allocates 128 KB.
 */
#define VIDC_DESC_BUF_SIZE			SZ_128K

/*
 * Shared-memory region used to pass per-command parameters between
 * host and on-chip RISC. Layout matches legacy
 * webos-linux-kernel-touchpad/drivers/video/msm/vidc/1080p/ddl/
 * vcd_ddl_shared_mem.c — exact byte offsets are part of the firmware
 * ABI, do not reorder.
 *
 * One region per channel (we use ch0 only). The region's fw-relative
 * offset (bytes, not shifted) is programmed into CH0_SHARED_MEM
 * before every command that reads/writes it: SEQ_HEADER, INIT_BUFFERS,
 * FRAME_DATA.
 */
#define VIDC_SHM_SIZE				SZ_4K	/* legacy uses 1 page */

/*
 * Offset within the SHM page for the metadata input buffer.
 * webOS DDL uses a separate allocation (metadata_shared_input) for this;
 * we carve out 256 bytes (DDL_METADATA_CLIENT_INPUTBUFSIZE) from the
 * high half of the 4 KB SHM page, safely above all SHM fields (max ~0x120).
 * The fw-relative raw byte address written to SHM+0x0044 is therefore
 * (core->shm_offset + VIDC_META_INPUT_OFF).
 */
#define VIDC_META_INPUT_OFF			0x200

#define VIDC_SHM_METADATA_ENABLE		0x0038
#define VIDC_SHM_EXT_METADATA_START_ADDR	0x0044
#define VIDC_SHM_ALLOCATED_LUMA_DPB_SIZE	0x0064
#define VIDC_SHM_ALLOCATED_CHROMA_DPB_SIZE	0x0068
#define VIDC_SHM_ALLOCATED_MV_SIZE		0x006c
#define VIDC_SHM_FLUSH_CMD_TYPE			0x0080
#define VIDC_SHM_FLUSH_CMD_INBUF1		0x0084
#define VIDC_SHM_FLUSH_CMD_INBUF2		0x0088
#define VIDC_SHM_MIN_LUMA_DPB_SIZE		0x00b0
#define VIDC_SHM_MIN_CHROMA_DPB_SIZE		0x00bc

/*
 * VIDC_CMD_FLUSH type values written to VIDC_SHM_FLUSH_CMD_TYPE
 * before issuing the flush command. The legacy DDL doesn't expose
 * named constants for these; values are inferred from typical
 * Qualcomm bitmap conventions.
 */
#define VIDC_FLUSH_INPUT			BIT(0)
#define VIDC_FLUSH_OUTPUT			BIT(1)
#define VIDC_FLUSH_ALL				(VIDC_FLUSH_INPUT | VIDC_FLUSH_OUTPUT)

/*
 * Channel 0 command parameter registers (decode/encode instance 0).
 * Addresses confirmed from webOS vidc_hwio_reg.h (VIDC_BLACKBIRD_REG_BASE +
 * offset).  The 0x0100-0x01FF range does NOT exist in the VIDC hardware —
 * registers jump from 0x0080 to 0x0508 with nothing in between.
 *
 * Decoder and encoder share the 0x2040-0x206c block but assign different
 * meanings to some registers:
 *   0x204c: decoder = descriptor_buffer_addr; encoder = stream_buffer_size
 *   0x2050: decoder = DivX3 height override; encoder = current_y_addr
 *   0x2054: decoder = DivX3 width override;  encoder = current_c_addr
 *   0x2058: decoder = stream_buffersize (total capacity); encoder = intra_frame
 */
#define VIDC_REG_CH0_STREAM_ADDR	0x2044	/* REG_117192 */
#define VIDC_REG_CH0_STREAM_SIZE	0x2048	/* REG_145068 - decoder stream_frame_size */
#define VIDC_REG_CH0_DESC_ADDR		0x204c	/* REG_921356 - decoder desc addr */
#define VIDC_REG_CH0_STREAM_BUF_SIZE	0x2058	/* REG_190381 - decoder stream total capacity */
#define VIDC_REG_CH0_DESC_BUF_SIZE	0x205c	/* REG_85655 */
#define VIDC_REG_CH0_DPB_RELEASE	0x2060	/* REG_86830 */
#define VIDC_REG_CH0_SHARED_MEM		0x2064	/* REG_889944 */
#define VIDC_REG_CH0_DPB_CONFIG		0x2068	/* REG_404623 */
#define VIDC_REG_CH0_CMD_SEQ_NUM	0x206c	/* REG_397087 */

/* Encoder input frame addresses (same 0x2040-0x206c block, encoder usage) */
#define VIDC_REG_CH0_Y_ADDR		0x2050	/* REG_612810 - encoder current_y_addr */
#define VIDC_REG_CH0_C_ADDR		0x2054	/* REG_175608 - encoder current_c_addr */
#define VIDC_REG_CH0_INTRA_FRAME	0x2058	/* REG_190381 - encoder intra_frame flag */

/* Encoder output buffer capacity (0x204c has different meaning for encoder) */
#define VIDC_REG_ENC_OUT_BUF_SIZE	0x204c	/* REG_921356 - encoder stream_buffer_size */

/* Sequence info registers (after SEQ_DONE) — REG_845544..REG_853667 */
#define VIDC_REG_SEQ_IMG_SIZE_Y		0x2004	/* REG_845544 - height */
#define VIDC_REG_SEQ_IMG_SIZE_X		0x2008	/* REG_859906 - width */
#define VIDC_REG_SEQ_MIN_DPB		0x200c	/* REG_490078 */
#define VIDC_REG_SEQ_FRAME_SIZE		0x2018	/* REG_489688 - dec_frm_size */
#define VIDC_REG_SEQ_DISPLAY_INFO	0x201c	/* REG_853667 - progressive/crop */

/* Decode result registers (after FRAME_DONE) */
#define VIDC_REG_DEC_DISPLAY_Y		0x2010	/* REG_640904 */
#define VIDC_REG_DEC_DISPLAY_C		0x2014	/* REG_60114 */
#define VIDC_REG_DEC_DECODE_Y		0x2024	/* REG_378318 */
#define VIDC_REG_DEC_DECODE_C		0x2028	/* REG_203487 */
#define VIDC_REG_DEC_DISPLAY_STATUS	0x201c	/* REG_853667 */
#define VIDC_REG_DEC_DECODE_STATUS	0x202c	/* REG_692991 */

/*
 * Display-status field in VIDC_REG_DEC_DISPLAY_STATUS. Legacy
 * VIDC_1080P_SI_RG7_DISPLAY_STATUS_MASK occupies the low nibble;
 * higher bits encode display-coding and resolution-change flags
 * we don't yet consume.
 */
#define VIDC_DISPLAY_STATUS_MASK		0xF
#define VIDC_DISPLAY_STATUS_DECODE_ONLY		0
#define VIDC_DISPLAY_STATUS_DECODE_AND_DISPLAY	1
#define VIDC_DISPLAY_STATUS_DISPLAY_ONLY	2
#define VIDC_DISPLAY_STATUS_DPB_EMPTY		3
#define VIDC_DISPLAY_STATUS_NOOP		4

/*
 * Encode result registers (after ENC_COMPLETE).
 * The 0x2004-0x2018 block is shared with decode SEQ_DONE result registers;
 * the firmware repurposes them depending on the operation type.
 */
#define VIDC_REG_ENC_FRAME_SIZE		0x2004	/* REG_845544 */
#define VIDC_REG_ENC_PICTURE_COUNT	0x2008	/* REG_859906 */
#define VIDC_REG_ENC_WRITE_PTR		0x200c	/* REG_490078 */
#define VIDC_REG_ENC_FRAME_TYPE		0x2010	/* REG_640904 */
#define VIDC_REG_ENC_LUMA_ADDR		0x2014	/* REG_60114 */
#define VIDC_REG_ENC_CHROMA_ADDR	0x2018	/* REG_489688 */

/*
 * Codec-specific decoder configuration registers. Offsets sourced
 * from legacy
 * webos-linux-kernel-touchpad/drivers/video/msm/vidc/1080p/ddl/
 * vidc_hwio_reg.h (REG_152500 / REG_612810 / REG_175608).
 *
 * VIDC_REG_DEC_MPEG4_PP_FILTER : 2-bit LF_CONTROL field; 0 disables
 *                                the post-loop deblock filter, 1
 *                                enables it. Required for MPEG-4 /
 *                                H.263 decode quality.
 *
 * VIDC_REG_DEC_DIVX3_HEIGHT    : DivX 3.11 manual frame-height
 * VIDC_REG_DEC_DIVX3_WIDTH       override - DivX 3.11 bitstreams
 *                                lack resolution info; without these
 *                                the decoder faults on SEQ_HEADER.
 */
#define VIDC_REG_DEC_MPEG4_PP_FILTER	0x0848
#define VIDC_REG_DEC_DIVX3_HEIGHT	0x2050
#define VIDC_REG_DEC_DIVX3_WIDTH	0x2054

/*
 * Encode configuration registers.
 * Offsets from vidc_hwio_reg.h (VIDC_BLACKBIRD_REG_BASE + offset):
 *   REG_934655 = frame width  @ +0x818
 *   REG_179070 = frame height @ +0x81c
 *   REG_63643  = profile/level @ +0x830
 *     bits 15:8 = level, bits 5:0 = profile
 *     H264 profiles: Main=0, High=1, Baseline=2
 *     H264 levels:   Level3=30, Level4=40, etc.
 *   REG_559908 = RC config @ +0xc5a0
 *     bit 9: frame_level_rc, bit 8: mb_level_rc, bits 5:0: frame_qp
 *   REG_977937 = frame rate (millifps, fps*1000) @ +0xd0d0
 *   REG_166135 = target bitrate (bps) @ +0xc5a8
 *   REG_109072 = QP range @ +0xc5ac
 *     bits 13:8 = max_qp, bits 5:0 = min_qp
 *   REG_550322 = reaction coefficient @ +0xc5b0
 *   REG_783891 = encode picture period @ +0xc504
 *     bit 18: ENC_PIC_TYPE_USE, bits 17:16: B_FRM_CTRL, bits 15:0: I_FRM_CTRL
 *     I_FRM_CTRL=0 (HW default) causes firmware DIVIDE_BY_ZERO in SEQ_HEADER
 *   REG_645603 = input frame format @ +0xc51c
 *     0=planar, 3=TILE_64x32 (NV12MT)
 */
#define VIDC_REG_ENC_FRAME_WIDTH	0x0818
#define VIDC_REG_ENC_FRAME_HEIGHT	0x081c
#define VIDC_REG_ENC_PROFILE_LEVEL	0x0830
#define VIDC_REG_ENC_RC_CONFIG		0xc5a0
#define VIDC_REG_ENC_FRAME_RATE		0xd0d0
#define VIDC_REG_ENC_TARGET_BITRATE	0xc5a8
#define VIDC_REG_ENC_REACTION_COEFF	0xc5b0
#define VIDC_REG_ENC_QP_RANGE		0xc5ac
#define VIDC_REG_ENC_PICTURE_PERIOD	0xc504
#define VIDC_REG_ENC_FRAME_FORMAT	0xc51c

/* Shared-memory offsets for encoder-only params (written via core->shm_vaddr) */
#define VIDC_SHM_ENC_EXT_CTRL		0x0028	/* VIDC_SM_ENC_EXT_CTRL_ADDR   */
#define VIDC_SHM_ENC_VOP_TIMING	0x0030	/* VIDC_SM_ENC_VOP_TIMING_ADDR */
#define VIDC_SHM_ENC_HEC_PERIOD	0x0034	/* VIDC_SM_ENC_HEC_PERIOD_ADDR */
#define VIDC_SHM_ENC_INIT_RC_VALUE	0x011c	/* vidc_sm_set_encoder_init_rc_value */

/* DPB buffer registers (decode) — REG_515200/REG_759068/REG_2067 arrays */
#define VIDC_REG_DPB_LUMA_BASE		0x0700	/* REG_759068 base */
#define VIDC_REG_DPB_CHROMA_BASE	0x0600	/* REG_515200 base */
#define VIDC_REG_DPB_MV_BASE		0x0780	/* MV collocated data base */
#define VIDC_MAX_DPB_BUFFERS		32

/*
 * H.264 decoder work buffer registers (programmed before INIT_BUFFERS).
 * webOS vidc_1080p_set_h264_decode_buffers writes the >>11 fw-relative
 * offset of two scratch buffers the firmware needs for slice decode:
 *
 *   0x068c (REG_931311) = vertical neighbour MV buffer (16 KB, sz_vert_nb_mv)
 *   0x0690 (REG_16277)  = neighbour intra-prediction buffer (32 KB, sz_nb_ip)
 *
 * Sizes from webOS ddl_calc_dec_hw_buffers_size (vcd_ddl_helper.c).
 * Without these the firmware accepts INIT_BUFFERS but never replies.
 */
#define VIDC_REG_H264_VERT_NB_MV	0x068c
#define VIDC_REG_H264_NB_IP		0x0690
#define VIDC_H264_VERT_NB_MV_SIZE	SZ_16K
#define VIDC_H264_NB_IP_SIZE		SZ_32K

/* Recon buffer registers (encode) */
#define VIDC_REG_RECON_LUMA_0		0x0480
#define VIDC_REG_RECON_CHROMA_0		0x0484
#define VIDC_REG_RECON_LUMA_1		0x0488
#define VIDC_REG_RECON_CHROMA_1		0x048c
#define VIDC_MAX_RECON_BUFFERS		4

/* Instance state */
enum vidc_inst_state {
	VIDC_STATE_IDLE,
	VIDC_STATE_OPEN,
	VIDC_STATE_SEQ_PARSED,
	VIDC_STATE_RUNNING,
	VIDC_STATE_STOPPED,
	VIDC_STATE_ERROR,
};

/* AXI control bits */
#define VIDC_AXI_HALT_REQ		BIT(0)
#define VIDC_AXI_RESET			BIT(4)
#define VIDC_AXI_HALT_ACK_MASK		0x3  /* Bits 1:0, not 3:2 */

/* Codec types */
enum vidc_codec {
	VIDC_CODEC_H264_DEC	= 0,
	VIDC_CODEC_VC1_DEC	= 1,
	VIDC_CODEC_MPEG4_DEC	= 2,
	VIDC_CODEC_MPEG2_DEC	= 3,
	VIDC_CODEC_H263_DEC	= 4,
	VIDC_CODEC_VC1_RCV_DEC	= 5,
	VIDC_CODEC_DIVX311_DEC	= 6,
	VIDC_CODEC_DIVX412_DEC	= 7,
	VIDC_CODEC_DIVX502_DEC	= 8,
	VIDC_CODEC_DIVX503_DEC	= 9,
	VIDC_CODEC_H264_ENC	= 16,
	VIDC_CODEC_MPEG4_ENC	= 17,
	VIDC_CODEC_H263_ENC	= 18,
};

struct vidc_inst;

struct vidc_core {
	struct device *dev;
	void __iomem *base;
	int irq;

	/* Clocks */
	struct clk *core_clk;
	struct clk *iface_clk;
	struct clk *axi_clk;

	/* Power */
	struct regulator *gdsc;

	/* Interconnect */
	struct icc_path *icc_path;	/* video-smi: RISC fetch from SMI */
	struct icc_path *icc_ebi_path;	/* video-ebi: RISC fetch from EBI/DRAM */
	u32 icc_bw_avg;		/* average bandwidth in bytes/sec */
	u32 icc_bw_peak;	/* peak bandwidth in bytes/sec */

	/*
	 * Firmware buffer — lives in SMI (System Memory Interface) memory.
	 *
	 * The VIDC RISC CPU fetches instructions via the MMSS fabric SMI
	 * slave port. It cannot reach EBI/DRAM. Firmware must be placed in
	 * the reserved SMI region (vidc_fw_mem, 0x38000000, 3MB).
	 *
	 * SMI is not DMA-able from the CPU perspective; we use ioremap_wc()
	 * for write access during firmware copy, then iounmap() it. The RISC
	 * uses the raw physical address via DRAM_BASE_A/B.
	 *
	 * DRAM_BASE_A/B must be 128KB-aligned (bits [31:17] are used).
	 * SMI base 0x38000000 is already 128KB-aligned.
	 *
	 *   fw_phys_base  - physical base of the SMI reserved region (from DT)
	 *   fw_phys_size  - size of the reserved region
	 *   fw_dma_addr   - 128KB-aligned physical address written to DRAM_BASE
	 *   fw_vaddr      - ioremap_wc() mapping for CPU write access
	 *   fw_size       - size of the firmware blob
	 *   fw_alloc_size - total allocated size (fw + ctxt pool + desc + shm)
	 */
	const struct firmware *fw;
	phys_addr_t fw_phys_base;	/* SMI reserved region physical base */
	size_t fw_phys_size;		/* SMI reserved region size */
	dma_addr_t fw_dma_addr;		/* 128KB-aligned phys for DRAM_BASE */
	void *fw_vaddr;			/* ioremap_wc() virtual address */
	size_t fw_size;			/* firmware blob size */
	size_t fw_alloc_size;		/* total layout size in SMI */
	/* Legacy fields — kept for iounmap reference tracking */
	dma_addr_t fw_alloc_dma_addr;
	void *fw_alloc_vaddr;
	size_t fw_align_off;

	/*
	 * Context-memory pool. Sits immediately after the firmware
	 * within the same dma_alloc_coherent() block so that each
	 * instance's context buffer has a small, fixed offset from
	 * fw_dma_addr (which the on-chip RISC sees as DRAM_BASE_A).
	 * ctxt_pool_next is bumped per vidc_open_channel(); we don't
	 * recycle slots — open/close cycles within one driver lifetime
	 * are bounded by VIDC_MAX_INSTANCES.
	 */
	size_t ctxt_pool_size;
	size_t ctxt_pool_used;

	/*
	 * Per-channel descriptor (scratch) buffer. Used by the firmware
	 * for per-frame state during SEQ_HEADER + FRAME_DATA. Address
	 * goes in CH0_DESC_ADDR (>>VIDC_ADDR_SHIFT encoding) and the
	 * size in CH0_DESC_BUF_SIZE (raw bytes).
	 */
	u32 desc_offset;	/* fw-relative byte offset */

	/*
	 * Shared-memory region. One per channel (we use ch0 only).
	 * Sits inside the firmware-adjacent allocation so its
	 * fw-relative offset is a byte value the firmware can use
	 * directly (no shift). Programmed into CH0_SHARED_MEM before
	 * each command that reads/writes parameters.
	 *
	 *   shm_vaddr     - kernel virtual base of the SHM area
	 *   shm_offset    - byte offset from fw_dma_addr (== value
	 *                   written to CH0_SHARED_MEM register)
	 */
	void *shm_vaddr;
	u32 shm_offset;

	/* V4L2 */
	struct v4l2_device v4l2_dev;
	struct video_device *vfd_dec;
	struct video_device *vfd_enc;
	struct v4l2_m2m_dev *m2m_dev_dec;
	struct v4l2_m2m_dev *m2m_dev_enc;

	/* State */
	struct mutex lock;
	spinlock_t irqlock;
	/*
	 * fw_loaded - request_firmware() succeeded and the binary is
	 *             memcpy'd into fw_vaddr; the DRAM contents survive
	 *             runtime suspend / GDSC drop. One-time per probe.
	 * fw_running - the on-chip RISC has been booted via DRAM_BASE_A
	 *              programming + SYS_INIT and ack'd; cleared when
	 *              runtime suspend drops the GDSC. Must be re-set
	 *              via vidc_boot_firmware() on the next resume
	 *              before any HOST2RISC command will be honoured.
	 */
	bool fw_loaded;
	bool fw_running;
	u32 fw_version;
	/*
	 * Boot handshake completions. Legacy webOS DDL (see
	 * webos-linux-kernel-touchpad/drivers/video/msm/vidc/1080p/ddl/
	 * vcd_ddl_interrupt_handler.c:38) shows the on-chip RISC fires
	 * two separate IRQs on bring-up:
	 *
	 *   FW_STATUS_RET  (cmd=9) — unsolicited "I'm alive" raised when
	 *                            SW_RESET is released
	 *   SYS_INIT_RET   (cmd=8) — response to HOST2RISC SYS_INIT
	 *
	 * Host must wait for FW_STATUS before sending SYS_INIT. Sending
	 * SYS_INIT before the RISC is ready was the failure mode for the
	 * mainline driver — it timed out on sys_init_done because the
	 * firmware hadn't even started executing.
	 */
	struct completion fw_status_done;
	struct completion sys_init_done;

	/*
	 * Defensive IRQ-storm guard. The hardware will raise cmd=0
	 * (RISC2HOST_CMD = EMPTY) IRQs in tight loops when the on-chip
	 * RISC is wedged — e.g. when boot stub never completes and the
	 * firmware never sends FW_STATUS_RET. After exceeding the
	 * threshold the ISR disables its own line via disable_irq_nosync
	 * to keep the kernel alive while userspace times out cleanly.
	 */
	unsigned int empty_irq_streak;
	bool irq_disabled_by_storm;
	/*
	 * True when the firmware booted in recovery mode (GDSC cycle
	 * wipe, cmd=51 seen on re-init).  In recovery mode the on-chip
	 * RISC ACKs SEQ_HEADER with cmd=0 (EMPTY) instead of a proper
	 * RESP_SEQ_DONE (cmd=4).  This flag guards the recovery path in
	 * the EMPTY IRQ handler so that the same EMPTY IRQ that fires as
	 * a normal command-received ACK in clean-boot mode is not
	 * mishandled as an early SEQ_DONE.
	 */
	bool fw_recovery_mode;

	/* Instance management */
	struct list_head instances;
	unsigned int num_instances;
	struct vidc_inst *curr_inst;	/* Currently processing instance */
	u32 cmd_seq_num;		/* Command sequence number */
};

/* Forward declaration */
struct vidc_format;

struct vidc_inst {
	struct vidc_core *core;
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct mutex lock;
	struct list_head list;

	enum vidc_codec codec;
	bool decoder;
	u32 inst_id;

	/* State machine */
	enum vidc_inst_state state;
	struct completion done;
	int error;

	/* Format info */
	const struct vidc_format *fmt_out;
	const struct vidc_format *fmt_cap;
	u32 width;
	u32 height;
	u32 out_width;
	u32 out_height;
	u32 out_buf_size;	/* OUTPUT queue sizeimage from S_FMT (compressed input) */

	/* Streaming state */
	bool streamon_out;
	bool streamon_cap;
	u32 sequence_out;
	u32 sequence_cap;

	/* Current frame being processed */
	struct vb2_v4l2_buffer *src_buf;
	struct vb2_v4l2_buffer *dst_buf;

	/* Encoder parameters */
	u32 framerate;
	u32 bitrate;

	/* Sequence info from firmware */
	u32 seq_width;
	u32 seq_height;
	u32 min_dpb_count;

	/*
	 * Last frame result.
	 *
	 * result_size       - bytes the firmware decoded (legacy
	 *                     SEQ_FRAME_SIZE register). Currently only
	 *                     used for the encode path; the decode path
	 *                     computes plane sizes from inst->width /
	 *                     inst->height instead because the firmware's
	 *                     value is the compressed-frame consumed size,
	 *                     not the output payload size.
	 * display_y_raw     - DEC_DISPLAY_Y register value (fw-relative
	 *                     offset >> VIDC_ADDR_SHIFT). Used by the
	 *                     device_run thread to identify which DPB
	 *                     slot the firmware wrote the displayable
	 *                     frame into, so we can memcpy it out to
	 *                     the userspace CAPTURE buffer.
	 * display_c_raw     - same for the chroma plane (DEC_DISPLAY_C).
	 */
	u32 result_size;
	u32 display_y_raw;
	u32 display_c_raw;

	/*
	 * display_status from VIDC_REG_DEC_DISPLAY_STATUS, low nibble.
	 * Tells frame_done_work whether the current FRAME_DONE event
	 * means "decoded a new frame and want to display it now"
	 * (DECODE_AND_DISPLAY) vs "decoded a B-frame, holding it back
	 * for later reorder" (DECODE_ONLY) vs other special cases.
	 * See VIDC_DISPLAY_STATUS_* in vidc_core.h.
	 */
	u32 display_status;

	/*
	 * Per-instance context memory. Allocated as a sub-region of the
	 * core's firmware buffer (so the offset from DRAM_BASE_A is a
	 * single u32 the on-chip RISC can address). Filled in by
	 * vidc_open_channel(); zero when the instance has no open channel.
	 */
	dma_addr_t ctxt_mem_dma_addr;
	void *ctxt_mem_vaddr;
	u32 ctxt_mem_offset;	/* offset from core->fw_dma_addr */
	bool ch_open;

	/*
	 * Stateful-decoder gate. False until the firmware has parsed the
	 * stream's sequence header (SPS for H.264, VOL for MPEG-4, etc.)
	 * and responded with RESP_SEQ_DONE. While false, frame submissions
	 * use VIDC_OP_SEQ_HEADER so the firmware treats the buffer as a
	 * codec-config blob; after the SOURCE_CHANGE event is emitted to
	 * userspace, subsequent submissions use VIDC_OP_FRAME_DATA.
	 * Cleared again on vidc_close_channel().
	 */
	bool seq_parsed;

	/*
	 * Internal DPB (reference frame) pool. Allocated after SEQ_DONE
	 * once the firmware reports min_dpb_count + bitstream dimensions.
	 * The hardware writes decoded frames into these buffers; we copy
	 * out to the userspace CAPTURE buffer in the FRAME_DONE handler.
	 *
	 *   dpb_*_vaddr        - dma_alloc_coherent return values
	 *   dpb_*_dma_addr     - bus addresses (same coherent alloc)
	 *   dpb_y_size         - per-slot luma size (NV12 tile geometry)
	 *   dpb_c_size         - per-slot chroma size (= dpb_y_size / 2)
	 *   dpb_mv_size        - per-slot MV size (H.264 only, else 0)
	 *   dpb_count          - actual allocated slot count (>= min_dpb_count)
	 *   dpb_inited         - true once RESP_INIT_BUFFERS has acked
	 */
	void *dpb_y_vaddr;
	dma_addr_t dpb_y_dma_addr;
	size_t dpb_y_alloc_size;
	u32 dpb_y_size;
	u32 dpb_c_size;
	u32 dpb_mv_size;
	u32 dpb_count;
	bool dpb_inited;
	/*
	 * DPB slot availability bitmask for DPB_RELEASE register.  Bit N=1
	 * means slot N is free for the firmware to write into.  Initialised
	 * to (1<<dpb_count)-1 (all free) after INIT_BUFFERS.  After a
	 * FRAME_DONE the IRQ handler clears the used slot's bit; after the
	 * frame is copied to the CAPTURE buffer, the frame_done_work re-sets
	 * it so it's available for the next decode cycle.
	 */
	u32 dpb_hw_mask;

	/*
	 * H.264 decoder work buffers programmed via VIDC_REG_H264_VERT_NB_MV
	 * and VIDC_REG_H264_NB_IP before INIT_BUFFERS.  Allocated from the
	 * VIDC device's SMIPOOL coherent DMA pool so the firmware can reach
	 * them.  Zero when the instance is not H.264 or has not yet reached
	 * vidc_init_buffers().
	 */
	void *h264_vert_nb_mv_vaddr;
	dma_addr_t h264_vert_nb_mv_dma_addr;
	void *h264_nb_ip_vaddr;
	dma_addr_t h264_nb_ip_dma_addr;

	/*
	 * Async completion work. The IRQ handler can't sleep, but the
	 * post-frame work (V4L2 event queue, vb2 buf_done, DPB copy)
	 * involves locking that can. Have the IRQ stash response state
	 * and schedule one of these workqueue items; the work runs in
	 * process context where blocking is fine.
	 *
	 *   seq_done_work   — fired on RESP_SEQ_DONE. Allocates DPB,
	 *                     issues INIT_BUFFERS (synchronously),
	 *                     emits V4L2_EVENT_SOURCE_CHANGE, marks
	 *                     the consumed buffers DONE.
	 *   frame_done_work — fired on RESP_FRAME_DONE. Copies the
	 *                     displayed DPB slot to the userspace
	 *                     CAPTURE buffer and marks both buffers
	 *                     DONE.
	 *   seq_header_work — decoder only. The V4L2 M2M scheduler
	 *                     requires both queues to have buffers
	 *                     before calling device_run, but stateful
	 *                     decoders need to send SEQ_HEADER with
	 *                     just an OUTPUT buffer present (before
	 *                     the CAPTURE queue is configured).
	 *                     seq_header_work bypasses M2M and submits
	 *                     SEQ_HEADER directly when the first OUTPUT
	 *                     buffer is queued.
	 *
	 * seq_done_work and frame_done_work end with v4l2_m2m_job_finish
	 * so the m2m worker is free to call device_run again.
	 * seq_header_work does NOT go through M2M; seq_hdr_direct flags
	 * this so seq_done_work skips the job_finish call.
	 */
	struct work_struct seq_done_work;
	struct work_struct frame_done_work;
	struct work_struct seq_header_work;
	bool seq_hdr_direct;		/* SEQ_HEADER sent outside M2M job */

	/*
	 * enc_complete_work — encoder-side analog of frame_done_work.
	 * Scheduled from the IRQ handler on RESP_ENC_COMPLETE.
	 * INIT_WORK happens in vidc_enc_open (decoder instances leave
	 * it uninitialised since they never see ENC_COMPLETE).
	 */
	struct work_struct enc_complete_work;

	/*
	 * Set true by vidc_enc_send_seq_header() just before issuing the
	 * SEQ_HEADER command.  In recovery-mode boots the firmware acks
	 * SEQ_HEADER with an EMPTY IRQ (cmd=0) rather than RESP_SEQ_DONE;
	 * the ISR checks this flag on EMPTY to trigger seq_done_work and
	 * unblock the caller.  Cleared by the ISR (recovery path) or by
	 * vidc_enc_seq_done_work (normal path) or on timeout.
	 */
	bool seq_header_pending;
};

/* Core functions */
void vidc_core_deinit(struct vidc_core *core);
int vidc_load_firmware(struct vidc_core *core);
void vidc_unload_firmware(struct vidc_core *core);
int vidc_boot_firmware(struct vidc_core *core);

/* Channel management */
int vidc_open_channel(struct vidc_inst *inst);
int vidc_close_channel(struct vidc_inst *inst);
int vidc_init_buffers(struct vidc_inst *inst);
int vidc_enc_send_seq_header(struct vidc_inst *inst);
int vidc_init_enc_buffers(struct vidc_inst *inst);
void vidc_free_buffers(struct vidc_inst *inst);
int vidc_apply_dec_codec_config(struct vidc_inst *inst);
int vidc_apply_enc_codec_config(struct vidc_inst *inst);
int vidc_flush_channel(struct vidc_inst *inst, u32 flush_type);
int vidc_copy_dpb_to_dst(struct vidc_inst *inst, void *dst_vaddr,
			 size_t dst_size, size_t *out_payload);

/* Hardware access */
static inline u32 vidc_read(struct vidc_core *core, u32 reg)
{
	return readl(core->base + reg);
}

static inline void vidc_write(struct vidc_core *core, u32 reg, u32 val)
{
	writel(val, core->base + reg);
}

int vidc_hw_reset(struct vidc_core *core, u32 dram_base_addr);
int vidc_send_cmd(struct vidc_core *core, u32 cmd, u32 arg1, u32 arg2,
		  u32 arg3, u32 arg4);
int vidc_get_response(struct vidc_core *core, u32 *cmd, u32 *arg1,
		      u32 *arg2, u32 *arg3, u32 *arg4);

#endif /* __VIDC_CORE_H__ */
