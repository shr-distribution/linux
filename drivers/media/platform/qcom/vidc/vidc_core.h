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
#define VIDC_REG_FW_VERSION		0x0090
#define VIDC_REG_FW_STATUS		0x0094
#define VIDC_REG_DRAM_BASE_A		0x00a0
#define VIDC_REG_DRAM_BASE_B		0x00a4
#define VIDC_REG_AXI_CTRL		0x00b0
#define VIDC_REG_AXI_STATUS		0x00b4
#define VIDC_REG_BURST_CONFIG		0x00bc
#define VIDC_REG_CH0_INST_ID		0x0200
#define VIDC_REG_CH1_INST_ID		0x0204

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

/* Address shift for hardware registers */
#define VIDC_ADDR_SHIFT			11

/* Channel 0 registers (decode/encode instance 0) */
#define VIDC_REG_CH0_STREAM_ADDR	0x0100
#define VIDC_REG_CH0_STREAM_SIZE	0x0104
#define VIDC_REG_CH0_DESC_ADDR		0x0108
#define VIDC_REG_CH0_STREAM_BUF_SIZE	0x010c
#define VIDC_REG_CH0_DESC_BUF_SIZE	0x0110
#define VIDC_REG_CH0_SHARED_MEM		0x0114
#define VIDC_REG_CH0_CMD_SEQ_NUM	0x0118
#define VIDC_REG_CH0_DPB_CONFIG		0x011c
#define VIDC_REG_CH0_DPB_RELEASE	0x0120
#define VIDC_REG_CH0_Y_ADDR		0x0124
#define VIDC_REG_CH0_C_ADDR		0x0128
#define VIDC_REG_CH0_INTRA_FRAME	0x012c

/* Sequence info registers (after SEQ_DONE) */
#define VIDC_REG_SEQ_IMG_SIZE_Y		0x0140
#define VIDC_REG_SEQ_IMG_SIZE_X		0x0144
#define VIDC_REG_SEQ_MIN_DPB		0x0148
#define VIDC_REG_SEQ_FRAME_SIZE		0x014c
#define VIDC_REG_SEQ_DISPLAY_INFO	0x0150

/* Decode result registers (after FRAME_DONE) */
#define VIDC_REG_DEC_DISPLAY_Y		0x0160
#define VIDC_REG_DEC_DISPLAY_C		0x0164
#define VIDC_REG_DEC_DECODE_Y		0x0168
#define VIDC_REG_DEC_DECODE_C		0x016c
#define VIDC_REG_DEC_DISPLAY_STATUS	0x0170
#define VIDC_REG_DEC_DECODE_STATUS	0x0174

/* Encode result registers (after ENC_COMPLETE) */
#define VIDC_REG_ENC_FRAME_SIZE		0x0140
#define VIDC_REG_ENC_PICTURE_COUNT	0x0144
#define VIDC_REG_ENC_WRITE_PTR		0x0148
#define VIDC_REG_ENC_FRAME_TYPE		0x0160
#define VIDC_REG_ENC_LUMA_ADDR		0x0164
#define VIDC_REG_ENC_CHROMA_ADDR	0x014c

/* Encode configuration registers */
#define VIDC_REG_ENC_FRAME_WIDTH	0x0180
#define VIDC_REG_ENC_FRAME_HEIGHT	0x0184
#define VIDC_REG_ENC_PROFILE_LEVEL	0x0188
#define VIDC_REG_ENC_RC_CONFIG		0x018c
#define VIDC_REG_ENC_FRAME_RATE		0x0190
#define VIDC_REG_ENC_TARGET_BITRATE	0x0194
#define VIDC_REG_ENC_REACTION_COEFF	0x0198
#define VIDC_REG_ENC_QP_RANGE		0x019c

/* DPB buffer registers (decode) */
#define VIDC_REG_DPB_LUMA_BASE		0x0300
#define VIDC_REG_DPB_CHROMA_BASE	0x0380
#define VIDC_REG_DPB_MV_BASE		0x0400
#define VIDC_MAX_DPB_BUFFERS		32

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
#define VIDC_AXI_RESET			BIT(1)
#define VIDC_AXI_HALT_ACK_MASK		0x0c

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
	struct icc_path *icc_path;

	/* Firmware */
	const struct firmware *fw;
	dma_addr_t fw_dma_addr;
	void *fw_vaddr;
	size_t fw_size;

	/* V4L2 */
	struct v4l2_device v4l2_dev;
	struct video_device *vfd_dec;
	struct video_device *vfd_enc;
	struct v4l2_m2m_dev *m2m_dev_dec;
	struct v4l2_m2m_dev *m2m_dev_enc;

	/* State */
	struct mutex lock;
	spinlock_t irqlock;
	bool fw_loaded;
	u32 fw_version;

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
	struct v4l2_m2m_dev *m2m_dev;
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

	/* Last frame result */
	u32 result_size;
};

/* Core functions */
int vidc_core_init(struct vidc_core *core);
void vidc_core_deinit(struct vidc_core *core);
int vidc_load_firmware(struct vidc_core *core);
void vidc_unload_firmware(struct vidc_core *core);

/* Hardware access */
static inline u32 vidc_read(struct vidc_core *core, u32 reg)
{
	return readl(core->base + reg);
}

static inline void vidc_write(struct vidc_core *core, u32 reg, u32 val)
{
	writel(val, core->base + reg);
}

int vidc_hw_reset(struct vidc_core *core);
int vidc_send_cmd(struct vidc_core *core, u32 cmd, u32 arg1, u32 arg2,
		  u32 arg3, u32 arg4);
int vidc_get_response(struct vidc_core *core, u32 *cmd, u32 *arg1,
		      u32 *arg2, u32 *arg3, u32 *arg4);

#endif /* __VIDC_CORE_H__ */
