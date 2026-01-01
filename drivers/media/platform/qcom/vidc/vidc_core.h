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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
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
	bool fw_loaded;
	u32 fw_version;

	/* Instance management */
	struct list_head instances;
	unsigned int num_instances;
};

struct vidc_inst {
	struct vidc_core *core;
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct mutex lock;
	struct list_head list;

	enum vidc_codec codec;
	bool decoder;

	/* Format info */
	u32 width;
	u32 height;
	u32 out_fmt;
	u32 cap_fmt;

	/* Sequence info from firmware */
	u32 seq_width;
	u32 seq_height;
	u32 min_dpb_count;
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
