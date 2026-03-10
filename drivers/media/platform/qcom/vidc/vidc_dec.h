/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Qualcomm VIDC 1080p Video Decoder driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Linux-SHR Project
 */

#ifndef __VIDC_DEC_H__
#define __VIDC_DEC_H__

#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#include "vidc_core.h"

/**
 * struct vidc_format - VIDC format description
 * @pixfmt: V4L2 pixel format FourCC
 * @num_planes: Number of planes for this format
 * @type: V4L2 buffer type (capture or output)
 * @codec: VIDC codec type for compressed formats
 * @flags: V4L2 format flags (e.g., V4L2_FMT_FLAG_DYN_RESOLUTION)
 */
struct vidc_format {
	u32 pixfmt;
	u32 num_planes;
	u32 type;
	enum vidc_codec codec;
	u32 flags;
};

/**
 * struct vidc_buffer - VIDC buffer wrapper
 * @vb: V4L2 video buffer base
 * @list: List entry for pending buffers
 * @inst: Pointer to the owning instance
 * @dma_addr: DMA address of the buffer
 * @bytesused: Number of bytes used in the buffer
 */
struct vidc_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	struct vidc_inst *inst;
	dma_addr_t dma_addr;
	size_t bytesused;
};

/**
 * vidc_file_to_inst - Get instance from file handle
 * @file: V4L2 file handle
 *
 * Return: Pointer to the VIDC instance
 */
static inline struct vidc_inst *vidc_file_to_inst(struct file *file)
{
	return container_of(file->private_data, struct vidc_inst, fh);
}

/**
 * to_vidc_buffer - Get VIDC buffer from vb2 buffer
 * @vb: VB2 buffer
 *
 * Return: Pointer to the VIDC buffer wrapper
 */
static inline struct vidc_buffer *to_vidc_buffer(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	return container_of(vbuf, struct vidc_buffer, vb);
}

/* Decoder registration */
int vidc_dec_register(struct vidc_core *core);
void vidc_dec_unregister(struct vidc_core *core);

#endif /* __VIDC_DEC_H__ */
