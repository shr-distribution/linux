// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm VIDC 1080p Video Decoder driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Linux-SHR Project
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vidc_core.h"
#include "vidc_dec.h"

/* Supported decode formats */
static const struct vidc_format vidc_dec_fmts[] = {
	/* Capture formats (decoded output) */
	{
		.pixfmt = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.codec = 0, /* raw format */
	},
	/* Output formats (compressed input) */
	{
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = VIDC_CODEC_H264_DEC,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = VIDC_CODEC_MPEG4_DEC,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.pixfmt = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = VIDC_CODEC_H263_DEC,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = VIDC_CODEC_MPEG2_DEC,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.pixfmt = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = VIDC_CODEC_VC1_DEC,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.pixfmt = V4L2_PIX_FMT_XVID,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = VIDC_CODEC_DIVX503_DEC,
		.flags = V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
};

#define VIDC_DEC_NUM_FMTS ARRAY_SIZE(vidc_dec_fmts)

/* Default format parameters */
#define VIDC_DEFAULT_WIDTH	1280
#define VIDC_DEFAULT_HEIGHT	720
#define VIDC_MIN_WIDTH		96
#define VIDC_MIN_HEIGHT		96
#define VIDC_MAX_WIDTH		1920
#define VIDC_MAX_HEIGHT		1088

static const struct vidc_format *vidc_dec_find_format(u32 pixfmt, u32 type)
{
	unsigned int i;

	for (i = 0; i < VIDC_DEC_NUM_FMTS; i++) {
		if (vidc_dec_fmts[i].pixfmt == pixfmt &&
		    vidc_dec_fmts[i].type == type)
			return &vidc_dec_fmts[i];
	}

	return NULL;
}

static const struct vidc_format *vidc_dec_find_format_by_index(unsigned int index,
							       u32 type)
{
	unsigned int i, k = 0;

	for (i = 0; i < VIDC_DEC_NUM_FMTS; i++) {
		if (vidc_dec_fmts[i].type != type)
			continue;
		if (k == index)
			return &vidc_dec_fmts[i];
		k++;
	}

	return NULL;
}

static u32 vidc_dec_get_framesize(u32 pixfmt, u32 width, u32 height)
{
	u32 y_stride, uv_stride, y_plane, uv_plane;

	switch (pixfmt) {
	case V4L2_PIX_FMT_NV12:
		y_stride = ALIGN(width, 128);
		uv_stride = y_stride;
		y_plane = y_stride * ALIGN(height, 32);
		uv_plane = uv_stride * ALIGN(height / 2, 32);
		return y_plane + uv_plane;
	default:
		/* Compressed formats - estimate based on resolution */
		return (width * height * 3) / 2;
	}
}

/* V4L2 ioctl operations */

static int vidc_dec_querycap(struct file *file, void *fh,
			     struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-vidc", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm VIDC 1080p Decoder", sizeof(cap->card));

	return 0;
}

static int vidc_dec_enum_fmt(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	const struct vidc_format *fmt;

	fmt = vidc_dec_find_format_by_index(f->index, f->type);
	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->pixfmt;
	f->flags = fmt->flags;

	return 0;
}

static int vidc_dec_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	const struct vidc_format *fmt;
	u32 szimage;

	fmt = vidc_dec_find_format(pixmp->pixelformat, f->type);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_NV12;
		else
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
		fmt = vidc_dec_find_format(pixmp->pixelformat, f->type);
		if (!fmt)
			return -EINVAL;
	}

	pixmp->width = clamp(pixmp->width, (u32)VIDC_MIN_WIDTH,
			     (u32)VIDC_MAX_WIDTH);
	pixmp->height = clamp(pixmp->height, (u32)VIDC_MIN_HEIGHT,
			      (u32)VIDC_MAX_HEIGHT);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		pixmp->height = ALIGN(pixmp->height, 32);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	szimage = vidc_dec_get_framesize(pixmp->pixelformat,
					 pixmp->width, pixmp->height);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->plane_fmt[0].sizeimage = szimage;
		pixmp->plane_fmt[0].bytesperline = ALIGN(pixmp->width, 128);
	} else {
		pixmp->plane_fmt[0].sizeimage =
			clamp_t(u32, pixmp->plane_fmt[0].sizeimage, szimage,
				SZ_8M);
		pixmp->plane_fmt[0].bytesperline = 0;
	}

	return 0;
}

static int vidc_dec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	const struct vidc_format *fmt;
	struct vb2_queue *q;
	int ret;

	q = v4l2_m2m_get_vq(inst->m2m_ctx, f->type);
	if (!q)
		return -EINVAL;

	if (vb2_is_busy(q))
		return -EBUSY;

	ret = vidc_dec_try_fmt(file, fh, f);
	if (ret)
		return ret;

	fmt = vidc_dec_find_format(pixmp->pixelformat, f->type);
	if (!fmt)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		inst->fmt_out = fmt;
		inst->codec = fmt->codec;
		inst->out_width = pixmp->width;
		inst->out_height = pixmp->height;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		inst->fmt_cap = fmt;
		inst->width = pixmp->width;
		inst->height = pixmp->height;
	}

	return 0;
}

static int vidc_dec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	const struct vidc_format *fmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt = inst->fmt_cap;
		pixmp->width = inst->width;
		pixmp->height = inst->height;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmt = inst->fmt_out;
		pixmp->width = inst->out_width;
		pixmp->height = inst->out_height;
	} else {
		return -EINVAL;
	}

	if (!fmt)
		return -EINVAL;

	pixmp->pixelformat = fmt->pixfmt;
	pixmp->num_planes = fmt->num_planes;
	pixmp->field = V4L2_FIELD_NONE;

	return vidc_dec_try_fmt(file, fh, f);
}

static int vidc_dec_reqbufs(struct file *file, void *fh,
			    struct v4l2_requestbuffers *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_reqbufs(file, inst->m2m_ctx, b);
}

static int vidc_dec_querybuf(struct file *file, void *fh,
			     struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_querybuf(file, inst->m2m_ctx, b);
}

static int vidc_dec_create_bufs(struct file *file, void *fh,
				struct v4l2_create_buffers *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_create_bufs(file, inst->m2m_ctx, b);
}

static int vidc_dec_prepare_buf(struct file *file, void *fh,
				struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_prepare_buf(file, inst->m2m_ctx, b);
}

static int vidc_dec_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_qbuf(file, inst->m2m_ctx, b);
}

static int vidc_dec_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_dqbuf(file, inst->m2m_ctx, b);
}

static int vidc_dec_expbuf(struct file *file, void *fh,
			   struct v4l2_exportbuffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_expbuf(file, inst->m2m_ctx, b);
}

static int vidc_dec_streamon(struct file *file, void *fh,
			     enum v4l2_buf_type type)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_streamon(file, inst->m2m_ctx, type);
}

static int vidc_dec_streamoff(struct file *file, void *fh,
			      enum v4l2_buf_type type)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_streamoff(file, inst->m2m_ctx, type);
}

static int vidc_dec_subscribe_event(struct v4l2_fh *fh,
				    const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static const struct v4l2_ioctl_ops vidc_dec_ioctl_ops = {
	.vidioc_querycap = vidc_dec_querycap,
	.vidioc_enum_fmt_vid_cap = vidc_dec_enum_fmt,
	.vidioc_enum_fmt_vid_out = vidc_dec_enum_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidc_dec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidc_dec_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidc_dec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidc_dec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidc_dec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidc_dec_g_fmt,
	.vidioc_reqbufs = vidc_dec_reqbufs,
	.vidioc_querybuf = vidc_dec_querybuf,
	.vidioc_create_bufs = vidc_dec_create_bufs,
	.vidioc_prepare_buf = vidc_dec_prepare_buf,
	.vidioc_qbuf = vidc_dec_qbuf,
	.vidioc_dqbuf = vidc_dec_dqbuf,
	.vidioc_expbuf = vidc_dec_expbuf,
	.vidioc_streamon = vidc_dec_streamon,
	.vidioc_streamoff = vidc_dec_streamoff,
	.vidioc_subscribe_event = vidc_dec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* VB2 queue operations */

static int vidc_dec_queue_setup(struct vb2_queue *q,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	u32 width, height, size;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		width = inst->out_width;
		height = inst->out_height;
	} else {
		width = inst->width;
		height = inst->height;
	}

	size = vidc_dec_get_framesize(
		q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ?
		inst->fmt_cap->pixfmt : inst->fmt_out->pixfmt,
		width, height);

	if (*num_planes) {
		if (*num_planes != 1 || sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*num_planes = 1;
	sizes[0] = size;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		*num_buffers = max(*num_buffers, 4U);
	else
		*num_buffers = max(*num_buffers, 8U);

	return 0;
}

static int vidc_dec_buf_init(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vidc_buffer *buf = to_vidc_buffer(vb);

	buf->inst = inst;
	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int vidc_dec_buf_prepare(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	u32 size;

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		size = vidc_dec_get_framesize(inst->fmt_out->pixfmt,
					      inst->out_width, inst->out_height);
	else
		size = vidc_dec_get_framesize(inst->fmt_cap->pixfmt,
					      inst->width, inst->height);

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;

	return 0;
}

static int vidc_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* Start decode session */
		if (!inst->streamon_out) {
			ret = pm_runtime_resume_and_get(core->dev);
			if (ret < 0)
				goto unlock;

			inst->streamon_out = true;
			inst->sequence_out = 0;
		}
	} else {
		if (!inst->streamon_cap) {
			inst->streamon_cap = true;
			inst->sequence_cap = 0;
		}
	}

	mutex_unlock(&inst->lock);
	return 0;

unlock:
	mutex_unlock(&inst->lock);
	return ret;
}

static void vidc_dec_stop_streaming(struct vb2_queue *q)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *vbuf;

	mutex_lock(&inst->lock);

	/* Return all buffers to userspace */
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((vbuf = v4l2_m2m_src_buf_remove(inst->m2m_ctx)))
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		if (inst->streamon_out) {
			inst->streamon_out = false;
			pm_runtime_put(core->dev);
		}
	} else {
		while ((vbuf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx)))
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		inst->streamon_cap = false;
	}

	mutex_unlock(&inst->lock);
}

static void vidc_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);
}

static const struct vb2_ops vidc_dec_vb2_ops = {
	.queue_setup = vidc_dec_queue_setup,
	.buf_init = vidc_dec_buf_init,
	.buf_prepare = vidc_dec_buf_prepare,
	.start_streaming = vidc_dec_start_streaming,
	.stop_streaming = vidc_dec_stop_streaming,
	.buf_queue = vidc_dec_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* M2M operations */

static void vidc_dec_submit_frame(struct vidc_inst *inst,
				  dma_addr_t src_addr, u32 src_size,
				  dma_addr_t dst_addr)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;

	spin_lock_irqsave(&core->irqlock, flags);

	/* Set current instance for IRQ handler */
	core->curr_inst = inst;

	/* Clear previous response */
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);

	/* Write stream buffer address and size (shifted by 11 bits) */
	vidc_write(core, VIDC_REG_CH0_STREAM_ADDR, src_addr >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_STREAM_SIZE, src_size);
	vidc_write(core, VIDC_REG_CH0_STREAM_BUF_SIZE, src_size);

	/* Write output buffer address for decoded frame */
	vidc_write(core, VIDC_REG_CH0_Y_ADDR, dst_addr >> VIDC_ADDR_SHIFT);
	/* Chroma follows luma in NV12 format */
	vidc_write(core, VIDC_REG_CH0_C_ADDR,
		   (dst_addr + ALIGN(inst->width, 128) *
		    ALIGN(inst->height, 32)) >> VIDC_ADDR_SHIFT);

	/* Increment and write command sequence number */
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);

	/* Write channel instance ID with operation type to start decode */
	vidc_write(core, VIDC_REG_CH0_INST_ID,
		   VIDC_OP_FRAME_DATA | inst->inst_id);

	spin_unlock_irqrestore(&core->irqlock, flags);

	dev_dbg(core->dev, "Decode submit: src=0x%pad size=%u dst=0x%pad\n",
		&src_addr, src_size, &dst_addr);
}

static void vidc_dec_device_run(void *priv)
{
	struct vidc_inst *inst = priv;
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t src_addr, dst_addr;
	u32 src_size;

	src_buf = v4l2_m2m_next_src_buf(inst->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(inst->m2m_ctx);

	if (!src_buf || !dst_buf) {
		v4l2_m2m_job_finish(inst->m2m_dev, inst->m2m_ctx);
		return;
	}

	/* Save current buffers for completion handler */
	inst->src_buf = src_buf;
	inst->dst_buf = dst_buf;

	src_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	src_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);

	/* Initialize completion for this frame */
	reinit_completion(&inst->done);
	inst->error = 0;

	/* Submit decode command to hardware */
	vidc_dec_submit_frame(inst, src_addr, src_size, dst_addr);

	/*
	 * Wait for decode completion with timeout.
	 * In a production driver, this would be fully async with
	 * the completion handled in a workqueue.
	 */
	if (!wait_for_completion_timeout(&inst->done,
					 msecs_to_jiffies(1000))) {
		dev_err(core->dev, "Decode timeout\n");
		inst->error = -ETIMEDOUT;
	}

	/* Remove buffers and mark as done */
	src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);

	src_buf->sequence = inst->sequence_out++;
	dst_buf->sequence = inst->sequence_cap++;

	if (inst->error) {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
	} else {
		/* Set output buffer payload from hardware result */
		if (inst->result_size > 0)
			vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
					      inst->result_size);
		else
			vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
					      vidc_dec_get_framesize(
						      inst->fmt_cap->pixfmt,
						      inst->width,
						      inst->height));

		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	}

	/* Clear current instance */
	core->curr_inst = NULL;

	v4l2_m2m_job_finish(inst->m2m_dev, inst->m2m_ctx);
}

static void vidc_dec_job_abort(void *priv)
{
	struct vidc_inst *inst = priv;

	v4l2_m2m_job_finish(inst->m2m_dev, inst->m2m_ctx);
}

static const struct v4l2_m2m_ops vidc_dec_m2m_ops = {
	.device_run = vidc_dec_device_run,
	.job_abort = vidc_dec_job_abort,
};

/* Queue initialization for M2M context */

static int vidc_dec_queue_init(void *priv, struct vb2_queue *src_vq,
			       struct vb2_queue *dst_vq)
{
	struct vidc_inst *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vidc_dec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = inst;
	src_vq->buf_struct_size = sizeof(struct vidc_buffer);
	src_vq->allow_zero_bytesused = 1;
	src_vq->min_queued_buffers = 1;
	src_vq->dev = inst->core->dev;
	src_vq->lock = &inst->lock;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vidc_dec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = inst;
	dst_vq->buf_struct_size = sizeof(struct vidc_buffer);
	dst_vq->allow_zero_bytesused = 1;
	dst_vq->min_queued_buffers = 1;
	dst_vq->dev = inst->core->dev;
	dst_vq->lock = &inst->lock;

	return vb2_queue_init(dst_vq);
}

/* File operations */

static int vidc_dec_open(struct file *file)
{
	struct vidc_core *core = video_drvdata(file);
	struct vidc_inst *inst;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->core = core;
	inst->decoder = true;
	mutex_init(&inst->lock);
	INIT_LIST_HEAD(&inst->list);
	init_completion(&inst->done);

	/* Set default formats */
	inst->fmt_out = vidc_dec_find_format(V4L2_PIX_FMT_H264,
					     V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	inst->fmt_cap = vidc_dec_find_format(V4L2_PIX_FMT_NV12,
					     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	inst->codec = VIDC_CODEC_H264_DEC;
	inst->width = VIDC_DEFAULT_WIDTH;
	inst->height = VIDC_DEFAULT_HEIGHT;
	inst->out_width = VIDC_DEFAULT_WIDTH;
	inst->out_height = VIDC_DEFAULT_HEIGHT;

	/* Initialize M2M device */
	inst->m2m_dev = v4l2_m2m_init(&vidc_dec_m2m_ops);
	if (IS_ERR(inst->m2m_dev)) {
		ret = PTR_ERR(inst->m2m_dev);
		goto err_free;
	}

	inst->m2m_ctx = v4l2_m2m_ctx_init(inst->m2m_dev, inst,
					  vidc_dec_queue_init);
	if (IS_ERR(inst->m2m_ctx)) {
		ret = PTR_ERR(inst->m2m_ctx);
		goto err_m2m_release;
	}

	v4l2_fh_init(&inst->fh, core->vfd_dec);
	inst->fh.ctrl_handler = &inst->ctrl_handler;
	file->private_data = &inst->fh;
	v4l2_fh_add(&inst->fh);

	/* Add to instance list */
	mutex_lock(&core->lock);
	list_add_tail(&inst->list, &core->instances);
	core->num_instances++;
	mutex_unlock(&core->lock);

	return 0;

err_m2m_release:
	v4l2_m2m_release(inst->m2m_dev);
err_free:
	kfree(inst);
	return ret;
}

static int vidc_dec_close(struct file *file)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct vidc_core *core = inst->core;

	mutex_lock(&core->lock);
	list_del(&inst->list);
	core->num_instances--;
	mutex_unlock(&core->lock);

	v4l2_fh_del(&inst->fh);
	v4l2_fh_exit(&inst->fh);

	v4l2_m2m_ctx_release(inst->m2m_ctx);
	v4l2_m2m_release(inst->m2m_dev);

	kfree(inst);

	return 0;
}

static const struct v4l2_file_operations vidc_dec_fops = {
	.owner = THIS_MODULE,
	.open = vidc_dec_open,
	.release = vidc_dec_close,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* Video device registration */

int vidc_dec_register(struct vidc_core *core)
{
	struct video_device *vdev;
	int ret;

	vdev = video_device_alloc();
	if (!vdev)
		return -ENOMEM;

	strscpy(vdev->name, "qcom-vidc-dec", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vidc_dec_fops;
	vdev->ioctl_ops = &vidc_dec_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	video_set_drvdata(vdev, core);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		video_device_release(vdev);
		return ret;
	}

	core->vfd_dec = vdev;

	return 0;
}

void vidc_dec_unregister(struct vidc_core *core)
{
	if (core->vfd_dec) {
		video_unregister_device(core->vfd_dec);
		core->vfd_dec = NULL;
	}
}
