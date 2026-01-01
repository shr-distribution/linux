// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm VIDC 1080p Video Encoder driver
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
#include "vidc_enc.h"

/* Supported encode formats */
static const struct vidc_format vidc_enc_fmts[] = {
	/* Output formats (raw input) */
	{
		.pixfmt = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.codec = 0, /* raw format */
	},
	/* Capture formats (compressed output) */
	{
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.codec = VIDC_CODEC_H264_ENC,
	},
	{
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.codec = VIDC_CODEC_MPEG4_ENC,
	},
	{
		.pixfmt = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.codec = VIDC_CODEC_H263_ENC,
	},
};

#define VIDC_ENC_NUM_FMTS ARRAY_SIZE(vidc_enc_fmts)

/* Default format parameters */
#define VIDC_DEFAULT_WIDTH	1280
#define VIDC_DEFAULT_HEIGHT	720
#define VIDC_MIN_WIDTH		96
#define VIDC_MIN_HEIGHT		96
#define VIDC_MAX_WIDTH		1920
#define VIDC_MAX_HEIGHT		1088

/* Default bitrate in bits per second */
#define VIDC_DEFAULT_BITRATE	(2 * 1024 * 1024)
#define VIDC_MIN_BITRATE	(32 * 1024)
#define VIDC_MAX_BITRATE	(20 * 1024 * 1024)

/* Default framerate */
#define VIDC_DEFAULT_FRAMERATE	30

static const struct vidc_format *vidc_enc_find_format(u32 pixfmt, u32 type)
{
	unsigned int i;

	for (i = 0; i < VIDC_ENC_NUM_FMTS; i++) {
		if (vidc_enc_fmts[i].pixfmt == pixfmt &&
		    vidc_enc_fmts[i].type == type)
			return &vidc_enc_fmts[i];
	}

	return NULL;
}

static const struct vidc_format *vidc_enc_find_format_by_index(unsigned int index,
							       u32 type)
{
	unsigned int i, k = 0;

	for (i = 0; i < VIDC_ENC_NUM_FMTS; i++) {
		if (vidc_enc_fmts[i].type != type)
			continue;
		if (k == index)
			return &vidc_enc_fmts[i];
		k++;
	}

	return NULL;
}

static u32 vidc_enc_get_framesize_raw(u32 width, u32 height)
{
	u32 y_stride, uv_stride, y_plane, uv_plane;

	/* NV12 format */
	y_stride = ALIGN(width, 128);
	uv_stride = y_stride;
	y_plane = y_stride * ALIGN(height, 32);
	uv_plane = uv_stride * ALIGN(height / 2, 32);

	return y_plane + uv_plane;
}

static u32 vidc_enc_get_framesize_compressed(u32 width, u32 height)
{
	/*
	 * Compressed buffer size estimate.
	 * Use worst case of 2 bits per pixel for high quality encoding.
	 */
	return max_t(u32, (width * height * 2) / 8, SZ_512K);
}

/* V4L2 ioctl operations */

static int vidc_enc_querycap(struct file *file, void *fh,
			     struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-vidc", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm VIDC 1080p Encoder", sizeof(cap->card));

	return 0;
}

static int vidc_enc_enum_fmt(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	const struct vidc_format *fmt;

	fmt = vidc_enc_find_format_by_index(f->index, f->type);
	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->pixfmt;
	f->flags = fmt->flags;

	return 0;
}

static int vidc_enc_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	const struct vidc_format *fmt;
	u32 szimage;

	fmt = vidc_enc_find_format(pixmp->pixelformat, f->type);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_NV12;
		else
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
		fmt = vidc_enc_find_format(pixmp->pixelformat, f->type);
		if (!fmt)
			return -EINVAL;
	}

	pixmp->width = clamp(pixmp->width, (u32)VIDC_MIN_WIDTH,
			     (u32)VIDC_MAX_WIDTH);
	pixmp->height = clamp(pixmp->height, (u32)VIDC_MIN_HEIGHT,
			      (u32)VIDC_MAX_HEIGHT);

	/* Align dimensions for hardware requirements */
	pixmp->width = ALIGN(pixmp->width, 16);
	pixmp->height = ALIGN(pixmp->height, 16);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* Raw input */
		szimage = vidc_enc_get_framesize_raw(pixmp->width,
						     pixmp->height);
		pixmp->plane_fmt[0].sizeimage = szimage;
		pixmp->plane_fmt[0].bytesperline = ALIGN(pixmp->width, 128);
		pixmp->colorspace = V4L2_COLORSPACE_REC709;
	} else {
		/* Compressed output */
		szimage = vidc_enc_get_framesize_compressed(pixmp->width,
							    pixmp->height);
		pixmp->plane_fmt[0].sizeimage = szimage;
		pixmp->plane_fmt[0].bytesperline = 0;
	}

	return 0;
}

static int vidc_enc_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
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

	ret = vidc_enc_try_fmt(file, fh, f);
	if (ret)
		return ret;

	fmt = vidc_enc_find_format(pixmp->pixelformat, f->type);
	if (!fmt)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		inst->fmt_out = fmt;
		inst->out_width = pixmp->width;
		inst->out_height = pixmp->height;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		inst->fmt_cap = fmt;
		inst->codec = fmt->codec;
		inst->width = pixmp->width;
		inst->height = pixmp->height;
	}

	return 0;
}

static int vidc_enc_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
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

	return vidc_enc_try_fmt(file, fh, f);
}

static int vidc_enc_g_parm(struct file *file, void *fh,
			   struct v4l2_streamparm *parm)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct v4l2_fract *timeperframe;

	if (parm->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		timeperframe = &parm->parm.output.timeperframe;
		parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	} else if (parm->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		timeperframe = &parm->parm.capture.timeperframe;
		parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	} else {
		return -EINVAL;
	}

	timeperframe->numerator = 1;
	timeperframe->denominator = inst->framerate ?: VIDC_DEFAULT_FRAMERATE;

	return 0;
}

static int vidc_enc_s_parm(struct file *file, void *fh,
			   struct v4l2_streamparm *parm)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct v4l2_fract *timeperframe;
	u32 fps;

	if (parm->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		timeperframe = &parm->parm.output.timeperframe;
	else if (parm->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		timeperframe = &parm->parm.capture.timeperframe;
	else
		return -EINVAL;

	if (timeperframe->numerator == 0 || timeperframe->denominator == 0) {
		timeperframe->numerator = 1;
		timeperframe->denominator = VIDC_DEFAULT_FRAMERATE;
	}

	fps = timeperframe->denominator / timeperframe->numerator;
	fps = clamp(fps, 1U, 120U);
	inst->framerate = fps;

	timeperframe->numerator = 1;
	timeperframe->denominator = fps;

	return 0;
}

static int vidc_enc_reqbufs(struct file *file, void *fh,
			    struct v4l2_requestbuffers *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_reqbufs(file, inst->m2m_ctx, b);
}

static int vidc_enc_querybuf(struct file *file, void *fh,
			     struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_querybuf(file, inst->m2m_ctx, b);
}

static int vidc_enc_create_bufs(struct file *file, void *fh,
				struct v4l2_create_buffers *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_create_bufs(file, inst->m2m_ctx, b);
}

static int vidc_enc_prepare_buf(struct file *file, void *fh,
				struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_prepare_buf(file, inst->m2m_ctx, b);
}

static int vidc_enc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_qbuf(file, inst->m2m_ctx, b);
}

static int vidc_enc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_dqbuf(file, inst->m2m_ctx, b);
}

static int vidc_enc_expbuf(struct file *file, void *fh,
			   struct v4l2_exportbuffer *b)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_expbuf(file, inst->m2m_ctx, b);
}

static int vidc_enc_streamon(struct file *file, void *fh,
			     enum v4l2_buf_type type)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_streamon(file, inst->m2m_ctx, type);
}

static int vidc_enc_streamoff(struct file *file, void *fh,
			      enum v4l2_buf_type type)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	return v4l2_m2m_streamoff(file, inst->m2m_ctx, type);
}

static int vidc_enc_subscribe_event(struct v4l2_fh *fh,
				    const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static int vidc_enc_encoder_cmd(struct file *file, void *fh,
				struct v4l2_encoder_cmd *cmd)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct v4l2_event ev = { .type = V4L2_EVENT_EOS };

	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
		/* Signal EOS to userspace */
		v4l2_event_queue_fh(&inst->fh, &ev);
		break;
	case V4L2_ENC_CMD_START:
		/* Nothing special needed */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vidc_enc_try_encoder_cmd(struct file *file, void *fh,
				    struct v4l2_encoder_cmd *cmd)
{
	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
	case V4L2_ENC_CMD_START:
		cmd->flags = 0;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops vidc_enc_ioctl_ops = {
	.vidioc_querycap = vidc_enc_querycap,
	.vidioc_enum_fmt_vid_cap = vidc_enc_enum_fmt,
	.vidioc_enum_fmt_vid_out = vidc_enc_enum_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidc_enc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidc_enc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidc_enc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidc_enc_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidc_enc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidc_enc_g_fmt,
	.vidioc_g_parm = vidc_enc_g_parm,
	.vidioc_s_parm = vidc_enc_s_parm,
	.vidioc_reqbufs = vidc_enc_reqbufs,
	.vidioc_querybuf = vidc_enc_querybuf,
	.vidioc_create_bufs = vidc_enc_create_bufs,
	.vidioc_prepare_buf = vidc_enc_prepare_buf,
	.vidioc_qbuf = vidc_enc_qbuf,
	.vidioc_dqbuf = vidc_enc_dqbuf,
	.vidioc_expbuf = vidc_enc_expbuf,
	.vidioc_streamon = vidc_enc_streamon,
	.vidioc_streamoff = vidc_enc_streamoff,
	.vidioc_subscribe_event = vidc_enc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_encoder_cmd = vidc_enc_try_encoder_cmd,
	.vidioc_encoder_cmd = vidc_enc_encoder_cmd,
};

/* VB2 queue operations */

static int vidc_enc_queue_setup(struct vb2_queue *q,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	u32 size;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* Raw input */
		size = vidc_enc_get_framesize_raw(inst->out_width,
						  inst->out_height);
	} else {
		/* Compressed output */
		size = vidc_enc_get_framesize_compressed(inst->width,
							 inst->height);
	}

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
		*num_buffers = max(*num_buffers, 4U);

	return 0;
}

static int vidc_enc_buf_init(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vidc_buffer *buf = to_vidc_buffer(vb);

	buf->inst = inst;
	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int vidc_enc_buf_prepare(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	u32 size;

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		size = vidc_enc_get_framesize_raw(inst->out_width,
						  inst->out_height);
	else
		size = vidc_enc_get_framesize_compressed(inst->width,
							 inst->height);

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;

	return 0;
}

static int vidc_enc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	int ret;

	mutex_lock(&inst->lock);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* Start encode session */
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

static void vidc_enc_stop_streaming(struct vb2_queue *q)
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

static void vidc_enc_buf_queue(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);
}

static const struct vb2_ops vidc_enc_vb2_ops = {
	.queue_setup = vidc_enc_queue_setup,
	.buf_init = vidc_enc_buf_init,
	.buf_prepare = vidc_enc_buf_prepare,
	.start_streaming = vidc_enc_start_streaming,
	.stop_streaming = vidc_enc_stop_streaming,
	.buf_queue = vidc_enc_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* M2M operations */

static void vidc_enc_submit_frame(struct vidc_inst *inst,
				  dma_addr_t src_addr, u32 src_size,
				  dma_addr_t dst_addr, u32 dst_size)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	u32 y_stride, y_size;

	spin_lock_irqsave(&core->irqlock, flags);

	/* Set current instance for IRQ handler */
	core->curr_inst = inst;

	/* Clear previous response */
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);

	/*
	 * Set encode configuration registers.
	 * These would normally be set during session open, but we
	 * ensure they're correct here.
	 */
	vidc_write(core, VIDC_REG_ENC_FRAME_WIDTH, inst->out_width);
	vidc_write(core, VIDC_REG_ENC_FRAME_HEIGHT, inst->out_height);
	vidc_write(core, VIDC_REG_ENC_TARGET_BITRATE, inst->bitrate);
	vidc_write(core, VIDC_REG_ENC_FRAME_RATE, inst->framerate);

	/* Calculate Y plane size for NV12 format */
	y_stride = ALIGN(inst->out_width, 128);
	y_size = y_stride * ALIGN(inst->out_height, 32);

	/* Write raw input frame address (Y and C planes, shifted) */
	vidc_write(core, VIDC_REG_CH0_Y_ADDR, src_addr >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_C_ADDR,
		   (src_addr + y_size) >> VIDC_ADDR_SHIFT);

	/* Write output stream buffer address for compressed bitstream */
	vidc_write(core, VIDC_REG_CH0_STREAM_ADDR, dst_addr >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_STREAM_BUF_SIZE, dst_size);

	/* Increment and write command sequence number */
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);

	/* Write channel instance ID with operation type to start encode */
	vidc_write(core, VIDC_REG_CH0_INST_ID,
		   VIDC_OP_FRAME_DATA | inst->inst_id);

	spin_unlock_irqrestore(&core->irqlock, flags);

	dev_dbg(core->dev, "Encode submit: src=0x%pad (%ux%u) dst=0x%pad\n",
		&src_addr, inst->out_width, inst->out_height, &dst_addr);
}

static void vidc_enc_device_run(void *priv)
{
	struct vidc_inst *inst = priv;
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t src_addr, dst_addr;
	u32 src_size, dst_size;

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
	dst_size = vb2_plane_size(&dst_buf->vb2_buf, 0);

	/* Initialize completion for this frame */
	reinit_completion(&inst->done);
	inst->error = 0;

	/* Submit encode command to hardware */
	vidc_enc_submit_frame(inst, src_addr, src_size, dst_addr, dst_size);

	/*
	 * Wait for encode completion with timeout.
	 * In a production driver, this would be fully async with
	 * the completion handled in a workqueue.
	 */
	if (!wait_for_completion_timeout(&inst->done,
					 msecs_to_jiffies(1000))) {
		dev_err(core->dev, "Encode timeout\n");
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
					      vidc_enc_get_framesize_compressed(
						      inst->width,
						      inst->height));

		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	}

	/* Clear current instance */
	core->curr_inst = NULL;

	v4l2_m2m_job_finish(inst->m2m_dev, inst->m2m_ctx);
}

static void vidc_enc_job_abort(void *priv)
{
	struct vidc_inst *inst = priv;

	v4l2_m2m_job_finish(inst->m2m_dev, inst->m2m_ctx);
}

static const struct v4l2_m2m_ops vidc_enc_m2m_ops = {
	.device_run = vidc_enc_device_run,
	.job_abort = vidc_enc_job_abort,
};

/* Queue initialization for M2M context */

static int vidc_enc_queue_init(void *priv, struct vb2_queue *src_vq,
			       struct vb2_queue *dst_vq)
{
	struct vidc_inst *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vidc_enc_vb2_ops;
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
	dst_vq->ops = &vidc_enc_vb2_ops;
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

static int vidc_enc_open(struct file *file)
{
	struct vidc_core *core = video_drvdata(file);
	struct vidc_inst *inst;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->core = core;
	inst->decoder = false;
	mutex_init(&inst->lock);
	INIT_LIST_HEAD(&inst->list);
	init_completion(&inst->done);

	/* Set default formats */
	inst->fmt_out = vidc_enc_find_format(V4L2_PIX_FMT_NV12,
					     V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	inst->fmt_cap = vidc_enc_find_format(V4L2_PIX_FMT_H264,
					     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	inst->codec = VIDC_CODEC_H264_ENC;
	inst->width = VIDC_DEFAULT_WIDTH;
	inst->height = VIDC_DEFAULT_HEIGHT;
	inst->out_width = VIDC_DEFAULT_WIDTH;
	inst->out_height = VIDC_DEFAULT_HEIGHT;
	inst->framerate = VIDC_DEFAULT_FRAMERATE;
	inst->bitrate = VIDC_DEFAULT_BITRATE;

	/* Initialize M2M device */
	inst->m2m_dev = v4l2_m2m_init(&vidc_enc_m2m_ops);
	if (IS_ERR(inst->m2m_dev)) {
		ret = PTR_ERR(inst->m2m_dev);
		goto err_free;
	}

	inst->m2m_ctx = v4l2_m2m_ctx_init(inst->m2m_dev, inst,
					  vidc_enc_queue_init);
	if (IS_ERR(inst->m2m_ctx)) {
		ret = PTR_ERR(inst->m2m_ctx);
		goto err_m2m_release;
	}

	v4l2_fh_init(&inst->fh, core->vfd_enc);
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

static int vidc_enc_close(struct file *file)
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

static const struct v4l2_file_operations vidc_enc_fops = {
	.owner = THIS_MODULE,
	.open = vidc_enc_open,
	.release = vidc_enc_close,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* Video device registration */

int vidc_enc_register(struct vidc_core *core)
{
	struct video_device *vdev;
	int ret;

	vdev = video_device_alloc();
	if (!vdev)
		return -ENOMEM;

	strscpy(vdev->name, "qcom-vidc-enc", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vidc_enc_fops;
	vdev->ioctl_ops = &vidc_enc_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	video_set_drvdata(vdev, core);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		video_device_release(vdev);
		return ret;
	}

	core->vfd_enc = vdev;

	return 0;
}

void vidc_enc_unregister(struct vidc_core *core)
{
	if (core->vfd_enc) {
		video_unregister_device(core->vfd_enc);
		core->vfd_enc = NULL;
	}
}
