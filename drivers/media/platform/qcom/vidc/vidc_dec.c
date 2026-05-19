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
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vidc_core.h"
#include "vidc_dec.h"

/*
 * Supported decode formats.
 *
 * Capture side advertises V4L2_PIX_FMT_NV12MT (64x32 macroblock-tile
 * NV12) rather than plain V4L2_PIX_FMT_NV12. The on-chip RISC writes
 * decoded frames into the DPB in tile format and we memcpy slot-to-
 * dst without detiling - so the bytes in the userspace CAPTURE
 * buffer are tile-NV12 too. Advertising NV12 (linear) would lie about
 * the layout and a naive consumer (gst-plugin, ffmpeg raw save) would
 * display garbage. NV12MT consumers know to either pass tiled data to
 * a tile-aware sink (MDP4 supports composition from tile-NV12 in HW)
 * or run a software detile pass (libyuv has ConvertNV12_TileToNV12).
 */
static const struct vidc_format vidc_dec_fmts[] = {
	/* Capture formats (decoded output) */
	{
		.pixfmt = V4L2_PIX_FMT_NV12MT,
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
	case V4L2_PIX_FMT_NV12MT:
		/*
		 * 64x32 tiled NV12. Same total byte count as linear NV12
		 * with 128-pixel stride alignment: each tile is 64x32 bytes
		 * for luma, and the plane is rounded up to a whole number
		 * of tile rows. Two tiles abreast fill the 128-pixel stride.
		 */
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
			pixmp->pixelformat = V4L2_PIX_FMT_NV12MT;
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
		inst->out_buf_size = pixmp->plane_fmt[0].sizeimage;
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

/*
 * V4L2_DEC_CMD_STOP — userspace signals end-of-stream.
 *
 * The standard stateful-decoder protocol is:
 *   1. Userspace sends V4L2_DEC_CMD_STOP
 *   2. Driver flushes the firmware's input queue + drains remaining
 *      held-back frames (firmware emits DISPLAY_ONLY events for each
 *      still-pending DPB slot, then a DPB_EMPTY event)
 *   3. CAPTURE queue gets one buffer with V4L2_BUF_FLAG_LAST
 *
 * The frame_done_work handler at vidc_dec.c already sets BUF_FLAG_LAST
 * when display_status==DPB_EMPTY, so this command just needs to issue
 * VIDC_CMD_FLUSH and let the existing IRQ→work path emit the LAST
 * marker buffer.
 */
static int vidc_dec_decoder_cmd(struct file *file, void *fh,
				struct v4l2_decoder_cmd *dec_cmd)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	switch (dec_cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		if (!inst->ch_open)
			return 0;
		return vidc_flush_channel(inst, VIDC_FLUSH_ALL);

	case V4L2_DEC_CMD_START:
		/*
		 * Resume after a previous STOP. Channel state is already
		 * fine (CLOSE_CH wasn't issued, just flush); nothing to do
		 * on the firmware side. m2m core will start scheduling
		 * device_run again as buffers get queued.
		 */
		return 0;

	default:
		return -EINVAL;
	}
}

static int vidc_dec_subscribe_event(struct v4l2_fh *fh,
				    const struct v4l2_event_subscription *sub)
{
	struct vidc_inst *inst = container_of(fh, struct vidc_inst, fh);
	int ret;

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		ret = v4l2_src_change_event_subscribe(fh, sub);
		if (!ret && inst->seq_parsed) {
			/*
			 * GStreamer subscribes inside its decoding thread,
			 * which starts AFTER the first OUTPUT buffer is sent.
			 * seq_done_work may have already fired and queued
			 * SOURCE_CHANGE before this subscription was registered,
			 * silently dropping the event.  Re-deliver it now so the
			 * decoding thread does not stall forever.
			 */
			struct v4l2_event ev = {
				.type = V4L2_EVENT_SOURCE_CHANGE,
				.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
			};
			v4l2_event_queue_fh(fh, &ev);
		}
		return ret;
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
	.vidioc_decoder_cmd = vidc_dec_decoder_cmd,
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
	u32 size;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/*
		 * Use the sizeimage negotiated in S_FMT so the VB2 buffer
		 * size matches what GStreamer (and other apps) stored from the
		 * S_FMT response. Recomputing vidc_dec_get_framesize() for
		 * compressed formats returns the raw YUV size (wrong), causing
		 * GStreamer's MMAP allocator to reject the buffer as too small.
		 */
		size = inst->out_buf_size;
	} else {
		size = vidc_dec_get_framesize(inst->fmt_cap->pixfmt,
					      inst->width, inst->height);
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

/*
 * vb2 framework holds vq->lock (== inst->lock, see vidc_dec_queue_init)
 * across start_streaming / stop_streaming / buf_queue / etc. Re-acquiring
 * inst->lock here would self-deadlock — observed as v4l2-ctl blocked for
 * 122+ seconds with "blocked on a mutex likely owned by task v4l2-ctl"
 * during VIDIOC_STREAMON. So don't take the lock; the vb2 wrapper has us
 * covered.
 */
static int vidc_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	int ret;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* Start decode session */
		if (!inst->streamon_out) {
			ret = pm_runtime_resume_and_get(core->dev);
			if (ret < 0)
				return ret;

			/*
			 * Open the firmware channel for this decode instance.
			 * Sends HOST2RISC OPEN_CH with the selected codec and
			 * a per-instance context-memory slot allocated from
			 * the pool adjacent to the firmware. The on-chip RISC
			 * acknowledges with RESP_OPEN_CH; vidc_open_channel()
			 * blocks on the completion for up to 1 s.
			 *
			 * Without this the subsequent CH0_INST_ID writes in
			 * vidc_dec_submit_frame() target a channel the
			 * firmware does not know about and the SEQ_DONE /
			 * FRAME_DONE responses never arrive.
			 */
			ret = vidc_open_channel(inst);
			if (ret) {
				pm_runtime_put(core->dev);
				return ret;
			}

			/*
			 * Codec-specific config that the firmware cannot
			 * auto-derive from the bitstream. For H.264 this is
			 * a no-op; for MPEG-4/H.263/DivX/VC-1 it emits a
			 * warn-once and continues (full implementations
			 * pending).
			 */
			ret = vidc_apply_dec_codec_config(inst);
			if (ret) {
				vidc_close_channel(inst);
				pm_runtime_put(core->dev);
				return ret;
			}

			inst->streamon_out = true;
			inst->sequence_out = 0;

			/*
			 * GStreamer queues the SEQ_HEADER buffer before calling
			 * STREAMON, so buf_queue's streamon_out check was false
			 * at queue time and seq_header_work was never scheduled.
			 * Trigger it now that streaming is active.
			 */
			if (!inst->seq_parsed && !inst->seq_hdr_direct) {
				dev_info(core->dev,
					 "start_streaming: queuing seq_header_work\n");
				queue_work(system_wq, &inst->seq_header_work);
			}
		}
	} else {
		if (!inst->streamon_cap) {
			inst->streamon_cap = true;
			inst->sequence_cap = 0;
		}
	}

	return 0;
}

static void vidc_dec_stop_streaming(struct vb2_queue *q)
{
	struct vidc_inst *inst = vb2_get_drv_priv(q);
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *vbuf;

	/* Return all buffers to userspace */
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((vbuf = v4l2_m2m_src_buf_remove(inst->m2m_ctx)))
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		if (inst->streamon_out) {
			/*
			 * Close the firmware channel before dropping the
			 * runtime-PM reference. Channel close releases the
			 * context-memory slot's per-instance state on the
			 * RISC; if we skipped this and pm_runtime_put()
			 * powered the codec off, the next stream-on for any
			 * instance would see stale firmware state.
			 */
			vidc_close_channel(inst);

			inst->streamon_out = false;
			pm_runtime_put(core->dev);
		}
	} else {
		while ((vbuf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx)))
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		inst->streamon_cap = false;
	}
}

static void vidc_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vidc_inst *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	dev_info(core->dev,
		 "buf_queue: type=%u seq_parsed=%d hdr_direct=%d streamon_out=%d\n",
		 vb->type, inst->seq_parsed, inst->seq_hdr_direct,
		 inst->streamon_out);

	v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);

	/*
	 * Stateful decode: the M2M scheduler only calls device_run when
	 * BOTH queues have buffers, but V4L2_EVENT_SOURCE_CHANGE (which
	 * prompts userspace to configure the CAPTURE queue) only comes
	 * after SEQ_HEADER is processed. Trigger seq_header_work directly
	 * when the first OUTPUT buffer arrives so SEQ_HEADER can fire
	 * without waiting for CAPTURE to be ready.
	 */
	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    !inst->seq_parsed && !inst->seq_hdr_direct && inst->streamon_out)
		queue_work(system_wq, &inst->seq_header_work);
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
	u32 op;

	op = inst->seq_parsed ? VIDC_OP_FRAME_DATA : VIDC_OP_SEQ_HEADER;

	spin_lock_irqsave(&core->irqlock, flags);

	/* Set current instance for IRQ handler */
	core->curr_inst = inst;

	/* Clear previous response */
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);

	/*
	 * INIT_CH: write 0xffff to INST_ID before programming parameters.
	 * The legacy DDL (vidc_1080p_decode_seq_start_ch0) always does this
	 * first to put the channel in a known idle state before writing the
	 * command parameters.  Without it the firmware may read stale values
	 * from a previous command.
	 */
	vidc_write(core, VIDC_REG_CH0_INST_ID, VIDC_INIT_CH_INST_ID);

	/*
	 * Clear DivX3 resolution registers before every command.  For H.264
	 * and all non-DivX3 codecs, webOS ddl_vidc_decode_init_codec calls
	 * vidc_set_divx3_resolution(0, 0) which writes 0 to 0x2050/0x2054
	 * before the SEQ_HEADER command registers.  Without this, stale values
	 * from a previous GDSC power cycle can confuse the firmware.
	 */
	vidc_write(core, VIDC_REG_CH0_Y_ADDR, 0);
	vidc_write(core, VIDC_REG_CH0_C_ADDR, 0);

	/*
	 * All VIDC buffer addresses are firmware-relative: byte offset from
	 * fw_dma_addr (the SMI SRAM base, 0x38000000), shifted right by
	 * VIDC_ADDR_SHIFT.  This mirrors DDL_OFFSET(dram_base_a, buf_phys)
	 * from the legacy vidc_1080p driver.  Writing src_addr directly
	 * (without subtracting fw_dma_addr) pointed the firmware at DDR
	 * garbage and caused a silent firmware hang on every SEQ_HEADER.
	 */
	vidc_write(core, VIDC_REG_CH0_STREAM_ADDR,
		   (src_addr - core->fw_dma_addr) >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_STREAM_SIZE, src_size);

	/* DEBUG: dump first 40 bytes of the stream the firmware will read */
	if (inst->src_buf) {
		const u8 *kva = vb2_plane_vaddr(&inst->src_buf->vb2_buf, 0);
		if (kva)
			print_hex_dump(KERN_INFO, "vidc stream[0:40]: ",
				       DUMP_PREFIX_NONE, 16, 1, kva, 40, false);
	}

	/*
	 * Descriptor buffer: webOS DDL always allocates a 128 KB desc buffer
	 * (DDL_KILO_BYTE(128), unconditionally in ddl_calc_dec_hw_buffers_size)
	 * during the OPEN_CH callback, before SEQ_HEADER is issued.  Both
	 * SEQ_HEADER and FRAME_DATA receive a non-zero desc address pointing
	 * at this buffer.  Pass our global 128 KB SMI SRAM descriptor scratch
	 * buffer for both operations.
	 */
	vidc_write(core, VIDC_REG_CH0_DESC_ADDR,
		   core->desc_offset >> VIDC_ADDR_SHIFT);

	/*
	 * STREAM_BUF_SIZE: buffer capacity for this command.
	 *
	 * webOS DDL sets stream_buffersize = max(client_input_buf_req.sz,
	 * header_len + DDL_LINEAR_BUFFER_ALIGN_BYTES + VCD_SEQ_HDR_PADDING_BYTES)
	 * for SEQ_HEADER (vcd_ddl_vidc.c:227-232).  client_input_buf_req.sz is
	 * the full allocated input buffer, so the result equals vb2_plane_size.
	 * Use vb2_plane_size for both SEQ_HEADER and FRAME_DATA to match webOS.
	 */
	vidc_write(core, VIDC_REG_CH0_STREAM_BUF_SIZE,
		   (u32)vb2_plane_size(&inst->src_buf->vb2_buf, 0));
	vidc_write(core, VIDC_REG_CH0_DESC_BUF_SIZE, VIDC_DESC_BUF_SIZE);

	/*
	 * Metadata start address (SHM+0x0044): webOS DDL always writes a
	 * valid fw-relative RAW byte offset here before SEQ_HEADER and
	 * FRAME_DATA (not the >>11-shifted value used by hw address regs).
	 * The firmware reads version=0x00000101 header entries from this
	 * address; leaving it at zero (or pointing at a zeroed buffer) causes
	 * error 26 (0x1a).  Point it at VIDC_META_INPUT_OFF within the SHM
	 * page, which vidc_load_firmware() initialises with the 8 decoder
	 * entry headers (DATANONE, QPARRAY, CONCEALMB, VC1, SEI, VUI,
	 * PASSTHROUGH, QCOMFILLER) matching ddl_set_default_meta_data_hdr().
	 */
	writel(core->shm_offset + VIDC_META_INPUT_OFF,
	       core->shm_vaddr + VIDC_SHM_EXT_METADATA_START_ADDR);

	vidc_write(core, VIDC_REG_CH0_SHARED_MEM, core->shm_offset);

	/* Increment and write command sequence number */
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);

	/*
	 * FRAME_DATA additionally requires DPB_RELEASE and DPB_CONFIG.
	 * Legacy vidc_1080p_decode_frame_start_ch0 always writes these.
	 * DPB_RELEASE = 0: do not release any DPB slots back to firmware.
	 * DPB_CONFIG  = dpb_count: tell firmware how many DPB slots exist.
	 */
	if (op == VIDC_OP_FRAME_DATA) {
		vidc_write(core, VIDC_REG_CH0_DPB_RELEASE, 0);
		vidc_write(core, VIDC_REG_CH0_DPB_CONFIG, inst->dpb_count);
	}

	dev_info(core->dev,
		 "submit_frame: op=0x%x inst_id=0x%x src_phys=0x%pad fw_rel=0x%x payload=%u buf_sz=%u desc_addr=0x%x meta_addr=0x%x\n",
		 op, inst->inst_id, &src_addr,
		 (u32)((src_addr - core->fw_dma_addr) >> VIDC_ADDR_SHIFT),
		 src_size,
		 (u32)vb2_plane_size(&inst->src_buf->vb2_buf, 0),
		 (u32)(core->desc_offset >> VIDC_ADDR_SHIFT),
		 core->shm_offset + VIDC_META_INPUT_OFF);

	if (op == VIDC_OP_SEQ_HEADER) {
		dev_info(core->dev,
			 "SEQ_HDR SHM[0x38]=0x%08x SHM[0x44]=0x%08x meta_hdr[v/p/t]=0x%08x/0x%08x/0x%08x\n",
			 readl(core->shm_vaddr + VIDC_SHM_METADATA_ENABLE),
			 readl(core->shm_vaddr + VIDC_SHM_EXT_METADATA_START_ADDR),
			 readl(core->shm_vaddr + VIDC_META_INPUT_OFF + 33 * 4),
			 readl(core->shm_vaddr + VIDC_META_INPUT_OFF + 34 * 4),
			 readl(core->shm_vaddr + VIDC_META_INPUT_OFF + 35 * 4));
	}

	/* Trigger: operation type | instance id */
	vidc_write(core, VIDC_REG_CH0_INST_ID, op | inst->inst_id);

	spin_unlock_irqrestore(&core->irqlock, flags);

	dev_dbg(core->dev,
		"Decode submit: op=0x%x src=0x%pad size=%u dst=0x%pad\n",
		op, &src_addr, src_size, &dst_addr);
}

/*
 * Workqueue handler: submit SEQ_HEADER without going through the M2M
 * scheduler. The M2M framework requires both OUTPUT and CAPTURE queues
 * to have buffers before invoking device_run, but the V4L2 stateful
 * decoder protocol requires SEQ_HEADER to be processed with only the
 * OUTPUT buffer present (CAPTURE isn't configured until after
 * V4L2_EVENT_SOURCE_CHANGE). This bypasses that constraint.
 *
 * Sets inst->seq_hdr_direct so that the resulting seq_done_work skips
 * the v4l2_m2m_job_finish call (there is no M2M job to finish).
 * dst=0 is safe for SEQ_HEADER; the firmware only reads the stream
 * buffer and never writes decoded pixels during header parsing.
 */
static void vidc_dec_seq_header_work_fn(struct work_struct *w)
{
	struct vidc_inst *inst =
		container_of(w, struct vidc_inst, seq_header_work);
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *src_buf;
	dma_addr_t src_addr;
	u32 src_size;

	dev_info(core->dev,
		 "seq_header_work: seq_parsed=%d hdr_direct=%d\n",
		 inst->seq_parsed, inst->seq_hdr_direct);

	if (inst->seq_parsed || inst->seq_hdr_direct)
		return;

	src_buf = v4l2_m2m_next_src_buf(inst->m2m_ctx);
	dev_info(core->dev, "seq_header_work: src_buf=%p\n", src_buf);
	if (!src_buf)
		return;

	inst->src_buf = src_buf;
	inst->dst_buf = NULL;

	src_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	src_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);
	if (!src_size)
		src_size = vb2_plane_size(&src_buf->vb2_buf, 0);

	/*
	 * WebOS DDL always passes bare SPS+PPS to the SEQ_HEADER command —
	 * no AUD, no IDR/slice data.  GStreamer's h264parse alignment=au
	 * output prepends an AUD (NAL 9) and appends an IDR slice (NAL 5)
	 * to the first buffer.  Strip both ends:
	 *
	 * 1. AUD removal: VIDC firmware returns error 52 (HEADER_NOT_FOUND)
	 *    when NAL type 9 is the first NAL.  memmove the body to byte 0
	 *    because VIDC_ADDR_SHIFT=11 means the address register has 2048-
	 *    byte granularity — a 6-byte pointer advance is truncated to 0.
	 *
	 * 2. Slice truncation: firmware SEQ_HEADER must not contain slice
	 *    data (IDR=5, non-IDR=1).  SEI (type 6) may follow PPS and is
	 *    kept; only truncate at the first slice NAL after PPS.
	 */
	{
		u8 *kva = vb2_plane_vaddr(&src_buf->vb2_buf, 0);

		if (!kva)
			goto submit;

		print_hex_dump(KERN_INFO, "vidc seq_hdr BEFORE[0:16]: ",
			       DUMP_PREFIX_NONE, 16, 1, kva,
			       min_t(u32, src_size, 16), false);

		/* Step 1: strip leading AUD */
		if (src_size >= 6 &&
		    kva[0] == 0 && kva[1] == 0 && kva[2] == 0 && kva[3] == 1 &&
		    (kva[4] & 0x1f) == 9) {
			u32 skip = 5; /* past 4-byte start code + AUD NAL type */

			/* AUD payload is 1 byte; scan for next start code */
			while (skip + 3 < src_size) {
				if (kva[skip] == 0 && kva[skip + 1] == 0 &&
				    kva[skip + 2] == 0 && kva[skip + 3] == 1)
					break;
				if (kva[skip] == 0 && kva[skip + 1] == 0 &&
				    kva[skip + 2] == 1)
					break;
				skip++;
			}
			dev_info(core->dev,
				 "seq_header_work: memmove past %u-byte AUD; next NAL=0x%02x\n",
				 skip, kva[skip + 4] & 0x1f);
			memmove(kva, kva + skip, src_size - skip);
			src_size -= skip;
			/* src_addr stays at the aligned buffer base */
		}

		/* Step 2: truncate at first NAL after PPS — webOS sends SPS+PPS only */
		{
			u32 pos = 0;
			bool after_pps = false;

			while (pos + 5 <= src_size) {
				if (kva[pos] == 0 && kva[pos + 1] == 0 &&
				    kva[pos + 2] == 0 && kva[pos + 3] == 1) {
					u8 nal_type = kva[pos + 4] & 0x1f;

					if (nal_type == 8) {
						after_pps = true;
					} else if (after_pps) {
						dev_info(core->dev,
							 "seq_header_work: truncated %u bytes at NAL type %u after PPS\n",
							 src_size - pos, nal_type);
						src_size = pos;
						break;
					}
					pos += 5;
				} else {
					pos++;
				}
			}
		}

		print_hex_dump(KERN_INFO, "vidc seq_hdr AFTER[0:40]: ",
			       DUMP_PREFIX_NONE, 16, 1, kva,
			       min_t(u32, src_size, 40), false);

		/*
		 * Flush CPU write-combine/cache stores to DRAM after memmove.
		 * The memmove modified the DMA-coherent buffer in-place.
		 * dma_sync ensures the VIDC DMA engine sees the updated content.
		 */
		dma_sync_single_for_device(core->dev, src_addr,
					   src_size, DMA_TO_DEVICE);
	}
submit:

	inst->seq_hdr_direct = true;
	vidc_dec_submit_frame(inst, src_addr, src_size, 0);
}

/*
 * Workqueue handler for RESP_SEQ_DONE.
 *
 * Runs in process context (system_wq) — safe to call vidc_init_buffers
 * (which blocks on its own wait_for_completion for RESP_INIT_BUFFERS),
 * v4l2_event_queue_fh (which takes the fh event-list spinlock with
 * irqs off), and the vb2 buf_done helpers (which acquire the vb2
 * queue lock).
 *
 * The IRQ already populated inst->seq_width / seq_height /
 * min_dpb_count via vidc_handle_seq_done() before scheduling us, so
 * all we need is the geometry update + DPB allocation + event
 * emission.
 */
static void vidc_dec_seq_done_work(struct work_struct *w)
{
	struct vidc_inst *inst =
		container_of(w, struct vidc_inst, seq_done_work);
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};
	int dpb_ret;

	if (inst->seq_width)
		inst->width = inst->seq_width;
	if (inst->seq_height)
		inst->height = inst->seq_height;

	inst->seq_parsed = true;

	dev_info(core->dev,
		 "Sequence parsed: %ux%u, min_dpb=%u — initialising DPB\n",
		 inst->seq_width, inst->seq_height, inst->min_dpb_count);

	dpb_ret = vidc_init_buffers(inst);
	if (dpb_ret)
		dev_err(core->dev,
			"DPB init failed: %d (channel unrecoverable)\n",
			dpb_ret);

	v4l2_event_queue_fh(&inst->fh, &ev);

	src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);

	if (src_buf) {
		src_buf->sequence = inst->sequence_out++;
		v4l2_m2m_buf_done(src_buf,
				  dpb_ret ? VB2_BUF_STATE_ERROR
					  : VB2_BUF_STATE_DONE);
	}

	if (dst_buf) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		v4l2_m2m_buf_done(dst_buf,
				  dpb_ret ? VB2_BUF_STATE_ERROR
					  : VB2_BUF_STATE_DONE);
	}

	core->curr_inst = NULL;

	/*
	 * SEQ_HEADER submitted via seq_header_work bypasses the M2M job
	 * scheduler (seq_hdr_direct=true). There is no M2M job to finish
	 * in that case, so skip the job_finish call that would otherwise
	 * double-finish and confuse the M2M state machine.
	 */
	if (inst->seq_hdr_direct) {
		inst->seq_hdr_direct = false;
	} else {
		v4l2_m2m_job_finish(inst->core->m2m_dev_dec, inst->m2m_ctx);
	}
}

/*
 * Workqueue handler for RESP_FRAME_DONE.
 *
 * The IRQ stashed the displayed DPB slot's fw-relative offset in
 * inst->display_y_raw / display_c_raw via vidc_handle_frame_done().
 * We translate that back to a slot index, memcpy tile-NV12 contents
 * to the userspace CAPTURE buffer, mark buffers DONE, and finish
 * the m2m job so the worker can pick up the next queued pair.
 */
/*
 * Helper: copy from the firmware-indicated DPB slot to dst, mark
 * dst DONE with the resulting payload, and return error code.
 * Caller already removed dst from the m2m queue.
 */
static int vidc_dec_emit_dpb(struct vidc_inst *inst,
			     struct vb2_v4l2_buffer *dst_buf)
{
	struct vidc_core *core = inst->core;
	u8 *dst_vaddr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	size_t dst_size = vb2_plane_size(&dst_buf->vb2_buf, 0);
	size_t payload = 0;
	int copy_ret;

	copy_ret = vidc_copy_dpb_to_dst(inst, dst_vaddr, dst_size, &payload);
	if (copy_ret) {
		dev_err(core->dev, "DPB->dst copy failed: %d\n", copy_ret);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
	} else {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, payload);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	}
	return copy_ret;
}

static void vidc_dec_frame_done_work(struct work_struct *w)
{
	struct vidc_inst *inst =
		container_of(w, struct vidc_inst, frame_done_work);
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;

	/*
	 * Dispatch on display_status captured by vidc_handle_frame_done().
	 *
	 * The mapping between the firmware's response and the V4L2 m2m
	 * buffer pair is:
	 *
	 *   DECODE_AND_DISPLAY — typical low-latency: src consumed, dst
	 *                        receives the decoded frame, both DONE.
	 *
	 *   DECODE_ONLY        — B-frame held back for reorder. src is
	 *                        consumed (input_done fires on legacy
	 *                        msm72k), but dst stays in the m2m queue
	 *                        because the firmware has nothing to put
	 *                        in it yet. A later FRAME_DONE will
	 *                        report DISPLAY_ONLY for the held frame.
	 *
	 *   DISPLAY_ONLY       — the firmware has a decoded-but-pending
	 *                        frame ready to emit, but didn't consume
	 *                        a fresh source this cycle. dst gets the
	 *                        held frame; we leave src in the queue
	 *                        (will be consumed by a future FRAME_DATA).
	 *
	 *   DPB_EMPTY          — EOS drain complete. Mark both buffers
	 *                        DONE; the dst gets payload=0 and the
	 *                        V4L2_BUF_FLAG_LAST flag so userspace
	 *                        knows the stream is finished.
	 *
	 *   NOOP / others      — unexpected; log and treat as
	 *                        DECODE_AND_DISPLAY for safety.
	 */
	switch (inst->display_status) {
	case VIDC_DISPLAY_STATUS_DECODE_ONLY:
		src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		if (src_buf) {
			src_buf->sequence = inst->sequence_out++;
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		}
		break;

	case VIDC_DISPLAY_STATUS_DISPLAY_ONLY:
		dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);
		if (dst_buf) {
			dst_buf->sequence = inst->sequence_cap++;
			vidc_dec_emit_dpb(inst, dst_buf);
		}
		break;

	case VIDC_DISPLAY_STATUS_DPB_EMPTY:
		src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);
		if (src_buf) {
			src_buf->sequence = inst->sequence_out++;
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		}
		if (dst_buf) {
			dst_buf->sequence = inst->sequence_cap++;
			dst_buf->flags |= V4L2_BUF_FLAG_LAST;
			vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		}
		break;

	case VIDC_DISPLAY_STATUS_NOOP:
		dev_warn(core->dev,
			 "frame_done_work: firmware reported NOOP\n");
		fallthrough;

	case VIDC_DISPLAY_STATUS_DECODE_AND_DISPLAY:
	default:
		src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);

		if (!src_buf || !dst_buf) {
			dev_warn(core->dev,
				 "frame_done_work: missing buffer (src=%p dst=%p)\n",
				 src_buf, dst_buf);
			break;
		}

		src_buf->sequence = inst->sequence_out++;
		dst_buf->sequence = inst->sequence_cap++;

		if (vidc_dec_emit_dpb(inst, dst_buf))
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		else
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		break;
	}

	core->curr_inst = NULL;
	v4l2_m2m_job_finish(inst->core->m2m_dev_dec, inst->m2m_ctx);
}

/*
 * V4L2 m2m device_run callback.
 *
 * Kicks the hardware and returns immediately. Post-frame work
 * (buffer DONE marking, payload copy, V4L2_EVENT_SOURCE_CHANGE
 * emission, m2m_job_finish) runs from vidc_dec_seq_done_work or
 * vidc_dec_frame_done_work, scheduled by the IRQ handler when
 * the corresponding response arrives from the firmware.
 *
 * Buffers are NOT removed from the m2m queues here — that happens
 * in the work handlers. This means v4l2_m2m_next_*_buf still
 * returns them for inspection, but a second device_run call will
 * not happen until job_finish runs (the m2m core serialises us).
 */
static void vidc_dec_device_run(void *priv)
{
	struct vidc_inst *inst = priv;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t src_addr, dst_addr;
	u32 src_size;

	src_buf = v4l2_m2m_next_src_buf(inst->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(inst->m2m_ctx);

	if (!src_buf || !dst_buf) {
		dev_info(inst->core->dev,
			 "device_run: no buffers (src=%p dst=%p), aborting\n",
			 src_buf, dst_buf);
		v4l2_m2m_job_finish(inst->core->m2m_dev_dec, inst->m2m_ctx);
		return;
	}

	/* Cache for the work handlers (race-free: m2m serialises runs) */
	inst->src_buf = src_buf;
	inst->dst_buf = dst_buf;

	src_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	src_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);

	dev_info(inst->core->dev,
		 "device_run: FRAME_DATA src=0x%pad size=%u seq_parsed=%d\n",
		 &src_addr, src_size, inst->seq_parsed);
	inst->error = 0;
	vidc_dec_submit_frame(inst, src_addr, src_size, dst_addr);

	/*
	 * Return without waiting. The IRQ will fire RESP_SEQ_DONE or
	 * RESP_FRAME_DONE and queue seq_done_work / frame_done_work,
	 * which calls v4l2_m2m_job_finish to release the m2m worker
	 * for the next pair.
	 */
}

static void vidc_dec_job_abort(void *priv)
{
	struct vidc_inst *inst = priv;

	v4l2_m2m_job_finish(inst->core->m2m_dev_dec, inst->m2m_ctx);
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
	INIT_WORK(&inst->seq_done_work, vidc_dec_seq_done_work);
	INIT_WORK(&inst->frame_done_work, vidc_dec_frame_done_work);
	INIT_WORK(&inst->seq_header_work, vidc_dec_seq_header_work_fn);

	/* Set default formats */
	inst->fmt_out = vidc_dec_find_format(V4L2_PIX_FMT_H264,
					     V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	inst->fmt_cap = vidc_dec_find_format(V4L2_PIX_FMT_NV12MT,
					     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	inst->codec = VIDC_CODEC_H264_DEC;
	inst->width = VIDC_DEFAULT_WIDTH;
	inst->height = VIDC_DEFAULT_HEIGHT;
	inst->out_width = VIDC_DEFAULT_WIDTH;
	inst->out_height = VIDC_DEFAULT_HEIGHT;

	/* Initialize M2M context (uses shared m2m_dev from core) */
	inst->m2m_ctx = v4l2_m2m_ctx_init(core->m2m_dev_dec, inst,
					  vidc_dec_queue_init);
	if (IS_ERR(inst->m2m_ctx)) {
		ret = PTR_ERR(inst->m2m_ctx);
		goto err_free;
	}

	ret = v4l2_ctrl_handler_init(&inst->ctrl_handler, 0);
	if (ret)
		goto err_m2m_ctx_release;

	v4l2_fh_init(&inst->fh, core->vfd_dec);
	inst->fh.ctrl_handler = &inst->ctrl_handler;
	inst->fh.m2m_ctx = inst->m2m_ctx;	/* required by v4l2_m2m_fop_mmap */
	file->private_data = &inst->fh;
	v4l2_fh_add(&inst->fh, file);

	/* Add to instance list */
	mutex_lock(&core->lock);
	list_add_tail(&inst->list, &core->instances);
	core->num_instances++;
	mutex_unlock(&core->lock);

	return 0;

err_m2m_ctx_release:
	v4l2_m2m_ctx_release(inst->m2m_ctx);
err_free:
	kfree(inst);
	return ret;
}

static int vidc_dec_close(struct file *file)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);
	struct vidc_core *core = inst->core;

	/*
	 * Flush any pending async work before tearing down state.
	 * The m2m_ctx release below will reject buffer ops, so if a
	 * frame-done IRQ raced our close it must finish (and call
	 * v4l2_m2m_job_finish) before we free inst.
	 */
	cancel_work_sync(&inst->seq_header_work);
	cancel_work_sync(&inst->seq_done_work);
	cancel_work_sync(&inst->frame_done_work);

	mutex_lock(&core->lock);
	list_del(&inst->list);
	core->num_instances--;
	mutex_unlock(&core->lock);

	v4l2_fh_del(&inst->fh, file);
	v4l2_fh_exit(&inst->fh);

	v4l2_ctrl_handler_free(&inst->ctrl_handler);

	v4l2_m2m_ctx_release(inst->m2m_ctx);
	/* Don't release m2m_dev — it's shared and owned by vidc_core */

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

	/* Initialize shared M2M device for decoder instances */
	core->m2m_dev_dec = v4l2_m2m_init(&vidc_dec_m2m_ops);
	if (IS_ERR(core->m2m_dev_dec))
		return PTR_ERR(core->m2m_dev_dec);

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_m2m_release;
	}

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
		goto err_m2m_release;
	}

	core->vfd_dec = vdev;

	return 0;

err_m2m_release:
	v4l2_m2m_release(core->m2m_dev_dec);
	core->m2m_dev_dec = NULL;
	return ret;
}

void vidc_dec_unregister(struct vidc_core *core)
{
	if (core->vfd_dec) {
		video_unregister_device(core->vfd_dec);
		core->vfd_dec = NULL;
	}
	if (core->m2m_dev_dec) {
		v4l2_m2m_release(core->m2m_dev_dec);
		core->m2m_dev_dec = NULL;
	}
}
