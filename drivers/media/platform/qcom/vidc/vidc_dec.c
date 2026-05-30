// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm VIDC 1080p Video Decoder driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
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
		 * 64x32 tiled NV12. Each tile is 64x32 bytes for luma; the
		 * plane is rounded up to whole tile rows (128-pixel stride,
		 * 32-line height) AND each plane size is finally aligned up to
		 * DDL_TILE_MULTIPLY_FACTOR (8192) — the same formula as
		 * vidc_dpb_calc_sizes()/ddl_get_yuv_buf_size().
		 *
		 * The 8192 alignment is NOT optional: the CAPTURE buffer IS the
		 * DPB slot the firmware decodes into, so it must be exactly the
		 * 8192-aligned y_size + c_size (== vidc_dpb_calc_sizes()).
		 * Omitting the alignment made the buffer too small whenever
		 * stride*height was not already a multiple of 8192 (e.g. 640x480:
		 * 471040 vs 475136), and the firmware would write past it.
		 */
		y_stride = ALIGN(width, 128);
		uv_stride = y_stride;
		y_plane = ALIGN(y_stride * ALIGN(height, 32),
				VIDC_DPB_TILE_MULTIPLY_FACTOR);
		uv_plane = ALIGN(uv_stride * ALIGN(height / 2, 32),
				 VIDC_DPB_TILE_MULTIPLY_FACTOR);
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

static int vidc_dec_enum_framesizes(struct file *file, void *fh,
				    struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;

	/* Same coded-size range for the compressed input and decoded output. */
	switch (fsize->pixel_format) {
	case V4L2_PIX_FMT_H264:
	case V4L2_PIX_FMT_NV12MT:
		break;
	default:
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = VIDC_MIN_WIDTH;
	fsize->stepwise.max_width = VIDC_MAX_WIDTH;
	fsize->stepwise.step_width = 16;	/* H.264 macroblock */
	fsize->stepwise.min_height = VIDC_MIN_HEIGHT;
	fsize->stepwise.max_height = VIDC_MAX_HEIGHT;
	fsize->stepwise.step_height = 16;

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
		if (!inst->ch_open || !inst->seq_parsed ||
		    inst->drain_pending || inst->draining)
			return 0;
		/*
		 * Drain (not flush): mark the drain pending. Frames already
		 * queued on the OUTPUT queue keep flowing as normal FRAME_DATA;
		 * once the OUTPUT queue is empty, device_run issues a single
		 * VIDC_OP_LAST_FRAME and the firmware emits the held reorder
		 * frames as DISPLAY_ONLY responses, ending in DPB_EMPTY, where
		 * the frame-done worker marks the last CAPTURE buffer
		 * V4L2_BUF_FLAG_LAST and queues V4L2_EVENT_EOS.
		 *
		 * Kick the scheduler: if the OUTPUT queue is already empty
		 * there is no in-flight job whose completion would otherwise
		 * trigger the drain.
		 */
		inst->drain_pending = true;
		inst->drain_count = 0;
		v4l2_m2m_try_schedule(inst->m2m_ctx);
		return 0;

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

/*
 * Report the visible (crop) rectangle within the coded CAPTURE frame.
 * The firmware-reported H.264 crop offsets are captured at SEQ_DONE; the
 * coded buffer is tile-aligned and padded, so userspace needs the compose
 * rectangle to present only the valid picture area.
 */
static int vidc_dec_g_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	struct vidc_inst *inst = vidc_file_to_inst(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = inst->crop_left;
		s->r.top = inst->crop_top;
		s->r.width = inst->crop_width ? inst->crop_width : inst->width;
		s->r.height = inst->crop_height ? inst->crop_height :
						  inst->height;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ioctl_ops vidc_dec_ioctl_ops = {
	.vidioc_querycap = vidc_dec_querycap,
	.vidioc_enum_fmt_vid_cap = vidc_dec_enum_fmt,
	.vidioc_enum_fmt_vid_out = vidc_dec_enum_fmt,
	.vidioc_enum_framesizes = vidc_dec_enum_framesizes,
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
	.vidioc_g_selection = vidc_dec_g_selection,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
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

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		*num_buffers = max(*num_buffers, 4U);
	} else {
		/*
		 * CAPTURE buffers ARE the DPB, so we need at least the firmware
		 * minimum plus reorder/display headroom, capped at the DPB
		 * register slots.
		 */
		u32 floor = (inst->min_dpb_count ? inst->min_dpb_count : 4) + 4;

		*num_buffers = clamp(*num_buffers, floor,
				     (u32)VIDC_DPB_REG_SLOTS);
	}

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
				dev_dbg(core->dev,
					"start_streaming: queuing seq_header_work\n");
				queue_work(system_wq, &inst->seq_header_work);
			}
		}
	} else {
		if (!inst->streamon_cap) {
			u32 i, n = 0;

			/*
			 * CAPTURE buffers ARE the DPB. Their bus addresses are
			 * only known now (REQBUF'd/QBUF'd in response to
			 * SOURCE_CHANGE), so record them and program the firmware
			 * DPB + INIT_BUFFERS here (deferred from SEQ_DONE).
			 */
			for (i = 0; i < VIDC_DPB_REG_SLOTS; i++) {
				struct vb2_buffer *vb = vb2_get_buffer(q, i);

				if (!vb)
					break;
				inst->dpb_cap_dma[i] =
					vb2_dma_contig_plane_dma_addr(vb, 0);
				n++;
			}
			if (n < (inst->min_dpb_count ? inst->min_dpb_count : 1)) {
				dev_err(core->dev,
					"CAPTURE count %u < firmware min_dpb %u\n",
					n, inst->min_dpb_count);
				return -EINVAL;
			}
			inst->dpb_count = n;

			ret = vidc_init_buffers(inst);
			if (ret) {
				dev_err(core->dev,
					"DPB init failed: %d\n", ret);
				return ret;
			}

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
	unsigned long flags;

	/*
	 * If the firmware is still decoding a frame, wait for it to retire
	 * BEFORE tearing anything down. device_run() calls v4l2_m2m_job_finish()
	 * immediately after submit_frame(), so the m2m job (and the
	 * v4l2_m2m_cancel_job() in streamoff) completes while the firmware is
	 * still mid-frame on a mid-stream STREAMOFF (seek/abort). Sending
	 * CLOSE_CH then times out — the firmware cannot ack a close while
	 * decoding — and wedges the resident codec: no subsequent OPEN_CH is
	 * acked until reboot (the cmd=51 family; HW cold-reset is unviable).
	 * Letting the in-flight frame finish turns the abnormal close into a
	 * graceful one. curr_inst is still set here, so the FRAME_DONE IRQ can
	 * fire and signal frame_done_compl. Best-effort: if the firmware is
	 * already stuck the wait times out and we close anyway.
	 */
	if (inst->frame_in_flight &&
	    !wait_for_completion_timeout(&inst->frame_done_compl,
					 msecs_to_jiffies(500)))
		dev_warn(core->dev,
			 "stop_streaming: in-flight frame did not retire in 500ms; closing anyway\n");
	inst->frame_in_flight = false;

	/*
	 * Quiesce async completion work BEFORE returning buffers or closing
	 * the channel.  A FRAME_DONE that raced streamoff would otherwise run
	 * frame_done_work against buffers we're about to return to userspace,
	 * or — worse — call vidc_dec_emit_dpb() reading inst->dpb_y_vaddr
	 * while vidc_close_channel()->vidc_free_buffers() frees that DPB pool
	 * (use-after-free; the frame_done_work oops + multimedia-IOMMU fault
	 * storm seen on aborted playback).
	 *
	 * Clear curr_inst under irqlock so the IRQ handler (which reads
	 * curr_inst and queues work under the same lock) stops dispatching
	 * into this instance, then drain any already-queued work.
	 * frame_done_work / seq_done_work take only vb2/m2m spinlocks, not
	 * inst->lock, so cancel_work_sync() here cannot deadlock against the
	 * queue lock held across stop_streaming.
	 */
	spin_lock_irqsave(&core->irqlock, flags);
	if (core->curr_inst == inst)
		core->curr_inst = NULL;
	spin_unlock_irqrestore(&core->irqlock, flags);

	cancel_work_sync(&inst->frame_done_work);
	cancel_work_sync(&inst->seq_done_work);

	/* Clear any in-progress drain so a stream restart starts clean. */
	inst->draining = false;
	inst->drain_pending = false;

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
			/*
			 * Drop this session's PM ref. After the first boot the
			 * driver holds a permanent keep-resident pin (see
			 * vidc_boot_firmware / fw_pinned), so this never brings the
			 * usage count to zero and the device stays runtime-active:
			 * VED is never power-collapsed and the firmware is never
			 * re-booted. (Forcing a collapse here — tried via
			 * put_sync — re-triggers the broken genpd cold-reset and
			 * boots cmd=51 recovery from the 2nd session on.)
			 */
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

	dev_dbg(core->dev,
		"buf_queue: type=%u seq_parsed=%d hdr_direct=%d streamon_out=%d\n",
		vb->type, inst->seq_parsed, inst->seq_hdr_direct,
		inst->streamon_out);

	v4l2_m2m_buf_queue(inst->m2m_ctx, vbuf);

	/*
	 * CAPTURE buffer QBUF'd: release this DPB slot to the firmware. The
	 * next FRAME_DATA's CH0_DPB_RELEASE = dpb_hw_mask write hands it back
	 * (webOS ddl MARK_FREE). This MUST cover both the initial population
	 * AND re-queues — the firmware may only decode into slots whose buffer
	 * is currently in the m2m CAPTURE queue, otherwise a FRAME_DONE names a
	 * slot we cannot dequeue. So set the bit on every CAPTURE QBUF
	 * regardless of dpb_inited (the slot count is fixed at REQBUFS); the
	 * INIT_BUFFERS path no longer pre-fills the mask.
	 */
	if (vb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    vb->index < VIDC_DPB_REG_SLOTS)
		inst->dpb_hw_mask |= (1u << vb->index);

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
};

/* M2M operations */

static void vidc_dec_submit_frame(struct vidc_inst *inst,
				  dma_addr_t src_addr, u32 src_size,
				  dma_addr_t dst_addr)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	u32 op;

	spin_lock_irqsave(&core->irqlock, flags);

	/*
	 * Teardown race: device_run picks this frame on the m2m worker thread,
	 * but stop_streaming / vidc_close_channel may close the channel on the
	 * STREAMOFF (or fd-release) thread before we get here. Submitting to a
	 * closed channel faults the firmware — and worst of all, !seq_parsed
	 * (also cleared by close) would now select the SEQ_HEADER opcode, so a
	 * stale frame lands as a bogus sequence-header on a dead channel
	 * (observed firmware error 0x5 + device crash). ch_open is cleared
	 * under this same irqlock at the very start of close, so checking it
	 * here closes the window: bail without touching the firmware.
	 */
	if (!inst->ch_open) {
		spin_unlock_irqrestore(&core->irqlock, flags);
		dev_dbg(core->dev,
			"submit_frame: channel closed, dropping stale submit\n");
		return;
	}

	if (!inst->seq_parsed)
		op = VIDC_OP_SEQ_HEADER;
	else if (inst->draining)
		op = VIDC_OP_LAST_FRAME;	/* drain: flush held reorder frames */
	else
		op = VIDC_OP_FRAME_DATA;

	/*
	 * FRAME_DATA/LAST_FRAME produce a FRAME_DONE the firmware must finish
	 * before the channel can be closed cleanly. Arm the in-flight tracker
	 * (under irqlock, before the trigger write below, so the FRAME_DONE IRQ
	 * — which also takes irqlock — cannot clear it before we set it).
	 * SEQ_HEADER produces SEQ_DONE instead, so leave the tracker alone.
	 */
	if (op == VIDC_OP_FRAME_DATA || op == VIDC_OP_LAST_FRAME) {
		reinit_completion(&inst->frame_done_compl);
		inst->frame_in_flight = true;
	}

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
	 * fw_dma_addr (SMI base 0x38000000), shifted right by VIDC_ADDR_SHIFT.
	 * vb2 buffers come from the SMIPOOL reserved-memory pool at
	 * 0x38300000+ (attached as the device's coherent DMA pool in probe),
	 * so src_addr is within the SMI window the firmware can fetch from.
	 */
	vidc_write(core, VIDC_REG_CH0_STREAM_ADDR,
		   (src_addr - core->fw_dma_addr) >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_STREAM_SIZE, src_size);

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
	 * STREAM_BUF_SIZE: buffer capacity for this command.  webOS DDL uses
	 * max(client_input_buf_req.sz, header_len + 2048 + 256) for SEQ_HEADER
	 * and the full input buffer for FRAME_DATA.  client_input_buf_req.sz
	 * is the full allocated input buffer (decoder default 1 MB), so the
	 * result equals vb2_plane_size for both ops.  Previous earlier scans
	 * of "zero memory beyond the SPS" were a symptom of the firmware not
	 * being able to fetch the DDR-backed vb2 buffer at all; with vb2 now
	 * in SMIPOOL the firmware reads the actual bitstream bytes.
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
	 *
	 * DPB_RELEASE is a bitmask of DPB slots the host is releasing back
	 * to the firmware for writing decoded frames.  Bit N=1 means slot N
	 * is available.  webOS sets this from dpb_mask->hw_mask, which
	 * accumulates bits as the client returns displayed output buffers.
	 * Writing 0 causes error 125 (VIDC_1080P_ERROR_NO_BUFFER_RELEASED_FROM_HOST).
	 *
	 * The DPB slots ARE the V4L2 CAPTURE buffers (zero-copy, s5p-mfc
	 * style).  inst->dpb_hw_mask starts empty and gains a bit per CAPTURE
	 * QBUF (buf_queue), loses it on display (FRAME_DONE), regains it on
	 * re-queue — so it names exactly the buffers currently sitting in the
	 * m2m CAPTURE queue, which is precisely the set the firmware may write.
	 *
	 * DPB_CONFIG = dpb_count: tell firmware how many DPB slots exist.
	 */
	if (op == VIDC_OP_FRAME_DATA || op == VIDC_OP_LAST_FRAME) {
		u32 dpb_config = inst->dpb_count;

		/*
		 * On a pending flush (seek / DECODER_CMD STOP), set the
		 * DPB_FLUSH bit so the firmware drops its reference frames
		 * with this FRAME_DATA. Legacy ORs dpb_flush<<14 the same way.
		 */
		if (inst->flush_pending) {
			dpb_config |= 1u << VIDC_DPB_FLUSH_SHIFT;
			inst->flush_pending = false;
		}
		vidc_write(core, VIDC_REG_CH0_DPB_RELEASE, inst->dpb_hw_mask);
		vidc_write(core, VIDC_REG_CH0_DPB_CONFIG, dpb_config);

		/*
		 * The rest (per-frame frame tag + pixel-cache setup) only
		 * applies to a real input frame. A LAST_FRAME drain carries no
		 * input (zero stream), so skip it — held reorder frames keep
		 * the tags they were given when first decoded.
		 */
		if (op == VIDC_OP_FRAME_DATA) {
		/*
		 * Per-frame shared memory (legacy ddl_vidc_decode_frame_run /
		 * input_done):
		 *  - frame tag: the firmware echoes it on the displayed frame
		 *    (VIDC_SHM_GET_FRAME_TAG_TOP); record the input timestamp
		 *    against the tag so reordered output keeps the right PTS;
		 *  - start_byte_number reset to 0 each frame.
		 */
		{
			u32 tag = inst->frame_tag_next++;

			inst->frame_tag_ts[tag % VIDC_DPB_REG_SLOTS] =
				inst->src_buf->vb2_buf.timestamp;
			writel(tag, core->shm_vaddr + VIDC_SHM_SET_FRAME_TAG);
			writel(0, core->shm_vaddr + VIDC_SHM_SET_START_BYTE_NUMBER);
		}

		/*
		 * Pixel-cache per-frame setup — webOS ddl_vidc_decode_frame_run
		 * (vcd_ddl_vidc.c:840-853).  Decoded macroblocks flow through
		 * the pix cache on the way to DRAM; without telling the cache
		 * where to write (per-slot luma addrs), how big each frame is,
		 * and clearing the tags before each frame, no pixel data
		 * reaches the DPB slots.
		 *
		 * Luma addresses written here are ABSOLUTE physical addresses,
		 * not fw-relative offsets (matches webOS dec_pic_buffers
		 * [i].vcd_frm.physical).
		 */
		{
			u32 fw, slot_count = min_t(u32, inst->dpb_count,
						   VIDC_PIX_CACHE_MAX_DPB);
			u32 frame_size, frame_range;
			u32 i;

			for (i = 0; i < slot_count; i++) {
				/*
				 * Pixel-cache per-slot luma base. The working
				 * decode path leaves this 0 — the firmware uses
				 * the DPB_LUMA[i] registers (programmed from the
				 * CAPTURE buffers) for the actual decode target,
				 * and a non-zero cache base here activates a cache
				 * write path that conflicts with the firmware's
				 * own DPB write and hangs FRAME_DATA. Keep it 0.
				 */
				vidc_write(core,
					   VIDC_REG_PIX_CACHE_LUMA_BASE + i * 4,
					   0);
				/*
				 * webOS passes NULL for chroma_offset which
				 * makes vidc_pix_cache_init_luma_chroma_base_addr
				 * write DPB_RESET_VALUE (0xFFFFFFF8) to every
				 * chroma slot.  The cache derives chroma from
				 * luma_base + frame_range internally.
				 */
				vidc_write(core,
					   VIDC_REG_PIX_CACHE_CHROMA_BASE + i * 4,
					   VIDC_PIX_CACHE_DPB_RESET);
			}
			/* Reset remaining unused slots to DPB_RESET (webOS does this) */
			for (; i < VIDC_PIX_CACHE_MAX_DPB; i++) {
				vidc_write(core,
					   VIDC_REG_PIX_CACHE_LUMA_BASE + i * 4,
					   VIDC_PIX_CACHE_DPB_RESET);
				vidc_write(core,
					   VIDC_REG_PIX_CACHE_CHROMA_BASE + i * 4,
					   VIDC_PIX_CACHE_DPB_RESET);
			}

			frame_size = ((inst->seq_height & 0x7ff) << 16) |
				     (inst->seq_width & 0x7ff);
			vidc_write(core, VIDC_REG_PIX_CACHE_FRAME_SIZE,
				   frame_size);

			frame_range =
				(((inst->dpb_y_size / VIDC_PIX_CACHE_TILE_FACTOR) & 0xff) << 8) |
				((inst->dpb_c_size / VIDC_PIX_CACHE_TILE_FACTOR) & 0xff);
			vidc_write(core, VIDC_REG_PIX_CACHE_FRAME_RANGE,
				   frame_range);

			/* Clear cache tags: set bit 0, then clear it */
			fw = vidc_read(core, VIDC_REG_PIX_CACHE_CONFIG);
			vidc_write(core, VIDC_REG_PIX_CACHE_CONFIG,
				   fw | VIDC_PIX_CACHE_TAG_CLEAR_BIT);
			vidc_write(core, VIDC_REG_PIX_CACHE_CONFIG,
				   fw & ~VIDC_PIX_CACHE_TAG_CLEAR_BIT);
		}
		}	/* end if (op == VIDC_OP_FRAME_DATA) */
	}

	dev_dbg(core->dev,
		"submit_frame: op=0x%x inst_id=0x%x src_phys=0x%pad fw_rel=0x%x payload=%u buf_sz=%u desc_addr=0x%x meta_addr=0x%x\n",
		op, inst->inst_id, &src_addr,
		(u32)((src_addr - core->fw_dma_addr) >> VIDC_ADDR_SHIFT),
		src_size,
		(u32)vb2_plane_size(&inst->src_buf->vb2_buf, 0),
		(u32)(core->desc_offset >> VIDC_ADDR_SHIFT),
		core->shm_offset + VIDC_META_INPUT_OFF);

	if (op == VIDC_OP_SEQ_HEADER) {
		dev_dbg(core->dev,
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

	dev_dbg(core->dev,
		"seq_header_work: seq_parsed=%d hdr_direct=%d\n",
		inst->seq_parsed, inst->seq_hdr_direct);

	if (inst->seq_parsed || inst->seq_hdr_direct)
		return;

	src_buf = v4l2_m2m_next_src_buf(inst->m2m_ctx);
	dev_dbg(core->dev, "seq_header_work: src_buf=%p\n", src_buf);
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
		u8 *uva = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
		u32 copy = min_t(u32, src_size, VIDC_SEQ_SCRATCH_SIZE);
		u8 *kva;

		if (!uva)
			goto submit;

		/*
		 * Copy the head of the bitstream into a private coherent
		 * scratch buffer and strip there. The application still owns
		 * the OUTPUT buffer contents, so we must not rewrite it in
		 * place (it also breaks DMABUF-imported buffers). SPS/PPS/AUD
		 * always live in the first few KB, so copying up to the
		 * scratch size is sufficient.
		 */
		if (!inst->seq_scratch_vaddr) {
			inst->seq_scratch_vaddr = dma_alloc_coherent(core->dev,
								     VIDC_SEQ_SCRATCH_SIZE,
								     &inst->seq_scratch_dma, GFP_KERNEL);
		}
		if (!inst->seq_scratch_vaddr ||
		    inst->seq_scratch_dma < core->fw_dma_addr)
			goto submit;	/* fall back: submit original unmodified */

		memcpy(inst->seq_scratch_vaddr, uva, copy);
		kva = inst->seq_scratch_vaddr;
		src_addr = inst->seq_scratch_dma;
		src_size = copy;

		print_hex_dump_debug("vidc seq_hdr BEFORE[0:16]: ",
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
			dev_dbg(core->dev,
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
						dev_dbg(core->dev,
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

		print_hex_dump_debug("vidc seq_hdr AFTER[0:40]: ",
				     DUMP_PREFIX_NONE, 16, 1, kva,
				     min_t(u32, src_size, 40), false);

		/*
		 * No dma_sync needed.  vb2 buffers now come from the SMIPOOL
		 * shared-dma-pool (no-map reserved memory at 0x38300000), which
		 * is DMA-coherent — CPU writes are visible to the firmware
		 * immediately.  Calling dma_sync_single_for_device on a no-map
		 * region crashes with a paging fault in v7_dma_clean_range
		 * because phys_to_virt(0x38300000) = 0x78300000 has no linear
		 * mapping for the kernel cache helper to operate on.
		 */
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

	/* Publish the firmware minimum so userspace can size CAPTURE. */
	if (inst->ctrl_min_cap && inst->min_dpb_count)
		v4l2_ctrl_s_ctrl(inst->ctrl_min_cap, inst->min_dpb_count);

	/*
	 * DPB is NOT allocated here. The CAPTURE buffers ARE the DPB, so the
	 * DPB registers + INIT_BUFFERS are programmed later, in CAPTURE
	 * start_streaming, once userspace has REQBUF'd/QBUF'd them in response
	 * to this SOURCE_CHANGE. Just publish the geometry and notify userspace.
	 */
	dpb_ret = 0;
	dev_dbg(core->dev,
		"Sequence parsed: %ux%u, min_dpb=%u — awaiting CAPTURE setup\n",
		inst->seq_width, inst->seq_height, inst->min_dpb_count);

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
 * Helper: hand back the CAPTURE buffer the firmware just displayed (zero-copy).
 *
 * The CAPTURE buffers ARE the DPB, so the firmware decoded the displayed frame
 * directly into one of them. Map the firmware's display address to the DPB slot
 * (== CAPTURE buffer index), remove THAT specific buffer from the m2m queue
 * (display order != queue order under B-frame reorder), drop the slot from the
 * firmware DPB mask, restore the input timestamp, and mark it DONE. No memcpy.
 */
static int vidc_dec_emit_dpb(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	struct vb2_v4l2_buffer *dst_buf;
	size_t payload = 0;
	u32 slot;
	int ret;

	ret = vidc_lookup_dpb_slot(inst, &slot, &payload);
	if (ret)
		return ret;

	dst_buf = v4l2_m2m_dst_buf_remove_by_idx(inst->m2m_ctx, slot);
	if (!dst_buf) {
		dev_err(core->dev,
			"display slot %u not in CAPTURE queue\n", slot);
		return -EFAULT;
	}

	/* Now owned by userspace — remove from the firmware DPB pool. */
	inst->dpb_hw_mask &= ~(1u << slot);

	dst_buf->sequence = inst->sequence_cap++;
	dst_buf->vb2_buf.timestamp =
		inst->frame_tag_ts[inst->display_frame_tag % VIDC_DPB_REG_SLOTS];
	vb2_set_plane_payload(&dst_buf->vb2_buf, 0, payload);
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	return 0;
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
	/*
	 * Mid-stream resolution change: the firmware flagged that the coded
	 * size changed (new SPS). Notify userspace so it can renegotiate the
	 * CAPTURE queue; emitting frames at the stale geometry would produce
	 * corrupt output.
	 */
	if (inst->display_resl_change) {
		static const struct v4l2_event ev = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
		};

		dev_dbg(core->dev,
			"mid-stream resolution change reported by firmware\n");
		v4l2_event_queue_fh(&inst->fh, &ev);
		inst->display_resl_change = 0;
	}

	switch (inst->display_status) {
	case VIDC_DISPLAY_STATUS_DECODE_ONLY:
		src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		if (src_buf) {
			src_buf->sequence = inst->sequence_out++;
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		}
		break;

	case VIDC_DISPLAY_STATUS_DISPLAY_ONLY:
		vidc_dec_emit_dpb(inst);
		break;

	case VIDC_DISPLAY_STATUS_DPB_EMPTY:
		/* Drain finished — no more held frames. */
		inst->draining = false;
		src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		if (src_buf) {
			src_buf->sequence = inst->sequence_out++;
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		}
		/*
		 * No decoded frame here — take any remaining queued CAPTURE
		 * buffer (head) as the empty end-of-stream marker.
		 */
		dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);
		if (dst_buf) {
			dst_buf->sequence = inst->sequence_cap++;
			inst->dpb_hw_mask &= ~(1u << dst_buf->vb2_buf.index);
			dst_buf->flags |= V4L2_BUF_FLAG_LAST;
			vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		}
		/* Drain complete — signal end of stream to userspace. */
		{
			static const struct v4l2_event eos = {
				.type = V4L2_EVENT_EOS,
			};
			v4l2_event_queue_fh(&inst->fh, &eos);
		}
		break;

	case VIDC_DISPLAY_STATUS_NOOP:
		dev_warn(core->dev,
			 "frame_done_work: firmware reported NOOP\n");
		fallthrough;

	case VIDC_DISPLAY_STATUS_DECODE_AND_DISPLAY:
	default:
		/*
		 * During a drain there is no input frame in flight, so emit
		 * the displayed held frame like DISPLAY_ONLY (no src buffer to
		 * pair). Otherwise both buffers are required.
		 */
		if (inst->draining) {
			vidc_dec_emit_dpb(inst);
			break;
		}

		src_buf = v4l2_m2m_src_buf_remove(inst->m2m_ctx);
		if (!src_buf) {
			dev_warn(core->dev,
				 "frame_done_work: missing src buffer\n");
			break;
		}
		src_buf->sequence = inst->sequence_out++;

		if (vidc_dec_emit_dpb(inst))
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		else
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		break;
	}

	/*
	 * Normal job completion. During a drain each LAST_FRAME is its own
	 * one-shot job: this job_finish lets the scheduler run device_run
	 * again, which re-issues the next LAST_FRAME (job_ready stays true
	 * while draining) until the firmware reports DPB_EMPTY — matching the
	 * webOS handler that re-runs the EOS command per held frame.
	 */
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

	/*
	 * Drain: the OUTPUT queue is empty but a STOP was requested. Issue a
	 * single VIDC_OP_LAST_FRAME (empty stream) so the firmware flushes
	 * the held reorder frames. draining (set here, not at STOP time)
	 * makes submit_frame use the LAST_FRAME opcode and keeps curr_inst
	 * alive across the resulting DISPLAY_ONLY responses. A dst buffer
	 * must be available for the firmware to emit the first held frame.
	 */
	if (!src_buf && (inst->drain_pending || inst->draining) && dst_buf) {
		inst->drain_pending = false;
		inst->draining = true;

		/*
		 * Safety: each LAST_FRAME yields one held frame; the firmware
		 * eventually reports DPB_EMPTY. If it never does, cap the
		 * re-issues so we don't loop forever — force EOS and stop.
		 */
		if (inst->drain_count++ > inst->dpb_count + 8) {
			static const struct v4l2_event eos = {
				.type = V4L2_EVENT_EOS,
			};
			dev_warn(inst->core->dev,
				 "drain cap reached (%u), forcing EOS\n",
				 inst->drain_count);
			if (dst_buf) {
				dst_buf = v4l2_m2m_dst_buf_remove(inst->m2m_ctx);
				if (dst_buf) {
					dst_buf->sequence = inst->sequence_cap++;
					dst_buf->flags |= V4L2_BUF_FLAG_LAST;
					vb2_set_plane_payload(&dst_buf->vb2_buf,
							      0, 0);
					v4l2_m2m_buf_done(dst_buf,
							  VB2_BUF_STATE_DONE);
				}
			}
			v4l2_event_queue_fh(&inst->fh, &eos);
			inst->draining = false;
			v4l2_m2m_job_finish(inst->core->m2m_dev_dec,
					    inst->m2m_ctx);
			return;
		}

		inst->error = 0;
		vidc_dec_submit_frame(inst, inst->core->fw_dma_addr, 0, 0);
		return;
	}

	if (!src_buf || !dst_buf) {
		dev_dbg(inst->core->dev,
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

	dev_dbg(inst->core->dev,
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

/*
 * The default m2m scheduler only runs device_run when BOTH queues have a
 * buffer. We additionally allow a run when a drain is pending and the
 * OUTPUT queue has drained (no src), provided a CAPTURE buffer is free for
 * the firmware to emit the first held frame into.
 */
static int vidc_dec_job_ready(void *priv)
{
	struct vidc_inst *inst = priv;
	bool have_src = v4l2_m2m_num_src_bufs_ready(inst->m2m_ctx) > 0;
	bool have_dst = v4l2_m2m_num_dst_bufs_ready(inst->m2m_ctx) > 0;

	if (!have_dst)
		return 0;
	return have_src || inst->drain_pending || inst->draining;
}

static const struct v4l2_m2m_ops vidc_dec_m2m_ops = {
	.device_run = vidc_dec_device_run,
	.job_ready = vidc_dec_job_ready,
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
	/*
	 * MMAP only: the CAPTURE buffers ARE the DPB and must live in the SMI
	 * bank the firmware can address (no IOMMU). An imported DMABUF from
	 * arbitrary memory is unreachable by the RISC — same limitation as
	 * s5p-mfc v5's memory bank.
	 */
	dst_vq->io_modes = VB2_MMAP;
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
	init_completion(&inst->frame_done_compl);
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

	/*
	 * Mark the OUTPUT (bitstream) queue "buffered" so the m2m scheduler
	 * still consults job_ready when no source buffer is queued. Without
	 * this, v4l2_m2m_try_schedule() bails on an empty OUTPUT queue and
	 * device_run() is never called for the EOS drain (which issues
	 * LAST_FRAME with no input). job_ready() gates the no-source case to
	 * an active drain only.
	 */
	v4l2_m2m_set_src_buffered(inst->m2m_ctx, true);

	ret = v4l2_ctrl_handler_init(&inst->ctrl_handler, 1);
	if (ret)
		goto err_m2m_ctx_release;

	/*
	 * Required by the V4L2 stateful decoder uAPI: userspace reads this
	 * to size the CAPTURE queue. Read-only; updated with the firmware
	 * minimum after the sequence header is parsed (vidc_dec_seq_done_work).
	 */
	inst->ctrl_min_cap = v4l2_ctrl_new_std(&inst->ctrl_handler, NULL,
					       V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
					       1, 32, 1, 4);
	if (inst->ctrl_min_cap)
		inst->ctrl_min_cap->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (inst->ctrl_handler.error) {
		ret = inst->ctrl_handler.error;
		v4l2_ctrl_handler_free(&inst->ctrl_handler);
		goto err_m2m_ctx_release;
	}

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
	 * Order matters.  The VIDC IRQ handler runs entirely under
	 * core->irqlock: it reads inst = core->curr_inst and, for a
	 * FRAME_DONE, queue_work(&inst->frame_done_work).  If we cancelled
	 * the work first and only THEN cleared curr_inst, an IRQ landing in
	 * between would re-queue frame_done_work after the cancel — and it
	 * would run against this inst after we kfree() it (use-after-free,
	 * the source of the frame_done_work oops + the multimedia-IOMMU
	 * fault storm).
	 *
	 * So clear curr_inst under irqlock FIRST (after this the IRQ sees
	 * NULL and the `if (inst)` guards skip queueing), THEN drain any
	 * work already queued before the clear.
	 */
	{
		unsigned long flags;
		spin_lock_irqsave(&core->irqlock, flags);
		if (core->curr_inst == inst)
			core->curr_inst = NULL;
		spin_unlock_irqrestore(&core->irqlock, flags);
	}

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

	if (inst->seq_scratch_vaddr)
		dma_free_coherent(inst->core->dev, VIDC_SEQ_SCRATCH_SIZE,
				  inst->seq_scratch_vaddr, inst->seq_scratch_dma);

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
