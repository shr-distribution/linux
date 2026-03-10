// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 Rotator V4L2 mem2mem driver
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_rotator driver from webOS kernel.
 * Ported to V4L2 mem2mem framework for mainline Linux.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "rotator_hw.h"

#define ROTATOR_NAME		"qcom-rotator"
#define ROTATOR_MIN_WIDTH	16
#define ROTATOR_MIN_HEIGHT	16

/* Interconnect bandwidth */
#define ROTATOR_BW_AVG		(500 * 1024 * 1024)	/* 500 MB/s */
#define ROTATOR_BW_PEAK		(1000 * 1024 * 1024)	/* 1 GB/s */

struct rotator_fmt {
	u32 fourcc;
	u32 depth;
	u32 num_planes;
	u32 chroma;
	bool cbcr;
	bool has_alpha;
};

static const struct rotator_fmt rotator_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= 16,
		.num_planes	= 1,
		.chroma		= ROTATOR_CHROMA_RGB,
	},
	{
		.fourcc		= V4L2_PIX_FMT_XRGB32,
		.depth		= 32,
		.num_planes	= 1,
		.chroma		= ROTATOR_CHROMA_RGB,
	},
	{
		.fourcc		= V4L2_PIX_FMT_ARGB32,
		.depth		= 32,
		.num_planes	= 1,
		.chroma		= ROTATOR_CHROMA_RGB,
		.has_alpha	= true,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= 12,
		.num_planes	= 2,
		.chroma		= ROTATOR_CHROMA_H2V2,
		.cbcr		= true,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV21,
		.depth		= 12,
		.num_planes	= 2,
		.chroma		= ROTATOR_CHROMA_H2V2,
		.cbcr		= false,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV16,
		.depth		= 16,
		.num_planes	= 2,
		.chroma		= ROTATOR_CHROMA_H2V1,
		.cbcr		= true,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV61,
		.depth		= 16,
		.num_planes	= 2,
		.chroma		= ROTATOR_CHROMA_H2V1,
		.cbcr		= false,
	},
};

struct rotator_frame {
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
	const struct rotator_fmt *fmt;
};

struct rotator_dev {
	struct device *dev;
	void __iomem *base;
	int irq;

	struct clk *core_clk;
	struct clk *axi_clk;
	struct clk *ahb_clk;
	struct icc_path *icc_path;

	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex lock;

	/* Current operation */
	struct completion done;
	int error;
};

struct rotator_ctx {
	struct v4l2_fh fh;
	struct rotator_dev *rot;
	struct v4l2_ctrl_handler ctrl_handler;

	struct rotator_frame src;
	struct rotator_frame dst;
	u32 rotation;
	bool hflip;
	bool vflip;
};

static inline struct rotator_ctx *rotator_fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct rotator_ctx, fh);
}

static const struct rotator_fmt *find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rotator_formats); i++) {
		if (rotator_formats[i].fourcc == fourcc)
			return &rotator_formats[i];
	}
	return NULL;
}

/*
 * V4L2 ioctl operations
 */

static int rotator_querycap(struct file *file, void *priv,
			    struct v4l2_capability *cap)
{
	strscpy(cap->driver, ROTATOR_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm MSM8660 Rotator", sizeof(cap->card));

	return 0;
}

static int rotator_enum_fmt(struct file *file, void *priv,
			    struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(rotator_formats))
		return -EINVAL;

	f->pixelformat = rotator_formats[f->index].fourcc;

	return 0;
}

static int rotator_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct rotator_ctx *ctx = rotator_fh_to_ctx(file->private_data);
	struct rotator_frame *frame;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		frame = &ctx->src;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		frame = &ctx->dst;
	else
		return -EINVAL;

	f->fmt.pix.width = frame->width;
	f->fmt.pix.height = frame->height;
	f->fmt.pix.pixelformat = frame->fmt->fourcc;
	f->fmt.pix.bytesperline = frame->bytesperline;
	f->fmt.pix.sizeimage = frame->sizeimage;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
	f->fmt.pix.field = V4L2_FIELD_NONE;

	return 0;
}

static int rotator_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct rotator_fmt *fmt;
	u32 bpp;

	fmt = find_format(f->fmt.pix.pixelformat);
	if (!fmt)
		fmt = &rotator_formats[0];

	f->fmt.pix.pixelformat = fmt->fourcc;
	f->fmt.pix.width = clamp(f->fmt.pix.width,
				 (u32)ROTATOR_MIN_WIDTH,
				 (u32)ROTATOR_MAX_WIDTH);
	f->fmt.pix.height = clamp(f->fmt.pix.height,
				  (u32)ROTATOR_MIN_HEIGHT,
				  (u32)ROTATOR_MAX_HEIGHT);

	/* Align to 16 pixels for hardware requirements */
	f->fmt.pix.width = ALIGN(f->fmt.pix.width, 16);
	f->fmt.pix.height = ALIGN(f->fmt.pix.height, 16);

	bpp = fmt->depth / 8;
	if (fmt->num_planes == 1) {
		f->fmt.pix.bytesperline = f->fmt.pix.width * bpp;
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
				       f->fmt.pix.height;
	} else {
		/* YUV planar */
		f->fmt.pix.bytesperline = f->fmt.pix.width;
		f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height *
				       fmt->depth / 8;
	}

	f->fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
	f->fmt.pix.field = V4L2_FIELD_NONE;

	return 0;
}

static int rotator_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct rotator_ctx *ctx = rotator_fh_to_ctx(file->private_data);
	struct rotator_frame *frame;
	const struct rotator_fmt *fmt;
	int ret;

	ret = rotator_try_fmt(file, priv, f);
	if (ret)
		return ret;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		frame = &ctx->src;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		frame = &ctx->dst;
	else
		return -EINVAL;

	fmt = find_format(f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	frame->width = f->fmt.pix.width;
	frame->height = f->fmt.pix.height;
	frame->bytesperline = f->fmt.pix.bytesperline;
	frame->sizeimage = f->fmt.pix.sizeimage;
	frame->fmt = fmt;

	return 0;
}

static int rotator_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rotator_ctx *ctx = container_of(ctrl->handler,
					       struct rotator_ctx,
					       ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctx->rotation = ctrl->val;
		break;
	case V4L2_CID_HFLIP:
		ctx->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->vflip = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops rotator_ctrl_ops = {
	.s_ctrl = rotator_s_ctrl,
};

/*
 * mem2mem operations
 */

static void rotator_device_run(void *priv)
{
	struct rotator_ctx *ctx = priv;
	struct rotator_dev *rot = ctx->rot;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t src_y, src_c, dst_y, dst_c;
	u32 rotation = 0;
	u32 src_w, src_h, dst_w, dst_h;
	int ret;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	src_y = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	dst_y = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);

	/* Calculate chroma plane addresses for YUV formats */
	if (ctx->src.fmt->num_planes == 2) {
		src_c = src_y + ctx->src.width * ctx->src.height;
		dst_c = dst_y + ctx->dst.width * ctx->dst.height;
	} else {
		src_c = 0;
		dst_c = 0;
	}

	src_w = ctx->src.width;
	src_h = ctx->src.height;

	/* Calculate destination dimensions based on rotation */
	if (ctx->rotation == 90 || ctx->rotation == 270) {
		dst_w = src_h;
		dst_h = src_w;
	} else {
		dst_w = src_w;
		dst_h = src_h;
	}

	/* Build rotation flags */
	if (ctx->rotation == 90)
		rotation = ROTATOR_ROT_90;
	else if (ctx->rotation == 180)
		rotation = ROTATOR_FLIP_LR | ROTATOR_FLIP_UD;
	else if (ctx->rotation == 270)
		rotation = ROTATOR_ROT_90 | ROTATOR_FLIP_LR | ROTATOR_FLIP_UD;

	if (ctx->hflip)
		rotation ^= ROTATOR_FLIP_LR;
	if (ctx->vflip)
		rotation ^= ROTATOR_FLIP_UD;

	/* Enable clocks via runtime PM */
	ret = pm_runtime_resume_and_get(rot->dev);
	if (ret < 0) {
		rot->error = ret;
		goto done;
	}

	reinit_completion(&rot->done);
	rot->error = 0;

	/* Configure hardware */
	rotator_hw_reset(rot->base);
	rotator_hw_set_src_size(rot->base, src_w, src_h);
	rotator_hw_set_src_addr(rot->base, src_y, src_c);
	rotator_hw_set_dst_addr(rot->base, dst_y, dst_c);
	rotator_hw_set_strides(rot->base, ctx->src.bytesperline,
			       ctx->dst.bytesperline);
	rotator_hw_set_rotation(rot->base, rotation, ctx->src.fmt->chroma);

	if (ctx->src.fmt->chroma == ROTATOR_CHROMA_RGB) {
		u32 bpp = ctx->src.fmt->depth / 8;

		rotator_hw_set_format_rgb(rot->base, bpp,
					  ctx->src.fmt->has_alpha);
	} else {
		rotator_hw_set_format_yuv(rot->base, ctx->src.fmt->chroma,
					  ctx->src.fmt->cbcr);
	}

	/* Enable interrupt and start */
	rotator_hw_enable_irq(rot->base);
	rotator_hw_start(rot->base);

	/* Wait for completion */
	if (!wait_for_completion_timeout(&rot->done,
					 msecs_to_jiffies(500))) {
		dev_err(rot->dev, "rotation timeout\n");
		rot->error = -ETIMEDOUT;
	}

	rotator_hw_disable_irq(rot->base);
	pm_runtime_put(rot->dev);

done:
	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (rot->error) {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
	} else {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
				      ctx->dst.sizeimage);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	}

	v4l2_m2m_job_finish(rot->m2m_dev, ctx->fh.m2m_ctx);
}

static const struct v4l2_m2m_ops rotator_m2m_ops = {
	.device_run = rotator_device_run,
};

/*
 * videobuf2 operations
 */

static int rotator_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct rotator_ctx *ctx = vb2_get_drv_priv(vq);
	struct rotator_frame *frame;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		frame = &ctx->src;
	else
		frame = &ctx->dst;

	if (*nplanes) {
		if (sizes[0] < frame->sizeimage)
			return -EINVAL;
	} else {
		*nplanes = 1;
		sizes[0] = frame->sizeimage;
	}

	return 0;
}

static int rotator_buf_prepare(struct vb2_buffer *vb)
{
	struct rotator_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct rotator_frame *frame;

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		frame = &ctx->src;
	else
		frame = &ctx->dst;

	vb2_set_plane_payload(vb, 0, frame->sizeimage);

	return 0;
}

static void rotator_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rotator_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static const struct vb2_ops rotator_qops = {
	.queue_setup	= rotator_queue_setup,
	.buf_prepare	= rotator_buf_prepare,
	.buf_queue	= rotator_buf_queue,
	.wait_prepare	= vb2_ops_wait_prepare,
	.wait_finish	= vb2_ops_wait_finish,
};

static int rotator_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct rotator_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->ops = &rotator_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->rot->lock;
	src_vq->dev = ctx->rot->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->ops = &rotator_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->rot->lock;
	dst_vq->dev = ctx->rot->dev;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */

static int rotator_open(struct file *file)
{
	struct rotator_dev *rot = video_drvdata(file);
	struct rotator_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->rot = rot;
	v4l2_fh_init(&ctx->fh, &rot->vfd);
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(rot->m2m_dev, ctx,
					    rotator_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_fh;
	}

	/* Initialize control handler */
	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rotator_ctrl_ops,
			  V4L2_CID_ROTATE, 0, 270, 90, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rotator_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &rotator_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto err_ctrl;
	}

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	v4l2_fh_add(&ctx->fh, file);

	/* Set default format */
	ctx->src.width = ROTATOR_MIN_WIDTH;
	ctx->src.height = ROTATOR_MIN_HEIGHT;
	ctx->src.bytesperline = ROTATOR_MIN_WIDTH * 4;
	ctx->src.sizeimage = ctx->src.bytesperline * ROTATOR_MIN_HEIGHT;
	ctx->src.fmt = &rotator_formats[2];	/* ARGB32 */

	ctx->dst = ctx->src;
	ctx->rotation = 0;
	ctx->hflip = false;
	ctx->vflip = false;

	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int rotator_release(struct file *file)
{
	struct rotator_ctx *ctx = rotator_fh_to_ctx(file->private_data);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations rotator_fops = {
	.owner		= THIS_MODULE,
	.open		= rotator_open,
	.release	= rotator_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops rotator_ioctl_ops = {
	.vidioc_querycap	= rotator_querycap,
	.vidioc_enum_fmt_vid_cap = rotator_enum_fmt,
	.vidioc_enum_fmt_vid_out = rotator_enum_fmt,
	.vidioc_g_fmt_vid_cap	= rotator_g_fmt,
	.vidioc_g_fmt_vid_out	= rotator_g_fmt,
	.vidioc_try_fmt_vid_cap	= rotator_try_fmt,
	.vidioc_try_fmt_vid_out	= rotator_try_fmt,
	.vidioc_s_fmt_vid_cap	= rotator_s_fmt,
	.vidioc_s_fmt_vid_out	= rotator_s_fmt,
	.vidioc_reqbufs		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf	= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf		= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf		= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf	= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs	= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf		= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon	= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff	= v4l2_m2m_ioctl_streamoff,
};

/*
 * IRQ handler
 */

static irqreturn_t rotator_irq(int irq, void *data)
{
	struct rotator_dev *rot = data;
	u32 status;

	status = rotator_hw_get_irq_status(rot->base);

	if (status & ROTATOR_IRQ_ERROR) {
		dev_err(rot->dev, "rotation error\n");
		rot->error = -EIO;
	}

	rotator_hw_clear_irq(rot->base);
	complete(&rot->done);

	return IRQ_HANDLED;
}

/*
 * Power management
 */

static int rotator_runtime_suspend(struct device *dev)
{
	struct rotator_dev *rot = dev_get_drvdata(dev);

	clk_disable_unprepare(rot->axi_clk);
	clk_disable_unprepare(rot->ahb_clk);
	clk_disable_unprepare(rot->core_clk);

	if (rot->icc_path)
		icc_set_bw(rot->icc_path, 0, 0);

	return 0;
}

static int rotator_runtime_resume(struct device *dev)
{
	struct rotator_dev *rot = dev_get_drvdata(dev);
	int ret;

	if (rot->icc_path) {
		ret = icc_set_bw(rot->icc_path, ROTATOR_BW_AVG, ROTATOR_BW_PEAK);
		if (ret) {
			dev_err(dev, "failed to set interconnect bw: %d\n",
				ret);
			return ret;
		}
	}

	ret = clk_prepare_enable(rot->core_clk);
	if (ret)
		goto err_icc;

	ret = clk_prepare_enable(rot->ahb_clk);
	if (ret)
		goto err_core;

	ret = clk_prepare_enable(rot->axi_clk);
	if (ret)
		goto err_ahb;

	return 0;

err_ahb:
	clk_disable_unprepare(rot->ahb_clk);
err_core:
	clk_disable_unprepare(rot->core_clk);
err_icc:
	if (rot->icc_path)
		icc_set_bw(rot->icc_path, 0, 0);
	return ret;
}

static const struct dev_pm_ops rotator_pm_ops = {
	SET_RUNTIME_PM_OPS(rotator_runtime_suspend, rotator_runtime_resume, NULL)
};

/*
 * Platform driver
 */

static int rotator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rotator_dev *rot;
	int ret;

	rot = devm_kzalloc(dev, sizeof(*rot), GFP_KERNEL);
	if (!rot)
		return -ENOMEM;

	rot->dev = dev;
	mutex_init(&rot->lock);
	init_completion(&rot->done);

	/* Map registers */
	rot->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rot->base))
		return PTR_ERR(rot->base);

	/* Get IRQ */
	rot->irq = platform_get_irq(pdev, 0);
	if (rot->irq < 0)
		return rot->irq;

	ret = devm_request_irq(dev, rot->irq, rotator_irq, 0,
			       ROTATOR_NAME, rot);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		return ret;
	}

	/* Get clocks */
	rot->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(rot->core_clk)) {
		dev_err(dev, "failed to get core clock\n");
		return PTR_ERR(rot->core_clk);
	}

	rot->ahb_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(rot->ahb_clk)) {
		dev_err(dev, "failed to get iface clock\n");
		return PTR_ERR(rot->ahb_clk);
	}

	rot->axi_clk = devm_clk_get(dev, "axi");
	if (IS_ERR(rot->axi_clk)) {
		dev_err(dev, "failed to get axi clock\n");
		return PTR_ERR(rot->axi_clk);
	}

	/* Get optional interconnect path */
	rot->icc_path = devm_of_icc_get(dev, "rotator-mem");
	if (IS_ERR(rot->icc_path)) {
		ret = PTR_ERR(rot->icc_path);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_dbg(dev, "interconnect not available\n");
		rot->icc_path = NULL;
	}

	/* Register V4L2 device */
	ret = v4l2_device_register(dev, &rot->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register V4L2 device: %d\n", ret);
		return ret;
	}

	/* Initialize mem2mem device */
	rot->m2m_dev = v4l2_m2m_init(&rotator_m2m_ops);
	if (IS_ERR(rot->m2m_dev)) {
		ret = PTR_ERR(rot->m2m_dev);
		dev_err(dev, "failed to init mem2mem device: %d\n", ret);
		goto err_v4l2;
	}

	/* Register video device */
	rot->vfd.fops = &rotator_fops;
	rot->vfd.ioctl_ops = &rotator_ioctl_ops;
	rot->vfd.minor = -1;
	rot->vfd.release = video_device_release_empty;
	rot->vfd.lock = &rot->lock;
	rot->vfd.v4l2_dev = &rot->v4l2_dev;
	rot->vfd.vfl_dir = VFL_DIR_M2M;
	rot->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	strscpy(rot->vfd.name, ROTATOR_NAME, sizeof(rot->vfd.name));

	video_set_drvdata(&rot->vfd, rot);

	ret = video_register_device(&rot->vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "failed to register video device: %d\n", ret);
		goto err_m2m;
	}

	platform_set_drvdata(pdev, rot);
	pm_runtime_enable(dev);

	dev_info(dev, "Qualcomm MSM8660 Rotator driver probed (HW version: 0x%08x)\n",
		 rotator_hw_get_version(rot->base));

	return 0;

err_m2m:
	v4l2_m2m_release(rot->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&rot->v4l2_dev);
	return ret;
}

static void rotator_remove(struct platform_device *pdev)
{
	struct rotator_dev *rot = platform_get_drvdata(pdev);

	pm_runtime_disable(rot->dev);
	video_unregister_device(&rot->vfd);
	v4l2_m2m_release(rot->m2m_dev);
	v4l2_device_unregister(&rot->v4l2_dev);
}

static const struct of_device_id rotator_of_match[] = {
	{ .compatible = "qcom,msm8660-rotator" },
	{ },
};
MODULE_DEVICE_TABLE(of, rotator_of_match);

static struct platform_driver rotator_driver = {
	.probe	= rotator_probe,
	.remove	= rotator_remove,
	.driver	= {
		.name		= ROTATOR_NAME,
		.of_match_table	= rotator_of_match,
		.pm		= &rotator_pm_ops,
	},
};

module_platform_driver(rotator_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660 Rotator V4L2 driver");
MODULE_LICENSE("GPL v2");
