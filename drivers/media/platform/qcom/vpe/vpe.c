// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 Video Processing Engine (VPE) V4L2 mem2mem driver
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_vpe1 driver from webOS kernel.
 * Ported to V4L2 mem2mem framework.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vpe_hw.h"

#define VPE_NAME		"qcom-vpe"
#define VPE_MAX_WIDTH		1920
#define VPE_MAX_HEIGHT		1088
#define VPE_MIN_WIDTH		32
#define VPE_MIN_HEIGHT		32
#define VPE_ALIGN		16

/* Interconnect bandwidth in KB/s (~1.5 GB/s) */
#define VPE_ICC_AVG_BW		1521190
#define VPE_ICC_PEAK_BW		1521190

enum vpe_fmt_type {
	VPE_FMT_TYPE_OUTPUT	= BIT(0),
	VPE_FMT_TYPE_CAPTURE	= BIT(1),
};

struct vpe_fmt {
	u32	fourcc;
	int	depth;
	u32	types;
};

static const struct vpe_fmt vpe_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_NV12,
		.depth	= 12,
		.types	= VPE_FMT_TYPE_OUTPUT | VPE_FMT_TYPE_CAPTURE,
	},
};

struct vpe_dev {
	struct device		*dev;
	void __iomem		*base;
	struct clk		*core_clk;
	struct clk		*axi_clk;
	struct clk		*ahb_clk;
	struct icc_path		*icc_path;
	int			irq;

	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct mutex		lock;	/* device lock */

	spinlock_t		irqlock;
};

struct vpe_frame {
	u32			width;
	u32			height;
	u32			bytesperline;
	u32			sizeimage;
	const struct vpe_fmt	*fmt;
};

struct vpe_ctx {
	struct v4l2_fh		fh;
	struct vpe_dev		*vpe;
	struct v4l2_ctrl_handler ctrl_handler;

	struct vpe_frame	src;
	struct vpe_frame	dst;
	int			rotation;
};

static inline struct vpe_ctx *vpe_fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct vpe_ctx, fh);
}

static const struct vpe_fmt *vpe_find_format(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vpe_formats); i++) {
		if (vpe_formats[i].fourcc == fourcc)
			return &vpe_formats[i];
	}

	return NULL;
}

/*
 * V4L2 ioctl operations
 */

static int vpe_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strscpy(cap->driver, VPE_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm VPE", sizeof(cap->card));

	return 0;
}

static int vpe_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	const struct vpe_fmt *fmt;
	u32 type;

	if (f->index >= ARRAY_SIZE(vpe_formats))
		return -EINVAL;

	type = V4L2_TYPE_IS_OUTPUT(f->type) ? VPE_FMT_TYPE_OUTPUT : VPE_FMT_TYPE_CAPTURE;
	fmt = &vpe_formats[f->index];

	if (!(fmt->types & type))
		return -EINVAL;

	f->pixelformat = fmt->fourcc;

	return 0;
}

static int vpe_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vpe_ctx *ctx = vpe_fh_to_ctx(priv);
	struct vpe_frame *frame;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	frame = V4L2_TYPE_IS_OUTPUT(f->type) ? &ctx->src : &ctx->dst;

	pix->width = frame->width;
	pix->height = frame->height;
	pix->pixelformat = frame->fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = frame->bytesperline;
	pix->sizeimage = frame->sizeimage;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	return 0;
}

static int vpe_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const struct vpe_fmt *fmt;
	u32 type;

	type = V4L2_TYPE_IS_OUTPUT(f->type) ? VPE_FMT_TYPE_OUTPUT : VPE_FMT_TYPE_CAPTURE;
	fmt = vpe_find_format(pix->pixelformat);

	if (!fmt || !(fmt->types & type))
		fmt = &vpe_formats[0];

	pix->pixelformat = fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;

	/* Align dimensions */
	pix->width = clamp(pix->width, VPE_MIN_WIDTH, VPE_MAX_WIDTH);
	pix->height = clamp(pix->height, VPE_MIN_HEIGHT, VPE_MAX_HEIGHT);
	pix->width = ALIGN(pix->width, VPE_ALIGN);
	pix->height = ALIGN(pix->height, VPE_ALIGN);

	pix->bytesperline = pix->width;
	pix->sizeimage = pix->bytesperline * pix->height * fmt->depth / 8;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	return 0;
}

static int vpe_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vpe_ctx *ctx = vpe_fh_to_ctx(priv);
	struct vpe_frame *frame;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	ret = vpe_try_fmt(file, priv, f);
	if (ret)
		return ret;

	frame = V4L2_TYPE_IS_OUTPUT(f->type) ? &ctx->src : &ctx->dst;

	frame->width = pix->width;
	frame->height = pix->height;
	frame->bytesperline = pix->bytesperline;
	frame->sizeimage = pix->sizeimage;
	frame->fmt = vpe_find_format(pix->pixelformat);

	return 0;
}

static int vpe_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpe_ctx *ctx = container_of(ctrl->handler, struct vpe_ctx, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctx->rotation = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops vpe_ctrl_ops = {
	.s_ctrl = vpe_s_ctrl,
};

static const struct v4l2_ioctl_ops vpe_ioctl_ops = {
	.vidioc_querycap		= vpe_querycap,

	.vidioc_enum_fmt_vid_cap	= vpe_enum_fmt,
	.vidioc_enum_fmt_vid_out	= vpe_enum_fmt,

	.vidioc_g_fmt_vid_cap		= vpe_g_fmt,
	.vidioc_g_fmt_vid_out		= vpe_g_fmt,

	.vidioc_try_fmt_vid_cap		= vpe_try_fmt,
	.vidioc_try_fmt_vid_out		= vpe_try_fmt,

	.vidioc_s_fmt_vid_cap		= vpe_s_fmt,
	.vidioc_s_fmt_vid_out		= vpe_s_fmt,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/*
 * V4L2 mem2mem operations
 */

static void vpe_device_run(void *priv)
{
	struct vpe_ctx *ctx = priv;
	struct vpe_dev *vpe = ctx->vpe;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t src_y, src_cbcr, dst_y, dst_cbcr;
	u32 src_stride, dst_stride;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	/* Get buffer addresses (NV12: Y plane followed by interleaved CbCr) */
	src_y = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	src_cbcr = src_y + ctx->src.bytesperline * ctx->src.height;
	src_stride = ctx->src.bytesperline;

	dst_y = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	dst_cbcr = dst_y + ctx->dst.bytesperline * ctx->dst.height;
	dst_stride = ctx->dst.bytesperline;

	/* Configure hardware */
	vpe_hw_set_src_addr(vpe->base, src_y, src_cbcr);
	vpe_hw_set_dst_addr(vpe->base, dst_y, dst_cbcr);
	vpe_hw_set_src_size(vpe->base, ctx->src.width, ctx->src.height, src_stride);
	vpe_hw_set_dst_size(vpe->base, ctx->dst.width, ctx->dst.height, dst_stride);
	vpe_hw_set_scale(vpe->base, ctx->src.width, ctx->src.height,
			 ctx->dst.width, ctx->dst.height);
	vpe_hw_set_rotation(vpe->base, ctx->rotation);

	/* Enable interrupt and start processing */
	vpe_hw_enable_irq(vpe->base);
	vpe_hw_start(vpe->base);
}

static irqreturn_t vpe_irq_handler(int irq, void *dev_id)
{
	struct vpe_dev *vpe = dev_id;
	struct vpe_ctx *ctx;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	u32 status;

	status = vpe_hw_get_irq_status(vpe->base);
	if (!(status & VPE_INTR_STATUS_DONE))
		return IRQ_NONE;

	vpe_hw_clear_irq(vpe->base);
	vpe_hw_disable_irq(vpe->base);

	ctx = v4l2_m2m_get_curr_priv(vpe->m2m_dev);
	if (!ctx)
		return IRQ_HANDLED;

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (src_buf && dst_buf) {
		dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
		dst_buf->timecode = src_buf->timecode;
		dst_buf->flags = src_buf->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	}

	v4l2_m2m_job_finish(vpe->m2m_dev, ctx->fh.m2m_ctx);

	return IRQ_HANDLED;
}

static int vpe_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct vpe_ctx *ctx = vb2_get_drv_priv(vq);
	struct vpe_frame *frame;

	frame = V4L2_TYPE_IS_OUTPUT(vq->type) ? &ctx->src : &ctx->dst;

	if (*nplanes) {
		if (sizes[0] < frame->sizeimage)
			return -EINVAL;
	} else {
		*nplanes = 1;
		sizes[0] = frame->sizeimage;
	}

	return 0;
}

static int vpe_buf_prepare(struct vb2_buffer *vb)
{
	struct vpe_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vpe_frame *frame;

	frame = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ? &ctx->src : &ctx->dst;

	if (vb2_plane_size(vb, 0) < frame->sizeimage) {
		dev_err(ctx->vpe->dev, "Buffer too small (%lu < %u)\n",
			vb2_plane_size(vb, 0), frame->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, frame->sizeimage);

	return 0;
}

static void vpe_buf_queue(struct vb2_buffer *vb)
{
	struct vpe_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int vpe_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vpe_ctx *ctx = vb2_get_drv_priv(vq);
	struct vpe_dev *vpe = ctx->vpe;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		ret = pm_runtime_resume_and_get(vpe->dev);
		if (ret < 0)
			return ret;

		ret = vpe_hw_reset(vpe->base);
		if (ret) {
			pm_runtime_put(vpe->dev);
			return ret;
		}
	}

	return 0;
}

static void vpe_stop_streaming(struct vb2_queue *vq)
{
	struct vpe_ctx *ctx = vb2_get_drv_priv(vq);
	struct vpe_dev *vpe = ctx->vpe;
	struct vb2_v4l2_buffer *buf;

	/* Return all buffers */
	while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		pm_runtime_put(vpe->dev);
}

static const struct vb2_ops vpe_qops = {
	.queue_setup		= vpe_queue_setup,
	.buf_prepare		= vpe_buf_prepare,
	.buf_queue		= vpe_buf_queue,
	.start_streaming	= vpe_start_streaming,
	.stop_streaming		= vpe_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int vpe_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct vpe_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &vpe_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->vpe->lock;
	src_vq->dev = ctx->vpe->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &vpe_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->vpe->lock;
	dst_vq->dev = ctx->vpe->dev;

	return vb2_queue_init(dst_vq);
}

static const struct v4l2_m2m_ops vpe_m2m_ops = {
	.device_run	= vpe_device_run,
};

/*
 * File operations
 */

static int vpe_open(struct file *file)
{
	struct vpe_dev *vpe = video_drvdata(file);
	struct vpe_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->vpe = vpe;
	v4l2_fh_init(&ctx->fh, &vpe->vfd);
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vpe->m2m_dev, ctx, vpe_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_fh;
	}

	/* Initialize control handler */
	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &vpe_ctrl_ops,
			  V4L2_CID_ROTATE, 0, 270, 90, 0);

	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto err_ctrl;
	}

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	v4l2_fh_add(&ctx->fh, file);

	/* Set default format */
	ctx->src.width = VPE_MIN_WIDTH;
	ctx->src.height = VPE_MIN_HEIGHT;
	ctx->src.bytesperline = VPE_MIN_WIDTH;
	ctx->src.sizeimage = VPE_MIN_WIDTH * VPE_MIN_HEIGHT * 3 / 2;
	ctx->src.fmt = &vpe_formats[0];

	ctx->dst = ctx->src;
	ctx->rotation = 0;

	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int vpe_release(struct file *file)
{
	struct vpe_ctx *ctx = vpe_fh_to_ctx(file->private_data);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations vpe_fops = {
	.owner		= THIS_MODULE,
	.open		= vpe_open,
	.release	= vpe_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/*
 * Platform driver
 */

static int vpe_runtime_suspend(struct device *dev)
{
	struct vpe_dev *vpe = dev_get_drvdata(dev);

	icc_set_bw(vpe->icc_path, 0, 0);

	clk_disable_unprepare(vpe->ahb_clk);
	clk_disable_unprepare(vpe->axi_clk);
	clk_disable_unprepare(vpe->core_clk);

	return 0;
}

static int vpe_runtime_resume(struct device *dev)
{
	struct vpe_dev *vpe = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(vpe->core_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(vpe->axi_clk);
	if (ret)
		goto err_core;

	ret = clk_prepare_enable(vpe->ahb_clk);
	if (ret)
		goto err_axi;

	icc_set_bw(vpe->icc_path, VPE_ICC_AVG_BW, VPE_ICC_PEAK_BW);

	return 0;

err_axi:
	clk_disable_unprepare(vpe->axi_clk);
err_core:
	clk_disable_unprepare(vpe->core_clk);
	return ret;
}

static int vpe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vpe_dev *vpe;
	u32 version;
	int ret;

	vpe = devm_kzalloc(dev, sizeof(*vpe), GFP_KERNEL);
	if (!vpe)
		return -ENOMEM;

	vpe->dev = dev;
	platform_set_drvdata(pdev, vpe);

	mutex_init(&vpe->lock);
	spin_lock_init(&vpe->irqlock);

	/* Get resources */
	vpe->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpe->base))
		return PTR_ERR(vpe->base);

	vpe->irq = platform_get_irq(pdev, 0);
	if (vpe->irq < 0)
		return vpe->irq;

	/* Get clocks */
	vpe->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(vpe->core_clk))
		return dev_err_probe(dev, PTR_ERR(vpe->core_clk),
				     "Failed to get core clock\n");

	vpe->axi_clk = devm_clk_get(dev, "axi");
	if (IS_ERR(vpe->axi_clk))
		return dev_err_probe(dev, PTR_ERR(vpe->axi_clk),
				     "Failed to get AXI clock\n");

	vpe->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(vpe->ahb_clk))
		return dev_err_probe(dev, PTR_ERR(vpe->ahb_clk),
				     "Failed to get AHB clock\n");

	/* Get interconnect path (optional) */
	vpe->icc_path = devm_of_icc_get(dev, "vpe-mem");
	if (IS_ERR(vpe->icc_path)) {
		if (PTR_ERR(vpe->icc_path) != -ENODATA)
			return dev_err_probe(dev, PTR_ERR(vpe->icc_path),
					     "Failed to get interconnect\n");
		vpe->icc_path = NULL;
	}

	/* Enable runtime PM */
	pm_runtime_enable(dev);

	/* Check hardware version */
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto err_pm;

	version = vpe_hw_get_version(vpe->base);
	dev_info(dev, "VPE hardware version: 0x%08x\n", version);

	pm_runtime_put(dev);

	/* Request IRQ */
	ret = devm_request_irq(dev, vpe->irq, vpe_irq_handler, 0, VPE_NAME, vpe);
	if (ret) {
		dev_err(dev, "Failed to request IRQ\n");
		goto err_pm;
	}

	/* Initialize V4L2 device */
	ret = v4l2_device_register(dev, &vpe->v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register V4L2 device\n");
		goto err_pm;
	}

	/* Initialize mem2mem device */
	vpe->m2m_dev = v4l2_m2m_init(&vpe_m2m_ops);
	if (IS_ERR(vpe->m2m_dev)) {
		ret = PTR_ERR(vpe->m2m_dev);
		dev_err(dev, "Failed to init mem2mem device\n");
		goto err_v4l2;
	}

	/* Initialize video device */
	vpe->vfd.fops = &vpe_fops;
	vpe->vfd.ioctl_ops = &vpe_ioctl_ops;
	vpe->vfd.minor = -1;
	vpe->vfd.release = video_device_release_empty;
	vpe->vfd.lock = &vpe->lock;
	vpe->vfd.v4l2_dev = &vpe->v4l2_dev;
	vpe->vfd.vfl_dir = VFL_DIR_M2M;
	vpe->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	strscpy(vpe->vfd.name, VPE_NAME, sizeof(vpe->vfd.name));

	video_set_drvdata(&vpe->vfd, vpe);

	ret = video_register_device(&vpe->vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "Failed to register video device\n");
		goto err_m2m;
	}

	dev_info(dev, "Qualcomm VPE registered as /dev/video%d\n", vpe->vfd.num);

	return 0;

err_m2m:
	v4l2_m2m_release(vpe->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&vpe->v4l2_dev);
err_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void vpe_remove(struct platform_device *pdev)
{
	struct vpe_dev *vpe = platform_get_drvdata(pdev);

	video_unregister_device(&vpe->vfd);
	v4l2_m2m_release(vpe->m2m_dev);
	v4l2_device_unregister(&vpe->v4l2_dev);
	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops vpe_pm_ops = {
	RUNTIME_PM_OPS(vpe_runtime_suspend, vpe_runtime_resume, NULL)
};

static const struct of_device_id vpe_of_match[] = {
	{ .compatible = "qcom,msm8660-vpe" },
	{ .compatible = "qcom,apq8060-vpe" },
	{ }
};
MODULE_DEVICE_TABLE(of, vpe_of_match);

static struct platform_driver vpe_driver = {
	.probe		= vpe_probe,
	.remove		= vpe_remove,
	.driver		= {
		.name	= VPE_NAME,
		.pm	= pm_ptr(&vpe_pm_ops),
		.of_match_table = vpe_of_match,
	},
};

module_platform_driver(vpe_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660 Video Processing Engine driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Herrie <herrie.org>");
