// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660 JPEG Encoder/Decoder (Gemini) V4L2 mem2mem driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024-2026 Herrie (herrie.org)
 *
 * Based on legacy msm_gemini driver from webOS kernel.
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

#include "gemini_hw.h"

#define GEMINI_NAME		"qcom-gemini"
#define GEMINI_MAX_WIDTH	8192
#define GEMINI_MAX_HEIGHT	8192
#define GEMINI_MIN_WIDTH	32
#define GEMINI_MIN_HEIGHT	32
#define GEMINI_ALIGN		16

/* Interconnect bandwidth in KB/s (~1.5 GB/s) */
#define GEMINI_ICC_AVG_BW	1521190
#define GEMINI_ICC_PEAK_BW	1521190

/* Default JPEG quality (1-100) */
#define GEMINI_DEFAULT_QUALITY	80

enum gemini_fmt_type {
	GEMINI_FMT_TYPE_OUTPUT	= BIT(0),
	GEMINI_FMT_TYPE_CAPTURE	= BIT(1),
};

struct gemini_fmt {
	u32	fourcc;
	int	depth;
	u32	types;
};

static const struct gemini_fmt gemini_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_NV12,
		.depth	= 12,
		.types	= GEMINI_FMT_TYPE_OUTPUT,
	},
	{
		.fourcc	= V4L2_PIX_FMT_JPEG,
		.depth	= 16,	/* Variable, estimate */
		.types	= GEMINI_FMT_TYPE_CAPTURE,
	},
};

struct gemini_dev {
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

struct gemini_frame {
	u32			width;
	u32			height;
	u32			bytesperline;
	u32			sizeimage;
	const struct gemini_fmt	*fmt;
};

struct gemini_ctx {
	struct v4l2_fh		fh;
	struct gemini_dev	*gemini;
	struct v4l2_ctrl_handler ctrl_handler;

	struct gemini_frame	src;
	struct gemini_frame	dst;
	int			quality;
};

static inline struct gemini_ctx *gemini_fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct gemini_ctx, fh);
}

static const struct gemini_fmt *gemini_find_format(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gemini_formats); i++) {
		if (gemini_formats[i].fourcc == fourcc)
			return &gemini_formats[i];
	}

	return NULL;
}

/*
 * V4L2 ioctl operations
 */

static int gemini_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, GEMINI_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Gemini JPEG", sizeof(cap->card));

	return 0;
}

static int gemini_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	const struct gemini_fmt *fmt;
	u32 type;
	int i, num = 0;

	type = V4L2_TYPE_IS_OUTPUT(f->type) ? GEMINI_FMT_TYPE_OUTPUT :
					      GEMINI_FMT_TYPE_CAPTURE;

	for (i = 0; i < ARRAY_SIZE(gemini_formats); i++) {
		if (gemini_formats[i].types & type) {
			if (num == f->index) {
				fmt = &gemini_formats[i];
				f->pixelformat = fmt->fourcc;
				return 0;
			}
			num++;
		}
	}

	return -EINVAL;
}

static int gemini_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct gemini_ctx *ctx = gemini_fh_to_ctx(priv);
	struct gemini_frame *frame;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	frame = V4L2_TYPE_IS_OUTPUT(f->type) ? &ctx->src : &ctx->dst;

	pix->width = frame->width;
	pix->height = frame->height;
	pix->pixelformat = frame->fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = frame->bytesperline;
	pix->sizeimage = frame->sizeimage;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int gemini_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const struct gemini_fmt *fmt;
	u32 type;

	type = V4L2_TYPE_IS_OUTPUT(f->type) ? GEMINI_FMT_TYPE_OUTPUT :
					      GEMINI_FMT_TYPE_CAPTURE;
	fmt = gemini_find_format(pix->pixelformat);

	if (!fmt || !(fmt->types & type)) {
		/* Default format based on type */
		if (type == GEMINI_FMT_TYPE_OUTPUT)
			fmt = &gemini_formats[0];	/* NV12 */
		else
			fmt = &gemini_formats[1];	/* JPEG */
	}

	pix->pixelformat = fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;

	/* Align dimensions */
	pix->width = clamp(pix->width, GEMINI_MIN_WIDTH, GEMINI_MAX_WIDTH);
	pix->height = clamp(pix->height, GEMINI_MIN_HEIGHT, GEMINI_MAX_HEIGHT);
	pix->width = ALIGN(pix->width, GEMINI_ALIGN);
	pix->height = ALIGN(pix->height, GEMINI_ALIGN);

	if (fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		pix->bytesperline = 0;
		/* Estimate JPEG size (worst case: slightly larger than raw) */
		pix->sizeimage = pix->width * pix->height * 2;
	} else {
		pix->bytesperline = pix->width;
		pix->sizeimage = pix->bytesperline * pix->height * fmt->depth / 8;
	}

	pix->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int gemini_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct gemini_ctx *ctx = gemini_fh_to_ctx(priv);
	struct gemini_frame *frame;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	ret = gemini_try_fmt(file, priv, f);
	if (ret)
		return ret;

	frame = V4L2_TYPE_IS_OUTPUT(f->type) ? &ctx->src : &ctx->dst;

	frame->width = pix->width;
	frame->height = pix->height;
	frame->bytesperline = pix->bytesperline;
	frame->sizeimage = pix->sizeimage;
	frame->fmt = gemini_find_format(pix->pixelformat);

	return 0;
}

static int gemini_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gemini_ctx *ctx = container_of(ctrl->handler,
					      struct gemini_ctx, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ctx->quality = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops gemini_ctrl_ops = {
	.s_ctrl = gemini_s_ctrl,
};

static const struct v4l2_ioctl_ops gemini_ioctl_ops = {
	.vidioc_querycap		= gemini_querycap,

	.vidioc_enum_fmt_vid_cap	= gemini_enum_fmt,
	.vidioc_enum_fmt_vid_out	= gemini_enum_fmt,

	.vidioc_g_fmt_vid_cap		= gemini_g_fmt,
	.vidioc_g_fmt_vid_out		= gemini_g_fmt,

	.vidioc_try_fmt_vid_cap		= gemini_try_fmt,
	.vidioc_try_fmt_vid_out		= gemini_try_fmt,

	.vidioc_s_fmt_vid_cap		= gemini_s_fmt,
	.vidioc_s_fmt_vid_out		= gemini_s_fmt,

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

static void gemini_device_run(void *priv)
{
	struct gemini_ctx *ctx = priv;
	struct gemini_dev *gemini = ctx->gemini;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	dma_addr_t src_y, src_cbcr, dst_addr;
	u32 y_mcu_rows, cbcr_mcu_rows;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	/* Get source buffer addresses (NV12: Y plane followed by CbCr) */
	src_y = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	src_cbcr = src_y + ctx->src.bytesperline * ctx->src.height;

	/* Get destination buffer address */
	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);

	/* Calculate MCU rows (8 pixels per MCU for YUV420) */
	y_mcu_rows = (ctx->src.height + 7) / 8;
	cbcr_mcu_rows = (ctx->src.height / 2 + 7) / 8;

	/* Configure hardware */
	gemini_hw_set_fe_ping(gemini->base, src_y, src_cbcr,
			      y_mcu_rows, cbcr_mcu_rows);
	gemini_hw_set_we_ping(gemini->base, dst_addr, ctx->dst.sizeimage);

	/* Enable frame done interrupt and start encoding */
	gemini_hw_enable_irq(gemini->base, GEMINI_IRQ_FRAMEDONE |
					   GEMINI_IRQ_BUS_ERROR);
	gemini_hw_start_offline(gemini->base);
}

static irqreturn_t gemini_irq_handler(int irq, void *dev_id)
{
	struct gemini_dev *gemini = dev_id;
	struct gemini_ctx *ctx;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	u32 status, output_size;

	status = gemini_hw_get_irq_status(gemini->base);
	if (!status)
		return IRQ_NONE;

	gemini_hw_clear_irq(gemini->base, status);
	gemini_hw_disable_irq(gemini->base);

	ctx = v4l2_m2m_get_curr_priv(gemini->m2m_dev);
	if (!ctx)
		return IRQ_HANDLED;

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (src_buf && dst_buf) {
		enum vb2_buffer_state state = VB2_BUF_STATE_DONE;

		if (status & GEMINI_IRQ_FRAMEDONE) {
			/* Get actual JPEG output size */
			output_size = gemini_hw_get_output_size(gemini->base);
			vb2_set_plane_payload(&dst_buf->vb2_buf, 0, output_size);
		} else if (status & (GEMINI_IRQ_BUS_ERROR | GEMINI_IRQ_VIOLATION)) {
			dev_err(gemini->dev, "JPEG encoding error: 0x%08x\n", status);
			state = VB2_BUF_STATE_ERROR;
		}

		dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
		dst_buf->timecode = src_buf->timecode;
		dst_buf->flags = src_buf->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

		v4l2_m2m_buf_done(src_buf, state);
		v4l2_m2m_buf_done(dst_buf, state);
	}

	v4l2_m2m_job_finish(gemini->m2m_dev, ctx->fh.m2m_ctx);

	return IRQ_HANDLED;
}

static int gemini_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct gemini_ctx *ctx = vb2_get_drv_priv(vq);
	struct gemini_frame *frame;

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

static int gemini_buf_prepare(struct vb2_buffer *vb)
{
	struct gemini_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct gemini_frame *frame;

	frame = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ? &ctx->src : &ctx->dst;

	if (vb2_plane_size(vb, 0) < frame->sizeimage) {
		dev_err(ctx->gemini->dev, "Buffer too small (%lu < %u)\n",
			vb2_plane_size(vb, 0), frame->sizeimage);
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, frame->sizeimage);

	return 0;
}

static void gemini_buf_queue(struct vb2_buffer *vb)
{
	struct gemini_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int gemini_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct gemini_ctx *ctx = vb2_get_drv_priv(vq);
	struct gemini_dev *gemini = ctx->gemini;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		ret = pm_runtime_resume_and_get(gemini->dev);
		if (ret < 0)
			return ret;

		ret = gemini_hw_reset(gemini->base);
		if (ret) {
			dev_err(gemini->dev, "Hardware reset failed\n");
			pm_runtime_put(gemini->dev);
			return ret;
		}
	}

	return 0;
}

static void gemini_stop_streaming(struct vb2_queue *vq)
{
	struct gemini_ctx *ctx = vb2_get_drv_priv(vq);
	struct gemini_dev *gemini = ctx->gemini;
	struct vb2_v4l2_buffer *buf;

	/* Return all buffers */
	while ((buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	while ((buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		pm_runtime_put(gemini->dev);
}

static const struct vb2_ops gemini_qops = {
	.queue_setup		= gemini_queue_setup,
	.buf_prepare		= gemini_buf_prepare,
	.buf_queue		= gemini_buf_queue,
	.start_streaming	= gemini_start_streaming,
	.stop_streaming		= gemini_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int gemini_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct gemini_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &gemini_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->gemini->lock;
	src_vq->dev = ctx->gemini->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &gemini_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->gemini->lock;
	dst_vq->dev = ctx->gemini->dev;

	return vb2_queue_init(dst_vq);
}

static const struct v4l2_m2m_ops gemini_m2m_ops = {
	.device_run	= gemini_device_run,
};

/*
 * File operations
 */

static int gemini_open(struct file *file)
{
	struct gemini_dev *gemini = video_drvdata(file);
	struct gemini_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->gemini = gemini;
	v4l2_fh_init(&ctx->fh, &gemini->vfd);
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(gemini->m2m_dev, ctx,
					    gemini_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_fh;
	}

	/* Initialize control handler */
	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &gemini_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY, 1, 100, 1,
			  GEMINI_DEFAULT_QUALITY);

	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto err_ctrl;
	}

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	v4l2_fh_add(&ctx->fh, file);

	/* Set default format */
	ctx->src.width = GEMINI_MIN_WIDTH;
	ctx->src.height = GEMINI_MIN_HEIGHT;
	ctx->src.bytesperline = GEMINI_MIN_WIDTH;
	ctx->src.sizeimage = GEMINI_MIN_WIDTH * GEMINI_MIN_HEIGHT * 3 / 2;
	ctx->src.fmt = &gemini_formats[0];	/* NV12 */

	ctx->dst.width = GEMINI_MIN_WIDTH;
	ctx->dst.height = GEMINI_MIN_HEIGHT;
	ctx->dst.bytesperline = 0;
	ctx->dst.sizeimage = GEMINI_MIN_WIDTH * GEMINI_MIN_HEIGHT * 2;
	ctx->dst.fmt = &gemini_formats[1];	/* JPEG */

	ctx->quality = GEMINI_DEFAULT_QUALITY;

	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int gemini_release(struct file *file)
{
	struct gemini_ctx *ctx = gemini_fh_to_ctx(file->private_data);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations gemini_fops = {
	.owner		= THIS_MODULE,
	.open		= gemini_open,
	.release	= gemini_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/*
 * Platform driver
 */

static int gemini_runtime_suspend(struct device *dev)
{
	struct gemini_dev *gemini = dev_get_drvdata(dev);

	icc_set_bw(gemini->icc_path, 0, 0);

	clk_disable_unprepare(gemini->ahb_clk);
	clk_disable_unprepare(gemini->axi_clk);
	clk_disable_unprepare(gemini->core_clk);

	return 0;
}

static int gemini_runtime_resume(struct device *dev)
{
	struct gemini_dev *gemini = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(gemini->core_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(gemini->axi_clk);
	if (ret)
		goto err_core;

	ret = clk_prepare_enable(gemini->ahb_clk);
	if (ret)
		goto err_axi;

	icc_set_bw(gemini->icc_path, GEMINI_ICC_AVG_BW, GEMINI_ICC_PEAK_BW);

	return 0;

err_axi:
	clk_disable_unprepare(gemini->axi_clk);
err_core:
	clk_disable_unprepare(gemini->core_clk);
	return ret;
}

static int gemini_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gemini_dev *gemini;
	int ret;

	gemini = devm_kzalloc(dev, sizeof(*gemini), GFP_KERNEL);
	if (!gemini)
		return -ENOMEM;

	gemini->dev = dev;
	platform_set_drvdata(pdev, gemini);

	mutex_init(&gemini->lock);
	spin_lock_init(&gemini->irqlock);

	/* Get resources */
	gemini->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gemini->base))
		return PTR_ERR(gemini->base);

	gemini->irq = platform_get_irq(pdev, 0);
	if (gemini->irq < 0)
		return gemini->irq;

	/* Get clocks */
	gemini->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(gemini->core_clk))
		return dev_err_probe(dev, PTR_ERR(gemini->core_clk),
				     "Failed to get core clock\n");

	gemini->axi_clk = devm_clk_get(dev, "axi");
	if (IS_ERR(gemini->axi_clk))
		return dev_err_probe(dev, PTR_ERR(gemini->axi_clk),
				     "Failed to get AXI clock\n");

	gemini->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(gemini->ahb_clk))
		return dev_err_probe(dev, PTR_ERR(gemini->ahb_clk),
				     "Failed to get AHB clock\n");

	/* Get interconnect path (optional) */
	gemini->icc_path = devm_of_icc_get(dev, "jpeg-mem");
	if (IS_ERR(gemini->icc_path)) {
		if (PTR_ERR(gemini->icc_path) != -ENODATA)
			return dev_err_probe(dev, PTR_ERR(gemini->icc_path),
					     "Failed to get interconnect\n");
		gemini->icc_path = NULL;
	}

	/* Enable runtime PM */
	pm_runtime_enable(dev);

	/* Request IRQ */
	ret = devm_request_irq(dev, gemini->irq, gemini_irq_handler, 0,
			       GEMINI_NAME, gemini);
	if (ret) {
		dev_err(dev, "Failed to request IRQ\n");
		goto err_pm;
	}

	/* Initialize V4L2 device */
	ret = v4l2_device_register(dev, &gemini->v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register V4L2 device\n");
		goto err_pm;
	}

	/* Initialize mem2mem device */
	gemini->m2m_dev = v4l2_m2m_init(&gemini_m2m_ops);
	if (IS_ERR(gemini->m2m_dev)) {
		ret = PTR_ERR(gemini->m2m_dev);
		dev_err(dev, "Failed to init mem2mem device\n");
		goto err_v4l2;
	}

	/* Initialize video device */
	gemini->vfd.fops = &gemini_fops;
	gemini->vfd.ioctl_ops = &gemini_ioctl_ops;
	gemini->vfd.minor = -1;
	gemini->vfd.release = video_device_release_empty;
	gemini->vfd.lock = &gemini->lock;
	gemini->vfd.v4l2_dev = &gemini->v4l2_dev;
	gemini->vfd.vfl_dir = VFL_DIR_M2M;
	gemini->vfd.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	strscpy(gemini->vfd.name, GEMINI_NAME, sizeof(gemini->vfd.name));

	video_set_drvdata(&gemini->vfd, gemini);

	ret = video_register_device(&gemini->vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "Failed to register video device\n");
		goto err_m2m;
	}

	dev_info(dev, "Qualcomm Gemini JPEG registered as /dev/video%d\n",
		 gemini->vfd.num);

	return 0;

err_m2m:
	v4l2_m2m_release(gemini->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&gemini->v4l2_dev);
err_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void gemini_remove(struct platform_device *pdev)
{
	struct gemini_dev *gemini = platform_get_drvdata(pdev);

	video_unregister_device(&gemini->vfd);
	v4l2_m2m_release(gemini->m2m_dev);
	v4l2_device_unregister(&gemini->v4l2_dev);
	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops gemini_pm_ops = {
	RUNTIME_PM_OPS(gemini_runtime_suspend, gemini_runtime_resume, NULL)
};

static const struct of_device_id gemini_of_match[] = {
	{ .compatible = "qcom,msm8660-gemini" },
	{ .compatible = "qcom,apq8060-gemini" },
	{ }
};
MODULE_DEVICE_TABLE(of, gemini_of_match);

static struct platform_driver gemini_driver = {
	.probe		= gemini_probe,
	.remove		= gemini_remove,
	.driver		= {
		.name	= GEMINI_NAME,
		.pm	= pm_ptr(&gemini_pm_ops),
		.of_match_table = gemini_of_match,
	},
};

module_platform_driver(gemini_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660 Gemini JPEG Encoder driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Herrie <herrie.org>");
