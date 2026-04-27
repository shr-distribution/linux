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
#include <linux/completion.h>
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
#include "gemini_jpeg.h"

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

	/* Signaled when RESET_ACK arrives from the IRQ handler. */
	struct completion	reset_done;
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

	/* Scaled quant tables + cached header for the current frame */
	u16			q_luma[64];
	u16			q_chroma[64];
	size_t			hdr_len;
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
	struct gemini_ctx *ctx = gemini_fh_to_ctx(file->private_data);
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
	struct gemini_ctx *ctx = gemini_fh_to_ctx(file->private_data);
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
	u32 num_mcu_rows;
	u8 *dst_vaddr;
	size_t hdr_len, hdr_aligned, we_room;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	src_y = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	src_cbcr = src_y + ctx->src.bytesperline * ctx->src.height;

	dst_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	dst_vaddr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

	/*
	 * Build the JPEG marker preamble at the start of the destination
	 * buffer; the hardware will write the entropy-coded segment after
	 * it. WE_Y_PING_ADDR requires 8-byte alignment, so round the header
	 * length up; the gap will be overwritten harmlessly by the encoder
	 * when it writes the first byte of the entropy stream.
	 */
	hdr_len = gemini_build_jpeg_header(dst_vaddr,
					   ctx->src.width, ctx->src.height,
					   ctx->q_luma, ctx->q_chroma);
	hdr_aligned = ALIGN(hdr_len, 8);
	ctx->hdr_len = hdr_len;

	we_room = ctx->dst.sizeimage > hdr_aligned + 2 ?
		  ctx->dst.sizeimage - hdr_aligned - 2 : 0;

	/*
	 * H2V2 NV12 uses 16×16 MCUs, so the per-row count is height/16, not
	 * height/8. The cross-vendor libgemini wire format (Wm = (W+15)>>4,
	 * Hm = (H+15)>>4) confirms this for all four vendor binaries.
	 */
	num_mcu_rows = (ctx->src.height + 15) / 16;

	pr_info("gemini run: R0 device_run entered\n");
	gemini_hw_set_fe_ping(gemini->base, src_y, src_cbcr, num_mcu_rows);
	pr_info("gemini run: R1 set_fe_ping returned\n");

	gemini_hw_set_we_ping(gemini->base, dst_addr + hdr_aligned, we_room);
	pr_info("gemini run: R2 set_we_ping returned\n");

	gemini_hw_enable_irq(gemini->base, GEMINI_IRQ_FRAMEDONE |
					   GEMINI_IRQ_BUS_ERROR |
					   GEMINI_IRQ_VIOLATION);
	pr_info("gemini run: R3 enable_irq returned, calling start_offline\n");

	gemini_hw_start_offline(gemini->base);
	pr_info("gemini run: R4 start_offline returned\n");
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

	pr_info("gemini IRQ: status=0x%08x\n", status);

	gemini_hw_clear_irq(gemini->base, status);

	/*
	 * Only disable IRQs on terminal events. The encoder fires several
	 * transient IRQs while a single frame progresses (FE_RD_DONE,
	 * WE_Y_PINGPONG, etc.); if we mask everything on the first one,
	 * the eventual FRAMEDONE never reaches the CPU and userspace times
	 * out even though the encode succeeded.
	 */
	if (status & (GEMINI_IRQ_FRAMEDONE | GEMINI_IRQ_BUS_ERROR |
		      GEMINI_IRQ_VIOLATION | GEMINI_IRQ_RESET_ACK))
		gemini_hw_disable_irq(gemini->base);

	if (status & GEMINI_IRQ_RESET_ACK) {
		complete(&gemini->reset_done);
		return IRQ_HANDLED;
	}

	/*
	 * Transient IRQs (FE_RD_DONE, WE_*_PINGPONG, etc.) mean progress
	 * but no user-visible completion yet. Re-arm IRQ_MASK (the IP
	 * appears to auto-mask after firing) and ack. For FE_RD_DONE,
	 * also re-issue FE_CMD = OFFLINE_CMD_START — legacy webOS does
	 * this on every FE pingpong IRQ to advance the encoder. In our
	 * single-buffer M2M case the same buffer gets re-read, but the
	 * encoder needs the kick to progress past the read stage to
	 * entropy + WE.
	 */
	if (!(status & (GEMINI_IRQ_FRAMEDONE | GEMINI_IRQ_BUS_ERROR |
			GEMINI_IRQ_VIOLATION))) {
		gemini_hw_enable_irq(gemini->base, GEMINI_IRQ_FRAMEDONE |
						   GEMINI_IRQ_BUS_ERROR |
						   GEMINI_IRQ_VIOLATION |
						   GEMINI_IRQ_FE_RD_DONE |
						   GEMINI_IRQ_WE_Y_PINGPONG |
						   GEMINI_IRQ_WE_CBCR_PINGPONG);
		if (status & GEMINI_IRQ_FE_RD_DONE) {
			pr_info("gemini IRQ: FE_RD_DONE — re-issuing FE_CMD=START\n");
			gemini_hw_fe_reload(gemini->base);
			writel(GEMINI_OFFLINE_CMD_START,
			       gemini->base + GEMINI_FE_CMD);
		}
		return IRQ_HANDLED;
	}

	ctx = v4l2_m2m_get_curr_priv(gemini->m2m_dev);
	if (!ctx)
		return IRQ_HANDLED;

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (src_buf && dst_buf) {
		enum vb2_buffer_state state = VB2_BUF_STATE_DONE;

		if (status & GEMINI_IRQ_FRAMEDONE) {
			u8 *vaddr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
			size_t hdr_aligned = ALIGN(ctx->hdr_len, 8);
			size_t payload;

			output_size = gemini_hw_get_output_size(gemini->base);

			/* Append EOI right after the entropy stream */
			vaddr[hdr_aligned + output_size + 0] = 0xFF;
			vaddr[hdr_aligned + output_size + 1] = 0xD9;

			payload = hdr_aligned + output_size + 2;
			vb2_set_plane_payload(&dst_buf->vb2_buf, 0, payload);
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

static void gemini_load_tables(struct gemini_ctx *ctx)
{
	void __iomem *base = ctx->gemini->base;
	struct gemini_huff_pair luma_pairs[256] = {0};
	struct gemini_huff_pair chroma_pairs[256] = {0};

	pr_info("gemini tables: T0 enter (will load huffman then quant)\n");

	/*
	 * OPAL libgemini's gemini_lib_hw_config loads HUFFMAN BEFORE QUANT.
	 * We previously did quant first; swap to match.
	 *
	 *   gemini_huff_tables[0] = DC luma
	 *   gemini_huff_tables[1] = DC chroma
	 *   gemini_huff_tables[2] = AC luma
	 *   gemini_huff_tables[3] = AC chroma
	 *
	 * Build the per-huffval (code, size) pairs: AC table first, then DC
	 * (DC entries override AC at low huffvals, matching libqcameralib's
	 * gemini_lib_hw_set_huffman_tables behavior).
	 */
	gemini_build_huff_pairs(luma_pairs,
				gemini_huff_tables[2].bits,
				gemini_huff_tables[2].vals,
				gemini_huff_tables[2].n_vals);
	gemini_build_huff_pairs(luma_pairs,
				gemini_huff_tables[0].bits,
				gemini_huff_tables[0].vals,
				gemini_huff_tables[0].n_vals);
	gemini_build_huff_pairs(chroma_pairs,
				gemini_huff_tables[3].bits,
				gemini_huff_tables[3].vals,
				gemini_huff_tables[3].n_vals);
	gemini_build_huff_pairs(chroma_pairs,
				gemini_huff_tables[1].bits,
				gemini_huff_tables[1].vals,
				gemini_huff_tables[1].n_vals);

	pr_info("gemini tables: T1 huff pairs built, loading hw\n");
	gemini_hw_load_huffman_tables(base, luma_pairs, chroma_pairs);
	pr_info("gemini tables: T2 huffman load done\n");

	gemini_hw_load_quant_table(base, false, ctx->q_luma);
	pr_info("gemini tables: T3 quant luma done\n");

	gemini_hw_load_quant_table(base, true,  ctx->q_chroma);
	pr_info("gemini tables: T4 quant chroma done\n");

	/*
	 * OPAL's gemini_lib_hw_config issues a quant-table READBACK pass
	 * after the WRITE pass. Replicate that — without it, the encoder
	 * may use stale (bootloader-default) quant values regardless of
	 * what we wrote.
	 */
	gemini_hw_readback_quant_tables(base);
	pr_info("gemini tables: T5 quant readback done — exiting load_tables\n");
}

/*
 * Soft-reset the encoder and wait for RESET_ACK via the IRQ handler.
 *
 * Legacy webOS arms IRQ_MASK *before* writing RESET_CMD so the GIC latches
 * the RESET_ACK edge as the reset completes; userspace then waits for the
 * ack interrupt with a 500ms timeout. Without this wait, subsequent
 * register writes (FE_BUFFER_CFG, FE_*_PING_ADDR, FE_CMD) race the still-
 * in-progress reset and the encoder discards them silently — the symptom
 * is a clean hardware programming dump followed by IRQ_STATUS=0 forever.
 */
static int gemini_reset(struct gemini_dev *gemini)
{
	unsigned long timeout;

	reinit_completion(&gemini->reset_done);

	gemini_hw_clear_irq(gemini->base, GEMINI_IRQ_ALL);
	gemini_hw_enable_irq(gemini->base, GEMINI_IRQ_RESET_ACK);

	gemini_hw_reset(gemini->base);

	timeout = wait_for_completion_timeout(&gemini->reset_done,
					      msecs_to_jiffies(500));

	gemini_hw_disable_irq(gemini->base);
	gemini_hw_clear_irq(gemini->base, GEMINI_IRQ_ALL);

	if (!timeout) {
		dev_err(gemini->dev, "Timed out waiting for RESET_ACK\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int gemini_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct gemini_ctx *ctx = vb2_get_drv_priv(vq);
	struct gemini_dev *gemini = ctx->gemini;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		pr_info("gemini ss: S0 streamon, runtime resume\n");
		ret = pm_runtime_resume_and_get(gemini->dev);
		if (ret < 0)
			return ret;

		pr_info("gemini ss: S1 reset begin\n");
		ret = gemini_reset(gemini);
		if (ret) {
			dev_err(gemini->dev, "Hardware reset failed\n");
			pm_runtime_put(gemini->dev);
			return ret;
		}
		pr_info("gemini ss: S2 reset done\n");

		/*
		 * Diagnostic: dump HW reset defaults of all key registers
		 * BEFORE any writes. Compare to what we then write.
		 */
		pr_info("gemini RESET DEFAULTS:\n"
			"  PIPELINE_CFG=0x%08x\n"
			"  FE_INPUT_FMT=0x%08x  FE_DIMS=0x%08x  FE_PIPELINE_MODE=0x%08x\n"
			"  OP_ENC_MODE=0x%08x  OP_FORMAT_MAGIC=0x%08x\n"
			"  OP_GEOM[0..3]=0x%08x 0x%08x 0x%08x 0x%08x\n"
			"  OP_MATRIX[0..8]=0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n"
			"  WE_CFG=0x%08x WE_Y_UB_CFG=0x%08x  WE_Y_TH=0x%08x  WE_CBCR_TH=0x%08x\n"
			"  DRI_INTERVAL=0x%08x  IRQ_MASK=0x%08x  IRQ_STATUS=0x%08x\n",
			readl(gemini->base + GEMINI_PIPELINE_CFG),
			readl(gemini->base + GEMINI_FE_INPUT_FORMAT),
			readl(gemini->base + GEMINI_FE_DIMS),
			readl(gemini->base + GEMINI_FE_PIPELINE_MODE),
			readl(gemini->base + GEMINI_OP_ENCODE_MODE),
			readl(gemini->base + GEMINI_OP_FORMAT_MAGIC),
			readl(gemini->base + GEMINI_OP_GEOM(0)),
			readl(gemini->base + GEMINI_OP_GEOM(1)),
			readl(gemini->base + GEMINI_OP_GEOM(2)),
			readl(gemini->base + GEMINI_OP_GEOM(3)),
			readl(gemini->base + GEMINI_OP_MATRIX(0)),
			readl(gemini->base + GEMINI_OP_MATRIX(1)),
			readl(gemini->base + GEMINI_OP_MATRIX(2)),
			readl(gemini->base + GEMINI_OP_MATRIX(3)),
			readl(gemini->base + GEMINI_OP_MATRIX(4)),
			readl(gemini->base + GEMINI_OP_MATRIX(5)),
			readl(gemini->base + GEMINI_OP_MATRIX(6)),
			readl(gemini->base + GEMINI_OP_MATRIX(7)),
			readl(gemini->base + GEMINI_OP_MATRIX(8)),
			readl(gemini->base + GEMINI_WE_CFG),
			readl(gemini->base + GEMINI_WE_Y_UB_CFG),
			readl(gemini->base + GEMINI_WE_Y_THRESHOLD),
			readl(gemini->base + GEMINI_WE_CBCR_THRESHOLD),
			readl(gemini->base + GEMINI_DRI_INTERVAL),
			readl(gemini->base + GEMINI_IRQ_MASK),
			readl(gemini->base + GEMINI_IRQ_STATUS));

		/*
		 * Legacy msm_gemini_core_reset writes WE_Y_UB_CFG +
		 * WE_Y_THRESHOLD + WE_CBCR_THRESHOLD immediately after
		 * RESET, before any other configure register. The values
		 * must be valid before PIPELINE_CFG arms the offline
		 * pipeline; otherwise the WE engine is armed with zero
		 * thresholds and stalls / faults on first write.
		 */
		gemini_hw_we_post_reset_cfg(gemini->base);

		gemini_scale_quant_luma(ctx->q_luma, ctx->quality);
		gemini_scale_quant_chroma(ctx->q_chroma, ctx->quality);

		pr_info("gemini ss: S3 entering configure_encode_h2v2\n");
		gemini_hw_configure_encode_h2v2(gemini->base,
						ctx->src.width,
						ctx->src.height);
		pr_info("gemini ss: S4 entering load_tables\n");
		gemini_load_tables(ctx);
		pr_info("gemini ss: S5 streamon done\n");
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

	/*
	 * The hardware does not respond to register writes (including the
	 * reset command) at the 27 MHz default. Legacy webOS set 144 MHz
	 * via clk_set_min_rate(); the closest mainline freq_tbl entry is
	 * 153.6 MHz, which the RCG will round up to.
	 */
	ret = clk_set_rate(gemini->core_clk, 153600000);
	if (ret)
		return ret;

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
	init_completion(&gemini->reset_done);

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
