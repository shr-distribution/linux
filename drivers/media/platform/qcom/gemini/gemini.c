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
#include <linux/dma-mapping.h>
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
	 * FE_BUFFER_CFG's "MCU_ROWS" fields actually carry the number of MCUs
	 * per row (the horizontal MCU count Wm), which the fetch engine uses
	 * to compute the row stride when walking the source frame. The live
	 * webOS delta-log confirms this: for a 1024x1280 (Wm=64, Hm=80) frame
	 * FE_BUFFER_CFG = 0x003f003f, i.e. Wm-1 = 63 in both fields (== the
	 * high half of FE_DIMS), NOT Hm-1 = 79. Feeding Hm here gave the FE
	 * the wrong stride, so only the first MCU row read correctly and the
	 * rest was misaligned garbage / the encode produced no output.
	 *
	 * H2V2 NV12 uses 16x16 MCUs: Wm = (W+15)>>4.
	 */
	num_mcu_rows = (ctx->src.width + 15) / 16;

	/*
	 * Cache coherency for the offline DMA path.
	 *
	 * The Gemini hardware reads the source frame from DRAM and writes
	 * the entropy-coded JPEG stream back to DRAM. APQ8060/MSM8660 is a
	 * non-coherent platform — peripheral DMA does NOT snoop the CPU
	 * caches. Without explicit syncs, the encoder reads whatever
	 * DRAM-resident bytes happen to be there (often zeros from initial
	 * allocation, with occasional cache lines that got written through
	 * the L1/L2 to DRAM by chance), not the bytes the CPU just wrote
	 * via mmap. That produces deterministic but wrong output —
	 * historically the "10-MCU band, 49% black" corruption pattern.
	 *
	 * Sync source planes for the device before kicking the encoder, so
	 * any CPU-side dirty cache lines (from userspace mmap writes +
	 * msync) are forced out to DRAM. Use DMA_TO_DEVICE because we are
	 * making CPU writes visible to the device.
	 *
	 * For the destination plane we use DMA_BIDIRECTIONAL: the CPU just
	 * wrote the JPEG header at the start of the buffer, AND the
	 * hardware will write the entropy segment to the rest. The header
	 * needs to be visible to the device; the entropy segment will
	 * later need to be visible to the CPU (handled in the IRQ handler
	 * via dma_sync_single_for_cpu before vb2_set_plane_payload).
	 */
	dma_sync_single_for_device(gemini->dev, src_y,
				   ctx->src.sizeimage, DMA_TO_DEVICE);
	dma_sync_single_for_device(gemini->dev, dst_addr,
				   ctx->dst.sizeimage, DMA_BIDIRECTIONAL);

	pr_info("gemini run: src_y=%pad src_cbcr=%pad dst=%pad src_sz=%u dst_sz=%u hdr=%zu\n",
		&src_y, &src_cbcr, &dst_addr,
		ctx->src.sizeimage, ctx->dst.sizeimage, hdr_aligned);

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

	/*
	 * Per-IRQ trace for FE->WE handoff debugging: log the status plus the
	 * running encoded-output size and FE_CMD each time. The sequence of
	 * these lines shows how many FE_RD_DONE (0x2) fire before any WE
	 * activity and whether ENCODE_OUTPUT_SIZE ever advances (i.e. whether
	 * the encoder progresses from read to write or stalls after a single
	 * chunk). FE_DIMS/PIPELINE_CFG are read back to confirm the armed
	 * geometry survived to encode time.
	 */
	pr_info("gemini IRQ: status=0x%08x osize=0x%08x fecmd=0x%08x pipe=0x%08x fedims=0x%08x\n",
		status,
		readl(gemini->base + GEMINI_ENCODE_OUTPUT_SIZE),
		readl(gemini->base + GEMINI_FE_CMD),
		readl(gemini->base + GEMINI_PIPELINE_CFG),
		readl(gemini->base + GEMINI_FE_DIMS));

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
	 * Transient IRQs (FE_RD_DONE, WE_*_PINGPONG) mean progress but no
	 * user-visible completion yet. The legacy webOS driver
	 * (msm_gemini_fe_pingpong_irq) only re-arms the FE on an FE pingpong
	 * IRQ when ANOTHER input buffer is queued, i.e. for multi-chunk input
	 * it feeds the next chunk's addresses; when the input queue is empty
	 * it does nothing and lets the encoder drain what it already read
	 * through entropy + WE to FRAMEDONE.
	 *
	 * Our M2M path hands the whole frame to the FE as a single buffer
	 * (num_mcu_rows = full height), so there is never a next chunk. Do
	 * NOT re-issue FE_CMD here: re-reading the frame mid-encode corrupts
	 * every MCU past the first and can wedge the engine (the encoder then
	 * never returns RESET_ACK on the next job). Just re-arm for the
	 * terminal IRQs and wait for FRAMEDONE.
	 */
	if (!(status & (GEMINI_IRQ_FRAMEDONE | GEMINI_IRQ_BUS_ERROR |
			GEMINI_IRQ_VIOLATION))) {
		gemini_hw_enable_irq(gemini->base, GEMINI_IRQ_FRAMEDONE |
						   GEMINI_IRQ_BUS_ERROR |
						   GEMINI_IRQ_VIOLATION);
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
			dma_addr_t dst_addr = vb2_dma_contig_plane_dma_addr(
					&dst_buf->vb2_buf, 0);
			size_t hdr_aligned = ALIGN(ctx->hdr_len, 8);
			size_t payload;

			/*
			 * The encoder wrote the entropy segment to DRAM via
			 * its own DMA path; the CPU caches for that range
			 * are stale. Invalidate them before we read /
			 * write-EOI through the vaddr mapping. See the
			 * matching dma_sync_single_for_device() in
			 * gemini_device_run() for the full rationale.
			 */
			dma_sync_single_for_cpu(gemini->dev, dst_addr,
						ctx->dst.sizeimage,
						DMA_FROM_DEVICE);

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

/* Per-table pair buffer size: AC main loop indexes up to 175 (nibble-swapped
 * huffval 0xFA -> 0xAF); 256 matches OPAL's allocation and leaves headroom. */
#define GEMINI_HUFF_PAIRS	256

static void gemini_load_tables(struct gemini_ctx *ctx)
{
	void __iomem *base = ctx->gemini->base;
	struct gemini_huff_pair *pairs;
	struct gemini_huff_pair *luma_dc, *luma_ac, *chroma_dc, *chroma_ac;

	pr_info("gemini tables: T0 enter (will load huffman then quant)\n");

	/*
	 * FOUR single-purpose, zero-initialised pair buffers: DC luma, AC
	 * luma, DC chroma, AC chroma. OPAL keeps DC and AC separate
	 * (gemini_lib_hw_set_huffman_tables(dc_luma, dc_chroma, ac_luma,
	 * ac_chroma)) — the prologue is loaded from the DC tables, the main
	 * per-symbol LUT from the AC tables. Merging them lets the AC EOB
	 * symbol (huffval 0x00) clobber DC category-0 at pair index 0, which
	 * breaks every DC-difference-of-zero coding (every block of a flat
	 * region) and produces DC-drift noise after the first MCU.
	 *
	 * Heap-allocated (4 x 256 entries) to keep this off the stack.
	 *
	 *   gemini_huff_tables[0] = DC luma   [2] = AC luma
	 *   gemini_huff_tables[1] = DC chroma [3] = AC chroma
	 *
	 * For AC tables the huffval is nibble-swapped before indexing (OPAL's
	 * gemini_lib_hw_create_huffman_table convention); see
	 * gemini_build_huff_pairs().
	 */
	pairs = kcalloc(4 * GEMINI_HUFF_PAIRS, sizeof(*pairs), GFP_KERNEL);
	if (!pairs) {
		pr_err("gemini tables: pair buffer alloc failed\n");
		return;
	}
	luma_dc   = pairs + 0 * GEMINI_HUFF_PAIRS;
	luma_ac   = pairs + 1 * GEMINI_HUFF_PAIRS;
	chroma_dc = pairs + 2 * GEMINI_HUFF_PAIRS;
	chroma_ac = pairs + 3 * GEMINI_HUFF_PAIRS;

	gemini_build_huff_pairs(luma_dc,
				gemini_huff_tables[0].bits,
				gemini_huff_tables[0].vals,
				gemini_huff_tables[0].n_vals,
				false);   /* DC luma */
	gemini_build_huff_pairs(luma_ac,
				gemini_huff_tables[2].bits,
				gemini_huff_tables[2].vals,
				gemini_huff_tables[2].n_vals,
				true);    /* AC luma */
	gemini_build_huff_pairs(chroma_dc,
				gemini_huff_tables[1].bits,
				gemini_huff_tables[1].vals,
				gemini_huff_tables[1].n_vals,
				false);   /* DC chroma */
	gemini_build_huff_pairs(chroma_ac,
				gemini_huff_tables[3].bits,
				gemini_huff_tables[3].vals,
				gemini_huff_tables[3].n_vals,
				true);    /* AC chroma */

	pr_info("gemini tables: T1 huff pairs built, loading hw\n");
	gemini_hw_load_huffman_tables(base, luma_dc, luma_ac,
				      chroma_dc, chroma_ac);
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

	kfree(pairs);
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

	/*
	 * Set up DMA parameters. Without this:
	 *   - vb2_mmap NULL-derefs on dev->dma_parms->max_segment_size
	 *   - vb2_dma_contig_plane_dma_addr may return raw CMA physical
	 *     addresses instead of IOMMU-translated iovas, which then
	 *     fault when the JPEG engine tries to read them through its
	 *     SMMU (ijpeg_iommu @ 0x7800000).
	 *
	 * Observed 2026-05-13: JPEG engine kicked off with
	 * `src_y=0x7c500000`, `we=0x7c700270` (raw CMA addresses), then
	 * the SMMU emitted "Unexpected IOMMU page fault in context 1
	 * FAR = 7c74c4f0" at ~60 kHz rate until the device locked up.
	 *
	 * 32-bit DMA mask is correct: JPEG sub-4GB addressing, CMA at
	 * 0x7c000000-0x7dffffff.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set DMA mask: %d\n", ret);
		return ret;
	}
	dev->dma_parms = devm_kzalloc(dev, sizeof(*dev->dma_parms),
				      GFP_KERNEL);
	if (!dev->dma_parms)
		return -ENOMEM;
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));

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
