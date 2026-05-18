// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dma/qcom_adm.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "../dmaengine.h"
#include "../virt-dma.h"

/* ADM registers - calculated from channel number and security domain */
#define ADM_CHAN_MULTI			0x4
#define ADM_CI_MULTI			0x4
#define ADM_CRCI_MULTI			0x4
#define ADM_EE_MULTI			0x800
#define ADM_CHAN_OFFS(chan)		(ADM_CHAN_MULTI * (chan))
#define ADM_EE_OFFS(ee)			(ADM_EE_MULTI * (ee))
#define ADM_CHAN_EE_OFFS(chan, ee)	(ADM_CHAN_OFFS(chan) + ADM_EE_OFFS(ee))
#define ADM_CHAN_OFFS(chan)		(ADM_CHAN_MULTI * (chan))
#define ADM_CI_OFFS(ci)			(ADM_CHAN_OFF(ci))
#define ADM_CH_CMD_PTR(chan, ee)	(ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_RSLT(chan, ee)		(0x40 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_FLUSH_STATE0(chan, ee)	(0x80 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_STATUS_SD(chan, ee)	(0x200 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_CONF(chan, ee)		(0x240 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_RSLT_CONF(chan, ee)	(0x300 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_SEC_DOMAIN_IRQ_STATUS(ee)	(0x380 + ADM_EE_OFFS(ee))
#define ADM_CI_CONF(ci)			(0x390 + (ci) * ADM_CI_MULTI)
#define ADM_GP_CTL			0x3d8
#define ADM_CRCI_CTL(crci, ee)		(0x400 + (crci) * ADM_CRCI_MULTI + \
						ADM_EE_OFFS(ee))

/* channel status */
#define ADM_CH_STATUS_VALID		BIT(1)

/* channel result */
#define ADM_CH_RSLT_VALID		BIT(31)
#define ADM_CH_RSLT_ERR			BIT(3)
#define ADM_CH_RSLT_FLUSH		BIT(2)
#define ADM_CH_RSLT_TPD			BIT(1)

/* channel conf */
#define ADM_CH_CONF_SHADOW_EN		BIT(12)
#define ADM_CH_CONF_MPU_DISABLE		BIT(11)
#define ADM_CH_CONF_PERM_MPU_CONF	BIT(9)
#define ADM_CH_CONF_FORCE_RSLT_EN	BIT(7)
#define ADM_CH_CONF_IRQ_EN		BIT(6)
#define ADM_CH_CONF_SEC_DOMAIN(ee)	((((ee) & 0x3) << 4) | (((ee) & 0x4) << 11))

/* channel result conf */
#define ADM_CH_RSLT_CONF_FLUSH_EN	BIT(1)
#define ADM_CH_RSLT_CONF_IRQ_EN		BIT(0)

/* CRCI CTL */
#define ADM_CRCI_CTL_MUX_SEL		BIT(18)
#define ADM_CRCI_CTL_RST		BIT(17)

/* CI configuration */
#define ADM_CI_RANGE_END(x)		((x) << 24)
#define ADM_CI_RANGE_START(x)		((x) << 16)
#define ADM_CI_BURST_4_WORDS		BIT(2)
#define ADM_CI_BURST_8_WORDS		BIT(3)

/* GP CTL */
#define ADM_GP_CTL_LP_EN		BIT(12)
#define ADM_GP_CTL_LP_CNT(x)		((x) << 8)

/* Command pointer list entry */
#define ADM_CPLE_LP			BIT(31)
#define ADM_CPLE_CMD_PTR_LIST		BIT(29)

/* Command list entry */
#define ADM_CMD_LC			BIT(31)
#define ADM_CMD_DST_CRCI(n)		(((n) & 0xf) << 7)
#define ADM_CMD_SRC_CRCI(n)		(((n) & 0xf) << 3)

/*
 * In-flight byte/short/word swap bits on the SRC or DST side of each
 * BOX/SINGLE command.  The hardware reorders bytes on the peripheral
 * port during the DMA transfer, per CRCI handshake.
 */
#define ADM_CMD_SRC_SWAP_BYTES		BIT(11)
#define ADM_CMD_SRC_SWAP_SHORTS		BIT(12)
#define ADM_CMD_SRC_SWAP_WORDS		BIT(13)
#define ADM_CMD_DST_SWAP_BYTES		BIT(14)
#define ADM_CMD_DST_SWAP_SHORTS		BIT(15)
#define ADM_CMD_DST_SWAP_WORDS		BIT(16)

#define ADM_CMD_TYPE_SINGLE		0x0
#define ADM_CMD_TYPE_BOX		0x3

#define ADM_CRCI_MUX_SEL		BIT(4)
#define ADM_DESC_ALIGN			8
#define ADM_MAX_XFER			(SZ_64K - 1)
#define ADM_MAX_ROWS			(SZ_64K - 1)
#define ADM_MAX_CHANNELS		16

/* Descriptor pool configuration for reduced allocation overhead */
#define ADM_MAX_SG_PER_DESC		64	/* Max SG entries per pooled desc */
#define ADM_CPL_BUF_SIZE		2048	/* CPL buffer size (64 box descs) */
#define ADM_DESC_POOL_SIZE		8	/* Descriptors per channel */
#define ADM_TOTAL_DESC_POOL		(ADM_MAX_CHANNELS * ADM_DESC_POOL_SIZE)

struct adm_desc_hw_box {
	u32 cmd;
	u32 src_addr;
	u32 dst_addr;
	u32 row_len;
	u32 num_rows;
	u32 row_offset;
};

struct adm_desc_hw_single {
	u32 cmd;
	u32 src_addr;
	u32 dst_addr;
	u32 len;
};

struct adm_async_desc {
	struct virt_dma_desc vd;
	struct adm_device *adev;

	size_t length;
	enum dma_transfer_direction dir;
	dma_addr_t dma_addr;
	size_t dma_len;

	void *cpl;
	dma_addr_t cp_addr;
	u32 crci;
	u32 mux;
	u32 blk_size;

	/*
	 * Peripheral pre-submit hook (legacy webOS msm_dmov exec_func
	 * pattern). Called under adev->submit_lock right before the
	 * channel's CMD_PTR write in adm_start_dma.
	 */
	void (*exec_func)(void *exec_user);
	void *exec_user;

	/*
	 * Per-completion fields populated by the hardirq and consumed
	 * by the completion tasklet. last_result holds the value the
	 * hardirq read from ADM_CH_RSLT; completed_node lets the desc
	 * be queued on adm_chan.pending_completed while it waits for
	 * tasklet processing. vd.node is unused at this point because
	 * adm_start_dma list_del'd it when the descriptor became
	 * curr_txd, so it remains free for vchan_cookie_complete in
	 * the tasklet.
	 */
	u32 last_result;
	struct list_head completed_node;

	/* Pool management */
	struct list_head pool_node;
	int pool_index;		/* -1 = dynamic alloc, >=0 = pooled */
};

struct adm_chan {
	struct virt_dma_chan vc;
	struct adm_device *adev;

	/* parsed from DT */
	u32 id;			/* channel id */

	struct adm_async_desc *curr_txd;
	struct dma_slave_config slave;
	u32 crci;
	u32 mux;
	u32 cmd_flags;	/* QCOM_ADM_CMD_FLAG_* from peripheral_config */

	/*
	 * Per-channel exec_func, set via adm_slave_config from
	 * struct qcom_adm_peripheral_config. Inherited by descriptors
	 * prepared on this channel.
	 */
	void (*exec_func)(void *exec_user);
	void *exec_user;

	/*
	 * Per-channel queue of completed descriptors waiting for
	 * tasklet processing. The hardirq reads CH_RSLT (acks the
	 * device IRQ), captures curr_txd along with its result, and
	 * appends to this queue. The tasklet drains the queue,
	 * calling vchan_cookie_complete + adm_start_dma for each.
	 *
	 * A queue (not a single slot) is required because under
	 * heavy I/O the hardware can fire several completions on the
	 * same channel before the softirq tasklet gets to run; a
	 * single-slot stash would lose the earlier ones and the
	 * peripheral driver would never see those cookies marked
	 * complete (mmci then waits indefinitely -> rootfs cascade).
	 *
	 * Protected by achan->vc.lock.
	 */
	struct list_head pending_completed;

	struct list_head node;

	int error;
	int initialized;
};

static inline struct adm_chan *to_adm_chan(struct dma_chan *common)
{
	return container_of(common, struct adm_chan, vc.chan);
}

struct adm_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct device_dma_parameters dma_parms;
	struct adm_chan *channels;

	u32 ee;

	struct clk *core_clk;
	struct clk *iface_clk;

	struct reset_control *clk_reset;
	struct reset_control *c0_reset;
	struct reset_control *c1_reset;
	struct reset_control *c2_reset;
	struct icc_path *icc_path;
	int irq;

	/* Descriptor pool for reduced per-transfer allocation overhead */
	struct adm_async_desc *desc_pool;	/* Pre-allocated descriptors */
	void *cpl_pool_virt;			/* Coherent CPL buffer pool */
	dma_addr_t cpl_pool_dma;		/* DMA address of CPL pool */
	struct list_head desc_free_list;	/* Free descriptor list */
	spinlock_t pool_lock;			/* Protects free list */

	/*
	 * Per-controller submit serialization. Held across the
	 * peripheral exec_func call and the CMD_PTR write in
	 * adm_start_dma, so that submissions from different channels
	 * on the same ADM controller don't interleave their SDCC/MMIO
	 * setup with each other's ADM CMD_PTR write -- matching the
	 * legacy webOS msm_dmov per-ADM spinlock behaviour.
	 */
	spinlock_t submit_lock;

	/*
	 * Completion tasklet. The hardirq stays lean (read
	 * SD_IRQ_STATUS, read each completed channel's CH_RSLT to
	 * ack at the device, stash result, schedule this tasklet);
	 * the heavy per-channel work -- error logging, vchan
	 * completion, kicking off the next descriptor via
	 * adm_start_dma -- runs in softirq context where it can't
	 * delay other GIC hardirqs. Matches legacy msm_dmov's
	 * "hardirq calls complete_func which schedules a tasklet"
	 * pattern: keeps the ADM IRQ off the critical path of
	 * concurrent SDCC PIO refill IRQs on the same GIC.
	 */
	struct tasklet_struct completion_tasklet;
	unsigned long pending_completion;	/* bit per chan needing work */
};

/**
 * adm_free_chan - Frees dma resources associated with the specific channel
 *
 * @chan: dma channel
 *
 * Free all allocated descriptors associated with this channel
 */
static void adm_free_chan(struct dma_chan *chan)
{
	/* free all queued descriptors */
	vchan_free_chan_resources(to_virt_chan(chan));
}

/**
 * adm_desc_pool_init - Initialize pre-allocated descriptor pool
 * @adev: ADM device
 *
 * Allocates coherent DMA memory for command lists and descriptor structures
 * to eliminate per-transfer allocation overhead.
 */
static int adm_desc_pool_init(struct adm_device *adev)
{
	size_t cpl_pool_size = ADM_TOTAL_DESC_POOL * ADM_CPL_BUF_SIZE;
	int i;

	/* Allocate coherent memory for all CPL buffers */
	adev->cpl_pool_virt = dma_alloc_coherent(adev->dev, cpl_pool_size,
						 &adev->cpl_pool_dma, GFP_KERNEL);
	if (!adev->cpl_pool_virt)
		return -ENOMEM;

	/* Allocate descriptor structures */
	adev->desc_pool = kcalloc(ADM_TOTAL_DESC_POOL,
				  sizeof(struct adm_async_desc), GFP_KERNEL);
	if (!adev->desc_pool) {
		dma_free_coherent(adev->dev, cpl_pool_size,
				  adev->cpl_pool_virt, adev->cpl_pool_dma);
		adev->cpl_pool_virt = NULL;
		return -ENOMEM;
	}

	/* Initialize free list and spinlock */
	INIT_LIST_HEAD(&adev->desc_free_list);
	spin_lock_init(&adev->pool_lock);

	/* Link descriptors to CPL buffers and add to free list */
	for (i = 0; i < ADM_TOTAL_DESC_POOL; i++) {
		struct adm_async_desc *desc = &adev->desc_pool[i];

		desc->adev = adev;
		desc->pool_index = i;
		desc->cpl = adev->cpl_pool_virt + (i * ADM_CPL_BUF_SIZE);
		desc->dma_addr = adev->cpl_pool_dma + (i * ADM_CPL_BUF_SIZE);
		INIT_LIST_HEAD(&desc->pool_node);
		INIT_LIST_HEAD(&desc->completed_node);
		list_add_tail(&desc->pool_node, &adev->desc_free_list);
	}

	dev_dbg(adev->dev, "ADM descriptor pool: %d descs, %zu KB coherent\n",
		ADM_TOTAL_DESC_POOL, cpl_pool_size / 1024);

	return 0;
}

/**
 * adm_desc_pool_destroy - Free descriptor pool resources
 * @adev: ADM device
 */
static void adm_desc_pool_destroy(struct adm_device *adev)
{
	size_t cpl_pool_size = ADM_TOTAL_DESC_POOL * ADM_CPL_BUF_SIZE;

	kfree(adev->desc_pool);
	adev->desc_pool = NULL;

	if (adev->cpl_pool_virt) {
		dma_free_coherent(adev->dev, cpl_pool_size,
				  adev->cpl_pool_virt, adev->cpl_pool_dma);
		adev->cpl_pool_virt = NULL;
	}
}

/**
 * adm_desc_get - Get a descriptor from the pool
 * @adev: ADM device
 *
 * Returns a pre-allocated descriptor or NULL if pool is exhausted.
 */
static struct adm_async_desc *adm_desc_get(struct adm_device *adev)
{
	struct adm_async_desc *desc = NULL;
	unsigned long flags;

	spin_lock_irqsave(&adev->pool_lock, flags);
	if (!list_empty(&adev->desc_free_list)) {
		desc = list_first_entry(&adev->desc_free_list,
					struct adm_async_desc, pool_node);
		list_del_init(&desc->pool_node);
	}
	spin_unlock_irqrestore(&adev->pool_lock, flags);

	if (desc) {
		/* Reset descriptor state for reuse */
		memset(&desc->vd, 0, sizeof(desc->vd));
		desc->length = 0;
		desc->dma_len = 0;
		desc->crci = 0;
		desc->mux = 0;
		desc->blk_size = 0;
	}

	return desc;
}

/**
 * adm_desc_put - Return a descriptor to the pool
 * @desc: Descriptor to return
 */
static void adm_desc_put(struct adm_async_desc *desc)
{
	struct adm_device *adev = desc->adev;
	unsigned long flags;

	spin_lock_irqsave(&adev->pool_lock, flags);
	list_add_tail(&desc->pool_node, &adev->desc_free_list);
	spin_unlock_irqrestore(&adev->pool_lock, flags);
}

/**
 * adm_desc_alloc_fallback - Dynamic allocation when pool exhausted or oversized
 * @adev: ADM device
 * @cpl_size: Required CPL buffer size
 *
 * Falls back to dynamic allocation for transfers that exceed pool capacity.
 */
static struct adm_async_desc *adm_desc_alloc_fallback(struct adm_device *adev,
						      size_t cpl_size)
{
	struct adm_async_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->cpl = dma_alloc_coherent(adev->dev, cpl_size,
				       &desc->dma_addr, GFP_NOWAIT);
	if (!desc->cpl) {
		kfree(desc);
		return NULL;
	}

	desc->adev = adev;
	desc->pool_index = -1;	/* Mark as dynamic allocation */
	desc->dma_len = cpl_size;
	INIT_LIST_HEAD(&desc->pool_node);
	INIT_LIST_HEAD(&desc->completed_node);

	return desc;
}

/**
 * adm_get_blksize - Get block size from burst value
 *
 * @burst: Burst size of transaction
 */
static int adm_get_blksize(unsigned int burst)
{
	int ret;

	/*
	 * Burst is in bytes. Map to ADM block size encoding:
	 * 8 bytes -> 1 (MMCI qcom variant uses burst 8 for src_maxburst)
	 * 16 bytes -> 0, 32 bytes -> 1, 64 bytes -> 2, 128 bytes -> 3
	 * 192 bytes -> 4, 256 bytes -> 5
	 */
	switch (burst) {
	case 8:
		/* MMCI qcom variant uses src_maxburst=8 words * addr_width=4 = 32 bytes
		 * which results in burst=32, but some paths may send burst=8 bytes */
		ret = 1;
		break;
	case 16:
	case 32:
	case 64:
	case 128:
		ret = ffs(burst >> 4) - 1;
		break;
	case 192:
		ret = 4;
		break;
	case 256:
		ret = 5;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * adm_process_fc_descriptors - Process descriptors for flow controlled xfers
 *
 * @achan: ADM channel
 * @desc: Descriptor memory pointer
 * @sg: Scatterlist entry
 * @crci: CRCI value
 * @burst: Burst size of transaction
 * @direction: DMA transfer direction
 */
static void *adm_process_fc_descriptors(struct adm_chan *achan, void *desc,
					struct scatterlist *sg, u32 crci,
					u32 burst,
					enum dma_transfer_direction direction)
{
	struct adm_desc_hw_box *box_desc = NULL;
	struct adm_desc_hw_single *single_desc;
	u32 remainder = sg_dma_len(sg);
	u32 rows, row_offset, crci_cmd;
	u32 mem_addr = sg_dma_address(sg);
	u32 *incr_addr = &mem_addr;
	u32 *src, *dst;

	if (direction == DMA_DEV_TO_MEM) {
		crci_cmd = ADM_CMD_SRC_CRCI(crci);
		row_offset = burst;
		src = &achan->slave.src_addr;
		dst = &mem_addr;
		/*
		 * Data flows peripheral -> memory; the peripheral is the
		 * source.  Map the direction-agnostic peripheral_config
		 * swap flags onto the SRC swap bits so the ADM reorders
		 * bytes as they leave the peripheral.
		 */
		if (achan->cmd_flags & QCOM_ADM_CMD_FLAG_SWAP_BYTES)
			crci_cmd |= ADM_CMD_SRC_SWAP_BYTES;
		if (achan->cmd_flags & QCOM_ADM_CMD_FLAG_SWAP_SHORTS)
			crci_cmd |= ADM_CMD_SRC_SWAP_SHORTS;
		if (achan->cmd_flags & QCOM_ADM_CMD_FLAG_SWAP_WORDS)
			crci_cmd |= ADM_CMD_SRC_SWAP_WORDS;
	} else {
		crci_cmd = ADM_CMD_DST_CRCI(crci);
		row_offset = burst << 16;
		src = &mem_addr;
		dst = &achan->slave.dst_addr;
		/* Data flows memory -> peripheral; the peripheral is the
		 * destination.  Map the swap flags onto the DST swap bits.
		 */
		if (achan->cmd_flags & QCOM_ADM_CMD_FLAG_SWAP_BYTES)
			crci_cmd |= ADM_CMD_DST_SWAP_BYTES;
		if (achan->cmd_flags & QCOM_ADM_CMD_FLAG_SWAP_SHORTS)
			crci_cmd |= ADM_CMD_DST_SWAP_SHORTS;
		if (achan->cmd_flags & QCOM_ADM_CMD_FLAG_SWAP_WORDS)
			crci_cmd |= ADM_CMD_DST_SWAP_WORDS;
	}

	while (remainder >= burst) {
		box_desc = desc;
		box_desc->cmd = ADM_CMD_TYPE_BOX | crci_cmd;
		box_desc->row_offset = row_offset;
		box_desc->src_addr = *src;
		box_desc->dst_addr = *dst;

		rows = remainder / burst;
		rows = min_t(u32, rows, ADM_MAX_ROWS);
		box_desc->num_rows = rows << 16 | rows;
		box_desc->row_len = burst << 16 | burst;

		*incr_addr += burst * rows;
		remainder -= burst * rows;
		desc += sizeof(*box_desc);
	}

	/* if leftover bytes, do one single descriptor */
	if (remainder) {
		single_desc = desc;
		single_desc->cmd = ADM_CMD_TYPE_SINGLE | crci_cmd;
		single_desc->len = remainder;
		single_desc->src_addr = *src;
		single_desc->dst_addr = *dst;
		desc += sizeof(*single_desc);

		if (sg_is_last(sg))
			single_desc->cmd |= ADM_CMD_LC;
	} else {
		if (box_desc && sg_is_last(sg))
			box_desc->cmd |= ADM_CMD_LC;
	}

	return desc;
}

/**
 * adm_process_non_fc_descriptors - Process descriptors for non-fc xfers
 *
 * @achan: ADM channel
 * @desc: Descriptor memory pointer
 * @sg: Scatterlist entry
 * @direction: DMA transfer direction
 */
static void *adm_process_non_fc_descriptors(struct adm_chan *achan, void *desc,
					    struct scatterlist *sg,
					    enum dma_transfer_direction direction)
{
	struct adm_desc_hw_single *single_desc;
	u32 remainder = sg_dma_len(sg);
	u32 mem_addr = sg_dma_address(sg);
	u32 *incr_addr = &mem_addr;
	u32 *src, *dst;

	if (direction == DMA_DEV_TO_MEM) {
		src = &achan->slave.src_addr;
		dst = &mem_addr;
	} else {
		src = &mem_addr;
		dst = &achan->slave.dst_addr;
	}

	do {
		single_desc = desc;
		single_desc->cmd = ADM_CMD_TYPE_SINGLE;
		single_desc->src_addr = *src;
		single_desc->dst_addr = *dst;
		single_desc->len = (remainder > ADM_MAX_XFER) ?
				ADM_MAX_XFER : remainder;

		remainder -= single_desc->len;
		*incr_addr += single_desc->len;
		desc += sizeof(*single_desc);
	} while (remainder);

	/* set last command if this is the end of the whole transaction */
	if (sg_is_last(sg))
		single_desc->cmd |= ADM_CMD_LC;

	return desc;
}

/**
 * adm_prep_slave_sg - Prep slave sg transaction
 *
 * @chan: dma channel
 * @sgl: scatter gather list
 * @sg_len: length of sg
 * @direction: DMA transfer direction
 * @flags: DMA flags
 * @context: transfer context (unused)
 */
static struct dma_async_tx_descriptor *adm_prep_slave_sg(struct dma_chan *chan,
							 struct scatterlist *sgl,
							 unsigned int sg_len,
							 enum dma_transfer_direction direction,
							 unsigned long flags,
							 void *context)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct adm_device *adev = achan->adev;
	struct adm_async_desc *async_desc;
	struct scatterlist *sg;
	u32 i, burst;
	u32 single_count = 0, box_count = 0, crci = 0;
	void *desc;
	u32 *cple;
	int blk_size = 0;

	if (!is_slave_direction(direction)) {
		dev_err(adev->dev, "invalid dma direction\n");
		return NULL;
	}

	/*
	 * Get burst value from slave configuration and convert to bytes.
	 * The DMA slave config specifies maxburst in units of addr_width,
	 * but ADM box descriptors need the burst size in bytes.
	 */
	if (direction == DMA_MEM_TO_DEV) {
		burst = achan->slave.dst_maxburst * achan->slave.dst_addr_width;
	} else {
		burst = achan->slave.src_maxburst * achan->slave.src_addr_width;
	}

	/* if using flow control, validate burst and crci values */
	if (achan->slave.device_fc) {
		blk_size = adm_get_blksize(burst);
		if (blk_size < 0) {
			dev_err(adev->dev, "invalid burst value: %d\n",
				burst);
			return NULL;
		}

		crci = achan->crci & 0xf;
		if (!crci || achan->crci > 0x1f) {
			dev_err(adev->dev, "invalid crci value\n");
			return NULL;
		}
	}

	/* iterate through sgs and compute allocation size of structures */
	for_each_sg(sgl, sg, sg_len, i) {
		if (achan->slave.device_fc) {
			box_count += DIV_ROUND_UP(sg_dma_len(sg) / burst,
						  ADM_MAX_ROWS);
			if (sg_dma_len(sg) % burst)
				single_count++;
		} else {
			single_count += DIV_ROUND_UP(sg_dma_len(sg),
						     ADM_MAX_XFER);
		}
	}

	/* Calculate required CPL buffer size */
	{
		size_t required_cpl_size;

		required_cpl_size = single_count * sizeof(struct adm_desc_hw_single) +
				    box_count * sizeof(struct adm_desc_hw_box) +
				    sizeof(*cple) + 2 * ADM_DESC_ALIGN;

		/* Try to get descriptor from pool if it fits */
		if (required_cpl_size <= ADM_CPL_BUF_SIZE) {
			async_desc = adm_desc_get(adev);
			if (async_desc)
				async_desc->dma_len = required_cpl_size;
		} else {
			async_desc = NULL;
		}

		/* Fallback to dynamic allocation if pool exhausted or oversized */
		if (!async_desc) {
			async_desc = adm_desc_alloc_fallback(adev, required_cpl_size);
			if (!async_desc) {
				dev_err(adev->dev, "unable to allocate descriptor\n");
				return NULL;
			}
		}
	}

	async_desc->mux = achan->mux ? ADM_CRCI_CTL_MUX_SEL : 0;
	async_desc->crci = crci;
	async_desc->blk_size = blk_size;
	/* Inherit exec_func from channel slave config */
	async_desc->exec_func = achan->exec_func;
	async_desc->exec_user = achan->exec_user;

	/* both command list entry and descriptors must be 8 byte aligned */
	cple = PTR_ALIGN(async_desc->cpl, ADM_DESC_ALIGN);
	desc = PTR_ALIGN(cple + 1, ADM_DESC_ALIGN);

	for_each_sg(sgl, sg, sg_len, i) {
		async_desc->length += sg_dma_len(sg);

		if (achan->slave.device_fc)
			desc = adm_process_fc_descriptors(achan, desc, sg, crci,
							  burst, direction);
		else
			desc = adm_process_non_fc_descriptors(achan, desc, sg,
							      direction);
	}

	/*
	 * For pooled descriptors, CPL buffer is already DMA-coherent.
	 * No dma_map_single or dma_sync needed - just set up the cmdptr.
	 */
	*cple = ADM_CPLE_LP;
	*cple |= (async_desc->dma_addr + ADM_DESC_ALIGN) >> 3;

	return vchan_tx_prep(&achan->vc, &async_desc->vd, flags);
}

/**
 * adm_terminate_all - terminate all transactions on a channel
 * @chan: dma channel
 *
 * Dequeues and frees all transactions, aborts current transaction
 * No callbacks are done
 *
 */
static int adm_terminate_all(struct dma_chan *chan)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct adm_device *adev = achan->adev;
	struct virt_dma_desc *vd, *_vd;
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&achan->vc.lock, flags);
	vchan_get_all_descriptors(&achan->vc, &head);

	/*
	 * Do NOT clear curr_txd here. The flush command will generate
	 * an IRQ, and the IRQ handler needs curr_txd to call the completion
	 * callback. The UART driver relies on this callback to restart RX DMA.
	 * The IRQ handler will clear curr_txd after completing the descriptor.
	 */

	/* send flush command to terminate current transaction */
	writel_relaxed(0x0,
		       adev->regs + ADM_CH_FLUSH_STATE0(achan->id, adev->ee));

	/*
	 * Free descriptors while holding the lock to prevent race conditions.
	 * Without this, a descriptor could be submitted between lock release
	 * and vchan_dma_desc_free_list, causing list corruption (LIST_POISON
	 * values in node pointers) and kernel crashes.
	 *
	 * We inline vchan_dma_desc_free_list logic here and clear REUSE flag
	 * to ensure desc_free is called directly without re-taking the lock.
	 */
	list_for_each_entry_safe(vd, _vd, &head, node) {
		list_del(&vd->node);
		dmaengine_desc_clear_reuse(&vd->tx);
		achan->vc.desc_free(vd);
	}

	spin_unlock_irqrestore(&achan->vc.lock, flags);

	return 0;
}

/*
 * qcom_adm_program_crci_ee0 - one-shot CRCI_CTL write to EE=0 window.
 *
 * On APQ8060/MSM8660 (Tenderloin) CRCI_CTL writes to EE=1 are silently
 * dropped; the live register lives at EE=0. The bootloader pre-programs
 * EE=0 for peripherals it enables (eMMC CRCI=1, SDC, NAND). Peripherals
 * the bootloader never touched — QCE crypto CRCI=4 (CE_IN) and CRCI=5
 * (CE_OUT) — read as zero at both EE windows after boot. Without a valid
 * EE=0 entry the CRCI handshake never fires: ADM pushes the first burst
 * into the QCE FIFO and then stalls indefinitely waiting for a flow-
 * control assertion that never comes.
 *
 * Call once at probe while the channel is idle. Writing mid-transfer
 * corrupts the in-flight burst (an earlier patch that did this inside
 * adm_start_dma killed eMMC — see git log).
 */
int qcom_adm_program_crci_ee0(struct dma_chan *chan, u32 crci_val)
{
	struct adm_chan *achan;
	struct adm_device *adev;

	if (!chan || !chan->device)
		return -EINVAL;

	achan = to_adm_chan(chan);
	adev  = achan->adev;

	if (!achan->crci)
		return -EINVAL;

	writel(crci_val, adev->regs + ADM_CRCI_CTL(achan->crci, 0));
	dev_dbg(adev->dev,
		"ADM program_crci_ee0: chan=%d crci=%d val=0x%x\n",
		achan->id, achan->crci, crci_val);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_adm_program_crci_ee0);

static int adm_slave_config(struct dma_chan *chan, struct dma_slave_config *cfg)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct qcom_adm_peripheral_config *config = cfg->peripheral_config;
	unsigned long flag;

	spin_lock_irqsave(&achan->vc.lock, flag);
	memcpy(&achan->slave, cfg, sizeof(struct dma_slave_config));
	if (cfg->peripheral_size == sizeof(*config)) {
		achan->crci = config->crci;
		achan->cmd_flags = config->cmd_flags;
		achan->exec_func = config->exec_func;
		achan->exec_user = config->exec_user;
	}
	spin_unlock_irqrestore(&achan->vc.lock, flag);

	return 0;
}

/**
 * adm_start_dma - start next transaction
 * @achan: ADM dma channel
 */
static void adm_start_dma(struct adm_chan *achan)
{
	struct virt_dma_desc *vd = vchan_next_desc(&achan->vc);
	struct adm_device *adev = achan->adev;
	struct adm_async_desc *async_desc;

	lockdep_assert_held(&achan->vc.lock);

	if (!vd)
		return;

	list_del(&vd->node);

	/* write next command list out to the CMD FIFO */
	async_desc = container_of(vd, struct adm_async_desc, vd);
	achan->curr_txd = async_desc;

	/* reset channel error */
	achan->error = 0;

	if (!achan->initialized) {
		/*
		 * On MSM8660/APQ8060 (Tenderloin) we INTENTIONALLY do NOT
		 * rewrite CH_CONF here. The bootloader programs CH_CONF in
		 * the EE=0 master view with the correct security-domain (SD)
		 * field for each channel — e.g. ch2 (eMMC) = 0x180008d5 with
		 * SD=1 (bit 4 set). The ADM scheduler honours those master-
		 * view values regardless of which EE window the host writes
		 * commands through.
		 *
		 * The previous RMW pattern (clear SEC_DOMAIN(7), set
		 * SEC_DOMAIN(ee) | SHADOW_EN) was harmless when adev->ee = 1
		 * because writes to the EE=1 CH_CONF window are silently
		 * dropped on this SoC. But it became destructive at EE=0
		 * because writes there DO stick, and SEC_DOMAIN(7) clears
		 * bit 4 — flipping the SD field from 1 → 0 and causing the
		 * eMMC channel to stop responding (CMDTIMEOUT on every
		 * subsequent transfer).
		 *
		 * Legacy webOS msm_dmov.c does write CH_CONF, but at EE=1
		 * where the writes don't stick, so the bootloader values are
		 * preserved by accident. We replicate the working behaviour
		 * intentionally: just enable IRQ + FLUSH in RSLT_CONF (which
		 * IS writable at EE=1) and leave CH_CONF alone.
		 */
		writel(ADM_CH_RSLT_CONF_IRQ_EN | ADM_CH_RSLT_CONF_FLUSH_EN,
		       adev->regs + ADM_CH_RSLT_CONF(achan->id, adev->ee));

		achan->initialized = 1;
	}

	/*
	 * Per-ADM-controller submit serialization, plus peripheral
	 * exec_func atomic hook. Matches the legacy webOS msm_dmov
	 * pattern: while we set up CRCI_CTL, call the peripheral's
	 * pre-submit hook (e.g. mmci writes DATACTRL + CMD), and write
	 * CMD_PTR, no other channel on this ADM controller can start
	 * its own submission.
	 *
	 * IRQs are saved/restored so adm_start_dma can also be called
	 * from the IRQ handler (after a channel completion, picking up
	 * the next pending descriptor).
	 */
	{
	unsigned long submit_flags;

	spin_lock_irqsave(&adev->submit_lock, submit_flags);

	/* set the crci block size if this transaction requires CRCI */
	if (async_desc->crci) {
		u32 crci_val = async_desc->mux | async_desc->blk_size;
		writel(crci_val,
		       adev->regs + ADM_CRCI_CTL(async_desc->crci, adev->ee));
	}

	/* make sure IRQ enable doesn't get reordered */
	wmb();

	/*
	 * Peripheral pre-submit hook (legacy msm_dmov exec_func). Called
	 * with submit_lock held, IRQs off, right before the CMD_PTR write.
	 * Lets the consumer driver (mmci, etc.) commit its own MMIO setup
	 * atomically with the ADM start.
	 */
	if (async_desc->exec_func)
		async_desc->exec_func(async_desc->exec_user);

	/* write next command list out to the CMD FIFO */
	writel(ALIGN(async_desc->dma_addr, ADM_DESC_ALIGN) >> 3,
	       adev->regs + ADM_CH_CMD_PTR(achan->id, adev->ee));

	/*
	 * EXPERIMENTAL: On MSM8660/APQ8060 (Tenderloin), also write CMD_PTR
	 * to EE=0. Most peripherals work with EE=1-only writes, but QCE crypto
	 * (channels 2-3 on ADM0) seems to require EE=0 writes as well.
	 * This is a workaround pending proper root-cause analysis.
	 */
	if (adev->ee == 1) {
		writel(ALIGN(async_desc->dma_addr, ADM_DESC_ALIGN) >> 3,
		       adev->regs + ADM_CH_CMD_PTR(achan->id, 0));
	}

	spin_unlock_irqrestore(&adev->submit_lock, submit_flags);
	}
}

/**
 * adm_dma_irq - irq handler for ADM controller
 * @irq: IRQ of interrupt
 * @data: callback data
 *
 * IRQ handler for the bam controller
 */
static irqreturn_t adm_dma_irq(int irq, void *data)
{
	struct adm_device *adev = data;
	u32 srcs;
	int i;

	srcs = readl_relaxed(adev->regs +
			ADM_SEC_DOMAIN_IRQ_STATUS(adev->ee));
	if (!srcs)
		return IRQ_NONE;

	/*
	 * Minimal hardirq path. For each channel that fired, read
	 * STATUS + CH_RSLT (the RSLT read is what acks the IRQ in the
	 * device), then atomically claim curr_txd under the channel's
	 * vc.lock so a peripheral driver re-submitting a fresh request
	 * between this hardirq and the tasklet running can't overwrite
	 * the descriptor we're about to complete. Heavy work -- error
	 * logging, vchan_cookie_complete, next-descriptor adm_start_dma
	 * -- is deferred to the completion tasklet so it can't delay
	 * other GIC hardirqs (notably the SDCC PIO refill IRQ).
	 */
	for (i = 0; i < ADM_MAX_CHANNELS; i++) {
		struct adm_chan *achan = &adev->channels[i];
		struct adm_async_desc *done;
		u32 status, result;
		unsigned long flags;

		if (!(srcs & BIT(i)))
			continue;

		status = readl_relaxed(adev->regs +
				       ADM_CH_STATUS_SD(i, adev->ee));
		if (!(status & ADM_CH_STATUS_VALID))
			continue;

		result = readl_relaxed(adev->regs +
				       ADM_CH_RSLT(i, adev->ee));
		if (!(result & ADM_CH_RSLT_VALID))
			continue;

		spin_lock_irqsave(&achan->vc.lock, flags);
		done = achan->curr_txd;
		achan->curr_txd = NULL;
		if (done) {
			done->last_result = result;
			list_add_tail(&done->completed_node,
				      &achan->pending_completed);
		}
		spin_unlock_irqrestore(&achan->vc.lock, flags);

		if (done)
			set_bit(i, &adev->pending_completion);
	}

	tasklet_schedule(&adev->completion_tasklet);
	return IRQ_HANDLED;
}

/**
 * adm_completion_tasklet_fn - process per-channel completions
 * @t: tasklet handle pointing at adm_device.completion_tasklet
 *
 * Runs in softirq context. Picks up the results the hardirq stashed
 * in each channel's pending_result, dispatches errors, completes the
 * vchan cookie, and kicks off the next descriptor via adm_start_dma.
 */
static void adm_completion_tasklet_fn(struct tasklet_struct *t)
{
	struct adm_device *adev = from_tasklet(adev, t, completion_tasklet);
	unsigned long pending;
	int i;

	pending = xchg(&adev->pending_completion, 0);

	for_each_set_bit(i, &pending, ADM_MAX_CHANNELS) {
		struct adm_chan *achan = &adev->channels[i];
		struct adm_async_desc *done, *tmp;
		LIST_HEAD(local);
		unsigned long flags;

		/*
		 * Splice the per-channel completed queue onto our local
		 * list under the channel lock, then drop the lock for
		 * vchan_cookie_complete + adm_start_dma. This avoids
		 * holding vc.lock across the vchan tasklet schedule.
		 */
		spin_lock_irqsave(&achan->vc.lock, flags);
		list_splice_init(&achan->pending_completed, &local);

		/*
		 * Kick off next DMA on this channel if mmci queued anything
		 * while we were waiting. adm_start_dma needs vc.lock held;
		 * it's a no-op if vchan_next_desc returns null.
		 */
		adm_start_dma(achan);
		spin_unlock_irqrestore(&achan->vc.lock, flags);

		list_for_each_entry_safe(done, tmp, &local, completed_node) {
			u32 result = done->last_result;

			list_del_init(&done->completed_node);

			if (result & ADM_CH_RSLT_ERR) {
				achan->error = 1;
				dev_err(adev->dev,
					"ADM DMA error: chan=%d result=0x%08x (err=%d flush=%d tpd=%d)\n",
					i, result,
					!!(result & ADM_CH_RSLT_ERR),
					!!(result & ADM_CH_RSLT_FLUSH),
					!!(result & ADM_CH_RSLT_TPD));
			}

			spin_lock_irqsave(&achan->vc.lock, flags);
			vchan_cookie_complete(&done->vd);
			spin_unlock_irqrestore(&achan->vc.lock, flags);
		}
	}
}

/**
 * adm_tx_status - returns status of transaction
 * @chan: dma channel
 * @cookie: transaction cookie
 * @txstate: DMA transaction state
 *
 * Return status of dma transaction
 */
static enum dma_status adm_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;
	size_t residue = 0;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	spin_lock_irqsave(&achan->vc.lock, flags);

	vd = vchan_find_desc(&achan->vc, cookie);
	if (vd)
		residue = container_of(vd, struct adm_async_desc, vd)->length;

	spin_unlock_irqrestore(&achan->vc.lock, flags);

	/*
	 * residue is either the full length if it is in the issued list, or 0
	 * if it is in progress.  We have no reliable way of determining
	 * anything in between
	 */
	dma_set_residue(txstate, residue);

	if (achan->error)
		return DMA_ERROR;

	return ret;
}

/**
 * adm_issue_pending - starts pending transactions
 * @chan: dma channel
 *
 * Issues all pending transactions and starts DMA
 */
static void adm_issue_pending(struct dma_chan *chan)
{
	struct adm_chan *achan = to_adm_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&achan->vc.lock, flags);

	if (vchan_issue_pending(&achan->vc) && !achan->curr_txd)
		adm_start_dma(achan);
	spin_unlock_irqrestore(&achan->vc.lock, flags);
}

/**
 * adm_dma_free_desc - free descriptor memory
 * @vd: virtual descriptor
 *
 * Returns pooled descriptors to the pool, frees dynamically allocated ones.
 */
static void adm_dma_free_desc(struct virt_dma_desc *vd)
{
	struct adm_async_desc *async_desc = container_of(vd,
			struct adm_async_desc, vd);

	if (async_desc->pool_index >= 0) {
		/* Return pooled descriptor to free list */
		adm_desc_put(async_desc);
	} else {
		/* Dynamic allocation - free coherent memory and struct */
		dma_free_coherent(async_desc->adev->dev, async_desc->dma_len,
				  async_desc->cpl, async_desc->dma_addr);
		kfree(async_desc);
	}
}

static void adm_channel_init(struct adm_device *adev, struct adm_chan *achan,
			     u32 index)
{
	achan->id = index;
	achan->adev = adev;

	INIT_LIST_HEAD(&achan->pending_completed);

	vchan_init(&achan->vc, &adev->common);
	achan->vc.desc_free = adm_dma_free_desc;
}

/**
 * adm_dma_xlate
 * @dma_spec:	pointer to DMA specifier as found in the device tree
 * @ofdma:	pointer to DMA controller data
 *
 * This can use either 1-cell or 2-cell formats, the first cell
 * identifies the slave device, while the optional second cell
 * contains the crci value.
 *
 * Returns pointer to appropriate dma channel on success or NULL on error.
 */
static struct dma_chan *adm_dma_xlate(struct of_phandle_args *dma_spec,
			       struct of_dma *ofdma)
{
	struct dma_device *dev = ofdma->of_dma_data;
	struct dma_chan *chan, *candidate = NULL;
	struct adm_chan *achan;

	if (!dev || dma_spec->args_count > 2)
		return NULL;

	list_for_each_entry(chan, &dev->channels, device_node)
		if (chan->chan_id == dma_spec->args[0]) {
			candidate = chan;
			break;
		}

	if (!candidate)
		return NULL;

	achan = to_adm_chan(candidate);
	if (dma_spec->args_count == 2)
		achan->crci = dma_spec->args[1];
	else
		achan->crci = 0;

	dev_dbg(dev->dev, "ADM xlate: chan=%d args_count=%d crci=%d\n",
		dma_spec->args[0], dma_spec->args_count, achan->crci);

	return dma_get_slave_channel(candidate);
}

static int adm_dma_probe(struct platform_device *pdev)
{
	struct adm_device *adev;
	int ret;
	u32 i;

	adev = devm_kzalloc(&pdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->dev = &pdev->dev;

	/*
	 * Per-controller submit serialization (legacy msm_dmov pattern).
	 * Initialised early so adm_start_dma can always take it.
	 */
	spin_lock_init(&adev->submit_lock);

	/*
	 * Completion tasklet: hardirq stashes per-channel results and
	 * schedules this; tasklet does the heavy per-channel processing
	 * (vchan + adm_start_dma) in softirq context to keep hardirq
	 * path minimal.
	 */
	tasklet_setup(&adev->completion_tasklet, adm_completion_tasklet_fn);

	adev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adev->regs))
		return PTR_ERR(adev->regs);

	adev->irq = platform_get_irq(pdev, 0);
	if (adev->irq < 0)
		return adev->irq;

	ret = of_property_read_u32(pdev->dev.of_node, "qcom,ee", &adev->ee);
	if (ret) {
		dev_err(adev->dev, "Execution environment unspecified\n");
		return ret;
	}

	adev->core_clk = devm_clk_get(adev->dev, "core");
	if (IS_ERR(adev->core_clk))
		return PTR_ERR(adev->core_clk);

	adev->iface_clk = devm_clk_get(adev->dev, "iface");
	if (IS_ERR(adev->iface_clk))
		return PTR_ERR(adev->iface_clk);

	adev->clk_reset = devm_reset_control_get_exclusive(&pdev->dev, "clk");
	if (IS_ERR(adev->clk_reset)) {
		dev_err(adev->dev, "failed to get ADM0 reset\n");
		return PTR_ERR(adev->clk_reset);
	}

	adev->c0_reset = devm_reset_control_get_exclusive(&pdev->dev, "c0");
	if (IS_ERR(adev->c0_reset)) {
		dev_err(adev->dev, "failed to get ADM0 C0 reset\n");
		return PTR_ERR(adev->c0_reset);
	}

	adev->c1_reset = devm_reset_control_get_exclusive(&pdev->dev, "c1");
	if (IS_ERR(adev->c1_reset)) {
		dev_err(adev->dev, "failed to get ADM0 C1 reset\n");
		return PTR_ERR(adev->c1_reset);
	}

	adev->c2_reset = devm_reset_control_get_exclusive(&pdev->dev, "c2");
	if (IS_ERR(adev->c2_reset)) {
		dev_err(adev->dev, "failed to get ADM0 C2 reset\n");
		return PTR_ERR(adev->c2_reset);
	}

	ret = clk_prepare_enable(adev->core_clk);
	if (ret) {
		dev_err(adev->dev, "failed to prepare/enable core clock\n");
		return ret;
	}

	ret = clk_prepare_enable(adev->iface_clk);
	if (ret) {
		dev_err(adev->dev, "failed to prepare/enable iface clock\n");
		goto err_disable_core_clk;
	}

	/*
	 * EBI interconnect path for DMA memory access.
	 *
	 * The legacy webOS kernel used ebi1_adm_clk clock voter to ensure
	 * minimum EBI bandwidth during DMA operations. The interconnect
	 * framework provides equivalent functionality.
	 *
	 * Path: ADM -> SFAB -> AFAB -> EBI (system memory)
	 */
	adev->icc_path = devm_of_icc_get(adev->dev, "memory");
	if (IS_ERR(adev->icc_path)) {
		ret = PTR_ERR(adev->icc_path);
		if (ret != -ENODATA) {
			dev_err(adev->dev, "failed to get interconnect path: %d\n", ret);
			goto err_disable_clks;
		}
		/* No interconnect defined in DT - optional for backwards compat */
		adev->icc_path = NULL;
	}

	if (adev->icc_path) {
		/*
		 * Vote for minimum EBI bandwidth to keep the path active.
		 * Legacy kernel used clk_set_rate(ebiclk, 27) for 27 MHz minimum.
		 * Using 128 MB/s as a reasonable floor for DMA operations.
		 */
		ret = icc_set_bw(adev->icc_path, 0, 128000);
		if (ret) {
			dev_err(adev->dev, "failed to set interconnect bandwidth: %d\n", ret);
			goto err_disable_clks;
		}
	}

	/*
	 * Skip reset sequence - webOS kernel doesn't reset ADM in probe.
	 * The ADM may be pre-configured by bootloader/TrustZone.
	 */
#if 0
	reset_control_assert(adev->clk_reset);
	reset_control_assert(adev->c0_reset);
	reset_control_assert(adev->c1_reset);
	reset_control_assert(adev->c2_reset);

	udelay(2);

	reset_control_deassert(adev->clk_reset);
	reset_control_deassert(adev->c0_reset);
	reset_control_deassert(adev->c1_reset);
	reset_control_deassert(adev->c2_reset);
#endif

	adev->channels = devm_kcalloc(adev->dev, ADM_MAX_CHANNELS,
				      sizeof(*adev->channels), GFP_KERNEL);

	if (!adev->channels) {
		ret = -ENOMEM;
		goto err_disable_clks;
	}

	/* allocate and initialize channels */
	INIT_LIST_HEAD(&adev->common.channels);

	for (i = 0; i < ADM_MAX_CHANNELS; i++)
		adm_channel_init(adev, &adev->channels[i], i);

	/* Initialize descriptor pool for reduced per-transfer overhead */
	ret = adm_desc_pool_init(adev);
	if (ret) {
		dev_err(adev->dev, "failed to initialize descriptor pool\n");
		goto err_disable_clks;
	}

	/* reset CRCIs */
	for (i = 0; i < 16; i++)
		writel(ADM_CRCI_CTL_RST, adev->regs +
			ADM_CRCI_CTL(i, adev->ee));

	/*
	 * Initialize per-channel state. RSLT_CONF gets the IRQ + FLUSH
	 * enable; CH_CONF is deliberately NOT rewritten — see the long
	 * comment in the channel-alloc path above for why. The
	 * bootloader-programmed CH_CONF values (visible in the master
	 * EE=0 view) carry the correct SD field for each channel and
	 * must not be touched.
	 */
	for (i = 0; i < ADM_MAX_CHANNELS; i++) {
		writel(ADM_CH_RSLT_CONF_IRQ_EN | ADM_CH_RSLT_CONF_FLUSH_EN,
		       adev->regs + ADM_CH_RSLT_CONF(i, adev->ee));
		adev->channels[i].initialized = 1;
	}

	/*
	 * Diagnostic: read back CH_CONF and RSLT_CONF for all channels.
	 * Helps identify if writes don't stick (clock/SD ownership issue).
	 */
	for (i = 0; i < ADM_MAX_CHANNELS; i++) {
		u32 ch_conf, rslt_conf;

		ch_conf = readl_relaxed(adev->regs + ADM_CH_CONF(i, adev->ee));
		rslt_conf = readl_relaxed(adev->regs + ADM_CH_RSLT_CONF(i, adev->ee));
		dev_info(adev->dev,
			 "ADM ch%d EE%d: CH_CONF=0x%08x RSLT_CONF=0x%08x\n",
			 i, adev->ee, ch_conf, rslt_conf);
	}

	ret = devm_request_irq(adev->dev, adev->irq, adm_dma_irq,
			       0, "adm_dma", adev);
	if (ret)
		goto err_disable_clks;

	platform_set_drvdata(pdev, adev);

	adev->common.dev = adev->dev;
	adev->common.dev->dma_parms = &adev->dma_parms;

	/* set capabilities */
	dma_cap_zero(adev->common.cap_mask);
	dma_cap_set(DMA_SLAVE, adev->common.cap_mask);
	dma_cap_set(DMA_PRIVATE, adev->common.cap_mask);

	/* initialize dmaengine apis */
	adev->common.directions = BIT(DMA_DEV_TO_MEM | DMA_MEM_TO_DEV);
	adev->common.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	adev->common.src_addr_widths = DMA_SLAVE_BUSWIDTH_4_BYTES;
	adev->common.dst_addr_widths = DMA_SLAVE_BUSWIDTH_4_BYTES;
	adev->common.device_free_chan_resources = adm_free_chan;
	adev->common.device_prep_slave_sg = adm_prep_slave_sg;
	adev->common.device_issue_pending = adm_issue_pending;
	adev->common.device_tx_status = adm_tx_status;
	adev->common.device_terminate_all = adm_terminate_all;
	adev->common.device_config = adm_slave_config;

	ret = dma_async_device_register(&adev->common);
	if (ret) {
		dev_err(adev->dev, "failed to register dma async device\n");
		goto err_pool_destroy;
	}

	ret = of_dma_controller_register(pdev->dev.of_node, adm_dma_xlate,
					 &adev->common);
	if (ret)
		goto err_unregister_dma;

	return 0;

err_unregister_dma:
	dma_async_device_unregister(&adev->common);
err_pool_destroy:
	adm_desc_pool_destroy(adev);
err_disable_clks:
	clk_disable_unprepare(adev->iface_clk);
err_disable_core_clk:
	clk_disable_unprepare(adev->core_clk);

	return ret;
}

static void adm_dma_remove(struct platform_device *pdev)
{
	struct adm_device *adev = platform_get_drvdata(pdev);
	struct adm_chan *achan;
	u32 i;

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&adev->common);

	for (i = 0; i < ADM_MAX_CHANNELS; i++) {
		achan = &adev->channels[i];

		/* mask IRQs for this channel/EE pair */
		writel(0, adev->regs + ADM_CH_RSLT_CONF(achan->id, adev->ee));

		tasklet_kill(&adev->channels[i].vc.task);
		adm_terminate_all(&adev->channels[i].vc.chan);
	}

	devm_free_irq(adev->dev, adev->irq, adev);

	/* Drain any pending tasklet now that no new IRQs can fire. */
	tasklet_kill(&adev->completion_tasklet);

	/* Free descriptor pool */
	adm_desc_pool_destroy(adev);

	clk_disable_unprepare(adev->core_clk);
	clk_disable_unprepare(adev->iface_clk);
}

static const struct of_device_id adm_of_match[] = {
	{ .compatible = "qcom,adm", },
	{}
};
MODULE_DEVICE_TABLE(of, adm_of_match);

static struct platform_driver adm_dma_driver = {
	.probe = adm_dma_probe,
	.remove = adm_dma_remove,
	.driver = {
		.name = "adm-dma-engine",
		.of_match_table = adm_of_match,
	},
};

module_platform_driver(adm_dma_driver);

MODULE_AUTHOR("Andy Gross <agross@codeaurora.org>");
MODULE_DESCRIPTION("QCOM ADM DMA engine driver");
MODULE_LICENSE("GPL v2");
