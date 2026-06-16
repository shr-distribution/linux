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
#include <linux/timer.h>

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
#define ADM_CH_FLUSH_STATE1(chan, ee)	(0xc0 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_FLUSH_STATE2(chan, ee)	(0x100 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_FLUSH_STATE3(chan, ee)	(0x140 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_FLUSH_STATE4(chan, ee)	(0x180 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_FLUSH_STATE5(chan, ee)	(0x1c0 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_STATUS_SD(chan, ee)	(0x200 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_CONF(chan, ee)		(0x240 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_CH_RSLT_CONF(chan, ee)	(0x300 + ADM_CHAN_EE_OFFS(chan, ee))
#define ADM_SEC_DOMAIN_IRQ_STATUS(ee)	(0x380 + ADM_EE_OFFS(ee))
#define ADM_CI_CONF(ci)			(0x390 + (ci) * ADM_CI_MULTI)
#define ADM_GP_CTL			0x3d8
#define ADM_CRCI_CTL(crci, ee)		(0x400 + (crci) * ADM_CRCI_MULTI + \
						ADM_EE_OFFS(ee))

/* channel status (STATUS_SD register) */
#define ADM_CH_STATUS_CMD_PTR_RDY	BIT(0)
#define ADM_CH_STATUS_VALID		BIT(1)

/* channel flush command (written to ADM_CH_FLUSH_STATE0) */
#define ADM_CH_FLUSH_GRACEFUL		BIT(31)	/* drain in-flight data & report it; 0 = abrupt abort/discard */

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
 * Per-word byteswap bits in the ADM command word. Required by the CE2
 * Crypto Engine on MSM8x60: the engine consumes data at DATA_SHADOW0
 * as big-endian-packed integers, and the engine's CRCI handshake is
 * tied to ADM doing the swap rather than the peripheral driver
 * pre-swapping in memory.  Toggled per channel via
 * qcom_adm_peripheral_config.{swap_bytes, swap_shorts}.
 */
#define ADM_CMD_DST_SWAP_BYTES		BIT(14)
#define ADM_CMD_DST_SWAP_SHORTS		BIT(15)
#define ADM_CMD_SRC_SWAP_BYTES		BIT(11)
#define ADM_CMD_SRC_SWAP_SHORTS		BIT(12)

#define ADM_CMD_TYPE_SINGLE		0x0
#define ADM_CMD_TYPE_BOX		0x3

#define ADM_CRCI_MUX_SEL		BIT(4)
#define ADM_DESC_ALIGN			8
#define ADM_MAX_XFER			(SZ_64K - 1)
#define ADM_MAX_ROWS			(SZ_64K - 1)
#define ADM_MAX_CHANNELS		16

/*
 * Descriptor pool configuration.
 *
 * The pool exists so adm_prep_slave_sg() never has to call
 * dma_alloc_coherent(GFP_NOWAIT) on the hot submit path; the fallback
 * allocation path is fragile under coherent-pool fragmentation
 * (observed during initramfs LVM activation: large dm-linear reads
 * that exceeded the per-descriptor CPL buffer were forced through
 * the fallback, and a stale fragmented coherent pool returned NULL,
 * surfacing as "unable to allocate descriptor" floods that wedged
 * vgchange).
 *
 * Sizing trades memory against fallback dependence:
 *
 *  - ADM_CPL_BUF_SIZE = 8 KiB holds ~340 BOX descriptors per pooled
 *    descriptor (struct adm_desc_hw_box = 24 bytes).  mmci-pl18x
 *    caps requests at max_segs = 128 SG entries, so a single mmci
 *    transfer needs at most 128 * 24 = 3 KiB of CPL space, well
 *    inside one pool entry.  Device-mapper stacked reads from
 *    LVM/dm-crypt also fit because dm respects the underlying
 *    queue's max_segs limit.  The previous 2 KiB sizing held only
 *    ~83 BOX descs and forced anything above 64 SG to the fallback.
 *
 *  - ADM_DESC_POOL_SIZE = 8 per channel * ADM_MAX_CHANNELS = 16 gives
 *    128 pooled descriptors per controller -- well above the actual
 *    in-flight ceiling (~5-6 concurrent transfers across all
 *    consumers: 2 SDCC + 2 UART RX + QCE).  Legacy webOS msm_sdcc
 *    pre-allocated dmov_box cmd[NR_SG=32] per controller once and
 *    never allocated at request time; this is the same model.
 *
 *  Memory cost: 128 * 8 KiB = 1 MiB DMA-coherent per ADM controller,
 *  2 MiB total across both ADMs on apq8060.  One-time, paid at probe.
 */
#define ADM_MAX_SG_PER_DESC		64	/* Max SG entries per pooled desc (BOX rows) */
#define ADM_CPL_BUF_SIZE		8192	/* CPL buffer size (340 box descs) */
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

	/* Pool management */
	struct list_head pool_node;
	int pool_index;		/* -1 = dynamic alloc, >=0 = pooled */
};

/*
 * A wedged ADM channel produces no FLUSH IRQ and never re-asserts
 * RSLT_VALID, so the dmaengine layer has no completion signal to act
 * on.  The per-channel watchdog gives us a deterministic upper bound
 * on how long a single submission can hold a channel before we
 * recover by completing the queued descriptor with an error and
 * starting the next one.  500 ms matches the watchdog interval used
 * by drivers/dma/imx-dma.c for the same class of "no-IRQ" failure
 * mode -- long enough to avoid false positives on slow SDCC PIO
 * fallbacks, short enough that systemd's mount / udev retries
 * proceed before they declare the device dead.
 */
#define ADM_WATCHDOG_TIMEOUT_MS	500

struct adm_chan {
	struct virt_dma_chan vc;
	struct adm_device *adev;

	/* parsed from DT */
	u32 id;			/* channel id */

	struct adm_async_desc *curr_txd;
	struct dma_slave_config slave;
	u32 crci;

	/*
	 * Per-channel swap config from qcom_adm_peripheral_config.
	 * Encoded into the ADM command word at descriptor build time
	 * (DST_SWAP for MEM_TO_DEV, SRC_SWAP for DEV_TO_MEM).
	 */
	bool swap_bytes;
	bool swap_shorts;

	/*
	 * Per-channel exec_func, set via adm_slave_config from
	 * struct qcom_adm_peripheral_config. Inherited by descriptors
	 * prepared on this channel.
	 */
	void (*exec_func)(void *exec_user);
	void *exec_user;

	struct list_head node;
	struct timer_list watchdog;

	int error;
	int initialized;

	/*
	 * CMD_PTR_RDY busy-wait tracking (legacy webOS parity, see comment in
	 * adm_start_dma).  Counter increments every time we found the channel
	 * NOT ready at CMD_PTR write time; logged flag is set after the first
	 * one-shot dev_warn so subsequent occurrences do not burn IRQ latency.
	 */
	u32 cmd_ptr_not_rdy_count;
	u8 cmd_ptr_not_rdy_logged;

	/*
	 * One-shot FLUSH-state full dump.  The ADM-DIAG SDCC-drain-stall path
	 * reads all six FLUSH_STATE registers on the first FLUSH per channel
	 * and emits them in a single dev_warn; subsequent FLUSHes on the same
	 * channel fall back to the single-register variant.  Tracked
	 * per-channel here (was previously a function-static u8 bitmask in
	 * adm_dma_irq, which had program-wide lifetime and let the first
	 * stall on any controller silence the diagnostic on every other
	 * controller's matching channel index).
	 */
	u8 flush_state_dumped;
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
	struct icc_path *icc_path_p1;
	int irq;

	/* Descriptor pool for reduced per-transfer allocation overhead */
	struct adm_async_desc *desc_pool;	/* Pre-allocated descriptors */
	void *cpl_pool_virt;			/* Coherent CPL buffer pool */
	dma_addr_t cpl_pool_dma;		/* DMA address of CPL pool */
	struct list_head desc_free_list;	/* Free descriptor list */
	spinlock_t pool_lock;			/* Protects free list */

	/*
	 * Per-CRCI submit serialization. Held across the peripheral
	 * exec_func call and the CMD_PTR write in adm_start_dma.
	 *
	 * Originally a single per-controller submit_lock (matching the
	 * legacy webOS msm_dmov per-ADM spinlock), but that wedged eMMC
	 * under concurrent WiFi load on tenderloin: both CRCI 1 (eMMC,
	 * sdcc1/mmci) and CRCI 5 (WiFi, sdcc4/mmci) submit with
	 * exec_func != NULL.  Holding a global lock across mmci's ~6-8 µs
	 * SDCC-reg-write window in one peripheral's exec_func blocks the
	 * other peripheral's submit long enough for its SDCC RX FIFO to
	 * overflow (RXOVERRUN-marked-as-fabric -> DATATIMEOUT -> CMD
	 * timeout cascade -> eMMC card stuck busy -> full eMMC death).
	 *
	 * CRCI 1 and CRCI 5 touch entirely separate state - different
	 * CRCI_CTL MMIO addresses (per-CRCI cache lookup), different
	 * peripheral MMIO in their exec_funcs (sdcc1 vs sdcc4 base
	 * registers), different ADM channel CMD_PTR registers.  Nothing
	 * is shared between them at this layer.  Per-CRCI locks let them
	 * submit truly in parallel, matching legacy webOS dma.c:321
	 * check_crci_conflict which never blocks non-overlapping CRCIs
	 * against each other.
	 *
	 * 16 locks = one per possible CRCI on this SoC (CRCI index is
	 * a 4-bit field in CRCI_CTL).  Index 0 is "no CRCI" and goes
	 * unused for exec_func paths (which always have an SDCC CRCI).
	 *
	 * See memory note: project_adm_submit_lock_starvation.
	 */
	spinlock_t submit_lock[16];

	/*
	 * CRCI_CTL write-cache (Fix 2 of project_adm_submit_lock_starvation).
	 *
	 * Legacy webOS dma.c:378 (`crci_mask_compare`) only writes the CRCI
	 * register when the desired mask actually differs from what's
	 * currently programmed.  Mainline qcom_adm wrote CRCI_CTL on every
	 * adm_start_dma call unconditionally — at our BT sustained 308 pkt/sec
	 * RX rate that's 308 extra ~600 ns MMIO writes per second to a
	 * register whose value rarely changes.
	 *
	 * Cache the last-written value per CRCI index (0-15 = 16 CRCIs total
	 * per controller).  Updated under submit_lock so the write+cache pair
	 * is atomic vs another mmci submit; the no-exec_func path (BT RX) can
	 * read-and-skip lock-free because the only way the value changes is
	 * through the exec_func gated path which barriers via the lock release.
	 *
	 * crci_ctl_cache_valid is a bitmap so 0-initialised state correctly
	 * forces the first write of each CRCI to actually hit MMIO.
	 */
	u32 crci_ctl_cache[16];
	u16 crci_ctl_cache_valid;	/* one bit per CRCI */
};

/**
 * adm_free_chan - Frees dma resources associated with the specific channel
 *
 * @chan: dma channel
 *
 * Free all allocated descriptors associated with this channel
 */
static void adm_start_dma(struct adm_chan *achan);

static void adm_free_chan(struct dma_chan *chan)
{
	struct adm_chan *achan = to_adm_chan(chan);

	/* free all queued descriptors */
	timer_delete_sync(&achan->watchdog);
	vchan_free_chan_resources(to_virt_chan(chan));
}

/*
 * adm_watchdog_timeout - per-channel watchdog handler
 *
 * Fires ADM_WATCHDOG_TIMEOUT_MS after adm_start_dma() writes CMD_PTR
 * if the channel has not produced a RSLT_VALID IRQ to acknowledge the
 * transfer.  On apq8060 / HP TouchPad this can happen when the
 * channel pipeline state machine wedges (CMD_PTR_RDY stays at 0
 * indefinitely or a FLUSH never posts a result), holding curr_txd
 * busy and blocking every subsequent submission on the same channel.
 *
 * Recovery flow follows drivers/dma/imx-dma.c:imxdma_watchdog():
 *
 *   1. Take the vchan lock so the IRQ handler can't race us.
 *   2. If a curr_txd is still set, mark the channel error,
 *      complete the descriptor via vchan_cookie_complete() so the
 *      consumer's submit_error/callback path runs, and clear
 *      curr_txd.  The dmaengine layer will report DMA_ERROR to the
 *      MMC / UART / NAND / crypto consumer and they will recover
 *      via their normal retry/timeout machinery.
 *   3. Issue an abrupt FLUSH (bit 31 cleared) to the channel so any
 *      stuck in-flight DMA pipeline is discarded.  We deliberately
 *      do not use ADM_CH_FLUSH_GRACEFUL here -- the channel is by
 *      definition unresponsive at this point and a graceful flush
 *      would just wait again for a response that never comes.
 *
 * Intentionally do NOT auto-restart the next queued descriptor.
 * If the channel is still wedged at the hardware level, calling
 * adm_start_dma() here would re-arm the watchdog 500 ms later
 * for the next descriptor while a higher-level consumer keeps
 * refilling the queue with retries, producing a non-terminating
 * recovery loop.  Letting the consumer observe DMA_ERROR via
 * adm_tx_status() and drive the next dma_async_issue_pending()
 * itself converges naturally: a transient wedge recovers on
 * the consumer's retry; a persistent one stops getting fresh
 * work and the channel goes idle.
 *
 * Doing the recovery in soft-irq (timer) context is safe because
 * the IRQ handler also takes achan->vc.lock; spin_lock_irqsave()
 * here serialises against it.
 */
static void adm_watchdog_timeout(struct timer_list *t)
{
	struct adm_chan *achan = timer_container_of(achan, t, watchdog);
	struct adm_device *adev = achan->adev;
	struct adm_async_desc *async_desc;
	unsigned long flags;

	spin_lock_irqsave(&achan->vc.lock, flags);

	async_desc = achan->curr_txd;
	if (!async_desc) {
		/* Raced with the RSLT_VALID IRQ handler; nothing to do. */
		spin_unlock_irqrestore(&achan->vc.lock, flags);
		return;
	}

	achan->error = 1;
	achan->curr_txd = NULL;

	dev_warn_ratelimited(adev->dev,
			     "ADM ch%u watchdog: no RSLT_VALID within %u ms, recovering channel\n",
			     achan->id, ADM_WATCHDOG_TIMEOUT_MS);

	/* Abrupt flush -- graceful would wait for a response we won't get. */
	writel_relaxed(0, adev->regs + ADM_CH_FLUSH_STATE0(achan->id, adev->ee));

	vchan_cookie_complete(&async_desc->vd);

	/*
	 * Intentionally do NOT auto-start the next queued descriptor.
	 * If the channel is still wedged at the hardware level, calling
	 * adm_start_dma() here would re-arm the watchdog 500 ms later
	 * for the next descriptor, creating a non-terminating recovery
	 * loop while a higher-level consumer keeps refilling the queue
	 * with retries.  Let the consumer's callback path observe
	 * DMA_ERROR via adm_tx_status() and drive the next
	 * dma_async_issue_pending() itself -- this converges naturally
	 * whether the wedge is transient or persistent.
	 */

	spin_unlock_irqrestore(&achan->vc.lock, flags);
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

/* devm-action wrapper so any probe-error path after pool_init implicitly
 * unwinds the pool (~256 KB coherent DMA buffer + struct array).  Without
 * this, the goto err_disable_clks paths between pool_init and
 * dma_async_device_register would silently leak the entire pool on
 * probe failure.
 */
static void adm_desc_pool_destroy_action(void *data)
{
	adm_desc_pool_destroy((struct adm_device *)data);
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

	dev_dbg(adev->dev, "ADM fallback alloc: cpl_size=%zu\n", cpl_size);

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
	 * Burst is in bytes.  ADM blk_size encoding:
	 *   0 = 16 B, 1 = 32 B, 2 = 64 B, 3 = 128 B, 4 = 192 B, 5 = 256 B
	 *
	 * The ADM has no encoding for sub-16-byte bursts; an 8 B burst would
	 * be silently rounded by callers that ignore the error, producing a
	 * mis-paced CRCI handshake.  Reject explicitly so the consumer is
	 * forced to fix its slave config (e.g. mmci's src_maxburst=8 with
	 * addr_width=4 already yields a 32 B burst -- correct -- so any
	 * caller arriving here with burst < 16 is misconfigured).
	 */
	switch (burst) {
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
		if (achan->swap_bytes)
			crci_cmd |= ADM_CMD_SRC_SWAP_BYTES;
		if (achan->swap_shorts)
			crci_cmd |= ADM_CMD_SRC_SWAP_SHORTS;
		row_offset = burst;
		src = &achan->slave.src_addr;
		dst = &mem_addr;
	} else {
		crci_cmd = ADM_CMD_DST_CRCI(crci);
		if (achan->swap_bytes)
			crci_cmd |= ADM_CMD_DST_SWAP_BYTES;
		if (achan->swap_shorts)
			crci_cmd |= ADM_CMD_DST_SWAP_SHORTS;
		row_offset = burst << 16;
		src = &mem_addr;
		dst = &achan->slave.dst_addr;
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

		dev_dbg(achan->adev->dev,
			"ADM box: cmd=0x%x src=0x%x dst=0x%x row_len=0x%x num_rows=0x%x row_off=0x%x burst=%u dir=%d crci=%u\n",
			box_desc->cmd, box_desc->src_addr, box_desc->dst_addr,
			box_desc->row_len, box_desc->num_rows,
			box_desc->row_offset, burst, direction, crci);

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


	dev_dbg(adev->dev,
		"ADM prep_slave_sg: chan=%d device_fc=%d achan->crci=%d burst=%d dir=%d\n",
		achan->id, achan->slave.device_fc, achan->crci, burst, direction);

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

		/*
		 * SDCC block-size override (half-FIFO).
		 *
		 * Legacy webOS msm_dmov adm1_crci_conf[] hardcodes blk_size=1
		 * for every SDCC CRCI:
		 *   CRCI 1 = sdcc1 (eMMC),  CRCI 5 = sdcc4 (WiFi),
		 *   CRCI 2 / 4 = other SDCCs on the same ADM.
		 * blk_size=1 = half-FIFO (32 B) = matches the SDCC raising
		 * CRCI at its half-full FIFO threshold (mainline derives
		 * blk_size from burst, so a 64 B FIFO becomes blk_size=2 =
		 * full-FIFO -> mis-paced CRCI handshake -> DATACRCFAIL /
		 * RXOVERRUN on large eMMC reads, and silent DMA-data
		 * corruption on the AR6003 HTC mailbox writes that broke
		 * WPA-handshake EAPOL frames on tenderloin).
		 *
		 * Gate the override on burst == 64 so it stays scoped to
		 * SDCC consumers.  Other CRCI peripherals (QCE Crypto Engine
		 * CE_OUT on some SoCs uses 16 B handshake granularity, so
		 * leaves burst at 16 -> adm_get_blksize() returns 0 ->
		 * blk_size already correct).  Without the burst guard, an
		 * earlier CRCI-only override on CRCI 5 broke QCE on platforms
		 * where CRCI 5 routed to the Crypto Engine: engine DOUT FIFO
		 * fills faster than ADM drained, engine stalled in PROCESSING
		 * state with DOUT_AVAIL=4 dwords after ~6 AES blocks
		 * (STATUS=0x1120120c).
		 *
		 * On APQ8060/tenderloin specifically, CRCI 5 routes to
		 * sdcc4 (WiFi) per qcom-apq8060-tenderloin-common.dtsi:2913
		 * (qcom,sdcc-crci = <5>).
		 *
		 * Assumption / limitation: burst == 64 is taken as a proxy for
		 * "this CRCI is an SDCC controller" because every known
		 * Qualcomm SoC routes SDCC CRCIs to qcom_adm with a 64-byte
		 * half-FIFO handshake, while every other CRCI peripheral
		 * (QCE 16, UART 16, NAND variable but != 64) has a smaller
		 * granularity.  A peripheral with a 64-byte burst that ISN'T
		 * an SDCC would silently receive this override and likely run
		 * with mis-paced CRCI handshake -- no such peripheral exists
		 * on the SoCs currently bound to qcom_adm (IPQ8064 NAND,
		 * APQ8060/MSM8660 SDCC1+SDCC4, MSM8960 SDCC).  Long-term this
		 * heuristic should be replaced with a per-SoC CRCI table
		 * keyed off OF match data identifying SDCC CRCI numbers
		 * explicitly; tracked in task #31 (per-compat gating).
		 */
		if (burst == 64)
			blk_size = 1;
	}

	/* iterate through sgs and compute allocation size of structures */
	if (achan->slave.device_fc) {
		for_each_sg(sgl, sg, sg_len, i) {
			box_count += DIV_ROUND_UP(sg_dma_len(sg) / burst,
						  ADM_MAX_ROWS);
			if (sg_dma_len(sg) % burst)
				single_count++;
		}
	} else {
		single_count = sg_nents_for_dma(sgl, sg_len, ADM_MAX_XFER);
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

	/*
	 * Derive MUX_SEL from the CRCI's mux-select flag (BIT4). Muxed CRCIs
	 * encode the flag in the DT value, e.g. GSBI10 HSUART2 RX touchscreen
	 * CRCI 26 = (1<<4)+10: base CRCI 10 + ADM_CRCI_MUX_SEL. Since Fix #4
	 * writes CRCI_CTL at the live EE=0, failing to set MUX_SEL clears it on
	 * hardware and kills the muxed CRCI handshake (touchscreen RX went
	 * dead). On EE=1 the write was dropped, so this was previously latent.
	 */
	async_desc->mux = (achan->crci & ADM_CRCI_MUX_SEL) ? ADM_CRCI_CTL_MUX_SEL : 0;
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
	 *
	 * LIMITATION (Sashiko High #6): if the channel is wedged at the
	 * hardware level the FLUSH IRQ may never fire and curr_txd will
	 * stay set indefinitely, holding a reference to the descriptor and
	 * blocking the next submission.  There is currently no watchdog
	 * timer / fallback path; the channel must be re-armed at the SoC
	 * level (e.g. ADM reset_control sequence on platforms that allow
	 * it -- not tenderloin, see commit b49e0b8b0ac4).  Adding a
	 * delayed_work-based recovery -- 'force-clear curr_txd if no
	 * FLUSH IRQ within Nms' -- is the proper fix and is tracked as a
	 * follow-up; intentionally omitted here to keep the
	 * tenderloin-stress refactor series scoped.
	 */

	/*
	 * Send a GRACEFUL flush (BIT 31) rather than an abrupt abort (0x0).
	 * Graceful flush makes the ADM drain any in-flight read/write data to
	 * memory and post a result with the partial state, instead of
	 * discarding it. This matches legacy msm_dmov (DMOV_FLUSH_TYPE = 1<<31)
	 * and the HTC 3.4 driver. The abrupt 0x0 abort dropped residual bytes
	 * still in the ADM read pipeline -> RX DMA returned "correct count,
	 * zero data" and SDCC/WiFi DMA transfers wedged on cleanup.
	 *
	 * EE: FLUSH_STATE0 is in the per-channel COMMAND bank (CMD_PTR/RSLT/
	 * FLUSH/STATUS), which on MSM8660/APQ8060 is live at EE=1. Verified by
	 * /dev/mem on webOS: CMD_PTR is live at the EE=1 aperture (+0x800),
	 * while only the config bank (CONF/CRCI_CTL) is live at EE=0. So
	 * adev->ee (=1) is the correct aperture for the flush.
	 */
	writel_relaxed(ADM_CH_FLUSH_GRACEFUL,
		       adev->regs + ADM_CH_FLUSH_STATE0(achan->id, adev->ee));

	spin_unlock_irqrestore(&achan->vc.lock, flags);

	/*
	 * Walk the local 'head' list and free descriptors OUTSIDE the
	 * spinlock.  For dynamic descriptors achan->vc.desc_free() routes
	 * to adm_dma_free_desc -> dma_free_coherent(), which may sleep
	 * under PREEMPT_RT / KASAN -- the previous in-lock loop violated
	 * the "no sleeping under spin_lock_irqsave" rule (Sashiko High #2).
	 *
	 * The old "race against fresh submission" concern does not apply
	 * here: vchan_get_all_descriptors() spliced the entire pending
	 * list onto 'head' (a stack local) under the lock; any new submit
	 * goes into the freshly empty vc.desc_submitted -- it cannot
	 * race-add to the local list we are about to walk.
	 */
	list_for_each_entry_safe(vd, _vd, &head, node) {
		list_del(&vd->node);
		dmaengine_desc_clear_reuse(&vd->tx);
		achan->vc.desc_free(vd);
	}

	return 0;
}

/*
 * qcom_adm_program_crci_ee0 - one-shot CRCI_CTL write to EE=0 window.
 *
 * On APQ8060/MSM8660 (Tenderloin) CRCI_CTL writes to EE=1 are silently
 * dropped; the live register lives at EE=0. The bootloader pre-programs
 * EE=0 for peripherals it enables (eMMC CRCI=1, SDC, NAND). Peripherals
 * the bootloader never touched — QCE crypto CRCI=4 (CE_IN), 5 (CE_OUT)
 * and the CE_HASH CRCI — read as zero at both EE windows after boot.
 * Without a valid EE=0 entry the CRCI handshake never fires: ADM pushes
 * the first burst into the QCE FIFO and then stalls indefinitely waiting
 * for a flow-control assertion that never comes.
 *
 * @chan:     any ADM DMA channel on the target controller — only used to
 *            resolve the adm_device pointer; the CRCI to program does NOT
 *            need to be (and typically isn't) this channel's own CRCI.
 *            QCE uses two channels (rx/tx) but needs three CRCIs programmed
 *            (CE_IN, CE_OUT, CE_HASH); pass any one channel for all three
 *            calls.
 * @crci:     CRCI number to program (1..15; CRCI 0 is "no CRCI").
 * @crci_val: value to write — typically (mux_sel | blk_size).
 *
 * Also seeds the per-CRCI write-cache so the per-transfer adm_start_dma
 * path does not overwrite this value on the next submission.
 *
 * Call once at probe while the channel is idle. Writing mid-transfer
 * corrupts the in-flight burst.
 */
int qcom_adm_program_crci_ee0(struct dma_chan *chan, u32 crci, u32 crci_val)
{
	struct adm_chan *achan;
	struct adm_device *adev;

	if (!chan || !chan->device)
		return -EINVAL;

	achan = to_adm_chan(chan);
	adev  = achan->adev;

	if (!crci || crci >= ARRAY_SIZE(adev->crci_ctl_cache))
		return -EINVAL;

	writel(crci_val, adev->regs + ADM_CRCI_CTL(crci, 0));
	adev->crci_ctl_cache[crci] = crci_val;
	adev->crci_ctl_cache_valid |= BIT(crci);

	dev_info(adev->dev,
		 "ADM program_crci_ee0: chan=%d crci=%u val=0x%x (cache seeded)\n",
		 achan->id, crci, crci_val);
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
		achan->exec_func = config->exec_func;
		achan->exec_user = config->exec_user;
		achan->swap_bytes = config->swap_bytes;
		achan->swap_shorts = config->swap_shorts;
	}
	spin_unlock_irqrestore(&achan->vc.lock, flag);

	dev_dbg(achan->adev->dev,
		"ADM slave_config: chan=%d device_fc=%d crci=%d "
		"src_maxburst=%d dst_maxburst=%d src_addr_width=%d dst_addr_width=%d\n",
		achan->id, cfg->device_fc, achan->crci,
		cfg->src_maxburst, cfg->dst_maxburst,
		cfg->src_addr_width, cfg->dst_addr_width);

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
		 * the EE=0 window with the correct priority + security-domain
		 * (SD) fields. Verified by /dev/mem dump of running webOS
		 * 2.6.35-palm (reports/adm-investigation/webos-live-dump.txt):
		 *   ch2 (sdcc1 eMMC, CRCI 1) = 0x000008D5 (priority=5, SD=1)
		 *   ch5 (sdcc4 WiFi, CRCI 5) = 0x000008D6 (priority=6, SD=1)
		 * SD layout: bit 4 = SD bit 0, bit 5 = SD bit 1, bit 14 = SD
		 * bit 2.  Priority bits 0-3.  Bits 26-28 are per-channel
		 * attribute bits (set on ADM0 ch0-3 audio path only, NOT on
		 * any ADM1 SDCC channel; not SD bits).
		 *
		 * Bootloader explicitly gives sdcc4 (WiFi) one priority level
		 * above sdcc1 (eMMC), so mainline benefits from this routing
		 * without touching the registers.
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
	 * Per-ADM-controller submit serialization is needed ONLY for
	 * channels that supply an exec_func — that hook drives the
	 * peripheral's own MMIO (mmci writes DATATIMER+DATACTRL+CMD before
	 * the CMD_PTR fires) and has to be atomic with the ADM start.
	 * Without an exec_func, the only state touched here is per-CRCI
	 * CRCI_CTL (unique per peripheral) and the channel-local CMD_PTR
	 * register — no shared MMIO with any other channel.
	 *
	 * Holding the lock unconditionally was the original design but
	 * starved a real workload: BT-UART RX restarts at ~192/sec via
	 * msm_complete_rx_dma -> msm_start_rx_dma -> adm_issue_pending,
	 * none of which set exec_func.  Each BT-RX restart was blocking
	 * an in-flight mmci submission (exec_func writes 5 SDCC regs over
	 * ~6-8 µs of the slow APB bus) — long enough for SDCC's 64 B RX
	 * FIFO to overflow before the next ADM CMD_PTR drains it.  Result:
	 * RXOVERRUN-marked-as-fabric → DATATIMEOUT → CMD12/13/23 timeout
	 * cascade → eMMC card stuck busy → HW-reset -110 → full eMMC death.
	 *
	 * Gate the lock on exec_func presence: BT RX submits run unlocked,
	 * mmci submits keep their atomic CRCI_CTL+exec_func+CMD_PTR window.
	 * IRQs are saved/restored only when we take the lock so adm_start_dma
	 * can still be called from the IRQ completion path safely.
	 *
	 * See memory note: project_adm_submit_lock_starvation.
	 */
	{
	unsigned long submit_flags;
	bool need_submit_lock = async_desc->exec_func != NULL;
	/*
	 * Per-CRCI lock: CRCI 1 (eMMC sdcc1) and CRCI 5 (WiFi sdcc4)
	 * each get their own spinlock so concurrent submissions don't
	 * block each other across mmci's slow ~6-8 µs SDCC-reg-write
	 * window.  See the submit_lock[] declaration comment for the
	 * full rationale.  Mask to 4 bits because CRCI is a 4-bit field
	 * in CRCI_CTL.
	 */
	unsigned int lock_idx = async_desc->crci & 0xf;

	if (need_submit_lock)
		spin_lock_irqsave(&adev->submit_lock[lock_idx], submit_flags);

	/* set the crci block size if this transaction requires CRCI */
	if (async_desc->crci) {
		u32 crci_val;
		u32 blk_size = async_desc->blk_size;

		/*
		 * SDCC CRCIs use the legacy webOS msm_dmov block size of 1
		 * (half-FIFO, 32 B), matching adm1_crci_conf which sets
		 * DMOV_CRCI_CONF(sd=1, blk=1) for both SDCC CRCIs:
		 *   CRCI 1 = eMMC sdcc1, CRCI 5 = WiFi sdcc4.
		 * The half-FIFO granularity matches the SDCC raising CRCI at its
		 * half-full FIFO threshold. Mainline deriving it from the 64 B
		 * burst (adm_get_blksize -> 2) over-paces the handshake; the
		 * ADM/FIFO drift accumulates over long transfers and latches
		 * DATACRCFAIL — on WiFi the 128 B HTC mailbox read, and on eMMC
		 * the 128 KB (256-block) reads that then cascade into
		 * cmd12/cmd13 timeouts and a card re-init.
		 *
		 * NOTE: this is the CRCI block size ONLY. The DMA-completion
		 * callback workaround stays WiFi-only (mmc1, reads) — applying
		 * THAT to eMMC is what destabilised it before, not this value.
		 * All other CRCIs (crypto CE, etc.) keep the computed value.
		 */
		/*
		 * Defense-in-depth re-assert of the SDCC blk_size=1 override
		 * for any descriptor whose computed blk_size is 2 (= burst 64
		 * = SDCC FIFO).  The slave-prep path (adm_prep_slave_sg) is
		 * the primary site for this; this catches paths that bypass
		 * it.  Same rationale: SDCC raises CRCI at half-FIFO (32 B),
		 * not full-FIFO, so the handshake granularity is blk_size=1.
		 * QCE / other 16-byte-CRCI peripherals have blk_size=0 and
		 * are untouched.
		 *
		 * On APQ8060/tenderloin the affected CRCIs are 1 (sdcc1
		 * eMMC) and 5 (sdcc4 WiFi).  The earlier CRCI-only override
		 * gated on `crci == 1` missed CRCI 5 and let the AR6003
		 * mailbox writes corrupt: HTC service-connect TX returned 0
		 * at the SDIO layer but the chip ignored the packet (bytes
		 * mis-paced through the FIFO) and the WPA 4-way handshake
		 * EAPOL frames timed out.
		 */
		if (blk_size == 2)
			blk_size = 1;

		/*
		 * Fix #4: CRCI_CTL is in the ADM CONFIG register bank, which on
		 * MSM8660/APQ8060 is live at EE=0 (verified by /dev/mem on the
		 * working webOS kernel: CRCI_CTL reads back 0x1 / blk_size=1 for
		 * crci1 (eMMC) and crci5 (WiFi) at the EE=0 aperture, while the
		 * EE=1 aperture is a dead mirror reading 0). Writing at adev->ee
		 * (=1) silently dropped this block-size programming, so the SDCC
		 * channels ran on whatever blk_size the bootloader left in EE=0 ->
		 * mis-paced ADM<->FIFO CRCI handshake -> the residual eMMC/WiFi
		 * "error during DMA transfer". Write at the live EE=0 aperture.
		 * (Contrast: the per-channel command bank - CMD_PTR/RSLT/FLUSH -
		 * is live at EE=1; only the CONFIG bank CONF/CRCI_CTL is at EE=0.)
		 */
		crci_val = async_desc->mux | blk_size;

		/*
		 * Fix 2: skip the CRCI_CTL MMIO write when the cached value
		 * matches the desired value.  CRCI 1 (eMMC) and CRCI 9 (BT)
		 * both write the same constants on every submit so 95%+ of
		 * these MMIO writes are no-ops.  See
		 * project_adm_submit_lock_starvation for the rationale.
		 *
		 * The cache is updated under submit_lock when need_submit_lock
		 * is true.  When BT RX runs without the lock the read is racy
		 * but harmless: at worst we do one redundant write, and the
		 * value being written would be identical to whatever a racing
		 * mmci start writes (same CRCI = same blk_size+mux constants).
		 */
		{
			bool need_write = !(adev->crci_ctl_cache_valid &
					    BIT(async_desc->crci)) ||
					  adev->crci_ctl_cache[async_desc->crci] != crci_val;
			u32 cached = adev->crci_ctl_cache[async_desc->crci];
			u32 reg_pre = 0;
			bool is_qce = (async_desc->crci == 4 ||
				       async_desc->crci == 5 ||
				       async_desc->crci == 15);

			/* DEBUG: live CRCI_CTL register readback for QCE CRCIs */
			if (is_qce)
				reg_pre = readl_relaxed(adev->regs +
							ADM_CRCI_CTL(async_desc->crci, 0));

			if (need_write) {
				writel(crci_val,
				       adev->regs + ADM_CRCI_CTL(async_desc->crci, 0));
				adev->crci_ctl_cache[async_desc->crci] = crci_val;
				adev->crci_ctl_cache_valid |= BIT(async_desc->crci);
			}

			if (is_qce) {
				u32 reg_post = readl_relaxed(adev->regs +
						ADM_CRCI_CTL(async_desc->crci, 0));

				dev_info(adev->dev,
					 "ADM start_dma QCE crci=%u: crci_val=0x%x cached=0x%x reg_pre=0x%x reg_post=0x%x %s\n",
					 async_desc->crci, crci_val, cached,
					 reg_pre, reg_post,
					 need_write ? "WROTE" : "skipped");
			}
		}
	}

	/*
	 * Order the CRCI_CTL write before exec_func + CMD_PTR.  Full wmb()
	 * (DSB on ARMv7) is overkill — these are MMIO writes via writel
	 * which already emits dmb_st; dma_wmb is the lighter device-write
	 * barrier and is sufficient here.
	 */
	dma_wmb();

	/*
	 * Peripheral pre-submit hook (legacy msm_dmov exec_func). Called
	 * with submit_lock held, IRQs off, right before the CMD_PTR write.
	 * Lets the consumer driver (mmci, etc.) commit its own MMIO setup
	 * atomically with the ADM start.
	 */
	if (async_desc->exec_func)
		async_desc->exec_func(async_desc->exec_user);

	dev_dbg(adev->dev, "ADM start_dma: chan=%d crci=%d cmd_ptr=0x%08x len=%zu\n",
		achan->id, async_desc->crci,
		(u32)(ALIGN(async_desc->dma_addr, ADM_DESC_ALIGN) >> 3),
		async_desc->length);

	/* DEBUG: log descriptor + CPL details for QCE channels */
	if (async_desc->crci == 4 || async_desc->crci == 5 ||
	    async_desc->crci == 15) {
		u32 *cple = (u32 *)async_desc->cpl;
		struct adm_desc_hw_box *box =
			(struct adm_desc_hw_box *)((u8 *)async_desc->cpl +
						   ADM_DESC_ALIGN);

		dev_info(adev->dev,
			 "ADM QCE descriptor crci=%u pool_idx=%d dma_addr=0x%llx len=%zu cpl[0]=0x%08x cpl[1]=0x%08x box: cmd=0x%08x src=0x%08x dst=0x%08x row_len=0x%08x num_rows=0x%08x\n",
			 async_desc->crci, async_desc->pool_index,
			 (u64)async_desc->dma_addr, async_desc->length,
			 cple[0], cple[1],
			 box->cmd, box->src_addr, box->dst_addr,
			 box->row_len, box->num_rows);
	}

	/*
	 * Wait for CMD_PTR_RDY before writing CMD_PTR (legacy webOS parity).
	 *
	 * Legacy webOS arch/arm/mach-msm/dma.c msm_dmov_enqueue_cmd_ext()
	 * and start_ready_cmds() always read DMOV_STATUS(ch) and gate the
	 * CMD_PTR write on bit 0 (CMD_PTR_RDY).  If the channel is not
	 * ready, the command goes onto a ready_commands list and the next
	 * RSLT_VALID IRQ drains it.
	 *
	 * Earlier upstream qcom_adm.c iterations replaced that deferred
	 * pattern with a read + dev_warn_once "tripwire" only -- on the
	 * empirical observation that under typical workloads
	 * CMD_PTR_RDY was set every time the IRQ handler reached this
	 * point.  But high-throughput pinned-rate workloads (the MSM8x60
	 * NoC interconnect series pins fabric clocks at INT_MAX) shrink
	 * the wall-clock window between RSLT_VALID asserting and the
	 * channel pipeline state machine clearing CMD_PTR_RDY, and the
	 * tripwire starts firing -- writing CMD_PTR with CMD_PTR_RDY = 0
	 * leaves the channel with two partial chains and FLUSH fires with
	 * STATE5 = 0x00000003 ("drain stage 3"), the exact SDCC drain
	 * stall captured in project_adm_flush_state_decode_2026_06_14.
	 *
	 * Restore a bounded busy-wait: poll CMD_PTR_RDY for up to 100 us
	 * before giving up.  In hardirq context this is short enough not
	 * to disturb other IRQs (well under the standard 250 us IRQ-off
	 * budget for ARM platforms), and the one-shot dev_warn that
	 * follows lets us see if any consumer's workload still needs the
	 * deferred-list pattern.  See feedback_no_devwarn_in_adm_irq for
	 * the broader "no unbounded dev_warns inside ADM hardirq" rule;
	 * the bounded udelay loop here is the kind of wait that rule
	 * always permitted.
	 */
	{
		u32 status;
		int us;

		for (us = 0; us < 100; us++) {
			status = readl_relaxed(adev->regs +
					ADM_CH_STATUS_SD(achan->id, adev->ee));
			if (status & ADM_CH_STATUS_CMD_PTR_RDY)
				break;
			udelay(1);
		}

		if (!(status & ADM_CH_STATUS_CMD_PTR_RDY)) {
			achan->cmd_ptr_not_rdy_count++;

			if (!achan->cmd_ptr_not_rdy_logged) {
				achan->cmd_ptr_not_rdy_logged = 1;
				dev_warn(adev->dev,
					"ADM ch%d STATUS_SD=0x%08x: CMD_PTR_RDY not set after %d us busy-wait\n",
					achan->id, status, us);
			}
		}
	}

	/*
	 * Write CMD_PTR with the ADM_CPLE_CMD_PTR_LIST flag (bit 29) so the
	 * ADM treats async_desc->dma_addr as a "command pointer list" pointer
	 * (i.e. address of a CPLE array whose entries each point to a
	 * descriptor chain), matching legacy webOS msm_dmov dma.c:465:
	 *
	 *   hdr.cmdptr = DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(cmdptr_busaddr);
	 *   writel(cmd->cmdptr, DMOV_REG(DMOV_CMD_PTR(ch), adm));
	 *
	 * where DMOV_CMD_PTR_LIST = (1 << 29) = ADM_CPLE_CMD_PTR_LIST here.
	 *
	 * Without the bit, ADM treats the address as a "single command"
	 * pointer and tries to parse the bytes at that address as the four
	 * words of a TYPE_SINGLE descriptor.  Our CPL buffer layout starts
	 * with the CPLE word (ADM_CPLE_LP | (desc_addr>>3)) which has bit 31
	 * set; ADM reads that as CMD_LC + TYPE_SINGLE + garbage CRCI bits
	 * and limps along by accidentally fetching our actual BOX descriptors
	 * from the address bits.  It "mostly works" for solo workloads but
	 * loses robustness under fabric contention: the ADM ch2 (eMMC) FLUSH
	 * stall captured on 2026-06-14 (FLUSH_STATE0=0x8000c003 mid-transfer
	 * with STATE2=0 indicating no next-cmd queued) is consistent with
	 * the single-command-mode pipeline running out at descriptor stage
	 * 3 (STATE5=3) instead of traversing the full CPL list.
	 *
	 * See project_adm_flush_state_decode_2026_06_14 memory.
	 */
	writel(ADM_CPLE_CMD_PTR_LIST |
	       (ALIGN(async_desc->dma_addr, ADM_DESC_ALIGN) >> 3),
	       adev->regs + ADM_CH_CMD_PTR(achan->id, adev->ee));

	/*
	 * EXPERIMENTAL: On MSM8660/APQ8060 (Tenderloin), also write CMD_PTR
	 * to EE=0. Most peripherals work with EE=1-only writes, but QCE crypto
	 * (channels 2-3 on ADM0) seems to require EE=0 writes as well.
	 * This is a workaround pending proper root-cause analysis.
	 */
	if (adev->ee == 1) {
		writel(ADM_CPLE_CMD_PTR_LIST |
		       (ALIGN(async_desc->dma_addr, ADM_DESC_ALIGN) >> 3),
		       adev->regs + ADM_CH_CMD_PTR(achan->id, 0));
	}

	if (need_submit_lock)
		spin_unlock_irqrestore(&adev->submit_lock[lock_idx], submit_flags);
	}

	/*
	 * Arm the per-channel watchdog so a wedged channel is recovered
	 * even when neither RSLT_VALID nor FLUSH ever fires.  Disarmed by
	 * the IRQ handler in the normal-completion path; fires
	 * adm_watchdog_timeout() otherwise (see that function for the
	 * recovery sequence).
	 *
	 * Only memory-paced submissions get a watchdog.  Peripheral-paced
	 * RX channels (e.g. UART RX on msm_serial, BT-UART RX) legitimately
	 * wait indefinitely until the remote peer transmits, with no
	 * meaningful upper bound on completion time -- a fixed timeout
	 * just produces false-positive recoveries that the consumer
	 * resubmits, looping at the watchdog rate and starving the CPU
	 * (observed live: ch7 HSUART1_RX + ch8 BT-UART_RX both retrying
	 * every 510 ms before any peer was transmitting, driving CPU0
	 * into soft-lockup during initramfs).  Use exec_func presence as
	 * the discriminator: the consumers that benefit from wedge
	 * recovery (mmci eMMC / WiFi) all set exec_func to perform atomic
	 * peripheral-MMIO setup at submit time; UART RX, BT-UART RX,
	 * qpic NAND, qce crypto, and similar peripheral-paced or
	 * external-data-paced consumers do not.
	 */
	if (async_desc->exec_func)
		mod_timer(&achan->watchdog,
			  jiffies + msecs_to_jiffies(ADM_WATCHDOG_TIMEOUT_MS));
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
	u32 srcs, i;
	struct adm_async_desc *async_desc;

	srcs = readl_relaxed(adev->regs +
			ADM_SEC_DOMAIN_IRQ_STATUS(adev->ee));

	dev_dbg(adev->dev, "ADM IRQ: srcs=0x%08x ee=%d\n", srcs, adev->ee);

	/*
	 * Iterate only the set channel bits in srcs.  Fixed 16-iteration
	 * loop wasted ~14 branch checks per IRQ on tenderloin where only
	 * 1-2 channels complete at a time.
	 */
	while (srcs) {
		struct adm_chan *achan;
		u32 status, result;

		i = __ffs(srcs);
		srcs &= ~BIT(i);
		achan = &adev->channels[i];

		status = readl_relaxed(adev->regs +
				       ADM_CH_STATUS_SD(i, adev->ee));

		/* if no result present, skip */
		if (!(status & ADM_CH_STATUS_VALID))
			continue;

		result = readl_relaxed(adev->regs +
			ADM_CH_RSLT(i, adev->ee));

		/* no valid results, skip */
		if (!(result & ADM_CH_RSLT_VALID))
			continue;

		/*
		 * Fix #2 (lightweight): surface the FLUSH state for visibility.
		 * Legacy msm_dmov read FLUSH0..5 (fill_errdata) and handed the
		 * partial-transfer state to the client. Our consumers recover
		 * their byte count from peripheral HW counters (UART
		 * UARTDM_RX_TOTAL_SNAP, SDCC DATACNT), not the dmaengine residue,
		 * so we don't compute a residue here — but log FLUSH_STATE0 (read
		 * from the EE=1 command bank, where it is live on MSM8660) so a
		 * partial/flushed transfer is observable rather than silently
		 * reported as a clean DONE. Pairs with Fix #1's graceful flush.
		 */
		if (result & ADM_CH_RSLT_FLUSH) {
			u32 fstate = readl_relaxed(adev->regs +
					ADM_CH_FLUSH_STATE0(i, adev->ee));

			/*
			 * For the SDCC channels (ADM1 ch2=eMMC/CRCI1,
			 * ch5=WiFi/CRCI5) log FLUSH at warn level so it can be
			 * correlated by timestamp with the mmci DIAG[DATACRCFAIL]
			 * dump. A FLUSH on the SDCC channel coincident with an
			 * SDCC RXOVERRUN means the ADM stopped draining the FIFO
			 * (fabric/EBI drain starvation) rather than a card error.
			 *
			 * One-shot per channel: read all 6 FLUSH_STATE registers
			 * (matching legacy webOS msm_dmov fill_errdata) and emit
			 * in a SINGLE dev_warn so we don't burn IRQ latency with
			 * multiple printks (see feedback_no_devwarn_in_adm_irq:
			 * b97ca25f4615 emitted 2-3 dev_warns per stall, blocked
			 * the ADM IRQ for 15-35 ms, made mmci miss its busy-poll
			 * deadline -> eMMC wedge).  Two printks per first-time
			 * stall is still much cheaper than the 3-line loop, and
			 * the one-shot caps the cost on later stalls.
			 */
			if (i == 2 || i == 5) {
				bool dump_full = !achan->flush_state_dumped;

				if (dump_full) {
					u32 fs1 = readl_relaxed(adev->regs +
						ADM_CH_FLUSH_STATE1(i, adev->ee));
					u32 fs2 = readl_relaxed(adev->regs +
						ADM_CH_FLUSH_STATE2(i, adev->ee));
					u32 fs3 = readl_relaxed(adev->regs +
						ADM_CH_FLUSH_STATE3(i, adev->ee));
					u32 fs4 = readl_relaxed(adev->regs +
						ADM_CH_FLUSH_STATE4(i, adev->ee));
					u32 fs5 = readl_relaxed(adev->regs +
						ADM_CH_FLUSH_STATE5(i, adev->ee));

					achan->flush_state_dumped = 1;
					dev_warn(adev->dev,
						"ADM-DIAG ch%d FLUSH result=0x%08x STATE 0..5: %08x %08x %08x %08x %08x %08x (one-shot)\n",
						i, result,
						fstate, fs1, fs2, fs3, fs4, fs5);
				} else {
					dev_warn(adev->dev,
						"ADM-DIAG ch%d FLUSH result=0x%08x FLUSH_STATE0=0x%08x (SDCC drain stalled)\n",
						i, result, fstate);
				}
			} else {
				dev_dbg(adev->dev,
					"ADM ch%d FLUSH result=0x%08x FLUSH_STATE0=0x%08x\n",
					i, result, fstate);
			}
		}

		/*
		 * Flag error only if ERR bit is set (real hardware error).
		 * Flush-only results are expected behavior - UART drivers
		 * use dmaengine_terminate_all() to retrieve partial RX data.
		 */
		if (result & ADM_CH_RSLT_ERR) {
			achan->error = 1;
			dev_err(adev->dev,
				"ADM DMA error: chan=%d result=0x%08x (err=%d flush=%d tpd=%d)\n",
				i, result,
				!!(result & ADM_CH_RSLT_ERR),
				!!(result & ADM_CH_RSLT_FLUSH),
				!!(result & ADM_CH_RSLT_TPD));
		}

		/*
		 * Plain spin_lock — we are in hardirq, IRQs are already
		 * disabled by the CPU; spin_lock_irqsave's save/restore is
		 * wasted work here.
		 */
		spin_lock(&achan->vc.lock);

		/*
		 * Disarm the watchdog now that the channel has reported.
		 * Holding vc.lock serialises us against a concurrent
		 * adm_watchdog_timeout(): if it already cleared curr_txd
		 * the timer_delete is a no-op and the !async_desc branch
		 * below skips the duplicate completion.
		 */
		timer_delete(&achan->watchdog);

		async_desc = achan->curr_txd;

		achan->curr_txd = NULL;

		if (async_desc) {
			dma_async_tx_callback callback = NULL;
			void *callback_param = NULL;

			/*
			 * Replicate webOS msm_dmov synchronous completion pattern:
			 * complete cookie, invoke callback, recycle descriptor, and
			 * start next DMA - all in hardirq without vchan tasklet
			 * deferral.  MMCI expects immediate notification and can't
			 * tolerate vchan's deferred cleanup racing with next
			 * transfer setup in adm_start_dma().
			 *
			 * KNOWN ABI NOTE (Sashiko High #1): mainline dmaengine
			 * documents tx callbacks as running in softirq / tasklet
			 * context.  Consumers other than mmci's atomic_submit
			 * (msm_serial UART RX, qpic NAND, qce crypto) call back
			 * via the standard dmaengine_desc_callback / vchan path
			 * and may set up locking against soft-IRQ rather than
			 * hard-IRQ.  We call the callback OUTSIDE the channel's
			 * vc.lock to allow lock nesting, but the caller is still
			 * in hardirq context.  Long-term the right split is to
			 * gate the inline callback on a per-channel flag (e.g.
			 * achan->atomic_completion = exec_func != NULL) and
			 * defer through vchan_complete() for non-atomic-submit
			 * consumers; tracked as a follow-up so this Sashiko-pass
			 * stays scoped to the fixes that don't require auditing
			 * every existing consumer's lock-class assumptions.
			 */
			dma_cookie_complete(&async_desc->vd.tx);

			if (async_desc->vd.tx.callback) {
				callback = async_desc->vd.tx.callback;
				callback_param = async_desc->vd.tx.callback_param;
			}

			/*
			 * Pool descriptors are returned to the free list inline
			 * (no allocator hop, no sleeping).  Dynamic descriptors
			 * MUST be queued to the vchan tasklet because their
			 * cleanup path (adm_dma_free_desc) calls
			 * dma_free_coherent(), which warns when invoked from
			 * hardirq context (kernel/dma/mapping.c:689 WARN_ON in
			 * dma_free_attrs).
			 *
			 * We already inline-call the consumer's tx.callback
			 * below; NULL out vd.tx.callback first so vchan_complete()
			 * in the tasklet doesn't call it a SECOND time when it
			 * processes desc_completed (Sashiko Critical #3 was the
			 * double-completion -- 3272cece1bad fixed it by freeing
			 * dynamic inline, but that introduced the
			 * dma_free_coherent-in-hardirq WARN; the correct split
			 * is: inline-call the callback, defer cleanup to the
			 * tasklet, but don't let the tasklet re-fire the
			 * callback).
			 */
			if (async_desc->pool_index >= 0) {
				spin_lock(&adev->pool_lock);
				list_add_tail(&async_desc->pool_node,
					      &adev->desc_free_list);
				spin_unlock(&adev->pool_lock);
			} else {
				/* Defer free to tasklet; suppress double callback. */
				async_desc->vd.tx.callback = NULL;
				async_desc->vd.tx.callback_param = NULL;
				list_add_tail(&async_desc->vd.node,
					      &achan->vc.desc_completed);
				tasklet_schedule(&achan->vc.task);
			}

			/* kick off next DMA */
			adm_start_dma(achan);

			/* Invoke callback after starting next DMA */
			if (callback) {
				spin_unlock(&achan->vc.lock);
				callback(callback_param);
				spin_lock(&achan->vc.lock);
			}
		}

		spin_unlock(&achan->vc.lock);
	}

	return IRQ_HANDLED;
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

	vchan_init(&achan->vc, &adev->common);
	achan->vc.desc_free = adm_dma_free_desc;
	timer_setup(&achan->watchdog, adm_watchdog_timeout, 0);
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
	 * Per-CRCI submit serialization (was a single per-controller
	 * lock; see the submit_lock[] declaration comment for why this
	 * is split per-CRCI now).  Initialised early so adm_start_dma
	 * can always take whichever CRCI's lock it needs.
	 */
	for (int i = 0; i < ARRAY_SIZE(adev->submit_lock); i++)
		spin_lock_init(&adev->submit_lock[i]);

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
		 * Vote a SUSTAINED EBI floor to keep the ADM->SFAB->AFAB->EBI
		 * path from collapsing while the ADM drains the SDCC FIFOs.
		 * Legacy webOS used clk_set_rate(ebi1_adm_clk, 27) -- a minimal
		 * EBI keep-alive (the heavy data-path vote was the per-SDCC
		 * dfab=64MHz in msm_sdcc, not the ADM).  Vote it as avg_bw too
		 * (not just peak) so the floor holds across RPM active/sleep
		 * contexts during sustained concurrent eMMC + WiFi DMA; with
		 * avg=0 the ADM's own EBI floor lapsed and the drain could
		 * starve.  128 MB/s is comfortably above legacy's keep-alive.
		 *
		 * Gated on the board (root) compat string -- IPQ8064 NAND is
		 * low-bandwidth (single ADM channel, infrequent large bursts)
		 * and was working without any sustained vote in mainline before
		 * this driver gained ICC paths; voting a 128 MB/s floor there
		 * needlessly pins EBI bandwidth.  Long-term should move to
		 * per-SoC OF match data with .data = { .icc_vote_mbps = 128 };
		 * tracked alongside the other per-SoC quirk gates in task #31.
		 */
		if (of_machine_is_compatible("qcom,msm8660") ||
		    of_machine_is_compatible("qcom,apq8060")) {
			ret = icc_set_bw(adev->icc_path, 1024000, 1024000);
			if (ret) {
				dev_err(adev->dev, "failed to set interconnect bandwidth: %d\n", ret);
				goto err_disable_clks;
			}
		}
	}

	/*
	 * ADM1 has two SFAB master ports (PORT0 = mas_port 4, PORT1 = mas_port 5).
	 * The hardware routes channel traffic across both based on its
	 * internal channel -> port mapping.  Without an ICC vote on PORT1,
	 * RPM ARB programs zero bandwidth on it -- which under sustained
	 * concurrent eMMC + WiFi DMA starves whichever channel the HW
	 * routed through PORT1.  Symptom: "ADM-DIAG chX FLUSH ...
	 * (SDCC drain stalled)" + DATACRCFAIL on a 1 MB eMMC read mid-
	 * transfer.  Match PORT1's vote to PORT0 so both ports stay alive.
	 *
	 * The DT exposes this second path via interconnect-names = "memory-p1";
	 * it is optional for backwards compatibility (older DTs without the
	 * second entry simply skip this vote, matching the previous behaviour).
	 */
	adev->icc_path_p1 = devm_of_icc_get(adev->dev, "memory-p1");
	if (IS_ERR(adev->icc_path_p1)) {
		ret = PTR_ERR(adev->icc_path_p1);
		if (ret != -ENODATA && ret != -ENOENT) {
			dev_err(adev->dev,
				"failed to get PORT1 interconnect path: %d\n", ret);
			goto err_disable_clks;
		}
		adev->icc_path_p1 = NULL;
	}
	if (adev->icc_path_p1 &&
	    (of_machine_is_compatible("qcom,msm8660") ||
	     of_machine_is_compatible("qcom,apq8060"))) {
		ret = icc_set_bw(adev->icc_path_p1, 1024000, 1024000);
		if (ret) {
			dev_err(adev->dev,
				"failed to set PORT1 interconnect bw: %d\n", ret);
			goto err_disable_clks;
		}
	}

	/*
	 * Probe-time reset: IPQ8064 (the original upstream user) needs this
	 * to bring the ADM into a known state before any peripheral CRCI is
	 * armed.  MSM8660 / APQ8060 must NOT reset here -- the bootloader
	 * pre-programs CRCI_CTL at the EE=0 aperture for eMMC, NAND, SDC,
	 * WiFi-SDIO, and the reset would wipe those entries; the consumer
	 * peripherals do not currently know how to re-populate them, so the
	 * first CRCI handshake would hang.
	 *
	 * Use the board (root) compat string rather than a new ADM compat
	 * variant for now: the proper long-term fix is to add a per-SoC OF
	 * match entry with .data = { .probe_reset = false } and key off
	 * of_device_get_match_data(); tracked as a follow-up for the
	 * msm8660-adm DT binding series.
	 */
	if (!of_machine_is_compatible("qcom,msm8660") &&
	    !of_machine_is_compatible("qcom,apq8060")) {
		reset_control_assert(adev->clk_reset);
		reset_control_assert(adev->c0_reset);
		reset_control_assert(adev->c1_reset);
		reset_control_assert(adev->c2_reset);

		udelay(2);

		reset_control_deassert(adev->clk_reset);
		reset_control_deassert(adev->c0_reset);
		reset_control_deassert(adev->c1_reset);
		reset_control_deassert(adev->c2_reset);
	}

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

	/* Wire pool cleanup into device-managed unwind so any subsequent
	 * probe-error goto (err_disable_clks, etc.) automatically frees the
	 * pool without an explicit adm_desc_pool_destroy() call on every
	 * error label.
	 */
	ret = devm_add_action_or_reset(&pdev->dev,
				       adm_desc_pool_destroy_action, adev);
	if (ret) {
		dev_err(adev->dev,
			"failed to register pool-destroy devm action: %d\n",
			ret);
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
	 *
	 * On MSM8660/APQ8060 (Tenderloin) the live CH_CONF + RSLT_CONF
	 * windows are at EE=0 (offset 0x240 / 0x300), not at adev->ee=1
	 * where the kernel writes RSLT_CONF. Reading at adev->ee=1
	 * returns all zeros and masks the actual bootloader-programmed
	 * priorities. Verified by live /dev/mem dump of running webOS
	 * 2.6.35-palm: EE=0 holds the truth, EE=1/2/3 read 0.
	 *
	 * Real values on Tenderloin ADM1 (sample, EE=0):
	 *   ch2 (sdcc1 eMMC, CRCI 1) = 0x000008D5 (priority=5, SD=1)
	 *   ch5 (sdcc4 WiFi, CRCI 5) = 0x000008D6 (priority=6, SD=1)
	 *
	 * So bootloader explicitly gives sdcc4 (WiFi) one priority level
	 * higher than sdcc1 (eMMC) on the ADM channel arbiter. The driver
	 * does not need to touch CH_CONF -- bootloader has it covered.
	 *
	 * Read at EE=0 here regardless of adev->ee, so the diagnostic is
	 * actually useful on this SoC.
	 */
	for (i = 0; i < ADM_MAX_CHANNELS; i++) {
		u32 ch_conf, rslt_conf;

		ch_conf = readl_relaxed(adev->regs + ADM_CH_CONF(i, 0));
		rslt_conf = readl_relaxed(adev->regs +
					  ADM_CH_RSLT_CONF(i, 0));
		dev_dbg(adev->dev,
			"ADM ch%d (EE=0 live): CH_CONF=0x%08x RSLT_CONF=0x%08x\n",
			i, ch_conf, rslt_conf);
	}

	ret = devm_request_irq(adev->dev, adev->irq, adm_dma_irq,
			       IRQF_NOBALANCING, "adm_dma", adev);
	if (ret)
		goto err_disable_clks;

	/*
	 * Pin the ADM completion IRQ to CPU1 when available.
	 *
	 * Tenderloin (APQ8060) shares adm_dma1 between sdcc1 (eMMC, DMA) and
	 * sdcc4 (WiFi, PIO for BMI). When both IRQs land on CPU0, the ADM
	 * hardirq + tasklet path delays the sdcc4 PIO TXFIFOHALFEMPTY refill
	 * IRQ enough that DPSM underflows the 64-byte FIFO and the AR6003
	 * BMI write times out. Moving the ADM IRQ to the second core lets the
	 * two run in parallel.
	 */
	if (num_online_cpus() > 1 && cpu_online(1)) {
		int aff_ret = irq_set_affinity_and_hint(adev->irq,
							cpumask_of(1));
		if (aff_ret)
			dev_warn(adev->dev,
				 "Failed to pin ADM IRQ to CPU1: %d\n",
				 aff_ret);
	}

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
		goto err_disable_clks;
	}

	ret = of_dma_controller_register(pdev->dev.of_node, adm_dma_xlate,
					 &adev->common);
	if (ret)
		goto err_unregister_dma;

	return 0;

err_unregister_dma:
	dma_async_device_unregister(&adev->common);
	/* Pool cleanup is now wired via devm_add_action_or_reset so it
	 * runs automatically when devres unwinds.  No err_pool_destroy
	 * label needed.
	 */
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

	/* Descriptor pool is freed via the devm action registered at probe. */

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
