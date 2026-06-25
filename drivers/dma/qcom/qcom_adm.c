// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 */

#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
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
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
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
 * Per-SoC driver_data. Bound to each entry of adm_of_match[]. Lets
 * the driver vary behaviour per compatible string instead of probing
 * the board root compatible at runtime.
 *
 * .name                 - human-readable, for dev_info / debug.
 * .needs_probe_reset    - assert the five reset_controls at probe.
 *                         Required on IPQ806x; MUST NOT be done on
 *                         MSM8660/APQ8060 because the bootloader has
 *                         already programmed CRCI_CTL at EE=0 for the
 *                         peripherals it enabled and a reset would
 *                         wipe them, hanging the first transfer.
 * .icc_bw_kbps_peak/avg - per-path interconnect bandwidth floor. 0 =
 *                         do not vote.  MSM8660/APQ8060 vote 1 GB/s
 *                         on both "memory" and "memory-p1" paths; the
 *                         drain stalls go away.  IPQ8064 (single
 *                         NAND consumer) leaves both at 0.
 * .crci_defaults        - optional per-CRCI defaults applied at probe.
 *                         Supersedes the burst==64 -> blk_size=1
 *                         heuristic; per-SoC table is authoritative.
 *                         Can be overridden per-submit via
 *                         qcom_adm_peripheral_config.
 * .nr_crci_defaults     - number of entries in the array above.
 */
struct adm_crci_default {
	u8 crci;
	u8 blk_size;
	u8 mux;
};

struct adm_soc_data {
	const char *name;
	bool needs_probe_reset;
	/*
	 * On this SoC the live CRCI_CTL registers are mapped at EE=0
	 * even when adev->ee != 0 (verified on APQ8060 Tenderloin via
	 * /dev/mem dump of running webOS: CRCI_CTL writes at EE=1 are
	 * silently dropped). When true the driver writes CRCI_CTL at
	 * EE=0 in the submit path AND exports qcom_adm_program_crci_ee0
	 * to consumers that need to seed CRCI_CTL for peripherals the
	 * bootloader never touched. When false (most SoCs) CRCI_CTL is
	 * written at adev->ee like every other per-channel register.
	 */
	bool crci_ctl_at_ee0;
	u32 icc_bw_kbps_peak;
	u32 icc_bw_kbps_avg;
	const struct adm_crci_default *crci_defaults;
	int nr_crci_defaults;
};

/*
 * MSM8660 / APQ8060 — Tenderloin (HP TouchPad), Dragonboard.
 *
 * Two ADM instances (ADM0 + ADM1).  Bootloader programs CONF +
 * CRCI_CTL at EE=0 for eMMC, NAND, SDC, WiFi; do NOT reset at probe.
 * Vote 1 GB/s on both SFAB master ports — ADM1 PORT1 starves under
 * concurrent eMMC + WiFi otherwise.
 *
 * CRCI defaults pulled from arch/arm/mach-msm/dma.c adm[01]_crci_conf[]
 * in the webOS 2.6.35 kernel + verified against running /dev/mem:
 *   CRCI 1  (eMMC sdcc1)       blk_size=1 (32 B half-FIFO), mux=0
 *   CRCI 5  (WiFi sdcc4)       blk_size=1 (32 B half-FIFO), mux=0
 *   CRCI 4  (CE2 cipher in)    blk_size=0 (16 B),           mux=0
 *   CRCI 15 (CE2 hash out)     blk_size=0 (16 B),           mux=0
 *   CRCI 9  (HSUART1 RX BT)    blk_size=0 (16 B),           mux=0
 *   CRCI 10 (HSUART2 RX TS)    blk_size=0 (16 B),           mux=1
 */
static const struct adm_crci_default adm_msm8660_crcis[] = {
	{ .crci = 1,  .blk_size = 1, .mux = 0 },
	{ .crci = 4,  .blk_size = 0, .mux = 0 },
	{ .crci = 5,  .blk_size = 1, .mux = 0 },
	{ .crci = 9,  .blk_size = 0, .mux = 0 },
	{ .crci = 10, .blk_size = 0, .mux = 1 },
	{ .crci = 15, .blk_size = 0, .mux = 0 },
};

static const struct adm_soc_data adm_msm8660_data = {
	.name = "MSM8660/APQ8060",
	.needs_probe_reset = false,
	.crci_ctl_at_ee0 = true,
	.icc_bw_kbps_peak = 1024000,
	.icc_bw_kbps_avg = 1024000,
	.crci_defaults = adm_msm8660_crcis,
	.nr_crci_defaults = ARRAY_SIZE(adm_msm8660_crcis),
};

/*
 * IPQ8064 — router platform with a single ADM-driven consumer (NAND).
 * Original upstream user of qcom_adm.c; needs probe-time reset to
 * bring the controller into a known state. No sustained interconnect
 * floor needed (NAND is low-bandwidth + bursty).
 */
static const struct adm_soc_data adm_ipq8064_data = {
	.name = "IPQ8064",
	.needs_probe_reset = true,
};

/*
 * Stub for SoCs we haven't characterised yet; behaves like IPQ8064
 * (probe-reset, no interconnect floor, no CRCI defaults).
 */
static const struct adm_soc_data adm_generic_data = {
	.name = "generic",
	.needs_probe_reset = true,
};

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

	/*
	 * Cyclic descriptor. The IRQ handler invokes vchan_cyclic_callback()
	 * for each RSLT_VALID and does NOT remove the descriptor from
	 * curr_txd until terminate_all is called. The ADM auto-walks the
	 * CPL chain after each period completes (the last entry has
	 * CMD_LC set but the CPL has CMD_PTR_LP so the controller loops).
	 */
	bool cyclic;

	/* FLUSH-state snapshot captured by the IRQ handler on FLUSH/ERR. */
	u32 errdata[6];
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

	/*
	 * Optional peripheral state-dump callback, populated from
	 * qcom_adm_peripheral_config.dump_state at slave_config time.
	 * Scheduled by adm_watchdog_timeout (via dump_state_work) so the
	 * consumer can pm_runtime_get_sync, take its own locks, sleep,
	 * etc. — none of which are safe from the watchdog softirq itself.
	 */
	void (*dump_state)(void *dump_user);
	void *dump_user;
	struct work_struct dump_state_work;

	struct list_head node;
	struct timer_list watchdog;

	int error;
	/*
	 * Cookie of the most recent descriptor that was completed via
	 * the RSLT_ERR / RSLT_FLUSH path. adm_tx_status() consults this
	 * to surface DMA_ERROR to consumer queries: dma_cookie_status()
	 * returns DMA_COMPLETE for any cookie advanced past
	 * chan->completed_cookie regardless of how it was completed.
	 *
	 * Set in the err-result handler. Cleared when the channel is
	 * reused (R63 recovery — see adm_recover_channel). Cookies are
	 * monotonic per channel and never reach 0 (dmaengine cycles
	 * DMA_MIN_COOKIE..INT_MAX), so 0 is a safe "no error recorded"
	 * sentinel.
	 *
	 * Earlier revisions of this driver treated a channel as
	 * permanently dead after RSLT_ERR (matching legacy
	 * arch/arm/mach-msm/dma.c:645). That is incorrect: under the
	 * R63 recovery sequence the channel IS reusable after a
	 * documented sequence (FLUSH0=0, CRCI_CTL.RST per attached
	 * CRCI, transition state -> IDLE). Consumers that retry pass
	 * through the recovery path and the channel comes back.
	 */
	dma_cookie_t error_cookie;
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
	 * One-shot diagnostic dump in adm_start_dma() so the first
	 * submit on each channel logs the bracketing register state
	 * (CRCI_CTL, CONF, STATUS_SD, RSLT_CONF, CMD_PTR being written,
	 * CPL first word). Lets a boot-time wedge surface enough state
	 * to diff against a known-good reference without re-running.
	 */
	u8 first_submit_logged;

	/*
	 * One-shot diagnostic in adm_watchdog_timeout(): log once when
	 * the watchdog fires after the IRQ already completed the cookie
	 * (achan->curr_txd == NULL race). Without this, "no watchdog
	 * message" is ambiguous between "watchdog never armed" and "IRQ
	 * ran first" — both leave dmesg silent on the recovery path.
	 */
	u8 watchdog_raced_logged;

	/*
	 * One-shot debug flags: log first time the watchdog timer was
	 * armed for this channel, and first time the timer callback
	 * actually fired. Together they prove or disprove "did the
	 * watchdog ever execute on this channel" when a log gap makes
	 * it ambiguous.
	 */
	u8 watchdog_armed_logged;
	u8 watchdog_fired_logged;

	/*
	 * One-shot diagnostic on the first RSLT_VALID IRQ per channel.
	 * Confirms the ADM hardware actually walked the chain to
	 * completion (result code, FLUSH state). Logging only the first
	 * IRQ keeps the noise down on healthy long-running channels.
	 */
	u8 first_irq_logged;

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
	/*
	 * Physical base of the controller's reg block. Captured at probe
	 * so the IRQ / submit hot paths can distinguish ADM0 (0x18320000)
	 * from ADM1 (0x18420000) on MSM8X60-class SoCs without an extra
	 * platform_get_resource() call. Used by the QCE-channel-specific
	 * EE=0 mirror write in adm_start_dma().
	 */
	resource_size_t reg_phys;
	struct device *dev;
	struct dma_device common;
	struct device_dma_parameters dma_parms;
	struct adm_chan *channels;

	const struct adm_soc_data *soc_data;

	u32 ee;

	/*
	 * Linux owns the MASTER SD when this is true and may write the
	 * per-channel CONF + global CRCI_CONF0/1 + CI_CONF + DBG_ERR
	 * registers. Driven by the qcom,owns-master-sd DT property.
	 * IPQ8064 sets it (qcom,ee = <0>, owns master); MSM8660 leaves
	 * it false (qcom,ee = <1>, bootloader / TZ owns master at EE=0).
	 */
	bool owns_master_sd;

	/*
	 * Bitmap of channels (0..ADM_MAX_CHANNELS-1) this AARM may use.
	 * Bit set => channel is registered with the dmaengine layer and
	 * touchable by IRQ/CMD_PTR paths. Bit clear => modem / TrustZone
	 * / unused; the driver must not write per-channel registers for
	 * that channel.
	 *
	 * Populated at probe from the qcom,channels-aarm DT property,
	 * defaults to "all 16" if the property is absent (back-compat
	 * for IPQ8064 boards predating the binding update).
	 */
	DECLARE_BITMAP(channels_aarm, ADM_MAX_CHANNELS);

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

	/*
	 * Optional debugfs surface. Created at probe if CONFIG_DEBUG_FS
	 * is set; removed at remove. The errors ring holds the last
	 * ADM_ERR_LOG_DEPTH error events with timestamps, captured by
	 * the IRQ handler's RSLT_ERR path.
	 */
#ifdef CONFIG_DEBUG_FS
	struct dentry *dbg_dir;
#endif
	struct adm_err_log_entry {
		u64 ktime_ns;
		u8 channel;
		u8 crci;
		u32 result;
		u32 flush[6];
	} err_log[16];
	u8 err_log_head;	/* next slot to write */
	spinlock_t err_log_lock;
};

#define ADM_ERR_LOG_DEPTH ARRAY_SIZE(((struct adm_device *)0)->err_log)

/*
 * Forward declarations.
 */
static void adm_start_dma(struct adm_chan *achan);

/*
 * MMIO write helpers with master-SD ownership guard.
 *
 * adm_write_aarm() — write to a per-channel register through the SD
 *                    the driver owns (adev->ee). Always allowed.
 * adm_write_master() — write to a master-SD-only register. WARN_ON
 *                    and refuse if Linux does not own the master SD
 *                    on this controller (qcom,owns-master-sd absent
 *                    or false).
 *
 * The current driver consciously avoids writing CH_CONF / CRCI_CONF /
 * CI_CONF / DBG_ERR (see adm_start_dma's CH_CONF skip comment for the
 * rationale on Tenderloin). These helpers exist so any future master-
 * SD write goes through one place; bare writel() calls into the master
 * window are a foot-gun (Tenderloin destabilised on the SEC_DOMAIN
 * RMW pattern that was harmless at EE=1 but destructive at EE=0).
 */
static inline void adm_write_aarm(struct adm_device *adev, u32 offset, u32 val)
{
	writel(val, adev->regs + offset);
}

static inline void adm_write_master(struct adm_device *adev, u32 offset, u32 val)
{
	if (WARN_ON_ONCE(!adev->owns_master_sd)) {
		dev_err_ratelimited(adev->dev,
			"refusing master-SD write at 0x%x: driver does not own master SD on this controller\n",
			offset);
		return;
	}
	writel(val, adev->regs + offset);
}

/*
 * adm_log_error — capture a result/FLUSH snapshot into the per-controller
 * debug ring. Called from the IRQ handler error path under the per-vchan
 * lock; the ring has its own spinlock so the IRQ side is the only writer
 * and the debugfs reader can iterate without racing.
 */
static void adm_log_error(struct adm_device *adev, u8 channel, u8 crci,
			  u32 result, const u32 flush[6])
{
	struct adm_err_log_entry *e;
	unsigned long flags;

	spin_lock_irqsave(&adev->err_log_lock, flags);
	e = &adev->err_log[adev->err_log_head];
	e->ktime_ns = ktime_get_ns();
	e->channel = channel;
	e->crci = crci;
	e->result = result;
	memcpy(e->flush, flush, sizeof(e->flush));
	adev->err_log_head = (adev->err_log_head + 1) % ADM_ERR_LOG_DEPTH;
	spin_unlock_irqrestore(&adev->err_log_lock, flags);
}

#ifdef CONFIG_DEBUG_FS
static int adm_dbg_errors_show(struct seq_file *s, void *unused)
{
	struct adm_device *adev = s->private;
	unsigned long flags;
	int i, slot;

	seq_puts(s, "idx  ts_ns                ch crci result    flush0   flush1   flush2   flush3   flush4   flush5\n");
	spin_lock_irqsave(&adev->err_log_lock, flags);
	for (i = 0; i < ADM_ERR_LOG_DEPTH; i++) {
		struct adm_err_log_entry *e;

		slot = (adev->err_log_head + i) % ADM_ERR_LOG_DEPTH;
		e = &adev->err_log[slot];
		if (!e->ktime_ns)
			continue;
		seq_printf(s,
			   "%2d  %20llu  %2u %4u 0x%08x 0x%06x 0x%06x 0x%06x 0x%06x 0x%06x 0x%06x\n",
			   i, e->ktime_ns, e->channel, e->crci, e->result,
			   e->flush[0], e->flush[1], e->flush[2],
			   e->flush[3], e->flush[4], e->flush[5]);
	}
	spin_unlock_irqrestore(&adev->err_log_lock, flags);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adm_dbg_errors);

static void adm_debugfs_init(struct adm_device *adev)
{
	/*
	 * dev_name(adev->dev) is unique per platform_device and survives
	 * load/unload cleanly. Sits under /sys/kernel/debug/<name>/.
	 */
	adev->dbg_dir = debugfs_create_dir(dev_name(adev->dev), NULL);
	debugfs_create_file("errors", 0400, adev->dbg_dir, adev,
			    &adm_dbg_errors_fops);
}

static void adm_debugfs_remove(struct adm_device *adev)
{
	debugfs_remove_recursive(adev->dbg_dir);
}
#else
static inline void adm_debugfs_init(struct adm_device *adev) { }
static inline void adm_debugfs_remove(struct adm_device *adev) { }
#endif

/*
 * adm_recover_channel — R63 post-error recovery sequence.
 *
 * Called from the IRQ handler RSLT_ERR path. Brings a channel that
 * just reported a hardware error back to a usable state so the
 * consumer's retry on the same channel succeeds.
 *
 * The legacy webOS comment "this does not seem to work, once we get
 * an error the datamover will no longer accept commands" was based on
 * the pre-recovery sequence (just writel(0, FLUSH0)). This adds:
 *
 *   1. FLUSH0 = 0     — clear the latched error state (legacy keeps).
 *   2. CRCI_CTL.RST   — pulse the per-CRCI reset bit for every CRCI
 *                       attached to the failed descriptor, so the
 *                       slave-side FIFO state machine clears too.
 *                       Without this the next handshake stalls.
 *   3. RSLT_CONF rearm — re-write IRQ_EN + FLUSH_EN. RSLT_CONF can
 *                       latch into a non-arm state after consecutive
 *                       errors on some silicon revisions; the cost
 *                       of always doing this is one MMIO write.
 *   4. error_cookie cleared — channel is no longer treated as
 *                       permanently dead by adm_tx_status() / the
 *                       CMD_PTR_RDY bail-out path.
 *
 * Caller MUST hold achan->vc.lock.
 */
static void adm_recover_channel(struct adm_chan *achan, u32 crci_for_desc)
{
	struct adm_device *adev = achan->adev;

	lockdep_assert_held(&achan->vc.lock);

	writel_relaxed(0, adev->regs +
		       ADM_CH_FLUSH_STATE0(achan->id, adev->ee));

	if (crci_for_desc) {
		u32 c = crci_for_desc & 0xf;
		/*
		 * Match the EE-selection logic used in adm_start_dma: on
		 * SoCs whose CRCI_CTL is live at EE=0 (Tenderloin/APQ8060)
		 * the write MUST go to EE=0 regardless of which SD Linux
		 * owns; writing to adev->ee (=1) silently drops the reset
		 * and the FIFO state machine stays stuck. The previous
		 * owns_master_sd gate was wrong: Tenderloin does not set
		 * qcom,owns-master-sd, so the EE=0 fallback never ran and
		 * R63 recovery never actually reached the hardware.
		 */
		u32 ctl_ee = (adev->soc_data && adev->soc_data->crci_ctl_at_ee0)
			     ? 0 : adev->ee;

		writel_relaxed(ADM_CRCI_CTL_RST,
			       adev->regs + ADM_CRCI_CTL(c, ctl_ee));
		adev->crci_ctl_cache_valid &= ~BIT(c);
	}

	writel(ADM_CH_RSLT_CONF_IRQ_EN | ADM_CH_RSLT_CONF_FLUSH_EN,
	       adev->regs + ADM_CH_RSLT_CONF(achan->id, adev->ee));

	achan->error = 0;
	achan->cmd_ptr_not_rdy_logged = 0;
	/*
	 * Do NOT clear achan->error_cookie here. The IRQ handler set
	 * it just before calling us so adm_tx_status() can surface
	 * DMA_ERROR for the failed cookie. It is cleared on the next
	 * successful completion (see vchan_cookie_complete path) or on
	 * an explicit terminate_all + restart.
	 */
}

/*
 * adm_drain_rslt — at probe time, drain any residual RSLT_VALID entries
 * left by the bootloader on each AARM-owned channel. Without this the
 * first real submission on a "dirty" channel delivers a ghost
 * completion before the consumer's descriptor has even fired.
 *
 * Bounded so a stuck VALID bit can't wedge probe.
 */
static void adm_drain_rslt(struct adm_device *adev)
{
	unsigned int ch, drain;

	for_each_set_bit(ch, adev->channels_aarm, ADM_MAX_CHANNELS) {
		for (drain = 0; drain < 8; drain++) {
			u32 status = readl_relaxed(adev->regs +
					ADM_CH_STATUS_SD(ch, adev->ee));

			if (!(status & ADM_CH_STATUS_VALID))
				break;
			(void) readl_relaxed(adev->regs +
					     ADM_CH_RSLT(ch, adev->ee));
		}
	}
}

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

	/*
	 * One-shot diagnostic: prove the timer actually fired for this
	 * channel. Logs once per (channel, boot) so even if everything
	 * downstream of the lock acquisition wedges, we see this.
	 */
	if (!achan->watchdog_fired_logged) {
		achan->watchdog_fired_logged = 1;
		dev_info(adev->dev,
			 "ADM ch%u watchdog FIRED (entering timeout fn)\n",
			 achan->id);
	}

	spin_lock_irqsave(&achan->vc.lock, flags);

	async_desc = achan->curr_txd;
	if (!async_desc) {
		/*
		 * Raced with the RSLT_VALID IRQ handler; nothing to do.
		 * Log once per channel so we can distinguish "watchdog
		 * never armed" from "IRQ ran first" when diagnosing why a
		 * recovery dump is missing in the kernel log.
		 */
		if (!achan->watchdog_raced_logged) {
			achan->watchdog_raced_logged = 1;
			dev_info(adev->dev,
				 "ADM ch%u watchdog: fired AFTER IRQ already completed cookie (race; channel healthy)\n",
				 achan->id);
		}
		spin_unlock_irqrestore(&achan->vc.lock, flags);
		return;
	}

	achan->error = 1;
	achan->curr_txd = NULL;

	/*
	 * Full channel snapshot before we tear down the wedged transfer.
	 * Captures both the CONFIG bank (live at EE=0 on Tenderloin —
	 * CRCI_CTL and CONF) and the per-channel command bank (live at
	 * adev->ee — STATUS, CMD_PTR, RSLT, FLUSH_STATE0..5). Together
	 * these answer "did the channel accept CMD_PTR, did it start
	 * walking the chain, where did it stall, was the CRCI gate
	 * armed correctly" without needing a second flash to add probes.
	 */
	{
		u32 ctl_ee_dbg = (adev->soc_data &&
				  adev->soc_data->crci_ctl_at_ee0)
				 ? 0 : adev->ee;
		u32 status_sd = readl_relaxed(adev->regs +
			ADM_CH_STATUS_SD(achan->id, adev->ee));
		u32 cmd_ptr   = readl_relaxed(adev->regs +
			ADM_CH_CMD_PTR(achan->id, adev->ee));
		u32 rslt      = readl_relaxed(adev->regs +
			ADM_CH_RSLT(achan->id, adev->ee));
		u32 rslt_conf = readl_relaxed(adev->regs +
			ADM_CH_RSLT_CONF(achan->id, adev->ee));
		u32 conf_ee0  = readl_relaxed(adev->regs +
			ADM_CH_CONF(achan->id, 0));
		u32 crci_ctl  = async_desc->crci ?
			readl_relaxed(adev->regs +
				ADM_CRCI_CTL(async_desc->crci, ctl_ee_dbg)) : 0;
		u32 fs0 = readl_relaxed(adev->regs +
			ADM_CH_FLUSH_STATE0(achan->id, adev->ee));
		u32 fs1 = readl_relaxed(adev->regs +
			ADM_CH_FLUSH_STATE1(achan->id, adev->ee));
		u32 fs2 = readl_relaxed(adev->regs +
			ADM_CH_FLUSH_STATE2(achan->id, adev->ee));
		u32 fs3 = readl_relaxed(adev->regs +
			ADM_CH_FLUSH_STATE3(achan->id, adev->ee));
		u32 fs4 = readl_relaxed(adev->regs +
			ADM_CH_FLUSH_STATE4(achan->id, adev->ee));
		u32 fs5 = readl_relaxed(adev->regs +
			ADM_CH_FLUSH_STATE5(achan->id, adev->ee));

		dev_warn_ratelimited(adev->dev,
			"ADM ch%u watchdog: no RSLT_VALID within %u ms — status=0x%08x cmd_ptr=0x%08x rslt=0x%08x rslt_conf=0x%08x conf@ee0=0x%08x crci_ctl[%u]@ee%u=0x%08x flush=%08x %08x %08x %08x %08x %08x\n",
			achan->id, ADM_WATCHDOG_TIMEOUT_MS,
			status_sd, cmd_ptr, rslt, rslt_conf, conf_ee0,
			async_desc->crci, ctl_ee_dbg, crci_ctl,
			fs0, fs1, fs2, fs3, fs4, fs5);
	}

	/*
	 * Snapshot the peripheral's MMIO state SYNCHRONOUSLY here, BEFORE
	 * the abrupt FLUSH below — typically SDCC DATACTRL/MMCISTATUS for
	 * mmci-pl18x. This must be synchronous (not deferred to a
	 * workqueue) to capture the genuine wedge state: for an eMMC read
	 * the consuming thread is still blocked in mmc_wait_for_req at +500
	 * ms (no DMA-done callback on mmc0), so the SDCC clock is on and its
	 * registers are live RIGHT NOW. A deferred workqueue dump races
	 * mmci's own data timeout teardown (which zeroes DATACTRL) and we'd
	 * just see post-teardown DCTL=0. mmci_qcom_dump_state only does
	 * readl + one printk — no locks, no sleep — so it is safe from this
	 * timer-softirq + vc.lock-held context.
	 */
	if (achan->dump_state)
		achan->dump_state(achan->dump_user);

	/* Abrupt flush -- graceful would wait for a response we won't get. */
	writel_relaxed(0, adev->regs + ADM_CH_FLUSH_STATE0(achan->id, adev->ee));

	/*
	 * Stash the wedged cookie BEFORE vchan_cookie_complete so the
	 * consumer's adm_tx_status() call (typically from its DMA
	 * callback) reads DMA_ERROR. Without this the watchdog-rescued
	 * cookie races to DMA_COMPLETE and the consumer thinks a 0-byte
	 * transfer succeeded — silent data loss instead of a clean -EIO
	 * up the stack.
	 */
	achan->error_cookie = async_desc->vd.tx.cookie;

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
		/*
		 * Reset cyclic flag. If a previous user of this pool slot
		 * was a cyclic descriptor and this caller is preparing a
		 * one-shot slave_sg/memcpy, leaving cyclic=true routes the
		 * RSLT_VALID IRQ through vchan_cyclic_callback() — the
		 * cookie never reaches DMA_COMPLETE and the consumer waits
		 * forever. exec_func / exec_user have the same pool-reuse
		 * trap: a stale exec_func from the previous owner will fire
		 * under submit_lock on every subsequent submit on this slot.
		 */
		desc->cyclic = false;
		desc->exec_func = NULL;
		desc->exec_user = NULL;
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
	/*
	 * achan->crci is written under achan->vc.lock by adm_slave_config().
	 * Snapshot the encoded value (mux<<4 | crci) here under the same lock
	 * so a concurrent reconfig cannot race the descriptor-build path.
	 * Reading the field multiple times unprotected would let a torn or
	 * stale value reach the descriptor.
	 */
	u32 chan_crci_enc;
	unsigned long lock_flags;

	if (!is_slave_direction(direction)) {
		dev_err(adev->dev, "invalid dma direction\n");
		return NULL;
	}

	spin_lock_irqsave(&achan->vc.lock, lock_flags);
	chan_crci_enc = achan->crci;
	spin_unlock_irqrestore(&achan->vc.lock, lock_flags);

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
		"ADM prep_slave_sg: chan=%d device_fc=%d chan_crci=0x%x burst=%d dir=%d\n",
		achan->id, achan->slave.device_fc, chan_crci_enc, burst, direction);

	/* if using flow control, validate burst and crci values */
	if (achan->slave.device_fc) {
		blk_size = adm_get_blksize(burst);
		if (blk_size < 0) {
			dev_err(adev->dev, "invalid burst value: %d\n",
				burst);
			return NULL;
		}

		crci = chan_crci_enc & 0xf;
		if (!crci || chan_crci_enc > 0x1f) {
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
	async_desc->mux = (chan_crci_enc & ADM_CRCI_MUX_SEL) ? ADM_CRCI_CTL_MUX_SEL : 0;
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

	/*
	 * This helper exists for SoCs whose CRCI_CTL is mapped at EE=0
	 * even when Linux owns a different SD (APQ8060 / MSM8660). On
	 * any other SoC the EE=0 write would land in another SD's
	 * window and corrupt unrelated state — refuse.
	 */
	if (!adev->soc_data || !adev->soc_data->crci_ctl_at_ee0) {
		dev_err_ratelimited(adev->dev,
			"qcom_adm_program_crci_ee0 refused: SoC does not expose CRCI_CTL at EE=0\n");
		return -EPERM;
	}

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
		/*
		 * Accept the encoded form (legacy webOS macros stuff the
		 * mux bit into the high nibble of crci, e.g.
		 * DMOV_HSUART2_RX_CRCI = (1<<4)+10 = 26) AND the new
		 * separate (crci, mux) form. If config->mux is set, fold
		 * it; otherwise trust the legacy encoding embedded in
		 * config->crci. Either way achan->crci ends up in the
		 * single encoded form that the descriptor-build path
		 * expects.
		 */
		if (config->mux)
			achan->crci = ADM_CRCI_MUX_SEL | (config->crci & 0xf);
		else
			achan->crci = config->crci;
		achan->exec_func = config->exec_func;
		achan->exec_user = config->exec_user;
		achan->dump_state = config->dump_state;
		achan->dump_user  = config->dump_user;
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
			/*
			 * On SoCs with crci_ctl_at_ee0 (Tenderloin / APQ8060)
			 * CRCI_CTL is live in the EE=0 window; the EE-aware
			 * macro target is hardcoded to 0. On other SoCs
			 * CRCI_CTL is in adev->ee like every other per-channel
			 * register. Pick the right window once.
			 */
			u32 ctl_ee = (adev->soc_data && adev->soc_data->crci_ctl_at_ee0)
				     ? 0 : adev->ee;
			bool is_qce = adev->soc_data &&
				      adev->soc_data->crci_ctl_at_ee0 &&
				      (async_desc->crci == 4 ||
				       async_desc->crci == 5 ||
				       async_desc->crci == 15);
			/*
			 * Force-write CRCI_CTL for QCE CRCIs regardless of
			 * cache match (commit 02090386de45 cache-skip drops a
			 * load-bearing side effect the CE2 engine relies on
			 * for per-op pacing on Tenderloin). SDCC / BT / NAND
			 * keep the cache-skip. Gated on crci_ctl_at_ee0 so
			 * non-Tenderloin SoCs don't pay the cost.
			 */
			bool need_write = is_qce ||
					  !(adev->crci_ctl_cache_valid &
					    BIT(async_desc->crci)) ||
					  adev->crci_ctl_cache[async_desc->crci] != crci_val;

			if (need_write) {
				writel(crci_val,
				       adev->regs + ADM_CRCI_CTL(async_desc->crci, ctl_ee));
				adev->crci_ctl_cache[async_desc->crci] = crci_val;
				adev->crci_ctl_cache_valid |= BIT(async_desc->crci);
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
			/*
			 * Writing CMD_PTR into a not-ready channel leaves
			 * the pipeline with two partial chains and the next
			 * FLUSH IRQ returns the canonical SDCC drain-stall
			 * signature FLUSH STATE0=0x8000c003 STATE5=0x00000003
			 * (project_adm_flush_state_decode_2026_06_14).
			 * Mirror legacy webOS arch/arm/mach-msm/dma.c
			 * msm_dmov_enqueue_cmd_ext() and NOT write CMD_PTR
			 * into a channel that has not signalled readiness.
			 *
			 * Two sub-cases on the bail-out:
			 *
			 *  - achan->error_cookie set -- channel is in known-
			 *    dead state (the err-result IRQ stashed a prior
			 *    cookie and the channel cannot accept commands,
			 *    per legacy arch/arm/mach-msm/dma.c:645).  Don't
			 *    strand the descriptor: stash this cookie too,
			 *    clear curr_txd, complete via vchan_cookie_complete
			 *    so the consumer's callback fires from the vchan
			 *    tasklet (softirq) and reads DMA_ERROR via
			 *    adm_tx_status().  mmci_qcom_dma_complete()'s
			 *    DMA_ERROR path sets data->error = -EIO, runs
			 *    mmci_stop_data() (DATACTRL = 0, unpriming the
			 *    SDCC half-armed by exec_func()) and ends the
			 *    request.  The block layer's retry counter
			 *    exhausts and -EIO surfaces to userspace cleanly.
			 *
			 *  - achan->error_cookie clear -- channel is healthy
			 *    but transiently busy (RSLT_VALID race window).
			 *    Leave curr_txd set and rely on the existing
			 *    consumer-side DATATIMEOUT recovery -- mmci's
			 *    own data timer will fire DATATIMEOUT, drive
			 *    mmci_dma_error() -> dmaengine_terminate_all()
			 *    -> graceful FLUSH, and the FLUSH RSLT IRQ runs
			 *    adm_dma_irq()'s normal completion path.
			 *    Watchdog stays disarmed in this branch to avoid
			 *    racing the mmci-side recovery.
			 */
			achan->cmd_ptr_not_rdy_count++;
			if (!achan->cmd_ptr_not_rdy_logged) {
				achan->cmd_ptr_not_rdy_logged = 1;
				dev_warn(adev->dev,
					"ADM ch%d STATUS_SD=0x%08x: CMD_PTR_RDY not set after %d us busy-wait\n",
					achan->id, status, us);
			}

			/*
			 * R63 removed the "permanently dead channel" model:
			 * if a previous RSLT_ERR ran the recovery sequence,
			 * CMD_PTR_RDY should be set by the time we get here.
			 * If it is not, either:
			 *   - The recovery sequence is insufficient on this
			 *     SoC (need to extend [[kit-interrupts-errors]]
			 *     R63), OR
			 *   - The channel is genuinely transiently busy.
			 *
			 * Leave curr_txd set and rely on the watchdog (for
			 * atomic / memory-paced consumers) or the consumer
			 * driver's own timeout (for peripheral-paced RX) to
			 * resolve. Arm the watchdog here for atomic consumers
			 * since we are bailing out before the normal
			 * mod_timer call after the CMD_PTR write.
			 */
			if (async_desc->exec_func)
				mod_timer(&achan->watchdog,
					  jiffies + msecs_to_jiffies(ADM_WATCHDOG_TIMEOUT_MS));

			if (need_submit_lock)
				spin_unlock_irqrestore(&adev->submit_lock[lock_idx],
						       submit_flags);
			return;
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
	{
		u32 cmd_ptr_val = ADM_CPLE_CMD_PTR_LIST |
				  (ALIGN(async_desc->dma_addr, ADM_DESC_ALIGN) >> 3);

		/*
		 * One-shot per (channel,direction) diagnostic: dump the
		 * channel state that bracket the CMD_PTR write so we can
		 * compare against a known-good (webOS) reference when a
		 * boot wedges on the first transfer. Disambiguates "channel
		 * setup wrong" vs "channel started but never completed."
		 */
		if (!achan->first_submit_logged) {
			u32 ctl_ee_dbg = (adev->soc_data &&
					  adev->soc_data->crci_ctl_at_ee0)
					 ? 0 : adev->ee;
			u32 status_pre = readl_relaxed(adev->regs +
				ADM_CH_STATUS_SD(achan->id, adev->ee));
			u32 rslt_conf = readl_relaxed(adev->regs +
				ADM_CH_RSLT_CONF(achan->id, adev->ee));
			u32 crci_ctl = async_desc->crci ?
				readl_relaxed(adev->regs +
					ADM_CRCI_CTL(async_desc->crci, ctl_ee_dbg)) : 0;
			u32 ch_conf_ee0 = readl_relaxed(adev->regs +
				ADM_CH_CONF(achan->id, 0));
			u32 first_word = *(u32 *)async_desc->cpl;

			dev_info(adev->dev,
				 "ADM first submit ch%u: crci=%u cmd_ptr=0x%08x cpl[0]=0x%08x status_sd=0x%x rslt_conf=0x%x crci_ctl[%u]@ee%u=0x%x conf@ee0=0x%x\n",
				 achan->id, async_desc->crci, cmd_ptr_val,
				 first_word, status_pre, rslt_conf,
				 async_desc->crci, ctl_ee_dbg, crci_ctl,
				 ch_conf_ee0);
		}

		writel(cmd_ptr_val,
		       adev->regs + ADM_CH_CMD_PTR(achan->id, adev->ee));

		/*
		 * Post-CMD_PTR snapshot. If the ADM hardware accepted the
		 * command, STATUS_SD's CMD_PTR_RDY bit should go to 0 within
		 * a few cycles (hardware now "owns" CMD_PTR). If the bit
		 * stays at 1 then the hardware silently ignored the write,
		 * which on Tenderloin is the symptom of ADM ch2 (eMMC
		 * sdcc1) wedging — confirmed by webOS-live devmem readback:
		 * ch2 STATUS_SD = CMD_PTR_RDY=1 across mainline boots where
		 * no real DMA happens vs. WORKING webOS where ch2 actively
		 * transfers EXT_CSD and STATUS_SD oscillates.
		 *
		 * Log once per channel + log the CMD_PTR readback so we can
		 * see if the channel's CMD_PTR register actually latched
		 * our value (further proves hardware accepted write).
		 */
		if (!achan->first_submit_logged) {
			u32 status_post = readl_relaxed(adev->regs +
				ADM_CH_STATUS_SD(achan->id, adev->ee));
			u32 cmd_ptr_rb = readl_relaxed(adev->regs +
				ADM_CH_CMD_PTR(achan->id, adev->ee));
			dev_info(adev->dev,
				 "ADM first submit ch%u POST CMD_PTR: status_sd=0x%08x cmd_ptr_rb=0x%08x (wrote 0x%08x; CMD_PTR_RDY %s; exec_func=%s)\n",
				 achan->id, status_post, cmd_ptr_rb, cmd_ptr_val,
				 (status_post & ADM_CH_STATUS_CMD_PTR_RDY) ?
				 "STILL SET (hardware ignored)" :
				 "CLEARED (hardware accepted)",
				 async_desc->exec_func ? "SET (atomic-submit)" :
				 "NULL (inline DATACTRL)");
			achan->first_submit_logged = 1;
		}
	}

	/*
	 * EE=0 mirror write — only meaningful on SoCs whose CRCI_CTL is
	 * live at EE=0 (Tenderloin/APQ8060), AND only needed on the QCE
	 * crypto channels 2-3 of ADM0 where the original investigation
	 * found CMD_PTR at EE=1 alone wasn't sufficient. Running this
	 * mirror write for *every* channel was too broad — on ADM1 the
	 * SDCC channels (ch2=eMMC/CRCI1, ch5=WiFi/CRCI5) get a second
	 * CMD_PTR pushed into a window owned by another security domain
	 * which silently corrupts the channel state, leaving the
	 * descriptor unwalked. Symptom: eMMC card identification never
	 * completes (no mmcblk0, no mmcblk0p12, no touchpad-tokens
	 * provider, all consumer MACs (BT/WiFi/USB) fall back to
	 * random_ether_addr each boot, host-side networking breaks);
	 * ath6kl SDIO BMI firmware download wedges in mmci_request.
	 *
	 * Restrict to ADM0 channels 2 and 3 (the actual QCE channels)
	 * on Tenderloin. Other ADMs / channels keep the single EE=1
	 * CMD_PTR write, matching what legacy webOS msm_dmov did.
	 */
	if (adev->soc_data && adev->soc_data->crci_ctl_at_ee0 &&
	    adev->ee != 0 &&
	    /*
	     * QCE crypto lives on ADM0 channels 2 and 3 specifically.
	     * ADM0's reg base is 0x18320000 on MSM8660/APQ8060; ADM1 at
	     * 0x18420000 hosts SDCC1 eMMC (ch2/CRCI1) and SDCC4 WiFi
	     * (ch5/CRCI5) and those channels MUST NOT see the EE=0
	     * mirror write or their CMD_PTR ends up in another security
	     * domain's window and the channel wedges (eMMC card
	     * identification never completes; ath6kl BMI download hangs).
	     */
	    adev->reg_phys == 0x18320000 &&
	    (achan->id == 2 || achan->id == 3)) {
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
	 * Discriminator: arm only for transfers we know are bounded.
	 *   - exec_func != NULL: mmci atomic-submit WRITEs (eMMC sdcc1
	 *     writes, WiFi sdcc4 writes triggered by atomic-submit gate).
	 *   - CRCI 1 or 5: mmci READs on Tenderloin — eMMC EXT_CSD reads
	 *     and WiFi BMI/SDIO reads. mmci does not set exec_func for
	 *     reads, but the transfers are still memory-paced and should
	 *     finish quickly.
	 * Peripheral-paced RX (UART RX = CRCI 9 BT, CRCI 10 HSUART2/TS,
	 * QCE, NAND) legitimately waits for an external sender with no
	 * meaningful upper bound — armed unconditionally those produce
	 * 510-ms recovery loops that starve the CPU (observed live: ch8
	 * watchdog fired 3x at boot before bailing out, masking what
	 * actually happens on the SDCC channels).
	 *
	 * TODO: replace the hardcoded CRCI list with a per-channel
	 * "watchdog_required" flag in qcom_adm_peripheral_config so
	 * non-Tenderloin SoCs don't depend on this magic.
	 */
	if (async_desc->exec_func ||
	    async_desc->crci == 1 || async_desc->crci == 5) {
		mod_timer(&achan->watchdog,
			  jiffies + msecs_to_jiffies(ADM_WATCHDOG_TIMEOUT_MS));
		/*
		 * One-shot diagnostic: prove mod_timer ran for this channel.
		 * If we never see the matching "watchdog fired" or "watchdog
		 * armed" pair in the log on a wedged boot, mod_timer never
		 * even executed (gate path bug) - this disambiguates.
		 */
		if (!achan->watchdog_armed_logged) {
			achan->watchdog_armed_logged = 1;
			dev_info(adev->dev,
				 "ADM ch%u watchdog ARMED (crci=%u jiffies=%lu)\n",
				 achan->id, async_desc->crci, jiffies);
		}
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
	u32 srcs, i;
	struct adm_async_desc *async_desc;
	bool serviced = false;

	srcs = readl_relaxed(adev->regs +
			ADM_SEC_DOMAIN_IRQ_STATUS(adev->ee));

	if (!srcs)
		return IRQ_NONE;

	dev_dbg(adev->dev, "ADM IRQ: srcs=0x%08x ee=%d\n", srcs, adev->ee);

	while (srcs) {
		struct adm_chan *achan;
		u32 status, result;
		u32 flush_snap[6] = { 0 };

		i = __ffs(srcs);
		srcs &= ~BIT(i);

		/*
		 * Defensive: a channel not in qcom,channels-aarm should
		 * never raise an interrupt at adev->ee, but if it does we
		 * must not touch its registers — that SD belongs to the
		 * modem.
		 */
		if (!test_bit(i, adev->channels_aarm))
			continue;

		achan = &adev->channels[i];

		status = readl_relaxed(adev->regs +
				       ADM_CH_STATUS_SD(i, adev->ee));
		if (!(status & ADM_CH_STATUS_VALID))
			continue;

		result = readl_relaxed(adev->regs +
				       ADM_CH_RSLT(i, adev->ee));
		if (!(result & ADM_CH_RSLT_VALID))
			continue;

		serviced = true;

		/*
		 * Capture the full FLUSH snapshot on FLUSH or ERR. The
		 * snapshot is attached to the descriptor's errdata[] so
		 * consumer callbacks (dma_async_tx_callback_result) can
		 * inspect partial-transfer state. SDCC channels (i==2,5)
		 * also get a one-shot dev_warn for live triage; see
		 * achan->flush_state_dumped for the rate-limit gate.
		 */
		if (result & (ADM_CH_RSLT_FLUSH | ADM_CH_RSLT_ERR)) {
			flush_snap[0] = readl_relaxed(adev->regs +
				ADM_CH_FLUSH_STATE0(i, adev->ee));
			flush_snap[1] = readl_relaxed(adev->regs +
				ADM_CH_FLUSH_STATE1(i, adev->ee));
			flush_snap[2] = readl_relaxed(adev->regs +
				ADM_CH_FLUSH_STATE2(i, adev->ee));
			flush_snap[3] = readl_relaxed(adev->regs +
				ADM_CH_FLUSH_STATE3(i, adev->ee));
			flush_snap[4] = readl_relaxed(adev->regs +
				ADM_CH_FLUSH_STATE4(i, adev->ee));
			flush_snap[5] = readl_relaxed(adev->regs +
				ADM_CH_FLUSH_STATE5(i, adev->ee));

			if ((result & ADM_CH_RSLT_FLUSH) && (i == 2 || i == 5) &&
			    !achan->flush_state_dumped) {
				achan->flush_state_dumped = 1;
				dev_warn(adev->dev,
					"ADM-DIAG ch%u FLUSH result=0x%08x STATE 0..5: %08x %08x %08x %08x %08x %08x (one-shot)\n",
					i, result,
					flush_snap[0], flush_snap[1],
					flush_snap[2], flush_snap[3],
					flush_snap[4], flush_snap[5]);
			}
		}

		if (result & ADM_CH_RSLT_ERR) {
			achan->error = 1;
			dev_err_ratelimited(adev->dev,
				"ADM ch%u RSLT_ERR result=0x%08x flush=%08x %08x %08x %08x %08x %08x\n",
				i, result,
				flush_snap[0], flush_snap[1], flush_snap[2],
				flush_snap[3], flush_snap[4], flush_snap[5]);
		}

		/*
		 * Hardirq context: IRQs already off. spin_lock_irqsave
		 * would be wasted work.
		 */
		spin_lock(&achan->vc.lock);

		timer_delete(&achan->watchdog);

		async_desc = achan->curr_txd;

		/*
		 * One-shot per channel: log the first RSLT_VALID we see so a
		 * boot-time wedge can be classified as "channel never
		 * completed" vs "channel completed, consumer hung after". The
		 * watchdog log + this irq log together bracket every
		 * possible outcome of the first submit.
		 */
		if (!achan->first_irq_logged) {
			achan->first_irq_logged = 1;
			dev_info(adev->dev,
				 "ADM first IRQ ch%u: result=0x%08x flush=%08x %08x %08x %08x %08x %08x async_desc=%s\n",
				 i, result,
				 flush_snap[0], flush_snap[1], flush_snap[2],
				 flush_snap[3], flush_snap[4], flush_snap[5],
				 async_desc ? "live" : "NULL (racer cleared)");
		}

		if (!async_desc) {
			/* Raced with watchdog / terminate_all. Nothing to do. */
			spin_unlock(&achan->vc.lock);
			continue;
		}

		/* Stamp errdata for consumer callback_result. */
		memcpy(async_desc->errdata, flush_snap, sizeof(flush_snap));

		/*
		 * Cyclic period completion: keep the descriptor live and
		 * fire the per-period callback. ADM auto-walks the CPL
		 * chain so we do not re-issue. FLUSH or ERR on a cyclic
		 * descriptor falls through to the normal termination path
		 * below — this is the "stop streaming" event.
		 */
		if (async_desc->cyclic &&
		    !(result & (ADM_CH_RSLT_FLUSH | ADM_CH_RSLT_ERR))) {
			spin_unlock(&achan->vc.lock);
			vchan_cyclic_callback(&async_desc->vd);
			continue;
		}

		{
			bool err = result & ADM_CH_RSLT_ERR;
			/*
			 * Consumers that set exec_func need synchronous
			 * completion: mmci's next-request setup paths
			 * cannot tolerate vchan tasklet deferral. Other
			 * consumers (UART RX, NAND, crypto, dmatest) take
			 * the standard tasklet path.
			 */
			bool atomic_consumer = async_desc->exec_func != NULL;

			achan->curr_txd = NULL;

			if (err) {
				/*
				 * R63: stash the cookie for tx_status,
				 * log the snapshot to the debug ring, and
				 * run the recovery sequence so the channel
				 * is usable by the consumer's next submit.
				 */
				achan->error_cookie = async_desc->vd.tx.cookie;
				adm_log_error(adev, i, async_desc->crci,
					      result, flush_snap);
				adm_recover_channel(achan, async_desc->crci);
			} else {
				/*
				 * Clean completion — clear any stale error
				 * cookie left from a previous-cycle error
				 * so tx_status() stops returning DMA_ERROR
				 * for cookies past the recovery boundary.
				 */
				achan->error_cookie = 0;
			}

			if (atomic_consumer) {
				/*
				 * Inline path (preserves mmci's
				 * synchronous-completion contract).
				 * Pool desc: return to free list inline.
				 * Dynamic desc: defer to tasklet for
				 * dma_free_coherent (cannot run in hardirq).
				 */
				dma_async_tx_callback cb = NULL;
				void *cb_param = NULL;

				dma_cookie_complete(&async_desc->vd.tx);

				if (async_desc->vd.tx.callback) {
					cb = async_desc->vd.tx.callback;
					cb_param = async_desc->vd.tx.callback_param;
				}

				if (async_desc->pool_index >= 0) {
					spin_lock(&adev->pool_lock);
					list_add_tail(&async_desc->pool_node,
						      &adev->desc_free_list);
					spin_unlock(&adev->pool_lock);
				} else {
					async_desc->vd.tx.callback = NULL;
					async_desc->vd.tx.callback_param = NULL;
					list_add_tail(&async_desc->vd.node,
						      &achan->vc.desc_completed);
					tasklet_schedule(&achan->vc.task);
				}

				if (!err)
					adm_start_dma(achan);

				if (cb) {
					spin_unlock(&achan->vc.lock);
					cb(cb_param);
					spin_lock(&achan->vc.lock);
				}
			} else {
				/*
				 * Standard tasklet path. vchan_cookie_complete
				 * advances the cookie, queues the descriptor on
				 * desc_completed and schedules the tasklet,
				 * which fires callback / callback_result (with
				 * errdata for ERR/FLUSH) and runs desc_free.
				 */
				vchan_cookie_complete(&async_desc->vd);

				if (!err)
					adm_start_dma(achan);
			}
		}

		spin_unlock(&achan->vc.lock);
	}

	return serviced ? IRQ_HANDLED : IRQ_NONE;
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

	/*
	 * Surface a recorded RSLT_ERR BEFORE consulting cookie state.
	 * dma_cookie_status() returns DMA_COMPLETE for any cookie
	 * advanced past chan->completed_cookie regardless of how the
	 * descriptor completed, including via the error path --
	 * without this early check a consumer querying from its
	 * completion callback (e.g. mmci_qcom_dma_complete()) cannot
	 * tell ADM-internal errors from clean completions.  cookies
	 * are monotonic per channel so the equality test is
	 * unambiguous; 0 is a safe "no error recorded" sentinel
	 * (dmaengine cookies cycle DMA_MIN_COOKIE..INT_MAX, never 0).
	 */
	if (achan->error_cookie && achan->error_cookie == cookie)
		return DMA_ERROR;

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

/*
 * Worker for the watchdog dump_state callback. Runs in process context
 * (workqueue), so the consumer is free to pm_runtime_get_sync, sleep,
 * acquire its own locks — none of which the watchdog softirq can do.
 */
static void adm_dump_state_work_fn(struct work_struct *work)
{
	struct adm_chan *achan =
		container_of(work, struct adm_chan, dump_state_work);

	if (achan->dump_state)
		achan->dump_state(achan->dump_user);
}

static void adm_channel_init(struct adm_device *adev, struct adm_chan *achan,
			     u32 index)
{
	achan->id = index;
	achan->adev = adev;

	vchan_init(&achan->vc, &adev->common);
	achan->vc.desc_free = adm_dma_free_desc;
	timer_setup(&achan->watchdog, adm_watchdog_timeout, 0);
	INIT_WORK(&achan->dump_state_work, adm_dump_state_work_fn);
}

/**
 * adm_dma_xlate
 * @dma_spec:	pointer to DMA specifier as found in the device tree
 * @ofdma:	pointer to DMA controller data
 *
 * 3-cell DT binding:
 *   cell 0: channel index (0..15)
 *   cell 1: CRCI index (0..15). 0 = no static CRCI (memcpy-only
 *           channels, or NAND / multi-CRCI consumers that override
 *           per-submit via qcom_adm_peripheral_config).
 *   cell 2: CRCI MUX bit (0 or 1). Only meaningful on ADM3 hardware
 *           (MSM8X60); other SoCs ignore the value.
 *
 * The CRCI / MUX pair is folded into achan->crci as the legacy webOS
 * encoded form (mux<<4 | crci) so the descriptor-build path
 * (adm_prep_slave_sg) which already reads bits this way works
 * unchanged.
 *
 * Returns the channel on success, NULL on bad args, or ERR_PTR on a
 * filter mismatch (channel not declared in qcom,channels-aarm).
 */
static struct dma_chan *adm_dma_xlate(struct of_phandle_args *dma_spec,
			       struct of_dma *ofdma)
{
	struct dma_device *dev = ofdma->of_dma_data;
	struct dma_chan *chan, *candidate = NULL;
	struct adm_chan *achan;
	struct adm_device *adev;
	u32 chan_idx, crci_idx, mux;

	if (!dev || dma_spec->args_count != 3) {
		pr_err("qcom_adm: dmas property must have 3 cells (channel crci mux), got %u\n",
		       dma_spec ? dma_spec->args_count : 0);
		return NULL;
	}

	chan_idx = dma_spec->args[0];
	crci_idx = dma_spec->args[1];
	mux      = dma_spec->args[2];

	if (chan_idx >= ADM_MAX_CHANNELS || crci_idx > 15 || mux > 1) {
		pr_err("qcom_adm: bad dmas cells (chan=%u crci=%u mux=%u)\n",
		       chan_idx, crci_idx, mux);
		return NULL;
	}

	/*
	 * Match the hardware channel index (achan->id) NOT chan->chan_id.
	 *
	 * dma_async_device_register() assigns chan_id sequentially based on
	 * registration order, which equals the hardware index only when every
	 * channel is registered. With qcom,channels-aarm filtering out modem /
	 * unused channels the chan_ids compact (Tenderloin ADM1 with
	 * qcom,channels-aarm = <2 5 7> ends up with chan_id 0, 1, 2 for
	 * hardware channels 2, 5, 7). achan->id is stashed at
	 * adm_channel_init() time and is always the hardware index.
	 */
	list_for_each_entry(chan, &dev->channels, device_node)
		if (to_adm_chan(chan)->id == chan_idx) {
			candidate = chan;
			break;
		}

	if (!candidate)
		return NULL;

	achan = to_adm_chan(candidate);
	adev = achan->adev;

	/* Channels not declared in qcom,channels-aarm are not Linux's. */
	if (!test_bit(chan_idx, adev->channels_aarm)) {
		dev_err(adev->dev,
			"ADM xlate: channel %u not in qcom,channels-aarm\n",
			chan_idx);
		return NULL;
	}

	/*
	 * Encode mux into the high nibble so the descriptor-build path
	 * (which masks 0xf for the index and tests BIT(4) for the mux)
	 * works unchanged. Consumer drivers calling dmaengine_slave_config
	 * with qcom_adm_peripheral_config.mux set override this — see
	 * adm_slave_config.
	 */
	achan->crci = (mux ? ADM_CRCI_MUX_SEL : 0) | (crci_idx & 0xf);

	dev_dbg(dev->dev, "ADM xlate: chan=%u crci=%u mux=%u (encoded=0x%x)\n",
		chan_idx, crci_idx, mux, achan->crci);

	return dma_get_slave_channel(candidate);
}

/**
 * adm_prep_dma_memcpy - prepare a TYPE_SINGLE memcpy chain
 *
 * Memcpy descriptors are simple TYPE_SINGLE entries with no CRCI and
 * no flow control. The ADM moves bytes from src to dst at its own
 * pace, decomposed into chunks of ADM_MAX_XFER if the request is
 * longer. The last entry in the chain carries CMD_LC; the CPL list
 * has CMD_PTR_LP on its single entry.
 *
 * Used by the in-tree dmatest selftest and by any consumer that
 * wants a plain memory-to-memory engine.
 */
static struct dma_async_tx_descriptor *
adm_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		    size_t len, unsigned long flags)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct adm_device *adev = achan->adev;
	struct adm_async_desc *async_desc;
	struct adm_desc_hw_single *single;
	void *desc;
	u32 *cple;
	size_t cpl_size, remaining;
	dma_addr_t s = src, d = dst;
	unsigned int n_descs;

	if (!len)
		return NULL;
	if (!IS_ALIGNED(src, ADM_DESC_ALIGN) || !IS_ALIGNED(dst, ADM_DESC_ALIGN))
		return NULL;
	/*
	 * Refuse pathological lengths before they wrap DIV_ROUND_UP's
	 * (len + ADM_MAX_XFER - 1) intermediate. SZ_1G is far above any
	 * realistic memcpy on this hardware and well below the overflow
	 * boundary on every supported arch.
	 */
	if (len > SZ_1G)
		return NULL;

	n_descs = DIV_ROUND_UP(len, ADM_MAX_XFER);
	cpl_size = n_descs * sizeof(*single) + sizeof(*cple) + 2 * ADM_DESC_ALIGN;

	if (cpl_size <= ADM_CPL_BUF_SIZE) {
		async_desc = adm_desc_get(adev);
		if (async_desc)
			async_desc->dma_len = cpl_size;
	} else {
		async_desc = NULL;
	}
	if (!async_desc) {
		async_desc = adm_desc_alloc_fallback(adev, cpl_size);
		if (!async_desc) {
			dev_err(adev->dev, "memcpy: unable to allocate descriptor\n");
			return NULL;
		}
	}

	async_desc->crci = 0;
	async_desc->mux  = 0;
	async_desc->blk_size = 0;
	async_desc->length = len;
	async_desc->dir = DMA_MEM_TO_MEM;
	async_desc->exec_func = NULL;
	async_desc->exec_user = NULL;

	cple = PTR_ALIGN(async_desc->cpl, ADM_DESC_ALIGN);
	desc = PTR_ALIGN(cple + 1, ADM_DESC_ALIGN);
	remaining = len;

	while (remaining) {
		size_t chunk = min_t(size_t, remaining, ADM_MAX_XFER);

		single = desc;
		single->cmd = ADM_CMD_TYPE_SINGLE;
		single->src_addr = s;
		single->dst_addr = d;
		single->len = chunk;

		s += chunk;
		d += chunk;
		remaining -= chunk;
		desc += sizeof(*single);

		if (!remaining)
			single->cmd |= ADM_CMD_LC;
	}

	*cple = ADM_CPLE_LP |
		(((u32)(async_desc->dma_addr + ADM_DESC_ALIGN)) >> 3);

	return vchan_tx_prep(&achan->vc, &async_desc->vd, flags);
}

/**
 * adm_prep_dma_cyclic - prepare a cyclic (period-ring) chain
 *
 * Builds a chain of TYPE_SINGLE descriptors, one per period, each
 * with the slave-side CRCI from the channel's slave_config. The
 * descriptor list loops: the CPL has CMD_PTR_LP on its last entry
 * but the last descriptor's cmd does NOT carry CMD_LC, so the ADM
 * re-walks the CPL after the last period completes. Per-period
 * notification is delivered via vchan_cyclic_callback() from the IRQ
 * handler when RSLT_VALID fires partway through the chain (the ADM
 * posts RSLT after each CMD_LC; for cyclic we set CMD_LC on every
 * period descriptor so the driver gets one callback per period).
 *
 * Useful for streaming audio / TSIF consumers.
 */
static struct dma_async_tx_descriptor *
adm_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		    size_t period_len, enum dma_transfer_direction direction,
		    unsigned long flags)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct adm_device *adev = achan->adev;
	struct adm_async_desc *async_desc;
	struct adm_desc_hw_single *single;
	void *desc;
	u32 *cple;
	size_t cpl_size;
	unsigned int n_periods, i;
	u32 crci_cmd;
	dma_addr_t slave_addr;
	/*
	 * Snapshot of achan->crci taken under achan->vc.lock at entry to
	 * close the adm_slave_config() race; see the same pattern in
	 * adm_prep_slave_sg() for rationale. Encoded form (mux<<4 | crci).
	 */
	u32 chan_crci_enc;
	unsigned long lock_flags;

	if (!buf_len || !period_len || buf_len % period_len)
		return NULL;
	if (!is_slave_direction(direction))
		return NULL;
	if (!IS_ALIGNED(buf_addr, ADM_DESC_ALIGN) ||
	    !IS_ALIGNED(period_len, ADM_DESC_ALIGN))
		return NULL;
	/* Same sanity cap as adm_prep_dma_memcpy — see comment there. */
	if (buf_len > SZ_1G)
		return NULL;

	spin_lock_irqsave(&achan->vc.lock, lock_flags);
	chan_crci_enc = achan->crci;
	spin_unlock_irqrestore(&achan->vc.lock, lock_flags);

	n_periods = buf_len / period_len;
	cpl_size = n_periods * sizeof(*single) + sizeof(*cple) + 2 * ADM_DESC_ALIGN;

	if (cpl_size <= ADM_CPL_BUF_SIZE) {
		async_desc = adm_desc_get(adev);
		if (async_desc)
			async_desc->dma_len = cpl_size;
	} else {
		async_desc = NULL;
	}
	if (!async_desc) {
		async_desc = adm_desc_alloc_fallback(adev, cpl_size);
		if (!async_desc) {
			dev_err(adev->dev, "cyclic: unable to allocate descriptor\n");
			return NULL;
		}
	}

	async_desc->crci = chan_crci_enc & 0xf;
	async_desc->mux = (chan_crci_enc & ADM_CRCI_MUX_SEL) ?
			  ADM_CRCI_CTL_MUX_SEL : 0;
	async_desc->blk_size = 0;	/* per-CRCI default applies */
	async_desc->length = buf_len;
	async_desc->dir = direction;
	async_desc->exec_func = NULL;
	async_desc->exec_user = NULL;
	async_desc->cyclic = true;

	if (direction == DMA_DEV_TO_MEM) {
		crci_cmd = ADM_CMD_SRC_CRCI(async_desc->crci);
		slave_addr = achan->slave.src_addr;
	} else {
		crci_cmd = ADM_CMD_DST_CRCI(async_desc->crci);
		slave_addr = achan->slave.dst_addr;
	}

	cple = PTR_ALIGN(async_desc->cpl, ADM_DESC_ALIGN);
	desc = PTR_ALIGN(cple + 1, ADM_DESC_ALIGN);

	for (i = 0; i < n_periods; i++) {
		single = desc;
		single->cmd = ADM_CMD_TYPE_SINGLE | crci_cmd | ADM_CMD_LC;
		single->len = period_len;
		if (direction == DMA_DEV_TO_MEM) {
			single->src_addr = slave_addr;
			single->dst_addr = buf_addr + i * period_len;
		} else {
			single->src_addr = buf_addr + i * period_len;
			single->dst_addr = slave_addr;
		}
		desc += sizeof(*single);
	}

	/*
	 * Single CPL entry pointing at the first descriptor. With
	 * CMD_PTR_LP set the ADM does NOT auto-restart at the head; we
	 * mark async_desc->dir to enable the cyclic callback path in
	 * the IRQ handler, which re-issues the descriptor when the
	 * final period in the buffer completes.
	 */
	*cple = ADM_CPLE_LP |
		(((u32)(async_desc->dma_addr + ADM_DESC_ALIGN)) >> 3);

	return vchan_tx_prep(&achan->vc, &async_desc->vd, flags);
}

/*
 * device_pause: graceful flush, current burst completes. Channel
 * stays in FLUSHING until the IRQ handler observes RSLT_FLUSH; at
 * that point the consumer's callback gets a partial-transfer
 * notification and the channel is back to IDLE.
 */
static int adm_device_pause(struct dma_chan *chan)
{
	struct adm_chan *achan = to_adm_chan(chan);
	struct adm_device *adev = achan->adev;
	unsigned long flags;

	spin_lock_irqsave(&achan->vc.lock, flags);
	if (achan->curr_txd) {
		writel_relaxed(ADM_CH_FLUSH_GRACEFUL,
			       adev->regs +
			       ADM_CH_FLUSH_STATE0(achan->id, adev->ee));
	}
	spin_unlock_irqrestore(&achan->vc.lock, flags);
	return 0;
}

/*
 * device_resume: re-issue any pending descriptor from the queue.
 * The pause-state descriptor was completed via the FLUSH IRQ path
 * with a partial-transfer record; consumers either treat it as done
 * or queue a fresh descriptor for the rest of the work.
 */
static int adm_device_resume(struct dma_chan *chan)
{
	struct adm_chan *achan = to_adm_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&achan->vc.lock, flags);
	if (vchan_issue_pending(&achan->vc) && !achan->curr_txd)
		adm_start_dma(achan);
	spin_unlock_irqrestore(&achan->vc.lock, flags);
	return 0;
}

/*
 * device_synchronize: wait (sleeping) for any in-flight FLUSH /
 * callback to land. Pairs with terminate_async.
 */
static void adm_device_synchronize(struct dma_chan *chan)
{
	struct adm_chan *achan = to_adm_chan(chan);

	vchan_synchronize(&achan->vc);
	synchronize_irq(achan->adev->irq);
}

/*
 * Runtime PM callbacks. Clocks are reference-counted by usage_count;
 * the per-submit pm_runtime_resume_and_get() / per-completion
 * pm_runtime_put_autosuspend() pair lets autosuspend drop clocks
 * after ADM_RUNTIME_AUTOSUSPEND_MS of idle.
 */
#define ADM_RUNTIME_AUTOSUSPEND_MS 100

static int __maybe_unused adm_runtime_suspend(struct device *dev)
{
	struct adm_device *adev = dev_get_drvdata(dev);

	disable_irq(adev->irq);
	if (adev->iface_clk)
		clk_disable_unprepare(adev->iface_clk);
	clk_disable_unprepare(adev->core_clk);
	return 0;
}

static int __maybe_unused adm_runtime_resume(struct device *dev)
{
	struct adm_device *adev = dev_get_drvdata(dev);
	int ret;
	unsigned int ch;

	ret = clk_prepare_enable(adev->core_clk);
	if (ret) {
		dev_err(dev, "runtime resume: core clock enable failed: %d\n", ret);
		return ret;
	}
	if (adev->iface_clk) {
		ret = clk_prepare_enable(adev->iface_clk);
		if (ret) {
			dev_err(dev, "runtime resume: iface clock enable failed: %d\n", ret);
			clk_disable_unprepare(adev->core_clk);
			return ret;
		}
	}

	/* Re-arm IRQ_EN + FLUSH_EN per channel after power collapse. */
	for_each_set_bit(ch, adev->channels_aarm, ADM_MAX_CHANNELS)
		writel(ADM_CH_RSLT_CONF_IRQ_EN | ADM_CH_RSLT_CONF_FLUSH_EN,
		       adev->regs + ADM_CH_RSLT_CONF(ch, adev->ee));

	enable_irq(adev->irq);
	return 0;
}

static int __maybe_unused adm_runtime_idle(struct device *dev)
{
	/* Defer to autosuspend timer; idle just acks. */
	return 0;
}

static const struct dev_pm_ops adm_pm_ops = {
	SET_RUNTIME_PM_OPS(adm_runtime_suspend, adm_runtime_resume,
			   adm_runtime_idle)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static int adm_dma_probe(struct platform_device *pdev)
{
	struct adm_device *adev;
	int ret;
	u32 i, nr_aarm;
	u32 ch_list[ADM_MAX_CHANNELS];

	adev = devm_kzalloc(&pdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->dev = &pdev->dev;
	adev->soc_data = of_device_get_match_data(&pdev->dev);
	if (!adev->soc_data) {
		dev_err(adev->dev, "no driver_data on of_device_id match\n");
		return -EINVAL;
	}
	dev_info(adev->dev, "ADM %s\n", adev->soc_data->name);

	/* Per-CRCI submit locks. See submit_lock[] declaration comment. */
	for (i = 0; i < ARRAY_SIZE(adev->submit_lock); i++)
		spin_lock_init(&adev->submit_lock[i]);
	spin_lock_init(&adev->err_log_lock);

	{
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return -EINVAL;
		adev->reg_phys = res->start;
	}

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

	adev->owns_master_sd = of_property_read_bool(pdev->dev.of_node,
						     "qcom,owns-master-sd");

	/*
	 * qcom,channels-aarm filter. Absent property = "all 16" for
	 * back-compat with older DTSes that didn't declare the list.
	 */
	nr_aarm = of_property_count_u32_elems(pdev->dev.of_node,
					      "qcom,channels-aarm");
	if (nr_aarm > 0 && nr_aarm <= ADM_MAX_CHANNELS) {
		ret = of_property_read_u32_array(pdev->dev.of_node,
						 "qcom,channels-aarm",
						 ch_list, nr_aarm);
		if (ret) {
			dev_err(adev->dev, "qcom,channels-aarm read failed: %d\n",
				ret);
			return ret;
		}
		for (i = 0; i < nr_aarm; i++) {
			if (ch_list[i] >= ADM_MAX_CHANNELS) {
				dev_err(adev->dev,
					"qcom,channels-aarm entry %u out of range\n",
					ch_list[i]);
				return -EINVAL;
			}
			set_bit(ch_list[i], adev->channels_aarm);
		}
	} else {
		bitmap_fill(adev->channels_aarm, ADM_MAX_CHANNELS);
	}
	dev_info(adev->dev, "AARM-owned channels mask = 0x%lx\n",
		 *adev->channels_aarm);

	adev->core_clk = devm_clk_get(adev->dev, "core");
	if (IS_ERR(adev->core_clk))
		return PTR_ERR(adev->core_clk);

	/*
	 * iface is optional — not every SoC has a separate AHB clock.
	 * devm_clk_get_optional() returns NULL when absent.
	 */
	adev->iface_clk = devm_clk_get_optional(adev->dev, "iface");
	if (IS_ERR(adev->iface_clk))
		return PTR_ERR(adev->iface_clk);

	adev->clk_reset = devm_reset_control_get_exclusive(&pdev->dev, "clk");
	if (IS_ERR(adev->clk_reset)) {
		dev_err(adev->dev, "failed to get clk reset\n");
		return PTR_ERR(adev->clk_reset);
	}

	adev->c0_reset = devm_reset_control_get_exclusive(&pdev->dev, "c0");
	if (IS_ERR(adev->c0_reset)) {
		dev_err(adev->dev, "failed to get C0 reset\n");
		return PTR_ERR(adev->c0_reset);
	}

	adev->c1_reset = devm_reset_control_get_exclusive(&pdev->dev, "c1");
	if (IS_ERR(adev->c1_reset)) {
		dev_err(adev->dev, "failed to get C1 reset\n");
		return PTR_ERR(adev->c1_reset);
	}

	adev->c2_reset = devm_reset_control_get_exclusive(&pdev->dev, "c2");
	if (IS_ERR(adev->c2_reset)) {
		dev_err(adev->dev, "failed to get C2 reset\n");
		return PTR_ERR(adev->c2_reset);
	}

	ret = dma_set_mask_and_coherent(adev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(adev->dev, "failed to set 32-bit DMA mask: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(adev->core_clk);
	if (ret) {
		dev_err(adev->dev, "failed to prepare/enable core clock\n");
		return ret;
	}

	if (adev->iface_clk) {
		ret = clk_prepare_enable(adev->iface_clk);
		if (ret) {
			dev_err(adev->dev,
				"failed to prepare/enable iface clock\n");
			goto err_disable_core_clk;
		}
	}

	/*
	 * Interconnect: replaces the legacy ebi1_adm_clk voter. Vote at
	 * the floor declared by the per-SoC driver_data; 0/0 = no vote.
	 * Both "memory" and "memory-p1" are optional and absence is fine.
	 */
	adev->icc_path = devm_of_icc_get(adev->dev, "memory");
	if (IS_ERR(adev->icc_path)) {
		ret = PTR_ERR(adev->icc_path);
		if (ret != -ENODATA && ret != -ENOENT) {
			dev_err(adev->dev,
				"failed to get interconnect path: %d\n", ret);
			goto err_disable_clks;
		}
		adev->icc_path = NULL;
	}
	if (adev->icc_path && adev->soc_data->icc_bw_kbps_peak) {
		ret = icc_set_bw(adev->icc_path,
				 adev->soc_data->icc_bw_kbps_avg,
				 adev->soc_data->icc_bw_kbps_peak);
		if (ret) {
			dev_err(adev->dev,
				"failed to set interconnect bw: %d\n", ret);
			goto err_disable_clks;
		}
	}

	adev->icc_path_p1 = devm_of_icc_get(adev->dev, "memory-p1");
	if (IS_ERR(adev->icc_path_p1)) {
		ret = PTR_ERR(adev->icc_path_p1);
		if (ret != -ENODATA && ret != -ENOENT) {
			dev_err(adev->dev,
				"failed to get PORT1 interconnect path: %d\n",
				ret);
			goto err_disable_clks;
		}
		adev->icc_path_p1 = NULL;
	}
	if (adev->icc_path_p1 && adev->soc_data->icc_bw_kbps_peak) {
		ret = icc_set_bw(adev->icc_path_p1,
				 adev->soc_data->icc_bw_kbps_avg,
				 adev->soc_data->icc_bw_kbps_peak);
		if (ret) {
			dev_err(adev->dev,
				"failed to set PORT1 interconnect bw: %d\n",
				ret);
			goto err_disable_clks;
		}
	}

	/*
	 * Probe-time reset is per-SoC: needed on IPQ8064 to bring the
	 * controller into a known state; forbidden on MSM8660/APQ8060
	 * because the bootloader has already programmed CRCI_CTL at the
	 * EE=0 aperture for the peripherals it enabled and a reset would
	 * wipe those entries.
	 */
	if (adev->soc_data->needs_probe_reset) {
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

	INIT_LIST_HEAD(&adev->common.channels);

	/*
	 * Channel filter: only register channels declared in
	 * qcom,channels-aarm. Channels owned by the modem (10-14 on
	 * MSM8660 ADM0) or unused must not be exposed to dmaengine.
	 */
	for (i = 0; i < ADM_MAX_CHANNELS; i++) {
		if (test_bit(i, adev->channels_aarm))
			adm_channel_init(adev, &adev->channels[i], i);
		else
			adev->channels[i].id = i;	/* parked, unused */
	}

	ret = adm_desc_pool_init(adev);
	if (ret) {
		dev_err(adev->dev, "failed to initialize descriptor pool\n");
		goto err_disable_clks;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       adm_desc_pool_destroy_action, adev);
	if (ret) {
		dev_err(adev->dev,
			"failed to register pool-destroy devm action: %d\n",
			ret);
		goto err_disable_clks;
	}

	/*
	 * Reset all CRCIs once at probe so a previously-armed CRCI from
	 * a kexec/warm boot doesn't deliver stale data on first submit.
	 */
	for (i = 0; i < 16; i++)
		writel(ADM_CRCI_CTL_RST, adev->regs +
			ADM_CRCI_CTL(i, adev->ee));

	/*
	 * Per-SoC CRCI defaults. Seed CRCI_CTL with the documented value
	 * so the cache-skip path in adm_start_dma doesn't pick up the
	 * post-RST 0 as authoritative. Applied to the live EE aperture
	 * (EE=0 on Tenderloin via crci_ctl_at_ee0, adev->ee elsewhere).
	 */
	if (adev->soc_data->crci_defaults) {
		u32 ctl_ee = adev->soc_data->crci_ctl_at_ee0 ? 0 : adev->ee;
		int j;

		for (j = 0; j < adev->soc_data->nr_crci_defaults; j++) {
			const struct adm_crci_default *def =
				&adev->soc_data->crci_defaults[j];
			u32 val = def->blk_size |
				  (def->mux ? ADM_CRCI_CTL_MUX_SEL : 0);

			writel(val,
			       adev->regs + ADM_CRCI_CTL(def->crci, ctl_ee));
			adev->crci_ctl_cache[def->crci] = val;
			adev->crci_ctl_cache_valid |= BIT(def->crci);
		}
	}

	/*
	 * Per-channel state: RSLT_CONF gets IRQ + FLUSH enable, and CH_CONF
	 * gets SHADOW_EN — both at adev->ee (the AARM/master window).
	 *
	 * CH_CONF + SHADOW_EN is the piece this driver was missing and the
	 * root cause of the SDCC↔ADM CRCI handshake stall (FLUSH_STATE5=3
	 * "drain stage 3"): the ADM walked the descriptor and waited for a
	 * CRCI assertion that never connected, because the channel's
	 * config-bank shadow (CRCI_CTL/CONF live at EE=0 on this SoC) was
	 * never linked to the per-channel command bank (CMD_PTR/RSLT/STATUS
	 * live at EE=1). SHADOW_EN in the EE=1 (master) CONF is what arms
	 * that link.
	 *
	 * Legacy webOS arch/arm/mach-msm/dma.c config_datamover() does
	 * exactly this for every AARM channel:
	 *
	 *   conf = readl(DMOV_CONF(i));            // master window (SD=1)
	 *   conf &= ~DMOV_CONF_SD(7);
	 *   conf |= DMOV_CONF_SD(chan_conf[i].sd); // sd = 1 (AARM)
	 *   writel(conf | DMOV_CONF_SHADOW_EN, DMOV_CONF(i));
	 *
	 * An earlier iteration of this driver concluded EE=1 CH_CONF writes
	 * were "silently dropped" because the register reads back 0 — but
	 * SHADOW_EN is a write-effect bit: readback 0 does NOT mean the
	 * write had no effect. The effective per-channel config (priority,
	 * IRQ_EN, FORCE_RSLT, MPU_DISABLE) stays in the EE=0 shadow that the
	 * bootloader programmed (0x8d5 for ch2); SHADOW_EN connects it. We
	 * only set SD + SHADOW_EN here and leave the EE=0 shadow untouched,
	 * matching legacy.
	 */
	for_each_set_bit(i, adev->channels_aarm, ADM_MAX_CHANNELS) {
		writel(ADM_CH_RSLT_CONF_IRQ_EN | ADM_CH_RSLT_CONF_FLUSH_EN,
		       adev->regs + ADM_CH_RSLT_CONF(i, adev->ee));

		if (adev->soc_data && adev->soc_data->crci_ctl_at_ee0) {
			u32 conf = readl(adev->regs +
					 ADM_CH_CONF(i, adev->ee));

			conf &= ~ADM_CH_CONF_SEC_DOMAIN(7);
			conf |= ADM_CH_CONF_SEC_DOMAIN(adev->ee);
			conf |= ADM_CH_CONF_SHADOW_EN;
			writel(conf, adev->regs + ADM_CH_CONF(i, adev->ee));
		}

		adev->channels[i].initialized = 1;
	}

	/* Drain any RSLT residue left by the bootloader / warm reset. */
	adm_drain_rslt(adev);

	ret = devm_request_irq(adev->dev, adev->irq, adm_dma_irq,
			       IRQF_NOBALANCING, "adm_dma", adev);
	if (ret)
		goto err_disable_clks;

	/*
	 * Pin the ADM completion IRQ to CPU1 when available. Tenderloin
	 * shares adm_dma1 between sdcc1 (eMMC DMA) and sdcc4 (WiFi PIO);
	 * letting the ADM hardirq stay on CPU0 alongside sdcc4 caused
	 * DPSM FIFO underflows. Cheap on other SoCs.
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

	/* Capabilities */
	dma_cap_zero(adev->common.cap_mask);
	dma_cap_set(DMA_SLAVE,   adev->common.cap_mask);
	dma_cap_set(DMA_PRIVATE, adev->common.cap_mask);
	dma_cap_set(DMA_MEMCPY,  adev->common.cap_mask);
	dma_cap_set(DMA_CYCLIC,  adev->common.cap_mask);

	adev->common.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
				  BIT(DMA_MEM_TO_MEM);
	adev->common.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	adev->common.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
				       BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				       BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	adev->common.dst_addr_widths = adev->common.src_addr_widths;
	adev->common.copy_align = ADM_DESC_ALIGN;

	adev->common.device_free_chan_resources = adm_free_chan;
	adev->common.device_prep_slave_sg       = adm_prep_slave_sg;
	adev->common.device_prep_dma_memcpy     = adm_prep_dma_memcpy;
	adev->common.device_prep_dma_cyclic     = adm_prep_dma_cyclic;
	adev->common.device_issue_pending       = adm_issue_pending;
	adev->common.device_tx_status           = adm_tx_status;
	adev->common.device_terminate_all       = adm_terminate_all;
	adev->common.device_synchronize         = adm_device_synchronize;
	adev->common.device_pause               = adm_device_pause;
	adev->common.device_resume              = adm_device_resume;
	adev->common.device_config              = adm_slave_config;

	ret = dma_async_device_register(&adev->common);
	if (ret) {
		dev_err(adev->dev, "failed to register dma async device\n");
		goto err_disable_clks;
	}

	ret = of_dma_controller_register(pdev->dev.of_node, adm_dma_xlate,
					 &adev->common);
	if (ret)
		goto err_unregister_dma;

	/*
	 * Runtime PM intentionally NOT enabled here.
	 *
	 * The original autosuspend (100 ms) + adm_runtime_suspend
	 * disable_irq() pair would race against any consumer that
	 * does not call pm_runtime_get_sync() before submitting --
	 * which, today, is every consumer (mmci-pl18x, ath6kl_sdio,
	 * qcrypto, msm_serial BT). 100 ms after probe the autosuspend
	 * timer would fire, masking the ADM GIC IRQ (GICD_ISENABLER6
	 * bit 7 / 11 dropped to 0) AND gating the channel clocks. Any
	 * later submit would queue the descriptor and wait forever for
	 * an IRQ that is masked at the distributor and a channel clock
	 * that is off. Confirmed live by reading /proc/interrupts
	 * (count=0 on both adm_dma IRQs) + GICD_ISENABLER6 via devmem
	 * on a wedged boot.
	 *
	 * Until all dmaengine consumers learn to pm_runtime_get_sync()
	 * the channel before submit, keep the ADM controller always-
	 * active: clocks stay on (bootloader left them on; the probe
	 * clk_prepare_enable bumped the usage count to 1 and we never
	 * drop it) and the GIC IRQ stays unmasked.
	 *
	 * TODO: re-introduce runtime PM once mmci's qcom variant grows
	 * a per-submit pm_runtime_get_sync hook on the rx/tx ADM
	 * channels; same for the other ADM consumers.
	 */

	adm_debugfs_init(adev);

	return 0;

err_unregister_dma:
	dma_async_device_unregister(&adev->common);
err_disable_clks:
	if (adev->iface_clk)
		clk_disable_unprepare(adev->iface_clk);
err_disable_core_clk:
	clk_disable_unprepare(adev->core_clk);

	return ret;
}

static void adm_dma_remove(struct platform_device *pdev)
{
	struct adm_device *adev = platform_get_drvdata(pdev);
	unsigned int i;

	adm_debugfs_remove(adev);

	pm_runtime_disable(adev->dev);
	pm_runtime_dont_use_autosuspend(adev->dev);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&adev->common);

	for_each_set_bit(i, adev->channels_aarm, ADM_MAX_CHANNELS) {
		writel(0, adev->regs + ADM_CH_RSLT_CONF(i, adev->ee));
		tasklet_kill(&adev->channels[i].vc.task);
		adm_terminate_all(&adev->channels[i].vc.chan);
	}

	devm_free_irq(adev->dev, adev->irq, adev);

	/* Descriptor pool is freed via the devm action registered at probe. */

	if (adev->iface_clk)
		clk_disable_unprepare(adev->iface_clk);
	clk_disable_unprepare(adev->core_clk);
}

static const struct of_device_id adm_of_match[] = {
	{ .compatible = "qcom,adm",         .data = &adm_generic_data,   },
	{ .compatible = "qcom,adm-apq8060", .data = &adm_msm8660_data,   },
	{ .compatible = "qcom,adm-msm8660", .data = &adm_msm8660_data,   },
	{ .compatible = "qcom,adm-ipq8064", .data = &adm_ipq8064_data,   },
	/*
	 * Stub entries for known SoCs that are not yet differentiated
	 * from the generic profile. Adding a real adm_soc_data when a
	 * board's quirks are characterised should be a same-file change.
	 */
	{ .compatible = "qcom,adm-mpq8064", .data = &adm_generic_data,   },
	{ .compatible = "qcom,adm-msm7x30", .data = &adm_generic_data,   },
	{ .compatible = "qcom,adm-msm8960", .data = &adm_generic_data,   },
	{ .compatible = "qcom,adm-msm8974", .data = &adm_generic_data,   },
	{}
};
MODULE_DEVICE_TABLE(of, adm_of_match);

static struct platform_driver adm_dma_driver = {
	.probe = adm_dma_probe,
	.remove = adm_dma_remove,
	.driver = {
		.name = "adm-dma-engine",
		.of_match_table = adm_of_match,
		.pm = &adm_pm_ops,
	},
};

module_platform_driver(adm_dma_driver);

MODULE_AUTHOR("Andy Gross <agross@codeaurora.org>");
MODULE_DESCRIPTION("QCOM ADM DMA engine driver");
MODULE_LICENSE("GPL v2");
