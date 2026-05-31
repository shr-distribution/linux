// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSM8x60 family (MSM8260/MSM8660/APQ8060) MPM (MSM Power Manager) wakeup interrupt controller
 *
 * The MPM is an always-on hardware block that keeps a small set of wake
 * sources alive while the application processor is powered down for
 * cpuidle Power Collapse or suspend-to-RAM. On MSM8x60 the
 * vMPM (virtual MPM) registers live INSIDE the RPM's 4 KB control block
 * at:
 *
 *   request (control) regs: RPM_BASE + 0x9d8  (ENABLE, DETECT_CTL,
 *                                              POLARITY, CLEAR)
 *   status (pending) regs:  RPM_BASE + 0xdf8  (== 0x9d8 + 0x420)
 *
 * The mainline qcom-mpm driver (drivers/irqchip/irq-qcom-mpm.c) is
 * fundamentally incompatible with this layout:
 *   - it assumes a dedicated MPM SRAM region separate from RPM;
 *   - it assumes a mailbox controller (IPCC) for wake notification;
 *   - it uses IRQCHIP_DECLARE which runs before platform devices exist,
 *     so of_find_device_by_node() returns NULL and the init silently
 *     hangs.
 *
 * This driver replicates the 2.6.35-palm `arch/arm/mach-msm/mpm.c`
 * mechanism as a regular platform driver: probes after platform
 * infrastructure is ready, ioremaps the vMPM sub-region of the RPM
 * control block (the qcom,rpm driver maps the surrounding area for
 * its own use; the two mappings overlap and neither claims exclusive
 * ownership), and uses the qcom-apcs-ipc mailbox for wake notification
 * (writing to GCC + 0x008 bit 1).
 *
 * Register access is done with readl_relaxed/writel_relaxed rather than
 * via the RPM syscon regmap. The IRQ core invokes our mask/unmask/
 * set_type/set_wake callbacks with the irq_desc's raw_spinlock_t held,
 * and syscon regmaps use a sleepable spinlock_t which on PREEMPT_RT
 * would deadlock under that raw lock. Direct MMIO is also what every
 * other SoC irqchip (qcom-pdc, gic-v3, ...) does.
 *
 * Two consumer interfaces:
 *
 *   1. Hierarchical irqdomain: for MPM pins that map to GIC SPIs (USB,
 *      HDMI, ...). Consumers wire their interrupts through this
 *      controller via interrupts-extended and the kernel manages
 *      enable / mask / set_type / set_wake via the IRQ subsystem.
 *
 *   2. Raw-pin API: for MPM pins that do NOT correspond to a GIC IRQ
 *      (SDC3_DAT1=21, SDC3_DAT3=22, SDC4_DAT1=23, SDC4_DAT3=24).
 *      These are physical wake-signal lines monitored by MPM
 *      directly. Consumers (mmci for SDC4 wake) call
 *      msm8660_mpm_set_pin_wake() etc. The consumer API establishes
 *      a device_link from consumer to producer so the MPM device
 *      cannot disappear while a consumer holds a handle.
 *
 * Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
 * Copyright (c) 2010-2012, The Linux Foundation (legacy mpm.c reference)
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <soc/qcom/msm8660-mpm.h>

/*
 * vMPM register offsets (relative to the start of the ioremap'd window
 * = RPM base + 0x9d8). Each register is two 32-bit slots because MPM
 * exposes 64 wake pins.
 */
#define MSM8660_MPM_REG_ENABLE		0x00
#define MSM8660_MPM_REG_DETECT_CTL	0x08
#define MSM8660_MPM_REG_POLARITY	0x10
#define MSM8660_MPM_REG_CLEAR		0x18

/* Status registers at +0x420 from vMPM base (== RPM + 0xdf8). */
#define MSM8660_MPM_STATUS_OFFSET	0x420

#define MSM8660_MPM_PIN_COUNT		64
#define MSM8660_MPM_REG_WIDTH		2

struct msm8660_mpm_pin {
	int pin;
	int hwirq;
};

struct msm8660_mpm {
	struct device *dev;
	void __iomem *base;
	struct irq_domain *domain;
	struct msm8660_mpm_pin *pin_map;
	unsigned int pin_map_count;
	u32 *raw_pins;
	unsigned int raw_pin_count;
	int parent_irq;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;

	/*
	 * The IRQ subsystem calls our mask/unmask/set_type/set_wake under
	 * the irq_desc raw_spinlock_t. The mailbox core's chan->lock is a
	 * spinlock_t which becomes an rt_mutex on PREEMPT_RT, so doorbells
	 * cannot be issued from those callbacks. Implement the standard
	 * irq_bus_lock pattern: callbacks only stage cached register state
	 * under the sleepable bus_lock mutex; the actual MMIO writes and
	 * the RPM doorbell are flushed in irq_bus_sync_unlock, which runs
	 * in process context (sleepable) after the IRQ core has released
	 * irq_desc->lock.
	 *
	 * The same mutex serializes the raw-pin C API (msm8660_mpm_enable_pin
	 * / set_pin_wake / set_pin_type) so concurrent consumers cannot lose
	 * each other's RMW updates on the shared bank registers.
	 */
	struct mutex bus_lock;

	/* Per-bank cached state (last value committed to hardware) */
	u32 enable_cached[MSM8660_MPM_REG_WIDTH];
	u32 detect_cached[MSM8660_MPM_REG_WIDTH];
	u32 polarity_cached[MSM8660_MPM_REG_WIDTH];

	/* Per-bank pending state (staged by callbacks, written in sync_unlock) */
	u32 enable[MSM8660_MPM_REG_WIDTH];
	u32 detect[MSM8660_MPM_REG_WIDTH];
	u32 polarity[MSM8660_MPM_REG_WIDTH];
};

/*
 * Singleton - there is only one MPM instance per SoC. msm8660_mpm_get()
 * returns this. Updates are serialised through the binding lifecycle so
 * a plain pointer is sufficient.
 */
static struct msm8660_mpm *msm8660_mpm_global;

static u32 msm8660_mpm_read(struct msm8660_mpm *mpm, unsigned int reg)
{
	return readl_relaxed(mpm->base + reg);
}

static void msm8660_mpm_write(struct msm8660_mpm *mpm, unsigned int reg,
			      u32 val)
{
	writel_relaxed(val, mpm->base + reg);
}

/*
 * Doorbell the RPM after touching the vMPM request registers. Without
 * this the RPM keeps using its last cached copy of the enable/detect/
 * polarity state and our configuration changes have no effect.
 *
 * Called from raw_spinlock_t-held contexts (irq_chip mask/unmask/
 * set_type/set_wake), so the mailbox driver must accept that. The
 * qcom-apcs-ipc mailbox just does a writel into the IPC trigger
 * register; it is safe under a raw lock.
 */
static void msm8660_mpm_doorbell(struct msm8660_mpm *mpm)
{
	int ret;

	if (!mpm->mbox_chan)
		return;

	ret = mbox_send_message(mpm->mbox_chan, NULL);
	if (ret < 0)
		dev_warn_ratelimited(mpm->dev,
				     "RPM doorbell failed: %d\n", ret);
}

/*
 * Resolve an MPM pin number to its parent GIC SPI for the irqdomain
 * hierarchy. Pins listed in qcom,mpm-pin-map have a corresponding GIC
 * SPI; pins listed only in qcom,mpm-raw-pins (or pins that are not
 * declared at all) return -ENOENT and cannot be allocated through the
 * irqdomain path.
 */
static int msm8660_mpm_pin_to_gic_spi(struct msm8660_mpm *mpm, unsigned int pin)
{
	unsigned int i;

	for (i = 0; i < mpm->pin_map_count; i++) {
		if (mpm->pin_map[i].pin == pin)
			return mpm->pin_map[i].hwirq;
	}
	return -ENOENT;
}

/*
 * Return true if @pin is declared as a raw wake source in
 * qcom,mpm-raw-pins. The raw-pin C API rejects requests for pins
 * that are not in the board's allow-list, so a consumer cannot
 * silently enable a wake source the integrator has not opted in to.
 */
static bool msm8660_mpm_raw_pin_allowed(struct msm8660_mpm *mpm,
					unsigned int pin)
{
	unsigned int i;

	for (i = 0; i < mpm->raw_pin_count; i++) {
		if (mpm->raw_pins[i] == pin)
			return true;
	}
	return false;
}

/*
 * IPC handler: MPM fires this IRQ when one or more enabled wake pins
 * have pending activity. Read pending status, CLEAR the pending bits
 * BEFORE dispatching the per-pin handlers so a fresh edge that arrives
 * during dispatch cannot be wiped out by a later CLEAR write, then
 * replay each pending pin through the irqdomain.
 *
 * After all CLEAR writes have been issued we MUST:
 *   1. Flush them out of the CPU write buffer by doing a read-back
 *      from the same MMIO window. Without this the relaxed CLEAR
 *      can reorder past the parent GIC EOI and the level-triggered
 *      IPC line will re-assert immediately, producing an interrupt
 *      storm.
 *   2. Doorbell the RPM so it re-reads the request banks and stops
 *      asserting the IPC line. The RPM caches the vMPM request
 *      copies and will not notice the CLEAR until it is poked.
 */
static irqreturn_t msm8660_mpm_irq(int irq, void *data)
{
	struct msm8660_mpm *mpm = data;
	unsigned long pending[MSM8660_MPM_REG_WIDTH];
	unsigned long enable[MSM8660_MPM_REG_WIDTH];
	bool any_cleared = false;
	int i, j;

	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		pending[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_STATUS_OFFSET + i * 4);
		enable[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_REG_ENABLE + i * 4);
		pending[i] &= enable[i];

		/*
		 * Clear before dispatching: a new edge latched on this pin
		 * after this point will set the pending bit again and we
		 * will service it on the next IPC IRQ. Clearing AFTER the
		 * handler would race with that new latch and silently lose
		 * the new edge. Relaxed writes here are flushed by the
		 * readl + doorbell after the loop.
		 */
		if (pending[i]) {
			msm8660_mpm_write(mpm,
				MSM8660_MPM_REG_CLEAR + i * 4, pending[i]);
			any_cleared = true;
		}
	}

	/*
	 * Flush the relaxed CLEAR writes out of the CPU write buffer
	 * before we doorbell the RPM and before we return to the IRQ
	 * core (which EOIs the parent GIC). A read-back from the same
	 * MMIO window is the architecturally portable way to force
	 * preceding posted writes to complete.
	 */
	if (any_cleared) {
		(void)readl(mpm->base + MSM8660_MPM_REG_CLEAR);
		msm8660_mpm_doorbell(mpm);
	}

	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		unsigned long bits = pending[i];

		for_each_set_bit(j, &bits, 32) {
			unsigned int pin = i * 32 + j;

			/*
			 * Domain hwirq == MPM pin. Only pins that have a GIC
			 * SPI mapping are allocated through the irqdomain;
			 * dispatch them. Raw pins (declared in
			 * qcom,mpm-raw-pins) have no irqdomain consumer and
			 * are silently dropped here - their wake event is
			 * the IPC SPI itself, which the consumer's normal
			 * device-suspend path already handled by the time
			 * we get here.
			 */
			if (msm8660_mpm_pin_to_gic_spi(mpm, pin) >= 0) {
				dev_dbg(mpm->dev, "wake pin %u\n", pin);
				generic_handle_domain_irq(mpm->domain, pin);
			}
		}
	}

	return IRQ_HANDLED;
}

/*
 * Stage detect/polarity changes for a pin in the cached banks. Caller
 * must hold mpm->bus_lock. Returns 0 or -EINVAL for unsupported type.
 */
static int msm8660_mpm_stage_type(struct msm8660_mpm *mpm, unsigned int pin,
				  unsigned int type)
{
	u32 mask = BIT(pin % 32);
	unsigned int bank = pin / 32;
	u32 detect = mpm->detect[bank];
	u32 polarity = mpm->polarity[bank];

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		detect |= mask;
		polarity |= mask;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		detect |= mask;
		polarity &= ~mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		detect &= ~mask;
		polarity |= mask;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		detect &= ~mask;
		polarity &= ~mask;
		break;
	default:
		return -EINVAL;
	}

	mpm->detect[bank] = detect;
	mpm->polarity[bank] = polarity;
	return 0;
}

/*
 * Stage an ENABLE bit change. Caller must hold mpm->bus_lock.
 */
static void msm8660_mpm_stage_enable(struct msm8660_mpm *mpm, unsigned int pin,
				     bool enable)
{
	u32 mask = BIT(pin % 32);
	unsigned int bank = pin / 32;

	if (enable)
		mpm->enable[bank] |= mask;
	else
		mpm->enable[bank] &= ~mask;
}

/*
 * Commit any pending bank state to MMIO + doorbell the RPM. Caller
 * must hold mpm->bus_lock; runs in sleepable context so the mailbox
 * send is RT-safe.
 */
static void msm8660_mpm_commit(struct msm8660_mpm *mpm)
{
	bool changed = false;
	unsigned int i;

	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		if (mpm->enable[i] != mpm->enable_cached[i]) {
			msm8660_mpm_write(mpm,
				MSM8660_MPM_REG_ENABLE + i * 4,
				mpm->enable[i]);
			mpm->enable_cached[i] = mpm->enable[i];
			changed = true;
		}
		if (mpm->detect[i] != mpm->detect_cached[i]) {
			msm8660_mpm_write(mpm,
				MSM8660_MPM_REG_DETECT_CTL + i * 4,
				mpm->detect[i]);
			mpm->detect_cached[i] = mpm->detect[i];
			changed = true;
		}
		if (mpm->polarity[i] != mpm->polarity_cached[i]) {
			msm8660_mpm_write(mpm,
				MSM8660_MPM_REG_POLARITY + i * 4,
				mpm->polarity[i]);
			mpm->polarity_cached[i] = mpm->polarity[i];
			changed = true;
		}
	}

	if (changed) {
		(void)readl(mpm->base + MSM8660_MPM_REG_ENABLE);
		msm8660_mpm_doorbell(mpm);
	}
}

/* ===================================================================
 * irq_chip callbacks (run under irq_desc->lock for unmask/mask/set_type/
 * set_wake; only stage cached state, defer hardware writes to
 * bus_sync_unlock which runs in process context).
 * ===================================================================
 */

static void msm8660_mpm_bus_lock(struct irq_data *d)
{
	struct msm8660_mpm *mpm = irq_data_get_irq_chip_data(d);

	mutex_lock(&mpm->bus_lock);
}

static void msm8660_mpm_bus_sync_unlock(struct irq_data *d)
{
	struct msm8660_mpm *mpm = irq_data_get_irq_chip_data(d);

	msm8660_mpm_commit(mpm);
	mutex_unlock(&mpm->bus_lock);
}

/*
 * mask / unmask are pure parent-GIC operations. The MPM ENABLE bit
 * controls whether the pin is monitored during power-collapse; that is
 * orthogonal to whether the IRQ is logically masked at the GIC during
 * normal operation, and is owned by set_wake instead. Touching ENABLE
 * here would corrupt the wake state across an unmask-on-resume cycle
 * (the IRQ core does not call unmask if the IRQ is logically already
 * unmasked, even if our HW state was disabled by a recent set_wake(0)).
 */
static void msm8660_mpm_mask_irq(struct irq_data *d)
{
	irq_chip_mask_parent(d);
}

static void msm8660_mpm_unmask_irq(struct irq_data *d)
{
	irq_chip_unmask_parent(d);
}

static int msm8660_mpm_set_type(struct irq_data *d, unsigned int type)
{
	struct msm8660_mpm *mpm = irq_data_get_irq_chip_data(d);
	int ret;

	ret = msm8660_mpm_stage_type(mpm, d->hwirq, type);
	if (ret)
		return ret;

	return irq_chip_set_type_parent(d, type);
}

/*
 * Enable / disable MPM monitoring for this pin. Programming the HW
 * ENABLE bit is what makes the pin a wake source, so it is owned
 * exclusively by set_wake. The parent GIC's set_wake also runs so the
 * GIC stays alive during power-collapse to receive the IPC SPI.
 */
static int msm8660_mpm_set_wake(struct irq_data *d, unsigned int on)
{
	struct msm8660_mpm *mpm = irq_data_get_irq_chip_data(d);

	msm8660_mpm_stage_enable(mpm, d->hwirq, !!on);

	return irq_chip_set_wake_parent(d, on);
}

static struct irq_chip msm8660_mpm_chip = {
	.name			= "msm8660-mpm",
	.irq_mask		= msm8660_mpm_mask_irq,
	.irq_unmask		= msm8660_mpm_unmask_irq,
	.irq_set_type		= msm8660_mpm_set_type,
	.irq_set_wake		= msm8660_mpm_set_wake,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_bus_lock		= msm8660_mpm_bus_lock,
	.irq_bus_sync_unlock	= msm8660_mpm_bus_sync_unlock,
	.flags			= IRQCHIP_MASK_ON_SUSPEND,
};

static int msm8660_mpm_domain_alloc(struct irq_domain *domain,
				    unsigned int virq, unsigned int nr_irqs,
				    void *data)
{
	struct msm8660_mpm *mpm = domain->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	unsigned int pin;
	int gic_spi, i, ret;

	if (fwspec->param_count != 2)
		return -EINVAL;

	pin = fwspec->param[0];
	if (pin >= MSM8660_MPM_PIN_COUNT)
		return -EINVAL;

	/*
	 * Resolve the MPM pin to its parent GIC SPI. Only pins declared in
	 * qcom,mpm-pin-map have a GIC SPI; reject allocation for pins that
	 * are not (raw wake-only pins, or pins not in DT at all).
	 */
	gic_spi = msm8660_mpm_pin_to_gic_spi(mpm, pin);
	if (gic_spi < 0) {
		dev_err(mpm->dev,
			"MPM pin %u has no GIC SPI mapping in qcom,mpm-pin-map\n",
			pin);
		return -ENOENT;
	}

	/*
	 * Domain hwirq == MPM pin. The parent GIC's hwirq is the SPI we
	 * just looked up. Keeping them separate is essential because the
	 * IPC handler dispatches by pin number, while the parent fwspec
	 * needs an SPI for the GIC distributor.
	 */
	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_hwirq_and_chip(domain, virq + i, pin + i,
					      &msm8660_mpm_chip, mpm);

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0] = 0;
	parent_fwspec.param[1] = gic_spi;
	parent_fwspec.param[2] = fwspec->param[1];

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					   &parent_fwspec);
	if (ret) {
		dev_err(mpm->dev, "irq_domain_alloc_irqs_parent failed: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int msm8660_mpm_translate(struct irq_domain *d,
				 struct irq_fwspec *fwspec,
				 unsigned long *hwirq, unsigned int *type)
{
	if (fwspec->param_count != 2)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static const struct irq_domain_ops msm8660_mpm_domain_ops = {
	.translate	= msm8660_mpm_translate,
	.alloc		= msm8660_mpm_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

/* ===================================================================
 * Raw-pin consumer API
 * ===================================================================
 */

/**
 * msm8660_mpm_get() - acquire a handle to the MPM for raw-pin use
 * @consumer: device of the consumer driver
 * @np: optional device-tree node containing a phandle reference
 * @propname: optional property name for that phandle (e.g. "qcom,mpm")
 *
 * Returns the singleton MPM handle, ERR_PTR(-EPROBE_DEFER) if the MPM
 * driver has not finished probing yet, or ERR_PTR(-ENOENT) if @np is
 * given and the phandle does not resolve.
 *
 * On success this also establishes a consumer-supplier device_link so
 * the MPM device cannot be unbound while the consumer holds the
 * handle. The link is auto-removed when @consumer is unbound.
 */
struct msm8660_mpm *msm8660_mpm_get(struct device *consumer,
				    struct device_node *np,
				    const char *propname)
{
	struct msm8660_mpm *mpm = smp_load_acquire(&msm8660_mpm_global);
	struct device_node *mpm_np;
	struct device_link *link;

	if (!mpm)
		return ERR_PTR(-EPROBE_DEFER);

	if (np && propname) {
		mpm_np = of_parse_phandle(np, propname, 0);
		if (!mpm_np)
			return ERR_PTR(-ENOENT);
		/*
		 * The phandle must resolve to our own node. Without this
		 * check a typo in the consumer's DT would silently return
		 * the global handle and grant access to MPM pins the
		 * integrator did not intend to expose.
		 */
		if (mpm_np != mpm->dev->of_node) {
			of_node_put(mpm_np);
			return ERR_PTR(-ENOENT);
		}
		of_node_put(mpm_np);
	}

	if (!consumer)
		return mpm;

	link = device_link_add(consumer, mpm->dev,
			       DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link) {
		dev_warn(consumer, "failed to link to MPM, deferring\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	return mpm;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_get);

/**
 * msm8660_mpm_enable_pin() - enable/disable MPM monitoring of a pin
 * @mpm: handle from msm8660_mpm_get()
 * @pin: MPM pin index (0..MSM8660_MPM_PIN_COUNT-1)
 * @enable: true to monitor, false to ignore
 *
 * Programs the ENABLE register directly. Intended for "raw" wake pins
 * (SDC3_DAT1=21, SDC3_DAT3=22, SDC4_DAT1=23, SDC4_DAT3=24) that have no
 * GIC IRQ mapping. For pins that DO have a GIC mapping (in
 * qcom,mpm-pin-map), use the irqdomain path instead.
 *
 * The pin must be declared in qcom,mpm-raw-pins; otherwise the call is
 * rejected with -EINVAL.
 */
int msm8660_mpm_enable_pin(struct msm8660_mpm *mpm, unsigned int pin,
			   bool enable)
{
	if (!mpm || pin >= MSM8660_MPM_PIN_COUNT)
		return -EINVAL;
	if (!msm8660_mpm_raw_pin_allowed(mpm, pin)) {
		dev_warn(mpm->dev,
			 "pin %u not declared in qcom,mpm-raw-pins\n", pin);
		return -EINVAL;
	}

	mutex_lock(&mpm->bus_lock);
	msm8660_mpm_stage_enable(mpm, pin, enable);
	msm8660_mpm_commit(mpm);
	mutex_unlock(&mpm->bus_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_enable_pin);

/**
 * msm8660_mpm_set_pin_wake() - mark a pin as wake-capable
 * @mpm: handle
 * @pin: MPM pin index
 * @on:  true to allow this pin to wake the system, false to clear.
 *
 * Equivalent to msm8660_mpm_enable_pin() on MSM8660 - the hardware has
 * a single ENABLE register, not separate enable + wake masks. The name
 * is kept for API parity with the legacy mpm.h interface so consumers
 * can express the wake-source intent explicitly.
 */
int msm8660_mpm_set_pin_wake(struct msm8660_mpm *mpm, unsigned int pin,
			     bool on)
{
	return msm8660_mpm_enable_pin(mpm, pin, on);
}
EXPORT_SYMBOL_GPL(msm8660_mpm_set_pin_wake);

/**
 * msm8660_mpm_set_pin_type() - set trigger type for a raw MPM pin
 * @mpm: handle
 * @pin: MPM pin index
 * @flow_type: standard IRQ_TYPE_* constants
 *
 * On MSM8660 the trigger config is split across DETECT_CTL (edge vs
 * level) and POLARITY (rising/high vs falling/low).
 */
int msm8660_mpm_set_pin_type(struct msm8660_mpm *mpm, unsigned int pin,
			     unsigned int flow_type)
{
	unsigned int sense = flow_type & IRQ_TYPE_SENSE_MASK;
	int ret;

	if (!mpm || pin >= MSM8660_MPM_PIN_COUNT)
		return -EINVAL;
	if (!msm8660_mpm_raw_pin_allowed(mpm, pin)) {
		dev_warn(mpm->dev,
			 "pin %u not declared in qcom,mpm-raw-pins\n", pin);
		return -EINVAL;
	}

	mutex_lock(&mpm->bus_lock);
	ret = msm8660_mpm_stage_type(mpm, pin, sense);
	if (!ret)
		msm8660_mpm_commit(mpm);
	mutex_unlock(&mpm->bus_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_set_pin_type);

/* ===================================================================
 * Platform driver
 * ===================================================================
 */

static int msm8660_mpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct msm8660_mpm *mpm;
	struct irq_domain *parent_domain;
	struct device_node *parent_np;
	struct resource *res;
	int ret, i;

	if (msm8660_mpm_global)
		return dev_err_probe(dev, -EBUSY,
				     "only one MPM instance is supported\n");

	mpm = devm_kzalloc(dev, sizeof(*mpm), GFP_KERNEL);
	if (!mpm)
		return -ENOMEM;

	mpm->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -ENODEV, "missing reg property\n");

	/*
	 * Use a non-exclusive mapping: the qcom,rpm driver maps the
	 * surrounding RPM control block via its own platform resource,
	 * and our vMPM sub-region overlaps that mapping. devm_ioremap()
	 * does not call request_mem_region() so there is no conflict.
	 */
	mpm->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!mpm->base)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to ioremap vMPM at %pR\n", res);

	mutex_init(&mpm->bus_lock);

	/*
	 * Seed the cache from whatever state the firmware / boot loader
	 * left in the vMPM banks. Anything we have not staged ourselves
	 * (rare except for boot-time wake configuration) is preserved on
	 * the next commit because commit() skips banks whose pending value
	 * still matches the cached value.
	 */
	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		mpm->enable_cached[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_REG_ENABLE + i * 4);
		mpm->detect_cached[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_REG_DETECT_CTL + i * 4);
		mpm->polarity_cached[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_REG_POLARITY + i * 4);
		mpm->enable[i] = mpm->enable_cached[i];
		mpm->detect[i] = mpm->detect_cached[i];
		mpm->polarity[i] = mpm->polarity_cached[i];
	}

	/*
	 * Parse pin map (IRQ-mapped wake pins; raw pins like SDC4_DAT1
	 * are not listed here - they are accessed via the pin-API).
	 */
	ret = of_property_count_u32_elems(np, "qcom,mpm-pin-map");
	if (ret < 0 || ret % 2)
		return dev_err_probe(dev, -EINVAL,
				     "invalid qcom,mpm-pin-map\n");

	mpm->pin_map_count = ret / 2;
	mpm->pin_map = devm_kcalloc(dev, mpm->pin_map_count,
				    sizeof(*mpm->pin_map), GFP_KERNEL);
	if (!mpm->pin_map)
		return -ENOMEM;

	for (i = 0; i < mpm->pin_map_count; i++) {
		u32 pin, hwirq;

		of_property_read_u32_index(np, "qcom,mpm-pin-map",
					   i * 2, &pin);
		of_property_read_u32_index(np, "qcom,mpm-pin-map",
					   i * 2 + 1, &hwirq);

		if (pin >= MSM8660_MPM_PIN_COUNT)
			return dev_err_probe(dev, -EINVAL,
				"qcom,mpm-pin-map entry %d: pin %u >= %u\n",
				i, pin, MSM8660_MPM_PIN_COUNT);

		mpm->pin_map[i].pin = pin;
		mpm->pin_map[i].hwirq = hwirq;

		dev_dbg(dev, "pin map: pin %u -> hwirq %u\n", pin, hwirq);
	}

	/*
	 * Parse raw wake pins (no GIC SPI mapping). Optional: a board
	 * that does not use any direct MPM wake source can omit the
	 * property and the raw-pin API will reject every request.
	 */
	ret = of_property_count_u32_elems(np, "qcom,mpm-raw-pins");
	if (ret > 0) {
		mpm->raw_pin_count = ret;
		mpm->raw_pins = devm_kcalloc(dev, ret, sizeof(*mpm->raw_pins),
					     GFP_KERNEL);
		if (!mpm->raw_pins)
			return -ENOMEM;

		of_property_read_u32_array(np, "qcom,mpm-raw-pins",
					   mpm->raw_pins, ret);
		for (i = 0; i < mpm->raw_pin_count; i++) {
			if (mpm->raw_pins[i] >= MSM8660_MPM_PIN_COUNT)
				return dev_err_probe(dev, -EINVAL,
					"qcom,mpm-raw-pins entry %d: pin %u >= %u\n",
					i, mpm->raw_pins[i],
					MSM8660_MPM_PIN_COUNT);
			dev_dbg(dev, "raw pin: %u\n", mpm->raw_pins[i]);
		}
	}

	parent_np = of_irq_find_parent(np);
	if (!parent_np)
		return dev_err_probe(dev, -ENODEV,
				     "failed to find parent interrupt controller\n");

	parent_domain = irq_find_host(parent_np);
	of_node_put(parent_np);
	if (!parent_domain)
		return dev_err_probe(dev, -ENODEV,
				     "failed to find parent IRQ domain\n");

	mpm->domain = irq_domain_create_hierarchy(parent_domain, 0,
						  MSM8660_MPM_PIN_COUNT,
						  of_fwnode_handle(np),
						  &msm8660_mpm_domain_ops,
						  mpm);
	if (!mpm->domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create IRQ domain\n");

	mpm->parent_irq = platform_get_irq(pdev, 0);
	if (mpm->parent_irq < 0) {
		ret = mpm->parent_irq;
		goto err_remove_domain;
	}

	/*
	 * Mailbox channel for poking MPM to re-read its config. Get this
	 * BEFORE registering our IRQ handler so that doorbell-from-IRQ
	 * never sees a partially-initialised channel pointer.
	 */
	mpm->mbox_client.dev = dev;
	mpm->mbox_client.knows_txdone = true;
	mpm->mbox_chan = mbox_request_channel(&mpm->mbox_client, 0);
	if (IS_ERR(mpm->mbox_chan)) {
		ret = PTR_ERR(mpm->mbox_chan);
		mpm->mbox_chan = NULL;
		if (ret == -EPROBE_DEFER)
			goto err_remove_domain;
		dev_warn(dev, "no mailbox channel: %d (continuing without RPM doorbell)\n",
			 ret);
	}

	/*
	 * .suppress_bind_attrs = true prevents the driver from ever being
	 * unbound, so devm_request_irq is safe: the handler cannot outlive
	 * the irqdomain because both have the lifetime of the kernel.
	 */
	ret = devm_request_irq(dev, mpm->parent_irq, msm8660_mpm_irq,
			       IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			       "msm8660-mpm", mpm);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d: %d\n",
			mpm->parent_irq, ret);
		goto err_free_mbox;
	}

	platform_set_drvdata(pdev, mpm);
	/* Publish only after everything is wired up. */
	smp_store_release(&msm8660_mpm_global, mpm);

	dev_info(dev, "ready: %u pin mappings, %u raw pins, irq=%d\n",
		 mpm->pin_map_count, mpm->raw_pin_count, mpm->parent_irq);

	return 0;

err_free_mbox:
	if (mpm->mbox_chan)
		mbox_free_channel(mpm->mbox_chan);
err_remove_domain:
	irq_domain_remove(mpm->domain);
	return ret;
}

static const struct of_device_id msm8660_mpm_of_match[] = {
	{ .compatible = "qcom,msm8660-mpm" },
	{}
};
MODULE_DEVICE_TABLE(of, msm8660_mpm_of_match);

/*
 * No ->remove(): this is a wake-controller required for system suspend
 * and is referenced by the irqdomain hierarchy of every consumer that
 * routes a wake source through it. Allowing the driver to be unbound
 * while consumers hold virq mappings would leave dangling pointers to
 * the freed irq_domain inside the core IRQ subsystem (irq_data of each
 * mapped IRQ). Combined with .suppress_bind_attrs = true below, this
 * means the driver binds once at boot and stays bound for the lifetime
 * of the system, which also lets msm8660_mpm_global be safely read
 * lockless from msm8660_mpm_get().
 */
static struct platform_driver msm8660_mpm_driver = {
	.probe		= msm8660_mpm_probe,
	.driver		= {
		.name			= "msm8660-mpm",
		.of_match_table		= msm8660_mpm_of_match,
		.suppress_bind_attrs	= true,
	},
};

static int __init msm8660_mpm_init(void)
{
	return platform_driver_register(&msm8660_mpm_driver);
}
subsys_initcall(msm8660_mpm_init);

MODULE_DESCRIPTION("Qualcomm MSM8x60 MPM wakeup interrupt controller");
MODULE_LICENSE("GPL");
