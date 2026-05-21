// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSM8660 / APQ8060 MPM (MSM Power Manager) wakeup interrupt controller
 *
 * The MPM is an always-on hardware block that keeps a small set of wake
 * sources alive while the application processor is powered down for
 * cpuidle Power Collapse or suspend-to-RAM. On MSM8660 / APQ8060 the
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
 * infrastructure is ready, accesses RPM via a syscon phandle, and uses
 * the qcom-apcs-ipc mailbox for wake notification (writing to
 * GCC + 0x008 bit 1).
 *
 * Two consumer interfaces:
 *
 *   1. Hierarchical irqdomain: for MPM pins that map to GIC SPIs (USB,
 *      HDMI, ...). Consumers wire their interrupts through this
 *      controller via interrupts-extended and the kernel manages
 *      enable / mask / set_type via the IRQ subsystem. This is the
 *      mainline-recommended path for IRQ-mappable wake sources.
 *
 *   2. Raw-pin API: for MPM pins that do NOT correspond to a GIC IRQ
 *      (SDC3_DAT1=21, SDC3_DAT3=22, SDC4_DAT1=23, SDC4_DAT3=24).
 *      These are physical wake-signal lines monitored by MPM
 *      directly. Consumers (mmci for SDC4 wake) call
 *      msm8660_mpm_set_pin_wake() etc. to twiddle the MPM enable
 *      register for these "raw" pins.
 *
 * Copyright (c) 2026, Herman van Hazendonk
 * Copyright (c) 2010-2012, The Linux Foundation (legacy mpm.c reference)
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <soc/qcom/msm8660-mpm.h>

/* vMPM register offsets (relative to RPM base + 0x9d8). Each register is
 * a window of two 32-bit slots since MPM exposes 64 wake pins.
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
	struct regmap *regmap;
	unsigned int regmap_offset;
	struct irq_domain *domain;
	struct msm8660_mpm_pin *pin_map;
	unsigned int pin_map_count;
	int parent_irq;
	raw_spinlock_t lock;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
};

/* Singleton — there is only one MPM instance per SoC. The pin-API
 * lookup helpers (msm8660_mpm_get()) return this.
 */
static struct msm8660_mpm *msm8660_mpm_global;

static u32 msm8660_mpm_read(struct msm8660_mpm *mpm, unsigned int reg)
{
	u32 val;
	regmap_read(mpm->regmap, mpm->regmap_offset + reg, &val);
	return val;
}

static void msm8660_mpm_write(struct msm8660_mpm *mpm, unsigned int reg,
			      u32 val)
{
	regmap_write(mpm->regmap, mpm->regmap_offset + reg, val);
}

static int msm8660_mpm_pin_to_hwirq(struct msm8660_mpm *mpm, int pin)
{
	int i;

	for (i = 0; i < mpm->pin_map_count; i++) {
		if (mpm->pin_map[i].pin == pin)
			return mpm->pin_map[i].hwirq;
	}
	return -ENOENT;
}

/*
 * IPC handler: MPM fires this IRQ when one or more enabled wake pins
 * have pending activity. Read pending status, for each pin look up the
 * corresponding GIC hwirq (if mapped), replay it through the
 * irqdomain, then clear the pending bit.
 */
static irqreturn_t msm8660_mpm_irq(int irq, void *data)
{
	struct msm8660_mpm *mpm = data;
	unsigned long pending[MSM8660_MPM_REG_WIDTH];
	unsigned long enable[MSM8660_MPM_REG_WIDTH];
	int i, j;

	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		pending[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_STATUS_OFFSET + i * 4);
		enable[i] = msm8660_mpm_read(mpm,
			MSM8660_MPM_REG_ENABLE + i * 4);
	}

	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		unsigned long bits = pending[i] & enable[i];

		for_each_set_bit(j, &bits, 32) {
			int pin = i * 32 + j;
			int hwirq = msm8660_mpm_pin_to_hwirq(mpm, pin);

			if (hwirq >= 0) {
				dev_dbg(mpm->dev, "wake pin %d -> hwirq %d\n",
					pin, hwirq);
				generic_handle_domain_irq(mpm->domain, hwirq);
			}
		}

		if (bits)
			msm8660_mpm_write(mpm,
				MSM8660_MPM_REG_CLEAR + i * 4, bits);
	}

	return IRQ_HANDLED;
}

static void msm8660_mpm_enable_hwirq(struct irq_data *d, bool enable)
{
	struct msm8660_mpm *mpm = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	int i, pin = -1;
	u32 val;

	for (i = 0; i < mpm->pin_map_count; i++) {
		if (mpm->pin_map[i].hwirq == d->hwirq) {
			pin = mpm->pin_map[i].pin;
			break;
		}
	}

	if (pin < 0)
		return;

	raw_spin_lock_irqsave(&mpm->lock, flags);

	val = msm8660_mpm_read(mpm,
		MSM8660_MPM_REG_ENABLE + (pin / 32) * 4);
	if (enable)
		val |= BIT(pin % 32);
	else
		val &= ~BIT(pin % 32);
	msm8660_mpm_write(mpm,
		MSM8660_MPM_REG_ENABLE + (pin / 32) * 4, val);

	raw_spin_unlock_irqrestore(&mpm->lock, flags);
}

static void msm8660_mpm_mask_irq(struct irq_data *d)
{
	msm8660_mpm_enable_hwirq(d, false);
	irq_chip_mask_parent(d);
}

static void msm8660_mpm_unmask_irq(struct irq_data *d)
{
	msm8660_mpm_enable_hwirq(d, true);
	irq_chip_unmask_parent(d);
}

static int msm8660_mpm_set_type(struct irq_data *d, unsigned int type)
{
	struct msm8660_mpm *mpm = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	int i, pin = -1;
	u32 detect, polarity;

	for (i = 0; i < mpm->pin_map_count; i++) {
		if (mpm->pin_map[i].hwirq == d->hwirq) {
			pin = mpm->pin_map[i].pin;
			break;
		}
	}

	if (pin < 0)
		return -ENOENT;

	raw_spin_lock_irqsave(&mpm->lock, flags);

	detect = msm8660_mpm_read(mpm,
		MSM8660_MPM_REG_DETECT_CTL + (pin / 32) * 4);
	polarity = msm8660_mpm_read(mpm,
		MSM8660_MPM_REG_POLARITY + (pin / 32) * 4);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		detect |= BIT(pin % 32);
		polarity |= BIT(pin % 32);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		detect |= BIT(pin % 32);
		polarity &= ~BIT(pin % 32);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		detect &= ~BIT(pin % 32);
		polarity |= BIT(pin % 32);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		detect &= ~BIT(pin % 32);
		polarity &= ~BIT(pin % 32);
		break;
	default:
		raw_spin_unlock_irqrestore(&mpm->lock, flags);
		return -EINVAL;
	}

	msm8660_mpm_write(mpm,
		MSM8660_MPM_REG_DETECT_CTL + (pin / 32) * 4, detect);
	msm8660_mpm_write(mpm,
		MSM8660_MPM_REG_POLARITY + (pin / 32) * 4, polarity);

	raw_spin_unlock_irqrestore(&mpm->lock, flags);

	return irq_chip_set_type_parent(d, type);
}

static struct irq_chip msm8660_mpm_chip = {
	.name			= "msm8660-mpm",
	.irq_mask		= msm8660_mpm_mask_irq,
	.irq_unmask		= msm8660_mpm_unmask_irq,
	.irq_set_type		= msm8660_mpm_set_type,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_SKIP_SET_WAKE,
};

static int msm8660_mpm_domain_alloc(struct irq_domain *domain,
				    unsigned int virq,
				    unsigned int nr_irqs, void *data)
{
	struct msm8660_mpm *mpm = domain->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	int hwirq, ret;

	if (fwspec->param_count != 2)
		return -EINVAL;

	hwirq = fwspec->param[0];

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0] = 0;	/* GIC_SPI */
	parent_fwspec.param[1] = hwirq;
	parent_fwspec.param[2] = fwspec->param[1];

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					   &parent_fwspec);
	if (ret)
		return ret;

	irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
				      &msm8660_mpm_chip, mpm);

	return 0;
}

static const struct irq_domain_ops msm8660_mpm_domain_ops = {
	.alloc	= msm8660_mpm_domain_alloc,
	.free	= irq_domain_free_irqs_common,
	.translate = irq_domain_translate_twocell,
};

/* ===================================================================
 * Raw-pin API for wake sources without a GIC IRQ mapping (SDC3/4 DATx)
 * ===================================================================
 *
 * These helpers operate on MPM pins directly via the ENABLE / DETECT_CTL
 * / POLARITY registers. Used by consumer drivers (mmci for SDC4 wake)
 * that need to monitor a physical signal line via MPM but don't have a
 * corresponding GIC interrupt to route through the irqdomain.
 */

/**
 * msm8660_mpm_get() - get a handle to the MPM device
 * @np: device node of the consumer (used for phandle lookup)
 * @propname: phandle property name; if NULL, returns the singleton
 *            without phandle verification.
 *
 * Returns the singleton MPM handle, or ERR_PTR(-EPROBE_DEFER) if the
 * MPM driver hasn't probed yet.
 */
struct msm8660_mpm *msm8660_mpm_get(struct device_node *np,
				    const char *propname)
{
	struct device_node *mpm_np;

	if (!msm8660_mpm_global)
		return ERR_PTR(-EPROBE_DEFER);

	if (np && propname) {
		mpm_np = of_parse_phandle(np, propname, 0);
		if (!mpm_np)
			return ERR_PTR(-ENOENT);
		of_node_put(mpm_np);
	}

	return msm8660_mpm_global;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_get);

/**
 * msm8660_mpm_enable_pin() - enable/disable MPM monitoring of a pin
 * @mpm: handle from msm8660_mpm_get()
 * @pin: MPM pin index (0..63)
 * @enable: true to monitor, false to ignore
 *
 * Programs the ENABLE register directly. Intended for "raw" wake pins
 * (SDC3_DAT1=21, SDC3_DAT3=22, SDC4_DAT1=23, SDC4_DAT3=24) that have no
 * GIC IRQ mapping. For pins that DO have a GIC mapping (in
 * qcom,mpm-pin-map), use the irqdomain path instead.
 */
int msm8660_mpm_enable_pin(struct msm8660_mpm *mpm, unsigned int pin,
			   bool enable)
{
	unsigned long flags;
	u32 val;

	if (!mpm || pin >= MSM8660_MPM_PIN_COUNT)
		return -EINVAL;

	raw_spin_lock_irqsave(&mpm->lock, flags);
	val = msm8660_mpm_read(mpm,
		MSM8660_MPM_REG_ENABLE + (pin / 32) * 4);
	if (enable)
		val |= BIT(pin % 32);
	else
		val &= ~BIT(pin % 32);
	msm8660_mpm_write(mpm,
		MSM8660_MPM_REG_ENABLE + (pin / 32) * 4, val);
	raw_spin_unlock_irqrestore(&mpm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(msm8660_mpm_enable_pin);

/**
 * msm8660_mpm_set_pin_wake() - mark a pin as wake-capable
 * @mpm: handle
 * @pin: MPM pin index
 * @on:  true to allow this pin to wake the system, false to clear.
 *
 * Equivalent to msm8660_mpm_enable_pin() on MSM8660 — the hardware has
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
	unsigned long flags;
	u32 detect, polarity;
	bool edge, polarity_high;

	if (!mpm || pin >= MSM8660_MPM_PIN_COUNT)
		return -EINVAL;

	edge = !!(flow_type & IRQ_TYPE_EDGE_BOTH);
	polarity_high = !!(flow_type & (IRQ_TYPE_EDGE_RISING |
					IRQ_TYPE_LEVEL_HIGH));

	raw_spin_lock_irqsave(&mpm->lock, flags);

	detect = msm8660_mpm_read(mpm,
		MSM8660_MPM_REG_DETECT_CTL + (pin / 32) * 4);
	polarity = msm8660_mpm_read(mpm,
		MSM8660_MPM_REG_POLARITY + (pin / 32) * 4);

	if (edge)
		detect |= BIT(pin % 32);
	else
		detect &= ~BIT(pin % 32);

	if (polarity_high)
		polarity |= BIT(pin % 32);
	else
		polarity &= ~BIT(pin % 32);

	msm8660_mpm_write(mpm,
		MSM8660_MPM_REG_DETECT_CTL + (pin / 32) * 4, detect);
	msm8660_mpm_write(mpm,
		MSM8660_MPM_REG_POLARITY + (pin / 32) * 4, polarity);

	raw_spin_unlock_irqrestore(&mpm->lock, flags);

	return 0;
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
	int ret, i;
	u32 offset;

	if (msm8660_mpm_global) {
		dev_err(dev, "only one MPM instance is supported\n");
		return -EBUSY;
	}

	mpm = devm_kzalloc(dev, sizeof(*mpm), GFP_KERNEL);
	if (!mpm)
		return -ENOMEM;

	mpm->dev = dev;
	raw_spin_lock_init(&mpm->lock);

	mpm->regmap = syscon_regmap_lookup_by_phandle(np, "qcom,rpm-syscon");
	if (IS_ERR(mpm->regmap)) {
		dev_err(dev, "failed to get RPM syscon: %ld\n",
			PTR_ERR(mpm->regmap));
		return PTR_ERR(mpm->regmap);
	}

	ret = of_property_read_u32(np, "qcom,mpm-offset", &offset);
	if (ret) {
		dev_err(dev, "failed to read qcom,mpm-offset: %d\n", ret);
		return ret;
	}
	mpm->regmap_offset = offset;

	dev_info(dev, "vMPM registers at RPM offset 0x%x\n", offset);

	/* Parse pin map (IRQ-mapped wake pins; raw pins like SDC4_DAT1 are
	 * not listed here — they're accessed via the pin-API below).
	 */
	ret = of_property_count_u32_elems(np, "qcom,mpm-pin-map");
	if (ret < 0 || ret % 2) {
		dev_err(dev, "invalid qcom,mpm-pin-map\n");
		return -EINVAL;
	}

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

		mpm->pin_map[i].pin = pin;
		mpm->pin_map[i].hwirq = hwirq;

		dev_dbg(dev, "pin map: pin %d -> hwirq %d\n", pin, hwirq);
	}

	parent_np = of_irq_find_parent(np);
	if (!parent_np) {
		dev_err(dev, "failed to find parent interrupt controller\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent_np);
	of_node_put(parent_np);
	if (!parent_domain) {
		dev_err(dev, "failed to find parent IRQ domain\n");
		return -ENODEV;
	}

	mpm->domain = irq_domain_create_hierarchy(parent_domain, 0,
						  MSM8660_MPM_PIN_COUNT,
						  of_node_to_fwnode(np),
						  &msm8660_mpm_domain_ops,
						  mpm);
	if (!mpm->domain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	mpm->parent_irq = platform_get_irq(pdev, 0);
	if (mpm->parent_irq < 0) {
		ret = mpm->parent_irq;
		goto err_remove_domain;
	}

	ret = devm_request_irq(dev, mpm->parent_irq, msm8660_mpm_irq,
			       IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			       "msm8660-mpm", mpm);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d: %d\n",
			mpm->parent_irq, ret);
		goto err_remove_domain;
	}

	/* Mailbox channel for poking MPM to re-read its config (writes to
	 * GCC + 0x008 bit 1 via the qcom-apcs-ipc mailbox).
	 */
	mpm->mbox_client.dev = dev;
	mpm->mbox_client.knows_txdone = true;
	mpm->mbox_chan = mbox_request_channel(&mpm->mbox_client, 0);
	if (IS_ERR(mpm->mbox_chan)) {
		dev_warn(dev, "no mailbox channel: %ld (continuing)\n",
			 PTR_ERR(mpm->mbox_chan));
		mpm->mbox_chan = NULL;
	}

	platform_set_drvdata(pdev, mpm);
	msm8660_mpm_global = mpm;

	dev_info(dev, "ready: %d pin mappings, irq=%d\n",
		 mpm->pin_map_count, mpm->parent_irq);

	return 0;

err_remove_domain:
	irq_domain_remove(mpm->domain);
	return ret;
}

static void msm8660_mpm_remove(struct platform_device *pdev)
{
	struct msm8660_mpm *mpm = platform_get_drvdata(pdev);

	msm8660_mpm_global = NULL;
	if (mpm->mbox_chan)
		mbox_free_channel(mpm->mbox_chan);
	irq_domain_remove(mpm->domain);
}

static const struct of_device_id msm8660_mpm_of_match[] = {
	{ .compatible = "qcom,msm8660-mpm" },
	{}
};
MODULE_DEVICE_TABLE(of, msm8660_mpm_of_match);

static struct platform_driver msm8660_mpm_driver = {
	.probe		= msm8660_mpm_probe,
	.remove		= msm8660_mpm_remove,
	.driver		= {
		.name	= "msm8660-mpm",
		.of_match_table = msm8660_mpm_of_match,
	},
};

static int __init msm8660_mpm_init(void)
{
	return platform_driver_register(&msm8660_mpm_driver);
}
subsys_initcall(msm8660_mpm_init);

static void __exit msm8660_mpm_exit(void)
{
	platform_driver_unregister(&msm8660_mpm_driver);
}
module_exit(msm8660_mpm_exit);

MODULE_DESCRIPTION("MSM8660 / APQ8060 MPM wakeup interrupt controller");
MODULE_LICENSE("GPL");
