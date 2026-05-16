// SPDX-License-Identifier: GPL-2.0-only
/*
 * MSM8660 Wakeup Interrupt Controller
 *
 * Minimal driver for MSM8660 vMPM (virtual MSM Power Manager) wakeup interrupts.
 * Unlike the generic qcom-mpm driver which uses IRQCHIP_MATCH early init, this
 * is a regular platform driver that can properly handle syscon/regmap and RPM
 * dependencies.
 *
 * The vMPM registers live inside the RPM memory region at offset 0x9d8 and are
 * accessed via syscon to avoid resource conflicts.
 *
 * Copyright (c) 2026, Herman van Hazendonk
 * Copyright (c) 2010-2012, The Linux Foundation (legacy mpm.c reference)
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

/* MSM8660 vMPM register offsets (relative to RPM base + 0x9d8) */
#define MSM8660_MPM_REG_ENABLE		0x00
#define MSM8660_MPM_REG_DETECT_CTL	0x08
#define MSM8660_MPM_REG_POLARITY	0x10
#define MSM8660_MPM_REG_CLEAR		0x18

/* Status registers at offset 0x420 from vMPM base */
#define MSM8660_MPM_STATUS_OFFSET	0x420

/* Number of MPM pins (64 pins = 2 x 32-bit registers) */
#define MSM8660_MPM_PIN_COUNT		64
#define MSM8660_MPM_REG_WIDTH		2

struct msm8660_wakeup_pin {
	int pin;
	int hwirq;
};

struct msm8660_wakeup {
	struct device *dev;
	struct regmap *regmap;
	unsigned int regmap_offset;
	struct irq_domain *domain;
	struct msm8660_wakeup_pin *pin_map;
	unsigned int pin_map_count;
	int parent_irq;
	raw_spinlock_t lock;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
};

static u32 msm8660_wakeup_read(struct msm8660_wakeup *wakeup, unsigned int reg)
{
	u32 val;
	regmap_read(wakeup->regmap, wakeup->regmap_offset + reg, &val);
	return val;
}

static void msm8660_wakeup_write(struct msm8660_wakeup *wakeup, unsigned int reg, u32 val)
{
	regmap_write(wakeup->regmap, wakeup->regmap_offset + reg, val);
}

static int msm8660_wakeup_pin_to_hwirq(struct msm8660_wakeup *wakeup, int pin)
{
	int i;

	for (i = 0; i < wakeup->pin_map_count; i++) {
		if (wakeup->pin_map[i].pin == pin)
			return wakeup->pin_map[i].hwirq;
	}
	return -ENOENT;
}

static irqreturn_t msm8660_wakeup_irq(int irq, void *data)
{
	struct msm8660_wakeup *wakeup = data;
	unsigned long pending[MSM8660_MPM_REG_WIDTH];
	unsigned long enable[MSM8660_MPM_REG_WIDTH];
	int i, j;

	/* Read pending and enabled interrupts */
	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		pending[i] = msm8660_wakeup_read(wakeup,
			MSM8660_MPM_STATUS_OFFSET + i * 4);
		enable[i] = msm8660_wakeup_read(wakeup,
			MSM8660_MPM_REG_ENABLE + i * 4);
	}

	/* Handle each pending enabled interrupt */
	for (i = 0; i < MSM8660_MPM_REG_WIDTH; i++) {
		unsigned long bits = pending[i] & enable[i];

		for_each_set_bit(j, &bits, 32) {
			int pin = i * 32 + j;
			int hwirq = msm8660_wakeup_pin_to_hwirq(wakeup, pin);

			if (hwirq >= 0) {
				dev_dbg(wakeup->dev, "wakeup pin %d -> hwirq %d\n",
					pin, hwirq);
				generic_handle_domain_irq(wakeup->domain, hwirq);
			}
		}

		/* Clear the pending interrupts for this register */
		if (bits)
			msm8660_wakeup_write(wakeup,
				MSM8660_MPM_REG_CLEAR + i * 4, bits);
	}

	return IRQ_HANDLED;
}

static void msm8660_wakeup_enable_irq(struct irq_data *d, bool enable)
{
	struct msm8660_wakeup *wakeup = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	int i, pin = -1;
	u32 val;

	/* Find MPM pin for this hwirq */
	for (i = 0; i < wakeup->pin_map_count; i++) {
		if (wakeup->pin_map[i].hwirq == d->hwirq) {
			pin = wakeup->pin_map[i].pin;
			break;
		}
	}

	if (pin < 0)
		return;

	raw_spin_lock_irqsave(&wakeup->lock, flags);

	/* Read-modify-write the enable register */
	val = msm8660_wakeup_read(wakeup,
		MSM8660_MPM_REG_ENABLE + (pin / 32) * 4);
	if (enable)
		val |= BIT(pin % 32);
	else
		val &= ~BIT(pin % 32);
	msm8660_wakeup_write(wakeup,
		MSM8660_MPM_REG_ENABLE + (pin / 32) * 4, val);

	raw_spin_unlock_irqrestore(&wakeup->lock, flags);
}

static void msm8660_wakeup_mask_irq(struct irq_data *d)
{
	msm8660_wakeup_enable_irq(d, false);
	irq_chip_mask_parent(d);
}

static void msm8660_wakeup_unmask_irq(struct irq_data *d)
{
	msm8660_wakeup_enable_irq(d, true);
	irq_chip_unmask_parent(d);
}

static int msm8660_wakeup_set_type(struct irq_data *d, unsigned int type)
{
	struct msm8660_wakeup *wakeup = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	int i, pin = -1;
	u32 detect, polarity;

	/* Find MPM pin for this hwirq */
	for (i = 0; i < wakeup->pin_map_count; i++) {
		if (wakeup->pin_map[i].hwirq == d->hwirq) {
			pin = wakeup->pin_map[i].pin;
			break;
		}
	}

	if (pin < 0)
		return -ENOENT;

	raw_spin_lock_irqsave(&wakeup->lock, flags);

	detect = msm8660_wakeup_read(wakeup,
		MSM8660_MPM_REG_DETECT_CTL + (pin / 32) * 4);
	polarity = msm8660_wakeup_read(wakeup,
		MSM8660_MPM_REG_POLARITY + (pin / 32) * 4);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		detect |= BIT(pin % 32);	/* Edge */
		polarity |= BIT(pin % 32);	/* Rising */
		break;
	case IRQ_TYPE_EDGE_FALLING:
		detect |= BIT(pin % 32);	/* Edge */
		polarity &= ~BIT(pin % 32);	/* Falling */
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		detect &= ~BIT(pin % 32);	/* Level */
		polarity |= BIT(pin % 32);	/* High */
		break;
	case IRQ_TYPE_LEVEL_LOW:
		detect &= ~BIT(pin % 32);	/* Level */
		polarity &= ~BIT(pin % 32);	/* Low */
		break;
	default:
		raw_spin_unlock_irqrestore(&wakeup->lock, flags);
		return -EINVAL;
	}

	msm8660_wakeup_write(wakeup,
		MSM8660_MPM_REG_DETECT_CTL + (pin / 32) * 4, detect);
	msm8660_wakeup_write(wakeup,
		MSM8660_MPM_REG_POLARITY + (pin / 32) * 4, polarity);

	raw_spin_unlock_irqrestore(&wakeup->lock, flags);

	return irq_chip_set_type_parent(d, type);
}

static struct irq_chip msm8660_wakeup_chip = {
	.name			= "msm8660-wakeup",
	.irq_mask		= msm8660_wakeup_mask_irq,
	.irq_unmask		= msm8660_wakeup_unmask_irq,
	.irq_set_type		= msm8660_wakeup_set_type,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_SKIP_SET_WAKE,
};

static int msm8660_wakeup_domain_alloc(struct irq_domain *domain,
					unsigned int virq,
					unsigned int nr_irqs, void *data)
{
	struct msm8660_wakeup *wakeup = domain->host_data;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	int hwirq, ret;

	if (fwspec->param_count != 2)
		return -EINVAL;

	hwirq = fwspec->param[0];

	/* Set up parent IRQ */
	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 3;
	parent_fwspec.param[0] = 0;	/* GIC_SPI */
	parent_fwspec.param[1] = hwirq;
	parent_fwspec.param[2] = fwspec->param[1];

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &parent_fwspec);
	if (ret)
		return ret;

	irq_domain_set_hwirq_and_chip(domain, virq, hwirq,
				       &msm8660_wakeup_chip, wakeup);

	return 0;
}

static const struct irq_domain_ops msm8660_wakeup_domain_ops = {
	.alloc	= msm8660_wakeup_domain_alloc,
	.free	= irq_domain_free_irqs_common,
	.translate = irq_domain_translate_twocell,
};

static int msm8660_wakeup_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct msm8660_wakeup *wakeup;
	struct irq_domain *parent_domain;
	struct device_node *parent_np;
	int ret, i;
	u32 offset;

	dev_info(dev, "MSM8660 wakeup interrupt controller probe\n");

	wakeup = devm_kzalloc(dev, sizeof(*wakeup), GFP_KERNEL);
	if (!wakeup)
		return -ENOMEM;

	wakeup->dev = dev;
	raw_spin_lock_init(&wakeup->lock);

	/* Get RPM syscon regmap */
	wakeup->regmap = syscon_regmap_lookup_by_phandle(np, "qcom,rpm-syscon");
	if (IS_ERR(wakeup->regmap)) {
		dev_err(dev, "failed to get RPM syscon: %ld\n",
			PTR_ERR(wakeup->regmap));
		return PTR_ERR(wakeup->regmap);
	}

	ret = of_property_read_u32(np, "qcom,mpm-offset", &offset);
	if (ret) {
		dev_err(dev, "failed to read qcom,mpm-offset: %d\n", ret);
		return ret;
	}
	wakeup->regmap_offset = offset;

	dev_info(dev, "vMPM registers at RPM offset 0x%x\n", offset);

	/* Parse pin map */
	ret = of_property_count_u32_elems(np, "qcom,mpm-pin-map");
	if (ret < 0 || ret % 2) {
		dev_err(dev, "invalid qcom,mpm-pin-map\n");
		return -EINVAL;
	}

	wakeup->pin_map_count = ret / 2;
	wakeup->pin_map = devm_kcalloc(dev, wakeup->pin_map_count,
				       sizeof(*wakeup->pin_map), GFP_KERNEL);
	if (!wakeup->pin_map)
		return -ENOMEM;

	for (i = 0; i < wakeup->pin_map_count; i++) {
		u32 pin, hwirq;

		of_property_read_u32_index(np, "qcom,mpm-pin-map",
					    i * 2, &pin);
		of_property_read_u32_index(np, "qcom,mpm-pin-map",
					    i * 2 + 1, &hwirq);

		wakeup->pin_map[i].pin = pin;
		wakeup->pin_map[i].hwirq = hwirq;

		dev_dbg(dev, "pin map: pin %d -> hwirq %d\n", pin, hwirq);
	}

	/* Get parent interrupt domain (GIC) */
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

	/* Create IRQ domain */
	wakeup->domain = irq_domain_add_hierarchy(parent_domain, 0,
						  MSM8660_MPM_PIN_COUNT,
						  np, &msm8660_wakeup_domain_ops,
						  wakeup);
	if (!wakeup->domain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	/* Get parent IRQ for wakeup notifications */
	wakeup->parent_irq = platform_get_irq(pdev, 0);
	if (wakeup->parent_irq < 0) {
		dev_err(dev, "failed to get parent IRQ: %d\n", wakeup->parent_irq);
		ret = wakeup->parent_irq;
		goto err_remove_domain;
	}

	ret = devm_request_irq(dev, wakeup->parent_irq, msm8660_wakeup_irq,
			       IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			       "msm8660-wakeup", wakeup);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d: %d\n",
			wakeup->parent_irq, ret);
		goto err_remove_domain;
	}

	/* Get mailbox channel for RPM notifications */
	wakeup->mbox_client.dev = dev;
	wakeup->mbox_client.knows_txdone = true;
	wakeup->mbox_chan = mbox_request_channel(&wakeup->mbox_client, 0);
	if (IS_ERR(wakeup->mbox_chan)) {
		dev_warn(dev, "failed to get mailbox channel: %ld (continuing without)\n",
			 PTR_ERR(wakeup->mbox_chan));
		wakeup->mbox_chan = NULL;
	}

	platform_set_drvdata(pdev, wakeup);

	dev_info(dev, "MSM8660 wakeup controller initialized with %d pin mappings\n",
		 wakeup->pin_map_count);

	return 0;

err_remove_domain:
	irq_domain_remove(wakeup->domain);
	return ret;
}

static void msm8660_wakeup_remove(struct platform_device *pdev)
{
	struct msm8660_wakeup *wakeup = platform_get_drvdata(pdev);

	if (wakeup->mbox_chan)
		mbox_free_channel(wakeup->mbox_chan);

	irq_domain_remove(wakeup->domain);
}

static const struct of_device_id msm8660_wakeup_of_match[] = {
	{ .compatible = "qcom,msm8660-wakeup" },
	{}
};
MODULE_DEVICE_TABLE(of, msm8660_wakeup_of_match);

static struct platform_driver msm8660_wakeup_driver = {
	.probe		= msm8660_wakeup_probe,
	.remove		= msm8660_wakeup_remove,
	.driver		= {
		.name	= "msm8660-wakeup",
		.of_match_table = msm8660_wakeup_of_match,
	},
};

static int __init msm8660_wakeup_init(void)
{
	return platform_driver_register(&msm8660_wakeup_driver);
}
subsys_initcall(msm8660_wakeup_init);

static void __exit msm8660_wakeup_exit(void)
{
	platform_driver_unregister(&msm8660_wakeup_driver);
}
module_exit(msm8660_wakeup_exit);

MODULE_DESCRIPTION("MSM8660 Wakeup Interrupt Controller");
MODULE_LICENSE("GPL");
