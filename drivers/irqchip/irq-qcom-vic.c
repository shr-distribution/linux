// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM VIC interrupt controller driver
 *
 * Copyright (c) 2024, Linux community
 *
 * Based on legacy arch/arm/mach-msm/irq-vic.c
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * The MSM VIC is a custom Qualcomm interrupt controller used in
 * MSM7x30, QSD8x50, and similar Scorpion-based SoCs. It differs
 * from the standard ARM PL190/PL192 VIC in several ways:
 * - Supports 128 interrupts (4 banks of 32)
 * - Per-interrupt edge/level type configuration
 * - Per-interrupt polarity configuration
 * - Uses separate set/clear registers for enable
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/exception.h>

/* Register offsets - MSM VIC has 4 banks of 32 interrupts */
#define VIC_INT_SELECT(n)	(0x0000 + (n) * 4)	/* 1: FIQ, 0: IRQ */
#define VIC_INT_EN(n)		(0x0010 + (n) * 4)	/* Enable status */
#define VIC_INT_ENCLEAR(n)	(0x0020 + (n) * 4)	/* Write 1 to disable */
#define VIC_INT_ENSET(n)	(0x0030 + (n) * 4)	/* Write 1 to enable */
#define VIC_INT_TYPE(n)		(0x0040 + (n) * 4)	/* 1: EDGE, 0: LEVEL */
#define VIC_INT_POLARITY(n)	(0x0050 + (n) * 4)	/* 1: NEG, 0: POS */
#define VIC_NO_PEND_VAL		0x0060
#define VIC_NO_PEND_VAL_FIQ	0x0064			/* Scorpion only */
#define VIC_INT_MASTEREN	0x0068			/* 1: IRQ, 2: FIQ */
#define VIC_CONFIG		0x006C			/* 1: Use SC VIC */
#define VIC_IRQ_STATUS(n)	(0x0080 + (n) * 4)
#define VIC_FIQ_STATUS(n)	(0x0090 + (n) * 4)
#define VIC_RAW_STATUS(n)	(0x00A0 + (n) * 4)
#define VIC_INT_CLEAR(n)	(0x00B0 + (n) * 4)
#define VIC_SOFTINT(n)		(0x00C0 + (n) * 4)
#define VIC_IRQ_VEC_RD		0x00D0			/* Pending int # */
#define VIC_IRQ_VEC_PEND_RD	0x00D4			/* Pending vector addr */
#define VIC_IRQ_VEC_WR		0x00D8
#define VIC_FIQ_VEC_RD		0x00DC			/* Scorpion only */
#define VIC_FIQ_VEC_PEND_RD	0x00E0			/* Scorpion only */
#define VIC_FIQ_VEC_WR		0x00E4			/* Scorpion only */
#define VIC_IRQ_IN_SERVICE	0x00E8
#define VIC_IRQ_IN_STACK	0x00EC
#define VIC_FIQ_IN_SERVICE	0x00F0
#define VIC_FIQ_IN_STACK	0x00F4
#define VIC_TEST_BUS_SEL	0x00F8
#define VIC_IRQ_CTRL_CONFIG	0x00FC
#define VIC_VECTPRIORITY(n)	(0x0200 + (n) * 4)
#define VIC_VECTADDR(n)		(0x0400 + (n) * 4)

#define VIC_NUM_BANKS		4
#define VIC_IRQS_PER_BANK	32
#define VIC_NUM_IRQS		(VIC_NUM_BANKS * VIC_IRQS_PER_BANK)

/**
 * struct msm_vic - MSM VIC device structure
 * @base: Base address of VIC registers
 * @domain: IRQ domain for this VIC
 * @int_type: Shadow of interrupt type registers (for suspend/resume)
 * @int_polarity: Shadow of polarity registers (for suspend/resume)
 * @int_enable: Shadow of enable registers (for suspend/resume)
 */
struct msm_vic {
	void __iomem *base;
	struct irq_domain *domain;
	u32 int_type[VIC_NUM_BANKS];
	u32 int_polarity[VIC_NUM_BANKS];
	u32 int_enable[VIC_NUM_BANKS];
};

static struct msm_vic *msm_vic_data;

static inline u32 vic_bank(irq_hw_number_t hwirq)
{
	return hwirq >> 5;
}

static inline u32 vic_bit(irq_hw_number_t hwirq)
{
	return 1 << (hwirq & 31);
}

static void msm_vic_mask_irq(struct irq_data *d)
{
	struct msm_vic *vic = irq_data_get_irq_chip_data(d);
	u32 bank = vic_bank(d->hwirq);
	u32 mask = vic_bit(d->hwirq);

	pr_debug("MSM VIC: mask hwirq %lu (bank %u, mask 0x%x)\n",
		 d->hwirq, bank, mask);

	vic->int_enable[bank] &= ~mask;
	writel(mask, vic->base + VIC_INT_ENCLEAR(bank));
}

static void msm_vic_unmask_irq(struct irq_data *d)
{
	struct msm_vic *vic = irq_data_get_irq_chip_data(d);
	u32 bank = vic_bank(d->hwirq);
	u32 mask = vic_bit(d->hwirq);

	pr_debug("MSM VIC: unmask hwirq %lu (bank %u, mask 0x%x)\n",
		 d->hwirq, bank, mask);

	vic->int_enable[bank] |= mask;
	writel(mask, vic->base + VIC_INT_ENSET(bank));
}

static void msm_vic_ack_irq(struct irq_data *d)
{
	struct msm_vic *vic = irq_data_get_irq_chip_data(d);
	u32 bank = vic_bank(d->hwirq);
	u32 mask = vic_bit(d->hwirq);

	pr_debug("MSM VIC: ack hwirq %lu\n", d->hwirq);

	writel(mask, vic->base + VIC_INT_CLEAR(bank));
}

static void msm_vic_eoi_irq(struct irq_data *d)
{
	struct msm_vic *vic = irq_data_get_irq_chip_data(d);

	pr_debug("MSM VIC: eoi hwirq %lu\n", d->hwirq);

	/* Write any value to acknowledge interrupt processing complete */
	writel(0, vic->base + VIC_IRQ_VEC_WR);
}

static int msm_vic_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct msm_vic *vic = irq_data_get_irq_chip_data(d);
	u32 bank = vic_bank(d->hwirq);
	u32 mask = vic_bit(d->hwirq);
	u32 type, polarity;

	pr_debug("MSM VIC: set_type hwirq %lu, flow_type 0x%x\n",
		 d->hwirq, flow_type);

	polarity = vic->int_polarity[bank];
	type = vic->int_type[bank];

	/* Configure polarity */
	if (flow_type & (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW))
		polarity |= mask;
	else if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH))
		polarity &= ~mask;

	/* Configure edge vs level */
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		type |= mask;
		irq_set_handler_locked(d, handle_edge_irq);
	} else if (flow_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
		type &= ~mask;
		irq_set_handler_locked(d, handle_level_irq);
	}

	vic->int_type[bank] = type;
	vic->int_polarity[bank] = polarity;

	writel(type, vic->base + VIC_INT_TYPE(bank));
	writel(polarity, vic->base + VIC_INT_POLARITY(bank));

	return 0;
}

static struct irq_chip msm_vic_chip = {
	.name		= "MSM-VIC",
	.irq_mask	= msm_vic_mask_irq,
	.irq_unmask	= msm_vic_unmask_irq,
	.irq_ack	= msm_vic_ack_irq,
	.irq_eoi	= msm_vic_eoi_irq,
	.irq_set_type	= msm_vic_set_type,
	.flags		= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_MASK_ON_SUSPEND,
};

static int msm_vic_irq_domain_map(struct irq_domain *d, unsigned int irq,
				  irq_hw_number_t hwirq)
{
	struct msm_vic *vic = d->host_data;

	pr_debug("MSM VIC: domain_map irq %u -> hwirq %lu\n", irq, hwirq);

	irq_set_chip_and_handler(irq, &msm_vic_chip, handle_level_irq);
	irq_set_chip_data(irq, vic);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops msm_vic_irq_domain_ops = {
	.map = msm_vic_irq_domain_map,
	.xlate = irq_domain_xlate_onecell,
};

/*
 * Handle interrupts from the VIC.
 * We iterate through all banks checking for pending interrupts.
 */
static void __exception_irq_entry msm_vic_handle_irq(struct pt_regs *regs)
{
	struct msm_vic *vic = msm_vic_data;
	u32 stat, hwirq;
	int bank;

	do {
		for (bank = 0; bank < VIC_NUM_BANKS; bank++) {
			stat = readl_relaxed(vic->base + VIC_IRQ_STATUS(bank));
			while (stat) {
				hwirq = __ffs(stat);
				stat &= ~(1 << hwirq);
				hwirq += bank * VIC_IRQS_PER_BANK;
				generic_handle_domain_irq(vic->domain, hwirq);
			}
		}
		/*
		 * Check if any new interrupts arrived while we were
		 * processing. The VIC_IRQ_VEC_RD returns -1 when no
		 * interrupts are pending.
		 */
	} while (readl_relaxed(vic->base + VIC_IRQ_VEC_RD) != 0xFFFFFFFF);
}

static void msm_vic_init_hw(struct msm_vic *vic)
{
	int i;

	pr_info("MSM VIC: Initializing hardware\n");

	/*
	 * Follow legacy initialization sequence exactly:
	 * 1. Select level interrupts (TYPE = 0)
	 * 2. Select high-level/rising-edge polarity (POLARITY = 0)
	 * 3. Select IRQ not FIQ (SELECT = 0)
	 * 4. Disable all interrupts (ENCLEAR = all ones)
	 * 5. Don't use vectored mode (CONFIG = 0)
	 * 6. Enable interrupt controller (MASTEREN = 3)
	 */
	for (i = 0; i < VIC_NUM_BANKS; i++) {
		/* Select level interrupts */
		writel(0, vic->base + VIC_INT_TYPE(i));
		/* Select high-level polarity */
		writel(0, vic->base + VIC_INT_POLARITY(i));
		/* Select IRQ not FIQ */
		writel(0, vic->base + VIC_INT_SELECT(i));
		/* Disable all interrupts */
		writel(0xFFFFFFFF, vic->base + VIC_INT_ENCLEAR(i));

		pr_info("MSM VIC: Bank %d: TYPE=0x%08x POL=0x%08x SEL=0x%08x EN=0x%08x\n",
			i,
			readl(vic->base + VIC_INT_TYPE(i)),
			readl(vic->base + VIC_INT_POLARITY(i)),
			readl(vic->base + VIC_INT_SELECT(i)),
			readl(vic->base + VIC_INT_EN(i)));
	}

	/* Don't use vectored interrupts */
	writel(0, vic->base + VIC_CONFIG);

	/* Enable both IRQ and FIQ at master level */
	writel(3, vic->base + VIC_INT_MASTEREN);

	pr_info("MSM VIC: CONFIG=0x%08x MASTEREN=0x%08x\n",
		readl(vic->base + VIC_CONFIG),
		readl(vic->base + VIC_INT_MASTEREN));
}

static int __init msm_vic_of_init(struct device_node *node,
				  struct device_node *parent)
{
	struct msm_vic *vic;
	int ret;

	pr_info("MSM VIC: Starting initialization\n");

	vic = kzalloc(sizeof(*vic), GFP_KERNEL);
	if (!vic)
		return -ENOMEM;

	vic->base = of_iomap(node, 0);
	if (!vic->base) {
		pr_err("MSM VIC: Failed to map registers\n");
		ret = -ENOMEM;
		goto err_free;
	}

	pr_info("MSM VIC: Mapped base at %p\n", vic->base);

	/* Initialize hardware */
	msm_vic_init_hw(vic);

	/* Create IRQ domain */
	pr_info("MSM VIC: Creating IRQ domain\n");
	vic->domain = irq_domain_add_linear(node, VIC_NUM_IRQS,
					    &msm_vic_irq_domain_ops, vic);
	if (!vic->domain) {
		pr_err("MSM VIC: Failed to create IRQ domain\n");
		ret = -ENOMEM;
		goto err_unmap;
	}

	msm_vic_data = vic;

	pr_info("MSM VIC: Setting IRQ handler\n");
	set_handle_irq(msm_vic_handle_irq);

	pr_info("MSM VIC: Initialization complete with %d interrupts\n", VIC_NUM_IRQS);

	return 0;

err_unmap:
	iounmap(vic->base);
err_free:
	kfree(vic);
	return ret;
}

IRQCHIP_DECLARE(qcom_msm_vic, "qcom,msm-vic", msm_vic_of_init);

MODULE_DESCRIPTION("Qualcomm MSM VIC interrupt controller driver");
MODULE_LICENSE("GPL v2");
