// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024 Herrie <herrie@herrie.org>
 *
 * Z180 2D Graphics Engine driver for Qualcomm MSM8660/APQ8060
 *
 * This is a minimal driver for the Z180 2D graphics accelerator.
 * The Z180 is a vector graphics processor that supports 2D operations
 * like blitting, scaling, and compositing.
 *
 * Based on legacy kgsl_g12 driver from webOS kernel.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "z180_gpu.h"

struct z180_device {
	struct device *dev;
	void __iomem *mmio;
	struct clk *core_clk;
	struct clk *iface_clk;
	int irq;

	/* Hardware state */
	spinlock_t cmdwin_lock;
	int timestamp;
	int current_timestamp;
};

static inline void z180_write(struct z180_device *z180, u32 reg, u32 data)
{
	writel(data, z180->mmio + (reg << 2));
}

static inline u32 z180_read(struct z180_device *z180, u32 reg)
{
	return readl(z180->mmio + (reg << 2));
}

/*
 * The MH registers must be accessed through a 2-step write/read process
 * via the command window. These registers may be accessed from interrupt
 * context, so we use a spinlock.
 */
static void z180_regwrite_mh(struct z180_device *z180, u32 offsetwords, u32 value)
{
	unsigned long flags;
	u32 cmdwinaddr;

	cmdwinaddr = ((Z180_CMDWINDOW_MMU << Z180_CMDWINDOW_TARGET_SHIFT) &
		      Z180_CMDWINDOW_TARGET_MASK);
	cmdwinaddr |= ((offsetwords << Z180_CMDWINDOW_ADDR_SHIFT) &
		       Z180_CMDWINDOW_ADDR_MASK);

	spin_lock_irqsave(&z180->cmdwin_lock, flags);
	z180_write(z180, Z180_VGC_MMUCOMMANDSTREAM >> 2, cmdwinaddr);
	z180_write(z180, Z180_VGC_MMUCOMMANDSTREAM >> 2, value);
	spin_unlock_irqrestore(&z180->cmdwin_lock, flags);
}

static void z180_regread_mh(struct z180_device *z180, u32 offsetwords, u32 *value)
{
	unsigned long flags;

	spin_lock_irqsave(&z180->cmdwin_lock, flags);
	z180_write(z180, Z180_VGC_MH_READ_ADDR >> 2, offsetwords);
	*value = z180_read(z180, Z180_VGC_MH_DATA_ADDR >> 2);
	spin_unlock_irqrestore(&z180->cmdwin_lock, flags);
}

static irqreturn_t z180_irq_handler(int irq, void *data)
{
	struct z180_device *z180 = data;
	u32 status;
	int count;

	status = z180_read(z180, Z180_VGC_IRQSTATUS >> 2);

	if (!(status & (Z180_IRQ_MH_MASK | Z180_IRQ_G2D_MASK | Z180_IRQ_FIFO_MASK)))
		return IRQ_NONE;

	/* Acknowledge interrupts */
	z180_write(z180, Z180_VGC_IRQSTATUS >> 2,
		   status & (Z180_IRQ_MH_MASK | Z180_IRQ_G2D_MASK | Z180_IRQ_FIFO_MASK));

	if (status & Z180_IRQ_FIFO_MASK)
		dev_err(z180->dev, "Z180 FIFO interrupt\n");

	if (status & Z180_IRQ_MH_MASK) {
		u32 mh_status;

		z180_regread_mh(z180, Z180_MH_INTERRUPT_STATUS, &mh_status);

		if (mh_status & Z180_MH_INT_AXI_READ_ERROR)
			dev_err(z180->dev, "Z180 AXI read error\n");
		if (mh_status & Z180_MH_INT_AXI_WRITE_ERROR)
			dev_err(z180->dev, "Z180 AXI write error\n");
		if (mh_status & Z180_MH_INT_MMU_PAGE_FAULT)
			dev_err(z180->dev, "Z180 MMU page fault\n");

		z180_regwrite_mh(z180, Z180_MH_INTERRUPT_CLEAR, mh_status);
	}

	if (status & Z180_IRQ_G2D_MASK) {
		/* Read active count register */
		count = z180_read(z180, Z180_VGC_IRQ_ACTIVE_CNT >> 2);
		count = (count >> 8) & 0xff;
		z180->timestamp += count;

		/* Wake up any waiters */
	}

	return IRQ_HANDLED;
}

static int z180_hw_init(struct z180_device *z180)
{
	/* Set up MH arbiter */
	z180_regwrite_mh(z180, Z180_MH_ARBITER_CONFIG, Z180_CFG_MHARB);
	z180_regwrite_mh(z180, Z180_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030F27);
	z180_regwrite_mh(z180, Z180_MH_CLNT_INTF_CTRL_CONFIG2, 0x004B274F);

	/* Enable interrupts */
	z180_write(z180, Z180_VGC_IRQENABLE >> 2, 0x3);

	/* Initialize timestamps */
	z180->timestamp = 0;
	z180->current_timestamp = 0;

	dev_info(z180->dev, "Z180 hardware initialized\n");

	return 0;
}

static int z180_runtime_suspend(struct device *dev)
{
	struct z180_device *z180 = dev_get_drvdata(dev);

	/* Disable interrupts */
	z180_write(z180, Z180_VGC_IRQENABLE >> 2, 0);

	clk_disable_unprepare(z180->core_clk);
	clk_disable_unprepare(z180->iface_clk);

	return 0;
}

static int z180_runtime_resume(struct device *dev)
{
	struct z180_device *z180 = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(z180->iface_clk);
	if (ret) {
		dev_err(dev, "Failed to enable iface clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(z180->core_clk);
	if (ret) {
		dev_err(dev, "Failed to enable core clock: %d\n", ret);
		clk_disable_unprepare(z180->iface_clk);
		return ret;
	}

	return z180_hw_init(z180);
}

static int z180_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct z180_device *z180;
	int ret;

	z180 = devm_kzalloc(dev, sizeof(*z180), GFP_KERNEL);
	if (!z180)
		return -ENOMEM;

	z180->dev = dev;
	spin_lock_init(&z180->cmdwin_lock);
	platform_set_drvdata(pdev, z180);

	/* Map registers */
	z180->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(z180->mmio))
		return PTR_ERR(z180->mmio);

	/* Get clocks */
	z180->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(z180->core_clk)) {
		dev_err(dev, "Failed to get core clock\n");
		return PTR_ERR(z180->core_clk);
	}

	z180->iface_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(z180->iface_clk)) {
		dev_err(dev, "Failed to get iface clock\n");
		return PTR_ERR(z180->iface_clk);
	}

	/* Get IRQ */
	z180->irq = platform_get_irq(pdev, 0);
	if (z180->irq < 0)
		return z180->irq;

	ret = devm_request_irq(dev, z180->irq, z180_irq_handler,
			       IRQF_TRIGGER_HIGH, "z180", z180);
	if (ret) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	/* Enable runtime PM */
	pm_runtime_enable(dev);

	/* Initialize hardware */
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err(dev, "Failed to resume device: %d\n", ret);
		goto err_pm_disable;
	}

	dev_info(dev, "Z180 2D GPU probed successfully\n");

	pm_runtime_put(dev);

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static void z180_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops z180_pm_ops = {
	SET_RUNTIME_PM_OPS(z180_runtime_suspend, z180_runtime_resume, NULL)
};

static const struct of_device_id z180_of_match[] = {
	{ .compatible = "qcom,msm8660-z180" },
	{ }
};
MODULE_DEVICE_TABLE(of, z180_of_match);

static struct platform_driver z180_driver = {
	.probe = z180_probe,
	.remove = z180_remove,
	.driver = {
		.name = "qcom-z180",
		.pm = &z180_pm_ops,
		.of_match_table = z180_of_match,
	},
};

module_platform_driver(z180_driver);

MODULE_DESCRIPTION("Qualcomm Z180 2D Graphics Engine Driver");
MODULE_LICENSE("GPL");
