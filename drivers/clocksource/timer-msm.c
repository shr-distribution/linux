// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM7x30 Timer driver
 *
 * Copyright (c) 2024, Linux community
 *
 * Based on:
 * - Legacy arch/arm/mach-msm/timer.c
 *   Copyright (C) 2007 Google, Inc.
 *   Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * - drivers/clocksource/timer-qcom.c
 *   Copyright (c) 2009-2012,2014, The Linux Foundation. All rights reserved.
 *
 * MSM7x30 Timer Hardware:
 * - Timer block at 0xC0100000
 * - GPT (General Purpose Timer) at offset 0x04 - 32.768 kHz
 * - DGT (Debug Timer) at offset 0x24 - 6.144 MHz (LPXO/4)
 * - DGT_CLK_CTL at offset 0x34
 * - Single core Scorpion, no per-CPU timers
 */

#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#include <asm/delay.h>

/* Timer register offsets */
#define TIMER_MATCH_VAL			0x0000
#define TIMER_COUNT_VAL			0x0004
#define TIMER_ENABLE			0x0008
#define TIMER_CLEAR			0x000C

/* Timer enable register bits */
#define TIMER_ENABLE_EN			BIT(0)
#define TIMER_ENABLE_CLR_ON_MATCH_EN	BIT(1)

/* DGT clock control - at offset 0x10 from DGT base (0x34 from timer base) */
#define DGT_CLK_CTL			0x0010
#define DGT_CLK_CTL_DIV_1		0
#define DGT_CLK_CTL_DIV_2		1
#define DGT_CLK_CTL_DIV_3		2
#define DGT_CLK_CTL_DIV_4		3

/* Timer base offsets for MSM7x30 */
#define GPT_OFFSET			0x04
#define DGT_OFFSET			0x24

/* Clock frequencies */
#define GPT_HZ				32768
#define DGT_HZ				6144000	/* LPXO (24.576 MHz) / 4 */

static void __iomem *event_base;	/* GPT for clock events */
static void __iomem *source_base;	/* DGT for clocksource */

static irqreturn_t msm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	/* Stop the timer if in oneshot mode */
	if (clockevent_state_oneshot(evt)) {
		u32 ctrl = readl_relaxed(event_base + TIMER_ENABLE);
		ctrl &= ~TIMER_ENABLE_EN;
		writel_relaxed(ctrl, event_base + TIMER_ENABLE);
	}

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static int msm_timer_set_next_event(unsigned long cycles,
				    struct clock_event_device *evt)
{
	u32 ctrl = readl_relaxed(event_base + TIMER_ENABLE);

	/* Disable timer */
	ctrl &= ~TIMER_ENABLE_EN;
	writel_relaxed(ctrl, event_base + TIMER_ENABLE);

	/* Clear and set new match value */
	writel_relaxed(1, event_base + TIMER_CLEAR);
	writel_relaxed(cycles, event_base + TIMER_MATCH_VAL);

	/* Enable timer */
	writel_relaxed(ctrl | TIMER_ENABLE_EN, event_base + TIMER_ENABLE);

	return 0;
}

static int msm_timer_shutdown(struct clock_event_device *evt)
{
	u32 ctrl = readl_relaxed(event_base + TIMER_ENABLE);
	ctrl &= ~(TIMER_ENABLE_EN | TIMER_ENABLE_CLR_ON_MATCH_EN);
	writel_relaxed(ctrl, event_base + TIMER_ENABLE);
	return 0;
}

static struct clock_event_device msm_clockevent = {
	.name			= "gp_timer",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 200,
	.set_state_shutdown	= msm_timer_shutdown,
	.set_state_oneshot	= msm_timer_shutdown,
	.tick_resume		= msm_timer_shutdown,
	.set_next_event		= msm_timer_set_next_event,
};

static u64 notrace msm_read_timer_count(struct clocksource *cs)
{
	return readl_relaxed(source_base + TIMER_COUNT_VAL);
}

static struct clocksource msm_clocksource = {
	.name		= "dg_timer",
	.rating		= 300,
	.read		= msm_read_timer_count,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static u64 notrace msm_sched_clock_read(void)
{
	return msm_read_timer_count(&msm_clocksource);
}

static unsigned long msm_read_current_timer(void)
{
	return msm_read_timer_count(&msm_clocksource);
}

static struct delay_timer msm_delay_timer = {
	.read_current_timer = msm_read_current_timer,
};

static int __init msm_timer_init(struct device_node *np)
{
	void __iomem *base;
	int irq, ret;
	u32 freq;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("MSM Timer: Failed to map registers\n");
		return -ENXIO;
	}

	/* GPT for clock events */
	event_base = base + GPT_OFFSET;

	/* DGT for clocksource */
	source_base = base + DGT_OFFSET;

	/* Get GPT interrupt (index 1 in DT: timer 0 is DGT, timer 1 is GPT) */
	irq = irq_of_parse_and_map(np, 1);
	if (irq <= 0) {
		pr_err("MSM Timer: Failed to get GPT IRQ\n");
		ret = -EINVAL;
		goto err_unmap;
	}

	/* Read clock frequency from DT or use default */
	if (of_property_read_u32(np, "clock-frequency", &freq))
		freq = DGT_HZ * 4;  /* Default: LPXO = 24.576 MHz */

	/* Configure DGT clock divider to /4 */
	writel_relaxed(DGT_CLK_CTL_DIV_4, source_base + DGT_CLK_CTL);
	freq /= 4;

	/* Initialize DGT for clocksource */
	writel_relaxed(TIMER_ENABLE_EN, source_base + TIMER_ENABLE);

	/* Register clocksource */
	ret = clocksource_register_hz(&msm_clocksource, freq);
	if (ret) {
		pr_err("MSM Timer: Failed to register clocksource\n");
		goto err_unmap;
	}

	/* Register sched_clock */
	sched_clock_register(msm_sched_clock_read, 32, freq);

	/* Register delay timer */
	msm_delay_timer.freq = freq;
	register_current_timer_delay(&msm_delay_timer);

	/* Initialize GPT for clock events */
	writel_relaxed(0, event_base + TIMER_ENABLE);
	writel_relaxed(1, event_base + TIMER_CLEAR);
	writel_relaxed(0, event_base + TIMER_MATCH_VAL);

	/* Setup clock event IRQ */
	ret = request_irq(irq, msm_timer_interrupt,
			  IRQF_TIMER | IRQF_NOBALANCING | IRQF_TRIGGER_RISING,
			  "gp_timer", &msm_clockevent);
	if (ret) {
		pr_err("MSM Timer: Failed to request IRQ\n");
		goto err_clocksource;
	}

	/* Register clock event device */
	msm_clockevent.irq = irq;
	msm_clockevent.cpumask = cpumask_of(0);
	clockevents_config_and_register(&msm_clockevent, GPT_HZ, 4, 0xffffffff);

	pr_info("MSM Timer: DGT clocksource @ %u Hz, GPT clockevent @ %u Hz\n",
		freq, GPT_HZ);

	return 0;

err_clocksource:
	clocksource_unregister(&msm_clocksource);
err_unmap:
	iounmap(base);
	return ret;
}

TIMER_OF_DECLARE(msm_timer, "qcom,msm-timer", msm_timer_init);

MODULE_DESCRIPTION("Qualcomm MSM7x30 Timer driver");
MODULE_LICENSE("GPL v2");
