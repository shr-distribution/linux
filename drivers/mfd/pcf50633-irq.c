/* NXP PCF50633 Power Management Unit (PMU) driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * 		Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/mfd/pcf50633/core.h>

/* Two MBCS registers used during cold start */
#define PCF50633_REG_MBCS1		0x4b
#define PCF50633_REG_MBCS2		0x4c
#define PCF50633_MBCS1_USBPRES 		0x01
#define PCF50633_MBCS1_ADAPTPRES	0x01

static void pcf50633_irq_lock(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);

	mutex_lock(&pcf->irq_lock);
}

static void pcf50633_irq_sync_unlock(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pcf->mask_regs); ++i) {
		if (pcf->mask_regs[i] == pcf->mask_regs_cur[i])
			continue;

		pcf->mask_regs[i] = pcf->mask_regs_cur[i];
		pcf50633_reg_write(pcf, PCF50633_REG_INT1M + i,
				pcf->mask_regs[i]);
	}

	mutex_unlock(&pcf->irq_lock);
}

static void pcf50633_irq_mask(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);
	int irq = data->irq;
	u8 bit;
	int idx;

	idx = irq >> 3;
	bit = 1 << (irq & 0x07);

	pcf->mask_regs[idx] |= bit;
}

static void pcf50633_irq_unmask(struct irq_data *data)
{
	struct pcf50633 *pcf = irq_data_get_irq_chip_data(data);
	int irq = data->irq;
	u8 bit;
	int idx;

	idx = irq >> 3;
	bit = 1 << (irq & 0x07);

	pcf->mask_regs[idx] &= ~bit;
}

static struct irq_chip pcf50633_irq_chip = {
	.name = "pcf50633-irq",
	.irq_mask = pcf50633_irq_mask,
	.irq_unmask = pcf50633_irq_unmask,
	.irq_bus_lock = pcf50633_irq_lock,
	.irq_bus_sync_unlock = pcf50633_irq_sync_unlock,
};

/* Maximum amount of time ONKEY is held before emergency action is taken */
#define PCF50633_ONKEY1S_TIMEOUT 8

static irqreturn_t pcf50633_irq(int irq, void *data)
{
	struct pcf50633 *pcf = data;
	int ret, i, j;
	u8 pcf_int[5], chgstat;

	/* Read the 5 INT regs in one transaction */
	ret = pcf50633_read_block(pcf, PCF50633_REG_INT1,
						ARRAY_SIZE(pcf_int), pcf_int);
	if (ret != ARRAY_SIZE(pcf_int)) {
		dev_err(pcf->dev, "Error reading INT registers\n");

		/*
		 * If this doesn't ACK the interrupt to the chip, we'll be
		 * called once again as we're level triggered.
		 */
		goto out;
	}

	/* defeat 8s death from lowsys on A5 */
	pcf50633_reg_write(pcf, PCF50633_REG_OOCSHDWN,  0x04);

	/* We immediately read the usb and adapter status. We thus make sure
	 * only of USBINS/USBREM IRQ handlers are called */
	if (pcf_int[0] & (PCF50633_INT1_USBINS | PCF50633_INT1_USBREM)) {
		chgstat = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
		if (chgstat & (0x3 << 4))
			pcf_int[0] &= ~PCF50633_INT1_USBREM;
		else
			pcf_int[0] &= ~PCF50633_INT1_USBINS;
	}

	/* Make sure only one of ADPINS or ADPREM is set */
	if (pcf_int[0] & (PCF50633_INT1_ADPINS | PCF50633_INT1_ADPREM)) {
		chgstat = pcf50633_reg_read(pcf, PCF50633_REG_MBCS2);
		if (chgstat & (0x3 << 4))
			pcf_int[0] &= ~PCF50633_INT1_ADPREM;
		else
			pcf_int[0] &= ~PCF50633_INT1_ADPINS;
	}

	dev_dbg(pcf->dev, "INT1=0x%02x INT2=0x%02x INT3=0x%02x "
			"INT4=0x%02x INT5=0x%02x\n", pcf_int[0],
			pcf_int[1], pcf_int[2], pcf_int[3], pcf_int[4]);

	/* Some revisions of the chip don't have a 8s standby mode on
	 * ONKEY1S press. We try to manually do it in such cases. */
	if ((pcf_int[0] & PCF50633_INT1_SECOND) && pcf->onkey1s_held) {
		dev_info(pcf->dev, "ONKEY1S held for %d secs\n",
							pcf->onkey1s_held);
		if (pcf->onkey1s_held++ == PCF50633_ONKEY1S_TIMEOUT)
			if (pcf->pdata->force_shutdown)
				pcf->pdata->force_shutdown(pcf);
	}

	if (pcf_int[2] & PCF50633_INT3_ONKEY1S) {
		dev_info(pcf->dev, "ONKEY1S held\n");
		pcf->onkey1s_held = 1 ;

		/* Unmask IRQ_SECOND */
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_INT1M,
						PCF50633_INT1_SECOND);

		/* Unmask IRQ_ONKEYR */
		pcf50633_reg_clear_bits(pcf, PCF50633_REG_INT2M,
						PCF50633_INT2_ONKEYR);
	}

	if ((pcf_int[1] & PCF50633_INT2_ONKEYR) && pcf->onkey1s_held) {
		pcf->onkey1s_held = 0;

		/* Mask SECOND and ONKEYR interrupts */
		if (pcf->mask_regs[0] & PCF50633_INT1_SECOND)
			pcf50633_reg_set_bit_mask(pcf,
					PCF50633_REG_INT1M,
					PCF50633_INT1_SECOND,
					PCF50633_INT1_SECOND);

		if (pcf->mask_regs[1] & PCF50633_INT2_ONKEYR)
			pcf50633_reg_set_bit_mask(pcf,
					PCF50633_REG_INT2M,
					PCF50633_INT2_ONKEYR,
					PCF50633_INT2_ONKEYR);
	}

	/* Have we just resumed ? */
	if (pcf->is_suspended) {
		pcf->is_suspended = 0;

		/* Set the resume reason filtering out non resumers */
		for (i = 0; i < ARRAY_SIZE(pcf_int); i++)
			pcf->resume_reason[i] = pcf_int[i] &
						pcf->pdata->resumers[i];

		/* Make sure we don't pass on any ONKEY events to
		 * userspace now */
		pcf_int[1] &= ~(PCF50633_INT2_ONKEYR | PCF50633_INT2_ONKEYF);
	}

	irq = pcf->irq_base;
	for (i = 0; i < ARRAY_SIZE(pcf_int); i++) {
		/* Unset masked interrupts */
		pcf_int[i] &= ~pcf->mask_regs[i];

		for (j = 0; j < 8 ; j++) {
			if (pcf_int[i] & (1 << j))
				handle_nested_irq(irq);
			++irq;
		}
	}

out:
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

int pcf50633_irq_suspend(struct pcf50633 *pcf)
{
	int ret;
	int i;
	u8 res[5];


	/* Make sure our interrupt handlers are not called
	 * henceforth */
	disable_irq(pcf->irq);

	/* Write wakeup irq masks */
	for (i = 0; i < ARRAY_SIZE(res); i++)
		res[i] = ~pcf->pdata->resumers[i];

	ret = pcf50633_write_block(pcf, PCF50633_REG_INT1M,
					ARRAY_SIZE(res), &res[0]);
	if (ret < 0) {
		dev_err(pcf->dev, "error writing wakeup irq masks\n");
		goto out;
	}

	pcf->is_suspended = 1;

out:
	return ret;
}

int pcf50633_irq_resume(struct pcf50633 *pcf)
{
	int ret;

	/* Write the saved mask registers */
	ret = pcf50633_write_block(pcf, PCF50633_REG_INT1M,
				ARRAY_SIZE(pcf->mask_regs),
					pcf->mask_regs);
	if (ret < 0)
		dev_err(pcf->dev, "Error restoring saved suspend masks\n");

	enable_irq(pcf->irq);

	return ret;
}

#endif

int pcf50633_irq_init(struct pcf50633 *pcf, int irq)
{
	int irq_base;
	int ret;
	int i;

	irq_base = irq_alloc_descs(-1, 0, PCF50633_NUM_IRQ, 0);
	if (irq_base < 0) {
		dev_err(pcf->dev, "Failed to allocate irq descs: %d\n", irq_base);
		return irq_base;
	}

	mutex_init(&pcf->irq_lock);
	pcf->irq = irq;
	pcf->irq_base = irq_base;

	/* Mask all irqs */
	for (i = 0; i < 5; ++i)
		pcf->mask_regs[i] = 0xff;

	ret = pcf50633_write_block(pcf, PCF50633_REG_INT1M,
					ARRAY_SIZE(pcf->mask_regs), pcf->mask_regs);
	if (ret < 0)
		goto err_irq_free_descs;


	for (i = irq_base; i < irq_base + PCF50633_NUM_IRQ; ++i) {
		irq_set_chip_data(i, pcf);
		irq_set_nested_thread(i, 1);
		irq_set_chip_and_handler(i, &pcf50633_irq_chip, handle_simple_irq);
		irq_modify_status(i, IRQ_NOREQUEST, IRQ_NOPROBE);
	}

	ret = request_threaded_irq(irq, NULL, pcf50633_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"pcf50633", pcf);

	if (ret) {
		dev_err(pcf->dev, "Failed to request IRQ %d\n", ret);
		goto err_irq_free_descs;
	}

	if (enable_irq_wake(irq) < 0)
		dev_err(pcf->dev, "IRQ %u cannot be enabled as wake-up source"
			"in this hardware revision", irq);


	return 0;

err_irq_free_descs:
	irq_free_descs(pcf->irq_base, PCF50633_NUM_IRQ);

	return ret;
}

void pcf50633_irq_free(struct pcf50633 *pcf)
{
	free_irq(pcf->irq, pcf);
	irq_free_descs(pcf->irq_base, PCF50633_NUM_IRQ);
}
