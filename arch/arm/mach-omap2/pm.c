/*
 * linux/arch/arm/mach-omap2/pm.c
 *
 * OMAP Power Management Common Routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/device.h>

#include <mach/cpu.h>
#include <asm/mach/time.h>
#include <asm/atomic.h>

#include <mach/pm.h>
#include <mach/powerdomain.h>
#include <mach/omapdev.h>
#include <mach/resource.h>
#include <mach/omap-pm.h>

#include "prm-regbits-34xx.h"
#include "pm.h"
#include "smartreflex.h"
#include "resource34xx_mutex.h"
#include "omap3-opp.h"

struct omap_opp omap3_mpu_rate_table[] = {
	{0, 0, 0},
	{0, 1, 0x1E},
	/*underclocking*/
	{S125M, 2, 0x1E},
	/*default*/
	{S250M, 3, 0x26},
	{S500M, 4, 0x30},
	{S550M, 5, 0x36},
	{S600M, 6, 0x3C},
	/*overclocking*/
	{S700M, 7, 0x3C},
	{S750M, 8, 0x3C},
	{S805M, 9, 0x3C},
	{S850M, 10, 0x3C},
	{S900M, 11, 0x3C},
	{S950M, 12, 0x3C},
	{S1000M, 13, 0x3C},
	{S1100M, 14, 0x48},
	{S1150M, 15, 0x48},
};

struct omap_opp omap3_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{0, VDD2_OPP1, 0x1E},
	/*OPP2*/
	{S83M, VDD2_OPP2, 0x24},
	/*OPP3*/
	{S166M, VDD2_OPP3, 0x2C},
};

struct omap_opp omap3_dsp_rate_table[] = {
	{0, 0, 0},
	/*underclocking*/
	{S90M,  1, 0x1E},
	/*default*/
	{S90M,  2, 0x1E},
	{S180M, 3, 0x26},
	{S360M, 4, 0x30},
	{S400M, 5, 0x36},
	{S430M, 6, 0x3C},
	/*overclocking*/
	{S430M, 7, 0x3C},
	{S430M, 8, 0x3C},
	{S430M, 9, 0x3C},/*800MHz*/
	{S500M, 10, 0x3C},
	{S500M, 11, 0x3C},
	{S500M, 12, 0x3C},
	{S500M, 13, 0x3C},
	{S520M, 14, 0x48},
	{S520M, 15, 0x48},
};

unsigned short enable_dyn_sleep;
unsigned short clocks_off_while_idle;
unsigned short enable_off_mode;
EXPORT_SYMBOL(enable_off_mode);
unsigned short voltage_off_while_idle;
atomic_t sleep_block = ATOMIC_INIT(0);
static int vdd1_locked;
static int vdd2_locked;

static ssize_t idle_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t idle_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);

static struct kobj_attribute sleep_while_idle_attr =
	__ATTR(sleep_while_idle, 0644, idle_show, idle_store);

static struct kobj_attribute clocks_off_while_idle_attr =
	__ATTR(clocks_off_while_idle, 0644, idle_show, idle_store);

static struct kobj_attribute enable_off_mode_attr =
	__ATTR(enable_off_mode, 0644, idle_show, idle_store);

static struct kobj_attribute voltage_off_while_idle_attr =
	__ATTR(voltage_off_while_idle, 0644, idle_show, idle_store);

#ifdef CONFIG_OMAP_PM_SRF
static ssize_t vdd_opp_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t vdd_opp_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);
static struct kobj_attribute vdd1_opp_attr =
	__ATTR(vdd1_opp, 0644, vdd_opp_show, vdd_opp_store);

static struct kobj_attribute vdd2_opp_attr =
	__ATTR(vdd2_opp, 0644, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd1_lock_attr =
	__ATTR(vdd1_lock, 0644, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd2_lock_attr =
	__ATTR(vdd2_lock, 0644, vdd_opp_show, vdd_opp_store);

#endif

static ssize_t omap_vdd1_opps_vsel_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int i;
	char *b=buf;
	for(i=1;i<=MAX_VDD1_OPP;i++)
		b+=sprintf(b, "%hu ", mpu_opps[i].vsel);
	b+=sprintf(b, "\n");
	return b-buf;
}

static ssize_t omap_vdd1_opps_vsel_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	u16 value[16];
	int i;

	if (sscanf(buf, "%hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu",
				&value[0], &value[1], &value[2], &value[3],
				&value[4], &value[5], &value[6], &value[7],
				&value[8], &value[9], &value[10], &value[11],
				&value[12], &value[13], &value[14], &value[15]
				) != MAX_VDD1_OPP) {
		printk(KERN_ERR "vdd1_opps_vsel: Invalid value\n");
		return -EINVAL;
	}

	mutex_lock(&dvfs_mutex);

	for(i=1;i<=MAX_VDD1_OPP;i++) {
		if(value[i-1]<0x49) {
			mpu_opps[i].vsel = value[i-1];
		}
	}

	mutex_unlock(&dvfs_mutex);

	return n;
}

static struct kobj_attribute vdd1_opps_vsel = {
	.attr = {
	.name = __stringify(vdd1_opps_vsel),
	.mode = 0644,
	},
	.show = omap_vdd1_opps_vsel_show,
	.store = omap_vdd1_opps_vsel_store,
};

static ssize_t omap_dsp_opps_rate_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int i;
	char *b=buf;
	for(i=1;i<=MAX_VDD1_OPP;i++)
		b+=sprintf(b, "%u ", (u16) (dsp_opps[i].rate/1000000));
	b+=sprintf(b, "\n");
	return b-buf;
}

static ssize_t omap_dsp_opps_rate_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	u16 value[16];
	int i;

	if (sscanf(buf, "%hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu %hu",
				&value[0], &value[1], &value[2], &value[3],
				&value[4], &value[5], &value[6], &value[7],
				&value[8], &value[9], &value[10], &value[11],
				&value[12], &value[13], &value[14], &value[15]
				) != MAX_VDD1_OPP) {
		printk(KERN_ERR "dsp_opps_rate: Invalid value\n");
		return -EINVAL;
	}

	mutex_lock(&dvfs_mutex);

	for(i=1;i<=MAX_VDD1_OPP;i++) {
		if(value[i-1]<=600) {
			dsp_opps[i].rate = 1000000 * (u32)value[i-1];
		}
	}

	mutex_unlock(&dvfs_mutex);

	return n;
}

static struct kobj_attribute dsp_opps_rate = {
	.attr = {
	.name = __stringify(dsp_opps_rate),
	.mode = 0644,
	},
	.show = omap_dsp_opps_rate_show,
	.store = omap_dsp_opps_rate_store,
};



static ssize_t idle_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &sleep_while_idle_attr)
		return sprintf(buf, "%hu\n", enable_dyn_sleep);
	else if (attr == &clocks_off_while_idle_attr)
		return sprintf(buf, "%hu\n", clocks_off_while_idle);
	else if (attr == &enable_off_mode_attr)
		return sprintf(buf, "%hu\n", enable_off_mode);
	else if (attr == &voltage_off_while_idle_attr)
		return sprintf(buf, "%hu\n", voltage_off_while_idle);
	else
		return -EINVAL;
}

static ssize_t idle_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "idle_store: Invalid value\n");
		return -EINVAL;
	}

	if (attr == &sleep_while_idle_attr) {
		enable_dyn_sleep = value;
	} else if (attr == &clocks_off_while_idle_attr) {
		clocks_off_while_idle = value;
	} else if (attr == &enable_off_mode_attr) {
		enable_off_mode = value;
		omap3_pm_off_mode_enable(enable_off_mode);
	} else if (attr == &voltage_off_while_idle_attr) {
		voltage_off_while_idle = value;
		if (voltage_off_while_idle)
			prm_set_mod_reg_bits(OMAP3430_SEL_OFF, OMAP3430_GR_MOD,
					OMAP3_PRM_VOLTCTRL_OFFSET);
		else
			prm_clear_mod_reg_bits(OMAP3430_SEL_OFF,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VOLTCTRL_OFFSET);
	} else {
		return -EINVAL;
	}
	return n;
}

#ifdef CONFIG_OMAP_PM_SRF
static ssize_t vdd_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &vdd1_opp_attr)
		return sprintf(buf, "%hu\n", resource_get_level("vdd1_opp"));
	else if (attr == &vdd2_opp_attr)
		return sprintf(buf, "%hu\n", resource_get_level("vdd2_opp"));
	else if (attr == &vdd1_lock_attr)
		return sprintf(buf, "%hu\n", resource_get_opp_lock(PRCM_VDD1));
	else if (attr == &vdd2_lock_attr)
		return sprintf(buf, "%hu\n", resource_get_opp_lock(PRCM_VDD2));
	else
		return -EINVAL;
}

static ssize_t vdd_opp_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;
	int flags = 0;

	if (sscanf(buf, "%hu", &value) != 1)
		return -EINVAL;

	/* Check locks */
	if (attr == &vdd1_lock_attr) {
		flags = OPP_IGNORE_LOCK;
		attr = &vdd1_opp_attr;
		if (vdd1_locked && value == 0) {
			resource_unlock_opp(PRCM_VDD1);
			resource_refresh();
			vdd1_locked = 0;
			return n;
		}
		if (vdd1_locked == 0 && value != 0) {
			resource_lock_opp(PRCM_VDD1);
			vdd1_locked = 1;
		}
	} else if (attr == &vdd2_lock_attr) {
		flags = OPP_IGNORE_LOCK;
		attr = &vdd2_opp_attr;
		if (vdd2_locked && value == 0) {
			resource_unlock_opp(PRCM_VDD2);
			resource_refresh();
			vdd2_locked = 0;
			return n;
		}
		if (vdd2_locked == 0 && value != 0) {
			resource_lock_opp(PRCM_VDD2);
			vdd2_locked = 1;
		}
	}

	if (attr == &vdd1_opp_attr) {
		if (value < 1 || value > 5) {
			printk(KERN_ERR "vdd_opp_store: Invalid value\n");
			return -EINVAL;
		}
		resource_set_opp_level(PRCM_VDD1, value, flags);
	} else if (attr == &vdd2_opp_attr) {
		if (value < 1 || value > 3) {
			printk(KERN_ERR "vdd_opp_store: Invalid value\n");
			return -EINVAL;
		}
		resource_set_opp_level(PRCM_VDD2, value, flags);
	} else {
		return -EINVAL;
	}
	return n;
}
#endif

void omap2_block_sleep(void)
{
	atomic_inc(&sleep_block);
}

void omap2_allow_sleep(void)
{
	int i;

	i = atomic_dec_return(&sleep_block);
	BUG_ON(i < 0);
}

int get_last_off_on_transaction_id(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapdev *odev = omapdev_find_pdev(pdev);
	struct powerdomain *pwrdm;

	if (odev) {
		pwrdm = omapdev_get_pwrdm(odev);
		if (pwrdm)
			return pwrdm->state_counter[0] & INT_MAX;
	}

	return 0;
}

static int __init omap_pm_init(void)
{
	int error = -1;

	if (cpu_is_omap24xx())
		error = omap2_pm_init();
	if (cpu_is_omap34xx())
		error = omap3_pm_init();
	if (error) {
		printk(KERN_ERR "omap2|3_pm_init failed: %d\n", error);
		return error;
	}

	/* disabled till drivers are fixed */
	enable_dyn_sleep = 0;
	error = sysfs_create_file(power_kobj, &sleep_while_idle_attr.attr);
	if (error)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
	error = sysfs_create_file(power_kobj,
				  &clocks_off_while_idle_attr.attr);
	if (error)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
	error = sysfs_create_file(power_kobj,
				  &enable_off_mode_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj, &vdd1_opps_vsel.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj, &dsp_opps_rate.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}

#ifdef CONFIG_OMAP_PM_SRF
	error = sysfs_create_file(power_kobj,
				  &vdd1_opp_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj,
				  &vdd2_opp_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(power_kobj, &vdd1_lock_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(power_kobj, &vdd2_lock_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
#endif
	voltage_off_while_idle = 0;
	/* Going to 0V on anything under ES2.1 will eventually cause a crash */
	if (omap_rev() > OMAP3430_REV_ES2_0) {
		error = sysfs_create_file(power_kobj,
				  &voltage_off_while_idle_attr.attr);
		if (error)
			printk(KERN_ERR "sysfs_create_file failed: %d\n",
								error);
	}
	return error;
}

late_initcall(omap_pm_init);
