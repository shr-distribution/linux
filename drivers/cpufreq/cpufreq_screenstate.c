/*
 *  linux/drivers/cpufreq/cpufreq_screenstate.c
 *
 *  Marco Benton marco@unixpsycho.com 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>

// Cap min freq capped to 500MHz, undef to set to policy->min
//#define SCREENSTATE_CAP_MIN_FREQ

static unsigned int cpu_is_managed=0;
static unsigned int lcd_state;
static unsigned int charging_state;

int gadget_event_state_current(void);
static int ds2784_getcurrent(int *ret_current);
static inline void check_charger(struct work_struct *work);
unsigned short get_vdd1_arm_opp_for_freq(unsigned int freq);

static DEFINE_MUTEX(screenstate_mutex);

static DECLARE_DELAYED_WORK(dbs_work, check_charger);

#ifdef CONFIG_CPU_FREQ_OVERRIDE
u8 override_show_chrg_ovrd();
u8 ch_override;
#endif

static inline void check_charger(struct work_struct *work) {
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	int cur=0,current_mA=0;

	mutex_lock(&screenstate_mutex);
#ifdef CONFIG_CPU_FREQ_OVERRIDE
	int oc = override_show_chrg_ovrd();

	if((oc) && (!ch_override)) {
		printk("screenstate: charger override set\n");
		charging_state=0;
		ch_override=1;
		if(lcd_state) __cpufreq_driver_target(policy, policy->max,
							CPUFREQ_RELATION_H);
	}
	else if((!oc) && (ch_override)) {
		ch_override=0;
		printk("screenstate: charger override off\n");
	}

	if(ch_override) goto out;
#endif

	ds2784_getcurrent(&cur);
	current_mA=gadget_event_state_current();
	if((cur>0) && (current_mA < 500)) {
		// Assume Touchstone
		if(!charging_state) {
			charging_state=1;
			__cpufreq_driver_target(policy, 500000,
						CPUFREQ_RELATION_L);
			printk("screenstate: TS found!\n");
		}
	} 
	else {
		if(current_mA == 1000) {
			if(!charging_state) {
				charging_state=1;
				__cpufreq_driver_target(policy, 500000,
							CPUFREQ_RELATION_L);
				printk("screenstate: 1000mA charger found!\n");
			}
		}
		else {
			if(charging_state) {
				charging_state=0;
				printk("screenstate: charger unplugged!\n");
				if(lcd_state)
					__cpufreq_driver_target(policy,
						policy->max, CPUFREQ_RELATION_H);
			}
		}
	}
#ifdef CONFIG_CPU_FREQ_OVERRIDE
out:
#endif
	schedule_delayed_work(&dbs_work,1000);
	mutex_unlock(&screenstate_mutex);
	return;
}

static int cpufreq_governor_screenstate(struct cpufreq_policy *policy,
				   unsigned int event) {

	switch (event) {
		case CPUFREQ_GOV_START:
			if(cpu_is_managed) break;

			cpu_is_managed = 1;
			lcd_state = 1;
			charging_state = 0;
#ifdef CONFIG_CPU_FREQ_OVERRIDE
			ch_override = override_show_chrg_ovrd();
#endif
			__cpufreq_driver_target(policy, policy->max,
							CPUFREQ_RELATION_H);
			schedule_delayed_work(&dbs_work,1000);
			
			break;
		case CPUFREQ_GOV_STOP:
			cpu_is_managed = 0;
			lcd_state = 0;
#ifdef CONFIG_CPU_FREQ_OVERRIDE
			ch_override = 0;
#endif
			cancel_delayed_work(&dbs_work);
			break;
		case CPUFREQ_GOV_LIMITS:
			printk("screenstate: policy change\n");
			if(charging_state)
				__cpufreq_driver_target(policy, 500000,
							CPUFREQ_RELATION_L);
			else {
				if(lcd_state) __cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
#ifdef SCREENSTATE_CAP_MIN_FREQ
				else __cpufreq_driver_target(policy, 500000,
							CPUFREQ_RELATION_L);
#else
				else __cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
#endif
			}
			break;
		}
		return 0;
}

struct cpufreq_governor cpufreq_gov_screenstate = {
	.name		= "screenstate",
	.governor	= cpufreq_governor_screenstate,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gov_screenstate_init(void) {
	return cpufreq_register_governor(&cpufreq_gov_screenstate);
}

static void __exit cpufreq_gov_screenstate_exit(void) {
	flush_scheduled_work();
	cpufreq_unregister_governor(&cpufreq_gov_screenstate);
}

void cpufreq_gov_screenstate_lcdoff(void) {
	if(cpu_is_managed) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(0);
		mutex_lock(&screenstate_mutex);
		printk("screenstate: lcd off\n");
		lcd_state = 0;
#ifdef SCREENSTATE_CAP_MIN_FREQ
		__cpufreq_driver_target(policy, 500000, CPUFREQ_RELATION_L);
#else
		__cpufreq_driver_target(policy, policy->min, CPUFREQ_RELATION_L);
#endif
		mutex_unlock(&screenstate_mutex);
	}
}
EXPORT_SYMBOL(cpufreq_gov_screenstate_lcdoff);

void cpufreq_gov_screenstate_lcdon(void) {
        if(cpu_is_managed) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(0);
		mutex_lock(&screenstate_mutex);
		printk("screenstate: lcd on\n");
		lcd_state = 1;
                if(!charging_state) __cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else __cpufreq_driver_target(policy, 500000, CPUFREQ_RELATION_L);
		mutex_unlock(&screenstate_mutex);
        }
}
EXPORT_SYMBOL(cpufreq_gov_screenstate_lcdon);

EXPORT_SYMBOL(cpufreq_gov_screenstate);

MODULE_AUTHOR ("marco@unixpsycho.com");
MODULE_DESCRIPTION ("CPUfreq policy governor 'screenstate'");
MODULE_LICENSE ("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCREENSTATE
fs_initcall(cpufreq_gov_screenstate_init);
#else
module_init(cpufreq_gov_screenstate_init);
#endif
module_exit(cpufreq_gov_screenstate_exit);
