/*
 *  drivers/cpufreq/cpufreq_override.c
 *
 *  	Marco Benton <marco@unixpsycho.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/cpu.h>
#include <linux/sysfs.h>
#include <linux/cpufreq.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>

// VDD1 Vsel max
#define VDD1_VSEL_MAX 112

// VDD1 Vsel min
#define VDD1_VSEL_MIN 25

// VDD2 Vsel max
#define VDD2_VSEL_MAX 55

// VDD2 Vsel min
#define VDD2_VSEL_MIN 25

// High temp alarm and cap
#define HIGHTEMP_SCALEBACK 55

//Reset temp from alarm
#define LOWTEMP_RESET 50

// Polling frequency jiffies
#define OVERRIDE_POLLING 1000

// Battery scaleback percent
#define BATTERY_PERCENT 25

// Battery scaleback speed
#define BATTERY_PERCENT_SPEED 500000

void omap_pm_opp_get_volts(u8 *vdd1_volts[]);
void omap_pm_opp_set_volts(u8 vdd1_volts[]);
void omap_pm_opp_get_vdd2_volts(u8 *vdd2_volts[]);
void omap_pm_opp_set_vdd2_volts(u8 vdd2_volts[]);
void omap_pm_opp_get_vdd2_freq(u8 *vdd2_freqs[]);
int omap34xx_get_temp(void);
int cpufreq_set_policy(struct cpufreq_policy *policy);
int ds2784_getpercent(int *ret_percent);

static inline void check_stuff(struct work_struct *work);
int prev_maxspeed_temp=0;
int prev_minspeed_temp=0;
int prev_maxspeed_bat=0;
int prev_minspeed_bat=0;
#ifdef CONFIG_CPU_FREQ_GOV_SCREENSTATE
u8 charger_override=0;
#endif

static u32 override_hightemp=HIGHTEMP_SCALEBACK;
static u32 override_lowtemp=LOWTEMP_RESET;
static int battery_scaleback_percent=BATTERY_PERCENT;
static int battery_scaleback_speed=BATTERY_PERCENT_SPEED;

static unsigned int overtemp_alarm=0;
static unsigned int battery_alarm=0;

static DEFINE_MUTEX(override_mutex);
static DECLARE_DELAYED_WORK(dbs_work, check_stuff);

#define CPUFREQ_OVERRIDE_ATTR(_name,_mode,_show,_store) \
static struct freq_attr _attr_##_name = {\
        .attr = {.name = __stringify(_name), .mode = _mode, }, \
        .show = _show,\
        .store = _store,\
};

#define CPUFREQ_OVERRIDE_ATTR2(_name,_mode,_show) \
static struct freq_attr _attr_##_name = {\
	.attr = {.name = __stringify(_name), .mode = _mode, }, \
	.show = _show,\
};

#ifdef CONFIG_MACH_SIRLOIN_3630
static int tidx = 1;
#else
static int tidx = 0;
#endif

static inline void check_stuff(struct work_struct *work) {
        struct cpufreq_policy new_policy, *policy = cpufreq_cpu_get(0);
        u32 cputemp;
	int battery_per;

        mutex_lock(&override_mutex);
        cputemp = omap34xx_get_temp();    // Get CPU temp
	ds2784_getpercent(&battery_per);  // Get battery percent left

	// Check values in case driver hasnt polled
	battery_per = (battery_per > 0) ? battery_per : 100;
	cputemp = (cputemp < 100) ? cputemp : 0;

        if(cputemp > override_hightemp) {
                if(!overtemp_alarm) {
                        printk("CPUfreq: CPU temp warning! %dC\n",cputemp);
                        overtemp_alarm = 1;
                        cpufreq_get_policy(&new_policy,0);
                        prev_minspeed_temp=policy->min;
                        prev_maxspeed_temp=policy->max;
                        new_policy.min=500000;
                        new_policy.max=500000;
			cpufreq_set_policy(&new_policy);
                }
        }
        else {
                if((overtemp_alarm) && (cputemp < override_lowtemp)) {
                        printk("CPUfreq: CPU temp back under control! %dC\n",
								cputemp);
                        if (overtemp_alarm) {
				cpufreq_get_policy(&new_policy,0);
				new_policy.min=prev_minspeed_temp;
				new_policy.max=prev_maxspeed_temp;
				cpufreq_set_policy(&new_policy);
                        	overtemp_alarm = 0;
			}
                }
        }

	if(battery_per<battery_scaleback_percent) {
		if((!battery_alarm) && (!overtemp_alarm)) {
			printk("CPUFreq: battery low! < %d%%\n",battery_per);
			battery_alarm = 1;
			// TODO: clean this up to not call all this code twice
			cpufreq_get_policy(&new_policy,0);
			prev_minspeed_bat=policy->min;
			prev_maxspeed_bat=policy->max;
			new_policy.min=battery_scaleback_speed;
			new_policy.max=battery_scaleback_speed;
			cpufreq_set_policy(&new_policy);
		}
	}
	else {
		if((battery_alarm) && (!overtemp_alarm)) {
			printk("CPUFreq: battery OK\n");
			cpufreq_get_policy(&new_policy,0);
			new_policy.min=prev_minspeed_bat;
			new_policy.max=prev_maxspeed_bat;
			cpufreq_set_policy(&new_policy);
			battery_alarm = 0;
		}
	}

        mutex_unlock(&override_mutex);
	schedule_delayed_work(&dbs_work,OVERRIDE_POLLING);
}

static ssize_t show_vdd1_vsel_max(struct cpufreq_policy *policy, char *buf) {
	return sprintf(buf, "%hu\n",VDD1_VSEL_MAX);
}

static ssize_t show_vdd1_vsel_min(struct cpufreq_policy *policy, char *buf) {
	return sprintf(buf, "%hu\n",VDD1_VSEL_MIN);
}

static ssize_t show_vdd1_vsel(struct cpufreq_policy *policy, char *buf) {
        u8 volt[2][7];

        omap_pm_opp_get_volts(&volt);
#ifdef CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
        return sprintf(buf, "%hu %hu %hu %hu %hu\n", volt[tidx][6],
                                        volt[tidx][5],volt[tidx][4],
                                        volt[tidx][3],volt[tidx][2]);
#else
        return sprintf(buf, "%hu %hu %hu %hu %hu %hu %hu\n", volt[tidx][6],
                                    volt[tidx][5],volt[tidx][4],volt[tidx][3],
                                    volt[tidx][2],volt[tidx][1],volt[tidx][0]);
#endif
}

static ssize_t show_vdd2_vsel_max(struct cpufreq_policy *policy, char *buf) {
        return sprintf(buf, "%hu\n",VDD2_VSEL_MAX);
}

static ssize_t show_vdd2_vsel_min(struct cpufreq_policy *policy, char *buf) {
        return sprintf(buf, "%hu\n",VDD2_VSEL_MIN);
}

static ssize_t show_vdd2_vsel(struct cpufreq_policy *policy, char *buf) {
        u8 volt[2][3];

        omap_pm_opp_get_vdd2_volts(&volt);
        return sprintf(buf, "%hu %hu %hu\n", volt[tidx][2],
                                        volt[tidx][1],volt[tidx][0]);
}

static ssize_t show_vdd2_freqs(struct cpufreq_policy *policy, char *buf) {
        u8 freqs[2][3];
        omap_pm_opp_get_vdd2_freq(&freqs);

        return sprintf(buf, "%hu %hu %hu\n", freqs[tidx][2],
                                        freqs[tidx][1],freqs[tidx][0]);
}

static ssize_t store_vdd1_vsel(struct cpufreq_policy *policy, char *buf,
						size_t count) {
        u8 volt[2][7], i;

        mutex_lock(&override_mutex);
#ifdef CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
        if(sscanf(buf, "%hhu %hhu %hhu %hhu %hhu", &volt[tidx][6],&volt[tidx][5],
                       &volt[tidx][4],&volt[tidx][3],&volt[tidx][2]) == 5) {
		for(i=0;i < 5;i++) {
#else
        if(sscanf(buf, "%hhu %hhu %hhu %hhu %hhu %hhu %hhu", &volt[tidx][6],&volt[tidx][5],
                                   &volt[tidx][4],&volt[tidx][3],&volt[tidx][2],
                                   &volt[tidx][1],&volt[tidx][0]) == 7) {
		for(i=0;i < 7;i++) {
#endif
			if((volt[tidx][i] < VDD1_VSEL_MIN) || (volt[tidx][i] >
							VDD1_VSEL_MAX)) {
				printk("CPUfreq: invalid vsel\n");
				break;
			}
		}
#ifdef CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
		if(i == 5) omap_pm_opp_set_volts(volt);
#else
		if(i == 7) omap_pm_opp_set_volts(volt);
#endif
	}
	else printk("CPUfreq: missing vsel values\n");

        mutex_unlock(&override_mutex);
	return count;
}

static ssize_t store_vdd2_vsel(struct cpufreq_policy *policy, char *buf,
                                                size_t count) {
        u8 volt[2][3], i;

        mutex_lock(&override_mutex);
        if(sscanf(buf, "%hhu %hhu %hhu", &volt[tidx][2],&volt[tidx][1], &volt[tidx][0]) == 3) {
                for(i=0;i < 3;i++) {
                        if((volt[tidx][i] < VDD2_VSEL_MIN) || (volt[tidx][i] >
                                                        VDD2_VSEL_MAX)) {
                                printk("CPUfreq: invalid vsel\n");
                                break;
                        }
                }
                if(i == 3) omap_pm_opp_set_vdd2_volts(volt);
        }
        else printk("CPUfreq: missing vsel values\n");

        mutex_unlock(&override_mutex);
        return count;
}


static ssize_t show_hightemp_scaleback(struct cpufreq_policy *policy,
						char *buf) {
        return sprintf(buf, "%d\n", override_hightemp);
}

static ssize_t store_hightemp_scaleback(struct cpufreq_policy *policy,
						char *buf, size_t count) {
        int maxtemp=0;

        if(sscanf(buf, "%d", &maxtemp) == 1)
                override_hightemp=(maxtemp) ? maxtemp : HIGHTEMP_SCALEBACK;
        else printk("CPUfreq: invalid max temp\n");

	return count;
}

static ssize_t show_battery_scaleback_per(struct cpufreq_policy *policy,
                                                char *buf) {
	return sprintf(buf, "%d\n", battery_scaleback_percent);
}

static ssize_t store_battery_scaleback_per(struct cpufreq_policy *policy,
                                                char *buf, size_t count) {
	int bat=0;

	if(sscanf(buf, "%d", &bat) == 1)
		battery_scaleback_percent=((bat > -1) && (bat<100))
						? bat : BATTERY_PERCENT;
	else printk("CPUfreq: invalid battery percentage\n");

	return count;
}

static ssize_t show_battery_scaleback_speed(struct cpufreq_policy *policy,
                                                char *buf) {
	return sprintf(buf, "%d\n", battery_scaleback_speed);
}

static ssize_t store_battery_scaleback_speed(struct cpufreq_policy *policy,
                                                char *buf, size_t count) {
	int bat=0;

	if(sscanf(buf, "%d", &bat) == 1)
		battery_scaleback_speed=(bat>125000)
						? bat : BATTERY_PERCENT_SPEED;
	else printk("CPUfreq: invalid battery scaleback speed\n");

	return count;
}

static ssize_t show_lowtemp_reset(struct cpufreq_policy *policy, char *buf) {

        return sprintf(buf, "%d\n", override_lowtemp);
}

static ssize_t store_lowtemp_reset(struct cpufreq_policy *policy, char *buf,
						size_t count) {
        int lowtemp=0;

        if(sscanf(buf, "%d", &lowtemp) == 1)
                override_lowtemp=(lowtemp) ? lowtemp : LOWTEMP_RESET;
        else printk("CPUfreq: invalid low temp\n");

	return count;
}

#ifdef CONFIG_CPU_FREQ_GOV_SCREENSTATE
static ssize_t show_charger_override(struct cpufreq_policy *policy, char *buf) {
	return sprintf(buf, "%hu\n",charger_override);
}

static ssize_t store_charger_override(struct cpufreq_policy *policy,
					char *buf, size_t count) {
	u8 override=0;

	if(sscanf(buf, "%hhu", &override) == 1)
		charger_override=(override) ? 1 : 0;
	else printk("CPUfreq: invalid charger override value\n");

	return count;
}

int override_show_chrg_ovrd() {
	return charger_override;
}
EXPORT_SYMBOL(override_show_chrg_ovrd);

CPUFREQ_OVERRIDE_ATTR(override_charger,0644,show_charger_override,
			store_charger_override);
#endif

CPUFREQ_OVERRIDE_ATTR(vdd1_vsel,0644,show_vdd1_vsel,store_vdd1_vsel);
CPUFREQ_OVERRIDE_ATTR(vdd2_vsel,0644,show_vdd2_vsel,store_vdd2_vsel);
CPUFREQ_OVERRIDE_ATTR(battery_scaleback_percent,0644,
			show_battery_scaleback_per,
			store_battery_scaleback_per);
CPUFREQ_OVERRIDE_ATTR(battery_scaleback_speed,0644,
			show_battery_scaleback_speed,
			store_battery_scaleback_speed);
CPUFREQ_OVERRIDE_ATTR2(vdd1_vsel_min,0444,show_vdd1_vsel_min);
CPUFREQ_OVERRIDE_ATTR2(vdd1_vsel_max,0444,show_vdd1_vsel_max);
CPUFREQ_OVERRIDE_ATTR2(vdd2_vsel_min,0444,show_vdd2_vsel_min);
CPUFREQ_OVERRIDE_ATTR2(vdd2_vsel_max,0444,show_vdd2_vsel_max);
CPUFREQ_OVERRIDE_ATTR2(vdd2_freqs,0444,show_vdd2_freqs);
CPUFREQ_OVERRIDE_ATTR(cpu_hightemp_alarm,0644,show_hightemp_scaleback,
			store_hightemp_scaleback);
CPUFREQ_OVERRIDE_ATTR(cpu_hightemp_reset,0644,show_lowtemp_reset,
			store_lowtemp_reset);

static struct attribute *default_attrs[] = {
        &_attr_vdd1_vsel.attr,
        &_attr_vdd1_vsel_min.attr,
        &_attr_vdd1_vsel_max.attr,
        &_attr_vdd2_vsel.attr,
        &_attr_vdd2_vsel_min.attr,
        &_attr_vdd2_vsel_max.attr,
        &_attr_vdd2_freqs.attr,
        &_attr_cpu_hightemp_alarm.attr,
        &_attr_cpu_hightemp_reset.attr,
	&_attr_battery_scaleback_percent.attr,
	&_attr_battery_scaleback_speed.attr,
#ifdef CONFIG_CPU_FREQ_GOV_SCREENSTATE
	&_attr_override_charger.attr,
#endif
        NULL
};

static struct attribute_group override_attr_group = {
        .attrs = default_attrs,
        .name = "override"
};

int cpufreq_override_driver_init(void) {
	schedule_delayed_work(&dbs_work,OVERRIDE_POLLING);
        struct cpufreq_policy *data = cpufreq_cpu_get(0);
        return sysfs_create_group(&data->kobj,&override_attr_group);
}
EXPORT_SYMBOL(cpufreq_override_driver_init);

void cpufreq_override_driver_exit(void) {
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	cancel_delayed_work(&dbs_work);
	sysfs_remove_group(&policy->kobj, &override_attr_group);
	flush_scheduled_work();
}
EXPORT_SYMBOL(cpufreq_override_driver_exit);

MODULE_AUTHOR ("marco@unixpsycho.com");
MODULE_DESCRIPTION ("'cpufreq_override' - A driver to do cool stuff ");
MODULE_LICENSE ("GPL");
