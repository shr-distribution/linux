/*
 * linux/arch/arm/mach-omap3pe/fastpath.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/rtc.h>
#include <linux/battery_simple.h>

#include "power.h"

/**
 * Debug
 */

#define DEBUG(...)                           \
do {					                     \
	if (fastpath_debug_level)                \
		printk(__VA_ARGS__);                 \
} while (0)

#define INFO(...)                            \
do {                                         \
	printk(KERN_INFO __VA_ARGS__);           \
} while (0)

#define ERROR(...)                           \
do {                                         \
	printk(KERN_ERR __VA_ARGS__);            \
} while (0)

/**
 * Module parameters
 */

static uint32_t fastpath_enabled = 1;
module_param_named(enabled, fastpath_enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "enable/disable fastpath.");

/** 
* @brief Turn on debug messages.
*/
static uint32_t fastpath_debug_level = 0;
module_param_named(debug_level, fastpath_debug_level, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "fastpath debug level.");

struct battery {
	int enabled;
	int percent;
	int voltage_uV;
	int temperature_C;
	unsigned long last_read_sec;
};

struct battery battery;
struct battery fake_battery;

/** 
* @brief Turn fake battery on.
*/
module_param_named(fake_battery, fake_battery.enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(fake_battery, "Use fake_battery parameters.");

/** 
* @brief Fake battery parameters.
*/
module_param_named(fake_percent, fake_battery.percent, int, S_IRUGO | S_IWUSR);
module_param_named(fake_voltage_uV, fake_battery.voltage_uV, int, S_IRUGO | S_IWUSR);
module_param_named(fake_temperature_C, fake_battery.temperature_C, int, S_IRUGO | S_IWUSR);

/** 
* @brief Battery reporting thresholds
*/
static int battery_threshold_array[] = {
	100,
	20,
	10,
	5,
};
static int battery_threshold_size = ARRAY_SIZE(battery_threshold_array);

module_param_array_named(battery_thresholds, battery_threshold_array,
		 int, &battery_threshold_size, S_IRUGO | S_IWUSR);

/** 
* @brief Last reported threshold (index into battery_threshold_array)
*/
static int last_battery_threshold;
module_param_named(last_battery_threshold, last_battery_threshold, int, S_IRUGO | S_IWUSR);

/**
 * Globals
 */

static unsigned long alarm_user;

/** 
* @brief State nodes.
*/
enum {
	FASTPATH_STATE_WAKE = 0,
	FASTPATH_STATE_RTC_SYSTEM,
	FASTPATH_STATE_RTC_USER,
};
static int fastpath_state;

/** 
* @brief If from batterycheck, the reason why we woke up.
*/
enum {
	BATTERYCHECK_NONE = 0,
	BATTERYCHECK_THRESHOLD_REACHED,
	BATTERYCHECK_CRITICAL_LOW_BATTERY,
	BATTERYCHECK_CRITICAL_TEMPERATURE,
	BATTERYCHECK_END,
};
static int batterycheck_wakeup = BATTERYCHECK_NONE;

static const char *batterycheck_wakeup_string[BATTERYCHECK_END] = {
	"none",
	"threshold",
	"criticalbatt",
	"criticaltemp",
};

/**
 * Battery parameters... this should eventually be factored out into the arch
 * specific portion.
 */
#define BATTERY_MAX_TEMPERATURE_C  60
#define BATTERY_HIGH_TEMPERATURE_C 50     /* TODO check this experimentally */
#define BATTERY_MIN_VOLTAGE_UV    3400000

#define BATTERY_PERCENT_SAMPLES 5

#define BATTERY_VOLTAGE_SAMPLES 5
#define BATTERY_VOLTAGE_MAJORITY 3

#define SECS_PER_MIN 60
#define POLL_FREQ_SEC_3800          (10*SECS_PER_MIN)
#define POLL_FREQ_SEC_3600_3800     (5*SECS_PER_MIN)
#define POLL_FREQ_SEC_3500_3600     (1*SECS_PER_MIN)
#define POLL_FREQ_SEC_3400_3500     (10)
#define POLL_FREQ_SEC_LT_3400       (5)

#define POLL_FREQ_HIGH_TEMP         (5) 
#define POLL_FREQ_DEFAULT           (5)

struct battery_poll_freq {
	int voltage_uV;
	int poll_interval_secs;
};

/**
 * How frequently we poll for batttery updates when asleep.
 * TODO: make this a module parameter.
 */
static struct battery_poll_freq battery_poll_freq[] = {
  { 3800000, POLL_FREQ_SEC_3800      },
  { 3600000, POLL_FREQ_SEC_3600_3800 },
  { 3500000, POLL_FREQ_SEC_3500_3600 },
  { 3400000, POLL_FREQ_SEC_3400_3500 },
  { 0,		 POLL_FREQ_SEC_LT_3400   },
};

static int percent_sample[BATTERY_PERCENT_SAMPLES];
static int percent_sample_index = 0;

static int voltage_sample_uV[BATTERY_VOLTAGE_SAMPLES];
static int voltage_sample_index = 0;

/**
 * Methods.
 */

static int __init fastpath_init(void)
{
	fastpath_state = FASTPATH_STATE_WAKE;
	return 0;
}

static int __init batterycheck_init(void)
{
	batterycheck_wakeup = BATTERYCHECK_NONE;

	memset(percent_sample, 0, sizeof(percent_sample));
	memset(voltage_sample_uV, 0, sizeof(voltage_sample_uV));

	return 0;
}

static void percent_samples_record(int percent)
{
	percent_sample_index =
		( percent_sample_index + 1 ) % BATTERY_PERCENT_SAMPLES;

	percent_sample[percent_sample_index] = percent;
}

/** 
* @brief Returns the last percent sample.
* 
* @retval
*/
static int percent_get_last(void)
{
	return percent_sample[percent_sample_index];
}

static int battery_threshold_changed(void)
{
	int i;
	int percent;
	int threshold = 0;

	percent = percent_get_last();

	for (i = 0; i < ARRAY_SIZE(battery_threshold_array); i++) {
		if (percent <= battery_threshold_array[i]) {
			threshold = i;
		}
	}

	DEBUG("%s: Percent %d%%, threshold %d\n",
			__FUNCTION__, percent, threshold);
	
	if (threshold != last_battery_threshold) {
		last_battery_threshold = threshold;
		return 1;
	}
	return 0;
}

static void voltage_samples_record(int voltage)
{
	voltage_sample_index =
		( voltage_sample_index + 1 ) % BATTERY_VOLTAGE_SAMPLES;

	voltage_sample_uV[voltage_sample_index] = voltage;
}

static int voltage_samples_less_than(int min_voltage_uV)
{
	int i;
	int count = 0;
	int voltage;

	for (i = 0; i < BATTERY_VOLTAGE_SAMPLES; i++) {
		voltage = voltage_sample_uV[i];
		if (voltage != 0 && voltage <= min_voltage_uV)
			count++;
	}

	return count >= BATTERY_VOLTAGE_MAJORITY;
}

/** 
* @brief Returns the last voltage sample.
* 
* @retval
*/
static int voltage_get_last(void)
{
	return voltage_sample_uV[voltage_sample_index];
}

/** 
* @brief Returns the next wakeup time interval.
* 
* @retval
*/
static int battery_poll_next_seconds(void)
{
	int i;
	int poll_interval = POLL_FREQ_DEFAULT;
	int last_voltage;
	struct battery_poll_freq *poll_freq;
	
	last_voltage = voltage_get_last();

	for (i = 0; i < ARRAY_SIZE(battery_poll_freq); i++) {

		poll_freq = &battery_poll_freq[i];

		if (last_voltage >= poll_freq->voltage_uV) {
			poll_interval = poll_freq->poll_interval_secs;
			break;
		}
	}

	if (battery.temperature_C >= BATTERY_HIGH_TEMPERATURE_C)
	{
		poll_interval = min(poll_interval, POLL_FREQ_HIGH_TEMP);
	}

	return poll_interval;
}

static int battery_read(unsigned long now)
{
	int retval;
	int percent;
	int voltage_uV;
	int temperature_C;

	if (fake_battery.enabled) {
		/* Debugging overrides */
		percent = fake_battery.percent;
		voltage_uV = fake_battery.voltage_uV;
		temperature_C = fake_battery.temperature_C;
	}
	else {
		retval = battery_get_percent(&percent);
		if (retval < 0) {
			ERROR("%s: Could not get battery percent.\n", __FUNCTION__);
			goto error;
		}
		retval = battery_get_voltage(&voltage_uV);
		if (retval < 0) {
			ERROR("%s: Could not get battery voltage.\n", __FUNCTION__);
			goto error;
		}
		retval = battery_get_temperature(&temperature_C);
		if (retval < 0) {
			ERROR("%s: Could not get battery temperature.\n", __FUNCTION__);
			goto error;
		}
	}

	percent_samples_record(percent);
	voltage_samples_record(voltage_uV);

	battery.percent = percent;
	battery.voltage_uV = voltage_uV;
	battery.temperature_C = temperature_C;

	if (now) {
		battery.last_read_sec = now;
	}

	DEBUG("%s: Battery P: %d%%, V: %duV, T: %dC\n",
		__FUNCTION__, percent, voltage_uV, temperature_C);
	return 0;
error:
	return -1;
}

/** 
* @brief Returns true if battery level is critical, and false otherwise.
* 
* @retval
*/
static int battery_is_critical(void)
{
	return voltage_samples_less_than(BATTERY_MIN_VOLTAGE_UV);
}

static int battery_is_too_hot(void)
{
	return (battery.temperature_C >= BATTERY_MAX_TEMPERATURE_C);
}

/** 
* @brief Return the current rtc alarm
* 
* @retval
*/
static unsigned long rtc_alarm(struct rtc_device *rtc)
{
	int retval;
	unsigned long alarm_secs;
	struct rtc_wkalrm alm;

	retval = rtc_read_alarm(rtc, &alm);
	if (retval < 0) {
		ERROR("%s: unable to read the rtc alarm\n", __FUNCTION__);
		goto error;
	}

	if (alm.enabled) {
		DEBUG("%s: Read current rtc alarm of %02d:%02d:%02d\n", __FUNCTION__,
				alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

		rtc_tm_to_time(&alm.time, &alarm_secs);
	}
	else {
		alarm_secs = 0;
	}
	return alarm_secs;

error:
	return 0;
}

static int rtc_alarm_write(struct rtc_device *rtc, unsigned long next_alarm)
{
	int retval;
	struct rtc_wkalrm alm;

	alm.enabled = 1;
	rtc_time_to_tm(next_alarm, &alm.time);

	retval = rtc_set_alarm(rtc, &alm);
	if (retval < 0) {
		ERROR("%s: Error %d could not set the rtc alarm\n",
				__FUNCTION__, retval);
		goto error;
	}

	DEBUG("%s: Setting rtc alarm to %02d:%02d:%02d\n", __FUNCTION__,
		alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

error:
	return retval;
}

static unsigned long rtc_now(struct rtc_device *rtc)
{
	int retval;
	unsigned long now;
	struct rtc_wkalrm alm;

	retval = rtc_read_time(rtc, &alm.time);
	if (retval < 0) {
		ERROR("%s: unable to read the rtc (hardware) clock\n", __FUNCTION__);
		goto cleanup;
	}

	DEBUG("%s: Now %02d:%02d:%02d\n", __FUNCTION__,
			alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

	rtc_tm_to_time(&alm.time, &now);

	return now;
cleanup:
	return 0;
}

/** 
* @brief Set up RTC to wake us up periodically so that we can
*        check the battery.
* 
* @retval
*/
int fastpath_prepare(void)
{
	unsigned long now, next_alarm;
	struct rtc_device *rtc;

	DEBUG("================================%s\n", __FUNCTION__);

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (NULL == rtc) {
		ERROR("%s: unable to open rtc device (%s)\n",
			__FUNCTION__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -ENODEV;
	}

	if (!fastpath_enabled) {

		if (fastpath_debug_level) {
			rtc_now(rtc);
			rtc_alarm(rtc);
		}
		rtc_class_close(rtc);
		return 0;
	}

	/* now = current rtc time */
	now = rtc_now(rtc);

	/* Read battery if last reading was a while ago... */
	if (now > battery.last_read_sec + 10 * SECS_PER_MIN) {
		int retval;
		retval = battery_read(now);
		if (retval < 0) {
			DEBUG("%s: Error reading battery.\n", __FUNCTION__);
		}
	}

	/* Calculate the next alarm */
	next_alarm = now + battery_poll_next_seconds();

	/* On first entry, save the user rtc alarm */
	if (FASTPATH_STATE_WAKE == fastpath_state) {
		alarm_user = rtc_alarm(rtc);
	}

	if (fastpath_debug_level || FASTPATH_STATE_WAKE == fastpath_state) {
		struct rtc_time alm_tm;
		struct rtc_time now_tm;
		struct rtc_time next_batt_tm;
		struct rtc_time last_batt_tm;

		rtc_time_to_tm(now, &now_tm);
		rtc_time_to_tm(alarm_user, &alm_tm);
		rtc_time_to_tm(next_alarm, &next_batt_tm);
		rtc_time_to_tm(battery.last_read_sec, &last_batt_tm);

		INFO("%s: RTC = %02d:%02d:%02d, UserAlarm = %02d:%02d:%02d, "
				 "NextBatt = %02d:%02d:%02d (%ds), "
				 "LastBatt = %02d:%02d:%02d (%d%%,%duV,%dC)\n",
				__FUNCTION__,
			now_tm.tm_hour, now_tm.tm_min, now_tm.tm_sec,
			alm_tm.tm_hour, alm_tm.tm_min, alm_tm.tm_sec,
			next_batt_tm.tm_hour, next_batt_tm.tm_min, next_batt_tm.tm_sec,
			battery_poll_next_seconds(),
			last_batt_tm.tm_hour, last_batt_tm.tm_min, last_batt_tm.tm_sec,
			battery.percent, battery.voltage_uV, battery.temperature_C);
	}

	/* Pick whether next alarm should be user rtc alarm or system rtc alarm.
	 *
	 * alarm_user >= now - 2 is to catch those cases where the alarm might
	 * fire on the way down in the suspend path.  The suspend logic should
	 * treat these cases as fall-throughs, but it is not currently.
	 */ 
	if (alarm_user && alarm_user >= now - 2 &&
		alarm_user <= next_alarm) {

		next_alarm = alarm_user;
		fastpath_state = FASTPATH_STATE_RTC_USER;
	}
	else {
		fastpath_state = FASTPATH_STATE_RTC_SYSTEM;
	}

	/* Set the new alarm. */
	rtc_alarm_write(rtc, max(next_alarm, now + 2));

	rtc_class_close(rtc);
	return 0;
}

static const char *batterycheck_wakeup_to_string(void)
{
	return batterycheck_wakeup_string[batterycheck_wakeup];
}

/** 
* @brief Called to check if we should go back to sleep.
* 
* @retval 1 if we should go back to sleep.
*/
int fastpath_fastsleep(int wakeup_is_rtc)
{
	int fastsleep_required = 0;
	struct rtc_device *rtc;
	unsigned long now = 0;

	batterycheck_wakeup = BATTERYCHECK_NONE;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (NULL == rtc) {
		ERROR("%s: unable to open rtc device (%s)\n",
				__FUNCTION__, CONFIG_RTC_HCTOSYS_DEVICE);
	}
	else {
		now = rtc_now(rtc);
	}

	if (!fastpath_enabled) {

		if (fastpath_debug_level) {
			rtc_now(rtc);
			rtc_alarm(rtc);
		}
		goto end;
	}

	/* Skip battery check and don't re-enter sleep
	 * if not real RTC wakeup. */
	if (wakeup_is_rtc) {
		unsigned long now;
		unsigned long alarm;
		int diff;

		now = rtc_now(rtc);
		alarm = rtc_alarm(rtc);

		diff = alarm - now;
		if (diff > 0) {
			DEBUG("%s: not real RTC wakeup.\n", __FUNCTION__);
			wakeup_is_rtc = 0;
		}
	}

	if (FASTPATH_STATE_RTC_SYSTEM == fastpath_state && wakeup_is_rtc) {

		int retval;

		retval = battery_read(now);
		if (retval < 0) {
			DEBUG("%s: Error reading battery.\n", __FUNCTION__);
		}
		else if (battery_is_critical()) {
			batterycheck_wakeup = BATTERYCHECK_CRITICAL_LOW_BATTERY;
		}
		else if (battery_is_too_hot()) {
			batterycheck_wakeup = BATTERYCHECK_CRITICAL_TEMPERATURE;
		}
		else if (battery_threshold_changed()) {
			batterycheck_wakeup = BATTERYCHECK_THRESHOLD_REACHED;
		}

		if (batterycheck_wakeup != BATTERYCHECK_NONE) {
			DEBUG("%s: Battery %s... waking up.\n",
				__FUNCTION__, batterycheck_wakeup_to_string());
		}
		else {
			DEBUG("%s: Fast sleep!\n", __FUNCTION__);
			fastsleep_required = 1;
			goto end;
		}
	}

	/* Wake up path. */

	/* Restore the original user set rtc alarm. */	
	if (alarm_user && rtc) {

		rtc_alarm_write(rtc, alarm_user);
	}

	fastpath_state = FASTPATH_STATE_WAKE;

end:
	if (rtc) {
		rtc_class_close(rtc);
	}

	DEBUG("================================%s (sleep req %d)\n", __FUNCTION__, fastsleep_required);
	return fastsleep_required;
}

static ssize_t batterycheck_wakeup_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%s\n", batterycheck_wakeup_to_string());
}

/**
 * Init
 */

static struct subsys_attribute batterycheck_wakeup_sysfs = {
	.attr = {
		.name = __stringify(batterycheck_wakeup),
		.mode = 0644,
	},
	.show  = batterycheck_wakeup_show,
	.store = NULL,
};

static int __init fastpath_sysfs_init(void)
{
	int retval;

	retval = subsys_create_file(&power_subsys, &batterycheck_wakeup_sysfs);
	if (retval)
		goto error;

error:
	return retval;
}

static int __init fastpath_batterycheck_init(void)
{
	int rc;

	rc = batterycheck_init();
	if (rc) {
		ERROR("%s could not init batterycheck: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	rc = fastpath_init();
	if (rc) {
		ERROR("%s could not init fastpath: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	rc = fastpath_sysfs_init();
	if (rc) {
		ERROR("%s could not init fastpath_sysfs: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	return 0;
}

static void __exit fastpath_batterycheck_exit(void)
{
}

module_init(fastpath_batterycheck_init);
module_exit(fastpath_batterycheck_exit);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("Provides the ability to periodically wake device to check for critical battery.");
MODULE_LICENSE("GPL");
