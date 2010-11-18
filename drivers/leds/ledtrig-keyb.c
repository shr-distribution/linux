/*
 * LED keyboard driver.
 *
 * Author: Matan Ziv-Av <matan@svgalib.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include "leds.h"

static int keyb_led_active;
static struct led_classdev *keyb_led_cdev;

void keyb_led_set (int v) {
	if(keyb_led_active) {
		led_set_brightness(keyb_led_cdev, v ? LED_FULL:LED_OFF ) ;
	}
}

EXPORT_SYMBOL_GPL(keyb_led_set);
//static DEVICE_ATTR(delay_on, 0644, led_delay_on_show, led_delay_on_store);
//static DEVICE_ATTR(delay_off, 0644, led_delay_off_show, led_delay_off_store);

static void trig_activate(struct led_classdev *led_cdev)
{
	keyb_led_active=1;
	keyb_led_cdev = led_cdev;
}

static void trig_deactivate(struct led_classdev *led_cdev)
{
	keyb_led_active=0;
}

static struct led_trigger keyb_led_trigger = {
	.name     = "keyb",
	.activate = trig_activate,
	.deactivate = trig_deactivate,
};

static int __init keyb_trig_init(void)
{
	return led_trigger_register(&keyb_led_trigger);
}

static void __exit keyb_trig_exit(void)
{
	led_trigger_unregister(&keyb_led_trigger);
}

module_init(keyb_trig_init);
module_exit(keyb_trig_exit);

MODULE_AUTHOR("Matan Ziv-Av <matan@svgalib.org>");
MODULE_DESCRIPTION("Keyboard LED trigger");
MODULE_LICENSE("GPL");
