// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - SBW (Spy-Bi-Wire) GPIO operations
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Thin GPIO descriptor wrappers that adapt the JTAG/SBW core to modern
 * gpiod_* operations. The legacy SBW interface uses board-specific function
 * pointers, so this file translates them into gpiod_set_value() etc. on the
 * tck/tdio/wakeup GPIOs owned by the active device.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/types.h>

#include "a6_internal.h"

uint8_t _convert_hex_char_to_decimal(uint8_t x)
{
	x -= '0';
	if (x > 9) {
		x = x - ('A' - ('9' + 1));
		if (x > 15)
			x = x - ('a' - 'A');
	}

	return x;
}

/*
 * GPIO descriptor wrapper functions for SBW interface
 * These replace the board-specific implementations with GPIO descriptors
 */
struct a6_device_state *current_sbw_state; /* For SBW operations */

static uint16_t a6_gpio_set_sbwtck(void)
{
	if (current_sbw_state && current_sbw_state->tck_gpio)
		gpiod_set_value(current_sbw_state->tck_gpio, 1);
	return 0;
}

static uint16_t a6_gpio_clr_sbwtck(void)
{
	if (current_sbw_state && current_sbw_state->tck_gpio)
		gpiod_set_value(current_sbw_state->tck_gpio, 0);
	return 0;
}

static uint16_t a6_gpio_set_sbwtdio(void)
{
	if (current_sbw_state && current_sbw_state->tdio_gpio)
		gpiod_set_value(current_sbw_state->tdio_gpio, 1);
	return 0;
}

static uint16_t a6_gpio_clr_sbwtdio(void)
{
	if (current_sbw_state && current_sbw_state->tdio_gpio)
		gpiod_set_value(current_sbw_state->tdio_gpio, 0);
	return 0;
}

static uint16_t a6_gpio_set_in_sbwtdio(void)
{
	if (current_sbw_state && current_sbw_state->tdio_gpio)
		gpiod_direction_input(current_sbw_state->tdio_gpio);
	return 0;
}

static uint16_t a6_gpio_set_out_sbwtdio(void)
{
	if (current_sbw_state && current_sbw_state->tdio_gpio)
		gpiod_direction_output(current_sbw_state->tdio_gpio, 0);
	return 0;
}

static uint16_t a6_gpio_get_sbwtdio(void)
{
	if (current_sbw_state && current_sbw_state->tdio_gpio)
		return gpiod_get_value(current_sbw_state->tdio_gpio);
	return 0;
}

static uint16_t a6_gpio_set_sbwakeup(void)
{
	if (current_sbw_state && current_sbw_state->wakeup_gpio)
		gpiod_set_value(current_sbw_state->wakeup_gpio, 1);
	return 0;
}

static uint16_t a6_gpio_clr_sbwakeup(void)
{
	if (current_sbw_state && current_sbw_state->wakeup_gpio)
		gpiod_set_value(current_sbw_state->wakeup_gpio, 0);
	return 0;
}

static void a6_delay_impl(uint32_t delay_us)
{
	if (delay_us < 10)
		udelay(delay_us);
	else if (delay_us < 20000)
		usleep_range(delay_us, delay_us + 10);
	else
		msleep(delay_us / 1000);
}

/* SBW interface structure for GPIO descriptor-based operations */
struct a6_sbw_interface a6_gpio_sbw_ops = {
	.a6_per_device_interface = {
		.SetSBWTCK = a6_gpio_set_sbwtck,
		.ClrSBWTCK = a6_gpio_clr_sbwtck,
		.SetSBWTDIO = a6_gpio_set_sbwtdio,
		.ClrSBWTDIO = a6_gpio_clr_sbwtdio,
		.SetInSBWTDIO = a6_gpio_set_in_sbwtdio,
		.SetOutSBWTDIO = a6_gpio_set_out_sbwtdio,
		.GetSBWTDIO = a6_gpio_get_sbwtdio,
		.SetSBWAKEUP = a6_gpio_set_sbwakeup,
		.ClrSBWAKEUP = a6_gpio_clr_sbwakeup,
	},
	.a6_per_target_interface = {
		.delay = a6_delay_impl,
	},
};
