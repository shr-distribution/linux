// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - power_supply class integration
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs.
 *
 * Wires the A6 controller into the kernel power_supply class so userspace can
 * read battery state via /sys/class/power_supply/a6-N. The supply is
 * registered either at probe time (if the device responds) or lazily on the
 * first hot-plug IRQ.
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "a6_internal.h"

static enum power_supply_property a6_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_HEALTH,
};

/* Register-id arrays for the 16-bit paired reads (LSB-leading per A6 fw) */
static const u16 a6_id_volt[]		= { TS2_I2C_BAT_VOLT_LSB,	TS2_I2C_BAT_VOLT_MSB };
static const u16 a6_id_cur[]		= { TS2_I2C_BAT_CUR_LSB,	TS2_I2C_BAT_CUR_MSB };
static const u16 a6_id_avg_cur[]	= { TS2_I2C_BAT_AVG_CUR_LSB,	TS2_I2C_BAT_AVG_CUR_MSB };
static const u16 a6_id_temp[]		= { TS2_I2C_BAT_TEMP_LSB,	TS2_I2C_BAT_TEMP_MSB };
static const u16 a6_id_full[]		= { TS2_I2C_BAT_FULL_LSB,	TS2_I2C_BAT_FULL_MSB };
static const u16 a6_id_coulomb[]	= { TS2_I2C_BAT_COULOMB_LSB,	TS2_I2C_BAT_COULOMB_MSB };
static const u16 a6_id_rarc[]		= { TS2_I2C_BAT_RARC };

/*
 * Read a 16-bit signed register pair (LSB, MSB) into a host s16.
 *
 * The dispatcher writes the LSB into data[0] and the MSB into data[1]
 * regardless of host endianness, so an explicit little-endian unaligned
 * load gives the correct value on every supported architecture and avoids
 * the strict-aliasing UB inherent in the old (s16 *) cast.
 */
static int a6_read_s16(struct i2c_client *client, const u16 *ids, s16 *out)
{
	u8 data[2];
	int ret;

	ret = a6_i2c_read_reg(client, ids, 2, data);
	if (ret < 0)
		return ret;

	*out = (s16)get_unaligned_le16(data);
	return 0;
}

static int a6_current_uA(const struct a6_device_state *state, s16 raw)
{
	/* I = raw * (3.125 mV / 2) / rsense(mOhm) → uA */
	return ((int)raw * 3125) / 2 / (int)state->cached_rsense_val;
}

static int a6_battery_get_property_locked(struct a6_device_state *state,
					  enum power_supply_property psp,
					  union power_supply_propval *val)
{
	struct i2c_client *client = state->i2c_dev;
	int ret;
	s16 raw;
	u8 byte;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = a6_read_s16(client, a6_id_avg_cur, &raw);
		if (ret < 0)
			return ret;
		/* +ve current = charging, -ve = discharging */
		if (raw > 0)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (raw < 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		/*
		 * PRESENT mirrors whether the controller has been
		 * initialised. Userspace polls this on a stale supply when
		 * the controller is hot-removed.
		 */
		val->intval = test_bit(IS_INITIALIZED_BIT, state->flags) ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = a6_read_s16(client, a6_id_volt, &raw);
		if (ret < 0)
			return ret;
		/* 11-bit signed result, LSB = 4880 uV */
		val->intval = ((int)raw >> 5) * 4880;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = a6_read_s16(client, a6_id_cur, &raw);
		if (ret < 0)
			return ret;
		val->intval = a6_current_uA(state, raw);
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = a6_read_s16(client, a6_id_avg_cur, &raw);
		if (ret < 0)
			return ret;
		val->intval = a6_current_uA(state, raw);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ret = a6_i2c_read_reg(client, a6_id_rarc, 1, &byte);
		if (ret < 0)
			return ret;
		val->intval = byte;	/* RARC is already 0..100 */
		break;

	case POWER_SUPPLY_PROP_TEMP: {
		u8 temp_data[2];

		ret = a6_i2c_read_reg(client, a6_id_temp, 2, temp_data);
		if (ret < 0)
			return ret;
		/*
		 * temp_data[0] = LSB (fractional, ignored),
		 * temp_data[1] = MSB (signed degrees C).
		 * power_supply expects 0.1 C units.
		 */
		val->intval = (s8)temp_data[1] * 10;
		break;
	}

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = a6_read_s16(client, a6_id_full, &raw);
		if (ret < 0)
			return ret;
		/* Full capacity, 1.6 mAh units → uAh */
		val->intval = (int)raw * 1600;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW: {
		/*
		 * Synthesize remaining charge from RARC (0..100 %) and
		 * FULL (1.6 mAh units). Compute (full_uAh / 100) * rarc
		 * rather than the algebraically cleaner (full_uAh * rarc)
		 * / 100 because full_uAh can approach 5.2e7 and the
		 * pre-divide product would overflow s32. The divide-first
		 * form loses at most 99 uAh of precision per read, which
		 * is negligible compared to A6 quantization.
		 */
		s16 full_raw;
		int full_uah;

		ret = a6_i2c_read_reg(client, a6_id_rarc, 1, &byte);
		if (ret < 0)
			return ret;
		ret = a6_read_s16(client, a6_id_full, &full_raw);
		if (ret < 0)
			return ret;
		full_uah = (int)full_raw * 1600;
		val->intval = (full_uah / 100) * byte;
		break;
	}

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/* Raw coulomb counter, 6.25 uVh units → uAh */
		ret = a6_read_s16(client, a6_id_coulomb, &raw);
		if (ret < 0)
			return ret;
		val->intval = ((int)raw * 6250) / (int)state->cached_rsense_val;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int a6_battery_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct a6_device_state *state = power_supply_get_drvdata(psy);
	int ret;

	/*
	 * Serialise the per-property back-to-back A6 register reads against
	 * the IRQ bottom-half (which may run a6_init_state) and any other
	 * concurrent power_supply user.
	 */
	guard(mutex)(&state->dev_mutex);

	/*
	 * Assert wakeup_gpio around the I2C transactions. The A6 firmware
	 * can NAK accesses while in low-power mode, matching the same
	 * hardware contract a6_init_state() / a6_irq_work_handler() honor.
	 *
	 * Give the MSP430 a chance to exit LPM3/4 before driving the bus:
	 * the chip NAKs the address phase if the I2C transfer fires within
	 * microseconds of the wake edge. ~1 ms is enough in practice (the
	 * secondary A6 on multi-cell TouchPads sleeps deepest and was the
	 * one observed NAKing without this settle).
	 */
	gpiod_set_value_cansleep(state->wakeup_gpio, 1);
	usleep_range(1000, 2000);
	ret = a6_battery_get_property_locked(state, psp, val);
	gpiod_set_value_cansleep(state->wakeup_gpio, 0);
	return ret;
}

/**
 * a6_register_power_supply() - register the power_supply for an A6 device
 * @state: device state
 *
 * Registers either during probe (when the controller responds at bind
 * time) or later from the IRQ bottom-half on hot-plug (e.g. Touchstone
 * connect). Idempotent: subsequent calls after a successful registration
 * are a no-op.
 */
int a6_register_power_supply(struct a6_device_state *state)
{
	struct i2c_client *client = state->i2c_dev;
	struct power_supply_config psy_cfg = {};

	if (state->battery)
		return 0;

	psy_cfg.drv_data = state;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	if (!state->battery_desc.name) {
		state->battery_desc.name = devm_kasprintf(&client->dev,
							  GFP_KERNEL, "a6-%d",
							  state->device_index);
		if (!state->battery_desc.name)
			return -ENOMEM;
	}

	state->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	state->battery_desc.properties = a6_battery_props;
	state->battery_desc.num_properties = ARRAY_SIZE(a6_battery_props);
	state->battery_desc.get_property = a6_battery_get_property;

	state->battery = devm_power_supply_register(&client->dev,
						    &state->battery_desc,
						    &psy_cfg);
	if (IS_ERR(state->battery)) {
		int ret = PTR_ERR(state->battery);

		state->battery = NULL;
		dev_err(&client->dev,
			"failed to register power_supply: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "power_supply %s registered\n",
		 state->battery_desc.name);
	return 0;
}
