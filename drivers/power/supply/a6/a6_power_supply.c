// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - power_supply class integration
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Wires the A6 controller into the kernel power_supply class so userspace can
 * read battery state via /sys/class/power_supply/a6-N. The supply is
 * registered either at probe time (if the device responds) or lazily on the
 * first hot-plug IRQ, hence the small a6_register_power_supply() helper.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "a6_internal.h"

/*
 * Modern kernel API support - Power Supply Integration
 */
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
	POWER_SUPPLY_PROP_HEALTH,
};

static int a6_battery_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct a6_device_state *state = power_supply_get_drvdata(psy);
	struct i2c_client *client = state->i2c_dev;
	uint8_t data[2];
	int ret;
	int32_t raw_val;
	/* Register IDs for 2-byte reads (LSB first, then MSB) */
	static const uint16_t volt_ids[] = {TS2_I2C_BAT_VOLT_LSB, TS2_I2C_BAT_VOLT_MSB};
	static const uint16_t cur_ids[] = {TS2_I2C_BAT_CUR_LSB, TS2_I2C_BAT_CUR_MSB};
	static const uint16_t avg_cur_ids[] = {TS2_I2C_BAT_AVG_CUR_LSB, TS2_I2C_BAT_AVG_CUR_MSB};
	static const uint16_t temp_ids[] = {TS2_I2C_BAT_TEMP_LSB, TS2_I2C_BAT_TEMP_MSB};
	static const uint16_t full_ids[] = {TS2_I2C_BAT_FULL_LSB, TS2_I2C_BAT_FULL_MSB};
	static const uint16_t rarc_id[] = {TS2_I2C_BAT_RARC};

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/* Use average current direction to determine charge status */
		ret = a6_i2c_read_reg(client, avg_cur_ids, 2, data);
		if (ret < 0)
			return ret;
		raw_val = *(int16_t *)data;
		/* Positive current = charging, negative = discharging */
		if (raw_val > 0)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (raw_val < 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;  /* Battery is always present */
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = a6_i2c_read_reg(client, volt_ids, 2, data);
		if (ret < 0)
			return ret;
		raw_val = *(int16_t *)data;
		/* 11-bit signed value, unit = 4880µV */
		val->intval = (raw_val >> 5) * 4880;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = a6_i2c_read_reg(client, cur_ids, 2, data);
		if (ret < 0)
			return ret;
		raw_val = *(int16_t *)data;
		/* Convert using rsense: (raw * 3125) / 2 / rsense */
		val->intval = (raw_val * 3125) / 2 / (int32_t)state->cached_rsense_val;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = a6_i2c_read_reg(client, avg_cur_ids, 2, data);
		if (ret < 0)
			return ret;
		raw_val = *(int16_t *)data;
		/* Convert using rsense: (raw * 3125) / 2 / rsense */
		val->intval = (raw_val * 3125) / 2 / (int32_t)state->cached_rsense_val;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ret = a6_i2c_read_reg(client, rarc_id, 1, data);
		if (ret < 0)
			return ret;
		val->intval = data[0];  /* RARC is already percentage 0-100 */
		break;

	case POWER_SUPPLY_PROP_TEMP:
		ret = a6_i2c_read_reg(client, temp_ids, 2, data);
		if (ret < 0)
			return ret;
		/* MSB is temperature in °C, power_supply expects 0.1°C units */
		val->intval = (int8_t)data[1] * 10;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = a6_i2c_read_reg(client, full_ids, 2, data);
		if (ret < 0)
			return ret;
		raw_val = *(int16_t *)data;
		/* Full capacity in 1.6mAh units, convert to µAh */
		val->intval = (raw_val * 1600);
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW: {
		/*
		 * The A6 doesn't expose a coulomb-counter-derived "remaining
		 * charge" register directly. Synthesize it from the relative
		 * remaining percentage (RARC, 0-100) and the full-charge
		 * capacity (FULL, 1.6 mAh units). Userspace expects µAh.
		 *
		 * Read both registers fresh in the same call so the result is
		 * self-consistent — RARC and FULL update on different cadences
		 * inside the MSP430 firmware, and a stale pair would yield a
		 * misleading "charge increased while discharging" reading.
		 *
		 * Compute (full_uAh / 100) * rarc rather than the algebraically
		 * cleaner (full_uAh * rarc) / 100 to avoid overflow: full_uAh
		 * can approach 5.2e7, and 5.2e7 * 100 = 5.2e9 overruns int32_t.
		 * The divide-first form loses at most 99 uAh of precision per
		 * read, which is negligible compared to A6 quantization.
		 */
		uint8_t rarc_data;
		int32_t full_uah;

		ret = a6_i2c_read_reg(client, rarc_id, 1, &rarc_data);
		if (ret < 0)
			return ret;
		ret = a6_i2c_read_reg(client, full_ids, 2, data);
		if (ret < 0)
			return ret;
		raw_val = *(int16_t *)data;
		full_uah = raw_val * 1600;
		val->intval = (full_uah / 100) * rarc_data;
		break;
	}

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/**
 * a6_register_power_supply() - Register power supply for A6 device
 * @state: Device state structure
 *
 * Registers the power supply interface for the A6 device. This can be called
 * either during probe (if device responds) or later during hot-plug when
 * a Touchstone charger is connected.
 *
 * Returns 0 on success, negative error code on failure.
 */
int a6_register_power_supply(struct a6_device_state *state)
{
	struct i2c_client *client = state->i2c_dev;
	struct power_supply_config psy_cfg = {};

	/* Already registered? */
	if (state->battery)
		return 0;

	psy_cfg.drv_data = state;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	/* Allocate name if not already done */
	if (!state->battery_desc.name) {
		state->battery_desc.name = devm_kasprintf(&client->dev, GFP_KERNEL,
							  "a6-%d", state->device_index);
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
		int rc = PTR_ERR(state->battery);
		state->battery = NULL;
		dev_err(&client->dev, "Failed to register power supply: %d\n", rc);
		return rc;
	}

	dev_info(&client->dev, "A6 power supply registered\n");
	return 0;
}
