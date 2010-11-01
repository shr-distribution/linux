/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>


#if 0
#define DRIVER_VERSION			"1.1.0"

#define BQ27x00_REG_TEMP		0x06 //   BQ27x00_TEMP_L OK
#define BQ27x00_REG_VOLT		0x08 //   BQ27x00_VOLT_L OK
#define BQ27x00_REG_AI			0x14 //   BQ27x00_AI_L OK
#define BQ27x00_REG_FLAGS		0x0A //   BQ27x00_FLAGS OK
#define BQ27x00_REG_TTE			0x16 //   BQ27x00_TTE_L OK
#define BQ27x00_REG_TTF			0x18 //   BQ27x00_TTF_L OK
#define BQ27x00_REG_TTECP		0x26 //   BQ27x00_TTECP_L OK

#define BQ27000_REG_RSOC		0x0B //   BQ27x00_RSOC OK
#define BQ27000_FLAG_CHGS		BIT(7) // BQ27000_STATUS_CHGS 

#define BQ27500_REG_SOC			0x2c //   BQ27x00_CSOC OK
#define BQ27500_FLAG_DSC		BIT(0) 
#define BQ27500_FLAG_FC			BIT(9)
#else

#define DRIVER_VERSION			"1.1.1"
/*imported from old driver */
#define BQ27500_FLAG_DSC		BIT(0) /* Discharging flag */
#define BQ27500_FLAG_FC			BIT(9) /* Full capacity */

enum bq27x00_regs {
	/* RAM regs */
		/* read-write after this */
	BQ27x00_CTRL = 0, /* Device Control Register */
	BQ27x00_MODE, /* Device Mode Register */
	BQ27x00_AR_L, /* At-Rate H L */
	BQ27x00_AR_H,
		/* read-only after this */
	BQ27x00_ARTTE_L, /* At-Rate Time To Empty H L */
	BQ27x00_ARTTE_H,
	BQ27x00_TEMP_L, /* Reported Temperature H L */ 
	BQ27x00_TEMP_H,
	BQ27x00_VOLT_L, /* Reported Voltage H L */
	BQ27x00_VOLT_H,
	BQ27x00_FLAGS, /* Status Flags */
	BQ27x00_RSOC, /* Relative State of Charge */
	BQ27x00_NAC_L, /* Nominal Available Capacity H L */
	BQ27x00_NAC_H,
	BQ27x00_CACD_L, /* Discharge Compensated H L */
	BQ27x00_CACD_H,
	BQ27x00_CACT_L, /* Temperature Compensated H L */
	BQ27x00_CACT_H,
	BQ27x00_LMD_L, /* Last measured discharge H L */
	BQ27x00_LMD_H,
	BQ27x00_AI_L, /* Average Current H L */
	BQ27x00_AI_H,
	BQ27x00_TTE_L, /* Time to Empty H L */
	BQ27x00_TTE_H,
	BQ27x00_TTF_L, /* Time to Full H L */
	BQ27x00_TTF_H,
	BQ27x00_SI_L, /* Standby Current H L */
	BQ27x00_SI_H,
	BQ27x00_STTE_L, /* Standby Time To Empty H L */
	BQ27x00_STTE_H,
	BQ27x00_MLI_L, /* Max Load Current H L */
	BQ27x00_MLI_H,
	BQ27x00_MLTTE_L, /* Max Load Time To Empty H L */
	BQ27x00_MLTTE_H,
	BQ27x00_SAE_L, /* Available Energy H L */
	BQ27x00_SAE_H,
	BQ27x00_AP_L, /* Available Power H L */
	BQ27x00_AP_H,
	BQ27x00_TTECP_L, /* Time to Empty at Constant Power H L */
	BQ27x00_TTECP_H,
	BQ27x00_CYCL_L, /* Cycle count since learning cycle H L */
	BQ27x00_CYCL_H,
	BQ27x00_CYCT_L, /* Cycle Count Total H L */
	BQ27x00_CYCT_H,
	BQ27x00_CSOC, /* Compensated State Of Charge */
	/* EEPROM regs */
		/* read-write after this */
	BQ27x00_EE_EE_EN = 0x6e, /* EEPROM Program Enable */
	BQ27x00_EE_ILMD = 0x76, /* Initial Last Measured Discharge High Byte */
	BQ27x00_EE_SEDVF, /* Scaled EDVF Threshold */
	BQ27x00_EE_SEDV1, /* Scaled EDV1 Threshold */
	BQ27x00_EE_ISLC, /* Initial Standby Load Current */
	BQ27x00_EE_DMFSD, /* Digital Magnitude Filter and Self Discharge */
	BQ27x00_EE_TAPER, /* Aging Estimate Enable, Charge Termination Taper */
	BQ27x00_EE_PKCFG, /* Pack Configuration Values */
	BQ27x00_EE_IMLC, /* Initial Max Load Current or ID #3 */
	BQ27x00_EE_DCOMP, /* Discharge rate compensation constants or ID #2 */
	BQ27x00_EE_TCOMP, /* Temperature Compensation constants or ID #1 */
};

enum bq27000_status_flags {
	BQ27000_STATUS_CHGS = 0x80, /* 1 = being charged */
	BQ27000_STATUS_NOACT = 0x40, /* 1 = no activity */
	BQ27000_STATUS_IMIN = 0x20, /* 1 = Lion taper current mode */
	BQ27000_STATUS_CI = 0x10, /* 1 = capacity likely  innacurate */
	BQ27000_STATUS_CALIP = 0x08, /* 1 = calibration in progress */
	BQ27000_STATUS_VDQ = 0x04, /* 1 = capacity should be accurate */
	BQ27000_STATUS_EDV1 = 0x02, /* 1 = end of discharge.. <6% left */
	BQ27000_STATUS_EDVF = 0x01, /* 1 = no, it's really empty now */
};

#endif
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

enum bq27x00_chip { BQ27000, BQ27500 };

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;
	enum bq27x00_chip	chip;

	struct i2c_client	*client;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

/*
 * Common code for BQ27x00 devices
 */

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27x00_read(BQ27x00_TEMP_L, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	if (di->chip == BQ27500)
		return temp - 2731;
	else
		return ((temp >> 2) - 273) * 10;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27x00_read(BQ27x00_VOLT_L, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
	int ret;
	int curr = 0;
	int flags = 0;

	ret = bq27x00_read(BQ27x00_AI_L, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	if (di->chip == BQ27500) {
		/* bq27500 returns signed value */
		curr = (int)(s16)curr;
	} else {
		ret = bq27x00_read(BQ27x00_FLAGS, &flags, 0, di);
		if (ret < 0) {
			dev_err(di->dev, "error reading flags\n");
			return 0;
		}
		if (flags & BQ27000_STATUS_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}
	}

	return curr * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
	int ret;
	int rsoc = 0;

	if (di->chip == BQ27500)
		ret = bq27x00_read(BQ27x00_CSOC, &rsoc, 0, di);
	else
		ret = bq27x00_read(BQ27x00_RSOC, &rsoc, 1, di);
	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return rsoc;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
				  union power_supply_propval *val)
{
	int flags = 0;
	int status;
	int ret;

	ret = bq27x00_read(BQ27x00_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	if (di->chip == BQ27500) {
		if (flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (flags & BQ27500_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (flags & BQ27000_STATUS_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	val->intval = status;
	return 0;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_time(struct bq27x00_device_info *di, int reg,
				union power_supply_propval *val)
{
	int tval = 0;
	int ret;

	ret = bq27x00_read(reg, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}

	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;
	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27x00_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27x00_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27x00_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27x00_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_battery_time(di, BQ27x00_TTE_L, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_battery_time(di, BQ27x00_TTECP_L, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_battery_time(di, BQ27x00_TTF_L, val);
		break; 
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = NULL;
}

/*
 * i2c specific code
 */

static int bq27x00_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int num;
	int retval = 0;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;
	di->chip = id->driver_data;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = &bq27x00_read_i2c;
	di->bus = bus;
	di->client = client;

	bq27x00_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27500", BQ27500 },
	{},
};

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27x00-battery",
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

struct bq27200_platform_data {
       int dummy;
};

static struct bq27200_platform_data bq27200_config = {
       .dummy = 0,
};

static struct i2c_board_info rx51_camera_board_info_2 = 
       {
               I2C_BOARD_INFO("bq27200", 0x55),
               .platform_data = &bq27200_config,
       };

static struct i2c_client *client;


static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27x00_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 driver\n");
	client = i2c_new_device(i2c_get_adapter(2), &rx51_camera_board_info_2);
	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
