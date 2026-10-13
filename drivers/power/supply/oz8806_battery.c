// SPDX-License-Identifier: GPL-2.0
/*
 * O2Micro OZ8806 battery fuel gauge driver for the Unihertz Titan Pocket.
 *
 * Clean-room replacement for the Agold/O2Micro "coulomb" driver in the
 * stock kernel. Register layout, scaling and battery tables were recovered
 * from the stock 4.14.186 kernel binary (afe_read_* / bmu_* functions).
 *
 * The chip keeps a coulomb counter (CAR) that the stock driver uses as the
 * remaining capacity in mAh, so state of charge is CAR / design capacity,
 * re-anchored to 100% at end of charge and to 0% at end of discharge, and
 * seeded from the OCV table when the counter holds nothing sensible.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power/oz8806_battery.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

/* drivers/pinctrl/mediatek/pinctrl-paris.c */
extern int agold_gpio_get_in(unsigned gpio);

#define OZ8806_REG_CHIP_ID	0x00
#define OZ8806_REG_OP_CTRL	0x09
#define OZ8806_REG_TEMP		0x0c
#define OZ8806_REG_VOLT		0x0e
#define OZ8806_REG_OCV		0x10
#define OZ8806_REG_CURR		0x12
#define OZ8806_REG_CAR		0x14

#define OZ8806_OP_CTRL_SLEEP	BIT(6)
#define OZ8806_OP_CTRL_ACTIVE	0x20	/* single cell, awake */
#define OZ8806_OP_CTRL_PWRDOWN	0x60	/* single cell, sleep */

/* Values from the stock kernel's config_data (Titan Pocket battery) */
#define OZ8806_RSENSE_MOHM	10
#define OZ8806_CAR_LSB		5	/* uVh */
#define OZ8806_CURR_LSB		781	/* 7.8125 uV, x100 */
#define OZ8806_VOLT_LSB		250	/* 2.5 mV, x100 */
#define OZ8806_NTC_REF_MV	1800
#define OZ8806_DESIGN_MAH	4250
#define OZ8806_CV_MV		4400
#define OZ8806_EOC_MA		300
#define OZ8806_EOD_MV		3450

#define OZ8806_POLL_MS		5000
#define OZ8806_ANCHOR_POLLS	3	/* consecutive polls before re-anchoring */
#define OZ8806_IDLE_MA		20

struct oz8806_lut {
	int x;
	int y;
};

/* Open-circuit voltage (mV) -> state of charge (%) */
static const struct oz8806_lut oz8806_ocv[] = {
	{3104, 0}, {3472, 2}, {3583, 3}, {3658, 5}, {3685, 6}, {3687, 8},
	{3688, 9}, {3689, 11}, {3693, 12}, {3707, 14}, {3717, 16}, {3727, 17},
	{3735, 19}, {3741, 20}, {3746, 22}, {3753, 23}, {3759, 25}, {3765, 27},
	{3771, 28}, {3777, 30}, {3782, 31}, {3787, 33}, {3792, 34}, {3797, 36},
	{3802, 38}, {3807, 39}, {3813, 41}, {3819, 42}, {3827, 44}, {3834, 45},
	{3841, 47}, {3850, 48}, {3859, 50}, {3869, 52}, {3879, 53}, {3893, 55},
	{3911, 56}, {3929, 58}, {3946, 59}, {3960, 61}, {3974, 62}, {3987, 64},
	{3998, 66}, {4012, 67}, {4030, 69}, {4046, 70}, {4061, 72}, {4077, 73},
	{4094, 75}, {4110, 77}, {4128, 78}, {4145, 80}, {4162, 81}, {4180, 83},
	{4198, 84}, {4216, 86}, {4234, 88}, {4253, 89}, {4271, 91}, {4289, 92},
	{4307, 94}, {4326, 95}, {4343, 97}, {4362, 98}, {4386, 100},
};

/* NTC resistance (ohm) -> temperature (degC) */
static const struct oz8806_lut oz8806_ntc[] = {
	{721, 112}, {755, 107}, {908, 102}, {1024, 97}, {1158, 92},
	{1314, 87}, {1496, 82}, {1710, 77}, {1962, 72}, {2261, 67},
	{2615, 62}, {3039, 57}, {3547, 52}, {4160, 47}, {4903, 42},
	{5809, 37}, {6921, 32}, {8295, 27}, {10000, 22}, {12133, 17},
	{14820, 12}, {18234, 7}, {22594, 2}, {28223, -3}, {35548, -8},
	{45168, -13}, {57926, -18}, {75021, -23},
};

struct oz8806 {
	struct i2c_client *client;
	struct power_supply *psy;
	struct delayed_work work;
	struct mutex lock;

	u32 ntc_pullup_ohm;
	int volt_mv;
	int curr_ma;
	int car_mah;
	int temp_dc;	/* tenths of a degree C */
	int full_polls;
	int empty_polls;
	bool valid;
#ifdef CONFIG_BATTERY_OZ8806_MTK_FEED
	int fed_capacity;	/* last capacity the "battery" supply was told of */
#endif
};

#ifdef CONFIG_BATTERY_OZ8806_MTK_FEED
/* Set once the gauge is registered; the phone has one and it never goes away. */
static struct oz8806 *oz8806_feed;
#endif

static int oz8806_lut_interp(const struct oz8806_lut *t, int n, int x)
{
	int i;

	if (x <= t[0].x)
		return t[0].y;
	if (x >= t[n - 1].x)
		return t[n - 1].y;
	for (i = 1; i < n; i++) {
		if (x <= t[i].x)
			return t[i - 1].y + (x - t[i - 1].x) *
			       (t[i].y - t[i - 1].y) / (t[i].x - t[i - 1].x);
	}
	return t[n - 1].y;
}

static int oz8806_read_s16(struct oz8806 *oz, u8 reg, s16 *val)
{
	int i, ret;

	/* The stock driver retries every access; the bus is shared and noisy */
	for (i = 0; i < 8; i++) {
		ret = i2c_smbus_read_word_data(oz->client, reg);
		if (ret >= 0) {
			*val = (s16)ret;
			return 0;
		}
		msleep(5);
	}
	return ret;
}

static int oz8806_raw_to_mv(s16 raw)
{
	return (raw / 16) * OZ8806_VOLT_LSB / 100;
}

static int oz8806_write_car(struct oz8806 *oz, int mah)
{
	s16 raw = mah * OZ8806_RSENSE_MOHM / OZ8806_CAR_LSB;

	return i2c_smbus_write_word_data(oz->client, OZ8806_REG_CAR, (u16)raw);
}

static int oz8806_wake(struct oz8806 *oz)
{
	int ctrl = i2c_smbus_read_byte_data(oz->client, OZ8806_REG_OP_CTRL);

	if (ctrl < 0)
		return ctrl;
	if (ctrl & OZ8806_OP_CTRL_SLEEP)
		return i2c_smbus_write_byte_data(oz->client, OZ8806_REG_OP_CTRL,
						 OZ8806_OP_CTRL_ACTIVE);
	return 0;
}

static int oz8806_update(struct oz8806 *oz)
{
	s16 v, c, car, t;
	int ret, ntc_mv;

	ret = oz8806_read_s16(oz, OZ8806_REG_VOLT, &v);
	if (!ret)
		ret = oz8806_read_s16(oz, OZ8806_REG_CURR, &c);
	if (!ret)
		ret = oz8806_read_s16(oz, OZ8806_REG_CAR, &car);
	if (!ret)
		ret = oz8806_read_s16(oz, OZ8806_REG_TEMP, &t);
	if (ret) {
		dev_warn_ratelimited(&oz->client->dev, "read failed: %d\n", ret);
		return ret;
	}

	oz->volt_mv = oz8806_raw_to_mv(v);
	oz->curr_ma = c * OZ8806_CURR_LSB / OZ8806_RSENSE_MOHM / 100 / 4;
	oz->car_mah = car * OZ8806_CAR_LSB / OZ8806_RSENSE_MOHM;

	ntc_mv = oz8806_raw_to_mv(t);
	if (ntc_mv > 0 && ntc_mv < OZ8806_NTC_REF_MV) {
		int r = ntc_mv * oz->ntc_pullup_ohm / (OZ8806_NTC_REF_MV - ntc_mv);

		oz->temp_dc = 10 * oz8806_lut_interp(oz8806_ntc,
						     ARRAY_SIZE(oz8806_ntc), r);
	} else {
		oz->temp_dc = 250;	/* NTC open/shorted: stock assumes 25C */
	}

	/* End of charge: near CV voltage with taper current below threshold */
	if (oz->volt_mv >= OZ8806_CV_MV - 50 &&
	    oz->curr_ma > 0 && oz->curr_ma < OZ8806_EOC_MA)
		oz->full_polls++;
	else
		oz->full_polls = 0;

	/* End of discharge: under cut-off voltage while discharging */
	if (oz->volt_mv < OZ8806_EOD_MV && oz->curr_ma < 0)
		oz->empty_polls++;
	else
		oz->empty_polls = 0;

	if (oz->full_polls >= OZ8806_ANCHOR_POLLS &&
	    oz->car_mah < OZ8806_DESIGN_MAH) {
		oz8806_write_car(oz, OZ8806_DESIGN_MAH);
		oz->car_mah = OZ8806_DESIGN_MAH;
	} else if (oz->empty_polls >= OZ8806_ANCHOR_POLLS && oz->car_mah > 0) {
		oz8806_write_car(oz, 0);
		oz->car_mah = 0;
	} else if (oz->car_mah > OZ8806_DESIGN_MAH) {
		oz8806_write_car(oz, OZ8806_DESIGN_MAH);
		oz->car_mah = OZ8806_DESIGN_MAH;
	}

	oz->valid = true;
	return 0;
}

static void oz8806_seed_car(struct oz8806 *oz)
{
	s16 car, ocv;
	int mah, soc;

	if (oz8806_read_s16(oz, OZ8806_REG_CAR, &car))
		return;
	mah = car * OZ8806_CAR_LSB / OZ8806_RSENSE_MOHM;
	if (mah > 0 && mah <= OZ8806_DESIGN_MAH + OZ8806_DESIGN_MAH / 20) {
		dev_info(&oz->client->dev, "using retained capacity %d mAh\n", mah);
		return;
	}

	if (oz8806_read_s16(oz, OZ8806_REG_OCV, &ocv))
		return;
	soc = oz8806_lut_interp(oz8806_ocv, ARRAY_SIZE(oz8806_ocv),
				oz8806_raw_to_mv(ocv));
	mah = OZ8806_DESIGN_MAH * soc / 100;
	dev_info(&oz->client->dev, "seeding capacity from OCV %d mV: %d%% (%d mAh)\n",
		 oz8806_raw_to_mv(ocv), soc, mah);
	oz8806_write_car(oz, mah);
}

static int oz8806_capacity(struct oz8806 *oz)
{
	return clamp(oz->car_mah * 100 / OZ8806_DESIGN_MAH, 0, 100);
}

#ifdef CONFIG_BATTERY_OZ8806_MTK_FEED
/*
 * As on stock, the phone has one battery supply: MediaTek's "battery", which
 * reports its capacity and charge from this gauge (mtk_battery.c asks through
 * oz8806_battery_read()). Tell that supply when the percentage moves, so user
 * space hears of it from the supply it is listening to.
 */
int oz8806_battery_read(struct oz8806_reading *r)
{
	struct oz8806 *oz = READ_ONCE(oz8806_feed);

	if (!oz)
		return -ENODEV;

	mutex_lock(&oz->lock);
	if (!oz->valid) {
		mutex_unlock(&oz->lock);
		return -EAGAIN;
	}
	r->capacity = oz8806_capacity(oz);
	r->charge_now_mah = max(oz->car_mah, 0);
	r->charge_full_mah = OZ8806_DESIGN_MAH;
	mutex_unlock(&oz->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(oz8806_battery_read);

static void oz8806_feed_changed(struct oz8806 *oz)
{
	struct power_supply *bat;
	int cap;

	mutex_lock(&oz->lock);
	cap = oz->valid ? oz8806_capacity(oz) : -1;
	mutex_unlock(&oz->lock);
	if (cap < 0 || cap == oz->fed_capacity)
		return;

	bat = power_supply_get_by_name("battery");
	if (!bat)
		return;
	oz->fed_capacity = cap;
	power_supply_changed(bat);
	power_supply_put(bat);
}
#else
static void oz8806_feed_changed(struct oz8806 *oz)
{
}
#endif

static void oz8806_work(struct work_struct *work)
{
	struct oz8806 *oz = container_of(work, struct oz8806, work.work);

	mutex_lock(&oz->lock);
	oz8806_update(oz);
	mutex_unlock(&oz->lock);

	power_supply_changed(oz->psy);
	oz8806_feed_changed(oz);
	schedule_delayed_work(&oz->work, msecs_to_jiffies(OZ8806_POLL_MS));
}

static int oz8806_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct oz8806 *oz = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&oz->lock);
	if (!oz->valid) {
		mutex_unlock(&oz->lock);
		return -EAGAIN;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (oz->curr_ma > OZ8806_IDLE_MA)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (oz->curr_ma < -OZ8806_IDLE_MA)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (oz8806_capacity(oz) >= 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = oz->volt_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = oz->curr_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = oz8806_capacity(oz);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = max(oz->car_mah, 0) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = OZ8806_DESIGN_MAH * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = OZ8806_CV_MV * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = OZ8806_EOD_MV * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = oz->temp_dc;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&oz->lock);
	return ret;
}

static enum power_supply_property oz8806_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc oz8806_desc = {
	.name		= "oz8806-battery",
#ifdef CONFIG_BATTERY_OZ8806_MTK_FEED
	/*
	 * Not a second battery: user space lists every supply of type Battery
	 * and would show two. The readings stay here for inspection.
	 */
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
#else
	.type		= POWER_SUPPLY_TYPE_BATTERY,
#endif
	.properties	= oz8806_props,
	.num_properties	= ARRAY_SIZE(oz8806_props),
	.get_property	= oz8806_get_property,
};

static int oz8806_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct power_supply_config cfg = {};
	struct oz8806 *oz;
	int chip_id, ret;

	oz = devm_kzalloc(&client->dev, sizeof(*oz), GFP_KERNEL);
	if (!oz)
		return -ENOMEM;

	oz->client = client;
	mutex_init(&oz->lock);
	INIT_DELAYED_WORK(&oz->work, oz8806_work);
	i2c_set_clientdata(client, oz);

	/* Board strap: stock uses a 24k NTC pull-up when GPIO5 reads high */
	oz->ntc_pullup_ohm = agold_gpio_get_in(5) == 1 ? 24000 : 120000;
	of_property_read_u32(client->dev.of_node, "o2micro,ntc-pullup-ohm",
			     &oz->ntc_pullup_ohm);

	chip_id = i2c_smbus_read_byte_data(client, OZ8806_REG_CHIP_ID);
	if (chip_id < 0) {
		dev_err(&client->dev, "no response from gauge: %d\n", chip_id);
		return chip_id;
	}

	ret = oz8806_wake(oz);
	if (ret)
		dev_warn(&client->dev, "wake failed: %d\n", ret);
	oz8806_seed_car(oz);

	cfg.drv_data = oz;
	cfg.of_node = client->dev.of_node;
	oz->psy = devm_power_supply_register(&client->dev, &oz8806_desc, &cfg);
	if (IS_ERR(oz->psy))
		return PTR_ERR(oz->psy);

	mutex_lock(&oz->lock);
	oz8806_update(oz);
	mutex_unlock(&oz->lock);
	schedule_delayed_work(&oz->work, msecs_to_jiffies(OZ8806_POLL_MS));

	dev_info(&client->dev, "OZ8806 id 0x%02x: %d mV, %d mA, %d%%\n",
		 chip_id, oz->volt_mv, oz->curr_ma, oz8806_capacity(oz));
#ifdef CONFIG_BATTERY_OZ8806_MTK_FEED
	oz->fed_capacity = -1;
	WRITE_ONCE(oz8806_feed, oz);
#endif
	return 0;
}

static int oz8806_remove(struct i2c_client *client)
{
	struct oz8806 *oz = i2c_get_clientdata(client);

#ifdef CONFIG_BATTERY_OZ8806_MTK_FEED
	WRITE_ONCE(oz8806_feed, NULL);
#endif
	cancel_delayed_work_sync(&oz->work);
	return 0;
}

static void oz8806_shutdown(struct i2c_client *client)
{
	struct oz8806 *oz = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&oz->work);
	/* Keep coulomb counting but drop the chip to its low-power state */
	i2c_smbus_write_byte_data(client, OZ8806_REG_OP_CTRL,
				  OZ8806_OP_CTRL_PWRDOWN);
}

static int __maybe_unused oz8806_suspend(struct device *dev)
{
	struct oz8806 *oz = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&oz->work);
	return 0;
}

static int __maybe_unused oz8806_resume(struct device *dev)
{
	struct oz8806 *oz = dev_get_drvdata(dev);

	oz8806_wake(oz);
	schedule_delayed_work(&oz->work, 0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(oz8806_pm_ops, oz8806_suspend, oz8806_resume);

static const struct of_device_id oz8806_of_match[] = {
	{ .compatible = "mediatek,coulomb" },	/* node name used by stock dtbo */
	{ .compatible = "o2micro,oz8806" },
	{ }
};
MODULE_DEVICE_TABLE(of, oz8806_of_match);

static const struct i2c_device_id oz8806_id[] = {
	{ "oz8806", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, oz8806_id);

static struct i2c_driver oz8806_driver = {
	.driver = {
		.name		= "oz8806",
		.of_match_table	= oz8806_of_match,
		.pm		= &oz8806_pm_ops,
	},
	.probe		= oz8806_probe,
	.remove		= oz8806_remove,
	.shutdown	= oz8806_shutdown,
	.id_table	= oz8806_id,
};
module_i2c_driver(oz8806_driver);

MODULE_DESCRIPTION("O2Micro OZ8806 fuel gauge (Titan Pocket)");
MODULE_LICENSE("GPL v2");
