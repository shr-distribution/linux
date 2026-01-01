// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM3528 Backlight Driver
 *
 * Copyright (C) 2010 HP Inc.
 * Copyright (C) 2026 webOS Community
 *
 * Based on the original Palm/HP webOS driver.
 *
 * The LM3528 is a dual-channel white LED driver with I2C interface,
 * commonly used for LCD backlight in smartphones.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* Register addresses */
#define LM3528_REG_GP			0x10	/* General Purpose */
#define LM3528_REG_BMAIN		0xa0	/* Brightness Main */

/* GP register bits */
#define LM3528_GP_ENM			BIT(0)	/* Main LED Enable */
#define LM3528_GP_ENS			BIT(1)	/* Sub LED Enable */
#define LM3528_GP_UNI			BIT(2)	/* Unison Mode */
#define LM3528_GP_ENABLE		(LM3528_GP_ENM | LM3528_GP_ENS | LM3528_GP_UNI)

/* Max brightness value from lookup table */
#define LM3528_MAX_BRIGHTNESS		100

/*
 * Brightness lookup table mapping percentage (0-100) to PWM register value.
 * This provides a perceptually linear brightness curve.
 */
static const u8 lm3528_brightness_table[] = {
	0x00,
	0x43, 0x46, 0x4a, 0x4d, 0x50,
	0x51, 0x52, 0x53, 0x54, 0x54,
	0x56, 0x57, 0x59, 0x5a, 0x5c,
	0x5c, 0x5d, 0x5d, 0x5e, 0x5e,
	0x5f, 0x60, 0x61, 0x62, 0x63,
	0x63, 0x63, 0x64, 0x64, 0x64,
	0x65, 0x66, 0x66, 0x67, 0x68,
	0x68, 0x68, 0x69, 0x69, 0x69,
	0x69, 0x6a, 0x6a, 0x6b, 0x6c,
	0x6c, 0x6c, 0x6c, 0x6d, 0x6d,
	0x6d, 0x6d, 0x6e, 0x6e, 0x6f,
	0x6f, 0x6f, 0x6f, 0x6f, 0x70,
	0x70, 0x70, 0x71, 0x71, 0x71,
	0x71, 0x72, 0x72, 0x72, 0x72,
	0x72, 0x73, 0x73, 0x73, 0x74,
	0x74, 0x74, 0x74, 0x74, 0x74,
	0x74, 0x75, 0x75, 0x75, 0x75,
	0x76, 0x76, 0x76, 0x76, 0x76,
	0x76, 0x76, 0x77, 0x77, 0x77,
	0x77, 0x77, 0x77, 0x78, 0x78,
};

struct lm3528 {
	struct i2c_client *client;
	struct regmap *regmap;
	struct backlight_device *backlight;
	u32 default_brightness;
};

static const struct regmap_config lm3528_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xa0,
};

static int lm3528_backlight_update_status(struct backlight_device *bl)
{
	struct lm3528 *lm = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);
	u8 pwm_val;
	int ret;

	pwm_val = lm3528_brightness_table[brightness];

	if (brightness) {
		/* Enable the LED driver */
		ret = regmap_write(lm->regmap, LM3528_REG_GP, LM3528_GP_ENABLE);
		if (ret)
			return ret;

		/* Set brightness */
		ret = regmap_write(lm->regmap, LM3528_REG_BMAIN, pwm_val);
	} else {
		/* Turn off brightness first, then disable */
		ret = regmap_write(lm->regmap, LM3528_REG_BMAIN, 0);
		if (ret)
			return ret;

		ret = regmap_write(lm->regmap, LM3528_REG_GP, 0);
	}

	return ret;
}

static const struct backlight_ops lm3528_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= lm3528_backlight_update_status,
};

static int lm3528_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct lm3528 *lm;
	int ret;

	lm = devm_kzalloc(dev, sizeof(*lm), GFP_KERNEL);
	if (!lm)
		return -ENOMEM;

	lm->client = client;

	lm->regmap = devm_regmap_init_i2c(client, &lm3528_regmap_config);
	if (IS_ERR(lm->regmap))
		return dev_err_probe(dev, PTR_ERR(lm->regmap),
				     "failed to init regmap\n");

	/* Get default brightness from DT or use 50% */
	if (device_property_read_u32(dev, "default-brightness",
				     &lm->default_brightness))
		lm->default_brightness = LM3528_MAX_BRIGHTNESS / 2;

	if (lm->default_brightness > LM3528_MAX_BRIGHTNESS)
		lm->default_brightness = LM3528_MAX_BRIGHTNESS;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = LM3528_MAX_BRIGHTNESS;
	props.brightness = lm->default_brightness;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, lm,
					    &lm3528_backlight_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");

	lm->backlight = bl;

	/* Enable and set initial brightness */
	ret = regmap_write(lm->regmap, LM3528_REG_GP, LM3528_GP_ENABLE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable device\n");

	backlight_update_status(bl);

	i2c_set_clientdata(client, lm);

	dev_info(dev, "LM3528 backlight registered\n");

	return 0;
}

static void lm3528_remove(struct i2c_client *client)
{
	struct lm3528 *lm = i2c_get_clientdata(client);

	lm->backlight->props.brightness = 0;
	backlight_update_status(lm->backlight);
}

static void lm3528_shutdown(struct i2c_client *client)
{
	struct lm3528 *lm = i2c_get_clientdata(client);

	/* Ensure backlight is off on shutdown */
	regmap_write(lm->regmap, LM3528_REG_BMAIN, 0);
	regmap_write(lm->regmap, LM3528_REG_GP, 0);
}

static const struct of_device_id lm3528_of_match[] = {
	{ .compatible = "ti,lm3528" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3528_of_match);

static const struct i2c_device_id lm3528_ids[] = {
	{ "lm3528" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3528_ids);

static struct i2c_driver lm3528_driver = {
	.driver = {
		.name = "lm3528",
		.of_match_table = lm3528_of_match,
	},
	.probe = lm3528_probe,
	.remove = lm3528_remove,
	.shutdown = lm3528_shutdown,
	.id_table = lm3528_ids,
};
module_i2c_driver(lm3528_driver);

MODULE_DESCRIPTION("TI LM3528 Backlight Driver");
MODULE_AUTHOR("HP Inc.");
MODULE_AUTHOR("webOS Community");
MODULE_LICENSE("GPL");
