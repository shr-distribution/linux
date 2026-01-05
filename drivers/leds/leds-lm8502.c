// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM8502 LED Driver
 *
 * Copyright (C) 2008 Palm Inc.
 * Copyright (C) 2024 Christophe Chapuis <chris.music.music@gmail.com>
 * Copyright (C) 2024 Herman van Hazendonk <github.com@herrie.org>
 *
 * Original authors:
 *   Kevin McCray (kevin.mccray@palm.com)
 *   Brian Xiong (brian.xiong@palm.com)
 *   Rajat Gupta (rajat.gupta@palm.com)
 *   Amon Xie (amon.xie@palm.com)
 *
 * Modernized for mainline Linux with device tree support.
 *
 * The LM8502 is a combo LED controller + vibrator + camera flash device.
 * It has 10 LED outputs with programmable current control, two programmable
 * sequencing engines, haptic feedback controller, and flash/torch modes.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>

/* LM8502 Register Map */
#define LM8502_ENGINE_CNTRL1		0x00
#define LM8502_ENGINE_CNTRL2		0x01
#define LM8502_GROUP_FADING1		0x02
#define LM8502_GROUP_FADING2		0x03
#define LM8502_GROUP_FADING3		0x04

#define LM8502_D1_CONTROL		0x06
#define LM8502_D2_CONTROL		0x07
#define LM8502_D3_CONTROL		0x08
#define LM8502_D4_CONTROL		0x09
#define LM8502_D5_CONTROL		0x0A
#define LM8502_D6_CONTROL		0x0B
#define LM8502_D7_CONTROL		0x0C
#define LM8502_D8_CONTROL		0x0D
#define LM8502_D9_CONTROL		0x0E
#define LM8502_D10_CONTROL		0x0F

#define LM8502_HAPTIC_CONTROL		0x10

#define LM8502_HAPTIC_FEEDBACK_CTRL	0x21
#define LM8502_HAPTIC_PWM_DUTY_CYCLE	0x22

#define LM8502_D1_CURRENT_CTRL		0x26
#define LM8502_D2_CURRENT_CTRL		0x27
#define LM8502_D3_CURRENT_CTRL		0x28
#define LM8502_D4_CURRENT_CTRL		0x29
#define LM8502_D5_CURRENT_CTRL		0x2A
#define LM8502_D6_CURRENT_CTRL		0x2B
#define LM8502_D7_CURRENT_CTRL		0x2C
#define LM8502_D8_CURRENT_CTRL		0x2D
#define LM8502_D9_CURRENT_CTRL		0x2E
#define LM8502_D10_CURRENT_CTRL		0x2F

#define LM8502_MISC			0x36
#define LM8502_ENGINE1_PC		0x37
#define LM8502_ENGINE2_PC		0x38
#define LM8502_STATUS			0x3A
#define LM8502_INT			0x3B
#define LM8502_RESET			0x3D

#define LM8502_GROUP_FADER1		0x48
#define LM8502_GROUP_FADER2		0x49
#define LM8502_GROUP_FADER3		0x4A

#define LM8502_ENG1_PROG_START_ADDR	0x4C
#define LM8502_ENG2_PROG_START_ADDR	0x4D
#define LM8502_PROG_MEM_PAGE_SELECT	0x4F

#define LM8502_PROG_MEM_START		0x50
#define LM8502_PROG_MEM_END		0x6F

#define LM8502_TORCH_BRIGHTNESS		0xA0
#define LM8502_FLASH_BRIGHTNESS		0xB0
#define LM8502_FLASH_DURATION		0xC0

/* Register bit definitions */
#define LM8502_CHIP_EN			BIT(6)
#define LM8502_MISC_POWER_SAVE		BIT(5)
#define LM8502_MISC_BOOST_EN		BIT(3)
#define LM8502_MISC_PWM_INPUT		BIT(1)

/* Engine control bits */
#define LM8502_ENG1_SHIFT		4
#define LM8502_ENG2_SHIFT		2
#define LM8502_ENG_HOLD			0
#define LM8502_ENG_STEP			1
#define LM8502_ENG_FREERUN		2
#define LM8502_ENG_EXECONCE		3

#define LM8502_ENG_DISABLE		0
#define LM8502_ENG_LOAD			1
#define LM8502_ENG_RUN			2
#define LM8502_ENG_HALT			3

/* LED control bits */
#define LM8502_LED_HW_GRP_MASK		0xC0
#define LM8502_LED_HW_GRP_SHIFT		6
#define LM8502_LED_MAX_CURRENT_MASK	0x18
#define LM8502_LED_MAX_CURRENT_SHIFT	3

/* Maximum number of LEDs */
#define LM8502_MAX_LEDS			10

struct lm8502_led {
	struct led_classdev cdev;
	struct lm8502_data *priv;
	u8 current_reg;
	u8 control_reg;
	u8 led_num;
};

struct lm8502_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct regulator *vcc;
	struct mutex lock;
	struct gpio_desc *enable_gpio;
	struct lm8502_led leds[LM8502_MAX_LEDS];
	int num_leds;
	bool suspended;
};

static const struct regmap_config lm8502_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xF0,
};

static int lm8502_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct lm8502_led *led = container_of(cdev, struct lm8502_led, cdev);
	struct lm8502_data *priv = led->priv;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_write(priv->regmap, led->current_reg, brightness);
	mutex_unlock(&priv->lock);

	return ret;
}

static int lm8502_chip_init(struct lm8502_data *priv)
{
	int ret;

	/* Reset the chip */
	ret = regmap_write(priv->regmap, LM8502_RESET, 0xFF);
	if (ret)
		return ret;

	msleep(50);

	/* Enable the chip */
	ret = regmap_update_bits(priv->regmap, LM8502_ENGINE_CNTRL1,
				 LM8502_CHIP_EN, LM8502_CHIP_EN);
	if (ret)
		return ret;

	/* Configure power saving, boost enable, and PWM input */
	ret = regmap_update_bits(priv->regmap, LM8502_MISC,
				 LM8502_MISC_POWER_SAVE | LM8502_MISC_BOOST_EN |
				 LM8502_MISC_PWM_INPUT,
				 LM8502_MISC_POWER_SAVE | LM8502_MISC_BOOST_EN |
				 LM8502_MISC_PWM_INPUT);

	return ret;
}

static int lm8502_parse_dt(struct lm8502_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct device_node *child;
	int ret;
	int i = 0;

	for_each_available_child_of_node(dev->of_node, child) {
		struct lm8502_led *led;
		u32 reg;
		const char *label;

		if (i >= LM8502_MAX_LEDS) {
			dev_warn(dev, "Too many LEDs defined, max %d\n",
				 LM8502_MAX_LEDS);
			of_node_put(child);
			break;
		}

		ret = of_property_read_u32(child, "reg", &reg);
		if (ret || reg >= LM8502_MAX_LEDS) {
			dev_err(dev, "Invalid LED reg property\n");
			of_node_put(child);
			return ret ? ret : -EINVAL;
		}

		led = &priv->leds[i];
		led->priv = priv;
		led->led_num = reg;
		led->current_reg = LM8502_D1_CURRENT_CTRL + reg;
		led->control_reg = LM8502_D1_CONTROL + reg;

		ret = of_property_read_string(child, "label", &label);
		if (ret)
			label = child->name;

		led->cdev.name = label;
		led->cdev.brightness_set_blocking = lm8502_brightness_set;
		led->cdev.max_brightness = 255;

		ret = of_property_read_u32(child, "led-max-microamp", &reg);
		if (!ret) {
			/* Convert microamp to register value (0-3) */
			u8 max_current;

			if (reg <= 3000)
				max_current = 0;
			else if (reg <= 6000)
				max_current = 1;
			else if (reg <= 9000)
				max_current = 2;
			else
				max_current = 3;

			mutex_lock(&priv->lock);
			regmap_update_bits(priv->regmap, led->control_reg,
					   LM8502_LED_MAX_CURRENT_MASK,
					   max_current << LM8502_LED_MAX_CURRENT_SHIFT);
			mutex_unlock(&priv->lock);
		}

		ret = devm_led_classdev_register(dev, &led->cdev);
		if (ret) {
			dev_err(dev, "Failed to register LED %s\n", label);
			of_node_put(child);
			return ret;
		}

		i++;
	}

	priv->num_leds = i;
	return 0;
}

static int lm8502_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lm8502_data *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	mutex_init(&priv->lock);

	priv->regmap = devm_regmap_init_i2c(client, &lm8502_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	/* Get optional power supply */
	priv->vcc = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR(priv->vcc)) {
		ret = PTR_ERR(priv->vcc);
		if (ret == -ENODEV) {
			priv->vcc = NULL;
		} else {
			return dev_err_probe(dev, ret, "Failed to get vcc supply\n");
		}
	}

	/* Enable power supply if available */
	if (priv->vcc) {
		ret = regulator_enable(priv->vcc);
		if (ret) {
			dev_err(dev, "Failed to enable vcc supply: %d\n", ret);
			return ret;
		}
	}

	/* Get enable GPIO */
	priv->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(priv->enable_gpio)) {
		ret = PTR_ERR(priv->enable_gpio);
		dev_err(dev, "Failed to get enable GPIO: %d\n", ret);
		goto err_disable_vcc;
	}

	/* Enable the chip via GPIO if available */
	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, 1);

	i2c_set_clientdata(client, priv);

	ret = lm8502_chip_init(priv);
	if (ret) {
		dev_err(dev, "Failed to initialize chip: %d\n", ret);
		goto err_disable_vcc;
	}

	ret = lm8502_parse_dt(priv);
	if (ret) {
		dev_err(dev, "Failed to parse device tree: %d\n", ret);
		goto err_disable_vcc;
	}

	dev_info(dev, "LM8502 LED controller initialized with %d LEDs\n",
		 priv->num_leds);

	return 0;

err_disable_vcc:
	if (priv->vcc)
		regulator_disable(priv->vcc);
	return ret;
}

static void lm8502_remove(struct i2c_client *client)
{
	struct lm8502_data *priv = i2c_get_clientdata(client);

	/* Disable the chip */
	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, 0);

	/* Disable power supply */
	if (priv->vcc)
		regulator_disable(priv->vcc);
}

static int lm8502_suspend(struct device *dev)
{
	struct lm8502_data *priv = dev_get_drvdata(dev);
	int i;

	if (priv->suspended)
		return 0;

	/* Turn off all LEDs */
	mutex_lock(&priv->lock);
	for (i = 0; i < priv->num_leds; i++)
		regmap_write(priv->regmap, priv->leds[i].current_reg, 0);

	/* Disable the chip */
	regmap_update_bits(priv->regmap, LM8502_ENGINE_CNTRL1,
			   LM8502_CHIP_EN, 0);
	mutex_unlock(&priv->lock);

	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, 0);

	if (priv->vcc)
		regulator_disable(priv->vcc);

	priv->suspended = true;

	return 0;
}

static int lm8502_resume(struct device *dev)
{
	struct lm8502_data *priv = dev_get_drvdata(dev);
	int ret;

	if (!priv->suspended)
		return 0;

	if (priv->vcc) {
		ret = regulator_enable(priv->vcc);
		if (ret) {
			dev_err(dev, "Failed to enable vcc supply: %d\n", ret);
			return ret;
		}
	}

	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, 1);

	ret = lm8502_chip_init(priv);
	if (ret) {
		dev_err(dev, "Failed to reinitialize chip: %d\n", ret);
		return ret;
	}

	priv->suspended = false;

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(lm8502_pm_ops, lm8502_suspend, lm8502_resume);

static const struct of_device_id lm8502_of_match[] = {
	{ .compatible = "ti,lm8502" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm8502_of_match);

static const struct i2c_device_id lm8502_id[] = {
	{ "lm8502" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm8502_id);

static struct i2c_driver lm8502_driver = {
	.driver = {
		.name = "lm8502",
		.of_match_table = lm8502_of_match,
		.pm = pm_sleep_ptr(&lm8502_pm_ops),
	},
	.probe = lm8502_probe,
	.remove = lm8502_remove,
	.id_table = lm8502_id,
};
module_i2c_driver(lm8502_driver);

MODULE_DESCRIPTION("TI LM8502 LED Driver");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
