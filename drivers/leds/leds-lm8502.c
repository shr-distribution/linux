// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM8502 LED-class driver (MFD child)
 *
 * Copyright (C) 2008 Palm Inc.
 * Copyright (C) 2024 Christophe Chapuis <chris.music.music@gmail.com>
 * Copyright (C) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * Per-subsystem child of the LM8502 MFD core (drivers/mfd/lm8502.c).
 * Handles only LED-class registration and brightness writes for outputs
 * D1..D10. Chip-level resources (regmap, regulator, enable GPIO, reset
 * and init sequencing) live in the parent.
 *
 * Bound by name "lm8502-leds" from the MFD core's mfd_cell[]; the
 * matching DT node is the "leds" subnode of "ti,lm8502" with compatible
 * "ti,lm8502-leds". Per-LED nodes appear as children of that subnode,
 * addressed by their D-number (reg = <0..9>).
 */

#include <linux/leds.h>
#include <linux/mfd/lm8502.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct lm8502_led {
	struct led_classdev cdev;
	struct lm8502_leds *parent;
	u8 current_reg;
	u8 control_reg;
	u8 led_num;
};

struct lm8502_leds {
	struct lm8502 *chip;
	struct device *dev;
	struct lm8502_led leds[LM8502_MAX_LEDS];
	int num_leds;
};

static int lm8502_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct lm8502_led *led = container_of(cdev, struct lm8502_led, cdev);
	struct lm8502 *chip = led->parent->chip;
	int ret = 0;

	mutex_lock(&chip->lock);
	if (!chip->suspended)
		ret = regmap_write(chip->regmap, led->current_reg, brightness);
	mutex_unlock(&chip->lock);

	return ret;
}

/*
 * Program the per-channel control register: MAX_CURRENT field = 2 (cap
 * at 9 mA on this chip), DIRECT-mapping mode so brightness writes to
 * D<n>_CURRENT_CTRL are honoured immediately. Called once per LED in
 * probe; the parent's chip_init has already brought the chip up and
 * verified bidirectional I2C, so plain regmap_write() is safe here.
 */
static int lm8502_led_program_control(struct lm8502_leds *priv,
				      struct lm8502_led *led, u32 max_ua)
{
	struct lm8502 *chip = priv->chip;
	u8 max_current;
	u8 val;
	int ret;

	if (max_ua <= 3000)
		max_current = 0;
	else if (max_ua <= 6000)
		max_current = 1;
	else if (max_ua <= 9000)
		max_current = 2;
	else
		max_current = 3;

	val = (max_current << LM8502_LED_MAX_CURRENT_SHIFT) |
	      LM8502_LED_MAPPING_DIRECT;

	mutex_lock(&chip->lock);
	ret = regmap_write(chip->regmap, led->control_reg, val);
	if (ret == 0)
		ret = regmap_write(chip->regmap, led->current_reg, 0);
	mutex_unlock(&chip->lock);

	return ret;
}

static int lm8502_leds_parse(struct lm8502_leds *priv)
{
	struct device *dev = priv->dev;
	struct device_node *child;
	int i = 0;
	int ret;

	for_each_available_child_of_node(dev->of_node, child) {
		struct lm8502_led *led;
		const char *label;
		u32 reg, max_ua;

		if (i >= LM8502_MAX_LEDS) {
			dev_warn(dev, "too many LEDs defined, max %d\n",
				 LM8502_MAX_LEDS);
			of_node_put(child);
			break;
		}

		ret = of_property_read_u32(child, "reg", &reg);
		if (ret || reg >= LM8502_MAX_LEDS) {
			of_node_put(child);
			return dev_err_probe(dev, ret ? ret : -EINVAL,
					     "invalid LED reg property\n");
		}

		led = &priv->leds[i];
		led->parent = priv;
		led->led_num = reg;
		led->current_reg = LM8502_D1_CURRENT_CTRL + reg;
		led->control_reg = LM8502_D1_CONTROL + reg;

		ret = of_property_read_string(child, "label", &label);
		if (ret)
			label = child->name;

		led->cdev.name = label;
		led->cdev.brightness_set_blocking = lm8502_brightness_set;
		led->cdev.max_brightness = 255;

		ret = of_property_read_u32(child, "led-max-microamp", &max_ua);
		if (ret)
			max_ua = 9000;	/* per-channel chip default */

		ret = lm8502_led_program_control(priv, led, max_ua);
		if (ret) {
			of_node_put(child);
			return dev_err_probe(dev, ret,
					     "failed to program LED %s\n",
					     label);
		}

		ret = devm_led_classdev_register(dev, &led->cdev);
		if (ret) {
			of_node_put(child);
			return dev_err_probe(dev, ret,
					     "failed to register LED %s\n",
					     label);
		}

		i++;
	}

	priv->num_leds = i;
	return 0;
}

static int lm8502_leds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lm8502_leds *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip = dev_get_drvdata(dev->parent);
	if (!priv->chip)
		return dev_err_probe(dev, -ENODEV, "no parent lm8502 chip\n");

	priv->dev = dev;

	return lm8502_leds_parse(priv);
}

static const struct of_device_id lm8502_leds_of_match[] = {
	{ .compatible = "ti,lm8502-leds" },
	{}
};
MODULE_DEVICE_TABLE(of, lm8502_leds_of_match);

static struct platform_driver lm8502_leds_driver = {
	.driver = {
		.name = "lm8502-leds",
		.of_match_table = lm8502_leds_of_match,
	},
	.probe = lm8502_leds_probe,
};
module_platform_driver(lm8502_leds_driver);

MODULE_ALIAS("platform:lm8502-leds");
MODULE_DESCRIPTION("TI LM8502 LED-class driver (MFD child)");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
