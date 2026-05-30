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
 * D1..D10. Chip-level resources (regmap, regulator, enable GPIO, deferred
 * reset/configuration sequence) live in the parent.
 *
 * Bound by name "lm8502-leds" from the MFD core's mfd_cell[]; the
 * matching DT node is the "leds" subnode of "ti,lm8502" with
 * compatible "ti,lm8502-leds". Per-LED nodes appear as children of
 * that subnode, addressed by their D-number (reg = <0..9>).
 */

#include <linux/leds.h>
#include <linux/mfd/lm8502.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* LED control-register bits */
#define LM8502_LED_MAPPING_MASK		0x03
#define LM8502_LED_MAPPING_DIRECT	0x00
#define LM8502_LED_MAX_CURRENT_MASK	0x18
#define LM8502_LED_MAX_CURRENT_SHIFT	3

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
	struct delayed_work init_work;  /* defer LED-side register init until
					 * after the parent's chip_init landed */
};

static int lm8502_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct lm8502_led *led = container_of(cdev, struct lm8502_led, cdev);
	struct lm8502 *chip = led->parent->chip;

	if (!lm8502_chip_ready(chip))
		return -EAGAIN;

	return regmap_write(chip->regmap, led->current_reg, brightness);
}

/*
 * Initialize the per-channel LED registers AFTER the parent has run
 * chip_init. This sets D1..D10 CONTROL = 0x10 (MAX_CURRENT field = 2 =
 * 9 mA, DIRECT mapping mode) and zeros the current registers so LEDs
 * are off at start. Matches the webOS init sequence.
 */
static void lm8502_leds_init_work(struct work_struct *w)
{
	struct lm8502_leds *priv =
		container_of(to_delayed_work(w), struct lm8502_leds, init_work);
	struct lm8502 *chip = priv->chip;
	int i;

	if (!lm8502_chip_ready(chip)) {
		/* Parent's chip_init hasn't completed yet -- check back. */
		schedule_delayed_work(&priv->init_work, msecs_to_jiffies(500));
		return;
	}

	mutex_lock(&chip->lock);
	for (i = 0; i < LM8502_MAX_LEDS; i++) {
		regmap_write(chip->regmap, LM8502_D1_CONTROL + i, 0x10);
		regmap_write(chip->regmap, LM8502_D1_CURRENT_CTRL + i, 0);
	}
	mutex_unlock(&chip->lock);
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
		u32 reg;
		u32 max_ua;

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
		if (!ret) {
			u8 max_current;

			if (max_ua <= 3000)
				max_current = 0;
			else if (max_ua <= 6000)
				max_current = 1;
			else if (max_ua <= 9000)
				max_current = 2;
			else
				max_current = 3;

			regmap_update_bits(priv->chip->regmap, led->control_reg,
					   LM8502_LED_MAX_CURRENT_MASK,
					   max_current <<
					   LM8502_LED_MAX_CURRENT_SHIFT);
		}

		ret = devm_led_classdev_register(dev, &led->cdev);
		if (ret) {
			dev_err(dev, "Failed to register LED %s: %d\n",
				label, ret);
			of_node_put(child);
			return ret;
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
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip = dev_get_drvdata(dev->parent);
	if (!priv->chip)
		return dev_err_probe(dev, -ENODEV, "no parent lm8502 chip\n");

	priv->dev = dev;
	INIT_DELAYED_WORK(&priv->init_work, lm8502_leds_init_work);

	ret = lm8502_leds_parse(priv);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	/*
	 * Schedule LED-side register init for ~1 s after the parent's
	 * 3 s chip_init window. The lm8502_leds_init_work() handler will
	 * re-arm itself if the parent isn't ready yet.
	 */
	schedule_delayed_work(&priv->init_work, msecs_to_jiffies(4000));

	dev_info(dev, "%d LED(s) registered\n", priv->num_leds);
	return 0;
}

static void lm8502_leds_remove(struct platform_device *pdev)
{
	struct lm8502_leds *priv = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&priv->init_work);
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
	.remove = lm8502_leds_remove,
};
module_platform_driver(lm8502_leds_driver);

MODULE_ALIAS("platform:lm8502-leds");
MODULE_DESCRIPTION("TI LM8502 LED-class driver (MFD child)");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
