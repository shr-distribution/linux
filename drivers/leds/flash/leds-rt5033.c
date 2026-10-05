// SPDX-License-Identifier: GPL-2.0-only
/*
 * Richtek RT5033 PMIC flash LED driver
 *
 * The MFD driver has always registered an "rt5033-led" cell, but no driver
 * ever existed for it. Register layout and the value encodings below are taken
 * from Richtek's vendor driver (drivers/leds/rt5033_fled.c in Samsung's msm8916
 * trees) and cross-checked against the reset values a Galaxy A3 (2015) comes up
 * with: strobe timeout 0x0f -> 544 ms and strobe current 0x12 -> 500 mA both
 * match the vendor platform defaults for this board.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/led-class-flash.h>
#include <linux/mfd/rt5033-private.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* RT5033_REG_FLED_FUNCTION1 */
#define RT5033_FLED_FUNC1_FLED1_EN	BIT(0)
#define RT5033_FLED_FUNC1_FLED2_EN	BIT(1)
#define RT5033_FLED_FUNC1_STROBE_MODE	BIT(2)	/* 0: torch, 1: flash */
/*
 * Hands the output over to the external pins. Without it the chip ignores
 * them and stays dark however the rest of the registers are programmed, which
 * is a confusing failure: everything reads back exactly as if the LED were on.
 */
#define RT5033_FLED_FUNC1_PIN_CTRL	BIT(4)

/* RT5033_REG_FLED_FUNCTION2 */
#define RT5033_FLED_FUNC2_EN		BIT(0)
#define RT5033_FLED_FUNC2_STROBE	BIT(7)

/*
 * The FLED oscillator enable. Register 0x1a is not described in
 * rt5033-private.h; the vendor driver reaches it directly through a helper
 * (rt5033_set_fled_osc_en() in drivers/leds/rt5033_fled.c) and this is the only
 * bit of it anything touches.
 */
#define RT5033_REG_FLED_OSC		0x1a
#define RT5033_FLED_OSC_EN		BIT(5)

/* RT5033_REG_FLED_STROBE_CTRL1 */
#define RT5033_FLED_STROBE_CURR_MASK	GENMASK(4, 0)

/* RT5033_REG_FLED_STROBE_CTRL2 */
#define RT5033_FLED_STROBE_TIMEOUT_MASK	GENMASK(5, 0)

/* RT5033_REG_FLED_CTRL1 */
#define RT5033_FLED_TORCH_CURR_MASK	GENMASK(7, 4)

/*
 * Flash: 50 mA to 850 mA in 25 mA steps. Torch: 12.5 mA to 200 mA in 12.5 mA
 * steps, which the LED flash class cannot express in whole microamps at every
 * step, so the torch is driven in 25 mA steps (every second hardware step).
 */
#define RT5033_FLED_STROBE_CURR_MIN	50000
#define RT5033_FLED_STROBE_CURR_MAX	850000
#define RT5033_FLED_STROBE_CURR_STEP	25000

#define RT5033_FLED_TORCH_CURR_MIN	25000
#define RT5033_FLED_TORCH_CURR_MAX	200000
#define RT5033_FLED_TORCH_CURR_STEP	25000

/* 64 ms to 2080 ms in 32 ms steps. */
#define RT5033_FLED_TIMEOUT_MIN		64000
#define RT5033_FLED_TIMEOUT_MAX		2080000
#define RT5033_FLED_TIMEOUT_STEP	32000

struct rt5033_led {
	struct led_classdev_flash fled;
	struct regmap *regmap;
	struct mutex lock;	/* serialises the multi-register on/off dance */
	struct device *dev;
	/*
	 * On the boards this is used on the output is gated by two pins rather
	 * than by the enable bit alone: one holds the LED on for torch, the
	 * other fires a strobe. The registers below still set the current and
	 * the timing, but without these the part is configured perfectly and
	 * stays dark.
	 */
	struct gpio_desc *enable_gpio;
	struct gpio_desc *flash_gpio;
};

static struct rt5033_led *to_rt5033_led(struct led_classdev_flash *fled)
{
	return container_of(fled, struct rt5033_led, fled);
}

/*
 * Enabling means selecting torch or flash in FUNCTION1 and then pulsing
 * FUNCTION2: the vendor driver clears both EN and STROBE, then sets EN, and the
 * chip only picks the mode up on that transition. Disabling has to drop STROBE
 * before EN, with a short settle in between, or the output stays latched on.
 */
static int rt5033_led_set_output(struct rt5033_led *led, bool flash, bool on)
{
	int ret;

	if (!on) {
		ret = regmap_clear_bits(led->regmap, RT5033_REG_FLED_FUNCTION2,
					RT5033_FLED_FUNC2_STROBE);
		if (ret)
			return ret;

		usleep_range(500, 1000);

		ret = regmap_clear_bits(led->regmap, RT5033_REG_FLED_FUNCTION2,
					RT5033_FLED_FUNC2_EN);
		if (ret)
			return ret;

		gpiod_set_value_cansleep(led->flash_gpio, 0);
		gpiod_set_value_cansleep(led->enable_gpio, 0);

		return regmap_clear_bits(led->regmap, RT5033_REG_FLED_OSC,
					 RT5033_FLED_OSC_EN);
	}

	ret = regmap_update_bits(led->regmap, RT5033_REG_FLED_FUNCTION1,
				 RT5033_FLED_FUNC1_STROBE_MODE,
				 flash ? RT5033_FLED_FUNC1_STROBE_MODE : 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(led->regmap, RT5033_REG_FLED_FUNCTION2,
				 RT5033_FLED_FUNC2_EN | RT5033_FLED_FUNC2_STROBE, 0);
	if (ret)
		return ret;

	/*
	 * The enable bit only takes effect while the FLED oscillator is running,
	 * and the oscillator is not left on afterwards - the vendor driver
	 * pulses it around exactly this write. Without the pulse the output
	 * stays dark while every register reads back as though the LED were on.
	 */
	ret = regmap_set_bits(led->regmap, RT5033_REG_FLED_OSC,
			      RT5033_FLED_OSC_EN);
	if (ret)
		return ret;

	ret = regmap_update_bits(led->regmap, RT5033_REG_FLED_FUNCTION2,
				 RT5033_FLED_FUNC2_EN | RT5033_FLED_FUNC2_STROBE,
				 RT5033_FLED_FUNC2_EN);
	if (ret)
		return ret;

	ret = regmap_clear_bits(led->regmap, RT5033_REG_FLED_OSC,
				RT5033_FLED_OSC_EN);
	if (ret)
		return ret;

	/* Finally let the light out. */
	if (flash)
		gpiod_set_value_cansleep(led->flash_gpio, 1);
	else
		gpiod_set_value_cansleep(led->enable_gpio, 1);

	return 0;
}

static int rt5033_led_brightness_set(struct led_classdev *cdev,
				     enum led_brightness brightness)
{
	struct led_classdev_flash *fled = lcdev_to_flcdev(cdev);
	struct rt5033_led *led = to_rt5033_led(fled);
	int ret;

	mutex_lock(&led->lock);

	if (!brightness) {
		ret = rt5033_led_set_output(led, false, false);
		goto out;
	}

	ret = regmap_update_bits(led->regmap, RT5033_REG_FLED_CTRL1,
				 RT5033_FLED_TORCH_CURR_MASK,
				 FIELD_PREP(RT5033_FLED_TORCH_CURR_MASK,
					    (brightness * 2) - 1));
	if (ret)
		goto out;

	ret = rt5033_led_set_output(led, false, true);
out:
	mutex_unlock(&led->lock);

	return ret;
}

static int rt5033_led_flash_brightness_set(struct led_classdev_flash *fled,
					   u32 brightness)
{
	struct rt5033_led *led = to_rt5033_led(fled);
	struct led_flash_setting *s = &fled->brightness;
	int ret;

	mutex_lock(&led->lock);
	ret = regmap_update_bits(led->regmap, RT5033_REG_FLED_STROBE_CTRL1,
				 RT5033_FLED_STROBE_CURR_MASK,
				 (brightness - s->min) / s->step);
	mutex_unlock(&led->lock);

	return ret;
}

static int rt5033_led_flash_timeout_set(struct led_classdev_flash *fled,
					u32 timeout)
{
	struct rt5033_led *led = to_rt5033_led(fled);
	struct led_flash_setting *s = &fled->timeout;
	int ret;

	mutex_lock(&led->lock);
	ret = regmap_update_bits(led->regmap, RT5033_REG_FLED_STROBE_CTRL2,
				 RT5033_FLED_STROBE_TIMEOUT_MASK,
				 (timeout - s->min) / s->step);
	mutex_unlock(&led->lock);

	return ret;
}

static int rt5033_led_flash_strobe_set(struct led_classdev_flash *fled,
				       bool state)
{
	struct rt5033_led *led = to_rt5033_led(fled);
	int ret;

	mutex_lock(&led->lock);
	ret = rt5033_led_set_output(led, true, state);
	if (!ret)
		fled->led_cdev.brightness = 0;
	mutex_unlock(&led->lock);

	return ret;
}

static const struct led_flash_ops rt5033_led_flash_ops = {
	.flash_brightness_set = rt5033_led_flash_brightness_set,
	.timeout_set = rt5033_led_flash_timeout_set,
	.strobe_set = rt5033_led_flash_strobe_set,
};

static void rt5033_led_init_setting(struct led_flash_setting *s,
				    u32 min, u32 max, u32 step)
{
	s->min = min;
	s->max = max;
	s->step = step;
	s->val = max;
}

static int rt5033_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct led_init_data init_data = {};
	struct led_classdev *cdev;
	struct rt5033_led *led;
	int ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->dev = dev;
	led->regmap = dev_get_regmap(dev->parent, NULL);
	if (!led->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get parent regmap\n");

	ret = devm_mutex_init(dev, &led->lock);
	if (ret)
		return ret;

	led->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(led->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(led->enable_gpio),
				     "Failed to get the enable gpio\n");

	led->flash_gpio = devm_gpiod_get_optional(dev, "flash", GPIOD_OUT_LOW);
	if (IS_ERR(led->flash_gpio))
		return dev_err_probe(dev, PTR_ERR(led->flash_gpio),
				     "Failed to get the flash gpio\n");

	rt5033_led_init_setting(&led->fled.brightness,
				RT5033_FLED_STROBE_CURR_MIN,
				RT5033_FLED_STROBE_CURR_MAX,
				RT5033_FLED_STROBE_CURR_STEP);
	rt5033_led_init_setting(&led->fled.timeout,
				RT5033_FLED_TIMEOUT_MIN,
				RT5033_FLED_TIMEOUT_MAX,
				RT5033_FLED_TIMEOUT_STEP);
	led->fled.ops = &rt5033_led_flash_ops;

	cdev = &led->fled.led_cdev;
	cdev->brightness_set_blocking = rt5033_led_brightness_set;
	cdev->max_brightness = RT5033_FLED_TORCH_CURR_MAX /
			       RT5033_FLED_TORCH_CURR_STEP;
	cdev->flags |= LED_DEV_CAP_FLASH;

	/* Make sure the output is off and both channels are available. */
	ret = rt5033_led_set_output(led, false, false);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to turn the LED off\n");

	/*
	 * Select pin control here rather than alongside each enable: switching
	 * the mode while the pin is already asserted takes visible fractions of
	 * a second to take effect, where establishing it once at probe makes
	 * later GPIO transitions immediate.
	 */
	ret = regmap_set_bits(led->regmap, RT5033_REG_FLED_FUNCTION1,
			      RT5033_FLED_FUNC1_FLED1_EN |
			      RT5033_FLED_FUNC1_FLED2_EN |
			      RT5033_FLED_FUNC1_PIN_CTRL);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable the LED channels\n");

	init_data.fwnode = dev_fwnode(dev);
	init_data.devicename = "rt5033";
	init_data.default_label = "flash";

	ret = devm_led_classdev_flash_register_ext(dev, &led->fled, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register the flash LED\n");

	return 0;
}

static const struct of_device_id rt5033_led_of_match[] = {
	{ .compatible = "richtek,rt5033-led", },
	{ }
};
MODULE_DEVICE_TABLE(of, rt5033_led_of_match);

static struct platform_driver rt5033_led_driver = {
	.driver = {
		.name = "rt5033-led",
		.of_match_table = rt5033_led_of_match,
	},
	.probe = rt5033_led_probe,
};
module_platform_driver(rt5033_led_driver);

MODULE_DESCRIPTION("Richtek RT5033 flash LED driver");
MODULE_LICENSE("GPL");
