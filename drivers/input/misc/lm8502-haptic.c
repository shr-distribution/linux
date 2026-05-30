// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM8502 haptic / vibrator child driver (FF_RUMBLE)
 *
 * Copyright (C) 2008 Palm Inc.
 * Copyright (C) 2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * Per-subsystem child of the LM8502 MFD core (drivers/mfd/lm8502.c).
 * Exposes the LM8502's internal H-bridge haptic output as a standard
 * EV_FF / FF_RUMBLE input device.
 *
 * Bound by name "lm8502-haptic" from the MFD core's mfd_cell[]; the
 * matching DT node is the "haptic" subnode of "ti,lm8502" with
 * compatible "ti,lm8502-haptic". DT properties:
 *
 *   ti,invert-direction (boolean) - flip bit 0 of HAPTIC_FEEDBACK_CTRL
 *                                   so the H-bridge drives the motor
 *                                   in the polarity the board expects
 *
 * I2C sequence per FF_RUMBLE start/stop (matches legacy webOS):
 *
 *   start: D10_CURRENT_CTRL      = 0x00       // mux D10 pin to vibrator
 *          HAPTIC_PWM_DUTY_CYCLE = magnitude>>8
 *          HAPTIC_FEEDBACK_CTRL  = 0x02 | dir // bit1 enable, bit0 dir
 *   stop:  HAPTIC_FEEDBACK_CTRL  = 0
 *
 * The play_effect callback only stores the magnitude and schedules a
 * workqueue item; the actual I2C writes happen in the work handler in
 * sleeping context and gate on lm8502_chip_ready() so this child can
 * register its input device at probe even while the parent's deferred
 * chip_init is still pending.
 */

#include <linux/input.h>
#include <linux/mfd/lm8502.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

struct lm8502_haptic {
	struct lm8502 *chip;
	struct input_dev *input;
	struct work_struct work;
	u16 magnitude;
	bool invert_direction;
};

static void lm8502_haptic_work(struct work_struct *w)
{
	struct lm8502_haptic *priv =
		container_of(w, struct lm8502_haptic, work);
	struct lm8502 *chip = priv->chip;
	u8 duty;
	u8 dir;
	int ret;

	if (!lm8502_chip_ready(chip))
		return;

	mutex_lock(&chip->lock);

	if (priv->magnitude) {
		/* Mux the shared D10 pin to the vibrator path. */
		ret = lm8502_write_retry(chip, LM8502_D10_CURRENT_CTRL, 0);
		if (ret)
			goto out;

		duty = priv->magnitude >> 8;
		ret = lm8502_write_retry(chip, LM8502_HAPTIC_PWM_DUTY_CYCLE,
					 duty);
		if (ret)
			goto out;

		dir = priv->invert_direction ? 1 : 0;
		ret = lm8502_write_retry(chip, LM8502_HAPTIC_FEEDBACK_CTRL,
					 0x02 | dir);
	} else {
		ret = lm8502_write_retry(chip, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
	}

out:
	mutex_unlock(&chip->lock);
}

static int lm8502_haptic_play_effect(struct input_dev *dev, void *data,
				     struct ff_effect *effect)
{
	struct lm8502_haptic *priv = input_get_drvdata(dev);
	u16 mag = effect->u.rumble.strong_magnitude;

	if (!mag)
		mag = effect->u.rumble.weak_magnitude;

	priv->magnitude = mag;
	schedule_work(&priv->work);

	return 0;
}

static int lm8502_haptic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lm8502_haptic *priv;
	struct input_dev *input;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip = dev_get_drvdata(dev->parent);
	if (!priv->chip)
		return dev_err_probe(dev, -ENODEV, "no parent lm8502 chip\n");

	priv->invert_direction =
		device_property_read_bool(dev, "ti,invert-direction");

	INIT_WORK(&priv->work, lm8502_haptic_work);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	priv->input = input;
	input_set_drvdata(input, priv);

	input->name = "lm8502_haptic";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	input_set_capability(input, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(input, NULL, lm8502_haptic_play_effect);
	if (ret)
		return dev_err_probe(dev, ret, "FF memless create failed\n");

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register input device\n");

	platform_set_drvdata(pdev, priv);

	dev_info(dev, "FF_RUMBLE haptic registered (invert=%d)\n",
		 priv->invert_direction);
	return 0;
}

static void lm8502_haptic_remove(struct platform_device *pdev)
{
	struct lm8502_haptic *priv = platform_get_drvdata(pdev);
	struct lm8502 *chip = priv->chip;

	cancel_work_sync(&priv->work);

	/* Best-effort: stop the motor before tearing down. */
	if (lm8502_chip_ready(chip)) {
		mutex_lock(&chip->lock);
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
		mutex_unlock(&chip->lock);
	}
}

static const struct of_device_id lm8502_haptic_of_match[] = {
	{ .compatible = "ti,lm8502-haptic" },
	{}
};
MODULE_DEVICE_TABLE(of, lm8502_haptic_of_match);

static struct platform_driver lm8502_haptic_driver = {
	.driver = {
		.name = "lm8502-haptic",
		.of_match_table = lm8502_haptic_of_match,
	},
	.probe = lm8502_haptic_probe,
	.remove = lm8502_haptic_remove,
};
module_platform_driver(lm8502_haptic_driver);

MODULE_ALIAS("platform:lm8502-haptic");
MODULE_DESCRIPTION("TI LM8502 haptic / vibrator driver (FF_RUMBLE)");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
