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
 * I2C sequence per FF_RUMBLE start/stop (matches legacy vendor):
 *
 *   start: D10_CURRENT_CTRL      = 0x00       // mux D10 pin to vibrator
 *          HAPTIC_PWM_DUTY_CYCLE = magnitude>>8
 *          HAPTIC_FEEDBACK_CTRL  = ENABLE | (INVERT_DIR if requested)
 *   stop:  HAPTIC_FEEDBACK_CTRL  = 0
 *
 * The play_effect callback only stores the magnitude and schedules a
 * workqueue item; the actual I2C writes happen in the work handler in
 * sleeping context.
 */

#include <linux/input.h>
#include <linux/mfd/lm8502.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

struct lm8502_haptic {
	struct lm8502 *chip;
	struct input_dev *input;
	struct work_struct work;
	u16 magnitude;
	u8 d10_saved;	/* LM8502_D10_CURRENT_CTRL snapshot taken under
			 * chip->lock the moment before we mux D10 to the
			 * vibrator path; restored when the rumble ends.
			 * Captured at start time (not probe) so concurrent
			 * writes from sibling MFD children -- e.g. a LED
			 * brightness write to the same register -- are
			 * preserved across vibrations.
			 */
	bool d10_saved_valid;
	bool invert_direction;
};

static void lm8502_haptic_work(struct work_struct *w)
{
	struct lm8502_haptic *priv =
		container_of(w, struct lm8502_haptic, work);
	struct lm8502 *chip = priv->chip;
	u16 magnitude;
	u8 fb;

	mutex_lock(&chip->lock);
	if (chip->suspended)
		goto out;

	magnitude = READ_ONCE(priv->magnitude);

	if (magnitude) {
		unsigned int val;

		/*
		 * Snapshot whatever the LED child (or any other MFD
		 * consumer) last wrote to the shared D10 register, under
		 * chip->lock, so we restore that exact value when the
		 * vibration ends. Done at start (not probe) so concurrent
		 * sibling writes are not clobbered.
		 *
		 * Only snapshot on the *first* magnitude>0 of a sequence
		 * (d10_saved_valid is false). Userspace can submit a fresh
		 * ff_effect mid-vibration; without this guard the second
		 * work invocation would read D10 back as 0 (which we just
		 * wrote ourselves), overwriting the original sibling value
		 * and breaking restore when the sequence finally stops.
		 *
		 * If the read fails we bail out without touching D10 at all:
		 * muxing without a valid snapshot would strand D10 in the
		 * vibrator path indefinitely.
		 */
		if (!priv->d10_saved_valid) {
			if (regmap_read(chip->regmap,
					LM8502_D10_CURRENT_CTRL, &val))
				goto out;
			priv->d10_saved = val;
			priv->d10_saved_valid = true;
		}

		/* Mux the shared D10 pin to the vibrator path. */
		regmap_write(chip->regmap, LM8502_D10_CURRENT_CTRL, 0);
		regmap_write(chip->regmap, LM8502_HAPTIC_PWM_DUTY_CYCLE,
			     magnitude >> 8);

		fb = LM8502_HAPTIC_FB_ENABLE;
		if (priv->invert_direction)
			fb |= LM8502_HAPTIC_FB_INVERT_DIR;
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, fb);
	} else {
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);

		/* Restore the shared D10 pin so the LED child resumes. */
		if (priv->d10_saved_valid) {
			regmap_write(chip->regmap, LM8502_D10_CURRENT_CTRL,
				     priv->d10_saved);
			priv->d10_saved_valid = false;
		}
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

	/*
	 * Userspace can call play_op concurrently with the work handler
	 * already running; use WRITE_ONCE so the work item sees a clean
	 * 16-bit value (the corresponding READ_ONCE is in the work
	 * function above). The 'effect lost if play_op fires twice in
	 * quick succession' case is benign -- the second magnitude
	 * naturally supersedes the first.
	 */
	WRITE_ONCE(priv->magnitude, mag);
	schedule_work(&priv->work);

	return 0;
}

static int lm8502_haptic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lm8502_haptic *priv;
	struct input_dev *input;
	int ret;

	/*
	 * Plain kzalloc rather than devm_kzalloc: the input core may
	 * keep the input_dev (and through input_set_drvdata, references
	 * to priv) alive past .remove if userspace still holds an evdev
	 * fd. devm would free priv on .remove return, opening a UAF
	 * window if the input core's deferred release fires later and
	 * any callback dereferences priv. We free priv ourselves in
	 * .remove after input_unregister_device has flushed callbacks.
	 */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip = dev_get_drvdata(dev->parent);
	if (!priv->chip) {
		ret = dev_err_probe(dev, -ENODEV, "no parent lm8502 chip\n");
		goto err_free_priv;
	}

	priv->invert_direction =
		device_property_read_bool(dev, "ti,invert-direction");

	INIT_WORK(&priv->work, lm8502_haptic_work);

	/*
	 * Use plain input_allocate_device() rather than the devm helper so
	 * .remove can call input_unregister_device() *before*
	 * cancel_work_sync(): devm release runs after .remove returns,
	 * which would leave a window where userspace can issue an
	 * ff_effect play_op after cancel_work_sync and re-enter the work
	 * handler with freed driver state.
	 */
	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto err_free_priv;
	}

	priv->input = input;
	input_set_drvdata(input, priv);

	input->name = "lm8502_haptic";
	input->id.bustype = BUS_I2C;
	/*
	 * Required when not using devm_input_allocate_device(): the devm
	 * helper sets dev.parent for us, plain input_allocate_device()
	 * does not. Without it the device lands under
	 * /sys/devices/virtual/input/ instead of under the platform_device.
	 */
	input->dev.parent = dev;

	input_set_capability(input, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(input, NULL, lm8502_haptic_play_effect);
	if (ret) {
		dev_err_probe(dev, ret, "FF memless create failed\n");
		goto err_free;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register input device\n");
		goto err_free;
	}

	platform_set_drvdata(pdev, priv);

	return 0;

err_free:
	input_free_device(input);
err_free_priv:
	kfree(priv);
	return ret;
}

static void lm8502_haptic_stop(struct lm8502_haptic *priv)
{
	struct lm8502 *chip = priv->chip;

	cancel_work_sync(&priv->work);

	/* Best-effort: stop the motor and release the shared D10 pin. */
	mutex_lock(&chip->lock);
	if (!chip->suspended) {
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
		if (priv->d10_saved_valid) {
			regmap_write(chip->regmap, LM8502_D10_CURRENT_CTRL,
				     priv->d10_saved);
			priv->d10_saved_valid = false;
		}
	}
	mutex_unlock(&chip->lock);
}

static void lm8502_haptic_remove(struct platform_device *pdev)
{
	struct lm8502_haptic *priv = platform_get_drvdata(pdev);

	/*
	 * Unregister first so userspace can no longer schedule new work
	 * via the ff_effect play_op callback; then drain the pending work
	 * item and stop the motor. After this point no input callback
	 * can dereference priv, so it is safe to kfree() it.
	 */
	input_unregister_device(priv->input);
	lm8502_haptic_stop(priv);
	kfree(priv);
}

static void lm8502_haptic_shutdown(struct platform_device *pdev)
{
	struct lm8502_haptic *priv = platform_get_drvdata(pdev);

	/*
	 * On orderly system shutdown / reboot .remove is not called, so a
	 * vibrator left running by the last effect would keep buzzing
	 * until the regulator drops. Stop the motor explicitly.
	 */
	lm8502_haptic_stop(priv);
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
	.shutdown = lm8502_haptic_shutdown,
};
module_platform_driver(lm8502_haptic_driver);

MODULE_ALIAS("platform:lm8502-haptic");
MODULE_DESCRIPTION("TI LM8502 haptic / vibrator driver (FF_RUMBLE)");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
