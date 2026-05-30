// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM8502 Multi-Function Device core
 *
 * Copyright (C) 2008 Palm Inc.
 * Copyright (C) 2024 Christophe Chapuis <chris.music.music@gmail.com>
 * Copyright (C) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * The LM8502 is a combo LED + haptic + camera-flash controller on a
 * single I2C address (0x33 on the HP TouchPad). This MFD core handles
 * everything chip-wide:
 *
 *   - the i2c_client and regmap
 *   - the chip-enable GPIO (a TLMM line on tenderloin)
 *   - the vcc regulator and the High Power Mode (HPM) handshake with
 *     RPM-managed PMIC LDOs (PM8058_L16 on tenderloin)
 *   - the post-reset chip configuration sequence (software reset,
 *     ENGINE_CNTRL1/2, MISC) -- intentionally deferred ~3 s after
 *     probe because on this platform the QUP i2c controller doesn't
 *     report write NAKs immediately after the chip is powered up,
 *     so a probe-time init silently doesn't reach the chip
 *   - PM suspend/resume of the whole chip via the CHIP_EN bit
 *
 * The actual per-subsystem code lives in child drivers:
 *
 *   drivers/leds/leds-lm8502.c        - LED class for outputs D1..D10
 *   drivers/input/misc/lm8502-haptic.c - EV_FF / FF_RUMBLE for the
 *                                        internal H-bridge vibrator
 *
 * Both children are spawned via mfd_add_devices() with explicit
 * .of_compatible matches so their DT subnodes drive the per-subsystem
 * configuration (the LED children, the "ti,invert-direction" property,
 * etc).
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/lm8502.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

/* Driver-private wrapper around the shared struct lm8502. */
struct lm8502_core {
	struct lm8502 chip;             /* shared with children via parent drvdata */

	/* Resources only the core touches: */
	struct i2c_client *client;
	struct regulator *vcc;
	struct gpio_desc *enable_gpio;
	struct delayed_work init_work;  /* deferred chip_init */
};

static bool lm8502_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;                    /* no caching: every read goes to bus */
}

static const struct regmap_config lm8502_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xF0,
	.volatile_reg = lm8502_volatile_reg,
	.cache_type = REGCACHE_NONE,
};

/* MFD children spawned by this core. */
static const struct mfd_cell lm8502_devs[] = {
	{
		.name = "lm8502-leds",
		.of_compatible = "ti,lm8502-leds",
	},
	{
		.name = "lm8502-haptic",
		.of_compatible = "ti,lm8502-haptic",
	},
};

int lm8502_write_retry(struct lm8502 *chip, unsigned int reg, unsigned int val)
{
	int ret;

	ret = regmap_write(chip->regmap, reg, val);
	if (ret == -ENXIO) {
		usleep_range(2000, 3000);
		ret = regmap_write(chip->regmap, reg, val);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(lm8502_write_retry);

static int lm8502_chip_init(struct lm8502_core *core)
{
	struct lm8502 *chip = &core->chip;
	struct device *dev = chip->dev;
	int ret;

	/*
	 * Software reset. Mirrors the webOS sequence: written after the
	 * chip-enable GPIO goes high, before any other register writes
	 * land. Writing 0xFF to RESET (0x3D) triggers the reset.
	 */
	dev_info(dev, "Sending software reset\n");
	ret = lm8502_write_retry(chip, LM8502_RESET, 0xFF);
	if (ret) {
		dev_err(dev, "Software reset failed: %d\n", ret);
		return ret;
	}
	msleep(50);

	/*
	 * Bring the chip out of reset:
	 *   ENGINE_CNTRL1 = 0x40 (CHIP_EN)
	 *   ENGINE_CNTRL2 = 0x20
	 *
	 * webOS does a read-modify-write of ENGINE_CNTRL1 here but
	 * ignores i2c errors and just OR's in CHIP_EN. We hardcode 0x40
	 * for the same result without depending on a post-reset read
	 * succeeding (empirically fails with -ENXIO on the first attempt
	 * on tenderloin).
	 */
	ret = lm8502_write_retry(chip, LM8502_ENGINE_CNTRL1, LM8502_CHIP_EN);
	if (ret) {
		dev_err(dev, "ENGINE_CNTRL1 write failed: %d\n", ret);
		return ret;
	}

	ret = regmap_write(chip->regmap, LM8502_ENGINE_CNTRL2, 0x20);
	if (ret) {
		dev_err(dev, "ENGINE_CNTRL2 write failed: %d\n", ret);
		return ret;
	}

	/*
	 * MISC (0x36) = 0x6a:
	 *   bit 5: POWER_SAVE enabled
	 *   bit 3: BOOST_EN (required for LED current!)
	 *   bit 1: PWM_INPUT
	 * Matches the webOS configuration.
	 */
	ret = regmap_write(chip->regmap, LM8502_MISC, 0x6a);
	if (ret) {
		dev_err(dev, "MISC config failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Chip initialized, boost enabled\n");
	return 0;
}

static void lm8502_chip_init_work(struct work_struct *w)
{
	struct lm8502_core *core =
		container_of(to_delayed_work(w), struct lm8502_core, init_work);
	struct lm8502 *chip = &core->chip;
	struct device *dev = chip->dev;
	int ret;

	mutex_lock(&chip->lock);
	ret = lm8502_chip_init(core);
	if (ret) {
		dev_err(dev, "Deferred chip_init failed: %d (retrying in 5 s)\n",
			ret);
		mutex_unlock(&chip->lock);
		schedule_delayed_work(&core->init_work, msecs_to_jiffies(5000));
		return;
	}
	chip->initialized = true;
	mutex_unlock(&chip->lock);

	dev_info(dev, "Deferred chip_init complete; children can now drive\n");
}

static int lm8502_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lm8502_core *core;
	struct lm8502 *chip;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	chip = &core->chip;
	core->client = client;
	chip->dev = dev;
	mutex_init(&chip->lock);

	chip->regmap = devm_regmap_init_i2c(client, &lm8502_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "Failed to allocate regmap\n");

	/*
	 * Optional vcc regulator. On the HP TouchPad this is PM8058_L16, a
	 * 1.8 V always-on RPM-managed LDO with "regulator-allow-set-load"
	 * so we can ask for High Power Mode below.
	 */
	core->vcc = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR(core->vcc)) {
		ret = PTR_ERR(core->vcc);
		if (ret == -ENODEV) {
			core->vcc = NULL;
		} else {
			return dev_err_probe(dev, ret,
					     "Failed to get vcc supply\n");
		}
	}

	if (core->vcc) {
		ret = regulator_enable(core->vcc);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable vcc supply\n");

		/*
		 * Ask for High Power Mode. The LM8502 internal boost converter
		 * plus all ten LED outputs at up to 9 mA each draw ~100 mA worst
		 * case. On RPM-managed LDOs (PM8058_L16) the default Low Power
		 * Mode caps the regulator at ~1 mA, leaving the chip
		 * current-starved: i2c reads return 0xff and writes are NAKed
		 * (-ENXIO) without surfacing as errors in probe or LED sysfs.
		 *
		 * regulator_set_load() is a no-op on consumers whose supply
		 * doesn't carry "regulator-allow-set-load", so this is safe on
		 * platforms with simpler regulator topology.
		 */
		ret = regulator_set_load(core->vcc, 100000);
		if (ret)
			dev_warn(dev, "regulator_set_load(100mA) failed: %d\n",
				 ret);

		/*
		 * Give RPM time to actually switch the rail to HPM before we
		 * raise the chip-enable pin. The set_load call only sends an
		 * IPC; the rail change is asynchronous.
		 */
		usleep_range(5000, 10000);
	}

	core->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(core->enable_gpio)) {
		ret = PTR_ERR(core->enable_gpio);
		goto err_disable_vcc;
	}

	if (core->enable_gpio) {
		gpiod_set_value_cansleep(core->enable_gpio, 1);
		usleep_range(1000, 2000);   /* let the chip settle */
	}

	dev_set_drvdata(dev, chip);

	INIT_DELAYED_WORK(&core->init_work, lm8502_chip_init_work);

	/*
	 * Spawn LED and haptic child platform devices. Children retrieve
	 * the shared struct lm8502 via dev_get_drvdata(pdev->dev.parent)
	 * and gate their register writes on lm8502_chip_ready().
	 */
	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, lm8502_devs,
				   ARRAY_SIZE(lm8502_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to add child devices: %d\n", ret);
		goto err_disable_chip;
	}

	/*
	 * Defer the chip-side configuration. brightness_set / haptic work
	 * handlers in the children return / bail with -EAGAIN until
	 * lm8502_chip_ready() flips true.
	 */
	schedule_delayed_work(&core->init_work, msecs_to_jiffies(3000));

	dev_info(dev, "LM8502 MFD core registered (chip_init deferred 3 s)\n");
	return 0;

err_disable_chip:
	if (core->enable_gpio)
		gpiod_set_value_cansleep(core->enable_gpio, 0);
err_disable_vcc:
	if (core->vcc) {
		regulator_set_load(core->vcc, 0);
		regulator_disable(core->vcc);
	}
	return ret;
}

static void lm8502_i2c_remove(struct i2c_client *client)
{
	struct lm8502 *chip = dev_get_drvdata(&client->dev);
	struct lm8502_core *core = container_of(chip, struct lm8502_core, chip);

	cancel_delayed_work_sync(&core->init_work);

	if (core->enable_gpio)
		gpiod_set_value_cansleep(core->enable_gpio, 0);

	if (core->vcc) {
		regulator_set_load(core->vcc, 0);
		regulator_disable(core->vcc);
	}
}

static int lm8502_suspend(struct device *dev)
{
	struct lm8502 *chip = dev_get_drvdata(dev);
	struct lm8502_core *core = container_of(chip, struct lm8502_core, chip);

	if (chip->suspended)
		return 0;

	mutex_lock(&chip->lock);

	if (chip->initialized) {
		/* Stop the haptic output in case a child left it running. */
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);

		/* Drop CHIP_EN to power-gate the analog side. */
		regmap_update_bits(chip->regmap, LM8502_ENGINE_CNTRL1,
				   LM8502_CHIP_EN, 0);
	}

	if (core->enable_gpio)
		gpiod_set_value_cansleep(core->enable_gpio, 0);

	chip->suspended = true;
	mutex_unlock(&chip->lock);

	return 0;
}

static int lm8502_resume(struct device *dev)
{
	struct lm8502 *chip = dev_get_drvdata(dev);
	struct lm8502_core *core = container_of(chip, struct lm8502_core, chip);

	if (!chip->suspended)
		return 0;

	mutex_lock(&chip->lock);

	if (core->enable_gpio) {
		gpiod_set_value_cansleep(core->enable_gpio, 1);
		usleep_range(1000, 2000);
	}

	if (chip->initialized)
		regmap_update_bits(chip->regmap, LM8502_ENGINE_CNTRL1,
				   LM8502_CHIP_EN, LM8502_CHIP_EN);

	chip->suspended = false;
	mutex_unlock(&chip->lock);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(lm8502_pm_ops, lm8502_suspend, lm8502_resume);

static const struct of_device_id lm8502_of_match[] = {
	{ .compatible = "ti,lm8502" },
	{}
};
MODULE_DEVICE_TABLE(of, lm8502_of_match);

static const struct i2c_device_id lm8502_i2c_id[] = {
	{ "lm8502" },
	{}
};
MODULE_DEVICE_TABLE(i2c, lm8502_i2c_id);

static struct i2c_driver lm8502_i2c_driver = {
	.driver = {
		.name = "lm8502",
		.of_match_table = lm8502_of_match,
		.pm = pm_sleep_ptr(&lm8502_pm_ops),
	},
	.probe = lm8502_i2c_probe,
	.remove = lm8502_i2c_remove,
	.id_table = lm8502_i2c_id,
};
module_i2c_driver(lm8502_i2c_driver);

MODULE_DESCRIPTION("TI LM8502 MFD core (LED + haptic + flash)");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
