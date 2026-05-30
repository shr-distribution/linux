// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM8502 Multi-Function Device core
 *
 * Copyright (C) 2008 Palm Inc.
 * Copyright (C) 2024 Christophe Chapuis <chris.music.music@gmail.com>
 * Copyright (C) 2024-2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * The LM8502 is a combo LED + haptic controller on a single I2C address
 * (0x33 on the HP TouchPad). This MFD core handles everything chip-wide:
 *
 *   - the i2c_client and regmap
 *   - the chip-enable GPIO (a TLMM line on tenderloin)
 *   - the vcc regulator and the High Power Mode (HPM) handshake with
 *     RPM-managed PMIC LDOs (PM8058_L16 on tenderloin)
 *   - the post-reset chip configuration sequence (software reset,
 *     ENGINE_CNTRL1/2, MISC) with explicit write-then-readback so
 *     I2C controllers that do not surface NAKs (notably MSM8x60 QUP
 *     in the immediate post-power-up window) fail fast in probe
 *     rather than silently leaving the chip unconfigured
 *   - PM suspend/resume of the whole chip via the CHIP_EN bit
 *
 * Per-subsystem code lives in child drivers:
 *
 *   drivers/leds/leds-lm8502.c         - LED class for outputs D1..D10
 *   drivers/input/misc/lm8502-haptic.c - EV_FF / FF_RUMBLE for the
 *                                        internal H-bridge vibrator
 *
 * Both children are spawned via devm_mfd_add_devices() with explicit
 * .of_compatible matches so their DT subnodes drive per-subsystem
 * configuration (the LED children, ti,invert-direction, etc).
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

/* Maximum time to wait for the chip to ACK reads/writes after power-up. */
#define LM8502_DETECT_TIMEOUT_MS	250
#define LM8502_DETECT_POLL_MS		5

/* Time the chip needs after RESET=0xFF before responding again. */
#define LM8502_RESET_SETTLE_MS		50

/*
 * After raising enable_gpio the chip's internal reset deasserts and
 * the I2C front-end becomes responsive; datasheet calls out ~1 ms.
 */
#define LM8502_ENABLE_SETTLE_US		1000

/*
 * regulator_set_load() on RPM-managed PMIC LDOs (PM8058_L16) only
 * sends an IPC; the rail change is asynchronous. 5 ms is the worst-
 * case RPM commit window measured on MSM8x60 silicon -- no polling
 * API exposes commit completion.
 */
#define LM8502_RPM_HPM_SETTLE_US	5000

/* Driver-private wrapper around the shared struct lm8502. */
struct lm8502_core {
	struct lm8502 chip;		/* shared with children via parent drvdata */

	/* Resources only the core touches: */
	struct i2c_client *client;
	struct regulator *vcc;
	struct gpio_desc *enable_gpio;
};

static bool lm8502_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;			/* every read goes to the bus */
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

/*
 * Bring the chip out of reset and verify it is ACKing the bus.
 *
 * The post-power-up window is tricky on MSM8x60 QUP: writes complete
 * without -ENXIO even when the chip is not actually responding, so a
 * blind probe-time init silently writes into the void. We use the
 * LP55xx-family pattern: write CHIP_EN, read it back, retry until the
 * read matches or we time out.
 */
static int lm8502_wait_for_chip(struct lm8502 *chip)
{
	unsigned int val;
	int elapsed = 0;
	int ret;

	while (elapsed < LM8502_DETECT_TIMEOUT_MS) {
		ret = regmap_write(chip->regmap, LM8502_ENGINE_CNTRL1,
				   LM8502_CHIP_EN);
		if (ret)
			goto sleep;

		ret = regmap_read(chip->regmap, LM8502_ENGINE_CNTRL1, &val);
		if (ret == 0 && (val & LM8502_CHIP_EN))
			return 0;

sleep:
		fsleep(LM8502_DETECT_POLL_MS * USEC_PER_MSEC);
		elapsed += LM8502_DETECT_POLL_MS;
	}

	return -ENODEV;
}

static int lm8502_chip_init(struct lm8502_core *core)
{
	struct lm8502 *chip = &core->chip;
	struct device *dev = chip->dev;
	int ret, i;

	/*
	 * Software reset. Writes through regmap; the chip may briefly
	 * NAK reads in this window so we do not bother checking the
	 * return value -- lm8502_wait_for_chip() proves bidirectional
	 * communication below.
	 */
	regmap_write(chip->regmap, LM8502_RESET, LM8502_RESET_MAGIC);
	fsleep(LM8502_RESET_SETTLE_MS * USEC_PER_MSEC);

	ret = lm8502_wait_for_chip(chip);
	if (ret)
		return dev_err_probe(dev, ret, "chip did not respond\n");

	ret = regmap_write(chip->regmap, LM8502_ENGINE_CNTRL2,
			   LM8502_ENGINE_CNTRL2_INIT);
	if (ret)
		return dev_err_probe(dev, ret, "ENGINE_CNTRL2 write failed\n");

	ret = regmap_write(chip->regmap, LM8502_MISC, LM8502_MISC_INIT);
	if (ret)
		return dev_err_probe(dev, ret, "MISC write failed\n");

	/*
	 * Program every D<n>_CONTROL register so the LED child can issue
	 * brightness writes to D<n>_CURRENT_CTRL immediately. Without
	 * this the channels stay at the post-reset default and ignore
	 * current writes until the user manually configures them.
	 */
	for (i = 0; i < LM8502_MAX_LEDS; i++) {
		ret = regmap_write(chip->regmap, LM8502_D1_CONTROL + i,
				   LM8502_LED_CONTROL_INIT);
		if (ret)
			return dev_err_probe(dev, ret,
					     "D%d_CONTROL write failed\n",
					     i + 1);
	}

	return 0;
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
				     "failed to allocate regmap\n");

	/*
	 * Optional vcc regulator. On the HP TouchPad this is PM8058_L16,
	 * a 1.8 V always-on RPM-managed LDO with "regulator-allow-set-load"
	 * so we can ask for High Power Mode below.
	 */
	core->vcc = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR(core->vcc)) {
		ret = PTR_ERR(core->vcc);
		if (ret == -ENODEV) {
			core->vcc = NULL;
		} else {
			return dev_err_probe(dev, ret,
					     "failed to get vcc supply\n");
		}
	}

	if (core->vcc) {
		ret = regulator_enable(core->vcc);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to enable vcc supply\n");

		/*
		 * Ask for High Power Mode. The LM8502 internal boost converter
		 * plus all ten LED outputs at up to 9 mA each draw ~100 mA
		 * worst case. On RPM-managed LDOs (PM8058_L16) the default Low
		 * Power Mode caps the regulator at ~1 mA, leaving the chip
		 * current-starved.
		 *
		 * regulator_set_load() is a no-op on consumers whose supply
		 * does not carry "regulator-allow-set-load", so this is safe
		 * on platforms with simpler regulator topology.
		 */
		ret = regulator_set_load(core->vcc, 100000);
		if (ret) {
			dev_err_probe(dev, ret,
				      "regulator_set_load(100mA) failed\n");
			goto err_disable_vcc;
		}

		fsleep(LM8502_RPM_HPM_SETTLE_US);
	}

	core->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						    GPIOD_OUT_LOW);
	if (IS_ERR(core->enable_gpio)) {
		ret = PTR_ERR(core->enable_gpio);
		goto err_disable_vcc;
	}

	if (core->enable_gpio) {
		gpiod_set_value_cansleep(core->enable_gpio, 1);
		fsleep(LM8502_ENABLE_SETTLE_US);
	}

	dev_set_drvdata(dev, chip);

	ret = lm8502_chip_init(core);
	if (ret)
		goto err_disable_chip;

	/*
	 * Spawn LED and haptic child platform devices. Children retrieve
	 * the shared struct lm8502 via dev_get_drvdata(pdev->dev.parent);
	 * since chip_init above already completed, children can issue
	 * register writes from their own probe paths immediately.
	 */
	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, lm8502_devs,
				   ARRAY_SIZE(lm8502_devs), NULL, 0, NULL);
	if (ret) {
		dev_err_probe(dev, ret, "failed to add child devices\n");
		goto err_disable_chip;
	}

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
	int ret;

	mutex_lock(&chip->lock);
	if (chip->suspended) {
		mutex_unlock(&chip->lock);
		return 0;
	}

	/* Stop the haptic output in case a child left it running. */
	ret = regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
	if (ret) {
		dev_err(dev, "suspend: HAPTIC_FEEDBACK_CTRL write failed: %d\n",
			ret);
		goto unlock;
	}

	/* Drop CHIP_EN to power-gate the analog side. */
	ret = regmap_update_bits(chip->regmap, LM8502_ENGINE_CNTRL1,
				 LM8502_CHIP_EN, 0);
	if (ret) {
		dev_err(dev, "suspend: CHIP_EN clear failed: %d\n", ret);
		goto unlock;
	}

	if (core->enable_gpio)
		gpiod_set_value_cansleep(core->enable_gpio, 0);

	/*
	 * Drop the regulator to Low Power Mode now that the chip is
	 * off; HPM is only needed while the analog stage draws ~100 mA,
	 * and leaving it set wastes power across system suspend on
	 * RPM-managed PMIC LDOs. Restored in resume.
	 */
	if (core->vcc)
		regulator_set_load(core->vcc, 0);

	chip->suspended = true;
	mutex_unlock(&chip->lock);

	return 0;

unlock:
	mutex_unlock(&chip->lock);
	return ret;
}

static int lm8502_resume(struct device *dev)
{
	struct lm8502 *chip = dev_get_drvdata(dev);
	struct lm8502_core *core = container_of(chip, struct lm8502_core, chip);
	int ret;

	mutex_lock(&chip->lock);
	if (!chip->suspended) {
		mutex_unlock(&chip->lock);
		return 0;
	}

	/*
	 * Restore High Power Mode so the regulator can supply the
	 * ~100 mA the analog stage draws under load. Symmetric with the
	 * drop in suspend; safe no-op on consumers without
	 * regulator-allow-set-load.
	 */
	if (core->vcc) {
		ret = regulator_set_load(core->vcc, 100000);
		if (ret) {
			dev_err(dev, "resume: regulator_set_load(HPM) failed: %d\n",
				ret);
			goto unlock;
		}
		fsleep(LM8502_RPM_HPM_SETTLE_US);
	}

	if (core->enable_gpio) {
		gpiod_set_value_cansleep(core->enable_gpio, 1);
		fsleep(LM8502_ENABLE_SETTLE_US);
	}

	/*
	 * Suspend de-asserts the enable_gpio, which on most boards power-
	 * gates the chip and clears its register state. Re-run the full
	 * init sequence so ENGINE_CNTRL1/2, MISC and the per-channel
	 * D<n>_CONTROL registers are restored before any child driver
	 * touches the chip. lm8502_chip_init() also sets CHIP_EN as a
	 * side-effect of programming ENGINE_CNTRL1.
	 */
	ret = lm8502_chip_init(core);
	if (ret)
		goto err_disable;

	chip->suspended = false;
	mutex_unlock(&chip->lock);

	return 0;

err_disable:
	/*
	 * chip_init failed; we already raised enable_gpio and committed
	 * HPM. Unwind in reverse so the rail is back in a known-off
	 * state -- otherwise the chip would sit half-initialised with
	 * the regulator stuck in HPM until the next system suspend.
	 */
	if (core->enable_gpio)
		gpiod_set_value_cansleep(core->enable_gpio, 0);
	if (core->vcc)
		regulator_set_load(core->vcc, 0);
unlock:
	mutex_unlock(&chip->lock);
	return ret;
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

MODULE_DESCRIPTION("TI LM8502 combo LED + haptic controller (MFD core)");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_LICENSE("GPL");
