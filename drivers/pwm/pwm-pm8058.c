// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8058 PWM driver
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Christophe Chabanois <christophe.chabanois@gmail.com>
 *
 * Based on downstream pmic8058-pwm.c driver.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define PM8058_LPG_CHANNELS		8

/* LPG register offsets from base (0x13c) */
#define LPG_CTL0_REG			0x00
#define LPG_CTL1_REG			0x01
#define LPG_CTL2_REG			0x02
#define LPG_CTL3_REG			0x03
#define LPG_CTL4_REG			0x04
#define LPG_CTL5_REG			0x05
#define LPG_CTL6_REG			0x06
#define LPG_BANK_SEL_REG		0x07
#define LPG_BANK_EN_REG			0x08

/* Control 0 bits */
#define PWM_OUTPUT_EN			BIT(3)
#define PWM_EN				BIT(2)

/* Control 1 bits */
#define PWM_BYPASS_LUT			BIT(6)

/* Control 4 bits */
#define PWM_CLK_SEL_MASK		GENMASK(6, 5)
#define PWM_CLK_SEL_SHIFT		5
#define PWM_PREDIV_MASK			GENMASK(4, 3)
#define PWM_PREDIV_SHIFT		3
#define PWM_M_MASK			GENMASK(2, 0)

/* Control 5 bits */
#define PWM_SIZE_9_BIT			BIT(0)

/* Clock selection */
#define PWM_CLK_1KHZ			1
#define PWM_CLK_32KHZ			2
#define PWM_CLK_19P2MHZ			3

/* Pre-divider values */
#define PWM_PREDIV_2			0
#define PWM_PREDIV_3			1
#define PWM_PREDIV_5			2
#define PWM_PREDIV_6			3

struct pm8058_pwm_chip {
	struct regmap *regmap;
	u32 base;
	u8 bank_mask;
	struct mutex lock;
};

static int pm8058_pwm_bank_sel(struct pm8058_pwm_chip *chip, unsigned int bank)
{
	return regmap_write(chip->regmap, chip->base + LPG_BANK_SEL_REG, bank);
}

/* Called with chip->lock held */
static int pm8058_pwm_bank_enable(struct pm8058_pwm_chip *chip, unsigned int bank, bool enable)
{
	if (enable)
		chip->bank_mask |= BIT(bank);
	else
		chip->bank_mask &= ~BIT(bank);

	return regmap_write(chip->regmap, chip->base + LPG_BANK_EN_REG, chip->bank_mask);
}

static int pm8058_pwm_apply(struct pwm_chip *pwm_chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct pm8058_pwm_chip *chip = pwmchip_get_drvdata(pwm_chip);
	unsigned int channel = pwm->hwpwm;
	u64 period_ns = state->period;
	u64 duty_ns = state->duty_cycle;
	unsigned int pwm_value, max_value;
	int clk_sel, pre_div, m;
	u8 ctl[6];
	int ret, i;

	mutex_lock(&chip->lock);

	ret = pm8058_pwm_bank_sel(chip, channel);
	if (ret)
		goto unlock;

	if (!state->enabled) {
		/* Disable PWM output */
		ret = regmap_write(chip->regmap, chip->base + LPG_CTL0_REG, 0);
		if (ret)
			goto unlock;

		pm8058_pwm_bank_enable(chip, channel, false);
		goto unlock;
	}

	/*
	 * Simple clock selection:
	 * - Use 19.2MHz clock with pre-div 2 for short periods
	 * - Use 32kHz for medium periods
	 * - Use 1kHz for long periods
	 * Use 9-bit PWM size for better resolution.
	 */
	if (period_ns < 100000) {
		/* < 100us: use 19.2MHz, pre-div 2 */
		clk_sel = PWM_CLK_19P2MHZ;
		pre_div = PWM_PREDIV_2;
		m = 0;
	} else if (period_ns < 1000000) {
		/* < 1ms: use 32kHz, pre-div 2 */
		clk_sel = PWM_CLK_32KHZ;
		pre_div = PWM_PREDIV_2;
		m = 2;
	} else {
		/* >= 1ms: use 1kHz, pre-div 2 */
		clk_sel = PWM_CLK_1KHZ;
		pre_div = PWM_PREDIV_2;
		m = 0;
	}

	/* Calculate PWM value (9-bit) */
	max_value = 511; /* 2^9 - 1 */
	if (period_ns > 0)
		pwm_value = div64_u64(duty_ns * max_value, period_ns);
	else
		pwm_value = 0;

	if (pwm_value > max_value)
		pwm_value = max_value;

	/* Build control registers */
	ctl[0] = 0; /* Will enable after configuring */
	ctl[1] = PWM_BYPASS_LUT; /* Direct PWM mode */
	ctl[2] = 0;
	ctl[3] = pwm_value & 0xff; /* PWM value bits 7:0 */
	ctl[4] = ((clk_sel << PWM_CLK_SEL_SHIFT) & PWM_CLK_SEL_MASK) |
		 ((pre_div << PWM_PREDIV_SHIFT) & PWM_PREDIV_MASK) |
		 (m & PWM_M_MASK) |
		 ((pwm_value >> 1) & 0x80); /* PWM value bit 8 */
	ctl[5] = PWM_SIZE_9_BIT;

	/* Write configuration registers */
	for (i = 0; i < 6; i++) {
		ret = regmap_write(chip->regmap, chip->base + i, ctl[i]);
		if (ret)
			goto unlock;
	}

	/* Enable bank */
	ret = pm8058_pwm_bank_enable(chip, channel, true);
	if (ret)
		goto unlock;

	/* Select bank again and enable output */
	ret = pm8058_pwm_bank_sel(chip, channel);
	if (ret)
		goto unlock;

	ret = regmap_write(chip->regmap, chip->base + LPG_CTL0_REG,
			   PWM_OUTPUT_EN | PWM_EN);

unlock:
	mutex_unlock(&chip->lock);
	return ret;
}

static int pm8058_pwm_get_state(struct pwm_chip *pwm_chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct pm8058_pwm_chip *chip = pwmchip_get_drvdata(pwm_chip);
	unsigned int channel = pwm->hwpwm;
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);

	ret = pm8058_pwm_bank_sel(chip, channel);
	if (ret)
		goto unlock;

	ret = regmap_read(chip->regmap, chip->base + LPG_CTL0_REG, &val);
	if (ret)
		goto unlock;

	state->enabled = !!(val & PWM_OUTPUT_EN);
	/* Default values - actual period/duty would require reading more registers */
	state->period = 100000; /* 100us default */
	state->duty_cycle = state->enabled ? 50000 : 0;
	state->polarity = PWM_POLARITY_NORMAL;

unlock:
	mutex_unlock(&chip->lock);
	return ret;
}

static const struct pwm_ops pm8058_pwm_ops = {
	.apply = pm8058_pwm_apply,
	.get_state = pm8058_pwm_get_state,
};

static int pm8058_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pm8058_pwm_chip *chip;
	struct pwm_chip *pwm_chip;
	u32 reg;
	int ret;

	pwm_chip = devm_pwmchip_alloc(dev, PM8058_LPG_CHANNELS, sizeof(*chip));
	if (IS_ERR(pwm_chip))
		return PTR_ERR(pwm_chip);

	chip = pwmchip_get_drvdata(pwm_chip);
	mutex_init(&chip->lock);

	chip->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(dev, "Parent regmap unavailable\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(dev->of_node, "reg", &reg);
	if (ret) {
		dev_err(dev, "Missing reg property\n");
		return ret;
	}
	chip->base = reg;

	pwm_chip->ops = &pm8058_pwm_ops;

	ret = devm_pwmchip_add(dev, pwm_chip);
	if (ret) {
		dev_err(dev, "Failed to add PWM chip: %d\n", ret);
		return ret;
	}

	dev_info(dev, "PM8058 PWM driver loaded (base=0x%x)\n", chip->base);

	return 0;
}

static const struct of_device_id pm8058_pwm_of_match[] = {
	{ .compatible = "qcom,pm8058-lpg" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8058_pwm_of_match);

static struct platform_driver pm8058_pwm_driver = {
	.probe = pm8058_pwm_probe,
	.driver = {
		.name = "pm8058-pwm",
		.of_match_table = pm8058_pwm_of_match,
	},
};
module_platform_driver(pm8058_pwm_driver);

MODULE_DESCRIPTION("Qualcomm PM8058 PWM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pm8058-pwm");
