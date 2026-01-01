// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI LM8502 LED Controller Driver
 *
 * Copyright (C) 2008 Palm Inc.
 * Copyright (C) 2026 webOS Community
 *
 * The LM8502 is a LED controller with:
 * - 10 LED channels with individual current control
 * - 2 programmable engines for LED animation
 * - Vibrator/haptic feedback support
 * - Camera flash/torch functionality
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* Register addresses */
#define LM8502_ENGINE_CNTRL1		0x00
#define LM8502_ENGINE_CNTRL2		0x01
#define LM8502_GROUP_FADING1		0x02
#define LM8502_GROUP_FADING2		0x03
#define LM8502_GROUP_FADING3		0x04

#define LM8502_D1_CONTROL		0x06
#define LM8502_D2_CONTROL		0x07
#define LM8502_D3_CONTROL		0x08
#define LM8502_D4_CONTROL		0x09
#define LM8502_D5_CONTROL		0x0a
#define LM8502_D6_CONTROL		0x0b
#define LM8502_D7_CONTROL		0x0c
#define LM8502_D8_CONTROL		0x0d
#define LM8502_D9_CONTROL		0x0e
#define LM8502_D10_CONTROL		0x0f

#define LM8502_HAPTIC_CONTROL		0x10
#define LM8502_HAPTIC_FEEDBACK_CTRL	0x21
#define LM8502_HAPTIC_PWM_DUTY_CYCLE	0x22

#define LM8502_D1_CURRENT		0x26
#define LM8502_D2_CURRENT		0x27
#define LM8502_D3_CURRENT		0x28
#define LM8502_D4_CURRENT		0x29
#define LM8502_D5_CURRENT		0x2a
#define LM8502_D6_CURRENT		0x2b
#define LM8502_D7_CURRENT		0x2c
#define LM8502_D8_CURRENT		0x2d
#define LM8502_D9_CURRENT		0x2e
#define LM8502_D10_CURRENT		0x2f

#define LM8502_MISC			0x36
#define LM8502_ENGINE1_PC		0x37
#define LM8502_ENGINE2_PC		0x38
#define LM8502_STATUS			0x3a
#define LM8502_INT			0x3b
#define LM8502_RESET			0x3d

#define LM8502_GROUP_FADER1		0x48
#define LM8502_GROUP_FADER2		0x49
#define LM8502_GROUP_FADER3		0x4a

#define LM8502_ENG1_PROG_START		0x4c
#define LM8502_ENG2_PROG_START		0x4d
#define LM8502_PROG_MEM_PAGE_SEL	0x4f
#define LM8502_PROG_MEM_START		0x50
#define LM8502_PROG_MEM_END		0x6f

#define LM8502_TORCH_BRIGHTNESS		0xa0
#define LM8502_FLASH_BRIGHTNESS		0xb0
#define LM8502_FLASH_DURATION		0xc0
#define LM8502_CONFIG_REG1		0xe0
#define LM8502_CONFIG_REG2		0xf0

/* Engine control bits */
#define LM8502_CHIP_EN			BIT(6)
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

/* LED control register bits */
#define LM8502_LED_ON			BIT(6)
#define LM8502_LED_MAPPING_MASK		0x03

/* Misc register bits */
#define LM8502_MISC_POWER_SAVE		BIT(5)

/* Flash/torch bits */
#define LM8502_STROBE_TIMEOUT		BIT(7)
#define LM8502_FLASH_MODE		0x03
#define LM8502_TORCH_MODE		0x02

#define LM8502_MAX_LEDS			10
#define LM8502_MAX_CURRENT		0xff
#define LM8502_INSTR_LEN		96
#define LM8502_INSTR_PER_PAGE		16

/* Flash current table (mA) */
static const u16 lm8502_flash_current[] = {
	38, 75, 113, 150, 188, 225, 263, 300,
	338, 375, 413, 450, 488, 525, 563, 600
};

/* Torch current table (mA) */
static const u16 lm8502_torch_current[] = {
	18, 37, 56, 75, 93, 112, 131, 150
};

struct lm8502_led {
	struct led_classdev cdev;
	struct lm8502 *chip;
	struct work_struct work;
	u8 control_reg;
	u8 current_reg;
	u8 brightness;
	u8 hw_group;
};

struct lm8502 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;
	struct gpio_desc *enable_gpio;

	/* LEDs */
	struct lm8502_led leds[LM8502_MAX_LEDS];
	int num_leds;

	/* Vibrator */
	struct input_dev *input_dev;
	struct work_struct vib_work;
	bool vib_enabled;
	u8 vib_duty_cycle;
	u8 vib_direction;
	bool vib_invert;

	/* Flash/Torch */
	struct led_classdev flash_cdev;
	struct led_classdev torch_cdev;
	bool flash_enabled;
	u16 flash_current;
	u16 torch_current;
	u16 flash_duration;

	/* Programmable engines */
	u16 instruct[LM8502_INSTR_LEN];
	int engine_startpage[2];
	int engine_endpage[2];

	/* Interrupt handling */
	struct work_struct irq_work;
	wait_queue_head_t engine_wait;
	int engine_status;
};

static const struct regmap_config lm8502_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xf0,
};

/* LED functions */
static void lm8502_led_work(struct work_struct *work)
{
	struct lm8502_led *led = container_of(work, struct lm8502_led, work);
	struct lm8502 *chip = led->chip;
	u8 pwm_value = (led->brightness * 255) / LED_FULL;

	mutex_lock(&chip->lock);

	if (led->hw_group > 0 && led->hw_group <= 3) {
		/* Use group fader for hardware groups */
		regmap_write(chip->regmap,
			     LM8502_GROUP_FADER1 + led->hw_group - 1,
			     pwm_value);
	} else {
		/* Direct current control */
		regmap_write(chip->regmap, led->current_reg, pwm_value);
	}

	mutex_unlock(&chip->lock);
}

static int lm8502_led_set(struct led_classdev *cdev,
			  enum led_brightness brightness)
{
	struct lm8502_led *led = container_of(cdev, struct lm8502_led, cdev);
	struct lm8502 *chip = led->chip;
	int ret = 0;

	led->brightness = brightness;

	mutex_lock(&chip->lock);

	if (brightness == LED_OFF) {
		ret = regmap_update_bits(chip->regmap, led->control_reg,
					 LM8502_LED_ON, 0);
	} else {
		u8 pwm = (brightness * 255) / LED_FULL;

		ret = regmap_write(chip->regmap, led->current_reg, pwm);
		if (!ret)
			ret = regmap_update_bits(chip->regmap, led->control_reg,
						 LM8502_LED_ON | LM8502_LED_MAPPING_MASK,
						 LM8502_LED_ON);
	}

	mutex_unlock(&chip->lock);
	return ret;
}

/* Vibrator functions */
static void lm8502_vib_work(struct work_struct *work)
{
	struct lm8502 *chip = container_of(work, struct lm8502, vib_work);
	u8 ctrl_val;

	mutex_lock(&chip->lock);

	if (chip->vib_enabled) {
		/* Set duty cycle */
		regmap_write(chip->regmap, LM8502_HAPTIC_PWM_DUTY_CYCLE,
			     (chip->vib_duty_cycle * 255) / 100);

		/* Enable haptic feedback with direction */
		ctrl_val = 0x02;
		if (chip->vib_invert ^ chip->vib_direction)
			ctrl_val |= 0x01;
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, ctrl_val);
	} else {
		regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
	}

	mutex_unlock(&chip->lock);
}

static int lm8502_vib_play(struct input_dev *dev, void *data,
			   struct ff_effect *effect)
{
	struct lm8502 *chip = input_get_drvdata(dev);
	u16 magnitude = effect->u.rumble.strong_magnitude;

	if (!magnitude)
		magnitude = effect->u.rumble.weak_magnitude;

	chip->vib_duty_cycle = (magnitude * 100) / 0xffff;
	chip->vib_enabled = chip->vib_duty_cycle > 0;

	schedule_work(&chip->vib_work);
	return 0;
}

/* Flash/Torch functions */
static u8 lm8502_get_flash_index(u16 current_ma)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lm8502_flash_current); i++) {
		if (lm8502_flash_current[i] >= current_ma)
			return i;
	}
	return ARRAY_SIZE(lm8502_flash_current) - 1;
}

static u8 lm8502_get_torch_index(u16 current_ma)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lm8502_torch_current); i++) {
		if (lm8502_torch_current[i] >= current_ma)
			return i;
	}
	return ARRAY_SIZE(lm8502_torch_current) - 1;
}

static int lm8502_flash_set(struct led_classdev *cdev,
			    enum led_brightness brightness)
{
	struct lm8502 *chip = container_of(cdev, struct lm8502, flash_cdev);
	u8 idx, reg;
	int ret;

	mutex_lock(&chip->lock);

	if (brightness == LED_OFF) {
		ret = regmap_read(chip->regmap, LM8502_FLASH_BRIGHTNESS, &reg);
		if (!ret) {
			reg &= ~0x03;
			ret = regmap_write(chip->regmap, LM8502_FLASH_BRIGHTNESS, reg);
		}
	} else {
		u16 current_ma = (brightness * 600) / LED_FULL;

		idx = lm8502_get_flash_index(current_ma);
		chip->flash_current = lm8502_flash_current[idx];

		ret = regmap_write(chip->regmap, LM8502_FLASH_BRIGHTNESS,
				   LM8502_STROBE_TIMEOUT | (idx << 3) | LM8502_FLASH_MODE);
	}

	mutex_unlock(&chip->lock);
	return ret;
}

static int lm8502_torch_set(struct led_classdev *cdev,
			    enum led_brightness brightness)
{
	struct lm8502 *chip = container_of(cdev, struct lm8502, torch_cdev);
	u8 idx, reg;
	int ret;

	mutex_lock(&chip->lock);

	if (brightness == LED_OFF) {
		ret = regmap_read(chip->regmap, LM8502_FLASH_BRIGHTNESS, &reg);
		if (!ret) {
			reg &= ~0x03;
			ret = regmap_write(chip->regmap, LM8502_FLASH_BRIGHTNESS, reg);
		}
	} else {
		u16 current_ma = (brightness * 150) / LED_FULL;

		idx = lm8502_get_torch_index(current_ma);
		chip->torch_current = lm8502_torch_current[idx];

		ret = regmap_write(chip->regmap, LM8502_TORCH_BRIGHTNESS, idx << 3);
		if (!ret)
			ret = regmap_write(chip->regmap, LM8502_FLASH_BRIGHTNESS,
					   LM8502_TORCH_MODE);
	}

	mutex_unlock(&chip->lock);
	return ret;
}

/* Engine control functions */
static void lm8502_set_engine_mode(struct lm8502 *chip, int engine,
				   u8 cntrl_reg, u8 mode)
{
	unsigned int reg;
	int shift = (engine == 1) ? LM8502_ENG1_SHIFT : LM8502_ENG2_SHIFT;

	regmap_read(chip->regmap, cntrl_reg, &reg);
	reg &= ~(0x03 << shift);
	reg |= (mode << shift);
	regmap_write(chip->regmap, cntrl_reg, reg);
}

static int lm8502_run_engine(struct lm8502 *chip, int engine)
{
	int page_start, page_end;
	int engine_pc, prog_start;
	int i, j;
	u8 upper, lower;

	if (engine == 1) {
		page_start = chip->engine_startpage[0];
		page_end = chip->engine_endpage[0];
		engine_pc = LM8502_ENGINE1_PC;
		prog_start = LM8502_ENG1_PROG_START;
	} else {
		page_start = chip->engine_startpage[1];
		page_end = chip->engine_endpage[1];
		engine_pc = LM8502_ENGINE2_PC;
		prog_start = LM8502_ENG2_PROG_START;
	}

	/* Hold and disable engine */
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL1, LM8502_ENG_HOLD);
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL2, LM8502_ENG_DISABLE);

	/* Configure PC and start address */
	regmap_write(chip->regmap, engine_pc, page_start * 16);
	regmap_write(chip->regmap, prog_start, page_start * 16);
	regmap_write(chip->regmap, LM8502_PROG_MEM_PAGE_SEL, page_start);

	/* Enter load mode */
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL2, LM8502_ENG_LOAD);

	/* Write program memory */
	for (i = page_start; i <= page_end; i++) {
		regmap_write(chip->regmap, LM8502_PROG_MEM_PAGE_SEL, i);

		for (j = 0; j < 16; j++) {
			upper = chip->instruct[i * 16 + j] >> 8;
			lower = chip->instruct[i * 16 + j] & 0xff;

			regmap_write(chip->regmap, LM8502_PROG_MEM_START + (2 * j) % 32, upper);
			regmap_write(chip->regmap, LM8502_PROG_MEM_START + (2 * j + 1) % 32, lower);
		}
	}

	/* Run the program */
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL1, LM8502_ENG_FREERUN);
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL2, LM8502_ENG_RUN);

	usleep_range(1000, 2000);
	return 0;
}

static int lm8502_stop_engine(struct lm8502 *chip, int engine)
{
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL1, LM8502_ENG_HOLD);
	lm8502_set_engine_mode(chip, engine, LM8502_ENGINE_CNTRL2, LM8502_ENG_DISABLE);
	usleep_range(1000, 2000);
	return 0;
}

/* Interrupt handler */
static void lm8502_irq_work(struct work_struct *work)
{
	struct lm8502 *chip = container_of(work, struct lm8502, irq_work);
	unsigned int status;

	mutex_lock(&chip->lock);
	regmap_read(chip->regmap, LM8502_STATUS, &status);

	if (status & 0x01)
		chip->engine_status = 2;
	else if (status & 0x02)
		chip->engine_status = 1;
	else
		chip->engine_status = 0;

	mutex_unlock(&chip->lock);

	if (chip->engine_status > 0)
		wake_up_interruptible(&chip->engine_wait);
}

static irqreturn_t lm8502_irq_handler(int irq, void *data)
{
	struct lm8502 *chip = data;

	schedule_work(&chip->irq_work);
	return IRQ_HANDLED;
}

/* Hardware initialization */
static int lm8502_init_hw(struct lm8502 *chip)
{
	int ret;

	/* Assert enable GPIO */
	if (chip->enable_gpio) {
		gpiod_set_value_cansleep(chip->enable_gpio, 1);
		usleep_range(1000, 2000);
	}

	/* Soft reset */
	ret = regmap_write(chip->regmap, LM8502_RESET, 0xff);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	/* Enable chip and power save mode */
	ret = regmap_write(chip->regmap, LM8502_ENGINE_CNTRL1, LM8502_CHIP_EN);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, LM8502_MISC, LM8502_MISC_POWER_SAVE);
	if (ret)
		return ret;

	/* Disable both engines */
	ret = regmap_write(chip->regmap, LM8502_ENGINE_CNTRL2, 0);

	return ret;
}

/* Device tree parsing */
static int lm8502_parse_led_dt(struct lm8502 *chip, struct device_node *np,
			       struct lm8502_led *led)
{
	struct led_init_data init_data = {};
	u32 reg;
	int ret;

	ret = of_property_read_u32(np, "reg", &reg);
	if (ret || reg >= LM8502_MAX_LEDS) {
		dev_err(chip->dev, "Invalid LED reg %d\n", reg);
		return -EINVAL;
	}

	led->control_reg = LM8502_D1_CONTROL + reg;
	led->current_reg = LM8502_D1_CURRENT + reg;
	led->chip = chip;
	led->hw_group = 0;

	of_property_read_u32(np, "ti,hw-group", &led->hw_group);

	led->cdev.brightness_set_blocking = lm8502_led_set;
	led->cdev.max_brightness = LED_FULL;

	INIT_WORK(&led->work, lm8502_led_work);

	init_data.fwnode = of_fwnode_handle(np);
	init_data.devicename = "lm8502";
	init_data.default_label = ":";

	ret = devm_led_classdev_register_ext(chip->dev, &led->cdev, &init_data);
	if (ret)
		dev_err(chip->dev, "Failed to register LED %d\n", reg);

	return ret;
}

static int lm8502_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *child;
	struct lm8502 *chip;
	int ret, i = 0;
	int irq;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->client = client;
	mutex_init(&chip->lock);
	init_waitqueue_head(&chip->engine_wait);
	INIT_WORK(&chip->irq_work, lm8502_irq_work);
	INIT_WORK(&chip->vib_work, lm8502_vib_work);

	chip->regmap = devm_regmap_init_i2c(client, &lm8502_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "Failed to init regmap\n");

	chip->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(chip->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->enable_gpio),
				     "Failed to get enable GPIO\n");

	ret = lm8502_init_hw(chip);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init hardware\n");

	/* Parse LED child nodes */
	for_each_available_child_of_node(dev->of_node, child) {
		if (i >= LM8502_MAX_LEDS) {
			of_node_put(child);
			break;
		}

		ret = lm8502_parse_led_dt(chip, child, &chip->leds[i]);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		i++;
	}
	chip->num_leds = i;

	/* Register vibrator as input device */
	chip->vib_invert = device_property_read_bool(dev, "ti,vibrator-invert");

	chip->input_dev = devm_input_allocate_device(dev);
	if (!chip->input_dev)
		return -ENOMEM;

	chip->input_dev->name = "lm8502-vibrator";
	chip->input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(chip->input_dev, chip);
	input_set_capability(chip->input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(chip->input_dev, NULL, lm8502_vib_play);
	if (ret) {
		dev_err(dev, "Failed to create FF device\n");
		return ret;
	}

	ret = input_register_device(chip->input_dev);
	if (ret) {
		dev_err(dev, "Failed to register input device\n");
		return ret;
	}

	/* Register flash LED */
	chip->flash_cdev.name = "lm8502:flash";
	chip->flash_cdev.brightness_set_blocking = lm8502_flash_set;
	chip->flash_cdev.max_brightness = LED_FULL;
	chip->flash_cdev.flags = LED_DEV_CAP_FLASH;

	ret = devm_led_classdev_register(dev, &chip->flash_cdev);
	if (ret)
		dev_warn(dev, "Failed to register flash LED\n");

	/* Register torch LED */
	chip->torch_cdev.name = "lm8502:torch";
	chip->torch_cdev.brightness_set_blocking = lm8502_torch_set;
	chip->torch_cdev.max_brightness = LED_FULL;

	ret = devm_led_classdev_register(dev, &chip->torch_cdev);
	if (ret)
		dev_warn(dev, "Failed to register torch LED\n");

	/* Setup interrupt */
	irq = of_irq_get(dev->of_node, 0);
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, lm8502_irq_handler,
				       IRQF_TRIGGER_FALLING, "lm8502", chip);
		if (ret)
			dev_warn(dev, "Failed to request IRQ\n");
	}

	/* Default engine pages */
	chip->engine_startpage[0] = 0;
	chip->engine_endpage[0] = 2;
	chip->engine_startpage[1] = 3;
	chip->engine_endpage[1] = 5;

	i2c_set_clientdata(client, chip);

	dev_info(dev, "LM8502 with %d LEDs, vibrator, and flash\n", chip->num_leds);

	return 0;
}

static void lm8502_remove(struct i2c_client *client)
{
	struct lm8502 *chip = i2c_get_clientdata(client);
	int i;

	/* Stop engines */
	mutex_lock(&chip->lock);
	lm8502_stop_engine(chip, 1);
	lm8502_stop_engine(chip, 2);

	/* Turn off all LEDs */
	for (i = 0; i < chip->num_leds; i++) {
		regmap_write(chip->regmap, chip->leds[i].control_reg, 0);
		regmap_write(chip->regmap, chip->leds[i].current_reg, 0);
	}

	/* Disable vibrator and flash */
	regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
	regmap_write(chip->regmap, LM8502_FLASH_BRIGHTNESS, 0);

	mutex_unlock(&chip->lock);

	/* Disable chip */
	if (chip->enable_gpio)
		gpiod_set_value_cansleep(chip->enable_gpio, 0);
}

static void lm8502_shutdown(struct i2c_client *client)
{
	lm8502_remove(client);
}

#ifdef CONFIG_PM_SLEEP
static int lm8502_suspend(struct device *dev)
{
	struct lm8502 *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	lm8502_stop_engine(chip, 1);
	lm8502_stop_engine(chip, 2);
	regmap_write(chip->regmap, LM8502_HAPTIC_FEEDBACK_CTRL, 0);
	mutex_unlock(&chip->lock);

	if (chip->enable_gpio)
		gpiod_set_value_cansleep(chip->enable_gpio, 0);

	return 0;
}

static int lm8502_resume(struct device *dev)
{
	struct lm8502 *chip = dev_get_drvdata(dev);

	return lm8502_init_hw(chip);
}
#endif

static SIMPLE_DEV_PM_OPS(lm8502_pm_ops, lm8502_suspend, lm8502_resume);

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
		.pm = &lm8502_pm_ops,
	},
	.probe = lm8502_probe,
	.remove = lm8502_remove,
	.shutdown = lm8502_shutdown,
	.id_table = lm8502_id,
};
module_i2c_driver(lm8502_driver);

MODULE_DESCRIPTION("TI LM8502 LED Controller Driver");
MODULE_AUTHOR("Palm Inc.");
MODULE_AUTHOR("webOS Community");
MODULE_LICENSE("GPL");
