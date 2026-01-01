// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Cypress CY8CTMA300 capacitive touchscreen
 *
 * Copyright (C) 2024 webOS Community
 *
 * Based on cy8ctma140.c by Linus Walleij and cy8c_ts.c from
 * the legacy HP Pre3 kernel.
 *
 * The CY8CTMA300 is a capacitive touch controller used in devices
 * like the HP Pre3 smartphone. It supports up to 10 simultaneous
 * touch points and communicates over I2C.
 *
 * Register map:
 *   0x01 - Status register
 *   0x03 - Touch data register
 *
 * Touch data format (8 bytes per touch):
 *   [0] - Touch status/ID
 *   [1] - Reserved
 *   [2] - Reserved
 *   [3] - Z (pressure)
 *   [4] - Y high byte
 *   [5] - Y low byte
 *   [6] - X high byte
 *   [7] - X low byte
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#define CY8CTMA300_NAME			"cy8ctma300"

#define CY8CTMA300_MAX_FINGERS		10

/* Register addresses */
#define CY8CTMA300_REG_STATUS		0x01
#define CY8CTMA300_REG_DATA		0x03

/* Status register bits */
#define CY8CTMA300_STATUS_BUSY		BIT(7)

/* Touch data packet size per finger */
#define CY8CTMA300_TOUCH_SIZE		8

/* Maximum packet size: header + 10 fingers */
#define CY8CTMA300_MAX_PACKET_SIZE	(2 + CY8CTMA300_MAX_FINGERS * CY8CTMA300_TOUCH_SIZE)

/* Offsets within each touch record */
#define CY8CTMA300_TOUCH_STATUS		0
#define CY8CTMA300_TOUCH_Z		3
#define CY8CTMA300_TOUCH_Y_HI		4
#define CY8CTMA300_TOUCH_Y_LO		5
#define CY8CTMA300_TOUCH_X_HI		6
#define CY8CTMA300_TOUCH_X_LO		7

struct cy8ctma300 {
	struct input_dev *input;
	struct touchscreen_properties props;
	struct device *dev;
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data regulators[2];
};

static int cy8ctma300_read_touch_data(struct cy8ctma300 *ts, u8 *buf, int len)
{
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = 1,
			.buf = (u8[]){ CY8CTMA300_REG_DATA },
		}, {
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(ts->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		if (ret < 0)
			return ret;
		return -EIO;
	}

	return 0;
}

static void cy8ctma300_report_touch(struct cy8ctma300 *ts, u8 *touch_data,
				    int id, int slot)
{
	u16 x, y;
	u8 z;

	x = (touch_data[CY8CTMA300_TOUCH_X_HI] << 8) |
	     touch_data[CY8CTMA300_TOUCH_X_LO];
	y = (touch_data[CY8CTMA300_TOUCH_Y_HI] << 8) |
	     touch_data[CY8CTMA300_TOUCH_Y_LO];
	z = touch_data[CY8CTMA300_TOUCH_Z];

	dev_dbg(ts->dev, "touch %d: slot %d, x=%u, y=%u, z=%u\n",
		id, slot, x, y, z);

	input_mt_slot(ts->input, slot);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input, &ts->props, x, y, true);
	input_report_abs(ts->input, ABS_MT_PRESSURE, z);
}

static irqreturn_t cy8ctma300_irq_thread(int irq, void *d)
{
	struct cy8ctma300 *ts = d;
	u8 buf[CY8CTMA300_MAX_PACKET_SIZE];
	int n_fingers;
	int ret;
	int i;

	/* Read the header to get number of fingers */
	ret = cy8ctma300_read_touch_data(ts, buf, 2);
	if (ret) {
		dev_err(ts->dev, "failed to read touch header: %d\n", ret);
		goto out;
	}

	n_fingers = buf[0] & 0x0f;
	if (n_fingers > CY8CTMA300_MAX_FINGERS) {
		dev_warn(ts->dev, "invalid finger count: %d\n", n_fingers);
		n_fingers = CY8CTMA300_MAX_FINGERS;
	}

	if (n_fingers > 0) {
		/* Read touch data for all fingers */
		int data_len = n_fingers * CY8CTMA300_TOUCH_SIZE;

		ret = cy8ctma300_read_touch_data(ts, buf, 2 + data_len);
		if (ret) {
			dev_err(ts->dev, "failed to read touch data: %d\n", ret);
			goto out;
		}

		for (i = 0; i < n_fingers; i++) {
			u8 *touch = &buf[2 + i * CY8CTMA300_TOUCH_SIZE];
			int id = touch[CY8CTMA300_TOUCH_STATUS] & 0x0f;
			int slot;

			slot = input_mt_get_slot_by_key(ts->input, id);
			if (slot < 0)
				continue;

			cy8ctma300_report_touch(ts, touch, id, slot);
		}
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

out:
	return IRQ_HANDLED;
}

static void cy8ctma300_reset(struct cy8ctma300 *ts)
{
	if (!ts->reset_gpio)
		return;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(10);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(100);
}

static int cy8ctma300_power_up(struct cy8ctma300 *ts)
{
	int error;

	error = regulator_bulk_enable(ARRAY_SIZE(ts->regulators),
				      ts->regulators);
	if (error) {
		dev_err(ts->dev, "failed to enable regulators: %d\n", error);
		return error;
	}

	msleep(50);
	cy8ctma300_reset(ts);

	return 0;
}

static void cy8ctma300_power_down(struct cy8ctma300 *ts)
{
	if (ts->reset_gpio)
		gpiod_set_value_cansleep(ts->reset_gpio, 1);

	regulator_bulk_disable(ARRAY_SIZE(ts->regulators), ts->regulators);
}

static void cy8ctma300_power_off_action(void *d)
{
	struct cy8ctma300 *ts = d;

	cy8ctma300_power_down(ts);
}

static int cy8ctma300_probe(struct i2c_client *client)
{
	struct cy8ctma300 *ts;
	struct input_dev *input;
	struct device *dev = &client->dev;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		dev_err(dev, "I2C adapter doesn't support required functionality\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	ts->dev = dev;
	ts->client = client;
	ts->input = input;

	/* Get reset GPIO (optional) */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Get regulators */
	ts->regulators[0].supply = "vcpin";
	ts->regulators[1].supply = "vdd";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->regulators),
					ts->regulators);
	if (error)
		return dev_err_probe(dev, error, "failed to get regulators\n");

	/* Setup input device */
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(input, true, &ts->props);
	input_abs_set_fuzz(input, ABS_MT_POSITION_X, 0);
	input_abs_set_fuzz(input, ABS_MT_POSITION_Y, 0);

	error = input_mt_init_slots(input, CY8CTMA300_MAX_FINGERS,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	input->name = CY8CTMA300_NAME;
	input->id.bustype = BUS_I2C;
	input_set_drvdata(input, ts);

	/* Power up the device */
	error = cy8ctma300_power_up(ts);
	if (error)
		return error;

	error = devm_add_action_or_reset(dev, cy8ctma300_power_off_action, ts);
	if (error)
		return error;

	/* Request interrupt */
	error = devm_request_threaded_irq(dev, client->irq,
					  NULL, cy8ctma300_irq_thread,
					  IRQF_ONESHOT,
					  CY8CTMA300_NAME, ts);
	if (error) {
		dev_err(dev, "failed to request IRQ %d: %d\n",
			client->irq, error);
		return error;
	}

	error = input_register_device(input);
	if (error)
		return error;

	i2c_set_clientdata(client, ts);

	dev_info(dev, "Cypress CY8CTMA300 touchscreen initialized\n");

	return 0;
}

static int cy8ctma300_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8ctma300 *ts = i2c_get_clientdata(client);

	disable_irq(client->irq);

	if (!device_may_wakeup(dev))
		cy8ctma300_power_down(ts);

	return 0;
}

static int cy8ctma300_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8ctma300 *ts = i2c_get_clientdata(client);
	int error;

	if (!device_may_wakeup(dev)) {
		error = cy8ctma300_power_up(ts);
		if (error)
			return error;
	}

	enable_irq(client->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cy8ctma300_pm,
				cy8ctma300_suspend, cy8ctma300_resume);

static const struct i2c_device_id cy8ctma300_idtable[] = {
	{ CY8CTMA300_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cy8ctma300_idtable);

static const struct of_device_id cy8ctma300_of_match[] = {
	{ .compatible = "cypress,cy8ctma300" },
	{ }
};
MODULE_DEVICE_TABLE(of, cy8ctma300_of_match);

static struct i2c_driver cy8ctma300_driver = {
	.driver = {
		.name = CY8CTMA300_NAME,
		.pm = pm_sleep_ptr(&cy8ctma300_pm),
		.of_match_table = cy8ctma300_of_match,
	},
	.id_table = cy8ctma300_idtable,
	.probe = cy8ctma300_probe,
};
module_i2c_driver(cy8ctma300_driver);

MODULE_AUTHOR("webOS Community");
MODULE_DESCRIPTION("Cypress CY8CTMA300 Touchscreen Driver");
MODULE_LICENSE("GPL");
