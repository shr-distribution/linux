// SPDX-License-Identifier: GPL-2.0-only
/*
 * Capella CM36652 ambient light and proximity sensor
 *
 * Register numbers, the configuration words and the threshold encoding come
 * from the vendor driver in Samsung's msm8916 trees (drivers/sensors/cm36652.c)
 * and from the per-board properties its device tree node carries.
 *
 * The part has no identification register, so probe verifies the device is
 * there by reading a configuration register back instead.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#define CM36652_REG_CS_CONF1		0x00
#define CM36652_REG_PS_CONF1		0x03
#define CM36652_REG_PS_CONF3		0x04
#define CM36652_REG_PS_THD		0x05
#define CM36652_REG_PS_CANC		0x06
#define CM36652_REG_PS_DATA		0x07
#define CM36652_REG_ALS_RED		0x08
#define CM36652_REG_ALS_GREEN		0x09
#define CM36652_REG_ALS_BLUE		0x0a
#define CM36652_REG_ALS_WHITE		0x0b

/* Bit 0 of either configuration register shuts that half of the chip down. */
#define CM36652_SD			BIT(0)

/*
 * The proximity configuration word the vendor programs: integration time,
 * LED drive current and interrupt persistence in one register. Kept as the
 * vendor's value because the individual fields are not documented anywhere
 * public, and a wrong LED current here is a hardware concern rather than
 * merely a wrong reading.
 */
#define CM36652_PS_CONF1_DEFAULT	0x5308
#define CM36652_PS_CONF3_DEFAULT	0x0000

/* Thresholds pack as (high << 8) | low. The A3 (2015) uses 20 and 17. */
#define CM36652_PS_THD_DEFAULT		0x1411

struct cm36652 {
	struct i2c_client *client;
	struct gpio_desc *leden_gpio;
	struct regulator_bulk_data supplies[2];
};

static int cm36652_write(struct cm36652 *data, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(data->client, reg, val);
}

static int cm36652_read(struct cm36652 *data, u8 reg)
{
	return i2c_smbus_read_word_data(data->client, reg);
}

static int cm36652_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct cm36652 *data = iio_priv(indio_dev);
	int ret;
	u8 reg;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	switch (chan->type) {
	case IIO_LIGHT:
		/*
		 * The vendor driver reports the green channel as the ambient
		 * light reading with no conversion of its own, so expose it as
		 * illuminance directly rather than inventing a scale.
		 */
		reg = CM36652_REG_ALS_GREEN;
		break;
	case IIO_INTENSITY:
		switch (chan->channel2) {
		case IIO_MOD_LIGHT_RED:
			reg = CM36652_REG_ALS_RED;
			break;
		case IIO_MOD_LIGHT_GREEN:
			reg = CM36652_REG_ALS_GREEN;
			break;
		case IIO_MOD_LIGHT_BLUE:
			reg = CM36652_REG_ALS_BLUE;
			break;
		case IIO_MOD_LIGHT_CLEAR:
			reg = CM36652_REG_ALS_WHITE;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_PROXIMITY:
		reg = CM36652_REG_PS_DATA;
		break;
	default:
		return -EINVAL;
	}

	ret = cm36652_read(data, reg);
	if (ret < 0)
		return ret;

	/*
	 * Proximity is reported in the low byte; the high byte carries the
	 * interrupt flags, which would otherwise show up as a huge reading.
	 */
	*val = chan->type == IIO_PROXIMITY ? (ret & 0xff) : ret;

	return IIO_VAL_INT;
}

static const struct iio_info cm36652_info = {
	.read_raw = cm36652_read_raw,
};

#define CM36652_INTENSITY_CHANNEL(_mod)					\
{									\
	.type = IIO_INTENSITY,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_LIGHT_##_mod,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
}

static const struct iio_chan_spec cm36652_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	CM36652_INTENSITY_CHANNEL(RED),
	CM36652_INTENSITY_CHANNEL(GREEN),
	CM36652_INTENSITY_CHANNEL(BLUE),
	CM36652_INTENSITY_CHANNEL(CLEAR),
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int cm36652_power_on(struct cm36652 *data)
{
	int ret;

	/* Clearing SD in both configuration registers starts the chip. */
	ret = cm36652_write(data, CM36652_REG_PS_CONF1,
			    CM36652_PS_CONF1_DEFAULT);
	if (ret)
		return ret;

	ret = cm36652_write(data, CM36652_REG_PS_CONF3,
			    CM36652_PS_CONF3_DEFAULT);
	if (ret)
		return ret;

	ret = cm36652_write(data, CM36652_REG_PS_THD, CM36652_PS_THD_DEFAULT);
	if (ret)
		return ret;

	ret = cm36652_write(data, CM36652_REG_PS_CANC, 0x0000);
	if (ret)
		return ret;

	return cm36652_write(data, CM36652_REG_CS_CONF1, 0x0000);
}

static void cm36652_power_off(void *p)
{
	struct cm36652 *data = p;

	cm36652_write(data, CM36652_REG_CS_CONF1, CM36652_SD);
	cm36652_write(data, CM36652_REG_PS_CONF1, CM36652_SD);

	/* The infrared emitter the proximity half uses is switched separately. */
	gpiod_set_value_cansleep(data->leden_gpio, 0);
}

static void cm36652_supplies_disable(void *p)
{
	struct cm36652 *data = p;

	regulator_bulk_disable(ARRAY_SIZE(data->supplies), data->supplies);
}

static int cm36652_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct cm36652 *data;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;

	data->supplies[0].supply = "vdd";
	data->supplies[1].supply = "vio";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->supplies),
				      data->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(data->supplies), data->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, cm36652_supplies_disable, data);
	if (ret)
		return ret;

	data->leden_gpio = devm_gpiod_get_optional(dev, "leden", GPIOD_OUT_HIGH);
	if (IS_ERR(data->leden_gpio))
		return dev_err_probe(dev, PTR_ERR(data->leden_gpio),
				     "Failed to get the LED enable gpio\n");

	/*
	 * No identification register exists, so confirm the device answers by
	 * reading a configuration register instead of trusting the bus.
	 */
	ret = cm36652_read(data, CM36652_REG_CS_CONF1);
	if (ret < 0)
		return dev_err_probe(dev, ret, "No device at this address\n");

	ret = cm36652_power_on(data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialise the sensor\n");

	ret = devm_add_action_or_reset(dev, cm36652_power_off, data);
	if (ret)
		return ret;

	indio_dev->name = "cm36652";
	indio_dev->info = &cm36652_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = cm36652_channels;
	indio_dev->num_channels = ARRAY_SIZE(cm36652_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id cm36652_id[] = {
	{ "cm36652" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cm36652_id);

static const struct of_device_id cm36652_of_match[] = {
	{ .compatible = "capella,cm36652" },
	{ }
};
MODULE_DEVICE_TABLE(of, cm36652_of_match);

static struct i2c_driver cm36652_driver = {
	.driver = {
		.name = "cm36652",
		.of_match_table = cm36652_of_match,
	},
	.probe = cm36652_probe,
	.id_table = cm36652_id,
};
module_i2c_driver(cm36652_driver);

MODULE_DESCRIPTION("Capella CM36652 light and proximity sensor driver");
MODULE_LICENSE("GPL");
