/*
 * drivers/media/video/adp1653.c
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
 *          Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * NOTES:
 * - Torch and Indicator lights are enabled by just increasing
 *   intensity from zero
 * - Increasing Flash light intensity does nothing until it is
 *   strobed (strobe control set to 1)
 * - Strobing flash disables Torch light (sets intensity to zero).
 *   This might be changed later.
 *
 * TODO:
 * - fault interrupt handling
 * - faster strobe (use i/o pin instead of i2c)
 *   - should ensure that the pin is in some sane state even if not used
 * - strobe control could return whether flash is still on (measure time)
 * - power doesn't need to be ON if all lights are off
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <media/adp1653.h>
#include <media/smiaregs.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>

#define TIMEOUT_US_TO_CODE(t)	((820000 + 27300 - (t))/54600)
#define TIMEOUT_CODE_TO_US(c)	(820000 - (c) * 54600)

/* Write values into ADP1653 registers. Do nothing if power is off. */
static int adp1653_update_hw(struct adp1653_flash *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);
	u8 out_sel;
	u8 config;
	int rval;

	if (!flash->power)
		return 0;

	out_sel = flash->indicator_intensity << ADP1653_REG_OUT_SEL_ILED_SHIFT;
	/* Set torch intensity to zero--prevents false triggering of SC Fault */
	rval = i2c_smbus_write_byte_data(client, ADP1653_REG_OUT_SEL, out_sel);
	if (rval < 0)
		return rval;

	if (flash->torch_intensity > 0) {
		/* Torch mode, light immediately on, duration indefinite */
		out_sel |= flash->torch_intensity
			   << ADP1653_REG_OUT_SEL_HPLED_SHIFT;
		config = 0;
	} else {
		/* Flash mode, light on with strobe, duration from timer */
		out_sel |= flash->flash_intensity
			   << ADP1653_REG_OUT_SEL_HPLED_SHIFT;
		config = ADP1653_REG_CONFIG_TMR_CFG;
		config |= TIMEOUT_US_TO_CODE(flash->flash_timeout)
			  << ADP1653_REG_CONFIG_TMR_SET_SHIFT;
	}

	rval = i2c_smbus_write_byte_data(client, ADP1653_REG_OUT_SEL, out_sel);
	if (rval < 0)
		return rval;

	rval = i2c_smbus_write_byte_data(client, ADP1653_REG_CONFIG, config);
	if (rval < 0)
		return rval;

	return 0;
}

static int adp1653_strobe(struct adp1653_flash *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);
	int rval;

	if (flash->torch_intensity > 0) {
		/* Disabling torch enables flash in update_hw() */
		flash->torch_intensity = 0;
		rval = adp1653_update_hw(flash);
		if (rval)
			return rval;
	}

	if (flash->platform_data->strobe)
		/* Hardware-specific strobe using I/O pin */
		return flash->platform_data->strobe(&flash->subdev);

	/* Software strobe using i2c */
	rval = i2c_smbus_write_byte_data(client, ADP1653_REG_SW_STROBE,
		ADP1653_REG_SW_STROBE_SW_STROBE);
	if (rval)
		return rval;
	return i2c_smbus_write_byte_data(client, ADP1653_REG_SW_STROBE, 0);
}

static int adp1653_get_fault(struct adp1653_flash *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);

	return i2c_smbus_read_byte_data(client, ADP1653_REG_FAULT);
}

/* --------------------------------------------------------------------------
 * V4L2 subdev operations
 */
static int
adp1653_get_chip_ident(struct v4l2_subdev *subdev,
		       struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_ADP1653, 0);
}

#define CTRL_FLASH_STROBE			0
#define CTRL_FLASH_TIMEOUT			1
#define CTRL_FLASH_INTENSITY			2
#define CTRL_TORCH_INTENSITY			3
#define CTRL_INDICATOR_INTENSITY		4
#define CTRL_FLASH_FAULT_SCP			5
#define CTRL_FLASH_FAULT_OT			6
#define CTRL_FLASH_FAULT_TMR			7
#define CTRL_FLASH_FAULT_OV			8

static const struct v4l2_queryctrl adp1653_ctrls[] = {
	{
		.id		= V4L2_CID_FLASH_STROBE,
		.type		= V4L2_CTRL_TYPE_BUTTON,
		.name		= "Flash strobe",
		.minimum	= 0,
		.maximum	= 0,
		.step		= 0,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_UPDATE,
	},

	{
		.id		= V4L2_CID_FLASH_TIMEOUT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash timeout [us]",
		.minimum	= 1000,
		.maximum	= 820000,
		.step		= 54600,
		.default_value	= 1000,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.id		= V4L2_CID_FLASH_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash intensity",
		.minimum	= ADP1653_FLASH_INTENSITY_MIN,
		.minimum	= ADP1653_FLASH_INTENSITY_MAX,
		.step		= 1,
		.default_value	= ADP1653_FLASH_INTENSITY_MIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.id		= V4L2_CID_TORCH_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Torch intensity",
		.minimum	= ADP1653_TORCH_INTENSITY_MIN,
		.maximum	= ADP1653_TORCH_INTENSITY_MAX,
		.step		= 1,
		.default_value	= ADP1653_TORCH_INTENSITY_MIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.id		= V4L2_CID_INDICATOR_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Indicator intensity",
		.minimum	= ADP1653_INDICATOR_INTENSITY_MIN,
		.maximum	= ADP1653_INDICATOR_INTENSITY_MAX,
		.step		= 1,
		.default_value	= ADP1653_INDICATOR_INTENSITY_MIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},

	/* Faults */
	{
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_SCP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Short-circuit fault",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_OT,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Overtemperature fault",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_TMR,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Timeout fault",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_OV,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Overvoltage fault",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
	}
};

static int
adp1653_query_ctrl(struct v4l2_subdev *subdev, struct v4l2_queryctrl *ctrl)
{
	struct adp1653_flash *flash = to_adp1653_flash(subdev);
	int rval;

	rval = smia_ctrl_query(adp1653_ctrls, ARRAY_SIZE(adp1653_ctrls), ctrl);
	if (rval < 0)
		return rval;

	/* Override global values with platform-specific data. */
	switch (ctrl->id) {
	case V4L2_CID_FLASH_TIMEOUT:
		ctrl->maximum = flash->platform_data->max_flash_timeout;
		ctrl->default_value = flash->platform_data->max_flash_timeout;
		break;
	case V4L2_CID_FLASH_INTENSITY:
		ctrl->maximum = flash->platform_data->max_flash_intensity;
		break;
	case V4L2_CID_TORCH_INTENSITY:
		ctrl->maximum = flash->platform_data->max_torch_intensity;
		break;
	case V4L2_CID_INDICATOR_INTENSITY:
		ctrl->maximum = flash->platform_data->max_indicator_intensity;
		break;
	}

	return 0;
}

static int
adp1653_get_ctrl(struct v4l2_subdev *subdev, struct v4l2_control *vc)
{
	struct adp1653_flash *flash = to_adp1653_flash(subdev);

	switch (vc->id) {
	case V4L2_CID_FLASH_TIMEOUT:
		vc->value = flash->flash_timeout;
		break;
	case V4L2_CID_FLASH_INTENSITY:
		vc->value = flash->flash_intensity;
		break;
	case V4L2_CID_TORCH_INTENSITY:
		vc->value = flash->torch_intensity;
		break;
	case V4L2_CID_INDICATOR_INTENSITY:
		vc->value = flash->indicator_intensity;
		break;

	case V4L2_CID_FLASH_ADP1653_FAULT_SCP:
		vc->value = (adp1653_get_fault(flash)
			    & ADP1653_REG_FAULT_FLT_SCP) != 0;
		break;
	case V4L2_CID_FLASH_ADP1653_FAULT_OT:
		vc->value = (adp1653_get_fault(flash)
			    & ADP1653_REG_FAULT_FLT_OT) != 0;
		break;
	case V4L2_CID_FLASH_ADP1653_FAULT_TMR:
		vc->value = (adp1653_get_fault(flash)
			    & ADP1653_REG_FAULT_FLT_TMR) != 0;
		break;
	case V4L2_CID_FLASH_ADP1653_FAULT_OV:
		vc->value = (adp1653_get_fault(flash)
			    & ADP1653_REG_FAULT_FLT_OV) != 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int
adp1653_set_ctrl(struct v4l2_subdev *subdev, struct v4l2_control *vc)
{
	struct adp1653_flash *flash = to_adp1653_flash(subdev);
	const struct v4l2_queryctrl *ctrl;
	unsigned int index;
	s32 maximum;
	u32 *value;

	switch (vc->id) {
	case V4L2_CID_FLASH_STROBE:
		return adp1653_strobe(flash);

	case V4L2_CID_FLASH_TIMEOUT:
		index = CTRL_FLASH_TIMEOUT;
		maximum = flash->platform_data->max_flash_timeout;
		value = &flash->flash_timeout;
		break;
	case V4L2_CID_FLASH_INTENSITY:
		index = CTRL_FLASH_INTENSITY;
		maximum = flash->platform_data->max_flash_intensity;
		value = &flash->flash_intensity;
		break;
	case V4L2_CID_TORCH_INTENSITY:
		index = CTRL_TORCH_INTENSITY;
		maximum = flash->platform_data->max_torch_intensity;
		value = &flash->torch_intensity;
		break;
	case V4L2_CID_INDICATOR_INTENSITY:
		index = CTRL_INDICATOR_INTENSITY;
		maximum = flash->platform_data->max_indicator_intensity;
		value = &flash->indicator_intensity;
		break;

	default:
		return -EINVAL;
	}

	ctrl = &adp1653_ctrls[index];
	vc->value = clamp(vc->value, ctrl->minimum, maximum);
	vc->value = DIV_ROUND_CLOSEST(vc->value - ctrl->minimum, ctrl->step);
	vc->value = vc->value * ctrl->step + ctrl->minimum;
	*value = vc->value;

	return adp1653_update_hw(flash);
}

static int
adp1653_set_config(struct v4l2_subdev *subdev, int irq, void *platform_data)
{
	struct adp1653_flash *flash = to_adp1653_flash(subdev);

	if (platform_data == NULL)
		return -EINVAL;

	flash->platform_data = platform_data;

	flash->flash_timeout =
		flash->platform_data->max_flash_timeout;
	flash->flash_intensity =
		adp1653_ctrls[CTRL_FLASH_INTENSITY].default_value;
	flash->torch_intensity =
		adp1653_ctrls[CTRL_TORCH_INTENSITY].default_value;
	flash->indicator_intensity =
		adp1653_ctrls[CTRL_INDICATOR_INTENSITY].default_value;

	return 0;
}

static int
adp1653_init_device(struct adp1653_flash *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);
	int rval;

	/* Clear FAULT register by writing zero to OUT_SEL */
	rval = i2c_smbus_write_byte_data(client, ADP1653_REG_OUT_SEL, 0);
	if (rval < 0) {
		dev_err(&client->dev, "failed writing fault register\n");
		return -EIO;
	}

	/* Read FAULT register */
	rval = i2c_smbus_read_byte_data(client, ADP1653_REG_FAULT);
	if (rval < 0) {
		dev_err(&client->dev, "failed reading fault register\n");
		return -EIO;
	}

	if ((rval & 0x0f) != 0) {
		dev_err(&client->dev, "device fault\n");
		return -EIO;
	}

	rval = adp1653_update_hw(flash);
	if (rval) {
		dev_err(&client->dev,
			"adp1653_update_hw failed at %s\n", __func__);
		return -EIO;
	}

	return 0;
}

static int
adp1653_set_power(struct v4l2_subdev *subdev, int on)
{
	struct adp1653_flash *flash = to_adp1653_flash(subdev);
	int rval = 0;

	if (on == flash->power)
		return 0;

	rval = flash->platform_data->power(subdev, on);
	if (rval)
		return rval;

	flash->power = on;
	if (!on)
		return 0;

	rval = adp1653_init_device(flash);
	if (rval) {
		flash->platform_data->power(subdev, 0);
		flash->power = 0;
	}

	return rval;
}

static const struct v4l2_subdev_core_ops adp1653_core_ops = {
	.g_chip_ident = adp1653_get_chip_ident,
	.s_config = adp1653_set_config,
	.queryctrl = adp1653_query_ctrl,
	.g_ctrl = adp1653_get_ctrl,
	.s_ctrl = adp1653_set_ctrl,
	.s_power = adp1653_set_power,
};

static const struct v4l2_subdev_ops adp1653_ops = {
	.core = &adp1653_core_ops,
};

/* --------------------------------------------------------------------------
 * I2C driver
 */
#ifdef CONFIG_PM

static int adp1653_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct adp1653_flash *flash = to_adp1653_flash(subdev);

	if (!flash->power)
		return 0;

	return flash->platform_data->power(subdev, 0);
}

static int adp1653_resume(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct adp1653_flash *flash = to_adp1653_flash(subdev);

	if (!flash->power)
		return 0;

	flash->power = 0;
	return adp1653_set_power(subdev, 1);
}

#else

#define adp1653_suspend	NULL
#define adp1653_resume	NULL

#endif /* CONFIG_PM */

static const struct media_entity_operations adp1653_entity_ops = {
	.set_power = v4l2_subdev_set_power,
};

static int adp1653_probe(struct i2c_client *client,
			 const struct i2c_device_id *devid)
{
	struct adp1653_flash *flash;
	int ret;

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&flash->subdev, client, &adp1653_ops);
	flash->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	flash->subdev.entity.ops = &adp1653_entity_ops;
	ret = media_entity_init(&flash->subdev.entity, 0, NULL, 0);
	if (ret < 0)
		kfree(flash);

	return ret;
}

static int __exit adp1653_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct adp1653_flash *flash = to_adp1653_flash(subdev);

	v4l2_device_unregister_subdev(&flash->subdev);
	media_entity_cleanup(&flash->subdev.entity);
	kfree(flash);
	return 0;
}

static const struct i2c_device_id adp1653_id_table[] = {
	{ ADP1653_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adp1653_id_table);

static struct i2c_driver adp1653_i2c_driver = {
	.driver		= {
		.name	= ADP1653_NAME,
	},
	.probe		= adp1653_probe,
	.remove		= __exit_p(adp1653_remove),
	.suspend	= adp1653_suspend,
	.resume		= adp1653_resume,
	.id_table	= adp1653_id_table,
};

static int __init adp1653_init(void)
{
	int rval;

	rval = i2c_add_driver(&adp1653_i2c_driver);
	if (rval)
		printk(KERN_ALERT "%s: failed at i2c_add_driver\n", __func__);

	return rval;
}

static void __exit adp1653_exit(void)
{
	i2c_del_driver(&adp1653_i2c_driver);
}

module_init(adp1653_init);
module_exit(adp1653_exit);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@nokia.com>");
MODULE_DESCRIPTION("Analog Devices ADP1653 LED flash driver");
MODULE_LICENSE("GPL");
