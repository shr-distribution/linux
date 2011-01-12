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
 * V4L2 controls
 */

static int adp1653_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adp1653_flash *flash =
		container_of(ctrl->handler, struct adp1653_flash, ctrls);
	int fault;

	fault = adp1653_get_fault(flash);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_ADP1653_FAULT_SCP:
		ctrl->cur.val = !!(fault & ADP1653_REG_FAULT_FLT_SCP);
		break;
	case V4L2_CID_FLASH_ADP1653_FAULT_OT:
		ctrl->cur.val = !!(fault & ADP1653_REG_FAULT_FLT_OT);
		break;
	case V4L2_CID_FLASH_ADP1653_FAULT_TMR:
		ctrl->cur.val = !!(fault & ADP1653_REG_FAULT_FLT_TMR);
		break;
	case V4L2_CID_FLASH_ADP1653_FAULT_OV:
		ctrl->cur.val = !!(fault & ADP1653_REG_FAULT_FLT_OV);
		break;
	}

	return 0;
}

static int adp1653_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adp1653_flash *flash =
		container_of(ctrl->handler, struct adp1653_flash, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_STROBE:
		return adp1653_strobe(flash);
	case V4L2_CID_FLASH_TIMEOUT:
		flash->flash_timeout = ctrl->val;
		break;
	case V4L2_CID_FLASH_INTENSITY:
		flash->flash_intensity = ctrl->val;
		break;
	case V4L2_CID_TORCH_INTENSITY:
		flash->torch_intensity = ctrl->val;
		break;
	case V4L2_CID_INDICATOR_INTENSITY:
		flash->indicator_intensity = ctrl->val;
		break;
	}

	return adp1653_update_hw(flash);
}

static const struct v4l2_ctrl_ops adp1653_ctrl_ops = {
	.g_volatile_ctrl = adp1653_get_ctrl,
	.s_ctrl = adp1653_set_ctrl,
};

static const struct v4l2_ctrl_config adp1653_ctrls[] = {
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_STROBE,
		.type		= V4L2_CTRL_TYPE_BUTTON,
		.name		= "Flash strobe",
		.min		= 0,
		.max		= 1,
		.step		= 0,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_UPDATE,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_TIMEOUT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash timeout [us]",
		.min		= 1000,
		.max		= 820000,
		.step		= 54600,
		.def		= 1000,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash intensity",
		.min		= ADP1653_FLASH_INTENSITY_MIN,
		.max		= ADP1653_FLASH_INTENSITY_MAX,
		.step		= 1,
		.def		= ADP1653_FLASH_INTENSITY_MIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_TORCH_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Torch intensity",
		.min		= ADP1653_TORCH_INTENSITY_MIN,
		.max		= ADP1653_TORCH_INTENSITY_MAX,
		.step		= 1,
		.def		= ADP1653_TORCH_INTENSITY_MIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_INDICATOR_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Indicator intensity",
		.min		= ADP1653_INDICATOR_INTENSITY_MIN,
		.max		= ADP1653_INDICATOR_INTENSITY_MAX,
		.step		= 1,
		.def		= ADP1653_INDICATOR_INTENSITY_MIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	/* Faults */
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_SCP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Short-circuit fault",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.is_volatile	= 1,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_OT,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Overtemperature fault",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.is_volatile	= 1,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_TMR,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Timeout fault",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.is_volatile	= 1,
	},
	{
		.ops		= &adp1653_ctrl_ops,
		.id		= V4L2_CID_FLASH_ADP1653_FAULT_OV,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Overvoltage fault",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.is_volatile	= 1,
	}
};

static int adp1653_init_controls(struct adp1653_flash *flash)
{
	struct v4l2_ctrl *ctrl;
	unsigned int i;

	v4l2_ctrl_handler_init(&flash->ctrls, ARRAY_SIZE(adp1653_ctrls));

	for (i = 0; i < ARRAY_SIZE(adp1653_ctrls); ++i) {
		v4l2_ctrl_new_custom(&flash->ctrls, &adp1653_ctrls[i], NULL);
		if (flash->ctrls.error) {
			printk(KERN_INFO "%s: error registering control %u\n", __func__, i);
			break;
		}
	}

	if (flash->ctrls.error)
		return flash->ctrls.error;

	/* Update the controls limits using the value passed through platform
	 * data, and initialize the parameters stored in the adp1653 structure
	 * with default values.
	 */

	/* V4L2_CID_FLASH_TIMEOUT */
	ctrl = v4l2_ctrl_find(&flash->ctrls, V4L2_CID_FLASH_TIMEOUT);
	ctrl->maximum = flash->platform_data->max_flash_timeout;
	ctrl->default_value = ctrl->maximum;
	ctrl->cur.val = ctrl->default_value;
	ctrl->val = ctrl->default_value;
	flash->flash_timeout = ctrl->default_value;

	/* V4L2_CID_FLASH_INTENSITY */
	ctrl = v4l2_ctrl_find(&flash->ctrls, V4L2_CID_FLASH_INTENSITY);
	ctrl->maximum = flash->platform_data->max_flash_intensity;
	flash->flash_intensity = ctrl->default_value;

	/* V4L2_CID_TORCH_INTENSITY */
	ctrl = v4l2_ctrl_find(&flash->ctrls, V4L2_CID_TORCH_INTENSITY);
	ctrl->maximum = flash->platform_data->max_torch_intensity;
	flash->torch_intensity = ctrl->default_value;

	/* V4L2_CID_INDICATOR_INTENSITY */
	ctrl = v4l2_ctrl_find(&flash->ctrls, V4L2_CID_INDICATOR_INTENSITY);
	ctrl->maximum = flash->platform_data->max_indicator_intensity;
	flash->indicator_intensity = ctrl->default_value;

	flash->subdev.ctrl_handler = &flash->ctrls;
	return 0;
}

/* --------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static int
adp1653_set_config(struct v4l2_subdev *subdev, int irq, void *platform_data)
{
	struct adp1653_flash *flash = to_adp1653_flash(subdev);

	if (platform_data == NULL)
		return -EINVAL;

	flash->platform_data = platform_data;

	return adp1653_init_controls(flash);
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
	.s_config = adp1653_set_config,
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
