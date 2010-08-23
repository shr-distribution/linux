/*
 * drivers/media/video/et8ek8.h
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
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
 */

#ifndef ET8EK8_H
#define ET8EK8_H

#include <linux/i2c.h>
#include <media/smiaregs.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>

struct regulator;

#define ET8EK8_NAME		"et8ek8"
#define ET8EK8_I2C_ADDR		(0x7C >> 1)

#define ET8EK8_PRIV_MEM_SIZE	128
#define ET8EK8_NCTRLS		3

struct et8ek8_platform_data {
	int (*g_priv)(struct v4l2_subdev *subdev, void *priv);
	int (*configure_interface)(struct v4l2_subdev *subdev,
				   struct smia_mode *mode);
	int (*set_xclk)(struct v4l2_subdev *subdev, int hz);
	int (*set_xshutdown)(struct v4l2_subdev *subdev, int set);
};

struct et8ek8_sensor;

/* Current values for V4L2 controls */
struct et8ek8_control {
	s32 minimum;
	s32 maximum;
	s32 step;
	s32 default_value;
	s32 value;
	int (*set)(struct et8ek8_sensor *sensor, s32 value);
};

#define to_et8ek8_sensor(sd)	container_of(sd, struct et8ek8_sensor, subdev)

struct et8ek8_sensor {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct et8ek8_platform_data *platform_data;
	struct regulator *vana;

	u16 version;

	struct et8ek8_control controls[ET8EK8_NCTRLS];

	struct smia_reglist *current_reglist;

	const struct firmware *fw;
	struct smia_meta_reglist *meta_reglist;
	u8 priv_mem[ET8EK8_PRIV_MEM_SIZE];

	int power : 1;
};

#endif /* ET8EK8_H */
