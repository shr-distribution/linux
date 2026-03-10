// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STMicroelectronics VX6953 5.1MP EDOF Camera Sensor
 *
 * Copyright (C) 2024 Linux community
 *
 * Based on legacy driver by Code Aurora Forum
 * Based on vgxy61.c driver structure by STMicroelectronics
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Sensor ID */
#define VX6953_MODEL_ID				0x6953
#define VX6953_REVISION_CUT2			0x10
#define VX6953_REVISION_CUT3			0x20

/* Register addresses */
#define VX6953_REG_MODEL_ID			CCI_REG16(0x0000)
#define VX6953_REG_REVISION			CCI_REG8(0x0002)
#define VX6953_REG_SOFTWARE_RESET		CCI_REG8(0x0103)
#define VX6953_REG_MODE_SELECT			CCI_REG8(0x0100)
#define VX6953_REG_GROUPED_PARAM_HOLD		CCI_REG8(0x0104)

/* Output format */
#define VX6953_REG_CSI_DATA_FORMAT_HI		CCI_REG8(0x0112)
#define VX6953_REG_CSI_DATA_FORMAT_LO		CCI_REG8(0x0113)
#define VX6953_REG_CSI_LANE_MODE		CCI_REG8(0x0111)

/* External clock */
#define VX6953_REG_EXTCLK_HI			CCI_REG8(0x0136)
#define VX6953_REG_EXTCLK_LO			CCI_REG8(0x0137)

/* PLL Configuration */
#define VX6953_REG_VT_PIX_CLK_DIV		CCI_REG8(0x0301)
#define VX6953_REG_PRE_PLL_CLK_DIV		CCI_REG8(0x0305)
#define VX6953_REG_PLL_MULTIPLIER		CCI_REG8(0x0307)
#define VX6953_REG_OP_PIX_CLK_DIV		CCI_REG8(0x0309)

/* Frame/Line timing */
#define VX6953_REG_FRAME_LENGTH_HI		CCI_REG8(0x0340)
#define VX6953_REG_FRAME_LENGTH_LO		CCI_REG8(0x0341)
#define VX6953_REG_LINE_LENGTH_HI		CCI_REG8(0x0342)
#define VX6953_REG_LINE_LENGTH_LO		CCI_REG8(0x0343)

/* Output size */
#define VX6953_REG_X_OUTPUT_SIZE_HI		CCI_REG8(0x034c)
#define VX6953_REG_X_OUTPUT_SIZE_LO		CCI_REG8(0x034d)
#define VX6953_REG_Y_OUTPUT_SIZE_HI		CCI_REG8(0x034e)
#define VX6953_REG_Y_OUTPUT_SIZE_LO		CCI_REG8(0x034f)

/* Exposure/Gain */
#define VX6953_REG_COARSE_INT_TIME_HI		CCI_REG8(0x0202)
#define VX6953_REG_COARSE_INT_TIME_LO		CCI_REG8(0x0203)
#define VX6953_REG_ANALOG_GAIN_HI		CCI_REG8(0x0204)
#define VX6953_REG_ANALOG_GAIN_LO		CCI_REG8(0x0205)

/* Digital gain */
#define VX6953_REG_DIGITAL_GAIN_GR_HI		CCI_REG8(0x020e)
#define VX6953_REG_DIGITAL_GAIN_GR_LO		CCI_REG8(0x020f)
#define VX6953_REG_DIGITAL_GAIN_R_HI		CCI_REG8(0x0210)
#define VX6953_REG_DIGITAL_GAIN_R_LO		CCI_REG8(0x0211)
#define VX6953_REG_DIGITAL_GAIN_B_HI		CCI_REG8(0x0212)
#define VX6953_REG_DIGITAL_GAIN_B_LO		CCI_REG8(0x0213)
#define VX6953_REG_DIGITAL_GAIN_GB_HI		CCI_REG8(0x0214)
#define VX6953_REG_DIGITAL_GAIN_GB_LO		CCI_REG8(0x0215)

/* Binning/Skipping */
#define VX6953_REG_BINNING_MODE			CCI_REG8(0x0900)
#define VX6953_REG_BINNING_TYPE			CCI_REG8(0x0901)
#define VX6953_REG_BINNING_WEIGHT		CCI_REG8(0x0902)
#define VX6953_REG_X_ODD_INC			CCI_REG8(0x0383)
#define VX6953_REG_Y_ODD_INC			CCI_REG8(0x0387)

/* Test pattern */
#define VX6953_REG_TEST_PATTERN			CCI_REG8(0x0601)

/* Sensor status */
#define VX6953_REG_STATUS			CCI_REG8(0x3368)

/* EDOF registers */
#define VX6953_REG_EDOF_MODE			CCI_REG8(0x0b80)
#define VX6953_REG_LENS_SHADING			CCI_REG8(0x0b00)

/* Sensor-specific registers */
#define VX6953_REG_0x3030			CCI_REG8(0x3030)
#define VX6953_REG_0x3001			CCI_REG8(0x3001)
#define VX6953_REG_0x3004			CCI_REG8(0x3004)
#define VX6953_REG_0x3007			CCI_REG8(0x3007)
#define VX6953_REG_0x3005			CCI_REG8(0x3005)
#define VX6953_REG_0x3010			CCI_REG8(0x3010)
#define VX6953_REG_0x3011			CCI_REG8(0x3011)
#define VX6953_REG_0x3016			CCI_REG8(0x3016)
#define VX6953_REG_0x301a			CCI_REG8(0x301a)
#define VX6953_REG_0x301d			CCI_REG8(0x301d)
#define VX6953_REG_0x3035			CCI_REG8(0x3035)
#define VX6953_REG_0x3036			CCI_REG8(0x3036)
#define VX6953_REG_0x3041			CCI_REG8(0x3041)
#define VX6953_REG_0x3042			CCI_REG8(0x3042)
#define VX6953_REG_0x3045			CCI_REG8(0x3045)
#define VX6953_REG_0x317e			CCI_REG8(0x317e)
#define VX6953_REG_0x317f			CCI_REG8(0x317f)
#define VX6953_REG_0x3400			CCI_REG8(0x3400)
#define VX6953_REG_0x3410			CCI_REG8(0x3410)
#define VX6953_REG_0x3098			CCI_REG8(0x3098)
#define VX6953_REG_0x309d			CCI_REG8(0x309d)

/* Defect correction */
#define VX6953_REG_DEFCOR_SINGLE_EN		CCI_REG8(0x0b06)
#define VX6953_REG_DEFCOR_SINGLE_WEIGHT		CCI_REG8(0x0b07)
#define VX6953_REG_DEFCOR_COUPLET_EN		CCI_REG8(0x0b08)
#define VX6953_REG_DEFCOR_COUPLET_WEIGHT	CCI_REG8(0x0b09)

/* EDOF configuration */
#define VX6953_REG_0x0b83			CCI_REG8(0x0b83)
#define VX6953_REG_0x0b84			CCI_REG8(0x0b84)
#define VX6953_REG_0x0b85			CCI_REG8(0x0b85)
#define VX6953_REG_0x0b88			CCI_REG8(0x0b88)
#define VX6953_REG_0x0b89			CCI_REG8(0x0b89)
#define VX6953_REG_0x0b8a			CCI_REG8(0x0b8a)

/* ROI registers */
#define VX6953_REG_0x1716			CCI_REG8(0x1716)
#define VX6953_REG_0x1717			CCI_REG8(0x1717)
#define VX6953_REG_0x1718			CCI_REG8(0x1718)
#define VX6953_REG_0x1719			CCI_REG8(0x1719)
#define VX6953_REG_0x3210			CCI_REG8(0x3210)

/* Mode select values */
#define VX6953_MODE_STANDBY			0x00
#define VX6953_MODE_STREAMING			0x01

/* Sensor status values */
#define VX6953_STATUS_STARTUP			0
#define VX6953_STATUS_SW_STANDBY		1
#define VX6953_STATUS_STREAMING_STARTUP		4
#define VX6953_STATUS_STREAMING			5
#define VX6953_STATUS_STREAMING_SHUTDOWN	6

/* Pixel array dimensions */
#define VX6953_PIXEL_ARRAY_WIDTH		2608
#define VX6953_PIXEL_ARRAY_HEIGHT		1960

/* External clock frequency */
#define VX6953_EXTCLK_FREQ			24000000

/* Exposure limits */
#define VX6953_EXPOSURE_MIN			1
#define VX6953_EXPOSURE_MAX			2922
#define VX6953_EXPOSURE_DEFAULT			976

/* Analog gain limits (0x00-0xF0) */
#define VX6953_ANALOG_GAIN_MIN			0x00
#define VX6953_ANALOG_GAIN_MAX			0xF0
#define VX6953_ANALOG_GAIN_DEFAULT		0xC0

/* Digital gain limits */
#define VX6953_DIGITAL_GAIN_MIN			0x0100
#define VX6953_DIGITAL_GAIN_MAX			0x0FFF
#define VX6953_DIGITAL_GAIN_DEFAULT		0x0100

#define VX6953_TIMEOUT_MS			500

static const char * const vx6953_supply_names[] = {
	"vana",  /* Analog power supply 2.8V */
	"vdig",  /* Digital power supply 1.8V */
	"vio",   /* I/O power supply 1.8V */
};

#define VX6953_NUM_SUPPLIES ARRAY_SIZE(vx6953_supply_names)

enum vx6953_mode_id {
	VX6953_MODE_PREVIEW,
	VX6953_MODE_CAPTURE,
	VX6953_MODE_MAX,
};

struct vx6953_mode {
	u32 width;
	u32 height;
	u32 frame_length;
	u32 line_length;
	u8 binning;
	u8 binning_type;
	u8 x_odd_inc;
	u8 y_odd_inc;
	u8 edof_mode;
};

static const struct vx6953_mode vx6953_modes[] = {
	/* Preview mode: 1304x980 (2x2 binning) */
	[VX6953_MODE_PREVIEW] = {
		.width = 1304,
		.height = 980,
		.frame_length = 1008,	/* 0x03f0 */
		.line_length = 2932,	/* 0x0b74 */
		.binning = 0x01,
		.binning_type = 0x22,
		.x_odd_inc = 0x03,
		.y_odd_inc = 0x03,
		.edof_mode = 0x02,	/* Estimation mode */
	},
	/* Capture mode: 2608x1960 (full resolution) */
	[VX6953_MODE_CAPTURE] = {
		.width = 2608,
		.height = 1960,
		.frame_length = 2000,	/* 0x07d0 */
		.line_length = 2956,	/* 0x0b8c */
		.binning = 0x00,
		.binning_type = 0x00,
		.x_odd_inc = 0x01,
		.y_odd_inc = 0x01,
		.edof_mode = 0x01,	/* Application mode */
	},
};

static const struct cci_reg_sequence vx6953_common_regs[] = {
	/* Output format: RAW10 */
	{ VX6953_REG_CSI_DATA_FORMAT_HI, 0x0a },
	{ VX6953_REG_CSI_DATA_FORMAT_LO, 0x0a },
	/* CSI lane mode */
	{ VX6953_REG_CSI_LANE_MODE, 0x02 },
	/* PLL configuration for 24MHz input */
	{ VX6953_REG_VT_PIX_CLK_DIV, 0x09 },
	{ VX6953_REG_PRE_PLL_CLK_DIV, 0x04 },
	{ VX6953_REG_PLL_MULTIPLIER, 0x85 },	/* 133 */
	{ VX6953_REG_OP_PIX_CLK_DIV, 0x0a },
	/* External clock: 24MHz (0x1800) */
	{ VX6953_REG_EXTCLK_HI, 0x18 },
	{ VX6953_REG_EXTCLK_LO, 0x00 },
	/* Sensor-specific settings */
	{ VX6953_REG_0x3030, 0x08 },
	{ VX6953_REG_LENS_SHADING, 0x01 },	/* Enable lens shading */
	{ VX6953_REG_0x3001, 0x30 },
	{ VX6953_REG_0x3004, 0x33 },
	{ VX6953_REG_0x3007, 0x09 },
	{ VX6953_REG_0x3016, 0x1f },
	{ VX6953_REG_0x301d, 0x03 },
	{ VX6953_REG_0x317e, 0x11 },
	{ VX6953_REG_0x317f, 0x09 },
	{ VX6953_REG_0x3400, 0x38 },
	/* Defect correction */
	{ VX6953_REG_DEFCOR_SINGLE_EN, 0x00 },
	{ VX6953_REG_DEFCOR_SINGLE_WEIGHT, 0x80 },
	{ VX6953_REG_DEFCOR_COUPLET_EN, 0x01 },
	{ VX6953_REG_DEFCOR_COUPLET_WEIGHT, 0x4f },
	/* EDOF settings */
	{ VX6953_REG_0x0b83, 0x20 },
	{ VX6953_REG_0x0b84, 0x90 },
	{ VX6953_REG_0x0b85, 0x20 },
	{ VX6953_REG_0x0b88, 0x80 },
	{ VX6953_REG_0x0b89, 0x00 },
	{ VX6953_REG_0x0b8a, 0x00 },
};

static const struct cci_reg_sequence vx6953_preview_regs[] = {
	{ VX6953_REG_FRAME_LENGTH_HI, 0x03 },
	{ VX6953_REG_FRAME_LENGTH_LO, 0xf0 },
	{ VX6953_REG_LINE_LENGTH_HI, 0x0b },
	{ VX6953_REG_LINE_LENGTH_LO, 0x74 },
	{ VX6953_REG_X_OUTPUT_SIZE_HI, 0x05 },
	{ VX6953_REG_X_OUTPUT_SIZE_LO, 0x18 },
	{ VX6953_REG_Y_OUTPUT_SIZE_HI, 0x03 },
	{ VX6953_REG_Y_OUTPUT_SIZE_LO, 0xd4 },
	{ VX6953_REG_0x3005, 0x03 },
	{ VX6953_REG_0x3010, 0x00 },
	{ VX6953_REG_0x3011, 0x01 },
	{ VX6953_REG_0x301a, 0x6a },
	{ VX6953_REG_0x3035, 0x03 },
	{ VX6953_REG_0x3036, 0x2c },
	{ VX6953_REG_0x3041, 0x00 },
	{ VX6953_REG_0x3042, 0x24 },
	{ VX6953_REG_0x3045, 0x81 },
	{ VX6953_REG_EDOF_MODE, 0x02 },
	{ VX6953_REG_BINNING_MODE, 0x01 },
	{ VX6953_REG_BINNING_TYPE, 0x22 },
	{ VX6953_REG_BINNING_WEIGHT, 0x04 },
	{ VX6953_REG_X_ODD_INC, 0x03 },
	{ VX6953_REG_Y_ODD_INC, 0x03 },
	{ VX6953_REG_0x1716, 0x02 },
	{ VX6953_REG_0x1717, 0x04 },
	{ VX6953_REG_0x1718, 0x08 },
	{ VX6953_REG_0x1719, 0x2c },
	{ VX6953_REG_0x3210, 0x01 },
	{ VX6953_REG_0x3410, 0x01 },
	{ VX6953_REG_0x3098, 0x01 },
	{ VX6953_REG_0x309d, 0x05 },
};

static const struct cci_reg_sequence vx6953_capture_regs[] = {
	{ VX6953_REG_FRAME_LENGTH_HI, 0x07 },
	{ VX6953_REG_FRAME_LENGTH_LO, 0xd0 },
	{ VX6953_REG_LINE_LENGTH_HI, 0x0b },
	{ VX6953_REG_LINE_LENGTH_LO, 0x8c },
	{ VX6953_REG_X_OUTPUT_SIZE_HI, 0x0a },
	{ VX6953_REG_X_OUTPUT_SIZE_LO, 0x30 },
	{ VX6953_REG_Y_OUTPUT_SIZE_HI, 0x07 },
	{ VX6953_REG_Y_OUTPUT_SIZE_LO, 0xa8 },
	{ VX6953_REG_0x3005, 0x01 },
	{ VX6953_REG_0x3010, 0x00 },
	{ VX6953_REG_0x3011, 0x00 },
	{ VX6953_REG_0x301a, 0x55 },
	{ VX6953_REG_0x3035, 0x01 },
	{ VX6953_REG_0x3036, 0x23 },
	{ VX6953_REG_0x3041, 0x00 },
	{ VX6953_REG_0x3042, 0x24 },
	{ VX6953_REG_0x3045, 0xb7 },
	{ VX6953_REG_EDOF_MODE, 0x01 },
	{ VX6953_REG_BINNING_MODE, 0x00 },
	{ VX6953_REG_BINNING_TYPE, 0x00 },
	{ VX6953_REG_BINNING_WEIGHT, 0x00 },
	{ VX6953_REG_X_ODD_INC, 0x01 },
	{ VX6953_REG_Y_ODD_INC, 0x01 },
	{ VX6953_REG_0x1716, 0x02 },
	{ VX6953_REG_0x1717, 0x0d },
	{ VX6953_REG_0x1718, 0x07 },
	{ VX6953_REG_0x1719, 0x7d },
	{ VX6953_REG_0x3210, 0x01 },
	{ VX6953_REG_0x3410, 0x01 },
	{ VX6953_REG_0x3098, 0x01 },
	{ VX6953_REG_0x309d, 0x05 },
};

struct vx6953 {
	struct i2c_client *client;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[VX6953_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analog_gain;
	struct v4l2_ctrl *digital_gain;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *test_pattern;

	/* Lock to protect streaming state */
	struct mutex lock;
	bool streaming;

	const struct vx6953_mode *cur_mode;
	u8 revision;
};

static inline struct vx6953 *to_vx6953(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vx6953, sd);
}

static int vx6953_wait_state(struct vx6953 *sensor, u8 target_state)
{
	u64 val;
	int ret;
	unsigned int tries = VX6953_TIMEOUT_MS / 10;

	while (tries--) {
		ret = cci_read(sensor->regmap, VX6953_REG_STATUS, &val, NULL);
		if (ret)
			return ret;
		if (val == target_state)
			return 0;
		usleep_range(10000, 11000);
	}

	dev_err(&sensor->client->dev, "timeout waiting for state %u, got %llu\n",
		target_state, val);
	return -ETIMEDOUT;
}

static int vx6953_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vx6953 *sensor = container_of(ctrl->handler, struct vx6953, ctrls);
	int ret = 0;

	if (!sensor->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(sensor->regmap, VX6953_REG_COARSE_INT_TIME_HI,
				(ctrl->val >> 8) & 0xff, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_COARSE_INT_TIME_LO,
				ctrl->val & 0xff, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(sensor->regmap, VX6953_REG_ANALOG_GAIN_LO,
				ctrl->val & 0xff, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_GR_HI,
				(ctrl->val >> 8) & 0x0f, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_GR_LO,
				ctrl->val & 0xff, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_R_HI,
				(ctrl->val >> 8) & 0x0f, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_R_LO,
				ctrl->val & 0xff, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_B_HI,
				(ctrl->val >> 8) & 0x0f, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_B_LO,
				ctrl->val & 0xff, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_GB_HI,
				(ctrl->val >> 8) & 0x0f, NULL);
		ret = cci_write(sensor->regmap, VX6953_REG_DIGITAL_GAIN_GB_LO,
				ctrl->val & 0xff, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(sensor->regmap, VX6953_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops vx6953_ctrl_ops = {
	.s_ctrl = vx6953_set_ctrl,
};

static const char * const vx6953_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"100% color bars",
	"Fade to gray color bars",
};

static int vx6953_init_controls(struct vx6953 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	const struct vx6953_mode *mode = sensor->cur_mode;
	s64 hblank, vblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 8);

	/* Exposure */
	sensor->exposure = v4l2_ctrl_new_std(hdl, &vx6953_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     VX6953_EXPOSURE_MIN,
					     VX6953_EXPOSURE_MAX,
					     1, VX6953_EXPOSURE_DEFAULT);

	/* Analog gain */
	sensor->analog_gain = v4l2_ctrl_new_std(hdl, &vx6953_ctrl_ops,
						V4L2_CID_ANALOGUE_GAIN,
						VX6953_ANALOG_GAIN_MIN,
						VX6953_ANALOG_GAIN_MAX,
						1, VX6953_ANALOG_GAIN_DEFAULT);

	/* Digital gain */
	sensor->digital_gain = v4l2_ctrl_new_std(hdl, &vx6953_ctrl_ops,
						 V4L2_CID_DIGITAL_GAIN,
						 VX6953_DIGITAL_GAIN_MIN,
						 VX6953_DIGITAL_GAIN_MAX,
						 1, VX6953_DIGITAL_GAIN_DEFAULT);

	/* Blanking - read only for now */
	hblank = mode->line_length - mode->width;
	sensor->hblank = v4l2_ctrl_new_std(hdl, &vx6953_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->frame_length - mode->height;
	sensor->vblank = v4l2_ctrl_new_std(hdl, &vx6953_ctrl_ops,
					   V4L2_CID_VBLANK, vblank, vblank,
					   1, vblank);
	if (sensor->vblank)
		sensor->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Test pattern */
	sensor->test_pattern = v4l2_ctrl_new_std_menu_items(hdl,
					&vx6953_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(vx6953_test_pattern_menu) - 1,
					0, 0, vx6953_test_pattern_menu);

	/* Pixel rate - informational */
	v4l2_ctrl_new_std(hdl, &vx6953_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  88670000, 88670000, 1, 88670000);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;
}

static int vx6953_power_on(struct vx6953 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(VX6953_NUM_SUPPLIES, sensor->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		regulator_bulk_disable(VX6953_NUM_SUPPLIES, sensor->supplies);
		return ret;
	}

	/* Power down is active low on VX6953 */
	if (sensor->powerdown_gpio)
		gpiod_set_value_cansleep(sensor->powerdown_gpio, 0);

	/* Wait for sensor to settle after power on */
	usleep_range(10000, 15000);

	return 0;
}

static void vx6953_power_off(struct vx6953 *sensor)
{
	if (sensor->powerdown_gpio)
		gpiod_set_value_cansleep(sensor->powerdown_gpio, 1);

	clk_disable_unprepare(sensor->xclk);
	regulator_bulk_disable(VX6953_NUM_SUPPLIES, sensor->supplies);
}

static int vx6953_detect(struct vx6953 *sensor)
{
	u64 model_id, revision;
	int ret;

	ret = cci_read(sensor->regmap, VX6953_REG_MODEL_ID, &model_id, NULL);
	if (ret)
		return ret;

	if (model_id != VX6953_MODEL_ID) {
		dev_err(&sensor->client->dev,
			"unsupported model id 0x%04llx\n", model_id);
		return -ENODEV;
	}

	ret = cci_read(sensor->regmap, VX6953_REG_REVISION, &revision, NULL);
	if (ret)
		return ret;

	sensor->revision = revision;

	dev_info(&sensor->client->dev,
		 "VX6953 detected, revision 0x%02x (Cut%s)\n",
		 sensor->revision,
		 sensor->revision == VX6953_REVISION_CUT2 ? "2.0" :
		 sensor->revision == VX6953_REVISION_CUT3 ? "3.0" : "?");

	return 0;
}

static int vx6953_init_sensor(struct vx6953 *sensor)
{
	int ret;

	/* Software reset */
	ret = cci_write(sensor->regmap, VX6953_REG_SOFTWARE_RESET, 0x01, NULL);
	if (ret)
		return ret;

	usleep_range(10000, 15000);

	/* Put sensor in standby */
	ret = cci_write(sensor->regmap, VX6953_REG_MODE_SELECT,
			VX6953_MODE_STANDBY, NULL);
	if (ret)
		return ret;

	ret = vx6953_wait_state(sensor, VX6953_STATUS_SW_STANDBY);
	if (ret)
		return ret;

	/* Write common configuration */
	ret = cci_multi_reg_write(sensor->regmap, vx6953_common_regs,
				  ARRAY_SIZE(vx6953_common_regs), NULL);
	if (ret)
		return ret;

	return 0;
}

static int vx6953_set_mode(struct vx6953 *sensor, const struct vx6953_mode *mode)
{
	const struct cci_reg_sequence *regs;
	size_t num_regs;
	int ret;

	/* Select mode registers */
	if (mode == &vx6953_modes[VX6953_MODE_PREVIEW]) {
		regs = vx6953_preview_regs;
		num_regs = ARRAY_SIZE(vx6953_preview_regs);
	} else {
		regs = vx6953_capture_regs;
		num_regs = ARRAY_SIZE(vx6953_capture_regs);
	}

	ret = cci_multi_reg_write(sensor->regmap, regs, num_regs, NULL);
	if (ret)
		return ret;

	sensor->cur_mode = mode;
	return 0;
}

static int vx6953_start_streaming(struct vx6953 *sensor)
{
	int ret;

	ret = vx6953_init_sensor(sensor);
	if (ret)
		return ret;

	ret = vx6953_set_mode(sensor, sensor->cur_mode);
	if (ret)
		return ret;

	/* Apply controls */
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		return ret;

	/* Start streaming */
	ret = cci_write(sensor->regmap, VX6953_REG_MODE_SELECT,
			VX6953_MODE_STREAMING, NULL);
	if (ret)
		return ret;

	ret = vx6953_wait_state(sensor, VX6953_STATUS_STREAMING);
	if (ret)
		return ret;

	return 0;
}

static int vx6953_stop_streaming(struct vx6953 *sensor)
{
	int ret;

	ret = cci_write(sensor->regmap, VX6953_REG_MODE_SELECT,
			VX6953_MODE_STANDBY, NULL);
	if (ret)
		return ret;

	return vx6953_wait_state(sensor, VX6953_STATUS_SW_STANDBY);
}

static int vx6953_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vx6953 *sensor = to_vx6953(sd);
	int ret;

	mutex_lock(&sensor->lock);

	if (enable && !sensor->streaming) {
		ret = pm_runtime_resume_and_get(&sensor->client->dev);
		if (ret < 0)
			goto out;

		ret = vx6953_start_streaming(sensor);
		if (ret) {
			pm_runtime_put(&sensor->client->dev);
			goto out;
		}
		sensor->streaming = true;
	} else if (!enable && sensor->streaming) {
		vx6953_stop_streaming(sensor);
		pm_runtime_put(&sensor->client->dev);
		sensor->streaming = false;
	}

	ret = 0;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int vx6953_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int vx6953_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= VX6953_MODE_MAX)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = vx6953_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = vx6953_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void vx6953_update_pad_format(const struct vx6953_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int vx6953_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct vx6953 *sensor = to_vx6953(sd);
	struct v4l2_mbus_framefmt *mbus_fmt;

	mutex_lock(&sensor->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mbus_fmt = v4l2_subdev_state_get_format(state, 0);
		fmt->format = *mbus_fmt;
	} else {
		vx6953_update_pad_format(sensor->cur_mode, &fmt->format);
	}
	mutex_unlock(&sensor->lock);

	return 0;
}

static int vx6953_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct vx6953 *sensor = to_vx6953(sd);
	const struct vx6953_mode *mode;
	struct v4l2_mbus_framefmt *mbus_fmt;
	int i;

	mutex_lock(&sensor->lock);

	/* Find best matching mode */
	mode = &vx6953_modes[VX6953_MODE_CAPTURE];
	for (i = 0; i < VX6953_MODE_MAX; i++) {
		if (vx6953_modes[i].width >= fmt->format.width &&
		    vx6953_modes[i].height >= fmt->format.height) {
			mode = &vx6953_modes[i];
			break;
		}
	}

	vx6953_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mbus_fmt = v4l2_subdev_state_get_format(state, 0);
		*mbus_fmt = fmt->format;
	} else {
		sensor->cur_mode = mode;
	}

	mutex_unlock(&sensor->lock);

	return 0;
}

static int vx6953_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *mbus_fmt;

	mbus_fmt = v4l2_subdev_state_get_format(state, 0);
	vx6953_update_pad_format(&vx6953_modes[VX6953_MODE_PREVIEW], mbus_fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops vx6953_video_ops = {
	.s_stream = vx6953_s_stream,
};

static const struct v4l2_subdev_pad_ops vx6953_pad_ops = {
	.enum_mbus_code = vx6953_enum_mbus_code,
	.enum_frame_size = vx6953_enum_frame_size,
	.get_fmt = vx6953_get_fmt,
	.set_fmt = vx6953_set_fmt,
};

static const struct v4l2_subdev_ops vx6953_subdev_ops = {
	.video = &vx6953_video_ops,
	.pad = &vx6953_pad_ops,
};

static const struct v4l2_subdev_internal_ops vx6953_internal_ops = {
	.init_state = vx6953_init_state,
};

static int vx6953_get_regulators(struct vx6953 *sensor)
{
	int i;

	for (i = 0; i < VX6953_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = vx6953_supply_names[i];

	return devm_regulator_bulk_get(&sensor->client->dev,
				       VX6953_NUM_SUPPLIES,
				       sensor->supplies);
}

static int vx6953_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct vx6953 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->client = client;
	mutex_init(&sensor->lock);

	/* Get regmap */
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	/* Get clock */
	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(dev, PTR_ERR(sensor->xclk),
				     "failed to get clock\n");

	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	if (sensor->xclk_freq != VX6953_EXTCLK_FREQ)
		dev_warn(dev, "clock frequency %u differs from expected %u\n",
			 sensor->xclk_freq, VX6953_EXTCLK_FREQ);

	/* Get regulators */
	ret = vx6953_get_regulators(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/* Get power down GPIO (active high in DT, but active low on sensor) */
	sensor->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->powerdown_gpio))
		return PTR_ERR(sensor->powerdown_gpio);

	/* Default mode */
	sensor->cur_mode = &vx6953_modes[VX6953_MODE_PREVIEW];

	/* Power on and detect sensor */
	ret = vx6953_power_on(sensor);
	if (ret)
		return ret;

	ret = vx6953_detect(sensor);
	if (ret)
		goto err_power_off;

	/* Initialize controls */
	ret = vx6953_init_controls(sensor);
	if (ret)
		goto err_power_off;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&sensor->sd, client, &vx6953_subdev_ops);
	sensor->sd.internal_ops = &vx6953_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrl_free;

	/* Power off for now, will be powered on when streaming starts */
	vx6953_power_off(sensor);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_entity_cleanup;

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);

	return 0;

err_entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
err_ctrl_free:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_power_off:
	vx6953_power_off(sensor);
	return ret;
}

static void vx6953_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vx6953 *sensor = to_vx6953(sd);

	pm_runtime_disable(&client->dev);
	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	mutex_destroy(&sensor->lock);
}

static int vx6953_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct vx6953 *sensor = to_vx6953(sd);

	vx6953_power_off(sensor);
	return 0;
}

static int vx6953_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct vx6953 *sensor = to_vx6953(sd);

	return vx6953_power_on(sensor);
}

static const struct dev_pm_ops vx6953_pm_ops = {
	RUNTIME_PM_OPS(vx6953_runtime_suspend, vx6953_runtime_resume, NULL)
};

static const struct of_device_id vx6953_of_match[] = {
	{ .compatible = "st,vx6953" },
	{ }
};
MODULE_DEVICE_TABLE(of, vx6953_of_match);

static const struct i2c_device_id vx6953_id[] = {
	{ "vx6953" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vx6953_id);

static struct i2c_driver vx6953_driver = {
	.driver = {
		.name = "vx6953",
		.pm = pm_ptr(&vx6953_pm_ops),
		.of_match_table = vx6953_of_match,
	},
	.probe = vx6953_probe,
	.remove = vx6953_remove,
	.id_table = vx6953_id,
};

module_i2c_driver(vx6953_driver);

MODULE_DESCRIPTION("STMicroelectronics VX6953 5.1MP EDOF Camera Sensor Driver");
MODULE_AUTHOR("Linux community");
MODULE_LICENSE("GPL");
