// SPDX-License-Identifier: GPL-2.0-only
/*
 * mt9m113.c Aptina MT9M113 sensor driver
 *
 * Copyright (c) 2024 Linux Enthusiasts
 *
 * MT9M113 is a 1.3MP SOC sensor with dual context support:
 *   Context A: 640x480 preview mode (binned)
 *   Context B: 1280x1024 capture mode (full resolution)
 *
 * Ported from webOS kernel mt9m113.c/mt9m113_reg.c.
 * Uses MCU indirect access (0x098C/0x0990) for configuration.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

/* Module parameters for MT9M113 tuning */
static int mt9m113_pre_mipi_delay_ms = 10;
module_param(mt9m113_pre_mipi_delay_ms, int, 0644);
MODULE_PARM_DESC(mt9m113_pre_mipi_delay_ms,
		 "Delay (ms) before enabling MIPI output (default 10)");

static int mt9m113_cont_mipi_clk = 1;
module_param(mt9m113_cont_mipi_clk, int, 0644);
MODULE_PARM_DESC(mt9m113_cont_mipi_clk,
		 "Use continuous MIPI clock (0=LP, 1=continuous default)");

static int mt9m113_skip_short_pkt = 1;
module_param(mt9m113_skip_short_pkt, int, 0644);
MODULE_PARM_DESC(mt9m113_skip_short_pkt,
		 "Skip CUSTOM_SHORT_PKT write to match webOS (1=skip default)");

static int mt9m113_fake_yuv;
module_param(mt9m113_fake_yuv, int, 0644);
MODULE_PARM_DESC(mt9m113_fake_yuv,
		 "Use YUV MIPI data type for RAW output (debug only)");

/* MT9M113 Context V4L2 Control */
#define V4L2_CID_MT9M113_CONTEXT	(V4L2_CID_USER_BASE + 0x1001)
#define MT9M113_CONTEXT_A		0	/* 640x480 preview */
#define MT9M113_CONTEXT_B		1	/* 1280x1024 capture */

/* MT9M113 chip ID */
#define MT9M113_CHIP_ID				CCI_REG16(0x0000)
#define MT9M113_CHIP_ID_VALUE			0x2480

/* Sysctl registers */
#define MT9M113_COMMAND_REGISTER		CCI_REG16(0x0080)
#define MT9M113_PLL_DIVIDERS			CCI_REG16(0x0010)
#define MT9M113_PLL_P_DIVIDERS			CCI_REG16(0x0012)
#define MT9M113_PLL_CONTROL			CCI_REG16(0x0014)
#define MT9M113_CLOCKS_CONTROL			CCI_REG16(0x0016)
#define MT9M113_STANDBY_CONTROL			CCI_REG16(0x0018)
#define MT9M113_RESET_AND_MISC_CONTROL		CCI_REG16(0x001a)
#define MT9M113_RESET_SOC			BIT(0)
#define MT9M113_MCU_BOOT_MODE			CCI_REG16(0x001c)
#define MT9M113_PAD_SLEW			CCI_REG16(0x001e)

/* XDMA / MCU indirect access registers */
#define MT9M113_ACCESS_CTL_STAT			CCI_REG16(0x0982)
#define MT9M113_MCU_ADDRESS			CCI_REG16(0x098c)
#define MT9M113_MCU_DATA			CCI_REG16(0x0990)

/* MCU variable addresses */
#define MT9M113_SEQ_CMD				0xa103
#define MT9M113_SEQ_CMD_RUN			0x0001
#define MT9M113_SEQ_CMD_CAPTURE			0x0002
#define MT9M113_SEQ_CMD_REFRESH			0x0005
#define MT9M113_SEQ_CMD_REFRESH_MODE		0x0006
#define MT9M113_SEQ_STATE			0xa104
#define MT9M113_SEQ_CAP_MODE			0xa115
#define MT9M113_SEQ_CAP_NUM_FRAMES		0xa116	/* 0 = infinite/continuous */

/* Mode Output Format registers (Driver ID 7) */
#define MT9M113_MODE_OUTPUT_FORMAT_A		0x2755
#define MT9M113_MODE_OUTPUT_FORMAT_B		0x2757
#define MT9M113_MODE_OUTPUT_FORMAT_PROCESSED_BAYER	BIT(8)

/* Special effects registers */
#define MT9M113_MODE_SPEC_EFFECTS_A		CCI_REG16(0x2759)
#define MT9M113_MODE_SPEC_EFFECTS_B		CCI_REG16(0x275B)
#define MT9M113_SPEC_EFFECTS_DEFAULT		0x6440
#define MT9M113_SPEC_EFFECTS_MASK		0x0007
#define MT9M113_SPEC_EFFECTS_NONE		0x0000
#define MT9M113_SPEC_EFFECTS_MONOCHROME		0x0001
#define MT9M113_SPEC_EFFECTS_SEPIA		0x0002
#define MT9M113_SPEC_EFFECTS_NEGATIVE		0x0003
#define MT9M113_SPEC_EFFECTS_SOLARIZE		0x0004

/* Auto Exposure MCU variables (for preview vs snapshot optimization) */
#define MT9M113_AE_MAX_INDEX			0xa20c
#define MT9M113_AE_MAX_VIRTGAIN			0xa20e
#define MT9M113_AE_MAX_DGAIN_AE1		0xa21a
#define MT9M113_AE_JUMP_DIVISOR			0xa21c
#define MT9M113_AE_SKIP_FRAMES			0xa21e

/* Sensor core registers */
#define MT9M113_RESET_REGISTER			CCI_REG16(0x301a)
#define MT9M113_RESET_REG_STREAMING		0x120C
#define MT9M113_RESET_REG_SNAPSHOT		0x12CE
#define MT9M113_OFIFO_CONTROL_STATUS		CCI_REG16(0x321c)

/* OUTPUT_CONTROL register (0x3400) - MIPI control */
#define MT9M113_OUTPUT_CONTROL			CCI_REG16(0x3400)
#define MT9M113_OUTPUT_CONTROL_RO_MASK		0x0008
#define MT9M113_OUTPUT_CONTROL_MIPI_ENABLE	0x7A08	/* YUV422 dt=0x1E */
#define MT9M113_OUTPUT_CONTROL_MIPI_RAW8	0xAA08	/* RAW8 dt=0x2A */
#define MT9M113_OUTPUT_CONTROL_MIPI_RAW10	0xAC08	/* RAW10 dt=0x2B */

/* CUSTOM_SHORT_PKT register */
#define MT9M113_CUSTOM_SHORT_PKT		CCI_REG16(0x3404)
#define MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN	0x0080

/* Camera Control registers */
#define MT9M113_CAM_OUTPUT_WIDTH		CCI_REG16(0xc868)
#define MT9M113_CAM_OUTPUT_HEIGHT		CCI_REG16(0xc86a)
#define MT9M113_CAM_OUTPUT_FORMAT		CCI_REG16(0xc86c)
#define MT9M113_CAM_OUTPUT_FORMAT_FORMAT_YUV	(0 << 8)
#define MT9M113_CAM_OUTPUT_FORMAT_FORMAT_RGB	(1 << 8)
#define MT9M113_CAM_OUTPUT_FORMAT_FORMAT_BAYER	(2 << 8)
#define MT9M113_CAM_OUTPUT_FORMAT_FORMAT_MASK	(3 << 8)
#define MT9M113_CAM_OUTPUT_FORMAT_BAYER_FORMAT_RAWR10		(0 << 10)
#define MT9M113_CAM_OUTPUT_FORMAT_BAYER_FORMAT_PROCESSED8	(3 << 10)
#define MT9M113_CAM_OUTPUT_FORMAT_BAYER_FORMAT_MASK		(3 << 10)

/* Monitor registers */
#define MT9M113_MON_MAJOR_VERSION		CCI_REG16(0x8000)
#define MT9M113_MON_MINOR_VERSION		CCI_REG16(0x8002)
#define MT9M113_MON_RELEASE_VERSION		CCI_REG16(0x8004)
#define MT9M113_CUSTOMER_REV			CCI_REG16(0x31fe)

/* Pixel array dimensions */
#define MT9M113_PIXEL_ARRAY_WIDTH		1296U
#define MT9M113_PIXEL_ARRAY_HEIGHT		1040U

/* Default blanking */
#define MT9M113_DEF_FRAME_RATE			30

/* -----------------------------------------------------------------------------
 * Data Structures
 */

enum mt9m113_format_flag {
	MT9M113_FMT_FLAG_PARALLEL = BIT(0),
	MT9M113_FMT_FLAG_CSI2 = BIT(1),
};

struct mt9m113_format_info {
	u32 code;
	u32 output_format;
	u32 flags;
};

struct mt9m113 {
	struct i2c_client *client;
	struct regmap *regmap;

	struct clk *clk;
	struct gpio_desc *reset;
	struct gpio_desc *powerdown;
	struct regulator_bulk_data supplies[3];
	struct v4l2_fwnode_endpoint bus_cfg;

	unsigned int pixrate;
	bool streaming;

	/* Pixel Array sub-device */
	struct {
		struct v4l2_subdev sd;
		struct media_pad pad;
		struct v4l2_ctrl_handler hdl;
	} pa;

	/* Image Flow Processor sub-device */
	struct {
		struct v4l2_subdev sd;
		struct media_pad pads[2];

		struct v4l2_ctrl_handler hdl;
		struct v4l2_ctrl *context;
	} ifp;
};

/* -----------------------------------------------------------------------------
 * Formats
 */

static const struct mt9m113_format_info mt9m113_format_infos[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.flags = MT9M113_FMT_FLAG_CSI2,
		.output_format = MT9M113_CAM_OUTPUT_FORMAT_FORMAT_YUV,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.flags = MT9M113_FMT_FLAG_CSI2,
		.output_format = MT9M113_CAM_OUTPUT_FORMAT_FORMAT_YUV,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.output_format = MT9M113_CAM_OUTPUT_FORMAT_BAYER_FORMAT_PROCESSED8
			       | MT9M113_CAM_OUTPUT_FORMAT_FORMAT_BAYER,
		.flags = MT9M113_FMT_FLAG_CSI2,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.output_format = MT9M113_CAM_OUTPUT_FORMAT_BAYER_FORMAT_RAWR10
			| MT9M113_CAM_OUTPUT_FORMAT_FORMAT_BAYER,
		.flags = MT9M113_FMT_FLAG_CSI2,
	}
};

static const struct mt9m113_format_info *
mt9m113_format_info(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mt9m113_format_infos); i++) {
		if (mt9m113_format_infos[i].code == code)
			return &mt9m113_format_infos[i];
	}

	return &mt9m113_format_infos[0];
}

/* -----------------------------------------------------------------------------
 * MCU Variable Access Helpers
 */

static int mt9m113_write_mcu_var(struct mt9m113 *sensor, u16 addr, u16 value)
{
	int ret = 0;

	cci_write(sensor->regmap, MT9M113_MCU_ADDRESS, addr, &ret);
	cci_write(sensor->regmap, MT9M113_MCU_DATA, value, &ret);
	return ret;
}

static int mt9m113_read_mcu_var(struct mt9m113 *sensor, u16 addr, u64 *value)
{
	int ret;

	ret = cci_write(sensor->regmap, MT9M113_MCU_ADDRESS, addr, NULL);
	if (ret)
		return ret;
	return cci_read(sensor->regmap, MT9M113_MCU_DATA, value, NULL);
}

static int mt9m113_poll_mcu_var(struct mt9m113 *sensor, u16 addr,
				u16 expected, unsigned int timeout_ms)
{
	unsigned int i;
	u64 value;
	int ret;

	for (i = 0; i < timeout_ms / 10; i++) {
		ret = mt9m113_read_mcu_var(sensor, addr, &value);
		if (ret < 0)
			return ret;
		if (value == expected)
			return 0;
		msleep(10);
	}

	dev_err(&sensor->client->dev,
		"MCU var 0x%04x timeout (got 0x%llx, expected 0x%04x)\n",
		addr, value, expected);
	return -ETIMEDOUT;
}

/* -----------------------------------------------------------------------------
 * Initialization Table (from webOS mt9m113_reg.c)
 */

struct mt9m113_reg_entry {
	u16 reg;
	u16 value;
	u16 delay_ms;
};

static const struct mt9m113_reg_entry mt9m113_init_table[] = {
	/* OFIFO control */
	{ 0x321C, 0x0003, 0 },

	/* Context A output (640x480 preview) */
	{ 0x098C, 0x2703, 0 },
	{ 0x0990, 0x0280, 0 },
	{ 0x098C, 0x2705, 0 },
	{ 0x0990, 0x01E0, 0 },

	/* Context B output (1280x1024 capture) */
	{ 0x098C, 0x2707, 0 },
	{ 0x0990, 0x0500, 0 },
	{ 0x098C, 0x2709, 0 },
	{ 0x0990, 0x0400, 0 },

	/*
	 * Context A sensor configuration (datasheet defaults from Table 19)
	 * Preview mode: 640x480 output with 2x skip/binning
	 */
	{ 0x098C, 0x270D, 0 },		/* MODE_SENSOR_ROW_START_A */
	{ 0x0990, 0x0000, 0 },		/* 0 */
	{ 0x098C, 0x270F, 0 },		/* MODE_SENSOR_COL_START_A */
	{ 0x0990, 0x0000, 0 },		/* 0 */
	{ 0x098C, 0x2711, 0 },		/* MODE_SENSOR_ROW_END_A */
	{ 0x0990, 0x04BD, 0 },		/* 1213 - datasheet default */
	{ 0x098C, 0x2713, 0 },		/* MODE_SENSOR_COL_END_A */
	{ 0x0990, 0x064D, 0 },		/* 1613 - datasheet default */
	{ 0x098C, 0x2715, 0 },		/* MODE_SENSOR_ROW_SPEED_A */
	{ 0x0990, 0x2112, 0 },		/* 8466 - datasheet default */
	{ 0x098C, 0x2717, 0 },		/* MODE_SENSOR_READ_MODE_A */
	{ 0x0990, 0x046C, 0 },		/* 1132 - 2x skip, x-mirror */
	{ 0x098C, 0x2719, 0 },		/* MODE_SENSOR_FINE_CORRECTION_A */
	{ 0x0990, 0x007B, 0 },		/* 123 - datasheet default */
	{ 0x098C, 0x271B, 0 },		/* MODE_SENSOR_FINE_IT_MIN_A */
	{ 0x0990, 0x0408, 0 },		/* 1032 - datasheet default */
	{ 0x098C, 0x271D, 0 },		/* MODE_SENSOR_FINE_IT_MAX_MARGIN_A */
	{ 0x0990, 0x00AB, 0 },		/* 171 - datasheet default */
	{ 0x098C, 0x271F, 0 },		/* MODE_SENSOR_FRAME_LENGTH_A */
	{ 0x0990, 0x0293, 0 },		/* 659 - datasheet default */
	{ 0x098C, 0x2721, 0 },		/* MODE_SENSOR_LINE_LENGTH_PCK_A */
	{ 0x0990, 0x07D0, 0 },		/* 2000 - datasheet default */

	/*
	 * Context B sensor configuration (datasheet defaults from Table 19)
	 * Capture mode: 1280x1024 full resolution
	 */
	{ 0x098C, 0x2723, 0 },		/* MODE_SENSOR_ROW_START_B */
	{ 0x0990, 0x0004, 0 },		/* 4 - datasheet default */
	{ 0x098C, 0x2725, 0 },		/* MODE_SENSOR_COL_START_B */
	{ 0x0990, 0x0004, 0 },		/* 4 - datasheet default */
	{ 0x098C, 0x2727, 0 },		/* MODE_SENSOR_ROW_END_B */
	{ 0x0990, 0x04BB, 0 },		/* 1211 - datasheet default */
	{ 0x098C, 0x2729, 0 },		/* MODE_SENSOR_COL_END_B */
	{ 0x0990, 0x064B, 0 },		/* 1611 - datasheet default */
	{ 0x098C, 0x272B, 0 },		/* MODE_SENSOR_ROW_SPEED_B */
	{ 0x0990, 0x2111, 0 },		/* 8465 - datasheet default */
	{ 0x098C, 0x272D, 0 },		/* MODE_SENSOR_READ_MODE_B */
	{ 0x0990, 0x0024, 0 },		/* 36 - no skip, x-mirror */
	{ 0x098C, 0x272F, 0 },		/* MODE_SENSOR_FINE_CORRECTION_B */
	{ 0x0990, 0x00A4, 0 },		/* 164 - datasheet default */
	{ 0x098C, 0x2731, 0 },		/* MODE_SENSOR_FINE_IT_MIN_B */
	{ 0x0990, 0x0408, 0 },		/* 1032 - datasheet default */
	{ 0x098C, 0x2733, 0 },		/* MODE_SENSOR_FINE_IT_MAX_MARGIN_B */
	{ 0x0990, 0x00A4, 0 },		/* 164 - datasheet default */
	{ 0x098C, 0x2735, 0 },		/* MODE_SENSOR_FRAME_LENGTH_B */
	{ 0x0990, 0x04ED, 0 },		/* 1261 - datasheet default */
	{ 0x098C, 0x2737, 0 },		/* MODE_SENSOR_LINE_LENGTH_PCK_B */
	{ 0x0990, 0x0D06, 0 },		/* 3334 - datasheet default */

	/* Crop configuration - Context A (640x480 output) */
	{ 0x098C, 0x2739, 0 },		/* MODE_CROP_X0_A */
	{ 0x0990, 0x0000, 0 },		/* 0 */
	{ 0x098C, 0x273B, 0 },		/* MODE_CROP_X1_A */
	{ 0x0990, 0x027F, 0 },		/* 639 */
	{ 0x098C, 0x273D, 0 },		/* MODE_CROP_Y0_A */
	{ 0x0990, 0x0000, 0 },		/* 0 */
	{ 0x098C, 0x273F, 0 },		/* MODE_CROP_Y1_A */
	{ 0x0990, 0x01DF, 0 },		/* 479 - for 480 output height */

	/* Crop configuration - Context B (1280x1024 output) */
	{ 0x098C, 0x2747, 0 },		/* MODE_CROP_X0_B */
	{ 0x0990, 0x0000, 0 },		/* 0 */
	{ 0x098C, 0x2749, 0 },		/* MODE_CROP_X1_B */
	{ 0x0990, 0x04FF, 0 },		/* 1279 */
	{ 0x098C, 0x274B, 0 },		/* MODE_CROP_Y0_B */
	{ 0x0990, 0x0000, 0 },		/* 0 */
	{ 0x098C, 0x274D, 0 },		/* MODE_CROP_Y1_B */
	{ 0x0990, 0x03FF, 0 },		/* 1023 */

	/* Flicker detection */
	{ 0x098C, 0x222D, 0 },
	{ 0x0990, 0x00CC, 0 },
	{ 0x098C, 0xA404, 0 },
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0xA408, 0 },
	{ 0x0990, 0x0032, 0 },
	{ 0x098C, 0xA409, 0 },
	{ 0x0990, 0x0034, 0 },
	{ 0x098C, 0xA40A, 0 },
	{ 0x0990, 0x003C, 0 },
	{ 0x098C, 0xA40B, 0 },
	{ 0x0990, 0x003E, 0 },
	{ 0x098C, 0x2411, 0 },		/* FD_R9_STEP_F60_A */
	{ 0x0990, 0x009D, 0 },		/* 157 - datasheet default */
	{ 0x098C, 0x2413, 0 },		/* FD_R9_STEP_F50_A */
	{ 0x0990, 0x00BC, 0 },		/* 188 - datasheet default */
	{ 0x098C, 0x2415, 0 },		/* FD_R9_STEP_F60_B */
	{ 0x0990, 0x0000, 0 },		/* 0 - datasheet default */
	{ 0x098C, 0x2417, 0 },		/* FD_R9_STEP_F50_B */
	{ 0x0990, 0x00E0, 0 },		/* 224 - datasheet default */
	{ 0x098C, 0xA40D, 0 },
	{ 0x0990, 0x0002, 0 },
	{ 0x098C, 0xA40E, 0 },
	{ 0x0990, 0x0003, 0 },
	{ 0x098C, 0xA410, 0 },
	{ 0x0990, 0x000A, 0 },

	/* Sensor core reserved registers */
	{ 0x3044, 0x0504, 0 },
	{ 0x3086, 0x24F7, 0 },
	{ 0x3088, 0xF059, 0 },
	{ 0x3090, 0x0716, 0 },
	{ 0x3092, 0xAB1F, 0 },
	{ 0x30D4, 0x9020, 0 },
	{ 0x30E2, 0x6645, 0 },
	{ 0x30E4, 0x7A66, 0 },
	{ 0x30E6, 0x6652, 0 },
	{ 0x30E8, 0x7766, 0 },
	{ 0x30EA, 0x2E03, 0 },
	{ 0x30EC, 0x452E, 0 },
	{ 0x30EE, 0x2E17, 0 },
	{ 0x30F0, 0x452E, 0 },
	{ 0x30F6, 0x0501, 0 },
	{ 0x30F8, 0x0501, 0 },
	{ 0x30FA, 0x0401, 0 },
	{ 0x30FC, 0x0401, 0 },
	{ 0x30FE, 0x5145, 0 },
	{ 0x3100, 0x4F45, 0 },
	{ 0x3102, 0x652E, 0 },
	{ 0x3104, 0x7552, 0 },
	{ 0x3106, 0x2D05, 0 },
	{ 0x3108, 0x4405, 0 },
	{ 0x311A, 0x5045, 0 },
	{ 0x311E, 0x0601, 0 },
	{ 0x3122, 0x0601, 0 },
	{ 0x316C, 0x8406, 0 },

	/* Noise reduction */
	{ 0x098C, 0xAB2D, 0 },
	{ 0x0990, 0x002A, 0 },
	{ 0x098C, 0xAB31, 0 },
	{ 0x0990, 0x002E, 0 },

	/* Low-light enhancement */
	{ 0x098C, 0x2B28, 0 },
	{ 0x0990, 0x1F40, 0 },
	{ 0x098C, 0x2B2A, 0 },
	{ 0x0990, 0x3A98, 0 },
	{ 0x098C, 0x2B38, 0 },
	{ 0x0990, 0x1F40, 0 },
	{ 0x098C, 0x2B3A, 0 },
	{ 0x0990, 0x3A98, 0 },

	/* AE settings */
	{ 0x098C, 0x2257, 0 },
	{ 0x0990, 0x2710, 0 },
	{ 0x098C, 0x2250, 0 },
	{ 0x0990, 0x1B58, 0 },
	{ 0x098C, 0x2252, 0 },
	{ 0x0990, 0x32C8, 0 },
	{ 0x098C, 0xA24B, 0 },
	{ 0x0990, 0x0082, 0 },

	/* Aperture */
	{ 0x326C, 0x0C00, 0 },

	/* More Context A settings */
	{ 0x098C, 0x2717, 0 },
	{ 0x0990, 0x046C, 0 },
	{ 0x098C, 0x2719, 0 },
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0x271B, 0 },
	{ 0x0990, 0x01F1, 0 },
	{ 0x098C, 0x271D, 0 },
	{ 0x0990, 0x013F, 0 },
	{ 0x098C, 0x271F, 0 },
	{ 0x0990, 0x032E, 0 },
	{ 0x098C, 0x2721, 0 },
	{ 0x0990, 0x04CC, 0 },
	{ 0x098C, 0x275F, 0 },
	{ 0x0990, 0x0596, 0 },
	{ 0x098C, 0x2761, 0 },
	{ 0x0990, 0x0094, 0 },

	/* Lens shading correction */
	{ 0x364E, 0x07B0, 0 },
	{ 0x3650, 0x7E0E, 0 },
	{ 0x3652, 0x3D31, 0 },
	{ 0x3654, 0x80AE, 0 },
	{ 0x3656, 0xE131, 0 },
	{ 0x3658, 0x01B0, 0 },
	{ 0x365A, 0x878D, 0 },
	{ 0x365C, 0x2671, 0 },
	{ 0x365E, 0x7D2D, 0 },
	{ 0x3660, 0xA5D1, 0 },
	{ 0x3662, 0x03B0, 0 },
	{ 0x3664, 0x5A0E, 0 },
	{ 0x3666, 0x0E71, 0 },
	{ 0x3668, 0x99EE, 0 },
	{ 0x366A, 0xA671, 0 },
	{ 0x366C, 0x0170, 0 },
	{ 0x366E, 0xF44D, 0 },
	{ 0x3670, 0x2971, 0 },
	{ 0x3672, 0x2D4A, 0 },
	{ 0x3674, 0xD671, 0 },
	{ 0x3676, 0x674C, 0 },
	{ 0x3678, 0x748D, 0 },
	{ 0x367A, 0x3FEE, 0 },
	{ 0x367C, 0x89AE, 0 },
	{ 0x367E, 0xB410, 0 },
	{ 0x3680, 0x168C, 0 },
	{ 0x3682, 0xC56D, 0 },
	{ 0x3684, 0x7CAC, 0 },
	{ 0x3686, 0x038F, 0 },
	{ 0x3688, 0xA86F, 0 },
	{ 0x368A, 0xDB6B, 0 },
	{ 0x368C, 0xA2AE, 0 },
	{ 0x368E, 0xFA8D, 0 },
	{ 0x3690, 0x5C8E, 0 },
	{ 0x3692, 0x740C, 0 },
	{ 0x3694, 0x9F4B, 0 },
	{ 0x3696, 0x1C4D, 0 },
	{ 0x3698, 0x978D, 0 },
	{ 0x369A, 0x21EC, 0 },
	{ 0x369C, 0xF5AD, 0 },
	{ 0x369E, 0x7D10, 0 },
	{ 0x36A0, 0x3E2E, 0 },
	{ 0x36A2, 0x8953, 0 },
	{ 0x36A4, 0xD910, 0 },
	{ 0x36A6, 0x3033, 0 },
	{ 0x36A8, 0x06D1, 0 },
	{ 0x36AA, 0xAD4E, 0 },
	{ 0x36AC, 0xD2D2, 0 },
	{ 0x36AE, 0x5CCE, 0 },
	{ 0x36B0, 0x3B93, 0 },
	{ 0x36B2, 0x50D0, 0 },
	{ 0x36B4, 0x79AD, 0 },
	{ 0x36B6, 0xDFF2, 0 },
	{ 0x36B8, 0x88AF, 0 },
	{ 0x36BA, 0x2453, 0 },
	{ 0x36BC, 0x0051, 0 },
	{ 0x36BE, 0x81CF, 0 },
	{ 0x36C0, 0x8313, 0 },
	{ 0x36C2, 0x2250, 0 },
	{ 0x36C4, 0x4A53, 0 },
	{ 0x36C6, 0x0C8D, 0 },
	{ 0x36C8, 0x362B, 0 },
	{ 0x36CA, 0xAD51, 0 },
	{ 0x36CC, 0xA470, 0 },
	{ 0x36CE, 0x3DD2, 0 },
	{ 0x36D0, 0x174C, 0 },
	{ 0x36D2, 0x152F, 0 },
	{ 0x36D4, 0x82F1, 0 },
	{ 0x36D6, 0xDED0, 0 },
	{ 0x36D8, 0x6F12, 0 },
	{ 0x36DA, 0xD36C, 0 },
	{ 0x36DC, 0x51AE, 0 },
	{ 0x36DE, 0xD0AE, 0 },
	{ 0x36E0, 0x274E, 0 },
	{ 0x36E2, 0x25F2, 0 },
	{ 0x36E4, 0xDCCA, 0 },
	{ 0x36E6, 0x438E, 0 },
	{ 0x36E8, 0xD64E, 0 },
	{ 0x36EA, 0x8A71, 0 },
	{ 0x36EC, 0x1492, 0 },
	{ 0x36EE, 0xD5B1, 0 },
	{ 0x36F0, 0xEBF0, 0 },
	{ 0x36F2, 0x53F3, 0 },
	{ 0x36F4, 0x3492, 0 },
	{ 0x36F6, 0x9AF4, 0 },
	{ 0x36F8, 0x8BF1, 0 },
	{ 0x36FA, 0x204F, 0 },
	{ 0x36FC, 0x3A93, 0 },
	{ 0x36FE, 0xB551, 0 },
	{ 0x3700, 0xE214, 0 },
	{ 0x3702, 0xF2B0, 0 },
	{ 0x3704, 0x8C30, 0 },
	{ 0x3706, 0x3053, 0 },
	{ 0x3708, 0x64F0, 0 },
	{ 0x370A, 0xFC73, 0 },
	{ 0x370C, 0xD311, 0 },
	{ 0x370E, 0x336F, 0 },
	{ 0x3710, 0x5AF3, 0 },
	{ 0x3712, 0x4EAF, 0 },
	{ 0x3714, 0xDBD4, 0 },

	/* Lens shading origin */
	{ 0x3644, 0x02A0, 0 },
	{ 0x3642, 0x01FC, 0 },
	{ 0x3210, 0x01B8, 0 },

	/* Color correction matrix - Low light */
	{ 0x098C, 0x2306, 0 },
	{ 0x0990, 0x0233, 0 },
	{ 0x098C, 0x2308, 0 },
	{ 0x0990, 0xFF0B, 0 },
	{ 0x098C, 0x230A, 0 },
	{ 0x0990, 0x0024, 0 },
	{ 0x098C, 0x230C, 0 },
	{ 0x0990, 0xFFC8, 0 },
	{ 0x098C, 0x230E, 0 },
	{ 0x0990, 0x01DE, 0 },
	{ 0x098C, 0x2310, 0 },
	{ 0x0990, 0xFFBD, 0 },
	{ 0x098C, 0x2312, 0 },
	{ 0x0990, 0x0019, 0 },
	{ 0x098C, 0x2314, 0 },
	{ 0x0990, 0xFF2B, 0 },
	{ 0x098C, 0x2316, 0 },
	{ 0x0990, 0x01E8, 0 },
	{ 0x098C, 0x2318, 0 },
	{ 0x0990, 0x0024, 0 },
	{ 0x098C, 0x231A, 0 },
	{ 0x0990, 0x0030, 0 },

	/* Color correction matrix - RL (delta) */
	{ 0x098C, 0x231C, 0 },
	{ 0x0990, 0xFF7D, 0 },
	{ 0x098C, 0x231E, 0 },
	{ 0x0990, 0x002C, 0 },
	{ 0x098C, 0x2320, 0 },
	{ 0x0990, 0x002C, 0 },
	{ 0x098C, 0x2322, 0 },
	{ 0x0990, 0x0006, 0 },
	{ 0x098C, 0x2324, 0 },
	{ 0x0990, 0x00A3, 0 },
	{ 0x098C, 0x2326, 0 },
	{ 0x0990, 0xFF75, 0 },
	{ 0x098C, 0x2328, 0 },
	{ 0x0990, 0xFFF4, 0 },
	{ 0x098C, 0x232A, 0 },
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0x232C, 0 },
	{ 0x0990, 0xFF75, 0 },
	{ 0x098C, 0x232E, 0 },
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0x2330, 0 },
	{ 0x0990, 0xFFF4, 0 },

	/* AWB settings */
	{ 0x098C, 0xA348, 0 },
	{ 0x0990, 0x0008, 0 },
	{ 0x098C, 0xA349, 0 },
	{ 0x0990, 0x0002, 0 },
	{ 0x098C, 0xA34A, 0 },
	{ 0x0990, 0x0059, 0 },
	{ 0x098C, 0xA34B, 0 },
	{ 0x0990, 0x00A6, 0 },
	{ 0x098C, 0xA351, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xA352, 0 },
	{ 0x0990, 0x007F, 0 },
	{ 0x098C, 0xA35D, 0 },
	{ 0x0990, 0x0078, 0 },
	{ 0x098C, 0xA35E, 0 },
	{ 0x0990, 0x0086, 0 },
	{ 0x098C, 0xA35F, 0 },
	{ 0x0990, 0x007E, 0 },
	{ 0x098C, 0xA360, 0 },
	{ 0x0990, 0x0082, 0 },

	/* Cold color adjustment */
	{ 0x098C, 0xA369, 0 },
	{ 0x0990, 0x0097, 0 },
	{ 0x098C, 0xA36A, 0 },
	{ 0x0990, 0x008C, 0 },
	{ 0x098C, 0xA36B, 0 },
	{ 0x0990, 0x0080, 0 },

	/* AWB window */
	{ 0x098C, 0xA302, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xA303, 0 },
	{ 0x0990, 0x00FF, 0 },

	/* AE preview settings */
	{ 0x098C, 0xA11D, 0 },
	{ 0x0990, 0x0002, 0 },
	{ 0x098C, 0x271F, 0 },
	{ 0x0990, 0x032E, 0 },
	{ 0x098C, 0x2721, 0 },
	{ 0x0990, 0x04CC, 0 },

	/* AE gain settings */
	{ 0x098C, 0xA216, 0 },
	{ 0x0990, 0x0060, 0 },
	{ 0x098C, 0xA215, 0 },
	{ 0x0990, 0x000A, 0 },
	{ 0x098C, 0xA20C, 0 },
	{ 0x0990, 0x0028, 0 },
	{ 0x098C, 0xA24F, 0 },
	{ 0x0990, 0x0042, 0 },
	{ 0x098C, 0xA20E, 0 },
	{ 0x0990, 0x0060, 0 },

	/* AE window */
	{ 0x098C, 0xA202, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xA203, 0 },
	{ 0x0990, 0x00FF, 0 },
	{ 0x098C, 0xA207, 0 },
	{ 0x0990, 0x0004, 0 },

	/* Gamma morph control */
	{ 0x098C, 0xAB37, 0 },
	{ 0x0990, 0x0003, 0 },
	{ 0x098C, 0x2B38, 0 },
	{ 0x0990, 0x3A98, 0 },
	{ 0x098C, 0x2B3A, 0 },
	{ 0x0990, 0x5000, 0 },

	/* Saturation */
	{ 0x098C, 0xAB20, 0 },
	{ 0x0990, 0x0023, 0 },
	{ 0x098C, 0xAB24, 0 },
	{ 0x0990, 0x0010, 0 },

	/* AE speed */
	{ 0x098C, 0xA109, 0 },
	{ 0x0990, 0x0020, 0 },
	{ 0x098C, 0xA10A, 0 },
	{ 0x0990, 0x0002, 0 },

	/* Gamma table A */
	{ 0x098C, 0xAB3C, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xAB3D, 0 },
	{ 0x0990, 0x0006, 0 },
	{ 0x098C, 0xAB3E, 0 },
	{ 0x0990, 0x0014, 0 },
	{ 0x098C, 0xAB3F, 0 },
	{ 0x0990, 0x0038, 0 },
	{ 0x098C, 0xAB40, 0 },
	{ 0x0990, 0x005F, 0 },
	{ 0x098C, 0xAB41, 0 },
	{ 0x0990, 0x0079, 0 },
	{ 0x098C, 0xAB42, 0 },
	{ 0x0990, 0x008D, 0 },
	{ 0x098C, 0xAB43, 0 },
	{ 0x0990, 0x009E, 0 },
	{ 0x098C, 0xAB44, 0 },
	{ 0x0990, 0x00AC, 0 },
	{ 0x098C, 0xAB45, 0 },
	{ 0x0990, 0x00B8, 0 },
	{ 0x098C, 0xAB46, 0 },
	{ 0x0990, 0x00C3, 0 },
	{ 0x098C, 0xAB47, 0 },
	{ 0x0990, 0x00CD, 0 },
	{ 0x098C, 0xAB48, 0 },
	{ 0x0990, 0x00D5, 0 },
	{ 0x098C, 0xAB49, 0 },
	{ 0x0990, 0x00DE, 0 },
	{ 0x098C, 0xAB4A, 0 },
	{ 0x0990, 0x00E5, 0 },
	{ 0x098C, 0xAB4B, 0 },
	{ 0x0990, 0x00EC, 0 },
	{ 0x098C, 0xAB4C, 0 },
	{ 0x0990, 0x00F3, 0 },
	{ 0x098C, 0xAB4D, 0 },
	{ 0x0990, 0x00F9, 0 },
	{ 0x098C, 0xAB4E, 0 },
	{ 0x0990, 0x00FF, 0 },

	/* Noise reduction RGB */
	{ 0x098C, 0xAB2C, 0 },
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0xAB2D, 0 },
	{ 0x0990, 0x002A, 0 },
	{ 0x098C, 0xAB2E, 0 },
	{ 0x0990, 0x0010, 0 },
	{ 0x098C, 0xAB2F, 0 },
	{ 0x0990, 0x0010, 0 },

	/* Gamma table B */
	{ 0x098C, 0xAB4F, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0xAB50, 0 },
	{ 0x0990, 0x0004, 0 },
	{ 0x098C, 0xAB51, 0 },
	{ 0x0990, 0x000D, 0 },
	{ 0x098C, 0xAB52, 0 },
	{ 0x0990, 0x0028, 0 },
	{ 0x098C, 0xAB53, 0 },
	{ 0x0990, 0x0053, 0 },
	{ 0x098C, 0xAB54, 0 },
	{ 0x0990, 0x0075, 0 },
	{ 0x098C, 0xAB55, 0 },
	{ 0x0990, 0x0092, 0 },
	{ 0x098C, 0xAB56, 0 },
	{ 0x0990, 0x00A7, 0 },
	{ 0x098C, 0xAB57, 0 },
	{ 0x0990, 0x00B7, 0 },
	{ 0x098C, 0xAB58, 0 },
	{ 0x0990, 0x00C4, 0 },
	{ 0x098C, 0xAB59, 0 },
	{ 0x0990, 0x00CF, 0 },
	{ 0x098C, 0xAB5A, 0 },
	{ 0x0990, 0x00D8, 0 },
	{ 0x098C, 0xAB5B, 0 },
	{ 0x0990, 0x00DF, 0 },
	{ 0x098C, 0xAB5C, 0 },
	{ 0x0990, 0x00E6, 0 },
	{ 0x098C, 0xAB5D, 0 },
	{ 0x0990, 0x00EC, 0 },
	{ 0x098C, 0xAB5E, 0 },
	{ 0x0990, 0x00F2, 0 },
	{ 0x098C, 0xAB5F, 0 },
	{ 0x0990, 0x00F6, 0 },
	{ 0x098C, 0xAB60, 0 },
	{ 0x0990, 0x00FB, 0 },
	{ 0x098C, 0xAB61, 0 },
	{ 0x0990, 0x00FF, 0 },

	/* Read mode - no mirror/flip */
	{ 0x098C, 0x2717, 0 },
	{ 0x0990, 0x046C, 0 },
	{ 0x098C, 0x272D, 0 },
	{ 0x0990, 0x0024, 0 },

	/* Reset command before sequencer */
	{ 0x001A, 0x021C, 0 },

	/* Issue refresh command */
	{ 0x098C, 0xA103, 0 },
	{ 0x0990, 0x0006, 0 },
};

/* Apply the MT9M113 initialization table */
static int mt9m113_sensor_init(struct mt9m113 *sensor)
{
	struct device *dev = &sensor->client->dev;
	int ret = 0;
	unsigned int i;

	dev_info(dev, "MT9M113: applying init table (%zu entries)\n",
		 ARRAY_SIZE(mt9m113_init_table));

	for (i = 0; i < ARRAY_SIZE(mt9m113_init_table); i++) {
		const struct mt9m113_reg_entry *entry = &mt9m113_init_table[i];

		ret = cci_write(sensor->regmap, CCI_REG16(entry->reg),
				entry->value, NULL);
		if (ret < 0) {
			dev_err(dev, "MT9M113: reg 0x%04x write failed: %d\n",
				entry->reg, ret);
			return ret;
		}

		if (entry->delay_ms > 0)
			msleep(entry->delay_ms);
	}

	/* Wait for MCU to complete refresh */
	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 1000);
	if (ret < 0) {
		dev_err(dev, "MT9M113: MCU refresh timeout\n");
		return ret;
	}

	/* Issue sequencer refresh */
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				    MT9M113_SEQ_CMD_REFRESH);
	if (ret < 0)
		return ret;

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 1000);
	if (ret < 0) {
		dev_err(dev, "MT9M113: SEQ refresh timeout\n");
		return ret;
	}

	/* Configure short packets */
	if (!mt9m113_skip_short_pkt) {
		ret = cci_write(sensor->regmap, MT9M113_CUSTOM_SHORT_PKT,
				MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN, NULL);
		if (ret < 0)
			return ret;
	}

	/* Disable MIPI output until streaming starts */
	ret = cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL, 0x0000, NULL);
	if (ret < 0)
		return ret;

	dev_info(dev, "MT9M113: init complete\n");
	return 0;
}

/* -----------------------------------------------------------------------------
 * Streaming
 */

/*
 * Configure AE (Auto Exposure) parameters for preview vs snapshot/video mode.
 * From webOS driver: snapshot mode allows longer exposure for better quality,
 * while preview mode optimizes for higher frame rate and lower power.
 */
static int mt9m113_configure_ae_mode(struct mt9m113 *sensor, bool snapshot_mode)
{
	struct device *dev = &sensor->client->dev;
	int ret;

	if (snapshot_mode) {
		/* Snapshot/Capture mode: allow longer exposure for quality */
		dev_dbg(dev, "MT9M113: configuring AE for snapshot mode\n");
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_MAX_INDEX, 0x0028);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_MAX_VIRTGAIN, 0x0060);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_MAX_DGAIN_AE1, 0x00C8);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_JUMP_DIVISOR, 0x0002);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_SKIP_FRAMES, 0x0002);
		if (ret)
			return ret;
	} else {
		/* Preview mode: optimize for frame rate and responsiveness */
		dev_dbg(dev, "MT9M113: configuring AE for preview mode\n");
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_MAX_INDEX, 0x0008);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_MAX_VIRTGAIN, 0x00A0);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_MAX_DGAIN_AE1, 0x0150);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_JUMP_DIVISOR, 0x0001);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_SKIP_FRAMES, 0x0001);
		if (ret)
			return ret;
	}

	return 0;
}

static int mt9m113_start_streaming(struct mt9m113 *sensor,
				   struct v4l2_subdev_state *state)
{
	struct device *dev = &sensor->client->dev;
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *compose;
	bool use_context_b;
	u16 output_ctrl_val;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	/* MCU health check */
	{
		u64 health_check;

		ret = mt9m113_read_mcu_var(sensor, 0x2703, &health_check);
		if (ret < 0 || health_check == 0) {
			dev_warn(dev, "MT9M113: MCU unresponsive, re-init\n");
			ret = mt9m113_sensor_init(sensor);
			if (ret < 0)
				goto error;
		}
	}

	/* Apply all V4L2 controls (color effects, etc.) before streaming */
	ret = __v4l2_ctrl_handler_setup(&sensor->ifp.hdl);
	if (ret) {
		dev_err(dev, "Failed to setup controls: %d\n", ret);
		goto error;
	}

	compose = v4l2_subdev_state_get_compose(state, 0);
	/* Get source pad format (pad 1) for MIPI output configuration */
	format = v4l2_subdev_state_get_format(state, 1);

	/* Determine context based on resolution */
	use_context_b = (compose->width > 640 || compose->height > 480);

	dev_info(dev, "MT9M113: %ux%u -> Context %c\n",
		 compose->width, compose->height, use_context_b ? 'B' : 'A');

	/*
	 * Configure AE parameters for the selected context:
	 * - Context B (snapshot/capture): longer exposure allowed for quality
	 * - Context A (preview): optimize for frame rate and responsiveness
	 */
	ret = mt9m113_configure_ae_mode(sensor, use_context_b);
	if (ret) {
		dev_err(dev, "Failed to configure AE mode: %d\n", ret);
		goto error;
	}

	/* Wait for CSIPHY stabilization */
	msleep(mt9m113_pre_mipi_delay_ms);

	/* Configure MIPI output based on source pad format */
	{
		bool is_bayer = (format->code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
				 format->code == MEDIA_BUS_FMT_SGRBG10_1X10);

		if (is_bayer && !mt9m113_fake_yuv) {
			if (format->code == MEDIA_BUS_FMT_SGRBG10_1X10)
				output_ctrl_val = MT9M113_OUTPUT_CONTROL_MIPI_RAW10;
			else
				output_ctrl_val = MT9M113_OUTPUT_CONTROL_MIPI_RAW8;
			dev_info(dev, "MT9M113: MIPI output RAW mode (code=0x%04x)\n",
				 format->code);
		} else {
			output_ctrl_val = MT9M113_OUTPUT_CONTROL_MIPI_ENABLE;
			dev_info(dev, "MT9M113: MIPI output YUV mode (code=0x%04x)\n",
				 format->code);
		}

		if (mt9m113_cont_mipi_clk)
			output_ctrl_val |= 0x0004;

		ret = cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL,
				output_ctrl_val, NULL);
		if (ret)
			goto error;

		/* Refresh after OUTPUT_CONTROL change */
		mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				      MT9M113_SEQ_CMD_REFRESH_MODE);
		mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
		mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				      MT9M113_SEQ_CMD_REFRESH);
		mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
	}

	/* Set RESET_REGISTER for streaming */
	ret = cci_write(sensor->regmap, MT9M113_RESET_REGISTER,
			MT9M113_RESET_REG_STREAMING, NULL);
	if (ret)
		goto error;

	/*
	 * Configure capture/streaming mode.
	 *
	 * SEQ_CAP_MODE (0xa115) bit 1 controls video vs capture mode:
	 * - Bit 1 = 0: Capture mode - returns to preview after N frames
	 * - Bit 1 = 1: Video mode - stays in Context B until explicit switch
	 *
	 * For V4L2 continuous streaming, we use video mode (0x0002) so the
	 * sequencer stays in Context B indefinitely. SEQ_CAP_NUM_FRAMES is
	 * only used in capture mode (bit 1 = 0).
	 */
	if (use_context_b) {
		mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CAP_MODE, 0x0002);
	} else {
		mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CAP_MODE, 0x0030);
	}

	msleep(40);

	/*
	 * Issue SEQ_CMD to start streaming.
	 *
	 * SEQ_CMD_RUN (0x01): Continue streaming in current context
	 * SEQ_CMD_CAPTURE (0x02): Switch to Context B and capture
	 *
	 * For Context B, we use CAPTURE to trigger the context switch, but
	 * with SEQ_CAP_NUM_FRAMES=0 for infinite/continuous streaming.
	 * For Context A, we use RUN for continuous preview.
	 */
	if (use_context_b) {
		ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
					    MT9M113_SEQ_CMD_CAPTURE);
	} else {
		ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
					    MT9M113_SEQ_CMD_RUN);
	}
	if (ret)
		goto error;

	mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);

	/* Wait for context switch if using Context B */
	if (use_context_b) {
		u64 seq_state;
		int i;

		for (i = 0; i < 100; i++) {
			mt9m113_read_mcu_var(sensor, MT9M113_SEQ_STATE, &seq_state);
			if (seq_state == 0x07)
				break;
			msleep(10);
		}

		if (seq_state != 0x07) {
			dev_err(dev, "MT9M113: Context B switch failed\n");
			ret = -ETIMEDOUT;
			goto error;
		}
		msleep(50);
	} else {
		msleep(20);
	}

	/* Re-write OUTPUT_CONTROL after SEQ_CMD (MCU may reset it) */
	ret = cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL,
			output_ctrl_val, NULL);
	if (ret)
		goto error;

	sensor->streaming = true;
	return 0;

error:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int mt9m113_stop_streaming(struct mt9m113 *sensor)
{
	struct device *dev = &sensor->client->dev;

	sensor->streaming = false;

	/* Disable MIPI output */
	cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL, 0x0000, NULL);
	dev_info(dev, "MT9M113: streaming stopped\n");

	pm_runtime_put_autosuspend(dev);
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static inline struct mt9m113 *pa_to_mt9m113(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m113, pa.sd);
}

static inline struct mt9m113 *ifp_to_mt9m113(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m113, ifp.sd);
}

static int mt9m113_ifp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (enable)
		ret = mt9m113_start_streaming(sensor, state);
	else
		ret = mt9m113_stop_streaming(sensor);

	v4l2_subdev_unlock_state(state);
	return ret;
}

/* -----------------------------------------------------------------------------
 * Pixel Array Subdev Operations
 */

static int mt9m113_pa_init_state(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = MT9M113_PIXEL_ARRAY_WIDTH;
	crop->height = MT9M113_PIXEL_ARRAY_HEIGHT;

	format = v4l2_subdev_state_get_format(state, 0);
	format->width = MT9M113_PIXEL_ARRAY_WIDTH;
	format->height = MT9M113_PIXEL_ARRAY_HEIGHT;
	format->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

static int mt9m113_pa_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int mt9m113_pa_enum_framesizes(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = MT9M113_PIXEL_ARRAY_WIDTH;
	fse->max_width = MT9M113_PIXEL_ARRAY_WIDTH;
	fse->min_height = MT9M113_PIXEL_ARRAY_HEIGHT;
	fse->max_height = MT9M113_PIXEL_ARRAY_HEIGHT;

	return 0;
}

static int mt9m113_pa_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = MT9M113_PIXEL_ARRAY_WIDTH;
		sel->r.height = MT9M113_PIXEL_ARRAY_HEIGHT;
		return 0;

	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_pad_ops mt9m113_pa_pad_ops = {
	.enum_mbus_code = mt9m113_pa_enum_mbus_code,
	.enum_frame_size = mt9m113_pa_enum_framesizes,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = mt9m113_pa_get_selection,
};

static const struct v4l2_subdev_ops mt9m113_pa_ops = {
	.pad = &mt9m113_pa_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9m113_pa_internal_ops = {
	.init_state = mt9m113_pa_init_state,
};

/* -----------------------------------------------------------------------------
 * IFP Subdev Operations
 */

static int mt9m113_ifp_init_state(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	struct v4l2_rect *compose;

	/* Sink pad - receives raw data from PA */
	format = v4l2_subdev_state_get_format(state, 0);
	format->width = MT9M113_PIXEL_ARRAY_WIDTH;
	format->height = MT9M113_PIXEL_ARRAY_HEIGHT;
	format->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = MT9M113_PIXEL_ARRAY_WIDTH;
	crop->height = MT9M113_PIXEL_ARRAY_HEIGHT;

	compose = v4l2_subdev_state_get_compose(state, 0);
	compose->left = 0;
	compose->top = 0;
	compose->width = 640;
	compose->height = 480;

	/* Source pad - outputs processed data to host */
	format = v4l2_subdev_state_get_format(state, 1);
	format->width = 640;
	format->height = 480;
	format->code = MEDIA_BUS_FMT_UYVY8_1X16;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int mt9m113_ifp_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	/* Sink pad only accepts raw from PA */
	if (code->pad == 0) {
		if (code->index > 0)
			return -EINVAL;
		code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
		return 0;
	}

	/* Source pad supports multiple output formats */
	if (code->index >= ARRAY_SIZE(mt9m113_format_infos))
		return -EINVAL;

	code->code = mt9m113_format_infos[code->index].code;
	return 0;
}

static int mt9m113_ifp_enum_frame_size(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	/* Sink pad - fixed raw input size */
	if (fse->pad == 0) {
		if (fse->index > 0)
			return -EINVAL;
		if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
			return -EINVAL;
		fse->min_width = MT9M113_PIXEL_ARRAY_WIDTH;
		fse->max_width = MT9M113_PIXEL_ARRAY_WIDTH;
		fse->min_height = MT9M113_PIXEL_ARRAY_HEIGHT;
		fse->max_height = MT9M113_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	/* Source pad - Context A (640x480) and Context B (1280x1024) */
	if (fse->index > 1)
		return -EINVAL;

	if (fse->index == 0) {
		fse->min_width = 640;
		fse->max_width = 640;
		fse->min_height = 480;
		fse->max_height = 480;
	} else {
		fse->min_width = 1280;
		fse->max_width = 1280;
		fse->min_height = 1024;
		fse->max_height = 1024;
	}

	return 0;
}

static int mt9m113_ifp_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *compose;
	const struct mt9m113_format_info *info;

	/* Sink pad format is fixed */
	if (fmt->pad == 0) {
		format = v4l2_subdev_state_get_format(state, 0);
		fmt->format = *format;
		return 0;
	}

	/* Source pad */
	info = mt9m113_format_info(fmt->format.code);

	/* Clamp to supported sizes (Context A or Context B) */
	if (fmt->format.width <= 640) {
		fmt->format.width = 640;
		fmt->format.height = 480;
	} else {
		fmt->format.width = 1280;
		fmt->format.height = 1024;
	}

	format = v4l2_subdev_state_get_format(state, 1);
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = info->code;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	compose = v4l2_subdev_state_get_compose(state, 0);
	compose->width = format->width;
	compose->height = format->height;

	fmt->format = *format;
	return 0;
}

static int mt9m113_ifp_get_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_selection *sel)
{
	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
		sel->r = *v4l2_subdev_state_get_compose(state, 0);
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = MT9M113_PIXEL_ARRAY_WIDTH;
		sel->r.height = MT9M113_PIXEL_ARRAY_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = MT9M113_PIXEL_ARRAY_WIDTH;
		sel->r.height = MT9M113_PIXEL_ARRAY_HEIGHT;
		return 0;

	default:
		return -EINVAL;
	}
}

/* IFP registered callback - registers PA and creates link */
static void mt9m113_ifp_unregistered(struct v4l2_subdev *sd)
{
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);

	v4l2_device_unregister_subdev(&sensor->pa.sd);
}

static int mt9m113_ifp_registered(struct v4l2_subdev *sd)
{
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);
	int ret;

	ret = v4l2_device_register_subdev(sd->v4l2_dev, &sensor->pa.sd);
	if (ret < 0) {
		dev_err(&sensor->client->dev,
			"Failed to register pixel array subdev\n");
		return ret;
	}

	ret = media_create_pad_link(&sensor->pa.sd.entity, 0,
				    &sensor->ifp.sd.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret < 0) {
		dev_err(&sensor->client->dev,
			"Failed to link pixel array to ifp\n");
		v4l2_device_unregister_subdev(&sensor->pa.sd);
		return ret;
	}

	return 0;
}

static const struct v4l2_subdev_video_ops mt9m113_ifp_video_ops = {
	.s_stream = mt9m113_ifp_s_stream,
};

static const struct v4l2_subdev_pad_ops mt9m113_ifp_pad_ops = {
	.enum_mbus_code = mt9m113_ifp_enum_mbus_code,
	.enum_frame_size = mt9m113_ifp_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mt9m113_ifp_set_fmt,
	.get_selection = mt9m113_ifp_get_selection,
};

static const struct v4l2_subdev_ops mt9m113_ifp_ops = {
	.video = &mt9m113_ifp_video_ops,
	.pad = &mt9m113_ifp_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9m113_ifp_internal_ops = {
	.init_state = mt9m113_ifp_init_state,
	.registered = mt9m113_ifp_registered,
	.unregistered = mt9m113_ifp_unregistered,
};

/* -----------------------------------------------------------------------------
 * Controls
 */

static const char * const mt9m113_context_menu[] = {
	"Context A (640x480)",
	"Context B (1280x1024)",
};

static int mt9m113_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m113 *sensor = container_of(ctrl->handler,
					       struct mt9m113, ifp.hdl);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(&sensor->client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_MT9M113_CONTEXT:
		if (sensor->streaming) {
			if (ctrl->val == MT9M113_CONTEXT_B) {
				/* Video mode (bit 1=1) for continuous streaming */
				mt9m113_write_mcu_var(sensor,
					MT9M113_SEQ_CAP_MODE, 0x0002);
				ret = mt9m113_write_mcu_var(sensor,
					MT9M113_SEQ_CMD,
					MT9M113_SEQ_CMD_CAPTURE);
			} else {
				ret = mt9m113_write_mcu_var(sensor,
					MT9M113_SEQ_CMD,
					MT9M113_SEQ_CMD_RUN);
			}
			mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD,
					     0x0000, 500);
		}
		break;

	case V4L2_CID_COLORFX: {
		u16 effect;

		switch (ctrl->val) {
		case V4L2_COLORFX_NONE:
			effect = MT9M113_SPEC_EFFECTS_NONE;
			break;
		case V4L2_COLORFX_BW:
			effect = MT9M113_SPEC_EFFECTS_MONOCHROME;
			break;
		case V4L2_COLORFX_SEPIA:
			effect = MT9M113_SPEC_EFFECTS_SEPIA;
			break;
		case V4L2_COLORFX_NEGATIVE:
			effect = MT9M113_SPEC_EFFECTS_NEGATIVE;
			break;
		case V4L2_COLORFX_SOLARIZATION:
			effect = MT9M113_SPEC_EFFECTS_SOLARIZE;
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (!ret) {
			effect |= (MT9M113_SPEC_EFFECTS_DEFAULT &
				   ~MT9M113_SPEC_EFFECTS_MASK);
			/* MCU variables need indirect access via 0x098C/0x0990 */
			ret = mt9m113_write_mcu_var(sensor, 0x2759, effect);
			if (!ret)
				ret = mt9m113_write_mcu_var(sensor, 0x275B, effect);

			/* Issue REFRESH to apply the effect change */
			if (!ret) {
				mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
						      MT9M113_SEQ_CMD_REFRESH);
				mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD,
						     0x0000, 500);
			}
		}
		break;
	}

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(&sensor->client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops mt9m113_ctrl_ops = {
	.s_ctrl = mt9m113_s_ctrl,
};

static const struct v4l2_ctrl_config mt9m113_context_ctrl_cfg = {
	.ops = &mt9m113_ctrl_ops,
	.id = V4L2_CID_MT9M113_CONTEXT,
	.name = "MT9M113 Context",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = MT9M113_CONTEXT_A,
	.max = MT9M113_CONTEXT_B,
	.def = MT9M113_CONTEXT_A,
	.qmenu = mt9m113_context_menu,
};

/* -----------------------------------------------------------------------------
 * Power Management
 */

static int mt9m113_power_on(struct mt9m113 *sensor)
{
	struct device *dev = &sensor->client->dev;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0)
		return ret;

	if (sensor->powerdown)
		gpiod_set_value(sensor->powerdown, 0);

	usleep_range(20000, 25000);

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto error_regulator;

	msleep(20);

	if (sensor->reset) {
		gpiod_set_value(sensor->reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value(sensor->reset, 0);
		usleep_range(44500, 50000);
	} else if (sensor->powerdown) {
		usleep_range(44500, 50000);
	}

	/* MT9M113 MCU boot sequence */
	{
		u64 clocks_val = 0;

		cci_read(sensor->regmap, MT9M113_CLOCKS_CONTROL, &clocks_val, NULL);
		if (clocks_val != 0) {
			dev_info(dev, "MT9M113 already initialized\n");
			msleep(50);
			return 0;
		}

		/* Soft reset */
		cci_write(sensor->regmap, MT9M113_RESET_AND_MISC_CONTROL,
			  MT9M113_RESET_SOC, &ret);
		cci_write(sensor->regmap, MT9M113_RESET_AND_MISC_CONTROL,
			  0, &ret);
		if (ret < 0)
			goto error_clock;
		msleep(200);

		/* Boot MCU */
		cci_write(sensor->regmap, MT9M113_MCU_BOOT_MODE, 0x0001, &ret);
		usleep_range(1000, 2000);
		cci_write(sensor->regmap, MT9M113_MCU_BOOT_MODE, 0x0000, &ret);
		if (ret < 0)
			goto error_clock;
		msleep(200);

		/* Configure PLL */
		cci_write(sensor->regmap, MT9M113_CLOCKS_CONTROL, 0x00FF, &ret);
		cci_write(sensor->regmap, MT9M113_STANDBY_CONTROL, 0x0028, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x2145, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x2145, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x2145, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_DIVIDERS, 0x0114, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_P_DIVIDERS, 0x00F1, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x2545, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x2547, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x3447, &ret);
		if (ret < 0)
			goto error_clock;
		msleep(20);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x3047, &ret);
		cci_write(sensor->regmap, MT9M113_PLL_CONTROL, 0x3046, &ret);
		cci_write(sensor->regmap, MT9M113_RESET_AND_MISC_CONTROL, 0x0218, &ret);
		cci_write(sensor->regmap, MT9M113_STANDBY_CONTROL, 0x002A, &ret);
		if (ret < 0)
			goto error_clock;
		msleep(50);

		/* Configure OFIFO */
		cci_write(sensor->regmap, MT9M113_OFIFO_CONTROL_STATUS, 0x0003, &ret);
		if (ret < 0)
			goto error_clock;
	}

	return 0;

error_clock:
	clk_disable_unprepare(sensor->clk);
error_regulator:
	if (!sensor->powerdown)
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
	return ret;
}

static void mt9m113_power_off(struct mt9m113 *sensor)
{
	if (sensor->powerdown) {
		gpiod_set_value(sensor->powerdown, 1);
		clk_disable_unprepare(sensor->clk);
	} else {
		clk_disable_unprepare(sensor->clk);
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
	}
}

static int __maybe_unused mt9m113_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);
	int ret;

	if (sensor->powerdown)
		return 0;

	ret = mt9m113_power_on(sensor);
	if (ret)
		return ret;

	return mt9m113_sensor_init(sensor);
}

static int __maybe_unused mt9m113_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);

	if (sensor->powerdown)
		return 0;

	mt9m113_power_off(sensor);
	return 0;
}

static const struct dev_pm_ops mt9m113_pm_ops = {
	SET_RUNTIME_PM_OPS(mt9m113_runtime_suspend, mt9m113_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int mt9m113_identify(struct mt9m113 *sensor)
{
	u64 value;
	int ret;

	ret = cci_read(sensor->regmap, MT9M113_CHIP_ID, &value, NULL);
	if (ret) {
		dev_err(&sensor->client->dev, "Failed to read chip ID\n");
		return -ENXIO;
	}

	if (value != MT9M113_CHIP_ID_VALUE) {
		dev_err(&sensor->client->dev,
			"Invalid chip ID 0x%04llx (expected 0x%04x)\n",
			value, MT9M113_CHIP_ID_VALUE);
		return -ENXIO;
	}

	dev_info(&sensor->client->dev, "MT9M113 detected (ID 0x%04llx)\n",
		 value);
	return 0;
}

static int mt9m113_parse_dt(struct mt9m113 *sensor)
{
	struct fwnode_handle *fwnode = dev_fwnode(&sensor->client->dev);
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	sensor->bus_cfg.bus_type = V4L2_MBUS_UNKNOWN;
	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &sensor->bus_cfg);
	fwnode_handle_put(ep);
	if (ret < 0)
		return ret;

	if (sensor->bus_cfg.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(&sensor->client->dev, "Unsupported bus type\n");
		v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
		return -EINVAL;
	}

	return 0;
}

static int mt9m113_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mt9m113 *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->client = client;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	ret = mt9m113_parse_dt(sensor);
	if (ret < 0)
		return ret;

	sensor->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->clk)) {
		ret = PTR_ERR(sensor->clk);
		goto error_ep_free;
	}

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sensor->reset)) {
		ret = PTR_ERR(sensor->reset);
		goto error_ep_free;
	}

	sensor->powerdown = devm_gpiod_get_optional(dev, "powerdown",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->powerdown)) {
		ret = PTR_ERR(sensor->powerdown);
		goto error_ep_free;
	}

	sensor->supplies[0].supply = "vddio";
	sensor->supplies[1].supply = "vdd";
	sensor->supplies[2].supply = "vaa";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret < 0)
		goto error_ep_free;

	/* Power on and identify */
	ret = mt9m113_power_on(sensor);
	if (ret < 0)
		goto error_ep_free;

	ret = mt9m113_identify(sensor);
	if (ret < 0)
		goto error_power_off;

	ret = mt9m113_sensor_init(sensor);
	if (ret < 0)
		goto error_power_off;

	/* Calculate pixel rate from input clock
	 * MT9M113 PLL: MCLK=27MHz, typical output ~54MHz pixel clock
	 * Using half the input clock rate as a reasonable default
	 */
	sensor->pixrate = clk_get_rate(sensor->clk);
	if (sensor->pixrate == 0)
		sensor->pixrate = 27000000; /* Default 27MHz if clock rate unknown */
	sensor->pixrate = sensor->pixrate * 2; /* PLL typically doubles the rate */

	dev_info(dev, "MT9M113: pixel rate %u Hz\n", sensor->pixrate);

	/* Initialize Pixel Array subdev */
	v4l2_subdev_init(&sensor->pa.sd, &mt9m113_pa_ops);
	sensor->pa.sd.internal_ops = &mt9m113_pa_internal_ops;
	v4l2_i2c_subdev_set_name(&sensor->pa.sd, client, NULL, " pixel array");
	sensor->pa.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pa.sd.owner = THIS_MODULE;
	sensor->pa.sd.dev = dev;
	v4l2_set_subdevdata(&sensor->pa.sd, client);
	sensor->pa.sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pa.pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->pa.sd.entity, 1, &sensor->pa.pad);
	if (ret < 0)
		goto error_power_off;

	/* Initialize PA controls - PIXEL_RATE is needed by camss for link freq */
	v4l2_ctrl_handler_init(&sensor->pa.hdl, 1);
	v4l2_ctrl_new_std(&sensor->pa.hdl, NULL, V4L2_CID_PIXEL_RATE,
			  sensor->pixrate, sensor->pixrate, 1, sensor->pixrate);
	if (sensor->pa.hdl.error) {
		ret = sensor->pa.hdl.error;
		goto error_pa_hdl;
	}
	sensor->pa.sd.state_lock = sensor->pa.hdl.lock;

	ret = v4l2_subdev_init_finalize(&sensor->pa.sd);
	if (ret < 0)
		goto error_pa_hdl;

	sensor->pa.sd.ctrl_handler = &sensor->pa.hdl;

	/* Initialize IFP subdev */
	v4l2_i2c_subdev_init(&sensor->ifp.sd, client, &mt9m113_ifp_ops);
	v4l2_i2c_subdev_set_name(&sensor->ifp.sd, client, NULL, " ifp");
	sensor->ifp.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->ifp.sd.internal_ops = &mt9m113_ifp_internal_ops;
	sensor->ifp.sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	sensor->ifp.pads[0].flags = MEDIA_PAD_FL_SINK;
	sensor->ifp.pads[1].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->ifp.sd.entity, 2, sensor->ifp.pads);
	if (ret < 0)
		goto error_pa_subdev;

	/* Initialize controls on IFP */
	v4l2_ctrl_handler_init(&sensor->ifp.hdl, 5);
	sensor->ifp.context = v4l2_ctrl_new_custom(&sensor->ifp.hdl,
						   &mt9m113_context_ctrl_cfg, NULL);
	v4l2_ctrl_new_std_menu(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			       V4L2_CID_COLORFX,
			       V4L2_COLORFX_SOLARIZATION, 0,
			       V4L2_COLORFX_NONE);

	/* Link frequency control - required by CSIPHY
	 * Use DT link-frequencies if available, otherwise calculate default
	 * For MIPI CSI-2: link_freq = pixel_rate * bpp / (2 * lanes)
	 * MT9M113: 1 lane, YUV422 16bpp -> link_freq = pixel_rate * 8
	 */
	{
		static s64 default_link_freq;
		struct v4l2_ctrl *link_freq;
		const s64 *frequencies;
		unsigned int nfreqs;

		if (sensor->bus_cfg.nr_of_link_frequencies > 0) {
			frequencies = sensor->bus_cfg.link_frequencies;
			nfreqs = sensor->bus_cfg.nr_of_link_frequencies;
		} else {
			/* Default: pixel_rate * 8 for 1-lane YUV422 */
			default_link_freq = (s64)sensor->pixrate * 8;
			frequencies = &default_link_freq;
			nfreqs = 1;
			dev_info(dev, "MT9M113: using default link freq %lld Hz\n",
				 default_link_freq);
		}

		link_freq = v4l2_ctrl_new_int_menu(&sensor->ifp.hdl, NULL,
						   V4L2_CID_LINK_FREQ,
						   nfreqs - 1, 0, frequencies);
		if (link_freq)
			link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	/* Pixel rate control */
	v4l2_ctrl_new_std(&sensor->ifp.hdl, NULL, V4L2_CID_PIXEL_RATE,
			  sensor->pixrate, sensor->pixrate, 1, sensor->pixrate);

	if (sensor->ifp.hdl.error) {
		ret = sensor->ifp.hdl.error;
		goto error_ifp_entity;
	}

	sensor->ifp.sd.ctrl_handler = &sensor->ifp.hdl;
	sensor->ifp.sd.state_lock = sensor->ifp.hdl.lock;

	ret = v4l2_subdev_init_finalize(&sensor->ifp.sd);
	if (ret < 0)
		goto error_ifp_handler;

	/* Enable runtime PM */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	/* Register only the IFP - PA will be registered in ifp_registered callback */
	ret = v4l2_async_register_subdev(&sensor->ifp.sd);
	if (ret < 0)
		goto error_pm;

	pm_runtime_put_autosuspend(dev);

	dev_info(dev, "MT9M113 driver with IFP sub-device initialized\n");
	return 0;

error_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	v4l2_subdev_cleanup(&sensor->ifp.sd);
error_ifp_handler:
	v4l2_ctrl_handler_free(&sensor->ifp.hdl);
error_ifp_entity:
	media_entity_cleanup(&sensor->ifp.sd.entity);
error_pa_subdev:
	v4l2_subdev_cleanup(&sensor->pa.sd);
error_pa_hdl:
	v4l2_ctrl_handler_free(&sensor->pa.hdl);
error_pa_entity:
	media_entity_cleanup(&sensor->pa.sd.entity);
error_power_off:
	mt9m113_power_off(sensor);
error_ep_free:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static void mt9m113_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(&sensor->ifp.sd);
	v4l2_subdev_cleanup(&sensor->ifp.sd);
	v4l2_ctrl_handler_free(&sensor->ifp.hdl);
	media_entity_cleanup(&sensor->ifp.sd.entity);

	v4l2_subdev_cleanup(&sensor->pa.sd);
	v4l2_ctrl_handler_free(&sensor->pa.hdl);
	media_entity_cleanup(&sensor->pa.sd.entity);

	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		mt9m113_power_off(sensor);
	pm_runtime_set_suspended(dev);
}

static const struct of_device_id mt9m113_of_ids[] = {
	{ .compatible = "aptina,mt9m113" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt9m113_of_ids);

static struct i2c_driver mt9m113_driver = {
	.driver = {
		.name	= "mt9m113",
		.pm	= &mt9m113_pm_ops,
		.of_match_table = mt9m113_of_ids,
	},
	.probe		= mt9m113_probe,
	.remove		= mt9m113_remove,
};

module_i2c_driver(mt9m113_driver);

MODULE_DESCRIPTION("Aptina MT9M113 Sensor Driver");
MODULE_LICENSE("GPL");
