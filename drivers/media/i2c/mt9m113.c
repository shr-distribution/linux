// SPDX-License-Identifier: GPL-2.0-only
/*
 * mt9m113.c Aptina MT9M113 sensor driver
 *
 * Copyright (C) 2024 Herman van Hazendonk <github.com@herrie.org>
 *
 * MT9M113 is a 1.3MP SOC sensor with dual context support:
 *   Context A: 640x480 preview mode (binned)
 *   Context B: 1280x1024 capture mode (full resolution)
 *
 * Ported from legacy vendor kernel mt9m113.c/mt9m113_reg.c.
 * Uses MCU indirect access (0x098C/0x0990) for configuration.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
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

/* Delay before enabling MIPI output, allowing CSIPHY to stabilize. */
#define MT9M113_PRE_MIPI_DELAY_MS	10

/*
 * The MT9M113 MCU intermittently wedges on stream start (SEQ_CMD stuck,
 * 0xa103 timeout, SEQ_STATE never reaching preview 0x04 / capture 0x07).
 * The failures cluster at the start of a run and then clear, so a bounded
 * retry that power-cycles the sensor between attempts reliably gets the
 * stream up.  3 attempts is enough in practice (with the per-session
 * runtime-PM re-init as the primary fix); a larger cap would stack
 * power-cycle + ~600-entry init-table replay into a STREAMON latency
 * userspace clients treat as a hang.  Total attempts (1 initial + retries).
 */
#define MT9M113_STREAM_START_RETRIES	3

/* MT9M113 chip ID */
#define MT9M113_CHIP_ID				CCI_REG16(0x0000)
#define MT9M113_CHIP_ID_VALUE			0x2480

/* Sysctl registers */
#define MT9M113_PLL_DIVIDERS			CCI_REG16(0x0010)
#define MT9M113_PLL_P_DIVIDERS			CCI_REG16(0x0012)
#define MT9M113_PLL_CONTROL			CCI_REG16(0x0014)
#define MT9M113_CLOCKS_CONTROL			CCI_REG16(0x0016)
#define MT9M113_STANDBY_CONTROL			CCI_REG16(0x0018)
#define MT9M113_STANDBY_CONTROL_ACTIVE		0x0028	/* MCU running, not in standby */
#define MT9M113_RESET_AND_MISC_CONTROL		CCI_REG16(0x001a)
#define MT9M113_RESET_SOC			BIT(0)
#define MT9M113_MCU_BOOT_MODE			CCI_REG16(0x001c)

/* MCU indirect access registers */
#define MT9M113_MCU_ADDRESS			CCI_REG16(0x098c)
#define MT9M113_MCU_DATA			CCI_REG16(0x0990)

/* MCU variable addresses */
#define MT9M113_SEQ_CMD				0xa103
#define MT9M113_SEQ_CMD_RUN			0x0001
#define MT9M113_SEQ_CMD_CAPTURE			0x0002
#define MT9M113_SEQ_CMD_REFRESH			0x0005
#define MT9M113_SEQ_CMD_REFRESH_MODE		0x0006
#define MT9M113_SEQ_STATE			0xa104
/*
 * Per datasheet R0x0004 seq_state values: 0x02 = Enter preview, 0x03 =
 * Preview (stable), 0x04 = Leave preview (transient, only seen while
 * exiting preview for a context switch). 0x07 is stable Capture
 * (Context B). The Context-B preview-first step must wait for the
 * stable 0x03, not the transient 0x04 - waiting for 0x04 here misses
 * the window and times out on every Context B stream-start.
 */
#define MT9M113_SEQ_STATE_PREVIEW		0x03
#define MT9M113_SEQ_STATE_CAPTURE		0x07
#define MT9M113_SEQ_CAP_MODE			0xa115
#define MT9M113_SEQ_CAP_MODE_PREVIEW		0x0030	/* continuous preview */
#define MT9M113_SEQ_CAP_MODE_VIDEO		0x0002	/* stay in Context B */

/* Mode Output dimension/format MCU variables (Driver ID 7) */
#define MT9M113_MODE_OUTPUT_WIDTH_A		0x2703
#define MT9M113_MODE_OUTPUT_HEIGHT_A		0x2705
#define MT9M113_MODE_OUTPUT_WIDTH_B		0x2707
#define MT9M113_MODE_OUTPUT_HEIGHT_B		0x2709
#define MT9M113_MODE_OUTPUT_FORMAT_A		0x2755
#define MT9M113_MODE_OUTPUT_FORMAT_B		0x2757
#define MT9M113_MODE_OUTPUT_FORMAT_YUV		0x0000
/*
 * YUV byte ordering (datasheet table 31 "mode_output_format_a/b"):
 *   default (0x0000)   = Cb Y0 Cr Y1 = UYVY
 *   SWAP_LUMA_CHROMA   = Y0 Cb Y1 Cr = YUYV
 */
#define MT9M113_MODE_OUTPUT_FORMAT_SWAP_LUMA_CHROMA	BIT(1)

/* Special effects MCU variables */
#define MT9M113_MODE_SPEC_EFFECTS_A		0x2759
#define MT9M113_MODE_SPEC_EFFECTS_B		0x275b
#define MT9M113_SPEC_EFFECTS_DEFAULT		0x6440
#define MT9M113_SPEC_EFFECTS_MASK		0x0007
#define MT9M113_SPEC_EFFECTS_NONE		0x0000
#define MT9M113_SPEC_EFFECTS_MONOCHROME		0x0001
#define MT9M113_SPEC_EFFECTS_SEPIA		0x0002
#define MT9M113_SPEC_EFFECTS_NEGATIVE		0x0003
#define MT9M113_SPEC_EFFECTS_SOLARIZE		0x0004

/* Sensor Read Mode MCU variables (for flip/mirror control) */
#define MT9M113_SENSOR_READ_MODE_A		0x2717
#define MT9M113_SENSOR_READ_MODE_B		0x272d
#define MT9M113_SENSOR_READ_MODE_HMIRROR	BIT(0)
#define MT9M113_SENSOR_READ_MODE_VMIRROR	BIT(1)

/*
 * Context A/B sensor configuration MCU variables
 * These control the sensor readout window and must be re-applied when
 * switching contexts to ensure correct line counts.
 */
#define MT9M113_MODE_SENSOR_ROW_START_A		0x270D
#define MT9M113_MODE_SENSOR_COL_START_A		0x270F
#define MT9M113_MODE_SENSOR_ROW_END_A		0x2711
#define MT9M113_MODE_SENSOR_COL_END_A		0x2713
#define MT9M113_MODE_SENSOR_ROW_SPEED_A		0x2715
#define MT9M113_MODE_SENSOR_FRAME_LENGTH_A	0x271F
#define MT9M113_MODE_SENSOR_LINE_LENGTH_PCK_A	0x2721

#define MT9M113_MODE_SENSOR_ROW_START_B		0x2723
#define MT9M113_MODE_SENSOR_COL_START_B		0x2725
#define MT9M113_MODE_SENSOR_ROW_END_B		0x2727
#define MT9M113_MODE_SENSOR_COL_END_B		0x2729
#define MT9M113_MODE_SENSOR_ROW_SPEED_B		0x272B
#define MT9M113_MODE_SENSOR_FRAME_LENGTH_B	0x2735
#define MT9M113_MODE_SENSOR_LINE_LENGTH_PCK_B	0x2737

/* Context A sensor config values (from legacy vendor driver, 640x480 binned) */
#define MT9M113_CONTEXT_A_ROW_START		0x0000
#define MT9M113_CONTEXT_A_COL_START		0x0000
#define MT9M113_CONTEXT_A_ROW_END		0x03CD	/* 973 */
#define MT9M113_CONTEXT_A_COL_END		0x050D	/* 1293 */
#define MT9M113_CONTEXT_A_ROW_SPEED		0x2111
#define MT9M113_CONTEXT_A_READ_MODE		0x046C	/* Binning enabled */
#define MT9M113_CONTEXT_A_FRAME_LENGTH		0x032E	/* 814 lines (incl vblank) */
#define MT9M113_CONTEXT_A_LINE_LENGTH_PCK	0x04CC	/* 1228 pixclks/line */

/* Context B sensor config values (from legacy vendor driver, 1280x1024 full res) */
#define MT9M113_CONTEXT_B_ROW_START		0x0004
#define MT9M113_CONTEXT_B_COL_START		0x0004
#define MT9M113_CONTEXT_B_ROW_END		0x040B	/* 1035 */
#define MT9M113_CONTEXT_B_COL_END		0x050B	/* 1291 */
#define MT9M113_CONTEXT_B_ROW_SPEED		0x2111
#define MT9M113_CONTEXT_B_READ_MODE		0x0024	/* Full resolution */
#define MT9M113_CONTEXT_B_FRAME_LENGTH		0x0559	/* 1369 lines (incl vblank) */
#define MT9M113_CONTEXT_B_LINE_LENGTH_PCK	0x0722	/* 1826 pixclks/line */

/* Auto Exposure MCU variables (for preview vs snapshot optimization) */
#define MT9M113_AE_MAX_INDEX			0xa20c
#define MT9M113_AE_MAX_VIRTGAIN			0xa20e
#define MT9M113_AE_MAX_DGAIN_AE1		0xa21a
#define MT9M113_AE_JUMP_DIVISOR			0xa21c
#define MT9M113_AE_SKIP_FRAMES			0xa21e

/* Flicker detection MCU variable */
#define MT9M113_FD_MODE				0xa404
#define MT9M113_FD_MODE_DISABLED		0x00
#define MT9M113_FD_MODE_50HZ			0x01
#define MT9M113_FD_MODE_60HZ			0x02
#define MT9M113_FD_MODE_AUTO			0x03

/* AWB/Color MCU variables */
#define MT9M113_AWB_SATURATION			0xa354
#define MT9M113_AWB_MODE			0xa34a

/* Test pattern MCU variables (mode_common_mode_settings) */
#define MT9M113_CAM_MODE_SELECT			0xc84c
#define MT9M113_CAM_MODE_SELECT_NORMAL		0x00

/*
 * Test pattern register (Driver 7 R0x0066 mode_common_mode_settings_test_mode,
 * MT9M113 datasheet "Table 31: Driver ID = 7: Mode Variables"). MCU logical
 * addresses are NOT a clean (driver_id << 12) | offset encoding - each
 * driver has its own hardcoded base page set by the chip firmware. Driver 7
 * Mode variables share base 0x2700 (see MT9M113_MODE_OUTPUT_WIDTH_A = 0x2703
 * for R0x0003, MT9M113_MODE_SPEC_EFFECTS_A = 0x2759 for R0x0059, etc.), so
 * R0x0066 maps to 0x2766.
 *
 * Per the datasheet "Test Pattern Generator. Changes take effect only
 * after REFRESH command", so writes are paired with mt9m113_refresh()
 * when streaming. The datasheet also recommends disabling the MCU before
 * enabling test patterns (BOOT_MODE=1 halt), which
 * mt9m113_apply_test_pattern() performs after the REFRESH succeeds.
 *
 * Do NOT go via the CAM_MODE driver's test-pattern knob (CAM_MODE_SELECT =
 * 0x02 TEST_PATTERN + CAM_MODE_TEST_PATTERN_SELECT in Driver 8): switching
 * CAM_MODE into TEST_PATTERN mode wedges the sequencer so SEQ_CMD never
 * processes REFRESH and stream_on() hangs.
 *
 * NOTE: forensic investigation on observed silicon shows the IFP TPG MUX
 * does NOT actually engage despite mode_test_mode accepting every value:
 *
 *   - R0x2766 (mode_test_mode) is a read/write logical variable, but the
 *     MCU firmware resets it to a default (0x8e8e) on every standby wake.
 *   - Hardware registers associated with the TPG MUX (R0x3246-R0x3252)
 *     are non-responsive dead zones, and bit 4 of R0x321C is hardwired
 *     to 0 - both classic Aptina foundry-removal fingerprints.
 *   - No raw sensor-core TPG (R0x3070-R0x3071) is present.
 *   - Confirmed by stripping the color pipeline (R0x3210=0) and
 *     verifying that even in a "naked" IFP state the live sensor data
 *     persists without any pattern injection.
 *
 * The V4L2_CID_TEST_PATTERN plumbing below is intentionally retained on
 * the (faint) chance a different silicon variant or vendor SROM patch
 * actually wires up the TPG block. Setting test_pattern > 0 on the
 * current die is harmless - the writes succeed but no pattern appears.
 */
#define MT9M113_MODE_TEST_MODE			0x2766
#define MT9M113_TEST_MODE_DISABLED		0x00
#define MT9M113_TEST_MODE_SOLID_WHITE		0x01
#define MT9M113_TEST_MODE_GREY_RAMP		0x02
#define MT9M113_TEST_MODE_COLOR_BAR_RAMP	0x03
#define MT9M113_TEST_MODE_SOLID_WHITE_COLOR_BARS 0x04
#define MT9M113_TEST_MODE_NOISE			0x05

/* Double buffer control register */
#define MT9M113_DOUBLE_BUFFER_CONTROL		CCI_REG16(0x0248)
#define MT9M113_DOUBLE_BUFFER_SUSPEND		BIT(15)

/* Auto Exposure MCU variables */
#define MT9M113_AE_GATE				0xa207
#define MT9M113_AE_GATE_ENABLE			0x0000
#define MT9M113_AE_GATE_DISABLE			0x00FF

/* Sensor core exposure/gain registers */
#define MT9M113_COARSE_IT_TIME_A		CCI_REG16(0x3012)
#define MT9M113_COARSE_IT_TIME_B		CCI_REG16(0x3014)
#define MT9M113_ANALOG_GAIN			CCI_REG16(0x3028)

/* Sensor core registers */
#define MT9M113_RESET_REGISTER			CCI_REG16(0x301a)
#define MT9M113_RESET_REG_STREAMING		0x120C
#define MT9M113_OFIFO_CONTROL_STATUS		CCI_REG16(0x321c)
#define MT9M113_OFIFO_BYPASS			0x0003	/* FIFO bypass (YUV path) */

/* Color pipeline (ISP) control */
#define MT9M113_COLOR_PIPELINE_CONTROL		CCI_REG16(0x3210)
#define MT9M113_COLOR_PIPELINE_DEFAULT		0x01b8	/* scaler+gamma+ap+shading */

/* OUTPUT_CONTROL register (0x3400) - MIPI control */
#define MT9M113_OUTPUT_CONTROL			CCI_REG16(0x3400)
#define MT9M113_OUTPUT_CONTROL_MIPI_ENABLE	0x7A08	/* YUV422 dt=0x1E */

/* CUSTOM_SHORT_PKT register */
#define MT9M113_CUSTOM_SHORT_PKT		CCI_REG16(0x3404)
#define MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN	0x0080

/* Pixel array dimensions */
#define MT9M113_PIXEL_ARRAY_WIDTH		1296U
#define MT9M113_PIXEL_ARRAY_HEIGHT		1040U

/* -----------------------------------------------------------------------------
 * Data Structures
 */

struct mt9m113_format_info {
	u32 code;
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
	s64 link_freq;
	bool streaming;
	bool test_pattern_active;
	u8 test_pattern_value;	/* last V4L2_CID_TEST_PATTERN menu index */

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
	} ifp;
};

/* -----------------------------------------------------------------------------
 * Formats
 */

/*
 * Source-pad output formats.  Only YUV422 is exposed: MT9M113_OUTPUT_CONTROL
 * advertises the YUV422 CSI-2 data type (0x1E) unconditionally on the wire,
 * and adding RGB565 would require also driving the matching CSI-2 dt (0x22)
 * out of OUTPUT_CONTROL — verifying the exact bit layout for that needs the
 * sensor datasheet and a hardware re-test, so RGB is deferred to a follow-up.
 */
static const struct mt9m113_format_info mt9m113_format_infos[] = {
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16 },
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16 },
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

/*
 * Map a V4L2 mbus code to the value programmed into MODE_OUTPUT_FORMAT_{A,B}.
 * Driving the chroma/luma swap bit is what differentiates UYVY from YUYV;
 * without it both codes would emit the same bytes on the MIPI bus.
 */
static u16 mt9m113_format_val(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return MT9M113_MODE_OUTPUT_FORMAT_YUV |
		       MT9M113_MODE_OUTPUT_FORMAT_SWAP_LUMA_CHROMA;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	default:
		return MT9M113_MODE_OUTPUT_FORMAT_YUV;
	}
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
	u64 value = 0;
	int ret;

	for (i = 0; i < timeout_ms / 10; i++) {
		ret = mt9m113_read_mcu_var(sensor, addr, &value);
		if (ret < 0)
			return ret;
		if (value == expected)
			return 0;
		usleep_range(10000, 11000);
	}

	dev_err(&sensor->client->dev,
		"MCU var 0x%04x timeout (got 0x%llx, expected 0x%04x)\n",
		addr, value, expected);
	return -ETIMEDOUT;
}

/*
 * Ensure MCU sequencer is idle before issuing a new command.
 * This prevents cascading failures where a timed-out command
 * causes subsequent commands to also fail.
 */
static int mt9m113_seq_cmd_ready(struct mt9m113 *sensor)
{
	u64 seq_cmd;
	int ret;

	ret = mt9m113_read_mcu_var(sensor, MT9M113_SEQ_CMD, &seq_cmd);
	if (ret < 0)
		return ret;

	if (seq_cmd == 0)
		return 0;  /* Already idle */

	/*
	 * A previous command is still pending. This can happen if:
	 * 1. Previous streaming start failed mid-way
	 * 2. The MCU is slow to process a command
	 *
	 * Wait up to 2 seconds for it to complete. This is longer than
	 * normal command timeouts to handle recovery scenarios.
	 */
	dev_dbg(&sensor->client->dev,
		 "MT9M113: SEQ_CMD=0x%llx, waiting for idle\n", seq_cmd);

	return mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 2000);
}

/*
 * mt9m113_refresh - Issue REFRESH_MODE + REFRESH sequence
 *
 * Per datasheet: "It is recommended that refresh and refresh mode commands
 * always be run together, and that refresh mode should be issued BEFORE
 * the refresh command."
 */
static int mt9m113_refresh(struct mt9m113 *sensor)
{
	int ret;

	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				    MT9M113_SEQ_CMD_REFRESH_MODE);
	if (ret)
		return ret;

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
	if (ret < 0)
		dev_warn(&sensor->client->dev, "MT9M113: REFRESH_MODE timeout\n");

	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				    MT9M113_SEQ_CMD_REFRESH);
	if (ret)
		return ret;

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
	if (ret < 0)
		dev_warn(&sensor->client->dev, "MT9M113: REFRESH timeout\n");

	return 0;
}

/*
 * Re-program SENSOR_READ_MODE_{A,B} for the context geometry while keeping
 * the live HFLIP/VFLIP bits, which the V4L2 control handler has already
 * applied via s_ctrl().  Writing CONTEXT_X_READ_MODE blindly would clear
 * mirror bits 0/1 and silently undo the user's flip on first stream-start.
 */
static int mt9m113_apply_read_mode(struct mt9m113 *sensor, u16 reg, u16 base)
{
	const u16 mirror_mask = MT9M113_SENSOR_READ_MODE_HMIRROR |
				MT9M113_SENSOR_READ_MODE_VMIRROR;
	u64 cur = 0;
	int ret;

	ret = mt9m113_read_mcu_var(sensor, reg, &cur);
	if (ret)
		return ret;

	return mt9m113_write_mcu_var(sensor, reg,
				     (base & ~mirror_mask) |
				     ((u16)cur & mirror_mask));
}

/**
 * mt9m113_configure_sensor_context - Configure sensor readout parameters
 * @sensor: MT9M113 sensor device
 * @use_context_b: true for Context B (1280x1024), false for Context A (640x480)
 *
 * Re-applies the sensor configuration registers for the selected context.
 * This is necessary because the MT9M113's MCU variables can become corrupted
 * or stale after switching between contexts. Without explicitly re-writing
 * these values, Context A may use incorrect row_end and read_mode values
 * after running Context B, resulting in wrong line counts and corrupted images.
 *
 * Must be called before mt9m113_refresh() to ensure the new values take effect.
 */
static int mt9m113_configure_sensor_context(struct mt9m113 *sensor,
					    bool use_context_b)
{
	struct device *dev = &sensor->client->dev;
	int ret;

	if (use_context_b) {
		/* Context B: 1280x1024 full resolution */
		dev_dbg(dev, "MT9M113: Configuring Context B sensor params\n");

		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_ROW_START_B,
					    MT9M113_CONTEXT_B_ROW_START);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_COL_START_B,
					    MT9M113_CONTEXT_B_COL_START);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_ROW_END_B,
					    MT9M113_CONTEXT_B_ROW_END);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_COL_END_B,
					    MT9M113_CONTEXT_B_COL_END);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_ROW_SPEED_B,
					    MT9M113_CONTEXT_B_ROW_SPEED);
		if (ret)
			return ret;
		ret = mt9m113_apply_read_mode(sensor,
					      MT9M113_SENSOR_READ_MODE_B,
					      MT9M113_CONTEXT_B_READ_MODE);
		if (ret)
			return ret;
		/*
		 * Frame/line length define the total frame period (active +
		 * blanking). They must be re-applied with the rest of the
		 * context geometry: the VFE CAMIF counts lines to find frame
		 * boundaries (no MIPI FE short packet), so a stale period makes
		 * the captured image walk vertically frame-over-frame.
		 */
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_FRAME_LENGTH_B,
					    MT9M113_CONTEXT_B_FRAME_LENGTH);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_LINE_LENGTH_PCK_B,
					    MT9M113_CONTEXT_B_LINE_LENGTH_PCK);
		if (ret)
			return ret;
	} else {
		/* Context A: 640x480 binned preview */
		dev_dbg(dev, "MT9M113: Configuring Context A sensor params\n");

		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_ROW_START_A,
					    MT9M113_CONTEXT_A_ROW_START);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_COL_START_A,
					    MT9M113_CONTEXT_A_COL_START);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_ROW_END_A,
					    MT9M113_CONTEXT_A_ROW_END);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_COL_END_A,
					    MT9M113_CONTEXT_A_COL_END);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_ROW_SPEED_A,
					    MT9M113_CONTEXT_A_ROW_SPEED);
		if (ret)
			return ret;
		ret = mt9m113_apply_read_mode(sensor,
					      MT9M113_SENSOR_READ_MODE_A,
					      MT9M113_CONTEXT_A_READ_MODE);
		if (ret)
			return ret;
		/* See Context B above: frame period must track the geometry. */
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_FRAME_LENGTH_A,
					    MT9M113_CONTEXT_A_FRAME_LENGTH);
		if (ret)
			return ret;
		ret = mt9m113_write_mcu_var(sensor,
					    MT9M113_MODE_SENSOR_LINE_LENGTH_PCK_A,
					    MT9M113_CONTEXT_A_LINE_LENGTH_PCK);
		if (ret)
			return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Double Buffer Control
 *
 * The MT9M113 uses double buffering for context parameters. Setting bit 15
 * of register 0x0248 suspends updates from shadow to active registers,
 * allowing atomic multi-register configuration changes.
 */

static int mt9m113_double_buffer_suspend(struct mt9m113 *sensor)
{
	u64 value;
	int ret;

	ret = cci_read(sensor->regmap, MT9M113_DOUBLE_BUFFER_CONTROL, &value, NULL);
	if (ret)
		return ret;

	value |= MT9M113_DOUBLE_BUFFER_SUSPEND;
	return cci_write(sensor->regmap, MT9M113_DOUBLE_BUFFER_CONTROL, value, NULL);
}

static int mt9m113_double_buffer_resume(struct mt9m113 *sensor)
{
	u64 value;
	int ret;

	ret = cci_read(sensor->regmap, MT9M113_DOUBLE_BUFFER_CONTROL, &value, NULL);
	if (ret)
		return ret;

	value &= ~MT9M113_DOUBLE_BUFFER_SUSPEND;
	return cci_write(sensor->regmap, MT9M113_DOUBLE_BUFFER_CONTROL, value, NULL);
}

/* -----------------------------------------------------------------------------
 * Initialization Table (from legacy vendor kernel mt9m113_reg.c)
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

	/* Context A sensor configuration */
	{ 0x098C, 0x270D, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x270F, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x2711, 0 },		/* MODE_SENSOR_ROW_END_A */
	{ 0x0990, 0x03CD, 0 },		/* 973 (from legacy vendor driver) */
	{ 0x098C, 0x2713, 0 },
	{ 0x0990, 0x050D, 0 },
	{ 0x098C, 0x2715, 0 },
	{ 0x0990, 0x2111, 0 },
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

	/* Context B sensor configuration */
	{ 0x098C, 0x2723, 0 },
	{ 0x0990, 0x0004, 0 },
	{ 0x098C, 0x2725, 0 },
	{ 0x0990, 0x0004, 0 },
	{ 0x098C, 0x2727, 0 },
	{ 0x0990, 0x040B, 0 },
	{ 0x098C, 0x2729, 0 },
	{ 0x0990, 0x050B, 0 },
	{ 0x098C, 0x272B, 0 },
	{ 0x0990, 0x2111, 0 },
	{ 0x098C, 0x272D, 0 },
	{ 0x0990, 0x0024, 0 },
	{ 0x098C, 0x272F, 0 },
	{ 0x0990, 0x004C, 0 },
	{ 0x098C, 0x2731, 0 },
	{ 0x0990, 0x00F9, 0 },
	{ 0x098C, 0x2733, 0 },
	{ 0x0990, 0x00A7, 0 },
	{ 0x098C, 0x2735, 0 },
	{ 0x0990, 0x0559, 0 },
	{ 0x098C, 0x2737, 0 },
	{ 0x0990, 0x0722, 0 },

	/* Crop configuration - Context A */
	{ 0x098C, 0x2739, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x273B, 0 },
	{ 0x0990, 0x027F, 0 },
	{ 0x098C, 0x273D, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x273F, 0 },
	{ 0x0990, 0x01DF, 0 },

	/* Crop configuration - Context B */
	{ 0x098C, 0x2747, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x2749, 0 },
	{ 0x0990, 0x04FF, 0 },
	{ 0x098C, 0x274B, 0 },
	{ 0x0990, 0x0000, 0 },
	{ 0x098C, 0x274D, 0 },
	{ 0x0990, 0x03FF, 0 },

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
	{ 0x098C, 0x2411, 0 },
	{ 0x0990, 0x00CC, 0 },
	{ 0x098C, 0x2413, 0 },
	{ 0x0990, 0x00F4, 0 },
	{ 0x098C, 0x2415, 0 },
	{ 0x0990, 0x0089, 0 },
	{ 0x098C, 0x2417, 0 },
	{ 0x0990, 0x00A4, 0 },
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

	/* MCU is about to be fully re-initialised, so any prior test-pattern
	 * override is gone. Clear the bookkeeping so a subsequent
	 * V4L2_CID_TEST_PATTERN=0 does not re-run the disable sequence;
	 * also drop the cached pattern value so stream_on() doesn't
	 * replay the apply after a runtime suspend/resume cycle that
	 * already reset MCU state.
	 */
	sensor->test_pattern_active = false;
	sensor->test_pattern_value = 0;

	dev_dbg(dev, "MT9M113: applying init table (%zu entries)\n",
		 ARRAY_SIZE(mt9m113_init_table));

	/*
	 * Suspend double buffer updates so all init table writes take effect
	 * atomically at frame start. Per datasheet, R0x0248[15] inhibits
	 * transfers from pending to live registers.
	 */
	ret = mt9m113_double_buffer_suspend(sensor);
	if (ret) {
		dev_warn(dev, "Failed to suspend double buffer: %d\n", ret);
		/* Continue anyway - not fatal */
	}

	for (i = 0; i < ARRAY_SIZE(mt9m113_init_table); i++) {
		const struct mt9m113_reg_entry *entry = &mt9m113_init_table[i];

		ret = cci_write(sensor->regmap, CCI_REG16(entry->reg),
				entry->value, NULL);
		if (ret < 0) {
			dev_err(dev, "MT9M113: reg 0x%04x write failed: %d\n",
				entry->reg, ret);
			mt9m113_double_buffer_resume(sensor);
			return ret;
		}

		if (entry->delay_ms > 0)
			msleep(entry->delay_ms);
	}

	/* Resume double buffer updates - changes take effect at next frame start */
	ret = mt9m113_double_buffer_resume(sensor);
	if (ret) {
		dev_warn(dev, "Failed to resume double buffer: %d\n", ret);
		/* Continue anyway - not fatal */
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

	/*
	 * Enable MIPI Frame-Start/End short packets. Without them the VFE CAMIF
	 * has no per-frame boundary marker and runs in line-counting (APS) mode,
	 * which slips against the sensor's free-running readout and makes the
	 * captured image walk vertically (~tens of lines/frame, resolution
	 * dependent). Emitting the frame-sync short packets locks every frame.
	 */
	ret = cci_write(sensor->regmap, MT9M113_CUSTOM_SHORT_PKT,
			MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN, NULL);
	if (ret < 0)
		return ret;

	/*
	 * Do NOT enable MIPI output here. It is enabled in start_streaming once
	 * the output config is programmed, followed by REFRESH_MODE + REFRESH.
	 * Enabling MIPI before the CSIPHY is configured causes problems.
	 */

	dev_dbg(dev, "MT9M113: init complete\n");
	return 0;
}

/* -----------------------------------------------------------------------------
 * Streaming
 */

/*
 * Configure AE (Auto Exposure) parameters for preview vs snapshot/video mode.
 * From legacy vendor driver: snapshot mode allows longer exposure for better quality,
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

/*
 * Program the per-context output dimensions and pixel format, then re-apply the
 * sensor readout geometry. Output configuration is double-buffered so it
 * latches atomically at the next frame start.
 */
static int mt9m113_configure_output(struct mt9m113 *sensor, bool use_context_b,
				    u32 code)
{
	struct device *dev = &sensor->client->dev;
	u16 width_reg, height_reg, format_reg;
	u16 width_val, height_val, format_val;
	int ret;

	if (use_context_b) {
		width_reg = MT9M113_MODE_OUTPUT_WIDTH_B;
		height_reg = MT9M113_MODE_OUTPUT_HEIGHT_B;
		format_reg = MT9M113_MODE_OUTPUT_FORMAT_B;
		width_val = 1280;
		height_val = 1024;
	} else {
		width_reg = MT9M113_MODE_OUTPUT_WIDTH_A;
		height_reg = MT9M113_MODE_OUTPUT_HEIGHT_A;
		format_reg = MT9M113_MODE_OUTPUT_FORMAT_A;
		width_val = 640;
		height_val = 480;
	}

	format_val = mt9m113_format_val(code);

	ret = mt9m113_double_buffer_suspend(sensor);
	if (ret)
		dev_warn(dev, "Failed to suspend double buffer: %d\n", ret);

	ret = mt9m113_write_mcu_var(sensor, width_reg, width_val);
	if (!ret)
		ret = mt9m113_write_mcu_var(sensor, height_reg, height_val);
	if (!ret)
		ret = mt9m113_write_mcu_var(sensor, format_reg, format_val);

	{
		int resume_ret = mt9m113_double_buffer_resume(sensor);

		if (!ret)
			ret = resume_ret;
	}
	if (ret)
		return ret;

	dev_dbg(dev, "MT9M113: Context %c: %ux%u, format=0x%04x\n",
		use_context_b ? 'B' : 'A', width_val, height_val, format_val);

	/*
	 * Re-apply the sensor readout geometry. The MCU variables for
	 * row_end/read_mode can be left stale after switching from Context B to
	 * Context A, producing wrong line counts and a corrupted image.
	 */
	ret = mt9m113_configure_sensor_context(sensor, use_context_b);
	if (ret)
		dev_err(dev, "Failed to configure sensor context: %d\n", ret);

	return ret;
}

/*
 * The MCU resets OUTPUT_CONTROL and MODE_OUTPUT_FORMAT to their defaults during
 * sequencer state transitions, so re-apply them once the stream is running.
 * Best-effort: a failure is logged but does not abort the stream.
 */
static void mt9m113_reassert_output(struct mt9m113 *sensor, u16 output_ctrl_val,
				    u16 format_reg, u32 code)
{
	struct device *dev = &sensor->client->dev;
	u16 format_val = mt9m113_format_val(code);

	if (cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL, output_ctrl_val,
		      NULL))
		dev_warn(dev, "MT9M113: OUTPUT_CONTROL re-write failed\n");
	if (mt9m113_write_mcu_var(sensor, format_reg, format_val))
		dev_warn(dev, "MT9M113: MODE_OUTPUT_FORMAT re-write failed\n");
}

/* Context A (640x480 preview): enable output and run the preview sequencer. */
static int mt9m113_stream_context_a(struct mt9m113 *sensor, u16 output_ctrl_val,
				    u32 code)
{
	struct device *dev = &sensor->client->dev;
	int ret;

	ret = cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL, output_ctrl_val,
			NULL);
	if (ret)
		return ret;
	dev_dbg(dev, "MT9M113: OUTPUT_CONTROL=0x%04x enabled\n", output_ctrl_val);

	ret = cci_write(sensor->regmap, MT9M113_RESET_REGISTER,
			MT9M113_RESET_REG_STREAMING, NULL);
	if (ret)
		return ret;

	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CAP_MODE,
				    MT9M113_SEQ_CAP_MODE_PREVIEW);
	if (ret)
		return ret;
	msleep(40);

	dev_dbg(dev, "MT9M113: Writing SEQ_CMD_RUN\n");
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD, MT9M113_SEQ_CMD_RUN);
	if (ret)
		return ret;

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
	if (ret < 0)
		dev_warn(dev, "MT9M113: SEQ_CMD_RUN did not complete\n");

	mt9m113_reassert_output(sensor, output_ctrl_val,
				MT9M113_MODE_OUTPUT_FORMAT_A, code);
	msleep(20);
	return 0;
}

/*
 * Context B (1280x1024 capture): the MCU only accepts CAPTURE from stable
 * preview, so enter preview first, wait for SEQ_STATE=PREVIEW, then switch.
 */
static int mt9m113_stream_context_b(struct mt9m113 *sensor, u16 output_ctrl_val,
				    u32 code)
{
	struct device *dev = &sensor->client->dev;
	u64 seq_state = 0;
	unsigned int i;
	int ret;

	ret = cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL, output_ctrl_val,
			NULL);
	if (ret)
		return ret;
	dev_dbg(dev, "MT9M113: OUTPUT_CONTROL=0x%04x enabled\n", output_ctrl_val);

	ret = cci_write(sensor->regmap, MT9M113_RESET_REGISTER,
			MT9M113_RESET_REG_STREAMING, NULL);
	if (ret)
		return ret;

	/* Enter preview first. */
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CAP_MODE,
				    MT9M113_SEQ_CAP_MODE_PREVIEW);
	if (ret)
		return ret;
	msleep(40);

	dev_dbg(dev, "MT9M113: Entering preview mode first (SEQ_CMD_RUN)\n");
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD, MT9M113_SEQ_CMD_RUN);
	if (ret)
		return ret;

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
	if (ret < 0)
		dev_warn(dev, "MT9M113: SEQ_CMD_RUN did not complete\n");

	for (i = 0; i < 100; i++) {
		mt9m113_read_mcu_var(sensor, MT9M113_SEQ_STATE, &seq_state);
		if (seq_state == MT9M113_SEQ_STATE_PREVIEW)
			break;
		usleep_range(10000, 11000);
	}
	if (seq_state != MT9M113_SEQ_STATE_PREVIEW)
		dev_warn(dev,
			 "MT9M113: preview state not reached (SEQ_STATE=0x%llx)\n",
			 seq_state);

	/* Switch to video mode (stays in Context B). */
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CAP_MODE,
				    MT9M113_SEQ_CAP_MODE_VIDEO);
	if (ret)
		return ret;
	msleep(40);

	ret = mt9m113_seq_cmd_ready(sensor);
	if (ret < 0) {
		dev_err(dev, "MT9M113: MCU not ready for CAPTURE command\n");
		return ret;
	}

	dev_dbg(dev, "MT9M113: switching to Context B (SEQ_CMD_CAPTURE)\n");
	ret = mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD,
				    MT9M113_SEQ_CMD_CAPTURE);
	if (ret)
		return ret;

	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 500);
	if (ret < 0)
		dev_warn(dev, "MT9M113: SEQ_CMD_CAPTURE did not complete\n");

	for (i = 0; i < 100; i++) {
		mt9m113_read_mcu_var(sensor, MT9M113_SEQ_STATE, &seq_state);
		if (seq_state == MT9M113_SEQ_STATE_CAPTURE)
			break;
		usleep_range(10000, 11000);
	}
	if (seq_state != MT9M113_SEQ_STATE_CAPTURE) {
		dev_err(dev, "MT9M113: Context B switch failed (SEQ_STATE=0x%llx)\n",
			seq_state);
		return -ETIMEDOUT;
	}
	dev_dbg(dev, "MT9M113: reached capture state\n");

	mt9m113_reassert_output(sensor, output_ctrl_val,
				MT9M113_MODE_OUTPUT_FORMAT_B, code);

	/* The sensor pipeline needs time to reconfigure for 1280x1024. */
	msleep(200);
	return 0;
}

/*
 * Apply the MCU-side test-pattern config and freeze the MCU in boot mode
 * so the normal sequencer cannot overwrite it. Per the MT9M113 datasheet
 * R0x0066 (mode_common_mode_settings_test_mode) "Test Pattern Generator.
 * Changes take effect only after REFRESH command", and "Disabling the MCU
 * is recommended before enabling test patterns."
 *
 * The pattern index is written verbatim to the Driver-7 R0x0066 register
 * (encoded as MT9M113_MODE_TEST_MODE = 0x2766) - menu items already match
 * the datasheet's value table 1..5. NOTE: do NOT switch CAM_MODE_SELECT
 * into a "test pattern" state - that puts the camera-mode driver into a
 * mode where the sequencer no longer processes SEQ_CMD, and the REFRESH
 * below hangs.
 *
 * Called from two places:
 *
 *   1. mt9m113_s_ctrl(V4L2_CID_TEST_PATTERN > 0) while streaming - apply
 *      immediately.
 *
 *   2. mt9m113_stream_on(), after stream_context_{a,b}() has brought the
 *      pipeline up - re-apply when the test pattern was selected before
 *      streaming started. __v4l2_ctrl_handler_setup() in stream_on runs
 *      with sensor->streaming still false, so the in-handler halt is
 *      skipped; replaying here makes the "set test pattern, then STREAMON"
 *      sequence work.
 *
 * Caller must hold the same context as mt9m113_s_ctrl() (an active PM
 * reference); we touch MCU vars + BOOT_MODE.
 */
static int mt9m113_apply_test_pattern(struct mt9m113 *sensor, u8 pattern_idx)
{
	int ret;

	ret = mt9m113_write_mcu_var(sensor, MT9M113_MODE_TEST_MODE, pattern_idx);
	if (ret)
		return ret;
	ret = mt9m113_refresh(sensor);
	if (ret)
		return ret;

	dev_dbg(&sensor->client->dev,
		"MT9M113: Halting MCU to lock test pattern %u\n", pattern_idx);
	cci_write(sensor->regmap, MT9M113_MCU_BOOT_MODE, 0x0001, &ret);
	return ret;
}

static int mt9m113_stream_on(struct mt9m113 *sensor,
			     struct v4l2_subdev_state *state)
{
	struct device *dev = &sensor->client->dev;
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *compose;
	u64 health_check, seq_cmd;
	bool use_context_b;
	u16 output_ctrl_val;
	int ret;

	/*
	 * Wake the MCU out of standby and give it time to settle (50ms matches
	 * the legacy vendor driver).
	 */
	dev_dbg(dev, "MT9M113: start_streaming\n");
	cci_write(sensor->regmap, MT9M113_STANDBY_CONTROL,
		  MT9M113_STANDBY_CONTROL_ACTIVE, NULL);
	msleep(50);

	/*
	 * MCU health check: MODE_OUTPUT_WIDTH_A reads back non-zero once the
	 * MCU is running. If it is unresponsive, re-apply the init table.
	 */
	ret = mt9m113_read_mcu_var(sensor, MT9M113_MODE_OUTPUT_WIDTH_A,
				   &health_check);
	if (ret < 0 || health_check == 0) {
		dev_warn(dev, "MT9M113: MCU unresponsive, re-init\n");
		ret = mt9m113_sensor_init(sensor);
		if (ret < 0)
			return ret;
	}

	/*
	 * Clear stale SEQ_CMD. The init table ends with REFRESH (SEQ_CMD=0x05)
	 * which may not complete before the sensor enters standby/suspend. On
	 * resume, SEQ_CMD is stuck at 0x05 and all subsequent commands fail.
	 * Force-clear it.
	 */
	ret = mt9m113_read_mcu_var(sensor, MT9M113_SEQ_CMD, &seq_cmd);
	if (ret == 0 && seq_cmd != 0) {
		dev_warn(dev, "MT9M113: Clearing stale SEQ_CMD=0x%llx\n", seq_cmd);
		mt9m113_write_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000);
		msleep(50);
	}

	/*
	 * Re-assert the MIPI Frame-Start/End short packets on every stream
	 * start. The MCU clears CUSTOM_SHORT_PKT across standby/refresh
	 * cycles, and mt9m113_sensor_init() (which also sets it) only runs on
	 * the MCU-recovery path - so without this the bit is lost after the
	 * first stream. Without the per-frame boundary marker the VFE CAMIF
	 * falls back to line-counting and the image walks vertically.
	 */
	ret = cci_write(sensor->regmap, MT9M113_CUSTOM_SHORT_PKT,
			MT9M113_CUSTOM_SHORT_PKT_FRAME_CNT_EN, NULL);
	if (ret < 0)
		return ret;

	/* Apply all V4L2 controls (color effects, etc.) before streaming */
	ret = __v4l2_ctrl_handler_setup(&sensor->ifp.hdl);
	if (ret) {
		dev_err(dev, "Failed to setup controls: %d\n", ret);
		return ret;
	}

	compose = v4l2_subdev_state_get_compose(state, 0);
	/* Source pad (pad 1) carries the MIPI output format. */
	format = v4l2_subdev_state_get_format(state, 1);

	use_context_b = (compose->width > 640 || compose->height > 480);
	dev_dbg(dev, "MT9M113: %ux%u -> Context %c\n",
		compose->width, compose->height, use_context_b ? 'B' : 'A');

	/*
	 * Configure AE for the selected context: Context B (capture) allows a
	 * longer exposure for quality, Context A (preview) optimises for frame
	 * rate.
	 */
	ret = mt9m113_configure_ae_mode(sensor, use_context_b);
	if (ret) {
		dev_err(dev, "Failed to configure AE mode: %d\n", ret);
		return ret;
	}

	ret = mt9m113_configure_output(sensor, use_context_b, format->code);
	if (ret)
		return ret;

	/* Wait for CSIPHY stabilization */
	msleep(MT9M113_PRE_MIPI_DELAY_MS);

	/*
	 * YUV/RGB output: enable MIPI CSI-2 in FIFO-bypass mode and restore the
	 * default ISP colour pipeline.
	 */
	output_ctrl_val = MT9M113_OUTPUT_CONTROL_MIPI_ENABLE;
	ret = cci_write(sensor->regmap, MT9M113_OFIFO_CONTROL_STATUS,
			MT9M113_OFIFO_BYPASS, NULL);
	if (ret)
		return ret;
	ret = cci_write(sensor->regmap, MT9M113_COLOR_PIPELINE_CONTROL,
			MT9M113_COLOR_PIPELINE_DEFAULT, NULL);
	if (ret)
		return ret;

	/* The MCU must be idle before it will accept a sequencer command. */
	ret = mt9m113_seq_cmd_ready(sensor);
	if (ret < 0) {
		dev_err(dev, "MT9M113: MCU not ready for streaming command\n");
		return ret;
	}

	/*
	 * One REFRESH applies the per-session output dimensions/format; without
	 * it SEQ_CMD_RUN hangs (the MCU never clears SEQ_CMD). OFIFO and the
	 * colour pipeline are already programmed above - they must not be
	 * written while a REFRESH is pending or it times out.
	 */
	ret = mt9m113_refresh(sensor);
	if (ret)
		dev_warn(dev, "MT9M113: REFRESH failed, continuing\n");

	/*
	 * For Context B the MCU must reach stable preview (SEQ_STATE=0x04)
	 * before it will accept the CAPTURE command, so enter preview first and
	 * then switch. Context A simply runs preview.
	 */
	if (use_context_b)
		ret = mt9m113_stream_context_b(sensor, output_ctrl_val,
					       format->code);
	else
		ret = mt9m113_stream_context_a(sensor, output_ctrl_val,
					       format->code);
	if (ret)
		return ret;

	/*
	 * If V4L2_CID_TEST_PATTERN was set to a non-zero value before the
	 * stream started, mt9m113_s_ctrl() noted it on test_pattern_{active,
	 * value} but couldn't commit the MCU writes (the normal stream init
	 * below would have overwritten MODE_SELECT). Replay the apply now
	 * that the pipeline is up so STREAMON ends with the test pattern
	 * visible. Read the cached value directly - do NOT call
	 * v4l2_ctrl_find() here, the ctrl-handler mutex is already held by
	 * the V4L2 core for the duration of this s_stream callback, so a
	 * lookup would self-deadlock.
	 */
	if (sensor->test_pattern_active && sensor->test_pattern_value > 0) {
		ret = mt9m113_apply_test_pattern(sensor,
						 sensor->test_pattern_value);
		if (ret)
			return ret;
	}

	dev_dbg(dev, "MT9M113: streaming started\n");
	return 0;
}

/*
 * The MT9M113 MCU intermittently wedges on stream start; the failures cluster
 * at the start of a run and then clear. Retry a bounded number of times, fully
 * power-cycling the sensor (runtime-PM suspend -> resume = power_off + power_on
 * + sensor_init) between attempts. Exactly one pm_runtime get/put pair runs per
 * attempt; on success the reference is held for the streaming session and
 * released by mt9m113_stop_streaming().
 */
static int mt9m113_start_streaming(struct mt9m113 *sensor,
				   struct v4l2_subdev_state *state)
{
	struct device *dev = &sensor->client->dev;
	unsigned int attempt;
	int ret = 0;

	/*
	 * Guard against a redundant s_stream(1): a second
	 * pm_runtime_resume_and_get() here would leak a PM reference and pin
	 * the device awake forever, because mt9m113_stop_streaming() only
	 * drops one reference per stop.
	 */
	if (sensor->streaming)
		return 0;

	for (attempt = 0; attempt < MT9M113_STREAM_START_RETRIES; attempt++) {
		ret = pm_runtime_resume_and_get(dev);
		if (ret)
			return ret;

		ret = mt9m113_stream_on(sensor, state);
		if (!ret) {
			sensor->streaming = true;
			return 0;
		}

		pm_runtime_put_sync_suspend(dev);
		if (attempt + 1 < MT9M113_STREAM_START_RETRIES)
			dev_warn(dev,
				 "MT9M113: stream start failed (%d), power-cycle + retry %u/%u\n",
				 ret, attempt + 1, MT9M113_STREAM_START_RETRIES - 1);
	}

	return ret;
}

static int mt9m113_stop_streaming(struct mt9m113 *sensor)
{
	struct device *dev = &sensor->client->dev;
	int ret;

	if (!sensor->streaming)
		return 0;

	sensor->streaming = false;

	/* Disable MIPI output */
	ret = cci_write(sensor->regmap, MT9M113_OUTPUT_CONTROL, 0x0000, NULL);
	if (ret < 0)
		dev_warn(dev,
			 "MT9M113: MIPI output disable failed (%d), receiver may still see traffic\n",
			 ret);

	/*
	 * Wait briefly for any pending SEQ_CMD to complete.
	 * Don't issue REFRESH here - it can leave MCU stuck if it times out.
	 * Start_streaming will handle full re-initialization.
	 */
	ret = mt9m113_poll_mcu_var(sensor, MT9M113_SEQ_CMD, 0x0000, 100);
	if (ret < 0)
		dev_dbg(dev, "MT9M113: SEQ_CMD did not complete before stop\n");

	dev_dbg(dev, "MT9M113: streaming stopped\n");

	pm_runtime_put_autosuspend(dev);
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

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
	if (mt9m113_format_info(fse->code)->code != fse->code)
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

static int mt9m113_ifp_enum_frame_interval(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *state,
					   struct v4l2_subdev_frame_interval_enum *fie)
{
	/* Only source pad (pad 1) supports frame interval enumeration */
	if (fie->pad != 1)
		return -EINVAL;

	/* One interval per resolution */
	if (fie->index > 0)
		return -EINVAL;

	/* Context A: 640x480 @ 30fps, Context B: 1280x1024 @ 15fps */
	if (fie->width == 640 && fie->height == 480) {
		fie->interval.numerator = 1;
		fie->interval.denominator = 30;
	} else if (fie->width == 1280 && fie->height == 1024) {
		fie->interval.numerator = 1;
		fie->interval.denominator = 15;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int mt9m113_ifp_g_frame_interval(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_frame_interval *fi)
{
	struct v4l2_mbus_framefmt *format;

	/* Frame intervals are only defined on the source pad. */
	if (fi->pad != 1)
		return -EINVAL;

	format = v4l2_subdev_state_get_format(state, 1);

	/* Return interval based on current resolution */
	if (format->width <= 640 && format->height <= 480) {
		/* Context A: 30fps */
		fi->interval.numerator = 1;
		fi->interval.denominator = 30;
	} else {
		/* Context B: 15fps */
		fi->interval.numerator = 1;
		fi->interval.denominator = 15;
	}

	return 0;
}

static int mt9m113_ifp_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *fmt)
{
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *compose;
	const struct mt9m113_format_info *info;

	/*
	 * Reject geometry/format changes while the pipeline is live.
	 * The sensor is only programmed in mt9m113_stream_on(); silently
	 * updating the active state would let userspace see one format while
	 * the wire still carries the previous one.
	 */
	if (sensor->streaming)
		return -EBUSY;

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
	.enum_frame_interval = mt9m113_ifp_enum_frame_interval,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mt9m113_ifp_set_fmt,
	.get_frame_interval = mt9m113_ifp_g_frame_interval,
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

/*
 * Menu items map directly to MT9M113_TEST_MODE_* values in datasheet order
 * (MT9M113 datasheet R0x0066 "Test Pattern Generator"). The driver writes
 * the menu index unchanged to MT9M113_MODE_TEST_MODE; no value translation
 * is needed.
 */
static const char * const mt9m113_test_pattern_menu[] = {
	"Disabled",
	"Solid White",
	"Grey Ramp",
	"Color Bar Ramp",
	"Solid White (Color Bars)",
	"Noise",
};

static int mt9m113_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m113 *sensor = container_of(ctrl->handler,
					       struct mt9m113, ifp.hdl);
	int pm_ret, ret = 0;

	/*
	 * pm_runtime_get_if_in_use() returns >0 on success (HW active, kref
	 * incremented), 0 if the device is suspended, and -EINVAL when
	 * CONFIG_PM is disabled. Treat -EINVAL as "PM core is absent, HW is
	 * always on" and proceed without taking a kref (the matching put at
	 * the tail must then be skipped). Only 0 means "skip silently".
	 */
	pm_ret = pm_runtime_get_if_in_use(&sensor->client->dev);
	if (pm_ret == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP: {
		u64 mode_a, mode_b;

		ret = mt9m113_read_mcu_var(sensor, MT9M113_SENSOR_READ_MODE_A,
					   &mode_a);
		if (ret)
			break;
		ret = mt9m113_read_mcu_var(sensor, MT9M113_SENSOR_READ_MODE_B,
					   &mode_b);
		if (ret)
			break;

		if (ctrl->val) {
			mode_a |= MT9M113_SENSOR_READ_MODE_HMIRROR;
			mode_b |= MT9M113_SENSOR_READ_MODE_HMIRROR;
		} else {
			mode_a &= ~MT9M113_SENSOR_READ_MODE_HMIRROR;
			mode_b &= ~MT9M113_SENSOR_READ_MODE_HMIRROR;
		}

		ret = mt9m113_write_mcu_var(sensor, MT9M113_SENSOR_READ_MODE_A,
					    mode_a);
		if (!ret)
			ret = mt9m113_write_mcu_var(sensor,
						    MT9M113_SENSOR_READ_MODE_B,
						    mode_b);
		/*
		 * Only refresh if streaming and the MCU is not held in boot
		 * mode for the test pattern - calling REFRESH while the MCU
		 * is halted (test_pattern_active) blocks for two 500 ms
		 * SEQ_CMD timeouts and never applies the new control.
		 */
		if (!ret && sensor->streaming && !sensor->test_pattern_active)
			mt9m113_refresh(sensor);
		break;
	}

	case V4L2_CID_VFLIP: {
		u64 mode_a, mode_b;

		ret = mt9m113_read_mcu_var(sensor, MT9M113_SENSOR_READ_MODE_A,
					   &mode_a);
		if (ret)
			break;
		ret = mt9m113_read_mcu_var(sensor, MT9M113_SENSOR_READ_MODE_B,
					   &mode_b);
		if (ret)
			break;

		if (ctrl->val) {
			mode_a |= MT9M113_SENSOR_READ_MODE_VMIRROR;
			mode_b |= MT9M113_SENSOR_READ_MODE_VMIRROR;
		} else {
			mode_a &= ~MT9M113_SENSOR_READ_MODE_VMIRROR;
			mode_b &= ~MT9M113_SENSOR_READ_MODE_VMIRROR;
		}

		ret = mt9m113_write_mcu_var(sensor, MT9M113_SENSOR_READ_MODE_A,
					    mode_a);
		if (!ret)
			ret = mt9m113_write_mcu_var(sensor,
						    MT9M113_SENSOR_READ_MODE_B,
						    mode_b);
		/*
		 * Only refresh if streaming and the MCU is not held in boot
		 * mode for the test pattern - calling REFRESH while the MCU
		 * is halted (test_pattern_active) blocks for two 500 ms
		 * SEQ_CMD timeouts and never applies the new control.
		 */
		if (!ret && sensor->streaming && !sensor->test_pattern_active)
			mt9m113_refresh(sensor);
		break;
	}

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
			ret = mt9m113_write_mcu_var(sensor,
						    MT9M113_MODE_SPEC_EFFECTS_A,
						    effect);
			if (!ret)
				ret = mt9m113_write_mcu_var(sensor,
							    MT9M113_MODE_SPEC_EFFECTS_B,
							    effect);

			/* See HFLIP/VFLIP: skip refresh while test_pattern halts MCU. */
			if (!ret && sensor->streaming && !sensor->test_pattern_active)
				mt9m113_refresh(sensor);
		}
		break;
	}

	case V4L2_CID_POWER_LINE_FREQUENCY: {
		u8 fd_mode;

		switch (ctrl->val) {
		case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
			fd_mode = MT9M113_FD_MODE_DISABLED;
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
			fd_mode = MT9M113_FD_MODE_50HZ;
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
			fd_mode = MT9M113_FD_MODE_60HZ;
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
			fd_mode = MT9M113_FD_MODE_AUTO;
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (!ret)
			ret = mt9m113_write_mcu_var(sensor, MT9M113_FD_MODE,
						    fd_mode);
		break;
	}

	case V4L2_CID_SATURATION:
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AWB_SATURATION,
					    ctrl->val);
		/*
		 * Only refresh if streaming and the MCU is not held in boot
		 * mode for the test pattern - calling REFRESH while the MCU
		 * is halted (test_pattern_active) blocks for two 500 ms
		 * SEQ_CMD timeouts and never applies the new control.
		 */
		if (!ret && sensor->streaming && !sensor->test_pattern_active)
			mt9m113_refresh(sensor);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		/* Enable/disable internal AE algorithm via AE_GATE */
		if (ctrl->val == V4L2_EXPOSURE_AUTO)
			ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_GATE,
						    MT9M113_AE_GATE_ENABLE);
		else
			ret = mt9m113_write_mcu_var(sensor, MT9M113_AE_GATE,
						    MT9M113_AE_GATE_DISABLE);
		break;

	case V4L2_CID_EXPOSURE:
		/* Write coarse integration time to both contexts */
		ret = cci_write(sensor->regmap, MT9M113_COARSE_IT_TIME_A,
				ctrl->val, NULL);
		if (!ret)
			ret = cci_write(sensor->regmap, MT9M113_COARSE_IT_TIME_B,
					ctrl->val, NULL);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/* Write analog gain register */
		ret = cci_write(sensor->regmap, MT9M113_ANALOG_GAIN,
				ctrl->val, NULL);
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		/* Enable/disable internal AWB via awb_mode */
		ret = mt9m113_write_mcu_var(sensor, MT9M113_AWB_MODE,
					    ctrl->val ? 0x02 : 0x00);
		break;

	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == 0) {
			/*
			 * Disable test pattern. Only restart MCU if test
			 * pattern was previously active. On first call
			 * (ctrl_handler_setup default), the MCU is already
			 * in normal mode - restarting it here would disrupt
			 * the MCU state and cause REFRESH timeouts.
			 *
			 * Keep test_pattern_active set until both writes
			 * succeed: a half-applied disable leaves the MCU
			 * halted, and a subsequent CID=0 must be allowed to
			 * retry the full sequence.
			 */
			if (sensor->test_pattern_active) {
				dev_dbg(&sensor->client->dev,
					 "MT9M113: Disabling test pattern, restarting MCU\n");
				/*
				 * Reverse of mt9m113_apply_test_pattern(): release
				 * the MCU from halt, write MODE_TEST_MODE=0, and
				 * REFRESH if streaming so the sequencer reverts to
				 * the normal sensor stream and frames resume.
				 */
				cci_write(sensor->regmap, MT9M113_MCU_BOOT_MODE,
					  0x0000, &ret);
				if (!ret) {
					usleep_range(10000, 15000);
					ret = mt9m113_write_mcu_var(sensor,
								    MT9M113_MODE_TEST_MODE,
								    MT9M113_TEST_MODE_DISABLED);
				}
				if (!ret && sensor->streaming)
					ret = mt9m113_refresh(sensor);
				if (!ret) {
					sensor->test_pattern_active = false;
					sensor->test_pattern_value = 0;
				}
			}
		} else {
			/*
			 * Enable test pattern mode.
			 *
			 * Only commit the MCU vars + halt here while we're
			 * already streaming. Otherwise the MCU is asleep
			 * (autosuspend) or not yet running the sequencer,
			 * and either the writes would race a power_on or
			 * the subsequent normal stream init would overwrite
			 * MODE_SELECT. Cache the requested pattern in
			 * test_pattern_value so mt9m113_stream_on() can
			 * replay the apply at the end of its normal-init
			 * sequence without doing a v4l2_ctrl_find() lookup
			 * (which would self-deadlock on the ctrl-handler
			 * mutex that the V4L2 core holds across s_stream).
			 */
			sensor->test_pattern_active = true;
			sensor->test_pattern_value = ctrl->val;
			dev_dbg(&sensor->client->dev,
				 "MT9M113: Enabling test pattern %d\n", ctrl->val);
			if (sensor->streaming)
				ret = mt9m113_apply_test_pattern(sensor,
								 ctrl->val);
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (pm_ret > 0)
		pm_runtime_put_autosuspend(&sensor->client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops mt9m113_ctrl_ops = {
	.s_ctrl = mt9m113_s_ctrl,
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

	/*
	 * Aptina/onsemi SOC sensors require EXTCLK to be running before
	 * STANDBY (powerdown) is released; otherwise the internal
	 * clock-domain state machines come out of reset non-deterministically
	 * and the MCU can wedge on SEQ_CMD.  Enable the clock first, let the
	 * supplies and clock settle, then deassert powerdown.
	 */
	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto error_regulator;

	usleep_range(20000, 25000);

	if (sensor->powerdown)
		gpiod_set_value_cansleep(sensor->powerdown, 0);

	msleep(20);

	if (sensor->reset) {
		gpiod_set_value_cansleep(sensor->reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(sensor->reset, 0);
		usleep_range(44500, 50000);
	} else if (sensor->powerdown) {
		usleep_range(44500, 50000);
	}

	/* MT9M113 MCU boot sequence */
	{
		u64 clocks_val = 0;

		cci_read(sensor->regmap, MT9M113_CLOCKS_CONTROL, &clocks_val, NULL);
		if (clocks_val != 0) {
			u64 seq_cmd = 0;
			int read_ret;

			/*
			 * Sensor already has clocks running (warm reboot or
			 * resume). Check if MCU is responsive by reading
			 * SEQ_CMD. If stuck (non-zero) or if the read itself
			 * failed (I2C error), force the soft-reset path instead
			 * of silently returning success on an uninitialised
			 * sensor.
			 */
			read_ret = mt9m113_read_mcu_var(sensor, MT9M113_SEQ_CMD,
							&seq_cmd);
			if (read_ret == 0 && seq_cmd == 0) {
				dev_dbg(dev, "MT9M113 already initialized, MCU OK\n");
				msleep(50);
				return 0;
			}
			if (read_ret < 0)
				dev_warn(dev,
					 "MT9M113: SEQ_CMD read failed (%d), forcing soft reset\n",
					 read_ret);
			else
				dev_warn(dev,
					 "MT9M113: MCU stuck (SEQ_CMD=0x%llx), forcing soft reset\n",
					 seq_cmd);
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
		cci_write(sensor->regmap, MT9M113_STANDBY_CONTROL,
			  MT9M113_STANDBY_CONTROL_ACTIVE, &ret);
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
		/* Keep the MCU active (do not enter standby 0x002A) after power-on. */
		cci_write(sensor->regmap, MT9M113_STANDBY_CONTROL,
			  MT9M113_STANDBY_CONTROL_ACTIVE, &ret);
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
	if (sensor->powerdown)
		gpiod_set_value_cansleep(sensor->powerdown, 1);
	else if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
			       sensor->supplies);
	return ret;
}

static void mt9m113_power_off(struct mt9m113 *sensor)
{
	/*
	 * Symmetric with mt9m113_power_on(): hold the chip in reset, then
	 * gate the clock and drop the supplies so the enable/disable refcount
	 * stays balanced across runtime-PM cycles and the MCU is fully reset.
	 *
	 * Prefer the powerdown GPIO when available; fall back to driving
	 * RESET_BAR active so it is not left floating after VDD drops on
	 * boards that only wire reset-gpios.  Mirror the settling gaps from
	 * power_on so the MCU sees a clean down-then-up sequence across
	 * runtime-PM cycles.
	 */
	if (sensor->powerdown)
		gpiod_set_value_cansleep(sensor->powerdown, 1);
	else if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);
	usleep_range(1000, 2000);
	clk_disable_unprepare(sensor->clk);
	usleep_range(1000, 2000);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
			       sensor->supplies);
}

static int mt9m113_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);
	int ret;

	/*
	 * Always power-cycle and re-initialise on resume, including when a
	 * powerdown GPIO is present. The MT9M113 MCU must be hardware-reset
	 * (powerdown + clock) and re-initialised on each session, otherwise a
	 * wedged MCU (SEQ_CMD/0xA103 stuck) is never recovered and only a
	 * physical power cycle helps - matching the legacy vendor kernel per-open behaviour.
	 */
	ret = mt9m113_power_on(sensor);
	if (ret)
		return ret;

	ret = mt9m113_sensor_init(sensor);
	if (ret) {
		/*
		 * sensor_init() can fail (-ETIMEDOUT on an MCU lockup, I2C
		 * errors); on that path the PM core reverts runtime_status to
		 * SUSPENDED without touching driver-side state, so the
		 * clk/regulator refcounts taken in power_on() would otherwise
		 * leak and the next resume would double-enable them.
		 */
		mt9m113_power_off(sensor);
		return ret;
	}

	return 0;
}

static int mt9m113_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m113 *sensor = ifp_to_mt9m113(sd);

	/* Power down (powerdown GPIO + clock off) so the MCU is reset next resume. */
	mt9m113_power_off(sensor);
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(mt9m113_pm_ops,
				 mt9m113_runtime_suspend,
				 mt9m113_runtime_resume,
				 NULL);

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

	dev_dbg(&sensor->client->dev, "MT9M113 detected (ID 0x%04llx)\n",
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
		dev_err(&sensor->client->dev, "Unsupported bus type %d\n",
			sensor->bus_cfg.bus_type);
		v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
		return -EINVAL;
	}

	if (sensor->bus_cfg.nr_of_link_frequencies < 1) {
		dev_err(&sensor->client->dev, "no link-frequencies specified\n");
		v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
		return -EINVAL;
	}

	/*
	 * The MT9M113 has a single-lane MIPI CSI-2 D-PHY interface, so the
	 * binding no longer advertises data-lanes (a maxItems=1 array adds no
	 * information). v4l2_fwnode_endpoint_parse() therefore leaves
	 * num_data_lanes at 0; force it to 1 so the pixrate calculation
	 * below has a non-zero multiplier and receivers like camss size the
	 * CSI-2 link budget correctly.
	 */
	if (sensor->bus_cfg.bus.mipi_csi2.num_data_lanes == 0)
		sensor->bus_cfg.bus.mipi_csi2.num_data_lanes = 1;

	if (sensor->bus_cfg.bus.mipi_csi2.num_data_lanes != 1) {
		dev_err(&sensor->client->dev,
			"unsupported data-lanes count %u (must be 1)\n",
			sensor->bus_cfg.bus.mipi_csi2.num_data_lanes);
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

	/*
	 * The CSI-2 link frequency is mandatory (enforced by the binding and
	 * mt9m113_parse_dt()). Derive the pixel rate from it: every supported
	 * source format is 16 bits/pixel on the bus and the D-PHY clocks data
	 * DDR, so pixel_rate = link_freq * 2 * lanes / bpp.
	 */
	sensor->link_freq = sensor->bus_cfg.link_frequencies[0];
	sensor->pixrate = div_u64(2ULL * sensor->link_freq *
				  sensor->bus_cfg.bus.mipi_csi2.num_data_lanes,
				  16);

	dev_dbg(dev, "MT9M113: link freq %lld Hz, pixel rate %u Hz\n",
		sensor->link_freq, sensor->pixrate);

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
		goto error_pa_entity;

	/*
	 * LINK_FREQ and PIXEL_RATE live on the pixel-array subdev because
	 * receivers walk the media graph upstream until they reach the
	 * MEDIA_ENT_F_CAM_SENSOR entity and read these controls from that
	 * subdev's handler (see camss_find_sensor_pad() / v4l2_get_link_freq()
	 * in drivers/media/platform/qcom/camss/camss.c). The IFP is a
	 * MEDIA_ENT_F_PROC_VIDEO_ISP and the walk does not stop there, so
	 * controls on the IFP alone are invisible to the receiver and the
	 * pipeline fails at CSIPHY stream-on with -EINVAL.
	 */
	v4l2_ctrl_handler_init(&sensor->pa.hdl, 2);
	{
		struct v4l2_ctrl *link_freq_ctrl;

		link_freq_ctrl = v4l2_ctrl_new_int_menu(&sensor->pa.hdl, NULL,
				V4L2_CID_LINK_FREQ,
				sensor->bus_cfg.nr_of_link_frequencies - 1, 0,
				sensor->bus_cfg.link_frequencies);
		if (link_freq_ctrl)
			link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}
	v4l2_ctrl_new_std(&sensor->pa.hdl, NULL, V4L2_CID_PIXEL_RATE,
			  sensor->pixrate, sensor->pixrate, 1, sensor->pixrate);
	if (sensor->pa.hdl.error) {
		ret = sensor->pa.hdl.error;
		goto error_pa_handler;
	}
	sensor->pa.sd.ctrl_handler = &sensor->pa.hdl;
	sensor->pa.sd.state_lock = sensor->pa.hdl.lock;

	ret = v4l2_subdev_init_finalize(&sensor->pa.sd);
	if (ret < 0)
		goto error_pa_handler;

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
		goto error_ifp_entity;

	/* Initialize controls on IFP */
	v4l2_ctrl_handler_init(&sensor->ifp.hdl, 10);
	v4l2_ctrl_new_std(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	/* COLORFX: mask out unsupported effects 4-12 (EMBOSS through SILHOUETTE) */
	v4l2_ctrl_new_std_menu(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			       V4L2_CID_COLORFX,
			       V4L2_COLORFX_SOLARIZATION, 0x1ff0,
			       V4L2_COLORFX_NONE);
	v4l2_ctrl_new_std_menu(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			       V4L2_CID_POWER_LINE_FREQUENCY,
			       V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
			       V4L2_CID_POWER_LINE_FREQUENCY_AUTO);
	v4l2_ctrl_new_std(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std_menu(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO,
			       V4L2_EXPOSURE_MANUAL, 0,
			       V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			  V4L2_CID_EXPOSURE, 1, 1000, 1, 100);
	v4l2_ctrl_new_std(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 127, 1, 32);
	v4l2_ctrl_new_std(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	v4l2_ctrl_new_std_menu_items(&sensor->ifp.hdl, &mt9m113_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mt9m113_test_pattern_menu) - 1,
				     0, 0, mt9m113_test_pattern_menu);

	if (sensor->ifp.hdl.error) {
		ret = sensor->ifp.hdl.error;
		goto error_ifp_handler;
	}

	sensor->ifp.sd.ctrl_handler = &sensor->ifp.hdl;
	sensor->ifp.sd.state_lock = sensor->ifp.hdl.lock;

	ret = v4l2_subdev_init_finalize(&sensor->ifp.sd);
	if (ret < 0)
		goto error_ifp_handler;

	/*
	 * Enable runtime PM.  Configure autosuspend before enabling the
	 * runtime PM core so a put issued between enable and use_autosuspend
	 * cannot race with autosuspend being unconfigured.  The 2 s autosuspend
	 * delay amortises the unconditional power-cycle + full init-table
	 * replay in runtime_resume over typical preview/snapshot sequences:
	 * the cold init is load-bearing for MCU-wedge recovery so it cannot
	 * be conditionalised on health, only amortised.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	/* Register only the IFP - PA will be registered in ifp_registered callback */
	ret = v4l2_async_register_subdev(&sensor->ifp.sd);
	if (ret < 0)
		goto error_pm;

	pm_runtime_put_autosuspend(dev);

	dev_dbg(dev, "MT9M113 driver with IFP sub-device initialized\n");
	return 0;

error_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
error_ifp_handler:
	v4l2_subdev_cleanup(&sensor->ifp.sd);
	v4l2_ctrl_handler_free(&sensor->ifp.hdl);
error_ifp_entity:
	media_entity_cleanup(&sensor->ifp.sd.entity);
	v4l2_subdev_cleanup(&sensor->pa.sd);
error_pa_handler:
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
	pm_runtime_dont_use_autosuspend(dev);
	if (!pm_runtime_status_suspended(dev))
		mt9m113_power_off(sensor);
	pm_runtime_set_suspended(dev);
}

static const struct of_device_id mt9m113_of_ids[] = {
	{ .compatible = "aptina,mt9m113" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt9m113_of_ids);

static const struct i2c_device_id mt9m113_id[] = {
	{ "mt9m113" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, mt9m113_id);

static struct i2c_driver mt9m113_driver = {
	.driver = {
		.name	= "mt9m113",
		.pm	= pm_ptr(&mt9m113_pm_ops),
		.of_match_table = mt9m113_of_ids,
	},
	.probe		= mt9m113_probe,
	.remove		= mt9m113_remove,
	.id_table	= mt9m113_id,
};

module_i2c_driver(mt9m113_driver);

MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_DESCRIPTION("Aptina MT9M113 Sensor Driver");
MODULE_LICENSE("GPL");
