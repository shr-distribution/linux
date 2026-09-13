/*
 * imx219mipiraw_Sensor.c — IMX219 MIPI RAW sensor driver (Titan).
 *
 * C rewrite of the stock Titan driver, which was carried in this tree as
 * disassembled imx219mipiraw_Sensor.S + imx219mipiraw_data.S. Structure
 * follows the sibling MediaTek imgsensor drivers; every register value,
 * timing constant and state transition matches the stock driver. Places
 * where the stock driver differs from what the sibling drivers do — some of
 * them plain bugs that the sensor depends on by now — are marked TITAN.
 *
 * The stock build folded imgsensor_info and the imgsensor state struct into
 * separate statics, dropping the fields nothing reads and narrowing pclk and
 * line_length to flags because all five modes share one value. The statics
 * below carry the real values instead; the driver programs the same bytes.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx219mipiraw_Sensor.h"

#define PFX "imx219_camera_sensor"

#define LOG_INF(fmt, args...) pr_debug(PFX "[%s] " fmt, __func__, ##args)
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")

static DEFINE_SPINLOCK(imgsensor_drv_lock);

/* TITAN: every mode runs at the same pixel clock and line length (2 lanes). */
#define TITAN_PCLK		172800000
#define TITAN_LINE_LENGTH	3448

/* Frame lengths per mode; preview, video and the custom modes share one. */
#define TITAN_FL_PREVIEW	1332
#define TITAN_FL_CAPTURE	2544

#define TITAN_MARGIN		5
#define TITAN_MIN_SHUTTER	2
#define TITAN_MAX_FRAME_LENGTH	0xFFFF

static kal_uint32 imgsensor_shutter = 0x3D0;
static kal_uint16 imgsensor_current_fps = 300;
static kal_uint8 imgsensor_i2c_write_id = 0x20;
static kal_uint16 iPreGain;		/* cleared by open(), never updated */
static kal_bool imgsensor_test_pattern;
static kal_uint32 imgsensor_pclk;
static kal_uint32 imgsensor_frame_length;
static kal_uint32 imgsensor_line_length;
static kal_uint32 imgsensor_min_frame_length;
static kal_uint16 imgsensor_dummy_line;
static kal_bool imgsensor_autoflicker_en;
static enum MSDK_SCENARIO_ID_ENUM current_scenario_id;

/* i2c_addr_table: both slave addresses map to the same 7-bit address. */
static const kal_uint8 imgsensor_i2c_addr_table[] = {0x21, 0x20, 0xFF};

static const struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	/* preview */
	{3280, 2464, 0, 0, 3280, 2464, 1640, 1232,
	 0, 0, 1640, 1232, 2, 2, 1632, 1224},
	/* capture */
	{3280, 2464, 0, 0, 3280, 2464, 3280, 2464,
	 0, 0, 3280, 2464, 4, 4, 3264, 2448},
	/* normal video */
	{3280, 2464, 0, 0, 3280, 2464, 1640, 1232,
	 0, 0, 1640, 1232, 2, 2, 1632, 1224},
	/* high speed video */
	{3280, 2464, 0, 0, 3280, 2464, 3280, 2464,
	 0, 0, 3280, 2464, 4, 4, 3264, 2448},
	/* slim video */
	{3280, 2464, 0, 0, 3280, 2464, 1640, 926,
	 0, 0, 1640, 926, 0, 0, 1280, 720},
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {
		(char)(addr >> 8), (char)(addr & 0xFF)
	};

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor_i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)
	};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor_i2c_write_id);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor_dummy_line, 0);

	write_cmos_sensor(0x0160, (imgsensor_frame_length >> 8) & 0xFF);
	write_cmos_sensor(0x0161, imgsensor_frame_length & 0xFF);
	write_cmos_sensor(0x0162, (imgsensor_line_length >> 8) & 0xFF);
	write_cmos_sensor(0x0163, imgsensor_line_length & 0xFF);
}

static void set_max_framerate(kal_uint16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length;

	LOG_INF("framerate = %d, min framelength should enable = %d\n",
		framerate, min_framelength_en);

	frame_length = imgsensor_pclk / framerate * 10 / imgsensor_line_length;

	spin_lock(&imgsensor_drv_lock);
	if (frame_length > imgsensor_min_frame_length)
		imgsensor_frame_length = frame_length;
	else
		imgsensor_frame_length = imgsensor_min_frame_length;
	imgsensor_dummy_line =
		imgsensor_frame_length - imgsensor_min_frame_length;
	if (imgsensor_frame_length > TITAN_MAX_FRAME_LENGTH) {
		imgsensor_frame_length = TITAN_MAX_FRAME_LENGTH;
		imgsensor_dummy_line =
			imgsensor_frame_length - imgsensor_min_frame_length;
	}
	if (min_framelength_en)
		imgsensor_min_frame_length = imgsensor_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}

/* TITAN: the stock driver only ever uses the low 16 bits of the shutter. */
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor_shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* if shutter bigger than frame_length, extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor_min_frame_length - TITAN_MARGIN)
		imgsensor_frame_length = shutter + TITAN_MARGIN;
	else
		imgsensor_frame_length = imgsensor_min_frame_length;
	if (imgsensor_frame_length > TITAN_MAX_FRAME_LENGTH)
		imgsensor_frame_length = TITAN_MAX_FRAME_LENGTH;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < TITAN_MIN_SHUTTER) ? TITAN_MIN_SHUTTER : shutter;
	if (shutter > TITAN_MAX_FRAME_LENGTH - TITAN_MARGIN)
		shutter = TITAN_MAX_FRAME_LENGTH - TITAN_MARGIN;

	if (imgsensor_autoflicker_en) {
		realtime_fps = imgsensor_pclk / imgsensor_line_length * 10
			/ imgsensor_frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			write_cmos_sensor(0x0160,
					  (imgsensor_frame_length >> 8) & 0xFF);
			write_cmos_sensor(0x0161,
					  imgsensor_frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor(0x0160, (imgsensor_frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x0161, imgsensor_frame_length & 0xFF);
	}

	write_cmos_sensor(0x015A, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x015B, shutter & 0xFF);

	LOG_INF("Exit! shutter =%d, framelength =%d\n",
		shutter, imgsensor_frame_length);
}

/* Set the shutter and, when frame_length is non-trivial, the frame time. */
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor_shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (frame_length > 1)
		imgsensor_frame_length = frame_length;
	if (imgsensor_frame_length - TITAN_MARGIN < shutter)
		imgsensor_frame_length = shutter + TITAN_MARGIN;
	if (imgsensor_frame_length > TITAN_MAX_FRAME_LENGTH)
		imgsensor_frame_length = TITAN_MAX_FRAME_LENGTH;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < TITAN_MIN_SHUTTER) ? TITAN_MIN_SHUTTER : shutter;
	if (shutter > TITAN_MAX_FRAME_LENGTH - TITAN_MARGIN)
		shutter = TITAN_MAX_FRAME_LENGTH - TITAN_MARGIN;

	if (imgsensor_autoflicker_en) {
		realtime_fps = imgsensor_pclk / imgsensor_line_length * 10
			/ imgsensor_frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			write_cmos_sensor(0x0160,
					  (imgsensor_frame_length >> 8) & 0xFF);
			write_cmos_sensor(0x0161,
					  imgsensor_frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor(0x0160, (imgsensor_frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x0161, imgsensor_frame_length & 0xFF);
	}

	write_cmos_sensor(0x015A, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x015B, shutter & 0xFF);

	LOG_INF("Exit! shutter =%d, framelength =%d\n",
		shutter, imgsensor_frame_length);
}

#define IMX219MIPI_MaxGainIndex 97

static const kal_uint16
IMX219MIPI_sensorGainMapping[IMX219MIPI_MaxGainIndex][2] = {
	{64, 0}, {68, 12}, {71, 23}, {74, 33}, {77, 42}, {81, 52}, {84, 59},
	{87, 66}, {90, 73}, {93, 79}, {96, 85}, {100, 91}, {103, 96},
	{106, 101}, {109, 105}, {113, 110}, {116, 114}, {120, 118}, {122, 121},
	{125, 125}, {128, 128}, {132, 131}, {135, 134}, {138, 137}, {141, 139},
	{144, 142}, {148, 145}, {151, 147}, {153, 149}, {157, 151}, {160, 153},
	{164, 156}, {168, 158}, {169, 159}, {173, 161}, {176, 163}, {180, 165},
	{182, 166}, {187, 168}, {189, 169}, {193, 171}, {196, 172}, {200, 174},
	{203, 175}, {205, 176}, {208, 177}, {213, 179}, {216, 180}, {219, 181},
	{222, 182}, {225, 183}, {228, 184}, {232, 185}, {235, 186}, {238, 187},
	{241, 188}, {245, 189}, {249, 190}, {253, 191}, {256, 192}, {260, 193},
	{265, 194}, {269, 195}, {274, 196}, {278, 197}, {283, 198}, {288, 199},
	{293, 200}, {298, 201}, {304, 202}, {310, 203}, {315, 204}, {322, 205},
	{328, 206}, {335, 207}, {342, 208}, {349, 209}, {357, 210}, {365, 211},
	{373, 212}, {381, 213}, {400, 215}, {420, 217}, {432, 218}, {443, 219},
	{468, 221}, {482, 222}, {497, 223}, {512, 224}, {529, 225}, {546, 226},
	{566, 227}, {585, 228}, {607, 229}, {631, 230}, {656, 231}, {683, 232},
};

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;

	LOG_INF("[IMX219MIPI]enter IMX219MIPIGain2Reg function\n");

	for (iI = 0; iI < (IMX219MIPI_MaxGainIndex - 1); iI++) {
		if (gain < IMX219MIPI_sensorGainMapping[iI][0])
			break;
		if (gain == IMX219MIPI_sensorGainMapping[iI][0])
			return IMX219MIPI_sensorGainMapping[iI][1];
	}
	if (gain != IMX219MIPI_sensorGainMapping[iI][0])
		LOG_INF("Gain mapping don't correctly:%d %d \n",
			gain, IMX219MIPI_sensorGainMapping[iI][0]);

	LOG_INF("exit IMX219MIPIGain2Reg function\n");
	/*
	 * Gains below the first table entry (1x) are out of range; the stock
	 * driver indexes [-1] for them. Clamp instead — that is the only
	 * place this driver deliberately differs from the stock one.
	 */
	return IMX219MIPI_sensorGainMapping[iI ? iI - 1 : 0][1];
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* TITAN: iPreGain is only ever cleared by open(), so this skips 0. */
	if (gain == iPreGain)
		return gain;

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	/* TITAN: nothing reads the cached gain; the stock build dropped it. */
	spin_unlock(&imgsensor_drv_lock);

	write_cmos_sensor(0x0157, reg_gain);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	return gain;
}

static void set_test_pattern_mode(kal_bool enable)
{
	kal_uint16 width, height;

	LOG_INF("enable: %d\n", enable);

	imgsensor_test_pattern = enable;

	if (current_scenario_id == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG) {
		width = 3280;
		height = 2464;
		/*
		 * TITAN: in capture the pattern is switched on either way;
		 * only the cached flag above follows the argument.
		 */
		enable = KAL_TRUE;
	} else {
		width = 1640;
		height = 1232;
	}

	write_cmos_sensor(0x0600, 0x00);
	write_cmos_sensor(0x0601, enable ? 0x02 : 0x00);
	write_cmos_sensor(0x0624, (width >> 8) & 0xFF);
	write_cmos_sensor(0x0625, width & 0xFF);
	write_cmos_sensor(0x0626, (height >> 8) & 0xFF);
	write_cmos_sensor(0x0627, height & 0xFF);
	write_cmos_sensor(0x6128, 0x00);
	write_cmos_sensor(0x6129, 0x02);
	write_cmos_sensor(0x613C, (width >> 8) & 0xFF);
	write_cmos_sensor(0x613D, width & 0xFF);
	write_cmos_sensor(0x613E, (height >> 8) & 0xFF);
	write_cmos_sensor(0x613F, height & 0xFF);
	write_cmos_sensor(0x6506, 0x00);
	write_cmos_sensor(0x6507, 0x00);
}

/*
 * TITAN: only the IMAGE_NORMAL arm survives in the stock build (control()
 * always passes IMAGE_NORMAL); it discards the value it reads back, writes a
 * literal and falls through to the default log.
 */
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
	LOG_INF("set_mirror_flip function\n");

	read_cmos_sensor(0x0172);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0172, 0x03);
		/* fall through */
	default:
		LOG_INF("Error image_mirror setting\n");
	}
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0100, 0x01);
	else
		write_cmos_sensor(0x0100, 0x00);

	return ERROR_NONE;
}

/* Mode register tables, transcribed from the stock driver. */
static const kal_uint16 preview_setting_table[] = {
	0x0100, 0x00, 0x30eb, 0x05, 0x30eb, 0x0c, 0x300a, 0xff,
	0x300b, 0xff, 0x30eb, 0x05, 0x30eb, 0x09, 0x0114, 0x01,
	0x0128, 0x00, 0x012a, 0x18, 0x012b, 0x00, 0x0160, 0x05,
	0x0161, 0x34, 0x0162, 0x0d, 0x0163, 0x78, 0x0164, 0x00,
	0x0165, 0x00, 0x0166, 0x0c, 0x0167, 0xcf, 0x0168, 0x00,
	0x0169, 0x00, 0x016a, 0x09, 0x016b, 0x9f, 0x016c, 0x06,
	0x016d, 0x68, 0x016e, 0x04, 0x016f, 0xd0, 0x0170, 0x01,
	0x0171, 0x01, 0x0172, 0x00, 0x0174, 0x01, 0x0175, 0x01,
	0x018c, 0x0a, 0x018d, 0x0a, 0x0301, 0x05, 0x0303, 0x01,
	0x0304, 0x03, 0x0305, 0x03, 0x0306, 0x00, 0x0307, 0x36,
	0x0309, 0x0a, 0x030b, 0x01, 0x030c, 0x00, 0x030d, 0x6c,
	0x455e, 0x00, 0x471e, 0x4b, 0x4767, 0x0f, 0x4750, 0x14,
	0x4540, 0x00, 0x47b4, 0x14, 0x4713, 0x30, 0x478b, 0x10,
	0x478f, 0x10, 0x4793, 0x10, 0x4797, 0x0e, 0x479b, 0x0e,
	0x0100, 0x01,
};
static const kal_uint16 capture_setting_table[] = {
	0x0100, 0x00, 0x30eb, 0x05, 0x30eb, 0x0c, 0x300a, 0xff,
	0x300b, 0xff, 0x30eb, 0x05, 0x30eb, 0x09, 0x0114, 0x01,
	0x0128, 0x00, 0x012a, 0x18, 0x012b, 0x00, 0x0160, 0x09,
	0x0161, 0xf0, 0x0162, 0x0d, 0x0163, 0x78, 0x0164, 0x00,
	0x0165, 0x00, 0x0166, 0x0c, 0x0167, 0xcf, 0x0168, 0x00,
	0x0169, 0x00, 0x016a, 0x09, 0x016b, 0x9f, 0x016c, 0x0c,
	0x016d, 0xd0, 0x016e, 0x09, 0x016f, 0xa0, 0x0170, 0x01,
	0x0171, 0x01, 0x0172, 0x00, 0x0174, 0x00, 0x0175, 0x00,
	0x018c, 0x0a, 0x018d, 0x0a, 0x0301, 0x05, 0x0303, 0x01,
	0x0304, 0x03, 0x0305, 0x03, 0x0306, 0x00, 0x0307, 0x36,
	0x0309, 0x0a, 0x030b, 0x01, 0x030c, 0x00, 0x030d, 0x6c,
	0x455e, 0x00, 0x471e, 0x4b, 0x4767, 0x0f, 0x4750, 0x14,
	0x4540, 0x00, 0x47b4, 0x14, 0x4713, 0x30, 0x478b, 0x10,
	0x478f, 0x10, 0x4793, 0x10, 0x4797, 0x0e, 0x479b, 0x0e,
	0x0100, 0x01,
};
/* TITAN: normal video reuses the preview window, hs video the capture one. */
static const kal_uint16 video_setting_table[] = {
	0x0100, 0x00, 0x30eb, 0x05, 0x30eb, 0x0c, 0x300a, 0xff,
	0x300b, 0xff, 0x30eb, 0x05, 0x30eb, 0x09, 0x0114, 0x01,
	0x0128, 0x00, 0x012a, 0x18, 0x012b, 0x00, 0x0160, 0x05,
	0x0161, 0x34, 0x0162, 0x0d, 0x0163, 0x78, 0x0164, 0x00,
	0x0165, 0x00, 0x0166, 0x0c, 0x0167, 0xcf, 0x0168, 0x00,
	0x0169, 0x00, 0x016a, 0x09, 0x016b, 0x9f, 0x016c, 0x06,
	0x016d, 0x68, 0x016e, 0x04, 0x016f, 0xd0, 0x0170, 0x01,
	0x0171, 0x01, 0x0172, 0x00, 0x0174, 0x01, 0x0175, 0x01,
	0x018c, 0x0a, 0x018d, 0x0a, 0x0301, 0x05, 0x0303, 0x01,
	0x0304, 0x03, 0x0305, 0x03, 0x0306, 0x00, 0x0307, 0x36,
	0x0309, 0x0a, 0x030b, 0x01, 0x030c, 0x00, 0x030d, 0x6c,
	0x455e, 0x00, 0x471e, 0x4b, 0x4767, 0x0f, 0x4750, 0x14,
	0x4540, 0x00, 0x47b4, 0x14, 0x4713, 0x30, 0x478b, 0x10,
	0x478f, 0x10, 0x4793, 0x10, 0x4797, 0x0e, 0x479b, 0x0e,
	0x0100, 0x01,
};
static const kal_uint16 hs_setting_table[] = {
	0x0100, 0x00, 0x30eb, 0x05, 0x30eb, 0x0c, 0x300a, 0xff,
	0x300b, 0xff, 0x30eb, 0x05, 0x30eb, 0x09, 0x0114, 0x01,
	0x0128, 0x00, 0x012a, 0x18, 0x012b, 0x00, 0x0160, 0x09,
	0x0161, 0xf0, 0x0162, 0x0d, 0x0163, 0x78, 0x0164, 0x00,
	0x0165, 0x00, 0x0166, 0x0c, 0x0167, 0xcf, 0x0168, 0x00,
	0x0169, 0x00, 0x016a, 0x09, 0x016b, 0x9f, 0x016c, 0x0c,
	0x016d, 0xd0, 0x016e, 0x09, 0x016f, 0xa0, 0x0170, 0x01,
	0x0171, 0x01, 0x0172, 0x00, 0x0174, 0x00, 0x0175, 0x00,
	0x018c, 0x0a, 0x018d, 0x0a, 0x0301, 0x05, 0x0303, 0x01,
	0x0304, 0x03, 0x0305, 0x03, 0x0306, 0x00, 0x0307, 0x36,
	0x0309, 0x0a, 0x030b, 0x01, 0x030c, 0x00, 0x030d, 0x6c,
	0x455e, 0x00, 0x471e, 0x4b, 0x4767, 0x0f, 0x4750, 0x14,
	0x4540, 0x00, 0x47b4, 0x14, 0x4713, 0x30, 0x478b, 0x10,
	0x478f, 0x10, 0x4793, 0x10, 0x4797, 0x0e, 0x479b, 0x0e,
	0x0100, 0x01,
};
static const kal_uint16 slim_setting_table[] = {
	0x0100, 0x00, 0x30eb, 0x05, 0x30eb, 0x0c, 0x300a, 0xff,
	0x300b, 0xff, 0x30eb, 0x05, 0x30eb, 0x09, 0x0114, 0x01,
	0x0128, 0x00, 0x012a, 0x18, 0x012b, 0x00, 0x0160, 0x09,
	0x0161, 0xf0, 0x0162, 0x0d, 0x0163, 0x78, 0x0164, 0x00,
	0x0165, 0x00, 0x0166, 0x0c, 0x0167, 0xcf, 0x0168, 0x01,
	0x0169, 0x32, 0x016a, 0x08, 0x016b, 0x6d, 0x016c, 0x06,
	0x016d, 0x68, 0x016e, 0x03, 0x016f, 0x9e, 0x0170, 0x01,
	0x0171, 0x01, 0x0172, 0x00, 0x0174, 0x01, 0x0175, 0x01,
	0x018c, 0x0a, 0x018d, 0x0a, 0x0301, 0x05, 0x0303, 0x01,
	0x0304, 0x03, 0x0305, 0x03, 0x0306, 0x00, 0x0307, 0x36,
	0x0309, 0x0a, 0x030b, 0x01, 0x030c, 0x00, 0x030d, 0x6c,
	0x455e, 0x00, 0x471e, 0x4b, 0x4767, 0x0f, 0x4750, 0x14,
	0x4540, 0x00, 0x47b4, 0x14, 0x4713, 0x30, 0x478b, 0x10,
	0x478f, 0x10, 0x4793, 0x10, 0x4797, 0x0e, 0x479b, 0x0e,
	0x0100, 0x01,
};

#define TABLE_SIZE(t) (sizeof(t) / sizeof((t)[0]))

static void write_table(const kal_uint16 *table, kal_uint32 n)
{
	kal_uint32 i;

	for (i = 0; i < n; i += 2)
		write_cmos_sensor(table[i], table[i + 1]);
}

static void preview_setting(void)
{
	write_table(preview_setting_table, TABLE_SIZE(preview_setting_table));
}

/*
 * TITAN: the four framerate-taking variants only log the framerate; the
 * register tables are fixed. slim_video_setting() logs plain "E" instead.
 */
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	write_table(capture_setting_table, TABLE_SIZE(capture_setting_table));
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	write_table(video_setting_table, TABLE_SIZE(video_setting_table));
}

static void hs_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	write_table(hs_setting_table, TABLE_SIZE(hs_setting_table));
}

static void slim_video_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
	write_table(slim_setting_table, TABLE_SIZE(slim_setting_table));
}

/*
 * TITAN: sensor_init() writes no registers — the mode tables carry the whole
 * power-on sequence.
 */
static void sensor_init(void)
{
	LOG_INF("E\n");
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_i2c_addr_table[i] != 0xFF) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor_i2c_write_id = imgsensor_i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == IMX219_SENSOR_ID) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor_i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, i2c write id: 0x%x id: 0x%x\n",
				imgsensor_i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != IMX219_SENSOR_ID) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_INF("imx219,MIPI 2LANE\n");
	LOG_2;

	while (imgsensor_i2c_addr_table[i] != 0xFF) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor_i2c_write_id = imgsensor_i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == IMX219_SENSOR_ID)
				break;
			LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n",
				imgsensor_i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == IMX219_SENSOR_ID)
			break;
		retry = 2;
	}
	if (sensor_id != IMX219_SENSOR_ID)
		return ERROR_SENSOR_CONNECT_FAIL;

	LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
		imgsensor_i2c_write_id, sensor_id);

	sensor_init();

	iPreGain = 0;
	spin_lock(&imgsensor_drv_lock);
	imgsensor_pclk = TITAN_PCLK;
	imgsensor_line_length = TITAN_LINE_LENGTH;
	imgsensor_frame_length = TITAN_FL_PREVIEW;
	imgsensor_min_frame_length = TITAN_FL_PREVIEW;
	imgsensor_autoflicker_en = KAL_FALSE;
	imgsensor_dummy_line = 0;
	imgsensor_current_fps = 300;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	return ERROR_NONE;
}

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor_pclk = TITAN_PCLK;
	imgsensor_line_length = TITAN_LINE_LENGTH;
	imgsensor_frame_length = TITAN_FL_PREVIEW;
	imgsensor_min_frame_length = TITAN_FL_PREVIEW;
	imgsensor_autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor_pclk = TITAN_PCLK;
	imgsensor_line_length = TITAN_LINE_LENGTH;
	imgsensor_frame_length = TITAN_FL_CAPTURE;
	imgsensor_min_frame_length = TITAN_FL_CAPTURE;
	imgsensor_autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor_current_fps);
	set_mirror_flip(IMAGE_NORMAL);

	/* Capture is the only mode that re-applies a pending test pattern. */
	if (imgsensor_test_pattern) {
		set_test_pattern_mode(KAL_TRUE);
		imgsensor_test_pattern = KAL_FALSE;
	}
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor_pclk = TITAN_PCLK;
	imgsensor_line_length = TITAN_LINE_LENGTH;
	imgsensor_frame_length = TITAN_FL_PREVIEW;
	imgsensor_min_frame_length = TITAN_FL_PREVIEW;
	imgsensor_autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor_current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor_pclk = TITAN_PCLK;
	imgsensor_line_length = TITAN_LINE_LENGTH;
	imgsensor_frame_length = TITAN_FL_CAPTURE;
	imgsensor_min_frame_length = TITAN_FL_CAPTURE;
	imgsensor_dummy_line = 0;
	imgsensor_autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	hs_video_setting(imgsensor_current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor_pclk = TITAN_PCLK;
	imgsensor_line_length = TITAN_LINE_LENGTH;
	imgsensor_frame_length = TITAN_FL_CAPTURE;
	imgsensor_min_frame_length = TITAN_FL_CAPTURE;
	imgsensor_dummy_line = 0;
	imgsensor_autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	slim_video_setting(imgsensor_current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32 get_resolution
	(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");

	sensor_resolution->SensorPreviewWidth = 1632;
	sensor_resolution->SensorPreviewHeight = 1224;
	sensor_resolution->SensorFullWidth = 3264;
	sensor_resolution->SensorFullHeight = 2448;
	sensor_resolution->SensorVideoWidth = 1632;
	sensor_resolution->SensorVideoHeight = 1224;
	sensor_resolution->SensorHighSpeedVideoWidth = 3264;
	sensor_resolution->SensorHighSpeedVideoHeight = 2448;
	sensor_resolution->SensorSlimVideoWidth = 1280;
	sensor_resolution->SensorSlimVideoHeight = 720;
	sensor_resolution->SensorCustom1Width = 1632;
	sensor_resolution->SensorCustom1Height = 1224;
	sensor_resolution->SensorCustom2Width = 1632;
	sensor_resolution->SensorCustom2Height = 1224;
	sensor_resolution->SensorCustom3Width = 1632;
	sensor_resolution->SensorCustom3Height = 1224;
	sensor_resolution->SensorCustom4Width = 1632;
	sensor_resolution->SensorCustom4Height = 1224;
	sensor_resolution->SensorCustom5Width = 1632;
	sensor_resolution->SensorCustom5Height = 1224;

	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockFreq = 24;
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;
	sensor_info->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;
	sensor_info->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_B;
	sensor_info->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;

	sensor_info->CaptureDelayFrame = 2;
	sensor_info->PreviewDelayFrame = 2;
	sensor_info->VideoDelayFrame = 2;
	sensor_info->HighSpeedVideoDelayFrame = 2;
	sensor_info->SlimVideoDelayFrame = 2;
	sensor_info->Custom1DelayFrame = 2;
	sensor_info->Custom2DelayFrame = 2;
	sensor_info->Custom3DelayFrame = 2;
	sensor_info->Custom4DelayFrame = 2;
	sensor_info->Custom5DelayFrame = 2;

	sensor_info->SensorDrivingCurrent = ISP_DRIVING_8MA;
	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->AEShutDelayFrame = 0;
	sensor_info->AESensorGainDelayFrame = 1;
	sensor_info->AEISPGainDelayFrame = 2;
	sensor_info->FrameTimeDelayFrame = 2;
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->MIPIsensorType = MIPI_OPHY_NCSI2;
	sensor_info->SensorModeNum = 3;
	sensor_info->IHDR_Support = 0;
	sensor_info->IHDR_LE_FirstLine = 0;
	sensor_info->SettleDelayMode = MIPI_SETTLEDELAY_MANUAL;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = 4;
		sensor_info->SensorGrabStartY = 4;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 85;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = 2;
		sensor_info->SensorGrabStartY = 2;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = 4;
		sensor_info->SensorGrabStartY = 4;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = 4;
		sensor_info->SensorGrabStartY = 4;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;
		break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
		sensor_info->SensorGrabStartX = 2;
		sensor_info->SensorGrabStartY = 2;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 32;
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	spin_lock(&imgsensor_drv_lock);
	current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	/*
	 * TITAN: the five custom scenarios run the preview mode. The stock
	 * driver has a copy of preview() per scenario, differing only in the
	 * name it logs.
	 */
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CUSTOM2:
	case MSDK_SCENARIO_ID_CUSTOM3:
	case MSDK_SCENARIO_ID_CUSTOM4:
	case MSDK_SCENARIO_ID_CUSTOM5:
		preview(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);

	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		return ERROR_NONE;

	spin_lock(&imgsensor_drv_lock);
	if (framerate == 300 && imgsensor_autoflicker_en)
		imgsensor_current_fps = 296;
	else if (framerate == 150 && imgsensor_autoflicker_en)
		imgsensor_current_fps = 146;
	else
		imgsensor_current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);

	set_max_framerate(imgsensor_current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);

	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor_autoflicker_en = KAL_TRUE;
	else
		imgsensor_autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

/*
 * Program the frame length for a framerate, padding the mode's own frame
 * length with dummy lines, and push it out if it would not cut the current
 * exposure short.
 */
static void apply_framerate(MUINT32 framerate, kal_uint32 mode_framelength)
{
	kal_uint32 frame_length;

	frame_length = TITAN_PCLK / framerate * 10 / TITAN_LINE_LENGTH;

	spin_lock(&imgsensor_drv_lock);
	imgsensor_dummy_line = (frame_length > mode_framelength)
		? (frame_length - mode_framelength) : 0;
	imgsensor_frame_length = mode_framelength + imgsensor_dummy_line;
	imgsensor_min_frame_length = imgsensor_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	if (imgsensor_frame_length > imgsensor_shutter)
		set_dummy();
}

static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		apply_framerate(framerate, TITAN_FL_PREVIEW);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		apply_framerate(framerate, TITAN_FL_CAPTURE);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		apply_framerate(framerate, TITAN_FL_PREVIEW);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		apply_framerate(framerate, TITAN_FL_CAPTURE);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		apply_framerate(framerate, TITAN_FL_CAPTURE);
		/*
		 * TITAN: the stock driver has no break here, so slim video is
		 * immediately reprogrammed with the preview frame length
		 * below. Kept as is — the tuning depends on it.
		 */
		/* fall through */
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CUSTOM2:
	case MSDK_SCENARIO_ID_CUSTOM3:
	case MSDK_SCENARIO_ID_CUSTOM4:
	case MSDK_SCENARIO_ID_CUSTOM5:
		apply_framerate(framerate, TITAN_FL_PREVIEW);
		break;
	default:	/* coding with preview scenario by default */
		apply_framerate(framerate, TITAN_FL_PREVIEW);
		LOG_INF("error scenario_id = %d, we use preview scenario \n",
			scenario_id);
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = 200;
		break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	case MSDK_SCENARIO_ID_CUSTOM1:
	case MSDK_SCENARIO_ID_CUSTOM2:
	case MSDK_SCENARIO_ID_CUSTOM3:
	case MSDK_SCENARIO_ID_CUSTOM4:
	case MSDK_SCENARIO_ID_CUSTOM5:
		*framerate = 300;
		break;
	default:
		/* TITAN: out-of-range scenarios leave *framerate alone. */
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data =
		(unsigned long long *) feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);

	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor_line_length;
		*feature_return_para_16 = imgsensor_frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor_pclk, imgsensor_current_fps);
		*feature_return_para_32 = imgsensor_pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
				  sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* no lens driver: LENS_DRIVER_ID_DO_NOT_CARE */
		*feature_return_para_32 = -1;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = 0x9E08861C;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor_current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
		spin_lock(&imgsensor_drv_lock);
		/* TITAN: the ihdr flag is stored nowhere; nothing reads it. */
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32) *feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy(wininfo, &imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy(wininfo, &imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy(wininfo, &imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy(wininfo, &imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy(wininfo, &imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		/*
		 * TITAN: the stock driver has no break here either, so
		 * GET_CROP_INFO also emits the iHDR log below.
		 */
		/* fall through */
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data, (UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data,
					 (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
		streaming_control(KAL_TRUE);
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX219_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
