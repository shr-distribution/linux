 /*
 *
 * Filename:
 * ---------
 *     bf2553Lmipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
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

#include "bf2553lmipiraw_Sensor.h"

/************************** Modify Following Strings for Debug **************************/
#define PFX "bf2553L_camera_sensor"
#define LOG_1 LOG_INF("BF2553LMIPI, 2LANE\n")
/****************************   Modify end    *******************************************/
#define BF2553L_DEBUG 0
#if BF2553L_DEBUG
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct bf2553L_otp_t bf2553L_otp_data;

/* SENSOR FLIP */
#define BF2553L_HFLIP_EN    0
#define BF2553L_VFLIP_EN    1

static UINT8	gVersion_ID	= 0x00;	
#define STATE1    0x0c
#define STATE2    (BF2553L_HFLIP_EN << 3) | (BF2553L_VFLIP_EN << 2) 

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = BF2553L_SENSOR_ID,
	.checksum_value = 0x30feb9af,

	.pre = {
		.pclk = 166400000,
		.linelength = 2820,
		.framelength = 1966,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 166400000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 166400000,
		.linelength = 2820,
		.framelength = 1966,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 166400000,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 166400000,
		.linelength = 2820,
		.framelength = 1966,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 166400000,
		.max_framerate = 240,             /*less than 13M(include 13M)*/
	},
	.normal_video = {
		.pclk = 166400000,
		.linelength = 2820,
		.framelength = 1966,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 166400000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 166400000,
		.linelength = 2820,
		.framelength = 1966,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 166400000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 166400000,
		.linelength = 2820,
		.framelength = 1966,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 166400000,
		.max_framerate = 300,
	},

	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 4,
	.margin = 2,
	.min_shutter = 1,
	.max_frame_length = 0xfffe,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
#if BF2553L_VFLIP_EN
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x7c, 0xff},
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x258,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x7c,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* Preview */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* capture */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* video  */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944},/* hs video */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0, 0, 2592, 1944} /*slim video */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = {(char)(addr & 0xff)};

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xff), (char)(para & 0xff)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

static kal_uint8 bf2553L_read_otp(kal_uint16 addr)
{
	kal_uint8 value;

	write_cmos_sensor(0xb0, ((addr >> 8) & 0xff));
	write_cmos_sensor(0xb1, addr & 0xff);
	write_cmos_sensor(0xb3, 0x80);
	value = read_cmos_sensor(0xb4);
	write_cmos_sensor(0xb3, 0x00);//complete

	return value;
}

static void bf2553L_read_otp_group(kal_uint16 addr, kal_uint8 *buff, int size)
{
	kal_uint8 i;

	write_cmos_sensor(0xb6, 0x15);	
	write_cmos_sensor(0xb0, ((addr >> 8) & 0xff));
	write_cmos_sensor(0xb1, addr & 0xff);
	write_cmos_sensor(0xb3, 0x80);

	for (i = 0; i < size; i++)
		buff[i] = read_cmos_sensor(0xb4);
		
	write_cmos_sensor(0xb3, 0x00);//complete
}

static void bf2553L_byd_read_reg(void)
{
	kal_uint8 i = 0;
	kal_uint8 j = 0;
	kal_uint16 base_group = 0;
	kal_uint8 reg[BF2553L_OTP_REG_DATA_SIZE];
	struct bf2553L_reg_update_t *pRegs = &bf2553L_otp_data.regs;

	memset(&reg, 0, BF2553L_OTP_REG_DATA_SIZE);
	pRegs->flag = bf2553L_read_otp(BF2553L_OTP_REG_FLAG_OFFSET);
	LOG_INF("register update flag = 0x%x\n", pRegs->flag);
	if (pRegs->flag == BF2553L_OTP_FLAG_VALID) {
		bf2553L_read_otp_group(BF2553L_OTP_REG_DATA_OFFSET, &reg[0], BF2553L_OTP_REG_DATA_SIZE);

		for (i = 0, pRegs->cnt = 0; i < BF2553L_OTP_REG_MAX_GROUP; i++) {
			base_group = i * BF2553L_OTP_REG_BYTE_PER_GROUP;
			LOG_INF("register flag[%d] = 0x%x\n", i, reg[base_group]);			
			for (j = 0; j < BF2553L_OTP_REG_REG_PER_GROUP; j++)
				if (BF2553L_OTP_CHECK_1BIT_FLAG(reg[base_group], j)) {
					pRegs->reg[pRegs->cnt].addr =
						reg[base_group + j * BF2553L_OTP_REG_BYTE_PER_REG + 1];
					pRegs->reg[pRegs->cnt].value =
						reg[base_group + j * BF2553L_OTP_REG_BYTE_PER_REG + 2];
					LOG_INF("register[%d]:0x%x->0x%x\n",
						pRegs->cnt,pRegs->reg[pRegs->cnt].addr, pRegs->reg[pRegs->cnt].value);
					pRegs->cnt++;
				}
		}

	}
}
#if BF2553L_OTP_FOR_CUSTOMER
static kal_uint8 bf2553L_otp_read_module_info(void)
{
	kal_uint8 i = 0;
	kal_uint8 idx = 0;
	kal_uint8 flag = 0;
	kal_uint16 check = 0;
	kal_uint16 module_start_offset = BF2553L_OTP_MODULE_DATA_OFFSET;
	kal_uint8 info[BF2553L_OTP_MODULE_DATA_SIZE];
	struct bf2553L_module_info_t module_info = { 0 };

	memset(&info, 0, BF2553L_OTP_MODULE_DATA_SIZE);
	memset(&module_info, 0, sizeof(struct bf2553L_module_info_t));

	flag = bf2553L_read_otp(BF2553L_OTP_MODULE_FLAG_OFFSET);
	LOG_INF("flag = 0x%x\n", flag);

	for (idx = 0; idx < BF2553L_OTP_GROUP_CNT; idx++) {
		switch (BF2553L_OTP_GET_2BIT_FLAG(flag, 2 * idx)) {
		case BF2553L_OTP_FLAG_EMPTY: {
			LOG_INF("group %d is empty!\n", idx + 1);
			break;
		}
		case BF2553L_OTP_FLAG_VALID: {
			LOG_INF("group %d is valid!\n", idx + 1);
			module_start_offset = BF2553L_OTP_MODULE_DATA_OFFSET
				+ idx * BF2553L_OTP_DATA_SIZE_OFFSET;
			bf2553L_read_otp_group(module_start_offset, &info[0], BF2553L_OTP_MODULE_DATA_SIZE);
			for (i = 0; i < BF2553L_OTP_MODULE_DATA_SIZE - 1; i++)
				check += info[i];

			if ((check % 255 + 1) == info[BF2553L_OTP_MODULE_DATA_SIZE - 1]) {
				module_info.module_id = info[0];
				module_info.lens_id = info[1];
				module_info.vcm_id = info[2];			
				module_info.year = ((info[4] << 8) | info[3]);
				module_info.month = info[5];
				module_info.day = info[6];

				LOG_INF("module_id = 0x%x\n", module_info.module_id);
				LOG_INF("lens_id = 0x%x\n", module_info.lens_id);
				LOG_INF("vcm_id = 0x%x\n", module_info.vcm_id);				
				LOG_INF("data = %d-%d-%d\n", module_info.year, module_info.month, module_info.day);
			} else
				LOG_INF("check sum %d error! check sum = 0x%x, calculate result = 0x%x\n",
					idx + 1, info[BF2553L_OTP_MODULE_DATA_SIZE - 1], (check % 255 + 1));
			check = 0;
			break;
		}
		case BF2553L_OTP_FLAG_INVALID:
		case BF2553L_OTP_FLAG_INVALID2: {
			LOG_INF("group %d is invalid!\n", idx + 1);
			break;
		}
		default:
			break;
		}
	}

	return module_info.module_id;
}
static void bf2553L_otp_read_wb_info(void)
{
	kal_uint8 i = 0;
	kal_uint8 idx = 0;
	kal_uint8 flag_wb = 0;
	kal_uint8 flag_golden = 0;	
	kal_uint16 wb_check = 0;
	kal_uint16 golden_check = 0;
	kal_uint16 wb_start_offset = BF2553L_OTP_WB_DATA_OFFSET;
	kal_uint16 golden_start_offset = BF2553L_OTP_GOLDEN_DATA_OFFSET;
	kal_uint8 wb[BF2553L_OTP_WB_DATA_SIZE];
	kal_uint8 golden[BF2553L_OTP_GOLDEN_DATA_SIZE];
	struct bf2553L_wb_t *pWB = &bf2553L_otp_data.wb;
	struct bf2553L_wb_t *pGolden = &bf2553L_otp_data.golden;

	memset(&wb, 0, BF2553L_OTP_WB_DATA_SIZE);
	memset(&golden, 0, BF2553L_OTP_GOLDEN_DATA_SIZE);
	flag_wb = bf2553L_read_otp(BF2553L_OTP_WB_FLAG_OFFSET);
	flag_golden = bf2553L_read_otp(BF2553L_OTP_WB_GOLDEN_FLAG_OFFSET);	
	LOG_INF("flag_wb = 0x%x, flag_golden = 0x%x\n", flag_wb, flag_golden);

	for (idx = 0; idx < BF2553L_OTP_GROUP_CNT; idx++) {
		switch (BF2553L_OTP_GET_2BIT_FLAG(flag_wb, 2 * idx)) {
		case BF2553L_OTP_FLAG_EMPTY: {
			LOG_INF("wb group %d is empty!\n", idx + 1);
			pWB->flag = pWB->flag | BF2553L_OTP_FLAG_EMPTY;
			break;
		}
		case BF2553L_OTP_FLAG_VALID: {
			LOG_INF("wb group %d is valid!\n", idx + 1);
			wb_start_offset = BF2553L_OTP_WB_DATA_OFFSET
				+ idx * BF2553L_OTP_DATA_SIZE_OFFSET;
			bf2553L_read_otp_group(wb_start_offset, &wb[0], BF2553L_OTP_WB_DATA_SIZE);

			for (i = 0; i < BF2553L_OTP_WB_DATA_SIZE - 1; i++)
				wb_check += wb[i];
				LOG_INF("wb_check = 0x%x\n", wb_check);
			if ((wb_check % 255 + 1) == wb[BF2553L_OTP_WB_DATA_SIZE - 1]) {
				pWB->rg = (((wb[0] << 8) & 0xff00) | wb[1]);
				pWB->bg = (((wb[2] << 8) & 0xff00) | wb[3]);
				pWB->rg = pWB->rg == 0 ? BF2553L_OTP_WB_RG_TYPICAL : pWB->rg;
				pWB->bg = pWB->bg == 0 ? BF2553L_OTP_WB_BG_TYPICAL : pWB->bg;
				pWB->flag = pWB->flag | BF2553L_OTP_FLAG_VALID;
				LOG_INF("wb r/g = 0x%x\n", pWB->rg);
				LOG_INF("wb b/g = 0x%x\n", pWB->bg);
			} else {
				pWB->flag = pWB->flag | BF2553L_OTP_FLAG_INVALID;
				LOG_INF("wb check sum %d error! check sum = 0x%x, calculate result = 0x%x\n",
					idx + 1, wb[BF2553L_OTP_WB_DATA_SIZE - 1], (wb_check % 255 + 1));
			}
			wb_check = 0;
			break;
		}
		case BF2553L_OTP_FLAG_INVALID:
		case BF2553L_OTP_FLAG_INVALID2: {
			LOG_INF("wb group %d is invalid!\n", idx + 1);
			pWB->flag = pWB->flag | BF2553L_OTP_FLAG_INVALID;
			break;
		}
		default:
			break;
		}

		switch (BF2553L_OTP_GET_2BIT_FLAG(flag_golden, 2 * idx)) {
		case BF2553L_OTP_FLAG_EMPTY: {
			LOG_INF("golden group %d is empty!\n", idx + 1);
			pGolden->flag = pGolden->flag | BF2553L_OTP_FLAG_EMPTY;
			break;
		}
		case BF2553L_OTP_FLAG_VALID: {
			LOG_INF("golden group %d is valid!\n", idx + 1);
			golden_start_offset = BF2553L_OTP_GOLDEN_DATA_OFFSET
				+ idx * BF2553L_OTP_DATA_SIZE_OFFSET;
			bf2553L_read_otp_group(golden_start_offset, &golden[0], BF2553L_OTP_GOLDEN_DATA_SIZE);
			for (i = 0; i < BF2553L_OTP_GOLDEN_DATA_SIZE - 1; i++)
				golden_check += golden[i];
				LOG_INF("golden_check = 0x%x\n", golden_check);
			if ((golden_check % 255 + 1) == golden[BF2553L_OTP_GOLDEN_DATA_SIZE - 1]) {
				pGolden->rg = (((golden[0] << 8) & 0xff00) | golden[1]);
				pGolden->bg = (((golden[2] << 8) & 0xff00) | golden[3]);
				pGolden->rg = pGolden->rg == 0 ? BF2553L_OTP_WB_RG_TYPICAL : pGolden->rg;
				pGolden->bg = pGolden->bg == 0 ? BF2553L_OTP_WB_BG_TYPICAL : pGolden->bg;
				pGolden->flag = pGolden->flag | BF2553L_OTP_FLAG_VALID;
				LOG_INF("golden r/g = 0x%x\n", pGolden->rg);
				LOG_INF("golden b/g = 0x%x\n", pGolden->bg);
			} else {
				pGolden->flag = pGolden->flag | BF2553L_OTP_FLAG_INVALID;
				LOG_INF("golden check sum %d error! check sum = 0x%x, calculate result = 0x%x\n",
					idx + 1, golden[BF2553L_OTP_WB_DATA_SIZE - 1], (golden_check % 255 + 1));
			}
			golden_check = 0;
			break;
		}
		case BF2553L_OTP_FLAG_INVALID:
		case BF2553L_OTP_FLAG_INVALID2: {
			LOG_INF("golden group %d is invalid!\n", idx + 1);
			pGolden->flag = pGolden->flag | BF2553L_OTP_FLAG_INVALID;
			break;
		}
		default:
			break;
		}
	}
}
#endif
static kal_uint8 bf2553L_otp_read_sensor_info(void)
{
	kal_uint8 moduleID = 0;
#if BF2553L_OTP_DEBUG
	kal_uint16 i = 0;
	kal_uint8 debug[BF2553L_OTP_DATA_LENGTH];
#endif
	bf2553L_byd_read_reg();
	
#if BF2553L_OTP_FOR_CUSTOMER
	moduleID = bf2553L_otp_read_module_info();
	bf2553L_otp_read_wb_info();
#endif

#if BF2553L_OTP_DEBUG
	memset(&debug[0], 0, BF2553L_OTP_DATA_LENGTH);
	bf2553L_read_otp_group(BF2553L_OTP_START_ADDR, &debug[0], BF2553L_OTP_DATA_LENGTH);
	for (i = 0; i < BF2553L_OTP_DATA_LENGTH; i++)
		LOG_INF("addr = 0x%x, data = 0x%x\n", BF2553L_OTP_START_ADDR + i, debug[i]);
#endif

	return moduleID;
}

#if BF2553L_OTP_FOR_CUSTOMER
static void bf2553L_otp_update_wb(void)
{
	kal_uint16 r_gain = BF2553L_OTP_WB_GAIN_BASE;
	kal_uint16 b_gain = BF2553L_OTP_WB_GAIN_BASE;
	kal_uint16 rg_typical = BF2553L_OTP_WB_RG_TYPICAL;
	kal_uint16 bg_typical = BF2553L_OTP_WB_BG_TYPICAL;
	struct bf2553L_wb_t *pWB = &bf2553L_otp_data.wb;
	struct bf2553L_wb_t *pGolden = &bf2553L_otp_data.golden;

	if (BF2553L_OTP_CHECK_1BIT_FLAG(pGolden->flag, 0)) {
		rg_typical = pGolden->rg;
		bg_typical = pGolden->bg;
	} else {
		rg_typical = BF2553L_OTP_WB_RG_TYPICAL;
		bg_typical = BF2553L_OTP_WB_BG_TYPICAL;
	}
	LOG_INF("typical rg = 0x%x, bg = 0x%x\n", rg_typical, bg_typical);

	if (BF2553L_OTP_CHECK_1BIT_FLAG(pWB->flag, 0)) {
		r_gain = BF2553L_OTP_WB_GAIN_BASE * rg_typical / pWB->rg;
		b_gain = BF2553L_OTP_WB_GAIN_BASE * bg_typical / pWB->bg;
		LOG_INF("channel gain r = 0x%x, b = 0x%x\n", r_gain, b_gain);

		write_cmos_sensor(0x8c, 0x20);
		write_cmos_sensor(0x8d, 0x20);		
		write_cmos_sensor(0x66, (r_gain & 0xff) | 0x80);
		write_cmos_sensor(0x67, b_gain & 0xff);
	}
}
#endif

static void bf2553L_otp_update_reg(void)
{
	kal_uint8 i = 0;

	LOG_INF("reg count = %d\n", bf2553L_otp_data.regs.cnt);

	if (BF2553L_OTP_CHECK_1BIT_FLAG(bf2553L_otp_data.regs.flag, 0))
		for (i = 0; i < bf2553L_otp_data.regs.cnt; i++) {
			write_cmos_sensor(bf2553L_otp_data.regs.reg[i].addr, bf2553L_otp_data.regs.reg[i].value);
			LOG_INF("reg[%d]:0x%x -> 0x%x\n", i, bf2553L_otp_data.regs.reg[i].addr, bf2553L_otp_data.regs.reg[i].value);
		}
}

static void bf2553L_otp_function(void)
{
	bf2553L_otp_read_sensor_info();
#if BF2553L_OTP_FOR_CUSTOMER	
	bf2553L_otp_update_wb();
#endif	
	bf2553L_otp_update_reg();
}

static void set_dummy(void)
{
	kal_uint32 vb = 0;
	kal_uint32 basic_line = 1966;

	vb = imgsensor.frame_length - basic_line;

	write_cmos_sensor(0x07, (vb >> 8) & 0xff);
	write_cmos_sensor(0x06, vb & 0xff);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0xfc) << 8) | read_cmos_sensor(0xfd));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	/* kal_uint32 frame_length = 0; */
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*if shutter bigger than frame_length, should extend frame length first*/
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

/*	if (imgsensor.autoflicker_en) {
*		if (realtime_fps >= 297 && realtime_fps <= 305)
*			set_max_framerate(296, 0);
*		else if (realtime_fps >= 147 && realtime_fps <= 150)
*			set_max_framerate(146, 0);
*		else
*			set_max_framerate(realtime_fps, 0);
*	} else
*		set_max_framerate(realtime_fps, 0);
*/

	write_cmos_sensor(0x6a, (shutter >> 8) & 0xff);
	write_cmos_sensor(0x6d, shutter & 0xff);

	LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = ((gain << 4) / BASEGAIN) - 1;

	reg_gain = reg_gain & 0xFF;
	
	LOG_INF( "dengjianjun----BASEGAIN=%d, gain=%d, reg_gain=0x%x\n ", BASEGAIN,gain, reg_gain );

	return (kal_uint16)reg_gain;
}



static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain = 0;

	if (gain < 0x40)
		gain = 0x40;
	else if (gain > 1024)
		gain = 1024;
	reg_gain = gain2reg( gain );
	spin_lock( &imgsensor_drv_lock );
	imgsensor.gain = reg_gain;
	spin_unlock( &imgsensor_drv_lock );
	LOG_INF( "gain=%d, reg_gain=0x%x\n ", gain, reg_gain );
	
	write_cmos_sensor(0x6b, reg_gain & 0xff);
	
	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}


/*static void set_mirror_flip(kal_uint8 image_mirror)
*{
*	LOG_INF("image_mirror = %d\n", image_mirror);
*}
*/

static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}

static void sensor_init(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0xf2, 0x01);//Reset
	write_cmos_sensor(0x02, 0xaf);
	write_cmos_sensor(0x03, 0x60);
	write_cmos_sensor(0x14, 0xff);
	write_cmos_sensor(0x29, 0x28);//29-28 0919
	write_cmos_sensor(0x2a, 0x2c);
	write_cmos_sensor(0x32, 0xfc);
	write_cmos_sensor(0x34, 0xe0);//add 20220805	
	write_cmos_sensor(0x35, 0xb8);//add 20220919	
	write_cmos_sensor(0x40, 0x62);//52-62 0712
	if( gVersion_ID == 0 ) {
	write_cmos_sensor(0xe1, 0x49);
	}
	else {
	write_cmos_sensor(0xe1, 0xc9);
	}//add 20220919
	write_cmos_sensor(0xe6, 0x73);//9b-93 0712
	write_cmos_sensor(0xe7, 0x23);//add 20220805		
	write_cmos_sensor(0xe9, 0xd0);//d0-f0 1112 d0 20220805
	write_cmos_sensor(0xea, 0xab);

	//5M Resolution		
	write_cmos_sensor(0xe2, 0x02);
	write_cmos_sensor(0xe3, 0x66);
	write_cmos_sensor(0xe4, 0xd0);
	write_cmos_sensor(0xe5, 0x06);//MIPICLK:832M //0a-06 0712
	write_cmos_sensor(0x00, (0x41 | STATE1));
	write_cmos_sensor(0xa0, 0x03);
	write_cmos_sensor(0xca, 0xa0);
	write_cmos_sensor(0xcb, 0x70);
	if(BF2553L_HFLIP_EN == 1 ) {
	write_cmos_sensor(0xcc, 0x02);
	write_cmos_sensor(0xcd, 0x22);	
	}
	else {
	write_cmos_sensor(0xcc, 0x04);
	write_cmos_sensor(0xcd, 0x24);
	}//add 20220919		
	write_cmos_sensor(0xce, 0x04);
	write_cmos_sensor(0xcf, 0x9c);
	//black target
	write_cmos_sensor(0x58, 0x20);
	write_cmos_sensor(0x59, 0x20);
	write_cmos_sensor(0x5a, 0x20);
	write_cmos_sensor(0x5b, 0x20);
	//832M MIPI Setting 
	write_cmos_sensor(0x70, 0x08);
	write_cmos_sensor(0x71, 0x07);
	write_cmos_sensor(0x72, 0x16);
	write_cmos_sensor(0x73, 0x09);
	write_cmos_sensor(0x74, 0x08);
	write_cmos_sensor(0x75, 0x06);
	write_cmos_sensor(0x76, 0x30);
	write_cmos_sensor(0x77, 0x02);
	write_cmos_sensor(0x78, 0x10);
	write_cmos_sensor(0x79, 0x0a);
	write_cmos_sensor(0x7d, 0x1f);
		
	write_cmos_sensor(0x6A, 0x06);//intt_H
	write_cmos_sensor(0x6D, 0xea);//intt_L
	write_cmos_sensor(0x6B, 0x4f);//global gain
	write_cmos_sensor(0x6f, 0x10);//digital gain
	//color gain 1X
	write_cmos_sensor(0x66, 0x90);
	write_cmos_sensor(0x67, 0x10);
	write_cmos_sensor(0x68, 0x10);
	write_cmos_sensor(0x69, 0x10);
	if(BF2553L_HFLIP_EN == 1 && BF2553L_VFLIP_EN == 0)
	write_cmos_sensor(0x98, 0x90);
}

static void preview_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x01, 0x4a); //0x01[3]=1
	write_cmos_sensor(0xf3, 0x00); //Power up
	write_cmos_sensor(0x56, 0x10);
	write_cmos_sensor(0x01, 0x42); //0x01[3]=0
	if(BF2553L_HFLIP_EN == 0 && BF2553L_VFLIP_EN == 0)	
	mdelay(25);
	write_cmos_sensor(0x00, (0x41 | STATE2));
}

static void capture_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x01, 0x4a); //0x01[3]=1
	write_cmos_sensor(0xf3, 0x00); //Power up
	write_cmos_sensor(0x56, 0x10);
	write_cmos_sensor(0x01, 0x42); //0x01[3]=0
	if(BF2553L_HFLIP_EN == 0 && BF2553L_VFLIP_EN == 0)	
	mdelay(25);
	write_cmos_sensor(0x00, (0x41 | STATE2));		
}
static void normal_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x01, 0x4a); //0x01[3]=1
	write_cmos_sensor(0xf3, 0x00); //Power up
	write_cmos_sensor(0x56, 0x10);
	write_cmos_sensor(0x01, 0x42); //0x01[3]=0	
	if(BF2553L_HFLIP_EN == 0 && BF2553L_VFLIP_EN == 0)	
	mdelay(25);
	write_cmos_sensor(0x00, (0x41 | STATE2));	
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x01, 0x4a); //0x01[3]=1
	write_cmos_sensor(0xf3, 0x00); //Power up
	write_cmos_sensor(0x56, 0x10);
	write_cmos_sensor(0x01, 0x42); //0x01[3]=0
	if(BF2553L_HFLIP_EN == 0 && BF2553L_VFLIP_EN == 0)	
	mdelay(25);
	write_cmos_sensor(0x00, (0x41 | STATE2));		
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x01, 0x4a); //0x01[3]=1
	write_cmos_sensor(0xf3, 0x00); //Power up
	write_cmos_sensor(0x56, 0x10);
	write_cmos_sensor(0x01, 0x42); //0x01[3]=0
	if(BF2553L_HFLIP_EN == 0 && BF2553L_VFLIP_EN == 0)	
	mdelay(25);
	write_cmos_sensor(0x00, (0x41 | STATE2));		
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x94, 0x80);
	} else {
		write_cmos_sensor(0x94, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	printk("bf2553l get_imgsensor_id enter\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("bf2553l i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			printk("bf2553l Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
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
	kal_uint8	value = 0;	

	LOG_1;

	printk("bf2553l open enter\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				printk("bf2553l i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			printk("bf2553l Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id){
			value = read_cmos_sensor( 0xfb ) & 0x0f;
			if( ( value == 4 ) || ( value == 2 ) )	gVersion_ID = 0;
			else	gVersion_ID = 1;			
			break;
		}
	    retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	bf2553L_otp_function();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/* No Need to implement this function */
	return ERROR_NONE;
}


static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}


static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting();
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	 MSDK_SENSOR_INFO_STRUCT *sensor_info,
	 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
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
	imgsensor.current_scenario_id = scenario_id;
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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
	/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
			imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		//*(feature_data + 2) = imgsensor_info.exp_step;
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;

	/*only for Android R (Android 11)/ISP 5.0 
	delete SENSOR_FEATURE_GET_PIXEL_RATE in Android Q (Android 10) & earlier versions*/
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;
		
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps: %d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable: %d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId: %d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
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

UINT32 BF2553L_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
