// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx766gmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
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

#include "imx766gmipiraw_Sensor.h"

/***************Modify Following Strings for Debug**********************/
#define PFX "IMX766G_camera_sensor"
/****************************   Modify end	**************************/
#ifdef IMX766G_DEBUG
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif
//#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args) // pr_debug

static DEFINE_SPINLOCK(imgsensor_drv_lock);
//static kal_uint8  test_pattern_flag=0;

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX766G_SENSOR_ID,

	.checksum_value = 0x467afd3a,

	.pre = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
	.cap = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
	.cap1 = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
	.normal_video = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},	

	.hs_video = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
	.slim_video = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
    .custom1 = {
		.pclk = 1785600000,
		.linelength = 27648,
		.framelength = 6458,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8192,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 100,
		.mipi_pixel_rate = 742400000,
	},
    .custom2 = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
    .custom3 = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
    .custom4 = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
    .custom5 = {
		.pclk = 4128000000,
		.linelength = 46848,
		.framelength = 3164,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 280,
		.mipi_pixel_rate = 512000000,
	},
	.margin = 48,
	.min_shutter = 30,
    .min_gain = 64,
    .max_gain = 1024,
    .min_gain_iso = 100,
    //.exp_step = 2,
    .gain_step = 2,
    .gain_type = 2,
	.max_frame_length = 0x7fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
    .frame_time_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 8,	  //support sensor mode num

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,//SENSOR_OUTPUT_FORMAT_RAW_R,
	.mclk = 24,
  //pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;	
	.mipi_lane_num = SENSOR_MIPI_4_LANE, //SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0xff},
    .i2c_speed = 300, // i2c read/write speed
};

//#define IMX766MIPI_WRITE_ID (0x34)
//#define IMX766MIPI_READ_ID	(0x35)

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_mode = 0,
    .hdr_mode = 0,
	.i2c_write_id = 0x20, //0x20,
    .current_ae_effective_frame = 1,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =
{ 
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // Preview
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // capture
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // video
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, //hight speed video
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // slim video
	{ 8192, 6144,	 0,	0, 8192, 6144, 8192, 6144, 0000, 0000, 8192, 6144,	  0,	0, 8192,  6144}, // Custom1
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // Custom2
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // Custom3
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // Custom4
	{ 8192, 6144,	 0,	0, 8192, 6144, 4096,  3072, 0000, 0000, 4096, 3072,	  0,	0, 4096,  3072}, // Custom5
 };

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0} },
    .i4PosR = {{0, 0} },
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,

    .i4Crop = {
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
    },  //{0, 1632}
    .iMirrorFlip = 0,
};


static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[6] = {
	{
		//vc2(L+R) 2048*768
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,//preview
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},
			{VC_PDAF_STATS, 0x00, 0x30, 0xA00, 0x300},
			//{VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x500, 0x180},
		},//VC_PDAF_STATS :2     VC_STAGGER_NE:9
		1 //0
	},
	{
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,//preview
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},
			{VC_PDAF_STATS, 0x00, 0x30, 0xA00, 0x300},
			//{VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x500, 0x180},
		},//VC_PDAF_STATS :2     VC_STAGGER_NE:9
		1 //0
	},

	{
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,//video
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},//
			{VC_PDAF_STATS, 0x00, 0x30, 0xA00, 0x300},//		
			//{VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x500, 0x180},
		},
		1 //2
	},
	{
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,//custom1
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x2000, 0x1800},//
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xA00, 0x240},//	
			//{VC_PDAF_STATS_PIX_2, 0x00, 0x31, 0x500, 0x180},
		},
		1 //3
	},
	
};
static void get_vc_info_2(struct SENSOR_VC_INFO2_STRUCT *pvcinfo2, kal_uint32 scenario)
{
	LOG_INF("scenario:%d \n",scenario);
	switch (scenario) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[1],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[2],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[3],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[4],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	/*case MSDK_SCENARIO_ID_CUSTOM3:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[5],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;*/
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	}
}
// Gain Index
#define MaxGainIndex (97)
#if 0
static kal_uint16 sensorGainMapping[MaxGainIndex][2] ={
{ 64 ,0  },
{ 68 ,12 },
{ 71 ,23 },
{ 74 ,33 },
{ 77 ,42 },
{ 81 ,52 },
{ 84 ,59 },
{ 87 ,66 },
{ 90 ,73 },
{ 93 ,79 },
{ 96 ,85 },
{ 100,91 },
{ 103,96 },
{ 106,101},
{ 109,105},
{ 113,110},
{ 116,114},
{ 120,118},
{ 122,121},
{ 125,125},
{ 128,128},
{ 132,131},
{ 135,134},
{ 138,137},
{ 141,139},
{ 144,142},
{ 148,145},
{ 151,147},
{ 153,149},
{ 157,151},
{ 160,153},
{ 164,156},
{ 168,158},
{ 169,159},
{ 173,161},
{ 176,163},
{ 180,165},
{ 182,166},
{ 187,168},
{ 189,169},
{ 193,171},
{ 196,172},
{ 200,174},
{ 203,175},
{ 205,176},
{ 208,177},
{ 213,179},
{ 216,180},
{ 219,181},
{ 222,182},
{ 225,183},
{ 228,184},
{ 232,185},
{ 235,186},
{ 238,187},
{ 241,188},
{ 245,189},
{ 249,190},
{ 253,191},
{ 256,192},
{ 260,193},
{ 265,194},
{ 269,195},
{ 274,196},
{ 278,197},
{ 283,198},
{ 288,199},
{ 293,200},
{ 298,201},
{ 304,202},
{ 310,203},
{ 315,204},
{ 322,205},
{ 328,206},
{ 335,207},
{ 342,208},
{ 349,209},
{ 357,210},
{ 365,211},
{ 373,212},
{ 381,213},
{ 400,215},
{ 420,217},
{ 432,218},
{ 443,219},
{ 468,221},
{ 482,222},
{ 497,223},
{ 512,224},
{ 529,225},
{ 546,226},
{ 566,227},
{ 585,228},
{ 607,229},
{ 631,230},
{ 656,231},
{ 683,232}
};
#endif

#if 0 //mars
// Gain Indexs
//kal_uint16 IMX766MIPI_sensorGainMapping[IMX766MIPI_MaxGainIndex][2] = {
static kal_uint16 sensorGainMapping[MaxGainIndex][2] ={
	{64 ,1	},
	{67 ,13 },
	{70 ,24 },
	{74 ,34 },
	{77 ,43 },
	{80 ,52 },
	{83 ,59 },
	{87 ,67 },
	{90 ,73 },
	{93 ,80 },
	{96 ,85 },
	{99 ,91 },
	{102,96 },
	{106,101},
	{109,105},
	{112,110},
	{115,114},
	{119,118},
	{122,121},
	{125,125},
	{128,128},
	{131,131},
	{134,134},
	{138,137},
	{141,140},
	{143,141},
	{144,142},
	{148,145},
	{150,147},
	{153,149},
	{157,152},
	{159,153},
	{164,156},
	{167,158},
	{171,160},
	{174,162},
	{178,164},
	{182,166},
	{186,168},
	{191,170},
	{195,172},
	{200,174},
	{205,176},
	{210,178}, 
	{216,180}, 
	{221,182}, 
	{228,184}, 
	{234,186}, 
	{241,188}, 
	{248,190}, 
	{256,192}, 
	{264,194},
	{273,196},
	{282,198},
	{292,200},
	{303,202},
	{328,206},
	{341,208},
	{356,210},
	{372,212},
	{381,213},
	{390,214},
	{399,215},
	{410,216},
	{420,217},
	{431,218},
	{443,219},
	{455,220},
	{468,221},
	{482,222},
	{497,223},
	{512,224},
	{512,224},
	{546,226},
	{585,228},
	{630,230},
	{683,232},
	{712,233},
	{745,234},
	{780,235},
	{819,236},
	{862,237},
	{910,238},
	{964,239},
	{1024,240}
};
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
 
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	   kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
 	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
   kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

/*
static kal_uint16 read_eeprom_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xa0);
	return get_byte;
}*/

static void imx766g_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
		pr_debug("%x %x", regDa[idx], regDa[idx+1]);
	}
}
static void imx766g_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		write_cmos_sensor_8(regDa[idx], regDa[idx + 1]);
		pr_debug("%x %x", regDa[idx], regDa[idx+1]);
	}
}
/*

static void set_dummy(void)
{
//IMX766MIPI_write_cmos_sensor(0x0104, 1);        
//IMX766MIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
//IMX766MIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	
//IMX766MIPI_write_cmos_sensor(0x0342, (line_length >>8) & 0xFF);
//IMX766MIPI_write_cmos_sensor(0x0343, line_length & 0xFF);
//IMX766MIPI_write_cmos_sensor(0x0104, 0);
	
//	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor(0x0340, (imgsensor.frame_length >>8) & 0xFF);
    write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
   // write_cmos_sensor(0x0342, (imgsensor.line_length >>8) & 0xFF);
//    write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);
}	 */


static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
    if (enable) {
        write_cmos_sensor(0x0100, 0x01);
    } else {
        write_cmos_sensor(0x0100, 0x00);
    }
    mdelay(10);
    return ERROR_NONE;
}
extern int   current_sensor_idx;
static void set_max_framerate(kal_uint16 framerate,kal_bool min_framelength_en)
{
//	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;
	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)?frame_length:imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
	//	imgsensor.dummy_line = 0;
	//else
	//	imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	//set_dummy();
}	/*	set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
  //  kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    //write_shutter(shutter);
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    // Framelength should be an even number
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;


	
////IMX766MIPI_write_cmos_sensor(0x0100,0x00);// STREAM STop
//IMX766MIPI_write_cmos_sensor(0x0104, 1);        
//IMX766MIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
//IMX766MIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	
//IMX766MIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
//IMX766MIPI_write_cmos_sensor(0x0203, shutter  & 0xFF);
//IMX766MIPI_write_cmos_sensor(0x0104, 0);  

	if (imgsensor.autoflicker_en) {
		  realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		  if(realtime_fps >= 297 && realtime_fps <= 305)
		  {
			  set_max_framerate(296,0);
			  write_cmos_sensor_8(0x0104, 0x01);
			  write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
			  write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			  //write_cmos_sensor(0x0342, (imgsensor.line_length >> 8) & 0xFF);
			  //write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
			  write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
			  write_cmos_sensor(0x0203, shutter  & 0xFF);
			  write_cmos_sensor_8(0x0104, 0x00);
			  LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
			  //set_dummy();
		  }
		  else if(realtime_fps >= 147 && realtime_fps <= 150)
		  {
			  set_max_framerate(146,0);
			  write_cmos_sensor_8(0x0104, 0x01);
			  write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
			  write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			//  write_cmos_sensor(0x0342, (imgsensor.line_length >> 8) & 0xFF);
			//  write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
			  write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
			  write_cmos_sensor(0x0203, shutter  & 0xFF);
			  write_cmos_sensor_8(0x0104, 0x00);
			  LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
			  //set_dummy();
		  }
		  else {
			  // Extend frame length
			  write_cmos_sensor_8(0x0104, 0x01);
			  write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
			  write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			  write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
			  write_cmos_sensor(0x0203, shutter  & 0xFF);
			  write_cmos_sensor_8(0x0104, 0x00);
		  }
	  } else {
		  // Extend frame length
		  write_cmos_sensor_8(0x0104, 0x01);
		  write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
		  write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		  write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
		  write_cmos_sensor(0x0203, shutter  & 0xFF);
		  write_cmos_sensor_8(0x0104, 0x00);
	  }
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X	*/
	/* [4:9] = M meams M X		 */
	/* Total gain = M + N /16 X   */
	if (gain < BASEGAIN || gain > 32 * BASEGAIN)
	{
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;
	}


	
	reg_gain = 16384 - (16384*64)/gain;


	//reg_gain = gain2reg(gain);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
//IMX766MIPI_write_cmos_sensor(0x0104, 1);
//IMX766MIPI_write_cmos_sensor(0x0205, (kal_uint8)iReg);
//IMX766MIPI_write_cmos_sensor(0x0104, 0);
//iReg=IMX766MIPI_read_cmos_sensor(0x205);
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	return gain;
}	/*	set_gain  */

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        // Extend frame length first
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

        set_gain(gain);
    }

}

#endif
#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{


	/********************************************************
	   *
	   *   0x0101 Sensor mirror flip
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	kal_uint8  iTemp;
	LOG_INF("image_mirror = %d\n", image_mirror);

	iTemp = read_cmos_sensor(0x0101);
	iTemp&= ~0x03; //Clear the mirror and flip bits.
	switch (image_mirror)
	{
		case IMAGE_NORMAL:
			write_cmos_sensor_8(0x0101, iTemp);    //Set normal
			break;
		case IMAGE_H_MIRROR:
            write_cmos_sensor_8(0x0101, iTemp | 0x01); //Set mirror
			break;
		case IMAGE_V_MIRROR:
            write_cmos_sensor_8(0x0101, iTemp | 0x02); //Set flip
			break;
		case IMAGE_HV_MIRROR:
            write_cmos_sensor_8(0x0101, iTemp | 0x03); //Set mirror and flip
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

#endif

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/

static void sensor_init(void)
{
    write_cmos_sensor(0x0136,0x18);
    write_cmos_sensor(0x0137,0x00);
    write_cmos_sensor(0x33F0,0x01);
    write_cmos_sensor(0x33F1,0x08);
    write_cmos_sensor(0x0111,0x02);
    write_cmos_sensor(0x33D3,0x01);
    write_cmos_sensor(0x3892,0x01);
    write_cmos_sensor(0x4C14,0x00);
    write_cmos_sensor(0x4C15,0x07);
    write_cmos_sensor(0x4C16,0x00);
    write_cmos_sensor(0x4C17,0x1B);
    write_cmos_sensor(0x4C1A,0x00);
    write_cmos_sensor(0x4C1B,0x03);
    write_cmos_sensor(0x4C1C,0x00);
    write_cmos_sensor(0x4C1D,0x00);
    write_cmos_sensor(0x4C1E,0x00);
    write_cmos_sensor(0x4C1F,0x02);
    write_cmos_sensor(0x4C20,0x00);
    write_cmos_sensor(0x4C21,0x5F);
    write_cmos_sensor(0x4C26,0x00);
    write_cmos_sensor(0x4C27,0x43);
    write_cmos_sensor(0x4C28,0x00);
    write_cmos_sensor(0x4C29,0x09);
    write_cmos_sensor(0x4C2A,0x00);
    write_cmos_sensor(0x4C2B,0x4A);
    write_cmos_sensor(0x4C2C,0x00);
    write_cmos_sensor(0x4C2D,0x00);
    write_cmos_sensor(0x4C2E,0x00);
    write_cmos_sensor(0x4C2F,0x02);
    write_cmos_sensor(0x4C30,0x00);
    write_cmos_sensor(0x4C31,0xC6);
    write_cmos_sensor(0x4C3E,0x00);
    write_cmos_sensor(0x4C3F,0x55);
    write_cmos_sensor(0x4C52,0x00);
    write_cmos_sensor(0x4C53,0x97);
    write_cmos_sensor(0x4CB4,0x00);
    write_cmos_sensor(0x4CB5,0x55);
    write_cmos_sensor(0x4CC8,0x00);
    write_cmos_sensor(0x4CC9,0x97);
    write_cmos_sensor(0x4D04,0x00);
    write_cmos_sensor(0x4D05,0x4F);
    write_cmos_sensor(0x4D74,0x00);
    write_cmos_sensor(0x4D75,0x55);
    write_cmos_sensor(0x4F06,0x00);
    write_cmos_sensor(0x4F07,0x5F);
    write_cmos_sensor(0x4F48,0x00);
    write_cmos_sensor(0x4F49,0xC6);
    write_cmos_sensor(0x544A,0xFF);
    write_cmos_sensor(0x544B,0xFF);
    write_cmos_sensor(0x544E,0x01);
    write_cmos_sensor(0x544F,0xBD);
    write_cmos_sensor(0x5452,0xFF);
    write_cmos_sensor(0x5453,0xFF);
    write_cmos_sensor(0x5456,0x00);
    write_cmos_sensor(0x5457,0xA5);
    write_cmos_sensor(0x545A,0xFF);
    write_cmos_sensor(0x545B,0xFF);
    write_cmos_sensor(0x545E,0x00);
    write_cmos_sensor(0x545F,0xA5);
    write_cmos_sensor(0x5496,0x00);
    write_cmos_sensor(0x5497,0xA2);
    write_cmos_sensor(0x54F6,0x01);
    write_cmos_sensor(0x54F7,0x55);
    write_cmos_sensor(0x54F8,0x01);
    write_cmos_sensor(0x54F9,0x61);
    write_cmos_sensor(0x5670,0x00);
    write_cmos_sensor(0x5671,0x85);
    write_cmos_sensor(0x5672,0x01);
    write_cmos_sensor(0x5673,0x77);
    write_cmos_sensor(0x5674,0x01);
    write_cmos_sensor(0x5675,0x2F);
    write_cmos_sensor(0x5676,0x02);
    write_cmos_sensor(0x5677,0x55);
    write_cmos_sensor(0x5678,0x00);
    write_cmos_sensor(0x5679,0x85);
    write_cmos_sensor(0x567A,0x01);
    write_cmos_sensor(0x567B,0x77);
    write_cmos_sensor(0x567C,0x01);
    write_cmos_sensor(0x567D,0x2F);
    write_cmos_sensor(0x567E,0x02);
    write_cmos_sensor(0x567F,0x55);
    write_cmos_sensor(0x5680,0x00);
    write_cmos_sensor(0x5681,0x85);
    write_cmos_sensor(0x5682,0x01);
    write_cmos_sensor(0x5683,0x77);
    write_cmos_sensor(0x5684,0x01);
    write_cmos_sensor(0x5685,0x2F);
    write_cmos_sensor(0x5686,0x02);
    write_cmos_sensor(0x5687,0x55);
    write_cmos_sensor(0x5688,0x00);
    write_cmos_sensor(0x5689,0x85);
    write_cmos_sensor(0x568A,0x01);
    write_cmos_sensor(0x568B,0x77);
    write_cmos_sensor(0x568C,0x01);
    write_cmos_sensor(0x568D,0x2F);
    write_cmos_sensor(0x568E,0x02);
    write_cmos_sensor(0x568F,0x55);
    write_cmos_sensor(0x5690,0x01);
    write_cmos_sensor(0x5691,0x7A);
    write_cmos_sensor(0x5692,0x02);
    write_cmos_sensor(0x5693,0x6C);
    write_cmos_sensor(0x5694,0x01);
    write_cmos_sensor(0x5695,0x35);
    write_cmos_sensor(0x5696,0x02);
    write_cmos_sensor(0x5697,0x5B);
    write_cmos_sensor(0x5698,0x01);
    write_cmos_sensor(0x5699,0x7A);
    write_cmos_sensor(0x569A,0x02);
    write_cmos_sensor(0x569B,0x6C);
    write_cmos_sensor(0x569C,0x01);
    write_cmos_sensor(0x569D,0x35);
    write_cmos_sensor(0x569E,0x02);
    write_cmos_sensor(0x569F,0x5B);
    write_cmos_sensor(0x56A0,0x01);
    write_cmos_sensor(0x56A1,0x7A);
    write_cmos_sensor(0x56A2,0x02);
    write_cmos_sensor(0x56A3,0x6C);
    write_cmos_sensor(0x56A4,0x01);
    write_cmos_sensor(0x56A5,0x35);
    write_cmos_sensor(0x56A6,0x02);
    write_cmos_sensor(0x56A7,0x5B);
    write_cmos_sensor(0x56A8,0x01);
    write_cmos_sensor(0x56A9,0x80);
    write_cmos_sensor(0x56AA,0x02);
    write_cmos_sensor(0x56AB,0x72);
    write_cmos_sensor(0x56AC,0x01);
    write_cmos_sensor(0x56AD,0x2F);
    write_cmos_sensor(0x56AE,0x02);
    write_cmos_sensor(0x56AF,0x55);
    write_cmos_sensor(0x5902,0x0E);
    write_cmos_sensor(0x5A50,0x04);
    write_cmos_sensor(0x5A51,0x04);
    write_cmos_sensor(0x5A69,0x01);
    write_cmos_sensor(0x5C49,0x0D);
    write_cmos_sensor(0x5D60,0x08);
    write_cmos_sensor(0x5D61,0x08);
    write_cmos_sensor(0x5D62,0x08);
    write_cmos_sensor(0x5D63,0x08);
    write_cmos_sensor(0x5D64,0x08);
    write_cmos_sensor(0x5D67,0x08);
    write_cmos_sensor(0x5D6C,0x08);
    write_cmos_sensor(0x5D6E,0x08);
    write_cmos_sensor(0x5D71,0x08);
    write_cmos_sensor(0x5D8E,0x14);
    write_cmos_sensor(0x5D90,0x03);
    write_cmos_sensor(0x5D91,0x0A);
    write_cmos_sensor(0x5D92,0x1F);
    write_cmos_sensor(0x5D93,0x05);
    write_cmos_sensor(0x5D97,0x1F);
    write_cmos_sensor(0x5D9A,0x06);
    write_cmos_sensor(0x5D9C,0x1F);
    write_cmos_sensor(0x5DA1,0x1F);
    write_cmos_sensor(0x5DA6,0x1F);
    write_cmos_sensor(0x5DA8,0x1F);
    write_cmos_sensor(0x5DAB,0x1F);
    write_cmos_sensor(0x5DC0,0x06);
    write_cmos_sensor(0x5DC1,0x06);
    write_cmos_sensor(0x5DC2,0x07);
    write_cmos_sensor(0x5DC3,0x06);
    write_cmos_sensor(0x5DC4,0x07);
    write_cmos_sensor(0x5DC7,0x07);
    write_cmos_sensor(0x5DCC,0x07);
    write_cmos_sensor(0x5DCE,0x07);
    write_cmos_sensor(0x5DD1,0x07);
    write_cmos_sensor(0x5E3E,0x00);
    write_cmos_sensor(0x5E3F,0x00);
    write_cmos_sensor(0x5E41,0x00);
    write_cmos_sensor(0x5E48,0x00);
    write_cmos_sensor(0x5E49,0x00);
    write_cmos_sensor(0x5E4A,0x00);
    write_cmos_sensor(0x5E4C,0x00);
    write_cmos_sensor(0x5E4D,0x00);
    write_cmos_sensor(0x5E4E,0x00);
    write_cmos_sensor(0x6026,0x03);
    write_cmos_sensor(0x6028,0x03);
    write_cmos_sensor(0x602A,0x03);
    write_cmos_sensor(0x602C,0x03);
    write_cmos_sensor(0x602F,0x03);
    write_cmos_sensor(0x6036,0x03);
    write_cmos_sensor(0x6038,0x03);
    write_cmos_sensor(0x603A,0x03);
    write_cmos_sensor(0x603C,0x03);
    write_cmos_sensor(0x603F,0x03);
    write_cmos_sensor(0x6074,0x19);
    write_cmos_sensor(0x6076,0x19);
    write_cmos_sensor(0x6078,0x19);
    write_cmos_sensor(0x607A,0x19);
    write_cmos_sensor(0x607D,0x19);
    write_cmos_sensor(0x6084,0x32);
    write_cmos_sensor(0x6086,0x32);
    write_cmos_sensor(0x6088,0x32);
    write_cmos_sensor(0x608A,0x32);
    write_cmos_sensor(0x608D,0x32);
    write_cmos_sensor(0x60C2,0x4A);
    write_cmos_sensor(0x60C4,0x4A);
    write_cmos_sensor(0x60CB,0x4A);
    write_cmos_sensor(0x60D2,0x4A);
    write_cmos_sensor(0x60D4,0x4A);
    write_cmos_sensor(0x60DB,0x4A);
    write_cmos_sensor(0x62F9,0x14);
    write_cmos_sensor(0x6305,0x13);
    write_cmos_sensor(0x6307,0x13);
    write_cmos_sensor(0x630A,0x13);
    write_cmos_sensor(0x630D,0x0D);
    write_cmos_sensor(0x6317,0x0D);
    write_cmos_sensor(0x632F,0x2E);
    write_cmos_sensor(0x6333,0x2E);
    write_cmos_sensor(0x6339,0x2E);
    write_cmos_sensor(0x6343,0x2E);
    write_cmos_sensor(0x6347,0x2E);
    write_cmos_sensor(0x634D,0x2E);
    write_cmos_sensor(0x6352,0x00);
    write_cmos_sensor(0x6353,0x5F);
    write_cmos_sensor(0x6366,0x00);
    write_cmos_sensor(0x6367,0x5F);
    write_cmos_sensor(0x638F,0x95);
    write_cmos_sensor(0x6393,0x95);
    write_cmos_sensor(0x6399,0x95);
    write_cmos_sensor(0x63A3,0x95);
    write_cmos_sensor(0x63A7,0x95);
    write_cmos_sensor(0x63AD,0x95);
    write_cmos_sensor(0x63B2,0x00);
    write_cmos_sensor(0x63B3,0xC6);
    write_cmos_sensor(0x63C6,0x00);
    write_cmos_sensor(0x63C7,0xC6);
    write_cmos_sensor(0x8BDB,0x02);
    write_cmos_sensor(0x8BDE,0x02);
    write_cmos_sensor(0x8BE1,0x2D);
    write_cmos_sensor(0x8BE4,0x00);
    write_cmos_sensor(0x8BE5,0x00);
    write_cmos_sensor(0x8BE6,0x01);
    write_cmos_sensor(0x9002,0x14);
    write_cmos_sensor(0x9200,0xB5);
    write_cmos_sensor(0x9201,0x9E);
    write_cmos_sensor(0x9202,0xB5);
    write_cmos_sensor(0x9203,0x42);
    write_cmos_sensor(0x9204,0xB5);
    write_cmos_sensor(0x9205,0x43);
    write_cmos_sensor(0x9206,0xBD);
    write_cmos_sensor(0x9207,0x20);
    write_cmos_sensor(0x9208,0xBD);
    write_cmos_sensor(0x9209,0x22);
    write_cmos_sensor(0x920A,0xBD);
    write_cmos_sensor(0x920B,0x23);
    write_cmos_sensor(0xB5D7,0x10);
    write_cmos_sensor(0xBD24,0x00);
    write_cmos_sensor(0xBD25,0x00);
    write_cmos_sensor(0xBD26,0x00);
    write_cmos_sensor(0xBD27,0x00);
    write_cmos_sensor(0xBD28,0x00);
    write_cmos_sensor(0xBD29,0x00);
    write_cmos_sensor(0xBD2A,0x00);
    write_cmos_sensor(0xBD2B,0x00);
    write_cmos_sensor(0xBD2C,0x32);
    write_cmos_sensor(0xBD2D,0x70);
    write_cmos_sensor(0xBD2E,0x25);
    write_cmos_sensor(0xBD2F,0x30);
    write_cmos_sensor(0xBD30,0x3B);
    write_cmos_sensor(0xBD31,0xE0);
    write_cmos_sensor(0xBD32,0x69);
    write_cmos_sensor(0xBD33,0x40);
    write_cmos_sensor(0xBD34,0x25);
    write_cmos_sensor(0xBD35,0x90);
    write_cmos_sensor(0xBD36,0x58);
    write_cmos_sensor(0xBD37,0x00);
    write_cmos_sensor(0xBD38,0x00);
    write_cmos_sensor(0xBD39,0x00);
    write_cmos_sensor(0xBD3A,0x00);
    write_cmos_sensor(0xBD3B,0x00);
    write_cmos_sensor(0xBD3C,0x32);
    write_cmos_sensor(0xBD3D,0x70);
    write_cmos_sensor(0xBD3E,0x25);
    write_cmos_sensor(0xBD3F,0x90);
    write_cmos_sensor(0xBD40,0x58);
    write_cmos_sensor(0xBD41,0x00);
    //
    write_cmos_sensor(0x793B,0x01);
    write_cmos_sensor(0xACC6,0x00);
    write_cmos_sensor(0xACF5,0x00);
    write_cmos_sensor(0x793B,0x00);
    //
    write_cmos_sensor(0x1F04,0xB3);
    write_cmos_sensor(0x1F05,0x01);
    write_cmos_sensor(0x1F06,0x07);
    write_cmos_sensor(0x1F07,0x66);
    write_cmos_sensor(0x1F08,0x01);
    write_cmos_sensor(0x4D18,0x00);
    write_cmos_sensor(0x4D19,0x9D);
    write_cmos_sensor(0x4D88,0x00);
    write_cmos_sensor(0x4D89,0x97);
    write_cmos_sensor(0x5C57,0x0A);
    write_cmos_sensor(0x5D94,0x1F);
    write_cmos_sensor(0x5D9E,0x1F);
    write_cmos_sensor(0x5E50,0x23);
    write_cmos_sensor(0x5E51,0x20);
    write_cmos_sensor(0x5E52,0x07);
    write_cmos_sensor(0x5E53,0x20);
    write_cmos_sensor(0x5E54,0x07);
    write_cmos_sensor(0x5E55,0x27);
    write_cmos_sensor(0x5E56,0x0B);
    write_cmos_sensor(0x5E57,0x24);
    write_cmos_sensor(0x5E58,0x0B);
    write_cmos_sensor(0x5E60,0x24);
    write_cmos_sensor(0x5E61,0x24);
    write_cmos_sensor(0x5E62,0x1B);
    write_cmos_sensor(0x5E63,0x23);
    write_cmos_sensor(0x5E64,0x1B);
    write_cmos_sensor(0x5E65,0x28);
    write_cmos_sensor(0x5E66,0x22);
    write_cmos_sensor(0x5E67,0x28);
    write_cmos_sensor(0x5E68,0x23);
    write_cmos_sensor(0x5E70,0x25);
    write_cmos_sensor(0x5E71,0x24);
    write_cmos_sensor(0x5E72,0x20);
    write_cmos_sensor(0x5E73,0x24);
    write_cmos_sensor(0x5E74,0x20);
    write_cmos_sensor(0x5E75,0x28);
    write_cmos_sensor(0x5E76,0x27);
    write_cmos_sensor(0x5E77,0x29);
    write_cmos_sensor(0x5E78,0x24);
    write_cmos_sensor(0x5E80,0x25);
    write_cmos_sensor(0x5E81,0x25);
    write_cmos_sensor(0x5E82,0x24);
    write_cmos_sensor(0x5E83,0x25);
    write_cmos_sensor(0x5E84,0x23);
    write_cmos_sensor(0x5E85,0x2A);
    write_cmos_sensor(0x5E86,0x28);
    write_cmos_sensor(0x5E87,0x2A);
    write_cmos_sensor(0x5E88,0x28);
    write_cmos_sensor(0x5E90,0x24);
    write_cmos_sensor(0x5E91,0x24);
    write_cmos_sensor(0x5E92,0x28);
    write_cmos_sensor(0x5E93,0x29);
    write_cmos_sensor(0x5E97,0x25);
    write_cmos_sensor(0x5E98,0x25);
    write_cmos_sensor(0x5E99,0x2A);
    write_cmos_sensor(0x5E9A,0x2A);
    write_cmos_sensor(0x5E9E,0x3A);
    write_cmos_sensor(0x5E9F,0x3F);
    write_cmos_sensor(0x5EA0,0x17);
    write_cmos_sensor(0x5EA1,0x3F);
    write_cmos_sensor(0x5EA2,0x17);
    write_cmos_sensor(0x5EA3,0x32);
    write_cmos_sensor(0x5EA4,0x10);
    write_cmos_sensor(0x5EA5,0x33);
    write_cmos_sensor(0x5EA6,0x10);
    write_cmos_sensor(0x5EAE,0x3D);
    write_cmos_sensor(0x5EAF,0x48);
    write_cmos_sensor(0x5EB0,0x3B);
    write_cmos_sensor(0x5EB1,0x45);
    write_cmos_sensor(0x5EB2,0x37);
    write_cmos_sensor(0x5EB3,0x3A);
    write_cmos_sensor(0x5EB4,0x31);
    write_cmos_sensor(0x5EB5,0x3A);
    write_cmos_sensor(0x5EB6,0x31);
    write_cmos_sensor(0x5EBE,0x40);
    write_cmos_sensor(0x5EBF,0x48);
    write_cmos_sensor(0x5EC0,0x3F);
    write_cmos_sensor(0x5EC1,0x45);
    write_cmos_sensor(0x5EC2,0x3F);
    write_cmos_sensor(0x5EC3,0x3A);
    write_cmos_sensor(0x5EC4,0x32);
    write_cmos_sensor(0x5EC5,0x3A);
    write_cmos_sensor(0x5EC6,0x33);
    write_cmos_sensor(0x5ECE,0x4B);
    write_cmos_sensor(0x5ECF,0x4A);
    write_cmos_sensor(0x5ED0,0x48);
    write_cmos_sensor(0x5ED1,0x4C);
    write_cmos_sensor(0x5ED2,0x45);
    write_cmos_sensor(0x5ED3,0x3F);
    write_cmos_sensor(0x5ED4,0x3A);
    write_cmos_sensor(0x5ED5,0x3F);
    write_cmos_sensor(0x5ED6,0x3A);
    write_cmos_sensor(0x5EDE,0x48);
    write_cmos_sensor(0x5EDF,0x45);
    write_cmos_sensor(0x5EE0,0x3A);
    write_cmos_sensor(0x5EE1,0x3A);
    write_cmos_sensor(0x5EE5,0x4A);
    write_cmos_sensor(0x5EE6,0x4C);
    write_cmos_sensor(0x5EE7,0x3F);
    write_cmos_sensor(0x5EE8,0x3F);
    write_cmos_sensor(0x5EEC,0x06);
    write_cmos_sensor(0x5EED,0x06);
    write_cmos_sensor(0x5EEE,0x02);
    write_cmos_sensor(0x5EEF,0x06);
    write_cmos_sensor(0x5EF0,0x01);
    write_cmos_sensor(0x5EF1,0x09);
    write_cmos_sensor(0x5EF2,0x05);
    write_cmos_sensor(0x5EF3,0x06);
    write_cmos_sensor(0x5EF4,0x04);
    write_cmos_sensor(0x5EFC,0x07);
    write_cmos_sensor(0x5EFD,0x09);
    write_cmos_sensor(0x5EFE,0x05);
    write_cmos_sensor(0x5EFF,0x08);
    write_cmos_sensor(0x5F00,0x04);
    write_cmos_sensor(0x5F01,0x09);
    write_cmos_sensor(0x5F02,0x05);
    write_cmos_sensor(0x5F03,0x09);
    write_cmos_sensor(0x5F04,0x04);
    write_cmos_sensor(0x5F0C,0x08);
    write_cmos_sensor(0x5F0D,0x09);
    write_cmos_sensor(0x5F0E,0x06);
    write_cmos_sensor(0x5F0F,0x09);
    write_cmos_sensor(0x5F10,0x06);
    write_cmos_sensor(0x5F11,0x09);
    write_cmos_sensor(0x5F12,0x09);
    write_cmos_sensor(0x5F13,0x09);
    write_cmos_sensor(0x5F14,0x06);
    write_cmos_sensor(0x5F1C,0x09);
    write_cmos_sensor(0x5F1D,0x09);
    write_cmos_sensor(0x5F1E,0x09);
    write_cmos_sensor(0x5F1F,0x09);
    write_cmos_sensor(0x5F20,0x08);
    write_cmos_sensor(0x5F21,0x09);
    write_cmos_sensor(0x5F22,0x09);
    write_cmos_sensor(0x5F23,0x09);
    write_cmos_sensor(0x5F24,0x09);
    write_cmos_sensor(0x5F2C,0x09);
    write_cmos_sensor(0x5F2D,0x09);
    write_cmos_sensor(0x5F2E,0x09);
    write_cmos_sensor(0x5F2F,0x09);
    write_cmos_sensor(0x5F33,0x09);
    write_cmos_sensor(0x5F34,0x09);
    write_cmos_sensor(0x5F35,0x09);
    write_cmos_sensor(0x5F36,0x09);
    write_cmos_sensor(0x5F3A,0x01);
    write_cmos_sensor(0x5F3D,0x07);
    write_cmos_sensor(0x5F3F,0x01);
    write_cmos_sensor(0x5F4B,0x01);
    write_cmos_sensor(0x5F4D,0x04);
    write_cmos_sensor(0x5F4F,0x02);
    write_cmos_sensor(0x5F51,0x02);
    write_cmos_sensor(0x5F5A,0x02);
    write_cmos_sensor(0x5F5B,0x01);
    write_cmos_sensor(0x5F5D,0x03);
    write_cmos_sensor(0x5F5E,0x07);
    write_cmos_sensor(0x5F5F,0x01);
    write_cmos_sensor(0x5F60,0x01);
    write_cmos_sensor(0x5F61,0x01);
    write_cmos_sensor(0x5F6A,0x01);
    write_cmos_sensor(0x5F6C,0x01);
    write_cmos_sensor(0x5F6D,0x01);
    write_cmos_sensor(0x5F6E,0x04);
    write_cmos_sensor(0x5F70,0x02);
    write_cmos_sensor(0x5F72,0x02);
    write_cmos_sensor(0x5F7A,0x01);
    write_cmos_sensor(0x5F7B,0x03);
    write_cmos_sensor(0x5F7C,0x01);
    write_cmos_sensor(0x5F7D,0x01);
    write_cmos_sensor(0x5F82,0x01);
    write_cmos_sensor(0x60C6,0x4A);
    write_cmos_sensor(0x60C8,0x4A);
    write_cmos_sensor(0x60D6,0x4A);
    write_cmos_sensor(0x60D8,0x4A);
    write_cmos_sensor(0x62E4,0x33);
    write_cmos_sensor(0x62E9,0x33);
    write_cmos_sensor(0x62EE,0x1C);
    write_cmos_sensor(0x62EF,0x33);
    write_cmos_sensor(0x62F3,0x33);
    write_cmos_sensor(0x62F6,0x1C);
    write_cmos_sensor(0x33F2,0x01);
    write_cmos_sensor(0x1F04,0xA3);
    write_cmos_sensor(0x1F05,0x01);
    write_cmos_sensor(0x406E,0x00);
    write_cmos_sensor(0x406F,0x08);
    write_cmos_sensor(0x4D08,0x00);
    write_cmos_sensor(0x4D09,0x2C);
    write_cmos_sensor(0x4D0E,0x00);
    write_cmos_sensor(0x4D0F,0x64);
    write_cmos_sensor(0x4D18,0x00);
    write_cmos_sensor(0x4D19,0xB1);
    write_cmos_sensor(0x4D1E,0x00);
    write_cmos_sensor(0x4D1F,0xCB);
    write_cmos_sensor(0x4D3A,0x00);
    write_cmos_sensor(0x4D3B,0x91);
    write_cmos_sensor(0x4D40,0x00);
    write_cmos_sensor(0x4D41,0x64);
    write_cmos_sensor(0x4D4C,0x00);
    write_cmos_sensor(0x4D4D,0xE8);
    write_cmos_sensor(0x4D52,0x00);
    write_cmos_sensor(0x4D53,0xCB);
    write_cmos_sensor(0x4D78,0x00);
    write_cmos_sensor(0x4D79,0x2C);
    write_cmos_sensor(0x4D7E,0x00);
    write_cmos_sensor(0x4D7F,0x64);
    write_cmos_sensor(0x4D88,0x00);
    write_cmos_sensor(0x4D89,0xAB);
    write_cmos_sensor(0x4D8E,0x00);
    write_cmos_sensor(0x4D8F,0xCB);
    write_cmos_sensor(0x4DA6,0x00);
    write_cmos_sensor(0x4DA7,0xE7);
    write_cmos_sensor(0x4DAC,0x00);
    write_cmos_sensor(0x4DAD,0xCB);
    write_cmos_sensor(0x5B98,0x00);
    write_cmos_sensor(0x5C52,0x05);
    write_cmos_sensor(0x5C57,0x09);
    write_cmos_sensor(0x5D94,0x0A);
    write_cmos_sensor(0x5D9E,0x0A);
    write_cmos_sensor(0x5E50,0x22);
    write_cmos_sensor(0x5E51,0x22);
    write_cmos_sensor(0x5E52,0x07);
    write_cmos_sensor(0x5E53,0x20);
    write_cmos_sensor(0x5E54,0x06);
    write_cmos_sensor(0x5E55,0x23);
    write_cmos_sensor(0x5E56,0x0A);
    write_cmos_sensor(0x5E57,0x23);
    write_cmos_sensor(0x5E58,0x0A);
    write_cmos_sensor(0x5E60,0x25);
    write_cmos_sensor(0x5E61,0x29);
    write_cmos_sensor(0x5E62,0x1C);
    write_cmos_sensor(0x5E63,0x26);
    write_cmos_sensor(0x5E64,0x1C);
    write_cmos_sensor(0x5E65,0x2D);
    write_cmos_sensor(0x5E66,0x1E);
    write_cmos_sensor(0x5E67,0x2A);
    write_cmos_sensor(0x5E68,0x1E);
    write_cmos_sensor(0x5E70,0x26);
    write_cmos_sensor(0x5E71,0x26);
    write_cmos_sensor(0x5E72,0x22);
    write_cmos_sensor(0x5E73,0x23);
    write_cmos_sensor(0x5E74,0x20);
    write_cmos_sensor(0x5E75,0x28);
    write_cmos_sensor(0x5E76,0x23);
    write_cmos_sensor(0x5E77,0x28);
    write_cmos_sensor(0x5E78,0x23);
    write_cmos_sensor(0x5E80,0x28);
    write_cmos_sensor(0x5E81,0x28);
    write_cmos_sensor(0x5E82,0x29);
    write_cmos_sensor(0x5E83,0x27);
    write_cmos_sensor(0x5E84,0x26);
    write_cmos_sensor(0x5E85,0x2A);
    write_cmos_sensor(0x5E86,0x2D);
    write_cmos_sensor(0x5E87,0x2A);
    write_cmos_sensor(0x5E88,0x2A);
    write_cmos_sensor(0x5E90,0x26);
    write_cmos_sensor(0x5E91,0x23);
    write_cmos_sensor(0x5E92,0x28);
    write_cmos_sensor(0x5E93,0x28);
    write_cmos_sensor(0x5E97,0x2F);
    write_cmos_sensor(0x5E98,0x2E);
    write_cmos_sensor(0x5E99,0x32);
    write_cmos_sensor(0x5E9A,0x32);
    write_cmos_sensor(0x5E9E,0x50);
    write_cmos_sensor(0x5E9F,0x50);
    write_cmos_sensor(0x5EA0,0x1E);
    write_cmos_sensor(0x5EA1,0x50);
    write_cmos_sensor(0x5EA2,0x1D);
    write_cmos_sensor(0x5EA3,0x3E);
    write_cmos_sensor(0x5EA4,0x14);
    write_cmos_sensor(0x5EA5,0x3E);
    write_cmos_sensor(0x5EA6,0x14);
    write_cmos_sensor(0x5EAE,0x58);
    write_cmos_sensor(0x5EAF,0x5E);
    write_cmos_sensor(0x5EB0,0x4B);
    write_cmos_sensor(0x5EB1,0x5A);
    write_cmos_sensor(0x5EB2,0x4B);
    write_cmos_sensor(0x5EB3,0x4C);
    write_cmos_sensor(0x5EB4,0x3A);
    write_cmos_sensor(0x5EB5,0x4C);
    write_cmos_sensor(0x5EB6,0x38);
    write_cmos_sensor(0x5EBE,0x56);
    write_cmos_sensor(0x5EBF,0x57);
    write_cmos_sensor(0x5EC0,0x50);
    write_cmos_sensor(0x5EC1,0x55);
    write_cmos_sensor(0x5EC2,0x50);
    write_cmos_sensor(0x5EC3,0x46);
    write_cmos_sensor(0x5EC4,0x3E);
    write_cmos_sensor(0x5EC5,0x46);
    write_cmos_sensor(0x5EC6,0x3E);
    write_cmos_sensor(0x5ECE,0x5A);
    write_cmos_sensor(0x5ECF,0x5F);
    write_cmos_sensor(0x5ED0,0x5E);
    write_cmos_sensor(0x5ED1,0x5A);
    write_cmos_sensor(0x5ED2,0x5A);
    write_cmos_sensor(0x5ED3,0x50);
    write_cmos_sensor(0x5ED4,0x4C);
    write_cmos_sensor(0x5ED5,0x50);
    write_cmos_sensor(0x5ED6,0x4C);
    write_cmos_sensor(0x5EDE,0x57);
    write_cmos_sensor(0x5EDF,0x55);
    write_cmos_sensor(0x5EE0,0x46);
    write_cmos_sensor(0x5EE1,0x46);
    write_cmos_sensor(0x5EE5,0x73);
    write_cmos_sensor(0x5EE6,0x6E);
    write_cmos_sensor(0x5EE7,0x5F);
    write_cmos_sensor(0x5EE8,0x5A);
    write_cmos_sensor(0x5EEC,0x0A);
    write_cmos_sensor(0x5EED,0x0A);
    write_cmos_sensor(0x5EEE,0x0F);
    write_cmos_sensor(0x5EEF,0x0A);
    write_cmos_sensor(0x5EF0,0x0E);
    write_cmos_sensor(0x5EF1,0x08);
    write_cmos_sensor(0x5EF2,0x0C);
    write_cmos_sensor(0x5EF3,0x0C);
    write_cmos_sensor(0x5EF4,0x0F);
    write_cmos_sensor(0x5EFC,0x0A);
    write_cmos_sensor(0x5EFD,0x0A);
    write_cmos_sensor(0x5EFE,0x14);
    write_cmos_sensor(0x5EFF,0x0A);
    write_cmos_sensor(0x5F00,0x14);
    write_cmos_sensor(0x5F01,0x0A);
    write_cmos_sensor(0x5F02,0x14);
    write_cmos_sensor(0x5F03,0x0A);
    write_cmos_sensor(0x5F04,0x19);
    write_cmos_sensor(0x5F0C,0x0A);
    write_cmos_sensor(0x5F0D,0x0A);
    write_cmos_sensor(0x5F0E,0x0A);
    write_cmos_sensor(0x5F0F,0x05);
    write_cmos_sensor(0x5F10,0x0A);
    write_cmos_sensor(0x5F11,0x06);
    write_cmos_sensor(0x5F12,0x08);
    write_cmos_sensor(0x5F13,0x0A);
    write_cmos_sensor(0x5F14,0x0C);
    write_cmos_sensor(0x5F1C,0x0A);
    write_cmos_sensor(0x5F1D,0x0A);
    write_cmos_sensor(0x5F1E,0x0A);
    write_cmos_sensor(0x5F1F,0x0A);
    write_cmos_sensor(0x5F20,0x0A);
    write_cmos_sensor(0x5F21,0x0A);
    write_cmos_sensor(0x5F22,0x0A);
    write_cmos_sensor(0x5F23,0x0A);
    write_cmos_sensor(0x5F24,0x0A);
    write_cmos_sensor(0x5F2C,0x0A);
    write_cmos_sensor(0x5F2D,0x05);
    write_cmos_sensor(0x5F2E,0x06);
    write_cmos_sensor(0x5F2F,0x0A);
    write_cmos_sensor(0x5F33,0x0A);
    write_cmos_sensor(0x5F34,0x0A);
    write_cmos_sensor(0x5F35,0x0A);
    write_cmos_sensor(0x5F36,0x0A);
    write_cmos_sensor(0x5F3A,0x00);
    write_cmos_sensor(0x5F3D,0x02);
    write_cmos_sensor(0x5F3F,0x0A);
    write_cmos_sensor(0x5F4A,0x0A);
    write_cmos_sensor(0x5F4B,0x0A);
    write_cmos_sensor(0x5F4D,0x0F);
    write_cmos_sensor(0x5F4F,0x00);
    write_cmos_sensor(0x5F51,0x00);
    write_cmos_sensor(0x5F5A,0x00);
    write_cmos_sensor(0x5F5B,0x00);
    write_cmos_sensor(0x5F5D,0x0A);
    write_cmos_sensor(0x5F5E,0x02);
    write_cmos_sensor(0x5F5F,0x0A);
    write_cmos_sensor(0x5F60,0x0A);
    write_cmos_sensor(0x5F61,0x00);
    write_cmos_sensor(0x5F6A,0x00);
    write_cmos_sensor(0x5F6C,0x0A);
    write_cmos_sensor(0x5F6D,0x06);
    write_cmos_sensor(0x5F6E,0x0F);
    write_cmos_sensor(0x5F70,0x00);
    write_cmos_sensor(0x5F72,0x00);
    write_cmos_sensor(0x5F7A,0x00);
    write_cmos_sensor(0x5F7B,0x0A);
    write_cmos_sensor(0x5F7C,0x0A);
    write_cmos_sensor(0x5F7D,0x00);
    write_cmos_sensor(0x5F82,0x06);
    write_cmos_sensor(0x60C6,0x36);
    write_cmos_sensor(0x60C8,0x36);
    write_cmos_sensor(0x60D6,0x36);
    write_cmos_sensor(0x60D8,0x36);
    write_cmos_sensor(0x62DF,0x56);
    write_cmos_sensor(0x62E0,0x52);
    write_cmos_sensor(0x62E4,0x38);
    write_cmos_sensor(0x62E5,0x51);
    write_cmos_sensor(0x62E9,0x35);
    write_cmos_sensor(0x62EA,0x54);
    write_cmos_sensor(0x62EE,0x1D);
    write_cmos_sensor(0x62EF,0x38);
    write_cmos_sensor(0x62F3,0x33);
    write_cmos_sensor(0x62F6,0x26);
    write_cmos_sensor(0x6412,0x1E);
    write_cmos_sensor(0x6413,0x1E);
    write_cmos_sensor(0x6414,0x1E);
    write_cmos_sensor(0x6415,0x1E);
    write_cmos_sensor(0x6416,0x1E);
    write_cmos_sensor(0x6417,0x1E);
    write_cmos_sensor(0x6418,0x1E);
    write_cmos_sensor(0x641A,0x1E);
    write_cmos_sensor(0x641B,0x1E);
    write_cmos_sensor(0x641C,0x1E);
    write_cmos_sensor(0x641D,0x1E);
    write_cmos_sensor(0x641E,0x1E);
    write_cmos_sensor(0x641F,0x1E);
    write_cmos_sensor(0x6420,0x1E);
    write_cmos_sensor(0x6421,0x1E);
    write_cmos_sensor(0x6422,0x1E);
    write_cmos_sensor(0x6424,0x1E);
    write_cmos_sensor(0x6425,0x1E);
    write_cmos_sensor(0x6426,0x1E);
    write_cmos_sensor(0x6427,0x1E);
    write_cmos_sensor(0x6428,0x1E);
    write_cmos_sensor(0x6429,0x1E);
    write_cmos_sensor(0x642A,0x1E);
    write_cmos_sensor(0x642B,0x1E);
    write_cmos_sensor(0x642C,0x1E);
    write_cmos_sensor(0x642E,0x1E);
    write_cmos_sensor(0x642F,0x1E);
    write_cmos_sensor(0x6430,0x1E);
    write_cmos_sensor(0x6431,0x1E);
    write_cmos_sensor(0x6432,0x1E);
    write_cmos_sensor(0x6433,0x1E);
    write_cmos_sensor(0x6434,0x1E);
    write_cmos_sensor(0x6435,0x1E);
    write_cmos_sensor(0x6436,0x1E);
    write_cmos_sensor(0x6438,0x1E);
    write_cmos_sensor(0x6439,0x1E);
    write_cmos_sensor(0x643A,0x1E);
    write_cmos_sensor(0x643B,0x1E);
    write_cmos_sensor(0x643D,0x1E);
    write_cmos_sensor(0x643E,0x1E);
    write_cmos_sensor(0x643F,0x1E);
    write_cmos_sensor(0x6441,0x1E);
    write_cmos_sensor(0x33F2,0x02);
    write_cmos_sensor(0x1F08,0x00);
} /* sensor_init */

static void preview_setting(void)
{
	write_cmos_sensor(0x0112,0x0a);
	write_cmos_sensor(0x0113,0x0a);
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0342,0xb7);//46848
	write_cmos_sensor(0x0343,0x00);
	write_cmos_sensor(0x0340,0x0c);//3164
	write_cmos_sensor(0x0341,0x5c);
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x1f);
	write_cmos_sensor(0x0349,0xff);
	write_cmos_sensor(0x034a,0x17);
	write_cmos_sensor(0x034b,0xff);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0902,0x08);
	write_cmos_sensor(0x3005,0x03);
	write_cmos_sensor(0x3120,0x04);
	write_cmos_sensor(0x3121,0x01);
	write_cmos_sensor(0x3200,0x41);
	write_cmos_sensor(0x3201,0x41);
	write_cmos_sensor(0x32d6,0x00);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040a,0x00);
	write_cmos_sensor(0x040b,0x00);
	write_cmos_sensor(0x040c,0x10);
	write_cmos_sensor(0x040d,0x00);
	write_cmos_sensor(0x040e,0x0c);
	write_cmos_sensor(0x040f,0x00);
	write_cmos_sensor(0x034c,0x10);
	write_cmos_sensor(0x034d,0x00);
	write_cmos_sensor(0x034e,0x0c);
	write_cmos_sensor(0x034f,0x00);
	write_cmos_sensor(0x0301,0x05);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x01);
	write_cmos_sensor(0x0307,0xae);//206  430
	//pclk=206x8/5/2x16=2636.8
	write_cmos_sensor(0x030b,0x02);
	write_cmos_sensor(0x030d,0x03);
	write_cmos_sensor(0x030e,0x01);//260
	write_cmos_sensor(0x030f,0x40);
	//mipidata=260x12/4=780
	write_cmos_sensor(0x30cb,0x01);
	write_cmos_sensor(0x30cc,0x08);
	write_cmos_sensor(0x30cd,0x00);
	write_cmos_sensor(0x30ce,0x03);
	write_cmos_sensor(0x30cf,0x00);
	write_cmos_sensor(0x319c,0x01);
	write_cmos_sensor(0x3800,0x01);
	write_cmos_sensor(0x3801,0x01);
	write_cmos_sensor(0x3802,0x02);
	write_cmos_sensor(0x3847,0x03);
	write_cmos_sensor(0x38b0,0x00);
	write_cmos_sensor(0x38b1,0x64);
	write_cmos_sensor(0x38b2,0x00);
	write_cmos_sensor(0x38b3,0x64);
	write_cmos_sensor(0x38c4,0x00);
	write_cmos_sensor(0x38c5,0x64);
	write_cmos_sensor(0x4c3a,0x02);
	write_cmos_sensor(0x4c3b,0xd2);
	write_cmos_sensor(0x4c68,0x04);
	write_cmos_sensor(0x4c69,0x7e);
	write_cmos_sensor(0x4cf8,0x16);
	write_cmos_sensor(0x4cf9,0xe0);
	write_cmos_sensor(0x4db8,0x08);
	write_cmos_sensor(0x4db9,0x98);
	write_cmos_sensor(0x0202,0x0c);
	write_cmos_sensor(0x0203,0x2c);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xf4);
	write_cmos_sensor(0x313a,0x01);
	write_cmos_sensor(0x313b,0xf4);
	write_cmos_sensor(0x3803,0x00);
	write_cmos_sensor(0x3804,0x17);
	write_cmos_sensor(0x3805,0xc0);
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x020e,0x01);
	write_cmos_sensor(0x020f,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x0218,0x01);
	write_cmos_sensor(0x0219,0x00);
	write_cmos_sensor(0x313c,0x00);
	write_cmos_sensor(0x313d,0x00);
	write_cmos_sensor(0x313e,0x01);
	write_cmos_sensor(0x313f,0x00);
	write_cmos_sensor(0x3220,0x00);
	write_cmos_sensor(0x30b4,0x01);//PHASE_PIX_OUT_EN
	write_cmos_sensor(0x3066,0x00);
	write_cmos_sensor(0x3067,0x30);//data type
	write_cmos_sensor(0x3068,0x00);
	write_cmos_sensor(0x3069,0x30);

    write_cmos_sensor(0x33D0,0x00);
    write_cmos_sensor(0x33D1,0x00);
    write_cmos_sensor(0x33D4,0x01);
    write_cmos_sensor(0x33DC,0x0A);
    write_cmos_sensor(0x33DD,0x0A);
    write_cmos_sensor(0x33DE,0x0A);
    write_cmos_sensor(0x33DF,0x0A);
    write_cmos_sensor(0x3070,0x01);
    write_cmos_sensor(0x3077,0x01);
    write_cmos_sensor(0x3078,0x30);
    write_cmos_sensor(0x3079,0x01);
    write_cmos_sensor(0x307A,0x30);
    write_cmos_sensor(0x307B,0x01);
    write_cmos_sensor(0x3080,0x02);
    write_cmos_sensor(0x3087,0x02);
    write_cmos_sensor(0x3088,0x30);
    write_cmos_sensor(0x3089,0x02);
    write_cmos_sensor(0x308A,0x30);
    write_cmos_sensor(0x308B,0x02);
    write_cmos_sensor(0x3901,0x2B);
    write_cmos_sensor(0x3902,0x00);
    write_cmos_sensor(0x3903,0x12);
    write_cmos_sensor(0x3905,0x2B);
    write_cmos_sensor(0x3906,0x01);
    write_cmos_sensor(0x3907,0x12);
    write_cmos_sensor(0x3909,0x2B);
    write_cmos_sensor(0x390A,0x02);
    write_cmos_sensor(0x390B,0x12);
    write_cmos_sensor(0x3911,0x00);
    //write_cmos_sensor(0x0100,0x01);
} /* preview_setting */

static void custom1_setting(void)
{

write_cmos_sensor(0x0112,0x0A);
write_cmos_sensor(0x0113,0x0A);
write_cmos_sensor(0x0114,0x03);
write_cmos_sensor(0x0342,0x6C);
write_cmos_sensor(0x0343,0x00);
write_cmos_sensor(0x0340,0x19);
write_cmos_sensor(0x0341,0x3A);
write_cmos_sensor(0x0344,0x00);
write_cmos_sensor(0x0345,0x00);
write_cmos_sensor(0x0346,0x00);
write_cmos_sensor(0x0347,0x00);
write_cmos_sensor(0x0348,0x1F);
write_cmos_sensor(0x0349,0xFF);
write_cmos_sensor(0x034A,0x17);
write_cmos_sensor(0x034B,0xFF);
write_cmos_sensor(0x0900,0x00);
write_cmos_sensor(0x0901,0x11);
write_cmos_sensor(0x0902,0x0A);
write_cmos_sensor(0x3005,0x00);
write_cmos_sensor(0x3120,0x00);
write_cmos_sensor(0x3121,0x01);
write_cmos_sensor(0x3200,0x00);
write_cmos_sensor(0x3201,0x00);
write_cmos_sensor(0x32D2,0x00);
write_cmos_sensor(0x32D6,0x01);
write_cmos_sensor(0x0408,0x00);
write_cmos_sensor(0x0409,0x00);
write_cmos_sensor(0x040A,0x00);
write_cmos_sensor(0x040B,0x00);
write_cmos_sensor(0x040C,0x20);
write_cmos_sensor(0x040D,0x00);
write_cmos_sensor(0x040E,0x18);
write_cmos_sensor(0x040F,0x00);
write_cmos_sensor(0x034C,0x20);
write_cmos_sensor(0x034D,0x00);
write_cmos_sensor(0x034E,0x18);
write_cmos_sensor(0x034F,0x00);
write_cmos_sensor(0x0301,0x05);
write_cmos_sensor(0x0303,0x02);
write_cmos_sensor(0x0305,0x04);
write_cmos_sensor(0x0306,0x00);
write_cmos_sensor(0x0307,0xBA);
write_cmos_sensor(0x030B,0x02);
write_cmos_sensor(0x030D,0x03);
write_cmos_sensor(0x030E,0x01);
write_cmos_sensor(0x030F,0xD0);
write_cmos_sensor(0x30CB,0x00);
write_cmos_sensor(0x30CC,0x10);
write_cmos_sensor(0x30CD,0x00);
write_cmos_sensor(0x30CE,0x03);
write_cmos_sensor(0x30CF,0x00);
write_cmos_sensor(0x319C,0x00);
write_cmos_sensor(0x3800,0x00);
write_cmos_sensor(0x3801,0x00);
write_cmos_sensor(0x3802,0x04);
write_cmos_sensor(0x3847,0x00);
write_cmos_sensor(0x38B0,0x00);
write_cmos_sensor(0x38B1,0x00);
write_cmos_sensor(0x38B2,0x00);
write_cmos_sensor(0x38B3,0x00);
write_cmos_sensor(0x38C4,0x01);
write_cmos_sensor(0x38C5,0x7C);
write_cmos_sensor(0x4C3A,0x02);
write_cmos_sensor(0x4C3B,0xD2);
write_cmos_sensor(0x4C68,0x04);
write_cmos_sensor(0x4C69,0x7E);
write_cmos_sensor(0x4CF8,0x07);
write_cmos_sensor(0x4CF9,0x9E);
write_cmos_sensor(0x4DB8,0x08);
write_cmos_sensor(0x4DB9,0x98);
write_cmos_sensor(0x0202,0x19);
write_cmos_sensor(0x0203,0x0A);
write_cmos_sensor(0x0224,0x01);
write_cmos_sensor(0x0225,0xF4);
write_cmos_sensor(0x313A,0x01);
write_cmos_sensor(0x313B,0xF4);
write_cmos_sensor(0x3803,0x01);
write_cmos_sensor(0x3804,0x55);
write_cmos_sensor(0x3805,0x90);
write_cmos_sensor(0x0204,0x00);
write_cmos_sensor(0x0205,0x00);
write_cmos_sensor(0x020E,0x01);
write_cmos_sensor(0x020F,0x00);
write_cmos_sensor(0x0216,0x00);
write_cmos_sensor(0x0217,0x00);
write_cmos_sensor(0x0218,0x01);
write_cmos_sensor(0x0219,0x00);
write_cmos_sensor(0x313C,0x00);
write_cmos_sensor(0x313D,0x00);
write_cmos_sensor(0x313E,0x01);
write_cmos_sensor(0x313F,0x00);
write_cmos_sensor(0x30B4,0x00);
write_cmos_sensor(0x3066,0x00);
write_cmos_sensor(0x3067,0x30);
write_cmos_sensor(0x3068,0x00);
write_cmos_sensor(0x3069,0x30);
write_cmos_sensor(0x33D0,0x00);
write_cmos_sensor(0x33D1,0x00);
write_cmos_sensor(0x33D4,0x01);
write_cmos_sensor(0x33DC,0x0A);
write_cmos_sensor(0x33DD,0x0A);
write_cmos_sensor(0x33DE,0x0A);
write_cmos_sensor(0x33DF,0x0A);
write_cmos_sensor(0x3070,0x01);
write_cmos_sensor(0x3077,0x01);
write_cmos_sensor(0x3078,0x30);
write_cmos_sensor(0x3079,0x01);
write_cmos_sensor(0x307A,0x30);
write_cmos_sensor(0x307B,0x01);
write_cmos_sensor(0x3080,0x02);
write_cmos_sensor(0x3087,0x02);
write_cmos_sensor(0x3088,0x30);
write_cmos_sensor(0x3089,0x02);
write_cmos_sensor(0x308A,0x30);
write_cmos_sensor(0x308B,0x02);
write_cmos_sensor(0x3901,0x2B);
write_cmos_sensor(0x3902,0x00);
write_cmos_sensor(0x3903,0x12);
write_cmos_sensor(0x3905,0x2B);
write_cmos_sensor(0x3906,0x01);
write_cmos_sensor(0x3907,0x12);
write_cmos_sensor(0x3909,0x2B);
write_cmos_sensor(0x390A,0x02);
write_cmos_sensor(0x390B,0x12);
write_cmos_sensor(0x3911,0x00);
//write_cmos_sensor(0x0100,0x01);

}	/*	preview_setting  */
static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);
    if(enable)
	{   // enable color bar
		//test_pattern_flag=TRUE;
      //  write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
      //  write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
     //   write_cmos_sensor(0x0601, 0x02);  // color bar test pattern
    }
	else
	{
		//test_pattern_flag=FALSE;
      //  write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
	LOG_INF("[IMX766MIPI] exit IMX766MIPISetTestPatternMode function \n");
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

extern int nstart_get_cmdline_lcm_state(void);
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	int checkgmslcm = nstart_get_cmdline_lcm_state();
	
	if(checkgmslcm >= 0){
	    LOG_INF("Read sensor id checkgmslcm=%d",checkgmslcm);
	}
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
      *sensor_id = ((read_cmos_sensor_8(0x0016) << 8)| read_cmos_sensor_8(0x0017));
			if (*sensor_id == IMX766_SENSOR_ID && current_sensor_idx == 1 && checkgmslcm == 0) {
				*sensor_id = IMX766G_SENSOR_ID;;
				LOG_INF("IMX766g,MIPI 4LANE i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("IMX766g Read sensor id fail, write id:0x%x id: 0x%x current_sensor_idx==%d\n", imgsensor.i2c_write_id,*sensor_id,current_sensor_idx);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if ((*sensor_id != imgsensor_info.sensor_id) || (checkgmslcm!= 0) || (current_sensor_idx != 1)) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		LOG_INF("IMX766g teric fail");
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	int checkgmslcm = nstart_get_cmdline_lcm_state();
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
      sensor_id = ((read_cmos_sensor_8(0x0016) << 8)| read_cmos_sensor_8(0x0017));
			if (sensor_id == IMX766_SENSOR_ID && current_sensor_idx == 1 && checkgmslcm == 0) {
				sensor_id = IMX766G_SENSOR_ID;
				LOG_INF("IMX766g,MIPI 4LANE i2c write id: 0x%x, sensor id: 0x%x current_sensor_idx=%d\n", imgsensor.i2c_write_id,sensor_id,current_sensor_idx);
				break;
			}
			LOG_INF("IMX766g,MIPI 4LANE Read sensor id fail, write id:0x%x id: 0x%x current_sensor_idx= %d\n", imgsensor.i2c_write_id,sensor_id,current_sensor_idx);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	
	if ((imgsensor_info.sensor_id != sensor_id) || (checkgmslcm != 0) || (current_sensor_idx != 1))
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	  sensor_init();


    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en = KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/
  //mdelay(100);
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		set_test_pattern_mode(TRUE);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.test_pattern = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
	}

	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /* capture() */
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
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting(); //normal_video_setting(imgsensor.current_fps); //mars

	//set_mirror_flip(sensor_config_data->SensorImageMirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting(); //hs_video_setting(); //mars

	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting(); //slim_video_setting(); //mars

    return ERROR_NONE;
}   /*  slim_video   */

/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    custom1_setting();

    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();

    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();

    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();

    return ERROR_NONE;
}   /*  Custom4   */


static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();

    return ERROR_NONE;
}   /*  Custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet*/
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
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame begin
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
    // Antaiui <AI_BSP_CAM> <xieht> <2021-03-12> frame time delay frame end
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->PDAF_Support = 7;
    sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR, 4:four-cell mVHDR*/
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

            break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */

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
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
            break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
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
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength):0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength):0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len = 4;
            break;
            //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request begin
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
        case SENSOR_FEATURE_GET_BINNING_TYPE:
            *feature_return_para_32 = 1;
            pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
                    *feature_return_para_32);
            *feature_para_len = 4;
            break;
        case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
            /*
             * 1, if driver support new sw frame sync
             * set_shutter_frame_length() support third para auto_extend_en
             */
            pr_info("SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO \n");
            *(feature_data + 1) = 0;
            /* margin info by scenario */
            *(feature_data + 2) = imgsensor_info.margin;
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
                case MSDK_SCENARIO_ID_CUSTOM1:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom1.pclk;
                    break;
                case MSDK_SCENARIO_ID_CUSTOM2:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom2.pclk;
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
                case MSDK_SCENARIO_ID_CUSTOM1:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom1.framelength << 16)
                        + imgsensor_info.custom1.linelength;
                    break;
                case MSDK_SCENARIO_ID_CUSTOM2:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom2.framelength << 16)
                        + imgsensor_info.custom2.linelength;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                        + imgsensor_info.pre.linelength;
                    break;
            }
            break;
            //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request end
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len = 4;
            break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
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
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
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
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO2 scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[1],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
        case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
            //set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
            break;
        case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
            LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
            streaming_control(KAL_FALSE);
            break;
        case SENSOR_FEATURE_SET_STREAMING_RESUME:
            LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
            if (*feature_data != 0)
                set_shutter(*feature_data);
            streaming_control(KAL_TRUE);
            break;
        case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
            LOG_INF("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
            memcpy(feature_return_para_32, &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
            break;
        case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
            LOG_INF("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
            *feature_return_para_32 =  imgsensor.current_ae_effective_frame;
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
                    case MSDK_SCENARIO_ID_CUSTOM1:
                        rate = imgsensor_info.custom1.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_CUSTOM2:
                        rate = imgsensor_info.custom2.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                        rate = imgsensor_info.pre.mipi_pixel_rate;
                        break;
                    default:
                        rate = 0;
                        break;
                }
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
            }
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("hdr enable :%d\n", *feature_data_32);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.hdr_mode = (UINT8)*feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER:
            /*LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, SE=%d, ME=%d\n",
                    (UINT16) *feature_data,
                    (UINT16) *(feature_data + 1),
                    (UINT16) *(feature_data + 2));
            hdr_write_tri_shutter((UINT16)*feature_data,
                    (UINT16)*(feature_data+1),
                    (UINT16)*(feature_data+2));*/
            break;
        case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
            /*LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
                    (UINT16) *feature_data,
                    (UINT16) *(feature_data + 1),
                    (UINT16) *(feature_data + 2));
            hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
            */break;
						#if 0 
        case SENSOR_FEATURE_SET_HDR_MOTION:
            hdr_set_motion((UINT32)*feature_data);
            break;

						#endif
		case SENSOR_FEATURE_GET_VC_INFO2: {
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n",
							(UINT16) (*feature_data));
		pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
		get_vc_info_2(pvcinfo2, *feature_data_32);
		}
		break;
		
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			/* LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", *feature_data); */
			/* PDAF capacity enable or not, 2p8 only full size support PDAF */
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:	

			case MSDK_SCENARIO_ID_CUSTOM1:			
			
	
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
					
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			}
			break;
				
				
			case SENSOR_FEATURE_GET_PDAF_INFO:
			pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
				(UINT16) *feature_data);
			PDAFinfo =
			  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: 
			case MSDK_SCENARIO_ID_SLIM_VIDEO:  
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW: 
			default:
				memcpy((void *)PDAFinfo,
					(void *)&imgsensor_pd_info_binning,
					sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			}
			break;

		case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
			pr_debug("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
				(*feature_para_len));
			imx766g_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
						   , feature_data_16);
			break;
		case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
			pr_debug("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
				(*feature_para_len));
			imx766g_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
						   , feature_data_16);
			break;

		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			imgsensor.pdaf_mode= *feature_data_16;
			break;
		
        case SENSOR_FEATURE_SET_AWB_GAIN:
            //set_awb_gain(pSetSensorAWB);
            break;

      
        case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
            /*
             * SENSOR_VHDR_MODE_NONE  = 0x0,
             * SENSOR_VHDR_MODE_IVHDR = 0x01,
             * SENSOR_VHDR_MODE_MVHDR = 0x02,
             * SENSOR_VHDR_MODE_ZVHDR = 0x09
             * SENSOR_VHDR_MODE_4CELL_MVHDR = 0x0A
             */
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                case MSDK_SCENARIO_ID_CUSTOM1:
                case MSDK_SCENARIO_ID_CUSTOM2:
                default:
                    *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
                    break;
            }
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n"
                    , *feature_data, *(feature_data+1));
            break;

        default:
            break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX766G_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* IMX766G_MIPI_RAW_SensorInit */
