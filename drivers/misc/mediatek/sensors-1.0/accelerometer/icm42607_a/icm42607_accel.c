/* 
 * ICM42607 sensor driver
 * Copyright (C) 2018 Invensense, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#endif

#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <cust_acc.h>
#include "accel.h"
#include "icm42607_register.h"
#include "icm42607_share.h"
#include "icm42607_accel.h"

#define ICM42607_ACCEL_DEV_NAME    "ICM42607_ACCEL"
#define MISC_DEVICE_IOCTL  0

static bool icm42607_accel_first_enable = false;
static int icm42607_accel_discardcount = 0;
extern void app_get_g_sensor_name(char *g_name);

struct icm42607_accel_i2c_data {
    struct i2c_client    *client;
#if ICM42607_USE_INTERRUPT
	/* interrupt */
	struct work_struct	eint_work;
	int			irq;
#endif
    struct acc_hw    *hw;
    struct hwmsen_convert    cvt;
    /*misc*/
    atomic_t    trace;
    atomic_t    suspend;
    atomic_t    selftest;
    atomic_t    is_enabled;
    /*data*/
    s16    cali_sw[ICM42607_AXIS_NUM+1];
    s16    data[ICM42607_AXIS_NUM+1];
    u8    offset[ICM42607_AXIS_NUM+1];
};

static int icm42607_accel_init_flag =  -1;
struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;
struct i2c_client *icm42607_accel_i2c_client;
static struct icm42607_accel_i2c_data *obj_i2c_data;
#ifdef ICM42607_SELFTEST
static char selftestRes[8] = { 0 };
#define SELF_TEST_ACC_BW_IND        BIT_ACCEL_UI_LNM_BW_34HZ
#endif

// +/-4G as default
static int g_icm42607_accel_sensitivity = ICM42607_ACCEL_DEFAULT_SENSITIVITY;
static int icm42607_accel_local_init(void);
static int icm42607_accel_remove(void);
static int icm42607_accel_set_delay(u64 ns);
static int icm42607_accel_get_data(int *x , int *y, int *z, int *status);
static int icm42607_accel_enable_nodata(int en);

static struct acc_init_info icm42607_accel_init_info = {
    .name = ICM42607_ACCEL_DEV_NAME,
    .init = icm42607_accel_local_init,
    .uninit = icm42607_accel_remove,
};

#define DEBUG_MAX_CHECK_COUNT    32
#define DEBUG_MAX_IOCTL_CMD_COUNT    168
#define DEBUG_MAX_LOG_STRING_LENGTH    512
#define DEBUG_IOCTL_CMD_DEFAULT    0xFF
#define DEBUG_PRINT_ELEMENT_COUNT    8
#define DEBUG_PRINT_CHAR_COUNT_PER_LINE    24

struct icm42607_debug_data {
    int    nTestCount[DEBUG_MAX_CHECK_COUNT];
    int    nUsedTestCount;
    unsigned char    ucCmdLog[DEBUG_MAX_IOCTL_CMD_COUNT];
    int    nLogIndex;
    unsigned char    ucPrevCmd;
    int    nDriverState;
    char    strDebugDump[DEBUG_MAX_LOG_STRING_LENGTH];
    int    nStrIndex;
};

struct icm42607_debug_data icm42607_accel_debug;

static void initDebugData(void)
{
    memset(&icm42607_accel_debug, 0x00, sizeof(icm42607_accel_debug));
    icm42607_accel_debug.ucPrevCmd = DEBUG_IOCTL_CMD_DEFAULT;
    memset(&(icm42607_accel_debug.ucCmdLog[0]),
        DEBUG_IOCTL_CMD_DEFAULT, DEBUG_MAX_IOCTL_CMD_COUNT);
}

#if MISC_DEVICE_IOCTL
static void addCmdLog(unsigned int cmd)
{
    unsigned char converted_cmd = DEBUG_IOCTL_CMD_DEFAULT;

    switch (cmd) {
        case GSENSOR_IOCTL_INIT:    converted_cmd = 0; break;
        case GSENSOR_IOCTL_READ_CHIPINFO:    converted_cmd = 1; break;
        case GSENSOR_IOCTL_READ_SENSORDATA:    converted_cmd = 2; break;
        case GSENSOR_IOCTL_READ_OFFSET:    converted_cmd = 3; break;
        case GSENSOR_IOCTL_READ_GAIN:    converted_cmd = 4; break;
        case GSENSOR_IOCTL_READ_RAW_DATA:    converted_cmd = 5; break;
        case GSENSOR_IOCTL_SET_CALI:    converted_cmd = 6; break;
        case GSENSOR_IOCTL_GET_CALI:    converted_cmd = 7; break;
        case GSENSOR_IOCTL_CLR_CALI:    converted_cmd = 8; break;
    }
    icm42607_accel_debug.nDriverState = 0;
    if ( (icm42607_accel_debug.nLogIndex < DEBUG_MAX_IOCTL_CMD_COUNT) &&
        (converted_cmd != DEBUG_IOCTL_CMD_DEFAULT) ) {
        icm42607_accel_debug.ucCmdLog[icm42607_accel_debug.nLogIndex++]
            = converted_cmd;
        icm42607_accel_debug.ucPrevCmd = converted_cmd;
    }
}

static void printCmdLog(void)
{
    int i;
    int nRowCount = 0;

    nRowCount = (int)(icm42607_accel_debug.nLogIndex / 8);
    for (i = 0; i < nRowCount + 1; i++) {
        sprintf(&icm42607_accel_debug.strDebugDump[i * 24],
            "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
            icm42607_accel_debug.ucCmdLog[i*8+0],
            icm42607_accel_debug.ucCmdLog[i*8+1],
            icm42607_accel_debug.ucCmdLog[i*8+2],
            icm42607_accel_debug.ucCmdLog[i*8+3],
            icm42607_accel_debug.ucCmdLog[i*8+4],
            icm42607_accel_debug.ucCmdLog[i*8+5],
            icm42607_accel_debug.ucCmdLog[i*8+6],
            icm42607_accel_debug.ucCmdLog[i*8+7]);
    }
    GSE_INFO("ACCEL_IOCTL_CMD_LOG : \n%s\t%d\n",
        icm42607_accel_debug.strDebugDump,icm42607_accel_debug.nLogIndex);
}
#endif

static int icm42607_accel_SetFullScale(struct i2c_client *client, u8 accel_fsr)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm42607_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GSE_ERR("read fsr register err!\n");
        return ICM42607_ERR_BUS;
    }
    /* clear FSR bit */
    databuf[0] &= ~BIT_ACCEL_FSR;
    databuf[0] |= accel_fsr << SHIFT_ACCEL_FS_SEL;
    g_icm42607_accel_sensitivity = ICM42607_ACCEL_MIN_SENSITIVITY << accel_fsr;
    res = icm42607_share_write_register(REG_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GSE_ERR("write fsr register err!\n");
        return ICM42607_ERR_BUS;
    }
	GSE_INFO("g_icm42607_accel_sensitivity %d ,write %x\n",g_icm42607_accel_sensitivity,databuf[0]);
#ifdef DEBUG
	res = icm42607_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
    if (res < 0) {
        GSE_ERR("read fsr register err!\n");
        return ICM42607_ERR_BUS;
    }
	GSE_INFO("READ fs REG_ACCEL_CONFIG0 %x\n",databuf[0]);
#endif
    return ICM42607_SUCCESS;
}

static int icm42607_accel_SetFilter(struct i2c_client *client, u8 accel_filter)
{
    u8 databuf[2] = {0};
    int res = 0;

    res = icm42607_share_read_register(REG_ACCEL_CONFIG1, databuf, 1);
    if (res < 0) {
        GSE_ERR("read filter register err!\n");
        return ICM42607_ERR_BUS;
    }
    /*  clear filter bit */
    databuf[0] &= 0x0;
    databuf[0] |= accel_filter;
    res = icm42607_share_write_register(REG_ACCEL_CONFIG1, databuf, 1);
    if (res < 0) {
        GSE_ERR("write filter register err!\n");
        return ICM42607_ERR_BUS;
    }
    return ICM42607_SUCCESS;
}

#ifdef ICM42607_SELFTEST
static int icm42607_accel_ReadSensorDataDirect(struct i2c_client *client,
    s16 data[ICM42607_AXIS_NUM])
{
    char databuf[6];
    int i;
    int res = 0;

    if (client == NULL)
        return ICM42607_ERR_INVALID_PARAM;
    res = icm42607_share_read_register(REG_ACCEL_DATA_X0_UI, databuf,
        ICM42607_DATA_LEN);
    if (res < 0) {
        GSE_ERR("read accelerometer data error\n");
        return ICM42607_ERR_BUS ;
    }
    /*  convert 8-bit to 16-bit */
    for (i = 0; i < ICM42607_AXIS_NUM; i++)
        data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    return ICM42607_SUCCESS;
}

#else
	#if MISC_DEVICE_IOCTL
static int icm42607_accel_ReadSensorDataDirect(struct i2c_client *client,
    s16 data[ICM42607_AXIS_NUM])
{
    char databuf[6];
    int i;
    int res = 0;

    if (client == NULL)
        return ICM42607_ERR_INVALID_PARAM;
    res = icm42607_share_read_register(REG_ACCEL_DATA_X0_UI, databuf,
        ICM42607_DATA_LEN);
    if (res < 0) {
        GSE_ERR("read accelerometer data error\n");
        return ICM42607_ERR_BUS ;
    }
    /*  convert 8-bit to 16-bit */
    for (i = 0; i < ICM42607_AXIS_NUM; i++)
        data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
    return ICM42607_SUCCESS;
}
 	#endif
 #endif
 
static int icm42607_accel_ReadSensorData(struct i2c_client *client,
    char *buf, int bufsize)
{
    char databuf[6];
    int  data[3];
	int  temp_buff[3];
    int  i = 0;
    int res = 0;
	static int discard_count = 0;
    struct icm42607_accel_i2c_data * obj = i2c_get_clientdata(client);

    if (client == NULL)
        return ICM42607_ERR_INVALID_PARAM;
    res = icm42607_share_read_register(REG_ACCEL_DATA_X0_UI, databuf,
        ICM42607_DATA_LEN);
    if (res < 0) {
        GSE_ERR("read accelerometer data error\n");
        return ICM42607_ERR_BUS;
    }
    /*  convert 8-bit to 16-bit */
    for (i = 0; i < ICM42607_AXIS_NUM; i++) {
        obj->data[i] = be16_to_cpup((__be16 *) (&databuf[i*2]));
		// Unit Translation
		temp_buff[i] = obj->data[i] * GRAVITY_EARTH_1000  / g_icm42607_accel_sensitivity;

    }
    /* orientation translation lower (sensor) 
    --> upper (device) & unit translation */
 	#ifdef DEBUG

		GSE_INFO("ACC Data 1- %d %d %d\n", 
					temp_buff[ICM42607_AXIS_X], 
					temp_buff[ICM42607_AXIS_Y], 
					temp_buff[ICM42607_AXIS_Z]);

		GSE_INFO("offset - %d %d %d\n", 
					obj->cali_sw[ICM42607_AXIS_X], 
					obj->cali_sw[ICM42607_AXIS_Y], 
					obj->cali_sw[ICM42607_AXIS_Z]);

		GSE_INFO("ACC map x- %d %d \n", 
					obj->cvt.map[ICM42607_AXIS_X], 
					obj->cvt.sign[ICM42607_AXIS_X]);
		GSE_INFO("ACC map y- %d %d \n", 
					obj->cvt.map[ICM42607_AXIS_Y], 
					obj->cvt.sign[ICM42607_AXIS_Y]);	
		GSE_INFO("ACC map z- %d %d \n", 
					obj->cvt.map[ICM42607_AXIS_Z], 
					obj->cvt.sign[ICM42607_AXIS_Z]);	
	#endif	

	/* Add Cali */
	//obj->data[i] += obj->cali_sw[i];
    temp_buff[ICM42607_AXIS_X]+= obj->cali_sw[ICM42607_AXIS_X];
	temp_buff[ICM42607_AXIS_Y]+= obj->cali_sw[ICM42607_AXIS_Y];
  	temp_buff[ICM42607_AXIS_Z]+= obj->cali_sw[ICM42607_AXIS_Z];

    /* 3 . Orientation Translation Lower (sensor) --> Upper (Device) */
	data[obj->cvt.map[ICM42607_AXIS_X]] = obj->cvt.sign[ICM42607_AXIS_X] * temp_buff[ICM42607_AXIS_X];
	data[obj->cvt.map[ICM42607_AXIS_Y]] = obj->cvt.sign[ICM42607_AXIS_Y] * temp_buff[ICM42607_AXIS_Y];
	data[obj->cvt.map[ICM42607_AXIS_Z]] = obj->cvt.sign[ICM42607_AXIS_Z] * temp_buff[ICM42607_AXIS_Z];

	if (icm42607_accel_first_enable){
		GSE_INFO("ACC Data 2J- %d %d %d\n", 
					data[ICM42607_AXIS_X], 
					data[ICM42607_AXIS_Y], 
					data[ICM42607_AXIS_Z]);
		if(discard_count < icm42607_accel_discardcount){	
			discard_count ++;
			GSE_ERR("discard times %d\n",discard_count);
			return ICM42607_ERR_INVALID_PARAM;
		}
		else{
			discard_count =0;
			icm42607_accel_first_enable = false;
		}
	}		
	sprintf(buf, "%04x %04x %04x",
        data[ICM42607_AXIS_X],
        data[ICM42607_AXIS_Y],
        data[ICM42607_AXIS_Z]);
    if (atomic_read(&obj->trace)) {
        GSE_INFO("Accelerometer data - %04x %04x %04x\n",
            data[ICM42607_AXIS_X],
            data[ICM42607_AXIS_Y],
            data[ICM42607_AXIS_Z]);
    }
    return ICM42607_SUCCESS;
}

#if MISC_DEVICE_IOCTL
static int icm42607_accel_ReadOffsetData(struct i2c_client *client, char *buf)
{
    struct icm42607_accel_i2c_data * obj = i2c_get_clientdata(client);

    if (client == NULL)
        return ICM42607_ERR_INVALID_PARAM;
    /* offset value [0, 0, 0] */
    sprintf(buf, "%04x %04x %04x",
        obj->offset[ICM42607_AXIS_X],
        obj->offset[ICM42607_AXIS_Y],
        obj->offset[ICM42607_AXIS_Z]);
    return ICM42607_SUCCESS;
}

static int icm42607_accel_ReadGain(struct i2c_client *client,
    struct GSENSOR_VECTOR3D *gsensor_gain)
{
    if (client == NULL)
        return ICM42607_ERR_INVALID_PARAM;
    gsensor_gain->x = gsensor_gain->y = gsensor_gain->z =
        g_icm42607_accel_sensitivity;
    return ICM42607_SUCCESS;
}

static int icm42607_accel_ReadRawData(struct i2c_client * client, char * buf)
{	
    int res = 0;
    s16 data[ICM42607_AXIS_NUM] = { 0, 0, 0 };

    res = icm42607_accel_ReadSensorDataDirect(client, data);
    if (res < 0) {
        GSE_ERR("read accelerometer raw data  error\n");
        return ICM42607_ERR_BUS;
    }
    /* sensor raw data direct read from sensor register	
    no orientation translation, no unit translation */
    sprintf(buf, "%04x %04x %04x",
        data[ICM42607_AXIS_X],
        data[ICM42607_AXIS_Y],
        data[ICM42607_AXIS_Z]);
    return ICM42607_SUCCESS;
}
#endif

static int icm42607_accel_ResetCalibration(struct i2c_client *client)
{
    struct icm42607_accel_i2c_data *obj = i2c_get_clientdata(client);

    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return ICM42607_SUCCESS;
}

static int icm42607_accel_ReadCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    struct icm42607_accel_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;
    int cali[ICM42607_AXIS_NUM];
	
	cali[obj->cvt.map[ICM42607_AXIS_X]] = obj->cvt.sign[ICM42607_AXIS_X]*obj->cali_sw[ICM42607_AXIS_X];
    	cali[obj->cvt.map[ICM42607_AXIS_Y]] = obj->cvt.sign[ICM42607_AXIS_Y]*obj->cali_sw[ICM42607_AXIS_Y];
    	cali[obj->cvt.map[ICM42607_AXIS_Z]] = obj->cvt.sign[ICM42607_AXIS_Z]*obj->cali_sw[ICM42607_AXIS_Z];
	sensor_data->x = cali[ICM42607_AXIS_X] ;
	sensor_data->y = cali[ICM42607_AXIS_Y] ;
	sensor_data->z = cali[ICM42607_AXIS_Z] ;

    if (atomic_read(&obj->trace)) {
        GSE_INFO("Accel ReadCalibration:[sensor_data:%5d %5d %5d]\n",
            sensor_data->x, sensor_data->y, sensor_data->z);
        GSE_INFO("Accel ReadCalibration:[cali_sw:%5d %5d %5d]\n",
            obj->cali_sw[ICM42607_AXIS_X],
            obj->cali_sw[ICM42607_AXIS_Y],
            obj->cali_sw[ICM42607_AXIS_Z]);
    }
    return res;
}

static int icm42607_accel_write_rel_calibration(struct icm42607_accel_i2c_data *obj, int dat[ICM_ACCEL_AXES_NUM])
{
   	obj->cali_sw[ICM42607_AXIS_X] = obj->cvt.sign[ICM42607_AXIS_X]*dat[obj->cvt.map[ICM42607_AXIS_X]];
    obj->cali_sw[ICM42607_AXIS_Y] = obj->cvt.sign[ICM42607_AXIS_Y]*dat[obj->cvt.map[ICM42607_AXIS_Y]];
	obj->cali_sw[ICM42607_AXIS_Z] = obj->cvt.sign[ICM42607_AXIS_Z]*dat[obj->cvt.map[ICM42607_AXIS_Z]];
#ifdef DEBUG
        GSE_INFO("test (%5d, %5d, %5d))\n",obj->cali_sw[ICM42607_AXIS_X],obj->cali_sw[ICM42607_AXIS_Y],obj->cali_sw[ICM42607_AXIS_Z]);		
#endif
    return 0;
}

static int icm42607_accel_WriteCalibration(struct i2c_client *client,
    struct SENSOR_DATA *sensor_data)
{
    struct icm42607_accel_i2c_data *obj = i2c_get_clientdata(client);
    int cali[ICM42607_AXIS_NUM];
    int res = 0;

    cali[obj->cvt.map[ICM42607_AXIS_X]] = obj->cvt.sign[ICM42607_AXIS_X]*obj->cali_sw[ICM42607_AXIS_X];
    cali[obj->cvt.map[ICM42607_AXIS_Y]] = obj->cvt.sign[ICM42607_AXIS_Y]*obj->cali_sw[ICM42607_AXIS_Y];
	cali[obj->cvt.map[ICM42607_AXIS_Z]] = obj->cvt.sign[ICM42607_AXIS_Z]*obj->cali_sw[ICM42607_AXIS_Z];
	cali[ICM42607_AXIS_X] += sensor_data ->x;
	cali[ICM42607_AXIS_Y] += sensor_data ->y;
	cali[ICM42607_AXIS_Z] += sensor_data ->z;
#ifdef DEBUG
	GSE_INFO("write accel calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n",
			sensor_data ->x, sensor_data ->y, sensor_data ->z,
			cali[ICM42607_AXIS_X],cali[ICM42607_AXIS_Y],cali[ICM42607_AXIS_Z]);
#endif
	return icm42607_accel_write_rel_calibration(obj, cali);

    return res;
}

#ifdef ICM42607_SELFTEST
static int  icm42607_accel_InitSelfTest(struct i2c_client * client)
{
    int res = 0;

    /* soft reset */
    res = icm42607_share_ChipSoftReset();
    if (res != ICM42607_SUCCESS)
        return res;
    /* set power mode */
    icm42607_share_set_sensor_power_mode(ICM42607_SENSOR_TYPE_ACC,
        BIT_ACCEL_MODE_LNM);
    /* setpowermode(true) --> exit sleep */
    res = icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, true);
    if (res != ICM42607_SUCCESS)
        return res;
    /* fsr : ICM42607_ACCEL_RANGE_2G */
    res = icm42607_accel_SetFullScale(client, ICM42607_ACCEL_RANGE_2G);
    if (res != ICM42607_SUCCESS)
        return res;
    /* filter : SELF_TEST_ACC_BW_IND */
    res = icm42607_accel_SetFilter(client, SELF_TEST_ACC_BW_IND);
    if (res != ICM42607_SUCCESS)
        return res;
    /* odr : 800hz */
    res = icm42607_share_SetSampleRate(ICM42607_SENSOR_TYPE_ACC,
        1250000, true);
    if (res != ICM42607_SUCCESS)
        return res;
    /* set enable sensor */
    res = icm42607_share_EnableSensor(ICM42607_SENSOR_TYPE_ACC, true,&icm42607_accel_first_enable);
    if (res != ICM42607_SUCCESS)
        return res;
    mdelay(SELF_TEST_READ_INTERVAL_MS);
    return res;
}

static int icm42607_accel_CalcAvgWithSamples(struct i2c_client *client,
    int avg[3], int count)
{
    int res = 0;
    int i, nAxis;
    s16 sensor_data[ICM42607_AXIS_NUM];
    s32 sum[ICM42607_AXIS_NUM] = { 0, 0, 0 };

    for (i = 0; i < count; i++) {
        res = icm42607_accel_ReadSensorDataDirect(client, sensor_data);
        if (res) {
            GSE_ERR("read data fail: %d\n", res);
            return ICM42607_ERR_STATUS;
        }
        for (nAxis = 0; nAxis < ICM42607_AXIS_NUM; nAxis++)
            sum[nAxis] += sensor_data[nAxis];
        /* data register updated @1khz */
        mdelay(1);
    }
    for (nAxis = 0; nAxis < ICM42607_AXIS_NUM; nAxis++)
        avg[nAxis] = (int)(sum[nAxis] / count) * SELF_TEST_PRECISION;
    return res;
}

static bool icm42607_accel_DoSelfTest(struct i2c_client *client)
{
    int res = 0;
    int i;
    int acc_ST_on[ICM42607_AXIS_NUM], acc_ST_off[ICM42607_AXIS_NUM];
    /* index of otp_lookup_tbl */
    u8  st_code[ICM42607_AXIS_NUM];
    u16 st_otp[ICM42607_AXIS_NUM];
    bool otp_value_has_zero = false;
    bool test_result = true;
    u8 databuf[2] = {0};
    int ratio, st_res;
    int retry;

    /* acquire OTP value from OTP lookup table */
    res = icm42607_share_read_blkreg(BIT_BLK_SEL_3, REG_XA_ST_DATA, &(st_code[0]), 3);
    if (res) {
        GSE_ERR("Read selftest opt data failed: %d\n", res);
        return false;
    }
    GSE_INFO("st_code: %02x, %02x, %02x\n", st_code[0], st_code[1], st_code[2]);

    /* lookup OTP value with st_code */
    for (i = 0; i < ICM42607_AXIS_NUM; i++) {
        if (st_code[i] != 0)
            st_otp[i] = st_otp_lookup_tbl[ st_code[i] - 1];
        else {
            st_otp[i] = 0;
            otp_value_has_zero = true;
        }
    }
    /* read sensor data and calculate average values from it */
    for (retry = 0 ; retry < RETRY_CNT_SELF_TEST ; retry++ ) {
        /* read 200 samples with selftest off */
        res = icm42607_accel_CalcAvgWithSamples(client, acc_ST_off,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            GSE_ERR("Read sample data failed: %d\n", res);
            return false;
        }
        /* set selftest on */
        databuf[0] = BIT_ACCEL_ST_EN;
        res = icm42607_share_write_blkreg(BIT_BLK_SEL_1, REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            GSE_ERR("enable st accel fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);
        /* Read 200 Samples with selftest on */
        res = icm42607_accel_CalcAvgWithSamples(client, acc_ST_on,
            SELF_TEST_SAMPLE_NB);
        if (res) {
            GSE_ERR("Read data fail: %d\n", res);
            return false;
        }
        /* set selftest off */
        databuf[0] = 0x00;
        res = icm42607_share_write_blkreg(BIT_BLK_SEL_1, REG_SELF_TEST_CONFIG, databuf, 1);
        if (res) {
            GSE_ERR("disable st accel fail: %d\n", res);
            return false;
        }
        /* wait 20ms for oscillations to stabilize */
        mdelay(20);        
        /* compare calculated value with OTP value to judge success or fail */
        if (otp_value_has_zero == false) {
            /* criteria a */
            for (i = 0; i < ICM42607_AXIS_NUM; i++) {
                st_res = acc_ST_on[i] - acc_ST_off[i];
                ratio = abs(st_res / st_otp[i] - SELF_TEST_PRECISION);
                if (ratio >= SELF_TEST_ACC_SHIFT_DELTA) {
                    GSE_INFO("error accel[%d] : st_res = %d, st_otp = %d\n",
                        i, st_res, st_otp[i]);
                    test_result = false;
                }
            }
        } else {
            /* criteria b */
            for (i = 0; i < ICM42607_AXIS_NUM; i++) {
                st_res = abs(acc_ST_on[i] - acc_ST_off[i]);
                if (st_res < SELF_TEST_MIN_ACC || st_res > SELF_TEST_MAX_ACC) {
                    GSE_INFO("error accel[%d] : st_res = %d, min = %d, max = %d\n",
                        i, st_res, SELF_TEST_MIN_ACC, SELF_TEST_MAX_ACC);
                    test_result = false;
                }
            }
        }
        if (test_result)
            break;
    }
    return test_result;
}
#endif

static int icm42607_accel_init_client(struct i2c_client *client, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};
    // ADD WHO AM i detect here
    /* exit sleep mode */
	char strbuf[ICM42607_BUFSIZE];

    res	= icm42607_share_ReadChipInfo(strbuf, ICM42607_BUFSIZE);
    if (res != ICM42607_SUCCESS)
    {
        return res;	
    	}
	res = icm42607_share_read_register(REG_INT_CONFIG, databuf, 0x1);
    if (res < 0) {
        GSE_ERR("read REG_INT_CONFIG register err!\n");
        return ICM42607_ERR_BUS;
    }
	GSE_INFO("read INT_CONFIG %x!\n", databuf[0]);
    res = icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, true);
    if (res != ICM42607_SUCCESS)
        return res;
    /* set fsr ICM42607_ACCEL_RANGE_4G as default */
    res = icm42607_accel_SetFullScale(client, ICM42607_ACCEL_RANGE_4G);
    if (res != ICM42607_SUCCESS)
        return res;
#ifdef ICM42607_ACCEL_LOW_POWER_MODE
    /* set power mode */
    icm42607_share_set_sensor_power_mode(ICM42607_SENSOR_TYPE_ACC,
        BIT_ACCEL_MODE_LPM);
    /* set filter BIT_ACCEL_UI_LPM_AVG_2 as default */
    res = icm42607_accel_SetFilter(client, BIT_ACCEL_UI_LPM_AVG_2);
    if (res != ICM42607_SUCCESS)
        return res;
#else
    /* set power mode */
    icm42607_share_set_sensor_power_mode(ICM42607_SENSOR_TYPE_ACC,
        BIT_ACCEL_MODE_LNM);
    /* set filter BIT_ACCEL_UI_LNM_BW_180HZ as default */
    res = icm42607_accel_SetFilter(client, BIT_ACCEL_UI_LNM_BW_180HZ);
    if (res != ICM42607_SUCCESS)
        return res;
#endif
    /* set 5ms(200hz) sample rate */
    res = icm42607_share_SetSampleRate(ICM42607_SENSOR_TYPE_ACC,
        5000000, false);
    if (res != ICM42607_SUCCESS)
        return res;
    /* disable sensor - standby mode for accelerometer */
    res = icm42607_share_EnableSensor(ICM42607_SENSOR_TYPE_ACC, enable,&icm42607_accel_first_enable);
    if (res != ICM42607_SUCCESS)
        return res;
    /* set power mode - sleep or normal */
    res = icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, enable);
    if (res != ICM42607_SUCCESS)
        return res;
    GSE_INFO("icm42607_accel_init_client OK!\n");
    return ICM42607_SUCCESS;
}

struct acc_hw *get_cust_acc(void)
{
    return &accel_cust;
}

/*----------------------------------------------------------------------------*/
/* For  driver Interrupt routine */
#if ICM42607_USE_INTERRUPT

static void icm42607_accel_eint_work(struct work_struct * work)
{
	extern void icm42607_sc_NotifySensorData(void);

	struct icm42607_accel_i2c_data *obj = obj_i2c_data;
	
	if (obj != NULL)
	{
		icm42607_sc_NotifySensorData();
		GSE_INFO("eint work calls NotifySensorData \n");

		enable_irq(obj->irq);
	}
}

static irqreturn_t icm42607_accel_eint_handler(int irq, void *arg)
{
	struct icm42607_accel_i2c_data *obj = obj_i2c_data;

	if (obj != NULL)
	{
		/* To check scheduled timestamp you can use this code */
		// sched_start_time = sched_clock();
		disable_irq_nosync(obj->irq);
		schedule_work(&obj->eint_work);
		
	}

    return IRQ_HANDLED;
}

static int icm42607_accel_setup_eint(struct i2c_client *client)
{
//	struct platform_device *PltFmDev;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;

	struct device_node *irq_node;
	struct icm42607_accel_i2c_data *obj = obj_i2c_data;

//	u32 ints[2] = { 0, 0 };

	int res = 0;
   
	//icm42607_share_ConfigInterrupt(ICM206XX_BIT_LATCH_INT_EN,false);        //config int as pulse int and high active is default.  true is latch and false is pulse
	
    pinctrl = devm_pinctrl_get(&client->dev);

	if (IS_ERR(pinctrl)) {
		res = PTR_ERR(pinctrl);
		GSE_ERR("Cannot find accel pinctrl!\n");
	}

	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		res = PTR_ERR(pins_default);
		GSE_ERR("Cannot find accel pinctrl default!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		res = PTR_ERR(pins_cfg);
		GSE_ERR("Cannot find accel pinctrl pin_cfg!\n");
	}
        GSE_INFO("pin_ctrl success \n");
	/* eint request */
	// jonny of_find_compatible_node is not recommend to use and anyway it need to rewrite dts file to support gsensor interrupt because maybe platform do not support gsensor defaultly 
	//	irq_node = of_find_compatible_node(NULL, NULL, "mediatek,gsensor");	
	irq_node = client->dev.of_node;

	if (irq_node)
	{
//		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
//		gpio_request(ints[0], "gsensor");
//		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);

		obj->irq = irq_of_parse_and_map(irq_node, 0);

		GSE_INFO("irq = %d\n", obj->irq);
		if (!obj->irq) {
			GSE_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}

		//res = request_irq(obj->irq, icm206xx_accel_eint_handler, IRQF_TRIGGER_NONE, "gsensor-eint", NULL);
		res = request_irq(obj->irq, icm42607_accel_eint_handler, IRQF_TRIGGER_RISING, "gsensor-eint", NULL);
		if(res) {
			GSE_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}

		enable_irq(obj->irq);
		GSE_INFO("irq get node and require sucess \n");
	}

	return 0;
}
#endif

/*--------------------end of interrupt function  --------------------------------*/

static void icm42607_accel_power(struct acc_hw *hw, unsigned int on)
{
    /* nothing to do here, because the power of sensor is always on */
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    char strbuf[ICM42607_BUFSIZE];
	int gpio_status;
	gpio_status = gpio_get_value(4);
	GSE_INFO("read  gpio_status 4:  %d\n",gpio_status);
    icm42607_share_ReadChipInfo(strbuf, ICM42607_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = icm42607_accel_i2c_client;
    char strbuf[ICM42607_BUFSIZE];

    if (NULL == client) {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    icm42607_accel_ReadSensorData(client, strbuf, ICM42607_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, 
    const char *buf, size_t count)
{
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;
    int trace;

    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (0 == kstrtoint(buf, 16, &trace))
        atomic_set(&obj->trace, trace);
    else
        GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
    return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if (obj->hw)
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
            obj->hw->i2c_num,
            obj->hw->direction,
            obj->hw->power_id,
            obj->hw->power_vol);
    else
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    return len;
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    GSE_INFO("[%s] default direction: %d\n", __func__, obj->hw->direction);
    len = snprintf(buf, PAGE_SIZE, "default direction = %d\n", obj->hw->direction);
    return len;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, 
    const char *buf, size_t tCount)
{
    int nDirection = 0;
    int res = 0;
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;

    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = kstrtoint(buf, 10, &nDirection);
    if (res != 0) {
        if (hwmsen_get_convert(nDirection, &obj->cvt))
            GSE_ERR("ERR: fail to set direction\n");
    }
    GSE_INFO("[%s] set direction: %d\n", __func__, nDirection);
    return tCount;
}

static ssize_t show_register_dump(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;
    int i;
    u8 databuf[2] = {0};
    int res = 0;

    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    len += snprintf(buf + len, PAGE_SIZE, "Register Dump for BANK0\n");
    for(i = 0x02; i < 0x7F; i++) {
        res = icm42607_share_read_register(i, databuf, 1);
        if (res < 0) {
            GSE_ERR("read register err!\n");
            return 0;
        }
        len += snprintf(buf + len, PAGE_SIZE, "0x%02x: 0x%02x\n",
            i, databuf[0]);
    }

	len += snprintf(buf + len, PAGE_SIZE, "Register Dump for BANK1\n");
	for(i = 0x14; i <= 0x4D; i++) {
        res = icm42607_share_read_blkreg(BIT_BLK_SEL_1, i, databuf, 1);
        if (res < 0) {
            GSE_ERR("read register err!\n");
            return 0;
        }
        len += snprintf(buf + len, PAGE_SIZE, "bank1 0x%02x: 0x%02x\n",
            i, databuf[0]);
    }

	len += snprintf(buf + len, PAGE_SIZE, "Register Dump for BANK3\n");
	for(i = 0x00; i <= 0x05; i++) {
        res = icm42607_share_read_blkreg(BIT_BLK_SEL_3, i, databuf, 1);
        if (res < 0) {
            GSE_ERR("read register err!\n");
            return 0;
        }
        len += snprintf(buf + len, PAGE_SIZE, "bank3 0x%02x: 0x%02x\n",
            i, databuf[0]);
    }

    return len;
}

#ifdef ICM42607_SELFTEST
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = icm42607_accel_i2c_client;

    if (NULL == client) {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    return snprintf(buf, 8, "%s\n", selftestRes);
}

static ssize_t store_selftest_value(struct device_driver *ddri,
    const char *buf, size_t count)
{
    struct i2c_client *client = icm42607_accel_i2c_client;
    int num;
    int res = 0;

    /* check parameter values to run selftest */
    res = kstrtoint(buf, 10, &num);
    if (res != 0) {
        GSE_ERR("parse number fail\n");
        return count;
    } else if (num == 0) {
        GSE_ERR("invalid data count\n");
        return count;
    }
    /* run selftest */
    res = icm42607_accel_InitSelfTest(client);
    if (icm42607_accel_DoSelfTest(client) == true) {
        strcpy(selftestRes, "y");
        GSE_INFO("ACCEL SELFTEST : PASS\n");
    } else {
        strcpy(selftestRes, "n");
        GSE_INFO("ACCEL SELFTEST : FAIL\n");
    }
    /* selftest is considered to be called only in factory mode
    in general mode, the condition before selftest will not be recovered
    and sensor will not be in sleep mode */
    res = icm42607_accel_init_client(client, true);
    return count;
}
#endif

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, 
    store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation,
    store_chip_orientation);
static DRIVER_ATTR(regdump, S_IRUGO, show_register_dump, NULL);
#ifdef ICM42607_SELFTEST
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_selftest_value, 
    store_selftest_value);
#endif

static struct driver_attribute *icm42607_accel_attr_list[] = {
    /* chip information */
    &driver_attr_chipinfo,
    /* dump sensor data */
    &driver_attr_sensordata,
    /* trace log */
    &driver_attr_trace,
    /* chip status */
    &driver_attr_status,
    /* chip orientation information */
    &driver_attr_orientation,
    /* register dump */
    &driver_attr_regdump,
    
#ifdef ICM42607_SELFTEST
    /* run selftest when store, report selftest result when show */
    &driver_attr_selftest
#endif
};

static int icm42607_accel_create_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = 
        (int)(sizeof(icm42607_accel_attr_list)/
        sizeof(icm42607_accel_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++) {
        res = driver_create_file(driver, icm42607_accel_attr_list[idx]);
        if (0 != res) {
            GSE_ERR("driver_create_file (%s) = %d\n",
                icm42607_accel_attr_list[idx]->attr.name, res);
            break;
        }
    }
    return res;
}

static int icm42607_accel_delete_attr(struct device_driver *driver)
{
    int idx;
    int res = 0;
    int num = 
        (int)(sizeof(icm42607_accel_attr_list)/
        sizeof(icm42607_accel_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;
    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, icm42607_accel_attr_list[idx]);
    return res;
}

#if MISC_DEVICE_IOCTL
static int icm42607_accel_open(struct inode *inode, struct file *file)
{
    file->private_data = icm42607_accel_i2c_client;
    if (file->private_data == NULL) {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}

static int icm42607_accel_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long icm42607_accel_unlocked_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    struct icm42607_accel_i2c_data * obj =
        (struct icm42607_accel_i2c_data *)i2c_get_clientdata(client);
    char strbuf[ICM42607_BUFSIZE] = {0};
    void __user *data;
    long res = 0;
    struct SENSOR_DATA sensor_data;
    static struct GSENSOR_VECTOR3D gsensor_gain;

    if (_IOC_DIR(cmd) & _IOC_READ)
        res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (res) {
        GSE_ERR("access error: %08X, (%2d, %2d)\n",
            cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }
    addCmdLog(cmd);
    printCmdLog();
    switch (cmd) {
    case GSENSOR_IOCTL_INIT:
        icm42607_share_ChipSoftReset();
        icm42607_accel_init_client(client, true);
        break;
    case GSENSOR_IOCTL_READ_CHIPINFO:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm42607_share_ReadChipInfo(strbuf, ICM42607_BUFSIZE);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EINVAL;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_SENSORDATA:
        data = (void __user *) arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm42607_accel_ReadSensorData(client, strbuf, ICM42607_BUFSIZE);
        if (copy_to_user(data, strbuf, sizeof(strbuf))) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_OFFSET:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm42607_accel_ReadOffsetData(client, strbuf);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_GAIN:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm42607_accel_ReadGain(client, &gsensor_gain);
        if (copy_to_user(data, &gsensor_gain,
            sizeof(struct GSENSOR_VECTOR3D))) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_RAW_DATA:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        icm42607_accel_ReadRawData(client, strbuf);
        if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_SET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
            res = -EFAULT;
            break;
        }
        if (atomic_read(&obj->suspend)) {
            GSE_ERR("Perform calibration in suspend state!!\n");
            res = -EINVAL;
        } else {
            GSE_INFO("accel set cali:[%5d %5d %5d]\n", 
                sensor_data.x, sensor_data.y, sensor_data.z);
            res = icm42607_accel_WriteCalibration(client, &sensor_data);
        }
        break;
    case GSENSOR_IOCTL_GET_CALI:
        data = (void __user *)arg;
        if (data == NULL) {
            res = -EINVAL;
            break;
        }
        res = icm42607_accel_ReadCalibration(client, &sensor_data);
        if (res)
            break;
        if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
            res = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_CLR_CALI:
        res = icm42607_accel_ResetCalibration(client);
        break;
    default:
        GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
        res = -ENOIOCTLCMD;
    }
    return res;
}

#ifdef CONFIG_COMPAT
static long icm42607_accel_compat_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
    long res = 0;
    void __user *arg32 = compat_ptr(arg);

    if(!file->f_op || !file->f_op->unlocked_ioctl){
        GSE_ERR("compat_ion_ioctl file has no f_op\n");
        GSE_ERR("or no f_op->unlocked_ioctl.\n");
        return -ENOTTY;
    }
    switch (cmd) {
    case COMPAT_GSENSOR_IOCTL_INIT:
    case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO: 
    case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
    case COMPAT_GSENSOR_IOCTL_READ_OFFSET:
    case COMPAT_GSENSOR_IOCTL_READ_GAIN:
    case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
    case COMPAT_GSENSOR_IOCTL_SET_CALI:
    case COMPAT_GSENSOR_IOCTL_GET_CALI:
    case COMPAT_GSENSOR_IOCTL_CLR_CALI:
        if (arg32 == NULL) {
            GSE_ERR("invalid argument.");
            return -EINVAL;
        }
        res = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
        break;
    default:
        GSE_ERR("%s not supported = 0x%04x", __func__, cmd);
        return -ENOIOCTLCMD;
    }
    return res;
}
#endif

static const struct file_operations icm42607_accel_fops = {
    .open = icm42607_accel_open,
    .release = icm42607_accel_release,
    .unlocked_ioctl = icm42607_accel_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = icm42607_accel_compat_ioctl,
#endif
};

static struct miscdevice icm42607_accel_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &icm42607_accel_fops,
};
#endif 

/************************* For MTK new factory mode ************************************/

static int icm42607_accel_factory_do_self_test(void)
{
        return 0;
}

static int icm42607_accel_factory_get_cali(int32_t data[3])
{
        int err;
	struct icm42607_accel_i2c_data *priv = obj_i2c_data;
        struct SENSOR_DATA  sensor_data;
	GSE_INFO();
	
        err = icm42607_accel_ReadCalibration(priv->client, &sensor_data);
        if (err) {
                GSE_INFO("icm42607_accel_ReadCalibration failed!\n");
                return -1;
        }
        data[0] = sensor_data.x ;
        data[1] = sensor_data.y ;
        data[2] = sensor_data.z ;
        return 0;
}

static int icm42607_accel_factory_set_cali(int32_t data[3])
{
        int err = 0;
	struct icm42607_accel_i2c_data *priv = obj_i2c_data;
	struct SENSOR_DATA  sensor_data;
        GSE_INFO();
        GSE_INFO("accel set cali:[%5d %5d %5d]\n", data[0], data[1], data[2]);
        sensor_data.x = data[0] ;
        sensor_data.y = data[1] ;
        sensor_data.z = data[2] ;	
        err = icm42607_accel_WriteCalibration(priv->client, &sensor_data);
        if (err) {
               GSE_INFO("icm42607_accel_WriteCalibration failed!\n");
                return -1;
        }
        return 0;
}

static int icm42607_accel_factory_enable_calibration(void)
{
        return 0;
}
static int icm42607_accel_factory_clear_cali(void)
{
        int err = 0;
        struct icm42607_accel_i2c_data *priv = obj_i2c_data;
		GSE_INFO();
        err = icm42607_accel_ResetCalibration(priv->client);
        if (err) {
                GSE_INFO("icm206xx_accel_factory_clear_cali failed!\n");
                return -1;
        }
        return 0;
}

static int icm42607_accel_factory_get_raw_data(int32_t data[3])
{
        
        GSE_INFO("do not support raw data now!\n");
        return 0;
}

static int icm42607_accel_factory_get_data(int32_t data[3], int *status)
{
        int res;	
        res =  icm42607_accel_get_data(&data[0], &data[1], &data[2], status);
	GSE_INFO("%s  %d %d %d!\n", __func__,data[0], data[1], data[2]);
	return res;

}

static int icm42607_accel_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
        int err;

        err = icm42607_accel_enable_nodata(enabledisable == true ? 1 : 0);
        if (err) {
                GSE_INFO("%s enable accel sensor failed!\n", __func__);
                return -1;
        }
        return 0;
}

static struct accel_factory_fops icm42607_accel_factory_fops = {
        .enable_sensor =icm42607_accel_factory_enable_sensor,
        .get_data = icm42607_accel_factory_get_data,
        .get_raw_data =icm42607_accel_factory_get_raw_data,
        .enable_calibration = icm42607_accel_factory_enable_calibration,
        .clear_cali = icm42607_accel_factory_clear_cali,
        .set_cali = icm42607_accel_factory_set_cali,
        .get_cali = icm42607_accel_factory_get_cali,
        .do_self_test = icm42607_accel_factory_do_self_test,
};

static struct accel_factory_public icm42607_accel_factory_device = {
        .gain = 1,
        .sensitivity = 1,
        .fops = &icm42607_accel_factory_fops,
};

static int icm42607_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{

    return icm42607_accel_set_delay((u64)samplingPeriodNs);
}

static int icm42607_acc_flush(void)
{
    return acc_flush_report();
}

static int icm42607_accel_open_report_data(int open)
{
    /* nothing to do here for 406xx */
    return 0;
}

static int icm42607_accel_enable_nodata(int en)
{
    /* if use this type of enable , 
    gsensor only enabled but not report inputEvent to HAL */
    int res = 0;
    bool power = false;
    struct icm42607_accel_i2c_data *obj = obj_i2c_data;

    if (1 == en) {
        power = true;
		
		res = icm42607_share_EnableSensor(ICM42607_SENSOR_TYPE_ACC, power,&icm42607_accel_first_enable);
        res |= icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, power);       
    }
    if (0 == en) {
        power = false;
        res = icm42607_share_SetSampleRate(ICM42607_SENSOR_TYPE_ACC, 0, false);
        res |= icm42607_share_EnableSensor(ICM42607_SENSOR_TYPE_ACC, power, &icm42607_accel_first_enable);
        res |= icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, power);
    }
    if (res != ICM42607_SUCCESS) {
        GSE_INFO("fail!\n");
        return -1;
    }
    atomic_set(&obj->is_enabled, en);
    GSE_INFO("icm42607_accel_enable_nodata OK!\n");
    return 0;
}

static int icm42607_accel_set_delay(u64 ns)
{
    /* power mode setting in case of set_delay
    is called before enable_nodata */
    GSE_INFO("%s is called [ns:%lld]\n", __func__, ns);
    if(icm42607_share_get_sensor_power(ICM42607_SENSOR_TYPE_ACC) == false)
        icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, true);
    icm42607_share_SetSampleRate(ICM42607_SENSOR_TYPE_ACC, ns, false);

	icm42607_accel_discardcount =  30 * 1000 *1000 / (unsigned int)(ns);

	GSE_INFO("%s  discardcount num %d\n", __func__, icm42607_accel_discardcount);
    return 0;
}

static int icm42607_accel_get_data(int *x , int *y, int *z, int *status)
{
    char buff[ICM42607_BUFSIZE];
    int res = 0;

    res = icm42607_accel_ReadSensorData(obj_i2c_data->client, 
        buff, ICM42607_BUFSIZE);
	if (res < 0) {
		GSE_ERR("read accelerometer data not ready\n");
		return ICM42607_ERR_BUS;
	}	
    res = sscanf(buff, "%x %x %x", x, y, z);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static const struct i2c_device_id icm42607_accel_i2c_id[] =
    {{ICM42607_ACCEL_DEV_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {},
};
#endif

static int icm42607_accel_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct icm42607_accel_i2c_data *obj;
    struct acc_control_path ctl = {0};
    struct acc_data_path data = {0};
    int res = 0;

    GSE_INFO();
    obj = kzalloc(sizeof(*obj), GFP_KERNEL);
    if (!obj) {
        res = -ENOMEM;
        goto exit;
    }
	
	  res = get_accel_dts_func(client->dev.of_node, hw);
        if (res < 0) {
                GSE_ERR("get accel dts info fail\n");
                goto exit;
        }	
    memset(obj, 0, sizeof(struct icm42607_accel_i2c_data));
    /* hwmsen_get_convert() depend on the direction value */
    obj->hw = hw;
	client->addr = 0x68;
	obj->hw->direction = 5; // 6 1 2 3
    res = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
    if (res) {
        GSE_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }
	GSE_INFO("ACCEL_use_i2c_addr: %x\n", client->addr);
	GSE_INFO(" HW addr: %x\n", obj->hw->i2c_addr[0]);
	GSE_INFO(" direction: %d\n", obj->hw->direction);

#if ICM42607_USE_INTERRUPT
	// Interrupt
	INIT_WORK(&obj->eint_work, icm42607_accel_eint_work);
#endif
    obj_i2c_data = obj;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client, obj);
    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);
    atomic_set(&obj->is_enabled, 0);
    icm42607_accel_i2c_client = new_client;
    /* soft reset is called only in accelerometer init */
    /* do not call soft reset in gyro and step_count init */
    res = icm42607_share_ChipSoftReset();
    if (res != ICM42607_SUCCESS)
        goto exit_init_failed;
    res = icm42607_accel_init_client(new_client, false);
    if (res)
        goto exit_init_failed;

#if  ICM42607_USE_INTERRUPT
	// Configure Interrupt
	res = icm42607_accel_setup_eint(new_client);
	if (res) 
		goto exit_init_failed;
#endif
#if  MISC_DEVICE_IOCTL
    /* misc_register() for factory mode, engineer mode and so on */
    res = misc_register(&icm42607_accel_device);
    if (res) {
        GSE_ERR("icm42607_accel_device misc register failed!\n");
        goto exit_misc_device_register_failed;
    }
#endif
	res = accel_factory_device_register(&icm42607_accel_factory_device);

        if (res) {
                GSE_INFO("icm42607_accel_factory_device register failed!\n");
                goto exit_misc_device_register_failed;
        }
	
    /* create platform_driver attribute */
    res = icm42607_accel_create_attr(
        &(icm42607_accel_init_info.platform_diver_addr->driver));
    if (res) {
        GSE_ERR("icm42607_accel create attribute err = %d\n", res);
        goto exit_create_attr_failed;
    }
    /* fill the acc_control_path */
    ctl.is_use_common_factory = false;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;
    ctl.open_report_data = icm42607_accel_open_report_data;
    ctl.enable_nodata = icm42607_accel_enable_nodata;
    ctl.set_delay  = icm42607_accel_set_delay;
	ctl.batch = icm42607_acc_batch;
    ctl.flush = icm42607_acc_flush;
    /* register the acc_control_path */
    res = acc_register_control_path(&ctl);
    if (res) {
        GSE_ERR("register accel control path err\n");
        goto exit_kfree;
    }
    /* fill the acc_data_path */
    data.get_data = icm42607_accel_get_data;	
    data.vender_div = 1000;
    /* register the acc_data_path */
    res = acc_register_data_path(&data);
    if (res) {
        GSE_ERR("register accel_data_path fail = %d\n", res);
        goto exit_kfree;
    }
    icm42607_accel_init_flag = 0;
    GSE_INFO("%s: OK\n", __func__);
	app_get_g_sensor_name("icm42607");
    return 0;

exit_create_attr_failed:
#if  MISC_DEVICE_IOCTL
    misc_deregister(&icm42607_accel_device);
#endif 
	  accel_factory_device_deregister(&icm42607_accel_factory_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
exit:
    kfree(obj);
    obj = NULL;
    icm42607_accel_init_flag =  -1;
    GSE_ERR("%s: err = %d\n", __func__, res);
    return res;
}

static int icm42607_accel_i2c_remove(struct i2c_client *client)
{
    int res = 0;

    res = icm42607_accel_delete_attr(
        &(icm42607_accel_init_info.platform_diver_addr->driver));
    if (res)
        GSE_ERR("icm42607_accel_delete_attr fail: %d\n", res);
  
#if MISC_DEVICE_IOCTL
	misc_deregister(&icm42607_accel_device);
#endif
	accel_factory_device_deregister(&icm42607_accel_factory_device);

    icm42607_accel_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static int icm42607_accel_i2c_detect(struct i2c_client *client,
    struct i2c_board_info *info)
{
    strcpy(info->type, ICM42607_ACCEL_DEV_NAME);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int icm42607_accel_i2c_suspend(struct device *dev)
{
    int res = 0;
	struct i2c_client *client = to_i2c_client(dev);	
    struct icm42607_accel_i2c_data *obj = i2c_get_clientdata(client);

        if (obj == NULL) {
            GSE_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->suspend, 1);
        res = icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, false);
        if (res < 0) {
            GSE_ERR("write power control fail!\n");
            return res;
        }
        icm42607_accel_power(obj->hw, 0);
        GSE_INFO("icm42607_accel suspend ok\n");
    return res;
}

static int icm42607_accel_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct icm42607_accel_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;

    if (obj == NULL) {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    icm42607_accel_power(obj->hw, 1);
    if(atomic_read(&obj->is_enabled) == 1) {
        res = icm42607_share_SetPowerMode(ICM42607_SENSOR_TYPE_ACC, true);
    }
    if (res) {
        GSE_ERR("resume client fail!!\n");
        return res;
    }
    atomic_set(&obj->suspend, 0);
    GSE_INFO("icm42607_accel resume ok\n");
    return 0;
}

static const struct dev_pm_ops icm42607_accel_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(icm42607_accel_i2c_suspend, icm42607_accel_i2c_resume)
};
#endif

static struct i2c_driver icm42607_accel_i2c_driver = {
    .driver = {
        .name = ICM42607_ACCEL_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = accel_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
    .pm = &icm42607_accel_pm_ops,
#endif   
    },
    .probe = icm42607_accel_i2c_probe,
    .remove = icm42607_accel_i2c_remove,
    .detect = icm42607_accel_i2c_detect,
    .id_table = icm42607_accel_i2c_id,
};

static int icm42607_accel_remove(void)
{
    icm42607_accel_power(hw, 0);
    i2c_del_driver(&icm42607_accel_i2c_driver);
    return 0;
}

static int icm42607_accel_local_init(void)
{
    icm42607_accel_power(hw, 1);
    initDebugData();
    if (i2c_add_driver(&icm42607_accel_i2c_driver)) {
        GSE_ERR("add driver error\n");
        return -1;
    }
    if (-1 == icm42607_accel_init_flag)
        return -1;
    return 0;
}

static int __init icm42607_accel_init(void)
{
	GSE_INFO("%s: OK\n",  __func__);
    acc_driver_add(&icm42607_accel_init_info);
    return 0;
}

static void __exit icm42607_accel_exit(void)
{
    /* nothing to do here */
}

module_init(icm42607_accel_init);
module_exit(icm42607_accel_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm42607 accelerometer driver");
