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


#include "cust_acc.h"
#include "accel.h"
#include "icm42607_register.h"
#include "icm42607_share.h"
#include "icm42607_share_interface.h"

struct icm42607_sensor_status_info {
    bool sensor_power;
    int sample_rate;
    u8  sensor_power_mode;
};
static struct icm42607_sensor_status_info 
    icm42607_all_sensor_status_info[ICM42607_SENSOR_TYPE_MAX];

static void icm42607_set_sensor_power_mode(int paramSensor,
    u8 sensor_power_mode)
{
    icm42607_all_sensor_status_info[paramSensor].sensor_power_mode
        = sensor_power_mode;
}

static u8 icm42607_get_sensor_power_mode(int paramSensor)
{
    return icm42607_all_sensor_status_info[paramSensor].sensor_power_mode;
}

static bool icm42607_get_sensor_power(int paramSensor)
{
    return icm42607_all_sensor_status_info[paramSensor].sensor_power;
}

static bool icm42607_any_accel_based_sensor_is_on(void)
{
/* return true if any other step counter sensors are enabled except itself
    step counter sensors are 
       : ICM42607_SENSOR_TYPE_SC,
       : ICM42607_SENSOR_TYPE_SD
*/
    if (icm42607_all_sensor_status_info
        [ICM42607_SENSOR_TYPE_SC].sensor_power)
        return true;
    if (icm42607_all_sensor_status_info
        [ICM42607_SENSOR_TYPE_SD].sensor_power)
        return true;
    if (icm42607_all_sensor_status_info
        [ICM42607_SENSOR_TYPE_SMD].sensor_power)
        return true;
    return false;
}


static int icm42607_i2c_driver_setting(void)
{
	int res = 0;
    u8 databuf[2] = {0};
	u8 i2c_slew_rate =1;
	u8 spi_slew_rate =1;
	
	res = icm42607_share_read_register(REG_INTF_CONFIG1, databuf, 1);
   	if (res) {
    	GSE_ERR("Read REG_INTF_CONFIG1: %d\n", res);
    }
	//clear bit 2 ,3 of REG_INTF_CONFIG1
    databuf[0] &= ~(BIT_I3C_SDR_EN | BIT_I3C_DDR_EN);
     	
	res = icm42607_share_write_register(REG_INTF_CONFIG1, databuf, 1);
    if (res) {
      	GSE_ERR("write REG_INTF_CONFIG1: %d\n", res);
    }
	GSE_INFO("write REG_INTF_CONFIG1 %x!\n", databuf[0]);

	databuf[0] |= i2c_slew_rate <<3 | spi_slew_rate;
	res = icm42607_share_write_register(REG_DRIVE_CONFIG2, databuf, 1);
    if (res) {
      	GSE_ERR("write REG_DRIVE_CONFIG2: %d\n", res);
    }
	GSE_INFO("write REG_DRIVE_CONFIG2 %x!\n", databuf[0]);

	databuf[0] |= spi_slew_rate;
	res = icm42607_share_write_register(REG_DRIVE_CONFIG3, databuf, 1);
    if (res) {
      	GSE_ERR("write REG_DRIVE_CONFIG3: %d\n", res);
    }
	GSE_INFO("write REG_DRIVE_CONFIG3 %x!\n", databuf[0]);

	msleep(50);
	return ICM42607_SUCCESS;
}

static int icm42607_ChipSoftReset(void)
{
    u8 databuf[10];
    int res = 0;
    int i;

    memset(databuf, 0, sizeof(u8) * 10);
    /* read */
    res = icm42607_share_read_register(REG_CHIP_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        GSE_ERR("read power ctl register err!\n");
        return ICM42607_ERR_BUS;
    }
    /* set device_reset bit to do soft reset */
    databuf[0] |= BIT_SOFT_RESET;
    res = icm42607_share_write_register(REG_CHIP_CONFIG_REG, databuf, 0x1);
    if (res < 0) {
        GSE_ERR("write power ctl register err!\n");
        return ICM42607_ERR_BUS;
    }
    mdelay(100);

	res = icm42607_i2c_driver_setting();

	 if (res < 0) {
        GSE_ERR("write i2c driver config err!\n");
        return ICM42607_ERR_BUS;
    }
    /* sensor status reset */
    for (i = 0; i < ICM42607_SENSOR_TYPE_MAX; i++) {
        icm42607_all_sensor_status_info[i].sensor_power = false;
        icm42607_all_sensor_status_info[i].sample_rate = 0;
    }
    return ICM42607_SUCCESS;
}

static int icm42607_SetPowerMode(int sensor_type, bool enable)
{
    //u8 databuf[2] = {0};
    //int res = 0;
    //int i;

    if(sensor_type >= ICM42607_SENSOR_TYPE_MAX)
        return ICM42607_ERR_INVALID_PARAM;
    icm42607_all_sensor_status_info[sensor_type].sensor_power = enable;

    GSE_INFO("set power mode ok %d!\n", enable);
    return ICM42607_SUCCESS;
}

static int icm42607_InterruptConfig_INT1(void)
{
    u8 databuf[2] = {0};
    int res = 0;

	res = icm42607_share_read_register(REG_INT_CONFIG, databuf, 0x1);
    if (res < 0) {
        GSE_ERR("read REG_INT_CONFIG register err!\n");
        return ICM42607_ERR_BUS;
    }
	GSE_INFO("read INT_CONFIG %x!\n", databuf[0]);
	// config interrupt as active high, push pull , pulsed mode
	databuf[0] &= ~0x07;
    databuf[0] |= (INT_POLARITY << SHIFT_INT1_POLARITY) |
   	    (INT_DRIVE_CIRCUIT << SHIFT_INT1_DRIVE_CIRCUIT) |
        (INT_MODE << SHIFT_INT1_MODE);
	
	res = icm42607_share_write_register(REG_INT_CONFIG, databuf, 1);
	GSE_INFO("write INT_CONFIG %x!\n", databuf[0]);
    if (res < 0) {
        GSE_ERR("write interrupt config failed!\n");
        //return ICM42607_ERR_BUS;
    }
    return ICM42607_SUCCESS;
}


static int icm42607_EnableInterrupt(u8 int_type, bool enable)
{
    int res = 0;
    u8 databuf[2] = {0};
	u8 intsource[2] = {0};
	GSE_INFO("icm42607_EnableInterrupt type %d en %d!\n", int_type,enable);

    if(ICM42607_INT_TYPE_STD == int_type) {
    	res = icm42607_share_read_blkreg(BIT_BLK_SEL_1, REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		GSE_ERR("Read REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}

		if(enable == true) {
    	    intsource[0] |= BIT_STEP_DET_INT1_EN;
     	} else {
       	  	intsource[0] &= ~BIT_STEP_DET_INT1_EN;
     	}
		res = icm42607_share_write_blkreg(BIT_BLK_SEL_1, REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		GSE_ERR("write REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}
		return res;
    } else if(ICM42607_INT_TYPE_STC_OFL == int_type) {
    	res = icm42607_share_read_blkreg(BIT_BLK_SEL_1, REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		GSE_ERR("Read REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}

		if(enable == true) {
    	    intsource[0] |= BIT_STEP_CNT_OFL_INT1_EN;
     	} else {
       	  	intsource[0] &= ~BIT_STEP_CNT_OFL_INT1_EN;
     	}
		res = icm42607_share_write_blkreg(BIT_BLK_SEL_1, REG_INT_SOURCE6, intsource, 1);
    	if (res) {
      		GSE_ERR("write REG_INT_SOURCE6: %d\n", res);
        	return false;
    	}
		msleep(2);
		return res;
    } else {
    	res = icm42607_share_read_register(REG_INT_SOURCE1, databuf, 1);
		if (res < 0) {
			GSE_ERR("read REG_INT_SOURCE1 err!\n");
			return ICM42607_ERR_BUS;
		}

    	switch (int_type) {
    	case ICM42607_INT_TYPE_SMD:
#if(!ICM42607_WOM_SMD)
			if(enable == true) {
				databuf[0] |= BIT_INT_SMD_INT1_EN;
			} else {
				databuf[0] &= ~BIT_INT_SMD_INT1_EN;
			}
			break;
#endif
    	case ICM42607_INT_TYPE_WOM:
			if(enable == true) {
				databuf[0] |= BIT_INT_WOM_XYZ_INT1_EN;
			} else {
				databuf[0] &= ~BIT_INT_WOM_XYZ_INT1_EN;
			}
			break;
		default:
			GSE_ERR("interrupt tyep %x is not supported", int_type);
			return -ICM42607_ERR_INVALID_PARAM;
		}

    	res = icm42607_share_write_register(REG_INT_SOURCE1, databuf, 1);
    	if (res < 0) {
			GSE_ERR("write REG_INT_SOURCE1 err!\n");
			return ICM42607_ERR_BUS;
    	}
		GSE_INFO("write REG_INT_SOURCE1 %x \n", databuf[0]);
    	return ICM42607_SUCCESS;
    }
}

static int icm42607_EnableSensor(int sensor_type, bool enable , bool *first_enable_flag)
{
    u8 databuf[2] = {0};
    int res = 0;

    if(sensor_type >= ICM42607_SENSOR_TYPE_MAX)
        return ICM42607_ERR_INVALID_PARAM;
    
    res = icm42607_share_read_register(REG_PWR_MGMT_0, databuf, 1);
    if (res < 0) {
        GSE_ERR("read power mgmt register err!\n");
        return ICM42607_ERR_BUS;
    }
    /* gyro based sensors */
    if (sensor_type == ICM42607_SENSOR_TYPE_GYRO) {
        /* clear gyro enable */
        databuf[0] &= ~0x0C;
        if (enable == true) {
			if (false == icm42607_all_sensor_status_info[ICM42607_SENSOR_TYPE_GYRO].sensor_power)
				*first_enable_flag = true;
			else	
				*first_enable_flag = false;
			
			if (icm42607_all_sensor_status_info
                [ICM42607_SENSOR_TYPE_GYRO].sample_rate > ICM42607_LPM_MAX_RATE)
                databuf[0] |= BIT_GYRO_MODE_LNM;
            else
                databuf[0] |=
                    icm42607_get_sensor_power_mode(ICM42607_SENSOR_TYPE_GYRO);
        }
    }
    /* accel based sensors */
    if (sensor_type == ICM42607_SENSOR_TYPE_ACC ||
        sensor_type == ICM42607_SENSOR_TYPE_SC ||
        sensor_type == ICM42607_SENSOR_TYPE_SD
    ) {
        /* clear accel enable */
        databuf[0] &= ~0x03;
        if (enable == true || icm42607_any_accel_based_sensor_is_on()) {
			if ((false == icm42607_all_sensor_status_info[ICM42607_SENSOR_TYPE_ACC].sensor_power)&&  
				(false == icm42607_any_accel_based_sensor_is_on()))
				*first_enable_flag = true;
			else
				*first_enable_flag = false;
			
            if (icm42607_all_sensor_status_info
                [ICM42607_SENSOR_TYPE_ACC].sample_rate > ICM42607_LPM_MAX_RATE)
                databuf[0] |= BIT_ACCEL_MODE_LNM;
            else
                databuf[0] |=
                    icm42607_get_sensor_power_mode(ICM42607_SENSOR_TYPE_ACC);
        }
    }
    res = icm42607_share_write_register(REG_PWR_MGMT_0, databuf, 1);
    if (res < 0) {
        GSE_ERR("set power mgmt failed!\n");
        return ICM42607_ERR_BUS;
    }
	
	icm42607_all_sensor_status_info[sensor_type].sensor_power = enable;
    if(enable == true)
        mdelay(1);
    return ICM42607_SUCCESS;
}

static int icm42607_ReadChipInfo(char *buf, int bufsize)
{
    u8 databuf[2] = {0};
    int res = 0;

    if ((NULL == buf) || (bufsize <= 30))
        return -1;
	GSE_FUNC();
    res = icm42607_share_read_register(REG_WHO_AM_I, databuf, 1);
	GSE_INFO("icm42607 chip id: 0x%x",databuf[0]);
    if (res < 0) {
        GSE_ERR("read who_am_i register err!\n");
        return ICM42607_ERR_BUS;
    }
    switch (databuf[0]) {
    case WHO_AM_I_ICM42607P:
        sprintf(buf, "ICM42607P [0x%x]", databuf[0]);
        break;
    case WHO_AM_I_ICM42607C:
        sprintf(buf, "ICM42607C [0x%x]", databuf[0]);
        break;
    default:
        sprintf(buf, "Unknown Sensor [0x%x]", databuf[0]);
        return ICM42607_ERR_STATUS;
    }
    return ICM42607_SUCCESS;
}

static int icm42607_SetSampleRate(int sensor_type,
    unsigned int delay_ns, bool force_800hz)
{
    u8 databuf[2] = {0};
    unsigned int sample_rate = 0;
    int res = 0;
    int i, highest_sample_rate = 0;

    /* ns to us */
    sample_rate = (delay_ns / 1000);
    /* us to ms */
    sample_rate = (sample_rate / 1000);
    /* ms to hz */
    if(sample_rate != 0)
        sample_rate = (int)(1000 / sample_rate);
        GSE_INFO("sample_rate odr %d\n",sample_rate);
	/* sample rate: 5hz to 500hz;  UP to android O , we need support up to 500 hz  */
    /* when force_800hz is true, it means self test mode is running at 800hz */
    if ((sample_rate > 800) || (force_800hz == true)) {
        sample_rate = 800;
    } else if (sample_rate < 25) {
        sample_rate = 25;
	}
    if(icm42607_all_sensor_status_info[sensor_type].sample_rate == sample_rate)
        return ICM42607_SUCCESS;
    icm42607_all_sensor_status_info[sensor_type].sample_rate = sample_rate;
    if (sensor_type == ICM42607_SENSOR_TYPE_GYRO) {
        res = icm42607_share_read_register(REG_GYRO_CONFIG0, databuf, 1);
        if (res < 0) {
            GSE_ERR("read odr register err!\n");
            return ICM42607_ERR_BUS;
        }
        /*
        b'0110: 800Hz
        b'0111: 400Hz
        b'1000: 200Hz
        b'1001: 100Hz
        b'1010: 50Hz
        b'1011: 25Hz
        */
        databuf[0] &= ~BIT_GYRO_ODR;
        if (sample_rate > 400) {
            databuf[0] |= 6;
        } else if (sample_rate > 200) {
            databuf[0] |= 7;
        } else if (sample_rate > 100) {
            databuf[0] |= 8;
        } else if (sample_rate > 50) {
            databuf[0] |= 9;
        } else if (sample_rate > 25) {
            databuf[0] |= 10;
        } else {
            databuf[0] |= 11;
		}
        res = icm42607_share_write_register(REG_GYRO_CONFIG0, databuf, 1);
        if (res < 0) {
            GSE_ERR("write odr register err!\n");
            return ICM42607_ERR_BUS;
        }
    }
    if (sensor_type == ICM42607_SENSOR_TYPE_ACC ||
        sensor_type == ICM42607_SENSOR_TYPE_SC ||
        sensor_type == ICM42607_SENSOR_TYPE_SD ||
        sensor_type == ICM42607_SENSOR_TYPE_SD
    ) {
        /* check sample rate of enabled sensors */
        for(i = 0; i < ICM42607_SENSOR_TYPE_MAX; i++) {
            if (i == ICM42607_SENSOR_TYPE_GYRO)
                continue;
            if(icm42607_all_sensor_status_info[i].sensor_power == true) {
				GSE_INFO("highest_sample_rate %d,type %d rate %d\n",highest_sample_rate,i,icm42607_all_sensor_status_info[i].sample_rate);
                if(highest_sample_rate <
                    icm42607_all_sensor_status_info[i].sample_rate)
                    highest_sample_rate =
                        icm42607_all_sensor_status_info[i].sample_rate;
            }
        }
        res = icm42607_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            GSE_ERR("read odr register err!\n");
            return ICM42607_ERR_BUS;
        }
        /*
        b'0110: 800Hz
        b'0111: 400Hz
        b'1000: 200Hz
        b'1001: 100Hz
        b'1010: 50Hz
        b'1011: 25Hz
        */
        GSE_INFO("highest_sample_rate odr %d,REG_ACCEL_CONFIG0 buf %x\n",highest_sample_rate,databuf[0]);
        databuf[0] &= ~BIT_ACCEL_ODR;
        if (highest_sample_rate > 400) {
            databuf[0] |= 6;
        } else if (highest_sample_rate > 200) {
            databuf[0] |= 7;
        } else if (highest_sample_rate > 100) {
            databuf[0] |= 8;
        } else if (highest_sample_rate > 50) {
            databuf[0] |= 9;
        } else if (highest_sample_rate > 25) {
            databuf[0] |= 10;
        } else {
            databuf[0] |= 11;
		}
		//databuf[0] |= 0x40;   //force to set 4g here ,always use 4g here ?
        res = icm42607_share_write_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            GSE_ERR("write odr register err!\n");
            return ICM42607_ERR_BUS;
        }
		GSE_INFO("REG_ACCEL_CONFIG0 write buf %x\n",databuf[0]);
#ifdef DEBUG
		res = icm42607_share_read_register(REG_ACCEL_CONFIG0, databuf, 1);
        if (res < 0) {
            GSE_ERR("read odr register err!\n");
            return ICM42607_ERR_BUS;
        }
	    GSE_INFO("REG_ACCEL_CONFIG0 buf %x\n",databuf[0]);
#endif
    }
    return ICM42607_SUCCESS;
}

/* wrappers */

int icm42607_share_InterruptConfig_INT1(void){
	return icm42607_InterruptConfig_INT1();
}
EXPORT_SYMBOL(icm42607_share_InterruptConfig_INT1);

void icm42607_share_set_sensor_power_mode(int paramSensor, u8 paramPower)
{
    icm42607_set_sensor_power_mode(paramSensor, paramPower);
}
EXPORT_SYMBOL(icm42607_share_set_sensor_power_mode);

u8 icm42607_share_get_sensor_power_mode(int paramSensor)
{
    return icm42607_get_sensor_power_mode(paramSensor);
}
EXPORT_SYMBOL(icm42607_share_get_sensor_power_mode);

bool icm42607_share_get_sensor_power(int paramSensor)
{
    return icm42607_get_sensor_power(paramSensor);
}
EXPORT_SYMBOL(icm42607_share_get_sensor_power);

bool icm42607_share_any_accel_based_sensor_is_on(void)
{
    return icm42607_any_accel_based_sensor_is_on();
}
EXPORT_SYMBOL(icm42607_share_any_accel_based_sensor_is_on);

int icm42607_share_ChipSoftReset(void)
{
    return icm42607_ChipSoftReset();
}
EXPORT_SYMBOL(icm42607_share_ChipSoftReset);

int icm42607_share_SetPowerMode(int sensor_type, bool enable)
{
    return icm42607_SetPowerMode(sensor_type, enable);
}
EXPORT_SYMBOL(icm42607_share_SetPowerMode);

int icm42607_share_EnableInterrupt(u8 int_type, bool enable)
{
    return icm42607_EnableInterrupt(int_type, enable);
}
EXPORT_SYMBOL(icm42607_share_EnableInterrupt);

int icm42607_share_EnableSensor(int sensor_type, bool enable,bool *flag)
{
    return icm42607_EnableSensor(sensor_type, enable , flag);
}
EXPORT_SYMBOL(icm42607_share_EnableSensor);

int icm42607_share_ReadChipInfo(char *buf, int bufsize)
{
    return icm42607_ReadChipInfo(buf, bufsize);
}
EXPORT_SYMBOL(icm42607_share_ReadChipInfo);

int icm42607_share_SetSampleRate(int sensor_type,
    u64 delay_ns, bool force_800hz)
{
    return icm42607_SetSampleRate(sensor_type, delay_ns, force_800hz);
}
EXPORT_SYMBOL(icm42607_share_SetSampleRate);
