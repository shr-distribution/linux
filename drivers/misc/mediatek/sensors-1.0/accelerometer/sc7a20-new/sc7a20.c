/******************** (C) COPYRIGHT 2016 MEMSIC ********************
 *
 * File Name          : sc7a20.c
 * Description        : SC7A20 accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, MEMSIC SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH MEMSIC PARTS.
 *

 ******************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/time.h>
#include <linux/kernel.h>


#include "accel.h"
#include "cust_acc.h"
#include "sensors_io.h"
#include "sc7a20.h"


#define I2C_DRIVERID_SC7A20		120
#define SW_CALIBRATION
#define DRIVER_VERSION				"V1.0"
#define SC7A20_BUF_SIZE 			256

//add by djq, 2017-09-19 gsensor offset because of different working voltage(VDD)  0.
//////////////////////////////////
// 0  -->  1.8V
// 1  -->  2.8V
// 2  -->  3.3V
// 3  -->  default   do not read related offset register
/////////////////////////////////
#define WORKING_VOLTAGE  3 
//end

//  add by djq 2017-08-02 start	
#define Z_OFF_DEFAULT  (110)    //djq 2017-08-07 
#define XY_THR_N      (-200)
#define XY_THR_P      (200)
#define Z_THR_MIN_N   (-780)
#define Z_THR_MIN_P   (780)
#define Z_THR_MAX_N   (-1200)
#define Z_THR_MAX_P   (1200)
#define SUM_DOT       (50)
#define THRESHOLD_VAL (40)
//  add by djq 2017-08-02 end
#define GSE_DEBUG_ON          		0
#define GSE_DEBUG_FUNC_ON     		0
#if GSE_DEBUG_ON
/* Log define */
#define GSE_INFO(fmt, arg...)      	printk("<<-GSE INFO->> "fmt"\n", ##arg)
#define GSE_ERR(fmt, arg...)          	printk("<<-GSE ERROR->> "fmt"\n", ##arg)
#define GSE_DEBUG(fmt, arg...)		do {\
						if (GSE_DEBUG_ON)\
							printk("<<-GSE DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
					} while (0)
#define GSE_DEBUG_FUNC()		do {\
						if (GSE_DEBUG_FUNC_ON)\
							printk("<<-GSE FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
					} while (0)
#else
//#define GSE_TAG
#define GSE_INFO(fmt, args...)	do {} while (0)
#define GSE_ERR(fmt, args...)	do {} while (0)
#define GSE_DEBUG(fmt, args...)	do {} while (0)
#define GSE_DEBUG_FUNC()    	do {} while (0)


#endif

extern void app_get_g_sensor_name(char *g_name);


#define SC7A20_AXIS_X          	0
#define SC7A20_AXIS_Y          	1
#define SC7A20_AXIS_Z          	2
#define SC7A20_AXES_NUM        	3
#define SC7A20_DATA_LEN        	6
#define C_MAX_FIR_LENGTH 		(32)
#define  USE_DELAY

struct acc_hw accel_cust;
struct acc_hw *hw = &accel_cust;
#ifdef USE_DELAY
static int delay_state = 0;
#endif
static atomic_t open_flag = ATOMIC_INIT(0);

static const struct i2c_device_id sc7a20_i2c_id[] = { { SC7A20_DEV_NAME, 0 }, { }, };
static int sc7a20_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int sc7a20_i2c_remove(struct i2c_client *client);

#ifdef CONFIG_PM_SLEEP
static int sc7a20_suspend(struct device *dev);
static int sc7a20_resume(struct device *dev);
#endif
static DECLARE_WAIT_QUEUE_HEAD(open_wq);
static int  sc7a20_local_init(void);
static int sc7a20_remove(void);
static int sc7a20_flush(void);

//add by djq, 2017-09-19 gsensor offset because of different working voltage(VDD)  1.
static u8 offset_buf[3] = {0, 0, 0};
//end
typedef enum {
	ADX_TRC_FILTER   = 0x01,
	ADX_TRC_RAWDATA  = 0x02,
	ADX_TRC_IOCTL	 = 0x04,
	ADX_TRC_CALI	 = 0X08,
	ADX_TRC_INFO	 = 0X10,
} ADX_TRC;

struct scale_factor{
	u8  whole;
	u8  fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int    sensitivity;
};


struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][SC7A20_AXES_NUM];
	int sum[SC7A20_AXES_NUM];
	int num;
	int idx;
};
static int sc7a20_init_flag = -1;
/*----------------------------------------------------------------------------*/
static struct acc_init_info sc7a20_init_info = {
		.name = "sc7a20",
		.init = sc7a20_local_init,
		.uninit = sc7a20_remove,
};
struct sc7a20_i2c_data {
		 struct i2c_client *client;
		 struct acc_hw *hw;
		 struct hwmsen_convert	 cvt;
		 atomic_t layout;
		 /*misc*/
		 struct data_resolution *reso;
		 atomic_t				 trace;
		 atomic_t				 suspend;
		 atomic_t				 selftest;
		 atomic_t				 filter;
		 s16					 cali_sw[SC7A20_AXES_NUM+1];

		 /*data*/
		 s8 					 offset[SC7A20_AXES_NUM+1];  /*+1: for 4-byte alignment*/
		 s16					 data[SC7A20_AXES_NUM+1];
#if defined(CONFIG_SC7A20_LOWPASS)
		 atomic_t				 firlen;
		 atomic_t				 fir_en;
		 struct data_filter 	 fir;
#endif
		 /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
		 struct early_suspend	 early_drv;
#endif
			bool flush;
};

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops sc7a20_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sc7a20_suspend, sc7a20_resume)
};
#endif

static struct i2c_driver sc7a20_i2c_driver = {
	 .driver = {
		// .owner		   = THIS_MODULE,
		 .name			 = SC7A20_DEV_NAME,
		 
#ifdef CONFIG_PM_SLEEP
		.pm = &sc7a20_i2c_pm_ops,
#endif
		 
	   #ifdef CONFIG_OF
		.of_match_table = accel_of_match,
	   #endif
	 },
	 .probe 			 = sc7a20_i2c_probe,
	 .remove			 = sc7a20_i2c_remove,
	 .id_table = sc7a20_i2c_id,
};



struct i2c_client      			*sc7a20_i2c_client = NULL;
static struct sc7a20_i2c_data 		*obj_i2c_data = NULL;
static bool sensor_power = false;
static struct GSENSOR_VECTOR3D gsensor_gain;
static struct mutex sc7a20_mutex;
static bool enable_status = false;

static struct data_resolution sc7a20_data_resolution[] = {
    {{ 0, 9}, 1024},   /*+/-2g  in 12-bit resolution:  0.9 mg/LSB*/
    {{ 1, 9}, 512},   /*+/-4g  in 12-bit resolution:  1.9 mg/LSB*/
    {{ 3, 9},  256},   /*+/-8g  in 12-bit resolution: 3.9 mg/LSB*/
};
static struct data_resolution sc7a20_offset_resolution = {{ 0, 9}, 1024}; //{{3, 9}, 256};

static int sc7a20_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,	.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
				addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	return err;

}
static void sc7a20_power(struct acc_hw *hw, unsigned int on)
{
	 static unsigned int power_on = 0;

	 power_on = on;
}



static int SC7A20_SetDataResolution(struct sc7a20_i2c_data *obj)
{
 	obj->reso = &sc7a20_data_resolution[0];//obj->reso = &sc7a20_data_resolution[2];
	return SC7A20_SUCCESS;
}

void find_max_min(s16 *array, int len, s16 *max, s16 *min)
{
	s16 temp_max = *array;
	s16 temp_min = *array;
	
	int i = 0;
	for(i = 0; i < len; i++)
	{
		if(*(array + i) > temp_max)
			temp_max = *(array + i);
		
		if(*(array + i) < temp_min)
			temp_min = *(array + i);		
	}
	
	*max = temp_max;
	*min = temp_min;
}

static int SC7A20_ReadData(struct i2c_client *client, s16 data[SC7A20_AXES_NUM])
{
#ifdef CONFIG_SC7A20_LOWPASS
	struct sc7a20_i2c_data *priv = i2c_get_clientdata(client);
#endif
	u8 addr = SC7A20_REG_X | 0x80;  //djq,2017-12-28
	u8 buf[SC7A20_DATA_LEN] = {0};
	int err = 0;


//  add by djq 2017-08-02 start
	static s16 axis_x_off = 0;  			//jq 2017-08-07
	static s16 axis_y_off = 0;  			//jq 2017-08-07	
	static s16 axis_z_off = Z_OFF_DEFAULT;  //jq 2017-08-07
	static u16 index = 0;
	static s16 x_value[SUM_DOT] ={0};
	static s16 y_value[SUM_DOT] ={0};
	static s16 z_value[SUM_DOT] ={0};
	
	int flag_x = 0;
	int flag_y = 0;
	int flag_z = 0;
	
	int i = 0;

	s16 x_min = 0;
	s16 x_max = 0;	
	s16 y_min = 0;
	s16 y_max = 0;
	s16 z_min = 0;	
	s16 z_max = 0;
		
	s32 temp_x_value = 0;
	s32 temp_y_value = 0;	
	s32 temp_z_value = 0;
//  add by djq 2017-08-02 end	
//add by djq, 2017-09-19 gsensor offset because of different working voltage(VDD) 3.
	u8 temp_buf[3] = {0, 0, 0};
//end

#ifdef USE_DELAY
	if(delay_state){

	  //printk("djq debug sc7a20 delay 300ms begin \n");
		msleep(300);
		delay_state = 0;
	}
#endif

	if(NULL == client)
	{
        	GSE_ERR("client is null\n");
		err = -EINVAL;
	}
	if((err = sc7a20_i2c_read_block(client, addr, buf, SC7A20_DATA_LEN))!=0)
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
	    temp_buf[0] = (u8)((s8)buf[1] - (s8)offset_buf[0]);
	    temp_buf[1] = (u8)((s8)buf[3] - (s8)offset_buf[1]);
	    temp_buf[2] = (u8)((s8)buf[5] - (s8)offset_buf[2]);
		data[SC7A20_AXIS_X] = ((s16) (((u8)temp_buf[0] * 256) + (u8)buf[0] ) >> 4);
		data[SC7A20_AXIS_Y] = ((s16) (((u8)temp_buf[1] * 256) + (u8)buf[2] ) >> 4);	
		data[SC7A20_AXIS_Z] = ((s16) (((u8)temp_buf[2] * 256) + (u8)buf[4] ) >> 4);				
//		data[SC7A20_AXIS_X] = (s16)(buf[1] << 8 | buf[0]) >> 4;
//		data[SC7A20_AXIS_Y] = (s16)(buf[3] << 8 | buf[2]) >> 4;
//		data[SC7A20_AXIS_Z] = (s16)(buf[5] << 8 | buf[4]) >> 4;

//  add by djq 2017-08-02 start	
	    if( data[SC7A20_AXIS_X] > XY_THR_N && data[SC7A20_AXIS_X] < XY_THR_P) 		
	    	flag_x = 1;
		else
			flag_x = 0;
			
	    if( data[SC7A20_AXIS_Y] > XY_THR_N && data[SC7A20_AXIS_Y] < XY_THR_P) 		
	    	flag_y = 1;
		else
			flag_y = 0;	

        if((data[SC7A20_AXIS_Z] > Z_THR_MAX_N && data[SC7A20_AXIS_Z] < Z_THR_MIN_N)  || (data[SC7A20_AXIS_Z] > Z_THR_MIN_P && data[SC7A20_AXIS_Z] < Z_THR_MAX_P)) 		
	    	flag_z = 1;
		else
			flag_z = 0;	

        if(flag_x == 1 && flag_y == 1 && flag_z == 1)
        {
			x_value[index] = data[SC7A20_AXIS_X];
			y_value[index] = data[SC7A20_AXIS_Y];		    
			z_value[index] = data[SC7A20_AXIS_Z];
			index = index + 1;
        }
        else
            index = 0;		
		
        if(index == SUM_DOT)	
        {
			find_max_min(x_value, SUM_DOT, &x_max, &x_min);	
			find_max_min(y_value, SUM_DOT, &y_max, &y_min);	
			find_max_min(z_value, SUM_DOT, &z_max, &z_min);	
			
			if( ((x_max - x_min) < THRESHOLD_VAL)  && ((y_max - y_min) < THRESHOLD_VAL) && ((z_max - z_min) < THRESHOLD_VAL))	
			{	
			    
				temp_x_value = 0;
				for(i = 0; i < SUM_DOT; i++)
				{
					temp_x_value += x_value[i];
				}
				temp_x_value = temp_x_value / SUM_DOT;
				axis_x_off = 0 - (s16)temp_x_value;
			
				temp_y_value = 0;
				for(i = 0; i < SUM_DOT; i++)
				{
					temp_y_value += y_value[i];
				}
				temp_y_value = temp_y_value / SUM_DOT;
				axis_y_off = 0 - (s16)temp_y_value;				
				
				temp_z_value = 0;
				for(i = 0; i < SUM_DOT; i++)
				{
					temp_z_value += z_value[i];
				}
				temp_z_value = temp_z_value / SUM_DOT;
			
				if(temp_z_value > Z_THR_MAX_N && temp_z_value < Z_THR_MIN_N) 		
					axis_z_off = -1024 - (s16)temp_z_value;
				else 
					axis_z_off = 1024 - (s16)temp_z_value;	
			   				
			}
			index = 0;

		}
		data[SC7A20_AXIS_X] +=  axis_x_off;		
	    data[SC7A20_AXIS_Y] +=  axis_y_off;
	    data[SC7A20_AXIS_Z] +=  axis_z_off;
//  add by djq 2017-08-02 end	
		{
			// decrease print log frequence.
			static unsigned int cycle = 0;
			if( cycle++%100==0) //print a msg about 2sec 
				GSE_DEBUG("reg data x = %d %d y = %d %d z = %d %d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		}
#ifdef CONFIG_SC7A20_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);
				if(priv->fir.num < firlen)
				{
					priv->fir.raw[priv->fir.num][SC7A20_AXIS_X] = data[SC7A20_AXIS_X];
					priv->fir.raw[priv->fir.num][SC7A20_AXIS_Y] = data[SC7A20_AXIS_Y];
					priv->fir.raw[priv->fir.num][SC7A20_AXIS_Z] = data[SC7A20_AXIS_Z];
					priv->fir.sum[SC7A20_AXIS_X] += data[SC7A20_AXIS_X];
					priv->fir.sum[SC7A20_AXIS_Y] += data[SC7A20_AXIS_Y];
					priv->fir.sum[SC7A20_AXIS_Z] += data[SC7A20_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_DEBUG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][SC7A20_AXIS_X], priv->fir.raw[priv->fir.num][SC7A20_AXIS_Y], priv->fir.raw[priv->fir.num][SC7A20_AXIS_Z],
							priv->fir.sum[SC7A20_AXIS_X], priv->fir.sum[SC7A20_AXIS_Y], priv->fir.sum[SC7A20_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[SC7A20_AXIS_X] -= priv->fir.raw[idx][SC7A20_AXIS_X];
					priv->fir.sum[SC7A20_AXIS_Y] -= priv->fir.raw[idx][SC7A20_AXIS_Y];
					priv->fir.sum[SC7A20_AXIS_Z] -= priv->fir.raw[idx][SC7A20_AXIS_Z];
					priv->fir.raw[idx][SC7A20_AXIS_X] = data[SC7A20_AXIS_X];
					priv->fir.raw[idx][SC7A20_AXIS_Y] = data[SC7A20_AXIS_Y];
					priv->fir.raw[idx][SC7A20_AXIS_Z] = data[SC7A20_AXIS_Z];
					priv->fir.sum[SC7A20_AXIS_X] += data[SC7A20_AXIS_X];
					priv->fir.sum[SC7A20_AXIS_Y] += data[SC7A20_AXIS_Y];
					priv->fir.sum[SC7A20_AXIS_Z] += data[SC7A20_AXIS_Z];
					priv->fir.idx++;
					data[SC7A20_AXIS_X] = priv->fir.sum[SC7A20_AXIS_X]/firlen;
					data[SC7A20_AXIS_Y] = priv->fir.sum[SC7A20_AXIS_Y]/firlen;
					data[SC7A20_AXIS_Z] = priv->fir.sum[SC7A20_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_DEBUG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][SC7A20_AXIS_X], priv->fir.raw[idx][SC7A20_AXIS_Y], priv->fir.raw[idx][SC7A20_AXIS_Z],
						priv->fir.sum[SC7A20_AXIS_X], priv->fir.sum[SC7A20_AXIS_Y], priv->fir.sum[SC7A20_AXIS_Z],
						data[SC7A20_AXIS_X], data[SC7A20_AXIS_Y], data[SC7A20_AXIS_Z]);
					}
				}
			}
		}
#endif
	}
	
	return err;
}


static int SC7A20_ReadOffset(struct i2c_client *client, s8 ofs[SC7A20_AXES_NUM])
{
	int err;
	err = 0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#endif
	return err;
}

static int SC7A20_ResetCalibration(struct i2c_client *client)
{
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	err = 0;

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

static int SC7A20_ReadCalibration(struct i2c_client *client, int dat[SC7A20_AXES_NUM])
{
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	int  err = 0;
	int mul;

#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if ((err = SC7A20_ReadOffset(client, obj->offset))) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/sc7a20_offset_resolution.sensitivity;
#endif

	dat[obj->cvt.map[SC7A20_AXIS_X]] = obj->cvt.sign[SC7A20_AXIS_X]*(obj->offset[SC7A20_AXIS_X]*mul + obj->cali_sw[SC7A20_AXIS_X]);
	dat[obj->cvt.map[SC7A20_AXIS_Y]] = obj->cvt.sign[SC7A20_AXIS_Y]*(obj->offset[SC7A20_AXIS_Y]*mul + obj->cali_sw[SC7A20_AXIS_Y]);
	dat[obj->cvt.map[SC7A20_AXIS_Z]] = obj->cvt.sign[SC7A20_AXIS_Z]*(obj->offset[SC7A20_AXIS_Z]*mul + obj->cali_sw[SC7A20_AXIS_Z]);

	return err;
}
static int SC7A20_ReadCalibrationEx(struct i2c_client *client, int act[SC7A20_AXES_NUM], int raw[SC7A20_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	err = 0;


#ifdef SW_CALIBRATION
	mul = 0;//only SW Calibration, disable HW Calibration
#else
	if((err = SC7A20_ReadOffset(client, obj->offset)))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	mul = obj->reso->sensitivity/sc7a20_offset_resolution.sensitivity;
#endif

	raw[SC7A20_AXIS_X] = obj->offset[SC7A20_AXIS_X]*mul + obj->cali_sw[SC7A20_AXIS_X];
	raw[SC7A20_AXIS_Y] = obj->offset[SC7A20_AXIS_Y]*mul + obj->cali_sw[SC7A20_AXIS_Y];
	raw[SC7A20_AXIS_Z] = obj->offset[SC7A20_AXIS_Z]*mul + obj->cali_sw[SC7A20_AXIS_Z];

	act[obj->cvt.map[SC7A20_AXIS_X]] = obj->cvt.sign[SC7A20_AXIS_X]*raw[SC7A20_AXIS_X];
	act[obj->cvt.map[SC7A20_AXIS_Y]] = obj->cvt.sign[SC7A20_AXIS_Y]*raw[SC7A20_AXIS_Y];
	act[obj->cvt.map[SC7A20_AXIS_Z]] = obj->cvt.sign[SC7A20_AXIS_Z]*raw[SC7A20_AXIS_Z];

	return 0;
}

static int SC7A20_WriteCalibration(struct i2c_client *client, int dat[SC7A20_AXES_NUM])
{
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[SC7A20_AXES_NUM], raw[SC7A20_AXES_NUM];
#ifdef SW_CALIBRATION
#else
	int lsb;
	lsb = sc7a20_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif
	if((err = SC7A20_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_DEBUG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
			raw[SC7A20_AXIS_X], raw[SC7A20_AXIS_Y], raw[SC7A20_AXIS_Z],
			obj->offset[SC7A20_AXIS_X], obj->offset[SC7A20_AXIS_Y], obj->offset[SC7A20_AXIS_Z],
			obj->cali_sw[SC7A20_AXIS_X], obj->cali_sw[SC7A20_AXIS_Y], obj->cali_sw[SC7A20_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[SC7A20_AXIS_X] += dat[SC7A20_AXIS_X];
	cali[SC7A20_AXIS_Y] += dat[SC7A20_AXIS_Y];
	cali[SC7A20_AXIS_Z] += dat[SC7A20_AXIS_Z];

	GSE_DEBUG("UPDATE: (%+3d %+3d %+3d)\n",
			dat[SC7A20_AXIS_X], dat[SC7A20_AXIS_Y], dat[SC7A20_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[SC7A20_AXIS_X] = obj->cvt.sign[SC7A20_AXIS_X]*(cali[obj->cvt.map[SC7A20_AXIS_X]]);
	obj->cali_sw[SC7A20_AXIS_Y] = obj->cvt.sign[SC7A20_AXIS_Y]*(cali[obj->cvt.map[SC7A20_AXIS_Y]]);
	obj->cali_sw[SC7A20_AXIS_Z] = obj->cvt.sign[SC7A20_AXIS_Z]*(cali[obj->cvt.map[SC7A20_AXIS_Z]]);
#else
	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[SC7A20_AXIS_X] = (s8)(obj->cvt.sign[SC7A20_AXIS_X]*(cali[obj->cvt.map[SC7A20_AXIS_X]])/(divisor));
	obj->offset[SC7A20_AXIS_Y] = (s8)(obj->cvt.sign[SC7A20_AXIS_Y]*(cali[obj->cvt.map[SC7A20_AXIS_Y]])/(divisor));
	obj->offset[SC7A20_AXIS_Z] = (s8)(obj->cvt.sign[SC7A20_AXIS_Z]*(cali[obj->cvt.map[SC7A20_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[SC7A20_AXIS_X] = obj->cvt.sign[SC7A20_AXIS_X]*(cali[obj->cvt.map[SC7A20_AXIS_X]])%(divisor);
	obj->cali_sw[SC7A20_AXIS_Y] = obj->cvt.sign[SC7A20_AXIS_Y]*(cali[obj->cvt.map[SC7A20_AXIS_Y]])%(divisor);
	obj->cali_sw[SC7A20_AXIS_Z] = obj->cvt.sign[SC7A20_AXIS_Z]*(cali[obj->cvt.map[SC7A20_AXIS_Z]])%(divisor);

	GSE_DEBUG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
			obj->offset[SC7A20_AXIS_X]*divisor + obj->cali_sw[SC7A20_AXIS_X],
			obj->offset[SC7A20_AXIS_Y]*divisor + obj->cali_sw[SC7A20_AXIS_Y],
			obj->offset[SC7A20_AXIS_Z]*divisor + obj->cali_sw[SC7A20_AXIS_Z],
			obj->offset[SC7A20_AXIS_X], obj->offset[SC7A20_AXIS_Y], obj->offset[SC7A20_AXIS_Z],
			obj->cali_sw[SC7A20_AXIS_X], obj->cali_sw[SC7A20_AXIS_Y], obj->cali_sw[SC7A20_AXIS_Z]);

#endif
	msleep(1);
	return err;
}

static int SC7A20_CheckDeviceID(struct i2c_client *client)   //djq,2017-12-28
{
	u8 addr=SC7A20_REG_ID;
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	res = sc7a20_i2c_read_block(client, addr, databuf, 1);    //normal return value is 0
	if(res < 0)
	{
		goto exit_SC7A20_CheckDeviceID;
	}



	databuf[0]= (databuf[0]&0xff);

	if(databuf[0]!= SC7A20_ID_1)
	{
		return SC7A20_ERR_IDENTIFICATION;
	}

	GSE_INFO("SC7A20_CheckDeviceID %d done!\n ", databuf[0]);

	return SC7A20_SUCCESS;

exit_SC7A20_CheckDeviceID:
	if (res < 0)
	{
		GSE_ERR("SC7A20_CheckDeviceID %d failed!\n ", SC7A20_ERR_I2C);
		return SC7A20_ERR_I2C;
	}
	return SC7A20_ERR_I2C;
}


static int SC7A20_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0, i=0;
	u8 addr = SC7A20_REG_CTL_REG1;  //djq,2017-12-28
	
    res = sc7a20_i2c_read_block(client, addr, databuf, 1);    //normal return value is 0
	if(res < 0)
		return SC7A20_ERR_I2C;
		
	databuf[0] &= ~SC7A20_POWER_MODE_MASK;	
	if(enable == 1)
	{
		databuf[0] |= SC7A20_AWAKE;
	}
	else
	{
		databuf[0] |= SC7A20_SLEEP;
	}
	msleep(SC7A20_STABLE_DELAY);
	databuf[1] = databuf[0];	
	databuf[0] = SC7A20_REG_CTL_REG1;
	while (i++ < 3)
	{
		res = i2c_master_send(client, databuf, 0x2);
		msleep(5);
		if(res > 0)
			break;
	}

	if(res <= 0)
	{
		GSE_ERR("silan set power mode failed!\n");
		return SC7A20_ERR_I2C;
	}
	sensor_power = enable;
#ifdef USE_DELAY
	delay_state = enable;
#else
	msleep(300);
#endif
	if (obj_i2c_data->flush) {
		if (sensor_power) {
			GSE_ERR("remain flush");
			sc7a20_flush();
		} else
		obj_i2c_data->flush = false;
	}
	return SC7A20_SUCCESS;
}
static int SC7A20_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	u8 addr = SC7A20_REG_CTL_REG4;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	
    res = sc7a20_i2c_read_block(client, addr, databuf, 1);    //normal return value is 0
	if(res < 0)
		return SC7A20_ERR_I2C;

	databuf[0] &= ~SC7A20_RANGE;
	
	if(SC7A20_RANGE_8G == dataformat)
	{
		databuf[0] |= SC7A20_RANGE_8G;
	}
	else
	{
		databuf[0] |= SC7A20_RANGE_2G;
	}
	databuf[0] |= 0x88;
	databuf[1] = databuf[0];
	databuf[0] = SC7A20_REG_CTL_REG4;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("set power mode failed!\n");
		return SC7A20_ERR_I2C;
	}

	return SC7A20_SetDataResolution(obj);
}
static int SC7A20_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
    u8 addr = SC7A20_REG_CTL_REG1;
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);
	
    res = sc7a20_i2c_read_block(client, addr, databuf, 1);    //normal return value is 0
	if(res < 0)
		return SC7A20_ERR_I2C;
		
	databuf[0] &= ~SC7A20_BW;

	if(SC7A20_BW_400HZ == bwrate)
	{
		databuf[0] |= SC7A20_BW_400HZ;
	}
	else
	{
		 databuf[0] |= SC7A20_BW_100HZ;
	}
	databuf[1] = databuf[0];
	databuf[0] = SC7A20_REG_CTL_REG1;	
	
	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		GSE_ERR("SC7A20_SetBWRate failed!\n");	
		return SC7A20_ERR_I2C;
	}
		
	return SC7A20_SUCCESS;
}

static int sc7a20_init_client(struct i2c_client *client, int reset_cali)
{
	 struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	 int res = 0;

	 GSE_DEBUG_FUNC();
	 res = SC7A20_SetPowerMode(client, true);
	 if(res != SC7A20_SUCCESS)
	 {
		return res;
	 }
	 res = SC7A20_CheckDeviceID(client);
	 if(res != SC7A20_SUCCESS)
	 {
	 	 GSE_ERR("SC7A20 check device id failed\n");
	 	 return res;
	 }

	res = SC7A20_SetBWRate(client, SC7A20_BW_50HZ);
	if(res != SC7A20_SUCCESS )
	{
		GSE_ERR("SC7A20 Set BWRate failed\n");
		return res;
	}

	res = SC7A20_SetDataFormat(client, SC7A20_RANGE_2G);//res = SC7A20_SetDataFormat(client, SC7A20_RANGE_8G);
	if(res != SC7A20_SUCCESS)
	{
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	if(0 != reset_cali)
	{
	 	/*reset calibration only in power on*/
		 res = SC7A20_ResetCalibration(client);
		 if(res != SC7A20_SUCCESS)
		 {
			 return res;
		 }
	}
	GSE_INFO("sc7a20_init_client OK!\n");
	
//add by djq, 2017-09-19 gsensor offset because of different working voltage(VDD)  2.
#if (WORKING_VOLTAGE == 0)
{
	sc7a20_i2c_read_block(client, 0x13, &offset_buf[0], 1);
	sc7a20_i2c_read_block(client, 0x14, &offset_buf[1], 1);
	sc7a20_i2c_read_block(client, 0x17, &offset_buf[2], 1);
}
#elif (WORKING_VOLTAGE == 1)
{
	sc7a20_i2c_read_block(client, 0x18, &offset_buf[0], 1);
	sc7a20_i2c_read_block(client, 0x19, &offset_buf[1], 1);
	sc7a20_i2c_read_block(client, 0x1a, &offset_buf[2], 1);
}

#elif (WORKING_VOLTAGE == 2)
{
	sc7a20_i2c_read_block(client, 0x50, &offset_buf[0], 1);
	sc7a20_i2c_read_block(client, 0x51, &offset_buf[1], 1);
	sc7a20_i2c_read_block(client, 0x52, &offset_buf[2], 1);
}

#else
{

}
#endif
//end	
#ifdef CONFIG_SC7A20_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif
	msleep(20);

	return SC7A20_SUCCESS;
}


static int SC7A20_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}

	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "SC7A20 Chip");
	return 0;
}



static int SC7A20_ReadSensorData(struct i2c_client *client, int *buf, int bufsize)
{
	struct sc7a20_i2c_data *obj = (struct sc7a20_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[SC7A20_AXES_NUM] = {0};
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		GSE_ERR("buf is null !!!\n");
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		GSE_ERR("client is null !!!\n");
		return SC7A20_ERR_STATUS;
	}

	if (atomic_read(&obj->suspend) && !enable_status)
	{
		GSE_DEBUG("sensor in suspend read not data!\n");
		return SC7A20_ERR_GETGSENSORDATA;
	}


	if((res = SC7A20_ReadData(client, obj->data)))
	{
		GSE_ERR("I2C error: ret value=%d", res);
		return SC7A20_ERR_I2C;
	}
	else
	{
		obj->data[SC7A20_AXIS_X] += obj->cali_sw[SC7A20_AXIS_X];
		obj->data[SC7A20_AXIS_Y] += obj->cali_sw[SC7A20_AXIS_Y];
		obj->data[SC7A20_AXIS_Z] += obj->cali_sw[SC7A20_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[SC7A20_AXIS_X]] = obj->cvt.sign[SC7A20_AXIS_X]*obj->data[SC7A20_AXIS_X];
		acc[obj->cvt.map[SC7A20_AXIS_Y]] = obj->cvt.sign[SC7A20_AXIS_Y]*obj->data[SC7A20_AXIS_Y];
		acc[obj->cvt.map[SC7A20_AXIS_Z]] = obj->cvt.sign[SC7A20_AXIS_Z]*obj->data[SC7A20_AXIS_Z];

		//Out put the mg
		acc[SC7A20_AXIS_X] = acc[SC7A20_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[SC7A20_AXIS_Y] = acc[SC7A20_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[SC7A20_AXIS_Z] = acc[SC7A20_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		buf[0] = -acc[SC7A20_AXIS_X];
		buf[1] = -acc[SC7A20_AXIS_Y];
		buf[2] = acc[SC7A20_AXIS_Z];

	}

	return res;
}

static int SC7A20_ReadRawData(struct i2c_client *client, char *buf)
{
	struct sc7a20_i2c_data *obj = (struct sc7a20_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
        GSE_ERR(" buf or client is null !!\n");
		return EINVAL;
	}

	if((res = SC7A20_ReadData(client, obj->data)))
	{
		GSE_ERR("I2C error: ret value=%d\n", res);
		return EIO;
	}
	else
	{
		buf[0] = (int)obj->data[SC7A20_AXIS_X];
		buf[1] = (int)obj->data[SC7A20_AXIS_Y];
		buf[2] = (int)obj->data[SC7A20_AXIS_Z];
	}

	return 0;
}


static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a20_i2c_client;
	char strbuf[SC7A20_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	SC7A20_ReadChipInfo(client, strbuf, SC7A20_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", (char*)strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a20_i2c_client;
	int strbuf[SC7A20_BUFSIZE] = {0};

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	SC7A20_ReadSensorData(client, strbuf, SC7A20_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", (char*)strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a20_i2c_client;
	struct sc7a20_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[SC7A20_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return SC7A20_ERR_STATUS;
	}

	obj = i2c_get_clientdata(client);



	if((err = SC7A20_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if((err = SC7A20_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{
		mul = obj->reso->sensitivity/sc7a20_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
			obj->offset[SC7A20_AXIS_X], obj->offset[SC7A20_AXIS_Y], obj->offset[SC7A20_AXIS_Z],
			obj->offset[SC7A20_AXIS_X], obj->offset[SC7A20_AXIS_Y], obj->offset[SC7A20_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			obj->cali_sw[SC7A20_AXIS_X], obj->cali_sw[SC7A20_AXIS_Y], obj->cali_sw[SC7A20_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			obj->offset[SC7A20_AXIS_X]*mul + obj->cali_sw[SC7A20_AXIS_X],
			obj->offset[SC7A20_AXIS_Y]*mul + obj->cali_sw[SC7A20_AXIS_Y],
			obj->offset[SC7A20_AXIS_Z]*mul + obj->cali_sw[SC7A20_AXIS_Z],
			tmp[SC7A20_AXIS_X], tmp[SC7A20_AXIS_Y], tmp[SC7A20_AXIS_Z]);

		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = sc7a20_i2c_client;
	int err, x, y, z;
	int dat[SC7A20_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = SC7A20_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[SC7A20_AXIS_X] = x;
		dat[SC7A20_AXIS_Y] = y;
		dat[SC7A20_AXIS_Z] = z;
		if((err = SC7A20_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}
	}
	else
	{
		GSE_ERR("invalid format\n");
	}

	return 0;
}
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_SC7A20_LOWPASS
	struct i2c_client *client = sc7a20_i2c_client;
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
	 	int idx, len = atomic_read(&obj->firlen);
	 	GSE_DEBUG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_INFO("[%5d %5d %5d]\n", obj->fir.raw[idx][SC7A20_AXIS_X], obj->fir.raw[idx][SC7A20_AXIS_Y], obj->fir.raw[idx][SC7A20_AXIS_Z]);
		}

		GSE_INFO("sum = [%5d %5d %5d]\n", obj->fir.sum[SC7A20_AXIS_X], obj->fir.sum[SC7A20_AXIS_Y], obj->fir.sum[SC7A20_AXIS_Z]);
		GSE_INFO("avg = [%5d %5d %5d]\n", obj->fir.sum[SC7A20_AXIS_X]/len, obj->fir.sum[SC7A20_AXIS_Y]/len, obj->fir.sum[SC7A20_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_SC7A20_LOWPASS
	struct i2c_client *client = sc7a20_i2c_client;
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return 0;
}
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct sc7a20_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct sc7a20_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
	    GSE_ERR("invalid content: '%s', length = %ld\n", buf, (long int)count);
	}
	return count;
}
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct sc7a20_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{

	if(sensor_power)
		GSE_INFO("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_INFO("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", sensor_power);
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a20_i2c_client;
	struct sc7a20_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = sc7a20_i2c_client;
	struct sc7a20_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			GSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			GSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			GSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_reg_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = sc7a20_i2c_client;
	int count = 0;
	//u8 reg[0x5b];
    
	

	char i;	
	char buffer;
	i = 0x0f;
    //*buffer = i;
	sc7a20_i2c_read_block(client, 0xf, &buffer, 1);
    count += sprintf(buf, "0x%x: 0x%x\n", i, buffer);
    for(i=0x0e;i<0x5b;i++)
    {
		//*buffer = i;
	
    	//sensor_rx_data(sc7a30_client, i, 1); 
		sc7a20_i2c_read_block(client, i, &buffer, 1);
		count += sprintf(&buf[count],"0x%x: 0x%x\n", i, buffer);
    }   
    return count;   
}
/*----------------------------------------------------------------------------*/
static ssize_t store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct i2c_client *client = sc7a20_i2c_client;
	int address, value;
    int result = 0;
	char databuf[2]={0};
    sscanf(buf, "0x%x=0x%x", &address, &value);    
	
	databuf[1] = value;
	databuf[0] = address;
	

	result = i2c_master_send(client, databuf, 0x2);
    
   // result = sensor_write_reg(sc7a30_client, address,value);

    if(result)
		GSE_ERR("%s:fail to write sensor_register\n",__func__);

    return count;    
}
static DRIVER_ATTR(chipinfo,	S_IWUSR | S_IRUGO, show_chipinfo_value,		NULL);
static DRIVER_ATTR(sensordata,	S_IWUSR | S_IRUGO, show_sensordata_value,	NULL);
static DRIVER_ATTR(cali,	S_IWUSR | S_IRUGO, show_cali_value,		store_cali_value);
static DRIVER_ATTR(firlen,	S_IWUSR | S_IRUGO, show_firlen_value,		store_firlen_value);
static DRIVER_ATTR(trace,	S_IWUSR | S_IRUGO, show_trace_value,		store_trace_value);
static DRIVER_ATTR(layout, 	S_IRUGO | S_IWUSR, show_layout_value,		store_layout_value);
static DRIVER_ATTR(status, 	S_IRUGO, show_status_value,			NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value,		NULL);
static DRIVER_ATTR(reg,        S_IWUSR | S_IRUGO, show_reg_value,         store_reg_value);

static struct driver_attribute *sc7a20_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,	   /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,		   /*trace log*/
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_reg,
};
static int sc7a20_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(sc7a20_attr_list)/sizeof(sc7a20_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, sc7a20_attr_list[idx])))
		{
			GSE_ERR("driver_create_file (%s) = %d\n", sc7a20_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
static int sc7a20_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(sc7a20_attr_list)/sizeof(sc7a20_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, sc7a20_attr_list[idx]);
	}

	return err;
}



//#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM_SLEEP

static int sc7a20_suspend(struct device *dev)
{
	 struct i2c_client *client = to_i2c_client(dev);
	 struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	 int err = 0;

	 //if(msg.event == PM_EVENT_SUSPEND)
	 {
		 if(obj == NULL)
		 {
			 GSE_ERR("null sc7a20!!\n");
			 return -EINVAL;
		 }
		 mutex_lock(&sc7a20_mutex);
		 atomic_set(&obj->suspend, 1);
		 if(!enable_status)
		 {
			err = SC7A20_SetPowerMode(client, false);
			if(err)
			{
				GSE_ERR("write power control fail!!\n");
				mutex_unlock(&sc7a20_mutex);
				return -EINVAL;
			}
		 }
		 mutex_unlock(&sc7a20_mutex);
	 }
	 return err;
}

static int sc7a20_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc7a20_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;


	if(obj == NULL)
	{
		GSE_ERR("null sc7a20!!\n");
		return -EINVAL;
	}
	mutex_lock(&sc7a20_mutex);
	if(enable_status)
	{
		err = SC7A20_SetPowerMode(client, true);
		if(err != SC7A20_SUCCESS)
		{
			GSE_ERR("Set PowerMode fail!!\n");
			mutex_unlock(&sc7a20_mutex);
			return -EINVAL;
		}
	}
	atomic_set(&obj->suspend, 0);
	mutex_unlock(&sc7a20_mutex);
	return err;
}


#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void sc7a20_early_suspend(struct early_suspend *h)
{
	 struct sc7a20_i2c_data *obj = container_of(h, struct sc7a20_i2c_data, early_drv);
	 int err;

	 if(obj == NULL)
	 {
		 GSE_ERR("null sc7a20!!\n");
		 return;
	 }
	mutex_lock(&sc7a20_mutex);
	atomic_set(&obj->suspend, 1);
	if(!enable_status)
	{
		if(err = SC7A20_SetPowerMode(obj->client, false))
		{
			GSE_ERR("write power control fail!!\n");
			mutex_unlock(&sc7a20_mutex);
			return;
		}
	}
	mutex_unlock(&sc7a20_mutex);
	sc7a20_power(obj->hw, 0);
}

static void sc7a20_late_resume(struct early_suspend *h)
{
	 struct sc7a20_i2c_data *obj = container_of(h, struct sc7a20_i2c_data, early_drv);
	 int err;

	 if(obj == NULL)
	 {
		 GSE_ERR("null sc7a20!!\n");
		 return;
	 }

	 sc7a20_power(obj->hw, 1);
	 mutex_lock(&sc7a20_mutex);
	 if(enable_status)
	{
		err = SC7A20_SetPowerMode(client, true);
		if(err != SC7A20_SUCCESS)
		{
			GSE_ERR("Set PowerMode fail!!\n");
			mutex_unlock(&sc7a20_mutex);
			return -EINVAL;
		}
	}
	 atomic_set(&obj->suspend, 0);
	 mutex_unlock(&sc7a20_mutex);
}
#endif 
#endif /*CONFIG_PM_SLEEP*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int sc7a20_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int sc7a20_enable_nodata(int en)
{
	int res =0;
	bool power = false;

	if(1==en)
	{
		power = true;
		atomic_set(&open_flag, 1);
	}
	if(0==en)
	{
		power = false;
		atomic_set(&open_flag, 0);
	}
	res = SC7A20_SetPowerMode(obj_i2c_data->client, power);
	if(res != SC7A20_SUCCESS)
	{
		GSE_ERR("SC7A20_SetPowerMode fail!\n");
		return -1;
	}
	GSE_DEBUG("sc7a20_enable_nodata OK en = %d sensor_power = %d\n", en, sensor_power);
	enable_status = en;
	if(enable_status)
	{
		wake_up(&open_wq);
	}
	return 0;
}

static int sc7a20_set_delay(u64 ns)
{
	return 0;
}

static int sc7a20_get_data(int* x ,int* y,int* z, int* status)
{
	int buff[SC7A20_BUFSIZE] = {0};
	SC7A20_ReadSensorData(obj_i2c_data->client, buff, SC7A20_BUFSIZE);
	*x = buff[0];
	*y = buff[1];
	*z = buff[2];   //djq,2017-12-28

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
static int sc7a20_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int sc7a20_flush(void)
{
	int err = 0;
	/*Only flush after sensor was enabled*/
	if (!sensor_power) {
		obj_i2c_data->flush = true;
		return 0;
	}
	err = acc_flush_report();
	if (err >= 0)
		obj_i2c_data->flush = false;
	return err;
}

static int sc7a20_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = sc7a20_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		GSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = sc7a20_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int sc7a20_factory_get_data(int32_t data[3], int *status)
{
	return sc7a20_get_data(&data[0], &data[1], &data[2], status);

}
static int sc7a20_factory_get_raw_data(int32_t data[3])
{
	char strbuf[SC7A20_BUF_SIZE] = { 0 };

	SC7A20_ReadRawData(sc7a20_i2c_client, strbuf);
	GSE_ERR("support sc7a20_factory_get_raw_data!\n");
	return 0;
}
static int sc7a20_factory_enable_calibration(void)
{
	return 0;
}
static int sc7a20_factory_clear_cali(void)
{
	int err = 0;

	err = SC7A20_ResetCalibration(sc7a20_i2c_client);
	if (err) {
		GSE_ERR("SC7A20_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int sc7a20_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };

	/* obj */
	//obj_i2c_data->cali_sw[SC7A20_AXIS_X] += data[0];
	//obj_i2c_data->cali_sw[SC7A20_AXIS_Y] += data[1];
	//obj_i2c_data->cali_sw[SC7A20_AXIS_Z] += data[2];

	cali[SC7A20_AXIS_X] = data[0] * gsensor_gain.x / GRAVITY_EARTH_1000;
	cali[SC7A20_AXIS_Y] = data[1] * gsensor_gain.y / GRAVITY_EARTH_1000;
	cali[SC7A20_AXIS_Z] = data[2] * gsensor_gain.z / GRAVITY_EARTH_1000;
	err = SC7A20_WriteCalibration(sc7a20_i2c_client, cali);
	if (err) {
		GSE_ERR("SC7A20_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}
static int sc7a20_factory_get_cali(int32_t data[3])
{
	data[0] = obj_i2c_data->cali_sw[SC7A20_AXIS_X];
	data[1] = obj_i2c_data->cali_sw[SC7A20_AXIS_Y];
	data[2] = obj_i2c_data->cali_sw[SC7A20_AXIS_Z];
	return 0;
}
static int sc7a20_factory_do_self_test(void)
{
	return 0;
}

#if 0//wff
static int sc7a20_factory_read_reg(int32_t data[2])
{
	u8 reg[2];
	int err = 0;
	reg[0] = data[0];
	reg[1] = data[1];
	err = sc7a20_i2c_read_block(sc7a20_i2c_client, reg[0], &reg[1], 1);
		if(err <= 0)
			{
				GSE_ERR("read reg failed!\n");
				err = -EFAULT;
				
			}
	return err;
}

static int sc7a20_factory_write_reg(int32_t data[2])
{
	u8 reg[2];
	int err = 0;
	reg[0] = data[0];
	reg[1] = data[1];
	err = i2c_master_send(sc7a20_i2c_client, reg, 0x2);
	if(err <= 0)
			{
				GSE_ERR("write reg failed!\n");
				err = -EFAULT;
				
			}
	return err;
}

static int sc7a20_factory_get_layout(void)
{
	
	return hw->direction;
}

static int sc7a20_factory_set_data(int32_t data[3])
{

    SC7A20_SaveData(data);
	return 0;
}
/*
static int sc7a20_factory_set_temp(void)
{
	
	u8 addr = SC7A20_REG_TEMP;  //not defined
	int err = 0;
	s8 temp = 0;

	
	if((err = sc7a20_i2c_read_block(sc7a20_i2c_client, addr, &temp, 1)))
	{
		GSE_ERR("error: %d\n", err);
	}
	return err;
}
*/
static int sc7a20_factory_read_sensor_data(void)
{
char strbuf[SC7A20_BUFSIZE] = {0};
		if(!sensor_power)
			{
				SC7A20_SetPowerMode(sc7a20_i2c_client,true);
			}
			SC7A20_ReadSensorDataFactory(sc7a20_i2c_client, strbuf, SC7A20_BUFSIZE);
			return 0;
}
#endif		

static struct accel_factory_fops sc7a20_factory_fops = {
	.enable_sensor = sc7a20_factory_enable_sensor,
	.get_data = sc7a20_factory_get_data,
	.get_raw_data = sc7a20_factory_get_raw_data,
	.enable_calibration = sc7a20_factory_enable_calibration,
	.clear_cali = sc7a20_factory_clear_cali,
	.set_cali = sc7a20_factory_set_cali,
	.get_cali = sc7a20_factory_get_cali,
	.do_self_test = sc7a20_factory_do_self_test,
};

static struct accel_factory_public sc7a20_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &sc7a20_factory_fops,
};


/*----------------------------------------------------------------------------*/
static int sc7a20_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct sc7a20_i2c_data *obj;

	int err = 0;
	int i = 0;

	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
	//GSE_DEBUG_FUNC();
	GSE_INFO("driver version = %s\n",DRIVER_VERSION);

	GSE_INFO("<<-GSE INFO->> client->dev.of_node->name = %s\n",client->dev.of_node->name);
	err = get_accel_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		GSE_ERR("get cust_baro dts info fail\n");
		goto exit_kfree;
	}
	
	
	GSE_INFO("--------------> along client->addr = %x line = %d\n", client->addr, __LINE__);
//	printk("--------------> along sc7a20 hw->i2c_num=0x%x\n",hw->i2c_num);
	GSE_INFO("--------------> along sc7a20 hw->i2c_addr[0]=0x%x\n",hw->i2c_addr[0]);
	
	for (i = 0; i < G_CUST_I2C_ADDR_NUM; i++) {
		client->addr = hw->i2c_addr[i];
		err = SC7A20_CheckDeviceID(client);
		if (SC7A20_SUCCESS == err) break;
	}
	if(err != SC7A20_SUCCESS)
	{
		 goto exit;
	}
//	printk("<<-GSE INFO->> sc7a20 hw->i2c_num=0x%x\n",hw->i2c_num);
//	printk("<<-GSE INFO->> sc7a20 hw->i2c_addr[0]=0x%x\n",hw->i2c_addr[0]);
//	printk("<<-GSE INFO->> sc7a20 hw->direction=0x%x\n",hw->direction);

	hw->direction = 6;//1 2 4 3 5
//	printk("<<along>> sc7a20 hw->direction=0x%x\n",hw->direction);
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct sc7a20_i2c_data));

	obj->hw = hw;
	atomic_set(&obj->layout, obj->hw->direction);
	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	init_waitqueue_head(&open_wq);

#ifdef CONFIG_SC7A20_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}

	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}

#endif
	sc7a20_i2c_client = new_client;

	if((err = sc7a20_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}

	if((err = accel_factory_device_register(&sc7a20_factory_device)))
	{
		GSE_ERR("sc7a20_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = sc7a20_create_attr(&sc7a20_init_info.platform_diver_addr->driver)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = sc7a20_open_report_data;
	ctl.enable_nodata = sc7a20_enable_nodata;
	ctl.set_delay  = sc7a20_set_delay;
	ctl.batch = sc7a20_batch;
	ctl.flush = sc7a20_flush;
	ctl.is_report_input_direct = false;

	err = acc_register_control_path(&ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = sc7a20_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level	 = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = sc7a20_early_suspend,
	obj->early_drv.resume	 = sc7a20_late_resume,
	register_early_suspend(&obj->early_drv);
#endif
	sc7a20_init_flag = 0;
	GSE_INFO("%s: OK\n", __func__);
	app_get_g_sensor_name("sc7a20");
	return 0;

exit_create_attr_failed:
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	sc7a20_init_flag = -1;
	return err;
}

static int sc7a20_i2c_remove(struct i2c_client *client)
{
	 int err = 0;

	 if((err = sc7a20_delete_attr(&sc7a20_init_info.platform_diver_addr->driver)))
	 {
		 GSE_ERR("sc7a20_delete_attr fail: %d\n", err);
	 }

	 sc7a20_i2c_client = NULL;
	 i2c_unregister_device(client);
	 accel_factory_device_deregister(&sc7a20_factory_device);
	 kfree(i2c_get_clientdata(client));
	 return 0;
}


static int sc7a20_local_init(void)
{
	GSE_DEBUG_FUNC();
	sc7a20_power(hw, 1);

	if(i2c_add_driver(&sc7a20_i2c_driver))
	{
		 GSE_ERR("add driver error\n");
		 return -1;
	}
	if(-1 == sc7a20_init_flag)
	{
		GSE_ERR("sc7a20_local_init failed sc7a20_init_flag=%d\n",sc7a20_init_flag);
	   	return -1;
	}
	return 0;
}
static int sc7a20_remove(void)
{
	 GSE_DEBUG_FUNC();
	 sc7a20_power(hw, 0);
	 i2c_del_driver(&sc7a20_i2c_driver);
	 return 0;
}

static int __init sc7a20_driver_init(void)
{

	GSE_DEBUG_FUNC();
	acc_driver_add(&sc7a20_init_info);

	mutex_init(&sc7a20_mutex);

	return 0;
}

static void __exit sc7a20_driver_exit(void)
{
	GSE_DEBUG_FUNC();
	mutex_destroy(&sc7a20_mutex);
}

module_init(sc7a20_driver_init);
module_exit(sc7a20_driver_exit);


MODULE_AUTHOR("Jianqing Dong<dongjianqing@silan.com.cn>");
MODULE_DESCRIPTION("SILAN SC7A20 Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");
