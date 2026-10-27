// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "imx766_pdafotp"
#define LOG_INF(format, args...) printk(PFX "[%s] " format, __func__, ##args)


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imx766mipiraw_Sensor.h"

#define Sleep(ms) mdelay(ms)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

BYTE imx766_QSC_data[3080]= {0};
BYTE imx766_step_data[1500]={0};


#define USHORT             unsigned short
#define BYTE               unsigned char

#define EEPROM_WRITE_ID         0xA0

#define OTP_DATA                imx766_eeprom_data
#define OTP_PLATFORM_CHOICE     0x0b     //bit0:awb, bit1:af, bit3:lsc  1010
#define OTP_FLAG_ADDR           0x0000
#define AWB_ADDR                0x000B
#define AWB_CHECK_ADDR          0x001B
#define LSC_ADDR                0x001D
#define LSC_CHECK_ADDR          0x0769
#define AF_ADDR                 0x076B
#define AF_CHECK_ADDR           0x076F


#define PD1_START_ADDR		0x0907
#define PD1_END_ADDR    	0x0AF6
#define PD2_START_ADDR		0x0AFB
#define PD2_END_ADDR    	0x0EE6


#define QSC_ADDR		0x109B
#define QSC_CHECK_ADDR		0x1C9B


#define CHECKSUM_METHOD(x,addr)  \
( ((x) % 255 + 1)  == read_cmos_sensor_8(addr) )

BYTE OTP_DATA[2048]= {0};
//BYTE imx766_PDAF_data[1411] = {0};
BYTE imx766_PDAF_data[1500] = {0};
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,EEPROM_WRITE_ID);
    return get_byte;
}

static void read_imx766_QSC(void)
{
	int size = 3072;
	int addr = QSC_ADDR;
	int i;
	kal_uint32 checksum = 0;

	LOG_INF("read imx766 QSC, size = %d   QSC-flag(0x%x)=%d\n", size,QSC_ADDR-1,(read_cmos_sensor_8(QSC_ADDR-1)));

	for(i = 0; i < size; i++)
	{
		imx766_QSC_data[i] = read_cmos_sensor_8(addr + i);
		checksum += imx766_QSC_data[i];
		//printk("carrot: I:[%d] QSC(0x%x)=%d\n", i,addr + i,(read_cmos_sensor_8(addr + i)));
	}


	printk("carrot: [%s] QSC-checksum=%d QSC_CHECK_ADDR(0x%x) = %d\n",__func__, checksum, QSC_CHECK_ADDR,
			 (read_cmos_sensor_8(QSC_CHECK_ADDR)));

	if(CHECKSUM_METHOD(checksum, QSC_CHECK_ADDR))
		LOG_INF("QSC Checksum Success");
	else
		LOG_INF("QSC Checksum Failed!!!");

}


int imx766_get_otp_data(void)
{

	int ret = 1;
  if(0x01 != read_cmos_sensor_8(OTP_FLAG_ADDR))
	{
        LOG_INF("OTP DATA Invalid!!!\n");
        return 0;
  }
	read_imx766_QSC();

	return ret;
}


//xqadd fot otp
#if 1//k510
 unsigned int  imx766_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size)
{

	if(addr > 2048)
	{
		printk("??? \n");
  		return 0;
	}

	if(addr == 2048)
	{
		memcpy((void *)data,(void *)&imx766_PDAF_data[0],size);
	}
	else
	{
    	memcpy((void *)data,(void *)&imx766_eeprom_data[addr],size);//addr = 4003 over size le
    	}
	printk("imx766_selective_read_region addr:%d,size %d data read = 0x%x\n",addr,size, *data);
    return size;
}

//xqadd fot otp



EXPORT_SYMBOL(imx766_selective_read_region);
#endif
