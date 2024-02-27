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

#ifndef _INV_REG_42607_H_
#define _INV_REG_42607_H_

/*bank 0 register*/
#define REG_MCLK_RDY				0x00
#define REG_CHIP_CONFIG_REG         0x02
#define REG_DRIVE_CONFIG2			0x04
#define REG_DRIVE_CONFIG3			0x05
#define REG_INT_CONFIG				0x06
#define REG_ACCEL_DATA_X0_UI        0x0B
#define REG_GYRO_DATA_X0_UI         0x11
#define REG_PWR_MGMT_0              0x1F
#define REG_GYRO_CONFIG0            0x20
#define REG_ACCEL_CONFIG0           0x21
#define REG_GYRO_CONFIG1            0x23
#define REG_ACCEL_CONFIG1           0x24
#define REG_APEX_CONFIG0            0x25
#define REG_APEX_CONFIG1            0x26
#define REG_WOM_CONFIG				0x27
#define REG_INT_SOURCE1             0x2C
#define REG_APEX_DATA0				0x31
#define REG_APEX_DATA1				0x32
#define REG_INTF_CONFIG1            0x36
#define REG_INT_STATUS				0x3A
#define REG_INT_STATUS2             0x3B
#define REG_INT_STATUS3             0x3C
#define REG_WHO_AM_I                0x75
#define REG_BLK_SEL_W				0x79
#define REG_MADDR_W					0x7A
#define REG_M_W						0x7B
#define REG_BLK_SEL_R				0x7C
#define REG_MADDR_R					0x7D
#define REG_M_R						0x7E
/*bank 1 register*/
#define REG_SELF_TEST_CONFIG		0x14
#define REG_INT_SOURCE6				0x2F
#define REG_APEX_CONFIG2			0X44
#define REG_APEX_CONFIG3			0X45
#define REG_APEX_CONFIG4			0X46
#define REG_APEX_CONFIG9			0X48
#define REG_ACCEL_WOM_X_THR			0x4B
#define REG_ACCEL_WOM_Y_THR			0x4C
#define REG_ACCEL_WOM_Z_THR			0x4D
/*bank 3 register*/
#define REG_XA_ST_DATA              0x00
#define REG_YA_ST_DATA              0x01
#define REG_ZA_ST_DATA              0x02
#define REG_XG_ST_DATA              0x03
#define REG_YG_ST_DATA              0x04
#define REG_ZG_ST_DATA              0x05

/* REG_WHO_AM_I */
#define WHO_AM_I_ICM42607P			0x60
#define WHO_AM_I_ICM42607C			0x61

/* REG_REG_BANK_SEL */
#define BIT_BLK_SEL_1				0x00
#define BIT_BLK_SEL_2				0x28
#define BIT_BLK_SEL_3				0x50

/* REG_CHIP_CONFIG_REG */
#define BIT_SOFT_RESET              0x10

/* REG_GYRO_CONFIG0/REG_ACCEL_CONFIG0 */
#define SHIFT_GYRO_FS_SEL           5
#define SHIFT_ACCEL_FS_SEL          5
#define BIT_GYRO_FSR                0xE0
#define BIT_GYRO_ODR                0x0F
#define BIT_ACCEL_FSR               0xE0
#define BIT_ACCEL_ODR               0x0F

/* REG_GYRO_CONFIG1/REG_ACCEL_CONFIG1 */
#define BIT_ACCEL_FILTER            0xFF
#define BIT_GYRO_FILTER             0x0F

/* REG_PWR_MGMT_0 */
#define BIT_GYRO_MODE_OFF           0x00
#define BIT_GYRO_MODE_STBY          0x04
#define BIT_GYRO_MODE_LNM           0x0C
#define BIT_ACCEL_MODE_OFF          0x00
#define BIT_ACCEL_MODE_LPM          0x02
#define BIT_ACCEL_MODE_LNM          0x03


/* REG_INT_SOURCE1 */
#define BIT_INT_WOM_X_INT1_EN       0x01
#define BIT_INT_WOM_Y_INT1_EN       0x02
#define BIT_INT_WOM_Z_INT1_EN       0x04
#define BIT_INT_SMD_INT1_EN         0x08
#define BIT_INT_WOM_XYZ_INT1_EN     \
    (BIT_INT_WOM_X_INT1_EN | BIT_INT_WOM_Y_INT1_EN | BIT_INT_WOM_Z_INT1_EN)

/*INT_SOURCE6 */
#define BIT_STEP_DET_INT1_EN		0x20
#define BIT_STEP_CNT_OFL_INT1_EN	0x10

/* REG_SELF_TEST_CONFIG */
#define BIT_GYRO_ST_EN				0x80
#define BIT_ACCEL_ST_EN				0x40

/* REG_INT_STATUS2 */
#define BIT_INT_STATUS_WOM_X        0x04
#define BIT_INT_STATUS_WOM_Y        0x02
#define BIT_INT_STATUS_WOM_Z        0x01
#define BIT_INT_STATUS_SMD          0x08
#define BIT_INT_STATUS_WOM_XYZ      \
    (BIT_INT_STATUS_WOM_X | BIT_INT_STATUS_WOM_Y | BIT_INT_STATUS_WOM_Z)

/* REG_INT_STATUS3 */
#define BIT_STEP_DET_INT			0x20
#define BIT_STEP_CNT_OVF_INT		0x10

/* REG_ACCEL_CONFIG1 */
#define BIT_ACCEL_UI_LNM_NO_FILTER  0x00
#define BIT_ACCEL_UI_LNM_BW_180HZ   0x01
#define BIT_ACCEL_UI_LNM_BW_121HZ   0x02
#define BIT_ACCEL_UI_LNM_BW_73HZ    0x03
#define BIT_ACCEL_UI_LNM_BW_53HZ    0x04
#define BIT_ACCEL_UI_LNM_BW_34HZ    0x05
#define BIT_ACCEL_UI_LNM_BW_25HZ    0x06
#define BIT_ACCEL_UI_LNM_BW_16HZ    0x07
#define BIT_ACCEL_UI_LPM_AVG_2      0x00
#define BIT_ACCEL_UI_LPM_AVG_4      0x10
#define BIT_ACCEL_UI_LPM_AVG_8      0x20
#define BIT_ACCEL_UI_LPM_AVG_16     0x30
#define BIT_ACCEL_UI_LPM_AVG_32     0x40
#define BIT_ACCEL_UI_LPM_AVG_64     0x50

/* REG_GYRO_CONFIG1 */
#define BIT_GYRO_UI_LNM_NO_FILTER   0x00
#define BIT_GYRO_UI_LNM_BW_180HZ    0x01
#define BIT_GYRO_UI_LNM_BW_121HZ    0x02
#define BIT_GYRO_UI_LNM_BW_73HZ     0x03
#define BIT_GYRO_UI_LNM_BW_53HZ     0x04
#define BIT_GYRO_UI_LNM_BW_34HZ     0x05
#define BIT_GYRO_UI_LNM_BW_25HZ     0x06
#define BIT_GYRO_UI_LNM_BW_16HZ     0x07


/* REG_WOM_CONFIG */
#define BIT_WOM_MODE_OFF			0x00
#define BIT_WOM_MODE_ON				0x01
#define BIT_WOM_INT_DUR1			0x01
#define BIT_WOM_INT_DUR3			0x03

/* REG_DMP_CONFIG_MASK  */
//#define DMP_ODR_100HZ               0x03
#define DMP_ODR_50HZ                0x02
#define DMP_ODR_25HZ                0x00

/* DMP_ODR */
#define BIT_APEX_CONFIG0_DMP_ODR_POS       0
#define BIT_APEX_CONFIG0_DMP_ODR_MASK   (0x3 << BIT_APEX_CONFIG0_DMP_ODR_POS)

#define DMP_INIT_EN                 0x04
#define DMP_MEM_RESET_EN            0x01

#define PED_ENABLE                  0x08

/* I2C BUS CONFIG_MASK in BLANK1 */
#define BIT_I3C_SDR_EN				0x08
#define BIT_I3C_DDR_EN				0x04

/* REG_INT_CONFIG */
#define SHIFT_INT1_POLARITY         0
#define SHIFT_INT1_DRIVE_CIRCUIT    1
#define SHIFT_INT1_MODE             2

/* INT configurations */
// Polarity: 0 -> Active Low, 1 -> Active High
#define INT_POLARITY    1
// Drive circuit: 0 -> Open Drain, 1 -> Push-Pull
#define INT_DRIVE_CIRCUIT    1
// Mode: 0 -> Pulse, 1 -> Latch
#define INT_MODE    0

#define GSE_DEBUG_ON          		0

#if GSE_DEBUG_ON
/* Log define */
#define GSE_INFO(fmt, arg...)      	printk("<<-GSE INFO->> "fmt"\n", ##arg)
#define GSE_ERR(fmt, arg...)          	printk("<<-GSE ERROR->> "fmt"\n", ##arg)
#define GSE_FUNC(fmt, arg...)               printk("<<-GSE FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);

#else
//#define GSE_TAG
#define GSE_INFO(fmt, args...)	do {} while (0)
#define GSE_ERR(fmt, args...)	do {} while (0)
#define GSE_FUNC(fmt, arg...)   do {} while (0) 

#endif


#endif /* _INV_REG_42607_H_ */
