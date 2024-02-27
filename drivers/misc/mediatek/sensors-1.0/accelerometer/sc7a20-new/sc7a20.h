/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Jianqing Dong
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for SC7A20 accelorometer sensor chip.
 */
#ifndef __SC7A20_H__
#define __SC7A20_H__


#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>



/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/
#define SC7A20_DEV_NAME		"SC7A20"

#define SC7A20_I2C_ADDR		0x18//0x19//0x18
#define SC7A20_ID_1			0x11


/* SC7A20 register address */
#define SC7A20_REG_CTL_REG1		0x20
#define SC7A20_REG_CTL_REG4     0x23
#define SC7A20_REG_X			0x28
#define SC7A20_REG_Y			0x2a
#define SC7A20_REG_Z			0x2c


#define SC7A20_REG_ID			0x0F


/*para setting*/
#define SC7A20_AWAKE			0x50	/* power on */
#define SC7A20_SLEEP			0x00	/* power donw */
#define SC7A20_POWER_MODE_MASK  0xf0	/* power mask */

#define SC7A20_BW	            0xf0
#define SC7A20_BW_400HZ			0x70 //400 or 100 on other choise //changed
#define SC7A20_BW_200HZ			0x60
#define SC7A20_BW_100HZ			0x50
#define SC7A20_BW_50HZ			0x40



#define SC7A20_RANGE			0x30   //8g or 2g no ohter choise//changed
#define SC7A20_RANGE_2G			0x00 	

#define SC7A20_RANGE_8G			0x20 



/*ERR code*/
#define SC7A20_SUCCESS			0
#define SC7A20_ERR_I2C			-1
#define SC7A20_ERR_STATUS		-3
#define SC7A20_ERR_SETUP_FAILURE	-4
#define SC7A20_ERR_GETGSENSORDATA	-5
#define SC7A20_ERR_IDENTIFICATION	-6

#define SC7A20_BUFSIZE			256
#define SC7A20_STABLE_DELAY	10


#endif /* __SC7A20_H__ */

