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

#include <hwmsensor.h>
#include "cust_acc.h"
#include "accel.h"
#include "icm42607_register.h"
#include "icm42607_share_interface.h"

#define RETRY_CNT 2

extern struct i2c_client *icm42607_accel_i2c_client;

static DEFINE_MUTEX(icm42607_accel_i2c_mutex);

static int need_mclk_cnt = 0;

static int icm42607_i2c_read_register(struct i2c_client *client,
    u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int res = 0;
    struct i2c_msg msgs[2] = {{0}, {0} };
    int retry = RETRY_CNT;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &beg;
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    if (!client) {
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }
    do {
        res = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
        if (res != 2) {
            GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",
                addr, data, len, res);
            res = -EIO;
        } else
            res = 0;
    } while (res !=0 && --retry > 0);
    return res;
}

static int icm42607_i2c_write_register(struct i2c_client *client,
    u8 addr, u8 *data, u8 len)
{
    /*because address also occupies one byte,
    the maximum length for write is 7 bytes*/
    int idx, num;
    int res = 0;
    char buf[C_I2C_FIFO_SIZE];
    int retry = RETRY_CNT;

    if (!client) {
        return -EINVAL;
    } else if (len >= C_I2C_FIFO_SIZE) {
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        return -EINVAL;
    }
    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
        buf[num++] = data[idx];
    do {
        res = i2c_master_send(client, buf, num);
        if (res < 0) {
            GSE_ERR("send command error!!\n");
            return -EFAULT;
        } else
            res = 0;
    } while (res !=0 && --retry > 0);
    return res;
}

int icm42607_share_read_register(u8 addr, u8 *data, u8 len)
{
    int ret;

    mutex_lock(&icm42607_accel_i2c_mutex);
    ret = icm42607_i2c_read_register(icm42607_accel_i2c_client,
        addr, data, len);
    mutex_unlock(&icm42607_accel_i2c_mutex);
    return ret;
}
EXPORT_SYMBOL(icm42607_share_read_register);

int icm42607_share_write_register(u8 addr, u8 *data, u8 len)
{
    int ret;

    mutex_lock(&icm42607_accel_i2c_mutex);
    ret = icm42607_i2c_write_register(icm42607_accel_i2c_client,
        addr, data, len);
    mutex_unlock(&icm42607_accel_i2c_mutex);
    return ret;
}
EXPORT_SYMBOL(icm42607_share_write_register);

int icm42607_switch_on_mclk(void)
{
	int res = 0;
	u8 regdata = 0;

	/* set IDLE bit only if it is not set yet */
	if (need_mclk_cnt == 0) {
		res = icm42607_share_read_register(REG_PWR_MGMT_0, &regdata, 1);
    	if (res < 0) {
        	GSE_ERR("Read REG_PWR_MGMT_0 failed: %d\n", res);
        	return ICM42607_ERR_BUS;
    	}
		regdata |= 0x10;
		res = icm42607_share_write_register(REG_PWR_MGMT_0, &regdata, 1);
    	if (res < 0) {
        	GSE_ERR("Write REG_PWR_MGMT_0 failed: %d\n", res);
        	return ICM42607_ERR_BUS;
    	}
    	
    	/* Check if MCLK is ready */
    	do {
			res = icm42607_share_read_register(REG_MCLK_RDY, &regdata, 1);
	    	if (res < 0) {
	        	GSE_ERR("Read REG_MCLK_RDY failed: %d\n", res);
	        	return ICM42607_ERR_BUS;
	    	}
    	} while (!(regdata & 0x08));
	} else {
		/* Make sure it is already on */
		res = icm42607_share_read_register(REG_PWR_MGMT_0, &regdata, 1);
    	if (res < 0) {
        	GSE_ERR("Read REG_PWR_MGMT_0 failed: %d\n", res);
        	return ICM42607_ERR_BUS;
    	}
    	if (!(regdata & 0x10)) {
    		res |= ICM42607_ERR_STATUS;
        	//GSE_ERR("mismatch status in icm42607_switch_on_mclk!! %d count %d",regdata,need_mclk_cnt);
    	}
	}
	
	/* Increment the counter to keep track of number of MCLK requesters */
	need_mclk_cnt++;

	return res;
}

int icm42607_switch_off_mclk(void)
{
	int res = 0;
	u8 regdata = 0;

	/* Reset the IDLE but only if there is one requester left */
	if (need_mclk_cnt == 1) {
		res = icm42607_share_read_register(REG_PWR_MGMT_0, &regdata, 1);
    	if (res < 0) {
        	GSE_ERR("Read REG_PWR_MGMT_0 failed: %d\n", res);
        	return ICM42607_ERR_BUS;
    	}
		regdata &= ~0x10;
		res = icm42607_share_write_register(REG_PWR_MGMT_0, &regdata, 1);
    	if (res < 0) {
        	GSE_ERR("Write REG_PWR_MGMT_0 failed: %d\n", res);
        	return ICM42607_ERR_BUS;
    	}
	} else {
		/* Make sure it is still on */
		res = icm42607_share_read_register(REG_PWR_MGMT_0, &regdata, 1);
    	if (res < 0) {
        	GSE_ERR("Read REG_PWR_MGMT_0 failed: %d\n", res);
        	return ICM42607_ERR_BUS;
    	}
    	if (!(regdata & 0x10)) {
    		res |= ICM42607_ERR_STATUS;
        	//GSE_ERR("mismatch status in icm42607_switch_on_mclk!! %d count %d",regdata,need_mclk_cnt);
    	}
	}
	
	/* Increment the counter to keep track of number of MCLK requesters */
	need_mclk_cnt--;

	return res;
}

int icm42607_share_read_blkreg(u8 blk, u8 addr, u8 *data, u8 len)
{
    int res = 0;
	u8 databuf[2] = {0};

	// Have IMU not in IDLE mode to access MCLK domain
	res = icm42607_switch_on_mclk();
	if (res < 0) {
        GSE_ERR("icm42607_switch_on_mclk err!\n");
        return res;
    }

	databuf[0] = blk;
    res = icm42607_share_write_register(REG_BLK_SEL_R, databuf, 1);
    if (res < 0) {
        GSE_ERR("write REG_BLK_SEL_R 0x%0x err!\n", blk);
        return ICM42607_ERR_BUS;
    }

	databuf[0] = addr;
	res = icm42607_share_write_register(REG_MADDR_R, databuf, 1);
    if (res < 0) {
    	GSE_ERR("write REG_MADDR_R 0x%0x: %d\n", addr, res);
    	return ICM42607_ERR_BUS;
    }

    res = icm42607_share_read_register(REG_M_R, data, len);
    if (res < 0) {
        GSE_ERR("Read REG_M_R 0x%0x failed: %d\n", addr, res);
        return ICM42607_ERR_BUS;
    }

	databuf[0] = 0;
    res = icm42607_share_write_register(REG_BLK_SEL_R, databuf, 1);
    if (res < 0) {
        GSE_ERR("write REG_BLK_SEL_R 0x00 err!\n");
        return ICM42607_ERR_BUS;
    }

	// switch OFF MCLK if needed
	res = icm42607_switch_off_mclk();
	if (res < 0) {
        GSE_ERR("icm42607_switch_off_mclk err!\n");
    }

    return res;
}
EXPORT_SYMBOL(icm42607_share_read_blkreg);

int icm42607_share_write_blkreg(u8 blk, u8 addr, u8 *data, u8 len)
{
    int res = 0;
	u8 databuf[2] = {0};

	// Have IMU not in IDLE mode to access MCLK domain
	res = icm42607_switch_on_mclk();
	if (res < 0) {
        GSE_ERR("icm42607_switch_on_mclk err!\n");
        return res;
    }

	databuf[0] = blk;
    res = icm42607_share_write_register(REG_BLK_SEL_W, databuf, 1);
    if (res < 0) {
        GSE_ERR("write REG_BLK_SEL_W 0x%0x err!\n", blk);
        return ICM42607_ERR_BUS;
    }

	databuf[0] = addr;
	res = icm42607_share_write_register(REG_MADDR_W, databuf, 1);
    if (res < 0) {
    	GSE_ERR("write REG_MADDR_W 0x%0x: %d\n", addr, res);
    	return ICM42607_ERR_BUS;
    }

    res = icm42607_share_write_register(REG_M_W, data, len);
    if (res < 0) {
        GSE_ERR("Read REG_M_W 0x%0x failed: %d\n", addr, res);
        return ICM42607_ERR_BUS;
    }

	databuf[0] = 0;
    res = icm42607_share_write_register(REG_BLK_SEL_W, databuf, 1);
    if (res < 0) {
        GSE_ERR("write REG_BLK_SEL_W 0x00 err!\n");
        return ICM42607_ERR_BUS;
    }

	// switch OFF MCLK if needed
	res = icm42607_switch_off_mclk();
	if (res < 0) {
        GSE_ERR("icm42607_switch_off_mclk err!\n");
    }

    return res;
}
EXPORT_SYMBOL(icm42607_share_write_blkreg);
