// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - I2C register helpers
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Synchronous read/write helpers that drive the MSP430 A6 firmware over
 * I2C. Each helper builds the chained i2c_msg array required by the
 * A6 register protocol (MSB-leading 16-bit address per access) and
 * forwards it through i2c_transfer().
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "a6_internal.h"

/*
 * The A6 firmware accepts up to a small handful of chained register
 * accesses per i2c_transfer() invocation. The vendor driver capped this
 * at 20 to keep the on-stack msg array bounded; keep that limit.
 */
#define A6_MAX_IDS	20

int a6_i2c_read_reg(struct i2c_client *client, const u16 *ids,
		    u32 num_ids, u8 *out)
{
	struct i2c_msg msg[A6_MAX_IDS * 2], *msg_itr;
	u8 addr_buf[A6_MAX_IDS][2];
	int ret, i;

	if (!num_ids || num_ids > A6_MAX_IDS)
		return -EINVAL;

	msg_itr = &msg[0];
	for (i = num_ids - 1; i >= 0; i--) {
		/* address phase: MSB-leading 16-bit register id (host-endian-safe) */
		put_unaligned_be16(ids[i], addr_buf[i]);
		msg_itr->addr = client->addr;
		msg_itr->flags = 0;
		msg_itr->len = sizeof(u16);
		msg_itr->buf = addr_buf[i];

		/* data phase: 1 byte read */
		(msg_itr + 1)->addr = client->addr;
		(msg_itr + 1)->flags = I2C_M_RD;
		(msg_itr + 1)->len = sizeof(u8);
		(msg_itr + 1)->buf = &out[i];

#ifdef CONFIG_BATTERY_PALM_A6_I2C_SINGLE_BYTE
		/*
		 * Single-byte mode: issue one repeated-START / 1-byte read
		 * per register on hosts that cannot handle a chained N-byte
		 * read across multiple registers in a single transaction.
		 */
		ret = i2c_transfer(client->adapter, msg_itr, 2);
		if (ret < 0) {
			dev_err_ratelimited(&client->dev,
					    "%s: i2c err %d on id 0x%x\n",
					    __func__, ret, ids[i]);
			return ret;
		}
		if (ret != 2) {
			dev_err_ratelimited(&client->dev,
					    "%s: short i2c transfer %d/2 on id 0x%x\n",
					    __func__, ret, ids[i]);
			return -EIO;
		}
		/* MSP430 firmware needs a short recovery gap between txns */
		usleep_range(700, 900);
#endif
		msg_itr += 2;
	}

#ifndef CONFIG_BATTERY_PALM_A6_I2C_SINGLE_BYTE
	ret = i2c_transfer(client->adapter, msg, num_ids * 2);
	if (ret < 0) {
		dev_err_ratelimited(&client->dev, "%s: i2c err %d\n",
				    __func__, ret);
		return ret;
	}
	if (ret != num_ids * 2) {
		dev_err_ratelimited(&client->dev,
				    "%s: short i2c transfer %d/%u\n",
				    __func__, ret, num_ids * 2);
		return -EIO;
	}
#endif

	return 0;
}

int a6_i2c_write_reg(struct i2c_client *client, const u16 *ids,
		     u32 num_ids, const u8 *in)
{
	struct i2c_msg msg[A6_MAX_IDS], *msg_itr;
	u8 i2c_buf[3 * A6_MAX_IDS];
	int ret, i;

	if (!num_ids || num_ids > A6_MAX_IDS)
		return -EINVAL;

	msg_itr = &msg[0];
	for (i = num_ids - 1; i >= 0; i--) {
		i2c_buf[i * 3 + 0] = (u8)(ids[i] >> 8);
		i2c_buf[i * 3 + 1] = (u8)(ids[i]);
		i2c_buf[i * 3 + 2] = in[i];

		msg_itr->addr = client->addr;
		msg_itr->flags = 0;
		msg_itr->len = 3;
		msg_itr->buf = &i2c_buf[i * 3];
		msg_itr++;
	}

	ret = i2c_transfer(client->adapter, msg, num_ids);
	if (ret < 0) {
		dev_err_ratelimited(&client->dev, "%s: i2c err %d\n",
				    __func__, ret);
		return ret;
	}
	if (ret != num_ids) {
		dev_err_ratelimited(&client->dev,
				    "%s: short i2c transfer %d/%u\n",
				    __func__, ret, num_ids);
		return -EIO;
	}

	return 0;
}
