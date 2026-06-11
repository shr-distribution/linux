/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Palm A6 Battery Controller Driver - private definitions
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 */

#ifndef _A6_INTERNAL_H_
#define _A6_INTERNAL_H_

#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>

/*
 * MSP430 A6 register map. The A6 firmware lays its registers out across
 * several 256-byte "pages" addressed by a 16-bit register ID. Only the
 * subset consumed by the read-only battery driver is kept here; the
 * rest of the register map is documented in the legacy vendor sources.
 *
 * 16-bit values comprising an LSB/MSB pair must be addressed LSB-first
 * in the id[] array passed to a6_i2c_read_reg(); the dispatcher swaps
 * to MSB-leading on the wire per the A6 firmware constraint.
 */

/* page 0x00 - host interrupts */
#define TS2_I2C_INT_MASK_2				0x0002
#define TS2_I2C_INT_MASK_3				0x0003
#define TS2_I2C_INT_STATUS_1				0x0005
#define TS2_I2C_INT_STATUS_2				0x0006
#define TS2_I2C_INT_STATUS_3				0x0007
#define TS2_I2C_INT_2_BAT_TEMP_HIGH			0x20
#define TS2_I2C_INT_2_BAT_TEMP_LOW			0x10
#define TS2_I2C_INT_2_BAT_VOLT_LOW			0x08
#define TS2_I2C_INT_2_BAT_RARC_CRIT			0x04
#define TS2_I2C_INT_2_BAT_RARC_LOW2			0x02
#define TS2_I2C_INT_2_BAT_RARC_LOW1			0x01
#define TS2_I2C_INT_3_A2A_CONNECT_CHANGE		0x08
#define TS2_I2C_INT_3_FLAGS_CHANGE			0x04
#define TS2_I2C_INT_3_LOG				0x02
#define TS2_I2C_INT_3_RESET				0x01

/* page 0x01 - battery */
#define TS2_I2C_BAT_STATUS				0x0100
#define TS2_I2C_BAT_RARC				0x0101
#define TS2_I2C_BAT_AVG_CUR_MSB				0x0103
#define TS2_I2C_BAT_AVG_CUR_LSB				0x0104
#define TS2_I2C_BAT_TEMP_MSB				0x0105
#define TS2_I2C_BAT_TEMP_LSB				0x0106
#define TS2_I2C_BAT_VOLT_MSB				0x0107
#define TS2_I2C_BAT_VOLT_LSB				0x0108
#define TS2_I2C_BAT_CUR_MSB				0x0109
#define TS2_I2C_BAT_CUR_LSB				0x010a
#define TS2_I2C_BAT_COULOMB_MSB				0x010b
#define TS2_I2C_BAT_COULOMB_LSB				0x010c
#define TS2_I2C_BAT_FULL_MSB				0x010e
#define TS2_I2C_BAT_FULL_LSB				0x010f
#define TS2_I2C_BAT_RSNSP				0x0112

/* page 0x07 - puck (charger / source) flags */
#define TS2_I2C_FLAGS_0					0x0701
#define TS2_I2C_FLAGS_0_PUCK_PRIORITY			BIT(0)
#define TS2_I2C_FLAGS_2					0x0703
#define TS2_I2C_FLAGS_2_PUCK				0x02
#define TS2_I2C_FLAGS_2_PUCK_DETECT			0x01

/* page 0x04 - local enumeration / version */
#define TS2_I2C_ENUM_MFGR_ID_HI				0x0403
#define TS2_I2C_ENUM_MFGR_ID_LO				0x0404
#define TS2_I2C_ENUM_PRODUCT_TYPE_HI			0x0405
#define TS2_I2C_ENUM_PRODUCT_TYPE_LO			0x0406
#define TS2_I2C_ENUM_SERNO_0				0x040E
#define TS2_I2C_ENUM_SERNO_1				0x040D
#define TS2_I2C_ENUM_SERNO_2				0x040C
#define TS2_I2C_ENUM_SERNO_3				0x040B
#define TS2_I2C_ENUM_SERNO_4				0x040A
#define TS2_I2C_ENUM_SERNO_5				0x0409
#define TS2_I2C_ENUM_SERNO_6				0x0408
#define TS2_I2C_ENUM_SERNO_7				0x0407
#define TS2_I2C_ENUM_ASSY_REV				0x040F
#define TS2_I2C_ENUM_FW_VER_2				0x0410
#define TS2_I2C_ENUM_FW_VER_1				0x0411
#define TS2_I2C_ENUM_FW_VER_0				0x0412

/* Default rsense (mOhms) when the device reports 0 / read fails */
#define RSENSE_DEFAULT					20

/* state flags */
enum {
	IS_INITIALIZED_BIT = 0,
	A2A_CONNECTED,
	IS_SUSPENDED,
	INT_PENDING,
	SIZE_FLAGS
};

struct a6_device_state {
	struct i2c_client *i2c_dev;

	/* Serializes A6 register reads/writes and state-flag updates */
	struct mutex dev_mutex;

	struct work_struct a6_irq_work;
	struct workqueue_struct *ka6d_workqueue;

	u16 cached_rsense_val;
	DECLARE_BITMAP(flags, SIZE_FLAGS);

	/* Host-driven wake-from-sleep line */
	struct gpio_desc *wakeup_gpio;

	struct power_supply *battery;
	struct power_supply_desc battery_desc;
	int device_index;
};

/* i2c register access helpers (defined in a6_dispatch.c) */
int a6_i2c_read_reg(struct i2c_client *client, const u16 *ids,
		    u32 num_ids, u8 *out);
int a6_i2c_write_reg(struct i2c_client *client, const u16 *ids,
		     u32 num_ids, const u8 *in);

/* device state init / power supply (defined in a6.c, a6_power_supply.c) */
int a6_init_state(struct i2c_client *client);
int a6_register_power_supply(struct a6_device_state *state);

/* IRQ + bottom-half handlers (defined in a6.c) */
irqreturn_t a6_irq(int irq, void *dev_id);
void a6_irq_work_handler(struct work_struct *work);

#endif /* _A6_INTERNAL_H_ */
