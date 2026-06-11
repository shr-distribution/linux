/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Palm A6 auxiliary-bus glue: shared structure linking the base
 * a6-battery driver (which owns the i2c_client + dev_mutex) to the
 * downstream-only a6_a2a_comm module (which re-exposes the legacy
 * /dev/a6_N + sysfs ABI consumed by webOS tap2shared).
 *
 * Downstream only. Not for upstream.
 *
 * Copyright (C) 2026 Herman van Hazendonk <github.com@herrie.org>
 */

#ifndef _PALM_A6_AUX_H_
#define _PALM_A6_AUX_H_

#include <linux/auxiliary_bus.h>
#include <linux/types.h>

struct i2c_client;

/**
 * struct a6_aux_dev - parent object placed on the auxiliary bus by the
 *		       base a6-battery driver and consumed by a6_a2a_comm.
 * @adev:	the auxiliary_device handed to the bus core.
 * @client:	the I2C client owned by the base driver. The aux driver
 *		may pass it back through the @read_regs / @write_regs
 *		callbacks but must not touch the underlying i2c_adapter
 *		directly.
 * @device_index:	0 or 1 - identifies which A6 instance this is
 *		(I2C addresses 0x31 and 0x32 respectively). Used by the
 *		aux driver to name the miscdevice "a6_N".
 * @read_regs:	mutex-protected wrapper around the base driver's
 *		a6_i2c_read_reg(). The callback takes &state->dev_mutex
 *		internally so the aux driver does not need direct access
 *		to base-driver private state.
 * @write_regs:	companion to @read_regs for register writes.
 *
 * Both callbacks must not be called from atomic context: they may
 * block on the dev_mutex and on the i2c_transfer().
 */
struct a6_aux_dev {
	struct auxiliary_device adev;
	struct i2c_client *client;
	int device_index;
	int (*read_regs)(struct i2c_client *client, const u16 *ids,
			 u32 num_ids, u8 *out);
	int (*write_regs)(struct i2c_client *client, const u16 *ids,
			  u32 num_ids, const u8 *in);
};

static inline struct a6_aux_dev *to_a6_aux_dev(struct auxiliary_device *adev)
{
	return container_of(adev, struct a6_aux_dev, adev);
}

#endif /* _PALM_A6_AUX_H_ */
