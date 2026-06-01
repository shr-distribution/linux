// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - sysfs / device attributes
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Provides the per-register sysfs surface: the format helpers that turn raw
 * A6 register bytes into human-readable strings, the generic show/store
 * callbacks bound to every entry in a6_register_desc_arr[], the diagnostic
 * extract/cksum nodes (a6_diag, validate_cksum), and the create/remove
 * helpers that wire these into a device.
 */

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "a6_internal.h"
#include "high_level_funcs.h"

int32_t format_current(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t *)val; // (val[0]<<8) + val[1];

	/* in uA */
	/* TODO: Bootie source code states:
	 *  Define R sense as 20 mOhms, as opposed as a variable and reading it
	 *  from the pack, not all packs have the correct R sense programmed
	 */
	conv_val = (conv_val * 3125)/2/(int32_t)state->cached_rsense_val;

	ret = scnprintf(fmt_buffer, size_buffer, "%d\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "Current value (uA): %s\n", fmt_buffer);
	return ret;
}

int32_t format_voltage(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t *)val;  // (val[0]<<8) + val[1];

	/* in uV -- 11-bit signed value, unit = 4880uV */
	conv_val = (conv_val>>5) * 4880;
	ret = scnprintf(fmt_buffer, size_buffer, "%d\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Voltage value (uV): %s\n",
		fmt_buffer);
	return ret;
}

int32_t r_format_voltage(const struct a6_device_state *state, const uint8_t *fmt_buffer,
			 uint8_t *val, uint32_t size_buffer)
{
	int32_t ret;
	unsigned long conv_val;

	ret = kstrtoul(fmt_buffer, 0, &conv_val);
	if (ret)
		return 0;

	/* in uV -- 11-bit signed value, unit = 4880uV */
	conv_val = (conv_val / 4880) << 5;
	// capped at 16 bits...
	if (conv_val >> 0x10)
		return 0;

	*(uint16_t *)val = conv_val;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: ip: %s op: %ld\n",
		   __func__, fmt_buffer, conv_val);

	return strlen(fmt_buffer);
}
/**
 * Take the Accumulated Current Register (ACR), which is expressed in units of 6.25uVh
 * and convert to uAh.
 */
int32_t format_rawcoulomb(const struct a6_device_state *state, const uint8_t *val,
			  uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t *)val; // (val[0]<<8) + val[1];

	// in uAh
	conv_val = (conv_val * 6250) / (int32_t)state->cached_rsense_val;
	ret = scnprintf(fmt_buffer, size_buffer, "%d.%03d\n", conv_val/1000,
		       ((conv_val >= 0) ? conv_val : -conv_val) % 1000);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Raw Coulomb value (uAh): %s\n",
		fmt_buffer);
	return ret;
}

/**
 * Take the Remaining Active Absolute Capacity (RAAC) register, which is in
 * units of 1.6 mAh and convert to mAh
 */
int32_t format_coulomb(const struct a6_device_state *state, const uint8_t *val,
			  uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t *)val;

	// in mAh
	conv_val = (conv_val * 8) / 5;
	ret = scnprintf(fmt_buffer, size_buffer, "%d\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Coulomb value (mAh): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_age(const struct a6_device_state *state, const uint8_t *val,
		   uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret, conv_val;

	// Age conversion factor, in .01 percent. Raw value 0x80 = 100%
	conv_val = (10000*val[0]) >> 7;
	ret = scnprintf(fmt_buffer, size_buffer, "%d.%02d\n",
		conv_val/100, conv_val%100);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Battery life left (%%): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_fullx(const struct a6_device_state *state, const uint8_t *val,
		     uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int32_t conv_val =  *(int16_t *)val; // (val[0]<<8) + val[1];

	// Convert 6.25uVh to uAh
	conv_val = (conv_val * 6250) / state->cached_rsense_val;
	ret = scnprintf(fmt_buffer, size_buffer,
		"%d.%03d\n", conv_val/1000, conv_val%1000);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Battery life left (uAh): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_temp(const struct a6_device_state *state, const uint8_t *val,
		    uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int conv_val = val[1];	/* Ignoring the fraction part */

	// in C
	ret = snprintf(fmt_buffer, size_buffer, "%d\n", (int8_t)conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "Temperature (C): %s\n",
		fmt_buffer);
	return ret;
}

static int32_t __maybe_unused format_command(const struct a6_device_state *state,
		    const uint8_t *val, uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	int conv_command_debug_code = val[0];

	ret = snprintf(fmt_buffer, size_buffer, "%d\n", (int8_t)conv_command_debug_code);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "debug code(C): %s\n",
		fmt_buffer);
	return ret;
}

int32_t format_accessory(const struct a6_device_state *state, const uint8_t *val,
		    uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer, "%02X\n", val[0]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_comm_status(const struct a6_device_state *state, const uint8_t *val,
		    uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer, "%02X\n", val[0]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t r_format_temp(const struct a6_device_state *state, const uint8_t *fmt_buffer,
		      uint8_t *val, uint32_t size_buffer)
{
	int32_t ret;
	long conv_val;

	ret = kstrtol(fmt_buffer, 0, &conv_val);
	if (ret)
		return 0;

	pr_err("conv_val : %ld, hex: %lx\n", conv_val, conv_val);
	// capped at 8 bits...
	if (conv_val < -128)
		return 0;

	/* Ignoring the fraction part */
	*(uint16_t *)val = (uint8_t)conv_val << 8;

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: ip: %s op: %ld\n",
		   __func__, fmt_buffer, conv_val << 8);

	return strlen(fmt_buffer);
}

int32_t format_status(const struct a6_device_state *state, const uint8_t *val,
		      uint8_t *fmt_buffer, uint32_t size_buffer)
{
	uint32_t conv_val = *val;
	int32_t count, i;

	static const char * const status_bits[] = {
		NULL,
		"power-on-reset",
		"undervoltage",
		NULL,
		"learn",
		"standby-empty",
		"active-empty",
		"charge-termination"};

	for (i = count = 0; i < ARRAY_SIZE(status_bits); i++)
		if ((conv_val & (1 << i)) && status_bits[i] != NULL)
			count += scnprintf(fmt_buffer + count, size_buffer - count, "%s\n", status_bits[i]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "Battery status: %s\n", fmt_buffer);

	return count;
}

int32_t format_rsense(const struct a6_device_state *state, const uint8_t *val,
		      uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val = *val;

	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", (!conv_val) ? conv_val : 1000/conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "rsense (ohm): %s\n",
		fmt_buffer);
	return ret;
}

/**
 * Determine what, if any chargers are attached to the system.  Only one
 * result will be returned, even if multiple chargers are detected.
 *
 * Output is a combination of:
 *   0x01 - Puck Detected
 *   0x02 - Puck Connected
 *   0x04 - Wired Power Detected
 *   0x08 - Puck Power Connected
 *   0x10 - Wired Power Connected
 *   0x20 - Battery Present
 *
 * Not all values are currently supported.
 */
int32_t format_charge_source_status(const struct a6_device_state *state, const uint8_t *val,
				    uint8_t *fmt_buffer, uint32_t size_buffer)
{
	struct charge_source_status_map {
		uint32_t cs_mask;
		const char *cs_name;
	} cs_map[] = {
		[0] = {0x01, "Puck Detected"},
		[1] = {0x02, "Puck Connected"},
		[2] = {0x04, "Wired Power Detected"},
		[3] = {0x08, "Puck Power Connected"},
		[4] = {0x10, "Wired Power Connected"},
		[5] = {0x20, "Battery Present"},
	};
	uint32_t conv_val = *val, count = 0;
	int32_t i;

	for (i = 0; i < ARRAY_SIZE(cs_map); i++)
		if (conv_val & cs_map[i].cs_mask)
			count += scnprintf(fmt_buffer + count, (size_buffer - 1) - count, "%s\n", cs_map[i].cs_name);

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "charge source status (formatted): %s\n", fmt_buffer);

#ifdef A6_REPORT_CONNECTED_ONLY
	conv_val = 0;
	/* map charge source status to user-space values */
	/* TODO: differentiate wall charger from USB */
	if (val[0] & TS2_I2C_FLAGS_2_WIRED_CHARGE)
		conv_val |= 1;
	if (val[0] & TS2_I2C_FLAGS_2_PUCK_CHARGE)
		conv_val |= 4;
#endif

	/* output register contents */
	count = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "charge source status (literal): %s\n", fmt_buffer);
	return count;
}

int32_t format_version(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer,
			"A6 Version: HW: %d, FW (M.m.B): %d.%d.%d, ManID: %d, ProdTyp: %d\n",
			val[12], val[15], val[14], val[13],
			*(int16_t *)&(val[0]), *(int16_t *)&(val[2]));
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_v_offset(const struct a6_device_state *state, const uint8_t *val,
			uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val = *val;

	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "v_offset: %s\n", fmt_buffer);
	return ret;
}

int32_t format_raw_unsigned(const struct a6_device_state *state, const uint8_t *val,
			    uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t conv_val = *val;

	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "raw_unsigned: %s\n", fmt_buffer);
	return ret;
}

int32_t format_u16_hex(const struct a6_device_state *state, const uint8_t *val,
		       uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint16_t conv_val = *(uint16_t *)val;

	ret = scnprintf(fmt_buffer, size_buffer, "0x%04hx\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "u16_hex: %s\n", fmt_buffer);
	return ret;
}

int32_t format_max_power_available(const struct a6_device_state *state, const uint8_t *val,
				   uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;
	uint32_t vnode_max_val = *(uint16_t *)val;
	uint32_t inode_max_val = *(uint16_t *)(val+2);
	uint32_t power_val     = *(val+4);
	uint32_t conv_val;

	conv_val = (vnode_max_val * inode_max_val * power_val)/2560;
	ret = scnprintf(fmt_buffer, size_buffer, "%u\n", conv_val);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_accessory_combo(const struct a6_device_state *state, const uint8_t *val,
			       uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer,
			"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7],
			val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_serno_v1(const struct a6_device_state *state, const uint8_t *val,
			       uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer, "%02x%02x%02x%02x%02x%02x\n",
			val[0], val[1], val[2], val[3], val[4], val[5]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

int32_t format_serno_v2(const struct a6_device_state *state, const uint8_t *val,
			uint8_t *fmt_buffer, uint32_t size_buffer)
{
	int32_t ret;

	ret = scnprintf(fmt_buffer, size_buffer, "%02x%02x%02x%02x%02x%02x%02x%02x\n",
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO, "%s", fmt_buffer);
	return ret;
}

ssize_t a6_generic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t ret = 0, i = 0;
	uint8_t vals[id_size];
	struct a6_register_desc *reg_desc;
	struct a6_device_state *state = i2c_get_clientdata(client);

	// critsec for manipulating flags
	ret = mutex_lock_interruptible(&state->dev_mutex);
	if (ret) {
		pr_err("%s: mutex_lock_interruptible interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: are we in bootload phase?
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			// no: so go ahead and allow concurrent i2c ops (these are
			// synchronized separately to allow priority based execution).
			break;
		}

		// in bootload phase: get on a waitq
		mutex_unlock(&state->dev_mutex);
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// bootload bit set? wait to be cleared (at least 5 mins: in jiffies)
		ret = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags), 300*HZ);
		if (!ret) {
			pr_err("%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		ret = mutex_lock_interruptible(&state->dev_mutex);
		if (ret) {
			pr_err("%s: mutex_lock_interruptible interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	// increment busy refcount
	state->busy_count++;
	// done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	reg_desc = container_of(attr, struct a6_register_desc, dev_attr);

	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0)
		goto err0;

	if (reg_desc->format) {
		ret = reg_desc->format(state, vals, buf, PAGE_SIZE);
	} else {
		/*
		 * if output restricted to 2 8-bit values, show as int16_t
		 * (common case of msb+lsb retrieval)
		 */
		if (reg_desc->num_ids == 2)
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", *(int16_t *)vals);
		/*
		 * if output > or < 2 8-bit values, show as
		 * space-delimited list of int8_t values
		 */
		else {
			i = ret = 0;
			while (i < reg_desc->num_ids) {
				ret += scnprintf(buf+ret, PAGE_SIZE-ret, "%d ", (int8_t)vals[i]);
				i++;
			}
			ret += scnprintf(buf+ret, PAGE_SIZE-ret, "\n");
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "Buffer: %s\n", buf);
	}

#ifdef A6_DEBUG
	{
		/* id_size * 7 + 1 for "0x%04x " format per id */
		uint8_t d_ids[id_size * (4 + 2 + 1) + 1];
		/* id_size * 5 + 1 for "0x%02x " format per val */
		uint8_t d_vals[id_size * (2 + 2 + 1) + 1];
		int32_t i = 0, ret_ids = 0, ret_vals = 0;

		while (i < reg_desc->num_ids) {
			ret_ids += sprintf(d_ids+ret_ids, "0x%04x ", reg_desc->id[i]);
			ret_vals += sprintf(d_vals+ret_vals, "0x%02x ", vals[i]);
			i++;
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_INFO,
			"showing reg name: %s, num_ids: %d, ids: %s, vals: %s\n",
			reg_desc->debug_name, reg_desc->num_ids, d_ids, d_vals);
	}
#endif

err0:
	/* reset busy state */
	mutex_lock(&state->dev_mutex);
	/* decrement busy refcount */
	if (state->busy_count)
		state->busy_count--;
	if (!state->busy_count)
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return ret;
}

// constraints:
// * multi-valued stores must have individual values delimited by whitespace
// * multi-valued stores have individual values constrained to reg size (8-bit)
// * single-valued stores can specify 16-bit values to update two regs (msb+lsb)
// * value count must exactly match for multi-values stores
// * value-count for single value stores may be one less than the reg count specified.
ssize_t a6_generic_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t ret = 0, num_ids, val_cnt, i;
	uint32_t val;
	char *endp = NULL, *bufp;
	uint16_t parsed_vals[id_size];
	uint8_t in_vals[id_size];
	struct a6_register_desc *reg_desc;
	struct a6_device_state *state = i2c_get_clientdata(client);

	// critsec for manipulating flags
	ret = mutex_lock_interruptible(&state->dev_mutex);
	if (ret) {
		pr_err("%s: mutex_lock_interruptible interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: are we in bootload phase?
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			// no: so go ahead and allow concurrent i2c ops (these are
			// synchronized separately to allow priority based execution).
			break;
		}

		// in bootload phase: get on a waitq
		mutex_unlock(&state->dev_mutex);
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// bootload bit set? wait to be cleared (at least 5 mins: in jiffies)
		ret = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags), 300*HZ);
		if (!ret) {
			pr_err("%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		ret = mutex_lock_interruptible(&state->dev_mutex);
		if (ret) {
			pr_err("%s: mutex_lock_interruptible interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	// increment busy refcount
	state->busy_count++;
	// done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	reg_desc = container_of(attr, struct a6_register_desc, dev_attr);
	num_ids = reg_desc->num_ids;

	if (reg_desc->r_format) {
		ret = reg_desc->r_format(state, buf, in_vals, count);
	} else {
		bufp = (char *)buf;
		val_cnt = 0;
		do {
			// let strtoul perform base determination (handles '0x' and '0' prefixes) ...
			val = (uint16_t)simple_strtoul(bufp, &endp, 0);
			// each space-delimited write unit is capped at 16 bits...
			if (val > 0xffff) {
				ret = -EINVAL;
				goto err0;
			}

			parsed_vals[val_cnt] = val;

			/* skip whitespace */
			while (*endp && isspace(*endp))
				endp++;
			/* reset for next iteration */
			bufp = endp;
		} while ((++val_cnt < num_ids) && ((endp - buf) < count));

		/*
		 * three levels of value count validation:
		 * - if extra values: fail
		 * - if multiple values and does not match reg count specified: fail
		 * - if single value and reg count > 2: fail
		 */
		if (*endp) {
			ret = -EINVAL;
			goto err0;
		} else if (val_cnt > 1) {
			if (val_cnt != num_ids) {
				ret = -EINVAL;
				goto err0;
			}
		} else if (num_ids > 2) {
			ret = -EINVAL;
			goto err0;
		}

		// if multi-valued store or single-valued store to one reg: we constrain each
		// value to reg size (8 bits)
		if ((val_cnt > 1) || (num_ids == 1)) {
			for (i = 0; i < val_cnt; i++) {
				if (parsed_vals[i] > 0xff) {
					ret = -EINVAL;
					goto err0;
				}

				in_vals[i] = parsed_vals[i];
			}
		}
		// single-valued store can be 8-bit or 16-bit (for msb+lsb)
		else
			((uint16_t *)in_vals)[0] = ((uint16_t *)parsed_vals)[0];
	}

#ifdef A6_DEBUG
	{
		/* id_size * 7 for "0x%04x " format per id */
		uint8_t d_ids[id_size * (4 + 2 + 1)];
		/* id_size * 5 for "0x%02x " format per val */
		uint8_t d_vals[id_size * (2 + 2 + 1)];
		int32_t i = 0, ret_ids = 0, ret_vals = 0;

		while (i < reg_desc->num_ids) {
			ret_ids = sprintf(d_ids+ret_ids, "0x%04x ", reg_desc->id[i]);
			ret_vals = sprintf(d_vals+ret_vals, "0x%02x ", in_vals[i]);
			i++;
		}

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
			"storing reg name: %s, num_ids: %d, ids: %s, vals: %s\n",
			reg_desc->debug_name, reg_desc->num_ids, d_ids, d_vals);
	}
#endif

	ret = a6_i2c_write_reg(client, reg_desc->id, num_ids, in_vals);
	if (ret < 0)
		goto err0;

	ret = count;

err0:
	/* reset busy state */
	mutex_lock(&state->dev_mutex);
	/* decrement busy refcount */
	if (state->busy_count)
		state->busy_count--;
	if (!state->busy_count)
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return ret;
}

enum {
	ACTIVATE_EXTRACT,
	NONE
};

ssize_t a6_val_cksum_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t rc = 0, reloop = 0, failed = 0;
	struct a6_device_state *state = i2c_get_clientdata(client);
	uint16_t cksum1, cksum2, cksum_cycles, cksum_errors;
	struct a6_register_desc *reg_desc;
	uint8_t vals[id_size];



	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted\n", __func__);
		return -EIO;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 5 minutes: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(DEVICE_BUSY_BIT, state->flags), 300*HZ);
		if (!rc) {
			pr_err("%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			pr_err("%s: mutex_lock interrupted(2)\n", __func__);
			goto err0;
		}
	}

	rc = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
	ASSERT(!rc);

	// we're done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	do {
		/* write command (cksum1) */
		reg_desc = &a6_register_desc_arr[35];
		vals[0] = TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_1;
		rc = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			break;
		}

		/* read cksum1 */
		reg_desc = &a6_register_desc_arr[80];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum1 = vals[0] | vals[1] << 8;

		/* read cksum cycle counter */
		reg_desc = &a6_register_desc_arr[81];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum_cycles = vals[0] | vals[1] << 8;

		/* write command (cksum2) */
		reg_desc = &a6_register_desc_arr[35];
		vals[0] = TS2_I2C_COMMAND_FRAM_CHECKSUM_READ_2;
		rc = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			break;
		}

		/* read cksum2 */
		reg_desc = &a6_register_desc_arr[80];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum2 = vals[0] | vals[1] << 8;

		/* read cksum error counter */
		reg_desc = &a6_register_desc_arr[81];
		memset(vals, 0, sizeof(vals));
		rc = a6_i2c_read_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (rc < 0) {
			pr_err("%s: error reading reg: %s, ids: 0x%x 0x%x\n",
			       __func__, reg_desc->debug_name, reg_desc->id[0], reg_desc->id[1]);
			break;
		}
		cksum_errors = vals[0] | vals[1] << 8;

		/* validate cksum */
		if (reloop || !cksum1 || !cksum2 || (cksum1 != cksum2)) {
			pr_err("A6 checksum (%s) validation failure:\n"
					"cksum1: 0x%02x; cksum2: 0x%02x; cksum_cycles: 0x%02x;"
					" cksum_errors: 0x%02x\n",
					(!reloop) ? "first-stage" : "second-stage",
					cksum1, cksum2, cksum_cycles, cksum_errors);
			if (reloop) {
				reloop--;
				failed = 1;
			} else {
				reloop++;
			}
		}
	} while (reloop);

	if (rc < 0) {
		pr_err("%s: checksum retrieval enountered i2c errors: fallback to sbw(jtag) access.\n",
				__func__);
		get_checksum_data_sbw((struct a6_sbw_interface *)state->plat_data->sbw_ops, &cksum1,
				       &cksum2, &cksum_cycles, &cksum_errors);
	}

	/* validate cksum */
	if (failed || !cksum1 || !cksum2 || (cksum1 != cksum2)) {
		pr_err("A6 checksum validation failed:\n"
				"cksum1: 0x%02x; cksum2: 0x%02x; cksum_cycles: 0x%02x;"
				" cksum_errors: 0x%02x\n",
				cksum1, cksum2, cksum_cycles, cksum_errors);
		rc = snprintf(buf, PAGE_SIZE, "%d\n", 0);
	} else {
		rc = snprintf(buf, PAGE_SIZE, "%d\n", 1);
	}

err0:
	mutex_lock(&state->dev_mutex);
	clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
	clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return rc;
}

static char *activation_strlist[] = {
	[ACTIVATE_EXTRACT] = "extract",
	[NONE] = "none"
};

ssize_t a6_diag_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t rc = 0, ret_val, action_i = 0;
	struct a6_device_state *state = i2c_get_clientdata(client);
	bool skip = false;


	action_i = (sizeof(activation_strlist)/sizeof(char *));
	do {
		if (strncmp(buf, activation_strlist[action_i-1],
		    strlen(activation_strlist[action_i-1])) == 0)
			break;
	} while (action_i--);

	if (!action_i)
		return -EINVAL;

	// store the action index
	action_i--;

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted\n", __func__);
		return -EIO;
	}

	// are we busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: get on a waitq
		mutex_unlock(&state->dev_mutex);

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 5 minutes: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(DEVICE_BUSY_BIT, state->flags), 300*HZ);
		if (!rc) {
			pr_err("%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			pr_err("%s: mutex_lock interrupted(2)\n", __func__);
			goto err0;
		}
	}

	if (action_i == ACTIVATE_EXTRACT) {
		if (test_bit(EXTRACT_INITIATED, state->flags))
			skip = true;
		else {
			ret_val = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
			ASSERT(!ret_val);
			ret_val = test_and_set_bit(EXTRACT_INITIATED, state->flags);
			ASSERT(!ret_val);
		}
	} else if (action_i == NONE) {
		if (!test_bit(EXTRACT_INITIATED, state->flags))
			skip = true;
	}

	// we're done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	if (true == skip)
		goto err0;

	if (action_i == ACTIVATE_EXTRACT) {
		// reset force-wake state to always force wake on first i2c txn
		// after pmem extraction
		timer_delete(&state->a6_force_wake_timer);
		mutex_lock(&state->a6_force_wake_mutex);
		if (test_bit(CAP_PERIODIC_WAKE, state->flags))
			clear_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		mutex_unlock(&state->a6_force_wake_mutex);

		// start pmem extract
		pr_err("%s: starting ttf_extract.\n", __func__);
		rc = ttf_extract_fw_sbw((struct a6_sbw_interface *)state->plat_data->sbw_ops);
		if (rc) {
			pr_err("Failed to ttf_extract a6 pmem.\n");
			goto err0;
		} else {
			pr_err("A6: Completed ttf_extract a6 pmem.\n");
			// wait for the A6 to boot...
			msleep(3000);
			// - re-init state
			// - if init fails: ignore
			a6_init_state(state->i2c_dev);
		}
	} else if (action_i == NONE) {
		pr_err("%s: clearing ttf_extract cache.\n", __func__);
		rc = ttf_extract_cache_clear();
	} else {
		// invalid
	}

err0:
	mutex_lock(&state->dev_mutex);
	if (!skip) {
		if (rc || action_i == NONE) {
			if (test_bit(EXTRACT_INITIATED, state->flags))
				clear_bit(EXTRACT_INITIATED, state->flags);
		}
		if (test_bit(BOOTLOAD_ACTIVE_BIT, state->flags))
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
	}
	clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);

	return rc ? rc : count;
}

ssize_t a6_diag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int32_t rc = 0;
	struct a6_device_state *state = i2c_get_clientdata(client);


	mutex_lock(&state->dev_mutex);
	if (test_bit(EXTRACT_INITIATED, state->flags))
		rc = snprintf(buf, PAGE_SIZE, "%s\n", activation_strlist[ACTIVATE_EXTRACT]);
	mutex_unlock(&state->dev_mutex);

	return rc;
}

int32_t a6_create_dev_files(struct a6_device_state *state, struct device *dev)
{
	int32_t rc = 0, reg_cnt = a6_register_desc_arr_size;
	int32_t idx = 0, cust_idx = 0;

	for (idx = 0; idx < reg_cnt; idx++) {
		rc = device_create_file(dev, &a6_register_desc_arr[idx].dev_attr);
		if (rc < 0) {
			pr_err("%s: failed to create dev_attr file for %s.\n",
			       __func__, a6_register_desc_arr[idx].dev_attr.attr.name);
			goto err0;
		}
	}

	reg_cnt = custom_devattr_size;
	for (cust_idx = 0; cust_idx < reg_cnt; cust_idx++) {
		rc = device_create_file(dev, &custom_devattr[cust_idx]);
		if (rc < 0) {
			pr_err("%s: failed to create dev_attr file for %s.\n",
			       __func__, custom_devattr[cust_idx].attr.name);
			goto err0;
		}
	}

	rc = sysfs_create_link(&state->mdev.this_device->kobj, &dev->kobj, "regs");
	if (rc)
		pr_err("%s: error in creating symlink.\n", __func__);

	return 0;

err0:
	/* error: cleanup files already created. */
	for (idx--; idx >= 0; idx--)
		device_remove_file(dev, &a6_register_desc_arr[idx].dev_attr);
	for (cust_idx--; cust_idx >= 0; cust_idx--)
		device_remove_file(dev, &custom_devattr[cust_idx]);
	return rc;
}

void a6_remove_dev_files(struct a6_device_state *state, struct device *dev)
{
	int32_t reg_cnt = a6_register_desc_arr_size;
	int32_t idx;

	for (idx = 0; idx < reg_cnt; idx++)
		device_remove_file(dev, &a6_register_desc_arr[idx].dev_attr);

	reg_cnt = custom_devattr_size;
	for (idx = 0; idx < reg_cnt; idx++)
		device_remove_file(dev, &custom_devattr[idx]);
}
