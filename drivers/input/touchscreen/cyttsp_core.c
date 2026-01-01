// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009, 2010, 2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2012 Javier Martinez Canillas <javier@dowhile0.org>
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 */

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include "cyttsp_core.h"

/* Palm: Power state string for debugging */
static const char *cyttsp_power_state_string(u8 power_state)
{
	switch (power_state) {
	case CY_PWR_IDLE_STATE:
		return "IDLE";
	case CY_PWR_ACTIVE_STATE:
		return "ACTIVE";
	case CY_PWR_LOW_STATE:
		return "LOW_POWER";
	case CY_PWR_SLEEP_STATE:
		return "SLEEP";
	default:
		return "UNKNOWN";
	}
}

/* Bootloader number of command keys */
#define CY_NUM_BL_KEYS		8

/* helpers */
#define GET_NUM_TOUCHES(x)		((x) & 0x0F)
#define IS_LARGE_AREA(x)		(((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)			((x) & 0x20)
#define IS_VALID_APP(x)			((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)		((x) & 0x3F)
#define GET_HSTMODE(reg)		(((reg) & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)		(((reg) & 0x10) >> 4)

#define CY_REG_BASE			0x00
#define CY_REG_ACT_DIST			0x1E
#define CY_REG_ACT_INTRVL		0x1D
#define CY_REG_TCH_TMOUT		(CY_REG_ACT_INTRVL + 1)
#define CY_REG_LP_INTRVL		(CY_REG_TCH_TMOUT + 1)
#define CY_MAXZ				255
#define CY_DELAY_DFLT			20 /* ms */
#define CY_DELAY_MAX			500
/* Active distance in pixels for a gesture to be reported */
#define CY_ACT_DIST_DFLT		0xF8 /* pixels */
#define CY_ACT_DIST_MASK		0x0F
/* Active Power state scanning/processing refresh interval */
#define CY_ACT_INTRVL_DFLT		0x00 /* ms */
/* Low Power state scanning/processing refresh interval */
#define CY_LP_INTRVL_DFLT		0x0A /* ms */
/* touch timeout for the Active power */
#define CY_TCH_TMOUT_DFLT		0xFF /* ms */
#define CY_HNDSHK_BIT			0x80
/* device mode bits */
#define CY_OPERATE_MODE			0x00
#define CY_SYSINFO_MODE			0x10
/* power mode select bits */
#define CY_SOFT_RESET_MODE		0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE		0x02
#define CY_LOW_POWER_MODE		0x04

/* Slots management */
#define CY_MAX_FINGER			4
#define CY_MAX_ID			16

static const u8 bl_command[] = {
	0x00,			/* file offset */
	0xFF,			/* command */
	0xA5,			/* exit bootloader command */
	0, 1, 2, 3, 4, 5, 6, 7	/* default keys */
};

static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
				u8 length, void *buf)
{
	int error;
	int tries;

	for (tries = 0; tries < CY_NUM_RETRY; tries++) {
		error = ts->bus_ops->read(ts->dev, ts->xfer_buf, command,
				length, buf);
		if (!error)
			return 0;

		msleep(CY_DELAY_DFLT);
	}

	return -EIO;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
				 u8 length, void *buf)
{
	int error;
	int tries;

	for (tries = 0; tries < CY_NUM_RETRY; tries++) {
		error = ts->bus_ops->write(ts->dev, ts->xfer_buf, command,
				length, buf);
		if (!error)
			return 0;

		msleep(CY_DELAY_DFLT);
	}

	return -EIO;
}

static int ttsp_send_command(struct cyttsp *ts, u8 cmd)
{
	return ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
}

static int cyttsp_handshake(struct cyttsp *ts)
{
	if (ts->use_hndshk)
		return ttsp_send_command(ts,
				ts->xy_data.hst_mode ^ CY_HNDSHK_BIT);

	return 0;
}

static int cyttsp_load_bl_regs(struct cyttsp *ts)
{
	memset(&ts->bl_data, 0, sizeof(ts->bl_data));
	ts->bl_data.bl_status = 0x10;

	return ttsp_read_block_data(ts, CY_REG_BASE,
				    sizeof(ts->bl_data), &ts->bl_data);
}

static int cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int error;
	u8 bl_cmd[sizeof(bl_command)];

	memcpy(bl_cmd, bl_command, sizeof(bl_command));
	if (ts->bl_keys)
		memcpy(&bl_cmd[sizeof(bl_command) - CY_NUM_BL_KEYS],
			ts->bl_keys, CY_NUM_BL_KEYS);

	error = ttsp_write_block_data(ts, CY_REG_BASE,
				      sizeof(bl_cmd), bl_cmd);
	if (error)
		return error;

	/* wait for TTSP Device to complete the operation */
	msleep(CY_DELAY_DFLT);

	error = cyttsp_load_bl_regs(ts);
	if (error)
		return error;

	if (GET_BOOTLOADERMODE(ts->bl_data.bl_status))
		return -EIO;

	return 0;
}

static int cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int error;

	error = ttsp_send_command(ts, CY_OPERATE_MODE);
	if (error)
		return error;

	/* wait for TTSP Device to complete switch to Operational mode */
	error = ttsp_read_block_data(ts, CY_REG_BASE,
				     sizeof(ts->xy_data), &ts->xy_data);
	if (error)
		return error;

	error = cyttsp_handshake(ts);
	if (error)
		return error;

	return ts->xy_data.act_dist == CY_ACT_DIST_DFLT ? -EIO : 0;
}

static int cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int error;

	memset(&ts->sysinfo_data, 0, sizeof(ts->sysinfo_data));

	/* switch to sysinfo mode */
	error = ttsp_send_command(ts, CY_SYSINFO_MODE);
	if (error)
		return error;

	/* read sysinfo registers */
	msleep(CY_DELAY_DFLT);
	error = ttsp_read_block_data(ts, CY_REG_BASE, sizeof(ts->sysinfo_data),
				      &ts->sysinfo_data);
	if (error)
		return error;

	error = cyttsp_handshake(ts);
	if (error)
		return error;

	if (!ts->sysinfo_data.tts_verh && !ts->sysinfo_data.tts_verl)
		return -EIO;

	return 0;
}

static int cyttsp_set_sysinfo_regs(struct cyttsp *ts)
{
	int retval = 0;

	if (ts->act_intrvl != CY_ACT_INTRVL_DFLT ||
	    ts->tch_tmout != CY_TCH_TMOUT_DFLT ||
	    ts->lp_intrvl != CY_LP_INTRVL_DFLT) {

		u8 intrvl_ray[] = {
			ts->act_intrvl,
			ts->tch_tmout,
			ts->lp_intrvl
		};

		/* set intrvl registers */
		retval = ttsp_write_block_data(ts, CY_REG_ACT_INTRVL,
					sizeof(intrvl_ray), intrvl_ray);
		msleep(CY_DELAY_DFLT);
	}

	return retval;
}

static void cyttsp_hard_reset(struct cyttsp *ts)
{
	if (ts->reset_gpio) {
		/*
		 * According to the CY8CTMA340 datasheet page 21, the external
		 * reset pulse width should be >= 1 ms. The datasheet does not
		 * specify how long we have to wait after reset but a vendor
		 * tree specifies 5 ms here.
		 */
		gpiod_set_value_cansleep(ts->reset_gpio, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(ts->reset_gpio, 0);
		usleep_range(5000, 6000);
	}
}

static int cyttsp_soft_reset(struct cyttsp *ts)
{
	int retval;

	/* wait for interrupt to set ready completion */
	reinit_completion(&ts->bl_ready);
	ts->state = CY_BL_STATE;

	enable_irq(ts->irq);

	retval = ttsp_send_command(ts, CY_SOFT_RESET_MODE);
	if (retval) {
		dev_err(ts->dev, "failed to send soft reset\n");
		goto out;
	}

	if (!wait_for_completion_timeout(&ts->bl_ready,
			msecs_to_jiffies(CY_DELAY_DFLT * CY_DELAY_MAX))) {
		dev_err(ts->dev, "timeout waiting for soft reset\n");
		retval = -EIO;
	}

out:
	ts->state = CY_IDLE_STATE;
	disable_irq(ts->irq);
	return retval;
}

static int cyttsp_act_dist_setup(struct cyttsp *ts)
{
	u8 act_dist_setup = ts->act_dist;

	/* Init gesture; active distance setup */
	return ttsp_write_block_data(ts, CY_REG_ACT_DIST,
				sizeof(act_dist_setup), &act_dist_setup);
}

static void cyttsp_extract_track_ids(struct cyttsp_xydata *xy_data, int *ids)
{
	ids[0] = xy_data->touch12_id >> 4;
	ids[1] = xy_data->touch12_id & 0xF;
	ids[2] = xy_data->touch34_id >> 4;
	ids[3] = xy_data->touch34_id & 0xF;
}

static const struct cyttsp_tch *cyttsp_get_tch(struct cyttsp_xydata *xy_data,
					       int idx)
{
	switch (idx) {
	case 0:
		return &xy_data->tch1;
	case 1:
		return &xy_data->tch2;
	case 2:
		return &xy_data->tch3;
	case 3:
		return &xy_data->tch4;
	default:
		return NULL;
	}
}

/*
 * Palm: Validate IRQ counter between driver and device.
 * The touchscreen controller maintains an internal interrupt counter
 * that we can use to detect lost interrupts or firmware issues.
 */
static void cyttsp_validate_irq_cnt(struct cyttsp *ts)
{
	u8 dev_cnt;

	if (!ts->use_irq_cnt)
		return;

	/* The IRQ counter is in the tt_undef area of the xy_data */
	dev_cnt = ts->xy_data.tt_undef[CY_IRQ_CNT_REG];

	ts->irq_cnt_total++;
	ts->irq_cnt++;

	if (ts->irq_cnt != dev_cnt) {
		ts->irq_err_cnt++;
		dev_dbg(ts->dev,
			"IRQ count mismatch: drv=%u dev=%u hst_mode=0x%02X "
			"total=%u errors=%u\n",
			ts->irq_cnt, dev_cnt, ts->xy_data.hst_mode,
			ts->irq_cnt_total, ts->irq_err_cnt);

		/* Sync with device counter */
		ts->irq_cnt = dev_cnt;
	}
}

/*
 * Palm: Store current touch positions for next frame comparison.
 * This enables gesture detection based on position deltas.
 */
static void cyttsp_store_touch_positions(struct cyttsp *ts,
					 int num_tch, int *ids)
{
	struct cyttsp_xydata *xy_data = &ts->xy_data;
	const struct cyttsp_tch *tch;
	int i;

	if (!ts->enhanced_tracking)
		return;

	/* Clear active tracking */
	for (i = 0; i < CY_NUM_TRK_ID; i++)
		ts->act_trk[i] = 0;

	for (i = 0; i < num_tch && i < CY_NUM_MT_TCH_ID; i++) {
		tch = cyttsp_get_tch(xy_data, i);
		if (tch && ids[i] < CY_NUM_TRK_ID) {
			ts->prv_mt_pos[ids[i]][0] = be16_to_cpu(tch->x);
			ts->prv_mt_pos[ids[i]][1] = be16_to_cpu(tch->y);
			ts->prv_mt_tch[i] = ids[i];
			ts->act_trk[ids[i]] = 1;
		}
	}

	ts->num_prv_tch = num_tch;
}

static void cyttsp_report_tchdata(struct cyttsp *ts)
{
	struct cyttsp_xydata *xy_data = &ts->xy_data;
	struct input_dev *input = ts->input;
	int num_tch = GET_NUM_TOUCHES(xy_data->tt_stat);
	const struct cyttsp_tch *tch;
	int ids[CY_MAX_ID];
	int i;
	DECLARE_BITMAP(used, CY_MAX_ID);

	if (IS_LARGE_AREA(xy_data->tt_stat) == 1) {
		/* terminate all active tracks */
		num_tch = 0;
		dev_dbg(ts->dev, "%s: Large area detected\n", __func__);
	} else if (num_tch > CY_MAX_FINGER) {
		/* terminate all active tracks */
		num_tch = 0;
		dev_dbg(ts->dev, "%s: Num touch error detected\n", __func__);
	} else if (IS_BAD_PKT(xy_data->tt_mode)) {
		/* terminate all active tracks */
		num_tch = 0;
		dev_dbg(ts->dev, "%s: Invalid buffer detected\n", __func__);
	}

	/*
	 * Palm: Check for spurious reset - if we had touches but now
	 * the device is in bootloader mode, something went wrong.
	 */
	if (ts->enhanced_tracking && ts->num_prv_tch > 0 && num_tch == 0) {
		if (GET_BOOTLOADERMODE(xy_data->tt_mode)) {
			ts->spurious_reset_cnt++;
			dev_warn(ts->dev,
				 "Spurious reset detected (count=%u), had %u touches\n",
				 ts->spurious_reset_cnt, ts->num_prv_tch);
		}
	}

	cyttsp_extract_track_ids(xy_data, ids);

	bitmap_zero(used, CY_MAX_ID);

	for (i = 0; i < num_tch; i++) {
		tch = cyttsp_get_tch(xy_data, i);

		input_mt_slot(input, ids[i]);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_abs(input, ABS_MT_POSITION_X, be16_to_cpu(tch->x));
		input_report_abs(input, ABS_MT_POSITION_Y, be16_to_cpu(tch->y));
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, tch->z);

		__set_bit(ids[i], used);
	}

	for (i = 0; i < CY_MAX_ID; i++) {
		if (test_bit(i, used))
			continue;

		input_mt_slot(input, i);
		input_mt_report_slot_inactive(input);
	}

	/* Palm: Store positions for next frame comparison */
	cyttsp_store_touch_positions(ts, num_tch, ids);

	input_sync(input);
}

static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = handle;
	int error;

	if (unlikely(ts->state == CY_BL_STATE)) {
		complete(&ts->bl_ready);
		goto out;
	}

	/* Get touch data from CYTTSP device */
	error = ttsp_read_block_data(ts, CY_REG_BASE,
				 sizeof(struct cyttsp_xydata), &ts->xy_data);
	if (error)
		goto out;

	/* provide flow control handshake */
	error = cyttsp_handshake(ts);
	if (error)
		goto out;

	/* Palm: Validate IRQ counter */
	cyttsp_validate_irq_cnt(ts);

	if (unlikely(ts->state == CY_IDLE_STATE))
		goto out;

	if (GET_BOOTLOADERMODE(ts->xy_data.tt_mode)) {
		/*
		 * TTSP device has reset back to bootloader mode.
		 * Restore to operational mode.
		 */
		error = cyttsp_exit_bl_mode(ts);
		if (error) {
			dev_err(ts->dev,
				"Could not return to operational mode, err: %d\n",
				error);
			ts->state = CY_IDLE_STATE;
		}
	} else {
		cyttsp_report_tchdata(ts);
	}

out:
	return IRQ_HANDLED;
}

static int cyttsp_power_on(struct cyttsp *ts)
{
	int error;

	error = cyttsp_soft_reset(ts);
	if (error)
		return error;

	error = cyttsp_load_bl_regs(ts);
	if (error)
		return error;

	if (GET_BOOTLOADERMODE(ts->bl_data.bl_status) &&
	    IS_VALID_APP(ts->bl_data.bl_status)) {
		error = cyttsp_exit_bl_mode(ts);
		if (error) {
			dev_err(ts->dev, "failed to exit bootloader mode\n");
			return error;
		}
	}

	if (GET_HSTMODE(ts->bl_data.bl_file) != CY_OPERATE_MODE ||
	    IS_OPERATIONAL_ERR(ts->bl_data.bl_status)) {
		return -ENODEV;
	}

	error = cyttsp_set_sysinfo_mode(ts);
	if (error)
		return error;

	error = cyttsp_set_sysinfo_regs(ts);
	if (error)
		return error;

	error = cyttsp_set_operational_mode(ts);
	if (error)
		return error;

	/* init active distance */
	error = cyttsp_act_dist_setup(ts);
	if (error)
		return error;

	/* Palm: Set both state machine state and power state */
	ts->state = CY_ACTIVE_STATE;
	ts->power_state = CY_PWR_ACTIVE_STATE;

	return 0;
}

static int cyttsp_enable(struct cyttsp *ts)
{
	int error;

	/*
	 * The device firmware can wake on an I2C or SPI memory slave
	 * address match. So just reading a register is sufficient to
	 * wake up the device. The first read attempt will fail but it
	 * will wake it up making the second read attempt successful.
	 */
	error = ttsp_read_block_data(ts, CY_REG_BASE,
				     sizeof(ts->xy_data), &ts->xy_data);
	if (error)
		return error;

	if (GET_HSTMODE(ts->xy_data.hst_mode))
		return -EIO;

	enable_irq(ts->irq);

	return 0;
}

static int cyttsp_disable(struct cyttsp *ts)
{
	int error;
	u8 sleep_mode;

	/* Palm: Use configured sleep mode instead of always low power */
	if (ts->use_sleep & CY_USE_DEEP_SLEEP_SEL)
		sleep_mode = CY_DEEP_SLEEP_MODE;
	else
		sleep_mode = CY_LOW_POWER_MODE;

	error = ttsp_send_command(ts, sleep_mode);
	if (error)
		return error;

	/* Palm: Track power state */
	if (sleep_mode == CY_DEEP_SLEEP_MODE)
		ts->power_state = CY_PWR_SLEEP_STATE;
	else
		ts->power_state = CY_PWR_LOW_STATE;

	disable_irq(ts->irq);

	return 0;
}

static int cyttsp_suspend(struct device *dev)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;

	dev_dbg(dev, "Enter Sleep (current state: %s)\n",
		cyttsp_power_state_string(ts->power_state));

	mutex_lock(&ts->input->mutex);

	if (ts->suspended) {
		dev_dbg(dev, "Already in sleep state\n");
		mutex_unlock(&ts->input->mutex);
		return 0;
	}

	if (input_device_enabled(ts->input) &&
	    (ts->use_sleep & CY_USE_SLEEP) &&
	    (ts->power_state == CY_PWR_ACTIVE_STATE)) {
		retval = cyttsp_disable(ts);
	}

	if (retval == 0)
		ts->suspended = true;

	dev_dbg(dev, "Sleep Power state is %s\n",
		cyttsp_power_state_string(ts->power_state));

	mutex_unlock(&ts->input->mutex);

	return retval;
}

static int cyttsp_resume(struct device *dev)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int retval = 0;

	dev_dbg(dev, "Wake Up (current state: %s)\n",
		cyttsp_power_state_string(ts->power_state));

	mutex_lock(&ts->input->mutex);

	if (ts->suspended && input_device_enabled(ts->input) &&
	    (ts->power_state != CY_PWR_ACTIVE_STATE)) {
		retval = cyttsp_enable(ts);
		if (retval == 0)
			ts->power_state = CY_PWR_ACTIVE_STATE;
	}

	ts->suspended = false;

	dev_dbg(dev, "Wake Up %s (state: %s)\n",
		retval < 0 ? "FAIL" : "PASS",
		cyttsp_power_state_string(ts->power_state));

	mutex_unlock(&ts->input->mutex);

	return retval;
}

EXPORT_GPL_SIMPLE_DEV_PM_OPS(cyttsp_pm_ops, cyttsp_suspend, cyttsp_resume);

static int cyttsp_open(struct input_dev *dev)
{
	struct cyttsp *ts = input_get_drvdata(dev);
	int retval = 0;

	if (!ts->suspended)
		retval = cyttsp_enable(ts);

	return retval;
}

static void cyttsp_close(struct input_dev *dev)
{
	struct cyttsp *ts = input_get_drvdata(dev);

	if (!ts->suspended)
		cyttsp_disable(ts);
}

static int cyttsp_parse_properties(struct cyttsp *ts)
{
	struct device *dev = ts->dev;
	u32 dt_value;
	int ret;

	ts->bl_keys = devm_kzalloc(dev, CY_NUM_BL_KEYS, GFP_KERNEL);
	if (!ts->bl_keys)
		return -ENOMEM;

	/* Set some default values */
	ts->use_hndshk = false;
	ts->act_dist = CY_ACT_DIST_DFLT;
	ts->act_intrvl = CY_ACT_INTRVL_DFLT;
	ts->tch_tmout = CY_TCH_TMOUT_DFLT;
	ts->lp_intrvl = CY_LP_INTRVL_DFLT;

	/* Palm: Initialize power state and tracking defaults */
	ts->use_sleep = CY_USE_SLEEP | CY_USE_DEEP_SLEEP_SEL;
	ts->power_state = CY_PWR_IDLE_STATE;
	ts->irq_cnt = 0;
	ts->irq_cnt_total = 0;
	ts->irq_err_cnt = 0;
	ts->use_irq_cnt = false;
	ts->num_prv_tch = 0;
	ts->spurious_reset_cnt = 0;
	ts->enhanced_tracking = false;
	memset(ts->act_trk, 0, sizeof(ts->act_trk));
	memset(ts->prv_mt_pos, 0, sizeof(ts->prv_mt_pos));
	memset(ts->prv_mt_tch, 0, sizeof(ts->prv_mt_tch));

	ret = device_property_read_u8_array(dev, "bootloader-key",
					    ts->bl_keys, CY_NUM_BL_KEYS);
	if (ret) {
		dev_err(dev,
			"bootloader-key property could not be retrieved\n");
		return ret;
	}

	ts->use_hndshk = device_property_present(dev, "use-handshake");

	if (!device_property_read_u32(dev, "active-distance", &dt_value)) {
		if (dt_value > 15) {
			dev_err(dev, "active-distance (%u) must be [0-15]\n",
				dt_value);
			return -EINVAL;
		}
		ts->act_dist &= ~CY_ACT_DIST_MASK;
		ts->act_dist |= dt_value;
	}

	if (!device_property_read_u32(dev, "active-interval-ms", &dt_value)) {
		if (dt_value > 255) {
			dev_err(dev, "active-interval-ms (%u) must be [0-255]\n",
				dt_value);
			return -EINVAL;
		}
		ts->act_intrvl = dt_value;
	}

	if (!device_property_read_u32(dev, "lowpower-interval-ms", &dt_value)) {
		if (dt_value > 2550) {
			dev_err(dev, "lowpower-interval-ms (%u) must be [0-2550]\n",
				dt_value);
			return -EINVAL;
		}
		/* Register value is expressed in 0.01s / bit */
		ts->lp_intrvl = dt_value / 10;
	}

	if (!device_property_read_u32(dev, "touch-timeout-ms", &dt_value)) {
		if (dt_value > 2550) {
			dev_err(dev, "touch-timeout-ms (%u) must be [0-2550]\n",
				dt_value);
			return -EINVAL;
		}
		/* Register value is expressed in 0.01s / bit */
		ts->tch_tmout = dt_value / 10;
	}

	/* Palm: Parse sleep mode configuration */
	if (device_property_present(dev, "use-deep-sleep"))
		ts->use_sleep |= CY_USE_DEEP_SLEEP_SEL;

	if (device_property_present(dev, "disable-sleep"))
		ts->use_sleep &= ~CY_USE_SLEEP;

	/* Palm: Enable IRQ counter validation if requested */
	ts->use_irq_cnt = device_property_present(dev, "use-irq-counter");

	/* Palm: Enable enhanced touch tracking if requested */
	ts->enhanced_tracking = device_property_present(dev, "enhanced-tracking");

	return 0;
}

struct cyttsp *cyttsp_probe(const struct cyttsp_bus_ops *bus_ops,
			    struct device *dev, int irq, size_t xfer_buf_size)
{
	/*
	 * VCPIN is the analog voltage supply
	 * VDD is the digital voltage supply
	 */
	static const char * const supplies[] = { "vcpin", "vdd" };
	struct cyttsp *ts;
	struct input_dev *input_dev;
	int error;

	ts = devm_kzalloc(dev, sizeof(*ts) + xfer_buf_size, GFP_KERNEL);
	if (!ts)
		return ERR_PTR(-ENOMEM);

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev)
		return ERR_PTR(-ENOMEM);

	ts->dev = dev;
	ts->input = input_dev;
	ts->bus_ops = bus_ops;
	ts->irq = irq;

	error = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(supplies),
					       supplies);
	if (error) {
		dev_err(dev, "Failed to enable regulators: %d\n", error);
		return ERR_PTR(error);
	}

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio)) {
		error = PTR_ERR(ts->reset_gpio);
		dev_err(dev, "Failed to request reset gpio, error %d\n", error);
		return ERR_PTR(error);
	}

	error = cyttsp_parse_properties(ts);
	if (error)
		return ERR_PTR(error);

	init_completion(&ts->bl_ready);

	input_dev->name = "Cypress TTSP TouchScreen";
	input_dev->id.bustype = bus_ops->bustype;
	input_dev->dev.parent = ts->dev;

	input_dev->open = cyttsp_open;
	input_dev->close = cyttsp_close;

	input_set_drvdata(input_dev, ts);

	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_Y);
	/* One byte for width 0..255 so this is the limit */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(input_dev, true, NULL);

	error = input_mt_init_slots(input_dev, CY_MAX_ID, INPUT_MT_DIRECT);
	if (error) {
		dev_err(dev, "Unable to init MT slots.\n");
		return ERR_PTR(error);
	}

	error = devm_request_threaded_irq(dev, ts->irq, NULL, cyttsp_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  "cyttsp", ts);
	if (error) {
		dev_err(ts->dev, "failed to request IRQ %d, err: %d\n",
			ts->irq, error);
		return ERR_PTR(error);
	}

	cyttsp_hard_reset(ts);

	error = cyttsp_power_on(ts);
	if (error)
		return ERR_PTR(error);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(ts->dev, "failed to register input device: %d\n",
			error);
		return ERR_PTR(error);
	}

	return ts;
}
EXPORT_SYMBOL_GPL(cyttsp_probe);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");
