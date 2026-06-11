// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Core entry point for the A6 driver: i2c_driver bind/unbind, suspend/resume
 * PM ops, top-half IRQ handler + its bottom-half work, the periodic
 * force-wake timer/work, device state initialisation, and module
 * registration. The bulk of the driver lives in the sibling files
 * (a6_dispatch.c, a6_sysfs.c, a6_cdev.c, a6_power_supply.c,
 *  a6_sbw_gpio.c, a6_regs.c).
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "a6_internal.h"

int a6_debug_mask;
int a6_tp_irq_count;
int a6_t2s_dup_correct;

module_param_named(
		   debug_mask, a6_debug_mask, int, 0664
		  );

module_param_named(
		   a6_irq_count, a6_tp_irq_count, int, 0664
		  );

module_param_named(
		   t2s_dup_correct, a6_t2s_dup_correct, int, 0664
		  );

#if defined PROFILE_USAGE
bool reset_active;
uint32_t start_time;
int32_t diff_time;

uint32_t start_last_a6_activity;
#endif

int32_t a6_init_state(struct i2c_client *client)
{
	int32_t ret = 0;
	struct a6_device_state *state = i2c_get_clientdata(client);
	struct a6_register_desc *reg_desc;
	uint8_t vals[id_size];

	// early initialization of cached rsense to prevent un-initialized usage
	// due to early-boot i2c failures.
	state->cached_rsense_val = RSENSE_DEFAULT;

	/* (1) enable external/internal wake, if required */
	/* periodic wake capability enabled? */
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		struct a6_wake_ops *wake_ops = (struct a6_wake_ops *)state->plat_data->wake_ops;

		/* enable external periodic wake? */
		if (wake_ops->enable_periodic_wake) {
			pr_err("%s: enabling external PMIC-generated A6 wake.\n", __func__);

			wake_ops->enable_periodic_wake(wake_ops->data);

			/* disable a6 internal-wake... */
			reg_desc = &a6_register_desc_arr[34];
			/* format wakeup parameters in register format */
			vals[0] = 0;
			ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
			if (ret < 0)
				goto err0;
		} else if (wake_ops->internal_wake_enable_state) {
			/* enable internal periodic wake? */
			uint32_t wake_period = 0, wake_enable = 1;

			pr_err("%s: enabling A6 internal wake.\n", __func__);
			/* get wake_period from ops if available */
			if (wake_ops->internal_wake_period) {
				wake_period = wake_ops->internal_wake_period(wake_ops->data);
				if (wake_period > 0x100)
					wake_period = 0x100;
			}
			/* wake_period defaults to 0 if not set */

			reg_desc = &a6_register_desc_arr[34];

			/* disable wake if no period set */
			if (!wake_period)
				wake_enable = 0;

			/*
			 * bit [4]: Enable sleep.
			 * bit [3]: Enable automatic wakeup.
			 * bits [2:0]: Sleep period.
			 */
			if (wake_period) {
				wake_period = find_last_bit((const unsigned long *)&wake_period, 32);
				wake_period--;
			}
			vals[0] = 0x10 | wake_enable << 3 | (wake_period & 0x07);
			pr_err("%s: TS2_I2C_WAKEUP_PERIOD = 0x%02x\n",
			       __func__, vals[0]);
			ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
			if (ret < 0)
				goto err0;
		} else {
			/*
			 * CAP_PERIODIC_WAKE was set in DT/board flags but
			 * neither external nor internal wake callback was
			 * registered. This is a driver-configuration bug;
			 * warn loudly and proceed with the wake feature
			 * effectively disabled rather than panicking.
			 */
			WARN_ONCE(1,
				  "%s: CAP_PERIODIC_WAKE set without wake_ops callback\n",
				  __func__);
		}
	}
	/* periodic wake capability not defined: force a6 awake using SBW_WAKEUP */
	else if (state->wakeup_gpio) {
		gpiod_set_value_cansleep(state->wakeup_gpio, 1);
	}

	/* (2) cache rsense val */
	reg_desc = &a6_register_desc_arr[19];

	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	/* ignore read error for rsense */

	/*
	 * rsense == 0: invalid and results in default
	 * (logic compatible with legacy w1 driver implementation)
	 */
	if (vals[0]) {
		state->cached_rsense_val = 1000 / vals[0];
		if (!state->cached_rsense_val)
			state->cached_rsense_val = RSENSE_DEFAULT;
	}
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
		   "%s: (cached) rsense value: %u\n", __func__, state->cached_rsense_val);

	/* (3) enable irqs: MASK_A2A_CONNECT_CHANGE, MASK_FLAGS_CHANGE */
	reg_desc = &a6_register_desc_arr[3];

	vals[0] = 0xf3;
	ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0)
		goto err0;

	/*
	 * (4) enable irqs:
	 * MASK_BAT_TEMP_HIGH, MASK_BAT_TEMP_LOW,
	 * MASK_BAT_VOLT_LOW, MASK_BAT_RARC_CRIT,
	 * MASK_BAT_RARC_LOW2, MASK_BAT_RARC_LOW1
	 */
	reg_desc = &a6_register_desc_arr[2];

	vals[0] = 0xc0;
	ret = a6_i2c_write_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0)
		goto err0;

	/* read version */
	reg_desc = &a6_register_desc_arr[32];

	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(client, reg_desc->id, reg_desc->num_ids, vals);
	if (ret < 0)
		pr_err("%s: error reading A6 version\n", __func__);
	else {
		uint8_t buf[150];

		reg_desc->format(state, vals, buf, sizeof(buf));
		pr_info("%s", buf);
	}

	// if first init (device boot): clear all interrupt status regs because we might
	// have missed the level-triggered a6-irq during device boot and the user-space
	// components read a6 registers to set their initial state correctly...
	if (!test_bit(IS_INITIALIZED_BIT, state->flags)) {
		vals[0] = 0xff;

		/* clear int_status3 */
		reg_desc = &a6_register_desc_arr[7];
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (ret < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			goto err0;
		}

		/* clear int_status2 */
		reg_desc = &a6_register_desc_arr[6];
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (ret < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			goto err0;
		}

		/* clear int_status1 */
		reg_desc = &a6_register_desc_arr[5];
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc->id, reg_desc->num_ids, vals);
		if (ret < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			__func__, reg_desc->debug_name, reg_desc->id[0]);
			goto err0;
		}
	}

	set_bit(IS_INITIALIZED_BIT, state->flags);

err0:
	return ret;
}

/*
 *  a6 interrupt handler
 */
irqreturn_t a6_irq(int irq, void *dev_id)
{
	struct a6_device_state *state = (struct a6_device_state *)dev_id;

	a6_tp_irq_count++;
#if defined PROFILE_USAGE
	/*
	 * if (reset_active) {
	 *	diff_time = (long)jiffies - (long)start_time;
	 *	reset_active = false;
	 *	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
	 *		   "A6_IRQ toggle post power-cycle after: %d ms\n",
	 *		   diff_time*1000/HZ);
	 * }
	 */
#endif


	if (test_bit(IS_SUSPENDED, state->flags))
		set_bit(INT_PENDING, state->flags);
	else
		queue_work(state->ka6d_workqueue, &state->a6_irq_work);

	return IRQ_HANDLED;
}

void a6_irq_work_handler(struct work_struct *work)
{
	struct a6_device_state *state =
			container_of(work, struct a6_device_state, a6_irq_work);
	struct a6_register_desc *reg_desc_status3, *reg_desc_status2;
	uint8_t vals[id_size], reg_val_status3 = 0, reg_val_status2 = 0;
	int32_t ret = 0;
	char *envp[] = {
		[0] = "A6_ACTION=EMERGENCY_RESET_NOTIFY",
		[1] = NULL,
		[2] = "A6_ACTION=LOG_THRESHOLD_NOTIFY",
		[3] = NULL,
		[4] = "A6_ACTION=CHARGE_SOURCE_NOTIFY",
		[5] = NULL,
		[6] = "A6_ACTION=PERCENT_LOW_WARN1_NOTIFY",
		[7] = NULL,
		[8] = "A6_ACTION=PERCENT_LOW_WARN2_NOTIFY",
		[9] = NULL,
		[10] = "A6_ACTION=PECENT_LOW_CRIT_NOTIFY",
		[11] = NULL,
		[12] = "A6_ACTION=VOLTAGE_LOW_CRIT_NOTIFY",
		[13] = NULL,
		[14] = "A6_ACTION=TEMP_LOW_CRIT_NOTIFY",
		[15] = NULL,
		[16] = "A6_ACTION=TEMP_HIGH_CRIT_NOTIFY",
		[17] = NULL,
		[18] = "A6_ACTION=A2A_CONNECT_NOTIFY",
		[19] = NULL
	};

	// block irq processing while a6 fw flashing is in progress because i2c requests
	// will fail anyway and we dont' want to fiddle with SBW_WKUP while flashing is
	// in progress...

	/*
	 * Hot-plug support: If device was not initialized during probe
	 * (e.g., Touchstone wasn't connected), try to initialize now.
	 * This enables connecting Touchstone after boot.
	 */
	if (!test_bit(IS_INITIALIZED_BIT, state->flags)) {
		int init_rc;

		pr_info("%s: device not initialized, attempting hot-plug init\n",
			__func__);

		init_rc = a6_init_state(state->i2c_dev);
		if (init_rc < 0) {
			pr_debug_ratelimited("%s: hot-plug init failed (%d), device not ready\n",
					     __func__, init_rc);
			return;
		}

		/* Device initialized successfully, register power supply */
		pr_info("%s: hot-plug init succeeded, registering power supply\n",
			__func__);
		init_rc = a6_register_power_supply(state);
		if (init_rc < 0) {
			pr_err("%s: failed to register power supply on hot-plug: %d\n",
			       __func__, init_rc);
			/* Continue anyway - device is initialized */
		}
	}

	// critsec for manipulating flags
	mutex_lock(&state->dev_mutex);

	// busy?
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		// yes: are we in bootload phase?
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			// no: so go ahead and allow concurrent i2c ops (these are
			// synchronized separately to allow priority based execution).
			break;
		}

		// in bootload phase: get on a waitq
		mutex_unlock(&state->dev_mutex);
		pr_err("%s: about to wait for device non-busy...\n", __func__);

		// bootload bit set? wait to be cleared (at least 5 mins: in jiffies)
		ret = wait_event_interruptible_timeout(state->dev_busyq,
				!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags), 300*HZ);
		if (!ret) {
			pr_err("%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		// we're about to manipulate flags again: acquire critsec
		mutex_lock(&state->dev_mutex);
	}

	/* increment busy refcount */
	state->busy_count++;

	// done with flags: exit critsec
	mutex_unlock(&state->dev_mutex);

	/* determine irq cause */
	reg_desc_status3 = &a6_register_desc_arr[7];
	reg_desc_status2 = &a6_register_desc_arr[6];

	/* (1) read int_status3: emergency reset and charge-source change */
	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(state->i2c_dev, reg_desc_status3->id, reg_desc_status3->num_ids, vals);
	if (ret < 0) {
		pr_err("%s: error reading reg: %s, id: 0x%x\n",
		       __func__, reg_desc_status3->debug_name, reg_desc_status3->id[0]);
		goto err0;
	}
	reg_val_status3 = vals[0];

	/* (2) read int_status2: other irq causes */
	memset(vals, 0, sizeof(vals));
	ret = a6_i2c_read_reg(state->i2c_dev, reg_desc_status2->id, reg_desc_status2->num_ids, vals);
	if (ret < 0) {
		pr_err("%s: error reading reg: %s, id: 0x%x\n",
		       __func__, reg_desc_status2->debug_name, reg_desc_status2->id[0]);
		goto err0;
	}
	reg_val_status2 = vals[0];

	/* (3) now clear all status bits: this "releases" the irq line early */
	if (reg_val_status3) {
		vals[0] = reg_val_status3;
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc_status3->id, reg_desc_status3->num_ids, vals);
		if (ret < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			       __func__, reg_desc_status3->debug_name, reg_desc_status3->id[0]);
			goto err0;
		}
	}
	if (reg_val_status2) {
		vals[0] = reg_val_status2;
		ret = a6_i2c_write_reg(state->i2c_dev, reg_desc_status2->id, reg_desc_status2->num_ids, vals);
		if (ret < 0) {
			pr_err("%s: error writing reg: %s, id: 0x%x\n",
			       __func__, reg_desc_status3->debug_name, reg_desc_status3->id[0]);
			goto err0;
		}
	}

	/* (4) process emergency reset and charge-source changes...*/
	if (reg_val_status3) {
		/* emergency reset? */
		if (reg_val_status3 & TS2_I2C_INT_3_RESET) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: emergency reset detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[0]);
			/*
			 * this condition overrides all others and we can
			 * probably skip resetting the status bits
			 */
			goto err0;
		}

		/* a2a connect change? */
		if (reg_val_status3 & TS2_I2C_INT_3_A2A_CONNECT_CHANGE) {
			struct a6_register_desc *reg_desc_charger;
			uint8_t chg_vals[id_size];

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: a2a connect change detected.\n",
				   __func__);
			pr_err("%s: a2a connect change detected.\n", __func__);
			reg_desc_charger = &a6_register_desc_arr[31];
			memset(chg_vals, 0, sizeof(chg_vals));
			if (a6_i2c_read_reg(state->i2c_dev, reg_desc_charger->id,
					     reg_desc_charger->num_ids, chg_vals) < 0) {
				pr_err("%s: error reading reg: %s, id: 0x%x\n",
				       __func__, reg_desc_charger->debug_name,
				       reg_desc_charger->id[0]);
			} else {
				if (chg_vals[0] & TS2_I2C_FLAGS_2_A2A_CONNECT)
					set_bit(A2A_CONNECTED, state->flags);
				else
					clear_bit(A2A_CONNECTED, state->flags);
				/* Send uevent */
				kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[18]);
			}
		}

		/* charger-source change? */
		if (reg_val_status3 & TS2_I2C_INT_3_FLAGS_CHANGE) {
			struct a6_register_desc *reg_desc_charger;
			struct a6_register_desc *reg_desc_puck_prio;
			uint8_t chg_vals[id_size];
			uint8_t prio_val;
			static unsigned long last_handshake_jiffies;

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: charger-source change detected.\n",
				   __func__);

			/*
			 * Read FLAGS_2 to check if Touchstone puck is physically
			 * detected but A2A communication hasn't started yet.
			 * If so, toggle puck_priority to initiate A2A handshake.
			 * This is needed because the A6 firmware requires the
			 * puck_priority bit to be toggled to start A2A.
			 */
			reg_desc_charger = &a6_register_desc_arr[31];
			memset(chg_vals, 0, sizeof(chg_vals));
			if (a6_i2c_read_reg(state->i2c_dev, reg_desc_charger->id,
					    reg_desc_charger->num_ids, chg_vals) >= 0) {
				/*
				 * PUCK_DETECT (0x01) = puck physically detected via coils
				 * PUCK (0x02) = A2A communication established
				 * If we have detection but no A2A, trigger handshake
				 */
				if ((chg_vals[0] & TS2_I2C_FLAGS_2_PUCK_DETECT) &&
				    !(chg_vals[0] & TS2_I2C_FLAGS_2_PUCK)) {
					/*
					 * Rate-limit handshake attempts to once per second.
					 * Rapid Touchstone connect/disconnect can flood IRQs,
					 * causing excessive I2C traffic and system instability.
					 */
					if (time_after(jiffies, last_handshake_jiffies + HZ)) {
						last_handshake_jiffies = jiffies;
						pr_info("%s: Touchstone detected, initiating A2A handshake\n",
							__func__);

						reg_desc_puck_prio = &a6_register_desc_arr[30];
						/* Clear puck_priority first */
						prio_val = 0;
						a6_i2c_write_reg(state->i2c_dev, reg_desc_puck_prio->id,
								 reg_desc_puck_prio->num_ids, &prio_val);
						/* Small delay for firmware to process */
						usleep_range(10000, 20000);
						/* Set puck_priority to trigger A2A */
						prio_val = TS2_I2C_FLAGS_0_PUCK_PRIORITY;
						a6_i2c_write_reg(state->i2c_dev, reg_desc_puck_prio->id,
								 reg_desc_puck_prio->num_ids, &prio_val);
					}
				}
			}

			/* next, unblock task on charger_source_notify node */
			//sysfs_notify_dirent(state->notify_nodes[DIRENT_CHG_SRC_NOTIFY]);
			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[4]);
		}

		/* log threshold change? */
		if (reg_val_status3 & TS2_I2C_INT_3_LOG) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: log threshold detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[2]);
		}
	}

	/* (5) process other irq causes... */
	if (reg_val_status2) {
		/* battery low critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_RARC_CRIT) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery low critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[10]);
		}

		/* battery voltage low critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_VOLT_LOW) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery voltage low critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[12]);
		}

		/* battery temp high critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_TEMP_HIGH) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery temp high critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[16]);
		}

		/* battery temp low critical? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_TEMP_LOW) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery temp low critical detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[14]);
		}

		/* battery low percent warn2? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_RARC_LOW2) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery low percent warn2 detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[8]);
		}

		/* battery low percent warn1? */
		if (reg_val_status2 & TS2_I2C_INT_2_BAT_RARC_LOW1) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: battery low percent warn1 detected.\n",
				   __func__);

			/* Send uevent */
			kobject_uevent_env(&state->i2c_dev->dev.kobj, KOBJ_CHANGE, &envp[6]);
		}
	}

err0:
	/* reset busy state with proper locking */
	mutex_lock(&state->dev_mutex);
	/* decrement busy refcount */
	if (state->busy_count)
		state->busy_count--;
	if (!state->busy_count)
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: Visited\n", __func__);
}

void a6_force_wake_work_handler(struct work_struct *work)
{
	struct a6_device_state *state =
			container_of(work, struct a6_device_state, a6_force_wake_work);
	struct a6_wake_ops *wake_ops = (struct a6_wake_ops *)state->plat_data->wake_ops;
	long diff_time;

	diff_time = (long)jiffies - (long)start_last_a6_activity;
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: forcing sleep after: %ld ms\n",
		   __func__, diff_time * 1000/HZ);

	start_last_a6_activity = 0;

	// force A6 sleep and switch back to periodic wake...
	// * timer may be scheduled just after we clear FORCE_WAKE_ACTIVE_BIT
	//   and hence will result in the callback being invoked with
	//   FORCE_WAKE_ACTIVE_BIT cleared. We just ignore this case...
	mutex_lock(&state->a6_force_wake_mutex);
	if (!test_bit(DEVICE_BUSY_BIT, state->flags)) {
		if (test_bit(FORCE_WAKE_ACTIVE_BIT, state->flags)) {
			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
				"%s: disabling force_wake and enabling periodic_wake\n",
				__func__);
			/* force A6 sleep */
			if (wake_ops->force_sleep)
				wake_ops->force_sleep(wake_ops->data);
			/* enable periodic a6 wake (if defined) */
			if (wake_ops->enable_periodic_wake)
				wake_ops->enable_periodic_wake(wake_ops->data);
			/* now we are ready to clear FORCE_WAKE_ACTIVE_BIT */
			clear_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
	}
	mutex_unlock(&state->a6_force_wake_mutex);
}


/* Timer callback used to force sleep after a force wake - Modern API */
void a6_force_wake_timer_callback(struct timer_list *t)
{
	struct a6_device_state *state = timer_container_of(state, t, a6_force_wake_timer);
	int32_t rc;

	rc = queue_work(state->ka6d_fw_workqueue, &state->a6_force_wake_work);
	if (!rc) {
		dev_err(&state->i2c_dev->dev,
			"Failed queueing force_wake work item\n");
	}
}

/* a6_probe() - Modern probe function */
static int a6_probe(struct i2c_client *client)
{
	struct a6_device_state *state;
	const char *dev_name;
	int rc, irq;

	/* Allocate driver state with automatic cleanup */
	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->a2a_rd_buf = devm_kzalloc(&client->dev, A2A_RD_BUFF_SIZE, GFP_KERNEL);
	if (!state->a2a_rd_buf)
		return -ENOMEM;

	state->a2a_wr_buf = devm_kzalloc(&client->dev, A2A_WR_BUFF_SIZE, GFP_KERNEL);
	if (!state->a2a_wr_buf)
		return -ENOMEM;

	/* Store I2C client device in state */
	state->i2c_dev = client;
	i2c_set_clientdata(client, state);

	/* Get device index from device tree */
	rc = device_property_read_u32(&client->dev, "palm,device-index", &state->device_index);
	if (rc) {
		/* Fallback: use I2C address to determine index */
		state->device_index = (client->addr == 0x31) ? 0 : 1;
		dev_info(&client->dev, "No device-index in DT, using %d based on address\n",
			 state->device_index);
	}

	/* Initialize mutexes and wait queues */
	mutex_init(&state->dev_mutex);
	mutex_init(&state->a6_force_wake_mutex);
	init_waitqueue_head(&state->dev_busyq);

#ifdef A6_PQ
	init_completion(&state->aq_enq_complete);
	init_completion(&state->aid_exit_complete);
	mutex_init(&state->aq_mutex);
	INIT_LIST_HEAD(&state->aq_head);
#endif

	/* Zero-init wait flags */
	bitmap_zero(state->flags, SIZE_FLAGS);

	/*
	 * Initialize rsense to default value early to prevent division by zero
	 * if power_supply queries properties before a6_init_state() runs.
	 * a6_init_state() will read the actual value from the device later.
	 */
	state->cached_rsense_val = RSENSE_DEFAULT;

	/* Create workqueues with managed cleanup */
	state->ka6d_workqueue = alloc_ordered_workqueue("ka6d", WQ_MEM_RECLAIM);
	if (!state->ka6d_workqueue)
		return -ENOMEM;

	state->ka6d_fw_workqueue = alloc_ordered_workqueue("ka6d_fwd", WQ_MEM_RECLAIM);
	if (!state->ka6d_fw_workqueue) {
		destroy_workqueue(state->ka6d_workqueue);
		return -ENOMEM;
	}

	/* Initialize work structures */
	INIT_WORK(&state->a6_irq_work, a6_irq_work_handler);
	INIT_WORK(&state->a6_force_wake_work, a6_force_wake_work_handler);

	/*
	 * Initialize force wake timer unconditionally. This is required because
	 * timer_delete() is called in I2C read/write paths regardless of whether
	 * CAP_PERIODIC_WAKE is set. Without initialization, timer_delete() on
	 * an uninitialized timer causes undefined behavior/crashes.
	 */
	timer_setup(&state->a6_force_wake_timer, a6_force_wake_timer_callback, 0);

	state->cpufreq_hold_flag = 0;

	/* Get GPIO descriptors from device tree */
	state->tck_gpio = devm_gpiod_get(&client->dev, "tck", GPIOD_OUT_LOW);
	if (IS_ERR(state->tck_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(state->tck_gpio),
				     "Failed to get TCK GPIO\n");

	state->tdio_gpio = devm_gpiod_get(&client->dev, "tdio", GPIOD_OUT_HIGH);
	if (IS_ERR(state->tdio_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(state->tdio_gpio),
				     "Failed to get TDIO GPIO\n");

	state->wakeup_gpio = devm_gpiod_get(&client->dev, "wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(state->wakeup_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(state->wakeup_gpio),
				     "Failed to get wakeup GPIO\n");

	state->irq_gpio = devm_gpiod_get_optional(&client->dev, "interrupt", GPIOD_IN);
	if (IS_ERR(state->irq_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(state->irq_gpio),
				     "Failed to get IRQ GPIO\n");

	/*
	 * Set up legacy platform data for compatibility with existing code
	 * Modern code should use GPIO descriptors directly, but SBW/JTAG
	 * code still uses the legacy interface
	 */
	state->plat_data = devm_kzalloc(&client->dev, sizeof(*state->plat_data),
					GFP_KERNEL);
	if (!state->plat_data)
		return -ENOMEM;

	state->plat_data->sbw_ops = &a6_gpio_sbw_ops;
	state->plat_data->dev_name = (state->device_index == 0) ? A6_DEVICE_0 : A6_DEVICE_1;

	/* Set current SBW state for GPIO wrapper functions */
	current_sbw_state = state;

	/* Set up interrupt */
	if (state->irq_gpio) {
		irq = gpiod_to_irq(state->irq_gpio);
	} else if (client->irq) {
		irq = client->irq;
	} else {
		dev_warn(&client->dev, "No interrupt configured\n");
		irq = 0;
	}

	if (irq > 0) {
		rc = devm_request_threaded_irq(&client->dev, irq, NULL,
					       a6_irq,
					       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					       "a6", state);
		if (rc)
			return dev_err_probe(&client->dev, rc,
					     "Failed to request IRQ\n");
	}

	/* Register as misc device */
	dev_name = (state->device_index == 0) ? A6_DEVICE_0 : A6_DEVICE_1;

	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.name = dev_name;
	state->mdev.fops = &a6_fops;
	rc = misc_register(&state->mdev);
	if (rc < 0)
		return dev_err_probe(&client->dev, rc,
				     "Failed to register misc device\n");

	/* Register pmem diagnostic device */
	state->pmem_mdev.minor = MISC_DYNAMIC_MINOR;
	snprintf(state->pmem_dev_name, sizeof(state->pmem_dev_name),
		 "%s_diag", dev_name);
	state->pmem_mdev.name = state->pmem_dev_name;
	state->pmem_mdev.fops = &a6_pmem_fops;
	rc = misc_register(&state->pmem_mdev);
	if (rc < 0) {
		dev_err(&client->dev, "Failed to register pmem misc device\n");
		goto err_misc;
	}

	/* Create device files */
	rc = a6_create_dev_files(state, &client->dev);
	if (rc < 0) {
		dev_err(&client->dev, "Failed to create sysfs files: %d\n", rc);
		goto err_pmem;
	}

#ifdef A6_PQ
	rc = a6_start_ai_dispatch_task(state);
	if (rc < 0) {
		dev_err(&client->dev, "Failed to start dispatch task: %d\n", rc);
		goto err_devfiles;
	}
#endif

	/*
	 * Try to initialize device state. If this fails, the device is not
	 * present or has corrupt firmware. We still register the misc device
	 * to allow firmware flashing via ioctl, but skip power_supply
	 * registration to avoid continuous polling errors.
	 */
	rc = a6_init_state(client);
	if (rc < 0) {
		dev_warn(&client->dev,
			 "A6 device not responding (%d), hot-plug supported\n",
			 rc);
		dev_info(&client->dev,
			 "A6 misc device registered, power_supply will register on connect\n");
		return 0;
	}

	/* Register power supply - device is present and responding */
	rc = a6_register_power_supply(state);
	if (rc < 0) {
		dev_err(&client->dev,
			"Failed to register power supply: %d\n", rc);
		goto err_dispatch;
	}

#if defined(A6_PQ) && defined(A6_DEBUG)
	rc = a6_create_debug_interface(state);
	if (rc < 0)
		dev_warn(&client->dev, "Failed to create debug interface\n");
#endif

	/*
	 * Enable wake-capability at the device level. The DT wakeup-source
	 * property + this call together tell the PM core that the A6 IRQ
	 * may wake the system from suspend; the actual enable_irq_wake()
	 * happens in a6_suspend() below. Match legacy vendor behaviour where
	 * the A6 power-GPIO IRQ was always set up as a wake source so the
	 * SoC could sleep through long idle periods and only wake on
	 * charge-source / threshold / emergency-reset events.
	 */
	if (client->irq)
		device_init_wakeup(&client->dev, true);

	dev_info(&client->dev, "A6 battery controller initialized\n");
	return 0;

	/*
	 * Cleanup chain on probe failure. Each rung undoes one
	 * registration / setup step; jump to the rung that matches the
	 * last successful step.  Order mirrors registration order
	 * (reverse for teardown):
	 *
	 *   misc_register(mdev)         <- err_misc
	 *   misc_register(pmem_mdev)    <- err_pmem
	 *   a6_create_dev_files()       <- err_devfiles
	 *   a6_start_ai_dispatch_task() <- err_dispatch  (A6_PQ only)
	 *
	 * a6_init_state() and a6_register_power_supply() have no
	 * matching teardown step here — init_state failure returns 0
	 * (hot-plug path); power_supply failure jumps to err_dispatch
	 * because the dispatch task was already running.
	 */
err_dispatch:
#ifdef A6_PQ
	a6_stop_ai_dispatch_task(state);
#endif
err_devfiles:
	a6_remove_dev_files(state, &client->dev);
err_pmem:
	misc_deregister(&state->pmem_mdev);
err_misc:
	misc_deregister(&state->mdev);
	return rc;
}

/* a6_i2c_remove() */
static void a6_i2c_remove(struct i2c_client *client)
{
	struct a6_device_state *state = (struct a6_device_state *)i2c_get_clientdata(client);

	/*
	 * Tear down in reverse of probe order. Until this was complete the
	 * driver leaked both /sys/devices/virtual/misc/a6_N and the
	 * misc-device minor number, so a subsequent probe (re-bind via the
	 * driver-binding sysfs, or unbind/bind cycle from userspace) failed
	 * inside misc_register() with -EEXIST:
	 *
	 *   sysfs: cannot create duplicate filename '/devices/virtual/misc/a6_1'
	 *   kobject_add_internal failed for a6_1 with -EEXIST
	 *   a6-battery 2-0032: error -EEXIST: Failed to register misc device
	 *   a6-battery 2-0032: probe with driver a6-battery failed with error -17
	 *
	 * device_init_wakeup() also needs the matching teardown call, or the
	 * dev's power.can_wakeup flag stays set across rebinds.
	 */
	a6_remove_dev_files(state, &client->dev);

	if (state->ka6d_workqueue)
		destroy_workqueue(state->ka6d_workqueue);

	if (state->ka6d_fw_workqueue)
		destroy_workqueue(state->ka6d_fw_workqueue);

	misc_deregister(&state->pmem_mdev);
	misc_deregister(&state->mdev);

	device_init_wakeup(&client->dev, false);
}

#ifdef CONFIG_PM_SLEEP
/*
 * Suspend / resume PM ops.
 *
 * Wake-source semantics:
 *   device_init_wakeup(dev, true) is called from probe so the device is
 *   permanently flagged wake-capable. During each suspend, if user policy
 *   still allows wake (sysfs /sys/.../power/wakeup = "enabled"),
 *   enable_irq_wake() actually marks the IRQ as a system wake source so
 *   the SoC can drop into deeper sleep states while still being
 *   interruptible by A6 events (charge source detect, battery threshold
 *   crossings, emergency-reset latch).
 *
 *   This mirrors the legacy vendor a6 driver which did:
 *       set_bit(IS_SUSPENDED, ...);
 *       enable_irq_wake(gpio_to_irq(pwr_gpio));
 *
 *   The previous mainline implementation called device_init_wakeup() in
 *   suspend, which doesn't actually arm the IRQ as a wake source — it
 *   only sets the wake-capability flag, which is a probe-time concern.
 *   That left the A6 IRQ un-armed for wake, so charge events couldn't
 *   wake the host from suspend.
 */
static int a6_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct a6_device_state *state = i2c_get_clientdata(client);

	set_bit(IS_SUSPENDED, state->flags);

	if (client->irq && device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int a6_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct a6_device_state *state = i2c_get_clientdata(client);

	if (client->irq && device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	clear_bit(IS_SUSPENDED, state->flags);

	/* Queue pending interrupt work if needed */
	if (test_and_clear_bit(INT_PENDING, state->flags))
		queue_work(state->ka6d_workqueue, &state->a6_irq_work);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(a6_pm_ops, a6_suspend, a6_resume);
#endif /* CONFIG_PM_SLEEP */

/* Device tree match table */
static const struct of_device_id a6_of_match[] = {
	{ .compatible = "palm,a6-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, a6_of_match);

/* I2C device ID table */
static const struct i2c_device_id a6_ids[] = {
	{ "a6-battery", 0 },
	{ "a6_0", 0 },  /* Legacy compatibility */
	{ "a6_1", 0 },  /* Legacy compatibility */
	{ }
};
MODULE_DEVICE_TABLE(i2c, a6_ids);

/* I2C driver structure */
static struct i2c_driver a6_driver = {
	.driver = {
		.name = "a6-battery",
		.of_match_table = a6_of_match,
		.pm = pm_sleep_ptr(&a6_pm_ops),
	},
	.id_table = a6_ids,
	.probe = a6_probe,
	.remove = a6_i2c_remove,
};

module_i2c_driver(a6_driver);

MODULE_DESCRIPTION("Palm A6 Battery Controller Driver");
MODULE_LICENSE("GPL");
