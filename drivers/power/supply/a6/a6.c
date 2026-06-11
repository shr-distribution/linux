// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs.
 *
 * Core entry point for the A6 driver: i2c_driver bind/unbind, the
 * threaded wakeup IRQ + its bottom-half work, suspend/resume PM ops,
 * device-state initialisation, and module registration. The bulk of
 * the driver lives in the sibling files (a6_dispatch.c for the
 * synchronous I2C helpers, a6_power_supply.c for the power_supply
 * class adapter).
 */

#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "a6_internal.h"

#if IS_ENABLED(CONFIG_BATTERY_PALM_A6_A2A_COMM)
#include "palm-a6-aux.h"
#endif

/*
 * a6_init_state() - bring the A6 controller out of sleep and arm the
 *                   interrupt sources the driver cares about
 *
 * Called once at probe (if the device responds) and again lazily from
 * the IRQ bottom half if the controller comes online later (e.g. a
 * Touchstone puck is connected after boot). Must be called with
 * @state->dev_mutex held.
 */
int a6_init_state(struct i2c_client *client)
{
	struct a6_device_state *state = i2c_get_clientdata(client);
	static const u16 rsense_id[]	= { TS2_I2C_BAT_RSNSP };
	static const u16 mask1_id[]	= { TS2_I2C_INT_MASK_1 };
	static const u16 mask2_id[]	= { TS2_I2C_INT_MASK_2 };
	static const u16 mask3_id[]	= { TS2_I2C_INT_MASK_3 };
	static const u16 status2_id[]	= { TS2_I2C_INT_STATUS_2 };
	static const u16 status3_id[]	= { TS2_I2C_INT_STATUS_3 };
	u8 val;
	int ret;

	/*
	 * Force A6 awake while we touch its registers, then settle so the
	 * MSP430 has time to exit LPM3/4 before the first I2C transfer.
	 */
	gpiod_set_value_cansleep(state->wakeup_gpio, 1);
	usleep_range(1000, 2000);

	/* Early default: prevent divide-by-zero in power_supply readers */
	state->cached_rsense_val = RSENSE_DEFAULT;

	/* Cache the rsense value. A read error or 0 leaves the default. */
	ret = a6_i2c_read_reg(client, rsense_id, 1, &val);
	if (ret == 0 && val) {
		u16 r = 1000 / val;

		if (r)
			state->cached_rsense_val = r;
	}

	/*
	 * INT_MASK_x registers use 1=disable / 0=enable. Spell the
	 * enabled bits via their named constants and write the
	 * bitwise complement so the wire value tracks the named set
	 * if more bits get enabled later.
	 *
	 * MASK_3: A2A connect-change + flags-change (LOG and RESET
	 * are intentionally left masked; v1 only consumes the two
	 * events that update power_supply state).
	 */
	val = (u8)~(TS2_I2C_INT_3_A2A_CONNECT_CHANGE |
		    TS2_I2C_INT_3_FLAGS_CHANGE);
	ret = a6_i2c_write_reg(client, mask3_id, 1, &val);
	if (ret < 0)
		goto out_release_wakeup;

	/* MASK_2: all six battery temperature / voltage / RARC threshold bits. */
	val = (u8)~(TS2_I2C_INT_2_BAT_TEMP_HIGH |
		    TS2_I2C_INT_2_BAT_TEMP_LOW |
		    TS2_I2C_INT_2_BAT_VOLT_LOW |
		    TS2_I2C_INT_2_BAT_RARC_CRIT |
		    TS2_I2C_INT_2_BAT_RARC_LOW2 |
		    TS2_I2C_INT_2_BAT_RARC_LOW1);
	ret = a6_i2c_write_reg(client, mask2_id, 1, &val);
	if (ret < 0)
		goto out_release_wakeup;

	/*
	 * MASK_1: explicitly disable every source. v1 has no consumer of
	 * any INT_STATUS_1 bit (COMM_RX_FULL / COMM_TX_EMPTY are owned by
	 * the optional T2S aux interface, not by this battery driver), so
	 * leave them masked here instead of relying on whatever default
	 * the firmware programmed. Without this, a stray STATUS_1 source
	 * could fire an IRQ that the bottom half never acks, hanging on
	 * a level-triggered line or producing storms on edge.
	 */
	val = 0xff;
	ret = a6_i2c_write_reg(client, mask1_id, 1, &val);
	if (ret < 0)
		goto out_release_wakeup;

	/*
	 * Clear stale interrupt status latched while the controller was
	 * powered up but no driver was listening. Userspace polls
	 * power_supply for initial state, so a missed level IRQ from boot
	 * would otherwise leave us deaf to the first real event.
	 * Only STATUS_3 and STATUS_2 are cleared — STATUS_1 sources are
	 * masked above, so there's no listener for them here.
	 */
	if (!test_bit(IS_INITIALIZED_BIT, state->flags)) {
		val = 0xff;
		ret = a6_i2c_write_reg(client, status3_id, 1, &val);
		if (ret < 0)
			goto out_release_wakeup;
		ret = a6_i2c_write_reg(client, status2_id, 1, &val);
		if (ret < 0)
			goto out_release_wakeup;
	}

	set_bit(IS_INITIALIZED_BIT, state->flags);
	ret = 0;

out_release_wakeup:
	/*
	 * Release the keep-awake assertion: on success the A6 will drop
	 * back to its low-power state and wake on its own when the IRQ
	 * line asserts; on error we still must not pin it awake.
	 */
	gpiod_set_value_cansleep(state->wakeup_gpio, 0);
	return ret;
}

/*
 * Top-half IRQ handler. The A6 IRQ is exposed as edge-falling on the
 * host GPIO (see DT binding), so we defer the actual register dance to
 * the bottom-half work item — the threaded IRQ acks the edge by
 * returning. During suspend, the IRQ is armed as a wake source and we
 * latch INT_PENDING so the bottom half runs at resume.
 */
irqreturn_t a6_irq(int irq, void *dev_id)
{
	struct a6_device_state *state = dev_id;

	if (test_bit(IS_SUSPENDED, state->flags))
		set_bit(INT_PENDING, state->flags);
	else
		queue_work(state->ka6d_workqueue, &state->a6_irq_work);

	return IRQ_HANDLED;
}

/*
 * Bottom-half: drain interrupt status, dispatch on cause, and notify
 * the power_supply class so userspace pollers and uevent consumers
 * see updated state.
 *
 * Holds dev_mutex across the I2C transactions so it serialises against
 * the lazy hot-plug init path that may also walk a6_init_state() the
 * first time the controller comes online.
 */
void a6_irq_work_handler(struct work_struct *work)
{
	struct a6_device_state *state =
		container_of(work, struct a6_device_state, a6_irq_work);
	struct i2c_client *client = state->i2c_dev;
	static const u16 status3_id[]	= { TS2_I2C_INT_STATUS_3 };
	static const u16 status2_id[]	= { TS2_I2C_INT_STATUS_2 };
	static const u16 flags2_id[]	= { TS2_I2C_FLAGS_2 };
	u8 status3 = 0, status2 = 0, val;
	int ret;

	mutex_lock(&state->dev_mutex);

	/*
	 * Hot-plug: if the controller wasn't responsive at probe time,
	 * try to initialise it now. The first IRQ after a Touchstone is
	 * docked typically lands here. power_supply was registered at
	 * probe; on success this lights up POWER_SUPPLY_PROP_PRESENT.
	 *
	 * a6_init_state() handles its own wakeup assert/release.
	 */
	if (!test_bit(IS_INITIALIZED_BIT, state->flags)) {
		ret = a6_init_state(client);
		if (ret < 0) {
			dev_dbg_ratelimited(&client->dev,
					    "hot-plug init failed: %d\n", ret);
			goto out;
		}
	}

	/*
	 * Assert wakeup_gpio around the status-register I2C transactions.
	 * The A6 firmware can NAK accesses while in low-power mode; the
	 * legacy driver always holds wake high before touching registers.
	 * Settle delay so the MSP430 exits LPM3/4 before the I2C address
	 * phase reaches it.
	 */
	gpiod_set_value_cansleep(state->wakeup_gpio, 1);
	usleep_range(1000, 2000);

	ret = a6_i2c_read_reg(client, status3_id, 1, &status3);
	if (ret < 0)
		goto out_release_wakeup;
	ret = a6_i2c_read_reg(client, status2_id, 1, &status2);
	if (ret < 0)
		goto out_release_wakeup;

	/* Ack: write-1-to-clear releases the IRQ assertion */
	if (status3)
		a6_i2c_write_reg(client, status3_id, 1, &status3);
	if (status2)
		a6_i2c_write_reg(client, status2_id, 1, &status2);

	/* Track A2A (accessory) connect state for the power_supply view */
	if (status3 & TS2_I2C_INT_3_A2A_CONNECT_CHANGE) {
		ret = a6_i2c_read_reg(client, flags2_id, 1, &val);
		if (ret == 0) {
			if (val & TS2_I2C_FLAGS_2_PUCK)
				set_bit(A2A_CONNECTED, state->flags);
			else
				clear_bit(A2A_CONNECTED, state->flags);
		}
	}

	if (status3 & TS2_I2C_INT_3_RESET)
		dev_warn_ratelimited(&client->dev, "A6 emergency reset\n");
	if (status3 & TS2_I2C_INT_3_LOG)
		dev_dbg(&client->dev, "A6 log threshold reached\n");
	if (status2 & TS2_I2C_INT_2_BAT_RARC_CRIT)
		dev_warn_ratelimited(&client->dev, "battery capacity critical\n");
	if (status2 & TS2_I2C_INT_2_BAT_VOLT_LOW)
		dev_warn_ratelimited(&client->dev, "battery voltage low\n");
	if (status2 & TS2_I2C_INT_2_BAT_TEMP_HIGH)
		dev_warn_ratelimited(&client->dev, "battery temperature high\n");
	if (status2 & TS2_I2C_INT_2_BAT_TEMP_LOW)
		dev_warn_ratelimited(&client->dev, "battery temperature low\n");

	/* Notify the power_supply class of any change */
	if (state->battery && (status3 || status2))
		power_supply_changed(state->battery);

out_release_wakeup:
	gpiod_set_value_cansleep(state->wakeup_gpio, 0);
out:
	mutex_unlock(&state->dev_mutex);
}

/*
 * devm action: cancel pending IRQ work and tear down the workqueue
 * before devres frees the threaded IRQ. Ordering matters: the IRQ
 * handler queues work on ka6d_workqueue, so the IRQ must be quiesced
 * first. devres unwinds in reverse-registration order, so we register
 * this action AFTER the IRQ to guarantee that ordering.
 */
static void a6_workqueue_release(void *data)
{
	struct a6_device_state *state = data;

	cancel_work_sync(&state->a6_irq_work);
	destroy_workqueue(state->ka6d_workqueue);
}

#if IS_ENABLED(CONFIG_BATTERY_PALM_A6_A2A_COMM)
/*
 * Auxiliary-bus glue. Exposes a thin facade over a6_i2c_read_reg /
 * a6_i2c_write_reg that the downstream-only a6_a2a_comm module
 * consumes to re-create the legacy /dev/a6_N + sysfs ABI. Both
 * callbacks take state->dev_mutex internally so the aux driver does
 * not need access to a6_internal.h.
 *
 * This entire block compiles out when CONFIG_BATTERY_PALM_A6_A2A_COMM=n
 * so the upstream-bound base driver carries no extra symbols.
 */
static int a6_aux_read_regs(struct i2c_client *client, const u16 *ids,
			    u32 num_ids, u8 *out)
{
	struct a6_device_state *state = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&state->dev_mutex);
	ret = a6_i2c_read_reg(client, ids, num_ids, out);
	mutex_unlock(&state->dev_mutex);
	return ret;
}

static int a6_aux_write_regs(struct i2c_client *client, const u16 *ids,
			     u32 num_ids, const u8 *in)
{
	struct a6_device_state *state = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&state->dev_mutex);
	ret = a6_i2c_write_reg(client, ids, num_ids, in);
	mutex_unlock(&state->dev_mutex);
	return ret;
}

static void a6_aux_dev_release(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);
	struct a6_aux_dev *a6dev = to_a6_aux_dev(adev);

	kfree(a6dev);
}

static void a6_aux_unregister(void *data)
{
	struct auxiliary_device *adev = data;

	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static int a6_register_aux(struct a6_device_state *state)
{
	struct i2c_client *client = state->i2c_dev;
	struct a6_aux_dev *a6dev;
	struct auxiliary_device *adev;
	int ret;

	a6dev = kzalloc_obj(*a6dev);
	if (!a6dev)
		return -ENOMEM;

	a6dev->client = client;
	a6dev->device_index = state->device_index;
	a6dev->read_regs = a6_aux_read_regs;
	a6dev->write_regs = a6_aux_write_regs;

	adev = &a6dev->adev;
	adev->name = "a2a-comm";
	adev->id = state->device_index;
	adev->dev.parent = &client->dev;
	adev->dev.release = a6_aux_dev_release;

	ret = auxiliary_device_init(adev);
	if (ret) {
		kfree(a6dev);
		return ret;
	}

	ret = auxiliary_device_add(adev);
	if (ret) {
		/*
		 * uninit triggers the release() callback which frees
		 * the wrapper - do not kfree() again here.
		 */
		auxiliary_device_uninit(adev);
		return ret;
	}

	return devm_add_action_or_reset(&client->dev, a6_aux_unregister, adev);
}
#else
static inline int a6_register_aux(struct a6_device_state *state) { return 0; }
#endif /* CONFIG_BATTERY_PALM_A6_A2A_COMM */

static int a6_probe(struct i2c_client *client)
{
	struct a6_device_state *state;
	int ret;

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->i2c_dev = client;
	i2c_set_clientdata(client, state);

	/*
	 * Distinguish the two A6 controllers on TouchPad 4G by their I2C
	 * reg address (0x31 vs 0x32). This is the only difference between
	 * the two instances; no DT property is needed.
	 */
	state->device_index = (client->addr == 0x31) ? 0 : 1;

	mutex_init(&state->dev_mutex);
	bitmap_zero(state->flags, SIZE_FLAGS);
	state->cached_rsense_val = RSENSE_DEFAULT;

	/* GPIO descriptors */
	state->wakeup_gpio = devm_gpiod_get(&client->dev, "wakeup",
					    GPIOD_OUT_LOW);
	if (IS_ERR(state->wakeup_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(state->wakeup_gpio),
				     "failed to get wakeup GPIO\n");

	state->ka6d_workqueue = alloc_ordered_workqueue("ka6d",
							WQ_MEM_RECLAIM |
							WQ_FREEZABLE);
	if (!state->ka6d_workqueue)
		return -ENOMEM;

	INIT_WORK(&state->a6_irq_work, a6_irq_work_handler);

	/*
	 * Register the power_supply unconditionally. If a6_init_state()
	 * below fails (controller not yet responsive at probe), userspace
	 * just sees POWER_SUPPLY_PROP_PRESENT = 0 until the IRQ bottom
	 * half successfully initialises the controller.
	 *
	 * The workqueue is non-devres at this point, so on a failure
	 * return we must destroy it by hand — devres won't know about it
	 * until devm_add_action_or_reset() below.
	 */
	ret = a6_register_power_supply(state);
	if (ret) {
		destroy_workqueue(state->ka6d_workqueue);
		return ret;
	}

	/*
	 * Register workqueue teardown AFTER power_supply registration so
	 * the devres LIFO unwind order is:
	 *
	 *   free_irq()         -> stops new a6_irq from queueing work,
	 *                         and waits for the threaded handler;
	 *   workqueue_release()-> cancel_work_sync() drains any in-flight
	 *                         work that was already queued, THEN
	 *                         destroys the workqueue;
	 *   power_supply       -> unregisters with state->battery quiesced.
	 *
	 * Registering workqueue_release before power_supply (the previous
	 * order) inverted this — power_supply unregistered with IRQ work
	 * potentially still in flight referencing state->battery (UAF).
	 *
	 * devm_add_action_or_reset() invokes a6_workqueue_release() on its
	 * own failure path, so the workqueue is still cleaned up if the
	 * devres add itself fails.
	 */
	ret = devm_add_action_or_reset(&client->dev, a6_workqueue_release,
				       state);
	if (ret)
		return ret;

	/*
	 * Try to initialise the controller now. Continue on failure: the
	 * IRQ may still arrive later (e.g. on Touchstone connect) and the
	 * bottom-half handler will retry a6_init_state().
	 */
	mutex_lock(&state->dev_mutex);
	ret = a6_init_state(client);
	mutex_unlock(&state->dev_mutex);
	if (ret < 0)
		dev_warn(&client->dev,
			 "A6 not responding (%d); will retry on first IRQ\n",
			 ret);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						a6_irq,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), state);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to request IRQ\n");

		device_init_wakeup(&client->dev, true);
	}

	/*
	 * Spawn the legacy /dev/a6_N + sysfs ABI on the auxiliary bus
	 * if CONFIG_BATTERY_PALM_A6_A2A_COMM is enabled. The aux module
	 * binds to "a6_battery.a2a-comm.N" and re-creates the userspace
	 * surface consumed by webOS tap2shared. Failure here is fatal:
	 * by the time we return from probe the rest of devm is already
	 * registered, and there is no graceful "downgrade" if the aux
	 * registration fails on a system where the kernel was built
	 * with the option enabled.
	 */
	ret = a6_register_aux(state);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to register A2A aux device\n");

	dev_info(&client->dev, "A6 battery controller initialised\n");
	return 0;
}

static void a6_remove(struct i2c_client *client)
{
	device_init_wakeup(&client->dev, false);
	/*
	 * Everything else is devm-managed: the threaded IRQ is freed
	 * first, then a6_workqueue_release() cancels pending IRQ work
	 * and destroys the workqueue, then the power_supply is
	 * unregistered, then the GPIOs and state struct are freed.
	 */
}

/*
 * Suspend / resume PM ops.
 *
 * device_init_wakeup() in probe permanently flags the device
 * wake-capable. During each suspend, if user policy still allows wake
 * (sysfs .../power/wakeup = "enabled"), enable_irq_wake() arms the
 * A6 IRQ as a system wake source so the SoC can drop into deeper sleep
 * states while still being interruptible by A6 events (charge-source
 * change, battery threshold crossings, emergency reset).
 */
static int __maybe_unused a6_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct a6_device_state *state = i2c_get_clientdata(client);

	set_bit(IS_SUSPENDED, state->flags);

	if (client->irq && device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused a6_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct a6_device_state *state = i2c_get_clientdata(client);

	if (client->irq && device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	clear_bit(IS_SUSPENDED, state->flags);
	clear_bit(INT_PENDING, state->flags);

	/*
	 * Unconditionally queue the IRQ work on resume to close the
	 * a6_irq vs. clear_bit(IS_SUSPENDED) race: an IRQ that arrived
	 * after IS_SUSPENDED was already cleared by a6_resume but before
	 * a6_irq sampled it could otherwise leave INT_PENDING latched
	 * with no work queued. The handler is idempotent — it reads
	 * status registers and exits with no work to do if none.
	 */
	queue_work(state->ka6d_workqueue, &state->a6_irq_work);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(a6_pm_ops, a6_suspend, a6_resume);

static const struct of_device_id a6_of_match[] = {
	{ .compatible = "palm,a6-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, a6_of_match);

static const struct i2c_device_id a6_ids[] = {
	{ "a6-battery", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, a6_ids);

static struct i2c_driver a6_driver = {
	.driver = {
		.name = "a6-battery",
		.of_match_table = a6_of_match,
		.pm = pm_sleep_ptr(&a6_pm_ops),
	},
	.id_table = a6_ids,
	.probe = a6_probe,
	.remove = a6_remove,
};
module_i2c_driver(a6_driver);

MODULE_DESCRIPTION("Palm A6 battery controller driver (HP TouchPad)");
MODULE_LICENSE("GPL");
