// SPDX-License-Identifier: GPL-2.0-only
/*
 * Palm A6 Battery Controller Driver - I2C dispatcher + action item queue
 *
 * Copyright (C) 2008-2011 Palm, Inc.
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Modernized for device tree and modern kernel APIs
 *
 * Houses the asynchronous I2C action-item queue (A6_PQ), the dispatcher
 * kthread that drains it, the low-level i2c register read/write helpers
 * (__a6_i2c_read_reg / __a6_i2c_write_reg + their retry wrappers), and the
 * optional debugfs interface that exercises start/stop of the dispatch task.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "a6_internal.h"

#ifdef A6_PQ
/* a6 debugfs interface... */
#ifdef A6_DEBUG


static int32_t a6_restart_aid_thread_fn(void *param)
{
	struct a6_device_state *state = param;
	int32_t rc = 0;

	while (!kthread_should_stop() && !state->dbgflg_kill_raid) {
		// stop aid task
		rc = a6_stop_ai_dispatch_task(state);
		if (rc) {
			pr_err("%s: failed to stop ai_dispatch_task.\n", __func__);
			break;
		}

		// re-start aid task
		rc = a6_start_ai_dispatch_task(state);
		if (rc) {
			pr_err("%s: failed to start ai_dispatch_task.\n", __func__);
			break;
		}

		// sleep
		msleep(500);
	}

	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock_interruptible interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	state->debug_restart_aid = 0;
	state->dbgflg_kill_raid = 0;
	mutex_unlock(&state->dev_mutex);

	return rc;
}

static int a6_test_restart_aid_set(void *data, u64 val)
{
	int32_t rc = 0;
	struct a6_device_state *state = data;
	uint8_t in_val, curr_val;

	in_val = val ? 1 : val;
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock_interruptible interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	curr_val = state->debug_restart_aid;

	// are we actually changing state?
	if (in_val ^ curr_val) {
		if (in_val) {
			// prev task still being killed or active: fail
			if (state->dbgflg_kill_raid || state->debug_restart_aid) {
				pr_err("%s: prev task still being killed or active: fail\n",
				       __func__);
				goto err0;
			}
			// create ai dispatcher task...
			state->restart_aid_task = kthread_run(a6_restart_aid_thread_fn, state,
							      "dbg_aidrst_%s", state->plat_data->dev_name);
			ASSERT(!IS_ERR(state->restart_aid_task));
			state->debug_restart_aid = in_val;
		} else {
			// prev task still being killed or not active: fail
			if (state->dbgflg_kill_raid || !state->debug_restart_aid) {
				pr_err("%s: prev task still being killed or not active: fail\n",
				       __func__);
				goto err0;
			}
			// intent to kill task; task resets state as part of teardown
			state->dbgflg_kill_raid = 1;
		}
	}
	mutex_unlock(&state->dev_mutex);


err0:
	return rc;
}
static int a6_test_restart_aid_get(void *data, u64 *val)
{
	struct a6_device_state *state = data;

	*val = state->debug_restart_aid;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_a6_test_restart_aid, a6_test_restart_aid_get, a6_test_restart_aid_set, "%llu\n");

int32_t a6_create_debug_interface(struct a6_device_state *state)
{
	int32_t rc = 0;
	struct dentry *dentry_parent, *dentry_child;
	char *name;

	/*
	 * The HP TouchPad carries two A6 controllers on i2c-2 (0x31 and
	 * 0x32, one per battery cell), so a bare "a6" debugfs directory
	 * name collides between the two probe calls and the second
	 * instance fails with:
	 *   debugfs: 'a6' already exists in '/'
	 *   a6-battery 2-0032: Failed to create debug interface
	 * Prefix with the i2c device name to keep each instance unique
	 * (e.g. "a6-2-0031", "a6-2-0032").
	 */
	name = kasprintf(GFP_KERNEL, "a6-%s", dev_name(&state->i2c_dev->dev));
	if (!name)
		return -ENOMEM;

	dentry_parent = debugfs_create_dir(name, NULL);
	kfree(name);
	if (IS_ERR(dentry_parent)) {
		rc = PTR_ERR(dentry_parent);
		goto err0;
	}

	dentry_child = debugfs_create_file("periodic_restart_aid", 0644,
					   dentry_parent, state, &fops_a6_test_restart_aid);
	if (IS_ERR(dentry_child)) {
		rc = PTR_ERR(dentry_child);
		goto err0;
	}

	return 0;

err0:
	debugfs_remove_recursive(dentry_parent);
	return rc;
}
#endif
#endif // A6_PQ
#ifdef A6_PQ
// a6 action item types
enum {
	AI_I2C_TYPE,
};

struct a6_action_item {
	void *ai_payload;
	uint32_t ai_type: 4;
	uint32_t ai_complete:1;
	int32_t (*ai_do_action)(void *payload_param);
	int32_t* (*ai_ret_code)(void *payload_param);
	wait_queue_head_t ai_waitq;
	struct list_head list;
};

/* enq_a6_action_item: q's action item and blocks till action completion */
static int32_t enq_a6_action_item(struct a6_device_state *state, struct a6_action_item *ai, bool enq_tail)
{
	int32_t rc = 0;

	mutex_lock(&state->aq_mutex);
	if (enq_tail)
		list_add_tail(&ai->list, &state->aq_head);
	else
		list_add(&ai->list, &state->aq_head);
	mutex_unlock(&state->aq_mutex);

	// signal the action_item dispatcher thread
	complete(&state->aq_enq_complete);

	// ** wait for action_item to be processed
	a6_wait_event_ex(ai->ai_waitq, ai->ai_complete);
	rc = *ai->ai_ret_code(ai->ai_payload);

	return rc;
}

/* dq_a6_action_item: dq's head item from action item list */
static struct a6_action_item *dq_a6_action_item(struct a6_device_state *state)
{
	struct a6_action_item *ai = NULL;

	mutex_lock(&state->aq_mutex);
	if (!list_empty(&state->aq_head)) {
		ai = list_first_entry(&state->aq_head, struct a6_action_item, list);
		list_del(state->aq_head.next);
	}
	mutex_unlock(&state->aq_mutex);

	return ai;
}

/*
 * flush_a6_action_items: iterates action item list, dq's each item, marks it
 * cancelled and complete and then wakes up the requestor.
 */
int32_t flush_a6_action_items(struct a6_device_state *state)
{
	int32_t rc = 0;
	struct a6_action_item *ai = NULL;

	// lock list until all items removed
	mutex_lock(&state->aq_mutex);
	while (!list_empty(&state->aq_head)) {
		// get first entry...
		ai = list_first_entry(&state->aq_head, struct a6_action_item, list);
		// ... and remove from list
		list_del(state->aq_head.next);
		*ai->ai_ret_code(ai->ai_payload) = -ECANCELED;
		// set completion indicator
		ai->ai_complete = 1;
		// signal ai requestor
		wake_up(&ai->ai_waitq);
	}
	mutex_unlock(&state->aq_mutex);

	return rc;
}


struct ai_i2c_payload {
	struct i2c_client *client;
	struct i2c_msg *msg;
	uint32_t num_msgs;
	int32_t rc;
};

/* do_i2c_action_item: i2c-specific action implementation */
static int32_t do_i2c_action_item(void *payload_param)
{
	struct ai_i2c_payload *trf_data = (struct ai_i2c_payload *)payload_param;

	trf_data->rc = i2c_transfer(trf_data->client->adapter, trf_data->msg, trf_data->num_msgs);
	udelay(700);
	if (trf_data->rc < 0) {
		pr_err_ratelimited("%s: err code: %d\n", __func__, trf_data->rc);
		goto err0;
	}

	trf_data->rc = 0;
err0:
	return trf_data->rc;
}

/* i2c_ret_code: i2c-specific ret-code reference retrieval */
static int32_t *i2c_ret_code(void *payload_param)
{
	struct ai_i2c_payload *trf_data = (struct ai_i2c_payload *)payload_param;

	return &trf_data->rc;
}

/* q_a6_i2c_action_item: creates the action item structure and enq's it */
static int32_t q_a6_i2c_action_item(struct i2c_client *client, struct i2c_msg *msg, uint32_t num_msgs)
{
	int32_t ret = 0;
	struct ai_i2c_payload data = {
		.client = client,
		.msg = msg,
		.num_msgs = num_msgs,
		.rc = 0
	};

	struct a6_action_item a6_i2c_ai = {
		.ai_payload = &data,
		.ai_type = AI_I2C_TYPE,
		.ai_complete = 0,
		.ai_do_action = do_i2c_action_item,
		.ai_ret_code = i2c_ret_code,
		.list = LIST_HEAD_INIT(a6_i2c_ai.list)
	};

	init_waitqueue_head(&a6_i2c_ai.ai_waitq);
	ret = enq_a6_action_item(i2c_get_clientdata(client), &a6_i2c_ai, true/*enq tail*/);

	return ret;
}

/* ai_dispatch_thread_fn: function that handles ai processing in the aid task */
static int ai_dispatch_thread_fn(void *param)
{
	int32_t rc = 0;
	struct a6_action_item *ai;
	struct a6_device_state *state = param;

	do {
		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: about to wait for ai enq complete.\n", __func__);
		// wait for ai to be enq'd
		rc = wait_for_completion_interruptible(&state->aq_enq_complete);
		if (rc >= 0) {
			// check kill status: intent to kill?
			if (kthread_should_stop() || test_bit(KILLING_AID_TASK, state->flags)) {
				// remove all items from q
				flush_a6_action_items(state);
				// and signal killer before exiting stage...
				complete(&state->aid_exit_complete);
				break;
			}

			ai = dq_a6_action_item(state);
			if (ai) {
				int32_t ret_val;

				A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: got ai.\n", __func__);
				// invoke ai-specific action
				ret_val = ai->ai_do_action(ai->ai_payload);
				if (ret_val)
					pr_err("%s: ai_do_action failed.\n", __func__);

				// set completion indicator
				ai->ai_complete = 1;
				// signal ai requestor
				wake_up(&ai->ai_waitq);
			} else {
				A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: no ai.\n", __func__);
			}
		}
		/*
		 * wait interrupted by -ERESTARTSYS: let the do-while
		 * condition (rc >= 0) take us out of the loop cleanly.
		 */
		else {
			pr_err("%s: wait for action_item enq interrupted.\n",
			       __func__);
			WARN_ON_ONCE(rc < 0);
		}
	} while (rc >= 0);

	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "*** %s: exiting.***\n", __func__);

	return rc;
}
#endif  // A6_PQ


static int32_t __a6_i2c_read_reg(struct i2c_client *client, const uint16_t *ids, uint32_t num_ids, uint8_t *out)
{
	int32_t ret = 0, i;
	uint16_t swp_addr[id_size];
	struct i2c_msg msg[id_size * 2], *msg_itr;
	struct a6_device_state *state = i2c_get_clientdata(client);
#ifdef A6_I2C_PROFILE
	ktime_t start, end;
#endif

#ifdef A6_PQ
	if (test_bit(IS_QUIESCED, ((struct a6_device_state *)i2c_get_clientdata(client))->flags)) {
		ret = -ECANCELED;
		goto err0;
	}
#endif // A6_PQ

	// force A6 wakeup...
	if (start_last_a6_activity) {
		long diff_time = (long)jiffies - (long)start_last_a6_activity;

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: time since last activity: %ld ms\n",
		       __func__, diff_time * 1000/HZ);
	}
	start_last_a6_activity = jiffies;

	// prevent timer expiry during force_wake related action...
	timer_delete(&state->a6_force_wake_timer);

	mutex_lock(&state->a6_force_wake_mutex);
	// a6 external wake enabled?
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		if (!test_bit(FORCE_WAKE_ACTIVE_BIT, state->flags)) {
			struct a6_wake_ops *wake_ops = (struct a6_wake_ops *)state->plat_data->wake_ops;

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
				"%s: disabling periodic_wake and switching to force_wake\n",
				__func__);

			/* periodic wake active: disable it and switch to force wake */
			if (wake_ops->disable_periodic_wake)
				wake_ops->disable_periodic_wake(wake_ops->data);
			if (wake_ops->force_wake)
				wake_ops->force_wake(wake_ops->data);

			set_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
	}
	mutex_unlock(&state->a6_force_wake_mutex);

#ifdef A6_I2C_PROFILE
	start = ktime_get();
#endif
	msg_itr = &msg[0];
	for (i = num_ids-1; i >= 0; i--) {
		msg_itr->addr = client->addr;
		msg_itr->flags = 0;
		msg_itr->len = sizeof(uint16_t);
		swp_addr[i] = ids[i] >> 8 | ids[i] << 8;
		msg_itr->buf = (uint8_t *)&swp_addr[i];

		(msg_itr+1)->addr = client->addr;
		(msg_itr+1)->flags = I2C_M_RD;
		(msg_itr+1)->len = sizeof(uint8_t);
		(msg_itr+1)->buf = &out[i];
#ifdef CONFIG_BATTERY_PALM_A6_I2C_SINGLE_BYTE
#ifdef A6_PQ
		ret = q_a6_i2c_action_item(client, msg, 2);
#else
		ret = i2c_transfer(client->adapter, msg, 2);
#endif // A6_PQ
#endif
		msg_itr += 2;
	}


#ifndef CONFIG_BATTERY_PALM_A6_I2C_SINGLE_BYTE
#ifdef A6_PQ
	ret = q_a6_i2c_action_item(client, msg, num_ids*2);
#else
	ret = i2c_transfer(client->adapter, msg, num_ids*2);
#endif // A6_PQ
#endif
	//msleep(1);
#ifdef A6_I2C_PROFILE
	{
		end = ktime_get();
		pr_err("%s[0x%02x]: elpased time: %lld\n",
		       __func__, client->addr, ktime_to_ns(ktime_sub(end, start)));
	}
#endif
	if (ret < 0) {
		pr_err_ratelimited("%s[0x%02x]: err code: %d\n", __func__, client->addr, ret);
		// reset the force_wake timer inedependent of i2c failure
		//goto err0;
	}
	A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "ret val: 0x%x\n", *out);

	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		// [re]start force-wake expiry timer
		mod_timer(&state->a6_force_wake_timer, jiffies+FORCE_WAKE_TIMER_EXPIRY);
	}

err0:
	return ret;
}

int32_t a6_i2c_read_reg(struct i2c_client *client, const uint16_t *ids, uint32_t num_ids, uint8_t *out)
{
	int32_t ret;
#ifdef A6_I2C_RETRY
	int32_t retry = 5;
#else
	int32_t retry = 0;
#endif

	do {
		ret = __a6_i2c_read_reg(client, ids, num_ids, out);
		if (ret < 0) {
			pr_err_ratelimited("%s: a6 i2c transaction failed. %s...\n", __func__, retry ? "retry" : " ");
			msleep(30);
		}
	} while (ret != 0 && retry-- > 0);

	return ret;
}

static int32_t __a6_i2c_write_reg(struct i2c_client *client, const uint16_t *ids, uint32_t num_ids, const uint8_t *in)
{
	int32_t ret = 0, i;
	uint8_t i2c_buf[(2 + 1) * id_size];
	struct i2c_msg msg[id_size], *msg_itr;
	struct a6_device_state *state = i2c_get_clientdata(client);

#ifdef A6_PQ
	if (test_bit(IS_QUIESCED, ((struct a6_device_state *)i2c_get_clientdata(client))->flags)) {
		ret = -ECANCELED;
		goto err0;
	}
#endif // A6_PQ

	// force A6 wakeup...
	if (start_last_a6_activity) {
		long diff_time = (long)jiffies - (long)start_last_a6_activity;

		A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR, "%s: time since last activity: %ld ms\n",
		       __func__, diff_time * 1000/HZ);
	}
	start_last_a6_activity = jiffies;

	// prevent timer expiry during force_wake related action...
	timer_delete(&state->a6_force_wake_timer);

	mutex_lock(&state->a6_force_wake_mutex);
	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		if (!test_and_set_bit(FORCE_WAKE_ACTIVE_BIT, state->flags)) {
			struct a6_wake_ops *wake_ops = (struct a6_wake_ops *)state->plat_data->wake_ops;

			A6_DPRINTK(A6_DEBUG_VERBOSE, KERN_ERR,
				"%s: disabling periodic_wake and switching to force_wake\n",
				__func__);

			/* periodic wake active: disable it and switch to force wake */
			if (wake_ops->disable_periodic_wake)
				wake_ops->disable_periodic_wake(wake_ops->data);
			if (wake_ops->force_wake)
				wake_ops->force_wake(wake_ops->data);

			set_bit(FORCE_WAKE_ACTIVE_BIT, state->flags);
		}
	}
	mutex_unlock(&state->a6_force_wake_mutex);

	msg_itr = &msg[0];
	for (i = num_ids-1; i >= 0; i--) {
		i2c_buf[(i*(2+1))+0] = (uint8_t)(ids[i] >> 8);
		i2c_buf[(i*(2+1))+1] = (uint8_t)(ids[i]);
		i2c_buf[(i*(2+1))+2] = in[i];

		msg_itr->addr = client->addr;
		msg_itr->flags = 0;
		msg_itr->len = 2+1;
		msg_itr->buf = &i2c_buf[i*(2+1)];
#ifdef CONFIG_BATTERY_PALM_A6_I2C_SINGLE_BYTE_WRITE
#ifdef A6_PQ
		ret = q_a6_i2c_action_item(client, msg_itr, 1);
#else
		ret = i2c_transfer(client->adapter, msg_itr, 1);
#endif // A6_PQ
		udelay(700);
		if (ret < 0)
			break;
#endif
		msg_itr++;
	}

#ifndef CONFIG_BATTERY_PALM_A6_I2C_SINGLE_BYTE_WRITE
#ifdef A6_PQ
	ret = q_a6_i2c_action_item(client, msg, num_ids);
#else
	ret = i2c_transfer(client->adapter, msg, num_ids);
#endif // A6_PQ
	//msleep(1);
#endif
	if (ret < 0) {
		pr_err_ratelimited("%s[0x%02x]: err code: %d\n", __func__, client->addr, ret);
		// reset the force_wake timer inedependent of i2c failure
		//goto err0;
	}

	if (test_bit(CAP_PERIODIC_WAKE, state->flags)) {
		// [re]start force-wake expiry timer
		mod_timer(&state->a6_force_wake_timer, jiffies+FORCE_WAKE_TIMER_EXPIRY);
	}

err0:
	return ret;
}

int32_t a6_i2c_write_reg(struct i2c_client *client, const uint16_t *ids, uint32_t num_ids, const uint8_t *in)
{
	int32_t ret;
#ifdef A6_I2C_RETRY
	int32_t retry = 5;
#else
	int32_t retry = 0;
#endif

	do {
		ret = __a6_i2c_write_reg(client, ids, num_ids, in);
		if (ret < 0) {
			pr_err_ratelimited("%s: a6 i2c transaction failed. %s...\n", __func__, retry ? "retry" : " ");
			msleep(30);
		}
	} while (ret != 0 && retry-- > 0);

	return ret;
}

#ifdef A6_PQ
int32_t a6_stop_ai_dispatch_task(struct a6_device_state *state)
{
	int32_t rc = 0;

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	// stopping during a start? fail
	if (test_bit(STARTING_AID_TASK, state->flags)) {
		pr_err("%s: aid task not fully started. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// stopping during a stop? fail
	if (test_bit(KILLING_AID_TASK, state->flags)) {
		pr_err("%s: aid task being stopped. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// task never started? fail
	if (!state->ai_dispatch_task) {
		pr_err("%s: no aid task. failing op.\n", __func__);
		rc = -EPERM;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// transition to quiesced state: stops accepting new requests
	set_bit(IS_QUIESCED, state->flags);
	// declare intent to kill ai dispatch task
	set_bit(KILLING_AID_TASK, state->flags);
	// exit critsec
	mutex_unlock(&state->dev_mutex);

	// kick task if asleep
	complete(&state->aq_enq_complete);

	// wait for ai dispatch task exit
	rc = wait_for_completion_interruptible(&state->aid_exit_complete);
	if (rc < 0) {
		pr_err("%s: wait for ai dispatch task exit interrupted.\n",
		       __func__);
	}

	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}
	// ai dispatch task exited (assume it did if we get interrupted): reset state
	clear_bit(KILLING_AID_TASK, state->flags);
	state->ai_dispatch_task = NULL;
	// exit critsec
	mutex_unlock(&state->dev_mutex);


err0:
	return rc;
}

int32_t a6_start_ai_dispatch_task(struct a6_device_state *state)
{
	int32_t rc = 0;


	// critsec for manipulating flags
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted\n", __func__);
		return -ERESTARTSYS;
	}
	// starting during a start? fail
	if (test_bit(STARTING_AID_TASK, state->flags)) {
		pr_err("%s: aid task not fully started. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// starting during a stop? fail
	if (test_bit(KILLING_AID_TASK, state->flags)) {
		pr_err("%s: aid task being stopped. failing op.\n", __func__);
		rc = -EBUSY;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// task never stopped? fail
	if (state->ai_dispatch_task) {
		pr_err("%s: aid task exists. failing op.\n", __func__);
		rc = -EPERM;
		mutex_unlock(&state->dev_mutex);
		goto err0;
	}

	// declare intent to start ai dispatch task
	set_bit(STARTING_AID_TASK, state->flags);
	// exit critsec
	mutex_unlock(&state->dev_mutex);


	// create ai dispatcher task...
	state->ai_dispatch_task = kthread_run(ai_dispatch_thread_fn, state,
					      "aid_%s", state->plat_data->dev_name);
	if (IS_ERR(state->ai_dispatch_task)) {
		pr_err("%s: failed to create ai dispatcher task.\n", __func__);
		rc = PTR_ERR(state->ai_dispatch_task);
		state->ai_dispatch_task = NULL;
		goto err0;
	}

	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		pr_err("%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}
	/* kthread_run() already returned the task_struct pointer, no need to look it up */
	ASSERT(state->ai_dispatch_task);

	// transition to active state: start accepting new requests
	clear_bit(IS_QUIESCED, state->flags);
	// ai dispatch task started: reset state
	clear_bit(STARTING_AID_TASK, state->flags);
	// exit critsec
	mutex_unlock(&state->dev_mutex);

err0:
	return rc;
}
#endif // A6_PQ
