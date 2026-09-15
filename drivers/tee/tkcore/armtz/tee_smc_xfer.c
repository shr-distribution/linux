/*
 * Copyright (c) 2015-2018 TrustKernel Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>

#include <linux/version.h>

#define TKCORE_BL

#ifdef TKCORE_BL
#include <linux/kthread.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <asm/topology.h>
#endif

#include <linux/tee_clkmgr.h>
#include <linux/tee_core.h>

#include <tee_kernel_lowlevel_api.h>
#include <arm_common/teesmc.h>
#include <arm_common/teesmc_st.h>

#ifdef TKCORE_BL

static int nr_cpus __read_mostly;

/*
 * if a smc_task waits over 100ms
 * for one available core, we give up
 * and choose any core that is available
 * at this moment of time
 */
static const s64 tee_task_timeout_us = 100000LL;

struct tee_task {
	struct smc_param *param;
	struct smc_param *last_param;

	struct work_struct work;
};

#endif

struct tee_task_ctl {
	/* guarantee the mutual-exlusiveness of smc */
	struct mutex g_lock;

	/* cmds that wait for
	 * available TEE thread slots
	 */
	atomic_t nr_waiting_cmds;
	struct completion smc_comp;

#ifdef TKCORE_BL
	/*
	 * records information for the big cpus that are locked
	 * in the online state due to running smc commands
	 */
	int *tasks;
	spinlock_t task_lock;

	enum cpuhp_state tkcore_cpuhp_state;
	struct workqueue_struct *wq;
#endif

	/* statistics information */
	s64 max_smc_time;
	s64 max_task_time;
};

static struct tee_task_ctl tee_task_ctl;

static inline void trace_tee_smc(struct tee_task_ctl *ctl, int rv,
				 s64 time_start, s64 time_end)
{
	s64 duration = time_end - time_start;

	if (duration > 1000000LL) {
		pr_warn("WARNING SMC[0x%x] %sDURATION %lld us\n", rv,
			rv == TEESMC_RPC_FUNC_IRQ ? "IRQ " : "", duration);
	}

	/* we needn't handle concurrency here. */
	if (duration > ctl->max_smc_time)
		ctl->max_smc_time = duration;
}

static inline void trace_tee_smc_done(struct tee_task_ctl *ctl,
				s64 time_start,
				s64 time_end)
{
	s64 duration = time_end - time_start;

	if (duration > ctl->max_task_time)
		ctl->max_task_time = duration;
}

/* return 0 for nonpreempt rpc, 1 for others */
static int handle_nonpreempt_rpc(struct smc_param *p)
{
	uint32_t func_id = TEESMC_RETURN_GET_RPC_FUNC(p->a0);

#if IS_ENABLED(CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT)
	/* for compatibility with legacy tee-os which
	 * does not support clkmgr
	 */
	if (func_id == T6SMC_RPC_CLKMGR_LEGACY_CMD) {
		p->a1 = tee_clkmgr_handle(p->a1, p->a2);
		return 0;
	}
#endif

	if ((func_id & 0xff) != T6SMC_RPC_NONPREEMPT_CMD)
		return 1;

	switch (T6SMC_RPC_NONPREEMPT_GET_FUNC(p->a0)) {
#if IS_ENABLED(CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT)
	case T6SMC_RPC_CLKMGR_CMD:
		/* compatible with old interface */
		p->a1 = tee_clkmgr_handle(p->a1,
			(p->a1 & TEE_CLKMGR_TOKEN_NOT_LEGACY) ?
				p->a2 : (p->a2 | TEE_CLKMGR_OP_ENABLE));
		break;
#endif
	default:
		pr_err("Unknown non-preempt rpc cmd: 0x%llx\n",
			(unsigned long long) p->a0);
	}

	return 0;
}

static inline unsigned long rpc_ret_by_cmd_id(unsigned long cmd_id)
{
	return cmd_id == TEESMC32_FASTCALL_WITH_ARG ?
		 TEESMC32_FASTCALL_RETURN_FROM_RPC : TEESMC32_CALL_RETURN_FROM_RPC;
}

static void backup_call(struct smc_param *call, struct smc_param *last_call)
{
	last_call->a0 = call->a0;
	last_call->a1 = call->a1;
}

static void do_smc(struct tee_task_ctl *ctl,
				struct smc_param *call, struct smc_param *last_call)
{
	unsigned long a0 = rpc_ret_by_cmd_id(call->a0);
	s64 task_start, task_end, call_start, call_end;

	mutex_lock(&ctl->g_lock);

	task_start = ktime_to_us(ktime_get());

	for (;;) {
		/* backup a0 and a1 in case TEESMC_RETURN_EAFFINITY */
		backup_call(call, last_call);

		call_start = ktime_to_us(ktime_get());
		tee_smc_call(call);
		call_end = ktime_to_us(ktime_get());

		trace_tee_smc(ctl, TEESMC_RETURN_GET_RPC_FUNC(call->a0),
			call_start, call_end);

		if (!TEESMC_RETURN_IS_RPC(call->a0))
			goto out;

		if (handle_nonpreempt_rpc(call)) {
			if (TEESMC_RETURN_GET_RPC_FUNC(call->a0) != TEESMC_RPC_FUNC_IRQ)
				goto out;
		}

		call->a0 = a0;
	}

	task_end = ktime_to_us(ktime_get());
	trace_tee_smc_done(ctl, task_start, task_end);

out:
	mutex_unlock(&ctl->g_lock);
}

#ifdef TKCORE_BL

/* callers shall hold cpu->task_lock */
static inline void lock_cpu(struct tee_task_ctl *ctl, int cpu)
{
	++ctl->tasks[cpu];
	smp_mb();
}

/* callers shall hold cpu->task_lock */
static inline void unlock_cpu(struct tee_task_ctl *ctl, int cpu)
{
	--ctl->tasks[cpu];
	WARN_ON(ctl->tasks[cpu] < 0);
	smp_mb();
}

/*
 * can only be called in cpu hp callback or w/ ctl->task_lock
 *
 * It's OK to call cpu_locked from cpuhp callback w/o
 * ctl->task_lock, because we already grab cpus_read_lock()
 * before lock_cpu()
 */
static inline bool cpu_locked(struct tee_task_ctl *ctl, int cpu)
{
	bool locked = !!ctl->tasks[cpu];
	smp_mb();
	return locked;
}

static int tee_cpu_prepare_down(unsigned int cpu)
{
	struct tee_task_ctl *ctl = &tee_task_ctl;

	if (unlikely(cpu >= nr_cpus)) {
		pr_err("Bad cpu: %d\n", cpu);
		return -1;
	}

	return cpu_locked(ctl, cpu) ? -1 : 0;
}

static void do_run_tee_task(struct work_struct *w)
{
	struct tee_task *task =
		container_of(w, struct tee_task, work);

	do_smc(&tee_task_ctl, task->param, task->last_param);
}

static int find_matched_cpu(struct tee_task_ctl *ctl,
				const struct tee_task_cpu_affinity *aff)
{
	int cpu;

	for_each_online_cpu(cpu) {
		if (aff->ops->match_cpu(aff->priv, cpu))
			return cpu;
	}

	return -1;
}

static void init_tee_task(struct tee_task *task,
				struct smc_param *param, struct smc_param *last_param)
{
	// initialize call to tee
	task->param = param;
	task->last_param = last_param;

	INIT_WORK(&task->work, do_run_tee_task);
}

static int post_tee_task(struct tee_task_ctl *ctl,
			const struct tee_task_cpu_affinity *aff,
			struct smc_param *call,
			struct smc_param *last_call)
{
	int r = 0, picked_cpu = -1;

	s64 tee_task_start, tee_task_current;

	struct tee_task task;

	init_tee_task(&task, call, last_call);
	tee_task_start = ktime_to_us(ktime_get());

	while (picked_cpu < 0) {
		int candidate_cpu;

		get_online_cpus();
		picked_cpu = find_matched_cpu(ctl, aff);

		spin_lock(&ctl->task_lock);
		if (picked_cpu >= 0)
			lock_cpu(ctl, picked_cpu);
		spin_unlock(&ctl->task_lock);
		put_online_cpus();

		if (picked_cpu >= 0) {
			if (WARN_ON(!queue_work_on(picked_cpu, ctl->wq, &task.work))) {
				r = -1;
				goto out;
			}
			r = 0;
		} else {
			tee_task_current = ktime_to_us(ktime_get());
			if (tee_task_current - tee_task_start >= tee_task_timeout_us) {
				pr_warn("submit tee task to target cpu timeout\n");
				break;
			}

			candidate_cpu = aff->ops->get_candidate_cpu(aff->priv);
			r = add_cpu(candidate_cpu);
			if (r != 0) {
				pr_warn("cpu_on(%d) failed with ret: %d\n",
					candidate_cpu, r);
			}
		}
	}

	if (picked_cpu < 0)
		return -1;

	flush_work(&task.work);

out:
	spin_lock(&ctl->task_lock);
	unlock_cpu(ctl, picked_cpu);
	spin_unlock(&ctl->task_lock);
	return r;
}

static bool tee_bind_cpu_request_need_bind_cpu(void *priv)
{
	(void) priv;
	return true;
}

static bool tee_bind_cpu_request_match_cpu(void *priv, int cpu)
{
	uint32_t cpumask = ((uint32_t) (unsigned long) priv);

	return !!(cpumask & (1U << cpu));
}

static int tee_bind_cpu_request_get_candidate_cpu(void *priv)
{
	uint32_t cpumask = ((uint32_t) (unsigned long) priv);

	if (cpumask == 0)
		return -1;

	return __builtin_ffs(cpumask) - 1;
}

const struct tee_task_cpu_affinity_operations tee_bind_cpu_request_operations = {
	.need_bind_cpu = tee_bind_cpu_request_need_bind_cpu,
	.match_cpu = tee_bind_cpu_request_match_cpu,
	.get_candidate_cpu = tee_bind_cpu_request_get_candidate_cpu,
};

static void update_tee_task_affinity(struct tee_task_cpu_affinity *aff, uint32_t cpu_mask)
{
	aff->ops = &tee_bind_cpu_request_operations;
	aff->priv = (void *) ((unsigned long) cpu_mask);
}

static void restore_call(struct smc_param *call, struct smc_param *last_call)
{
	call->a0 = last_call->a0;
	call->a1 = last_call->a1;
}

static void submit_tee_task(struct tee_task_ctl *ctl,
			struct tee_task_cpu_affinity *aff,
			struct smc_param *call)
{
	int r;
	struct smc_param last_call;
	unsigned long cmd_id = call->a0;

	for (;;) {
		r = aff->ops->need_bind_cpu(aff->priv) ?
			post_tee_task(ctl, aff, call, &last_call) : -1;
		if (r < 0)
			do_smc(ctl, call, &last_call);

		if (call->a0 == TEESMC_RETURN_TKCORE_RPC_BIND_CPU) {
			update_tee_task_affinity(aff, (uint32_t) call->a1);
			call->a0 = rpc_ret_by_cmd_id(cmd_id);
			call->a1 = 0;
		} else if (call->a0 == TEESMC_RETURN_EAFFINITY) {
			update_tee_task_affinity(aff, (uint32_t) call->a1);
			restore_call(call, &last_call);
		} else {
			break;
		}
	}
}

static int platform_bl_init(struct tee_task_ctl *ctl)
{
	int r;

	nr_cpus = num_possible_cpus();
	if (nr_cpus > NR_CPUS) {
		pr_err("nr_cpus %d exceeds NR_CPUS %d\n", nr_cpus, NR_CPUS);
		return -1;
	}

	ctl->tasks = (int *) kzalloc(nr_cpus * sizeof(int), GFP_KERNEL);
	if (ctl->tasks == NULL) {
		return -ENOMEM;
	}

	spin_lock_init(&ctl->task_lock);

	ctl->wq = alloc_workqueue("tee_work", WQ_CPU_INTENSIVE, 0);
	if (ctl->wq == NULL) {
		pr_err("bad alloc tee_work wq\n");
		r = -ENOMEM;
		goto err;
	}

	r = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
		"tee/tkcore_tzdrv:cpu_listener",
		NULL, tee_cpu_prepare_down);
	if (r < 0)
		goto err;

	ctl->tkcore_cpuhp_state = r;
	return 0;

err:
	if (ctl->tasks) {
		kfree(ctl->tasks);
		ctl->tasks = NULL;
	}

	if (ctl->wq) {
		destroy_workqueue(ctl->wq);
		ctl->wq = NULL;
	}

	return r;
}

static void platform_bl_deinit(struct tee_task_ctl *ctl)
{
	destroy_workqueue(ctl->wq);
	ctl->wq = NULL;

	kfree(ctl->tasks);
	ctl->tasks = NULL;

	cpuhp_remove_state(ctl->tkcore_cpuhp_state);
}

#else

static inline void submit_tee_task(struct tee_task_ctl *ctl,
			const struct tee_task_cpu_affinity *aff,
			struct smc_param *p)
{
	struct smc_param last_call;

	(void) aff;
	do_smc(ctl, p, &last_call);
}

static int platform_bl_init(struct tee_task_ctl *ctl) { return 0; }

static void platform_bl_deinit(struct tee_task_ctl *ctl) { }

#endif

static void waiters_enqueue(struct tee_task_ctl *ctl)
{
	/*TODO handle too long time of waiting */
	atomic_inc(&ctl->nr_waiting_cmds);
	wait_for_completion(&ctl->smc_comp);
}

static void waiters_dequeue(struct tee_task_ctl *ctl)
{
	if (atomic_dec_if_positive(&ctl->nr_waiting_cmds) >= 0)
		complete(&ctl->smc_comp);
}

void run_tee_task(struct smc_param *p, struct tee_task_cpu_affinity *aff)
{
	/* NOTE!!! we remove the e_lock_teez(ptee) here !!!! */
#ifdef ARM64
	uint64_t orig_a0 = p->a0;
#else
	uint32_t orig_a0 = p->a0;
#endif
	for (;;) {
		submit_tee_task(&tee_task_ctl, aff, p);
		if (p->a0 == TEESMC_RETURN_ETHREAD_LIMIT) {
			waiters_enqueue(&tee_task_ctl);
			p->a0 = orig_a0;
		} else {
			if (!TEESMC_RETURN_IS_RPC(p->a0))
				waiters_dequeue(&tee_task_ctl);
			break;
		}
	}
}

void run_tee_task_nowait(struct smc_param *p, struct tee_task_cpu_affinity *aff)
{
	submit_tee_task(&tee_task_ctl, aff, p);
}

int tee_init_task(void)
{
	struct tee_task_ctl *ctl = &tee_task_ctl;

	mutex_init(&ctl->g_lock);

	atomic_set(&ctl->nr_waiting_cmds, 0);
	init_completion(&ctl->smc_comp);

	ctl->max_smc_time = 0LL;
	ctl->max_task_time = 0LL;

	return platform_bl_init(ctl);
}

void tee_exit_task(void)
{
	struct tee_task_ctl *ctl = &tee_task_ctl;

	platform_bl_deinit(ctl);
}
