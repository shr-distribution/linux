// SPDX-License-Identifier: GPL-2.0
/*
 * A2XX GPU Debug Module
 *
 * Provides debugging infrastructure for tracking down graphics rendering
 * issues on Adreno 200/220/225 GPUs. Specifically designed to help debug
 * memory coherency and timing issues that may correlate with system activity.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/ktime.h>

#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>

#include "a2xx_gpu.h"
#include "a2xx_debug.h"
#include "msm_gpu.h"

/*
 * Debug parameters - can be changed at runtime via /sys/module/msm/parameters/
 */

/* Enable verbose per-submit logging */
static bool a2xx_debug_submit;
module_param(a2xx_debug_submit, bool, 0644);
MODULE_PARM_DESC(a2xx_debug_submit, "Log every GPU submit with timing info");

/* Enable extra cache flush before every submit */
static bool a2xx_debug_flush_cache = true;
module_param(a2xx_debug_flush_cache, bool, 0644);
MODULE_PARM_DESC(a2xx_debug_flush_cache, "Force outer cache flush before GPU submit");

/* Log RBBM status on every submit completion */
static bool a2xx_debug_rbbm_status;
module_param(a2xx_debug_rbbm_status, bool, 0644);
MODULE_PARM_DESC(a2xx_debug_rbbm_status, "Log RBBM status on submit completion");

/* Delay in microseconds between ringbuffer write and WPTR update */
static int a2xx_debug_wptr_delay;
module_param(a2xx_debug_wptr_delay, int, 0644);
MODULE_PARM_DESC(a2xx_debug_wptr_delay, "Microseconds to delay before WPTR write");

/* Count submits for correlation */
static atomic_t submit_counter = ATOMIC_INIT(0);

/* Capture SQ state on first few submits */
static bool a2xx_debug_first_submit_sq = true;
module_param(a2xx_debug_first_submit_sq, bool, 0644);
MODULE_PARM_DESC(a2xx_debug_first_submit_sq, "Log SQ state on first 3 submits");

/*
 * Log SQ state - critical for debugging interpolation issues
 */
static void log_sq_state(struct msm_gpu *gpu, int submit_num)
{
	u32 gpr_mgmt, interp, program, inst_store, context;

	gpr_mgmt = gpu_read(gpu, REG_A2XX_SQ_GPR_MANAGEMENT);
	interp = gpu_read(gpu, REG_A2XX_SQ_INTERPOLATOR_CNTL);
	program = gpu_read(gpu, REG_A2XX_SQ_PROGRAM_CNTL);
	inst_store = gpu_read(gpu, REG_A2XX_SQ_INST_STORE_MANAGMENT);
	context = gpu_read(gpu, REG_A2XX_SQ_CONTEXT_MISC);

	dev_info(gpu->dev->dev,
		"[A2XX-SQ] submit#%d: GPR_MGMT=%08x INTERP=%08x PROG=%08x INST=%08x CTX=%08x\n",
		submit_num, gpr_mgmt, interp, program, inst_store, context);
}

/*
 * Log GPU state at submit time - helps correlate issues with specific submits
 */
void a2xx_debug_log_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit)
{
	u32 rbbm_status, cp_rptr, cp_wptr;
	int seq;
	ktime_t now;

	seq = atomic_inc_return(&submit_counter);

	/* Log SQ state on first 3 submits to catch initialization differences */
	if (a2xx_debug_first_submit_sq && seq <= 3)
		log_sq_state(gpu, seq);

	if (!a2xx_debug_submit)
		return;
	now = ktime_get();

	rbbm_status = gpu_read(gpu, REG_A2XX_RBBM_STATUS);
	cp_rptr = gpu_read(gpu, REG_AXXX_CP_RB_RPTR);
	cp_wptr = gpu_read(gpu, REG_AXXX_CP_RB_WPTR);

	dev_info(gpu->dev->dev,
		"[A2XX-DBG] submit #%d: seqno=%d cmds=%d rbbm=%08x rptr=%x wptr=%x time=%lld\n",
		seq, submit->seqno, submit->nr_cmds,
		rbbm_status, cp_rptr, cp_wptr, ktime_to_ns(now));
}
EXPORT_SYMBOL(a2xx_debug_log_submit);

/*
 * Log GPU state after submit completes
 */
void a2xx_debug_log_retire(struct msm_gpu *gpu, u32 seqno)
{
	u32 rbbm_status;

	if (!a2xx_debug_rbbm_status)
		return;

	rbbm_status = gpu_read(gpu, REG_A2XX_RBBM_STATUS);

	dev_info(gpu->dev->dev,
		"[A2XX-DBG] retire: seqno=%d rbbm=%08x%s\n",
		seqno, rbbm_status,
		(rbbm_status & A2XX_RBBM_STATUS_GUI_ACTIVE) ? " BUSY" : " idle");
}
EXPORT_SYMBOL(a2xx_debug_log_retire);

/*
 * Force outer cache sync - call before GPU submit
 */
void a2xx_debug_cache_flush(void)
{
	if (!a2xx_debug_flush_cache)
		return;

#ifdef CONFIG_OUTER_CACHE
	outer_sync();
#endif
	/* Full memory barrier to ensure all writes are visible */
	mb();
}
EXPORT_SYMBOL(a2xx_debug_cache_flush);

/*
 * Get the WPTR delay setting
 */
int a2xx_debug_get_wptr_delay(void)
{
	return a2xx_debug_wptr_delay;
}
EXPORT_SYMBOL(a2xx_debug_get_wptr_delay);

/*
 * Coherency test - write pattern from CPU, read from GPU
 * Returns 0 on success, -1 on coherency failure
 */
static int coherency_test(struct msm_gpu *gpu)
{
	u32 test_val = 0xDEADBEEF;
	u32 read_val;
	int i;

	/* Write to scratch register from CPU */
	gpu_write(gpu, REG_AXXX_CP_SCRATCH_REG7, test_val);

	/* Memory barrier */
	wmb();

#ifdef CONFIG_OUTER_CACHE
	outer_sync();
#endif

	/* Read it back - should see our value */
	read_val = gpu_read(gpu, REG_AXXX_CP_SCRATCH_REG7);

	if (read_val != test_val) {
		dev_err(gpu->dev->dev,
			"[A2XX-DBG] Coherency test FAILED: wrote %08x, read %08x\n",
			test_val, read_val);
		return -1;
	}

	/* Test multiple writes */
	for (i = 0; i < 8; i++) {
		test_val = 0x12340000 | i;
		gpu_write(gpu, REG_AXXX_CP_SCRATCH_REG0 + i, test_val);
	}

	wmb();
#ifdef CONFIG_OUTER_CACHE
	outer_sync();
#endif

	for (i = 0; i < 8; i++) {
		test_val = 0x12340000 | i;
		read_val = gpu_read(gpu, REG_AXXX_CP_SCRATCH_REG0 + i);
		if (read_val != test_val) {
			dev_err(gpu->dev->dev,
				"[A2XX-DBG] Scratch%d test FAILED: wrote %08x, read %08x\n",
				i, test_val, read_val);
			return -1;
		}
	}

	dev_info(gpu->dev->dev, "[A2XX-DBG] Coherency test PASSED\n");
	return 0;
}

/*
 * Debugfs: coherency test
 */
static int coherency_test_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;
	int ret;

	if (!gpu) {
		seq_puts(m, "GPU not initialized\n");
		return 0;
	}

	pm_runtime_get_sync(&gpu->pdev->dev);
	ret = coherency_test(gpu);
	pm_runtime_put_sync(&gpu->pdev->dev);

	seq_printf(m, "Coherency test: %s\n", ret == 0 ? "PASSED" : "FAILED");

	return 0;
}

/*
 * Debugfs: submit counter
 */
static int submit_count_show(struct seq_file *m, void *arg)
{
	seq_printf(m, "Total submits: %d\n", atomic_read(&submit_counter));
	return 0;
}

/*
 * Debugfs: timing test - measure latency of GPU operations
 */
static int timing_test_show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;
	ktime_t start, end;
	u32 val;
	int i;

	if (!gpu) {
		seq_puts(m, "GPU not initialized\n");
		return 0;
	}

	pm_runtime_get_sync(&gpu->pdev->dev);

	/* Time register reads */
	start = ktime_get();
	for (i = 0; i < 1000; i++)
		val = gpu_read(gpu, REG_A2XX_RBBM_STATUS);
	end = ktime_get();
	seq_printf(m, "1000 register reads: %lld ns\n",
		   ktime_to_ns(ktime_sub(end, start)));

	/* Time register writes */
	start = ktime_get();
	for (i = 0; i < 1000; i++)
		gpu_write(gpu, REG_AXXX_CP_SCRATCH_REG0, i);
	end = ktime_get();
	seq_printf(m, "1000 register writes: %lld ns\n",
		   ktime_to_ns(ktime_sub(end, start)));

	/* Time memory barrier */
	start = ktime_get();
	for (i = 0; i < 1000; i++)
		mb();
	end = ktime_get();
	seq_printf(m, "1000 mb(): %lld ns\n",
		   ktime_to_ns(ktime_sub(end, start)));

#ifdef CONFIG_OUTER_CACHE
	/* Time outer_sync */
	start = ktime_get();
	for (i = 0; i < 100; i++)
		outer_sync();
	end = ktime_get();
	seq_printf(m, "100 outer_sync(): %lld ns\n",
		   ktime_to_ns(ktime_sub(end, start)));
#else
	seq_puts(m, "outer_sync(): not available (CONFIG_OUTER_CACHE disabled)\n");
#endif

	pm_runtime_put_sync(&gpu->pdev->dev);

	(void)val; /* suppress unused warning */

	return 0;
}

static int show_wrapper(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	int (*show_fn)(struct seq_file *, void *) = node->info_ent->data;
	return show_fn(m, arg);
}

static const struct drm_info_list a2xx_debug_list[] = {
	{ "coherency_test", show_wrapper, 0, coherency_test_show },
	{ "timing_test", show_wrapper, 0, timing_test_show },
	{ "submit_count", show_wrapper, 0, submit_count_show },
};

void a2xx_debug_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(a2xx_debug_list,
				 ARRAY_SIZE(a2xx_debug_list),
				 minor->debugfs_root, minor);
}
