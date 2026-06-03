// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018 The Linux Foundation. All rights reserved. */

#include <linux/delay.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "a2xx_gpu.h"
#include "msm_gem.h"
#include "msm_mmu.h"

extern bool hang_debug;

/*
 * a2xx_kgsl_boost (module parameter) selects whether the a2xx sub-driver
 * enables gpu->kgsl_style_boost. With it true (default, historic post-
 * rewrite behaviour), msm_devfreq_active() ramps the boost_freq QoS
 * request to MAX OPP on every idle->active transition. We track a
 * single GPU pointer here (a2xx has at most one device on msm8660) so
 * a sysfs write to /sys/module/msm/parameters/a2xx_kgsl_boost propagates
 * to the live gpu->kgsl_style_boost without a reboot -- crucial for
 * A/B testing on hardware where the hang appears minutes after boot.
 *
 * Hypothesis under test: dev_pm_qos_update_request() in
 * msm_devfreq_active() is asynchronous against the submit() call that
 * follows it in msm_gpu.c::msm_submit() -- the GPU starts processing
 * the command stream at the OLD low clock, then the clock changes
 * mid-execution, hanging the CP and producing the recurring
 * QSGRenderThread lockups observed on tenderloin.
 */
static bool a2xx_kgsl_boost = true;
static struct msm_gpu *a2xx_live_gpu;

static int a2xx_kgsl_boost_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret && a2xx_live_gpu) {
		a2xx_live_gpu->kgsl_style_boost = a2xx_kgsl_boost;
		pr_info("a2xx: kgsl_style_boost runtime-updated to %d\n",
			a2xx_kgsl_boost);
	}
	return ret;
}

static const struct kernel_param_ops a2xx_kgsl_boost_ops = {
	.set = a2xx_kgsl_boost_set,
	.get = param_get_bool,
};
module_param_cb(a2xx_kgsl_boost, &a2xx_kgsl_boost_ops, &a2xx_kgsl_boost, 0644);
MODULE_PARM_DESC(a2xx_kgsl_boost,
		 "Enable KGSL-style binary boost on idle->active transitions (default 1). Toggleable at runtime; the write also updates the live gpu->kgsl_style_boost.");

static void a2xx_dump(struct msm_gpu *gpu);
static bool a2xx_idle(struct msm_gpu *gpu);

/*
 * Memory bandwidth vote in icc units (kBps), proportional to GPU clock with
 * 8 bytes/cycle on the 64-bit memory bus. Legacy webOS msm_bus voted
 * ab=ib=2008 MB/s for grp3d_max — set both avg and peak so the AFAB
 * aggregator (sum of avg, max of peak) sees the GPU's full demand and
 * scales fabric clock appropriately.
 */
static u32 a2xx_icc_bw_for_freq(unsigned long freq_hz)
{
	return Bps_to_icc(freq_hz) * 8;
}

static void a2xx_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit)
{
	struct msm_ringbuffer *ring = submit->ring;
	unsigned int i;

	/*
	 * a20x/a220 erratum (per legacy KGSL a2xx_drawctxt_draw_workaround):
	 * "if the events for shader space reuse get dropped, the CP block would
	 * wait indefinitely". This happens with repeated idles between submits
	 * (the kernel ring's per-submit CP_WAIT_FOR_IDLE below) and is the
	 * recurring back-end wedge -- the CP parks forever waiting for shader
	 * instruction space that never frees (RBBM shows RB/PA/SC stuck, CP
	 * busy), worst right after a GDSC power-collapse/resume. KGSL unblocks
	 * the CP by re-issuing CP_SET_SHADER_BASES. Emit it at the start of
	 * every submit so the reuse event cannot accumulate-drop across the
	 * inter-submit idles. 0x80000180 = adreno_encode_istore_size() |
	 * pix_shader_start for a220 -- the same value Mesa's fd2_emit_restore
	 * programs per batch.
	 */
	OUT_PKT3(ring, CP_SET_SHADER_BASES, 1);
	OUT_RING(ring, 0x80000180);

	for (i = 0; i < submit->nr_cmds; i++) {
		switch (submit->cmd[i].type) {
		case MSM_SUBMIT_CMD_IB_TARGET_BUF:
			/* ignore IB-targets */
			break;
		case MSM_SUBMIT_CMD_CTX_RESTORE_BUF:
			/* ignore if there has not been a ctx switch: */
			if (ring->cur_ctx_seqno == submit->queue->ctx->seqno)
				break;
			fallthrough;
		case MSM_SUBMIT_CMD_BUF:
			OUT_PKT3(ring, CP_INDIRECT_BUFFER_PFD, 2);
			OUT_RING(ring, lower_32_bits(submit->cmd[i].iova));
			OUT_RING(ring, submit->cmd[i].size);
			OUT_PKT2(ring);
			break;
		}
	}

	OUT_PKT0(ring, REG_AXXX_CP_SCRATCH_REG2, 1);
	OUT_RING(ring, submit->seqno);

	/* wait for idle before cache flush/interrupt */
	OUT_PKT3(ring, CP_WAIT_FOR_IDLE, 1);
	OUT_RING(ring, 0x00000000);

	OUT_PKT3(ring, CP_EVENT_WRITE, 3);
	OUT_RING(ring, CACHE_FLUSH_TS);
	OUT_RING(ring, rbmemptr(ring, fence));
	OUT_RING(ring, submit->seqno);
	OUT_PKT3(ring, CP_INTERRUPT, 1);
	OUT_RING(ring, 0x80000000);

	adreno_flush(gpu, ring, REG_AXXX_CP_RB_WPTR);
}

static bool a2xx_me_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);
	struct msm_ringbuffer *ring = gpu->rb[0];

	OUT_PKT3(ring, CP_ME_INIT, 18);

	/* All fields present (bits 9:0) */
	OUT_RING(ring, 0x000003ff);
	/* Disable/Enable Real-Time Stream processing (present but ignored) */
	OUT_RING(ring, 0x00000000);
	/* Enable (2D <-> 3D) implicit synchronization (present but ignored) */
	OUT_RING(ring, 0x00000000);

	OUT_RING(ring, REG_A2XX_RB_SURFACE_INFO - 0x2000);
	OUT_RING(ring, REG_A2XX_PA_SC_WINDOW_OFFSET - 0x2000);
	OUT_RING(ring, REG_A2XX_VGT_MAX_VTX_INDX - 0x2000);
	OUT_RING(ring, REG_A2XX_SQ_PROGRAM_CNTL - 0x2000);
	OUT_RING(ring, REG_A2XX_RB_DEPTHCONTROL - 0x2000);
	OUT_RING(ring, REG_A2XX_PA_SU_POINT_SIZE - 0x2000);
	OUT_RING(ring, REG_A2XX_PA_SC_LINE_CNTL - 0x2000);
	OUT_RING(ring, REG_A2XX_PA_SU_POLY_OFFSET_FRONT_SCALE - 0x2000);

	/* Vertex and Pixel Shader Start Addresses in instructions
	 * (3 DWORDS per instruction) */
	OUT_RING(ring, 0x80000180);
	/* Maximum Contexts */
	OUT_RING(ring, 0x00000001);
	/* Write Confirm Interval and The CP will wait the
	 * wait_interval * 16 clocks between polling  */
	OUT_RING(ring, 0x00000000);
	/* NQ and External Memory Swap */
	OUT_RING(ring, 0x00000000);
	/* protected mode error checking (0x1f2 is REG_AXXX_CP_INT_CNTL) */
	if (a2xx_gpu->protection_disabled)
		OUT_RING(ring, 0x00000000);
	else
		OUT_RING(ring, 0x200001f2);
	/* Disable header dumping and Header dump address */
	OUT_RING(ring, 0x00000000);
	/* Header dump size */
	OUT_RING(ring, 0x00000000);

	if (!a2xx_gpu->protection_disabled) {
		/* enable protected mode */
		OUT_PKT3(ring, CP_SET_PROTECTED_MODE, 1);
		OUT_RING(ring, 1);
	}

	adreno_flush(gpu, ring, REG_AXXX_CP_RB_WPTR);
	return a2xx_idle(gpu);
}

static int a2xx_hw_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);
	dma_addr_t pt_base, tran_error;
	uint32_t *ptr, len;
	int i, ret;

	a2xx_gpummu_params(to_msm_vm(gpu->vm)->mmu, &pt_base, &tran_error);

	DBG("%s", gpu->name);

	/* halt ME to avoid ucode upload issues on a20x */
	gpu_write(gpu, REG_AXXX_CP_ME_CNTL, AXXX_CP_ME_CNTL_HALT);

	gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE1, 0xfffffffe);
	gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0xffffffff);

	/*
	 * KGSL (a2xx_start) resets ALL blocks (0xffffffff) only on the very
	 * first init; on every subsequent a22x (re)init -- crucially including
	 * resume from a GDSC power-collapse -- it resets ONLY the CP block
	 * (0x1). Repeating the full block soft-reset on a22x resume leaves the
	 * 3D pipe in a state where the first draw after resume wedges the
	 * back-end (the recurring resume-from-autosuspend hang). a20x always
	 * takes the full reset. Mirror KGSL.
	 */
	if (adreno_is_a20x(adreno_gpu) || !a2xx_gpu->soft_reset_done)
		gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0xffffffff);
	else
		gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0x00000001);
	a2xx_gpu->soft_reset_done = true;
	msleep(30);
	gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0x00000000);

	if (adreno_is_a225(adreno_gpu))
		gpu_write(gpu, REG_A2XX_SQ_FLOW_CONTROL, 0x18000000);

	/* note: kgsl uses 0x0000ffff for a20x */
	gpu_write(gpu, REG_A2XX_RBBM_CNTL, 0x00004442);

	/* MPU: physical range */
	gpu_write(gpu, REG_A2XX_MH_MMU_MPU_BASE, 0x00000000);
	gpu_write(gpu, REG_A2XX_MH_MMU_MPU_END, 0xfffff000);

	gpu_write(gpu, REG_A2XX_MH_MMU_CONFIG, A2XX_MH_MMU_CONFIG_MMU_ENABLE |
		A2XX_MH_MMU_CONFIG_RB_W_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_CP_W_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_CP_R0_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_CP_R1_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_CP_R2_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_CP_R3_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_CP_R4_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_VGT_R0_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_VGT_R1_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_TC_R_CLNT_BEHAVIOR(BEH_TRAN_RNG) |
		A2XX_MH_MMU_CONFIG_PA_W_CLNT_BEHAVIOR(BEH_TRAN_RNG));

	/* same as parameters in adreno_gpu */
	gpu_write(gpu, REG_A2XX_MH_MMU_VA_RANGE, SZ_16M |
		A2XX_MH_MMU_VA_RANGE_NUM_64KB_REGIONS(0xfff));

	gpu_write(gpu, REG_A2XX_MH_MMU_PT_BASE, pt_base);
	gpu_write(gpu, REG_A2XX_MH_MMU_TRAN_ERROR, tran_error);

	gpu_write(gpu, REG_A2XX_MH_MMU_INVALIDATE,
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_ALL |
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_TC);

	/*
	 * MH ARBITER config - matches legacy KGSL's KGSL_CFG_YAMATO_MHARB
	 * for both Yamato (a20x) and Leia (a22x = REV470). Live readback
	 * from webOS/KGSL on a Leia REV470 (HP TouchPad) confirms
	 * MH_ARBITER_CONFIG = 0x07c86590 for both generations.
	 *
	 * IN_FLIGHT_LIMIT_ENABLE (bit 15) is intentionally NOT set:
	 * mainline used to set it (with IN_FLIGHT_LIMIT=8), but the MH
	 * arbiter's per-tag tracking with the limit enforced was producing
	 * a deterministic period-8 rendering cycle on a22x where 1/8 of
	 * fresh DRM contexts rendered correctly and 7/8 had channel-drop
	 * + GMEM-tile artifacts. KGSL leaves the limit unenforced (the
	 * IN_FLIGHT_LIMIT(8) field is still written but the enable bit
	 * is off, so the field is dead). Mirror KGSL exactly.
	 */
	gpu_write(gpu, REG_A2XX_MH_ARBITER_CONFIG,
		A2XX_MH_ARBITER_CONFIG_SAME_PAGE_LIMIT(16) |
		A2XX_MH_ARBITER_CONFIG_L1_ARB_ENABLE |
		A2XX_MH_ARBITER_CONFIG_L1_ARB_HOLD_ENABLE |
		A2XX_MH_ARBITER_CONFIG_PAGE_SIZE(1) |
		A2XX_MH_ARBITER_CONFIG_TC_REORDER_ENABLE |
		A2XX_MH_ARBITER_CONFIG_TC_ARB_HOLD_ENABLE |
		A2XX_MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT(8) |
		A2XX_MH_ARBITER_CONFIG_CP_CLNT_ENABLE |
		A2XX_MH_ARBITER_CONFIG_VGT_CLNT_ENABLE |
		A2XX_MH_ARBITER_CONFIG_TC_CLNT_ENABLE |
		A2XX_MH_ARBITER_CONFIG_RB_CLNT_ENABLE |
		A2XX_MH_ARBITER_CONFIG_PA_CLNT_ENABLE);
	if (!adreno_is_a20x(adreno_gpu))
		gpu_write(gpu, REG_A2XX_MH_CLNT_INTF_CTRL_CONFIG1, 0x00032f07);

	gpu_write(gpu, REG_A2XX_SQ_VS_PROGRAM, 0x00000000);
	gpu_write(gpu, REG_A2XX_SQ_PS_PROGRAM, 0x00000000);

	gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE1, 0);
	/*
	 * A22x (Leia) requires 0x1a0 for proper clock gating per Palm kernel.
	 * Bit 0x40 keeps the RBBM performance-monitor block clocked so the RBBM
	 * busy perfcounter (the devfreq load source, enabled via CP_PERFMON_CNTL
	 * below) actually counts -- KGSL sets (PM_OVERRIDE2 | 0x40) in
	 * a2xx_busy_cycles() for exactly this. Without it the counter stays
	 * frozen, devfreq sees ~0 load and parks the GPU at its minimum OPP.
	 */
	if (!adreno_is_a20x(adreno_gpu))
		gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0x1a0 | 0x40);
	else
		gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0x40);

	/*
	 * Initialize SQ_GPR_MANAGEMENT to the legacy KGSL value
	 * (0x00040400 = VTX=64, PIX=64, static). Without an explicit init,
	 * random power-on values could starve one shader type of GPRs.
	 */
	gpu_write(gpu, REG_A2XX_SQ_GPR_MANAGEMENT, 0x00040400);

	/* note: gsl doesn't set this */
	gpu_write(gpu, REG_A2XX_RBBM_DEBUG, 0x00080000);

	gpu_write(gpu, REG_A2XX_RBBM_INT_CNTL,
		A2XX_RBBM_INT_CNTL_RDERR_INT_MASK);
	gpu_write(gpu, REG_AXXX_CP_INT_CNTL,
		AXXX_CP_INT_CNTL_T0_PACKET_IN_IB_MASK |
		AXXX_CP_INT_CNTL_OPCODE_ERROR_MASK |
		AXXX_CP_INT_CNTL_PROTECTED_MODE_ERROR_MASK |
		AXXX_CP_INT_CNTL_RESERVED_BIT_ERROR_MASK |
		AXXX_CP_INT_CNTL_IB_ERROR_MASK |
		AXXX_CP_INT_CNTL_IB1_INT_MASK |
		AXXX_CP_INT_CNTL_RB_INT_MASK);
	gpu_write(gpu, REG_A2XX_SQ_INT_CNTL, 0);
	gpu_write(gpu, REG_A2XX_MH_INTERRUPT_MASK,
		A2XX_MH_INTERRUPT_MASK_AXI_READ_ERROR |
		A2XX_MH_INTERRUPT_MASK_AXI_WRITE_ERROR |
		A2XX_MH_INTERRUPT_MASK_MMU_PAGE_FAULT);

	for (i = 3; i <= 5; i++)
		if ((SZ_16K << i) == adreno_gpu->info->gmem)
			break;
	gpu_write(gpu, REG_A2XX_RB_EDRAM_INFO, i);

	/*
	 * Select RBBM perfcounter 1 as the devfreq busy-cycle source.
	 * RBBM1_NRT_BUSY (general non-real-time 3D-pipe busy) is what legacy
	 * KGSL a2xx_busy_cycles() selects -- broader than RB_BUSY, so it also
	 * tracks vertex/shader-bound work, giving devfreq a truer load signal.
	 * NOTE: the counter only advances while the perfmon is ENABLED via
	 * CP_PERFMON_CNTL, which Mesa owns per-batch (freedreno fd2_emit_restore
	 * writes it every batch); stock Mesa writes CP_PERFMON_CNTL=0 (perfmon
	 * frozen) so this never counted and devfreq parked the GPU at min. The
	 * paired Mesa change makes fd2_emit_restore emit CP_PERFMON_CNTL=1.
	 */
	gpu_write(gpu, REG_A2XX_RBBM_PERFCOUNTER1_SELECT, RBBM1_NRT_BUSY);

	/*
	 * Select RB perfcounter 0 as a "retired rendering work" signal for the
	 * hangcheck progress check (a2xx_progress). RBPERF_SX_RB_QUAD_SEND counts
	 * quads that have finished the fragment shader and are sent to the render
	 * backend, so it climbs throughout a heavy fragment-bound draw (where the
	 * CP parks waiting on the pixel pipeline and the IB pointers go static)
	 * yet stops if the pipeline genuinely wedges -- unlike the *_BUSY counters
	 * which keep ticking on a stuck-busy back-end. Like the devfreq counter
	 * above it only advances while the perfmon is enabled (CP_PERFMON_CNTL,
	 * kept on per-batch by the Mesa change).
	 */
	gpu_write(gpu, REG_A2XX_RB_PERFCOUNTER0_SELECT, RBPERF_SX_RB_QUAD_SEND);

	ret = adreno_hw_init(gpu);
	if (ret)
		return ret;

	gpu_write(gpu, REG_AXXX_CP_RB_CNTL,
		MSM_GPU_RB_CNTL_DEFAULT | AXXX_CP_RB_CNTL_NO_UPDATE);

	gpu_write(gpu, REG_AXXX_CP_RB_BASE, lower_32_bits(gpu->rb[0]->iova));

	/* NOTE: PM4/micro-engine firmware registers look to be the same
	 * for a2xx and a3xx.. we could possibly push that part down to
	 * adreno_gpu base class.  Or push both PM4 and PFP but
	 * parameterize the pfp ucode addr/data registers..
	 */

	/* Load PM4: */
	ptr = (uint32_t *)(adreno_gpu->fw[ADRENO_FW_PM4]->data);
	len = adreno_gpu->fw[ADRENO_FW_PM4]->size / 4;
	DBG("loading PM4 ucode version: %x", ptr[1]);

	/*
	 * New firmware files seem to have GPU and firmware version in this
	 * word (0x20xxxx for A200, 0x220xxx for A220, 0x225xxx for A225).
	 * Older firmware files, which lack protection support, have 0 instead.
	 */
	if (ptr[1] == 0) {
		dev_warn(gpu->dev->dev,
			 "Legacy firmware detected, disabling protection support\n");
		a2xx_gpu->protection_disabled = true;
	}

	gpu_write(gpu, REG_AXXX_CP_DEBUG,
			AXXX_CP_DEBUG_MIU_128BIT_WRITE_ENABLE);
	gpu_write(gpu, REG_AXXX_CP_ME_RAM_WADDR, 0);
	for (i = 1; i < len; i++)
		gpu_write(gpu, REG_AXXX_CP_ME_RAM_DATA, ptr[i]);

	/* Load PFP: */
	ptr = (uint32_t *)(adreno_gpu->fw[ADRENO_FW_PFP]->data);
	len = adreno_gpu->fw[ADRENO_FW_PFP]->size / 4;
	DBG("loading PFP ucode version: %x", ptr[5]);

	gpu_write(gpu, REG_A2XX_CP_PFP_UCODE_ADDR, 0);
	for (i = 1; i < len; i++)
		gpu_write(gpu, REG_A2XX_CP_PFP_UCODE_DATA, ptr[i]);

	gpu_write(gpu, REG_AXXX_CP_QUEUE_THRESHOLDS, 0x000C0804);

	/*
	 * Clear any pending CP interrupts before starting the micro engine.
	 * This matches the KGSL driver sequence and ensures the CP starts
	 * with a clean interrupt state, which is important for recovery.
	 */
	gpu_write(gpu, REG_AXXX_CP_INT_ACK, 0xFFFFFFFF);

	/*
	 * Reset ring pointers immediately before starting the ME.
	 * This matches the KGSL sequence where rb->rptr = rb->wptr = 0
	 * is set just before clearing ME_HALT. Critical for recovery.
	 */
	gpu->rb[0]->cur = gpu->rb[0]->next = gpu->rb[0]->start;
	gpu->rb[0]->memptrs->rptr = 0;
	gpu_write(gpu, REG_AXXX_CP_RB_WPTR, 0);

	/* clear ME_HALT to start micro engine */
	gpu_write(gpu, REG_AXXX_CP_ME_CNTL, 0);

	return a2xx_me_init(gpu) ? 0 : -EINVAL;
}

static void a2xx_recover(struct msm_gpu *gpu)
{
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(to_adreno_gpu(gpu));
	int i;

	adreno_dump_info(gpu);

	for (i = 0; i < 8; i++) {
		printk("CP_SCRATCH_REG%d: %u\n", i,
			gpu_read(gpu, REG_AXXX_CP_SCRATCH_REG0 + i));
	}

	/* dump registers before resetting gpu, if enabled: */
	if (hang_debug)
		a2xx_dump(gpu);

	/*
	 * Perform a full GPU reset matching the sequence in a2xx_hw_init()
	 * and the legacy KGSL driver. A partial reset (0x1 = CP only) is
	 * insufficient when the GPU is truly hung: halt the ME, power all
	 * blocks on via PM_OVERRIDE, assert reset on all blocks, wait 30ms
	 * (per KGSL), then deassert.
	 */
	gpu_write(gpu, REG_AXXX_CP_ME_CNTL, AXXX_CP_ME_CNTL_HALT);

	gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE1, 0xfffffffe);
	gpu_write(gpu, REG_A2XX_RBBM_PM_OVERRIDE2, 0xffffffff);

	gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0xffffffff);
	msleep(30);
	gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0);

	/*
	 * Clear the hung SQ / parameter-cache SRAM that RBBM_SOFT_RESET above
	 * cannot: assert the GFX3D *core* reset (mmcc GFX3D_RESET, the same reset
	 * the gfx3d GDSC fires on a cold power-on -- the period-8 fix). Without
	 * it, the post-recover a2xx_hw_init() -> a2xx_me_init() -> a2xx_idle()
	 * times out (hw_init -EINVAL), turning every lockup into an
	 * unrecoverable hangcheck -> recover death-spiral.
	 *
	 * We assert it directly rather than power-cycling the GDSC: two-tier
	 * runtime PM (GENPD_FLAG_RPM_ALWAYS_ON) keeps the gfx3d footswitch
	 * powered across runtime idle, so a pm_runtime put/get no longer
	 * collapses the domain to re-fire the reset. Pulse width follows the
	 * legacy footswitch sequence (assert, ~us, deassert).
	 */
	if (a2xx_gpu->core_reset) {
		reset_control_assert(a2xx_gpu->core_reset);
		udelay(2);
		reset_control_deassert(a2xx_gpu->core_reset);
	} else {
		/*
		 * No "core" reset in DT: fall back to a runtime-PM power cycle,
		 * which only re-fires the GDSC reset where the domain is actually
		 * allowed to collapse (i.e. not RPM_ALWAYS_ON).
		 */
		pm_runtime_put_sync_suspend(&gpu->pdev->dev);
		pm_runtime_get_sync(&gpu->pdev->dev);
	}

	/*
	 * The core was just reset, so its state (incl. loaded microcode) is gone
	 * regardless of the runtime-PM path taken above. Force the re-init done
	 * by adreno_recover() -> msm_gpu_hw_init(); a light pm_resume there would
	 * otherwise leave needs_hw_init clear and skip it.
	 */
	gpu->needs_hw_init = true;

	adreno_recover(gpu);
}

static void a2xx_destroy(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);

	DBG("%s", gpu->name);

	if (READ_ONCE(a2xx_live_gpu) == gpu)
		WRITE_ONCE(a2xx_live_gpu, NULL);

	adreno_gpu_cleanup(adreno_gpu);

	kfree(a2xx_gpu);
}

/*
 * Mask of all RBBM_STATUS bits that indicate the GPU is busy.
 * Derived from the webOS KGSL driver which checks RBBM_STATUS == 0x110
 * for idle (only CMDFIFO_AVAIL and HIRQ_PENDING are allowed to be set).
 * All these bits must be clear for the gfx3d_axi_clk to halt properly.
 */
#define A2XX_RBBM_BUSY_MASK ( \
	A2XX_RBBM_STATUS_TC_BUSY | \
	A2XX_RBBM_STATUS_CPRQ_PENDING | \
	A2XX_RBBM_STATUS_CFRQ_PENDING | \
	A2XX_RBBM_STATUS_PFRQ_PENDING | \
	A2XX_RBBM_STATUS_VGT_BUSY_NO_DMA | \
	A2XX_RBBM_STATUS_RBBM_WU_BUSY | \
	A2XX_RBBM_STATUS_CP_NRT_BUSY | \
	A2XX_RBBM_STATUS_MH_BUSY | \
	A2XX_RBBM_STATUS_MH_COHERENCY_BUSY | \
	A2XX_RBBM_STATUS_SX_BUSY | \
	A2XX_RBBM_STATUS_TPC_BUSY | \
	A2XX_RBBM_STATUS_SC_CNTX_BUSY | \
	A2XX_RBBM_STATUS_PA_BUSY | \
	A2XX_RBBM_STATUS_VGT_BUSY | \
	A2XX_RBBM_STATUS_SQ_CNTX17_BUSY | \
	A2XX_RBBM_STATUS_SQ_CNTX0_BUSY | \
	A2XX_RBBM_STATUS_RB_CNTX_BUSY | \
	A2XX_RBBM_STATUS_GUI_ACTIVE)

static bool a2xx_idle(struct msm_gpu *gpu)
{
	uint32_t status;

	/* wait for ringbuffer to drain: */
	if (!adreno_idle(gpu, gpu->rb[0]))
		return false;

	/* then wait for GPU to finish: */
	if (spin_until(!(gpu_read(gpu, REG_A2XX_RBBM_STATUS) &
			A2XX_RBBM_STATUS_GUI_ACTIVE))) {
		DRM_ERROR("%s: timeout waiting for GPU to idle!\n", gpu->name);

		/* TODO maybe we need to reset GPU here to recover from hang? */
		return false;
	}

	/*
	 * Wait for ALL busy bits to clear, not just the obvious ones.
	 * The webOS KGSL driver waits for RBBM_STATUS == 0x110 (only
	 * CMDFIFO_AVAIL and HIRQ_PENDING set). If any busy bit remains
	 * set, the gfx3d_axi_clk branch clock cannot halt properly.
	 */
	if (spin_until(!((status = gpu_read(gpu, REG_A2XX_RBBM_STATUS)) &
			 A2XX_RBBM_BUSY_MASK))) {
		DRM_ERROR("%s: timeout waiting for GPU idle, RBBM_STATUS=%08x\n",
			  gpu->name, status);
		return false;
	}

	return true;
}

static irqreturn_t a2xx_irq(struct msm_gpu *gpu)
{
	uint32_t mstatus, status;

	mstatus = gpu_read(gpu, REG_A2XX_MASTER_INT_SIGNAL);

	if (mstatus & A2XX_MASTER_INT_SIGNAL_MH_INT_STAT) {
		status = gpu_read(gpu, REG_A2XX_MH_INTERRUPT_STATUS);

		/*
		 * MH_INT bit0=AXI_READ_ERR, bit1=AXI_WRITE_ERR, bit2=MMU_PAGE_FAULT.
		 * AXI_WRITE_ERROR is recoverable in place: the bus dropped a write,
		 * the CP carries on, ack-and-ignore is the legacy KGSL behaviour.
		 * AXI_READ_ERROR / MMU_PAGE_FAULT, however, leave the CP parked on
		 * the faulting fetch -- the MH line re-asserts on every bus cycle.
		 * Without halting the CP and masking MH, the ISR re-enters in a
		 * tight loop, starves the FIFO-LOW gpu_worker / drm_sched workers
		 * (RT throttle), and the hangcheck softirq never runs -> RCU stall.
		 *
		 * AXI_ERR (raw reg 0x0a45 = MH_AXI_ERROR, between MH_INTERRUPT_CLEAR
		 * 0xa44 and PERFCOUNTER0 0xa46) holds the offending AXI address.
		 */
		dev_warn_ratelimited(gpu->dev->dev,
			"MH_INT: %08X AXI_ERR: %08X PF: %08X RBBM: %08X IB1: %08X/%u IB2: %08X COPY_DEST: %08X COLOR_INFO: %08X\n",
			status,
			gpu_read(gpu, 0x0a45 /* MH_AXI_ERROR */),
			gpu_read(gpu, REG_A2XX_MH_MMU_PAGE_FAULT),
			gpu_read(gpu, REG_A2XX_RBBM_STATUS),
			gpu_read(gpu, REG_AXXX_CP_IB1_BASE),
			gpu_read(gpu, REG_AXXX_CP_IB1_BUFSZ),
			gpu_read(gpu, REG_AXXX_CP_IB2_BASE),
			gpu_read(gpu, REG_A2XX_RB_COPY_DEST_BASE),
			gpu_read(gpu, REG_A2XX_RB_COLOR_INFO));

		gpu_write(gpu, REG_A2XX_MH_INTERRUPT_CLEAR, status);

		if (status & (A2XX_MH_INTERRUPT_MASK_AXI_READ_ERROR |
			      A2XX_MH_INTERRUPT_MASK_MMU_PAGE_FAULT)) {
			/*
			 * Halt the micro-engine so it stops re-fetching the faulting
			 * address, then mask MH so any residual edge cannot re-arm
			 * before a2xx_hw_init() re-enables it during recovery.
			 */
			gpu_write(gpu, REG_AXXX_CP_ME_CNTL, AXXX_CP_ME_CNTL_HALT);
			gpu_write(gpu, REG_A2XX_MH_INTERRUPT_MASK, 0);

			/*
			 * Kick recover_work directly instead of waiting for the
			 * 500ms hangcheck softirq -- under an MH IRQ storm the
			 * softirq doesn't run in time. recover_worker takes
			 * gpu->lock, dumps crash state (which records the offending
			 * submit + BO list), runs ->recover(), and replays.
			 */
			kthread_queue_work(gpu->worker, &gpu->recover_work);
		}
	}

	if (mstatus & A2XX_MASTER_INT_SIGNAL_CP_INT_STAT) {
		status = gpu_read(gpu, REG_AXXX_CP_INT_STATUS);

		/* only RB_INT is expected */
		if (status & ~AXXX_CP_INT_CNTL_RB_INT_MASK)
			dev_warn(gpu->dev->dev, "CP_INT: %08X\n", status);

		gpu_write(gpu, REG_AXXX_CP_INT_ACK, status);
	}

	if (mstatus & A2XX_MASTER_INT_SIGNAL_RBBM_INT_STAT) {
		status = gpu_read(gpu, REG_A2XX_RBBM_INT_STATUS);

		dev_dbg(gpu->dev->dev, "RBBM_INT: %08X\n", status);

		gpu_write(gpu, REG_A2XX_RBBM_INT_ACK, status);
	}

	msm_gpu_retire(gpu);

	return IRQ_HANDLED;
}

static const unsigned int a200_registers[] = {
	0x0000, 0x0002, 0x0004, 0x000B, 0x003B, 0x003D, 0x0040, 0x0044,
	0x0046, 0x0047, 0x01C0, 0x01C1, 0x01C3, 0x01C8, 0x01D5, 0x01D9,
	0x01DC, 0x01DD, 0x01EA, 0x01EA, 0x01EE, 0x01F3, 0x01F6, 0x01F7,
	0x01FC, 0x01FF, 0x0391, 0x0392, 0x039B, 0x039E, 0x03B2, 0x03B5,
	0x03B7, 0x03B7, 0x03F8, 0x03FB, 0x0440, 0x0440, 0x0443, 0x0444,
	0x044B, 0x044B, 0x044D, 0x044F, 0x0452, 0x0452, 0x0454, 0x045B,
	0x047F, 0x047F, 0x0578, 0x0587, 0x05C9, 0x05C9, 0x05D0, 0x05D0,
	0x0601, 0x0604, 0x0606, 0x0609, 0x060B, 0x060E, 0x0613, 0x0614,
	0x0A29, 0x0A2B, 0x0A2F, 0x0A31, 0x0A40, 0x0A43, 0x0A45, 0x0A45,
	0x0A4E, 0x0A4F, 0x0C2C, 0x0C2C, 0x0C30, 0x0C30, 0x0C38, 0x0C3C,
	0x0C40, 0x0C40, 0x0C44, 0x0C44, 0x0C80, 0x0C86, 0x0C88, 0x0C94,
	0x0C99, 0x0C9A, 0x0CA4, 0x0CA5, 0x0D00, 0x0D03, 0x0D06, 0x0D06,
	0x0D08, 0x0D0B, 0x0D34, 0x0D35, 0x0DAE, 0x0DC1, 0x0DC8, 0x0DD4,
	0x0DD8, 0x0DD9, 0x0E00, 0x0E00, 0x0E02, 0x0E04, 0x0E17, 0x0E1E,
	0x0EC0, 0x0EC9, 0x0ECB, 0x0ECC, 0x0ED0, 0x0ED0, 0x0ED4, 0x0ED7,
	0x0EE0, 0x0EE2, 0x0F01, 0x0F02, 0x0F0C, 0x0F0C, 0x0F0E, 0x0F12,
	0x0F26, 0x0F2A, 0x0F2C, 0x0F2C, 0x2000, 0x2002, 0x2006, 0x200F,
	0x2080, 0x2082, 0x2100, 0x2109, 0x210C, 0x2114, 0x2180, 0x2184,
	0x21F5, 0x21F7, 0x2200, 0x2208, 0x2280, 0x2283, 0x2293, 0x2294,
	0x2300, 0x2308, 0x2312, 0x2312, 0x2316, 0x231D, 0x2324, 0x2326,
	0x2380, 0x2383, 0x2400, 0x2402, 0x2406, 0x240F, 0x2480, 0x2482,
	0x2500, 0x2509, 0x250C, 0x2514, 0x2580, 0x2584, 0x25F5, 0x25F7,
	0x2600, 0x2608, 0x2680, 0x2683, 0x2693, 0x2694, 0x2700, 0x2708,
	0x2712, 0x2712, 0x2716, 0x271D, 0x2724, 0x2726, 0x2780, 0x2783,
	0x4000, 0x4003, 0x4800, 0x4805, 0x4900, 0x4900, 0x4908, 0x4908,
	~0   /* sentinel */
};

static const unsigned int a220_registers[] = {
	0x0000, 0x0002, 0x0004, 0x000B, 0x003B, 0x003D, 0x0040, 0x0044,
	0x0046, 0x0047, 0x01C0, 0x01C1, 0x01C3, 0x01C8, 0x01D5, 0x01D9,
	0x01DC, 0x01DD, 0x01EA, 0x01EA, 0x01EE, 0x01F3, 0x01F6, 0x01F7,
	0x01FC, 0x01FF, 0x0391, 0x0392, 0x039B, 0x039E, 0x03B2, 0x03B5,
	0x03B7, 0x03B7, 0x03F8, 0x03FB, 0x0440, 0x0440, 0x0443, 0x0444,
	0x044B, 0x044B, 0x044D, 0x044F, 0x0452, 0x0452, 0x0454, 0x045B,
	0x047F, 0x047F, 0x0578, 0x0587, 0x05C9, 0x05C9, 0x05D0, 0x05D0,
	0x0601, 0x0604, 0x0606, 0x0609, 0x060B, 0x060E, 0x0613, 0x0614,
	0x0A29, 0x0A2B, 0x0A2F, 0x0A31, 0x0A40, 0x0A40, 0x0A42, 0x0A43,
	0x0A45, 0x0A45, 0x0A4E, 0x0A4F, 0x0C30, 0x0C30, 0x0C38, 0x0C39,
	0x0C3C, 0x0C3C, 0x0C80, 0x0C81, 0x0C88, 0x0C93, 0x0D00, 0x0D03,
	0x0D05, 0x0D06, 0x0D08, 0x0D0B, 0x0D34, 0x0D35, 0x0DAE, 0x0DC1,
	0x0DC8, 0x0DD4, 0x0DD8, 0x0DD9, 0x0E00, 0x0E00, 0x0E02, 0x0E04,
	0x0E17, 0x0E1E, 0x0EC0, 0x0EC9, 0x0ECB, 0x0ECC, 0x0ED0, 0x0ED0,
	0x0ED4, 0x0ED7, 0x0EE0, 0x0EE2, 0x0F01, 0x0F02, 0x2000, 0x2002,
	0x2006, 0x200F, 0x2080, 0x2082, 0x2100, 0x2102, 0x2104, 0x2109,
	0x210C, 0x2114, 0x2180, 0x2184, 0x21F5, 0x21F7, 0x2200, 0x2202,
	0x2204, 0x2204, 0x2208, 0x2208, 0x2280, 0x2282, 0x2294, 0x2294,
	0x2300, 0x2308, 0x2309, 0x230A, 0x2312, 0x2312, 0x2316, 0x2316,
	0x2318, 0x231D, 0x2324, 0x2326, 0x2380, 0x2383, 0x2400, 0x2402,
	0x2406, 0x240F, 0x2480, 0x2482, 0x2500, 0x2502, 0x2504, 0x2509,
	0x250C, 0x2514, 0x2580, 0x2584, 0x25F5, 0x25F7, 0x2600, 0x2602,
	0x2604, 0x2606, 0x2608, 0x2608, 0x2680, 0x2682, 0x2694, 0x2694,
	0x2700, 0x2708, 0x2712, 0x2712, 0x2716, 0x2716, 0x2718, 0x271D,
	0x2724, 0x2726, 0x2780, 0x2783, 0x4000, 0x4003, 0x4800, 0x4805,
	0x4900, 0x4900, 0x4908, 0x4908,
	~0   /* sentinel */
};

static const unsigned int a225_registers[] = {
	0x0000, 0x0002, 0x0004, 0x000B, 0x003B, 0x003D, 0x0040, 0x0044,
	0x0046, 0x0047, 0x013C, 0x013C, 0x0140, 0x014F, 0x01C0, 0x01C1,
	0x01C3, 0x01C8, 0x01D5, 0x01D9, 0x01DC, 0x01DD, 0x01EA, 0x01EA,
	0x01EE, 0x01F3, 0x01F6, 0x01F7, 0x01FC, 0x01FF, 0x0391, 0x0392,
	0x039B, 0x039E, 0x03B2, 0x03B5, 0x03B7, 0x03B7, 0x03F8, 0x03FB,
	0x0440, 0x0440, 0x0443, 0x0444, 0x044B, 0x044B, 0x044D, 0x044F,
	0x0452, 0x0452, 0x0454, 0x045B, 0x047F, 0x047F, 0x0578, 0x0587,
	0x05C9, 0x05C9, 0x05D0, 0x05D0, 0x0601, 0x0604, 0x0606, 0x0609,
	0x060B, 0x060E, 0x0613, 0x0614, 0x0A29, 0x0A2B, 0x0A2F, 0x0A31,
	0x0A40, 0x0A40, 0x0A42, 0x0A43, 0x0A45, 0x0A45, 0x0A4E, 0x0A4F,
	0x0C01, 0x0C1D, 0x0C30, 0x0C30, 0x0C38, 0x0C39, 0x0C3C, 0x0C3C,
	0x0C80, 0x0C81, 0x0C88, 0x0C93, 0x0D00, 0x0D03, 0x0D05, 0x0D06,
	0x0D08, 0x0D0B, 0x0D34, 0x0D35, 0x0DAE, 0x0DC1, 0x0DC8, 0x0DD4,
	0x0DD8, 0x0DD9, 0x0E00, 0x0E00, 0x0E02, 0x0E04, 0x0E17, 0x0E1E,
	0x0EC0, 0x0EC9, 0x0ECB, 0x0ECC, 0x0ED0, 0x0ED0, 0x0ED4, 0x0ED7,
	0x0EE0, 0x0EE2, 0x0F01, 0x0F02, 0x2000, 0x200F, 0x2080, 0x2082,
	0x2100, 0x2109, 0x210C, 0x2114, 0x2180, 0x2184, 0x21F5, 0x21F7,
	0x2200, 0x2202, 0x2204, 0x2206, 0x2208, 0x2210, 0x2220, 0x2222,
	0x2280, 0x2282, 0x2294, 0x2294, 0x2297, 0x2297, 0x2300, 0x230A,
	0x2312, 0x2312, 0x2315, 0x2316, 0x2318, 0x231D, 0x2324, 0x2326,
	0x2340, 0x2357, 0x2360, 0x2360, 0x2380, 0x2383, 0x2400, 0x240F,
	0x2480, 0x2482, 0x2500, 0x2509, 0x250C, 0x2514, 0x2580, 0x2584,
	0x25F5, 0x25F7, 0x2600, 0x2602, 0x2604, 0x2606, 0x2608, 0x2610,
	0x2620, 0x2622, 0x2680, 0x2682, 0x2694, 0x2694, 0x2697, 0x2697,
	0x2700, 0x270A, 0x2712, 0x2712, 0x2715, 0x2716, 0x2718, 0x271D,
	0x2724, 0x2726, 0x2740, 0x2757, 0x2760, 0x2760, 0x2780, 0x2783,
	0x4000, 0x4003, 0x4800, 0x4806, 0x4808, 0x4808, 0x4900, 0x4900,
	0x4908, 0x4908,
	~0   /* sentinel */
};

/* would be nice to not have to duplicate the _show() stuff with printk(): */
static void a2xx_dump(struct msm_gpu *gpu)
{
	printk("status:   %08x\n",
			gpu_read(gpu, REG_A2XX_RBBM_STATUS));
	adreno_dump(gpu);
}

static struct msm_gpu_state *a2xx_gpu_state_get(struct msm_gpu *gpu)
{
	struct msm_gpu_state *state = kzalloc(sizeof(*state), GFP_KERNEL);

	if (!state)
		return ERR_PTR(-ENOMEM);

	adreno_gpu_state_get(gpu, state);

	state->rbbm_status = gpu_read(gpu, REG_A2XX_RBBM_STATUS);

	return state;
}

static u64 a2xx_gpu_busy(struct msm_gpu *gpu, unsigned long *out_sample_rate)
{
	u64 busy_cycles;

	busy_cycles = gpu_read64(gpu, REG_A2XX_RBBM_PERFCOUNTER1_LO);
	*out_sample_rate = clk_get_rate(gpu->core_clk);

	return busy_cycles;
}

static struct drm_gpuvm *
a2xx_create_vm(struct msm_gpu *gpu, struct platform_device *pdev)
{
	struct msm_mmu *mmu = a2xx_gpummu_new(&pdev->dev, gpu);
	struct drm_gpuvm *vm;

	vm = msm_gem_vm_create(gpu->dev, mmu, "gpu", SZ_16M, 0xfff * SZ_64K, true);

	if (IS_ERR(vm) && !IS_ERR(mmu))
		mmu->funcs->destroy(mmu);

	return vm;
}

static u32 a2xx_get_rptr(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	ring->memptrs->rptr = gpu_read(gpu, REG_AXXX_CP_RB_RPTR);
	return ring->memptrs->rptr;
}

/*
 * Hangcheck progress check. Two complementary signals are sampled; if either
 * changed since the previous hangcheck the GPU is making forward progress and
 * is not hung:
 *
 *  - CP IB1/IB2 base + remaining buffer size: advances as the CP consumes the
 *    command stream (covers CP/state-setup and geometry phases).
 *
 *  - RB retired-quad counter (RBPERF_SX_RB_QUAD_SEND, configured in hw_init):
 *    advances as shaded quads reach the render backend. This is essential on
 *    the a220 because a heavy fragment-bound draw (e.g. glmark2's multi-pass
 *    blur at ~1fps) parks the CP at the draw packet -- the IB pointers go
 *    static for seconds while the pixel pipeline grinds -- so the CP signal
 *    alone false-positives as a lockup. The quad counter keeps climbing while
 *    rendering, and (unlike the *_BUSY counters) stops if the pipeline truly
 *    wedges, so a genuine back-end hang is still detected.
 *
 * made_progress() bounds the number of progress retries, so a wedged GPU that
 * somehow keeps a counter creeping is still eventually recovered.
 */
static bool a2xx_progress(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	struct msm_cp_state cp_state = {
		.ib1_base = gpu_read(gpu, REG_AXXX_CP_IB1_BASE),
		.ib2_base = gpu_read(gpu, REG_AXXX_CP_IB2_BASE),
		.ib1_rem  = gpu_read(gpu, REG_AXXX_CP_IB1_BUFSZ),
		.ib2_rem  = gpu_read(gpu, REG_AXXX_CP_IB2_BUFSZ),
		.retired_work = gpu_read(gpu, REG_A2XX_RB_PERFCOUNTER0_LOW),
	};
	bool progress;

	progress = !!memcmp(&cp_state, &ring->last_cp_state, sizeof(cp_state));
	ring->last_cp_state = cp_state;

	return progress;
}

static int a2xx_pm_suspend(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);

	/*
	 * Idle the GPU and wait for all AXI transactions to complete.
	 * Without this the gfx3d_axi_clk branch clock cannot halt
	 * because the AXI bus is still servicing GPU requests.
	 */
	if (!a2xx_idle(gpu)) {
		dev_warn(gpu->dev->dev, "GPU didn't idle before suspend\n");

		/*
		 * GPU failed to idle normally. Halt the command processor
		 * and perform a soft reset to force the AXI interface into
		 * a quiescent state. This ensures the gfx3d_axi_clk branch
		 * clock can be disabled even if the GPU was stuck.
		 */
		gpu_write(gpu, REG_AXXX_CP_ME_CNTL, AXXX_CP_ME_CNTL_HALT);

		gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 1);
		gpu_read(gpu, REG_A2XX_RBBM_SOFT_RESET);
		gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0);

		/* Wait for reset to complete */
		udelay(100);

		/*
		 * The soft reset dropped CP/ME state. With two-tier runtime PM a
		 * light resume would otherwise skip hw_init and run an
		 * uninitialised GPU, so force a full re-init on the next resume.
		 */
		gpu->needs_hw_init = true;
	}

	/*
	 * Memory barrier to ensure all AXI transactions have completed
	 * before we clear the interconnect vote. This is critical on
	 * non-coherent platforms like MSM8660.
	 */
	wmb();

	/*
	 * Clear interconnect bandwidth vote before disabling clocks.
	 * This tells the bus fabric we no longer need memory bandwidth,
	 * allowing the AXI clock to halt properly. The legacy KGSL
	 * driver did this via msm_bus_scale_client_update_request(BW_INIT).
	 */
	if (a2xx_gpu->icc_path)
		icc_set_bw(a2xx_gpu->icc_path, 0, 0);

	return msm_gpu_pm_suspend(gpu);
}

static int a2xx_pm_resume(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);
	int ret;

	ret = msm_gpu_pm_resume(gpu);
	if (ret)
		return ret;

	if (a2xx_gpu->icc_path) {
		u32 bw = a2xx_icc_bw_for_freq(gpu->fast_rate);

		icc_set_bw(a2xx_gpu->icc_path, bw, bw);
	}

	return 0;
}

/*
 * Set GPU frequency and bandwidth via OPP framework.
 * This is called by devfreq when scaling frequency.
 */
static void a2xx_gpu_set_freq(struct msm_gpu *gpu, struct dev_pm_opp *opp,
			      bool suspended)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);
	unsigned long freq = dev_pm_opp_get_freq(opp);

	/*
	 * Don't change bandwidth when suspended - pm_resume will restore it.
	 * Only update our manual ICC path bandwidth to match the new frequency.
	 */
	if (suspended)
		return;

	/*
	 * The a2xx GFX3D core clock is NOT glitch-free across a rate change:
	 * switching it while the 3D pipe is rendering wedges the back-end
	 * (RB/PA/SC stuck busy, CP parked). Legacy KGSL idles the GPU before the
	 * switch (kgsl_pwrctrl_pwrlevel_change, idle_needed) -- "instability is
	 * caused on changing clock freq when the core is busy". We have no GMU
	 * and must not take gpu->lock here (gpu_set_freq runs under the devfreq
	 * lock; the submit path takes active_lock -> devfreq lock, so the reverse
	 * order would deadlock). So only switch when the 3D pipe is already idle:
	 * if it is busy right now, skip this change -- devfreq retries next tick
	 * and the clock holds until an idle window (the idle->active transition
	 * and inter-frame gaps provide them). This never switches mid-render and
	 * never blocks or spams the log (no ring-drain wait). The recover_worker()
	 * recovery-storm guard backs any rare idle-race miss.
	 */
	if (gpu_read(gpu, REG_A2XX_RBBM_STATUS) & A2XX_RBBM_BUSY_MASK)
		return;

	/*
	 * Set both avg and peak bandwidth proportional to frequency,
	 * matching the legacy webOS msm_bus grp3d_max_vectors pattern
	 * (ab = ib = clock × 8 bytes/cycle).
	 */
	if (a2xx_gpu->icc_path) {
		u32 bw = a2xx_icc_bw_for_freq(freq);

		icc_set_bw(a2xx_gpu->icc_path, bw, bw);
	}

	/*
	 * Step the GFX3D clock ONE OPP at a time toward the target, exactly like
	 * legacy KGSL (kgsl_pwrctrl_pwrlevel_change): "Don't shift by more than
	 * one level at a time to avoid glitches." A direct multi-OPP jump of the
	 * GFX3D clock (devfreq routinely asks for e.g. 27 <-> 266 MHz, ~10 OPPs
	 * at once) glitches the PLL/MN divider and wedges the marginal a220
	 * back-end. dev_pm_opp_set_rate() applies the matching voltage for each
	 * intermediate step. KGSL ran a2xx DVFS stably doing exactly this.
	 */
	{
		struct device *dev = &gpu->pdev->dev;
		unsigned long cur = clk_get_rate(gpu->core_clk);
		int guard = 0;

		while (cur != freq && ++guard <= 16) {
			struct dev_pm_opp *step;
			unsigned long sf;

			if (freq > cur) {
				sf = cur + 1;
				step = dev_pm_opp_find_freq_ceil(dev, &sf);
			} else {
				sf = cur - 1;
				step = dev_pm_opp_find_freq_floor(dev, &sf);
			}
			if (IS_ERR(step) || sf == cur)
				break;
			dev_pm_opp_put(step);
			dev_pm_opp_set_rate(dev, sf);
			cur = sf;
		}
	}
}

static struct msm_gpu *a2xx_gpu_init(struct drm_device *dev);

const struct adreno_gpu_funcs a2xx_gpu_funcs = {
	.base = {
		.get_param = adreno_get_param,
		.set_param = adreno_set_param,
		.hw_init = a2xx_hw_init,
		.pm_suspend = a2xx_pm_suspend,
		.pm_resume = a2xx_pm_resume,
		.recover = a2xx_recover,
		.submit = a2xx_submit,
		.active_ring = adreno_active_ring,
		.irq = a2xx_irq,
		.destroy = a2xx_destroy,
#if defined(CONFIG_DEBUG_FS) || defined(CONFIG_DEV_COREDUMP)
		.show = adreno_show,
#endif
		.gpu_busy = a2xx_gpu_busy,
		.gpu_state_get = a2xx_gpu_state_get,
		.gpu_state_put = adreno_gpu_state_put,
		.create_vm = a2xx_create_vm,
		.get_rptr = a2xx_get_rptr,
		.gpu_set_freq = a2xx_gpu_set_freq,
		.progress = a2xx_progress,
	},
	.init = a2xx_gpu_init,
};

static const struct msm_gpu_perfcntr perfcntrs[] = {
/* TODO */
};

static struct msm_gpu *a2xx_gpu_init(struct drm_device *dev)
{
	struct a2xx_gpu *a2xx_gpu = NULL;
	struct adreno_gpu *adreno_gpu;
	struct msm_gpu *gpu;
	struct msm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = priv->gpu_pdev;
	int ret;

	if (!pdev) {
		dev_err(dev->dev, "no a2xx device\n");
		ret = -ENXIO;
		goto fail;
	}

	a2xx_gpu = kzalloc(sizeof(*a2xx_gpu), GFP_KERNEL);
	if (!a2xx_gpu) {
		ret = -ENOMEM;
		goto fail;
	}

	adreno_gpu = &a2xx_gpu->base;
	gpu = &adreno_gpu->base;

	gpu->perfcntrs = perfcntrs;
	gpu->num_perfcntrs = ARRAY_SIZE(perfcntrs);

	ret = adreno_gpu_init(dev, pdev, adreno_gpu, &a2xx_gpu_funcs, 1);
	if (ret)
		goto fail;

	/*
	 * Two-tier runtime PM: keep the GFX3D rail and power domain up across
	 * runtime idle (clocks still gate), so a routine resume skips the
	 * a2xx_hw_init microcode reload whose MMIO burst can stall the shared
	 * MMSS AXI when it lands during an MDP display client-switch underrun.
	 * Matches legacy KGSL (SLEEP keeps the rail up during use; only SLUMBER
	 * on system suspend power-collapses). REQUIRES the GFX3D GDSC to carry
	 * GENPD_FLAG_RPM_ALWAYS_ON (set via the mmcc gdsc RPM_ALWAYS_ON flag) so
	 * the domain genuinely retains power across runtime idle; on imageon
	 * (no power domain) runtime idle only gates clocks, which retains state
	 * just the same, so skipping hw_init stays correct.
	 */
	gpu->retain_power_runtime = true;

	/*
	 * Enable KGSL-style binary boost: on idle->active transitions, clamp
	 * GPU min_freq directly to the MAX OPP for the duration of GPU work,
	 * mirroring legacy KGSL's CLK_ON->KGSL_MAX_FREQ behavior. Cleared on
	 * the next idle transition by msm_devfreq_idle_work().
	 *
	 * Required because simple_ondemand UNDERSHOOTS on a2xx: 0016's per-tile
	 * CACHE_FLUSH_TS+WFI drain makes the 3D pipe look idle to RBBM1_NRT_BUSY
	 * for most of the wall-time at low clock; the busy ratio never crosses
	 * the upthreshold to ramp up. Validated 2026-05-28: binner_test heavy
	 * stuck at 27MHz (0.94 fps); userspace pinning to 266MHz gave 5.5x
	 * speedup matching legacy KGSL (5.19 vs 5.45 fps).
	 *
	 * Module parameter `kgsl_boost` lets us A/B test this on hardware.
	 * Default 1 = on (current behaviour). Set 0 in a kernel cmdline or
	 * /sys/module/msm/parameters/a2xx_kgsl_boost (gated by the
	 * module_param below) to leave the simple_ondemand governor in
	 * charge and disable the boost. Hypothesis: dev_pm_qos_update_request
	 * in msm_devfreq_active() is asynchronous against the submit() call
	 * that follows it in msm_gpu.c -- the GPU starts processing commands
	 * at the OLD low clock, then the clock changes mid-execution, hanging
	 * the CP. Disabling the boost should eliminate the QSGRenderThread
	 * hangs we see on idle->active transitions.
	 */
	gpu->kgsl_style_boost = a2xx_kgsl_boost;
	WRITE_ONCE(a2xx_live_gpu, gpu); /* publish for sysfs runtime toggle */

	/*
	 * Optional GFX3D core reset used by a2xx_recover() (see there). Optional
	 * so platforms without it in DT (e.g. imageon) still probe; -EPROBE_DEFER
	 * and real errors propagate.
	 */
	a2xx_gpu->core_reset =
		devm_reset_control_get_optional_exclusive(&pdev->dev, "core");
	if (IS_ERR(a2xx_gpu->core_reset)) {
		ret = PTR_ERR(a2xx_gpu->core_reset);
		goto fail;
	}

	/*
	 * The a220 is slow: heavy fragment-bound frames (e.g. glmark2's
	 * multi-pass desktop blur) legitimately run ~1fps. With a2xx_progress()
	 * the hangcheck only fires when the CP stops advancing, so it is safe to
	 * tolerate such a still-rendering frame for much longer than the default
	 * before declaring a hang -- a genuinely stuck GPU makes no progress and
	 * is still caught in a single hangcheck period. Allow ~4s (the 250ms
	 * progress-halved hangcheck period x 16) so these frames complete instead
	 * of being needlessly recovered (which would drop the in-flight submit).
	 */
	gpu->hangcheck_progress_retries = 16;

	/* Get interconnect path for memory bandwidth voting */
	a2xx_gpu->icc_path = devm_of_icc_get(&pdev->dev, "gfx-mem");
	if (IS_ERR(a2xx_gpu->icc_path)) {
		ret = PTR_ERR(a2xx_gpu->icc_path);
		/* Allow -ENODATA, interconnect is optional for older DTs */
		if (ret != -ENODATA) {
			DRM_DEV_ERROR(dev->dev, "failed to get interconnect path: %d\n", ret);
			goto fail;
		}
		a2xx_gpu->icc_path = NULL;
	}

	/*
	 * Set initial interconnect bandwidth to max (avg = peak), matching
	 * legacy webOS grp3d_max_vectors. Adjusted during runtime PM and
	 * devfreq scaling.
	 */
	if (a2xx_gpu->icc_path) {
		u32 bw = a2xx_icc_bw_for_freq(gpu->fast_rate);

		icc_set_bw(a2xx_gpu->icc_path, bw, bw);
	}

	if (adreno_is_a20x(adreno_gpu))
		adreno_gpu->registers = a200_registers;
	else if (adreno_is_a225(adreno_gpu))
		adreno_gpu->registers = a225_registers;
	else
		adreno_gpu->registers = a220_registers;

	return gpu;

fail:
	if (a2xx_gpu)
		a2xx_destroy(&a2xx_gpu->base.base);

	return ERR_PTR(ret);
}
