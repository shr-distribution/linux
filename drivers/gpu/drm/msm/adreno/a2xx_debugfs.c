// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2024 The Linux Foundation. All rights reserved.
 *
 * A2XX GPU debugfs interface for debugging Adreno 200/220/225 GPUs.
 * Provides access to internal GPU state for debugging rendering issues.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>

#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>
#include <drm/drm_print.h>

#include "a2xx_gpu.h"
#include "a2xx_debug.h"

/*
 * CP (Command Processor) state dump
 * Shows ring buffer status, command queues, and ME state
 */
static void cp_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	int i;

	drm_printf(p, "CP State:\n");
	drm_printf(p, "  RB_BASE:     %08x\n", gpu_read(gpu, REG_AXXX_CP_RB_BASE));
	drm_printf(p, "  RB_CNTL:     %08x\n", gpu_read(gpu, REG_AXXX_CP_RB_CNTL));
	drm_printf(p, "  RB_RPTR:     %08x\n", gpu_read(gpu, REG_AXXX_CP_RB_RPTR));
	drm_printf(p, "  RB_WPTR:     %08x\n", gpu_read(gpu, REG_AXXX_CP_RB_WPTR));
	drm_printf(p, "  ME_CNTL:     %08x\n", gpu_read(gpu, REG_AXXX_CP_ME_CNTL));
	drm_printf(p, "  ME_STATUS:   %08x\n", gpu_read(gpu, REG_AXXX_CP_ME_STATUS));
	drm_printf(p, "  DEBUG:       %08x\n", gpu_read(gpu, REG_AXXX_CP_DEBUG));
	drm_printf(p, "  STAT:        %08x\n", gpu_read(gpu, REG_AXXX_CP_STAT));
	drm_printf(p, "  INT_STATUS:  %08x\n", gpu_read(gpu, REG_AXXX_CP_INT_STATUS));
	drm_printf(p, "  INT_CNTL:    %08x\n", gpu_read(gpu, REG_AXXX_CP_INT_CNTL));

	drm_printf(p, "\nCommand Stream Queues:\n");
	drm_printf(p, "  CSQ_RB_STAT:  %08x\n", gpu_read(gpu, REG_AXXX_CP_CSQ_RB_STAT));
	drm_printf(p, "  CSQ_IB1_STAT: %08x\n", gpu_read(gpu, REG_AXXX_CP_CSQ_IB1_STAT));
	drm_printf(p, "  CSQ_IB2_STAT: %08x\n", gpu_read(gpu, REG_AXXX_CP_CSQ_IB2_STAT));
	drm_printf(p, "  CSQ_AVAIL:    %08x\n", gpu_read(gpu, REG_AXXX_CP_CSQ_AVAIL));
	drm_printf(p, "  STQ_AVAIL:    %08x\n", gpu_read(gpu, REG_AXXX_CP_STQ_AVAIL));
	drm_printf(p, "  MEQ_AVAIL:    %08x\n", gpu_read(gpu, REG_AXXX_CP_MEQ_AVAIL));

	drm_printf(p, "\nQueue Stats:\n");
	drm_printf(p, "  STQ_ST_STAT:  %08x\n", gpu_read(gpu, REG_AXXX_CP_STQ_ST_STAT));
	drm_printf(p, "  MEQ_STAT:     %08x\n", gpu_read(gpu, REG_AXXX_CP_MEQ_STAT));
	drm_printf(p, "  MIU_TAG_STAT: %08x\n", gpu_read(gpu, REG_AXXX_CP_MIU_TAG_STAT));

	drm_printf(p, "\nScratch Registers:\n");
	for (i = 0; i < 8; i++) {
		drm_printf(p, "  SCRATCH_REG%d: %08x\n", i,
			   gpu_read(gpu, REG_AXXX_CP_SCRATCH_REG0 + i));
	}
}

/*
 * CP internal state debug
 * Uses CP_STATE_DEBUG_INDEX/DATA to read internal state
 */
static void cp_state_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	int i;

	drm_printf(p, "CP Internal State (via STATE_DEBUG):\n");

	for (i = 0; i < 32; i++) {
		gpu_write(gpu, REG_AXXX_CP_STATE_DEBUG_INDEX, i);
		drm_printf(p, "  [%02x]: %08x\n", i,
			   gpu_read(gpu, REG_AXXX_CP_STATE_DEBUG_DATA));
	}
}

/*
 * RBBM (Ring Buffer Bus Manager) state
 */
static void rbbm_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	uint32_t status = gpu_read(gpu, REG_A2XX_RBBM_STATUS);

	drm_printf(p, "RBBM State:\n");
	drm_printf(p, "  STATUS:        %08x\n", status);

	/* Decode status bits */
	drm_printf(p, "    GUI_ACTIVE:    %d\n", !!(status & A2XX_RBBM_STATUS_GUI_ACTIVE));
	drm_printf(p, "    RB_CNTX_BUSY:  %d\n", !!(status & A2XX_RBBM_STATUS_RB_CNTX_BUSY));
	drm_printf(p, "    SQ_CNTX0_BUSY: %d\n", !!(status & A2XX_RBBM_STATUS_SQ_CNTX0_BUSY));
	drm_printf(p, "    SQ_CNTX17_BUSY:%d\n", !!(status & A2XX_RBBM_STATUS_SQ_CNTX17_BUSY));
	drm_printf(p, "    VGT_BUSY:      %d\n", !!(status & A2XX_RBBM_STATUS_VGT_BUSY));
	drm_printf(p, "    PA_BUSY:       %d\n", !!(status & A2XX_RBBM_STATUS_PA_BUSY));
	drm_printf(p, "    SC_CNTX_BUSY:  %d\n", !!(status & A2XX_RBBM_STATUS_SC_CNTX_BUSY));
	drm_printf(p, "    TPC_BUSY:      %d\n", !!(status & A2XX_RBBM_STATUS_TPC_BUSY));
	drm_printf(p, "    SX_BUSY:       %d\n", !!(status & A2XX_RBBM_STATUS_SX_BUSY));
	drm_printf(p, "    MH_BUSY:       %d\n", !!(status & A2XX_RBBM_STATUS_MH_BUSY));
	drm_printf(p, "    MH_COHERENCY:  %d\n", !!(status & A2XX_RBBM_STATUS_MH_COHERENCY_BUSY));
	drm_printf(p, "    CP_NRT_BUSY:   %d\n", !!(status & A2XX_RBBM_STATUS_CP_NRT_BUSY));
	drm_printf(p, "    TC_BUSY:       %d\n", !!(status & A2XX_RBBM_STATUS_TC_BUSY));

	drm_printf(p, "\n  PM_OVERRIDE1:  %08x\n", gpu_read(gpu, REG_A2XX_RBBM_PM_OVERRIDE1));
	drm_printf(p, "  PM_OVERRIDE2:  %08x\n", gpu_read(gpu, REG_A2XX_RBBM_PM_OVERRIDE2));
	drm_printf(p, "  DEBUG:         %08x\n", gpu_read(gpu, REG_A2XX_RBBM_DEBUG));
	drm_printf(p, "  INT_CNTL:      %08x\n", gpu_read(gpu, REG_A2XX_RBBM_INT_CNTL));
	drm_printf(p, "  INT_STATUS:    %08x\n", gpu_read(gpu, REG_A2XX_RBBM_INT_STATUS));
	drm_printf(p, "  MASTER_INT:    %08x\n", gpu_read(gpu, REG_A2XX_MASTER_INT_SIGNAL));
}

/*
 * MH (Memory Hub) state including MMU
 */
static void mh_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	drm_printf(p, "MH (Memory Hub) State:\n");
	drm_printf(p, "  ARBITER_CONFIG:    %08x\n", gpu_read(gpu, REG_A2XX_MH_ARBITER_CONFIG));
	drm_printf(p, "  INTERRUPT_MASK:    %08x\n", gpu_read(gpu, REG_A2XX_MH_INTERRUPT_MASK));
	drm_printf(p, "  INTERRUPT_STATUS:  %08x\n", gpu_read(gpu, REG_A2XX_MH_INTERRUPT_STATUS));

	drm_printf(p, "\nMMU State:\n");
	drm_printf(p, "  MMU_CONFIG:        %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_CONFIG));
	drm_printf(p, "  MMU_VA_RANGE:      %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_VA_RANGE));
	drm_printf(p, "  MMU_PT_BASE:       %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_PT_BASE));
	drm_printf(p, "  MMU_PAGE_FAULT:    %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_PAGE_FAULT));
	drm_printf(p, "  MMU_TRAN_ERROR:    %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_TRAN_ERROR));
	drm_printf(p, "  MMU_MPU_BASE:      %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_MPU_BASE));
	drm_printf(p, "  MMU_MPU_END:       %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_MPU_END));
}

/*
 * SQ (Shader Sequencer) state
 */
static void sq_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	drm_printf(p, "SQ (Shader Sequencer) State:\n");
	drm_printf(p, "  PROGRAM_CNTL:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_PROGRAM_CNTL));
	drm_printf(p, "  CONTEXT_MISC:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_CONTEXT_MISC));
	drm_printf(p, "  INTERPOLATOR_CNTL: %08x\n", gpu_read(gpu, REG_A2XX_SQ_INTERPOLATOR_CNTL));
	drm_printf(p, "  VS_PROGRAM:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_VS_PROGRAM));
	drm_printf(p, "  PS_PROGRAM:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_PS_PROGRAM));
	drm_printf(p, "  GPR_MANAGEMENT:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_GPR_MANAGEMENT));
	drm_printf(p, "  INST_STORE_MGMT:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_INST_STORE_MANAGMENT));
	drm_printf(p, "  INT_CNTL:          %08x\n", gpu_read(gpu, REG_A2XX_SQ_INT_CNTL));
	drm_printf(p, "  INT_STATUS:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_INT_STATUS));

	drm_printf(p, "\nSQ Debug FSMs:\n");
	drm_printf(p, "  DEBUG_MISC:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_MISC));
	drm_printf(p, "  DEBUG_INPUT_FSM:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_INPUT_FSM));
	drm_printf(p, "  DEBUG_CONST_MGR:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_CONST_MGR_FSM));
	drm_printf(p, "  DEBUG_TP_FSM:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_TP_FSM));
	drm_printf(p, "  DEBUG_FSM_ALU_0:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_FSM_ALU_0));
	drm_printf(p, "  DEBUG_FSM_ALU_1:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_FSM_ALU_1));
	drm_printf(p, "  DEBUG_EXP_ALLOC:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_EXP_ALLOC));
	drm_printf(p, "  DEBUG_PTR_BUFF:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_PTR_BUFF));
	drm_printf(p, "  DEBUG_GPR_VTX:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_GPR_VTX));
	drm_printf(p, "  DEBUG_GPR_PIX:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_GPR_PIX));
}

/*
 * VGT (Vertex/Geometry/Tessellation) state
 */
static void vgt_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);

	drm_printf(p, "VGT (Vertex/Geometry) State:\n");
	drm_printf(p, "  MAX_VTX_INDX:      %08x\n", gpu_read(gpu, REG_A2XX_VGT_MAX_VTX_INDX));
	drm_printf(p, "  MIN_VTX_INDX:      %08x\n", gpu_read(gpu, REG_A2XX_VGT_MIN_VTX_INDX));
	drm_printf(p, "  INDX_OFFSET:       %08x\n", gpu_read(gpu, REG_A2XX_VGT_INDX_OFFSET));
	drm_printf(p, "  VERTEX_REUSE_CNTL: %08x\n", gpu_read(gpu, REG_A2XX_VGT_VERTEX_REUSE_BLOCK_CNTL));
	drm_printf(p, "  OUT_DEALLOC_CNTL:  %08x\n", gpu_read(gpu, REG_A2XX_VGT_OUT_DEALLOC_CNTL));
	drm_printf(p, "  ENHANCE:           %08x\n", gpu_read(gpu, REG_A2XX_VGT_ENHANCE));
	drm_printf(p, "  CURRENT_BIN_MIN:   %08x\n", gpu_read(gpu, REG_A2XX_VGT_CURRENT_BIN_ID_MIN));
	drm_printf(p, "  CURRENT_BIN_MAX:   %08x\n", gpu_read(gpu, REG_A2XX_VGT_CURRENT_BIN_ID_MAX));

	/* A22X specific */
	if (!adreno_is_a20x(adreno_gpu)) {
		drm_printf(p, "\nA22X VSC:\n");
		drm_printf(p, "  VSC_BIN_SIZE:      %08x\n", gpu_read(gpu, REG_A2XX_A220_VSC_BIN_SIZE));
	}
}

/*
 * TP (Texture Pipe) and TC (Texture Cache) state
 */
static void tp_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	drm_printf(p, "TP/TC (Texture Pipe/Cache) State:\n");
	drm_printf(p, "  TP0_CHICKEN:       %08x\n", gpu_read(gpu, REG_A2XX_TP0_CHICKEN));
	drm_printf(p, "  TC_CNTL_STATUS:    %08x\n", gpu_read(gpu, REG_A2XX_TC_CNTL_STATUS));

	drm_printf(p, "\nSQ Wrapping:\n");
	drm_printf(p, "  SQ_WRAPPING_0:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_WRAPPING_0));
	drm_printf(p, "  SQ_WRAPPING_1:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_WRAPPING_1));
}

/*
 * RB (Render Backend) state
 */
static void rb_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);

	drm_printf(p, "RB (Render Backend) State:\n");
	drm_printf(p, "  MODECONTROL:       %08x\n", gpu_read(gpu, REG_A2XX_RB_MODECONTROL));
	drm_printf(p, "  SURFACE_INFO:      %08x\n", gpu_read(gpu, REG_A2XX_RB_SURFACE_INFO));
	drm_printf(p, "  COLOR_INFO:        %08x\n", gpu_read(gpu, REG_A2XX_RB_COLOR_INFO));
	drm_printf(p, "  DEPTH_INFO:        %08x\n", gpu_read(gpu, REG_A2XX_RB_DEPTH_INFO));
	drm_printf(p, "  EDRAM_INFO:        %08x\n", gpu_read(gpu, REG_A2XX_RB_EDRAM_INFO));
	drm_printf(p, "  DEPTHCONTROL:      %08x\n", gpu_read(gpu, REG_A2XX_RB_DEPTHCONTROL));
	drm_printf(p, "  COLORCONTROL:      %08x\n", gpu_read(gpu, REG_A2XX_RB_COLORCONTROL));
	drm_printf(p, "  BLEND_CONTROL:     %08x\n", gpu_read(gpu, REG_A2XX_RB_BLEND_CONTROL));
	drm_printf(p, "  COLOR_MASK:        %08x\n", gpu_read(gpu, REG_A2XX_RB_COLOR_MASK));

	/* A22X specific registers */
	if (!adreno_is_a20x(adreno_gpu)) {
		drm_printf(p, "\nA22X Specific:\n");
		drm_printf(p, "  LRZ_VSC_CONTROL:   %08x\n",
			   gpu_read(gpu, REG_A2XX_A220_RB_LRZ_VSC_CONTROL));
		drm_printf(p, "  GRAS_CONTROL:      %08x\n",
			   gpu_read(gpu, REG_A2XX_A220_GRAS_CONTROL));
	}
}

/*
 * PA (Primitive Assembly) state
 */
static void pa_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	drm_printf(p, "PA (Primitive Assembly) State:\n");
	drm_printf(p, "  SC_WINDOW_OFFSET:  %08x\n", gpu_read(gpu, REG_A2XX_PA_SC_WINDOW_OFFSET));
	drm_printf(p, "  SC_AA_CONFIG:      %08x\n", gpu_read(gpu, REG_A2XX_PA_SC_AA_CONFIG));
	drm_printf(p, "  SC_AA_MASK:        %08x\n", gpu_read(gpu, REG_A2XX_PA_SC_AA_MASK));
	drm_printf(p, "  SU_SC_MODE_CNTL:   %08x\n", gpu_read(gpu, REG_A2XX_PA_SU_SC_MODE_CNTL));
	drm_printf(p, "  CL_VTE_CNTL:       %08x\n", gpu_read(gpu, REG_A2XX_PA_CL_VTE_CNTL));
	drm_printf(p, "  CL_CLIP_CNTL:      %08x\n", gpu_read(gpu, REG_A2XX_PA_CL_CLIP_CNTL));

	drm_printf(p, "\nPA Debug:\n");
	drm_printf(p, "  SU_DEBUG_CNTL:     %08x\n", gpu_read(gpu, REG_A2XX_PA_SU_DEBUG_CNTL));
	drm_printf(p, "  SU_DEBUG_DATA:     %08x\n", gpu_read(gpu, REG_A2XX_PA_SU_DEBUG_DATA));
}

/*
 * Full GPU state summary
 */
static void summary_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a2xx_gpu *a2xx_gpu = to_a2xx_gpu(adreno_gpu);
	uint32_t status = gpu_read(gpu, REG_A2XX_RBBM_STATUS);

	drm_printf(p, "===== A2XX GPU State Summary =====\n\n");

	drm_printf(p, "GPU: %s (%s)\n",
		   adreno_is_a20x(adreno_gpu) ? "Adreno 200 (Yamato)" :
		   adreno_is_a225(adreno_gpu) ? "Adreno 225" : "Adreno 220 (Leia)",
		   a2xx_gpu->protection_disabled ? "legacy firmware" : "modern firmware");

	drm_printf(p, "RBBM_STATUS: %08x (%s)\n", status,
		   (status & A2XX_RBBM_STATUS_GUI_ACTIVE) ? "BUSY" : "IDLE");

	drm_printf(p, "\nRing Buffer:\n");
	drm_printf(p, "  RPTR: %08x  WPTR: %08x\n",
		   gpu_read(gpu, REG_AXXX_CP_RB_RPTR),
		   gpu_read(gpu, REG_AXXX_CP_RB_WPTR));

	drm_printf(p, "\nInterrupts:\n");
	drm_printf(p, "  MASTER_INT: %08x\n", gpu_read(gpu, REG_A2XX_MASTER_INT_SIGNAL));
	drm_printf(p, "  CP_INT:     %08x\n", gpu_read(gpu, REG_AXXX_CP_INT_STATUS));
	drm_printf(p, "  RBBM_INT:   %08x\n", gpu_read(gpu, REG_A2XX_RBBM_INT_STATUS));
	drm_printf(p, "  MH_INT:     %08x\n", gpu_read(gpu, REG_A2XX_MH_INTERRUPT_STATUS));
	drm_printf(p, "  SQ_INT:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_INT_STATUS));

	drm_printf(p, "\nMMU:\n");
	drm_printf(p, "  PAGE_FAULT: %08x\n", gpu_read(gpu, REG_A2XX_MH_MMU_PAGE_FAULT));

	drm_printf(p, "\nShader/Interpolation (critical for faceted debug):\n");
	drm_printf(p, "  SQ_PROGRAM_CNTL:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_PROGRAM_CNTL));
	drm_printf(p, "  SQ_INTERPOLATOR_CNTL: %08x\n", gpu_read(gpu, REG_A2XX_SQ_INTERPOLATOR_CNTL));
	drm_printf(p, "  SQ_GPR_MANAGEMENT:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_GPR_MANAGEMENT));
	drm_printf(p, "  SQ_FLOW_CONTROL:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_FLOW_CONTROL));
	drm_printf(p, "  SQ_INST_STORE_MGMT:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_INST_STORE_MANAGMENT));
	drm_printf(p, "  SQ_VS_PROGRAM:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_VS_PROGRAM));
	drm_printf(p, "  SQ_PS_PROGRAM:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_PS_PROGRAM));
	drm_printf(p, "  SQ_PS_CONST:          %08x\n", gpu_read(gpu, REG_A2XX_SQ_PS_CONST));
	drm_printf(p, "  SQ_WRAPPING_0:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_WRAPPING_0));
	drm_printf(p, "  SQ_WRAPPING_1:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_WRAPPING_1));

	drm_printf(p, "\nVGT (Vertex Grouper):\n");
	drm_printf(p, "  VGT_VERTEX_REUSE:     %08x\n", gpu_read(gpu, REG_A2XX_VGT_VERTEX_REUSE_BLOCK_CNTL));
	drm_printf(p, "  VGT_OUT_DEALLOC_CNTL: %08x\n", gpu_read(gpu, REG_A2XX_VGT_OUT_DEALLOC_CNTL));
	drm_printf(p, "  VGT_ENHANCE:          %08x\n", gpu_read(gpu, REG_A2XX_VGT_ENHANCE));

	drm_printf(p, "\nTexture/Cache:\n");
	drm_printf(p, "  TP0_CHICKEN:          %08x\n", gpu_read(gpu, REG_A2XX_TP0_CHICKEN));
	drm_printf(p, "  TC_CNTL_STATUS:       %08x\n", gpu_read(gpu, REG_A2XX_TC_CNTL_STATUS));

	drm_printf(p, "\nGMEM/Clipping:\n");
	drm_printf(p, "  RB_MODECONTROL:       %08x\n", gpu_read(gpu, REG_A2XX_RB_MODECONTROL));
	drm_printf(p, "  PA_CL_CLIP_CNTL:      %08x\n", gpu_read(gpu, REG_A2XX_PA_CL_CLIP_CNTL));
	drm_printf(p, "  PA_SU_SC_MODE_CNTL:   %08x\n", gpu_read(gpu, REG_A2XX_PA_SU_SC_MODE_CNTL));

	drm_printf(p, "\nSQ Debug (shader sequencer internal state):\n");
	drm_printf(p, "  SQ_DEBUG_MISC:        %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_MISC));
	drm_printf(p, "  SQ_DEBUG_INPUT_FSM:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_INPUT_FSM));
	drm_printf(p, "  SQ_DEBUG_CONST_MGR:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_CONST_MGR_FSM));
	drm_printf(p, "  SQ_DEBUG_TP_FSM:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_TP_FSM));
	drm_printf(p, "  SQ_DEBUG_FSM_ALU_0:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_FSM_ALU_0));
	drm_printf(p, "  SQ_DEBUG_FSM_ALU_1:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_FSM_ALU_1));
	drm_printf(p, "  SQ_DEBUG_EXP_ALLOC:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_EXP_ALLOC));
	drm_printf(p, "  SQ_DEBUG_PTR_BUFF:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_PTR_BUFF));
	drm_printf(p, "  SQ_DEBUG_GPR_VTX:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_GPR_VTX));
	drm_printf(p, "  SQ_DEBUG_GPR_PIX:     %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_GPR_PIX));
	drm_printf(p, "  SQ_DEBUG_TB_STATUS:   %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_TB_STATUS_SEL));
	drm_printf(p, "  SQ_DEBUG_VTX_TB_0:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_VTX_TB_0));
	drm_printf(p, "  SQ_DEBUG_VTX_TB_1:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_VTX_TB_1));
	drm_printf(p, "  SQ_DEBUG_PIX_TB_0:    %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_PIX_TB_0));
	drm_printf(p, "  SQ_DEBUG_MISC_0:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_MISC_0));
	drm_printf(p, "  SQ_DEBUG_MISC_1:      %08x\n", gpu_read(gpu, REG_A2XX_SQ_DEBUG_MISC_1));

	drm_printf(p, "\nOther Debug:\n");
	drm_printf(p, "  PA_SU_DEBUG_CNTL:     %08x\n", gpu_read(gpu, REG_A2XX_PA_SU_DEBUG_CNTL));
	drm_printf(p, "  PA_SU_DEBUG_DATA:     %08x\n", gpu_read(gpu, REG_A2XX_PA_SU_DEBUG_DATA));
	drm_printf(p, "  PC_DEBUG_CNTL:        %08x\n", gpu_read(gpu, REG_A2XX_PC_DEBUG_CNTL));
	drm_printf(p, "  PC_DEBUG_DATA:        %08x\n", gpu_read(gpu, REG_A2XX_PC_DEBUG_DATA));
	drm_printf(p, "  RB_DEBUG_CNTL:        %08x\n", gpu_read(gpu, REG_A2XX_RB_DEBUG_CNTL));
	drm_printf(p, "  RB_DEBUG_DATA:        %08x\n", gpu_read(gpu, REG_A2XX_RB_DEBUG_DATA));
	drm_printf(p, "  GRAS_DEBUG_CNTL:      %08x\n", gpu_read(gpu, REG_A2XX_GRAS_DEBUG_CNTL));
	drm_printf(p, "  GRAS_DEBUG_DATA:      %08x\n", gpu_read(gpu, REG_A2XX_GRAS_DEBUG_DATA));

	/* A22X specific */
	if (!adreno_is_a20x(adreno_gpu)) {
		drm_printf(p, "\nA22X Specific:\n");
		drm_printf(p, "  VSC_BIN_SIZE:         %08x\n", gpu_read(gpu, REG_A2XX_A220_VSC_BIN_SIZE));
		drm_printf(p, "  LRZ_VSC_CONTROL:      %08x\n", gpu_read(gpu, REG_A2XX_A220_RB_LRZ_VSC_CONTROL));
		drm_printf(p, "  GRAS_CONTROL:         %08x\n", gpu_read(gpu, REG_A2XX_A220_GRAS_CONTROL));
	}

	drm_printf(p, "\n==================================\n");
}

static int show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_printer p = drm_seq_file_printer(m);
	struct msm_gpu *gpu = priv->gpu;
	void (*print_fn)(struct msm_gpu *gpu, struct drm_printer *p) =
		node->info_ent->data;

	if (!gpu)
		return -ENODEV;

	pm_runtime_get_sync(&gpu->pdev->dev);
	print_fn(gpu, &p);
	pm_runtime_put_sync(&gpu->pdev->dev);

	return 0;
}

#define ENT(n) { .name = #n, .show = show, .data = n ##_print }
static struct drm_info_list a2xx_debugfs_list[] = {
	ENT(summary),
	ENT(cp),
	ENT(cp_state),
	ENT(rbbm),
	ENT(mh),
	ENT(sq),
	ENT(vgt),
	ENT(tp),
	ENT(rb),
	ENT(pa),
};

/*
 * Reset GPU - write any value to trigger
 */
static int reset_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (!gpu)
		return -ENODEV;

	dev_info(gpu->dev->dev, "A2XX: Manual GPU reset triggered via debugfs\n");

	mutex_lock(&gpu->lock);

	gpu->needs_hw_init = true;

	pm_runtime_get_sync(&gpu->pdev->dev);
	gpu->funcs->recover(gpu);
	pm_runtime_put_sync(&gpu->pdev->dev);

	mutex_unlock(&gpu->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(reset_fops, NULL, reset_set, "%llu\n");

/*
 * Force a full GFX3D GDSC power-collapse cycle.
 *
 * Validates the "GDSC stays warm, SQ slot SRAM persists" theory by
 * driving the GPU's parent genpd through a real power-off / power-on
 * sequence (which the qcom GDSC driver registers with gov=NULL,
 * preventing the framework from doing this on idle).
 *
 * Sequence:
 *   1. pm_runtime_force_suspend(gpu_dev)  - releases device-side state
 *      and drops the genpd ref. Without a governor the parent genpd
 *      stays "on" at this point.
 *   2. genpd->power_off(genpd)            - calls gdsc_disable, which
 *      for LEGACY_FOOTSWITCH | SW_RESET on MSM8660 asserts AHB reset,
 *      clamps I/O, and clears the rail-enable bit. Silicon is now
 *      electrically off; SRAM contents are lost.
 *   3. mdelay(5)                          - ensure rail fully drains
 *      before we re-enable.
 *   4. genpd->power_on(genpd)             - calls gdsc_enable; ramps
 *      the rail back up, deasserts reset, unclamps. ~7us.
 *   5. pm_runtime_force_resume(gpu_dev)   - kernel runs the full a2xx
 *      runtime-resume callback chain including a2xx_hw_init, which
 *      re-initializes MMU, reloads PM4/PFP firmware, runs ME_INIT.
 *
 * Usage:
 *   echo 1 > /sys/kernel/debug/dri/0/force_collapse
 *
 * Expected outcome:
 *   - period-8 hash cycle collapses to a single deterministic hash
 *     (channel means matching the webOS reference render).
 *   - if so: the cycle is GDSC-persistent SQ slot SRAM, fix path is
 *     either Option A (kernel governor for gfx3d_gdsc) or Option B
 *     (driver-side a2xx_pm_runtime_suspend explicit genpd toggle).
 */
static int force_collapse_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;
	struct device *gdev;
	struct generic_pm_domain *genpd;
	int ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (!gpu)
		return -ENODEV;
	if (!val)
		return 0;

	gdev = &gpu->pdev->dev;
	if (!gdev->pm_domain) {
		dev_err(gdev, "force_collapse: no pm_domain on adreno device\n");
		return -ENODEV;
	}
	genpd = pd_to_genpd(gdev->pm_domain);
	if (!genpd || !genpd->power_off || !genpd->power_on) {
		dev_err(gdev, "force_collapse: genpd has no power callbacks\n");
		return -ENODEV;
	}

	dev_info(gdev, "force_collapse: cycling %s genpd via runtime suspend/resume\n",
		 genpd->name);

	mutex_lock(&gpu->lock);
	gpu->needs_hw_init = true;
	mutex_unlock(&gpu->lock);

	ret = pm_runtime_force_suspend(gdev);
	if (ret) {
		dev_err(gdev, "force_collapse: pm_runtime_force_suspend: %d\n", ret);
		return ret;
	}

	dev_info(gdev, "force_collapse: power_off (gdsc_disable)\n");
	ret = genpd->power_off(genpd);
	if (ret)
		dev_warn(gdev, "force_collapse: power_off returned %d\n", ret);

	mdelay(5);

	dev_info(gdev, "force_collapse: power_on (gdsc_enable)\n");
	ret = genpd->power_on(genpd);
	if (ret)
		dev_warn(gdev, "force_collapse: power_on returned %d\n", ret);

	ret = pm_runtime_force_resume(gdev);
	if (ret) {
		dev_err(gdev, "force_collapse: pm_runtime_force_resume: %d\n", ret);
		return ret;
	}

	dev_info(gdev, "force_collapse: complete\n");
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(force_collapse_fops, NULL, force_collapse_set, "%llu\n");

/*
 * On-demand MMIO register read.
 *
 * Lets userspace probe any A2XX MMIO offset (in word units, matching
 * the freedreno/registers/adreno/a2xx.xml convention). Useful for
 * register-state-leak debugging when the static hangdump register
 * list (a220_registers[]) doesn't cover the offset of interest.
 *
 * Usage:
 *   echo 0x4000 > /sys/kernel/debug/dri/0/regrw_offset
 *   cat /sys/kernel/debug/dri/0/regrw_value
 *     -> 0xVVVVVVVV
 *
 * The offset is the WORD offset (a2xx.xml convention). Internally
 * gpu_read() takes the same convention, so we pass it through.
 *
 * No write side - read-only register inspection. CAP_SYS_ADMIN
 * required because arbitrary MMIO read can hit reserved/secure
 * regions on other variants; A2XX is generally permissive but the
 * upstream pattern is to gate.
 */
static u32 a2xx_regrw_offset;

static int regrw_offset_set(void *data, u64 val)
{
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	a2xx_regrw_offset = (u32)val;
	return 0;
}

static int regrw_offset_get(void *data, u64 *val)
{
	*val = a2xx_regrw_offset;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(regrw_offset_fops,
			 regrw_offset_get, regrw_offset_set, "0x%08llx\n");

static int regrw_value_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (!gpu)
		return -ENODEV;

	pm_runtime_get_sync(&gpu->pdev->dev);
	*val = gpu_read(gpu, a2xx_regrw_offset);
	pm_runtime_put_sync(&gpu->pdev->dev);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(regrw_value_fops,
			 regrw_value_get, NULL, "0x%08llx\n");

void a2xx_debugfs_init(struct msm_gpu *gpu, struct drm_minor *minor)
{
	struct drm_device *dev;

	if (!minor)
		return;

	dev = minor->dev;

	drm_debugfs_create_files(a2xx_debugfs_list,
				 ARRAY_SIZE(a2xx_debugfs_list),
				 minor->debugfs_root, minor);

	debugfs_create_file_unsafe("reset", 0200, minor->debugfs_root, dev,
				   &reset_fops);

	debugfs_create_file_unsafe("force_collapse", 0200, minor->debugfs_root,
				   dev, &force_collapse_fops);

	/* On-demand MMIO register read: write offset to regrw_offset,
	 * then read regrw_value to get the live register value. Useful
	 * for inspecting offsets the hangdump register-list doesn't cover.
	 */
	debugfs_create_file_unsafe("regrw_offset", 0600, minor->debugfs_root,
				   dev, &regrw_offset_fops);
	debugfs_create_file_unsafe("regrw_value", 0400, minor->debugfs_root,
				   dev, &regrw_value_fops);

	/* Initialize debug/test infrastructure */
	a2xx_debug_init(minor);

	dev_info(gpu->dev->dev,
		 "A2XX debugfs: /sys/kernel/debug/dri/%d/{summary,cp,rbbm,mh,sq,rb,pa,reset,force_collapse,regrw_offset,regrw_value,coherency_test,timing_test}\n",
		 minor->index);
}
