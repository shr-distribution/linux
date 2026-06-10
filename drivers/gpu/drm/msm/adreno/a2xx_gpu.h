// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018 The Linux Foundation. All rights reserved. */

#ifndef __A2XX_GPU_H__
#define __A2XX_GPU_H__

#include "adreno_gpu.h"

struct icc_path;
struct reset_control;

/* arrg, somehow fb.h is getting pulled in: */
#undef ROP_COPY
#undef ROP_XOR

#include "a2xx.xml.h"

struct a2xx_gpu {
	struct adreno_gpu base;
	bool pm_enabled;
	bool protection_disabled;
	struct icc_path *icc_path;
	/*
	 * GFX3D core reset (mmcc GFX3D_RESET). Pulsed by a2xx_recover() to clear
	 * a hung SQ / parameter-cache when two-tier runtime PM keeps the gfx3d
	 * GDSC powered (so the old power-collapse no longer re-fires the reset).
	 * NULL on platforms whose DT does not provide the "core" reset.
	 */
	struct reset_control *core_reset;
};
#define to_a2xx_gpu(x) container_of(x, struct a2xx_gpu, base)

extern const struct adreno_gpu_funcs a2xx_gpu_funcs;

struct msm_mmu *a2xx_gpummu_new(struct device *dev, struct msm_gpu *gpu);
void a2xx_gpummu_params(struct msm_mmu *mmu, dma_addr_t *pt_base,
		dma_addr_t *tran_error);

#endif /* __A2XX_GPU_H__ */
