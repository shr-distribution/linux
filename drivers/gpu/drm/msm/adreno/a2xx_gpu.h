// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018 The Linux Foundation. All rights reserved. */

#ifndef __A2XX_GPU_H__
#define __A2XX_GPU_H__

#include "adreno_gpu.h"

struct icc_path;

/* arrg, somehow fb.h is getting pulled in: */
#undef ROP_COPY
#undef ROP_XOR

#include "a2xx.xml.h"

struct a2xx_gpu {
	struct adreno_gpu base;
	bool pm_enabled;
	bool protection_disabled;
	/*
	 * Set once the first FULL RBBM_SOFT_RESET (0xffffffff) has been done.
	 * KGSL resets all blocks only on the first init; every subsequent a22x
	 * (re)init -- including resume from GDSC power-collapse -- resets only
	 * the CP block (0x1). See a2xx_hw_init().
	 */
	bool soft_reset_done;
	struct icc_path *icc_path;
};
#define to_a2xx_gpu(x) container_of(x, struct a2xx_gpu, base)

struct msm_mmu *a2xx_gpummu_new(struct device *dev, struct msm_gpu *gpu);
void a2xx_gpummu_params(struct msm_mmu *mmu, dma_addr_t *pt_base,
		dma_addr_t *tran_error);

#endif /* __A2XX_GPU_H__ */
