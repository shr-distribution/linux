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
	struct icc_path *icc_path;
};
#define to_a2xx_gpu(x) container_of(x, struct a2xx_gpu, base)

struct msm_mmu *a2xx_gpummu_new(struct device *dev, struct msm_gpu *gpu);
void a2xx_gpummu_params(struct msm_mmu *mmu, dma_addr_t *pt_base,
		dma_addr_t *tran_error);
void a2xx_gpummu_debug_fault(struct msm_mmu *mmu, uint32_t fault_addr);

/* debugfs */
#if defined(CONFIG_DEBUG_FS)
void a2xx_debugfs_init(struct msm_gpu *gpu, struct drm_minor *minor);
#else
static inline void a2xx_debugfs_init(struct msm_gpu *gpu,
				     struct drm_minor *minor) {}
#endif

#endif /* __A2XX_GPU_H__ */
