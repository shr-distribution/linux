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

	/*
	 * Shader-constant shadow memory for cross-context sanitization.
	 *
	 * KGSL on legacy kernels mirrored the per-context constant SRAMs
	 * here and used PM4_LOAD_CONSTANT_CONTEXT to bulk-restore them
	 * at every context switch.
	 *
	 * Mainline doesn't have a per-context shadow concept, so we
	 * allocate a single shared all-zeros shadow buffer and use it
	 * to wipe Mesa's user-controllable constant range
	 * (vec4 slots 32..511, dwords 0x80..0x7FF) at every cross-DRM-
	 * client boundary. SoC-reserved slots 0..31 are skipped via
	 * the LOAD_CONSTANT_CONTEXT offset operand.
	 *
	 * Address alignment is critical: the LOAD_CONSTANT_CONTEXT
	 * payload is masked to 0xFFFFE000 (8 KB granularity). We
	 * allocate 16 KB and pick an 8 KB-aligned IOVA inside it so
	 * the requirement is satisfied regardless of the underlying
	 * IOMMU/page allocator's alignment behavior.
	 */
	struct drm_gem_object *shadow_bo;
	uint64_t shadow_iova;       /* 8 KB-aligned ALU shadow IOVA */
	uint64_t shadow_tex_iova;   /* 8 KB-aligned TEX shadow IOVA (=shadow_iova+8K) */
	void *shadow_vaddr;
};
#define to_a2xx_gpu(x) container_of(x, struct a2xx_gpu, base)

struct msm_mmu *a2xx_gpummu_new(struct device *dev, struct msm_gpu *gpu);
void a2xx_gpummu_params(struct msm_mmu *mmu, dma_addr_t *pt_base,
		dma_addr_t *tran_error);
void a2xx_gpummu_debug_fault(struct msm_mmu *mmu, uint32_t fault_addr);
void *a2xx_gpummu_iova_to_kvirt(struct msm_mmu *mmu, uint64_t iova);

/* debugfs */
#if defined(CONFIG_DEBUG_FS)
void a2xx_debugfs_init(struct msm_gpu *gpu, struct drm_minor *minor);
#else
static inline void a2xx_debugfs_init(struct msm_gpu *gpu,
				     struct drm_minor *minor) {}
#endif

#endif /* __A2XX_GPU_H__ */
