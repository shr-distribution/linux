// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018 The Linux Foundation. All rights reserved. */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <asm/barrier.h>

#include "msm_drv.h"
#include "msm_mmu.h"

#include "adreno_gpu.h"
#include "a2xx_gpu.h"

#include "a2xx.xml.h"
#include "adreno_common.xml.h"

struct a2xx_gpummu {
	struct msm_mmu base;
	struct msm_gpu *gpu;
	dma_addr_t pt_base;
	uint32_t *table;
};
#define to_a2xx_gpummu(x) container_of(x, struct a2xx_gpummu, base)

#define GPUMMU_VA_START SZ_16M
#define GPUMMU_VA_RANGE (0xfff * SZ_64K)
#define GPUMMU_PAGE_SIZE SZ_4K
#define TABLE_SIZE (sizeof(uint32_t) * GPUMMU_VA_RANGE / GPUMMU_PAGE_SIZE)

static void a2xx_gpummu_detach(struct msm_mmu *mmu)
{
}

static int a2xx_gpummu_map(struct msm_mmu *mmu, uint64_t iova,
			   struct sg_table *sgt, size_t off, size_t len,
			   int prot)
{
	struct a2xx_gpummu *gpummu = to_a2xx_gpummu(mmu);
	unsigned idx = (iova - GPUMMU_VA_START) / GPUMMU_PAGE_SIZE;
	struct sg_dma_page_iter dma_iter;
	unsigned prot_bits = 0;
	bool gpu_suspended;
	int timeout;
	uint32_t status;

	WARN_ON(off != 0);

	if (prot & IOMMU_WRITE)
		prot_bits |= 1;
	if (prot & IOMMU_READ)
		prot_bits |= 2;

	for_each_sgtable_dma_page(sgt, &dma_iter, 0) {
		dma_addr_t addr = sg_page_iter_dma_address(&dma_iter);
		int i;

		for (i = 0; i < PAGE_SIZE; i += GPUMMU_PAGE_SIZE)
			gpummu->table[idx++] = (addr + i) | prot_bits;
	}

	/*
	 * Clean the page-table buffer to memory and drain the outer-cache
	 * write FIFO before the GPU walks the new entries (matches legacy
	 * KGSL; dma_sync alone does not necessarily flush the PL310 FIFO).
	 */
	dma_sync_single_for_device(mmu->dev, gpummu->pt_base, TABLE_SIZE,
				   DMA_TO_DEVICE);
	mb();
	dsb(sy);
#ifdef CONFIG_OUTER_CACHE
	outer_sync();
#endif

	/*
	 * Do not write MH_MMU_INVALIDATE while the GPU may be mid-frame on a
	 * previous submit: the direct CPU-side register write can race a
	 * running submit's in-flight TLB walks. Drain the ringbuffer and wait
	 * for RBBM idle first. If the GPU is runtime-suspended it is idle and
	 * its registers are off-limits; skip the wait (the TLB is empty and
	 * a2xx_hw_init() reloads the page-table base on resume).
	 */
	gpu_suspended = pm_runtime_status_suspended(&gpummu->gpu->pdev->dev);
	if (!gpu_suspended) {
		/* wait for ringbuffer drain (rptr == wptr) — up to 200ms */
		timeout = 200;
		while (timeout > 0) {
			uint32_t rptr = gpu_read(gpummu->gpu, REG_AXXX_CP_RB_RPTR);
			uint32_t wptr = gpu_read(gpummu->gpu, REG_AXXX_CP_RB_WPTR);
			if (rptr == wptr)
				break;
			usleep_range(500, 1000);
			timeout--;
		}
		/* wait for RBBM idle (HIRQ_PENDING bit 8 is OK) — up to 200ms */
		timeout = 200;
		while (timeout > 0) {
			status = gpu_read(gpummu->gpu, REG_A2XX_RBBM_STATUS);
			if ((status & ~0x100) == 0x010)
				break;
			usleep_range(500, 1000);
			timeout--;
		}
		if (timeout == 0)
			dev_warn_ratelimited(mmu->dev,
				"gpummu map: GPU busy after 200ms idle wait, status=0x%08x — issuing INVALIDATE anyway\n",
				status);
	}

	gpu_write(gpummu->gpu, REG_A2XX_MH_MMU_INVALIDATE,
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_ALL |
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_TC);

	mb();
	dsb(sy);

	return 0;
}

static int a2xx_gpummu_unmap(struct msm_mmu *mmu, uint64_t iova, size_t len)
{
	struct a2xx_gpummu *gpummu = to_a2xx_gpummu(mmu);
	unsigned idx = (iova - GPUMMU_VA_START) / GPUMMU_PAGE_SIZE;
	unsigned i;
	int timeout;
	uint32_t status;
	bool gpu_suspended;

	/*
	 * If the GPU is runtime-suspended it is guaranteed idle and its
	 * registers are off. The page table lives in system RAM, so just
	 * clear the entries; the TLB will be fresh when the GPU resumes.
	 */
	gpu_suspended = pm_runtime_status_suspended(&gpummu->gpu->pdev->dev);
	if (gpu_suspended) {
		for (i = 0; i < len / GPUMMU_PAGE_SIZE; i++, idx++)
			gpummu->table[idx] = 0;
		dma_sync_single_for_device(mmu->dev, gpummu->pt_base, TABLE_SIZE,
					   DMA_TO_DEVICE);
		return 0;
	}

	/*
	 * GPU is active — wait for it to go idle BEFORE clearing the page-
	 * table entries. Clearing a PTE the GPU is still walking makes the
	 * MMU translate to physical 0 and fault (MMU_PAGE_FAULT FAR=0), which
	 * re-fires every refresh and wedges the device. Drain the ringbuffer,
	 * then wait for RBBM idle (mirrors legacy KGSL kgsl_mmu_unmap).
	 */
	/* Step 1: ringbuffer drain (rptr == wptr) — up to 2s */
	timeout = 2000;
	while (timeout > 0) {
		uint32_t rptr = gpu_read(gpummu->gpu, REG_AXXX_CP_RB_RPTR);
		uint32_t wptr = gpu_read(gpummu->gpu, REG_AXXX_CP_RB_WPTR);
		if (rptr == wptr)
			break;
		usleep_range(500, 1000);
		timeout--;
	}
	if (timeout == 0)
		dev_warn_ratelimited(mmu->dev,
			"gpummu unmap: timeout waiting for ringbuffer drain\n");

	/* Step 2: RBBM idle (HIRQ_PENDING bit 8 is OK) — up to 3s */
	timeout = 3000;
	while (timeout > 0) {
		status = gpu_read(gpummu->gpu, REG_A2XX_RBBM_STATUS);
		if ((status & ~0x100) == 0x010)
			break;
		usleep_range(500, 1000);
		timeout--;
	}
	if (timeout == 0) {
		/*
		 * GPU still busy — clearing PTEs now would fault the live
		 * engine. Abort the unmap (the mapping is cleaned up on
		 * destroy / resume); leaking a stale PTE is far less harmful
		 * than a fault storm that hard-wedges the device.
		 */
		dev_err_ratelimited(mmu->dev,
			"gpummu unmap: GPU still busy after 3s, status=0x%08x — aborting unmap (iova=0x%llx len=%zx)\n",
			status, iova, len);
		return 0;
	}

	mb();
	dsb(sy);

	for (i = 0; i < len / GPUMMU_PAGE_SIZE; i++, idx++)
		gpummu->table[idx] = 0;

	dma_sync_single_for_device(mmu->dev, gpummu->pt_base, TABLE_SIZE,
				   DMA_TO_DEVICE);
	wmb();
	dsb(sy);

	gpu_write(gpummu->gpu, REG_A2XX_MH_MMU_INVALIDATE,
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_ALL |
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_TC);

	mb();
	dsb(sy);

	return 0;
}

static void a2xx_gpummu_destroy(struct msm_mmu *mmu)
{
	struct a2xx_gpummu *gpummu = to_a2xx_gpummu(mmu);

	dma_free_attrs(mmu->dev, TABLE_SIZE, gpummu->table, gpummu->pt_base,
		DMA_ATTR_FORCE_CONTIGUOUS);

	kfree(gpummu);
}

static const struct msm_mmu_funcs funcs = {
		.detach = a2xx_gpummu_detach,
		.map = a2xx_gpummu_map,
		.unmap = a2xx_gpummu_unmap,
		.destroy = a2xx_gpummu_destroy,
};

struct msm_mmu *a2xx_gpummu_new(struct device *dev, struct msm_gpu *gpu)
{
	struct a2xx_gpummu *gpummu;

	gpummu = kzalloc(sizeof(*gpummu), GFP_KERNEL);
	if (!gpummu)
		return ERR_PTR(-ENOMEM);

	gpummu->table = dma_alloc_attrs(dev, TABLE_SIZE + 32, &gpummu->pt_base,
		GFP_KERNEL | __GFP_ZERO, DMA_ATTR_FORCE_CONTIGUOUS);
	if (!gpummu->table) {
		kfree(gpummu);
		return ERR_PTR(-ENOMEM);
	}

	gpummu->gpu = gpu;
	msm_mmu_init(&gpummu->base, dev, &funcs, MSM_MMU_GPUMMU);

	return &gpummu->base;
}

void a2xx_gpummu_params(struct msm_mmu *mmu, dma_addr_t *pt_base,
		dma_addr_t *tran_error)
{
	dma_addr_t base = to_a2xx_gpummu(mmu)->pt_base;

	*pt_base = base;
	*tran_error = base + TABLE_SIZE; /* 32-byte aligned */
}
