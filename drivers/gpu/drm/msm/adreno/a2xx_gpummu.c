// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018 The Linux Foundation. All rights reserved. */

#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include "msm_drv.h"
#include "msm_mmu.h"

#include "adreno_gpu.h"
#include "a2xx_gpu.h"

#include "a2xx.xml.h"

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

	WARN_ON(off != 0);

	dev_dbg(mmu->dev, "gpummu map: iova=%llx len=%zx prot=%x idx=%u\n",
		iova, len, prot, idx);

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

	/* Sync page table to device for non-coherent platforms (e.g. MSM8660) */
	dma_sync_single_for_device(mmu->dev, gpummu->pt_base, TABLE_SIZE,
				   DMA_TO_DEVICE);

	/* Ensure DMA sync completes before invalidating TLB */
	wmb();

	/* we can improve by deferring flush for multiple map() */
	gpu_write(gpummu->gpu, REG_A2XX_MH_MMU_INVALIDATE,
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_ALL |
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_TC);

	/* Wait for Memory Hub to process TLB invalidation */
	mb();

	return 0;
}

static int a2xx_gpummu_unmap(struct msm_mmu *mmu, uint64_t iova, size_t len)
{
	struct a2xx_gpummu *gpummu = to_a2xx_gpummu(mmu);
	unsigned idx = (iova - GPUMMU_VA_START) / GPUMMU_PAGE_SIZE;
	unsigned i;
	int timeout = 10000; /* 10ms timeout */

	dev_dbg(mmu->dev, "gpummu unmap: iova=%llx len=%zx idx=%u\n",
		iova, len, idx);

	/*
	 * Wait for Memory Hub to complete outstanding transactions before
	 * clearing page table entries. This prevents page faults when the
	 * GPU is still accessing memory that we're about to unmap.
	 *
	 * The webOS kgsl_yamato driver calls kgsl_yamato_idle() before any
	 * page table operations, waiting for RBBM_STATUS == 0x110 (complete
	 * idle). We can't easily do a full GPU idle here, so we just wait
	 * for MH (Memory Hub) to be idle which should ensure no outstanding
	 * memory transactions.
	 */
	while (timeout > 0) {
		uint32_t status = gpu_read(gpummu->gpu, REG_A2XX_RBBM_STATUS);
		if (!(status & (A2XX_RBBM_STATUS_MH_BUSY |
				A2XX_RBBM_STATUS_MH_COHERENCY_BUSY)))
			break;
		udelay(1);
		timeout--;
	}
	if (timeout == 0)
		dev_warn_once(mmu->dev, "gpummu unmap: timeout waiting for MH idle\n");

	for (i = 0; i < len / GPUMMU_PAGE_SIZE; i++, idx++)
		gpummu->table[idx] = 0;

	/* Sync page table to device for non-coherent platforms (e.g. MSM8660) */
	dma_sync_single_for_device(mmu->dev, gpummu->pt_base, TABLE_SIZE,
				   DMA_TO_DEVICE);

	/* Ensure DMA sync completes before invalidating TLB */
	wmb();

	gpu_write(gpummu->gpu, REG_A2XX_MH_MMU_INVALIDATE,
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_ALL |
		A2XX_MH_MMU_INVALIDATE_INVALIDATE_TC);

	/* Wait for Memory Hub to process TLB invalidation */
	mb();

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

void a2xx_gpummu_debug_fault(struct msm_mmu *mmu, uint32_t fault_addr)
{
	struct a2xx_gpummu *gpummu = to_a2xx_gpummu(mmu);
	uint32_t pte;
	unsigned idx;

	/* Check if address is in valid range */
	if (fault_addr < GPUMMU_VA_START ||
	    fault_addr >= GPUMMU_VA_START + GPUMMU_VA_RANGE) {
		dev_err(mmu->dev, "GPUMMU fault addr 0x%08x outside VA range [0x%x-0x%lx]\n",
			fault_addr, GPUMMU_VA_START,
			(unsigned long)(GPUMMU_VA_START + GPUMMU_VA_RANGE));
		return;
	}

	idx = (fault_addr - GPUMMU_VA_START) / GPUMMU_PAGE_SIZE;
	pte = gpummu->table[idx];

	dev_err(mmu->dev, "GPUMMU fault: addr=0x%08x idx=%u pte=0x%08x (phys=0x%08x prot=%s%s)\n",
		fault_addr, idx, pte,
		pte & ~3,
		(pte & 2) ? "R" : "",
		(pte & 1) ? "W" : "");

	/* Also dump nearby entries for context */
	if (idx > 0)
		dev_err(mmu->dev, "  pte[%u-1]=0x%08x\n", idx, gpummu->table[idx-1]);
	dev_err(mmu->dev, "  pte[%u]=0x%08x\n", idx, gpummu->table[idx]);
	if (idx < (GPUMMU_VA_RANGE / GPUMMU_PAGE_SIZE) - 1)
		dev_err(mmu->dev, "  pte[%u+1]=0x%08x\n", idx, gpummu->table[idx+1]);
}
