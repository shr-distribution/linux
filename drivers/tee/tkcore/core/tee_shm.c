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

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/dma-buf.h>
#include <linux/hugetlb.h>
#include <linux/version.h>
#include <linux/anon_inodes.h>

#include <linux/sched.h>
#include <linux/mm.h>

#include "tee_core_priv.h"
#include "tee_shm.h"

int __weak sg_nents(struct scatterlist *sg)
{
	int nents;

	for (nents = 0; sg; sg = sg_next(sg))
		nents++;
	return nents;
}

struct tee_shm_attach {
	struct sg_table sgt;
	enum dma_data_direction dir;
	bool is_mapped;
};

static struct tee_shm *tee_shm_alloc_static(struct tee *tee, size_t size,
		uint32_t flags)
{
	struct tee_shm *shm;
	unsigned long pfn;
	unsigned int nr_pages;
	struct page *page;
	int ret;

	shm = tee->ops->alloc(tee, size, flags);
	if (IS_ERR_OR_NULL(shm)) {
		pr_err("allocation failed (s=%d,flags=0x%08x) err=%ld\n",
			(int) size, flags, PTR_ERR(shm));
		goto exit;
	}

	pfn = shm->static_shm.paddr >> PAGE_SHIFT;
	page = pfn_to_page(pfn);
	if (IS_ERR_OR_NULL(page)) {
		pr_err("pfn_to_page(%lx) failed\n", pfn);
		tee->ops->free(shm);
		return (struct tee_shm *) page;
	}

	/* Only one page of contiguous physical memory */
	nr_pages = 1;

	ret = sg_alloc_table_from_pages(&shm->static_shm.sgt, &page,
					nr_pages, 0,
					nr_pages * PAGE_SIZE, GFP_KERNEL);
	if (ret) {
		pr_err("sg_alloc_table_from_pages() failed\n");
		tee->ops->free(shm);
		shm = ERR_PTR(ret);
	}

exit:
	return shm;
}

struct tee_shm *tkcore_alloc_shm(struct tee *tee, size_t size, uint32_t flags)
{
	struct tee_shm *shm;

	shm = tee_shm_alloc_static(tee, size, flags);
	if (IS_ERR_OR_NULL(shm))
		goto exit;

	shm->tee = tee;

exit:
	return shm;
}

void tkcore_shm_free(struct tee_shm *shm)
{
	struct tee *tee;

	if (IS_ERR_OR_NULL(shm))
		return;

	tee = shm->tee;

	if (tee == NULL) {
		pr_warn("tkcoredrv: %s(): NULL tee\n",
			__func__);
		return;
	}
	if (shm->tee == NULL) {
		pr_warn("tkcoredrv: %s(): invalid shm\n", __func__);
		return;
	}

	sg_free_table(&shm->static_shm.sgt);
	shm->tee->ops->free(shm);
}

static int __tee_shm_attach_dma_buf(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attach)
{
	struct tee_shm_attach *tee_shm_attach;
	struct tee_shm *shm;
	struct tee *tee;

	shm = dmabuf->priv;
	tee = shm->tee;


	tee_shm_attach = devm_kzalloc(_DEV(tee),
				sizeof(*tee_shm_attach), GFP_KERNEL);
	if (!tee_shm_attach)
		return -ENOMEM;

	tee_shm_attach->dir = DMA_NONE;
	attach->priv = tee_shm_attach;

	return 0;
}

static void __tee_shm_detach_dma_buf(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attach)
{
	struct tee_shm_attach *tee_shm_attach = attach->priv;
	struct sg_table *sgt;
	struct tee_shm *shm;
	struct tee *tee;

	shm = dmabuf->priv;
	tee = shm->tee;


	if (!tee_shm_attach) {
		pr_err("No shm attached with this dmabuf context");
		return;
	}

	sgt = &tee_shm_attach->sgt;

	if (tee_shm_attach->dir != DMA_NONE)
		dma_unmap_sg(attach->dev, sgt->sgl, sgt->nents,
			tee_shm_attach->dir);

	sg_free_table(sgt);
	devm_kfree(_DEV(tee), tee_shm_attach);
	attach->priv = NULL;
}

static struct sg_table *__tee_shm_dma_buf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct tee_shm_attach *tee_shm_attach = attach->priv;
	struct tee_shm *tee_shm = attach->dmabuf->priv;
	struct sg_table *sgt = NULL;
	struct scatterlist *rd, *wr;
	unsigned int i;
	int nents, ret;
	struct tee *tee;

	tee = tee_shm->tee;


	/* just return current sgt if already requested. */
	if (tee_shm_attach->dir == dir && tee_shm_attach->is_mapped)
		return &tee_shm_attach->sgt;

	sgt = &tee_shm_attach->sgt;

	ret = sg_alloc_table(sgt, tee_shm->static_shm.sgt.orig_nents, GFP_KERNEL);
	if (ret) {
		pr_err("failed to alloc sgt.\n");
		return ERR_PTR(-ENOMEM);
	}

	rd = tee_shm->static_shm.sgt.sgl;
	wr = sgt->sgl;
	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	if (dir != DMA_NONE) {
		nents = dma_map_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
		if (!nents) {
			pr_err("failed to map sgl with iommu.\n");
			sg_free_table(sgt);
			sgt = ERR_PTR(-EIO);
			goto err_unlock;
		}
	}

	tee_shm_attach->is_mapped = true;
	tee_shm_attach->dir = dir;
	attach->priv = tee_shm_attach;

err_unlock:
	return sgt;
}

static void __tee_shm_dma_buf_unmap_dma_buf(struct dma_buf_attachment *attach,
		struct sg_table *table,
		enum dma_data_direction dir)
{
}

static void __tee_shm_dma_buf_release(struct dma_buf *dmabuf)
{
	struct tee_shm *shm = dmabuf->priv;
	struct tee_context *ctx;
	struct tee *tee;

	tee = shm->ctx->tee;
	ctx = shm->ctx;

	tee_shm_free_io(shm);
}

static int __tee_shm_dma_buf_mmap(struct dma_buf *dmabuf,
				  struct vm_area_struct *vma)
{
	struct tee_shm *shm = dmabuf->priv;
	size_t size = vma->vm_end - vma->vm_start;
	struct tee *tee;
	int ret;
	pgprot_t prot;
	unsigned long pfn;

	tee = shm->ctx->tee;
	pfn = shm->static_shm.paddr >> PAGE_SHIFT;


	if (shm->flags & TEE_SHM_CACHED)
		prot = vma->vm_page_prot;
	else
		prot = pgprot_noncached(vma->vm_page_prot);

	ret =
		remap_pfn_range(vma, vma->vm_start, pfn, size, prot);
	if (!ret)
		vma->vm_private_data = (void *)shm;

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)

static void *__tee_shm_dma_buf_kmap_atomic(struct dma_buf *dmabuf,
		unsigned long pgnum)
{
	return NULL;
}

static void *__tee_shm_dma_buf_kmap(struct dma_buf *db, unsigned long pgnum)
{
	struct tee_shm *shm = db->priv;

	/*
	 * A this stage, a shm allocated by the tee
	 * must be have a kernel address
	 */
	return shm->static_shm.kaddr;
}

static void __tee_shm_dma_buf_kunmap(
	struct dma_buf *db, unsigned long pfn, void *kaddr)
{
	/* unmap is done at the de init of the shm pool */
}

#endif

static const struct dma_buf_ops tee_static_shm_dma_buf_ops = {
	.attach = __tee_shm_attach_dma_buf,
	.detach = __tee_shm_detach_dma_buf,
	.map_dma_buf = __tee_shm_dma_buf_map_dma_buf,
	.unmap_dma_buf = __tee_shm_dma_buf_unmap_dma_buf,
	.release = __tee_shm_dma_buf_release,
#if  LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
	.kmap_atomic = __tee_shm_dma_buf_kmap_atomic,
	.kmap = __tee_shm_dma_buf_kmap,
	.kunmap = __tee_shm_dma_buf_kunmap,
#endif
	.mmap = __tee_shm_dma_buf_mmap,
};

static int tee_static_shm_export(struct tee *tee, struct tee_shm *shm,
				 int *export)
{
	struct dma_buf *dmabuf;
	int ret = 0;

#if defined(DEFINE_DMA_BUF_EXPORT_INFO)
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
#endif

	/* Temporary fix to support both older and newer kernel versions. */
#if defined(DEFINE_DMA_BUF_EXPORT_INFO)
	exp_info.priv = shm;
	exp_info.ops = &tee_static_shm_dma_buf_ops;
	exp_info.size = shm->size_alloc;
	exp_info.flags = O_RDWR;

	dmabuf = dma_buf_export(&exp_info);
#else
	dmabuf = dma_buf_export(shm, &tee_static_shm_dma_buf_ops,
				shm->size_alloc, O_RDWR, NULL);
#endif
	if (IS_ERR_OR_NULL(dmabuf)) {
		pr_err("dmabuf: couldn't export buffer (%ld)\n",
			PTR_ERR(dmabuf));
		ret = -EINVAL;
		goto out;
	}

	*export = dma_buf_fd(dmabuf, O_CLOEXEC);
out:
	return ret;
}

struct tee_shm *tee_shm_alloc_from_rpc(struct tee *tee, size_t size,
					uint32_t extra_flags)
{
	struct tee_shm *shm;


	mutex_lock(&tee->lock);
	shm = tkcore_alloc_shm(tee, size,
		TEE_SHM_TEMP | TEE_SHM_FROM_RPC | extra_flags);
	if (IS_ERR_OR_NULL(shm)) {
		pr_err("buffer allocation failed (%ld)\n",
			PTR_ERR(shm));
		goto out;
	}

	tee_inc_stats(&tee->stats[TEE_STATS_SHM_IDX]);
	list_add_tail(&shm->entry, &tee->list_rpc_shm);

	shm->ctx = NULL;

out:
	mutex_unlock(&tee->lock);
	return shm;
}

void tee_shm_free_from_rpc(struct tee_shm *shm)
{
	struct tee *tee;

	if (shm == NULL)
		return;

	tee = shm->tee;
	mutex_lock(&tee->lock);

	if (shm->ctx == NULL) {
		tee_dec_stats(&shm->tee->stats[TEE_STATS_SHM_IDX]);
		list_del(&shm->entry);
	}

	tkcore_shm_free(shm);
	mutex_unlock(&tee->lock);
}

static struct tee_shm *shm_from_paddr(struct tee *tee, void *paddr)
{
	struct list_head *pshm;

	if (list_empty(&tee->list_rpc_shm))
		return NULL;

	list_for_each(pshm, &tee->list_rpc_shm) {
		void *this_addr;
		struct tee_shm *shm;

		shm = list_entry(pshm, struct tee_shm, entry);
		this_addr = (void *) (unsigned long) shm->static_shm.paddr;

		if (this_addr == paddr)
			return shm;
	}

	return NULL;
}

/* Buffer allocated by rpc from fw and to be accessed by the user
 * Not need to be registered as it is not allocated by the user
 */
int tee_shm_fd_for_rpc(struct tee_context *ctx, struct tee_shm_io *shm_io)
{
	int ret;

	struct tee_shm *shm;
	struct tee *tee = ctx->tee;

	shm_io->fd_shm = 0;

	mutex_lock(&tee->lock);

	shm = shm_from_paddr(tee, shm_io->buffer);
	if (shm == NULL) {
		pr_err("Can't find shm for %p\n", shm_io->buffer);
		ret = -ENOMEM;
		goto out;
	}

	ret = tee_static_shm_export(tee, shm, &shm_io->fd_shm);

	if (ret) {
		ret = -ENOMEM;
		goto out;
	}

	shm->ctx = ctx;
	list_move(&shm->entry, &ctx->list_shm);

	shm->dev = get_device(_DEV(tee));
	ret = tee_get(tee);
	WARN_ON(ret);
	tee_context_get(ctx);

	WARN_ON(!tee->ops->shm_inc_ref(shm));
out:
	mutex_unlock(&tee->lock);
	return ret;
}

int tee_shm_alloc_io(struct tee_context *ctx, struct tee_shm_io *shm_io)
{
	struct tee_shm *shm;
	struct tee *tee = ctx->tee;
	int ret;


	if (ctx->usr_client)
		shm_io->fd_shm = 0;

	mutex_lock(&tee->lock);
	shm = tkcore_alloc_shm(tee, shm_io->size, shm_io->flags);
	if (IS_ERR_OR_NULL(shm)) {
		pr_err("buffer allocation failed (%ld)\n",
			PTR_ERR(shm));
		ret = PTR_ERR(shm);
		goto out;
	}

	if (ctx->usr_client) {
		ret = tee_static_shm_export(tee, shm, &shm_io->fd_shm);
		if (ret) {
			tkcore_shm_free(shm);
			ret = -ENOMEM;
			goto out;
		}

		shm->flags |= TEEC_MEM_DMABUF;
	} else {
		/*
		 * use shm_io.tee_shm to point to shm,
		 * so that kernel mode TEEC_AllocateSharedMemory()
		 * could find a reference to struct tee_shm
		 */
		shm_io->tee_shm = (void *) shm;
	}

	shm->ctx = ctx;
	shm->dev = get_device(_DEV(tee));
	ret = tee_get(tee);
	WARN_ON(ret);		/* tee_core_get must not issue */
	tee_context_get(ctx);

	tee_inc_stats(&tee->stats[TEE_STATS_SHM_IDX]);
	list_add_tail(&shm->entry, &ctx->list_shm);
out:
	mutex_unlock(&tee->lock);
	return ret;
}

void tee_shm_free_io(struct tee_shm *shm)
{
	struct tee_context *ctx = shm->ctx;
	struct tee *tee = ctx->tee;
	struct device *dev = shm->dev;

	mutex_lock(&tee->lock);
	tee_dec_stats(&tee->stats[TEE_STATS_SHM_IDX]);
	list_del(&shm->entry);

	tkcore_shm_free(shm);
	tee_put(ctx->tee);
	tee_context_put(ctx);
	if (dev)
		put_device(dev);
	mutex_unlock(&tee->lock);
}

static int tee_shm_db_get(struct tee *tee, struct tee_shm *shm, int fd,
			  unsigned int flags, size_t size, int offset)
{
	struct tee_shm_dma_buf *sdb;
	struct dma_buf *dma_buf;
	int ret = 0;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf)) {
		ret = PTR_ERR(dma_buf);
		goto exit;
	}

	sdb = kzalloc(sizeof(*sdb), GFP_KERNEL);
	if (IS_ERR_OR_NULL(sdb)) {
		pr_err("can't alloc tee_shm_dma_buf\n");
		ret = PTR_ERR(sdb);
		goto buf_put;
	}
	shm->static_shm.sdb = sdb;

	if (dma_buf->size < size + offset) {
		pr_err("dma_buf too small %zd < %zd + %d\n",
			dma_buf->size, size, offset);
		ret = -EINVAL;
		goto free_sdb;
	}

	sdb->attach = dma_buf_attach(dma_buf, _DEV(tee));
	if (IS_ERR_OR_NULL(sdb->attach)) {
		ret = PTR_ERR(sdb->attach);
		goto free_sdb;
	}

	sdb->sgt = dma_buf_map_attachment(sdb->attach, DMA_NONE);
	if (IS_ERR_OR_NULL(sdb->sgt)) {
		ret = PTR_ERR(sdb->sgt);
		goto buf_detach;
	}

	if (sg_nents(sdb->sgt->sgl) != 1) {
		ret = -EINVAL;
		goto buf_unmap;
	}

	shm->static_shm.paddr = sg_phys(sdb->sgt->sgl) + offset;
	if (dma_buf->ops->attach == __tee_shm_attach_dma_buf)
		sdb->tee_allocated = true;
	else
		sdb->tee_allocated = false;

	shm->flags |= TEEC_MEM_DMABUF;

	goto exit;

buf_unmap:
	dma_buf_unmap_attachment(sdb->attach, sdb->sgt, DMA_NONE);
buf_detach:
	dma_buf_detach(dma_buf, sdb->attach);
free_sdb:
	kfree(sdb);
buf_put:
	dma_buf_put(dma_buf);
exit:
	return ret;
}

struct tee_shm *tkcore_shm_get(struct tee_context *ctx,
				struct TEEC_SharedMemory *c_shm,
				size_t size, int offset)
{
	struct tee_shm *shm;
	struct tee *tee = ctx->tee;
	int ret;

	mutex_lock(&tee->lock);
	shm = kzalloc(sizeof(*shm), GFP_KERNEL);
	if (IS_ERR_OR_NULL(shm)) {
		pr_err("can't alloc tee_shm\n");
		ret = -ENOMEM;
		goto err;
	}

	shm->ctx = ctx;
	shm->tee = tee;
	shm->dev = _DEV(tee);
	shm->flags = c_shm->flags | TEE_SHM_MEMREF;
	shm->size_req = size;
	shm->size_alloc = 0;

	if (c_shm->flags & TEEC_MEM_KAPI) {
		struct tee_shm *kc_shm = (struct tee_shm *)c_shm->d.ptr;

		if (!kc_shm) {
			pr_err("kapi fd null\n");
			ret = -EINVAL;
			goto err;
		}
		shm->static_shm.paddr = kc_shm->static_shm.paddr;

		if (kc_shm->size_alloc < size + offset) {
			pr_err("kapi buff too small %zd < %zd + %d\n",
				kc_shm->size_alloc, size, offset);
			ret = -EINVAL;
			goto err;
		}

	} else if (c_shm->d.fd) {
		ret = tee_shm_db_get(tee, shm,
			c_shm->d.fd, c_shm->flags, size, offset);
		if (ret)
			goto err;
	} else if (!c_shm->buffer) {
		pr_debug("null buffer, pass 'as is'\n");
	} else {
		ret = -EINVAL;
		goto err;
	}

	mutex_unlock(&tee->lock);
	return shm;

err:
	kfree(shm);
	mutex_unlock(&tee->lock);
	return ERR_PTR(ret);
}

void tkcore_shm_put(struct tee_context *ctx, struct tee_shm *shm)
{
	if (WARN_ON(!ctx))
		return;

	if (WARN_ON(!ctx->tee))
		return;

	if (WARN_ON(!shm))
		return;

	WARN_ON(!(shm->flags & TEE_SHM_MEMREF));

	mutex_lock(&ctx->tee->lock);

	if (shm->flags & TEEC_MEM_DMABUF) {
		struct tee_shm_dma_buf *sdb;
		struct dma_buf *dma_buf;

		sdb = shm->static_shm.sdb;
		dma_buf = sdb->attach->dmabuf;

		dma_buf_unmap_attachment(sdb->attach, sdb->sgt, DMA_NONE);
		dma_buf_detach(dma_buf, sdb->attach);
		dma_buf_put(dma_buf);

		kfree(sdb);
		sdb = 0;
	}

	kfree(shm);
	mutex_unlock(&ctx->tee->lock);
}
