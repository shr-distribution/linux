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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/idr.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cpufeature.h>
#include <asm/cpu.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <asm-generic/ioctl.h>
#include <linux/sched.h>
#include <linux/version.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "linux/tee_core.h"
#include "linux/tee_ioc.h"
#include <linux/tee_kernel_api.h>

#include "tee_core_priv.h"
#include "tee_sysfs.h"
#include "tee_shm.h"
#include "tee_supp_com.h"

#include "tee_procfs.h"

#include <version.h>

static uint32_t nsdrv_feature_flags;

bool __attribute__((weak)) plat_cpu_is_big(int cpu)
{
	(void) cpu;
	return true;
}

int __attribute__((weak)) plat_tee_init(void) { return 0; }

#if defined(CONFIG_ARM)

#include <asm/mach/map.h>
#include <linux/io.h>

void *__arm_ioremap(unsigned long phys_addr, size_t size, unsigned int mtype)
{
	phys_addr_t last_addr;
	unsigned long offset = phys_addr & ~PAGE_MASK;
	unsigned long pfn = __phys_to_pfn(phys_addr);

	last_addr = phys_addr + size - 1;
	if (!size || last_addr < phys_addr)
		return NULL;

	return __arm_ioremap_pfn(pfn, offset, size, mtype);
}

#endif
void *tee_map_cached_shm(unsigned long pa, size_t len)
{
#if defined(CONFIG_ARM64)
	return ioremap_cache(pa, len);
#elif defined(CONFIG_ARM)
	return __arm_ioremap(pa, len, MT_MEMORY_RW);
#else
#error "tee_map_cached_shm() not implemented for this platform"
#endif
}
EXPORT_SYMBOL(tee_map_cached_shm);

void tee_unmap_cached_shm(void *va)
{
	iounmap(va);
}
EXPORT_SYMBOL(tee_unmap_cached_shm);


/* Store the class misc reference */
static struct class *misc_class;

/*
 * For the kernel api.
 * Get a reference on a device tee from the device needed
 */
struct tee *tee_get_tee(const char *devname)
{
	struct tee *tee;
	struct tee_user_device *tee_udev;
	struct device *device;

	if (!devname)
		return NULL;

	device = class_find_device(misc_class, NULL,
		(void *) devname, device_match_name);
	if (!device) {
		pr_err("can't find device [%s]\n", devname);
		return NULL;
	}

	tee_udev = container_of(dev_get_drvdata(device),
		struct tee_user_device, misc);
	if (!tee_udev->config->kernel_callable)
		return NULL;

	tee = dev_get_drvdata(device->parent);
	WARN_ON(!tee);

	return tee;
}

void tee_inc_stats(struct tee_stats_entry *entry)
{
	entry->count++;
	if (entry->count > entry->max)
		entry->max = entry->count;
}

void tee_dec_stats(struct tee_stats_entry *entry)
{
	entry->count--;
}

int __tee_get(struct tee *tee)
{
	int ret = 0;
	int v;

	WARN_ON(!tee);

	v = atomic_inc_return(&tee->refcount);
	if (v == 1) {
		WARN_ON(!try_module_get(tee->ops->owner));
		get_device(tee->dev);

		if (tee->ops->start)
			ret = tee->ops->start(tee);

		if (ret) {
			pr_err("%s::start() failed, err=0x%x\n",
				tee->name, ret);
			put_device(tee->dev);
			module_put(tee->ops->owner);
			atomic_dec(&tee->refcount);
		}
	} else {
		dev_warn(_DEV(tee), "Unexpected tee->refcount: 0x%x\n", v);
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL(__tee_get);


/**
 * tee_get - increases refcount of the tee
 * @tee:	[in]	tee to increase refcount of
 *
 * @note: If tee.ops.start() callback function is available,
 * it is called when refcount is equal at 1.
 */
int tee_get(struct tee *tee)
{
	int count;

	if (WARN_ON(!tee))
		return -EINVAL;

	count = atomic_inc_return(&tee->refcount);
	/*
	 * reference counter of tee device shall be
	 * larger than or equal to 1 in the whole lifetime
	 */
	WARN_ON(count == 1);

	if (count > tee->max_refcount)
		tee->max_refcount = count;

	return 0;
}

/**
 * tee_put - decreases refcount of the tee
 * @tee:	[in]	tee to reduce refcount of
 *
 * @note: If tee.ops.stop() callback function is available,
 * it is called when refcount is equal at 0.
 */
int tee_put(struct tee *tee)
{
	if (WARN_ON(!tee))
		return -EINVAL;

	WARN_ON(atomic_dec_and_test(&tee->refcount));
	return 0;
}

static struct tee_context *tee_common_open(struct inode *inode, struct miscdevice *misc)
{
	struct tee *tee;
	struct tee_context *ctx;
	struct tee_user_device *tee_udev;

	if (WARN_ON(misc->minor != iminor(inode)))
		return NULL;

	tee_udev = container_of(misc, struct tee_user_device, misc);

	tee = dev_get_drvdata(misc->parent);
	if (WARN_ON(!tee))
		return NULL;

	ctx = tee_context_create(tee);
	if (IS_ERR_OR_NULL(ctx))
		return NULL;

	ctx->usr_client = 1;
	ctx->config = tee_udev->config;

	return ctx;
}

static int tee_ctx_common_release(struct inode *inode, struct tee_context *ctx)
{
	if (WARN_ON(!ctx) || WARN_ON(!ctx->tee))
		return -EINVAL;

	tee_context_destroy(ctx);
	return 0;
}

static int tee_do_create_session(struct tee_context *ctx,
				 struct tee_cmd_io __user *u_cmd)
{
	int ret = -EINVAL;
	struct tee_cmd_io k_cmd;
	struct tee *tee;

	tee = ctx->tee;
	WARN_ON(!ctx->usr_client);

	if (copy_from_user(&k_cmd, (void *)u_cmd, sizeof(struct tee_cmd_io))) {
		pr_err("create_session: copy_from_user failed\n");
		goto exit;
	}

	if (k_cmd.fd_sess > 0) {
		pr_err("invalid fd_sess %d\n", k_cmd.fd_sess);
		goto exit;
	}

	if ((k_cmd.op == NULL) || (k_cmd.uuid == NULL) ||
		((k_cmd.data != NULL) && (k_cmd.data_size == 0)) ||
		((k_cmd.data == NULL) && (k_cmd.data_size != 0))) {
		pr_err("op or/and data parameters are not valid\n");
		goto exit;
	}

	ret = tee_session_create_fd(ctx, &k_cmd);
	put_user(k_cmd.err, &u_cmd->err);
	put_user(k_cmd.origin, &u_cmd->origin);
	if (ret)
		goto exit;

	put_user(k_cmd.fd_sess, &u_cmd->fd_sess);

exit:
	return ret;
}

static int tee_do_shm_alloc(struct tee_context *ctx,
	struct tee_shm_io __user *u_shm)
{
	int ret = -EINVAL;
	struct tee_shm_io k_shm;
	struct tee *tee = ctx->tee;

	WARN_ON(!ctx->usr_client);


	if (copy_from_user(&k_shm, (void *)u_shm, sizeof(struct tee_shm_io))) {
		pr_err("copy_from_user failed\n");
		goto exit;
	}

	if ((k_shm.buffer != NULL) || (k_shm.fd_shm != 0) ||
		((k_shm.flags & tee->shm_flags) == 0) ||
		(k_shm.registered != 0)) {
		pr_err(
			"shm parameters are not valid %p %d %08x %08x %d\n",
			(void *) k_shm.buffer,
			k_shm.fd_shm,
			(unsigned int) k_shm.flags,
			(unsigned int) tee->shm_flags,
			k_shm.registered);
		goto exit;
	}

	ret = tee_shm_alloc_io(ctx, &k_shm);
	if (ret)
		goto exit;

	put_user(k_shm.fd_shm, &u_shm->fd_shm);
	put_user(k_shm.flags, &u_shm->flags);

exit:
	return ret;
}

static int tee_do_get_fd_for_rpc_shm(struct tee_context *ctx,
	struct tee_shm_io __user *u_shm)
{
	int ret = -EINVAL;
	struct tee_shm_io k_shm;
	struct tee *tee = ctx->tee;


	WARN_ON(!ctx->usr_client);

	if (copy_from_user(&k_shm, (void *)u_shm, sizeof(struct tee_shm_io))) {
		pr_err("copy_from_user failed\n");
		goto exit;
	}

	if (k_shm.registered != 0) {
		pr_err("expecting shm to be unregistered\n");
		goto exit;
	}

	if ((k_shm.buffer == NULL) || (k_shm.size == 0) ||
		(k_shm.fd_shm != 0)) {
		pr_err("Invalid shm param. buffer: %p size: %u fd: %d\n",
			k_shm.buffer, k_shm.size, k_shm.fd_shm);
		goto exit;
	}

	if ((k_shm.flags & ~(tee->shm_flags)) ||
		((k_shm.flags & tee->shm_flags) == 0)) {
		pr_err(
			"Invalid shm flags: 0x%x expecting to be within 0x%x\n",
			k_shm.flags, tee->shm_flags);
		goto exit;
	}

	ret = tee_shm_fd_for_rpc(ctx, &k_shm);
	if (ret)
		goto exit;

	put_user(k_shm.fd_shm, &u_shm->fd_shm);

exit:

	return ret;
}

static long tee_internal_ioctl(struct tee_context *ctx,
				unsigned int cmd,
				void __user *u_arg)
{
	int ret = -EINVAL;

	switch (cmd) {
	case TEE_OPEN_SESSION_IOC:
		ret = tee_do_create_session(ctx,
			(struct tee_cmd_io __user *) u_arg);
		break;

	case TEE_ALLOC_SHM_IOC:
		ret = tee_do_shm_alloc(ctx,
			(struct tee_shm_io __user *) u_arg);
		break;

	case TEE_GET_FD_FOR_RPC_SHM_IOC:
		/* this ioc is only for tee-daemon */
		ret = (ctx->config && ctx->config->admin) ?
			tee_do_get_fd_for_rpc_shm(ctx,
					(struct tee_shm_io __user *) u_arg) : -EINVAL;
		break;

	case TEE_QUERY_DRV_FEATURE_IOC:
		if (u_arg) {
			if (copy_to_user(u_arg, &nsdrv_feature_flags,
					sizeof(nsdrv_feature_flags))) {
				ret = -EFAULT;
			} else {
				ret = 0;
			}
		} else
			ret = -EINVAL;

		break;

	default:
		pr_err("internal_ioctl: Unknown command: %u\n", cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT

static int convert_compat_tee_shm(struct TEEC_SharedMemory __user *shm)
{
	if (unlikely(put_user(0, ((uint32_t __user *) &(shm->buffer)) + 1)))
		return -EFAULT;

	if (unlikely(put_user(0, ((uint32_t __user *) &(shm->size)) + 1)))
		return -EFAULT;

	if (unlikely(put_user(0, ((uint32_t __user *) &(shm->d.fd)) + 1)))
		return -EFAULT;

	return 0;
}

static int convert_compat_tee_param(union TEEC_Parameter __user *p,
	uint32_t type)
{
	struct TEEC_SharedMemory __user *p_shm;

	switch (type) {
	case TEEC_MEMREF_TEMP_INPUT:
	case TEEC_MEMREF_TEMP_OUTPUT:
	case TEEC_MEMREF_TEMP_INOUT:

		if (unlikely(put_user(0,
				((uint32_t __user *) &(p->tmpref.buffer)) + 1)))
			return -EFAULT;

		if (unlikely(put_user(0,
				((uint32_t __user *) &(p->tmpref.size)) + 1)))
			return -EFAULT;

		break;

	case TEEC_MEMREF_PARTIAL_INPUT:
	case TEEC_MEMREF_PARTIAL_OUTPUT:
	case TEEC_MEMREF_PARTIAL_INOUT:
	case TEEC_MEMREF_WHOLE:

		if (unlikely(put_user(0,
				((uint32_t __user *) &(p->memref.parent)) + 1)))
			return -EFAULT;

		if (unlikely(put_user(0,
				((uint32_t __user *) &(p->memref.size)) + 1)))
			return -EFAULT;

		if (unlikely(put_user(0,
				((uint32_t __user *) &(p->memref.offset)) + 1)))
			return -EFAULT;

		if ((copy_from_user(&p_shm, &p->memref.parent,
				sizeof(p_shm))))
			return -EFAULT;

		if (p_shm == NULL)
			break;

		if (convert_compat_tee_shm(p_shm))
			return -EFAULT;

		break;

	default:
		break;
	}

	return 0;
}

int convert_compat_tee_cmd(struct tee_cmd_io __user *u_cmd)
{
	uint32_t i;
	struct TEEC_Operation __user *p_op;
	uint32_t paramTypes;

	if (u_cmd == NULL)
		return -EINVAL;

	if (unlikely(put_user(0, ((uint32_t __user *) &u_cmd->uuid) + 1)))
		return -EFAULT;

	if (unlikely(put_user(0, ((uint32_t __user *) &u_cmd->data) + 1)))
		return -EFAULT;

	if (unlikely(put_user(0, ((uint32_t __user *) &u_cmd->op) + 1)))
		return -EFAULT;

	if (copy_from_user(&p_op, &u_cmd->op, sizeof(p_op)))
		return -EFAULT;

	if (p_op == NULL)
		return -EINVAL;

	if (copy_from_user(&paramTypes, (uint32_t __user *) &p_op->paramTypes,
			sizeof(p_op->paramTypes)))
		return -EFAULT;

	if (unlikely(put_user(0, ((uint32_t __user *) &p_op->session) + 1)))
		return -EFAULT;

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; i++) {
		if (convert_compat_tee_param(&p_op->params[i],
			TEEC_PARAM_TYPE_GET(paramTypes, i))) {
			pr_err("bad param %u\n", i);
			return -EFAULT;
		}
	}

	return 0;
}

int convert_compat_tee_shm_io(struct tee_shm_io __user *shm_io)
{
	if (shm_io == NULL)
		return -EINVAL;

	if (unlikely(put_user(0, ((uint32_t __user *) &shm_io->buffer) + 1)))
		return -EFAULT;

	return 0;
}

int convert_compat_tee_spta_inst(struct tee_spta_inst_desc __user *spta)
{
	if (spta == NULL)
		return -EINVAL;

	if (unlikely(put_user(0,
		((uint32_t __user *) &spta->ta_binary) + 1)))
		return -EFAULT;

	if (unlikely(put_user(0,
		((uint32_t __user *) &spta->response_len) + 1)))
		return -EFAULT;

	return 0;
}

static long tee_compat_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	struct tee_context *ctx = filp->private_data;
	void __user *u_arg;

	WARN_ON(!ctx);
	WARN_ON(!ctx->tee);

	if (is_compat_task())
		u_arg = compat_ptr(arg);
	else
		u_arg = (void __user *)arg;

	switch (cmd) {
	case TEE_OPEN_SESSION_IOC:
		if (convert_compat_tee_cmd((struct tee_cmd_io __user *) u_arg))
			return -EFAULT;
		break;

	case TEE_ALLOC_SHM_IOC:
	case TEE_GET_FD_FOR_RPC_SHM_IOC:
		if (convert_compat_tee_shm_io(
				(struct tee_shm_io __user *) u_arg))
			return -EFAULT;
		break;

	case TEE_INSTALL_TA_IOC:
		if (convert_compat_tee_spta_inst(
			(struct tee_spta_inst_desc __user *) u_arg))
			return -EFAULT;
		break;

	default:
		break;
	}

	return tee_internal_ioctl(ctx, cmd, u_arg);
}
#endif

static long tee_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct tee_context *ctx = filp->private_data;

	WARN_ON(!ctx);
	WARN_ON(!ctx->tee);

	return tee_internal_ioctl(ctx, cmd, (void __user *) arg);
}

static bool big_core_affinity_need_bind_cpu(void *priv)
{
	(void) priv;
	return true;
}

static bool big_core_affinity_match_cpu(void *priv, int cpu)
{
	(void) priv;
	return plat_cpu_is_big(cpu);
}

static int big_core_affinity_get_candidate_cpu(void *priv)
{
    int cpu;
	static int candidate_cpu = -1;

	(void) cpu;
	if (candidate_cpu >= 0)
		return candidate_cpu;

	for_each_possible_cpu(cpu) {
		if (plat_cpu_is_big(cpu)) {
			candidate_cpu = cpu;
			pr_info("tkcore: candidate big cpu set to %d\n", candidate_cpu);
			break;
		}
	}

	return candidate_cpu;
}

static const struct tee_task_cpu_affinity_operations
		big_core_affinity_operations = {
	.need_bind_cpu = big_core_affinity_need_bind_cpu,
	.match_cpu = big_core_affinity_match_cpu,
	.get_candidate_cpu = big_core_affinity_get_candidate_cpu,
};

static const struct tee_task_cpu_affinity big_core_affinity = {
	.ops = &big_core_affinity_operations,
	.priv = NULL,
};

#if IS_ENABLED(CONFIG_TRUSTKERNEL_TEE_LEGACY_CLIENT_DEVICE)

static char *_tee_supp_app_name = "teed";

static int tee_legacy_release(struct inode *inode, struct file *filp)
{
	int r;
	struct tee *tee;
	struct tee_context *ctx = filp->private_data;

	tee = ctx->tee;
	r = tee_ctx_common_release(inode, ctx);
	if (r != 0)
		return r;

	if (WARN_ON(!tee) || WARN_ON(!(tee->rpc)))
		return -1;

	if ((atomic_read(&tee->rpc->used) == 1) &&
			(strncmp(_tee_supp_app_name, current->comm,
					 strlen(_tee_supp_app_name)) == 0)) {
		atomic_sub(1, &tee->rpc->used);
	}

	return 0;
}

static int tee_legacy_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct tee *tee;
	struct tee_context *ctx;

	struct miscdevice *misc = (struct miscdevice *) filp->private_data;

	ctx = tee_common_open(inode, misc);
	if (ctx == NULL)
		return -1;

	tee = ctx->tee;
	if (WARN_ON(!tee->rpc)) {
		ret = -1;
		goto error;
	}

	if (!strncmp(_tee_supp_app_name, current->comm,
				strlen(_tee_supp_app_name))) {
		if (atomic_add_return(1, &tee->rpc->used) > 1) {
			ret = -EBUSY;
			pr_warn("Only one daemon is allowed\n");
			atomic_sub(1, &tee->rpc->used);
			goto error;
		}
	}

	filp->private_data = ctx;
	return 0;

error:
	tee_context_destroy(ctx);
	return ret;
}

const struct file_operations tee_admin_fops = {
	.owner = THIS_MODULE,
	.read = tee_supp_read,
	.write = tee_supp_write,
	.open = tee_legacy_open,
	.release = tee_legacy_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_compat_ioctl,
#endif
	.unlocked_ioctl = tee_ioctl
};

static struct tee_device_config tee_device_configs[] = {
	{
		.name = "tkcoredrv",
		.fops = &tee_admin_fops,
		.aff = NULL,
		.admin = true,
		.kernel_callable = true,
		.is_required = NULL
	},
    {
		.name = "tkcore_fp",
		.fops = &tee_admin_fops,
		.aff = &big_core_affinity,
		.admin = false,
		.kernel_callable = false,
		.is_required = NULL
    }
};

#else

static int tee_supp_release(struct inode *inode, struct file *filp)
{
	int r;
	struct tee *tee;
	struct tee_context *ctx = filp->private_data;

	tee = ctx->tee;
	r = tee_ctx_common_release(inode, ctx);
	if (r != 0)
		return r;

	if (WARN_ON(!tee) || WARN_ON(!(tee->rpc)))
		return -1;

	atomic_sub(1, &tee->rpc->used);
	return 0;
}

static int tee_supp_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct tee *tee;
	struct tee_context *ctx;

	struct miscdevice *misc = (struct miscdevice *) filp->private_data;

	ctx = tee_common_open(inode, misc);
	if (ctx == NULL)
		return -1;

	tee = ctx->tee;
	if (WARN_ON(!tee->rpc)) {
		ret = -1;
		goto error;
	}

	if (atomic_add_return(1, &tee->rpc->used) > 1) {
		ret = -EBUSY;
		pr_warn("Only one daemon is allowed\n");
		atomic_sub(1, &tee->rpc->used);
		goto error;
	}

	filp->private_data = ctx;
	return 0;

error:
	tee_context_destroy(ctx);
	return ret;
}

static int tee_ctx_open(struct inode *inode, struct file *filp)
{
	struct tee_context *ctx;
	struct miscdevice *misc = (struct miscdevice *) filp->private_data;

	ctx = tee_common_open(inode, misc);
	if (ctx == NULL)
		return -1;

	filp->private_data = ctx;
	return 0;
}

static int tee_ctx_release(struct inode *inode, struct file *filp)
{
	return tee_ctx_common_release(inode,
		(struct tee_context *) filp->private_data);
}

const struct file_operations tee_client_fops = {
	.owner = THIS_MODULE,
	.open = tee_ctx_open,
	.release = tee_ctx_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_compat_ioctl,
#endif
	.unlocked_ioctl = tee_ioctl
};

const struct file_operations tee_admin_fops = {
	.owner = THIS_MODULE,
	.read = tee_supp_read,
	.write = tee_supp_write,
	.open = tee_supp_open,
	.release = tee_supp_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_compat_ioctl,
#endif
	.unlocked_ioctl = tee_ioctl
};

static struct tee_device_config tee_device_configs[] = {
	{
		.name = "tkcore_admin",
		.fops = &tee_admin_fops,
		.aff = NULL,
		.admin = true,
		.kernel_callable = false,
		.is_required = NULL
	},
	{
		.name = "tkcore_client",
		.fops = &tee_client_fops,
		.aff = NULL,
		.admin = false,
		.kernel_callable = true,
		.is_required = NULL
	},
	{
		.name = "tkcore_fp",
		.fops = &tee_client_fops,
		.aff = &big_core_affinity,
		.admin = false,
		.kernel_callable = false,
		.is_required = NULL
	},
};

#endif

static void tee_plt_device_release(struct device *dev)
{
	(void) dev;
}

static spinlock_t tee_idr_lock;
static struct idr tee_idr;

/* let caller to guarantee tee
 * and id are not NULL. using lock to protect
 */
int tee_core_alloc_uuid(void *ptr)
{
	int r;

	idr_preload(GFP_KERNEL);

	spin_lock(&tee_idr_lock);
	r = idr_alloc(&tee_idr, ptr, 1, 0, GFP_NOWAIT);
	if (r < 0)
		pr_err("Bad alloc tee_uuid. rv: %d\n",
			r);
	spin_unlock(&tee_idr_lock);

	idr_preload_end();

	return r;
}

void *tee_core_uuid2ptr(int id)
{
	return idr_find(&tee_idr, id);
}

/* let caller to guarantee tee
 *and id are not NULL
 */
void tee_core_free_uuid(int id)
{
	idr_remove(&tee_idr, id);
}

struct tee *tee_core_alloc(struct device *dev, char *name, int id,
	const struct tee_ops *ops, size_t len)
{
	struct tee *tee;

	if (!dev || !name || !ops ||
		!ops->open || !ops->close || !ops->alloc || !ops->free)
		return NULL;

	tee = devm_kzalloc(dev, sizeof(struct tee) + len, GFP_KERNEL);
	if (!tee) {
		tee = NULL;
		pr_err("core_alloc: kzalloc failed\n");
		return tee;
	}

	if (!dev->release)
		dev->release = tee_plt_device_release;

	tee->dev = dev;
	tee->id = id;
	tee->ops = ops;
	tee->priv = &tee[1];

	snprintf(tee->name, sizeof(tee->name), "%s", name);
	pr_info("TEE core: Alloc the misc device \"%s\" (id=%d)\n",
		tee->name, tee->id);

	mutex_init(&tee->lock);
	atomic_set(&tee->refcount, 0);

	INIT_LIST_HEAD(&tee->udevs);

	INIT_LIST_HEAD(&tee->list_ctx);
	INIT_LIST_HEAD(&tee->list_rpc_shm);

	tee->state = TEE_OFFLINE;
	tee->shm_flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT | TEEC_MEM_NONSECURE;

	if ((tee_supp_init(tee))) {
		devm_kfree(dev, tee);
		return NULL;
	}

	return tee;
}
EXPORT_SYMBOL(tee_core_alloc);

int tee_core_free(struct tee *tee)
{
	if (tee) {
		tee_supp_deinit(tee);
		devm_kfree(tee->dev, tee);
	}
	return 0;
}
EXPORT_SYMBOL(tee_core_free);

static void init_tee_user_device(struct tee_user_device *tee_udev,
				struct device *parent, const struct tee_device_config *cfg)
{
	tee_udev->config = cfg;
	tee_udev->misc.parent = parent;
	tee_udev->misc.name = cfg->name;
	tee_udev->misc.fops = cfg->fops;
	tee_udev->misc.minor = MISC_DYNAMIC_MINOR;
}

int tee_core_add(struct tee *tee)
{
	int i, rc = 0;
	struct tee_user_device *tee_udev, *tmp;

	if (!tee)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tee_device_configs); i++) {
		struct tee_user_device *tee_udev;
		const struct tee_device_config *cfg = &tee_device_configs[i];

		if (cfg->is_required && !cfg->is_required()) {
			// skip device that is not required
			continue;
		}

		// must use kzalloc, or the name of device under /dev would be wrong
		tee_udev = devm_kzalloc(tee->dev, sizeof(struct tee_user_device), GFP_KERNEL);
		if (tee_udev == NULL) {
			rc = -ENOMEM;
			goto err_deregister_misc;
		}
		init_tee_user_device(tee_udev, tee->dev, cfg);

		rc = misc_register(&tee_udev->misc);
		if (rc != 0) {
			pr_err("register misc device[%d] failed with %d\n", i, rc);
			kfree(tee_udev);
			goto err_deregister_misc;
		}

		/*
		 * insert new tee_udev into tee->udevs,
		 * so we could correctly release them
		 * in case it fails
		 */
		list_add_tail(&tee_udev->le, &tee->udevs);

		if (WARN_ON(!tee_udev->misc.this_device->class)) {
			rc = -ENOENT;
			goto err_deregister_misc;
		}

		/* Register a static reference on the class misc
		 * to allow finding device by class
		 */
		if (!misc_class) {
			misc_class = tee_udev->misc.this_device->class;
		} else if (WARN_ON(misc_class != tee_udev->misc.this_device->class)) {
			/* all misc device must share the same class */
			rc = -ENOENT;
			goto err_deregister_misc;
		}
	}

	rc = tee_init_procfs(tee);
	if (rc)
		goto err_deregister_misc;

	rc = tee_init_sysfs(tee);
	if (rc)
		goto err_deinit_procfs;

	tee_peridev_init();

	list_for_each_entry(tee_udev, &tee->udevs, le) {
		pr_info(
			"TKCore misc: Register the misc device \"%s\" (id=%d,minor=%d)\n",
			dev_name(tee_udev->misc.this_device), tee->id,
			tee_udev->misc.minor);
	}

	return 0;

err_deinit_procfs:
	tee_exit_procfs();

err_deregister_misc:
	list_for_each_entry_safe(tee_udev, tmp, &tee->udevs, le) {
		misc_deregister(&tee_udev->misc);

		list_del(&tee_udev->le);
		kfree(tee_udev);
	}

	return rc;
}
EXPORT_SYMBOL(tee_core_add);

int tee_core_del(struct tee *tee)
{
	struct tee_user_device *tee_udev, *tmp;

	if (!tee)
		return 0;

	tee_peridev_exit();

	tee_cleanup_sysfs(tee);

	list_for_each_entry_safe(tee_udev, tmp, &tee->udevs, le) {
		misc_deregister(&tee_udev->misc);

		list_del(&tee_udev->le);
		kfree(tee_udev);
	}

	tee_core_free(tee);
	return 0;
}
EXPORT_SYMBOL(tee_core_del);

static int __init tee_core_init(void)
{
	spin_lock_init(&tee_idr_lock);
	idr_init(&tee_idr);

	plat_tee_init();
	return 0;
}

static void __exit tee_core_exit(void)
{
}

#ifndef MODULE
rootfs_initcall(tee_core_init);
#else
module_init(tee_core_init);
#endif
module_exit(tee_core_exit);

MODULE_AUTHOR("TrustKernel");
MODULE_DESCRIPTION("TrustKernel TKCore TEEC v1.0");
MODULE_VERSION(TKCORE_NSDRV_VER);
MODULE_LICENSE("GPL");
