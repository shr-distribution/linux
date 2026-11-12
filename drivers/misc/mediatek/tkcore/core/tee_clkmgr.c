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

#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/string.h>

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/tee_clkmgr.h>
#include <linux/tee_client_api.h>

#include "tee_clkmgr_priv.h"

#ifdef CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#endif

struct clkmgr_handle {
	uint32_t token;
	void *e, *d;
	const void *p0, *p1, *p2;
	size_t argnum;
	struct list_head le;
};

/* sync with tee-os */
enum tee_clkmgr_type {
	TEE_CLKMGR_TYPE_SPI = 0,
	TEE_CLKMGR_TYPE_I2C,
	TEE_CLKMGR_TYPE_I2C_DMA
};

static const char * const clkid[] = {
	[TEE_CLKMGR_TYPE_SPI] = "spi",
	[TEE_CLKMGR_TYPE_I2C] = "i2c",
	[TEE_CLKMGR_TYPE_I2C_DMA] = "i2c-dma",
};

static LIST_HEAD(clk_list);
static DEFINE_SPINLOCK(clk_list_lock);

/* called inside list_lock */
static struct clkmgr_handle *get_clkmgr_handle(uint32_t token)
{
	struct clkmgr_handle *h;

	list_for_each_entry(h, &clk_list, le) {
		if (h->token == token)
			return h;
	}

	return NULL;
}

#ifdef CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT
/*
 * An SPI clock the TEE asks for without anything having registered it.
 *
 * TrustKernel's fingerprint trusted apps drive their sensor over SPI from
 * the secure side and ask for the controller's clock by token: type "spi",
 * id = the bus number. Nothing in this tree registers SPI clocks, so every
 * such request failed with TEEC_ERROR_ITEM_NOT_FOUND ("invalid token 1")
 * and the fingerprint HAL never came up. The stock kernel's
 * tee_clkmgr_handle() does not need a registration either: for an unknown
 * SPI token it looks the master up by bus number, takes "spi-clk" from the
 * controller's device, and keeps a handle using clk_prepare_enable() and
 * clk_disable_unprepare(). This does the same, reconstructed from that
 * binary, but allocates and looks up outside the list lock rather than
 * under it.
 */
static int clk_enable_wrap(const void *clk)
{
	return clk_prepare_enable((struct clk *) clk);
}

static void clk_disable_wrap(const void *clk)
{
	clk_disable_unprepare((struct clk *) clk);
}

static struct clkmgr_handle *spi_clkmgr_handle_create(uint32_t token)
{
	uint32_t id = token >> TEE_CLKMGR_TOKEN_ID_SHIFT;
	struct spi_master *master;
	struct clkmgr_handle *h, *w;
	struct clk *clk;

	master = spi_busnum_to_master(id);
	if (master == NULL) {
		pr_warn("tkcoredrv: spi%u not found\n", id);
		return NULL;
	}
	clk = devm_clk_get(master->dev.parent, "spi-clk");
	put_device(&master->dev);
	if (IS_ERR(clk)) {
		pr_warn("tkcoredrv: failed to get spi-clk: %ld\n", PTR_ERR(clk));
		return NULL;
	}

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (h == NULL)
		return NULL;
	h->token = token;
	h->e = (void *) clk_enable_wrap;
	h->d = (void *) clk_disable_wrap;
	h->p0 = clk;
	h->argnum = 1;

	spin_lock(&clk_list_lock);
	list_for_each_entry(w, &clk_list, le) {
		if (w->token == token) {
			spin_unlock(&clk_list_lock);
			kfree(h);
			return w;
		}
	}
	list_add(&h->le, &clk_list);
	spin_unlock(&clk_list_lock);

	return h;
}
#endif

int tee_clkmgr_handle(uint32_t token, uint32_t op)
{
	struct clkmgr_handle *ph, h;
	void *fn;

	spin_lock(&clk_list_lock);

	ph = get_clkmgr_handle(token | TEE_CLKMGR_TOKEN_NOT_LEGACY);
#ifdef CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT
	if (ph == NULL && ((token | TEE_CLKMGR_TOKEN_NOT_LEGACY) >>
			   TEE_CLKMGR_TOKEN_TYPE_SHIFT) == TEE_CLKMGR_TYPE_SPI) {
		spin_unlock(&clk_list_lock);
		ph = spi_clkmgr_handle_create(token | TEE_CLKMGR_TOKEN_NOT_LEGACY);
		spin_lock(&clk_list_lock);
	}
#endif
	if (ph == NULL) {
		pr_err("invalid token %u\n", token);
		spin_unlock(&clk_list_lock);
		return TEEC_ERROR_ITEM_NOT_FOUND;
	}

	memcpy(&h, ph, sizeof(h));

	spin_unlock(&clk_list_lock);

	fn = (op & TEE_CLKMGR_OP_ENABLE) ? h.e : h.d;

	if (h.argnum == 0) {
		((void (*)(void)) fn) ();
	} else if (h.argnum == 1) {
		((void (*)(const void *)) fn) (h.p0);
	} else if (h.argnum == 2) {
		((void (*)(const void *, const void *)) fn) (h.p0, h.p1);
	} else if (h.argnum == 3) {
		((void (*) (const void *, const void *, const void *)) fn)
			(h.p0, h.p1, h.p2);
	} else {
		pr_err("unsupported token %u argnum %zu\n",
			h.token, h.argnum);
		return TEEC_ERROR_NOT_SUPPORTED;
	}

	return 0;
}
EXPORT_SYMBOL(tee_clkmgr_handle);

int tee_clkmgr_register(const char *clkname, int id, void *e, void *d,
	void *p0, void *p1, void *p2, size_t argnum)
{
	size_t n;

	struct clkmgr_handle *h, *w;

	pr_info("tkcoredrv: clkname=%s id=%d\n",
		clkname, id);

	if (argnum > 3) {
		pr_err("does not support argnum %zu\n", argnum);
		return -EINVAL;
	}

	for (n = 0; n < ARRAY_SIZE(clkid); n++) {
		if (clkid[n] && strcmp(clkname, clkid[n]) == 0)
			break;
	}

	if (n == ARRAY_SIZE(clkid)) {
		pr_err("invalid clkname %s\n", clkname);
		return -EINVAL;
	}

	if ((id << TEE_CLKMGR_TOKEN_ID_SHIFT) &
		(TEE_CLKMGR_TOKEN_TYPE_MASK << TEE_CLKMGR_TOKEN_TYPE_SHIFT)) {
		pr_err("%s-%d: invalid id\n", clkname, id);
		return -EINVAL;
	}

	h = kmalloc(sizeof(struct clkmgr_handle), GFP_KERNEL);
	if (h == NULL)
		return -ENOMEM;

	h->token = TEE_CLKMGR_TOKEN((uint32_t) n, (uint32_t) id);
	h->e = e;
	h->d = d;
	h->p0 = p0;
	h->p1 = p1;
	h->p2 = p2;
	h->argnum = argnum;

	spin_lock(&clk_list_lock);

	/* check for duplication */
	list_for_each_entry(w, &clk_list, le) {
		if (w->token == h->token) {
			pr_err("clk 0x%x already registered\n",
				h->token);
			spin_unlock(&clk_list_lock);
			return -EINVAL;
		}
	}

	list_add(&(h->le), &clk_list);
	spin_unlock(&clk_list_lock);

	return 0;
}
EXPORT_SYMBOL(tee_clkmgr_register);

int tee_clkmgr_init(void)
{
	return 0;
}

void tee_clkmgr_exit(void)
{
	struct clkmgr_handle *h, *n;

	spin_lock(&clk_list_lock);

	list_for_each_entry_safe(h, n, &clk_list, le) {
		list_del(&(h->le));
		kfree(h);
	}

	spin_unlock(&clk_list_lock);
}
