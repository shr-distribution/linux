/*
 * SMedia Glamo 336x/337x command queue handling
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Copyright (C) 2009 Andreas Pokorny <andreas.pokorny@gmail.com>
 * Based on xf86-video-glamo (see below for details)
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Command queue handling functions based on those from xf86-video-glamo, to
 * which the following licence applies:
 *
 * Copyright  2007 OpenMoko, Inc.
 * Copyright © 2009 Lars-Peter Clausen <lars@metafoo.de>
 *
 * This driver is based on Xati,
 * Copyright  2004 Eric Anholt
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */


#include <drm/drmP.h>
#include <drm/glamo_drm.h>

#include <linux/mfd/glamo-core.h>
#include <linux/mfd/glamo-regs.h>

#include "glamo-drm-private.h"
#include "glamo-buffer.h"


#define GLAMO_CMDQ_SIZE (128 * 1024)    /* 128k ring buffer */


static inline void reg_write(struct glamodrm_handle *gdrm,
                             u_int16_t reg, u_int16_t val)
{
	iowrite16(val, gdrm->reg_base + reg);
}


static inline u16 reg_read(struct glamodrm_handle *gdrm, u_int16_t reg)
{
	return ioread16(gdrm->reg_base + reg);
}


static u32 glamo_get_read(struct glamodrm_handle *gdrm)
{
	/* we could turn off clock here */
	u32 ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL);
	ring_read |= (reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH) & 0x7) << 16;

	return ring_read;
}


static u32 glamo_get_write(struct glamodrm_handle *gdrm)
{
	u32 ring_write = reg_read(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL);
	ring_write |= (reg_read(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH) & 0x7) << 16;

	return ring_write;
}


/* Add commands to the ring buffer */
int glamo_add_to_ring(struct glamodrm_handle *gdrm, u16 *addr,
                      unsigned int count)
{
	size_t ring_write, ring_read;
	size_t new_ring_write;

	if ( count >= GLAMO_CMDQ_SIZE ) {
		printk(KERN_WARNING "[glamo-drm] CmdQ submission too large\n");
		return -EINVAL;
	}

	down(&gdrm->add_to_ring);

	ring_write = glamo_get_write(gdrm);

	/* Calculate where we'll end up */
	new_ring_write = (ring_write + count) % GLAMO_CMDQ_SIZE;

	/* Wait until there is enough space to queue the cmd buffer */
	if (new_ring_write > ring_write) {
		/* Loop while the read pointer is between the old and new
		 * positions */
		do {
			ring_read = glamo_get_read(gdrm);
		} while (ring_read > ring_write && ring_read < new_ring_write);
	} else {
		/* Same, but kind of inside-out */
		do {
			ring_read = glamo_get_read(gdrm);
		} while (ring_read > ring_write || ring_read < new_ring_write);
	}

	/* Are we about to wrap around? */
	if (ring_write >= new_ring_write) {

		u32 rest_size;

		/* Wrap around */
		rest_size = GLAMO_CMDQ_SIZE - ring_write; /* Space left */

		/* Write from current position to end */
		memcpy_toio(gdrm->cmdq_base+ring_write, addr, rest_size);

		/* Write from start */
		memcpy_toio(gdrm->cmdq_base, addr+(rest_size>>1),
		            count - rest_size);

		/* ring_write being 0 will result in a deadlock because the
		 * cmdq read will never stop. To avoid such an behaviour insert
		 * an empty instruction. */
		if (new_ring_write == 0) {
			iowrite16(0x0000, gdrm->cmdq_base);
			iowrite16(0x0000, gdrm->cmdq_base + 2);
			new_ring_write = 4;
		}

	} else {

		memcpy_toio(gdrm->cmdq_base+ring_write, addr, count);

	}

	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH,
			(new_ring_write >> 16) & 0x7f);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL,
			new_ring_write & 0xffff);

	if ( !(reg_read(gdrm, GLAMO_REG_CMDQ_STATUS) & 1<<3)  ) {
		printk(KERN_ERR "[glamo-drm] CmdQ decode failure.\n");
	}

	up(&gdrm->add_to_ring);

	return 0;
}


/* Return true for a legal sequence of commands, otherwise false */
static int glamo_sanitize_buffer(u16 *cmds, unsigned int count)
{
	/* XXX FIXME TODO: Implementation... */
	return 1;
}


/* Substitute the real addresses in VRAM for any required buffer objects */
static int glamo_do_relocation(struct glamodrm_handle *gdrm,
                               drm_glamo_cmd_buffer_t *cbuf, u16 *cmds,
                               struct drm_device *dev,
                               struct drm_file *file_priv)
{
	u32 *handles;
	int *offsets;
	int nobjs =  cbuf->nobjs;
	int i;

	if ( nobjs > 32 ) return -EINVAL;	/* Get real... */

	handles = kmalloc(nobjs*sizeof(u32), GFP_KERNEL);
	if ( handles == NULL ) return -1;
	if ( copy_from_user(handles, cbuf->objs, nobjs*sizeof(u32)) )
		return -1;

	offsets = kmalloc(nobjs*sizeof(int), GFP_KERNEL);
	if ( offsets == NULL ) return -1;
	if ( copy_from_user(offsets, cbuf->obj_pos, nobjs*sizeof(int)) )
		return -1;

	for ( i=0; i<nobjs; i++ ) {

		u32 handle = handles[i];
		int offset = offsets[i];
		struct drm_gem_object *obj;
		struct drm_glamo_gem_object *gobj;
		u32 addr;
		u16 addr_low, addr_high;

		if ( offset > cbuf->bufsz ) {
			printk(KERN_WARNING "[glamo-drm] Offset out of range"
			                    " for this relocation!\n");
			goto fail;
		}

		obj = drm_gem_object_lookup(dev, file_priv, handle);
		if ( obj == NULL ) return -1;

		/* Unref the object now, or it'll never get freed.
		 * This should really happen after the GPU has finished
		 * the commands which are about to be submitted. */
		drm_gem_object_unreference(obj);

		gobj = obj->driver_private;
		if ( gobj == NULL ) {
			printk(KERN_WARNING "[glamo-drm] This object has no"
			                    " private data!\n");
			goto fail;
		}

		addr = GLAMO_OFFSET_FB + gobj->block->start;
		addr_low = addr & 0xffff;
		addr_high = (addr >> 16) & 0x7f;

		/* FIXME: Should really check that the register is a
		 * valid one for this relocation. */

		*(cmds+(offset/2)+1) = addr_low;
		*(cmds+(offset/2)+3) = addr_high;

	}

	kfree(handles);
	kfree(offsets);
	return 0;

fail:
	kfree(handles);
	kfree(offsets);
	return -1;
}


/* This is DRM_IOCTL_GLAMO_CMDBUF */
int glamo_ioctl_cmdbuf(struct drm_device *dev, void *data,
                       struct drm_file *file_priv)
{
	int ret = 0;
	struct glamodrm_handle *gdrm;
	unsigned int count;
	drm_glamo_cmd_buffer_t *cbuf = data;
	u16 *cmds;

	gdrm = dev->dev_private;

	count = cbuf->bufsz;

	if ( count > PAGE_SIZE ) return -EINVAL;

	cmds = kmalloc(count, GFP_KERNEL);
	if ( cmds == NULL ) return -ENOMEM;
	if ( copy_from_user(cmds, cbuf->buf, count) ) 	{
		printk(KERN_WARNING "[glamo-drm] copy from user failed\n");
		ret = -EINVAL;
		goto cleanup;
	}

	/* Check the buffer isn't going to tell Glamo to enact naughtiness */
	if ( !glamo_sanitize_buffer(cmds, count) ) {
		printk(KERN_WARNING "[glamo-drm] sanitize buffer failed\n");
		ret = -EINVAL;
		goto cleanup;
	}

	/* Perform relocation, if necessary */
	if ( cbuf->nobjs ) {
		if ( glamo_do_relocation(gdrm, cbuf, cmds, dev, file_priv) )
		{
			printk(KERN_WARNING "[glamo-drm] Relocation failed\n");
			ret = -EINVAL;
			goto cleanup;
		}
	}

	glamo_add_to_ring(gdrm, cmds, count);

cleanup:
	kfree(cmds);

	return ret;
}


/* Return true for a legal sequence of commands, otherwise false */
static int glamo_sanitize_burst(u16 base, u16 *cmds, unsigned int count)
{
	/* XXX FIXME TODO: Implementation... */
	return 1;
}


static int glamo_relocate_burst(struct glamodrm_handle *gdrm,
                                drm_glamo_cmd_burst_t *cbuf, u16 *data,
                                struct drm_device *dev,
                                struct drm_file *file_priv)
{
	u32 *handles;
	int *offsets;
	int nobjs =  cbuf->nobjs;
	int i;

	if ( nobjs > 32 ) return -EINVAL;	/* Get real... */

	handles = kmalloc(nobjs*sizeof(u32), GFP_KERNEL);
	if ( handles == NULL ) return -1;
	if ( copy_from_user(handles, cbuf->objs, nobjs*sizeof(u32)) )
		return -1;

	offsets = kmalloc(nobjs*sizeof(int), GFP_KERNEL);
	if ( offsets == NULL ) return -1;
	if ( copy_from_user(offsets, cbuf->obj_pos, nobjs*sizeof(int)) )
		return -1;

	for ( i=0; i<nobjs; i++ ) {

		u32 handle = handles[i];
		int offset = offsets[i];
		struct drm_gem_object *obj;
		struct drm_glamo_gem_object *gobj;
		u32 addr;
		u16 addr_low, addr_high;

		if ( offset > cbuf->bufsz ) {
			printk(KERN_WARNING "[glamo-drm] Offset out of range"
			                    " for this relocation!\n");
			goto fail;
		}

		obj = drm_gem_object_lookup(dev, file_priv, handle);
		if ( obj == NULL ) return -1;

		/* Unref the object now, or it'll never get freed.
		 * FIXME: This should really happen after the GPU has
		 * finished executing these commands. */
		drm_gem_object_unreference(obj);

		gobj = obj->driver_private;
		if ( gobj == NULL ) {
			printk(KERN_WARNING "[glamo-drm] This object has no"
			                    " private data!\n");
			goto fail;
		}

		addr = GLAMO_OFFSET_FB + gobj->block->start;
		addr_low = addr & 0xffff;
		addr_high = (addr >> 16) & 0x7f;

		/* FIXME: Should really check that the register is a
		 * valid one for this relocation. */

		*(data+(offset/2)+0) = addr_low;
		*(data+(offset/2)+1) = addr_high;

	}

	kfree(handles);
	kfree(offsets);
	return 0;

fail:
	kfree(handles);
	kfree(offsets);
	return -1;
}


/* This is DRM_IOCTL_GLAMO_CMDBURST */
int glamo_ioctl_cmdburst(struct drm_device *dev, void *data,
                         struct drm_file *file_priv)
{
	int ret = 0;
	struct glamodrm_handle *gdrm;
	drm_glamo_cmd_burst_t *cbuf = data;
	u16 *burst;
	size_t burst_size;
	size_t data_size;

	gdrm = dev->dev_private;

	data_size = cbuf->bufsz;
	if ( data_size % 4 ) data_size += 2;
	if ( data_size % 4 ) return -EINVAL;
	burst_size = data_size + 4;  /* Add space for header */
	if ( burst_size > PAGE_SIZE ) return -EINVAL;

	burst = kmalloc(burst_size, GFP_KERNEL);
	if ( burst == NULL ) return -ENOMEM;

	/* Get data from userspace */
	if ( copy_from_user(burst+2, cbuf->data, cbuf->bufsz) ) 	{
		printk(KERN_WARNING "[glamo-drm] copy from user failed\n");
		ret = -EINVAL;
		goto cleanup;
	}

	/* Sanitise */
	if ( !glamo_sanitize_burst(cbuf->base, burst+2, cbuf->bufsz) ) {
		printk(KERN_WARNING "[glamo-drm] sanitize buffer failed\n");
		ret = -EINVAL;
		goto cleanup;
	}

	/* Relocate */
	if ( cbuf->nobjs ) {
		if ( glamo_relocate_burst(gdrm, cbuf, burst+2, dev, file_priv) )
		{
			printk(KERN_WARNING "[glamo-drm] Relocation failed\n");
			ret = -EINVAL;
			goto cleanup;
		}
	}

	/* Add burst header */
	burst[0] = 1<<15 | cbuf->base;
	burst[1] = data_size / 2;  /* -> 2-byte words */
	if ( burst[1] & 0x01 ) {
		printk(KERN_WARNING "[glamo-drm] Burst not aligned!\n");
		goto cleanup;
	}

	/* Zero-pad if necessary */
	if ( data_size % 4 ) {
		burst[burst_size-1] = 0x0000;
	}

	/* Add to command queue */
	glamo_add_to_ring(gdrm, burst, burst_size);

cleanup:
	kfree(burst);

	return ret;
}


int glamo_cmdq_setup(struct glamodrm_handle *gdrm)
{
	unsigned int i;

	init_MUTEX(&gdrm->add_to_ring);

	/* Enable 2D and 3D */
	glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_2D);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_2D);

	/* Start by zeroing the command queue memory */
	for ( i=0; i<GLAMO_CMDQ_SIZE; i+=2 ) {
		iowrite16(0x0000, gdrm->cmdq_base+i);
	}

	glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_CMDQ);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_CMDQ);

	/* Set up command queue location */
	reg_write(gdrm, GLAMO_REG_CMDQ_BASE_ADDRL,
					gdrm->cmdq_offs & 0xffff);
	reg_write(gdrm, GLAMO_REG_CMDQ_BASE_ADDRH,
					(gdrm->cmdq_offs >> 16) & 0x7f);

	/* Length of command queue in 1k blocks, minus one */
	reg_write(gdrm, GLAMO_REG_CMDQ_LEN, (GLAMO_CMDQ_SIZE >> 10)-1);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH, 0);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL, 0);
	reg_write(gdrm, GLAMO_REG_CMDQ_CONTROL,
	                                 1 << 12 |   /* Turbo flip (?) */
	                                 5 << 8  |   /* no interrupt */
	                                 8 << 4);    /* HQ threshold */

	return 0;
}


int glamo_cmdq_init(struct glamodrm_handle *gdrm)
{
	struct drm_gem_object *obj;
	struct drm_glamo_gem_object *gobj;
	int ret = 0;

	obj = glamo_gem_object_alloc(dev, GLAMO_CMDQ_SIZE, 4);
	if ( !obj ) {
		printk(KERN_ERR "[glamo-drm] Failed to allocate CmdQ\n");
		ret = -ENOMEM;
		goto out;
	}
	gobj = fbo->driver_private;
	gdrm->cmdq_offs = GLAMO_OFFSET_FB + gobj->block->start;
	gdrm->cmdq_base = ioremap(gdrm->vram->start + offs, GLAMO_CMDQ_SIZE);

	/* Set up registers */
	glamo_cmdq_setup(gdrm);

out:
	return ret;
}


int glamo_cmdq_shutdown(struct glamodrm_handle *gdrm)
{
	return 0;
}


void glamo_cmdq_suspend(struct glamodrm_handle *gdrm)
{
	/* Placeholder... */
}


void glamo_cmdq_resume(struct glamodrm_handle *gdrm)
{
	glamo_cmdq_setup(gdrm);
}


/* Initialise an object's contents to zero.
 * This is in glamo-cmdq.c in the hope that we can accelerate it later. */
void glamo_cmdq_blank(struct glamodrm_handle *gdrm, struct drm_gem_object *obj)
{
	char __iomem *cookie;
	struct drm_glamo_gem_object *gobj;
	int i;

	gobj = obj->driver_private;

	cookie = ioremap(gdrm->vram->start + gobj->block->start, obj->size);
	for ( i=0; i<obj->size; i+=2 ) {
		iowrite16(0, cookie+i);
	}
	iounmap(cookie);
}
