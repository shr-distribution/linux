/*
 * SMedia Glamo 336x/337x memory management
 *
 * Copyright (c) 2009 Thomas White <taw@bitwiz.org.uk>
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Memory mapping functions based on i915_gem.c, to which the following
 * notice applies:
 *
 * Copyright © 2008 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 */


#include <drm/drmP.h>
#include <drm/glamo_drm.h>

#include "glamo-drm-private.h"
#include "glamo-cmdq.h"	/* For glamo_cmdq_blank() */


struct drm_gem_object *glamo_gem_object_alloc(struct drm_device *dev, int size,
                                              int alignment)
{
	struct drm_gem_object *obj;
	struct glamodrm_handle *gdrm;
	struct drm_glamo_gem_object *gobj;

	gdrm = dev->dev_private;

	size = roundup(size, PAGE_SIZE);

	obj = drm_gem_object_alloc(dev, size);
	if (obj == NULL) return NULL;

	/* See glamodrm_gem_init_object() below */
	gobj = obj->driver_private;

	/* Allocate memory for this object in VRAM */
	gobj->block = drm_mm_search_free(gdrm->mmgr, size, alignment, 1);
	if (!gobj->block) {
		goto fail;
	}
	gobj->block = drm_mm_get_block(gobj->block, size, alignment);
	if (!gobj->block) {
		goto fail;
	}

	/* Arrange for the contents to be set to zero */
	glamo_cmdq_blank(gdrm, obj);

	return obj;

fail:
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	printk(KERN_INFO "[glamo-drm] Failed to allocate object\n");

	return NULL;
}


int glamo_ioctl_gem_create(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_glamo_gem_create *args = data;
	struct drm_gem_object *obj;
	int handle, ret, alignment, size;

	/* Alignment must be a non-zero multiple of 2 */
	alignment = args->alignment;
	if ( alignment < 2 ) alignment = 2;
	if ( alignment % 2 ) alignment *= 2;

	/* Size must be similarly sanitised */
	size = args->size;
	if ( size < 2 ) size = 2;
	if ( size % 2 ) size += 1;

	/* Create an object */
	obj = glamo_gem_object_alloc(dev, size, alignment);
	if ( obj == NULL ) return -ENOMEM;

	/* Create a handle for it */
	ret = drm_gem_handle_create(file_priv, obj, &handle);
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_handle_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	if (ret) goto fail;

	/* Return */
	args->handle = handle;
	return 0;

fail:
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	printk(KERN_INFO "[glamo-drm] Failed to allocate object\n");
	return ret;
}


int glamodrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct drm_device *dev = obj->dev;
	struct drm_glamo_gem_object *gobj = obj->driver_private;
	struct glamodrm_handle *gdrm = dev->dev_private;
	pgoff_t page_offset;
	unsigned long pfn;
	int ret = 0;

	/* We don't use vmf->pgoff since that has the fake offset */
	page_offset = ((unsigned long)vmf->virtual_address - vma->vm_start) >>
	               PAGE_SHIFT;

	mutex_lock(&dev->struct_mutex);
	pfn = ((gdrm->vram->start + GLAMO_OFFSET_FB + gobj->block->start)
	       >> PAGE_SHIFT) + page_offset;
	ret = vm_insert_pfn(vma, (unsigned long)vmf->virtual_address, pfn);
	mutex_unlock(&dev->struct_mutex);

	switch (ret) {
	case -ENOMEM:
	case -EAGAIN:
		return VM_FAULT_OOM;
	case -EFAULT:
	case -EBUSY:
		DRM_ERROR("can't insert pfn??  fault or busy...\n");
		return VM_FAULT_SIGBUS;
	default:
		return VM_FAULT_NOPAGE;
	}
}


static int glamo_gem_create_mmap_offset(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct drm_gem_mm *mm = dev->mm_private;
	struct drm_glamo_gem_object *gobj = obj->driver_private;
	struct drm_map_list *list;
	struct drm_local_map *map;
	int ret = 0;

	/* Set the object up for mmap'ing */
	list = &obj->map_list;
	list->map = kzalloc(sizeof(struct drm_map_list), GFP_KERNEL);
	if (!list->map)
		return -ENOMEM;

	map = list->map;
	map->type = _DRM_GEM;
	map->size = obj->size;
	map->handle = obj;

	/* Get a DRM GEM mmap offset allocated... */
	list->file_offset_node = drm_mm_search_free(&mm->offset_manager,
	                                            obj->size / PAGE_SIZE, 0, 0);
	if (!list->file_offset_node) {
		DRM_ERROR("failed to allocate offset for bo %d\n", obj->name);
		ret = -ENOMEM;
		goto out_free_list;
	}

	list->file_offset_node = drm_mm_get_block(list->file_offset_node,
						  obj->size / PAGE_SIZE, 0);
	if (!list->file_offset_node) {
		ret = -ENOMEM;
		goto out_free_list;
	}

	list->hash.key = list->file_offset_node->start;
	if (drm_ht_insert_item(&mm->offset_hash, &list->hash)) {
		DRM_ERROR("failed to add to map hash\n");
		goto out_free_mm;
	}

	/* By now we should be all set, any drm_mmap request on the offset
	 * below will get to our mmap & fault handler */
	gobj->mmap_offset = ((uint64_t) list->hash.key) << PAGE_SHIFT;

	return 0;

out_free_mm:
	drm_mm_put_block(list->file_offset_node);
out_free_list:
	kfree(list->map);

	return ret;
}


int glamo_ioctl_gem_mmap(struct drm_device *dev, void *data,
			 struct drm_file *file_priv)
{
	struct drm_glamo_gem_mmap *args = data;
	struct drm_gem_object *obj;
	struct drm_glamo_gem_object *gobj;
	int ret;

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (obj == NULL)
		return -EBADF;

	mutex_lock(&dev->struct_mutex);

	gobj = obj->driver_private;
	if (!gobj->mmap_offset) {
		ret = glamo_gem_create_mmap_offset(obj);
		if (ret) {
			printk(KERN_CRIT "Couldn't create mmap offset\n");
			drm_gem_object_unreference(obj);
			mutex_unlock(&dev->struct_mutex);
			return ret;
		}
	}

	args->offset = gobj->mmap_offset;

	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}


int glamo_ioctl_gem_pin(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_pin\n");
	return 0;
}


int glamo_ioctl_gem_unpin(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_unpin\n");
	return 0;
}


int glamo_ioctl_gem_pread(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_pread\n");
	return 0;
}


int glamo_ioctl_gem_pwrite(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_pwrite\n");
	return 0;
}


int glamodrm_gem_init_object(struct drm_gem_object *obj)
{
	struct drm_glamo_gem_object *gobj;

	/* Allocate a private structure */
	gobj = kzalloc(sizeof(*gobj), GFP_KERNEL);
	if (!gobj) return -ENOMEM;

	obj->driver_private = gobj;
	gobj->obj = obj;

	return 0;
}


void glamodrm_gem_free_object(struct drm_gem_object *obj)
{
	struct drm_glamo_gem_object *gobj;
	struct drm_map_list *list;
	struct drm_device *dev;
	struct drm_gem_mm *mm;
	struct drm_local_map *map;

	dev = obj->dev;
	mm = dev->mm_private;
	gobj = obj->driver_private;

	/* Free the VRAM */
	if ( gobj->block != NULL ) {
		drm_mm_put_block(gobj->block);
	}

	/* Release mappings */
	list = &obj->map_list;
	drm_ht_remove_item(&mm->offset_hash, &list->hash);
	if (list->file_offset_node) {
		drm_mm_put_block(list->file_offset_node);
		list->file_offset_node = NULL;
	}
	map = list->map;
	if (map) {
		kfree(map);
		list->map = NULL;
	}

	/* Free the private structure */
	kfree(obj->driver_private);
}


/* Memory management initialisation */
int glamo_buffer_init(struct glamodrm_handle *gdrm)
{
	gdrm->mmgr = kzalloc(sizeof(struct drm_mm), GFP_KERNEL);
	drm_mm_init(gdrm->mmgr, 0, gdrm->vram_size);

	/* Reserve a scratch buffer.  We do this outside the protections
	 * of the other GEM code.  To do this safely, the allocation must
	 * be a multiple of PAGE_SIZE. */
	gdrm->scratch = drm_mm_search_free(gdrm->mmgr, PAGE_SIZE, 4, 1);
	if ( gdrm->scratch ) {
		gdrm->scratch = drm_mm_get_block(gdrm->scratch, PAGE_SIZE, 4);
	}
	if ( !gdrm->scratch ) {
		printk(KERN_WARNING "[glamo-drm] Couldn't allocate"
		                    " scratch buffer!\n");
	}

	return 0;
}


/* Memory management finalisation */
int glamo_buffer_final(struct glamodrm_handle *gdrm)
{
	drm_mm_takedown(gdrm->mmgr);
	kfree(gdrm->mmgr);
	return 0;
}
