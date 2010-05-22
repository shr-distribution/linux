/* Smedia Glamo 336x/337x DRM private bits
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Copyright (C) 2009 Andreas Pokorny <andreas.pokorny@gmail.com>
 * Based on xf86-video-glamo
 * Copyright  2007 OpenMoko, Inc.
 * Copyright © 2009 Lars-Peter Clausen <lars@metafoo.de>
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
 */

#ifndef __GLAMO_DRMPRIV_H
#define __GLAMO_DRMPRIV_H


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <linux/mfd/glamo-core.h>


/* Memory to allocate for the framebuffer.
 * The rest is reserved for the DRM memory manager */
#define GLAMO_FRAMEBUFFER_ALLOCATION (2*480*640)


struct glamodrm_handle {

	/* This device */
	struct device *dev;

	/* The parent device handle */
	struct glamo_core *glamo_core;

	/* Framebuffer handle for the console (i.e. /dev/fb0) */
	struct fb_info *fb;

	/* Command queue registers */
	struct resource *reg;
	char __iomem *reg_base;

	/* VRAM region */
	struct resource *vram;

	/* Command queue region */
	char __iomem *cmdq_base;
	unsigned long cmdq_offs;

	/* LCD controller registers */
	struct resource *lcd_regs;
	char __iomem *lcd_base;

	/* 2D engine registers and IRQ */
	struct resource *twod_regs;
	char __iomem *twod_base;
	unsigned int twod_irq;

	ssize_t vram_size;

	/* Memory management */
	struct drm_mm *mmgr;

	/* semaphore against concurrent ioctl */
	struct semaphore add_to_ring;

	/* Saved state */
	u_int16_t saved_clock;
	u_int16_t saved_width;
	u_int16_t saved_height;
	u_int16_t saved_pitch;
	u_int16_t saved_htotal;
	u_int16_t saved_hrtrst;
	u_int16_t saved_hrtren;
	u_int16_t saved_hdspst;
	u_int16_t saved_hdspen;
	u_int16_t saved_vtotal;
	u_int16_t saved_vrtrst;
	u_int16_t saved_vrtren;
	u_int16_t saved_vdspst;
	u_int16_t saved_vdspen;

	/* Fencing */
	atomic_t curr_seq;              /* The last used stamp number */
	struct list_head fence_list;    /* List of active fences */
	rwlock_t fence_list_lock;       /* Lock to protect fence_list */
	wait_queue_head_t fence_queue;  /* Waitqueue */
	struct tasklet_struct fence_tl; /* Tasklet for fence IRQ */

	/* A scratch block */
	struct drm_mm_node *scratch;

	/* We only have one */
	struct drm_crtc *crtc;
};


/* Private data.  This is where we keep our memory management bits */
struct drm_glamo_gem_object {
	struct drm_gem_object *obj;	/* The GEM object this refers to */
	struct drm_mm_node *block;	/* Block handle for drm_mm */
	uint64_t mmap_offset;
};


struct glamo_crtc {

	struct drm_crtc base;
	struct glamodrm_handle *gdrm;
	/* a mode_set for fbdev users on this crtc */
	struct drm_mode_set mode_set;
	int blank_mode;

	int pixel_clock_on;

	int current_mode_set;
	struct drm_display_mode current_mode;
	struct drm_framebuffer *current_fb;
};


struct glamo_framebuffer {
	struct drm_framebuffer base;
	struct drm_gem_object *obj;
};


struct glamo_output {
	struct drm_connector base;
	struct drm_encoder enc;
	struct glamodrm_handle *gdrm;
};


/* Colour mode for KMS framebuffer */
enum {
	GLAMO_FB_RGB565,
	GLAMO_FB_ARGB1555,
	GLAMO_FB_ARGB4444
};


#define to_glamo_crtc(x) container_of(x, struct glamo_crtc, base)
#define to_glamo_output(x) container_of(x, struct glamo_output, base)
#define enc_to_glamo_output(x) container_of(x, struct glamo_output, enc)
#define to_glamo_framebuffer(x) container_of(x, struct glamo_framebuffer, base)


#endif /* __GLAMO_DRMPRIV_H */
