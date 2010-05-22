/*
 * SMedia Glamo 336x/337x KMS Framebuffer
 *
 * Copyright (C) 2009 Thomas White <taw@bitwiz.org.uk>
 *
 * Based on glamo-fb.c (C) 2007-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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
 *
 * Based on intel_fb.c from drivers/gpu/drm/i915
 *  to which the following licence applies:
 *
 * Copyright © 2006-2007 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *
 */


#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_crtc.h>

#include <linux/mfd/glamo-core.h>

#include "glamo-drm-private.h"
#include "glamo-display.h"
#include "glamo-buffer.h"


struct glamofb_par {
	struct drm_device *dev;
	struct drm_display_mode *our_mode;
	struct glamo_framebuffer *glamo_fb;
	int crtc_count;
	/* crtc currently bound to this */
	uint32_t crtc_ids[2];
};


static int glamofb_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp,
			struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_crtc *crtc;
	int i;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);
		struct drm_mode_set *modeset = &glamo_crtc->mode_set;
		struct drm_framebuffer *fb = modeset->fb;

		for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;


		if (regno > 255)
			return 1;

		if (regno < 16) {
			switch (fb->depth) {
			case 15:
				fb->pseudo_palette[regno] = ((red & 0xf800) >> 1) |
					((green & 0xf800) >>  6) |
					((blue & 0xf800) >> 11);
				break;
			case 16:
				fb->pseudo_palette[regno] = (red & 0xf800) |
					((green & 0xfc00) >>  5) |
					((blue  & 0xf800) >> 11);
				break;
			case 24:
			case 32:
				fb->pseudo_palette[regno] = ((red & 0xff00) << 8) |
					(green & 0xff00) |
					((blue  & 0xff00) >> 8);
				break;
			}
		}
	}
	return 0;
}

static int glamofb_check_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct glamo_framebuffer *glamo_fb = par->glamo_fb;
	struct drm_framebuffer *fb = &glamo_fb->base;
	int depth;

	/* Need to resize the fb object !!! */
	if (var->xres > fb->width || var->yres > fb->height) {
		DRM_ERROR("Cannot resize framebuffer object (%dx%d > %dx%d)\n",
		          var->xres,var->yres,fb->width,fb->height);
		DRM_ERROR("Need resizing code.\n");
		return -EINVAL;
	}

	switch (var->bits_per_pixel) {
	case 16:
		depth = (var->green.length == 6) ? 16 : 15;
		break;
	case 32:
		depth = (var->transp.length > 0) ? 32 : 24;
		break;
	default:
		depth = var->bits_per_pixel;
		break;
	}

	switch (depth) {
	case 16:
		var->red.offset = 11;
		var->green.offset = 5;
		var->blue.offset = 0;
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
		var->transp.length = 0;
		var->transp.offset = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* this will let fbcon do the mode init */
/* FIXME: take mode config lock? */
static int glamofb_set_par(struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct fb_var_screeninfo *var = &info->var;
	int i;
	struct drm_crtc *crtc;
	int ret;

	DRM_DEBUG("%d %d\n", var->xres, var->pixclock);

	if (var->pixclock != -1) {
		DRM_ERROR("Warning: userspace gave me a pixel clock value (%i)"
		          "- I'm ignoring it.\n", var->pixclock);
	}

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);

		for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;

		if (crtc->fb == glamo_crtc->mode_set.fb) {
			mutex_lock(&dev->mode_config.mutex);
			ret = crtc->funcs->set_config(&glamo_crtc->mode_set);
			mutex_unlock(&dev->mode_config.mutex);
			if (ret)
				return ret;
		}
	}
	return 0;
}

static int glamofb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_mode_set *modeset;
	struct drm_crtc *crtc;
	struct glamo_crtc *glamo_crtc;
	int ret = 0;
	int i;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;

		glamo_crtc = to_glamo_crtc(crtc);
		modeset = &glamo_crtc->mode_set;

		modeset->x = var->xoffset;
		modeset->y = var->yoffset;

		if (modeset->num_connectors) {
			mutex_lock(&dev->mode_config.mutex);
			ret = crtc->funcs->set_config(modeset);
			mutex_unlock(&dev->mode_config.mutex);
			if (!ret) {
				info->var.xoffset = var->xoffset;
				info->var.yoffset = var->yoffset;
			}
		}
	}

	return ret;
}

static void glamofb_on(struct fb_info *info)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct glamodrm_handle *gdrm = dev->dev_private;

	gdrm = dev->dev_private;

	glamo_lcd_power(gdrm, 1);
}

static void glamofb_off(struct fb_info *info, int dpms_mode)
{
	struct glamofb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct glamodrm_handle *gdrm = dev->dev_private;

	glamo_lcd_power(gdrm, 0);
}

static int glamofb_blank(int blank, struct fb_info *info)
{
	switch (blank) {
	case FB_BLANK_UNBLANK:
		glamofb_on(info);
		break;
	case FB_BLANK_NORMAL:
		glamofb_off(info, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		glamofb_off(info, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		glamofb_off(info, DRM_MODE_DPMS_SUSPEND);
		break;
	case FB_BLANK_POWERDOWN:
		glamofb_off(info, DRM_MODE_DPMS_OFF);
		break;
	}
	return 0;
}

static struct fb_ops glamofb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = glamofb_check_var,
	.fb_set_par = glamofb_set_par,
	.fb_setcolreg = glamofb_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_pan_display = glamofb_pan_display,
	.fb_blank = glamofb_blank,
};


#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)


/* Here, we create a GEM object of the correct size, and then turn it into
 * /dev/fbX so that the kernel can put a console on it. */
int glamofb_create(struct drm_device *dev, uint32_t fb_width,
                   uint32_t fb_height, uint32_t surface_width,
                   uint32_t surface_height, int colour_mode,
                   struct glamo_framebuffer **glamo_fb_p)
{
	struct fb_info *info;
	struct glamofb_par *par;
	struct drm_framebuffer *fb;
	struct glamo_framebuffer *glamo_fb;
	struct drm_mode_fb_cmd mode_cmd;
	struct drm_gem_object *fbo = NULL;
	struct drm_glamo_gem_object *gobj;
	struct device *device = &dev->platform_dev->dev;
	struct glamodrm_handle *gdrm;
	int size, ret;
	unsigned long offs;

	gdrm = dev->dev_private;

	mode_cmd.width = surface_width;
	mode_cmd.height = surface_height;

	mode_cmd.bpp = 16;
	mode_cmd.pitch = ALIGN(mode_cmd.width * ((mode_cmd.bpp + 1) / 8), 64);
	mode_cmd.depth = 16;

	size = mode_cmd.pitch * mode_cmd.height;
	size = ALIGN(size, PAGE_SIZE);
	if ( size > GLAMO_FRAMEBUFFER_ALLOCATION ) {
		printk(KERN_ERR "[glamo-drm] Not enough memory for fb\n");
		ret = -ENOMEM;
		goto out;
	}
	fbo = glamo_gem_object_alloc(dev, GLAMO_FRAMEBUFFER_ALLOCATION, 2);
	if (!fbo) {
		printk(KERN_ERR "[glamo-drm] Failed to allocate framebuffer\n");
		ret = -ENOMEM;
		goto out;
	}
	gobj = fbo->driver_private;

	mutex_lock(&dev->struct_mutex);

	ret = glamo_framebuffer_create(dev, &mode_cmd, &fb, fbo);
	if (ret) {
		DRM_ERROR("failed to allocate fb.\n");
		goto out_unref;
	}

	list_add(&fb->filp_head, &dev->mode_config.fb_kernel_list);

	glamo_fb = to_glamo_framebuffer(fb);
	*glamo_fb_p = glamo_fb;

	info = framebuffer_alloc(sizeof(struct glamofb_par), device);
	if (!info) {
		ret = -ENOMEM;
		goto out_unref;
	}

	par = info->par;

	strcpy(info->fix.id, "glamodrmfb");
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1; /* doing it in hw */
	info->fix.ypanstep = 1; /* doing it in hw */
	info->fix.ywrapstep = 0;
	info->fix.accel = FB_ACCEL_GLAMO;
	info->fix.type_aux = 0;
	info->flags = FBINFO_DEFAULT;

	info->fbops = &glamofb_ops;

	info->fix.line_length = fb->pitch;

	info->flags = FBINFO_DEFAULT;

	offs = gobj->block->start;
	info->screen_base = ioremap(gdrm->vram->start + offs + GLAMO_OFFSET_FB,
	                            GLAMO_FRAMEBUFFER_ALLOCATION);
	if (!info->screen_base) {
		printk(KERN_ERR "[glamo-drm] Couldn't map framebuffer!\n");
		ret = -ENOSPC;
		goto out_unref;
	}
	info->fix.smem_start = (unsigned long)gdrm->vram->start + offs;
	info->fix.smem_len = size;
	info->screen_size = size;

	info->pseudo_palette = fb->pseudo_palette;
	info->var.xres_virtual = fb->width;
	info->var.yres_virtual = fb->height;
	info->var.bits_per_pixel = fb->bits_per_pixel;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = -1;
	info->var.width = -1;
	info->var.xres = fb_width;
	info->var.yres = fb_height;

	info->fix.mmio_start = 0;
	info->fix.mmio_len = 0;

	info->pixmap.size = 64*1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;

	switch (fb->depth) {
	case 16:
		switch ( colour_mode ) {
		case GLAMO_FB_RGB565:
			info->var.red.offset	= 11;
			info->var.green.offset	= 5;
			info->var.blue.offset	= 0;
			info->var.red.length	= 5;
			info->var.green.length	= 6;
			info->var.blue.length	= 5;
			info->var.transp.length	= 0;
			break;
		case GLAMO_FB_ARGB1555:
			info->var.transp.offset	= 15;
			info->var.red.offset	= 10;
			info->var.green.offset	= 5;
			info->var.blue.offset	= 0;
			info->var.transp.length	= 1;
			info->var.red.length	= 5;
			info->var.green.length	= 5;
			info->var.blue.length	= 5;
			break;
		case GLAMO_FB_ARGB4444:
			info->var.transp.offset	= 12;
			info->var.red.offset	= 8;
			info->var.green.offset	= 4;
			info->var.blue.offset	= 0;
			info->var.transp.length	= 4;
			info->var.red.length	= 4;
			info->var.green.length	= 4;
			info->var.blue.length	= 4;
			break;
		}
		break;
	case 24:
	case 32:
	default:
		/* The Smedia Glamo doesn't support anything but 16bit color */
		printk(KERN_ERR "[glamo-drm] Only 16bpp is supported.\n");
		return -EINVAL;
	}

	fb->fbdev = info;
	par->glamo_fb = glamo_fb;
	par->dev = dev;
	gdrm->fb = info;

	info->var.pixclock = -1;

	printk(KERN_INFO "[glamo-drm] Allocated %dx%d fb: bo %p\n",
	       glamo_fb->base.width, glamo_fb->base.height, fbo);
	mutex_unlock(&dev->struct_mutex);
	return 0;

out_unref:
	drm_gem_object_unreference(fbo);
	mutex_unlock(&dev->struct_mutex);
out:
	return ret;
}


void glamo_kmsfb_suspend(struct glamodrm_handle *gdrm)
{
	fb_set_suspend(gdrm->fb, 1);
}


void glamo_kmsfb_resume(struct glamodrm_handle *gdrm)
{
	fb_set_suspend(gdrm->fb, 0);
}
