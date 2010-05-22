/*
 * SMedia Glamo 336x/337x KMS framebuffer
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
 */

#ifndef __GLAMO_KMS_FB_H
#define __GLAMO_KMS_FB_H

#include <drm/drmP.h>
#include "glamo-drm-private.h"

extern int glamofb_create(struct drm_device *dev, uint32_t fb_width,
                          uint32_t fb_height, uint32_t surface_width,
                          uint32_t surface_height, int colour_mode,
                          struct glamo_framebuffer **glamo_fb_p);

extern void glamo_kmsfb_suspend(struct glamodrm_handle *gdrm);
extern void glamo_kmsfb_resume(struct glamodrm_handle *gdrm);

#endif /* __GLAMO_KMS_FB_H */
