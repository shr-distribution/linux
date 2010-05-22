/* Smedia Glamo 336x/337x Display
 *
 * Copyright (c) 2008-2009 Thomas White <taw@bitwiz.org.uk>
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

#ifndef __GLAMO_DISPLAY_H
#define __GLAMO_DISPLAY_H

#include <drm/drmP.h>
#include "glamo-drm-private.h"

extern int glamo_display_init(struct drm_device *dev);

extern int glamo_framebuffer_create(struct drm_device *dev,
                                    struct drm_mode_fb_cmd *mode_cmd,
                                    struct drm_framebuffer **fb,
                                    struct drm_gem_object *obj);

extern void glamo_display_suspend(struct glamodrm_handle *gdrm);
extern void glamo_display_resume(struct glamodrm_handle *gdrm);

extern void glamo_lcd_power(struct glamodrm_handle *gdrm, int mode);

#endif /* __GLAMO_DISPLAY_H */
