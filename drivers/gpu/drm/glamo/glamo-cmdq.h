/* Smedia Glamo 336x/337x command queue handling
 *
 * Copyright (c) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Copyright (c) 2009 Andreas Pokorny <andreas.pokorny@gmail.com>
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

#ifndef __GLAMO_CMDQ_H
#define __GLAMO_CMDQ_H

#include <drm/drmP.h>

#include "glamo-drm-private.h"

extern int glamo_ioctl_cmdbuf(struct drm_device *dev, void *data,
			      struct drm_file *file_priv);
extern int glamo_ioctl_cmdburst(struct drm_device *dev, void *data,
			        struct drm_file *file_priv);
extern void glamo_cmdq_blank(struct glamodrm_handle *gdrm,
                             struct drm_gem_object *obj);

extern int glamo_cmdq_init(struct drm_device *de);
extern int glamo_cmdq_shutdown(struct glamodrm_handle *gdrm);
extern void glamo_cmdq_suspend(struct glamodrm_handle *gdrm);
extern void glamo_cmdq_resume(struct glamodrm_handle *gdrm);

extern int glamo_add_to_ring(struct glamodrm_handle *gdrm, u16 *addr,
                             unsigned int count);

#endif /* __GLAMO_CMDQ_H */
