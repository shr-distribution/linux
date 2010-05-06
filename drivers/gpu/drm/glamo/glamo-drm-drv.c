/* Smedia Glamo 336x/337x Graphics Driver
 *
 * Copyright (C) 2009 Openmoko, Inc. Jorge Luis Zapata <turran@openmoko.com>
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Copyright (C) 2009 Andreas Pokorny <andreas.pokorny@gmail.com>
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


#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <linux/glamofb.h>

#include <linux/mfd/glamo-core.h>

#include "glamo-cmdq.h"
#include "glamo-buffer.h"
#include "glamo-drm-private.h"
#include "glamo-display.h"
#include "glamo-kms-fb.h"
#include "glamo-fence.h"

#define DRIVER_AUTHOR           "Openmoko, Inc."
#define DRIVER_NAME             "glamo-drm"
#define DRIVER_DESC             "SMedia Glamo 3362"
#define DRIVER_DATE             "20090614"


static int glamo_ioctl_swap(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_swap\n");
	return 0;
}


static int glamo_ioctl_gem_info(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_info\n");
	return 0;
}


struct drm_ioctl_desc glamo_ioctls[] = {
	DRM_IOCTL_DEF(DRM_GLAMO_CMDBUF, glamo_ioctl_cmdbuf, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_SWAP, glamo_ioctl_swap, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_CMDBURST, glamo_ioctl_cmdburst, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_INFO, glamo_ioctl_gem_info, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_CREATE, glamo_ioctl_gem_create, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_MMAP, glamo_ioctl_gem_mmap, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_PIN, glamo_ioctl_gem_pin, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_UNPIN, glamo_ioctl_gem_unpin, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_PREAD, glamo_ioctl_gem_pread, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_PWRITE, glamo_ioctl_gem_pwrite, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_GLAMO_GEM_WAIT_RENDERING,
	              glamo_ioctl_wait_rendering, DRM_AUTH),
};


static int glamodrm_firstopen(struct drm_device *dev)
{
	DRM_DEBUG("\n");
	return 0;
}


static int glamodrm_open(struct drm_device *dev, struct drm_file *fh)
{
	DRM_DEBUG("\n");
	return 0;
}


static void glamodrm_preclose(struct drm_device *dev, struct drm_file *fh)
{
	DRM_DEBUG("\n");
}

static void glamodrm_postclose(struct drm_device *dev, struct drm_file *fh)
{
	DRM_DEBUG("\n");
}


static void glamodrm_lastclose(struct drm_device *dev)
{
	DRM_DEBUG("\n");
}


static int glamodrm_master_create(struct drm_device *dev,
				  struct drm_master *master)
{
	DRM_DEBUG("\n");

        return 0;
}


static void glamodrm_master_destroy(struct drm_device *dev,
				    struct drm_master *master)
{
	DRM_DEBUG("\n");
}


static int glamodrm_load(struct drm_device *dev, unsigned long flags)
{
	struct glamodrm_handle *gdrm;
	gdrm = dev->dev_private;

	glamo_buffer_init(gdrm);
	glamo_cmdq_init(dev);
	glamo_fence_init(gdrm);
	glamo_display_init(dev);

	return 0;
}


static int glamodrm_unload(struct drm_device *dev)
{
	struct glamodrm_handle *gdrm;

	gdrm = dev->dev_private;

	glamo_engine_disable(gdrm->glamo_core, GLAMO_ENGINE_2D);
	glamo_engine_disable(gdrm->glamo_core, GLAMO_ENGINE_3D);
	glamo_buffer_final(gdrm);
	glamo_fence_shutdown(gdrm);

	return 0;
}


static struct vm_operations_struct glamodrm_gem_vm_ops = {
	.fault = glamodrm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_driver glamodrm_drm_driver = {
	.driver_features = DRIVER_IS_PLATFORM | DRIVER_GEM | DRIVER_MODESET,
	.firstopen = glamodrm_firstopen,
	.load = glamodrm_load,
	.unload = glamodrm_unload,
	.open = glamodrm_open,
	.preclose = glamodrm_preclose,
	.postclose = glamodrm_postclose,
	.lastclose = glamodrm_lastclose,
	.reclaim_buffers = drm_core_reclaim_buffers,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
	.master_create = glamodrm_master_create,
	.master_destroy = glamodrm_master_destroy,
	.gem_init_object = glamodrm_gem_init_object,
	.gem_free_object = glamodrm_gem_free_object,
	.gem_vm_ops = &glamodrm_gem_vm_ops,
	.ioctls = glamo_ioctls,
	.fops = {
		.owner = THIS_MODULE,
		.open = drm_open,
		.release = drm_release,
		.ioctl = drm_ioctl,
		.mmap = drm_gem_mmap,
		.poll = drm_poll,
		.fasync = drm_fasync,
	},
	.major = 0,
	.minor = 1,
	.patchlevel = 0,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
};


static int glamodrm_probe(struct platform_device *pdev)
{
	int rc;
	struct glamodrm_handle *gdrm;
	struct glamo_core *core = dev_get_drvdata(pdev->dev.parent);

	printk(KERN_INFO "[glamo-drm] SMedia Glamo Direct Rendering Support\n");

	gdrm = kzalloc(sizeof(*gdrm), GFP_KERNEL);
	if ( !gdrm )
		return -ENOMEM;
	platform_set_drvdata(pdev, gdrm);
	gdrm->glamo_core = core;
	gdrm->dev = &pdev->dev;

	/* Find the command queue registers */
	gdrm->reg = platform_get_resource_byname(pdev, IORESOURCE_MEM,
	                                         "glamo-cmdq-regs");
	if ( !gdrm->reg ) {
		dev_err(&pdev->dev, "Unable to find cmdq registers.\n");
		rc = -ENOENT;
		goto out_free;
	}
	gdrm->reg = request_mem_region(gdrm->reg->start,
					  resource_size(gdrm->reg), pdev->name);
	if ( !gdrm->reg ) {
		dev_err(&pdev->dev, "failed to request MMIO region\n");
		rc = -ENOENT;
		goto out_free;
	}
	gdrm->reg_base = ioremap_nocache(gdrm->reg->start,
	                                 resource_size(gdrm->reg));
	if ( !gdrm->reg_base ) {
		dev_err(&pdev->dev, "failed to ioremap() MMIO registers\n");
		rc = -ENOENT;
		goto out_release_regs;
	}

	/* Find the VRAM */
	gdrm->vram = platform_get_resource_byname(pdev, IORESOURCE_MEM,
	                                          "glamo-fb-mem");
	if ( !gdrm->vram ) {
		dev_err(&pdev->dev, "Unable to find VRAM.\n");
		rc = -ENOENT;
		goto out_unmap_regs;
	}
	gdrm->vram = request_mem_region(gdrm->vram->start,
	                                resource_size(gdrm->vram), pdev->name);
	if ( !gdrm->vram ) {
		dev_err(&pdev->dev, "failed to request VRAM region\n");
		rc = -ENOENT;
		goto out_unmap_regs;
	}

	/* Find the LCD controller */
	gdrm->lcd_regs = platform_get_resource_byname(pdev, IORESOURCE_MEM,
	                                              "glamo-fb-regs");
	if ( !gdrm->lcd_regs ) {
		dev_err(&pdev->dev, "Unable to find LCD registers.\n");
		rc = -ENOENT;
		goto out_release_vram;
	}
	gdrm->lcd_regs = request_mem_region(gdrm->lcd_regs->start,
	                                    resource_size(gdrm->lcd_regs),
	                                    pdev->name);
	if ( !gdrm->lcd_regs ) {
		dev_err(&pdev->dev, "failed to request LCD registers\n");
		rc = -ENOENT;
		goto out_release_vram;
	}
	gdrm->lcd_base = ioremap_nocache(gdrm->lcd_regs->start,
	                                 resource_size(gdrm->lcd_regs));
	if ( !gdrm->lcd_base ) {
		dev_err(&pdev->dev, "failed to ioremap() LCD registers\n");
		rc = -ENOENT;
		goto out_release_lcd;
	}

	/* Find the 2D engine */
	gdrm->twod_regs = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	if ( !gdrm->twod_regs ) {
		dev_err(&pdev->dev, "Unable to find 2D registers.\n");
		rc = -ENOENT;
		goto out_unmap_lcd;
	}
	gdrm->twod_regs = request_mem_region(gdrm->twod_regs->start,
	                                     resource_size(gdrm->twod_regs),
	                                     pdev->name);
	if ( !gdrm->twod_regs ) {
		dev_err(&pdev->dev, "failed to request 2D registers\n");
		rc = -ENOENT;
		goto out_unmap_lcd;
	}
	gdrm->twod_base = ioremap(gdrm->twod_regs->start,
	                          resource_size(gdrm->twod_regs));
	if ( !gdrm->twod_base ) {
		dev_err(&pdev->dev, "failed to ioremap() 2D registers\n");
		rc = -ENOENT;
		goto out_release_2d;
	}

	/* Hook up IRQ handle for fence processing */
	gdrm->twod_irq = platform_get_irq_byname(pdev, "glamo-2d-irq");

	gdrm->vram_size = GLAMO_FB_SIZE;
	printk(KERN_INFO "[glamo-drm] %lli bytes of VRAM\n",
	                 (long long int)gdrm->vram_size);

	/* Initialise DRM */
	drm_platform_init(&glamodrm_drm_driver, pdev, (void *)gdrm);

	return 0;

out_release_2d:
	release_mem_region(gdrm->twod_regs->start,
	                   resource_size(gdrm->twod_regs));
out_unmap_lcd:
	iounmap(gdrm->lcd_base);
out_release_lcd:
	release_mem_region(gdrm->lcd_regs->start,
	                   resource_size(gdrm->lcd_regs));
out_release_vram:
	release_mem_region(gdrm->vram->start, resource_size(gdrm->vram));
out_unmap_regs:
	iounmap(gdrm->reg_base);
out_release_regs:
	release_mem_region(gdrm->reg->start, resource_size(gdrm->reg));
out_free:
	kfree(gdrm);
	dev_set_drvdata(&pdev->dev, NULL);
	return rc;
}


static int glamodrm_remove(struct platform_device *pdev)
{
	struct glamodrm_handle *gdrm = platform_get_drvdata(pdev);

	glamo_buffer_final(gdrm);
	glamo_cmdq_shutdown(gdrm);

	drm_exit(&glamodrm_drm_driver);

	platform_set_drvdata(pdev, NULL);

	/* Release registers */
	iounmap(gdrm->reg_base);
	release_mem_region(gdrm->reg->start, resource_size(gdrm->reg));

	/* Release VRAM */
	release_mem_region(gdrm->vram->start, resource_size(gdrm->vram));

	/* Release LCD registers */
	iounmap(gdrm->lcd_base);
	release_mem_region(gdrm->lcd_regs->start,
	                   resource_size(gdrm->lcd_regs));

	/* Release 2D engine  */
	iounmap(gdrm->twod_base);
	release_mem_region(gdrm->twod_regs->start,
	                   resource_size(gdrm->twod_regs));

	kfree(gdrm);

	return 0;
}


static int glamodrm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct glamodrm_handle *gdrm = platform_get_drvdata(pdev);

	glamo_kmsfb_suspend(gdrm);
	glamo_display_suspend(gdrm);
	glamo_cmdq_suspend(gdrm);

	/* glamo_core.c will suspend the engines for us */

	return 0;
}


static int glamodrm_resume(struct platform_device *pdev)
{
	struct glamodrm_handle *gdrm = platform_get_drvdata(pdev);

	glamo_cmdq_resume(gdrm);
	glamo_display_resume(gdrm);
	glamo_kmsfb_resume(gdrm);

	return 0;
}


static struct platform_driver glamodrm_driver = {
	.probe          = glamodrm_probe,
	.remove         = glamodrm_remove,
	.suspend	= glamodrm_suspend,
	.resume		= glamodrm_resume,
	.driver         = {
		.name   = "glamo-fb",
		.owner  = THIS_MODULE,
	},
};


static int __devinit glamodrm_init(void)
{
	glamodrm_drm_driver.num_ioctls = DRM_ARRAY_SIZE(glamo_ioctls);
	return platform_driver_register(&glamodrm_driver);
}


static void __exit glamodrm_exit(void)
{
	platform_driver_unregister(&glamodrm_driver);
}


module_init(glamodrm_init);
module_exit(glamodrm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
