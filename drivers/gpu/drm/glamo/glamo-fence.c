/*
 * SMedia Glamo 336x/337x fence objects
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
 * Loosely based on radeon_fence.c, to which the following notice applies:
 *
 * Copyright 2009 Jerome Glisse.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 */
/*
 * Authors:
 *    Jerome Glisse <glisse@freedesktop.org>
 *    Dave Airlie
 */


#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/lockdep.h>

#include <linux/mfd/glamo-core.h>
#include <linux/mfd/glamo-regs.h>

#include "glamo-drm-private.h"
#include "glamo-cmdq.h"


static struct lock_class_key glamo_fence_lock_key;


struct glamo_fence
{
	struct list_head list;
	uint16_t               seq;       /* Wait for at least this ID */
	int                    signalled; /* Non-zero when fence has passed */
	struct glamodrm_handle *gdrm;
};


static void glamo_fence_emit(struct glamo_fence *fence)
{
	u16 fring[6];

	fring[0] = 0x8000 | GLAMO_REG_2D_ID1;
	fring[1] = 3;
	fence->seq = atomic_inc_return(&fence->gdrm->curr_seq);
	if ( fence->seq > 1<<14 ) {
		atomic_set(&fence->gdrm->curr_seq, 0);
		fence->seq = atomic_inc_return(&fence->gdrm->curr_seq);
	}
	fring[2] = 1<<15 | fence->seq;
	fring[3] = 0;  /* Unused */
	fring[4] = 0;  /* Unused */
	fring[5] = 0;  /* Padding */

	glamo_add_to_ring(fence->gdrm, fring, 12);
}


static void glamo_fence_enable(struct glamodrm_handle *gdrm)
{
	enable_irq( GLAMO_IRQ_2D);
}


static inline u16 reg_read_2d(struct glamodrm_handle *gdrm, u_int16_t reg)
{
	/* For command queue, the address is given relative to
	 * the overall base of Glamo.  This isn't the case here. */
	return ioread16(gdrm->twod_base + reg-GLAMO_REGOFS_2D);
}


static inline u16 reg_read_cmdq(struct glamodrm_handle *gdrm, u_int16_t reg)
{
	return ioread16(gdrm->reg_base + reg);
}


static void glamo_cmdq_wait(struct glamodrm_handle *gdrm,
                            enum glamo_engine engine)
{
	u16 mask, val, status;
	int i;

	switch (engine)
	{
		case GLAMO_ENGINE_ALL:
			mask = 1 << 2;
			val  = mask;
			break;
		default:
			return;
	}

	for ( i=0; i<1000; i++ ) {
		status = reg_read_cmdq(gdrm, GLAMO_REG_CMDQ_STATUS);
		if ((status & mask) == val) break;
		mdelay(1);
	}
	if ( i == 1000 ) {
		size_t ring_read;
		printk(KERN_WARNING "[glamo-drm] CmdQ timeout!\n");
		printk(KERN_WARNING "[glamo-drm] status = %x\n", status);
		ring_read = reg_read_cmdq(gdrm, GLAMO_REG_CMDQ_READ_ADDRL);
		ring_read |= ((reg_read_cmdq(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
				& 0x7) << 16);
		printk(KERN_INFO "[glamo-drm] ring_read now 0x%x\n",
				 ring_read);
	}
}


static irqreturn_t glamo_fence_irq_handler(int irq, void *data)
{
	struct glamodrm_handle *gdrm = data;
	if (!gdrm) return IRQ_NONE;
	tasklet_schedule(&gdrm->fence_tl);
	return IRQ_HANDLED;
}


/* This is nasty.  I'm sorry. */
static void glamo_fence_debodge(struct glamodrm_handle *gdrm)
{
	struct list_head *tmp;

	printk(KERN_ERR "[glamo-drm] Attempting to recover...\n");

	glamo_cmdq_wait(gdrm, GLAMO_ENGINE_ALL);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_2D);

	read_lock(&gdrm->fence_list_lock);
	list_for_each(tmp, &gdrm->fence_list) {

		struct glamo_fence *fence;

		fence = list_entry(tmp, struct glamo_fence, list);

		if ( fence->signalled != 1 ) {
			printk(KERN_ERR "[glamo-drm] Fence seq#%i was not"
			                " signalled\n", fence->seq);
		}
		fence->signalled = 1;

	}
	read_unlock(&gdrm->fence_list_lock);

	wake_up_all(&gdrm->fence_queue);
}


static void glamo_fence_tl(unsigned long data)
{
	struct glamodrm_handle *gdrm = (struct glamodrm_handle *)data;
	int wake = 0;
	u16 seq;
	struct list_head *tmp;

	seq = reg_read_2d(gdrm, GLAMO_REG_2D_ID1) & 0x7fff;

	read_lock(&gdrm->fence_list_lock);
	list_for_each(tmp, &gdrm->fence_list) {

		struct glamo_fence *fence;

		fence = list_entry(tmp, struct glamo_fence, list);
		if ( seq >= fence->seq ) {
			fence->signalled = 1;
			wake = 1;
		}

	}
	read_unlock(&gdrm->fence_list_lock);

	if ( wake ) wake_up_all(&gdrm->fence_queue);
}


static struct glamo_fence *glamo_fence_new(struct glamodrm_handle *gdrm)
{
	struct glamo_fence *fence;
	unsigned long irq_flags;

	fence = kmalloc(sizeof(*fence), GFP_KERNEL);
	fence->signalled = 0;
	fence->gdrm = gdrm;

	/* Add to list */
	write_lock_irqsave(&gdrm->fence_list_lock, irq_flags);
	list_add(&fence->list, &gdrm->fence_list);
	write_unlock_irqrestore(&gdrm->fence_list_lock, irq_flags);

	return fence;
}


static struct glamo_fence *glamo_fence_destroy(struct glamo_fence *fence)
{
	unsigned long irq_flags;
	struct glamodrm_handle *gdrm = fence->gdrm;

	/* Remove from list */
	write_lock_irqsave(&gdrm->fence_list_lock, irq_flags);
	list_del(&fence->list);
	write_unlock_irqrestore(&gdrm->fence_list_lock, irq_flags);

	kfree(fence);

	return fence;
}


int glamo_ioctl_wait_rendering(struct drm_device *dev, void *data,
                               struct drm_file *file_priv)
{
	struct glamodrm_handle *gdrm;
	struct drm_glamo_gem_wait_rendering *args = data;
	struct glamo_fence *fence;
	int r;

	gdrm = dev->dev_private;

	if ( !args->have_handle ) {
		glamo_cmdq_wait(gdrm, GLAMO_ENGINE_ALL);
		return 0;
	}

	fence = glamo_fence_new(gdrm);
	if ( fence == NULL ) {
		printk(KERN_WARNING "[glamo-drm] Couldn't allocate fence -"
		                    " falling back to busy wait.\n");
		glamo_cmdq_wait(gdrm, GLAMO_ENGINE_ALL);
		return 0;
	}

	glamo_fence_emit(fence);

	/* Wait... */
	r = wait_event_interruptible_timeout(gdrm->fence_queue,
	                                     fence->signalled, HZ);
	if ( r == 0 ) {
		printk(KERN_ERR "[glamo-drm] Timeout!\n");
		glamo_fence_debodge(gdrm);
	}

	glamo_fence_destroy(fence);

	return 0;
}


void glamo_fence_init(struct glamodrm_handle *gdrm)
{
	unsigned long irq_flags;
	int r;

	if ( gdrm->twod_irq == 0 ) {
		printk(KERN_ERR "[glamo-drm] Don't know which IRQ to use!\n");
		return;
	}

	gdrm->fence_list_lock = __RW_LOCK_UNLOCKED(gdrm->fence_list_lock);
	lockdep_set_class(&gdrm->fence_list_lock, &glamo_fence_lock_key);
	init_waitqueue_head(&gdrm->fence_queue);

	atomic_set(&gdrm->curr_seq, 0);

	write_lock_irqsave(&gdrm->fence_list_lock, irq_flags);
	INIT_LIST_HEAD(&gdrm->fence_list);
	write_unlock_irqrestore(&gdrm->fence_list_lock, irq_flags);

	tasklet_init(&gdrm->fence_tl, glamo_fence_tl, (unsigned long)gdrm);

	r = request_irq(gdrm->twod_irq, glamo_fence_irq_handler,
	                IRQF_SHARED, "glamo-fence", gdrm);
	if ( r ) {
		printk(KERN_ERR "[glamo-drm] Failed to register irq.\n");
		return;
	}

	glamo_fence_enable(gdrm);
}


void glamo_fence_shutdown(struct glamodrm_handle *gdrm)
{
	free_irq(gdrm->twod_irq, gdrm);
	wake_up_all(&gdrm->fence_queue);
	tasklet_kill(&gdrm->fence_tl);
}
