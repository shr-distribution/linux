#ifndef __GLAMO_CORE_H
#define __GLAMO_CORE_H

#include <linux/mfd/glamo.h>

/* Amount of Glamo memory */
#define GLAMO_INTERNAL_RAM_SIZE 0x800000

/* Arbitrarily determined amount for the hardware cursor */
#define GLAMO_CURSOR_SIZE (4096)
#define GLAMO_MMC_BUFFER_SIZE (64 * 1024) /* 64k MMC buffer */
/* Remaining memory will be used for 2D and 3D graphics */
#define GLAMO_FB_SIZE (GLAMO_INTERNAL_RAM_SIZE     \
			 - GLAMO_CURSOR_SIZE       \
			 - GLAMO_MMC_BUFFER_SIZE)
/* A 640x480, 16bpp, double-buffered framebuffer */
#if (GLAMO_FB_SIZE < (640 * 480 * 4))	/* == 0x12c000 */
#error Not enough Glamo VRAM for framebuffer!
#endif

/* for the time being, we put the on-screen framebuffer into the lowest
 * VRAM space.  This should make the code easily compatible with the various
 * 2MB/4MB/8MB variants of the Smedia chips
 * glamo-fb.c assumes FB comes first, followed by cursor, so DON'T MOVE THEM
 * (see glamo_regs[] in glamo-fb.c for more information) */
#define GLAMO_MEM_BASE 		(0x800000)
#define GLAMO_OFFSET_FB		(0x000000)
#define GLAMO_OFFSET_CURSOR	(GLAMO_OFFSET_FB + GLAMO_FB_SIZE)
#define GLAMO_OFFSET_MMC	(GLAMO_OFFSET_CURSOR + GLAMO_CURSOR_SIZE)

enum glamo_pll {
	GLAMO_PLL1,
	GLAMO_PLL2,
};

enum glamo_engine_state {
	GLAMO_ENGINE_DISABLED,
	GLAMO_ENGINE_SUSPENDED,
	GLAMO_ENGINE_ENABLED,
};

struct glamo_core {
	int irq;
	int irq_base;
	struct resource *mem;
	void __iomem *base;
	struct platform_device *pdev;
	struct glamo_platform_data *pdata;
	enum glamo_engine_state engine_state[__NUM_GLAMO_ENGINES];
	spinlock_t lock;
	uint16_t saved_irq_mask;
	int slowed_divider;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
#endif
};

struct glamo_script {
	uint16_t reg;
	uint16_t val;
};

int glamo_pll_rate(struct glamo_core *glamo, enum glamo_pll pll);

int glamo_engine_enable(struct glamo_core *glamo, enum glamo_engine engine);
int glamo_engine_suspend(struct glamo_core *glamo, enum glamo_engine engine);
int glamo_engine_disable(struct glamo_core *glamo, enum glamo_engine engine);
void glamo_engine_reset(struct glamo_core *glamo, enum glamo_engine engine);
int glamo_engine_reclock(struct glamo_core *glamo,
			 enum glamo_engine engine, int ps);
void glamo_pixclock_slow (struct glamo_core *glamo);
void glamo_pixclock_fast (struct glamo_core *glamo);
#endif /* __GLAMO_CORE_H */
