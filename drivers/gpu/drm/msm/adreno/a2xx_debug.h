/* SPDX-License-Identifier: GPL-2.0 */
/*
 * A2XX GPU Debug Module Header
 */

#ifndef __A2XX_DEBUG_H__
#define __A2XX_DEBUG_H__

#include <drm/drm_drv.h>

struct msm_gpu;
struct msm_gem_submit;

#ifdef CONFIG_DEBUG_FS

void a2xx_debug_log_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit);
void a2xx_debug_log_retire(struct msm_gpu *gpu, u32 seqno);
void a2xx_debug_cache_flush(void);
int a2xx_debug_get_wptr_delay(void);
void a2xx_debug_init(struct drm_minor *minor);

#else

static inline void a2xx_debug_log_submit(struct msm_gpu *gpu,
					 struct msm_gem_submit *submit) {}
static inline void a2xx_debug_log_retire(struct msm_gpu *gpu, u32 seqno) {}
static inline void a2xx_debug_cache_flush(void) {}
static inline int a2xx_debug_get_wptr_delay(void) { return 0; }
static inline void a2xx_debug_init(struct drm_minor *minor) {}

#endif /* CONFIG_DEBUG_FS */

#endif /* __A2XX_DEBUG_H__ */
