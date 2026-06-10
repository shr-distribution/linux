/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#ifndef __MDP4_KMS_H__
#define __MDP4_KMS_H__

#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#include "msm_drv.h"
#include "msm_kms.h"
#include "disp/mdp_kms.h"
#include "mdp4.xml.h"

struct device_node;
struct icc_path;

struct mdp4_kms {
	struct mdp_kms base;

	struct drm_device *dev;

	int rev;

	void __iomem *mmio;

	struct regulator *vdd;

	struct clk *clk;
	struct clk *pclk;
	struct clk *lut_clk;
	struct clk *axi_clk;
	struct clk *vsync_clk;

	/*
	 * LVDS PHY state save/restore across system sleep.
	 * On MSM8x60 the LVDS PHY (PLL CTRL_0..9, INTF_CTL, MUX_CTL[4],
	 * CFG0/CFG2) is initialized by the bootloader at boot but loses its
	 * configuration across s2idle even though mdp_gdsc has the
	 * ALWAYS_ON workaround. The internal mdp4_lvds_pll clk_hw is
	 * never registered on MSM8x60 (mdp4_get_lcdc_clock prefers
	 * MMCC's lcdc_clk), so the framework can't fire a re-init.
	 * We snapshot the working state at the first runtime_resume
	 * (when bootloader-programmed values are intact) and replay it
	 * on every subsequent runtime_resume to rearm the LVDS
	 * transmitter (otherwise INTF_CTL ENABLE+LANE_EN bits stay 0
	 * and the panel sees no pixels).
	 */
	bool lvds_state_saved;
	u32 lvds_intf_ctl;
	u32 lvds_mux_ctl[4];
	u32 lvds_phy_cfg0;
	u32 lvds_phy_cfg2;
	u32 lvds_phy_pll_ctrl[10];

	/*
	 * Interconnect paths from the two MDP master ports out to memory.
	 * MDP framebuffers live in EBI on tenderloin, so the EBI-targeted
	 * paths carry real bandwidth; SMI paths are kept for completeness
	 * but only voted at zero (so they keep mapping but don't add to
	 * fabric clock or arbitration pressure).
	 */
	struct icc_path *path_ebi[2];
	struct icc_path *path_smi[2];
	u32 icc_avg_bw_kbps;
	u32 icc_peak_bw_kbps;

	struct mdp_irq error_handler;

	/*
	 * INTF-underrun IRQ storm mitigation. PRIMARY_INTF_UDERRUN is a level
	 * condition: a *continuous* underrun (e.g. the DMA_P scanout left
	 * underflowing after a KMS client-switch modeset) re-asserts the IRQ
	 * every scanline, so mdp4_irq() runs nonstop and the CPU is pinned in
	 * interrupt -> the device hard-wedges (netconsole dead-stops, no
	 * khungtaskd). The IRQ is only used for logging, so when it storms we
	 * temporarily mask it (clear it from error_handler.irqmask) and re-arm
	 * after a delay, capping its rate so it can never lock up the CPU.
	 */
	unsigned long underrun_window_start;
	unsigned int underrun_count;
	struct delayed_work underrun_rearm_work;

	bool rpm_enabled;

	/* empty/blank cursor bo to use when cursor is "disabled" */
	struct drm_gem_object *blank_cursor_bo;
	uint64_t blank_cursor_iova;

	/*
	 * Permanently-mapped black scratch bo that a DISABLED pipe's
	 * SRCP0_BASE points at instead of 0. If a pipe-disable fails to latch
	 * (e.g. an underrun stalls the LCDC and DMA_P stops generating vsync,
	 * so the flush never completes), the still-staged pipe keeps fetching
	 * its base address. Pointing it at a valid mapped page makes that
	 * stray fetch read black instead of faulting the display IOMMU at an
	 * unmapped iova (the FAR=0x780 fault storm that wedged the device).
	 */
	struct drm_gem_object *blank_pipe_bo;
	uint64_t blank_pipe_iova;
};
#define to_mdp4_kms(x) container_of(x, struct mdp4_kms, base)

static inline void mdp4_write(struct mdp4_kms *mdp4_kms, u32 reg, u32 data)
{
	writel(data, mdp4_kms->mmio + reg);
}

static inline u32 mdp4_read(struct mdp4_kms *mdp4_kms, u32 reg)
{
	return readl(mdp4_kms->mmio + reg);
}

static inline uint32_t pipe2flush(enum mdp4_pipe pipe)
{
	switch (pipe) {
	case VG1:      return MDP4_OVERLAY_FLUSH_VG1;
	case VG2:      return MDP4_OVERLAY_FLUSH_VG2;
	case RGB1:     return MDP4_OVERLAY_FLUSH_RGB1;
	case RGB2:     return MDP4_OVERLAY_FLUSH_RGB2;
	default:       return 0;
	}
}

static inline uint32_t ovlp2flush(int ovlp)
{
	switch (ovlp) {
	case 0:        return MDP4_OVERLAY_FLUSH_OVLP0;
	case 1:        return MDP4_OVERLAY_FLUSH_OVLP1;
	default:       return 0;
	}
}

static inline uint32_t dma2irq(enum mdp4_dma dma)
{
	switch (dma) {
	case DMA_P:    return MDP4_IRQ_DMA_P_DONE;
	case DMA_S:    return MDP4_IRQ_DMA_S_DONE;
	case DMA_E:    return MDP4_IRQ_DMA_E_DONE;
	default:       return 0;
	}
}

static inline uint32_t dma2err(enum mdp4_dma dma)
{
	switch (dma) {
	case DMA_P:    return MDP4_IRQ_PRIMARY_INTF_UDERRUN;
	case DMA_S:    return 0;  // ???
	case DMA_E:    return MDP4_IRQ_EXTERNAL_INTF_UDERRUN;
	default:       return 0;
	}
}

static inline uint32_t mixercfg(uint32_t mixer_cfg, int mixer,
		enum mdp4_pipe pipe, enum mdp_mixer_stage_id stage)
{
	switch (pipe) {
	case VG1:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE0__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE0_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE0(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE0_MIXER1);
		break;
	case VG2:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE1__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE1_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE1(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE1_MIXER1);
		break;
	case RGB1:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE2__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE2_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE2(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE2_MIXER1);
		break;
	case RGB2:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE3__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE3_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE3(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE3_MIXER1);
		break;
	case RGB3:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE4__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE4_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE4(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE4_MIXER1);
		break;
	case VG3:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE5__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE5_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE5(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE5_MIXER1);
		break;
	case VG4:
		mixer_cfg &= ~(MDP4_LAYERMIXER_IN_CFG_PIPE6__MASK |
				MDP4_LAYERMIXER_IN_CFG_PIPE6_MIXER1);
		mixer_cfg |= MDP4_LAYERMIXER_IN_CFG_PIPE6(stage) |
			COND(mixer == 1, MDP4_LAYERMIXER_IN_CFG_PIPE6_MIXER1);
		break;
	default:
		WARN(1, "invalid pipe");
		break;
	}

	return mixer_cfg;
}

int mdp4_disable(struct mdp4_kms *mdp4_kms);
int mdp4_enable(struct mdp4_kms *mdp4_kms);

void mdp4_set_irqmask(struct mdp_kms *mdp_kms, uint32_t irqmask,
		uint32_t old_irqmask);
void mdp4_irq_preinstall(struct msm_kms *kms);
int mdp4_irq_postinstall(struct msm_kms *kms);
void mdp4_irq_uninstall(struct msm_kms *kms);
irqreturn_t mdp4_irq(struct msm_kms *kms);
int mdp4_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc);
void mdp4_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc);

static inline uint32_t mdp4_pipe_caps(enum mdp4_pipe pipe)
{
	switch (pipe) {
	case VG1:
	case VG2:
	case VG3:
	case VG4:
		return MDP_PIPE_CAP_HFLIP | MDP_PIPE_CAP_VFLIP |
				MDP_PIPE_CAP_SCALE | MDP_PIPE_CAP_CSC;
	case RGB1:
	case RGB2:
	case RGB3:
		return MDP_PIPE_CAP_SCALE;
	default:
		return 0;
	}
}

enum mdp4_pipe mdp4_plane_pipe(struct drm_plane *plane);
struct drm_plane *mdp4_plane_init(struct drm_device *dev,
		enum mdp4_pipe pipe_id, bool private_plane);

uint32_t mdp4_crtc_vblank(struct drm_crtc *crtc);
void mdp4_crtc_set_config(struct drm_crtc *crtc, uint32_t config);
void mdp4_crtc_set_intf(struct drm_crtc *crtc, enum mdp4_intf intf, int mixer);
void mdp4_crtc_wait_for_commit_done(struct drm_crtc *crtc);
struct drm_crtc *mdp4_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, int ovlp_id,
		enum mdp4_dma dma_id);

long mdp4_dtv_round_pixclk(struct drm_encoder *encoder, unsigned long rate);
struct drm_encoder *mdp4_dtv_encoder_init(struct drm_device *dev);

struct drm_encoder *mdp4_lcdc_encoder_init(struct drm_device *dev);

#ifdef CONFIG_DRM_MSM_DSI
struct drm_encoder *mdp4_dsi_encoder_init(struct drm_device *dev);
#else
static inline struct drm_encoder *mdp4_dsi_encoder_init(struct drm_device *dev)
{
	return ERR_PTR(-ENODEV);
}
#endif

struct clk *mdp4_get_lcdc_clock(struct drm_device *dev);

#endif /* __MDP4_KMS_H__ */
