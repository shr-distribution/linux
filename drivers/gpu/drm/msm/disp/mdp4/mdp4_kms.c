// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include <linux/delay.h>
#include <linux/interconnect.h>
#include <linux/of_reserved_mem.h>

#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_vblank.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"
#include "mdp4_kms.h"

static int mdp4_hw_init(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(to_mdp_kms(kms));
	struct drm_device *dev = mdp4_kms->dev;
	u32 dmap_cfg, vg_cfg;
	unsigned long clk;

	pm_runtime_get_sync(dev->dev);

	if (mdp4_kms->rev > 1) {
		mdp4_write(mdp4_kms, REG_MDP4_CS_CONTROLLER0, 0x0707ffff);
		mdp4_write(mdp4_kms, REG_MDP4_CS_CONTROLLER1, 0x03073f3f);
	}

	mdp4_write(mdp4_kms, REG_MDP4_PORTMAP_MODE, 0x3);

	/*
	 * Max read pending cmd config: Use 8 pending requests on APQ8060/MSM8660.
	 * webOS kernel: "if MDP clock >= AXI clock, use 8 pending". On APQ8060
	 * both are ~200 MHz, so 8 pending requests are needed to keep the memory
	 * bus saturated and prevent display underflow during USB activity.
	 * 0x08888 = 8 pending requests per client (DMA_P, DMA_E, VG pipes)
	 */
	mdp4_write(mdp4_kms, REG_MDP4_READ_CNFG, 0x08888);

	clk = clk_get_rate(mdp4_kms->clk);

	if ((mdp4_kms->rev >= 1) || (clk >= 90000000)) {
		dmap_cfg = 0x47;     /* 16 bytes-burst x 8 req */
		vg_cfg = 0x47;       /* 16 bytes-burs x 8 req */
	} else {
		dmap_cfg = 0x27;     /* 8 bytes-burst x 8 req */
		vg_cfg = 0x43;       /* 16 bytes-burst x 4 req */
	}

	DBG("fetch config: dmap=%02x, vg=%02x", dmap_cfg, vg_cfg);

	mdp4_write(mdp4_kms, REG_MDP4_DMA_FETCH_CONFIG(DMA_P), dmap_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_FETCH_CONFIG(DMA_E), dmap_cfg);

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(VG1), vg_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(VG2), vg_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(RGB1), vg_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(RGB2), vg_cfg);

	if (mdp4_kms->rev >= 2)
		mdp4_write(mdp4_kms, REG_MDP4_LAYERMIXER_IN_CFG_UPDATE_METHOD, 1);
	mdp4_write(mdp4_kms, REG_MDP4_LAYERMIXER_IN_CFG, 0);

	/* disable CSC matrix / YUV by default: */
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_OP_MODE(VG1), 0);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_OP_MODE(VG2), 0);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_P_OP_MODE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_S_OP_MODE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_CSC_CONFIG(1), 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_CSC_CONFIG(2), 0);

	if (mdp4_kms->rev > 1)
		mdp4_write(mdp4_kms, REG_MDP4_RESET_STATUS, 1);

	pm_runtime_put_sync(dev->dev);

	return 0;
}

static void mdp4_enable_commit(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(to_mdp_kms(kms));
	mdp4_enable(mdp4_kms);
}

static void mdp4_disable_commit(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(to_mdp_kms(kms));
	mdp4_disable(mdp4_kms);
}

static void mdp4_flush_commit(struct msm_kms *kms, unsigned crtc_mask)
{
	/* TODO */
}

static void mdp4_wait_flush(struct msm_kms *kms, unsigned crtc_mask)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(to_mdp_kms(kms));
	struct drm_crtc *crtc;

	for_each_crtc_mask(mdp4_kms->dev, crtc, crtc_mask)
		mdp4_crtc_wait_for_commit_done(crtc);
}

static void mdp4_complete_commit(struct msm_kms *kms, unsigned crtc_mask)
{
}

static long mdp4_round_pixclk(struct msm_kms *kms, unsigned long rate,
		struct drm_encoder *encoder)
{
	/* if we had >1 encoder, we'd need something more clever: */
	switch (encoder->encoder_type) {
	case DRM_MODE_ENCODER_TMDS:
		return mdp4_dtv_round_pixclk(encoder, rate);
	case DRM_MODE_ENCODER_LVDS:
	case DRM_MODE_ENCODER_DSI:
	default:
		return rate;
	}
}

static void mdp4_destroy(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(to_mdp_kms(kms));
	struct device *dev = mdp4_kms->dev->dev;

	if (mdp4_kms->blank_cursor_bo) {
		if (mdp4_kms->blank_cursor_iova && kms->vm)
			msm_gem_unpin_iova(mdp4_kms->blank_cursor_bo, kms->vm);
		drm_gem_object_put(mdp4_kms->blank_cursor_bo);
	}

	if (kms->vm) {
		struct msm_mmu *mmu = to_msm_vm(kms->vm)->mmu;

		mmu->funcs->detach(mmu);
		drm_gpuvm_put(kms->vm);
	}

	if (mdp4_kms->rpm_enabled)
		pm_runtime_disable(dev);

	mdp_kms_destroy(&mdp4_kms->base);
}

static const struct mdp_kms_funcs kms_funcs = {
	.base = {
		.hw_init         = mdp4_hw_init,
		.irq_preinstall  = mdp4_irq_preinstall,
		.irq_postinstall = mdp4_irq_postinstall,
		.irq_uninstall   = mdp4_irq_uninstall,
		.irq             = mdp4_irq,
		.enable_vblank   = mdp4_enable_vblank,
		.disable_vblank  = mdp4_disable_vblank,
		.enable_commit   = mdp4_enable_commit,
		.disable_commit  = mdp4_disable_commit,
		.flush_commit    = mdp4_flush_commit,
		.wait_flush      = mdp4_wait_flush,
		.complete_commit = mdp4_complete_commit,
		.round_pixclk    = mdp4_round_pixclk,
		.destroy         = mdp4_destroy,
	},
	.set_irqmask         = mdp4_set_irqmask,
};

int mdp4_disable(struct mdp4_kms *mdp4_kms)
{
	DBG("");

	clk_disable_unprepare(mdp4_kms->clk);
	clk_disable_unprepare(mdp4_kms->pclk);
	clk_disable_unprepare(mdp4_kms->lut_clk);
	clk_disable_unprepare(mdp4_kms->axi_clk);

	return 0;
}

int mdp4_enable(struct mdp4_kms *mdp4_kms)
{
	DBG("");

	clk_prepare_enable(mdp4_kms->clk);
	clk_prepare_enable(mdp4_kms->pclk);
	clk_prepare_enable(mdp4_kms->lut_clk);
	clk_prepare_enable(mdp4_kms->axi_clk);

	return 0;
}


static int mdp4_modeset_init_intf(struct mdp4_kms *mdp4_kms,
				  int intf_type)
{
	struct drm_device *dev = mdp4_kms->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_bridge *next_bridge;
	int dsi_id;
	int ret;

	switch (intf_type) {
	case DRM_MODE_ENCODER_LVDS:
		/*
		 * bail out early if there is no panel node (no need to
		 * initialize LCDC encoder and LVDS connector)
		 */
		next_bridge = devm_drm_of_get_bridge(dev->dev, dev->dev->of_node, 0, 0);
		if (IS_ERR(next_bridge)) {
			ret = PTR_ERR(next_bridge);
			if (ret == -ENODEV)
				return 0;
			return ret;
		}

		encoder = mdp4_lcdc_encoder_init(dev);
		if (IS_ERR(encoder)) {
			DRM_DEV_ERROR(dev->dev, "failed to construct LCDC encoder\n");
			return PTR_ERR(encoder);
		}

		/* LCDC can be hooked to DMA_P (TODO: Add DMA_S later?) */
		encoder->possible_crtcs = 1 << DMA_P;

		ret = drm_bridge_attach(encoder, next_bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to attach LVDS panel/bridge: %d\n", ret);

			return ret;
		}

		connector = drm_bridge_connector_init(dev, encoder);
		if (IS_ERR(connector)) {
			DRM_DEV_ERROR(dev->dev, "failed to initialize LVDS connector\n");
			return PTR_ERR(connector);
		}

		ret = drm_connector_attach_encoder(connector, encoder);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to attach LVDS connector: %d\n", ret);

			return ret;
		}

		break;
	case DRM_MODE_ENCODER_TMDS:
		encoder = mdp4_dtv_encoder_init(dev);
		if (IS_ERR(encoder)) {
			DRM_DEV_ERROR(dev->dev, "failed to construct DTV encoder\n");
			return PTR_ERR(encoder);
		}

		/* DTV can be hooked to DMA_E: */
		encoder->possible_crtcs = 1 << 1;

		if (priv->kms->hdmi) {
			/* Construct bridge/connector for HDMI: */
			ret = msm_hdmi_modeset_init(priv->kms->hdmi, dev, encoder);
			if (ret) {
				DRM_DEV_ERROR(dev->dev, "failed to initialize HDMI: %d\n", ret);
				return ret;
			}
		}

		break;
	case DRM_MODE_ENCODER_DSI:
		/* only DSI1 supported for now */
		dsi_id = 0;

		if (!priv->kms->dsi[dsi_id])
			break;

		encoder = mdp4_dsi_encoder_init(dev);
		if (IS_ERR(encoder)) {
			ret = PTR_ERR(encoder);
			DRM_DEV_ERROR(dev->dev,
				"failed to construct DSI encoder: %d\n", ret);
			return ret;
		}

		/* TODO: Add DMA_S later? */
		encoder->possible_crtcs = 1 << DMA_P;

		ret = msm_dsi_modeset_init(priv->kms->dsi[dsi_id], dev, encoder);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to initialize DSI: %d\n",
				ret);
			return ret;
		}

		break;
	default:
		DRM_DEV_ERROR(dev->dev, "Invalid or unsupported interface\n");
		return -EINVAL;
	}

	return 0;
}

static int modeset_init(struct mdp4_kms *mdp4_kms)
{
	struct drm_device *dev = mdp4_kms->dev;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	struct drm_bridge *lvds_bridge;
	int i, ret;
	static const enum mdp4_pipe rgb_planes[] = {
		RGB1, RGB2,
	};
	static const enum mdp4_pipe vg_planes[] = {
		VG1, VG2,
	};
	static const enum mdp4_dma mdp4_crtcs[] = {
		DMA_P, DMA_E,
	};
	static const char * const mdp4_crtc_names[] = {
		"DMA_P", "DMA_E",
	};
	static const int mdp4_intfs[] = {
		DRM_MODE_ENCODER_LVDS,
		DRM_MODE_ENCODER_DSI,
		DRM_MODE_ENCODER_TMDS,
	};

	/*
	 * Check if LVDS bridge/panel is available BEFORE creating any DRM
	 * objects. This avoids creating planes/CRTCs that we can't clean up
	 * properly if we need to defer probe waiting for the panel driver.
	 */
	lvds_bridge = devm_drm_of_get_bridge(dev->dev, dev->dev->of_node, 0, 0);
	if (IS_ERR(lvds_bridge)) {
		ret = PTR_ERR(lvds_bridge);
		if (ret == -EPROBE_DEFER)
			return ret;  /* Defer before creating any objects */
		/* -ENODEV means no panel configured - that's OK, continue */
	}

	/* construct non-private planes: */
	for (i = 0; i < ARRAY_SIZE(vg_planes); i++) {
		plane = mdp4_plane_init(dev, vg_planes[i], false);
		if (IS_ERR(plane)) {
			DRM_DEV_ERROR(dev->dev,
				"failed to construct plane for VG%d\n", i + 1);
			ret = PTR_ERR(plane);
			goto fail;
		}
	}

	for (i = 0; i < ARRAY_SIZE(mdp4_crtcs); i++) {
		plane = mdp4_plane_init(dev, rgb_planes[i], true);
		if (IS_ERR(plane)) {
			DRM_DEV_ERROR(dev->dev,
				"failed to construct plane for RGB%d\n", i + 1);
			ret = PTR_ERR(plane);
			goto fail;
		}

		crtc  = mdp4_crtc_init(dev, plane, i,
				mdp4_crtcs[i]);
		if (IS_ERR(crtc)) {
			DRM_DEV_ERROR(dev->dev, "failed to construct crtc for %s\n",
				mdp4_crtc_names[i]);
			ret = PTR_ERR(crtc);
			goto fail;
		}
	}

	/*
	 * we currently set up two relatively fixed paths:
	 *
	 * LCDC/LVDS path: RGB1 -> DMA_P -> LCDC -> LVDS
	 *			or
	 * DSI path: RGB1 -> DMA_P -> DSI1 -> DSI Panel
	 *
	 * DTV/HDMI path: RGB2 -> DMA_E -> DTV -> HDMI
	 */

	for (i = 0; i < ARRAY_SIZE(mdp4_intfs); i++) {
		ret = mdp4_modeset_init_intf(mdp4_kms, mdp4_intfs[i]);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to initialize intf: %d, %d\n",
				i, ret);
			goto fail;
		}
	}

	return 0;

fail:
	return ret;
}

static void read_mdp_hw_revision(struct mdp4_kms *mdp4_kms,
				 u32 *major, u32 *minor)
{
	struct drm_device *dev = mdp4_kms->dev;
	u32 version;

	mdp4_enable(mdp4_kms);
	version = mdp4_read(mdp4_kms, REG_MDP4_VERSION);
	mdp4_disable(mdp4_kms);

	*major = FIELD(version, MDP4_VERSION_MAJOR);
	*minor = FIELD(version, MDP4_VERSION_MINOR);

	DRM_DEV_INFO(dev->dev, "MDP4 version v%d.%d", *major, *minor);
}

static int mdp4_kms_init(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(to_mdp_kms(priv->kms));
	struct msm_kms *kms = NULL;
	struct drm_gpuvm *vm;
	int ret;
	u32 major, minor;
	unsigned long max_clk;

	/* APQ8060/MSM8660 - use 200MHz (matching webOS kernel) */
	max_clk = 200000000;

	ret = mdp_kms_init(&mdp4_kms->base, &kms_funcs);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "failed to init kms\n");
		goto fail;
	}

	kms = priv->kms;

	mdp4_kms->dev = dev;

	if (mdp4_kms->vdd) {
		ret = regulator_enable(mdp4_kms->vdd);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to enable regulator vdd: %d\n", ret);
			goto fail;
		}
	}

	clk_set_rate(mdp4_kms->clk, max_clk);

	read_mdp_hw_revision(mdp4_kms, &major, &minor);

	if (major != 4) {
		DRM_DEV_ERROR(dev->dev, "unexpected MDP version: v%d.%d\n",
			      major, minor);
		ret = -ENXIO;
		goto fail;
	}

	mdp4_kms->rev = minor;

	if (mdp4_kms->rev >= 2) {
		if (!mdp4_kms->lut_clk) {
			DRM_DEV_ERROR(dev->dev, "failed to get lut_clk\n");
			ret = -ENODEV;
			goto fail;
		}
		clk_set_rate(mdp4_kms->lut_clk, max_clk);
	}

	pm_runtime_enable(dev->dev);
	mdp4_kms->rpm_enabled = true;

	/*
	 * Call modeset_init() before disabling display outputs. modeset_init()
	 * checks panel/bridge availability first (returning -EPROBE_DEFER if
	 * the panel driver hasn't probed yet) before creating any DRM objects.
	 *
	 * By doing this check before the LCDC disable below, we preserve the
	 * bootloader's display state when waiting for deferred probe. Without
	 * this ordering, the LCDC disable would kill pixel data to the panel
	 * while the LVDS receiver is still active, causing blue vertical lines
	 * that persist until the deferred probe retry eventually succeeds.
	 */
	ret = modeset_init(mdp4_kms);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "modeset_init failed: %d\n", ret);
		goto fail;
	}

	/* make sure things are off before attaching iommu (bootloader could
	 * have left things on, in which case we'll start getting faults if
	 * we don't disable):
	 */
	mdp4_enable(mdp4_kms);
	mdp4_write(mdp4_kms, REG_MDP4_DTV_ENABLE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_LCDC_ENABLE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_DSI_ENABLE, 0);
	mdp4_disable(mdp4_kms);
	mdelay(16);

	/*
	 * Initialize VM after modeset_init() succeeds. This avoids creating
	 * a VM that needs cleanup if modeset_init() returns -EPROBE_DEFER
	 * (waiting for panel/bridge driver).
	 */
	vm = msm_kms_init_vm(mdp4_kms->dev, NULL);
	if (IS_ERR(vm)) {
		ret = PTR_ERR(vm);
		goto fail;
	}

	kms->vm = vm;

	/*
	 * Blank cursor requires IOMMU for IOVA mapping.
	 * Skip cursor support when running without IOMMU.
	 */
	if (kms->vm) {
		mdp4_kms->blank_cursor_bo = msm_gem_new(dev, SZ_16K, MSM_BO_WC | MSM_BO_SCANOUT);
		if (IS_ERR(mdp4_kms->blank_cursor_bo)) {
			ret = PTR_ERR(mdp4_kms->blank_cursor_bo);
			DRM_DEV_ERROR(dev->dev, "could not allocate blank-cursor bo: %d\n", ret);
			mdp4_kms->blank_cursor_bo = NULL;
			goto fail;
		}

		ret = msm_gem_get_and_pin_iova(mdp4_kms->blank_cursor_bo, kms->vm,
				&mdp4_kms->blank_cursor_iova);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "could not pin blank-cursor bo: %d\n", ret);
			goto fail;
		}
	} else {
		DRM_DEV_INFO(dev->dev, "no IOMMU, cursor support disabled\n");
		mdp4_kms->blank_cursor_bo = NULL;
		mdp4_kms->blank_cursor_iova = 0;
	}

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	return 0;

fail:
	if (kms)
		mdp4_destroy(kms);

	return ret;
}

static const struct dev_pm_ops mdp4_pm_ops = {
	.prepare = msm_kms_pm_prepare,
	.complete = msm_kms_pm_complete,
};

/* Default bandwidth values (in kBps) - used if not specified in DT */
#define MDP4_DEFAULT_BW_AVG_KBPS	(500 * 1024)	/* 500 MB/s */
#define MDP4_DEFAULT_BW_PEAK_KBPS	(700 * 1024)	/* 700 MB/s */

/*
 * Set up interconnect paths for MDP to memory bandwidth.
 * This coordinates with the bus fabric to prevent USB RNDIS failures
 * when MDP is accessing memory on APQ8060/MSM8660.
 *
 * On APQ8060/MSM8660, MDP has both SMI (dedicated graphics memory) and
 * EBI (system memory) paths. The webOS kernel voted on BOTH path types
 * simultaneously to ensure proper bus fabric clock scaling. We replicate
 * that behavior here by voting on all available paths.
 *
 * Bandwidth can be configured via device tree properties:
 *   qcom,icc-bw-avg-kbps  - average bandwidth in kBps
 *   qcom,icc-bw-peak-kbps - peak bandwidth in kBps
 */
static int mdp4_setup_interconnect(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct icc_path *path0_smi, *path1_smi;
	struct icc_path *path0_ebi, *path1_ebi;
	u32 avg_bw = MDP4_DEFAULT_BW_AVG_KBPS;
	u32 peak_bw = MDP4_DEFAULT_BW_PEAK_KBPS;

	/* Get SMI paths (primary for dedicated graphics memory) */
	path0_smi = msm_icc_get(&pdev->dev, "mdp0-smi");
	path1_smi = msm_icc_get(&pdev->dev, "mdp1-smi");

	/* Get EBI paths (for proper fabric clock scaling) */
	path0_ebi = msm_icc_get(&pdev->dev, "mdp0-ebi");
	path1_ebi = msm_icc_get(&pdev->dev, "mdp1-ebi");

	/*
	 * Fall back to generic memory paths if SMI not available.
	 * This supports platforms without dedicated graphics memory.
	 */
	if (IS_ERR_OR_NULL(path0_smi))
		path0_smi = msm_icc_get(&pdev->dev, "mdp0-mem");
	if (IS_ERR_OR_NULL(path1_smi))
		path1_smi = msm_icc_get(&pdev->dev, "mdp1-mem");

	if (IS_ERR(path0_smi))
		return PTR_ERR(path0_smi);

	if (!path0_smi) {
		/*
		 * No interconnect support is not fatal - the platform may
		 * not have an interconnect driver yet. But warn about it
		 * as it may cause USB issues on APQ8060/MSM8660.
		 */
		dev_warn(&pdev->dev, "No interconnect support - may cause USB/display conflicts!\n");
		return 0;
	}

	/* Read bandwidth from device tree if specified */
	of_property_read_u32(np, "qcom,icc-bw-avg-kbps", &avg_bw);
	of_property_read_u32(np, "qcom,icc-bw-peak-kbps", &peak_bw);

	dev_dbg(&pdev->dev, "MDP interconnect bandwidth: avg=%u kBps, peak=%u kBps\n",
		avg_bw, peak_bw);

	/*
	 * Vote on all available paths. On APQ8060/MSM8660, this includes
	 * both SMI and EBI paths. The interconnect framework aggregates
	 * these votes using max(sum_bw, max_peak_bw) to determine the
	 * required bus clock rate.
	 */
	icc_set_bw(path0_smi, avg_bw, peak_bw);

	if (!IS_ERR_OR_NULL(path1_smi))
		icc_set_bw(path1_smi, avg_bw, peak_bw);

	if (!IS_ERR_OR_NULL(path0_ebi))
		icc_set_bw(path0_ebi, avg_bw, peak_bw);

	if (!IS_ERR_OR_NULL(path1_ebi))
		icc_set_bw(path1_ebi, avg_bw, peak_bw);

	return 0;
}

static int mdp4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mdp4_kms *mdp4_kms;
	int irq, ret;

	mdp4_kms = devm_kzalloc(dev, sizeof(*mdp4_kms), GFP_KERNEL);
	if (!mdp4_kms)
		return -ENOMEM;

	/*
	 * Initialize reserved memory region (SMI) if specified in device tree.
	 * This allows DRM to allocate framebuffers from the dedicated SMI
	 * memory region instead of system RAM.
	 */
	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		dev_warn(dev, "Could not get reserved memory: %d\n", ret);

	mdp4_kms->mmio = msm_ioremap(pdev, NULL);
	if (IS_ERR(mdp4_kms->mmio))
		return PTR_ERR(mdp4_kms->mmio);

	/*
	 * Disable display outputs immediately after mapping registers.
	 * The bootloader may have left LCDC/DTV running, which causes
	 * IOMMU page faults when the MDP tries to access the bootloader's
	 * framebuffer through the now-active IOMMU without proper mappings.
	 * Writing 0 to the enable registers stops the MDP from fetching
	 * pixel data, preventing the page faults.
	 */
	writel(0, mdp4_kms->mmio + REG_MDP4_LCDC_ENABLE);
	writel(0, mdp4_kms->mmio + REG_MDP4_DTV_ENABLE);
	writel(0, mdp4_kms->mmio + REG_MDP4_DSI_ENABLE);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get irq\n");

	mdp4_kms->base.base.irq = irq;

	/*
	 * VDD regulator is optional - on some boards the bootloader
	 * may leave it on.
	 */
	mdp4_kms->vdd = devm_regulator_get_optional(&pdev->dev, "vdd");
	if (IS_ERR(mdp4_kms->vdd))
		mdp4_kms->vdd = NULL;

	mdp4_kms->clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(mdp4_kms->clk))
		return dev_err_probe(dev, PTR_ERR(mdp4_kms->clk), "failed to get core_clk\n");

	mdp4_kms->pclk = devm_clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(mdp4_kms->pclk))
		mdp4_kms->pclk = NULL;

	mdp4_kms->axi_clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(mdp4_kms->axi_clk))
		return dev_err_probe(dev, PTR_ERR(mdp4_kms->axi_clk), "failed to get axi_clk\n");

	mdp4_kms->lut_clk = devm_clk_get_optional(&pdev->dev, "lut_clk");
	if (IS_ERR(mdp4_kms->lut_clk))
		return dev_err_probe(dev, PTR_ERR(mdp4_kms->lut_clk), "failed to get lut_clk\n");

	/* Set up interconnect bandwidth to prevent display underrun */
	ret = mdp4_setup_interconnect(pdev);
	if (ret)
		return ret;

	return msm_drv_probe(&pdev->dev, mdp4_kms_init, &mdp4_kms->base.base);
}

static void mdp4_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &msm_drm_ops);
}

static const struct of_device_id mdp4_dt_match[] = {
	{ .compatible = "qcom,mdp4" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdp4_dt_match);

static struct platform_driver mdp4_platform_driver = {
	.probe      = mdp4_probe,
	.remove     = mdp4_remove,
	.shutdown   = msm_kms_shutdown,
	.driver     = {
		.name   = "mdp4",
		.of_match_table = mdp4_dt_match,
		.pm     = &mdp4_pm_ops,
	},
};

void __init msm_mdp4_register(void)
{
	platform_driver_register(&mdp4_platform_driver);
}

void __exit msm_mdp4_unregister(void)
{
	platform_driver_unregister(&mdp4_platform_driver);
}
