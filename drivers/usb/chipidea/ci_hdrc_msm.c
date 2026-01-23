// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved. */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/usb/chipidea.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interconnect.h>
#include <linux/io.h>
#include <linux/reset-controller.h>
#include <linux/extcon.h>
#include <linux/of.h>

#include "ci.h"

#define HS_PHY_AHB_MODE			0x0098

#define HS_PHY_GENCONFIG		0x009c
#define HS_PHY_TXFIFO_IDLE_FORCE_DIS	BIT(4)

#define HS_PHY_GENCONFIG_2		0x00a0
#define HS_PHY_SESS_VLD_CTRL_EN		BIT(7)
#define HS_PHY_ULPI_TX_PKT_EN_CLR_FIX	BIT(19)

#define HSPHY_SESS_VLD_CTRL		BIT(25)

/* Vendor base starts at 0x200 beyond CI base */
#define HS_PHY_CTRL			0x0040
#define HS_PHY_SEC_CTRL			0x0078
#define HS_PHY_DIG_CLAMP_N		BIT(16)
#define HS_PHY_POR_ASSERT		BIT(0)

/*
 * USB HS bandwidth values for stability with display active.
 * The TouchPad experiences USB crashes when MDP display is enabled.
 * Requesting higher bandwidth ensures USB gets priority on the shared
 * APPSS fabric path to memory, preventing starvation during concurrent
 * display scanout operations.
 */
#define USB_HS_DEFAULT_BW_AVG_KBPS	(300 * 1024)	/* 300 MB/s avg */
#define USB_HS_DEFAULT_BW_PEAK_KBPS	(600 * 1024)	/* 600 MB/s peak */

/*
 * DFAB bandwidth for USB HS clock voter.
 * The webOS kernel used dfab_usb_hs_clk to vote on DFAB, keeping it stable
 * during USB activity. This prevents USB crashes during concurrent display
 * (MDP) operations. Higher values ensure stronger fabric priority for USB.
 * Initial testing at 64 MB/s showed USB crashes, increased to 128 MB/s.
 */
#define USB_HS_DFAB_BW_AVG_KBPS		(128 * 1024)	/* 128 MB/s */
#define USB_HS_DFAB_BW_PEAK_KBPS	(128 * 1024)	/* 128 MB/s */

struct ci_hdrc_msm {
	struct platform_device *ci;
	struct clk *core_clk;
	struct clk *iface_clk;
	struct clk *fs_clk;
	struct clk *dfab_clk;	/* DFAB clock voter (webOS dfab_usb_hs_clk) */
	struct icc_path *icc_path;
	struct icc_path *icc_path_dfab;	/* DFAB voter path */
	u32 icc_bw_avg;		/* average bandwidth in kBps */
	u32 icc_bw_peak;	/* peak bandwidth in kBps */
	struct ci_hdrc_platform_data pdata;
	struct reset_controller_dev rcdev;
	bool secondary_phy;
	bool hsic;
	void __iomem *base;
};

static int
ci_hdrc_msm_por_reset(struct reset_controller_dev *r, unsigned long id)
{
	struct ci_hdrc_msm *ci_msm = container_of(r, struct ci_hdrc_msm, rcdev);
	void __iomem *addr = ci_msm->base;
	u32 val;

	if (id)
		addr += HS_PHY_SEC_CTRL;
	else
		addr += HS_PHY_CTRL;

	val = readl_relaxed(addr);
	val |= HS_PHY_POR_ASSERT;
	writel(val, addr);
	/*
	 * wait for minimum 10 microseconds as suggested by manual.
	 * Use a slightly larger value since the exact value didn't
	 * work 100% of the time.
	 */
	udelay(12);
	val &= ~HS_PHY_POR_ASSERT;
	writel(val, addr);

	return 0;
}

static const struct reset_control_ops ci_hdrc_msm_reset_ops = {
	.reset = ci_hdrc_msm_por_reset,
};

static int ci_hdrc_msm_notify_event(struct ci_hdrc *ci, unsigned event)
{
	struct device *dev = ci->dev->parent;
	struct ci_hdrc_msm *msm_ci = dev_get_drvdata(dev);
	int ret;

	switch (event) {
	case CI_HDRC_CONTROLLER_RESET_EVENT:
		dev_info(dev, "CI_HDRC_CONTROLLER_RESET_EVENT received\n");

		hw_phymode_configure(ci);
		if (msm_ci->secondary_phy) {
			u32 val = readl_relaxed(msm_ci->base + HS_PHY_SEC_CTRL);
			val |= HS_PHY_DIG_CLAMP_N;
			writel_relaxed(val, msm_ci->base + HS_PHY_SEC_CTRL);
		}

		ret = phy_init(ci->phy);
		if (ret)
			return ret;

		ret = phy_power_on(ci->phy);
		if (ret) {
			phy_exit(ci->phy);
			return ret;
		}

		/* use AHB transactor, allow posted data writes */
		hw_write_id_reg(ci, HS_PHY_AHB_MODE, 0xffffffff, 0x8);

		/* workaround for rx buffer collision issue */
		hw_write_id_reg(ci, HS_PHY_GENCONFIG,
				HS_PHY_TXFIFO_IDLE_FORCE_DIS, 0);

		if (!msm_ci->hsic)
			hw_write_id_reg(ci, HS_PHY_GENCONFIG_2,
					HS_PHY_ULPI_TX_PKT_EN_CLR_FIX, 0);

		if (!IS_ERR(ci->platdata->vbus_extcon.edev) || ci->role_switch) {
			hw_write_id_reg(ci, HS_PHY_GENCONFIG_2,
					HS_PHY_SESS_VLD_CTRL_EN,
					HS_PHY_SESS_VLD_CTRL_EN);
			hw_write(ci, OP_USBCMD, HSPHY_SESS_VLD_CTRL,
				 HSPHY_SESS_VLD_CTRL);

		}
		break;
	case CI_HDRC_CONTROLLER_STOPPED_EVENT:
		dev_dbg(dev, "CI_HDRC_CONTROLLER_STOPPED_EVENT received\n");
		phy_power_off(ci->phy);
		phy_exit(ci->phy);
		break;
	default:
		dev_dbg(dev, "unknown ci_hdrc event\n");
		break;
	}

	return 0;
}

static int ci_hdrc_msm_mux_phy(struct ci_hdrc_msm *ci,
			       struct platform_device *pdev)
{
	struct regmap *regmap;
	struct device *dev = &pdev->dev;
	struct of_phandle_args args;
	u32 val;
	int ret;

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "phy-select", 2, 0,
					       &args);
	if (ret)
		return 0;

	regmap = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = regmap_write(regmap, args.args[0], args.args[1]);
	if (ret)
		return ret;

	ci->secondary_phy = !!args.args[1];
	if (ci->secondary_phy) {
		val = readl_relaxed(ci->base + HS_PHY_SEC_CTRL);
		val |= HS_PHY_DIG_CLAMP_N;
		writel_relaxed(val, ci->base + HS_PHY_SEC_CTRL);
	}

	return 0;
}

static int ci_hdrc_msm_probe(struct platform_device *pdev)
{
	struct ci_hdrc_msm *ci;
	struct platform_device *plat_ci;
	struct clk *clk;
	struct reset_control *reset;
	int ret;
	struct device_node *ulpi_node, *phy_node;

	dev_dbg(&pdev->dev, "ci_hdrc_msm_probe\n");

	ci = devm_kzalloc(&pdev->dev, sizeof(*ci), GFP_KERNEL);
	if (!ci)
		return -ENOMEM;
	platform_set_drvdata(pdev, ci);

	ci->pdata.name = "ci_hdrc_msm";
	ci->pdata.capoffset = DEF_CAPOFFSET;
	ci->pdata.flags	= CI_HDRC_REGS_SHARED | CI_HDRC_DISABLE_STREAMING |
			  CI_HDRC_OVERRIDE_AHB_BURST |
			  CI_HDRC_OVERRIDE_PHY_CONTROL;
	ci->pdata.notify_event = ci_hdrc_msm_notify_event;

	reset = devm_reset_control_get(&pdev->dev, "core");
	if (IS_ERR(reset))
		return PTR_ERR(reset);

	ci->core_clk = clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ci->iface_clk = clk = devm_clk_get(&pdev->dev, "iface");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ci->fs_clk = clk = devm_clk_get_optional(&pdev->dev, "fs");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	/*
	 * DFAB clock voter - webOS kernel used dfab_usb_hs_clk to keep DFAB
	 * running at a stable rate during USB activity. This prevents USB
	 * crashes when MDP display is active (both share memory fabric).
	 * Optional since not all platforms need this.
	 */
	ci->dfab_clk = devm_clk_get_optional(&pdev->dev, "dfab");
	if (IS_ERR(ci->dfab_clk))
		return PTR_ERR(ci->dfab_clk);

	ci->icc_path = devm_of_icc_get(&pdev->dev, "usb-mem");
	if (IS_ERR(ci->icc_path))
		return PTR_ERR(ci->icc_path);

	/*
	 * DFAB interconnect path for USB HS clock voter.
	 * The webOS kernel used dfab_usb_hs_clk to vote on DFAB, keeping it
	 * stable during USB activity. This prevents USB crashes during
	 * concurrent display (MDP) operations. Make it optional since not
	 * all platforms have this configured.
	 */
	ci->icc_path_dfab = devm_of_icc_get(&pdev->dev, "usb-dfab");
	if (IS_ERR(ci->icc_path_dfab)) {
		if (PTR_ERR(ci->icc_path_dfab) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		/* DFAB path is optional, continue without it */
		ci->icc_path_dfab = NULL;
	}

	/* Read bandwidth from device tree if specified, otherwise use defaults */
	ci->icc_bw_avg = USB_HS_DEFAULT_BW_AVG_KBPS;
	ci->icc_bw_peak = USB_HS_DEFAULT_BW_PEAK_KBPS;
	of_property_read_u32(pdev->dev.of_node, "qcom,icc-bw-avg-kbps",
			     &ci->icc_bw_avg);
	of_property_read_u32(pdev->dev.of_node, "qcom,icc-bw-peak-kbps",
			     &ci->icc_bw_peak);

	ci->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ci->base))
		return PTR_ERR(ci->base);

	ci->rcdev.owner = THIS_MODULE;
	ci->rcdev.ops = &ci_hdrc_msm_reset_ops;
	ci->rcdev.of_node = pdev->dev.of_node;
	ci->rcdev.nr_resets = 2;
	ret = devm_reset_controller_register(&pdev->dev, &ci->rcdev);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ci->fs_clk);
	if (ret)
		return ret;

	reset_control_assert(reset);
	usleep_range(10000, 12000);
	reset_control_deassert(reset);

	clk_disable_unprepare(ci->fs_clk);

	ret = clk_prepare_enable(ci->core_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ci->iface_clk);
	if (ret)
		goto err_iface;

	/*
	 * Enable DFAB clock voter at 64 MHz - matches webOS dfab_usb_hs_clk.
	 * This keeps Daytona Fabric clock stable during USB activity,
	 * preventing crashes when MDP display is scanning out memory.
	 */
	if (ci->dfab_clk) {
		ret = clk_set_rate(ci->dfab_clk, 64000000);
		if (ret)
			dev_warn(&pdev->dev, "DFAB clock rate set failed: %d\n", ret);
		ret = clk_prepare_enable(ci->dfab_clk);
		if (ret) {
			dev_err(&pdev->dev, "DFAB clock enable failed: %d\n", ret);
			goto err_dfab_clk;
		}
		dev_info(&pdev->dev, "USB HS: DFAB clock enabled at 64 MHz\n");
	}

	/*
	 * Set interconnect bandwidth for USB HS.
	 * Use sustained average bandwidth (not just peak) to prevent bursty
	 * traffic from conflicting with display scanout on the shared
	 * APPSS fabric memory path. Bandwidth values are read from device
	 * tree (qcom,icc-bw-avg-kbps and qcom,icc-bw-peak-kbps properties).
	 */
	dev_info(&pdev->dev, "USB HS: Setting interconnect bandwidth avg=%u peak=%u kBps\n",
		 ci->icc_bw_avg, ci->icc_bw_peak);
	ret = icc_set_bw(ci->icc_path, ci->icc_bw_avg, ci->icc_bw_peak);
	if (ret)
		goto err_icc;

	/*
	 * Set DFAB bandwidth to keep Daytona Fabric clock stable.
	 * This matches the webOS kernel's dfab_usb_hs_clk clock voter which
	 * kept DFAB running during USB activity, preventing USB crashes
	 * when display (MDP) is accessing memory concurrently.
	 */
	if (ci->icc_path_dfab) {
		ret = icc_set_bw(ci->icc_path_dfab,
				 USB_HS_DFAB_BW_AVG_KBPS,
				 USB_HS_DFAB_BW_PEAK_KBPS);
		if (ret)
			goto err_dfab;
	}

	ret = ci_hdrc_msm_mux_phy(ci, pdev);
	if (ret)
		goto err_mux;

	ulpi_node = of_get_child_by_name(pdev->dev.of_node, "ulpi");
	if (ulpi_node) {
		phy_node = of_get_next_available_child(ulpi_node, NULL);
		ci->hsic = of_device_is_compatible(phy_node, "qcom,usb-hsic-phy");
		of_node_put(phy_node);
	}
	of_node_put(ulpi_node);

	plat_ci = ci_hdrc_add_device(&pdev->dev, pdev->resource,
				     pdev->num_resources, &ci->pdata);
	if (IS_ERR(plat_ci)) {
		ret = PTR_ERR(plat_ci);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "ci_hdrc_add_device failed!\n");
		goto err_mux;
	}

	ci->ci = plat_ci;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_no_callbacks(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

err_mux:
	if (ci->icc_path_dfab)
		icc_set_bw(ci->icc_path_dfab, 0, 0);
err_dfab:
	icc_set_bw(ci->icc_path, 0, 0);
err_icc:
	if (ci->dfab_clk)
		clk_disable_unprepare(ci->dfab_clk);
err_dfab_clk:
	clk_disable_unprepare(ci->iface_clk);
err_iface:
	clk_disable_unprepare(ci->core_clk);
	return ret;
}

static void ci_hdrc_msm_remove(struct platform_device *pdev)
{
	struct ci_hdrc_msm *ci = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	ci_hdrc_remove_device(ci->ci);
	if (ci->icc_path_dfab)
		icc_set_bw(ci->icc_path_dfab, 0, 0);
	icc_set_bw(ci->icc_path, 0, 0);
	if (ci->dfab_clk)
		clk_disable_unprepare(ci->dfab_clk);
	clk_disable_unprepare(ci->iface_clk);
	clk_disable_unprepare(ci->core_clk);
}

static const struct of_device_id msm_ci_dt_match[] = {
	{ .compatible = "qcom,ci-hdrc", },
	{ }
};
MODULE_DEVICE_TABLE(of, msm_ci_dt_match);

static struct platform_driver ci_hdrc_msm_driver = {
	.probe = ci_hdrc_msm_probe,
	.remove = ci_hdrc_msm_remove,
	.driver = {
		.name = "msm_hsusb",
		.of_match_table = msm_ci_dt_match,
	},
};

static int __init ci_hdrc_msm_driver_init(void)
{
	return platform_driver_register(&ci_hdrc_msm_driver);
}
subsys_initcall_sync(ci_hdrc_msm_driver_init);

static void __exit ci_hdrc_msm_driver_exit(void)
{
	platform_driver_unregister(&ci_hdrc_msm_driver);
}
module_exit(ci_hdrc_msm_driver_exit);

MODULE_ALIAS("platform:msm_hsusb");
MODULE_ALIAS("platform:ci13xxx_msm");
MODULE_DESCRIPTION("ChipIdea Highspeed Dual Role Controller");
MODULE_LICENSE("GPL v2");
