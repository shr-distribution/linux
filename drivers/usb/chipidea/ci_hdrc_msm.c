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
#include <linux/delay.h>
#include <linux/ulpi/driver.h>
#include <linux/ulpi/regs.h>

#include "ci.h"
#include "bits.h"

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
 * USB HS bandwidth values for memory path.
 *
 * The webOS kernel did NOT use explicit bandwidth voting for USB->EBI memory
 * path. It only used dfab_usb_hs_clk as a clock voter to keep DFAB running.
 * Excessive bandwidth voting (e.g. 300/600 MB/s) competes with MDP for fabric
 * priority and can cause display underflows during USB activity.
 *
 * Use minimal values here - the actual USB 2.0 high-speed maximum is 480 Mbps
 * = 60 MB/s. Most RNDIS/CDC traffic is much lower. The DFAB voter is the
 * key mechanism for USB stability, not memory path bandwidth.
 */
#define USB_HS_DEFAULT_BW_AVG_KBPS	(60 * 1024)	/* 60 MB/s avg (USB 2.0 max) */
#define USB_HS_DEFAULT_BW_PEAK_KBPS	(60 * 1024)	/* 60 MB/s peak */

/*
 * DFAB bandwidth for USB HS clock voter.
 * The webOS kernel used dfab_usb_hs_clk to vote on DFAB, keeping it stable
 * during USB activity. This prevents USB crashes during concurrent display
 * (MDP) operations. Higher values ensure stronger fabric priority for USB.
 * Initial testing at 64 MB/s showed USB crashes, increased to 128 MB/s.
 */
#define USB_HS_DFAB_BW_AVG_KBPS		(128 * 1024)	/* 128 MB/s */
#define USB_HS_DFAB_BW_PEAK_KBPS	(128 * 1024)	/* 128 MB/s */

/*
 * ULPI address/value pair used in the PHY settle sequence.
 * Terminated by an entry with addr == 0.
 */
struct ci_msm_ulpi_seq {
	u8 addr;
	u8 val;
};

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
	/*
	 * Optional PHY settle sequence for MSM8660-class hardware.
	 * When present, CI_HDRC_CONTROLLER_RESET_EVENT runs the full
	 * legacy otg_reset() sequence: POR -> INT clears -> 100ms settle
	 * -> link reset -> PORTSC re-select -> ULPI vendor writes.
	 * Populated from "qcom,phy-settle-seq" DT property on the
	 * USB controller node (u8 addr/val pairs, raw ULPI addresses).
	 */
	struct ci_msm_ulpi_seq *settle_seq;
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

/*
 * ci_hdrc_msm_phy_settle - full MSM8660 ULPI PHY settle sequence
 *
 * Replicates the legacy msm72k_otg.c otg_reset() sequence required
 * for APQ8060/MSM8660-class hardware after every connect event.
 *
 * On-device trace (webOS 2.6.35-palm, 2026-05-20, 9 plug events):
 *   - every connect uses phy_reset=1 (full sequence, no exceptions)
 *   - ULPI regs 0x32/0x36 are always 0x00 before vendor writes
 *   - reg0x32=0x35, reg0x36=0x06 confirmed written correctly each time
 *
 * Precondition: hw_controller_reset() (USBCMD_RST) has already run —
 * it fires before CI_HDRC_CONTROLLER_RESET_EVENT in hw_device_reset().
 * hw_phymode_configure() has also already selected the ULPI transceiver.
 *
 * Sequence:
 *   1. PHY POR (HS_PHY_POR_ASSERT, 12 µs pulse)
 *   2. Disable all PHY rise/fall interrupt enables (INT_RISE_CLR /
 *      INT_FALL_CLR) — prevents spurious events during ULPI re-sync
 *   3. 100 ms ULPI settle — PHY hardware requirement after POR
 *   4. Link reset (USBCMD_RST) + PORTSC ULPI re-select — re-syncs
 *      the controller's ULPI engine; ULPI vendor regs are zeroed here
 *   5. Apply settle_seq ULPI writes (e.g. pre-emphasis, HS slope) —
 *      must be last, after the link reset that zeros vendor regs
 */
static void ci_hdrc_msm_phy_settle(struct ci_hdrc *ci,
				   struct ci_hdrc_msm *msm_ci)
{
	const struct ci_msm_ulpi_seq *seq;
	unsigned long timeout;

	/* 1. PHY POR */
	ci_hdrc_msm_por_reset(&msm_ci->rcdev, msm_ci->secondary_phy ? 1 : 0);

	/*
	 * 2. Clear all PHY interrupt enables so no spurious events fire
	 *    while the ULPI link re-syncs.  Legacy wrote 0xFF to the
	 *    SET-to-CLEAR registers (0x0F/0x12); using the direct base
	 *    registers and writing 0 achieves the same final state.
	 *    These writes may silently fail if the ULPI link is not yet
	 *    ready — that is acceptable, step 4 re-syncs the link.
	 */
	ulpi_write(ci->ulpi, ULPI_USB_INT_EN_RISE, 0);
	ulpi_write(ci->ulpi, ULPI_USB_INT_EN_FALL, 0);

	/* 3. 100 ms settle — hardware characteristic of this PHY family */
	msleep(100);

	/*
	 * 4. Second link reset + ULPI transceiver re-select.
	 *    This is what zeroes ULPI vendor regs 0x32/0x36 (confirmed
	 *    on-device); vendor writes below must come after this step.
	 */
	hw_write(ci, OP_USBCMD, USBCMD_RST, USBCMD_RST);
	timeout = jiffies + msecs_to_jiffies(100);
	while (hw_read(ci, OP_USBCMD, USBCMD_RST)) {
		if (time_after(jiffies, timeout)) {
			dev_err(ci->dev,
				"phy_settle: link reset timed out\n");
			break;
		}
		usleep_range(1000, 2000);
	}
	hw_phymode_configure(ci);	/* PORTSC = ULPI transceiver */

	/* 5. Apply vendor ULPI writes with register values now at 0x00 */
	for (seq = msm_ci->settle_seq; seq->addr; seq++)
		ulpi_write(ci->ulpi, seq->addr, seq->val);
}

static int ci_hdrc_msm_notify_event(struct ci_hdrc *ci, unsigned event)
{
	struct device *dev = ci->dev->parent;
	struct ci_hdrc_msm *msm_ci = dev_get_drvdata(dev);
	int ret;

	switch (event) {
	case CI_HDRC_CONTROLLER_RESET_EVENT:
		dev_dbg(dev, "CI_HDRC_CONTROLLER_RESET_EVENT received\n");

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

		/*
		 * MSM8660 PHY settle sequence: replaces simple POR + writes
		 * with the full legacy otg_reset() flow (POR -> INT clears ->
		 * 100ms -> link reset -> PORTSC -> vendor writes).  Must run
		 * after phy_power_on() so clocks and regulators are up.
		 */
		if (msm_ci->settle_seq)
			ci_hdrc_msm_phy_settle(ci, msm_ci);

		/* use AHB transactor, allow posted data writes */
		hw_write_id_reg(ci, HS_PHY_AHB_MODE, 0xffffffff, 0x8);

		/* workaround for rx buffer collision issue */
		hw_write_id_reg(ci, HS_PHY_GENCONFIG,
				HS_PHY_TXFIFO_IDLE_FORCE_DIS, 0);

		if (!msm_ci->hsic)
			hw_write_id_reg(ci, HS_PHY_GENCONFIG_2,
					HS_PHY_ULPI_TX_PKT_EN_CLR_FIX, 0);

		/*
		 * Enable PHY session-valid control unconditionally.  This wires
		 * the ULPI PHY's B-Session Valid signal into the controller so
		 * VBUS removal and reconnection are detected without needing an
		 * extcon device.  The webOS msm_otg driver set SESS_VLD_CTRL
		 * unconditionally; the extcon guard here is too conservative for
		 * MSM8660 hardware which always requires this path.
		 */
		hw_write_id_reg(ci, HS_PHY_GENCONFIG_2,
				HS_PHY_SESS_VLD_CTRL_EN,
				HS_PHY_SESS_VLD_CTRL_EN);
		hw_write(ci, OP_USBCMD, HSPHY_SESS_VLD_CTRL,
			 HSPHY_SESS_VLD_CTRL);
		break;
	case CI_HDRC_CONTROLLER_STOPPED_EVENT:
		dev_dbg(dev, "CI_HDRC_CONTROLLER_STOPPED_EVENT received\n");
		/*
		 * Do NOT power off the PHY on disconnect for MSM8660-class
		 * hardware that uses qcom,phy-settle-seq.  These boards lack
		 * an extcon-based VBUS source -- VBUS is sensed by the ULPI
		 * PHY itself (routed to OTGSC_BSV via SESS_VLD_CTRL) and the
		 * internal controller comparator also requires the PHY clocks
		 * and regulators to be alive.  Powering off the PHY here
		 * leaves OTGSC_BSV stuck at 0 forever -- the BSVIS interrupt
		 * never fires on the next cable plug-in and reconnect dies.
		 *
		 * Confirmed on HP TouchPad (APQ8060) 2026-05-21: with the
		 * phy_power_off() call active, gether_disconnect runs on
		 * cable pull, STOPPED_EVENT fires, then dmesg goes silent
		 * across 5 unplug/replug cycles -- BSVIS never re-fires.
		 *
		 * Mirrors legacy webOS msm72k_otg.c, which never disables the
		 * PHY clocks/regulators between connect events.  The trade-off
		 * is the ULPI ref/sleep clocks and v1p8/v3p3 regulators stay
		 * up while the cable is unplugged -- acceptable, since these
		 * are tiny and the platform suspends as a whole anyway.
		 */
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

	/*
	 * Optional PHY settle sequence for MSM8660-class hardware.
	 * Encodes ULPI (addr, val) pairs written as the final step of
	 * ci_hdrc_msm_phy_settle() on every connect event, after the
	 * POR + 100ms + link-reset + PORTSC sequence that zeros them.
	 * Absent on platforms that don't need the full settle flow.
	 */
	{
		int size = of_property_count_u8_elems(pdev->dev.of_node,
						      "qcom,phy-settle-seq");

		if (size > 0 && !(size & 1)) {
			ci->settle_seq = devm_kmalloc_array(&pdev->dev,
							    (size / 2) + 1,
							    sizeof(*ci->settle_seq),
							    GFP_KERNEL);
			if (!ci->settle_seq)
				return -ENOMEM;

			ret = of_property_read_u8_array(pdev->dev.of_node,
							"qcom,phy-settle-seq",
							(u8 *)ci->settle_seq,
							size);
			if (ret)
				return ret;

			ci->settle_seq[size / 2].addr = 0;
			ci->settle_seq[size / 2].val  = 0;
		} else if (size > 0) {
			dev_err(&pdev->dev,
				"qcom,phy-settle-seq must have an even number of bytes\n");
			return -EINVAL;
		}
	}

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
