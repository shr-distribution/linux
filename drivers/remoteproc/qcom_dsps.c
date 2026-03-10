// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660/APQ8060 DSPS (Dedicated Sensors Processor Subsystem)
 * Peripheral Image Loader
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Herrie <pingu@pingudev.org>
 *
 * Based on legacy msm_dsps.c and peripheral-reset.c from downstream MSM kernel.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interconnect.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/firmware/qcom/qcom_scm.h>

#include "remoteproc_internal.h"
#include "qcom_common.h"
#include "qcom_pil_info.h"

/* PPSS register offsets */
#define PPSS_PAUSE_REG		0x1804
#define PPSS_TIMER0_32KHZ_REG	0x1004
#define PPSS_TIMER0_20MHZ_REG	0x0804

/* PAS (Peripheral Authentication Service) ID for DSPS */
#define PAS_DSPS_ID		2

/* Interconnect bandwidth for DSPS (KB/s) - matches legacy dfab_dsps_clk @ 64 MHz */
#define DSPS_ICC_BW_KBPS	64000

struct qcom_dsps {
	struct device *dev;
	struct rproc *rproc;

	void __iomem *ppss_base;	/* PPSS register base */

	struct reset_control *reset;	/* PPSS reset control */

	struct clk *ppss_pclk;		/* PPSS peripheral clock */
	struct clk *pmem_clk;		/* Peripheral memory clock */

	struct regulator *vdd_l5;	/* 8058_l5 @ 2.85V */
	struct regulator *vdd_s3;	/* 8058_s3 @ 1.8V */

	struct icc_path *icc_path;	/* Interconnect path */

	phys_addr_t mem_phys;
	phys_addr_t mem_reloc;
	void *mem_region;
	size_t mem_size;

	bool use_pas;			/* Use PAS (trusted) boot */
	bool is_on;			/* Power state tracking */

	struct qcom_scm_pas_metadata pas_metadata;
};

static int qcom_dsps_power_on(struct qcom_dsps *dsps)
{
	int ret;

	if (dsps->is_on)
		return 0;

	/* Enable regulators */
	if (dsps->vdd_l5) {
		ret = regulator_set_voltage(dsps->vdd_l5, 2850000, 2850000);
		if (ret) {
			dev_err(dsps->dev, "Failed to set L5 voltage: %d\n", ret);
			return ret;
		}
		ret = regulator_enable(dsps->vdd_l5);
		if (ret) {
			dev_err(dsps->dev, "Failed to enable L5: %d\n", ret);
			return ret;
		}
	}

	if (dsps->vdd_s3) {
		ret = regulator_set_voltage(dsps->vdd_s3, 1800000, 1800000);
		if (ret) {
			dev_err(dsps->dev, "Failed to set S3 voltage: %d\n", ret);
			goto disable_l5;
		}
		ret = regulator_enable(dsps->vdd_s3);
		if (ret) {
			dev_err(dsps->dev, "Failed to enable S3: %d\n", ret);
			goto disable_l5;
		}
	}

	/* Enable clocks */
	ret = clk_prepare_enable(dsps->ppss_pclk);
	if (ret) {
		dev_err(dsps->dev, "Failed to enable ppss_pclk: %d\n", ret);
		goto disable_s3;
	}

	if (dsps->pmem_clk) {
		ret = clk_prepare_enable(dsps->pmem_clk);
		if (ret) {
			dev_err(dsps->dev, "Failed to enable pmem_clk: %d\n", ret);
			goto disable_ppss_pclk;
		}
	}

	/* Vote for interconnect bandwidth */
	if (dsps->icc_path) {
		ret = icc_set_bw(dsps->icc_path, 0, DSPS_ICC_BW_KBPS);
		if (ret) {
			dev_err(dsps->dev, "Failed to set ICC bandwidth: %d\n", ret);
			goto disable_pmem_clk;
		}
	}

	dsps->is_on = true;
	return 0;

disable_pmem_clk:
	if (dsps->pmem_clk)
		clk_disable_unprepare(dsps->pmem_clk);
disable_ppss_pclk:
	clk_disable_unprepare(dsps->ppss_pclk);
disable_s3:
	if (dsps->vdd_s3)
		regulator_disable(dsps->vdd_s3);
disable_l5:
	if (dsps->vdd_l5)
		regulator_disable(dsps->vdd_l5);

	return ret;
}

static void qcom_dsps_power_off(struct qcom_dsps *dsps)
{
	if (!dsps->is_on)
		return;

	/* Release interconnect bandwidth */
	if (dsps->icc_path)
		icc_set_bw(dsps->icc_path, 0, 0);

	/* Disable clocks */
	if (dsps->pmem_clk)
		clk_disable_unprepare(dsps->pmem_clk);
	clk_disable_unprepare(dsps->ppss_pclk);

	/* Disable regulators */
	if (dsps->vdd_s3)
		regulator_disable(dsps->vdd_s3);
	if (dsps->vdd_l5)
		regulator_disable(dsps->vdd_l5);

	dsps->is_on = false;
}

static void qcom_dsps_pause(struct qcom_dsps *dsps)
{
	writel(1, dsps->ppss_base + PPSS_PAUSE_REG);
}

static void qcom_dsps_resume(struct qcom_dsps *dsps)
{
	writel(0, dsps->ppss_base + PPSS_PAUSE_REG);
}

static int qcom_dsps_load(struct rproc *rproc, const struct firmware *fw)
{
	struct qcom_dsps *dsps = rproc->priv;
	int ret;

	if (dsps->use_pas) {
		ret = qcom_mdt_pas_init(dsps->dev, fw, rproc->firmware,
					PAS_DSPS_ID, dsps->mem_phys,
					&dsps->pas_metadata);
		if (ret)
			return ret;

		ret = qcom_mdt_load_no_init(dsps->dev, fw, rproc->firmware,
					    dsps->mem_region,
					    dsps->mem_phys, dsps->mem_size,
					    &dsps->mem_reloc);
	} else {
		ret = qcom_mdt_load_no_init(dsps->dev, fw, rproc->firmware,
					    dsps->mem_region,
					    dsps->mem_phys, dsps->mem_size,
					    &dsps->mem_reloc);
	}

	if (ret)
		return ret;

	qcom_pil_info_store("dsps", dsps->mem_phys, dsps->mem_size);

	return 0;
}

static int qcom_dsps_start(struct rproc *rproc)
{
	struct qcom_dsps *dsps = rproc->priv;
	int ret;

	/* Power on DSPS domain */
	ret = qcom_dsps_power_on(dsps);
	if (ret)
		return ret;

	if (dsps->use_pas) {
		ret = qcom_scm_pas_auth_and_reset(PAS_DSPS_ID);
		if (ret) {
			dev_err(dsps->dev, "PAS auth and reset failed: %d\n", ret);
			goto power_off;
		}
	} else {
		/* Take DSPS out of reset - write 0 to PPSS_RESET */
		ret = reset_control_deassert(dsps->reset);
		if (ret) {
			dev_err(dsps->dev, "Failed to deassert reset: %d\n", ret);
			goto power_off;
		}
	}

	/* Resume DSPS execution */
	qcom_dsps_resume(dsps);

	dev_info(dsps->dev, "DSPS started\n");
	return 0;

power_off:
	qcom_dsps_power_off(dsps);
	return ret;
}

static int qcom_dsps_stop(struct rproc *rproc)
{
	struct qcom_dsps *dsps = rproc->priv;
	int ret = 0;

	/* Pause DSPS execution */
	qcom_dsps_pause(dsps);

	if (dsps->use_pas) {
		ret = qcom_scm_pas_shutdown(PAS_DSPS_ID);
		if (ret)
			dev_err(dsps->dev, "PAS shutdown failed: %d\n", ret);

		qcom_scm_pas_metadata_release(&dsps->pas_metadata);
	} else {
		/* Put DSPS into reset - assert PPSS_RESET */
		ret = reset_control_assert(dsps->reset);
		if (ret)
			dev_err(dsps->dev, "Failed to assert reset: %d\n", ret);
	}

	qcom_dsps_power_off(dsps);

	dev_info(dsps->dev, "DSPS stopped\n");
	return ret;
}

static void *qcom_dsps_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct qcom_dsps *dsps = rproc->priv;
	int offset;

	offset = da - dsps->mem_reloc;
	if (offset < 0 || offset + len > dsps->mem_size)
		return NULL;

	return dsps->mem_region + offset;
}

static const struct rproc_ops qcom_dsps_ops = {
	.start = qcom_dsps_start,
	.stop = qcom_dsps_stop,
	.da_to_va = qcom_dsps_da_to_va,
	.load = qcom_dsps_load,
	.parse_fw = qcom_register_dump_segments,
};

static int qcom_dsps_alloc_memory_region(struct qcom_dsps *dsps)
{
	struct device_node *node;
	struct resource r;
	int ret;

	node = of_parse_phandle(dsps->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(dsps->dev, "No memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &r);
	of_node_put(node);
	if (ret)
		return ret;

	dsps->mem_phys = r.start;
	dsps->mem_size = resource_size(&r);
	dsps->mem_region = devm_ioremap_wc(dsps->dev, dsps->mem_phys, dsps->mem_size);
	if (!dsps->mem_region) {
		dev_err(dsps->dev, "Failed to map memory region\n");
		return -ENOMEM;
	}

	return 0;
}

static int qcom_dsps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_dsps *dsps;
	struct rproc *rproc;
	const char *firmware;
	int ret;

	ret = of_property_read_string(dev->of_node, "firmware-name", &firmware);
	if (ret) {
		dev_err(dev, "Missing firmware-name property\n");
		return ret;
	}

	rproc = devm_rproc_alloc(dev, pdev->name, &qcom_dsps_ops,
				 firmware, sizeof(*dsps));
	if (!rproc)
		return -ENOMEM;

	/* Don't auto-boot - DSPS should be started on demand */
	rproc->auto_boot = false;

	dsps = rproc->priv;
	dsps->dev = dev;
	dsps->rproc = rproc;

	/* Check if we should use PAS (trusted) boot */
	dsps->use_pas = qcom_scm_is_available() &&
			qcom_scm_pas_supported(PAS_DSPS_ID);

	dev_info(dev, "Using %s boot mode\n",
		 dsps->use_pas ? "PAS (trusted)" : "direct (untrusted)");

	/* Map PPSS registers */
	dsps->ppss_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsps->ppss_base))
		return PTR_ERR(dsps->ppss_base);

	/* Get reset control */
	dsps->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(dsps->reset))
		return dev_err_probe(dev, PTR_ERR(dsps->reset),
				     "Failed to get reset control\n");

	/* Get clocks */
	dsps->ppss_pclk = devm_clk_get(dev, "ppss_pclk");
	if (IS_ERR(dsps->ppss_pclk))
		return dev_err_probe(dev, PTR_ERR(dsps->ppss_pclk),
				     "Failed to get ppss_pclk\n");

	dsps->pmem_clk = devm_clk_get_optional(dev, "pmem");
	if (IS_ERR(dsps->pmem_clk))
		return PTR_ERR(dsps->pmem_clk);

	/* Get optional regulators */
	dsps->vdd_l5 = devm_regulator_get_optional(dev, "vdd-l5");
	if (IS_ERR(dsps->vdd_l5)) {
		if (PTR_ERR(dsps->vdd_l5) != -ENODEV)
			return PTR_ERR(dsps->vdd_l5);
		dsps->vdd_l5 = NULL;
	}

	dsps->vdd_s3 = devm_regulator_get_optional(dev, "vdd-s3");
	if (IS_ERR(dsps->vdd_s3)) {
		if (PTR_ERR(dsps->vdd_s3) != -ENODEV)
			return PTR_ERR(dsps->vdd_s3);
		dsps->vdd_s3 = NULL;
	}

	/* Get optional interconnect path */
	dsps->icc_path = devm_of_icc_get(dev, "memory");
	if (IS_ERR(dsps->icc_path)) {
		if (PTR_ERR(dsps->icc_path) != -ENODATA)
			return dev_err_probe(dev, PTR_ERR(dsps->icc_path),
					     "Failed to get interconnect\n");
		dsps->icc_path = NULL;
	}

	ret = qcom_dsps_alloc_memory_region(dsps);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dsps);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

	dev_info(dev, "DSPS remoteproc registered\n");
	return 0;
}

static const struct of_device_id qcom_dsps_of_match[] = {
	{ .compatible = "qcom,msm8660-dsps-pil" },
	{ .compatible = "qcom,apq8060-dsps-pil" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_dsps_of_match);

static struct platform_driver qcom_dsps_driver = {
	.probe = qcom_dsps_probe,
	.driver = {
		.name = "qcom_dsps",
		.of_match_table = qcom_dsps_of_match,
	},
};
module_platform_driver(qcom_dsps_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660/APQ8060 DSPS Peripheral Image Loader");
MODULE_LICENSE("GPL v2");
