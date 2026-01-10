// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8660/APQ8060 LPASS QDSP6v2 Peripheral Image Loader
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Christophe Chabanois <christophe.chabanois@gmail.com>
 * Copyright (c) 2024, Herrie <pingu@pingudev.org>
 *
 * Based on legacy peripheral-reset.c from downstream MSM kernel.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/smem.h>
#include <linux/firmware/qcom/qcom_scm.h>

#include "remoteproc_internal.h"
#include "qcom_common.h"
#include "qcom_pil_info.h"

/*
 * LCC_Q6_FUNC register bit definitions
 * This register controls the LPASS Q6 DSP power, clocks, and reset
 */
#define LV_EN			BIT(27)
#define STOP_CORE		BIT(26)
#define CLAMP_IO		BIT(25)
#define Q6SS_PRIV_ARES		BIT(24)
#define Q6SS_SS_ARES		BIT(23)
#define Q6SS_ISDB_ARES		BIT(22)
#define Q6SS_ETM_ARES		BIT(21)
#define CORE_ARES		BIT(10)
#define CORE_L1_MEM_CORE_EN	BIT(9)
#define CORE_TCM_MEM_CORE_EN	BIT(8)
#define CORE_TCM_MEM_PERPH_EN	BIT(7)
#define CORE_GFM4_CLK_EN	BIT(2)

/* QDSP6SS register offsets */
#define QDSP6SS_RST_EVB		0x0000
#define QDSP6SS_STRAP_TCM	0x001C
#define QDSP6SS_STRAP_AHB	0x0020

/* Q6 memory strap configuration values */
#define Q6_STRAP_AHB_UPPER	(0x290 << 12)
#define Q6_STRAP_AHB_LOWER	0x280
#define Q6_STRAP_TCM_BASE	(0x28C << 15)
#define Q6_STRAP_TCM_CONFIG	0x28B

/* LCC register offset for Q6_FUNC */
#define LCC_Q6_FUNC_OFFSET	0x001C

/* PAS (Peripheral Authentication Service) ID for Q6 */
#define PAS_Q6_ID		1

struct q6v2_lpass {
	struct device *dev;
	struct rproc *rproc;

	struct regmap *lcc_regmap;	/* LCC (LPASS Clock Controller) regmap */
	void __iomem *qdsp6ss_base;	/* QDSP6 Subsystem base */

	struct clk *pll;		/* PLL4 - LPASS PLL */

	phys_addr_t mem_phys;
	phys_addr_t mem_reloc;
	void *mem_region;
	size_t mem_size;

	bool use_pas;			/* Use PAS (trusted) boot */

	struct qcom_scm_pas_metadata pas_metadata;
	struct qcom_rproc_subdev smd_subdev;
};

static int q6v2_lpass_load(struct rproc *rproc, const struct firmware *fw)
{
	struct q6v2_lpass *q6v2 = rproc->priv;
	int ret;

	pr_emerg("Q6V2: ========== load() called ==========\n");
	pr_emerg("Q6V2: fw size=%zu\n", fw->size);
	pr_emerg("Q6V2: mem_phys=0x%pa, mem_size=0x%zx, mem_region=%px\n",
		 &q6v2->mem_phys, q6v2->mem_size, q6v2->mem_region);
	msleep(100);

	if (q6v2->use_pas) {
		pr_emerg("Q6V2: using PAS load\n");
		ret = qcom_mdt_pas_init(q6v2->dev, fw, rproc->firmware,
					PAS_Q6_ID, q6v2->mem_phys,
					&q6v2->pas_metadata);
		if (ret)
			return ret;

		ret = qcom_mdt_load_no_init(q6v2->dev, fw, rproc->firmware,
					    q6v2->mem_region,
					    q6v2->mem_phys, q6v2->mem_size,
					    &q6v2->mem_reloc);
	} else {
		pr_emerg("Q6V2: using direct load (untrusted)\n");
		pr_emerg("Q6V2: calling qcom_mdt_load_no_init (skip PAS/SCM)...\n");
		ret = qcom_mdt_load_no_init(q6v2->dev, fw, rproc->firmware,
					    q6v2->mem_region,
					    q6v2->mem_phys, q6v2->mem_size,
					    &q6v2->mem_reloc);
		pr_emerg("Q6V2: qcom_mdt_load_no_init returned %d\n", ret);
	}

	pr_emerg("Q6V2: mdt_load returned %d, mem_reloc=0x%pa\n",
		 ret, &q6v2->mem_reloc);

	if (ret)
		return ret;

	qcom_pil_info_store("lpass", q6v2->mem_phys, q6v2->mem_size);

	pr_emerg("Q6V2: load() complete\n");
	return 0;
}

static int q6v2_lpass_start_trusted(struct q6v2_lpass *q6v2)
{
	int ret;

	ret = qcom_scm_pas_auth_and_reset(PAS_Q6_ID);
	if (ret) {
		dev_err(q6v2->dev, "PAS auth and reset failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int q6v2_lpass_start_untrusted(struct q6v2_lpass *q6v2)
{
	u32 reg, test_val;
	int ret;

	pr_emerg("Q6V2: start_untrusted() begin\n");

	/* Read current Q6_FUNC register value */
	pr_emerg("Q6V2: reading LCC Q6_FUNC...\n");
	ret = regmap_read(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, &reg);
	if (ret) {
		pr_emerg("Q6V2: regmap_read failed: %d\n", ret);
		return ret;
	}
	pr_emerg("Q6V2: Q6_FUNC initial=0x%08x\n", reg);

	/*
	 * DEBUG: Try to read QDSP6SS registers before modifying Q6_FUNC.
	 * If QDSP6SS is not accessible (powered off), this will crash.
	 */
	pr_emerg("Q6V2: DEBUG - testing QDSP6SS read at %px...\n", q6v2->qdsp6ss_base);
	test_val = readl(q6v2->qdsp6ss_base + QDSP6SS_RST_EVB);
	pr_emerg("Q6V2: DEBUG - QDSP6SS_RST_EVB = 0x%08x\n", test_val);
	test_val = readl(q6v2->qdsp6ss_base + QDSP6SS_STRAP_TCM);
	pr_emerg("Q6V2: DEBUG - QDSP6SS_STRAP_TCM = 0x%08x\n", test_val);
	test_val = readl(q6v2->qdsp6ss_base + QDSP6SS_STRAP_AHB);
	pr_emerg("Q6V2: DEBUG - QDSP6SS_STRAP_AHB = 0x%08x\n", test_val);

	/* Put Q6 into reset */
	reg |= Q6SS_SS_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES | STOP_CORE | CORE_ARES;
	reg &= ~CORE_GFM4_CLK_EN;
	pr_emerg("Q6V2: writing Q6_FUNC=0x%08x (reset)\n", reg);
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	/* Verify the write worked */
	regmap_read(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, &test_val);
	pr_emerg("Q6V2: Q6_FUNC after reset write = 0x%08x\n", test_val);

	/* Wait 8 AHB cycles for Q6 to be fully reset (AHB = 1.5MHz) */
	usleep_range(20, 30);
	pr_emerg("Q6V2: reset wait done\n");

	/* Turn on Q6 memory */
	reg |= CORE_GFM4_CLK_EN | CORE_L1_MEM_CORE_EN | CORE_TCM_MEM_CORE_EN |
	       CORE_TCM_MEM_PERPH_EN;
	pr_emerg("Q6V2: writing Q6_FUNC=0x%08x (memory on)\n", reg);
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	/* Verify the write worked */
	regmap_read(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, &test_val);
	pr_emerg("Q6V2: Q6_FUNC after memory on = 0x%08x\n", test_val);

	/* Turn on Q6 core clocks and take core out of reset */
	reg &= ~(CLAMP_IO | Q6SS_SS_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES | CORE_ARES);
	pr_emerg("Q6V2: writing Q6_FUNC=0x%08x (clocks on, reset off)\n", reg);
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	/* Wait for clocks to be enabled */
	mb();

	/* Verify the write worked */
	regmap_read(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, &test_val);
	pr_emerg("Q6V2: Q6_FUNC after clocks on = 0x%08x\n", test_val);

	/* Program boot address */
	pr_emerg("Q6V2: programming boot addr=0x%08x at QDSP6SS+0x%x\n",
		 (u32)((q6v2->mem_reloc >> 12) & 0xFFFFF), QDSP6SS_RST_EVB);
	writel((q6v2->mem_reloc >> 12) & 0xFFFFF,
	       q6v2->qdsp6ss_base + QDSP6SS_RST_EVB);
	pr_emerg("Q6V2: RST_EVB written\n");

	/* Program TCM and AHB strap registers */
	pr_emerg("Q6V2: programming STRAP_TCM=0x%08x\n",
		 Q6_STRAP_TCM_CONFIG | Q6_STRAP_TCM_BASE);
	writel(Q6_STRAP_TCM_CONFIG | Q6_STRAP_TCM_BASE,
	       q6v2->qdsp6ss_base + QDSP6SS_STRAP_TCM);
	pr_emerg("Q6V2: STRAP_TCM written\n");

	pr_emerg("Q6V2: programming STRAP_AHB=0x%08x\n",
		 Q6_STRAP_AHB_UPPER | Q6_STRAP_AHB_LOWER);
	writel(Q6_STRAP_AHB_UPPER | Q6_STRAP_AHB_LOWER,
	       q6v2->qdsp6ss_base + QDSP6SS_STRAP_AHB);
	pr_emerg("Q6V2: STRAP_AHB written\n");

	/* Wait for addresses to be programmed before starting Q6 */
	mb();
	pr_emerg("Q6V2: straps programmed, about to start Q6 core\n");

	/* Start Q6 instruction execution */
	reg &= ~STOP_CORE;
	pr_emerg("Q6V2: writing Q6_FUNC=0x%08x (start Q6)\n", reg);
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	pr_emerg("Q6V2: start_untrusted() complete - Q6 should be running!\n");
	return 0;
}

static int q6v2_lpass_start(struct rproc *rproc)
{
	struct q6v2_lpass *q6v2 = rproc->priv;
	int ret;
	u32 reg;

	pr_emerg("Q6V2: ========== start() begin ==========\n");
	msleep(100); /* Force log flush */

	/*
	 * First enable PLL4 via RPM - this powers up the LPASS domain.
	 * We MUST do this BEFORE accessing any LCC registers.
	 */
	pr_emerg("Q6V2: Step 1 - enabling PLL4 via RPM...\n");
	msleep(100);
	ret = clk_prepare_enable(q6v2->pll);
	if (ret) {
		pr_emerg("Q6V2: Failed to enable PLL4: %d\n", ret);
		return ret;
	}
	pr_emerg("Q6V2: PLL4 enabled OK via RPM!\n");
	msleep(100);

	/* Give LPASS domain time to power up after RPM enables PLL4 */
	msleep(50);

	/*
	 * DEBUG: Now try to read LCC_Q6_FUNC - should work after PLL4 enable
	 */
	pr_emerg("Q6V2: Step 2 - testing LCC register read...\n");
	msleep(100);
	ret = regmap_read(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, &reg);
	if (ret) {
		pr_emerg("Q6V2: LCC read failed: %d\n", ret);
		clk_disable_unprepare(q6v2->pll);
		return ret;
	}
	pr_emerg("Q6V2: LCC_Q6_FUNC = 0x%08x - LCC accessible!\n", reg);
	msleep(100);

	pr_emerg("Q6V2: Step 3 - calling start routine...\n");
	msleep(100);

	if (q6v2->use_pas) {
		pr_emerg("Q6V2: using start_trusted()\n");
		ret = q6v2_lpass_start_trusted(q6v2);
	} else {
		pr_emerg("Q6V2: using start_untrusted()\n");
		ret = q6v2_lpass_start_untrusted(q6v2);
	}

	if (ret) {
		clk_disable_unprepare(q6v2->pll);
		return ret;
	}

	pr_emerg("Q6V2: start() complete\n");
	return 0;
}

static int q6v2_lpass_stop_trusted(struct q6v2_lpass *q6v2)
{
	int ret;

	ret = qcom_scm_pas_shutdown(PAS_Q6_ID);
	if (ret)
		dev_err(q6v2->dev, "PAS shutdown failed: %d\n", ret);

	qcom_scm_pas_metadata_release(&q6v2->pas_metadata);

	return ret;
}

static int q6v2_lpass_stop_untrusted(struct q6v2_lpass *q6v2)
{
	u32 reg;
	int ret;

	ret = regmap_read(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, &reg);
	if (ret)
		return ret;

	/* Stop Q6 execution */
	reg |= STOP_CORE;
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	/* Halt clocks and turn off memory */
	reg &= ~(CORE_L1_MEM_CORE_EN | CORE_TCM_MEM_CORE_EN | CORE_TCM_MEM_PERPH_EN);
	reg |= CLAMP_IO;
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	/* Put Q6 into reset */
	reg |= Q6SS_SS_ARES | Q6SS_ISDB_ARES | Q6SS_ETM_ARES | CORE_ARES;
	regmap_write(q6v2->lcc_regmap, LCC_Q6_FUNC_OFFSET, reg);

	return 0;
}

static int q6v2_lpass_stop(struct rproc *rproc)
{
	struct q6v2_lpass *q6v2 = rproc->priv;
	int ret;

	if (q6v2->use_pas)
		ret = q6v2_lpass_stop_trusted(q6v2);
	else
		ret = q6v2_lpass_stop_untrusted(q6v2);

	clk_disable_unprepare(q6v2->pll);

	return ret;
}

static void *q6v2_lpass_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct q6v2_lpass *q6v2 = rproc->priv;
	int offset;

	offset = da - q6v2->mem_reloc;
	if (offset < 0 || offset + len > q6v2->mem_size)
		return NULL;

	return q6v2->mem_region + offset;
}

static const struct rproc_ops q6v2_lpass_ops = {
	.start = q6v2_lpass_start,
	.stop = q6v2_lpass_stop,
	.da_to_va = q6v2_lpass_da_to_va,
	.load = q6v2_lpass_load,
	.parse_fw = qcom_register_dump_segments,
};

static int q6v2_lpass_alloc_memory_region(struct q6v2_lpass *q6v2)
{
	struct device_node *node;
	struct resource r;
	int ret;

	node = of_parse_phandle(q6v2->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(q6v2->dev, "No memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &r);
	of_node_put(node);
	if (ret)
		return ret;

	q6v2->mem_phys = r.start;
	q6v2->mem_size = resource_size(&r);
	q6v2->mem_region = devm_ioremap_wc(q6v2->dev, q6v2->mem_phys, q6v2->mem_size);
	if (!q6v2->mem_region) {
		dev_err(q6v2->dev, "Failed to map memory region\n");
		return -ENOMEM;
	}

	return 0;
}

static int q6v2_lpass_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct q6v2_lpass *q6v2;
	struct rproc *rproc;
	const char *firmware;
	int ret;

	/*
	 * Defer probe until SMEM is available. SMD subdevices need SMEM
	 * to register edges, and if SMEM isn't ready when auto_boot
	 * triggers, the SMD registration fails and remoteproc stays offline.
	 */
	if (!qcom_smem_is_available())
		return -EPROBE_DEFER;

	pr_emerg("Q6V2_LPASS: probe() ENTER - device %s\n", dev_name(dev));

	ret = of_property_read_string(dev->of_node, "firmware-name", &firmware);
	if (ret) {
		dev_err(dev, "Missing firmware-name property\n");
		return ret;
	}

	rproc = devm_rproc_alloc(dev, pdev->name, &q6v2_lpass_ops,
				 firmware, sizeof(*q6v2));
	if (!rproc)
		return -ENOMEM;

	rproc->auto_boot = true;

	q6v2 = rproc->priv;
	q6v2->dev = dev;
	q6v2->rproc = rproc;

	/* Check if we should use PAS (trusted) boot */
	q6v2->use_pas = qcom_scm_is_available() &&
			qcom_scm_pas_supported(PAS_Q6_ID);

	dev_info(dev, "Using %s boot mode\n",
		 q6v2->use_pas ? "PAS (trusted)" : "direct (untrusted)");

	/* Get LCC regmap via syscon - shared with LCC clock controller */
	q6v2->lcc_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "qcom,lcc");
	if (IS_ERR(q6v2->lcc_regmap))
		return dev_err_probe(dev, PTR_ERR(q6v2->lcc_regmap),
				     "Failed to get LCC syscon\n");

	/* Map QDSP6SS (QDSP6 SubSystem) */
	q6v2->qdsp6ss_base = devm_platform_ioremap_resource_byname(pdev, "qdsp6ss");
	if (IS_ERR(q6v2->qdsp6ss_base))
		return PTR_ERR(q6v2->qdsp6ss_base);

	/* Get PLL4 clock */
	q6v2->pll = devm_clk_get(dev, "pll");
	if (IS_ERR(q6v2->pll))
		return dev_err_probe(dev, PTR_ERR(q6v2->pll),
				     "Failed to get PLL clock\n");

	ret = q6v2_lpass_alloc_memory_region(q6v2);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, q6v2);

	qcom_add_smd_subdev(rproc, &q6v2->smd_subdev);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

	pr_emerg("Q6V2_LPASS: probe() SUCCESS - remoteproc registered\n");
	return 0;
}

static const struct of_device_id q6v2_lpass_of_match[] = {
	{ .compatible = "qcom,msm8660-lpass-pil" },
	{ .compatible = "qcom,apq8060-lpass-pil" },
	{ }
};
MODULE_DEVICE_TABLE(of, q6v2_lpass_of_match);

static struct platform_driver q6v2_lpass_driver = {
	.probe = q6v2_lpass_probe,
	.driver = {
		.name = "qcom_q6v2_lpass",
		.of_match_table = q6v2_lpass_of_match,
	},
};
module_platform_driver(q6v2_lpass_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660 LPASS QDSP6v2 Peripheral Image Loader");
MODULE_LICENSE("GPL v2");
