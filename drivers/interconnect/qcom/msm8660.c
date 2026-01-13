// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM8660/APQ8060 interconnect driver
 *
 * Copyright (c) 2026 LuneOS Project
 *
 * Based on msm8974.c by Brian Masney <masneyb@onstation.org>
 * and webOS kernel msm_bus_board_8660.c
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * MSM8660/APQ8060 has a fabric-based bus architecture:
 *
 *              +------------------+
 *              |   APPSS Fabric   |  (CPU, L2, Memory)
 *              +--------+---------+
 *                       |
 *         +-------------+-------------+
 *         |                           |
 *  +------+------+            +-------+-------+
 *  | MMSS Fabric |            | System Fabric |
 *  | (Display,   |            | (Peripherals, |
 *  |  Camera,    |            |  DMA, etc)    |
 *  |  Video)     |            +-------+-------+
 *  +-------------+                    |
 *                           +---------+---------+
 *                           |                   |
 *                    +------+------+     +------+------+
 *                    | System FPB  |     |  CPSS FPB   |
 *                    | (RPM, PMIC) |     | (GSBI, USB) |
 *                    +-------------+     +-------------+
 */

#include <dt-bindings/interconnect/qcom,msm8660.h>

#include <linux/args.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Internal node IDs - these map to the DT binding IDs plus fabric offset */
enum {
	/* APPSS Fabric nodes */
	MSM8660_AFAB_MAS_AMPSS_M0 = 1,
	MSM8660_AFAB_MAS_AMPSS_M1,
	MSM8660_AFAB_SLV_EBI_CH0,
	MSM8660_AFAB_SLV_AMPSS_L2,
	MSM8660_AFAB_TO_MMSS,
	MSM8660_AFAB_TO_SYSTEM,

	/* System Fabric nodes */
	MSM8660_SFAB_MAS_APPSS,
	MSM8660_SFAB_MAS_SPS,
	MSM8660_SFAB_MAS_ADM0_PORT0,
	MSM8660_SFAB_MAS_ADM0_PORT1,
	MSM8660_SFAB_MAS_ADM1_PORT0,
	MSM8660_SFAB_MAS_ADM1_PORT1,
	MSM8660_SFAB_MAS_LPASS_PROC,
	MSM8660_SFAB_MAS_MSS_PROCI,
	MSM8660_SFAB_MAS_MSS_PROCD,
	MSM8660_SFAB_MAS_MSS_MDM_PORT0,
	MSM8660_SFAB_MAS_LPASS,
	MSM8660_SFAB_MAS_MMSS_FPB,
	MSM8660_SFAB_MAS_ADM1_CI,
	MSM8660_SFAB_MAS_ADM0_CI,
	MSM8660_SFAB_MAS_MSS_MDM_PORT1,
	MSM8660_SFAB_MAS_USB_HS,
	MSM8660_SFAB_TO_APPSS,
	MSM8660_SFAB_TO_SYSTEM_FPB,
	MSM8660_SFAB_TO_CPSS_FPB,
	MSM8660_SFAB_SLV_SPS,
	MSM8660_SFAB_SLV_SYSTEM_IMEM,
	MSM8660_SFAB_SLV_AMPSS,
	MSM8660_SFAB_SLV_MSS,
	MSM8660_SFAB_SLV_LPASS,
	MSM8660_SFAB_SLV_MMSS_FPB,

	/* MMSS Fabric nodes */
	MSM8660_MMFAB_MAS_MDP_PORT0,
	MSM8660_MMFAB_MAS_MDP_PORT1,
	MSM8660_MMFAB_MAS_ADM1_PORT0,
	MSM8660_MMFAB_MAS_ROTATOR,
	MSM8660_MMFAB_MAS_GRAPHICS_3D,
	MSM8660_MMFAB_MAS_JPEG_DEC,
	MSM8660_MMFAB_MAS_GRAPHICS_2D_CORE0,
	MSM8660_MMFAB_MAS_VFE,
	MSM8660_MMFAB_MAS_VPE,
	MSM8660_MMFAB_MAS_JPEG_ENC,
	MSM8660_MMFAB_MAS_GRAPHICS_2D_CORE1,
	MSM8660_MMFAB_MAS_HD_CODEC_PORT0,
	MSM8660_MMFAB_MAS_HD_CODEC_PORT1,
	MSM8660_MMFAB_TO_APPSS,
	MSM8660_MMFAB_SLV_SMI,
	MSM8660_MMFAB_SLV_MM_IMEM,
};

#define to_msm8660_icc_provider(_provider) \
	container_of(_provider, struct msm8660_icc_provider, provider)

static const struct clk_bulk_data msm8660_afab_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
};

static const struct clk_bulk_data msm8660_sfab_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
};

static const struct clk_bulk_data msm8660_mmfab_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
};

/**
 * struct msm8660_icc_provider - MSM8660 specific interconnect provider
 * @provider: generic interconnect provider
 * @bus_clks: the clk_bulk_data table of bus clocks
 * @num_clks: the total number of clk_bulk_data entries
 */
struct msm8660_icc_provider {
	struct icc_provider provider;
	struct clk_bulk_data *bus_clks;
	int num_clks;
};

#define MSM8660_ICC_MAX_LINKS	3

/**
 * struct msm8660_icc_node - MSM8660 specific interconnect nodes
 * @name: the node name used in debugfs
 * @id: a unique node identifier
 * @links: an array of nodes where we can go next while traversing
 * @num_links: the total number of @links
 * @buswidth: width of the interconnect between a node and the bus (bytes)
 * @rate: current bus clock rate in Hz
 */
struct msm8660_icc_node {
	unsigned char *name;
	u16 id;
	u16 links[MSM8660_ICC_MAX_LINKS];
	u16 num_links;
	u16 buswidth;
	u64 rate;
};

struct msm8660_icc_desc {
	struct msm8660_icc_node * const *nodes;
	size_t num_nodes;
	const struct clk_bulk_data *bus_clks;
	size_t num_clks;
};

#define DEFINE_QNODE(_name, _id, _buswidth, ...)			\
	static struct msm8660_icc_node _name = {			\
		.name = #_name,						\
		.id = _id,						\
		.buswidth = _buswidth,					\
		.num_links = COUNT_ARGS(__VA_ARGS__),			\
		.links = { __VA_ARGS__ },				\
	}

/* APPSS Fabric nodes */
DEFINE_QNODE(mas_ampss_m0, MSM8660_AFAB_MAS_AMPSS_M0, 8,
	     MSM8660_AFAB_SLV_EBI_CH0, MSM8660_AFAB_TO_MMSS, MSM8660_AFAB_TO_SYSTEM);
DEFINE_QNODE(mas_ampss_m1, MSM8660_AFAB_MAS_AMPSS_M1, 8,
	     MSM8660_AFAB_SLV_EBI_CH0, MSM8660_AFAB_TO_MMSS, MSM8660_AFAB_TO_SYSTEM);
DEFINE_QNODE(slv_ebi_ch0, MSM8660_AFAB_SLV_EBI_CH0, 8);
DEFINE_QNODE(slv_ampss_l2, MSM8660_AFAB_SLV_AMPSS_L2, 8);
/*
 * Gateway nodes need links to both the cross-fabric gateway AND the memory
 * slave to enable cross-fabric paths. Without link to EBI_CH0, path_find()
 * can't route from MMSS/System fabric masters to main memory.
 */
DEFINE_QNODE(afab_to_mmss, MSM8660_AFAB_TO_MMSS, 8,
	     MSM8660_MMFAB_TO_APPSS, MSM8660_AFAB_SLV_EBI_CH0);
DEFINE_QNODE(afab_to_system, MSM8660_AFAB_TO_SYSTEM, 8,
	     MSM8660_SFAB_TO_APPSS, MSM8660_AFAB_SLV_EBI_CH0);

static struct msm8660_icc_node * const msm8660_afab_nodes[] = {
	[AFAB_MAS_AMPSS_M0] = &mas_ampss_m0,
	[AFAB_MAS_AMPSS_M1] = &mas_ampss_m1,
	[AFAB_SLV_EBI_CH0] = &slv_ebi_ch0,
	[AFAB_SLV_AMPSS_L2] = &slv_ampss_l2,
	[AFAB_TO_MMSS] = &afab_to_mmss,
	[AFAB_TO_SYSTEM] = &afab_to_system,
};

static const struct msm8660_icc_desc msm8660_afab = {
	.nodes = msm8660_afab_nodes,
	.num_nodes = ARRAY_SIZE(msm8660_afab_nodes),
	.bus_clks = msm8660_afab_clocks,
	.num_clks = ARRAY_SIZE(msm8660_afab_clocks),
};

/* System Fabric nodes */
DEFINE_QNODE(sfab_mas_appss, MSM8660_SFAB_MAS_APPSS, 8, MSM8660_AFAB_TO_SYSTEM);
DEFINE_QNODE(sfab_mas_sps, MSM8660_SFAB_MAS_SPS, 8, MSM8660_SFAB_SLV_SPS);
DEFINE_QNODE(sfab_mas_adm0_port0, MSM8660_SFAB_MAS_ADM0_PORT0, 8);
DEFINE_QNODE(sfab_mas_adm0_port1, MSM8660_SFAB_MAS_ADM0_PORT1, 8);
DEFINE_QNODE(sfab_mas_adm1_port0, MSM8660_SFAB_MAS_ADM1_PORT0, 8);
DEFINE_QNODE(sfab_mas_adm1_port1, MSM8660_SFAB_MAS_ADM1_PORT1, 8);
DEFINE_QNODE(sfab_mas_lpass_proc, MSM8660_SFAB_MAS_LPASS_PROC, 8);
DEFINE_QNODE(sfab_mas_mss_proci, MSM8660_SFAB_MAS_MSS_PROCI, 8);
DEFINE_QNODE(sfab_mas_mss_procd, MSM8660_SFAB_MAS_MSS_PROCD, 8);
DEFINE_QNODE(sfab_mas_mss_mdm_port0, MSM8660_SFAB_MAS_MSS_MDM_PORT0, 8);
DEFINE_QNODE(sfab_mas_lpass, MSM8660_SFAB_MAS_LPASS, 8);
DEFINE_QNODE(sfab_mas_mmss_fpb, MSM8660_SFAB_MAS_MMSS_FPB, 8);
DEFINE_QNODE(sfab_mas_adm1_ci, MSM8660_SFAB_MAS_ADM1_CI, 8);
DEFINE_QNODE(sfab_mas_adm0_ci, MSM8660_SFAB_MAS_ADM0_CI, 8);
DEFINE_QNODE(sfab_mas_mss_mdm_port1, MSM8660_SFAB_MAS_MSS_MDM_PORT1, 8);
DEFINE_QNODE(sfab_mas_usb_hs, MSM8660_SFAB_MAS_USB_HS, 8, MSM8660_SFAB_TO_APPSS);
DEFINE_QNODE(sfab_to_appss, MSM8660_SFAB_TO_APPSS, 8, MSM8660_AFAB_TO_SYSTEM);
DEFINE_QNODE(sfab_to_system_fpb, MSM8660_SFAB_TO_SYSTEM_FPB, 4);
DEFINE_QNODE(sfab_to_cpss_fpb, MSM8660_SFAB_TO_CPSS_FPB, 4);
DEFINE_QNODE(sfab_slv_sps, MSM8660_SFAB_SLV_SPS, 8);
DEFINE_QNODE(sfab_slv_system_imem, MSM8660_SFAB_SLV_SYSTEM_IMEM, 8);
DEFINE_QNODE(sfab_slv_ampss, MSM8660_SFAB_SLV_AMPSS, 8);
DEFINE_QNODE(sfab_slv_mss, MSM8660_SFAB_SLV_MSS, 8);
DEFINE_QNODE(sfab_slv_lpass, MSM8660_SFAB_SLV_LPASS, 8);
DEFINE_QNODE(sfab_slv_mmss_fpb, MSM8660_SFAB_SLV_MMSS_FPB, 8);

static struct msm8660_icc_node * const msm8660_sfab_nodes[] = {
	[SFAB_MAS_APPSS] = &sfab_mas_appss,
	[SFAB_MAS_SPS] = &sfab_mas_sps,
	[SFAB_MAS_ADM0_PORT0] = &sfab_mas_adm0_port0,
	[SFAB_MAS_ADM0_PORT1] = &sfab_mas_adm0_port1,
	[SFAB_MAS_ADM1_PORT0] = &sfab_mas_adm1_port0,
	[SFAB_MAS_ADM1_PORT1] = &sfab_mas_adm1_port1,
	[SFAB_MAS_LPASS_PROC] = &sfab_mas_lpass_proc,
	[SFAB_MAS_MSS_PROCI] = &sfab_mas_mss_proci,
	[SFAB_MAS_MSS_PROCD] = &sfab_mas_mss_procd,
	[SFAB_MAS_MSS_MDM_PORT0] = &sfab_mas_mss_mdm_port0,
	[SFAB_MAS_LPASS] = &sfab_mas_lpass,
	[SFAB_MAS_MMSS_FPB] = &sfab_mas_mmss_fpb,
	[SFAB_MAS_ADM1_CI] = &sfab_mas_adm1_ci,
	[SFAB_MAS_ADM0_CI] = &sfab_mas_adm0_ci,
	[SFAB_MAS_MSS_MDM_PORT1] = &sfab_mas_mss_mdm_port1,
	[SFAB_MAS_USB_HS] = &sfab_mas_usb_hs,
	[SFAB_TO_APPSS] = &sfab_to_appss,
	[SFAB_TO_SYSTEM_FPB] = &sfab_to_system_fpb,
	[SFAB_TO_CPSS_FPB] = &sfab_to_cpss_fpb,
	[SFAB_SLV_SPS] = &sfab_slv_sps,
	[SFAB_SLV_SYSTEM_IMEM] = &sfab_slv_system_imem,
	[SFAB_SLV_AMPSS] = &sfab_slv_ampss,
	[SFAB_SLV_MSS] = &sfab_slv_mss,
	[SFAB_SLV_LPASS] = &sfab_slv_lpass,
	[SFAB_SLV_MMSS_FPB] = &sfab_slv_mmss_fpb,
};

static const struct msm8660_icc_desc msm8660_sfab = {
	.nodes = msm8660_sfab_nodes,
	.num_nodes = ARRAY_SIZE(msm8660_sfab_nodes),
	.bus_clks = msm8660_sfab_clocks,
	.num_clks = ARRAY_SIZE(msm8660_sfab_clocks),
};

/* MMSS Fabric nodes - Multimedia subsystem (MDP, camera, video, GPU) */
DEFINE_QNODE(mmfab_mas_mdp_port0, MSM8660_MMFAB_MAS_MDP_PORT0, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_mdp_port1, MSM8660_MMFAB_MAS_MDP_PORT1, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_adm1_port0, MSM8660_MMFAB_MAS_ADM1_PORT0, 8);
DEFINE_QNODE(mmfab_mas_rotator, MSM8660_MMFAB_MAS_ROTATOR, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_graphics_3d, MSM8660_MMFAB_MAS_GRAPHICS_3D, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_jpeg_dec, MSM8660_MMFAB_MAS_JPEG_DEC, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_graphics_2d_core0, MSM8660_MMFAB_MAS_GRAPHICS_2D_CORE0, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_vfe, MSM8660_MMFAB_MAS_VFE, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_vpe, MSM8660_MMFAB_MAS_VPE, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_jpeg_enc, MSM8660_MMFAB_MAS_JPEG_ENC, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_graphics_2d_core1, MSM8660_MMFAB_MAS_GRAPHICS_2D_CORE1, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_hd_codec_port0, MSM8660_MMFAB_MAS_HD_CODEC_PORT0, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_mas_hd_codec_port1, MSM8660_MMFAB_MAS_HD_CODEC_PORT1, 16,
	     MSM8660_MMFAB_SLV_SMI, MSM8660_MMFAB_TO_APPSS);
DEFINE_QNODE(mmfab_to_appss, MSM8660_MMFAB_TO_APPSS, 8, MSM8660_AFAB_TO_MMSS);
DEFINE_QNODE(mmfab_slv_smi, MSM8660_MMFAB_SLV_SMI, 16);
DEFINE_QNODE(mmfab_slv_mm_imem, MSM8660_MMFAB_SLV_MM_IMEM, 8);

static struct msm8660_icc_node * const msm8660_mmfab_nodes[] = {
	[MMFAB_MAS_MDP_PORT0] = &mmfab_mas_mdp_port0,
	[MMFAB_MAS_MDP_PORT1] = &mmfab_mas_mdp_port1,
	[MMFAB_MAS_ADM1_PORT0] = &mmfab_mas_adm1_port0,
	[MMFAB_MAS_ROTATOR] = &mmfab_mas_rotator,
	[MMFAB_MAS_GRAPHICS_3D] = &mmfab_mas_graphics_3d,
	[MMFAB_MAS_JPEG_DEC] = &mmfab_mas_jpeg_dec,
	[MMFAB_MAS_GRAPHICS_2D_CORE0] = &mmfab_mas_graphics_2d_core0,
	[MMFAB_MAS_VFE] = &mmfab_mas_vfe,
	[MMFAB_MAS_VPE] = &mmfab_mas_vpe,
	[MMFAB_MAS_JPEG_ENC] = &mmfab_mas_jpeg_enc,
	[MMFAB_MAS_GRAPHICS_2D_CORE1] = &mmfab_mas_graphics_2d_core1,
	[MMFAB_MAS_HD_CODEC_PORT0] = &mmfab_mas_hd_codec_port0,
	[MMFAB_MAS_HD_CODEC_PORT1] = &mmfab_mas_hd_codec_port1,
	[MMFAB_TO_APPSS] = &mmfab_to_appss,
	[MMFAB_SLV_SMI] = &mmfab_slv_smi,
	[MMFAB_SLV_MM_IMEM] = &mmfab_slv_mm_imem,
};

static const struct msm8660_icc_desc msm8660_mmfab = {
	.nodes = msm8660_mmfab_nodes,
	.num_nodes = ARRAY_SIZE(msm8660_mmfab_nodes),
	.bus_clks = msm8660_mmfab_clocks,
	.num_clks = ARRAY_SIZE(msm8660_mmfab_clocks),
};

static int msm8660_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct msm8660_icc_node *src_qn;
	struct msm8660_icc_provider *qp;
	u64 sum_bw, max_peak_bw, rate;
	u32 agg_avg = 0, agg_peak = 0;
	struct icc_provider *provider;
	struct icc_node *n;
	int ret, i;

	src_qn = src->data;
	provider = src->provider;
	qp = to_msm8660_icc_provider(provider);

	list_for_each_entry(n, &provider->nodes, node_list)
		provider->aggregate(n, 0, n->avg_bw, n->peak_bw,
				    &agg_avg, &agg_peak);

	sum_bw = icc_units_to_bps(agg_avg);
	max_peak_bw = icc_units_to_bps(agg_peak);

	rate = max(sum_bw, max_peak_bw);
	do_div(rate, src_qn->buswidth);
	rate = min_t(u32, rate, INT_MAX);

	if (src_qn->rate == rate)
		return 0;

	for (i = 0; i < qp->num_clks; i++) {
		ret = clk_set_rate(qp->bus_clks[i].clk, rate);
		if (ret) {
			dev_err(provider->dev, "%s clk_set_rate error: %d\n",
				qp->bus_clks[i].id, ret);
			ret = 0;
		}
	}

	src_qn->rate = rate;

	return 0;
}

static int msm8660_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	*avg = 0;
	*peak = 0;
	return 0;
}

static int msm8660_icc_probe(struct platform_device *pdev)
{
	const struct msm8660_icc_desc *desc;
	struct msm8660_icc_node * const *qnodes;
	struct msm8660_icc_provider *qp;
	struct device *dev = &pdev->dev;
	struct icc_onecell_data *data;
	struct icc_provider *provider;
	struct icc_node *node;
	size_t num_nodes, i;
	int ret;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -EINVAL;

	qnodes = desc->nodes;
	num_nodes = desc->num_nodes;

	qp = devm_kzalloc(dev, sizeof(*qp), GFP_KERNEL);
	if (!qp)
		return -ENOMEM;

	data = devm_kzalloc(dev, struct_size(data, nodes, num_nodes),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num_nodes = num_nodes;

	qp->bus_clks = devm_kmemdup(dev, desc->bus_clks,
				    desc->num_clks * sizeof(*desc->bus_clks),
				    GFP_KERNEL);
	if (!qp->bus_clks)
		return -ENOMEM;

	qp->num_clks = desc->num_clks;

	/*
	 * MSM8660 fabric clocks are managed by RPM firmware and may not be
	 * available in mainline Linux yet. Make them optional.
	 */
	ret = devm_clk_bulk_get_optional(dev, qp->num_clks, qp->bus_clks);
	if (ret) {
		dev_warn(dev, "Failed to get bus clocks: %d (continuing anyway)\n", ret);
		qp->num_clks = 0;
	}

	if (qp->num_clks) {
		ret = clk_bulk_prepare_enable(qp->num_clks, qp->bus_clks);
		if (ret) {
			dev_warn(dev, "Failed to enable bus clocks: %d\n", ret);
			qp->num_clks = 0;
		}
	}

	provider = &qp->provider;
	provider->dev = dev;
	provider->set = msm8660_icc_set;
	provider->aggregate = icc_std_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = data;
	provider->get_bw = msm8660_get_bw;

	icc_provider_init(provider);

	for (i = 0; i < num_nodes; i++) {
		size_t j;

		if (!qnodes[i])
			continue;

		node = icc_node_create(qnodes[i]->id);
		if (IS_ERR(node)) {
			ret = PTR_ERR(node);
			goto err_remove_nodes;
		}

		node->name = qnodes[i]->name;
		node->data = qnodes[i];
		icc_node_add(node, provider);

		dev_dbg(dev, "registered node %s\n", node->name);

		/* populate links */
		for (j = 0; j < qnodes[i]->num_links; j++)
			icc_link_create(node, qnodes[i]->links[j]);

		data->nodes[i] = node;
	}

	ret = icc_provider_register(provider);
	if (ret)
		goto err_remove_nodes;

	platform_set_drvdata(pdev, qp);

	dev_info(dev, "MSM8660 interconnect provider registered\n");

	return 0;

err_remove_nodes:
	icc_nodes_remove(provider);
	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);

	return ret;
}

static void msm8660_icc_remove(struct platform_device *pdev)
{
	struct msm8660_icc_provider *qp = platform_get_drvdata(pdev);

	icc_provider_deregister(&qp->provider);
	icc_nodes_remove(&qp->provider);
	if (qp->num_clks)
		clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
}

static const struct of_device_id msm8660_noc_of_match[] = {
	{ .compatible = "qcom,msm8660-apps-fabric", .data = &msm8660_afab },
	{ .compatible = "qcom,msm8660-system-fabric", .data = &msm8660_sfab },
	{ .compatible = "qcom,msm8660-mmss-fabric", .data = &msm8660_mmfab },
	{ },
};
MODULE_DEVICE_TABLE(of, msm8660_noc_of_match);

static struct platform_driver msm8660_noc_driver = {
	.probe = msm8660_icc_probe,
	.remove = msm8660_icc_remove,
	.driver = {
		.name = "qnoc-msm8660",
		.of_match_table = msm8660_noc_of_match,
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(msm8660_noc_driver);

MODULE_DESCRIPTION("Qualcomm MSM8660/APQ8060 interconnect driver");
MODULE_LICENSE("GPL v2");
