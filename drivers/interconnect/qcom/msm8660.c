// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM8x60 family (MSM8260/MSM8660/APQ8060) interconnect driver
 *
 * Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
 *
 * Based on msm8974.c by Brian Masney <masneyb@onstation.org>
 * and legacy vendor kernel msm_bus_board_8660.c / msm_bus_fabric.c
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * MSM8x60 has a fabric-based bus architecture:
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
 *
 * Each fabric has an RPM arbitration interface that programs per-port
 * bandwidth and priority tier via MM_FABRIC_ARB / SYS_FABRIC_ARB /
 * APPS_FABRIC_ARB registers.  The legacy vendor kernel sent these as packed
 * u16 arrays (bwsum + arb) through msm_rpm_set().  This driver uses
 * the mainline qcom_rpm_write() interface to do the same.
 */

#include <dt-bindings/interconnect/qcom,msm8660.h>
#include <dt-bindings/mfd/qcom-rpm.h>

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/mfd/qcom_rpm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define to_msm8660_icc_provider(_provider) \
	container_of(_provider, struct msm8660_icc_provider, provider)

/*
 * Maximum RPM ARB buffer size across all fabrics.
 * MM fabric is largest at 23 u32 words.
 */
#define MSM8660_MAX_RPM_BUF		23

/*
 * Static pin rate for the fabric / EBI / SMI bus clocks (in Hz).
 *
 * 384 MHz is the rate the v4 (clk_set_rate-via-rpmcc) interconnect
 * driver effectively programmed under typical boot workloads on
 * MSM8x60 family (MSM8260/MSM8660/APQ8060): MSM8660_FABRIC_MIN_RATE
 * was the post-aggregation floor and the icc bandwidth aggregation
 * sat at that floor when no consumer voted a higher value.  Picking
 * the same rate here keeps RPM in a regime it is empirically known
 * to honour, and avoids the SDCC ADM-drain stalls (FLUSH_STATE5=3
 * at the eMMC channel) that an INT_MAX kHz vote (2.147 GHz) appears
 * to provoke on tenderloin under boot-time load.
 */
#define MSM8660_FABRIC_PIN_RATE		384000000UL	/* 384 MHz */

/*
 * RPM fabric arbitration data format (from legacy vendor kernel msm_bus_fabric.c):
 *
 * Each u16 arb entry: bit 15 = tier (1=TIER1 high priority), bits 14-0 = BW
 * Bandwidth is in 128KB units (bytes >> 17).
 * Two u16 values are packed into each u32 RPM register word.
 *
 * Buffer layout: [bwsum pairs] [arb pairs]
 * bwsum[slave_port] = total bandwidth to that slave
 * arb[(tier-1)*nmasters + master_port] = per-master arbitration entry
 */
#define ARB_BWMASK		GENMASK(14, 0)
#define ARB_TIERMASK		BIT(15)
#define ARB_TIER1		1
#define ARB_TIER2		2

/**
 * struct msm8660_icc_node - MSM8660 specific interconnect nodes
 * @name: the node name used in debugfs
 * @node: backing icc_node pointer, populated at probe via icc_node_create_dyn()
 * @num_links: the total number of @link_nodes
 * @buswidth: width of the interconnect between a node and the bus (bytes)
 * @mas_port: master port index for RPM ARB (-1 if not a master)
 * @slv_port: slave port index for RPM bwsum (-1 if not a slave)
 * @mas_tier: master priority tier (ARB_TIER1 or ARB_TIER2, 0 if N/A)
 * @no_arb: skip arb-row write for this master port even when @mas_port
 *	    is valid.  Used for cross-fabric gateways that legacy webOS
 *	    msm_bus_fabric (and the underlying RPM expectations) leaves
 *	    at arb=0 on their parent fabric.  Their BWSum contributions
 *	    are still aggregated so EBI / fabric clocks scale correctly;
 *	    only the arb cell is suppressed so the RPM arbiter does not
 *	    see them as competing masters competing with the real masters
 *	    on the same fabric.  Confirmed against on-device legacy
 *	    /sys/kernel/debug/msm-bus-dbg/commit-data dumps (2026-06-13):
 *	    afab_to_mmss arb is TIER1+bw when MDP votes (no_arb=false);
 *	    afab_to_system, sfab_to_appss, sfab_to_dfab, dfab_to_sfab,
 *	    and mmfab_to_appss are all 0 in their respective fabrics'
 *	    arb tables (no_arb=true).
 * @link_nodes: flexible array of pointers to qnodes reachable from this node
 */
struct msm8660_icc_node {
	const char *name;
	struct icc_node *node;
	u16 num_links;
	u16 buswidth;
	s8 mas_port;
	s8 slv_port;
	u8 mas_tier;
	bool no_arb;
	struct msm8660_icc_node *link_nodes[];
};

/**
 * struct msm8660_icc_desc - Fabric descriptor
 * @nodes: array of node pointers
 * @num_nodes: number of nodes
 * @arb_resource: QCOM_RPM_*_FABRIC_ARB constant, or -1 for no ARB
 * @bus_clk_id: QCOM_RPM_*_FABRIC_CLK constant for this fabric's bus rate vote
 * @extra_clk_id: secondary RPM clock resource voted in lockstep with the
 *                fabric clock (QCOM_RPM_EBI1_CLK on AFAB, QCOM_RPM_SMI_CLK
 *                on MMFAB), or 0 if the fabric only has one bus clock
 * @nmasters: number of master ports in this fabric (for ARB array sizing)
 * @nslaves: number of slave ports in this fabric (for bwsum array sizing)
 * @ntieredslaves: number of tiered slaves (ARB rows)
 * @default_tiered_slave: 1-based index of default tiered slave for masters
 * @rpm_buf_size: number of u32 words for RPM write
 * @bus_width: representative fabric bus width in bytes, used as the
 *             divisor for translating aggregate bytes/sec into a single
 *             clock rate that drives the whole fabric
 */
struct msm8660_icc_desc {
	struct msm8660_icc_node * const *nodes;
	size_t num_nodes;
	int arb_resource;
	u32 bus_clk_id;
	u32 extra_clk_id;
	u8 nmasters;
	u8 nslaves;
	u8 ntieredslaves;
	u8 default_tiered_slave;
	u8 rpm_buf_size;
	u8 bus_width;
};

/**
 * struct msm8660_icc_provider - MSM8660 specific interconnect provider
 * @provider: generic interconnect provider
 * @rpm: RPM handle for fabric arbitration and bus-clock writes; inherited
 *       from the parent qcom,rpm device's drvdata
 * @desc: fabric descriptor with RPM metadata
 * @arb: pre-allocated arbitration array (nmasters * ntieredslaves u16 entries)
 * @bwsum: pre-allocated bandwidth sum array (nslaves u16 entries)
 * @rpm_buf: pre-allocated RPM write buffer (rpm_buf_size u32 entries)
 * @commit_lock: serialises msm8660_rpm_commit(). The ICC core can dispatch
 *        provider->set concurrently from different CPUs; without this lock
 *        the shared @arb / @bwsum / @rpm_buf scratch buffers would race
 *        (interleaved memset() and overlapping per-node writes), corrupting
 *        the packet handed to qcom_rpm_write() and producing malformed
 *        fabric arbitration on the wire.
 */
struct msm8660_icc_provider {
	struct icc_provider provider;
	struct qcom_rpm *rpm;
	const struct msm8660_icc_desc *desc;
	u16 *arb;
	u16 *bwsum;
	u32 *rpm_buf;
	struct mutex commit_lock;
};

/*
 * Forward declarations for all qnodes.
 *
 * Because qnode definitions now use pointer-based .link_nodes = { &foo, ... }
 * with a flexible array member, every qnode that appears as a link target
 * must be visible at the point of use. Forward-declaring all qnodes up front
 * keeps the order-of-definition concern out of the per-fabric sections and
 * mirrors the pattern used by drivers/interconnect/qcom/sa8775p.c.
 */
/* APPSS Fabric */
static struct msm8660_icc_node mas_ampss_m0;
static struct msm8660_icc_node mas_ampss_m1;
static struct msm8660_icc_node slv_ebi_ch0;
static struct msm8660_icc_node slv_ampss_l2;
static struct msm8660_icc_node afab_to_mmss;
static struct msm8660_icc_node afab_to_system;

/* System Fabric */
static struct msm8660_icc_node sfab_mas_appss;
static struct msm8660_icc_node sfab_mas_sps;
static struct msm8660_icc_node sfab_mas_adm0_port0;
static struct msm8660_icc_node sfab_mas_adm0_port1;
static struct msm8660_icc_node sfab_mas_adm1_port0;
static struct msm8660_icc_node sfab_mas_adm1_port1;
static struct msm8660_icc_node sfab_mas_lpass_proc;
static struct msm8660_icc_node sfab_mas_mss_proci;
static struct msm8660_icc_node sfab_mas_mss_procd;
static struct msm8660_icc_node sfab_mas_mss_mdm_port0;
static struct msm8660_icc_node sfab_mas_lpass;
static struct msm8660_icc_node sfab_mas_mmss_fpb;
static struct msm8660_icc_node sfab_mas_adm1_ci;
static struct msm8660_icc_node sfab_mas_adm0_ci;
static struct msm8660_icc_node sfab_mas_mss_mdm_port1;
static struct msm8660_icc_node sfab_mas_usb_hs;
static struct msm8660_icc_node sfab_to_appss;
static struct msm8660_icc_node sfab_to_system_fpb;
static struct msm8660_icc_node sfab_to_cpss_fpb;
static struct msm8660_icc_node sfab_slv_sps;
static struct msm8660_icc_node sfab_slv_system_imem;
static struct msm8660_icc_node sfab_slv_ampss;
static struct msm8660_icc_node sfab_slv_mss;
static struct msm8660_icc_node sfab_slv_lpass;
static struct msm8660_icc_node sfab_slv_mmss_fpb;
static struct msm8660_icc_node sfab_to_dfab;

/* MMSS Fabric */
static struct msm8660_icc_node mmfab_mas_mdp_port0;
static struct msm8660_icc_node mmfab_mas_mdp_port1;
static struct msm8660_icc_node mmfab_mas_adm1_port0;
static struct msm8660_icc_node mmfab_mas_rotator;
static struct msm8660_icc_node mmfab_mas_graphics_3d;
static struct msm8660_icc_node mmfab_mas_jpeg_dec;
static struct msm8660_icc_node mmfab_mas_graphics_2d_core0;
static struct msm8660_icc_node mmfab_mas_vfe;
static struct msm8660_icc_node mmfab_mas_vpe;
static struct msm8660_icc_node mmfab_mas_jpeg_enc;
static struct msm8660_icc_node mmfab_mas_graphics_2d_core1;
static struct msm8660_icc_node mmfab_mas_hd_codec_port0;
static struct msm8660_icc_node mmfab_mas_hd_codec_port1;
static struct msm8660_icc_node mmfab_to_appss;
static struct msm8660_icc_node mmfab_slv_smi;
static struct msm8660_icc_node mmfab_slv_mm_imem;

/* Daytona Fabric (DFAB) */
static struct msm8660_icc_node dfab_mas_sdc1;
static struct msm8660_icc_node dfab_mas_sdc2;
static struct msm8660_icc_node dfab_mas_sdc3;
static struct msm8660_icc_node dfab_mas_sdc4;
static struct msm8660_icc_node dfab_mas_sdc5;
static struct msm8660_icc_node dfab_mas_adm0_master;
static struct msm8660_icc_node dfab_mas_adm1_master;
static struct msm8660_icc_node dfab_to_sfab;
static struct msm8660_icc_node dfab_slv_sdc1;
static struct msm8660_icc_node dfab_slv_sdc2;
static struct msm8660_icc_node dfab_slv_sdc3;
static struct msm8660_icc_node dfab_slv_sdc4;
static struct msm8660_icc_node dfab_slv_sdc5;
static struct msm8660_icc_node dfab_mas_usb_hs;
static struct msm8660_icc_node dfab_mas_dsps;

/*
 * APPSS Fabric nodes
 *
 * 4 masters, 4 slaves, 2 tiered slaves
 * Master ports: SMPSS_M0=0, SMPSS_M1=1, FAB_MMSS=2, FAB_SYSTEM=3
 * Slave ports:  EBI_CH0=0, SMPSS_L2=1, MMSS_FAB=2, SYSTEM_FAB=3
 * Default target: tiered slave 1 (EBI_CH0)
 */
static struct msm8660_icc_node mas_ampss_m0 = {
	.name = "mas_ampss_m0",
	.num_links = 3,
	.buswidth = 8,
	.mas_port = 0,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &slv_ebi_ch0, &afab_to_mmss, &afab_to_system },
};

static struct msm8660_icc_node mas_ampss_m1 = {
	.name = "mas_ampss_m1",
	.num_links = 3,
	.buswidth = 8,
	.mas_port = 1,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &slv_ebi_ch0, &afab_to_mmss, &afab_to_system },
};

static struct msm8660_icc_node slv_ebi_ch0 = {
	.name = "slv_ebi_ch0",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 0,
	.mas_tier = 0,
};

static struct msm8660_icc_node slv_ampss_l2 = {
	.name = "slv_ampss_l2",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 1,
	.mas_tier = 0,
};

/*
 * Gateway nodes need links to both the cross-fabric gateway AND the memory
 * slave to enable cross-fabric paths. Without link to EBI_CH0, path_find()
 * can't route from MMSS/System fabric masters to main memory.
 *
 * AFAB_TO_MMSS doubles as AFAB master port 2 (the FAB_MMSS master). MDP
 * scanout and GPU traffic enter AFAB through this gateway.
 *
 * 2026-06-13: TIER1 RESTORED on afab_to_mmss after live legacy webOS
 * capture (/sys/kernel/debug/msm-bus-dbg/commit-data/msm_apps_fab on
 * topaz running webOS 3.0.5) showed the EBI_CH0 arb cell for AFAB
 * master port 2 contains TIER1+bw whenever MDP votes EBI traffic:
 *
 *   msm_apps_fab BWSum:
 *     0x9f4  0x0  0x0  0x0
 *   Arb TSlave 0 (EBI_CH0 row):
 *     0x0   0x0   0x89f4   0x0
 *                  ^^^^^^
 *                  TIER1 bit (0x8000) + 0x9f4 BW
 *                  (afab_to_mmss master port carrying MDP fetch traffic)
 *
 * The legacy msm_bus_board_8660.c gateway nodes have no static `.tier`
 * field, but the runtime path-traversal in msm_bus_fabric_update_bw
 * accumulates the SLAVE's tier (EBI_CH0 = TIER1) onto each transit
 * master arb cell.  Mainline must mirror this so MDP TIER1 priority is
 * preserved across the MMFAB->AFAB boundary as legacy intended.
 *
 * The earlier concern that TIER1 on this gateway would starve ADM/SDC
 * is addressed by @no_arb on afab_to_system (gateway carrying SDC/ADM
 * traffic) -- in legacy that gateway has 0 arb entry because SDC/ADM
 * don't vote msm_bus_scale at all (they drive EBI via clk_set_rate on
 * ebi1_clk directly).  Mainline mmci + qcom_adm DO icc_set_bw, which
 * would propagate BW into afab_to_system's arb cell at TIER2 and create
 * the contention legacy never had.  Suppressing the arb cell on the
 * SDC/ADM gateway (while keeping its BWSum aggregation so EBI clock
 * still scales) restores the legacy ordering: MDP fetches at TIER1
 * unopposed, SDC/ADM drain on hardware-default round-robin at the EBI
 * floor clock rate (314 MHz min observed live on legacy).
 *
 * Full audit at Documentation/icc-msm8660-audit-2026-06-13.md;
 * legacy state capture in reports/icc-legacy-live-state-2026-06-13.md.
 */
static struct msm8660_icc_node afab_to_mmss = {
	.name = "afab_to_mmss",
	.num_links = 2,
	.buswidth = 8,
	.mas_port = 2,
	.slv_port = 2,
	.mas_tier = ARB_TIER1,
	.link_nodes = { &mmfab_to_appss, &slv_ebi_ch0 },
};

/*
 * afab_to_system: SFAB->AFAB gateway -- the AFAB-side master that
 * receives all SFAB-originated traffic (mmci sdcc1, mmci sdcc4,
 * qcom_adm channels) and propagates it to slv_ebi_ch0.
 *
 * @no_arb = true: legacy webOS keeps this AFAB arb cell at 0 because
 * msm_bus_fabric never sees any consumer voting through it -- SDC and
 * ADM drivers in legacy use clock-tree voting (clk_set_rate on
 * ebi1_clk, dfab_pclk) rather than msm_bus_scale.  Live capture
 * confirms:
 *
 *   Arb TSlave 0 (EBI_CH0 row):
 *     0x0   0x0   0x89f4   0x0
 *                          ^^^
 *                          port 3 (afab_to_system) = 0 (legacy)
 *
 * Mainline mmci + qcom_adm DO icc_set_bw, so path traversal would
 * accumulate (SDC1 + SDC4 + ADM1 PORT0 + ADM1 PORT1) bandwidth into
 * this gateway's arb cell at ARB_TIER2.  That mainline-only arb entry
 * competes against afab_to_mmss (TIER1, MDP) for EBI -- but more
 * critically, when MDP is NOT voting (afab_to_mmss arb=0) the gateway
 * still represents a large TIER2 demand that confuses the RPM arbiter
 * relative to the legacy fabric where this cell is always 0.
 *
 * Setting no_arb = true tells msm8660_rpm_commit() to skip the arb
 * write for this node while still aggregating BWSum on slv_ebi_ch0
 * (so EBI clock scales with SDC/ADM bandwidth demand, matching what
 * legacy's separate ebi1_clk path achieves).
 */
static struct msm8660_icc_node afab_to_system = {
	.name = "afab_to_system",
	.num_links = 2,
	.buswidth = 8,
	.mas_port = 3,
	.slv_port = 3,
	.mas_tier = ARB_TIER2,
	.no_arb = true,
	.link_nodes = { &sfab_to_appss, &slv_ebi_ch0 },
};

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
	.arb_resource = QCOM_RPM_APPS_FABRIC_ARB,
	.bus_clk_id = QCOM_RPM_APPS_FABRIC_CLK,
	.extra_clk_id = QCOM_RPM_EBI1_CLK,
	.nmasters = 4,
	.nslaves = 4,
	.ntieredslaves = 2,
	.default_tiered_slave = 1,	/* EBI_CH0 */
	.rpm_buf_size = 6,
	.bus_width = 8,			/* 64-bit APPSS fabric datapath */
};

/*
 * System Fabric nodes
 *
 * 17 masters, 9 slaves, 2 tiered slaves
 * Master ports: see enum msm_bus_8660_master_ports_type in legacy vendor kernel
 * Slave ports:  APPSS_FAB=0, SPS=1, SYSTEM_IMEM=2, SMPSS=3, MSS=4,
 *               LPASS=5, CPSS_FPB=6, SYSTEM_FPB=7, MMSS_FPB=8
 * Default target: tiered slave 1 (APPSS gateway)
 */
static struct msm8660_icc_node sfab_mas_appss = {
	.name = "sfab_mas_appss",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = 0,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &afab_to_system },
};

static struct msm8660_icc_node sfab_mas_sps = {
	.name = "sfab_mas_sps",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = 1,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &sfab_slv_sps },
};

/*
 * ADM DMA masters - route through SFAB_TO_APPSS to reach EBI memory.
 * Path: ADM -> SFAB_TO_APPSS -> AFAB_TO_SYSTEM -> AFAB_SLV_EBI_CH0
 * This enables proper EBI bandwidth voting for DMA operations.
 */
static struct msm8660_icc_node sfab_mas_adm0_port0 = {
	.name = "sfab_mas_adm0_port0",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = 2,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &sfab_to_appss },
};

static struct msm8660_icc_node sfab_mas_adm0_port1 = {
	.name = "sfab_mas_adm0_port1",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = 3,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &sfab_to_appss },
};

static struct msm8660_icc_node sfab_mas_adm1_port0 = {
	.name = "sfab_mas_adm1_port0",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = 4,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &sfab_to_appss },
};

static struct msm8660_icc_node sfab_mas_adm1_port1 = {
	.name = "sfab_mas_adm1_port1",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = 5,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &sfab_to_appss },
};

static struct msm8660_icc_node sfab_mas_lpass_proc = {
	.name = "sfab_mas_lpass_proc",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 6,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_mss_proci = {
	.name = "sfab_mas_mss_proci",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 7,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_mss_procd = {
	.name = "sfab_mas_mss_procd",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 8,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_mss_mdm_port0 = {
	.name = "sfab_mas_mss_mdm_port0",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 9,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_lpass = {
	.name = "sfab_mas_lpass",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 10,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_mmss_fpb = {
	.name = "sfab_mas_mmss_fpb",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 13,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_adm1_ci = {
	.name = "sfab_mas_adm1_ci",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 14,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_adm0_ci = {
	.name = "sfab_mas_adm0_ci",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 15,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node sfab_mas_mss_mdm_port1 = {
	.name = "sfab_mas_mss_mdm_port1",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 16,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

/* USB HS has no dedicated master port in legacy vendor kernel SFAB - bandwidth voting only */
static struct msm8660_icc_node sfab_mas_usb_hs = {
	.name = "sfab_mas_usb_hs",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &sfab_to_appss },
};

static struct msm8660_icc_node sfab_to_appss = {
	.name = "sfab_to_appss",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 0,
	.mas_tier = 0,
	.link_nodes = { &afab_to_system },
};

static struct msm8660_icc_node sfab_to_system_fpb = {
	.name = "sfab_to_system_fpb",
	.num_links = 0,
	.buswidth = 4,
	.mas_port = -1,
	.slv_port = 7,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_to_cpss_fpb = {
	.name = "sfab_to_cpss_fpb",
	.num_links = 0,
	.buswidth = 4,
	.mas_port = -1,
	.slv_port = 6,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_slv_sps = {
	.name = "sfab_slv_sps",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 1,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_slv_system_imem = {
	.name = "sfab_slv_system_imem",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 2,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_slv_ampss = {
	.name = "sfab_slv_ampss",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 3,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_slv_mss = {
	.name = "sfab_slv_mss",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 4,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_slv_lpass = {
	.name = "sfab_slv_lpass",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 5,
	.mas_tier = 0,
};

static struct msm8660_icc_node sfab_slv_mmss_fpb = {
	.name = "sfab_slv_mmss_fpb",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 8,
	.mas_tier = 0,
};

/*
 * Gateway to DFAB: links to DFAB_TO_SFAB for path traversal.
 * Also links to SFAB_TO_APPSS to enable DFAB->SFAB->AFAB->memory paths.
 * No slave port in legacy vendor kernel SFAB config (DFAB is separate fabric).
 */
static struct msm8660_icc_node sfab_to_dfab = {
	.name = "sfab_to_dfab",
	.num_links = 2,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab, &sfab_to_appss },
};

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
	[SFAB_TO_DFAB] = &sfab_to_dfab,
};

static const struct msm8660_icc_desc msm8660_sfab = {
	.nodes = msm8660_sfab_nodes,
	.num_nodes = ARRAY_SIZE(msm8660_sfab_nodes),
	.arb_resource = QCOM_RPM_SYS_FABRIC_ARB,
	.bus_clk_id = QCOM_RPM_SYS_FABRIC_CLK,
	.nmasters = 17,
	.nslaves = 9,
	.ntieredslaves = 2,
	.default_tiered_slave = 1,	/* APPSS gateway */
	.rpm_buf_size = 22,
	.bus_width = 8,			/* 64-bit System fabric datapath */
};

/*
 * MMSS Fabric nodes - Multimedia subsystem (MDP, camera, video, GPU)
 *
 * 14 masters, 4 slaves, 3 tiered slaves
 * Master ports: MDP0=0, MDP1=1, ADM1=2, ROT=3, 3D=4, JPEG_DEC=5,
 *               2D_CORE0=6, VFE=7, VPE=8, JPEG_ENC=9, 2D_CORE1=10,
 *               (APPS_FAB=11), HD_CODEC0=12, HD_CODEC1=13
 * Slave ports:  SMI=0, APPSS_FAB=1, (APPSS_FAB_1=2), MM_IMEM=3
 * Tiered slaves: SMI=1, APPSS_FAB=2, MM_IMEM=3
 * Default target: tiered slave 2 (APPSS gateway -> main memory)
 *
 * MDP ports get TIER1 (high priority) for guaranteed display refresh.
 * All other masters get TIER2 (default priority).
 */
static struct msm8660_icc_node mmfab_mas_mdp_port0 = {
	.name = "mmfab_mas_mdp_port0",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 0,
	.slv_port = -1,
	.mas_tier = ARB_TIER1,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_mdp_port1 = {
	.name = "mmfab_mas_mdp_port1",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 1,
	.slv_port = -1,
	.mas_tier = ARB_TIER1,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_adm1_port0 = {
	.name = "mmfab_mas_adm1_port0",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = 2,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
};

static struct msm8660_icc_node mmfab_mas_rotator = {
	.name = "mmfab_mas_rotator",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 3,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_graphics_3d = {
	.name = "mmfab_mas_graphics_3d",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 4,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_jpeg_dec = {
	.name = "mmfab_mas_jpeg_dec",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 5,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_graphics_2d_core0 = {
	.name = "mmfab_mas_graphics_2d_core0",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 6,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_vfe = {
	.name = "mmfab_mas_vfe",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 7,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_vpe = {
	.name = "mmfab_mas_vpe",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 8,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_jpeg_enc = {
	.name = "mmfab_mas_jpeg_enc",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 9,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_graphics_2d_core1 = {
	.name = "mmfab_mas_graphics_2d_core1",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 10,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_hd_codec_port0 = {
	.name = "mmfab_mas_hd_codec_port0",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 12,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

static struct msm8660_icc_node mmfab_mas_hd_codec_port1 = {
	.name = "mmfab_mas_hd_codec_port1",
	.num_links = 2,
	.buswidth = 16,
	.mas_port = 13,
	.slv_port = -1,
	.mas_tier = ARB_TIER2,
	.link_nodes = { &mmfab_slv_smi, &mmfab_to_appss },
};

/*
 * Gateway from APPSS into MMSS: slave (port 1) for outbound traffic
 * leaving MMSS, AND master (port 11) for inbound traffic arriving from
 * APPSS (e.g. CPU memremap_wc accesses to SMI BOs). Without the master
 * port and the forward links into MMFAB slaves (SMI / MM_IMEM), the
 * ICC path-finder has no route from AMPSS_M0 to MMFAB_SLV_SMI; the
 * cross-fabric gateway only worked for outbound traffic (MMSS masters
 * reaching APPSS slaves like EBI).
 *
 * @no_arb = true: legacy live capture of MMFAB arb at port 11 (this
 * gateway's master port within MMFAB) is always 0 regardless of MDP,
 * GPU, codec or camera activity:
 *
 *   msm_mm_fab Arb TSlave 1 (APPSS gateway row):
 *     0x89f4 0x0 0x0 0x0 0x3bd7 0x0 ... 0x0
 *     ^^^^^^                              ^^
 *     port 0 = MDP_PORT0 (TIER1+bw)       port 11 = mmfab_to_appss = 0
 *
 * Legacy populates only the real MMSS master arb cells (MDP, GPU);
 * the gateway port is not a "competing master" against them within
 * MMFAB.  Mirroring that here keeps the arb cells representing the
 * actual MMFAB-local arbitration ordering.  BWSum on slv_port 1 still
 * aggregates the outbound MMSS->AFAB traffic (so the MMFAB clock
 * scales correctly).
 */
static struct msm8660_icc_node mmfab_to_appss = {
	.name = "mmfab_to_appss",
	.num_links = 3,
	.buswidth = 8,
	.mas_port = 11,
	.slv_port = 1,
	.mas_tier = ARB_TIER2,
	.no_arb = true,
	.link_nodes = { &afab_to_mmss, &mmfab_slv_smi, &mmfab_slv_mm_imem },
};

static struct msm8660_icc_node mmfab_slv_smi = {
	.name = "mmfab_slv_smi",
	.num_links = 0,
	.buswidth = 16,
	.mas_port = -1,
	.slv_port = 0,
	.mas_tier = 0,
};

static struct msm8660_icc_node mmfab_slv_mm_imem = {
	.name = "mmfab_slv_mm_imem",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = 3,
	.mas_tier = 0,
};

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
	.arb_resource = QCOM_RPM_MM_FABRIC_ARB,
	.bus_clk_id = QCOM_RPM_MM_FABRIC_CLK,
	.extra_clk_id = QCOM_RPM_SMI_CLK,
	.nmasters = 14,
	.nslaves = 4,
	.ntieredslaves = 3,
	.default_tiered_slave = 2,	/* APPSS gateway */
	.rpm_buf_size = 23,
	.bus_width = 16,		/* 128-bit Multimedia fabric datapath */
};

/*
 * Daytona Fabric (DFAB) nodes - peripheral bus for SDCC and ADM DMA
 *
 * DFAB connects slower peripherals to SFAB via the DFAB_TO_SFAB gateway.
 * SDCC controllers (eMMC, SD card) connect here.
 *
 * No RPM ARB for DFAB - it's a simple peripheral bus with clock-only control.
 *
 * USB HS is included as a DFAB voter for compatibility with the
 * legacy vendor kernel clock voting mechanism.
 */
static struct msm8660_icc_node dfab_mas_sdc1 = {
	.name = "dfab_mas_sdc1",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_mas_sdc2 = {
	.name = "dfab_mas_sdc2",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_mas_sdc3 = {
	.name = "dfab_mas_sdc3",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_mas_sdc4 = {
	.name = "dfab_mas_sdc4",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_mas_sdc5 = {
	.name = "dfab_mas_sdc5",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_mas_adm0_master = {
	.name = "dfab_mas_adm0_master",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_mas_adm1_master = {
	.name = "dfab_mas_adm1_master",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node dfab_to_sfab = {
	.name = "dfab_to_sfab",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &sfab_to_dfab },
};

static struct msm8660_icc_node dfab_slv_sdc1 = {
	.name = "dfab_slv_sdc1",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
};

static struct msm8660_icc_node dfab_slv_sdc2 = {
	.name = "dfab_slv_sdc2",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
};

static struct msm8660_icc_node dfab_slv_sdc3 = {
	.name = "dfab_slv_sdc3",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
};

static struct msm8660_icc_node dfab_slv_sdc4 = {
	.name = "dfab_slv_sdc4",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
};

static struct msm8660_icc_node dfab_slv_sdc5 = {
	.name = "dfab_slv_sdc5",
	.num_links = 0,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
};

/* USB HS DFAB voter - keeps DFAB clock stable during USB activity */
static struct msm8660_icc_node dfab_mas_usb_hs = {
	.name = "dfab_mas_usb_hs",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

/* DSPS DFAB voter - keeps DFAB clock stable during sensor activity */
static struct msm8660_icc_node dfab_mas_dsps = {
	.name = "dfab_mas_dsps",
	.num_links = 1,
	.buswidth = 8,
	.mas_port = -1,
	.slv_port = -1,
	.mas_tier = 0,
	.link_nodes = { &dfab_to_sfab },
};

static struct msm8660_icc_node * const msm8660_dfab_nodes[] = {
	[DFAB_MAS_SDC1] = &dfab_mas_sdc1,
	[DFAB_MAS_SDC2] = &dfab_mas_sdc2,
	[DFAB_MAS_SDC3] = &dfab_mas_sdc3,
	[DFAB_MAS_SDC4] = &dfab_mas_sdc4,
	[DFAB_MAS_SDC5] = &dfab_mas_sdc5,
	[DFAB_MAS_ADM0_MASTER] = &dfab_mas_adm0_master,
	[DFAB_MAS_ADM1_MASTER] = &dfab_mas_adm1_master,
	[DFAB_TO_SFAB] = &dfab_to_sfab,
	[DFAB_SLV_SDC1] = &dfab_slv_sdc1,
	[DFAB_SLV_SDC2] = &dfab_slv_sdc2,
	[DFAB_SLV_SDC3] = &dfab_slv_sdc3,
	[DFAB_SLV_SDC4] = &dfab_slv_sdc4,
	[DFAB_SLV_SDC5] = &dfab_slv_sdc5,
	[DFAB_MAS_USB_HS] = &dfab_mas_usb_hs,
	[DFAB_MAS_DSPS] = &dfab_mas_dsps,
};

static const struct msm8660_icc_desc msm8660_dfab = {
	.nodes = msm8660_dfab_nodes,
	.num_nodes = ARRAY_SIZE(msm8660_dfab_nodes),
	.arb_resource = -1,		/* No RPM ARB for DFAB */
	.bus_clk_id = QCOM_RPM_DAYTONA_FABRIC_CLK,
	.bus_width = 8,			/* 64-bit Daytona fabric datapath */
};

/*
 * Pack bwsum[] and arb[] arrays into the u32 RPM buffer.
 *
 * Two u16 values are packed per u32 word: lower 16 bits first, upper 16 next.
 * Layout: [bwsum pairs] then [arb pairs], handling odd boundaries.
 *
 * This matches the legacy vendor kernel msm_bus_fabric_rpm_commit() packing algorithm.
 */
#define RPM_PAIR_LO	GENMASK(15, 0)
#define RPM_PAIR_HI	GENMASK(31, 16)

static void msm8660_pack_rpm_data(const u16 *bwsum, int nslaves,
				  const u16 *arb, int arb_size,
				  u32 *buf)
{
	int i, index = 0;

	/* Pack bwsum pairs */
	for (i = 0; i + 1 < nslaves; i += 2) {
		buf[index] = FIELD_PREP(RPM_PAIR_HI, bwsum[i + 1]) |
			     FIELD_PREP(RPM_PAIR_LO, bwsum[i]);
		index++;
	}

	/*
	 * Handle boundary between bwsum and arb for odd nslaves. When the
	 * fabric has no master ports (arb_size == 0) the arb[0] access would
	 * read out of bounds; pad the lone bwsum into the low half of the
	 * word instead.
	 */
	if (nslaves & BIT(0)) {
		if (arb_size > 0) {
			buf[index] = FIELD_PREP(RPM_PAIR_HI, arb[0]) |
				     FIELD_PREP(RPM_PAIR_LO, bwsum[i]);
			i = 1;
		} else {
			buf[index] = FIELD_PREP(RPM_PAIR_LO, bwsum[i]);
			i = 0;
		}
		index++;
	} else {
		i = 0;
	}

	/* Pack arb pairs */
	for (; i + 1 < arb_size; i += 2) {
		buf[index] = FIELD_PREP(RPM_PAIR_HI, arb[i + 1]) |
			     FIELD_PREP(RPM_PAIR_LO, arb[i]);
		index++;
	}

	/* Handle odd arb entry at end */
	if (i < arb_size) {
		buf[index] = FIELD_PREP(RPM_PAIR_LO, arb[i]);
		index++;
	}
}

/*
 * Send fabric arbitration data to RPM.
 *
 * Iterates over all ICC nodes in the provider, builds bwsum/arb arrays
 * from their aggregated bandwidth, and sends the packed data to RPM.
 */
static void msm8660_rpm_commit(struct msm8660_icc_provider *qp)
{
	const struct msm8660_icc_desc *desc = qp->desc;
	struct icc_provider *provider = &qp->provider;
	int nm = desc->nmasters;
	int ns = desc->nslaves;
	int nts = desc->ntieredslaves;
	int arb_size = nm * nts;
	int def_ts = desc->default_tiered_slave;
	struct icc_node *n;
	int ret;

	memset(qp->bwsum, 0, ns * sizeof(u16));
	memset(qp->arb, 0, arb_size * sizeof(u16));
	memset(qp->rpm_buf, 0, desc->rpm_buf_size * sizeof(u32));

	list_for_each_entry(n, &provider->nodes, node_list) {
		struct msm8660_icc_node *qn = n->data;
		u64 bw_bytes;
		u16 bw_128k;

		/* Use max of avg and peak bandwidth, convert to 128KB units */
		bw_bytes = max(icc_units_to_bps(n->avg_bw),
			       icc_units_to_bps(n->peak_bw));
		bw_128k = (u16)min_t(u64, bw_bytes >> 17, ARB_BWMASK);

		/*
		 * Set arb entry for masters at their default tiered slave.
		 *
		 * @no_arb gateways are skipped here per legacy webOS live
		 * arb captures (commit-data/msm_apps_fab on running webOS
		 * 3.0.5, 2026-06-13): cross-fabric gateways carrying
		 * SDC/ADM-class traffic that legacy never sees via
		 * msm_bus_scale -- mainline's icc_set_bw votes would
		 * propagate their accumulated bw into a phantom TIER2 arb
		 * cell that competes with the legitimate TIER1 master
		 * arbitration (afab_to_mmss/MDP).  Their BWSum contribution
		 * is still applied below so EBI / fabric clocks scale.
		 */
		if (qn->mas_port >= 0 && qn->mas_port < nm && def_ts > 0 &&
		    !qn->no_arb) {
			int idx = (def_ts - 1) * nm + qn->mas_port;
			u8 tier;

			if (idx < arb_size) {
				tier = bw_128k ? qn->mas_tier : ARB_TIER2;
				qp->arb[idx] = (tier == ARB_TIER1 ? ARB_TIERMASK : 0)
					       | (bw_128k & ARB_BWMASK);
			}
		}

		/* Set bwsum for slaves */
		if (qn->slv_port >= 0 && qn->slv_port < ns)
			qp->bwsum[qn->slv_port] = bw_128k;
	}

	msm8660_pack_rpm_data(qp->bwsum, ns, qp->arb, arb_size, qp->rpm_buf);

	ret = qcom_rpm_write(qp->rpm, QCOM_RPM_ACTIVE_STATE,
			     desc->arb_resource, qp->rpm_buf,
			     desc->rpm_buf_size);
	if (ret)
		dev_err_ratelimited(provider->dev,
				    "RPM fabric ARB write failed: %d\n", ret);
}

/*
 * Vote one fabric/EBI/SMI bus-clock rate to RPM.
 *
 * Writes the rate (in kHz) to both QCOM_RPM_ACTIVE_STATE and
 * QCOM_RPM_SLEEP_STATE for the resource, matching the effective
 * behaviour of the legacy clk-rpm path: both peers' clk_rpm_prepare()
 * ended up driving ACTIVE = SLEEP = rate via max() aggregation on
 * msm8660 fabric clocks.
 *
 * The clock framework's rpmcc node used to own these resources for
 * MSM8x60, but exposing them in both rpmcc and the interconnect
 * provider lets two writers race the same RPM resource.  This driver
 * is now the single source of truth.
 */
/*
 * Vote @rate (Hz) for a fabric / EBI / SMI bus-clock RPM resource.
 *
 * The ACTIVE_STATE write uses qcom_rpm_write_sync(), which queues the
 * vote and polls the per-resource status register until the applied
 * value is at least the requested one.  qcom_rpm_write_sync() is
 * best-effort: on SoCs whose status register does not actually track
 * the requested unit (e.g. MSM8x60 family, where empirical data shows
 * the status word for QCOM_RPM_*_FABRIC_CLK and friends never matches
 * the kHz value the RPM accepted), the helper logs a one-shot diagnostic
 * and falls back to fire-and-forget instead of failing the caller.
 *
 * The SLEEP_STATE write uses plain qcom_rpm_write(); the status
 * register only reflects sleep-context values during cluster sleep,
 * so polling against a sleep vote during the active phase would just
 * time out.  The qcom_rpm_write() ack is the strongest synchronisation
 * available for sleep votes.
 */
static int msm8660_rpm_set_bus_rate(struct qcom_rpm *rpm, u32 resource,
				    u64 rate)
{
	u32 khz = DIV_ROUND_UP_ULL(rate, 1000);
	int ret;

	ret = qcom_rpm_write_sync(rpm, QCOM_RPM_ACTIVE_STATE, resource,
				  &khz, 1);
	if (ret)
		return ret;
	return qcom_rpm_write(rpm, QCOM_RPM_SLEEP_STATE, resource, &khz, 1);
}

/*
 * u64-safe replacement for icc_std_aggregate(): the standard helper sums
 * average bandwidth into a u32, which can wrap around when summed across
 * many high-bandwidth nodes. We accumulate in u64 internally and saturate
 * back to U32_MAX on overflow rather than wrapping silently to a small
 * value that would underclock the fabric.
 */
static int msm8660_icc_aggregate(struct icc_node *node, u32 tag,
				 u32 avg_bw, u32 peak_bw,
				 u32 *agg_avg, u32 *agg_peak)
{
	u64 new_avg = (u64)*agg_avg + avg_bw;

	*agg_avg = (new_avg > U32_MAX) ? U32_MAX : (u32)new_avg;
	*agg_peak = max(*agg_peak, peak_bw);
	return 0;
}

static int msm8660_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct msm8660_icc_provider *qp;
	struct icc_provider *provider;

	provider = src->provider;
	qp = to_msm8660_icc_provider(provider);

	/*
	 * No dynamic fabric bus-clock scaling at runtime.
	 *
	 * The non-SMD `qcom_rpm` IPC used by the MSM8x60 family acks a
	 * resource write as soon as the request is queued, not when the
	 * RPM firmware has actually applied the new clock rate to the
	 * physical fabric.  Consumer drivers that vote bandwidth and
	 * then immediately issue a DMA (mmci-pl18x via the ADM, in
	 * particular) can race the rate ramp: ACK -> consumer submits DMA
	 * -> ADM "CMD_PTR_RDY not set at submit" before the fabric has
	 * caught up.  v4 happened to mask this through clk-rpm's per-clk
	 * iteration providing ~2x more IPC round-trips of implicit
	 * settling time per icc_set, but v5's direct RPM voting is too
	 * fast.
	 *
	 * Until a `qcom_rpm_write_sync` (status-register polling) helper
	 * exists in drivers/mfd/qcom_rpm.c, the only safe pattern on this
	 * RPM family is the one APQ8064's SCM driver already uses on the
	 * Daytona fabric clock: pin it at INT_MAX at probe and leave it
	 * there for the lifetime of the system -- see qcom_scm_probe()
	 * in drivers/firmware/qcom/qcom_scm.c, "vote for max clk rate for
	 * highest performance".  The kickstart writes the fabric, EBI and
	 * SMI clocks to INT_MAX at probe; this function only updates the
	 * RPM ARB (per-master / per-tiered-slave bandwidth tier) buffer,
	 * which is what real consumers actually depend on at runtime.
	 */
	if (qp->desc->arb_resource >= 0) {
		guard(mutex)(&qp->commit_lock);
		msm8660_rpm_commit(qp);
	}

	return 0;
}

/*
 * Drop cached icc_node * pointers stored in the static qnode templates.
 * icc_nodes_remove() / icc_node_destroy() frees the icc_node memory, but
 * because @qnodes is a const array of pointers into long-lived static
 * storage, the ->node field outlives a probe failure and a subsequent
 * unbind/rebind cycle. Without this clear, the next probe's
 * "if (!qn->node) icc_node_create_dyn();" check sees the dangling cached
 * pointer, skips re-allocation and hands the freed pointer back to the
 * interconnect core -- use-after-free. icc_link_nodes() can also populate
 * a target qnode's ->node mid-loop, so we always walk the full array.
 */
static void msm8660_clear_node_cache(struct msm8660_icc_node * const *qnodes,
				     size_t num_nodes)
{
	size_t i;

	for (i = 0; i < num_nodes; i++)
		if (qnodes[i])
			qnodes[i]->node = NULL;
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

	ret = devm_mutex_init(dev, &qp->commit_lock);
	if (ret)
		return ret;

	data = devm_kzalloc(dev, struct_size(data, nodes, num_nodes),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num_nodes = num_nodes;

	qp->desc = desc;

	/*
	 * The interconnect provider is a child of the qcom,rpm DT node,
	 * so the RPM driver always probes first and stores its struct
	 * qcom_rpm in the parent's drvdata.  The driver core wires the
	 * parent/child relationship and tracks the supplier link for us,
	 * so we just consume the handle here.
	 */
	qp->rpm = dev_get_drvdata(dev->parent);
	if (!qp->rpm)
		return dev_err_probe(dev, -ENODEV,
				     "parent RPM device has no drvdata\n");

	/*
	 * Pin the fabric / EBI / SMI bus clocks at MSM8660_FABRIC_PIN_RATE
	 * for the life of the system.  See the comment block in
	 * msm8660_icc_set() for why dynamic rate scaling cannot be done
	 * safely on the non-SMD `qcom_rpm` IPC family until a
	 * status-polling helper exists in drivers/mfd/qcom_rpm.c.
	 *
	 * Rate choice: 384 MHz matches the MSM8660_FABRIC_MIN_RATE floor
	 * that the v4 (clk_set_rate-via-rpmcc) interconnect driver applied
	 * to its computed-from-bandwidth rate before passing it to
	 * clk_rpm_set_rate, which is the rate that was effectively
	 * programmed into the RPM under typical boot workloads on
	 * tenderloin (HP TouchPad).  Earlier v5 iterations used INT_MAX,
	 * mirroring drivers/firmware/qcom/qcom_scm.c's APQ8064 Daytona
	 * pin pattern; on this hardware INT_MAX as a kHz vote
	 * (2.147 GHz) appears to be silently rejected or rolled back by
	 * the RPM firmware under load, producing SDCC ADM-drain stalls
	 * (FLUSH_STATE5=3 at the eMMC channel).  A concrete
	 * known-good-on-v4 rate avoids that.
	 *
	 * msm8660_rpm_set_bus_rate() writes both ACTIVE_STATE and
	 * SLEEP_STATE so the rate also applies during cluster-sleep
	 * windows; otherwise an in-flight DMA can race a transient
	 * micro-sleep to a SLEEP_STATE = 0 fabric and starve.
	 */
	ret = msm8660_rpm_set_bus_rate(qp->rpm, desc->bus_clk_id,
				       MSM8660_FABRIC_PIN_RATE);
	if (ret)
		return dev_err_probe(dev, ret,
				     "RPM bus clk %u pin vote failed\n",
				     desc->bus_clk_id);
	if (desc->extra_clk_id) {
		ret = msm8660_rpm_set_bus_rate(qp->rpm, desc->extra_clk_id,
					       MSM8660_FABRIC_PIN_RATE);
		if (ret)
			return dev_err_probe(dev, ret,
					     "RPM bus clk %u pin vote failed\n",
					     desc->extra_clk_id);
	}

	if (desc->arb_resource >= 0) {
		int arb_size = desc->nmasters * desc->ntieredslaves;

		qp->bwsum = devm_kcalloc(dev, desc->nslaves,
					 sizeof(u16), GFP_KERNEL);
		qp->arb = devm_kcalloc(dev, arb_size,
				       sizeof(u16), GFP_KERNEL);
		qp->rpm_buf = devm_kcalloc(dev, desc->rpm_buf_size,
					   sizeof(u32), GFP_KERNEL);
		if (!qp->bwsum || !qp->arb || !qp->rpm_buf)
			return -ENOMEM;

		/*
		 * One-shot sleep-context vote of zero bandwidth.  Without
		 * an explicit SLEEP_STATE write, RPM has no fabric
		 * bandwidth target for deep-sleep and may keep the active
		 * vote applied indefinitely, preventing DDR from dropping
		 * its rate when CPUs power-collapse.  The buffer is
		 * devm_kcalloc'd so it is all-zero at this point -- written
		 * before any consumer can drive an active vote that would
		 * dirty it.  msm8660_rpm_commit() writes ACTIVE_STATE only;
		 * SLEEP_STATE remains zero for the provider's lifetime, so
		 * this vote does not need refreshing.
		 */
		ret = qcom_rpm_write(qp->rpm, QCOM_RPM_SLEEP_STATE,
				     desc->arb_resource, qp->rpm_buf,
				     desc->rpm_buf_size);
		if (ret)
			dev_warn(dev, "RPM fabric sleep vote failed: %d\n", ret);
	}

	provider = &qp->provider;
	provider->dev = dev;
	provider->set = msm8660_icc_set;
	provider->aggregate = msm8660_icc_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = data;

	icc_provider_init(provider);

	/*
	 * Two-pass init.
	 *
	 * Pass 1 creates and adds every qnode to provider->nodes so that
	 * Pass 2's icc_link_nodes() never has to forward-allocate a target.
	 * If we created + linked in a single pass, a mid-loop probe failure
	 * could leave nodes that icc_link_nodes() allocated for a still-
	 * unprocessed qnode pointed-to only by the static qnodes[].node
	 * cache; the err_remove_nodes path's icc_nodes_remove() would not
	 * see them (they're not in provider->nodes yet), and
	 * msm8660_clear_node_cache() would then drop the only reference
	 * to that allocation -- leaking it.
	 */
	for (i = 0; i < num_nodes; i++) {
		if (!qnodes[i])
			continue;

		if (!qnodes[i]->node)
			qnodes[i]->node = icc_node_create_dyn();
		node = qnodes[i]->node;
		if (IS_ERR(node)) {
			ret = PTR_ERR(node);
			goto err_remove_nodes;
		}

		ret = icc_node_set_name(node, provider, qnodes[i]->name);
		if (ret) {
			icc_node_destroy(node->id);
			qnodes[i]->node = NULL;
			goto err_remove_nodes;
		}

		node->data = qnodes[i];
		icc_node_add(node, provider);
		data->nodes[i] = node;
	}

	/*
	 * Pass 2 links the nodes. Every target node already exists in
	 * provider->nodes from Pass 1, so icc_link_nodes() only ever
	 * extends the source's links[] array and never allocates a node;
	 * any failure here is cleanly caught by err_remove_nodes ->
	 * icc_nodes_remove which iterates provider->nodes.
	 */
	for (i = 0; i < num_nodes; i++) {
		size_t j;

		if (!qnodes[i])
			continue;

		node = qnodes[i]->node;

		for (j = 0; j < qnodes[i]->num_links; j++) {
			ret = icc_link_nodes(node,
					     &qnodes[i]->link_nodes[j]->node);
			if (ret)
				goto err_remove_nodes;
		}
	}

	ret = icc_provider_register(provider);
	if (ret)
		goto err_remove_nodes;

	platform_set_drvdata(pdev, qp);

	dev_info(dev, "MSM8660 interconnect provider registered\n");

	return 0;

err_remove_nodes:
	icc_nodes_remove(provider);
	msm8660_clear_node_cache(qnodes, num_nodes);
	return ret;
}

static void msm8660_icc_remove(struct platform_device *pdev)
{
	struct msm8660_icc_provider *qp = platform_get_drvdata(pdev);
	const struct msm8660_icc_desc *desc = of_device_get_match_data(&pdev->dev);

	icc_provider_deregister(&qp->provider);
	icc_nodes_remove(&qp->provider);
	if (desc)
		msm8660_clear_node_cache(desc->nodes, desc->num_nodes);
	/* clk cleanup happens via devm_add_action_or_reset on remove. */
}

static const struct of_device_id msm8660_noc_of_match[] = {
	{ .compatible = "qcom,msm8660-apps-fabric", .data = &msm8660_afab },
	{ .compatible = "qcom,msm8660-system-fabric", .data = &msm8660_sfab },
	{ .compatible = "qcom,msm8660-mmss-fabric", .data = &msm8660_mmfab },
	{ .compatible = "qcom,msm8660-daytona-fabric", .data = &msm8660_dfab },
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
		/*
		 * The four fabrics register as independent platform devices
		 * but link to each other via raw struct icc_node * pointers.
		 * Allowing individual sysfs unbind would let a target fabric
		 * free its nodes while a still-bound source fabric still holds
		 * pointers to them, dereferenced during path finding ->
		 * use-after-free. Suppress the sysfs bind/unbind attrs so the
		 * only way to unload is module removal, which unbinds all
		 * fabrics together.
		 */
		.suppress_bind_attrs = true,
	},
};

/*
 * Register the NOC provider at core_initcall, matching the mainline pattern
 * used by newer Qualcomm SoCs (sm8450, glymur, qdu1000, sc8280xp, sm8750).
 *
 * Why not module_platform_driver (device_initcall)?  drivers/Makefile lists
 * drivers/interconnect/ at position 189, *after* every ICC-consumer subdir
 * (clk/ @40, soc/ @46, gpu/ @68, base/mfd/ @76, spmi/ @89, usb/ @106,
 * i2c/ @116, mmc/ @133, remoteproc/ @158).  Within a single initcall level
 * execution order = link order, so a device_initcall registration here runs
 * *after* every consumer has already tried to probe.  Mainline relies on
 * deferred-probe retry to recover from that, but in this tree some consumer
 * (apcs-msm8660 + cpufreq cascade suspected) fails to recover within
 * deferred_probe_timeout=5 and boot dies at the Tux splash with no rootfs.
 * Empirically confirmed 2026-05-29 with module_platform_driver (commits
 * 99275d8a8ae9 + ca35c591854c, reverted).
 *
 * icc_provider_register does not require icc_init to have run first --
 * the framework's locks are statically DEFINE_MUTEX'd -- so registering
 * the provider at core_initcall (before icc_init at subsys_initcall) is
 * safe, same as mainline sm8450 etc.
 */
static int __init msm8660_noc_driver_init(void)
{
	return platform_driver_register(&msm8660_noc_driver);
}
core_initcall(msm8660_noc_driver_init);

/*
 * No module_exit: Kconfig is bool, the driver is built-in only, and
 * unbind/unload paths are not exercised. core_initcall + module_exit
 * mix badly anyway (you cannot unload something registered earlier
 * than module_init level).
 */

MODULE_DESCRIPTION("Qualcomm MSM8x60 interconnect driver");
MODULE_LICENSE("GPL");
