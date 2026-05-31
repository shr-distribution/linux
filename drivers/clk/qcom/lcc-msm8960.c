// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,lcc-msm8960.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "clk-regmap-divider.h"
#include "clk-regmap-mux.h"

static struct clk_parent_data pxo_parent_data = {
	.fw_name = "pxo", .name = "pxo_board",
};

static struct clk_pll pll4 = {
	.l_reg = 0x4,
	.m_reg = 0x8,
	.n_reg = 0xc,
	.config_reg = 0x14,
	.mode_reg = 0x0,
	.status_reg = 0x18,
	.status_bit = 16,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pll4",
		.parent_data = &pxo_parent_data,
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

enum {
	P_PXO,
	P_PLL4,
};

static const struct parent_map lcc_pxo_pll4_map[] = {
	{ P_PXO, 0 },
	{ P_PLL4, 2 }
};

static struct clk_parent_data lcc_pxo_pll4[] = {
	{ .fw_name = "pxo", .name = "pxo_board" },
	{ .fw_name = "pll4_vote", .name = "pll4_vote" },
};

static const struct freq_tbl clk_tbl_aif_osr_492[] = {
	{   512000, P_PLL4, 4, 1, 240 },
	{   768000, P_PLL4, 4, 1, 160 },
	{  1024000, P_PLL4, 4, 1, 120 },
	{  1536000, P_PLL4, 4, 1,  80 },
	{  2048000, P_PLL4, 4, 1,  60 },
	{  3072000, P_PLL4, 4, 1,  40 },
	{  4096000, P_PLL4, 4, 1,  30 },
	{  6144000, P_PLL4, 4, 1,  20 },
	{  8192000, P_PLL4, 4, 1,  15 },
	{ 12288000, P_PLL4, 4, 1,  10 },
	{ 24576000, P_PLL4, 4, 1,   5 },
	{ 27000000, P_PXO,  1, 0,   0 },
	{ }
};

static const struct freq_tbl clk_tbl_aif_osr_393[] = {
	{   512000, P_PLL4, 4, 1, 192 },
	{   768000, P_PLL4, 4, 1, 128 },
	{  1024000, P_PLL4, 4, 1,  96 },
	{  1536000, P_PLL4, 4, 1,  64 },
	{  2048000, P_PLL4, 4, 1,  48 },
	{  3072000, P_PLL4, 4, 1,  32 },
	{  4096000, P_PLL4, 4, 1,  24 },
	{  6144000, P_PLL4, 4, 1,  16 },
	{  8192000, P_PLL4, 4, 1,  12 },
	{ 12288000, P_PLL4, 4, 1,   8 },
	{ 24576000, P_PLL4, 4, 1,   4 },
	{ 27000000, P_PXO,  1, 0,   0 },
	{ }
};

/*
 * MSM8x60 family (MSM8260/MSM8660/APQ8060) PLL4 runs at 540.672 MHz
 * (24.576 MHz * 22, L=0x16). Divisors are taken from the legacy webOS
 * clock-8x60.c driver. AIF_OSR has an 8-bit M/N counter, so 512000 Hz
 * is not representable with this PLL frequency and is omitted.
 */
static const struct freq_tbl clk_tbl_aif_osr_540[] = {
	{   768000, P_PLL4, 4, 1, 176 },
	{  1024000, P_PLL4, 4, 1, 132 },
	{  1536000, P_PLL4, 4, 1,  88 },
	{  2048000, P_PLL4, 4, 1,  66 },
	{  3072000, P_PLL4, 4, 1,  44 },
	{  4096000, P_PLL4, 4, 1,  33 },
	{  6144000, P_PLL4, 4, 1,  22 },
	{  8192000, P_PLL4, 2, 1,  33 },
	{ 12288000, P_PLL4, 4, 1,  11 },
	{ 24576000, P_PLL4, 2, 1,  11 },
	{ 27000000, P_PXO,  1, 0,   0 },
	{ }
};

#define CLK_AIF_OSR_SRC(prefix, _ns, _md)			\
static struct clk_rcg prefix##_osr_src = {			\
	.ns_reg = _ns,						\
	.md_reg = _md,						\
	.mn = {							\
		.mnctr_en_bit = 8,				\
		.mnctr_reset_bit = 7,				\
		.mnctr_mode_shift = 5,				\
		.n_val_shift = 24,				\
		.m_val_shift = 8,				\
		.width = 8,					\
	},							\
	.p = {							\
		.pre_div_shift = 3,				\
		.pre_div_width = 2,				\
	},							\
	.s = {							\
		.src_sel_shift = 0,				\
		.parent_map = lcc_pxo_pll4_map,			\
	},							\
	.freq_tbl = clk_tbl_aif_osr_393,			\
	.clkr = {						\
		.enable_reg = _ns,				\
		.enable_mask = BIT(9),				\
		.hw.init = &(struct clk_init_data){		\
			.name = #prefix "_osr_src",		\
			.parent_data = lcc_pxo_pll4,		\
			.num_parents = ARRAY_SIZE(lcc_pxo_pll4), \
			.ops = &clk_rcg_ops,			\
			.flags = CLK_SET_RATE_GATE,		\
		},						\
	},							\
};								\

#define CLK_AIF_OSR_CLK(prefix, _ns, hr, en_bit)		\
static struct clk_branch prefix##_osr_clk = {			\
	.halt_reg = hr,						\
	.halt_bit = 1,						\
	.halt_check = BRANCH_HALT_ENABLE,			\
	.clkr = {						\
		.enable_reg = _ns,				\
		.enable_mask = BIT(en_bit),			\
		.hw.init = &(struct clk_init_data){		\
			.name = #prefix "_osr_clk",		\
			.parent_hws = (const struct clk_hw*[]){	\
				&prefix##_osr_src.clkr.hw,	\
			},					\
			.num_parents = 1,			\
			.ops = &clk_branch_ops,			\
			.flags = CLK_SET_RATE_PARENT,		\
		},						\
	},							\
};								\

#define CLK_AIF_OSR_DIV_CLK(prefix, _ns, _width)		\
static struct clk_regmap_div prefix##_div_clk = {		\
	.reg = _ns,						\
	.shift = 10,						\
	.width = _width,					\
	.clkr = {						\
		.hw.init = &(struct clk_init_data){		\
			.name = #prefix "_div_clk",		\
			.parent_hws = (const struct clk_hw*[]){	\
				&prefix##_osr_src.clkr.hw,	\
			},					\
			.num_parents = 1,			\
			.ops = &clk_regmap_div_ops,		\
		},						\
	},							\
};								\

#define CLK_AIF_OSR_BIT_DIV_CLK(prefix, _ns, hr, en_bit)	\
static struct clk_branch prefix##_bit_div_clk = {		\
	.halt_reg = hr,						\
	.halt_bit = 0,						\
	.halt_check = BRANCH_HALT_ENABLE,			\
	.clkr = {						\
		.enable_reg = _ns,				\
		.enable_mask = BIT(en_bit),			\
		.hw.init = &(struct clk_init_data){		\
			.name = #prefix "_bit_div_clk",		\
			.parent_hws = (const struct clk_hw*[]){	\
				&prefix##_div_clk.clkr.hw,	\
			},					\
			.num_parents = 1,			\
			.ops = &clk_branch_ops,			\
			.flags = CLK_SET_RATE_PARENT,		\
		},						\
	},							\
};								\

#define CLK_AIF_OSR_BIT_CLK(prefix, _ns, _shift)		\
static struct clk_regmap_mux prefix##_bit_clk = {		\
	.reg = _ns,						\
	.shift = _shift,					\
	.width = 1,						\
	.clkr = {						\
		.hw.init = &(struct clk_init_data){		\
			.name = #prefix "_bit_clk",		\
			.parent_data = (const struct clk_parent_data[]){ \
				{ .hw = &prefix##_bit_div_clk.clkr.hw, }, \
				{ .fw_name = #prefix "_codec_clk", \
				  .name = #prefix "_codec_clk", }, \
			},					\
			.num_parents = 2,			\
			.ops = &clk_regmap_mux_closest_ops,	\
			.flags = CLK_SET_RATE_PARENT,		\
		},						\
	},							\
};

CLK_AIF_OSR_SRC(mi2s, 0x48, 0x4c)
CLK_AIF_OSR_CLK(mi2s, 0x48, 0x50, 17)
CLK_AIF_OSR_DIV_CLK(mi2s, 0x48, 4)
CLK_AIF_OSR_BIT_DIV_CLK(mi2s, 0x48, 0x50, 15)
CLK_AIF_OSR_BIT_CLK(mi2s, 0x48, 14)

#define CLK_AIF_OSR_DIV(prefix, _ns, _md, hr)			\
	CLK_AIF_OSR_SRC(prefix, _ns, _md)			\
	CLK_AIF_OSR_CLK(prefix, _ns, hr, 21)			\
	CLK_AIF_OSR_DIV_CLK(prefix, _ns, 8)			\
	CLK_AIF_OSR_BIT_DIV_CLK(prefix, _ns, hr, 19)		\
	CLK_AIF_OSR_BIT_CLK(prefix, _ns, 18)

CLK_AIF_OSR_DIV(codec_i2s_mic, 0x60, 0x64, 0x68);
CLK_AIF_OSR_DIV(spare_i2s_mic, 0x78, 0x7c, 0x80);
CLK_AIF_OSR_DIV(codec_i2s_spkr, 0x6c, 0x70, 0x74);
CLK_AIF_OSR_DIV(spare_i2s_spkr, 0x84, 0x88, 0x8c);

static const struct freq_tbl clk_tbl_pcm_492[] = {
	{   256000, P_PLL4, 4, 1, 480 },
	{   512000, P_PLL4, 4, 1, 240 },
	{   768000, P_PLL4, 4, 1, 160 },
	{  1024000, P_PLL4, 4, 1, 120 },
	{  1536000, P_PLL4, 4, 1,  80 },
	{  2048000, P_PLL4, 4, 1,  60 },
	{  3072000, P_PLL4, 4, 1,  40 },
	{  4096000, P_PLL4, 4, 1,  30 },
	{  6144000, P_PLL4, 4, 1,  20 },
	{  8192000, P_PLL4, 4, 1,  15 },
	{ 12288000, P_PLL4, 4, 1,  10 },
	{ 24576000, P_PLL4, 4, 1,   5 },
	{ 27000000, P_PXO,  1, 0,   0 },
	{ }
};

static const struct freq_tbl clk_tbl_pcm_393[] = {
	{   256000, P_PLL4, 4, 1, 384 },
	{   512000, P_PLL4, 4, 1, 192 },
	{   768000, P_PLL4, 4, 1, 128 },
	{  1024000, P_PLL4, 4, 1,  96 },
	{  1536000, P_PLL4, 4, 1,  64 },
	{  2048000, P_PLL4, 4, 1,  48 },
	{  3072000, P_PLL4, 4, 1,  32 },
	{  4096000, P_PLL4, 4, 1,  24 },
	{  6144000, P_PLL4, 4, 1,  16 },
	{  8192000, P_PLL4, 4, 1,  12 },
	{ 12288000, P_PLL4, 4, 1,   8 },
	{ 24576000, P_PLL4, 4, 1,   4 },
	{ 27000000, P_PXO,  1, 0,   0 },
	{ }
};

static const struct freq_tbl clk_tbl_pcm_540[] = {
	{   256000, P_PLL4, 4, 1, 528 },
	{   512000, P_PLL4, 4, 1, 264 },
	{   768000, P_PLL4, 4, 1, 176 },
	{  1024000, P_PLL4, 4, 1, 132 },
	{  1536000, P_PLL4, 4, 1,  88 },
	{  2048000, P_PLL4, 4, 1,  66 },
	{  3072000, P_PLL4, 4, 1,  44 },
	{  4096000, P_PLL4, 4, 1,  33 },
	{  6144000, P_PLL4, 4, 1,  22 },
	{  8192000, P_PLL4, 2, 1,  33 },
	{ 12288000, P_PLL4, 4, 1,  11 },
	{ 24576000, P_PLL4, 2, 1,  11 },
	{ 27000000, P_PXO,  1, 0,   0 },
	{ }
};

static struct clk_rcg pcm_src = {
	.ns_reg = 0x54,
	.md_reg = 0x58,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 16,
		.m_val_shift = 16,
		.width = 16,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = lcc_pxo_pll4_map,
	},
	.freq_tbl = clk_tbl_pcm_393,
	.clkr = {
		.enable_reg = 0x54,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "pcm_src",
			.parent_data = lcc_pxo_pll4,
			.num_parents = ARRAY_SIZE(lcc_pxo_pll4),
			.ops = &clk_rcg_ops,
			.flags = CLK_SET_RATE_GATE,
		},
	},
};

static struct clk_branch pcm_clk_out = {
	.halt_reg = 0x5c,
	.halt_bit = 0,
	.halt_check = BRANCH_HALT_ENABLE,
	.clkr = {
		.enable_reg = 0x54,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "pcm_clk_out",
			.parent_hws = (const struct clk_hw*[]){
				&pcm_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_regmap_mux pcm_clk = {
	.reg = 0x54,
	.shift = 10,
	.width = 1,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "pcm_clk",
			.parent_data = (const struct clk_parent_data[]){
				{ .hw = &pcm_clk_out.clkr.hw },
				{ .fw_name = "pcm_codec_clk", .name = "pcm_codec_clk" },
			},
			.num_parents = 2,
			.ops = &clk_regmap_mux_closest_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_rcg slimbus_src = {
	.ns_reg = 0xcc,
	.md_reg = 0xd0,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = lcc_pxo_pll4_map,
	},
	.freq_tbl = clk_tbl_aif_osr_393,
	.clkr = {
		.enable_reg = 0xcc,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "slimbus_src",
			.parent_data = lcc_pxo_pll4,
			.num_parents = ARRAY_SIZE(lcc_pxo_pll4),
			.ops = &clk_rcg_ops,
			.flags = CLK_SET_RATE_GATE,
		},
	},
};

static struct clk_branch audio_slimbus_clk = {
	.halt_reg = 0xd4,
	.halt_bit = 0,
	.halt_check = BRANCH_HALT_ENABLE,
	.clkr = {
		.enable_reg = 0xcc,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "audio_slimbus_clk",
			.parent_hws = (const struct clk_hw*[]){
				&slimbus_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch sps_slimbus_clk = {
	.halt_reg = 0xd4,
	.halt_bit = 1,
	.halt_check = BRANCH_HALT_ENABLE,
	.clkr = {
		.enable_reg = 0xcc,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.name = "sps_slimbus_clk",
			.parent_hws = (const struct clk_hw*[]){
				&slimbus_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_regmap *lcc_msm8960_clks[] = {
	[PLL4] = &pll4.clkr,
	[MI2S_OSR_SRC] = &mi2s_osr_src.clkr,
	[MI2S_OSR_CLK] = &mi2s_osr_clk.clkr,
	[MI2S_DIV_CLK] = &mi2s_div_clk.clkr,
	[MI2S_BIT_DIV_CLK] = &mi2s_bit_div_clk.clkr,
	[MI2S_BIT_CLK] = &mi2s_bit_clk.clkr,
	[PCM_SRC] = &pcm_src.clkr,
	[PCM_CLK_OUT] = &pcm_clk_out.clkr,
	[PCM_CLK] = &pcm_clk.clkr,
	[SLIMBUS_SRC] = &slimbus_src.clkr,
	[AUDIO_SLIMBUS_CLK] = &audio_slimbus_clk.clkr,
	[SPS_SLIMBUS_CLK] = &sps_slimbus_clk.clkr,
	[CODEC_I2S_MIC_OSR_SRC] = &codec_i2s_mic_osr_src.clkr,
	[CODEC_I2S_MIC_OSR_CLK] = &codec_i2s_mic_osr_clk.clkr,
	[CODEC_I2S_MIC_DIV_CLK] = &codec_i2s_mic_div_clk.clkr,
	[CODEC_I2S_MIC_BIT_DIV_CLK] = &codec_i2s_mic_bit_div_clk.clkr,
	[CODEC_I2S_MIC_BIT_CLK] = &codec_i2s_mic_bit_clk.clkr,
	[SPARE_I2S_MIC_OSR_SRC] = &spare_i2s_mic_osr_src.clkr,
	[SPARE_I2S_MIC_OSR_CLK] = &spare_i2s_mic_osr_clk.clkr,
	[SPARE_I2S_MIC_DIV_CLK] = &spare_i2s_mic_div_clk.clkr,
	[SPARE_I2S_MIC_BIT_DIV_CLK] = &spare_i2s_mic_bit_div_clk.clkr,
	[SPARE_I2S_MIC_BIT_CLK] = &spare_i2s_mic_bit_clk.clkr,
	[CODEC_I2S_SPKR_OSR_SRC] = &codec_i2s_spkr_osr_src.clkr,
	[CODEC_I2S_SPKR_OSR_CLK] = &codec_i2s_spkr_osr_clk.clkr,
	[CODEC_I2S_SPKR_DIV_CLK] = &codec_i2s_spkr_div_clk.clkr,
	[CODEC_I2S_SPKR_BIT_DIV_CLK] = &codec_i2s_spkr_bit_div_clk.clkr,
	[CODEC_I2S_SPKR_BIT_CLK] = &codec_i2s_spkr_bit_clk.clkr,
	[SPARE_I2S_SPKR_OSR_SRC] = &spare_i2s_spkr_osr_src.clkr,
	[SPARE_I2S_SPKR_OSR_CLK] = &spare_i2s_spkr_osr_clk.clkr,
	[SPARE_I2S_SPKR_DIV_CLK] = &spare_i2s_spkr_div_clk.clkr,
	[SPARE_I2S_SPKR_BIT_DIV_CLK] = &spare_i2s_spkr_bit_div_clk.clkr,
	[SPARE_I2S_SPKR_BIT_CLK] = &spare_i2s_spkr_bit_clk.clkr,
};

static const struct regmap_config lcc_msm8960_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0xfc,
	.fast_io	= true,
};

static const struct qcom_cc_desc lcc_msm8960_desc = {
	.config = &lcc_msm8960_regmap_config,
	.clks = lcc_msm8960_clks,
	.num_clks = ARRAY_SIZE(lcc_msm8960_clks),
};

/*
 * Per-compatible quirks for SoC families that share the LCC IP block
 * but require behaviour that diverges from the MSM8960 default.
 *
 * The MSM8x60 family (MSM8260/MSM8660/APQ8060) needs:
 *
 *   - Singleton enforcement, because the file-static clk_rcg structs
 *     above are mutated by the freq-plan selection below and a second
 *     probe of the same driver on the same SoC would race the assignment
 *     and corrupt each other's freq_tbl pointers.
 *
 *   - Resume re-application of the LPASS Primary PLL mux: on this
 *     family the LPASS power domain is collapsed during system sleep,
 *     which resets register 0xc4 to its default of 0 (PXO). The audio
 *     clock tree silently produces wrong rates until the next reboot
 *     unless the mux is re-armed.
 *
 * .pll4_quirks gates both behaviours. The MSM8960/APQ8064/MDM9615
 * entries set it to false and the driver's behaviour for them is
 * unchanged.
 */
struct lcc_msm8960_quirks {
	bool pll4_quirks;
};

static const struct lcc_msm8960_quirks lcc_msm8x60_quirks = {
	.pll4_quirks = true,
};

static const struct of_device_id lcc_msm8960_match_table[] = {
	{ .compatible = "qcom,lcc-msm8260", .data = &lcc_msm8x60_quirks },
	{ .compatible = "qcom,lcc-msm8660", .data = &lcc_msm8x60_quirks },
	{ .compatible = "qcom,lcc-apq8060", .data = &lcc_msm8x60_quirks },
	{ .compatible = "qcom,lcc-msm8960" },
	{ .compatible = "qcom,lcc-apq8064" },
	{ .compatible = "qcom,lcc-mdm9615" },
	{ }
};
MODULE_DEVICE_TABLE(of, lcc_msm8960_match_table);

/*
 * Single-instance bind tracking for the MSM8x60 family. The probe path
 * mutates the file-static clk_rcg .freq_tbl pointers based on the
 * detected PLL4 plan; a second concurrent probe would race that
 * assignment. The bound flag is per-compatible-family rather than
 * per-driver-instance because the bug it guards against is the shared
 * file-static state, not anything tied to the driver struct.
 */
static bool lcc_msm8x60_bound;
static DEFINE_MUTEX(lcc_msm8x60_bound_lock);

static int lcc_msm8960_probe(struct platform_device *pdev)
{
	const struct lcc_msm8960_quirks *quirks;
	u32 val;
	int ret;
	struct regmap *regmap;

	quirks = of_device_get_match_data(&pdev->dev);

	if (quirks && quirks->pll4_quirks) {
		guard(mutex)(&lcc_msm8x60_bound_lock);
		if (lcc_msm8x60_bound)
			return dev_err_probe(&pdev->dev, -EBUSY,
				"only a single LCC instance is supported\n");
	}

	/* patch for the cxo <-> pxo difference */
	if (of_device_is_compatible(pdev->dev.of_node, "qcom,lcc-mdm9615")) {
		pxo_parent_data.fw_name = "cxo";
		pxo_parent_data.name = "cxo_board";
		lcc_pxo_pll4[0].fw_name = "cxo";
		lcc_pxo_pll4[0].name = "cxo_board";
	}

	regmap = qcom_cc_map(pdev, &lcc_msm8960_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Use the correct frequency plan depending on speed of PLL4 */
	regmap_read(regmap, 0x4, &val);
	if (val == 0x12) {
		slimbus_src.freq_tbl = clk_tbl_aif_osr_492;
		mi2s_osr_src.freq_tbl = clk_tbl_aif_osr_492;
		codec_i2s_mic_osr_src.freq_tbl = clk_tbl_aif_osr_492;
		spare_i2s_mic_osr_src.freq_tbl = clk_tbl_aif_osr_492;
		codec_i2s_spkr_osr_src.freq_tbl = clk_tbl_aif_osr_492;
		spare_i2s_spkr_osr_src.freq_tbl = clk_tbl_aif_osr_492;
		pcm_src.freq_tbl = clk_tbl_pcm_492;
	} else if (val == 0x16) {
		slimbus_src.freq_tbl = clk_tbl_aif_osr_540;
		mi2s_osr_src.freq_tbl = clk_tbl_aif_osr_540;
		codec_i2s_mic_osr_src.freq_tbl = clk_tbl_aif_osr_540;
		spare_i2s_mic_osr_src.freq_tbl = clk_tbl_aif_osr_540;
		codec_i2s_spkr_osr_src.freq_tbl = clk_tbl_aif_osr_540;
		spare_i2s_spkr_osr_src.freq_tbl = clk_tbl_aif_osr_540;
		pcm_src.freq_tbl = clk_tbl_pcm_540;
	}
	/* Enable PLL4 source on the LPASS Primary PLL Mux */
	regmap_write(regmap, 0xc4, 0x1);

	ret = qcom_cc_really_probe(&pdev->dev, &lcc_msm8960_desc, regmap);
	if (ret)
		return ret;

	if (quirks && quirks->pll4_quirks) {
		/* Stash regmap for the resume path's mux re-application. */
		platform_set_drvdata(pdev, regmap);
		scoped_guard(mutex, &lcc_msm8x60_bound_lock)
			lcc_msm8x60_bound = true;
	}
	return 0;
}

/*
 * Resume re-applies the LPASS Primary PLL mux selection because on
 * MSM8x60 the LPASS power domain collapses during system sleep and
 * register 0xc4 is reset to its hardware default of 0 (PXO). 8960 and
 * later SoCs do not exhibit this behaviour and do not set drvdata, so
 * the helper is a no-op for them.
 */
static int lcc_msm8960_resume(struct device *dev)
{
	struct regmap *regmap = dev_get_drvdata(dev);

	if (!regmap)
		return 0;

	return regmap_write(regmap, 0xc4, 0x1);
}

static DEFINE_SIMPLE_DEV_PM_OPS(lcc_msm8960_pm_ops, NULL, lcc_msm8960_resume);

static struct platform_driver lcc_msm8960_driver = {
	.probe		= lcc_msm8960_probe,
	.driver		= {
		.name	= "lcc-msm8960",
		.of_match_table = lcc_msm8960_match_table,
		.pm	= pm_sleep_ptr(&lcc_msm8960_pm_ops),
	},
};
module_platform_driver(lcc_msm8960_driver);

MODULE_DESCRIPTION("QCOM LCC MSM8960 Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:lcc-msm8960");
