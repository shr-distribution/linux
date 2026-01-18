// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2026, LuneOS Project. MSM8660/APQ8060 specific clocks.
 *
 * MSM8660/APQ8060 Multimedia Clock Controller driver
 *
 * Split from mmcc-msm8960.c to properly handle MSM8660-specific clock
 * configurations, particularly the GFX3D reset bits which differ from MSM8960.
 */

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,mmcc-msm8960.h>
#include <dt-bindings/reset/qcom,mmcc-msm8960.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "reset.h"

enum {
	P_PXO,
	P_PLL8,
	P_PLL2,
	P_PLL3,
	P_HDMI_PLL,
	P_DSI1_PLL_DSICLK,
	P_DSI2_PLL_DSICLK,
	P_DSI1_PLL_BYTECLK,
	P_DSI2_PLL_BYTECLK,
};

#define F_MN(f, s, _m, _n) { .freq = f, .src = s, .m = _m, .n = _n }

static struct clk_pll pll2 = {
	.l_reg = 0x320,
	.m_reg = 0x324,
	.n_reg = 0x328,
	.config_reg = 0x32c,
	.mode_reg = 0x31c,
	.status_reg = 0x334,
	.status_bit = 16,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pll2",
		.parent_data = (const struct clk_parent_data[]){
			{ .fw_name = "pxo", .name = "pxo_board" },
		},
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static const struct parent_map mmcc_pxo_pll8_pll2_map[] = {
	{ P_PXO, 0 },
	{ P_PLL8, 2 },
	{ P_PLL2, 1 }
};

static const struct clk_parent_data mmcc_pxo_pll8_pll2[] = {
	{ .fw_name = "pxo", .name = "pxo_board" },
	{ .fw_name = "pll8_vote", .name = "pll8_vote" },
	{ .hw = &pll2.clkr.hw },
};

static const struct parent_map mmcc_pxo_pll8_pll2_pll3_map[] = {
	{ P_PXO, 0 },
	{ P_PLL8, 2 },
	{ P_PLL2, 1 },
	{ P_PLL3, 3 }
};

static const struct clk_parent_data mmcc_pxo_pll8_pll2_pll3[] = {
	{ .fw_name = "pxo", .name = "pxo_board" },
	{ .fw_name = "pll8_vote", .name = "pll8_vote" },
	{ .hw = &pll2.clkr.hw },
	{ .fw_name = "pll3", .name = "pll3" },
};

static const struct parent_map mmcc_pxo_dsi2_dsi1_map[] = {
	{ P_PXO, 0 },
	{ P_DSI2_PLL_DSICLK, 1 },
	{ P_DSI1_PLL_DSICLK, 3 },
};

static const struct clk_parent_data mmcc_pxo_dsi2_dsi1[] = {
	{ .fw_name = "pxo", .name = "pxo_board" },
	{ .fw_name = "dsi2pll", .name = "dsi2pll" },
	{ .fw_name = "dsi1pll", .name = "dsi1pll" },
};

static const struct parent_map mmcc_pxo_dsi1_dsi2_byte_map[] = {
	{ P_PXO, 0 },
	{ P_DSI1_PLL_BYTECLK, 1 },
	{ P_DSI2_PLL_BYTECLK, 2 },
};

static const struct clk_parent_data mmcc_pxo_dsi1_dsi2_byte[] = {
	{ .fw_name = "pxo", .name = "pxo_board" },
	{ .fw_name = "dsi1pllbyte", .name = "dsi1pllbyte" },
	{ .fw_name = "dsi2pllbyte", .name = "dsi2pllbyte" },
};

static const struct freq_tbl clk_tbl_cam[] = {
	{   6000000, P_PLL8, 4, 1, 16 },
	{   8000000, P_PLL8, 4, 1, 12 },
	{  12000000, P_PLL8, 4, 1,  8 },
	{  16000000, P_PLL8, 4, 1,  6 },
	{  19200000, P_PLL8, 4, 1,  5 },
	{  24000000, P_PLL8, 4, 1,  4 },
	{  32000000, P_PLL8, 4, 1,  3 },
	{  48000000, P_PLL8, 4, 1,  2 },
	{  64000000, P_PLL8, 3, 1,  2 },
	{  96000000, P_PLL8, 4, 0,  0 },
	{ 128000000, P_PLL8, 3, 0,  0 },
	{ }
};

static struct clk_rcg camclk0_src = {
	.ns_reg = 0x0148,
	.md_reg = 0x0144,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 8,
		.reset_in_cc = true,
		.mnctr_mode_shift = 6,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_cam,
	.clkr = {
		.enable_reg = 0x0140,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "camclk0_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch camclk0_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 15,
	.clkr = {
		.enable_reg = 0x0140,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "camclk0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camclk0_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_rcg camclk1_src = {
	.ns_reg = 0x015c,
	.md_reg = 0x0158,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 8,
		.reset_in_cc = true,
		.mnctr_mode_shift = 6,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_cam,
	.clkr = {
		.enable_reg = 0x0154,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "camclk1_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch camclk1_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 16,
	.clkr = {
		.enable_reg = 0x0154,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "camclk1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camclk1_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static const struct freq_tbl clk_tbl_csi[] = {
	{  27000000, P_PXO,  1, 0, 0 },
	{  85330000, P_PLL8, 1, 2, 9 },
	{ 177780000, P_PLL2, 1, 2, 9 },
	{ }
};

static struct clk_rcg csi0_src = {
	.ns_reg = 0x0048,
	.md_reg = 0x0044,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_csi,
	.clkr = {
		.enable_reg = 0x0040,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "csi0_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch csi0_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 13,
	.clkr = {
		.enable_reg = 0x0040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csi0_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csi0_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch csi0_phy_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 9,
	.clkr = {
		.enable_reg = 0x0040,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csi0_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csi0_phy_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_rcg csi1_src = {
	.ns_reg = 0x0010,
	.md_reg = 0x0028,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_csi,
	.clkr = {
		.enable_reg = 0x0024,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "csi1_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch csi1_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 14,
	.clkr = {
		.enable_reg = 0x0024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csi1_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csi1_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch csi1_phy_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 10,
	.clkr = {
		.enable_reg = 0x0024,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csi1_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csi1_phy_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch csi_pix_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 3,
	.clkr = {
		.enable_reg = 0x0058,
		.enable_mask = BIT(26),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csi0_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csi_pix_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch csi_rdi_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 2,
	.clkr = {
		.enable_reg = 0x0058,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csi0_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csi_rdi_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_csiphytimer[] = {
	{  85330000, P_PLL8, 1, 2, 9 },
	{ 177780000, P_PLL2, 1, 2, 9 },
	{ }
};

static struct clk_rcg csiphytimer_src = {
	.ns_reg = 0x0168,
	.md_reg = 0x0164,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 8,
		.reset_in_cc = true,
		.mnctr_mode_shift = 6,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_csiphytimer,
	.clkr = {
		.enable_reg = 0x0160,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "csiphytimer_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch csiphy0_timer_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 17,
	.clkr = {
		.enable_reg = 0x0160,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csiphytimer_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csiphy0_timer_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch csiphy1_timer_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 18,
	.clkr = {
		.enable_reg = 0x0160,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&csiphytimer_src.clkr.hw
			},
			.num_parents = 1,
			.name = "csiphy1_timer_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_dsi[] = {
	{ }
};

static struct clk_rcg dsi1_src = {
	.ns_reg = 0x0054,
	.md_reg = 0x0050,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 24,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_dsi2_dsi1_map,
	},
	.freq_tbl = clk_tbl_dsi,
	.clkr = {
		.enable_reg = 0x004c,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_src",
			.parent_data = mmcc_pxo_dsi2_dsi1,
			.num_parents = ARRAY_SIZE(mmcc_pxo_dsi2_dsi1),
			.ops = &clk_rcg_bypass_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch dsi1_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 2,
	.clkr = {
		.enable_reg = 0x004c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&dsi1_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_rcg dsi1_byte_src = {
	.ns_reg = 0x00b0,
	.p = {
		.pre_div_shift = 12,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_dsi1_dsi2_byte_map,
	},
	.clkr = {
		.enable_reg = 0x0090,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_byte_src",
			.parent_data = mmcc_pxo_dsi1_dsi2_byte,
			.num_parents = ARRAY_SIZE(mmcc_pxo_dsi1_dsi2_byte),
			.ops = &clk_rcg_bypass_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch dsi1_byte_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 21,
	.clkr = {
		.enable_reg = 0x0090,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_byte_clk",
			.parent_hws = (const struct clk_hw*[]){
				&dsi1_byte_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_rcg dsi1_esc_src = {
	.ns_reg = 0x00b8,
	.p = {
		.pre_div_shift = 12,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_dsi1_dsi2_byte_map,
	},
	.clkr = {
		.enable_reg = 0x00b4,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_esc_src",
			.parent_data = mmcc_pxo_dsi1_dsi2_byte,
			.num_parents = ARRAY_SIZE(mmcc_pxo_dsi1_dsi2_byte),
			.ops = &clk_rcg_bypass_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch dsi1_esc_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 1,
	.clkr = {
		.enable_reg = 0x00b4,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_esc_clk",
			.parent_hws = (const struct clk_hw*[]){
				&dsi1_esc_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_rcg dsi1_pixel_src = {
	.ns_reg = 0x0138,
	.md_reg = 0x0134,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 12,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_dsi2_dsi1_map,
	},
	.clkr = {
		.enable_reg = 0x0130,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_pixel_src",
			.parent_data = mmcc_pxo_dsi2_dsi1,
			.num_parents = ARRAY_SIZE(mmcc_pxo_dsi2_dsi1),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch dsi1_pixel_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 6,
	.clkr = {
		.enable_reg = 0x0130,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "dsi1_pixel_clk",
			.parent_hws = (const struct clk_hw*[]){
				&dsi1_pixel_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_gfx2d[] = {
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 48000000, P_PLL8, 1,  8),
	F_MN( 54857000, P_PLL8, 1,  7),
	F_MN( 64000000, P_PLL8, 1,  6),
	F_MN( 76800000, P_PLL8, 1,  5),
	F_MN( 96000000, P_PLL8, 1,  4),
	F_MN(128000000, P_PLL8, 1,  3),
	F_MN(145455000, P_PLL2, 2, 11),
	F_MN(160000000, P_PLL2, 1,  5),
	F_MN(177778000, P_PLL2, 2,  9),
	F_MN(200000000, P_PLL2, 1,  4),
	F_MN(228571000, P_PLL2, 2,  7),
	{ }
};

static struct clk_dyn_rcg gfx2d0_src = {
	.ns_reg[0] = 0x0070,
	.ns_reg[1] = 0x0070,
	.md_reg[0] = 0x0064,
	.md_reg[1] = 0x0068,
	.bank_reg = 0x0060,
	.mn[0] = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 25,
		.mnctr_mode_shift = 9,
		.n_val_shift = 20,
		.m_val_shift = 4,
		.width = 4,
	},
	.mn[1] = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 24,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 4,
		.width = 4,
	},
	.s[0] = {
		.src_sel_shift = 3,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.s[1] = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.mux_sel_bit = 11,
	.freq_tbl = clk_tbl_gfx2d,
	.clkr = {
		.enable_reg = 0x0060,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gfx2d0_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_dyn_rcg_ops,
		},
	},
};

static struct clk_branch gfx2d0_clk = {
	.halt_reg = 0x01c8,
	.halt_bit = 9,
	.clkr = {
		.enable_reg = 0x0060,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gfx2d0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gfx2d0_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_dyn_rcg gfx2d1_src = {
	.ns_reg[0] = 0x007c,
	.ns_reg[1] = 0x007c,
	.md_reg[0] = 0x0078,
	.md_reg[1] = 0x006c,
	.bank_reg = 0x0074,
	.mn[0] = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 25,
		.mnctr_mode_shift = 9,
		.n_val_shift = 20,
		.m_val_shift = 4,
		.width = 4,
	},
	.mn[1] = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 24,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 4,
		.width = 4,
	},
	.s[0] = {
		.src_sel_shift = 3,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.s[1] = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.mux_sel_bit = 11,
	.freq_tbl = clk_tbl_gfx2d,
	.clkr = {
		.enable_reg = 0x0074,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gfx2d1_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_dyn_rcg_ops,
		},
	},
};

static struct clk_branch gfx2d1_clk = {
	.halt_reg = 0x01c8,
	.halt_bit = 14,
	.clkr = {
		.enable_reg = 0x0074,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gfx2d1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gfx2d1_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_gfx3d[] = {
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 48000000, P_PLL8, 1,  8),
	F_MN( 54857000, P_PLL8, 1,  7),
	F_MN( 64000000, P_PLL8, 1,  6),
	F_MN( 76800000, P_PLL8, 1,  5),
	F_MN( 96000000, P_PLL8, 1,  4),
	F_MN(128000000, P_PLL8, 1,  3),
	F_MN(145455000, P_PLL2, 2, 11),
	F_MN(160000000, P_PLL2, 1,  5),
	F_MN(177778000, P_PLL2, 2,  9),
	F_MN(200000000, P_PLL2, 1,  4),
	F_MN(228571000, P_PLL2, 2,  7),
	F_MN(266667000, P_PLL2, 1,  3),
	F_MN(320000000, P_PLL2, 2,  5),
	{ }
};

/*
 * MSM8660/APQ8060-specific GFX3D clocks
 *
 * MSM8660 uses different reset bits for the GFX3D banked MND divider:
 *   - Bank 0: mnctr_reset_bit = 23 (MSM8960 uses 25)
 *   - Bank 1: mnctr_reset_bit = 22 (MSM8960 uses 24)
 *
 * This was verified against the webOS 2.6 kernel source (clock-8x60.c).
 */
static struct clk_dyn_rcg gfx3d_src = {
	.ns_reg[0] = 0x008c,
	.ns_reg[1] = 0x008c,
	.md_reg[0] = 0x0084,
	.md_reg[1] = 0x0088,
	.bank_reg = 0x0080,
	.mn[0] = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 23,
		.mnctr_mode_shift = 9,
		.n_val_shift = 18,
		.m_val_shift = 4,
		.width = 4,
	},
	.mn[1] = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 22,
		.mnctr_mode_shift = 6,
		.n_val_shift = 14,
		.m_val_shift = 4,
		.width = 4,
	},
	.s[0] = {
		.src_sel_shift = 3,
		.parent_map = mmcc_pxo_pll8_pll2_pll3_map,
	},
	.s[1] = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_pll3_map,
	},
	.mux_sel_bit = 11,
	.freq_tbl = clk_tbl_gfx3d,
	.clkr = {
		.enable_reg = 0x0080,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gfx3d_src",
			.parent_data = mmcc_pxo_pll8_pll2_pll3,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2_pll3),
			.ops = &clk_dyn_rcg_ops,
		},
	},
};

static struct clk_branch gfx3d_clk = {
	.halt_reg = 0x01c8,
	.halt_bit = 4,
	.clkr = {
		.enable_reg = 0x0080,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gfx3d_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gfx3d_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_ijpeg[] = {
	F_MN( 27000000, P_PXO,  1, 0),
	F_MN( 36570000, P_PLL8, 1, 21),
	F_MN( 54860000, P_PLL8, 1, 7),
	F_MN( 96000000, P_PLL8, 1, 4),
	F_MN(109710000, P_PLL8, 1, 7),
	F_MN(128000000, P_PLL8, 1, 3),
	F_MN(153600000, P_PLL8, 2, 5),
	F_MN(200000000, P_PLL2, 1, 4),
	F_MN(228571000, P_PLL2, 2, 7),
	{ }
};

static struct clk_rcg ijpeg_src = {
	.ns_reg = 0x00a0,
	.md_reg = 0x009c,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 12,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_ijpeg,
	.clkr = {
		.enable_reg = 0x0098,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "ijpeg_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch ijpeg_clk = {
	.halt_reg = 0x01c8,
	.halt_bit = 24,
	.clkr = {
		.enable_reg = 0x0098,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "ijpeg_clk",
			.parent_hws = (const struct clk_hw*[]){
				&ijpeg_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_jpegd[] = {
	F_MN( 64000000, P_PLL8, 1, 6),
	F_MN( 76800000, P_PLL8, 1, 5),
	F_MN( 96000000, P_PLL8, 1, 4),
	F_MN(160000000, P_PLL2, 1, 5),
	F_MN(200000000, P_PLL2, 1, 4),
	{ }
};

static struct clk_rcg jpegd_src = {
	.ns_reg = 0x00ac,
	.p = {
		.pre_div_shift = 12,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_jpegd,
	.clkr = {
		.enable_reg = 0x00a4,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "jpegd_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch jpegd_clk = {
	.halt_reg = 0x01c8,
	.halt_bit = 19,
	.clkr = {
		.enable_reg = 0x00a4,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "jpegd_clk",
			.parent_hws = (const struct clk_hw*[]){
				&jpegd_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_mdp[] = {
	F_MN(  9600000, P_PLL8, 1, 40),
	F_MN( 13710000, P_PLL8, 1, 28),
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 29540000, P_PLL8, 1, 13),
	F_MN( 34290000, P_PLL8, 1, 22),
	F_MN( 38400000, P_PLL8, 1, 10),
	F_MN( 59080000, P_PLL8, 2, 13),
	F_MN( 76800000, P_PLL8, 1,  5),
	F_MN( 85330000, P_PLL8, 2,  9),
	F_MN( 96000000, P_PLL8, 1,  4),
	F_MN(128000000, P_PLL8, 1,  3),
	F_MN(160000000, P_PLL2, 1,  5),
	F_MN(177780000, P_PLL2, 2,  9),
	F_MN(200000000, P_PLL2, 1,  4),
	{ }
};

static struct clk_dyn_rcg mdp_src = {
	.ns_reg[0] = 0x00d0,
	.ns_reg[1] = 0x00d0,
	.md_reg[0] = 0x00c4,
	.md_reg[1] = 0x00c8,
	.bank_reg = 0x00c0,
	.mn[0] = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 31,
		.mnctr_mode_shift = 9,
		.n_val_shift = 22,
		.m_val_shift = 8,
		.width = 8,
	},
	.mn[1] = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 30,
		.mnctr_mode_shift = 6,
		.n_val_shift = 14,
		.m_val_shift = 8,
		.width = 8,
	},
	.s[0] = {
		.src_sel_shift = 3,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.s[1] = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.mux_sel_bit = 11,
	.freq_tbl = clk_tbl_mdp,
	.clkr = {
		.enable_reg = 0x00c0,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_dyn_rcg_ops,
		},
	},
};

static struct clk_branch mdp_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 10,
	.clkr = {
		.enable_reg = 0x00c0,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mdp_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_lut_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 13,
	.clkr = {
		.enable_reg = 0x016c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&mdp_src.clkr.hw
			},
			.num_parents = 1,
			.name = "mdp_lut_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_vsync_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 22,
	.clkr = {
		.enable_reg = 0x0058,
		.enable_mask = BIT(6),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_vsync_clk",
			.parent_data = (const struct clk_parent_data[]){
				{ .fw_name = "pxo", .name = "pxo_board" },
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * MSM8660/APQ8060-specific MDP pixel clocks
 *
 * MSM8660 uses MD16 register format where M value is in bits [31:16] and
 * D value (2*N-M) is in bits [15:0]. This differs from MSM8960 which uses
 * MD8 format with M in bits [23:16] and D in bits [7:0].
 *
 * NS register offset is 0x00DC (not 0x00E0 as in MSM8960).
 */
static const struct freq_tbl clk_tbl_mdp_pixel[] = {
	{ .src = P_PLL8 },
	{ }
};

static struct clk_rcg mdp_pixel_src = {
	.ns_reg = 0x00dc,
	.md_reg = 0x00d8,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 16,		/* MSM8660: M at bits [31:16] */
		.width = 16,			/* MSM8660: 16-bit M/N counters */
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_mdp_pixel,
	.clkr = {
		.enable_reg = 0x00d4,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_pixel_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_bypass2_ops,
		},
	},
};

static struct clk_branch mdp_pixel_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 23,
	.clkr = {
		.enable_reg = 0x00d4,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_pixel_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mdp_pixel_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_lcdc_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 21,
	.clkr = {
		.enable_reg = 0x00d4,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_lcdc_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mdp_pixel_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_rot[] = {
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 29540000, P_PLL8, 1, 13),
	F_MN( 32000000, P_PLL8, 1, 12),
	F_MN( 38400000, P_PLL8, 1, 10),
	F_MN( 48000000, P_PLL8, 1,  8),
	F_MN( 54860000, P_PLL8, 1,  7),
	F_MN( 64000000, P_PLL8, 1,  6),
	F_MN( 76800000, P_PLL8, 1,  5),
	F_MN( 96000000, P_PLL8, 1,  4),
	F_MN(100000000, P_PLL2, 1,  8),
	F_MN(114290000, P_PLL2, 2, 14),
	F_MN(128000000, P_PLL8, 1,  3),
	F_MN(133330000, P_PLL2, 1,  6),
	F_MN(160000000, P_PLL2, 1,  5),
	{ }
};

static struct clk_dyn_rcg rot_src = {
	.ns_reg[0] = 0x00e8,
	.ns_reg[1] = 0x00e8,
	.md_reg[0] = 0x00e0,
	.md_reg[1] = 0x00e4,
	.bank_reg = 0x00e8,
	.mn[0] = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 25,
		.mnctr_mode_shift = 9,
		.n_val_shift = 22,
		.m_val_shift = 8,
		.width = 8,
	},
	.mn[1] = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 24,
		.mnctr_mode_shift = 6,
		.n_val_shift = 14,
		.m_val_shift = 8,
		.width = 8,
	},
	.s[0] = {
		.src_sel_shift = 3,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.s[1] = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.mux_sel_bit = 11,
	.freq_tbl = clk_tbl_rot,
	.clkr = {
		.enable_reg = 0x00e0,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "rot_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_dyn_rcg_ops,
		},
	},
};

static struct clk_branch rot_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 15,
	.clkr = {
		.enable_reg = 0x00e0,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "rot_clk",
			.parent_hws = (const struct clk_hw*[]){
				&rot_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

#define F_TV(f, s, p_r, _m, _n) \
	{ \
		.freq = f, \
		.src = s, \
		.pre_div = p_r, \
		.m = _m, \
		.n = _n, \
	}

static const struct freq_tbl clk_tbl_tv[] = {
	F_TV( 25200000, P_HDMI_PLL, 1, 0, 0),
	F_TV( 27000000, P_HDMI_PLL, 1, 0, 0),
	F_TV( 27030000, P_HDMI_PLL, 1, 0, 0),
	F_TV( 74250000, P_HDMI_PLL, 1, 0, 0),
	F_TV(148500000, P_HDMI_PLL, 1, 0, 0),
	{ }
};

static const struct parent_map mmcc_pxo_hdmi_map[] = {
	{ P_PXO, 0 },
	{ P_HDMI_PLL, 3 }
};

static const struct clk_parent_data mmcc_pxo_hdmi[] = {
	{ .fw_name = "pxo", .name = "pxo_board" },
	{ .fw_name = "hdmipll", .name = "hdmi_pll" },
};

static struct clk_rcg tv_src = {
	.ns_reg = 0x00f4,
	.md_reg = 0x00f0,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 14,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_hdmi_map,
	},
	.freq_tbl = clk_tbl_tv,
	.clkr = {
		.enable_reg = 0x00ec,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "tv_src",
			.parent_data = mmcc_pxo_hdmi,
			.num_parents = ARRAY_SIZE(mmcc_pxo_hdmi),
			.ops = &clk_rcg_bypass_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch tv_enc_clk = {
	.halt_reg = 0x01d4,
	.halt_bit = 9,
	.clkr = {
		.enable_reg = 0x00ec,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&tv_src.clkr.hw
			},
			.num_parents = 1,
			.name = "tv_enc_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch tv_dac_clk = {
	.halt_reg = 0x01d4,
	.halt_bit = 10,
	.clkr = {
		.enable_reg = 0x00ec,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&tv_src.clkr.hw
			},
			.num_parents = 1,
			.name = "tv_dac_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_tv_clk = {
	.halt_reg = 0x01d4,
	.halt_bit = 12,
	.clkr = {
		.enable_reg = 0x00ec,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&tv_src.clkr.hw
			},
			.num_parents = 1,
			.name = "mdp_tv_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch hdmi_tv_clk = {
	.halt_reg = 0x01d4,
	.halt_bit = 11,
	.clkr = {
		.enable_reg = 0x00ec,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&tv_src.clkr.hw
			},
			.num_parents = 1,
			.name = "hdmi_tv_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch hdmi_app_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 25,
	.clkr = {
		.enable_reg = 0x005c,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.parent_data = (const struct clk_parent_data[]){
				{ .fw_name = "pxo", .name = "pxo_board" },
			},
			.num_parents = 1,
			.name = "hdmi_app_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static const struct freq_tbl clk_tbl_vcodec[] = {
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 32000000, P_PLL8, 1, 12),
	F_MN( 48000000, P_PLL8, 1,  8),
	F_MN( 54860000, P_PLL8, 1,  7),
	F_MN( 96000000, P_PLL8, 1,  4),
	F_MN(133330000, P_PLL2, 1,  6),
	F_MN(200000000, P_PLL2, 1,  4),
	F_MN(228570000, P_PLL2, 2,  7),
	{ }
};

static struct clk_dyn_rcg vcodec_src = {
	.ns_reg[0] = 0x0100,
	.ns_reg[1] = 0x0100,
	.md_reg[0] = 0x00fc,
	.md_reg[1] = 0x0128,
	.bank_reg = 0x00f8,
	.mn[0] = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 31,
		.mnctr_mode_shift = 6,
		.n_val_shift = 11,
		.m_val_shift = 8,
		.width = 8,
	},
	.mn[1] = {
		.mnctr_en_bit = 10,
		.mnctr_reset_bit = 30,
		.mnctr_mode_shift = 11,
		.n_val_shift = 19,
		.m_val_shift = 8,
		.width = 8,
	},
	.s[0] = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.s[1] = {
		.src_sel_shift = 14,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.mux_sel_bit = 13,
	.freq_tbl = clk_tbl_vcodec,
	.clkr = {
		.enable_reg = 0x00f8,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "vcodec_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_dyn_rcg_ops,
		},
	},
};

static struct clk_branch vcodec_clk = {
	.halt_reg = 0x01d0,
	.halt_bit = 29,
	.clkr = {
		.enable_reg = 0x00f8,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "vcodec_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vcodec_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_vpe[] = {
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 34909000, P_PLL8, 1, 11),
	F_MN( 38400000, P_PLL8, 1, 10),
	F_MN( 64000000, P_PLL8, 1,  6),
	F_MN( 76800000, P_PLL8, 1,  5),
	F_MN( 96000000, P_PLL8, 1,  4),
	F_MN(100000000, P_PLL2, 1,  8),
	F_MN(160000000, P_PLL2, 1,  5),
	{ }
};

static struct clk_rcg vpe_src = {
	.ns_reg = 0x0118,
	.p = {
		.pre_div_shift = 12,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_vpe,
	.clkr = {
		.enable_reg = 0x0110,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "vpe_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch vpe_clk = {
	.halt_reg = 0x01c8,
	.halt_bit = 28,
	.clkr = {
		.enable_reg = 0x0110,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "vpe_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vpe_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static const struct freq_tbl clk_tbl_vfe[] = {
	F_MN( 13960000, P_PLL8, 2, 55),
	F_MN( 27000000, P_PXO,  1,  0),
	F_MN( 36570000, P_PLL8, 1, 21),
	F_MN( 38400000, P_PLL8, 2, 20),
	F_MN( 45180000, P_PLL8, 2, 17),
	F_MN( 48000000, P_PLL8, 2, 16),
	F_MN( 54860000, P_PLL8, 1,  7),
	F_MN( 64000000, P_PLL8, 2, 12),
	F_MN( 76800000, P_PLL8, 1,  5),
	F_MN( 96000000, P_PLL8, 2,  8),
	F_MN(109710000, P_PLL8, 1,  7),
	F_MN(128000000, P_PLL8, 1,  3),
	F_MN(153600000, P_PLL8, 2,  5),
	F_MN(200000000, P_PLL2, 2,  8),
	F_MN(228570000, P_PLL2, 2,  7),
	{ }
};

static struct clk_rcg vfe_src = {
	.ns_reg = 0x0108,
	.md_reg = 0x0104,
	.mn = {
		.mnctr_en_bit = 5,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 6,
		.n_val_shift = 16,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 10,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = mmcc_pxo_pll8_pll2_map,
	},
	.freq_tbl = clk_tbl_vfe,
	.clkr = {
		.enable_reg = 0x0104,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_src",
			.parent_data = mmcc_pxo_pll8_pll2,
			.num_parents = ARRAY_SIZE(mmcc_pxo_pll8_pll2),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch vfe_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 6,
	.clkr = {
		.enable_reg = 0x0104,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vfe_src.clkr.hw
			},
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch vfe_csi_clk = {
	.halt_reg = 0x01cc,
	.halt_bit = 8,
	.clkr = {
		.enable_reg = 0x0104,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.parent_hws = (const struct clk_hw*[]){
				&vfe_src.clkr.hw
			},
			.num_parents = 1,
			.name = "vfe_csi_clk",
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch gmem_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 6,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(24),
		.hw.init = &(struct clk_init_data){
			.name = "gmem_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch ijpeg_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 4,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(21),
		.hw.init = &(struct clk_init_data){
			.name = "ijpeg_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch mmss_imem_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 7,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(22),
		.hw.init = &(struct clk_init_data){
			.name = "mmss_imem_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch jpegd_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 5,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(25),
		.hw.init = &(struct clk_init_data){
			.name = "jpegd_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vcodec_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 3,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(19),
		.hw.init = &(struct clk_init_data){
			.name = "vcodec_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vcodec_axi_a_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 26,
	.clkr = {
		.enable_reg = 0x0020,
		.enable_mask = BIT(25),
		.hw.init = &(struct clk_init_data){
			.name = "vcodec_axi_a_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vcodec_axi_b_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 25,
	.clkr = {
		.enable_reg = 0x0020,
		.enable_mask = BIT(26),
		.hw.init = &(struct clk_init_data){
			.name = "vcodec_axi_b_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vfe_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 0,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(18),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vpe_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 1,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(26),
		.hw.init = &(struct clk_init_data){
			.name = "vpe_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch mdp_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 8,
	.clkr = {
		.enable_reg = 0x0018,
		.enable_mask = BIT(23),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch rot_axi_clk = {
	.halt_reg = 0x01d8,
	.halt_bit = 2,
	.clkr = {
		.enable_reg = 0x0020,
		.enable_mask = BIT(22),
		.hw.init = &(struct clk_init_data){
			.name = "rot_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch gfx3d_axi_clk = {
	.halt_reg = 0x01e8,
	.halt_bit = 21,
	.clkr = {
		.enable_reg = 0x0244,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gfx3d_axi_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch amp_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 18,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(24),
		.hw.init = &(struct clk_init_data){
			.name = "amp_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch csi_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 16,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "csi_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch dsi_m_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 19,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "dsi_m_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch dsi_s_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 21,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(18),
		.hw.init = &(struct clk_init_data){
			.name = "dsi_s_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch gfx2d0_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 2,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(19),
		.hw.init = &(struct clk_init_data){
			.name = "gfx2d0_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch gfx2d1_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 3,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gfx2d1_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch gfx3d_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 4,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(3),
		.hw.init = &(struct clk_init_data){
			.name = "gfx3d_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch hdmi_m_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 5,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(14),
		.hw.init = &(struct clk_init_data){
			.name = "hdmi_m_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch hdmi_s_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 6,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(4),
		.hw.init = &(struct clk_init_data){
			.name = "hdmi_s_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch ijpeg_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 9,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "ijpeg_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch jpegd_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 7,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(21),
		.hw.init = &(struct clk_init_data){
			.name = "jpegd_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch mdp_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 11,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch mmss_imem_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 12,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(6),
		.hw.init = &(struct clk_init_data){
			.name = "mmss_imem_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch rot_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 13,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.name = "rot_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch smmu_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 22,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(15),
		.hw.init = &(struct clk_init_data){
			.name = "smmu_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch tv_enc_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 23,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(25),
		.hw.init = &(struct clk_init_data){
			.name = "tv_enc_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vcodec_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 10,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "vcodec_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vfe_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 14,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch vpe_ahb_clk = {
	.halt_reg = 0x01dc,
	.halt_bit = 15,
	.clkr = {
		.enable_reg = 0x0008,
		.enable_mask = BIT(16),
		.hw.init = &(struct clk_init_data){
			.name = "vpe_ahb_clk",
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_regmap *mmcc_msm8660_clks[] = {
	[TV_ENC_AHB_CLK] = &tv_enc_ahb_clk.clkr,
	[AMP_AHB_CLK] = &amp_ahb_clk.clkr,
	[JPEGD_AHB_CLK] = &jpegd_ahb_clk.clkr,
	[GFX2D0_AHB_CLK] = &gfx2d0_ahb_clk.clkr,
	[DSI_S_AHB_CLK] = &dsi_s_ahb_clk.clkr,
	[VPE_AHB_CLK] = &vpe_ahb_clk.clkr,
	[SMMU_AHB_CLK] = &smmu_ahb_clk.clkr,
	[HDMI_M_AHB_CLK] = &hdmi_m_ahb_clk.clkr,
	[VFE_AHB_CLK] = &vfe_ahb_clk.clkr,
	[ROT_AHB_CLK] = &rot_ahb_clk.clkr,
	[VCODEC_AHB_CLK] = &vcodec_ahb_clk.clkr,
	[MDP_AHB_CLK] = &mdp_ahb_clk.clkr,
	[DSI_M_AHB_CLK] = &dsi_m_ahb_clk.clkr,
	[CSI_AHB_CLK] = &csi_ahb_clk.clkr,
	[MMSS_IMEM_AHB_CLK] = &mmss_imem_ahb_clk.clkr,
	[IJPEG_AHB_CLK] = &ijpeg_ahb_clk.clkr,
	[HDMI_S_AHB_CLK] = &hdmi_s_ahb_clk.clkr,
	[GFX3D_AHB_CLK] = &gfx3d_ahb_clk.clkr,
	[GFX2D1_AHB_CLK] = &gfx2d1_ahb_clk.clkr,
	[JPEGD_AXI_CLK] = &jpegd_axi_clk.clkr,
	[GMEM_AXI_CLK] = &gmem_axi_clk.clkr,
	[MDP_AXI_CLK] = &mdp_axi_clk.clkr,
	[MMSS_IMEM_AXI_CLK] = &mmss_imem_axi_clk.clkr,
	[IJPEG_AXI_CLK] = &ijpeg_axi_clk.clkr,
	[GFX3D_AXI_CLK] = &gfx3d_axi_clk.clkr,
	[VCODEC_AXI_CLK] = &vcodec_axi_clk.clkr,
	[VFE_AXI_CLK] = &vfe_axi_clk.clkr,
	[VPE_AXI_CLK] = &vpe_axi_clk.clkr,
	[ROT_AXI_CLK] = &rot_axi_clk.clkr,
	[VCODEC_AXI_A_CLK] = &vcodec_axi_a_clk.clkr,
	[VCODEC_AXI_B_CLK] = &vcodec_axi_b_clk.clkr,
	[CSI0_SRC] = &csi0_src.clkr,
	[CSI0_CLK] = &csi0_clk.clkr,
	[CSI0_PHY_CLK] = &csi0_phy_clk.clkr,
	[CSI1_SRC] = &csi1_src.clkr,
	[CSI1_CLK] = &csi1_clk.clkr,
	[CSI1_PHY_CLK] = &csi1_phy_clk.clkr,
	[DSI_SRC] = &dsi1_src.clkr,
	[DSI_CLK] = &dsi1_clk.clkr,
	[CSI_PIX_CLK] = &csi_pix_clk.clkr,
	[CSI_RDI_CLK] = &csi_rdi_clk.clkr,
	[MDP_VSYNC_CLK] = &mdp_vsync_clk.clkr,
	[HDMI_APP_CLK] = &hdmi_app_clk.clkr,
	[GFX2D0_SRC] = &gfx2d0_src.clkr,
	[GFX2D0_CLK] = &gfx2d0_clk.clkr,
	[GFX2D1_SRC] = &gfx2d1_src.clkr,
	[GFX2D1_CLK] = &gfx2d1_clk.clkr,
	[GFX3D_SRC] = &gfx3d_src.clkr,
	[GFX3D_CLK] = &gfx3d_clk.clkr,
	[IJPEG_SRC] = &ijpeg_src.clkr,
	[IJPEG_CLK] = &ijpeg_clk.clkr,
	[JPEGD_SRC] = &jpegd_src.clkr,
	[JPEGD_CLK] = &jpegd_clk.clkr,
	[MDP_SRC] = &mdp_src.clkr,
	[MDP_CLK] = &mdp_clk.clkr,
	[MDP_LUT_CLK] = &mdp_lut_clk.clkr,
	[MDP_PIXEL_SRC] = &mdp_pixel_src.clkr,
	[MDP_PIXEL_CLK] = &mdp_pixel_clk.clkr,
	[MDP_LCDC_CLK] = &mdp_lcdc_clk.clkr,
	[DSI1_BYTE_SRC] = &dsi1_byte_src.clkr,
	[DSI1_BYTE_CLK] = &dsi1_byte_clk.clkr,
	[DSI1_ESC_SRC] = &dsi1_esc_src.clkr,
	[DSI1_ESC_CLK] = &dsi1_esc_clk.clkr,
	[ROT_SRC] = &rot_src.clkr,
	[ROT_CLK] = &rot_clk.clkr,
	[TV_ENC_CLK] = &tv_enc_clk.clkr,
	[TV_DAC_CLK] = &tv_dac_clk.clkr,
	[HDMI_TV_CLK] = &hdmi_tv_clk.clkr,
	[MDP_TV_CLK] = &mdp_tv_clk.clkr,
	[TV_SRC] = &tv_src.clkr,
	[VCODEC_SRC] = &vcodec_src.clkr,
	[VCODEC_CLK] = &vcodec_clk.clkr,
	[VFE_SRC] = &vfe_src.clkr,
	[VFE_CLK] = &vfe_clk.clkr,
	[VFE_CSI_CLK] = &vfe_csi_clk.clkr,
	[VPE_SRC] = &vpe_src.clkr,
	[VPE_CLK] = &vpe_clk.clkr,
	[DSI_PIXEL_SRC] = &dsi1_pixel_src.clkr,
	[DSI_PIXEL_CLK] = &dsi1_pixel_clk.clkr,
	[CAMCLK0_SRC] = &camclk0_src.clkr,
	[CAMCLK0_CLK] = &camclk0_clk.clkr,
	[CAMCLK1_SRC] = &camclk1_src.clkr,
	[CAMCLK1_CLK] = &camclk1_clk.clkr,
	[CSIPHYTIMER_SRC] = &csiphytimer_src.clkr,
	[CSIPHY1_TIMER_CLK] = &csiphy1_timer_clk.clkr,
	[CSIPHY0_TIMER_CLK] = &csiphy0_timer_clk.clkr,
	[PLL2] = &pll2.clkr,
};

static const struct qcom_reset_map mmcc_msm8660_resets[] = {
	[GFX3D_AXI_RESET] = { 0x0208, 17 },
	[VPE_AXI_RESET] = { 0x0208, 15 },
	[IJPEG_AXI_RESET] = { 0x0208, 14 },
	[MPD_AXI_RESET] = { 0x0208, 13 },
	[VFE_AXI_RESET] = { 0x0208, 9 },
	[SP_AXI_RESET] = { 0x0208, 8 },
	[VCODEC_AXI_RESET] = { 0x0208, 7 },
	[ROT_AXI_RESET] = { 0x0208, 6 },
	[VCODEC_AXI_A_RESET] = { 0x0208, 5 },
	[VCODEC_AXI_B_RESET] = { 0x0208, 4 },
	[FAB_S3_AXI_RESET] = { 0x0208, 3 },
	[FAB_S2_AXI_RESET] = { 0x0208, 2 },
	[FAB_S1_AXI_RESET] = { 0x0208, 1 },
	[FAB_S0_AXI_RESET] = { 0x0208 },
	[SMMU_GFX3D_ABH_RESET] = { 0x020c, 31 },
	[SMMU_VPE_AHB_RESET] = { 0x020c, 30 },
	[SMMU_VFE_AHB_RESET] = { 0x020c, 29 },
	[SMMU_ROT_AHB_RESET] = { 0x020c, 28 },
	[SMMU_VCODEC_B_AHB_RESET] = { 0x020c, 27 },
	[SMMU_VCODEC_A_AHB_RESET] = { 0x020c, 26 },
	[SMMU_MDP1_AHB_RESET] = { 0x020c, 25 },
	[SMMU_MDP0_AHB_RESET] = { 0x020c, 24 },
	[SMMU_JPEGD_AHB_RESET] = { 0x020c, 23 },
	[SMMU_IJPEG_AHB_RESET] = { 0x020c, 22 },
	[APU_AHB_RESET] = { 0x020c, 18 },
	[CSI_AHB_RESET] = { 0x020c, 17 },
	[TV_ENC_AHB_RESET] = { 0x020c, 15 },
	[VPE_AHB_RESET] = { 0x020c, 14 },
	[FABRIC_AHB_RESET] = { 0x020c, 13 },
	[GFX3D_AHB_RESET] = { 0x020c, 10 },
	[HDMI_AHB_RESET] = { 0x020c, 9 },
	[MSSS_IMEM_AHB_RESET] = { 0x020c, 8 },
	[IJPEG_AHB_RESET] = { 0x020c, 7 },
	[DSI_M_AHB_RESET] = { 0x020c, 6 },
	[DSI_S_AHB_RESET] = { 0x020c, 5 },
	[JPEGD_AHB_RESET] = { 0x020c, 4 },
	[MDP_AHB_RESET] = { 0x020c, 3 },
	[ROT_AHB_RESET] = { 0x020c, 2 },
	[VCODEC_AHB_RESET] = { 0x020c, 1 },
	[VFE_AHB_RESET] = { 0x020c, 0 },
	[CSIPHY0_RESET] = { 0x0210, 29 },
	[CSIPHY1_RESET] = { 0x0210, 28 },
	[CSI_RDI_RESET] = { 0x0210, 27 },
	[CSI_PIX_RESET] = { 0x0210, 26 },
	[VFE_CSI_RESET] = { 0x0210, 24 },
	[MDP_RESET] = { 0x0210, 21 },
	[AMP_RESET] = { 0x0210, 20 },
	[JPEGD_RESET] = { 0x0210, 19 },
	[CSI1_RESET] = { 0x0210, 18 },
	[VPE_RESET] = { 0x0210, 17 },
	[MMSS_FABRIC_RESET] = { 0x0210, 16 },
	[VFE_RESET] = { 0x0210, 15 },
	[GFX3D_RESET] = { 0x0210, 12 },
	[HDMI_RESET] = { 0x0210, 11 },
	[MMSS_IMEM_RESET] = { 0x0210, 10 },
	[IJPEG_RESET] = { 0x0210, 9 },
	[CSI0_RESET] = { 0x0210, 8 },
	[DSI_RESET] = { 0x0210, 7 },
	[VCODEC_RESET] = { 0x0210, 6 },
	[MDP_TV_RESET] = { 0x0210, 4 },
	[MDP_VSYNC_RESET] = { 0x0210, 3 },
	[ROT_RESET] = { 0x0210, 2 },
	[TV_HDMI_RESET] = { 0x0210, 1 },
	[TV_ENC_RESET] = { 0x0210, 0 },
};

static const struct regmap_config mmcc_msm8660_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x334,
	.fast_io	= true,
};

static const struct qcom_cc_desc mmcc_msm8660_desc = {
	.config = &mmcc_msm8660_regmap_config,
	.clks = mmcc_msm8660_clks,
	.num_clks = ARRAY_SIZE(mmcc_msm8660_clks),
	.resets = mmcc_msm8660_resets,
	.num_resets = ARRAY_SIZE(mmcc_msm8660_resets),
};

static const struct of_device_id mmcc_msm8660_match_table[] = {
	{ .compatible = "qcom,mmcc-msm8660", .data = &mmcc_msm8660_desc },
	{ .compatible = "qcom,mmcc-apq8060", .data = &mmcc_msm8660_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, mmcc_msm8660_match_table);

static int mmcc_msm8660_probe(struct platform_device *pdev)
{
	struct regmap *regmap;

	regmap = qcom_cc_map(pdev, &mmcc_msm8660_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return qcom_cc_really_probe(&pdev->dev, &mmcc_msm8660_desc, regmap);
}

static struct platform_driver mmcc_msm8660_driver = {
	.probe		= mmcc_msm8660_probe,
	.driver		= {
		.name	= "mmcc-msm8660",
		.of_match_table = mmcc_msm8660_match_table,
	},
};

module_platform_driver(mmcc_msm8660_driver);

MODULE_DESCRIPTION("QCOM MMCC MSM8660/APQ8060 Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mmcc-msm8660");
