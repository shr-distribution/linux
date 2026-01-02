// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Linux community
 *
 * Qualcomm MSM7x30 Global Clock Controller driver
 *
 * Based on legacy arch/arm/mach-msm/clock-7x30.c
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,gcc-msm7x30.h>

#include "common.h"
#include "clk-regmap.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "reset.h"

/*
 * Register offsets in Shadow Region 2 (SH2)
 * Base: 0xABA01000
 */
#define QUP_I2C_NS_REG			0x04F0
#define CAM_NS_REG			0x0374
#define CAM_VFE_NS_REG			0x0044
#define CLK_HALT_STATEA_REG		0x0108
#define CLK_HALT_STATEB_REG		0x010C
#define CLK_HALT_STATEC_REG		0x02D4
#define CSI_NS_REG			0x0174
#define EMDH_NS_REG			0x0050
#define GLBL_CLK_ENA_2_SC_REG		0x03C0
#define GLBL_CLK_ENA_SC_REG		0x03BC
#define GLBL_CLK_STATE_2_REG		0x037C
#define GLBL_CLK_STATE_REG		0x0004
#define GRP_2D_NS_REG			0x0034
#define GRP_NS_REG			0x0084
#define HDMI_NS_REG			0x0484
#define I2C_2_NS_REG			0x02D8
#define I2C_NS_REG			0x0068
#define JPEG_NS_REG			0x0164
#define LPA_CORE_CLK_MA0_REG		0x04F4
#define LPA_CORE_CLK_MA2_REG		0x04FC
#define LPA_NS_REG			0x02E8
#define MDC_NS_REG			0x007C
#define MDP_LCDC_NS_REG			0x0390
#define MDP_NS_REG			0x014C
#define MDP_VSYNC_REG			0x0460
#define MFC_NS_REG			0x0154
#define MI2S_CODEC_RX_DIV_REG		0x02EC
#define MI2S_CODEC_TX_DIV_REG		0x02F0
#define MI2S_DIV_REG			0x02E4
#define MI2S_NS_REG			0x02E0
#define MI2S_RX_NS_REG			0x0070
#define MI2S_TX_NS_REG			0x0078
#define MIDI_NS_REG			0x02D0
#define PLL_ENA_REG			0x0264
#define PMDH_NS_REG			0x008C
#define SDAC_NS_REG			0x009C
#define SDC1_NS_REG			0x00A4
#define SDC2_NS_REG			0x00AC
#define SDC3_NS_REG			0x00B4
#define SDC4_NS_REG			0x00BC
#define SPI_NS_REG			0x02C8
#define TSIF_NS_REG			0x00C4
#define TV_NS_REG			0x00CC
#define UART1DM_NS_REG			0x00D4
#define UART2DM_NS_REG			0x00DC
#define UART2_NS_REG			0x0464
#define UART_NS_REG			0x00E0
#define USBH2_NS_REG			0x046C
#define USBH3_NS_REG			0x0470
#define USBH_MD_REG			0x02BC
#define USBH_NS_REG			0x02C0
#define VPE_NS_REG			0x015C

/*
 * Register offsets in Base Region
 * Base: 0xAB800000
 */
#define PLL0_STATUS_REG			0x0318
#define PLL1_STATUS_REG			0x0334
#define PLL2_STATUS_REG			0x0350
#define PLL3_STATUS_REG			0x036C
#define PLL4_STATUS_REG			0x0254
#define PLL5_STATUS_REG			0x0258
#define PLL6_STATUS_REG			0x04EC

/* PLL frequencies (approximate) */
#define PLL0_FREQ	196608000	/* Modem PLL - not used by apps */
#define PLL1_FREQ	768000000	/* Global PLL */
#define PLL2_FREQ	1200000000	/* CPU PLL - managed by ACPU driver */
#define PLL3_FREQ	737280000	/* Multimedia/Peripheral PLL */
#define PLL4_FREQ	891000000	/* Display PLL */

/* Source select values */
#define SRC_SEL_TCXO	0
#define SRC_SEL_PLL1	1
#define SRC_SEL_PLL4	2
#define SRC_SEL_PLL3	3
#define SRC_SEL_PLL0	4
#define SRC_SEL_LPXO	6

enum {
	P_TCXO,
	P_PLL0,
	P_PLL1,
	P_PLL2,
	P_PLL3,
	P_PLL4,
	P_LPXO,
};

static const struct parent_map __maybe_unused gcc_pll_map[] = {
	{ P_TCXO, SRC_SEL_TCXO },
	{ P_PLL0, SRC_SEL_PLL0 },
	{ P_PLL1, SRC_SEL_PLL1 },
	{ P_PLL3, SRC_SEL_PLL3 },
	{ P_PLL4, SRC_SEL_PLL4 },
	{ P_LPXO, SRC_SEL_LPXO },
};

static const char * const gcc_pll_parents[] = {
	"tcxo",
	"pll0",
	"pll1",
	"pll3",
	"pll4",
	"lpxo",
};

/*
 * Fixed-rate PLLs
 * In reality these are configurable, but we treat them as fixed
 * since the rates are set by the bootloader/modem
 */

static struct clk_fixed_rate pll1_clk = {
	.fixed_rate = PLL1_FREQ,
	.hw.init = &(struct clk_init_data){
		.name = "pll1",
		.ops = &clk_fixed_rate_ops,
	},
};

static struct clk_fixed_rate pll3_clk = {
	.fixed_rate = PLL3_FREQ,
	.hw.init = &(struct clk_init_data){
		.name = "pll3",
		.ops = &clk_fixed_rate_ops,
	},
};

static struct clk_fixed_rate pll4_clk = {
	.fixed_rate = PLL4_FREQ,
	.hw.init = &(struct clk_init_data){
		.name = "pll4",
		.ops = &clk_fixed_rate_ops,
	},
};

/*
 * Root clock - must be enabled for all peripheral clocks
 */
static struct clk_branch glbl_root_clk = {
	.halt_check = BRANCH_HALT_SKIP,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(29),
		.hw.init = &(struct clk_init_data){
			.name = "glbl_root_clk",
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * I2C Clocks - Fixed at TCXO rate (19.2 MHz)
 */
static struct clk_branch i2c_clk = {
	.halt_reg = CLK_HALT_STATEA_REG,
	.halt_bit = 15,
	.clkr = {
		.enable_reg = I2C_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "i2c_clk",
			.parent_names = (const char *[]){ "tcxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch i2c_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 13,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.name = "i2c_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch i2c_2_clk = {
	.halt_reg = CLK_HALT_STATEC_REG,
	.halt_bit = 2,
	.clkr = {
		.enable_reg = I2C_2_NS_REG,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "i2c_2_clk",
			.parent_names = (const char *[]){ "tcxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch i2c_2_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 14,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(14),
		.hw.init = &(struct clk_init_data){
			.name = "i2c_2_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch qup_i2c_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 31,
	.clkr = {
		.enable_reg = QUP_I2C_NS_REG,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "qup_i2c_clk",
			.parent_names = (const char *[]){ "tcxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch qup_i2c_p_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_bit = 0,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "qup_i2c_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * UART clocks - Fixed at TCXO rate
 */
static struct clk_branch uart1_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 7,
	.clkr = {
		.enable_reg = UART_NS_REG,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "uart1_clk",
			.parent_names = (const char *[]){ "tcxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch uart1_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 10,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "uart1_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch uart2_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 5,
	.clkr = {
		.enable_reg = UART2_NS_REG,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "uart2_clk",
			.parent_names = (const char *[]){ "tcxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch uart2_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 11,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "uart2_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * UART3 - Debug UART (GPIOs 53/54)
 */
#define UART3_NS_REG	0x0468

static struct clk_branch uart3_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 4,
	.clkr = {
		.enable_reg = UART3_NS_REG,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "uart3_clk",
			.parent_names = (const char *[]){ "tcxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch uart3_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 12,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.name = "uart3_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * UART DM clocks - MND16 type with rate selection
 * Frequency table: 3.6864 - 64 MHz
 */
static const struct freq_tbl ftbl_uart_dm[] = {
	{  3686400, P_PLL3, 3, 3, 200 },
	{  7372800, P_PLL3, 3, 3, 100 },
	{ 14745600, P_PLL3, 3, 3, 50 },
	{ 32000000, P_PLL3, 3, 25, 192 },
	{ 40000000, P_PLL3, 3, 125, 768 },
	{ 46400000, P_PLL3, 3, 145, 768 },
	{ 48000000, P_PLL3, 3, 25, 128 },
	{ 51200000, P_PLL3, 3, 5, 24 },
	{ 56000000, P_PLL3, 3, 175, 768 },
	{ 58982400, P_PLL3, 3, 6, 25 },
	{ 64000000, P_PLL1, 4, 1, 3 },
	{ }
};

static struct clk_rcg __maybe_unused uart1dm_src = {
	.ns_reg = UART1DM_NS_REG,
	.md_reg = UART1DM_NS_REG - 4,
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
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_uart_dm,
	.clkr = {
		.enable_reg = UART1DM_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "uart1dm_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch uart1dm_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 6,
	.clkr = {
		.enable_reg = UART1DM_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "uart1dm_clk",
			.parent_names = (const char *[]){ "uart1dm_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_rcg __maybe_unused uart2dm_src = {
	.ns_reg = UART2DM_NS_REG,
	.md_reg = UART2DM_NS_REG - 4,
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
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_uart_dm,
	.clkr = {
		.enable_reg = UART2DM_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "uart2dm_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch uart2dm_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 23,
	.clkr = {
		.enable_reg = UART2DM_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "uart2dm_clk",
			.parent_names = (const char *[]){ "uart2dm_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch uart1dm_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 17,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(17),
		.hw.init = &(struct clk_init_data){
			.name = "uart1dm_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch uart2dm_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 26,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(26),
		.hw.init = &(struct clk_init_data){
			.name = "uart2dm_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * SDC clocks - MND8 type
 */
static const struct freq_tbl ftbl_sdc[] = {
	{   144000, P_LPXO, 1, 1, 171 },
	{   400000, P_LPXO, 1, 2, 123 },
	{ 16027000, P_PLL3, 3, 14, 215 },
	{ 17000000, P_PLL3, 4, 19, 206 },
	{ 20480000, P_PLL3, 4, 23, 212 },
	{ 24576000, P_LPXO, 1, 0, 0 },
	{ 49152000, P_PLL3, 3, 1, 5 },
	{ }
};

static struct clk_rcg __maybe_unused sdc1_src = {
	.ns_reg = SDC1_NS_REG,
	.md_reg = SDC1_NS_REG - 4,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 12,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_sdc,
	.clkr = {
		.enable_reg = SDC1_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "sdc1_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch sdc1_clk = {
	.halt_reg = CLK_HALT_STATEA_REG,
	.halt_bit = 1,
	.clkr = {
		.enable_reg = SDC1_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "sdc1_clk",
			.parent_names = (const char *[]){ "sdc1_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch sdc1_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 7,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "sdc1_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_rcg __maybe_unused sdc2_src = {
	.ns_reg = SDC2_NS_REG,
	.md_reg = SDC2_NS_REG - 4,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 13,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_sdc,
	.clkr = {
		.enable_reg = SDC2_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "sdc2_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch sdc2_clk = {
	.halt_reg = CLK_HALT_STATEA_REG,
	.halt_bit = 0,
	.clkr = {
		.enable_reg = SDC2_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "sdc2_clk",
			.parent_names = (const char *[]){ "sdc2_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch sdc2_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 8,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data){
			.name = "sdc2_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_rcg __maybe_unused sdc3_src = {
	.ns_reg = SDC3_NS_REG,
	.md_reg = SDC3_NS_REG - 4,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 12,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_sdc,
	.clkr = {
		.enable_reg = SDC3_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "sdc3_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch sdc3_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 24,
	.clkr = {
		.enable_reg = SDC3_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "sdc3_clk",
			.parent_names = (const char *[]){ "sdc3_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch sdc3_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 27,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(27),
		.hw.init = &(struct clk_init_data){
			.name = "sdc3_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_rcg __maybe_unused sdc4_src = {
	.ns_reg = SDC4_NS_REG,
	.md_reg = SDC4_NS_REG - 4,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 13,
		.m_val_shift = 8,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_sdc,
	.clkr = {
		.enable_reg = SDC4_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "sdc4_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch sdc4_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 25,
	.clkr = {
		.enable_reg = SDC4_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "sdc4_clk",
			.parent_names = (const char *[]){ "sdc4_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch sdc4_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 28,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(28),
		.hw.init = &(struct clk_init_data){
			.name = "sdc4_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * USB HS clocks
 */
static const struct freq_tbl ftbl_usb_hs[] = {
	{ 60000000, P_PLL1, 2, 5, 32 },
	{ }
};

static struct clk_rcg __maybe_unused usb_hs_src = {
	.ns_reg = USBH_NS_REG,
	.md_reg = USBH_MD_REG,
	.mn = {
		.mnctr_en_bit = 8,
		.mnctr_reset_bit = 7,
		.mnctr_mode_shift = 5,
		.n_val_shift = 16,
		.m_val_shift = 16,
		.width = 8,
	},
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 2,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_usb_hs,
	.clkr = {
		.enable_reg = USBH_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "usb_hs_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch usb_hs_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 26,
	.clkr = {
		.enable_reg = USBH_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "usb_hs_clk",
			.parent_names = (const char *[]){ "usb_hs_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch usb_hs_core_clk = {
	.halt_reg = CLK_HALT_STATEA_REG,
	.halt_bit = 27,
	.clkr = {
		.enable_reg = USBH_NS_REG,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.name = "usb_hs_core_clk",
			.parent_names = (const char *[]){ "usb_hs_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch usb_hs_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 25,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(25),
		.hw.init = &(struct clk_init_data){
			.name = "usb_hs_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * MDP clocks
 */
static const struct freq_tbl ftbl_mdp[] = {
	{  46080000, P_PLL3, 16, 0, 0 },
	{  49152000, P_PLL3, 15, 0, 0 },
	{  52663000, P_PLL3, 14, 0, 0 },
	{  92160000, P_PLL3, 8, 0, 0 },
	{ 122880000, P_PLL3, 6, 0, 0 },
	{ 147456000, P_PLL3, 5, 0, 0 },
	{ 153600000, P_PLL1, 5, 0, 0 },
	{ 192000000, P_PLL1, 4, 0, 0 },
	{ }
};

static struct clk_rcg __maybe_unused mdp_src = {
	.ns_reg = MDP_NS_REG,
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_mdp,
	.clkr = {
		.enable_reg = MDP_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch mdp_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 16,
	.clkr = {
		.enable_reg = MDP_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_clk",
			.parent_names = (const char *[]){ "mdp_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_p_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(6),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * MDP LCDC (LCD controller) clocks
 */
static const struct freq_tbl ftbl_mdp_lcdc[] = {
	{ 24576000, P_LPXO, 1, 0, 0 },
	{ 30720000, P_PLL3, 4, 1, 6 },
	{ 32768000, P_PLL3, 3, 2, 15 },
	{ 40960000, P_PLL3, 2, 1, 9 },
	{ 73728000, P_PLL3, 2, 1, 5 },
	{ }
};

static struct clk_rcg __maybe_unused mdp_lcdc_pclk_src = {
	.ns_reg = MDP_LCDC_NS_REG,
	.md_reg = MDP_LCDC_NS_REG - 4,
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
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_mdp_lcdc,
	.clkr = {
		.enable_reg = MDP_LCDC_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_lcdc_pclk_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch mdp_lcdc_pclk_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 28,
	.clkr = {
		.enable_reg = MDP_LCDC_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_lcdc_pclk_clk",
			.parent_names = (const char *[]){ "mdp_lcdc_pclk_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_lcdc_pad_pclk_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 29,
	.clkr = {
		.enable_reg = MDP_LCDC_NS_REG,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_lcdc_pad_pclk_clk",
			.parent_names = (const char *[]){ "mdp_lcdc_pclk_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch mdp_vsync_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 30,
	.clkr = {
		.enable_reg = MDP_VSYNC_REG,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "mdp_vsync_clk",
			.parent_names = (const char *[]){ "lpxo" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * MDDI clocks (Mobile Display Digital Interface)
 */
static const struct freq_tbl ftbl_mddi[] = {
	{  49150000, P_PLL3, 15, 0, 0 },
	{  92160000, P_PLL3, 8, 0, 0 },
	{ 122880000, P_PLL3, 6, 0, 0 },
	{ 184320000, P_PLL3, 4, 0, 0 },
	{ 245760000, P_PLL3, 3, 0, 0 },
	{ 368640000, P_PLL3, 2, 0, 0 },
	{ 384000000, P_PLL1, 2, 0, 0 },
	{ 445500000, P_PLL4, 2, 0, 0 },
	{ }
};

static struct clk_rcg __maybe_unused pmdh_src = {
	.ns_reg = PMDH_NS_REG,
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_mddi,
	.clkr = {
		.enable_reg = PMDH_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "pmdh_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch pmdh_clk = {
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = PMDH_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "pmdh_clk",
			.parent_names = (const char *[]){ "pmdh_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch pmdh_p_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(4),
		.hw.init = &(struct clk_init_data){
			.name = "pmdh_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * Graphics clocks (Adreno 205)
 */
static const struct freq_tbl ftbl_grp[] = {
	{  24576000, P_LPXO, 1, 0, 0 },
	{  46080000, P_PLL3, 16, 0, 0 },
	{  49152000, P_PLL3, 15, 0, 0 },
	{  52662875, P_PLL3, 14, 0, 0 },
	{  56713846, P_PLL3, 13, 0, 0 },
	{  61440000, P_PLL3, 12, 0, 0 },
	{  67025454, P_PLL3, 11, 0, 0 },
	{  73728000, P_PLL3, 10, 0, 0 },
	{  81920000, P_PLL3, 9, 0, 0 },
	{  92160000, P_PLL3, 8, 0, 0 },
	{ 105325714, P_PLL3, 7, 0, 0 },
	{ 122880000, P_PLL3, 6, 0, 0 },
	{ 147456000, P_PLL3, 5, 0, 0 },
	{ 184320000, P_PLL3, 4, 0, 0 },
	{ 192000000, P_PLL1, 4, 0, 0 },
	{ 245760000, P_PLL3, 3, 0, 0 },
	{ }
};

static struct clk_rcg __maybe_unused grp_2d_src = {
	.ns_reg = GRP_2D_NS_REG,
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_grp,
	.clkr = {
		.enable_reg = GRP_2D_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "grp_2d_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch grp_2d_clk = {
	.halt_reg = CLK_HALT_STATEA_REG,
	.halt_bit = 31,
	.clkr = {
		.enable_reg = GRP_2D_NS_REG,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "grp_2d_clk",
			.parent_names = (const char *[]){ "grp_2d_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch grp_2d_p_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_bit = 24,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(24),
		.hw.init = &(struct clk_init_data){
			.name = "grp_2d_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_rcg __maybe_unused grp_3d_src = {
	.ns_reg = GRP_NS_REG,
	.p = {
		.pre_div_shift = 3,
		.pre_div_width = 4,
	},
	.s = {
		.src_sel_shift = 0,
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_grp,
	.clkr = {
		.enable_reg = GRP_NS_REG,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data){
			.name = "grp_3d_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch grp_3d_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 18,
	.clkr = {
		.enable_reg = GRP_NS_REG,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "grp_3d_clk",
			.parent_names = (const char *[]){ "grp_3d_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch grp_3d_p_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_bit = 17,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(17),
		.hw.init = &(struct clk_init_data){
			.name = "grp_3d_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch imem_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 19,
	.clkr = {
		.enable_reg = GRP_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "imem_clk",
			.parent_names = (const char *[]){ "grp_3d_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

/*
 * Camera clocks
 */
static const struct freq_tbl ftbl_cam[] = {
	{  6000000, P_PLL1, 4, 1, 32 },
	{  8000000, P_PLL1, 4, 1, 24 },
	{ 12000000, P_PLL1, 4, 1, 16 },
	{ 16000000, P_PLL1, 4, 1, 12 },
	{ 19200000, P_PLL1, 4, 1, 10 },
	{ 24000000, P_PLL1, 4, 1, 8 },
	{ 32000000, P_PLL1, 4, 1, 6 },
	{ 48000000, P_PLL1, 4, 1, 4 },
	{ 64000000, P_PLL1, 4, 1, 3 },
	{ }
};

static struct clk_rcg __maybe_unused cam_m_src = {
	.ns_reg = CAM_NS_REG,
	.md_reg = CAM_NS_REG - 4,
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
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_cam,
	.clkr = {
		.enable_reg = CAM_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "cam_m_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch cam_m_clk = {
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = CAM_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "cam_m_clk",
			.parent_names = (const char *[]){ "cam_m_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

/*
 * VFE (Video Front End) clocks
 */
static const struct freq_tbl ftbl_vfe[] = {
	{  36864000, P_PLL3, 4, 1, 5 },
	{  46080000, P_PLL3, 4, 1, 4 },
	{  61440000, P_PLL3, 4, 1, 3 },
	{  73728000, P_PLL3, 2, 1, 5 },
	{  81920000, P_PLL3, 3, 1, 3 },
	{  92160000, P_PLL3, 4, 1, 2 },
	{  98304000, P_PLL3, 3, 2, 5 },
	{ 105326000, P_PLL3, 2, 2, 7 },
	{ 122880000, P_PLL3, 2, 1, 3 },
	{ 147456000, P_PLL3, 2, 2, 5 },
	{ 153600000, P_PLL1, 2, 2, 5 },
	{ }
};

static struct clk_rcg __maybe_unused vfe_src = {
	.ns_reg = CAM_VFE_NS_REG,
	.md_reg = CAM_VFE_NS_REG - 4,
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
		.parent_map = gcc_pll_map,
	},
	.freq_tbl = ftbl_vfe,
	.clkr = {
		.enable_reg = CAM_VFE_NS_REG,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_src",
			.parent_names = gcc_pll_parents,
			.num_parents = ARRAY_SIZE(gcc_pll_parents),
			.ops = &clk_rcg_ops,
		},
	},
};

static struct clk_branch vfe_clk = {
	.halt_reg = CLK_HALT_STATEB_REG,
	.halt_bit = 0,
	.clkr = {
		.enable_reg = CAM_VFE_NS_REG,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_clk",
			.parent_names = (const char *[]){ "vfe_src" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct clk_branch vfe_p_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_bit = 27,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(27),
		.hw.init = &(struct clk_init_data){
			.name = "vfe_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * ADM (Application Data Mover / DMA) clock
 */
static struct clk_branch adm_clk = {
	.halt_reg = GLBL_CLK_STATE_REG,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_SC_REG,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "adm_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * Rotator clocks
 */
static struct clk_branch rotator_imem_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_bit = 23,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(23),
		.hw.init = &(struct clk_init_data){
			.name = "rotator_imem_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

static struct clk_branch rotator_p_clk = {
	.halt_reg = GLBL_CLK_STATE_2_REG,
	.halt_bit = 25,
	.clkr = {
		.enable_reg = GLBL_CLK_ENA_2_SC_REG,
		.enable_mask = BIT(25),
		.hw.init = &(struct clk_init_data){
			.name = "rotator_p_clk",
			.parent_names = (const char *[]){ "glbl_root_clk" },
			.num_parents = 1,
			.ops = &clk_branch_ops,
		},
	},
};

/*
 * Clock registration array
 */
static struct clk_regmap *gcc_msm7x30_clks[] = {
	[GLBL_ROOT_CLK] = &glbl_root_clk.clkr,
	/* I2C clocks */
	[I2C_CLK] = &i2c_clk.clkr,
	[I2C_P_CLK] = &i2c_p_clk.clkr,
	[I2C_2_CLK] = &i2c_2_clk.clkr,
	[I2C_2_P_CLK] = &i2c_2_p_clk.clkr,
	[QUP_I2C_CLK] = &qup_i2c_clk.clkr,
	[QUP_I2C_P_CLK] = &qup_i2c_p_clk.clkr,
	/* UART clocks */
	[UART1_CLK] = &uart1_clk.clkr,
	[UART1_P_CLK] = &uart1_p_clk.clkr,
	[UART2_CLK] = &uart2_clk.clkr,
	[UART2_P_CLK] = &uart2_p_clk.clkr,
	[UART3_CLK] = &uart3_clk.clkr,
	[UART3_P_CLK] = &uart3_p_clk.clkr,
	[UART1DM_CLK] = &uart1dm_clk.clkr,
	[UART1DM_P_CLK] = &uart1dm_p_clk.clkr,
	[UART2DM_CLK] = &uart2dm_clk.clkr,
	[UART2DM_P_CLK] = &uart2dm_p_clk.clkr,
	/* SDC clocks */
	[SDC1_CLK] = &sdc1_clk.clkr,
	[SDC1_P_CLK] = &sdc1_p_clk.clkr,
	[SDC2_CLK] = &sdc2_clk.clkr,
	[SDC2_P_CLK] = &sdc2_p_clk.clkr,
	[SDC3_CLK] = &sdc3_clk.clkr,
	[SDC3_P_CLK] = &sdc3_p_clk.clkr,
	[SDC4_CLK] = &sdc4_clk.clkr,
	[SDC4_P_CLK] = &sdc4_p_clk.clkr,
	/* USB clocks */
	[USB_HS_CLK] = &usb_hs_clk.clkr,
	[USB_HS_CORE_CLK] = &usb_hs_core_clk.clkr,
	[USB_HS_P_CLK] = &usb_hs_p_clk.clkr,
	/* Display clocks */
	[MDP_CLK] = &mdp_clk.clkr,
	[MDP_P_CLK] = &mdp_p_clk.clkr,
	[MDP_LCDC_PCLK_CLK] = &mdp_lcdc_pclk_clk.clkr,
	[MDP_LCDC_PAD_PCLK_CLK] = &mdp_lcdc_pad_pclk_clk.clkr,
	[MDP_VSYNC_CLK] = &mdp_vsync_clk.clkr,
	[PMDH_CLK] = &pmdh_clk.clkr,
	[PMDH_P_CLK] = &pmdh_p_clk.clkr,
	/* Graphics clocks */
	[GRP_2D_CLK] = &grp_2d_clk.clkr,
	[GRP_2D_P_CLK] = &grp_2d_p_clk.clkr,
	[GRP_3D_CLK] = &grp_3d_clk.clkr,
	[GRP_3D_P_CLK] = &grp_3d_p_clk.clkr,
	[IMEM_CLK] = &imem_clk.clkr,
	/* Camera clocks */
	[CAM_M_CLK] = &cam_m_clk.clkr,
	[VFE_CLK] = &vfe_clk.clkr,
	[VFE_P_CLK] = &vfe_p_clk.clkr,
	/* DMA/Rotator clocks */
	[ADM_CLK] = &adm_clk.clkr,
	[ROTATOR_IMEM_CLK] = &rotator_imem_clk.clkr,
	[ROTATOR_P_CLK] = &rotator_p_clk.clkr,
};

static const struct regmap_config gcc_msm7x30_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x500,
	.fast_io = true,
};

static const struct qcom_cc_desc gcc_msm7x30_desc = {
	.config = &gcc_msm7x30_regmap_config,
	.clks = gcc_msm7x30_clks,
	.num_clks = ARRAY_SIZE(gcc_msm7x30_clks),
};

static int gcc_msm7x30_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(&pdev->dev, base,
				       &gcc_msm7x30_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Register fixed-rate PLLs */
	ret = devm_clk_hw_register(&pdev->dev, &pll1_clk.hw);
	if (ret)
		return ret;

	ret = devm_clk_hw_register(&pdev->dev, &pll3_clk.hw);
	if (ret)
		return ret;

	ret = devm_clk_hw_register(&pdev->dev, &pll4_clk.hw);
	if (ret)
		return ret;

	return qcom_cc_really_probe(&pdev->dev, &gcc_msm7x30_desc, regmap);
}

static const struct of_device_id gcc_msm7x30_match_table[] = {
	{ .compatible = "qcom,gcc-msm7x30" },
	{ }
};
MODULE_DEVICE_TABLE(of, gcc_msm7x30_match_table);

static struct platform_driver gcc_msm7x30_driver = {
	.probe = gcc_msm7x30_probe,
	.driver = {
		.name = "gcc-msm7x30",
		.of_match_table = gcc_msm7x30_match_table,
	},
};

static int __init gcc_msm7x30_init(void)
{
	return platform_driver_register(&gcc_msm7x30_driver);
}
core_initcall(gcc_msm7x30_init);

static void __exit gcc_msm7x30_exit(void)
{
	platform_driver_unregister(&gcc_msm7x30_driver);
}
module_exit(gcc_msm7x30_exit);

MODULE_DESCRIPTION("Qualcomm MSM7x30 Global Clock Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gcc-msm7x30");
