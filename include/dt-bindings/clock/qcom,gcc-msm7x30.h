/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024, Linux community
 *
 * Clock definitions for Qualcomm MSM7x30 Global Clock Controller
 */

#ifndef _DT_BINDINGS_CLK_MSM7X30_GCC_H
#define _DT_BINDINGS_CLK_MSM7X30_GCC_H

/* PLLs */
#define PLL0				0
#define PLL1				1
#define PLL2				2
#define PLL3				3
#define PLL4				4
#define PLL5				5
#define PLL6				6

/* Core clocks */
#define ACPU_CLK			10
#define ADM_CLK				11
#define CE_CLK				12

/* I2C clocks */
#define I2C_CLK				20
#define I2C_P_CLK			21
#define I2C_2_CLK			22
#define I2C_2_P_CLK			23
#define QUP_I2C_CLK			24
#define QUP_I2C_P_CLK			25

/* UART clocks */
#define UART1_CLK			30
#define UART1_P_CLK			31
#define UART2_CLK			32
#define UART2_P_CLK			33
#define UART3_CLK			34
#define UART3_P_CLK			35
#define UART1DM_CLK			36
#define UART1DM_P_CLK			37
#define UART2DM_CLK			38
#define UART2DM_P_CLK			39

/* SDCC clocks */
#define SDC1_CLK			40
#define SDC1_P_CLK			41
#define SDC2_CLK			42
#define SDC2_P_CLK			43
#define SDC3_CLK			44
#define SDC3_P_CLK			45
#define SDC4_CLK			46
#define SDC4_P_CLK			47

/* USB clocks */
#define USB_HS_SRC_CLK			50
#define USB_HS_CLK			51
#define USB_HS_CORE_CLK			52
#define USB_HS_P_CLK			53
#define USB_HS2_CLK			54
#define USB_HS2_CORE_CLK		55
#define USB_HS2_P_CLK			56
#define USB_HS3_CLK			57
#define USB_HS3_CORE_CLK		58
#define USB_HS3_P_CLK			59

/* Display clocks */
#define MDP_CLK				60
#define MDP_P_CLK			61
#define MDP_LCDC_PCLK_CLK		62
#define MDP_LCDC_PAD_PCLK_CLK		63
#define MDP_VSYNC_CLK			64
#define PMDH_CLK			65
#define PMDH_P_CLK			66
#define EMDH_CLK			67
#define EMDH_P_CLK			68
#define TV_CLK				69
#define TV_DAC_CLK			70
#define TV_ENC_CLK			71
#define HDMI_CLK			72
#define MDC_CLK				73

/* Graphics clocks */
#define GRP_2D_CLK			80
#define GRP_2D_P_CLK			81
#define GRP_3D_SRC_CLK			82
#define GRP_3D_CLK			83
#define GRP_3D_P_CLK			84
#define IMEM_CLK			85

/* Camera/Video clocks */
#define VFE_CLK				90
#define VFE_P_CLK			91
#define VFE_MDC_CLK			92
#define VFE_CAMIF_CLK			93
#define CAMIF_PAD_P_CLK			94
#define CAM_M_CLK			95
#define JPEG_CLK			96
#define JPEG_P_CLK			97
#define VPE_CLK				98
#define MFC_CLK				99
#define MFC_DIV2_CLK			100
#define MFC_P_CLK			101
#define CSI0_CLK			102
#define CSI0_VFE_CLK			103
#define CSI0_P_CLK			104
#define CSI1_CLK			105
#define CSI1_VFE_CLK			106
#define CSI1_P_CLK			107

/* Rotator clocks */
#define ROTATOR_CLK			110
#define ROTATOR_IMEM_CLK		111
#define ROTATOR_P_CLK			112

/* Audio clocks */
#define MI2S_CODEC_RX_M_CLK		120
#define MI2S_CODEC_RX_S_CLK		121
#define MI2S_CODEC_TX_M_CLK		122
#define MI2S_CODEC_TX_S_CLK		123
#define MI2S_M_CLK			124
#define MI2S_S_CLK			125
#define LPA_CODEC_CLK			126
#define LPA_CORE_CLK			127
#define LPA_P_CLK			128
#define MIDI_CLK			129
#define SDAC_M_CLK			130
#define SDAC_CLK			131

/* SPI clocks */
#define SPI_CLK				140
#define SPI_P_CLK			141

/* TSIF clocks */
#define TSIF_REF_CLK			150
#define TSIF_P_CLK			151

/* AXI clocks */
#define AXI_LI_VG_CLK			160
#define AXI_LI_GRP_CLK			161
#define AXI_LI_JPEG_CLK			162
#define AXI_GRP_2D_CLK			163
#define AXI_MFC_CLK			164
#define AXI_VPE_CLK			165
#define AXI_LI_VFE_CLK			166
#define AXI_LI_APPS_CLK			167
#define AXI_MDP_CLK			168
#define AXI_IMEM_CLK			169
#define AXI_LI_ADSP_A_CLK		170
#define AXI_ROTATOR_CLK			171

/* Global root */
#define GLBL_ROOT_CLK			180

#endif /* _DT_BINDINGS_CLK_MSM7X30_GCC_H */
