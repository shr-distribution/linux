/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024, Christophe Chabanois <christophe.chabanois@gmail.com>
 */

#ifndef _DT_BINDINGS_CLK_LCC_MSM8660_H
#define _DT_BINDINGS_CLK_LCC_MSM8660_H

/*
 * MSM8660/APQ8060 LCC clock IDs
 * These are compatible with MSM8960 IDs but slimbus clocks are not available
 */

#define PLL4				0
#define MI2S_OSR_SRC			1
#define MI2S_OSR_CLK			2
#define MI2S_DIV_CLK			3
#define MI2S_BIT_DIV_CLK		4
#define MI2S_BIT_CLK			5
#define PCM_SRC				6
#define PCM_CLK_OUT			7
#define PCM_CLK				8
/* IDs 9-11 reserved (slimbus not available on MSM8660) */
#define CODEC_I2S_MIC_OSR_SRC		12
#define CODEC_I2S_MIC_OSR_CLK		13
#define CODEC_I2S_MIC_DIV_CLK		14
#define CODEC_I2S_MIC_BIT_DIV_CLK	15
#define CODEC_I2S_MIC_BIT_CLK		16
#define SPARE_I2S_MIC_OSR_SRC		17
#define SPARE_I2S_MIC_OSR_CLK		18
#define SPARE_I2S_MIC_DIV_CLK		19
#define SPARE_I2S_MIC_BIT_DIV_CLK	20
#define SPARE_I2S_MIC_BIT_CLK		21
#define CODEC_I2S_SPKR_OSR_SRC		22
#define CODEC_I2S_SPKR_OSR_CLK		23
#define CODEC_I2S_SPKR_DIV_CLK		24
#define CODEC_I2S_SPKR_BIT_DIV_CLK	25
#define CODEC_I2S_SPKR_BIT_CLK		26
#define SPARE_I2S_SPKR_OSR_SRC		27
#define SPARE_I2S_SPKR_OSR_CLK		28
#define SPARE_I2S_SPKR_DIV_CLK		29
#define SPARE_I2S_SPKR_BIT_DIV_CLK	30
#define SPARE_I2S_SPKR_BIT_CLK		31

#endif
