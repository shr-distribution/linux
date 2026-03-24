// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Herrie <pingu@pingudev.org>
 *
 * lpass-msm8660.c -- ALSA SoC CPU DAI driver for MSM8660/APQ8060 LPASS
 *
 * This driver provides direct hardware access to the LPAIF (Low Power
 * Audio Interface) on MSM8660/APQ8060 SoCs, bypassing the Q6 DSP.
 * This approach matches how the original webOS kernel handled audio.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include "lpass-lpaif-reg.h"
#include "lpass.h"

/*
 * MSM8660/APQ8060 LPAIF I2S port definitions
 *
 * The LPAIF has 5 I2S ports:
 *   0: Primary codec speaker (CODEC_SPKR)
 *   1: Primary codec mic (CODEC_MIC)
 *   2: MI2S (multichannel)
 *   3: Secondary speaker
 *   4: Secondary mic
 */
enum msm8660_lpaif_i2s_ports {
	MSM8660_LPAIF_I2S_PORT_CODEC_SPKR,
	MSM8660_LPAIF_I2S_PORT_CODEC_MIC,
	MSM8660_LPAIF_I2S_PORT_MI2S,
	MSM8660_LPAIF_I2S_PORT_SEC_SPKR,
	MSM8660_LPAIF_I2S_PORT_SEC_MIC,
	MSM8660_LPAIF_I2S_PORT_MAX,
};

/*
 * MSM8660/APQ8060 LPAIF DMA channel definitions
 *
 * Channels 0-4 are for playback (RDMA)
 * Channels 5-7 are for capture (WRDMA)
 */
enum msm8660_lpaif_dma_channels {
	MSM8660_LPAIF_RDMA_CHAN_0,
	MSM8660_LPAIF_RDMA_CHAN_1,
	MSM8660_LPAIF_RDMA_CHAN_2,
	MSM8660_LPAIF_RDMA_CHAN_3,
	MSM8660_LPAIF_RDMA_CHAN_4,
	MSM8660_LPAIF_WRDMA_CHAN_5,
	MSM8660_LPAIF_WRDMA_CHAN_6,
	MSM8660_LPAIF_WRDMA_CHAN_7,
};

/*
 * MSM8660 has separate clocks for codec speaker and codec mic I2S ports.
 * We support both playback (speaker) and capture (mic).
 */
static struct snd_soc_dai_driver msm8660_lpass_cpu_dai_driver[] = {
	{
		.id = MSM8660_LPAIF_I2S_PORT_CODEC_SPKR,
		.name = "codec-spkr",
		.playback = {
			.stream_name = "Codec Speaker Playback",
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.rate_min = 8000,
			.rate_max = 96000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.ops = &asoc_qcom_lpass_cpu_dai_ops,
	},
	{
		.id = MSM8660_LPAIF_I2S_PORT_CODEC_MIC,
		.name = "codec-mic",
		.capture = {
			.stream_name = "Codec Mic Capture",
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.rate_min = 8000,
			.rate_max = 96000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.ops = &asoc_qcom_lpass_cpu_dai_ops,
	},
	{
		.id = MSM8660_LPAIF_I2S_PORT_SEC_SPKR,
		.name = "sec-spkr",
		.playback = {
			.stream_name = "Secondary Speaker Playback",
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.rate_min = 8000,
			.rate_max = 96000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.ops = &asoc_qcom_lpass_cpu_dai_ops,
	},
	{
		.id = MSM8660_LPAIF_I2S_PORT_SEC_MIC,
		.name = "sec-mic",
		.capture = {
			.stream_name = "Secondary Mic Capture",
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.rate_min = 8000,
			.rate_max = 96000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.ops = &asoc_qcom_lpass_cpu_dai_ops,
	},
};

static int msm8660_lpass_alloc_dma_channel(struct lpass_data *drvdata,
					   int direction, unsigned int dai_id)
{
	const struct lpass_variant *v = drvdata->variant;
	int chan;

	if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * Playback uses RDMA channels 0-4.
		 * Map I2S port to corresponding DMA channel.
		 */
		switch (dai_id) {
		case MSM8660_LPAIF_I2S_PORT_CODEC_SPKR:
			chan = MSM8660_LPAIF_RDMA_CHAN_0;
			break;
		case MSM8660_LPAIF_I2S_PORT_SEC_SPKR:
			chan = MSM8660_LPAIF_RDMA_CHAN_3;
			break;
		default:
			pr_err("msm8660-lpass: invalid dai_id %u for playback\n",
			       dai_id);
			return -EINVAL;
		}

		if (test_and_set_bit(chan, &drvdata->dma_ch_bit_map)) {
			pr_err("msm8660-lpass: DMA channel %d already in use\n",
			       chan);
			return -EBUSY;
		}
	} else {
		/*
		 * Capture uses WRDMA channels 5-7.
		 * Channel 5 corresponds to wrdma_channel_start.
		 */
		switch (dai_id) {
		case MSM8660_LPAIF_I2S_PORT_CODEC_MIC:
			chan = v->wrdma_channel_start;
			break;
		case MSM8660_LPAIF_I2S_PORT_SEC_MIC:
			chan = v->wrdma_channel_start + 1;
			break;
		default:
			pr_err("msm8660-lpass: invalid dai_id %u for capture\n",
			       dai_id);
			return -EINVAL;
		}

		if (test_and_set_bit(chan, &drvdata->dma_ch_bit_map)) {
			pr_err("msm8660-lpass: DMA channel %d already in use\n",
			       chan);
			return -EBUSY;
		}
	}

	return chan;
}

static int msm8660_lpass_free_dma_channel(struct lpass_data *drvdata,
					  int chan, unsigned int dai_id)
{
	clear_bit(chan, &drvdata->dma_ch_bit_map);
	return 0;
}

static int msm8660_lpass_init(struct platform_device *pdev)
{
	struct lpass_data *drvdata = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret, i;

	/* Get OSR and bit clocks for each I2S port */
	for (i = 0; i < drvdata->variant->num_dai; i++) {
		const char *osr_name = drvdata->variant->dai_osr_clk_names[i];
		const char *bit_name = drvdata->variant->dai_bit_clk_names[i];

		if (osr_name) {
			drvdata->mi2s_osr_clk[i] = devm_clk_get(dev, osr_name);
			if (IS_ERR(drvdata->mi2s_osr_clk[i])) {
				dev_err(dev, "error getting %s: %ld\n",
					osr_name,
					PTR_ERR(drvdata->mi2s_osr_clk[i]));
				return PTR_ERR(drvdata->mi2s_osr_clk[i]);
			}
		}

		if (bit_name) {
			drvdata->mi2s_bit_clk[i] = devm_clk_get(dev, bit_name);
			if (IS_ERR(drvdata->mi2s_bit_clk[i])) {
				dev_err(dev, "error getting %s: %ld\n",
					bit_name,
					PTR_ERR(drvdata->mi2s_bit_clk[i]));
				return PTR_ERR(drvdata->mi2s_bit_clk[i]);
			}
		}
	}

	/*
	 * Enable codec speaker OSR clock at 12.288 MHz.
	 * This is the master clock for audio at 48kHz sample rate.
	 * 12288000 / 48000 = 256 (standard oversampling ratio)
	 */
	if (drvdata->mi2s_osr_clk[MSM8660_LPAIF_I2S_PORT_CODEC_SPKR]) {
		ret = clk_set_rate(
			drvdata->mi2s_osr_clk[MSM8660_LPAIF_I2S_PORT_CODEC_SPKR],
			12288000);
		if (ret) {
			dev_err(dev, "error setting codec spkr osr rate: %d\n",
				ret);
			return ret;
		}

		ret = clk_prepare_enable(
			drvdata->mi2s_osr_clk[MSM8660_LPAIF_I2S_PORT_CODEC_SPKR]);
		if (ret) {
			dev_err(dev, "error enabling codec spkr osr clk: %d\n",
				ret);
			return ret;
		}
	}

	dev_info(dev, "MSM8660 LPASS initialized\n");
	return 0;
}

static int msm8660_lpass_exit(struct platform_device *pdev)
{
	struct lpass_data *drvdata = platform_get_drvdata(pdev);

	if (drvdata->mi2s_osr_clk[MSM8660_LPAIF_I2S_PORT_CODEC_SPKR])
		clk_disable_unprepare(
			drvdata->mi2s_osr_clk[MSM8660_LPAIF_I2S_PORT_CODEC_SPKR]);

	return 0;
}

/*
 * MSM8660/APQ8060 LPAIF variant data
 *
 * Register layout (from webOS audio_dma_msm8k.h):
 *   I2S control:  0x0004 + (0x04 * port)
 *   IRQ registers: 0x3000, stride 0x1000, 3 ports
 *   DMA registers: 0x6000, stride 0x1000, 8 channels
 *
 * Unlike newer SoCs, MSM8660 uses a single DMA register space for both
 * RDMA (playback) and WRDMA (capture), just with different channel numbers.
 */
static const struct lpass_variant msm8660_data = {
	/* I2S control registers */
	.i2sctrl_reg_base	= 0x0004,
	.i2sctrl_reg_stride	= 0x04,
	.i2s_ports		= MSM8660_LPAIF_I2S_PORT_MAX,

	/* IRQ registers */
	.irq_reg_base		= 0x3000,
	.irq_reg_stride		= 0x1000,
	.irq_ports		= 3,

	/* Read DMA (playback) - channels 0-4 */
	.rdma_reg_base		= 0x6000,
	.rdma_reg_stride	= 0x1000,
	.rdma_channels		= 5,

	/* Write DMA (capture) - channels 5-7, same register space */
	.wrdma_reg_base		= 0x6000 + (0x1000 * 5),
	.wrdma_reg_stride	= 0x1000,
	.wrdma_channels		= 3,
	.wrdma_channel_start	= 5,

	/*
	 * I2SCTL register fields (at offset 0x0004)
	 * Based on webOS audio_dma_msm8k.h bit definitions:
	 *   [15]    = loopback
	 *   [14]    = spk_en
	 *   [13:10] = spk_mode
	 *   [9]     = spk_mono
	 *   [8]     = mic_en
	 *   [7:4]   = mic_mode
	 *   [3]     = mic_mono
	 *   [2]     = ws_src (0=internal, 1=external)
	 *   [1:0]   = bit_width (0=16, 1=24, 2=32)
	 */
	.loopback	= REG_FIELD_ID(0x0004, 15, 15, 5, 0x04),
	.spken		= REG_FIELD_ID(0x0004, 14, 14, 5, 0x04),
	.spkmode	= REG_FIELD_ID(0x0004, 10, 13, 5, 0x04),
	.spkmono	= REG_FIELD_ID(0x0004,  9,  9, 5, 0x04),
	.micen		= REG_FIELD_ID(0x0004,  8,  8, 5, 0x04),
	.micmode	= REG_FIELD_ID(0x0004,  4,  7, 5, 0x04),
	.micmono	= REG_FIELD_ID(0x0004,  3,  3, 5, 0x04),
	.wssrc		= REG_FIELD_ID(0x0004,  2,  2, 5, 0x04),
	.bitwidth	= REG_FIELD_ID(0x0004,  0,  1, 5, 0x04),

	/*
	 * RDMA_CTL register fields (at offset 0x6000)
	 * Based on webOS audio_dma_msm8k.h:
	 *   [11]   = burst_en
	 *   [10:8] = wpscnt (words per sample count)
	 *   [7:4]  = audio_intf (interface select)
	 *   [3:1]  = fifo_watermrk
	 *   [0]    = enable
	 */
	.rdma_dyncclk	= REG_FIELD_ID(0x6000, 12, 12, 5, 0x1000),
	.rdma_bursten	= REG_FIELD_ID(0x6000, 11, 11, 5, 0x1000),
	.rdma_wpscnt	= REG_FIELD_ID(0x6000,  8, 10, 5, 0x1000),
	.rdma_intf	= REG_FIELD_ID(0x6000,  4,  7, 5, 0x1000),
	.rdma_fifowm	= REG_FIELD_ID(0x6000,  1,  3, 5, 0x1000),
	.rdma_enable	= REG_FIELD_ID(0x6000,  0,  0, 5, 0x1000),

	/* WRDMA_CTL register fields (channels 5-7) */
	.wrdma_dyncclk	= REG_FIELD_ID(0x6000 + (0x1000 * 5), 12, 12, 3, 0x1000),
	.wrdma_bursten	= REG_FIELD_ID(0x6000 + (0x1000 * 5), 11, 11, 3, 0x1000),
	.wrdma_wpscnt	= REG_FIELD_ID(0x6000 + (0x1000 * 5),  8, 10, 3, 0x1000),
	.wrdma_intf	= REG_FIELD_ID(0x6000 + (0x1000 * 5),  4,  7, 3, 0x1000),
	.wrdma_fifowm	= REG_FIELD_ID(0x6000 + (0x1000 * 5),  1,  3, 3, 0x1000),
	.wrdma_enable	= REG_FIELD_ID(0x6000 + (0x1000 * 5),  0,  0, 3, 0x1000),

	/* DAI drivers */
	.dai_driver		= msm8660_lpass_cpu_dai_driver,
	.num_dai		= ARRAY_SIZE(msm8660_lpass_cpu_dai_driver),

	/*
	 * Clock names for each I2S port.
	 * These match the clocks defined in LCC (qcom,lcc-msm8660).
	 */
	.dai_osr_clk_names	= (const char *[]) {
		"codec-i2s-spkr-osr-clk",	/* Port 0: codec speaker */
		"codec-i2s-mic-osr-clk",	/* Port 1: codec mic */
		"spare-i2s-spkr-osr-clk",	/* Port 2: secondary speaker */
		"spare-i2s-mic-osr-clk",	/* Port 3: secondary mic */
	},
	.dai_bit_clk_names	= (const char *[]) {
		"codec-i2s-spkr-bit-clk",	/* Port 0: codec speaker */
		"codec-i2s-mic-bit-clk",	/* Port 1: codec mic */
		"spare-i2s-spkr-bit-clk",	/* Port 2: secondary speaker */
		"spare-i2s-mic-bit-clk",	/* Port 3: secondary mic */
	},

	/* Callbacks */
	.init			= msm8660_lpass_init,
	.exit			= msm8660_lpass_exit,
	.alloc_dma_channel	= msm8660_lpass_alloc_dma_channel,
	.free_dma_channel	= msm8660_lpass_free_dma_channel,
};

static const struct of_device_id msm8660_lpass_cpu_device_id[] = {
	{ .compatible = "qcom,msm8660-lpass-cpu", .data = &msm8660_data },
	{ .compatible = "qcom,apq8060-lpass-cpu", .data = &msm8660_data },
	{}
};
MODULE_DEVICE_TABLE(of, msm8660_lpass_cpu_device_id);

static struct platform_driver msm8660_lpass_cpu_platform_driver = {
	.driver = {
		.name		= "msm8660-lpass-cpu",
		.of_match_table	= msm8660_lpass_cpu_device_id,
	},
	.probe	= asoc_qcom_lpass_cpu_platform_probe,
	.remove	= asoc_qcom_lpass_cpu_platform_remove,
	.shutdown = asoc_qcom_lpass_cpu_platform_shutdown,
};
module_platform_driver(msm8660_lpass_cpu_platform_driver);

MODULE_DESCRIPTION("QTi LPASS CPU Driver for MSM8660/APQ8060");
MODULE_LICENSE("GPL");
