/*
 * herring.c: Audio for Nexus S
 *
 * Copyright (C) 2012 Wolfson Microelectronics, plc
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <sound/soc.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <mach/gpio.h>

#include "../codecs/wm8994.h"

#define HERRING_MCLK_RATE 24000000

static int herring_set_bias_level(struct snd_soc_card *card,
				  enum snd_soc_bias_level level)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	int ret;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, 0, 0, 0);
		if (ret < 0)
			return ret;
	default:
		break;
	}

	return 0;
}

static int herring_set_bias_level_post(struct snd_soc_card *card,
				       enum snd_soc_bias_level level)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	int ret;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		/* Clock from the FLL for audio; noop if already set */
		ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1,
					  WM8994_FLL_SRC_MCLK1,
					  HERRING_MCLK_RATE,
					  44100 * 256);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}

	card->dapm.bias_level = level;

	return 0;
}

static int herring_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct snd_soc_dai *cpu_dai = card->rtd[0].cpu_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "Failed to set format: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Failed to set format: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1,
				     44100 * 256,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	return 0;
}

static int herring_cpu_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* Clock from the FLL for audio; noop if already set */
	ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1,
				  WM8994_FLL_SRC_MCLK1,
				  HERRING_MCLK_RATE,
				  44100 * 256);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Failed to start FLL: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops herring_cpu_ops = {
	.hw_params = herring_cpu_hw_params,
};

static struct snd_soc_dai_link herring_dai[] = {
	{
		.name = "CPU",
		.stream_name = "CPU",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8994-aif1",
		.platform_name = "samsung-audio",
		.codec_name = "wm8994-codec",
		.ops = &herring_cpu_ops,
	},
};

static struct snd_soc_card herring = {
	.name = "Nexus S",
	.dai_link = herring_dai,
	.num_links = ARRAY_SIZE(herring_dai),

	.late_probe = herring_late_probe,

	.set_bias_level = herring_set_bias_level,
	.set_bias_level_post = herring_set_bias_level_post,
};

static int __devinit herring_audio_probe(struct platform_device *pdev)
{
	herring.dev = &pdev->dev;

	return snd_soc_register_card(&herring);
}

static int __devexit herring_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(platform_get_drvdata(pdev));

	return 0;
}

static struct platform_driver herring_audio = {
	.driver = {
		.name = "herring-audio",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = herring_audio_probe,
	.remove = __devexit_p(herring_audio_remove),
};

static int __init herring_module_init(void)
{
	return platform_driver_register(&herring_audio);
}
module_init(herring_module_init);

/* Module information */
MODULE_DESCRIPTION("Audio for Nexus S");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
