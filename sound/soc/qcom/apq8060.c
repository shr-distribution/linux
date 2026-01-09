// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, The Linux Foundation. All rights reserved.
 *
 * APQ8060 ASoC Machine driver for HP TouchPad (Tenderloin)
 *
 * This driver bridges the QDSP6 audio subsystem (Q6AFE/Q6ASM) with
 * the WM8958 codec on APQ8060/MSM8660 devices like HP TouchPad.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <uapi/linux/input-event-codes.h>
#include <dt-bindings/sound/qcom,q6afe.h>

#include "common.h"

#define DRIVER_NAME "apq8060"

struct apq8060_snd_data {
	struct snd_soc_card card;
	struct snd_soc_jack hp_jack;
	struct gpio_desc *hp_jack_gpio;
};

/* DAPM widgets for HP TouchPad */
static const struct snd_soc_dapm_widget apq8060_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* Headphone jack pins */
static struct snd_soc_jack_pin apq8060_hp_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static int apq8060_snd_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void apq8060_snd_shutdown(struct snd_pcm_substream *substream)
{
}

static int apq8060_snd_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	/* Set codec DAI format - I2S, codec is bit/frame clock consumer */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBC_CFC);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "Failed to set codec DAI format: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_soc_ops apq8060_snd_ops = {
	.startup = apq8060_snd_startup,
	.shutdown = apq8060_snd_shutdown,
	.hw_params = apq8060_snd_hw_params,
};

static int apq8060_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/* Set fixed rate for backend - 48kHz stereo */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static int apq8060_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct apq8060_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	/* Only initialize jack on the first backend link */
	if (cpu_dai->id != PRIMARY_MI2S_RX)
		return 0;

	/* Setup headphone jack */
	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack",
					 SND_JACK_HEADSET |
					 SND_JACK_BTN_0 | SND_JACK_BTN_1 |
					 SND_JACK_BTN_2 | SND_JACK_BTN_3,
					 &data->hp_jack,
					 apq8060_hp_jack_pins,
					 ARRAY_SIZE(apq8060_hp_jack_pins));
	if (ret) {
		dev_err(card->dev, "Failed to create HP jack: %d\n", ret);
		return ret;
	}

	snd_jack_set_key(data->hp_jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(data->hp_jack.jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(data->hp_jack.jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(data->hp_jack.jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	return 0;
}

static void apq8060_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			/* Backend link */
			link->init = apq8060_init;
			link->be_hw_params_fixup = apq8060_be_hw_params_fixup;
			link->ops = &apq8060_snd_ops;
		}
	}
}

static int apq8060_snd_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct apq8060_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->driver_name = DRIVER_NAME;
	card->dapm_widgets = apq8060_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(apq8060_dapm_widgets);
	card->dev = dev;
	card->owner = THIS_MODULE;

	dev_set_drvdata(dev, card);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	snd_soc_card_set_drvdata(card, data);

	apq8060_add_be_ops(card);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id apq8060_snd_device_id[] = {
	{ .compatible = "qcom,apq8060-sndcard" },
	{ .compatible = "qcom,msm8660-sndcard" },
	{},
};
MODULE_DEVICE_TABLE(of, apq8060_snd_device_id);

static struct platform_driver apq8060_snd_driver = {
	.probe = apq8060_snd_platform_probe,
	.driver = {
		.name = "apq8060-sndcard",
		.of_match_table = apq8060_snd_device_id,
	},
};
module_platform_driver(apq8060_snd_driver);

MODULE_AUTHOR("Claude Code");
MODULE_DESCRIPTION("APQ8060/MSM8660 ASoC Machine driver");
MODULE_LICENSE("GPL");
