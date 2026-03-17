// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Herrie <pingu@pingudev.org>
 *
 * APQ8060/MSM8660 ASoC Machine driver using direct LPAIF hardware
 *
 * This machine driver connects the LPAIF CPU DAI to external codecs
 * (e.g., WM8958 on HP TouchPad) without going through the Q6 DSP.
 * This is similar to how the original webOS kernel handled audio.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <uapi/linux/input-event-codes.h>

#include "common.h"

#define DRIVER_NAME "apq8060-lpaif"

struct apq8060_lpaif_data {
	struct snd_soc_card card;
	struct snd_soc_jack jack;
	struct gpio_desc *spkr_amp_gpio;
	bool jack_setup;
};

static int apq8060_lpaif_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int rate = params_rate(params);
	unsigned int sysclk_rate;
	int ret;

	/*
	 * Set sysclk rate based on sample rate.
	 * Standard oversampling ratio is 256x.
	 */
	sysclk_rate = rate * 256;

	dev_dbg(rtd->dev, "hw_params: rate=%u, sysclk=%u\n", rate, sysclk_rate);

	/* Set CPU DAI sysclk */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk_rate, SND_SOC_CLOCK_OUT);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	/* Set codec sysclk if codec supports it */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk_rate, SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set codec sysclk: %d\n", ret);
		return ret;
	}

	return 0;
}

static int apq8060_lpaif_spkr_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct apq8060_lpaif_data *data = snd_soc_card_get_drvdata(card);

	if (!data->spkr_amp_gpio)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		gpiod_set_value_cansleep(data->spkr_amp_gpio, 1);
	else
		gpiod_set_value_cansleep(data->spkr_amp_gpio, 0);

	return 0;
}

static const struct snd_soc_dapm_widget apq8060_lpaif_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", apq8060_lpaif_spkr_event),
};

static const struct snd_soc_ops apq8060_lpaif_ops = {
	.hw_params = apq8060_lpaif_hw_params,
};

static int apq8060_lpaif_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct apq8060_lpaif_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *component = codec_dai->component;
	int ret;

	dev_info(card->dev, "LPAIF DAI init for %s\n", rtd->dai_link->name);

	/* Set up headphone jack detection if not already done */
	if (!data->jack_setup) {
		ret = snd_soc_card_jack_new(card, "Headset Jack",
					    SND_JACK_HEADSET |
					    SND_JACK_BTN_0 | SND_JACK_BTN_1 |
					    SND_JACK_BTN_2,
					    &data->jack);
		if (ret) {
			dev_err(card->dev, "Failed to create jack: %d\n", ret);
			return ret;
		}

		snd_jack_set_key(data->jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
		snd_jack_set_key(data->jack.jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
		snd_jack_set_key(data->jack.jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);

		/* Connect codec to jack if supported */
		ret = snd_soc_component_set_jack(component, &data->jack, NULL);
		if (ret && ret != -ENOTSUPP) {
			dev_warn(card->dev, "Failed to set codec jack: %d\n", ret);
		}

		data->jack_setup = true;
	}

	return 0;
}

static void apq8060_lpaif_add_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		link->ops = &apq8060_lpaif_ops;
		link->init = apq8060_lpaif_dai_init;
	}
}

static int apq8060_lpaif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apq8060_lpaif_data *data;
	struct snd_soc_card *card;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card = &data->card;
	card->owner = THIS_MODULE;
	card->dev = dev;
	card->dapm_widgets = apq8060_lpaif_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(apq8060_lpaif_dapm_widgets);

	snd_soc_card_set_drvdata(card, data);

	/* Get speaker amplifier GPIO if present */
	data->spkr_amp_gpio = devm_gpiod_get_optional(dev, "spkr-amp",
						      GPIOD_OUT_LOW);
	if (IS_ERR(data->spkr_amp_gpio)) {
		ret = PTR_ERR(data->spkr_amp_gpio);
		dev_err(dev, "failed to get spkr-amp gpio: %d\n", ret);
		return ret;
	}

	/* Parse audio routing from device tree */
	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret) {
		dev_err(dev, "failed to parse card name: %d\n", ret);
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret) {
		dev_err(dev, "failed to parse audio routing: %d\n", ret);
		return ret;
	}

	/* Parse DAI links from device tree */
	ret = qcom_snd_parse_of(card);
	if (ret) {
		dev_err(dev, "failed to parse DAI links: %d\n", ret);
		return ret;
	}

	/* Add ops to all DAI links */
	apq8060_lpaif_add_ops(card);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret) {
		dev_err(dev, "failed to register sound card: %d\n", ret);
		return ret;
	}

	dev_info(dev, "APQ8060 LPAIF audio initialized\n");
	return 0;
}

static const struct of_device_id apq8060_lpaif_device_id[] = {
	{ .compatible = "qcom,apq8060-lpaif-sndcard" },
	{ .compatible = "qcom,msm8660-lpaif-sndcard" },
	{}
};
MODULE_DEVICE_TABLE(of, apq8060_lpaif_device_id);

static struct platform_driver apq8060_lpaif_driver = {
	.probe = apq8060_lpaif_probe,
	.driver = {
		.name = "apq8060-lpaif-sndcard",
		.of_match_table = apq8060_lpaif_device_id,
	},
};
module_platform_driver(apq8060_lpaif_driver);

MODULE_DESCRIPTION("APQ8060/MSM8660 LPAIF ASoC Machine driver");
MODULE_LICENSE("GPL");
