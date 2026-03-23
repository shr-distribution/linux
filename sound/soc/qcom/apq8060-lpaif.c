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

#include "../codecs/wm8994.h"
#include "common.h"

#define DRIVER_NAME "apq8060-lpaif"

/* Include WM8994 register definitions for direct register access */
#include <linux/mfd/wm8994/registers.h>

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
	struct snd_soc_component *component = codec_dai->component;
	unsigned int rate = params_rate(params);
	unsigned int sysclk_rate;
	unsigned int bclk_rate;
	int ret;

	/*
	 * Configure I2S master/slave mode - CPU MASTER MODE.
	 *
	 * LPASS generates BCLK and LRCLK, codec receives them.
	 * GPIO 108 is used for LRCLK output in I2S mode.
	 *
	 * Note: GPIO 108 is also used for LDO2 enable, but since DCVDD
	 * is supplied externally by pm8058_s3, LDO2 isn't needed for
	 * codec power. The wm8994-ldo driver sets always_on=1 for LDO2
	 * to prevent the regulator framework from trying to disable it.
	 */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_BP_FP);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_BC_FC);
	dev_info(rtd->dev, "codec set_fmt returned: %d\n", ret);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set codec dai format: %d\n", ret);
		return ret;
	}

	/*
	 * Set sysclk rate based on sample rate.
	 * For 48kHz family: 12288000 Hz (256 * 48000)
	 * For 44.1kHz family: 11289600 Hz (256 * 44100)
	 */
	sysclk_rate = rate * 256;

	/* Ensure minimum sysclk of 4.096MHz */
	if (sysclk_rate < 4096000)
		sysclk_rate = 4096000;

	dev_info(rtd->dev, "hw_params: rate=%u, sysclk=%u\n", rate, sysclk_rate);

	/* Set CPU DAI sysclk */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk_rate, SND_SOC_CLOCK_OUT);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	/*
	 * Configure WM8994/WM8958 FLL1 using BCLK from LPASS.
	 *
	 * In CPU master mode, LPASS generates BCLK. The codec receives BCLK
	 * and uses it as the FLL reference to generate its internal SYSCLK.
	 * This ensures the codec is synchronized with LPASS clocks.
	 *
	 * BCLK rate = sample_rate * channels * bits_per_sample
	 * For 48kHz stereo 16-bit: 48000 * 2 * 16 = 1,536,000 Hz
	 */
	bclk_rate = rate * params_channels(params) *
		    snd_pcm_format_width(params_format(params));

	dev_info(rtd->dev, "FLL using BCLK=%u Hz from LPASS, output sysclk=%u Hz\n",
		 bclk_rate, sysclk_rate);

	ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, WM8994_FLL_SRC_BCLK,
				  bclk_rate, sysclk_rate);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set codec FLL: %d\n", ret);
		return ret;
	}

	/* Set codec sysclk to use FLL1 output */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1,
				     sysclk_rate, SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "failed to set codec sysclk: %d\n", ret);
		return ret;
	}

	/*
	 * Configure BCLK-only mode: Force internal LRCLK generation.
	 *
	 * Since GPIO 108 must stay HIGH for LDO2, we can't use it for LRCLK.
	 * Instead, configure the codec to derive LRCLK timing from BCLK:
	 *
	 * 1. Set AIF1_LRCLK_FRC bit to force internal LRCLK generation
	 * 2. Set AIF1DAC_LRCLK rate = BCLK cycles per frame (e.g., 32 for 16-bit stereo)
	 *
	 * The codec counts BCLK cycles and generates internal frame strobes,
	 * eliminating the need for an external LRCLK signal.
	 */
	{
		unsigned int lrclk_rate;
		unsigned int channels = params_channels(params);
		unsigned int width = snd_pcm_format_width(params_format(params));

		/* LRCLK rate = bits per channel * channels = bits per frame */
		lrclk_rate = width * channels;

		dev_info(rtd->dev, "BCLK-only mode: LRCLK_FRC=1, rate=%u (width=%u, ch=%u)\n",
			 lrclk_rate, width, channels);

		/* Set AIF1_LRCLK_FRC to force internal LRCLK generation */
		snd_soc_component_update_bits(component, WM8994_AIF1_CONTROL_1,
					      WM8994_AIF1_LRCLK_FRC,
					      WM8994_AIF1_LRCLK_FRC);

		/* Set AIF1DAC LRCLK rate (BCLK cycles per frame) */
		snd_soc_component_update_bits(component, WM8994_AIF1DAC_LRCLK,
					      WM8994_AIF1DAC_RATE_MASK,
					      lrclk_rate);

		/* Also set AIF1ADC LRCLK rate for symmetry */
		snd_soc_component_update_bits(component, WM8994_AIF1ADC_LRCLK,
					      WM8994_AIF1DAC_RATE_MASK,
					      lrclk_rate);
	}

	/*
	 * Force-enable power management registers for the output path.
	 * DAPM may not properly power up all widgets because the path
	 * detection doesn't always work correctly. Enable them here
	 * during hw_params so they're ready when playback starts.
	 *
	 * TouchPad uses LINEOUT (not SPKOUT) connected to external amp.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev_info(rtd->dev, "Enabling output power registers\n");

		/* PM5: Enable DAC1 left and right */
		snd_soc_component_update_bits(component, WM8994_POWER_MANAGEMENT_5,
					      WM8994_DAC1L_ENA | WM8994_DAC1R_ENA,
					      WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);

		/* PM3: Enable LINEOUT drivers and output mixers */
		snd_soc_component_update_bits(component, WM8994_POWER_MANAGEMENT_3,
					      WM8994_LINEOUT1P_ENA | WM8994_LINEOUT1N_ENA |
					      WM8994_LINEOUT2P_ENA | WM8994_LINEOUT2N_ENA |
					      WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA,
					      WM8994_LINEOUT1P_ENA | WM8994_LINEOUT1N_ENA |
					      WM8994_LINEOUT2P_ENA | WM8994_LINEOUT2N_ENA |
					      WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA);

		/* Enable DAC1 to output mixer paths */
		snd_soc_component_update_bits(component, WM8994_OUTPUT_MIXER_1,
					      WM8994_DAC1L_TO_MIXOUTL,
					      WM8994_DAC1L_TO_MIXOUTL);
		snd_soc_component_update_bits(component, WM8994_OUTPUT_MIXER_2,
					      WM8994_DAC1R_TO_MIXOUTR,
					      WM8994_DAC1R_TO_MIXOUTR);

		/* Enable output mixer to LINEOUT paths */
		snd_soc_component_update_bits(component, WM8994_LINE_MIXER_1,
					      WM8994_MIXOUTL_TO_LINEOUT1P,
					      WM8994_MIXOUTL_TO_LINEOUT1P);
		snd_soc_component_update_bits(component, WM8994_LINE_MIXER_2,
					      WM8994_MIXOUTR_TO_LINEOUT2P,
					      WM8994_MIXOUTR_TO_LINEOUT2P);

		/* Enable AIF1 to DAC1 path */
		snd_soc_component_update_bits(component, WM8994_DAC1_LEFT_MIXER_ROUTING,
					      WM8994_AIF1DAC1L_TO_DAC1L,
					      WM8994_AIF1DAC1L_TO_DAC1L);
		snd_soc_component_update_bits(component, WM8994_DAC1_RIGHT_MIXER_ROUTING,
					      WM8994_AIF1DAC1R_TO_DAC1R,
					      WM8994_AIF1DAC1R_TO_DAC1R);
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
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", apq8060_lpaif_spkr_event),
};

/*
 * Inter-link routes connecting CPU DAI streams to codec DAI streams.
 *
 * The LPAIF CPU DAI uses stream names like "Codec Speaker Playback"
 * while the WM8994 codec uses "AIF1 Playback". DAPM needs explicit
 * routes to connect these different stream names.
 */
static const struct snd_soc_dapm_route apq8060_lpaif_dapm_routes[] = {
	/* Connect LPAIF playback to WM8994 AIF1 */
	{ "AIF1 Playback", NULL, "Codec Speaker Playback" },
	/* Connect WM8994 AIF1 capture to LPAIF */
	{ "Codec Mic Capture", NULL, "AIF1 Capture" },
};

static const struct snd_soc_ops apq8060_lpaif_ops = {
	.hw_params = apq8060_lpaif_hw_params,
};

static int apq8060_lpaif_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct apq8060_lpaif_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_component *component = codec_dai->component;
	int ret;

	dev_info(card->dev, "LPAIF DAI init for %s\n", rtd->dai_link->name);

	/* Set up headphone jack detection if not already done */
	if (!data->jack_setup) {
		ret = snd_soc_card_jack_new(card, "Headphone",
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
	card->dapm_routes = apq8060_lpaif_dapm_routes;
	card->num_dapm_routes = ARRAY_SIZE(apq8060_lpaif_dapm_routes);

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
