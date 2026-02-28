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
#include <linux/mfd/wm8994/registers.h>

#include "common.h"
#include "../codecs/wm8994.h"

#define DRIVER_NAME "apq8060"

/* Audio clock rates */
#define WM_FS		48000
#define WM_CHANNELS	2
#define WM_BITS		16
#define WM_FLL_MULT	8	/* 2*16*8 = 256, clock rates must be >= 256*fs */
#define WM_BCLK		(WM_FS * WM_CHANNELS * WM_BITS)	/* 1.536MHz */
#define WM_FLL		(WM_FLL_MULT * WM_BCLK)		/* 12.288MHz */

struct apq8060_snd_data {
	struct snd_soc_card card;
	struct snd_soc_jack hp_jack;
	struct gpio_desc *hp_jack_gpio;
	/* FLL configuration tracking */
	unsigned int fll_rate;
	unsigned int bclk_rate;
	int fll_id;
	int fll_sysclk;
};

/*
 * Speaker amplifier enable via WM8994 GPIO_1.
 * The TouchPad uses an external Class-D amplifier controlled by this GPIO.
 * GPIO_1 = 0x41 enables the amp, 0x01 disables it.
 */
static int apq8060_spk_pwr_amp(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		dev_dbg(component->dev, "Enabling speaker amplifier\n");
		snd_soc_component_write(component, WM8994_GPIO_1, 0x41);
	} else {
		dev_dbg(component->dev, "Disabling speaker amplifier\n");
		snd_soc_component_write(component, WM8994_GPIO_1, 0x01);
	}
	return 0;
}

/* DAPM widgets for HP TouchPad */
static const struct snd_soc_dapm_widget apq8060_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", apq8060_spk_pwr_amp),
};

/* DAPM routes for HP TouchPad - speakers use LINEOUT */
static const struct snd_soc_dapm_route apq8060_dapm_routes[] = {
	{ "Headphone", NULL, "HPOUT1L" },
	{ "Headphone", NULL, "HPOUT1R" },

	{ "Speaker", NULL, "LINEOUT1P" },
	{ "Speaker", NULL, "LINEOUT1N" },
	{ "Speaker", NULL, "LINEOUT2P" },
	{ "Speaker", NULL, "LINEOUT2N" },

	/* Internal Mic via MICBIAS1 */
	{ "Internal Mic", NULL, "MICBIAS1" },
	{ "IN1LN", NULL, "Internal Mic" },

	/* Headset Mic via MICBIAS2 */
	{ "Headset Mic", NULL, "MICBIAS2" },
	{ "IN2LN", NULL, "Headset Mic" },
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
	struct snd_soc_card *card = rtd->card;
	struct apq8060_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int rate = params_rate(params);
	int ret;

	dev_info(rtd->dev, "APQ8060: hw_params rate=%u, stream=%s\n",
		 rate, substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		 "playback" : "capture");

	/*
	 * Set DAI format - I2S, CPU/DSP provides bit/frame clocks.
	 * The Q6 DSP via MI2S is the clock master, providing BCLK and LRCLK
	 * to the WM8958 codec which uses BCLK for its FLL.
	 */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_BP_FP);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "Failed to set CPU DAI format: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBC_CFC);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "Failed to set codec DAI format: %d\n", ret);
		return ret;
	}

	/*
	 * Configure WM8994 FLL using BCLK from Q6 DSP.
	 * The DSP provides BCLK = rate * channels * bits = rate * 2 * 16.
	 * FLL output must be >= 256 * fs and between 4.096MHz - 12.5MHz.
	 *
	 * Note: BCLK may not be present yet during hw_params (it starts in
	 * prepare when the Q6 AFE port starts). However, we must configure
	 * FLL and SYSCLK here because the codec's hw_params requires aifclk
	 * to be set. The FLL will lock when BCLK becomes available.
	 *
	 * Use FLL1 for playback (AIF1), FLL2 for capture (AIF2).
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		data->fll_id = WM8994_FLL1;
		data->fll_sysclk = WM8994_SYSCLK_FLL1;
	} else {
		data->fll_id = WM8994_FLL2;
		data->fll_sysclk = WM8994_SYSCLK_FLL2;
	}

	/* Calculate BCLK rate - always use stereo for proper clock ratio */
	data->bclk_rate = rate * WM_CHANNELS * WM_BITS;
	data->fll_rate = data->bclk_rate * WM_FLL_MULT;

	/* Ensure FLL rate is at least 4.096MHz */
	if (data->fll_rate < 4096000)
		data->fll_rate = 4096000;

	dev_info(rtd->dev, "APQ8060: Setting FLL%d from BCLK=%u to %u Hz\n",
		 data->fll_id == WM8994_FLL1 ? 1 : 2, data->bclk_rate, data->fll_rate);

	ret = snd_soc_dai_set_pll(codec_dai, data->fll_id, WM8994_FLL_SRC_BCLK,
				  data->bclk_rate, data->fll_rate);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "Failed to set FLL%d: %d\n",
			data->fll_id == WM8994_FLL1 ? 1 : 2, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, data->fll_sysclk, data->fll_rate, 0);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "Failed to set sysclk: %d\n", ret);
		return ret;
	}

	dev_info(rtd->dev, "APQ8060: Codec clock configured (FLL will lock when BCLK available)\n");
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

	/* Only initialize on the first backend link
	 * Note: We use SECONDARY_MI2S which maps to legacy PRIMARY_I2S
	 * ports (0/1) for the WM8958 codec on GPIO 108/109.
	 */
	if (cpu_dai->id != SECONDARY_MI2S_RX)
		return 0;

	/* Disable unused codec pins - TouchPad doesn't use internal speaker outputs */
	snd_soc_dapm_nc_pin(&card->dapm, "SPKOUTRN");
	snd_soc_dapm_nc_pin(&card->dapm, "SPKOUTRP");
	snd_soc_dapm_nc_pin(&card->dapm, "SPKOUTLN");
	snd_soc_dapm_nc_pin(&card->dapm, "SPKOUTLP");
	snd_soc_dapm_nc_pin(&card->dapm, "HPOUT2P");
	snd_soc_dapm_nc_pin(&card->dapm, "HPOUT2N");
	snd_soc_dapm_nc_pin(&card->dapm, "IN2RP:VXRP");
	snd_soc_dapm_nc_pin(&card->dapm, "IN2RN");
	snd_soc_dapm_nc_pin(&card->dapm, "IN1RN");
	snd_soc_dapm_nc_pin(&card->dapm, "IN1RP");

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

	dev_info(card->dev, "APQ8060 audio initialized\n");
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
	card->dapm_routes = apq8060_dapm_routes;
	card->num_dapm_routes = ARRAY_SIZE(apq8060_dapm_routes);
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
