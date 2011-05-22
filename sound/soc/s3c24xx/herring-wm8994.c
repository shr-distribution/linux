/*
 * crespo_wm8994.c
 *
 * Copyright (C) 2010, Samsung Elect. Ltd. -
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/regs-clock.h>
#include <plat/regs-iis.h>
#include <mach/gpio.h>
#include <mach/gpio-herring.h>

#include "../codecs/wm8994.h"
#include "s3c-dma.h"
#include "s5pc1xx-i2s.h"
#include "s3c-i2s-v2.h"

#include <linux/io.h>

#define I2S_NUM 0
#define SRC_CLK 66738000

/* #define CONFIG_SND_DEBUG */
#ifdef CONFIG_SND_DEBUG
#define debug_msg(x...) printk(x)
#else
#define debug_msg(x...)
#endif

static const char *hp_analogue_text[] = {
	"Playback Mode", "VoiceCall Mode"
};

static const struct soc_enum hp_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_analogue_text), hp_analogue_text),
};

static int get_hp_output_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	printk(KERN_DEBUG "It doens't support get() func. user doesn't need to read\n");

	return 0;
}

static int set_hp_output_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	unsigned int mode = ucontrol->value.integer.value[0];

	debug_msg("set hp mode : %s\n", hp_analogue_text[mode]);

	gpio_set_value(GPIO_EAR_SEL, mode);

	return 0;
}

static const struct snd_kcontrol_new herring_controls[] = {
	SOC_ENUM_EXT("HP Output Mode", hp_mode_enum[0],
			get_hp_output_mode, set_hp_output_mode),

	SOC_DAPM_PIN_SWITCH("HP"),
	SOC_DAPM_PIN_SWITCH("SPK"),
	SOC_DAPM_PIN_SWITCH("RCV"),
};

static int main_mic_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	debug_msg("mic event = %d\n", event);

	gpio_set_value(GPIO_MICBIAS_EN, SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);

	return 0;
}

/*
 * Main Mic Bias : is controlled by DAPM widget
 * HeadSet Mic Bias : is controlled by jack driver
 * send_end key interrupt must be worked when stream is unactive
 */
static const struct snd_soc_dapm_widget herring_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Main Mic", main_mic_event),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),

	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_SPK("RCV", NULL),
	SND_SOC_DAPM_SPK("SPK", NULL),
};

static const struct snd_soc_dapm_route herring_dapm_routes[] = {
	{"IN1LN", NULL, "Main Mic"},
	{"IN1LP", NULL, "Main Mic"},
	{"IN1RN", NULL, "Headset Mic"},
	{"IN1RP", NULL, "Headset Mic"},

	{"HP", NULL, "HPOUT1L"},
	{"HP", NULL, "HPOUT1R"},
	{"SPK", NULL, "SPKOUTLN"},
	{"SPK", NULL, "SPKOUTLP"},
	{"RCV", NULL, "HPOUT2N"},
	{"RCV", NULL, "HPOUT2P"},
};

static int herring_wm8994_init(struct snd_soc_codec *codec)
{
	int err;

	/* add herring specific kcontorls */
	err = snd_soc_add_controls(codec, herring_controls,
			ARRAY_SIZE(herring_controls));

	if (err < 0)
		return err;

	/* add herring specific widgets */
	snd_soc_dapm_new_controls(codec, herring_dapm_widgets,
			ARRAY_SIZE(herring_dapm_widgets));

	/* set up herring specific audio routes */
	snd_soc_dapm_add_routes(codec, herring_dapm_routes,
			ARRAY_SIZE(herring_dapm_routes));

	/* set endpoints to not connected */
	snd_soc_dapm_nc_pin(codec, "IN2LP:VXRN");
	snd_soc_dapm_nc_pin(codec, "IN2LP:VXRP");
	snd_soc_dapm_nc_pin(codec, "LINEOUT1N");
	snd_soc_dapm_nc_pin(codec, "LINEOUT1P");
	snd_soc_dapm_nc_pin(codec, "LINEOUT2N");
	snd_soc_dapm_nc_pin(codec, "LINEOUT2P");
	snd_soc_dapm_nc_pin(codec, "SPKOUTRN");
	snd_soc_dapm_nc_pin(codec, "SPKOUTRP");

	/*
	  * ignore codec suspend if both end points are connected
	  * if use-case is only one endpoint,
	  * AP can't enter sleep(playback or capture)
	  */
	snd_soc_dapm_ignore_suspend(codec, "Main Mic");
	snd_soc_dapm_ignore_suspend(codec, "Headset Mic");
	snd_soc_dapm_ignore_suspend(codec, "Earpiece Driver");
	snd_soc_dapm_ignore_suspend(codec, "SPKL Driver");
	snd_soc_dapm_ignore_suspend(codec, "Headphone Supply");

	snd_soc_dapm_sync(codec);

	return 0;
}

static int set_main_clk_on_suspend(bool on)
{
	struct clk *codec_main_clk;

	codec_main_clk = clk_get(NULL, "usb_osc");

	if (!codec_main_clk)
		return -EINVAL;

	if (on) {
		clk_enable(codec_main_clk);
		debug_msg("Main clk was enabled on suspend\n");
	} else {
		clk_disable(codec_main_clk);
		debug_msg("Main clk was disabled on suspend\n");
	}

	return 1;

}

/*
 * Since we don't want to reclock on the fly (as it will glitch audio)
 * and we ensure that the CODEC is always clocked from the highest
 * clock we might want to use in any use case.  As the AP always sends
 * us 44.1kHz audio we clock AIF1 at 256fs of that using the FLL,
 * making this much higher than the rates we need for telephony
 * clocks.  This ensures that the CODEC is always clocked from AIF1
 * and we can therefore always activate AIF1 if need be.
 *
 * TODO: Manage the AP clock here as well; we can control the clock
 * which provides MCLK1.
 */
static int set_bias_level_post(struct snd_soc_card *card,
			       enum snd_soc_bias_level level)
{
	static enum snd_soc_bias_level cur_level;
	struct snd_soc_codec *codec = card->codec;
	struct snd_soc_dai *aif1 = &codec->dai[0];
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (cur_level == SND_SOC_BIAS_OFF) {
			set_main_clk_on_suspend(1);

			ret = snd_soc_dai_set_pll(aif1, WM8994_FLL1,
						  WM8994_FLL_SRC_MCLK1,
						  24000000, 44100 * 256);
			if (ret < 0)
				pr_err("snd_soc_dai_set_pll failed: %d\n",
				       ret);

			ret = snd_soc_dai_set_sysclk(aif1,
						     WM8994_SYSCLK_FLL1,
						     44100 * 256, 0);
			if (ret < 0) {
				pr_err("snd_soc_dai_set_sysclk failed: %d\n",
				       ret);
			}
		}
		break;

	case SND_SOC_BIAS_OFF:
		ret = snd_soc_dai_set_sysclk(aif1,
					     WM8994_SYSCLK_MCLK1,
					     24000000, 0);
		if (ret < 0) {
			pr_err("snd_soc_dai_set_sysclk failed: %d\n",
			       ret);
		}

		ret = snd_soc_dai_set_pll(aif1, WM8994_FLL1, 0, 0, 0);
		if (ret < 0) {
			pr_err("snd_soc_dai_set_pll failed: %d\n",
			       ret);
		}
		set_main_clk_on_suspend(0);
		break;

	default:
		break;
	}

	cur_level = level;

	return 0;
}

/*  BLC(bits-per-channel) --> BFS(bit clock shud be >= FS*(Bit-per-channel)*2)*/
/*  BFS --> RFS(must be a multiple of BFS)                                  */
/*  RFS & SRC_CLK --> Prescalar Value(SRC_CLK / RFS_VAL / fs - 1)           */
static int herring_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int bfs, rfs, ret;
	debug_msg("%s\n", __func__);

	/* Choose BFS and RFS values combination that is supported by
	 * both the WM8994 codec as well as the S5P AP
	 *
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	/* Can take any RFS value for AP */
			bfs = 16;
			rfs = 256;
			break;
	case SNDRV_PCM_FORMAT_S16_LE:
	/* Can take any RFS value for AP */
			bfs = 32;
			rfs = 256;
			break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
			bfs = 48;
			rfs = 512;
			break;
	/* Impossible, as the AP doesn't support 64fs or more BFS */
	case SNDRV_PCM_FORMAT_S32_LE:
	default:
			return -EINVAL;
	}

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0) {
		printk(KERN_ERR "herring_hifi_hw_params :\
				 Codec DAI configuration error!\n");
		return ret;
	}

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0) {
		printk(KERN_ERR
			"herring_hifi_hw_params :\
				AP DAI configuration error!\n");
		return ret;
	}

	/* Select the AP Sysclk */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CDCLKSRC_EXT,
					params_rate(params), SND_SOC_CLOCK_IN);

	if (ret < 0) {
		printk(KERN_ERR
			"herring_hifi_hw_params :\
			AP sys clock INT setting error!\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_I2SEXT,
					params_rate(params), SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR
			"herring_hifi_hw_params :\
			AP sys clock I2SEXT setting error!\n");
		return ret;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops hifi_ops = {
	.hw_params = herring_hifi_hw_params,
};

static int aif2_clk_control(struct snd_soc_dai *dai, bool enable, int rate)
{
	static int refcount;
	int ret;

	if (enable) {
		refcount++;

		if (refcount == 1) {
			ret = snd_soc_dai_set_pll(dai, WM8994_FLL2,
						  WM8994_FLL_SRC_MCLK1,
						  24000000, rate * 256);
			if (ret < 0) {
				pr_err("snd_soc_dai_set_pll failed: %d\n",
				       ret);
				return ret;
			}

			ret = snd_soc_dai_set_sysclk(dai,
						     WM8994_SYSCLK_FLL2,
						     rate * 256, 0);
			if (ret < 0) {
				pr_err("snd_soc_dai_set_sysclk failed: %d\n",
				       ret);
				return ret;
			}
		}
	} else {
		refcount--;

		if (refcount == 0) {
			ret = snd_soc_dai_set_sysclk(dai,
						     WM8994_SYSCLK_MCLK1,
						     24000000, 0);
			if (ret < 0) {
				pr_err("snd_soc_dai_set_sysclk failed: %d\n",
				       ret);
			}

			ret = snd_soc_dai_set_pll(dai, WM8994_FLL2, 0, 0, 0);
			if (ret < 0) {
				pr_err("snd_soc_dai_set_pll failed: %d\n",
				       ret);
			}
		}
	}

	return 0;
}

static int cp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	/* Ideally the CP would be bus master so we could ensure everything
	 * is synced against the network clock.
	 */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_LEFT_J |
				SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0) {
		pr_err("Failed to configure AIF2 for CP\n");
		return ret;
	}

	ret = aif2_clk_control(codec_dai, true, params_rate(params));
	if (ret < 0)
		return ret;

	return 0;
}

static void cp_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	aif2_clk_control(codec_dai, false, 0);
}

static struct snd_soc_ops cp_ops = {
	.hw_params = cp_hw_params,
	.shutdown = cp_shutdown,
};

static struct snd_soc_dai cp_dai = {
	.name = "CP",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	 },
	.symmetric_rates = 1,
};

static struct snd_soc_dai bt_dai = {
	.name = "BT",
	.id = 0,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	 },
	.symmetric_rates = 1,
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link herring_dai[] = {
	{
		.name = "AP",
		.stream_name = "WM8994 HiFi",
		.cpu_dai = &s3c64xx_i2s_dai[I2S_NUM],
		.codec_dai = &wm8994_dai[0],
		.ops = &hifi_ops,
		.init = herring_wm8994_init,
	},
	{
		.name = "CP",
		.stream_name = "CP",
		.cpu_dai = &cp_dai,
		.codec_dai = &wm8994_dai[1],
		.ops = &cp_ops,
	},
	{
		.name = "BT",
		.stream_name = "BT",
		.cpu_dai = &bt_dai,
		.codec_dai = &wm8994_dai[2],
		.ops = &hifi_ops,
	},
};

static struct snd_soc_card herring = {
	.name = "herring",
	.platform = &s3c_dma_wrapper,
	.dai_link = herring_dai,
	.num_links = ARRAY_SIZE(herring_dai),
	.set_bias_level_post = set_bias_level_post,
};

/* audio subsystem */
static struct snd_soc_device herring_snd_devdata = {
	.card = &herring,
	.codec_dev = &soc_codec_dev_wm8994,
};

static struct platform_device *herring_snd_device;
static int __init herring_audio_init(void)
{
	int ret;

	debug_msg("%s\n", __func__);

	snd_soc_register_dai(&cp_dai);
	snd_soc_register_dai(&bt_dai);

	herring_snd_device = platform_device_alloc("soc-audio", 0);
	if (!herring_snd_device)
		return -ENOMEM;

	platform_set_drvdata(herring_snd_device, &herring_snd_devdata);
	herring_snd_devdata.dev = &herring_snd_device->dev;
	ret = platform_device_add(herring_snd_device);

	if (ret)
		platform_device_put(herring_snd_device);

	return ret;
}

static void __exit herring_audio_exit(void)
{
	debug_msg("%s\n", __func__);

	platform_device_unregister(herring_snd_device);
}

module_init(herring_audio_init);
module_exit(herring_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC WM8994 Herring(C110)");
MODULE_LICENSE("GPL");
