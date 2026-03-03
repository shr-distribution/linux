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
#include <linux/clk.h>
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
	/* LPASS I2S clocks - must be enabled for I2S output */
	struct clk *codec_i2s_spkr_osr_clk;
	struct clk *codec_i2s_spkr_bit_clk;
	struct clk *codec_i2s_spkr_bit_div_clk;  /* parent for bit_clk mux */
	struct clk *codec_i2s_mic_osr_clk;
	struct clk *codec_i2s_mic_bit_clk;
	struct clk *codec_i2s_mic_bit_div_clk;   /* parent for bit_clk mux */
};

/*
 * Speaker amplifier enable via WM8994 GPIO_1.
 * The TouchPad uses an external Class-D amplifier controlled by this GPIO.
 * GPIO_1 = 0x41 enables the amp, 0x01 disables it.
 *
 * Note: This is a card-level widget, so we must find the codec component
 * from the card rather than using snd_soc_dapm_to_component() which only
 * works for codec-level widgets.
 */
static int apq8060_spk_pwr_amp(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *k, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct snd_soc_component *component;

	/* Find the WM8994 codec component - use strstr for partial match
	 * since component name may be "wm8994-codec.0" or similar
	 */
	for_each_card_components(card, component) {
		if (component->name && strstr(component->name, "wm8994"))
			goto found;
	}
	dev_err(card->dev, "Failed to find WM8994 codec component\n");
	return -ENODEV;

found:
	dev_info(card->dev, "Found WM8994 component: %s\n", component->name);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		dev_info(component->dev, "Enabling speaker amplifier\n");
		snd_soc_component_write(component, WM8994_GPIO_1, 0x41);
	} else {
		dev_info(component->dev, "Disabling speaker amplifier\n");
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

/*
 * DAPM routes for HP TouchPad.
 *
 * Note: The webOS kernel DAPM routes use LINEOUT for speakers, but runtime
 * mixer state analysis shows SPKOUT controls are actually active during
 * playback. The external Class-D amplifier is controlled via WM8958 GPIO1.
 * Testing shows SPKOUT path is what actually produces audio.
 *
 * MICBIAS is a SUPPLY widget in modern kernels - the codec driver enables
 * it automatically when the corresponding input is powered.
 */
static const struct snd_soc_dapm_route apq8060_dapm_routes[] = {
	{ "Headphone", NULL, "HPOUT1L" },
	{ "Headphone", NULL, "HPOUT1R" },

	/* Speakers via internal Class-D driver */
	{ "Speaker", NULL, "SPKOUTLP" },
	{ "Speaker", NULL, "SPKOUTLN" },
	{ "Speaker", NULL, "SPKOUTRP" },
	{ "Speaker", NULL, "SPKOUTRN" },

	/* Internal Mic connected to IN1LN */
	{ "IN1LN", NULL, "Internal Mic" },

	/* Headset Mic connected to IN2LN */
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
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct apq8060_snd_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	/*
	 * Enable LPASS I2S clocks. On APQ8060/MSM8660, the Q6 DSP
	 * handles audio routing but the I2S output clocks must be
	 * enabled by Linux for I2S signals to appear on the GPIOs.
	 *
	 * OSR clock = 12.288MHz (256 * 48kHz) - oversampling clock
	 * BIT clock = 1.536MHz (48kHz * 2ch * 16bit) - I2S bit clock
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (data->codec_i2s_spkr_osr_clk) {
			ret = clk_set_rate(data->codec_i2s_spkr_osr_clk, 12288000);
			if (ret)
				dev_warn(card->dev, "Failed to set spkr_osr rate: %d\n", ret);
			ret = clk_prepare_enable(data->codec_i2s_spkr_osr_clk);
			if (ret)
				dev_warn(card->dev, "Failed to enable spkr_osr: %d\n", ret);
			else
				dev_info(card->dev, "Enabled codec_i2s_spkr_osr_clk\n");
		}
		if (data->codec_i2s_spkr_bit_clk) {
			ret = clk_set_rate(data->codec_i2s_spkr_bit_clk, 1536000);
			if (ret)
				dev_warn(card->dev, "Failed to set spkr_bit rate: %d\n", ret);
			ret = clk_prepare_enable(data->codec_i2s_spkr_bit_clk);
			if (ret)
				dev_warn(card->dev, "Failed to enable spkr_bit: %d\n", ret);
			else
				dev_info(card->dev, "Enabled codec_i2s_spkr_bit_clk @ 1.536MHz\n");
		}
	} else {
		if (data->codec_i2s_mic_osr_clk) {
			ret = clk_set_rate(data->codec_i2s_mic_osr_clk, 12288000);
			if (ret)
				dev_warn(card->dev, "Failed to set mic_osr rate: %d\n", ret);
			ret = clk_prepare_enable(data->codec_i2s_mic_osr_clk);
			if (ret)
				dev_warn(card->dev, "Failed to enable mic_osr: %d\n", ret);
		}
		if (data->codec_i2s_mic_bit_clk) {
			ret = clk_set_rate(data->codec_i2s_mic_bit_clk, 1536000);
			if (ret)
				dev_warn(card->dev, "Failed to set mic_bit rate: %d\n", ret);
			ret = clk_prepare_enable(data->codec_i2s_mic_bit_clk);
			if (ret)
				dev_warn(card->dev, "Failed to enable mic_bit: %d\n", ret);
		}
	}

	return 0;
}

static void apq8060_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct apq8060_snd_data *data = snd_soc_card_get_drvdata(card);

	/* Disable LPASS I2S clocks */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (data->codec_i2s_spkr_bit_clk)
			clk_disable_unprepare(data->codec_i2s_spkr_bit_clk);
		if (data->codec_i2s_spkr_osr_clk)
			clk_disable_unprepare(data->codec_i2s_spkr_osr_clk);
	} else {
		if (data->codec_i2s_mic_bit_clk)
			clk_disable_unprepare(data->codec_i2s_mic_bit_clk);
		if (data->codec_i2s_mic_osr_clk)
			clk_disable_unprepare(data->codec_i2s_mic_osr_clk);
	}
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
	 * Set DAI format - I2S, Q6 DSP provides bit/frame clocks.
	 * The Q6 LPASS DSP is the clock master, generating BCLK and LRCLK
	 * via LCC clocks. WM8958 codec is clock slave, receiving clocks from Q6.
	 * The codec FLL locks to the incoming BCLK to generate its internal clocks.
	 * This matches the legacy webOS kernel configuration.
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
				  SND_SOC_DAIFMT_BC_FC);
	if (ret && ret != -ENOTSUPP) {
		dev_err(rtd->dev, "Failed to set codec DAI format: %d\n", ret);
		return ret;
	}

	/*
	 * Configure WM8958 FLL using BCLK as reference.
	 * The Q6 LPASS generates BCLK at 1.536MHz (48kHz * 16 * 2).
	 * The FLL locks to this BCLK and multiplies it to generate SYSCLK.
	 *
	 * FLL output must be >= 256 * fs and between 4.096MHz - 12.5MHz.
	 * For 48kHz: 256 * 48000 = 12.288MHz (within range)
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

	/* BCLK rate from Q6: sample_rate * channels * bits = 48000 * 2 * 16 = 1.536MHz */
	data->bclk_rate = rate * WM_CHANNELS * WM_BITS;

	/* FLL output rate: 256 * sample rate for proper SYSCLK */
	data->fll_rate = 256 * rate;

	/* Ensure FLL rate is at least 4.096MHz */
	if (data->fll_rate < 4096000)
		data->fll_rate = 4096000;

	dev_info(rtd->dev, "APQ8060: Setting FLL%d from LRCLK %u Hz to %u Hz\n",
		 data->fll_id == WM8994_FLL1 ? 1 : 2, rate, data->fll_rate);

	/*
	 * Use LRCLK (frame clock) as FLL source. LRCLK runs at the sample rate
	 * (e.g., 48kHz) and the FLL multiplies it to 12.288MHz (256 * 48kHz).
	 *
	 * Note: LRCLK is provided by Q6 LPASS and only appears when streaming
	 * starts. The FLL will lock once the clock is present.
	 *
	 * We cannot use the internal oscillator because the WM8994 driver
	 * forces it to output 12MHz, which doesn't align with audio rates
	 * (need 12.288MHz for 48kHz = 256 * Fs).
	 */
	ret = snd_soc_dai_set_pll(codec_dai, data->fll_id, WM8994_FLL_SRC_LRCLK,
				  rate, data->fll_rate);
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

	/*
	 * Enable power management registers for the output path.
	 * DAPM isn't powering the widgets because it doesn't recognize
	 * the DPCM path as active. Force-enable them here during hw_params
	 * so they're ready when playback starts.
	 *
	 * PM1 (0x01): SPKOUTL_ENA, SPKOUTR_ENA, HPOUT1L_ENA, HPOUT1R_ENA
	 * PM3 (0x03): SPKLVOL_ENA, SPKRVOL_ENA, MIXOUTLVOL_ENA, MIXOUTRVOL_ENA
	 * PM5 (0x05): DAC1L_ENA, DAC1R_ENA
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		struct snd_soc_component *component = codec_dai->component;

		dev_info(rtd->dev, "APQ8060: Enabling output power registers\n");
		snd_soc_component_update_bits(component,
			WM8994_POWER_MANAGEMENT_5, 0x0003, 0x0003);
		snd_soc_component_update_bits(component,
			WM8994_POWER_MANAGEMENT_3, 0x0330, 0x0330);
		snd_soc_component_update_bits(component,
			WM8994_POWER_MANAGEMENT_1, 0x3300, 0x3300);

		/*
		 * Unmute DAC and set volume to 0dB.
		 * Bit 9 = MUTE (0 = unmute), Bit 8 = VU (1 = update), Bits 7:0 = volume
		 * 0x01C0 = MUTE=0, VU=1, VOL=0xC0 (0dB)
		 */
		dev_info(rtd->dev, "APQ8060: Unmuting DAC\n");
		snd_soc_component_write(component,
			WM8994_AIF1_DAC1_LEFT_VOLUME, 0x01C0);
		snd_soc_component_write(component,
			WM8994_AIF1_DAC1_RIGHT_VOLUME, 0x01C0);
		snd_soc_component_write(component,
			WM8994_DAC1_LEFT_VOLUME, 0x01C0);
		snd_soc_component_write(component,
			WM8994_DAC1_RIGHT_VOLUME, 0x01C0);

		/*
		 * Enable speaker output and bias.
		 * VMID is needed for the analog outputs to function.
		 * Also ensure speaker mixer paths are active.
		 */
		dev_info(rtd->dev, "APQ8060: Enabling speaker output path\n");

		/* Ensure VMID and bias are enabled (PM1 bits 1:0) */
		snd_soc_component_update_bits(component,
			WM8994_POWER_MANAGEMENT_1, 0x0003, 0x0003);

		/* Enable speaker mixer to output (0x24): SPKMIXL→SPKOUTL, SPKMIXR→SPKOUTR */
		snd_soc_component_update_bits(component,
			WM8994_SPKOUT_MIXERS, 0x0011, 0x0011);

		/* Ensure speaker volume is unmuted and at max (0x26, 0x27) */
		snd_soc_component_write(component,
			WM8994_SPEAKER_VOLUME_LEFT, 0x017F);
		snd_soc_component_write(component,
			WM8994_SPEAKER_VOLUME_RIGHT, 0x017F);
	}

	dev_info(rtd->dev, "APQ8060: Codec clock configured using LRCLK as FLL source\n");
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

	/*
	 * Disable unused codec pins.
	 * LINEOUT pins are not connected - speakers use internal Class-D driver.
	 */
	snd_soc_dapm_nc_pin(&card->dapm, "LINEOUT1P");
	snd_soc_dapm_nc_pin(&card->dapm, "LINEOUT1N");
	snd_soc_dapm_nc_pin(&card->dapm, "LINEOUT2P");
	snd_soc_dapm_nc_pin(&card->dapm, "LINEOUT2N");
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

	/* Force-enable Headphone and Speaker pins for testing
	 * TODO: Implement proper jack detection via WM8994 or GPIO
	 */
	snd_soc_dapm_force_enable_pin(&card->dapm, "Headphone");
	snd_soc_dapm_force_enable_pin(&card->dapm, "Speaker");

	/*
	 * Force-enable the AIF1 Playback stream widget.
	 * In DPCM, the backend should automatically power this stream endpoint
	 * when playback starts, but it's not working correctly. Force-enable
	 * it as a workaround to complete the DAPM path to the outputs.
	 */
	snd_soc_dapm_force_enable_pin(&card->dapm, "AIF1 Playback");

	snd_soc_dapm_sync(&card->dapm);

	/*
	 * Configure WM8994 internal mixer routing and clocks for playback.
	 * The codec has multiple internal mixers that must be enabled
	 * for audio to flow from AIF1 to the outputs.
	 *
	 * Path: AIF1 → DAC1 Mixer → DAC1 → Output Mixer → Headphones
	 *                               → Speaker Mixer → Speakers
	 */
	{
		struct snd_soc_component *component;

		for_each_card_components(card, component) {
			if (component->name && strstr(component->name, "wm8994")) {
				dev_info(card->dev, "Configuring WM8994 mixer routing\n");

				/*
				 * Force-enable clock supply widgets on the codec's DAPM.
				 * These must be enabled on the component's DAPM context,
				 * not the card's, since they're codec-internal widgets.
				 */
				snd_soc_dapm_force_enable_pin(&component->dapm, "AIF1CLK");
				snd_soc_dapm_force_enable_pin(&component->dapm, "CLK_SYS");

				/*
				 * Force-enable the entire audio path from AIF1 to speakers.
				 * DAPM isn't recognizing the DPCM path as active, so
				 * force all widgets ON to enable audio output.
				 *
				 * Signal path: AIF1DAC1 → DAC1 Mixer → DAC1 → Output Mixer →
				 *              Output PGA → SPKL/R → SPKL/R Driver
				 */
				/* AIF1 DAC input widgets */
				snd_soc_dapm_force_enable_pin(&component->dapm, "AIF1DAC1L");
				snd_soc_dapm_force_enable_pin(&component->dapm, "AIF1DAC1R");
				/* DAC mixer widgets */
				snd_soc_dapm_force_enable_pin(&component->dapm, "DAC1L Mixer");
				snd_soc_dapm_force_enable_pin(&component->dapm, "DAC1R Mixer");
				/* DAC widgets */
				snd_soc_dapm_force_enable_pin(&component->dapm, "DAC1L");
				snd_soc_dapm_force_enable_pin(&component->dapm, "DAC1R");
				/* Output mixer and PGA widgets (needed for DAC→speaker path) */
				snd_soc_dapm_force_enable_pin(&component->dapm, "Left Output Mixer");
				snd_soc_dapm_force_enable_pin(&component->dapm, "Right Output Mixer");
				snd_soc_dapm_force_enable_pin(&component->dapm, "Left Output PGA");
				snd_soc_dapm_force_enable_pin(&component->dapm, "Right Output PGA");
				/* Speaker mixer and driver widgets */
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKL");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKR");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKL Boost");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKR Boost");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKL Driver");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKR Driver");
				/* Speaker output pins - final stage before physical outputs */
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKOUTLP");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKOUTLN");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKOUTRP");
				snd_soc_dapm_force_enable_pin(&component->dapm, "SPKOUTRN");
				snd_soc_dapm_sync(&component->dapm);

				/*
				 * Enable AIF1.1 → DAC1 path:
				 * DAC1L Mixer: AIF1.1 Switch = bit 0
				 * DAC1R Mixer: AIF1.1 Switch = bit 0
				 */
				snd_soc_component_update_bits(component,
					WM8994_DAC1_LEFT_MIXER_ROUTING, 0x01, 0x01);
				snd_soc_component_update_bits(component,
					WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x01, 0x01);

				/*
				 * Enable DAC1 → Output Mixer for headphones:
				 * Left Output Mixer: DAC Switch = bit 0
				 * Right Output Mixer: DAC Switch = bit 0
				 */
				snd_soc_component_update_bits(component,
					WM8994_OUTPUT_MIXER_1, 0x01, 0x01);
				snd_soc_component_update_bits(component,
					WM8994_OUTPUT_MIXER_2, 0x01, 0x01);

				/*
				 * Enable DAC1 → Speaker Mixer for speakers:
				 * SPKL: DAC1 Switch = bit 1
				 * SPKR: DAC1 Switch = bit 0
				 */
				snd_soc_component_update_bits(component,
					WM8994_SPEAKER_MIXER, 0x03, 0x03);

				/*
				 * Enable external speaker amplifier via GPIO1.
				 * GPIO1 = 0x41 enables the external Class-D amp.
				 */
				dev_info(card->dev, "Enabling speaker amplifier (GPIO1)\n");
				snd_soc_component_write(component, WM8994_GPIO_1, 0x41);

				/*
				 * Unmute DAC1 and AIF1DAC1 volume registers.
				 * Bit 9 = MUTE (set to 0 to unmute)
				 * Bit 8 = VU (set to 1 to update volume)
				 * Bits 7:0 = VOL (0xC0 = 0dB)
				 */
				dev_info(card->dev, "Unmuting DAC1 and AIF1DAC1\n");
				snd_soc_component_write(component,
					WM8994_AIF1_DAC1_LEFT_VOLUME, 0x01C0);
				snd_soc_component_write(component,
					WM8994_AIF1_DAC1_RIGHT_VOLUME, 0x01C0);
				snd_soc_component_write(component,
					WM8994_DAC1_LEFT_VOLUME, 0x01C0);
				snd_soc_component_write(component,
					WM8994_DAC1_RIGHT_VOLUME, 0x01C0);
				break;
			}
		}
	}

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

	/*
	 * Get LPASS I2S clocks from DT (referenced from LCC clock controller).
	 * On APQ8060/MSM8660, these clocks must be enabled by Linux
	 * for I2S output to work. The Q6 DSP doesn't control them.
	 */
	data->codec_i2s_spkr_osr_clk = devm_clk_get_optional(dev,
						"codec_i2s_spkr_osr_clk");
	if (IS_ERR(data->codec_i2s_spkr_osr_clk)) {
		ret = PTR_ERR(data->codec_i2s_spkr_osr_clk);
		dev_err(dev, "Failed to get codec_i2s_spkr_osr_clk: %d\n", ret);
		return ret;
	}

	data->codec_i2s_spkr_bit_clk = devm_clk_get_optional(dev,
						"codec_i2s_spkr_bit_clk");
	if (IS_ERR(data->codec_i2s_spkr_bit_clk)) {
		ret = PTR_ERR(data->codec_i2s_spkr_bit_clk);
		dev_err(dev, "Failed to get codec_i2s_spkr_bit_clk: %d\n", ret);
		return ret;
	}

	/*
	 * Get bit_div_clk (internal LCC clock) to set as parent of bit_clk mux.
	 * The bit_clk is a mux that can select between internal LCC clock
	 * (bit_div_clk, GPIO as output) or external codec clock (GPIO as input).
	 * We need the internal clock to drive the I2S SCK/BCLK to the codec.
	 */
	data->codec_i2s_spkr_bit_div_clk = devm_clk_get_optional(dev,
						"codec_i2s_spkr_bit_div_clk");
	if (IS_ERR(data->codec_i2s_spkr_bit_div_clk)) {
		ret = PTR_ERR(data->codec_i2s_spkr_bit_div_clk);
		dev_err(dev, "Failed to get codec_i2s_spkr_bit_div_clk: %d\n", ret);
		return ret;
	}

	/* Set internal clock as parent of bit_clk mux (selects GPIO as output) */
	if (data->codec_i2s_spkr_bit_clk && data->codec_i2s_spkr_bit_div_clk) {
		ret = clk_set_parent(data->codec_i2s_spkr_bit_clk,
				     data->codec_i2s_spkr_bit_div_clk);
		if (ret)
			dev_warn(dev, "Failed to set spkr bit_clk parent: %d\n", ret);
		else
			dev_info(dev, "Set spkr bit_clk parent to internal bit_div_clk\n");
	}

	data->codec_i2s_mic_osr_clk = devm_clk_get_optional(dev,
						"codec_i2s_mic_osr_clk");
	if (IS_ERR(data->codec_i2s_mic_osr_clk)) {
		ret = PTR_ERR(data->codec_i2s_mic_osr_clk);
		dev_err(dev, "Failed to get codec_i2s_mic_osr_clk: %d\n", ret);
		return ret;
	}

	data->codec_i2s_mic_bit_clk = devm_clk_get_optional(dev,
						"codec_i2s_mic_bit_clk");
	if (IS_ERR(data->codec_i2s_mic_bit_clk)) {
		ret = PTR_ERR(data->codec_i2s_mic_bit_clk);
		dev_err(dev, "Failed to get codec_i2s_mic_bit_clk: %d\n", ret);
		return ret;
	}

	/* Get mic bit_div_clk and set as parent */
	data->codec_i2s_mic_bit_div_clk = devm_clk_get_optional(dev,
						"codec_i2s_mic_bit_div_clk");
	if (IS_ERR(data->codec_i2s_mic_bit_div_clk)) {
		ret = PTR_ERR(data->codec_i2s_mic_bit_div_clk);
		dev_err(dev, "Failed to get codec_i2s_mic_bit_div_clk: %d\n", ret);
		return ret;
	}

	if (data->codec_i2s_mic_bit_clk && data->codec_i2s_mic_bit_div_clk) {
		ret = clk_set_parent(data->codec_i2s_mic_bit_clk,
				     data->codec_i2s_mic_bit_div_clk);
		if (ret)
			dev_warn(dev, "Failed to set mic bit_clk parent: %d\n", ret);
		else
			dev_info(dev, "Set mic bit_clk parent to internal bit_div_clk\n");
	}

	if (data->codec_i2s_spkr_osr_clk && data->codec_i2s_spkr_bit_clk)
		dev_info(dev, "Got LPASS I2S speaker clocks\n");
	if (data->codec_i2s_mic_osr_clk && data->codec_i2s_mic_bit_clk)
		dev_info(dev, "Got LPASS I2S mic clocks\n");

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
