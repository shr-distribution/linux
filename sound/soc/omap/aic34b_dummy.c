/*
 * aic34b_dummy.c  --  Dummy driver for AIC34 block B parts used in Nokia RX51
 *
 * Purpose for this driver is to cover few audio connections on Nokia RX51 HW
 * which are connected into block B of TLV320AIC34 dual codec.
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * TODO:
 * - Get rid of this driver, at least when ASoC multi-component is merged into
 *   mainline.
 *   This driver is hacked only for Nokia RX51 HW.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "../codecs/tlv320aic3x.h"

struct i2c_client *aic34b_client;

static int aic34b_read(struct i2c_client *client, unsigned int reg)
{
	int val;

	val = i2c_smbus_read_byte_data(client, reg);

	return val;
}

static int aic34b_write(struct i2c_client *client, unsigned int reg,
			u8 value)
{
	u8 data[2];

	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	return (i2c_master_send(client, data, 2) == 2) ? 0 : -EIO;
}

static int aic34b_get_volsw(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int val;

	if (aic34b_client == NULL)
		return 0;

	val = (aic34b_read(aic34b_client, reg) >> shift) & mask;
	if (invert)
		val = max - val;
	ucontrol->value.integer.value[0] = val;

	return 0;
}

static int aic34b_put_volsw(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	int val_reg;

	if (aic34b_client == NULL)
		return 0;

	if (invert)
		val = max - val;

	val_reg = aic34b_read(aic34b_client, reg);
	if (((val_reg >> shift) & mask) == val) {
		return 0;
	}

	val_reg &= ~(mask << shift);
	val_reg |= val << shift;
	aic34b_write(aic34b_client, reg, val_reg);

	return 1;
}

static int aic34b_bypass_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	int val;

	if (aic34b_client == NULL)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Connect LINE2R to RADC in differential mode and 0 dB gain */
		aic34b_write(aic34b_client, LINE2R_2_RADC_CTRL, 0x80);
		/* Unmute Right ADC-PGA */
		aic34b_write(aic34b_client, RADC_VOL, 0x00);
		/* Right PGA -> HPLOUT */
		val = aic34b_read(aic34b_client, PGAR_2_HPLOUT_VOL);
		aic34b_write(aic34b_client, PGAR_2_HPLOUT_VOL, val | 0x80);
		/* Unmute HPLOUT with 1 dB gain */
		aic34b_write(aic34b_client, HPLOUT_CTRL, 0x19);
		/* Unmute HPLCOM with 1 dB gain */
		aic34b_write(aic34b_client, HPLCOM_CTRL, 0x19);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Disconnect LINE2R from RADC */
		aic34b_write(aic34b_client, LINE2R_2_RADC_CTRL, 0xF8);
		/* Mute Right ADC-PGA */
		aic34b_write(aic34b_client, RADC_VOL, 0x80);
		/* Detach Right PGA from HPLOUT */
		val = aic34b_read(aic34b_client, PGAR_2_HPLOUT_VOL);
		aic34b_write(aic34b_client, PGAR_2_HPLOUT_VOL, val & 0x7f);
		/* Power down HPLOUT */
		aic34b_write(aic34b_client, HPLOUT_CTRL, 0x06);
		/* Power down HPLCOM */
		aic34b_write(aic34b_client, HPLCOM_CTRL, 0x06);
		break;
	}

	return 0;
}

static int aic34b_mic_bias_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	if (aic34b_client == NULL)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		aic34b_write(aic34b_client, MICBIAS_CTRL, 2 << 6);
		break;
	case SND_SOC_DAPM_POST_PMD:
		aic34b_write(aic34b_client, MICBIAS_CTRL, 0);
		break;
	}

	return 0;
}

static DECLARE_TLV_DB_SCALE(output_stage_tlv, -5900, 50, 1);

static const struct snd_kcontrol_new aic34b_snd_controls[] = {
	SOC_SINGLE_EXT_TLV("34B HPL PGAR Bypass Playback Volume",
			   PGAR_2_HPLOUT_VOL, 0, 118, 1,
			   aic34b_get_volsw, aic34b_put_volsw,
			   output_stage_tlv),
};

static const struct snd_soc_dapm_widget aic34b_dapm_widgets[] = {
	SND_SOC_DAPM_PGA_E("34B LINE2R HPL Bypass", SND_SOC_NOPM,
			   0, 0, NULL, 0, aic34b_bypass_event,
			   SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("34B Mic Bias 2.5V", SND_SOC_NOPM,
			   0, 0, aic34b_mic_bias_event,
			   SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_OUTPUT("34B_HPLOUT"),
	SND_SOC_DAPM_OUTPUT("34B_HPLCOM"),
	SND_SOC_DAPM_INPUT("34B_LINE2R"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"34B LINE2R HPL Bypass", NULL, "34B_LINE2R"},
	{"34B_HPLOUT", NULL, "34B LINE2R HPL Bypass"},
	{"34B_HPLCOM", NULL, "34B LINE2R HPL Bypass"},
};

int aic34b_add_controls(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, aic34b_dapm_widgets,
				  ARRAY_SIZE(aic34b_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	return snd_soc_add_controls(codec, aic34b_snd_controls,
				    ARRAY_SIZE(aic34b_snd_controls));
}
EXPORT_SYMBOL_GPL(aic34b_add_controls);

static int aic34b_dummy_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	if (aic34b_read(client, AIC3X_PLL_PROGA_REG) != 0x10) {
		/* Chip not present */
		return -ENODEV;
	}
	aic34b_client = client;

	return 0;
}

static int aic34b_dummy_remove(struct i2c_client *client)
{
	aic34b_client = NULL;

	return 0;
}

static const struct i2c_device_id aic34b_dummy_id[] = {
	{ "aic34b_dummy", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aic34b_dummy_id);

static struct i2c_driver aic34b_dummy_driver = {
	.driver = {
		.name	= "aic34b_dummy"
	},
	.probe		= aic34b_dummy_probe,
	.remove		= aic34b_dummy_remove,
	.id_table	= aic34b_dummy_id,
};

static int __init aic34b_dummy_init(void)
{
	return i2c_add_driver(&aic34b_dummy_driver);
}

static void __exit aic34b_dummy_exit(void)
{
	i2c_del_driver(&aic34b_dummy_driver);
}

MODULE_AUTHOR();
MODULE_DESCRIPTION("Dummy driver for AIC34 block B parts used on Nokia RX51");
MODULE_LICENSE("GPL");

module_init(aic34b_dummy_init);
module_exit(aic34b_dummy_exit);
