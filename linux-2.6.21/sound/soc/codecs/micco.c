/*
 * linux/sound/soc/codecs/micco.c
 * Base on linux/sound/soc/codecs/wm9712.c
 * 
 * Copyright (C) 2007 Marvell International Ltd.  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/littleton.h>
#include <asm/arch/micco.h>

#define DA9034_ERRATA_17	1

#define MICCO_SOC_PROC 

#define	MICCO_MUX_MONO_V		0x16
#define	MICCO_MUX_BEAR_V		0x17
#define	MICCO_MUX_LINE_OUT_V		0x18
#define	MICCO_MUX_STEREO_CH1_V		0x19
#define	MICCO_MUX_STEREO_CH2_V		0x1a
#define	MICCO_MUX_TX_V			0x1b

extern int golden_muted;

static unsigned int micco_soc_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int micco_soc_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val);

static const u8 micco_regs[] = {
	0x00, 0x00, 0x00, 0x00,	/*0x00 ~ 0x03*/
	0x00, 0x25, 0x7f, 0x3f, /*0x04 ~ 0x07*/
	0x00, 0xbf, 0xbf, 0x00, /*0x08 ~ 0x0b*/
	0x0c, 0x60, 0x00, 0x00, /*0x0c ~ 0x0f*/
	0x04, 0x00, 0x00, 0x00,	/*0x10 ~ 0x13*/
	0x00, 0x00, 		/*0x14 ~ 0x15*/			
	0x00, 0x00, 0x00, 0x00,	/*0x16 ~ 0x19*/
	0x00, 0x00		/*0x1a ~ 0x1b*/
};


static int do_post_event (struct snd_soc_dapm_widget *w, int event)
{
	u16 v_val;
	char *name = w->name;

	if (event & SND_SOC_DAPM_PRE_REG)
		return 0;

	if (!strcmp("MONO Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_MONO_V);
		micco_soc_write(w->codec, MICCO_MUX_MONO, 1<<(v_val));
	} else if (!strcmp("BEAR Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_BEAR_V);
		micco_soc_write(w->codec, MICCO_MUX_BEAR, 1<<(v_val));
	} else if (!strcmp("LINE OUT Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_LINE_OUT_V);
		if(v_val<=7)
			micco_soc_write(w->codec, 
					MICCO_MUX_LINE_OUT, 1<<(v_val));
	} else if (!strcmp("STEREO_CH1 Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_STEREO_CH1_V);
		micco_soc_write(w->codec, MICCO_MUX_STEREO_CH1, 1<<(v_val));
	} else if (!strcmp("STEREO_CH2 Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_STEREO_CH2_V);
		micco_soc_write(w->codec, MICCO_MUX_STEREO_CH2, 1<<(v_val));
	} else if (!strcmp("TX Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_TX_V);
		if (v_val == 2 || v_val == 3)
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 3<<2);
		else
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 1<<(v_val));
	} else {
		printk(KERN_ERR "Invalid widget name: %s\n", name);
		return -1;
	}

	return 0;
}


#ifdef DA9034_ERRATA_17
#define MUX_INPUT	\
		"AUX1", "AUX2", "AUX3", "DAC1", "DAC2", "DAC3", "AUX2inv"
#else
#define MUX_INPUT	\
		"AUX1", "AUX2", "AUX2inv", "AUX3", "DAC1", "DAC2", "DAC3"

#endif
static const char *micco_mono_mux[] = {MUX_INPUT};
static const char *micco_bear_mux[] = {MUX_INPUT};
static const char *micco_line_out_mux[] = {MUX_INPUT, "MIC_P", 
				"NONE", "NONE", "NONE", "NONE", "NONE", "NONE", "NONE"};
static const char *micco_stereo_ch1_mux[] = {MUX_INPUT};
static const char *micco_stereo_ch2_mux[] = {MUX_INPUT};
static const char *micco_tx_mux[] = {"AUX2", "AUX1", "Mic1", "Mic2", "AUX2inv",
       				"AUX3", "DAC1", "DAC2", "NONE", "NONE", "NONE",
			       	"NONE", "NONE", "NONE", "NONE"};

static const struct soc_enum micco_enum[] = {
	SOC_ENUM_SINGLE(MICCO_MUX_MONO_V,	0, 7, micco_mono_mux), 
	SOC_ENUM_SINGLE(MICCO_MUX_BEAR_V,	0, 7, micco_bear_mux), 
	SOC_ENUM_SINGLE(MICCO_MUX_LINE_OUT_V,	0, 15, micco_line_out_mux),
	SOC_ENUM_SINGLE(MICCO_MUX_STEREO_CH1_V, 0, 7, micco_stereo_ch1_mux),
	SOC_ENUM_SINGLE(MICCO_MUX_STEREO_CH2_V, 0, 7, micco_stereo_ch2_mux),
	SOC_ENUM_SINGLE(MICCO_MUX_TX_V,		0, 15, micco_tx_mux),
};


static const struct snd_kcontrol_new micco_mono_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[0]);
static const struct snd_kcontrol_new micco_bear_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[1]);
static const struct snd_kcontrol_new micco_line_out_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[2]);
static const struct snd_kcontrol_new micco_stereo_ch1_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[3]);
static const struct snd_kcontrol_new micco_stereo_ch2_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[4]);
static const struct snd_kcontrol_new micco_tx_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[5]);

static const struct snd_soc_dapm_widget micco_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC1", "Left HiFi Playback", MICCO_HIFI_DAC_CONTROL,
			7, 0),
	SND_SOC_DAPM_DAC("DAC2", "Right HiFi Playback", MICCO_HIFI_DAC_CONTROL,
			7, 0),
	SND_SOC_DAPM_DAC("DAC3", "Voice Playback", MICCO_VCODEC_VDAC_CONTROL,
			3, 0),
	SND_SOC_DAPM_ADC("ADC", "Voice Capture", MICCO_VCODEC_ADC_CONTROL,
			0, 0),
	SND_SOC_DAPM_MUX_E("MONO Mux", SND_SOC_NOPM, 
			0, 0, &micco_mono_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("BEAR Mux", SND_SOC_NOPM,
		       	0, 0, &micco_bear_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("LINE_OUT Mux", SND_SOC_NOPM, 0, 
			0, &micco_line_out_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("STEREO_CH1 Mux", SND_SOC_NOPM, 
			0, 0, &micco_stereo_ch1_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("STEREO_CH2 Mux", SND_SOC_NOPM, 
			0, 0, &micco_stereo_ch2_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("TX Mux", SND_SOC_NOPM, 
			0, 0, &micco_tx_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_PGA("MONO PGA", MICCO_MONO_VOL, 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("BEAR PGA", MICCO_BEAR_VOL, 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LINE_OUT PGA", MICCO_AUDIO_LINE_AMP, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("STEREO_CH1 PGA", MICCO_STEREO_AMPLITUDE_CH1, 
			7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("STEREO_CH2 PGA", MICCO_STEREO_AMPLITUDE_CH1, 
			6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC PGA", MICCO_MIC_PGA, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX1 PGA", MICCO_PGA_AUX1_2, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX2 PGA", MICCO_PGA_AUX1_2, 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX3 PGA", MICCO_PGA_AUX3, 3, 0, NULL, 0),


	SND_SOC_DAPM_OUTPUT("BEAR1"),
	SND_SOC_DAPM_OUTPUT("BEAR2"),
	SND_SOC_DAPM_OUTPUT("MONO1"),
	SND_SOC_DAPM_OUTPUT("MONO2"),
	SND_SOC_DAPM_OUTPUT("LINE_OUT"),
	SND_SOC_DAPM_OUTPUT("STEREO_CH1"),
	SND_SOC_DAPM_OUTPUT("STEREO_CH2"),
	SND_SOC_DAPM_INPUT("AUX1"),
	SND_SOC_DAPM_INPUT("AUX2"),
	SND_SOC_DAPM_INPUT("AUX3"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
};

static const char *micco_audio_map[][3] = {
	/*MONO Mux*/
	{"MONO Mux", "AUX1", "AUX1"},
	{"MONO Mux", "AUX2", "AUX2"},
	{"MONO Mux", "AUX3", "AUX3"},
	{"MONO Mux", "DAC1", "DAC1"},
	{"MONO Mux", "DAC2", "DAC2"},
	{"MONO Mux", "DAC3", "DAC3"},

	{"MONO PGA", NULL, "MONO Mux"},
	{"MONO1", NULL, "MONO PGA"},
	{"MONO2", NULL, "MONO PGA"},

	/*BEAR Mux*/
	{"BEAR Mux", "AUX1", "AUX1"},
	{"BEAR Mux", "AUX2", "AUX2"},
	{"BEAR Mux", "AUX3", "AUX3"},
	{"BEAR Mux", "DAC1", "DAC1"},
	{"BEAR Mux", "DAC2", "DAC2"},
	{"BEAR Mux", "DAC3", "DAC3"},

	{"BEAR PGA", NULL, "BEAR Mux"},
	{"BEAR1", NULL, "BEAR PGA"},
	{"BEAR2", NULL, "BEAR PGA"},

	/*LINE OUT Mux Mux*/
	{"LINE_OUT Mux", "AUX1", "AUX1"},
	{"LINE_OUT Mux", "AUX2", "AUX2"},
	{"LINE_OUT Mux", "AUX3", "AUX3"},
	{"LINE_OUT Mux", "DAC1", "DAC1"},
	{"LINE_OUT Mux", "DAC2", "DAC2"},
	{"LINE_OUT Mux", "DAC3", "DAC3"},
	{"LINE_OUT Mux", "MIC_P", "MIC_P"},
	
	{"LINE_OUT PGA", NULL, "LINE_OUT Mux"},
	{"LINE_OUT", NULL, "LINE_OUT PGA"},
	

	/*STEREO_CH1 Mux*/
	{"STEREO_CH1 Mux", "AUX1", "AUX1"},
	{"STEREO_CH1 Mux", "AUX2", "AUX2"},
	{"STEREO_CH1 Mux", "AUX3", "AUX3"},
	{"STEREO_CH1 Mux", "DAC1", "DAC1"},
	{"STEREO_CH1 Mux", "DAC2", "DAC2"},
	{"STEREO_CH1 Mux", "DAC3", "DAC3"},

	{"STEREO_CH1 PGA", NULL, "STEREO_CH1 Mux"},
	{"STEREO_CH1", NULL, "STEREO_CH1 PGA"},
	
	/*STEREO_CH2 Mux*/
	{"STEREO_CH2 Mux", "AUX1", "AUX1"},
	{"STEREO_CH2 Mux", "AUX2", "AUX2"},
	{"STEREO_CH2 Mux", "AUX3", "AUX3"},
	{"STEREO_CH2 Mux", "DAC1", "DAC1"},
	{"STEREO_CH2 Mux", "DAC2", "DAC2"},
	{"STEREO_CH2 Mux", "DAC3", "DAC3"},

	{"STEREO_CH2 PGA", NULL, "STEREO_CH2 Mux"},
	{"STEREO_CH2", NULL, "STEREO_CH2 PGA"},

	/*TX Mux*/
	{"TX Mux", "AUX1", "AUX1"},
	{"TX Mux", "AUX2", "AUX2"},
	{"TX Mux", "AUX3", "AUX3"},
	{"TX Mux", "DAC1", "DAC1"},
	{"TX Mux", "DAC2", "DAC2"},
	{"TX Mux", "Mic1", "MIC PGA"},
	{"TX Mux", "Mic2", "MIC PGA"},

	{"ADC", NULL, "TX Mux"},
	{"MIC PGA", NULL, "MIC1"},
	{"MIC PGA", NULL, "MIC2"},

	{NULL, NULL, NULL},
};


static const struct snd_kcontrol_new micco_snd_controls[] = {
	SOC_SINGLE("Mono Volume", MICCO_MONO_VOL, 0, 0x3f, 1),
	SOC_SINGLE("Bear Volume", MICCO_BEAR_VOL, 0, 0x3f, 1),
	SOC_SINGLE("Line Out Volume", MICCO_AUDIO_LINE_AMP, 0, 0xf, 1),
	SOC_SINGLE("Stereo Ch1 Volume", MICCO_STEREO_AMPLITUDE_CH1, 0, 0x3f, 1),
	SOC_SINGLE("Stereo Ch2 Volume", MICCO_STEREO_AMPLITUDE_CH2, 0, 0x3f, 1),
	SOC_SINGLE("Mic Volume", MICCO_MIC_PGA, 0, 0x7, 0),
	SOC_SINGLE("AUX1 Volume", MICCO_PGA_AUX1_2, 0, 0x3, 0),
	SOC_SINGLE("AUX2 Volume", MICCO_PGA_AUX1_2, 4, 0x3, 0),
	SOC_SINGLE("AUX3 Volume", MICCO_PGA_AUX3, 0, 0x3, 0),
	SOC_SINGLE("TX Volume", MICCO_TX_PGA, 1, 0xf, 0),
	SOC_SINGLE("Mic Volume", MICCO_MIC_PGA, 0, 0x7, 0),
};


static int micco_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(micco_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&micco_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int micco_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(micco_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &micco_dapm_widgets[i]);
	}

	/* set up audio path audio_map nects */
	for(i = 0; micco_audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, micco_audio_map[i][0],
			micco_audio_map[i][1], micco_audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(codec);
	return 0;
}



static unsigned int micco_soc_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg > (ARRAY_SIZE(micco_regs)))
		return -EIO;
	return cache[reg];
}

static int micco_soc_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u8 *cache = codec->reg_cache;
//	unsigned int	valtmp;

	if (reg > (ARRAY_SIZE(micco_regs)))
		return -EIO;
	if (reg <= MICCO_SOFT_START_RAMP) {
		micco_codec_write(reg, val);
//		micco_codec_read(reg, &valtmp);
//		if (val != valtmp)
//			printk(KERN_ERR "Write micco reg 0x%x error!\n", reg);
//			return -EIO;
	}
	cache[reg] = val;
 	return 0;
}

static int micco_reset(struct snd_soc_codec *codec, int try_warm)
{
	unsigned int reg;

	for (reg=0; reg<ARRAY_SIZE(micco_regs); reg++) 
		micco_soc_write(codec, reg, micco_regs[reg]);
	return 0;
}

static int micco_voice_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 val;

#ifdef DA9034_ERRATA_17
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		/* workaound for Micco record. */
		micco_write(0x90, 0x01);
		micco_write(0x94, 0x40);
		micco_write(0x90, 0x00);
	}
#endif	

	val = micco_soc_read(codec, MICCO_VCODEC_ADC_CONTROL);
	switch (runtime->rate) {
		case 8000:
			val &= ~(0x03 << 3);
			break;
		case 16000:
			val = (val & (~(0x03 << 3))) | (0x01 << 3);
			break;

		case 32000:
			val |= (0x03 << 3);
			break;
		default:
			return -EINVAL;
	}
	micco_soc_write(codec, MICCO_VCODEC_ADC_CONTROL, val);
	return 0;
}

static int micco_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 val;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		val = micco_soc_read(codec, MICCO_I2S_CONTROL);
		switch (runtime->rate) {
			case 8000:
				val &= 0xF0;
				break;
			case 11025:
				val &= 0xF0;
				val |= 0x01;
				break;
			case 12000:
				val &= 0xF0;
				val |= 0x02;
				break;
			case 16000:
				val &= 0xF0;
				val |= 0x03;
				break;
			case 22050:
				val &= 0xF0;
				val |= 0x04;
				break;
			case 24000:
				val &= 0xF0;
				val |= 0x05;
				break;
			case 32000:
				val &= 0xF0;
				val |= 0x06;
				break;
			case 44100:
				val &= 0xF0;
				val |= 0x07;
				break;
			case 48000:
				val &= 0xF0;
				val |= 0x0F;
				break;
			default:
				return -EINVAL;
		}
		val &= 0x0F;
		val |= 0x10;
		micco_soc_write(codec, MICCO_I2S_CONTROL, val);

	} else {
		printk(KERN_ERR "Micco HIFI does not support capture!\n");
		return -EINVAL;
	}
	return 0;
}



#define MICCO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define MICCO_VOICE_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_32000)

#define MICCO_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

struct snd_soc_codec_dai micco_dai[] = {
{
	.name = "I2S HiFi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MICCO_HIFI_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.ops = {
		.prepare = micco_hifi_prepare,},
	},
	{
	.name = "PCM Voice",
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = MICCO_VOICE_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.capture = {
		.stream_name = "Voice Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = MICCO_VOICE_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.ops = {
		.prepare = micco_voice_prepare,},
	},
};
EXPORT_SYMBOL_GPL(micco_dai);

static int micco_dapm_event(struct snd_soc_codec *codec, int event)
{

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
#ifdef DA9034_ERRATA_17
		micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL, 
			(micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL)|(1<<7)));
#endif	
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
#ifdef DA9034_ERRATA_17
		micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL, 
			(micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL)
			&(~(1<<7))));
#endif	
		if(!golden_muted)
			break;
		
		micco_soc_write(codec, MICCO_AUDIO_LINE_AMP,
			(micco_soc_read(codec, MICCO_AUDIO_LINE_AMP)
			&(~MICCO_AUDIO_LINE_AMP_EN)));

		micco_soc_write(codec, MICCO_STEREO_AMPLITUDE_CH1,
			(micco_soc_read(codec, MICCO_STEREO_AMPLITUDE_CH1)
			&(~MICCO_STEREO_EN)));

		micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL,
			(micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL)
			&(~MICCO_HIFI_DAC_ON)));

		micco_soc_write(codec, MICCO_MONO_VOL,
			(micco_soc_read(codec, MICCO_MONO_VOL)
			&(~MICCO_MONO_EN)));

		micco_soc_write(codec, MICCO_BEAR_VOL,
			(micco_soc_read(codec, MICCO_BEAR_VOL)
			&(~MICCO_BEAR_EN)));

		micco_soc_write(codec, MICCO_MIC_PGA,
			(micco_soc_read(codec, MICCO_MIC_PGA)
			&(~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
				MICCO_MIC_PGA_AMP_EN))));

		micco_soc_write(codec, MICCO_VCODEC_ADC_CONTROL,
			(micco_soc_read(codec, MICCO_VCODEC_ADC_CONTROL)
			&(~MICCO_VCODEC_ADC_ON_EN)));

		micco_soc_write(codec, MICCO_VCODEC_VDAC_CONTROL,
			(micco_soc_read(codec, MICCO_VCODEC_VDAC_CONTROL)
			&(~MICCO_VDAC_ON)));

		micco_soc_write(codec, MICCO_SIDETONE,
			(micco_soc_read(codec, MICCO_SIDETONE)
			&(~MICCO_SIDETONE_EN)));

		micco_soc_write(codec, MICCO_PGA_AUX1_2,
			(micco_soc_read(codec, MICCO_PGA_AUX1_2)
			&(~(MICCO_PGA_AUX1_EN | MICCO_PGA_AUX2_EN))));

		micco_soc_write(codec, MICCO_PGA_AUX3,
			(micco_soc_read(codec, MICCO_PGA_AUX3)
			&(~MICCO_PGA_AUX3_EN)));
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		break;
	}
	codec->dapm_state = event;
	return 0;
}

static int micco_soc_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	micco_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	micco_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int micco_soc_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	u8 *cache = codec->reg_cache;
	unsigned int reg;


	for (reg=0; reg<ARRAY_SIZE(micco_regs); reg++) {
		micco_soc_write(codec, reg, cache[reg]);
	}	

	micco_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	return 0;
}

#ifdef MICCO_SOC_PROC
static ssize_t micco_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			i;
	int 			reg;
	struct snd_soc_codec 	*codec = data;
	unsigned int sscr0, sscr1, sspsp, sstsa;
	unsigned int ssacd, ssacdd, ssrsa;

	t = scnprintf(next, size, "Micco regs: \n");
	size -= t;
	next += t;


	for (i=0; i<ARRAY_SIZE(micco_regs); i++) {
		reg = micco_soc_read(codec, i);
		t = scnprintf(next, size, "[0x%02x]=0x%02x  \n", i, reg);
		size -= t;
		next += t;
	}


	sscr0 = SSCR0_P(3);
	sscr1 = SSCR1_P(3);
	sspsp = SSPSP_P(3);
	sstsa = SSTSA_P(3);
	ssrsa = SSRSA_P(3);
	ssacd = SSACD_P(3);
	ssacdd= SSACDD_P(3);

	t = scnprintf(next, size, "SSP3 regs: \n");
	size -= t;
	next += t;
	t = scnprintf(next, size, "sscr0:0x%x \n", sscr0);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sscr1:0x%x \n", sscr1);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sspsp:0x%x \n", sspsp);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sstsa:0x%x \n", sstsa);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssrsa:0x%x \n", ssrsa);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssacd:0x%x \n", ssacd);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssacdd:0x%x \n", ssacdd);
	size -= t;
	next += t;

	sscr0 = SSCR0_P(4);
	sscr1 = SSCR1_P(4);
	sspsp = SSPSP_P(4);
	sstsa = SSTSA_P(4);
	ssrsa = SSRSA_P(4);
	ssacd = SSACD_P(4);
	ssacdd= SSACDD_P(4);

	t = scnprintf(next, size, "SSP4 regs: \n");
	size -= t;
	next += t;
	t = scnprintf(next, size, "sscr0:0x%x \n", sscr0);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sscr1:0x%x \n", sscr1);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sspsp:0x%x \n", sspsp);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sstsa:0x%x \n", sstsa);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssrsa:0x%x \n", ssrsa);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssacd:0x%x \n", ssacd);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssacdd:0x%x \n", ssacdd);
	size -= t;
	next += t;



	*eof = 1;
	return count - size;
}

static int micco_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[4096];
	char *buf = kbuf;
	struct snd_soc_codec 	*codec = data;
	unsigned int	i, reg, reg2;
	char cmd;

	

	if (count >=4096)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%c 0x%x 0x%x", &cmd, &i, &reg);
	
	if('r' == cmd) {
		if(i>0x16 || (i&1)) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		reg = micco_soc_read(codec, i);
		printk(KERN_INFO "0x[%2x]=0x%2x\n", i, reg);
	}else if('w' == cmd) {
		if(i>0x16) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		if(reg>0xff) {
			printk(KERN_ERR "invalid value!\n");
			goto error;
		}
		micco_soc_write(codec, i, reg);
		reg2 = micco_soc_read(codec, i);
		printk(KERN_INFO 
			"write 0x%2x to 0x[%2x], read back 0x%2x\n", 
			reg, i, reg2);
	}else {
		printk(KERN_ERR "unknow opt!\n");
		goto error;
	}

	return count;
error:
	printk(KERN_INFO "r/w index(0x%%2x) value(0x%%2x)\n");
	return count;	
}
#endif

static int micco_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;
#ifdef MICCO_SOC_PROC	
	struct proc_dir_entry *micco_proc_entry;
#endif
	printk(KERN_INFO "Micco(DA9034) SoC Audio Codec\n");

	socdev->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	
	if (socdev->codec == NULL)
		return -ENOMEM;
	
	codec = socdev->codec;
	mutex_init(&codec->mutex);

	codec->reg_cache = kzalloc(sizeof(u16) * ARRAY_SIZE(micco_regs), 
				GFP_KERNEL);
	if (codec->reg_cache == NULL){
		ret = -ENOMEM;
		goto cache_err;
	}
	memcpy(codec->reg_cache, micco_regs,
		sizeof(u8) * ARRAY_SIZE(micco_regs));
	codec->reg_cache_size = sizeof(u8) * ARRAY_SIZE(micco_regs);
	codec->reg_cache_step = 1;
	codec->private_data = NULL;
	codec->name = "Micco";
	codec->owner = THIS_MODULE;
	codec->dai = micco_dai;
	codec->num_dai = ARRAY_SIZE(micco_dai);
	codec->write = micco_soc_write;
	codec->read = micco_soc_read;
	codec->dapm_event = micco_dapm_event;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto pcm_err;

	ret = micco_reset(codec, 0);
	if (ret < 0) {
		printk(KERN_ERR "Reset Micco error\n");
		goto reset_err;
	}
	micco_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	micco_add_controls(codec);
	micco_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0)
		goto reset_err;

#ifdef MICCO_SOC_PROC	
	micco_proc_entry = create_proc_entry("driver/codec", 0, NULL);
	if (micco_proc_entry) { 
		micco_proc_entry->data = codec; 
		micco_proc_entry->read_proc = micco_proc_read; 
		micco_proc_entry->write_proc = micco_proc_write; 
	} 

#endif
	return 0;

reset_err:
	snd_soc_free_pcms(socdev);

pcm_err:
	snd_soc_free_ac97_codec(codec);

cache_err:
	kfree(socdev->codec);
	socdev->codec = NULL;
	return ret;
}

static int micco_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec == NULL)
		return 0;

	snd_soc_dapm_free(socdev);
	snd_soc_free_pcms(socdev);
	snd_soc_free_ac97_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec->dai);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_micco = {
	.probe = 	micco_soc_probe,
	.remove = 	micco_soc_remove,
	.suspend =	micco_soc_suspend,
	.resume = 	micco_soc_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_micco);

MODULE_DESCRIPTION("ASoC Micco driver");
MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_LICENSE("GPL");

