/*
 * linux/sound/soc/pxa/littleton.c
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/littleton.h>
#include <asm/arch/audio.h>
#include <asm/arch/micco.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#define DA9034_ERRATA_17	1


#define I2S_SSP_PORT	3	
#define PCM_SSP_PORT	4	

void enable_oscc_pout(void);
void disable_oscc_pout(void);

extern struct snd_soc_codec_dai micco_dai[2];
extern struct snd_soc_codec_device soc_codec_dev_micco;

static int littleton_micco_init(struct snd_soc_codec *codec);

extern int golden_muted;


static int littleton_micco_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_cpu_dai *cpu_dai = machine->cpu_dai;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = SNDRV_PCM_RATE_48000;

	return 0;
}

static int littleton_micco_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_cpu_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_codec_dai *codec_dai = machine->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned long sscr0, sscr1, sspsp, sstsa;
	unsigned long ssacd, ssacdd, ssrsa;


	if(golden_muted) {
		return 0;
	}
	cpu_dai->dai_ops.set_sysclk(cpu_dai, PXA3XX_SSP_CLK_EXT, 13000000, 0);


	/* Because the internal 13M clock will be 10M in D0CS,
	 * we route SSP_CLK to GPIO126(EXT_CLK) and let SSP select
	 * NETWORK CLK as CLK source.
	 * This workaround need an ECO on Littleton mainboard.
	 */
	sscr0 = 0xA1E0003F;
	sscr1 = 0x00701DC0;
	sspsp = 0x40200004;
	sstsa = 0x00000003;
	ssrsa = 0x00000003;
	ssacd = 0x60;
	ssacdd= 0x00000040;
	sscr0 |= 0x00000300;
	
	SSCR0_P(I2S_SSP_PORT) = sscr0;	
	SSCR1_P(I2S_SSP_PORT) = sscr1;
	SSPSP_P(I2S_SSP_PORT) = sspsp;
	SSTSA_P(I2S_SSP_PORT) = sstsa;
	SSRSA_P(I2S_SSP_PORT) = ssrsa;
	SSACD_P(I2S_SSP_PORT) = ssacd;
	SSACDD_P(I2S_SSP_PORT) = ssacdd;

	enable_oscc_pout();

	
	return 0;

}


static void littleton_micco_hifi_shutdown(struct snd_pcm_substream *substream)
{
	if(golden_muted)
		return;

	disable_oscc_pout();
}



static int littleton_micco_voice_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_cpu_dai *cpu_dai = machine->cpu_dai;

	
	cpu_dai->playback.channels_min = 1;
	cpu_dai->playback.channels_max = 1;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_32000;

	return 0;
}




static int littleton_micco_voice_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_cpu_dai *cpu_dai = machine->cpu_dai;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;

	unsigned long sscr0, sscr1, sspsp;
	cpu_dai->dai_ops.set_sysclk(cpu_dai, PXA3XX_SSP_CLK_PLL, 13000000, 0);

	sscr0 = 0x00C0003F;
	sscr1 = 0x00701DC0;
	sspsp = 0x00800085;

	switch (rate) {
		case 8000:
			sscr0 |= (89 << 8);
			break;
		case 16000:
			sscr0 |= (44 << 8);	
			break;
		case 32000:
			sscr0 |= (22 << 8);
			break;
		default:
			return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sscr1 &= ~0x00800000;
	} else {
		sscr1 |= 0x00800000;
	}

	
	SSCR0_P(PCM_SSP_PORT) = sscr0;	
	SSCR1_P(PCM_SSP_PORT) = sscr1;
	SSPSP_P(PCM_SSP_PORT) = sspsp;

	enable_oscc_pout();

	return 0;
}

static void littleton_micco_voice_shutdown(struct snd_pcm_substream *substream)
{
	disable_oscc_pout();
}

/* machine stream operations */
static struct snd_soc_ops littleton_machine_ops[] = {
{
	.startup = littleton_micco_hifi_startup,
	.prepare = littleton_micco_hifi_prepare,
	.shutdown = littleton_micco_hifi_shutdown,
},
{
	.startup = littleton_micco_voice_startup,
	.prepare = littleton_micco_voice_prepare,
	.shutdown = littleton_micco_voice_shutdown,
},
};

/* littleton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link littleton_dai[] = {
{
	.name = "I2S",
	.stream_name = "I2S HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[I2S_SSP_PORT-1],
	.codec_dai = &micco_dai[0],
	.ops = &littleton_machine_ops[0],
	.init = littleton_micco_init,
},
{
	.name = "PCM",
	.stream_name = "PCM Voice",
	.cpu_dai = &pxa3xx_ssp_dai[PCM_SSP_PORT-1],
	.codec_dai = &micco_dai[1],
	.ops = &littleton_machine_ops[1],
},
};

/* littleton audio machine driver */
static struct snd_soc_machine snd_soc_machine_littleton = {
	.name = "littleton",
	.dai_link = littleton_dai,
	.num_links = ARRAY_SIZE(littleton_dai),
};

/* littleton audio subsystem */
static struct snd_soc_device littleton_snd_devdata = {
	.machine = &snd_soc_machine_littleton,
	.platform = &pxa3xx_soc_platform,
	.codec_dev = &soc_codec_dev_micco,
};

static struct platform_device *littleton_snd_device;

static int bb_enable = 0;

static int bb_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =1;
	return 0;
}

static int bb_control_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = bb_enable;
	return 0;
}

/* pay attention: application should lock the route set period. */
static int bb_control_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int i;
	struct snd_soc_codec *codec = micco_dai[1].codec;
	u16 val;
	
		
	if(bb_enable==1 && ucontrol->value.integer.value[0]==0) {
		printk(KERN_INFO "Disable BB!!\n");
		snd_soc_dapm_stream_event(codec, 
			"Voice Playback", SND_SOC_DAPM_STREAM_STOP);
		snd_soc_dapm_stream_event(codec, 
			"Voice Capture", SND_SOC_DAPM_STREAM_STOP);
		codec->write(codec, MICCO_HIFI_DAC_CONTROL, 0x00);
		codec->write(codec, MICCO_MONO_VOL, 0x80);
		codec->write(codec, MICCO_BEAR_VOL, 0x80);
	}
	if(bb_enable==0 && ucontrol->value.integer.value[0]==1) {
		printk(KERN_INFO "Enable BB!!\n");
		SSCR0_P(PCM_SSP_PORT) = 0;	
		SSCR1_P(PCM_SSP_PORT) = 0;
		SSPSP_P(PCM_SSP_PORT) = 0;
		SSTSA_P(PCM_SSP_PORT) = 0;
		SSRSA_P(PCM_SSP_PORT) = 0;
		SSACD_P(PCM_SSP_PORT) = 0;
		SSACDD_P(PCM_SSP_PORT) = 0;

		SSCR0_P(I2S_SSP_PORT) = 0;	
		SSCR1_P(I2S_SSP_PORT) = 0;
		SSPSP_P(I2S_SSP_PORT) = 0;
		SSTSA_P(I2S_SSP_PORT) = 0;
		SSRSA_P(I2S_SSP_PORT) = 0;
		SSACD_P(I2S_SSP_PORT) = 0;
		SSACDD_P(I2S_SSP_PORT) = 0;
	
		snd_soc_dapm_stream_event(codec, 
			"Voice Playback", SND_SOC_DAPM_STREAM_START);
		snd_soc_dapm_stream_event(codec, 
			"Voice Capture", SND_SOC_DAPM_STREAM_START);

#ifdef DA9034_ERRATA_17
		/* workaound for Micco record. */
		micco_write(0x90, 0x01);
		micco_write(0x94, 0x40);
		micco_write(0x90, 0x00);
#endif	
		val = codec->read(codec, MICCO_VCODEC_ADC_CONTROL);
		val &= ~(0x03 << 3);
		codec->write(codec, MICCO_VCODEC_ADC_CONTROL, val);
		codec->write(codec, MICCO_I2S_CONTROL, 0x10);
		codec->write(codec, MICCO_HIFI_DAC_CONTROL, 0x80);
		codec->write(codec, MICCO_MONO_VOL, 0x43);
		codec->write(codec, MICCO_BEAR_VOL, 0x43);
		schedule_timeout_interruptible(msecs_to_jiffies(10));
		
	}
	bb_enable = ucontrol->value.integer.value[0];	
	return 0;
}

static struct snd_kcontrol_new bb_kcontrol=
{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_CARD,
		.name = "BB",
		.private_value = 0,
		.info = bb_control_info,
		.get = bb_control_get,
		.put = bb_control_put,
};


static int mute_all_control_info(struct snd_kcontrol *kcontrol, 
					struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =1;
	return 0;
}

static int mute_all_control_get(struct snd_kcontrol * kcontrol, 
					struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = golden_muted;
	return 0;
}

/* pay attention: application should lock the route set period. */
static int mute_all_control_put(struct snd_kcontrol * kcontrol, 
					struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = micco_dai[1].codec;

	if(golden_muted==1 && ucontrol->value.integer.value[0]==0) {
		
	}
	if(golden_muted==0 && ucontrol->value.integer.value[0]==1) {
		codec->write(codec, MICCO_AUDIO_LINE_AMP, 0x0);
		codec->write(codec, MICCO_STEREO_AMPLITUDE_CH1, 0x7f);
		codec->write(codec, MICCO_HIFI_DAC_CONTROL, 0x0);
		codec->write(codec, MICCO_MONO_VOL, 0xbf);
		codec->write(codec, MICCO_BEAR_VOL, 0xbf);
		codec->write(codec, MICCO_MIC_PGA, 0x0);
		codec->write(codec, MICCO_VCODEC_VDAC_CONTROL, 0xc);		
	}
	golden_muted = ucontrol->value.integer.value[0];	
	return 0;
}

static struct snd_kcontrol_new mute_all_kcontrol=
{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_CARD,
		.name = "Golden Mute",
		.private_value = 0,
		.info = mute_all_control_info,
		.get = mute_all_control_get,
		.put = mute_all_control_put,
};



/*
 * Logic for a Micco as connected on a littleton Device
 */
static int littleton_micco_init(struct snd_soc_codec *codec)
{
	codec->write(codec, MICCO_I2S_CONTROL, 0x10);
	snd_ctl_add(codec->card,
		snd_ctl_new1(&bb_kcontrol, NULL));

	snd_ctl_add(codec->card,
		snd_ctl_new1(&mute_all_kcontrol, NULL));

	return 0;
}


static int __init littleton_init(void)
{
	int ret;

	if (!machine_is_littleton())
		return -ENODEV;

	littleton_snd_device = platform_device_alloc("soc-audio", -1);
	if (!littleton_snd_device)
		return -ENOMEM;

	platform_set_drvdata(littleton_snd_device, &littleton_snd_devdata);
	littleton_snd_devdata.dev = &littleton_snd_device->dev;
	ret = platform_device_add(littleton_snd_device);

	if (ret)
		platform_device_put(littleton_snd_device);



	return ret;
}

static void __exit littleton_exit(void)
{
	platform_device_unregister(littleton_snd_device);
}

module_init(littleton_init);
module_exit(littleton_exit);

/* Module information */
MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("ALSA SoC MICCO Littleton");
MODULE_LICENSE("GPL");
