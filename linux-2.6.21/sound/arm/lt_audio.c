/*
 *  linux/sound/arm/lt_audio.c
 *  
 *  Author:	Yin, Fengwei
 *  Created:	Nov 27th, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/codec/lt_micco_acodec.h>
#include <asm/arch/codec/lt_micco.h>

#include "lt_audio.h"

audio_info_t audio_info = {
	.support_pcm			= AUDIO_CODEC_HIFI_PCM |
						 AUDIO_CODEC_VOICE_PCM,
	.support_input_control		= MIC1_IN_CONTROL |
						MIC2_IN_CONTROL |
						HIFI_NEAR_IN_CONTROL,
	.support_output_control		= STEREO_HEAD_SET_OUT_CONTROL |
						HIFI_NEAR_OUT_CONTROL,
	.support_vol_control		= MIC1_GAIN |
						STEREO_HEAD_SET_VOL |
						SPEAKER_VOL |
						HAND_SET_VOL |
						MIC2_GAIN,
	.support_other_control		= 0,
	.support_hifi_input_sample_rate	= 0,
	.support_voice_input_sample_rate= SNDRV_PCM_RATE_8000 |
						SNDRV_PCM_RATE_16000 |
						SNDRV_PCM_RATE_32000,
	.support_hifi_output_sample_rate= SNDRV_PCM_RATE_48000,
	.support_voice_output_sample_rate= SNDRV_PCM_RATE_8000 |
						SNDRV_PCM_RATE_16000 |
						SNDRV_PCM_RATE_32000,
};
EXPORT_SYMBOL(audio_info);

/*
 * set_card_shortname- set the shortname for card
 * card: card instance
 * g_acodec_context: context of codec
 */
void set_card_shortname(struct snd_card *card, acodec_context_t *acodec_context)
{
	switch(acodec_context->acodec_id) {
		case WM_9713_ID:
			snprintf(card->shortname,
				sizeof(card->shortname), "WM9713");
			break;
		case WM_9712_ID:
			snprintf(card->shortname,
				sizeof(card->shortname), "WM9712");
			break;
		case MICCO_ID:
			snprintf(card->shortname,
				sizeof(card->shortname), "MICCO");
			break;
		default:
			snprintf(card->shortname,
				sizeof(card->shortname), "unknown codec");
			break;
	}
}

static acodec_context_t *g_acodec_context = NULL;
/*
 * alsa_prepare_for_lt
 * 
 * create and initialize the p_acodec_context_t
 * 
 * **codec_context:
 *	return the data structure p_acodec_context_t 
 *
 */
int alsa_prepare_for_lt(acodec_context_t **acodec_context)
{
	void *p_scenario = NULL;

	if (g_acodec_context) {
		g_acodec_context->use_count++;
		*acodec_context = g_acodec_context;
		return 0;
	}

	g_acodec_context = kzalloc(sizeof(acodec_context_t), GFP_KERNEL);
	if (!g_acodec_context)
		return -ENOMEM;

	g_acodec_context->acodec_id = (acodec_device_id_t) (MICCO_ID);
	g_acodec_context->p_voice_reg = NULL;

	g_acodec_context->p_scenario = p_scenario;

	g_acodec_context->use_count++;
	*acodec_context = g_acodec_context;

	return 0;
}
EXPORT_SYMBOL(alsa_prepare_for_lt);

extern int lt_acodec_deinit(acodec_context_t *codec_context);
void alsa_lt_codec_put(acodec_context_t *acodec_context)
{
	if (!acodec_context)
		return;

	if (--acodec_context->use_count)
		return;

	lt_acodec_deinit(acodec_context);

	if(acodec_context->p_scenario)
		kfree(acodec_context->p_scenario);
}
EXPORT_SYMBOL(alsa_lt_codec_put);

#ifdef CONFIG_PM
extern struct snd_pcm *p_hifi_lt_pcm;
extern struct snd_pcm *p_voice_lt_pcm;
extern int  snd_pcm_suspend_all(struct snd_pcm *pcm);
extern void voice_pcm_suspend(void);
extern void voice_pcm_resume(void);
extern void hifi_pcm_suspend(void);
extern void hifi_pcm_resume(void);
extern int lt_acodec_suspend(acodec_context_t *codec_context);
extern int lt_acodec_resume(acodec_context_t *codec_context);

int audio_codec_lt_do_suspend(struct snd_card *card,
		unsigned int state, acodec_context_t *acodec_context)
{
	if (card) {
		if (card->power_state != SNDRV_CTL_POWER_D3cold) {
			if(p_hifi_lt_pcm)
				snd_pcm_suspend_all(p_hifi_lt_pcm);

			if(p_voice_lt_pcm)
				snd_pcm_suspend_all(p_voice_lt_pcm);

			voice_pcm_suspend();
			hifi_pcm_suspend();

			snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);
		}

		if (0 == (--acodec_context->use_count)) {
			lt_acodec_suspend(acodec_context);
		}
	}

	// printk(KERN_INFO "alsa suspend!\n");
	return 0;
}
EXPORT_SYMBOL(audio_codec_lt_do_suspend);

int audio_codec_lt_do_resume(struct snd_card *card,
		unsigned int state, acodec_context_t *acodec_context)
{
	pr_debug("%s: alsa begin resume!\n", __func__);
	if (++acodec_context->use_count == 1) {
		lt_acodec_resume(acodec_context);
	}

	if (card->power_state != SNDRV_CTL_POWER_D0) {
		snd_power_change_state(card, SNDRV_CTL_POWER_D0);
		hifi_pcm_resume();
		voice_pcm_resume();
		// printk(KERN_INFO "alsa resume!\n");
	}
	return 0;
}
EXPORT_SYMBOL(audio_codec_lt_do_resume);
#endif

int set_codec_sub_state(int client, int state)
{
}
EXPORT_SYMBOL(set_codec_sub_state);

