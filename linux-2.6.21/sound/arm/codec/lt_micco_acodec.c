/*
 * linux/sound/arm/codec/littleton_micco_acodec.c
 *  
 *  Author:	fengwei.yin@marvell.com
 *  Created:	Nov 20, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/* Changlog:
 * 03-09-2007	<Yin, Fengwei> 
 * 		1. Mov the bus init/deinit code out of codec.
 *		   We want to make bus init/deinit NOT relate with codec.
 *		2. clean code style.
 */
#include <linux/module.h>
#include <linux/init.h>

#include <asm/arch/micco.h>
#include <asm/arch/codec/lt_micco_acodec.h>
#include <asm/arch/codec/lt_micco.h>

int lt_acodec_init(acodec_context_t *codec_context, int hw_init)
{
	/* set codec specific functions
	 * call specific init of codec
	 */

	int retval = 0;

	/* littleton micco specific functions */
	(codec_context->set_master_vol)		= lt_micco_set_master_vol;
	(codec_context->set_master_input_gain)	= lt_micco_set_master_input_gain;
	(codec_context->get_in_sample_rate)	= NULL;
	(codec_context->get_out_sample_rate)	= lt_micco_get_hifi_sample_rate;
	(codec_context->set_in_sample_rate)	= NULL;
	(codec_context->set_out_sample_rate)	= lt_micco_set_hifi_sample_rate;
	(codec_context->codec_specific_init)	= lt_micco_specific_init;
	(codec_context->codec_specific_dinit)	= lt_micco_specific_deinit;
	(codec_context->acodec_read)		= NULL;
	(codec_context->acodec_write)		= NULL;

	/* Yin, Fengwei
	 * Not sure what do these specific router enable functions for.
	 * TODO: 
	 *	Check ASOC implementation in future Linux kernel.
	 */
	(codec_context->hifi_stream_path_enable)	= lt_micco_hifi_path_enable;
	(codec_context->hifi_stream_path_disable)	= lt_micco_hifi_path_disable;
	(codec_context->recording_path_enable)	= lt_micco_recording_path_enable;
	(codec_context->recording_path_disable)	= lt_micco_recording_path_disable;
	(codec_context->voice_path_enable)	= NULL;
	(codec_context->voice_path_disable)	= NULL;
	(codec_context->side_tone_enable)		= NULL;
	(codec_context->side_tone_disable)	= NULL;

	(codec_context->head_set_enable)		= lt_micco_headset_enable;
	(codec_context->head_set_disable)		= lt_micco_headset_disable;
	(codec_context->ear_speaker_enable)	= lt_micco_ear_speaker_enable;
	(codec_context->ear_speaker_disable)	= lt_micco_ear_speaker_disable;
	(codec_context->lounder_speaker_enable)	= lt_micco_louder_speaker_enable;
	(codec_context->lounder_speaker_disable)	= lt_micco_louder_speaker_disable;
	(codec_context->mic_enable)		= lt_micco_handset_mic_enable;
	(codec_context->mic_disable)		= lt_micco_handset_mic_disable;

	(codec_context->set_volume_table)		= NULL;
	(codec_context->get_volume_table)		= NULL;
	(codec_context->set_tone_table)		= NULL;
	(codec_context->get_tone_table)		= NULL;
	(codec_context->set_route)		= lt_micco_set_route;
	(codec_context->sleep_codec)		= NULL;
	(codec_context->wake_codec)		= NULL;
	(codec_context->get_state)		= NULL;

	(codec_context->get_vol)			= lt_micco_get_vol;
	(codec_context->set_vol)			= lt_micco_set_vol;
	(codec_context->get_voice_in_sample_rate)	= NULL;
	(codec_context->set_voice_in_sample_rate)	= lt_micco_set_voice_sample_rate;
	(codec_context->set_voice_out_sample_rate)= lt_micco_set_voice_sample_rate;
	(codec_context->voice_prepare)		= lt_micco_voice_prepare;

	if (1 == hw_init)
	{
		retval = codec_context->codec_specific_init(codec_context);
	}

	return retval;
}

int lt_acodec_deinit(acodec_context_t *codec_context)
{
	/* power down codec by codec specific power down function */
	if (codec_context->codec_specific_dinit)
	{
		codec_context->codec_specific_dinit(codec_context);
	}

	return 0;
}

static volatile u8 micco_audio_regs[MICCO_AUDIO_REGS_NUM];

void micco_save_codec(void)
{
	int i;

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++)
		micco_read(MICCO_AUDIO_REG_BASE + i, &micco_audio_regs[i]);
}

void micco_restore_codec(void)
{
	int i;

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++)
		micco_write(MICCO_AUDIO_REG_BASE + i, micco_audio_regs[i]);
}

/* For Micco codec suspend/resume, we depend on Micco driver to do it for us.
 * TODO:
 *	call acodec_init/deinit here may be not a good chioce here because
 *	the router/amplifier information will be lost. We need use codec 
 *	suspend/resume routine replace them.
 */
int lt_acodec_suspend(acodec_context_t *codec_context)
{
	unsigned char val;

	micco_save_codec();

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_AUDIO_LINE_AMP, &val);
	val &= ~MICCO_AUDIO_LINE_AMP_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_AUDIO_LINE_AMP, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_STEREO_AMPLITUDE_CH1, &val);
	val &= ~MICCO_STEREO_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_STEREO_AMPLITUDE_CH1, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_HIFI_DAC_CONTROL, &val);
	val &= ~MICCO_HIFI_DAC_ON;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_HIFI_DAC_CONTROL, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_MONO_VOL, &val);
	val &= ~MICCO_MONO_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_MONO_VOL, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_BEAR_VOL, &val);
	val &= ~MICCO_BEAR_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_BEAR_VOL, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_MIC_PGA, &val);
	val &= ~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		MICCO_MIC_PGA_AMP_EN);
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_MIC_PGA, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_VCODEC_ADC_CONTROL, &val);
	val &= ~MICCO_VCODEC_ADC_ON_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_VCODEC_ADC_CONTROL, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_VCODEC_VDAC_CONTROL, &val);
	val &= ~MICCO_VDAC_ON;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_VCODEC_VDAC_CONTROL, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_SIDETONE, &val);
	val &= ~MICCO_SIDETONE_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_SIDETONE, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_PGA_AUX1_2, &val);
	val &= ~(MICCO_PGA_AUX1_EN | MICCO_PGA_AUX2_EN);
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_PGA_AUX1_2, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_PGA_AUX3, &val);
	val &= ~MICCO_PGA_AUX3_EN;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_PGA_AUX3, val);

	return ACODEC_SUCCESS;    
}

int lt_acodec_resume(acodec_context_t *codec_context)
{
	micco_restore_codec();
	return ACODEC_SUCCESS;    
}
