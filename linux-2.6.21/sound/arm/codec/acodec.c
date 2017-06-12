/*
 * linux/sound/arm/codec/acodec.c
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>

#include <asm/arch/codec/acodec.h>
#include <asm/arch/codec/wm9713.h>
#include <asm/arch/codec/ac97acodec.h>
#include <asm/arch/codec/ac97acodecplat.h>


acodec_error_t zy_acodec_init(acodec_context_t *codec_context, int hw_init)
{
	/* set codec specific functions
	 * set mfp for Zylonite platform
	 * call bus init function (AC97, I2S, I2C, SSP)
	 * call specific init of codec
	 */
	acodec_error_t retval = ACODEC_SUCCESS;

	if (codec_context->acodec_id != WM_9713_ID){
		/* on Zylonite, it is Wolfson 9713 codec only */
		return ACODEC_GENERAL_SW_ERR;
	}

	if (1 == hw_init) {
		zy_ac97_acodec_mfp_init(codec_context);
		zy_ac97_acodec_init(codec_context);  /* codec init common to ac97 */
	}

	/* wm9713-specific functions */
	(codec_context->set_master_vol)		= zy_wm9713_set_master_vol;
	(codec_context->set_master_input_gain)	= zy_wm9713_set_master_input_gain;
	(codec_context->get_in_sample_rate)	= zy_wm9713_get_in_sample_rate;
	(codec_context->get_out_sample_rate)	= zy_wm9713_get_out_sample_rate;
	(codec_context->set_in_sample_rate)	= zy_wm9713_set_in_sample_rate;
	(codec_context->set_out_sample_rate)	= zy_wm9713_set_out_sample_rate;
	(codec_context->codec_specific_init)	= zy_wm9713_specific_init;
	(codec_context->codec_specific_dinit)	= zy_wm9713_specific_deinit;
	(codec_context->acodec_read)		= zy_ac97_acodec_read;
	(codec_context->acodec_write)		= zy_ac97_acodec_write;

	(codec_context->hifi_stream_path_enable)	= zy_wm9713_hifi_stream_path_enable;
	(codec_context->hifi_stream_path_disable)	= zy_wm9713_hifi_stream_path_disable;
	(codec_context->recording_path_enable)  	= zy_wm9713_recording_path_enable;
	(codec_context->recording_path_disable)		= zy_wm9713_recording_path_disable;
	(codec_context->voice_path_enable)		= NULL;
	(codec_context->voice_path_disable)		= NULL;
	(codec_context->side_tone_enable)	 	= NULL;
	(codec_context->side_tone_disable)		= NULL;

	(codec_context->head_set_enable)		= zy_wm9713_headset_enable;
	(codec_context->head_set_disable)		= zy_wm9713_headset_disable;
	(codec_context->ear_speaker_enable)		= zy_wm9713_ear_speaker_enable;
	(codec_context->ear_speaker_disable)		= zy_wm9713_ear_speaker_disable;
	(codec_context->lounder_speaker_enable)		= zy_wm9713_louder_speaker_enable;
	(codec_context->lounder_speaker_disable)	= zy_wm9713_louder_speaker_disable;
	(codec_context->mic_enable)			= zy_wm9713_mic_enable;
	(codec_context->mic_disable)			= zy_wm9713_mic_disable;

	(codec_context->set_volume_table)		= NULL;
	(codec_context->get_volume_table)		= NULL;
	(codec_context->set_tone_table)			= NULL;
	(codec_context->get_tone_table)			= NULL;
	(codec_context->set_route)			= zy_wm9713_set_route;
	(codec_context->sleep_codec) 			= NULL;
	(codec_context->wake_codec) 			= NULL;
	(codec_context->get_state) 			= NULL;

	(codec_context->get_vol) 			= zy_wm9713_get_vol;
	(codec_context->set_vol) 			= zy_wm9713_set_vol;
	(codec_context->get_voice_in_sample_rate)	= NULL;
	(codec_context->set_voice_in_sample_rate)	= zy_wm9713_set_voice_in_sample_rate;
	(codec_context->set_voice_out_sample_rate)	= zy_wm9713_set_voice_out_sample_rate;
	(codec_context->voice_prepare)			= zy_wm9713_voice_prepare;

	(codec_context->event_ack)			= zy_wm9713_event_ack;
	(codec_context->get_event)			= zy_wm9713_get_event;
	(codec_context->disable_touch)			= zy_wm9713_disable_touch;
	(codec_context->enable_touch)			= zy_wm9713_enable_touch;

	if (1 == hw_init) {
		retval = codec_context->codec_specific_init(codec_context);
	}

	return retval;
}

acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context)
{
	/* power down codec by codec specific power down function */
	if (codec_context->codec_specific_dinit) {
		codec_context->codec_specific_dinit(codec_context);
	}

	/* call bus deinit function */
	zy_ac97_acodec_deinit(codec_context);
	/* restore MFP, set GPIO to suitable value */
	zy_ac97_acodec_mfp_deinit(codec_context);

	return ACODEC_SUCCESS;
}

acodec_error_t zy_acodec_pcm_bus_enable(acodec_context_t *codec_context)
{
	/* do later: PCM Bus is only used for BT audio */
	return ACODEC_SUCCESS;
}

acodec_error_t zy_acodec_pcm_disable(acodec_context_t *codec_context)
{
	/* do later: PCM Bus is only used for BT audio */
	return ACODEC_SUCCESS;
}

acodec_error_t zy_acodec_enable_amplifier(acodec_context_t *codec_context)
{
	/* no implementation on Zylonite. WM9713 has internal
	 * amplifier, so no board amplifier needed
	 */
	return ACODEC_SUCCESS;
}

acodec_error_t zy_acodec_suspend(acodec_context_t *codec_context)
{
	/* this function is used in OS driver while suspend/resume.
	 * Size of p_save_memory should be
	 * AC97_SAVE_CONTEXT_SIZE + CODEC_CONTEXT_SIZE
	 */

	char *save_memory = (char *)(codec_context->p_save_memory) +
			CODEC_CONTEXT_SIZE;

	/* Save WM9713 register and make wm9713 into sleep mode */
	zy_wm9713_sleep(codec_context);
	/* Save AC97 related functions */
	zy_ac97_acodec_save_context (codec_context,
			(zy_ac97_save_context_t *)save_memory);
	/* do later: Save PCM Bus Status	 */
	/* do later: Power down WM9713 */
	/* Shutdown ACLINK */
	zy_ac97_acodec_deinit (codec_context);
	return ACODEC_SUCCESS;
}

acodec_error_t zy_acodec_resume(acodec_context_t *codec_context)
{
	/* This function is used in OS driver while suspend/resume */
	 zy_ac97_acodec_mfp_init(codec_context);
	/* Warm/Cold Reset ACLink */
	zy_ac97_acodec_init(codec_context);
	/* Restore AC97 related functions */
	zy_ac97_acodec_restore_context(codec_context,
		(zy_ac97_save_context_t *) ((unsigned char *)(codec_context->p_save_memory) + CODEC_CONTEXT_SIZE) );
	/* Restore WM9713 register wake up wm9713 */
	zy_wm9713_wakeup(codec_context);
	/* do later: Restore PCM Bus Status */
	return ACODEC_SUCCESS;
}





