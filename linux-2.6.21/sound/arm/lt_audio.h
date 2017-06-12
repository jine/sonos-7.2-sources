/*
 * linux/sound/arm/lt_audio.h 
 *
 * Author:	Yin, Fengwei (fengwei.yin@marvell.com)
 * Created:	Nov, 2006
 * Copyright (C) 2006, Marvell Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _AUDIO_LT_H_
#define _AUDIO_LT_H_
#include <asm/arch/codec/lt_micco_acodec.h>

/* support_pcm */
#define AUDIO_CODEC_VOICE_PCM		(1 << 0)
#define AUDIO_CODEC_HIFI_PCM		(1 << 1)
#define SUPPORT_VOICE_PCM(ainfo)	((ainfo).support_pcm	\
		& AUDIO_CODEC_VOICE_PCM)
#define SUPPORT_HIFI_PCM(ainfo)		((ainfo).support_pcm	\
		& AUDIO_CODEC_HIFI_PCM)

/* support_xx_sample_rate */
#define AUDIO_RATE_5512			(1<<0)		/* 5512Hz */
#define AUDIO_RATE_8000			(1<<1)		/* 8000Hz */
#define AUDIO_RATE_11025		(1<<2)		/* 11025Hz */
#define AUDIO_RATE_16000		(1<<3)		/* 16000Hz */
#define AUDIO_RATE_22050		(1<<4)		/* 22050Hz */
#define AUDIO_RATE_32000		(1<<5)		/* 32000Hz */
#define AUDIO_RATE_44100		(1<<6)		/* 44100Hz */
#define AUDIO_RATE_48000		(1<<7)		/* 48000Hz */
#define AUDIO_RATE_64000		(1<<8)		/* 64000Hz */
#define AUDIO_RATE_88200		(1<<9)		/* 88200Hz */
#define AUDIO_RATE_96000		(1<<10)		/* 96000Hz */
#define AUDIO_RATE_176400		(1<<11)		/* 176400Hz */
#define AUDIO_RATE_192000		(1<<12)		/* 192000Hz */

/* volume set controls */
#define INDEX_FM_GAIN_CONTROL		FM
#define INDEX_MIC1_GAIN_CONTROL		MIC1
#define INDEX_MIC2_GAIN_CONTROL		MIC2
#define INDEX_SPEAKER_VOL_CONTROL	SPEAKER
#define INDEX_HEADSET_VOL_CONTROL	HEADSET
#define INDEX_HANDSET_VOL_CONTROL	HANDSET

/* other CONTROLS */
#define INDEX_SLEEP_CONTROL		0
#define INDEX_3D_CONTROL		1

/* input route control */
#define HIFI_NEAR_IN_CONTROL		((u16)1<< INDEX_HIFI_NEAR_IN_CONTROL)
#define VOICE_NEAR_IN_CONTROL		((u16)1<< INDEX_VOICE_NEAR_IN_CONTROL)
#define VOICE_BT_HIFI_NEAR_IN_CONTROL	((u16)1<< INDEX_BT_HIFI_NEAR_IN_CONTROL)
#define BT_VOICE_NEAR_IN_CONTROL	((u16)1<< INDEX_BT_VOICE_NEAR_IN_CONTROL)
#define BT_MIC_IN_CONTROL		((u16)1<< INDEX_BT_MIC_IN_CONTROL)
#define MIC1_IN_CONTROL			((u16)1<< INDEX_MIC1_IN_CONTROL)
#define MIC2_IN_CONTROL			((u16)1<< INDEX_MIC2_IN_CONTROL)
#define FAR_IN_CONTROL			((u16)1<< INDEX_FAR_IN_CONTROL)
#define MIDI_IN_CONTROL			((u16)1<< INDEX_MIDI_IN_CONTROL)
#define FM_IN_CONTROL			((u16)1<< INDEX_FM_IN_CONTROL)
#define	STEREO_IN_CONTROL		((u16)1<< INDEX_STEREO_IN_CONTROL)

/* ouput route control */
#define	STEREO_HEAD_SET_OUT_CONTROL	STEREO_HEAD_SET_OUT_ROUTE 
#define	SPEAKER_OUT_CONTROL		SPEAKER_OUT_ROUTE
#define	HAND_SET_OUT_CONTROL		SPEAKER_OUT_ROUTE
#define	BT_STEREO_HEAD_SET_OUT_CONTROL	BT_STEREO_HEAD_SET_OUT_ROUTE
#define	VOICE_NEAR_OUT_CONTROL		VOICE_NEAR_OUT_ROUTE
#define	HIFI_NEAR_OUT_CONTROL		HIFI_NEAR_OUT_ROUTE
#define	FAR_OUT_CONTROL			FAR_OUT_ROUTE

/* volume set controls */
#define	MIC1_GAIN			(1 << INDEX_MIC1_GAIN_CONTROL)
#define	MIC2_GAIN			(1 << INDEX_MIC2_GAIN_CONTROL)
#define	SPEAKER_VOL			(1 << INDEX_SPEAKER_VOL_CONTROL)
#define	HAND_SET_VOL			(1 << INDEX_HANDSET_VOL_CONTROL)
#define	STEREO_HEAD_SET_VOL		(1 << INDEX_HEADSET_VOL_CONTROL)
#define	FM_GAIN				(1 << INDEX_FM_GAIN_CONTROL)

/* other controls */
#define	SLEEP_CONTROL			(1<<INDEX_SLEEP_CONTROL)
#define	THREE_D_CONTROL			(1<< INDEX_3D_CONTROL)
#define SUPPORT_INPUT_CONTROL(ainfo,idx)	\
	((ainfo).support_input_control & (u16)1 <<idx)
#define SUPPORT_OUTPUT_CONTROL(ainfo,idx)	\
	((ainfo).support_output_control & (u16)1<<idx)
#define SUPPORT_VOL_CONTROL(ainfo,idx)		\
	((ainfo).support_vol_control & 1<<idx)
#define SUPPORT_OTHER_CONTROL(ainfo,idx)	\
	((ainfo).support_other_control & 1<<idx)

typedef struct _audio_info{
	u16 support_pcm;
	u16 support_input_control;
	u16 support_output_control;
	u32 support_vol_control;
	u16 support_other_control;
	u32 support_hifi_input_sample_rate;
	u32 support_voice_input_sample_rate;
	u32 support_hifi_output_sample_rate;
	u32 support_voice_output_sample_rate;
} audio_info_t;

typedef enum {
	UNKNOW_CONTROL = -1,
	INPUT_CONTROL = 0,
	OUTPUT_CONTROL = 1,
	VOL_CONTROL = 2,
	OTHER_CONTROL=3
} CONTROL_TYPE_T;

#if	0
extern void set_card_shortname(snd_card_t *card,
		acodec_context_t *acodec_context);
extern int alsa_prepare_for_lt(acodec_context_t **acodec_context);
extern void alsa_lt_codec_put(acodec_context_t *acodec_context);
extern int audio_codec_lt_do_suspend(snd_card_t *card,
		unsigned int state, acodec_context_t *codec_context);
extern int audio_codec_lt_do_resume(snd_card_t *card,
		unsigned int state, acodec_context_t *codec_context);
#endif
#endif
