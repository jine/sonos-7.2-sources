/*
 * linux/sound/arm/audio_codec_control.c.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 18, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/asoundef.h>
#include <sound/control.h>
#include "lt_audio.h"

#if	0
#ifdef	CONFIG_CODEC_AC97_ZY
#include <asm/arch/mhn_audio_plat.h>
#include <asm/arch/codec/acodec.h>
#include <asm/arch/codec/wm9713.h>
#endif

#ifdef	CONFIG_CODEC_MICCO_LT
#endif
#endif

#undef ALSA_ZY_CONTROL_DEBUG
#ifdef ALSA_ZY_CONTROL_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG format "\n",##arg)
#define CODEC_DUMP() alsa_codec_regs_dump(g_acodec_context)
#else
#define dbg(format, arg...)
#define CODEC_DUMP()
#endif

extern audio_info_t audio_info;
extern p_acodec_context_t g_acodec_context;

typedef struct _scenario_buffer
{
	/* the scenario want to set */
	u16 scenario_to_set[16];
	/* the current scenario */
	u16 current_scenario[16];
} scenario_buffer_t;

static u8 current_input_control = 0xff;
static scenario_buffer_t scenario_buffer;

#define CONTROL_PARAMETER(control_type,  control_index, mask, channel_count) \
			(((control_type) & 0xf)		 |		     \
			(((control_index) & 0xff) << 4)	 |		     \
			(((mask) & 0xffff) << 12)	 |		     \
			(((channel_count) & 0xf) << 28))

#define GET_INPUT_CONTROL_VLAUE(control_index)			\
	((scenario_buffer.scenario_to_set[(control_index)])

static int get_other_control_value(int control_index, int * value)
{
	acodec_state_t state = CODEC_WAKEUP;
	acodec_error_t re = ACODEC_SUCCESS;
	switch(control_index)
	{
		case INDEX_SLEEP_CONTROL:
			if(g_acodec_context->get_state) {
				re = g_acodec_context->get_state(g_acodec_context, &state);
				if (ACODEC_SUCCESS == re) {
					*value = state;
				}
			} else {
				printk(KERN_ALERT "Codec can not support get state function \n");
				return -1;
			}
			break;
		case INDEX_3D_CONTROL:
			break;
		default:
			printk(KERN_ALERT "can not support other control: 0x%x\n", control_index);
			return-1;
	}
		return 0;
}
extern int audio_codec_zy_suspend(struct device *_dev, u32 state, u32 level);
static int put_other_control_value(int control_index, int value)
{
	acodec_error_t re = ACODEC_SUCCESS;
	switch(control_index)
	{
		case INDEX_SLEEP_CONTROL:
			if(value){
				if(g_acodec_context->sleep_codec) {
					dbg("sleep the codec\n");
					re = g_acodec_context->sleep_codec(g_acodec_context);
				} else {
					printk(KERN_ALERT "Codec can not support function sleep\n");
					re = -1;
				}/* end g_acodec_context->sleep_codec */
				}else {
				if(g_acodec_context->wake_codec) {
					dbg("wake up the codec\n");
					re = g_acodec_context->wake_codec(g_acodec_context);
				} else {
					printk(KERN_ALERT "Codec can not support function wake up\n");
					re = -1;
				}/* end g_acodec_context->wake_codec */

			}/* end value */
		break;
		case INDEX_3D_CONTROL:
			break;
		default:
			printk(KERN_ALERT "can not support other control: 0x%x\n", control_index);
			re = -1;
		}

	if (ACODEC_SUCCESS != re){
		return -1;
	}
	return 0;
}


static  int get_volume(int control_index, int* value)
{
	acodec_error_t re = ACODEC_SUCCESS;
	if(g_acodec_context->get_vol){
		re = g_acodec_context->get_vol(g_acodec_context, (vol_port_type_t)control_index,(unsigned short*)value);
	}else
	{
		printk(KERN_ALERT "Codec can not support get volume function!\n");
		re =-1;
	}
	if (ACODEC_SUCCESS != re){
		return -1;
	}
	return 0;
}

/**
 * get_control_value - get the control value
 * @control_index: the index of the control
 * @value: the value of the control
 * @type: the control type
 * Returns zero if successful, or a negative error code on failure.
 */
static int get_control_value(int control_index, int * value, CONTROL_TYPE_T type)
{
	int re = 0;
	switch (type) {
		case INPUT_CONTROL:
			*value = scenario_buffer.scenario_to_set[control_index];
			break;
		case OUTPUT_CONTROL:
			*value = scenario_buffer.scenario_to_set[current_input_control] & (1 << control_index);
			break;
		case VOL_CONTROL:
			re = get_volume(control_index, value);
			break;
		case OTHER_CONTROL:
			re = get_other_control_value(control_index,value);
			break;
		default:
			re = -1;
	}
	return re;
}

static int put_vol_value(int control_index, int value)
{
	acodec_error_t re = ACODEC_SUCCESS;
	if(g_acodec_context->set_vol){
		re = g_acodec_context->set_vol(g_acodec_context, (vol_port_type_t)control_index,(unsigned short) value);
	}else
	{
		printk(KERN_ALERT "Codec can not support set volume function!\n");
		re =-1;
	}
	if (ACODEC_SUCCESS != re){
		return -1;
	}
	return 0;

}

static int put_control_value(int control_index, int value, CONTROL_TYPE_T type)
{
	int re = 0;
	u16 tem=0;
	switch (type) {
		case INPUT_CONTROL:
			if(value){
				dbg("set input control:%x -> Xs\n", control_index);
				current_input_control = control_index;
			}else {
				dbg("all the route input control:%x -> Xs\n", control_index);
				scenario_buffer.scenario_to_set[control_index] = 0;
			}
			break;
		case OUTPUT_CONTROL:
			if(current_input_control == 0xff) {
				printk(KERN_ALERT "Please specify the input control first!\n");
				re = -1;
			} else {
				dbg("scenario[0x%x]= 0x%x\n ",current_input_control,\
					scenario_buffer.scenario_to_set[current_input_control]);

				tem = scenario_buffer.scenario_to_set[current_input_control];
				scenario_buffer.scenario_to_set[current_input_control]\
					= (tem & ~(1 << control_index)) | ((!!value) << control_index);

				dbg("scenario[0x%x]= 0x%x\n ",current_input_control,\
					scenario_buffer.scenario_to_set[current_input_control]);
			}
			break;

		case VOL_CONTROL:
			re = put_vol_value(control_index, value);
			break;
		case OTHER_CONTROL:
			re = put_other_control_value(control_index,!!value);
			break;
		default:
			re = -1;
	}
	return re;

}
static int control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	int mask = (kcontrol->private_value >> 12) & 0xffff;
	int channel_count = (kcontrol->private_value >> 28) & 0xf;
	uinfo->type = mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* Some control such as "stereo_head_set out route"  manage 2 channel,
	 * but the configure of the two channels are always the same.
	 */
	uinfo->count = channel_count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;
	return 0;
}

static int control_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	CONTROL_TYPE_T control_type = kcontrol->private_value & 0xf;
	int control_index = (kcontrol->private_value >> 4) & 0xff;
	/* int mask = (kcontrol->private_value >> 12) & 0xffff; */
	int channel_count = (kcontrol->private_value >> 28) & 0xf;
	int val=0;
	int re =0;

	re = get_control_value(control_index, &val, control_type);
	if (re == 0){
		ucontrol->value.integer.value[0] = val ;

		if (channel_count ==2){
			ucontrol->value.integer.value[1] = val ;
		}
	}
	return re;
}

static int control_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int val = 0, err = 0;
	CONTROL_TYPE_T control_type = kcontrol->private_value & 0xf;
	int control_index = (kcontrol->private_value >> 4) & 0xff;
	int mask = (kcontrol->private_value >> 12) & 0xffff;
	val = (ucontrol->value.integer.value[0] & mask);
	dbg("val: %d\n",val);
	err = put_control_value(control_index, val, control_type);
	return err;
}
static int set_scenario_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 16;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =0xffff;
	return 0;
}

static int set_scenario_control_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int i = 0;
	for(i = 0; i < 16; i++) {
		ucontrol->value.integer.value[i] = (long)scenario_buffer.current_scenario[i];
	}
	return 0;
}

static int set_scenario_control_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int i = 0;
	acodec_error_t re = ACODEC_SUCCESS;
	for(i = 0; i < 16; i++) {
		 scenario_buffer.scenario_to_set[i] = (u16)ucontrol->value.integer.value[i];
	}

	re = g_acodec_context->set_route(g_acodec_context,
						scenario_buffer.scenario_to_set,
						scenario_buffer.current_scenario
						);
	if (ACODEC_SUCCESS == re){
		dbg("route set seuccess, so change the current scenario\n");
		memcpy(scenario_buffer.current_scenario, scenario_buffer.scenario_to_set,\
				sizeof(scenario_buffer.scenario_to_set));
		current_input_control = 0xff;
	}else {
		dbg("route set not sucess, so restore the scenario_buffer to current value\n");
			memcpy(scenario_buffer.scenario_to_set, scenario_buffer.current_scenario, \
				sizeof(scenario_buffer.scenario_to_set));
			current_input_control = 0xff;
			re = -1;
	}

	if (ACODEC_SUCCESS != re){
		return -1;
	}
	return 0;
;
}

static int set_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =1;
	return 0;
}

static int set_control_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	CODEC_DUMP();
	return 0;
}

/* pay attention: application should lock the route set period. */
static int set_control_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	unsigned short val = 0;
	acodec_error_t re = ACODEC_SUCCESS;
	if(ucontrol->value.integer.value) {
		val =!!ucontrol->value.integer.value[0] ;
	}
	if(val){
		dbg("set the routes\n");
		re = g_acodec_context->set_route(	g_acodec_context,
							scenario_buffer.scenario_to_set,
							scenario_buffer.current_scenario);
		if (ACODEC_SUCCESS == re){
			dbg("route set seuccess, so change the current scenario\n");
			memcpy(scenario_buffer.current_scenario, scenario_buffer.scenario_to_set,\
				sizeof(scenario_buffer.scenario_to_set));
			current_input_control = 0xff;
			return 0;
		}else {
			dbg("route set not sucess, so restore the scenario_buffer to current value\n");
			memcpy(scenario_buffer.scenario_to_set, scenario_buffer.current_scenario, \
				sizeof(scenario_buffer.scenario_to_set));
			current_input_control = 0xff;
			return -1;
		}
	}else {
		dbg("set 0 to restore the scenario_buffer to current value.\n");
		memcpy(scenario_buffer.scenario_to_set, scenario_buffer.current_scenario, \
				sizeof(scenario_buffer.scenario_to_set));
		current_input_control = 0xff;
		return 0;
	}
}

#define INPUT_CONTROL_ITEM(xname,control_index) \
{.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,\
  .iface = SNDRV_CTL_ELEM_IFACE_HWDEP,\
  .name = (xname),\
  .private_value = CONTROL_PARAMETER(INPUT_CONTROL, (control_index), 0xff, 1),\
  .info = control_info,\
  .get = control_get,\
  .put = control_put,\
}

#define OUTPUT_CONTROL_ITEM(xname,control_index) \
{.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,\
  .iface = SNDRV_CTL_ELEM_IFACE_HWDEP,\
  .name = (xname),\
  .private_value = CONTROL_PARAMETER(OUTPUT_CONTROL, (control_index), 0x1, 1),\
  .info = control_info,\
  .get = control_get,\
  .put = control_put,\
}

/* in order to make OSS mixer can work,
 * iface must be set as SNDRV_CTL_ELEM_IFACE_MIXER
 */
#define VOL_CONTROL_ITEM(xname,control_index,channel_count) \
{.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,\
  .iface = SNDRV_CTL_ELEM_IFACE_MIXER,\
  .name = (xname),\
  .private_value = CONTROL_PARAMETER(VOL_CONTROL, (control_index), 0xff, channel_count),\
  .info = control_info,\
  .get = control_get,\
  .put = control_put,\
}

#define OTHER_CONTROL_ITEM(xname,control_index) \
{.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,\
  .iface = SNDRV_CTL_ELEM_IFACE_HWDEP,\
  .name = (xname),\
  .private_value = CONTROL_PARAMETER(OTHER_CONTROL, (control_index), 0x1,1),\
  .info = control_info,\
  .get = control_get,\
  .put = control_put,\
}

#define MAX_INPUT_CONTROL 11
static struct snd_kcontrol_new input_control[MAX_INPUT_CONTROL] = {
/*0 INDEX_HIFI_NEAR_IN_CONTROL*/
	INPUT_CONTROL_ITEM("hifi_near_in_route", INDEX_HIFI_NEAR_IN_CONTROL ),
/*1 INDEX_VOICE_NEAR_IN_CONTROL*/
	INPUT_CONTROL_ITEM("voice_near_in_route", INDEX_VOICE_NEAR_IN_CONTROL),
/*2 INDEX_BT_HIFI_NEAR_IN_CONTROL*/
 	INPUT_CONTROL_ITEM("bt_hifi_near_in_route", INDEX_BT_HIFI_NEAR_IN_CONTROL),
/*3 INDEX_BT_VOICE_NEAR_IN_CONTROL*/
	INPUT_CONTROL_ITEM("bt_voice_near_in_route",INDEX_BT_VOICE_NEAR_IN_CONTROL),
/*4 INDEX_BT_MIC_IN_CONTROL*/
	INPUT_CONTROL_ITEM("bt_MIC_in_route", INDEX_BT_MIC_IN_CONTROL),
/*5 INDEX_MIC1_IN_CONTROL*/
	INPUT_CONTROL_ITEM("mic1_in_route", INDEX_MIC1_IN_CONTROL),
/*6 INDEX_MIC2_IN_CONTROL */
	INPUT_CONTROL_ITEM("mic2_in_route", INDEX_MIC2_IN_CONTROL),
/*7 INDEX_FAR_IN_CONTROL*/
	INPUT_CONTROL_ITEM("far_in_route",INDEX_FAR_IN_CONTROL),
/*8 INDEX_MIDI_IN_CONTROL*/
	INPUT_CONTROL_ITEM("midi_in_route", INDEX_MIDI_IN_CONTROL),
/*9 INDEX_FM_IN_CONTROL*/
	INPUT_CONTROL_ITEM("fm_in_route", INDEX_FM_IN_CONTROL),
/*10 INDEX_STEREO_IN_CONTROL */
	INPUT_CONTROL_ITEM("stereo_in_route", INDEX_STEREO_IN_CONTROL),
};/* end input_control */

#define MAX_OUTPUT_CONTROL 9
static struct snd_kcontrol_new output_control[MAX_OUTPUT_CONTROL] =
{
/*0 INDEX_STEREO_HEAD_SET_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("stereo_head_set_out_route",INDEX_STEREO_HEAD_SET_OUT_CONTROL),
/*1 INDEX_SPEAKER_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("speaker_out_route",INDEX_SPEAKER_OUT_CONTROL),
/*2 INDEX_HAND_SET_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("hand_set_out_route", INDEX_HAND_SET_OUT_CONTROL),
/*3 INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("bt_stereo_head_set_out_route", INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL),
/*4 INDEX_VOICE_NEAR_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("voice_near_out_route", INDEX_VOICE_NEAR_OUT_CONTROL),
/*5 INDEX_HIFI_NEAR_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("hifi_near_out_route", INDEX_HIFI_NEAR_OUT_CONTROL),
/*6 INDEX_FAR_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("far_out_route", INDEX_FAR_OUT_CONTROL),
/*7 INDEX_LINEOUT_OUT_CONTROL*/
	OUTPUT_CONTROL_ITEM("line_out_route", INDEX_LINEOUT_CONTROL),
/*8 INDEX_STEREO_OUT_CONTROL */
	OUTPUT_CONTROL_ITEM("stereo_out_route", INDEX_STEREO_OUT_CONTROL),

};/* end output_control[] */

static struct snd_kcontrol_new set_control=
{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_HWDEP,
		.name = "set",
		.private_value = 0,
		.info = set_control_info,
		.get = set_control_get,
		.put = set_control_put,
};

static struct snd_kcontrol_new set_scenario_control=
{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_HWDEP,
		.name = "set_scenario",
		.private_value = 0,
		.info = set_scenario_control_info,
		.get = set_scenario_control_get,
		.put = set_scenario_control_put,
};

#define MAX_VOL_CONTROL 6
static struct snd_kcontrol_new vol_control[MAX_VOL_CONTROL]={
/*0 INDEX_FM_GAIN_CONTROL*/
	VOL_CONTROL_ITEM("fm_vol_gain", INDEX_FM_GAIN_CONTROL, 1),
/*1 INDEX_MIC1_GAIN_CONTROL*/
	VOL_CONTROL_ITEM("mic1_vol_gain", INDEX_MIC1_GAIN_CONTROL,1),
/*2 INDEX_MIC2_GAIN_CONTROL */
	VOL_CONTROL_ITEM("mic2_vol_gain", INDEX_MIC2_GAIN_CONTROL,1),
/*3 INDEX_SPEAKER_VOL_CONTROL*/
	VOL_CONTROL_ITEM("speaker_vol_volume", INDEX_SPEAKER_VOL_CONTROL,1),
/*4 INDEX_HEADSET_VOL_CONTROL*/
	VOL_CONTROL_ITEM("headset_vol_volume", INDEX_HEADSET_VOL_CONTROL,2),
/*5 INDEX_HANDSET_VOL_CONTROL*/
	VOL_CONTROL_ITEM("handset_vol_volume", INDEX_HANDSET_VOL_CONTROL,1)
};

#define MAX_OTHER_CONTROL 2
static struct snd_kcontrol_new other_control[MAX_VOL_CONTROL]={
/*0 INDEX_SLEEP_CONTROL*/
	OTHER_CONTROL_ITEM("sleep", INDEX_SLEEP_CONTROL),
/*1 INDEX_3D_CONTROL*/
	OTHER_CONTROL_ITEM("sleep", INDEX_3D_CONTROL)
};

/**
 * audio_codec_control_new - create controls
 * @card: the card instance
 * Returns zero if successful, or a negative error code on failure.
 */
int audio_codec_control_new(struct snd_card *card)
{
	int control_index=0, err = 0;

	/* create set control */
	err = snd_ctl_add(
				card,
				snd_ctl_new1(&set_control, NULL)
				);
	if (err < 0) {
		dbg("set control can not be created!\n");
		return err;
	};

	/* create set_scenario control */
	err = snd_ctl_add(
				card,
				snd_ctl_new1(&set_scenario_control, NULL)
				);
	if (err < 0) {
		dbg("set_scenario_control can not be created!\n");
		return err;
	};

	/* create input control */
	for(control_index= 0; control_index < MAX_INPUT_CONTROL ; control_index++) {
		if (SUPPORT_INPUT_CONTROL(audio_info, control_index)) {
			 err = snd_ctl_add(
				card,
				snd_ctl_new1(input_control + control_index, NULL)
				);
			if (err < 0) {
				dbg("input control_index: %d can not be created!\n", control_index);
				return err;
			}
		}/* end SUPPORT_INPUT_CONTROL */
	}/* end for */

	/* create output control */
	for(control_index= 0; control_index < MAX_OUTPUT_CONTROL ; control_index++) {
		if (SUPPORT_OUTPUT_CONTROL(audio_info, control_index)) {
			 err = snd_ctl_add(
				card,
				snd_ctl_new1(output_control + control_index, NULL)
				);
			if (err < 0) {
				dbg("output control_index: %d can not be created!\n", control_index);
				return err;
			}
		}/* end SUPPORT_OUTPUT_CONTROL	*/
	}/* end for */

	/* create volume control */
	for(control_index= 0; control_index < MAX_VOL_CONTROL ; control_index++) {
		if (SUPPORT_VOL_CONTROL(audio_info, control_index)) {
			 err = snd_ctl_add(
				card,
				snd_ctl_new1(vol_control + control_index, NULL)
				);
			if (err < 0) {
				dbg("volume control_index: %d can not be created!\n", control_index);
				return err;
			}
		}/* end SUPPORT_VOL_CONTROL */
	}/* end for */

	/* create other control */
	for(control_index= 0; control_index < MAX_OTHER_CONTROL ; control_index++) {
		if (SUPPORT_OTHER_CONTROL(audio_info, control_index)) {
			 err = snd_ctl_add(
				card,
				snd_ctl_new1(other_control + control_index, NULL)
				);
			if (err < 0) {
				dbg("other control_index: %d can not be created!\n", control_index);
				return err;
			}
		}/* end SUPPORT_OTHER_CONTROL */
	}/* end for */
	return 0;
}/* end audio_codec_control_new() */

