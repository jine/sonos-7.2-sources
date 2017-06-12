/*
 * linux/sound/arm/codec/lt_micco.c
 *  
 *  Author:	fengwei.yin@marvell.com
 *  Created:	Nov 20, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/delay.h>

#include <asm/arch/micco.h>

#include <asm/arch/codec/lt_micco_acodec.h>
#include <asm/arch/codec/lt_micco.h>

#undef	DEBUG

struct micco_audio_setting {
	int		headset_vol;
	int		mic1_gain;
	int		mic2_gain;
	int		speaker_vol;
	unsigned int	hifi_sample_rate;
	unsigned int 	pcm_sample_rate;

};

static struct micco_audio_setting audio_setting;

acodec_error_t lt_micco_set_master_vol(acodec_context_t *codec_context,
		unsigned short gain_in_db)
{
	/* There is no master on MICCO */
	return ACODEC_SUCCESS;
}

/* We suppose the MIC1, 2 is the master input */
acodec_error_t lt_micco_set_master_input_gain(acodec_context_t *codec_context,
		unsigned short gain_in_db)
{

	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_get_hifi_sample_rate(acodec_context_t *codec_context,
		unsigned short * rate_in_hz)
{
	acodec_error_t status = ACODEC_SUCCESS;
	*rate_in_hz = audio_setting.hifi_sample_rate;

	return status;
}

acodec_error_t lt_micco_set_hifi_sample_rate(acodec_context_t *codec_context,
		unsigned short rate_in_hz)
{
	/* On littleton, we just support 16k, 44.1k and 48k */
	int ret;

	pr_debug("%s: enter with rate: %d\n", __func__, rate_in_hz);
	if ((rate_in_hz != 16000) && (rate_in_hz != 44100) &&
			(rate_in_hz != 48000)) {
		return ACODEC_SAMPLERATE_NOT_SUPPORTED;
	}

	ret = micco_codec_set_sample_rate(MICCO_HIFI_PORT, rate_in_hz);
	if (ret) {
		printk(KERN_ERR "%s: set hifi sample rate failed!\n", __func__);
		return ACODEC_SAMPLERATE_NOT_SUPPORTED;
	}

	audio_setting.hifi_sample_rate = rate_in_hz;
	return ret;
}

acodec_error_t lt_micco_voice_prepare(acodec_context_t *codec_context)
{
	acodec_error_t status = ACODEC_SUCCESS;
	return status;
}

/* Voice PCM channel on Micco */
acodec_error_t lt_micco_set_voice_sample_rate(acodec_context_t *codec_context,
		unsigned short rate_in_hz)
{
	int ret;
	acodec_error_t status = ACODEC_SUCCESS;

	pr_debug("%s: enter\n", __func__);

	ret = micco_codec_set_sample_rate(MICCO_VOICE_PORT, rate_in_hz);
	if (ret)
		return ACODEC_SAMPLERATE_NOT_SUPPORTED;

	audio_setting.pcm_sample_rate = rate_in_hz;
	return status;
}

acodec_error_t lt_micco_get_voice_sample_rate(acodec_context_t *codec_context,
		unsigned short *rate_in_hz)
{
	acodec_error_t status = ACODEC_SUCCESS;

	pr_debug("enter");
	*rate_in_hz = audio_setting.pcm_sample_rate;

	pr_debug("rate_in_hz = %d", *rate_in_hz);
	return status;
}

/* The HIFI playback IN for Micco is DAC1 and DAC2. */
int lt_micco_hifi_pb_in_enable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_enable_input(CODEC_HIFI))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	return status;
}

int lt_micco_hifi_pb_in_disable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_disable_input(CODEC_HIFI))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	return status;
}

/* The HIFI playback OUT for Micco is STEREO. */
int lt_micco_hifi_pb_out_enable (acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_enable_output(CODEC_STEREO))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	return status;
}

int lt_micco_hifi_pb_out_disable (acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_disable_output(CODEC_STEREO))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	return status;
}

/* The STEREO record IN for Micco is Aux1 and Aux2.
 * Micco doesn't support HIFI record functions. So no HIFI record OUT
 */
acodec_error_t lt_micco_stereo_record_in_enable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_enable_input(CODEC_AUX1))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	if (micco_codec_enable_input(CODEC_AUX2))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	return status;
}

acodec_error_t lt_micco_stereo_record_in_disable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_disable_input(CODEC_AUX1))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	if (micco_codec_disable_input(CODEC_AUX2))
		status = ACODEC_FEATURE_NO_SUPPORTED;

	return status;
}

/* VOICE playback IN is DAC3 on LT*/
acodec_error_t lt_micco_voice_pb_in_enable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_enable_input(CODEC_PCM))
		status = ACODEC_FEATURE_NO_SUPPORTED; 

	return status;
}

acodec_error_t lt_micco_voice_pb_in_disable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_disable_input(CODEC_PCM))
		status = ACODEC_FEATURE_NO_SUPPORTED; 

	return status;
}

/* VOICE record IN is ADC on LT*/
acodec_error_t lt_micco_voice_record_in_enable(acodec_context_t *codec_context)
{
	acodec_error_t status = ACODEC_SUCCESS;

	if (micco_codec_enable_output(CODEC_ADC))
		status = ACODEC_FEATURE_NO_SUPPORTED; 

	return status;
}

acodec_error_t lt_micco_voice_record_in_disable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_disable_output(CODEC_ADC))
		status = ACODEC_FEATURE_NO_SUPPORTED; 

	return status;
}

acodec_error_t lt_micco_lineout_enable(acodec_context_t *codec_context)
{
	acodec_error_t status = ACODEC_SUCCESS;

	if (micco_codec_enable_output(CODEC_LINE_OUT))
		status = ACODEC_FEATURE_NO_SUPPORTED; 

	return status;
}

acodec_error_t lt_micco_lineout_disable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;

	if (micco_codec_disable_output(CODEC_LINE_OUT))
		status = ACODEC_FEATURE_NO_SUPPORTED; 

	return status;
}


/* FIXME:
 *	Should we remove these path enable function when we have router
 *	setting funtion?
 *	Maybe we need keep typical path functions for general usage model
 *	to let them can enable/disable patch through /proc or /sys filesystem.
 */
acodec_error_t lt_micco_recording_path_enable (acodec_context_t *codec_context)
{
	int ret = ACODEC_SUCCESS;
	u8 val;

	/* enable the handset mic */
	ret = micco_codec_enable_input(CODEC_MIC1);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;

	/* enable the voice record in (ADC) */
	ret = lt_micco_voice_record_in_enable(codec_context);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;

	/* route the MIC1 to ADC */
	ret = micco_codec_read(MICCO_TX_PGA_MUX, &val);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;

	val |= 0x0C;	/* Enable P1, P2 */
	ret = micco_codec_write(MICCO_TX_PGA_MUX, val);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;

	ret = micco_codec_read(MICCO_MIC_PGA, &val);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;

	val &= ~MICCO_MIC_PGA_SELMIC_2;	/* Enable M1 */
	ret = micco_codec_write(MICCO_MIC_PGA, val);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;
	
	return ret;
}

acodec_error_t lt_micco_recording_path_disable (acodec_context_t *codec_context)
{
	int ret = ACODEC_SUCCESS;

	/* disable the handset mic */
	ret = micco_codec_disable_input(CODEC_MIC1);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;

	/* disable the voice record in (ADC) */
	ret = lt_micco_voice_record_in_disable(codec_context);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;
	return ret;
}
/* Recording path end here */

/* Need more information on this. */
acodec_error_t lt_micco_hifi_path_enable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	return status;
}

acodec_error_t lt_micco_hifi_path_disable(acodec_context_t *codec_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	return status;
}
/* hifi path end here */

/* LT headset out --> BEAR, in --> MIC2 */
acodec_error_t lt_micco_headset_enable (acodec_context_t *codec_context)
{
	int ret;

	ret = micco_codec_enable_input(CODEC_MIC2);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;

	ret = micco_codec_enable_output(CODEC_BEAR);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;
	
	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_headset_disable (acodec_context_t *codec_context)
{
	int ret;

	ret = micco_codec_disable_input(CODEC_MIC2);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;

	ret = micco_codec_disable_output(CODEC_BEAR);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;
	
	return ACODEC_SUCCESS;
}

/* LT ear speaker --> BEAR */
acodec_error_t lt_micco_ear_speaker_enable (acodec_context_t *codec_context)
{
	int ret;
	ret = micco_codec_enable_output(CODEC_BEAR);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;
	
	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_ear_speaker_disable (acodec_context_t *codec_context)
{
	int ret;
	ret = micco_codec_disable_output(CODEC_BEAR);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;
	
	return ACODEC_SUCCESS;
}

/* LT: louder speaker --> MONO */
acodec_error_t lt_micco_louder_speaker_enable (acodec_context_t *codec_context)
{
	int ret;

	ret = micco_codec_enable_output(CODEC_MONO);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;
	
	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_louder_speaker_disable (acodec_context_t *codec_context)
{
	int ret;

	ret = micco_codec_disable_output(CODEC_MONO);
	if (ret)
		return ACODEC_FEATURE_NO_SUPPORTED;
	
	return ACODEC_SUCCESS;
}

/* Handset Mic, MIC1 on LT */
acodec_error_t lt_micco_handset_mic_enable (acodec_context_t *codec_context,
		unsigned char mic_type)
{
	int ret;

	ret = micco_codec_enable_input(CODEC_MIC1);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;

	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_handset_mic_disable (acodec_context_t *codec_context,
		unsigned char mic_type)
{
	int ret;

	ret = micco_codec_disable_input(CODEC_MIC1);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;

	return ACODEC_SUCCESS;
}

/* Headset Mic, MIC2 on LT */
int lt_micco_headset_mic_enable (acodec_context_t *codec_context,
		unsigned char mic_type)
{
	int ret;

	ret = micco_codec_enable_input(CODEC_MIC2);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;

	return ACODEC_SUCCESS;
}

int lt_micco_headset_mic_disable (acodec_context_t *codec_context,
		unsigned char mic_type)
{
	int ret;

	ret = micco_codec_disable_input(CODEC_MIC2);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;

	return ACODEC_SUCCESS;
}

/* On Littleton, the headset is connect to BEAR.
 * The gain of the headset is from 15.0dB ~ -60dBA.
 * The gain_in_db range is 0 ~ 51. And 0 will mute the headset.
 */
int lt_micco_set_headset_vol(acodec_context_t *codec_context,
		unsigned short gain_in_db)
{
	int ret;
	unsigned short vol;
	acodec_error_t status = ACODEC_SUCCESS;

	if ((gain_in_db < (unsigned short) BEAR_MIN_VOL) ||	\
			(gain_in_db > BEAR_MAX_VOL)) {
		return -EINVAL;
	}

	/* The vol value BEAR_MAX_VOL should be mapped to 0,
	 * the vol value 0 should be mapped to BEAR_MAX_VOL.
	 */
	vol = BEAR_MAX_VOL - gain_in_db;

	ret = micco_codec_set_output_vol(CODEC_BEAR, vol);
	if (ret)
		status = ACODEC_FEATURE_NO_SUPPORTED;

	audio_setting.headset_vol = gain_in_db;
	return status;    
}

/* On Littleton, the speaker is connect to MONO.
 * The gain of the speaker is from 15.0dB ~ -60dBA.
 * The gain_in_db range is 0 ~ 51. And 0 will mute the speaker.
 */
int lt_micco_set_speaker_vol(acodec_context_t *codec_context,
		unsigned short gain_in_db)
{
	int ret;
	unsigned short vol;
	acodec_error_t status = ACODEC_SUCCESS;

	if ((gain_in_db < (unsigned short) MONO_MIN_VOL) ||	\
			(gain_in_db > MONO_MAX_VOL)) {
		return -EINVAL;
	}

	/* The vol value MONO_MAX_VOL should be mapped to 0,
	 * the vol value 0 should be mapped to MONO_MAX_VOL.
	 */
	vol = MONO_MAX_VOL - gain_in_db;

	ret = micco_codec_set_output_vol(CODEC_MONO, vol);
	if (ret)
		status = ACODEC_FEATURE_NO_SUPPORTED;

	audio_setting.speaker_vol = gain_in_db;
	return status;    
}

/* On Littleton, the MIC gain is from 0dB ~ 30dB.
 * the gain_in_db range is 0 ~ 7.
 */
int lt_micco_set_mic1_gain(acodec_context_t *p_devcie_context,
		unsigned short gain_in_db)
{
	int ret;
	int vol;

	vol = gain_in_db;
	if ((vol > (unsigned short) MIC_PGA_MAX_GAIN) ||	\
			(vol < (unsigned short) MIC_PGA_MIN_GAIN))
		return -EINVAL;

	ret = micco_codec_set_input_gain(CODEC_MIC1, vol);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;
	audio_setting.mic1_gain = gain_in_db;

	return ACODEC_SUCCESS;
}

int lt_micco_set_mic2_gain(acodec_context_t *p_devcie_context, signed short gain_in_db)
{
	int ret;
	int vol;

	vol = gain_in_db;
	if ((vol > (unsigned short) MIC_PGA_MAX_GAIN) ||	\
			(vol < (unsigned short) MIC_PGA_MIN_GAIN))
		return -EINVAL;

	ret = micco_codec_set_input_gain(CODEC_MIC1, vol);
	if (ret)
		return 	ACODEC_FEATURE_NO_SUPPORTED;
	audio_setting.mic2_gain = gain_in_db;

	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_get_vol(acodec_context_t *codec_context,
		vol_port_type_t port,  unsigned short *gain_in_db)
{

	switch(port){
		case MIC1:
			*gain_in_db = audio_setting.mic1_gain;
			break;
		case MIC2:
			*gain_in_db = audio_setting.mic2_gain;
			break;
		case HEADSET:
			*gain_in_db = audio_setting.headset_vol;
			break;
		case HANDSET:
			*gain_in_db = audio_setting.mic1_gain;
			break;
		case SPEAKER:
			*gain_in_db = audio_setting.speaker_vol;
			break;
		default:
			return ACODEC_FEATURE_NO_SUPPORTED;
	}
	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_set_vol(acodec_context_t *codec_context,
		vol_port_type_t port, unsigned short gain_in_db)
{
	acodec_error_t status = ACODEC_SUCCESS;

	switch(port){
		case MIC1:
			status = lt_micco_set_mic1_gain(
					codec_context,gain_in_db);
			break;
		case MIC2:
			status = lt_micco_set_mic2_gain(
					codec_context,gain_in_db);
			break;
		case HEADSET:
			status = lt_micco_set_headset_vol(
					codec_context,gain_in_db);
			break;
		case HANDSET:
			status = lt_micco_set_mic1_gain(
					codec_context,gain_in_db);
			break;
		case SPEAKER:
			status = lt_micco_set_speaker_vol(
					codec_context,gain_in_db);
			break;
		default:
			status = ACODEC_FEATURE_NO_SUPPORTED;
	}
	return status;
}

int lt_micco_save_context (acodec_context_t *codec_context)
{
	return ACODEC_SUCCESS;
}

int lt_micco_restore_context (acodec_context_t *codec_context)
{
	/* check later: need special order to restore codec register */
	return ACODEC_SUCCESS;
}

/* After audio_codec boot up, it should be in sleep mode to save
 * power. So application need wake up audio codec when use it first
 * time.
 */

/*
 * Function: lt_micco_sleep,lt_micco_wakeup
 * Purpose: sleep the codec. We don't have sleep/wakeup function on Micco
 *	 Parameter:
 *	 Returns: acodec_error_t
 */
int lt_micco_sleep(acodec_context_t *codec_context)
{		
	return ACODEC_SUCCESS;
}

int lt_micco_wakeup(acodec_context_t *codec_context)
{	
	return ACODEC_SUCCESS;
}

/* platform specific functions*/
acodec_error_t lt_micco_specific_init (acodec_context_t *codec_context)
{
	/* Add Littleton MICCO specific code here */
	micco_audio_init();
	lt_micco_set_vol(codec_context, MIC1, 7);
	lt_micco_set_vol(codec_context, MIC2, 7);
	lt_micco_set_vol(codec_context, HEADSET, 48);
	lt_micco_set_vol(codec_context, HANDSET, 7);
	lt_micco_set_vol(codec_context, SPEAKER, 48);

	return ACODEC_SUCCESS;
}

acodec_error_t lt_micco_specific_deinit (acodec_context_t *codec_context)
{
	return ACODEC_SUCCESS;
}

int support_scenario(unsigned short *rout_map)
{
	if(rout_map[INDEX_BT_HIFI_NEAR_IN_CONTROL]) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_BT_VOICE_NEAR_IN_CONTROL]) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_BT_MIC_IN_CONTROL]) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_FAR_IN_CONTROL]) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_MIDI_IN_CONTROL]) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_FM_IN_CONTROL]) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_HIFI_NEAR_IN_CONTROL] & 	\
			~(STEREO_HEAD_SET_OUT_ROUTE |	\
			HIFI_NEAR_OUT_ROUTE | 		\
			VOICE_NEAR_OUT_ROUTE)) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_MIC1_IN_CONTROL]& 		\
			~(STEREO_HEAD_SET_OUT_ROUTE |\
			HIFI_NEAR_OUT_ROUTE |	\
			VOICE_NEAR_OUT_ROUTE )) 
		return ACODEC_ROUTE_NO_SUPPORTED;

	return ACODEC_SUCCESS;
}

/* Function: lt_micco_set_route
 * Purpose:  set the codec route
 * Parameter: 
 * 	n_map:	New map. The route map want to set
 *	o_map:	Old map. The current rout map
 * explain:   n_map and o_map
 *	This function use the state of input devices state and ouput devices
 * 	state to to show the codec rout map. More information please refer to
 *	spec [linux-2.6.9-zylonite-alpha2] [ALSA] Subsystem High Level 
 *	Design.doc.
 * Returns: 
 * 	ACODEC_SUCCESS 
 *	ACODEC_ROUTE_NO_SUPPORTED
 */

extern void disable_oscc_pout(void);
extern void enable_oscc_pout(void);

/* The scinaria map of the littleton:
 * We try to align with Zylonite to minimize the supurise to the end user.
 * 
 * Input:
 *	scenario[0]: hifi_near_in. It's the DAC2,1 in of the Micco.
 *	scenario[1]: voice_near_in. It's the DAC3 in of the Micco.
 *	scenario[2]: reserved.
 *	scenario[3]: reserved.
 *	scenario[4]: reserved.
 *	scenario[5]: mic1_in. (Handset Mic on LT)
 *	scenario[6]: mic2_in. (Headset MIC on LT)
 *	scenario[7]: reserved. (Voice for CP on ZY)
 *	scenario[8]: reserved.
 *	scenario[9]: reserved.
 *	scenario[10]:stereo_in. It's the AUX1,2 in of the Micco.
 *
 * Output:
 *	bit0: headset_out. It's the BEAR out of the Micco.
 *	bit1: Speaker. It's the MONO out of the Micco.
 *	bit2: reserved. Not handset out on LT.
 *	bit3: reserved.
 *	bit4: voice_near_out. It's ADC out of the Micco.
 *	bit5: reserved. No hifi record channel on LT.
 *	bit6: reserved.
 *	bit7: line_out. It's LINE out of the Micco.
 *	bit8: stereo_out. It's STEREO_CH1,2 of the Micco.
 */
acodec_error_t lt_micco_set_route(acodec_context_t *ctx,
		unsigned short *n_map, unsigned short* o_map)
{

	u16 o_out_state, n_out_state;
	int mic1_s_chg, mic2_s_chg, mic1_mux, mic2_mux;
	u8 o_mux, n_mux, mic_pga;
	int i;

	printk(KERN_ALERT "%s: enter\n", __func__);

	/* Input */
	/* HIFI_NEAR_IN */
	printk(KERN_ALERT "omap[hifi_near_in]: 0x%08x,"
			"nmap[hifi_near_in]: 0x%08x\n",
			o_map[INDEX_HIFI_NEAR_IN_CONTROL],
			n_map[INDEX_HIFI_NEAR_IN_CONTROL]);

	o_out_state=n_out_state=0;
	for (i=0;i<=INDEX_STEREO_IN_CONTROL;i++) {
		o_out_state |= o_map[i];
		n_out_state |= n_map[i];
	}
	if ((o_out_state==0)&&(n_out_state!=0)) {
		printk("enable_oscc_pout\n");
		enable_oscc_pout();
	}
	if ((o_out_state!=0)&&(n_out_state==0)) {
		printk("disable_oscc_pout\n");
		disable_oscc_pout();
	}

	if ((o_map[INDEX_HIFI_NEAR_IN_CONTROL] == 0) && 
		(n_map[INDEX_HIFI_NEAR_IN_CONTROL] != 0)) {
		/* HIFI Near In is opened */
		lt_micco_hifi_pb_in_enable(ctx);
	} else if ((o_map[INDEX_HIFI_NEAR_IN_CONTROL] != 0) &&
			(n_map[INDEX_HIFI_NEAR_IN_CONTROL] == 0)) {
		lt_micco_hifi_pb_in_disable(ctx);
	}

	/* VOICE_NEAR_IN */
	printk(KERN_ALERT "omap[voice_near_in]: 0x%08x,"
			"nmap[voice_near_in]: 0x%08x\n",
			o_map[INDEX_VOICE_NEAR_IN_CONTROL],
			n_map[INDEX_VOICE_NEAR_IN_CONTROL]);
	if ((o_map[INDEX_VOICE_NEAR_IN_CONTROL] == 0) &&
		(n_map[INDEX_VOICE_NEAR_IN_CONTROL] != 0)) {
		/* Voice Near In is opened */
		lt_micco_voice_pb_in_enable(ctx);
	} else if ((o_map[INDEX_VOICE_NEAR_IN_CONTROL] != 0) &&
			(n_map[INDEX_VOICE_NEAR_IN_CONTROL] == 0)) {
		lt_micco_voice_pb_in_disable(ctx);

	}

	/* MIC1 IN */
	printk(KERN_ALERT "omap[mic1_in]: 0x%08x,"
			"nmap[mic1_in]: 0x%08x\n",
			o_map[INDEX_MIC1_IN_CONTROL],
			n_map[INDEX_MIC1_IN_CONTROL]);
	if ((o_map[INDEX_MIC1_IN_CONTROL] == 0) &&
		(n_map[INDEX_MIC1_IN_CONTROL] != 0)) {
		/* handset MIC is opened */
		lt_micco_handset_mic_enable(ctx, 0);
	} else if ((o_map[INDEX_MIC1_IN_CONTROL] != 0) &&
			(n_map[INDEX_MIC1_IN_CONTROL] == 0)) {
		lt_micco_handset_mic_disable(ctx, 0);

	}

	/* MIC2 IN */
	printk(KERN_ALERT "omap[mic2_in]: 0x%08x,"
			"nmap[mic2_in]: 0x%08x\n",
			o_map[INDEX_MIC2_IN_CONTROL],
			n_map[INDEX_MIC2_IN_CONTROL]);
	if ((o_map[INDEX_MIC2_IN_CONTROL] == 0) &&
		(n_map[INDEX_MIC2_IN_CONTROL] != 0)) {
		/* Headset Mic is opened */
		lt_micco_headset_mic_enable(ctx, 0);
	} else if ((o_map[INDEX_MIC2_IN_CONTROL] != 0) &&
			(n_map[INDEX_MIC2_IN_CONTROL] == 0)) {
		lt_micco_headset_mic_disable(ctx, 0);
	}

	/* STEREO IN */
	printk(KERN_ALERT "omap[stereo_in]: 0x%08x,"
			"nmap[stereo_in]: 0x%08x\n",
			o_map[INDEX_STEREO_IN_CONTROL],
			n_map[INDEX_STEREO_IN_CONTROL]);
	if ((o_map[INDEX_STEREO_IN_CONTROL] == 0) &&
		(n_map[INDEX_STEREO_IN_CONTROL] != 0)) {
		/* Stereo In is opened */
		lt_micco_stereo_record_in_enable(ctx);
	} else if ((o_map[INDEX_STEREO_IN_CONTROL] != 0) &&
			(n_map[INDEX_STEREO_IN_CONTROL] == 0)) {
		lt_micco_stereo_record_in_disable(ctx);
	}
	/* Input end here */

	/* Output */
	/* Headset out */
	o_out_state = n_out_state = 0;
	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		o_out_state |= (o_map[i] & STEREO_HEAD_SET_OUT_ROUTE);
	}

	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		n_out_state |= (n_map[i] & STEREO_HEAD_SET_OUT_ROUTE);
	}

	if ((o_out_state == 0) && (n_out_state != 0)) {
		/* Headset output is on */
		lt_micco_ear_speaker_enable(ctx);
	}

	if ((o_out_state != 0) && (n_out_state == 0)) {
		/* Headset output is off */
		lt_micco_ear_speaker_disable(ctx);
	}

	/* Speaker out */
	o_out_state = n_out_state = 0;
	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		o_out_state |= (o_map[i] & SPEAKER_OUT_ROUTE);
	}

	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		n_out_state |= (n_map[i] & SPEAKER_OUT_ROUTE);
	}

	if ((o_out_state == 0) && (n_out_state != 0)) {
		/* Speaker is on */
		lt_micco_louder_speaker_enable(ctx);
	}

	if ((o_out_state != 0) && (n_out_state == 0)) {
		/* Speaker is off */
		lt_micco_louder_speaker_disable(ctx);
	}

	/* Voice near out */
	o_out_state = n_out_state = 0;
	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		o_out_state |= (o_map[i] & VOICE_NEAR_OUT_ROUTE);
	}

	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		n_out_state |= (n_map[i] & VOICE_NEAR_OUT_ROUTE);
	}

	if ((o_out_state == 0) && (n_out_state != 0)) {
		/* Voice is on */
		lt_micco_voice_record_in_enable(ctx);
	}

	if ((o_out_state != 0) && (n_out_state == 0)) {
		/* Voice is off */
		lt_micco_voice_record_in_disable(ctx);
	}

	/* Line out */
	o_out_state = n_out_state = 0;
	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		o_out_state |= (o_map[i] & LINE_OUT_ROUTE);
	}

	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		n_out_state |= (n_map[i] & LINE_OUT_ROUTE);
	}

	if ((o_out_state == 0) && (n_out_state != 0)) {
		/* Lineout is on */
		lt_micco_lineout_enable(ctx);
	}

	if ((o_out_state != 0) && (n_out_state == 0)) {
		/* Lineout is off */
		lt_micco_lineout_enable(ctx);
	}

	/* Stereo out */
	o_out_state = n_out_state = 0;
	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		o_out_state |= (o_map[i] & STEREO_OUT_ROUTE);
	}

	for (i = 0; i <= INDEX_STEREO_IN_CONTROL; i++) {
		n_out_state |= (n_map[i] & STEREO_OUT_ROUTE);
	}

	if ((o_out_state == 0) && (n_out_state != 0)) {
		/* Stereo out is on */
		lt_micco_hifi_pb_in_enable(ctx);
	}

	if ((o_out_state != 0) && (n_out_state == 0)) {
		/* Stereo out is off */
		lt_micco_hifi_pb_in_disable(ctx);
	}

	/* Output end here */

	/* Micco MUX */
	/* MONO MUX */
	o_mux = n_mux = 0;
	if (o_map[0] & SPEAKER_OUT_ROUTE)
		o_mux |= (3 << 3);
	if (o_map[1] & SPEAKER_OUT_ROUTE)
		o_mux |= (1 << 5);
	if (o_map[10] & SPEAKER_OUT_ROUTE)
		o_mux |= 3;

	if (n_map[0] & SPEAKER_OUT_ROUTE)
		n_mux = (3 << 3);
	if (n_map[1] & SPEAKER_OUT_ROUTE)
		n_mux |= (1 << 5);
	if (n_map[10] & SPEAKER_OUT_ROUTE)
		n_mux |= 3;
	printk("%s: o_mux: 0x%08x, n_mux: 0x%08x\n", __func__, o_mux, n_mux);
	if (o_mux != n_mux)
		micco_codec_write(MICCO_MUX_MONO, n_mux);

	/* BEAR MUX */
	o_mux = n_mux = 0;
	printk(KERN_ALERT "%s: o_map[0]: 0x%08x, n_map[0]: 0x%08x\n", __func__,
			o_map[0], n_map[0]);
	if (o_map[0] & STEREO_HEAD_SET_OUT_ROUTE)
		o_mux |= (3 << 3);
	if (o_map[1] & STEREO_HEAD_SET_OUT_ROUTE)
		o_mux |= (1 << 5);
	if (o_map[10] & STEREO_HEAD_SET_OUT_ROUTE)
		o_mux |= 3;

	if (n_map[0] & STEREO_HEAD_SET_OUT_ROUTE)
		n_mux = (3 << 3);
	if (n_map[1] & STEREO_HEAD_SET_OUT_ROUTE)
		n_mux |= (1 << 5);
	if (n_map[10] & STEREO_HEAD_SET_OUT_ROUTE)
		n_mux |= 3;

	if (n_mux != o_mux)
		micco_codec_write(MICCO_MUX_BEAR, n_mux);

	/* LINE OUT MUX */
	o_mux = n_mux = 0;
	if (o_map[0] & LINE_OUT_ROUTE)
		o_mux = (3 << 3);
	if (o_map[1] & LINE_OUT_ROUTE)
		o_mux |= (1 << 5);
	if (o_map[10] & LINE_OUT_ROUTE)
		o_mux |= 3;
	if (o_map[5] & LINE_OUT_ROUTE)
		o_mux |= (1 << 7);
	if (o_map[6] & LINE_OUT_ROUTE)
		o_mux |= (1 << 7);

	if (n_map[0] & LINE_OUT_ROUTE)
		n_mux = (3 << 3);
	if (n_map[1] & LINE_OUT_ROUTE)
		n_mux |= (1 << 5);
	if (n_map[10] & LINE_OUT_ROUTE)
		n_mux |= 3;
	if (n_map[5] & LINE_OUT_ROUTE)
		n_mux |= (1 << 7);
	if (n_map[6] & LINE_OUT_ROUTE)
		n_mux |= (1 << 7);

	if (o_mux != n_mux)
		micco_codec_write(MICCO_MUX_LINE_OUT, n_mux);

	/* Voice Near out MUX */
	o_mux = n_mux = 0;
	if (o_map[0] & VOICE_NEAR_OUT_ROUTE)
		o_mux |= (3 << 6);
	if (o_map[10] & VOICE_NEAR_OUT_ROUTE)
		o_mux |= (3 << 1);

	if (n_map[0] & VOICE_NEAR_OUT_ROUTE)
		n_mux |= (3 << 6);
	if (n_map[10] & VOICE_NEAR_OUT_ROUTE)
		n_mux |= (3 << 1);

	mic1_mux = mic2_mux = 0;
	mic1_s_chg = mic2_s_chg = 0;
	if (o_mux != n_mux) {
		/* If the TX_PGA changed, we enable the MIC
		 * always. Use M1 and M2 to select MIC MUX.
		 */
		n_mux |= (3 << 2);
		micco_codec_write(MICCO_TX_PGA_MUX, n_mux);
	} else {
		/* if MIC1 route to voice near out state changed */
		if ((o_map[5] & VOICE_NEAR_OUT_ROUTE) != 
			(n_map[5] & VOICE_NEAR_OUT_ROUTE)) {
			if (n_map[5] & VOICE_NEAR_OUT_ROUTE) {
				n_mux |= (3 << 2);
				mic1_mux = 1;
				mic2_mux = 0;
			} else {
				mic1_mux = 0;
			}

			mic1_s_chg = 1;
		} else { /* Mic1 state is not changed */
			mic1_s_chg = 0;
		}

		/* if MIC2 route to voice near out state changed */
		if ((o_map[6] & VOICE_NEAR_OUT_ROUTE) != 
			(n_map[6] & VOICE_NEAR_OUT_ROUTE)) {
			if (n_map[6] & VOICE_NEAR_OUT_ROUTE) {
				mic2_mux = 1;
				mic1_mux = 0;
			} else {
				mic2_mux = 0;
			}

			mic2_s_chg = 1;
		} else { /* Mic2 state is not changed */
			mic2_s_chg = 0;
		}

		if (mic1_s_chg || mic2_s_chg) {
			/* Mic1 OR Mic2 state changed */
			if (!mic1_mux && ! mic2_mux) {
				/* Mic1 AND Mic2 disabled */
				micco_codec_read(MICCO_MIC_PGA, &mic_pga);
				mic_pga &= ~0x08;
				micco_codec_write(MICCO_MIC_PGA, mic_pga);				
			} else {
				/* Mic1 OR Mic2 enabled */
				if (mic1_mux == 1) {	/*Mic1 enabled */
					micco_codec_read(
						MICCO_MIC_PGA, &mic_pga);
					mic_pga &= ~0x10;
					micco_codec_write(
						MICCO_MIC_PGA, mic_pga);
				} else if (mic2_mux == 1) { /*Mic2 enabled */
					micco_codec_read(
						MICCO_MIC_PGA, &mic_pga);
					mic_pga |= 0x10;
					micco_codec_write(
						MICCO_MIC_PGA, mic_pga);
				}
			}
		}
	}

	/* Stereo_out MUX */	
	o_mux = n_mux = 0;
	if (o_map[0] & STEREO_OUT_ROUTE)
		o_mux |= (3 << 3);
	if (o_map[1] & STEREO_OUT_ROUTE)
		o_mux |= (1 << 5);
	if (o_map[10] & STEREO_OUT_ROUTE)
		o_mux |= 3;

	if (n_map[0] & STEREO_OUT_ROUTE)
		n_mux |= (3 << 3);
	if (n_map[1] & STEREO_OUT_ROUTE)
		n_mux |= (1 << 5);
	if (n_map[10] & STEREO_OUT_ROUTE)
		n_mux |= 3;
	if (o_mux != n_mux) {
		micco_codec_write(MICCO_MUX_STEREO_CH1, n_mux);
		micco_codec_write(MICCO_MUX_STEREO_CH2, n_mux);
	}

	/* MIC1, MIC2 just can route to ADC or Line out */

	return ACODEC_SUCCESS;
}
