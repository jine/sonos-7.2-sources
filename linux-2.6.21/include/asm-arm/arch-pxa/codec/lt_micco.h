#ifndef	__LT_MICCO_H__
#define	__LT_MICCO_H__

#include "lt_micco_acodec.h"

acodec_error_t lt_micco_set_master_vol(acodec_context_t *codec_context,
		unsigned short gain_in_db);
acodec_error_t lt_micco_set_master_input_gain(acodec_context_t *codec_context,
		unsigned short gain_in_db);
acodec_error_t lt_micco_get_hifi_sample_rate(acodec_context_t *codec_context,
		unsigned short * rate_in_hz);
acodec_error_t lt_micco_set_hifi_sample_rate(acodec_context_t *codec_context,
		unsigned short rate_in_hz);
acodec_error_t lt_micco_voice_prepare(acodec_context_t *codec_context);
acodec_error_t lt_micco_set_voice_sample_rate(acodec_context_t *codec_context,
		unsigned short rate_in_hz);
acodec_error_t lt_micco_get_voice_sample_rate(acodec_context_t *codec_context,
		unsigned short *rate_in_hz);
acodec_error_t lt_micco_specific_init (acodec_context_t *codec_context);
int lt_micco_hifi_in_enable(acodec_context_t *codec_context);
int lt_micco_hifi_in_disable(acodec_context_t *codec_context);
int lt_micco_hifi_out_enable (acodec_context_t *codec_context);
int lt_micco_hifi_out_disable (acodec_context_t *codec_context);
int lt_micco_voice_in_enable(acodec_context_t *codec_context);
int lt_micco_voice_in_disable(acodec_context_t *codec_context);
acodec_error_t lt_micco_recording_path_enable (acodec_context_t *codec_context);
acodec_error_t lt_micco_recording_path_disable (acodec_context_t *codec_context);
acodec_error_t lt_micco_hifi_path_enable(acodec_context_t *codec_context);
acodec_error_t lt_micco_hifi_path_disable(acodec_context_t *codec_context);

acodec_error_t lt_micco_headset_enable (acodec_context_t *codec_context);
acodec_error_t lt_micco_headset_disable (acodec_context_t *codec_context);
acodec_error_t lt_micco_ear_speaker_enable (acodec_context_t *codec_context);
acodec_error_t lt_micco_ear_speaker_disable (acodec_context_t *codec_context);
acodec_error_t lt_micco_louder_speaker_enable (acodec_context_t *codec_context);
acodec_error_t lt_micco_louder_speaker_disable (acodec_context_t *codec_context);
acodec_error_t lt_micco_handset_mic_enable (acodec_context_t *codec_context,
		unsigned char mic_type);
acodec_error_t lt_micco_handset_mic_disable (acodec_context_t *codec_context,
		unsigned char mic_type);
int lt_micco_headset_mic_enable (acodec_context_t *codec_context,
		unsigned char mic_type);
int lt_micco_headset_mic_disable (acodec_context_t *codec_context,
		unsigned char mic_type);
int lt_micco_set_headset_vol(acodec_context_t *codec_context,
		unsigned short gain_in_db);
acodec_error_t lt_micco_get_vol(acodec_context_t *codec_context, vol_port_type_t port,
		unsigned short *gain_in_db);
acodec_error_t lt_micco_set_vol(acodec_context_t *codec_context, vol_port_type_t port,
		unsigned short gain_in_db);
int lt_micco_save_context (acodec_context_t *codec_context);
int lt_micco_restore_context (acodec_context_t *codec_context);
int lt_micco_sleep(acodec_context_t *codec_context);
int lt_wakeup(acodec_context_t *codec_context);
acodec_error_t lt_micco_specific_deinit (acodec_context_t *codec_context);
int support_scenario(unsigned short *rout_map);

acodec_error_t lt_micco_set_route(acodec_context_t *codec_context,
		unsigned short *rout_map, unsigned short* current_map);
#endif
