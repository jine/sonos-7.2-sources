/*
 * include/asm-arm/arch-pxa/codec/lt_micco_acodec.h
 *  
 *  Author:	fengwei.yin@marvell.com
 *  Created:	Nov 21, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __LT_MICCO_ACODEC_H__
#define __LT_MICCO_ACODEC_H__


typedef unsigned int acodec_device_id_t;

#define AK_2440_ID 0x9e	/* this should be the I2C address */
#define WM_8753_ID 0x34 /* this should be the I2C address */
#define WM_8734_ID 2	/* this should be the I2C address */

#define UCB_1400_ID 0x4304 /* this should be the 7E register value */
#define WM_9703_ID  0x4C03 /* this should be the 7E register value */
#define WM_9704_ID  0x4C04 /* this should be the 7E register value */
#define WM_9705_ID  0x4C05 /* this should be the 7E register value */
#define WM_9712_ID  0x4C12 /* this should be the 7E register value */
#define WM_9713_ID  0x4C13 /* this should be the 7E register value */

#define	MICCO_ID	0x10

typedef enum acodec_error_t {     
	ACODEC_SUCCESS = 0,
	/* null pointer to registers or other software error */
	ACODEC_GENERAL_SW_ERR,
	/* time-out for waiting for respponse */
	ACODEC_CONTROLLER_INTERFACE_TIMEOUT,
	/* the sample rate is not supported either in controller or codec */
	ACODEC_SAMPLERATE_NOT_SUPPORTED,
	/* this codec feature is not supported */
	ACODEC_FEATURE_NO_SUPPORTED,
	/* other hardware error besides time out */
	ACODEC_GENERAL_HW_ERR,
	/* the codec can not set the route required */
	ACODEC_ROUTE_NO_SUPPORTED,
	ACODEC_SLEEP, /* the codec is sleep */
} acodec_error_t;

typedef enum _codec_state {
	CODEC_SLEEP  = 0,
	CODEC_WAKEUP = 1,
} acodec_state_t;

/* input route control */
enum {
	INDEX_HIFI_NEAR_IN_CONTROL =  0,
	INDEX_VOICE_NEAR_IN_CONTROL,
	INDEX_BT_HIFI_NEAR_IN_CONTROL,
	INDEX_BT_VOICE_NEAR_IN_CONTROL,
	INDEX_BT_MIC_IN_CONTROL,
	INDEX_MIC1_IN_CONTROL,
	INDEX_MIC2_IN_CONTROL,
	INDEX_FAR_IN_CONTROL,
	INDEX_MIDI_IN_CONTROL,
	INDEX_FM_IN_CONTROL,
	INDEX_STEREO_IN_CONTROL,
};

/* ouput route control */
enum {
	INDEX_STEREO_HEAD_SET_OUT_CONTROL = 0,
	INDEX_SPEAKER_OUT_CONTROL,
	INDEX_HAND_SET_OUT_CONTROL,
	INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL,
	INDEX_VOICE_NEAR_OUT_CONTROL,
	INDEX_HIFI_NEAR_OUT_CONTROL,
	INDEX_FAR_OUT_CONTROL,
	INDEX_LINEOUT_CONTROL,
	INDEX_STEREO_OUT_CONTROL,
};

/* ouput route control */
#define STEREO_HEAD_SET_OUT_ROUTE	\
	((unsigned short) 1<<INDEX_STEREO_HEAD_SET_OUT_CONTROL) 

#define SPEAKER_OUT_ROUTE		\
	((unsigned short)1<<INDEX_SPEAKER_OUT_CONTROL)

#define HAND_SET_OUT_ROUTE		\
	((unsigned short)1<<INDEX_HAND_SET_OUT_CONTROL)

#define BT_STEREO_HEAD_SET_OUT_ROUTE	\
	((unsigned short)1<<INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL)

#define VOICE_NEAR_OUT_ROUTE		\
	((unsigned short)1<<INDEX_VOICE_NEAR_OUT_CONTROL)

#define HIFI_NEAR_OUT_ROUTE		\
	((unsigned short)1<<INDEX_HIFI_NEAR_OUT_CONTROL)

#define FAR_OUT_ROUTE		\
	((unsigned short)1<<INDEX_FAR_OUT_CONTROL)

#define	LINE_OUT_ROUTE		\
	((unsigned short)1<<INDEX_LINEOUT_CONTROL)

#define	STEREO_OUT_ROUTE		\
	((unsigned short)1<<INDEX_STEREO_OUT_CONTROL)

#define SCENARIO_BYTES_SIZE    4

#define	EVENT_TYPE_NONE		0
#define	EVENT_TYPE_PDN		1

typedef enum {
	FM = 0,
	MIC1 = 1,
	MIC2 = 2,
	SPEAKER =3,
	HEADSET =4,
	HANDSET =5,
} vol_port_type_t;

typedef struct _context_t {
	unsigned acodec_id;
	int use_count;
	unsigned long init_number;

	void *p_voice_reg;
	void *p_hifi_reg; 
	void *p_ctrl_reg; 
	int  *p_ost_regs;	/* needed for time out */

	/* pointer to a memory region to save context while suspend */
	void *p_save_memory;
	void *p_scenario; /* pointer to the scenario data structure */

	/* The max timeout for read or write operation */
	long u_max_read_write_time_out_ms;
	/* The max timeout for setup of the ACODEC controller and codec */
	long u_max_setup_time_out_ms;

	/* member functions */
	acodec_error_t (*set_master_vol)(struct _context_t *codec_context,
				unsigned short gain_in_db);
	acodec_error_t (*set_master_input_gain)(struct _context_t *codec_context,
				unsigned short gain_in_db);

	/* For VOICE PCM */
	acodec_error_t (*get_voice_out_sample_rate)(
				struct _context_t *codec_context,
				unsigned short * rate_in_hz);

	acodec_error_t (*set_voice_out_sample_rate)(
				struct _context_t *codec_context,
				unsigned short rate_in_hz);

	acodec_error_t (*get_voice_in_sample_rate)(
				struct _context_t *codec_context,
				unsigned short * rate_in_hz);
	acodec_error_t (*set_voice_in_sample_rate)(
				struct _context_t *codec_context,
				unsigned short rate_in_hz);

	acodec_error_t (*voice_parepare)(struct _context_t *codec_context);

	/* For HIFI PCM */
	acodec_error_t (*get_in_sample_rate)(struct _context_t *codec_context,
				unsigned short * rate_in_hz);
	acodec_error_t (*get_out_sample_rate)(struct _context_t *codec_context,
				unsigned short * rate_in_hz);
	acodec_error_t (*set_in_sample_rate)(struct _context_t *codec_context,
				unsigned short rate_in_hz);
	acodec_error_t (*set_out_sample_rate)(struct _context_t *codec_context,
				unsigned short rate_in_hz);

	acodec_error_t (*codec_specific_init)(struct _context_t *codec_context);
	acodec_error_t (*codec_specific_dinit)(struct _context_t *codec_context);

	acodec_error_t (*acodec_read)(struct _context_t *codec_context,
				unsigned short reg_addr,
				unsigned short *p_reg_value);
	acodec_error_t (*acodec_write)(struct _context_t *codec_context,
				unsigned short reg_addr,
				unsigned short reg_value);

	acodec_error_t (*hifi_stream_path_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*hifi_stream_path_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*recording_path_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*recording_path_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*voice_path_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*voice_path_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*side_tone_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*side_tone_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*head_set_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*head_set_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*ear_speaker_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*ear_speaker_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*lounder_speaker_enable)(
				struct _context_t *codec_context);
	acodec_error_t (*lounder_speaker_disable)(
				struct _context_t *codec_context);
	acodec_error_t (*mic_enable)(
				struct _context_t *codec_context,
				unsigned char mic_type);
	acodec_error_t (*mic_disable)(
				struct _context_t *codec_context,
				unsigned char mic_type);
	acodec_error_t (*voice_prepare)(struct _context_t *codec_context);
	acodec_error_t (*set_volume_table)(struct _context_t *codec_context);
	acodec_error_t (*get_volume_table)(struct _context_t *codec_context);
	acodec_error_t (*set_tone_table)(struct _context_t *codec_context);
	acodec_error_t (*get_tone_table)(struct _context_t *codec_context);

	/* For route */
	acodec_error_t (*set_route)(struct _context_t *codec_context,
				unsigned short * rout_map,
				unsigned short* current_map);
	/* For suspend/wakeup codec */
	acodec_error_t (*sleep_codec)(struct _context_t *codec_context);
	acodec_error_t (*wake_codec)(struct _context_t *codec_context);

	/* Get codec state */
	acodec_error_t (*get_state)(struct _context_t *codec_context,
				acodec_state_t *p_state);

	/* For volume */
	acodec_error_t (*get_vol)(struct _context_t *codec_context,
				vol_port_type_t port,
				unsigned short *gain_in_db);
	acodec_error_t (*set_vol)(struct _context_t *codec_context,
				vol_port_type_t port,
				unsigned short gain_in_db);
} acodec_context_t, *p_acodec_context_t;

#endif /* __LT_MICCO_ACODEC_H__ */
