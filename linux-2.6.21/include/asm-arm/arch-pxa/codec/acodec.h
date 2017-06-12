/*
 * linux/sound/arm/codec/acodec.h.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ACODEC_H__
#define __ACODEC_H__


typedef unsigned int zy_acodec_device_id_t;

/* this should be the I2C address */
#define AK_2440_ID 0x9e
#define WM_8753_ID 0x34
#define WM_8734_ID 2

#define UCB_1400_ID 0x4304 /* this should be the 7E register value */
#define WM_9703_ID  0x4C03 /* this should be the 7E register value */
#define WM_9704_ID  0x4C04 /* this should be the 7E register value */
#define WM_9705_ID  0x4C05 /* this should be the 7E register value */
#define WM_9712_ID  0x4C12 /* this should be the 7E register value */
#define WM_9713_ID  0x4C13 /* this should be the 7E register value */


typedef enum _acodec_error_t {
	ACODEC_SUCCESS = 0,    /* successful completion of a function */
	ACODEC_GENERAL_SW_ERR, /* null pointer to registers or other software error */
	ACODEC_CONTROLLER_INTERFACE_TIMEOUT, /* time-out for waiting for respponse */
	ACODEC_SAMPLERATE_NOT_SUPPORTED, /* the sample rate is not supported either in controller or codec */
	ACODEC_FEATURE_NO_SUPPORTED, /* this codec feature is not supported  */
	ACODEC_GENERAL_HW_ERR ,/* other hardware error besides time out */
	ACODEC_ROUTE_NO_SUPPORTED, /* the codec can not set the route required */
	ACODEC_SLEEP, /* the codec is sleep */
} acodec_error_t;

typedef enum _codec_state {
	CODEC_SLEEP  = 0,
	CODEC_WAKEUP = 1,
} acodec_state_t;

/* input route control */
#define INDEX_HIFI_NEAR_IN_CONTROL		0
#define INDEX_VOICE_NEAR_IN_CONTROL          1
#define INDEX_BT_HIFI_NEAR_IN_CONTROL        2
#define INDEX_BT_VOICE_NEAR_IN_CONTROL       3
#define INDEX_BT_MIC_IN_CONTROL              4
#define INDEX_MIC1_IN_CONTROL                5
#define INDEX_MIC2_IN_CONTROL                6
#define INDEX_FAR_IN_CONTROL                 7
#define INDEX_MIDI_IN_CONTROL                8
#define INDEX_FM_IN_CONTROL                  9

/* ouput route control */
#define INDEX_STEREO_HEAD_SET_OUT_CONTROL    0
#define INDEX_SPEAKER_OUT_CONTROL            1
#define INDEX_HAND_SET_OUT_CONTROL           2
#define INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL 3
#define INDEX_VOICE_NEAR_OUT_CONTROL         4
#define INDEX_HIFI_NEAR_OUT_CONTROL          5
#define INDEX_FAR_OUT_CONTROL                6

/* ouput route control */
#define STEREO_HEAD_SET_OUT_ROUTE            ((unsigned short) 1<<INDEX_STEREO_HEAD_SET_OUT_CONTROL)
#define SPEAKER_OUT_ROUTE                    ((unsigned short)1<<INDEX_SPEAKER_OUT_CONTROL)
#define HAND_SET_OUT_ROUTE                   ((unsigned short)1<<INDEX_HAND_SET_OUT_CONTROL)
#define BT_STEREO_HEAD_SET_OUT_ROUTE         ((unsigned short)1<<INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL)
#define VOICE_NEAR_OUT_ROUTE                 ((unsigned short)1<<INDEX_VOICE_NEAR_OUT_CONTROL)
#define HIFI_NEAR_OUT_ROUTE                  ((unsigned short)1<<INDEX_HIFI_NEAR_OUT_CONTROL)
#define FAR_OUT_ROUTE                        ((unsigned short)1<<INDEX_FAR_OUT_CONTROL)

#define SCENARIO_BYTES_SIZE    		4

#define EVENT_TYPE_NONE			0
#define EVENT_TYPE_PDN			1

typedef enum {
	FM = 0,
	MIC1 = 1,
	MIC2 = 2,
	SPEAKER =3,
	HEADSET =4,
	HANDSET =5,
} vol_port_type_t;

typedef struct _context_t {
	int  use_count; 		/* probe/remove and suspend/resume usage count, sync among multiple devices */
	zy_acodec_device_id_t acodec_id;/* - an ID that uniquely identifies the codec to be used */
	unsigned long init_number; 	/* used by driver to track whether it is inited or not */

	void *p_voice_reg;       	/* pointer to Monahans registers that has PCM interface to codec */
	void *p_hifi_reg;        	/* pointer to Monahans registers that has hifi interface to codec */
	void *p_ctrl_reg;        	/* pointer to Monahans registers that has control interface to codec */
	int  *p_ost_regs;		/* needed for time out */
	void *p_save_memory;        	/* pointer to a memory region to save context while suspend */
	void *p_zy_scenario;        	/* pointer to the scenario data structure  */
	long u_max_read_write_time_out_ms;/* input the max time to wait in milliseconds before giving up on a read or write operation */
	long u_max_setup_time_out_ms;	/* input the maximum time in milliseconds to wait during initial setup of the ACODEC controller and codec */

	/* member functions these pointers must be set by */
	acodec_error_t (* set_master_vol)(struct _context_t *p_device_context, unsigned short gain_in_db);
	acodec_error_t (* set_master_input_gain)(struct _context_t *p_device_context, unsigned short gain_in_db);

	/* add for voice  */
	acodec_error_t (* get_voice_out_sample_rate)(struct _context_t *p_device_context, unsigned short * rate_in_hz);
	acodec_error_t (* set_voice_out_sample_rate)(struct _context_t *p_device_context, unsigned short rate_in_hz);
	acodec_error_t (* get_voice_in_sample_rate)(struct _context_t *p_device_context, unsigned short * rate_in_hz);
	acodec_error_t (* set_voice_in_sample_rate)(struct _context_t *p_device_context, unsigned short rate_in_hz);
	acodec_error_t (*voice_parepare) (struct _context_t *p_device_context);

	/* it is used to set hifi sample rate */
	acodec_error_t (* get_in_sample_rate)        (struct _context_t *p_device_context, unsigned short * rate_in_hz);
	acodec_error_t (* get_out_sample_rate)       (struct _context_t *p_device_context, unsigned short * rate_in_hz);
	acodec_error_t (* set_in_sample_rate)        (struct _context_t *p_device_context, unsigned short rate_in_hz);
	acodec_error_t (* set_out_sample_rate)       (struct _context_t *p_device_context, unsigned short rate_in_hz);

	acodec_error_t (* codec_specific_init)      (struct _context_t *p_device_context);
	acodec_error_t (* codec_specific_dinit)    (struct _context_t *p_device_context);
	acodec_error_t (* acodec_read)             (struct _context_t *p_dev_context, unsigned short reg_addr, unsigned short *p_reg_value);
	acodec_error_t (* acodec_write)            (struct _context_t *p_dev_context, unsigned short reg_addr, unsigned short reg_value);
	acodec_error_t (* hifi_stream_path_enable)   (struct _context_t *p_device_context);
	acodec_error_t (* hifi_stream_path_disable)  (struct _context_t *p_device_context);
	acodec_error_t (* recording_path_enable)    (struct _context_t *p_device_context);
	acodec_error_t (* recording_path_disable)   (struct _context_t *p_device_context);
	acodec_error_t (* voice_path_enable)        (struct _context_t *p_device_context);
	acodec_error_t (* voice_path_disable)       (struct _context_t *p_device_context);
	acodec_error_t (* side_tone_enable)         (struct _context_t *p_device_context);
	acodec_error_t (* side_tone_disable)        (struct _context_t *p_device_context);
	acodec_error_t (* head_set_enable)          (struct _context_t *p_device_context);
	acodec_error_t (* head_set_disable)         (struct _context_t *p_device_context);
	acodec_error_t (* ear_speaker_enable)       (struct _context_t *p_device_context);
	acodec_error_t (* ear_speaker_disable)      (struct _context_t *p_device_context);
	acodec_error_t (* lounder_speaker_enable)    (struct _context_t *p_device_context);
	acodec_error_t (* lounder_speaker_disable)   (struct _context_t *p_device_context);
	acodec_error_t (* mic_enable)              (struct _context_t *p_device_context, unsigned char mic_type);
	acodec_error_t (* mic_disable)             (struct _context_t *p_device_context, unsigned char mic_type);
	acodec_error_t (* voice_prepare)	(struct _context_t *p_device_context);
	acodec_error_t (* set_volume_table)         (struct _context_t *p_device_context);
	acodec_error_t (* get_volume_table)         (struct _context_t *p_device_context);
	acodec_error_t (* set_tone_table)           (struct _context_t *p_device_context);
	acodec_error_t (* get_tone_table)           (struct _context_t *p_device_context);

	/* add for route */
	acodec_error_t (* set_route)           (struct _context_t *p_device_context, unsigned short * rout_map ,unsigned short* current_map);
	/* add for sleep the codec */
	acodec_error_t (* sleep_codec)           (struct _context_t *p_device_context);
	/* add for Wake up the codec */
	acodec_error_t (* wake_codec)           (struct _context_t *p_device_context);
	/* add for get codec state */
	acodec_error_t (* get_state)           (struct _context_t *p_device_context, acodec_state_t *p_state);
	/* add for volume */
	acodec_error_t (* get_vol)(struct _context_t *p_device_context, vol_port_type_t port,  unsigned short *gain_in_db);
	acodec_error_t (* set_vol)(struct _context_t *p_device_context, vol_port_type_t port,  unsigned short gain_in_db);

	void (* get_event)(struct _context_t *p_device_context, unsigned char * event_type);
	void (* event_ack)(struct _context_t *p_device_context, unsigned char event_type);
	acodec_error_t (* enable_touch)(struct _context_t *p_device_context);
	acodec_error_t (* disable_touch)(struct _context_t *p_device_context);
} acodec_context_t, *p_acodec_context_t;

typedef enum {
	CODEC_POWER_ON = 0,
	CODEC_READY_LOWPOWER,
	CODEC_LOWPOWER,
	CODEC_POWER_OFF,
} codec_state_t;

typedef enum {
	CODEC_SUB_POWER_ON = 0,
	CODEC_SUB_LOWPOWER,
	CODEC_SUB_POWER_OFF,
} codec_sub_state_t;

extern int codec_client;
extern int set_codec_sub_state(int client, int state);
extern int register_codec(int *client);
extern int unregister_codec(int client);

#endif /* __ACODEC_H__ */
