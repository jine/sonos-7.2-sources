#ifndef __AC97CODEC_PM__
#define __AC97CODEC_PM__

typedef enum {
	CODEC_POWER_ON = 0,
	CODEC_LOWPOWER,
	CODEC_POWER_OFF,
	CODEC_READY_LOWPOWER,
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
void codec_save(void);
void codec_restore(void);
void codec_set_pll(unsigned int div);
void codec_cold_reset(void);

#endif

