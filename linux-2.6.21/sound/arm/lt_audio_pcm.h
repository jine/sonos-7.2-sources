/*
 * linux/sound/arm/lt_audio_pcm.h
 *
 * Author:	Yin, Fengwei
 * Created:	Jan, 2007
 * Copyright (C) 2007, Marvell International Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _AUDIO_PCM_LT_H_
#define _AUDIO_PCM_LT_H_

typedef struct {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	volatile u32 *drcmr;		/* the DMA request channel to use */
	u32 dev_addr;			/* device physical address for DMA */
} audio_pcm_dma_params_t;
	
typedef struct {
	audio_pcm_dma_params_t *playback_params;
	audio_pcm_dma_params_t *capture_params;
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
} auido_pcm_client_t;

extern int audio_codec_pcm_new(struct snd_card *card);
#endif
