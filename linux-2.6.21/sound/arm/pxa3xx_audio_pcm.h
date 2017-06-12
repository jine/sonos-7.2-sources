/*
 * linux/sound/arm/audio_codec_pcm.h
 *
 * Author:	Xu Jingqing
 * Created:	July, 2005
 * Copyright (C) 2005, Intel Corporation (jingqing.xu@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#ifndef _AUDIO_CODEC_PCM_ZY_H
#define _AUDIO_CODEC_PCM_ZY_H

struct pxa3xx_pcm_dma_params {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	volatile u32 *drcmr;		/* the DMA request channel to use */
	u32 dev_addr;			/* device physical address for DMA */
} ;

struct pxa3xx_pcm_client {
	struct pxa3xx_pcm_dma_params *playback_params;
	struct pxa3xx_pcm_dma_params *capture_params;
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
};

extern int audio_codec_pcm_new(struct snd_card *card);
#endif
