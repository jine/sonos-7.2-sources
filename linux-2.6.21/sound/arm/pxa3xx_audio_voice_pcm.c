/*
 *  --linux/sound/arm/pxa3xx_audio_voice__pcm.c: ALSA voice PCM interface for the PXA3xx 
 * ports from linux/sound/arm/pxa2xx-pcm.c
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/codec/acodec.h>

#include "pxa3xx_audio_pcm.h"

//#define ALSA_ZY_HIFI_PCM_DEBUG
#undef ALSA_ZY_HIFI_PCM_DEBUG
#ifdef ALSA_ZY_HIFI_PCM_DEBUG
#define dbg(format, arg...) printk("<1>" "file: " __FILE__ "Line:(%d) FUNC:%s******\n" format "\n",__LINE__,__func__,##arg)
#else
#define dbg(format, arg...)
#endif

static const struct snd_pcm_hardware voice_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	//The period must be the multiple of 32 bytes
	//because the AC97 link requires the value of DCMD.LEN
	//must be the multiple of 32 bytes.
	//so the minimal period bytes should be 32 bytes
	.period_bytes_min	= 32,
	//The max value of DCMD.LEN is 8K-1, and AC97
	//link requires the value of DCMD.LEN must be
	//mutiple of 32 bytes, so the maximl period bytes should be 8k-32.
	.period_bytes_max	= 8192 - 32,
	.periods_min		= 1,
	//The size of descriptor area is PAGE_SIZE
	.periods_max		= PAGE_SIZE/sizeof(pxa_dma_desc),
	//playback/capture ring buffer max size
	//ALSA use this vaule to pre allocate the ring buffer
	.buffer_bytes_max	= 128 * 1024,
	//bytes
	.fifo_size		= 32,//64,
	.channels_min = 1,
	.channels_max= 2,
};

struct voice_runtime_data {
	int dma_ch;
	struct pxa3xx_pcm_dma_params *params;
	pxa_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;
	dma_addr_t pause_dma_desc_array_phys;
	int pause;
};

static int voice_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voice_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	pxa_dma_desc *dma_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	dma_desc = rtd->dma_desc_array;
	next_desc_phys = rtd->dma_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;
	do {
		next_desc_phys += sizeof(pxa_dma_desc);
		dma_desc->ddadr = next_desc_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			dma_desc->dsadr = dma_buff_phys;
			dma_desc->dtadr = rtd->params->dev_addr;
		} else {
			dma_desc->dsadr = rtd->params->dev_addr;
			dma_desc->dtadr = dma_buff_phys;
		}
		if (period > totsize)
			period = totsize;
		dma_desc->dcmd = rtd->params->dcmd | period | DCMD_ENDIRQEN;
		dma_desc++;
		dma_buff_phys += period;
	} while (totsize -= period);
	dma_desc[-1].ddadr = rtd->dma_desc_array_phys;
	dbg("voice_pcm_hw_params success!\n");
	return 0;
}

static int voice_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct voice_runtime_data *rtd = substream->runtime->private_data;

	*rtd->params->drcmr = 0;
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int voice_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct pxa3xx_pcm_client *client = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voice_runtime_data *rtd = runtime->private_data;
	rtd->pause = 0;
	DCSR(rtd->dma_ch) &= ~DCSR_RUN;
	DCSR(rtd->dma_ch) = 0;
	DCMD(rtd->dma_ch) = 0;
	*rtd->params->drcmr = rtd->dma_ch | DRCMR_MAPVLD;
	if(client->prepare){
		return client->prepare(substream);
	}else
	{
		return 0;
	}
}
static int voice_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct voice_runtime_data *rtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		DDADR(rtd->dma_ch) = rtd->dma_desc_array_phys;
		DCSR(rtd->dma_ch) = DCSR_RUN;
		dbg("dma start!\n");
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		DCSR(rtd->dma_ch) &= ~DCSR_RUN;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		rtd->pause = 1;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rtd->pause = 0;
		DDADR(rtd->dma_ch) = rtd->pause_dma_desc_array_phys;
		DCSR(rtd->dma_ch) |= DCSR_RUN;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static void voice_pcm_dma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct voice_runtime_data *rtd = substream->runtime->private_data;
	int dcsr;

	dcsr = DCSR(dma_ch);
	DCSR(dma_ch) = dcsr & ~DCSR_STOPIRQEN;

	if (dcsr & DCSR_ENDINTR) {
		snd_pcm_period_elapsed(substream);
		dbg("next dma discriptor is 0x%x!\n", DDADR(rtd->dma_ch));
		if(rtd->pause) {
			rtd->pause_dma_desc_array_phys = DDADR(rtd->dma_ch);
			DCSR(rtd->dma_ch) &= ~DCSR_RUN;
		}
	} else {
		printk( KERN_ERR "%s: DMA error on channel %d (DCSR=%#x)\n",
			rtd->params->name, dma_ch, dcsr );
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
	}
}

static snd_pcm_uframes_t voice_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voice_runtime_data *rtd = runtime->private_data;
	dma_addr_t ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			 DSADR(rtd->dma_ch) : DTADR(rtd->dma_ch);
	snd_pcm_uframes_t x = bytes_to_frames(runtime, ptr - runtime->dma_addr);
	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int
voice_pcm_hw_rule_mult32(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *i = hw_param_interval(params, rule->var);
	int changed = 0;

	if (i->min & 31) {
		i->min = (i->min & ~31) + 32;
		i->openmin = 0;
		changed = 1;
	}

	if (i->max & 31) {
		i->max &= ~31;
		i->openmax = 0;
		changed = 1;
	}

	return changed;
}

static int voice_pcm_open(struct snd_pcm_substream *substream)
{
	struct pxa3xx_pcm_client *client = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voice_runtime_data *rtd;
	int ret;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	runtime->hw = voice_pcm_hardware;

	//The period must be the multiple of 32 bytes
	//because the AC97 link requires the value of DCMD.LEN
	//must be the multiple of 32 bytes.
	//So here we set the rule to make period size and buffer size
	//to be the multiple of 32 bytes.
	ret = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				  voice_pcm_hw_rule_mult32, NULL,
				  SNDRV_PCM_HW_PARAM_PERIOD_BYTES, -1);
	if (ret){
		dbg("Add rule error");
		goto out;
	}
	ret = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				  voice_pcm_hw_rule_mult32, NULL,
				  SNDRV_PCM_HW_PARAM_BUFFER_BYTES, -1);
	if (ret){
		dbg("Add rule error");
		goto out;
	}
	ret = -ENOMEM;
	rtd = kmalloc(sizeof(*rtd), GFP_KERNEL);
	if (!rtd){
		dbg("alloc memory error\n");
		goto out;
	}
	rtd->dma_desc_array =
		dma_alloc_writecombine(substream->pcm->card->dev, PAGE_SIZE,
				       &rtd->dma_desc_array_phys, GFP_KERNEL);
	if (!rtd->dma_desc_array){
		dbg("alloc dma desc error\n");
		goto err1;
	}
	rtd->params = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		      client->playback_params : client->capture_params;
	ret = pxa_request_dma(rtd->params->name, DMA_PRIO_LOW,
			      voice_pcm_dma_irq, substream);
	if (ret < 0){
		dbg("request dma error\n");
		goto err2;
	}
	rtd->dma_ch = ret;

	runtime->private_data = rtd;
	if(client->startup){
		ret = client->startup(substream);
		if (ret){
			dbg("start substeam error NO: %x\n",ret);
			goto err3;
		}
	}
	return 0;
 err3:
	pxa_free_dma(rtd->dma_ch);
 err2:
	dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,
			      rtd->dma_desc_array, rtd->dma_desc_array_phys);
 err1:
	kfree(rtd);
 out:
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return ret;
}

static int voice_pcm_close(struct snd_pcm_substream *substream)
{
	struct pxa3xx_pcm_client *client = substream->private_data;
	struct voice_runtime_data *rtd = substream->runtime->private_data;
	pxa_free_dma(rtd->dma_ch);
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	if(client->shutdown){
		client->shutdown(substream);
	}
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,
			      rtd->dma_desc_array, rtd->dma_desc_array_phys);
	kfree(rtd);
	return 0;
}
#if 0
static int
voice_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	return -1; /*dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);*/
}
#endif

static struct snd_pcm_ops voice_pcm_ops = {
	.open = voice_pcm_open,
	.close = voice_pcm_close,
	.ioctl	 = snd_pcm_lib_ioctl,
	.hw_params = voice_pcm_hw_params,
	.hw_free = voice_pcm_hw_free,
	.prepare = voice_pcm_prepare,
	.trigger = voice_pcm_trigger,
	.pointer = voice_pcm_pointer,
	.mmap = NULL,
};

static int voice_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = voice_pcm_hardware.buffer_bytes_max;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void voice_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 voice_pcm_dmamask = 0xffffffff;

/**
 * voice_pcm_new - create a voice PCM device
 * @client voice pcm client instance
 * @card: the card instance
 * @rpcm: return PCM device created
 * Returns zero if successful, or a negative error code on failure.
 */
int voice_pcm_new(struct snd_card *card, struct pxa3xx_pcm_client *client, int device, struct snd_pcm **rpcm)
 {
 	struct snd_pcm *pcm;
	int play = client->playback_params ? 1 : 0;
	int capt = client->capture_params ? 1 : 0;
	int ret;

	ret = snd_pcm_new(card, "audio_codec_voice_PCM", device, play, capt, &pcm);
	if (ret)
		goto out;

	pcm->private_data = client;
	pcm->private_free = voice_pcm_free_dma_buffers;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &voice_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (play) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;
		snd_pcm_set_ops(pcm, stream, &voice_pcm_ops);
		ret = voice_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret)
			goto out;
	}
	if (capt) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;
		snd_pcm_set_ops(pcm, stream, &voice_pcm_ops);
		ret = voice_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret)
			goto out;
	}

	if (rpcm)
		*rpcm = pcm;
	ret = 0;

 out:
	return ret;
 }
EXPORT_SYMBOL(voice_pcm_new);

