/*
 *  --linux/sound/arm/audio_codec_hifi_pcm.c: ALSA hifi PCM interface for PXA3xx 
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
#undef ALSA_ZY_HIFI_PCM_DEBUG

#ifdef ALSA_ZY_HIFI_PCM_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG format "\n",##arg)
#else
#define dbg(format, arg...)
#endif

extern int golden_muted;

static const struct snd_pcm_hardware hifi_pcm_hardware = {
// XXX BT mmap doesn't seem to be working for me don't know why...
	.info			= // SNDRV_PCM_INFO_MMAP |
				  // SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	/* The period must be the multiple of 32 bytes
	 * because the AC97 link requires the value of DCMD.LEN
	 * must be the multiple of 32 bytes.
	 * so the minimal period bytes should be 32 bytes
	 */
	.period_bytes_min	= 32,
	/* The max value of DCMD.LEN is 8K-1, and AC97
	 * link requires the value of DCMD.LEN must be
	 * mutiple of 32 bytes, so the maximl period bytes should be 8k-32.
	 */
	.period_bytes_max	= 8192 - 32,
	.periods_min		= 1,
	/* The size of descriptor area is PAGE_SIZE */
	.periods_max		= PAGE_SIZE/sizeof(pxa_dma_desc),
	/* playback/capture ring buffer max size
	 * ALSA use this vaule to pre allocate the ring buffer
	 */
	.buffer_bytes_max	= 128 * 1024,
	/* bytes */
	.fifo_size		= 64,
	.channels_min = 2,
	.channels_max= 2,
};

/*
 * structure for pseudo playback. At this time, data won't be transferred to audio codec.
 * Instead data will be costed by simulator. It's specialized usage of audio codec muted.
 */
struct pseudo_runtime_data {
	void *addr;		/* It's pseudo address */
	dma_addr_t addr_phys;
	void *rsv_dma_addr;	/* If use pseudo address, the real address is stored in it */
	dma_addr_t rsv_dma_addr_phys;
	spinlock_t lock;
	struct timer_list timer;
	unsigned int pcm_size;
	unsigned int pcm_count;
	unsigned int pcm_bps;
	unsigned int pcm_jiffie;
	unsigned int pcm_irq_pos;
	unsigned int pcm_buf_pos;
	struct snd_pcm_substream *substream;
};

struct hifi_runtime_data {
	int dma_ch;
	struct pxa3xx_pcm_dma_params *params;
	pxa_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;
	dma_addr_t pause_dma_desc_array_phys;
	int pause;
	struct pseudo_runtime_data pseudo;
};

static int hifi_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hifi_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	pxa_dma_desc *dma_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;

	printk(KERN_DEBUG "totsize size:0x%x period size:0x%x\n", totsize, period);
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	dma_desc = rtd->dma_desc_array;
	next_desc_phys = rtd->dma_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;
	do {
		next_desc_phys += sizeof(pxa_dma_desc);
		dma_desc->ddadr = next_desc_phys;
		if (golden_board) {
			if (golden_muted) {
				if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
					dma_desc->dsadr = dma_buff_phys;
					dma_desc->dtadr = rtd->pseudo.addr_phys;
				} else {
					dma_desc->dsadr = rtd->pseudo.addr_phys;
					dma_desc->dtadr = dma_buff_phys;
				}
			} else {
				if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
					dma_desc->dsadr = dma_buff_phys;
					dma_desc->dtadr = rtd->params->dev_addr;
				} else {
					dma_desc->dsadr = rtd->params->dev_addr;
					dma_desc->dtadr = dma_buff_phys;
				}
			}
 		} else {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma_desc->dsadr = dma_buff_phys;
				dma_desc->dtadr = rtd->params->dev_addr;
			} else {
				dma_desc->dsadr = rtd->params->dev_addr;
				dma_desc->dtadr = dma_buff_phys;
			}
		}
		if (period > totsize)
			period = totsize;
		dma_desc->dcmd = rtd->params->dcmd | period | DCMD_ENDIRQEN;
		dma_desc++;
		dma_buff_phys += period;
	} while (totsize -= period);
	dma_desc[-1].ddadr = rtd->dma_desc_array_phys;

	return 0;
}

static int hifi_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct hifi_runtime_data *rtd = substream->runtime->private_data;

	*rtd->params->drcmr = 0;
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int hifi_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct pxa3xx_pcm_client *client = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hifi_runtime_data *rtd = runtime->private_data;
	struct pseudo_runtime_data *dpcm = &(rtd->pseudo);
	unsigned int bps;
	int ret;

	if (golden_board) {
		set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
		/* prepare for pseudo transmission */
		bps = runtime->rate * runtime->channels;
		bps *= snd_pcm_format_width(runtime->format);
		bps /= 8;
		if (bps <= 0)
			return -EINVAL;
		dpcm->pcm_bps = bps;
		dpcm->pcm_jiffie = bps / HZ;
		dpcm->pcm_size = snd_pcm_lib_buffer_bytes(substream);
		dpcm->pcm_count = snd_pcm_lib_period_bytes(substream);
		dpcm->pcm_irq_pos = 0;
		dpcm->pcm_buf_pos = 0;

		/* prepare for hw transmission */
		rtd->pause = 0;
		DCSR(rtd->dma_ch) &= ~DCSR_RUN;
		DCSR(rtd->dma_ch) = 0;
		DCMD(rtd->dma_ch) = 0;
		*rtd->params->drcmr = rtd->dma_ch | DRCMR_MAPVLD;
		if (client->prepare) {
			ret = client->prepare(substream);
			set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
			return ret;
		}
		set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
		return 0;
	} else {
		set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
		rtd->pause = 0;
		DCSR(rtd->dma_ch) &= ~DCSR_RUN;
		DCSR(rtd->dma_ch) = 0;
		DCMD(rtd->dma_ch) = 0;
		*rtd->params->drcmr = rtd->dma_ch | DRCMR_MAPVLD;
		if (client->prepare) {
			ret = client->prepare(substream);
			set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
			return ret;
		}
		set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
		return 0;
	}
}

static void pxa3xx_pseudo_pcm_timer_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hifi_runtime_data *rtd = runtime->private_data;
	struct pseudo_runtime_data *dpcm = &(rtd->pseudo);

	dpcm->timer.expires = 1 + jiffies;
	add_timer(&dpcm->timer);
}

static void pxa3xx_pseudo_pcm_timer_stop(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hifi_runtime_data *rtd = runtime->private_data;
	struct pseudo_runtime_data *dpcm = &(rtd->pseudo);

	del_timer(&dpcm->timer);
}

static void pxa3xx_pseudo_pcm_timer_function(unsigned long data)
{
       struct pseudo_runtime_data *dpcm = (struct pseudo_runtime_data *)data;

       dpcm->timer.expires = 1 + jiffies;
       add_timer(&dpcm->timer);
       spin_lock_irq(&dpcm->lock);
       dpcm->pcm_irq_pos += dpcm->pcm_jiffie;
       dpcm->pcm_buf_pos += dpcm->pcm_jiffie;
       dpcm->pcm_buf_pos %= dpcm->pcm_size;
       if (dpcm->pcm_irq_pos >= dpcm->pcm_count) {
               dpcm->pcm_irq_pos %= dpcm->pcm_count;
               snd_pcm_period_elapsed(dpcm->substream);
       }
       spin_unlock_irq(&dpcm->lock);
}

static int hifi_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hifi_runtime_data *rtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (golden_board && golden_muted) {
			pr_debug("%s: golden_muted:%d\n", __FUNCTION__, golden_muted);
			pxa3xx_pseudo_pcm_timer_start(substream);
		} else {
			set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
			DDADR(rtd->dma_ch) = rtd->dma_desc_array_phys;
			DCSR(rtd->dma_ch) = DCSR_RUN;
			dbg("dma start!\n");
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (golden_board && golden_muted) {
			pr_debug("%s: golden_muted:%d\n", __FUNCTION__, golden_muted);
			pxa3xx_pseudo_pcm_timer_stop(substream);
		} else {
			DCSR(rtd->dma_ch) &= ~DCSR_RUN;
			set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
		}
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		rtd->pause = 1;
		if (!golden_muted)
			set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rtd->pause = 0;
		DDADR(rtd->dma_ch) = rtd->pause_dma_desc_array_phys;
		DCSR(rtd->dma_ch) |= DCSR_RUN;
		if (!golden_muted)
			set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static void hifi_pcm_dma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct hifi_runtime_data *rtd = substream->runtime->private_data;
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

static snd_pcm_uframes_t hifi_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hifi_runtime_data *rtd = runtime->private_data;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	if (golden_board && golden_muted) {
		return bytes_to_frames(runtime, rtd->pseudo.pcm_buf_pos);
	} else {
		ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			DSADR(rtd->dma_ch) : DTADR(rtd->dma_ch);
		x = bytes_to_frames(runtime, ptr - runtime->dma_addr);
		if (x == runtime->buffer_size)
			x = 0;
		return x;
	}
}

static int
hifi_pcm_hw_rule_mult32(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
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

static int hifi_pcm_open(struct snd_pcm_substream *substream)
{
	struct pxa3xx_pcm_client *client = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hifi_runtime_data *rtd;
	int ret;

	runtime->hw = hifi_pcm_hardware;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	/* The period must be the multiple of 32 bytes
	 * because the AC97 link requires the value of DCMD.LEN
	 * must be the multiple of 32 bytes.
	 * So here we set the rule to make period size and buffer size
	 * to be the multiple of 32 bytes.
	 */
	ret = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				  hifi_pcm_hw_rule_mult32, NULL,
				  SNDRV_PCM_HW_PARAM_PERIOD_BYTES, -1);
	if (ret)
		goto out;
	ret = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				  hifi_pcm_hw_rule_mult32, NULL,
				  SNDRV_PCM_HW_PARAM_BUFFER_BYTES, -1);
	if (ret)
		goto out;

	ret = -ENOMEM;
	rtd = kmalloc(sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		goto out;
	rtd->dma_desc_array =
		dma_alloc_writecombine(substream->pcm->card->dev, PAGE_SIZE,
				       &rtd->dma_desc_array_phys, GFP_KERNEL);
	if (!rtd->dma_desc_array)
		goto err1;

	if (golden_board) {
		rtd->pseudo.addr = dma_alloc_coherent(NULL, PAGE_SIZE, &(rtd->pseudo.addr_phys),
							GFP_KERNEL);
		if (!rtd->pseudo.addr)
			goto err2;
		init_timer(&(rtd->pseudo.timer));
		rtd->pseudo.timer.data = (unsigned long)(&(rtd->pseudo));
		rtd->pseudo.timer.function = pxa3xx_pseudo_pcm_timer_function;
		spin_lock_init(&(rtd->pseudo.lock));
		rtd->pseudo.substream = substream;
	}

	rtd->params = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		      client->playback_params : client->capture_params;
	ret = pxa_request_dma(rtd->params->name, DMA_PRIO_LOW,
			      hifi_pcm_dma_irq, substream);
	if (ret < 0)
		goto err3;
	rtd->dma_ch = ret;

	runtime->private_data = rtd;
	if(client->startup){
		ret = client->startup(substream);
		if (!ret)
			goto out;
	}

	pxa_free_dma(rtd->dma_ch);
err3:
	if (golden_board)
		dma_free_coherent(NULL, PAGE_SIZE, rtd->pseudo.addr, rtd->pseudo.addr_phys);
err2:
	dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,
			      rtd->dma_desc_array, rtd->dma_desc_array_phys);
err1:
	kfree(rtd);
out:
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return ret;
}

static int hifi_pcm_close(struct snd_pcm_substream *substream)
{
	struct pxa3xx_pcm_client *client = substream->private_data;
	struct hifi_runtime_data *rtd = substream->runtime->private_data;
	pxa_free_dma(rtd->dma_ch);
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	if(client->shutdown){
		client->shutdown(substream);
	}
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	if (golden_board)
		dma_free_coherent(NULL, PAGE_SIZE, rtd->pseudo.addr, rtd->pseudo.addr_phys);
	dma_free_writecombine(substream->pcm->card->dev, PAGE_SIZE,
			      rtd->dma_desc_array, rtd->dma_desc_array_phys);
	kfree(rtd);
	return 0;
}
#if 0
static int
hifi_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	return -1; /*dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);*/
}
#endif

static struct snd_pcm_ops hifi_pcm_ops = {
	.open = hifi_pcm_open,
	.close = hifi_pcm_close,
	.ioctl	 = snd_pcm_lib_ioctl,
	.hw_params = hifi_pcm_hw_params,
	.hw_free = hifi_pcm_hw_free,
	.prepare = hifi_pcm_prepare,
	.trigger = hifi_pcm_trigger,
	.pointer = hifi_pcm_pointer,
	.mmap = NULL,
};

static int hifi_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = hifi_pcm_hardware.buffer_bytes_max;

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

static void hifi_pcm_free_dma_buffers(struct snd_pcm *pcm)
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

static u64 hifi_pcm_dmamask = 0xffffffff;

/**
 * hifi_pcm_new - create a hifi PCM device
 * @client hifi pcm client instance
 * @card: the card instance
 * @rpcm: return PCM device created
 * Returns zero if successful, or a negative error code on failure.
 */
int hifi_pcm_new(struct snd_card *card, struct pxa3xx_pcm_client *client, int device, struct snd_pcm **rpcm)
 {
 	struct snd_pcm *pcm;
	int play = client->playback_params ? 1 : 0;
	int capt = client->capture_params ? 1 : 0;
	int ret;

	ret = snd_pcm_new(card, "audio_codec_pxa3xx_hifi_PCM", device, play, capt, &pcm);
	if (ret)
		goto out;

	pcm->private_data = client;
	pcm->private_free = hifi_pcm_free_dma_buffers;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &hifi_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (play) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;
		snd_pcm_set_ops(pcm, stream, &hifi_pcm_ops);
		ret = hifi_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret)
			goto out;
	}
	if (capt) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;
		snd_pcm_set_ops(pcm, stream, &hifi_pcm_ops);
		ret = hifi_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret)
			goto out;
	}

	if (rpcm)
		*rpcm = pcm;
	ret = 0;

 out:
	return ret;
 }
EXPORT_SYMBOL(hifi_pcm_new);


