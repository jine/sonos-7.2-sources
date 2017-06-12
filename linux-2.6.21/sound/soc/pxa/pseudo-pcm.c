/*
 * linux/sound/soc/pxa/pxa3xx-vpcm.c
 * 
 * Copyright (C) 2007 Marvell International Ltd.  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>
#include <asm/arch/micco.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ac97codec-pm.h" 


static const struct snd_pcm_hardware pseudo_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192 - 32,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/sizeof(pxa_dma_desc),
	.buffer_bytes_max	= 128 * 1024,
	.fifo_size		= 32,
};


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


static void pseudo_pcm_timer_function(unsigned long data)
{
       struct pseudo_runtime_data *prtd = (struct pseudo_runtime_data *)data;

       prtd->timer.expires = 1 + jiffies;
       add_timer(&prtd->timer);
       spin_lock_irq(&prtd->lock);
       prtd->pcm_irq_pos += prtd->pcm_jiffie;
       prtd->pcm_buf_pos += prtd->pcm_jiffie;
       prtd->pcm_buf_pos %= prtd->pcm_size;
       if (prtd->pcm_irq_pos >= prtd->pcm_count) {
               prtd->pcm_irq_pos %= prtd->pcm_count;
               snd_pcm_period_elapsed(prtd->substream);
       }
       spin_unlock_irq(&prtd->lock);
}

static void pseudo_pcm_timer_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd = runtime->private_data;

	prtd->timer.expires = 1 + jiffies;
	add_timer(&prtd->timer);
}

static void pseudo_pcm_timer_stop(struct snd_pcm_substream * substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd = runtime->private_data;

	del_timer(&prtd->timer);
}

static int pseudo_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;
	return 0;
}

static int pseudo_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static int pseudo_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd = runtime->private_data;
	unsigned int bps;


	/* prepare for pseudo transmission */
	bps = runtime->rate * runtime->channels;
	bps *= snd_pcm_format_width(runtime->format);
	bps /= 8;
	if (bps <= 0)
		return -EINVAL;
	prtd->pcm_bps = bps;
	prtd->pcm_jiffie = bps / HZ;
	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	prtd->pcm_buf_pos = 0;

	return 0;
}

static int pseudo_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pseudo_runtime_data *prtd = substream->runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_cpu_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_codec_dai *codec_dai = machine->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
#ifdef CONFIG_MACH_LITTLETON
		codec->write(codec, MICCO_AUDIO_LINE_AMP, 0x0);
		codec->write(codec, MICCO_STEREO_AMPLITUDE_CH1, 0x7f);
		codec->write(codec, MICCO_HIFI_DAC_CONTROL, 0x0);
		codec->write(codec, MICCO_MONO_VOL, 0xbf);
		codec->write(codec, MICCO_BEAR_VOL, 0xbf);
		codec->write(codec, MICCO_MIC_PGA, 0x0);
		codec->write(codec, MICCO_VCODEC_VDAC_CONTROL, 0xc);		
#endif		
		pseudo_pcm_timer_start(substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pseudo_pcm_timer_stop(substream);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		pseudo_pcm_timer_start(substream);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pseudo_pcm_timer_start(substream);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
pseudo_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd = runtime->private_data;

	return bytes_to_frames(runtime, prtd->pcm_buf_pos);
}

static int pseudo_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &pseudo_pcm_hardware);

	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime, 
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct pseudo_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	prtd->addr = dma_alloc_coherent(NULL, PAGE_SIZE, &(prtd->addr_phys),
						GFP_KERNEL);
	if (!prtd->addr) {
		ret = -ENOMEM;
		goto err1;
	}
	init_timer(&(prtd->timer));
	prtd->timer.data = (unsigned long)prtd;
	prtd->timer.function = pseudo_pcm_timer_function;
	spin_lock_init(&(prtd->lock));
	prtd->substream = substream;
	runtime->private_data = prtd;
	return 0;

 err1:
	kfree(prtd);
 out:
	return ret;
}

static int pseudo_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct pseudo_runtime_data *prtd = runtime->private_data;

	dma_free_coherent(NULL, PAGE_SIZE, prtd->addr, prtd->addr_phys);
	kfree(prtd);
	return 0;
}

static int pseudo_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

struct snd_pcm_ops pseudo_pcm_ops = {
	.open		= pseudo_pcm_open,
	.close		= pseudo_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= pseudo_pcm_hw_params,
	.hw_free	= pseudo_pcm_hw_free,
	.prepare	= pseudo_pcm_prepare,
	.trigger	= pseudo_pcm_trigger,
	.pointer	= pseudo_pcm_pointer,
	.mmap		= pseudo_pcm_mmap,
};

static int pseudo_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = pseudo_pcm_hardware.buffer_bytes_max;
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

static void pseudo_pcm_free_dma_buffers(struct snd_pcm *pcm)
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

static u64 pseudo_pcm_dmamask = DMA_32BIT_MASK;

int pseudo_pcm_new(struct snd_card *card, struct snd_soc_codec_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &pseudo_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		ret = pseudo_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = pseudo_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform pseudo_soc_platform = {
	.name		= "pseudo-audio",
	.pcm_ops 	= &pseudo_pcm_ops,
	.pcm_new	= pseudo_pcm_new,
	.pcm_free	= pseudo_pcm_free_dma_buffers,
};

EXPORT_SYMBOL_GPL(pseudo_soc_platform);

#define PSEUDO_RATES 0xffffffff
#define PSEUDO_FORMATS 0xffffffff
struct snd_soc_cpu_dai pseudo_cpu_dai = {
	.name = "pseudo cpu dai",
	.type = SND_SOC_DAI_PCM,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = PSEUDO_RATES,
		.formats = PSEUDO_FORMATS,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = PSEUDO_RATES,
		.formats = PSEUDO_FORMATS,},
		
};

struct snd_soc_codec_dai pseudo_codec_dai = {
	.name = "pseudo codec dai",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = PSEUDO_RATES,
		.formats = PSEUDO_FORMATS,},
	.capture = {
		.stream_name = "HiFi Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = PSEUDO_RATES,
		.formats = PSEUDO_FORMATS,},
}



MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("PSEUDO PCM module");
MODULE_LICENSE("GPL");
