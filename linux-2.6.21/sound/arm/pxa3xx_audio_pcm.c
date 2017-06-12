/*
 * linux/sound/arm/audio_codec_pcm.c -- ALSA PCM interface for the Intel PXA2xx chip
 *
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
#include <asm/arch/ssp.h>
#include "pxa3xx_audio_pcm.h"
#include <asm/arch/codec/acodec.h>
#include <asm/arch/codec/wm9713.h>
#include <asm/arch/pxa3xx_audio_plat.h>

#undef ALSA_ZY_PCM_DEBUG

#ifdef ALSA_ZY_PCM_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG "file: " __FILE__ "Line:(%d) FUNC:%s******\n" format "\n", __LINE__ , __func__ , ##arg)

#else
#define dbg(format, arg...)
#endif

extern acodec_context_t *g_acodec_context;
extern audio_info_t audio_info;

/* hifi pcm */
struct snd_pcm * p_hifi_zy_pcm;
EXPORT_SYMBOL(p_hifi_zy_pcm);
static struct pxa3xx_pcm_dma_params hifi_pcm_out = {
	.name = "audio_zy_codec hifi PCM out",
	.dev_addr = __PREG(PCDR),
	.drcmr = &DRCMRTXPCDR,
	.dcmd = DCMD_INCSRCADDR | DCMD_FLOWTRG |
			 DCMD_BURST32 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params hifi_pcm_in = {
	.name = "audio_zy_codec hifi PCM in",
	.dev_addr = __PREG(PCDR),
	.drcmr = &DRCMRRXPCDR,
	.dcmd = DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

/* called at open pcm */
static int hifi_pcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rates = audio_info.support_hifi_output_sample_rate;
	}
	else{
		runtime->hw.rates = audio_info.support_hifi_input_sample_rate;
	}
	snd_pcm_limit_hw_rates(runtime);

	return 0;
}

/* called at prepare */
static int hifi_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	acodec_error_t state = ACODEC_SUCCESS;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		state = g_acodec_context->set_out_sample_rate(g_acodec_context, (unsigned short)runtime->rate);
	}
	else {
		state = g_acodec_context->set_in_sample_rate(g_acodec_context, (unsigned short)runtime->rate);
	}
	if (state == ACODEC_SUCCESS) {
		return 0;
	}else {
		return -1;
	}
}

static struct pxa3xx_pcm_client hifi_pcm_client = {
	.playback_params = &hifi_pcm_out,
	.capture_params = &hifi_pcm_in,
	.startup = hifi_pcm_startup,
	.shutdown = NULL,
	.prepare	= hifi_pcm_prepare,
};

/* voice pcm */
#define ZY_AUDIO_SSP_PORT 3
struct snd_pcm *p_voice_zy_pcm;
EXPORT_SYMBOL(p_voice_zy_pcm);

static struct pxa3xx_pcm_dma_params voice_pcm_out = {
	.name = 	"audio_zy_codec voice PCM out",
	.dev_addr = 	__PREG(SSDR_P3),
	.drcmr = 	&DRCMRTXSS3DR ,
	.dcmd = 	(DCMD_INCSRCADDR|DCMD_FLOWTRG|DCMD_BURST16|DCMD_WIDTH2),
};

static struct pxa3xx_pcm_dma_params voice_pcm_in = {
	.name		= "audio_zy_codec voice PCM in",
	.dev_addr	= __PREG(SSDR_P3),
	.drcmr		= &DRCMRRXSS3DR,
	.dcmd		= (DCMD_INCTRGADDR|DCMD_FLOWSRC|DCMD_BURST16|DCMD_WIDTH2),
};
static struct ssp_dev ssp;
/* called at open pcm */
static int voice_pcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rates = audio_info.support_voice_output_sample_rate;
		runtime->hw.channels_min = 1;
		runtime->hw.channels_max = 2;

	}
	else{
		runtime->hw.rates = audio_info.support_voice_input_sample_rate;
		runtime->hw.channels_min = 1;
		runtime->hw.channels_max = 2;
	}
	snd_pcm_limit_hw_rates(runtime);

	if((ret = ssp_init(&ssp,ZY_AUDIO_SSP_PORT,SSP_NO_IRQ)) < 0){
        	dbg("ini ssp error\n");
		return ret;
	}
	ssp_disable(&ssp);
	return 0;
}

extern void pxa3xx_enable_ssp3_pins(void);


int voice_pcm_active =0 ;
#ifdef CONFIG_PM
static struct ssp_state ssp3_state;
void voice_pcm_suspend(void)
{
	if(!voice_pcm_active){
		return ;
	}

	pxa_set_cken(CKEN_SSP3, 0);
	ssp_save_state(&ssp, &ssp3_state);
	return;
}

void voice_pcm_resume(void)
{
	if(!voice_pcm_active){
		return ;
	}
	pxa3xx_enable_ssp3_pins();
	pxa_set_cken(CKEN_SSP3, 1);
	ssp_restore_state(&ssp, &ssp3_state);
	return;
}
EXPORT_SYMBOL(voice_pcm_resume);
EXPORT_SYMBOL(voice_pcm_suspend);
#endif


/* called at prepare */
static int voice_pcm_prepare(struct snd_pcm_substream *substream)
{
	u32 ssp_mode = 0, ssp_setup = 0, psp_mode = 0, speed =0, rate = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	acodec_error_t state = ACODEC_SUCCESS;
	/*set MFP*/
	pxa3xx_enable_ssp3_pins();
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		state = g_acodec_context->set_voice_out_sample_rate(g_acodec_context, (unsigned short)runtime->rate);
		if( state != ACODEC_SUCCESS){
			dbg("set sample rate error %x", runtime->rate);
			return -1;
		}
	}
	else {
		state = g_acodec_context->set_voice_in_sample_rate(g_acodec_context, (unsigned short)runtime->rate);
		if( state != ACODEC_SUCCESS){
			dbg("set sample error");
			return -1;
		}
	}
	state = g_acodec_context->voice_prepare(g_acodec_context);
	if( state != ACODEC_SUCCESS){
		dbg("preare error No:%x", state);
		return -1;
	}

	ssp_mode |= SSCR0_PSP;

	/*17 bit must be set as 17 bit*/
	ssp_mode |= SSCR0_EDSS;
	ssp_mode &= ~SSCR0_DSS;
	speed = 0;
	psp_mode |= SSPSP_SFRMP;
	psp_mode |= SSPSP_SCMODE(3);

	ssp_setup = SSCR1_RxTresh(14) | SSCR1_TxTresh(1) | SSCR1_TRAIL |
                    SSCR1_TSRE | SSCR1_RSRE | SSCR1_TIE | SSCR1_RIE;

	/*SSP slave mode*/
	ssp_setup |= (SSCR1_SCLKDIR | SSCR1_SFRMDIR);


#if 0
	g_acodec_context->acodec_read(g_acodec_context, 0x3c, &reg);
        g_acodec_context->acodec_write(g_acodec_context, 0x3c, reg & ~0x0200);
	ssp_mode = 0x1010b0;
	ssp_setup = 0x3f03840;
	psp_mode = 0x7;
#endif
	dbg("config ssp");
	ssp_disable(&ssp);
	ssp_config(&ssp, ssp_mode, ssp_setup, psp_mode, rate);
	dbg("mode %x setup %x psp %x clk %x \n", ssp_mode, ssp_setup, psp_mode,
        speed);
        ssp_enable(&ssp);
	voice_pcm_active =1;
	return 0;
}

static void voice_pcm_stop(struct snd_pcm_substream *substream)
{
	voice_pcm_active = 0 ;
	ssp_disable(&ssp);
	ssp_exit(&ssp);
}



static struct pxa3xx_pcm_client voice_pcm_client = {
	.playback_params = &voice_pcm_out,
	.capture_params = &voice_pcm_in,
	.startup = voice_pcm_startup,
	.shutdown = voice_pcm_stop,
	.prepare	= voice_pcm_prepare,
};
extern int hifi_pcm_new(struct snd_card *card, struct pxa3xx_pcm_client *client, int device, struct snd_pcm **rpcm);
extern int voice_pcm_new(struct snd_card *card, struct pxa3xx_pcm_client *client, int device, struct snd_pcm **rpcm);
/**
 * audio_codec_pcm_new - create a PCM devices
 * @card: the card instance
 * Returns zero if successful, or a negative error code on failure.
 */
int audio_codec_pcm_new(struct snd_card *card)
{
	int re = 0;
	int device = 0;
	if (SUPPORT_HIFI_PCM(audio_info)) {
		re = hifi_pcm_new(card, &hifi_pcm_client, device++, &p_hifi_zy_pcm);
		if (re) {
			return -1;
		}
	}

	if (SUPPORT_VOICE_PCM(audio_info)) {
		re = voice_pcm_new(card, &voice_pcm_client, device++, &p_voice_zy_pcm);
		if (re) {
			return -1;
		}
	}
	return 0;
}

EXPORT_SYMBOL(audio_codec_pcm_new);

MODULE_AUTHOR("jingqing.xu@intel.com");
MODULE_DESCRIPTION("ALSA ZY PCM Moudle");
MODULE_LICENSE("GPL");


