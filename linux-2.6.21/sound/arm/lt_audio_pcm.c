/*
 * linux/sound/arm/lt_audio_pcm.c
 *
 * Based on sound/arm/pxa2xx-pcm.c
 * Author:	Yin, Fengwei (fengwei.yin@marvell.com)
 * Created:	Nov, 2006
 * Copyright (C) 2006, Mavrell Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/ssp.h>
#include <asm/arch/micco.h>

#include <asm/arch/codec/lt_micco_acodec.h>
#include <asm/arch/codec/lt_micco.h>

#include "lt_audio_pcm.h"
#include "lt_audio.h"


#define	DEBUG

extern acodec_context_t *g_acodec_context;
extern audio_info_t audio_info;


/* On Littleton, the HIFI is routed to SSP3. And PCM is routed to SSP4 */
/* FIXME:
 *	SSP configure routine, maybe need move them to SSP driver.
 * TODO:
 *	support Monahans as SSP slave (clock and frame).
 */
enum {
	SSP_VOICE_TX = 0,
	SSP_VOICE_RX = 1,
};

static int config_ssp_voice_for_micco(int port, int rate, int mode)
{
	unsigned long sscr0, sscr1, sspsp;

	sscr0 = 0x00C0003F;
	sscr1 = 0x00701DC0;
	sspsp = 0x00800085;

	switch (rate) {
		case 8000:
			sscr0 |= (89 << 8);
			break;
		case 16000:
			sscr0 |= (44 << 8);	
			break;
		case 32000:
			sscr0 |= (22 << 8);
			break;
		default:
			return -EINVAL;
	}
	
	if (SSP_VOICE_TX == mode) {
		sscr1 &= ~0x00800000;	
	} else {
		sscr1 |= 0x00800000;
	}

	SSCR0_P(port) = sscr0;	
	SSCR1_P(port) = sscr1;
	SSPSP_P(port) = sspsp;
	return 0;
}

static int config_ssp_hifi_for_micco(int port, int rate, int mode)
{
	unsigned long sscr0, sscr1, sspsp, sstsa;
	unsigned long ssacd, ssacdd, ssrsa;

	/* Because the internal 13M clock will be 10M in D0CS,
	 * we route SSP_CLK to GPIO126(EXT_CLK) and let SSP select
	 * NETWORK CLK as CLK source.
	 * This workaround need an ECO on Littleton mainboard.
	 */
	// sscr0 = 0xA1E0003F;
	sscr0 = 0xE1C0003F; // XXX BT use internal audio clock
	sscr1 = 0x00701DC0;
	sspsp = 0x40200004;
	sstsa = 0x00000003;
	ssrsa = 0x00000003;
	// ssacd = 0x60;
	ssacd = 0x20;
	// ssacdd= 0x00000040;

	switch (rate) {
		case 48000:
			// sscr0 |= 0x00000300;
			break;
		default:
			return -EINVAL;
	}
	
	SSCR0_P(port) = sscr0;	
	SSCR1_P(port) = sscr1;
	SSPSP_P(port) = sspsp;
	SSTSA_P(port) = sstsa;
	SSRSA_P(port) = ssrsa;
	SSACD_P(port) = ssacd;
	// SSACDD_P(port) = ssacdd;
	return 0;
}


/* hifi pcm */
#define AUDIO_HIFI_SSP_PORT	3	
struct snd_pcm *p_hifi_lt_pcm;
EXPORT_SYMBOL(p_hifi_lt_pcm);

static audio_pcm_dma_params_t hifi_pcm_out = {
	.name		= "audio_lt_codec hifi PCM out",
	.dev_addr	= __PREG(SSDR_P3),
	.drcmr		= &DRCMRTXSS3DR,
	.dcmd		= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				 DCMD_BURST32 | DCMD_WIDTH4,
};

static struct ssp_dev hifi_ssp;
static int hifi_pcm_active =0 ;

#ifdef CONFIG_PM
extern void disable_oscc_pout(void);
extern void enable_oscc_pout(void);

static struct ssp_state ssp3_state;
void hifi_pcm_suspend(void)
{
	if(!hifi_pcm_active){
		return ;
	}
	
	pxa_set_cken(CKEN_SSP3, 0);
	ssp_save_state(&hifi_ssp, &ssp3_state);

	// XXX disable_oscc_pout();
	return;
}

void hifi_pcm_resume(void)
{
	if(!hifi_pcm_active){
		return ;
	}
	// XXX enable_oscc_pout();
	pxa3xx_enable_ssp3_pins();
	pxa_set_cken(CKEN_SSP3, 1);
	ssp_restore_state(&hifi_ssp, &ssp3_state);
	return;
}
EXPORT_SYMBOL(hifi_pcm_resume);
EXPORT_SYMBOL(hifi_pcm_suspend);
#endif

/* called when pcm is opened*/
static int hifi_pcm_startup(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rates = audio_info.support_hifi_output_sample_rate;
	} else {
		runtime->hw.rates = audio_info.support_hifi_input_sample_rate;
	}
	snd_pcm_limit_hw_rates(runtime);

	pxa_set_cken(CKEN_SSP3, 1);
	pxa3xx_enable_ssp3_pins();
	if((ret = ssp_init(&hifi_ssp,AUDIO_HIFI_SSP_PORT,SSP_NO_IRQ)) < 0){
		pxa_set_cken(CKEN_SSP3, 0);
		return ret;
	}
	ssp_disable(&hifi_ssp);

	return 0;
}

/* called when prepare */
static int hifi_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	acodec_error_t state = ACODEC_SUCCESS;
	int ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		state = g_acodec_context->set_out_sample_rate(
				g_acodec_context, (unsigned short)runtime->rate);
	} else
		return -EINVAL;

	/* Configure SSP3 to HIFI interface --by yfw*/
	ssp_disable(&hifi_ssp);
	ret = config_ssp_hifi_for_micco(hifi_ssp.port, runtime->rate, 0);
	if (ret) {
		printk(KERN_ERR "configure ssp for hifi failed!\n");
		return ret;
	}
	ssp_enable(&hifi_ssp);

	hifi_pcm_active = 1;
	// enable_oscc_pout();

	if (state == ACODEC_SUCCESS) {
		return 0;
	} else {
		printk(KERN_ERR "set sample rate failed");
		return -1;
	}
}

static void hifi_pcm_stop(struct snd_pcm_substream *substream)
{

	hifi_pcm_active = 0;
	ssp_disable(&hifi_ssp);
	ssp_exit(&hifi_ssp);

	pxa_set_cken(CKEN_SSP3, 0);
	// disable_oscc_pout();
}

static auido_pcm_client_t hifi_pcm_client = {
	.playback_params	= &hifi_pcm_out,
	.startup		= hifi_pcm_startup,
	.shutdown		= hifi_pcm_stop,
	.prepare		= hifi_pcm_prepare,
};

/* voice pcm */
#define AUDIO_PCM_SSP_PORT	4	
struct snd_pcm *p_voice_lt_pcm;
EXPORT_SYMBOL(p_voice_lt_pcm);

static audio_pcm_dma_params_t voice_pcm_out = {
	.name		= "audio_lt_codec voice PCM out",
	.dev_addr = 	__PREG(SSDR_P4),
	.drcmr = 	&DRCMR3,
	.dcmd = 	(DCMD_INCSRCADDR | DCMD_FLOWTRG | 	\
				DCMD_BURST16 | DCMD_WIDTH2),
};

static audio_pcm_dma_params_t voice_pcm_in = {
	.name		= "audio_lt_codec voice PCM in",
	.dev_addr	= __PREG(SSDR_P4),
	.drcmr		= &DRCMR2,
	.dcmd		= (DCMD_INCTRGADDR | DCMD_FLOWSRC |	\
				DCMD_BURST16 | DCMD_WIDTH2),
};

static struct ssp_dev pcm_ssp;
static int voice_pcm_active =0 ;

/* called at open pcm */
static int voice_pcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rates = audio_info.support_voice_output_sample_rate;
		runtime->hw.channels_min = 1;
		runtime->hw.channels_max = 1;
	} else {
		runtime->hw.rates = audio_info.support_voice_input_sample_rate;
		runtime->hw.channels_min = 1;
		runtime->hw.channels_max = 1;
	}

	snd_pcm_limit_hw_rates(runtime);
	
	/*set MFP*/
	pxa3xx_enable_ssp4_pins();
	pxa_set_cken(CKEN_SSP4, 1);

	if((ret = ssp_init(&pcm_ssp,AUDIO_PCM_SSP_PORT,SSP_NO_IRQ)) < 0){
        	printk(KERN_ERR "init ssp error\n");
		return ret;
	}

	ssp_disable(&pcm_ssp);
	return 0;
}


/* called at prepare */
static int voice_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ssp_mode;
	struct snd_pcm_runtime *runtime = substream->runtime;
	acodec_error_t state = ACODEC_SUCCESS;
	int ret;

	printk("%s: enter\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		state = g_acodec_context->set_voice_out_sample_rate(
				g_acodec_context, (unsigned short)runtime->rate);
		if( state != ACODEC_SUCCESS){
			printk(KERN_ERR "set sample rate error %x", runtime->rate);
			return -1;
		}
		ssp_mode = SSP_VOICE_TX;
	} else {
		state = g_acodec_context->set_voice_in_sample_rate(
				g_acodec_context, (unsigned short)runtime->rate);
		if( state != ACODEC_SUCCESS){
			printk(KERN_ERR "set sample error");
			return -1;
		}
		ssp_mode = SSP_VOICE_RX;
		/* workaound for Micco record. */
		micco_write(0x90, 0x01);
		micco_write(0x94, 0x40);
		micco_write(0x90, 0x00);
	}

	state = g_acodec_context->voice_prepare(g_acodec_context);
	if( state != ACODEC_SUCCESS){
		return -1;
	}

	/* Configure the SSP3 for PCM */
	ssp_disable(&pcm_ssp);
	ret = config_ssp_voice_for_micco(pcm_ssp.port, runtime->rate, ssp_mode);
	if (ret) {
		printk(KERN_ERR "configure SSP for voice failed\n");
		return ret;
	}

        ssp_enable(&pcm_ssp);
	voice_pcm_active =1;

	// enable_oscc_pout();
	return 0;
}

static void voice_pcm_stop(struct snd_pcm_substream *substream)
{
	voice_pcm_active = 0 ;
	ssp_disable(&pcm_ssp);
	ssp_exit(&pcm_ssp);	
	pxa_set_cken(CKEN_SSP4, 0);

	// disable_oscc_pout();
}

static auido_pcm_client_t voice_pcm_client = {
	.playback_params	= &voice_pcm_out,
	.capture_params		= &voice_pcm_in,
	.startup		= voice_pcm_startup,
	.shutdown		= voice_pcm_stop,
	.prepare		= voice_pcm_prepare,
};

#ifdef CONFIG_PM
static struct ssp_state ssp4_state;
void voice_pcm_suspend(void)
{
	if(!voice_pcm_active){
		return ;
	}
	
	pxa_set_cken(CKEN_SSP4, 0);
	ssp_save_state(&pcm_ssp, &ssp4_state);
	// XXX disable_oscc_pout();
	return;
}

void voice_pcm_resume(void)
{
	if(!voice_pcm_active){
		return ;
	}

	// XXX enable_oscc_pout();
	pxa3xx_enable_ssp4_pins();	
	pxa_set_cken(CKEN_SSP4, 1);
	ssp_restore_state(&pcm_ssp, &ssp4_state);
	return;
}
EXPORT_SYMBOL(voice_pcm_resume);
EXPORT_SYMBOL(voice_pcm_suspend);
#else
void voice_pcm_suspend(void)	{}
void voice_pcm_resume(void)	{}
#endif

extern int hifi_pcm_new(struct snd_card *card,
		auido_pcm_client_t *client, int device, struct snd_pcm **rpcm);
extern int voice_pcm_new(struct snd_card *card,
		auido_pcm_client_t *client, int device, struct snd_pcm **rpcm);

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
		re = hifi_pcm_new(card, &hifi_pcm_client,
			device++, &p_hifi_lt_pcm);
		if (re) {
			return -1;
		}
	}

	if (SUPPORT_VOICE_PCM(audio_info)) {
		re = voice_pcm_new(card, &voice_pcm_client,
			device++, &p_voice_lt_pcm);
		if (re) {
			return -1;
		}
	}
	return 0;
}

EXPORT_SYMBOL(audio_codec_pcm_new);

MODULE_AUTHOR("fengwei.yin@marvell.com");
MODULE_DESCRIPTION("HIFI/PCM module for Littleton");
MODULE_LICENSE("GPL");

