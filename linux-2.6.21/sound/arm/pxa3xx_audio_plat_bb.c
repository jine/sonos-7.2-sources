/*
 *  linux/sound/arm/audio_codec_plat.c
 *
 *  Author:	Jingqing Xu
 *  Created:	Mar 21, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/arch/pxa-regs.h>
#include <linux/errno.h>
#include <asm/hardware.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <linux/miscdevice.h>
#include <asm/arch/pxa3xx_audio_plat.h>

#ifdef CONFIG_MACH_ZYLONITE
#include <asm/arch/codec/ac97acodec.h>
#include <asm/arch/codec/wm9713.h>
#include <asm/arch/codec/acodec.h>

audio_info_t audio_info = {
	.support_pcm = AUDIO_CODEC_HIFI_PCM | AUDIO_CODEC_VOICE_PCM,
	.support_input_control = HIFI_NEAR_IN_CONTROL|MIC1_IN_CONTROL|MIC2_IN_CONTROL|HIFI_NEAR_IN_CONTROL|FAR_IN_CONTROL,
	.support_output_control = STEREO_HEAD_SET_OUT_CONTROL|HIFI_NEAR_OUT_CONTROL|FAR_OUT_CONTROL,
	.support_vol_control = MIC1_GAIN|STEREO_HEAD_SET_VOL,
	.support_other_control = 0,
	.support_hifi_input_sample_rate = 0xfe,
	.support_voice_input_sample_rate = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000| SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
	.support_hifi_output_sample_rate = 0xfe,
	.support_voice_output_sample_rate = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000| SNDRV_PCM_RATE_48000,
};
#elif
	/* different platform code here */
#endif

EXPORT_SYMBOL(audio_info);

#undef DEBUG_ALSA_ZY
#ifdef DEBUG_ALSA_ZY
#ifdef CONFIG_MACH_ZYLONITE
p_acodec_context_t debug_pac97ctxt = NULL;
int alsa_codec_regs_dump(p_acodec_context_t p_ac97_ctxt)
{
	u16 i=0,val=0;

	for (i=0 ; i<=0x7e ; i=i+2 ){
		if(ACODEC_SUCCESS == zy_ac97_acodec_read(p_ac97_ctxt,i,&val)){
			printk(KERN_INFO "reg addr: 0x%x---value: 0x%x\n", i, val );
		}
		else{
			printk(KERN_INFO "reg addr: 0x%x read error!\n", i);
		}
	}
	return 0;
}

static ssize_t audio_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
       u16 val;
       acodec_error_t re;
       u16 __user *t = buf;
       if (get_user(val, t)) {
               printk(KERN_ERR "get user error!\n");
               return -EFAULT;
               }
       printk(KERN_DEBUG "write:addr:0x%x value:0x%x\n", count, val);
       re = zy_ac97_acodec_write(debug_pac97ctxt, (u16)(count), (u16)val);
       if( re != ACODEC_SUCCESS){
               printk(KERN_ERR "write error!\n");
               return -EIO;
       }
       return 0;
}

static ssize_t audio_read(struct file *file, char __user *buf,
                       size_t count, loff_t *ppos)
{
       u16 val;
       acodec_error_t re;
       re = zy_ac97_acodec_read(debug_pac97ctxt, (u16)(count), &val);
       if(re != ACODEC_SUCCESS){
              printk(KERN_ERR "read error!\n");
	      return -EIO;
       }
       if(put_user(val,(u16 __user *)buf)){
              printk(KERN_ERR "put user error!\n");
	      return -EFAULT;
       }
       return 0;
}

static int audio_open(struct inode *inode, struct file *file)
{
       printk(KERN_DEBUG "Read from audio codec; Write to audio codec\n");
       return 0;
}

/*
 *     The various file operations we support.
 */
static struct file_operations audio_fops = {
       .owner          = THIS_MODULE,
       .read           = audio_read,
       .write          = audio_write,
       .open           = audio_open,
};

static struct miscdevice audio_dev=
{
       AUDIO_MINOR,
       "audio",
       &audio_fops
};
#elif

/* different platform code here */
#endif/* end ifdefCONFIG_MACH_ZYLONITE */
#else
int alsa_codec_regs_dump(p_acodec_context_t p_ac97_ctxt)
{
	return 0;
}
#endif
EXPORT_SYMBOL(alsa_codec_regs_dump);




/**
* set_card_shortname- set the shortname for card
* card: card instance
* g_acodec_context: context of codec
**/
void set_card_shortname(struct snd_card *card, p_acodec_context_t g_acodec_context)
{
	switch(g_acodec_context->acodec_id) {
		case WM_9713_ID:
			snprintf(card->shortname, sizeof(card->shortname), "WM9713");
			break;
		case WM_9712_ID:
			snprintf(card->shortname, sizeof(card->shortname), "WM9712");
			break;
		default:
			snprintf(card->shortname, sizeof(card->shortname), "unknown codec");
	}
}


static void * p_saved_memory = NULL;
static void * p_zy_scenario = NULL;
/**
 * alsa_prepare_for_zy - create and initialize the p_acodec_context_t
 * 				   open the clock of data link
 * @p_p_zy_ctxt: return the data structure p_acodec_context_t
 * return: 0 success ; -ENOMEM
 **/
int alsa_prepare_for_zy(p_acodec_context_t * p_p_zy_ctxt)
{
	p_acodec_context_t p_zy_ctxt = NULL;
	p_zy_ctxt = kmalloc(sizeof(acodec_context_t), GFP_KERNEL);
	if (!p_zy_ctxt)
		return -ENOMEM;
	memset(p_zy_ctxt, 0, sizeof(acodec_context_t));

#ifdef CONFIG_MACH_ZYLONITE
	p_saved_memory =
		    kmalloc(sizeof(ZY_9713_CONTEXT_SAVE_T) +
			    sizeof(zy_ac97_save_context_t), GFP_KERNEL);
	if (NULL == p_saved_memory) {
				return -ENOMEM;
	}
	memset(p_saved_memory, 0, sizeof(ZY_9713_CONTEXT_SAVE_T) +
			    sizeof(zy_ac97_save_context_t));

	p_zy_ctxt->acodec_id = (zy_acodec_device_id_t) (WM_9713_ID);
	/*
	p_zy_ctxt->pMfpRegBase = (unsigned long) (MFP_BASE);
	p_zy_ctxt->pMfpRmDb = ZY_MFP_RM_DATABASE;
	p_zy_ctxt->p_ost_regs = OST_BASE;
	*/
	p_zy_ctxt->p_voice_reg = NULL;
	p_zy_ctxt->p_hifi_reg = (void *) (&POCR);
	p_zy_ctxt->p_ctrl_reg = (void *) (&POCR);
	p_zy_ctxt->u_max_read_write_time_out_ms = ZY_AC97_RW_TIMEOUT_DEF;
	p_zy_ctxt->u_max_setup_time_out_ms = ZY_AC97_SETUP_TIMEOUT_DEF;
	p_zy_ctxt->p_save_memory = p_saved_memory;
	p_zy_ctxt->p_zy_scenario = p_zy_scenario;
#ifdef CONFIG_AC97_EXTCLK
	enable_oscc_pout();
#endif
	pxa_set_cken(CKEN_AC97, 1);
#ifdef DEBUG_ALSA_ZY
	debug_pac97ctxt = p_zy_ctxt;
	misc_register(&audio_dev);
#endif

#elif
	/* different platform code here */
#endif

	(*p_p_zy_ctxt) = p_zy_ctxt;

	return 0;
}

EXPORT_SYMBOL(alsa_prepare_for_zy);

void alsa_zy_codec_put(p_acodec_context_t p_acodectxt)
{
#ifdef CONFIG_MACH_ZYLONITE
	zy_acodec_deinit(p_acodectxt);
	pxa_set_cken(CKEN_AC97, 0);
#ifdef CONFIG_AC97_EXTCLK
	disable_oscc_pout();
#endif
#elif
	/* different platform code here */
#endif
	if(p_acodectxt->p_save_memory){
		kfree(p_saved_memory);
	}
	if(p_acodectxt->p_zy_scenario){
		kfree(p_zy_scenario);
	}
}

EXPORT_SYMBOL(alsa_zy_codec_put);

#ifdef CONFIG_PM
extern struct snd_pcm *p_hifi_zy_pcm;
extern struct snd_pcm *p_voice_zy_pcm;
extern int  snd_pcm_suspend_all(struct snd_pcm *pcm);
extern void voice_pcm_suspend(void);
extern void voice_pcm_resume(void);
int audio_codec_zy_do_suspend(struct snd_card *card, unsigned int state, p_acodec_context_t g_acodec_context)
{
#ifdef CONFIG_MACH_ZYLONITE
	#ifdef CONFIG_SND_ZYLONITE
		if(card){
			if (card->power_state != SNDRV_CTL_POWER_D3cold) {
				if(p_hifi_zy_pcm) {
					snd_pcm_suspend_all(p_hifi_zy_pcm);
				}
				if(p_voice_zy_pcm) {
					snd_pcm_suspend_all(p_voice_zy_pcm);
				}
				voice_pcm_suspend();
				zy_acodec_suspend(g_acodec_context);
				snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);
				pxa_set_cken(CKEN_AC97, 0);
#ifdef CONFIG_AC97_EXTCLK
				disable_oscc_pout();
#endif
			}
		}
	#else /* touch only */
		zy_acodec_suspend(g_acodec_context);
		pxa_set_cken(CKEN_AC97, 0);
		pxa_set_cken(CKEN_SSP3, 0);
	#endif/* CONFIG_MACH_ZYLONITE */

#elif
	/* different platform code here */
#endif
	printk(KERN_DEBUG "alsa into suspend!\n");
	return 0;
}
EXPORT_SYMBOL(audio_codec_zy_do_suspend);

int audio_codec_zy_do_resume(struct snd_card *card, unsigned int state, p_acodec_context_t g_acodec_context)
{
#ifdef CONFIG_MACH_ZYLONITE
	printk(KERN_DEBUG "alsa begin resume!\n");
	#ifdef CONFIG_SND_ZYLONITE
		if (card && card->power_state != SNDRV_CTL_POWER_D0) {
			pxa_set_cken(CKEN_AC97, 1);
			zy_acodec_resume(g_acodec_context);
			voice_pcm_resume();
			snd_power_change_state(card, SNDRV_CTL_POWER_D0);
			printk(KERN_DEBUG "alsa resume!\n");
		}
	#else/* touch */
#ifdef CONFIG_AC97_EXTCLK
		enable_oscc_pout();
#endif
		pxa_set_cken(CKEN_AC97, 1);
		zy_acodec_resume(g_acodec_context);
		printk(KERN_DEBUG "alsa touch resume!\n");
	#endif/* CONFIG_SND_ZYLONITE */
#elif
	/* different platform code here */
#endif
	return 0;
}
EXPORT_SYMBOL(audio_codec_zy_do_resume);
#endif

int set_codec_sub_state(int client, int state)
{
	return 0;
}
EXPORT_SYMBOL(set_codec_sub_state);

int register_codec(int *client)
{
	return 0;
}
EXPORT_SYMBOL(register_codec);

int unregister_codec(int client)
{
	return 0;
}
EXPORT_SYMBOL(unregister_codec);


