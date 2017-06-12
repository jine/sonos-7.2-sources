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
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>
#include <asm/arch/pxa3xx_audio_plat.h>
#include <asm/arch/pxa-regs.h>


#ifdef CONFIG_MACH_ZYLONITE
#include <asm/arch/codec/ac97acodec.h>
#include <asm/arch/codec/wm9713.h>
#include <asm/arch/codec/acodec.h>
#include <asm/arch/gpio.h>

audio_info_t audio_info = {
	.support_pcm = AUDIO_CODEC_HIFI_PCM | AUDIO_CODEC_VOICE_PCM,
#ifdef CONFIG_BASE_BAND_AUDIO
	.support_input_control = HIFI_NEAR_IN_CONTROL|MIC1_IN_CONTROL|MIC2_IN_CONTROL|HIFI_NEAR_IN_CONTROL|FAR_IN_CONTROL,
	.support_output_control = STEREO_HEAD_SET_OUT_CONTROL|HIFI_NEAR_OUT_CONTROL|FAR_OUT_CONTROL,
#else
	.support_input_control = HIFI_NEAR_IN_CONTROL|MIC1_IN_CONTROL|MIC2_IN_CONTROL|HIFI_NEAR_IN_CONTROL,
	.support_output_control = STEREO_HEAD_SET_OUT_CONTROL|HIFI_NEAR_OUT_CONTROL,
#endif
	.support_vol_control = MIC1_GAIN | STEREO_HEAD_SET_VOL | SPEAKER_VOL,
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

/* 1 second interval */
#define LP_TIME_INTERVAL	100

#ifdef CONFIG_AC97_EXTCLK
#define	enable_extclk()		do {				\
					enable_oscc_pout();	\
				} while (0)
#define disable_extclk()	do {				\
					disable_oscc_pout();	\
				} while (0)
#else
#define enable_extclk()		do {} while (0)
#define disable_extclk()	do {} while (0)
#endif

struct codec_info_node {
	int			id;	/* node id */
	int			count;	/* reference count */
	codec_sub_state_t	state;	/* node state */
	struct list_head	list;
};

struct codec_info_t {
	int			initialized;
	codec_state_t		codec_state;
	struct timer_list	*timer;
	struct list_head	list;
	spinlock_t		codec_lock;
};
static spinlock_t codec_lock = SPIN_LOCK_UNLOCKED;
static struct codec_info_t codec_info;


extern acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context);
extern acodec_error_t zy_acodec_suspend(acodec_context_t *codec_context);
extern acodec_error_t zy_acodec_resume(acodec_context_t *codec_context);
extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);


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
#endif

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
static p_acodec_context_t p_zy_ctxt = NULL;
extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);
/**
 * alsa_prepare_for_zy - create and initialize the p_acodec_context_t
 * 				   open the clock of data link
 * @p_p_zy_ctxt: return the data structure p_acodec_context_t
 * return: 0 success ; -ENOMEM
 **/
int alsa_prepare_for_zy(p_acodec_context_t * p_p_zy_ctxt)
{
	if (p_zy_ctxt) {
		p_zy_ctxt->use_count++;
		*p_p_zy_ctxt = p_zy_ctxt;
		return 0;
	}

	p_zy_ctxt = kzalloc(sizeof(acodec_context_t), GFP_KERNEL);
	if (!p_zy_ctxt)
		return -ENOMEM;

#ifdef CONFIG_MACH_ZYLONITE
	p_saved_memory = kzalloc(sizeof(ZY_9713_CONTEXT_SAVE_T) +
				sizeof(zy_ac97_save_context_t), GFP_KERNEL);
	if (NULL == p_saved_memory) {
		if (p_zy_ctxt)
			kfree(p_zy_ctxt);
		return -ENOMEM;
	}

	p_zy_ctxt->acodec_id = (zy_acodec_device_id_t) (WM_9713_ID);
	p_zy_ctxt->use_count++;
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

extern acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context);
void alsa_zy_codec_put(p_acodec_context_t p_acodectxt)
{
#ifdef CONFIG_MACH_ZYLONITE
	zy_acodec_deinit(p_acodectxt);
	pxa_set_cken(CKEN_AC97, 0);
#ifdef CONFIG_AC97_EXTCLK
	//disable_oscc_pout();
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
extern acodec_error_t zy_acodec_suspend(acodec_context_t *codec_context);
extern acodec_error_t zy_acodec_resume(acodec_context_t *codec_context);

int audio_codec_zy_do_suspend(struct snd_card *card, unsigned int state, p_acodec_context_t g_acodec_context)
{
#ifdef CONFIG_MACH_ZYLONITE
	pr_debug("alsa begin suspend!\n");
	if(card && (card->power_state != SNDRV_CTL_POWER_D3cold)) {
		if(p_hifi_zy_pcm) {
			snd_pcm_suspend_all(p_hifi_zy_pcm);
		}
		if(p_voice_zy_pcm) {
			snd_pcm_suspend_all(p_voice_zy_pcm);
		}
		voice_pcm_suspend();
		snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);
		printk(KERN_DEBUG "alsa pcm suspend!\n");
	}
	if (!--g_acodec_context->use_count) {
		zy_acodec_suspend(g_acodec_context);
		printk(KERN_DEBUG "codec suspend!\n");
	}
#elif
	/* different platform code here */
#endif
	return 0;
}
EXPORT_SYMBOL(audio_codec_zy_do_suspend);

int audio_codec_zy_do_resume(struct snd_card *card, unsigned int state, p_acodec_context_t g_acodec_context)
{
#ifdef CONFIG_MACH_ZYLONITE
	pr_debug("alsa begin resume!\n");
	if (++g_acodec_context->use_count == 1) {
		pxa_set_cken(CKEN_AC97, 1);
		zy_acodec_resume(g_acodec_context);
		printk(KERN_DEBUG "codec resume!\n");
	}

	if (card && (card->power_state != SNDRV_CTL_POWER_D0)) {
		voice_pcm_resume();
		snd_power_change_state(card, SNDRV_CTL_POWER_D0);
		printk(KERN_DEBUG "alsa pcm resume!\n");
	}
#elif
	/* different platform code here */
#endif
	return 0;
}
EXPORT_SYMBOL(audio_codec_zy_do_resume);
#endif

static void codec_warm_reset(void)
{
        p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(p_zy_ctxt->p_ctrl_reg);
	int timeout = 50;

	p_ac97_reg->gcr |= ZY_AC97_GCR_WARM_RESET_MSK;
	while (!(p_ac97_reg->gcr & ZY_AC97_GSR_PCRDY_MSK) && timeout--) {
		udelay(10);
	}
}

/* Set codec in low power mode */
static void codec_sleep(void)
{
	zy_ac97_acodec_write(p_zy_ctxt, GPIO_PIN_CFG, 0xFFF6);
	zy_ac97_acodec_write(p_zy_ctxt, GPIO_PIN_SHARING, 0xFFF6);
	mdelay(2);
	zy_ac97_acodec_write(p_zy_ctxt, POWERDOWN_CTRL_STAT, 0x7F00);
	mdelay(1);
}

/* Wakeup codec from low power mode */
static void codec_wakeup(void)
{
	codec_warm_reset();
	zy_ac97_acodec_write(p_zy_ctxt, POWERDOWN_CTRL_STAT, 0x0000);
	mdelay(1);
	zy_ac97_acodec_write(p_zy_ctxt, GPIO_PIN_CFG, 0xFFF6);
	zy_ac97_acodec_write(p_zy_ctxt, GPIO_PIN_SHARING, 0xFFF6);
	zy_ac97_acodec_write(p_zy_ctxt, GPIO_PIN_STICKY, 0x0008);
	mdelay(2);
}

/*
 * This function is used to change state of audio codec.
 */
void set_codec_state(codec_state_t state)
{
	struct codec_info_t	*info = &codec_info;

	if (info->codec_state == CODEC_POWER_OFF) {
		switch (state) {
		case CODEC_POWER_ON:
			info->codec_state = state;
			/* init codec */
			enable_extclk();
			pxa_set_cken(CKEN_AC97, 1);
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	} else if (info->codec_state == CODEC_POWER_ON) {
		switch (state) {
		case CODEC_READY_LOWPOWER:
			/* startup timer */
			codec_info.timer->expires = jiffies + LP_TIME_INTERVAL;
			add_timer(codec_info.timer);
			pr_debug("started timer\n");
			info->codec_state = state;
			break;
		case CODEC_POWER_ON:
			break;
		case CODEC_POWER_OFF:
			/* Only state will be set */
			info->codec_state = state;
			pxa_set_cken(CKEN_AC97, 0);
			disable_extclk();
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	} else if (info->codec_state == CODEC_READY_LOWPOWER) {
		switch (state) {
		case CODEC_POWER_ON:
			/* stop timer */
			info->codec_state = state;
			del_timer_sync(codec_info.timer);
			break;
		case CODEC_LOWPOWER:
			/* timer is already stopped */
			info->codec_state = state;
			if (golden_board) {
				codec_sleep();
				disable_extclk();
			}
			break;
		case CODEC_POWER_OFF:
			/* stop timer */
			del_timer_sync(codec_info.timer);
			/* Only state will be set */
			info->codec_state = state;
			pxa_set_cken(CKEN_AC97, 0);
			disable_extclk();
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	} else if (info->codec_state == CODEC_LOWPOWER) {
		switch (state) {
		case CODEC_POWER_ON:
			info->codec_state = state;
			if (golden_board) {
				enable_extclk();
				codec_wakeup();
			}
			break;
		case CODEC_POWER_OFF:
			/* Only state will be set */
			info->codec_state = state;
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	}
	pr_debug("%s: state is %d\n", __FUNCTION__, info->codec_state);
	return;
invalid_state:
	pr_debug("%s: current state is %d, next invalid state is %d\n",
			__FUNCTION__, info->codec_state, state);
}

static void codec_lp_timer_handler(unsigned long unused)
{
	set_codec_state(CODEC_LOWPOWER);
}

static int calc_codec_state(void)
{
	struct codec_info_node *p = NULL;
	struct list_head *q = NULL;
	int codec_state = CODEC_SUB_POWER_OFF;

	spin_lock(&codec_info.codec_lock);
	if (!list_empty(&codec_info.list)) {
		list_for_each(q, &codec_info.list) {
			p = list_entry(q, struct codec_info_node, list);
			pr_debug("%s: node %d, state %d\n", __FUNCTION__, p->id, p->state);
			if (p->state == CODEC_SUB_POWER_ON) {
				/* Even one node is POWER_ON, codec can't be shutdown */
				codec_state = CODEC_POWER_ON;
				break;
			} else if (p->state == CODEC_SUB_LOWPOWER) {
				codec_state = CODEC_READY_LOWPOWER;
			}
		}
	}
	spin_unlock(&codec_info.codec_lock);
	pr_debug("%s: codec state:%d\n", __FUNCTION__, codec_state);
	return codec_state;
}

int set_codec_sub_state(int client, int state)
{
	struct codec_info_node *p = NULL;
	struct list_head *q = NULL;
	int codec_state = CODEC_SUB_POWER_ON;
	
	pr_debug("%s entry, client:%d, state:%d\n", __FUNCTION__, client, state);
	spin_lock(&codec_info.codec_lock);
	/* find the client */
	if (!list_empty(&codec_info.list)) {
		list_for_each(q, &codec_info.list) {
			p = list_entry(q, struct codec_info_node, list);
			if (client == p->id) {
				pr_debug("Found node as id %d\n", client);
				break;
			}
		}
		if (client != p->id) {
			spin_unlock(&codec_info.codec_lock);
			printk(KERN_ERR "No such node exists\n");
			return -EIO;
		}
	} else {
		spin_unlock(&codec_info.codec_lock);
		printk(KERN_ERR "No such node exists\n");
		return -EIO;
	}

	/* When reference count isn't zero, the device is in use.
	 * It can't be set as low power mode or power off mode.
	 */
	if (state == CODEC_SUB_POWER_ON) {
		p->count++;
		p->state = state;
	} else if ((state == CODEC_SUB_LOWPOWER) || (state == CODEC_SUB_POWER_OFF)) {
		if (--p->count == 0) {
			p->state = state;
		}
	}
	codec_state = calc_codec_state();
	pr_debug("Set client %d as state %d, count:%d\n", client, codec_state, p->count);
	set_codec_state(codec_state);
	spin_unlock(&codec_lock);
	return 0;
}
EXPORT_SYMBOL(set_codec_sub_state);

/*
 * Initialize the state of codec.
 */
int init_codec_state(acodec_context_t *p_device_context)
{
	codec_info.timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (codec_info.timer == NULL) {
		printk(KERN_ERR "Can't allocate memory for codec state\n");
		return -ENOMEM;
	}

	init_timer(codec_info.timer);
	codec_info.timer->function = codec_lp_timer_handler;
	codec_info.timer->data = 0;

	INIT_LIST_HEAD(&codec_info.list);
	spin_lock_init(&codec_info.codec_lock);

	codec_info.codec_state = CODEC_POWER_OFF;
	codec_info.initialized = 1;

	return 0;
}

int register_codec(int *client)
{
	struct codec_info_node	*p = NULL;
	struct list_head *q = NULL;
	int id = 0;

	pr_debug("%s: entry\n", __FUNCTION__);
	if (codec_info.initialized == 0) {

		init_codec_state(p_zy_ctxt);
	}

	spin_lock(&codec_info.codec_lock);
	if (!list_empty(&codec_info.list)) {
		list_for_each(q, &(codec_info.list)) {
			p = list_entry(q, struct codec_info_node, list);
			if (p->id > id)
				id = p->id;
		}
	}
	id++;
	pr_debug("%s node id:%d\n", __FUNCTION__, id);

	p = (struct codec_info_node *)vmalloc(sizeof(struct codec_info_node));
	if (p == NULL) {
		printk(KERN_ERR "%s: Can't allocate memory\n", __FUNCTION__);
		return -ENOMEM;
	}
	p->count = 0;
	p->id = id;

	list_add(&(p->list), &(codec_info.list));
	spin_unlock(&codec_info.codec_lock);

	*client = id;
	return 0;
}
EXPORT_SYMBOL(register_codec);

int unregister_codec(int client)
{
	struct codec_info_node	*p = NULL;
	struct list_head *q = NULL;

	pr_debug("%s: entry\n", __FUNCTION__);
	spin_lock(&codec_info.codec_lock);
	if (list_empty(&codec_info.list)) {
		printk(KERN_ERR "No node is registered for codec\n");
		return -EIO;
	}
	list_for_each(q, &codec_info.list) {
		p = container_of(q, struct codec_info_node, list);
		if (client == p->id) {
			list_del(q);
			vfree(p);
		}
	}
	spin_unlock(&codec_info.codec_lock);
	return 0;
}
EXPORT_SYMBOL(unregister_codec);


