/*
 * linux/sound/arm/lt_audio_card.c.
 *  
 *  Author:	fengwei.yin@marvell.com
 *  Created:	Nov 27, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define	DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/ipmc.h>
#include <asm/arch/codec/lt_micco_acodec.h>

#include "lt_audio.h"
#include "lt_audio_pcm.h"
#include "pxa3xx_audio_control.h"

int codec_client;
int golden_muted = 0;

#ifdef	CONFIG_PROC_FS
#define	MICCO_PROC_FILE	"driver/micco_acodec"
static struct proc_dir_entry *acodec_proc_file;

extern void micco_dump_codec(void);
static ssize_t micco_acodec_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	micco_dump_codec();
	return 0;
}

static ssize_t micco_acodec_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	return 0;
}

static struct file_operations micco_acodec_proc_ops = {
	.read = micco_acodec_proc_read,
	.write = micco_acodec_proc_write,
};

static void create_micco_proc_files(void)
{
	acodec_proc_file = create_proc_entry(MICCO_PROC_FILE, 0644, NULL);
	if (acodec_proc_file) {
		acodec_proc_file->owner = THIS_MODULE;
		acodec_proc_file->proc_fops = &micco_acodec_proc_ops;
	}
}

static void remove_micco_proc_files(void)
{
	remove_proc_entry(MICCO_PROC_FILE, &proc_root);
}

#endif

p_acodec_context_t g_acodec_context = NULL;
EXPORT_SYMBOL(g_acodec_context);

/* The sound suspend just take care of the sound related. Needn't touch
 * the Micco codec. Micco driver will cover the Micco codec suspend/resume.
 */
#ifdef CONFIG_PM
extern int audio_codec_lt_do_suspend(struct snd_card *,
				     unsigned int, acodec_context_t *);
extern int audio_codec_lt_do_resume(struct snd_card *,
				    unsigned int, acodec_context_t *);
static int audio_codec_lt_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	struct snd_card *card = platform_get_drvdata(pdev);

	if (card) 
		ret = audio_codec_lt_do_suspend(card,
			SNDRV_CTL_POWER_D3cold, g_acodec_context);
	return ret;
}

static int audio_codec_lt_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_card *card = platform_get_drvdata(pdev);

	if (card)
		ret = audio_codec_lt_do_resume(card,
			SNDRV_CTL_POWER_D0, g_acodec_context);
	return ret;
}
#else
#define audio_codec_lt_suspend	NULL
#define audio_codec_lt_resume	NULL
#endif

extern int alsa_prepare_for_lt(acodec_context_t **acodec_context);
extern int lt_acodec_init(acodec_context_t *codec_context, int hw_init);
extern void set_card_shortname(struct snd_card *, p_acodec_context_t);
extern int lt_acodec_deinit(acodec_context_t *codec_context);
extern void alsa_lt_codec_put(acodec_context_t *acodec_context);

static int __devinit audio_codec_lt_probe(struct platform_device *pdev)
{
	struct snd_card *card = NULL;
	int ret;
	acodec_error_t status;

	pr_debug("enter");

	card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1, THIS_MODULE, 0);
	if (!card) {
		printk(KERN_ERR "snd_card_new failed!");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	strncpy(card->driver, pdev->name, sizeof(card->driver));

	ret = alsa_prepare_for_lt(&g_acodec_context);
	if (ret) {
		printk(KERN_ERR "alsa_prepare_for_lt failed!");
		goto err;
	}
	
	status = lt_acodec_init(g_acodec_context, 1);
	if (ACODEC_SUCCESS != status) {
		printk(KERN_ERR "lt acodec init failed!");
		ret = -EIO;
		goto err;
	}

	/*
	 * If no one use the acodec, power down the acodec to save power.
	 */
	g_acodec_context->codec_specific_dinit(g_acodec_context);

	ret = audio_codec_pcm_new (card);
	if (ret) {
		printk(KERN_ERR "%s: lt acodec pcm new failed!\n", __func__);
		goto err;
	}

	ret = audio_codec_control_new(card); /* clear functions ?? -- by yfw */
	if (ret) {
		printk(KERN_ERR "%s: lt acodec control new failed!\n", __func__);
		goto err;
	}

	set_card_shortname(card, g_acodec_context);
	snprintf(card->longname, sizeof(card->longname),
		 "%s (%s)", pdev->name, card->mixername);

	ret = snd_card_register(card);
	if (ret < 0) {
		printk(KERN_ERR "%s: lt snd_card_register failed!\n", __func__);
		goto err;
	}
	platform_set_drvdata(pdev, card);

#ifdef	CONFIG_PROC_FS
	create_micco_proc_files();
#endif

	return 0;
err:
	if (g_acodec_context) {
		lt_acodec_deinit(g_acodec_context);
		alsa_lt_codec_put(g_acodec_context);
		kfree(g_acodec_context);
		g_acodec_context = NULL;
	}

	if (card)
		snd_card_free(card);

	return ret;
}

static int __devexit audio_codec_lt_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

#ifdef	CONFIG_PROC_FS
	remove_micco_proc_files();
#endif

	if (g_acodec_context) {
		lt_acodec_deinit(g_acodec_context);
		alsa_lt_codec_put(g_acodec_context);
		kfree(g_acodec_context);
		g_acodec_context = NULL;
	}
	
	if (card) {
		snd_card_free(card);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static struct platform_driver audio_codec_lt_driver = {
	.driver	= {
		.name 	= 	"lt_micco_codec",
	},
	.probe 	= 	audio_codec_lt_probe,
	.remove = 	__devexit_p(audio_codec_lt_remove),
	.suspend= 	audio_codec_lt_suspend,
	.resume = 	audio_codec_lt_resume,
};

static int __init audio_codec_lt_init(void)
{
	pr_debug("enter");
	return platform_driver_register(&audio_codec_lt_driver);
	pr_debug("exit");
}

static void __exit audio_code_lt_exit(void)
{
	platform_driver_unregister(&audio_codec_lt_driver);
}

module_init(audio_codec_lt_init);
module_exit(audio_code_lt_exit);


MODULE_AUTHOR("fengwei.yin@marvell.com");
MODULE_DESCRIPTION("Audio codec driver for ALSA");
MODULE_LICENSE("GPL");

