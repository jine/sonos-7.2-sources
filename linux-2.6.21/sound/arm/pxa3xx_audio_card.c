/*
 *  sound/arm/pxa3xx_audio_card.c.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/ipmc.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>

#include <asm/arch/codec/acodec.h>
#include <asm/arch/pxa3xx_audio_plat.h>
#include "pxa3xx_audio_pcm.h"
#include "pxa3xx_audio_control.h"

#define ALSA_ZY_CARD_DEBUG
#undef ALSA_ZY_CARD_DEBUG

#ifdef ALSA_ZY_CARD_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG format "\n",##arg)
#else
#define dbg(format, arg...)
#endif

p_acodec_context_t g_acodec_context = NULL;

extern acodec_error_t zy_acodec_init(acodec_context_t *codec_context, int hw_init);
extern acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context);

#ifdef CONFIG_PM
extern int audio_codec_zy_do_suspend(struct snd_card *card, unsigned int state,
					p_acodec_context_t g_acodec_context);
extern int audio_codec_zy_do_resume(struct snd_card *card, unsigned int state,
					p_acodec_context_t g_acodec_context);
static int audio_codec_zy_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	int ret;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	ret = audio_codec_zy_do_suspend(card, SNDRV_CTL_POWER_D3cold, g_acodec_context);
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_OFF);
	return ret;
}

static int audio_codec_zy_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	int ret;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	ret = audio_codec_zy_do_resume(card, SNDRV_CTL_POWER_D0,
					g_acodec_context);
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return ret;
}
#else
#define audio_codec_zy_suspend	NULL
#define audio_codec_zy_resume	NULL
#endif

extern int alsa_prepare_for_zy(p_acodec_context_t * p_p_zy_ctxt);
extern acodec_error_t zy_acodec_init(acodec_context_t *codec_context,
					int hw_init);
extern void set_card_shortname(struct snd_card *card,
				p_acodec_context_t g_acodec_context);
extern acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context);
extern void alsa_zy_codec_put(p_acodec_context_t p_acodectxt);

int codec_client;
static int audio_codec_zy_probe(struct platform_device *pdev)
{
	struct snd_card *card = NULL;
	int ret;
	acodec_error_t status;

	ret = -ENOMEM;
	card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
				THIS_MODULE, 0);
	if (!card)
		goto err;

	card->dev = &pdev->dev;
	strncpy(card->driver, pdev->name, sizeof(card->driver));

	ret = alsa_prepare_for_zy(&g_acodec_context);
	if (ret)
		goto err;

	register_codec(&codec_client);
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	status = zy_acodec_init(g_acodec_context, 1);
	if (ACODEC_SUCCESS != status) {
		ret = -EIO;
		goto err;
	}
#ifdef CONFIG_MACH_ZYLONITE
	/* power down the units of the acodec, sleep the acodec, zy_acodec_init()
	 * will open all the units' power of the codec while ALSA need all the
	 * codec units power down and the codec should sleep if it can.
	 * So on the zylonite platform we call below function to power down and
	 * sleep wm9713 codec.
	 */
	g_acodec_context->codec_specific_dinit(g_acodec_context);
#endif

	ret = audio_codec_pcm_new (card);
	if (ret) {
		goto err;
	}

	ret = audio_codec_control_new(card);
	if (ret) {
		goto err;
	}

	set_card_shortname(card, g_acodec_context);
	snprintf(card->longname, sizeof(card->longname),
		 "%s (%s)", pdev->name, card->mixername);

	ret = snd_card_register(card);
	if (ret == 0) {
		platform_set_drvdata(pdev, card);
	}

	msleep(10);
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return 0;
err:
	if (g_acodec_context) {
		zy_acodec_deinit(g_acodec_context);
		kfree(g_acodec_context);
		g_acodec_context = NULL;
	}

	if (card)
		snd_card_free(card);
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return ret;
}

static int audio_codec_zy_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	if (g_acodec_context) {
		alsa_zy_codec_put(g_acodec_context);
		kfree(g_acodec_context);
		g_acodec_context = NULL;
	}

	if (card) {
		snd_card_free(card);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static struct platform_driver audio_codec_zy_driver = {
	.driver = {
		.name 	= "pxa2xx-ac97",
	},
	.probe 	= 	audio_codec_zy_probe,
	.remove = 	audio_codec_zy_remove,
	.suspend= 	audio_codec_zy_suspend,
	.resume = 	audio_codec_zy_resume,
};

static int __init audio_codec_zy_init(void)
{
	return platform_driver_register(&audio_codec_zy_driver);
}

static void __exit audio_code_zy_exit(void)
{
	platform_driver_unregister(&audio_codec_zy_driver);
}
module_init(audio_codec_zy_init);
module_exit(audio_code_zy_exit);

EXPORT_SYMBOL(g_acodec_context);

MODULE_AUTHOR("jingqing.xu@intel.com ");
MODULE_DESCRIPTION("zylonite audio codec driver on ALSA");
MODULE_LICENSE("GPL");


