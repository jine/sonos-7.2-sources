/*
 * linux/sound/soc/pxa/pxa3xx-ssp.c
 * Base on pxa2xx-ssp.c
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>
#include <asm/arch/ssp.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#define PXA_SSP_DEBUG 0
#if PXA_SSP_DEBUG
#define DBG(fmt, arg...) printk("pxa3xx-ssp: " fmt "\n", ## arg)
#else
#define DBG(fmt, arg...) do {} while (0)
#endif

/*
 * The following should be defined in pxa-regs.h
 */
#define SSCR0_ACS	(1 << 30) /* Audio Clock Select */
#define SSACD_SCDB	(1 << 3) /* SSPSYSCLK Divider Bypass */
#define SSACD_SCDX8	(1 << 7) /* SSPSYSCLK Divide by 8 */
#ifndef SSACD_ACPS
#define SSACD_ACPS(x)	(x << 4) /* Audio clock PLL select */
#endif
#ifndef SSACD_ACDS
#define SSACD_ACDS(x)	(x << 0) /* Audio clock divider select */
#endif
#ifndef SSCR0_FPCKE
#define SSCR0_FPCKE	(1<<29)
#endif

/*
 * SSP audio private data
 */
struct ssp_priv {
	unsigned int sysclk;
	int dai_fmt;
};

static struct ssp_priv ssp_clk[4];
static struct ssp_dev ssp[4];
#ifdef CONFIG_PM
static struct ssp_state ssp_state[4];
#endif

static struct pxa3xx_pcm_dma_params pxa3xx_ssp1_pcm_out = {
	.name			= "SSP1 PCM out",
	.dev_addr		= __PREG(SSDR_P1),
	.drcmr			= &DRCMRTXSSDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ssp1_pcm_in = {
	.name			= "SSP1 PCM in",
	.dev_addr		= __PREG(SSDR_P1),
	.drcmr			= &DRCMRRXSSDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ssp2_pcm_out = {
	.name			= "SSP2 PCM out",
	.dev_addr		= __PREG(SSDR_P2),
	.drcmr			= &DRCMRTXSS2DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ssp2_pcm_in = {
	.name			= "SSP2 PCM in",
	.dev_addr		= __PREG(SSDR_P2),
	.drcmr			= &DRCMRRXSS2DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

#ifdef CONFIG_MACH_ZYLONITE
static struct pxa3xx_pcm_dma_params pxa3xx_ssp3_pcm_out = {
	.name			= "SSP3 PCM out",
	.dev_addr		= __PREG(SSDR_P3),
	.drcmr			= &DRCMRTXSS3DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ssp3_pcm_in = {
	.name			= "SSP3 PCM Stereo in",
	.dev_addr		= __PREG(SSDR_P3),
	.drcmr			= &DRCMRRXSS3DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};
#else
static struct pxa3xx_pcm_dma_params pxa3xx_ssp3_pcm_out = {
	.name			= "SSP3 PCM out",
	.dev_addr		= __PREG(SSDR_P3),
	.drcmr			= &DRCMRTXSS3DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ssp3_pcm_in = {
	.name			= "SSP3 PCM Stereo in",
	.dev_addr		= __PREG(SSDR_P3),
	.drcmr			= &DRCMRRXSS3DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH4,
};
#endif


static struct pxa3xx_pcm_dma_params pxa3xx_ssp4_pcm_out = {
	.name			= "SSP3 PCM out",
	.dev_addr		= __PREG(SSDR_P4),
	.drcmr			= &DRCMR3,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ssp4_pcm_in = {
	.name			= "SSP3 PCM Stereo in",
	.dev_addr		= __PREG(SSDR_P4),
	.drcmr			= &DRCMR2,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};


static struct pxa3xx_pcm_dma_params *ssp_dma_params[4][2] = {
	{&pxa3xx_ssp1_pcm_out, &pxa3xx_ssp1_pcm_in,},
	{&pxa3xx_ssp2_pcm_out, &pxa3xx_ssp2_pcm_in,},
	{&pxa3xx_ssp3_pcm_out, &pxa3xx_ssp3_pcm_in,},
	{&pxa3xx_ssp4_pcm_out, &pxa3xx_ssp4_pcm_in,},
};

static int pxa3xx_ssp_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;

	if (!cpu_dai->active) {
		ret = ssp_init (&ssp[cpu_dai->id], cpu_dai->id + 1,
			SSP_NO_IRQ);
		if (ret < 0)
			return ret;
		ssp_disable(&ssp[cpu_dai->id]);
	}
	return ret;
}

static void pxa3xx_ssp_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	if (!cpu_dai->active) {
		ssp_disable(&ssp[cpu_dai->id]);
		ssp_exit(&ssp[cpu_dai->id]);
	}
}

static int cken[4] = {CKEN_SSP1, CKEN_SSP2, CKEN_SSP3, CKEN_SSP4};

#ifdef CONFIG_PM

static int pxa3xx_ssp_suspend(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	if (!dai->active)
		return 0;

	ssp_save_state(&ssp[dai->id], &ssp_state[dai->id]);
	pxa_set_cken(cken[dai->id], 0);
	return 0;
}

static int pxa3xx_ssp_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	if (!dai->active)
		return 0;

	pxa_set_cken(cken[dai->id], 1);
	ssp_restore_state(&ssp[dai->id], &ssp_state[dai->id]);
	ssp_enable(&ssp[dai->id]);

	return 0;
}

#else
#define pxa3xx_ssp_suspend	NULL
#define pxa3xx_ssp_resume	NULL
#endif

/*
 * Set the SSP ports SYSCLK.
 */
static int pxa3xx_ssp_set_dai_sysclk(struct snd_soc_cpu_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	int port = cpu_dai->id + 1;
	u32 sscr0 = SSCR0_P(port) &
		~(SSCR0_ECS |  SSCR0_NCS | SSCR0_MOD | SSCR0_ACS);

	switch (clk_id) {
	case PXA3XX_SSP_CLK_PLL:
		ssp_clk[cpu_dai->id].sysclk = 13000000;
		break;
	case PXA3XX_SSP_CLK_EXT:
		ssp_clk[cpu_dai->id].sysclk = freq;
		sscr0 |= SSCR0_ECS;
		break;
	case PXA3XX_SSP_CLK_NET:
		ssp_clk[cpu_dai->id].sysclk = freq;
		sscr0 |= (SSCR0_NCS | SSCR0_MOD);
		break;
	case PXA3XX_SSP_CLK_AUDIO:
		ssp_clk[cpu_dai->id].sysclk = 0;
		SSCR0_P(port) |= SSCR0_SerClkDiv(1);
		sscr0 |= SSCR0_ACS;
		break;
	default:
		return -ENODEV;
	}

	/* the SSP CKEN clock must be disabled when changing SSP clock mode */
	pxa_set_cken(cken[cpu_dai->id], 0);
	SSCR0_P(port) |= sscr0;
	pxa_set_cken(cken[cpu_dai->id], 1);
	return 0;
}

/*
 * Set the SSP clock dividers.
 */
static int pxa3xx_ssp_set_dai_clkdiv(struct snd_soc_cpu_dai *cpu_dai,
	int div_id, int div)
{
	int port = cpu_dai->id + 1;

	switch (div_id) {
	case PXA3XX_SSP_AUDIO_DIV_ACDS:
		SSACD_P(port) &= ~ 0x7;
		SSACD_P(port) |= SSACD_ACDS(div);
		break;
	case PXA3XX_SSP_AUDIO_DIV_SCDB:
		SSACD_P(port) &= ~(SSACD_SCDB | SSACD_SCDX8);
		switch (div) {
		case PXA3XX_SSP_CLK_SCDB_1:
			SSACD_P(port) |= SSACD_SCDB;
			break;
		case PXA3XX_SSP_CLK_SCDB_4:
			break;
		case PXA3XX_SSP_CLK_SCDB_8:
			SSACD_P(port) |= SSACD_SCDX8;
			break;
		default:
			return -EINVAL;
		}
		break;
	case PXA3XX_SSP_DIV_SCR:
		SSCR0_P(port) &= ~SSCR0_SCR;
		SSCR0_P(port) |= SSCR0_SerClkDiv(div);
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

/*
 * Configure the PLL frequency
 */
static int pxa3xx_ssp_set_dai_pll(struct snd_soc_cpu_dai *cpu_dai,
	int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	int port = cpu_dai->id + 1;

	SSACD_P(port) &= ~0x70;
	SSACDD_P(port) = 0;
	switch (freq_out) {
	case 5622000:
		break;
	case 11345000:
		SSACD_P(port) |= (0x1 << 4);
		break;
	case 12235000:
		SSACD_P(port) |= (0x2 << 4);
		break;
	case 14857000:
		SSACD_P(port) |= (0x3 << 4);
		break;
	case 32842000:
		SSACD_P(port) |= (0x4 << 4);
		break;
	case 48000000:
		SSACD_P(port) |= (0x5 << 4);
		break;
	case 12288000:
		SSACD_P(port) |= (0x6 << 4);
		SSACDD_P(port) = (1625 << 16) | 64;
		break;
	case 11289600:
		SSACD_P(port) |= (0x6 << 4);
		SSACDD_P(port) = (1769 << 16) | 64;
		break;
	case 4096000:
		SSACD_P(port) |= (0x6 << 4);
		SSACDD_P(port) = (4875 << 16) | 64;
		break;
	}
	return 0;
}

/*
 * Set the active slots in TDM/Network mode
 */
static int pxa3xx_ssp_set_dai_tdm_slot(struct snd_soc_cpu_dai *cpu_dai,
	unsigned int mask, int slots)
{
	int port = cpu_dai->id + 1;

	SSCR0_P(port) &= ~SSCR0_SlotsPerFrm(7);

	/* set number of active slots */
	SSCR0_P(port) |= SSCR0_SlotsPerFrm(slots);

	/* set active slot mask */
	SSTSA_P(port) = mask;
	SSRSA_P(port) = mask;
	return 0;
}

/*
 * Tristate the SSP DAI lines
 */
static int pxa3xx_ssp_set_dai_tristate(struct snd_soc_cpu_dai *cpu_dai,
	int tristate)
{
	int port = cpu_dai->id + 1;

	if (tristate)
		SSCR1_P(port) |= SSCR1_TTE;
	else
		SSCR1_P(port) &= ~SSCR1_TTE;

	return 0;
}

/*
 * Set up the SSP DAI format.
 * The SSP Port must be inactive before calling this function as the
 * physical interface format is changed.
 */
static int pxa3xx_ssp_set_dai_fmt(struct snd_soc_cpu_dai *cpu_dai,
		unsigned int fmt)
{
	int port = cpu_dai->id + 1;
	int dai_fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	if (SSCR0_P(port) & SSCR0_SSE)
		return 0;

	ssp_clk[cpu_dai->id].dai_fmt = dai_fmt;

	/*
	 * reset port settings
	 * PXA3xx docs say to use RxThresh = 8 and TxThresh = 7 with
	 * DMA bursts of 32
	 */
	SSCR0_P(port) = 0;
	SSCR1_P(port) = SSCR1_RxTresh(8) | SSCR1_TxTresh(7);
	SSPSP_P(port) = 0;

	switch (dai_fmt) {
	case SND_SOC_DAIFMT_I2S:
		SSCR0_P(port) = SSCR0_MOD | SSCR0_PSP;
		SSCR1_P(port) |= SSCR1_RWOT | SSCR1_TRAIL;
	
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			SSPSP_P(port) |= SSPSP_FSRT;
			break;
		case SND_SOC_DAIFMT_NB_IF:
			SSPSP_P(port) |= SSPSP_SFRMP | SSPSP_FSRT;
			break;
		case SND_SOC_DAIFMT_IB_IF:
			SSPSP_P(port) |= SSPSP_SFRMP;
			break;
		default:
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		SSCR0_P(port) = SSCR0_MOD | SSCR0_PSP;
		SSCR1_P(port) |= SSCR1_TRAIL | SSCR1_RWOT;
		if (dai_fmt == SND_SOC_DAIFMT_DSP_A)
			SSPSP_P(port) = SSPSP_FSRT;

		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			SSPSP_P(port) |= SSPSP_SFRMP;
			break;
		case SND_SOC_DAIFMT_IB_IF:
			break;
		default:
			return -EINVAL;
		}

		break;
	default:
		return -EINVAL;
	}

	switch(fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		SSCR1_P(port) |= SSCR1_SCLKDIR | SSCR1_SFRMDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		SSCR1_P(port) |= SSCR1_SCLKDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Set the SSP audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa3xx_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int dma = 0;
	int port = cpu_dai->id + 1;

	/* select correct DMA params */
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		dma = 1; /* capture DMA offset is 1 */
	cpu_dai->dma_data = ssp_dma_params[cpu_dai->id][dma];

	/* we can only change the settings if the port is not in use */
	if (SSCR0_P(port) & SSCR0_SSE)
		return 0;

	/* bit size */
	switch(params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		SSCR0_P(port) |= SSCR0_FPCKE | SSCR0_DataSize(16);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		SSCR0_P(port) |= SSCR0_EDSS | SSCR0_DataSize(16);
		break;
	default:
		return -EINVAL;
	}

	if (ssp_clk[cpu_dai->id].dai_fmt == SND_SOC_DAIFMT_I2S) {
		int sfrmwidth =
			snd_pcm_format_physical_width(params_format(params));
		SSPSP_P(port) |= SSPSP_SFRMWDTH(sfrmwidth);
	}

	DBG("SSCR0 %x SSCR1 %x SSTO %x SSPSP %x SSSR %x",
		SSCR0_P(port), SSCR1_P(port), SSTO_P(port),
		SSPSP_P(port), SSSR_P(port));
	DBG("SSACD %x SSACDD %x SSTSA %x SSRSA %x",
		SSACD_P(port), SSACDD_P(port), SSTSA_P(port),
		SSRSA_P(port));

	return 0;
}

static int pxa3xx_ssp_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	int port = cpu_dai->id + 1;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		ssp_enable(&ssp[cpu_dai->id]);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			SSCR1_P(port) |= SSCR1_TSRE;
		else
			SSCR1_P(port) |= SSCR1_RSRE;
		SSSR_P(port) |= SSSR_P(port);
		break;
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			SSCR1_P(port) |= SSCR1_TSRE;
		else
			SSCR1_P(port) |= SSCR1_RSRE;
		ssp_enable(&ssp[cpu_dai->id]);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			SSCR1_P(port) &= ~SSCR1_TSRE;
		else
			SSCR1_P(port) &= ~SSCR1_RSRE;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ssp_disable(&ssp[cpu_dai->id]);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			SSCR1_P(port) &= ~SSCR1_TSRE;
		else
			SSCR1_P(port) &= ~SSCR1_RSRE;
		break;

	default:
		ret = -EINVAL;
	}
	DBG("trig cmd %d", cmd);
	DBG("SSCR0 %x SSCR1 %x SSTO %x SSPSP %x SSSR %x",
		SSCR0_P(port), SSCR1_P(port), SSTO_P(port),
		SSPSP_P(port), SSSR_P(port));
	DBG("SSACD %x SSACDD %x SSTSA %x SSRSA %x",
		SSACD_P(port), SSACDD_P(port), SSTSA_P(port),
		SSRSA_P(port));
	return ret;
}


#define PXA3XX_SSP_RATES 0xffffffff

#define PXA3XX_SSP_FORMATS 0xffffffff


struct snd_soc_cpu_dai pxa3xx_ssp_dai[] = {
	{	.name = "pxa3xx-ssp1",
		.id = 0,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.ops = {
			.startup = pxa3xx_ssp_startup,
			.shutdown = pxa3xx_ssp_shutdown,
			.trigger = pxa3xx_ssp_trigger,
			.hw_params = pxa3xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa3xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa3xx_ssp_set_dai_clkdiv,
			.set_pll = pxa3xx_ssp_set_dai_pll,
			.set_fmt = pxa3xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa3xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa3xx_ssp_set_dai_tristate,
		},
	},
	{	.name = "pxa3xx-ssp2",
		.id = 1,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.ops = {
			.startup = pxa3xx_ssp_startup,
			.shutdown = pxa3xx_ssp_shutdown,
			.trigger = pxa3xx_ssp_trigger,
			.hw_params = pxa3xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa3xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa3xx_ssp_set_dai_clkdiv,
			.set_pll = pxa3xx_ssp_set_dai_pll,
			.set_fmt = pxa3xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa3xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa3xx_ssp_set_dai_tristate,
		},
	},
	{	.name = "pxa3xx-ssp3",
		.id = 2,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.ops = {
			.startup = pxa3xx_ssp_startup,
			.shutdown = pxa3xx_ssp_shutdown,
			.trigger = pxa3xx_ssp_trigger,
			.hw_params = pxa3xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa3xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa3xx_ssp_set_dai_clkdiv,
			.set_pll = pxa3xx_ssp_set_dai_pll,
			.set_fmt = pxa3xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa3xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa3xx_ssp_set_dai_tristate,
		},
	},
	{	.name = "pxa3xx-ssp4",
		.id = 3,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa3xx_ssp_suspend,
		.resume = pxa3xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES,
			.formats = PXA3XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA3XX_SSP_RATES|SNDRV_PCM_RATE_32000,
			.formats = PXA3XX_SSP_FORMATS},
		.ops = {
			.startup = pxa3xx_ssp_startup,
			.shutdown = pxa3xx_ssp_shutdown,
			.trigger = pxa3xx_ssp_trigger,
			.hw_params = pxa3xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa3xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa3xx_ssp_set_dai_clkdiv,
			.set_pll = pxa3xx_ssp_set_dai_pll,
			.set_fmt = pxa3xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa3xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa3xx_ssp_set_dai_tristate,
		},
	},
	
};
EXPORT_SYMBOL_GPL(pxa3xx_ssp_dai);

/* Module information */
MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("pxa3xx SSP/PCM SoC Interface");
MODULE_LICENSE("GPL");
