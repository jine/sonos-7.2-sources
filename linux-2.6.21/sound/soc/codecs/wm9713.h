/*
 * linux/sound/soc/codecs/wm9713.h
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
#ifndef _WM9713_H
#define _WM9713_H

/* clock inputs */
#define WM9713_CLKA_PIN			0
#define WM9713_CLKB_PIN			1

/* clock divider ID's */
#define WM9713_PCMCLK_DIV		0
#define WM9713_CLKA_MULT		1
#define WM9713_CLKB_MULT		2
#define WM9713_HIFI_DIV			3
#define WM9713_PCMBCLK_DIV		4

/* PCM clk div */
#define WM9713_PCMDIV(x)	((x - 1) << 8)

/* HiFi Div */
#define WM9713_HIFIDIV(x)	((x - 1) << 12)

/* MCLK clock mulitipliers */
#define WM9713_CLKA_X1		(0 << 1)
#define WM9713_CLKA_X2		(1 << 1)
#define WM9713_CLKB_X1		(0 << 2)
#define WM9713_CLKB_X2		(1 << 2)

/* MCLK clock MUX */
#define WM9713_CLK_MUX_A		(0 << 0)
#define WM9713_CLK_MUX_B		(1 << 0)

/* Voice DAI BCLK divider */
#define WM9713_PCMBCLK_DIV_1	(0 << 9)
#define WM9713_PCMBCLK_DIV_2	(1 << 9)
#define WM9713_PCMBCLK_DIV_4	(2 << 9)
#define WM9713_PCMBCLK_DIV_8	(3 << 9)
#define WM9713_PCMBCLK_DIV_16	(4 << 9)

#define WM9713_DAI_AC97_HIFI	0
#define WM9713_DAI_AC97_AUX		1
#define WM9713_DAI_PCM_VOICE	2

extern struct snd_soc_codec_device soc_codec_dev_wm9713;
extern struct snd_soc_codec_dai wm9713_dai[3];

int wm9713_reset(struct snd_soc_codec *codec,  int try_warm);

#endif
