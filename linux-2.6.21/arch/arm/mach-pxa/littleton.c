/*
 *  linux/arch/arm/mach-pxa/littleton.c
 *
 *  Support for the Marvell Littleton Development Platform.
 *
 *  Author:	Jason Chagas (largely modified code)
 *  Created:	Nov 20, 2006
 *  Copyright:	(C) Copyright 2006 Marvell International Ltd.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fb.h>
#include <linux/root_dev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/irda.h>
#include <asm/arch/ssp.h>
#include <asm/arch/pxa3xx_pmic.h>
#ifdef CONFIG_KEYBOARD_PXA3xx
#include <asm/arch/pxa3xx_keypad.h>
#endif
#include "generic.h"

#ifdef CONFIG_KEYBOARD_PXA3xx
static unsigned int littleton_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_1), KEY(0, 1, KEY_4), KEY(0, 2, KEY_7),
	KEY(1, 0, KEY_2), KEY(1, 1, KEY_5), KEY(1, 2, KEY_8), KEY(1, 3, KEY_0),
	KEY(2, 0, KEY_3), KEY(2, 1, KEY_6), KEY(2, 2, KEY_9),
	KEY(0, 3, KEY_KPASTERISK),
	KEY(2, 3, KEY_KPDOT),
	KEY(3, 0, KEY_F22),     /* soft1 */
	KEY(3, 1, KEY_F23),     /* soft2 */
	KEY(3, 2, KEY_HOME),
	KEY(3, 3, KEY_BACK),
	KEY(4, 0, KEY_SEND),
	KEY(4, 1, KEY_END),
	KEY(4, 2, KEY_VOLUMEUP),
	KEY(4, 3, KEY_VOLUMEDOWN),
	KEY(5, 0, KEY_UP),
	KEY(5, 1, KEY_DOWN),
	KEY(5, 2, KEY_LEFT),
	KEY(5, 3, KEY_RIGHT),
	KEY(5, 4, KEY_ENTER),
};

static struct pxa3xx_keypad_platform_data littleton_keypad_info = {
	.enable_repeat_key	= 1,
	.enable_matrix_key	= 1,
	.enable_direct_key	= 1,
	.enable_rotary_key	= 1,
	.matrix_key_debounce	= 30,
	.direct_key_debounce	= 30,
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.direct_key_num		= 2,

	.matrix_key_map		= littleton_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(littleton_matrix_key_map),

	.direct_key_map		= NULL,
	.direct_key_map_size	= -1,

	.rotary_up_key		= KEY_UP,
	.rotary_down_key	= KEY_DOWN,
};
#endif

#define USE_SSP_DRIVER_CODE

int golden_board = 0;
EXPORT_SYMBOL(golden_board);

static int __init golden(char *str)
{
	golden_board = 1;
	return 0;
}
__setup("golden", golden);

int pwri2c_flag = 0;
EXPORT_SYMBOL(pwri2c_flag);

int platform_id(void);

static void configure_littleton_lcd(void);

static void __init littleton_init_irq(void)
{
	pxa_init_irq();
}

static struct ssp_dev ssp2;

/* TODO: One might want to consider abstracting all these ssp_command 
 *       functions into a single function.
 *	 Change the Magic number to MACRO.
 */

static void ssp_command(struct ssp_dev *dev, u32 data)
{
	u32 command = 0;

	ssp_disable(dev);
	ssp_config(dev, 0x0, 0x18, 0x0, 0x001fff81);

	command = data << 9;
	ssp_write_word(dev, command);
	ssp_flush(dev);
}

static void ssp_command_one(struct ssp_dev *dev, u32 data, u32 param)
{
	u32 command = 0;

	ssp_disable(dev);
	ssp_config(dev, 0x0, 0x18, 0x0, 0x001fff81);

	command = data << 9;
	command |= 0x100;
	command |= param;

	ssp_write_word(dev, command);
	ssp_flush(dev);
}

static void ssp_command_two(struct ssp_dev *dev, 
		u32 data, u32 param_one, u32 param_two)
{
	u32 command = 0;

	/* 
	 * Note the least signficant nible is 'a' here, 
	 * not '1' as in previous functions. We're now sending 27 bits!!
	 */
	ssp_disable(dev);
	ssp_config(dev, 0x0, 0x18, 0x0, 0x001fff8a);

	command = data << 18;
	command |= 0x20000;
	command |= (param_one << 9);
	command |= 0x100;
	command |= param_two;

	ssp_write_word(dev, command);
	ssp_flush(dev);
}

#ifdef CONFIG_FB_PXA
/*
 * We have a seperate backlight driver for Littleton to
 * control the LCD backlight and Keypad backlight.
 */
static void littleton_backlight_power(int on)
{
	return;
}

extern get_pm_state(void);
static void littleton_lcd_power(int on)
{
	if (PM_SUSPEND_LCDREFRESH != get_pm_state()) {
		printk(KERN_INFO "turn off/on LCD\n");
		if(on){
			/* Turn LCD ON */
			ssp_command(&ssp2, 0x29); /* Display ON */
			ssp_command_two(&ssp2, 0xB8, 0xFF, 0xF9); /* Output Control */
			ssp_command(&ssp2, 0x11); /* Sleep out */
			ssp_command_one(&ssp2, 0xB0, 0x16); /* Wake */
			configure_littleton_lcd();
		} else {
			/* Turn LCD OFF */
			ssp_command_one(&ssp2, 0x28, 0x00); /* Display off */
			ssp_command_two(&ssp2, 0xB8, 0x80, 0x02); /* Output control */
			ssp_command_one(&ssp2, 0x10, 0x00); /* Sleep in */
			ssp_command_one(&ssp2, 0xB0, 0x00); /* Deep stand by in */
		}
 
	}

	return;
}

#ifdef CONFIG_FB_PXA_LCD_VGA
static struct pxafb_mach_info tpo_tdo24mtea1_vga __initdata = {
	.pixclock		= 38250,
	.xres			= 480,
	.yres			= 640,
	.bpp			= 16,
	.hsync_len		= 8,
	.left_margin		= 8,
	.right_margin		= 24,
	.vsync_len		= 2,
	.upper_margin		= 2,
	.lower_margin		= 4,
	.sync			= 0,
	.lccr0			= LCCR0_Act,
	.lccr3			= 0,
	.pxafb_backlight_power	= littleton_backlight_power,
	.pxafb_lcd_power	= littleton_lcd_power,
};
#endif /* CONFIG_FB_PXA_LCD_VGA */

#ifdef CONFIG_FB_PXA_LCD_QVGA
static struct pxafb_mach_info tpo_tdo24mtea1_qvga __initdata = {
	.pixclock		= 153000,
	.xres			= 240,
	.yres			= 320,
	.bpp			= 16,
	.hsync_len		= 8,
	.left_margin		= 8,
	.right_margin		= 88,
	.vsync_len		= 2,
	.upper_margin		= 2,
	.lower_margin		= 2,
	.sync			= 0,
	.lccr0			= LCCR0_Act,
	.lccr3			= 0,
	.pxafb_backlight_power	= littleton_backlight_power,
	.pxafb_lcd_power	= littleton_lcd_power,
};
#endif /* CONFIG_FB_PXA_LCD_QVGA */

#endif /* CONFIG_FB_PXA */

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= (LITTLETON_ETH_PHYS + 0x300),
		.end	= (LITTLETON_ETH_PHYS + 0xfffff),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_GPIO(MFP2GPIO(MFP_DEBUG_ETH_INT_GPIO)),
		.end	= IRQ_GPIO(MFP2GPIO(MFP_DEBUG_ETH_INT_GPIO)),
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

struct pxa3xx_pin_config littleton_ffuart_pins[] = {

#ifdef CONFIG_CPU_PXA300
/*workaround for BASEBAND FFUART ISSUE, they will be cleaned when we update the blob!!!*/
PXA3xx_MFP_CFG("FFUART RXD", MFP_PIN_GPIO30, MFP_AF0, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART TXD", MFP_PIN_GPIO31, MFP_AF0, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
#endif

PXA3xx_MFP_CFG("FFUART RXD", MFP_FFRXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART TXD", MFP_FFTXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART CTS", MFP_FFCTS, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART RTS", MFP_FFRTS, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),

#ifndef	CONFIG_CPU_PXA310
PXA3xx_MFP_CFG("FFUART DCD", MFP_FFDCD, MFP_FFDCD_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART DSR", MFP_FFDSR, MFP_FFDSR_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART RI",  MFP_FFRI,  MFP_FFRI_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART DTR", MFP_FFDTR, MFP_FFDTR_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
#endif
};

struct pxa3xx_pin_config littleton_btuart_pins[] = {
PXA3xx_MFP_CFG("BTUART RTS", MFP_BT_RTS, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("BTUART RXD", MFP_BT_RXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("BTUART TXD", MFP_BT_TXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("BTUART CTS", MFP_BT_CTS, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config littleton_stuart_pins[] = {
PXA3xx_MFP_CFG("STUART TXD", MFP_STD_TXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("STUART RXD", MFP_STD_RXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

void pxa3xx_enable_ffuart_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_ffuart_pins, ARRAY_SIZE(littleton_ffuart_pins));
}

void pxa3xx_enable_btuart_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_btuart_pins, ARRAY_SIZE(littleton_btuart_pins));
}

void pxa3xx_enable_stuart_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_stuart_pins, ARRAY_SIZE(littleton_stuart_pins));
}

struct pxa3xx_pin_config littleton_dfc_pins[] = {
/*          description,   pin,                 alt fn,  drive,  rdh, lpm,              edge */
PXA3xx_MFP_CFG("DF INT RnB",MFP_DF_INT_RnB,	MFP_AF0, MFP_DS10X, 0, MFP_LPM_FLOAT,     MFP_EDGE_NONE),
PXA3xx_MFP_CFG("DF nRE",    MFP_DF_nRE,		MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND nWE",    MFP_DF_nWE,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND CLE",    MFP_ND_CLE,    	MFP_AF0, MFP_DS10X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND ALE1", 	 MFP_DF_nADV1,	    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
PXA3xx_MFP_CFG("DF nCS0",   MFP_DF_NCS0,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("DF nCS1",   MFP_DF_NCS1,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<0>",  MFP_DF_IO_0,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<1>",  MFP_DF_IO_1,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<2>",  MFP_DF_IO_2,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<3>",  MFP_DF_IO_3,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<4>",  MFP_DF_IO_4,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<5>",  MFP_DF_IO_5,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<6>",  MFP_DF_IO_6,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<7>",  MFP_DF_IO_7,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<8>",  MFP_DF_IO_8,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<9>",  MFP_DF_IO_9,    	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<10>", MFP_DF_IO_10,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<11>", MFP_DF_IO_11,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<12>", MFP_DF_IO_12,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<13>", MFP_DF_IO_13,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<14>", MFP_DF_IO_14,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("ND IO<15>", MFP_DF_IO_15,   	MFP_AF1, MFP_DS10X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_dfc_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_dfc_pins, ARRAY_SIZE(littleton_dfc_pins));
}

struct pxa3xx_pin_config littleton_lcd_pins[] = {
/*          description,   pin,           alt fn,  drive,   rdh, lpm,              edge */
PXA3xx_MFP_CFG("LCD LDD<0>",  MFP_L_DD_0,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<1>",  MFP_L_DD_1,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<2>",  MFP_L_DD_2,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<3>",  MFP_L_DD_3,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<4>",  MFP_L_DD_4,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<5>",  MFP_L_DD_5,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<6>",  MFP_L_DD_6,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<7>",  MFP_L_DD_7,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<8>",  MFP_L_DD_8,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<9>",  MFP_L_DD_9,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<10>", MFP_L_DD_10,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<11>", MFP_L_DD_11,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<12>", MFP_L_DD_12,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<13>", MFP_L_DD_13,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<14>", MFP_L_DD_14,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<15>", MFP_L_DD_15,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<16>", MFP_L_DD_16,   MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<17>", MFP_L_DD_17,   MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD FCLK",    MFP_L_FCLK,    MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LCLK",    MFP_L_LCLK,    MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD PCLK",    MFP_L_PCLK,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD BIAS",    MFP_L_BIAS,    MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config picoton_lcd_pins[] = {
/*          description,   pin,           alt fn,  drive,   rdh, lpm,              edge */
PXA3xx_MFP_CFG("LCD LDD<0>",  MFP_L_DD_0,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<1>",  MFP_L_DD_1,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<2>",  MFP_L_DD_2,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<3>",  MFP_L_DD_3,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<4>",  MFP_L_DD_4,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<5>",  MFP_L_DD_5,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<6>",  MFP_L_DD_6,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<7>",  MFP_L_DD_7,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<8>",  MFP_L_DD_8,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<9>",  MFP_L_DD_9,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<10>", MFP_L_DD_10,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<11>", MFP_L_DD_11,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<12>", MFP_L_DD_12,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<13>", MFP_L_DD_13,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<14>", MFP_L_DD_14,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<15>", MFP_L_DD_15,   MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<16>", MFP_L_DD_16,   MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<17>", MFP_L_DD_17,   MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD FCLK",    MFP_L_FCLK,    MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LCLK",    MFP_L_LCLK,    MFP_AF1, MFP_DS02X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD PCLK",    MFP_L_PCLK,    MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD BIAS",    MFP_L_BIAS,    MFP_AF1, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config littleton_mlcd_pins[] = {
/*          description,   pin,              	alt fn,  drive,   rdh, lpm,              edge */
PXA3xx_MFP_CFG("MLCD LDD<8>",  MFP_L_DD_8,     	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<9>",  MFP_L_DD_9,     	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<10>", MFP_L_DD_10,    	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<11>", MFP_L_DD_11,    	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<12>", MFP_L_DD_12,    	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<13>", MFP_L_DD_13,    	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<14>", MFP_L_DD_14,    	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<15>", MFP_L_DD_15,     MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<16>", MFP_L_DD_16,     MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<17>", MFP_L_DD_17,     MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<0>",  MFP_L_DD_0,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<1>",  MFP_L_DD_1,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<2>",  MFP_L_DD_2,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<3>",  MFP_L_DD_3,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<4>",  MFP_L_DD_4,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<5>",  MFP_L_DD_5,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<6>",  MFP_L_DD_6,     	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LDD<7>",  MFP_L_DD_7,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD FCLK",    MFP_L_FCLK,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD LCLK",    MFP_L_LCLK,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD PCLK",    MFP_L_PCLK,     	MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MLCD BIAS",    MFP_L_BIAS,      MFP_AF7, MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

#ifdef CONFIG_FB_PXA
void pxa3xx_enable_lcd_pins(void)
{
	if (platform_id() == 0x0)	/* Littleton 1.x */
		pxa3xx_mfp_set_configs(littleton_lcd_pins, ARRAY_SIZE(littleton_lcd_pins));
	else				/* Littleton 4.x */
		pxa3xx_mfp_set_configs(picoton_lcd_pins, ARRAY_SIZE(picoton_lcd_pins));
}
void pxa3xx_enable_mlcd_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_mlcd_pins, ARRAY_SIZE(littleton_mlcd_pins));
}
#endif

/* We depend NAND enable the IO pins for SMC91x */
struct pxa3xx_pin_config littleton_eth_pins[] = {
	PXA3xx_MFP_CFG("ETH INT", MFP_DEBUG_ETH_INT_GPIO,
			MFP_DEBUG_ETH_INT_GPIO_AF, MFP_DS03X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_eth_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_eth_pins, ARRAY_SIZE(littleton_eth_pins));
}

struct pxa3xx_pin_config littleton_i2c_pins[] = {
PXA3xx_MFP_CFG("I2C SCL", MFP_SCL, MFP_AF1, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("I2C SDA", MFP_SDA, MFP_AF1, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_i2c_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_i2c_pins, ARRAY_SIZE(littleton_i2c_pins));
}

struct pxa3xx_pin_config littleton_cif_pins[] = {
PXA3xx_MFP_CFG("CIF DD<0>", MFP_CIF_DD_0, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<1>", MFP_CIF_DD_1, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<2>", MFP_CIF_DD_2, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<3>", MFP_CIF_DD_3, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<4>", MFP_CIF_DD_4, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<5>", MFP_CIF_DD_5, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<6>", MFP_CIF_DD_6, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<7>", MFP_CIF_DD_7, 		MFP_AF0, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<8>", MFP_CIF_DD_8, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF DD<9>", MFP_CIF_DD_9, 		MFP_AF1, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF MCLK",  MFP_CIF_MCLK, 		MFP_AF0, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF PCLK",  MFP_CIF_PCLK, 		MFP_AF0, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF HSYNC", MFP_CIF_HSYNC, 	MFP_AF0, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("CIF VSYNC", MFP_CIF_VSYNC, 	MFP_AF0, MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_cif_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_cif_pins, ARRAY_SIZE(littleton_cif_pins));
}

struct pxa3xx_pin_config littleton_keyp_pins[] = {
PXA3xx_MFP_CFG("KEYP DKIN0",  MFP_KP_DKIN_0,  MFP_AF2, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP DKIN1",  MFP_KP_DKIN_1,  MFP_AF2, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKIN0",  MFP_KP_MKIN_0,  MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKIN1",  MFP_KP_MKIN_1,  MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKIN2",  MFP_KP_MKIN_2,  MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKIN3",  MFP_KP_MKIN_3,  MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKIN4",  MFP_KP_MKIN_4,  MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKIN5",  MFP_KP_MKIN_5,  MFP_AF1, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
PXA3xx_MFP_CFG("KEYP MKOUT0", MFP_KP_MKOUT_0, MFP_AF1, MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("KEYP MKOUT1", MFP_KP_MKOUT_1, MFP_AF1, MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("KEYP MKOUT2", MFP_KP_MKOUT_2, MFP_AF1, MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("KEYP MKOUT3", MFP_KP_MKOUT_3, MFP_AF1, MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("KEYP MKOUT4", MFP_KP_MKOUT_4, MFP_AF1, MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_keyp_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_keyp_pins, ARRAY_SIZE(littleton_keyp_pins));
}

#ifdef CONFIG_MMC
struct pxa3xx_pin_config littleton_mmc1_pins[] = {
PXA3xx_MFP_CFG("MMC1 CD0", MFP_MMC_CD_0_GPIO,  MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC1 D0",  MFP_MMC_DAT0, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC1 D1",  MFP_MMC_DAT1, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC1 D2",  MFP_MMC_DAT2, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC1 D3",  MFP_MMC_DAT3, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC1 CLK", MFP_MMC_CLK, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC1 CMD0",MFP_MMC_CMD_0, MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config littleton_mmc2_pins[] = {
PXA3xx_MFP_CFG("MMC2 D0",  MFP_MMC2_DAT0,     MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC2 D1",  MFP_MMC2_DAT1, 	   MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC2 D2",  MFP_MMC2_DAT2_CS0, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC2 D3",  MFP_MMC2_DAT3_CS1, MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC2 CLK", MFP_MMC2_CLK,      MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("MMC2 CMD", MFP_MMC2_CMD,      MFP_AF4, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

#ifdef CONFIG_CPU_PXA310
struct pxa3xx_pin_config zylonite_mmc3_pins[] = {
	PXA3xx_MFP_CFG("MMC3 D0",  MFP_MMC3_DAT0, MFP_MMC3_DAT0_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC3 D1",  MFP_MMC3_DAT1, MFP_MMC3_DAT1_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC3 D2",  MFP_MMC3_DAT2, MFP_MMC3_DAT2_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC3 D3",  MFP_MMC3_DAT3, MFP_MMC3_DAT3_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC3 CLK", MFP_MMC3_CLK, MFP_MMC3_CLK_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC3 CMD", MFP_MMC3_CMD, MFP_MMC3_CMD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};
#endif

void pxa3xx_enable_mmc1_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_mmc1_pins, ARRAY_SIZE(littleton_mmc1_pins));
}

void pxa3xx_enable_mmc2_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_mmc2_pins, ARRAY_SIZE(littleton_mmc2_pins));
}

#ifdef CONFIG_CPU_PXA310
void pxa3xx_enable_mmc3_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_mmc3_pins,
			ARRAY_SIZE(zylonite_mmc3_pins));
}
#endif
#endif

#ifdef CONFIG_WIFI_HOST_SLEEP
extern int btwlancamera_board_read_max7321(void);
extern int btwlancamera_board_write_max7321(u8 value);

struct pxa3xx_pin_config wifi_host_sleep_pins[] = {
	PXA3xx_MFP_CFG("WIFI WAKEUP HOST", MFP_WIFI_WAKEUP_HOST, MFP_WIFI_WAKEUP_HOST_AF,
			MFP_DS03X, 1, 0x10 | MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config picoton_wifi_host_sleep_pins[] = {                      /* Littleton 4.x (aka Picoton) */
        PXA3xx_MFP_CFG("WIFI WAKEUP HOST", MFP_WL_WAKE_MH, MFP_WL_WAKE_MH_AF,
                        MFP_DS03X,0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("HOST WAKEUP WIFI", MFP_MH_WAKE_WL, MFP_MH_WAKE_WL_AF,
                        MFP_DS03X,0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};


void pxa3xx_enable_wifi_host_sleep_pins(void)
{
	if (platform_id() == 0x0) /* Littleton 1.x */
	{
		u8 tmp = btwlancamera_board_read_max7321();

		/* set HOST_WAKEUP_WIFI pin to level HIGH */
		tmp |= 0x80;
		btwlancamera_board_write_max7321(tmp);

		/* config the WIFI_WAKEUP_HOST pin as edge fall eakeup source */
		pxa3xx_mfp_set_configs(wifi_host_sleep_pins,
				ARRAY_SIZE(wifi_host_sleep_pins));
		pxa3xx_mfp_set_edge(MFP_WIFI_WAKEUP_HOST, MFP_EDGE_FALL);
	}
	else			/* Littleton 4.x */
	{
#if 0		/* Picoton (Rev 4.0) has some voltage issues with host sleep pins. This code is disabled for now */ 
		
		pxa3xx_mfp_set_configs(picoton_wifi_host_sleep_pins, 
				ARRAY_SIZE(picoton_wifi_host_sleep_pins));
		pxa3xx_gpio_set_direction(MFP_MH_WAKE_WL, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MH_WAKE_WL, GPIO_LEVEL_HIGH);
		pxa3xx_gpio_set_direction(MFP_WL_WAKE_MH, GPIO_DIR_IN);
		pxa3xx_gpio_set_level(MFP_WL_WAKE_MH, GPIO_LEVEL_LOW);
#endif
	}
}

void pxa3xx_wifi_wakeup(int active)
{
	if (platform_id() == 0x0) /* Littleton 1.x */
	{
		u8 tmp = btwlancamera_board_read_max7321();

		if (active)
			tmp &= ~0x80;
		else
			tmp |= 0x80;

		btwlancamera_board_write_max7321(tmp);
	}
	else			 /* Littleton 4.x */
	{
#if 0		/* Picoton (Rev 4.0) has some voltage issues with host sleep GPIO pins. This code is disabled for now */
		if (active)
	                pxa3xx_gpio_set_level(MFP_MH_WAKE_WL, GPIO_LEVEL_LOW);
	        else
	                pxa3xx_gpio_set_level(MFP_MH_WAKE_WL, GPIO_LEVEL_HIGH);
#endif
	}			 
}

#endif

struct pxa3xx_pin_config littleton_ssp2_pins[] = {
PXA3xx_MFP_CFG("SSP2 SCLK", MFP_SSP_2_CLK, MFP_AF2, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP2 SFRM", MFP_SSP_2_LCD_CS, MFP_AF2, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP2 TXD", MFP_SSP_2_TXD, MFP_AF2, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config littleton_ssp3_pins[] = {
PXA3xx_MFP_CFG("SSP3 BITCLK", MFP_SSP_AUDIO_SCLK, MFP_AF1, MFP_DS08X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE), 
PXA3xx_MFP_CFG("SSP3 FRMCLK", MFP_SSP_AUDIO_FRM,  MFP_AF1, MFP_DS08X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP3 TXDCLK", MFP_SSP_AUDIO_TXD,  MFP_AF1, MFP_DS08X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP3 RXDCLK", MFP_SSP_AUDIO_RXD,  MFP_AF1, MFP_DS08X, 0, MFP_LPM_FLOAT,MFP_EDGE_NONE),
/* We need use network CLK as CLK source for Audio SSP */
PXA3xx_MFP_CFG("SSP3 NETWORK CLK", MFP_PIN_GPIO126,  MFP_AF3, MFP_DS08X, 0, MFP_LPM_FLOAT,MFP_EDGE_NONE),
};

struct pxa3xx_pin_config littleton_ssp4_pins[] = {
PXA3xx_MFP_CFG("SSP4 SCLK",MFP_SSP_4_CLK, MFP_AF1, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP4 SFRM",MFP_SSP_4_FRM, MFP_AF1, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP4 TXD", MFP_SSP_4_TXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("SSP4 RXD", MFP_SSP_4_RXD, MFP_AF1, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_ssp2_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_ssp2_pins,
			ARRAY_SIZE(littleton_ssp2_pins));
}

void pxa3xx_enable_ssp3_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_ssp3_pins,
			ARRAY_SIZE(littleton_ssp3_pins));
}

void pxa3xx_enable_ssp4_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_ssp4_pins,
			ARRAY_SIZE(littleton_ssp4_pins));
}

/* 
 * Littleton just support OTG on Littelton based LV. And ULPI handle the 
 * OTG for Monahans LV. 
 */
void pxa3xx_enable_otg_pins(void)
{
	return;
}


/*
 * Littleton L have no support for host because no OTG.
 * Littleton LV needn't such pins because no port1 support on LV.
 * TODO: Maybe need remove these pins definition for Littleton.
 */
void pxa3xx_enable_usbh_pins(void)
{
	return;
}

#ifdef CONFIG_CPU_PXA310
#define U2D_MFP_DS	MFP_DS08X
struct pxa3xx_pin_config littleton_u2d_pins[] = {
PXA3xx_MFP_CFG("U2D ULPI CLK",     MFP_ULPI_CLK, 	    MFP_ULPI_CLK_AF,      
	       	U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA0",        MFP_ULPI_DATAOUT_0, MFP_ULPI_DATAOUT_0_AF,
	       	U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA1",        MFP_ULPI_DATAOUT_1, MFP_ULPI_DATAOUT_1_AF, 
		U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA2",        MFP_ULPI_DATAOUT_2, MFP_ULPI_DATAOUT_2_AF,
	       	U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA3",        MFP_ULPI_DATAOUT_3, MFP_ULPI_DATAOUT_3_AF, 
		U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA4",        MFP_ULPI_DATAOUT_4, MFP_ULPI_DATAOUT_4_AF, 
		U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA5",        MFP_ULPI_DATAOUT_5, MFP_ULPI_DATAOUT_5_AF,
	       	U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA6",        MFP_ULPI_DATAOUT_6, MFP_ULPI_DATAOUT_6_AF, 
		U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA7",        MFP_ULPI_DATAOUT_7, MFP_ULPI_DATAOUT_7_AF, 
		U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D ULPI RESET",   MFP_ULPI_RESET, MFP_ULPI_RESET_AF,                /* Only used by Littleton 4.x */
                U2D_MFP_DS, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_u2d_pins(void)
{
	pxa3xx_pmic_set_voltage(VCC_USB, 2800);
	pxa3xx_mfp_set_afds(MFP_PIN_ULPI_STP, 0, 0);
	pxa3xx_mfp_set_afds(MFP_PIN_ULPI_DIR, 0, 0);
	pxa3xx_mfp_set_afds(MFP_PIN_ULPI_NXT, 0, 0);

	pxa3xx_mfp_set_configs(littleton_u2d_pins, ARRAY_SIZE(littleton_u2d_pins));
}
#endif

/* Only used by Littleton 4.x (aka Picoton) */

struct pxa3xx_pin_config picoton_wlan_pins[] = {
	PXA3xx_MFP_CFG("8686 WLAN POWER DOWN", MFP_WLAN8686_PWDN_GPIO, MFP_WLAN8686_PWDN_AF,
		MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};
	
/* Enable On-Chip 8686 WLAN for Littleton 4.x */

void pxa3xx_picoton_enable_wlan(void)
{
	pxa3xx_mfp_set_configs(picoton_wlan_pins, ARRAY_SIZE(picoton_wlan_pins));
}

/* Power Up On-Chip 8686 WLAN for Littleton 4.x */

void pxa3xx_picoton_poweron_wlan(void) {
	pxa3xx_gpio_set_direction(MFP_WLAN8686_PWDN_GPIO, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_WLAN8686_PWDN_GPIO, GPIO_LEVEL_HIGH);
}


struct pxa3xx_pin_config littleton_1w_pins[] = {
PXA3xx_MFP_CFG("1Wire", MFP_ONE_WIRE, MFP_AF5, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

void pxa3xx_enable_1w_pins(void)
{
	pxa3xx_mfp_set_configs(littleton_1w_pins, ARRAY_SIZE(littleton_1w_pins));
}


#ifdef CONFIG_MONAHANS_MAX7321
struct pxa3xx_pin_config gpio_exp_max7321_pins[] = {
PXA3xx_MFP_CFG("Tech Wakeup 0", MFP_TECH_WAKEUP0, MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("Tech Wakeup 1", MFP_TECH_WAKEUP1, MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("Tech Wakeup 2", MFP_TECH_WAKEUP2, MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("Tech Wakeup 3", MFP_TECH_WAKEUP3, MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_gpio_exp_max7321_pins(void)
{
	pxa3xx_mfp_set_configs(gpio_exp_max7321_pins, ARRAY_SIZE(gpio_exp_max7321_pins));
	pxa3xx_gpio_set_direction(MFP_TECH_WAKEUP0, GPIO_DIR_IN); 
	pxa3xx_gpio_set_direction(MFP_TECH_WAKEUP1, GPIO_DIR_IN); 
	pxa3xx_gpio_set_direction(MFP_TECH_WAKEUP2, GPIO_DIR_IN); 
	pxa3xx_gpio_set_direction(MFP_TECH_WAKEUP3, GPIO_DIR_IN); 
}
#endif

void lcd_ssp_init(void)
{
	pxa3xx_enable_ssp2_pins();
	pxa_set_cken(CKEN_SSP2, 1);

	/* grab the port, configure it, then enable it */
	ssp_init(&ssp2, 2, SSP_NO_IRQ);
	ssp_disable(&ssp2);
	ssp_config(&ssp2, 0x0, 0x18, 0x0, 0x001fff81);
	ssp_enable(&ssp2);
	ssp_flush(&ssp2);
}

void lcd_panel_reset(void)
{
	/* start chucking commands at the lcd */
	ssp_command(&ssp2, 0x1); // reset
	ssp_command(&ssp2, 0x0); // nop
	ssp_command(&ssp2, 0x0); // nop
	ssp_command(&ssp2, 0x0); // nop
}

/****************************/
/* VGA pass-through display */
/* D4 D3 D1 D0              */
/*  0  0  0  0              */
/****************************/
void vga_pass_through_mode(void)
{
	ssp_command_one(&ssp2, 0xB0, 0x16);
	ssp_command_one(&ssp2, 0xBC, 0x80);
	ssp_command_one(&ssp2, 0xE1, 0x00);
	if (0 == platform_id())	/* Littleton Rev 1.x */
		ssp_command_one(&ssp2, 0x36, 0x50);
	else	/* Littleton Rev 4.x */
		ssp_command_one(&ssp2, 0x36, 0x00);
		
	ssp_command_one(&ssp2, 0x3B, 0x00);
}

/*****************************/
/* QVGA pass-through display */
/* D4 D3 D1 D0               */
/*  0  0  0  1               */
/*****************************/
void qvga_pass_through_mode(void)
{
	printk(KERN_ERR "qvga_pass_through_mode\n");
	ssp_command_one(&ssp2, 0xB0, 0x16);
	ssp_command_one(&ssp2, 0xBC, 0x81);
	ssp_command_one(&ssp2, 0xE1, 0x00);

	if (0 == platform_id())	/* Littleton Rev 1.x */
		ssp_command_one(&ssp2, 0x36, 0x50);
	else	/* Littleton Rev 4.x */
		ssp_command_one(&ssp2, 0x36, 0x00);

	ssp_command_one(&ssp2, 0x3B, 0x22);
}

/*************************************/
/* QVGA RAM display + QVGA RAM write */
/* D4 D3 D1 D0                       */
/*  0  1  1  0                       */
/*************************************/
void qvga_ram_write_mode(void)
{
	printk(KERN_ERR "qvga_ram_write_mode\n");
	ssp_command_one(&ssp2, 0xB0, 0x17);
	ssp_command_one(&ssp2, 0xBC, 0xAA);
	ssp_command_one(&ssp2, 0x36, 0xC0);
	ssp_command_one(&ssp2, 0x3B, 0x22);
	ssp_command_one(&ssp2, 0xE1, 0x01);
}

void vga_transfer_config(void)
{
	printk(KERN_ERR "vga_transfer_mode()\n");
	ssp_command_one(&ssp2, 0xcf, 0x02); // Blanking period control (1)
	ssp_command_two(&ssp2, 0xd0, 0x08, 0x04); // Blanking period control (2)
	ssp_command_one(&ssp2, 0xd1, 0x01); // CKV timing control on/off
	ssp_command_two(&ssp2, 0xd2, 0x14, 0x00); // CKV 1,2 timing control
	ssp_command_two(&ssp2, 0xd3, 0x1a, 0x0f); // OEV timing control
	ssp_command_two(&ssp2, 0xd4, 0x1f, 0xaf); // ASW timing control (1)
	ssp_command_one(&ssp2, 0xd5, 0x14); // ASW timing control (2)
	 
	ssp_command(&ssp2, 0x21); // Invert for normally black display
	ssp_command(&ssp2, 0x29); // Display on
}

void qvga_transfer_config(void)
{
	printk(KERN_ERR "qvga_transfer_config()\n");
	ssp_command_one(&ssp2, 0xd6, 0x02); // Blanking period control (1)
	ssp_command_two(&ssp2, 0xd7, 0x08, 0x04); // Blanking period control (2)
	ssp_command_one(&ssp2, 0xd8, 0x01); // CKV timing control on/off
	ssp_command_two(&ssp2, 0xd9, 0x00, 0x08); // CKV 1,2 timing control
	ssp_command_two(&ssp2, 0xde, 0x05, 0x0a); // OEV timing control
	ssp_command_two(&ssp2, 0xdf, 0x0a, 0x19); // ASW timing control (1)
	ssp_command_one(&ssp2, 0xe0, 0x0a); // ASW timing control (2)
	 
	ssp_command(&ssp2, 0x21); // Invert for normally black display
	ssp_command(&ssp2, 0x29); // Display on
}

void qvga_osc_config(void)
{
	printk(KERN_ERR "qvga_osc_config()\n");
	ssp_command_one(&ssp2, 0xe0, 0x0A);
	ssp_command_one(&ssp2, 0xe2, 0x00);
	ssp_command_one(&ssp2, 0xe3, 0x32);
	ssp_command_two(&ssp2, 0xe4, 0x00, 0x03);
	ssp_command_two(&ssp2, 0xe5, 0x02, 0x04);
	ssp_command_one(&ssp2, 0xe6, 0x03);
	ssp_command_two(&ssp2, 0xe7, 0x04, 0x0A);
	ssp_command_one(&ssp2, 0xe8, 0x04);
	ssp_command_one(&ssp2, 0xe9, 0x10);
	ssp_command_two(&ssp2, 0xea, 0x20, 0x00);
		
	ssp_command(&ssp2, 0x21); // Invert for normally black display
	ssp_command(&ssp2, 0x29); // Display on
}

void lcd_panel_config(void)
{
	ssp_command_two(&ssp2, 0xb8, 0xff, 0xf9); // Output control
	ssp_command(&ssp2, 0x11); // sleep out
	ssp_command_one(&ssp2, 0xba, 0x01); // Display mode (1)
	ssp_command_one(&ssp2, 0xbb, 0x00); // Display mode (2)
	ssp_command_one(&ssp2, 0x3a, 0x60); // Display mode 18-bit RGB
	ssp_command_one(&ssp2, 0xbf, 0x10); // Drive system change control
	ssp_command_one(&ssp2, 0xb1, 0x56); // Booster operation setup
	ssp_command_one(&ssp2, 0xb2, 0x33); // Booster mode setup
	ssp_command_one(&ssp2, 0xb3, 0x11); // Booster frequency setup
	ssp_command_one(&ssp2, 0xb4, 0x02); // Op amp/system clock
	ssp_command_one(&ssp2, 0xb5, 0x35); // VCS voltage
	ssp_command_one(&ssp2, 0xb6, 0x40); // VCOM voltage
	ssp_command_one(&ssp2, 0xb7, 0x03); // External display signal
	ssp_command_one(&ssp2, 0xbd, 0x00); // ASW slew rate
	ssp_command_one(&ssp2, 0xbe, 0x00); // Dummy data for QuadData operation
	ssp_command_one(&ssp2, 0xc0, 0x11); // Sleep out FR count (A)
	ssp_command_one(&ssp2, 0xc1, 0x11); // Sleep out FR count (B)
	ssp_command_one(&ssp2, 0xc2, 0x11); // Sleep out FR count (C)
	ssp_command_two(&ssp2, 0xc3, 0x20, 0x40); // Sleep out FR count (D)
	ssp_command_two(&ssp2, 0xc4, 0x60, 0xc0); // Sleep out FR count (E)
	ssp_command_two(&ssp2, 0xc5, 0x10, 0x20); // Sleep out FR count (F)
	ssp_command_one(&ssp2, 0xc6, 0xc0); // Sleep out FR count (G)
	ssp_command_two(&ssp2, 0xc7, 0x33, 0x43); // Gamma 1 fine tuning (1)
	ssp_command_one(&ssp2, 0xc8, 0x44); // Gamma 1 fine tuning (2)
	ssp_command_one(&ssp2, 0xc9, 0x33); // Gamma 1 inclination adjustment
	ssp_command_one(&ssp2, 0xca, 0x00); // Gamma 1 blue offset adjustment
	ssp_command_two(&ssp2, 0xec, 0x01, 0xf0); // Horizontal clock cycles
}

static void configure_littleton_lcd(void)
{ 
	printk("Configuring Littleton LCD panel...\n");

	lcd_ssp_init();
	lcd_panel_reset();

#ifdef CONFIG_FB_PXA_LCD_VGA
	vga_pass_through_mode();
#elif defined (CONFIG_FB_PXA_LCD_QVGA)
	#ifdef CONFIG_LITTLETON_SMART_PANEL
		qvga_ram_write_mode();
	#else	
		qvga_pass_through_mode();
	#endif
#endif

	lcd_panel_config();

#ifdef CONFIG_FB_PXA_LCD_VGA
	vga_transfer_config();
#elif defined (CONFIG_FB_PXA_LCD_QVGA)
	#ifdef CONFIG_LITTLETON_SMART_PANEL
		qvga_osc_config();
	#else
		qvga_transfer_config();
	#endif
#endif
}

#ifdef CONFIG_PXA_IRDA
extern int max7320_read(void);
extern int max7320_write(u8 value);
static void enable_ir()
{
	u8 val;
	val = max7320_read();
	val &= ~0x40;
	max7320_write(val);
}

static void disable_ir()
{
	u8 val;
	val = max7320_read();
	val |= 0x40;
	max7320_write(val);
}

static void littleton_irda_transceiver_mode(struct device *dev, int mode)
{
	unsigned long flags;
	static int irda_mfp_init = 0;

	if (!irda_mfp_init) {
		printk(KERN_INFO "littleton_irda_transceiver_mode: init\n");
		pxa3xx_mfp_set_afds(MFP_CIR_ON_PWM, MFP_AF0, MFP_DS03X);
		pxa3xx_mfp_set_lpm(MFP_CIR_ON_PWM, MFP_LPM_DRIVE_LOW);
		pxa3xx_gpio_set_direction(MFP_CIR_ON_PWM, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_CIR_ON_PWM, 0);

		irda_mfp_init = 1;
	}

	local_irq_save(flags);
	if (mode & IR_SIRMODE) {
		printk(KERN_INFO "littleton_irda_transceiver_mode: SIR\n");
		enable_ir();
	} else if (mode & IR_FIRMODE) {
		/* do not support FIR */
	}
	if (mode & IR_OFF) {
		printk(KERN_INFO "littleton_irda_transceiver_mode: OFF\n");
		disable_ir();
	}
	local_irq_restore(flags);
}

static struct pxaficp_platform_data littleton_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = littleton_irda_transceiver_mode,
};
#endif


/* if micco is detected, return 1.
 * Otherwise, return 0. At this time, we believe other PMIC is installed.
 * It should be invoked before I2C driver enabled.
 */
static int detect_micco(void)
{
	int micco_flag, tmp;

	/* when the pwr i2c is enabled, the system will crash after low power
	 * mode change for about thousands of times. so we disable this 
	 * feature here and use software i2c instead. To do this we just 
	 * return 0 here.
	 */
	return 0;

	/*
	 * Polling 0x34 address (micco address) in standard I2C bus.
	 * Required: I2C bus and SDA,SCL pins are initialized.
	 */
	/* enable ITEIE, BEIE */
	ICR = 0x500;
	/* enable I2C unit and SCL */
	ICR |= 0x60;

	/* write operation */
	IDBR = (0x34 << 1);
	/* set ICR[start], ICR[TB], clear ICR[STOP] */
	ICR |= 0x9;

	/* wait ITE */
	while (!(ISR & 0x40)) {}
	if (ISR & 0x400) {
		/* not detect micco
		 * I2C interface will auto-send stop
		 */
		micco_flag = 0;
	} else {
		/* detect micco */
		micco_flag = 1;
	}
	/* clear ISR */
	tmp = ISR;
	ISR = tmp;

	if (micco_flag) {
		/* set ICR[stop], ICR[TB], clear ICR[START] */
		tmp = ICR;
		tmp &= ~0x01;
		tmp |= 0x0a;
		ICR = tmp;

		while (!(ISR & 0x40)) {}

		/* clear ISR */
		tmp = ISR;
		ISR = tmp;

		/* clear ICR[stop] */
		ICR &= ~0x02;
	}

	return micco_flag;
}

/*
 * Read Platform & Revision ID for board from GPIOs (GPIO6_2..GPIO0_2).
 * Littleton: 0x00 (Rev 1.0), 0x1 (Rev 1.1), 0x02 (Rev 1.2)
 * 0x40 (AKA Picoton Rev 1.0)
 */

static unsigned int board_id = 0;
static uint32_t board_detect_pins_mfpr[] = {
	/* AF0, DS 1X, Pull Neither, Edge Clear */
	0x8440, 0x8440, 0x8440, 0x8440, 0x8440, 0x8440, 0x8440 };

#define	NUM_BOARD_DETECT_PINS	ARRAY_SIZE(board_detect_pins)
#define	PLATFORM_ID_BIT_2	0
#define	PLATFORM_ID_BIT_1	1
#define	PLATFORM_ID_BIT_0	2
#define	REV_ID_BIT_3		3
#define	REV_ID_BIT_2		4
#define	REV_ID_BIT_1		5
#define	REV_ID_BIT_0		6

static unsigned int board_detect_pins[] = {
	MFP_PIN_GPIO6_2,
	MFP_PIN_GPIO5_2,
	MFP_PIN_GPIO4_2,
	MFP_PIN_GPIO3_2,
	MFP_PIN_GPIO2_2,
	MFP_PIN_GPIO1_2,
	MFP_PIN_GPIO0_2,
};

static char *platform_rev_id[4] = {"Rev 1.0", "Rev 1.1", "Rev 1.2", "Rev 4.0"};

static void littleton_board_detect(void)
{
	unsigned int id = 0, i;
	uint32_t board_detect_saved_mfpr[NUM_BOARD_DETECT_PINS];

	/* Save the original MFP settings */
	for (i = 0; i < NUM_BOARD_DETECT_PINS; i++) {
		board_detect_saved_mfpr[i] = MFP_REG(board_detect_pins[i]);
		MFP_REG(board_detect_pins[i]) = board_detect_pins_mfpr[i];
	}
	MFP_REG(board_detect_pins[i-1]);

	/* Set detect pins as gpio input */
	for (i = 0; i < 7; i++)
		pxa3xx_gpio_set_direction(board_detect_pins[i], GPIO_DIR_IN);
	
	/* Get board ID */
	board_id = 0;
	board_id =
		(pxa3xx_gpio_get_level(board_detect_pins[PLATFORM_ID_BIT_2]) << 6)|
		(pxa3xx_gpio_get_level(board_detect_pins[PLATFORM_ID_BIT_1]) << 5)|
		(pxa3xx_gpio_get_level(board_detect_pins[PLATFORM_ID_BIT_0]) << 4)|
		(pxa3xx_gpio_get_level(board_detect_pins[REV_ID_BIT_3]) << 3)|
		(pxa3xx_gpio_get_level(board_detect_pins[REV_ID_BIT_2]) << 2)|
		(pxa3xx_gpio_get_level(board_detect_pins[REV_ID_BIT_1]) << 1)|
		(pxa3xx_gpio_get_level(board_detect_pins[REV_ID_BIT_0]));
	
	printk("%s: Platform: Littleton (%s) Board Id: 0x%2x\n",
		__func__, platform_rev_id[(board_id && 0x07)],board_id);

	for (i = 0; i < NUM_BOARD_DETECT_PINS; i++)
		MFP_REG(board_detect_pins[i]) = board_detect_saved_mfpr[i];
	MFP_REG(board_detect_pins[i-1]);
}

int platform_id(void)
{
	return ((board_id >> 4));
}

static void __init littleton_init(void)
{
	if (detect_micco()) {
		pwri2c_flag = 1;
		/* enable FVE,PVE,TVE bit */
		PVCR = 0xe0500034;
	} else {
		pwri2c_flag = 0;
		/* disable FVE,PVE,TVE,FVC bit */
		PVCR &= 0x0fffffff;
	}

	/*
	 * Note: we depend bootloader set the correct
	 * value to MSC register for SMC91x.
	 */
	platform_device_register(&smc91x_device);

	littleton_board_detect();

#ifdef CONFIG_KEYBOARD_PXA3xx
	pxa_set_keypad_info(&littleton_keypad_info);
#endif

#ifdef CONFIG_FB_PXA
	pxa3xx_enable_lcd_pins();
	configure_littleton_lcd();
#ifdef CONFIG_FB_PXA_LCD_VGA
	set_pxa_fb_info(&tpo_tdo24mtea1_vga);
#elif defined(CONFIG_FB_PXA_LCD_QVGA)
	set_pxa_fb_info(&tpo_tdo24mtea1_qvga);
#endif
#endif

#ifdef CONFIG_PXA_IRDA
	pxa_set_ficp_info(&littleton_ficp_platform_data);
#endif

	pxa3xx_enable_eth_pins();
	pxa3xx_enable_i2c_pins();
#ifdef CONFIG_MMC
	pxa3xx_enable_mmc1_pins();
	pxa3xx_enable_mmc2_pins();
#endif

	if (0x4 == platform_id())  /* PowerOn On-Chip 8686 WiFi just for 4.x Littleton */
	{
		pxa3xx_picoton_enable_wlan();
		pxa3xx_picoton_poweron_wlan();
	}

#ifdef CONFIG_WIFI_HOST_SLEEP
	pxa3xx_enable_wifi_host_sleep_pins();
#endif

	if (0 == platform_id())	/* Just 1.x Littleton has keypad */
		pxa3xx_enable_keyp_pins();

	pxa3xx_enable_ffuart_pins();
	pxa3xx_enable_1w_pins();

#ifdef CONFIG_MONAHANS_MAX7321
	pxa3xx_enable_gpio_exp_max7321_pins();
#endif
	pxa3xx_enable_ssp4_pins();
	pxa3xx_enable_ssp3_pins();
}

static struct map_desc littleton_io_desc[] __initdata = {
};

static void __init littleton_map_io(void)
{
	pxa_map_io();
	iotable_init(littleton_io_desc, ARRAY_SIZE(littleton_io_desc));
}

MACHINE_START(LITTLETON, "Marvell Form Factor Development Platform (aka Littleton)")
#ifdef CONFIG_CPU_PXA310
	.phys_io	= 0x40000000,
	.boot_params	= 0xb0000100,
#else
	.phys_io        = 0x40000000,
	.boot_params    = 0x80000100,
#endif
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = littleton_map_io,
	.init_irq       = littleton_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = littleton_init,
MACHINE_END

EXPORT_SYMBOL_GPL(pxa3xx_enable_i2c_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_dfc_pins);

#ifdef CONFIG_FB_PXA
EXPORT_SYMBOL_GPL(pxa3xx_enable_lcd_pins);
EXPORT_SYMBOL_GPL(vga_pass_through_mode);
EXPORT_SYMBOL_GPL(qvga_pass_through_mode);
EXPORT_SYMBOL_GPL(qvga_ram_write_mode);
EXPORT_SYMBOL_GPL(lcd_ssp_init);
EXPORT_SYMBOL_GPL(lcd_panel_config);
EXPORT_SYMBOL_GPL(vga_transfer_config);
EXPORT_SYMBOL_GPL(qvga_transfer_config);
EXPORT_SYMBOL_GPL(qvga_osc_config);
#endif

EXPORT_SYMBOL_GPL(pxa3xx_enable_keyp_pins);

#ifdef CONFIG_MMC
EXPORT_SYMBOL_GPL(pxa3xx_enable_mmc1_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_mmc2_pins);
#ifdef CONFIG_CPU_PXA310
EXPORT_SYMBOL_GPL(pxa3xx_enable_mmc3_pins);
#endif
#endif

EXPORT_SYMBOL_GPL(pxa3xx_enable_ffuart_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_btuart_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_stuart_pins);

#ifdef CONFIG_CPU_PXA310
EXPORT_SYMBOL_GPL(pxa3xx_enable_u2d_pins);
#endif

#ifdef CONFIG_WIFI_HOST_SLEEP
EXPORT_SYMBOL_GPL(pxa3xx_enable_wifi_host_sleep_pins);
EXPORT_SYMBOL_GPL(pxa3xx_wifi_wakeup);
#endif

EXPORT_SYMBOL_GPL(platform_id);
EXPORT_SYMBOL_GPL(pxa3xx_enable_1w_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp2_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp3_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp4_pins);

EXPORT_SYMBOL_GPL(pxa3xx_enable_cif_pins);
