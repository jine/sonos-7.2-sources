/*
 *  linux/arch/arm/mach-pxa/zylonite.c
 *
 *  Support for the Intel Zylonite Development Platform.
 *
 *  Author:	Roy Huang
 *  Created:	Nov 15, 2004
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
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fb.h>
#include <linux/root_dev.h>
#include <linux/delay.h>
#include <linux/nodemask.h>
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
#include <asm/arch/pxa3xx_pmic.h>
#ifdef CONFIG_MMC_OPEN_SOURCE
#include <asm/arch/mmc.h>
#endif
#ifdef CONFIG_KEYBOARD_PXA3xx
#include <asm/arch/pxa3xx_keypad.h>
#endif
#include <asm/arch/ohci.h>
#include "generic.h"

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

static void __init zylonite_init_irq(void)
{
	pxa_init_irq();
}

#ifdef CONFIG_MMC_OPEN_SOURCE

#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310)
#define MMC_CD0		(GPIO_EXT_TO_IRQ(128))
#define MMC_CD1		(GPIO_EXT_TO_IRQ(129))
#define MMC_CD3		(GPIO_EXT_TO_IRQ(158))
#else
#define MMC_CD0		IRQ_GPIO1
#define MMC_CD1		IRQ_GPIO(4)
#endif

static int zylonite_mci_ro(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int level = GPIO_LEVEL_LOW;
	
	if (pdev->id == 0) {
		level = pxa3xx_gpio_get_level(MFP_MMC_WP_0_N_GPIO);
	}
	return level == GPIO_LEVEL_HIGH;
}

static int zylonite_mci_init(struct device *dev, irqreturn_t (*zylonite_detect_int)(int, void *, struct pt_regs *), void *data)
{
	int err;
	struct platform_device *pdev = to_platform_device(dev);

	/*
	 * setup GPIO for Zylonite MMC controller
	 */
	if (pdev->id == 0) {
		pxa3xx_enable_mmc1_pins();
		pxa3xx_mfp_set_afds(MFP_MMC_CMD_0, MFP_AF4, MFP_DS03X);

		err = request_irq(MMC_CD0, zylonite_detect_int,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"MMC card detect", data);
		if (err) {
			printk(KERN_ERR "zylonite_mci_init: MMC/SD: can't request MMC card detect IRQ\n");
			return -1;
		}
		set_irq_type(MMC_CD0, IRQT_BOTHEDGE);
	}
	else if (pdev->id == 1) {
		pxa3xx_enable_mmc2_pins();
		pxa3xx_gpio_set_direction(MFP_MMC2_CMD, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC2_CMD, GPIO_LEVEL_HIGH);
	}

	return 0;
}

static void zylonite_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	if (pdev->id == 0)
		free_irq(MMC_CD0, data);
}

static struct pxamci_platform_data zylonite_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= zylonite_mci_init,
	.exit		= zylonite_mci_exit,
	.get_ro		= zylonite_mci_ro,
};
#endif

#ifdef CONFIG_KEYBOARD_PXA3xx
static unsigned int zylonite_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_A), KEY(0, 1, KEY_B), KEY(0, 2, KEY_C), KEY(0, 5, KEY_D),
	KEY(1, 0, KEY_E), KEY(1, 1, KEY_F), KEY(1, 2, KEY_G), KEY(1, 5, KEY_H),
	KEY(2, 0, KEY_I), KEY(2, 1, KEY_J), KEY(2, 2, KEY_K), KEY(2, 5, KEY_L),
	KEY(3, 0, KEY_M), KEY(3, 1, KEY_N), KEY(3, 2, KEY_O), KEY(3, 5, KEY_P),
	KEY(5, 0, KEY_Q), KEY(5, 1, KEY_R), KEY(5, 2, KEY_S), KEY(5, 5, KEY_T),
	KEY(6, 0, KEY_U), KEY(6, 1, KEY_V), KEY(6, 2, KEY_W), KEY(6, 5, KEY_X),
	KEY(7, 1, KEY_Y), KEY(7, 2, KEY_Z),

	KEY(4, 4, KEY_0), KEY(1, 3, KEY_1), KEY(4, 1, KEY_2), KEY(1, 4, KEY_3),
	KEY(2, 3, KEY_4), KEY(4, 2, KEY_5), KEY(2, 4, KEY_6), KEY(3, 3, KEY_7),
	KEY(4, 3, KEY_8), KEY(3, 4, KEY_9),

	KEY(4, 5, KEY_SPACE), 
	KEY(5, 3, KEY_KPASTERISK), 	/* * */
	KEY(5, 4, KEY_KPDOT), 		/* #" */

	KEY(0, 7, KEY_UP),
	KEY(1, 7, KEY_DOWN),
	KEY(2, 7, KEY_LEFT),
	KEY(3, 7, KEY_RIGHT),
	KEY(2, 6, KEY_HOME),
	KEY(3, 6, KEY_END),
	KEY(6, 4, KEY_DELETE),
	KEY(6, 6, KEY_BACK),
	KEY(6, 3, KEY_CAPSLOCK), 	/* KEY_LEFTSHIFT), */

	KEY(4, 6, KEY_ENTER),   /* scroll push */
	KEY(5, 7, KEY_ENTER),	/* keypad action */

	KEY(0, 4, KEY_EMAIL),
	KEY(5, 6, KEY_SEND),
	KEY(4, 0, KEY_CALENDAR),
	KEY(7, 6, KEY_RECORD),
	KEY(6, 7, KEY_VOLUMEUP),
	KEY(7, 7, KEY_VOLUMEDOWN),

	KEY(0, 6, KEY_F22),	/* soft1 */
	KEY(1, 6, KEY_F23),	/* soft2 */

	KEY(0, 3, KEY_AUX),	/* contact */
};

static struct pxa3xx_keypad_platform_data zylonite_keypad_info = {
	.enable_repeat_key	= 1,
	.enable_matrix_key	= 1,
	.enable_direct_key	= 1,
	.enable_rotary_key	= 1,
	.matrix_key_debounce	= 30,
	.direct_key_debounce	= 30,
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.direct_key_num		= 2,

	.matrix_key_map		= zylonite_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(zylonite_matrix_key_map),

	.direct_key_map		= NULL,
	.direct_key_map_size	= -1,

	.rotary_up_key		= KEY_UP,	/* KEY_SCROLLUP, */
	.rotary_down_key	= KEY_DOWN, 	/* KEY_SCROLLDOWN, */
};
#endif

#ifdef CONFIG_PXA_IRDA
static void zylonite_irda_transceiver_mode(struct device *dev, int mode)
{
        unsigned long flags;
	static int irda_mfp_init = 0;

	if (!irda_mfp_init) {
		printk(KERN_INFO "zylonite_irda_transceiver_mode: init\n");
		pxa3xx_mfp_set_afds(MFP_CIR_ON_PWM, MFP_AF0, MFP_DS03X);
		pxa3xx_mfp_set_lpm(MFP_CIR_ON_PWM, MFP_LPM_DRIVE_LOW);
#ifdef CONFIG_CPU_PXA320
		pxa3xx_mfp_set_afds(MFP_IR_SHDN_N_GPIO, MFP_AF0, MFP_DS03X);
		pxa3xx_mfp_set_lpm(MFP_IR_SHDN_N_GPIO, MFP_LPM_PULL_LOW);
#endif
		pxa3xx_gpio_set_direction(MFP_CIR_ON_PWM, GPIO_DIR_OUT);
		pxa3xx_gpio_set_direction(MFP_IR_SHDN_N_GPIO,  GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_CIR_ON_PWM, 0);
		pxa3xx_gpio_set_level(MFP_IR_SHDN_N_GPIO, 1);

		irda_mfp_init = 1;
	}

        local_irq_save(flags);
        if (mode & IR_SIRMODE) {
		printk(KERN_INFO "zylonite_irda_transceiver_mode: SIR\n");
		pxa3xx_gpio_set_level(MFP_CIR_ON_PWM, 0);
		pxa3xx_gpio_set_level(MFP_IR_SHDN_N_GPIO, 0);
        } else if (mode & IR_FIRMODE) {
		/* do not support FIR */
        }
        if (mode & IR_OFF) {
		printk(KERN_INFO "zylonite_irda_transceiver_mode: OFF\n");
		pxa3xx_gpio_set_level(MFP_CIR_ON_PWM, 0);
		pxa3xx_gpio_set_level(MFP_IR_SHDN_N_GPIO, 1);
        }
        local_irq_restore(flags);
}

static struct pxaficp_platform_data zylonite_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = zylonite_irda_transceiver_mode,
};
#endif

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= (ZYLONITE_ETH_PHYS + 0x300),
		.end	= (ZYLONITE_ETH_PHYS + 0xfffff),
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

#ifdef CONFIG_FB_PXA

static void zylonite_backlight_power(int on)
{
	/* MFP for MFP_BACKLIGHT_PWM is supposed to be configured */
	pxa3xx_gpio_set_direction(MFP_BACKLIGHT_PWM, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_BACKLIGHT_PWM, on);
}

#ifdef CONFIG_FB_PXA_LCD_QVGA
static struct pxafb_mach_info toshiba_ltm035a776c __initdata = {
	.pixclock		= 110000,
	.xres			= 240,
	.yres			= 320,
#if defined(CONFIG_FB_PXA_LCD_19BPP)
	.bpp			= 19,
#elif defined(CONFIG_FB_PXA_LCD_18BPP)
	.bpp			= 18,
#else
	.bpp			= 16,
#endif
	.hsync_len		= 4,
	.left_margin		= 6,
	.right_margin		= 4,
	.vsync_len		= 2,
	.upper_margin		= 2,
	.lower_margin		= 3,
	.sync			= FB_SYNC_VERT_HIGH_ACT,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP,
	.pxafb_backlight_power	= zylonite_backlight_power,
	.pxafb_lcd_power	= NULL,
};

static struct pxafb_mach_info sharp_ls037_qvga __initdata = {
	.pixclock		= 183333,		
	.xres			= 240,
	.yres			= 320,
#if defined(CONFIG_FB_PXA_LCD_19BPP)
	.bpp			= 19,
#elif defined(CONFIG_FB_PXA_LCD_18BPP)
	.bpp			= 18,
#else
	.bpp			= 16,
#endif
	.hsync_len		= 4,
	.left_margin		= 39,
	.right_margin		= 39,
	.vsync_len		= 1,
	.upper_margin		= 2,
	.lower_margin		= 3,
	.sync			= 0,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP,
	.pxafb_backlight_power	= zylonite_backlight_power,
	.pxafb_lcd_power	= NULL,
};
#endif

#ifdef CONFIG_FB_PXA_LCD_VGA
static struct pxafb_mach_info toshiba_ltm04c380k __initdata = {
	.pixclock		= 50000,
	.xres			= 640,
	.yres			= 480,
#if defined(CONFIG_FB_PXA_LCD_19BPP)
	.bpp			= 19,
#elif defined(CONFIG_FB_PXA_LCD_18BPP)
	.bpp			= 18,
#else
	.bpp			= 16,
#endif
	.hsync_len		= 1,
	.left_margin		= 0x9f,
	.right_margin		= 1,
	.vsync_len		= 0x2c,
	.upper_margin		= 0,
	.lower_margin		= 0,
	.sync			= FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP,
	.pxafb_backlight_power	= zylonite_backlight_power,
	.pxafb_lcd_power	= NULL,
};

static struct pxafb_mach_info sharp_ls037_vga __initdata = {
	.pixclock		= 45833,
	.xres			= 480,
	.yres			= 640,
#if defined(CONFIG_FB_PXA_LCD_19BPP)
	.bpp			= 19,
#elif defined(CONFIG_FB_PXA_LCD_18BPP)
	.bpp			= 18,
#else
	.bpp			= 16,
#endif
	.hsync_len		= 8,
	.left_margin		= 81,
	.right_margin		= 81,
	.vsync_len		= 1,
	.upper_margin		= 2,
	.lower_margin		= 7,
	.sync			= 0,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP,
	.pxafb_backlight_power	= zylonite_backlight_power,
	.pxafb_lcd_power	= NULL,
};
#endif

/*
 * lcd_id - readed from the LCD panel, the newly designed LCD panels
 * will have a specific ID. The old panel ID is zero. This variable
 * is made static so that enable_lcd_pins() can initialize the MFP
 * pins accordindly.
 *
 * The LCD ID uses those unused LCD pins, as listed in
 * zylonite_lcd_id_pins[]
 */
static unsigned int lcd_id = 0;
static unsigned int lcd_orient = 0;

#define NUM_LCD_DETECT_PINS	ARRAY_SIZE(lcd_detect_pins)
#define	LCD_L_DD_17 		0
#define	LCD_L_DD_16 		1
#define	LCD_L_BIAS		2
#define	LCD_L_LCLK		3
#define	LCD_L_FCLK		4
#define	LCD_L_CS		5
#define	LCD_L_VSYNC		6

static unsigned int lcd_detect_pins[] = {
	MFP_L_DD_17,   	/* L_DD[17] - ORIENT */
	MFP_L_DD_16,   	/* L_DD[16] - LCDID[5] */
	MFP_L_BIAS, 	/* L_BIAS   - LCDID[4] */
	MFP_L_LCLK, 	/* L_LCLK   - LCDID[3] */
	MFP_L_FCLK, 	/* L_FCLK   - LCDID[2] */
	MFP_L_CS,   	/* L_CS     - LCDID[1] */
	MFP_L_VSYNC,   	/* L_V_SYNC - LCDID[0] */
#ifdef CONFIG_CPU_PXA320
/*
 * set the MFP_PIN_GPIO 14/15/17 to alternate function other than GPIO.
 * Otherwise, the input levels conflict with pins 14_2, 15_2, 17_2;
 */
	MFP_PIN_GPIO14,
	MFP_PIN_GPIO15,
	MFP_PIN_GPIO17,
#endif
};

static uint32_t lcd_detect_pins_mfpr[] = {
	/* AF0, DS 1X, Pull Neither, Edge Clear */
	0x8440, 0x8440, 0x8440, 0x8440, 0x8440, 0x8440, 0x8440,
#ifdef CONFIG_CPU_PXA320
	0xc442, /* Backlight, Pull-Up, AF2 */
	0x8445, /* AF5 */
	0x8445, /* AF5 */
#endif
};

static void pxafb_detect_lcd_panel(void)
{
	unsigned int id = 0, i;

	uint32_t lcd_detect_saved_mfpr[NUM_LCD_DETECT_PINS];

	/* save the original MFP settings */
	for (i = 0; i < NUM_LCD_DETECT_PINS; i++)
		lcd_detect_saved_mfpr[i] = MFP_REG(lcd_detect_pins[i]);

	for (i = 0; i < NUM_LCD_DETECT_PINS; i++)
		MFP_REG(lcd_detect_pins[i]) = lcd_detect_pins_mfpr[i];
	MFP_REG(lcd_detect_pins[i - 1]);

	/* set detect pins as gpio input */
	for (i = 0; i < 7;i++){
		pxa3xx_gpio_set_direction(lcd_detect_pins[i], GPIO_DIR_IN);
	}

	/* get LCD ID */
	id = 0;
	id = (pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_VSYNC]) 	<< 0) |
 	     (pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_CS]) 	<< 1) |
 	     (pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_FCLK])	<< 2) |
 	     (pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_LCLK]) 	<< 3) |
 	     (pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_BIAS]) 	<< 4) |
 	     (pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_DD_16]) 	<< 5);

	/* lcd id, flush out bit 1 */
	lcd_id = id & 0x3D;

	/* lcd orientation, portrait or landscape */
	lcd_orient = pxa3xx_gpio_get_level(lcd_detect_pins[LCD_L_DD_17]);

	for (i = 0; i < NUM_LCD_DETECT_PINS; i++)
		MFP_REG(lcd_detect_pins[i]) = lcd_detect_saved_mfpr[i];
	MFP_REG(lcd_detect_pins[i - 1]);
}

static void pxafb_config_lcd_panel(void)
{
	if (lcd_id & 0x20) {
#ifdef CONFIG_FB_PXA_LCD_QVGA
		set_pxa_fb_info(&sharp_ls037_qvga);
#endif
#ifdef CONFIG_FB_PXA_LCD_VGA
		set_pxa_fb_info(&sharp_ls037_vga);
#endif
	} else {
#ifdef CONFIG_FB_PXA_LCD_QVGA
		set_pxa_fb_info(&toshiba_ltm035a776c);
#endif
#ifdef CONFIG_FB_PXA_LCD_VGA
		set_pxa_fb_info(&toshiba_ltm04c380k);
#endif
	}
}

static struct pxa3xx_pin_config zylonite_lcd_pins[] = {
/*    description,   pin,           alt fn,       drive,   rdh, lpm, edge */
	PXA3xx_MFP_CFG("LCD LDD<0>",  MFP_L_DD_0,  MFP_L_DD_0_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<1>",  MFP_L_DD_1,  MFP_L_DD_1_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<2>",  MFP_L_DD_2,  MFP_L_DD_2_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<3>",  MFP_L_DD_3,  MFP_L_DD_3_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<4>",  MFP_L_DD_4,  MFP_L_DD_4_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<5>",  MFP_L_DD_5,  MFP_L_DD_5_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<6>",  MFP_L_DD_6,  MFP_L_DD_6_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<7>",  MFP_L_DD_7,  MFP_L_DD_7_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<8>",  MFP_L_DD_8,  MFP_L_DD_8_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<9>",  MFP_L_DD_9,  MFP_L_DD_9_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<10>", MFP_L_DD_10, MFP_L_DD_10_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<11>", MFP_L_DD_11, MFP_L_DD_11_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<12>", MFP_L_DD_12, MFP_L_DD_12_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<13>", MFP_L_DD_13, MFP_L_DD_13_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<14>", MFP_L_DD_14, MFP_L_DD_14_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<15>", MFP_L_DD_15, MFP_L_DD_15_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<16>", MFP_L_DD_16, MFP_L_DD_16_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<17>", MFP_L_DD_17, MFP_AF0,        MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD CS",      MFP_L_CS,    MFP_AF0,        MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD V_SYNC",  MFP_L_VSYNC, MFP_AF0,        MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD FCLK",    MFP_L_FCLK,  MFP_L_FCLK_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LCLK",    MFP_L_LCLK,  MFP_L_LCLK_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD PCLK",    MFP_L_PCLK,  MFP_L_PCLK_AF,  MFP_DS04X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD BIAS",    MFP_L_BIAS,  MFP_L_BIAS_AF,  MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),

	PXA3xx_MFP_CFG("LCD BKLIGHT", MFP_BACKLIGHT_PWM, MFP_AF0,  MFP_DS12X, 1,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

static struct pxa3xx_pin_config zylonite_18bpp_lcd_pins[] = {
/*    description,   pin,           alt fn,       drive,   rdh, lpm, edge */
	PXA3xx_MFP_CFG("LCD LDD<16>", MFP_L_DD_16, MFP_L_DD_16_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("LCD LDD<17>", MFP_L_DD_17, MFP_L_DD_17_AF, MFP_DS01X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

static struct pxa3xx_pin_config zylonite_mlcd_pins[] = {
/*   description,  pin,    alt fn,    drive,    rdh, lpm,       edge */
	PXA3xx_MFP_CFG("MLCD LDD<8>",  MFP_L_LP_DD_8,  MFP_L_LP_DD_8_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<9>",  MFP_L_LP_DD_9,  MFP_L_LP_DD_9_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<10>", MFP_L_LP_DD_10, MFP_L_LP_DD_10_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<11>", MFP_L_LP_DD_11, MFP_L_LP_DD_11_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<12>", MFP_L_LP_DD_12, MFP_L_LP_DD_12_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<13>", MFP_L_LP_DD_13, MFP_L_LP_DD_13_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<14>", MFP_L_LP_DD_14, MFP_L_LP_DD_14_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<15>", MFP_L_LP_DD_15, MFP_L_LP_DD_15_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<16>", MFP_L_LP_DD_16, MFP_L_LP_DD_16_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<17>", MFP_L_LP_DD_17, MFP_L_LP_DD_17_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<0>",  MFP_L_LP_DD_0,  MFP_L_LP_DD_0_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<1>",  MFP_L_LP_DD_1,  MFP_L_LP_DD_1_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<2>",  MFP_L_LP_DD_2,  MFP_L_LP_DD_2_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<3>",  MFP_L_LP_DD_3,  MFP_L_LP_DD_3_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<4>",  MFP_L_LP_DD_4,  MFP_L_LP_DD_4_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<5>",  MFP_L_LP_DD_5,  MFP_L_LP_DD_5_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<6>",  MFP_L_LP_DD_6,  MFP_L_LP_DD_6_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LDD<7>",  MFP_L_LP_DD_7,  MFP_L_LP_DD_7_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD FCLK",    MFP_L_LP_FCLK,  MFP_L_LP_FCLK_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD LCLK",    MFP_L_LP_LCLK,  MFP_L_LP_LCLK_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD PCLK",    MFP_L_LP_PCLK,  MFP_L_LP_PCLK_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MLCD BIAS",    MFP_L_LP_BIAS,  MFP_L_LP_BIAS_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),

	PXA3xx_MFP_CFG("LCD BKLIGHT",  MFP_BACKLIGHT_PWM, MFP_AF0,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

static struct pxa3xx_pin_config zylonite_lcd_smart_pins[] = {
/*          description,   pin,           alt fn,       drive,   rdh, lpm,              edge */
PXA3xx_MFP_CFG("LCD LDD<0>",  MFP_L_DD_0,  MFP_L_DD_0_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<1>",  MFP_L_DD_1,  MFP_L_DD_1_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<2>",  MFP_L_DD_2,  MFP_L_DD_2_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<3>",  MFP_L_DD_3,  MFP_L_DD_3_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<4>",  MFP_L_DD_4,  MFP_L_DD_4_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<5>",  MFP_L_DD_5,  MFP_L_DD_5_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<6>",  MFP_L_DD_6,  MFP_L_DD_6_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<7>",  MFP_L_DD_7,  MFP_L_DD_7_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<8>",  MFP_L_DD_8,  MFP_L_DD_8_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<9>",  MFP_L_DD_9,  MFP_L_DD_9_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<10>", MFP_L_DD_10, MFP_L_DD_10_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<11>", MFP_L_DD_11, MFP_L_DD_11_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<12>", MFP_L_DD_12, MFP_L_DD_12_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<13>", MFP_L_DD_13, MFP_L_DD_13_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<14>", MFP_L_DD_14, MFP_L_DD_14_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<15>", MFP_L_DD_15, MFP_L_DD_15_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<16>", MFP_L_DD_16, MFP_L_DD_16_AF, MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LDD<17>", MFP_L_DD_17, MFP_AF0,        MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD CS",      MFP_L_CS,    MFP_AF0,        MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD V_SYNC",  MFP_L_VSYNC, MFP_AF0,        MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD FCLK",    MFP_L_FCLK,  MFP_L_FCLK_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD LCLK",    MFP_L_LCLK,  MFP_L_LCLK_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD PCLK",    MFP_L_PCLK,  MFP_L_PCLK_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("LCD BIAS",    MFP_L_BIAS,  MFP_L_BIAS_AF,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),

PXA3xx_MFP_CFG("LCD BKLIGHT", MFP_BACKLIGHT_PWM, MFP_AF0,  MFP_DS01X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_lcd_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_lcd_pins, ARRAY_SIZE(zylonite_lcd_pins));
	if (lcd_id & 0x20) { /* OLED/VGA/QVGA panel */
		pxa3xx_gpio_set_direction(MFP_L_DD_17, GPIO_DIR_OUT);
		pxa3xx_gpio_set_direction(MFP_L_VSYNC, GPIO_DIR_OUT);

		if(lcd_id & 0x01) {  /* REV1.2 Lead_free panel */
			pxa3xx_gpio_set_level(MFP_L_DD_17, GPIO_LEVEL_LOW);
#ifdef CONFIG_FB_PXA_LCD_QVGA
			/* force L_V_SYNC (MODE) HIGH */
			pxa3xx_gpio_set_level(MFP_L_VSYNC, GPIO_LEVEL_HIGH);
			pxa3xx_mfp_set_lpm(MFP_L_VSYNC, MFP_LPM_PULL_HIGH);
#endif
#ifdef CONFIG_FB_PXA_LCD_VGA
			/* force L_V_SYNC (MODE) LOW */
			pxa3xx_gpio_set_level(MFP_L_VSYNC, GPIO_LEVEL_LOW);
			pxa3xx_mfp_set_lpm(MFP_L_VSYNC, MFP_LPM_PULL_LOW);
#endif
		} else { /* REV1.1 Lead panel */
			pxa3xx_mfp_set_afds(MFP_L_BIAS, MFP_AF0, MFP_DS01X);
			pxa3xx_gpio_set_direction(MFP_L_BIAS, GPIO_DIR_OUT);
			pxa3xx_gpio_set_level(MFP_L_BIAS, GPIO_LEVEL_LOW);

			pxa3xx_mfp_set_lpm(MFP_L_DD_17, MFP_LPM_PULL_LOW);
			pxa3xx_gpio_set_level(MFP_L_DD_17, GPIO_LEVEL_LOW);
#ifdef CONFIG_FB_PXA_LCD_QVGA
			/* force L_V_SYNC (MODE) LOW */
			pxa3xx_mfp_set_lpm(MFP_L_VSYNC, MFP_LPM_PULL_LOW);
			pxa3xx_gpio_set_level(MFP_L_VSYNC, GPIO_LEVEL_LOW);
#endif
#ifdef CONFIG_FB_PXA_LCD_VGA
			/* force L_V_SYNC (MODE) HIGH */
			pxa3xx_mfp_set_lpm(MFP_L_VSYNC, MFP_LPM_PULL_HIGH);
			pxa3xx_gpio_set_level(MFP_L_VSYNC, GPIO_LEVEL_HIGH);
#endif
		}
	}

#if defined(CONFIG_FB_PXA_LCD_18BPP) || defined(CONFIG_FB_PXA_LCD_19BPP)
	/* reconfig lcd mfp to 18bpp */
	pxa3xx_mfp_set_configs(zylonite_18bpp_lcd_pins, ARRAY_SIZE(zylonite_18bpp_lcd_pins));
#endif
}

void pxa3xx_enable_mlcd_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_mlcd_pins, ARRAY_SIZE(zylonite_mlcd_pins));
}

void pxa3xx_enable_lcd_smart_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_lcd_smart_pins, ARRAY_SIZE(zylonite_lcd_smart_pins));

	pxa3xx_gpio_set_direction(MFP_L_DD_17, GPIO_DIR_OUT);
	pxa3xx_gpio_set_direction(MFP_L_VSYNC, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_L_DD_17, GPIO_LEVEL_HIGH);
	pxa3xx_gpio_set_level(MFP_L_VSYNC, GPIO_LEVEL_LOW);
}
#endif

static struct pxa3xx_pin_config zylonite_eth_pins[] = {
	PXA3xx_MFP_CFG("ETH nCS3", MFP_DEBUG_ETH_CS_N_GPIO,
			MFP_DEBUG_ETH_CS_N_GPIO_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ETH INT",  MFP_DEBUG_ETH_INT_GPIO,
			MFP_DEBUG_ETH_INT_GPIO_AF,  MFP_DS03X, 0,
			MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_eth_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_eth_pins, ARRAY_SIZE(zylonite_eth_pins));
}

static struct pxa3xx_pin_config zylonite_ffuart_pins[] = {
	PXA3xx_MFP_CFG("FFUART RXD", MFP_FFRXD, MFP_FFRXD_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("FFUART TXD", MFP_FFTXD, MFP_FFTXD_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("FFUART CTS", MFP_FFCTS, MFP_FFCTS_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("FFUART RTS", MFP_FFRTS, MFP_FFRTS_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
#ifndef CONFIG_CPU_PXA310
	PXA3xx_MFP_CFG("FFUART DCD", MFP_FFDCD, MFP_FFDCD_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("FFUART DSR", MFP_FFDSR, MFP_FFDSR_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("FFUART RI",  MFP_FFRI,  MFP_FFRI_AF,  MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("FFUART DTR", MFP_FFDTR, MFP_FFDTR_AF, MFP_DS03X, 0,
			MFP_LPM_FLOAT, MFP_EDGE_NONE),
#endif
};

static struct pxa3xx_pin_config zylonite_btuart_pins[] = {
#ifdef CONFIG_PXA_MWB_12
	/* In MWB 1.2, RXD and TXD are swapped, RTS and CTS are swapped */
	PXA3xx_MFP_CFG("BTUART RTS", MFP_BT_RTS, MFP_AF3,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("BTUART RXD", MFP_BT_RXD, MFP_AF3,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("BTUART TXD", MFP_BT_TXD, MFP_AF3,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("BTUART CTS", MFP_BT_CTS, MFP_AF3,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
#else
	PXA3xx_MFP_CFG("BTUART RTS", MFP_BT_RTS, MFP_BT_RTS_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("BTUART RXD", MFP_BT_RXD, MFP_BT_RXD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("BTUART TXD", MFP_BT_TXD, MFP_BT_TXD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("BTUART CTS", MFP_BT_CTS, MFP_BT_CTS_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#endif
};

static struct pxa3xx_pin_config zylonite_stuart_pins[] = {
	PXA3xx_MFP_CFG("STUART TXD", MFP_STD_TXD, MFP_STD_TXD_AF, MFP_DS03X, 0,
			MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("STUART RXD", MFP_STD_RXD, MFP_STD_RXD_AF, MFP_DS03X, 0,
			MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_ffuart_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_ffuart_pins,
			ARRAY_SIZE(zylonite_ffuart_pins));
}

void pxa3xx_enable_btuart_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_btuart_pins,
			ARRAY_SIZE(zylonite_btuart_pins));
}

#ifdef CONFIG_PXA_MWB_12
void pxa3xx_enable_pxa_mwb_bt(void)
{
	pxa3xx_gpio_set_direction(MFP_MWB_BT_RESET, GPIO_DIR_OUT);
	pxa3xx_gpio_set_direction(MFP_MWB_BT_CLK_EN_N, GPIO_DIR_OUT);

	pxa3xx_gpio_set_level(MFP_MWB_BT_CLK_EN_N, GPIO_LEVEL_LOW);
	mdelay(10);
	pxa3xx_gpio_set_level(MFP_MWB_BT_RESET, GPIO_LEVEL_HIGH);
	mdelay(10);
	pxa3xx_gpio_set_level(MFP_MWB_BT_RESET, GPIO_LEVEL_LOW);
}
void pxa3xx_enable_pxa_mwb_wifi(void)
{
	pxa3xx_gpio_set_direction(MFP_MWB_WL_RESET, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_MWB_WL_RESET, GPIO_LEVEL_HIGH);
	mdelay(10);
	pxa3xx_gpio_set_level(MFP_MWB_WL_RESET, GPIO_LEVEL_LOW);
}
#endif

void pxa3xx_enable_stuart_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_stuart_pins,
			ARRAY_SIZE(zylonite_stuart_pins));
}

static struct pxa3xx_pin_config zylonite_dfc_pins[] = {
/*      description,   pin,      alt fn,      drive,  rdh, lpm,   edge */
	PXA3xx_MFP_CFG("DF INT RnB", MFP_DF_INT_RnB, MFP_DF_INT_RnB_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("DF nRE",     MFP_DF_nRE,     MFP_DF_nRE_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND nWE",     MFP_DF_nWE,     MFP_DF_nWE_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND CLE",     MFP_ND_CLE,     MFP_ND_CLE_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND ALE1",    MFP_DF_nADV1,   MFP_DF_nADV1_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
#ifdef CONFIG_CPU_PXA320
	PXA3xx_MFP_CFG("ND ALE2",    MFP_DF_nADV2,   MFP_DF_nADV2_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
#endif
	PXA3xx_MFP_CFG("DF nCS0",    MFP_DF_NCS0,    MFP_DF_NCS0_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("DF nCS1",    MFP_DF_NCS1,    MFP_DF_NCS1_AF,
			MFP_DS10X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<0>",   MFP_DF_IO_0,    MFP_DF_IO_0_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<1>",   MFP_DF_IO_1,    MFP_DF_IO_1_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<2>",   MFP_DF_IO_2,    MFP_DF_IO_2_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<3>",   MFP_DF_IO_3,    MFP_DF_IO_3_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<4>",   MFP_DF_IO_4,    MFP_DF_IO_4_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<5>",   MFP_DF_IO_5,    MFP_DF_IO_5_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<6>",   MFP_DF_IO_6,    MFP_DF_IO_6_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<7>",   MFP_DF_IO_7,    MFP_DF_IO_7_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<8>",   MFP_DF_IO_8,    MFP_DF_IO_8_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<9>",   MFP_DF_IO_9,    MFP_DF_IO_9_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<10>",  MFP_DF_IO_10,   MFP_DF_IO_10_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<11>",  MFP_DF_IO_11,   MFP_DF_IO_11_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<12>",  MFP_DF_IO_12,   MFP_DF_IO_12_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<13>",  MFP_DF_IO_13,   MFP_DF_IO_13_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<14>",  MFP_DF_IO_14,   MFP_DF_IO_14_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("ND IO<15>",  MFP_DF_IO_15,   MFP_DF_IO_15_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_dfc_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_dfc_pins, ARRAY_SIZE(zylonite_dfc_pins));
}

static struct pxa3xx_pin_config zylonite_ac97_pins[] = {
#ifdef CONFIG_AC97_EXTCLK
/* At this time, the clock of ac97 codec is instead by CLK_POUT. */
	PXA3xx_MFP_CFG("AC97 SYSCLK", MFP_AC97_SYSCLK,      MFP_AF0,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#else
	PXA3xx_MFP_CFG("AC97 SYSCLK", MFP_AC97_SYSCLK,      MFP_AC97_SYSCLK_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#endif
	PXA3xx_MFP_CFG("AC97 BITCLK", MFP_AC97_AC97_BITCLK, MFP_AC97_AC97_BITCLK_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("AC97 SDIN",   MFP_AC97_SDATA_IN_0,  MFP_AC97_SDATA_IN_0_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("AC97 SYNC",   MFP_AC97_SYNC,        MFP_AC97_SYNC_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("AC97 SDOUT",  MFP_AC97_SDATA_OUT,   MFP_AC97_SDATA_OUT_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("AC97 nACRST", MFP_AC97_nACRESET,    MFP_AC97_nACRESET_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_LOW,MFP_EDGE_NONE),
};

void pxa3xx_enable_ac97_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_ac97_pins, ARRAY_SIZE(zylonite_ac97_pins));
}

static struct pxa3xx_pin_config zylonite_i2c_pins[] = {
	PXA3xx_MFP_CFG("I2C SCL", MFP_SCL, MFP_SCL_AF, MFP_DS03X, 0,
			MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("I2C SDA", MFP_SDA, MFP_SDA_AF, MFP_DS03X, 0,
			MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_i2c_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_i2c_pins, ARRAY_SIZE(zylonite_i2c_pins));
}

static struct pxa3xx_pin_config zylonite_cif_pins[] = {
	PXA3xx_MFP_CFG("CIF DD<0>",  MFP_CIF_DD_0,      MFP_CIF_DD_0_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<1>",  MFP_CIF_DD_1,      MFP_CIF_DD_1_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<2>",  MFP_CIF_DD_2,      MFP_CIF_DD_2_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<3>",  MFP_CIF_DD_3,      MFP_CIF_DD_3_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<4>",  MFP_CIF_DD_4,      MFP_CIF_DD_4_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<5>",  MFP_CIF_DD_5,      MFP_CIF_DD_5_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<6>",  MFP_CIF_DD_6,      MFP_CIF_DD_6_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<7>",  MFP_CIF_DD_7,      MFP_CIF_DD_7_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<8>",  MFP_CIF_DD_8,      MFP_CIF_DD_8_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF DD<9>",  MFP_CIF_DD_9,      MFP_CIF_DD_9_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF PCLK",   MFP_CIF_PCLK,      MFP_CIF_PCLK_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF HSYNC",  MFP_CIF_HSYNC,     MFP_CIF_HSYNC_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF VSYNC",  MFP_CIF_VSYNC,     MFP_CIF_VSYNC_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#ifndef CONFIG_PXA3xx_GPIOEX
	PXA3xx_MFP_CFG("CIF HIPWDN", MFP_CIF_HI_PWDN_GPIO, MFP_CIF_HI_PWDN_GPIO_AF,
			MFP_DS04X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CIF LOPWDN", MFP_CIF_LO_PWDN_GPIO, MFP_CIF_LO_PWDN_GPIO_AF,
			MFP_DS04X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
#endif
	PXA3xx_MFP_CFG("CIF MCLK",   MFP_CIF_MCLK,      MFP_CIF_MCLK_AF,
			MFP_DS04X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_cif_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_cif_pins, ARRAY_SIZE(zylonite_cif_pins));
#ifdef CONFIG_PXA3xx_GPIOEX
	pxa3xx_gpio_set_direction(MFP_CIF_HI_PWDN_GPIO, GPIO_DIR_OUT);
	pxa3xx_gpio_set_direction(MFP_CIF_LO_PWDN_GPIO, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_CIF_HI_PWDN_GPIO, 1);
	pxa3xx_gpio_set_level(MFP_CIF_LO_PWDN_GPIO, 1);
#endif
}


static struct pxa3xx_pin_config zylonite_keyp_pins[] = {
	PXA3xx_MFP_CFG("KEYP DKIN0",  MFP_KP_DKIN_0,  MFP_KP_DKIN_0_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP DKIN1",  MFP_KP_DKIN_1,  MFP_KP_DKIN_1_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN0",  MFP_KP_MKIN_0,  MFP_KP_MKIN_0_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN1",  MFP_KP_MKIN_1,  MFP_KP_MKIN_1_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN2",  MFP_KP_MKIN_2,  MFP_KP_MKIN_2_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN3",  MFP_KP_MKIN_3,  MFP_KP_MKIN_3_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN4",  MFP_KP_MKIN_4,  MFP_KP_MKIN_4_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN5",  MFP_KP_MKIN_5,  MFP_KP_MKIN_5_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN6",  MFP_KP_MKIN_6,  MFP_KP_MKIN_6_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKIN7",  MFP_KP_MKIN_7,  MFP_KP_MKIN_7_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_BOTH),
	PXA3xx_MFP_CFG("KEYP MKOUT0", MFP_KP_MKOUT_0, MFP_KP_MKOUT_0_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT1", MFP_KP_MKOUT_1, MFP_KP_MKOUT_1_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT2", MFP_KP_MKOUT_2, MFP_KP_MKOUT_2_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT3", MFP_KP_MKOUT_3, MFP_KP_MKOUT_3_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT4", MFP_KP_MKOUT_4, MFP_KP_MKOUT_4_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT5", MFP_KP_MKOUT_5, MFP_KP_MKOUT_5_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT6", MFP_KP_MKOUT_6, MFP_KP_MKOUT_6_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("KEYP MKOUT7", MFP_KP_MKOUT_7, MFP_KP_MKOUT_7_AF,
			MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_keyp_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_keyp_pins,
			ARRAY_SIZE(zylonite_keyp_pins));
}

static struct pxa3xx_pin_config zylonite_mmc1_pins[] = {
#ifndef CONFIG_PXA3xx_GPIOEX
	PXA3xx_MFP_CFG("MMC1 CD0",  MFP_MMC_CD_0_GPIO,  MFP_MMC_CD_0_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 CD1",  MFP_MMC_CD_1_GPIO,  MFP_MMC_CD_1_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 WP0",  MFP_MMC_WP_0_N_GPIO, MFP_MMC_WP_0_N_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 WP1",  MFP_MMC_WP_1_N_GPIO, MFP_MMC_WP_1_N_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
#endif
	PXA3xx_MFP_CFG("MMC1 D0",   MFP_MMC_DAT0,         MFP_MMC_DAT0_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 D1",   MFP_MMC_DAT1,         MFP_MMC_DAT1_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 D2",   MFP_MMC_DAT2,         MFP_MMC_DAT2_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 D3",   MFP_MMC_DAT3,         MFP_MMC_DAT3_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 CLK",  MFP_MMC_CLK,          MFP_MMC_CLK_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 CMD0", MFP_MMC_CMD_0,        MFP_AF0,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC1 CMD1", MFP_MMC_CMD_1,        MFP_AF0,
			MFP_DS03X, 0, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
#ifdef CONFIG_CPU_PXA320
	PXA3xx_MFP_CFG("MMC1 RSVD", MFP_PIN_GPIO1_2,     MFP_AF6,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#endif
};

static struct pxa3xx_pin_config zylonite_mmc2_pins[] = {
	PXA3xx_MFP_CFG("MMC2 D0",  MFP_MMC2_DAT0,     MFP_MMC2_DAT0_AF,
			MFP_DS08X, 0, 0x10|MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC2 D1",  MFP_MMC2_DAT1,     MFP_MMC2_DAT1_AF,
			MFP_DS08X, 0, 0x10|MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC2 D2",  MFP_MMC2_DAT2_CS0, MFP_MMC2_DAT2_CS0_AF,
			MFP_DS08X, 0, 0x10|MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC2 D3",  MFP_MMC2_DAT3_CS1, MFP_MMC2_DAT3_CS1_AF,
			MFP_DS08X, 0, 0x10|MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC2 CLK", MFP_MMC2_CLK,      MFP_MMC2_CLK_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("MMC2 CMD", MFP_MMC2_CMD,      MFP_MMC2_CMD_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
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
	pxa3xx_mfp_set_configs(zylonite_mmc1_pins,
			ARRAY_SIZE(zylonite_mmc1_pins));
	/* set direction of CD/WP to IN */
	pxa3xx_gpio_set_direction(MFP_MMC_CD_0_GPIO, GPIO_DIR_IN);
	pxa3xx_gpio_set_direction(MFP_MMC_WP_0_N_GPIO, GPIO_DIR_IN);
}

void pxa3xx_enable_mmc2_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_mmc2_pins,
			ARRAY_SIZE(zylonite_mmc2_pins));
}

#ifdef CONFIG_CPU_PXA310
void pxa3xx_enable_mmc3_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_mmc3_pins,
			ARRAY_SIZE(zylonite_mmc3_pins));
}
#endif

#ifdef CONFIG_WIFI_HOST_SLEEP
struct pxa3xx_pin_config zylonite_wifi_host_sleep_pins[] = {
	PXA3xx_MFP_CFG("WIFI WAKEUP HOST", MFP_WIFI_WAKEUP_HOST, MFP_WIFI_WAKEUP_HOST_AF,
			MFP_DS03X, 1, 0x10 | MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("HOST WAKEUP WIFI", MFP_HOST_WAKEUP_WIFI, MFP_HOST_WAKEUP_WIFI_AF,
			MFP_DS03X, 1, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
};

void pxa3xx_enable_wifi_host_sleep_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_wifi_host_sleep_pins,
			ARRAY_SIZE(zylonite_wifi_host_sleep_pins));

	pxa3xx_gpio_set_direction(MFP_HOST_WAKEUP_WIFI, GPIO_DIR_OUT);	
	pxa3xx_gpio_set_level(MFP_HOST_WAKEUP_WIFI, GPIO_LEVEL_HIGH);

	pxa3xx_mfp_set_edge(MFP_WIFI_WAKEUP_HOST, MFP_EDGE_FALL);
}

void pxa3xx_wifi_wakeup(int active)
{
	if (active)
		pxa3xx_gpio_set_level(MFP_HOST_WAKEUP_WIFI, GPIO_LEVEL_LOW);
	else
		pxa3xx_gpio_set_level(MFP_HOST_WAKEUP_WIFI, GPIO_LEVEL_HIGH);
}
#endif

#ifdef CONFIG_CPU_PXA320
struct pxa3xx_pin_config zylonite_true_ide_pins[] = {
	PXA3xx_MFP_CFG("PCCARD DREQ",   MFP_DREQ_GPIO,   MFP_DREQ_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD nPWAIT", MFP_NPWAIT_GPIO, MFP_NPWAIT_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,  MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD CFDC",   MFP_CFCD_GPIO,   MFP_CFCD_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,  MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD CFnIRQ", MFP_CFNIRQ_GPIO, MFP_CFNIRQ_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
};

void pxa3xx_enable_true_ide_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_true_ide_pins,
			ARRAY_SIZE(zylonite_true_ide_pins));
}

struct pxa3xx_pin_config zylonite_pccard_pins[] = {
	PXA3xx_MFP_CFG("PCCARD nPIOR",  MFP_NPIOR_GPIO,   MFP_NPIOR_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD nPIOW",  MFP_NPIOW_GPIO,   MFP_NPIOW_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD nPWAIT", MFP_NPWAIT_GPIO,  MFP_NPWAIT_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,  MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD nIOS16", MFP_NIOIS16_GPIO, MFP_NIOIS16_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,  MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD CFDC",   MFP_CFCD_GPIO,    MFP_CFCD_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,  MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("PCCARD CFnIRQ", MFP_CFNIRQ_GPIO,  MFP_CFNIRQ_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW,  MFP_EDGE_NONE),
};

void enable_pccard_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_pccard_pins,
			ARRAY_SIZE(zylonite_pccard_pins));
}

struct pxa3xx_pin_config zylonite_ide_pins[] = {
	PXA3xx_MFP_CFG("IDE DREQ",   MFP_DREQ_GPIO,   MFP_DREQ_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_HIGH, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("IDE nPWAIT", MFP_NPWAIT_GPIO, MFP_NPWAIT_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("IDE CFCD",   MFP_CFCD_GPIO,   MFP_CFCD_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT,MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("IDE CFnIRQ", MFP_CFNIRQ_GPIO, MFP_CFNIRQ_GPIO_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void enable_ide_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_ide_pins, ARRAY_SIZE(zylonite_ide_pins));
}
#endif /* CONFIG_CPU_PXA320 */

static struct pxa3xx_pin_config zylonite_ssp2_pins[] = {
	PXA3xx_MFP_CFG("SSP2 SCLK", MFP_SSP_2_CLK, MFP_SSP_2_CLK_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP2 SFRM", MFP_SSP_2_FRM, MFP_SSP_2_FRM_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP2 TXD",  MFP_SSP_2_TXD, MFP_SSP_2_TXD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP2 RXD",  MFP_SSP_2_RXD, MFP_SSP_2_RXD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

static struct pxa3xx_pin_config zylonite_ssp3_pins[] = {
	PXA3xx_MFP_CFG("SSP3 BITCLK", MFP_SSP_AUDIO_SCLK, MFP_SSP_AUDIO_SCLK_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP3 FRMCLK", MFP_SSP_AUDIO_FRM,  MFP_SSP_AUDIO_FRM_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP3 TXDCLK", MFP_SSP_AUDIO_TXD,  MFP_SSP_AUDIO_TXD_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP3 RXDCLK", MFP_SSP_AUDIO_RXD,  MFP_SSP_AUDIO_RXD_AF,
			MFP_DS08X, 0, MFP_LPM_PULL_HIGH,MFP_EDGE_NONE),
};

static struct pxa3xx_pin_config zylonite_ssp4_pins[] = {
	PXA3xx_MFP_CFG("SSP4 SCLK", MFP_SSP_4_CLK, MFP_SSP_4_CLK_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP4 SFRM", MFP_SSP_4_FRM, MFP_SSP_4_FRM_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP4 TXD",  MFP_SSP_4_TXD, MFP_SSP_4_TXD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SSP4 RXD",  MFP_SSP_4_RXD, MFP_SSP_4_RXD_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_ssp2_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_ssp2_pins,
			ARRAY_SIZE(zylonite_ssp2_pins));
}

void pxa3xx_enable_ssp3_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_ssp3_pins,
			ARRAY_SIZE(zylonite_ssp3_pins));
}

void pxa3xx_enable_ssp4_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_ssp4_pins,
			ARRAY_SIZE(zylonite_ssp4_pins));
}

static struct pxa3xx_pin_config zylonite_otg_pins[] = {
#ifdef CONFIG_CPU_PXA320
	PXA3xx_MFP_CFG("OTG SR", MFP_USB_OTG_SR, MFP_USB_OTG_SR_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("OTG EN", MFP_OTG_EN,     MFP_OTG_EN_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#endif
#if defined(CONFIG_CPU_PXA320) || defined(CONFIG_CPU_PXA300)
	PXA3xx_MFP_CFG("OTG ID", MFP_OTG_ID,     MFP_OTG_ID_AF,
			MFP_DS01X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
#endif
};

void pxa3xx_enable_otg_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_otg_pins, ARRAY_SIZE(zylonite_otg_pins));
}

static struct pxa3xx_pin_config zylonite_usbh_pins[] = {
#ifndef CONFIG_CPU_PXA310
	PXA3xx_MFP_CFG("USBH PWR", MFP_USBHPWR, MFP_USBHPWR_AF,
			MFP_DS03X, 1, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("USBH PEN", MFP_USBHPEN, MFP_USBHPEN_AF,
			MFP_DS03X, 1, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#endif
};

void pxa3xx_enable_usbh_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_usbh_pins,
			ARRAY_SIZE(zylonite_usbh_pins));
}

#ifdef CONFIG_USB
static int zylonite_ohci_init(struct device *dev)
{
	/* enable PWR & PEN pins */
	pxa3xx_enable_usbh_pins();
	
	return 0;
}

static struct pxaohci_platform_data zylonite_ohci_info = {
	.init		= zylonite_ohci_init,
	.exit		= NULL,
	.port_mode	= PMM_PERPORT_MODE,
};
#endif

#define U2D_MFP_DS	MFP_DS08X
static struct pxa3xx_pin_config zylonite_u2d_pins[] = {
#if defined(CONFIG_CPU_PXA320) || defined(CONFIG_CPU_PXA300)
PXA3xx_MFP_CFG("U2D UTM CLK",	MFP_U2D_UTM_CLK, 	MFP_U2D_UTM_CLK_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA0",  	MFP_U2D_DATA0,		MFP_U2D_DATA0_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA1",  	MFP_U2D_DATA1,		MFP_U2D_DATA1_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA2",  	MFP_U2D_DATA2, 		MFP_U2D_DATA2_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA3",  	MFP_U2D_DATA3, 		MFP_U2D_DATA3_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA4",  	MFP_U2D_DATA4, 		MFP_U2D_DATA4_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA5",  	MFP_U2D_DATA5, 		MFP_U2D_DATA5_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA6",  	MFP_U2D_DATA6, 		MFP_U2D_DATA6_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D DATA7",  	MFP_U2D_DATA7, 		MFP_U2D_DATA7_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D RESET",  	MFP_U2D_RESET, 		MFP_U2D_RESET_AF,
		U2D_MFP_DS, 0, MFP_LPM_DRIVE_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D XCVR SEL", 	MFP_U2D_XCVR_SELECT, 	MFP_U2D_XCVR_SELECT_AF,
	       	U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D TERM SEL", 	MFP_U2D_TERM_SELECT, 	MFP_U2D_TERM_SELECT_AF,
	       	U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D SUSPEND",  	MFP_U2D_SUSPENDM_X, 	MFP_U2D_SUSPENDM_X_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D LS0",  	MFP_U2D_LINESTATE0, 	MFP_U2D_LINESTATE0_AF,
		U2D_MFP_DS, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D LS1",		MFP_U2D_LINESTATE1, 	MFP_U2D_LINESTATE1_AF,
		U2D_MFP_DS, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D TXVALID",  	MFP_U2D_TXVALID,	MFP_U2D_TXVALID_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D TXREADY",  	MFP_U2D_TXREADY, 	MFP_U2D_TXREADY_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D RXREADY",  	MFP_U2D_RXVALID, 	MFP_U2D_RXVALID_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D RXACTIVE", 	MFP_U2D_RXACTIVE, 	MFP_U2D_RXACTIVE_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D RXERROR",  	MFP_U2D_RXERROR, 	MFP_U2D_RXERROR_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D OPMODE0",  	MFP_U2D_OPMODE0, 	MFP_U2D_OPMODE0_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("U2D OPMODE1",  	MFP_U2D_OPMODE1, 	MFP_U2D_OPMODE1_AF,
		U2D_MFP_DS, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
#elif defined(CONFIG_CPU_PXA310)
/* Please define MFP for ULPI U2D */
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
#endif
};

void pxa3xx_enable_u2d_pins(void)
{
#ifdef CONFIG_CPU_PXA300
	pxa3xx_gpio_set_direction(MFP_UTMI_SWITCH, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_UTMI_SWITCH, GPIO_LEVEL_HIGH);

	pxa3xx_gpio_set_direction(MFP_UTMI_TEST_EN, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_UTMI_TEST_EN, GPIO_LEVEL_LOW);
#endif

#ifdef CONFIG_CPU_PXA310
	pxa3xx_mfp_set_afds(MFP_PIN_ULPI_STP, 0, 0);
	pxa3xx_mfp_set_afds(MFP_PIN_ULPI_DIR, 0, 0);
	pxa3xx_mfp_set_afds(MFP_PIN_ULPI_NXT, 0, 0);
#else
	/* on MHN-L platform, set transceiver voltage supply to 2.8V;
	   if raise it to 3.2V as indicated in CY7C68000 spec, it 
           would cause many unnecessary suspend/resume interrupts 
           when disconnect the cable */
	pxa3xx_pmic_set_voltage(VCC_USB, 2800);
#endif

	pxa3xx_mfp_set_configs(zylonite_u2d_pins, ARRAY_SIZE(zylonite_u2d_pins));
}

#ifdef CONFIG_CPU_PXA320
struct pxa3xx_pin_config zylonite_1w_pins[] = {
	PXA3xx_MFP_CFG("1Wire", MFP_ONE_WIRE, MFP_ONE_WIRE_AF,
			MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

void pxa3xx_enable_1w_pins(void)
{
	pxa3xx_mfp_set_configs(zylonite_1w_pins, ARRAY_SIZE(zylonite_1w_pins));
}
#endif

#ifdef CONFIG_PXA3xx_GPIOEX
static struct pxa3xx_pin_config gpio_exp_pins[] = {
	PXA3xx_MFP_CFG("GPIO_EXP_0_N", MFP_GPIO_EXP_0_N, MFP_GPIO_EXP_0_N_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("GPIO_EXP_1_N", MFP_GPIO_EXP_1_N, MFP_GPIO_EXP_1_N_AF,
			MFP_DS03X, 0, MFP_LPM_PULL_LOW, MFP_EDGE_NONE),
};

void pxa3xx_enable_gpio_exp_pins(void)
{
	pxa3xx_mfp_set_configs(gpio_exp_pins, ARRAY_SIZE(gpio_exp_pins));
	pxa3xx_gpio_set_direction(MFP_GPIO_EXP_0_N, GPIO_DIR_IN);
	pxa3xx_gpio_set_direction(MFP_GPIO_EXP_1_N, GPIO_DIR_IN);
}
#endif

/* if micco is detected, return 1.
 * Otherwise, return 0. At this time, we believe other PMIC is installed.
 * It should be invoked before I2C driver enabled.
 */
static int detect_micco(void)
{
	int micco_flag, tmp;

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

static void __init zylonite_init(void)
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
	 * Note: We depend that the bootloader set
	 * the correct value to MSC register for SMC91x.
	 */
	platform_device_register(&smc91x_device);

#ifdef CONFIG_KEYBOARD_PXA3xx
	pxa_set_keypad_info(&zylonite_keypad_info);
#endif

#ifdef CONFIG_FB_PXA
	pxafb_detect_lcd_panel();
	pxafb_config_lcd_panel();
#endif

#ifdef CONFIG_PXA_IRDA
	pxa_set_ficp_info(&zylonite_ficp_platform_data);
#endif

	pxa3xx_enable_eth_pins();
	pxa3xx_enable_i2c_pins();
#ifndef CONFIG_MMC_OPEN_SOURCE
	pxa3xx_enable_mmc1_pins();

#ifdef CONFIG_MMC2
	pxa3xx_enable_mmc2_pins();
#endif
#else
	pxa_set_mci_info(&zylonite_mci_platform_data);
#endif

#ifdef CONFIG_MMC3
	pxa3xx_enable_mmc3_pins();
#endif

#ifdef CONFIG_WIFI_HOST_SLEEP
	pxa3xx_enable_wifi_host_sleep_pins();
#endif
	pxa3xx_enable_ac97_pins();
	pxa3xx_enable_keyp_pins();

	pxa3xx_enable_ffuart_pins();
#ifdef CONFIG_CPU_PXA320
	pxa3xx_enable_1w_pins();
#endif

#ifdef CONFIG_PXA3xx_GPIOEX
       pxa3xx_enable_gpio_exp_pins();
#endif

#ifdef CONFIG_USB
	pxa_set_ohci_info(&zylonite_ohci_info);
#endif
}

#ifdef CONFIG_DISCONTIGMEM
static void __init
fixup_zylonite(struct machine_desc *desc, struct tag *tags,
                char **cmdline, struct meminfo *mi)
{
/*	int nid;
	mi->nr_banks = NR_NODES;
	for (nid=0; nid < mi->nr_banks; nid++)
		SET_NODE(mi, nid);
*/
}
#endif

MACHINE_START(ZYLONITE, "Intel PXA3xx Development Platform (aka Zylonite)")
#ifdef CONFIG_CPU_PXA310
	.phys_io        = 0x40000000,
	.boot_params    = 0xb0000100,
#else
	.phys_io        = 0x40000000,
	.boot_params    = 0x80000100,
#endif
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
#ifdef CONFIG_DISCONTIGMEM
        .fixup          = fixup_zylonite,
#endif
	.map_io         = pxa_map_io,
	.init_irq       = zylonite_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = zylonite_init,
MACHINE_END

EXPORT_SYMBOL_GPL(pxa3xx_enable_eth_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_i2c_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_dfc_pins);
#ifdef CONFIG_FB_PXA
EXPORT_SYMBOL_GPL(pxa3xx_enable_lcd_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_mlcd_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_lcd_smart_pins);
#endif
EXPORT_SYMBOL_GPL(pxa3xx_enable_cif_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_keyp_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ac97_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_mmc1_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_mmc2_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ffuart_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_btuart_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_stuart_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_u2d_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_otg_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp3_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_usbh_pins);
#ifdef CONFIG_WIFI_HOST_SLEEP
EXPORT_SYMBOL_GPL(pxa3xx_enable_wifi_host_sleep_pins);
EXPORT_SYMBOL_GPL(pxa3xx_wifi_wakeup);
#endif
#ifdef CONFIG_PXA_MWB_12
EXPORT_SYMBOL(pxa3xx_enable_pxa_mwb_bt);
EXPORT_SYMBOL(pxa3xx_enable_pxa_mwb_wifi);
#endif
