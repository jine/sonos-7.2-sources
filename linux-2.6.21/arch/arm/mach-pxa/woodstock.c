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

#undef LCDTEST
#undef MOTIONSENSOR
#include <linux/init.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fb.h>
#include <linux/root_dev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/backlight.h>

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

static void __init woodstock_init_irq(void)
{
	pxa_init_irq();
}

struct pxa3xx_pin_config woodstock_ffuart_pins[] = {
PXA3xx_MFP_CFG("FFUART RXD", MFP_FFRXD, MFP_FFRXD_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("FFUART TXD", MFP_FFTXD, MFP_FFTXD_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

void pxa3xx_enable_ffuart_pins(void)
{
	pxa3xx_mfp_set_configs(woodstock_ffuart_pins, ARRAY_SIZE(woodstock_ffuart_pins));
}

struct pxa3xx_pin_config woodstock_i2c_pins[] = {
PXA3xx_MFP_CFG("I2C SCL", MFP_SCL, MFP_AF1, MFP_DS03X, 1, MFP_LPM_FLOAT, MFP_EDGE_NONE),
PXA3xx_MFP_CFG("I2C SDA", MFP_SDA, MFP_AF1, MFP_DS03X, 1, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

void pxa3xx_enable_i2c_pins(void)
{
	pxa3xx_mfp_set_configs(woodstock_i2c_pins, ARRAY_SIZE(woodstock_i2c_pins));
}

#if 0
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
#endif /*0*/

#define SETCLK(a) do { pxa3xx_gpio_set_level(42,a); } while(0)
#define SETMOSI(a) do { pxa3xx_gpio_set_level(41,a); } while(0)
#define SETCS(a) do { pxa3xx_gpio_set_level(40,a); } while(0)
#define GETMISO() (pxa3xx_gpio_get_level(39))
#define CLKDELAY() do { ndelay(100); } while(0)
#define CSDELAY() do { ndelay(100); } while(0)

static inline void woodstock_bbspi_out8(unsigned char x)
{
    int i;
    for (i=7;i>=0;i--) {
        // if ((x&(1<<i))) printk("1"); else printk("0");
        SETMOSI((x&(1<<i))?1:0);
        SETCLK(0);
        CLKDELAY();
        SETCLK(1);
        CLKDELAY();
    }
}

static inline unsigned char woodstock_bbspi_in8(void)
{
    int i;
    unsigned char x=0;
    for (i=7;i>=0;i--) {
        SETCLK(0);
        CLKDELAY();
        SETCLK(1);
        CLKDELAY();
        if (GETMISO()) x|=(1<<i);
    }
    return x;
}

void woodstock_lcdspi_write(unsigned char addr,unsigned char val)
{
    // printk("S");
    SETCS(0);
    CSDELAY();
    woodstock_bbspi_out8(0x70);
    woodstock_bbspi_out8(addr);
    // printk("D");
    SETCS(1);
    CSDELAY();
    // printk("S");
    SETCS(0);
    CSDELAY();
    woodstock_bbspi_out8(0x72);
    woodstock_bbspi_out8(val);
    SETCS(1);
    // printk("D");
    CSDELAY();
    // printk("\n");
}

unsigned char woodstock_lcdspi_read(unsigned char addr)
{
    unsigned char x;
    SETCS(0);
    CSDELAY();
    woodstock_bbspi_out8(0x70);
    woodstock_bbspi_out8(addr);
    SETCS(1);
    CSDELAY();
    SETCS(0);
    CSDELAY();
    woodstock_bbspi_out8(0x73);
    x=woodstock_bbspi_in8();
    SETCS(1);
    CSDELAY();
    return x;
}

#define LCD_DS MFP_DS12X

#ifdef LCDTEST
#define XAF MFP_AF0
#else
#define XAF MFP_AF1
#endif

struct pxa3xx_pin_config lcd_ctl_config[] = {
        PXA3xx_MFP_CFG("LCDLIGHTPWM",MFP_PIN_GPIO17,MFP_PIN_GPIO17_AF_PWM0_OUT,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("LCDRST",MFP_PIN_GPIO76,MFP_AF0,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("LCDSPIMISO",MFP_PIN_GPIO39,MFP_AF0,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("LCDSPICS",MFP_PIN_GPIO40,MFP_AF0,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("LCDSPIMOSI",MFP_PIN_GPIO41,MFP_AF0,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("LCDSPICLK",MFP_PIN_GPIO42,MFP_AF0,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
};

struct pxa3xx_pin_config lcd_config[] = {
        PXA3xx_MFP_CFG("BLUE1",MFP_PIN_GPIO54,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("BLUE2",MFP_PIN_GPIO55,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("BLUE3",MFP_PIN_GPIO56,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("BLUE4",MFP_PIN_GPIO57,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("BLUE5",MFP_PIN_GPIO58,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("GREEN0",MFP_PIN_GPIO59,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("GREEN1",MFP_PIN_GPIO60,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("GREEN2",MFP_PIN_GPIO61,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("GREEN3",MFP_PIN_GPIO62,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("GREEN4",MFP_PIN_GPIO63,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("GREEN5",MFP_PIN_GPIO64,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("RED1",MFP_PIN_GPIO65,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("RED2",MFP_PIN_GPIO66,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("RED3",MFP_PIN_GPIO67,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("RED4",MFP_PIN_GPIO68,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("RED5",MFP_PIN_GPIO69,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("VSYNC",MFP_PIN_GPIO72,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("HSYNC",MFP_PIN_GPIO73,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("PCLK",MFP_PIN_GPIO74,XAF,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("DE",MFP_PIN_GPIO75,MFP_AF0,LCD_DS,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
};

void woodstock_lcd_power(int on);
void woodstock_backlight_power(int on);

#ifdef LCDTEST
#define PCLKDELAY 25
#define HSYNCPERIOD 48
#define HBPPERIOD 56
#define HFPPERIOD 56
#define VSYNCPERIOD 2
#define VBPPERIOD 4
#define VFPPERIOD 4

static inline void lcdout(unsigned short data,int de, int hsync, int vsync)
{
    unsigned int x1=0,x2=0;
    unsigned int y1,y2;
    x1 |= ((data&0x3ff)<<22);
    x2 |= ((data&0xfc00)>>10);
    if (de) x2 |= (1<<11);
    if (!hsync) x2 |= (1<<9);
    if (!vsync) x2 |= (1<<8);
    
    y1 = ((~x1)&0xffc00000);
    y2 = ((~x2)&0x00000f3f);

    GPSR(32)=x1;
    (void)GPSR(32);
    GPCR(32)=y1;
    (void)GPCR(32);
    GPSR(64)=x2;
    (void)GPSR(64);
    GPCR(64)=y2;
    (void)GPCR(64);
    ndelay(PCLKDELAY);
    GPSR(64)=0x400;
    (void)GPSR(64);
    ndelay(PCLKDELAY);
}

static inline void lcdline(void)
{
    int x;
    for (x=0;x<HSYNCPERIOD;x++) lcdout(0,0,1,0);
    for (x=0;x<HBPPERIOD;x++) lcdout (0,0,0,0);
    for (x=0;x<80;x++) lcdout (0xffff,1,0,0);
    for (x=0;x<80;x++) lcdout (0xf800,1,0,0);
    for (x=0;x<80;x++) lcdout (0x07e0,1,0,0);
    for (x=0;x<80;x++) lcdout (0x001f,1,0,0);
    for (x=0;x<80;x++) lcdout (0x0000,1,0,0);
    for (x=0;x<80;x++) lcdout (0xffff,1,0,0);
    for (x=0;x<HFPPERIOD;x++) lcdout (0,0,0,0);
}

static inline void lcdnodataline(int vsync)
{
    int x;
    for (x=0;x<HSYNCPERIOD;x++) lcdout(0,0,1,vsync);
    for (x=0;x<(HBPPERIOD+480+HFPPERIOD);x++) lcdout(0,0,0,vsync);
}

static inline void lcdtest(void)
{
    int x,y;
    printk("LCD test\n");
    GSDR(32)=0xffc00000;
    (void)GSDR(32);
    GSDR(64)=0x00000f3f;
    (void)GSDR(64);
    for (y=0;y<10;y++) {
        for (x=0;x<VSYNCPERIOD;x++) lcdnodataline(1);
        for (x=0;x<VBPPERIOD;x++) lcdnodataline(0);
        for (x=0;x<640;x++) lcdline();
        for (x=0;x<VFPPERIOD;x++) lcdnodataline(0);
        printk(".");
    }
    printk("\n");
}
#endif /*LCDTEST*/

static struct pxafb_mach_info woodstock_vga __initdata = {
// XXX out of date XXX
// PCD pixclock   freq          refresh
//  26 125000.000   8000000.000  23.310
//  24 115384.615   8666666.666  25.252
//  22 105769.230   9454545.454  27.548
//  20  96153.846  10400000.000  30.303
//  18  86538.461  11555555.555  33.670
//  16  76923.076  13000000.000  37.878		8  7500000.000  21.853
//  14  67307.692  14857142.857  43.290		7  8571428.571  24.975
//  12  57692.307  17333333.333  50.505		6 10000000.000  29.137
//  10  48076.923  20800000.000  60.606		5 12000000.000  34.965
//   8  38461.538  26000000.000  75.757		4 15000000.000  43.706
//   6  28846.153  34666666.666 101.010		3 20000000.000  58.275
//   4  19230.769  52000000.000 151.515		2 30000000.000  87.412
//   2   9615.384 104000000.000 303.030		1 60000000.000 174.825
// XXX out of date XXX

#ifdef CONFIG_PXA3xx_D0CS
        .pixclock               = 76923, /* in picoseconds */
#else
	.pixclock		= 38462,
#endif
        .xres                   = 480,
        .yres                   = 640,
        .bpp                    = 16,
	/*
	 * Note that the total horizontal period must be at least long enough
	 * for the PSP/RSW/GSW/BSW pulses to complete, plus some slop time at
	 * the end of the BSW pulse.  Currently this means an absolute minimum
	 * of 556 clock cycles.
	 */
        .hsync_len              = 56,
        .left_margin            = 8,
        .right_margin           = 96,
        .vsync_len              = 3,
        .upper_margin           = 4,
        .lower_margin           = 4,
        .sync                   = 0,
        .lccr0                  = LCCR0_Act | (1<<27),
        .lccr3                  = LCCR3_PCP,
        .pxafb_backlight_power	= woodstock_backlight_power,
        .pxafb_lcd_power        = woodstock_lcd_power,
};

void pxa3xx_enable_lcd_ctl_pins(void)
{
    pxa3xx_mfp_set_configs(lcd_ctl_config, ARRAY_SIZE(lcd_ctl_config));

    /* assert panel reset */
    pxa3xx_gpio_set_level(76, GPIO_LEVEL_LOW);
    pxa3xx_gpio_set_direction(76, GPIO_DIR_OUT);
    mdelay(10);

    /* set up fake SPI interface */
    pxa3xx_gpio_set_level(40, GPIO_LEVEL_HIGH); /*CS is inactive high*/
    pxa3xx_gpio_set_level(41, GPIO_LEVEL_LOW);
    pxa3xx_gpio_set_level(42, GPIO_LEVEL_HIGH); /*CLK is inactive high*/
    pxa3xx_gpio_set_direction(39, GPIO_DIR_IN);
    pxa3xx_gpio_set_direction(40, GPIO_DIR_OUT);
    pxa3xx_gpio_set_direction(41, GPIO_DIR_OUT);
    pxa3xx_gpio_set_direction(42, GPIO_DIR_OUT);
}

void pxa3xx_enable_lcd_pins(void)
{
    pxa3xx_mfp_set_configs(lcd_config, ARRAY_SIZE(lcd_config));
    pxa3xx_gpio_set_level(72, GPIO_LEVEL_LOW);
    pxa3xx_gpio_set_level(73, GPIO_LEVEL_LOW);
    pxa3xx_gpio_set_level(75, GPIO_LEVEL_LOW);
    pxa3xx_gpio_set_direction(72, GPIO_DIR_OUT);
    pxa3xx_gpio_set_direction(73, GPIO_DIR_OUT);
    pxa3xx_gpio_set_direction(75, GPIO_DIR_OUT);
}

void woodstock_lcd_init(void)
{
    pxa3xx_enable_lcd_ctl_pins();

    /* clear panel reset */
    pxa3xx_gpio_set_level(76, GPIO_LEVEL_HIGH);
    mdelay(10);

// Software reset
    woodstock_lcdspi_write(0x03,0x01);

// leave powered off
//    woodstock_lcdspi_write(0x00,0x02);
    woodstock_lcdspi_write(0x00,0x0A); // LGD
    woodstock_lcdspi_write(0x02,0x01);

// display setting
    woodstock_lcdspi_write(0x01,0x10);
    woodstock_lcdspi_write(0x04,0x06);
    /*
     * The PXA3xx insists on changing the data pins on the opposite clock
     * edge from HSYNC/VSYNC changes.  The NEC chip wants to sample on
     * the opposite clock edge.  This presents a problem.  To work around
     * it, we don't use HSYNC/VSYNC; instead we use DE mode.
     */
    woodstock_lcdspi_write(0x05,0x14);
    woodstock_lcdspi_write(0x06,0x38);

// Gamma setting positive
    woodstock_lcdspi_write(0x20,0x20);
    woodstock_lcdspi_write(0x21,0x09);
    woodstock_lcdspi_write(0x22,0x18); //0F
    woodstock_lcdspi_write(0x23,0x10);
    woodstock_lcdspi_write(0x24,0x12); //13
    woodstock_lcdspi_write(0x25,0x1E); //19
    woodstock_lcdspi_write(0x26,0x1F);
    woodstock_lcdspi_write(0x27,0x02); //01

// Bias adjustment positive
    woodstock_lcdspi_write(0x28,0x05);
    woodstock_lcdspi_write(0x29,0x05);
    woodstock_lcdspi_write(0x2A,0x05);

// Gamma setting negative
    woodstock_lcdspi_write(0x2B,0x00);
    woodstock_lcdspi_write(0x2C,0x09);
    woodstock_lcdspi_write(0x2D,0x18); //0F
    woodstock_lcdspi_write(0x2E,0x10);
    woodstock_lcdspi_write(0x2F,0x12); //13
    woodstock_lcdspi_write(0x30,0x1E); //19
    woodstock_lcdspi_write(0x31,0x1F);
    woodstock_lcdspi_write(0x32,0x02); //01

// Bias adjustment negative
    woodstock_lcdspi_write(0x33,0x05);
    woodstock_lcdspi_write(0x34,0x05);
    woodstock_lcdspi_write(0x35,0x05);

// Bias adjustment source amp
    woodstock_lcdspi_write(0x37,0x04);
    woodstock_lcdspi_write(0x3A,0x04);

// other setting
    woodstock_lcdspi_write(0x50,0x0C); //04

// timing setting
    woodstock_lcdspi_write(0x53,0x42);
    woodstock_lcdspi_write(0x54,0x42);
    woodstock_lcdspi_write(0x55,0x49);
    woodstock_lcdspi_write(0x56,0x14);
    woodstock_lcdspi_write(0x59,0x09); //0B
    woodstock_lcdspi_write(0x5A,0x08);
    woodstock_lcdspi_write(0x5B,0x0B);
    woodstock_lcdspi_write(0x5C,0x08);
    woodstock_lcdspi_write(0x5D,0x00);
    woodstock_lcdspi_write(0x5E,0x1C);
    woodstock_lcdspi_write(0x5F,0x23);
    woodstock_lcdspi_write(0x62,0x45);
    woodstock_lcdspi_write(0x63,0x23);
    woodstock_lcdspi_write(0x66,0x6E);
    woodstock_lcdspi_write(0x67,0x23);
    woodstock_lcdspi_write(0x6A,0x02);
    woodstock_lcdspi_write(0x70,0x00);
    woodstock_lcdspi_write(0x71,0x00);
    woodstock_lcdspi_write(0x74,0x00);
    woodstock_lcdspi_write(0x75,0xC1);
    woodstock_lcdspi_write(0x76,0xC1);
    woodstock_lcdspi_write(0x79,0x04);

// Power on setting
//    woodstock_lcdspi_write(0x10,0xAF);
    woodstock_lcdspi_write(0x11,0x03);
    woodstock_lcdspi_write(0x12,0x60);
    woodstock_lcdspi_write(0x13,0x55);
    woodstock_lcdspi_write(0x14,0x00);
//    woodstock_lcdspi_write(0x15,0x70);
//    woodstock_lcdspi_write(0x16,0x3B);
//    woodstock_lcdspi_write(0x17,0x2E); //2B
//    woodstock_lcdspi_write(0x18,0x2E); //2B
    woodstock_lcdspi_write(0x16,0x35); // LGD
    woodstock_lcdspi_write(0x17,0x27); // LGD
    woodstock_lcdspi_write(0x18,0x27); // LGD
    woodstock_lcdspi_write(0x19,0x02);
    woodstock_lcdspi_write(0x1A,0x02);
    woodstock_lcdspi_write(0x1B,0xA0);
    woodstock_lcdspi_write(0x1C,0x00);
    woodstock_lcdspi_write(0x10,0xAE); // LGD
    woodstock_lcdspi_write(0x15,0x70); // LGD
    woodstock_lcdspi_write(0x10,0xAF); // LGD

#if 0
    for (x=0;x<=127;x++) {
        if (!(x&0xf)) printk("\n");
        printk("(%02x,%02x) ",x,woodstock_lcdspi_read(x));
    }
    printk("\n");
#endif
#ifdef LCDTEST
    lcdtest();
#endif
}

static struct platform_device woodstockbl_device = {
        .name           = "woodstock-bl",
        .id             = -1,
};

static struct platform_device woodstocklcd_device = {
	.name		= "woodstock-lcd",
	.id		= -1,
};

void woodstock_lcd_power(int on)
{
    if (on) {
	woodstock_lcdspi_write(0x02,0x00);
	/*
	 * According to section 7.2 of the NEC manual, the longest rise time
	 * for the power supplies, T8, is 15*256/DCCLK, where DCCLK=Fosc/32,
	 * with R20[DUPT]=00.  With R4[OSCADJ]=011, Fosc is 1.16MHz.  This
	 * comes out to about 106ms.  The actual value may depend on other
	 * factors such as operating voltage, though.  We could reduce the
	 * power on time a little by increasing OSCADJ -- however, this may
	 * take more power.  - mycroft, 20080808
	 */
	mdelay(106);
	woodstock_lcdspi_write(0x00,0x02);
	/*
	 * If we're to believe the timing diagrams from the NEC manual, the
	 * chip takes at least an entire frame time to even start driving the
	 * panel outputs to a normal state after telling it to come out of
	 * standby mode.  Depending on the state of R18, the entire array will
	 * either be driven to white, or will display a fading copy of the
	 * old contents, during this time.  Therefore, we leave the backlight
	 * off for at least two vertical refresh periods -- one to reach the
	 * beginning of the frame, and one for the chip to resync.  This delay
	 * is dependent on the refresh rate.  - mycroft, 20080808
	 */
	mdelay(75);
    } else {
	woodstock_lcdspi_write(0x00,0x0A);
	woodstock_lcdspi_write(0x02,0x01);
	/*
	 * Give the standby logic time to discharge the panel.
	 */
	mdelay(80);
    }
}

void woodstock_backlight_power(int on)
{
	struct backlight_device *bd = platform_get_drvdata(&woodstockbl_device);
	if (bd) {
		bd->props.power = on ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
		if (on)
			bd->props.brightness = 148;
		backlight_update_status(bd);
	}
}

struct pxa3xx_pin_config mmc_config[]={
	PXA3xx_MFP_CFG("D0",  MFP_MMC_DAT0,  MFP_MMC_DAT0_AF,  MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("D1",  MFP_MMC_DAT1,  MFP_MMC_DAT1_AF,  MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("D2",  MFP_MMC_DAT2,  MFP_MMC_DAT2_AF,  MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("D3",  MFP_MMC_DAT3,  MFP_MMC_DAT3_AF,  MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("CLK", MFP_MMC_CLK,   MFP_MMC_CLK_AF,   MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("CMD", MFP_MMC_CMD_0, MFP_MMC_CMD_0_AF, MFP_DS03X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config keypad_config[]={
	PXA3xx_MFP_CFG("MUTE", MFP_KP_DKIN_0, MFP_KP_DKIN_0_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("UP",   MFP_KP_DKIN_1, MFP_KP_DKIN_1_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("DOWN", MFP_KP_DKIN_2, MFP_KP_DKIN_2_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_NONE),
        PXA3xx_MFP_CFG("ZONE", MFP_KP_DKIN_3, MFP_KP_DKIN_3_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config keypad_sleep_config[]={
        PXA3xx_MFP_CFG("MUTE", MFP_KP_DKIN_0, MFP_KP_DKIN_0_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_BOTH),
        PXA3xx_MFP_CFG("UP",   MFP_KP_DKIN_1, MFP_KP_DKIN_1_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_BOTH),
        PXA3xx_MFP_CFG("DOWN", MFP_KP_DKIN_2, MFP_KP_DKIN_2_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_BOTH),
        PXA3xx_MFP_CFG("ZONE", MFP_KP_DKIN_3, MFP_KP_DKIN_3_AF, MFP_DS03X, 0, MFP_LPM_PULL_LOW|0x10, MFP_EDGE_BOTH),
};


struct pxa3xx_pin_config lightsensor_config[] = {
	PXA3xx_MFP_CFG("LIGHT_SENS_EN",MFP_PIN_GPIO105,MFP_AF0,MFP_DS03X,1,MFP_LPM_DRIVE_HIGH,MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("WIFI_NTC_EN",MFP_PIN_GPIO106,MFP_AF0,MFP_DS03X,1,MFP_LPM_DRIVE_LOW,MFP_EDGE_NONE),
};

struct pxa3xx_pin_config ssp3_config[] = {
	PXA3xx_MFP_CFG("SCLK3",MFP_PIN_GPIO91,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("SFRM3",MFP_PIN_GPIO92,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
	PXA3xx_MFP_CFG("TXD3",MFP_PIN_GPIO93,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
};

struct pxa3xx_pin_config motionsensor_boot_config[]={
    PXA3xx_MFP_CFG("CHANGE",MFP_PIN_GPIO84,MFP_AF0,MFP_DS01X,0,
            MFP_LPM_FLOAT,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("CLK",MFP_PIN_GPIO85,MFP_AF1,MFP_DS03X,1,
            MFP_LPM_DRIVE_LOW,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("SS",MFP_PIN_GPIO86,MFP_AF1,MFP_DS03X,1,
            MFP_LPM_DRIVE_HIGH,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("MOSI",MFP_PIN_GPIO87,MFP_AF1,MFP_DS03X,1,
            MFP_LPM_DRIVE_LOW,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("MISO",MFP_PIN_GPIO88,MFP_AF1,MFP_DS03X,0,
            MFP_LPM_FLOAT,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("NOTMOTION",MFP_PIN_GPIO127,MFP_AF0,MFP_DS03X,0,
            MFP_LPM_PULL_LOW|0x10,MFP_EDGE_NONE),
};


struct pxa3xx_pin_config motionsensor_config[] = {
	PXA3xx_MFP_CFG("MMA7455L_INT1", MFP_PIN_GPIO83, MFP_AF0, MFP_DS01X, 0, MFP_LPM_FLOAT, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config motionsensor_sleep_config[] = {
	PXA3xx_MFP_CFG("MMA7455L_INT1", MFP_PIN_GPIO83, MFP_AF0, MFP_DS01X, 0, MFP_LPM_FLOAT, MFP_EDGE_RISE),
};

void pxa3xx_enable_ssp3_pins(void)
{
	pxa3xx_mfp_set_configs(ssp3_config, ARRAY_SIZE(ssp3_config));
}

void pxa3xx_enable_ssp4_pins(void)
{
	// xxx
}

void woodstock_keypad_config(void)
{
        pxa3xx_mfp_set_configs(keypad_config, ARRAY_SIZE(keypad_config));
}

void woodstock_keypad_sleep_config(void)
{
	pxa3xx_mfp_set_configs(keypad_sleep_config, ARRAY_SIZE(keypad_sleep_config));
}

void woodstock_motionsensor_config(void)
{
	pxa3xx_mfp_set_configs(motionsensor_config, ARRAY_SIZE(motionsensor_config));
}

void woodstock_motionsensor_sleep_config(void)
{
	pxa3xx_mfp_set_configs(motionsensor_sleep_config, ARRAY_SIZE(motionsensor_sleep_config));
}

static unsigned int woodstock_direct_key_map[] = {
    KEY_MUTE, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_Z
};

static struct pxa3xx_keypad_platform_data woodstock_keypad_info = {
        .enable_repeat_key      = 1,
	.enable_matrix_key	= 0,
        .enable_direct_key      = 1,
        .enable_rotary_key      = 0,
        .matrix_key_debounce    = 30,
        .direct_key_debounce    = 30,
        .matrix_key_rows        = 0,
        .matrix_key_cols        = 0,
        .direct_key_num         = 4,

        .matrix_key_map         = NULL,
        .matrix_key_map_size    = -1,

        .direct_key_map         = woodstock_direct_key_map,
        .direct_key_map_size    = ARRAY_SIZE(woodstock_direct_key_map),
};

static struct platform_device *devices[] __initdata = {
	&woodstockbl_device,
	&woodstocklcd_device,
};

static void __init woodstock_init(void)
{
	ARB_CNTRL1 = (1<<23);

	if (0/*detect_micco()*/) {
		pwri2c_flag = 1;
		/* enable FVE,PVE,TVE bit */
		PVCR = 0xe0500034;
	} else {
		pwri2c_flag = 0;
		/* disable FVE,PVE,TVE,FVC bit */
		PVCR &= 0x0fffffff;
	}

	pxa3xx_mfp_set_configs(mmc_config, ARRAY_SIZE(mmc_config));

	pxa3xx_mfp_set_configs(lightsensor_config, ARRAY_SIZE(lightsensor_config));
	pxa3xx_gpio_set_level(105, GPIO_LEVEL_HIGH); // xxx hold always high because of long time constant of circuit
	pxa3xx_gpio_set_direction(105, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(106, GPIO_LEVEL_LOW);
	pxa3xx_gpio_set_direction(106, GPIO_DIR_OUT);
	printk("Enabled light sensor and wifi NTC pins\n");

	pxa3xx_mfp_set_configs(motionsensor_boot_config, ARRAY_SIZE(motionsensor_boot_config));
	pxa3xx_mfp_set_configs(motionsensor_config, ARRAY_SIZE(motionsensor_config));
	pxa3xx_gpio_set_direction(127, GPIO_DIR_IN);
	printk("Enabled motion sensor pins\n");

        woodstock_lcd_init();
        set_pxa_fb_info(&woodstock_vga);
        printk("Performed Woodstock LCD initialization (w/ the LGD patch 6/30/2009)\n");

#ifdef CONFIG_KEYBOARD_PXA3xx
	pxa3xx_mfp_set_configs(keypad_config, ARRAY_SIZE(keypad_config));
	pxa_set_keypad_info(&woodstock_keypad_info);
#endif

#ifdef CONFIG_I2C_PXA
	pxa3xx_enable_i2c_pins();
#endif

	pxa3xx_enable_ffuart_pins();

#if 0
	pxa3xx_enable_ssp4_pins();
	pxa3xx_enable_ssp3_pins();
#endif

        platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc woodstock_io_desc[] __initdata = {
};

static void __init woodstock_map_io(void)
{
	pxa_map_io();
	iotable_init(woodstock_io_desc, ARRAY_SIZE(woodstock_io_desc));
}

MACHINE_START(WOODSTOCK, "Sonos Woodstock Alpha 2")
#ifdef CONFIG_CPU_PXA310
	.phys_io	= 0x40000000,
	.boot_params	= 0xb0000100,
#else
	.phys_io        = 0x40000000,
	.boot_params    = 0x80000100,
#endif
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = woodstock_map_io,
	.init_irq       = woodstock_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = woodstock_init,
MACHINE_END

//EXPORT_SYMBOL_GPL(pxa3xx_enable_i2c_pins);
EXPORT_SYMBOL_GPL(pxa3xx_enable_ffuart_pins);
//EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp2_pins);
//EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp3_pins);
//EXPORT_SYMBOL_GPL(pxa3xx_enable_ssp4_pins);
