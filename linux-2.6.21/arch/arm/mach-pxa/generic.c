/*
 *  linux/arch/arm/mach-pxa/generic.c
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 * Code common to all PXA machines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Since this file should be linked before any other machine specific file,
 * the __initcall() here will be executed first.  This serves as default
 * initialization stuff for PXA machines which can be overridden later if
 * need be.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/pm.h>
#include <linux/string.h>

#include <linux/sched.h>
#include <asm/cnt32_to_63.h>
#include <asm/div64.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/udc.h>
#include <asm/arch/u2d.h>
#include <asm/arch/ohci.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mmc.h>
#include <asm/arch/irda.h>
#include <asm/arch/i2c.h>
#include <asm/arch/pxa3xx_gpio.h>

#include "generic.h"

/*
 * This is the PXA2xx sched_clock implementation. This has a resolution
 * of at least 308ns and a maximum value that depends on the value of
 * CLOCK_TICK_RATE.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to this function.
 */
unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(OSCR);
	/* Note: top bit ov v needs cleared unless multiplier is even. */

#if	CLOCK_TICK_RATE == 3686400
	/* 1E9 / 3686400 => 78125 / 288, max value = 32025597s (370 days). */
	/* The <<1 is used to get rid of tick.hi top bit */
	v *= 78125<<1;
	do_div(v, 288<<1);
#elif	CLOCK_TICK_RATE == 3250000
	/* 1E9 / 3250000 => 4000 / 13, max value = 709490156s (8211 days) */
	v *= 4000;
	do_div(v, 13);
#elif	CLOCK_TICK_RATE == 3249600
#warning "consider fixing sched_clock for your value of CLOCK_TICK_RATE"
	/* 1E9 / 3249600 => 625000 / 2031, max value = 4541295s (52 days) */
	v *= 625000;
	do_div(v, 2031);
#else
	/*
	 * 96-bit math to perform tick * NSEC_PER_SEC / CLOCK_TICK_RATE for
	 * any value of CLOCK_TICK_RATE. Max value is in the 80 thousand
	 * years range and truncation to unsigned long long limits it to
	 * sched_clock's max range of ~584 years.  This is nice but with
	 * higher computation cost.
	 */
	{
		union {
			unsigned long long val;
			struct { unsigned long lo, hi; };
		} x;
		unsigned long long y;

		x.val = v;
		x.hi &= 0x7fffffff;
		y = (unsigned long long)x.lo * NSEC_PER_SEC;
		x.lo = y;
		y = (y >> 32) + (unsigned long long)x.hi * NSEC_PER_SEC;
		x.hi = do_div(y, CLOCK_TICK_RATE);
		do_div(x.val, CLOCK_TICK_RATE);
		x.hi += y;
		v = x.val;
	}
#endif

	return v;
}

/*
 * Handy function to set GPIO alternate functions
 */

int pxa_gpio_mode(int gpio_mode)
{
	unsigned long flags;
	int gpio = gpio_mode & GPIO_MD_MASK_NR;
#ifndef CONFIG_PXA3xx
	int fn = (gpio_mode & GPIO_MD_MASK_FN) >> 8;
	int gafr;
#endif

	if (gpio > PXA_LAST_GPIO)
		return -EINVAL;

	local_irq_save(flags);
#ifndef CONFIG_PXA3xx
	if (gpio_mode & GPIO_DFLT_LOW)
		GPCR(gpio) = GPIO_bit(gpio);
	else if (gpio_mode & GPIO_DFLT_HIGH)
		GPSR(gpio) = GPIO_bit(gpio);
	if (gpio_mode & GPIO_MD_MASK_DIR)
		GPDR(gpio) |= GPIO_bit(gpio);
	else
		GPDR(gpio) &= ~GPIO_bit(gpio);
	gafr = GAFR(gpio) & ~(0x3 << (((gpio) & 0xf)*2));
	GAFR(gpio) = gafr |  (fn  << (((gpio) & 0xf)*2));
#else
	GCDR(gpio) &= (1 << (gpio & 0x1f));
#endif
	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(pxa_gpio_mode);

/*
 * Return GPIO level
 */
int pxa_gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

EXPORT_SYMBOL(pxa_gpio_get_value);

/*
 * Set output GPIO level
 */
void pxa_gpio_set_value(unsigned gpio, int value)
{
	__gpio_set_value(gpio, value);
}

EXPORT_SYMBOL(pxa_gpio_set_value);

/*
 * Routine to safely enable or disable a clock in the CKEN
 */
void pxa_set_cken(int clock, int enable)
{
	unsigned long flags;
	int hsio2_enable = 0;
	local_irq_save(flags);

#if defined(CONFIG_PXA3xx)
	switch (clock) {
		/*special case1:*/
		case CKEN_AC97:
			if (enable) {
				CKENA |= (0x1u << clock);
				if (CKEN_AC97 == clock) {
				/*
				 * REVISIT: Make out-clock for AC97 24.576MHz
				 * This is workaround for that AC97 clock is
				 * not correct after reset.
				 */
					AC97_DIV = 1625<<12 | 128;
				}
			} else {
				CKENA &= ~(0x1u << clock);
			}
			break;
			
		/*special case2:*/
		case CKEN_USB2:
			if (enable) {
				CKENA |= (0x1u << clock);
			} else {
				CKENA &= ~(0x1u << clock);
			}
		case CKEN_GRAPHICS:
			if (CKEN_GRAPHICS < 32) {
				if (enable) {
					CKENA |= (0x1u << clock);
				} else {
					CKENA &= ~(0x1u << clock);
				}
			} else {
				if (enable) {
					CKENB |= (0x1u << (clock - 32));
				} else {
					CKENB &= ~(0x1u << (clock - 32));
				}
			}
#ifdef CONFIG_CPU_PXA310
		case CKEN_MVED:
			if (enable) {
				CKENB |= (0x1u << (clock-32) );
			} else {
				CKENB &= ~(0x1u << (clock - 32));
			}
#endif			
		case CKEN_HSIO2:	
			if (enable)
				hsio2_enable++;
			if (CKENA & (0x1u << CKEN_USB2))
				hsio2_enable++;
#if (CKEN_GRAPHICS < 32)
			if (CKENA & (0x1u << CKEN_GRAPHICS))
				hsio2_enable++;
#else
			if (CKENB & (0x1u << (CKEN_GRAPHICS - 32)))
				hsio2_enable++;
#endif
#ifdef CONFIG_CPU_PXA310
			if (CKENB & (0x1u << (CKEN_MVED - 32)))
				hsio2_enable++;
#endif			

			if (hsio2_enable)
				CKENB |= (0x1u << (CKEN_HSIO2-32) );
			else
				CKENB &= ~(0x1u << (CKEN_HSIO2-32) );
			break;
						
		default:	/*normal case:*/
			if (clock < 32) {
				if (enable) {
					CKENA |= (0x1u << clock);
				} else {
					CKENA &= ~(0x1u << clock);
				}
			} else {
				if (enable) {
					CKENB |= (0x1u << (clock - 32) );
				} else {
					CKENB &= ~(0x1u << (clock - 32));
				}
			}
			break;
	}
#else
	if (enable)
		CKEN |= clock;
	else
		CKEN &= ~clock;

#endif
	local_irq_restore(flags);
}

EXPORT_SYMBOL(pxa_set_cken);

#ifndef CONFIG_PXA3xx
/*
 * Intel PXA2xx internal register mapping.
 *
 * Note 1: not all PXA2xx variants implement all those addresses.
 *
 * Note 2: virtual 0xfffe0000-0xffffffff is reserved for the vector table
 *         and cache flush area.
 */
static struct map_desc standard_io_desc[] __initdata = {
  	{	/* Devs */
		.virtual	=  0xf2000000,
		.pfn		= __phys_to_pfn(0x40000000),
		.length		= 0x02000000,
		.type		= MT_DEVICE
	}, {	/* LCD */
		.virtual	=  0xf4000000,
		.pfn		= __phys_to_pfn(0x44000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* Mem Ctl */
		.virtual	=  0xf6000000,
		.pfn		= __phys_to_pfn(0x48000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* USB host */
		.virtual	=  0xf8000000,
		.pfn		= __phys_to_pfn(0x4c000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* Camera */
		.virtual	=  0xfa000000,
		.pfn		= __phys_to_pfn(0x50000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* IMem ctl */
		.virtual	=  0xfe000000,
		.pfn		= __phys_to_pfn(0x58000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* UNCACHED_PHYS_0 */
		.virtual	= 0xff000000,
		.pfn		= __phys_to_pfn(0x00000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}
};
#else
/*
 * Intel PXA3xx internal register mapping.
 *
 * Note 1: not all variants implement all those addresses.
 *
 * Note 2: virtual 0xfffe0000-0xffffffff is reserved for the vector table
 *         and cache flush area.
 */
static struct map_desc standard_io_desc[] __initdata = {
	{	/* VLIO IO            */
		.virtual	= 0xf5000000,
		.pfn		= __phys_to_pfn(0x14000000),
		.length		= 0x01000000,
		.type		= MT_DEVICE
	}, {	/* devices            */
		.virtual	= 0xf6000000,
		.pfn		= __phys_to_pfn(0x40000000),
		.length		= 0x02000000,
		.type		= MT_DEVICE
	}, {	/* mmc2 & usim2	      */
		.virtual	= 0xf8000000,
		.pfn		= __phys_to_pfn(0x42000000),
		.length		= 0x00200000,
		.type		= MT_DEVICE
	}, {	/* nand               */
		.virtual	= 0xf8300000,
		.pfn		= __phys_to_pfn(0x43100000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* lcd                */
		.virtual	= 0xf8400000,
		.pfn		= __phys_to_pfn(0x44000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* mini-lcd           */
		.virtual	= 0xf8800000,
		.pfn		= __phys_to_pfn(0x46000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* dynamic mem ctl    */
		.virtual	= 0xf8d00000,
		.pfn		= __phys_to_pfn(0x48100000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* static memory ctl  */
		.virtual	= 0xf9000000,
		.pfn		= __phys_to_pfn(0x4a000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {
		/* usb host           */
		.virtual	= 0xf9400000,
		.pfn		= __phys_to_pfn(0x4c000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* camera             */
		.virtual	= 0xfa000000,
		.pfn		= __phys_to_pfn(0x50000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {	/* 2d-graphics & usb2 */
		.virtual	= 0xfa400000,
		.pfn		= __phys_to_pfn(0x54000000),
		.length		= 0x00200000,
		.type		= MT_DEVICE
	}, {	/* internal SRAM ctl  */
		.virtual	= 0xfa800000,
		.pfn		= __phys_to_pfn(0x58000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}, {    /* MMC Controller 3 */
		.virtual        = 0xfa900000,
		.pfn            = __phys_to_pfn(0x42500000),
		.length         = 0x00100000,
		.type           = MT_DEVICE
	},
};
#endif

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
	get_clk_frequency_khz(1);
}

struct pxaohci_platform_data pxa_ohci_info;

void __init pxa_set_ohci_info(struct pxaohci_platform_data *info)
{
	memcpy(&pxa_ohci_info, info, sizeof *info);
}

EXPORT_SYMBOL_GPL(pxa_set_ohci_info);

static u64 ohci_hcd_pxa_dmamask = 0xffffffffUL;

static struct resource pxa3xx_ohci_resources[] = {
	{
		.start= 0x4C000000,
		.end= 0x4C000fff,
		.flags= IORESOURCE_MEM,
	}, {
		.start= IRQ_USBH1,
		.end= IRQ_USBH1,
		.flags= IORESOURCE_IRQ,
	},
};

static struct platform_device ohci_hcd_pxa_device = {
	.name = "pxa3xx-ohci",
	.id = -1,
	.dev		= {
		.platform_data	= &pxa_ohci_info,
		.dma_mask = &ohci_hcd_pxa_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources = ARRAY_SIZE(pxa3xx_ohci_resources),
	.resource      = pxa3xx_ohci_resources,
};

#ifndef CONFIG_PXA3xx	/* to differentiate MMC between PXA variants */
static struct resource pxamci_resources[] = {
	[0] = {
		.start	= 0x41100000,
		.end	= 0x41100fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 pxamci_dmamask = 0xffffffffUL;

static struct platform_device pxamci_device = {
	.name		= "pxa2xx-mci",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxamci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxamci_resources),
	.resource	= pxamci_resources,
};

#else				/* PXA3xx */

static u64 pxa_mmc_controller_dmamask = 0xffffffffUL;

#ifndef CONFIG_MMC_OPEN_SOURCE	/* not open source */
static struct resource mmc1_resources[] = {
	{
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc1_device = {
	.name		= "mmc_controller",
	.id		= 0,
	.resource	= mmc1_resources,
	.num_resources	= ARRAY_SIZE(mmc1_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.coherent_dma_mask =	0xffffffff,
	},
};

#ifdef CONFIG_MMC2
static struct resource mmc2_resources[] = {
	{
		.start	= IRQ_MMC2,
		.end	= IRQ_MMC2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc2_device = {
	.name		= "mmc_controller",
	.id		= 1,
	.resource	= mmc2_resources,
	.num_resources	= ARRAY_SIZE(mmc2_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.coherent_dma_mask =	0xffffffff,
	},
};
#endif

#ifdef CONFIG_MMC3
static struct resource mmc3_resources[] = {
	{
		.start	= IRQ_MMC3,
		.end	= IRQ_MMC3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc3_device = {
	.name		= "mmc_controller",
	.id		= 2,
	.resource	= mmc3_resources,
	.num_resources	= ARRAY_SIZE(mmc3_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.coherent_dma_mask =	0xffffffff,
	},
};
#endif

#else	/* open source driver */

static struct resource mmc1_resources[] = {
	{
		.start	= 0x41100000,
		.end	= 0x41100fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_MMC,
		.end	= IRQ_MMC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc1_device = {
	.name		=	"pxa2xx-mci",
	.id		=	0,
	.resource	= mmc1_resources,
	.num_resources	= ARRAY_SIZE(mmc1_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.coherent_dma_mask =	0xffffffff,
	},
};

#ifdef CONFIG_MMC2
static struct resource mmc2_resources[] = {
	{
		.start	= 0x42000000,
		.end	= 0x42000fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_MMC2,
		.end	= IRQ_MMC2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmc2_device = {
	.name		=	"pxa2xx-mci",
	.id		=	1,
	.resource	= mmc2_resources,
	.num_resources	= ARRAY_SIZE(mmc2_resources),
	.dev		= {
		.dma_mask	   =	&pxa_mmc_controller_dmamask,
		.coherent_dma_mask =	0xffffffff,
	},
};
#endif /* MMC2 */

void __init pxa_set_mci_info(struct pxamci_platform_data *info)
{
#ifndef CONFIG_PXA3xx
	pxamci_device.dev.platform_data = info;
#else
	mmc1_device.dev.platform_data = info;
#ifdef CONFIG_MMC2
	mmc2_device.dev.platform_data = info;
#endif
#endif
}

EXPORT_SYMBOL_GPL(pxa_set_mci_info);

#endif /* open source */
#endif /* CPU_PXA_3xx */

#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA320)
static struct pxa2xx_udc_mach_info pxa_udc_info;

void __init pxa_set_udc_info(struct pxa2xx_udc_mach_info *info)
{
	memcpy(&pxa_udc_info, info, sizeof *info);
}

EXPORT_SYMBOL_GPL(pxa_set_udc_info);

static struct resource pxa2xx_udc_resources[] = {
	[0] = {
		.start	= 0x40600000,
		.end	= 0x4060ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USB,
		.end	= IRQ_USB,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 udc_dma_mask = ~(u32)0;

static struct platform_device udc_device = {
	.name		= "pxa3xx-udc",
	.id		= -1,
	.resource	= pxa2xx_udc_resources,
	.num_resources	= ARRAY_SIZE(pxa2xx_udc_resources),
	.dev		=  {
		.platform_data	= &pxa_udc_info,
		.dma_mask	= &udc_dma_mask,
#if defined(CONFIG_PXA3xx)
		.coherent_dma_mask = 0xffffffff,
#endif
	}
};

#endif

static struct resource pxafb_resources[] = {
	[0] = {
		.start	= 0x44000000,
		.end	= 0x4400ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_LCD,
		.end	= IRQ_LCD,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

static struct platform_device pxafb_device = {
	.name		= "pxa2xx-fb",
	.id		= -1,
	.dev		= {
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(pxafb_resources),
	.resource	= pxafb_resources,
};

void __init set_pxa_fb_info(struct pxafb_mach_info *info)
{
	pxafb_device.dev.platform_data = info;
}
EXPORT_SYMBOL_GPL(set_pxa_fb_info);

void __init set_pxa_fb_parent(struct device *parent_dev)
{
	pxafb_device.dev.parent = parent_dev;
}

static struct platform_device ffuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 0,
};
static struct platform_device btuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 1,
};
static struct platform_device stuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 2,
};
static struct platform_device hwuart_device = {
	.name		= "pxa2xx-uart",
	.id		= 3,
};

static struct resource i2c_resources[] = {
	{
		.start	= 0x40301680,
		.end	= 0x403016a3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_I2C,
		.end	= IRQ_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device i2c_device = {
	.name		= "pxa2xx-i2c",
	.id		= 0,
	.resource	= i2c_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
};

#ifdef CONFIG_MACH_WOODSTOCK
static struct platform_device woodstock_i2c_device = {
	.name 		= "woodstock-i2c",
	.id		= 0,
};
#endif

#ifdef CONFIG_PXA27x
static struct resource i2c_power_resources[] = {
	{
		.start	= 0x40f00180,
		.end	= 0x40f001a3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_PWRI2C,
		.end	= IRQ_PWRI2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device i2c_power_device = {
	.name		= "pxa2xx-i2c",
	.id		= 1,
	.resource	= i2c_power_resources,
	.num_resources	= ARRAY_SIZE(i2c_resources),
};
#endif

void __init pxa_set_i2c_info(struct i2c_pxa_platform_data *info)
{
	i2c_device.dev.platform_data = info;
}

static u64 pxaficp_dmamask = ~(u32)0;

static struct platform_device pxaficp_device = {
	.name		= "pxa2xx-ir",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxaficp_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
};

#if defined(CONFIG_PXA3xx)
static struct platform_device nand_device = {
	.name		= "pxa3xx-nand",
	.id		= -1,
};

static struct platform_device camera_device = {
	.name		= "pxa3xx-camera",
	.id		= -1,
};

static struct resource pxaac97_resources[] = {
        {
                .start = 0x40500000,
                .end = 0x40500fff,
                .flags = IORESOURCE_MEM,
        }, {
                .start  = IRQ_AC97,
                .end = IRQ_AC97,
                .flags = IORESOURCE_IRQ,
        },
};

static u64 pxa_ac97_dmamask = 0xffffffffUL;

static struct platform_device pxaac97_device = {
	.name           = "pxa2xx-ac97",
	.id             = -1,
	.dev            = {
                .dma_mask = &pxa_ac97_dmamask,
                .coherent_dma_mask = 0xffffffffUL,
	},
	.num_resources  = ARRAY_SIZE(pxaac97_resources),
	.resource       = pxaac97_resources,
};

#ifdef CONFIG_CPU_PXA320
static struct platform_device true_ide_device = {
        .name           = "trueide-microdisk",
        .id             = -1,
};
#endif

#ifdef CONFIG_PXA3xx_GPIOEX
static struct resource gpio_exp0_resources[] = {
	[0] = {
		.start	= IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_0_N)),
		.end	= IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_0_N)),
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= GPIO_EXP0_ADDRESS,
		.end	= GPIO_EXP0_ADDRESS,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start  = GPIO_EXP0_START,
		.end	= GPIO_EXP0_END,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device gpio_exp0_device = {
	.name           = "gpio-exp",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(gpio_exp0_resources),
	.resource       = gpio_exp0_resources,
};

static struct resource gpio_exp1_resources[] = {
	[0] = {
		.start	= IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_1_N)),
		.end	= IRQ_GPIO(MFP2GPIO(MFP_GPIO_EXP_1_N)),
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= GPIO_EXP1_ADDRESS,
		.end	= GPIO_EXP1_ADDRESS,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= GPIO_EXP1_START,
		.end	= GPIO_EXP1_END,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device gpio_exp1_device = {
	.name           = "gpio-exp",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(gpio_exp1_resources),
	.resource       = gpio_exp1_resources,
};
#endif

#ifdef CONFIG_PXA_MWB_12
static struct resource gpio_exp2_resources[] = {
	[0] = {
		.start  = GPIO_EXP2_ADDRESS,
		.end = GPIO_EXP2_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start  = GPIO_EXP2_START,
		.end = GPIO_EXP2_END,
		.flags = IORESOURCE_IO,
	},
};

static struct platform_device gpio_exp2_device = {
	.name           = "gpio-exp",
	.id             = 2,
	.num_resources  = ARRAY_SIZE(gpio_exp2_resources),
	.resource       = gpio_exp2_resources,
};
#endif

static struct platform_device pxa3xx_pmic_device = {
        .name 		= "pxa3xx_pmic",
        .id 		= -1,
};

static struct platform_device pxa3xx_fv_device = {
        .name 		= "pxa3xx_fv",
        .id 		= -1,
};

static struct resource w1_resources[] = {
	{
 		.start	= 0x41B00000,
		.end	= 0x41B00010,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device w1_device = {
	.name		= "1-wire-mp",
	.id		= 0,
	.resource	= w1_resources,
	.num_resources	= ARRAY_SIZE(w1_resources),
};

#ifdef CONFIG_USB_PXA3xx_OTG
static struct platform_device otg_device = {
	.name	=	"pxa3xx-otg",
	.id	=	-1,
};
#endif

static struct pxa3xx_u2d_mach_info pxa_u2d_info;

void __init pxa_set_u2d_info(struct pxa3xx_u2d_mach_info *info)
{
	memcpy(&pxa_u2d_info, info, sizeof *info);
}

EXPORT_SYMBOL_GPL(pxa_set_u2d_info);

static u64 u2d_dma_mask = ~(u32)0;

static struct resource pxa3xx_u2d_resources[] = {
	[0] = {
		.start	= 0x54100000,
		.end	= 0x5410ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USB2,
		.end	= IRQ_USB2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device u2d_device = {
	.name		= "pxa3xx-u2d",
	.id		= -1,
	.resource	= pxa3xx_u2d_resources,
	.num_resources	= ARRAY_SIZE(pxa3xx_u2d_resources),
	.dev		=  {
		.platform_data	= &pxa_u2d_info,
		.dma_mask	= &u2d_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};

static struct platform_device zyt_pcmcia_device = {
	.name		= "pxa2xx-pcmcia",
	.id		= 0,
	.resource	= 0,
	.num_resources	= 0,
 	.dev		=  {
 		.platform_data	= NULL,
 		.dma_mask	= 0,
 		.coherent_dma_mask = 0,
		.release = 0,
 	}
};

void pxa_set_pcmcia_info(void *info)
{
        zyt_pcmcia_device.dev.platform_data = info;
}

EXPORT_SYMBOL_GPL(pxa_set_pcmcia_info);

static struct resource keypad_resources[2] = {
	{
 		.start = 0x41500000,
		.end   = 0x415000ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_KEYPAD,
		.end   = IRQ_KEYPAD,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device keypad_device = {
	.name		= "pxa3xx_keypad",
	.id		= -1,
	.resource	= keypad_resources,
	.num_resources	= ARRAY_SIZE(keypad_resources),
};

void __init pxa_set_keypad_info(void *info)
{
	keypad_device.dev.platform_data = info;
}
EXPORT_SYMBOL_GPL(pxa_set_keypad_info);

#endif

static struct resource m2d_resources[2] = {
	[0] = {
		.start = 0x54000000,
		.end   = 0x540fffff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GRPHICS,
		.end   = IRQ_GRPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device gcu_device = {
	.name		= "m2d",
	.id		= -1,
	.resource       = m2d_resources,
	.num_resources  = ARRAY_SIZE(m2d_resources),
};

#ifdef CONFIG_MACH_LITTLETON
static struct platform_device lt_ts_device = {
	.name		= "lt_ts",
	.id		= -1,
};
#endif

#if defined(CONFIG_MACH_LITTLETON)||defined(CONFIG_MACH_WOODSTOCK)
static struct platform_device lt_audio_device = {
	.name		= "lt_micco_codec",
	.id		= -1,
};
#endif

#if defined(CONFIG_MACH_LITTLETON) || defined(CONFIG_MACH_WOODSTOCK_ALPHAII)
static struct platform_device lt_charger_device = {
	.name		= "lt_charger",
	.id		= -1,
};
#endif
#ifdef CONFIG_MACH_WOODSTOCK_ALPHAII
static struct platform_device mma_device = {
	.name = "mma7455l",
	.id = -1,
};
#endif

void __init pxa_set_ficp_info(struct pxaficp_platform_data *info)
{
	pxaficp_device.dev.platform_data = info;
}

static struct platform_device pxatouch_device = {
        .name           = "pxa2xx-touch",
        .id             = -1,
};

static struct platform_device pxartc_device = {
	.name		= "pxa-rtc",
	.id		= -1,
};

#ifdef CONFIG_PXA3xx
static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_MACH_WOODSTOCK
	&woodstock_i2c_device,
#else
	&i2c_device,
#endif
	&pxa3xx_pmic_device,
#ifdef CONFIG_PXA3xx_GPIOEX
	&gpio_exp0_device,
	&gpio_exp1_device,
#endif
#ifdef CONFIG_PXA_MWB_12
	&gpio_exp2_device,
#endif
	&pxa3xx_fv_device,
	&pxafb_device,
	&ffuart_device,
#ifndef CONFIG_MACH_WOODSTOCK
	&btuart_device,
#endif
#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA320)
	&udc_device,
#endif
	&ohci_hcd_pxa_device,
	&u2d_device,
#ifndef CONFIG_MACH_WOODSTOCK
#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310)
	&stuart_device,
#endif
#endif
	&nand_device,
	&camera_device,
	&w1_device,
	&pxartc_device,
#ifdef CONFIG_CPU_PXA320
	&true_ide_device,
#endif
#ifdef CONFIG_PXA_IRDA
	&pxaficp_device,
#endif
	&mmc1_device,
#ifdef CONFIG_MMC2
	&mmc2_device,
#endif
#ifdef CONFIG_MMC3
	&mmc3_device,
#endif
	&keypad_device,
#ifdef CONFIG_USB_PXA3xx_OTG
	&otg_device,
#endif
	&pxaac97_device,
	&pxatouch_device,
	&gcu_device,
#ifdef CONFIG_MACH_WOODSTOCK
	&i2c_device,
#endif
#if defined(CONFIG_MACH_LITTLETON) || defined(CONFIG_MACH_WOODSTOCK)
	&lt_audio_device,
#endif
#ifdef  CONFIG_MACH_LITTLETON
	&lt_ts_device,
	&lt_charger_device,
#endif
#ifdef CONFIG_MACH_WOODSTOCK_ALPHAII
	&lt_charger_device,
	&mma_device,
#endif
};
#else
static struct platform_device *devices[] __initdata = {
	&pxamci_device,
#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA320)
	&udc_device,
#endif
	&pxafb_device,
	&ffuart_device,
	&btuart_device,
	&stuart_device,
	&pxaficp_device,
	&i2c_device,
#ifdef CONFIG_PXA27x
	&i2c_power_device,
#endif
	&i2s_device,
	&pxartc_device,
};
#endif

static void pxa_clock_init(void)
{
#ifdef CONFIG_CPU_MONAHANS_LV
	pxa_set_cken(CKEN_MVED, 0);
#endif
	pxa_set_cken(CKEN_GRAPHICS, 0);
	pxa_set_cken(CKEN_USB2, 0);
	pxa_set_cken(CKEN_HSIO2, 0);
}

void pxa_u_wait(int us)
{
	unsigned int ticks;
	unsigned int start_time;
	unsigned int cur_time;
	unsigned int delta;

	ticks = (us*325 + 99)/100;
	start_time = OSCR;
	do {
		cur_time = OSCR;
		delta = ((cur_time - start_time) > 0) ?
			(cur_time - start_time) :
			(0xffffffff - start_time + cur_time);
	} while (delta < ticks);
}

#ifdef CONFIG_PXA3xx
void pxa3xx_dmemc_init(void);
#endif

static int __init pxa_init(void)
{
	int cpuid, ret;

	/* clear RDH */
	ASCR &= 0x7fffffff;

	pxa_clock_init();
	ret = platform_add_devices(devices, ARRAY_SIZE(devices));
	if (ret)
		return ret;

	/* Only add HWUART for PXA255/26x; PXA210/250/27x do not have it. */
	cpuid = read_cpuid(CPUID_ID);
	if (((cpuid >> 4) & 0xfff) == 0x2d0 ||
	    ((cpuid >> 4) & 0xfff) == 0x290)
		ret = platform_device_register(&hwuart_device);

#ifdef CONFIG_PXA3xx
	pxa3xx_dmemc_init();
#endif

	return ret;
}

subsys_initcall(pxa_init);
