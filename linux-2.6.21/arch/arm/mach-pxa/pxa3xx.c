/*
 * linux/arch/arm/mach-pxa/pxa3xx.c
 *
 * Porting to PXA3xx based on PXA27x.c
 * Copyright (C) 2006 Marvell International Ltd .
 *
 * Code specific to PXA3xx.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/cacheflush.h>
#include <asm/arch/pxa-regs.h>
#include "generic.h"

/* Crystal clock: 13MHz */
#define BASE_CLK	13000000

/* ACSR field */
#define ACSR_SMC_MASK	0x03800000	/* Static Memory Controller Frequency Select */
#define ACSR_SRAM_MASK	0x000c0000	/* SRAM Controller Frequency Select */
#define ACSR_FC_MASK	0x00030000	/* Frequency Change Frequency Select */
#define ACSR_HSIO_MASK	0x0000c000	/* High Speed IO Frequency Select */
#define ACSR_DDR_MASK	0x00003000	/* DDR Memory Controller Frequency Select */
#define ACSR_XN_MASK	0x00000700	/* Run Mode Frequency to Turbo Mode Frequency Multiplier */
#define ACSR_XL_MASK	0x0000001f	/* Crystal Frequency to Memory Frequency Multiplier */
#define ACSR_XPDIS	(1 << 31)
#define ACSR_SPDIS	(1 << 30)
#define ACSR_13MEND1	(1 << 27)
#define ACSR_D0CS	(1 << 26)
#define ACSR_13MEND2	(1 << 21)
/*
 * Get the clock frequency as reflected by CCSR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 */
unsigned int get_clk_frequency_khz(int info)
{
	unsigned long acsr_val;
	int XL, XN, HSS;
	int clk, s_clk = 0, d0cs = 0;

	acsr_val = ACSR;
	XL = acsr_val & ACSR_XL_MASK;
	XN = (acsr_val & ACSR_XN_MASK) >> 8;

	clk = XL * XN * BASE_CLK;

	if (acsr_val & ACSR_D0CS) {
		d0cs = 1;
		clk = 60000000;
	}

	if (info) {
		if (d0cs) {
			s_clk = 60000000;
			printk(KERN_INFO "Run Mode Clock: %dMHz\n",
					(clk / 1000000));
			printk(KERN_INFO "High Speed I/O Bus Clock: %dMHz\n",
					(s_clk / 1000000));
		} else {
			HSS = (acsr_val & ACSR_HSIO_MASK) >> 14;
			switch (HSS) {
			case 0:
				s_clk = 104000000;
				break;
			case 1:
				s_clk = 156000000;
				break;
			case 2:
				s_clk = 208000000;
				break;
			default:
				break;
			}
			printk(KERN_INFO "Run Mode Clock: %dMHz\n",
				(XL * BASE_CLK / 1000000));
			if (XN > 1)
				printk(KERN_INFO "Turbo Mode Clock: %dMHz\n",
					(XL * XN * BASE_CLK / 1000000));
			printk(KERN_INFO "High Speed I/O Bus Clock: %dMHz\n",
					(s_clk / 1000000));
		}
	}

	return (clk / 1000);
}

/*
 * Return the current mem clock frequency in units of 10kHz
 */
unsigned int get_memclk_frequency_10khz(void)
{
	unsigned long acsr_val;
	int ddr_clk = 0;

	acsr_val = ACSR;
	if (acsr_val & ACSR_D0CS) {
		/* Ring Oscillator mode */
		ddr_clk = 30;
	} else {
		switch ((acsr_val & ACSR_DDR_MASK) >> 12) {
		case 0:
			ddr_clk = 26;
			break;
		case 3:
			ddr_clk = 260;
			break;
		default:
			break;
		}
	}
	return (ddr_clk * 100);
}

/*
 * Return the current LCD clock frequency in units of 10kHz as
 * LCLK is from High Speed IO Bus Clock
 */
unsigned int get_lcdclk_frequency_10khz(void)
{
	unsigned long acsr_val;
	int s_clk=0, HSS;

	acsr_val = ACSR;
	if (acsr_val & ACSR_D0CS) {
		/* Ring Oscillator mode */
		s_clk = 60;
	} else {
		HSS = (acsr_val & ACSR_HSIO_MASK) >> 14;
		switch (HSS) {
		case 0:
			s_clk = 104;
			break;
		case 1:
			s_clk = 156;
			break;
		case 2:
			s_clk = 208;
			break;
		default:
			break;
		}
	}
	return (s_clk * 100);
}

EXPORT_SYMBOL_GPL(get_clk_frequency_khz);
EXPORT_SYMBOL_GPL(get_memclk_frequency_10khz);
EXPORT_SYMBOL_GPL(get_lcdclk_frequency_10khz);

static unsigned long oscc_pout_refcount = 0;
void enable_oscc_pout(void)
{
	unsigned long val;
	unsigned long flags;

	local_irq_save(flags);
	if (!oscc_pout_refcount++) {
		val = __raw_readl((void*)&OSCC);
		val |= OSCC_PEN;
		__raw_writel(val, (void*)&OSCC);
	}
	local_irq_restore(flags);
	return;
}

void disable_oscc_pout(void)
{
	unsigned long val;
	unsigned long flags;
	
	local_irq_save(flags);
	if (!--oscc_pout_refcount) {
		val = __raw_readl((void*)&OSCC);
		val &= ~OSCC_PEN;
		__raw_writel(val, (void*)&OSCC);
	}
	local_irq_restore(flags);
	return;
}

EXPORT_SYMBOL(enable_oscc_pout);
EXPORT_SYMBOL(disable_oscc_pout);

#define pxa_inv_range		__glue(_CACHE,_dma_inv_range)
#define pxa_clean_range		__glue(_CACHE,_dma_clean_range)
#define pxa_flush_range		__glue(_CACHE,_dma_flush_range)

extern void pxa_inv_range(const void *, const void *);
extern void pxa_clean_range(const void *, const void *);
extern void pxa_flush_range(const void *, const void *);

EXPORT_SYMBOL(pxa_inv_range);
EXPORT_SYMBOL(pxa_clean_range);
EXPORT_SYMBOL(pxa_flush_range);


#define SLEW(A,B,C,PCODE,NCODE) ( ( A + ( B * (PCODE)) + ( C * (NCODE)) + 5000 ) / 10000 )
#define PSLEW30x32x(PCODE,NCODE) SLEW(-29600,-435,3480,PCODE,NCODE)
#define NSLEW30x32x(PCODE,NCODE) SLEW(28800,2500,-1250,PCODE,NCODE)
#define PSLEW31x(PCODE,NCODE) SLEW(39114,-1172,1964,PCODE,NCODE)
#define NSLEW31x(PCODE,NCODE) SLEW(-94340,3564,1906,PCODE,NCODE)

#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA320)
#define PSLEW PSLEW30x32x
#define NSLEW NSLEW30x32x
#elif defined(CONFIG_CPU_PXA310)
#define PSLEW PSLEW31x
#define NSLEW NSLEW31x
#else
#error "You must have one of CONFIG_CPU_PXA300 or CONFIG_CPU_PXA310 or CONFIG_CPU_PXA320"
#endif

void pxa3xx_calc_rcomp(void)
{
	int pslew,nslew;
	int pcode,ncode;
	unsigned int dmcisr,ss;

	dmcisr=DMCISR;
	pcode=((dmcisr>>22)&0x7f);
	ncode=((dmcisr>>15)&0x7f);

	pslew=PSLEW(pcode,ncode);
	nslew=NSLEW(pcode,ncode);
	if (pslew<0) pslew=0;
	if (pslew>15) pslew=15;
	if (nslew<0) nslew=0;
	if (nslew>15) nslew=15;
	ss = (pcode<<24)|(ncode<<16)|(pslew<<8)|nslew;
	// printk("pxa3xx_calc_rcomp: pcode %d ncode %d pslew %d nslew %d ss %08x\n",pcode,ncode,pslew,nslew,ss);
	PAD_MA = ss;
	PAD_MDLSB = ss;
	PAD_SDMEM = ss;
	PAD_SDCLK = ss;
	PAD_SDCS = ss;
	// printk("pxa3xx_calc_rcomp: set pad_ma %08x pad_mdlsb %08x pad_sdmem %08x pad_sdclk %08x pad_sdcs %08x\n",PAD_MA,PAD_MDLSB,PAD_SDMEM,PAD_SDCLK,PAD_SDCS);
	RCOMP=0x450319b2; // update, rcrng=2, setalways, rei=500ms

}

static irqreturn_t pxa3xx_dmemc_int(int irq,void *foo)
{
	unsigned int dmcisr;
	dmcisr=DMCISR;
	DMCISR=(dmcisr&0xe0000000); //ack ints
	if (dmcisr&0x80000000) pxa3xx_calc_rcomp();
	if (dmcisr&0x20000000) {
		// printk("pxa3xx_dmemc_int DLP dmcisr %08x ddr_dls %08x\n",dmcisr,DDR_DLS);
		DDR_HCAL=0x98000002;
	}
	return IRQ_HANDLED;
}

void _pxa3xx_dmemc_init(void);
void pxa3xx_dmemc_init(void)
{
	local_irq_disable();
	request_irq(IRQ_DMEMC,pxa3xx_dmemc_int,0,"dmemc",0);
	_pxa3xx_dmemc_init();
	local_irq_enable();
}

void _pxa3xx_dmemc_init(void)
{
	// printk("--pxa3xx_dmemc_init initial settings--\n");
	// printk("MDCNFG %08x MDREFR %08x DDR_HCAL %08x DDR_WCAL %08x\n",MDCNFG,MDREFR,DDR_HCAL,DDR_WCAL);
	// printk("DMCIER %08x DMCISR %08x DDR_DLS %08x RCOMP %08x\n",DMCIER,DMCISR,DDR_DLS,RCOMP);
	// printk("PAD_MA %08x PAD_MDLSB %08x PAD_SDMEM %08x PAD_SDCLK %08x PAD_SDCS %08x\n",PAD_MA,PAD_MDLSB,PAD_SDMEM,PAD_SDCLK,PAD_SDCS);
	// printk("ACCR %08x ACSR %08x ASCR %08x AGENP %08x\n",ACCR,ACSR,ASCR,AGENP);
	DMCISR=0xe0000000; // clear ints
	DMCIER=0xa0000000; // enable rcomp and dlp
	DDR_WCAL=0;
	DDR_HCAL=0x98000000; // hcen, hcprog, setalways, hcrng=0
	RCOMP=0x810319b2; // sweval, rcrng=0, setalways, rei=500ms
	// printk("--post-init--\n");
	// printk("DMCIER %08x DMCISR %08x DDR_HCAL %08x DDR_WCAL %08x RCOMP %08x\n",DMCIER,DMCISR,DDR_HCAL,DDR_WCAL,RCOMP);

}
