/* 
 * linux/arch/sh/kernel/setup_7751se.c
 *
 * Copyright (C) 2000  Kazumoto Kojima
 *
 * Hitachi SolutionEngine Support.
 *
 * Modified for 7751 Solution Engine by
 * Ian da Silva and Jeremy Siegel, 2001.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <linux/hdreg.h>
#ifdef CONFIG_IDE
#include <linux/ide.h>
#endif
#include <asm/io.h>
#include <asm/hitachi_7751se.h>

/*
 * Configure the Super I/O chip
 */
#if 0
/* Leftover code from regular Solution Engine, for reference. */
/* The SH7751 Solution Engine has a different SuperIO. */
static void __init smsc_config(int index, int data)
{
	outb_p(index, INDEX_PORT);
	outb_p(data, DATA_PORT);
}

static void __init init_smsc(void)
{
	outb_p(CONFIG_ENTER, CONFIG_PORT);
	outb_p(CONFIG_ENTER, CONFIG_PORT);

	/* FDC */
	smsc_config(CURRENT_LDN_INDEX, LDN_FDC);
	smsc_config(ACTIVATE_INDEX, 0x01);
	smsc_config(IRQ_SELECT_INDEX, 6); /* IRQ6 */

	/* IDE1 */
	smsc_config(CURRENT_LDN_INDEX, LDN_IDE1);
	smsc_config(ACTIVATE_INDEX, 0x01);
	smsc_config(IRQ_SELECT_INDEX, 14); /* IRQ14 */

	/* AUXIO (GPIO): to use IDE1 */
	smsc_config(CURRENT_LDN_INDEX, LDN_AUXIO);
	smsc_config(GPIO46_INDEX, 0x00); /* nIOROP */
	smsc_config(GPIO47_INDEX, 0x00); /* nIOWOP */

	/* COM1 */
	smsc_config(CURRENT_LDN_INDEX, LDN_COM1);
	smsc_config(ACTIVATE_INDEX, 0x01);
	smsc_config(IO_BASE_HI_INDEX, 0x03);
	smsc_config(IO_BASE_LO_INDEX, 0xf8);
	smsc_config(IRQ_SELECT_INDEX, 4); /* IRQ4 */

	/* COM2 */
	smsc_config(CURRENT_LDN_INDEX, LDN_COM2);
	smsc_config(ACTIVATE_INDEX, 0x01);
	smsc_config(IO_BASE_HI_INDEX, 0x02);
	smsc_config(IO_BASE_LO_INDEX, 0xf8);
	smsc_config(IRQ_SELECT_INDEX, 3); /* IRQ3 */

	/* RTC */
	smsc_config(CURRENT_LDN_INDEX, LDN_RTC);
	smsc_config(ACTIVATE_INDEX, 0x01);
	smsc_config(IRQ_SELECT_INDEX, 8); /* IRQ8 */

	/* XXX: PARPORT, KBD, and MOUSE will come here... */
	outb_p(CONFIG_EXIT, CONFIG_PORT);
}
#endif

/*
 * Initialize IRQ setting
 */
void __init init_7751se_IRQ(void)
{

  /* Leave old Solution Engine code in for reference. */
#if defined(CONFIG_SH_SOLUTION_ENGINE)
	/*
	 * Super I/O (Just mimic PC):
	 *  1: keyboard
	 *  3: serial 0
	 *  4: serial 1
	 *  5: printer
	 *  6: floppy
	 *  8: rtc
	 * 12: mouse
	 * 14: ide0
	 */
	make_ipr_irq(14, BCR_ILCRA, 2, 0x0f-14);
	make_ipr_irq(12, BCR_ILCRA, 1, 0x0f-12); 
	make_ipr_irq( 8, BCR_ILCRB, 1, 0x0f- 8); 
	make_ipr_irq( 6, BCR_ILCRC, 3, 0x0f- 6);
	make_ipr_irq( 5, BCR_ILCRC, 2, 0x0f- 5);
	make_ipr_irq( 4, BCR_ILCRC, 1, 0x0f- 4);
	make_ipr_irq( 3, BCR_ILCRC, 0, 0x0f- 3);
	make_ipr_irq( 1, BCR_ILCRD, 3, 0x0f- 1);

	make_ipr_irq(10, BCR_ILCRD, 1, 0x0f-10); /* LAN */

	make_ipr_irq( 0, BCR_ILCRE, 3, 0x0f- 0); /* PCIRQ3 */
	make_ipr_irq(11, BCR_ILCRE, 2, 0x0f-11); /* PCIRQ2 */
	make_ipr_irq( 9, BCR_ILCRE, 1, 0x0f- 9); /* PCIRQ1 */
	make_ipr_irq( 7, BCR_ILCRE, 0, 0x0f- 7); /* PCIRQ0 */

	/* #2, #13 are allocated for SLOT IRQ #1 and #2 (for now) */
	/* NOTE: #2 and #13 are not used on PC */
	make_ipr_irq(13, BCR_ILCRG, 1, 0x0f-13); /* SLOTIRQ2 */
	make_ipr_irq( 2, BCR_ILCRG, 0, 0x0f- 2); /* SLOTIRQ1 */

#elif defined(CONFIG_SONOS)

	make_ipr_irq(11,INTC_IPRD,  0, 4);	/* -IRL3 Microcontroller */
	make_ipr_irq(8, INTC_IPRD,  1, 7);	/* -IRL2 Lower Mini PCI */
	make_ipr_irq(5, INTC_IPRD,  2, 10);	/* -IRL1 Display Controller */
	make_ipr_irq(2, INTC_IPRD,  3, 13);	/* -IRL0 Upper Mini PCI */

#elif defined(CONFIG_SH_7751_SOLUTION_ENGINE)

	make_ipr_irq(13, BCR_ILCRD, 3, 2);	/* SLOTIRQ1 */
	make_ipr_irq(9, BCR_ILCRD, 2, 6);	/* SLOTIRQ1 */
	make_ipr_irq(11, INTC_IPRD, 1, 7);	/* -IRQ2 */
	make_ipr_irq(8, INTC_IPRD, 0, 4);	/* -IRQ4 */

	/* Add additional calls to make_ipr_irq() as drivers are added
	 * and tested.
	 */

#if 0
        printk(KERN_INFO "BCR1: 0x%08x\n", ctrl_inl(0xff800000));
        printk(KERN_INFO "BCR2: 0x%08x\n", ctrl_inw(0xff800004));
        printk(KERN_INFO "BCR3: 0x%08x\n", ctrl_inw(0xff800050));
        printk(KERN_INFO "BCR4: 0x%08x\n", ctrl_inl(0xfe0a00f0));
        printk(KERN_INFO "WCR1: 0x%08x\n", ctrl_inl(0xff800008));
        printk(KERN_INFO "WCR2: 0x%08x\n", ctrl_inl(0xff80000c));
        printk(KERN_INFO "WCR3: 0x%08x\n", ctrl_inl(0xff800010));
        printk(KERN_INFO "MCR: 0x%08x\n", ctrl_inl(0xff800014));
        printk(KERN_INFO "PCR: 0x%08x\n", ctrl_inw(0xff800018));
#endif

/*
 *      BCR1: 0x80080008
 *      BCR2: 0xeffc
 *      WCR1: 0x02770771
 *      WCR2: 0x7ffe4fe7
 *      WCR3: 0x01777771
 *      MCR: 0x500901b4
 *      PCR: 0x0000
 */
	{
		unsigned short bcr2;
		unsigned long bcr4;
		unsigned long wcr1, wcr2, wcr3;

		/* setup area1 access registers */
		/* area primarily used to access dsp on CS1 */

		/* make area 1 bus width 8 bit */
		bcr2 = ctrl_inw( 0xff800004 );		/* get bcr2 */
		bcr2 &= ~0x0c;				/* clear area1 access bits */
		bcr2 |= 0x04;				/* 8 bit area1 accesses */
		ctrl_outw( bcr2,0xff800004 );		/* set the register */

		/* set area 1 idle cyc between r/w and off area to area ops */
		wcr1 = ctrl_inl( 0xff800008 );		/* get wcr1 */
		wcr1 &= ~0x70;				/* clear area 1 mask */
//		wcr1 |= 0x70;				/* default is 7 idle cyc between accesses */
		wcr1 |= 0x30;				/* try 3 idle cyc between accesses */
		ctrl_outl( wcr1,0xff800008 );		/* put the reg back */

		/* set area 1 wait states during cycle */
		wcr2 = ctrl_inl( 0xff80000c );		/* get wcr2 */
		wcr2 &= ~0x1c0;				/* clear 3 bit area 1 wait state field */
//		wcr2 |= 0x1c0;				/* default is 15 wait states */	
		wcr2 |= 0x080;				/* try 2 wait states */	
							/* values are 0,1,2,3,6,9,12,15 */
		ctrl_outl( wcr2,0xff80000c );		/* put reg back */

		/* set area 1 wait cyc between addr and write strobe or rd strob negation */
		wcr3 = ctrl_inl( 0xff800010 );		/* get wcr3 */
		wcr3 &= ~0xE0;				/* read and write hold cycles */
		wcr3 |= 0x60;				/* no read and 3 write hold cycles */
		ctrl_outl( wcr3,0xff800010 );		/* put get back */
		
		/* make rdy line generation asynch */
		bcr4 = ctrl_inl( 0xfe0a00f0 );		/* get bcr4 */
		bcr4 |= 0x01;				/* async rdy access */
		ctrl_outl( bcr4,0xfe0a00f0 );		/* write the reg */
	}

#endif

}

#define	FRQCR_60MHZ			0xe1a
#define	FRQCR_80MHZ			0xe11

/*
 * Initialize the board
 */
void __init setup_7751se(void)
{
#ifdef CONFIG_SONOS
	unsigned long icr;
 
	/* set ICR for IRL0-IRL3 as independent interrupts */
	icr = ctrl_inw( INTC_ICR );		/* ICR */
	icr |= INTC_ICR_IRLM;			/* set IRLM */
	ctrl_outw( icr,INTC_ICR );		/* set the register */
 
 	/* setup CS1:area1 and CS4:area4 access registers */
 	/* area primarily used to access DSP on CS1 on the zoneplayer */
 	/* and for access to the graphics controller memory on the handheld */
 	{
 		unsigned short bcr2;
 		unsigned long bcr4;
 		unsigned long wcr1, wcr2, wcr3;
 
 
 		/* make area 1 bus width 8 bit */
 		bcr2 = ctrl_inw( 0xff800004 );		/* get bcr2 */
 		bcr2 &= ~0x0c;				/* clear area1 access bits */
 		bcr2 |= 0x04;				/* 8 bit area1 accesses */
 		ctrl_outw( bcr2,0xff800004 );		/* set the register */
 
 		/* set area 1 idle cyc between r/w and off area to area ops */
 		wcr1 = ctrl_inl( 0xff800008 );		/* get wcr1 */
 		wcr1 &= ~0x70;				/* clear area 1 mask */
 		wcr1 |= 0x30;				/* try 3 idle cyc between accesses */
 		ctrl_outl( wcr1,0xff800008 );		/* put the reg back */
 
 		/* set area 1 wait states during cycle */
 		wcr2 = ctrl_inl( 0xff80000c );		/* get wcr2 */
 		wcr2 &= ~0x1c0;				/* clear 3 bit area 1 wait state field */
 		wcr2 |= 0x080;				/* try 2 wait states */	
 							/* values are 0,1,2,3,6,9,12,15 */
 		ctrl_outl( wcr2,0xff80000c );		/* put reg back */
 
 		/* set area 1 wait cyc between addr and write strobe or rd strob negation */
 		wcr3 = ctrl_inl( 0xff800010 );		/* get wcr3 */
 		wcr3 &= ~0xE0;				/* read and write hold cycles */
 		wcr3 |= 0x60;				/* no read and 3 write hold cycles */
 		ctrl_outl( wcr3,0xff800010 );		/* put get back */
 		
#ifdef CONFIG_SONOS_ZP
		/* XXX BT - I think the HH display controller wants #RDY to be synchronous*/
 		/* make rdy line generation asynch */
 		bcr4 = ctrl_inl( 0xfe0a00f0 );		/* get bcr4 */
 		bcr4 |= 0x01;				/* async rdy access */
 		ctrl_outl( bcr4,0xfe0a00f0 );		/* write the reg */

        /* until a better way is available - use the memory size */
        /* differential 16Mb for ES2 zone players and 32Mb for TS1 and later */
        /* zone players to decide to speed switch to 80Mhz operation */
        {
        volatile unsigned int *mcr=(unsigned int *)0xff800014;
        unsigned int v;
        v=*mcr;
        v&=0x00000038;
        v=v>>3;
        switch(v) {
            case 2:
                /* 16 Megabytes of SDRAM detected */
                break;
            case 3:
                /* 32 Megabytes of SDRAM detected */
                /* speed shift the zoneplayer TS1 boards to 80mhz */
                ctrl_outw( FRQCR_80MHZ,0xffc00000 );    /* reset our clock frequency */
                break;
            default:
                /* Unable to detect SDRAM size */
            }
        }
#endif
 	}
#endif
}
