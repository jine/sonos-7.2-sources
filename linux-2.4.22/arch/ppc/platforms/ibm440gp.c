/*
 * arch/ppc/platforms/ibm440gp.c
 *
 * PPC440GP I/O descriptions
 *
 * Matt Porter <mporter@mvista.com>
 * Armin Kuster <akuster@mvista.com>
 *
 * Copyright 2002 MontaVista Software Inc.
 *
 * Eugene Surovegin <eugene.surovegin@zultys.com> or <ebs@ebshome.net>
 * Copyright (c) 2003 Zultys Technologies
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/param.h>
#include <linux/string.h>
#include <asm/ibm440.h>
#include <asm/mmu.h>
#include <asm/ocp.h>
#include "ebony.h"

phys_addr_t fixup_bigphys_addr(phys_addr_t addr, phys_addr_t size)
{
	phys_addr_t page_4gb = 0;

        /*
	 * Trap the least significant 32-bit portions of an
	 * address in the 440's 36-bit address space.  Fix
	 * them up with the appropriate ERPN
	 */
	if ((addr >= PPC440_IO_LO) && (addr < PPC440_IO_HI))
		page_4gb = PPC440_IO_PAGE;
	else if ((addr >= PPC440_PCICFG_LO) && (addr < PPC440_PCICFG_HI))
		page_4gb = PPC440_PCICFG_PAGE;
	else if ((addr >= PPC440_PCIMEM_LO) && (addr < PPC440_PCIMEM_HI))
		page_4gb = PPC440_PCIMEM_PAGE;

	return (page_4gb | addr);
};

static struct ocp_func_emac_data ibm440gp_emac0_def = {
	.zmii_idx	= 0,		/* ZMII device index */
	.zmii_mux	= 0,		/* ZMII input of this EMAC */
	.mal_idx	= 0,		/* MAL device index */
	.mal_rx_chan	= 0,		/* MAL rx channel number */
	.mal_tx1_chan	= 0,		/* MAL tx channel 1 number */
	.mal_tx2_chan	= 1,		/* MAL tx channel 2 number */
	.wol_irq	= BL_MAC_WOL,	/* WOL interrupt number */
	.mdio_idx	= -1,		/* No shared MDIO */
};

static struct ocp_func_emac_data ibm440gp_emac1_def = {
	.zmii_idx	= 0,		/* ZMII device index */
	.zmii_mux	= 1,		/* ZMII input of this EMAC */
	.mal_idx	= 0,		/* MAL device index */
	.mal_rx_chan	= 1,		/* MAL rx channel number */
	.mal_tx1_chan	= 2,		/* MAL tx channel 1 number */
	.mal_tx2_chan	= 3,		/* MAL tx channel 2 number */
	.wol_irq	= BL_MAC_WOL1,	/* WOL interrupt number */
	.mdio_idx	= -1,		/* No shared MDIO */
};

static struct ocp_func_mal_data ibm440gp_mal0_def = {
	.num_tx_chans	= 2*EMAC_NUMS,	/* Number of TX channels */
	.num_rx_chans	= EMAC_NUMS,	/* Number of RX channels */
};

struct ocp_def core_ocp[]  __initdata = {
/*	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_OPB,
	  .index	= 0,
	  .paddr	= OPB_BASE_START,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	},
*/	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_16550,
	  .index	= 0,
	  .paddr	= PPC440GP_UART0_ADDR,
	  .irq		= UART0_INT,
	  .pm		= IBM_CPM_UART0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_16550,
	  .index	= 1,
	  .paddr	= PPC440GP_UART1_ADDR,
	  .irq		= UART1_INT,
	  .pm		= IBM_CPM_UART1
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_IIC,
	  .index	= 0,
	  .paddr	= PPC440GP_IIC0_ADDR,
	  .irq		= IIC0_IRQ,
	  .pm		= IBM_CPM_IIC0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_IIC,
	  .index	= 1,
	  .paddr	= PPC440GP_IIC1_ADDR,
	  .irq		= IIC1_IRQ,
	  .pm		= IBM_CPM_IIC1
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_GPIO,
	  .paddr	= PPC440GP_GPIO0_ADDR,
	  .irq		= OCP_IRQ_NA,
	  .pm		= IBM_CPM_GPIO0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_MAL,
	  .paddr	= OCP_PADDR_NA,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ibm440gp_mal0_def,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_ZMII,
	  .paddr	= PPC440GP_ZMII_ADDR,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	},	
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 0,
	  .paddr	= PPC440GP_EMAC0_ADDR,
	  .irq		= BL_MAC_ETH0,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ibm440gp_emac0_def,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 1,
	  .paddr	= PPC440GP_EMAC1_ADDR,
	  .irq		= BL_MAC_ETH1,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ibm440gp_emac1_def,
	},
	{ .vendor	= OCP_VENDOR_INVALID
	}
 };

#if defined(EMAC_NUMS) && EMAC_NUMS > 0
/* NOTE: This is defined in ppc4xx_setup.c too, for some reasons,
 * we don't use this file for 440. This should be fixed either
 * way to avoid this ugly duplication. --BenH.
 */
u32 emac_phy_map[EMAC_NUMS];
EXPORT_SYMBOL(emac_phy_map);
#endif

/*
 * Calculate 440GP clocks
 */
void ibm440gp_get_clocks(struct ibm440gp_clocks* p, unsigned int sys_clk, 
	unsigned int ser_clk)
{
	u32 cpc0_sys0 = mfdcr(DCRN_CPC0_SYS0);
	u32 cpc0_cr0 = mfdcr(DCRN_CPC0_CR0);
	u32 opdv, epdv;
	
	if (cpc0_sys0 & 0x2){
		/* Bypass system PLL */
		p->cpu = p->plb = sys_clk;
	}
	else {
		u32 fbdv, fwdva, fwdvb, m, vco;
		
		fbdv = (cpc0_sys0 >> 18) & 0x0f;
		if (!fbdv)
			fbdv = 16;
    
		fwdva = 8 - ((cpc0_sys0 >> 15) & 0x7);
		fwdvb = 8 - ((cpc0_sys0 >> 12) & 0x7);
    
    		/* Feedback path */	    
		if (cpc0_sys0 & 0x00000080){
			/* PerClk */
			m = fwdvb * opdv * epdv;
		}
		else {
			/* CPU clock */
			m = fbdv * fwdva;
    		}
		vco = sys_clk * m;
		p->cpu = vco / fwdva;
		p->plb = vco / fwdvb;
	}
    
	opdv = ((cpc0_sys0 >> 10) & 0x3) + 1;
	epdv = ((cpc0_sys0 >> 8) & 0x3) + 1;
	
	p->opb = p->plb / opdv;
	p->ebc = p->opb / epdv;
    
	if (cpc0_cr0 & 0x00400000){
		/* External UART clock */
		p->uart = ser_clk;
	}
	else {
		/* Internal UART clock */    
    		u32 uart_div = ((cpc0_cr0 >> 16) & 0x1f) + 1;
		p->uart = p->plb / uart_div;
	}
}

