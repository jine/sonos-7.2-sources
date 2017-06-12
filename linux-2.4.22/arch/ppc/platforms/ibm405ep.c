/*
 * arch/ppc/platforms/ibm405ep.c
 *
 * Support for IBM PPC 405EP processors.
 *
 * Author: SAW (IBM), derived from ibmnp405l.c.
 *         Maintained by MontaVista Software <source@mvista.com>
 *
 * 2003 (c) MontaVista Softare Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/param.h>
#include <linux/string.h>
#include <platforms/ibm405ep.h>
#include <asm/ibm4xx.h>
#include <asm/ocp.h>


static struct ocp_func_emac_data ibm405ep_emac0_def = {
	.zmii_idx	= -1,		/* ZMII device index */
	.zmii_mux	= 0,		/* ZMII input of this EMAC */
	.mal_idx	= 0,		/* MAL device index */
	.mal_rx_chan	= 0,		/* MAL rx channel number */
	.mal_tx1_chan	= 0,		/* MAL tx channel 1 number */
	.mal_tx2_chan	= 1,		/* MAL tx channel 2 number */
	.wol_irq	= BL_MAC_WOL,	/* WOL interrupt number */
	.mdio_idx	= -1,		/* No shared MDIO */
};

// FIXME: Check WOL IRQ
static struct ocp_func_emac_data ibm405ep_emac1_def = {
	.zmii_idx	= -1,		/* ZMII device index */
	.zmii_mux	= 0,		/* ZMII input of this EMAC */
	.mal_idx	= 0,		/* MAL device index */
	.mal_rx_chan	= 1,		/* MAL rx channel number */
	.mal_tx1_chan	= 2,		/* MAL tx channel 1 number */
	.mal_tx2_chan	= 3,		/* MAL tx channel 2 number */
	.wol_irq	= BL_MAC_WOL,	/* WOL interrupt number */
	.mdio_idx	= -1,		/* No shared MDIO */
};

static struct ocp_func_mal_data ibm405ep_mal0_def = {
	.num_tx_chans	= 2*EMAC_NUMS,	/* Number of TX channels */
	.num_rx_chans	= EMAC_NUMS,	/* Number of RX channels */
};

struct ocp_def core_ocp[]  __initdata = {
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_OPB,
	  .index	= 0,
	  .paddr	= OPB_BASE_START,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_16550,
	  .index	= 0,
	  .paddr	= UART0_IO_BASE,
	  .irq		= UART0_INT,
	  .pm		= IBM_CPM_UART0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_16550,
	  .index	= 1,
	  .paddr	= UART1_IO_BASE,
	  .irq		= UART1_INT,
	  .pm		= IBM_CPM_UART1
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_IIC,
	  .paddr	= IIC0_BASE,
	  .irq		= IIC0_IRQ,
	  .pm		= IBM_CPM_IIC0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_GPIO,
	  .paddr	= GPIO0_BASE,
	  .irq		= OCP_IRQ_NA,
	  .pm		= IBM_CPM_GPIO0
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_MAL,
	  .paddr	= OCP_PADDR_NA,
	  .irq		= OCP_IRQ_NA,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ibm405ep_mal0_def,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 0,
	  .paddr	= EMAC0_BASE,
	  .irq		= BL_MAC_ETH0,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ibm405ep_emac0_def,
	},
	{ .vendor	= OCP_VENDOR_IBM,
	  .function	= OCP_FUNC_EMAC,
	  .index	= 1,
	  .paddr	= EMAC0_BASE,
	  .irq		= BL_MAC_ETH1,
	  .pm		= OCP_CPM_NA,
	  .additions	= &ibm405ep_emac1_def,
	},
	{ .vendor	= OCP_VENDOR_INVALID
	}
};
