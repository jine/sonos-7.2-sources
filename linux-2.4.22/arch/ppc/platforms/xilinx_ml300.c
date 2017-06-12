/*
 * xilinx_ml300.c
 *
 * Xilinx ML300 evaluation board initialization
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/pagemap.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ocp.h>
#include <asm/keyboard.h>
#include <asm/pci-bridge.h>

#include "xilinx_ocp/xbasic_types.h"
extern u32 XWaitInAssert;
/*
 * The Xilinx drivers use something called XASSERT.  It should be
 * disabled in running systems to save time and space.  This is done by
 * defining NDEBUG when compiling anything that includes
 * xilinx_ocp/xbasic_types.h.  During development, it is handy to leave
 * NDEBUG off, but by default, the Xilinx code just tight loops.  In our
 * board_init function, we register a handler to be called that prints
 * out that we hit an XASSERT.  In addition, we tell it to not tight
 * loop by setting XWaitInAssert to zero.  This will probably lead to an
 * Oops and thus a backtrace.
 */

/* SAATODO: Make sure NDEBUG is on. */
void
reportXAssert(char *File, int Line)
{
	printk(KERN_CRIT "Xilinx OS Independent Code XAssert: %s:%d\n",
	       File, Line);
	printk(KERN_CRIT "Code may crash due to unhandled errors.\n");
}

/* Have OCP take care of the serial ports. */
struct ocp_def core_ocp[] = {
#ifdef XPAR_UARTNS550_0_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 0,
	  .paddr	= XPAR_UARTNS550_0_BASEADDR,
	  .irq		= 31 - XPAR_INTC_0_UARTNS550_0_VEC_ID,
	},
#ifdef XPAR_UARTNS550_1_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 1,
	  .paddr	= XPAR_UARTNS550_1_BASEADDR,
	  .irq		= 31 - XPAR_INTC_1_UARTNS550_0_VEC_ID,
	},
#ifdef XPAR_UARTNS550_2_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 2,
	  .paddr	= XPAR_UARTNS550_2_BASEADDR,
	  .irq		= 31 - XPAR_INTC_2_UARTNS550_0_VEC_ID,
	},
#ifdef XPAR_UARTNS550_3_BASEADDR
	{ .vendor	= OCP_VENDOR_XILINX,
	  .function	= OCP_FUNC_16550,
	  .index	= 3,
	  .paddr	= XPAR_UARTNS550_3_BASEADDR,
	  .irq		= 31 - XPAR_INTC_3_UARTNS550_0_VEC_ID,
	},
#ifdef XPAR_UARTNS550_4_BASEADDR
#error Edit this file to add more devices.
#endif				/* 4 */
#endif				/* 3 */
#endif				/* 2 */
#endif				/* 1 */
#endif				/* 0 */
	{OCP_NULL_TYPE, 0x0, OCP_IRQ_NA, OCP_CPM_NA}
};
#define NR_SER_PORTS (ARRAY_SIZE(core_ocp)-1)

#ifdef CONFIG_PCI
int __init
ppc405_map_irq(struct pci_dev *dev, unsigned char idsel, unsigned char pin)
{
#define PCI_INTA (31 - XPAR_INTC_0_PCI_0_VEC_ID_A)
#define PCI_INTB (31 - XPAR_INTC_0_PCI_0_VEC_ID_B)
#define PCI_INTC (31 - XPAR_INTC_0_PCI_0_VEC_ID_C)
#define PCI_INTD (31 - XPAR_INTC_0_PCI_0_VEC_ID_D)
	static signed char pci_irq_table[][4] =
	{
		{ PCI_INTA, PCI_INTB, PCI_INTC, PCI_INTD },	/* loopback */
		{ PCI_INTA, PCI_INTB, PCI_INTC, PCI_INTD },	/* PMC */
		{ PCI_INTA, PCI_INTB, PCI_INTC, -1 },		/* Card Bus */
	};
	const long min_idsel = 1, max_idsel = 3, irqs_per_slot = 4;
	return PCI_IRQ_TABLE_LOOKUP;
};
#endif

/*
 * As an overview of how the following functions (board_init,
 * board_io_mapping and board_setup_arch) fit into the kernel startup
 * procedure, here's a call tree:
 *
 * start_here					  arch/ppc/kernel/head_4xx.S
 *   machine_init				  arch/ppc/kernel/setup.c
 *     platform_init				  arch/ppc/kernel/setup.c
 *   	 board_init				  this file
 *   MMU_init					  arch/ppc/mm/init.c
 *     *ppc_md.setup_io_mappings == m4xx_map_io	  arch/ppc/kernel/ppc4xx_setup.c
 *	 io_block_mapping of ONB and PCI
 *   	 board_io_mapping			  this file
 *   start_kernel				  init/main.c
 *     setup_arch				  arch/ppc/kernel/setup.c
 *   	 *ppc_md.setup_arch == ppc4xx_setup_arch  arch/ppc/kernel/ppc4xx_setup.c
 *   	   ppc4xx_find_bridges			  arch/ppc/kernel/ppc405_pci.c
 *   	   board_setup_arch			  this file
 *
 * The main take away from this is that we need to do the initialization
 * of the PCI core after the io_block_mapping of the PCI areas, but
 * before ppc4xx_find_bridges.  So that means we put it in
 * board_io_mapping, even though that does not seem like the intuitive
 * place.
 */

void __init
board_init(void)
{
#if defined(CONFIG_MAGIC_SYSRQ) && defined(CONFIG_PC_KEYBOARD)	&& \
		defined(CONFIG_VT)
	extern unsigned char pckbd_sysrq_xlate[128];
	ppc_md.ppc_kbd_sysrq_xlate = pckbd_sysrq_xlate;
	SYSRQ_KEY = 0x54;
#endif

	XWaitInAssert = 0;
	XAssertSetCallback(reportXAssert);
}

void __init
board_io_mapping(void)
{
	extern struct serial_state rs_table[];
	int i;

	/* Remap all the serial ports */
	for (i = 0; i < NR_SER_PORTS; i++)
		rs_table[i].iomem_base =
		    ioremap((unsigned long) rs_table[i].iomem_base, 16);
#ifdef CONFIG_PCI
	/*
	 * Enable the PCI initiator functions in the PCI core by writing
	 * to the self-configuration space as described in the
	 * Configuration section of the OPB to PCI Bridge chapter of the
	 * Virtex-II Pro Platform FPGA Developer's Kit manual.
	 */
	out_be32((volatile unsigned *)(PPC4xx_PCI_LCFG_VADDR + PCI_COMMAND),
		 (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER
		  | PCI_COMMAND_PARITY | PCI_COMMAND_SERR | 0xffff0000));
#endif
}

void __init
board_setup_arch(void)
{
#ifdef CONFIG_PCI
	static const int bus = 0;
	static const int devfn = PCI_DEVFN(2, 0);
	static const int cr_offset = 0x80;
	static const u32 cr = ((1 << 27) |	/* P2CCLK */
			       (1 << 22) |	/* CBRSVD */
			       (4 << 16) |	/* CDMACHAN = not used */
			       (1 << 15) |	/* MRBURSTDN */
			       (1 << 6) |	/* PWRSAVINGS */
			       (1 << 5) |	/* SUBSYSRW */
			       (1 << 1));	/* KEEPCLK */
	static const int mfunc_offset = 0x8C;
	static const u32 mfunc = ((2 << 8) |	/* MFUNC2 = INTC */
				  (2 << 4) |	/* MFUNC1 = INTB */
				  (2 << 0));	/* MFUNC0 = INTA */
	
	/* Set up the clocks on the PCI44451 CardBus/FireWire bridge. */
	early_write_config_dword(0, bus, devfn, cr_offset, cr);

	/* Set up the interrupts on the PCI44451 CardBus/FireWire bridge. */
	early_write_config_dword(0, bus, devfn, mfunc_offset, mfunc);
#endif

	/* Identify the system */
	printk
	    ("Xilinx Virtex-II Pro port (C) 2002 MontaVista Software, Inc. (source@mvista.com)\n");
}

/* Called after board_setup_irq from ppc4xx_init_IRQ(). */
void __init
board_setup_irq(void)
{
}
