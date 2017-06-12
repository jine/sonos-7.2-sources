/*
 * xilinx_ml300.h
 *
 * Include file that defines the Xilinx ML300 evaluation board
 *
 * Author: MontaVista Software, Inc.
 *         source@mvista.com
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under the terms
 * of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#ifdef __KERNEL__
#ifndef __ASM_XILINX_ML300_H__
#define __ASM_XILINX_ML300_H__

/* ML300 has a Xilinx Virtex-II Pro processor */
#include <platforms/virtex-ii_pro.h>
#include <platforms/xilinx_ocp/xparameters.h>

#ifndef __ASSEMBLY__
typedef struct board_info {
	unsigned int bi_memsize;	/* DRAM installed, in bytes */
	unsigned char bi_enetaddr[6];	/* Local Ethernet MAC address */
	unsigned int bi_intfreq;	/* Processor speed, in Hz */
	unsigned int bi_busfreq;	/* Bus speed, in Hz */
	unsigned int bi_pci_busfreq;	/* PCI Bus speed, in Hz */
} bd_t;

/* Some 4xx parts use a different timebase frequency from the internal clock.
*/
#define bi_tbfreq bi_intfreq

#endif				/* !__ASSEMBLY__ */


#ifdef CONFIG_PCI
/* PCI memory space */
#define PPC405_PCI_MEM_BASE	XPAR_PCI_0_MEM_BASEADDR
#define PPC405_PCI_LOWER_MEM	XPAR_PCI_0_MEM_BASEADDR
#define PPC405_PCI_UPPER_MEM	XPAR_PCI_0_MEM_HIGHADDR

/* PCI I/O space parameters for io_block_mapping. */
#define PPC4xx_PCI_IO_PADDR	((uint)XPAR_PCI_0_IO_BASEADDR)
#define PPC4xx_PCI_IO_VADDR	PPC4xx_PCI_IO_PADDR
#define PPC4xx_PCI_IO_SIZE	0x10000 /* Hardcoded size from ppc405_pci.c */
/* PCI I/O space processor address */
#define PPC405_PCI_PHY_IO_BASE	XPAR_PCI_0_IO_BASEADDR
/* PCI I/O space PCI address */
#define PPC405_PCI_LOWER_IO	0x00000000
#define PPC405_PCI_UPPER_IO	(PPC405_PCI_LOWER_IO + PPC4xx_PCI_IO_SIZE - 1)

/* PCI Configuration space parameters for io_block_mapping. */
#define PPC4xx_PCI_CFG_PADDR	((uint)XPAR_PCI_0_CONFIG_ADDR)
#define PPC4xx_PCI_CFG_VADDR	PPC4xx_PCI_CFG_PADDR
#define PPC4xx_PCI_CFG_SIZE	8u /* size of two registers */
/* PCI Configuration space address and data registers. */
#define PPC405_PCI_CONFIG_ADDR	XPAR_PCI_0_CONFIG_ADDR
#define PPC405_PCI_CONFIG_DATA	XPAR_PCI_0_CONFIG_DATA

/* PCI Local configuration space parameters for io_block_mapping. */
#define PPC4xx_PCI_LCFG_PADDR	((uint)XPAR_PCI_0_LCONFIG_ADDR)
#define PPC4xx_PCI_LCFG_VADDR	PPC4xx_PCI_LCFG_PADDR
#define PPC4xx_PCI_LCFG_SIZE	256u /* PCI configuration address space size */

#define _IO_BASE	        isa_io_base
#define _ISA_MEM_BASE	        isa_mem_base
#define PCI_DRAM_OFFSET	        pci_dram_offset
#endif /* CONFIG_PCI */

/* We don't need anything mapped.  Size of zero will accomplish that. */
#define PPC4xx_ONB_IO_PADDR	0u
#define PPC4xx_ONB_IO_VADDR	0u
#define PPC4xx_ONB_IO_SIZE	0u

/* *** Serial Port Constants *** */
#define BASE_BAUD               (XPAR_UARTNS550_0_CLOCK_FREQ_HZ/16)

/* The serial ports in the Virtex-II Pro have each I/O byte in the
 * LSByte of a word.  This means that iomem_reg_shift needs to be 2 to
 * change the byte offsets into word offsets.  In addition the base
 * addresses need to have 3 added to them to get to the LSByte.
 */
#define STD_UART_OP(num)						\
	{ 0, BASE_BAUD, 0, 31-XPAR_INTC_0_UARTNS550_##num##_VEC_ID,	\
		(ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST),		\
		iomem_base: (u8 *)XPAR_UARTNS550_##num##_BASEADDR + 3,	\
		iomem_reg_shift: 2,					\
		io_type: SERIAL_IO_MEM},

#if defined(XPAR_INTC_0_UARTNS550_0_VEC_ID)
#define ML300_UART0 STD_UART_OP(0)
#else
#define ML300_UART0
#endif

#if defined(XPAR_INTC_0_UARTNS550_1_VEC_ID)
#define ML300_UART1 STD_UART_OP(1)
#else
#define ML300_UART1
#endif

#if defined(XPAR_INTC_0_UARTNS550_2_VEC_ID)
#define ML300_UART2 STD_UART_OP(2)
#else
#define ML300_UART2
#endif

#if defined(XPAR_INTC_0_UARTNS550_3_VEC_ID)
#define ML300_UART3 STD_UART_OP(3)
#else
#define ML300_UART3
#endif

#if defined(XPAR_INTC_0_UARTNS550_4_VEC_ID)
#error Edit this file to add more devices.
#endif

#if defined(CONFIG_UART0_TTYS0)
#define SERIAL_PORT_DFNS	\
	ML300_UART0		\
	ML300_UART1		\
	ML300_UART2		\
	ML300_UART3
#endif

#if defined(CONFIG_UART0_TTYS1)
#define SERIAL_PORT_DFNS	\
	ML300_UART1		\
	ML300_UART0		\
	ML300_UART2		\
	ML300_UART3
#endif

/* ps2 keyboard and mouse */
#define KEYBOARD_IRQ		(31 - XPAR_INTC_0_PS2_1_VEC_ID)
#define AUX_IRQ			(31 - XPAR_INTC_0_PS2_0_VEC_ID)

#define PPC4xx_MACHINE_NAME	"Xilinx ML300"

#endif				/* __ASM_XILINX_ML300_H__ */
#endif				/* __KERNEL__ */
