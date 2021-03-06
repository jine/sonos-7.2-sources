/*
 *
 *    Copyright 2000 MontaVista Software Inc.
 * 	Author: Armin Kuster <akuster@mvista.com>
 * 		MontaVista Software, Inc.
 *
 *    Module name: sycamore.h
 *
 *    Description:
 *      Macros, definitions, and data structures specific to the IBM PowerPC
 *      based boards.
 *
 *      This includes:
 *
 *         405GP "Sycamore" evaluation board
 *
 * Please read the COPYING file for all license details.
 */

#ifdef __KERNEL__
#ifndef __ASM_SYCAMORE_H__
#define __ASM_SYCAMORE_H__

#include <platforms/ibm405gpr.h>

#ifndef __ASSEMBLY__
/*
 * Data structure defining board information maintained by the boot
 * ROM on IBM's "Sycamore" evaluation board. An effort has been made to
 * keep the field names consistent with the 8xx 'bd_t' board info
 * structures.
 */

typedef struct board_info {
	unsigned char	 bi_s_version[4];	/* Version of this structure */
	unsigned char	 bi_r_version[30];	/* Version of the IBM ROM */
	unsigned int	 bi_memsize;		/* DRAM installed, in bytes */
	unsigned char	 bi_enetaddr[6];	/* Local Ethernet MAC address */
	unsigned char	 bi_pci_enetaddr[6];	/* PCI Ethernet MAC address */
	unsigned int	 bi_intfreq;		/* Processor speed, in Hz */
	unsigned int	 bi_busfreq;		/* PLB Bus speed, in Hz */
	unsigned int	 bi_pci_busfreq;	/* PCI Bus speed, in Hz */
	unsigned long	 bi_opb_busfreq;	/* OPB Bus speed, in Hz */
	int		 bi_iic_fast[1];	/* Use fast i2c mode */
} bd_t;

/* Some 4xx parts use a different timebase frequency from the internal clock.
*/
#define bi_tbfreq bi_intfreq


/* Memory map for the IBM "Sycamore" 405GP evaluation board.
 * Generic 4xx plus RTC.
 */

extern void *walnut_rtc_base;
#define SYCAMORE_RTC_PADDR	((uint)0xf0000000)
#define SYCAMORE_RTC_VADDR	SYCAMORE_RTC_PADDR
#define SYCAMORE_RTC_SIZE		((uint)8*1024)

/* ps2 keyboard and mouse */
#define KEYBOARD_IRQ		25
#define AUX_IRQ			26

#ifdef CONFIG_PPC405GP_INTERNAL_CLOCK
#define BASE_BAUD		201600
#else
#define BASE_BAUD		691200
#endif

#define SYCAMORE_PS2_BASE		0xF0100000
#define SYCAMORE_FPGA_BASE	0xF0300000


extern void *kb_cs;
extern void *kb_data;
#define kbd_read_input()	readb(kb_data)
#define kbd_read_status()	readb(kb_cs)
#define kbd_write_output(val)	writeb(val, kb_data)
#define kbd_write_command(val)	writeb(val, kb_cs)

#define PPC4xx_MACHINE_NAME	"IBM Sycamore"

#endif /* !__ASSEMBLY__ */
#endif /* __ASM_SYCAMORE_H__ */
#endif /* __KERNEL__ */
