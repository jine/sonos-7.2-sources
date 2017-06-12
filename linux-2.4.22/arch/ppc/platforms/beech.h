/*
 * include/asm-ppc/platforms/beech.h   Platform definitions for the IBM Beech
 *                                     board
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (C) 2002, International Business Machines Corporation
 * All Rights Reserved.
 *
 * Bishop Brock
 * IBM Research, Austin Center for Low-Power Computing
 * bcbrock@us.ibm.com
 * March, 2002
 * 
 */

#ifdef __KERNEL__
#ifndef __ASM_BEECH_H__
#define __ASM_BEECH_H__

#include <platforms/ibm405lp.h>

#ifndef __ASSEMBLY__

/*
 * Data structure defining board information maintained by the standard boot
 * ROM on the IBM Beech board. An effort has been made to
 * keep the field names consistent with the 8xx 'bd_t' board info
 * structures.  
 */

typedef struct board_info {
	unsigned char	bi_s_version[4];	/* Version of this structure */
	unsigned long	bi_tbfreq;		/* Frequency of SysTmrClk */
	unsigned char	bi_r_version[30];	/* Version of the IBM ROM */
	unsigned int	bi_memsize;		/* DRAM installed, in bytes */
	unsigned long	sysclock_period;	/* SysClk period in ns */
	unsigned long	sys_speed;		/* SysCLk frequency in Hz */
	unsigned long	bi_intfreq;		/* Processor speed, in Hz */
	unsigned long	vco_speed;		/* PLL VCO speed, in Hz */
	unsigned long	bi_busfreq;		/* PLB Bus speed, in Hz */
	unsigned int  	bi_opb_busfreq;		/* OPB Bus speed, in Hz */
	unsigned long	bi_ebc_busfreq;		/* EBC Bus speed, in Hz */
	int		bi_iic_fast[1];		/* Use fast i2c mode */
} bd_t;

/* EBC Bank 0 controls the boot flash and SRAM */

#define BEECH_BANK0_PADDR      ((uint)0xfff00000)
#define BEECH_BANK0_EBC_SIZE   EBC0_BnCR_BS_1MB

#define BEECH_SRAM_PADDR		BEECH_BANK0_PADDR
#define BEECH_SRAM_SIZE			((uint)(512 * 1024))

#define BEECH_BOOTFLASH_PADDR		(BEECH_BANK0_PADDR + (512 * 1024))
#define BEECH_BOOTFLASH_SIZE		((uint)(512 * 1024))

/* EBC bank 1 controls the NVRAM, Ethernet and CPLD registers. The different
   areas are mapped in as small an area as possible to help catch any kernel
   addressing errors.

   NVRAM is improperly connected on Beech Pass 1.  Only every other location is
   accessible.  This is a 32 KB NVRAM.

   The Ethernet chip maps 13 address lines. We only map the "I/O" space used by
   the current driver.

   The FPGA "registers" are decoded on 128 KB boundarys. Each is mapped in a
   separate page. */

#define BEECH_BANK1_PADDR       ((uint)0xffe00000)
#define BEECH_BANK1_EBC_SIZE    EBC0_BnCR_BS_1MB

#define BEECH_NVRAM_PADDR		BEECH_BANK1_PADDR
#define BEECH_NVRAM_SIZE		((uint) (32 * 1024))

#define BEECH_ETHERNET_PADDR		(BEECH_BANK1_PADDR + 0x00020000)
#define BEECH_ETHERNET_SIZE		((uint) (8 * 1024))

#define BEECH_FPGA_REG_0_PADDR		(BEECH_BANK1_PADDR + 0x00080000)
#define BEECH_FPGA_REG_0_SIZE		PAGE_SIZE

#define BEECH_FPGA_REG_1_PADDR		(BEECH_BANK1_PADDR + 0x000A0000)
#define BEECH_FPGA_REG_1_SIZE		PAGE_SIZE

#define BEECH_FPGA_REG_2_PADDR		(BEECH_BANK1_PADDR + 0x000C0000)
#define BEECH_FPGA_REG_2_SIZE		PAGE_SIZE

#define BEECH_FPGA_REG_3_PADDR		(BEECH_BANK1_PADDR + 0x000E0000)
#define BEECH_FPGA_REG_3_SIZE		PAGE_SIZE

#define BEECH_FPGA_REG_4_PADDR		(BEECH_BANK1_PADDR + 0x00060000)
#define BEECH_FPGA_REG_4_SIZE		PAGE_SIZE

/* FPGA Register Bits (From IBM BIOS) [ May not be valid for Beech Pass 1 ]*/

#define FPGA_REG_0_FLASH_N		0x01
#define FPGA_REG_0_FLASH_ONBD_N		0x02
#define FPGA_REG_0_HITA_TOSH_N		0x04	/* New in Pass 2 */
#define FPGA_REG_0_STAT_OC		0x20
#define FPGA_REG_0_AC_SOURCE_SEL_N	0x40
#define FPGA_REG_0_AC_ACTIVE_N		0x80

#define FPGA_REG_1_USB_ACTIVE		0x01	/* New in Pass 2 */
#define FPGA_REG_1_CLK_VARIABLE		0x02
#define FPGA_REG_1_CLK_TEST		0x04
#define FPGA_REG_1_CLK_SS		0x08
#define FPGA_REG_1_EXT_IRQ_N		0x10
#define FPGA_REG_1_SMI_MODE_N		0x20
#define FPGA_REG_1_BATT_LOW_N		0x40
#define FPGA_REG_1_PCMCIA_PWR_FAULT_N	0x80

#define FPGA_REG_2_DEFAULT_UART1_N	0x01
#define FPGA_REG_2_EN_1_8V_PLL_N	0x02
#define FPGA_REG_2_PC_BUF_EN_N		0x08
#define FPGA_REG_2_CODEC_RESET_N	0x10	/* New in Pass 2 */
#define FPGA_REG_2_TP_JSTICK_N		0x20	/* New in Pass 2 */

#define FPGA_REG_3_GAS_GAUGE_IO		0x01

#define FPGA_REG_4_SDRAM_CLK3_ENAB	0x01
#define FPGA_REG_4_SDRAM_CLK2_ENAB	0x02
#define FPGA_REG_4_SDRAM_CLK1_ENAB	0x04
#define FPGA_REG_4_SDRAM_CLK0_ENAB	0x08
#define FPGA_REG_4_PCMCIA_5V		0x10	/* New in Pass 2 */
#define FPGA_REG_4_IRQ3			0x20	/* New in Pass 2 */

/* EBC Bank 2 contains the 16 MB "Linux" flash. The FPGA decodes a 32 MB
   bank. The lower 16 MB are available for expansion devices.  The upper 16 MB
   are used for the "Linux" flash.

   Partitioning information is for the benefit of the MTD driver.  See 
   drivers/mtd/maps/ibm4xx.c. We currently allocate the lower 1 MB for a
   kernel, and the other 15 MB for a filesystem.

*/

/* Bank 2 mappings changed between Beech Pass 1 and Pass 2 */

#ifdef CONFIG_BEECH_PASS1
#define BEECH_BIGFLASH_OFFSET 0
#else
#define BEECH_BIGFLASH_OFFSET (16 * 1024 * 1024)
#endif

#define BEECH_BANK2_PADDR      ((uint)0xf8000000)
#define BEECH_BANK2_EBC_SIZE   EBC0_BnCR_BS_32MB

#define BEECH_BIGFLASH_PADDR		(BEECH_BANK2_PADDR + BEECH_BIGFLASH_OFFSET)
#define BEECH_BIGFLASH_SIZE		(16 * 1024 * 1024)

#define BEECH_KERNEL_OFFSET    0
#define BEECH_KERNEL_SIZE      (1 * 1024 * 1024)

#define BEECH_FREE_AREA_OFFSET BEECH_KERNEL_SIZE
#define BEECH_FREE_AREA_SIZE   (BEECH_BIGFLASH_SIZE - BEECH_KERNEL_SIZE)

/* We do not currently support the internal clock mode for the UART.  This
   limits the minimum OPB frequency to just over 2X the UART oscillator
   frequency. At OPB frequencies less than this the serial port will not
   function due to the way that SerClk is sampled. */

#define PPC4xx_SERCLK_FREQ		11059200
#define BASE_BAUD			(PPC4xx_SERCLK_FREQ / 16)

#define RTC_DVBITS			RTC_DVBITS_1MHZ	/* 1MHz RTC */

#define PPC4xx_MACHINE_NAME		"IBM 405LP Beech"

#define PCI_DRAM_OFFSET		0

#include <asm/pccf_4xx.h>
#define _IO_BASE		(pccf_4xx_io_base)
#define _ISA_MEM_BASE		(pccf_4xx_mem_base)


/*****************************************************************************
 * Serial port definitions
 *****************************************************************************/

#ifdef CONFIG_SERIAL_MANY_PORTS
#define RS_TABLE_SIZE	64
#else
#define RS_TABLE_SIZE	4
#endif

#define UART0_INT	UIC_IRQ_U0
#define UART1_INT	UIC_IRQ_U1
#define UART0_IO_BASE	0xEF600300
#define UART1_IO_BASE	0xEF600400

#define STD_UART_OP(num)					\
	{ 0, BASE_BAUD, 0, UART##num##_INT,			\
		(ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST),	\
		iomem_base:(u8 *) UART##num##_IO_BASE,		\
		io_type: SERIAL_IO_MEM},

#if defined(CONFIG_UART0_TTYS0)
#define SERIAL_DEBUG_IO_BASE    UART0_IO_BASE
#define SERIAL_PORT_DFNS        \
        STD_UART_OP(0)          \
        STD_UART_OP(1)
#endif

#if defined(CONFIG_UART0_TTYS1)
#define SERIAL_DEBUG_IO_BASE    UART1_IO_BASE
#define SERIAL_PORT_DFNS        \
        STD_UART_OP(1)          \
        STD_UART_OP(0)
#endif

/****************************************************************************
 * Non-standard board support follows
 ****************************************************************************/

extern volatile u8 *beech_fpga_reg_0;
extern volatile u8 *beech_fpga_reg_2;
extern volatile u8 *beech_fpga_reg_4;

extern int beech_sram_free(void *p);
extern int ibm405lp_set_pixclk(unsigned pixclk_low, unsigned pixclk_high);
extern void *beech_sram_alloc(size_t size);

/* PM Button support */

#ifdef CONFIG_405LP_PM_BUTTON
#define IBM405LP_PM_IRQ      APM0_IRQ_WUI2
#define IBM405LP_PM_POLARITY 1
#endif

#endif				/* !__ASSEMBLY__ */
#endif				/* __ASM_BEECH_H__ */
#endif				/* __KERNEL__ */
