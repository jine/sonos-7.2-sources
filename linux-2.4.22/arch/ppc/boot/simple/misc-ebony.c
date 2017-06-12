/*
 * arch/ppc/simple/misc-ebony.c
 *
 * Misc. bootloader code for IBM Ebony reference platform 
 *
 * based on code by Matt Porter <mporter@mvista.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/types.h>
#include <linux/config.h>
#include <asm/processor.h>
#include <platforms/ebony.h>

extern unsigned long timebase_period_ns;
extern struct bi_record *decompress_kernel(unsigned long load_addr,
	int num_words, unsigned long cksum);

struct bi_record *
load_kernel(unsigned long load_addr, int num_words, unsigned long cksum)
{
	timebase_period_ns = 3;
	
	mtdcr(DCRN_MALCR(DCRN_MAL_BASE), MALCR_MMSR);		  /* reset MAL */
	while (mfdcr(DCRN_MALCR(DCRN_MAL_BASE)) & MALCR_MMSR) {}; /* wait for reset */
	*(volatile unsigned long *)EBONY_EMAC0_MR0 = 0x20000000;  /* reset EMAC */
	__asm__ __volatile__("eieio");				  /* enforce ordering */

	return decompress_kernel(load_addr, num_words, cksum);
}

