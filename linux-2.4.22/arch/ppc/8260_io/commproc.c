/*
 * General Purpose functions for the global management of the
 * 8260 Communication Processor Module.
 * Copyright (c) 1999 Dan Malek (dmalek@jlc.net)
 * Copyright (c) 2000 MontaVista Software, Inc (source@mvista.com)
 *	2.3.99 Updates
 *
 * In addition to the individual control of the communication
 * channels, there are a few functions that globally affect the
 * communication processor.
 *
 * Buffer descriptors must be allocated from the dual ported memory
 * space.  The allocator for that is here.  When the communication
 * process is reset, we reclaim the memory available.  There is
 * currently no deallocator for this memory.
 */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/bootmem.h>
#include <asm/irq.h>
#include <asm/mpc8260.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/immap_8260.h>
#include <asm/cpm_8260.h>
#include <asm/bitops.h>

static	uint	dp_alloc_base;	/* Starting offset in DP ram */
static	uint	dp_alloc_top;	/* Max offset + 1 */
static	uint	host_buffer;	/* One page of host buffer */
static	uint	host_end;	/* end + 1 */
cpm8260_t	*cpmp;		/* Pointer to comm processor space */

/* We allocate this here because it is used almost exclusively for
 * the communication processor devices.
 */
immap_t		*immr;

void
m8260_cpm_reset(void)
{
	volatile immap_t	 *imp;
	volatile cpm8260_t	*commproc;
	uint			vpgaddr;

	immr = imp = (volatile immap_t *)IMAP_ADDR;
	commproc = &imp->im_cpm;

	/* Reclaim the DP memory for our use.
	*/
	dp_alloc_base = CPM_DATAONLY_BASE;
	dp_alloc_top = dp_alloc_base + CPM_DATAONLY_SIZE;

	/* Set the host page for allocation.
	*/
	host_buffer =
		(uint) alloc_bootmem_pages(PAGE_SIZE * NUM_CPM_HOST_PAGES);
	host_end = host_buffer + (PAGE_SIZE * NUM_CPM_HOST_PAGES);

	vpgaddr = host_buffer;

	/* Tell everyone where the comm processor resides.
	*/
	cpmp = (cpm8260_t *)commproc;
	commproc->cp_cpcr = CPM_CR_RST | CPM_CR_FLG;
	while ((commproc->cp_cpcr)&CPM_CR_FLG) ;
}

/* Allocate some memory from the dual ported ram.
 * To help protocols with object alignment restrictions, we do that
 * if they ask.
 */
uint
m8260_cpm_dpalloc(uint size, uint align)
{
	uint	retloc;
	uint	align_mask, off;
	uint	savebase;

	align_mask = align - 1;
	savebase = dp_alloc_base;

	if ((off = (dp_alloc_base & align_mask)) != 0)
		dp_alloc_base += (align - off);

	if ((dp_alloc_base + size) >= dp_alloc_top) {
		dp_alloc_base = savebase;
		return(CPM_DP_NOSPACE);
	}

	retloc = dp_alloc_base;
	dp_alloc_base += size;

	return(retloc);
}

/* We also own one page of host buffer space for the allocation of
 * UART "fifos" and the like.
 */
uint
m8260_cpm_hostalloc(uint size, uint align)
{
	uint	retloc;
	uint	align_mask, off;
	uint	savebase;

	align_mask = align - 1;
	savebase = host_buffer;

	if ((off = (host_buffer & align_mask)) != 0)
		host_buffer += (align - off);

	if ((host_buffer + size) >= host_end) {
		host_buffer = savebase;
		return(0);
	}

	retloc = host_buffer;
	host_buffer += size;

	return(retloc);
}

/* Set a baud rate generator.  This needs lots of work.  There are
 * eight BRGs, which can be connected to the CPM channels or output
 * as clocks.  The BRGs are in two different block of internal
 * memory mapped space.
 * The baud rate clock is the system clock divided by something.
 * It was set up long ago during the initial boot phase and is
 * is given to us.
 * Baud rate clocks are zero-based in the driver code (as that maps
 * to port numbers).  Documentation uses 1-based numbering.
 */
#define BRG_INT_CLK	(((bd_t *)__res)->bi_brgfreq)
#define BRG_UART_CLK	(BRG_INT_CLK/16)

/* This function is used by UARTS, or anything else that uses a 16x
 * oversampled clock.
 */
void
m8260_cpm_setbrg(uint brg, uint rate)
{
	volatile uint	*bp;

	/* This is good enough to get SMCs running.....
	*/
	if (brg < 4) {
		bp = (uint *)&immr->im_brgc1;
	}
	else {
		bp = (uint *)&immr->im_brgc5;
		brg -= 4;
	}
	bp += brg;
	*bp = (((BRG_UART_CLK / rate)-1) << 1) | CPM_BRG_EN;
}

/* This function is used to set high speed synchronous baud rate
 * clocks.
 */
void
m8260_cpm_fastbrg(uint brg, uint rate, int div16)
{
	volatile uint	*bp;

	if (brg < 4) {
		bp = (uint *)&immr->im_brgc1;
	}
	else {
		bp = (uint *)&immr->im_brgc5;
		brg -= 4;
	}
	bp += brg;
	*bp = (((BRG_INT_CLK / rate)-1) << 1) | CPM_BRG_EN;
	if (div16)
		*bp |= CPM_BRG_DIV16;
}

void m8260_port_set(int port,int pin,uint opts)
{
	volatile iop8260_t *iop;
	volatile uint *podr,*pdat,*pdir,*ppar,*psor;
	int bit=31-pin;
	iop=&(immr->im_ioport);
	switch(port) {
		case 0:
			podr= &(iop->iop_podra);
			pdat= &(iop->iop_pdata);
			pdir= &(iop->iop_pdira);
			ppar= &(iop->iop_ppara);
			psor= &(iop->iop_psora);
			break;
                case 1:
                        podr= &(iop->iop_podrb);
                        pdat= &(iop->iop_pdatb);
                        pdir= &(iop->iop_pdirb);
                        ppar= &(iop->iop_pparb);
                        psor= &(iop->iop_psorb);
                        break;
                case 2:
                        podr= &(iop->iop_podrc);
                        pdat= &(iop->iop_pdatc);
                        pdir= &(iop->iop_pdirc);
                        ppar= &(iop->iop_pparc);
                        psor= &(iop->iop_psorc);
                        break;
                case 3:
                        podr= &(iop->iop_podrd);
                        pdat= &(iop->iop_pdatd);
                        pdir= &(iop->iop_pdird);
                        ppar= &(iop->iop_ppard);
                        psor= &(iop->iop_psord);
                        break;
		default:
			printk("There is no such IO port as port %d\n",port);
			return;
	}
        eieio();
	if (opts&PORT_SET_ODR) set_bit(bit,podr);
	if (opts&PORT_CLEAR_ODR) clear_bit(bit,podr);
	if (opts&PORT_SET_DAT) set_bit(bit,pdat);
	if (opts&PORT_CLEAR_DAT) clear_bit(bit,pdat);
	if (opts&PORT_SET_DIR) set_bit(bit,pdir);
	if (opts&PORT_CLEAR_DIR) clear_bit(bit,pdir);
	if (opts&PORT_SET_PAR) set_bit(bit,ppar);
	if (opts&PORT_CLEAR_PAR) clear_bit(bit,ppar);
	if (opts&PORT_SET_SOR) set_bit(bit,psor);
	if (opts&PORT_CLEAR_SOR) clear_bit(bit,psor);
        eieio();

}

int m8260_port_read(int port,int pin)
{
        volatile iop8260_t *iop;
        volatile uint *pdat;
        int bit=31-pin;
        iop=&(immr->im_ioport);
        switch(port) {
                case 0:
                        pdat= &(iop->iop_pdata);
                        break;
		case 1:
			pdat= &(iop->iop_pdatb);
			break;
		case 2:
			pdat= &(iop->iop_pdatc);
			break;
		case 3:
			pdat= &(iop->iop_pdatd);
			break;
		default:
			printk("There is no such port as port %d\n",port);
			return -1;
	}
	eieio();
	if ((*pdat)&(1<<bit)) {eieio(); return 1;} else {eieio(); return 0;}

}


