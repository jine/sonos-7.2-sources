/*
 *	$Id: io_dc.c,v 1.1.1.1 2001/10/15 20:44:48 mrbrown Exp $
 *	I/O routines for SEGA Dreamcast
 */

#include <asm/io.h>
#include <asm/machvec.h>

unsigned long dreamcast_isa_port2addr(unsigned long offset)
{
	return offset + 0xa0000000;
}
