/* $Id: sh_bios.c,v 1.4 2003/09/22 20:55:24 vangool Exp $
 *
 *  linux/arch/sh/kernel/sh_bios.c
 *  C interface for trapping into the standard LinuxSH BIOS.
 *
 *  Copyright (C) 2000 Greg Banks, Mitch Davis
 *
 */

#include <asm/sh_bios.h>

#define BIOS_CALL_CONSOLE_WRITE     	0
#define BIOS_CALL_READ_BLOCK     	1
#define BIOS_CALL_ETH_NODE_ADDR		10
#define BIOS_CALL_SHUTDOWN		11
#define BIOS_CALL_CHAR_OUT     	    	0x1f  	/* TODO: hack */
#define BIOS_CALL_GDB_GET_MODE_PTR     	0xfe
#define BIOS_CALL_GDB_DETACH     	0xff

#define HACK_HACK_FOR_NOW

#ifdef HACK_HACK_FOR_NOW

static __inline__ long sh_bios_call(long func, long arg0, long arg1, long arg2, long arg3)
{
    register long r0 __asm__("r0") = func;
    register long r4 __asm__("r4") = arg0;
    register long r5 __asm__("r5") = arg1;
    register long r6 __asm__("r6") = arg2;
    register long r7 __asm__("r7") = arg3;
    __asm__ __volatile__("trapa	#0x3f"
	 : "=z" (r0)
	 : "0" (r0), "r" (r4), "r" (r5), "r" (r6), "r" (r7)
	 : "memory");
    return r0;
}


void sh_bios_console_write(const char *buf, unsigned int len)
{
    sh_bios_call(BIOS_CALL_CONSOLE_WRITE, (long)buf, (long)len, 0, 0);
}


void sh_bios_char_out(char ch)
{
    sh_bios_call(BIOS_CALL_CHAR_OUT, ch, 0, 0, 0);
}


int sh_bios_in_gdb_mode(void)
{
    static char queried = 0;
    static char *gdb_mode_p = 0;

    if (!queried)
    {
    	/* Query the gdb stub for address of its gdb mode variable */
    	long r = sh_bios_call(BIOS_CALL_GDB_GET_MODE_PTR, 0, 0, 0, 0);
	if (r != ~0)	/* BIOS returns -1 for unknown function */
	    gdb_mode_p = (char *)r;
	queried = 1;
    }
    return (gdb_mode_p != 0 ? *gdb_mode_p : 0);
}

void sh_bios_gdb_detach(void)
{
    sh_bios_call(BIOS_CALL_GDB_DETACH, 0, 0, 0, 0);
}

void sh_bios_get_node_addr (unsigned char *node_addr)
{
    sh_bios_call(BIOS_CALL_ETH_NODE_ADDR, 0, (long)node_addr, 0, 0);
}

void sh_bios_shutdown(unsigned int how)
{
    sh_bios_call(BIOS_CALL_SHUTDOWN, how, 0, 0, 0);
}

void *sh_bios_get_mdp(void)
{
    return (void *)sh_bios_call(41,0,0,0,0);
}

void sh_bios_standby(void)
{
    sh_bios_call(40,0,0,0,0);
}

#else

/* just stick it in the zero page base of the image for the moment */
static char *console_output_ptr = (char *)0x8c00e000;

void sh_bios_console_write(const char *buf, unsigned int len)
{
	while( len-- )
		*console_output_ptr++ = *buf++;
}

void sh_bios_char_out(char ch)
{
	*console_output_ptr++ = ch;
}

int sh_bios_in_gdb_mode(void)
{
	return( 0 );
}

void sh_bios_gdb_detach(void)
{
}

void sh_bios_get_node_addr (unsigned char *node_addr)
{
	*node_addr++ = 0x88;
	*node_addr++ = 0x08;
	*node_addr++ = 0x12;
	*node_addr++ = 0x34;
	*node_addr++ = 0x56;
	*node_addr++ = 0x78;
}

void sh_bios_shutdown(unsigned int how)
{
	while( 1 ) ;
}

#endif
