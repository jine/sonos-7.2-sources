/*
 *	Priority extension
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include "br_private.h"

int br_priority_for_addr(const unsigned char *addr)
{
	/* REVIEW: Currently hardcoded to Sonos OUIs.  We will extend this at
	 *         some point in the future (whitelist, different priorities,
	 *         etc).
	 */
	if ((0x00 == addr[0] && 0x0e == addr[1] && 0x58 == addr[2]) ||
	    (0x78 == addr[0] && 0x28 == addr[1] && 0xca == addr[2]) ||
	    (0x94 == addr[0] && 0x9f == addr[1] && 0x3e == addr[2]) ||
	    (0xb8 == addr[0] && 0xe9 == addr[1] && 0x37 == addr[2]) ||
	    (0x5c == addr[0] && 0xaa == addr[1] && 0xfd == addr[2])) {
		return 1;
	}

	return 0;
}

int br_priority_for_bpdu()
{
	return 2;
}
