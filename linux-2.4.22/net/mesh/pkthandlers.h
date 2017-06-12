/*
 *   Mesh network routing
 *
 *      $Id: pkthandlers.h,v 1.1 2004/01/14 17:36:17 vangool Exp $
 *
 */
#ifndef PKTHANDLERS_H
#define PKTHANDLERS_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

/* Register and unregister the packet handlers */
int init_pkthandlers(void);
void deinit_pkthandlers(void);

#endif
