/*
 *   Mesh network routing
 *
 *      $Id: pktqueue.h,v 1.1 2004/01/14 17:36:17 vangool Exp $
 *
 */
#ifndef PKTQUEUE_H
#define PKTQUEUE_H

#include <linux/kernel.h>
#include <linux/module.h>

int init_packetqueue(void);
void deinit_packetqueue(void);
void pktqueue_send_ip(u_int32_t ip);
void pktqueue_drop_ip(u_int32_t ip);

#endif
