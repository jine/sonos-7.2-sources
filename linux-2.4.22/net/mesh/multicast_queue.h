/*
 *   Mesh network routing
 *
 *      $Id: multicast_queue.h,v 1.1 2004/01/09 19:30:34 vangool Exp $
 *
 */
#ifndef MULTICAST_QUEUE_H
#define MULTICAST_QUEUE_H

#include <linux/kernel.h>
#include <linux/module.h>

int init_multicast_queue(void);
void kill_multicast_thread(void);

int insert_multicast_message(u_int32_t dst_ip, unsigned int size,
        void *msgdata, u_int8_t ttl);

#endif
