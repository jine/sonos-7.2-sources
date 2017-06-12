/*
 *   Mesh network routing
 *
 *      $Id: message_queue.h,v 1.1 2004/01/14 17:36:17 vangool Exp $
 *
 */
#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>

struct message_entry
{
    /* Pointers to the previous and next entry in the double linked list */
    struct message_entry *prev;
    struct message_entry *next;

    /* IP address the packet came from */
    u_int32_t src_ip;

    /* IP address the packet is going to */
    u_int32_t dst_ip;

    /* MAC address of the source */
    unsigned char src_hw_addr[ETH_ALEN];

    /* MAC address of the destination */
    unsigned char dst_hw_addr[ETH_ALEN];

    /* The network device we received the packet on */
    struct net_device *dev;

    /* What type of packet is it */
    int type;

    /* Size of the message and the message itself */
    unsigned int size;
    void *msgdata;

    /* The TTL of the packet */
    u_int8_t ttl;

    /* The time we received the packet */
    u_int64_t time;
};

/* Initialize the queue */
int init_message_queue(void);

/* Clean up the queue */
void deinit_message_queue(void);

/* Insert a new message into the queue (used by the packet handler) */
int insert_message_entry(int type, struct sk_buff *packet);

/* Used to iterate over the messages in the queue */
struct message_entry *get_next_message(void);

#endif
