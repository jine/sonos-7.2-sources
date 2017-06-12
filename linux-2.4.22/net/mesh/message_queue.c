/*
 *   Mesh network routing
 *
 *      $Id: message_queue.c,v 1.1 2004/01/14 17:36:17 vangool Exp $
 *
 */
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/udp.h>

#include "message_queue.h"
#include "utils.h"
#include "mainthread.h"

/*
 * Pointers to the head and tail of the double-linked list used to hold
 * messages that need to be acted on
 */
struct message_entry *message_queue_head;
struct message_entry *message_queue_tail;

/* Simple spinlock to serialize access to the message queue */
spinlock_t message_lock = SPIN_LOCK_UNLOCKED;

/* Lock the queue */
void lock_message_queue(void)
{
    spin_lock_bh(&message_lock);
}

/* Unlock the queue */
void unlock_message_queue(void)
{
    spin_unlock_bh(&message_lock);
}

/* Insert a new message into the queue */
int insert_message_entry(int type, struct sk_buff *packet)
{
    struct message_entry *new_msg;
    struct iphdr *ip;
    int start_point;

    /* The packet data is after the UDP/IP headers */
    start_point = sizeof(struct iphdr) + sizeof(struct udphdr);

    /* Allocate memory for the new message */
    if ((new_msg = (struct message_entry*)kmalloc(sizeof(struct message_entry),
                    GFP_ATOMIC)) == NULL)
    {
        /* Inform the user of the error */
        printk(KERN_WARNING "Unable to allocate new message entry\n");

        /* Return the error */
        return -ENOMEM;
    }

    /* Fill in the message info */
    new_msg->type = type;
    new_msg->time = get_current_time();
    new_msg->prev = NULL;

    /* If we have a packet, extract some info from it */
    if (packet) {
        /* Go to the IP part */
        ip  = packet->nh.iph;

        /* Get source and destination IP addresses */
        new_msg->src_ip = ip->saddr;
        new_msg->dst_ip = ip->daddr;

        /* Get source and destination MAC addresses */
        memcpy(&(new_msg->src_hw_addr), &(packet->mac.ethernet->h_source),
                sizeof(unsigned char)*ETH_ALEN);
        memcpy(&(new_msg->dst_hw_addr), &(packet->mac.ethernet->h_dest),
                sizeof(unsigned char)*ETH_ALEN);

        /* Record the device we received the packet on */
        new_msg->dev = packet->dev;

        /* Get the TTL */
        new_msg->ttl = ip->ttl;

        /* Remember the size of the packet */
        new_msg->size = packet->len-start_point;

        /* Allocate memory to store the actual packet */
        if ((new_msg->msgdata = kmalloc(new_msg->size, GFP_ATOMIC)) == NULL) {
            /* Inform the user of the error */
            printk(KERN_WARNING "Unable to allocate memory for message data\n");

            /* On error, clean up */
            kfree(new_msg);

            /* Return the error */
            return -ENOMEM;
        }

        /* Make a copy of the message data */
        memcpy(new_msg->msgdata, packet->data + start_point, new_msg->size);
    }

    /* Lock the message queue */
    lock_message_queue();

    /* Insert the message at the head of the list */
    new_msg->next = message_queue_head;

    /* Properly update the list */
    if (message_queue_head == NULL) {
        message_queue_tail = new_msg;
    } else {
        message_queue_head->prev = new_msg;
    }

    /* Update the head */
    message_queue_head = new_msg;

    /* Unlock the message queue */
    unlock_message_queue();

    /* Wake up the AODV thread so it processes the message */
    wakeup_main_thread();

    /* Return success */
    return 0;
}

/* Get the next message to be processed (starting from the tail) */
struct message_entry* get_next_message(void)
{
    struct message_entry* msg = NULL;

    /*lock table*/
    lock_message_queue();

    /* If there is something queued... */
    if (message_queue_tail != NULL) {
        /* That's our message */
        msg = message_queue_tail;

        /* If that was the only message... */
        if (msg->prev == NULL) {
            /* Reinit the queue */
            message_queue_tail = NULL;
            message_queue_head = NULL;
        } else {
            /* Or just update the tail */
            message_queue_tail = msg->prev;
            message_queue_tail->next = NULL;
        }
    }

    /* Unlock the queue */
    unlock_message_queue();

    /* Return the message */
    return msg;
}

/* Initialize the queue */
int init_message_queue(void)
{
    /* Initialize the head and tail */
    message_queue_head = NULL;
    message_queue_tail = NULL;

    /* Return success */
    return 0;
}

/* Clean up the message queue */
void deinit_message_queue(void)
{
    struct message_entry* entry = NULL;

    /* Traverse all message in the queue */
    while ((entry = get_next_message()) != NULL) {
        /* Free up the memory for the message */
        kfree(entry->msgdata);
        /* Free up the memory for the entry */
        kfree(entry);
    }
}

