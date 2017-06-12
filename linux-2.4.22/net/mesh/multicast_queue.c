/*
 *   Mesh network routing
 *
 *      $Id: multicast_queue.c,v 1.1 2004/01/09 19:30:34 vangool Exp $
 *
 */
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include "multicast_queue.h"
#include "utils.h"

#define MAX_MCAST_AGE 50

struct multicast_message_entry
{
    /* Pointers to the previous and next entry in the double linked list */
    struct multicast_message_entry *prev;
    struct multicast_message_entry *next;

    /* Where to send to packet (the multicast group) */
    u_int32_t multicast_group_ip;

    /* Size of the message and the message itself */
    unsigned int size;
    void *msgdata;

    /* TTL of the message */
    u_int8_t ttl;

    /* Time we received the message */
    u_int64_t time;
};

/*
 * Pointers to the head and tail of the double-linked list used to hold
 * multicast packets that need to be acted on
 */
struct multicast_message_entry *multicast_message_head;
struct multicast_message_entry *multicast_message_tail;

/* Simple spinlock to serialize access to the multicast queue */
spinlock_t multicast_lock = SPIN_LOCK_UNLOCKED;

/* Semaphore to communicate state of the queue */
static wait_queue_head_t multicast_wait;

/* Atomic data element to inform multicast thread to shutdown */
static atomic_t kill_thread;

/* Forward declarations */
void wakeup_multicast_thread(void);
void multicast_thread(void);

/* Lock the queue */
void lock_multicast(void)
{
    spin_lock_bh(&multicast_lock);
}

/* Unlock the queue */
void unlock_multicast(void)
{
    spin_unlock_bh(&multicast_lock);
}

/* Add a multicast message to the queue */
int insert_multicast_message(u_int32_t multicast_group_ip, unsigned int size,
        void *msgdata, u_int8_t ttl)
{
    struct multicast_message_entry *new_msg;

    /* Allocate memory for the new queue entry */
    if ((new_msg = (struct multicast_message_entry*)
                kmalloc(sizeof(struct multicast_message_entry),
                    GFP_ATOMIC)) == NULL)
    {
        /* Inform user of the error */
        printk(KERN_WARNING "Unable to allocate memory for message\n");

        /* Return the error */
        return -ENOMEM;
    }

    /* Fill in the data and decrease TTL by 1 */
    new_msg->prev = NULL;
    new_msg->next = NULL;
    new_msg->multicast_group_ip = multicast_group_ip;
    new_msg->size = size;
    new_msg->ttl = ttl - 1;
    new_msg->time = get_current_time();

    /* Allocate memory for the message contents */
    if ((new_msg->msgdata = kmalloc(size, GFP_ATOMIC)) ==  NULL) {
        /* It failed so clean up */
        kfree(new_msg);

        /* Inform user of the error */
        printk(KERN_WARNING "Unable to allocate memory for message data\n");

        /* Return the error */
        return -ENOMEM;
    }

    /* Copy the actual message contents to the new entry */
    memcpy(new_msg->msgdata, msgdata, size);

    /* Lock the queue */
    lock_multicast();

    /* Add the new message to the head of the queue */
    new_msg->next = multicast_message_head;
    if (multicast_message_head == NULL) {
        /* If this is the first message update the tail as well */
        multicast_message_tail = new_msg;
    } else {
        /* Otherwise have the current head point to the new message */
        multicast_message_head->prev = new_msg;
    }
    /* And update the head */
    multicast_message_head = new_msg;

    /* Unlock the queue */
    unlock_multicast();

    /* Wake up the thread so it processes the message */
    wakeup_multicast_thread();

    /* Return success */
    return 0;
}

/* Used to traverse the queue */
struct multicast_message_entry *get_multicast_message(void)
{
    struct multicast_message_entry* msg = NULL;

    /* Lock the queue */
    lock_multicast();

    /* Start at the tail and work backwards */
    if (multicast_message_tail != NULL) {
        /* Get the message */
        msg = multicast_message_tail;

        if (msg->prev == NULL) {
            /* If that was the only message, reinit the queue */
            multicast_message_head = NULL;
            multicast_message_tail = NULL;
        } else {
            /* Otherwise just move the tail */
            multicast_message_tail = msg->prev;
            multicast_message_tail->next = NULL;
        }
    }

    /* Unlock the queue */
    unlock_multicast();

    /* Return a pointer to the message */
    return msg;
}

/* Initialize the multicast queue */
int init_multicast_queue(void)
{
    pid_t pid;

    /* Initialize the queue itself */
    multicast_message_head = NULL;
    multicast_message_tail = NULL;

    /* Kick of the processing thread */
    pid = kernel_thread((void*)&multicast_thread, NULL, 0);

    /* Deal with the possible error */
    if (pid < 0) {
        printk(KERN_ERR "Unable to create multicast thread, errno %d\n", -pid);
        return 1;
    }

    /* All is well */
    return 0;
}

/* Tell the multicast thread to shutdown */
void kill_multicast_thread(void)
{
    /* Wake up the thread */
    wakeup_multicast_thread();

    /* And tell it to exit */
    atomic_set(&kill_thread, 1);
}

/* Wake up the multicast thread */
void wakeup_multicast_thread()
{
    /* Interrupt the multicast thread during its sleep */
    wake_up_interruptible(&multicast_wait);
}

/* The multicast thread that handles all received multicast packets */
void multicast_thread()
{
    struct multicast_message_entry *msg;

    /* Initialize the wait semaphore */
    init_waitqueue_head(&multicast_wait);

    /* We start in a running state */
    atomic_set(&kill_thread, 0);

    /* Give our thread a name (requires locking the kernel) */
    lock_kernel();
    sprintf(current->comm,"mesh_multicast");
    exit_mm(current);
    unlock_kernel();

    /* Loop forever */
    for (;;) {
        /* Start by sleeping until we get an interrupt */
        interruptible_sleep_on(&multicast_wait);

        /* Check if we should exit */
        if (atomic_read(&kill_thread)) {
            /* Gotta go */
            break;
        }

        /* Read all entries in the queue and process them */
        while ((msg = get_multicast_message()) != NULL) {

            /* Make sure it's not too old */
            if ((get_current_time() - msg->time) < MAX_MCAST_AGE) {
                send_multicast_message(msg->multicast_group_ip, msg->size,
                        msg->msgdata, msg->ttl);
            }

            /* Now free up the message itself and the queue entry */
            kfree(msg->msgdata);
            kfree(msg);
        }
    }
}

