/*
 *   Mesh network routing
 *
 *      $Id: mainthread.c,v 1.1 2004/01/14 17:36:17 vangool Exp $
 *
 */
#include <linux/signal.h>
#include <linux/smp_lock.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/tqueue.h>

#include "mainthread.h"
#include "module.h"
#include "message_queue.h"
#include "mesh.h"
#include "messages.h"
#include "rrequest_cache.h"
#include "utils.h"

static int thread_pid;
static wait_queue_head_t thread_wait;
static atomic_t thread_exit;
static atomic_t thread_exited;

void wakeup_main_thread(void)
{
    /* The main thread is sleeping and it needs to wake up */
    wake_up_interruptible(&thread_wait);
}

void stop_main_thread()
{
    wait_queue_head_t queue;
    int count = 0;

    /* Starts a wait queue */
    init_waitqueue_head(&queue);

    /* Tell the thread to exit, and wait for it to happen */
    while ((atomic_read(&thread_exited) == 0) && (count < 10))
    {
        /* Inform the thread to wake up and exit */
        atomic_set(&thread_exit, 1);
        wake_up_interruptible(&thread_wait);

        /* Sleep for a second */
        interruptible_sleep_on_timeout(&queue,HZ);

        /* Let's give it another try... */
        count++;
    }
}

void main_thread(void)
{
    /* Pointer to the message we'll be working on */
    struct message_entry *working_event;

    /* Initialize... */
    init_waitqueue_head(&thread_wait);
    atomic_set(&thread_exit, 0);
    atomic_set(&thread_exited, 0);

    /* Name the thread */
    lock_kernel();
    sprintf(current->comm, "mesh");
    exit_mm(current);
    unlock_kernel();

    /* Loop until told to exit */
    for (;;) {

        /* Should we exit? */
        if (atomic_read(&thread_exit)) {
            break;
        }

        /* Go to sleep until we receive an interrupt */
        interruptible_sleep_on(&thread_wait);

        /* Should we exit? */
        if (atomic_read(&thread_exit)) {
            break;
        }

        /* Get the next message in the queue */
        while ((working_event=get_next_message()) != NULL) {

            /* Check the age of the package */
            if ((get_current_time() - working_event->time) > 1000) {
                printk(KERN_INFO "Packet is too old\n");
            }

            /* Handle the packet based on its type */
            switch (working_event->type)
            {
                /* RREQ */
                case RREQ:
                    if (!is_passive()) {
                        recv_rreq(working_event);
                    }
                    break;

                /* RREP */
                case RREP:
                    recv_rrep(working_event);
                    break;

                /* RREP ACK */
                case RREP_ACK:
                    recv_rrep_ack(working_event);
                    break;

                /* RERR */
                case RERR:
                    recv_rerr(working_event);
                    break;

                /* MACT */
                case MACT:
                    recv_mact(working_event);
                    break;

                /* GRPH */
                case GRPH:
                    recv_grph(working_event);
                    break;

                /* Clean up */
                default:
                    remove_inactive_routes();
                    delete_old_rreqs();
                    break;
            }

            /* Free the memory associated with the packet message */
            if (working_event->msgdata) kfree(working_event->msgdata);

            /* We're done with this packet */
            kfree(working_event);
        }
    }

    /* Inform everyone we've exited */
    atomic_set(&thread_exited, 1);
}

void start_main_thread(void)
{
    /* Start the main mesh processing thread */
    thread_pid = kernel_thread((void *) &main_thread, NULL, 0);
}

