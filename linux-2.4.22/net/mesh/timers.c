#include "timers.h"
#include "pktqueue.h"
#include "message_queue.h"
#include "iflist.h"
#include "messages.h"

/* 'Master' timer */
struct timer_list mesh_timer;

/* Head of the timer list */
struct timer_queue_entry *timer_queue = NULL;

/* Lock for timer list */
rwlock_t timer_lock = RW_LOCK_UNLOCKED;
unsigned long lock_flags;

void timer_read_lock(void)
{
    read_lock_irqsave(&timer_lock, lock_flags);
}

void timer_read_unlock(void)
{
    read_unlock_irqrestore(&timer_lock, lock_flags);
}

void timer_write_lock(void)
{
    write_lock_irqsave(&timer_lock, lock_flags);
}

void timer_write_unlock(void)
{
    write_unlock_irqrestore(&timer_lock, lock_flags);
}

/* Function to deal with no response to sent RREQ */ 
int timer_rreq(struct timer_queue_entry *tmp_entry)
{
    struct rtable_entry *tmp_route;
    struct rreq *tmp_rreq;

    /* Get a pointer to the sent RREQ */
    tmp_rreq = tmp_entry->data;

    /* Check how may time we have sent it already */
    if (tmp_entry->retries >= RREQ_RETRIES) {
        /* It has been sent maximum number */
        /* Drop the queued packages for this IP */
        pktqueue_drop_ip(tmp_rreq->dst_ip);
        /* Clean up the data since we're not going to reschedule */
        kfree(tmp_entry->data);
        /* Return error */
        return 0;
    } else {
        /* Increment the number of retries */
        tmp_entry->retries++;

        /* Check new TTL */
        if (tmp_entry->ttl > TTL_THRESHOLD) {
            tmp_entry->ttl = NET_DIAMETER;
        } else {
            tmp_entry->ttl += TTL_INCREMENT;
        }

        /* Get the corresponding route */
        tmp_route = (ip2if(tmp_rreq->src_ip))->route_entry;
        /* Update the RREQ id */
        (tmp_route->rreq_id)++;
        /* And convert it to network order */
        tmp_rreq->rreq_id = htonl(tmp_route->rreq_id);

        /* Add it to the cache */
        if (insert_rreq(tmp_rreq->src_ip, tmp_rreq->dst_ip, tmp_route->rreq_id,
                    get_current_time() + tmp_entry->ttl * 2 * NODE_TRAVERSAL_TIME ) <0)
        {
            /* Inform the user of the error */
            printk(KERN_WARNING "Could not add to cache\n");
            /* Free the data since we are not going to reschedule */
            kfree(tmp_entry->data);
            /* Return the error */
            return -ENOMEM;
        }

        /* Send the RREQ again */
        send_broadcast(tmp_entry->ttl, tmp_rreq, tmp_entry->size);

        /* Reschedule this timer */
        insert_timer(get_current_time() + NET_TRAVERSAL_TIME, tmp_rreq,
                tmp_entry->size, tmp_rreq->dst_ip, tmp_entry->retries,
                tmp_entry->ttl, timer_rreq);
    }

    return 0;
}

/* Timer function to timeout neighbours */
int timer_neighbour(struct timer_queue_entry *timer_entry)
{
    struct mesh_neighbour *the_neighbour;
    struct rtable_entry *tmp_route;

    /* Get a pointer to the neighbour */
    the_neighbour = find_neighbour(timer_entry->id, NULL);

    /* Assuming we found it... */
    if (the_neighbour != NULL) {
        /* Get the corresponding route */
        tmp_route = find_route_table_entry(the_neighbour->ip);
        if (tmp_route != NULL) {
            /* Expire the route */
            tmp_route->lifetime = get_current_time() - 1;
            /* Delete the neighbour */
            delete_neighbour(the_neighbour->ip, NULL);
        }
    }

    return 0;
}

/* Timer function that gets called to send scheduled HELLO messages */
int hello_resend(struct timer_queue_entry *tmp_entry)
{
    struct rrep *hello_msg;
    struct if_entry *interface;

    /* The data contains a copy of the HELLO message */
    hello_msg = tmp_entry->data;

    /* Get the interface for this message */
    interface = ip2if(hello_msg->dst_ip);
    if (interface == NULL) {
        /* Inform the user */
        printk(KERN_WARNING "Can't find the interface\n");
        /* We're not going to reschedule, so free the memory */
        kfree(hello_msg);
        /* Get out */
        return 0;
    }

    /* Set the new sequence count */
    hello_msg->dst_seq = htonl(interface->route_entry->dst_seq);

    /* Send out the HELLO message */
    send_broadcast(1, hello_msg, sizeof(struct rrep));

    /* Reschedule the timer function */
    insert_timer(tmp_entry->tv + HELLO_INTERVAL, hello_msg, tmp_entry->size,
            hello_msg->dst_ip, tmp_entry->retries, tmp_entry->ttl, hello_resend);

    return 0;
}

/* Clean up mess */
int timer_cleanup(struct timer_queue_entry *timer_entry)
{
    /* Tell the message queue to clean itself up */
    insert_message_entry(0, NULL);

    /* Do another one of these after my route times out */
    insert_timer(get_current_time() + ACTIVE_ROUTE_TIMEOUT, NULL, 0,
            get_my_ip_address(), 0, 0, timer_cleanup);
    return 0;
}

int init_timer_queue()
{
    /* Initialize the timer */
    init_timer(&mesh_timer);

    /* Initialize the queue */
    timer_queue = NULL;

    return 0;
}

void cleanup_timer_queue(void)
{
    /* Remove the timer */
    del_timer(&mesh_timer);
}

/* Convenience function to convert time value struct to jiffies */
static unsigned long tvtojiffies(struct timeval *value)
{
    unsigned long sec = (unsigned) value->tv_sec;
    unsigned long usec = (unsigned) value->tv_usec;

    if (sec > (ULONG_MAX / HZ))
        return ULONG_MAX;
    usec += 1000000 / HZ - 1;
    usec /= 1000000 / HZ;
    return HZ*sec+usec;
}

/* Given a time, return an entry that is due and remove it from the queue */
struct timer_queue_entry *find_first_timer_queue_entry_due(u_int64_t tv)
{
    struct timer_queue_entry *tmp_entry;

    /* Get a write lock on the queue */
    timer_write_lock();

    /* If there are timers... */
    if (timer_queue != NULL) {
        /* ...and they are due... */
        if ((timer_queue->tv) < tv) {
            /* then remove it from the queue */
            tmp_entry = timer_queue;
            timer_queue = timer_queue->next;

            /* Release the write lock */
            timer_write_unlock();

            /* Return it */
            return tmp_entry;
        }
    }

    /* Release the write lock */
    timer_write_unlock();

    /* Nothing to return */
    return NULL;
}

/* Callback function for expired timer */
void timer_queue_signal(unsigned long dummy)
{
    struct timer_queue_entry *tmp_entry;
    u_int64_t currtime;

    /* Get the current time */
    currtime = get_current_time();

    /* Start with the first entry that needs to be handled */
    tmp_entry = find_first_timer_queue_entry_due(currtime);

    /* While there are more events that need to be handled */
    while (tmp_entry != NULL) {
        /* Execute its callback */
        if (tmp_entry->func) {
            tmp_entry->func(tmp_entry);
        } else {
            printk(KERN_ERR "No timer function defined.\n");
        }
        /* Free the memory associated with the event */
        kfree(tmp_entry);

        /* Get the time again */
        currtime = get_current_time();

        /* and get the next event that needs to be handled */
        tmp_entry = find_first_timer_queue_entry_due(currtime);
    }

    /* Now update the queue to determine how long to sleep */
    update_timer_queue();
}

/* Update the timer after the state has changed */
void update_timer_queue()
{
    struct timeval delay_time;
    u_int64_t currtime;
    u_int64_t tv;
    u_int64_t remainder, numerator;

    delay_time.tv_sec = 0;
    delay_time.tv_usec = 0;

    /* Read lock the queue */
    timer_read_lock();

    if (timer_queue == NULL) {
        /* No reason to set the timer */
        delay_time.tv_sec = 0;
        delay_time.tv_usec = 0;
    } else {
        /* Look at the first queue entry */
        tv = timer_queue->tv;

        /* Get the current time */
        currtime = get_current_time();

        /* If the event has already occurred, set the timeout to 1 ms */
        if (tv <= currtime) {
            delay_time.tv_sec = 0;
            delay_time.tv_usec = 1;
        } else {
            /* Otherwise set the timer to te first event */
            numerator = ( tv - currtime );
            remainder = do_div( numerator, 1000 );

            delay_time.tv_sec =  numerator;
            delay_time.tv_usec = remainder * 1000;
        }
    }

    /* If there is no timer active... */
    if (!timer_pending(&mesh_timer)) {
        /* Set the callback function */
        mesh_timer.function = &timer_queue_signal;
        /* Set the sleep time */
        mesh_timer.expires = jiffies + tvtojiffies(&delay_time);
        /* And add the timer */
        add_timer(&mesh_timer);
    } else {
        /* Just modify the existing timer */
        mod_timer(&mesh_timer,jiffies + tvtojiffies(&delay_time));
    }

    /* Release the read lock */
    timer_read_unlock();

    return;
}

/* Insert a new timer into the queue */
int insert_timer(u_int64_t msec, void *data, int size, u_int32_t id,
        u_int16_t retries, u_int8_t ttl, timer_func_ptr func)
{
    struct timer_queue_entry *prev_entry = NULL;
    struct timer_queue_entry *entry;
    struct timer_queue_entry *new_entry;

    /* Allocate memory for the new entry */
    if ((new_entry = kmalloc(sizeof(struct timer_queue_entry), GFP_ATOMIC))
            == NULL)
    {
        /* Inform the user of the error */
        printk(KERN_WARNING "Unable to allocate a new timer.\n");
        /* Return the error */
        return -ENOMEM;
    }

    /* Initialize the new entry with the passed-in data */
    new_entry->tv = msec;
    new_entry->size = size;
    new_entry->id = id;
    new_entry->ttl = ttl;
    new_entry->retries = retries;
    new_entry->data = data;
    new_entry->func = func;

    /* Get a write lock so we can add it */
    timer_write_lock();

    /* Find the proper place to add the entry into the list */
    entry = timer_queue;
    while (entry != NULL && new_entry->tv > entry ->tv) {
        prev_entry = entry;
        entry = entry->next;
    }

    /* If the previous search points to the head, add it to the head */
    if (timer_queue == entry) {
        new_entry->next = timer_queue;
        timer_queue = new_entry;
    } else {
        /* And if it points to the tail, add it to the tail */
        if (entry == NULL) {
            new_entry->next = NULL;
            prev_entry->next = new_entry;
        } else {
            /* Or at the location the search returned */
            new_entry->next = prev_entry->next;
            prev_entry->next = new_entry;
        }
    }

    /* Release the write lock */
    timer_write_unlock();

    return 0;
}

/* Find a timer given an id */
struct timer_queue_entry * find_first_timer_queue_entry_of_id(u_int32_t id)
{
    struct timer_queue_entry *entry;

    /* Get a read lock */
    timer_read_lock();

    /* Start at the head */
    entry = timer_queue;

    /* Traverse and entries and try to match the ids */
    while (entry != NULL && entry->id != id) {
        /* Move to the next entry */
        entry = entry->next;
    }

    /* Release the read lock */
    timer_read_unlock();

    return entry;
}

/* Find a timer given an id and handler */
struct timer_queue_entry *
find_first_timer_queue_entry_of_id_and_func(u_int32_t id, timer_func_ptr func)
{
    struct timer_queue_entry *entry;

    /* If both arguments are unspecified, return the head */
    if (id == 0 && func == 0) {
        return timer_queue;
    }

    /* Get a read lock */
    timer_read_lock();

    /* Start at the head */
    entry = timer_queue;

    /* Traverse and entries and try to match the ids and handlers */
    while (entry != NULL && entry->id != id && entry->func != func) {
        /* Move to the next entry */
        entry = entry->next;
    }

    /* Release the read lock */
    timer_read_unlock();

    return entry;
}

void delete_timer(u_int32_t id, timer_func_ptr func)
{
    struct timer_queue_entry *entry;
    struct timer_queue_entry *prev_entry = NULL;
    struct timer_queue_entry *dead_entry;
    int deleted = 0;

    /* Get a write lock */
    timer_write_lock();

    /* Start at the head */
    entry = timer_queue;

    /* Traverse all entries */
    while (entry != NULL) {
        /* If we have a match... */
        if (entry->id == id && entry->func == func) {
            /* ...we need to remove it from the list */
            if (prev_entry == NULL) {
                /* If it was the first entry, update the head */
                timer_queue = entry->next;
            } else {
                /* Otherwise connect the previous and next entry */
                prev_entry->next = entry->next;
            }

            /* Keep a pointer to this entry */
            dead_entry = entry;

            /* Move to the next entry */
            entry = entry->next;

            /* Free the memory used by this entry */
            kfree(dead_entry->data);
            kfree(dead_entry);

            /*
             * Remember that we change the list, so that we call
             * update_timer_queue() when we're done
             */
            deleted=1;
        } else {
            /* Move on to the next entry */
            prev_entry = entry;
            entry = entry->next;
        }
    }

    /* Release the write lock */
    timer_write_unlock();

    /* Adjust the timer if there was a change */
    if (deleted) update_timer_queue();
}

