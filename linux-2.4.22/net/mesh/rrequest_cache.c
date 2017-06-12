/*
 *   Mesh network routing
 *
 *      $Id: rrequest_cache.c,v 1.2 2004/01/14 17:36:17 vangool Exp $
 *
 */
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include "rrequest_cache.h"
#include "utils.h"

struct rrequest_cache_entry *rreq_id_cache;
rwlock_t rreq_lock = RW_LOCK_UNLOCKED;

/*
 * Functions to (un)lock the cache for reading and writing
 */
void rreq_cache_read_lock(void)
{
    write_lock_bh(&rreq_lock);
}

void rreq_cache_read_unlock(void)
{
    write_unlock_bh(&rreq_lock);
}

void rreq_cache_write_lock(void)
{
    write_lock_bh(&rreq_lock);
}

void rreq_cache_write_unlock(void)
{
    write_unlock_bh(&rreq_lock);
}

int init_rrequest_cache( void )
{
    /* Initialize cache to known value */
    rreq_id_cache = NULL;

    /* That's all */
    return 0;
}

void cleanup_rrequest_cache()
{
    struct rrequest_cache_entry *entry, *dead_entry;

    /* Start from the beginning */
    entry = rreq_id_cache;

    /* Loop through all entries */
    while (entry != NULL) {
        /* Keep a pointer to the current entry */
        dead_entry = entry;
        /* Move to the next one */
        entry = entry->next;
        /* Free the entry */
        kfree(dead_entry);
    }

    /* Reset the cache to a known value */
    rreq_id_cache = NULL;
}

/* Search the cache for an entry with matching ID and src IP */
struct rrequest_cache_entry *find_rreq(u_int32_t src_ip, u_int32_t id )
{
    struct rrequest_cache_entry *entry;
    u_int64_t curr = get_current_time();

    /* Get a read lock for the cache */
    rreq_cache_read_lock();

    /* Start at the beginning of the cache */
    entry = rreq_id_cache;

    /* Loop through all entries in the cache */
    while (entry != NULL) {

        /* Check the validity of the entry */
        if (entry->lifetime < curr) {
            /* Release the read lock */
            rreq_cache_read_unlock();

            /* Nothing to return */
            return NULL;
        }

        /* Check if there is a match */
        if (src_ip == entry->src_ip && id == entry->id) {
            /* Release the read lock */
            rreq_cache_read_unlock();

            /* Return the entry */
            return entry;
        }

        /* Move on to the next entry */
        entry = entry->next;
    }
    /* Release the read lock */
    rreq_cache_read_unlock();

    /* Nothing to return */
    return NULL;
}

int insert_rreq(u_int32_t src_ip, u_int32_t dst_ip, u_int32_t id,
      u_int64_t lifetime)
{
    struct rrequest_cache_entry *entry;

    /* Allocate memory for the new entry */
    if ((entry = (struct rrequest_cache_entry*)
                kmalloc(sizeof(struct rrequest_cache_entry), GFP_ATOMIC))
            == NULL)
    {
        /* Inform the user of the error */
        printk(KERN_WARNING "Unable to allocate rreq cache entry.\n");

        /* Return the error */
        return -ENOMEM;
    }

    /* Fill in the information */
    entry->id = id;
    entry->src_ip = src_ip;
    entry->dst_ip = dst_ip;
    entry->lifetime = lifetime;

    /* Get a write lock for the cache */
    rreq_cache_write_lock();

    /* Add the entry to the head of the list */
    entry->next = rreq_id_cache;
    rreq_id_cache = entry;

    /* Release the write lock */
    rreq_cache_write_unlock();

    /* Return success */
    return 0;
}

int delete_old_rreqs(void) 
{
    struct rrequest_cache_entry *entry, *prev_entry, *dead_entry;
    u_int64_t  curr_time = get_current_time();
    int cnt = 0;

    /* Get a writelock for the cache */
    rreq_cache_write_lock( );

    /* Start at the beginning */
    entry = rreq_id_cache;
    prev_entry = NULL;

    /* Loop through the entire cache */
    while (entry != NULL) {
        /* If the entry has expired... */
        if (curr_time > entry->lifetime) {
            /* If it is the first entry... */
            if (prev_entry == NULL) {
                rreq_id_cache = entry->next;
            } else {
                prev_entry->next = entry->next;
            }

            /* Keep a pointer to the entry */
            dead_entry = entry;
            /* Move to the next entry */
            entry = entry->next;
            /* Clean up the entry */
            kfree(dead_entry);
            /* Increment the counter */
            cnt++;
        } else {
            /* Just move to the next entry */
            prev_entry = entry;
            entry = entry->next;
        }
    }

    /* Release the write lock */
    rreq_cache_write_unlock();

    /* Return the number of removed entries */
    return cnt;
}

