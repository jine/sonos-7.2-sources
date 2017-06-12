#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/net.h>
#include <linux/in.h>
#include <net/route.h>
#include <net/sock.h>
#include <asm/div64.h>
#include <asm/uaccess.h>

#include "rtable.h"
#include "pktqueue.h"
#include "utils.h"
#include "mesh.h"
#include "messages.h"

/* Default netmask for routes */
u_int32_t default_netmask;

rwlock_t rtable_lock = RW_LOCK_UNLOCKED;
struct rtable_entry *rtable;

/* Convenience functions for (read/write) locking the route table */
void route_read_lock(void)
{
    read_lock_bh(&rtable_lock);
}

void route_read_unlock(void)
{
    read_unlock_bh(&rtable_lock);
}

void route_write_lock(void)
{
    write_lock_bh(&rtable_lock);
}

void route_write_unlock(void)
{
    write_unlock_bh(&rtable_lock);
}

struct rtentry *create_kroute_entry(u_int32_t dst_ip, u_int32_t gw_ip,
        char *interface)
{
    struct rtentry *entry;
    struct sockaddr_in dst;
    struct sockaddr_in gateway;
    struct sockaddr_in genmask;

    /* Allocate memory for the entry */
    if ((entry = kmalloc(sizeof(struct rtentry), GFP_ATOMIC)) == NULL) {
        /* Failed to allocate memory */
        return NULL;
    }

    /* Target address */
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = dst_ip;

    /* Gateway address */
    gateway.sin_family = AF_INET;
    gateway.sin_addr.s_addr = gw_ip;

    /* Target network mask */
    genmask.sin_family = AF_INET;
    genmask.sin_addr.s_addr = default_netmask;

    /* Fill in the entry */
    entry->rt_dst = *(struct sockaddr*)&dst;
    entry->rt_gateway = *(struct sockaddr*)&gateway;
    entry->rt_genmask = *(struct sockaddr*)&genmask;
    entry->rt_flags = RTF_UP | RTF_HOST | RTF_GATEWAY;
    entry->rt_metric = 0;
    entry->rt_dev = interface;

    /* Return it */
    return entry;
}

int add_kroute(u_int32_t dst_ip, u_int32_t gw_ip, char *interface)
{
    struct rtentry *krtentry;
    mm_segment_t oldfs;
    int res = 0;

    /* Allocate memory for the entry and initialize it */
    if ((krtentry = create_kroute_entry(dst_ip, gw_ip, interface)) == NULL) {
        /* We failed to allocate memory for the entry */
        return 1;
    }

    /* Preserve DS and FS segments (Intel only) */
    oldfs = get_fs();
    set_fs(get_ds());

    /* Add the entry to the kernel route table */
    res = ip_rt_ioctl(SIOCADDRT, (char*)krtentry);

    /* Restore DS and FS segments (Intel only) */
    set_fs(oldfs);

    /* Free allocated entry */
    kfree(krtentry);

    return res ? 1 : 0;
}

int delete_kroute(u_int32_t dst_ip, u_int32_t gw_ip)
{
    struct rtentry *krtentry;
    mm_segment_t oldfs;
    int res = 0;

    /* Allocate memory for the entry and initialize it */
    if ((krtentry = create_kroute_entry(dst_ip, gw_ip, NULL)) == NULL) {
        /* We failed to allocate memory for the entry */
        return 1;
    }

    /* Preserve DS and FS segments (Intel only) */
    oldfs = get_fs();
    set_fs(KERNEL_DS);

    /* Delete the entry from the kernel route table */
    res = ip_rt_ioctl(SIOCDELRT, (char*)krtentry);

    /* Restore DS and FS segments (Intel only) */
    set_fs(oldfs);

    /* Free allocated entry */
    kfree(krtentry);

    return res ? 1 : 0;
}

int init_route_table(void)
{
    /* Set default netmask */
    inet_aton("255.255.255.255", &default_netmask);

    /* Set route table to known value */
    rtable = NULL;

    return 0;
}

struct rtable_entry* rtable_head(void)
{
    return rtable;
}

/* Clean up the internal route list */
void remove_inactive_routes(void)
{
    struct rtable_entry *tmp_route;
    struct rtable_entry *dead_route;
    u_int64_t curr_time;

    /* Get the current time for comparison */
    curr_time = get_current_time();

    /* Lock the route table */
    route_write_lock();

    /* Start at the beginning */
    tmp_route = rtable_head();

    /* And iterate over the link of route entries */
    while (tmp_route != NULL) {
        /*
         * If the route has expired and it's not a static-route (static routes
	 * do not expire)
         */
        if ((tmp_route->lifetime < curr_time) && !tmp_route->static_route) {

            /*
             * If the route is not valid, we can simply remove it from the
             * list and be done. But if the route is valid, we also have to
             * remove the kernel route and will potentially have to send out
             * a RERR message.
             */
            if(!tmp_route->route_valid) {
                /* Keep a pointer to the entry */
                dead_route = tmp_route;

                /* Go to the next entry */
                tmp_route = tmp_route->next;

                /* If this was the head, just update the head */
                if (rtable == dead_route)
                    rtable = dead_route->next;

                /* Update the node before this one */
                if (dead_route->prev != NULL)
                    dead_route->prev->next = dead_route->next;

                /* Update the node after this one */
                if (dead_route->next != NULL)
                    dead_route->next->prev = dead_route->prev;

                /* Free the associated memory */
                delete_precursors_from_route(dead_route);
                kfree(dead_route);
            } else {
                /* We're here because the route was valid */

                /* Only if it's not a static route */
                if (tmp_route->static_route == 0) {
                    /*
                     * If the next hop is the destination, it's my
                     * neighbour. If this route expired, it's up to me to
                     * send out RERRs.
                     */
                    if (tmp_route->next_hop == tmp_route->dst_ip) {
                        /* Temporarily unlock the table */
                        route_write_unlock();
                        /* Update broken routes and remove from the kernel */
                        link_break(tmp_route->dst_ip);
                        /* And lock it again */
                        route_write_lock();

                    } else {
                        /* Temporarily unlock the table */
                        route_write_unlock();
                        /*
                         * Not our job to send out RERRs so just remove the
                         * route from the kernel
                         */
                        expire_route(tmp_route);
                        /* And lock it again */
                        route_write_lock();
                    }
                }
                /* Go to the next entry */
                tmp_route = tmp_route->next;
            }
        } else {
            /* Just go to the next entry */
            tmp_route = tmp_route->next;
        }
    }

    /* Unlock the table */
    route_write_unlock();
}

/* Create and initialize a new entry */
struct rtable_entry *create_route_table_entry()
{
   struct rtable_entry *entry;

   /* Allocate memory */
   if((entry = (struct rtable_entry*)kmalloc(sizeof(struct rtable_entry),
                   GFP_ATOMIC)) == NULL)
   {
       /* Inform the user of the error */
       printk(KERN_WARNING "Unable to allocate memory for new route entry\n");

       /* Return error */
       return NULL;
   }

   /* Initialize the entry */
   entry->precursors = NULL;
   entry->static_route = FALSE;
   entry->rreq_id = 0;
   entry->link = 255;
   entry->dev = NULL;
   entry->route_valid = FALSE;
   entry->route_seq_valid = FALSE;
   entry->prev = NULL;

   /* Lock the table so we can add it */
   route_write_lock();

   /* Add it at the head of the list */
   if (rtable != NULL)
      rtable->prev = entry;
   entry->next = rtable;
   rtable = entry;

   /* Unlock the table */
   route_write_unlock();

   /* Return the entry */
   return entry;
}

/* Find the route entry given an IP address (no locking) */ 
struct rtable_entry *fast_find_route_table_entry(u_int32_t ip_addr)
{
   struct rtable_entry *entry;

   /* Start at the head */
   entry = rtable;

   /* Traverse the list */
   while (entry != NULL) {
      /* Check if this is the one we want */
      if (entry->dst_ip == ip_addr) {
         /* Return it */
	 return entry;
      }
      /* Move to the next entry */
      entry = entry->next;
   }

   /* Return failure */
   return NULL;
}

struct rtable_entry *find_route_table_entry(u_int32_t ip_addr)
{
   struct rtable_entry *entry;

   /* Lock the table */
   route_read_lock();

   /* Find it */
   entry = fast_find_route_table_entry(ip_addr);

   /* Unlock the table */
   route_read_unlock();

   return entry;
}

/* Delete an entry from the table given an IP address */ 
int delete_route_table_entry(u_int32_t ip_addr)
{
   struct rtable_entry *entry;

   /* Try to find it first */
   if ((entry = find_route_table_entry(ip_addr)) == NULL) {
      /* It's not there */
      return -ENOENT;
   }

   /* Remove it from the kernel */
   delete_kroute(entry->dst_ip, entry->next_hop);

   /* Lock the table so we can remove the entry */
   route_write_lock();    

   /* If it's the head update the head */
   if (rtable == entry)
      rtable = entry->next;

   /* Update the node before this one */
   if (entry->prev != NULL)
      entry->prev->next = entry->next;

   /* Update the node after this one */
   if (entry->next != NULL)
      entry->next->prev = entry->prev;

   /* Unlock the table */
   route_write_unlock();    

   /* Free the associated memory */
   delete_precursors_from_route(entry);
   kfree(entry);

   /* Return success */
   return 0;
}

/* Clean up this mess */
void cleanup_route_table(void)
{
   struct rtable_entry *entry;
   struct rtable_entry *dead_entry;

   /* Start at the beginning */
   entry = rtable;

   /* And traverse the list */
   while (entry != NULL) {
      /* Remove the route from the kernel */
      if (delete_kroute(entry->dst_ip, entry->next_hop) != 0)
          printk(KERN_WARNING "Unable to remov kernel route (%s)\n",
                  inet_ntoa((__u32) entry->dst_ip));

      /* Keep a pointer to the entry */
      dead_entry = entry;

      /* Move to the next entry */
      entry = entry->next;

      /* Clean up the mess */
      delete_precursors_from_route(dead_entry);
      kfree(dead_entry);
   }

   /* Set the table to a know value */
   rtable = NULL;
}

/* Add a precursor to a route */
int add_precursor(struct rtable_entry *entry, u_int32_t ip_addr)
{
    struct precursor_entry *precursor;

    /* First check if it isn't already in the list */
    if (find_precursor(entry, ip_addr) == NULL) {
        /* It wasn't there. Now malloc memory for the new entry */
        if ((precursor = kmalloc(sizeof(struct precursor_entry), GFP_ATOMIC))
                == NULL)
        {
            /* Inform user of the error */
            printk(KERN_WARNING "Unable to allocate memory for precursor\n");

            /* Return error */
            return -ENOMEM;
        }

        /* Fill in the data */
        precursor->ip = ip_addr;
        precursor->prev = NULL;
        precursor->next = entry->precursors;

        /* Add the new entry to the list */
        if (entry->precursors != NULL)
            entry->precursors->prev = precursor;
        entry->precursors = precursor;

        /* Return success */
        return 0;
    }

    /* Return some kind of error */
    return -EINVAL;
}

/* Delete a precursor from a route */
void delete_precursor(struct rtable_entry *entry, u_int32_t ip_addr)
{
    struct precursor_entry *precursor;

    /* First, try to find the entry */
    if ((precursor = find_precursor(entry, ip_addr)) != NULL) {
        /* Update the node before this one */
        if (precursor->prev != NULL)
            precursor->prev->next = precursor->next;
        /* Update the node after this one */
        if (precursor->next != NULL)
            precursor->next->prev = precursor->prev;
        /* Potentially update the head */
        if (entry->precursors == precursor)
            entry->precursors = precursor->next;
        /* Free the memory associated with the entry */
        kfree(precursor);
    }
}

/* Removes a precursor from all routes */
void delete_precursor_from_routes(u_int32_t ip_addr)
{
   struct rtable_entry *entry;

   /* Lock the table */
   route_read_lock();    

   /* Start at the head */
   entry = rtable;

   /* Traverse the list */
   while (entry != NULL) {
      /* Delete the precursor from this route entry */
      delete_precursor(entry, ip_addr);
      /* Move to the next entry */
      entry = entry->next;
   }

   /* Unlock the table */
   route_read_unlock();    
}

/* Tries to find a precursor in a route entry */
struct precursor_entry* find_precursor(struct rtable_entry *entry,
        u_int32_t ip_addr)
{
    struct precursor_entry *precursor;

    /* Start at the beginning */
    precursor = entry->precursors;

    /* Traverse the list */
    while (precursor != NULL) {
        /* Is this the one? */
        if (precursor->ip == ip_addr) {
            /* Return it */
            return precursor;
        }
        /* Move to the next one */
        precursor = precursor->next;
    }

    /* Return 'not found' */
    return NULL;
}

/* Update a route */
int update_route_entry(u_int32_t ip, u_int32_t next_hop_ip, u_int8_t hop_count,
        u_int32_t seq, struct net_device *dev)
{
   struct rtable_entry *entry;
   u_int64_t curr_time;

   /* See if the entry already exists */
   entry = find_route_table_entry(ip);

   /* If the route doesn't exist, or the new one is 'better'... */
   if ((entry == NULL) || (!entry->route_valid) ||
	 (!entry->route_seq_valid) ||
	 (seq_greater(seq , entry->dst_seq)) ||
	 ((seq == entry->dst_seq) &&
	  (hop_count < entry->hop_count)))
   {
      /* Create one if it didn't exist */
      if (entry == NULL) {
         /* Create new entry */
	 entry = create_route_table_entry();
         /* Did it work */
	 if (entry == NULL) {
            /* Return error */
	    return -ENOMEM;
         }
         /* Fill in some data */
	 entry->dst_ip = ip;
      } else {
         /*
          * If we get here it means the route already exists but we are
          * going to replace it with a new (and different) one. So we need
          * to delete the existing kernel route because we are going to add
          * a new one later on.
          */
	 delete_kroute(entry->dst_ip, entry->next_hop);
      }

      /* Fill in the rest of the data */
      entry->dst_seq = seq;
      entry->next_hop = next_hop_ip;
      entry->hop_count = hop_count;
      entry->dev = dev;
      entry->route_valid = TRUE;

      /* Add the kernel route */
      add_kroute(entry->dst_ip, entry->next_hop, entry->dev->name);

      /* Flush any packets we may have for that IP address */
      pktqueue_send_ip(ip);
   }

   /*
    * Either we created a new entry or the existing one never changed.
    * Either way we can update the lifetime of the route.
    */

   /* Get current time */
   curr_time = get_current_time();

   /* Update the entry */
   entry->lifetime = curr_time + ACTIVE_ROUTE_TIMEOUT;

   /* Return success */
   return 0;
}

/* Removes all precursors from a route entry */
void delete_precursors_from_route(struct rtable_entry *entry)
{
    struct precursor_entry *precursor;
    struct precursor_entry *dead_precursor;

    /* Start at the beginning of the list */
    precursor = entry->precursors;

    /* Iterate over all entries */
    while (precursor != NULL) {
        /* Keep a pointer to the entry */
        dead_precursor = precursor;
        /* Move to the next entry */
        precursor = precursor->next;
        /* Free the memory associated with this entry */
        kfree(dead_precursor);
    }

    /* Re-initialize the precursor list */
    entry->precursors = NULL;
}

/* Add a static route */
void add_static_route(char* ip, struct net_device *dev)
{
   u_int32_t ipaddr;
   struct rtable_entry *tmp_route;

   /* Add the new entry to the list */
   tmp_route = create_route_table_entry();

   /* Fill in the data */
   inet_aton(ip, &ipaddr);
   tmp_route->dst_ip = ipaddr;
   tmp_route->static_route = 1;
   tmp_route->dst_seq = 1;
   tmp_route->rreq_id = 1;
   tmp_route->hop_count = 0;
   tmp_route->next_hop = tmp_route->dst_ip;
   tmp_route->lifetime = -1;
   tmp_route->route_valid = 1;
   tmp_route->route_seq_valid = 1;
   tmp_route->precursors = NULL;
   tmp_route->dev = dev;

   /* Create the corresponding kernel route */
   add_kroute(tmp_route->dst_ip, tmp_route->dst_ip,
              dev->name);
}

