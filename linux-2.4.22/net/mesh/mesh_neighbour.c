/*
 *   Mesh network routing
 *
 *      $Id: mesh_neighbour.c,v 1.1 2004/01/08 21:31:24 vangool Exp $
 *
 */
#include <linux/slab.h>

#include "mesh_neighbour.h"
#include "mesh.h"
#include "timers.h"

struct mesh_neighbour* neighbour_head;

/* Initialize neighbour list */
int init_neighbour_list()
{
    /* Set head to NULL */
    neighbour_head = NULL;

    /* All is well */
    return 0;
}

struct mesh_neighbour* create_neighbour(u_int32_t ip, char *hw_addr,
        struct net_device *dev)
{
    struct mesh_neighbour *new_entry;

    if ((new_entry = kmalloc(sizeof(struct mesh_neighbour), GFP_ATOMIC))
            == NULL)
    {
        /* Inform user of the error */
        printk(KERN_WARNING "Unable to allocate new neighbour entry\n");
        return NULL;
    }

    /* Fill in the data */
    new_entry->ip = ip;
    new_entry->signal_strength = 255;
    new_entry->dev = dev;
    new_entry->conn_type =
            (dev->get_wireless_stats == NULL) ? WIRED : WIRELESS;
    memcpy(&(new_entry->hw_addr), hw_addr, ETH_ALEN);

    /* Add to the head of the list */
    new_entry->next = neighbour_head;
    neighbour_head = new_entry;

    /* Return pointer to the new neighbour */
    return new_entry;
}

/* Delete a neighbour from the list given its IP or MAC address */
int delete_neighbour(u_int32_t ip, char *hw_addr)
{  
    struct mesh_neighbour *entry = neighbour_head;
    struct mesh_neighbour *prev_entry = NULL;

    /* Sanity check */
    if (!ip && !hw_addr) {
        /* Inform user of the error */
        printk(KERN_WARNING "No valid input parameters to delete_neighbour\n");
        /* Nothing to do */
        return 0;
    }

    /* Traverse the list */
    while (entry != NULL) {
        /* Compare the IP/MAC addresses */
        if ((ip && entry->ip == ip) ||
            (hw_addr && !memcmp(&(entry->hw_addr), hw_addr, ETH_ALEN)))
        {

            /* Found it so update the list */
            if (prev_entry == NULL) {
                /* This was the first entry on the list */
                neighbour_head = entry->next;
            } else {
                /* Unlink this node from the list */
                prev_entry->next = entry->next;
            }

            /* Free the memory associated with this neighbour */
            kfree(entry);

            /* Remove the timer associated with this neighbour */
            delete_timer(ip, timer_neighbour);

            /* Recalculate timer sleep interval */
            update_timer_queue();

            /* Return success */
            return 0;
        }
        /* Keep a pointer to this one so we can repair the list */
        prev_entry = entry;

        /* Move on to the next one */
        entry = entry->next;
    }

    /* Return failure status */
    return -ENODATA;
}

/* Find neighbour info given an IP address or MAC address */
struct mesh_neighbour *find_neighbour(u_int32_t ip, char *hw_addr)
{
    struct mesh_neighbour *entry = neighbour_head;

    /* Check input parameters */
    if (!ip && !hw_addr) {
        /* Apparently any neighbour will do so return the head */
        return entry;
    }

    /* Traverse the neighbour list */
    while (entry != NULL) {
        /* Compare the IP/MAC addresses */
        if ((ip && entry->ip == ip) ||
            (hw_addr && !memcmp(&(entry->hw_addr), hw_addr, ETH_ALEN)))
        {
            /* Return it */
            return entry;
        }

        /* Move on to the next one */
        entry = entry->next;
    }

    /* We didn't find it */
    return NULL;
}

