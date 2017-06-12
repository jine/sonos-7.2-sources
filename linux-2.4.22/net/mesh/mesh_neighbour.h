/*
 *   Mesh network routing
 *
 *      $Id: mesh_neighbour.h,v 1.2 2004/01/09 17:05:50 vangool Exp $
 *
 */
#ifndef MESH_NEIGHBOUR_H
#define MESH_NEIGHBOUR_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/if_ether.h>

/* Type of connection to neighbour */
enum mesh_link_type
{
    WIRED,
    WIRELESS
};

/* Info we keep track of regarding our neighbours */
struct mesh_neighbour
{
    /* Pointer to the next neighbour in this single-linked list */
    struct mesh_neighbour* next;

    /* The IP address */
    u_int32_t ip;

    /* The hardware address (MAC) */
    unsigned char hw_addr[ETH_ALEN];

    /* Signal strength to that neighbour */
    u_int8_t signal_strength;

    /* The device that tracks the signal strength for this neighbour */
    struct net_device *dev;

    /* Type of connection to this neighbour */
    enum mesh_link_type conn_type;
};

/* Initiate the list */
int init_neighbour_list(void);

/* Create a new neighbour */
struct mesh_neighbour *create_neighbour(u_int32_t ip, char *hw_addr,
        struct net_device *dev);

/* Delete a neighbour entry given an IP or MAC address */
int delete_neighbour(u_int32_t ip, char *hw_addr);

/*
 * Find a neighbour entry given an IP or MAC address. If both arguments are
 * 'empty', the head of the list is returned.
 */
struct mesh_neighbour *find_neighbour(u_int32_t ip, char *hw_addr);

#endif 
