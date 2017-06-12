#ifndef INTERFACE_LIST_H
#define INTERFACE_LIST_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>

#include "rtable.h"

/* The info we store for each IP network interface we find */
struct if_entry
{
    /* Pointer to the next entry in the list */
    struct if_entry *next;

    /* net_device info associated with the interface */
    struct net_device *dev;

    /* The IP address for this interface */
    u_int32_t ip;

    /* The name of the interface */
    char name[IFNAMSIZ];

    /* The entry in the internal route table for this interface */
    struct rtable_entry *route_entry;

    /* The socket we opened on this interface */
    struct socket *sock;
};

/* Setup and clean up the list */
int init_iflist(void);
void deinit_iflist(void);

/* Find an interface given its IP address */
struct if_entry *ip2if(u_int32_t ip);

/* Find an interface given its net_device */
struct if_entry *dev2if(struct net_device *dev);

/* Return the IP address for a given net_device */
u_int32_t dev2ip(struct net_device *dev);

/* Return the head of the interface list */
struct if_entry *get_first_interface(void);

/* Return the IP address of the machine */
u_int32_t get_my_ip_address(void);

/* Return a pointer to the default route entry for this machine */
struct rtable_entry* get_my_route_entry(void);

#endif 
