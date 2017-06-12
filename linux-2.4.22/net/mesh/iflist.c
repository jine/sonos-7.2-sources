#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/rtnetlink.h>

#include "iflist.h"
#include "messages.h"
#include "module.h"

struct if_entry *if_list;

struct rtable_entry *my_route;
u_int32_t my_ip;

u_int32_t get_my_ip_address(void)
{
    return my_ip;
}

struct rtable_entry* get_my_route_entry(void)
{
    return my_route;
}

int init_iflist()
{
    struct rtable_entry *tmp_route;
    struct if_entry *new_if_entry = NULL;
    struct net_device *net_dev_ptr;
    struct in_device *in_dev_ptr;
    int res;

    /* Initialize list */
    if_list = NULL;

    /*
     * We now have to iterate over all known network devices. This
     * requires proper locking of kernel structures so the data doesn't
     * get changed from underneath us.
     */
    read_lock(&dev_base_lock);

    /* We particularly want to lock the associated IP network devices */
    read_lock(&inetdev_lock);

    /* Point to the kernel table of network devices and iterate */
    for (net_dev_ptr = dev_base; net_dev_ptr; net_dev_ptr=net_dev_ptr->next) {
        /* Check to see it's an IP network device (not Appletalk, etc.) */
        if ((in_dev_ptr = __in_dev_get(net_dev_ptr)) == NULL)
            /* We're not interested in this one */
            continue;

        /* Lock the device */
        read_lock(&in_dev_ptr->lock);

        /* Now iterate over the interfaces provided by the device */
        for_primary_ifa(in_dev_ptr) {

            /* Ignore local loopback */
            if (strcmp(net_dev_ptr->name, "lo") != 0) {
                /* Allocate memory for the interface entry */
                if ((new_if_entry = kmalloc(sizeof(struct if_entry),
                                GFP_ATOMIC)) == NULL)
                {
                    return -ENOMEM;
                }

                tmp_route = create_route_table_entry();
                tmp_route->dst_ip = ifa->ifa_address;
                printk(KERN_INFO "Adding interface %s (%s)\n",
                        net_dev_ptr->name, inet_ntoa(ifa->ifa_address));

                /* Fill in the route info */
                tmp_route->static_route = 1;
                tmp_route->dst_seq = 1;
                tmp_route->rreq_id = 1;
                tmp_route->hop_count = 0;
                tmp_route->next_hop = tmp_route->dst_ip;
                tmp_route->lifetime = -1;
                tmp_route->route_valid = 1;
                tmp_route->route_seq_valid = 1;
                tmp_route->precursors = NULL;
                tmp_route->dev = net_dev_ptr;

                my_route = tmp_route;
                my_ip = my_route->dst_ip;

                /* Fill in the if data */
                strncpy(new_if_entry->name, net_dev_ptr->name, IFNAMSIZ);
                new_if_entry->ip = tmp_route->dst_ip;
                new_if_entry->route_entry = tmp_route;
                new_if_entry->next = if_list;
                new_if_entry->dev = net_dev_ptr;

                /* Update interface list */
                if_list = new_if_entry;

                /* Create socket to use for mesh protocol traffic */
                res = sock_create(PF_INET,SOCK_DGRAM, 0, &(new_if_entry->sock));
                if (res < 0) {
                    /* Unlink entry */
                    if_list = new_if_entry->next;

                    /* Something went wrong. Clean up the mess */
                    kfree(tmp_route);
                    kfree(new_if_entry);
                    return res;
                }

                /* Set it up */
                init_sock(new_if_entry->sock, new_if_entry->ip,
                        net_dev_ptr->name);

                /* Enter a kernel route to ourselves */
                add_kroute(tmp_route->dst_ip, tmp_route->dst_ip,
                        new_if_entry->name);

                /* Start sending hello's if we're an active module (ZP) */
                if (!is_passive()) start_hello(new_if_entry->ip);
            }
        } endfor_ifa(in_dev_ptr);

        /* Unlock the device */
        read_unlock(&in_dev_ptr->lock);
    }

    /* Unlock kernel data structures */
    read_unlock(&inetdev_lock);
    read_unlock(&dev_base_lock);

    return 0;
}

void deinit_iflist(void)
{
    struct if_entry *entry = if_list;

    /* Traverse the list */
    while (entry != NULL) {
        /* Delete the route data */
        kfree(entry->route_entry);

        /* And then the interface itself */
        kfree(entry);

        /* Move to the next entry */
        entry = entry->next;
    }
}

struct if_entry *get_first_interface(void)
{
    return if_list;
}

/* Return an IP address given a net_device */
u_int32_t dev2ip(struct net_device *dev)
{
    struct in_device *indev;

    /* Quick sanity check */
    if (dev == NULL || dev->ip_ptr == NULL) {
        return -EFAULT;
    }

    /* If present, return the IP address associated with this IP device */
    indev = (struct in_device*)dev->ip_ptr;
    if (indev->ifa_list != NULL) {
        /* It's got an address */
        return indev->ifa_list->ifa_address;
    } else {
        /* No address */
        return 0;
    }

    /* Should never get here */
    return 0;
}

struct if_entry *ip2if(u_int32_t ip)
{  
    struct if_entry *entry = if_list;

    /* Traverse the list */
    while (entry != NULL) {
        /* Compare the IP addresses */
        if (entry->ip == ip) {
            return entry;
        }

        /* Move to the next entry */
        entry = entry->next;
    }

    /* No such interface found */
    return NULL;
}

/* Return an interface entry given a net_device */
struct if_entry *dev2if(struct net_device *dev)
{
    u_int32_t ip;

    /* Get the IP address for this net device */
    ip = dev2ip(dev);

    /* Search the list for this IP address */
    if (ip > 0) {
        return ip2if(ip);
    } else {
        return NULL;
    }
}

