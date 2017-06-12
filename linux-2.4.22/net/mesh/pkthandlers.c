/*
 *   Mesh network routing
 *
 *      $Id: pkthandlers.c,v 1.3 2004/01/14 17:36:17 vangool Exp $
 *
 */
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/netfilter_ipv4.h>
#include <net/checksum.h>

#include "pkthandlers.h"
#include "multicast_queue.h"
#include "iflist.h"
#include "mc_cache.h"
#include "mesh.h"
#include "messages.h"
#include "utils.h"

/* Netfilter hooks */
struct nf_hook_ops input_filter;
struct nf_hook_ops output_filter;
struct nf_hook_ops forward_filter;

/*
 * Perform a quick sanity check on this (possible) AODV message. These
 * checks are based on the 15 July 2000 draft of manet-maodv-01
 * (draft-ietf-manet-maodv-00.txt) and the 17 February 2003 draft of AODV
 * (draft-ietf-manet-aodv-13.txt).
 */
int aodv_sanity_check(int type, void *data_in, int size)
{
    int *data = data_in;

    /*
     * All we can really do is check if it is a valid type and that the
     * size is the correct one given the message type
     */
    if (type == 1) {
        /* This is a RREQ message */
        if (size == 24) {
           /* This is a normal RREQ message (no extensions) */
            return 0;
        }
        /*
         * If it's an extension it should at least be larger than 26
         * bytes since byte 25 and 26 are type and length, resp.
         */
        if (size <= 26) {
            /* Smaller than RREQ plus minimum extension header */
            return 1;
        }
        if (data[25] == 3 && data[26] == 8 && size == 34) {
            /* Multicast Group Leader Extension (see sect. 10.1) */
            return 0;
        }
        if (data[25] == 4 && data[26] == 2 && size == 28) {
            /* Multicast Group Rebuild Extension (see sect. 10.2) */
            return 0;
        }
    } else if (type == 2) {
        /* This is a RREP message */
        if (size == 20) {
            /* This is a normal RREP message (no extensions) */
            return 0;
        }
        /*
         * If it's an extension it should at least be larger than 22
         * bytes since byte 21 and 22 are type and length, resp.
         */
        if (size <= 22) {
            /* Smaller than RREP plus minimum extension header */
            return 1;
        }
        if (data[21] == 5 && data[22] == 6 && size == 28) {
            /* Multicast Group Information Extension (see 10.3) */
            return 0;
        }
    } else if (type == 3) {
        /* This is a RERR message */
        int dstcount;

        /* Get the number of destinations in the RERR message */
        dstcount = ((struct rerr*)data_in)->dst_count;

        if (size == 4 + 8 * dstcount) {
            return 0;
        }
    } else if (type == 4) {
        /* This is a RREP-ACK message */
        if (size == 2) {
            return 0;
        }
    } else if (type == 5) {
        /* This is a MACT message */
        if (size == 16) {
            return 0;
        }
    } else if (type == 6) {
        /* This is a GRPH message */
        if (size == 20) {
            return 0;
        }
    }

    /* Never heard of this type of message */
    return 1;
}

int process_packet(struct sk_buff *packet)
{
    struct iphdr *ip = packet->nh.iph;
    struct net_device *dev = packet->dev;
    struct rtable_entry *tmp_route;
    u_int32_t dev_ip;
    u_int8_t aodv_type;
    int start_point;

    /* Look up the IP address for the device we received the packet on */
    dev_ip = dev2ip(dev);

    /* No need to process my own AODV messages */
    if (dev_ip == ip->saddr) {
        return NF_DROP;
    }

    /* If this is not a broadcast message... */
    if (ip->daddr!=INADDR_BROADCAST) {
        /* We should have a route for it then, so get it */
        tmp_route = find_route_table_entry(ip->daddr);

        /*
         * If there is no such route, or it is marked invalid, the
         * destination is unreachable
         */
        if ((tmp_route == NULL) || !tmp_route->route_valid) {
            /* Inform the user */
            printk( KERN_NOTICE "Destination %s unreachable\n",
                    inet_ntoa(ip->daddr)); 
            /* Send out the error */
            host_unreachable(ip->daddr);
        }
    }

    /* Start after the IP and UDP headers */
    start_point = sizeof(struct iphdr) + sizeof(struct udphdr);

    /* Get the type */
    aodv_type = (int)packet->data[start_point];

    /* Verify the packet is a valid AODV message */
    if (aodv_sanity_check(aodv_type, packet->data+start_point,
                packet->len-start_point))
    {
        /* Inform the user */
        printk(KERN_NOTICE "Type %d and size %u is invalid AODV message\n",
                aodv_type, packet->len-start_point);
        /* Not much else to do than to drop the packet */
        return NF_DROP;

    }

    /* Need to 'touch' the route entry to the source to keep it alive */
    tmp_route = find_route_table_entry(ip->saddr);
    if ((tmp_route != NULL) && (tmp_route->route_valid)) {
        tmp_route->lifetime = (HELLO_INTERVAL * ALLOWED_HELLO_LOSS) +
                get_current_time();
    }

    /* Hand it off to the AODV thread for proper processing */
    insert_message_entry(aodv_type, packet);

    /* We accepted the message */
    return NF_ACCEPT;
}

/* The input handler called by netfilter */
unsigned int input_handler(unsigned int hooknum, struct sk_buff **skb,
        const struct net_device *in, const struct net_device *out,
        int (*okfn)(struct sk_buff *))
{
    struct iphdr *ip = (*skb)->nh.iph;  
    struct udphdr *hdr = (struct udphdr *)(ip + ip->ihl);
    struct ethhdr *mac = (*skb)->mac.ethernet;

    /* If it's a multicast packet that I didn't send... */
    if ((ip != NULL) && IN_MULTICAST(ip->daddr) &&
            (ip->saddr != get_my_ip_address()))
    {
        /* Verify we didn't send it out before */
        if (check_mc_cache(ip->saddr, ntohs(ip->id))) {
            /* Insert into the queue */
            insert_multicast_message(ip->daddr, (*skb)->len, (*skb)->data,
                    ip->ttl);
        } else {
            /* We already sent this message. No need to send it again */
            return NF_DROP;
        }

    }

    /* Check for UDP headers */
    if ((*skb)->h.uh != NULL) {
        /* Was it a UDP/IP packet for our AODV port? */
        if ((hdr->dest == htons(AODVPORT)) &&
                (mac->h_proto == htons(ETH_P_IP)))
        {
            return process_packet(*(skb));
        }
    }

    /* Seems a normal packet */
    return NF_ACCEPT;
}

/* Unique identifier for tagging outbound multicast packets */
static u_int16_t unique_id = 0;

/* Netfilter handler that gets called for all out-bound packets */
unsigned int output_handler(unsigned int hooknum, struct sk_buff **skb,
        const struct net_device *in, const struct net_device *out,
        int (*okfn)(struct sk_buff *))
{
    struct iphdr *ip = (*skb)->nh.iph;
    struct rtable_entry  *tmp_route;

    /* If it's a local process sending it... */
    if (hooknum == NF_IP_LOCAL_OUT) {
        /* And it's an non-tagged 'alive' multicast message */
        if (IN_MULTICAST(ip->daddr) && (ip->id==0) && (ip->ttl!=0)) {
            /* Tag it */
            unique_id++;
            ip->id = htons(unique_id);

            /* Recalculate the IP checksum since we changed ip->id */
            ip->check = 0;
            ip->check = ip_fast_csum((unsigned char *)ip, ip->ihl);
        }
    }

    /* For now multicast and broadcast messages go right through */
    if (IN_MULTICAST(ip->daddr) || (ip->daddr == INADDR_BROADCAST)) {
        return NF_ACCEPT;
    }

    /* Try to get a route to the destination */
    tmp_route = find_route_table_entry(ip->daddr);

    /*
     * If we don't have a route, or the route we have is invalid, we need
     * to establish a route
     */
    if ((tmp_route == NULL) || !(tmp_route->route_valid)) {
        /* Try to create a route from me to the destination */
        send_rreq(get_my_ip_address(), ip->daddr);
        /* Queue the message until we have a route */
        return NF_QUEUE;
    }

    /* If we have a route that is valid, update its lifetime */
    if ((tmp_route != NULL) && (tmp_route->route_valid)) {
        tmp_route->lifetime = MAX(tmp_route->lifetime,
                get_current_time() + ACTIVE_ROUTE_TIMEOUT);
    }

    /* We accept the message */
    return NF_ACCEPT;
}

/* Set up the netfilter hooks*/
int init_pkthandlers(void)
{
    printk("Setting up packet handlers\n");

    /* Input hook */
    input_filter.list.next = NULL;
    input_filter.list.prev = NULL;
    input_filter.hook = input_handler;
    input_filter.pf = PF_INET;
    input_filter.hooknum = NF_IP_PRE_ROUTING;

    /* Output hook */
    output_filter.list.next = NULL;
    output_filter.list.prev = NULL;
    output_filter.hook = output_handler;
    output_filter.pf = PF_INET;
    output_filter.hooknum = NF_IP_LOCAL_OUT;

    /* Forward hook */
    forward_filter.list.next = NULL;
    forward_filter.list.prev = NULL;
    forward_filter.hook = output_handler;
    forward_filter.pf = PF_INET;
    forward_filter.hooknum = NF_IP_FORWARD;

    /* Register the filters */
    if (nf_register_hook(&output_filter))
        return 1;
    if (nf_register_hook(&input_filter))
        return 1;
    if (nf_register_hook(&forward_filter))
        return 1;

    /* All is well */
    return 0;
}

/* Unregister netfilter hooks */
void deinit_pkthandlers(void)
{
    printk("Cleaning up packet handlers\n");

    /* Unregister the packet handlers */
    nf_unregister_hook(&input_filter);
    nf_unregister_hook(&output_filter);
    nf_unregister_hook(&forward_filter);
}

