/*
 *	Forwarding decision
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_forward.c,v 1.1.1.1 2006/12/23 00:48:21 holmgren Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/skbuff.h>
#include <linux/if_bridge.h>
#include <linux/if_ether.h>
#include <linux/netfilter_bridge.h>
#include "br_private.h"

/* REVIEW:
 *
 * __br_skb_*: This uses skb->cb to stash data in.  This conflicts with
 * netfilter, but we (Sonos) do not use netfilter (and we've already broken it
 * in other ways).  Looks like cb[4] is safe to use even w/ netfilter, but
 * whatever...
 */
static inline void __br_skb_mark_direct(struct sk_buff *skb, int direct)
{
	skb->cb[0] = direct;
}

static inline int __br_skb_marked_direct(struct sk_buff *skb)
{
	return skb->cb[0];
}


/* if the specified port is disabled, or if the packet arrived on the 
   specified port in the first place, don't forward it */
static inline int should_deliver(struct net_bridge_port *p_arriving, 
                                 struct net_bridge_port *p_outgoing)
{
    if (p_outgoing->state != BR_STATE_FORWARDING 
        || p_outgoing->remote_state == BR_STATE_BLOCKING
        || p_arriving == p_outgoing)
        return 0;

    return 1;
}

static int __br_encap_skb(struct sk_buff *skb,
			  const unsigned char *dest,
			  const unsigned char *src,
			  unsigned short proto)
{
	struct ethhdr *ether;

	/* Get some space for another header */
	if ((skb_headroom(skb) < ETH_HLEN) && 
	    (pskb_expand_head(skb, ETH_HLEN - skb_headroom(skb), 0, 
			      GFP_ATOMIC))) {
		dev_kfree_skb(skb);
		return -ENOMEM;
	}

	ether = (struct ethhdr*)skb_push(skb, ETH_HLEN);

	/* Set up the addresses and proto */
	memcpy(ether->h_dest, dest, ETH_ALEN);
	memcpy(ether->h_source, src, ETH_ALEN);
	ether->h_proto = htons(proto);

	return 0;
}

static int __dev_queue_wrapper_xmit(struct net_bridge_port *p, struct sk_buff *skb)
{
	int ret = 0;

        if (p->is_p2p) {

            if (!p->is_unencap) {

		/* Encapsulate the original ethernet frame inside our own frame 
		 * addressed directly to the point-to-point link
		 *
		 * destination is point-to-point endpoint
		 * source is MAC address of outgoing interface
		 * protocol number for tunneled frames (direct is
		 * unlearned)
		 */
		ret = __br_encap_skb(skb, 
				     p->p2p_dest_addr,
				     skb->dev->dev_addr, 
				     (__br_skb_marked_direct(skb) ? 
				      BR_TUNNEL2_PROTOCOLNUM :
				      BR_TUNNEL_PROTOCOLNUM));

		if (ret) {
			return ret;
		}

            } else if (p->is_unicast) {

                /* destination is point-to-point endpoint */
                memcpy(skb->mac.ethernet->h_dest, p->p2p_dest_addr, ETH_ALEN);
            }

        }

	/* Encapsulate multicast management traffic going onto the network */
	if (!p->is_p2p && br_mcast_is_management_header(skb->mac.ethernet)) {

		ret = __br_encap_skb(skb, 
				     broadcast_addr, 
				     skb->mac.ethernet->h_source, 
				     BR_MCAST_GL_PROTOCOLNUM);

		if (ret) {
			return ret;
		}
	}

        dev_queue_xmit(skb);

        return 0;
}

static int __dev_queue_push_xmit(struct net_bridge_port *to, 
                                 struct sk_buff *skb)
{
        skb_push(skb, ETH_HLEN);

        __dev_queue_wrapper_xmit(to, skb);

        return 0 ;
}

static int __br_forward_finish(struct net_bridge_port *to, 
                               struct sk_buff *skb)
{
        __dev_queue_push_xmit(to, skb);

	return 0;
}

static void __br_deliver(struct net_bridge_port *to, struct sk_buff *skb)
{
	skb->dev = to->dev;
        __br_forward_finish(to, skb);
}

static void __br_forward(struct net_bridge_port *to, struct sk_buff *skb)
{
	skb->dev = to->dev;
        __br_forward_finish(to, skb);
}

/* called under bridge lock */
void br_deliver(struct net_bridge_port *from, 
                struct net_bridge_port *to, struct sk_buff *skb)
{
	if (should_deliver(from, to)) {
		__br_skb_mark_direct(skb, 0);
		__br_deliver(to, skb);
		return;
	}

	kfree_skb(skb);
}

/* called with rcu_read_lock */
void br_deliver_direct(const struct net_bridge_port *from,
                       const struct net_bridge_port *to,
                       struct sk_buff *skb)
{
	/*
         * REVIEW: This is different since we don't care about STP state.
         *
         *         Also note that this traffic will be sent down a tunnel that
         *         tells the device on the other end not to learn.  Sending all
         *         traffic to a ZP down this path is a *bad* idea, since it
         *         will never learn the return path and end up constantly
         *         flooding the replies.
         */
	if (from != to) {
		__br_skb_mark_direct(skb, 1);
                __br_deliver(to, skb);
                return;
        }

	kfree_skb(skb);
}

void br_forward_direct(const struct net_bridge_port *from,
		       const struct net_bridge_port *to,
		       struct sk_buff *skb)
{
	/*
         * REVIEW: This is different since we don't care about STP state.
         */
	if (from != to) {
		__br_skb_mark_direct(skb, 1);
 		__br_forward(to, skb);
                return;
        }
 
 	kfree_skb(skb);
}

/* SONOS: Only called with bpdus.  It doesn't really matter how we tunnel them,
 *        but sending it the same way we used to send it before direct routing
 *        makes me feel better.
 */
void br_deliver_bpdu(const struct net_bridge_port *p,
                     struct sk_buff *skb)
{
	__br_skb_mark_direct(skb, 0);
	__dev_queue_wrapper_xmit(p, skb);
}


/* called under bridge lock */
void br_forward(struct net_bridge_port *from, 
                struct net_bridge_port *to, struct sk_buff *skb)
{
	if (should_deliver(from, to)) {
		__br_skb_mark_direct(skb, 0);
		__br_forward(to, skb);
		return;
	}

	kfree_skb(skb);
}

/* called under bridge lock */
static void br_true_flood(struct net_bridge *br, struct net_bridge_port *from, 
        struct sk_buff *skb, int clone, int p2p_flood_deliver,
	void (*__packet_hook)(struct net_bridge_port *p, struct sk_buff *skb))
{
	struct net_bridge_port *p;
	struct net_bridge_port *prev;

	if (clone) {
		struct sk_buff *skb2;

		if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
			br->statistics.tx_dropped++;
			return;
		}

		skb = skb2;
	}

	prev = NULL;

        /* handle deliveries to ports running STP */
	p = br->port_list;
	while (p != NULL) {
            if (should_deliver(from, p)) {
                if (prev != NULL) {
                    struct sk_buff *skb2;

                    if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
                        br->statistics.tx_dropped++;
                        kfree_skb(skb);
                        return;
                    }
                    __packet_hook(prev, skb2);
                }

                prev = p;
            }

            p = p->next;
	}

        /* handle deliveries to p2p leaf stations */
	p = br->leaf_list;

        if (p && p2p_flood_deliver) {
            while (p != NULL) {
                if (should_deliver(from, p)) {
                    if (prev != NULL) {
                        struct sk_buff *skb2;

                        if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
                            br->statistics.tx_dropped++;
                            kfree_skb(skb);
                            return;
                        }

                        __packet_hook(prev, skb2);
                    }

                    prev = p;
                }

                p = p->next;
            }
        }

	if (prev != NULL) {
		__packet_hook(prev, skb);
		return;
	}

	kfree_skb(skb);
}

/* called under bridge lock */
static void br_mcast(struct net_bridge *br, struct net_bridge_port *from, 
        struct sk_buff *skb, int clone,
        void (*__packet_hook)(struct net_bridge_port *p, struct sk_buff *skb),
        struct net_bridge_mcast_entry *me)
{
	struct net_bridge_port *p;
	struct net_bridge_port *prev;

	if (clone) {
		struct sk_buff *skb2;

		if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
			br->statistics.tx_dropped++;
			return;
		}

		skb = skb2;
	}

	prev = NULL;

        struct net_bridge_mcast_rx_port* mrxp;
        for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {
            p = mrxp->dst;

            /* don't forward to the local device */
            if (!p)
                continue;

            if (p->is_p2p) {
                if (should_deliver(from, p)) {
                    if (prev != NULL) {
                        struct sk_buff *skb2;

                        if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
                            br->statistics.tx_dropped++;
                            kfree_skb(skb);
                            return;
                        }

                        __packet_hook(prev, skb2);
                    }

                    prev = p;
                }
            } else {
                /* when multicasting onto a normal network segment, 'expand'
                   the multicast into n unicasts to each of the MAC addresses
                   that have expressed interest in receiving the data.  This
                   avoids flooding the customer's home network with multicast
                   data. */
                if (should_deliver(from, p)) {
                    struct net_bridge_mcast_rx_mac* mrxm;
                    for (mrxm = mrxp->rx_mac_list; 
                         mrxm != NULL; 
                         mrxm = mrxm->next) {
                        struct sk_buff *skb2;

                        if ((skb2 = skb_copy(skb, GFP_ATOMIC))) {
                            memcpy(skb2->mac.ethernet->h_dest,
                                   mrxm->addr,
                                   ETH_ALEN);

                            __packet_hook(p, skb2);
                        } else
                            br->statistics.tx_dropped++;
                    }
                }
            }
	}

	if (prev != NULL) {
		__packet_hook(prev, skb);
		return;
	}

	kfree_skb(skb);        
}

unsigned char broadcast_addr[]  = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
unsigned char upnp_addr[]       = {0x01, 0x00, 0x5e, 0x7f, 0xff, 0xfa};
unsigned char igmp_ah_addr[]    = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x01};
unsigned char igmp_ar_addr[]    = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x02};
unsigned char igmp_amr_addr[]   = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x16};
unsigned char mdns_addr[]       = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb};
unsigned char rincon_gmp_addr[] = {0x01, 0x0e, 0x58, 0xdd, 0xdd, 0xdd};

static void br_flood(struct net_bridge *br, struct net_bridge_port *from, 
        struct sk_buff *skb, int clone,
	void (*__packet_hook)(struct net_bridge_port *p, struct sk_buff *skb))
{
	/* Make sure we do not direct route anything by accident */
	__br_skb_mark_direct(skb, 0);

    /* The following is a slight hack, but is very convenient for our
       purposes.  The p2p leaf concept is used by the Rincon handheld
       remote controls, which only need to receive unicasts destined
       for themselves, MAC broadcasts, and MAC multicasts to the 
       UPnP-reserved multicast group.  We block all other 
       multicasts. */
    unsigned char *dest;
    dest = skb->mac.ethernet->h_dest;
    int p2p_flood_deliver = 0;
    if (dest[0] & 1) {
        if (0 == memcmp(dest, broadcast_addr, ETH_ALEN) ||
            0 == memcmp(dest, upnp_addr, ETH_ALEN) ||
            0 == memcmp(dest, mdns_addr, ETH_ALEN) ||
            0 == memcmp(dest, igmp_ah_addr, ETH_ALEN) ||
            0 == memcmp(dest, igmp_ar_addr, ETH_ALEN) ||
            0 == memcmp(dest, igmp_amr_addr, ETH_ALEN))
            p2p_flood_deliver = 1;
    } else
        p2p_flood_deliver = 1;    

    if (p2p_flood_deliver)
        br_true_flood(br, from, skb, clone, 1, __packet_hook);
    else {
        /* see if we can limit the delivery of this packet to just the subset
           of the network that is a member of the MAC multicast group */
        struct net_bridge_mcast_entry *me = br_mcast_get(br, dest);
        if (me) {
            br_mcast(br, from, skb, clone, __packet_hook, me);
            br_mcast_put(me);
        } else {
            /* if a packet originates locally, and it is destined for a 
               MAC multicast group that is 'unknown' to us, then drop 
               the packet. */
            if (__packet_hook == __br_deliver) {
                if (0 == memcmp(rincon_gmp_addr, dest, 6))
                    br_true_flood(br, from, skb, clone, 0, __packet_hook);
                else
                    kfree_skb(skb);
            } else
                br_true_flood(br, from, skb, clone, 0, __packet_hook);
        }
    }
}

/* called under bridge lock */
void br_flood_deliver(struct net_bridge *br, struct net_bridge_port *from, 
                      struct sk_buff *skb, int clone)
{
	br_flood(br, from, skb, clone, __br_deliver);
}

/* called under bridge lock */
void br_flood_forward(struct net_bridge *br, struct net_bridge_port *from, 
                      struct sk_buff *skb, int clone)
{
	br_flood(br, from, skb, clone, __br_forward);
}
