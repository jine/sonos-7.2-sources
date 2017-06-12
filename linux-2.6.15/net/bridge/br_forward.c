/*
 *	Forwarding decision
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br_forward.c#1 $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
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

static inline int should_deliver(const struct net_bridge_port *p_arriving,
                                 const struct net_bridge_port *p_outgoing)    
{
	/* if the specified port is disabled, or if the packet arrived on the
	   specified port in the first place, don't forward it */
	if (p_outgoing->state != BR_STATE_FORWARDING
	    || p_outgoing->remote_state == BR_STATE_BLOCKING
	    || p_arriving == p_outgoing)
		return 0;
	
	return 1;
}

static struct sk_buff * __br_encap_skb(struct sk_buff *skb,
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
		return NULL;
	}

	/* Find current ethernet header */
	ether = eth_hdr(skb);

	skb_push(skb, ETH_HLEN);
	skb->mac.raw -= ETH_HLEN;                
	ether = eth_hdr(skb);

	/* Set up the addresses and proto */
	memcpy(ether->h_dest, dest, ETH_ALEN);
	memcpy(ether->h_source, src, ETH_ALEN);
	ether->h_proto = htons(proto);

	return skb;
}

static void __dev_queue_wrapper_xmit(const struct net_bridge_port *p, struct sk_buff *skb)
{
	/* Encapsulate tunneled p2p traffic (with some exceptions) */
	if (!p->is_unencap) {

		/* Encapsulate the original ethernet frame inside our own frame 
		 * addressed directly to the point-to-point link
		 *
		 * destination is point-to-point endpoint
		 * source is MAC address of outgoing interface
		 * protocol number for tunneled frames (direct is
		 * unlearned)
		 */
		skb = __br_encap_skb(skb, 
				     p->p2p_dest_addr, 
				     skb->dev->dev_addr,
				     __br_skb_marked_direct(skb) ? 
				     BR_TUNNEL2_PROTOCOLNUM :
				     BR_TUNNEL_PROTOCOLNUM);
		
	}  else if (p->is_unicast) {

		/* destination is point-to-point endpoint */
		memcpy(eth_hdr(skb)->h_dest, p->p2p_dest_addr, ETH_ALEN);
	}

	/* Encapsulate multicast management traffic going onto the network */
	if ((!p->is_p2p || p->is_uplink) && skb &&
	    br_mcast_is_management_header(eth_hdr(skb))) {

		skb = __br_encap_skb(skb, 
				     broadcast_addr, 
				     eth_hdr(skb)->h_source, 
				     BR_MCAST_GL_PROTOCOLNUM);
	}

	/* If we still have something to transmit, go for it */
	if (skb) {
		dev_queue_xmit(skb);
	}
}

static void __dev_queue_push_xmit(const struct net_bridge_port *to, 
				  struct sk_buff *skb)
{
	skb_push(skb, ETH_HLEN);
        __dev_queue_wrapper_xmit(to, skb);
}


static int _br_forward_finish(const struct net_bridge_port *to,
			      struct sk_buff *skb)
{
	__dev_queue_push_xmit(to, skb);
	return 0;
}

static void __br_deliver(const struct net_bridge_port *to, struct sk_buff *skb)
{
	skb->dev = to->dev;
	_br_forward_finish(to,skb);
}

static void __br_forward(const struct net_bridge_port *to, struct sk_buff *skb)
{
	skb->dev = to->dev;
	skb->ip_summed = CHECKSUM_NONE;
        _br_forward_finish(to, skb);
}

/* called with rcu_read_lock */
void br_deliver(const struct net_bridge_port *from,
		const struct net_bridge_port *to,
		struct sk_buff *skb)
{
	if (should_deliver(from, to)) {
		__br_skb_mark_direct(skb, 0);
		__br_deliver(to, skb);
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

/* called with rcu_read_lock */
void br_forward(const struct net_bridge_port *from,
		const struct net_bridge_port *to,
		struct sk_buff *skb)
{
	if (should_deliver(from, to)) {
		__br_skb_mark_direct(skb, 0);
		__br_forward(to, skb);
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

/* called with rcu_read_lock */
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

/* called under bridge lock */
static void br_true_flood(struct net_bridge *br, struct net_bridge_port *from, 
			  struct sk_buff *skb, int clone, int p2p_flood_deliver,
			  void (*__packet_hook)(const struct net_bridge_port *p,
						struct sk_buff *skb))
{
	struct net_bridge_port *p;
	struct net_bridge_port *prev;

	/* Send in the clones */
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
	list_for_each_entry_rcu(p, &br->port_list, list) {
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
	}

        /* handle deliveries to p2p leaf stations */
	if (p2p_flood_deliver) {
		list_for_each_entry_rcu(p, &br->leaf_list, list) {
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
        void (*__packet_hook)(const struct net_bridge_port *p, struct sk_buff *skb),
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
						memcpy((char *)(eth_hdr(skb2)->h_dest),
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

static void
br_mcast_direct(struct net_bridge *br, struct net_bridge_port *from, 
		struct sk_buff *skb, int clone,
		void (*__packet_hook)(const struct net_bridge_port *p,
				      struct sk_buff *skb),
		struct net_bridge_mcast_entry *me)
{
#define BR_MAX_DIRECT_DESTS 32

	struct {
		struct net_bridge_mcast_rx_mac* mrxm;
		struct net_bridge_port*         port;
	} destinations[32];

	int num = 0, bad = 0;
		
	/* REVIEW: This is a prototype.  Spinning through all of this for every
	 *         packet is not the best idea, so we should figure out how to
	 *         update the multicast entries themselves so that they always
	 *         contain the correct information.
	 *
	 *         OK, so it isn't *that* bad.  If we are all wired or capable
	 *         of direct routing it isn't any less efficient than the old
	 *         code.  If we are not, we spin through a couple of lists a
	 *         second time.  We're not doing hash table lookups, so I think
	 *         we'll be OK.
	 */
        struct net_bridge_mcast_rx_port* mrxp;
        for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {

		struct net_bridge_port* p = mrxp->dst;
		
		/* Skip local ports.  We know this originated locally, so we
		 * don't care about should_deliver() at all.  If we can see all
		 * of the destinations, we're going for it.  This is a little
		 * sketchy for ethernet ports, but I we're all good since we're
		 * converting everything to unicast and should not be able to
		 * generate a loop.
		 *
		 * Also skip ports that are not at least learning.
		 */
		if (!p) {
			continue;
		}

		/* Figure out how to send each packet, aborting once we find out
		 * we can't direct route.
		 */
 		struct net_bridge_mcast_rx_mac* mrxm;
                struct net_bridge_port* port = p;

		for (mrxm = mrxp->rx_mac_list;
		     mrxm != NULL;
		     mrxm = mrxm->next) {

			port = p;

			/* If the STP port is wireless, see if there is a
			 * better way to get to the DA (i.e. a direct route).
			 */
			if (p->is_p2p) {
				
				/* Do we actually know the neighbor and is the
				 * signal strength good enough?
				 */
 				if (!mrxm->direct_dst ||
				    !mrxm->direct_dst->direct_enabled) {
					goto abort;
				}

				/* Does it want direct routed multicast
				 * traffic?  We can tolerate one guy not
				 * wanting this, but not all.
				 */
				if (!(mrxm->direct_dst->direct_enabled & 2)) {
					if (++bad == 2) {
						goto abort;
					}
				}
                                
				port = mrxm->direct_dst;
				
				/* If port is not forwarding or blocked, drop */
				if (port->state <= BR_STATE_LEARNING)
					continue;

			} else {

				/* If the port is not enabled, drop */
				if (port->state == BR_STATE_DISABLED) {
					continue;
				}
			}

			/* Create another entry in the array */
			destinations[num].mrxm = mrxm;
			destinations[num].port = port;

			if (++num == BR_MAX_DIRECT_DESTS)
				goto abort;
		}
	}

#ifdef BR_DEBUG_DIRECT
	{
		static int i = 0;
		
		if (++i == 512) {
			printk("mc: rock! (%d, %d)\n", num, bad);
			i = 0;
		}
	}
#endif

	/*
	 * Unicast to all destinations
	 */

	/* Clone if required.  This *must* be a copy, not a clone. */
	if (clone) {
		struct sk_buff *skb2;

		if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
			br->statistics.tx_dropped++;
			return;
		}

		skb = skb2;
	}

	/* Mark it direct so the wireless dests don't learn the wrong path */
	__br_skb_mark_direct(skb, 1);

	/* Unicast to all of the destinations
	 *
	 * REVIEW: Should port the "prev" trickery from below to save a copy if
	 *         possible.
	 */
	unsigned int i;
	for (i = 0; i < num; i++) {
		struct net_bridge_port* port;
		struct net_bridge_mcast_rx_mac* mrxm;
		struct sk_buff *skb2;		

		port = destinations[i].port;
		mrxm = destinations[i].mrxm;
		
		if ((skb2 = skb_copy(skb, GFP_ATOMIC))) {
			
			memcpy((char *)(eth_hdr(skb2)->h_dest),
			       mrxm->addr,
			       ETH_ALEN);

			//printk("mc: %d: %p %p %p\n",
			//i, port, mrxm, skb2);
			
			__packet_hook(port, skb2);
			
		} else {
			br->statistics.tx_dropped++;
			break;
		}
	}

	/*
	 * Free the original since we made copies of everything.  We can save a
	 * copy here (and the free) with some work.
	 */
	kfree_skb(skb);	
	return;
        
abort:

#ifdef BR_DEBUG_DIRECT
	{
		static int i = 0;

		if (++i == 512) {
			printk("mc: abort!\n");
			i = 0;
		}
	}
#endif

	/* Can't do it directly, go old school */
	br_mcast(br, from, skb, clone, __packet_hook, me);
}

unsigned char broadcast_addr[]  = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
unsigned char upnp_addr[]       = {0x01, 0x00, 0x5e, 0x7f, 0xff, 0xfa};
unsigned char igmp_ah_addr[]    = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x01};
unsigned char igmp_ar_addr[]    = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x02};
unsigned char igmp_amr_addr[]   = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x16};
unsigned char mdns_addr[]       = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb};
unsigned char rincon_gmp_addr[] = {0x01, 0x0e, 0x58, 0xdd, 0xdd, 0xdd};

/* called under bridge lock */
static void br_flood(struct net_bridge *br,
		     struct net_bridge_port *from,
		     struct sk_buff *skb, int clone,
		     void (*__packet_hook)(const struct net_bridge_port *p, 
					   struct sk_buff *skb))
{
	unsigned char *dest  = (unsigned char *)(eth_hdr(skb)->h_dest);

        /* Make sure we do not direct route anything by accident */
	__br_skb_mark_direct(skb, 0);

	/* The following is a slight hack, but is very convenient for our
	   purposes.  The p2p leaf concept is used by the Rincon handheld
	   remote controls, which only need to receive unicasts destined
	   for themselves, MAC broadcasts, and MAC multicasts to the 
	   UPnP-reserved multicast group.  We block all other 
	   multicasts. */
	int p2p_flood_deliver = 0;
	if (dest[0] & 1) {
		if (br_mcast_dest_is_allowed(dest)) {
			p2p_flood_deliver = 1;
		}
	} else {
		p2p_flood_deliver = 1;
	}

	if (p2p_flood_deliver)
		br_true_flood(br, from, skb, clone, 1, __packet_hook);
	else {
		/* see if we can limit the delivery of this packet to just the subset
		   of the network that is a member of the MAC multicast group */
		struct net_bridge_mcast_entry *me = br_mcast_get(br, dest);
		if (me) {
			/* See if we can direct route the wireless bits */
			if (__packet_hook == __br_deliver)
				br_mcast_direct(br, from, skb, clone, __packet_hook, me);
			else
				br_mcast(br, from, skb, clone, __packet_hook, me);
			br_mcast_put(me);
		} else {
			/* if a packet originates locally, and it is destined for a 
			   MAC multicast group that is 'unknown' to us, then drop 
			   the packet. */
			if (__packet_hook == __br_deliver) {
				if (br_mcast_dest_is_allowed_from_local(dest))
					br_true_flood(br, from, skb, clone, 0, __packet_hook);
				else
					kfree_skb(skb);
			} else
				br_true_flood(br, from, skb, clone, 0, __packet_hook);
		}
	}
}

/* called with rcu_read_lock */
void br_flood_deliver(struct net_bridge *br,
                      struct net_bridge_port *from,
                      struct sk_buff *skb, int clone)
{
	br_flood(br, from, skb, clone, __br_deliver);
}

/* called with rcu_read_lock */
void br_flood_forward(struct net_bridge *br,
                      struct net_bridge_port *from,
                      struct sk_buff *skb, int clone)
{
	br_flood(br, from, skb, clone, __br_forward);
}
