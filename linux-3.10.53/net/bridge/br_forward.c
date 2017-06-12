/*
 *	Forwarding decision
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/netpoll.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/netfilter_bridge.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/udp.h>

#include <net/arp.h>
#include <net/checksum.h>

#include "br_private.h"

// #define DEBUG_BR_FLOOD 1

/* REVIEW:
 *
 * __br_skb_*: This uses skb->cb to stash data in.  This conflicts with
 * netfilter, but we (Sonos) do not use netfilter (and we've already broken it
 * in other ways).  Looks like cb[4] is safe to use even w/ netfilter, but
 * whatever...
 */
static inline void __br_skb_mark_direct(struct sk_buff *skb, int direct)
{
	BR_SKB_CB(skb)->direct = direct;
}

static inline int __br_skb_marked_direct(struct sk_buff *skb)
{
	return BR_SKB_CB(skb)->direct;
}

#ifdef CONFIG_SONOS_BRIDGE_PROXY
static inline int
should_proxy_up(const struct net_bridge_port *p_arriving,
                const struct net_bridge_port *p_outgoing) {
        if (!p_outgoing->br->proxy_mode || !p_arriving)
		return 0;
	return (p_outgoing->is_uplink || (p_outgoing->direct_enabled && !p_outgoing->is_satellite)) && p_arriving->is_satellite;
}
#endif

static inline int should_proxy(const struct net_bridge_port *p_arriving,
                               const struct net_bridge_port *p_outgoing) {
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	/* If we're proxying, assume here that we'll deliver it if it's
	 * sourced or destined from the uplink port and the other port
	 * is a satellite, or if both ports are satellites.
	 *
	 * This code has been written to explicitly allow early return;
	 * the optimizer didn't appear to be generating code that
	 * allowed this when we were assigning each logical condition
	 * to a variable and then ORing them.
	 *
	 * We can't assume that p_arriving is non-zero because this
	 * function is called when a packet is generated on this machine
	 * and doesn't have a "port". However, in such a case, we're
	 * not doing any proxying, either.
	 */

	unsigned char proxy_up, proxy_down, both_sat;

	if (!p_outgoing->br->proxy_mode || !p_arriving)
		return 0;

	proxy_up = should_proxy_up(p_arriving, p_outgoing);
	if (proxy_up)
		return 1;

	proxy_down = p_arriving->is_uplink && p_outgoing->is_satellite;
	if (proxy_down)
		return 1;

	both_sat = p_arriving->is_satellite && p_outgoing->is_satellite;
	if (both_sat)
		return 1;

	/* if (proxy_up || proxy_down || both_sat) return 1; */

#endif
        return 0;
}
static inline int should_deliver(const struct net_bridge_port *p_arriving,
                                 const struct net_bridge_port *p_outgoing)
{
	/* Don't forward if the port is the same */
	if (p_arriving == p_outgoing) return 0;

	if (should_proxy(p_arriving, p_outgoing)) return 1;

	/* if the specified port is disabled, don't forward it */
	if (p_outgoing->state != BR_STATE_FORWARDING
	    || p_outgoing->remote_state == BR_STATE_BLOCKING)
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
	skb->mac_header -= ETH_HLEN;
	ether = eth_hdr(skb);

	/* Set up the addresses and proto */
	memcpy(ether->h_dest, dest, ETH_ALEN);
	memcpy(ether->h_source, src, ETH_ALEN);
	ether->h_proto = htons(proto);

	return skb;
}

static void __dev_queue_wrapper_xmit(const struct net_bridge_port *p, struct sk_buff *skb)
{
	/* 3.10 kernel will remove vlan header and store it in vlan_tci of skb.
	 * We need to recover vlan header here to avoid kernel inserting vlan header
	 * before p2p tunneling header.
	 */
	if (vlan_tx_tag_present(skb)) {
		skb = __vlan_put_tag(skb, skb->vlan_proto, vlan_tx_tag_get(skb));
		if (!skb) {
			printk("Returned null skb during vlan header recovering before sending on %s\n",
				p->dev->name);
			return;
		}
		skb->vlan_tci = 0;
	}
	/* Encapsulate tunneled p2p traffic (with some exceptions) */
	if (p->is_p2p) {
		if (!p->is_unencap)  {

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

		} else if (p->is_unicast) {
			/* destination is point-to-point endpoint */
			memcpy(eth_hdr(skb)->h_dest, p->p2p_dest_addr, ETH_ALEN);
		}

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

#ifdef CONFIG_SONOS_BRIDGE_PROXY
static void _set_sat_ip(struct net_bridge_port *sat_port, u32 new_ip)
{
	int run_check = 0;
	u32 current_sat_ip = sat_port->sat_ip;
	u32 current_br_ip = sat_port->br->current_ipv4_addr;

	/* Check if I even want to set it */
	if (current_sat_ip == new_ip || !new_ip) {
		return;
	}

	/* Check if we may be transitioning in or out of conflict. */
	unsigned char is_changing_to_current = (new_ip == current_br_ip);
	unsigned char is_changing_from_current = (current_sat_ip == current_br_ip);

	if (is_changing_to_current || is_changing_from_current) {
		run_check = 1;
	}

	sat_port->sat_ip = new_ip;

	if (run_check) {
		br_dupip_check(sat_port->br);
	}
}

/*
 * Check whether a frame is a DHCP datagram.
 */
unsigned char br_is_dhcp(struct sk_buff *skb,
			  struct iphdr **oiph,
			  struct udphdr **oudph,
			  unsigned char **odhcph)
{

	struct iphdr *iph;
	struct udphdr *udph;

	int total_offset = 0;

	skb->network_header = (unsigned char *)(eth_hdr(skb) + 1);

	total_offset += sizeof(struct iphdr);
	if (!pskb_may_pull(skb, total_offset))
		return 0;

	iph = ip_hdr(skb);
	*oiph = iph;

	if (iph->protocol != IPPROTO_UDP)
		return 0;

	skb->transport_header = (unsigned char *)((__u32 *)iph + iph->ihl);

	/* subtract size of iphdr because iph->ihl includes it */
	total_offset += (iph->ihl * 4 - sizeof(struct iphdr) + sizeof(struct udphdr));
	if (!pskb_may_pull(skb, total_offset))
		return 0;

	udph = udp_hdr(skb);
	*oudph = udph;

	if (!(
		(udph->source == htons(67) && udph->dest == htons(68)) ||
		(udph->source == htons(68) && udph->dest == htons(67))
	     ))
		return 0;

	total_offset += (ntohs(udph->len) - sizeof(struct udphdr));

	if (!pskb_may_pull(skb, total_offset))
		return 0;

	*odhcph = (unsigned char *)(udph + 1);

        return 1;
}

void br_log_dhcp(struct udphdr *udph, unsigned char *dhcph)
{
	const unsigned int tid_offset = 4;
	const unsigned int cip_offset = 12;
	const unsigned int chaddr_offset = 28;
	const unsigned int first_option_offset = 240;
	const unsigned char message_type_code = 0x35;

	const char *msg_names[] = { "???", "DIS", "OFR", "REQ", "DEC", "ACK", "NAK", };

	unsigned int idx = 0;
	const int dhcp_len = ntohs(udph->len) - sizeof(struct udphdr);
	if (dhcp_len <= first_option_offset + 2) {
		return;
	}

	if (dhcph[first_option_offset] == message_type_code) {
		if (dhcph[first_option_offset + 2] < sizeof(msg_names)/sizeof(msg_names[0])) {
			idx = dhcph[first_option_offset + 2];
		}
	}

	printk("br proxy dhcp: %s 0x%x %pM %pI4\n",
		msg_names[idx],
		ntohl(*(__u32 *)&dhcph[tid_offset]),
		&dhcph[chaddr_offset],
		(__u32 *)&dhcph[cip_offset]);
}

/* Check for a DHCP frame. Set the broadcast bit and recalculate the
 * checksum if we find it.
 */
static void _br_proxy_mangle_for_ip(struct sk_buff *skb)
{
	struct iphdr *iph = NULL;
	struct udphdr *udph = NULL;
	unsigned char *dhcph = NULL;

	int dhcp_flag_offset = 10;
	__u16 *flags;
	__u16 dhcp_bcast_flag = htons(0x8000);

	__wsum csum;

	unsigned char is_dhcp = br_is_dhcp(skb, &iph, &udph, &dhcph);

	if (iph) {
		_set_sat_ip(BR_SKB_CB(skb)->source_port, iph->saddr);
	}

	/* If it's not a DHCP frame destined for the server, do nothing.
	 */
	if (!is_dhcp || !iph || !udph || !dhcph || (udph->source != htons(68) ||
						    udph->dest != htons(67))) {
		return;
	}

	br_log_dhcp(udph, dhcph);

	/* found it, set the bit */
	flags = (__u16 *)&dhcph[dhcp_flag_offset];
	*flags |= dhcp_bcast_flag;

	/* recalculate the UDP checksum */
	udph->check = 0;
	csum = csum_partial(skb_transport_header(skb), ntohs(udph->len), 0);
	udph->check = csum_tcpudp_magic(iph->saddr, iph->daddr, ntohs(udph->len),
					IPPROTO_UDP, csum);

}

static void _br_proxy_mangle_for_arp(struct net_bridge *br, struct sk_buff *skb)
{
	unsigned char *arp_ptr = (unsigned char *)(eth_hdr(skb) + 1) + sizeof(struct arphdr);
	unsigned char *arp_sender_mac;
	__u32* arp_sender_ip;

	if (!pskb_may_pull(skb, sizeof(struct arphdr) + ETH_ALEN + sizeof(__u32)))
		return;

	skb->network_header = (unsigned char *)(eth_hdr(skb) + 1);
	arp_ptr = (unsigned char *)(arp_hdr(skb) + 1);

	arp_sender_mac = arp_ptr;

	arp_ptr += ETH_ALEN;

	arp_sender_ip = (__u32 *)arp_ptr;

	/* Record the sender IP. We'll use it to forward frames
	 * from the uplink to this satellite. */
	_set_sat_ip(BR_SKB_CB(skb)->source_port, *arp_sender_ip);

	/* mess with arp header */
	memcpy(arp_sender_mac, br->static_mac, ETH_ALEN);
}
#endif
static void br_mangle_if_proxying_up(const struct net_bridge_port *to,
				     struct sk_buff *skb)
{
#ifdef CONFIG_SONOS_BRIDGE_PROXY
        if (BR_SKB_CB(skb)->should_proxy_up &&
	    to->br->use_static_mac &&
	    should_proxy_up(BR_SKB_CB(skb)->source_port, to)) {
		/* We're a proxy bridge forwarding a frame from
		 * a satellite to the uplink or a direct-route port, so
		 * let's do our naughty proxy stuff.
		 */
		__be16 proto = eth_hdr(skb)->h_proto;

		if (ntohs(proto) == ETH_P_ARP) {
			_br_proxy_mangle_for_arp(to->br, skb);
		} else if (ntohs(proto) == ETH_P_IP) {
			_br_proxy_mangle_for_ip(skb);
		}

		memcpy(eth_hdr(skb)->h_source, to->br->static_mac, ETH_ALEN);
        }
#endif
}

static int _br_forward_finish(const struct net_bridge_port *to,
			      struct sk_buff *skb)
{
	br_mangle_if_proxying_up(to, skb);
	__dev_queue_push_xmit(to, skb);
	return 0;
}

void __br_deliver(const struct net_bridge_port *to, struct sk_buff *skb)
{

	skb->dev = to->dev;

	_br_forward_finish(to,skb);
}

void __br_forward(const struct net_bridge_port *to, struct sk_buff *skb)
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
        struct net_bridge_mcast_rx_port* mrxp;

	if (clone) {
		struct sk_buff *skb2;

		if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
			br->statistics.tx_dropped++;
			return;
		}

		skb = skb2;
	}

	prev = NULL;

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
	struct net_bridge_mcast_rx_mac* mrxm;
        struct net_bridge_port* port;
	unsigned int i;

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
unsigned char mdns_addr[]       = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb};
unsigned char igmp_ah_addr[]    = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x01};
unsigned char igmp_ar_addr[]    = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x02};
unsigned char igmp_amr_addr[]   = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x16};
unsigned char rincon_gmp_addr[] = {0x01, 0x0e, 0x58, 0xdd, 0xdd, 0xdd};

/* called under bridge lock */
static void br_flood(struct net_bridge *br,
		     struct net_bridge_port *from,
		     struct sk_buff *skb, int clone,
		     void (*__packet_hook)(const struct net_bridge_port *p,
					   struct sk_buff *skb))
{
	const unsigned char *dest  = (unsigned char *)(eth_hdr(skb)->h_dest);
	int p2p_flood_deliver = 0;

	/* Make sure we do not direct route anything by accident */
	__br_skb_mark_direct(skb, 0);

	/* The following is a slight hack, but is very convenient for our
	   purposes.  The p2p leaf concept is used by the Rincon handheld
	   remote controls, which only need to receive unicasts destined
	   for themselves, MAC broadcasts, and MAC multicasts to the
	   UPnP-reserved multicast group.  We block all other
	   multicasts. */
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
