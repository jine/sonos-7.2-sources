/*
 *	Uplink mode extensions
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <net/arp.h>

#include <asm/uaccess.h>
#include "br_private.h"

/* #define BR_DEBUG_UPLINK 1 */

extern unsigned char broadcast_addr[6];

static struct net_bridge_port *_get_uplink(struct net_bridge *br)
{
	struct list_head *ports = &br->port_list;
	struct net_bridge_port *p_tmp;

	list_for_each_entry_rcu(p_tmp, ports, list) {			
 		if (p_tmp->is_uplink)
			return p_tmp;
	}
	return NULL;
}

static struct net_bridge_port *_get_port_for_da(struct net_bridge *br,
						const unsigned char *da)
{
	struct list_head *ports = &br->port_list;
	struct net_bridge_port *p_tmp, *p_dst = 0, *p_uplink = 0;

	list_for_each_entry_rcu(p_tmp, ports, list) {	
		if ((p_tmp->direct_enabled) &&
		    (0 == memcmp(da, p_tmp->direct_addr, 6))) {
			p_dst = p_tmp;
			break;
		}
		if (p_tmp->is_uplink) {
			p_uplink = p_tmp;
		}
	}

	if (!p_dst)
		p_dst = p_uplink;

	return p_dst;
}

static inline void _deliver(struct net_bridge_port *port, struct sk_buff *skb)
{

	if (!skb->priority)
		skb->priority = port->priority;

	if (port->direct_enabled) {
		br_deliver_direct(0, port, skb);
	} else {
		__br_deliver(port, skb);
	}
}

#ifdef CONFIG_SONOS_BRIDGE_PROXY
/* Looking for a port that matches our da that is either a direct-route
 * destination, or another satellite. Return the uplink if nothing else
 * matches.
 */
static struct
net_bridge_port *_get_port_for_da_proxied(struct net_bridge *br,
					  const unsigned char *da)
{
	struct list_head *ports = &br->port_list;
	struct net_bridge_port *p_tmp, *p_dst = NULL, *p_uplink = NULL;

	list_for_each_entry_rcu(p_tmp, ports, list) {
		if ((p_tmp->direct_enabled) &&
		    (0 == memcmp(da, p_tmp->direct_addr, 6))) {
			p_dst = p_tmp;
			break;
		} else if ((0 == memcmp(da, p_tmp->p2p_dest_addr, 6))
			    && p_tmp->is_satellite) {
			p_dst = p_tmp;
			break;
		}
		if (p_tmp->is_uplink) {
			p_uplink = p_tmp;
		}
	}

	if (!p_dst)
		p_dst = p_uplink;

	return p_dst;
}

/* A frame has been delivered from a proxied host (currently only
 * satellites). For unicast, we'll optionally pass it up, send it to
 * another satellite, deliver it to a direct-routed destination, or send
 * it to the uplink. For multicast and broadcast, we will only handle
 * the special Sonos multicast groups as well as broadcast, and we will
 * forward those to all other satellites, the uplink and pass up to our
 * device.
 */
void br_uplink_proxy(struct net_bridge *br,
		     struct net_bridge_port *p,
		     struct sk_buff *skb,
		     const char *dest)
{
	struct net_bridge_port *p_dst;

#ifdef BR_DEBUG_UPLINK
	printk("brup_proxy: tx: p=%04x d=%02x:%02x:%02x:%02x:%02x:%02x\n",
	       eth_hdr(skb)->h_proto,
	       dest[0], dest[1], dest[2],
	       dest[3], dest[4], dest[5]);
        if (eth_hdr(skb)->h_proto == htons(ETH_P_IP)) {
             unsigned char *data = skb->data;
             printk("brup_proxy: tx IP: src=%pI4 dst=%pI4 proto=0x%x\n",
                     data + 12, data + 16, *(data + 9));
        }
#endif

	/*
	 * Unicast
	 */
	if (0 == (dest[0] & 1)) {
		if (!compare_ether_addr(br->dev->dev_addr, dest)) {
			skb->pkt_type = PACKET_HOST;
			br_pass_frame_up(br, skb);
			return;
		}

		p_dst = _get_port_for_da_proxied(br, dest);

		if (p_dst) {
			_deliver(p_dst, skb);
		} else {
			br->statistics.tx_dropped++;
			kfree_skb(skb);
		}
		return;
	}

	if (br_mcast_dest_is_allowed(dest)) {
		struct net_bridge_port *destp;
		list_for_each_entry_rcu(destp, &br->port_list, list) {
			struct sk_buff *skb2;
			if (p == destp)
				continue;

			if (destp->is_satellite || destp->is_uplink) {
				if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
					br->statistics.tx_dropped++;
					continue;
				}
				_deliver(destp, skb2);
			}
		}
		br_pass_frame_up(br, skb);
	} else {
		kfree_skb(skb);
	}
	return;
}
#endif

/*
 * Packets that are sourced on our device that need to be delivered all come
 * through here if we are in uplink mode.
 *
 * Pretty simple stuff (convert
 * multicast to unicast, all unicast and broadcast packets get forwarded over
 * the uplink).
 */

void br_uplink_xmit(struct net_bridge *br,
		    struct sk_buff *skb,
		    const char *dest)
{
	struct net_bridge_port *p_uplink, *p_dst;
	struct sk_buff *skb2;
	struct net_bridge_mcast_entry *me;
	struct net_bridge_mcast_rx_port* mrxp;

#ifdef BR_DEBUG_UPLINK
	printk("brup: tx: p=%04x d=%02x:%02x:%02x:%02x:%02x:%02x\n",
	       eth_hdr(skb)->h_proto,
	       dest[0], dest[1], dest[2],
	       dest[3], dest[4], dest[5]);
        if (eth_hdr(skb)->h_proto == htons(ETH_P_IP)) {
		unsigned char *data = skb->data;
		printk("brup: tx IP: src=%pI4 dst=%pI4 proto=0x%x\n",
				data + 12, data + 16, *(data + 9));
        }
#endif

	/* 
	 * Unicast
	 */
	if (0 == (dest[0] & 1)) {
		p_dst = _get_port_for_da(br, dest);

		if (p_dst) {
			_deliver(p_dst, skb);
		} else {
			br->statistics.tx_dropped++;
			kfree_skb(skb);
		}
		return;
	}

	/* 
	 * Broadcast
	 *
	 * This is a little harder.  If it is a known multicast group we
	 * convert to unicast, otherwise it just goes to the AP.
	 *
	 * REVIEW: We filter some multicasts in SonosNet mode.  Perhaps we
	 *         should filter here as well?
	 */
	p_uplink = _get_uplink(br);

	me = br_mcast_get(br, dest);

	if (NULL == me) {
		/* No Sonos multicast group, so make sure this is something we
		 * really want to send.
		 *
		 * This is primarily to avoid sending audio as multicast.  If
		 * the multicast group is unknown because the AP is stripping
		 * our group join messages, we want to know this.
		 */

#ifdef CONFIG_SONOS_BRIDGE_PROXY
		/* If no multicast group, unicast to each satellite. */
		struct net_bridge_port *p;
		if (br->proxy_mode &&
		    (br_mcast_dest_is_allowed(dest) ||
		     br_mcast_dest_is_allowed_from_local(dest))) {
			struct sk_buff *skb2;
			list_for_each_entry_rcu(p, &br->port_list, list) {
				if (!p->is_satellite)
					continue;
				if ((skb2 = skb_copy(skb, GFP_ATOMIC)) == NULL) {
					br->statistics.tx_dropped++;
					continue;
				}
				_deliver(p, skb2);
			}
		}
#endif

		if (p_uplink &&
		    (br_mcast_dest_is_allowed(dest) ||
		     br_mcast_dest_is_allowed_from_local(dest))) {

			_deliver(p_uplink, skb);

		} else {
			br->statistics.tx_dropped++;
			kfree_skb(skb);
		}
		return;
	}

#ifdef BR_DEBUG_UPLINK
	printk("brup: tx: mc\n");
#endif

	/* Walk the multicast entry, unicasting to each destination */
	for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {

		struct net_bridge_port* p = mrxp->dst;
		/* Walk the MACs for each port (should all be in the
		 * uplink port, so there should only be one).
		 */
		struct net_bridge_mcast_rx_mac* mrxm;
		struct net_bridge_port* p_dst;

		/* Skip local ports */
		if (!p) {
			continue;
		}

			
		for (mrxm = mrxp->rx_mac_list;
		     mrxm != NULL;
		     mrxm = mrxm->next) {

			/* This should always be true? */
			if (!p->is_p2p) {
				continue;
			}

			/* Direct or uplink? */
			if (mrxm->direct_dst && mrxm->direct_dst->direct_enabled) {
				p_dst = mrxm->direct_dst;
			} else {
				p_dst = p_uplink;
			}

			/* Copy, adjust DA, send */
			if (p_dst && (skb2 = skb_copy(skb, GFP_ATOMIC))) {

				memcpy((char *)(eth_hdr(skb2)->h_dest),
				       mrxm->addr,
				       ETH_ALEN);
				
				_deliver(p_dst, skb2);

			} else {
				br->statistics.tx_dropped++;
				break;
			}
		}
	}

 	br_mcast_put(me);
	kfree_skb(skb);
}

#ifdef CONFIG_SONOS_BRIDGE_PROXY

/* Construct a sonosnet address from a serial address. There are two
 * possibilities based on the evenness of the serial address.
 */
inline void
br_construct_serial_from_sonosnet_addr(unsigned char* addr_target,
				       unsigned char* sonosnet)
{
	if (sonosnet[0] & 0x2) { /* odd */
		memcpy(addr_target + 1, sonosnet + 1, ETH_ALEN - 1);
		addr_target[0] = (sonosnet[0] & ~(0x2));
	} else { /* even */
		memcpy(addr_target, sonosnet, ETH_ALEN - 1);
		addr_target[ETH_ALEN - 1] = (sonosnet[ETH_ALEN - 1] & ~(0x1));
	}
}

/* This function proxies a frame from uplink to a satellite. It returns
 * true if we proxy-forwarded the frame. */
int br_uplink_proxy_frame_down(struct net_bridge *br, struct sk_buff *skb)
{
	/* Find out whether this a frame that needs to be proxied from a
	 * direct-route port or the uplink to a satellite. We know
	 * the destination address is our bridge by contract with the
	 * caller, but now we have to find out which of the satellites
	 * that we're aware of should actually be forwarded the frame.
	 *
	 * We find out if it needs to be proxied:
	 * a. If it's an ARP frame, we check to see if the target IP
	 * address is in the neighbor table.
	 * b. Otherwise, if it's an IPv4 frame, then we check to see if
	 * the destination IP address is in the neighbor table.
	 *
	 * We then use the neighbor table's hardware address entry to
	 * forward the frame. We've made sure elsewhere that ARP frames
	 * originating from satellites populate our ARP table.
	 *
	 * If the frame is actually destined to our IP, then we'll fail
	 * to find a p2p link whose dest_addr matches the neighbor entry
	 * we found, and we'll return 0.
	 */

	__be32 *target_ip;
	struct net_bridge_port *p;

	skb->network_header = (unsigned char *)(eth_hdr(skb) + 1);
	if (eth_hdr(skb)->h_proto == htons(ETH_P_ARP)) {
		int tgt_ip_idx = ETH_ALEN + sizeof(__be32) + ETH_ALEN;
		unsigned char *arp_ptr = (unsigned char *)(arp_hdr(skb) + 1);

		if (!pskb_may_pull(skb, sizeof(struct arphdr) +
                                   2 * (ETH_ALEN + sizeof(__be32))))
			goto end;

		/* Offset into ARP payload. There's no convenient
		 * data structure that does this for us. */
		target_ip = (__be32 *)&arp_ptr[tgt_ip_idx];

	} else if (eth_hdr(skb)->h_proto == htons(ETH_P_IP))  {
		if (!pskb_may_pull(skb, sizeof(struct iphdr)))
			goto end;
		target_ip = &ip_hdr(skb)->daddr;
	} else {
		goto end;
	}

	/* If the target address matches us, just deliver to us instead of
	 * looking for a satellite, for consistency's sake. */
	if (br->current_ipv4_addr && br->current_ipv4_addr == *target_ip) {
		return 0;
	}

	list_for_each_entry_rcu(p, &br->port_list, list) {
		struct iphdr *iph = NULL;
		struct udphdr *udph = NULL;
		unsigned char *dhcph = NULL;

		if (!(p->sat_ip == *target_ip && p->is_satellite))
			continue;
		/* proxy forward it down! */
		br_construct_serial_from_sonosnet_addr(eth_hdr(skb)->h_dest, p->p2p_dest_addr);
		if (eth_hdr(skb)->h_proto == htons(ETH_P_ARP)) {
			int tgt_mac_offset = ETH_ALEN + sizeof(__be32);
			unsigned char* tgt_mac = (unsigned char *)(arp_hdr(skb) + 1)
						 + tgt_mac_offset;
			memcpy(tgt_mac, eth_hdr(skb)->h_dest, ETH_ALEN);
		}

		if (eth_hdr(skb)->h_proto == htons(ETH_P_IP) &&
			br_is_dhcp(skb, &iph, &udph, &dhcph)) {
			br_log_dhcp(udph, dhcph);
		}

		__br_forward(p, skb);
		return 1;
	}
end:
	/* we didn't deliver it. */
	return 0;

}
#endif

static int _br_uplink_handle_ipv4_broadcast(struct net_bridge *br, struct sk_buff *skb)
{
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	/* Yes, this is a bit nasty. We set the ethernet address to
	 * broadcast if the IPv4 address happens to be broadcast. This
	 * is to work around a particular type of wireless extender that
	 * mangles broadcast DHCP datagrams to be unicast at ethernet
	 * (MCS-1555). */
	if (br->proxy_mode && eth_hdr(skb)->h_proto == htons(ETH_P_IP))  {
		__u32 target_ip;
		if (!pskb_may_pull(skb, sizeof(struct iphdr)))
			goto end;
		target_ip = ntohl(ip_hdr(skb)->daddr);
		if (target_ip == INADDR_BROADCAST) {
			memcpy(eth_hdr(skb)->h_dest, broadcast_addr, sizeof(broadcast_addr));
			return 1;
		}
	}
end:
#endif
	return 0;
}
/*
 * Packets that have been received from the uplink or a direct-route
 * tunnel are handled here. If we're not proxying, there's no forwarding,
 * and we just have to determine whether we're the destination and to
 * pass it up or not. If we're proxying, we determine the appropriate
 * proxy destination via br_proxy_frame_down().
 */
struct sk_buff *br_uplink_handle_frame(struct net_bridge_port *p,
				       struct sk_buff *skb,
				       const unsigned char *src,
				       const unsigned char *dst)
{
	struct net_bridge *br = p->br;
	int passup = 0;

	/*
	 * REVIEW: Optimize by dropping multicasts we don't care about?
	 *
	 * It looks like all broadcast traffic gets passed up, which strikes me
	 * as odd.  I'd think that multicast traffic for a multicast group we
	 * are not part of would be dropped, but what do I know?
	 *
	 */
	
	/* Always check for multicast table updates */
	br_mcast_check(skb, p->br, p);

	/* KLUDGE: Why are we seeing broadcast packets that we sent?
	 *         It looks like we see them when the AP sends them, which is
	 *         irritating at best.  Higher layers drop them, but they
	 *         making debugging a PITA.  I'm stripping them for now.
	 */
	if (0 == memcmp(src, br->dev->dev_addr, 6)) {
		goto drop;
	}

#ifdef BR_DEBUG_UPLINK
	printk("brup: rx: p=%04x s=%02x:%02x:%02x:%02x:%02x:%02x "
	       "d=%02x:%02x:%02x:%02x:%02x:%02x (%d,%d)\n",
	       eth_hdr(skb)->h_proto,
	       src[0], src[1], src[2],
	       src[3], src[4], src[5],
	       dst[0], dst[1], dst[2],
	       dst[3], dst[4], dst[5],
	       p->port_no,
	       skb->len);                        
#endif

	/*
	 * Unless proxying, we either pass up or drop. Only proxied
	 * frames are forwarded toward satellites.
	 */
	passup = 0;

	if (dst[0] & 1 || _br_uplink_handle_ipv4_broadcast(br, skb)) {
		passup = 1;
#ifdef CONFIG_SONOS_BRIDGE_PROXY
		if (br->proxy_mode) {
			/* Ok to flood here because should_deliver() will
			 * cause us to flood only the satellites. */
			br_flood_forward(br, p, skb, 1);
		}
#endif
	} else if (0 == memcmp(dst, br->dev->dev_addr, 6)) {
		br_fdb_update(p->br, p, src);
#ifdef CONFIG_SONOS_BRIDGE_PROXY
		if (br_uplink_proxy_frame_down(br, skb)) {
			return NULL;
		}
#endif
		passup = 1;
		skb->pkt_type = PACKET_HOST;
	}

	if (passup) {
		br_pass_frame_up(br, skb);
		return NULL;
	} else {
#ifdef BR_DEBUG_UPLINK
		printk("brup: rx: drop\n");
#endif
	}

drop:
	kfree_skb(skb);
	return NULL;
}

void br_set_uplink_mode(struct net_bridge *br, int enable)
{
	spin_lock_bh(&br->lock);

	if (enable != br->uplink_mode) {

		br->uplink_mode = enable;

#ifdef CONFIG_SONOS_BRIDGE_PROXY
		printk("%s: Setting proxy mode to %d", br->dev->name, enable);
		br->proxy_mode = enable;
#endif

		/*
		 * REVIEW: Do we really want to automatically enable/disable
		 *         STP?  I can't see any good reason why not?
		 */
		br->stp_enabled = !enable;

		/*
		 * Bounce everything that needs bouncing.  Multicast table and
		 * FDB seem like good candidates since all of the forwarding
		 * rules are about to change (as are the aging timers).
		 */
		br_mcast_destroy_list(br);
		br_fdb_delete_non_local(br);

		/* We send 4x more join messages in uplink mode */
		if (br->uplink_mode) {
			br->mcast_ageing_time    = 120 * HZ;
			br->mcast_advertise_time =   5 * HZ;
		} else {
			br->mcast_ageing_time    =  60 * HZ;
			br->mcast_advertise_time =  10 * HZ;
		}
	}

	spin_unlock_bh(&br->lock);
}
