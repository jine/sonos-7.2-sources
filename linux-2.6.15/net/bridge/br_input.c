/*
 *	Handle incoming frames
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br_input.c#1 $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/netfilter_bridge.h>
#include "br_private.h"

#include <linux/ip.h>

const unsigned char bridge_ula[6] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x00 };

/* Make sure nobody thinks this will work.  See SONOS: NETFILTER below */
#ifdef CONFIG_BRIDGE_NETFILTER
#error "No netfilter for you!"
#endif

static void br_stats_update(struct net_bridge *br, const unsigned char *dest)
{
	unsigned long now = jiffies;
	unsigned long timestamp = now/HZ;
	if (time_after_eq(now, br->br_stats.rx_start_time + RX_STATS_CHECK_INTERVAL*HZ)) {
		unsigned long threshold = RX_STATS_CHECK_INTERVAL*BCMC_REPORT_FPS_THRESHOLD;
		if (br->br_stats.rx_bc_count >= threshold) {
			++br->br_stats.rx_bc_hit;
		}
		if (br->br_stats.rx_mc_count >= threshold) {
			++br->br_stats.rx_mc_hit;
		}
		if (br->br_stats.rx_bc_count > br->br_stats.rx_bc_count_peak) {
			br->br_stats.rx_bc_count_peak = br->br_stats.rx_bc_count;
			br->br_stats.rx_bc_peak_ts = timestamp;
		}
		if (br->br_stats.rx_mc_count > br->br_stats.rx_mc_count_peak) {
			br->br_stats.rx_mc_count_peak = br->br_stats.rx_mc_count;
			br->br_stats.rx_mc_peak_ts = timestamp;
		}
		br->br_stats.rx_bc_count = 0;
		br->br_stats.rx_mc_count = 0;
		br->br_stats.rx_start_time = now;
	}
	if (is_broadcast_ether_addr(dest)) {
		++br->br_stats.rx_bc_count;
	} else if (is_multicast_ether_addr(dest)) {
		++br->br_stats.rx_mc_count;
	}
}

static int br_pass_frame_up_finish(struct sk_buff *skb)
{
	netif_receive_skb(skb);
	return 0;
}

void br_pass_frame_up(struct net_bridge *br, struct sk_buff *skb)
{
	struct net_device *indev;

	br->statistics.rx_packets++;
	br->statistics.rx_bytes += skb->len;

	indev = skb->dev;
	skb->dev = br->dev;

        
	NF_HOOK(PF_BRIDGE, NF_BR_LOCAL_IN, skb, indev, NULL,
			br_pass_frame_up_finish);
}

/* note: already called with rcu_read_lock (preempt_disabled) */
static int br_handle_frame_finish(struct net_bridge_port *p,
                                  struct sk_buff *skb)
{
	const unsigned char *dest = eth_hdr(skb)->h_dest;
	struct net_bridge *br = p->br;
	struct net_bridge_fdb_entry *dst;

	int passedup = 0;        
	if (br->dev->flags & IFF_PROMISC) {
		struct sk_buff *skb2;

		skb2 = skb_clone(skb, GFP_ATOMIC);
		if (skb2 != NULL) {			
			passedup = 1;
			br_pass_frame_up(br, skb2);
		}
	}

	if (dest[0] & 1) {
		br_flood_forward(br, p, skb, !passedup);
		if (!passedup)
			br_pass_frame_up(br, skb);
		goto out;
	}

	dst = __br_fdb_get(br, dest);

	if (dst != NULL && dst->is_local) {
		if (!passedup)
			br_pass_frame_up(br, skb);
		else
			kfree_skb(skb);
		goto out;
	}

	if (dst != NULL) {

		if (0 == skb->priority) {
			skb->priority = dst->priority;
		}

		/* NOTE: This was not sourced on this device, but we may still
		 *       want to direct route it.  Just make sure we forward
		 *       instead of deliver.
		 */
		br_direct_unicast(p, dst, skb,
				  br_forward,
                                  br_forward_direct);
		goto out;
	}

	br_flood_forward(br, p, skb, 0);

out:
	return 0;
}


/* scan the list of bridge ports associated with an interface to find the
   one on which a packet arrived. */
struct net_bridge_port* br_find_port(const unsigned char *h_source,
                                     struct net_bridge_port_list_node *pl)
{
	struct net_bridge_port *uplink = NULL;

        while (pl) {	
		if (pl->port->is_p2p) {
			if (0 == memcmp(pl->port->p2p_dest_addr, h_source, ETH_ALEN))
				return pl->port;
			if (pl->port->is_uplink) {
				uplink = pl->port;
			}
		} else {
			/* this is a normal bridge port, not a point-to-point tunnel */
			return pl->port;
		}

		pl = pl->next;
        }

	/*
	 * REVIEW:  This is most unfortunate, but we need to assume that the
	 *          source of the packet is the uplink port if we didn't match
	 *          on any port but we did find an uplink.  There should be no
	 *          other way into the bridge, but I really can't verify that
	 *          this packet is good in any way that makes me happy.
	 */
	if (uplink) {
		return uplink;
	}

        return 0;
}

static inline struct sk_buff *br_handle_frame_std(struct net_bridge_port *p,
						  struct sk_buff *skb,
						  const unsigned char *src,
						  const unsigned char *dest,
						  int direct)
{
	/* Learning */
	if (p->state == BR_STATE_LEARNING ||
	    p->state == BR_STATE_FORWARDING ||
	    direct) {

		/* Only learn if the packet was not routed directly */
		if (!direct) {
			struct net_bridge_fdb_entry *fdb;
			fdb = br_fdb_update(p->br, p, src);
			if (fdb && fdb->priority == 0) {
				br_stats_update(p->br, dest);
			}
		}

		/* Always check for table updates regardless of source */
		br_mcast_check(skb, p->br, p);
	}

	/* STP maintenance */
	if (p->br->stp_enabled &&
	    !memcmp(dest, bridge_ula, 5) &&
	    !(dest[5] & 0xF0)) {
		if (!dest[5]) {
			br_stp_handle_bpdu(p, skb);
			return NULL;
		}
	}

	/* Forwarding */
	else if (p->state == BR_STATE_FORWARDING || direct) {

		if (!compare_ether_addr(p->br->dev->dev_addr, dest))
			skb->pkt_type = PACKET_HOST;

		br_handle_frame_finish(p, skb);
		return NULL;
	}

drop:
	kfree_skb(skb);
	return NULL;
}

/*
 * Called via br_handle_frame_hook.
 * Return 0 if *pskb should be processed furthur
 *	  1 if *pskb is handled
 * note: already called with rcu_read_lock (preempt_disabled) 
 */
int br_handle_frame(struct net_bridge_port_list_node *pl, struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	const unsigned char *dest = eth_hdr(skb)->h_dest;
	const unsigned char *src  = eth_hdr(skb)->h_source;
        int direct = 0;
	
#ifdef DEBUG_BR_INPUT
	printk("hf: p=%04x, s=" MAC_ADDR_FMT ", d=" MAC_ADDR_FMT "\n",
	       eth_hdr(skb)->h_proto,
	       MAC_ADDR_VAR(src),
	       MAC_ADDR_VAR(dest));
#endif

	/*
	 * (in)sanity checks
	 */
        if (NULL == pl || NULL == pl->port) {
		goto exit;
        }

	if (!is_valid_ether_addr(eth_hdr(skb)->h_source)) {
		goto exit;
	}

        struct net_bridge_port *p = br_find_port(src, pl);

	if (!p || (p->state == BR_STATE_DISABLED && !p->br->uplink_mode)) {
		goto exit;
	}
        
	/* if the frame arrived on a port that is the endpoint of a 
           point-to-point packet tunnel, it should be using the tunnel
           protocol number.  If so, pull off the header to obtain the 
           tunneled ethernet frame; if not, ignore the packet. */

        if (p->is_p2p && !p->is_unencap) {
		
		if (eth_hdr(skb)->h_proto != htons(BR_TUNNEL_PROTOCOLNUM)) {
			if (eth_hdr(skb)->h_proto != htons(BR_TUNNEL2_PROTOCOLNUM)) {
				printk("bad proto\n");
				goto exit;
			}
			
			direct = 1;
 		}
		
		/*
		 * Unencapsulate the tunneled ethernet frame.  Note that it
		 * looks like we need to update the protocol here (2.4
		 * calculates it *way* later on, but 2.6 doesn't).
		 */
                skb_pull(skb, ETH_HLEN);
		skb->mac.raw += ETH_HLEN;
		skb->protocol = eth_hdr(skb)->h_proto;
		
		dest = eth_hdr(skb)->h_dest;
		src  = eth_hdr(skb)->h_source;                
        }

	if (p->br->uplink_mode) {
		/* Anything that didn't come from the AP needs to have come
		 * over the unlearned tunnel 
		 */
		if (p->is_uplink || direct) {
			skb = br_uplink_handle_frame(p, skb, src, dest);
		}
	} else {
		skb = br_handle_frame_std(p, skb, src, dest, direct);
	}

exit:
	if (skb) {
		kfree_skb(skb);
	}
	return 1;
}
