/*
 *	Handle incoming frames
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/netfilter_bridge.h>
#include <linux/export.h>
#include <linux/rculist.h>
#include "br_private.h"

#include <linux/ip.h>

const unsigned char bridge_ula[6] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x00 };

/* #define DEBUG_BR_INPUT 1 */

/* Make sure nobody thinks this will work.  See SONOS: NETFILTER below */
#ifdef CONFIG_BRIDGE_NETFILTER
#error "No netfilter for you!"
#endif

static void br_log_bcmc_history(struct net_bridge *br,
                                unsigned char *src,
                                unsigned char *dest,
                                unsigned long timestamp,
                                unsigned long packet_count,
                                u8 packet_type)
{
	struct net_bridge_bcmc_hit *bcmc_hit;
	bcmc_hit = &(br->br_stats.bcmc_history[br->br_stats.bcmc_index]);
	br->br_stats.bcmc_index = (br->br_stats.bcmc_index + 1) % BR_BCMC_HIST_SIZE;

	memcpy(bcmc_hit->src, src, ETH_ALEN);
	memcpy(bcmc_hit->dest, dest, ETH_ALEN);
	bcmc_hit->timestamp = timestamp;
	bcmc_hit->packet_type = packet_type;
	bcmc_hit->packet_count = packet_count;
}

static void br_stats_update(struct net_bridge *br,
                            const unsigned char *src,
                            const unsigned char *dest)
{
	unsigned long now = jiffies;
	if (time_after_eq(now, br->br_stats.rx_start_time + RX_STATS_CHECK_INTERVAL*HZ)) {
		unsigned long timestamp = jiffies/HZ;

		if (br->br_stats.rx_bc_count >= BCMC_REPORT_TOTAL_PACKETS) {
			br_log_bcmc_history(br, br->br_stats.rx_bc_hit_src, br->br_stats.rx_bc_hit_dest,
						timestamp, br->br_stats.rx_bc_count, BCAST_TYPE);
			++br->br_stats.rx_bc_hit;
		}
		if (br->br_stats.rx_mc_count >= BCMC_REPORT_TOTAL_PACKETS) {
			br_log_bcmc_history(br, br->br_stats.rx_mc_hit_src, br->br_stats.rx_mc_hit_dest,
						timestamp, br->br_stats.rx_mc_count, MCAST_TYPE);
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
		if (br->br_stats.rx_bc_count == BCMC_REPORT_TOTAL_PACKETS) {
			memcpy(br->br_stats.rx_bc_hit_src, src, ETH_ALEN);
			memcpy(br->br_stats.rx_bc_hit_dest, dest, ETH_ALEN);
		}
	} else if (is_multicast_ether_addr(dest)) {
		++br->br_stats.rx_mc_count;
		if (br->br_stats.rx_mc_count == BCMC_REPORT_TOTAL_PACKETS) {
			memcpy(br->br_stats.rx_mc_hit_src, src, ETH_ALEN);
			memcpy(br->br_stats.rx_mc_hit_dest, dest, ETH_ALEN);
		}
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
int br_handle_frame_finish(struct net_bridge_port *p,
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

	if ((dst != NULL && dst->is_local) || skb->pkt_type == PACKET_HOST) {
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
				br_stats_update(p->br, src, dest);
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
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	else if (BR_SKB_CB(skb)->should_proxy_up) {
		br_uplink_proxy(p->br, p, skb, dest);
		return NULL;
	}
#endif

	kfree_skb(skb);
	return NULL;
}

#ifdef CONFIG_SONOS_BRIDGE_PROXY
/* Check whether a serial (bridge) address matches a sonosnet address.
 * If the bridge address is even, then it's simply checking that they
 * match with the last bit of the bridge set. If the bridge address is
 * odd, it's checking if they match with the second bit of the first byte
 * of the bridge set (the 'local admin' bit).
 */
inline int
br_check_sonosnet_serial_match(const unsigned char* sonosnet,
			       const unsigned char* serial)
{
	int ret;
	if (serial[ETH_ALEN - 1] & 0x1) { /* odd */
		ret = !memcmp(sonosnet + 1, serial + 1, ETH_ALEN - 1);
		ret = ret && ((serial[0] | 0x2) == sonosnet[0]);
	} else { /* even */
		ret = !memcmp(sonosnet, serial, ETH_ALEN - 1);
		ret = ret && ((serial[ETH_ALEN - 1] | 0x1) == sonosnet[ETH_ALEN - 1]);
	}
	return ret;
}
#endif

/*
 * Called via br_handle_frame_hook.
 * Return NULL if skb is handled
 * note: already called with rcu_read_lock (preempt_disabled)
 */
struct sk_buff *br_handle_frame(struct net_bridge_port_list_node *pl,
                                struct sk_buff *skb)
{
	const unsigned char *dest = eth_hdr(skb)->h_dest;
	const unsigned char *src  = eth_hdr(skb)->h_source;
	struct net_bridge_port *p ;
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
		goto drop;
        }

	if (!is_valid_ether_addr(eth_hdr(skb)->h_source)) {
		goto drop;
	}

	p = br_find_port(src, pl);

	if (!p || (p->state == BR_STATE_DISABLED && !p->br->uplink_mode)) {
		goto drop;
	}

#ifdef CONFIG_SONOS_BRIDGE_PROXY
	BR_SKB_CB(skb)->source_port = p;
	BR_SKB_CB(skb)->should_proxy_up = 0;
#endif

	/* If the frame arrived on a port that is the endpoint of a
	 * point-to-point packet tunnel, it should be using the tunnel
	 * protocol number.  If so, pull off the header to obtain the
         * tunneled ethernet frame; if not, ignore the packet.
	 *
	 * NOTE: We now have a learned and an unlearned tunnel.  This is a key
	 *       part of how direct routing works.
	 */
        if (p->is_p2p && !p->is_unencap) {

		if (eth_hdr(skb)->h_proto != htons(BR_TUNNEL_PROTOCOLNUM)) {

			if (eth_hdr(skb)->h_proto != htons(BR_TUNNEL2_PROTOCOLNUM)) {
				src  = eth_hdr(skb)->h_source;
                                print_hex_dump(KERN_INFO, "bad proto ethhdr ", DUMP_PREFIX_NONE, 32, 1,
                                               eth_hdr(skb), sizeof(struct ethhdr), false);
                                /* Log first 32 bytes of skb->data */
                                print_hex_dump(KERN_INFO, "skb->data ", DUMP_PREFIX_NONE, 32, 1,
                                               skb->data, min(skb->len, 32), false);

				goto drop;
			}
			direct = 1;
		}

		/*
		 * Unencapsulate the tunneled ethernet frame.  Note that it
		 * looks like we need to update the protocol here (2.4
		 * calculates it *way* later on, but 2.6 doesn't). Use
		 * the eth_type_trans() function that drivers (including
		 * ours) normally use. This accounts for the fact that
		 * we're unencapsulating a frame, and will set
		 * skb->pkt_type correctly.
		 */
		skb->protocol = eth_type_trans(skb, p->br->dev);

#ifdef CONFIG_SONOS_BRIDGE_PROXY
		if (p->br->proxy_mode &&
		    p->is_satellite &&
		    br_check_sonosnet_serial_match(src, eth_hdr(skb)->h_source)) {
			/* We only want to proxy to the uplink when the
			 * wireless TA equals the actual (encapsulated)
			 * source address. This is to ensure that we only
			 * proxy for satellites that are a single hop
			 * away. We also want to proxy up only for frames
			 * originating from satellites. */
			BR_SKB_CB(skb)->should_proxy_up = 1;
		}
#endif

		dest = eth_hdr(skb)->h_dest;
		src  = eth_hdr(skb)->h_source;

#ifdef DEBUG_BR_INPUT
		printk("hf unencap: p=%04x, s=" MAC_ADDR_FMT ", d=" MAC_ADDR_FMT "\n",
		       eth_hdr(skb)->h_proto,
		       MAC_ADDR_VAR(src),
		       MAC_ADDR_VAR(dest));
		if (eth_hdr(skb)->h_proto == htons(ETH_P_IP)) {
			unsigned char *data = skb->data;
			printk("hf unencap IP: src=%pI4 dst=%pI4 proto=%x\n",
				data + 12, data + 16, *(data + 9));
		}
#endif
        }
	if (p->br->uplink_mode) {
		/* Anything that didn't come from the AP needs to have come
		 * over the unlearned tunnel
		 */
		if (p->is_uplink || direct) {
			return br_uplink_handle_frame(p, skb, src, dest);
		}
#ifdef CONFIG_SONOS_BRIDGE_PROXY
                else if (BR_SKB_CB(skb)->should_proxy_up) {
			return br_handle_frame_std(p, skb, src, dest, direct);
                }
#endif
	} else {
		return br_handle_frame_std(p, skb, src, dest, direct);
	}

drop:
	kfree_skb(skb);
	return NULL;
}

rx_handler_result_t sonos_br_handle_frame(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct net_bridge_port_list_node *pl = skb->dev->br_port_list;

	if (unlikely(skb->pkt_type == PACKET_LOOPBACK))
		return RX_HANDLER_PASS;

	if ( pl == NULL )
		return RX_HANDLER_PASS;

	skb = br_handle_frame(pl, skb);
	if ( skb == NULL )
		return RX_HANDLER_CONSUMED;
	return RX_HANDLER_PASS;
}
