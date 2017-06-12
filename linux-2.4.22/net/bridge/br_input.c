/*
 *	Handle incoming frames
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_input.c,v 1.2 2004/01/30 00:07:03 millington Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/netfilter_bridge.h>
#include "br_private.h"

unsigned char bridge_ula[6] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x00 };

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
	if (dest[0] == 0xFF) {
		++br->br_stats.rx_bc_count;
	} else if (dest[0] & 0x01) {
		++br->br_stats.rx_mc_count;
	}
}

static int br_pass_frame_up_finish(struct sk_buff *skb)
{
	netif_rx(skb);

	return 0;
}

void br_pass_frame_up(struct net_bridge *br, struct sk_buff *skb)
{
	struct net_device *indev;

	br->statistics.rx_packets++;
	br->statistics.rx_bytes += skb->len;

	indev = skb->dev;
	skb->dev = &br->dev;
	skb->pkt_type = PACKET_HOST;
	skb_push(skb, ETH_HLEN);
	skb->protocol = eth_type_trans(skb, &br->dev);

	br_pass_frame_up_finish(skb);
}

static int br_handle_frame_finish(struct net_bridge_port *from,
                                  struct sk_buff *skb)
{
	struct net_bridge *br;
	unsigned char *dest;
	struct net_bridge_fdb_entry *dst;
	struct net_bridge_port_list_node *pl;
	int passedup;

	dest = skb->mac.ethernet->h_dest;

	pl = skb->dev->br_port_list;
	if (pl == NULL)
		goto err_nolock;

	br = pl->port->br;
	read_lock(&br->lock);
	if (skb->dev->br_port_list == NULL)
		goto err;

	passedup = 0;
	if (br->dev.flags & IFF_PROMISC) {
		struct sk_buff *skb2;

		skb2 = skb_clone(skb, GFP_ATOMIC);
		if (skb2 != NULL) {
			passedup = 1;
			br_pass_frame_up(br, skb2);
		}
	}

	if (dest[0] & 1) {
		br_flood_forward(br, from, skb, !passedup);
		if (!passedup)
			br_pass_frame_up(br, skb);
		goto out;
	}

	dst = br_fdb_get(br, dest);

	if (dst == NULL) {

		/* REVIEW: I don't like this extra memcmp for unknown MACs, but
		 *         learning the MAC is even more problematic since
		 *         there is no real port for it if we are a purely
		 *         wireless bridge (no port will have the system's MAC
		 *         since the p2p ports tend to be "system MAC + 1" or
		 *         "system MAC + 2").  Ugh...
		 */
		if (br->use_static_mac &&
		    0 == memcmp(br->static_mac, dest, ETH_ALEN)) {

                    if (!passedup)
                        br_pass_frame_up(br, skb);
                    else
                        kfree_skb(skb);
                    goto out;
                }

	} else if (dst->is_local) {
		
		if (!passedup)
			br_pass_frame_up(br, skb);
		else
			kfree_skb(skb);
		br_fdb_put(dst);
		goto out;
		
	} else {
		
		if (0 == skb->priority) {
			skb->priority = dst->priority;
		}

		/* NOTE: This was not sourced on this device, but we may still
		 *       want to direct route it.  Just make sure we forward
		 *       instead of deliver.
		 */
                br_direct_unicast(from, dst, skb,
                                  br_forward,
                                  br_forward_direct);
		br_fdb_put(dst);
		goto out;
	}

	br_flood_forward(br, from, skb, 0);

out:
	read_unlock(&br->lock);
	return 0;

err:
	read_unlock(&br->lock);
err_nolock:
	kfree_skb(skb);
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
	struct net_bridge *br = p->br;

	/* Learn */
	if (p->state == BR_STATE_LEARNING || p->state == BR_STATE_FORWARDING ||
	    direct) {

		/* Only learn if the packet was not routed directly */
		if (!direct) {
			struct net_bridge_fdb_entry *fdb;
			fdb = br_fdb_insert(br, p, skb->mac.ethernet->h_source, 0);
			if (fdb && fdb->priority == 0) {
				br_stats_update(p->br, dest);
			}
		}

		/* also maintain the multicast forwarding database here */
		br_mcast_check(skb, br, p);
        }

	/* Handle STP */
	if (br->stp_enabled &&
	    !memcmp(dest, bridge_ula, 5) &&
	    !(dest[5] & 0xF0)) {

		/* Handle BPDUs. br_stp_handle_bpdu() frees it. */
		if (0 == dest[5]) {
			br_stp_handle_bpdu(p, skb);
			return NULL;
		}
		
		/* Drop the packet regardless of the type */
		return skb;
	}

	/* Handle packets that we want to forward */
	if (p->state == BR_STATE_FORWARDING || direct) {
                br_handle_frame_finish(p, skb);
		return NULL;
	}

	/* Drop everything else */
	return skb;
}

void br_handle_frame(struct sk_buff *skb)
{
	struct net_bridge *br;
	unsigned char *dest, *src;
        struct net_bridge_port_list_node *pl;
	struct net_bridge_port *p;
	int direct = 0;

	pl = skb->dev->br_port_list;
	if (pl == NULL)
		goto err_nolock;

	br = pl->port->br;
	read_lock(&br->lock);
	if (skb->dev->br_port_list == NULL)
		goto err;

	if (!(br->dev.flags & IFF_UP))
		goto err;

	if (skb->mac.ethernet->h_source[0] & 1)
		goto err;

        /* figure out the port on which the packet arrived; there can be
           multiple ports per interface. */

        p = br_find_port(skb->mac.ethernet->h_source, pl);
        if (!p || (p->state == BR_STATE_DISABLED && !p->br->uplink_mode))
                goto err;

        /* if the frame arrived on a port that is the endpoint of a 
           point-to-point packet tunnel, it should be using the tunnel
           protocol number.  If so, pull off the header to obtain the 
           tunneled ethernet frame; if not, ignore the packet. */

        if (p->is_p2p && !p->is_unencap) {

		/* STP path or direct path? */
		if (skb->mac.ethernet->h_proto != htons(BR_TUNNEL_PROTOCOLNUM)) {
			if (skb->mac.ethernet->h_proto != htons(BR_TUNNEL2_PROTOCOLNUM)) {
				unsigned char *src  = skb->mac.ethernet->h_source;
				printk("bad proto %u from %02x%02x%02x%02x%02x%02x\n",
						ntohs(skb->mac.ethernet->h_proto),
						src[0], src[1], src[2], src[3], src[4], src[5]);
				goto err;
			}
			
			direct = 1;
 		}

            /* unencapsulate the tunneled ethernet frame. */
            skb->mac.ethernet = (struct ethhdr*)skb->data;
            skb_pull(skb, ETH_HLEN);

	    /* REVIEW: Should confirm that DA is local somewhere in the
	     *         transmit path.  If it is not, *bad* things will happen
	     *         later (like we'll learn this route in
	     *         br_handle_frame_finish).
	     */
        }

        src  = skb->mac.ethernet->h_source;
        dest = skb->mac.ethernet->h_dest;
	
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

err:
	read_unlock(&br->lock);
err_nolock:
	if (skb) {
		kfree_skb(skb);
	}
}
