/*
 *	Uplink mode extensions
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include "br_private.h"

/* #define BR_DEBUG_UPLINK 1 */

static struct net_bridge_port *_get_uplink(struct net_bridge *br)
{
	struct net_bridge_port *p_tmp;

	p_tmp = br->port_list;
	while (p_tmp != NULL) {
		if (p_tmp->is_uplink) {
			break;
		}
		p_tmp = p_tmp->next;
	}

	return p_tmp;
}

static struct net_bridge_port *_get_port_for_da(struct net_bridge *br,
						const unsigned char *da)
{
	struct net_bridge_port *p_tmp, *p_dst = 0, *p_uplink = 0;

	p_tmp = br->port_list;
	while (p_tmp != NULL) {

		if ((p_tmp->direct_enabled) &&
		    (0 == memcmp(da, p_tmp->direct_addr, 6))) {
			p_dst = p_tmp;
			break;
		}

		if (p_tmp->is_uplink) {
			p_uplink = p_tmp;
		}

		p_tmp = p_tmp->next;
	}

	if (!p_dst)
		p_dst = p_uplink;

	return p_dst;
}

static inline void _deliver(struct net_bridge_port *port, struct sk_buff *skb) {

	if (!skb->priority)
		skb->priority = port->priority;

	br_deliver_direct(0, port, skb);
}

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
		    const unsigned char *dest)
{
	struct net_bridge_port *p_uplink, *p_dst;
	struct sk_buff *skb2;

#ifdef BR_DEBUG_UPLINK
	printk("brup: tx: d=%02x:%02x:%02x:%02x:%02x:%02x\n",
	       dest[0], dest[1], dest[2],
	       dest[3], dest[4], dest[5]);
#endif

	/* 
	 * Unicast
	 */
	if (0 == (dest[0] & 1)) {

		p_dst = _get_port_for_da(br, dest);

#ifdef BR_DEBUG_UPLINK
		printk("brup: tx: uc: p_dst=%p (%d)\n", 
		       p_dst, p_dst ? p_dst->is_uplink : -1);
#endif

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
        
	struct net_bridge_mcast_entry *me = br_mcast_get(br, dest);

	if (NULL == me) {

		/* No Sonos multicast group, so make sure this is something we
		 * really want to send.
		 *
		 * This is primarily to avoid sending audio as multicast.  If
		 * the multicast group is unknown because the AP is stripping
		 * our group join messages, we want to know this.
		 */
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
	struct net_bridge_mcast_rx_port* mrxp;
	for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {

		struct net_bridge_port* p = mrxp->dst;

		/* Skip local ports */
		if (!p) {
			continue;
		}

		/* Walk the MACs for each port (should all be in the
		 * uplink port, so there should only be one).
		 */
		struct net_bridge_mcast_rx_mac* mrxm;
		struct net_bridge_port* p_dst;
			
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

				memcpy(skb2->mac.ethernet->h_dest,
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

/*
 * Packets that have been received from the outside are handled here.  We do
 * *not* forward when in uplink mode, so this is an exercise in determining if
 * we are the destination or not.
 *
 */
struct sk_buff *br_uplink_handle_frame(struct net_bridge_port *p,
				       struct sk_buff *skb,
				       const unsigned char *src,
				       const unsigned char *dst)
{
	struct net_bridge *br = p->br;

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
	if (0 == memcmp(src, br->dev.dev_addr, 6)) {
		goto drop;
	}

#ifdef BR_DEBUG_UPLINK
	printk("brup: rx: s=%02x:%02x:%02x:%02x:%02x:%02x "
	       "d=%02x:%02x:%02x:%02x:%02x:%02x (%d,%d)\n",
	       src[0], src[1], src[2],
	       src[3], src[4], src[5],
	       dst[0], dst[1], dst[2],
	       dst[3], dst[4], dst[5],
	       p->port_no,
	       skb->len);                        
#endif

	/*
	 * We either pass up or drop.  Nothing gets forwarded.
	 */
	int passup = 0;

	if (dst[0] & 1) {
		passup = 1;
	} else if (0 == memcmp(dst, br->dev.dev_addr, 6)) {
		passup = 1;		
		skb->pkt_type = PACKET_HOST;
		br_fdb_insert(br, p, skb->mac.ethernet->h_source, 0);
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

		/* 
		 * REVIEW: Do we really want to automatically enable/disable STP?  I
		 *         can't see any good reason why not?
		 */
		br->stp_enabled = !enable;
	
		/*
		 * Bounce everything that needs bouncing.  Multicast table and
		 * FDB seem like good candidates since all of the forwarding
		 * rules are about to change.
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
