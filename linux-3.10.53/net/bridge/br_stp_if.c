/*
 *	Spanning tree protocol; interface code
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

#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>

#include "br_private.h"
#include "br_private_stp.h"


/* Port id is composed of priority and port number.
 * NB: some bits of priority are dropped to
 *     make room for more ports.
 */
static inline port_id br_make_port_id(__u8 priority, __u16 port_no)
{
	return ((u16)priority << BR_PORT_BITS)
		| (port_no & ((1<<BR_PORT_BITS)-1));
}

/* called under bridge lock */
void br_init_port(struct net_bridge_port *p)
{
	p->port_id = br_make_port_id(p->priority, p->port_no);
	br_become_designated_port(p);
	p->state = BR_STATE_BLOCKING;
	p->topology_change_ack = 0;
	p->config_pending = 0;
	p->direct_enabled = 0;
	memset(&p->direct_addr, 0, 6);
	p->direct_last_stp_time = jiffies - BR_DIRECT_STP_TIME;
}

/* NO locks held */
void br_stp_enable_bridge(struct net_bridge *br)
{
	struct net_bridge_port *p;

	spin_lock_bh(&br->lock);
	mod_timer(&br->hello_timer, jiffies + br->hello_time);
	mod_timer(&br->gc_timer, jiffies + HZ * 4);
	mod_timer(&br->mcast_timer, jiffies + br->mcast_advertise_time);

	br_config_bpdu_generation(br);

	/* bring up STP-aware ports */
	list_for_each_entry(p, &br->port_list, list) {

		int bUp      = (p->dev->flags & IFF_UP) ? 1 : 0;
		int bCarrier = netif_carrier_ok(p->dev);

		if (bUp && bCarrier)
			br_stp_enable_port(p);
	}

	/* bring up leaf ports */
	list_for_each_entry(p, &br->leaf_list, list) {
		if (p->dev->flags & IFF_UP)
			p->state = BR_STATE_FORWARDING;
	}

	spin_unlock_bh(&br->lock);
}

/* NO locks held */
void br_stp_disable_bridge(struct net_bridge *br)
{
	struct net_bridge_port *p;

	spin_lock_bh(&br->lock);

	/* STP-aware ports */
	list_for_each_entry(p, &br->port_list, list) {
		if (p->state != BR_STATE_DISABLED)
			br_stp_disable_port(p);

	}

	/* Leaf nodes */
	list_for_each_entry(p, &br->leaf_list, list) {
		if (p->dev->flags & IFF_UP)
			p->state = BR_STATE_DISABLED;
	}

	br_mcast_destroy_list(br);

	br->topology_change = 0;
	br->topology_change_detected = 0;
	spin_unlock_bh(&br->lock);

	del_timer_sync(&br->hello_timer);
	del_timer_sync(&br->topology_change_timer);
	del_timer_sync(&br->tcn_timer);
	del_timer_sync(&br->gc_timer);
	del_timer_sync(&br->mcast_timer);
}

/* called under bridge lock */
void br_stp_enable_port(struct net_bridge_port *p)
{
	br_init_port(p);
	br_port_state_selection(p->br);
	br_log_state(p);
}

/* called under bridge lock */
void br_stp_disable_port(struct net_bridge_port *p)
{
	struct net_bridge *br = p->br;
	int wasroot;

	printk(KERN_INFO "%s: port %i(%s) entering %s state\n",
	       br->dev->name, p->port_no, p->dev->name, "disabled");

	wasroot = br_is_root_bridge(br);
	br_become_designated_port(p);
	p->state = BR_STATE_DISABLED;
	p->topology_change_ack = 0;
	p->config_pending = 0;

	br_log_state(p);

	del_timer(&p->message_age_timer);
	del_timer(&p->forward_delay_timer);
	del_timer(&p->hold_timer);

	br_configuration_update(br);

	br_port_state_selection(br);

	if (br_is_root_bridge(br) && !wasroot)
		br_become_root_bridge(br);
}

/* called under bridge lock */
void br_stp_change_bridge_id(struct net_bridge *br, const unsigned char *addr)
{
	/* should be aligned on 2 bytes for ether_addr_equal() */
	unsigned short oldaddr_aligned[ETH_ALEN >> 1];
	unsigned char *oldaddr = (unsigned char *)oldaddr_aligned;
	struct net_bridge_port *p;
	int wasroot;

	wasroot = br_is_root_bridge(br);

	memcpy(oldaddr, br->bridge_id.addr, ETH_ALEN);
	memcpy(br->bridge_id.addr, addr, ETH_ALEN);
	memcpy(br->dev->dev_addr, addr, ETH_ALEN);

	list_for_each_entry(p, &br->port_list, list) {
		if (ether_addr_equal(p->designated_bridge.addr, oldaddr))
			memcpy(p->designated_bridge.addr, addr, ETH_ALEN);

		if (ether_addr_equal(p->designated_root.addr, oldaddr))
			memcpy(p->designated_root.addr, addr, ETH_ALEN);
	}

	br_configuration_update(br);
	br_port_state_selection(br);
	if (br_is_root_bridge(br) && !wasroot)
		br_become_root_bridge(br);
}

/* should be aligned on 2 bytes for ether_addr_equal() */
static const unsigned short br_mac_zero_aligned[ETH_ALEN >> 1];

/* called under bridge lock */
bool br_stp_recalculate_bridge_id(struct net_bridge *br)
{
	const unsigned char *br_mac_zero =
			(const unsigned char *)br_mac_zero_aligned;
	const unsigned char *addr = br_mac_zero;
	struct net_bridge_port *p;

	/* SONOS: Allow setting of bridge MAC without funky heuristics below.
	 *        We don't always have a valid port in the bridge to use for
	 *        the MAC, sadly.
	 */
	if (br->use_static_mac) {

		if (compare_ether_addr(br->bridge_id.addr, br->static_mac)) {
			br_stp_change_bridge_id(br, br->static_mac);
		}
		return true;
	}
	list_for_each_entry(p, &br->port_list, list) {
		/* prefer first added wired ethernet interface for
		   bridge ID */
		if (addr == br_mac_zero ||
		    0 == strncmp(p->dev->name, "eth", 3))
			addr = p->dev->dev_addr;

	}

	if (ether_addr_equal(br->bridge_id.addr, addr))
		return false;	/* no change */

	br_stp_change_bridge_id(br, addr);
	return true;
}

/* called under bridge lock */
void br_stp_set_bridge_priority(struct net_bridge *br, u16 newprio)
{
	struct net_bridge_port *p;
	int wasroot;

	wasroot = br_is_root_bridge(br);

	list_for_each_entry(p, &br->port_list, list) {
		if (p->state != BR_STATE_DISABLED &&
		    br_is_designated_port(p)) {
			p->designated_bridge.prio[0] = (newprio >> 8) & 0xFF;
			p->designated_bridge.prio[1] = newprio & 0xFF;
		}

	}

	br->bridge_id.prio[0] = (newprio >> 8) & 0xFF;
	br->bridge_id.prio[1] = newprio & 0xFF;
	br_configuration_update(br);
	br_port_state_selection(br);
	if (br_is_root_bridge(br) && !wasroot)
		br_become_root_bridge(br);
}

/* called under bridge lock */
int br_stp_set_port_priority(struct net_bridge_port *p, unsigned long newprio)
{
	port_id new_port_id;

	new_port_id = br_make_port_id(newprio, p->port_no);
	if (br_is_designated_port(p))
		p->designated_port = new_port_id;

	p->port_id = new_port_id;
	p->priority = newprio;
	if (!memcmp(&p->br->bridge_id, &p->designated_bridge, 8) &&
	    p->port_id < p->designated_port) {
		br_become_designated_port(p);
		br_port_state_selection(p->br);
	}

	return 0;
}

/* called under bridge lock */
int br_stp_set_path_cost(struct net_bridge_port *p, unsigned long path_cost)
{
	p->path_cost = path_cost;
	br_configuration_update(p->br);
	br_port_state_selection(p->br);
	return 0;
}

ssize_t br_show_bridge_id(char *buf, const struct bridge_id *id)
{
	return sprintf(buf, "%.2x%.2x.%.2x%.2x%.2x%.2x%.2x%.2x\n",
	       id->prio[0], id->prio[1],
	       id->addr[0], id->addr[1], id->addr[2],
	       id->addr[3], id->addr[4], id->addr[5]);
}

/* called under bridge lock */
int br_stp_mod_port_addr(struct net_bridge *br,
                         unsigned char* oldaddr,
                         unsigned char* newaddr)
{
	struct net_bridge_port *p;

	/*
	 * Find the port w/ the old mac address.  This should be unique.
	 *
	 * This could all get entertaining if we also find a p2p port w/ the
	 * new MAC.  If so, I don't think we want to do anything since we're
	 * already going to end up with a topology change.
	 */
	list_for_each_entry(p, &br->port_list, list) {
		if (p->is_p2p && !memcmp(&p->p2p_dest_addr, oldaddr, ETH_ALEN)) {
			printk("br: Moving %d from "
			       "%02x:%02x:%02x:%02x:%02x:%02x "
			       "to %02x:%02x:%02x:%02x:%02x:%02x\n",
			       p->port_no,
			       oldaddr[0], oldaddr[1], oldaddr[2],
			       oldaddr[3], oldaddr[4], oldaddr[5],
			       newaddr[0], newaddr[1], newaddr[2],
			       newaddr[3], newaddr[4], newaddr[5]);
			memcpy(&p->p2p_dest_addr, newaddr, ETH_ALEN);
			return 0;
		}
	}

	return -EINVAL;
}

/* called under bridge lock */
int br_stp_mod_port_dev(struct net_bridge *br,
			unsigned char* oldaddr,
			struct net_device* dev)
{
	struct net_bridge_port *p;

	/*
	 * Find the port w/ the old mac address.  This should be unique.
	 */
	list_for_each_entry(p, &br->port_list, list) {

            if (p->is_p2p && !memcmp(&p->p2p_dest_addr, oldaddr, ETH_ALEN)) {
			struct net_device *old_dev = p->dev;
			struct net_bridge_port_list_node* pl_curr  = p->dev->br_port_list;
			struct net_bridge_port_list_node* pl_trail = NULL;

			if (p->dev == dev)
				return 0;

			dev_hold(dev);
			dev_hold(old_dev);

			printk("br: Moving %d from %s to %s\n",
				   p->port_no,
				   old_dev->name,
				   dev->name);

			/* find the entry in the original dev's list */
			while (pl_curr) {
					if (pl_curr->port == p) {
							break;
					}
					pl_trail = pl_curr;
					pl_curr  = pl_curr->next;
			}

			if (pl_curr) {
					/* Remove from one dev list */
					if (pl_trail) {
							pl_trail->next = pl_curr->next;
					} else {
							old_dev->br_port_list = pl_curr->next;
					}

					/* Add to the other */
					pl_curr->next      = dev->br_port_list;
					dev->br_port_list  = pl_curr;

					/* Update the actual port to point to the dev */
					p->dev = dev;

					/* MCS-1172: _new_p2p_tunnel has
					 * done a dev_hold on the old
					 * device, and _del_p2p_tunnel
					 * will do a dev_put on the new
					 * device if the p2p link is
					 * deleted. To account for this,
					 * we must do a dev_put on the
					 * old_dev and a dev_hold on the
					 * new dev.
					 */
					dev_put(old_dev);
					dev_hold(dev);
			}

			dev_put(old_dev);
			dev_put(dev);

			return 0;
		}
	}

	return -EINVAL;
}
