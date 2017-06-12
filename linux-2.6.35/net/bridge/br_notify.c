/*
 *	Device event handling
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br_notify.c#1 $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/rtnetlink.h>
#include <linux/inetdevice.h>

#include "br_private.h"

static int br_device_event(struct notifier_block *unused, unsigned long event, void *ptr);

struct notifier_block br_device_notifier = {
	.notifier_call = br_device_event
};

/*
 * Handle changes in state of network devices enslaved to a bridge.
 * 
 * Note: don't care about up/down if bridge itself is down, because
 *     port state is checked when bridge is brought up.
 */
static int br_device_event(struct notifier_block *unused, unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	struct net_bridge_port_list_node *pl = dev->br_port_list;
        struct net_bridge *br;

	if (NULL == pl) {
		return NOTIFY_DONE;
	}	

	spin_lock_bh(&pl->port->br->lock);
        br  = pl->port->br;
	
	switch (event) {
	case NETDEV_CHANGEMTU:
		dev_set_mtu(br->dev, br_min_mtu(br));
		break;

	case NETDEV_CHANGEADDR:
		br_fdb_changeaddr(pl, dev->dev_addr);
		br_stp_recalculate_bridge_id(br);
		break;
		
	case NETDEV_CHANGE:	/* device is up but carrier changed */
		{
		struct net_bridge_port_list_node* pl_curr = pl;
		int ok = netif_carrier_ok(dev);

		if (!(br->dev->flags & IFF_UP))
			break;
		
		while (pl_curr != NULL) {
			
			struct net_bridge_port *p = pl_curr->port;
			
			/* No STP for leaf nodes, so no need to toggle */
			if (! p->is_leaf) {
				if (ok) {
					if (p->state == BR_STATE_DISABLED)
						br_stp_enable_port(p);
				} else {
					if (p->state != BR_STATE_DISABLED)
						br_stp_disable_port(p);
				}
				
			}			
			pl_curr = pl_curr->next;
		}
		
		break;
		}

	case NETDEV_FEAT_CHANGE:
		if (br->dev->flags & IFF_UP) {
			br_features_recompute(pl->port->br);
		}
		/* could do recursive feature change notification
		 * but who would care?? 
		 */
		break;

	case NETDEV_DOWN:
		/* shut down all ports on this interface */
		if (pl->port->br->dev->flags & IFF_UP) {
                        struct net_bridge_port_list_node* pl_curr = pl;

                        while (pl_curr != NULL) {
                                if (pl_curr->port->is_leaf)
                                    pl_curr->port->state = BR_STATE_DISABLED;
                                else
                                    br_stp_disable_port(pl_curr->port);

                                pl_curr = pl_curr->next;
                        }
		}
		break;

	case NETDEV_UP:
		/* bring up all ports on this interface */
		if (pl->port->br->dev->flags & IFF_UP) {
                        struct net_bridge_port_list_node* pl_curr = pl;

                        while (pl_curr != NULL) {
                                if (pl_curr->port->is_leaf)
                                    pl_curr->port->state = BR_STATE_FORWARDING;
                                else
                                    br_stp_enable_port(pl_curr->port);

                                pl_curr = pl_curr->next;
                        }
		}
		break;

	case NETDEV_UNREGISTER:
		spin_unlock_bh(&pl->port->br->lock);
		br_del_if(br, dev);
		goto done;
		break;
	} 

	spin_unlock_bh(&pl->port->br->lock);

    /* Events that may cause spanning tree to refresh */
    if (event == NETDEV_CHANGEADDR || event == NETDEV_UP 
        || event == NETDEV_CHANGE || event == NETDEV_DOWN)
        br_ifinfo_notify(RTM_NEWLINK, pl->port);

 done:
	return NOTIFY_DONE;
}

#ifdef CONFIG_SONOS_BRIDGE_PROXY
static int br_inetaddr_event(struct notifier_block *unused, unsigned long event, void *ptr);

struct notifier_block br_inetaddr_notifier = {
	.notifier_call = br_inetaddr_event,
};

static int br_inetaddr_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = ptr;
	struct net_device *event_dev = ifa->ifa_dev->dev;
	struct net_device *dev;

	for_each_netdev(&init_net, dev) {
		if (event_dev == dev && (dev->priv_flags & IFF_EBRIDGE)) {
			struct net_bridge *br = netdev_priv(dev);

			switch (event) {
			case NETDEV_UP:
				br->current_ipv4_addr = ifa->ifa_local;
				br_dupip_check(br);
				return NOTIFY_OK;
			case NETDEV_DOWN:
				br->current_ipv4_addr = 0;
				br_dupip_check(br);
				return NOTIFY_OK;
			default:
				return NOTIFY_DONE;
			}
		}
	}
        return NOTIFY_DONE;
}
#endif
