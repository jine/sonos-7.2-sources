/*
 *	Device event handling
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_notify.c,v 1.1.1.1 2006/12/23 00:48:21 holmgren Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/if_bridge.h>
#include "br_private.h"

static int br_device_event(struct notifier_block *unused, unsigned long event, void *ptr);

struct notifier_block br_device_notifier =
{
	br_device_event,
	NULL,
	0
};

static int br_device_event(struct notifier_block *unused, unsigned long event, void *ptr)
{
	struct net_device *dev;
	struct net_bridge_port_list_node *pl;

	dev = ptr;
	pl = dev->br_port;

	if (pl == NULL)
		return NOTIFY_DONE;

	switch (event)
	{
	case NETDEV_CHANGEADDR:
                /* notify MAC address change for all ports on this interface */
		read_lock(&pl->port->br->lock);
		br_fdb_changeaddr(pl, dev->dev_addr);
		br_stp_recalculate_bridge_id(pl->port->br);
		read_unlock(&pl->port->br->lock);
		break;

	case NETDEV_GOING_DOWN:
		/* extend the protocol to send some kind of notification? */
		break;

	case NETDEV_DOWN:
                /* shut down all ports on this interface */
		if (pl->port->br->dev.flags & IFF_UP) {
                        struct net_bridge_port_list_node* pl_curr = pl;

			read_lock(&pl->port->br->lock);
                        while (pl_curr != NULL) {
                                if (pl_curr->port->is_leaf)
                                    pl_curr->port->state = BR_STATE_DISABLED;
                                else
                                    br_stp_disable_port(pl_curr->port);

                                pl_curr = pl_curr->next;
                        }
			read_unlock(&pl->port->br->lock);
		}
		break;

	case NETDEV_UP:
                /* bring up all ports on this interface */
		if (pl->port->br->dev.flags & IFF_UP) {
                        struct net_bridge_port_list_node* pl_curr = pl;

			read_lock(&pl->port->br->lock);
                        while (pl_curr != NULL) {
                                if (pl_curr->port->is_leaf)
                                    pl_curr->port->state = BR_STATE_FORWARDING;
                                else
                                    br_stp_enable_port(pl_curr->port);

                                pl_curr = pl_curr->next;
                        }
			read_unlock(&pl->port->br->lock);
		}
		break;

	case NETDEV_UNREGISTER:
		br_del_if(pl->port->br, dev);
		break;
	}

	return NOTIFY_DONE;
}
