/*
 *	Point to Point Packet Tunnel
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *      Nick Millington                 <nickmillington@msn.com>
 *
 *	$Id: br_tunnel.c,v 1.1.1.1 2006/12/23 00:48:21 holmgren Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/if_arp.h>
#include <linux/if_bridge.h>
#include <linux/brlock.h>
#include "br_private.h"

/* Call holding dev and br_lock */
static struct net_bridge_port_list_node*
_find_p2p_tunnel(struct net_bridge_port_list_node *pl,
		 unsigned char *daddr)
{
	while (pl) {
                if (pl->port->is_p2p && !pl->port->is_leaf) {
                        if (0 == memcmp(pl->port->p2p_dest_addr, 
                                        daddr, ETH_ALEN))
                            break;
                }

                pl = pl->next;
        }
	
	return pl;
}

int br_add_p2p_tunnel(struct net_bridge *br,
                      struct net_device *dev,
                      unsigned char* daddr,
                      int path_cost)
{
	int i;
	struct net_bridge_port *p;
        struct net_bridge_port_list_node *pl_new;

	if (dev->flags & IFF_LOOPBACK || dev->type != ARPHRD_ETHER)
		return -EINVAL;

	if (dev->hard_start_xmit == br_dev_xmit)
		return -ELOOP;

        dev_hold(dev);
        write_lock_bh(&br->lock);

        /* ensure that this interface isn't already a promiscuous bridge,
           and does not already have a tunnel to the specified MAC */
        struct net_bridge_port_list_node* pl_curr = dev->br_port;
        while (pl_curr) {
                if (!pl_curr->port->is_p2p)
                        break;
                if (0 == memcmp(pl_curr->port->p2p_dest_addr, 
                                daddr, ETH_ALEN))
                        break;
                pl_curr = pl_curr->next;
        }

        if (pl_curr) {
                /* pl_curr is the entry that caused new one to be invalid */
                write_unlock_bh(&br->lock);
		dev_put(dev);
		return -EBUSY;
        }

        if ((pl_new = kmalloc(sizeof(*pl_new), GFP_ATOMIC)) == NULL) {
                write_unlock_bh(&br->lock);
		dev_put(dev);
		return -EXFULL;
        }

        p = kmalloc(sizeof(*p), GFP_ATOMIC);
        if (p == NULL) {
                write_unlock_bh(&br->lock);
		dev_put(dev);
                kfree(pl_new);
		return -EXFULL;                
        }

	memset(p, 0, sizeof(*p));
	p->br = br;
	p->dev = dev;
	p->path_cost = path_cost;
	p->priority = 0x80;

        p->is_p2p = 1;
        memcpy(p->p2p_dest_addr, daddr, ETH_ALEN);

	for (i = 1; i < 255; i++)
		if (br_get_port(br, i) == NULL)
			break;

	if (i == 255) {
                write_unlock_bh(&br->lock);
		dev_put(dev);
                kfree(pl_new);
                kfree(p);
		return -EXFULL; 
	}

        pl_new->port = p;
        pl_new->next = dev->br_port;
	dev->br_port = pl_new;

	p->port_no = i;
	br_init_port(p);
	p->state = BR_STATE_DISABLED;

        p->next = br->port_list;
        br->port_list = p;

	br_stp_recalculate_bridge_id(br);
	br_fdb_insert(br, p, dev->dev_addr, 1);
	if ((br->dev.flags & IFF_UP) && (dev->flags & IFF_UP))
		br_stp_enable_port(p);
	write_unlock_bh(&br->lock);        

        return 0;
}

int br_set_p2p_tunnel_path_cost(struct net_bridge *br,
                                struct net_device *dev,
                                unsigned char* daddr,
                                int path_cost)
{
	int ret = -EINVAL;

        dev_hold(dev);
        write_lock_bh(&br->lock);

	/* find the specified tunnel */
        struct net_bridge_port_list_node* pl_curr;
	pl_curr = _find_p2p_tunnel(dev->br_port, daddr);

        if (pl_curr) {
		/* update the path cost */
		br_stp_set_path_cost(pl_curr->port, path_cost);
		ret = 0;
	}

        write_unlock_bh(&br->lock);
        dev_put(dev);
        return 0;
}

int br_set_p2p_direct_addr(struct net_bridge *br,
			   struct net_device *dev,
			   unsigned char* daddr,
			   unsigned char* direct_addr)
{
	int ret = -EINVAL;
	
        dev_hold(dev);
        spin_lock_bh(&br->lock);

        /* find the specified tunnel */
        struct net_bridge_port_list_node* pl_curr;
	pl_curr = _find_p2p_tunnel(dev->br_port, daddr);
	
        if (pl_curr) {
		/* update the address */
		memcpy(&pl_curr->port->direct_addr, direct_addr, 6);
		printk(KERN_INFO
		       "%s: port %i(%s) addr=%02x:%02x:%02x:%02x:%02x:%02x\n",
		       br->dev.name,
		       pl_curr->port->port_no, dev->name,
		       direct_addr[0], direct_addr[1], direct_addr[2], 
		       direct_addr[3], direct_addr[4], direct_addr[5]);
		
		ret = 0;

                /* Update FDB entries */
                br_fdb_update_dst_direct(br, pl_curr->port);                
	}
	
	spin_unlock_bh(&br->lock);
        dev_put(dev);
        return ret;
 }
 
int br_set_p2p_direct_enabled(struct net_bridge *br,
			      struct net_device *dev,
			      unsigned char* daddr,
			      int direct_enabled)
{
	int ret = -EINVAL;
	
        dev_hold(dev);
        spin_lock_bh(&br->lock);

        /* find the specified tunnel */
        struct net_bridge_port_list_node* pl_curr;
	pl_curr = _find_p2p_tunnel(dev->br_port, daddr);
	
        if (pl_curr) {
		/* update whether direct is allowed or not */
		pl_curr->port->direct_enabled = direct_enabled;
		printk(KERN_INFO
		       "%s: port %i(%s) direct=%d\n",
		       br->dev.name,
		       pl_curr->port->port_no, dev->name,
		       direct_enabled);		
		ret = 0;
	}
	
	spin_unlock_bh(&br->lock);
        dev_put(dev);
        return ret;
}

int br_set_p2p_tunnel_remote_stp_state(struct net_bridge *br,
                                          struct net_device *dev,
                                          unsigned char* daddr,
                                          int remote_stp_state)
{
	int ret = -EINVAL;

        dev_hold(dev);
        write_lock_bh(&br->lock);

        /* find the specified tunnel */
        struct net_bridge_port_list_node* pl_curr = dev->br_port;
	pl_curr = _find_p2p_tunnel(dev->br_port, daddr);

        if (pl_curr) {
		/* update the remote STP state; will take effect immediately */
		pl_curr->port->remote_state = remote_stp_state;
		ret = 0;
	}

        write_unlock_bh(&br->lock);
        dev_put(dev);
        return 0;
}

/* called under BR_NETPROTO_LOCK and bridge lock */
int __br_del_p2p_tunnel(struct net_bridge *br,
                        struct net_device *dev,
                        unsigned char* daddr)
{
        struct net_bridge_port_list_node *pl_before_curr;
        struct net_bridge_port_list_node *pl_curr;
        struct net_bridge_port **pptr;

        if (dev->br_port == NULL)
		return -EINVAL;

        /* make sure it's really a point-to-point tunnel */
        if (!dev->br_port->port->is_p2p)
                return -EINVAL;

        /* go through the list of ports associated with this interface, and
           see if we can find the point-to-point link that the caller wants
           to remove. */
        pl_before_curr = 0;
        pl_curr = dev->br_port;
        while (pl_curr) {
                if (0 == memcmp(pl_curr->port->p2p_dest_addr, 
                                daddr, ETH_ALEN))
                    break;

                pl_before_curr = pl_curr;
                pl_curr = pl_curr->next;
        }

        /* we couldn't find the one they wanted us to remove */
        if (!pl_curr)
                return -EINVAL;

        if (pl_curr->port->is_leaf)
                pl_curr->port->state = BR_STATE_DISABLED;
        else
                br_stp_disable_port(pl_curr->port);

        /* remove the entry from the device's list of ports */
        if (pl_before_curr)
                pl_before_curr->next = pl_curr->next;
        else
                dev->br_port = dev->br_port->next;

        /* remove from appropriate list maintained by bridge itself */
        if (pl_curr->port->is_leaf)
                pptr = &br->leaf_list;
        else
                pptr = &br->port_list;
        while (*pptr != NULL) {
	    if (*pptr == pl_curr->port) {
		    *pptr = pl_curr->port->next;
		    break;
            }

            pptr = &((*pptr)->next);
        }

        br_fdb_delete_by_port(br, pl_curr->port);
        br_mcast_delete_by_port(br, pl_curr->port);
        kfree(pl_curr->port);

        /* delete the node */
        kfree(pl_curr);

        dev_put(dev);

        return 0;
}

int br_del_p2p_tunnel(struct net_bridge *br,
                      struct net_device *dev,
                      unsigned char* daddr)
{
	int retval;

	br_write_lock_bh(BR_NETPROTO_LOCK);
	write_lock(&br->lock);
	retval = __br_del_p2p_tunnel(br, dev, daddr);
	br_stp_recalculate_bridge_id(br);
	write_unlock(&br->lock);
	br_write_unlock_bh(BR_NETPROTO_LOCK);

	return retval;        
}

/* a p2p leaf is a point to point link that ends in a device that does not
   forward packets; such a device cannot create loops, and therefore does not
   participate in the spanning tree protocol even if STP is enabled for the
   bridge as a whole. */
int br_add_p2p_tunnel_leaf(struct net_bridge *br,
                           struct net_device *dev,
                           unsigned char* daddr,
                           int flags)
{
	struct net_bridge_port *p;
        struct net_bridge_port_list_node *pl_new;

	if (dev->flags & IFF_LOOPBACK || dev->type != ARPHRD_ETHER)
		return -EINVAL;

	if (dev->hard_start_xmit == br_dev_xmit)
		return -ELOOP;

        dev_hold(dev);
        write_lock_bh(&br->lock);

        /* ensure that this interface isn't already a promiscuous bridge,
           and does not already have a tunnel to the specified MAC */
        struct net_bridge_port_list_node* pl_curr = dev->br_port;
        while (pl_curr) {
                if (!pl_curr->port->is_p2p)
                        break;
                if (0 == memcmp(pl_curr->port->p2p_dest_addr, 
                                daddr, ETH_ALEN))
                        break;
                pl_curr = pl_curr->next;
        }

        if (pl_curr) {
                /* pl_curr is the entry that caused new one to be invalid */
                write_unlock_bh(&br->lock);
		dev_put(dev);
		return -EBUSY;
        }

        if ((pl_new = kmalloc(sizeof(*pl_new), GFP_ATOMIC)) == NULL) {
                write_unlock_bh(&br->lock);
		dev_put(dev);
		return -EXFULL;
        }

        p = kmalloc(sizeof(*p), GFP_ATOMIC);
        if (p == NULL) {
                write_unlock_bh(&br->lock);
		dev_put(dev);
                kfree(pl_new);
		return -EXFULL;                
        }

	memset(p, 0, sizeof(*p));
	p->br = br;
	p->dev = dev;
        p->is_p2p = p->is_leaf = 1;
        p->is_unencap = flags & 1 ? 1 : 0;
        p->is_unicast = flags & 2 ? 1 : 0;
        memcpy(p->p2p_dest_addr, daddr, ETH_ALEN);

        /* leaf ports get added to the net device's port list... */
        pl_new->port = p;
        pl_new->next = dev->br_port;
	dev->br_port = pl_new;

        /* ...BUT they get added to a separate list on the bridge to keep
           them from interfering with STP */
        p->next = br->leaf_list;
        br->leaf_list = p;

	br_fdb_insert(br, p, dev->dev_addr, 1);

        /* a leaf port enters directly into the forwarding state */
	if ((br->dev.flags & IFF_UP) && (dev->flags & IFF_UP))
                p->state = BR_STATE_FORWARDING;
        else
                p->state = BR_STATE_DISABLED;

	write_unlock_bh(&br->lock);        

        return 0;
}

int br_get_p2p_tunnel_states(struct net_bridge *br,
                             struct net_device *dev,
                             unsigned int max_records,
                             unsigned char* state_data,
                             unsigned int* state_data_len)
{
        unsigned char* state_data_orig = state_data;

        /* initialize the output buffer */
        memset(state_data, 0, max_records * (ETH_ALEN + 1));

        dev_hold(dev);
        read_lock_bh(&br->lock);

        /* loop over all the point-to-point tunnels */
        struct net_bridge_port_list_node* pl_curr = dev->br_port;
        while (pl_curr) {
                if (pl_curr->port->is_p2p && !pl_curr->port->is_leaf) {
                        /* add a record for this port */
                        if (max_records > 0) {
                            memcpy(state_data, 
                                   pl_curr->port->p2p_dest_addr, 
                                   ETH_ALEN);
                            state_data += ETH_ALEN;
                            *(state_data++) = 
                                (unsigned char)pl_curr->port->state;

                            max_records--;
                        }
                }

                /* advance to the next port */
                pl_curr = pl_curr->next;
        }

        read_unlock_bh(&br->lock);
        dev_put(dev);

        *state_data_len = (state_data - state_data_orig);

        return 0;
}
