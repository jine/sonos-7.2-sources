/*
 *	Point to Point Packet Tunnel
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *      Nick Millington                 <nickmillington@msn.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/if_arp.h>
#include <linux/if_bridge.h>
#include <linux/rculist.h>

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

/*
 * Allocate a new port.  This returns non-zero for failure and 0 for success.
 * Nothing is held on failure, but dev_hold(dev) has been called and br_lock()
 * is held on success.  Ugly, but this is why it is static.
 */
static int _new_p2p_tunnel(struct net_bridge *br,
                           struct net_device *dev, 
                           unsigned char* daddr,
			   int get_port_number,
                           struct net_bridge_port **p_output,
                           struct net_bridge_port_list_node **pl_new_output)
{
	int ret = 0, port_no = 0;
	struct net_bridge_port *p;
	struct net_bridge_port_list_node *pl_curr;
	struct net_bridge_port_list_node *pl_new = NULL;

	*p_output = NULL;
	*pl_new_output = NULL;

	if (dev->flags & IFF_LOOPBACK || dev->type != ARPHRD_ETHER)
		return -EINVAL;
	
	if (dev->netdev_ops->ndo_start_xmit == br_dev_xmit)
		return -ELOOP;

        dev_hold(dev);
        spin_lock_bh(&br->lock);

        /* ensure that this interface isn't already a promiscuous bridge,
           and does not already have a tunnel to the specified MAC */
        pl_curr = dev->br_port_list;
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
		ret = -EBUSY;
		goto failure;
        }

	/* find a new port number */
	if (get_port_number) {
		for (port_no = 1; port_no < 255; port_no++)
			if (br_get_port(br, port_no) == NULL)
				break;
		
		if (port_no == 255) {
			ret = -EXFULL; 
			goto failure;
		}
	}
	
	/* allocate the port */
        if ((pl_new = kmalloc(sizeof(*pl_new), GFP_ATOMIC)) == NULL) {
		ret = -EXFULL;
		goto failure;
        }

        p = kmalloc(sizeof(*p), GFP_ATOMIC);
        if (p == NULL) {
		ret = -EXFULL;
		goto failure;
        }

	/* set up the common stuff in the port */
	memset(p, 0, sizeof(*p));
	p->br = br;
	p->dev = dev;
	p->is_p2p = 1;
	p->port_no = port_no;
	memcpy(p->p2p_dest_addr, daddr, ETH_ALEN);
	// SWPBL-55167: Init timers on every net_bridge_port, so that
	// unconditionally deleting them later makes sense.
	br_stp_port_timer_init(p);

	/* set the output parameters */
 	*p_output      = p;
	*pl_new_output = pl_new;

	return 0;

failure:
	
	spin_unlock_bh(&br->lock);
	dev_put(dev);

	if (pl_new) {
		kfree(pl_new);
	}
	
	return ret;
}
                           

                           
int br_add_uplink(struct net_bridge *br,
		  struct net_device *dev,
		  unsigned char* daddr)
{
	int ret;
	struct net_bridge_port *p;
        struct net_bridge_port_list_node *pl_new;

	/*
	 * Grab a new port.  If this succeeds, we're holding dev via dev_hold()
	 * and br_lock().
	 */
	ret = _new_p2p_tunnel(br, dev, daddr, 1, &p, &pl_new);
	if (0 != ret) {
		return ret;
	}

	/* Set up the port */
	p->is_uplink = 1;
	p->is_unencap = 1;
	memcpy(p->p2p_dest_addr, daddr, ETH_ALEN);
	
	pl_new->port = p;
	pl_new->next = dev->br_port_list;
	dev->br_port_list = pl_new;

	p->state = BR_STATE_DISABLED;

	list_add(&p->list, &br->port_list);

	br_fdb_insert(br, p, dev->dev_addr);

	spin_unlock_bh(&br->lock);        

        return 0;
}
	
int br_add_p2p_tunnel(struct net_bridge *br,
                      struct net_device *dev,
                      unsigned char* daddr,
                      struct __add_p2p_entry *ape)
{
	int ret;
	struct net_bridge_port *p;
        struct net_bridge_port_list_node *pl_new;

	/*
	 * Grab a new port.  If this succeeds, we're holding dev via dev_hold()
	 * and br_lock().
	 */
	ret = _new_p2p_tunnel(br, dev, daddr, 1, &p, &pl_new);
	if (0 != ret) {
		return ret;
	}

	/* Set up the port */
	p->path_cost = ape->stp_weight;
	p->priority = 0x80;

        pl_new->port = p;
        pl_new->next = dev->br_port_list;
	dev->br_port_list = pl_new;

	br_init_port(p);
	p->state = BR_STATE_DISABLED;
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	p->is_satellite = ape->is_satellite;
#endif

        list_add(&p->list, &br->port_list);

	br_stp_recalculate_bridge_id(br);
        br_fdb_insert(br, p, dev->dev_addr);
        
	if ((br->dev->flags & IFF_UP) && (dev->flags & IFF_UP)) {
		if (br->stp_enabled)
			br_stp_enable_port(p);		
	}
	spin_unlock_bh(&br->lock);        

        return 0;
}

int br_set_p2p_tunnel_path_cost(struct net_bridge *br,
                                struct net_device *dev,
                                unsigned char* daddr,
                                int path_cost)
{
        struct net_bridge_port_list_node *pl_curr;

        dev_hold(dev);
        spin_lock_bh(&br->lock);

        /* find the specified tunnel */
        pl_curr = dev->br_port_list;
        while (pl_curr) {
                if (pl_curr->port->is_p2p && !pl_curr->port->is_leaf) {
                        if (0 == memcmp(pl_curr->port->p2p_dest_addr, 
                                        daddr, ETH_ALEN))
                            break;
                }

                pl_curr = pl_curr->next;
        }

        if (!pl_curr) {
                spin_unlock_bh(&br->lock);
                dev_put(dev);
                return -EINVAL;
        }

        /* update the path cost */
        br_stp_set_path_cost(pl_curr->port, path_cost);

        spin_unlock_bh(&br->lock);
        dev_put(dev);
        return 0;
}

int br_set_p2p_direct_addr(struct net_bridge *br,
			   struct net_device *dev,
			   unsigned char* daddr,
			   unsigned char* direct_addr)
{
	int ret = -EINVAL;
        /* find the specified tunnel */
	struct net_bridge_port_list_node* pl_curr;
	
        dev_hold(dev);
        spin_lock_bh(&br->lock);

	pl_curr = _find_p2p_tunnel(dev->br_port_list, daddr);
	
        if (pl_curr) {
		/* update the address */
		memcpy(&pl_curr->port->direct_addr, direct_addr, 6);
		printk(KERN_INFO
		       "%s: port %i(%s) addr=%02x:%02x:%02x:%02x:%02x:%02x\n",
		       br->dev->name,
		       pl_curr->port->port_no, dev->name,
		       direct_addr[0], direct_addr[1], direct_addr[2], 
		       direct_addr[3], direct_addr[4], direct_addr[5]);
		
		ret = 0;

                /* Update FDB entries */
                br_fdb_update_dst_direct(br, pl_curr->port);

                /* Update multicast table entries */
                br_mcast_update_dst_direct(br, pl_curr->port);
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
        struct net_bridge_port_list_node* pl_curr;
	
        dev_hold(dev);
        spin_lock_bh(&br->lock);

        /* find the specified tunnel */
	pl_curr = _find_p2p_tunnel(dev->br_port_list, daddr);
	
        if (pl_curr) {
		/* update whether direct is allowed or not */
		pl_curr->port->direct_enabled = direct_enabled;
		printk(KERN_INFO
		       "%s: port %i(%s) direct=%d\n",
		       br->dev->name,
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
	struct net_bridge_port_list_node* pl_curr;

        dev_hold(dev);
        spin_lock_bh(&br->lock);

        /* find the specified tunnel */
	pl_curr = _find_p2p_tunnel(dev->br_port_list, daddr);

        if (pl_curr) {
		/* update the remote STP state; will take effect immediately */
		pl_curr->port->remote_state = remote_stp_state;
		ret = 0;
	}

        spin_unlock_bh(&br->lock);
        dev_put(dev);
        return ret;
}

static void free_port_rcu(struct rcu_head *head)
{
	struct net_bridge_port *p =
			container_of(head, struct net_bridge_port, rcu);
	kfree(p);
}

/* bridge lock */
int __br_del_p2p_tunnel(struct net_bridge *br,
                        struct net_device *dev,
                        unsigned char* daddr)
{
        struct net_bridge_port_list_node *pl_before_curr;
        struct net_bridge_port_list_node *pl_curr;

        if (dev->br_port_list == NULL)
		return -EINVAL;

        /* make sure it's really a point-to-point tunnel */
	
	/* REVIEW: Shouldn't we find the MAC first, and then check?
	 *
	 * Isn't this exactly wrong if we *do* want to support deleting leaf
	 * nodes with this function and we have no peers (CR100/CR200 w/ a
	 * single ZP)?
	 */
        if (!dev->br_port_list->port->is_p2p) {
				printk("br: del_p2p_tunnel: !is_p2p\n");
                return -EINVAL;
        }

        /* go through the list of ports associated with this interface, and
           see if we can find the point-to-point link that the caller wants
           to remove.

	   NOTE: dev->br_port_list is protected by br->lock (AKA bridge lock),
	         but br->port_list and br->leaf_list are potentially protected
	         solely by RCU trickery (readers can have nothing more than
	         rcu_read_lock). I think these end up being one and the same
	         for SONOS, but I'm still going through the motions of "doing
	         the right thing" for the rcu stuff.
	*/
        pl_before_curr = 0;
        pl_curr = dev->br_port_list;
        while (pl_curr) {
                if (0 == memcmp(pl_curr->port->p2p_dest_addr, 
                                daddr, ETH_ALEN))
                    break;

                pl_before_curr = pl_curr;
                pl_curr = pl_curr->next;
        }

        /* we couldn't find the one they wanted us to remove */
        if (!pl_curr) {
		printk("br: del_p2p_tunnel on non-existant port\n");
                return -EINVAL;
	}

	if (pl_curr->port->is_leaf)
		pl_curr->port->state = BR_STATE_DISABLED;
	else if (pl_curr->port->is_uplink) {
#ifdef CONFIG_SONOS_BRIDGE_PROXY
		struct net_bridge_port *p;
		list_for_each_entry_rcu(p, &br->port_list, list) {
			p->sat_ip = 0;
		}
#endif
	} else
                br_stp_disable_port(pl_curr->port);

        /* remove the entry from the device's list of ports */
        if (pl_before_curr)
                pl_before_curr->next = pl_curr->next;
        else
                dev->br_port_list = dev->br_port_list->next;
        
        /* remove from appropriate list maintained by bridge itself (free
	 * comes later) */
        list_del_rcu(&pl_curr->port->list);

        br_fdb_delete_by_port(br, pl_curr->port);        
        br_mcast_delete_by_port(br, pl_curr->port);

        /* Free the port */
        call_rcu(&pl_curr->port->rcu, free_port_rcu);

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

	spin_lock_bh(&br->lock);
	retval = __br_del_p2p_tunnel(br, dev, daddr);
	br_stp_recalculate_bridge_id(br);
	br_dupip_check(br);
	spin_unlock_bh(&br->lock);

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
	int ret;

	/*
	 * Grab a new port.  If this succeeds, we're holding dev via dev_hold()
	 * and br_lock().
	 */
	ret = _new_p2p_tunnel(br, dev, daddr, 0, &p, &pl_new);
	if (0 != ret) {
		return ret;
	}

	/* Set up the port */	
	p->is_leaf = 1;
	p->is_unencap = flags & BR_LEAF_FLAGS_IS_UNENCAP ? 1 : 0;
	p->is_unicast = flags & BR_LEAF_FLAGS_IS_UNICAST ? 1 : 0;

        /* leaf ports get added to the net device's port list... */
	pl_new->port = p;
	pl_new->next = dev->br_port_list;
	dev->br_port_list = pl_new;

        /* ...BUT they get added to a separate list on the bridge to keep
           them from interfering with STP */
	list_add(&p->list, &br->leaf_list);

	br_fdb_insert(br, p, dev->dev_addr);

        /* a leaf port enters directly into the forwarding state */
	if ((br->dev->flags & IFF_UP) && (dev->flags & IFF_UP))
                p->state = BR_STATE_FORWARDING;
        else
                p->state = BR_STATE_DISABLED;

	spin_unlock_bh(&br->lock);        

        return 0;	
}

int br_get_p2p_tunnel_states(struct net_bridge *br,
                             struct net_device *dev,
                             unsigned int max_records,
                             unsigned char* state_data,
                             unsigned int* state_data_len)
{
        unsigned char* state_data_orig = state_data;
        struct net_bridge_port_list_node *pl_curr;

        /* initialize the output buffer */
        memset(state_data, 0, max_records * (ETH_ALEN + 1));

        dev_hold(dev);
	
        spin_lock_bh(&br->lock);
		
        /* loop over all the point-to-point tunnels */
        pl_curr = dev->br_port_list;
        while (pl_curr) {
                if (pl_curr->port->is_p2p && 
                    !pl_curr->port->is_leaf &&
                    !pl_curr->port->is_uplink) {
                        /* add a record for this port */
                        if (max_records > 0) {
                            memcpy(state_data, 
                                   pl_curr->port->p2p_dest_addr, 
                                   ETH_ALEN);
                            state_data += ETH_ALEN;

                            if (br->uplink_mode
#ifdef CONFIG_SONOS_BRIDGE_PROXY
				&& !br->proxy_mode
#endif
				) {
                                /* SONOS:
                                 *
                                 * uplink is not running STP, but our peers
                                 * treat DISABLED as UNKNOWN and forward
                                 * multicast traffic to us.  This means we
                                 * get a dup of every packet from every peer
                                 * running SonosNet (only during upgrade,
                                 * but still irritating). However,
                                 * proxied satellites need the remote STP
                                 * state of their peer link to the proxy
                                 * to not be BLOCKING so they'll transmit
                                 * frames.
                                 */
                                *(state_data++) = BR_STATE_BLOCKING;
                            } else {
                                *(state_data++) = 
                                    (unsigned char)pl_curr->port->state;
                            }

                            max_records--;
                        }
                }

                /* advance to the next port */
                pl_curr = pl_curr->next;
        }

        spin_unlock_bh(&br->lock);
        dev_put(dev);

        *state_data_len = (state_data - state_data_orig);

        return 0;
}
