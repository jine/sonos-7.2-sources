/*
 *	Userspace interface
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
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/netpoll.h>
#include <linux/ethtool.h>
#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>
#include <linux/if_ether.h>
#include <linux/slab.h>
#include <net/sock.h>
#include <linux/if_vlan.h>

#include "br_private.h"

static int br_initial_port_cost(struct net_device *dev)
{
	/*
	 * Sonos only supports eth/ath, and occasionally wlan.  "Silly"
	 * heuristics are fine for us, and will continue to be until we support
	 * links that are much faster. :-)
	 */
	if (!strncmp(dev->name, "eth", 3))
		return 10;

	if (!strncmp(dev->name, "wlan", 4) || !strncmp(dev->name, "ath", 3))
		return 150;

	return 100;
}

static void release_nbp(struct kobject *kobj)
{
	struct net_bridge_port *p
		= container_of(kobj, struct net_bridge_port, kobj);
	kfree(p);
}

static struct kobj_type brport_ktype = {
#ifdef CONFIG_SYSFS
	.sysfs_ops = &brport_sysfs_ops,
#endif
	.release = release_nbp,
};

static void destroy_nbp(struct net_bridge_port *p)
{
	struct net_device *dev = p->dev;

	p->br = NULL;
	p->dev = NULL;
	dev_put(dev);

	/* SONOS: We currently only add a kobject for non-p2p ports, so just
	 *        free the net_bridge_port if the port is p2p.
	 *
	 *        Yes, we eventually need to fix the bridge so that all objects
	 *        are managed the same way.
	 */
        if (p->is_p2p) {
		kfree(p);
	} else {
		kobject_put(&p->kobj);
        }
}

static void destroy_nbp_rcu(struct rcu_head *head)
{
	struct net_bridge_port *p =
			container_of(head, struct net_bridge_port, rcu);
	destroy_nbp(p);
}

/* Delete port(interface) from bridge is done in two steps.
 * via RCU. First step, marks device as down. That deletes
 * all the timers and stops new packets from flowing through.
 *
 * Final cleanup doesn't occur until after all CPU's finished
 * processing packets.
 *
 * Protected from multiple admin operations by RTNL mutex
 */
static void del_nbp(struct net_bridge_port *p)
{
	struct net_bridge *br = p->br;
	struct net_device *dev = p->dev;

	struct net_bridge_port_list_node *pl;
	struct net_bridge_port_list_node *pl_curr;

	sysfs_remove_link(br->ifobj, p->dev->name);

	/* If we're not already on a bridge, punt */
	if ((pl = dev->br_port_list) == NULL)
		return;

	/*
	 * None of the paths in here grab br->lock, and this is what protects
	 * dev->br_port_list.  I don't think holding it for the entire function
	 * is going to cause any undue pain.
	 */
	spin_lock_bh(&br->lock);

	/*
         * Disable all ports hanging off of this interface.  Leaf ports just
         * get marked disabled, all others need to be taken out the stp
         * insanity.
         */
	pl_curr = pl;
	while (pl_curr != NULL) {
		if (pl_curr->port->is_leaf)
			pl_curr->port->state = BR_STATE_DISABLED;
		else {
			br_stp_disable_port(pl_curr->port);
			br_ifinfo_notify(RTM_DELLINK, pl_curr->port);
		}
		pl_curr = pl_curr->next;
	}

	/*
	 * if the bridge port is not a point-to-point tunnel, take it out
	 * of promiscuous mode.
	 */
        if (!pl->port->is_p2p)
                dev_set_promiscuity(dev, -1);

	/*
	 * Remove the pointer to the bridge from the device.  We have a copy of
	 * this in 'pl', and we'll free it later.
	 */
	dev->br_port_list = NULL;

	/*
	 * Walk the ports again, removing them from the list that the bridge
	 * maintains.  Leaf nodes are in br->leaf_list, everything else is on
	 * br->port_list.  This doesn't matter, though, since the lists are
	 * doubly-linked and list_del_rcu() deletes the entry from whatever
	 * list it is on.
	 */
	for (pl_curr = pl; pl_curr; pl_curr = pl_curr->next) {

		/* Grabs br->hash_lock */
                br_fdb_delete_by_port(br, pl_curr->port);  /* hash_lock */

		/* Grabs br->mcast_lock */
		br_mcast_delete_by_port(br, pl_curr->port);

		/* Remove from list (free comes later) */
		list_del_rcu(&pl_curr->port->list);
	}


	/*
	 * Walk one last time, freeing the bridge_port_list_node.  Could roll
	 * this into the previous loop, I suppose, but it is a wee bit
	 * awkward.
	 */
        while ((pl_curr = pl) != NULL) {
                pl = pl->next;
                kfree(pl_curr);
        }

	/*
	 * Release br->lock
	 */
	spin_unlock_bh(&br->lock);

	/*
	 * Kill timers. We must release the bridge lock beforehand so
	 * that a still-running timer handler on another CPU can get the
	 * lock.
	 *
	 * REVIEW: I think this is already handled in
	 *         br_stp_disable_port(pl_curr->port)
	 */
	del_timer_sync(&p->message_age_timer);
	del_timer_sync(&p->forward_delay_timer);
	del_timer_sync(&p->hold_timer);

	/* Free the port */
	if (!p->is_p2p) {
		kobject_uevent(&p->kobj, KOBJ_REMOVE);
		kobject_del(&p->kobj);
	}

	call_rcu(&p->rcu, destroy_nbp_rcu);
}

/* called with RTNL */
static void del_br(struct net_bridge *br)
{
	struct net_bridge_port *p, *n;

	list_for_each_entry_safe(p, n, &br->port_list, list) {
		del_nbp(p);
	}

	list_for_each_entry_safe(p, n, &br->leaf_list, list) {
		del_nbp(p);
	}

	del_timer_sync(&br->gc_timer);
	del_timer_sync(&br->mcast_timer);

	br_sysfs_delbr(br->dev);
	unregister_netdevice(br->dev);
}

static struct net_device *new_bridge_dev(const char *name)
{
	struct net_bridge *br;
	struct net_device *dev;

	dev = alloc_netdev(sizeof(struct net_bridge), name,
			   br_dev_setup);

	if (!dev)
		return NULL;

	br = netdev_priv(dev);
	br->dev = dev;

	spin_lock_init(&br->lock);
	INIT_LIST_HEAD(&br->port_list);
	INIT_LIST_HEAD(&br->leaf_list);
	spin_lock_init(&br->hash_lock);
	spin_lock_init(&br->mcast_lock);

	br->bridge_id.prio[0] = 0x80;
	br->bridge_id.prio[1] = 0x00;
	memset(br->bridge_id.addr, 0, ETH_ALEN);

    /* SONOS: All STP all the time... */
	br->stp_enabled = 1;

	br->designated_root = br->bridge_id;
	br->root_path_cost = 0;
	br->root_port = 0;
	br->bridge_max_age = br->max_age = 20 * HZ;
	br->bridge_hello_time = br->hello_time = 2 * HZ;
	br->bridge_forward_delay = br->forward_delay = 3 * HZ;
	br->topology_change = 0;
	br->topology_change_detected = 0;
	br->num_mcast_groups = 0;
	br->mcast_advertise_time = 10 * HZ;
	br->ageing_time = 60 * HZ;
	br->mcast_ageing_time = 60 * HZ;
	INIT_LIST_HEAD(&br->age_list);

	br_stp_timer_init(br);
	br_stats_init(br);

	return dev;
}

/* find an available port number */
static int find_portno(struct net_bridge *br)
{
	int index;
	struct net_bridge_port *p;
	unsigned long *inuse;

	inuse = kcalloc(BITS_TO_LONGS(BR_MAX_PORTS), sizeof(unsigned long),
			GFP_KERNEL);
	if (!inuse)
		return -ENOMEM;

	memset(inuse, 0, BITS_TO_LONGS(BR_MAX_PORTS)*sizeof(unsigned long));
	set_bit(0, inuse);	/* zero is reserved */
	list_for_each_entry(p, &br->port_list, list) {
		set_bit(p->port_no, inuse);
	}
	index = find_first_zero_bit(inuse, BR_MAX_PORTS);
	kfree(inuse);

	return (index >= BR_MAX_PORTS) ? -EXFULL : index;
}

/* called with RTNL but without bridge lock */
static struct net_bridge_port *new_nbp(struct net_bridge *br,
				       struct net_device *dev,
				       unsigned long cost)
{
	int index;
    int err;
	struct net_bridge_port *p;
	struct net_bridge_port_list_node *pl;

	index = find_portno(br);
	if (index < 0)
		return ERR_PTR(index);

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL)
		return ERR_PTR(-ENOMEM);

	pl = kzalloc(sizeof(*pl), GFP_KERNEL);
	if (pl == NULL) {
		kfree(p);
		return 0;
	} else {
		pl->port = p;
		pl->next = 0;
	}

	p->br = br;
	dev_hold(dev);
	p->dev = dev;
	p->path_cost = cost;
	p->priority = 0x8000 >> BR_PORT_BITS;
	dev->br_port_list = pl;
	p->port_no = index;
	br_init_port(p);
	p->state = BR_STATE_DISABLED;
	br_stp_port_timer_init(p);

    err = kobject_init_and_add(&p->kobj, &brport_ktype, &(dev->dev.kobj),
                       SYSFS_BRIDGE_PORT_ATTR);
    if (err) {
        printk("bridge: failed to add kernel obj\n") ;
    }

	return p;
}

int br_add_bridge(struct net *net, const char *name)
{
	struct net_device *dev;
	int ret;

	dev = new_bridge_dev(name);
	if (!dev)
		return -ENOMEM;

	rtnl_lock();
	if (strchr(dev->name, '%')) {
		ret = dev_alloc_name(dev, dev->name);
		if (ret < 0)
			goto err1;
	}

	ret = register_netdevice(dev);
	if (ret)
		goto err2;

	/* network device kobject is not setup until
	 * after rtnl_unlock does it's hotplug magic.
	 * so hold reference to avoid race.
	 */
	dev_hold(dev);
	rtnl_unlock();

	ret = br_sysfs_addbr(dev);
	dev_put(dev);

	if (ret)
		unregister_netdev(dev);
 out:
	return ret;

 err2:
	free_netdev(dev);
 err1:
	rtnl_unlock();
	goto out;
}

int br_del_bridge(struct net *net, const char *name)
{
	struct net_device *dev;
	int ret = 0;

	rtnl_lock();
	dev = __dev_get_by_name(net, name);
	if (dev == NULL)
		ret =  -ENXIO; 	/* Could not find device */

	else if (!(dev->priv_flags & IFF_EBRIDGE)) {
		/* Attempt to delete non bridge device! */
		ret = -EPERM;
	}

	else if (dev->flags & IFF_UP) {
		/* Not shutdown yet. */
		ret = -EBUSY;
	}

	else
		del_br(netdev_priv(dev));

	rtnl_unlock();
	return ret;
}

/* MTU of the bridge pseudo-device: ETH_DATA_LEN or the minimum of the ports */
int br_min_mtu(const struct net_bridge *br)
{
	const struct net_bridge_port *p;
	int mtu = 0;

	ASSERT_RTNL();

	if (list_empty(&br->port_list))
		mtu = 1500;
	else {
		list_for_each_entry(p, &br->port_list, list) {
			if (!mtu  || p->dev->mtu < mtu)
				mtu = p->dev->mtu;
		}
	}
	return mtu;
}

/*
 * Recomputes features using slave's features
 */
void br_features_recompute(struct net_bridge *br)
{
	struct net_bridge_port *p;
	unsigned long features, checksum;

	features = NETIF_F_SG | NETIF_F_FRAGLIST
		| NETIF_F_HIGHDMA | NETIF_F_TSO;
	checksum = NETIF_F_IP_CSUM;	/* least commmon subset */

	list_for_each_entry(p, &br->port_list, list) {
		if (!(p->dev->features
		      & (NETIF_F_IP_CSUM|NETIF_F_HW_CSUM)))
			checksum = 0;
		features &= p->dev->features;
	}

	br->dev->features = features | checksum | NETIF_F_LLTX;
}

/* called with RTNL */
int br_add_if(struct net_bridge *br, struct net_device *dev)
{
	struct net_bridge_port *p;
	int err = 0;
	if (dev->br_port_list != NULL)
		return -EBUSY;

	if (dev->flags & IFF_LOOPBACK || dev->type != ARPHRD_ETHER)
		return -EINVAL;

	if (dev->netdev_ops->ndo_start_xmit == br_dev_xmit)
		return -ELOOP;

	if (IS_ERR(p = new_nbp(br, dev, br_initial_port_cost(dev))))
		return PTR_ERR(p);

	if ((err = br_fdb_insert(br, p, dev->dev_addr)))
		destroy_nbp(p);

	else if ((err = br_sysfs_addif(p)))
		del_nbp(p);
	else {
		dev_set_promiscuity(dev, 1);

		list_add_rcu(&p->list, &br->port_list);

		spin_lock_bh(&br->lock);
		br_stp_recalculate_bridge_id(br);
		br_features_recompute(br);
		if ((br->dev->flags & IFF_UP)
		    && (dev->flags & IFF_UP) && netif_carrier_ok(dev)) {
			br_stp_enable_port(p);
			printk("br: new port w/ carrier: %s\n",
			       &(p->dev->name[0]));
		}
		spin_unlock_bh(&br->lock);

		br_ifinfo_notify(RTM_NEWLINK, p);

		kobject_uevent(&p->kobj, KOBJ_ADD);

		dev_set_mtu(br->dev, br_min_mtu(br));
	}
	return err;
}

/* called with RTNL */
int br_del_if(struct net_bridge *br, struct net_device *dev)
{
	struct net_bridge_port *p;
	if (NULL == dev->br_port_list || NULL == dev->br_port_list->port)
		return -EINVAL;

	p = dev->br_port_list->port;

	del_nbp(p);
	spin_lock_bh(&br->lock);
	br_stp_recalculate_bridge_id(br);
	br_features_recompute(br);
	spin_unlock_bh(&br->lock);
	return 0;
}

void br_set_static_mac(struct net_bridge *br, unsigned char *mac)
{
	int i, use_static_mac = 0;

	for (i = 0; i < ETH_ALEN; i++) {
		if (mac[i] != 0) {
			use_static_mac = 1;
			break;
		}
	}

	spin_lock_bh(&br->lock);
	br->use_static_mac = use_static_mac;
	memcpy(&br->static_mac[0], mac, ETH_ALEN);
	br_stp_recalculate_bridge_id(br);
	spin_unlock_bh(&br->lock);
}

void __net_exit br_net_exit(struct net *net)
{
	struct net_device *dev;

	rtnl_lock();
restart:
	for_each_netdev(net, dev) {
		if (dev->priv_flags & IFF_EBRIDGE) {
			del_br(netdev_priv(dev));
			goto restart ;
		}
	}
	rtnl_unlock();

}
