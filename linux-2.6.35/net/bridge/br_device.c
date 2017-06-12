/*
 *	Device handling code
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br_device.c#1 $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/rculist.h>
#include <asm/uaccess.h>
#include "br_private.h"

static struct net_device_stats *br_dev_get_stats(struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	return &br->statistics;
}

netdev_tx_t br_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	const unsigned char *dest = skb->data;
	struct net_bridge_fdb_entry *dst;

	br->statistics.tx_packets++;
	br->statistics.tx_bytes += skb->len;

	skb->mac_header = skb->data;
	skb_pull(skb, ETH_HLEN);

	rcu_read_lock();

	if (br->uplink_mode) {

		/* Uplink */
		br_uplink_xmit(br, skb, dest);

	} else if (dest[0] & 1) {
		
		/* Multicast/broadcast */
 		br_flood_deliver(br, 0, skb, 0);
		
	} else {

		/* Unicast */
		dst = __br_fdb_get(br, dest);

		if (dst) {
			/* Known address */			
			if (0 == skb->priority) {
				skb->priority = dst->priority;
			}

			br_direct_unicast(0, dst, skb,
                                          br_deliver,
                                          br_deliver_direct);

		} else {			
			/* Unknown address */			
			br_flood_deliver(br, 0, skb, 0);
		}
	}
        
        rcu_read_unlock();
        
	return NETDEV_TX_OK;
}

static int br_dev_open(struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);

	br_features_recompute(br);
	netif_start_queue(dev);
	br_stp_enable_bridge(br);

	return 0;
}

static void br_dev_set_multicast_list(struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	struct netdev_hw_addr *ha;

        spin_lock_bh(&br->lock);

        /* the netdev has the updated list of MAC multicast groups of which
           the device has been configured to be a member. */
        br->num_mcast_groups = 0;

	netdev_for_each_mc_addr(ha, dev) {
		/* keep certain 'system' multicast MAC addresses out of the list */
		if (0 == memcmp(igmp_ah_addr,    ha->addr, 6) ||
		    0 == memcmp(igmp_ar_addr,    ha->addr, 6) ||
		    0 == memcmp(igmp_amr_addr,   ha->addr, 6) ||
		    0 == memcmp(broadcast_addr,  ha->addr, 6) ||
		    0 == memcmp(rincon_gmp_addr, ha->addr, 6) ||
		    0 == memcmp(mdns_addr,       ha->addr, 6) ||
		    0 == memcmp(upnp_addr,       ha->addr, 6))
			continue;

		memcpy(br->mcast_groups[br->num_mcast_groups], ha->addr, 6);
		if (++(br->num_mcast_groups) >= BR_MAX_MCAST_GROUPS)
			break;
	}

        br_mcast_transmit_grouplist(br);

        spin_unlock_bh(&br->lock);
}

static int br_dev_stop(struct net_device *dev)
{
	br_stp_disable_bridge(netdev_priv(dev));

	netif_stop_queue(dev);

	return 0;
}

static int br_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu < 68 || new_mtu > br_min_mtu(netdev_priv(dev)))
		return -EINVAL;

	dev->mtu = new_mtu;
	return 0;
}

static void br_getinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strcpy(info->driver, "bridge");
	strcpy(info->version, BR_VERSION);
	strcpy(info->fw_version, "N/A");
	strcpy(info->bus_info, "N/A");
}

static int br_set_sg(struct net_device *dev, u32 data)
{
	printk("br: set_sg: %lu\n", (unsigned long)data);
	return 0;
}

static int br_set_tso(struct net_device *dev, u32 data)
{
	printk("br: set_tso: %lu\n", (unsigned long)data);
	return 0;
}

static int br_set_tx_csum(struct net_device *dev, u32 data)
{
	printk("br: set_tx_csum: %lu\n", (unsigned long)data);
	return 0;
}

/* Allow setting mac address to any valid ethernet address. */
static int br_dev_set_mac_address(struct net_device *dev, void *p)
{
        printk("br: set_mac_address");
	return 0;
}

static const struct ethtool_ops br_ethtool_ops = {
	.get_drvinfo    = br_getinfo,
	.get_link	= ethtool_op_get_link,
	.get_tx_csum	= ethtool_op_get_tx_csum,
	.set_tx_csum 	= br_set_tx_csum,
	.get_sg		= ethtool_op_get_sg,
	.set_sg		= br_set_sg,
	.get_tso	= ethtool_op_get_tso,
	.set_tso	= br_set_tso,
	.get_ufo	= ethtool_op_get_ufo,
	.set_ufo	= ethtool_op_set_ufo,
	.get_flags	= ethtool_op_get_flags,
};

static const struct net_device_ops br_netdev_ops = {
	.ndo_open		 = br_dev_open,
	.ndo_stop		 = br_dev_stop,
	.ndo_start_xmit		 = br_dev_xmit,
	.ndo_get_stats		 = br_dev_get_stats,
	.ndo_set_mac_address	 = br_dev_set_mac_address,
	.ndo_set_multicast_list	 = br_dev_set_multicast_list,
	.ndo_change_mtu		 = br_change_mtu,
	.ndo_do_ioctl		 = br_dev_ioctl,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_netpoll_cleanup	 = br_netpoll_cleanup,
#endif
};

static void br_dev_free(struct net_device *dev)
{
	//struct net_bridge *br = netdev_priv(dev);
	free_netdev(dev);
}

void br_dev_setup(struct net_device *dev)
{
	memset(dev->dev_addr, 0, ETH_ALEN);

	ether_setup(dev);

        dev->netdev_ops = &br_netdev_ops;
	dev->destructor = br_dev_free;

	SET_ETHTOOL_OPS(dev, &br_ethtool_ops);
	
	dev->tx_queue_len = 0;
	dev->priv_flags = IFF_EBRIDGE;

	dev->features = NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_HIGHDMA |
			NETIF_F_GSO_MASK | NETIF_F_NO_CSUM | NETIF_F_LLTX |
			NETIF_F_NETNS_LOCAL | NETIF_F_GSO;
}
