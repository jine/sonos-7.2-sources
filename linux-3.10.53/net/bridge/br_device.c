/*
 *	Device handling code
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
#include <linux/netpoll.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/list.h>
#include <linux/netfilter_bridge.h>

#include <asm/uaccess.h>
#include "br_private.h"

/* net device transmit always called with BH disabled */
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
	struct net_bridge *br = netdev_priv(dev);

	br_stp_disable_bridge(br);

	netif_stop_queue(dev);

	return 0;
}

static struct rtnl_link_stats64 *br_get_stats64(struct net_device *dev,
						struct rtnl_link_stats64 *stats)
{
	struct net_bridge *br = netdev_priv(dev);
	stats->tx_bytes   = br->statistics.tx_bytes;
	stats->tx_packets = br->statistics.tx_packets;
	stats->rx_bytes   = br->statistics.rx_bytes;
	stats->rx_packets = br->statistics.rx_packets;
	return stats;
}

static int br_change_mtu(struct net_device *dev, int new_mtu)
{
	struct net_bridge *br = netdev_priv(dev);
	if (new_mtu < 68 || new_mtu > br_min_mtu(br))
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}

/* Allow setting mac address to any valid ethernet address. */
static int br_set_mac_address(struct net_device *dev, void *p)
{
        printk("br: set_mac_address");
	return 0;
}

static void br_getinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, "bridge", sizeof(info->driver));
	strlcpy(info->version, BR_VERSION, sizeof(info->version));
	strlcpy(info->fw_version, "N/A", sizeof(info->fw_version));
	strlcpy(info->bus_info, "N/A", sizeof(info->bus_info));
}

static struct lock_class_key bridge_netdev_addr_lock_key;

static void br_set_lockdep_class(struct net_device *dev)
{
	lockdep_set_class(&dev->addr_list_lock, &bridge_netdev_addr_lock_key);
}

static int br_dev_init(struct net_device *dev)
{
	br_set_lockdep_class(dev);
	return 0;
}

static const struct ethtool_ops br_ethtool_ops = {
	.get_drvinfo    = br_getinfo,
	.get_link	= ethtool_op_get_link,
};

static const struct net_device_ops br_netdev_ops = {
	.ndo_open		 = br_dev_open,
	.ndo_stop		 = br_dev_stop,
	.ndo_init		 = br_dev_init,
	.ndo_start_xmit		 = br_dev_xmit,
	.ndo_get_stats64	 = br_get_stats64,
	.ndo_set_mac_address	 = br_set_mac_address,
	.ndo_set_rx_mode	 = br_dev_set_multicast_list,
	.ndo_change_mtu		 = br_change_mtu,
	.ndo_do_ioctl		 = br_dev_ioctl,
};

static void br_dev_free(struct net_device *dev)
{
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
			NETIF_F_GSO_MASK | NETIF_F_LLTX |
			NETIF_F_NETNS_LOCAL | NETIF_F_GSO;
}
