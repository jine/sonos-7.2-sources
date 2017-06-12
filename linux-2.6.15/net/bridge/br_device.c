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
#include <asm/uaccess.h>
#include "br_private.h"

static struct net_device_stats *br_dev_get_stats(struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	return &br->statistics;
}

int br_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	const unsigned char *dest = skb->data;
	struct net_bridge_fdb_entry *dst;

	br->statistics.tx_packets++;
	br->statistics.tx_bytes += skb->len;

	skb->mac.raw = skb->data;
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
	return 0;
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
	struct net_bridge *br;
        struct dev_mc_list *mc;
	
        br = dev->priv;
	
        spin_lock_bh(&br->lock);

        /* dev->mc_list has the updated list of MAC multicast groups of which
           the device has been configured to be a member. */
        br->num_mcast_groups = 0;
        for (mc = dev->mc_list; mc; mc = mc->next) {
            /* keep certain 'system' multicast MAC addresses out of the list */
            if (0 == memcmp(igmp_ah_addr, mc->dmi_addr, 6) ||
                0 == memcmp(igmp_ar_addr, mc->dmi_addr, 6) ||
                0 == memcmp(igmp_amr_addr, mc->dmi_addr, 6) ||
                0 == memcmp(broadcast_addr, mc->dmi_addr, 6) ||
                0 == memcmp(rincon_gmp_addr, mc->dmi_addr, 6) ||
                0 == memcmp(mdns_addr, mc->dmi_addr, 6) ||
                0 == memcmp(upnp_addr, mc->dmi_addr, 6))
                continue;

            memcpy(br->mcast_groups[br->num_mcast_groups], mc->dmi_addr, 6);
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

void br_dev_setup(struct net_device *dev)
{
	memset(dev->dev_addr, 0, ETH_ALEN);

	ether_setup(dev);

	dev->do_ioctl = br_dev_ioctl;
	dev->get_stats = br_dev_get_stats;
	dev->hard_start_xmit = br_dev_xmit;
	dev->open = br_dev_open;
	dev->set_multicast_list = br_dev_set_multicast_list;
	dev->change_mtu = br_change_mtu;
	dev->destructor = free_netdev;
	SET_MODULE_OWNER(dev);
	dev->stop = br_dev_stop;
	dev->tx_queue_len = 0;
	dev->set_mac_address = NULL;
	dev->priv_flags = IFF_EBRIDGE;
}
