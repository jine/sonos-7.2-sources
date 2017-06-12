/*
 *	Device handling code
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_device.c,v 1.3 2004/04/15 00:37:23 millington Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/if_bridge.h>
#include <asm/uaccess.h>
#include "br_private.h"

static int br_dev_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	unsigned long args[4];
	unsigned long *data;

	if (cmd != SIOCDEVPRIVATE)
		return -EOPNOTSUPP;

	data = (unsigned long *)rq->ifr_data;
	if (copy_from_user(args, data, 4*sizeof(unsigned long)))
		return -EFAULT;

	return br_ioctl(dev->priv, args[0], args[1], args[2], args[3]);
}

static struct net_device_stats *br_dev_get_stats(struct net_device *dev)
{
	struct net_bridge *br;

	br = dev->priv;

	return &br->statistics;
}

static int __br_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_bridge *br;
	unsigned char *dest;
	struct net_bridge_fdb_entry *dst;

	br = dev->priv;
	br->statistics.tx_packets++;
	br->statistics.tx_bytes += skb->len;

	dest = skb->mac.raw = skb->data;
	skb_pull(skb, ETH_HLEN);

	if (br->uplink_mode) {

		/* Uplink */
		br_uplink_xmit(br, skb, dest);

	} else if (dest[0] & 1) {
		
		/* Multicast/broadcast */
 		br_flood_deliver(br, 0, skb, 0);
		
	} else {

		/* Unicast */
		dst = br_fdb_get(br, dest);

		if (dst) {

			/* Known address */

			if (0 == skb->priority) {
				skb->priority = dst->priority;
			}

			br_direct_unicast(0, dst, skb,
					  br_deliver,
					  br_deliver_direct);

			br_fdb_put(dst);
			
		} else {
			
			/* Unknown address */
			br_flood_deliver(br, 0, skb, 0);			
		}
	}

	return 0;
}

int br_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_bridge *br;
	int ret;

	br = dev->priv;
	read_lock(&br->lock);
	ret = __br_dev_xmit(skb, dev);
	read_unlock(&br->lock);

	return ret;
}

static int br_dev_open(struct net_device *dev)
{
	struct net_bridge *br;

	netif_start_queue(dev);

	br = dev->priv;
	read_lock(&br->lock);
	br_stp_enable_bridge(br);
	read_unlock(&br->lock);

	return 0;
}

static void br_dev_set_multicast_list(struct net_device *dev)
{
        struct net_bridge *br;
        struct dev_mc_list *mc;

        br = dev->priv;

        write_lock_bh(&br->lock);

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

        write_unlock_bh(&br->lock);
}

static int br_dev_stop(struct net_device *dev)
{
	struct net_bridge *br;

	br = dev->priv;
	read_lock(&br->lock);
	br_stp_disable_bridge(br);
	read_unlock(&br->lock);

	netif_stop_queue(dev);

	return 0;
}

static int br_dev_accept_fastpath(struct net_device *dev, struct dst_entry *dst)
{
	return -1;
}

void br_dev_setup(struct net_device *dev)
{
	memset(dev->dev_addr, 0, ETH_ALEN);

	dev->do_ioctl = br_dev_do_ioctl;
	dev->get_stats = br_dev_get_stats;
	dev->hard_start_xmit = br_dev_xmit;
	dev->open = br_dev_open;
	dev->set_multicast_list = br_dev_set_multicast_list;
	dev->stop = br_dev_stop;
	dev->accept_fastpath = br_dev_accept_fastpath;
	dev->tx_queue_len = 0;
	dev->set_mac_address = NULL;
}
