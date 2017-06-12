/*
 *	Ioctl handler
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_ioctl.c,v 1.9 2004/08/10 03:46:10 millington Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/if_bridge.h>
#include <linux/inetdevice.h>
#include <asm/uaccess.h>
#include "br_private.h"

static void get_stats(struct net_bridge *br, struct __br_stats *stats)
{
	stats->rx_mc_count_peak = br->br_stats.rx_mc_count_peak;
	stats->rx_bc_count_peak = br->br_stats.rx_bc_count_peak;
	stats->rx_mc_peak_ts = br->br_stats.rx_mc_peak_ts;
	stats->rx_bc_peak_ts = br->br_stats.rx_bc_peak_ts;
	stats->rx_mc_hit = br->br_stats.rx_mc_hit;
	stats->rx_bc_hit = br->br_stats.rx_bc_hit;
}

static int br_ioctl_device(struct net_bridge *br,
			   unsigned int cmd,
			   unsigned long arg0,
			   unsigned long arg1,
			   unsigned long arg2)
{
	if (br == NULL)
		return -EINVAL;

	switch (cmd)
	{
	case BRCTL_ADD_IF:
	case BRCTL_DEL_IF:
	{
		struct net_device *dev;
		int ret;

		dev = dev_get_by_index(arg0);
		if (dev == NULL)
			return -EINVAL;

		if (cmd == BRCTL_ADD_IF)
			ret = br_add_if(br, dev);
		else
			ret = br_del_if(br, dev);

		dev_put(dev);
		return ret;
	}

	case BRCTL_GET_BRIDGE_INFO:
	{
		struct __bridge_info b;

		memset(&b, 0, sizeof(struct __bridge_info));
		memcpy(&b.designated_root, &br->designated_root, 8);
		memcpy(&b.bridge_id, &br->bridge_id, 8);
		b.root_path_cost = br->root_path_cost;
		b.max_age = br->max_age;
		b.hello_time = br->hello_time;
		b.forward_delay = br->forward_delay;
		b.bridge_max_age = br->bridge_max_age;
		b.bridge_hello_time = br->bridge_hello_time;
		b.bridge_forward_delay = br->bridge_forward_delay;
		b.topology_change = br->topology_change;
		b.topology_change_detected = br->topology_change_detected;
		b.root_port = br->root_port;
		b.stp_enabled = br->stp_enabled;
		b.ageing_time = br->ageing_time;
		b.gc_interval = br->gc_interval;
		b.hello_timer_value = br_timer_get_residue(&br->hello_timer);
		b.tcn_timer_value = br_timer_get_residue(&br->tcn_timer);
		b.topology_change_timer_value = br_timer_get_residue(&br->topology_change_timer);
		b.gc_timer_value = br_timer_get_residue(&br->gc_timer);

		if (copy_to_user((void *)arg0, &b, sizeof(b)))
			return -EFAULT;

		return 0;
	}

	case BRCTL_GET_PORT_LIST:
	{
		int i;
		int indices[256];

		for (i=0;i<256;i++)
			indices[i] = 0;

		br_get_port_ifindices(br, indices);
		if (copy_to_user((void *)arg0, indices, 256*sizeof(int)))
			return -EFAULT;

		return 0;
	}

	case BRCTL_SET_BRIDGE_FORWARD_DELAY:
		br->bridge_forward_delay = arg0;
		if (br_is_root_bridge(br))
			br->forward_delay = arg0;
		return 0;

	case BRCTL_SET_BRIDGE_HELLO_TIME:
		br->bridge_hello_time = arg0;
		if (br_is_root_bridge(br))
			br->hello_time = arg0;
		return 0;

	case BRCTL_SET_BRIDGE_MAX_AGE:
		br->bridge_max_age = arg0;
		if (br_is_root_bridge(br))
			br->max_age = arg0;
		return 0;

	case BRCTL_SET_AGEING_TIME:
		br->ageing_time = arg0;
		return 0;

	case BRCTL_SET_GC_INTERVAL:
		br->gc_interval = arg0;
		return 0;

	case BRCTL_GET_PORT_INFO:
	{
		struct __port_info p;
		struct net_bridge_port *pt;

		if ((pt = br_get_port(br, arg1)) == NULL)
			return -EINVAL;

		memset(&p, 0, sizeof(struct __port_info));
		memcpy(&p.designated_root, &pt->designated_root, 8);
		memcpy(&p.designated_bridge, &pt->designated_bridge, 8);
		p.port_id = pt->port_id;
		p.designated_port = pt->designated_port;
		p.path_cost = pt->path_cost;
		p.designated_cost = pt->designated_cost;
		p.state = pt->state;
		p.top_change_ack = pt->topology_change_ack;
		p.config_pending = pt->config_pending;
		p.message_age_timer_value = 
                    br_timer_get_residue(&pt->message_age_timer);
		p.forward_delay_timer_value = 
                    br_timer_get_residue(&pt->forward_delay_timer);
		p.hold_timer_value = br_timer_get_residue(&pt->hold_timer);
                p.is_p2p = pt->is_p2p;
                if (pt->is_p2p) {
			p.direct_enabled = pt->direct_enabled;
 			memcpy(p.p2p_dest_addr, pt->p2p_dest_addr, ETH_ALEN);
                }
                p.remote_state = pt->remote_state;
		p.is_uplink = pt->is_uplink;

		if (copy_to_user((void *)arg0, &p, sizeof(p)))
			return -EFAULT;

		return 0;
	}

	case BRCTL_SET_BRIDGE_STP_STATE:
		br->stp_enabled = arg0?1:0;
		return 0;

	case BRCTL_SET_BRIDGE_PRIORITY:
		br_stp_set_bridge_priority(br, arg0);
		return 0;

	case BRCTL_SET_PORT_PRIORITY:
	{
		struct net_bridge_port *p;

		if ((p = br_get_port(br, arg0)) == NULL)
			return -EINVAL;
		br_stp_set_port_priority(p, arg1);
		return 0;
	}

	case BRCTL_SET_PATH_COST:
	{
		struct net_bridge_port *p;

		if ((p = br_get_port(br, arg0)) == NULL)
			return -EINVAL;
		br_stp_set_path_cost(p, arg1);
		return 0;
	}

	case BRCTL_GET_FDB_ENTRIES:
		return br_fdb_get_entries(br, (void *)arg0, arg1, arg2);

        case BRCTL_MCAST_GET_FDB_ENTRIES:
                return br_mcast_fdb_get_entries(br, (unsigned char*)arg0, 
                                                arg1, arg2);

        case BRCTL_ADD_P2P_TUNNEL:
        case BRCTL_SET_P2P_TUNNEL_PATH_COST:
        case BRCTL_SET_P2P_TUNNEL_STP_STATE:
        case BRCTL_DEL_P2P_TUNNEL:
	case BRCTL_ADD_UPLINK:
        {
		struct net_device *dev;
                unsigned char daddr[ETH_ALEN];
		int ret;

		dev = dev_get_by_index(arg0);
		if (dev == NULL)
			return -EINVAL;

                if (copy_from_user(daddr, (void *)arg1, ETH_ALEN))
                        return -EFAULT;

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))
                BUILD_BUG_ON(sizeof(struct __add_p2p_entry) > sizeof(unsigned long));

                if (cmd == BRCTL_ADD_P2P_TUNNEL)
                        ret = br_add_p2p_tunnel(br, dev, daddr, ((struct __add_p2p_entry *)&arg2)->stp_weight);
                else if (cmd == BRCTL_SET_P2P_TUNNEL_PATH_COST)
                        ret = br_set_p2p_tunnel_path_cost(br, dev, 
                                                          daddr, arg2);
                else if (cmd == BRCTL_SET_P2P_TUNNEL_STP_STATE)
                        ret = br_set_p2p_tunnel_remote_stp_state(br, dev, 
                                                                 daddr, arg2);
                else if (cmd == BRCTL_ADD_UPLINK)
                        ret = br_add_uplink(br, dev, daddr);
                else
                        ret = br_del_p2p_tunnel(br, dev, daddr);

                dev_put(dev);
                return ret;
        }
        case BRCTL_ADD_P2P_TUNNEL_LEAF:
        {
		struct net_device *dev;
                unsigned char daddr[ETH_ALEN];
		int ret;

		dev = dev_get_by_index(arg0);
		if (dev == NULL)
			return -EINVAL;

                if (copy_from_user(daddr, (void *)arg1, ETH_ALEN))
                        return -EFAULT;

                ret = br_add_p2p_tunnel_leaf(br, dev, daddr, arg2);

                dev_put(dev);
                return ret;
        }
        case BRCTL_GET_P2P_TUNNEL_STATES:
        {
                struct net_device *dev;
                int ret;

                /* support returning data for up to 32 point-to-point tunnels;
                   inputs are the length of the list of dest mac addresses,
                   and the list of MAC addresses itself. */
                unsigned char state_data[32 * (1 + ETH_ALEN)];
                unsigned int state_data_len;
                
                dev = dev_get_by_index(arg0);
                if (dev == NULL)
                        return -EINVAL;

                ret = br_get_p2p_tunnel_states(br, dev, 32,
                                               state_data, 
                                               &state_data_len);

                dev_put(dev);

                if (ret != 0)
                        return ret;

                if (copy_to_user((void *)arg1, 
                                 state_data, sizeof(state_data)) ||
                    copy_to_user((void *)arg2,
                                 &state_data_len, sizeof(state_data_len)))
                        ret = -EFAULT;

                return ret;
        }

#ifdef CONFIG_SONOS_DIAGS
        case BRCTL_SET_BRIDGE_FORWARDING_STATE:
                br->sonos_diag_bridge_forward = arg0?1:0;
                return 0;
#endif

	case BRCTL_SET_P2P_DIRECT_ENABLED: {
	struct net_device *dev;
                unsigned char daddr[ETH_ALEN];
		int ret;

		dev = dev_get_by_index(arg0);
		if (dev == NULL) {
			return -EINVAL;
                }

                if (copy_from_user(daddr, (void *)(arg1), ETH_ALEN))
                        ret = -EFAULT;
		
		ret = br_set_p2p_direct_enabled(br, dev, &(daddr[0]), arg2);

		dev_put(dev);

                return ret;
	}
        case BRCTL_SET_P2P_DIRECT_ADDR: {
		struct net_device *dev;
                unsigned char daddr[ETH_ALEN];
                unsigned char direct_addr[ETH_ALEN];
		int ret;

		dev = dev_get_by_index(arg0);
		if (dev == NULL) {
			return -EINVAL;
                }

                if (copy_from_user(daddr, (void *)arg1, ETH_ALEN))
                        ret = -EFAULT;

		if (copy_from_user(direct_addr, (void *)arg2, ETH_ALEN))
			ret = -EFAULT;

		ret = br_set_p2p_direct_addr(br, dev, &(daddr[0]), &(direct_addr[0]));

		dev_put(dev);

                return ret;
	}

	case BRCTL_GET_ROUTING_CAPABILITIES: {
		int ret = 0;

                /* Ask the bridge what it can do */
		unsigned char rc;
                rc = br_get_routing_capabilities(br);
                
		if (copy_to_user((void *)arg0, &rc, sizeof(rc))) {
			ret = -EFAULT;
		}

		return ret;
	}
		
	case BRCTL_GET_ANY_FORWARDING: {
		int ret = 0;

                /* Ask the bridge if anyone is happy */
		unsigned char rc;
                rc = br_get_any_forwarding(br);

		if (copy_to_user((void *)arg0, &rc, sizeof(rc))) {
			ret = -EFAULT;
		}

		return ret;
	}
            
	case BRCTL_SET_STATIC_MAC: {
            int ret = 0;
            unsigned char mac[6];

            if (copy_from_user(mac, (void *)arg0, ETH_ALEN))
                ret = -EFAULT;

            br_set_static_mac(br, mac);

            return ret;
        }

	case BRCTL_MOD_PORT_ADDR:
        {
		unsigned char oldaddr[ETH_ALEN];
		unsigned char newaddr[ETH_ALEN];

		int ret = 0;
		
		if (copy_from_user(oldaddr, (void *)arg0, ETH_ALEN)) {
			ret = -EFAULT;
		}

		if (copy_from_user(newaddr, (void *)arg1, ETH_ALEN)) {
			ret = -EFAULT;
		}

		spin_lock_bh(&br->lock);
		if (br_stp_mod_port_addr(br, oldaddr, newaddr)) {
			ret = -EINVAL;
		}
		spin_unlock_bh(&br->lock);

		return ret;
	}

	case BRCTL_SET_UPLINK_MODE: {
		br_set_uplink_mode(br, (arg0 != 0));
		return 0;
	}

	case BRCTL_GET_STATS: {
		struct __br_stats stats;
		int ret = 0;

		get_stats(br, &stats);

		if (copy_to_user((void *)arg0, &stats, sizeof(stats))) {
			ret = -EFAULT;
		}
		return ret;
	}

	default:
		break;

	}

	return -EOPNOTSUPP;
}

static int br_ioctl_deviceless(unsigned int cmd,
			       unsigned long arg0,
			       unsigned long arg1)
{
	switch (cmd)
	{
	case BRCTL_GET_VERSION:
		return BRCTL_VERSION;

	case BRCTL_GET_BRIDGES:
	{
		int i;
		int indices[64];

		for (i=0;i<64;i++)
			indices[i] = 0;

		if (arg1 > 64)
			arg1 = 64;
		arg1 = br_get_bridge_ifindices(indices, arg1);
		if (copy_to_user((void *)arg0, indices, arg1*sizeof(int)))
			return -EFAULT;

		return arg1;
	}

	case BRCTL_ADD_BRIDGE:
	case BRCTL_DEL_BRIDGE:
	{
		char buf[IFNAMSIZ];

		if (copy_from_user(buf, (void *)arg0, IFNAMSIZ))
			return -EFAULT;

		buf[IFNAMSIZ-1] = 0;

		if (cmd == BRCTL_ADD_BRIDGE)
			return br_add_bridge(buf);

		return br_del_bridge(buf);
	}
	}

	return -EOPNOTSUPP;
}

DECLARE_MUTEX(ioctl_mutex);

int br_ioctl_deviceless_stub(unsigned long arg)
{
	int err;
	unsigned long i[3];

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (copy_from_user(i, (void *)arg, 3*sizeof(unsigned long)))
		return -EFAULT;

	down(&ioctl_mutex);
	err = br_ioctl_deviceless(i[0], i[1], i[2]);
	up(&ioctl_mutex);

	return err;
}

int br_ioctl(struct net_bridge *br, unsigned int cmd, unsigned long arg0, unsigned long arg1, unsigned long arg2)
{
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	down(&ioctl_mutex);
	err = br_ioctl_deviceless(cmd, arg0, arg1);
	if (err == -EOPNOTSUPP)
		err = br_ioctl_device(br, cmd, arg0, arg1, arg2);
	up(&ioctl_mutex);

	return err;
}

void br_call_ioctl_atomic(void (*fn)(void))
{
	down(&ioctl_mutex);
	fn();
	up(&ioctl_mutex);
}
