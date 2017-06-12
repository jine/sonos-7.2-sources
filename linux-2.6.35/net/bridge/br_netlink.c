/*
 *	Bridge netlink control interface
 *
 *	Authors:
 *	Stephen Hemminger		<shemminger@osdl.org>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <net/rtnetlink.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include "br_private.h"

static inline size_t br_nlmsg_size(void)
{
	return NLMSG_ALIGN(sizeof(struct ifinfomsg))
	       + nla_total_size(IFNAMSIZ) /* IFLA_IFNAME */
	       + nla_total_size(MAX_ADDR_LEN) /* IFLA_ADDRESS */
	       + nla_total_size(4) /* IFLA_MASTER */
	       + nla_total_size(4) /* IFLA_MTU */
	       + nla_total_size(4) /* IFLA_LINK */
	       + nla_total_size(1) /* IFLA_OPERSTATE */
	       + nla_total_size(1); /* IFLA_PROTINFO */
}

/*
 * Create one netlink message for one interface
 * Contains port and master info as well as carrier and bridge state.
 */
static int br_fill_ifinfo(struct sk_buff *skb, const struct net_bridge_port *port,
			  u32 pid, u32 seq, int event, unsigned int flags)
{
	const struct net_bridge *br = port->br;
	const struct net_device *dev = port->dev;
	struct ifinfomsg *hdr;
	struct nlmsghdr *nlh;
	u8 operstate = netif_running(dev) ? dev->operstate : IF_OPER_DOWN;

	pr_debug("br_fill_info event %d port %s master %s\n",
		 event, dev->name, br->dev->name);

	nlh = nlmsg_put(skb, pid, seq, event, sizeof(*hdr), flags);
	if (nlh == NULL)
		return -EMSGSIZE;

	hdr = nlmsg_data(nlh);
	hdr->ifi_family = AF_BRIDGE;
	hdr->__ifi_pad = 0;
	hdr->ifi_type = dev->type;
	hdr->ifi_index = dev->ifindex;
	hdr->ifi_flags = dev_get_flags(dev);
	hdr->ifi_change = 0;

	NLA_PUT_STRING(skb, IFLA_IFNAME, dev->name);
	NLA_PUT_U32(skb, IFLA_MASTER, br->dev->ifindex);
	NLA_PUT_U32(skb, IFLA_MTU, dev->mtu);
	NLA_PUT_U8(skb, IFLA_OPERSTATE, operstate);

	if (dev->addr_len)
		NLA_PUT(skb, IFLA_ADDRESS, dev->addr_len, dev->dev_addr);

	if (dev->ifindex != dev->iflink)
		NLA_PUT_U32(skb, IFLA_LINK, dev->iflink);

	if (event == RTM_NEWLINK)
		NLA_PUT_U8(skb, IFLA_PROTINFO, port->state);

	return nlmsg_end(skb, nlh);

nla_put_failure:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

/*
 * Notify listeners of a change in port information
 */
void br_ifinfo_notify(int event, struct net_bridge_port *port)
{
	struct net *net = dev_net(port->dev);
	struct sk_buff *skb;
	int err = -ENOBUFS;

	pr_debug("bridge notify event=%d\n", event);
	skb = nlmsg_new(br_nlmsg_size(), GFP_ATOMIC);
	if (skb == NULL)
		goto errout;

	err = br_fill_ifinfo(skb, port, 0, 0, event, 0);
	if (err < 0) {
		/* -EMSGSIZE implies BUG in br_nlmsg_size() */
		WARN_ON(err == -EMSGSIZE);
		kfree_skb(skb);
		goto errout;
	}
	rtnl_notify(skb, net, 0, RTNLGRP_LINK, NULL, GFP_ATOMIC);
errout:
	if (err < 0)
		rtnl_set_sk_err(net, RTNLGRP_LINK, err);
}

#ifdef CONFIG_SONOS_BRIDGE_PROXY
static unsigned int sats_match_bridge(const struct net_bridge *br) {

	struct net_bridge_port *p;
	unsigned int num_sats_checked = 0;

	if (!br->current_ipv4_addr) {
		return 0;
	}

	list_for_each_entry_rcu(p, &br->port_list, list) {
		if (p->is_satellite) {
			num_sats_checked++;
			if (!p->sat_ip || p->sat_ip != br->current_ipv4_addr) {
				/* We're very strict here on purpose. If
				 * there's even a single satellite that
				 * either has no IP set yet, or has an
				 * IP that doesn't match the bridge,
				 * we'll return "no match". */
				return 0;
			}
		}
	}

	/* If we checked at least one sat and it matched, return true. */
	return num_sats_checked;
}

static void br_dupip_notify(struct net_bridge *br, int time)
{
	struct net *net = dev_net(br->dev);
	struct sk_buff *skb;
	int err = -ENOBUFS;
	struct nlmsghdr *nlh;

	skb = nlmsg_new(RTA_LENGTH(sizeof(time)), GFP_ATOMIC);

	if (skb == NULL)
		goto errout;

	nlh = nlmsg_put(skb, 0, 0, RWM_DUPE_IP, 0, 0);
	RTA_PUT(skb, RWA_DUPE_TIME, sizeof(time), &time);
	nlmsg_end(skb, nlh);
	rtnl_notify(skb, net, 0, RTMGRP_Rincon, NULL, GFP_ATOMIC);
	return;

rtattr_failure:
	nlmsg_cancel(skb, nlh);

errout:
	kfree_skb(skb);
	rtnl_set_sk_err(net, RTMGRP_Rincon, err);
}

void br_dupip_timer_expired(unsigned long arg)
{
	struct net_bridge *br = (struct net_bridge *)arg;
	static const int dupip_notify_interval_secs = 30;

	spin_lock_bh(&br->lock);

	if (sats_match_bridge(br)) {
		br_dupip_notify(br, ((long)jiffies - (long)br->dupip_start) / HZ);
		mod_timer(&br->dupip_timer,
			  jiffies + dupip_notify_interval_secs*HZ);
	} else {
		printk("br proxy: IP conflict event ends %d\n", __LINE__);
		br_dupip_notify(br, -1);
	}

	spin_unlock_bh(&br->lock);
}
#endif

void br_dupip_check(struct net_bridge *br)
{
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	if (!sats_match_bridge(br) && timer_pending(&br->dupip_timer)) {
		printk("br proxy: IP conflict event ends %d\n", __LINE__);
		br_dupip_notify(br, -1);
		del_timer(&br->dupip_timer);
	} else if (sats_match_bridge(br) && !timer_pending(&br->dupip_timer)) {
		printk("br proxy: IP conflict event begins\n");
		br->dupip_start = jiffies;
		mod_timer(&br->dupip_timer, jiffies);
	}
#endif
}

/*
 * Dump information about all ports, in response to GETLINK
 */
static int br_dump_ifinfo(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const unsigned char *src  = eth_hdr(skb)->h_source;
	struct net_bridge_port *p ;
	int idx;

	idx = 0;
	for_each_netdev(net, dev) {
		/* not a bridge port */
		if (dev->br_port_list == NULL || idx < cb->args[0])
			goto skip;

        p = br_find_port(src, dev->br_port_list);

		if (br_fill_ifinfo(skb, p, NETLINK_CB(cb->skb).pid,
				   cb->nlh->nlmsg_seq, RTM_NEWLINK,
				   NLM_F_MULTI) < 0)
			break;
skip:
		++idx;
	}

	cb->args[0] = idx;

	return skb->len;
}

/*
 * Change state of port (ie from forwarding to blocking etc)
 * Used by spanning tree in user space.
 */
static int br_rtm_setlink(struct sk_buff *skb,  struct nlmsghdr *nlh, void *arg)
{
	struct net *net = sock_net(skb->sk);
    const unsigned char *src  = eth_hdr(skb)->h_source;
	struct ifinfomsg *ifm;
	struct nlattr *protinfo;
	struct net_device *dev;
	struct net_bridge_port *p;
	u8 new_state;

	if (nlmsg_len(nlh) < sizeof(*ifm))
		return -EINVAL;

	ifm = nlmsg_data(nlh);
	if (ifm->ifi_family != AF_BRIDGE)
		return -EPFNOSUPPORT;

	protinfo = nlmsg_find_attr(nlh, sizeof(*ifm), IFLA_PROTINFO);
	if (!protinfo || nla_len(protinfo) < sizeof(u8))
		return -EINVAL;

	new_state = nla_get_u8(protinfo);
	if (new_state > BR_STATE_BLOCKING)
		return -EINVAL;

	dev = __dev_get_by_index(net, ifm->ifi_index);
	if (!dev)
		return -ENODEV;

	if (!dev->br_port_list)
		return -EINVAL;

    p = br_find_port(src, dev->br_port_list);

	/* if kernel STP is running, don't allow changes */
	if (p->br->stp_enabled)
		return -EBUSY;

	if (!netif_running(dev) ||
	    (!netif_carrier_ok(dev) && new_state != BR_STATE_DISABLED))
		return -ENETDOWN;

	p->state = new_state;
	br_log_state(p);
	return 0;
}


int __init br_netlink_init(void)
{
	if (__rtnl_register(PF_BRIDGE, RTM_GETLINK, NULL, br_dump_ifinfo))
		return -ENOBUFS;

	/* Only the first call to __rtnl_register can fail */
	__rtnl_register(PF_BRIDGE, RTM_SETLINK, br_rtm_setlink, NULL);

	return 0;
}

void __exit br_netlink_fini(void)
{
	rtnl_unregister_all(PF_BRIDGE);
}

