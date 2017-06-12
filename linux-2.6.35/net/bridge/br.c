/*
 *	Generic parts
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br.c#1 $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>

#include "br_private.h"

static struct pernet_operations br_net_ops = {
    .exit   = br_net_exit,
};

int (*br_should_route_hook)(struct sk_buff *skb) = NULL;

static int __init br_init(void)
{
	br_fdb_init();

#ifdef CONFIG_BRIDGE_NETFILTER
	if (br_netfilter_init())
		return 1;
#endif
	brioctl_set(br_ioctl_deviceless_stub);
	br_handle_frame_hook = br_handle_frame;
        
	// KLUDGE: Only used by ATM so no porting: br_fdb_get_hook = br_fdb_get;
	// KLUDGE: Only used by ATM so no porting: br_fdb_put_hook = br_fdb_put;

	register_netdevice_notifier(&br_device_notifier);
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	register_inetaddr_notifier(&br_inetaddr_notifier);
#endif
	register_pernet_subsys(&br_net_ops);

	br_netlink_init();

	return 0;
}

static void __exit br_deinit(void)
{
#ifdef CONFIG_BRIDGE_NETFILTER
	br_netfilter_fini();
#endif

	br_netlink_fini();

	unregister_netdevice_notifier(&br_device_notifier);
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	unregister_inetaddr_notifier(&br_inetaddr_notifier);
#endif
	brioctl_set(NULL);

	unregister_pernet_subsys(&br_net_ops);

	synchronize_net();

	// KLUDGE: Only used by ATM so no porting: br_fdb_get_hook = NULL;
	// KLUDGE: Only used by ATM so no porting: br_fdb_put_hook = NULL;

	br_handle_frame_hook = NULL;
	br_fdb_fini();
}

EXPORT_SYMBOL(br_should_route_hook);

module_init(br_init)
module_exit(br_deinit)
MODULE_LICENSE("GPL");
