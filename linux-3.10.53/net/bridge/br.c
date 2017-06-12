/*
 *	Generic parts
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/llc.h>
#include <net/llc.h>
#include <net/stp.h>

#include "br_private.h"
static struct pernet_operations br_net_ops = {
	.exit	= br_net_exit,
};

static int __init br_init(void)
{
	int err;

	err = br_fdb_init();
	if (err)
		goto err_out;

	err = register_pernet_subsys(&br_net_ops);
	if (err)
		goto err_out1;

	err = br_netfilter_init();
	if (err)
		goto err_out2;

	err = register_netdevice_notifier(&br_device_notifier);
	if (err)
		goto err_out3;

#ifdef CONFIG_SONOS_BRIDGE_PROXY
	err = register_inetaddr_notifier(&br_inetaddr_notifier);
	if (err)
		goto err_out4;
#endif

	err = br_netlink_init();
	if (err)
		goto err_out5;

	brioctl_set(br_ioctl_deviceless_stub);

	br_handle_frame_hook = sonos_br_handle_frame;

	return 0;
err_out5:
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	unregister_inetaddr_notifier(&br_inetaddr_notifier);
err_out4:
#endif
	unregister_netdevice_notifier(&br_device_notifier);
err_out3:
	br_netfilter_fini();
err_out2:
	unregister_pernet_subsys(&br_net_ops);
err_out1:
	br_fdb_fini();
err_out:
	return err;
}

static void __exit br_deinit(void)
{

	br_netlink_fini();
	unregister_netdevice_notifier(&br_device_notifier);
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	unregister_inetaddr_notifier(&br_inetaddr_notifier);
#endif
	brioctl_set(NULL);

	unregister_pernet_subsys(&br_net_ops);

	rcu_barrier(); /* Wait for completion of call_rcu()'s */

	br_netfilter_fini();

	br_handle_frame_hook = NULL;

	br_fdb_fini();
}

int (*br_should_route_hook)(struct sk_buff *skb) = NULL;
EXPORT_SYMBOL(br_should_route_hook);

module_init(br_init)
module_exit(br_deinit)
MODULE_LICENSE("GPL");
MODULE_VERSION(BR_VERSION);
MODULE_ALIAS_RTNL_LINK("bridge");
