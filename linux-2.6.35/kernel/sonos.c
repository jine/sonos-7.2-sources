/*
 * Copyright (c) 2013, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */
#include <linux/module.h>
#include "mdp.h"

/* In memory copy of the MDP */
struct manufacturing_data_page sys_mdp;
EXPORT_SYMBOL(sys_mdp);

/* These functions are defined to provide a callback interface
 * for getting statistics about Ethernet ports on various players that use
 * internal switches that do not report interfaces directly to the kernel.
 * The callback function is registered from the switch-specific driver module
 * and is called by the IOCTL SIOCGIFSTATS, defined in net/core/dev.c.
 */
#if defined (CONFIG_SONOS_FILLMORE) || defined (CONFIG_SONOS_LIMELIGHT)
#include <linux/netdevice.h>

static int (*sonos_ethernet_get_stats_cback)(char port, struct net_device_stats *phy_stats);

int switch_stats_register_callback(int (*func)(char arg, struct net_device_stats *stats)) {
	if(sonos_ethernet_get_stats_cback) {
		printk(KERN_WARNING "%s: callback already registered!\n", __func__);
		return -EINVAL;
	}
	sonos_ethernet_get_stats_cback = func;
	return 0;
}
EXPORT_SYMBOL(switch_stats_register_callback);

int switch_stats_unregister_callback(void) {
	if(!sonos_ethernet_get_stats_cback) {
		printk(KERN_WARNING "%s: callback already unregistered!\n", __func__);
		return -EINVAL;
	}
	sonos_ethernet_get_stats_cback = NULL;
	return 0;
}
EXPORT_SYMBOL(switch_stats_unregister_callback);

int sonos_ethernet_callback(char arg, struct net_device_stats *stats) {
	if(!sonos_ethernet_get_stats_cback) {
		printk(KERN_ERR "%s: no callback has been registered!\n", __func__);
		return -EINVAL;
	}
	return sonos_ethernet_get_stats_cback(arg, stats);
}
#endif
