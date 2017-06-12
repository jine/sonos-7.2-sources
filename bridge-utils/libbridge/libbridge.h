/*
 * Copyright (C) 2000 Lennert Buytenhek
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _LIBBRIDGE_H
#define _LIBBRIDGE_H

#include <linux/autoconf.h>
#include <linux/version.h>

#include <net/if.h>
#include <linux/if_ether.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) && !defined(CONFIG_SONOS_BRIDGE_PROXY)
#include <linux/in6.h>
#endif // Any 3.10 device without bridge proxy
#if !((LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) && defined(CONFIG_SONOS_BRIDGE_PROXY))
#include <linux/if_bridge.h>
#endif // Any device except 3.10 with bridge proxy

#ifndef BCMC_HIST_SIZE
#define BCMC_HIST_SIZE 32
#endif

#define MULTICAST_TYPE 1
#define BROADCAST_TYPE 2

struct bridge;
struct bridge_info;
struct fdb_entry;
struct port;
struct port_info;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
struct bridge_bcmc_hit
{
	u_int8_t      src[ETH_ALEN];
	u_int8_t      dest[ETH_ALEN];
	unsigned long timestamp;
	unsigned long packet_count;
	u_int8_t      packet_type;
};
#endif

struct bridge_stats
{
	u_int32_t rx_mc_count_peak;
	u_int32_t rx_bc_count_peak;
	u_int32_t rx_mc_peak_ts;
	u_int32_t rx_bc_peak_ts;
	u_int32_t rx_mc_hit;
	u_int32_t rx_bc_hit;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct bridge_bcmc_hit bcmc_history[BCMC_HIST_SIZE];
#endif
};

struct bridge_id
{
	unsigned char prio[2];
	unsigned char addr[6];
};

struct bridge_info
{
	struct bridge_id designated_root;
	struct bridge_id bridge_id;
	int root_path_cost;
	struct timeval max_age;
	struct timeval hello_time;
	struct timeval forward_delay;
	struct timeval bridge_max_age;
	struct timeval bridge_hello_time;
	struct timeval bridge_forward_delay;
	unsigned topology_change:1;
	unsigned topology_change_detected:1;
	int root_port;
	unsigned stp_enabled:1;
	struct timeval ageing_time;
	struct timeval gc_interval;
	struct timeval hello_timer_value;
	struct timeval tcn_timer_value;
	struct timeval topology_change_timer_value;
	struct timeval gc_timer_value;
};

struct bridge
{
	struct bridge *next;

	int ifindex;
	char ifname[IFNAMSIZ];
	struct port *firstport;
	struct port *ports[256];
	struct bridge_info info;
	struct bridge_stats stats;
};

struct fdb_entry
{
	u_int8_t mac_addr[6];
	int port_no;
	unsigned is_local:1;
	struct timeval ageing_timer_value;
};

struct port_info
{
	struct bridge_id designated_root;
	struct bridge_id designated_bridge;
	u_int16_t port_id;
	u_int16_t designated_port;
	int path_cost;
	int designated_cost;
	int state;
	unsigned top_change_ack:1;
	unsigned config_pending:1;
	struct timeval message_age_timer_value;
	struct timeval forward_delay_timer_value;
	struct timeval hold_timer_value;
	unsigned is_p2p:1;
	unsigned is_uplink:1;
	u_int8_t direct_enabled;
	u_int8_t p2p_dest_addr[6];
	u_int8_t remote_state;
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	u_int32_t sat_ip;
#endif
};

struct port
{
	struct port *next;

	int index;
	int ifindex;
	struct bridge *parent;
	struct port_info info;
};

extern struct bridge *bridge_list;

int br_init(void);
int br_refresh(void);
struct bridge *br_find_bridge(char *brname);
struct port *br_find_port(struct bridge *br, char *portname);
char *br_get_state_name(int state);

int br_add_bridge(char *brname);
int br_del_bridge(char *brname);
int br_add_interface(struct bridge *br, int ifindex);
int br_del_interface(struct bridge *br, int ifindex);
int br_set_bridge_forward_delay(struct bridge *br, struct timeval *tv);
int br_set_bridge_hello_time(struct bridge *br, struct timeval *tv);
int br_set_bridge_max_age(struct bridge *br, struct timeval *tv);
int br_set_ageing_time(struct bridge *br, struct timeval *tv);
int br_set_gc_interval(struct bridge *br, struct timeval *tv);
int br_set_stp_state(struct bridge *br, int stp_state);
int br_set_uplink_mode(struct bridge *br, int uplink_mode);
#ifdef CONFIG_SONOS_DIAGS
int br_set_forwarding_state(struct bridge *br, int stp_state);
#endif
int br_set_bridge_priority(struct bridge *br, int bridge_priority);
int br_set_port_priority(struct port *p, int port_priority);
int br_set_path_cost(struct port *p, int path_cost);
int br_read_fdb(struct bridge *br, struct fdb_entry *fdbs, int offset, int num);
int br_read_mcast_fdb(struct bridge *br, unsigned char *buf, int len,
                      int offset);
int br_add_p2p_tunnel(struct bridge *br, int ifindex, unsigned char *daddr, 
                      int path_cost);
int br_set_p2p_tunnel_path_cost(struct bridge *br, int ifindex, 
                                unsigned char *daddr, int path_cost);
int br_del_p2p_tunnel(struct bridge *br, int ifindex, unsigned char *daddr);
int br_get_p2p_tunnel_states(struct bridge *br, int ifindex, 
                             unsigned char *state_data, 
                             unsigned int *state_data_len);
int br_set_mac(struct bridge *br, unsigned char *daddr);

/* libc5 combatability */
char *if_indextoname(unsigned int __ifindex, char *__ifname);
unsigned int if_nametoindex(const char *__ifname);

#endif
