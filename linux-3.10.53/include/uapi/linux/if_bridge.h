/*
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

#ifndef _UAPI_LINUX_IF_BRIDGE_H
#define _UAPI_LINUX_IF_BRIDGE_H

#include <linux/types.h>

#define SYSFS_BRIDGE_ATTR	"bridge"
#define SYSFS_BRIDGE_FDB	"brforward"
#define SYSFS_BRIDGE_PORT_SUBDIR "brif"
#define SYSFS_BRIDGE_PORT_ATTR	"brport"
#define SYSFS_BRIDGE_PORT_LINK	"bridge"

#define BRCTL_VERSION 1

#define BRCTL_GET_VERSION 0
#define BRCTL_GET_BRIDGES 1
#define BRCTL_ADD_BRIDGE 2
#define BRCTL_DEL_BRIDGE 3
#define BRCTL_ADD_IF 4
#define BRCTL_DEL_IF 5
#define BRCTL_GET_BRIDGE_INFO 6
#define BRCTL_GET_PORT_LIST 7
#define BRCTL_SET_BRIDGE_FORWARD_DELAY 8
#define BRCTL_SET_BRIDGE_HELLO_TIME 9
#define BRCTL_SET_BRIDGE_MAX_AGE 10
#define BRCTL_SET_AGEING_TIME 11
#define BRCTL_SET_GC_INTERVAL 12
#define BRCTL_GET_PORT_INFO 13
#define BRCTL_SET_BRIDGE_STP_STATE 14
#define BRCTL_SET_BRIDGE_PRIORITY 15
#define BRCTL_SET_PORT_PRIORITY 16
#define BRCTL_SET_PATH_COST 17
#define BRCTL_GET_FDB_ENTRIES 18

#if defined(CONFIG_SONOS) || defined(__SONOS_LINUX__)
#define BRCTL_ADD_P2P_TUNNEL 19
#define BRCTL_SET_P2P_TUNNEL_PATH_COST 20
#define BRCTL_ADD_P2P_TUNNEL_LEAF 21
/* Flags field for BRCTL_ADD_P2P_TUNNEL_LEAF */
#  define BR_LEAF_FLAGS_IS_UNENCAP (1 << 0)
#  define BR_LEAF_FLAGS_IS_UNICAST (1 << 1)
#  define BR_LEAF_FLAGS_IS_UPLINK  (1 << 15)
#define BRCTL_DEL_P2P_TUNNEL 22
#define BRCTL_MCAST_GET_FDB_ENTRIES 23
#define BRCTL_GET_P2P_TUNNEL_STATES 24
#define BRCTL_SET_P2P_TUNNEL_STP_STATE 25
#define BRCTL_MOD_PORT_ADDR 26
#define BRCTL_MOD_PORT_DEV 27
#define BRCTL_SET_P2P_DIRECT_ADDR 28
#define BRCTL_SET_P2P_DIRECT_ENABLED 29
#define BRCTL_GET_ROUTING_CAPABILITIES 30
#define BRCTL_GET_ANY_FORWARDING 31
#define BRCTL_SET_STATIC_MAC 32
#define BRCTL_SET_UPLINK_MODE 33
#define BRCTL_ADD_UPLINK 34
#define BRCTL_SET_BRIDGE_FORWARDING_STATE 99
#define BRCTL_GET_STATS 35
#endif	// CONFIG_SONOS

#define BR_STATE_DISABLED 0
#define BR_STATE_LISTENING 1
#define BR_STATE_LEARNING 2
#define BR_STATE_FORWARDING 3
#define BR_STATE_BLOCKING 4

struct __bridge_info {
	__u64 designated_root;
	__u64 bridge_id;
	__u32 root_path_cost;
	__u32 max_age;
	__u32 hello_time;
	__u32 forward_delay;
	__u32 bridge_max_age;
	__u32 bridge_hello_time;
	__u32 bridge_forward_delay;
	__u8 topology_change;
	__u8 topology_change_detected;
	__u8 root_port;
	__u8 stp_enabled;
	__u32 ageing_time;
	__u32 gc_interval;
	__u32 hello_timer_value;
	__u32 tcn_timer_value;
	__u32 topology_change_timer_value;
	__u32 gc_timer_value;
};

struct __port_info {
	__u64 designated_root;
	__u64 designated_bridge;
	__u16 port_id;
	__u16 designated_port;
	__u32 path_cost;
	__u32 designated_cost;
	__u8 state;
	__u8 top_change_ack;
	__u8 config_pending;
	__u8 unused0;
	__u32 message_age_timer_value;
	__u32 forward_delay_timer_value;
	__u32 hold_timer_value;

#if defined(CONFIG_SONOS) || defined(__SONOS_LINUX__)
	__u8 is_p2p;
	__u8 p2p_dest_addr[6];
	__u8 remote_state;
	__u8 direct_enabled;
	__u8 is_uplink;
	__u32 sat_ip;
#endif
};

struct __fdb_entry {
	__u8 mac_addr[6];
	__u8 port_no;
	__u8 is_local;
	__u32 ageing_timer_value;
	__u8 port_hi;
	__u8 pad0;
	__u16 unused;
};

#if defined(CONFIG_SONOS) || defined(__SONOS_LINUX__)

#define BCMC_HIST_SIZE 32

struct __br_bcmc_hit {
	__u8 src[6];
	__u8 dest[6];
	__u64 timestamp;
	__u32 packet_count;
	__u8 packet_type;
};

struct __br_stats {
	__u32 rx_mc_count_peak;
	__u32 rx_bc_count_peak;
	__u64 rx_mc_peak_ts;
	__u64 rx_bc_peak_ts;
	__u32 rx_mc_hit;
	__u32 rx_bc_hit;

	struct __br_bcmc_hit bcmc_history[BCMC_HIST_SIZE];
};

/* protocol numbers for packets carrying tunneled ethernet frames; this is
   used by the wireless network only and does not appear on the user's LAN */
#define BR_TUNNEL_PROTOCOLNUM  0x6969 /* Learned     */
#define BR_TUNNEL2_PROTOCOLNUM 0x6971 /* Not learned */

/* protocol number for multicast group membership announcement; this does
   appear on the user's lan, addressed to the 'Rincon group management' 
   group (01:0E:58:DD:DD:DD) */
#define BR_MCAST_GL_PROTOCOLNUM 0x6970
#endif	// CONFIG_SONOS

/* Bridge Flags */
#define BRIDGE_FLAGS_MASTER	1	/* Bridge command to/from master */
#define BRIDGE_FLAGS_SELF	2	/* Bridge command to/from lowerdev */

#define BRIDGE_MODE_VEB		0	/* Default loopback mode */
#define BRIDGE_MODE_VEPA	1	/* 802.1Qbg defined VEPA mode */

/* Bridge management nested attributes
 * [IFLA_AF_SPEC] = {
 *     [IFLA_BRIDGE_FLAGS]
 *     [IFLA_BRIDGE_MODE]
 *     [IFLA_BRIDGE_VLAN_INFO]
 * }
 */
enum {
	IFLA_BRIDGE_FLAGS,
	IFLA_BRIDGE_MODE,
	IFLA_BRIDGE_VLAN_INFO,
	__IFLA_BRIDGE_MAX,
};
#define IFLA_BRIDGE_MAX (__IFLA_BRIDGE_MAX - 1)

#define BRIDGE_VLAN_INFO_MASTER	(1<<0)	/* Operate on Bridge device as well */
#define BRIDGE_VLAN_INFO_PVID	(1<<1)	/* VLAN is PVID, ingress untagged */
#define BRIDGE_VLAN_INFO_UNTAGGED	(1<<2)	/* VLAN egresses untagged */

struct bridge_vlan_info {
	__u16 flags;
	__u16 vid;
};

/* Bridge multicast database attributes
 * [MDBA_MDB] = {
 *     [MDBA_MDB_ENTRY] = {
 *         [MDBA_MDB_ENTRY_INFO]
 *     }
 * }
 * [MDBA_ROUTER] = {
 *    [MDBA_ROUTER_PORT]
 * }
 */
enum {
	MDBA_UNSPEC,
	MDBA_MDB,
	MDBA_ROUTER,
	__MDBA_MAX,
};
#define MDBA_MAX (__MDBA_MAX - 1)

enum {
	MDBA_MDB_UNSPEC,
	MDBA_MDB_ENTRY,
	__MDBA_MDB_MAX,
};
#define MDBA_MDB_MAX (__MDBA_MDB_MAX - 1)

enum {
	MDBA_MDB_ENTRY_UNSPEC,
	MDBA_MDB_ENTRY_INFO,
	__MDBA_MDB_ENTRY_MAX,
};
#define MDBA_MDB_ENTRY_MAX (__MDBA_MDB_ENTRY_MAX - 1)

enum {
	MDBA_ROUTER_UNSPEC,
	MDBA_ROUTER_PORT,
	__MDBA_ROUTER_MAX,
};
#define MDBA_ROUTER_MAX (__MDBA_ROUTER_MAX - 1)

struct br_port_msg {
	__u8  family;
	__u32 ifindex;
};

struct br_mdb_entry {
	__u32 ifindex;
#define MDB_TEMPORARY 0
#define MDB_PERMANENT 1
	__u8 state;
	struct {
		union {
			__be32	ip4;
			struct in6_addr ip6;
		} u;
		__be16		proto;
	} addr;
};

enum {
	MDBA_SET_ENTRY_UNSPEC,
	MDBA_SET_ENTRY,
	__MDBA_SET_ENTRY_MAX,
};
#define MDBA_SET_ENTRY_MAX (__MDBA_SET_ENTRY_MAX - 1)

#endif /* _UAPI_LINUX_IF_BRIDGE_H */