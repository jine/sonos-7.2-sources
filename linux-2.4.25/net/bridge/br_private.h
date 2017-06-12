/*
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_private.h,v 1.1.1.1 2006/12/23 00:48:21 holmgren Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#ifndef _BR_PRIVATE_H
#define _BR_PRIVATE_H

#include <linux/netdevice.h>
#include <linux/miscdevice.h>
#include <linux/if_bridge.h>
#include "br_private_timer.h"

#define BR_HASH_BITS 8
#define BR_HASH_SIZE (1 << BR_HASH_BITS)

#define BR_HOLD_TIME       (1*HZ)
#define BR_DIRECT_STP_TIME (10*HZ)

#define BR_MAX_MCAST_GROUPS 16

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#define RX_STATS_CHECK_INTERVAL 10
/*
 * Frames per second to detect MC/BC stream.
 * This number is chosen because voice stream like VOIP may be sent by 33.3
 * or 50 frames per second, and it should be a safe threshold to detect video
 * stream.
 */
#define BCMC_REPORT_FPS_THRESHOLD 64

typedef struct bridge_id bridge_id;
typedef struct mac_addr mac_addr;
typedef __u16 port_id;

struct bridge_id
{
	unsigned char	prio[2];
	unsigned char	addr[6];
};

struct mac_addr
{
	unsigned char	addr[6];
	unsigned char	pad[2];
};

struct net_bridge_fdb_entry
{
	struct net_bridge_fdb_entry	*next_hash;
	struct net_bridge_fdb_entry	**pprev_hash;
	atomic_t			use_count;
	mac_addr			addr;
	struct net_bridge_port		*dst;
	struct net_bridge_port		*dst_direct;
	int                             priority;
	unsigned long			ageing_timer;
	unsigned			is_local:1;
	unsigned			is_static:1;
};

/* This is the list of members of the multicast group associated with a 
   particular port on the bridge. */
struct net_bridge_mcast_rx_mac
{
        unsigned char                   addr[6];
        unsigned long                   ageing_timer;
        struct net_bridge_mcast_rx_mac  *next;
};

/* This is the list of ports where members of the multicast group have been
   observed.  An entry with dst == 0 indicates the local bridge interface is
   a member of the multicast group. */
struct net_bridge_mcast_rx_port
{
        struct net_bridge_port		*dst;
        struct net_bridge_mcast_rx_mac  *rx_mac_list;
        struct net_bridge_mcast_rx_port *next;
};

/* This tracks each multicast group. */
struct net_bridge_mcast_entry
{
	struct net_bridge_mcast_entry	*next_hash;
	struct net_bridge_mcast_entry	*prev_hash;
	atomic_t			use_count;
	unsigned char 			addr[6];
        struct net_bridge_mcast_rx_port *rx_port_list;
};

struct net_bridge_port
{
	struct net_bridge_port		*next;
	struct net_bridge		*br;
	struct net_device		*dev;
	int				port_no;

        /* Point-to-Point packet tunnelling */
        unsigned int                    is_p2p:1;
        unsigned int                    is_leaf:1;
        unsigned int                    is_unencap:1;
        unsigned int                    is_unicast:1;
        unsigned char                   p2p_dest_addr[6];

	/* STP */
	port_id				port_id;
	int				state;
        int                             remote_state;
	int				path_cost;
	bridge_id			designated_root;
	int				designated_cost;
	bridge_id			designated_bridge;
	port_id				designated_port;
	unsigned			topology_change_ack:1;
	unsigned			config_pending:1;
	int				priority;

	struct br_timer			forward_delay_timer;
	struct br_timer			hold_timer;
	struct br_timer			message_age_timer;

    	/* direct routing */
	u8                              direct_enabled;
	unsigned char                   direct_addr[6];
	u32                             direct_last_stp_time;
};

struct net_bridge_port_list_node
{
        struct net_bridge_port*                port;
        struct net_bridge_port_list_node*      next;
};

struct net_bridge_stats
{
	unsigned long	rx_mc_count;
	unsigned long	rx_mc_count_peak;
	unsigned long   rx_mc_peak_ts;
	unsigned long	rx_mc_hit;
	unsigned long	rx_bc_count;
	unsigned long	rx_bc_count_peak;
	unsigned long   rx_bc_peak_ts;
	unsigned long	rx_bc_hit;
	unsigned long	rx_start_time;
};

struct net_bridge
{
	struct net_bridge		*next;
	rwlock_t			lock;
	struct net_bridge_port		*port_list;
        struct net_bridge_port          *leaf_list;
	struct net_device		dev;
	struct net_device_stats		statistics;
	rwlock_t			hash_lock;
	struct net_bridge_fdb_entry	*hash[BR_HASH_SIZE];
	struct timer_list		tick;

	/* STP */
	bridge_id			designated_root;
	int				root_path_cost;
	int				root_port;
	int				max_age;
	int				hello_time;
	int				forward_delay;
	bridge_id			bridge_id;
	int				bridge_max_age;
	int				bridge_hello_time;
	int				bridge_forward_delay;
	unsigned			stp_enabled:1;
	unsigned			topology_change:1;
	unsigned			topology_change_detected:1;

        /* multicast group management */
        unsigned                        num_mcast_groups;
        unsigned char                   mcast_groups[BR_MAX_MCAST_GROUPS][6];
        int                             mcast_advertise_time;
        rwlock_t                        mcast_lock;
        struct net_bridge_mcast_entry   *mcast_hash[BR_HASH_SIZE];

	/* SONOS: Fixed MAC address */
	unsigned char                   use_static_mac;
	unsigned char                   static_mac[6];

	struct br_timer			hello_timer;
	struct br_timer			tcn_timer;
	struct br_timer			topology_change_timer;
	struct br_timer			gc_timer;
        struct br_timer                 mcast_timer;

	int				ageing_time;
	int				gc_interval;
	struct net_bridge_stats		br_stats;
};

extern struct notifier_block br_device_notifier;
extern unsigned char bridge_ula[6];
extern unsigned char rincon_gmp_addr[6];
extern unsigned char broadcast_addr[6];
extern unsigned char igmp_ah_addr[6];
extern unsigned char igmp_ar_addr[6];
extern unsigned char igmp_amr_addr[6];
extern unsigned char upnp_addr[6];
extern unsigned char mdns_addr[6];

static inline void br_stats_init(struct net_bridge *br)
{
	memset(&br->br_stats, 0, sizeof(br->br_stats));
	br->br_stats.rx_start_time = jiffies;
}

/* br_direct.c */
extern int br_direct_unicast(struct net_bridge_port *src,
			     struct net_bridge_fdb_entry *fdbe,
			     struct sk_buff *skb,
			     void (*__stp_hook)(const struct net_bridge_port *src,
						const struct net_bridge_port *dst,
						struct sk_buff *skb),
			     void (*__direct_hook)(const struct net_bridge_port *src,
						   const struct net_bridge_port *dst,
						   struct sk_buff *skb));

/* br.c */
extern void br_dec_use_count(void);
extern void br_inc_use_count(void);

/* br_device.c */
extern void br_dev_setup(struct net_device *dev);
extern int br_dev_xmit(struct sk_buff *skb, struct net_device *dev);

/* br_fdb.c */
extern void br_fdb_changeaddr(struct net_bridge_port_list_node *pl,
		       unsigned char *newaddr);
extern void br_fdb_cleanup(struct net_bridge *br);
extern void br_fdb_delete_by_port(struct net_bridge *br,
			   struct net_bridge_port *p);
extern struct net_bridge_fdb_entry *br_fdb_get(struct net_bridge *br,
					unsigned char *addr);
extern void br_fdb_put(struct net_bridge_fdb_entry *ent);
extern int br_fdb_get_entries(struct net_bridge *br,
			unsigned char *_buf,
			int maxnum,
			int offset);
extern struct net_bridge_fdb_entry *br_fdb_insert(struct net_bridge *br,
						  struct net_bridge_port *source,
						  unsigned char *addr,
						  int is_local);
extern void br_fdb_update_dst_direct(struct net_bridge *br,
				     struct net_bridge_port *p);

/* br_forward.c */
extern void br_deliver(struct net_bridge_port *from,
                struct net_bridge_port *to,
		struct sk_buff *skb);
extern void br_deliver_direct(const struct net_bridge_port *from,
                              const struct net_bridge_port *to,
                              struct sk_buff *skb);
extern void br_forward(struct net_bridge_port *from, 
                struct net_bridge_port *to,
		struct sk_buff *skb);
extern void br_forward_direct(const struct net_bridge_port *from,
                              const struct net_bridge_port *to,
			      struct sk_buff *skb);
extern void br_flood_deliver(struct net_bridge *br,
                      struct net_bridge_port *from, 
		      struct sk_buff *skb,
		      int clone);
extern void br_flood_forward(struct net_bridge *br,
                      struct net_bridge_port *from, 
		      struct sk_buff *skb,
		      int clone);

/* br_if.c */
extern int br_initial_port_cost(struct net_device *dev);
extern int br_add_bridge(char *name);
extern int br_del_bridge(char *name);
extern int br_add_if(struct net_bridge *br,
	      struct net_device *dev);
extern int br_del_if(struct net_bridge *br,
	      struct net_device *dev);
extern int br_get_bridge_ifindices(int *indices,
			    int num);
extern void br_get_port_ifindices(struct net_bridge *br,
			   int *ifindices);
extern unsigned char br_get_any_forwarding(struct net_bridge *br);
extern void br_set_static_mac(struct net_bridge *br, unsigned char *mac);

/* br_input.c */
extern void br_handle_frame(struct sk_buff *skb);

/* br_ioctl.c */
extern void br_call_ioctl_atomic(void (*fn)(void));
extern int br_ioctl(struct net_bridge *br,
	     unsigned int cmd,
	     unsigned long arg0,
	     unsigned long arg1,
	     unsigned long arg2);
extern int br_ioctl_deviceless_stub(unsigned long arg);

/* br_stp.c */
extern void br_log_state(struct net_bridge_port *p, char *state);
extern int br_is_root_bridge(struct net_bridge *br);
extern struct net_bridge_port *br_get_port(struct net_bridge *br,
				    int port_no);
extern void br_init_port(struct net_bridge_port *p);
extern port_id br_make_port_id(struct net_bridge_port *p);
extern void br_become_designated_port(struct net_bridge_port *p);

/* br_stp_if.c */
extern void br_stp_enable_bridge(struct net_bridge *br);
extern void br_stp_disable_bridge(struct net_bridge *br);
extern void br_stp_enable_port(struct net_bridge_port *p);
extern void br_stp_disable_port(struct net_bridge_port *p);
extern void br_stp_recalculate_bridge_id(struct net_bridge *br);
extern void br_stp_set_bridge_priority(struct net_bridge *br,
				int newprio);
extern void br_stp_set_port_priority(struct net_bridge_port *p,
			      int newprio);
extern void br_stp_set_path_cost(struct net_bridge_port *p,
			  int path_cost);
extern int br_stp_mod_port_addr(struct net_bridge *br,
				unsigned char* oldaddr,
				unsigned char* newaddr);

/* br_stp_bpdu.c */
extern void br_stp_handle_bpdu(struct net_bridge_port *p,
                          struct sk_buff *skb);

/* br_tunnel.c */
extern int br_add_p2p_tunnel(struct net_bridge *br,
                             struct net_device *dev,
                             unsigned char* daddr,
                             int path_cost);
extern int br_set_p2p_tunnel_path_cost(struct net_bridge *br,
                                          struct net_device *dev,
                                          unsigned char* daddr,
                                          int path_cost);
extern int br_set_p2p_tunnel_remote_stp_state(struct net_bridge *br,
                                          struct net_device *dev,
                                          unsigned char* daddr,
                                          int remote_stp_state);
extern int br_del_p2p_tunnel(struct net_bridge *br,
                             struct net_device *dev,
                             unsigned char* daddr);
extern int br_add_p2p_tunnel_leaf(struct net_bridge *br,
                                  struct net_device *dev,
                                  unsigned char* daddr,
                                  int is_unencap);
extern int br_get_p2p_tunnel_states(struct net_bridge *br,
                                    struct net_device *dev,
                                    unsigned int max_records,
                                    unsigned char* state_data,
                                    unsigned int* state_data_len);

extern int br_set_p2p_direct_enabled(struct net_bridge *br,
				     struct net_device *dev,
				     unsigned char* daddr,
				     int enabled);

int br_set_p2p_direct_addr(struct net_bridge *br,
			   struct net_device *dev,
			   unsigned char* daddr,
			   unsigned char* direct_addr);

/* br_mcast.c */
extern void br_mcast_transmit_grouplist(struct net_bridge *br);
extern void br_mcast_handle_grouplist(struct net_bridge *br,
                                      struct net_bridge_port *source,
                                      struct sk_buff *skb);
extern void br_mcast_destroy_list(struct net_bridge *br);
extern void br_mcast_age_list(struct net_bridge *br);
extern void br_mcast_delete_by_port(struct net_bridge *br,
                                    struct net_bridge_port *p);
extern void br_mcast_put(struct net_bridge_mcast_entry *me);
extern struct net_bridge_mcast_entry *br_mcast_get(struct net_bridge *br, 
                                                   unsigned char *addr);
extern int br_mcast_fdb_get_entries(struct net_bridge *br,
                                    unsigned char *buf,
                                    unsigned int buf_len,
                                    int offset);

/* br_priority.c */
extern int br_priority_for_addr(const unsigned char *addr);
extern int br_priority_for_bpdu(void);

#endif
extern void br_mcast_check(struct sk_buff *skb, 
			   struct net_bridge *br, 
			   struct net_bridge_port *p);
extern int br_mcast_is_management_header(struct ethhdr *ether);

