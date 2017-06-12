/*
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br_private.h#1 $
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
#include <linux/ip.h>
#include <linux/udp.h>

/* DEBUG */
/* #define BR_DEBUG_DIRECT 1 */

#define MAC_ADDR_FMT    "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ADDR_VAR(x) (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5]

#define BR_HASH_BITS 8
#define BR_HASH_SIZE (1 << BR_HASH_BITS)

#define BR_HOLD_TIME       (1*HZ)
#define BR_DIRECT_STP_TIME (10*HZ)

#define BR_VERSION	"6.9"

#if defined(CONFIG_SONOS) || defined(__SONOS_LINUX__)
/*
 * SONOS: Make BR_PORT_BITS match other products (8 instead of 10).  We don't
 *        need more than this, and it makes it easier to play "what's
 *        different" when debugging Casbah bridge code
 */
#define BR_PORT_BITS	8
#else
#define BR_PORT_BITS	10
#endif	// CONFIG_SONOS
#define BR_MAX_PORTS	(1<<BR_PORT_BITS)

#define BR_MAX_MCAST_GROUPS 16

#define RX_STATS_CHECK_INTERVAL 10
/*
 * Frames per second to detect MC/BC stream.
 * This number is chosen because voice stream like VOIP may be sent by 33.3
 * or 50 frames per second, and it should be a safe threshold to detect video
 * stream.
 */
#define BCMC_REPORT_FPS_THRESHOLD 64
#define BCMC_REPORT_TOTAL_PACKETS (BCMC_REPORT_FPS_THRESHOLD * RX_STATS_CHECK_INTERVAL)

#define MCAST_TYPE		  1
#define BCAST_TYPE		  2

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
};

struct net_bridge_fdb_entry
{
	struct hlist_node		hlist;
	struct net_bridge_port		*dst;
	struct net_bridge_port		*dst_direct;
	int                             priority;

	struct rcu_head			rcu;
	atomic_t			use_count;
	unsigned long			ageing_timer;
	mac_addr			addr;
	unsigned char			is_local;
	unsigned char			is_static;
};

/* This is the list of members of the multicast group associated with a 
   particular port on the bridge. */
struct net_bridge_mcast_rx_mac
{
	unsigned char                   addr[6];
	unsigned long                   ageing_timer;
	struct net_bridge_port		*direct_dst;	
	struct net_bridge_mcast_rx_mac  *next;
};

/* This is the list of ports where members of the multicast group have been
   observed.  An entry with dst == 0 indicates the local bridge interface is
   a member of the multicast group. */
struct net_bridge_mcast_rx_port
{
	struct net_bridge_mcast_rx_port *next;
        struct net_bridge_port		*dst;
        struct net_bridge_mcast_rx_mac  *rx_mac_list;
};

/* This tracks each multicast group.
 */
struct net_bridge_mcast_entry
{
	struct net_bridge_mcast_entry  *next_hash;
	struct net_bridge_mcast_entry  *prev_hash;
	atomic_t			use_count;
	unsigned char                   addr[6];
	struct net_bridge_mcast_rx_port *rx_port_list;
};

struct net_bridge_port
{
	struct net_bridge		*br;
	struct net_device		*dev;
	struct list_head		list;

	/* Point-to-Point packet tunnelling */
	unsigned int                    is_p2p:1;
	unsigned int                    is_leaf:1;
	unsigned int                    is_unencap:1;
        unsigned int                    is_unicast:1;
	unsigned int                    is_uplink:1;
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	unsigned int                    is_satellite:1;
	u32                             sat_ip;
#endif
	unsigned char                   p2p_dest_addr[6];

	/* STP */
	u8				priority;
	u16				state;
    	u16				remote_state;
	u32				port_no;
	unsigned char			topology_change_ack;
	unsigned char			config_pending;
	port_id				port_id;
	port_id				designated_port;
	bridge_id			designated_root;
	bridge_id			designated_bridge;
	u32				path_cost;
	u32				designated_cost;

	/* direct routing */
	u8                              direct_enabled;
	unsigned char                   direct_addr[6];
	u32                             direct_last_stp_time;

	struct timer_list		forward_delay_timer;
	struct timer_list		hold_timer;
	struct timer_list		message_age_timer;
	struct kobject			kobj;
	struct rcu_head			rcu;
};

/* SONOS: Every port is on two lists.
 *
 *        The first is maintained using br_port_list, which is a struct
 *        net_bridge_port_list_node *, in struct net_device.
 *        This list contains all ports on the bridge, including leaf ports.
 *
 *        The second list is either port_list or leaf_list in struct
 *        net_bridge.  These contain the only the leaf ports and only the
 *        non-leaf ports, respectively.
 */
struct net_bridge_port_list_node
{
	struct net_bridge_port*                port;
	struct net_bridge_port_list_node*      next;
};

struct net_bridge_bcmc_hit
{
	unsigned char   src[ETH_ALEN];
	unsigned char   dest[ETH_ALEN];
	unsigned long   timestamp;
	unsigned long   packet_count;
	u8              packet_type;
};

struct net_bridge_stats
{
	unsigned long   rx_mc_count;
	unsigned long   rx_mc_count_peak;
	unsigned long   rx_mc_peak_ts;
	unsigned long   rx_mc_hit;
	unsigned char	rx_mc_hit_src[ETH_ALEN];
	unsigned char	rx_mc_hit_dest[ETH_ALEN];
	unsigned long   rx_bc_count;
	unsigned long   rx_bc_count_peak;
	unsigned long   rx_bc_peak_ts;
	unsigned long   rx_bc_hit;
	unsigned char	rx_bc_hit_src[ETH_ALEN];
	unsigned char	rx_bc_hit_dest[ETH_ALEN];
	unsigned long   rx_start_time;

	unsigned int    bcmc_index;
	struct net_bridge_bcmc_hit bcmc_history[BCMC_HIST_SIZE];
};

struct net_bridge
{
	spinlock_t			lock;
	struct list_head		port_list;
	struct list_head		leaf_list;
	struct net_device		*dev;
	struct net_device_stats		statistics;
	spinlock_t			hash_lock;
	struct hlist_head		hash[BR_HASH_SIZE];
	struct list_head		age_list;

	/* STP */
	bridge_id			designated_root;
	bridge_id			bridge_id;
	u32				root_path_cost;
	unsigned long			max_age;
	unsigned long			hello_time;
	unsigned long			forward_delay;
	unsigned long			bridge_max_age;
	unsigned long			ageing_time;
	unsigned long			mcast_ageing_time;
	unsigned long			bridge_hello_time;
	unsigned long			bridge_forward_delay;

	u8              		group_addr[ETH_ALEN];
	u16				root_port;
	unsigned char			stp_enabled;
	unsigned char			topology_change;
	unsigned char			topology_change_detected;

	struct timer_list		hello_timer;
	struct timer_list		tcn_timer;
	struct timer_list		topology_change_timer;
	struct timer_list		gc_timer;
	struct kobject			*ifobj;

	/* SONOS: Multicast group management */
        unsigned                        num_mcast_groups;
	unsigned char                   mcast_groups[BR_MAX_MCAST_GROUPS][6];
	int                             mcast_advertise_time;
        spinlock_t                      mcast_lock;
	struct timer_list		mcast_timer;
	struct net_bridge_mcast_entry  *mcast_hash[BR_HASH_SIZE];

	/* SONOS: Fixed MAC address */
	unsigned char                   use_static_mac;
	unsigned char                   static_mac[6];	

	/* SONOS: Uplink port */
	unsigned char                   uplink_mode;
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	unsigned char                   proxy_mode;
	u32                             current_ipv4_addr;
	struct timer_list               dupip_timer;
	unsigned long                   dupip_start;
#endif
	struct net_bridge_stats		br_stats;
};

struct br_cb {
	unsigned char direct:1;
#ifdef CONFIG_SONOS_BRIDGE_PROXY
	unsigned char should_proxy_up:1;
	struct net_bridge_port *source_port;
#endif
};

#define BR_SKB_CB(skb)    ((struct br_cb *)(&skb->cb[0]))

extern struct notifier_block br_inetaddr_notifier;
extern struct notifier_block br_device_notifier;
extern const unsigned char bridge_ula[6];
#define br_group_address bridge_ula

/* SONOS: Globals */
extern unsigned char rincon_gmp_addr[6];
extern unsigned char broadcast_addr[6];
extern unsigned char igmp_ah_addr[6];
extern unsigned char igmp_ar_addr[6];
extern unsigned char igmp_amr_addr[6];
extern unsigned char upnp_addr[6];
extern unsigned char mdns_addr[6];

/* Filter functions */
static inline int br_mcast_dest_is_allowed(const unsigned char *addr)
{
    return (0 == memcmp(addr, broadcast_addr,  ETH_ALEN) ||
            0 == memcmp(addr, upnp_addr,       ETH_ALEN) ||
            0 == memcmp(addr, mdns_addr,       ETH_ALEN) ||
            0 == memcmp(addr, igmp_ah_addr,    ETH_ALEN) ||
            0 == memcmp(addr, igmp_ar_addr,    ETH_ALEN) ||
            0 == memcmp(addr, igmp_amr_addr,   ETH_ALEN));
}

static inline int br_mcast_dest_is_allowed_from_local(const unsigned char *addr)
{
    return (0 == memcmp(addr, rincon_gmp_addr,  ETH_ALEN));
}

/* called under bridge lock */
static inline int br_is_root_bridge(const struct net_bridge *br)
{
	return !memcmp(&br->bridge_id, &br->designated_root, 8);
}

static inline void br_stats_init(struct net_bridge *br)
{
	memset(&br->br_stats, 0, sizeof(br->br_stats));
	br->br_stats.rx_start_time = jiffies;
}

/* br_device.c */
extern void br_dev_setup(struct net_device *dev);
extern int br_dev_xmit(struct sk_buff *skb, struct net_device *dev);

/* br_fdb.c */
extern void br_fdb_init(void);
extern void br_fdb_fini(void);
extern void br_fdb_changeaddr(struct net_bridge_port_list_node *pl,
			      const unsigned char *newaddr);
extern void br_fdb_cleanup(unsigned long arg);
extern void br_fdb_delete_by_port(struct net_bridge *br,
			   struct net_bridge_port *p);
extern void br_fdb_delete_non_local(struct net_bridge *br);
extern struct net_bridge_fdb_entry *__br_fdb_get(struct net_bridge *br,
						 const unsigned char *addr);
extern struct net_bridge_fdb_entry *br_fdb_get(struct net_bridge *br,
					       unsigned char *addr);
extern void br_fdb_put(struct net_bridge_fdb_entry *ent);
extern int br_fdb_fillbuf(struct net_bridge *br, void *buf,
			  unsigned long count, unsigned long off);
extern int br_fdb_insert(struct net_bridge *br,
			 struct net_bridge_port *source,
			 const unsigned char *addr);
extern struct net_bridge_fdb_entry *br_fdb_update(struct net_bridge *br,
						  struct net_bridge_port *source,
						  const unsigned char *addr);
extern void br_fdb_update_dst_direct(struct net_bridge *br,
				     struct net_bridge_port *p);

/* br_forward.c */
extern void br_deliver(const struct net_bridge_port *from,
		       const struct net_bridge_port *to,
		       struct sk_buff *skb);
extern void br_deliver_direct(const struct net_bridge_port *from,
                              const struct net_bridge_port *to,
                              struct sk_buff *skb);
extern void br_deliver_bpdu(const struct net_bridge_port *to,
                            struct sk_buff *skb);
extern int br_dev_queue_push_xmit(struct sk_buff *skb);
extern void br_forward_direct(const struct net_bridge_port *from,
			      const struct net_bridge_port *to,
			      struct sk_buff *skb);
extern void br_forward(const struct net_bridge_port *from,
		       const struct net_bridge_port *to,
		       struct sk_buff *skb);
extern int br_forward_finish(struct sk_buff *skb);
extern void br_flood_deliver(struct net_bridge *br,
			     struct net_bridge_port *from,
			     struct sk_buff *skb,
			     int clone);
extern void br_flood_forward(struct net_bridge *br,
			     struct net_bridge_port *from,
			     struct sk_buff *skb,
			     int clone);
extern void __br_forward(const struct net_bridge_port *to,
			 struct sk_buff *skb);
extern void __br_deliver(const struct net_bridge_port *to,
			 struct sk_buff *skb);
extern unsigned char br_is_dhcp(struct sk_buff *skb,
				struct iphdr **oiph,
				struct udphdr **oudph,
				unsigned char **odhcph);
extern void br_log_dhcp(struct udphdr *udph,
			unsigned char *dhcph);

/* br_if.c */
extern int br_add_bridge(const char *name);
extern int br_del_bridge(const char *name);
void br_net_exit(struct net *net) ;
extern int br_add_if(struct net_bridge *br,
	      struct net_device *dev);
extern int br_del_if(struct net_bridge *br,
	      struct net_device *dev);
extern int br_min_mtu(const struct net_bridge *br);
extern void br_features_recompute(struct net_bridge *br);
extern void br_set_static_mac(struct net_bridge *br, unsigned char *mac);

/* br_input.c */
/* SONOS: static now: extern int br_handle_frame_finish(struct net_bridge_port *p, struct sk_buff *skb);*/
extern struct sk_buff *br_handle_frame(struct net_bridge_port_list_node *pl, struct sk_buff *skb);
struct net_bridge_port* br_find_port(const unsigned char *h_source, struct net_bridge_port_list_node *pl);

extern int br_check_sonosnet_serial_match(const unsigned char* sonosnet, const unsigned char* serial);
extern void br_construct_sonosnet_addr(unsigned char* addr_target, unsigned char* serial);

/* REVIEW: Used by br_uplink now, so these can no longer be static.  Ugh... */
extern void br_pass_frame_up(struct net_bridge *br, struct sk_buff *skb);

/* br_ioctl.c */
extern int br_dev_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
extern int br_ioctl_deviceless_stub(struct net *, unsigned int cmd, void __user *arg);

/* br_netfilter.c */
extern int br_netfilter_init(void);
extern void br_netfilter_fini(void);

/* br_stp.c */
extern void br_log_state(const struct net_bridge_port *p);
extern struct net_bridge_port *br_get_port(struct net_bridge *br,
				    	   u16 port_no);
extern void br_init_port(struct net_bridge_port *p);
extern void br_become_designated_port(struct net_bridge_port *p);

/* br_stp_if.c */
extern void br_stp_enable_bridge(struct net_bridge *br);
extern void br_stp_disable_bridge(struct net_bridge *br);
extern void br_stp_enable_port(struct net_bridge_port *p);
extern void br_stp_disable_port(struct net_bridge_port *p);
extern void br_stp_recalculate_bridge_id(struct net_bridge *br);
extern void br_stp_set_bridge_priority(struct net_bridge *br,
				       u16 newprio);
extern void br_stp_set_port_priority(struct net_bridge_port *p,
				     u8 newprio);
extern void br_stp_set_path_cost(struct net_bridge_port *p,
				 u32 path_cost);
extern ssize_t br_show_bridge_id(char *buf, const struct bridge_id *id);

extern int br_stp_mod_port_addr(struct net_bridge *br,
				unsigned char* oldaddr,
                                unsigned char* newaddr);

extern int br_stp_mod_port_dev(struct net_bridge *br,
			       unsigned char* oldaddr,
			       struct net_device *dev);

/* br_stp_bpdu.c */
extern int br_stp_handle_bpdu(struct net_bridge_port *p,
                              struct sk_buff *skb);

/* br_tunnel.c */
extern int br_add_p2p_tunnel(struct net_bridge *br,
                             struct net_device *dev,
                             unsigned char* daddr,
                             struct __add_p2p_entry* ape);

extern int br_set_p2p_tunnel_path_cost(struct net_bridge *br,
                                          struct net_device *dev,
                                          unsigned char* daddr,
                                          int path_cost);

extern int br_set_p2p_direct_enabled(struct net_bridge *br,
				     struct net_device *dev,
				     unsigned char* daddr,
				     int enabled);

int br_set_p2p_direct_addr(struct net_bridge *br,
			   struct net_device *dev,
			   unsigned char* daddr,
			   unsigned char* direct_addr);

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

extern int br_add_uplink(struct net_bridge *br,
			 struct net_device *dev,
			 unsigned char* daddr);

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

extern void br_mcast_update_dst_direct(struct net_bridge *br,
                                       struct net_bridge_port *p);

extern void br_mcast_check(struct sk_buff *skb, 
			   struct net_bridge *br, 
			   struct net_bridge_port *p);
extern int br_mcast_is_management_header(struct ethhdr *ether);

/* br_stp_timer.c */
extern void br_stp_timer_init(struct net_bridge *br);
extern void br_stp_port_timer_init(struct net_bridge_port *p);
extern unsigned long br_timer_value(const struct timer_list *timer);

/* br.c */
extern struct net_bridge_fdb_entry *(*br_fdb_get_hook)(struct net_bridge *br,
						       unsigned char *addr);
extern void (*br_fdb_put_hook)(struct net_bridge_fdb_entry *ent);

/* br_netlink.c */
extern int br_netlink_init(void);
extern void br_netlink_fini(void);
extern void br_ifinfo_notify(int event, struct net_bridge_port *port);
extern void br_dupip_check(struct net_bridge *br);
extern void br_dupip_timer_expired(unsigned long arg);

#if 0 /* def CONFIG_SYSFS */
/* br_sysfs_if.c */
extern struct sysfs_ops brport_sysfs_ops;
extern int br_sysfs_addif(struct net_bridge_port *p);

/* br_sysfs_br.c */
extern int br_sysfs_addbr(struct net_device *dev);
extern void br_sysfs_delbr(struct net_device *dev);

#else

#define br_sysfs_addif(p)   (0) 
#define br_sysfs_addbr(dev) (0) 
#define br_sysfs_delbr(dev) do { } while(0)

#endif /* CONFIG_SYSFS */

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

/* br_priority.c */
extern int br_priority_for_addr(const unsigned char *addr);
extern int br_priority_for_bpdu(void);

/* br_uplink.c */
extern void br_uplink_xmit(struct net_bridge *br,
			   struct sk_buff *skb,
			   const char *dest);
extern void br_uplink_proxy(struct net_bridge *br,
			    struct net_bridge_port *p,
			    struct sk_buff *skb,
			    const char *dest);
extern void br_set_uplink_mode(struct net_bridge *br, int enable);
extern struct sk_buff *br_uplink_handle_frame(struct net_bridge_port *p,
					      struct sk_buff *skb,
					      const unsigned char *src,
					      const unsigned char *dst);
#endif
