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

#ifndef _BR_PRIVATE_H
#define _BR_PRIVATE_H

#include <linux/netdevice.h>
#include <linux/if_bridge.h>
#include <linux/netpoll.h>
#include <linux/u64_stats_sync.h>
#include <net/route.h>
#include <linux/if_vlan.h>

#define BR_HASH_BITS 8
#define BR_HASH_SIZE (1 << BR_HASH_BITS)

#define BR_HOLD_TIME (1*HZ)

/*
 * SONOS: Make BR_PORT_BITS match other products (8 instead of 10).  We don't
 *        need more than this, and it makes it easier to play "what's
 *        different" when debugging Casbah bridge code
 */
#define BR_PORT_BITS	8
#define BR_MAX_PORTS	(1<<BR_PORT_BITS)
#define BR_VLAN_BITMAP_LEN	BITS_TO_LONGS(VLAN_N_VID)

/* DEBUG */
/* #define BR_DEBUG_DIRECT 1 */


#define MAC_ADDR_FMT    "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ADDR_VAR(x) (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5]

#define BR_DIRECT_STP_TIME (10*HZ)

#define BR_VERSION	"6.9"

/* Control of forwarding link local multicast */
#define BR_GROUPFWD_DEFAULT	0
/* Don't allow forwarding control protocols like STP and LLDP */
#define BR_GROUPFWD_RESTRICTED	0x4007u

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

#define BR_BCMC_HIST_SIZE         32
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

struct br_ip
{
	union {
		__be32	ip4;
#if IS_ENABLED(CONFIG_IPV6)
		struct in6_addr ip6;
#endif
	} u;
	__be16		proto;
	__u16		vid;
};

struct net_port_vlans {
	u16				port_idx;
	u16				pvid;
	union {
		struct net_bridge_port		*port;
		struct net_bridge		*br;
	}				parent;
	struct rcu_head			rcu;
	unsigned long			vlan_bitmap[BR_VLAN_BITMAP_LEN];
	unsigned long			untagged_bitmap[BR_VLAN_BITMAP_LEN];
	u16				num_vlans;
};

struct net_bridge_fdb_entry
{
	struct hlist_node		hlist;
	struct net_bridge_port		*dst;

	struct rcu_head			rcu;
	unsigned long			updated;
	unsigned long			used;
	mac_addr			addr;
	unsigned char			is_local;
	unsigned char			is_static;
	__u16				vlan_id;

	struct net_bridge_port		*dst_direct;
	int                             priority;
	atomic_t			use_count;
	unsigned long			ageing_timer;
};

struct net_bridge_port_group {
	struct net_bridge_port		*port;
	struct net_bridge_port_group __rcu *next;
	struct hlist_node		mglist;
	struct rcu_head			rcu;
	struct timer_list		timer;
	struct br_ip			addr;
	unsigned char			state;
};

struct net_bridge_mdb_entry
{
	struct hlist_node		hlist[2];
	struct net_bridge		*br;
	struct net_bridge_port_group __rcu *ports;
	struct rcu_head			rcu;
	struct timer_list		timer;
	struct br_ip			addr;
	bool				mglist;
};

struct net_bridge_mdb_htable
{
	struct hlist_head		*mhash;
	struct rcu_head			rcu;
	struct net_bridge_mdb_htable	*old;
	u32				size;
	u32				max;
	u32				secret;
	u32				ver;
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
	u32				port_no;
	u16				remote_state;
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

	unsigned long 			flags;
#define BR_HAIRPIN_MODE		0x00000001
#define BR_BPDU_GUARD           0x00000002
#define BR_ROOT_BLOCK		0x00000004
#define BR_MULTICAST_FAST_LEAVE	0x00000008
#define BR_ADMIN_COST		0x00000010

#ifdef CONFIG_SYSFS
	char				sysfs_name[IFNAMSIZ];
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
	struct netpoll			*np;
#endif
#ifdef CONFIG_BRIDGE_VLAN_FILTERING
	struct net_port_vlans __rcu	*vlan_info;
#endif
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

#define br_port_exists(dev) (dev->priv_flags & IFF_BRIDGE_PORT)

static inline struct net_bridge_port *br_port_get_rcu(const struct net_device *dev)
{
	return rcu_dereference(dev->rx_handler_data);
}

static inline struct net_bridge_port *br_port_get_rtnl(const struct net_device *dev)
{
	return br_port_exists(dev) ?
		rtnl_dereference(dev->rx_handler_data) : NULL;
}

struct br_cpu_netstats {
	u64			rx_packets;
	u64			rx_bytes;
	u64			tx_packets;
	u64			tx_bytes;
	struct u64_stats_sync	syncp;
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
	struct net_bridge_bcmc_hit bcmc_history[BR_BCMC_HIST_SIZE];
};

struct net_bridge
{
	spinlock_t			lock;
	struct list_head		port_list;
	struct net_device		*dev;
	struct list_head		leaf_list;
	struct net_device_stats		statistics;
	struct list_head		age_list;

	struct br_cpu_netstats __percpu *stats;
	spinlock_t			hash_lock;
	struct hlist_head		hash[BR_HASH_SIZE];
#ifdef CONFIG_BRIDGE_NETFILTER
	struct rtable 			fake_rtable;
	bool				nf_call_iptables;
	bool				nf_call_ip6tables;
	bool				nf_call_arptables;
#endif
	u16				group_fwd_mask;

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

	u8				group_addr[ETH_ALEN];
	u16				root_port;

	enum {
		BR_NO_STP, 		/* no spanning tree */
		BR_KERNEL_STP,		/* old STP in kernel */
		BR_USER_STP,		/* new RSTP in userspace */
	} stp_enabled;

	unsigned char			topology_change;
	unsigned char			topology_change_detected;

	struct timer_list		hello_timer;
	struct timer_list		tcn_timer;
	struct timer_list		topology_change_timer;
	struct timer_list		gc_timer;
	struct kobject			*ifobj;
#ifdef CONFIG_BRIDGE_VLAN_FILTERING
	u8				vlan_enabled;
	struct net_port_vlans __rcu	*vlan_info;
#endif
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
	struct net_bridge_stats         br_stats;
};

struct br_input_skb_cb {
	struct net_device *brdev;
};

#define BR_INPUT_SKB_CB(__skb)	((struct br_input_skb_cb *)(__skb)->cb)

# define BR_INPUT_SKB_CB_MROUTERS_ONLY(__skb)	(0)

#define br_printk(level, br, format, args...)	\
	printk(level "%s: " format, (br)->dev->name, ##args)

#define br_err(__br, format, args...)			\
	br_printk(KERN_ERR, __br, format, ##args)
#define br_warn(__br, format, args...)			\
	br_printk(KERN_WARNING, __br, format, ##args)
#define br_notice(__br, format, args...)		\
	br_printk(KERN_NOTICE, __br, format, ##args)
#define br_info(__br, format, args...)			\
	br_printk(KERN_INFO, __br, format, ##args)

#define br_debug(br, format, args...)			\
	pr_debug("%s: " format,  (br)->dev->name, ##args)

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
extern void br_dev_delete(struct net_device *dev, struct list_head *list);
extern netdev_tx_t br_dev_xmit(struct sk_buff *skb,
			       struct net_device *dev);
#ifdef CONFIG_NET_POLL_CONTROLLER
static inline struct netpoll_info *br_netpoll_info(struct net_bridge *br)
{
	return br->dev->npinfo;
}

static inline void br_netpoll_send_skb(const struct net_bridge_port *p,
				       struct sk_buff *skb)
{
	struct netpoll *np = p->np;

	if (np)
		netpoll_send_skb(np, skb);
}

extern int br_netpoll_enable(struct net_bridge_port *p, gfp_t gfp);
extern void br_netpoll_disable(struct net_bridge_port *p);
#else
static inline struct netpoll_info *br_netpoll_info(struct net_bridge *br)
{
	return NULL;
}

static inline void br_netpoll_send_skb(const struct net_bridge_port *p,
				       struct sk_buff *skb)
{
}

static inline int br_netpoll_enable(struct net_bridge_port *p, gfp_t gfp)
{
	return 0;
}

static inline void br_netpoll_disable(struct net_bridge_port *p)
{
}
#endif

/* br_fdb.c */
extern int br_fdb_init(void);
extern void br_fdb_fini(void);
extern void br_fdb_flush(struct net_bridge *br);
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
extern int br_add_bridge(struct net *net, const char *name);
extern int br_del_bridge(struct net *net, const char *name);
extern void br_net_exit(struct net *net);
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
extern rx_handler_result_t sonos_br_handle_frame(struct sk_buff **pskb);
struct net_bridge_port* br_find_port(const unsigned char *h_source, struct net_bridge_port_list_node *pl);

extern int br_check_sonosnet_serial_match(const unsigned char* sonosnet, const unsigned char* serial);
extern void br_construct_sonosnet_addr(unsigned char* addr_target, unsigned char* serial);

/* REVIEW: Used by br_uplink now, so these can no longer be static.  Ugh... */
extern void br_pass_frame_up(struct net_bridge *br, struct sk_buff *skb);

/* br_ioctl.c */
extern int br_dev_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
extern int br_ioctl_deviceless_stub(struct net *net, unsigned int cmd, void __user *arg);

/* br_multicast.c */
static inline int br_multicast_rcv(struct net_bridge *br,
				   struct net_bridge_port *port,
				   struct sk_buff *skb)
{
	return 0;
}

static inline struct net_bridge_mdb_entry *br_mdb_get(struct net_bridge *br,
						      struct sk_buff *skb, u16 vid)
{
	return NULL;
}

static inline void br_multicast_add_port(struct net_bridge_port *port)
{
}

static inline void br_multicast_del_port(struct net_bridge_port *port)
{
}

static inline void br_multicast_enable_port(struct net_bridge_port *port)
{
}

static inline void br_multicast_disable_port(struct net_bridge_port *port)
{
}

static inline void br_multicast_init(struct net_bridge *br)
{
}

static inline void br_multicast_open(struct net_bridge *br)
{
}

static inline void br_multicast_stop(struct net_bridge *br)
{
}

static inline void br_multicast_deliver(struct net_bridge_mdb_entry *mdst,
					struct sk_buff *skb)
{
}

static inline void br_multicast_forward(struct net_bridge_mdb_entry *mdst,
					struct sk_buff *skb,
					struct sk_buff *skb2)
{
}
static inline bool br_multicast_is_router(struct net_bridge *br)
{
	return 0;
}
static inline void br_mdb_init(void)
{
}
static inline void br_mdb_uninit(void)
{
}

/* br_vlan.c */
#ifdef CONFIG_BRIDGE_VLAN_FILTERING
extern bool br_allowed_ingress(struct net_bridge *br, struct net_port_vlans *v,
			       struct sk_buff *skb, u16 *vid);
extern bool br_allowed_egress(struct net_bridge *br,
			      const struct net_port_vlans *v,
			      const struct sk_buff *skb);
extern struct sk_buff *br_handle_vlan(struct net_bridge *br,
				      const struct net_port_vlans *v,
				      struct sk_buff *skb);
extern int br_vlan_add(struct net_bridge *br, u16 vid, u16 flags);
extern int br_vlan_delete(struct net_bridge *br, u16 vid);
extern void br_vlan_flush(struct net_bridge *br);
extern int br_vlan_filter_toggle(struct net_bridge *br, unsigned long val);
extern int nbp_vlan_add(struct net_bridge_port *port, u16 vid, u16 flags);
extern int nbp_vlan_delete(struct net_bridge_port *port, u16 vid);
extern void nbp_vlan_flush(struct net_bridge_port *port);
extern bool nbp_vlan_find(struct net_bridge_port *port, u16 vid);

static inline struct net_port_vlans *br_get_vlan_info(
						const struct net_bridge *br)
{
	return rcu_dereference_rtnl(br->vlan_info);
}

static inline struct net_port_vlans *nbp_get_vlan_info(
						const struct net_bridge_port *p)
{
	return rcu_dereference_rtnl(p->vlan_info);
}

/* Since bridge now depends on 8021Q module, but the time bridge sees the
 * skb, the vlan tag will always be present if the frame was tagged.
 */
static inline int br_vlan_get_tag(const struct sk_buff *skb, u16 *vid)
{
	int err = 0;

	if (vlan_tx_tag_present(skb))
		*vid = vlan_tx_tag_get(skb) & VLAN_VID_MASK;
	else {
		*vid = 0;
		err = -EINVAL;
	}

	return err;
}

static inline u16 br_get_pvid(const struct net_port_vlans *v)
{
	/* Return just the VID if it is set, or VLAN_N_VID (invalid vid) if
	 * vid wasn't set
	 */
	smp_rmb();
	return (v->pvid & VLAN_TAG_PRESENT) ?
			(v->pvid & ~VLAN_TAG_PRESENT) :
			VLAN_N_VID;
}

#else
static inline bool br_allowed_ingress(struct net_bridge *br,
				      struct net_port_vlans *v,
				      struct sk_buff *skb,
				      u16 *vid)
{
	return true;
}

static inline bool br_allowed_egress(struct net_bridge *br,
				     const struct net_port_vlans *v,
				     const struct sk_buff *skb)
{
	return true;
}

static inline struct sk_buff *br_handle_vlan(struct net_bridge *br,
					     const struct net_port_vlans *v,
					     struct sk_buff *skb)
{
	return skb;
}

static inline int br_vlan_add(struct net_bridge *br, u16 vid, u16 flags)
{
	return -EOPNOTSUPP;
}

static inline int br_vlan_delete(struct net_bridge *br, u16 vid)
{
	return -EOPNOTSUPP;
}

static inline void br_vlan_flush(struct net_bridge *br)
{
}

static inline int nbp_vlan_add(struct net_bridge_port *port, u16 vid, u16 flags)
{
	return -EOPNOTSUPP;
}

static inline int nbp_vlan_delete(struct net_bridge_port *port, u16 vid)
{
	return -EOPNOTSUPP;
}

static inline void nbp_vlan_flush(struct net_bridge_port *port)
{
}

static inline struct net_port_vlans *br_get_vlan_info(
						const struct net_bridge *br)
{
	return NULL;
}
static inline struct net_port_vlans *nbp_get_vlan_info(
						const struct net_bridge_port *p)
{
	return NULL;
}

static inline bool nbp_vlan_find(struct net_bridge_port *port, u16 vid)
{
	return false;
}

static inline u16 br_vlan_get_tag(const struct sk_buff *skb, u16 *tag)
{
	return 0;
}
static inline u16 br_get_pvid(const struct net_port_vlans *v)
{
	return VLAN_N_VID;	/* Returns invalid vid */
}
#endif

/* br_netfilter.c */
#ifdef CONFIG_BRIDGE_NETFILTER
extern int br_netfilter_init(void);
extern void br_netfilter_fini(void);
extern void br_netfilter_rtable_init(struct net_bridge *);
#else
#define br_netfilter_init()	(0)
#define br_netfilter_fini()	do { } while(0)
#define br_netfilter_rtable_init(x)
#endif

/* br_stp.c */
extern void br_log_state(const struct net_bridge_port *p);
extern struct net_bridge_port *br_get_port(struct net_bridge *br,
					   u16 port_no);
extern void br_init_port(struct net_bridge_port *p);
extern void br_become_designated_port(struct net_bridge_port *p);

extern void __br_set_forward_delay(struct net_bridge *br, unsigned long t);
extern int br_set_forward_delay(struct net_bridge *br, unsigned long x);
extern int br_set_hello_time(struct net_bridge *br, unsigned long x);
extern int br_set_max_age(struct net_bridge *br, unsigned long x);


/* br_stp_if.c */
extern void br_stp_enable_bridge(struct net_bridge *br);
extern void br_stp_disable_bridge(struct net_bridge *br);
extern void br_stp_enable_port(struct net_bridge_port *p);
extern void br_stp_disable_port(struct net_bridge_port *p);
extern bool br_stp_recalculate_bridge_id(struct net_bridge *br);
extern void br_stp_set_bridge_priority(struct net_bridge *br,
				       u16 newprio);
extern int br_stp_set_port_priority(struct net_bridge_port *p,
				    unsigned long newprio);
extern int br_stp_set_path_cost(struct net_bridge_port *p,
				unsigned long path_cost);
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
                                                   const unsigned char *addr);
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
#if IS_ENABLED(CONFIG_ATM_LANE)
extern int (*br_fdb_test_addr_hook)(struct net_device *dev, unsigned char *addr);
#endif
extern struct net_bridge_fdb_entry *(*br_fdb_get_hook)(struct net_bridge *br,
						       unsigned char *addr);
extern void (*br_fdb_put_hook)(struct net_bridge_fdb_entry *ent);

/* br_netlink.c */
extern struct rtnl_link_ops br_link_ops;
extern int br_netlink_init(void);
extern void br_netlink_fini(void);
extern void br_ifinfo_notify(int event, struct net_bridge_port *port);
extern void br_dupip_check(struct net_bridge *br);
extern void br_dupip_timer_expired(unsigned long arg);
extern int br_setlink(struct net_device *dev, struct nlmsghdr *nlmsg);
extern int br_dellink(struct net_device *dev, struct nlmsghdr *nlmsg);
extern int br_getlink(struct sk_buff *skb, u32 pid, u32 seq,
		      struct net_device *dev, u32 filter_mask);

#ifdef CONFIG_SYSFS
/* br_sysfs_if.c */
extern const struct sysfs_ops brport_sysfs_ops;
extern int br_sysfs_addif(struct net_bridge_port *p);
extern int br_sysfs_renameif(struct net_bridge_port *p);

/* br_sysfs_br.c */
extern int br_sysfs_addbr(struct net_device *dev);
extern void br_sysfs_delbr(struct net_device *dev);

#else

static inline int br_sysfs_addif(struct net_bridge_port *p) { return 0; }
static inline int br_sysfs_renameif(struct net_bridge_port *p) { return 0; }
static inline int br_sysfs_addbr(struct net_device *dev) { return 0; }
static inline void br_sysfs_delbr(struct net_device *dev) { return; }
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
