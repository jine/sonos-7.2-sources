/*
 *   Mesh network routing
 *
 *      $Id: mesh_private.h,v 1.4 2004/01/14 17:36:17 vangool Exp $
 *
 */
#ifndef MESH_PRIVATE_H
#define MESH_PRIVATE_H

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <linux/netfilter_ipv4.h>
#include <linux/in.h>
#include <net/route.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

struct mesh_stats
{
   u_int16_t rreq;
   u_int16_t rrep;
   u_int16_t rrer;
   u_int32_t bytes;
   u_int32_t packets;
   u_int32_t routing_packets;
   u_int64_t last_read;
};
extern struct mesh_stats stats;

struct rtable_entry
{
   u_int32_t dst_ip;
   u_int32_t dst_seq;
   u_int8_t hop_count;
   u_int32_t next_hop;
   u_int64_t lifetime;
   struct net_device *dev;
   u_int8_t link;
   u_int8_t route_valid:1;
   u_int8_t route_seq_valid:1;
   u_int32_t rreq_id;
   u_int8_t self_route:1;
   struct rtable_entry *prev;
   struct rtable_entry *next;
};

/* misc.c */
u_int64_t getcurrtime(void);

/* rtable.c */
int route_table_init(void);
int route_table_deinit(void);

/* utils.c */
u32 in_aton(const char *str);

#endif
