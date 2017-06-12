#ifndef RTABLE_H
#define RTABLE_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>

struct precursor_entry
{
    u_int32_t ip;
    struct precursor_entry *next;
    struct precursor_entry *prev;
};

struct rtable_entry
{
    u_int32_t dst_ip;
    u_int32_t dst_seq;
    u_int8_t hop_count;
    u_int32_t next_hop;
    struct precursor_entry *precursors;
    u_int64_t lifetime;
    struct net_device *dev;
    u_int8_t link;
    u_int8_t     route_valid:1;
    u_int8_t     route_seq_valid:1;
    struct rtable_entry *next;
    struct rtable_entry *prev;
    u_int32_t rreq_id;
    u_int8_t static_route:1;
};

struct rtable_entry *rtable_head(void);
struct rtable_entry *create_route_table_entry(void);
struct rtable_entry *find_route_table_entry(u_int32_t tmp_ip);
struct rtable_entry *fast_find_route_table_entry(u_int32_t tmp_ip);
int delete_route_table_entry(u_int32_t tmp_ip);
void cleanup_route_table(void);
void remove_inactive_routes(void);
int add_precursor(struct rtable_entry *tmp_entry, u_int32_t tmp_ip);
void delete_precursor_entry(struct rtable_entry *tmp_entry, u_int32_t tmp_ip);
void delete_precursor_from_routes(u_int32_t tmp_ip);
struct precursor_entry* find_precursor(struct rtable_entry *tmp_entry,
        u_int32_t tmp_ip);
int update_route_entry(u_int32_t ip, u_int32_t next_hop_ip, u_int8_t hop_count,
        u_int32_t seq, struct net_device *dev);
void delete_precursors_from_route(struct rtable_entry *tmp_entry);
struct rtentry *create_kernel_route_entry(u_int32_t dst_ip, u_int32_t gw_ip,
        char *interf);

int init_route_table( void);
void reset_route_table_link(void);

int add_kroute(u_int32_t dst_ip, u_int32_t gw_ip, char *interface);
int delete_kroute(u_int32_t dst_ip, u_int32_t gw_ip);

void add_static_route(char* ip, struct net_device *dev);

#endif
