#ifndef MESSAGES_H
#define MESSAGES_H

#include <linux/kernel.h>
#include <linux/module.h>

#include "message_queue.h"
#include "rtable.h"

struct rreq
{  
   u_int8_t     type;

#if defined(__BIG_ENDIAN_BITFIELD)
   u_int8_t  j:1;
   u_int8_t  r:1;
   u_int8_t  g:1;
   u_int8_t  d:1;
   u_int8_t  u:1;
   u_int8_t  reserved:3;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
   u_int8_t  reserved:3;
   u_int8_t  u:1;
   u_int8_t  d:1;
   u_int8_t  g:1;
   u_int8_t  r:1;
   u_int8_t  j:1;
#else
#error "Please fix <asm/byteorder.h>"
#endif

   u_int8_t second_reserved;
   u_int8_t     hop_count;
   u_int32_t    rreq_id;
   u_int32_t    dst_ip;
   u_int32_t    dst_seq;
   u_int32_t    src_ip;
   u_int32_t    src_seq;
};

int recv_rreq(struct message_entry *working_packet);
int send_rreq(u_int32_t src, u_int32_t dst_ip);

struct rrep
{
    u_int8_t     type;

#if defined(__BIG_ENDIAN_BITFIELD)
    unsigned int r:1;
    unsigned int a:1;
    unsigned int reserved1:6;
    unsigned int reserved2:3;
    unsigned int prefix_sz:5;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
    unsigned int reserved1:6;
    unsigned int r:1;
    unsigned int a:1;
    unsigned int prefix_sz:5;
    unsigned int reserved2:3;
#else
#error "Please fix <asm/byteorder.h>"
#endif

    u_int8_t     hop_count;
    u_int32_t    dst_ip;
    u_int32_t    dst_seq;
    u_int32_t    src_ip;
    u_int32_t    lifetime;
};

int start_hello (u_int32_t ip);
int recv_rrep(struct message_entry *working_packet);
int send_rrep(u_int32_t src_ip, u_int32_t dst_ip, u_int32_t packet_src_ip,
        u_int32_t dst_seq, int grat_rrep);

/* Route reply acknowledgement */
struct rrep_ack 
{
    u_int8_t type;
    u_int8_t reserved;
};

void send_rrep_ack(u_int32_t dest);
int recv_rrep_ack(struct message_entry *working_packet);

struct rerr_unr_dst
{
    u_int32_t unr_dst_ip;
    u_int32_t unr_dst_seq;
    struct rerr_unr_dst *next;
};

struct rerrhdr
{
    u_int8_t type;
    unsigned int dst_count;
    struct rerr_unr_dst *unr_dst;
};

struct rerr
{
    u_int8_t type;

#if defined(__BIG_ENDIAN_BITFIELD)
    unsigned int n:1;
    unsigned int reserved:15;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
    unsigned int reserved:15;
    unsigned int n:1;
#else
#error "Please fix <asm/byteorder.h>"
#endif

    unsigned int dst_count:8;
};

struct rerrdst
{
    u_int32_t unr_dst_ip;
    u_int32_t unr_dst_seq;
};

int recv_rerr(struct message_entry* working_packet);
void expire_route(struct rtable_entry* entry);
int link_break(u_int32_t ip);
int host_unreachable(u_int32_t ip);

struct mact
{
   u_int8_t     type;

#if defined(__BIG_ENDIAN_BITFIELD)
   u_int8_t  j:1;
   u_int8_t  p:1;
   u_int8_t  g:1;
   u_int8_t  u:1;
   u_int8_t  r:1;
   u_int8_t  reserved1:3;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
   u_int8_t  reserved:3;
   u_int8_t  r:1;
   u_int8_t  u:1;
   u_int8_t  g:1;
   u_int8_t  p:1;
   u_int8_t  j:1;
#else
#error "Please fix <asm/byteorder.h>"
#endif

   u_int8_t     reserved2;
   u_int8_t     hop_count;
   u_int32_t    grp_ip;
   u_int32_t    src_ip;
   u_int32_t    src_seq;
};

void send_mact(u_int32_t dest);
int recv_mact(struct message_entry *working_packet);

struct grph
{
   u_int8_t     type;

#if defined(__BIG_ENDIAN_BITFIELD)
   u_int8_t  u:1;
   u_int8_t  o:1;
   u_int8_t  reserved1:6;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
   u_int8_t  reserved:6;
   u_int8_t  o:1;
   u_int8_t  u:1;
#else
#error "Please fix <asm/byteorder.h>"
#endif

   u_int8_t     reserved2;
   u_int8_t     hop_count;
   u_int32_t    grp_leader_ip;
   u_int32_t    grp_ip;
   u_int32_t    grp_seq;
   u_int32_t    grp_leader_seq;
};

void send_grph(u_int32_t dest);
int recv_grph(struct message_entry *working_packet);

#endif 
