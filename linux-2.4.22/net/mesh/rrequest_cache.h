/*
 *   Mesh network routing
 *
 *      $Id: rrequest_cache.h,v 1.2 2004/01/14 17:36:17 vangool Exp $
 *
 */
#ifndef RREQUEST_CACHE_H
#define RREQUEST_CACHE_H

#include <linux/kernel.h>
#include <linux/module.h>

/* Structure to keep track of an outstanding RREQ */
struct rrequest_cache_entry
{
   /* A per-host unique identifier for this entry (part of the RREQ message) */
   u_int32_t id;
   /* Source IP of the request */
   u_int32_t src_ip;
   /* Destination IP of the request */
   u_int32_t dst_ip;
   /* Lifetime of the request. After expiring the entry will be GC-ed */
   u_int64_t lifetime;
   /* Pointer to the next entry in the list */
   struct rrequest_cache_entry *next;
};

/* Initialize the cache */
int init_rrequest_cache(void);

/* Cleanup the cache */
void cleanup_rrequest_cache(void);

/* Insert a new RREQ entry */
int insert_rreq(u_int32_t src_ip, u_int32_t dst_ip, u_int32_t id,
      u_int64_t lifetime);

/* Find a RREQ */
struct rrequest_cache_entry *find_rreq(u_int32_t src_ip, u_int32_t id);

/* Cleanup old requests that have expired */
int delete_old_rreqs(void);

#endif
