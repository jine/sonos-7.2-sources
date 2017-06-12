/*
 *	Forwarding database
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: br_fdb.c,v 1.1.1.1 2006/12/23 00:48:21 holmgren Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/if_bridge.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include "br_private.h"

static __inline__ unsigned long __timeout(struct net_bridge *br)
{
	unsigned long timeout;

	if (br->topology_change)
		timeout = jiffies - br->forward_delay;
        else
		timeout = jiffies - br->ageing_time;

	return timeout;
}

static __inline__ int has_expired(struct net_bridge *br,
				  struct net_bridge_fdb_entry *fdb)
{
	if (!fdb->is_static &&
	    time_before_eq(fdb->ageing_timer, __timeout(br)))
		return 1;

	return 0;
}

static __inline__ void copy_fdb(struct __fdb_entry *ent, 
                                struct net_bridge_fdb_entry *f)
{
	memset(ent, 0, sizeof(struct __fdb_entry));
	memcpy(ent->mac_addr, f->addr.addr, ETH_ALEN);
	ent->port_no = f->dst?f->dst->port_no:0;
	ent->is_local = f->is_local;
	ent->ageing_timer_value = 0;
	if (!f->is_static)
		ent->ageing_timer_value = jiffies - f->ageing_timer;
}

static __inline__ int br_mac_hash(unsigned char *mac)
{
	unsigned long x;

	x = mac[0];
	x = (x << 2) ^ mac[1];
	x = (x << 2) ^ mac[2];
	x = (x << 2) ^ mac[3];
	x = (x << 2) ^ mac[4];
	x = (x << 2) ^ mac[5];

	x ^= x >> 8;

	return x & (BR_HASH_SIZE - 1);
}

static __inline__ void __hash_link(struct net_bridge *br,
				   struct net_bridge_fdb_entry *ent,
				   int hash)
{
	ent->next_hash = br->hash[hash];
	if (ent->next_hash != NULL)
		ent->next_hash->pprev_hash = &ent->next_hash;
	br->hash[hash] = ent;
	ent->pprev_hash = &br->hash[hash];
}

static __inline__ void __hash_unlink(struct net_bridge_fdb_entry *ent)
{
	*(ent->pprev_hash) = ent->next_hash;
	if (ent->next_hash != NULL)
		ent->next_hash->pprev_hash = ent->pprev_hash;
	ent->next_hash = NULL;
	ent->pprev_hash = NULL;
}



void br_fdb_changeaddr(struct net_bridge_port_list_node *pl, 
                       unsigned char *newaddr)
{
	struct net_bridge *br;
	int i;

	br = pl->port->br;
	write_lock_bh(&br->hash_lock);
	for (i=0;i<BR_HASH_SIZE;i++) {
		struct net_bridge_fdb_entry *f;

		f = br->hash[i];
		while (f != NULL) {
                        struct net_bridge_port_list_node* pl_curr = pl;
                        while (pl_curr != NULL) {
                            if (f->dst == pl_curr->port && f->is_local) {
				    __hash_unlink(f);
                                    memcpy(f->addr.addr, newaddr, ETH_ALEN);
                                    __hash_link(br, f, br_mac_hash(newaddr));
                                    write_unlock_bh(&br->hash_lock);
                                    return;
                            }
                            pl_curr = pl_curr->next;
                        }
                        f = f->next_hash;
                }
	}
	write_unlock_bh(&br->hash_lock);
}

void br_fdb_cleanup(struct net_bridge *br)
{
	int i;
	unsigned long timeout;

	timeout = __timeout(br);

	write_lock_bh(&br->hash_lock);
	for (i=0;i<BR_HASH_SIZE;i++) {
		struct net_bridge_fdb_entry *f;

		f = br->hash[i];
		while (f != NULL) {
			struct net_bridge_fdb_entry *g;

			g = f->next_hash;
			if (!f->is_static &&
			    time_before_eq(f->ageing_timer, timeout)) {
				__hash_unlink(f);
				br_fdb_put(f);
			}
			f = g;
		}
	}	
	spin_unlock_bh(&br->hash_lock);
}

/* Call holding br->lock */
void br_fdb_update_dst_direct(struct net_bridge *br, struct net_bridge_port *p)
{
	int i;

	/* REVIEW: Could we just look up p->direct_addr in the FDB and update
	 *         that single entry?  Oh well, this works and is rarely
	 *         called.
	 */
	spin_lock_bh(&br->hash_lock);
	for (i = 0; i < BR_HASH_SIZE; i++) {

		struct net_bridge_fdb_entry *f;
		f = br->hash[i];

		while (f != NULL) {
			struct net_bridge_fdb_entry *g;
			
			g = f->next_hash;

			if (0 == f->dst_direct &&
			    0 ==  memcmp(f->addr.addr, p->direct_addr, ETH_ALEN)) {
				printk("%s: updated FDB (port=%d)\n",
				       __func__, p->port_no);
				f->dst_direct = p;				
			}
			
			f = g;
		}
	}
	write_unlock_bh(&br->hash_lock);
}

/* Call holding br->lock */
void br_fdb_delete_by_port(struct net_bridge *br, struct net_bridge_port *p)
{
	int i;

	write_lock_bh(&br->hash_lock);
	for (i=0;i<BR_HASH_SIZE;i++) {
		struct net_bridge_fdb_entry *f;

		f = br->hash[i];
		while (f != NULL) {
			struct net_bridge_fdb_entry *g;
						
			g = f->next_hash;
			
			/* REVIEW: Delete if there is any way to route to the
			 *         port were dumping.  If we deleted a port it
			 *         is for good reason, but if we're wrong and
			 *         it is still accessible (say via ethernet),
			 *         we'll relearn it in a bit.
			 *
			 *         This should be just as safe as deleting a
			 *         port in any normal case (same locking,
			 *         etc).
			 */
			if (f->dst == p || f->dst_direct == p) {
				__hash_unlink(f);
				br_fdb_put(f);
			}
			f = g;
		}
	}
	write_unlock_bh(&br->hash_lock);
}

struct net_bridge_fdb_entry *br_fdb_get(struct net_bridge *br, 
                                        unsigned char *addr)
{
	struct net_bridge_fdb_entry *fdb;

	read_lock_bh(&br->hash_lock);
	fdb = br->hash[br_mac_hash(addr)];
	while (fdb != NULL) {
		if (!memcmp(fdb->addr.addr, addr, ETH_ALEN)) {
			if (!has_expired(br, fdb)) {
				atomic_inc(&fdb->use_count);
				read_unlock_bh(&br->hash_lock);
				return fdb;
			}

			read_unlock_bh(&br->hash_lock);
			return NULL;
		}

		fdb = fdb->next_hash;
	}

	read_unlock_bh(&br->hash_lock);
	return NULL;
}

void br_fdb_put(struct net_bridge_fdb_entry *ent)
{
	if (atomic_dec_and_test(&ent->use_count))
		kfree(ent);
}

int br_fdb_get_entries(struct net_bridge *br,
		       unsigned char *_buf,
		       int maxnum,
		       int offset)
{
	int i;
	int num;
	struct __fdb_entry *walk;

	num = 0;
	walk = (struct __fdb_entry *)_buf;

	read_lock_bh(&br->hash_lock);
	for (i=0;i<BR_HASH_SIZE;i++) {
		struct net_bridge_fdb_entry *f;

		f = br->hash[i];
		while (f != NULL && num < maxnum) {
			struct __fdb_entry ent;
			int err;
			struct net_bridge_fdb_entry *g;
			struct net_bridge_fdb_entry **pp; 

			if (has_expired(br, f)) {
				f = f->next_hash;
				continue;
			}

			if (offset) {
				offset--;
				f = f->next_hash;
				continue;
			}

			copy_fdb(&ent, f);

			atomic_inc(&f->use_count);
			read_unlock_bh(&br->hash_lock);
			err = copy_to_user(walk, &ent, 
                                           sizeof(struct __fdb_entry));
			read_lock_bh(&br->hash_lock);

			g = f->next_hash;
			pp = f->pprev_hash;
			br_fdb_put(f);

			if (err)
				goto out_fault;

			if (g == NULL && pp == NULL)
				goto out_disappeared;

			num++;
			walk++;

			f = g;
		}
	}

 out:
	read_unlock_bh(&br->hash_lock);
	return num;

 out_disappeared:
	num = -EAGAIN;
	goto out;

 out_fault:
	num = -EFAULT;
	goto out;
}

static __inline__ void __fdb_possibly_replace(struct net_bridge_fdb_entry *fdb,
					      struct net_bridge_port *source,
					      int is_local)
{
	if (!fdb->is_static || is_local) {
		fdb->dst = source;
		fdb->is_local = is_local;
		fdb->is_static = is_local;
		fdb->ageing_timer = jiffies;
	}
}


/* REVIEW: Move to br_tunnel.c? */
/* Call holding br->lock (at least for read) */
static struct net_bridge_port *_get_direct_port(const unsigned char *addr,
						struct net_bridge_port* p)	
{
        while (p) {
		if (0 == memcmp(addr, p->direct_addr, 6)) {
			// printk("direct port: %d (%02x:%02x:%02x)\n",
			// p->port_no, addr[3], addr[4], addr[5]);
			return (p);
		}
		p = p->next;
	}

	return 0;
}

/* Call holding br->lock (at least for read) */
struct net_bridge_fdb_entry *br_fdb_insert(struct net_bridge *br,
					   struct net_bridge_port *source,
					   unsigned char *addr,
					   int is_local)
{
	struct net_bridge_fdb_entry *fdb;
	int hash;

	hash = br_mac_hash(addr);

	write_lock_bh(&br->hash_lock);

	/* Try to update the entry */
	fdb = br->hash[hash];
	while (fdb != NULL) {
		if (!fdb->is_local &&
		    !memcmp(fdb->addr.addr, addr, ETH_ALEN)) {
			__fdb_possibly_replace(fdb, source, is_local);
			write_unlock_bh(&br->hash_lock);
			return fdb;
		}

		fdb = fdb->next_hash;
	}

	/* New copy entirely */
	fdb = kmalloc(sizeof(*fdb), GFP_ATOMIC);
	if (fdb == NULL) {
		write_unlock_bh(&br->hash_lock);
		return NULL;
	}

	memcpy(fdb->addr.addr, addr, ETH_ALEN);
	atomic_set(&fdb->use_count, 1);
	fdb->dst = source;
	fdb->is_local = is_local;
	fdb->is_static = is_local;
	fdb->ageing_timer = jiffies;

	fdb->dst_direct = _get_direct_port(addr, br->port_list);

        fdb->priority = br_priority_for_addr(addr);

	__hash_link(br, fdb, hash);

	write_unlock_bh(&br->hash_lock);
	return fdb;
}
