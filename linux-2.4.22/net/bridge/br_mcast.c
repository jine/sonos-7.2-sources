/*
 *	Multicast Handling
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *      Nick Millington                 <nickmillington@msn.com>
 *
 *	$Id: br_mcast.c,v 1.2 2004/08/10 03:46:10 millington Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/if_bridge.h>
#include <linux/brlock.h>
#include <asm/uaccess.h>
#include "br_private.h"

static __inline__ int br_mac_hash(const unsigned char *mac)
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

void br_mcast_put(struct net_bridge_mcast_entry *me)
{
    if (atomic_dec_and_test(&me->use_count)) {
        /* free the list of destination ports */
        while (me->rx_port_list) {
            struct net_bridge_mcast_rx_port *mrxp = me->rx_port_list;

            /* free the list of destination MAC addresses */
            while (mrxp->rx_mac_list) {
                struct net_bridge_mcast_rx_mac *mrxm = mrxp->rx_mac_list;
                mrxp->rx_mac_list = mrxp->rx_mac_list->next;
                kfree(mrxm);
            }

            me->rx_port_list = me->rx_port_list->next;
            kfree(mrxp);
        }

        kfree(me);
    }
}

struct net_bridge_mcast_entry *br_mcast_get(struct net_bridge *br, 
                                            unsigned char *addr)
{
    struct net_bridge_mcast_entry *me = 0;

    read_lock_bh(&br->mcast_lock);

    me = br->mcast_hash[br_mac_hash(addr)];
    while (me != NULL) {
        if (0 == memcmp(me->addr, addr, 6)) {
            atomic_inc(&me->use_count);
            break;
        }

        me = me->next_hash;
    }

    read_unlock_bh(&br->mcast_lock);

    return me;
}

/* called under bridge lock */
void br_mcast_transmit_grouplist(struct net_bridge *br)
{
    struct sk_buff *skb;

    if (br->num_mcast_groups == 0)
        return;

    unsigned short size = 2 * ETH_ALEN + 2 + 1 + 6 * br->num_mcast_groups;
    if (size < 60)
        size = 60;

    const unsigned short nbe_protocol = htons(BR_MCAST_GL_PROTOCOLNUM);

    if ((skb = dev_alloc_skb(size)) == NULL) {
        printk(KERN_INFO "br: memory squeeze!\n");
        return;
    }

    /* generate the MAC header */
    skb->dev = &br->dev;
    skb->protocol = nbe_protocol;
    skb->mac.raw = skb_put(skb, size);
    memcpy(skb->mac.raw, rincon_gmp_addr, ETH_ALEN);
    memcpy(skb->mac.raw + ETH_ALEN, br->dev.dev_addr, ETH_ALEN);
    memcpy(&skb->mac.raw[2 * ETH_ALEN], &nbe_protocol, 2);

    /* generate the data payload */
    skb->nh.raw = skb->mac.raw + 2 * ETH_ALEN + 2;
    
    unsigned char* data = skb->nh.raw;
    *(data++) = (unsigned char)(br->num_mcast_groups);

    int ix;
    for (ix = 0; ix < br->num_mcast_groups; ix++) {
        memcpy(data, br->mcast_groups[ix], 6);
        data += 6;
    }

    /* pad the rest of the packet */
    int length = (data - skb->mac.raw);
    if (length < skb->len)
        memset(data, 0xA5, skb->len - length);

    /* deliver the packet */
    br_mcast_handle_grouplist(br, 0, skb);
    skb_pull(skb, ETH_HLEN);
    if (br->uplink_mode)
        br_uplink_xmit(br, skb, rincon_gmp_addr);
    else
        br_flood_deliver(br, 0, skb, 0);
}

/* Call holding br_lock */
static inline
struct net_bridge_port *_get_direct_port(const unsigned char *addr,
					 struct net_bridge_port* p)	
{
        while (p) {
		if (0 == memcmp(addr, p->direct_addr, 6)) {
			break;
		}
		p = p->next;
	}

#ifdef BR_DEBUG_DIRECT
	printk("mc: direct_dst for %02x:%02x:%02x is %p\n",
	       addr[3], addr[4], addr[5],
	       p);
#endif

	return p;
}

void br_mcast_handle_grouplist(struct net_bridge *br, 
                               struct net_bridge_port *source,
                               struct sk_buff *skb)
{
    /* the packet has already been validated to be of the correct ether type
       and to the correct destination group; pull out the list of MAC
       multicast groups of which the device wants to be a member, and update
       the forwarding table accordingly */

    const unsigned char *data = skb->mac.raw + 2 * ETH_ALEN + 2;
    unsigned int num_addrs = *(data++);
    unsigned int real_len = skb->len - (2 * ETH_ALEN + 2 + 1);
    unsigned int ix;
    if (real_len < num_addrs * ETH_ALEN)
        return;

    write_lock_bh(&br->mcast_lock);

    for (ix = 0; ix < num_addrs; ix++) {
        const unsigned char* maddr = &data[ix * ETH_ALEN];

        /* reject entries that are not multicast groups */
        if (!maddr[0] & 1)
            continue;

        /* reject certain 'system' multicast groups */
        if (0 == memcmp(igmp_ah_addr, maddr, ETH_ALEN) ||
            0 == memcmp(igmp_ar_addr, maddr, ETH_ALEN) ||
            0 == memcmp(igmp_amr_addr, maddr, ETH_ALEN) ||
            0 == memcmp(broadcast_addr, maddr, ETH_ALEN) ||
            0 == memcmp(rincon_gmp_addr, maddr, ETH_ALEN) ||
            0 == memcmp(mdns_addr, maddr, ETH_ALEN) ||
            0 == memcmp(upnp_addr, maddr, ETH_ALEN))
            continue;

        /* figure out if this entry already exists in the table */
        int hash = br_mac_hash(maddr);
        struct net_bridge_mcast_entry *me = br->mcast_hash[hash];
        while (me != NULL) {
            if (0 == memcmp(me->addr, maddr, ETH_ALEN))
                break;

            me = me->next_hash;
        }

        /* if an entry was found, check to see if it already has a listing
           for our port and MAC address */
        if (me) {
            struct net_bridge_mcast_rx_port *mrxp;
            for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next)
                if (mrxp->dst == source)
                    break;

            if (mrxp) {
                /* There's already an entry for this port; check to see if 
                   this MAC address also appears. */
                struct net_bridge_mcast_rx_mac *mrxm;
                for (mrxm = mrxp->rx_mac_list; mrxm != NULL; mrxm = mrxm->next)
                    if (0 == memcmp(skb->mac.ethernet->h_source,
                                    mrxm->addr, ETH_ALEN))
                        break;
                
                if (mrxm) {
                    /* Nothing has changed; just update ageing timer */
                    mrxm->ageing_timer = jiffies;
                    continue;
                }
            }
        }

        /* we're going to need an update */
        struct net_bridge_mcast_entry *me_new = kmalloc(sizeof(*me_new),
                                                        GFP_ATOMIC);
        if (!me_new)
            continue;

        memcpy(me_new->addr, maddr, ETH_ALEN);
        me_new->rx_port_list = 0;
        atomic_set(&me_new->use_count, 1);

        /* clone each entry in the receiving port list */
        struct net_bridge_mcast_rx_port *mrxp_update = 0;
        if (me) {
            struct net_bridge_mcast_rx_port *mrxp;
            for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {
                struct net_bridge_mcast_rx_port *mrxp_new;
                mrxp_new = kmalloc(sizeof(*mrxp_new), GFP_ATOMIC);
                if (!mrxp_new)
                    break;
                
                mrxp_new->dst = mrxp->dst;
                mrxp_new->rx_mac_list = 0;

                if (mrxp_new->dst == source)
                    mrxp_update = mrxp_new;

                /* clone each entry in the receiving port's MAC address list */
                struct net_bridge_mcast_rx_mac *mrxm;
                for (mrxm = mrxp->rx_mac_list; 
                     mrxm != NULL; 
                     mrxm = mrxm->next) {
                    struct net_bridge_mcast_rx_mac *mrxm_new;
                    mrxm_new = kmalloc(sizeof(*mrxm_new), GFP_ATOMIC);
                    if (!mrxm_new)
                        break;
                    memcpy(mrxm_new->addr, mrxm->addr, ETH_ALEN);
		    mrxm_new->direct_dst = mrxm->direct_dst;
                    mrxm_new->ageing_timer = mrxm->ageing_timer;

                    mrxm_new->next = mrxp_new->rx_mac_list;
                    mrxp_new->rx_mac_list = mrxm_new;
                }

                mrxp_new->next = me_new->rx_port_list;
                me_new->rx_port_list = mrxp_new;
            }
        }

        if (!mrxp_update) {
            /* this is the first entry for this port */
            mrxp_update = kmalloc(sizeof(*mrxp_update), GFP_ATOMIC);
            if (mrxp_update) {
                mrxp_update->dst = source;
                mrxp_update->rx_mac_list = 0;
                mrxp_update->next = me_new->rx_port_list;
                me_new->rx_port_list = mrxp_update;
            }
        }

        if (mrxp_update) {
            /* create an entry for the MAC address */
            struct net_bridge_mcast_rx_mac *mrxm_new;
            mrxm_new = kmalloc(sizeof(*mrxm_new), GFP_ATOMIC);
            if (mrxm_new) {
                memcpy(mrxm_new->addr, skb->mac.ethernet->h_source, ETH_ALEN);
                mrxm_new->ageing_timer = jiffies;
                mrxm_new->direct_dst = _get_direct_port(mrxm_new->addr, br->port_list);
#ifdef BR_DEBUG_DIRECT
		printk("mc: new: %p(%d) %p\n",
		       mrxp_update->dst,
		       mrxp_update->dst ? mrxp_update->dst->port_no : -1,
		       mrxm_new->direct_dst);
#endif
                mrxm_new->next = mrxp_update->rx_mac_list;
                mrxp_update->rx_mac_list = mrxm_new;
            }
        }

        /* do the swap */
        if (me) {
            me_new->next_hash = me->next_hash;
            me_new->prev_hash = me->prev_hash;
            if (me_new->prev_hash)
                me_new->prev_hash->next_hash = me_new;
            else
                br->mcast_hash[hash] = me_new;
            if (me_new->next_hash)
                me_new->next_hash->prev_hash = me_new;
            br_mcast_put(me);
        } else {
            me_new->prev_hash = 0;
            me_new->next_hash = br->mcast_hash[hash];
            if (br->mcast_hash[hash])
                br->mcast_hash[hash]->prev_hash = me_new;
            br->mcast_hash[hash] = me_new;
        }
    }

    write_unlock_bh(&br->mcast_lock);
}

void br_mcast_update_dst_direct(struct net_bridge *br,
				struct net_bridge_port *p)
{
    /*
     * Walk the entire table looking for this MAC address. This only right
     * after a port is added, and ports don't generally flap (or at least they
     * should not), so this should not happen often.
     *
     * The only other oddd case is if direct_dst moves from one port to
     * another.  This shouldn't happen as it should require the port to be
     * deleted first, which should cause direct_dst to be NULL below.
     */
    spin_lock_bh(&br->mcast_lock);

    unsigned int ix;
    for (ix = 0; ix < BR_HASH_SIZE; ix++) {

        struct net_bridge_mcast_entry *me;
        for (me = br->mcast_hash[ix]; me; me = me->next_hash) {

            struct net_bridge_mcast_rx_port *mrxp;
            for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {

                struct net_bridge_mcast_rx_mac *mrxm;
                for (mrxm = mrxp->rx_mac_list; 
                     mrxm != NULL; 
                     mrxm = mrxm->next) {

                    /* If this MAC doesn't already have a direct_dst, set it */
                    if (!mrxm->direct_dst && 
                        !memcmp(mrxm->addr, p->direct_addr, 6)) {
                        
                        mrxm->direct_dst = p;
                    }
                }
		
                /* The DA may be in multiple multicast groups, so we
                 * can not exit here.  It will generally only be in one, but
                 * there are cases where it can be in many depending on what
                 * the user is up to.
                 */
            }
        }
    }

    spin_unlock_bh(&br->mcast_lock);
}

static void 
__br_mcast_delete_if(struct net_bridge *br,
                     int (*__predicate)(struct net_bridge_mcast_rx_port *mrxp,
                                        struct net_bridge_mcast_rx_mac *mrxm,
                                        void *pv),
                     void *pv,
                     struct net_bridge_port *deleted_port)
{
     write_lock_bh(&br->mcast_lock);

     unsigned int ix;
     for (ix = 0; ix < BR_HASH_SIZE; ix++) {
         struct net_bridge_mcast_entry *me = br->mcast_hash[ix];
         while (me != NULL) {
             /* iterate through the ports associated with this multicast
                address; do an up-front check for any deletes */
             unsigned int num_deletes = 0;
             unsigned int num_total = 0;
             struct net_bridge_mcast_rx_port *mrxp;
             for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {
                 struct net_bridge_mcast_rx_mac *mrxm;
                 for (mrxm = mrxp->rx_mac_list; 
                      mrxm != NULL; 
                      mrxm = mrxm->next) {
                     if (__predicate(mrxp, mrxm, pv))
                         num_deletes++;

                     num_total++;

                     /* Also adjust direct_dst here if it is going away,
                      * regardless of the delete predicate.  Could have handled
                      * this in update_dst_direct above, but this seems a
                      * little cleaner since we only have to call this function
                      * for a delete instead of this one and
                      * update_dst_direct.
                      */
                     if (deleted_port) {
                         if (mrxm->direct_dst == deleted_port) {
                             mrxm->direct_dst = NULL;
                         }
                     }
                 }
             }

             if (num_deletes >= num_total) {
                 /* the entire record is going away */
                 if (me->prev_hash)
                     me->prev_hash->next_hash = me->next_hash;
                 else
                     br->mcast_hash[ix] = me->next_hash;
                 if (me->next_hash)
                     me->next_hash->prev_hash = me->prev_hash;

                 struct net_bridge_mcast_entry *tmp = me->next_hash;
                 br_mcast_put(me);
                 me = tmp;
             } else if (num_deletes != 0) {
                 /* some subset of the record is going away */
                 struct net_bridge_mcast_entry *me_new = 
                     kmalloc(sizeof(*me_new), GFP_ATOMIC);

                 if (!me_new) {
                     me = me->next_hash;
                     continue;
                 }

                 /* set up the replacement record */
                 memcpy(me_new->addr, me->addr, ETH_ALEN);
                 me_new->rx_port_list = 0;
                 atomic_set(&me_new->use_count, 1);

                 /* clone each entry in the receiving port list */
                 for (mrxp = me->rx_port_list; 
                      mrxp != NULL; 
                      mrxp = mrxp->next) {
                     struct net_bridge_mcast_rx_port *mrxp_new;
                     mrxp_new = kmalloc(sizeof(*mrxp_new), GFP_ATOMIC);
                     if (!mrxp_new)
                         continue;

                     mrxp_new->dst = mrxp->dst;
                     mrxp_new->rx_mac_list = 0;
                         
                     /* clone each entry in the MAC address list */
                     struct net_bridge_mcast_rx_mac *mrxm;
                     for (mrxm = mrxp->rx_mac_list;
                          mrxm != NULL;
                          mrxm = mrxm->next) {
                         if (!__predicate(mrxp, mrxm, pv)) {
                             struct net_bridge_mcast_rx_mac *mrxm_new;
                             mrxm_new = kmalloc(sizeof(*mrxm_new), GFP_ATOMIC);
                             if (!mrxm_new)
                                 continue;

                             memcpy(mrxm_new->addr, mrxm->addr, ETH_ALEN);
			     mrxm_new->direct_dst = mrxm->direct_dst;
                             mrxm_new->ageing_timer = mrxm->ageing_timer;

                             mrxm_new->next = mrxp_new->rx_mac_list;
                             mrxp_new->rx_mac_list = mrxm_new;
                         }
                     }
                     
                     /* if there are none left, don't bother */
                     if (mrxp_new->rx_mac_list) {
                         mrxp_new->next = me_new->rx_port_list;
                         me_new->rx_port_list = mrxp_new;
                     } else
                         kfree(mrxp_new);
                 }

                 /* if it's empty, don't bother.  */
                 if (me_new->rx_port_list) {
                     /* replace the entry with the updated one */
                     me_new->next_hash = me->next_hash;
                     me_new->prev_hash = me->prev_hash;
                     if (me_new->prev_hash)
                         me_new->prev_hash->next_hash = me_new;
                     else
                         br->mcast_hash[ix] = me_new;
                     if (me_new->next_hash)
                         me_new->next_hash->prev_hash = me_new;
                     br_mcast_put(me);
                     me = me_new->next_hash;
                 } else {
                     kfree(me_new);

                     /* the entire record is going away */
                     if (me->prev_hash)
                         me->prev_hash->next_hash = me->next_hash;
                     else
                         br->mcast_hash[ix] = me->next_hash;
                     if (me->next_hash)
                         me->next_hash->prev_hash = me->prev_hash;

                     struct net_bridge_mcast_entry *tmp = me->next_hash;
                     br_mcast_put(me);
                     me = tmp;                     
                 }
             } else
                 me = me->next_hash;
         }
     }

     write_unlock_bh(&br->mcast_lock);
}

static int __age_list_predicate(struct net_bridge_mcast_rx_port *mrxp,
                                struct net_bridge_mcast_rx_mac *mrxm,
                                void *pv)
{
    struct net_bridge *br = (struct net_bridge *)pv;

    return ((jiffies - mrxm->ageing_timer) > br->mcast_ageing_time);
}

void br_mcast_age_list(struct net_bridge *br)
{
    __br_mcast_delete_if(br, __age_list_predicate, br, 0);
}

static int __destroy_list_predicate(struct net_bridge_mcast_rx_port *mrxp,
                                    struct net_bridge_mcast_rx_mac *mrxm,
                                    void *pv)
{
    return 1;
}

void br_mcast_destroy_list(struct net_bridge *br)
{
    __br_mcast_delete_if(br, __destroy_list_predicate, 0, 0);
}

static int __delete_by_port_predicate(struct net_bridge_mcast_rx_port *mrxp,
                                      struct net_bridge_mcast_rx_mac *mrxm,
                                      void *pv)
{
    struct net_bridge_port *p = (struct net_bridge_port *)pv;

    return (p == mrxp->dst);
}

void br_mcast_delete_by_port(struct net_bridge *br, 
                             struct net_bridge_port *p)
{
    __br_mcast_delete_if(br, __delete_by_port_predicate, p, p);
}

/* We return the multicast table data to user mode in the following manner:
    - return value from this function: number of records
    - each record:
         Multicast MAC address [6 bytes]
         Number of ports [4 bytes]
         - each port:
             port number [4 bytes]
             number of members [4 bytes]
             -each member:
                 MAC address [6 bytes]
                 seconds until expiration: [4 bytes]
*/
int br_mcast_fdb_get_entries(struct net_bridge *br,
                             unsigned char *buf,
                             unsigned int buf_len,
                             int offset)
{
    read_lock_bh(&br->mcast_lock);

    int num_returned = 0;
    int ret = 0;
    unsigned int ix;

    for (ix = 0; ix < BR_HASH_SIZE; ix++) {
         struct net_bridge_mcast_entry *me = br->mcast_hash[ix];
         while (me != NULL) {
             if (offset > 0) {
                 offset--;
                 me = me->next_hash;
                 continue;
             }

             /* calculate how much space it will take to return this record */
             unsigned int bytes_needed = 6 + 4;
             unsigned int num_ports = 0;

             struct net_bridge_mcast_rx_port *mrxp;
             for (mrxp = me->rx_port_list; mrxp != NULL; mrxp = mrxp->next) {
                 bytes_needed += (4 + 4);
                 num_ports++;

                 struct net_bridge_mcast_rx_mac *mrxm;
                 for (mrxm = mrxp->rx_mac_list; 
                      mrxm != NULL; 
                      mrxm = mrxm->next) {
                     bytes_needed += (6 + 4);
                 }
             }

             if (bytes_needed <= buf_len) {
                 /* OK, create a record for this multicast group */
                 if (copy_to_user(buf, me->addr, 6)) {
                     ret = -EFAULT;
                     goto cleanup;
                 } else {
                     buf += 6;
                     buf_len -= 6;
                 }

                 if (copy_to_user(buf, &num_ports, 4)) {
                     ret = -EFAULT;
                     goto cleanup;
                 } else {
                     buf += 4;
                     buf_len -= 4;
                 }

                 for (mrxp = me->rx_port_list; 
                      mrxp != NULL; 
                      mrxp = mrxp->next) {
                     int port_no;
                     if (mrxp->dst)
                         port_no = mrxp->dst->port_no;
                     else
                         port_no = -1;

                     if (copy_to_user(buf, &port_no, 4)) {
                         ret = -EFAULT;
                         goto cleanup;
                     } else {
                         buf += 4;
                         buf_len -= 4;
                     }

                     unsigned int num_macs = 0;
                     struct net_bridge_mcast_rx_mac *mrxm;
                     for (mrxm = mrxp->rx_mac_list; 
                          mrxm != NULL; 
                          mrxm = mrxm->next)
                         num_macs++;

                     if (copy_to_user(buf, &num_macs, 4)) {
                         ret = -EFAULT;
                         goto cleanup;
                     } else {
                         buf += 4;
                         buf_len -= 4;
                     }

                     for (mrxm = mrxp->rx_mac_list; 
                          mrxm != NULL; 
                          mrxm = mrxm->next) {
                         if (copy_to_user(buf, mrxm->addr, 6)) {
                             ret = -EFAULT;
                             goto cleanup;
                         } else {
                             buf += 6;
                             buf_len -= 6;
                         }

                         unsigned long jiffies_expire = 
                             mrxm->ageing_timer + br->mcast_ageing_time;
                         unsigned long jiffies_left;
                         if (jiffies < jiffies_expire)
                             jiffies_left = jiffies_expire - jiffies;
                         else
                             jiffies_left = 0;

                         unsigned long secs_left = jiffies_left / HZ;
                         if (copy_to_user(buf, &secs_left, 4)) {
                             ret = -EFAULT;
                             goto cleanup;
                         } else {
                             buf += 4;
                             buf_len -= 4;
                         }
                     }
                 }

                 num_returned++;
             } else
                 goto cleanup;

             me = me->next_hash;
         }
    }


cleanup:
    read_unlock_bh(&br->mcast_lock);

    return ret ? ret : num_returned;
}

int br_mcast_is_management_header(struct ethhdr *ether)
{
	if ((ether->h_proto == htons(BR_MCAST_GL_PROTOCOLNUM)) &&
	    (0 == memcmp(ether->h_dest, rincon_gmp_addr, 6))) {

		return 1;
	}

	return 0;
}

/* Used to maintain the multicast forwarding database */
void br_mcast_check(struct sk_buff *skb,
                    struct net_bridge *br,
                    struct net_bridge_port *p)
{
	struct ethhdr *ether = skb->mac.ethernet;

	/* The multicast management packets are either:
	 *
	 * 1) DA of 01:0E:58:DD:DD:DD, PROTO of 0x6970
	 * 2) DA of FF:FF:FF:FF:FF:FF, PROTO of 0x6970 encapsulating #1
	 *
	 * Care is taken to not mangle other packets that happen to have a
	 * proto of 0x6970 so can interoperate with other people that are using
	 * the same proto (however unlikely).
	 */
	if (ether->h_proto == htons(BR_MCAST_GL_PROTOCOLNUM)) {

		/* Encapsulated? If it is really our packet, strip the outer
		 * header. 
		 */
		if (0 == memcmp(ether->h_dest, broadcast_addr, 6)) {

			if (skb->len >= ETH_HLEN*2) {

				/* REVIEW: Cleaner/safer way to check this? */
				ether++;

				if (br_mcast_is_management_header(ether)) {
                    skb->mac.ethernet = (struct ethhdr*)skb->data;
                    skb_pull(skb, ETH_HLEN);
					br_mcast_handle_grouplist(br, p, skb);
				}
			}

		} else if (br_mcast_is_management_header(ether)) {
			br_mcast_handle_grouplist(br, p, skb);
		}
	}
}
