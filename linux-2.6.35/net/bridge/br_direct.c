/*
 *	Direct routing extensions
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include "br_private.h"

#ifdef BR_DEBUG_DIRECT
static void _log_direct(int idx,
                        struct net_bridge_port *p_src,
                        struct net_bridge_fdb_entry *fdbe)
{
	static unsigned long i = 0;

	struct net_bridge_port *p_stp, *p_dir;

	p_stp = fdbe->dst;
	p_dir = fdbe->dst_direct;
	
	if ((i & 0xff) == 0) {
		unsigned char *addr = (unsigned char *)&(fdbe->addr);
		printk("dir: %d: src=%d, stp=%d (%d), dir=%d (%d:%d), "
		       "addr=%02x:%02x:%2x\n",
		       idx,
		       p_src ? p_src->port_no : 0,
		       p_stp ? p_stp->port_no : 0,
		       p_stp ? p_stp->is_p2p : 0,
		       p_dir ? p_dir->port_no : 0,
		       p_dir ? p_dir->remote_state : 0,
		       p_dir ? p_dir->direct_enabled : 0,
		       addr[3], addr[4], addr[5]);
	}
	i++;
}
#define BR_LOG_DIRECT(a,b,c) _log_direct((a), (b), (c))
#else
#define BR_LOG_DIRECT(a,b,c)
#endif

/* Returns 0 if the packet was sent not direct, 1 if it is. */
int br_direct_unicast(struct net_bridge_port *p_src,
		      struct net_bridge_fdb_entry *fdbe,
		      struct sk_buff *skb,
		      void (*__stp_hook)(const struct net_bridge_port *from,
					 const struct net_bridge_port *to,
					 struct sk_buff *skb),
		      void (*__direct_hook)(const struct net_bridge_port *from,
					    const struct net_bridge_port *to,
					    struct sk_buff *skb))
{
	struct net_bridge_port *p_stp, *p_dir;
	struct sk_buff *skb_dup = 0;

	/*
	 * STP destination must be wireless
	 */
	p_stp = fdbe->dst;

	if (0 == p_stp || !p_stp->is_p2p) {		
		/* Not wireless, send down the STP path */
		BR_LOG_DIRECT(1, p_src, fdbe);
		(__stp_hook)(p_src,  p_stp, skb);
		return 0;
	}

	/*
	 * Direct route must be present, different, and forwarding/blocking
	 */
	p_dir = fdbe->dst_direct;

	if (0 == p_dir || p_dir == p_stp ||	    
	    !p_dir->direct_enabled ||
	    p_dir->state <= BR_STATE_LEARNING ||
	    p_dir->remote_state <= BR_STATE_LEARNING) {
		/* Not good to go, send down the STP path */
		BR_LOG_DIRECT(2, p_src, fdbe);
		(__stp_hook)(p_src,  p_stp, skb);
		return 0;
	}

	/* Now that know it is safe, make sure we're sending enough traffic
	 * down the STP path.
	 *
	 * REVIEW: We should also consider only direct routing if we're sending
	 *         enough traffic to make the dups worth it.  This can be done
	 *         by logging the last time we sent anything to that
	 *         destination, and if it is > 5 seconds or so we should just
	 *         send down the STP path and be done with it.
	 */
	if (jiffies - p_dir->direct_last_stp_time > BR_DIRECT_STP_TIME) {
		
		p_dir->direct_last_stp_time = jiffies;		
		skb_dup = skb_copy(skb, GFP_ATOMIC);

		/* If we can't send a copy, the original goes down the STP
		 * path
		 */
		if (!skb_dup) {
			BR_LOG_DIRECT(3, p_src, fdbe);
			(__stp_hook)(p_src, p_stp, skb);
			return 1;
		}	
	}

        BR_LOG_DIRECT(4, p_src, fdbe);                

	/* Original goes down the direct path */
	(__direct_hook)(p_src, p_dir, skb);
       
	/* Duplicate goes down the STP path */
	if (skb_dup) {		
		(__stp_hook)(p_src, p_stp, skb_dup);
	}

	return 1;
}
