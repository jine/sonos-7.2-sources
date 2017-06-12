/*
 *   Mesh network routing
 *
 *   This is standard Netfilter queueing code.
 *
 *      $Id: pktqueue.c,v 1.1 2004/01/14 17:36:17 vangool Exp $
 *
 */
#include <linux/if.h>
#include <linux/ip.h>
#include <linux/netdevice.h>
#include <net/route.h>
#include <net/icmp.h>
#include <linux/netfilter_ipv4.h>

#include "pktqueue.h"

/* We need to keep track of the following packet info for later routing */
struct pktqueue_route_info {
   u_int8_t tos;
   u_int32_t daddr;
   u_int32_t saddr;
};

struct pktqueue_element {
   struct list_head list;
   struct nf_info *info;
   struct sk_buff *skb;
   struct pktqueue_route_info route_info;
};

struct pktqueue {
   int len;
   int maxlen;
   unsigned char flush;
   unsigned char terminate;
   struct list_head list;
   spinlock_t lock;
};

/* Add a packet to the queue */
static int pktenqueue(struct pktqueue* queue, struct sk_buff *skb,
      struct nf_info *info)
{
   struct pktqueue_element* entry;

   /* Allocate memory for the new entry */
   entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
   if (entry == NULL) {
      printk(KERN_ERR "Unable to create pktqueue_element\n");
      return -ENOMEM;
   }

   /* Fill in the details */
   entry->info = info;
   entry->skb = skb;

   /* Outgoing packets have route info that needs to be preserved */
   if (entry->info->hook == NF_IP_LOCAL_OUT) {
      struct iphdr *iph = skb->nh.iph;

      entry->route_info.tos = iph->tos;
      entry->route_info.daddr = iph->daddr;
      entry->route_info.saddr = iph->saddr;
   }

   /* Lock the queue */
   spin_lock_bh(&queue->lock);

   /* If the queue is full... */
   if (queue->len >= queue->maxlen) {
      spin_unlock_bh(&queue->lock);
      if (net_ratelimit())
	 printk(KERN_WARNING "pktqueue: full at %d entries\n", queue->len);
      goto free_drop;
   }

   /* If we need to flush/terminate... */
   if (queue->flush || queue->terminate) {
      spin_unlock_bh(&queue->lock);
      goto free_drop;
   }

   /* All is well so add the packet to the queue */
   list_add(&entry->list, &queue->list);
   /* One more packet in the queue */
   queue->len++;

   /* Unlock the queue */
   spin_unlock_bh(&queue->lock);

   /* Return the new queue length */
   return queue->len;

free_drop:
   kfree(entry);

   return -EBUSY;
}

/*
 * Remove a packet from the queue (potentially after calling a comparison
 * function).
 */
static struct pktqueue_element* pktdequeue(struct pktqueue* queue,
      int (*cmp)(struct pktqueue_element*, u_int32_t), u_int32_t data)
{
   struct list_head *i;

   /* Lock the queue */
   spin_lock_bh(&queue->lock);
   /* Iterate over all entries */
   for (i = queue->list.prev; i != &queue->list; i = i->prev) {
      struct pktqueue_element* entry = (struct pktqueue_element*)i;

      /* If there is no comparison function, or there is a match */
      if (!cmp || cmp(entry, data)) {
	 /* Remove it from the list */
	 list_del(&entry->list);
	 /* 1 Packet less in the queue... */
	 queue->len--;
         /* Unlock the queue */
	 spin_unlock_bh(&queue->lock);
	 return entry;
      }
   }
   /* Unlock the queue */
   spin_unlock_bh(&queue->lock);

   return NULL;
}

/* Flush all packets in the queue */
static void pktqueue_flush(struct pktqueue* queue, int verdict)
{
   struct pktqueue_element *entry;

   /* Lock the queue and update state to indicate it's being flushed */
   spin_lock_bh(&queue->lock);
   queue->flush = 1;
   spin_unlock_bh(&queue->lock);

   /* While there are entries in the queue */
   while ((entry = pktdequeue(queue, NULL, 0))) {
      nf_reinject(entry->skb, entry->info, verdict);
      kfree(entry);
   }

   /* Lock the queue and update state to indicate it's not being flushed */
   spin_lock_bh(&queue->lock);
   queue->flush = 0;
   spin_unlock_bh(&queue->lock);
}

/*
 * The packet handler that we register with Netfilter. Its only function is
 * to add the packet to our queue.
 */
static int packet_receive(struct sk_buff *skb, struct nf_info *info, void *data)
{
   return pktenqueue((struct pktqueue*)data, skb, info);
}

struct pktqueue *pktqueue_create(int maxlen)
{
   struct pktqueue *queue = NULL;
   int res;

   /* Allocate memory for the queue */
   queue = kmalloc(sizeof(struct pktqueue), GFP_KERNEL);
   if (queue == NULL) {
      return NULL;
   }

   /* Initialize it */
   queue->len = 0;
   queue->maxlen = maxlen;
   queue->flush = 0;
   queue->terminate = 0;
   INIT_LIST_HEAD(&queue->list);
   spin_lock_init(&queue->lock);

   /* Register the queue handler */
   res = nf_register_queue_handler(PF_INET, packet_receive, queue);
   if (res < 0) {
      /* Deal with the error */
      printk("Unable to create packet queue %d\n", res);
      kfree(queue);
      return NULL;
   }

   /* Return the queue */
   return queue;
}

void pktqueue_destroy(struct pktqueue* queue)
{
   /* Unregister the handler so no more packets get queued */
   nf_unregister_queue_handler(PF_INET);

   /* Lock the queue and update state to indicate termination */
   spin_lock_bh(&queue->lock);
   queue->terminate = 1;
   spin_unlock_bh(&queue->lock);

   /* Flush the remaining packets */
   pktqueue_flush(queue, NF_DROP);

   /* And free the memory */
   kfree(queue);
}

/* This route_me_harder function is a subset of the real ip_route_me_harder
 * function that can be found in core/netfilter.c. We use a subset because
 * we don't want to get stuck in an eternal loop */
static int route_me_harder(struct sk_buff *skb)
{
   struct iphdr *iph = skb->nh.iph;
   struct rtable *rt;

   struct rt_key key = {
      dst:iph->daddr,
      src:iph->saddr,
      oif:skb->sk ? skb->sk->bound_dev_if : 0,
      tos:RT_TOS(iph->tos) | RTO_CONN,
#ifdef CONFIG_IP_ROUTE_FWMARK
      fwmark:skb->nfmark
#endif
   };

   if (ip_route_output_key(&rt, &key) != 0) {
      printk("route_me_harder: No route to be found\n");
      return -EINVAL;
   }

   /* Drop old route. */
   dst_release(skb->dst);
   skb->dst = &rt->u.dst;

   return 0;
}

static struct pktqueue* packet_queue;

/* Function used for pktdequeue to match IP addresses */
static inline int ipcmp(struct pktqueue_element* entry, u_int32_t ip)
{
   if (entry->route_info.daddr == ip)
      return 1;

   return 0;
}

/*
 * Iterate over all the packets for this IP, send them on their way, accept
 * the packet itself, and then free the memory used to hold the queue entry.
 */
void pktqueue_send_ip(u_int32_t ip)
{  
   struct pktqueue_element* entry;

   while ((entry = pktdequeue(packet_queue, ipcmp, ip))) {
      entry->skb->nfcache |= NFC_ALTERED;
      route_me_harder(entry->skb);
      nf_reinject(entry->skb, entry->info, NF_ACCEPT);
      kfree(entry);
   }
}

/*
 * Iterate over all the packets for this IP, send 'host unreachable to the
 * sender, drop the packet itself, and then free the memory used to hold the
 * queue entry.
 */
void pktqueue_drop_ip(u_int32_t ip)
{
   struct pktqueue_element* entry;

   while ((entry = pktdequeue(packet_queue, ipcmp, ip))) {
      icmp_send(entry->skb, ICMP_DEST_UNREACH, ICMP_HOST_UNREACH, 0);
      nf_reinject(entry->skb, entry->info, NF_DROP);
      kfree(entry);
   }
}

/* Allocate packet queue */
int init_packetqueue(void)
{

   packet_queue = pktqueue_create(1024);

   if (packet_queue == NULL) {
      printk("Unable to create packet queue\n");
      return 0;
   }

   return 1;
}

/* Clean up the packet queue */
void deinit_packetqueue(void)
{
   pktqueue_destroy(packet_queue);
}

