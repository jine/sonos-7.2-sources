#include <linux/netfilter_ipv4.h>
#include <linux/socket.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/net.h>
#include <net/sock.h>
#include <linux/fs.h>
#include <linux/wireless.h>
#include <linux/if_arp.h>
#include <linux/sockios.h>
#include <linux/ioctl.h>
#include <asm/div64.h>
#include <asm/uaccess.h>
#include <linux/fcntl.h>

#include <linux/ip.h>
#include <net/route.h>
#include <net/udp.h> 
#include <net/protocol.h>
#include <net/inet_common.h>
#include <linux/ctype.h>

#include "utils.h"
#include "mesh.h"
#include "timers.h"
#include "iflist.h"
#include "mesh_neighbour.h"

static struct sockaddr_in sin;

static struct socket *multicast_sock;

static struct socket *iw_sock;

int seq_less_or_equal(u_int32_t seq_one, u_int32_t seq_two)
{
   int *comp_seq_one = &seq_one;
   int *comp_seq_two = &seq_two;

   if ((*comp_seq_one - *comp_seq_two) > 0) {
      return 0;
   }

   return 1;
}

int seq_greater(u_int32_t seq_one, u_int32_t seq_two)
{
   int *comp_seq_one = &seq_one;
   int *comp_seq_two = &seq_two;

   if ((*comp_seq_one - *comp_seq_two) < 0) {
      return 0;
   }

   return 1;
}

int inet_aton(const char *cp, __u32 *addr)
{
   unsigned int val;
   int base, n;
   char c;
   u_int parts[4];
   u_int *pp = parts;

   for (;;) {
      val = 0;
      base = 10;
      if (*cp == '0') {
	 if (*++cp == 'x' || *cp == 'X') {
	    base = 16, cp++;
	 } else {
	    base = 8;
	 }
      }
      while ((c = *cp) != '\0') {
	 if (isascii(c) && isdigit(c)) {
	    val = (val * base) + (c - '0');
	    cp++;
	    continue;

	 }
	 if (base == 16 && isascii(c) && isxdigit(c)) {
	    val = (val << 4) +
		  (c + 10 - (islower(c) ? 'a' : 'A'));
	    cp++;
	    continue;
	 }
	 break;
      }
      if (*cp == '.') {
	 if (pp >= parts + 3 || val > 0xff)
	    return (0);
	 *pp++ = val, cp++;
      }
      else
	 break;
   }

   /* Check for trailing characters. */
   if (*cp && (!isascii(*cp) || !isspace(*cp)))
      return (0);

   n = pp - parts + 1;
   switch (n)
   {
      case 1:
	 break;

      case 2:
	 if (val > 0xffffff)
	    return (0);
	 val |= parts[0] << 24;
	 break;

      case 3:
	 if (val > 0xffff)
	    return (0);
	 val |= (parts[0] << 24) | (parts[1] << 16);
	 break;

      case 4:
	 if (val > 0xff)
	    return (0);
	 val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
	 break;
   }
   if (addr)
      *addr= htonl(val);
   return (1);
}

/* This routine is a hack. Cannot be called multiple time simultaneously */
char *inet_ntoa(__u32 ina)
{
   static char buf[4*sizeof "123"];
   unsigned char *ucp = (unsigned char *)&ina;

   sprintf(buf, "%d.%d.%d.%d",
	 ucp[0] & 0xff,
	 ucp[1] & 0xff,
	 ucp[2] & 0xff,
	 ucp[3] & 0xff);

   return buf;
}

int init_multicast_sock(void)
{
   mm_segment_t oldfs;
   struct ifreq interface;
   int res = 0;
   int bool = 1;
   int choice = 1;

   /* Create a socket */
   res = sock_create(PF_INET, SOCK_RAW, IPPROTO_RAW, &(multicast_sock));

   /* Initialize the address */
   memset(&sin,0,sizeof(sin));
   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr = get_my_ip_address();
   sin.sin_port = htons(AODVPORT);

   /* Initialize the socket */
   multicast_sock->sk->reuse =1;
   multicast_sock->sk->allocation = GFP_ATOMIC;
   multicast_sock->sk->priority = GFP_ATOMIC;

   /* Bind the socket to the address */
   res = multicast_sock->ops->bind(multicast_sock, (struct sockaddr*)&sin,
	 sizeof(struct sockaddr_in));

   /*
    * Default behaviour is to bind to eth0 unless we have a default route.
    * in which case we use the interface associated with the default route
    */
   if (get_my_route_entry()) {
      strncpy(interface.ifr_ifrn.ifrn_name, get_my_route_entry()->dev->name,
	    IFNAMSIZ);
   } else {
      strncpy(interface.ifr_ifrn.ifrn_name, "eth0", IFNAMSIZ);
   }

   /* Preserve DS and FS segments (Intel only) */
   oldfs = get_fs();
   set_fs(get_ds());

   /* Bind the socket to the device */
   if ((res = sock_setsockopt(multicast_sock, IPPROTO_IP, SO_BINDTODEVICE,
	       (char*)&interface, sizeof(interface))) < 0 )
   {
      printk(KERN_WARNING "Unable to bind to interface (%d)\n", res); 
   }

   if ((res = sock_setsockopt(multicast_sock, IPPROTO_IP, SO_DONTROUTE,
	       (char *)&choice, sizeof(int))) < 0)
   {
      printk(KERN_WARNING "Unable to mark not to route (%d)\n", res); 
   }

   if (res < 0) {
      printk(KERN_ERR "Error, %d  binding to socket.\n", res);

      /* Restore DS and FS segments (Intel only) */
      set_fs(oldfs);

      return 0;
   }

   if ((res = multicast_sock->ops->setsockopt(multicast_sock, IPPROTO_IP,
	       IP_HDRINCL, (char *)&bool, sizeof(bool))) < 0)
   {
      printk(KERN_WARNING "IP_HDRINCL failed (%d)\n", res);
   }

   /* Restore DS and FS segments (Intel only) */
   set_fs(oldfs);

   return 0;
}

int init_sock(struct socket *sock, u_int32_t ip, char *dev_name)
{
   int res;
   struct ifreq interface;
   mm_segment_t oldfs;

   /* Initialize the address */
   memset(&sin,0,sizeof(sin));
   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr = ip;
   sin.sin_port = htons(AODVPORT);

   /* Initialize the socket */
   sock->sk->reuse = 1;
   sock->sk->allocation = GFP_ATOMIC;
   sock->sk->priority = GFP_ATOMIC;

   /* Bind the socket to the address */
   res = sock->ops->bind(sock,(struct sockaddr*)&sin,sizeof(struct sockaddr_in));

   /* Set the interface we're going to use */
   strncpy(interface.ifr_ifrn.ifrn_name,dev_name,IFNAMSIZ);

   /* Preserve DS and FS segments (Intel only) */
   oldfs = get_fs();
   set_fs(KERNEL_DS);

   /* Try to bind to the specified device */
   res = sock_setsockopt(sock,SOL_SOCKET,SO_BINDTODEVICE,
	 (char *) &interface, sizeof(interface)) < 0;

   /* Restore DS and FS segments (Intel only) */
   set_fs(oldfs);

   if (res < 0) {
      printk(KERN_ERR "Error, %d  binding to socket.\n", res);
      return 0;
   }

   return 0;
}

void close_sock(void)
{
   struct if_entry *tmp_interface, *dead_interface;

   /* Close the multicast socket */
   sock_release(multicast_sock);

   /* Iterate over all interface, close them and release memory */
   tmp_interface=get_first_interface();
   while (tmp_interface != NULL) {
      /* Close the socket */
      sock_release(tmp_interface->sock);

      /* Keep a pointer to this entry */
      dead_interface = tmp_interface;

      /* Move to the next one */
      tmp_interface = tmp_interface->next;

      /* Release memory */
      kfree(dead_interface);
   }
}

void init_iw_sock(void)
{
   int res;

   res = sock_create(AF_INET, SOCK_DGRAM, 0, &iw_sock);
   if (res < 0) {
      printk(KERN_ERR "Error iw_sock (%d)\n", res);
   }
}

void close_iw_sock(void)
{
   sock_release(iw_sock);
}

/*
 * Set up the spy interface. This is a bit of a hack. We loop over all the
 * interfaces, and within that loop, loop over all the neighbours. So we
 * can add each neighbour to the spy list of the appropriate interface.
 */
void setup_spy_interface()
{
   mm_segment_t oldfs;
   int res;
   int i;
   struct mesh_neighbour *tmp_neigh;
   struct if_entry *tmp_interface;
   struct sockaddr iw_sa[IW_MAX_SPY];
   struct iwreq wrq;

   /* Loop over all the interfaces */
   tmp_interface = get_first_interface();
   while (tmp_interface != NULL) {

      /* Verify the interface is actually a wireless one */
      if ((tmp_interface->dev->get_wireless_stats != NULL) &&
	    (tmp_interface->dev->do_ioctl != NULL))
      {
	 i=0;

	 /* Now iterates over all neighbours */
	 tmp_neigh = find_neighbour(0, NULL);
	 while (tmp_neigh != NULL) {

	    /* Is it a neighbour on this interface? */
	    if ((tmp_interface->dev == tmp_neigh->dev)) {
	       /* Check if there is room in the spy list */
	       if (i < IW_MAX_SPY) {
		  /* Add this MAC to the spy list */
		  memcpy((char *)&(iw_sa[i].sa_data),
			(char *)&(tmp_neigh->hw_addr),
			sizeof(struct sockaddr));
		  /* Go to the next spy entry */
		  i++;
		  /* Set the signal strength in the neighbour entry */
		  tmp_neigh->signal_strength = 255;
	       }
	    }
	    /* Move to the next neighbour */
	    tmp_neigh=tmp_neigh->next;
	 }

	 /* Create the wireless request */
	 strncpy(wrq.ifr_name, tmp_interface->dev->name, IFNAMSIZ);
	 wrq.u.data.pointer = (caddr_t)&(iw_sa);
	 wrq.u.data.length = i;
	 wrq.u.data.flags = 0;

	 /* Preserve DS and FS segments (Intel only) */
	 oldfs = get_fs();
	 set_fs(KERNEL_DS);

	 res = tmp_interface->dev->do_ioctl(tmp_interface->dev,
	       (struct ifreq*)&wrq, SIOCSIWSPY);

	 /* Restore DS and FS segments (Intel only) */
	 set_fs(oldfs);

	 if (res < 0)
	    printk(KERN_WARNING "Unable to call SIOCSIWSPY (%d)\n", res);
      }
      /* Go to the new interface */
      tmp_interface = tmp_interface->next;
   }
}


int get_range_info(struct net_device *dev, char *ifname,
      struct iw_range *range)
{
   struct iwreq wrq;
   char buffer[sizeof(struct iw_range) * 2];

   /* Initialize */
   memset(buffer, 0, sizeof(range));
   strcpy(wrq.ifr_name, ifname);
   wrq.u.data.pointer = (caddr_t)buffer;
   wrq.u.data.length = 0;
   wrq.u.data.flags = 0;

   /* Get the range info from the device */
   if (dev->do_ioctl(dev, (struct ifreq *)&wrq, SIOCGIWRANGE) < 0)
      return -1;

   /* Copy info to output variable */
   memcpy((char*)range, buffer, sizeof(struct iw_range));

   return(0);
}

void get_wireless_stats()
{
   int n, i, has_range = 0;
   char buffer[(sizeof(struct iw_quality) +
	 sizeof(struct sockaddr)) * IW_MAX_SPY];
   struct iwreq wrq;
   struct if_entry *tmp_interface;
   struct sockaddr hwa[IW_MAX_SPY];
   struct iw_quality qual[IW_MAX_SPY];
   struct iw_range range;

   /* Loop over all interfaces to get their spy stats */
   tmp_interface = get_first_interface();
   while (tmp_interface != NULL) {
      /* Make sure it's a wireless interface */
      if ((tmp_interface->dev->get_wireless_stats != NULL) &&
	    (tmp_interface->dev->do_ioctl != NULL))
      {
	 /* Create the wireless request */
	 strncpy(wrq.ifr_name, tmp_interface->dev->name, IFNAMSIZ);
	 wrq.u.data.pointer = (caddr_t)buffer;
	 wrq.u.data.length = 0;
	 wrq.u.data.flags = 0;

	 /* Now call the ioctl to get the spy info */
	 tmp_interface->dev->do_ioctl(tmp_interface->dev,
	       (struct ifreq*)&wrq, SIOCGIWSPY);

	 /* Get the range info from that same interface */
	 if (get_range_info(tmp_interface->dev, tmp_interface->dev->name,
		  &(range)) >= 0)
	 {
	    has_range = 1;
	 }
	 n = wrq.u.data.length;

	 memcpy(hwa, buffer, n * sizeof(struct sockaddr));
	 memcpy(qual, buffer + n*sizeof(struct sockaddr),
	       n*sizeof(struct iw_quality));

	 for(i = 0; i < n; i++) {
	    if(has_range && (qual[i].level != 0)) {
	       if (range.max_qual.qual!=0) {
		  update_signal_strength_by_hw(hwa[i].sa_data,qual[i].level);
	       }
	    } else {
	       update_signal_strength_by_hw(hwa[i].sa_data,qual[i].level);
	    }
	 }
      }
      /* Move to the next interface */
      tmp_interface = tmp_interface->next;
   }
}

int send_multicast_message(u_int32_t dst_ip, u_int16_t datalen, void *data,
      u_int8_t ttl)
{
   mm_segment_t oldfs;
   struct msghdr msg;
   struct iovec iov;
   u_int32_t space;
   int res = 0;

   /* Make sure we have a TTL */
   if (ttl == 0 ) return 0;

   /* Check the available space in skbuff */
   space = sock_wspace(multicast_sock->sk);
   if (space < datalen) {
      /* Inform the user */
      printk(KERN_WARNING "Not enough space in skbuff (%d < %d)\n",
	    space, datalen);

      /* Return the error condition */
      return -ENOMEM;
   }

   /* Initialize the destination data */
   memset(&sin, 0, sizeof(sin));
   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr= dst_ip;

   /* Create the message */
   msg.msg_name = (void *)&(sin);
   msg.msg_namelen = sizeof(sin);
   msg.msg_iov = &iov;
   msg.msg_iovlen = 1;
   msg.msg_control = NULL;
   msg.msg_controllen = 0;
   msg.msg_flags = MSG_DONTWAIT | MSG_NOSIGNAL;
   msg.msg_iov->iov_len =  datalen;
   msg.msg_iov->iov_base = (char*)data;

   /* Setup the socket */
   multicast_sock->sk->broadcast = 1;
   multicast_sock->sk->protinfo.af_inet.ttl = ttl;

   /* Preserve DS and FS segments (Intel only) */
   oldfs = get_fs();
   set_fs(KERNEL_DS);

   res = sock_sendmsg(multicast_sock, &msg, datalen);
   if (res < 0) {
      /* Inform the user */
      printk(KERN_WARNING "Error sending message (errno = %d, dst: %s)\n",
	    res, inet_ntoa(dst_ip));
   }

   /* Restore DS and FS segments (Intel only) */
   set_fs(oldfs);

   return res;
}

int send_broadcast(u_int8_t ttl, void *data, int datalen)
{
   mm_segment_t oldfs;
   struct msghdr msg;
   struct iovec iov;
   struct if_entry *tmp_interface;
   int res = 0;

   /* Make sure we have a TTL */
   if (ttl == 0 ) return 0;

   /* Initialize the destination data */
   memset(&sin, 0, sizeof(sin));
   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr = INADDR_BROADCAST;
   sin.sin_port = htons((unsigned short)AODVPORT);

   /* Create the message */
   msg.msg_name = (void *) &(sin);
   msg.msg_namelen = sizeof(sin);
   msg.msg_iov = &iov;
   msg.msg_iovlen = 1;
   msg.msg_control = NULL;
   msg.msg_controllen = 0;
   msg.msg_flags = MSG_DONTWAIT | MSG_NOSIGNAL;
   msg.msg_iov->iov_len = datalen;
   msg.msg_iov->iov_base = (char*) data;

   /*
    * This is a broadcast message and so we need to send it out over every
    * interface. But make sure the interface has a socket and the socket
    * has enough space in the skbuff.
    */
   tmp_interface = get_first_interface();
   while (tmp_interface && tmp_interface->sock &&
	 (sock_wspace(tmp_interface->sock->sk) >= datalen) )
   {
      /* Setup the socket */
      tmp_interface->sock->sk->broadcast = 1;
      tmp_interface->sock->sk->protinfo.af_inet.ttl = ttl;

      /* Preserve DS and FS segments (Intel only) */
      oldfs = get_fs();
      set_fs(KERNEL_DS);

      /* Send it */
      res = sock_sendmsg(tmp_interface->sock, &msg, datalen);
      if (res < 0)
	 printk(KERN_WARNING "Error sending message (errno = %d, if: %s)\n",
	       res, tmp_interface->dev->name);

      /* Restore DS and FS segments (Intel only) */
      set_fs(oldfs);

      /* Move to the next interface */
      tmp_interface=tmp_interface->next;
   }

   return res;
}


int send_message(u_int32_t dst_ip, u_int8_t ttl, void *data, int datalen)
{
   mm_segment_t oldfs;
   struct msghdr msg;
   struct iovec iov;
   struct if_entry *tmp_interface;
   struct rtable_entry *tmp_route;
   u_int32_t space;
   int res = 0;

   /* Make sure we have a TTL */
   if (ttl == 0) return 0;

   /* Initialize the destination data */
   memset(&sin, 0, sizeof(sin));
   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr= dst_ip;
   sin.sin_port = htons((unsigned short)AODVPORT);

   /* Create the message */
   msg.msg_name = (void*)&(sin);
   msg.msg_namelen = sizeof(sin);
   msg.msg_iov = &iov;
   msg.msg_iovlen = 1;
   msg.msg_control = NULL;
   msg.msg_controllen = 0;
   msg.msg_flags = MSG_DONTWAIT | MSG_NOSIGNAL;
   msg.msg_iov->iov_len = datalen;
   msg.msg_iov->iov_base = (char*)data;

   /*
    * We need to determine what interface the send the message out of. So
    * we look up the route to the destination, and what device is associated
    * with that. Given that we can look up the interface.
    */
   tmp_route = find_route_table_entry(dst_ip);
   if (tmp_route == NULL) {
      /* This shouldn't really happen. Inform the user */
      printk(KERN_WARNING "Unable to find route to %s\n",inet_ntoa(dst_ip));

      /* Let's get out of here */
      return -EHOSTUNREACH;
   }

   /* We have the route. Now get the interface */
   tmp_interface = dev2if(tmp_route->dev);
   if (tmp_interface == NULL) {
      /* This shouldn't really happen. Inform the user */
      printk(KERN_WARNING "Unable to determine interface\n");

      /* Let's get out of here */
      return -ENODEV;
   }

   /* Verify there is enough space for this message */
   space = sock_wspace(tmp_interface->sock->sk);
   if (space < datalen) {
      /* Inform the user */
      printk(KERN_WARNING "Not enough space in skbuff (%d < %d)\n",
	    space, datalen);

      /* Return the error condition */
      return -ENOMEM;
   }

   /* Setup the socket */
   tmp_interface->sock->sk->broadcast = 0;
   tmp_interface->sock->sk->protinfo.af_inet.ttl = ttl;

   /* Preserve DS and FS segments (Intel only) */
   oldfs = get_fs();
   set_fs(KERNEL_DS);

   /* Send it */
   res = sock_sendmsg(tmp_interface->sock, &msg, datalen);
   if (res < 0) {
      /* Inform the user */
      printk(KERN_WARNING "Error sending message (errno = %d, dst: %s)\n",
	    res, inet_ntoa(dst_ip));
   }

   /* Restore DS and FS segments (Intel only) */
   set_fs(oldfs);

   return res;
}

/* Get the current time in millisecs */
u_int64_t get_current_time()
{
   struct timeval tv;
   u_int64_t      result;

   do_gettimeofday(&tv);

   result = (u_int64_t)tv.tv_usec;
   do_div(result, 1000);
   return ((u_int64_t)tv.tv_sec) * 1000 + result;
}

/* Given a MAC address, update the signal strength */
void update_signal_strength_by_hw(char *hw_addr, u_int8_t signal_strength)
{
   struct mesh_neighbour *tmp_entry;
   struct rtable_entry *tmp_route;

   /* Traverse the list to get the right entry */
   tmp_entry = find_neighbour(0, hw_addr);
   if (tmp_entry != NULL) {
      /* Update the signal strength */
      tmp_entry->signal_strength = signal_strength;

      /* Also update the route entry with the new info */
      tmp_route = find_route_table_entry(tmp_entry->ip);
      if (tmp_route != NULL) {
	 tmp_route->link = signal_strength;
      }
   }
}

