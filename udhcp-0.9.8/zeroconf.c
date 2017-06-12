/* Copyright (c) 2004, Rincon Networks, Inc.  All rights reserved. */

/*
 * Additions to uDHCPc made by Rincon for ZeroConf address assignment
 */

#include <sys/fcntl.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <linux/if_packet.h>

#include "dhcpc.h"
#include "packet.h"
#include "script.h"
#include "zeroconf.h"
#include "debug.h"

extern struct client_config_t client_config;

/* Create an ARP socket to receive ARPs related to a specified interface.
   Note that we want (and get) both broadcast and unicast ARP messages. */
int zc_arp_socket(const char* inf)
{
    struct sockaddr_ll ll_from;
    struct ifreq ifr;
    int fd;

    fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ARP));
    if (fd < 0)
        return -1;

    /* Get the hwaddr and ifindex of the interface */
    memset(ifr.ifr_name, 0, IFNAMSIZ);
    strncpy(ifr.ifr_name, inf, IFNAMSIZ-1);
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0) {
        close(fd);
        return -1;
    }

    memset(ll_from.sll_addr, 0, ETH_ALEN);
    memcpy(ll_from.sll_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        return -1;
    }

    ll_from.sll_family = AF_PACKET;
    ll_from.sll_protocol = htons(ETH_P_ARP);
    ll_from.sll_ifindex = ifr.ifr_ifindex;
    ll_from.sll_hatype = ARPHRD_ETHER;
    ll_from.sll_pkttype = PACKET_HOST;
    ll_from.sll_halen = ETH_ALEN;
    
    if (bind(fd, (struct sockaddr*)&ll_from, sizeof(ll_from)) < 0) {
        close(fd);
        return -1;
    }

    return fd;    
}

/* process an incoming ARP datagram */
void zc_handle_input(struct zc_config* zc, time_t now)
{
    if (zc->zc_state != ZEROCONF_DISABLED) {
        unsigned char buf[2048];

        int len = recv(zc->zc_fd, buf, 2048, 0);
        if (len >= ETH_HLEN + (int)sizeof(struct ether_arp)) {
            /* parse the datagram */
            const unsigned char* pch = buf;
        
            /* skip the SNAP header */
            pch += ETH_HLEN;

            /* get the ARP payload */
            const struct ether_arp* ap = (const struct ether_arp*)pch;

            /* check if it's an IP/ethernet ARP */
            if ((ap->arp_pro != htons(ETHERTYPE_IP) &&
                 ap->arp_pro != htons(ETHERTYPE_TRAIL)) 
                || ap->arp_hln != sizeof(ap->arp_sha)
                || ap->arp_pln != sizeof(ap->arp_spa))
                return;

            /* check if the sender IP address is the address we want,
               if this is an ARP request or reply */
            if (ap->arp_op == htons(ARPOP_REQUEST) ||
                ap->arp_op == htons(ARPOP_REPLY)) {
                if (0 == memcmp(ap->arp_spa, zc->zc_addr, 4))
                    goto collision;
            }

            if (zc->zc_state == ZEROCONF_PROBING) {
                /* while probing, check if the target IP address is the 
                   address we want, if this is an ARP request. */
                if (ap->arp_op == htons(ARPOP_REQUEST)) {
                    if (0 == memcmp(ap->arp_tpa, zc->zc_addr, 4) &&
                        0 != memcmp(ap->arp_sha, zc->zc_hw_addr, ETH_ALEN))
                        goto collision;
                }
            }
        }
    }

    return;

collision:
    /* we get here when a collision was detected. */
    LOG(LOG_DEBUG, "Detected zc collision...");
    zc_start_probing(zc, now);
}

/* from dhcpc.c */
void background(void);

void zc_handle_timer(struct zc_config* zc, time_t now)
{
    switch(zc->zc_state) {
    case ZEROCONF_DISABLED:
    case ZEROCONF_RUNNING:
    default:
        /* zeroconf has no work to do */
        zc->zc_timeout = 0x7fffffff;
        break;
    case ZEROCONF_PROBING:
        /* in the probing state, 4 arp messages are sent.  The first
           one is preceded by a random delay between 0 and 2 seconds;
           subsequent requests are each 2 seconds apart; at the end
           of the 2 seconds, if no one has defended the address, we
           claim it for our use and enter the ANNOUNCING state. */
        if (zc->zc_count == 0) {
            /* should wait for a random amount of time (FIX?) */
            zc->zc_timeout = now + 2;
            zc->zc_count++;
        } else if (zc->zc_count <= 4) {
            /* send ARP probe */
            zc_send_arp_probe(zc);

            /* wait for two seconds */
            zc->zc_timeout = now + 2;
            zc->zc_count++;
        } else {
            /* run the script to bring up the interface with this IP address */
            char rgch[32];
            sprintf(rgch, "%u.%u.%u.%u", zc->zc_addr[0], zc->zc_addr[1],
                    zc->zc_addr[2], zc->zc_addr[3]);
            run_script_zeroconf(rgch, "255.255.0.0", "bound");

            LOG(LOG_INFO, "zc announcing %s", rgch);

            /* move into announcing state */
            zc->zc_state = ZEROCONF_ANNOUNCING;
            zc->zc_count = 0;

            /* now that we have chosen an AutoIP address, switch to running
               in the background if the caller wants us to. */
            if (!client_config.foreground)
                background();

            /* continue immediately */
            zc->zc_timeout = now;
        }

        break;
    case ZEROCONF_ANNOUNCING:
        /* in the announcing state, 2 arp messages are sent, 2 seconds
           apart. */
        if (zc->zc_count == 0) {
            /* send ARP announcement */
            zc_send_arp_announce(zc);

            /* wait for two seconds */
            zc->zc_timeout = now + 2;
            zc->zc_count++;
        } else {
            /* send ARP announcement */
            zc_send_arp_announce(zc);            

            /* move into running state */
            zc->zc_state = ZEROCONF_RUNNING;
            zc->zc_count = 0;

            /* continue immediately */
            zc->zc_timeout = now;
        }

        break;
    };
}

/* initial state, to be invoked once during boot */
void zc_init(struct zc_config* zc, const unsigned char* hw_addr)
{
    zc->zc_fd = -1;
    zc->zc_state = ZEROCONF_DISABLED;
    zc->zc_timeout = 0x7ffffff;
    zc->zc_count = 0;

    memset(zc->zc_addr, 0, 4);
    
    /* compute the bound interface's hardware address */
    memcpy(zc->zc_hw_addr, hw_addr, ETH_ALEN);
}

/* reset state; to be called when DHCP takes over, etc */
void zc_reset(struct zc_config* zc)
{
    if (zc->zc_state != ZEROCONF_DISABLED) {
        close(zc->zc_fd);
        zc->zc_fd = -1;
        zc->zc_state = ZEROCONF_DISABLED;
        zc->zc_timeout = 0x7fffffff;
        zc->zc_count = 0;
        memset(zc->zc_addr, 0, 4);
        
        /* if the interface was configured, take it down */
        if (zc->zc_state == ZEROCONF_ANNOUNCING ||
            zc->zc_state == ZEROCONF_RUNNING)
            run_script(NULL, "deconfig");
    }
}

/* generate random IP address and enter probing state */
void zc_start_probing(struct zc_config* zc, time_t now)
{
    /* generate a random IP address in the zeroconf range */
    zc->zc_addr[0] = 169;
    zc->zc_addr[1] = 254;

    /* we should not be using /dev/random, as it is a limited source
       of entropy; if we run out of bits, we could be waiting forever.
       instead, use /dev/urandom, especially in this case where we don't
       need cryptographic-level entropy */
    int fd = open("/dev/urandom", O_RDONLY);
    /* REVIEW: need to robustify this -- use srand()/rand() if we fail? */

    do {
        read(fd, &zc->zc_addr[2], 1);
    } while (zc->zc_addr[2] < 1 || zc->zc_addr[2] > 254);
    
    read(fd, &zc->zc_addr[3], 1);

    close(fd);

    /* if the interface has been brought up with the zeroconf address, 
       deconfigure it. */
    if (zc->zc_state == ZEROCONF_ANNOUNCING || 
        zc->zc_state == ZEROCONF_RUNNING) {
        /* invoke the deconfigure script */
        run_script(NULL, "deconfig");
    }

    /* initialize state variables */
    LOG(LOG_DEBUG, "Trying zc probe of %u.%u.%u.%u",
        zc->zc_addr[0], zc->zc_addr[1], zc->zc_addr[2], zc->zc_addr[3]);
    zc->zc_state = ZEROCONF_PROBING;
    zc->zc_timeout = now;
    zc->zc_count = 0;
}

/* send an ARP probe message */
void zc_send_arp_probe(struct zc_config* zc)
{
    /* put together the ARP datagram */
    unsigned char buf[2048];
    struct ether_header* packethdr = (struct ether_header*)buf;
    struct ether_arp* ap = (struct ether_arp*)(packethdr + 1);

    /* fill in the ethernet header */
    memcpy(&(packethdr->ether_shost), zc->zc_hw_addr, ETH_ALEN);
    memset(&(packethdr->ether_dhost), 0xFF, ETH_ALEN);
    packethdr->ether_type = htons(ETHERTYPE_ARP);    

    /* fill in the ARP datagram */
    ap->arp_hrd = htons(ARPHRD_ETHER);
    ap->arp_pro = htons(ETHERTYPE_IP);
    ap->arp_hln = ETH_ALEN;
    ap->arp_pln = 4;
    ap->arp_op = htons(ARPOP_REQUEST);

    /* fill in the specifics of this ARP request */
    memcpy(ap->arp_sha, zc->zc_hw_addr, ETH_ALEN);
    memset(ap->arp_tha, 0, ETH_ALEN);
    memset(ap->arp_spa, 0, 4);
    memcpy(ap->arp_tpa, zc->zc_addr, 4);

    /* send the ARP */
    LOG(LOG_DEBUG, "Sending zc arp probe...");
    send(zc->zc_fd, buf, sizeof(*packethdr) + sizeof(*ap), 0);
}

/* send an ARP probe message */
void zc_send_arp_announce(struct zc_config* zc)
{
    /* put together the ARP datagram */
    unsigned char buf[2048];
    struct ether_header* packethdr = (struct ether_header*)buf;
    struct ether_arp* ap = (struct ether_arp*)(packethdr + 1);

    /* fill in the ethernet header */
    memcpy(&(packethdr->ether_shost), zc->zc_hw_addr, ETH_ALEN);
    memset(&(packethdr->ether_dhost), 0xFF, ETH_ALEN);
    packethdr->ether_type = htons(ETHERTYPE_ARP);    

    /* fill in the ARP datagram */
    ap->arp_hrd = htons(ARPHRD_ETHER);
    ap->arp_pro = htons(ETHERTYPE_IP);
    ap->arp_hln = ETH_ALEN;
    ap->arp_pln = 4;
    ap->arp_op = htons(ARPOP_REQUEST);

    /* fill in the specifics of this ARP request */
    memcpy(ap->arp_sha, zc->zc_hw_addr, ETH_ALEN);
    memset(ap->arp_tha, 0, ETH_ALEN);
    memcpy(ap->arp_spa, zc->zc_addr, 4);
    memcpy(ap->arp_tpa, zc->zc_addr, 4);

    /* send the ARP */
    LOG(LOG_DEBUG, "Sending zc announce...");
    send(zc->zc_fd, buf, sizeof(*packethdr) + sizeof(*ap), 0);
}
