/* Copyright (c) 2003, Rincon Networks, Inc.  All rights reserved. */

#include "nlhelp.h"

#include <asm/types.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/sockios.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef __SONOS_ARM__
#include <linux/if_link.h>              /* IFLA_WIRELESS */
#include <linux/wireless.h>             /* IFLA_xxx */
#endif

#include "zeroconf.h"
#include "debug.h"

void rtnl_close(struct rtnl_handle *rth)
{
    close(rth->fd);
}

int rtnl_open(struct rtnl_handle *rth, unsigned subscriptions)
{
    socklen_t addr_len;

    memset(rth, 0, sizeof(*rth));

    rth->fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (rth->fd < 0)
        return -1;

    memset(&rth->local, 0, sizeof(rth->local));
    rth->local.nl_family = AF_NETLINK;
    rth->local.nl_groups = subscriptions;

    if (bind(rth->fd, (struct sockaddr*)&rth->local, sizeof(rth->local)) < 0) {
        close(rth->fd);
        return -1;
    }

    addr_len = sizeof(rth->local);
    if (getsockname(rth->fd, (struct sockaddr*)&rth->local, &addr_len) < 0) {
        close(rth->fd);
        return -1;
    }

    if (addr_len != sizeof(rth->local)) {
        close(rth->fd);
        return -1;
    }

    if (rth->local.nl_family != AF_NETLINK) {
        close(rth->fd);
        return -1;
    }

    rth->seq = time(NULL);

    return 0;
}

#if defined(__SONOS_LINUX__)

static int __get_arp_ip(const char* if_name,
                        in_addr_t* out_addr)
{
    int ret = 0;
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd != -1) {
        char buffer[16384];
        struct ifconf ifConf;
        ifConf.ifc_len = sizeof(buffer);
        ifConf.ifc_ifcu.ifcu_buf = (caddr_t)buffer;
        if (ioctl(fd, SIOCGIFCONF, &ifConf) >= 0) {
            int i;
            for (i = 0; i < ifConf.ifc_len; ) {
                struct ifreq *pifReq =
                    (struct ifreq *)((caddr_t)ifConf.ifc_req + i);
                i += sizeof(*pifReq);

                if ((pifReq->ifr_addr.sa_family == AF_INET) &&
                    (0 == strcmp(if_name, pifReq->ifr_name))) {
                    struct sockaddr_in addr;
                    memcpy(&addr, &pifReq->ifr_addr, sizeof(pifReq->ifr_addr));
                    *out_addr = addr.sin_addr.s_addr;
                    ret = 1;
                    break;
                }
            }
        }

        close(fd);
    }

    return ret;
}

int rtnl_send_gratuitous_arp(const char* if_name,
                             const unsigned char* if_hw_addr)
{
    in_addr_t addr;

    if (__get_arp_ip(if_name, &addr)) {
        // send the gratuitious ARP, if we can determine the IP address for 
        // the interface in question
        int fd = zc_arp_socket(if_name);
        if (fd != -1) {
            /* put together the ARP datagram */
            unsigned char buf[2048];
            struct ether_header* packethdr = (struct ether_header*)buf;
            struct ether_arp* ap = (struct ether_arp*)(packethdr + 1);

            /* fill in the ethernet header */
            memcpy(&(packethdr->ether_shost), if_hw_addr, ETH_ALEN);
            memset(&(packethdr->ether_dhost), 0xFF, ETH_ALEN);
            packethdr->ether_type = htons(ETHERTYPE_ARP);    

            /* fill in the ARP datagram */
            ap->arp_hrd = htons(ARPHRD_ETHER);
            ap->arp_pro = htons(ETHERTYPE_IP);
            ap->arp_hln = ETH_ALEN;
            ap->arp_pln = 4;
            ap->arp_op = htons(ARPOP_REQUEST);

            /* fill in the specifics of this ARP request */
            memcpy(ap->arp_sha, if_hw_addr, ETH_ALEN);
            memset(ap->arp_tha, 0, ETH_ALEN);
            memcpy(ap->arp_spa, &addr, 4);
            memcpy(ap->arp_tpa, &addr, 4);

            /* send the ARP */
            send(fd, buf, sizeof(*packethdr) + sizeof(*ap), 0);

            close(fd);

            return 1;
        }
    }

    return 0;
}

#if defined(SONOS_ARCH_ARM)

static void __interpret_ifla_wireless_msg(const char *data,
                                          int data_len,
                                          const char* if_name,
                                          int* needs_dhcp_renew,
                                          int* needs_gratuitous_arp)
{
    struct iw_event iwe_buf, *iwe = &iwe_buf;
    const char *pos, *end;
    int bSawESSID = 0;
    static char prevSSID[IW_ESSID_MAX_SIZE + 1] = {0};

    pos = data;
    end = data + data_len;

    while ((pos + IW_EV_LCP_LEN <= end) && !bSawESSID) {
        /* Event data may be unaligned, so make a local, aligned copy
         * before processing. */
        memcpy(&iwe_buf, pos, IW_EV_LCP_LEN);
        if (iwe->len <= IW_EV_LCP_LEN)
            return;

        // check for valid length
        if (pos + iwe->len > end)
            return;

        // copy MIN(sizeof(iwe_buf), iwe->len) bytes to iwe_buf
        int iwe_len = (iwe->len < sizeof(iwe_buf)) ? 
            iwe->len : sizeof(iwe_buf);
        memcpy(&iwe_buf, pos, iwe_len);

        switch (iwe->cmd) {
        case SIOCGIWAP:
            // Association event (signaled from wifi driver)
            *needs_gratuitous_arp = 1;
            break;
       case SIOCSIWESSID: 
       {
            // Set ESSID event (signaled from Linux wireless.c)
            struct iwreq iwr;
            int err, fd;
            char essid[IW_ESSID_MAX_SIZE + 1];

            // This is a bit of a hack, but disconnects can be 
            // determined by an event length <= 8 bytes.  So no need to renew.
            if (iwe->len <= 8) {
                break;
            }

            // If the name is the same as the previous name or
            // an empty string, no need to renew.
            fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd != -1) {
                memset(essid, '\0', sizeof(essid));
                iwr.u.essid.pointer = (caddr_t) essid;
                iwr.u.essid.length = IW_ESSID_MAX_SIZE;
                iwr.u.essid.flags = 0;
                strncpy(iwr.ifr_name, if_name, IFNAMSIZ-1);
                iwr.ifr_name[IFNAMSIZ-1] = '\0';

                // Get the current ESSID.  Note: if we are disconnected
                // this returns an error and we would request a spurious renew.
                // The event length check above avoids this.
                if ((err=ioctl(fd, SIOCGIWESSID, &iwr)) >= 0) {
                    if ((strlen(essid) == 0) ||
                        (strcmp(essid, prevSSID) == 0)) {
                        close(fd);
                        break;      // break out of case
                    }
                    strncpy(prevSSID, essid, IW_ESSID_MAX_SIZE);
                }
                close(fd);
            }
            *needs_dhcp_renew = 1;
            break;
        }
        default:
            break;
        }
        
        pos += iwe->len;
    }
}

static void __interpret_newlink_msg(struct nlmsghdr* hdr, int len,
                                    const char* if_name,
                                    int* needs_dhcp_renew,
                                    int* needs_gratuitous_arp)
{
    *needs_dhcp_renew = *needs_gratuitous_arp = 0;

    struct ifinfomsg *ifi = (struct ifinfomsg *)NLMSG_DATA(hdr);
    if (len < (int)sizeof(*ifi))
        // malformed netlink packet
        return;

    // iterate through message attributes
    const int ifihdrlen = NLMSG_ALIGN(sizeof(struct ifinfomsg));
    struct rtattr* attr = (struct rtattr *)(((char *)ifi) + ifihdrlen);
    int attrlen = len - (((char *)attr) - ((char *)hdr));

    for (; RTA_OK(attr, attrlen); attr = RTA_NEXT(attr, attrlen)) {
        if (attr->rta_type == IFLA_WIRELESS) {
            const char *data = (const char *)RTA_DATA(attr);
            const int data_len = RTA_PAYLOAD(attr);
            __interpret_ifla_wireless_msg(data, data_len, if_name,
                                          needs_dhcp_renew,
                                          needs_gratuitous_arp);
        }
    }
}

#else

static int __check_dev_name_attr(struct nlmsghdr* hdr,
                                 const char* wifi_if_name)
{
    // iterate through message attributes
    void* pv = NLMSG_DATA(hdr);

    char rgchDev[IFNAMSIZ + 1];

    struct rtattr* attr = (struct rtattr *)pv;
    int attrlen = NLMSG_PAYLOAD(hdr, 0);

    for (; RTA_OK(attr, attrlen); attr = RTA_NEXT(attr, attrlen)) {
        if (attr->rta_type == RWA_DEV_NAME) {
            // handle the device name attribute
            if (RTA_PAYLOAD(attr) <= IFNAMSIZ) {
                memset(rgchDev, 0, sizeof(rgchDev));
                memcpy(rgchDev, RTA_DATA(attr), RTA_PAYLOAD(attr));
                if (0 == strcmp(rgchDev, wifi_if_name))
                    return 1;
            }
        }
    }

    return 0;
}

static void __arp_if_assoc_changed(struct nlmsghdr* hdr,
                                   const char* wifi_if_name,
                                   const char* if_name,
                                   const unsigned char* if_hw_addr)
{
    // iterate through message attributes
    void* pv = NLMSG_DATA(hdr);

    int our_dev = 0;

    unsigned char assoc_mac[ETH_ALEN] = 
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const unsigned char empty_mac[ETH_ALEN] = 
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    char rgchDev[IFNAMSIZ + 1];

    struct rtattr* attr = (struct rtattr *)pv;
    int attrlen = NLMSG_PAYLOAD(hdr, 0);

    for (; RTA_OK(attr, attrlen); attr = RTA_NEXT(attr, attrlen)) {
        if (attr->rta_type == RWA_DEV_NAME) {
            // handle the device name attribute
            if (RTA_PAYLOAD(attr) <= IFNAMSIZ) {
                memset(rgchDev, 0, sizeof(rgchDev));
                memcpy(rgchDev, RTA_DATA(attr), RTA_PAYLOAD(attr));
                if (0 == strcmp(rgchDev, wifi_if_name))
                    our_dev = 1;
                else
                    break; // no reason to continue
            }
        } else if (attr->rta_type == RWA_ASSOC_MAC) {
            if (RTA_PAYLOAD(attr) == ETH_ALEN) {
                memcpy(assoc_mac, RTA_DATA(attr), ETH_ALEN);
            }
        }
    }

    if (our_dev && 0 != memcmp(assoc_mac, empty_mac, ETH_ALEN))
        rtnl_send_gratuitous_arp(if_name, if_hw_addr);
}

#endif

#endif

int rtnl_msg_triggers_dhcp_renew(struct rtnl_handle *rth,
                                 const char* wifi_if_name,
                                 const char* if_name,
                                 const unsigned char* if_hw_addr)
{
#if defined(__SONOS_LINUX__)
    struct sockaddr_nl sanl;
    socklen_t sanllen = sizeof(sanl);
    
    int amt;
    char buf[2048];
    amt = recvfrom(rth->fd, buf, sizeof(buf),
                   0, (struct sockaddr*)&sanl, &sanllen);
    if (amt < 0) {
        // some error reading from the netlink socket?
        return 0;
    }

    if (sanl.nl_family != AF_NETLINK) {
        // why is this bound socket returning random address families?
        return 0;
    }

    struct nlmsghdr* hdr = (struct nlmsghdr*)buf;
    size_t cb = (size_t)amt;

    for (; NLMSG_OK(hdr, cb); hdr = NLMSG_NEXT(hdr, cb)) {
        // process netlink message
#ifdef __SONOS_ARM__
        (void)wifi_if_name;

        if (sanl.nl_groups == RTMGRP_LINK) {
            // handle posssible Wireless Extensions message
            if (hdr->nlmsg_type == RTM_NEWLINK) {
                int needs_dhcp_renew, needs_gratuitous_arp;
                __interpret_newlink_msg(hdr, cb, if_name,
                                        &needs_dhcp_renew,
                                        &needs_gratuitous_arp);
                if (needs_gratuitous_arp)
                    rtnl_send_gratuitous_arp(if_name, if_hw_addr);
                if (needs_dhcp_renew) {
                    // REVIEW: could this early return cause us to miss
                    // a subsequent message that would set 
                    // needs_gratuitous_arp?
                    return 1;
                }
            }
#else
        if (sanl.nl_groups == RTMGRP_Rincon) {
            // handle Rincon-proprietary message
            if (hdr->nlmsg_type == RWM_SSID) {
                // OK, we saw a SSID change message; check to see if it
                // is for 'our' interface; if so, DHCP is retriggered.
                if (__check_dev_name_attr(hdr, wifi_if_name)) {
                    LOG(LOG_INFO, "Event: SSID changed - renew DHCP");
                    return 1;
                }
            } else if (hdr->nlmsg_type == RWM_MII) {
                /* If we get an MII signal during bootup (ie eth driver phy timer)
                we end up going through a 3 min aggressive cycle before autoip.
                This way we simply ignore MII for 2-3 seconds after startup */
                /* This also solves the problem were we get MII events before 
                the port is in forwarding mode. */
                if((time(NULL) - rth->seq) < 3) {
                    return 0;
                }
                // OK, we saw an MII port scan message; trigger DHCP
                LOG(LOG_INFO, "Event: Link changed - renew DHCP");
                return 1;
            } else if (hdr->nlmsg_type == RWM_RENEW_DHCP) {
                /* Use this event to renew DHCP without causing anacapa to
                restart.  This is used in station mode after we reassoc to 
                the AP. */
                LOG(LOG_INFO, "Event: Renew DHCP");
                return 1;
            } else if (hdr->nlmsg_type == RWM_ASSOC) {
                // we have an association; send a gratuitous ARP so that 
                // ethernet switches on the network know where we are
                __arp_if_assoc_changed(hdr, wifi_if_name, if_name, if_hw_addr);
                LOG(LOG_INFO, "Event: ASSOC");
            }
#endif
        } else {
            // where did this come from?
            break;
        }
    }
#else
    (void)rth;
    (void)wifi_if_name;
    (void)if_name;
    (void)if_hw_addr;
#endif

    return 0;
}
