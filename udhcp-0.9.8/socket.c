/*
 * socket.c -- DHCP server client/server socket creation
 *
 * udhcp client/server
 * Copyright (C) 1999 Matthew Ramsay <matthewr@moreton.com.au>
 *			Chris Trew <ctrew@moreton.com.au>
 *
 * Rewrite by Russ Dill <Russ.Dill@asu.edu> July 2001
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <errno.h>
#include <features.h>
#if __GLIBC__ >=2 && __GLIBC_MINOR >= 1
#include <netpacket/packet.h>
#include <net/ethernet.h>
#else
#include <asm/types.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#endif
#if defined(__SONOS_LINUX__)
#include <linux/if_bridge.h>
#include <linux/sockios.h>

// Copy KLUDGE from enet_port_stats.cxx: 
// No good place to pull these typedefs from, needed by ethtool.h.
typedef unsigned long long u64;
typedef unsigned int       u32;
typedef unsigned short     u16;
typedef unsigned char       u8;

#include <linux/ethtool.h>
#endif

#include "debug.h"

int read_interface(const char *interface, int *ifindex, u_int32_t *addr, unsigned char *arp)
{
	int fd;
	struct ifreq ifr;
	struct sockaddr_in *our_ip;

	memset(&ifr, 0, sizeof(struct ifreq));
	if((fd = socket(AF_INET, SOCK_RAW, IPPROTO_RAW)) >= 0) {
		ifr.ifr_addr.sa_family = AF_INET;
		strcpy(ifr.ifr_name, interface);

		if (addr) { 
			if (ioctl(fd, SIOCGIFADDR, &ifr) == 0) {
				our_ip = (struct sockaddr_in *) &ifr.ifr_addr;
				*addr = our_ip->sin_addr.s_addr;
				DEBUG(LOG_INFO, "%s (our ip) = %s", ifr.ifr_name, inet_ntoa(our_ip->sin_addr));
			} else {
				LOG(LOG_ERR, "SIOCGIFADDR failed, is the interface up and configured?: %s", 
						strerror(errno));
				close(fd);
				return -1;
			}
		}
		
		if (ioctl(fd, SIOCGIFINDEX, &ifr) == 0) {
			DEBUG(LOG_INFO, "adapter index %d", ifr.ifr_ifindex);
			*ifindex = ifr.ifr_ifindex;
		} else {
			LOG(LOG_ERR, "SIOCGIFINDEX failed!: %s", strerror(errno));
			close(fd);
			return -1;
		}
		if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) {
			memcpy(arp, ifr.ifr_hwaddr.sa_data, 6);
			LOG(LOG_INFO, "Interface: %s Address: %02x:%02x:%02x:%02x:%02x:%02x",
                            interface, arp[0], arp[1], arp[2], arp[3], arp[4], arp[5]);
		} else {
			LOG(LOG_ERR, "SIOCGIFHWADDR failed!: %s", strerror(errno));
			close(fd);
			return -1;
		}
	} else {
		LOG(LOG_ERR, "socket failed!: %s", strerror(errno));
		return -1;
	}
	close(fd);
	return 0;
}


int listen_socket(unsigned int ip, int port, const char *inf)
{
	struct ifreq interface;
	int fd;
	struct sockaddr_in addr;
	int n = 1;

	DEBUG(LOG_INFO, "Opening listen socket on 0x%08x:%d %s\n", ip, port, inf);
	if ((fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		DEBUG(LOG_ERR, "socket call failed: %s", strerror(errno));
		return -1;
	}
	
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = ip;

	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *) &n, sizeof(n)) == -1) {
		close(fd);
		return -1;
	}
	if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (char *) &n, sizeof(n)) == -1) {
		close(fd);
		return -1;
	}

	strncpy(interface.ifr_ifrn.ifrn_name, inf, IFNAMSIZ-1);
	interface.ifr_ifrn.ifrn_name[IFNAMSIZ-1] = '\0';
	if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE,(char *)&interface, sizeof(interface)) < 0) {
		close(fd);
		return -1;
	}

	if (bind(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr)) == -1) {
		close(fd);
		return -1;
	}
	
	return fd;
}


int raw_socket(int ifindex)
{
	int fd;
	struct sockaddr_ll sock;

	DEBUG(LOG_INFO, "Opening raw socket on ifindex %d\n", ifindex);
	if ((fd = socket(PF_PACKET, SOCK_DGRAM, htons(ETH_P_IP))) < 0) {
		DEBUG(LOG_ERR, "socket call failed: %s", strerror(errno));
		return -1;
	}
	
	sock.sll_family = AF_PACKET;
	sock.sll_protocol = htons(ETH_P_IP);
	sock.sll_ifindex = ifindex;
	if (bind(fd, (struct sockaddr *) &sock, sizeof(sock)) < 0) {
		DEBUG(LOG_ERR, "bind call failed: %s", strerror(errno));
		close(fd);
		return -1;
	}

	return fd;
}


#if defined(__SONOS_LINUX__) && !defined(SONOS_ARCH_ARM)
int bridge_ready(const char *interface)
{
    char valid = 0;
    int fd;

    if ((fd = socket(AF_INET, SOCK_RAW, IPPROTO_RAW)) >= 0) {
        unsigned long args[4];
        struct ifreq ifr;

        args[0] = (unsigned long)BRCTL_GET_ANY_FORWARDING;
        args[1] = (unsigned long)&valid;

        memset(&ifr, 0, sizeof(struct ifreq));
        strcpy(ifr.ifr_name, interface);
        ((unsigned long *)(&ifr.ifr_data))[0] = (unsigned long)args;

        if (ioctl(fd, SIOCDEVPRIVATE, &ifr) != 0) {
            /* this should never happen, but if it does, treat as success
               so we fall back to the original behavior */
            DEBUG(LOG_ERR, "bridge ioctl failed");
            valid = 1;
        }

        close(fd);
    } else {
        /* if we can't get a socket, don't bother waiting any longer */
        DEBUG(LOG_ERR, "socket call failed: %s", strerror(errno));
        valid = 1;
    }

    return valid;
}

/* shamelessly borrowed from netstartd */
static int get_carrier(int device)
{
    int has_carrier = 0;
    int fd;

    if ((fd = socket(AF_INET, SOCK_RAW, IPPROTO_RAW)) >= 0) {
        struct ifreq ifr;
        struct ethtool_cmd cmd;

        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = ETHTOOL_GSET;

        memset(&ifr, 0, sizeof(struct ifreq));
        ifr.ifr_data = (char *)&cmd;
        sprintf((char *)(&ifr.ifr_name), "eth%d", device);

        /* driver sets speed to 0 if link is down... */
        if (ioctl(fd, SIOCETHTOOL, &ifr) == 0) {
            has_carrier = (cmd.speed != 0);
        }

        close(fd);
    }

    return has_carrier;
}

int ethernet_available()
{
    unsigned int i;

    // we only support 8 ports
    for (i = 0; i < 8; i++) {
        if (get_carrier(i) > 0) {
            return 1;
        }
    }

    return 0;
}
#endif

