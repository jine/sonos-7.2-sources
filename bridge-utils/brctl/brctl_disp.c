/*
 * Copyright (C) 2000 Lennert Buytenhek
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/autoconf.h>
#include <linux/version.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "libbridge.h"
#include "brctl.h"
#if !((LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)) && !defined(CONFIG_SONOS_BRIDGE_PROXY))
#include <arpa/inet.h> //please be aware that arpa/inet.h is needed by proxy
#endif // Any device except 3.10 without bridge proxy

void br_dump_bridge_id(unsigned char *x)
{
	printf("%.2x%.2x.%.2x%.2x%.2x%.2x%.2x%.2x", x[0], x[1], x[2], x[3],
	       x[4], x[5], x[6], x[7]);
}

void br_show_timer(struct timeval *tv)
{
	printf("%4i.%.2i", (int)tv->tv_sec, (int)tv->tv_usec/10000);
}

void br_dump_interface_list(struct bridge *br)
{
	char ifname[IFNAMSIZ];
	struct port *p;

	p = br->firstport;
	if (p != NULL) {
		printf("%s", if_indextoname(p->ifindex, ifname));
		p = p->next;
	}
	printf("\n");

	while (p != NULL) {
		printf("\t\t\t\t\t\t\t%s\n", if_indextoname(p->ifindex, ifname));
		p = p->next;
	}
}

void br_dump_port_info(struct port *p)
{
	char ifname[IFNAMSIZ];
	struct port_info *pi;

	pi = &p->info;

	printf("%s (%i)", if_indextoname(p->ifindex, ifname), p->index);
        if (pi->is_p2p) {
            printf(" - tunnel to %02X:%02X:%02X:%02X:%02X:%02X",
                   pi->p2p_dest_addr[0],
                   pi->p2p_dest_addr[1],
                   pi->p2p_dest_addr[2],
                   pi->p2p_dest_addr[3],
                   pi->p2p_dest_addr[4],
                   pi->p2p_dest_addr[5]);
            printf(" (remote STP state = %s, direct = %d)",
                   pi->remote_state == 0 ? "unknown" : 
                   br_get_state_name(pi->remote_state),
                   pi->direct_enabled);
        }
        printf("\n");
	printf(" port id\t\t%.4x\t\t\t", pi->port_id);
	printf("state\t\t\t%s\n", br_get_state_name(pi->state));
	printf(" designated root\t");
	br_dump_bridge_id((unsigned char *)&pi->designated_root);
	printf("\tpath cost\t\t%4i\n", pi->path_cost);

	printf(" designated bridge\t");
	br_dump_bridge_id((unsigned char *)&pi->designated_bridge);
	printf("\tmessage age timer\t");
	br_show_timer(&pi->message_age_timer_value);
	printf("\n designated port\t%.4x", pi->designated_port);
	printf("\t\t\tforward delay timer\t");
	br_show_timer(&pi->forward_delay_timer_value);
	printf("\n designated cost\t%4i", pi->designated_cost);
	printf("\t\t\thold timer\t\t");
	br_show_timer(&pi->hold_timer_value);
	printf("\n flags\t\t\t");
	if (pi->config_pending)
		printf("CONFIG_PENDING ");
	if (pi->top_change_ack)
		printf("TOPOLOGY_CHANGE_ACK ");
	printf("\n");
        printf("\n");
}

void br_dump_info(struct bridge *br)
{
	struct bridge_info *bri;
	struct port *p;

	bri = &br->info;

	printf("%s\n", br->ifname);
	if (!bri->stp_enabled) {
		printf(" STP is disabled for this interface\n");
		return;
	}

	printf(" bridge id\t\t");
	br_dump_bridge_id((unsigned char *)&bri->bridge_id);
	printf("\n designated root\t");
	br_dump_bridge_id((unsigned char *)&bri->designated_root);
	printf("\n root port\t\t%4i\t\t\t", bri->root_port);
	printf("path cost\t\t%4i\n", bri->root_path_cost);
	printf(" max age\t\t");
	br_show_timer(&bri->max_age);
	printf("\t\t\tbridge max age\t\t");
	br_show_timer(&bri->bridge_max_age);
	printf("\n hello time\t\t");
	br_show_timer(&bri->hello_time);
	printf("\t\t\tbridge hello time\t");
	br_show_timer(&bri->bridge_hello_time);
	printf("\n forward delay\t\t");
	br_show_timer(&bri->forward_delay);
	printf("\t\t\tbridge forward delay\t");
	br_show_timer(&bri->bridge_forward_delay);
	printf("\n ageing time\t\t");
	br_show_timer(&bri->ageing_time);
	printf("\t\t\tgc interval\t\t");
	br_show_timer(&bri->gc_interval);
	printf("\n hello timer\t\t");
	br_show_timer(&bri->hello_timer_value);
	printf("\t\t\ttcn timer\t\t");
	br_show_timer(&bri->tcn_timer_value);
	printf("\n topology change timer\t");
	br_show_timer(&bri->topology_change_timer_value);
	printf("\t\t\tgc timer\t\t");
	br_show_timer(&bri->gc_timer_value);
	printf("\n flags\t\t\t");
	if (bri->topology_change)
		printf("TOPOLOGY_CHANGE ");
	if (bri->topology_change_detected)
		printf("TOPOLOGY_CHANGE_DETECTED ");
	printf("\n");
	printf("\n");
	printf("\n");

	p = br->firstport;
	while (p != NULL) {
		br_dump_port_info(p);
		p = p->next;
	}
}

void br_dump_ports(struct bridge *br)
{
	struct port *p;

#ifdef CONFIG_SONOS_BRIDGE_PROXY
        printf("dev  port mac               u d satip\n");
#else
        printf("dev  port mac               u d\n");
#endif

        p = br->firstport;
	while (p != NULL) {

            	struct port_info *pi;
                pi = &p->info;

                if (pi->is_p2p) {
			char ifname[IFNAMSIZ];
#ifdef CONFIG_SONOS_BRIDGE_PROXY
			struct in_addr ip_addr;
			ip_addr.s_addr = pi->sat_ip;
			printf("%4s %-4d %02X:%02X:%02X:%02X:%02X:%02X %d %d %s\n",
#else
			printf("%4s %-4d %02X:%02X:%02X:%02X:%02X:%02X %d %d\n",
#endif
			       if_indextoname(p->ifindex, ifname),
			       p->index,
			       pi->p2p_dest_addr[0],
			       pi->p2p_dest_addr[1],
			       pi->p2p_dest_addr[2],
			       pi->p2p_dest_addr[3],
			       pi->p2p_dest_addr[4],
			       pi->p2p_dest_addr[5],
			       pi->is_uplink,
			       pi->direct_enabled
#ifdef CONFIG_SONOS_BRIDGE_PROXY
			       , (pi->sat_ip ? inet_ntoa(ip_addr) : "")
#endif
			       );
                }

		p = p->next;
	}

        printf("\n");
}

void br_dump_bcmc_hist(struct bridge_stats *stats)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct bridge_bcmc_hit *bcmc_hit;
	char type[10];
	unsigned char *src, *dest;

	printf("\n%11s  %17s  %17s  %11s  %9s\n",
		"Timestamp", "Source", "Destination", "Type(MC/BC)", "Count");

	int i;
	for (i = 0; i < BCMC_HIST_SIZE; i++) {
		bcmc_hit = (&(stats->bcmc_history[i]));

		if (bcmc_hit->packet_type == MULTICAST_TYPE) {
			strcpy(type, "Multicast");
		} else if (bcmc_hit->packet_type == BROADCAST_TYPE) {
			strcpy(type, "Broadcast");
		} else {
			strcpy(type, "");
		}

		src = bcmc_hit->src;
		dest = bcmc_hit->dest;

		printf("%11lu  %02x:%02x:%02x:%02x:%02x:%02x  %02x:%02x:%02x:%02x:%02x:%02x  %11s  %9lu\n",
			bcmc_hit->timestamp,
			src[0], src[1], src[2], src[3], src[4], src[5],
			dest[0], dest[1], dest[2], dest[3], dest[4], dest[5],
			type,
			bcmc_hit->packet_count);
	}
#endif
}

void br_dump_stats(struct bridge *br)
{
	printf("dev   rx_mc_peak   mc_peak_ts   rx_bc_peak   bc_peak_ts    rx_mc_hit    rx_bc_hit\n");
	printf("%-4s  %10u   %10u   %10u   %10u   %10u   %10u\n",
           br->ifname, br->stats.rx_mc_count_peak, br->stats.rx_mc_peak_ts,
           br->stats.rx_bc_count_peak, br->stats.rx_bc_peak_ts,
           br->stats.rx_mc_hit, br->stats.rx_bc_hit);

	br_dump_bcmc_hist(&(br->stats));
}
