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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <asm/param.h>
#include "libbridge.h"
#include "brctl.h"

void br_cmd_addbr(struct bridge *br, char *brname, char *arg1, char *arg2)
{
	int err;

	if ((err = br_add_bridge(brname)) == 0)
		return;

	switch (err) {
	case EEXIST:
		fprintf(stderr,	"device %s already exists; can't create "
			"bridge with the same name\n", brname);
		break;

	default:
		perror("br_add_bridge");
		break;
	}
}

void br_cmd_delbr(struct bridge *br, char *brname, char *arg1, char *arg2)
{
	int err;

	if ((err = br_del_bridge(brname)) == 0)
		return;

	switch (err) {
	case ENXIO:
		fprintf(stderr, "bridge %s doesn't exist; can't delete it\n",
			brname);
		break;

	case EBUSY:
		fprintf(stderr, "bridge %s is still up; can't delete it\n",
			brname);
		break;

	default:
		perror("br_del_bridge");
		break;
	}
}

void br_cmd_addif(struct bridge *br, char *ifname, char *arg1, char *arg2)
{
	int err;
	int ifindex;

	ifindex = if_nametoindex(ifname);
	if (!ifindex) {
		fprintf(stderr, "interface %s does not exist!\n", ifname);
		return;
	}

	if ((err = br_add_interface(br, ifindex)) == 0)
		return;

	switch (err) {
	case EBUSY:
		fprintf(stderr,	"device %s is already a member of a bridge; "
			"can't enslave it to bridge %s.\n", ifname,
			br->ifname);
		break;

	case ELOOP:
		fprintf(stderr, "device %s is a bridge device itself; "
			"can't enslave a bridge device to a bridge device.\n",
			ifname);
		break;

	default:
		perror("br_add_interface");
		break;
	}
}

int __parse_mac_address(const char *mac_address, unsigned char *daddr)
{
	char stripped_mac[16];
	int i, j;

	/* Strip out ':' and '-' */
	i = j = 0;
	while ((i < sizeof(stripped_mac)-1) && mac_address[j]) {
		char ch = mac_address[j++];
		if (ch != ':' && ch != '-') {
			stripped_mac[i++] = ch;
		}
	}
	stripped_mac[i] = '\0';
	mac_address = (const char *)&stripped_mac[0];

        /* the MAC address should be 12 characters long, and all characters
           should be hex digits */
        if (strlen(mac_address) != 12)
                return -1;

        while (*mac_address) {
                char rgch[3];
                rgch[0] = *mac_address;
                rgch[1] = *(mac_address + 1);
                rgch[2] = '\0';
                *daddr = (unsigned char)strtol(rgch, 0, 16);

                daddr++;
                mac_address += 2;
        }

        return 0;
}

void br_cmd_addtunnel(struct bridge *br, char *ifname, char *daddr, char *wt)
{
	int err;
	int ifindex;
        unsigned char rgchaddr[6];

	ifindex = if_nametoindex(ifname);
	if (!ifindex) {
		fprintf(stderr, "interface %s does not exist!\n", ifname);
		return;
	}

        if (0 != __parse_mac_address(daddr, rgchaddr)) {
		fprintf(stderr, "cannot parse MAC address!\n");
		return;            
        }

	if ((err = br_add_p2p_tunnel(br, ifindex, rgchaddr, atoi(wt))) == 0)
		return;

	switch (err) {

	default:
		perror("br_add_p2p_tunnel");
		break;
        }        
}

void br_cmd_settunnelpc(struct bridge *br, char *ifname, char *daddr, char *wt)
{
	int err;
	int ifindex;
        unsigned char rgchaddr[6];

	ifindex = if_nametoindex(ifname);
	if (!ifindex) {
		fprintf(stderr, "interface %s does not exist!\n", ifname);
		return;
	}

        if (0 != __parse_mac_address(daddr, rgchaddr)) {
		fprintf(stderr, "cannot parse MAC address!\n");
		return;            
        }

	if ((err = br_set_p2p_tunnel_path_cost(br, ifindex, rgchaddr, 
                                               atoi(wt))) == 0)
		return;

	switch (err) {

	default:
		perror("br_set_p2p_tunnel_path_cost");
		break;
        }        
}

void br_cmd_deltunnel(struct bridge *br, char *ifname, char *daddr, char *arg2)
{
	int err;
	int ifindex;
        unsigned char rgchaddr[6];

	ifindex = if_nametoindex(ifname);
	if (!ifindex) {
		fprintf(stderr, "interface %s does not exist!\n", ifname);
		return;
	}

        if (0 != __parse_mac_address(daddr, rgchaddr)) {
		fprintf(stderr, "cannot parse MAC address!\n");
		return;            
        }        

	if ((err = br_del_p2p_tunnel(br, ifindex, rgchaddr)) == 0)
		return;

	switch (err) {

	default:
		perror("br_del_p2p_tunnel");
		break;
        }        
}

void br_cmd_delif(struct bridge *br, char *ifname, char *arg1, char *arg2)
{
	int err;
	int ifindex;

	ifindex = if_nametoindex(ifname);
	if (!ifindex) {
		fprintf(stderr, "interface %s does not exist!\n", ifname);
		return;
	}

	if ((err = br_del_interface(br, ifindex)) == 0)
		return;

	switch (err) {
	case EINVAL:
		fprintf(stderr, "device %s is not a slave of %s\n",
			ifname, br->ifname);
		break;

	default:
		perror("br_del_interface");
		break;
	}
}

void br_cmd_setageing(struct bridge *br, char *time, char *arg1, char *arg2)
{
	double secs;
	struct timeval tv;

	sscanf(time, "%lf", &secs);
	tv.tv_sec = secs;
	tv.tv_usec = 1000000 * (secs - tv.tv_sec);
	br_set_ageing_time(br, &tv);
}

void br_cmd_setbridgeprio(struct bridge *br, char *_prio, char *arg1, char *arg2)
{
	int prio;

	sscanf(_prio, "%i", &prio);
	br_set_bridge_priority(br, prio);
}

void br_cmd_setfd(struct bridge *br, char *time, char *arg1, char *arg2)
{
	double secs;
	struct timeval tv;

	sscanf(time, "%lf", &secs);
	tv.tv_sec = secs;
	tv.tv_usec = 1000000 * (secs - tv.tv_sec);
	br_set_bridge_forward_delay(br, &tv);
}

void br_cmd_setgcint(struct bridge *br, char *time, char *arg1, char *arg2)
{
	double secs;
	struct timeval tv;

	sscanf(time, "%lf", &secs);
	tv.tv_sec = secs;
	tv.tv_usec = 1000000 * (secs - tv.tv_sec);
	br_set_gc_interval(br, &tv);
}

void br_cmd_sethello(struct bridge *br, char *time, char *arg1, char *arg2)
{
	double secs;
	struct timeval tv;

	sscanf(time, "%lf", &secs);
	tv.tv_sec = secs;
	tv.tv_usec = 1000000 * (secs - tv.tv_sec);
	br_set_bridge_hello_time(br, &tv);
}

void br_cmd_setmaxage(struct bridge *br, char *time, char *arg1, char *arg2)
{
	double secs;
	struct timeval tv;

	sscanf(time, "%lf", &secs);
	tv.tv_sec = secs;
	tv.tv_usec = 1000000 * (secs - tv.tv_sec);
	br_set_bridge_max_age(br, &tv);
}

void br_cmd_setpathcost(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	int cost;
	struct port *p;

	if ((p = br_find_port(br, arg0)) == NULL) {
		fprintf(stderr, "can't find port %s in bridge %s\n", arg0, br->ifname);
		return;
	}

	sscanf(arg1, "%i", &cost);
	br_set_path_cost(p, cost);
}

void br_cmd_setportprio(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	int cost;
	struct port *p;

	if ((p = br_find_port(br, arg0)) == NULL) {
		fprintf(stderr, "can't find port %s in bridge %s\n", arg0, br->ifname);
		return;
	}

	sscanf(arg1, "%i", &cost);
	br_set_port_priority(p, cost);
}

static int arg_to_boolean(char *arg)
{
	if (!strcmp(arg, "on") || !strcmp(arg, "yes") || !strcmp(arg, "1"))
		return 1;

	return 0;
}

void br_cmd_stp(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	br_set_stp_state(br, arg_to_boolean(arg0));
}

void br_cmd_uplink(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	br_set_uplink_mode(br, arg_to_boolean(arg0));
}

#ifdef CONFIG_SONOS_DIAGS
void br_cmd_forwarding(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	br_set_forwarding_state(br, arg_to_boolean(arg0));
}
#endif

void br_cmd_showstp(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	br_dump_info(br);
}

void br_cmd_showports(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	br_dump_ports(br);
}

void br_cmd_show(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	printf("bridge name\tbridge id\t\tSTP enabled\tinterfaces\n");
	br = bridge_list;
	while (br != NULL) {
		printf("%s\t\t", br->ifname);
		br_dump_bridge_id((unsigned char *)&br->info.bridge_id);
		printf("\t%s\t\t", br->info.stp_enabled?"yes":"no");
		br_dump_interface_list(br);

		br = br->next;
	}
}

static int compare_fdbs(const void *_f0, const void *_f1)
{
	const struct fdb_entry *f0 = _f0;
	const struct fdb_entry *f1 = _f1;

#if 0
	if (f0->port_no < f1->port_no)
		return -1;

	if (f0->port_no > f1->port_no)
		return 1;
#endif

	return memcmp(f0->mac_addr, f1->mac_addr, 6);
}

void __dump_fdb_entry(struct fdb_entry *f)
{
	printf("%3i\t", f->port_no);
	printf("%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\t",
	       f->mac_addr[0], f->mac_addr[1], f->mac_addr[2],
	       f->mac_addr[3], f->mac_addr[4], f->mac_addr[5]);
	printf("%s\t\t", f->is_local?"yes":"no");
	br_show_timer(&f->ageing_timer_value);
	printf("\n");
}

void br_cmd_showmacs(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	struct fdb_entry fdb[1024];
	int offset;

	printf("port no\tmac addr\t\tis local?\tageing timer\n");

	offset = 0;
	while (1) {
		int i;
		int num;

		num = br_read_fdb(br, fdb, offset, 1024);
		if (!num)
			break;

		qsort(fdb, num, sizeof(struct fdb_entry), compare_fdbs);

		for (i=0;i<num;i++)
			__dump_fdb_entry(fdb+i);

		offset += num;
	}

        printf("\nMulticast Forwarding DB\n\n");
        printf("mc mac\tport\tmember\t\t\tsecs left\n");

        offset = 0;
        while (1) {
                int i;
                int num;
                
                unsigned char buf[1024];
                num = br_read_mcast_fdb(br, buf, sizeof(buf), offset);
                if (!num)
                         break;

                unsigned char* p = buf;

                for (i = 0; i < num; i++) {
                    printf("%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
                           p[0], p[1], p[2], p[3], p[4], p[5]);
                    p += 6;

                    int num_ports;
                    memcpy(&num_ports, p, 4);
                    p += 4;

                    int j;
                    for (j = 0; j < num_ports; j++) {
                        int port_no;
                        memcpy(&port_no, p, 4);
                        p += 4;
                        if (port_no == -1)
                            printf("\tLocal\n");
                        else
                            printf("\tPort %d\n", port_no);

                        int num_macs;
                        memcpy(&num_macs, p, 4);
                        p += 4;
                        
                        int k;
                        for (k = 0; k < num_macs; k++) {
                            int secs_left;
                            memcpy(&secs_left, &p[6], 4);
                            
                            printf("\t\t%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\t%d\n",
                                   p[0], p[1], p[2], p[3], p[4], p[5],
                                   secs_left);

                            p += 10;
                        }
                    }
                }

                offset += num;
        }
}

void br_cmd_setmac(struct bridge *br, char *mac, char *arg1, char *arg2)
{
	int err;
        unsigned char rgchaddr[6];

        if (0 != __parse_mac_address(mac, rgchaddr)) {
		fprintf(stderr, "cannot parse MAC address!\n");
		return;            
        }

	if ((err = br_set_mac(br, rgchaddr)) == 0)
		return;

	switch (err) {
	default:
		perror("br_set_mac");
		break;
        }
}

void br_cmd_tunlstates(struct bridge *br, char *ifname, char *arg1, char *arg2)
{
    int err;
    int ifindex;

    ifindex = if_nametoindex(ifname);
    if (!ifindex) {
        fprintf(stderr, "interface %s does not exist!\n", ifname);
        return;
    }

    unsigned char state_data[32 * (6 + 1)];
    unsigned int state_data_len;
        
    if ((err = br_get_p2p_tunnel_states(br, ifindex, 
                                        state_data, &state_data_len)) == 0) {
        unsigned int ix;
        for (ix = 0; ix < state_data_len; ix += (6 + 1)) {
            unsigned char* rec = &state_data[ix];

            printf("%.2x:%.2x:%.2x:%.2x:%.2x:%.2x : %s\n",
                   rec[0], rec[1], rec[2], rec[3], rec[4],
                   rec[5], br_get_state_name(rec[6]));
        }

        return;
    }

    switch (err) {

    default:
        perror("br_set_p2p_tunnel_path_cost");
        break;
    }
}

void br_cmd_showstats(struct bridge *br, char *arg0, char *arg1, char *arg2)
{
	br_dump_stats(br);
}

static struct command commands[] = {
	{0, 1, "addbr", br_cmd_addbr},
	{1, 1, "addif", br_cmd_addif},
        {1, 3, "addtunnel", br_cmd_addtunnel},
	{0, 1, "delbr", br_cmd_delbr},
	{1, 1, "delif", br_cmd_delif},
        {1, 2, "deltunnel", br_cmd_deltunnel},
#ifdef CONFIG_SONOS_DIAGS
	{1, 1, "forwarding", br_cmd_forwarding},
#endif
	{1, 1, "setageing", br_cmd_setageing},
	{1, 1, "setbridgeprio", br_cmd_setbridgeprio},
	{1, 1, "setfd", br_cmd_setfd},
	{1, 1, "setgcint", br_cmd_setgcint},
	{1, 1, "sethello", br_cmd_sethello},
	{1, 1, "setmaxage", br_cmd_setmaxage},
	{1, 2, "setpathcost", br_cmd_setpathcost},
	{1, 2, "setportprio", br_cmd_setportprio},
        {1, 3, "settunlpc", br_cmd_settunnelpc},
	{0, 0, "show", br_cmd_show},
	{1, 0, "showmacs", br_cmd_showmacs},
	{1, 0, "showstp", br_cmd_showstp},
        {1, 0, "showports", br_cmd_showports},
	{1, 1, "stp", br_cmd_stp},
	{1, 1, "uplink", br_cmd_uplink},
        {1, 1, "setmac", br_cmd_setmac},
        {1, 1, "tunlstates", br_cmd_tunlstates},
	{1, 0, "showstats", br_cmd_showstats},
};

struct command *br_command_lookup(char *cmd)
{
	int i;
	int numcommands;

	numcommands = sizeof(commands)/sizeof(commands[0]);

	for (i=0;i<numcommands;i++)
		if (!strcmp(cmd, commands[i].name))
			return &commands[i];

	return NULL;
}
