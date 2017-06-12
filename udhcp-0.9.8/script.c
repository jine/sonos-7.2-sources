/* script.c
 *
 * Functions to call the DHCP client notification scripts 
 *
 * Russ Dill <Russ.Dill@asu.edu> July 2001
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

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

#include "options.h"
#include "dhcpd.h"
#include "dhcpc.h"
#include "packet.h"
#include "options.h"
#include "debug.h"

/* get a rough idea of how long an option will be (rounding up...) */
static int max_option_length[] = {
	[OPTION_IP] =		sizeof("255.255.255.255 "),
	[OPTION_IP_PAIR] =	sizeof("255.255.255.255 ") * 2,
	[OPTION_STRING] =	1,
	[OPTION_STRING_HOST] =  1,
	[OPTION_BOOLEAN] =	sizeof("yes "),
	[OPTION_U8] =		sizeof("255 "),
	[OPTION_U16] =		sizeof("65535 "),
	[OPTION_S16] =		sizeof("-32768 "),
	[OPTION_U32] =		sizeof("4294967295 "),
	[OPTION_S32] =		sizeof("-2147483684 "),
};


static int upper_length(int length, struct dhcp_option *option)
{
	return max_option_length[option->flags & TYPE_MASK] *
	       (length / option_lengths[option->flags & TYPE_MASK]);
}


static int sprintip(char *dest, const char *pre, const unsigned char *ip) {
	return sprintf(dest, "%s%d.%d.%d.%d ", pre, ip[0], ip[1], ip[2], ip[3]);
}


/* Check if a given label represents a valid DNS label
 * Return pointer to the first character after the label upon success,
 * NULL otherwise.
 * See RFC1035, 2.3.1
 */
/* We don't need to be particularly anal. For example, allowing _, hyphen
 * at the end, or leading and trailing dots would be ok, since it
 * can't be used for attacks. (Leading hyphen can be, if someone uses
 * cmd "$hostname"
 * in the script: then hostname may be treated as an option) 
 */
static const char *valid_domain_label(const char *label)
{
	unsigned char ch;
	unsigned pos = 0;

	for (;;) {
		ch = *label;
		if (((ch|0x20) < 'a' || (ch|0x20) > 'z') &&
				(ch < '0' || ch > '9')) {
			if (pos == 0) {
				/* label must begin with letter, but we allow digits too */
				return NULL;
			}
			if (ch == '\0' || ch == '.')  {
				return label;
			}
			/* DNS allows only '-', but we are more permissive */
			if (ch != '-' && ch != '_') {
				return NULL;
			}
		}
		label++;
		pos++;
		//Do we want this?
		//if (pos > 63) /* NS_MAXLABEL; labels must be 63 chars or less */
		//	return NULL;
	}
}

/* Check if a given name represents a valid DNS name */
/* See RFC1035, 2.3.1 */
static int good_hostname(const char *name)
{
	//const char *start = name;

	for (;;) {
		name = valid_domain_label(name);
		if (!name)
			return 0;
		if (!name[0])
			return 1;
			//Do we want this?
			//return ((name - start) < 1025); /* NS_MAXDNAME */
		name++;
	}
}


/* Fill dest with the text of option 'option'. */
static void fill_options(char *dest, unsigned char *option, struct dhcp_option *type_p)
{
	int type, optlen;
	u_int16_t val_u16;
	int16_t val_s16;
	u_int32_t val_u32;
	int32_t val_s32;
	int len = option[OPT_LEN - 2];

	dest += sprintf(dest, "%s=", type_p->name);

	type = type_p->flags & TYPE_MASK;
	optlen = option_lengths[type];
	for(;;) {
		uint32_t ipp_off = 0;
		if ((optlen > len) || (len % optlen != 0)) {
			/* short circuit because the data format doesn't match */
			return;
		}
		switch (type) {
		case OPTION_IP_PAIR:
			dest += sprintip(dest, "", option);
			*(dest++) = '/';
			ipp_off = 4;
		case OPTION_IP:	/* Works regardless of host byte order. */
			dest += sprintip(dest, "", option + ipp_off);
 			break;
		case OPTION_BOOLEAN:
			dest += sprintf(dest, *option ? "yes " : "no ");
			break;
		case OPTION_U8:
			dest += sprintf(dest, "%u ", *option);
			break;
		case OPTION_U16:
			memcpy(&val_u16, option, 2);
			dest += sprintf(dest, "%u ", ntohs(val_u16));
			break;
		case OPTION_S16:
			memcpy(&val_s16, option, 2);
			dest += sprintf(dest, "%d ", ntohs(val_s16));
			break;
		case OPTION_U32:
			memcpy(&val_u32, option, 4);
			dest += sprintf(dest, "%lu ", (unsigned long) ntohl(val_u32));
			break;
		case OPTION_S32:
			memcpy(&val_s32, option, 4);
			dest += sprintf(dest, "%ld ", (long) ntohl(val_s32));
			break;
		case OPTION_STRING:
		case OPTION_STRING_HOST:
			memcpy(dest, option, len);
			dest[len] = '\0';
			if (type == OPTION_STRING_HOST && !good_hostname(dest)) {
				strncpy(dest, "bad", len);
			}
			return;	 /* Short circuit this case */
		}
		option += optlen;
		len -= optlen;
		if (len <= 0) break;
	}
}


static const char *find_env(const char *prefix, const char *defaultstr)
{
	extern char **environ;
	char **ptr;
	const int len = strlen(prefix);

	for (ptr = environ; *ptr != NULL; ptr++) {
		if (strncmp(prefix, *ptr, len) == 0)
			return *ptr;
	}
	return defaultstr;
}

#define ZERONET(x)	(((x) & htonl(0xff000000)) == htonl(0x00000000))

/* Determine a default network mask, based on the IP address. */
static inline int inet_abc_len(u_int32_t addr)
{
  	if (ZERONET(addr))
  		return 0;

  	addr = ntohl(addr);
  	if (IN_CLASSA(addr)) 
  		return 8;
  	if (IN_CLASSB(addr)) 
  		return 16;
  	if (IN_CLASSC(addr)) 
  		return 24;

	/*
	 *	Something else, probably a multicast. 
	 */
  	 
  	return -1;
}

static inline u_int32_t inet_make_mask(int logmask)
{
	if (logmask)
		return htonl(~((1<<(32-logmask))-1));
	return 0;
}

/* put all the paramaters into an environment */
static char **fill_envp(struct dhcpMessage *packet)
{
	int num_options = 0;
	int i, j;
	char **envp;
	unsigned char *temp;
	char over = 0;
	int bNeedNetmask = 0;
	int bNeedBroadcast = 0;

	if (packet == NULL)
		num_options = 0;
	else {
		for (i = 0; options[i].code; i++)
			if (get_option(packet, options[i].code))
				num_options++;

		if (!get_option(packet, DHCP_SUBNET)) {
			bNeedNetmask = 1;
			num_options++;
		}
		if (!get_option(packet, DHCP_BROADCAST)) {
			bNeedBroadcast = 1;
			num_options++;
		}
		if (packet->siaddr) num_options++;
		if ((temp = get_option(packet, DHCP_OPTION_OVER)) && temp[OPT_LEN - 2] == 1)
			over = *temp;
		if (!(over & FILE_FIELD) && packet->file[0]) num_options++;
		if (!(over & SNAME_FIELD) && packet->sname[0]) num_options++;		
	}
	
	envp = xmalloc((num_options + 5) * sizeof(char *));
	envp[0] = xmalloc(sizeof("interface=") + strlen(client_config.interface));
	sprintf(envp[0], "interface=%s", client_config.interface);
	envp[1] = (char*)find_env("PATH", "PATH=/bin:/usr/bin:/sbin:/usr/sbin");
	envp[2] = (char*)find_env("HOME", "HOME=/");

	if (packet == NULL) {
		envp[3] = NULL;
		return envp;
	}

	envp[3] = xmalloc(sizeof("ip=255.255.255.255"));
	sprintip(envp[3], "ip=", (unsigned char *) &packet->yiaddr);
	for (i = 0, j = 4; options[i].code; i++) {
		if ((temp = get_option(packet, options[i].code))) {
			envp[j] = xmalloc(upper_length(temp[OPT_LEN - 2], &options[i]) + strlen(options[i].name) + 2);
			fill_options(envp[j], temp, &options[i]);
			j++;
		}
	}
	
	u_int32_t ipaddr = packet->yiaddr;
	u_int32_t netmask;
	int bNetmaskSet = 0;
	if (!bNeedNetmask) {
		unsigned char* tmpNet = get_option(packet, DHCP_SUBNET);
		if (tmpNet[OPT_LEN - 2] == sizeof(netmask)) {
			memcpy(&netmask, tmpNet, sizeof(netmask));
			bNetmaskSet = 1;
		}
	} else if (bNeedNetmask) {
		int prefixlen = inet_abc_len(ipaddr);
		if (prefixlen != -1) {
			netmask = inet_make_mask(prefixlen);
			bNetmaskSet = 1;
			/* add the netmask */
			envp[j] = xmalloc(sizeof("subnet=255.255.255.255"));
			sprintip(envp[j++], "subnet=", (unsigned char *) &netmask);
		}
	}
	if (bNeedBroadcast && bNetmaskSet) {
		u_int32_t broadcast = ipaddr|~netmask;
		/* add the broadcast */
		envp[j] = xmalloc(sizeof("broadcast=255.255.255.255"));
		sprintip(envp[j++], "broadcast=", (unsigned char *) &broadcast);
	}
	
	if (packet->siaddr) {
		envp[j] = xmalloc(sizeof("siaddr=255.255.255.255"));
		sprintip(envp[j++], "siaddr=", (unsigned char *) &packet->siaddr);
	}
	if (!(over & FILE_FIELD) && packet->file[0]) {
		/* watch out for invalid packets */
		packet->file[sizeof(packet->file) - 1] = '\0';
		envp[j] = xmalloc(sizeof("boot_file=") + strlen((const char*)packet->file));
		sprintf(envp[j++], "boot_file=%s", packet->file);
	}
	if (!(over & SNAME_FIELD) && packet->sname[0]) {
		/* watch out for invalid packets */
		packet->sname[sizeof(packet->sname) - 1] = '\0';
		envp[j] = xmalloc(sizeof("sname=") + strlen((const char*)packet->sname));
		sprintf(envp[j++], "sname=%s", packet->sname);
	}	
	envp[j] = NULL;
	return envp;
}

/* Call a script with a par file and env vars */
void run_script(struct dhcpMessage *packet, const char *name)
{
	int pid;
	const char* interestingParams[] =
		{"dns=", "router=", "serverid=", "subnet=", NULL};
	size_t i, j;
	char **envp;
	char params[256];
	params[0] = '\0';

	if (client_config.script == NULL)
		return;

	/* call script */
	pid = fork();
	if (pid) {
		waitpid(pid, NULL, 0);
		return;
	} else if (pid == 0) {
		envp = fill_envp(packet);

		/* close fd's? */

		// loop through the constructed environment strings, filtering out the ones we're interested in
		// I wish there was an easy and nicer way of doing this
		for (i = 0; envp[i]; i++) {
			for (j = 0; interestingParams[j]; j++){
				if(strncmp(envp[i], interestingParams[j], strlen(interestingParams[j])) == 0 &&
					(strlen(envp[i]) + strlen(params) + strlen("[] ") + 1 < sizeof(params))) {

					strcat(params, "[");
					strcat(params, envp[i]);
					strcat(params, "] ");

					break;
				}
			}
		}

		/* exec script */
		LOG(LOG_INFO, "%s parameters: %s", name, params);

		execle(client_config.script, client_config.script,
			   name, NULL, envp);
		LOG(LOG_ERR, "script %s failed: %s",
			client_config.script, strerror(errno));
		exit(1);
	}			
}

/* Alternate form of run_script that takes only IP address and subnet mask */
void run_script_zeroconf(const char* ip, const char* netmask, const char *name)
{
    int pid;

    if (client_config.script == NULL)
        return;

    /* call script */
    pid = fork();
    if (pid) {
        waitpid(pid, NULL, 0);
        return;
    } else if (pid == 0) {
        /* set up the environment variable vector */
        char* envp[8];

        /* interface being configured */
        char str_if[64];
        snprintf(str_if, 64, "interface=%s", client_config.interface);
        envp[0] = str_if;
		
        /* stock environment variables (copied from fill_envp) */
        envp[1] = (char*)find_env("PATH", "PATH=/bin:/usr/bin:/sbin:/usr/sbin");
        envp[2] = (char*)find_env("HOME", "HOME=/");

        /* IP address */
        char str_ip[64];
        snprintf(str_ip, 64, "ip=%s", ip);
        envp[3] = str_ip;

        /* netmask */
        char str_mask[64];
        snprintf(str_mask, 64, "subnet=%s", netmask);
        envp[4] = str_mask;

        /* terminate list with null */
        envp[5] = 0;

        /* close fd's? */
		
        /* exec script */
        DEBUG(LOG_INFO, "zeroconf execle'ing %s", client_config.script);
        execle(client_config.script, client_config.script,
               name, NULL, envp);
        LOG(LOG_ERR, "script %s failed: %s",
            client_config.script, strerror(errno));
        exit(1);
    }			
}
