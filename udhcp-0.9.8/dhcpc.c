/* dhcpc.c
 *
 * udhcp DHCP client
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
 
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/file.h>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/klog.h>

#include "dhcpd.h"
#include "dhcpc.h"
#include "options.h"
#include "clientpacket.h"
#include "packet.h"
#include "script.h"
#include "socket.h"
#include "debug.h"
#include "pidfile.h"
#include "nlhelp.h"
#include "zeroconf.h"

#include "event_reporter.h"

FILE* debug_file;

#ifdef __SONOS_LINUX__
#include <linux/rtnetlink.h>
#endif


#define ADDR_CONFLICT_DETECTION

#ifdef ADDR_CONFLICT_DETECTION
/* Socket descriptor used to listen on DHCP server port for conflicting DHCP
 * requests while in the bound-state.
 */
static int s_fd = -1;

static void open_server_socket(void);
static void close_server_socket(void);
#endif


static int state;
static unsigned long requested_ip; /* = 0 */
static unsigned long server_addr;
static time_t timeout;
static int packet_num; /* = 0 */
static int fd = -1;
static int signal_pipe[2];
static struct rtnl_handle nlh;

#define LISTEN_NONE 0
#define LISTEN_KERNEL 1
#define LISTEN_RAW 2
static int listen_mode;

static struct zc_config zc;

/* this is the time at which 'aggressive' search for a DHCP server ends */
static time_t time_aggressive_end;

/* this is the duration, in seconds, of the 'aggressive' search */
#define AGGR_DURATION  60
/* reducing AGGR_DURATION from 120 to 60  so that we wait for a DHCP_OFFER  longer after the 4th attempt */

/* this is the retry frequency of the 'aggressive' search, in seconds */
#define AGGR_INTERVAL    3
/* reducing AGGR_INTERVAL from 5 to 3 to conincide with extended listening time. Now 4 attempts */

/* this is the retry frequency of the 'normal' search, in seconds */
#define NORMAL_INTERVAL 54
/* reducing NORMAL_INTERVAL from 60 to 54 to make NORMAL_INTERVAL + 10 second listening time to be 64 sec */

/* this is the time to wait for a bridge interface is ready */
#define BRIDGE_WAIT_INTERVAL 20

/* this is the time to wait for ethernet to be initialized */
#define ETHERNET_WAIT_INTERVAL 3

/* this is the time we wait before trying to fallback to AutoIP on a failing
   renew call, to avoid bouncing too quickly */
static time_t time_zeroconf_fallback;

/* wait two minutes before falling back, one minute after the end of the
   aggressive timeout */
#define ZC_FALLBACK_DELAY 120

struct client_config_t client_config = {
    /* Default options. */
    abort_if_no_lease: 0,
    foreground: 0,
    quit_after_lease: 0,
    background_if_no_lease: 0,
    interface: "eth0",
    pidfile: "/tmp/udhcpc.pid",
    wifi: NULL,
    sdomain: NULL,
    script: NULL,
    clientid: NULL,
    hostname: NULL,
    ifindex: 0,
    arp: "\0\0\0\0\0\0",        /* appease gcc-3.0 */
};

enum stats_e {
    RENEW_ATTEMPT,
    RENEW_SUCCESS,
    RENEW_NAK,
    REQUEST_ATTEMPT,
    REQUEST_SUCCESS,
    REQUEST_NAK,
    DUPLICATION,
    ZEROCONF,
    NUM_STATS
};

#ifdef SONOS_ARCH_ATTR_SUPPORTS_EXTERNAL_EVENTS
static const char* stat_names[NUM_STATS] = {
    "renew_attempt",
    "renew_success",
    "renew_nak",
    "request_attempt",
    "request_success",
    "request_nak",
    "duplication",
    "zeroconf"
};
#endif

unsigned int stats[NUM_STATS];

static void dump_stats(void)
{
    LOG(LOG_INFO, "rna:%d rns:%d rnn:%d rqa:%d rqs:%d rqn:%d zc:%d dup:%d %s %s %s %s %s",
        stats[RENEW_ATTEMPT], stats[RENEW_SUCCESS], stats[RENEW_NAK],
        stats[REQUEST_ATTEMPT], stats[REQUEST_SUCCESS], stats[REQUEST_NAK],
        stats[ZEROCONF], stats[DUPLICATION],
        // Are renews succeeding less than half of the time?
        ((stats[RENEW_SUCCESS] < stats[RENEW_ATTEMPT] / 2) && (stats[RENEW_ATTEMPT] > 5)) ? "DHCP_RN_FAIL" : "",
        // Are requests succeeding less than half of the time?
        ((stats[REQUEST_SUCCESS] < stats[REQUEST_ATTEMPT] / 2) && (stats[REQUEST_ATTEMPT] > 5)) ? "DHCP_RQ_FAIL" : "",
        (stats[RENEW_NAK] > 0) ? "DHCP_RN_NAK" : "",
        (stats[REQUEST_NAK] > 0) ? "DHCP_RQ_NAK" : "",
        (stats[DUPLICATION] > 0) ? "DHCP_DUP" : ""
       );
}

static void inc_stats(enum stats_e s)
{
    stats[s]++;

    /* Log stats for every N renews, and for every
     * request and NAK. */
    if (s == RENEW_SUCCESS) {
	static int how_often = 0;
	how_often = (how_often + 1) % 5;
	if (how_often == 0) {
	    dump_stats();
	}
    } else if (s == REQUEST_NAK ||
	       s == RENEW_NAK ||
	       s == REQUEST_SUCCESS) {
	dump_stats();
    }
#ifdef SONOS_ARCH_ATTR_SUPPORTS_EXTERNAL_EVENTS
    struct event_context_t evctx;
    if (init_event_ctx(&evctx, "dhcp") ||
        add_pair_str(&evctx, "type", stat_names[s]) ||
        add_pair_uint(&evctx, "count", stats[s]) ||
        report_event(&evctx)) {
	    LOG(LOG_WARNING, "failed to report event");
    }
#endif
}

#ifndef BB_VER
static void show_usage(void)
{
#if 0 
    printf(
           "Usage: udhcpc [OPTIONS]\n\n"
           "  -c, --clientid=CLIENTID         Client identifier\n"
           "  -H, --hostname=HOSTNAME         Client hostname\n"
           "  -h                              Alias for -H\n"
           "  -f, --foreground                Do not fork after getting lease\n"
           "  -b, --background                Fork to background if lease cannot be\n"
           "                                  immediately negotiated.\n"
           "  -i, --interface=INTERFACE       Interface to use (default: eth0)\n"
           "  -d, --specialdomain=DOMAIN      Create /var/run/specialIP file if the\n"
           "                                  given domain name suffix matches.\n"
           "  -n, --now                       Exit with failure if lease cannot be\n"
           "                                  immediately negotiated.\n"
           "  -p, --pidfile=file              Store process ID of daemon in file\n"
           "  -q, --quit                      Quit after obtaining lease\n"
           "  -r, --request=IP                IP address to request (default: none)\n"
           "  -s, --script=file               Run file at dhcp events. \n"
           "  -w, --wifi                      Interface for SSID change monitor\n"
           "  -v, --version                   Display version\n"
           );
#endif
    exit(0);
}
#endif

/* Exit and cleanup */
static void exit_client(int retval)
{
    pidfile_delete(client_config.pidfile);
    dump_stats();
    CLOSE_LOG();
    exit(retval);
}


/* just a little helper */
static void change_mode(int new_mode)
{
    DEBUG(LOG_INFO, "Entering %s listen mode",
          new_mode ? (new_mode == 1 ? "kernel" : "raw") : "none");
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
    listen_mode = new_mode;
}

#ifdef ADDR_CONFLICT_DETECTION
static void open_server_socket()
{
    if (s_fd < 0) {
        /* Open up a socket to listen at SERVER_PORT in bound state */
        if (client_config.interface != NULL) {
            s_fd = listen_socket(INADDR_ANY, SERVER_PORT, client_config.interface);
            if (s_fd < 0) {
                LOG(LOG_ERR, "FATAL: couldn't listen on socket, %s", strerror(errno));
                exit_client(0);
            } else {
                LOG(LOG_DEBUG, "Opened address monitor");
            }
        }
    }
}

static void close_server_socket()
{
    if (s_fd >= 0){
        LOG(LOG_DEBUG,"Closing address monitor");
        close(s_fd);
        s_fd = -1;
    }
}
#endif

/* perform a renew */
static void perform_renew(void)
{
    int state_before = state;

#ifdef ADDR_CONFLICT_DETECTION
    close_server_socket();
#endif

    LOG(LOG_INFO, "Performing a DHCP renew");
    switch (state) {
    case BOUND:
        change_mode(LISTEN_KERNEL);
    case RENEWING:
    case REBINDING:
        state = RENEW_REQUESTED;
        inc_stats(RENEW_ATTEMPT);
        break;
    case RENEW_REQUESTED: /* impatient are we? fine, square 1 */
        run_script(NULL, "deconfig");
    case REQUESTING:
    case RELEASED:
        change_mode(LISTEN_RAW);
	inc_stats(REQUEST_ATTEMPT);
        state = INIT_SELECTING;
        break;
    case INIT_SELECTING:
        break;
    }

    /* start things over */
    packet_num = 0;

    /* Kill any timeouts because the user wants this to hurry along */
    timeout = 0;

    if (state_before == INIT_SELECTING) {
        /* If we haven't ever seen a DHCP server, and we are happily using a
           zeroconf adress, then 'renew' the zeroconf-assigned address, but 
           don't release it.  This avoids unnecessary IP address churn. */
        switch(zc.zc_state) {
        case ZEROCONF_DISABLED:
        case ZEROCONF_PROBING:
        default:
            zc_reset(&zc);
            break;
        case ZEROCONF_ANNOUNCING:
        case ZEROCONF_RUNNING:
            zc.zc_state = ZEROCONF_ANNOUNCING;
            zc.zc_count = 0;
            zc.zc_timeout = 0;
            break;
        }
    } else
        zc_reset(&zc);
}



///////////////////////////////////////////////////////////////////////////////

/* Signal handler */
static void signal_handler(int sig)
{
#ifdef __SONOS_MIPS__
    if (write(signal_pipe[1], &sig, sizeof(sig)) < 0) {
#else
    if (send(signal_pipe[1], &sig, sizeof(sig), MSG_DONTWAIT) < 0) {
#endif
        LOG(LOG_ERR, "Could not send signal: %s",
            strerror(errno));
    }
}

void background(void)
{
    DEBUG(LOG_INFO, "Backgrounding...");
    int pid_fd;

    /* hold lock during fork. */
    pid_fd = pidfile_acquire(client_config.pidfile); 
    /* don't let daemon close it */
    while (pid_fd >= 0 && pid_fd < 3) pid_fd = dup(pid_fd); 
    if (daemon(0, 0) == -1) {
        perror("fork");
        exit_client(1);
    }
    client_config.foreground = 1; /* Do not fork again. */
    pidfile_write_release(pid_fd);
}

#ifdef ADDR_CONFLICT_DETECTION
static int max_of_five(int w, int x, int y, int z, int z2)
{
    int max = w;

    if (x > max)
        max = x;
    if (y > max)
        max = y;
    if (z > max)
        max = z;
    if (z2 > max)
        max = z2;

    return max;
}
#else
static int max_of_four(int w, int x, int y, int z)
{
    int max = w;

    if (x > max)
        max = x;
    if (y > max)
        max = y;
    if (z > max)
        max = z;

    return max;
}
#endif



#ifdef COMBINED_BINARY
int udhcpc_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    unsigned char *temp, *message;
    unsigned long t1 = 0, t2 = 0, xid = 0;
    unsigned long start = 0, lease = 0;
    fd_set rfds;
    int retval;
    struct timeval tv;
    int c, len;
    struct dhcpMessage packet;
    struct in_addr temp_addr;
    int pid_fd;
    time_t now;
    int max_fd;
    int sig;
    int force_zc = 0;
    int ifwait = 1; /* wait for the interface to be ready */

    static struct option arg_options[] = {
        {"clientid",    required_argument,  0, 'c'},
        {"foreground",  no_argument,        0, 'f'},
        {"background",  no_argument,        0, 'b'},
        {"hostname",    required_argument,  0, 'H'},
        {"hostname",    required_argument,  0, 'h'},
        {"interface",   required_argument,  0, 'i'},
        {"now",         no_argument,        0, 'n'},
        {"pidfile",     required_argument,  0, 'p'},
        {"quit",        no_argument,        0, 'q'},
        {"request",     required_argument,  0, 'r'},
        {"script",      required_argument,  0, 's'},
        {"wifi",        required_argument,  0, 'w'},
        {"nowait",      no_argument,        0, 'z'},
        {"specialdomain", required_argument, 0, 'd'},
        {"version",     no_argument,        0, 'v'},
        {"help",        no_argument,        0, '?'},
        {0, 0, 0, 0}
    };

    /* get options */
    while (1) {
        int option_index = 0;
        c = getopt_long(argc, argv, "c:fbH:h:i:np:qr:s:w:d:vz", arg_options, 
                        &option_index);
        if (c == -1) break;
        
        switch (c) {
        case 'c':
            len = strlen(optarg) > 255 ? 255 : strlen(optarg);
            if (client_config.clientid) free(client_config.clientid);
            client_config.clientid = xmalloc(len + 2);
            client_config.clientid[OPT_CODE] = DHCP_CLIENT_ID;
            client_config.clientid[OPT_LEN] = len;
            client_config.clientid[OPT_DATA] = '\0';
            strncpy((char*)client_config.clientid + OPT_DATA, optarg, len);
            break;
        case 'f':
            client_config.foreground = 1;
            break;
        case 'b':
            client_config.background_if_no_lease = 1;
            break;
        case 'h':
        case 'H':
            len = strlen(optarg) > 255 ? 255 : strlen(optarg);
            if (client_config.hostname) free(client_config.hostname);
            client_config.hostname = xmalloc(len + 2);
            client_config.hostname[OPT_CODE] = DHCP_HOST_NAME;
            client_config.hostname[OPT_LEN] = len;
            strncpy((char*)client_config.hostname + 2, optarg, len);
            break;
        case 'i':
            client_config.interface = optarg;
            break;
        case 'n':
            client_config.abort_if_no_lease = 1;
            break;
        case 'd':
            client_config.sdomain = optarg;
            break;
        case 'p':
            client_config.pidfile = optarg;
            break;
        case 'q':
            client_config.quit_after_lease = 1;
            break;
        case 'r':
            requested_ip = inet_addr(optarg);
            break;
        case 's':
            client_config.script = optarg;
            break;
        case 'w':
            client_config.wifi = optarg;
            break;
        case 'v':
            printf("udhcpcd, version %s\n\n", VERSION);
            exit_client(0);
            break;
        case 'z':
            ifwait = 0;
            break;
        default:
            show_usage();
        }
    }

    /* if we wind up using a special IP address, ie, for
       Best Buy, this file will get created */
    unlink("/var/run/specialIP");

    /* if we wind up using a DHCP assigned address, this
       file will get created */
    unlink("/var/run/normalIP");

    OPEN_LOG("udhcpc");
    LOG(LOG_INFO, "udhcp client (v%s) started", VERSION);

    pid_fd = pidfile_acquire(client_config.pidfile);
    pidfile_write_release(pid_fd);

    if (read_interface(client_config.interface, &client_config.ifindex, 
                       NULL, client_config.arp) < 0)
        exit_client(1);
        
    if (!client_config.clientid) {
        client_config.clientid = xmalloc(6 + 3);
        client_config.clientid[OPT_CODE] = DHCP_CLIENT_ID;
        client_config.clientid[OPT_LEN] = 7;
        client_config.clientid[OPT_DATA] = 1;
        memcpy(client_config.clientid + 3, client_config.arp, 6);
    }

    /* Ignore user signals before 'bridge is now ready' while loop (MCS-1147) */
    signal(SIGUSR1, SIG_IGN);
    signal(SIGUSR2, SIG_IGN);

#if defined(__SONOS_LINUX__) && !defined(SONOS_ARCH_ARM)
    /* defer operations until the bridge interface is ready, otherwise we're just
     * sending packets to the bit bucket */
    if (ifwait && (0 == strncasecmp(client_config.interface, "br", 2))) {
        int has_netsettings = (0 == access("/jffs/netsettings.txt", F_OK));
        int iterations = 0;

        LOG(LOG_INFO, "Waiting for bridge to become ready...");
        while (iterations < BRIDGE_WAIT_INTERVAL && !bridge_ready(client_config.interface)) {
            /* when we are factory reset with no wires, we can assume auto-IP */
            if (!has_netsettings && (iterations >= ETHERNET_WAIT_INTERVAL) && !ethernet_available()) {
                force_zc = 1;
                break;
            }
            sleep(1);
            iterations++;
        }
        LOG(LOG_INFO, "Bridge is now ready");
    }
#else
    (void)ifwait;
#endif

    /* if appropriate, set up to listen for SSID change notifications. */
    if (client_config.wifi) {
#if defined(__SONOS_LINUX__)
#if defined(SONOS_ARCH_ARM)
        int ret = rtnl_open(&nlh, RTMGRP_LINK);
#else
        int ret = rtnl_open(&nlh, RTMGRP_Rincon);
#endif
#else
        int ret = -1;
#endif
        if (ret != 0) {
            LOG(LOG_ERR, "FATAL: couldn't listen for SSID change, %s", 
                strerror(errno));
            exit_client(0);                
        }
    } else
        nlh.fd = -1;

#ifdef __SONOS_MIPS__
    /* setup signal handlers */
    if (pipe(signal_pipe) != 0) {
        printf("SOCKET PAIR - ERRNO: %d\n",errno);
    }
#else
    /* setup signal handlers */
    socketpair(AF_UNIX, SOCK_STREAM, 0, signal_pipe);
#endif
    signal(SIGUSR1, signal_handler);
    signal(SIGUSR2, signal_handler);
    signal(SIGTERM, signal_handler);
    
    state = INIT_SELECTING;
    zc_init(&zc, client_config.arp);
    inc_stats(REQUEST_ATTEMPT);
    run_script(NULL, "deconfig");
    change_mode(LISTEN_RAW);

    time_aggressive_end = time(0) + AGGR_DURATION;
    /* On startup, fallback to zeroconf whenever we fail enough packets */
    time_zeroconf_fallback = time(0);

    /* there are cases when we can immediately pick a zeroconf address without probing */
    if (force_zc) {
        zc_start_probing(&zc, time(0));
        zc.zc_count = 0xff;
    }

    for (;;) {
        // we are juggling two state machines in parallel here; set the
        // timeout for the select to the sooner of the DHCP and zeroconf
        // timeouts. 
        if (zc.zc_timeout > timeout)
            tv.tv_sec = timeout - time(0);
        else
            tv.tv_sec = zc.zc_timeout - time(0);

        tv.tv_usec = 0;
        FD_ZERO(&rfds);

        if (listen_mode != LISTEN_NONE && fd < 0) {
            if (listen_mode == LISTEN_KERNEL)
                fd = listen_socket(INADDR_ANY, CLIENT_PORT, 
                                   client_config.interface);
            else
                fd = raw_socket(client_config.ifindex);
            if (fd < 0) {
                LOG(LOG_ERR, "FATAL: couldn't listen on socket, %s", 
                    strerror(errno));
                exit_client(0);
            }
        }

        if (zc.zc_state != ZEROCONF_DISABLED && zc.zc_fd < 0) {
            zc.zc_fd = zc_arp_socket(client_config.interface);
            if (zc.zc_fd < 0) {
                LOG(LOG_ERR, "FATAL: couldn't listen on ARP socket, %s", 
                    strerror(errno));
                exit_client(0);                        
            }
        }

        if (fd >= 0) FD_SET(fd, &rfds);
        if (zc.zc_fd >= 0) FD_SET(zc.zc_fd, &rfds);
        if (nlh.fd >= 0) FD_SET(nlh.fd, &rfds);
#ifdef ADDR_CONFLICT_DETECTION
        if (s_fd >= 0) FD_SET(s_fd, &rfds);
#endif
        FD_SET(signal_pipe[0], &rfds);

        if (tv.tv_sec > 0) {
#ifdef ADDR_CONFLICT_DETECTION
           max_fd = max_of_five(fd, zc.zc_fd, nlh.fd, signal_pipe[0], s_fd);
#else
           max_fd = max_of_four(fd, zc.zc_fd, nlh.fd, signal_pipe[0]);
#endif
            retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
        } else 
            retval = 0; /* If we already timed out, fall through */

        now = time(0);

        /* time-driven processing for DHCP state machine. */
        if (retval == 0 && timeout <= now) {
            switch (state) {
            case INIT_SELECTING:
                /* make sure we're listenting to all IP datagrams; if not,
                   start and get us to be called again (i.e. don't change 
                   the value of timeout. */
                if (listen_mode != LISTEN_RAW) {
                    change_mode(LISTEN_RAW);
                    goto out1;
                }

                if (packet_num < 3) {
                    if (packet_num == 0)
                        xid = random_xid();

                    /* send discover packet */
                    send_discover(xid, requested_ip); /* broadcast */
                    
                    /*equation must make timeout 2 then 4 then wait 10; old = ((packet_num == 2) ? 4 : 2) */
                    timeout = now + ((packet_num == 0) ? 2 : ((packet_num == 1) ? 4 : 10));
                    packet_num++;
                } else {
                    if (client_config.background_if_no_lease) {
                        LOG(LOG_INFO, "No lease, forking to background.");
                        background();
                    } else if (client_config.abort_if_no_lease) {
                        LOG(LOG_INFO, "No lease, failing.");
                        exit_client(1);
                    }
                    /* wait to try again */
                    packet_num = 0;
                    if (now < time_aggressive_end)
                        timeout = now + AGGR_INTERVAL;
                    else
                        timeout = now + NORMAL_INTERVAL;

                    /* while waiting for the next DHCP attempt, don't 
                       listen on the DHCP socket at all since the socket gets 
                       all IP datagrams and heavy network traffic can
                       saturate the CPU.
                    */
                    // As stated above, we would like to switch back to 
                    // LISTEN_NONE here now that we have timed out on this DHCP
                    // attempt. While we are in LISTEN_RAW mode, heavy network
                    // traffic results in local audio dropouts because the CPU
                    // cannot keep up with inspecting every incoming IP packet
                    // (bug 23686). However, in auto-ip mode we initiate a new
                    // DHCP attmempt every 60s and stay in LISTEN_RAW mode for
                    // up to 16s, so we would still be exposed to this issue about
                    // 20% of the time. Furthermore, the fix for ActionTec routers
                    // that has been in the field since 11/2008 kept us in
                    // LISTEN_RAW mode here, so there is some risk that switching
                    // back to LISTEN_NONE might break customers with ActionTec
                    // routers or other buggy DHCP servers that we work with today
                    // because staying in LISTEN_RAW allows the late arrival of
                    // DHCPOFFERs to still be processed. Afterall, our timeouts are
                    // much more aggressive than RFC2131 suggests. Leaving socket in
                    // LISTEN_RAW mode for now.
                    // 
                    // change_mode(LISTEN_NONE);

                    /* in the meantime, run zeroconf, if it isn't already */
                    /*  and it's been long enough                         */
                    if (zc.zc_state == ZEROCONF_DISABLED &&
                          now > time_zeroconf_fallback) {
                        LOG(LOG_INFO, "Starting zeroconf process");
			inc_stats(ZEROCONF);
                        zc_start_probing(&zc, now);
                    }
                }
                break;
            case RENEW_REQUESTED:
            case REQUESTING:
                if (packet_num < 3) {
                    /* send request packet */
                    if (state == RENEW_REQUESTED)
                        send_renew(xid, server_addr, requested_ip); /* ucast */
                    else 
                        send_selecting(xid, server_addr, requested_ip); /*bc*/
                    
                    /* original code had timeout = ((packet_num == 2) ? 10 : 2) */
                    timeout = now + ((packet_num == 0) ? 2 : ((packet_num == 1) ? 4 : 10));
                    packet_num++;
                } else {
                    /* timed out, go back to init state */
                    if (state == RENEW_REQUESTED) run_script(NULL, "deconfig");
                    packet_num = 0;
		    inc_stats(REQUEST_ATTEMPT);
                    state = INIT_SELECTING;
                    if (now < time_aggressive_end)
                        timeout = now + AGGR_INTERVAL;
                    else
                        timeout = now + NORMAL_INTERVAL;
                        
                    change_mode(LISTEN_RAW);
                }
                break;
            case BOUND:
#ifdef ADDR_CONFLICT_DETECTION
                close_server_socket();
#endif

                /* Lease is starting to run out, enter renewing state */
                state = RENEWING;
                inc_stats(RENEW_ATTEMPT);
                change_mode(LISTEN_KERNEL);
                DEBUG(LOG_INFO, "Entering renew state");
                /* fall right through */
            case RENEWING:
                /* Either set a new T1, or enter REBINDING state */
                if ((t2 - t1) <= (lease / 14400 + 1)) {
                    /* timed out, enter rebinding state */
                    state = REBINDING;
                    timeout = now + (t2 - t1);
                    DEBUG(LOG_INFO, "Entering rebinding state");
                } else {
                    /* send a request packet */
                    send_renew(xid, server_addr, requested_ip); /* unicast */
                    
                    t1 = (t2 - t1) / 2 + t1;
                    timeout = t1 + start;
                }
                break;
            case REBINDING:
                /* Either set a new T2, or enter INIT state */
                if ((lease - t2) <= (lease / 14400 + 1)) {
                    /* timed out, enter init state */
                    state = INIT_SELECTING;

                    LOG(LOG_INFO, "Lease lost, entering init state");
		    inc_stats(REQUEST_ATTEMPT);
                    run_script(NULL, "deconfig");
                    timeout = now;
                    packet_num = 0;
                    change_mode(LISTEN_RAW);
                } else {
                    /* send a request packet */
                    send_renew(xid, 0, requested_ip); /* broadcast */

                    t2 = (lease - t2) / 2 + t2;
                    timeout = t2 + start;
                }
                break;
            case RELEASED:
                /* yah, I know, *you* say it would never happen */
                timeout = 0x7fffffff;
                break;
            }
        out1:
            ;
        } 

        /* I/O-driven processing for DHCP state machine. */
        if (retval > 0 && listen_mode != LISTEN_NONE && 
                fd != -1 && FD_ISSET(fd, &rfds)) {
            if (listen_mode == LISTEN_KERNEL)
                len = get_packet(&packet, fd);
            else 
                len = get_raw_packet(&packet, fd);
            
            if (len == -1 && errno != EINTR) {
                DEBUG(LOG_INFO, "error on read, %s, reopening socket", 
                      strerror(errno));
                change_mode(listen_mode); /* just close and reopen */
            }
            if (len < 0) 
                goto out2;
            
            if (packet.xid != xid) {
                DEBUG(LOG_INFO, "Ignoring XID %lx (our xid is %lx)",
                      (unsigned long) packet.xid, xid);
                goto out2;
            }
            
            if ((message = get_option(&packet, DHCP_MESSAGE_TYPE)) == NULL || message[OPT_LEN - 2] != 1) {
                DEBUG(LOG_ERR, "couldnt get option from packet -- ignoring");
                goto out2;
            }
            
            switch (state) {
            case INIT_SELECTING:
                /* Must be a DHCPOFFER to one of our xid's */
                if (*message == DHCPOFFER) {
                    if ((temp = get_option(&packet, DHCP_SERVER_ID)) && temp[OPT_LEN - 2] == 4) {
                        memcpy(&server_addr, temp, 4);
                        xid = packet.xid;
                        requested_ip = packet.yiaddr;
                        
                        /* enter requesting state */
                        state = REQUESTING;
                        timeout = now;
                        packet_num = 0;
                    } else {
                        DEBUG(LOG_ERR, "No server ID in message");
                    }
                }
                break;
            case RENEW_REQUESTED:
            case REQUESTING:
            case RENEWING:
            case REBINDING:
                if (*message == DHCPACK) {
                    if (state == RENEW_REQUESTED || state == RENEWING || state == REBINDING) {
			inc_stats(RENEW_SUCCESS);
                    } else {
			inc_stats(REQUEST_SUCCESS);
                    }

                    if (!(temp = get_option(&packet, DHCP_LEASE_TIME)) || temp[OPT_LEN - 2] != 4) {
                        LOG(LOG_ERR, 
                            "No lease time with ACK, using 1 hour lease");
                        lease = 60 * 60;
                    } else {
                        memcpy(&lease, temp, 4);
                        lease = ntohl(lease);
                    }

                    /* if we receive an infinite or very large lease, cap it to
                     * a reasonable value. Otherwise we run into overflow issues
                     * adding t1 & t2 to epoch time. 
                     */
                    if (lease > DHCP_MAX_LEASE_TIME) {
                        lease = DHCP_MAX_LEASE_TIME;
                    }

                    /* make sure zeroconf isn't running */
                    zc_reset(&zc);
                        
                    /* enter bound state */
                    t1 = lease / 2;
                    
                    /* little fixed point for n * .875 */
                    t2 = (lease * 7) >> 3;
                    temp_addr.s_addr = packet.yiaddr;
                    LOG(LOG_INFO, "Lease of %s obtained, lease time %ld", 
                        inet_ntoa(temp_addr), lease);
                    start = now;
                    timeout = t1 + start;
                    requested_ip = packet.yiaddr;

                    /* figure out if the proper domain name suffix was 
                       specified by the DHCP server */
                    int proper_domain = 0;
                    if (client_config.sdomain) {
                        char *domain = (char*)get_option(&packet, DHCP_DOMAIN_NAME);
                        if (domain) {
                            unsigned char domain_len = domain[OPT_LEN - 2];

                            /* ignore any amount of null-termination */
                            while (domain_len > 0 && 
                                   (domain[domain_len - 1] == '\0'))
                                domain_len--;

                            if (domain_len == strlen(client_config.sdomain))
                                if (0 == strncasecmp(client_config.sdomain,
                                                     domain,
                                                     domain_len))
                                    proper_domain = 1;
                        }
                    }

                    /* notice if we're in Best Buy's domain */
                    if (proper_domain) {
                        /* leave behind a bread crumb indicating that we're
                           in the Best Buy domain */
                        int fd2 = creat("/var/run/specialIP", 0666);
                        if (fd2 >= 0) {
                            close(fd2);
                        }
                    } else {
                        /* leave behind a bread crumb indicating that we got
                           an IP address the usual way */
                        int fd2 = creat("/var/run/normalIP", 0666);
                        if (fd2 >= 0) {
                            close(fd2);
                        }
                    }

                    run_script(&packet,
                               ((state == RENEWING || state == REBINDING)
                                ? "renew" : "bound"));

                    state = BOUND;

#if defined(__SONOS_LINUX__)
                    LOG(LOG_DEBUG, "Sending gratuitous ARP");
                    rtnl_send_gratuitous_arp(client_config.interface, 
                                             client_config.arp);
#endif

#ifdef ADDR_CONFLICT_DETECTION
                    open_server_socket();
#endif

                    change_mode(LISTEN_NONE);
                    if (client_config.quit_after_lease) 
                        exit_client(0);
                    if (!client_config.foreground)
                        background();
                } else if (*message == DHCPNAK) {
                    /* return to init state */
                    LOG(LOG_INFO, "Received DHCP NAK");
                    if (state == RENEW_REQUESTED || state == RENEWING || state == REBINDING) {
			inc_stats(RENEW_NAK);
                    } else {
			inc_stats(REQUEST_NAK);
                    }
                    dump_stats();

                    run_script(&packet, "nak");
                    if (state != REQUESTING)
                        run_script(NULL, "deconfig");
		    inc_stats(REQUEST_ATTEMPT);
                    state = INIT_SELECTING;

#ifdef ADDR_CONFLICT_DETECTION
                    close_server_socket();
#endif

                    timeout = now;
                    requested_ip = 0;
                    packet_num = 0;
                    change_mode(LISTEN_RAW);
                    sleep(3); /* avoid excessive network traffic */
                }
                break;
                /* case BOUND, RELEASED: - ignore all packets */
            }
        out2:
            ;
        }

        /* time-driven processing for zeroconf state machine. */
        if (retval == 0 && zc.zc_timeout <= now)
            zc_handle_timer(&zc, now);

        /* I/O-driven processing for zeroconf state machine. */
        if (retval > 0 && zc.zc_fd != -1 && FD_ISSET(zc.zc_fd, &rfds))
            zc_handle_input(&zc, now);

        /* processing of netlink messages */
        if (retval > 0 && nlh.fd != -1 && FD_ISSET(nlh.fd, &rfds)) {
            if (rtnl_msg_triggers_dhcp_renew(&nlh, 
                                             client_config.wifi,
                                             client_config.interface,
                                             client_config.arp)) {
                /* nlhelp.c determined that this message should trigger a
                   DHCP renew; do it. */
                time_aggressive_end = time(0) + AGGR_DURATION;
                /* delay zeroconf fallback when triggered by netlink */
                time_zeroconf_fallback = time(0) + ZC_FALLBACK_DELAY;
                perform_renew();
            }
        }

#ifdef ADDR_CONFLICT_DETECTION
        /* Processing of DHCPREQUEST messages while in the BOUND state */
        if (retval > 0 && s_fd != -1 && FD_ISSET(s_fd, &rfds)) {
            if ((len = get_packet(&packet, s_fd)) < 0) { 
                if (len == -1 && errno != EINTR) {
                    close_server_socket();
                }
            } else if ((message = get_option(&packet, DHCP_MESSAGE_TYPE)) && message[OPT_LEN - 2] == 1 &&
                       message[0] == DHCPREQUEST) {
                unsigned char* opt;
                u_int32_t monitored_requested_ip = 0;
                //u_int32_t server_id;

                // Suppressed by default. Dont be too chatty
                LOG(LOG_DEBUG, "Received DHCP_REQUEST message");

                opt = get_option(&packet, DHCP_REQUESTED_IP);
                if (opt && opt[OPT_LEN - 2] == 4) {
                    memcpy(&monitored_requested_ip, opt, 4);
                }
                //opt = get_option(&packet, DHCP_SERVER_ID); 
                //if (opt) { 
                //    memcpy(&server_id, opt, 4);
                //}

                temp_addr.s_addr = monitored_requested_ip;
                LOG(LOG_DEBUG, "req ip = %s", inet_ntoa(temp_addr));
                temp_addr.s_addr = requested_ip;
                LOG(LOG_DEBUG, " my ip = %s", inet_ntoa(temp_addr));

                if (monitored_requested_ip && requested_ip == monitored_requested_ip) {
                    LOG(LOG_INFO, "Address duplication detected, INITIATING NEW DHCP");
		    inc_stats(DUPLICATION);
                    // Force a renewal to confirm our address.
                    perform_renew();
                    sleep(3); /* avoid excessive network traffic */
                }
            }
        }
#endif

        /* signal-driven processing */
        if (retval > 0 && FD_ISSET(signal_pipe[0], &rfds)) {
            if (read(signal_pipe[0], &sig, sizeof(sig)) < 0) {
                DEBUG(LOG_ERR, "Could not read signal: %s", 
                      strerror(errno));
                goto out3; /* probably just EINTR */
            }
            switch (sig) {
            case SIGUSR1: 
                perform_renew();
                break;
            case SIGTERM:
                LOG(LOG_INFO, "Received SIGTERM");
                exit_client(0);
            }
        out3:
            ;
        }
    }
    return 0;
}
