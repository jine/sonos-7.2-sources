/* Copyright (c) 2004, Rincon Networks, Inc.  All rights reserved. */

/*
 * Additions to uDHCPc made by Rincon for ZeroConf address assignment.
 */

#define ZEROCONF_DISABLED   0
#define ZEROCONF_PROBING    1
#define ZEROCONF_ANNOUNCING 2
#define ZEROCONF_RUNNING    3

struct zc_config {
    /* file descriptor for socket used to send/receive ARP messages */
    int zc_fd;

    /* zeroconf state variable */
    int zc_state;
    
    /* timeout for operations that require waiting */
    time_t zc_timeout;

    /* counter for operations that need to be repeated */
    int zc_count;
    
    /* 
     *  the IP address we're currently using 
     *  (valid in PROBING, ANNOUNCING, and RUNNING states)
     */
    unsigned char zc_addr[4];

    /* local MAC address */
    unsigned char zc_hw_addr[6];
};

/* create an ARP socket to receive ARPs related to a specified interface */
int zc_arp_socket(const char* inf);

/* process an incoming ARP datagram */
void zc_handle_input(struct zc_config* zc, time_t now);

/* handle time-driven processing */
void zc_handle_timer(struct zc_config* zc, time_t now);

/* initial state, to be invoked once during boot */
void zc_init(struct zc_config* zc, const unsigned char* hw_addr);

/* reset state; to be called when DHCP takes over, etc */
void zc_reset(struct zc_config* zc);

/* generate random IP address and enter probing state */
void zc_start_probing(struct zc_config* zc, time_t now);

/* send an ARP probe message */
void zc_send_arp_probe(struct zc_config* zc);

/* send an ARP announcement message */
void zc_send_arp_announce(struct zc_config* zc);
