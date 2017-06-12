/* Copyright (c) 2003, Rincon Networks, Inc.  All rights reserved. */

#ifndef NLHELP_H
#define NLHELP_H

#ifdef MACOSX
#include <sys/types.h>
#else
#include <asm/types.h>
#endif

#include <sys/socket.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#if defined(__cplusplus)
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////
//
// helper structures and functions for using RTNetLink
//

struct rtnl_handle
{
    int fd;
    struct sockaddr_nl local;
    struct sockaddr_nl peer;
    __u32 seq;
    __u32 dump;
};

int rtnl_send_gratuitous_arp(const char* if_name,
                             const unsigned char* if_hw_addr);

int rtnl_open(struct rtnl_handle *rth, unsigned subscriptions);
int rtnl_msg_triggers_dhcp_renew(struct rtnl_handle *rth, 
                                 const char* wifi_if_name,
                                 const char* if_name,
                                 const unsigned char* if_hw_addr);
void rtnl_close(struct rtnl_handle *rth);

#if defined(__cplusplus)
};
#endif

#endif
