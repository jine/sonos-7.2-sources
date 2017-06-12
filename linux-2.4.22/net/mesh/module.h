#ifndef MODULE_H
#define MODULE_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/types.h>
#include <linux/netfilter_ipv4.h>
#include <linux/ctype.h>
#include <linux/ip.h>
#include <linux/timer.h>

#include "mesh.h"
#include "pkthandlers.h"
#include "pktqueue.h"
#include "rtable.h"
#include "timers.h"
#include "message_queue.h"
#include "rrequest_cache.h"
#include "iflist.h"
#include "mesh_neighbour.h"
#include "multicast_queue.h"
#include "mc_cache.h"

int is_passive(void);

#endif
