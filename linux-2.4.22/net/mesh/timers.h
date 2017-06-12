#ifndef TIMERS_H
#define TIMERS_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/in.h>
#include <linux/signal.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/div64.h>
#include <linux/random.h>

#include "rtable.h"
#include "rrequest_cache.h"
#include "utils.h"
#include "mesh.h"
#include "mesh_neighbour.h"
#include "iflist.h"

struct timer_queue_entry
{
    /* Pointer to the next entry in the list */
    struct timer_queue_entry *next;

    u_int64_t tv;
    int size;
    u_int32_t id;
    u_int8_t ttl;
    u_int16_t retries;

    /* Data stored in the entry */
    void *data;

    /* The function to execute when the entry gets triggered */
    int (*func)(struct timer_queue_entry *);
};

typedef int (*timer_func_ptr)(struct timer_queue_entry *);

int init_timer_queue(void);
void cleanup_timer_queue(void);

void update_timer_queue(void);
int insert_timer(u_int64_t msec,void *data,int size, u_int32_t id,u_int16_t
        retries,u_int8_t ttl, timer_func_ptr func);
struct timer_queue_entry * find_first_timer_queue_entry_of_id(u_int32_t id);
struct timer_queue_entry *
find_first_timer_queue_entry_of_id_and_func(u_int32_t id, timer_func_ptr func);
void delete_timer(u_int32_t id, timer_func_ptr func);

int timer_rreq(struct timer_queue_entry* entry);
int hello_resend(struct timer_queue_entry* entry);
int timer_cleanup(struct timer_queue_entry* entry);
int timer_neighbour(struct timer_queue_entry* entry);


#endif
