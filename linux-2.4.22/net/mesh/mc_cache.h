#ifndef MC_CACHE_H
#define MC_CACHE_H

#include <linux/kernel.h>
#include <linux/module.h>

void init_mc_cache(void);
int check_mc_cache(u_int32_t ip, u_int16_t id);

#endif
