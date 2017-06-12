#ifndef _LINUX_CONFIG_H
#define _LINUX_CONFIG_H

#if defined(__SONOS_LINUX_SH4__)
#include <linux/autoconf_sh.h>
#elif defined(__SONOS_LINUX_PPC__)
#include <linux/autoconf_ppc.h>
#else
#error must define either __SONOS_LINUX_SH4__ or __SONOS_LINUX_PPC__
#endif

#endif
