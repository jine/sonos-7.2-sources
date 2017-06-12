/*
 * Copyright (c) 2011, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

// Comment out to remove all syslog logging
#define DBG_SYSLOG

#ifdef DBG_SYSLOG

#include <syslog.h>

// Non standard extension.
void setmaxlogsize(int size);

#else

#define openlog(...)
#define closelog(...)
#define setlogmask(...)
#define setmaxlogsize(...)
#define syslog(...)
#define vsyslog(...)


#endif // DBG_SYSLOG


