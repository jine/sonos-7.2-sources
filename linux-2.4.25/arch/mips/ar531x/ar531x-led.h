/*
 * Copyright (c) 2006, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos AR531X Led Driver
 */

#ifndef __AR531XLED_DRIVER_H__
#define __AR531XLED_DRIVER_H__


#define VERSION         "0.1"
#define LED_DRIVER_ID_STR   "Sonos LED Driver"

#define LED_BASENAME    "led"
#define LED_FULLNAME    "/dev/leds"
#define LED_DEVICE      LED_FULLNAME

#define LED_WHITE       0x01
#define LED_RED         0x02
#define LED_GREEN       0x04
#define LED_AMBER       0x08
#define LED_OVERRIDE    0x10
#define LED_MASK        0x0F

/* ioctl SIOCGETTIMESTAMP parameter. "Rosetta stone" of system timestamps.
 * jiffies - jiffie64 count from the kernel.
 * timeofday_sec - seconds output from gettimeofday().
 * timeofday_usec - microseconds output from gettimeofday().
 * kernel_stamp - printk() timestamp.
 */
struct time_formats {
    unsigned long long  jiffies;
    unsigned long       timeofday_sec;
    unsigned long       timeofday_usec;
    unsigned long       kernel_stamp;
};

#ifdef __linux__
#define SIOCSETLED   (SIOCDEVPRIVATE+0)
#define SIOCGETLED   (SIOCDEVPRIVATE+1)
#define SIOCBLINKLED (SIOCDEVPRIVATE+2)
#define SIOCBLINKRATE (SIOCDEVPRIVATE+3)
#else
#define SIOCSETLED   _IOR('i', 137, int *)
#define SIOCGETLED   _IOW('i', 138, int *)
#define SIOCBLINKLED _IOR('i', 139, int *)
#define SIOCBLINKRATE _IOR('i', 140, int *)
#endif

#define SIOCGETTIMESTAMP				_IOR('i', 141, struct time_formats)


#endif
