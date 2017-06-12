#ifndef __SONOS_LEDS_H__
#define __SONOS_LEDS_H__

/*
 *      Sonos Led Driver -
 *      Copyright (C) Sonos Inc. 2006
 */

#define VERSION         "0.1"
#define LED_DRIVER_ID_STR   "Sonos LED Driver"

#define LED_BASENAME    "led"
#define LED_FULLNAME    "/dev/leds"
#define LED_DEVICE      LED_FULLNAME

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

#define LED_WHITE       0x01
#define LED_RED         0x02
#define LED_GREEN       0x04
#define LED_AMBER       0x08
#define LED_OVERRIDE    0x10
#define LED_MASK        0x0F

#define SIOCSETLED      137
#define SIOCGETLED      138
#define SIOCBLINKLED    139
#define SIOCBLINKRATE   140
#define SIOCGETTIMESTAMP				141

#endif
