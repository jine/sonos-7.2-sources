/*
 * Copyright (c) 2006, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Led Driver
 */

#ifndef __SONOS_LEDS_H__
#define __SONOS_LEDS_H__

#define VERSION         "0.1"
#define LED_DRIVER_ID_STR   "Sonos LED Driver"

#define LED_BASENAME    "driver/led"
#define BLINK_BASENAME  "driver/blink"
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

//Fillmore PROTO-1
#define GPIO_LED_WHITE_PIN   20
#define GPIO_LED_GREEN_PIN   21
#define GPIO_LED_RED_PIN     22
#define GPIO_LED_AMBER_PIN   23

//Fillmore BB2
#define BB2_GPIO_LED_WHITE_PIN   GPIO_LED_WHITE_PIN /* 20 */
#define BB2_GPIO_LED_GREEN_PIN   GPIO_LED_GREEN_PIN /* 21 */
#define BB2_GPIO_LED_RED_PIN     GPIO_LED_RED_PIN   /* 22 */
#define BB2_GPIO_LED_AMBER_PIN   GPIO_LED_AMBER_PIN /* 23 */

#define GPIO_LED_WHITE       (1 << GPIO_LED_WHITE_PIN)
#define GPIO_LED_GREEN       (1 << GPIO_LED_GREEN_PIN)
#define GPIO_LED_RED         (1 << GPIO_LED_RED_PIN)
#define GPIO_LED_AMBER       (1 << GPIO_LED_AMBER_PIN)
#define GPIO_LED_MASK (GPIO_LED_WHITE | GPIO_LED_RED | GPIO_LED_GREEN | GPIO_LED_AMBER)

#define LED_WHITE       0x01
#define LED_RED         0x02
#define LED_GREEN       0x04
#define LED_AMBER       0x08
#define LED_OVERRIDE    0x10
#define LED_MASK        0x0F

#define SIOCSETLED   _IOR('i', 137, int *)
#define SIOCBLINKLED _IOR('i', 139, int *)
#define SIOCGETTIMESTAMP				_IOR('i', 140, struct time_formats)

#endif
