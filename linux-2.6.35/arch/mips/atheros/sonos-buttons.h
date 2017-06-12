/*
 * Copyright (c) 2013, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos ATHEROS Button Driver
 */

#ifndef __SONOS_BUTTON_DRIVER_H__
#define __SONOS_BUTTON_DRIVER_H__

#include "sonos-ctl.h"

#define VERSION_BUTTONS         "0.2"

#define AUDIOCTL_GET_BUTTON_STATE _IOR('s', 62, int[EVTSOURCE_NUM_SOURCES])
#define AUDIOCTL_GET_REPEAT_TIME  _IOR('s', 63, int)
#define AUDIOCTL_SET_REPEAT_TIME  _IOW('s', 64, int)

/* GPIO pins define by HW */
#define GPIO_BUTTON_HOUSEHOLD    16

/* Button masks */
#define BUTTON_HOUSEHOLD_MASK    (1 << GPIO_BUTTON_HOUSEHOLD)

#define BUTTON_DRIVER_ID_STR   "Sonos Button Driver"

#endif	// __SONOS_BUTTON_DRIVER_H__
