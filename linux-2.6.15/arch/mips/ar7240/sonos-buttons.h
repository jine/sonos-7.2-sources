/*
 * Copyright (c) 2006, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos AR7240 Button Driver
 */

#ifndef __SONOS_BUTTON_DRIVER_H__
#define __SONOS_BUTTON_DRIVER_H__

#define VERSION_BUTTONS         "0.2"
#define BUTTON_DRIVER_ID_STR   "Sonos Button Driver"

#define AUDIOCTL_GET_BUTTON_STATE _IOR('s', 62, int[EVTSOURCE_NUM_SOURCES])
#define AUDIOCTL_GET_REPEAT_TIME  _IOR('s', 63, int)
#define AUDIOCTL_SET_REPEAT_TIME  _IOW('s', 64, int)

#endif	// __SONOS_BUTTON_DRIVER_H__
