/*
 * Copyright (c) 2006, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos AR531X Button Driver
 */

#ifndef __AR531XBUTTON_DRIVER_H__
#define __AR531XBUTTON_DRIVER_H__

#include "sonos-ctl.h"

#define VERSION_BUTTON         "0.2"
#define BUTTON_DRIVER_ID_STR   "Sonos Button Driver"

#define AUDIOCTL_GET_BUTTON_STATE _IOR('s', 62, int[EVTSOURCE_NUM_SOURCES])
#define AUDIOCTL_GET_REPEAT_TIME  _IOR('s', 63, int)
#define AUDIOCTL_SET_REPEAT_TIME  _IOW('s', 64, int)

#endif	// __AR531XBUTTON_DRIVER_H__
