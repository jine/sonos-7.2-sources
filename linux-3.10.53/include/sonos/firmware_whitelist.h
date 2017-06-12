/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#ifndef FIRMWARE_WHITELIST_H
#define FIRMWARE_WHITELIST_H

#ifdef CONFIG_SONOS_SECBOOT

#include "sonos_fw_whitelist.h"
typedef struct __attribute__((packed))
{
	SonosFirmwareWhitelistHeader header;
	union {
		SonosCpuid_t cpuidEntries[2];
		SonosSerial_t serialEntries[2];
	} x;
} SonosFirmwareWhitelist;

extern const SonosFirmwareWhitelist SONOS_FIRMWARE_WHITELIST;

#else
#error you should not be including firmware_whitelist.h
#endif

#endif
