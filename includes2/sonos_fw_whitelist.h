/*
 * Copyright (c) 2014, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * sonos_fw_whitelist.h: structures representing a firmware whitelist
 *                       that binds a u-boot or kernel image to a specific
 *                       list of devices
 */

#ifndef SONOS_FW_WHITELIST_H
#define SONOS_FW_WHITELIST_H

#include "sonos_stdint.h"

#define SONOS_FWW_MAGIC_INIT \
    0x56, 0xea, 0x9f, 0x16, 0x21, 0x54, 0x31, 0xa5, \
    0xba, 0x0f, 0x50, 0xab, 0xf1, 0x64, 0xfa, 0x17, \
    0x6d, 0xab, 0xb0, 0x10, 0xc4, 0xb5, 0x71, 0x9e, \
    0x33, 0x31, 0x59, 0xc6, 0x82, 0xba, 0x4c,

#define SONOS_FWW_MAGIC_INIT_UNCOMPRESSED_COPY \
    0x47, 0x49, 0x46, 0x3B, 0x70, 0xB4, 0x33, 0xA6, \
    0x67, 0xF1, 0x7B, 0xD7, 0x4B, 0xA3, 0x1A, 0x4D, \
    0x5B, 0xA1, 0xE8, 0xAA, 0x99, 0x83, 0xA3, 0x8D, \
    0x58, 0xEE, 0xA1, 0xB6, 0xE6, 0x00, 0xCF,

#ifdef SONOS_FWW_IN_UNCOMPRESSED_KERNEL
#undef SONOS_FWW_MAGIC_INIT
#define SONOS_FWW_MAGIC_INIT SONOS_FWW_MAGIC_INIT_UNCOMPRESSED_COPY
#endif

#define SONOS_FWW_MAGIC_LEN    31

#define SONOS_FWW_MAX_ENTRIES  10000

#define SONOS_FWW_TYPE_CPUID   1
#define SONOS_FWW_TYPE_SERIAL  2

#define SONOS_FWW_LEN_CPUID    8
#define SONOS_FWW_LEN_SERIAL   6

#define SONOS_FWW_FMT_CPUID     "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
#define SONOS_FWW_FMT_SERIAL    "%02x:%02x:%02x:%02x:%02x:%02x"

#define SONOS_FWW_FMTARG_CPUID(p)   p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]
#define SONOS_FWW_FMTARG_SERIAL(p)  p[0], p[1], p[2], p[3], p[4], p[5]

typedef struct __attribute__((packed)) {
    uint8_t magic[SONOS_FWW_MAGIC_LEN];
    uint8_t whitelistType;
    uint32_t numEntries;
} SonosFirmwareWhitelistHeader;

#ifndef NDEBUG
typedef struct {
    uint8_t magic[SONOS_FWW_MAGIC_LEN];
    uint8_t whitelistType;
    uint32_t numEntries;
} SonosFirmwareWhitelistHeaderUnpacked;
#endif

typedef struct { uint8_t value[SONOS_FWW_LEN_CPUID]; } SonosCpuid_t;
typedef struct { uint8_t value[SONOS_FWW_LEN_SERIAL]; } SonosSerial_t;

#endif
