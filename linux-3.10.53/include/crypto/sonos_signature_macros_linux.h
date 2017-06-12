/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Signature format support
 */

#ifndef SONOS_SIGNATURE_MACROS_LINUX_H
#define SONOS_SIGNATURE_MACROS_LINUX_H

#include <linux/kernel.h>
#include <linux/string.h>
#include <crypto/sha.h>

/* various macros needed by the code in the next include */
#define SONOS_NASN_BE16_TO_CPU  be16_to_cpu
#define SONOS_NASN_BE32_TO_CPU  be32_to_cpu
#define SONOS_NASN_CPU_TO_BE16  cpu_to_be16
#define SONOS_NASN_CPU_TO_BE32  cpu_to_be32
#define SONOS_NASN_MEMCPY       memcpy
#define SS_MAX_DIGEST_LENGTH    SHA256_DIGEST_SIZE
#define SS_MEMSET               memset
#define SS_SIZET_MAX            ((size_t)-1)

#endif
