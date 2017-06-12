/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Signature format support
 */

#ifndef SONOS_SIGNATURE_COMMON_LINUX_H
#define SONOS_SIGNATURE_COMMON_LINUX_H

#include "sonos_signature.h"

int sonosHash(SonosDigestAlg_t alg, const void *buf, size_t bufLen,
              uint8_t *digest, size_t *pDigestLen);

#endif
