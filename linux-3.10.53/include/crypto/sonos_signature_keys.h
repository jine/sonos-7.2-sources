/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Signature format support
 */

#ifndef SONOS_SIGNATURE_KEYS_H
#define SONOS_SIGNATURE_KEYS_H

#include <crypto/sha.h>
#include "sonos_signature.h"

// SS_KI_SCHEME_X509_SKI
// SS_KI_SCHEME_SHA1
// SS_KI_SCHEME_NAME
typedef struct {
	struct key *key;
	const uint8_t *der;
	size_t derLen;
        const char *name;
        SonosKeyIdentifierScheme_t keyIdScheme;
        uint8_t keyId[SHA1_DIGEST_SIZE];
        size_t keyIdLen;
} SonosRsaKeyEntry;

/* a null-terminated list of signing keys known to u-boot */
extern const SonosRsaKeyEntry * const g_SonosSigningKeys[];

/* point the keys at array fields */
extern void sonosInitKeyTable(void);

#endif
