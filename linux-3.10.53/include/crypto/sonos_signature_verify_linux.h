/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Signature format support
 */

#ifndef SONOS_SIGNATURE_VERIFY_LINUX_H
#define SONOS_SIGNATURE_VERIFY_LINUX_H

#include "sonos_signature.h"

int sonosRawVerify(SonosSigningKey_t key,
		   SonosSignatureAlg_t signAlg,
		   SonosDigestAlg_t digestAlg,
		   const uint8_t *digest, size_t digestLen,
		   const uint8_t *signature, uint32_t sigLen);

/* cbArg is expected to be a const char * with a name */
SonosSigningKey_t sonosKeyLookup(const void *cbArg,
				 SonosKeyIdentifierScheme_t keyIdScheme,
				 const uint8_t *keyId,
				 uint32_t keyIdLen);

#endif
