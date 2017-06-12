/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Signature format support
 */

/* various macros needed by the included code */
#include <crypto/sonos_signature_macros_linux.h>

#include <crypto/public_key.h>
#include <keys/asymmetric-subtype.h>

#include <crypto/sonos_signature_common_linux.h>
#include <crypto/sonos_signature_verify_linux.h>

/* most of the verify implementation */
#include "sonos_signature_verify.c.inc"
#include "sonos_attr.c.inc"
#include "sonos_attr_parse.c.inc"
#include "sonos_attr_serialize.c.inc"

#include <crypto/sonos_signature_keys.h>

int
sonosRawVerify(SonosSigningKey_t vkey,
	       SonosSignatureAlg_t signAlg,
	       SonosDigestAlg_t digestAlg,
	       const uint8_t *digest, size_t digestLen,
	       const uint8_t *signature, uint32_t sigLen)
{
	const struct key *key = (const struct key *)vkey;
	struct public_key_signature sig;
	struct asymmetric_key_subtype *method;
	int result = 0;
	uint8_t digestCopy[SHA256_DIGEST_SIZE];

	if (signAlg != SONOS_SIGNATURE_ALG_RSAPKCS1 ||
	    digestAlg != SONOS_DIGEST_ALG_SHA256 ||
	    digestLen != SHA256_DIGEST_SIZE) {
		printk(KERN_ERR "sonosRawVerify: bad signAlg (%d) digestAlg (%d) or digestLen (%d)\n",
		       (int)signAlg, (int)digestAlg, (int)digestLen);
		return 0;
	}

	/* put the signature into the expected format */
	memset(&sig, 0, sizeof sig);
	/* avoid casting off cost since sig.digest isn't const */
	memcpy(digestCopy, digest, digestLen);
	sig.digest = digestCopy;
	sig.digest_size = digestLen;
	sig.nr_mpi = 1;
	sig.pkey_hash_algo = PKEY_HASH_SHA256;
	sig.rsa.s = mpi_read_raw_data(signature, sigLen);
	if (!sig.rsa.s) {
		printk(KERN_ERR "sonosRawVerify: mpi_read_raw_data failed\n");
		return 0;
	}

	method = asymmetric_key_subtype(key);
	if (method->verify_signature(key, &sig) == 0) {
		result = 1;
	}
	else {
		printk(KERN_ERR "sonosRawVerify: verify_signature failed\n");
	}

	if (sig.rsa.s) {
		mpi_free(sig.rsa.s);
	}

	return result;
}

SonosSigningKey_t
sonosKeyLookup(const void *cbArg,
	       SonosKeyIdentifierScheme_t keyIdScheme,
	       const uint8_t *keyId,
	       uint32_t keyIdLen)
{
	int i;
	const SonosRsaKeyEntry *keyEntry;
	const char *name = (const char *)cbArg;

	if (keyIdScheme != SS_KI_SCHEME_X509_SKI &&
	    keyIdScheme != SS_KI_SCHEME_SHA1 &&
	    keyIdScheme != SS_KI_SCHEME_NAME) {
		goto err;
	}

	if (keyIdScheme == SS_KI_SCHEME_X509_SKI ||
	    keyIdScheme == SS_KI_SCHEME_SHA1) {
		if (keyIdLen != SHA1_DIGEST_SIZE) {
			goto err;
		}
	}

	/* for now always override the lookup using cbArg */
	keyIdScheme = SS_KI_SCHEME_NAME;
	keyId = name;
	keyIdLen = strlen(name);

	for (i = 0; g_SonosSigningKeys[i] != NULL; i++) {
		keyEntry = g_SonosSigningKeys[i];
		if (keyIdScheme == SS_KI_SCHEME_NAME) {
			if (strlen(keyEntry->name) == keyIdLen &&
			    keyEntry->key &&
			    memcmp(keyEntry->name, keyId, keyIdLen) == 0) {
				printk(KERN_DEBUG "sonosKeyLookup chose key %d\n", i+1);
				return (SonosSigningKey_t)(keyEntry->key);
			}
		}
		else {
			if (keyEntry->keyIdScheme == keyIdScheme &&
			    keyEntry->keyIdLen == keyIdLen &&
			    keyEntry->key &&
			    memcmp(keyEntry->keyId, keyId, keyIdLen) == 0) {
				printk(KERN_DEBUG "sonosKeyLookup chose key %d\n", i+1);
				return (SonosSigningKey_t)(keyEntry->key);
			}
		}
	}

err:
	printk(KERN_CRIT "sonosKeyLookup failed (%d, %d)\n",
	       (int)keyIdScheme, (int)keyIdLen);
	return NULL;
}
