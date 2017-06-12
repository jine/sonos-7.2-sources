/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Sonos Signature format support
 */

/* various macros needed by the included code */
#include <crypto/sonos_signature_macros_linux.h>

#include <crypto/sha.h>
#include <linux/crypto.h>
#include <linux/err.h>
#include <linux/scatterlist.h>

#include <crypto/sonos_signature_common_linux.h>

/* most of the common implementation */
#include "sonos_signature_common.c.inc"

int
sonosHash(SonosDigestAlg_t alg, const void *buf, size_t bufLen,
	  uint8_t *digest, size_t *pDigestLen)
{
	struct scatterlist sg;
	struct crypto_hash *tfm;
	struct hash_desc desc;
	int result = 0;

	if (alg != SONOS_DIGEST_ALG_SHA256 ||
	    *pDigestLen < SHA256_DIGEST_SIZE) {
		printk(KERN_ERR "sonosHash: bad alg (%d) or digestLen (%d)\n",
		       (int)alg, (int)*pDigestLen);
		return 0;
	}

	tfm = crypto_alloc_hash("sha256", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		printk(KERN_ERR "sonosHash: could not allocate crypto hash\n");
		return 0;
	}

	sg_init_table(&sg, 1);
	sg_set_buf(&sg, buf, bufLen);
	desc.tfm = tfm;
	desc.flags = 0;

	if (crypto_hash_digest(&desc, &sg, bufLen, digest)) {
		printk(KERN_ERR "sonosHash: crypto_hash_digest failed\n");
	}
	else {
		*pDigestLen = SHA256_DIGEST_SIZE;
		result = 1;
	}

	crypto_free_hash(tfm);

	return result;
}
