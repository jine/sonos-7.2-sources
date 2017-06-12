/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Raw RSA parser
 */

#include <crypto/sha.h>
#include <linux/crypto.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/scatterlist.h>
#include "public_key.h"
#include "rawrsa_parser.h"
#include "rawrsa-asn1.h"

struct rawrsa_parse_context {
	struct rawrsa	*rsa;		/* Key being constructed */
	unsigned char	nr_mpi;			/* Number of MPIs stored */
	unsigned char ski[SHA1_DIGEST_SIZE];
};

void rawrsa_free(struct rawrsa *rsa)
{
	if (rsa) {
		public_key_destroy(rsa->pub);
		kfree(rsa->fingerprint);
		kfree(rsa);
	}
}

/*
 * Helper to compute the SHA1 hash of the PKCS#8 encoding of the public key
 * even though the input buffer here is in PKCS#1 encoding.
 */
static int computeKeyID(const void *in, size_t inLen, unsigned char *out)
{
	struct scatterlist sg[2];
	struct crypto_hash *tfm;
	struct hash_desc desc;
	uint16_t asn1Len;
	int ret;
/*
 * Prepend the PKCS#8 markup during the hash computation (it is a mostly
 * canned ASN.1 byte sequence that precedes the embedded PKCS#1 key).
 *
 * This does *not* deal with arbitrary key lengths (for example this will
 * misencode things for 1024-bit keys since the ASN.1 length field is itself
 * variable length and will shrink in that case).
 */
#define LENGTH_BYTE		0x00
#define LENGTH_DELTA	19
#define LENGTH_OFFSET_1 2
#define LENGTH_OFFSET_2	21
	static const unsigned char p8PrefixTemplate[] =
	{
		0x30, 0x82, LENGTH_BYTE, LENGTH_BYTE,
		0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00,
		0x03, 0x82, LENGTH_BYTE, LENGTH_BYTE, 0x00

	};
	unsigned char p8Prefix[sizeof p8PrefixTemplate];

	tfm = crypto_alloc_hash("sha1", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		return PTR_ERR(tfm);
	}

	/* populate the two length fields in the otherwise fixed template */
	memcpy(p8Prefix, p8PrefixTemplate, sizeof p8Prefix);
	asn1Len = inLen + sizeof p8Prefix - 4;
	p8Prefix[LENGTH_OFFSET_1]   = (asn1Len & 0xff00) >> 8;
	p8Prefix[LENGTH_OFFSET_1+1] = (asn1Len & 0x00ff) >> 0;
	asn1Len -= LENGTH_DELTA;
	p8Prefix[LENGTH_OFFSET_2]   = (asn1Len & 0xff00) >> 8;
	p8Prefix[LENGTH_OFFSET_2+1] = (asn1Len & 0x00ff) >> 0;

	sg_init_table(sg, 2);
	sg_set_buf(&sg[0], p8Prefix, sizeof p8Prefix);
	sg_set_buf(&sg[1], in, inLen);
	desc.tfm = tfm;
	desc.flags = 0;

	memset(out, 0, SHA1_DIGEST_SIZE);
	ret = crypto_hash_digest(&desc, sg, (sizeof p8Prefix) + inLen, out);
	crypto_free_hash(tfm);

	return ret;
}

struct rawrsa *rawrsa_parse(const void *data, size_t datalen)
{
	struct rawrsa *rsa = NULL;
	struct rawrsa_parse_context ctx;
	long ret = -ENOMEM;
	size_t i;

	memset(&ctx, 0, sizeof ctx);
	rsa = kzalloc(sizeof(struct rawrsa), GFP_KERNEL);
	if (!rsa)
		goto err;
	rsa->pub = kzalloc(sizeof(struct public_key), GFP_KERNEL);
	if (!rsa->pub)
		goto err;
	rsa->fingerprint = kzalloc(2*(sizeof ctx.ski)+1, GFP_KERNEL);
	if (!rsa->fingerprint)
		goto err;

	ctx.rsa = rsa;

	/* Decode the public key */
	ret = asn1_ber_decoder(&rawrsa_decoder, &ctx, data, datalen);
	if (ret < 0) {
		goto err;
	}

	/* encode its fingerprint which is also used for Sonos Signature keyid */
	ret = computeKeyID(data, datalen, ctx.ski);
	if (ret != 0) {
		goto err;
	}
	for (i = 0; i < sizeof ctx.ski; i++) {
		sprintf(rsa->fingerprint + (i*2), "%02x", ctx.ski[i]);
	}

	return rsa;

err:
	rawrsa_free(rsa);
	return ERR_PTR(ret);
}

int rawrsa_extract_mpi(void *context, size_t hdrlen,
		    unsigned char tag,
		    const void *value, size_t vlen)
{
	struct rawrsa_parse_context *ctx = context;
	MPI mpi;

	if (ctx->nr_mpi >= ARRAY_SIZE(ctx->rsa->pub->mpi)) {
		pr_err("Too many public key MPIs in certificate\n");
		return -EBADMSG;
	}

	mpi = mpi_read_raw_data(value, vlen);
	if (!mpi)
		return -ENOMEM;

	ctx->rsa->pub->mpi[ctx->nr_mpi++] = mpi;
	return 0;
}

