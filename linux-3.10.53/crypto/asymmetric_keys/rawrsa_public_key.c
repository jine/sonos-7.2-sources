/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Instantiate a public key crypto key from PKCS#1 DER RSA public key
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/mpi.h>
#include <linux/asn1_decoder.h>
#include <keys/asymmetric-subtype.h>
#include <keys/asymmetric-parser.h>
#include <crypto/hash.h>
#include "asymmetric_keys.h"
#include "public_key.h"
#include "rawrsa_parser.h"

/*
 * Attempt to parse a data blob for a key as raw RSA.
 */
static int rawrsa_key_preparse(struct key_preparsed_payload *prep)
{
	struct rawrsa *pubkey;
	char *desc = NULL;
	int ret = -ENOMEM;

	pubkey = rawrsa_parse(prep->data, prep->datalen);
	if (IS_ERR(pubkey)) {
		return PTR_ERR(pubkey);
	}

	pubkey->pub->algo = &RSA_public_key_algorithm;
	pubkey->pub->id_type = PKEY_ID_RAWRSA;

	/* Propose a description */
	desc = kmalloc(strlen(pubkey->fingerprint) + 1, GFP_KERNEL);
	if (!desc) {
		goto end;
	}
	strcpy(desc, pubkey->fingerprint);

	/* We're pinning the module by being linked against it */
	__module_get(public_key_subtype.owner);
	prep->type_data[0] = &public_key_subtype;
	prep->type_data[1] = pubkey->fingerprint;
	prep->payload = pubkey->pub;
	prep->description = desc;
	prep->quotalen = 100;

	/* We've finished with the rawrsa structure */
	pubkey->pub = NULL;
	pubkey->fingerprint = NULL;
	ret = 0;

end:
	rawrsa_free(pubkey);
	return ret;
}

static struct asymmetric_key_parser rawrsa_key_parser = {
	.owner	= THIS_MODULE,
	.name	= "rawrsa",
	.parse	= rawrsa_key_preparse,
};

/*
 * Module stuff
 */
static int __init rawrsa_key_init(void)
{
	return register_asymmetric_key_parser(&rawrsa_key_parser);
}

static void __exit rawrsa_key_exit(void)
{
	unregister_asymmetric_key_parser(&rawrsa_key_parser);
}

module_init(rawrsa_key_init);
module_exit(rawrsa_key_exit);
