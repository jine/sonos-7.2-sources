/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Raw RSA parser internal definitions
 */

#include <crypto/public_key.h>

struct rawrsa {
	struct public_key *pub;			/* Public key details */
	char		*fingerprint;		/* Key fingerprint as hex */
};

/*
 * rawrsa_parser.c
 */
extern void rawrsa_free(struct rawrsa *rsa);
extern struct rawrsa *rawrsa_parse(const void *data, size_t datalen);
