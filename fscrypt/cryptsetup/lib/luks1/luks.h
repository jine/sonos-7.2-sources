/*
 * LUKS - Linux Unified Key Setup
 *
 * Copyright (C) 2004-2006, Clemens Fruhwirth <clemens@endorphin.org>
 * Copyright (C) 2009-2012, Red Hat, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef INCLUDED_CRYPTSETUP_LUKS_LUKS_H
#define INCLUDED_CRYPTSETUP_LUKS_LUKS_H

/*
 * LUKS partition header
 */

#include "libcryptsetup.h"

#define LUKS_CIPHERNAME_L 32
#define LUKS_CIPHERMODE_L 32
#define LUKS_HASHSPEC_L 32
#define LUKS_DIGESTSIZE 20 // since SHA1
#define LUKS_HMACSIZE 32
#define LUKS_SALTSIZE 32
#define LUKS_NUMKEYS 8

// Minimal number of iterations
#define LUKS_MKD_ITERATIONS_MIN  1000
#define LUKS_SLOT_ITERATIONS_MIN 1000

#define LUKS_KEY_DISABLED_OLD 0
#define LUKS_KEY_ENABLED_OLD 0xCAFE

#define LUKS_KEY_DISABLED 0x0000DEAD
#define LUKS_KEY_ENABLED  0x00AC71F3

#define LUKS_STRIPES 4000

// partition header starts with magic
#define LUKS_MAGIC {'L','U','K','S', 0xba, 0xbe};
#define LUKS_MAGIC_L 6

/* Actually we need only 37, but we don't want struct autoaligning to kick in */
#define UUID_STRING_L 40

/* Offset to keyslot area [in bytes] */
#define LUKS_ALIGN_KEYSLOTS 4096

/* Any integer values are stored in network byte order on disk and must be
converted */

struct volume_key;
struct device_backend;

struct luks_phdr {
	char		magic[LUKS_MAGIC_L];
	uint16_t	version;
	char		cipherName[LUKS_CIPHERNAME_L];
	char		cipherMode[LUKS_CIPHERMODE_L];
	char            hashSpec[LUKS_HASHSPEC_L];
	uint32_t	payloadOffset;
	uint32_t	keyBytes;
	char		mkDigest[LUKS_DIGESTSIZE];
	char		mkDigestSalt[LUKS_SALTSIZE];
	uint32_t	mkDigestIterations;
	char            uuid[UUID_STRING_L];

	struct {
		uint32_t active;

		/* parameters used for password processing */
		uint32_t passwordIterations;
		char     passwordSalt[LUKS_SALTSIZE];

		/* parameters used for AF store/load */
		uint32_t keyMaterialOffset;
		uint32_t stripes;
	} keyblock[LUKS_NUMKEYS];

	/* Align it to 512 sector size */
	char		_padding[432];
};

int LUKS_generate_phdr(
	struct luks_phdr *header,
	const struct volume_key *vk,
	const char *cipherName,
	const char *cipherMode,
	const char *hashSpec,
	unsigned int stripes,
	unsigned int alignPayload,
	uint32_t iteration_time_ms,
	uint64_t *PBKDF2_per_sec,
	int detached_metadata_device);

int LUKS_read_phdr(
	struct luks_phdr *hdr,
	int require_luks_device,
	int repair,
	struct crypt_device *ctx);

int LUKS_write_phdr(
	struct luks_phdr *hdr,
	struct crypt_device *ctx);

int LUKS1_activate(struct crypt_device *cd,
		   const char *name,
		   struct volume_key *vk,
		   uint32_t flags);

#endif