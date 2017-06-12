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

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>

#include "luks.h"
#include "af.h"
#include "internal.h"
#include "sonos_common.h"

/* Get size of struct luks_phdr with all keyslots material space */
static size_t LUKS_device_sectors(size_t keyLen)
{
	size_t keyslot_sectors, sector;
	int i;

	keyslot_sectors = AF_split_sectors(keyLen, LUKS_STRIPES);
	sector = LUKS_ALIGN_KEYSLOTS / SECTOR_SIZE;

	for (i = 0; i < LUKS_NUMKEYS; i++) {
		sector = size_round_up(sector, LUKS_ALIGN_KEYSLOTS / SECTOR_SIZE);
		sector += keyslot_sectors;
	}

	return sector;
}

static int LUKS_check_device_size(struct crypt_device *ctx, size_t keyLength)
{
	struct device *device = crypt_metadata_device(ctx);
	uint64_t dev_sectors, hdr_sectors;

	if (!keyLength)
		return -EINVAL;

	if(device_size(device, &dev_sectors)) {
		syslog(LOG_ERR, "Cannot get device size for device %s.", device_path(device));
		return -EIO;
	}

	dev_sectors >>= SECTOR_SHIFT;
	hdr_sectors = LUKS_device_sectors(keyLength);

	if (hdr_sectors > dev_sectors) {
		syslog(LOG_ERR, "Device %s is too small.", device_path(device));
		return -EINVAL;
	}

	return 0;
}

/* Check keyslot to prevent access outside of header and keyslot area */
static int LUKS_check_keyslot_size(const struct luks_phdr *phdr, unsigned int keyIndex)
{
	uint32_t secs_per_stripes;

	/* First sectors is the header itself */
	if (phdr->keyblock[keyIndex].keyMaterialOffset * SECTOR_SIZE < sizeof(*phdr)) {
		syslog(LOG_ERR, "Invalid offset %u in keyslot %u.",
			phdr->keyblock[keyIndex].keyMaterialOffset, keyIndex);
		return 1;
	}

	/* Ignore following check for detached header where offset can be zero. */
	if (phdr->payloadOffset == 0)
		return 0;

	if (phdr->payloadOffset <= phdr->keyblock[keyIndex].keyMaterialOffset) {
		syslog(LOG_ERR, "Invalid offset %u in keyslot %u (beyond data area offset %u).",
			phdr->keyblock[keyIndex].keyMaterialOffset, keyIndex,
			phdr->payloadOffset);
		return 1;
	}

	secs_per_stripes = AF_split_sectors(phdr->keyBytes, phdr->keyblock[keyIndex].stripes);

	if (phdr->payloadOffset < (phdr->keyblock[keyIndex].keyMaterialOffset + secs_per_stripes)) {
		syslog(LOG_ERR, "Invalid keyslot size %u (offset %u, stripes %u) in "
			"keyslot %u (beyond data area offset %u).",
			secs_per_stripes,
			phdr->keyblock[keyIndex].keyMaterialOffset,
			phdr->keyblock[keyIndex].stripes,
			keyIndex, phdr->payloadOffset);
		return 1;
	}

	return 0;
}

static int _check_and_convert_hdr(const char *device,
				  struct luks_phdr *hdr,
				  int require_luks_device)
{
	int r = 0;
	unsigned int i;
	char luksMagic[] = LUKS_MAGIC;

	if(memcmp(hdr->magic, luksMagic, LUKS_MAGIC_L)) { /* Check magic */
		syslog(LOG_ERR, "LUKS header not detected.");
		if (require_luks_device)
			syslog(LOG_ERR, "Device %s is not a valid LUKS device.", device);
		return -EINVAL;
	} else if((hdr->version = ntohs(hdr->version)) != 1) {	/* Convert every uint16/32_t item from network byte order */
		syslog(LOG_ERR, "Unsupported LUKS version %d.", hdr->version);
		return -EINVAL;
	}

	hdr->hashSpec[LUKS_HASHSPEC_L - 1] = '\0';

	/* Header detected */
	hdr->payloadOffset      = ntohl(hdr->payloadOffset);
	hdr->keyBytes           = ntohl(hdr->keyBytes);
	hdr->mkDigestIterations = ntohl(hdr->mkDigestIterations);

	for(i = 0; i < LUKS_NUMKEYS; ++i) {
		hdr->keyblock[i].active             = ntohl(hdr->keyblock[i].active);
		hdr->keyblock[i].passwordIterations = ntohl(hdr->keyblock[i].passwordIterations);
		hdr->keyblock[i].keyMaterialOffset  = ntohl(hdr->keyblock[i].keyMaterialOffset);
		hdr->keyblock[i].stripes            = ntohl(hdr->keyblock[i].stripes);
		if (LUKS_check_keyslot_size(hdr, i)) {
			syslog(LOG_ERR, "LUKS keyslot %u is invalid.", i);
			r = -EINVAL;
		}
	}

	/* Avoid unterminated strings */
	hdr->cipherName[LUKS_CIPHERNAME_L - 1] = '\0';
	hdr->cipherMode[LUKS_CIPHERMODE_L - 1] = '\0';
	hdr->uuid[UUID_STRING_L - 1] = '\0';
	return r;
}

int LUKS_read_phdr(struct luks_phdr *hdr,
		   int require_luks_device,
		   int repair,
		   struct crypt_device *ctx)
{
	struct device *device = crypt_metadata_device(ctx);
	ssize_t hdr_size = sizeof(struct luks_phdr);
	int devfd = 0, r = 0;

	if (repair && !require_luks_device)
		return -EINVAL;

	devfd = device_open(device, O_RDONLY);
	if (devfd == -1) {
		syslog(LOG_ERR, "Cannot open device %s.", device_path(device));
		return -EINVAL;
	}

	if (read_blockwise(devfd, device_block_size(device), hdr, hdr_size)
		< hdr_size) {
		r = -EIO;
	} else
		r = _check_and_convert_hdr(device_path(device), hdr, require_luks_device);

	if (!r)
		r = LUKS_check_device_size(ctx, hdr->keyBytes);

	close(devfd);
	return r;
}

int LUKS_write_phdr(struct luks_phdr *hdr,
		    struct crypt_device *ctx)
{
	struct device *device = crypt_metadata_device(ctx);
	ssize_t hdr_size = sizeof(struct luks_phdr);
	int devfd = 0;
	unsigned int i;
	struct luks_phdr convHdr;
	int r;

	r = LUKS_check_device_size(ctx, hdr->keyBytes);
	if (r)
		return r;

	devfd = device_open(device, O_RDWR);
	if(-1 == devfd) {
		if (errno == EACCES)
			syslog(LOG_ERR, "Cannot write to device %s, permission denied.",
				device_path(device));
		else
			syslog(LOG_ERR, "Cannot open device %s.", device_path(device));
		return -EINVAL;
	}

	memcpy(&convHdr, hdr, hdr_size);
	memset(&convHdr._padding, 0, sizeof(convHdr._padding));

	/* Convert every uint16/32_t item to network byte order */
	convHdr.version            = htons(hdr->version);
	convHdr.payloadOffset      = htonl(hdr->payloadOffset);
	convHdr.keyBytes           = htonl(hdr->keyBytes);
	convHdr.mkDigestIterations = htonl(hdr->mkDigestIterations);
	for(i = 0; i < LUKS_NUMKEYS; ++i) {
		convHdr.keyblock[i].active             = htonl(hdr->keyblock[i].active);
		convHdr.keyblock[i].passwordIterations = htonl(hdr->keyblock[i].passwordIterations);
		convHdr.keyblock[i].keyMaterialOffset  = htonl(hdr->keyblock[i].keyMaterialOffset);
		convHdr.keyblock[i].stripes            = htonl(hdr->keyblock[i].stripes);
	}

	r = write_blockwise(devfd, device_block_size(device), &convHdr, hdr_size) < hdr_size ? -EIO : 0;
	if (r)
		syslog(LOG_ERR, "Error during update of LUKS header on device %s.",
			device_path(device));
	close(devfd);

	if (!r) {
		r = LUKS_read_phdr(hdr, 1, 0, ctx);
		if (r)
			syslog(LOG_ERR, "Error re-reading LUKS header after update on device %s.",
				device_path(device));
	}

	return r;
}

int LUKS_generate_phdr(struct luks_phdr *header,
		       const struct volume_key *vk,
		       const char *cipherName, const char *cipherMode, const char *hashSpec,
		       unsigned int stripes,
		       unsigned int alignPayload,
		       uint32_t iteration_time_ms,
		       uint64_t *PBKDF2_per_sec,
		       int detached_metadata_device)
{
	unsigned int i=0;
	size_t blocksPerStripeSet, currentSector;
	char luksMagic[] = LUKS_MAGIC;

	/* For separate metadata device allow zero alignment */
	if (alignPayload == 0 && !detached_metadata_device)
		alignPayload = DEFAULT_DISK_ALIGNMENT / SECTOR_SIZE;

	memset(header,0,sizeof(struct luks_phdr));

	/* Set Magic */
	memcpy(header->magic,luksMagic,LUKS_MAGIC_L);
	header->version=1;
	strncpy(header->cipherName, cipherName, sizeof(header->cipherName)-1);
	header->cipherName[sizeof(header->cipherName)-1] = '\0';
	strncpy(header->cipherMode, cipherMode, sizeof(header->cipherMode)-1);
	header->cipherMode[sizeof(header->cipherMode)-1] = '\0';
	strncpy(header->hashSpec, hashSpec, sizeof(header->hashSpec)-1);
	header->hashSpec[sizeof(header->hashSpec)-1] = '\0';

	header->keyBytes=vk->keylength;

	/* Compute master key digest */
	iteration_time_ms /= 8;
	header->mkDigestIterations = at_least((uint32_t)(*PBKDF2_per_sec/1024) * iteration_time_ms,
					      LUKS_MKD_ITERATIONS_MIN);
	currentSector = LUKS_ALIGN_KEYSLOTS / SECTOR_SIZE;
	blocksPerStripeSet = AF_split_sectors(vk->keylength, stripes);
	for(i = 0; i < LUKS_NUMKEYS; ++i) {
		header->keyblock[i].active = LUKS_KEY_DISABLED;
		header->keyblock[i].keyMaterialOffset = currentSector;
		header->keyblock[i].stripes = stripes;
		currentSector = size_round_up(currentSector + blocksPerStripeSet,
						LUKS_ALIGN_KEYSLOTS / SECTOR_SIZE);
	}
	header->payloadOffset = 4096;
	return 0;
}

int LUKS1_activate(struct crypt_device *cd,
		   const char *name,
		   struct volume_key *vk,
		   uint32_t flags)
{
	int r;
	char *dm_cipher = NULL;
	enum devcheck device_check;
	struct crypt_dm_active_device dmd = {
		.target = DM_CRYPT,
		.uuid   = crypt_get_uuid(cd),
		.flags  = flags,
		.size   = 0,
		.data_device = crypt_data_device(cd),
		.u.crypt = {
			.cipher = NULL,
			.vk     = vk,
			.offset = crypt_get_data_offset(cd),
			.iv_offset = 0,
		}
	};

	if (dmd.flags & CRYPT_ACTIVATE_SHARED)
		device_check = DEV_SHARED;
	else
		device_check = DEV_EXCL;

	r = device_block_adjust(dmd.data_device, device_check,
				 dmd.u.crypt.offset, &dmd.size, &dmd.flags);
	if (r)
		return r;

	r = asprintf(&dm_cipher, "%s%s", crypt_get_cipher(cd), crypt_get_cipher_mode(cd));
	if (r < 0)
		return -ENOMEM;

	dmd.u.crypt.cipher = dm_cipher;
	r = dm_create_device(cd, name, CRYPT_LUKS1, &dmd, 0);

	free(dm_cipher);
	return r;
}
