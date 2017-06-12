/*
 * libcryptsetup - cryptsetup library
 *
 * Copyright (C) 2004, Christophe Saout <christophe@saout.de>
 * Copyright (C) 2004-2007, Clemens Fruhwirth <clemens@endorphin.org>
 * Copyright (C) 2009-2012, Red Hat, Inc. All rights reserved.
 * Copyright (C) 2009-2013, Milan Broz
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>

#include "libcryptsetup.h"
#include "luks.h"
#include "internal.h"
#include "sonos_common.h"

struct crypt_device {
	char *type;

	struct device *device;
	struct device *metadata_device;

	struct volume_key *volume_key;
	uint64_t timeout;
	uint64_t iteration_time;
	int tries;
	int password_verify;
	int rng_type;

	// FIXME: private binary headers and access it properly
	// through sub-library (LUKS1, TCRYPT)

	union {
	struct { /* used in CRYPT_LUKS1 */
		struct luks_phdr hdr;
		uint64_t PBKDF2_per_sec;
	} luks1;
	} u;

	/* callbacks definitions */
	void (*log)(int level, const char *msg, void *usrptr);
	void *log_usrptr;
	int (*confirm)(const char *msg, void *usrptr);
	void *confirm_usrptr;
	int (*password)(const char *msg, char *buf, size_t length, void *usrptr);
	void *password_usrptr;

	/* last error message */
	char error[MAX_ERROR_LENGTH];
};


/* Log helper */
static void (*_default_log)(int level, const char *msg, void *usrptr) = NULL;

static const char *mdata_device_path(struct crypt_device *cd)
{
	return device_path(cd->metadata_device ?: cd->device);
}

/* internal only */
struct device *crypt_metadata_device(struct crypt_device *cd)
{
	return cd->metadata_device ?: cd->device;
}

struct device *crypt_data_device(struct crypt_device *cd)
{
	return cd->device;
}

static int isLUKS(const char *type)
{
	return (type && !strcmp(CRYPT_LUKS1, type));
}

/*
 * compares UUIDs returned by device-mapper (striped by cryptsetup) and uuid in header
 */
static int crypt_uuid_cmp(const char *dm_uuid, const char *hdr_uuid)
{
	int i, j;
	char *str;

	if (!dm_uuid || !hdr_uuid)
		return -EINVAL;

	str = strchr(dm_uuid, '-');
	if (!str)
		return -EINVAL;

	for (i = 0, j = 1; hdr_uuid[i]; i++) {
		if (hdr_uuid[i] == '-')
			continue;

		if (!str[j] || str[j] == '-')
			return -EINVAL;

		if (str[j] != hdr_uuid[i])
			return -EINVAL;
		j++;
	}

	return 0;
}

int crypt_confirm(struct crypt_device *cd, const char *msg)
{
	if (!cd || !cd->confirm)
		return 1;
	else
		return cd->confirm(msg, cd->confirm_usrptr);
}

void crypt_set_log_callback(struct crypt_device *cd,
	void (*log)(int level, const char *msg, void *usrptr),
	void *usrptr)
{
	if (!cd)
		_default_log = log;
	else {
		cd->log = log;
		cd->log_usrptr = usrptr;
	}
}

void crypt_set_confirm_callback(struct crypt_device *cd,
	int (*confirm)(const char *msg, void *usrptr),
	void *usrptr)
{
	cd->confirm = confirm;
	cd->confirm_usrptr = usrptr;
}

void crypt_set_password_callback(struct crypt_device *cd,
	int (*password)(const char *msg, char *buf, size_t length, void *usrptr),
	void *usrptr)
{
	cd->password = password;
	cd->password_usrptr = usrptr;
}

int crypt_init(struct crypt_device **cd, const char *device)
{
	struct crypt_device *h = NULL;
	int r;

	if (!cd)
		return -EINVAL;

	if (!(h = malloc(sizeof(struct crypt_device))))
		return -ENOMEM;

	memset(h, 0, sizeof(*h));

	r = device_alloc(&h->device, device);
	if (r < 0)
		goto bad;

	h->iteration_time = 1000;
	h->password_verify = 0;
	h->tries = 3;
	*cd = h;
	return 0;
bad:
	device_free(h->device);
	free(h);
	return r;
}

static int crypt_check_data_device_size(struct crypt_device *cd)
{
	int r;
	uint64_t size, size_min;

	/* Check data device size, require at least one sector */
	size_min = crypt_get_data_offset(cd) << SECTOR_SHIFT ?: SECTOR_SIZE;

	r = device_size(cd->device, &size);
	if (r < 0)
		return r;

	if (size < size_min) {
		syslog(LOG_ERR, "Header detected but device %s is too small.",
			device_path(cd->device));
		return -EINVAL;
	}

	return r;
}

int crypt_set_data_device(struct crypt_device *cd, const char *device)
{
	struct device *dev = NULL;
	int r;

	/* metadata device must be set */
	if (!cd->device || !device)
		return -EINVAL;

	r = device_alloc(&dev, device);
	if (r < 0)
		return r;

	if (!cd->metadata_device) {
		cd->metadata_device = cd->device;
	} else
		device_free(cd->device);

	cd->device = dev;

	return crypt_check_data_device_size(cd);
}

static int _crypt_load_luks1(struct crypt_device *cd, int require_header, int repair)
{
	struct luks_phdr hdr;
	int r;

	r = LUKS_read_phdr(&hdr, require_header, repair, cd);
	if (r < 0)
		return r;

	if (!cd->type && !(cd->type = strdup(CRYPT_LUKS1)))
		return -ENOMEM;

	memcpy(&cd->u.luks1.hdr, &hdr, sizeof(hdr));

	return r;
}

static int _init_by_name_crypt(struct crypt_device *cd, const char *name)
{
	struct crypt_dm_active_device dmd = {0};
	int r;

	r = dm_query_device(cd, name,
			DM_ACTIVE_DEVICE |
			DM_ACTIVE_UUID |
			DM_ACTIVE_CRYPT_CIPHER |
			DM_ACTIVE_CRYPT_KEYSIZE, &dmd);
	if (r < 0)
		goto out;

	if (isLUKS(cd->type)) {
		if (crypt_metadata_device(cd)) {
			r = _crypt_load_luks1(cd, 0, 0);
			if (r < 0) {
				syslog(LOG_ERR, "LUKS device header does not match active device.");
				free(cd->type);
				cd->type = NULL;
				r = 0;
				goto out;
			}
			/* check whether UUIDs match each other */
			r = crypt_uuid_cmp(dmd.uuid, cd->u.luks1.hdr.uuid);
			if (r < 0) {
				syslog(LOG_ERR, "LUKS device header uuid: %s mismatches DM returned uuid %s",
					cd->u.luks1.hdr.uuid, dmd.uuid);
				free(cd->type);
				cd->type = NULL;
				r = 0;
			}
		} else {
			syslog(LOG_ERR, "LUKS device header not available.");
			free(cd->type);
			cd->type = NULL;
			r = 0;
		}
	} 
out:
	crypt_free_volume_key(dmd.u.crypt.vk);
	device_free(dmd.data_device);
	free(CONST_CAST(void*)dmd.u.crypt.cipher);
	free(CONST_CAST(void*)dmd.uuid);
	return r;
}

int crypt_init_by_name_and_header(struct crypt_device **cd,
				  const char *name,
				  const char *header_device)
{
	crypt_status_info ci;
	struct crypt_dm_active_device dmd;
	int r;

	ci = crypt_status(NULL, name);
	if (ci == CRYPT_INVALID)
		return -ENODEV;

	if (ci < CRYPT_ACTIVE) {
		syslog(LOG_ERR, "Device %s is not active.", name);
		return -ENODEV;
	}

	r = dm_query_device(NULL, name, DM_ACTIVE_DEVICE | DM_ACTIVE_UUID, &dmd);
	if (r < 0)
		goto out;

	*cd = NULL;

	if (header_device) {
		r = crypt_init(cd, header_device);
	} else {
		r = crypt_init(cd, device_path(dmd.data_device));

		/* Underlying device disappeared but mapping still active */
		if (!dmd.data_device || r == -ENOTBLK)
			syslog(LOG_ERR, "Underlying device for crypt device %s disappeared.",
				    name);

		/* Underlying device is not readable but crypt mapping exists */
		if (r == -ENOTBLK) {
			device_free(dmd.data_device);
			dmd.data_device = NULL;
			r = crypt_init(cd, NULL);
		}
	}

	if (r < 0)
		goto out;

	if (dmd.uuid) {
		if (!strncmp(CRYPT_PLAIN, dmd.uuid, sizeof(CRYPT_PLAIN)-1))
			(*cd)->type = strdup(CRYPT_PLAIN);
		else if (!strncmp(CRYPT_LOOPAES, dmd.uuid, sizeof(CRYPT_LOOPAES)-1))
			(*cd)->type = strdup(CRYPT_LOOPAES);
		else if (!strncmp(CRYPT_LUKS1, dmd.uuid, sizeof(CRYPT_LUKS1)-1))
			(*cd)->type = strdup(CRYPT_LUKS1);
		else if (!strncmp(CRYPT_VERITY, dmd.uuid, sizeof(CRYPT_VERITY)-1))
			(*cd)->type = strdup(CRYPT_VERITY);
		else if (!strncmp(CRYPT_TCRYPT, dmd.uuid, sizeof(CRYPT_TCRYPT)-1))
			(*cd)->type = strdup(CRYPT_TCRYPT);
		else
			syslog(LOG_ERR, "Unknown UUID set, some parameters are not set.");
	} else
		syslog(LOG_ERR, "Active device has no UUID set, some parameters are not set.");

	if (header_device) {
		r = crypt_set_data_device(*cd, device_path(dmd.data_device));
		if (r < 0)
			goto out;
	}

	if (dmd.target == DM_CRYPT)
		r = _init_by_name_crypt(*cd, name);
out:
	if (r < 0) {
		crypt_free(*cd);
		*cd = NULL;
	}
	device_free(dmd.data_device);
	free(CONST_CAST(void*)dmd.uuid);
	return r;
}

int crypt_init_by_name(struct crypt_device **cd, const char *name)
{
	return crypt_init_by_name_and_header(cd, name, NULL);
}

static int _crypt_format_luks1(struct crypt_device *cd,
			       const char *cipher,
			       const char *cipher_mode,
			       const char *volume_key,
			       size_t volume_key_size,
			       struct crypt_params_luks1 *params)
{
	int r;
	unsigned long required_alignment = DEFAULT_DISK_ALIGNMENT;
	unsigned long alignment_offset = 0;

	if (!crypt_metadata_device(cd)) {
		syslog(LOG_ERR, "Can't format LUKS without device.\n");
		return -EINVAL;
	}

	if (!(cd->type = strdup(CRYPT_LUKS1)))
		return -ENOMEM;

	if (volume_key)
		cd->volume_key = crypt_alloc_volume_key(volume_key_size,
						      volume_key);
	else
		cd->volume_key = crypt_alloc_volume_key(volume_key_size, NULL);

	if(!cd->volume_key)
		return -ENOMEM;

	if (params && params->data_device) {
		cd->metadata_device = cd->device;
		cd->device = NULL;
		if (device_alloc(&cd->device, params->data_device) < 0)
			return -ENOMEM;
		required_alignment = params->data_alignment * SECTOR_SIZE;
	} else if (params && params->data_alignment) {
		required_alignment = params->data_alignment * SECTOR_SIZE;
	} else
		device_topology_alignment(cd->device,
				       &required_alignment,
				       &alignment_offset, DEFAULT_DISK_ALIGNMENT);

	/* Check early if we cannot allocate block device for key slot access */
	r = device_block_adjust(cd->device, DEV_OK, 0, NULL, NULL);
	if(r < 0)
		return r;

	r = LUKS_generate_phdr(&cd->u.luks1.hdr, cd->volume_key, cipher, cipher_mode,
			       (params && params->hash) ? params->hash : "sha1",
			       LUKS_STRIPES,
			       required_alignment / SECTOR_SIZE,
			       cd->iteration_time, &cd->u.luks1.PBKDF2_per_sec,
			       cd->metadata_device ? 1 : 0);
	if(r < 0)
		return r;

	/* Wipe first 8 sectors - fs magic numbers etc. */
	r = crypt_wipe(crypt_metadata_device(cd), 0, 8 * SECTOR_SIZE);
	if(r < 0) {
		if (r == -EBUSY)
			syslog(LOG_ERR, "Cannot format device %s which is still in use.",
				mdata_device_path(cd));
		else if (r == -EACCES) {
			syslog(LOG_ERR, "Cannot format device %s, permission denied.",
				mdata_device_path(cd));
			r = -EINVAL;
		} else
			syslog(LOG_ERR, "Cannot wipe header on device %s.",
				mdata_device_path(cd));

		return r;
	}

	r = LUKS_write_phdr(&cd->u.luks1.hdr, cd);

	return r;
}

int crypt_format(struct crypt_device *cd,
	const char *cipher,
	const char *cipher_mode,
	const char *volume_key,
	size_t volume_key_size,
	void *params)
{
	int r;

	if (cd->type) {
		syslog(LOG_ERR, "Context already formatted as %s.", cd->type);
		return -EINVAL;
	}

	r = _crypt_format_luks1(cd, cipher, cipher_mode,
		volume_key, volume_key_size, params);

	if (r < 0) {
		free(cd->type);
		cd->type = NULL;
		crypt_free_volume_key(cd->volume_key);
		cd->volume_key = NULL;
	}

	return r;
}

int crypt_load(struct crypt_device *cd,
	       const char *requested_type)
{
	int r;

	if (!crypt_metadata_device(cd))
		return -EINVAL;

	if (!requested_type || isLUKS(requested_type)) {
		if (cd->type && !isLUKS(cd->type)) {
			syslog(LOG_ERR, "Context is already initialised to type %s", cd->type);
			return -EINVAL;
		}

		r = _crypt_load_luks1(cd, 1, 0);
	} else
		return -EINVAL;

	return r;
}

void crypt_free(struct crypt_device *cd)
{
	if (cd) {
		crypt_free_volume_key(cd->volume_key);

		device_free(cd->device);
		device_free(cd->metadata_device);

		free(cd->type);
		memset(cd, 0, sizeof(*cd));
		free(cd);
	}
}

int sonos_crypt_activate(struct crypt_device *cd,
	const char *name,
	int keyslot,
	uint32_t flags)
{
	crypt_status_info ci;
	struct volume_key *vk = NULL;
	int r = -1;

	if (name) {
		ci = crypt_status(NULL, name);
		if (ci == CRYPT_INVALID)
			return -EINVAL;
		else if (ci >= CRYPT_ACTIVE) {
			syslog(LOG_ERR, "Device %s already exists.", name);
			return -EEXIST;
		}
	}

	/* zero key is a special sentinel for our kernel */
	vk = crypt_alloc_volume_key(KEY_LEN, NULL);
	if (vk) {
		r = 0;
		keyslot = r;
		if (name) {
			r = LUKS1_activate(cd, name, vk, flags);
		}
	}
	crypt_free_volume_key(vk);

	return r < 0  ? r : keyslot;
}

int sonos_crypt_deactivate(struct crypt_device *cd, const char *name)
{
	int r;

	if (!name)
		return -EINVAL;

	switch (crypt_status(cd, name)) {
		case CRYPT_ACTIVE:
		case CRYPT_BUSY:
			r = dm_remove_device(cd, name, 0, 0);
			if (r < 0 && crypt_status(cd, name) == CRYPT_BUSY) {
				syslog(LOG_INFO, "Device %s is still in use.", name);
				r = -EBUSY;
			}
			break;
		case CRYPT_INACTIVE:
			syslog(LOG_INFO, "Device %s is not active.", name);
			r = -ENODEV;
			break;
		default:
			syslog(LOG_ERR, "Invalid device %s.", name);
			r = -EINVAL;
	}
	return r;
}

// reporting
crypt_status_info crypt_status(struct crypt_device *cd, const char *name)
{
	int r;

	r = dm_status_device(cd, name);

	if (r < 0 && r != -ENODEV)
		return CRYPT_INVALID;

	if (r == 0)
		return CRYPT_ACTIVE;

	if (r > 0)
		return CRYPT_BUSY;

	return CRYPT_INACTIVE;
}

const char *crypt_get_cipher(struct crypt_device *cd)
{
	return cd->u.luks1.hdr.cipherName;
}

const char *crypt_get_cipher_mode(struct crypt_device *cd)
{
	return cd->u.luks1.hdr.cipherMode;
}

const char *crypt_get_uuid(struct crypt_device *cd)
{
	if (isLUKS(cd->type))
		return cd->u.luks1.hdr.uuid;

	return NULL;
}

const char *crypt_get_device_name(struct crypt_device *cd)
{
	const char *path = device_block_path(cd->device);

	if (!path)
		path = device_path(cd->device);

	return path;
}

int crypt_get_volume_key_size(struct crypt_device *cd)
{
	if (isLUKS(cd->type))
		return cd->u.luks1.hdr.keyBytes;
	return 0;
}

uint64_t crypt_get_data_offset(struct crypt_device *cd)
{
	if (isLUKS(cd->type))
		return cd->u.luks1.hdr.payloadOffset;
	return 0;
}

uint64_t crypt_get_iv_offset(struct crypt_device *cd)
{
	if (isLUKS(cd->type))
		return 0;
	return 0;
}

int crypt_keyslot_max(const char *type)
{
	if (type && isLUKS(type))
		return LUKS_NUMKEYS;

	return -EINVAL;
}

const char *crypt_get_type(struct crypt_device *cd)
{
	return cd->type;
}
