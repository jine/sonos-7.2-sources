/*
 * libdevmapper - device-mapper backend for cryptsetup
 *
 * Copyright (C) 2004, Christophe Saout <christophe@saout.de>
 * Copyright (C) 2004-2007, Clemens Fruhwirth <clemens@endorphin.org>
 * Copyright (C) 2009-2012, Red Hat, Inc. All rights reserved.
 * Copyright (C) 2009-2012, Milan Broz
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
#include <dirent.h>
#include <errno.h>
#include <libdevmapper.h>
#include <fcntl.h>
#include <linux/fs.h>

#include "internal.h"
#include "sonos_common.h"

#define DM_UUID_LEN		129
#define DM_UUID_PREFIX		"CRYPT-"
#define DM_UUID_PREFIX_LEN	6
#define DM_CRYPT_TARGET		"crypt"
#define DM_VERITY_TARGET	"verity"
#define RETRY_COUNT		5

/* Set if dm-crypt version was probed */
static int _dm_crypt_checked = 0;
static int _quiet_log = 0;
static uint32_t _dm_crypt_flags = 0;

static struct crypt_device *_context = NULL;

/* Compatibility for old device-mapper without udev support */
#define CRYPT_TEMP_UDEV_FLAGS	0
static int _dm_simple(int task, const char *name);

static void _dm_set_crypt_compat(const char *dm_version, unsigned crypt_maj,
				 unsigned crypt_min, unsigned crypt_patch)
{
	unsigned dm_maj, dm_min, dm_patch;

	if (sscanf(dm_version, "%u.%u.%u", &dm_maj, &dm_min, &dm_patch) != 3)
		dm_maj = dm_min = dm_patch = 0;

	syslog(LOG_INFO, "Detected dm-crypt version %i.%i.%i, dm-ioctl version %u.%u.%u.",
		crypt_maj, crypt_min, crypt_patch, dm_maj, dm_min, dm_patch);

	if (crypt_maj >= 1 && crypt_min >= 2)
		_dm_crypt_flags |= DM_KEY_WIPE_SUPPORTED;
	else
		syslog(LOG_INFO, "Suspend and resume disabled, no wipe key support.");

	if (crypt_maj >= 1 && crypt_min >= 10)
		_dm_crypt_flags |= DM_LMK_SUPPORTED;

	if (dm_maj >= 4 && dm_min >= 20)
		_dm_crypt_flags |= DM_SECURE_SUPPORTED;

	/* not perfect, 2.6.33 supports with 1.7.0 */
	if (crypt_maj >= 1 && crypt_min >= 8)
		_dm_crypt_flags |= DM_PLAIN64_SUPPORTED;

	if (crypt_maj >= 1 && crypt_min >= 11)
		_dm_crypt_flags |= DM_DISCARDS_SUPPORTED;

	/* Repeat test if dm-crypt is not present */
	if (crypt_maj > 0)
		_dm_crypt_checked = 1;
}

static int _dm_check_versions(void)
{
	struct dm_task *dmt;
	struct dm_versions *target, *last_target;
	char dm_version[16];
	int r = 0;

	if (_dm_crypt_checked)
		return 1;

	/* Shut up DM while checking */
	_quiet_log = 1;

	/* FIXME: add support to DM so it forces crypt target module load here */
	if (!(dmt = dm_task_create(DM_DEVICE_LIST_VERSIONS)))
		goto out;

	if (!dm_task_run(dmt))
		goto out;

	if (!dm_task_get_driver_version(dmt, dm_version, sizeof(dm_version)))
		goto out;

	target = dm_task_get_versions(dmt);
	do {
		last_target = target;
		if (!strcmp(DM_CRYPT_TARGET, target->name)) {
			_dm_set_crypt_compat(dm_version,
					     (unsigned)target->version[0],
					     (unsigned)target->version[1],
					     (unsigned)target->version[2]);
		}
		target = (struct dm_versions *)((char *) target + target->next);
	} while (last_target != target);

	r = 1;
	syslog(LOG_INFO, "Device-mapper backend running with UDEV support %sabled.",
		"dis");
out:
	if (dmt)
		dm_task_destroy(dmt);

	_quiet_log = 0;
	return r;
}

uint32_t dm_flags(void)
{
	_dm_check_versions();
	return _dm_crypt_flags;
}

/*
 * libdevmapper is not context friendly, switch context on every DM call.
 * FIXME: this is not safe if called in parallel but neither is DM lib.
 */
static int dm_init_context(struct crypt_device *cd)
{
	_context = cd;
	if (!_dm_check_versions()) {
		syslog(LOG_ERR, "Cannot initialize device-mapper. "
				      "Is dm_mod kernel module loaded?");
		_context = NULL;
		return -ENOTSUP;
	}
	return 0;
}
static void dm_exit_context(void)
{
	_context = NULL;
}

/* Return path to DM device */
char *dm_device_path(const char *prefix, int major, int minor)
{
	struct dm_task *dmt;
	const char *name;
	char path[PATH_MAX];

	if (!(dmt = dm_task_create(DM_DEVICE_STATUS)))
		return NULL;
	if (!dm_task_set_minor(dmt, minor) ||
	    !dm_task_set_major(dmt, major) ||
	    !dm_task_run(dmt) ||
	    !(name = dm_task_get_name(dmt))) {
		dm_task_destroy(dmt);
		return NULL;
	}

	if (snprintf(path, sizeof(path), "%s%s", prefix ?: "", name) < 0)
		path[0] = '\0';

	dm_task_destroy(dmt);

	return strdup(path);
}

static void hex_key(char *hexkey, size_t key_size, const char *key)
{
	unsigned i;

	for(i = 0; i < key_size; i++)
		sprintf(&hexkey[i * 2], "%02x", (unsigned char)key[i]);
}

/* http://code.google.com/p/cryptsetup/wiki/DMCrypt */
static char *get_dm_crypt_params(struct crypt_dm_active_device *dmd)
{
	int r, max_size, null_cipher = 0;
	char *params, *hexkey;
	const char *features = "";
	const uint8_t zeroes[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0
	};
	size_t hexkeySpace;

	if (!dmd)
		return NULL;

	if (dmd->flags & CRYPT_ACTIVATE_ALLOW_DISCARDS) {
		if (dm_flags() & DM_DISCARDS_SUPPORTED) {
			features = " 1 allow_discards";
			syslog(LOG_INFO, "Discard/TRIM is allowed.");
		} else
			syslog(LOG_INFO, "Discard/TRIM is not supported by the kernel.");
	}

	if (!strncmp(dmd->u.crypt.cipher, "cipher_null-", 12))
		null_cipher = 1;

	hexkeySpace = null_cipher ? 2 : (dmd->u.crypt.vk->keylength * 2 + 1);
	hexkey = crypt_safe_alloc(hexkeySpace);
	if (!hexkey)
		return NULL;

	if (null_cipher) {
		strncpy(hexkey, "-", 2);
	}
	/* translate one sentinel value into another */
	else if (dmd->u.crypt.vk->keylength == 16 &&
		 memcmp(dmd->u.crypt.vk->key, zeroes, 16) == 0) {
		const char *name = "mdp.rootfs.red";
		size_t nameLen = strlen(name);
		memset(hexkey, ':', hexkeySpace - 1);
		hexkey[hexkeySpace - 1] = '\0';
		memcpy(hexkey, name, nameLen);
	}
	else {
		hex_key(hexkey, dmd->u.crypt.vk->keylength, dmd->u.crypt.vk->key);
	}

	max_size = strlen(hexkey) + strlen(dmd->u.crypt.cipher) +
		   strlen(device_block_path(dmd->data_device)) +
		   strlen(features) + 64;
	params = crypt_safe_alloc(max_size);
	if (!params)
		goto out;

	r = snprintf(params, max_size, "%s %s %" PRIu64 " %s %" PRIu64 "%s",
		     dmd->u.crypt.cipher, hexkey, dmd->u.crypt.iv_offset,
		     device_block_path(dmd->data_device), dmd->u.crypt.offset,
		     features);
	if (r < 0 || r >= max_size) {
		crypt_safe_free(params);
		params = NULL;
	}
out:
	crypt_safe_free(hexkey);
	return params;
}

/* http://code.google.com/p/cryptsetup/wiki/DMVerity */
static char *get_dm_verity_params(struct crypt_params_verity *vp,
				   struct crypt_dm_active_device *dmd)
{
	int max_size, r;
	char *params = NULL, *hexroot = NULL, *hexsalt = NULL;

	if (!vp || !dmd)
		return NULL;

	hexroot = crypt_safe_alloc(dmd->u.verity.root_hash_size * 2 + 1);
	if (!hexroot)
		goto out;
	hex_key(hexroot, dmd->u.verity.root_hash_size, dmd->u.verity.root_hash);

	hexsalt = crypt_safe_alloc(vp->salt_size ? vp->salt_size * 2 + 1 : 2);
	if (!hexsalt)
		goto out;
	if (vp->salt_size)
		hex_key(hexsalt, vp->salt_size, vp->salt);
	else
		strncpy(hexsalt, "-", 2);

	max_size = strlen(hexroot) + strlen(hexsalt) +
		   strlen(device_block_path(dmd->data_device)) +
		   strlen(device_block_path(dmd->u.verity.hash_device)) +
		   strlen(vp->hash_name) + 128;

	params = crypt_safe_alloc(max_size);
	if (!params)
		goto out;

	r = snprintf(params, max_size,
		     "%u %s %s %u %u %" PRIu64 " %" PRIu64 " %s %s %s",
		     vp->hash_type, device_block_path(dmd->data_device),
		     device_block_path(dmd->u.verity.hash_device),
		     vp->data_block_size, vp->hash_block_size,
		     vp->data_size, dmd->u.verity.hash_offset,
		     vp->hash_name, hexroot, hexsalt);
	if (r < 0 || r >= max_size) {
		crypt_safe_free(params);
		params = NULL;
	}
out:
	crypt_safe_free(hexroot);
	crypt_safe_free(hexsalt);
	return params;

}

/* DM helpers */
static int _dm_simple(int task, const char *name)
{
	int r = 0;
	struct dm_task *dmt;

	if (!(dmt = dm_task_create(task)))
		return 0;

	if (name && !dm_task_set_name(dmt, name))
		goto out;

	r = dm_task_run(dmt);

      out:
	dm_task_destroy(dmt);
	return r;
}

static int _error_device(const char *name, size_t size)
{
	struct dm_task *dmt;
	int r = 0;

	if (!(dmt = dm_task_create(DM_DEVICE_RELOAD)))
		return 0;

	if (!dm_task_set_name(dmt, name))
		goto error;

	if (!dm_task_add_target(dmt, UINT64_C(0), size, "error", ""))
		goto error;

	if (!dm_task_set_ro(dmt))
		goto error;

	if (!dm_task_no_open_count(dmt))
		goto error;

	if (!dm_task_run(dmt))
		goto error;

	if (!_dm_simple(DM_DEVICE_RESUME, name)) {
		_dm_simple(DM_DEVICE_CLEAR, name);
		goto error;
	}

	r = 1;

error:
	dm_task_destroy(dmt);
	return r;
}

int dm_remove_device(struct crypt_device *cd, const char *name,
		     int force, uint64_t size)
{
	int r = -EINVAL;
	int retries = force ? RETRY_COUNT : 1;
	int error_target = 0;

	if (!name || (force && !size))
		return -EINVAL;

	if (dm_init_context(cd))
		return -ENOTSUP;

	do {
		r = _dm_simple(DM_DEVICE_REMOVE, name) ? 0 : -EINVAL;
		if (--retries && r) {
			syslog(LOG_ERR, "WARNING: other process locked internal device %s, %s.",
				name, retries ? "retrying remove" : "giving up");
			sleep(1);
			if (force && !error_target) {
				/* If force flag is set, replace device with error, read-only target.
				 * it should stop processes from reading it and also removed underlying
				 * device from mapping, so it is usable again.
				 * Force flag should be used only for temporary devices, which are
				 * intended to work inside cryptsetup only!
				 * Anyway, if some process try to read temporary cryptsetup device,
				 * it is bug - no other process should try touch it (e.g. udev).
				 */
				_error_device(name, size);
				error_target = 1;
			}
		}
	} while (r == -EINVAL && retries);

	dm_task_update_nodes();
	dm_exit_context();

	return r;
}

#define UUID_LEN 37 /* 36 + \0, libuuid ... */
/*
 * UUID has format: CRYPT-<devicetype>-[<uuid>-]<device name>
 * CRYPT-PLAIN-name
 * CRYPT-LUKS1-00000000000000000000000000000000-name
 * CRYPT-TEMP-name
 */
static int dm_prepare_uuid(const char *name, const char *type, char *buf, size_t buflen)
{
	unsigned i = 0;

	/* Remove '-' chars */
	i = snprintf(buf, buflen, DM_UUID_PREFIX "%s-%s",
		type, name);

	syslog(LOG_INFO, "DM-UUID is %s", buf);
	if (i >= buflen)
		syslog(LOG_ERR, "DM-UUID for device %s was truncated.", name);

	return 0;
}

static int _dm_create_device(const char *name, const char *type,
			     struct device *device, uint32_t flags,
			     const char *uuid, uint64_t size,
			     char *params, int reload)
{
	struct dm_task *dmt = NULL;
	struct dm_info dmi;
	char dev_uuid[DM_UUID_LEN] = {0};
	int r = -EINVAL;
	uint32_t read_ahead = 0;
	uint16_t udev_flags = 0;

	if (flags & CRYPT_ACTIVATE_PRIVATE)
		udev_flags = CRYPT_TEMP_UDEV_FLAGS;

	/* All devices must have DM_UUID, only resize on old device is exception */
	if (reload) {
		if (!(dmt = dm_task_create(DM_DEVICE_RELOAD)))
			goto out_no_removal;

		if (!dm_task_set_name(dmt, name))
			goto out_no_removal;
	} else {
		r = dm_prepare_uuid(name, type, dev_uuid, sizeof(dev_uuid));
		if (r < 0)
			return r;

		if (!(dmt = dm_task_create(DM_DEVICE_CREATE)))
			goto out_no_removal;

		if (!dm_task_set_name(dmt, name))
			goto out_no_removal;

		if (!dm_task_set_uuid(dmt, dev_uuid))
			goto out_no_removal;

	}

	if ((dm_flags() & DM_SECURE_SUPPORTED) && !dm_task_secure_data(dmt))
		goto out_no_removal;
	if ((flags & CRYPT_ACTIVATE_READONLY) && !dm_task_set_ro(dmt))
		goto out_no_removal;

	if (!dm_task_add_target(dmt, 0, size,
		!strcmp("VERITY", type) ? DM_VERITY_TARGET : DM_CRYPT_TARGET, params))
		goto out_no_removal;

#ifdef DM_READ_AHEAD_MINIMUM_FLAG
	if (device_read_ahead(device, &read_ahead) &&
	    !dm_task_set_read_ahead(dmt, read_ahead, DM_READ_AHEAD_MINIMUM_FLAG))
		goto out_no_removal;
#endif

	if (!dm_task_run(dmt))
		goto out_no_removal;

	if (reload) {
		dm_task_destroy(dmt);
		if (!(dmt = dm_task_create(DM_DEVICE_RESUME)))
			goto out;
		if (!dm_task_set_name(dmt, name))
			goto out;
		if (uuid && !dm_task_set_uuid(dmt, dev_uuid))
			goto out;
		if (!dm_task_run(dmt))
			goto out;
	}

	if (!dm_task_get_info(dmt, &dmi))
		goto out;

	r = 0;
out:
	if (r < 0 && !reload)
		_dm_simple(DM_DEVICE_REMOVE, name);

out_no_removal:

	if (params)
		crypt_safe_free(params);
	if (dmt)
		dm_task_destroy(dmt);

	dm_task_update_nodes();
	return r;
}

int dm_create_device(struct crypt_device *cd, const char *name,
		     const char *type,
		     struct crypt_dm_active_device *dmd,
		     int reload)
{
	char *table_params = NULL;
	int r = -EINVAL;

	if (!type)
		return -EINVAL;

	if (dm_init_context(cd))
		return -ENOTSUP;

	if (dmd->target == DM_CRYPT)
		table_params = get_dm_crypt_params(dmd);
	else if (dmd->target == DM_VERITY)
		table_params = get_dm_verity_params(dmd->u.verity.vp, dmd);

	//syslog(LOG_INFO, "table_params dm_create_device %s\n", table_params);
	if (table_params)
		r = _dm_create_device(name, type, dmd->data_device,
				      dmd->flags, dmd->uuid, dmd->size,
				      table_params, reload);
	dm_exit_context();
	return r;
}

static int dm_status_dmi(const char *name, struct dm_info *dmi,
			  const char *target, char **status_line)
{
	struct dm_task *dmt;
	uint64_t start, length;
	char *target_type, *params = NULL;
	void *next = NULL;
	int r = -EINVAL;

	if (!(dmt = dm_task_create(DM_DEVICE_STATUS)))
		goto out;

	if (!dm_task_set_name(dmt, name))
		goto out;

	if (!dm_task_run(dmt))
		goto out;

	if (!dm_task_get_info(dmt, dmi))
		goto out;

	if (!dmi->exists) {
		r = -ENODEV;
		goto out;
	}

	next = dm_get_next_target(dmt, next, &start, &length,
	                          &target_type, &params);

	if (!target_type || start != 0 || next)
		goto out;

	if (target && strcmp(target_type, target))
		goto out;

	/* for target == NULL check all supported */
	if (!target && (strcmp(target_type, DM_CRYPT_TARGET) &&
			strcmp(target_type, DM_VERITY_TARGET)))
		goto out;
	r = 0;
out:
	if (!r && status_line && !(*status_line = strdup(params)))
		r = -ENOMEM;

	if (dmt)
		dm_task_destroy(dmt);

	return r;
}

int dm_status_device(struct crypt_device *cd, const char *name)
{
	int r;
	struct dm_info dmi;
	struct stat st;

	/* libdevmapper is too clever and handles
	 * path argument differenly with error.
	 * Fail early here if parameter is non-existent path.
	 */
	if (strchr(name, '/') && stat(name, &st) < 0)
		return -ENODEV;

	if (dm_init_context(cd))
		return -ENOTSUP;
	r = dm_status_dmi(name, &dmi, NULL, NULL);
	dm_exit_context();
	if (r < 0)
		return r;

	return (dmi.open_count > 0);
}

/* FIXME use hex wrapper, user val wrappers for line parsing */
static int _dm_query_crypt(uint32_t get_flags,
			   struct dm_info *dmi,
			   char *params,
			   struct crypt_dm_active_device *dmd)
{
	uint64_t val64;
	char *rcipher, *key_, *rdevice, *endp, buffer[3], *arg;
	unsigned int i;
	int r;

	memset(dmd, 0, sizeof(*dmd));
	dmd->target = DM_CRYPT;

	rcipher = strsep(&params, " ");
	/* cipher */
	if (get_flags & DM_ACTIVE_CRYPT_CIPHER)
		dmd->u.crypt.cipher = strdup(rcipher);

	/* skip */
	key_ = strsep(&params, " ");
	if (!params)
		return -EINVAL;
	val64 = strtoull(params, &params, 10);
	if (*params != ' ')
		return -EINVAL;
	params++;

	dmd->u.crypt.iv_offset = val64;

	/* device */
	rdevice = strsep(&params, " ");
	if (get_flags & DM_ACTIVE_DEVICE) {
		arg = crypt_lookup_dev(rdevice);
		r = device_alloc(&dmd->data_device, arg);
		free(arg);
		if (r < 0 && r != -ENOTBLK)
			return r;
	}

	/*offset */
	if (!params)
		return -EINVAL;
	val64 = strtoull(params, &params, 10);
	dmd->u.crypt.offset = val64;

	/* Features section, available since crypt target version 1.11 */
	if (*params) {
		if (*params != ' ')
			return -EINVAL;
		params++;

		/* Number of arguments */
		val64 = strtoull(params, &params, 10);
		if (*params != ' ')
			return -EINVAL;
		params++;

		for (i = 0; i < val64; i++) {
			if (!params)
				return -EINVAL;
			arg = strsep(&params, " ");
			if (!strcasecmp(arg, "allow_discards"))
				dmd->flags |= CRYPT_ACTIVATE_ALLOW_DISCARDS;
			else /* unknown option */
				return -EINVAL;
		}

		/* All parameters shold be processed */
		if (params)
			return -EINVAL;
	}

	/* Never allow to return empty key */
	if ((get_flags & DM_ACTIVE_CRYPT_KEY) && dmi->suspended) {
		syslog(LOG_ERR, "Cannot read volume key while suspended.");
		return -EINVAL;
	}

	if (get_flags & DM_ACTIVE_CRYPT_KEYSIZE) {
		dmd->u.crypt.vk = crypt_alloc_volume_key(strlen(key_) / 2, NULL);
		if (!dmd->u.crypt.vk)
			return -ENOMEM;

		if (get_flags & DM_ACTIVE_CRYPT_KEY) {
			buffer[2] = '\0';
			for(i = 0; i < dmd->u.crypt.vk->keylength; i++) {
				memcpy(buffer, &key_[i * 2], 2);
				dmd->u.crypt.vk->key[i] = strtoul(buffer, &endp, 16);
				if (endp != &buffer[2]) {
					crypt_free_volume_key(dmd->u.crypt.vk);
					dmd->u.crypt.vk = NULL;
					return -EINVAL;
				}
			}
		}
	}
	memset(key_, 0, strlen(key_));

	return 0;
}

int dm_query_device(struct crypt_device *cd, const char *name,
		    uint32_t get_flags, struct crypt_dm_active_device *dmd)
{
	struct dm_task *dmt;
	struct dm_info dmi;
	uint64_t start, length;
	char *target_type, *params;
	const char *tmp_uuid;
	void *next = NULL;
	int r = -EINVAL;

	if (dm_init_context(cd))
		return -ENOTSUP;
	if (!(dmt = dm_task_create(DM_DEVICE_TABLE)))
		goto out;
	if ((dm_flags() & DM_SECURE_SUPPORTED) && !dm_task_secure_data(dmt))
		goto out;
	if (!dm_task_set_name(dmt, name))
		goto out;
	r = -ENODEV;
	if (!dm_task_run(dmt))
		goto out;

	r = -EINVAL;
	if (!dm_task_get_info(dmt, &dmi))
		goto out;

	if (!dmi.exists) {
		r = -ENODEV;
		goto out;
	}

	next = dm_get_next_target(dmt, next, &start, &length,
	                          &target_type, &params);

	if (!target_type || start != 0 || next)
		goto out;

	r = _dm_query_crypt(get_flags, &dmi, params, dmd);
	if (r < 0)
		goto out;

	dmd->size = length;

	if (dmi.read_only)
		dmd->flags |= CRYPT_ACTIVATE_READONLY;

	tmp_uuid = dm_task_get_uuid(dmt);
	if (!tmp_uuid)
		dmd->flags |= CRYPT_ACTIVATE_NO_UUID;
	else if (get_flags & DM_ACTIVE_UUID) {
		if (!strncmp(tmp_uuid, DM_UUID_PREFIX, DM_UUID_PREFIX_LEN))
			dmd->uuid = strdup(tmp_uuid + DM_UUID_PREFIX_LEN);
	}

	r = (dmi.open_count > 0);
out:
	if (dmt)
		dm_task_destroy(dmt);

	dm_exit_context();
	return r;
}

const char *dm_get_dir(void)
{
	return dm_dir();
}

int dm_is_dm_device(int major)
{
	return dm_is_dm_major((uint32_t)major);
}

int dm_is_dm_kernel_name(const char *name)
{
	return strncmp(name, "dm-", 3) ? 0 : 1;
}
