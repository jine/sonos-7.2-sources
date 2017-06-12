/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include "cryptsetup.h"
#include "sonos_common.h"

int sonos_action_luksFormat(const sonos_enfs *config)
{
	int r = -EINVAL, keysize;
	const char *header_device;
	char *key = NULL;
	char cipher[MAX_CIPHER_LEN], cipher_mode[MAX_CIPHER_LEN];
	struct crypt_device *cd = NULL;
	struct crypt_params_luks1 params = {
		.hash = DEFAULT_LUKS1_HASH,
		.data_alignment = 0,
		.data_device = NULL,
	};

	header_device = config->block_dev;

	memset(cipher, 0, MAX_CIPHER_LEN);
	memset(cipher_mode, 0, MAX_CIPHER_LEN);
	memcpy(cipher, "aes-cbc-essiv:sha256", 20);
	if ((r = crypt_init(&cd, header_device))) {
		syslog(LOG_ERR, "Cannot use %s as on-disk header.", header_device);
		goto out;
	}

	keysize = DEFAULT_LUKS1_KEYBITS / 8;

	r = crypt_format(cd, cipher, cipher_mode,
			 key, keysize, &params);
	if (r < 0) {
		syslog(LOG_ERR, "crypt_format error");
		r = -EINTR;
		goto out;
	}

out:
	crypt_free(cd);
	crypt_safe_free(key);
	return r;
}

int sonos_action_open_luks(const sonos_enfs *config)
{
	struct crypt_device *cd = NULL;
	const char *header_device, *activated_name;
	uint32_t flags = 0;
	int r;
	int opt_key_slot = CRYPT_ANY_SLOT;

	header_device = config->block_dev;

	activated_name = config->virtual_dev;

	if ((r = crypt_init(&cd, header_device)))
		goto out;

	if ((r = crypt_load(cd, CRYPT_LUKS1)))
		goto out;

	if ((crypt_get_data_offset(cd) < 8)) {
		syslog(LOG_ERR, "Reduced data offset is allowed only for detached LUKS header.");
		r = -EINVAL;
		goto out;
	}

	r = sonos_crypt_activate(cd, activated_name, opt_key_slot, flags);
out:
	crypt_free(cd);
	return r;
}

int action_close(const sonos_enfs *config)
{
	struct crypt_device *cd = NULL;
	int r;

	r = crypt_init_by_name(&cd, config->virtual_dev);
	if (r == 0)
		r = sonos_crypt_deactivate(cd, config->virtual_dev);

	crypt_free(cd);
	return r;
}
