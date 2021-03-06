/*
 * libcryptsetup - cryptsetup library
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

/**
 * @file libcryptsetup.h
 * @brief Public cryptsetup API
 *
 * For more verbose examples of LUKS related use cases,
 * please read @ref index "examples".
 */

#ifndef _LIBCRYPTSETUP_H
#define _LIBCRYPTSETUP_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

struct crypt_device; /* crypt device handle */

/**
 * Initialize crypt device handle and check if provided device exists.
 *
 * @param cd Returns pointer to crypt device handle
 * @param device Path to the backing device.
 * 	  If @e device is not a block device but a path to some file,
 * 	  the function will try to create a loopdevice and attach
 * 	  the file to the loopdevice with AUTOCLEAR flag set.
 * 	  If @e device is @e NULL function it will initialize dm backend only.
 *
 * @return @e 0 on success or negative errno value otherwise.
 *
 * @note Note that logging is not initialized here, possible messages uses
 * 	 default log function.
 */
int crypt_init(struct crypt_device **cd, const char *device);

/**
 * Initialize crypt device handle from provided active device name,
 * and, optionally, from separate metadata (header) device
 * and check if provided device exists.
 *
 * @return @e 0 on success or negative errno value otherwise.
 *
 * @param cd returns crypt device handle for active device
 * @param name name of active crypt device
 * @param header_device optional device containing on-disk header
 * 	  (@e NULL if it the same as underlying device on there is no on-disk header)
 *
 * @post In case @e device points to active LUKS device but header load fails,
 * context device type is set to @e NULL and @e 0 is returned as if it were successful.
 * Context with @e NULL device type can only be deactivated by sonos_crypt_deactivate
 *
 * @note @link crypt_init_by_name @endlink is equivalent to calling
 * 	 crypt_init_by_name_and_header(cd, name, NULL);
 */
int crypt_init_by_name_and_header(struct crypt_device **cd,
				  const char *name,
				  const char *header_device);

/**
 * This is equivalent to call
 * @ref crypt_init_by_name_and_header "crypt_init_by_name_and_header(cd, name, NULL)"
 *
 * @sa crypt_init_by_name_and_header
 */
int crypt_init_by_name(struct crypt_device **cd, const char *name);

/**
 * @defgroup loglevel Cryptsetup logging
 *
 * Set of functions and defines used in cryptsetup for
 * logging purposes
 *
 */

/**
 * @addtogroup loglevel
 * @{
 */

/** normal log level */
#define CRYPT_LOG_NORMAL 0
/** error log level */
#define CRYPT_LOG_ERROR  1
/** verbose log level */
#define CRYPT_LOG_VERBOSE  2
/** debug log level - always on stdout */
#define CRYPT_LOG_DEBUG -1

/**
 * Set log function.
 *
 * @param cd crypt device handle (can be @e NULL to set default log function)
 * @param log user defined log function reference
 * @param usrptr provided identification in callback
 * @param level log level below (debug messages can uses other levels)
 * @param msg log message
 */
void crypt_set_log_callback(struct crypt_device *cd,
	void (*log)(int level, const char *msg, void *usrptr),
	void *usrptr);

/**
 * Defines log function or use the default one otherwise.
 *
 * @see crypt_set_log_callback
 *
 * @param cd crypt device handle
 * @param level log level
 * @param msg log message
 */
void crypt_log(struct crypt_device *cd, int level, const char *msg);
/** @} */

/**
 * Set confirmation callback (yes/no)
 *
 * If code need confirmation (like resetting uuid or restoring LUKS header from file)
 * this function is called. If not defined, everything is confirmed.
 *
 * Callback function @e confirm should return @e 0 if operation is declined,
 * other values mean accepted.
 *
 * @param cd crypt device handle
 * @param confirm user defined confirm callback reference
 * @param usrptr provided identification in callback
 * @param msg Message for user to confirm
 *
 * @note Current version of cryptsetup API requires confirmation only when UUID is being changed
 */
void crypt_set_confirm_callback(struct crypt_device *cd,
	int (*confirm)(const char *msg, void *usrptr),
	void *usrptr);

/**
 * Set password query callback.
 *
 * If code need @e _interactive_ query for password, this callback is called.
 * If not defined, compiled-in default is called (uses terminal input).
 *
 * Callback should return length of password in buffer
 * or negative errno value in case of error.
 *
 * @param cd crypt device handle
 * @param password user defined password callback reference
 * @param usrptr provided identification in callback
 * @param msg Message for user
 * @param buf buffer for password
 * @param length size of buffer
 *
 * @note Note that if this function is defined, verify option is ignored
 *   (caller which provided callback is responsible for password verification)
 * @note Only zero terminated passwords can be entered this way, for complex
 *   use API functions directly.
 * @note Maximal length of password is limited to @e length @e - @e 1 (minimal 511 chars)
 *
 * @see Callback function is used in these call provided, that certain conditions are met:
 * @li crypt_keyslot_add_by_passphrase
 * @li sonos_crypt_activate
 * @li crypt_resume_by_passphrase
 * @li crypt_resume_by_keyfile
 * @li crypt_keyslot_add_by_keyfile
 * @li crypt_keyslot_add_by_volume_key
 *
 */
void crypt_set_password_callback(struct crypt_device *cd,
	int (*password)(const char *msg, char *buf, size_t length, void *usrptr),
	void *usrptr);

/**
 * Set data device
 * For LUKS it is encrypted data device when LUKS header is separated.
 * For VERITY it is data device when hash device is separated.
 *
 * @param cd crypt device handle
 * @param device path to device
 *
 */
int crypt_set_data_device(struct crypt_device *cd, const char *device);

/**
 * @defgroup rng Cryptsetup RNG
 *
 * @addtogroup rng
 * @{
 *
 */

/** CRYPT_RNG_URANDOM - use /dev/urandom */
#define CRYPT_RNG_URANDOM 0
/** CRYPT_RNG_RANDOM  - use /dev/random (waits if no entropy in system) */
#define CRYPT_RNG_RANDOM  1

/**
 * Set which RNG (random number generator) is used for generating long term key
 *
 * @param cd crypt device handle
 * @param rng_type kernel random number generator to use
 *
 */

/** plain crypt device, no on-disk header */
#define CRYPT_PLAIN "PLAIN"
/** LUKS version 1 header on-disk */
#define CRYPT_LUKS1 "LUKS1"
/** loop-AES compatibility mode */
#define CRYPT_LOOPAES "LOOPAES"
/** dm-verity mode */
#define CRYPT_VERITY "VERITY"
/** TCRYPT (TrueCrypt-compatible) mode */
#define CRYPT_TCRYPT "TCRYPT"

/**
 * Get device type
 *
 * @param cd crypt device handle
 * @return string according to device type or @e NULL if not known.
 */
const char *crypt_get_type(struct crypt_device *cd);

/**
 *
 * Structure used as parameter for PLAIN device type
 *
 * @see crypt_format
 */
struct crypt_params_plain {
	const char *hash; /**< password hash function */
	uint64_t offset; /**< offset in sectors */
	uint64_t skip; /**< IV offset / initialization sector */
	uint64_t size; /**< size of mapped device or @e 0 for autodetection */
};

/**
 * Structure used as parameter for LUKS device type
 *
 * @see crypt_format, crypt_load
 *
 * @note during crypt_format @e data_device attribute determines
 * 	 if the LUKS header is separated from encrypted payload device
 *
 */
struct crypt_params_luks1 {
	const char *hash; /**< hash used in LUKS header */
	size_t data_alignment; /**< data alignment in sectors, data offset is multiple of this */
	const char *data_device; /**< detached encrypted data device or @e NULL */
};

/**
 *
 * Structure used as parameter for loop-AES device type
 *
 * @see crypt_format
 *
 */
struct crypt_params_loopaes {
	const char *hash; /**< key hash function */
	uint64_t offset;  /**< offset in sectors */
	uint64_t skip;    /**< IV offset / initialization sector */
};

/**
 *
 * Structure used as parameter for dm-verity device type
 *
 * @see crypt_format, crypt_load
 *
 */
struct crypt_params_verity {
	const char *hash_name;     /**< hash function */
	const char *data_device;   /**< data_device (CRYPT_VERITY_CREATE_HASH) */
	const char *hash_device;   /**< hash_device (output only) */
	const char *salt;          /**< salt */
	uint32_t salt_size;        /**< salt size (in bytes) */
	uint32_t hash_type;        /**< in-kernel hashing type */
	uint32_t data_block_size;  /**< data block size (in bytes) */
	uint32_t hash_block_size;  /**< hash block size (in bytes) */
	uint64_t data_size;        /**< data area size (in data blocks) */
	uint64_t hash_area_offset; /**< hash/header offset (in bytes) */
	uint32_t flags;            /**< CRYPT_VERITY* flags */
};

/** No on-disk header (only hashes) */
#define CRYPT_VERITY_NO_HEADER   (1 << 0)
/** Verity hash in userspace before activation */
#define CRYPT_VERITY_CHECK_HASH  (1 << 1)
/** Create hash - format hash device */
#define CRYPT_VERITY_CREATE_HASH (1 << 2)

/**
 *
 * Structure used as parameter for TCRYPT device type
 *
 * @see crypt_load
 *
 */
struct crypt_params_tcrypt {
	const char *passphrase;    /**< passphrase to unlock header (input only) */
	size_t passphrase_size;    /**< passphrase size (input only, max length is 64) */
	const char **keyfiles;     /**< keyfile paths to unlock header (input only) */
	unsigned int keyfiles_count;/**< keyfiles count (input only) */
	const char *hash_name;     /**< hash function for PBKDF */
	const char *cipher;        /**< cipher chain c1[-c2[-c3]] */
	const char *mode;          /**< cipher block mode */
	size_t key_size;           /**< key size in bytes (the whole chain) */
	uint32_t flags;            /**< CRYPT_TCRYPT* flags */
};

/** Include legacy modes ehn scannig for header*/
#define CRYPT_TCRYPT_LEGACY_MODES    (1 << 0)
/** Try to load hidden header (describing hidden device) */
#define CRYPT_TCRYPT_HIDDEN_HEADER   (1 << 1)
/** Try to load backup header */
#define CRYPT_TCRYPT_BACKUP_HEADER   (1 << 2)
/** Device contains encrypted system (with boot loader) */
#define CRYPT_TCRYPT_SYSTEM_HEADER   (1 << 3)

/** @} */

/**
 * Create (format) new crypt device (and possible header on-disk) but not activates it.
 *
 * @pre @e cd contains initialized and not formatted device context (device type must @b not be set)
 *
 * @param cd crypt device handle
 * @param type type of device (optional params struct must be of this type)
 * @param cipher (e.g. "aes")
 * @param cipher_mode including IV specification (e.g. "xts-plain")
 * @param uuid requested UUID or @e NULL if it should be generated
 * @param volume_key pre-generated volume key or @e NULL if it should be generated (only for LUKS)
 * @param volume_key_size size of volume key in bytes.
 * @param params crypt type specific parameters (see @link crypt_type @endlink)
 *
 * @returns @e 0 on success or negative errno value otherwise.
 *
 * @note Note that crypt_format does not enable any keyslot (in case of work with LUKS device),
 * 	but it stores volume key internally and subsequent crypt_keyslot_add_* calls can be used.
 * @note For VERITY @link crypt_type @endlink, only uuid parameter is used, others paramaters
 * 	are ignored and verity specific attributes are set through mandatory params option.
 */
int crypt_format(struct crypt_device *cd,
	const char *cipher,
	const char *cipher_mode,
	const char *volume_key,
	size_t volume_key_size,
	void *params);

/**
 * Load crypt device parameters from on-disk header
 *
 * @param cd crypt device handle
 * @param requested_type @link crypt_type @endlink or @e NULL for all known
 * @param params crypt type specific parameters (see @link crypt_type @endlink)
 *
 * @returns 0 on success or negative errno value otherwise.
 *
 * @post In case LUKS header is read successfully but payload device is too small
 * error is returned and device type in context is set to @e NULL
 *
 * @note Note that in current version load works only for LUKS and VERITY device type.
 *
 */
int crypt_load(struct crypt_device *cd,
	       const char *requested_type);

/**
 * Releases crypt device context and used memory.
 *
 * @param cd crypt device handle
 */
void crypt_free(struct crypt_device *cd);

/**
 * @defgroup keyslot Cryptsetup LUKS keyslots
 * @addtogroup keyslot
 * @{
 *
 */

/** iterate through all keyslots and find first one that fits */
#define CRYPT_ANY_SLOT -1

/**
 * @defgroup aflags Device runtime attributes
 *
 * Activation flags
 *
 * @addtogroup aflags
 * @{
 *
 */
/** device is read only */
#define CRYPT_ACTIVATE_READONLY (1 << 0)
/** only reported for device without uuid */
#define CRYPT_ACTIVATE_NO_UUID  (1 << 1)
/** activate even if cannot grant exclusive access (DANGEROUS) */
#define CRYPT_ACTIVATE_SHARED   (1 << 2)
/** enable discards aka TRIM */
#define CRYPT_ACTIVATE_ALLOW_DISCARDS (1 << 3)
/** skip global udev rules in activation ("private device"), input only */
#define CRYPT_ACTIVATE_PRIVATE (1 << 4)
/** corruption detected (verity), output only */
#define CRYPT_ACTIVATE_CORRUPTED (1 << 5)

/**
 * Active device runtime attributes
 */
struct crypt_active_device {
	uint64_t offset; /**< offset in sectors */
	uint64_t iv_offset; /**< IV initialization sector */
	uint64_t size; /**< active device size */
	uint32_t flags; /**< activation flags */
};

int sonos_crypt_activate(struct crypt_device *cd,
	const char *name,
	int keyslot,
	uint32_t flags);

int sonos_crypt_deactivate(struct crypt_device *cd, const char *name);

/**
 * @defgroup devstat Crypt and Verity device status
 * @addtogroup devstat
 * @{
 */

/**
 * Device status
 */
typedef enum {
	CRYPT_INVALID, /**< device mapping is invalid in this context */
	CRYPT_INACTIVE, /**< no such mapped device */
	CRYPT_ACTIVE, /**< device is active */
	CRYPT_BUSY /**< device is active and has open count > 0 */
} crypt_status_info;

/**
 * Get status info about device name
 *
 * @param cd crypt device handle, can be @e NULL
 * @param name crypt device name
 *
 * @return value defined by crypt_status_info.
 *
 */
crypt_status_info crypt_status(struct crypt_device *cd, const char *name);

/**
 * Get cipher used in device
 *
 * @param cd crypt device handle
 *
 * @return used cipher, e.g. "aes" or @e NULL otherwise
 *
 */
const char *crypt_get_cipher(struct crypt_device *cd);

/**
 * Get cipher mode used in device
 *
 * @param cd crypt device handle
 *
 * @return used cipher mode e.g. "xts-plain" or @e otherwise
 *
 */
const char *crypt_get_cipher_mode(struct crypt_device *cd);

/**
 * Get device UUID
 *
 * @param cd crypt device handle
 *
 * @return device UUID or @e NULL if not set
 *
 */
const char *crypt_get_uuid(struct crypt_device *cd);

/**
 * Get path to underlaying device
 *
 * @param cd crypt device handle
 *
 * @return path to underlaying device name
 *
 */
const char *crypt_get_device_name(struct crypt_device *cd);

/**
 * Get device offset in sectors where real data starts on underlying device)
 *
 * @param cd crypt device handle
 *
 * @return device offset in sectors
 *
 */
uint64_t crypt_get_data_offset(struct crypt_device *cd);

/**
 * Get IV offset in sectors (skip)
 *
 * @param cd crypt device handle
 *
 * @return IV offset
 *
 */
uint64_t crypt_get_iv_offset(struct crypt_device *cd);

/**
 * Get size (in bytes) of volume key for crypt device
 *
 * @param cd crypt device handle
 *
 * @return volume key size
 *
 */
int crypt_get_volume_key_size(struct crypt_device *cd);

/**
 * Get device parameters for VERITY device
 *
 * @param cd crypt device handle
 * @param vp verity device info
 *
 * @e 0 on success or negative errno value otherwise.
 *
 */
int crypt_get_verity_info(struct crypt_device *cd,
	struct crypt_params_verity *vp);
/** @} */

/**
 * @defgroup benchmark Benchmarking
 *
 * Benchmarking of algorithms
 *
 * @addtogroup benchmark
 * @{
 *
 */

/**
 * Informational benchmark for ciphers
 *
 * @param cd crypt device handle
 * @param cipher (e.g. "aes")
 * @param cipher_mode (e.g. "xts"), IV generator is ignored
 * @param volume_key_size size of volume key in bytes
 * @param iv_size size of IV in bytes
 * @param buffer_size size of encryption buffer in bytes used in test
 * @param encryption_mbs measured encryption speed in MiB/s
 * @param decryption_mbs measured decryption speed in MiB/s
 *
 * @return @e 0 on success or negative errno value otherwise.
 */

/**
 * Informational benchmark for KDF
 *
 * @param cd crypt device handle
 * @param kdf Key derivation function (e.g. "pbkdf2")
 * @param hash Hash algorithm used in KDF (e.g. "sha256")
 * @param password password for benchmark
 * @param password_size size of password
 * @param salt salt for benchmark
 * @param salt_size size of salt
 * @param iterations_sec returns measured KDF iterations per second
 *
 * @return @e 0 on success or negative errno value otherwise.
 */

/**
 * @addtogroup keyslot
 * @{
 *
 */

/**
 * Crypt keyslot info
 */
typedef enum {
	CRYPT_SLOT_INVALID, /**< invalid keyslot */
	CRYPT_SLOT_INACTIVE, /**< keyslot is inactive (free) */
	CRYPT_SLOT_ACTIVE, /**< keyslot is active (used) */
	CRYPT_SLOT_ACTIVE_LAST /**< keylost is active (used)
				*   and last used at the same time */
} crypt_keyslot_info;

/**
 * Get information about particular key slot
 *
 *
 * @param cd crypt device handle
 * @param keyslot requested keyslot to check or CRYPT_ANY_SLOT
 *
 * @return value defined by crypt_keyslot_info
 *
 */
/** @} */

/**
 * Get number of keyslots supported for device type.
 *
 * @param type crypt device type
 *
 * @return slot count or negative errno otherwise if device
 * doesn't not support keyslots.
 */
int crypt_keyslot_max(const char *type);

/**
 * @defgroup dbg Library debug level
 *
 * Set library debug level
 *
 * @addtogroup dbg
 * @{
 */

/** Debug all */
#define CRYPT_DEBUG_ALL  -1
/** Debug none */
#define CRYPT_DEBUG_NONE  0
#ifdef __cplusplus
}
#endif
#endif /* _LIBCRYPTSETUP_H */
