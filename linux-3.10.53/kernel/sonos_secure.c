/*
 * Copyright (c) 2014, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <linux/module.h>
#include "mdp.h"
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/sonos_kernel.h>
#include <linux/slab.h>
#include <crypto/sonos_signature_common_linux.h>
#include <crypto/sonos_signature_verify_linux.h>
#include "sect_upgrade_header.h"

extern struct manufacturing_data_page sys_mdp;
extern struct manufacturing_data_page3 sys_mdp3;

extern int get_imx6_cpuid(uint8_t *, size_t);
extern int get_imx6_fuse(const char *, uint32_t *);

/* macros needed by the portable implementation of unlock/authz checking */
#define SU_CPU_TO_BE32		cpu_to_be32
#define SU_GETCPUID		get_imx6_cpuid
#define SU_GETFUSE		get_imx6_fuse
#define SU_PRINT		printk
#define SU_PLVL_DEBUG		KERN_DEBUG
#define SU_PLVL_INFO		KERN_INFO
#define SU_PLVL_ERR		KERN_ERR

/* get the portable implementation of unlock/authz checking */
#include "sonos_unlock.c.inc"

EXPORT_SYMBOL(sonosUnlockVerifyCpuSerialSig);
EXPORT_SYMBOL(sonosUnlockIsDeviceUnlocked);
EXPORT_SYMBOL(sonosUnlockIsAuthFeatureEnabled);

#undef SU_CPU_TO_BE32
#undef SU_GETCPUID
#undef SU_GETFUSE
#undef SU_PRINT
#undef SU_PLVL_DEBUG
#undef SU_PLVL_INFO
#undef SU_PLVL_ERR

/* Support the persistent unlock functionality... */
/* NOTE:  This function is NOT safe to run from interrupt context */
int is_mdp_authorized(__u32 mdp_authorization_flag)
{
	int result = 0;
	SonosSignature *sig = NULL;

	// This structure is pretty massive and could cause
	// us to run out of stack if we try to allocate it there,
	// so use kmalloc instead.  Optimization ideas are welcome.
	sig = kmalloc(sizeof(*sig), GFP_KERNEL);

	result = sig &&
		sonosUnlockIsAuthFeatureEnabled(mdp_authorization_flag,
						&sys_mdp,
						&sys_mdp3,
						sig,
						sonosHash,
						sonosRawVerify,
						sonosKeyLookup,
						"unit",
						"unlock",
						NULL);

	kfree(sig);
	return result;
}
EXPORT_SYMBOL(is_mdp_authorized);

/*
 *	Unlike the other authorization checks, the call to check
 *	sysrq happens at interrupt level, and the UnlockIsAuthFeature
 *	function is not interrupt safe.  So set a static at boot time,
 *	and only check that.
 */
static int sysrq_authorization = 0;
int is_sysrq_authorized(void)
{
	return sysrq_authorization;
}
EXPORT_SYMBOL(is_sysrq_authorized);

void check_sysrq_authorization(void)
{
	sysrq_authorization = is_mdp_authorized((__u32)MDP_AUTH_FLAG_SYSRQ_ENABLE);
}
EXPORT_SYMBOL(check_sysrq_authorization);

/* Provide access to the decrypt functionality before mounting rootfs... */
struct dm_ioctl;
extern int dm_dev_create(struct dm_ioctl *param, size_t param_size);
EXPORT_SYMBOL(dm_dev_create);
extern int dm_table_load(struct dm_ioctl *param, size_t param_size);
EXPORT_SYMBOL(dm_table_load);
extern int dm_dev_suspend(struct dm_ioctl *param, size_t param_size);
EXPORT_SYMBOL(dm_dev_suspend);


/* Handle the security keys needed... */

extern struct platform_device *sonos_sm_get_pdev(void);
extern int sonos_sm_init (struct platform_device *pdev);
extern void sonos_sm_exit (void);
extern int sonos_sm_encdec(struct platform_device *pdev, struct crypt_operation *op);

#define BLOB_OVERHEAD           48
#define BLOB_KEYLABEL_UBIFS     "ubifs"
#define BLOB_KEYLABEL_ROOTFS    "rootfs"

/* Returns 0 on success, non-zero on failure */
int sonos_key_encdec(int operation, int color, const void *in, int inlen,
		void *out, size_t *outlen, const char *keymod)
{
	int status;
	struct crypt_operation crypt_op;

	// Can't validate without a key...set up the unit/keyslot
	// in the CAAM driver...
	status = sonos_sm_init(sonos_sm_get_pdev());
	if ( status ) {
		printk(KERN_INFO "sm_init operation failed [%d]\n",status);
		goto endec_exit;
	}

	// Set up the encryption command for the CAAM driver...
	crypt_op.cmd = (operation == ENCRYPT) ? ENCRYPT : DECRYPT;
	crypt_op.color = color;
	crypt_op.input_buffer = (void*)in;
	crypt_op.input_length = inlen;
	crypt_op.output_buffer = out;
	crypt_op.output_length = outlen;
	crypt_op.original_length = (operation == ENCRYPT) ? inlen : inlen - BLOB_OVERHEAD;
	// Note that the usage of strncpy here is intentional.
	// If the length of source is less than n, strncpy() writes additional null bytes
	// to destination to ensure that a total of n bytes are written.
	// Destination here is a fixed-width field, used as a hash input,
	// so we need strncpy to guarantee that after source has been copied the rest of the bytes are zeroed out.
	strncpy(crypt_op.keymod, keymod, SECMEM_KEYMOD_LEN);

	status = sonos_sm_encdec(sonos_sm_get_pdev(), &crypt_op);
	if ( status ) {
		printk(KERN_INFO "sm en/decryption operation failed [%d]\n",status);
	}

	// Release the unit and keyslot...
	sonos_sm_exit();
endec_exit:
	return status;
}
EXPORT_SYMBOL(sonos_key_encdec);

/* Returns 0 on failure, non-zero on success */
static int sonos_decrypt_fskey(int color, const uint8_t* buf, size_t bufLen,
		uint8_t* out, size_t *pOutLen, const char* modifier)
{
	int result = 0;
	struct mdp_key_hdr hdr;
	const uint8_t* blob;
	size_t blobLen;

	if (color != USE_RED) {
		printk(KERN_INFO "bad key format %d\n", color);
		return 0;
	}

	if (bufLen < sizeof(hdr) + BLOB_OVERHEAD) {
		printk(KERN_INFO "bad input len %u\n", bufLen);
		return 0;
	}

	memcpy(&hdr, buf, sizeof(hdr));
	if (hdr.m_magic == MDP_KEY_HDR_MAGIC_CAAM_AES128_BLACK_KEY &&
		hdr.m_len <= bufLen - sizeof(hdr)) {

		// skip the black key and look for a red key
		buf += sizeof(hdr) + hdr.m_len;
		bufLen -= sizeof(hdr) + hdr.m_len;

		if (bufLen < sizeof(hdr) + BLOB_OVERHEAD) {
			printk(KERN_INFO "bad blob len %u\n", bufLen);
			return 0;
		}

		memcpy(&hdr, buf, sizeof(hdr));
		if (hdr.m_magic == MDP_KEY_HDR_MAGIC_CAAM_AES128_RED_KEY &&
			hdr.m_len <= bufLen - sizeof(hdr)) {
			blob = buf + sizeof(hdr);
			blobLen = hdr.m_len;

			result = sonos_key_encdec(DECRYPT, color, blob, blobLen,
					out, pOutLen, modifier) == 0;
		}
	}

	return result;
}

// MA! We really dont want to return a string here, we want to return a binary
// key. The preferred API would be something like:
//   int sonos_get_rootfs_key(int type, uint8_t *key, size_t* keyLen)
//
// Instead, we will have to convert our key into a hexstring.
int sonos_get_rootfs_key(int type, char *keystring)
{
	int retval = 0;
	uint8_t key[16];
	size_t keyLen = sizeof(key);

	// Calling function must allocate at least 65 bytes for the key string and
	// the terminator (although we currently only ever write 33 bytes at
	// most).

	// always null-terminate the output buffer no matter what
	keystring[0] = '\0';

	if ( type == SECT_UPGRADE_ROOTFS_FORMAT_PLAINTEXT ) {
		// No key, because we're not encrypting
		retval = 1;
	} else if ( type == SECT_UPGRADE_ROOTFS_FORMAT_FIXED_KEY ) {
		strlcpy(keystring, "a0d92447ee704af85fa924b59137aec2", 33);
		retval = 1;
	} else {
		int color = (type == SECT_UPGRADE_ROOTFS_FORMAT_BLACK_KEY) ? USE_BLACK : USE_RED;

		// Get the key from the CAAM
		if (sys_mdp3.mdp3_version >= MDP3_VERSION_DEV_CERT &&
				sonos_decrypt_fskey(color, sys_mdp3.mdp3_fskey2,
				sizeof(sys_mdp3.mdp3_fskey2),
				key, &keyLen, BLOB_KEYLABEL_ROOTFS) && keyLen == sizeof(key)) {
			int i;
			for (i = 0; i < sizeof(key); i++) {
				snprintf(&keystring[i * 2], 33 - (i * 2), "%02x", key[i]);
			}
			retval = 1;
		}
	}

	return retval;
}
EXPORT_SYMBOL(sonos_get_rootfs_key);

extern void ubifs_set_ubifs_key(const __u8*);
int sonos_set_ubifs_key(__u32 type)
{
	int	retval = 0;
	uint8_t key[16];
	size_t keyLen = sizeof(key);

	if ( type == UBIFS_CRYPT_TYPE_NONE ) {
		// Don't set a key - the fs will be accessed without encryption/decryption
		retval = 1;
	} else if ( type == UBIFS_CRYPT_TYPE_FIXED ) {
		static const __u8 key1[16] = { 0x55, 0x61, 0x4b, 0xde, 0x49, 0x5e, 0x0e, 0xd1, 0x50, 0x43, 0x77, 0x87, 0x94, 0x8e, 0x16, 0x3b };
		ubifs_set_ubifs_key(key1);
		retval = 1;
	} else {
		int color = (type == UBIFS_CRYPT_TYPE_BLACK_KEY) ? USE_BLACK : USE_RED;

		// Get the red key from the CAAM
		if (sys_mdp3.mdp3_version >= MDP3_VERSION_DEV_CERT &&
				sonos_decrypt_fskey(color, sys_mdp3.mdp3_fskey1,
				sizeof(sys_mdp3.mdp3_fskey1),
				key, &keyLen, BLOB_KEYLABEL_UBIFS) && keyLen == sizeof(key)) {
			ubifs_set_ubifs_key(key);
			retval = 1;
		}
	}

	return retval;
}
EXPORT_SYMBOL(sonos_set_ubifs_key);

/* API for root file system encryption */
struct mtd_info;
void get_rootfs_ubi_info(struct mtd_info * mtd, int *vol_id, int *ubi_num);
EXPORT_SYMBOL(get_rootfs_ubi_info);
struct ubi_volume_desc * rootfs_open(int vol_id, int ubi_num);
EXPORT_SYMBOL(rootfs_open);
int rootfs_update(struct ubi_volume_desc *desc, int64_t bytes);
EXPORT_SYMBOL(rootfs_update);
ssize_t rootfs_write(struct ubi_volume_desc *desc, const char *buf,
		size_t count, int flag);
EXPORT_SYMBOL(rootfs_write);
int rootfs_release(struct ubi_volume_desc *desc);
EXPORT_SYMBOL(rootfs_release);
