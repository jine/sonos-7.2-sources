/* sonos_mount.c.inc
 *
 * Copyright (c) 2015-2016, Sonos, Inc.
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Logically, and when built, this is part of init/do_mounts.c.  For sustaining,
 * and ease of kernel-kernel porting, it is maintained in a separate source code
 * module.  Initial implementation was for linux 3.10 and imx6 Solo-X SoC.
 *
 * There may be some Solo-X assumptions built-in to this module that will require
 * changes on other SoCs.
 */

#include <linux/inet.h>

#include <linux/crypto.h>
#include <linux/scatterlist.h>

#include <sonos/firmware_whitelist.h>
#include "sonos_digest_data.h"
#include <linux/device-mapper.h>
#include <linux/fcntl.h>
#include "sonos_lock.h"
#include <linux/dm-ioctl.h>
#include <crypto/sonos_signature_common_linux.h>
#include <crypto/sonos_signature_keys.h>
#include <crypto/sonos_signature_verify_linux.h>
#include "mdp.h"
#include "sonos_unlock.h"

// Couldnt find a header for this from drivers/char/fsl_otp.c
int get_imx6_cpuid(uint8_t* buf, size_t buf_len);

extern struct manufacturing_data_page sys_mdp;
extern struct manufacturing_data_page3 sys_mdp3;

// Need to allocate the structure to work with, but the decompressor will include
// a copy that the external tool will work on.  Need to NOT initialize the magic
// numbers, so as not to have two copies of them in the binary, and it doesn't matter
// what goes into the other fields - the decompressor will write the entire structure.
//
// NOTE:  The make system will find this address in vmlinux after the kernel builds
// and before the compression takes place.  It is looking for the string " rootfs_digest".
// If anything is added so that there are multiple copies of that string found in
// an objdump of the vmlinux file, the build will break.

rootfs_digest_t		rootfs_digest = {
	{ 0, 0 },				// digest_magic
	0,					// digest_value_length
	0,					// rootfs_length
	{"Placeholder for computed digest" },	// digest_value
} ;


static int get_verified_serial(uint8_t *serial, size_t serialLen)
{
	int result = 0;
	SonosSignature *sig = NULL;

	/*
	 * Some of these checks may be overkill within Linux (where
	 * sys_mdp/sys_mdp3 already got some sanity checking/scrubbing)
	 * but this code may also end up in u-boot.
	 */
	if (serialLen != 6 ||
	    sys_mdp.mdp_magic != MDP_MAGIC ||
	    sys_mdp.mdp_version < MDP_VERSION_AUTH_FLAGS ||
	    !(sys_mdp.mdp_pages_present & MDP_PAGE3_PRESENT) ||
	    sys_mdp3.mdp3_magic != MDP_MAGIC3 ||
	    sys_mdp3.mdp3_version < MDP3_VERSION_SECURE_BOOT) {
		printk(KERN_ERR "bad MDP in get_verified_serial: %lx %lx %lx %lx %lx\n",
		       sys_mdp.mdp_magic,
		       sys_mdp.mdp_version,
		       sys_mdp.mdp_pages_present,
		       sys_mdp3.mdp3_magic,
		       sys_mdp3.mdp3_version);
		return result;
	}

	sig = kmalloc(sizeof(*sig), GFP_KERNEL);
	if (sig && sonosUnlockVerifyCpuSerialSig(sys_mdp.mdp_serial, 6,
					   sys_mdp3.mdp3_cpuid_sig,
					   sizeof sys_mdp3.mdp3_cpuid_sig,
					   sig, sonosHash, sonosRawVerify,
					   sonosKeyLookup, "unit", NULL)) {
		printk(KERN_DEBUG "good cpuid/serial binding statement\n");
		memcpy(serial, sys_mdp.mdp_serial, serialLen);
		result = 1;
	}
	else {
		printk(KERN_ERR "bad cpuid/serial binding statement\n");
	}

	if (sig) {
		kfree(sig);
	}
	return result;
}

int validate_sonos_rootfs_digest(void);
extern int ath_nand_local_read(char *name, loff_t offset, size_t length, size_t *retleng, unsigned char *data);

/* implement some macros needed by the portable whitelist code */
#define SFW_GETCPUID get_imx6_cpuid
#define SFW_GETSERIAL get_verified_serial
#define SFW_PRINT printk
#define SFW_PLVL_INFO KERN_INFO
#define SFW_PLVL_EMERG KERN_EMERG
#define SFW_BE32_TO_CPU __be32_to_cpu

/* get the portable implementation of check_sonos_firmware_whitelist */
#include "sonos_fw_whitelist.c.inc"

#undef SFW_GETCPUID
#undef SFW_GETSERIAL
#undef SFW_PRINT
#undef SFW_PLVL_INFO
#undef SFW_PLVL_EMERG
#undef SFW_BE32_TO_CPU
#undef SFW_BE64_TO_CPU

/* Sonos customized mount_root - check for encryption, mount decrypted, constrain
 * the capabilities of other file systems.
 */

static int __init do_mount_root(char *name, char *fs, int flags, void *data)
{
	struct super_block *s;

	int err;
	int encrypted_root = 0;

	sonosInitKeyTable();

/*
 * Refuse to run at all if this kernel image is restricted to some
 * specific whitelist and we are not on it.
 *
 * No matter what other kernel customizations you might be tempted
 * to do, NEVER remove this code.
 *
 * This check is how we implement firmware that is specific to one unit.
 */
	if (!check_sonos_firmware_whitelist(&SONOS_FIRMWARE_WHITELIST.header)) {
		BUG();
	}

/*
 * Also validate that rootfs_digest value for this kernel matches the
 * value computed on the rootfs as it sits in flash.
 */
	encrypted_root = validate_sonos_rootfs_digest();

	if ( encrypted_root ) {
		printk(KERN_INFO "sys_mount - %s, %s, %s, x%x, x%x\n","/dev/mapper/crroot","/root",fs,(int)flags,(int)data);
		err = sys_mount("/dev/mapper/crroot", "/root", fs, flags, data);
	}
	else {
		printk(KERN_INFO "sys_mount - %s, %s, %s, x%x, x%x\n",name,"/root",fs,(int)flags,(int)data);
		err = sys_mount(name, "/root", fs, flags, data);
	}
	if (err)
		return err;

	sys_chdir("/root");
	s = current->fs->pwd.dentry->d_sb;
	ROOT_DEV = s->s_dev;
	printk(KERN_INFO
	       "VFS: Mounted root (%s filesystem)%s on device %u:%u.\n",
	       s->s_type->name,
	       s->s_flags & MS_RDONLY ?  " readonly" : "",
	       MAJOR(ROOT_DEV), MINOR(ROOT_DEV));
	/*
	 * The rootfs is the only filesystem that we want to allow
	 * execution out of or special nodes.  Set the noexec and
	 * nodev controls to constrained before mounting jffs or ramdisk...
	 */
	printk(KERN_INFO "Setting noexec and nodev modes to constrained...\n");
	if ( !is_mdp_authorized (MDP_AUTH_FLAG_EXEC_ENABLE) ) {
		printk(KERN_INFO "NOEXEC on all non-rootfs mounts\n");
		proc_noexec_set_constrained();
	}
	if ( !is_mdp_authorized (MDP_AUTH_FLAG_NODEV_CTRL) ) {
		printk(KERN_INFO "NODEV on all non-rootfs mounts\n");
		proc_nodev_set_constrained();
	}
	return 0;
}

#define SQUASHFS_LEN_OFFSET	40
#define CRAMFS_LEN_OFFSET	4

// rootfs validation - do not mount the rootfs if the calculated
// digest value does not match the value in kernel structure.
//
// return 1 if the rootfs is encrypted, 0 otherwise

int	calculate_digest(void *root, unsigned int rootlen, rootfs_digest_t *digest) ;
unsigned char	*handle_encrypted_rootfs( __s32*, char* );

int validate_sonos_rootfs_digest() {
	unsigned char   *proot;
	__s32   length;
	char	rootfs_file[16];
	rootfs_digest_t	 digest;
	__u32   *presult, *pkern;
	size_t  count = 4;
	size_t  retlen;
	int	 *pData;
	loff_t  offset = (loff_t) 0;
	int	encrypted = 0;

	strlcpy(rootfs_file, root_device_name, sizeof(rootfs_file));

	proot = handle_encrypted_rootfs(&length, rootfs_file);

	// Initialize the result to appease the compiler
	presult = (__u32 *)&digest;
	*presult = 0;

	// proot is a pointer that will be used for the rootfs data.  If it's NULL at this
	// point, that means that we do NOT have an encrypted rootfs.  Use the glubi driver
	// to load the plaintext rootfs into a buffer for the digest check.  If proot is
	// not NULL, that means that the digest_decrypt function found an encrypted rootfs,
	// mounted the decrypted partition, and loaded it into ram.

	if ( !proot ) {
		printk(KERN_INFO "Assuming rootfs is not encrypted\n");

		// Allocate a buffer to get the rootfs length
		length =  4;

		// get a buffer and copy the rootfs into it
		if( (proot = kmalloc(length, GFP_KERNEL)) == NULL) {
			// Could not get a buffer - fail
			printk(KERN_ERR "Could not get a buffer for the rootfs length check.\n");
			goto validation_fail;
		}

		// The first thing that needs to be done is to fetch the length of the rootfs section.
		// In order to do that correctly, we need to know which file system it is.
		pData = (int*)proot;

		// Read the first word to look for the filesystem magic number
		if ( ath_nand_local_read(rootfs_file, offset, count, &retlen, proot) ) {
			printk(KERN_ERR "Could not read %s\n", rootfs_file);
			goto validation_fail;
		}

		if (*pData == SQUASHFS_MAGIC) {
			printk(KERN_INFO "validating squashfs root\n");
			offset = (loff_t) SQUASHFS_LEN_OFFSET;
		} else {
			printk(KERN_INFO "validating cramfs root\n");
			offset = (loff_t) CRAMFS_LEN_OFFSET;
		}

		// Attempt to read the length information
		if ( ath_nand_local_read(rootfs_file, offset, count, &retlen, proot) ) {
			printk(KERN_ERR "Could not read rootfs length\n");
			goto validation_fail;
		}
		if ( retlen != count ) {
			printk(KERN_ERR "rootfs length read length failure [x%x != x%x]\n",retlen,count);
			goto validation_fail;
		}

		length = *pData;

		if (offset == SQUASHFS_LEN_OFFSET) {
			// round the length up to the next 4K boundary
			length = (((length-1)>>12)+1)<<12;
		}

		if ( length != be32_to_cpu(rootfs_digest.rootfs_length)  ) {
			// If the length doesn't match the digest structure, something is wrong, and there's no
			// point in continuing.  rootfs should be 5-6 MB and the length is poked
			// into the structure by the digest tool.
			printk(KERN_ERR "Rootfs length (x%x) invalid.\n",length);
			goto validation_fail;
		}

		// Ok, we can free that buffer and get a new one that's the right length.
		if ( proot ) kfree(proot);
		if ( (proot = kmalloc(length, GFP_KERNEL)) == NULL ) {
			// Could not get a buffer - fail
			printk(KERN_ERR "Could not get a buffer for the rootfs digest check.\n");
			goto validation_fail;
		}
		count = length;
		offset = (loff_t)0;

		// Read the rootfs into the allocated buffer...
		pData = (int*)proot;
		if ( ath_nand_local_read(rootfs_file, offset, count, &retlen, (unsigned char*)proot) ) {
			printk(KERN_ERR "Could not read rootfs\n");
			goto validation_fail;
		}
		if ( retlen != count ) {
			printk(KERN_ERR "rootfs read length failure [x%x != x%x]\n",retlen,count);
			goto validation_fail;
		}
	}
	else {
		encrypted = 1;
	}

	// Calculate the check value.
	printk(KERN_INFO "calculating the digest\n");
	if ( calculate_digest(proot, length, &digest) ) {
		printk(KERN_ERR "Error calculating rootfs digest\n");
		goto validation_fail;
	}

	// free the buffer, clean up
	kfree(proot);
	proot = NULL;

	// Setup to do the comparison...
	pkern = (__u32 *)&rootfs_digest.digest_value;
	presult = (__u32 *)&digest.digest_value;
	// ... including its length
	if ( be32_to_cpu(rootfs_digest.digest_value_length) != SONOS_DIGEST_LENGTH ) {
		printk(KERN_ERR "Incorrect rootfs digest length\n");
		goto validation_fail;
	}

	// Everything looks sane - does the digest match?
	if ( !memcmp(pkern, presult, be32_to_cpu(rootfs_digest.digest_value_length) ) ) {
		printk(KERN_INFO "checking %d byte digest (using %s)\n  calculated digest x%08x, kernel contained x%08x  - good rootfs\n",
				(int)be32_to_cpu(rootfs_digest.digest_value_length), ROOTFS_DIGEST_ALGORITHM, (int)*presult, (int)*pkern);
		goto validation_success;
	}
	printk(KERN_ERR "calculated digest x%08x, kernel contained x%08x - MISMATCH ON rootfs DIGEST!!\n",(int)*presult, (int)*pkern);

validation_fail:
	printk(KERN_ERR "rootfs digest validation failed.  Booting halted.\n");
	if ( proot ) kfree(proot);
	panic("rootfs digest validation failed.\n");

validation_success:
	return encrypted;

}

int	calculate_digest(void *root, unsigned int rootlen, rootfs_digest_t *digest) {
	struct scatterlist sg ;
	unsigned char *phash;
	struct  crypto_hash *tfm;
	struct  hash_desc desc;
	int     retval = 0;

	tfm = crypto_alloc_hash(ROOTFS_DIGEST_ALGORITHM, 0, CRYPTO_ALG_ASYNC);
	if(IS_ERR(tfm)) {
		printk(KERN_ERR "could not allocate crypto hash\n");
		goto calculate_exit;
	}

	sg_init_table(&sg, 1);
	sg_set_buf(&sg,root,rootlen);

	desc.tfm = tfm;
	desc.flags = 0;

	phash = (unsigned char*)&digest->digest_value;

	if (crypto_hash_digest(&desc, &sg, rootlen, phash)) {
		printk(KERN_ERR "crypto_hash_digest failed\n");
		return -1;
	}

calculate_exit:
	crypto_free_hash(tfm);
	return retval;
}


#if defined(SONOS_ARCH_ENCORE)
#define ROOTFS_MAX	(73*1024*1024)
#else
#define ROOTFS_MAX	(95*1024*1024)
#endif

extern int dm_dev_create(struct dm_ioctl *param, size_t param_size);
extern int dm_table_load(struct dm_ioctl *param, size_t param_size);
extern int dm_dev_suspend(struct dm_ioctl *param, size_t param_size);
extern int sonos_get_rootfs_key(int, char *);

/*	This function performs the ioctls necessary to get an encrypted
 *	partition decrypted by the drivers/md driver.  The ioctl are based
 *	on the functionality implemented in the crytpsetup application, but
 *	we do not want to run that application, or any userspace application,
 *	before mounting rootfs.  It is assumed that we're running with an
 *	initramfs that has mount points for the relevant block devices.
 *
 *	This function uses the glubi driver to perform an initial read of the
 *	block device hosting the rootfs, and determines whether it has
 *	been encrypted or not.  (Upgrades performed by uboot will not be.)
 *	If so, it decrypts the rootfs, and loads the decrypted data into a
 *	kernel buffer.  If not, it just returns NULL and the extant code
 *	deals with the unencrypted rootfs.
 *
 *	Return:
 *		pointer to buffer containing rootfs decrypted data or NULL
 *			if something failed
 */

unsigned char* handle_encrypted_rootfs( __s32 *buffer_length, char *rootfs_device )
{
	unsigned char   *proot = NULL;
	char	rootfs_file[24];
	char	encryption_check[8];
	__s32	length = 8;
	size_t  retlen;
	loff_t  offset = (loff_t)0;
	struct	dm_ioctl *dmi = NULL;
	struct  dm_target_spec	*dms;
	int	control;
	char	*parms;
	unsigned char parms_key[65];
	int	rfd = -1;
	int	bytes_read ;
	int	buffer[64];
	int	bytes_to_read;
	int	blocks = 0;
	int	key_type;
	int	dm_result;

	// Ok, let's start by reading the rootfs file and determining
	// whether it is a LUKS container or not...
	strlcpy(rootfs_file, rootfs_device, sizeof rootfs_file);

	// The first thing that needs to be done is to fetch the beginning of the rootfs
	// partition to determine whether or not it's encrypted.  If it is, it's a
	// LUKS container, and the first word contains "LUKS".

	// Read the first two words out of the device.
	if ( ath_nand_local_read(rootfs_file, offset, length, &retlen, encryption_check) ) {
		printk(KERN_ERR "Could not read rootfs length\n");
		goto dmcrypt_failed;
	}
	if ( retlen != length ) {
		printk(KERN_ERR "rootfs length read length failure [x%x != x%x]\n", retlen, length);
		goto dmcrypt_failed;
	}

	if ( !strncmp(encryption_check,"LUKS",4) ) {
		printk(KERN_INFO "LUKS container - encrypted rootfs\n");
	}
	else {
		printk(KERN_INFO "rootfs_device %s is not encrypted\n", rootfs_device);
		goto end;
	}

	// Eventually, we'll need a number of blocks to create the device - find
	// out how big a device we need
	{
		int	tfd;
		char	rootfs_devpoint[24];
		int	device_bytes;

		snprintf(rootfs_devpoint, sizeof(rootfs_devpoint), "/dev/%s",rootfs_device);

		if( (tfd = sys_open(rootfs_devpoint,O_RDONLY,0777)) < 0 )
			printk(KERN_INFO "Could not open %s\n",rootfs_devpoint);
		else {
			device_bytes = sys_lseek(tfd,0,SEEK_END);
			// blocks on device, excluding the 2MB LUKS header
			blocks = (device_bytes / 512) - 4096;
			printk(KERN_INFO "end of %s is %d bytes, %d blocks\n",rootfs_devpoint, device_bytes,blocks);
			sys_close(tfd);
		}
	}

	// Read the kernel section header to find out what kind of key was used.
	// NOTE:  There are two fundamental assumptions here, and those are that
	// the kernel partition immediately precedes the rootfs partition, and that
	// the kernel partition contains the section header as it's first content.
	// These are true by definition for initial SONOS_SECBOOT products, and
	// this code depends upon those assumptions.
	{
#include "sect_upgrade_header.h"

		SECT_UPGRADE_HEADER	sect_header;
		int	kern_block_number;
		char	kern_file[24];

		// Whichever mtdblock device has the rootfs, the one before it has the kernel...
		sscanf(rootfs_file,"mtdblock%d",&kern_block_number);
		kern_block_number--;
		snprintf(kern_file, sizeof(kern_file),"mtdblock%d",kern_block_number);
		printk(KERN_INFO "try to read the kernel section header at %s\n",kern_file);

		if ( ath_nand_local_read(kern_file, offset, sizeof(sect_header), &retlen, (char*)&sect_header) ) {
			printk(KERN_ERR "Could not read kernel section header\n");
			goto dmcrypt_failed;
		}
		key_type = sect_header.rootfsFormat;
		printk(KERN_INFO "rootfs format=%d\n",key_type);
	}

	// Get the decryption key.
	if ( sonos_get_rootfs_key(key_type, parms_key) != 1 ) {
		goto dmcrypt_failed;
	}

	// Issue a series of ioctls to the device-mapper driver in order to create
	// a mounted and decrypted device.

	// Allocate the dmi structure and open the control...

	if ( (control = sys_open("/dev/mapper/control",O_RDWR|O_CREAT,0777)) < 0 )
		printk(KERN_INFO "could not open /dev/mapper/control - %d\n", control);
	else
		printk(KERN_INFO "Opened /dev/mapper/control at descriptor %d\n",control);

	dmi = kmalloc(16384, GFP_KERNEL);

	// Several of the ioctl fields will not change - fill them in once.  They're either
	// the same for each command (version, data_size) or aren't used in any of the ioctl
	// commands that we're using (data).
	dmi->version[0] =      4;
	dmi->version[1] =      24;
	dmi->version[2] =      0;
	dmi->data_size =       312;
	dmi->data_start =      312;
	dmi->open_count =      0;
	dmi->padding =         0;
	{
		__u32	dmid;
		for(dmid=0;dmid < 7;dmid++)
			dmi->data[dmid] = 0;
	}

	// First we need to create the device...
	dmi->target_count =    0;
	dmi->flags =           4;
	dmi->event_nr =        0;
	dmi->dev =             0;
	strlcpy(dmi->name, "crroot", sizeof(dmi->name));
	dmi->uuid[0] = '\0';
	dm_result = dm_dev_create(dmi, 305);
	printk(KERN_INFO "dev_create returned %d\n", dm_result);

	// ...and then perform the table_load operation to create the cryptographic construct
	dmi->target_count =    1;
	dmi->flags =           5;
	dmi->event_nr =        0;
	dmi->dev =             0;
	strlcpy(dmi->name, "crroot", sizeof(dmi->name));
	dmi->uuid[0] = '\0';

	dms = (struct dm_target_spec *)((u8 *)dmi + dmi->data_start);
	dms->sector_start =	0;
	dms->length =		blocks;		// Size (in sectors) of virtual device created
						// to hold the encrypted rootfs
	dms->status =		0x0;
	dms->next =		0x98;
	strlcpy(dms->target_type, "crypt", sizeof(dms->target_type));
	parms = (char *)(dms + 1);
	sprintf(parms, "aes-cbc-essiv:sha256 %64s 0 /dev/%s 4096", parms_key, rootfs_file);
	dm_result = dm_table_load(dmi, 16384);
	printk(KERN_INFO "table_load returned %d\n", dm_result);

	// Finally, issue DM_DEV_SUSPEND
	dmi->target_count =    1;
	dmi->flags =           4;
	dmi->event_nr =        0x400000;
	dmi->dev =             0;
	strlcpy(dmi->name, "crroot", sizeof(dmi->name));
	dmi->uuid[0] = '\0';
	dm_result = dm_dev_suspend(dmi, 305);
	printk(KERN_INFO "dev_suspend returned %d\n", dm_result);

	// All done with the ioctls - free dmi
	kfree(dmi);
	dmi = NULL;

	// The decrypted device should now be mounted at /dev/mapper/crroot - load
	// the contents into a buffer and pass it back for the digest validation.
	rfd = sys_open("/dev/mapper/crroot",O_RDONLY,0777);

	if ( rfd < 0 ) {
		goto dmcrypt_failed;
	}

	sys_read(rfd, buffer, 64);

	// This decrypted rootfs can either be a cram file system, in which case the
	// length is at word 1, or a squash file system, in which case the length is
	// the next 4K multiple greater than word 10.
	if (buffer[0] == SQUASHFS_MAGIC) {
		// SQUASHFS root filesystem
		int calc_len = buffer[SQUASHFS_LEN_OFFSET/sizeof(int)];
		calc_len = (((calc_len-1)>>12)+1)<<12;
		// ... and that's the length
		printk(KERN_ERR "squashfs_bytes 0x%x - rootfs size 0x%x (%d) bytes\n", \
			buffer[SQUASHFS_LEN_OFFSET/sizeof(int)], calc_len, calc_len);
		*buffer_length = calc_len;
	} else {
		// CRAMFS root filesystem
		*buffer_length = buffer[CRAMFS_LEN_OFFSET/sizeof(int)];
	}

	// Sanity check...
	if ( *buffer_length > ROOTFS_MAX || *buffer_length < 8 ) {
		goto dmcrypt_failed;
	}

	printk(KERN_INFO "reading %d bytes from decrypted device\n",*buffer_length);
	proot = kmalloc(*buffer_length, GFP_KERNEL);
	sys_lseek(rfd,0,SEEK_SET);
	bytes_to_read = *buffer_length;
	bytes_read = sys_read(rfd, proot, bytes_to_read);

	// Everything appears to have worked - return the buffer to the validation code
	sys_close(rfd);
	return proot;

dmcrypt_failed:
	printk(KERN_INFO "dmcrypt failed\n");
end:
	if (rfd >= 0) sys_close(rfd);
	if (proot) kfree(proot);
	if (dmi) kfree(dmi);

	return NULL;
}
