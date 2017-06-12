/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * linux/fs/proc/sonos_lock.c
 *
 * /proc entry support to allow control over unlock authorization
 * functionality on secured Sonos products.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include "mdp.h"
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "sonos_rollback.h"
// sonos_lock.h needs to know whether this is
// is the source file...
#define SONOS_LOCK_C
#include "sonos_lock.h"
#ifdef CONFIG_SONOS_SECBOOT
#include <sonos/firmware_whitelist.h>
#endif

// Statics for use in maintaining the fallback state
static int saved_state;
static int state_valid = 0;



/*	/proc/sonos-lock/noexec_ctrl
 *		How should we treat MNT_NOEXEC when mounting or operating?
 *		rw
 *
 *	On secure devices, we are going to support three modes:
 *		Normal	-	noexec handling is standard linux handling
 *		Constrained -	noexec handling is respected on mounted
 *				filesystems, and no additional mounts will
 *				occur without noexec set
 *		Unconstrained -	noexec is ignored on persistently unlocked
 *				devices.
 *
 */

static int noexec_mode = NORMAL_MODE;

int sonos_forbid_execution(void)
{
	return ( (noexec_mode == CONSTRAINED_MODE) ? 1 : 0 );
}
EXPORT_SYMBOL(sonos_forbid_execution);
int sonos_allow_execution(void)
{
	return ( (noexec_mode == UNCONSTRAINED_MODE) ? 1 : 0 );
}
EXPORT_SYMBOL(sonos_allow_execution);
void proc_noexec_set_constrained(void)
{
	noexec_mode = CONSTRAINED_MODE;
}
EXPORT_SYMBOL(proc_noexec_set_constrained);

static int noexec_ctrl_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", noexec_mode );
	return 0;
}

static int noexec_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, noexec_ctrl_proc_show, NULL);
}

//	Allow the device to set or clear the noexec_mode flag.  It can be set on any unit,
//	but only cleared if persistently unlocked...
static int noexec_ctrl_write_data(struct file *file, const char __user * buf, size_t length, loff_t * offset)
{
	char	buffer[64];

        memset(buffer, 0, sizeof(buffer));
        if (length > sizeof(buffer) - 1)
                length = sizeof(buffer) - 1;
        if (copy_from_user(buffer, buf, length))
                return -EFAULT;
	// We can go from less constrained to more constrained without unlocking,
	// but not from more constrained to less.
	// MORE -- constrained <==> normal <==> unconstrained -- LESS
	if  ( !strncmp((char*)buffer, "constrained", strlen("constrained")) ) {
		// No restriction on going to constrained
		noexec_mode = CONSTRAINED_MODE;
	} else if ( !strncmp((char*)buffer, "normal", strlen("normal")) ) {
		// If already normal or unconstrained, or unlocked, ok
		if ( is_mdp_authorized(MDP_AUTH_FLAG_EXEC_ENABLE) || noexec_mode >= NORMAL_MODE ) {
			noexec_mode = NORMAL_MODE;
		} else {
			printk(KERN_INFO "Operation not supported on locked device.\n");
		}
	} else if  ( !strncmp((char*)buffer, "unconstrained", strlen("unconstrained")) ) {
		// If already unconstrained, or unlocked, ok
		if ( is_mdp_authorized(MDP_AUTH_FLAG_EXEC_ENABLE) || noexec_mode >= UNCONSTRAINED_MODE ) {
			noexec_mode = UNCONSTRAINED_MODE;
		} else {
			printk(KERN_INFO "Operation not supported on locked device.\n");
		}
	} else {
		printk(KERN_INFO "Unsupported operation.\n");
	}

	printk(KERN_INFO "%d\n", noexec_mode);
	return length;
}

static const struct file_operations noexec_ctrl_proc_fops = {
	.open		= noexec_ctrl_proc_open,
	.read		= seq_read,
	.write		= noexec_ctrl_write_data,
	.llseek		= seq_lseek,
	.release	= single_release,
};



/*	/proc/sonos-lock/insmod_ctrl
 *		Should we allow for insertion/initialization of modules?
 *		rw
 *
 *	On secure devices, we are going to support two modes:
 *		Normal	-	insmod handling is allowed
 *		Constrained -	insmod handling is disallowed
 */

static int insmod_mode = NORMAL_MODE;

int sonos_forbid_insmod(void)
{
	return ( (insmod_mode == CONSTRAINED_MODE) ? 1 : 0 );
}
EXPORT_SYMBOL(sonos_forbid_insmod);
void proc_insmod_set_constrained(void)
{
	insmod_mode = CONSTRAINED_MODE;
}
EXPORT_SYMBOL(proc_insmod_set_constrained);


static int insmod_ctrl_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", insmod_mode);
	return 0;
}

static int insmod_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, insmod_ctrl_proc_show, NULL);
}

//	Allow the device to set or clear forbid_insmod flag.
static int insmod_ctrl_write_data(struct file *file, const char __user * buf, size_t length, loff_t * offset)
{
	char	buffer[64];

        memset(buffer, 0, sizeof(buffer));
        if (length > sizeof(buffer) - 1)
                length = sizeof(buffer) - 1;
        if (copy_from_user(buffer, buf, length))
                return -EFAULT;
	// We can go normal to constrained without unlocking,
	// but not from more constrained to normal.
	if  ( !strncmp((char*)buffer, "forbid", strlen("forbid")) ) {
		// If it's an unlocked device or a diagnostics build, don't
		// constrain...
		if ( !is_mdp_authorized(MDP_AUTH_FLAG_INSMOD_CTRL) ) {
			insmod_mode = CONSTRAINED_MODE;
		} else {
			printk(KERN_INFO "insmod_ctrl authorized - do not constrain.\n");
		}
	} else if ( !strncmp((char*)buffer, "allow", strlen("allow")) ) {
		// Shouldn't matter, but if we're constrained AND not authorized, don't
		// allow, and if we're not, it's fine.
		if ( is_mdp_authorized(MDP_AUTH_FLAG_INSMOD_CTRL) || insmod_mode >= NORMAL_MODE ) {
			insmod_mode = NORMAL_MODE;
		} else {
			printk(KERN_INFO "Operation not supported on locked device.\n");
		}
	} else {
		printk(KERN_INFO "Unsupported operation.\n");
	}

	printk(KERN_INFO "%d\n", insmod_mode);
	return length;
}

static const struct file_operations insmod_ctrl_proc_fops = {
	.open		= insmod_ctrl_proc_open,
	.read		= seq_read,
	.write		= insmod_ctrl_write_data,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*	/proc/sonos-lock/nodev_ctrl
 *		Should we allow for mounting without MNT_NODEV
 *		rw
 *
 *	On secure devices, we are going to support three modes:
 *		Constrained -	nodev handling is applied to all mounts [post-root]
 *		Normal	-	nodev handling is normal
 */

static int nodev_mode = NORMAL_MODE;

int sonos_force_nodev(void)
{
	return ( (nodev_mode == CONSTRAINED_MODE) ? 1 : 0 );
}
EXPORT_SYMBOL(sonos_force_nodev);
void proc_nodev_set_constrained(void)
{
	nodev_mode = CONSTRAINED_MODE;
}
EXPORT_SYMBOL(proc_nodev_set_constrained);

static int nodev_ctrl_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", nodev_mode);
	return 0;
}

static int nodev_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nodev_ctrl_proc_show, NULL);
}

//	Allow the device to set or clear forbid_nodev flag.
static int nodev_ctrl_write_data(struct file *file, const char __user * buf, size_t length, loff_t * offset)
{
	char	buffer[64];

        memset(buffer, 0, sizeof(buffer));
        if (length > sizeof(buffer) - 1)
                length = sizeof(buffer) - 1;
        if (copy_from_user(buffer, buf, length))
                return -EFAULT;
	// We can go from less constrained to more constrained without unlocking,
	// but not from more constrained to less.
	// MORE -- constrained <==> normal <==> unconstrained -- LESS
	if  ( !strncmp((char*)buffer, "constrained",strlen("constrained")) ) {
		// No restriction on going to constrained
		nodev_mode = CONSTRAINED_MODE;
	} else if ( !strncmp((char*)buffer, "normal",strlen("normal")) ) {
		// If already normal or unconstrained, or unlocked, ok
		if ( is_mdp_authorized(MDP_AUTH_FLAG_NODEV_CTRL) || nodev_mode >= NORMAL_MODE ) {
			nodev_mode = NORMAL_MODE;
		} else {
			printk(KERN_INFO "Operation not supported on locked device.\n");
		}
	} else {
		printk(KERN_INFO "Unsupported operation.\n");
	}

	printk(KERN_INFO "%d\n", nodev_mode);
	return length;
}

static const struct file_operations nodev_ctrl_proc_fops = {
	.open		= nodev_ctrl_proc_open,
	.read		= seq_read,
	.write		= nodev_ctrl_write_data,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*
 *	/proc/sonos-lock/console_enable
 *		do we have authorization to start the console?
 *		ro
 */

static int console_ctrl_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", is_mdp_authorized(MDP_AUTH_FLAG_CONSOLE_ENABLE) );
	return 0;
}

static int console_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, console_ctrl_proc_show, NULL);
}

static const struct file_operations console_ctrl_proc_fops = {
	.open		= console_ctrl_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 *	/proc/sonos-lock/telnet_enable
 *		do we have authorization to start telnet
 *		ro
 */

static int telnet_ctrl_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", is_mdp_authorized(MDP_AUTH_FLAG_TELNET_ENABLE));
	return 0;
}

static int telnet_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, telnet_ctrl_proc_show, NULL);
}

static const struct file_operations telnet_ctrl_proc_fops = {
	.open		= telnet_ctrl_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 *	/proc/sonos-lock/mdp_flags
 *		Which sys_mdp.mdp_flags are set?
 *		ro
 */

static int mdp_flags_ctrl_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%08x\n", (int)sys_mdp.mdp_flags);
	return 0;
}

static int mdp_flags_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mdp_flags_ctrl_proc_show, NULL);
}

static const struct file_operations mdp_flags_ctrl_proc_fops = {
	.open		= mdp_flags_ctrl_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 *	/proc/unlock-auth/ubifs_crypt
 *		Initialized by ubifs driver at load time
 *		ro
 */

static int current_ubifs_type = 0;
void sonos_set_proc_crypt(int type)
{
	current_ubifs_type = type;
}
EXPORT_SYMBOL(sonos_set_proc_crypt);

static int ubifs_crypt_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", current_ubifs_type);
	return 0;
}

static int ubifs_crypt_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ubifs_crypt_proc_show, NULL);
}

static const struct file_operations ubifs_crypt_proc_fops = {
	.open		= ubifs_crypt_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*	/proc/sonos-lock/boot_counter
 *		User space access to GPR boot_counter register
 *		rw
 *
 *	When decompression starts, boot_counter gets incremented.  Upon
 *	successful boot, set to 0.  This is part of the auto-rollback
 *	mechanism to deal with bad installs.
 */

static struct regmap *gpr = NULL;
static int boot_counter;

// For reads and writes of the register, the boot counter is treated as
// a uint32.  For changing the boot_counter value, it's treated as a
// BootCounterReg_t.
static void update_boot_counter(int write, int value)
{
	BootCounterReg_t *data;
	if ( !gpr ) {
		gpr = syscon_regmap_lookup_by_compatible(SRC_GPR_COMPATIBLE);
	}

	if ( IS_ERR(gpr) ) {
		printk(KERN_ERR "cannot map imx6q-src\n");
	} else {
		if ( !state_valid ) {
			// First access to the boot counter - may be a read, may be
			// a write, may be coming from an access to the fallback
			// state proc entry.  Go ahead and save the state so that
			// we have it if/when it's needed.
			regmap_read(gpr, BOOT_COUNTER_OFFSET, &boot_counter);
			data = (BootCounterReg_t*)&boot_counter;
			saved_state = data->boot_state;
			state_valid = 1;
		}
		if ( write ) {
			regmap_read(gpr, BOOT_COUNTER_OFFSET, &boot_counter);
			data = (BootCounterReg_t*)&boot_counter;

			if ( value == 0 ) {
				// HACK #1 - there was a brief period, during initial
				// implementation, when the boot_counter was a full 32-bit counter,
				// before it got broken into the separate fields.  Unfortunately,
				// some of the uboot images which treat it that way got used with
				// the new kernel code and, as of Encore EVT2, were still in use.
				// The result of that is that, with those uboots, clearing the 8-bit
				// boot_counter field is not sufficient.  We need to clear both the
				// boot_counter and the boot_section.  Even if we're running with a
				// good uboot and a good kernel, the next image could contain a
				// "bad" - old - uboot, and if the boot section is 1, it will read
				// that as a boot_counter with a value of 0x1000000.  And, while
				// only the uboot will change the boot_state and fallback_fields, if
				// a "good" uboot sets those, the installation of a upd with an old
				// uboot could end up doing a fallback on its initial boot attempt.
				//
				// So if someone writes a 0 to boot_counter, clear all four fields.
				data->boot_section = 0;
				data->fallback_flags = 0;
				data->boot_state = 0;
				data->boot_counter = 0;
#if defined CONFIG_SONOS_DIAGS
			} else if ( value == -1 ) {
				// HACK #2 - In manufacturing we want to avoid a race condition
				// after the upgrade at the Upgrade Station where the operator
				// may disconnect power while various housekeeping tasks that
				// write to flash partitions are in progress. We accomplish this
				// by setting a flag to stop in uboot after a warm-boot.
				data->boot_section = 0;
				data->fallback_flags = BC_FLAG_STOP_BOOT;
				data->boot_state = 0;
				data->boot_counter = 0;
#endif
			} else {
				data->boot_counter = value;
			}
			regmap_write(gpr, BOOT_COUNTER_OFFSET, boot_counter);
		} else {
			regmap_read(gpr, BOOT_COUNTER_OFFSET, &boot_counter);
		}
	}
}

static int boot_counter_proc_show(struct seq_file *m, void *v)
{
	BootCounterReg_t *bc;
	update_boot_counter(0, 0);
	bc = (BootCounterReg_t*)&boot_counter;
	seq_printf(m, "%d.%d.%d.%d\n", bc->boot_counter, bc->boot_state, bc->fallback_flags, bc->boot_section);
	return 0;
}

static int boot_counter_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_counter_proc_show, NULL);
}

//	Allow the device to clear the boot_counter...
static int boot_counter_write_data(struct file *file, const char __user * buf, size_t length, loff_t * offset)
{
	char	buffer[64];
	__s32	new_count;

        memset(buffer, 0, sizeof(buffer));
        if (length > sizeof(buffer) - 1)
                length = sizeof(buffer) - 1;
        if (copy_from_user(buffer, buf, length))
                return -EFAULT;

	// For testing purposes, allow counts of 0 to 10...
	sscanf(buffer, "%d", &new_count);

	if ( new_count >= -1 && new_count <= 10 ) {
		update_boot_counter(1, new_count);
	}
	return length;
}

static const struct file_operations boot_counter_proc_fops = {
	.open		= boot_counter_proc_open,
	.read		= seq_read,
	.write		= boot_counter_write_data,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*	/proc/sonos-lock/watchdog_service_state
 *		who (kernel vs. userspace app) is responsible for petting the 'dog?
 *		rw
 */
extern void wdt2_set_service_state(int);
extern int  wdt2_get_service_state(void);

static int wdog_state_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", wdt2_get_service_state());
	return 0;
}

static int wdog_state_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, wdog_state_proc_show, NULL);
}

static int wdog_state_write_data(struct file *file, const char __user * buf, size_t length, loff_t * offset)
{
	char	buffer[64];
	__s32	new_state;

        memset(buffer, 0, sizeof(buffer));
        if (length > sizeof(buffer) - 1)
                length = sizeof(buffer) - 1;
        if (copy_from_user(buffer, buf, length))
                return -EFAULT;

	sscanf(buffer, "%d", &new_state);

	if ( new_state < 0 || new_state > 2 ) {
		printk(KERN_INFO "illegal state - must be 0, 1 or 2\n");
	} else {
		wdt2_set_service_state(new_state);
	}
	return length;
}

static const struct file_operations wdog_state_proc_fops = {
	.open		= wdog_state_proc_open,
	.read		= seq_read,
	.write		= wdog_state_write_data,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*
 *	/proc/sonos-lock/whitelist_flags
 *  	Reports flag 0x1 if uboot is a whitelist image (not yet implemented).
 *  	Reports flag 0x2 if kernel is a whitelist image.
 *  	ro
 */

static int whitelist_flags_ctrl_proc_show(struct seq_file *m, void *v)
{
	uint32_t flags = 0;
#ifdef CONFIG_SONOS_SECBOOT
	if (__be32_to_cpu(SONOS_FIRMWARE_WHITELIST.header.numEntries) > 0) {
		flags |= 0x2;
	}
#endif
	seq_printf(m, "0x%x\n", flags);
	return 0;
}

static int whitelist_flags_ctrl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, whitelist_flags_ctrl_proc_show, NULL);
}

static const struct file_operations whitelist_flags_proc_fops = {
	.open		= whitelist_flags_ctrl_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*	/proc/sonos-lock/fallback_state
 *		Make the bc->state available to users even if it's been
 *		cleared (which always happens during boot/init).
 *
 *		0 - running from the default partition
 *		1 - fallback due to boot counter
 *		2 - fallback due to sonosboot fallback
 */

static int fallback_state_proc_show(struct seq_file *m, void *v)
{
	if ( !state_valid ) {
		// A read from the boot_counter will set the saved_state.  This
		// should never happen on a healthy board, as the init code
		// clears the boot_counter, so it should always be accessed
		// before anyone is able to attempt it via the console or
		// a telnet session.
		update_boot_counter(0, 0);
	}
	seq_printf(m, "%d\n", saved_state );
	return 0;
}

static int fallback_state_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fallback_state_proc_show, NULL);
}

static const struct file_operations fallback_state_proc_fops = {
	.open		= fallback_state_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * unlock-auth module init
 */

static int __init proc_unlock_auth_init(void)
{
	//proc_create("noexec_ctrl", 0, NULL, &noexec_ctrl_proc_fops);
	proc_create("sonos-lock/console_enable", 0, NULL, &console_ctrl_proc_fops);
	proc_create("sonos-lock/telnet_enable", 0, NULL, &telnet_ctrl_proc_fops);
	proc_create("sonos-lock/insmod_ctrl", 0, NULL, &insmod_ctrl_proc_fops);
	proc_create("sonos-lock/mdp_flags", 0, NULL, &mdp_flags_ctrl_proc_fops);
	proc_create("sonos-lock/dev_enable", 0, NULL, &nodev_ctrl_proc_fops);
	proc_create("sonos-lock/exec_enable", 0, NULL, &noexec_ctrl_proc_fops);
	proc_create("sonos-lock/ubifs_crypt", 0, NULL, &ubifs_crypt_proc_fops);
	proc_create("sonos-lock/boot_counter", 0, NULL, &boot_counter_proc_fops);
	proc_create("sonos-lock/fallback_state", 0, NULL, &fallback_state_proc_fops);


	proc_create("sonos-lock/watchdog_service_state", 0, NULL, &wdog_state_proc_fops);
	proc_create("sonos-lock/whitelist_flags", 0, NULL, &whitelist_flags_proc_fops);
	return 0;
}
module_init(proc_unlock_auth_init);
