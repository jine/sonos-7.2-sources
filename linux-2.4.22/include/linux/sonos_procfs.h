/*
 * Copyright (c) 2014-2016, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#ifndef _SONOS_PROCFS_H
#define _SONOS_PROCFS_H

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
	void *PDE_DATA(const struct inode *);
	static inline struct inode *file_inode(struct file *f)
	{
		return f->f_dentry->d_inode;
	}

	int single_open_size(struct file *, int (*)(struct seq_file *, void *), void *, size_t);
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
	struct proc_dir_entry *proc_create_data(
		const char *name, mode_t mode,
		struct proc_dir_entry *parent,
		const struct file_operations *proc_fops,
		void *data);

	extern int proc_register(
		struct proc_dir_entry * dir,
		struct proc_dir_entry * dp);
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,4,22)
	void *single_start(struct seq_file *p, loff_t *pos);

	void *single_next(struct seq_file *p, void *v, loff_t *pos);

	void single_stop(struct seq_file *p, void *v);

	int single_open(
		struct file *file,
		int (*show)(struct seq_file *, void *),
		void *data);

	int single_release(struct inode *in_inode, struct file *in_file);
#endif

//ALLEN: Following Macros used by all kernels

#define SONOS_PROCFS_PERM_READ	(S_IRUSR | S_IRGRP | S_IROTH)
#define SONOS_PROCFS_PERM_WRITE	(S_IWUSR | S_IWGRP | S_IWOTH)

#define SONOS_PROCFS_INIT_WITH_SIZE(procReadFunc, procWriteFunc, fopsStr, size)	        \
	static int open_ ## procReadFunc(struct inode *inode, struct file *file)			\
	{																					\
		return single_open_size(file, procReadFunc, PDE_DATA(inode), size);				\
	}																					\
	static const struct file_operations fopsStr =										\
	{																					\
		.owner = THIS_MODULE,															\
		.open = open_ ## procReadFunc,													\
		.write = procWriteFunc,															\
		.read = seq_read,																\
		.llseek = seq_lseek,															\
		.release = single_release,														\
	};

#define SONOS_PROCFS_INIT(procReadFunc, procWriteFunc, fopsStr)							\
    SONOS_PROCFS_INIT_WITH_SIZE(procReadFunc, procWriteFunc, fopsStr, PAGE_SIZE)

#endif // ifndef _SONOS_PROCFS_H
