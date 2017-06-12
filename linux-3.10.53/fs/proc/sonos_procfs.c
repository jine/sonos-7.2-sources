/*
 * Copyright (c) 2014-2016, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <linux/sonos_procfs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/version.h>

//ALLEN: All the functions are copied from new kernels, so that old kernels can use it.

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,4,22)

static inline struct proc_dir_entry *PDE(const struct inode *inode)
{
	return (struct proc_dir_entry *) ((inode)->u.generic_ip);
}

void *single_start(struct seq_file *p, loff_t *pos)
{
	return NULL + (*pos == 0);
}

void *single_next(struct seq_file *p, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

void single_stop(struct seq_file *p, void *v)
{
}

int single_open(
	struct file *file,
	int (*show)(struct seq_file *, void *),
	void *data)
{
	struct seq_operations *op = kmalloc(sizeof(*op), GFP_KERNEL);
	int res = -ENOMEM;

	if (op) {
		op->start = single_start;
		op->next = single_next;
		op->stop = single_stop;
		op->show = show;

		res = seq_open(file, op);
		if (!res) {
			struct seq_file *seq_filp =
				(struct seq_file *)(file->private_data);

			seq_filp->private = data;
		} else {
			kfree(op);
		}
	}

	return res;
}
EXPORT_SYMBOL(single_open);


int single_release(struct inode *in_inode, struct file *in_file)
{
	struct seq_file *seq_filp =
		(struct seq_file *)(in_file->private_data);

	const struct seq_operations *op = seq_filp->op;
	int res = seq_release(in_inode, in_file);
	kfree(op);
	return res;
}
EXPORT_SYMBOL(single_release);

#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)

static int xlate_proc_name_ex(const char *name,
	struct proc_dir_entry **ret, const char **residual)
{
	const char		*cp = name, *next;
	struct proc_dir_entry	*de;
	int			len;

	de = *ret;  // start at *ret not proc_root
	if (!de)
		de = &proc_root;

	while (1) {
		next = strchr(cp, '/');
		if (!next)
			break;

		len = next - cp;
		for (de = de->subdir; de ; de = de->next) {
			if (proc_match(len, cp, de))
				break;
		}
		if (!de)
			return -ENOENT;
		cp += len + 1;
	}
	*residual = cp;
	*ret = de;
	return 0;
}

static struct proc_dir_entry *__proc_create(
	struct proc_dir_entry **parent,
	const char *name,
	mode_t mode,
	nlink_t nlink)
{
	struct proc_dir_entry *ent = NULL;
	const char *fn = name;
	int len;

	/* make sure name is valid */
	if (!name || !strlen(name)) goto out;

	// Call the extrapolated version which keeps the parent directory
	if (xlate_proc_name_ex(name, parent, &fn) != 0)
		goto out;

	/* At this point there must not be any '/' characters beyond *fn */
	if (strchr(fn, '/'))
		goto out;

	len = strlen(fn);

	ent = kmalloc(sizeof(struct proc_dir_entry) + len + 1, GFP_KERNEL);
	if (!ent) goto out;

	memset(ent, 0, sizeof(struct proc_dir_entry));
	memcpy(((char *) ent) + sizeof(struct proc_dir_entry), fn, len + 1);
	ent->name = ((char *) ent) + sizeof(*ent);
	ent->namelen = len;
	ent->mode = mode;
	ent->nlink = nlink;
	atomic_set(&ent->count, 1);
 out:
	return ent;
}

struct proc_dir_entry *proc_create_data(
	const char *name, mode_t mode,
	struct proc_dir_entry *parent,
	const struct file_operations *proc_fops,
	void *data)
{
	struct proc_dir_entry *pde;
	nlink_t nlink;

	if (S_ISDIR(mode)) {
		if ((mode & S_IALLUGO) == 0)
			mode |= S_IRUGO | S_IXUGO;
		nlink = 2;
	} else {
		if ((mode & S_IFMT) == 0)
			mode |= S_IFREG;
		if ((mode & S_IALLUGO) == 0)
			mode |= S_IRUGO;
		nlink = 1;
	}

	pde = __proc_create(&parent, name, mode, nlink);
	if (!pde)
		goto out;
	pde->proc_fops = (struct file_operations *) proc_fops;
	pde->data = data;
	if (proc_register(parent, pde) < 0)
		goto out_free;
	return pde;
out_free:
	kfree(pde);
out:
	return NULL;
}

EXPORT_SYMBOL(proc_create_data);

#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
static inline void *__PDE_DATA(const struct inode *inode)
{
	return PDE(inode)->data;
}

void *PDE_DATA(const struct inode *inode)
{
	return __PDE_DATA(inode);
}
EXPORT_SYMBOL(PDE_DATA);

int single_open_size(struct file *file, int (*show)(struct seq_file *, void *),
		void *data, size_t size)
{
	char *buf = kmalloc(size, GFP_KERNEL);
	int ret;
	if (!buf)
		return -ENOMEM;
	ret = single_open(file, show, data);
	if (ret) {
		kfree(buf);
		return ret;
	}
	((struct seq_file *)file->private_data)->buf = buf;
	((struct seq_file *)file->private_data)->size = size;
	return 0;
}
EXPORT_SYMBOL(single_open_size);
#endif
