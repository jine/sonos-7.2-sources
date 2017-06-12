#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

extern char uboot_version_str[120];

static int uboot_revision_proc_show(struct seq_file *m, void *v)
{
	if ( uboot_version_str[0] == 'U' )
		seq_printf(m, "%s\n", uboot_version_str);
	else
		seq_printf(m, "ASSERT: upgrade U-boot to 2014.04.04\n");
	return 0;
}

static int uboot_revision_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, uboot_revision_proc_show, NULL);
}

static const struct file_operations uboot_revision_proc_fops = {
	.open		= uboot_revision_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_uboot_revision_init(void)
{
	proc_create("uboot_revision", 0, NULL, &uboot_revision_proc_fops);
	return 0;
}
module_init(proc_uboot_revision_init);
