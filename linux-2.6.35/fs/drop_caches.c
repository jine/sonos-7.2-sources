/*
 * Implement the manual drop-all-pagecache function
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/writeback.h>
#include <linux/sysctl.h>
#include <linux/gfp.h>
#ifdef CONFIG_SONOS_FILLMORE
#include <linux/timer.h>
#include <linux/workqueue.h>

#define CACHE_CLEANUP_TIMEOUT (4 * HZ)

static struct timer_list cache_cleanup_timer = {
    .function = NULL,
    .data = 0,
};

static struct work_struct drop_cache_workq;
#endif

/* A global variable is a bit ugly, but it keeps the code simple */
int sysctl_drop_caches;

static void drop_pagecache_sb(struct super_block *sb, void *unused)
{
	struct inode *inode, *toput_inode = NULL;

	spin_lock(&inode_lock);
	list_for_each_entry(inode, &sb->s_inodes, i_sb_list) {
		if (inode->i_state & (I_FREEING|I_CLEAR|I_WILL_FREE|I_NEW))
			continue;
		if (inode->i_mapping->nrpages == 0)
			continue;
		__iget(inode);
		spin_unlock(&inode_lock);
		invalidate_mapping_pages(inode->i_mapping, 0, -1);
		iput(toput_inode);
		toput_inode = inode;
		spin_lock(&inode_lock);
	}
	spin_unlock(&inode_lock);
	iput(toput_inode);
}

static void drop_slab(void)
{
	int nr_objects;

	do {
		nr_objects = shrink_slab(1000, GFP_KERNEL, 1000);
	} while (nr_objects > 10);
}

#ifdef CONFIG_SONOS_FILLMORE
static void drop_caches_timer_handler(void)
{
        schedule_work(&drop_cache_workq);
}

static void drop_caches(void *arg)
{
        //printk("**** %s: starting the cache cleanup ...**** \n", __func__);
	iterate_supers(drop_pagecache_sb, NULL);
        drop_slab();
        mod_timer(&cache_cleanup_timer, (jiffies + CACHE_CLEANUP_TIMEOUT));
}
#endif

int drop_caches_sysctl_handler(ctl_table *table, int write,
	void __user *buffer, size_t *length, loff_t *ppos)
{
	proc_dointvec_minmax(table, write, buffer, length, ppos);
#ifdef CONFIG_SONOS_FILLMORE
        if (cache_cleanup_timer.function == NULL)
        {
            init_timer(&cache_cleanup_timer);
            INIT_WORK(&drop_cache_workq, (void *)drop_caches);
            cache_cleanup_timer.expires = (jiffies + CACHE_CLEANUP_TIMEOUT);
            cache_cleanup_timer.function = (void *)drop_caches_timer_handler;
            printk("**** %s: all done timer added ...**** \n", __func__);
            add_timer(&cache_cleanup_timer);
        }
        else
            *length = 0;
#else
	if (write) {
		if (sysctl_drop_caches & 1)
			iterate_supers(drop_pagecache_sb, NULL);
		if (sysctl_drop_caches & 2)
			drop_slab();
	}
#endif
	return 0;
}
