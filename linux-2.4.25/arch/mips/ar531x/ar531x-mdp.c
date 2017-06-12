/*
 * Copyright (c) 2006, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Support for board Manufacturing Data Page (MDP)
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>
#include <linux/tqueue.h>
#include <linux/mtd/mtd.h>
#include "mdp.h"
#include "ar531xlnx.h"
#include "ar531x.h"
#include "ar531x-mdp.h"

/*
 * Device Methods
 */

struct mdp cache_mdp;

static int
ar531xmdp_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int
ar531xmdp_close(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t
ar531xmdp_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    if( *ppos == 0 ) {
        if( count > sizeof(struct manufacturing_data_page) )
            count = sizeof(struct manufacturing_data_page);
        if (copy_to_user(buf, (void *)&cache_mdp.mdp, count))
            return -EFAULT;
    } else {
        if( count > sizeof(struct manufacturing_data_page2) )
            count = sizeof(struct manufacturing_data_page2);
        if (copy_to_user(buf, (void *)&cache_mdp.mdp2, count))
            return -EFAULT;
    }
        
    return( count );
}

static struct mtd_info *get_mtd( char *name )
{
    int i;
    struct mtd_info *mtd = NULL;

    /* get the mtd_info structure for the first mtd device*/
    for(i = 0; i < MAX_MTD_DEVICES; i++){
        mtd = get_mtd_device(NULL,i);
        if(!mtd || !strcmp(mtd->name,name) ) 
            break;
    }
    return( mtd );
}

#define SECTOR_SIZE         0x10000
#define MDP_SECTOR_NAME     "Mfr Data Page"
static struct erase_info ar531xmdp_erinfo;

static void ar531xmdp_erase_callback (struct erase_info *instr)
{
        wake_up((wait_queue_head_t *)instr->priv);
}

static int ar531xmdp_eraseSector(struct mtd_info *mtd, int sector)
{
    int err;
    wait_queue_head_t waitq;
    DECLARE_WAITQUEUE(wait, current);
    struct erase_info *e = &ar531xmdp_erinfo;

    init_waitqueue_head(&waitq);

    e->mtd = mtd;
    e->addr = sector;
    e->len = SECTOR_SIZE;
    e->callback = ar531xmdp_erase_callback;
    e->state = 0;
    e->priv = (unsigned long)&waitq;

    if( (err = mtd->erase(mtd, e)) == 0 ) {
        set_current_state(TASK_UNINTERRUPTIBLE);
        add_wait_queue(&waitq, &wait);
        if (e->state != MTD_ERASE_DONE && e->state != MTD_ERASE_FAILED)
                schedule();
        remove_wait_queue(&waitq, &wait);
        set_current_state(TASK_RUNNING);

        err = (e->state == MTD_ERASE_FAILED) ? -EIO : 0;
    } else
        printk("ER ERR! - %d\n",err);
    return err;
}

int ar531xmdp_flashUpdate( int off, const char *buf, int len )
{
    int ret, tlen;
    struct mtd_info *mtd = get_mtd( MDP_SECTOR_NAME );
    
    if(  mtd == NULL ) {
        return( -EBADF );
    }

    if( (ret = ar531xmdp_eraseSector( mtd,0 )) != 0 )
        goto out;
    if( (ret = (*(mtd->write))( mtd,0,len,&tlen,buf )) != 0 )
        goto out;

out:
    put_mtd_device( mtd );
    return( ret );
}

static ssize_t
ar531xmdp_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    int ret;

    if( *ppos == 0 ) {
        if( count > sizeof(struct manufacturing_data_page) )
            count = sizeof(struct manufacturing_data_page);
        if (copy_from_user((void *)&cache_mdp.mdp, buf, count))
            return -EFAULT; 
    } else {
        if( count > sizeof(struct manufacturing_data_page2) )
            count = sizeof(struct manufacturing_data_page2);
        if (copy_from_user((void *)&cache_mdp.mdp2, buf, count))
            return -EFAULT;
    }
    if( (ret = ar531xmdp_flashUpdate( 0,(void *)&cache_mdp,sizeof(struct mdp) )) != 0 )
        return( -ret );

    return count;
}
    
static int
ar531xmdp_ioctl(struct inode *inode,  struct file * file, unsigned int cmd,
	  unsigned long arg)
{
    return 0;
}

/*
 *	Kernel Interfaces
 */

static struct file_operations ar531xmdp_fops = {
        owner:          THIS_MODULE,
        read:           ar531xmdp_read,       /* read */
        write:          ar531xmdp_write,      /* write */
        ioctl:          ar531xmdp_ioctl,      /* ioctl */
        open:           ar531xmdp_open,       /* open */
        release:        ar531xmdp_close,      /* release */
};


static struct miscdevice ar531xmdp_miscdev = {
	MDP_MINOR,
	"mdp",
	&ar531xmdp_fops
};

static devfs_handle_t ar531x_devfs_handle;

static int __init
ar531xmdp_init(void)
{
    int i;
    int retlen;
    struct mtd_info *mtd = get_mtd( MDP_SECTOR_NAME );

    if(  mtd == NULL )
        return -ENODEV;

    i = (*(mtd->read))(mtd, 0, sizeof(struct mdp), &retlen, (void *)&cache_mdp);
    put_mtd_device( mtd );

    if( i != 0 ) 
        return i;

#ifdef CONFIG_DEVFS_FS
    ar531x_devfs_handle = devfs_register(NULL, "mdp", DEVFS_FL_AUTO_DEVNUM,
                        0, 0, S_IFCHR | S_IRUGO | S_IWUGO, &ar531xmdp_fops, NULL);
    if( !ar531x_devfs_handle ) {
        printk(KERN_ERR "%s:  failed to register device\n", DRIVER_ID_STR);
        return -ENODEV;
    }
#endif
    printk (KERN_INFO "AR531XMDP driver Version: %s\n",VERSION);
    if (misc_register (&ar531xmdp_miscdev)) {
        printk (KERN_WARNING "AR531XLed: Couldn't register device 10, "
                        "%d.\n", ar531xmdp_miscdev.minor);
        return -EBUSY;
    }
    return 0;
}

static void __exit
ar531xmdp_exit(void)
{
#ifdef CONFIG_DEVFS_FS
    if( ar531x_devfs_handle ) {
        devfs_unregister( ar531x_devfs_handle );
        ar531x_devfs_handle = NULL;
    }
#endif
    misc_deregister( &ar531xmdp_miscdev );
}

module_init(ar531xmdp_init);
module_exit(ar531xmdp_exit);

MODULE_AUTHOR("Sonos, Inc.");
MODULE_DESCRIPTION("Support for AP51/61 MDP");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Sonos");
#endif
