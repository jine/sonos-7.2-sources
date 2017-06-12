/*
 * Copyright (c) 2006, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Support for board leds
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
#include <linux/time.h>
#include "ar531xlnx.h"
#include "ar531x.h"
#include "ar531x-led.h"

static devfs_handle_t ar531x_devfs_handle;

static int led_ledoff = 0;                       /* low true */
static int led_blinking = 0;
static int led_blink_rate = -1;

static int led_Val = 0;

#define LED_OP_SIZE     8
static char led_ops[ LED_OP_SIZE ];
static int  led_cur_op;
static char led_override;

static struct timer_list       led_ledtimer;    /* led off timer */

static const struct blink_rate {
        u_int           rate;           /* tx/rx 802.11 rate */
        u_int16_t       ledon;         /* LED on time (ms) */
        u_int16_t       ledoff;        /* LED off time (ms) */
} led_blinkrates[] = {
        { 108,  (40*HZ)/1000,  (10*HZ)/1000 },
        {  96,  (44*HZ)/1000,  (11*HZ)/1000 },
        {  72,  (50*HZ)/1000,  (13*HZ)/1000 },
        {  48,  (57*HZ)/1000,  (14*HZ)/1000 },
        {  36,  (67*HZ)/1000,  (16*HZ)/1000 },
        {  24,  (80*HZ)/1000,  (20*HZ)/1000 },
        {  22, (100*HZ)/1000,  (25*HZ)/1000 },
        {  18, (133*HZ)/1000,  (34*HZ)/1000 },
        {  12, (160*HZ)/1000,  (40*HZ)/1000 },
        {  10, (200*HZ)/1000,  (50*HZ)/1000 },
        {   6, (240*HZ)/1000,  (58*HZ)/1000 },
        {   4, (267*HZ)/1000,  (66*HZ)/1000 },
        {   2, (400*HZ)/1000,  (100*HZ)/1000 },
        {   0, (500*HZ)/1000,  (130*HZ)/1000 },
        {   0, (750*HZ)/1000,  (750*HZ)/1000 }
};

#define N_BLINK_RATES (sizeof(led_blinkrates)/sizeof(struct blink_rate))
#define LED_BLINK_DEFAULT_RATE  14

static int  ar531xled_getLed(void);
static void ar531xled_led_blink(int rate);
static void ar531xled_setLed( int ledNewVal );

/*
 * Device Methods
 */
 
static void populate_time_formats(struct time_formats *formats)
{
    struct timeval timeofday;
    
    formats->jiffies = 0x0ull | (unsigned long long)jiffies;		//Make sure that the unused 4 bytes is not filled with garbage.
    do_gettimeofday(&timeofday);
    formats->timeofday_sec = timeofday.tv_sec;
    formats->timeofday_usec = timeofday.tv_usec;
    formats->kernel_stamp = jiffies / HZ;
}

static int
ar531xled_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int
ar531xled_close(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t
ar531xled_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    int ledVal = ar531xled_getLed();
    int bcount = count < sizeof(ledVal) ? count : sizeof(ledVal);

    if( copy_to_user( (void *)buf, &ledVal, bcount ) )
        return( -EFAULT );
    return bcount;
}

static ssize_t
ar531xled_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    int ledNewVal;
    int bcount = count < sizeof(ledNewVal) ? count : sizeof(ledNewVal);

    if( copy_from_user( &ledNewVal, (void *)buf, bcount ) )
        return( -EFAULT );
    ar531xled_setLed( ledNewVal );

    return( bcount );
}

static int
ar531xled_ioctl(struct inode *inode,  struct file * file, unsigned int cmd,
	  unsigned long arg)
{
    int ledNewVal;

    switch (cmd) {
    case SIOCSETLED:
        if (copy_from_user(&ledNewVal, (void *)arg, sizeof(ledNewVal)))
            return( -EFAULT );
        led_override = ledNewVal&LED_OVERRIDE;
        ledNewVal &= LED_MASK;

        ar531xled_setLed( ledNewVal );
        break;
    case SIOCGETLED:
        ledNewVal = ar531xled_getLed();
        if (copy_to_user((void *)arg, &ledNewVal, sizeof(ledNewVal)))
            return( -EFAULT );
        break;
    case SIOCBLINKLED:
        if (copy_from_user(&led_ops, (void *)arg, sizeof(led_ops)))
            return( -EFAULT );
        if( led_blink_rate == -1 )
            led_blink_rate = LED_BLINK_DEFAULT_RATE;
        ar531xled_led_blink(led_blink_rate);
        break;
    case SIOCBLINKRATE:
        if (copy_from_user(&ledNewVal, (void *)arg, sizeof(ledNewVal)))
            return( -EFAULT );
        if( ledNewVal >= 0 && ledNewVal < N_BLINK_RATES )
            led_blink_rate = ledNewVal;
        ar531xled_led_blink(led_blink_rate);
        break;
    case SIOCGETTIMESTAMP:
    {
        struct time_formats temp;
		int nbytes = _IOC_SIZE(cmd);
		if (nbytes > sizeof(temp))
			nbytes = sizeof(temp);
		if (copy_from_user((unsigned char *)&temp, (unsigned char *)arg, nbytes))
			return -EFAULT;
		populate_time_formats(&temp);
		//printk(KERN_INFO "timestamp: jiffies %llu, timeofday %lu.%06lu, kernel %lu\n", temp.jiffies, temp.timeofday_sec, temp.timeofday_usec, temp.kernel_stamp);
		if( _IOC_DIR(cmd)&_IOC_READ ) {
			nbytes = sizeof(temp);
			if (nbytes > _IOC_SIZE(cmd))
				nbytes = _IOC_SIZE(cmd);
			if (copy_to_user((unsigned char *)arg, (unsigned char *)&temp, nbytes))
				return -EFAULT;
		}
		break;
	}
    default:
        return( -EINVAL );
        break;
    }
    return 0;
}

/*
 *	Kernel Interfaces
 */

static void
ar531xled_setLed( int ledNewVal )
{
#ifdef CONFIG_AR531X_COBRA
    int ledReg = AR5315_GPIO_DO;
#else
    int ledReg = AR531X_GPIO_DO;
#endif

    led_Val = (sysRegRead(ledReg)& ~LED_MASK)|(ledNewVal&LED_MASK);
    sysRegWrite(ledReg,led_Val);
    (void)sysRegRead(ledReg); /* flush write to hardware */

}

static int 
ar531xled_getLed(void)
{
    return( led_Val&LED_MASK );
}

static void
ar531xled_led_blink_done(unsigned long arg)
{
    led_blinking = 0;
    if( led_blink_rate != -1 )
        ar531xled_led_blink( led_blink_rate );
}

/*
 * Turn the LED off: flip the pin and then set a timer so no
 * update will happen for the specified duration.
 */
static void
ar531xled_led_blink_off(unsigned long arg)
{
    if( led_override == 0 )
        ar531xled_setLed( led_ops[ led_cur_op++ ] );
    led_cur_op &= (LED_OP_SIZE-1);
    led_ledtimer.function = ar531xled_led_blink_done;
    led_ledtimer.expires = jiffies + led_ledoff;
    add_timer(&led_ledtimer);
}

/*
 * Blink the LED according to the specified on/off times.
 */

static void
ar531xled_led_blink_on(int on, int off)
{
    if( led_override == 0 )
        ar531xled_setLed( led_ops[ led_cur_op++ ] );
    led_cur_op &= (LED_OP_SIZE-1);
    led_blinking = 1;
    led_ledoff = off;
    led_ledtimer.function = ar531xled_led_blink_off;
    led_ledtimer.expires = jiffies + on;
    add_timer(&led_ledtimer);
}

static void
ar531xled_led_blink(int rate)
{
    if( led_blinking )
        return;

    if( rate >= 0 && rate < N_BLINK_RATES ) {
        ar531xled_led_blink_on(led_blinkrates[rate].ledon,
                               led_blinkrates[rate].ledoff);
        led_blink_rate = rate;
    }
}

#ifdef CONFIG_PROC_FS

static struct proc_dir_entry   *proc_led;
struct proc_dir_entry          *time_proc_file;

/*
 * fetchop_read_proc
 *
 * Implements /proc/led. Return led settings on/off.
 */
static int
led_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len;

        len = sprintf(page, "leds: %lu\n",(unsigned long)ar531xled_getLed());

        if (len <= off+count) *eof = 1;
        *start = page + off;
        len   -= off;
        if (len>count) len = count;
        if (len<0) len = 0;
        return len;
}

static int time_proc_read(char* page, char **start, off_t off, int count, int *eof, void *data) {
    int i = 0;
    struct time_formats info;
    
    populate_time_formats(&info);
    i += sprintf(page, "Sonos Kernel Time Information\n");
    i += sprintf(page + i,
                 "Jiffies: %llu \nUser space time: %lu.%06lu \nPrintk time: %lu\n",
                 info.jiffies, info.timeofday_sec,
                 info.timeofday_usec, info.kernel_stamp);
    return i;
}

static int
led_write_proc (struct file *file, const char *userbuf, unsigned long count, void *data)
{
    int ledVal;
    int bcount = count < sizeof(ledVal) ? count : sizeof(ledVal);

    if( copy_from_user( &ledVal,(void *)userbuf,bcount ) )
        return -EFAULT;
    ar531xled_setLed( ledVal );

    return bcount;
}
#endif /* CONFIG_PROC_FS */

static struct file_operations ar531xled_fops = {
	owner:		THIS_MODULE,
	read:		ar531xled_read,
	write:		ar531xled_write,
	ioctl:		ar531xled_ioctl,
	open:		ar531xled_open,
	release:	ar531xled_close,
};

static struct miscdevice ar531xled_miscdev = {
	LED_MINOR,
	"leds",
	&ar531xled_fops
};

static int __init
ar531xled_init(void)
{
    int i;
    int regVal;

#ifdef CONFIG_DEVFS_FS
    ar531x_devfs_handle = devfs_register(NULL, "leds", DEVFS_FL_AUTO_DEVNUM,
                        0, 0, S_IFCHR | S_IRUGO | S_IWUGO, &ar531xled_fops, NULL);
    if( !ar531x_devfs_handle ) {
        printk(KERN_ERR "%s:  failed to register device\n", LED_DRIVER_ID_STR);
        return -ENODEV;
    }
#endif

    init_timer(&led_ledtimer);

    printk (KERN_INFO "AR531XLed driver Version: %s\n",VERSION);
    if (misc_register (&ar531xled_miscdev)) {
            printk (KERN_WARNING "AR531XLed: Couldn't register device 10, "
                            "%d.\n", ar531xled_miscdev.minor);
            return -EBUSY;
    }

#ifdef CONFIG_PROC_FS
    if ((proc_led = create_proc_entry(LED_BASENAME, 0644, NULL)) == NULL) {
            printk(KERN_ERR "%s: unable to create proc entry", LED_DRIVER_ID_STR);
            return -EINVAL;
    }
    proc_led->read_proc = led_read_proc;
    // Time procfs file
	time_proc_file = create_proc_read_entry("timeinfo", (S_IRUSR|S_IWUSR), 0, time_proc_read, 0);
	if (!time_proc_file) {
		remove_proc_entry( LED_BASENAME,proc_led );
		return -EIO;
	}
    proc_led->write_proc = led_write_proc;
#endif /* CONFIG_PROC_FS */

    // set the dir register so we have an output
    regVal = sysRegRead( AR5315_GPIO_CR );
    regVal |= LED_MASK;
    sysRegWrite( AR5315_GPIO_CR,regVal );

    for( i = 0; i < 8; i++ )
        led_ops[i] = (i&01) ? 0 : LED_WHITE;
    ar531xled_led_blink( LED_BLINK_DEFAULT_RATE );

    return 0;

}

static void __exit
ar531xled_exit(void)
{
    if( led_blinking ) {
        del_timer(&led_ledtimer);
        led_blinking = 0;
    }
    ar531xled_setLed( 0 );
#ifdef CONFIG_PROC_FS
    if( proc_led )
        remove_proc_entry( LED_BASENAME,NULL );
    
    if (time_proc_file)
		remove_proc_entry("timeinfo", time_proc_file);
#endif
#ifdef CONFIG_DEVFS_FS
    if( ar531x_devfs_handle ) {
        devfs_unregister( ar531x_devfs_handle );
        ar531x_devfs_handle = NULL;
    }
#endif
    misc_deregister( &ar531xled_miscdev );
}

module_init(ar531xled_init);
module_exit(ar531xled_exit);

MODULE_AUTHOR("Sonos, Inc.");
MODULE_DESCRIPTION("Support for AP51/61 Leds");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Sonos");
#endif
