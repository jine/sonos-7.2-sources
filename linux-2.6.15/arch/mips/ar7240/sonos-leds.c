/*
 * Copyright (c) 2006-2010, Sonos, Inc.
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
#include <linux/jiffies.h>
#include "ar7240.h"
#include "sonos-leds.h"

#define LED_MINOR 128

#define GPIO_LED_WHITE_PIN   11
#define GPIO_LED_RED_PIN     16
#define GPIO_LED_GREEN_PIN   15
#define GPIO_LED_AMBER_PIN   14

#define GPIO_LED_WHITE       (1 << GPIO_LED_WHITE_PIN)
#define GPIO_LED_RED         (1 << GPIO_LED_RED_PIN)
#define GPIO_LED_GREEN       (1 << GPIO_LED_GREEN_PIN)
#define GPIO_LED_AMBER       (1 << GPIO_LED_AMBER_PIN)
#define GPIO_LED_MASK (GPIO_LED_WHITE | GPIO_LED_RED | GPIO_LED_GREEN | GPIO_LED_AMBER)

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

static int  sonos_led_getLed(void);
static void sonos_led_blink(int rate);
static void sonos_led_setLed( int ledNewVal );
void ar7240_gpio_config_output(int gpio);

extern unsigned long mdp_getRevision();
static unsigned long mdp_revision = 1;

/*
 * Device Methods
 */
 
static void populate_time_formats(struct time_formats *formats)
{
    struct timeval timeofday;
    
    formats->jiffies = get_jiffies_64();
    do_gettimeofday(&timeofday);
    formats->timeofday_sec = timeofday.tv_sec;
    formats->timeofday_usec = timeofday.tv_usec;
    formats->kernel_stamp = jiffies / HZ;
}

static int
sonos_led_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int
sonos_led_close(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t
sonos_led_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    int ledVal = sonos_led_getLed();
    int bcount = count < sizeof(ledVal) ? count : sizeof(ledVal);

    if( copy_to_user( (void *)buf, &ledVal, bcount ) )
        return( -EFAULT );
    return bcount;
}

static ssize_t
sonos_led_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    int ledNewVal;
    int bcount = count < sizeof(ledNewVal) ? count : sizeof(ledNewVal);

    if( copy_from_user( &ledNewVal, (void *)buf, bcount ) )
        return( -EFAULT );
    sonos_led_setLed( ledNewVal );

    return( bcount );
}

/* converts a GPIO LED mask to an API mask */
static int sonos_led_get_api_mask(int gpio)
{
   int api_mask = 0;

   /* White LED is always active HIGH */ 
   if ((gpio & GPIO_LED_WHITE) != 0)
      api_mask |= LED_WHITE;

   if (mdp_revision >= 1) {
      /* color LEDs are active HIGH on new HW */
      if ((gpio & GPIO_LED_RED) != 0)
         api_mask |= LED_RED;
      if ((gpio & GPIO_LED_GREEN) != 0)
         api_mask |= LED_GREEN;
      if ((gpio & GPIO_LED_AMBER) != 0)
         api_mask |= LED_AMBER;
   }
   else {
      /* color LEDs are active LOW on old HW */
      if ((gpio & GPIO_LED_RED) == 0)
         api_mask |= LED_RED;
      if ((gpio & GPIO_LED_GREEN) == 0)
         api_mask |= LED_GREEN;
      if ((gpio & GPIO_LED_AMBER) == 0)
         api_mask |= LED_AMBER;
   }
   
   return api_mask;      
}

/* converts an API mask to a GPIO register mask */
static int sonos_led_get_gpio_mask(int val)
{
   int gpio_mask = 0;

   /* White LED is active HIGH */
   if ((val & LED_WHITE) != 0)
      gpio_mask |= GPIO_LED_WHITE;

   /* color LEDs are active LOW */
   if (mdp_revision >= 1) {
      /* color LEDs are active HIGH on new HW */
      if ((val & LED_RED) != 0)
         gpio_mask |= GPIO_LED_RED;
      if ((val & LED_GREEN) != 0)
         gpio_mask |= GPIO_LED_GREEN;
      if ((val & LED_AMBER) != 0)
         gpio_mask |= GPIO_LED_AMBER;
   }
   else {
      /* color LEDs are active LOW on old HW */
      if ((val & LED_RED) == 0)
         gpio_mask |= GPIO_LED_RED;
      if ((val & LED_GREEN) == 0)
         gpio_mask |= GPIO_LED_GREEN;
      if ((val & LED_AMBER) == 0)
         gpio_mask |= GPIO_LED_AMBER;
   }
   
   return gpio_mask;      
}

static int
sonos_led_ioctl(struct inode *inode,  struct file * file, unsigned int cmd,
	  unsigned long arg)
{
    int ledNewVal;

    switch (cmd) {
    case SIOCSETLED:
        if (copy_from_user(&ledNewVal, (void *)arg, sizeof(ledNewVal)))
            return( -EFAULT );
        led_override = ledNewVal & LED_OVERRIDE;
        ledNewVal &= LED_MASK;

        sonos_led_setLed( ledNewVal );
        break;
    case SIOCGETLED:
        ledNewVal = sonos_led_getLed();
        if (copy_to_user((void *)arg, &ledNewVal, sizeof(ledNewVal)))
            return( -EFAULT );
        break;
    case SIOCBLINKLED:
        if (copy_from_user(&led_ops, (void *)arg, sizeof(led_ops)))
            return( -EFAULT );
        if( led_blink_rate == -1 )
            led_blink_rate = LED_BLINK_DEFAULT_RATE;
        sonos_led_blink(led_blink_rate);
        break;
    case SIOCBLINKRATE:
        if (copy_from_user(&ledNewVal, (void *)arg, sizeof(ledNewVal)))
            return( -EFAULT );
        if( ledNewVal >= 0 && ledNewVal < N_BLINK_RATES )
            led_blink_rate = ledNewVal;
        sonos_led_blink(led_blink_rate);
        break;
    case SIOCGETTIMESTAMP:
    {
        struct time_formats temp;
		if (copy_from_user((unsigned char *)&temp, (unsigned char *)arg, sizeof(struct time_formats)))
			return -EFAULT;
		populate_time_formats(&temp);
		//printk(KERN_INFO "timestamp: jiffies %llu, timeofday %lu.%06lu, kernel %lu\n", temp.jiffies, temp.timeofday_sec, temp.timeofday_usec, temp.kernel_stamp);
		if (copy_to_user((unsigned char *)arg, (unsigned char *)&temp, sizeof(struct time_formats)))
			return -EFAULT;
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
sonos_led_setLed( int ledNewVal )
{
   int gpioLedPins;
   int gpioOtherPins;
   int gpioAllPins;

   gpioLedPins = sonos_led_get_gpio_mask(ledNewVal);
   gpioOtherPins = ar7240_reg_rd(AR7240_GPIO_OUT) & ~GPIO_LED_MASK;
   gpioAllPins = gpioOtherPins | gpioLedPins;
   ar7240_reg_wr(AR7240_GPIO_OUT, gpioAllPins);
   led_Val = sonos_led_get_api_mask(gpioAllPins);
}

static int 
sonos_led_getLed(void)
{
    return( led_Val & LED_MASK );
}

static void
sonos_led_blink_done(unsigned long arg)
{
    led_blinking = 0;
    if( led_blink_rate != -1 )
        sonos_led_blink( led_blink_rate );
}

/*
 * Turn the LED off: flip the pin and then set a timer so no
 * update will happen for the specified duration.
 */
static void
sonos_led_blink_off(unsigned long arg)
{
    if( led_override == 0 )
        sonos_led_setLed( led_ops[ led_cur_op++ ] );
    led_cur_op &= (LED_OP_SIZE-1);
    led_ledtimer.function = sonos_led_blink_done;
    led_ledtimer.expires = jiffies + led_ledoff;
    add_timer(&led_ledtimer);
}

/*
 * Blink the LED according to the specified on/off times.
 */

static void
sonos_led_blink_on(int on, int off)
{
    if( led_override == 0 )
        sonos_led_setLed( led_ops[ led_cur_op++ ] );
    led_cur_op &= (LED_OP_SIZE-1);
    led_blinking = 1;
    led_ledoff = off;
    led_ledtimer.function = sonos_led_blink_off;
    led_ledtimer.expires = jiffies + on;
    add_timer(&led_ledtimer);
}

static void
sonos_led_blink(int rate)
{
    if( led_blinking )
        return;

    if( rate >= 0 && rate < N_BLINK_RATES ) {
        sonos_led_blink_on(led_blinkrates[rate].ledon,
                               led_blinkrates[rate].ledoff);
        led_blink_rate = rate;
    }
}

#ifdef CONFIG_PROC_FS

static struct proc_dir_entry   *proc_led;
static struct proc_dir_entry   *time_proc_file;

/*
 * fetchop_read_proc
 *
 * Implements /proc/led. Return led settings on/off.
 */
static int
led_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len;

        len = sprintf(page, "leds: %lu %08x %08x (color LEDs active %s)\n",
                      (unsigned long)sonos_led_getLed(),
                      ar7240_reg_rd(AR7240_GPIO_OUT), ar7240_reg_rd(AR7240_GPIO_OE),
                      mdp_revision ? "HIGH" : "LOW");

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
    unsigned long ledVal;
    char str[8];
    int bcount = count < sizeof(str) ? count : sizeof(str);

    if( copy_from_user( str,(void *)userbuf,bcount ) )
        return -EFAULT;
    str[bcount-1] = '\0';
    ledVal = simple_strtoul(str, NULL, 16);
    sonos_led_setLed( ledVal );

    return bcount;
}
#endif /* CONFIG_PROC_FS */

static struct file_operations sonos_led_fops = {
	owner:		THIS_MODULE,
	read:		   sonos_led_read,
	write:		sonos_led_write,
	ioctl:		sonos_led_ioctl,
	open:		   sonos_led_open,
	release:	   sonos_led_close,
};

static struct miscdevice sonos_led_miscdev = {
	LED_MINOR,
	"leds",
	&sonos_led_fops
};

static int __init
sonos_led_init(void)
{
    int i;

    init_timer(&led_ledtimer);

    printk (KERN_INFO "Sonos LED driver Version: %s\n",VERSION);
    if (misc_register (&sonos_led_miscdev)) {
            printk (KERN_WARNING "Sonos_Led: Couldn't register device 10, "
                            "%d.\n", sonos_led_miscdev.minor);
            return -EBUSY;
    }
    mdp_revision = mdp_getRevision();
    printk("MDP revision: %d\n", mdp_revision);

#ifdef CONFIG_PROC_FS
    if ((proc_led = create_proc_entry(LED_BASENAME, 0644, NULL)) == NULL) {
            printk(KERN_ERR "%s: unable to create proc entry", LED_DRIVER_ID_STR);
            return -EINVAL;
    }
    proc_led->read_proc = led_read_proc;
    proc_led->write_proc = led_write_proc;
    
    // Time procfs file
	time_proc_file = create_proc_read_entry("timeinfo", (S_IRUSR|S_IWUSR), 0, time_proc_read, 0);
	if (!time_proc_file) {
		printk(KERN_ERR "%s: unable to create time proc entry", LED_DRIVER_ID_STR);
		return -EINVAL;
	}
#endif /* CONFIG_PROC_FS */

    // set the dir register so we have an output
    ar7240_gpio_config_output(GPIO_LED_WHITE);
    ar7240_gpio_config_output(GPIO_LED_RED);
    ar7240_gpio_config_output(GPIO_LED_GREEN);
    ar7240_gpio_config_output(GPIO_LED_AMBER);

    for( i = 0; i < 8; i++ )
        led_ops[i] = (i&01) ? 0 : LED_WHITE;
    sonos_led_blink( LED_BLINK_DEFAULT_RATE );

    return 0;

}

static void __exit
sonos_led_exit(void)
{
    if( led_blinking ) {
        del_timer(&led_ledtimer);
        led_blinking = 0;
    }
    sonos_led_setLed( 0 );
#ifdef CONFIG_PROC_FS
    if( proc_led )
        remove_proc_entry( LED_BASENAME,NULL );
    if (time_proc_file)
		remove_proc_entry("timeinfo", time_proc_file);
#endif
    misc_deregister( &sonos_led_miscdev );
}

module_init(sonos_led_init);
module_exit(sonos_led_exit);

MODULE_AUTHOR("Sonos, Inc.");
MODULE_DESCRIPTION("Support for Sonos Dock Leds");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Sonos");
#endif
