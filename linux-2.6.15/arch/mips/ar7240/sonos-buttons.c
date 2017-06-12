/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright © 2003 Atheros Communications, Inc.,  All Rights Reserved.
 */

/*
 * Support for board buttons
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
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include "ar7240.h"
#include "sonos-ctl.h"
#include "sonos-buttons.h"
#include "button_event.h"

/* low level function protypes from gpio.c */
void ar7240_gpio_config_input(int gpio);
void ar7240_gpio_intr_disable(unsigned int irq);

/* FACTORY_RESET_MINOR is defined in inclue/linux/miscdevices.h and it is the
 * same minor number used by the zonebridge */
#define BUTTON_MINOR FACTORY_RESET_MINOR

#define KEYSTORE_SIZE 16

/* GPIO pins define by HW */
#define GPIO_BUTTON_HOUSEHOLD    12
#define GPIO_BUTTON_VOLUME_UP     6
#define GPIO_BUTTON_VOLUME_DOWN   7

/* Button masks */
#define BUTTON_HOUSEHOLD_MASK    (1 << GPIO_BUTTON_HOUSEHOLD)
#define BUTTON_VOLUMEUP_MASK     (1 << GPIO_BUTTON_VOLUME_UP)
#define BUTTON_VOLUMEDOWN_MASK   (1 << GPIO_BUTTON_VOLUME_DOWN)

static int last_button;
static int button_mask = (BUTTON_HOUSEHOLD_MASK |
                          BUTTON_VOLUMEUP_MASK  |
                          BUTTON_VOLUMEDOWN_MASK);
spinlock_t button_queue_lock;
wait_queue_head_t button_queue_kwq;
struct button_event_queue *button_queue;

struct button_sim_match button_sim_matches[] = {
    {"join",  EVTSOURCE_BUTTON0},
    {"volup", EVTSOURCE_BUTTON1},
    {"voldn", EVTSOURCE_BUTTON2},
    {NULL, EVTSOURCE_NO_SOURCE}};

// button gpio pins
#define SW_HOUSEHOLD                0x00000010U
#define SW_VOL_UP                   0x00000001U
#define SW_VOL_DWN                  0x00000002U


struct proc_dir_entry *sonos_button_proc_file;
struct timer_list sonos_button_proc_simulate_keys_timer;
unsigned long sonos_button_simulation_value;
unsigned long *sonos_button_simulation = NULL;


#define MAX_SIM_KEY_DURATION 15000  // 15sec
#define MS_TO_JIFFIES(x) (((x) * HZ) / 1000)
#define JIFFIES_TO_MS(x) (((x) * 1000) / HZ )

// assumptions: called with button_sem set since we're
// going to stuff chars into the key buffer
// returns 0 if no change, 1 if we put something into the keybuf
//
static int
process_button(int newbutton)
{
   int ret = 0;
   static int volup_ctr = 0;	// number of calls with button pressed
   static int voldn_ctr = 0;	// number of calls with button pressed

   if (sonos_button_simulation) {
      newbutton = *sonos_button_simulation;
   }

   /* newbutton is an int mask based on the gpio input register. Since
      the button queue is button_evt we need to convert to button_evt here.
      Down(0) is PRESSED, up(1) is RELEASED */

   // volup pressed/repeated/released
   if ((newbutton & BUTTON_VOLUMEUP_MASK) == 0) {
      if (!volup_ctr) { // volup button pressed
         button_event_send(button_queue, EVTSOURCE_BUTTON1, EVTINFO_PRESSED);
         ret = 1;
      }
      if (volup_ctr == 4 || (volup_ctr > 4 && ((volup_ctr&1) == 0))) {
         button_event_send(button_queue, EVTSOURCE_BUTTON1, EVTINFO_REPEATED);
         ret = 1;
      }
      volup_ctr++;
   } else {
      if (volup_ctr) {
         button_event_send(button_queue, EVTSOURCE_BUTTON1, EVTINFO_RELEASED);
         ret = 1;
      }
      volup_ctr = 0;
   }

   // voldn pressed/repeated/released
   if ((newbutton & BUTTON_VOLUMEDOWN_MASK) == 0) {
      if (!voldn_ctr) { // voldn button pressed
         button_event_send(button_queue, EVTSOURCE_BUTTON2, EVTINFO_PRESSED);
         ret = 1;
      }
      if (voldn_ctr == 4 || (voldn_ctr > 4 && ((voldn_ctr&1) == 0))) {
         button_event_send(button_queue, EVTSOURCE_BUTTON2, EVTINFO_REPEATED);
         ret = 1;
      }
      voldn_ctr++;
   } else {
      if (voldn_ctr) {
         button_event_send(button_queue, EVTSOURCE_BUTTON2, EVTINFO_RELEASED);
         ret = 1;
      }
      voldn_ctr = 0;
   }

   if ((newbutton & BUTTON_HOUSEHOLD_MASK) != (last_button & BUTTON_HOUSEHOLD_MASK)) {
      button_event_send(button_queue,
                        EVTSOURCE_BUTTON0,
                        (newbutton & BUTTON_HOUSEHOLD_MASK) ? EVTINFO_RELEASED : EVTINFO_PRESSED);
      ret = 1;
   }

   last_button = newbutton;
   return ret;
}

/*
 * Button poll thread
 */

static int  button_defer_poll = 0;

// called by button_device_thread to read button input

static void
button_poll(void)
{
    int ret;

    if( button_defer_poll )
        return;
    ret = process_button( ar7240_reg_rd(AR7240_GPIO_IN) & button_mask );
    if( ret )
        wake_up_interruptible( &button_queue_kwq );
}

/*
 * Button Event Thread Routines
 */

//Polling loop is 80 msec. Repeats are at 320ms, 160ms, 160ms, ...
#define BUTTON_THREAD_TIMEOUT   80         // 80 ms
static	volatile int		button_thread_leaving = 0;
static  wait_queue_head_t	button_thread_wq;
static  spinlock_t		button_thread_lock;
static	DECLARE_MUTEX_LOCKED(button_thread_exit_sem);

static
int button_device_thread(void *dummy)
{
    struct task_struct *tsk = current;

    tsk->rt_priority = 32;        // rr values range from 0 - 99 larger is more prio
    tsk->policy = SCHED_RR;        // say were real time scheduling wise
    //tsk->session = 1;
    //tsk->pgrp = 1;
    /* we might get involved when memory gets low, so use PF_MEMALLOC */
    tsk->flags |= PF_MEMALLOC;

    daemonize("buttond");
/* replaced with daemonize() call
    strcpy(tsk->comm, "buttond");
    tsk->tty = NULL;
    spin_lock_irq(&tsk->sigmask_lock);
    sigfillset(&tsk->blocked);
    recalc_sigpending(tsk);
    spin_unlock_irq(&tsk->sigmask_lock);
    exit_mm(tsk);
    exit_files(tsk);
    exit_sighand(tsk);
    exit_fs(tsk);
*/

    for( ;; ) {
        interruptible_sleep_on_timeout( &button_thread_wq,
                                        BUTTON_THREAD_TIMEOUT );
        if( button_thread_leaving )
            break;
        spin_lock( &button_thread_lock );
        button_poll();
        spin_unlock( &button_thread_lock );
    }

    up(&button_thread_exit_sem);
    return 0;
}

static int
button_thread_init(void)
{
    spin_lock_init(&button_thread_lock);

    button_defer_poll = 0;
    button_thread_leaving = 0;
    init_waitqueue_head( &button_thread_wq );
    kernel_thread (button_device_thread, NULL, CLONE_FS|CLONE_FILES|CLONE_SIGHAND);
    return( 0 );
}

static int
button_thread_cleanup(void)
{
    button_defer_poll = 1;
    button_thread_leaving = 1;
    wake_up( &button_thread_wq );
    down( &button_thread_exit_sem );	// wait for button thread to exit
    return( 0 );
}

static int
sonos_button_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char request[16];

	if (count > sizeof(request)) {
		return -EIO;
	}
	if (copy_from_user(request, buffer, count)) {
		return -EFAULT;
	}
	request[count] = '\0';

	button_sim_process_cmd(request);

	return count;
}

int
sonos_button_proc_init(void)
{
	// Procfs file
	sonos_button_proc_file = create_proc_entry("driver/button", S_IWUSR, 0);
	if(!sonos_button_proc_file) {
		return( -EIO );
	}
	sonos_button_proc_file->write_proc = sonos_button_proc_write;
	return( 0 );
}

void
sonos_button_proc_remove(void)
{
	remove_proc_entry("driver/button", sonos_button_proc_file);
}

/*
 *  Button Linux Driver
 */
static ssize_t
sonos_button_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    int ret = 0;

    ret = button_event_receive(button_queue, buf);
    return( ret );
}

static ssize_t
sonos_button_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    return -EINVAL;
}


static int
sonos_button_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	  unsigned long arg)
{
    int ret;
    int val;

    switch(cmd) {
//	case AUDIOCTL_GET_REPEAT_TIME:
//	    printk("AUDIOCTL_GET_REPEAT_TIME -- NOT SUPPORTED\n");
//            break;
//	case AUDIOCTL_SET_REPEAT_TIME:
//	    printk("AUDIOCTL_SET_REPEAT_TIME -- NOT SUPPORTED\n");
//            break;
	case AUDIOCTL_GET_BUTTON_STATE:
	    {
	        int src;
	        enum EventInfo kernel_state[EVTSOURCE_NUM_SOURCES];

	        if (_IOC_SIZE(cmd) < sizeof(kernel_state)) {
	            return -EINVAL;
	        }

	        /* initialize all to none, sources that apply are filled in */
	        for (src = EVTSOURCE_NO_SOURCE; src < EVTSOURCE_NUM_SOURCES; src++) {
	            kernel_state[src] = EVTINFO_NO_EVENT;
	        }

	        /* Return the current HW state of the button */
		val = ar7240_reg_rd(AR7240_GPIO_IN) & button_mask;
		kernel_state[EVTSOURCE_BUTTON0] = kernel_state[EVTSOURCE_BUTTON1] = kernel_state[EVTSOURCE_BUTTON2] = EVTINFO_RELEASED;
                if ((val & BUTTON_HOUSEHOLD_MASK) == 0)
			kernel_state[EVTSOURCE_BUTTON0] = EVTINFO_PRESSED;
                if ((val & BUTTON_VOLUMEUP_MASK) == 0)
			kernel_state[EVTSOURCE_BUTTON1] = EVTINFO_PRESSED;
                if ((val & BUTTON_VOLUMEDOWN_MASK) == 0)
			kernel_state[EVTSOURCE_BUTTON2] = EVTINFO_PRESSED;

                if(copy_to_user((void *)arg, (char *)kernel_state, sizeof(kernel_state))) {
                    return( -EFAULT );
	        }
                ret = sizeof(kernel_state);
	    }
            break;
	default:
		return -EINVAL;
    }
    return ret;
}

unsigned int
sonos_button_poll( struct file *fp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;

    poll_wait( fp,&button_queue_kwq,pt );
    if (!button_event_queue_empty(button_queue)) {
        mask = POLLIN | POLLRDNORM;
    }
    return( mask );
}

static int
sonos_button_open(struct inode *inode, struct file *file)
{
    button_defer_poll = 0;
    button_event_queue_flush(button_queue);
    return 0;
}

static int
sonos_button_close(struct inode *inode, struct file *file)
{
    button_defer_poll = 1;
    button_event_queue_flush(button_queue);
    return 0;
}

#ifdef CONFIG_PROC_FS

static struct proc_dir_entry   *proc_button;
static int
button_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
   int len;

   len = sprintf(page, "gpio-in %08x, mask %08x, gpio-masked %08x, timeout %d\n",
                 ar7240_reg_rd(AR7240_GPIO_IN),
                 button_mask,
                 ar7240_reg_rd(AR7240_GPIO_IN) & button_mask,
                 BUTTON_THREAD_TIMEOUT);

   if (len <= off+count) *eof = 1;
   *start = page + off;
   len   -= off;
   if (len>count) len = count;
   if (len<0) len = 0;
   return len;
}
#endif

/*
 *	Kernel Interfaces
 */

static struct file_operations sonos_button_fops = {
	owner:   THIS_MODULE,
	read:	 sonos_button_read,
	write:   sonos_button_write,
	ioctl:   sonos_button_ioctl,
	open:    sonos_button_open,
	release: sonos_button_close,
   poll:    sonos_button_poll,
};

static struct miscdevice sonos_button_miscdev = {
   BUTTON_MINOR,
	"buttons",
	&sonos_button_fops
};

static int __init
sonos_button_init(void)
{
    /* set up the button press ring */
    spin_lock_init(&button_queue_lock);
    init_waitqueue_head(&button_queue_kwq);
    button_queue = button_event_queue_create(16, &button_queue_kwq, &button_queue_lock);
    button_sim_init(button_queue, button_sim_matches);

    button_thread_init();

    // init button gpio pins as inputs
    ar7240_gpio_config_input(GPIO_BUTTON_HOUSEHOLD);
    ar7240_gpio_config_input(GPIO_BUTTON_VOLUME_UP);
    ar7240_gpio_config_input(GPIO_BUTTON_VOLUME_DOWN);

    // disable gpio button interrupt
    ar7240_gpio_intr_disable(GPIO_BUTTON_HOUSEHOLD);
    ar7240_gpio_intr_disable(GPIO_BUTTON_VOLUME_UP);
    ar7240_gpio_intr_disable(GPIO_BUTTON_VOLUME_DOWN);

    last_button = 0x00; // init this to be the mask for all keys unpressed

    printk (KERN_INFO "Sonos Button driver Version: %s\n",VERSION_BUTTONS);
    if (misc_register (&sonos_button_miscdev)) {
            printk (KERN_WARNING "Sonos_Button: Couldn't register device 10, "
                            "%d.\n", sonos_button_miscdev.minor);
            return -EBUSY;
    }

#ifdef CONFIG_PROC_FS
    if ((proc_button = create_proc_entry("button-gpio", 0644, NULL)) == NULL) {
            printk(KERN_ERR "%s: unable to create proc entry", "Sonos button driver");
            return -EINVAL;
    }
    proc_button->read_proc = button_read_proc;
    proc_button->write_proc = NULL;
#endif /* CONFIG_PROC_FS */

    if (sonos_button_proc_init()) {
            return -EIO;
    }

    return 0;
}

static void __exit
sonos_button_exit(void)
{
    button_thread_cleanup();
    misc_deregister( &sonos_button_miscdev );
#ifdef CONFIG_PROC_FS
    if( proc_button )
        remove_proc_entry("button-gpio", NULL);
#endif
    sonos_button_proc_remove();
}

module_init(sonos_button_init);
module_exit(sonos_button_exit);

MODULE_AUTHOR("Sonos Inc.");
MODULE_DESCRIPTION("Support for Sonos Dock Buttons");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Sonos");
#endif
