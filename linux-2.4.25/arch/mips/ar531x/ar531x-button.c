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
#include <linux/timer.h>
#include <linux/ioctl.h>
#include "ar531xlnx.h"
#include "ar531x.h"
#include "ar531x-button.h"
#include "button_event.h"

#define KEYSTORE_SIZE 16
#define SONOS_BUTTON_GPIO       4

static int button_mask = (1 << SONOS_BUTTON_GPIO);
static devfs_handle_t ar531x_devfs_handle;

struct proc_dir_entry *ar531xbutton_proc_file;
struct timer_list ar531xbutton_proc_simulate_key_timer;

#define MAX_SIM_KEY_DURATION 15000  // 15sec
#define MS_TO_JIFFIES(x) (((x) * HZ) / 1000)
#define JIFFIES_TO_MS(x) (((x) * 1000) / HZ )

static int last_button;
spinlock_t button_queue_lock;
wait_queue_head_t button_queue_kwq;
struct button_event_queue *button_queue;

struct button_sim_match button_sim_matches[] = {
    {"join", EVTSOURCE_BUTTON0},
    {NULL, EVTSOURCE_NO_SOURCE}};

// join button gpio pin
#define SW_HOUSEHOLD                0x00000010U

static inline int read_button_reg(void)
{
    // We only support one button. If we have a value other than 0 in the register,
    // assign the support button value. Otherwise zero.
    return ((sysRegRead(AR5315_GPIO_DI) & button_mask) ? SW_HOUSEHOLD : 0);
}


// assumptions: called with button_sem set since we're
// going to stuff chars into the key buffer
// returns 0 if no change, 1 if we put something into the keybuf
static int
process_button(int newbutton)
{
    if( newbutton != last_button) {
	    button_event_send(button_queue, EVTSOURCE_BUTTON0, newbutton ? EVTINFO_PRESSED : EVTINFO_RELEASED);
        last_button = newbutton;
        return( 1 );
    }
    return 0;
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
    ret = process_button( read_button_reg() );
    if( ret )
        wake_up_interruptible( &button_queue_kwq );
}

/*
 * Button Event Thread Routines
 */

#define BUTTON_THREAD_TIMEOUT   10         // 100 ms
static	volatile int 		button_thread_leaving = 0;
static  wait_queue_head_t	button_thread_wq;
static  spinlock_t		button_thread_lock;
static	DECLARE_MUTEX_LOCKED(button_thread_exit_sem);

static
int button_device_thread(void *dummy)
{
    struct task_struct *tsk = current;

    tsk->rt_priority = 32;        // rr values range from 0 - 99 larger is more prio
    tsk->policy = SCHED_RR;        // say were real time scheduling wise
    tsk->session = 1;
    tsk->pgrp = 1;
    /* we might get involved when memory gets low, so use PF_MEMALLOC */
    tsk->flags |= PF_MEMALLOC;
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

/*
 *  Button Linux Driver
 */
static ssize_t
ar531xbutton_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    int ret = 0;

    ret = button_event_receive(button_queue, buf);
    return( ret );
}

static ssize_t
ar531xbutton_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    return -EINVAL;
}


static int
ar531xbutton_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	  unsigned long arg)
{
    int ret = 0;
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
		val = read_button_reg();
                /* Convert the GPIO mask to an API mask */
	        kernel_state[EVTSOURCE_BUTTON0] = val ? EVTINFO_PRESSED : EVTINFO_RELEASED;

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
ar531xbutton_poll( struct file *fp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;

    poll_wait( fp,&button_queue_kwq,pt );
    if (!button_event_queue_empty(button_queue)) {
        mask = POLLIN | POLLRDNORM;
    }
    return( mask );
}

static int
ar531xbutton_open(struct inode *inode, struct file *file)
{
    button_defer_poll = 0;
    button_event_queue_flush(button_queue);
    return 0;
}

static int
ar531xbutton_close(struct inode *inode, struct file *file)
{
    button_defer_poll = 1;
    button_event_queue_flush(button_queue);
    return 0;
}


/*
 *	Proc Interfaces
 */

void ar531xbutton_proc_simulate_key_done(unsigned long last)
{
	// Restore the keys to their pre-simulated state
	printk("Simulated key press done\n");
	process_button(last);
	button_defer_poll = 0;
}

static inline void
ar531xbutton_proc_simulate_key(char *duration_arg)
{
	int last = last_button;
	unsigned long duration = 0;

	// Only one timer at a time
	if (timer_pending(&ar531xbutton_proc_simulate_key_timer)) {
		printk(KERN_WARNING "Key press simulation already in progress\n");
		return;
	}
	// Parse duration argument (empty strings and null = 0)
	if (duration_arg != NULL) {
		duration = simple_strtoul(duration_arg, NULL, 10);
		if (duration > MAX_SIM_KEY_DURATION) {
			printk(KERN_ERR "Argument exceeds maximum (%d ms)\n", MAX_SIM_KEY_DURATION);
			return;
		}
	}

	button_defer_poll = 1; /* Don't allow button polling thread to step on our simulation. */
	process_button(SW_HOUSEHOLD);

	// Schedule key up event based on timeout
	ar531xbutton_proc_simulate_key_timer.data = last;
	mod_timer(&ar531xbutton_proc_simulate_key_timer, (jiffies + MS_TO_JIFFIES(duration)));

	printk("Simulating key press 0x%02lx -> 0x%02x for %lu ms\n",
		ar531xbutton_proc_simulate_key_timer.data,
		last_button,
		JIFFIES_TO_MS(ar531xbutton_proc_simulate_key_timer.expires - jiffies));
}

static int
ar531xbutton_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char request[16];
	char *arg;

	if (count > sizeof(request)) {
		return -EIO;
	}
	if (copy_from_user(request, buffer, count)) {
		return -EFAULT;
	}
	request[count] = '\0';

	// Eliminate multi-line input
	arg = strchr(request, '\n');
	if (arg != NULL) {
		*arg = '\0';
	}

	// Parse out any arguments from the request string (ex. 'request=arg')
	arg = strchr(request, '=');
	if (arg != NULL) {
		// This overwrites the = splitting the request string into 2 null terminated strings
		// and points arg at the beginning of the 2nd string.
		*arg = '\0';
		arg++;
	}

	// Check for key test request
	if (strcmp(request, "join") == 0) {
		ar531xbutton_proc_simulate_key(arg);
	}

	return count;
}

int
ar531xbutton_proc_init(void)
{
	// Procfs file
	ar531xbutton_proc_file = create_proc_entry("driver/button", S_IWUSR, 0);
	if(!ar531xbutton_proc_file) {
		return( -EIO );
	}
	ar531xbutton_proc_file->write_proc = ar531xbutton_proc_write;

	// Key simulation timer
	init_timer(&ar531xbutton_proc_simulate_key_timer);
	ar531xbutton_proc_simulate_key_timer.function = ar531xbutton_proc_simulate_key_done;
	ar531xbutton_proc_simulate_key_timer.data = 0;
	return( 0 );
}

void
ar531xbutton_proc_remove(void)
{
	remove_proc_entry("driver/button", ar531xbutton_proc_file);
}

/*
 *	Kernel Interfaces
 */

static struct file_operations ar531xbutton_fops = {
	owner:		THIS_MODULE,
	read:		ar531xbutton_read,
	write:		ar531xbutton_write,
	ioctl:		ar531xbutton_ioctl,
	open:		ar531xbutton_open,
	release:	ar531xbutton_close,
        poll:           ar531xbutton_poll,
};

static struct miscdevice ar531xbutton_miscdev = {
        BUTTON_MINOR,
	"buttons",
	&ar531xbutton_fops
};

static int __init
ar531xbutton_init(void)
{
    int reg;

    /* set up the button press ring */
    spin_lock_init(&button_queue_lock);
    init_waitqueue_head(&button_queue_kwq);
    button_queue = button_event_queue_create(16, &button_queue_kwq, &button_queue_lock);
    button_sim_init(button_queue, button_sim_matches);

    button_thread_init();

    // init button gpio as input
    reg = sysRegRead(AR5315_GPIO_CR);
    reg &= ~(GPIO_CR_M(SONOS_BUTTON_GPIO));
    reg |= GPIO_CR_I(SONOS_BUTTON_GPIO);
    sysRegWrite(AR5315_GPIO_CR,reg);
    sysRegRead(AR5315_GPIO_CR);

    // disable gpio button interrupt
    reg = sysRegRead(AR5315_GPIO_INT);
    reg &= ~(GPIO_INT_M(0));
    reg &= ~(GPIO_INT_LVL_M(0));
    sysRegWrite(AR5315_GPIO_INT,reg);
    sysRegRead(AR5315_GPIO_INT);

    last_button = 0x00; // init this to be the mask for all keys unpressed

    printk (KERN_INFO "AR531XButton driver Version: %s\n",VERSION_BUTTON);
#ifdef CONFIG_DEVFS_FS
    ar531x_devfs_handle = devfs_register(NULL, "buttons", DEVFS_FL_AUTO_DEVNUM,
                        0, 0, S_IFCHR | S_IRUGO | S_IWUGO, &ar531xbutton_fops, NULL);
    if( !ar531x_devfs_handle ) {
        printk(KERN_ERR "%s:  failed to register device\n", BUTTON_DRIVER_ID_STR);
        return -ENODEV;
    }
#endif
    if (misc_register (&ar531xbutton_miscdev)) {
            printk (KERN_WARNING "AR531XButton: Couldn't register device 10, "
                            "%d.\n", ar531xbutton_miscdev.minor);
            return -EBUSY;
    }
    ar531xbutton_proc_init();
    return 0;
}

static void __exit
ar531xbutton_exit(void)
{
#ifdef CONFIG_DEVFS_FS
    if( ar531x_devfs_handle ) {
        devfs_unregister( ar531x_devfs_handle );
        ar531x_devfs_handle = NULL;
    }
#endif
    button_thread_cleanup();
    misc_deregister( &ar531xbutton_miscdev );
    ar531xbutton_proc_remove();
}

module_init(ar531xbutton_init);
module_exit(ar531xbutton_exit);

MODULE_AUTHOR("Sonos Inc.");
MODULE_DESCRIPTION("Support for AP51/61 Buttons");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Sonos");
#endif
