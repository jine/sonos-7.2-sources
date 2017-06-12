#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <asm/types.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/system.h>
#include <linux/ioctl.h>

#include <atheros.h>
#include "mdp.h"
#include "sonos-buttons.h"
#include "button_event.h"
#include "sonos-leds.h"

#define ATH_FACTORY_RESET		0x89ABCDEF

#ifndef AP_RESET_GPIO
static atomic_t ath_fr_status = ATOMIC_INIT(0);
#endif
static volatile int ath_fr_opened = 0;
static wait_queue_head_t ath_fr_wq;
static u_int32_t push_time = 0;
struct timer_list os_timer_t;
#ifndef CONFIG_SONOS_FILLMORE
static int initial_led_state = 0;
#endif	// CONFIG_SONOS_FILLMORE
#ifdef POWER_ON_RLED_GPIO
#define POWER_ON_TIMEOUT            60      /* 60 * 0.5 seconds */
#define POWER_LED_BLINK_INTERVAL    500     /* microseconds */
static volatile int power_on_finish = 0;
struct timer_list power_on_timer;
static struct proc_dir_entry *power_on_proc_entry = NULL;
#endif

static int mdp_revision;

#ifdef CONFIG_SONOS_FILLMORE

struct timer_list button_timer;
spinlock_t button_queue_lock;
wait_queue_head_t button_queue_kwq;
struct button_event_queue *button_queue;

struct button_sim_match button_sim_matches[] = {
    {"join", EVTSOURCE_BUTTON0},
    {NULL, EVTSOURCE_NO_SOURCE}};

#define MAX_SIM_KEY_DURATION 15000  // 15sec
#define MS_TO_JIFFIES(x) (((x) * HZ) / 1000)
#define JIFFIES_TO_MS(x) (((x) * 1000) / HZ )

struct proc_dir_entry *sonos_button_proc_file;
struct timer_list sonos_button_proc_simulate_key_timer;
#endif	// CONFIG_SONOS_FILLMORE

// Use the AP_RESET_GPIO button code to support GPIO_BUTTON_HOUSEHOLD
// AP_RESET_GPIO=16 (GPIO_BUTTON_HOUSEHOLD) is the button on fillmore

#define frdbg printk

#define WPS_LED_OFF	1
#define WPS_LED_ON	0

#define USB_LED_OFF 1
#define USB_LED_ON 0

#define POWER_LED_OFF 1
#define POWER_LED_ON 0

#define READY_LED_OFF 1
#define READY_LED_ON 0

// FILLMORE LEDs are active low.
#define FILLMORE_LED_OFF 0
#define FILLMORE_LED_ON 1

#define SIMPLE_CONFIG_OFF	1
#define SIMPLE_CONFIG_ON	2
#define SIMPLE_CONFIG_BLINK	3
#define SIMPLE_CONFIG_FAIL  4

#define OS_TIMER_FUNC(_fn)	\
	void _fn(unsigned long timer_arg)

#define OS_GET_TIMER_ARG(_arg, _type)	\
	(_arg) = (_type)(timer_arg)

#define OS_INIT_TIMER(_osdev, _timer, _fn, _arg)	\
do {							\
	init_timer(_timer);				\
	(_timer)->function = (_fn);			\
	(_timer)->data = (unsigned long)(_arg);		\
} while (0)

#define OS_SET_TIMER(_timer, _ms)	\
	mod_timer(_timer, jiffies + ((_ms)*HZ)/1000)

#define OS_CANCEL_TIMER(_timer)		del_timer(_timer)
/*
 * GPIO interrupt stuff
 */
typedef enum {
    INT_TYPE_EDGE,
    INT_TYPE_LEVEL,
} ath_gpio_int_type_t;

typedef enum {
    INT_POL_ACTIVE_LOW,
    INT_POL_ACTIVE_HIGH,
} ath_gpio_int_pol_t;
/*
** Simple Config stuff
*/
typedef irqreturn_t (*sc_callback_t) (int, void *, void *, void *);

/*
 * Multiple Simple Config callback support
 * For multiple radio scenarios, we need to post the button push to
 * all radios at the same time.  However, there is only 1 button, so
 * we only have one set of GPIO callback pointers.
 *
 * Creating a structure that contains each callback, tagged with the
 * name of the device registering the callback.  The unregister routine
 * will need to determine which element to "unregister", so the device
 * name will have to be passed to unregister also
 */

typedef struct {
	char		*name;
	sc_callback_t	registered_cb;
	void		*cb_arg1;
	void		*cb_arg2;
} multi_callback_t;

/*
 * Specific instance of the callback structure
 */
static multi_callback_t sccallback[3];
static volatile int ignore_pushbutton = 1;
static struct proc_dir_entry *simple_config_entry = NULL;
#ifndef CONFIG_SONOS_FILLMORE
static struct proc_dir_entry *simulate_push_button_entry = NULL;
#endif
static struct proc_dir_entry *simple_config_led_entry = NULL;
#ifndef CONFIG_SONOS_FILLMORE
static int wps_led_blinking = 0;
#endif	// CONFIG_SONOS_FILLMORE

void ap_ready_led_off(int led);
void ap_ready_led_on(int led);

void fillmore_led_off(int led);
void fillmore_led_on(int led);
uint32_t fillmore_led_state(void);

void ath_gpio_config_int(int gpio,
			 ath_gpio_int_type_t type,
			 ath_gpio_int_pol_t polarity)
{
	u32 val;

	/*
	 * allow edge sensitive/rising edge too
	 */
	if (type == INT_TYPE_LEVEL) {
		/* level sensitive */
		ath_reg_rmw_set(ATH_GPIO_INT_TYPE, (1 << gpio));
	} else {
		/* edge triggered */
		val = ath_reg_rd(ATH_GPIO_INT_TYPE);
		val &= ~(1 << gpio);
		ath_reg_wr(ATH_GPIO_INT_TYPE, val);
	}

	if (polarity == INT_POL_ACTIVE_HIGH) {
		ath_reg_rmw_set(ATH_GPIO_INT_POLARITY, (1 << gpio));
	} else {
		val = ath_reg_rd(ATH_GPIO_INT_POLARITY);
		val &= ~(1 << gpio);
		ath_reg_wr(ATH_GPIO_INT_POLARITY, val);
	}

	ath_reg_rmw_set(ATH_GPIO_INT_ENABLE, (1 << gpio));
}

void ath_gpio_config_output(int gpio)
{
#if defined(CONFIG_MACH_AR934x) || defined(CONFIG_MACH_QCA955x)
	ath_reg_rmw_clear(ATH_GPIO_OE, (1 << gpio));
#else
	ath_reg_rmw_set(ATH_GPIO_OE, (1 << gpio));
#endif
}
EXPORT_SYMBOL(ath_gpio_config_output);

void ath_gpio_config_input(int gpio)
{
#if defined(CONFIG_MACH_AR934x) || defined(CONFIG_MACH_QCA955x)
	ath_reg_rmw_set(ATH_GPIO_OE, (1 << gpio));
#else
	ath_reg_rmw_clear(ATH_GPIO_OE, (1 << gpio));
#endif
}

void ath_gpio_out_val(int gpio, int val)
{
// The ATH_GPIO_OUT register is read only
// To set/clear bits write the bits to ATH_GPIO_SET/ATH_GPIO_CLEAR
	if (val & 0x1) {
		ath_reg_wr(ATH_GPIO_SET, (1 << gpio));
	} else {
		ath_reg_wr(ATH_GPIO_CLEAR, (1 << gpio));
	}
}
EXPORT_SYMBOL(ath_gpio_out_val);

int ath_gpio_in_val(int gpio)
{
	return ((1 << gpio) & (ath_reg_rd(ATH_GPIO_IN)));
}

static void
ath_gpio_intr_enable(unsigned int irq)
{
	ath_reg_rmw_set(ATH_GPIO_INT_MASK,
				(1 << (irq - ATH_GPIO_IRQ_BASE)));
}

static void
ath_gpio_intr_disable(unsigned int irq)
{
	ath_reg_rmw_clear(ATH_GPIO_INT_MASK,
				(1 << (irq - ATH_GPIO_IRQ_BASE)));
}

static unsigned int
ath_gpio_intr_startup(unsigned int irq)
{
	ath_gpio_intr_enable(irq);
	return 0;
}

static void
ath_gpio_intr_shutdown(unsigned int irq)
{
	ath_gpio_intr_disable(irq);
}

static void
ath_gpio_intr_ack(unsigned int irq)
{
	ath_gpio_intr_disable(irq);
}

static void
ath_gpio_intr_end(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS)))
		ath_gpio_intr_enable(irq);
}

static int
ath_gpio_intr_set_affinity(unsigned int irq, const struct cpumask *dest)
{
	/*
	 * Only 1 CPU; ignore affinity request
	 */
	return 0;
}

struct irq_chip /* hw_interrupt_type */ ath_gpio_intr_controller = {
	.name = "ATH GPIO",
	.startup = ath_gpio_intr_startup,
	.shutdown = ath_gpio_intr_shutdown,
	.enable = ath_gpio_intr_enable,
	.disable = ath_gpio_intr_disable,
	.ack = ath_gpio_intr_ack,
	.end = ath_gpio_intr_end,
	.eoi = ath_gpio_intr_end,
	.set_affinity = ath_gpio_intr_set_affinity,
};

void ath_gpio_irq_init(int irq_base)
{
	int i;

	for (i = irq_base; i < irq_base + ATH_GPIO_IRQ_COUNT; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = NULL;
		irq_desc[i].depth = 1;
		//irq_desc[i].chip = &ath_gpio_intr_controller;
		set_irq_chip_and_handler(i, &ath_gpio_intr_controller,
					 handle_percpu_irq);
	}
}

void
ath_gpio_set_fn(int gpio, int fn)
{
#define gpio_fn_reg(x)	((x) / 4)
#define gpio_lsb(x)	(((x) % 4) * 8)
#define gpio_msb(x)	(gpio_lsb(x) + 7)
#define gpio_mask(x)	(0xffu << gpio_lsb(x))
#define gpio_set(x, f)	(((f) & 0xffu) << gpio_lsb(x))

	uint32_t *reg = ((uint32_t *)GPIO_OUT_FUNCTION0_ADDRESS) +
					gpio_fn_reg(gpio);

	ath_reg_wr(reg, (ath_reg_rd(reg) & ~gpio_mask(gpio)) | gpio_set(gpio, fn));
}

int32_t register_simple_config_callback(char *cbname, void *callback, void *arg1, void *arg2)
{
	int i;
	int cbnum = sizeof(sccallback)/sizeof(multi_callback_t);

	printk("SC Callback Registration for %s\n", cbname);

	for (i = 0; i< cbnum; i++)
	{
		if (!sccallback[i].name) {
			/* SONOS(MartyC):  add one for terminating null */
			sccallback[i].name = (char*)kmalloc(strlen(cbname) + 1, GFP_KERNEL);
			strcpy(sccallback[i].name, cbname);
			sccallback[i].registered_cb = (sc_callback_t) callback;
			sccallback[i].cb_arg1 = arg1;
			sccallback[i].cb_arg2 = arg2;
			break;
		}
	}

	if (i == cbnum)
	{
		printk("@@@@ Failed SC Callback Registration for %s\n", cbname);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(register_simple_config_callback);

int32_t unregister_simple_config_callback(char *cbname)
{
	int i;
	int cbnum = sizeof(sccallback)/sizeof(multi_callback_t);

	for (i = 0; i< cbnum; i++)
	{
		if (sccallback[i].name && strcmp(sccallback[i].name, cbname) == 0) {
			kfree(sccallback[i].name);
			sccallback[i].name = NULL;
			sccallback[i].registered_cb = NULL;
			sccallback[i].cb_arg1 = NULL;
			sccallback[i].cb_arg2 = NULL;
			break;
		}
	}

	if (i == cbnum){
		printk("!&!&!&!& ERROR: Unknown callback name %s\n", cbname);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(unregister_simple_config_callback);

#ifndef CONFIG_SONOS_FILLMORE
static OS_TIMER_FUNC(wps_led_blink)
{
	void wps_led_blink;
	static int WPSled = WPS_LED_ON, sec = 0;
	ath_gpio_out_val(WPS_LED_GPIO, WPSled);
	WPSled = !WPSled;
	sec++;
	if (sec < 130) {
		OS_SET_TIMER(&os_timer_t, 1000);
	} else {
		sec = 0;
		wps_led_blinking = 0;
		OS_CANCEL_TIMER(&os_timer_t);
		ath_gpio_out_val(WPS_LED_GPIO, initial_led_state);
	}
}

static OS_TIMER_FUNC(wps_led_fail)
{
	static int WPSled = WPS_LED_ON, sec = 0;
	ath_gpio_out_val(WPS_LED_GPIO, WPSled);
	WPSled = !WPSled;
	sec++;
	if (sec < 250 * 5) {//Keep blinking for 250 seconds & timer callback kicks in every 200 ms
		OS_SET_TIMER(&os_timer_t, 200);
	} else {
		sec = 0;
		wps_led_blinking = 0;
		OS_CANCEL_TIMER(&os_timer_t);
		ath_gpio_out_val(WPS_LED_GPIO, initial_led_state);
	}
}

static OS_TIMER_FUNC(wps_led_success)
{
	wps_led_blinking = 0;
	OS_CANCEL_TIMER(&os_timer_t);
	ath_gpio_out_val(WPS_LED_GPIO, initial_led_state);
}
#endif	// CONFIG_SONOS_FILLMORE

int ath_simple_config_invoke_cb(int simplecfg_only, int irq_enable, int cpl)
{
	int i;
	int cbnum = sizeof(sccallback)/sizeof(multi_callback_t);

	printk("%s: sc %d, irq %d, ignorepb %d, jiffies %lu\n", __func__,
		simplecfg_only, irq_enable, ignore_pushbutton, jiffies);
	if (simplecfg_only) {
		if (ignore_pushbutton) {
			ath_gpio_config_int(JUMPSTART_GPIO, INT_TYPE_LEVEL,
						INT_POL_ACTIVE_HIGH);
			ignore_pushbutton = 0;
			push_time = jiffies;
			return IRQ_HANDLED;
		}

		ath_gpio_config_int(JUMPSTART_GPIO, INT_TYPE_LEVEL,
					INT_POL_ACTIVE_LOW);
		ignore_pushbutton = 1;
	}

	if (irq_enable)
		local_irq_enable();

	if (push_time) {
		push_time = jiffies - push_time;
	}
	printk ("simple_config callback.. push dur in sec %d\n", push_time/HZ);


	for (i = 0; i< cbnum; i++)
	{
		if (sccallback[i].registered_cb) {
			if (sccallback[i].cb_arg2) {
				*(u_int32_t *)sccallback[i].cb_arg2 = push_time/HZ;
			}
			sccallback[i].registered_cb (cpl, sccallback[i].cb_arg1, NULL, sccallback[i].cb_arg2);
		}

	}

	return IRQ_HANDLED;
}

/*
 * Irq for front panel SW jumpstart switch
 * Connected to XSCALE through GPIO4
 */
irqreturn_t jumpstart_irq(int cpl, void *dev_id)
{
#ifndef AP_RESET_GPIO
	unsigned int delay;
	if (atomic_read(&ath_fr_status)) {
		local_irq_disable();

#define UDELAY_COUNT 4000
		push_time = jiffies;

		for (delay = UDELAY_COUNT; delay; delay--) {
			if (ath_gpio_in_val(JUMPSTART_GPIO)) {
				break;
			}
			udelay(1000);
		}

		if (!delay) {
			atomic_dec(&ath_fr_status);
			/*
			 * since we are going to reboot the board, we
			 * don't need the interrupt handler anymore,
			 * so disable it.
			 */
			disable_irq(ATH_GPIO_IRQn(JUMPSTART_GPIO));
			wake_up(&ath_fr_wq);
			printk("\nath: Factory configuration restored..\n");
			local_irq_enable();
			return IRQ_HANDLED;
		} else {
			return (ath_simple_config_invoke_cb
				(0, 1, cpl));
		}
	} else
#endif
		return (ath_simple_config_invoke_cb(1, 0, cpl));
}

#ifdef AP_RESET_GPIO
#ifdef CONFIG_SONOS_FILLMORE
/* FILLMORE has a single button: Join HouseHold
 * When interrupt low is triggered, queue the button
 * When interrupt triggers high, queue no button
 * Higher level software tracks presses and releases
 */
#endif	// CONFIG_SONOS_FILLMORE
irqreturn_t ath_reset_irq(int cpl, void *dev_id)
{
    local_irq_disable();

    if(push_time == 0){
        ath_gpio_config_int(AP_RESET_GPIO, INT_TYPE_LEVEL,
                 INT_POL_ACTIVE_HIGH);
        push_time = jiffies;
#ifdef CONFIG_SONOS_FILLMORE
	button_event_send(button_queue, EVTSOURCE_BUTTON0, EVTINFO_PRESSED);
#endif	// CONFIG_SONOS_FILLMORE
	local_irq_enable();
        return IRQ_HANDLED;
    }else{
        ath_gpio_config_int(AP_RESET_GPIO, INT_TYPE_LEVEL,
                INT_POL_ACTIVE_LOW);
#ifdef CONFIG_SONOS_FILLMORE
        push_time = 0;
	button_event_send(button_queue, EVTSOURCE_BUTTON0, EVTINFO_RELEASED);
	local_irq_enable();
	return IRQ_HANDLED;
#else
        push_time = jiffies - push_time;
#endif	// CONFIG_SONOS_FILLMORE
    }

#ifndef CONFIG_SONOS_FILLMORE
    if(push_time/HZ > 3){
		/*
		 * since we are going to reboot the board, we
		 * don't need the interrupt handler anymore,
		 * so disable it.
		 */
		disable_irq(ATH_GPIO_IRQn(AP_RESET_GPIO));
		wake_up(&ath_fr_wq);
		printk("\nath: factory configuration restored..\n");
        push_time = 0;
		local_irq_enable();
		return IRQ_HANDLED;
	} else if(push_time/HZ < 1){
        push_time = 0;
		local_irq_enable();
		return IRQ_HANDLED;
    }else{
		extern void ath_restart(char *);
		ath_restart(NULL);
		return IRQ_HANDLED;
	}
#endif	// CONFIG_SONOS_FILLMORE
}
#endif

#ifdef CONFIG_PROC_FS
#ifndef CONFIG_SONOS_FILLMORE
static int push_button_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	return 0;
}

static int push_button_write(struct file *file, const char *buf,
				unsigned long count, void *data)
{
	int i;
	int cbnum = sizeof(sccallback)/sizeof(multi_callback_t);

	for (i = 0; i< cbnum; i++){
		if (sccallback[i].registered_cb) {
			sccallback[i].registered_cb (0, sccallback[i].cb_arg1, 0, sccallback[i].cb_arg2);
		}
	}
	return count;
}
#endif	// CONFIG_SONOS_FILLMORE

typedef enum {
	LED_STATE_OFF = 1,
	LED_STATE_ON = 2,
	LED_STATE_BLINKING = 3,
} led_state_e;

#ifndef CONFIG_SONOS_FILLMORE
static led_state_e simple_config_led_state = LED_STATE_OFF;
#endif

static int gpio_simple_config_led_read(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
#ifdef CONFIG_SONOS_FILLMORE
	int LedState = fillmore_led_state();

	return sprintf(page, "%06x %s%s%s%s\n", LedState,
		(LedState & (1<<GPIO_LED_AMBER_PIN))?"A":"a",
		(LedState & (1<<GPIO_LED_RED_PIN))?"R":"r",
		(LedState & (1<<GPIO_LED_GREEN_PIN))?"G":"g",
		(LedState & (1<<GPIO_LED_WHITE_PIN))?"W":"w");
#else
	return 0;
#endif
}

static int gpio_simple_config_led_write(struct file *file, const char *buf,
					unsigned long count, void *data)
{
#ifdef CONFIG_SONOS_FILLMORE
	char request[64];
	char *rp = request;

	if (count >= sizeof(request))
		return -EIO;
	if (copy_from_user(request, buf, count))
		return -EFAULT;
	request[count] = 0;

	while (*rp) {
		switch (*rp) {
		case 'a' :
			fillmore_led_off(GPIO_LED_AMBER_PIN);
		break;
		case 'r' :
			fillmore_led_off(GPIO_LED_RED_PIN);
		break;
		case 'g' :
			fillmore_led_off(GPIO_LED_GREEN_PIN);
		break;
		case 'w' :
			fillmore_led_off(GPIO_LED_WHITE_PIN);
		break;
		case 'A' :
			fillmore_led_on(GPIO_LED_AMBER_PIN);
		break;
		case 'R' :
			fillmore_led_on(GPIO_LED_RED_PIN);
		break;
		case 'G' :
			fillmore_led_on(GPIO_LED_GREEN_PIN);
		break;
		case 'W' :
			fillmore_led_on(GPIO_LED_WHITE_PIN);
		break;
		default:
		break;
		}
		rp++;
	}
#else	// CONFIG_SONOS_FILLMORE
	u_int32_t val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
    
    if(val == SIMPLE_CONFIG_BLINK){
        if( ath_gpio_in_val(WPS_LED_GPIO) == 0 ){
            initial_led_state = WPS_LED_ON;
        }else{ 
            initial_led_state = WPS_LED_OFF;
        }
    }

	if ((val == SIMPLE_CONFIG_BLINK) && !wps_led_blinking) { /* wps LED blinking */
		wps_led_blinking = 1;
		simple_config_led_state = SIMPLE_CONFIG_BLINK;
		ath_gpio_out_val(WPS_LED_GPIO, WPS_LED_ON);
		OS_CANCEL_TIMER(&os_timer_t);
		OS_INIT_TIMER(NULL, &os_timer_t, wps_led_blink, &os_timer_t);
		OS_SET_TIMER(&os_timer_t, 1000);
	} else if (val == SIMPLE_CONFIG_FAIL) {	/* WPS failed */
		wps_led_blinking = 0;
		simple_config_led_state = SIMPLE_CONFIG_FAIL;
		ath_gpio_out_val(WPS_LED_GPIO, WPS_LED_ON);
		OS_CANCEL_TIMER(&os_timer_t);
		OS_INIT_TIMER(NULL, &os_timer_t, wps_led_fail, &os_timer_t);
		OS_SET_TIMER(&os_timer_t, 200);
	} else if (val == SIMPLE_CONFIG_ON) {	/* WPS Success */
		wps_led_blinking = 0;
		simple_config_led_state = SIMPLE_CONFIG_ON;
		OS_CANCEL_TIMER(&os_timer_t);
		ath_gpio_out_val(WPS_LED_GPIO, WPS_LED_ON);
		OS_INIT_TIMER(NULL, &os_timer_t, wps_led_success, &os_timer_t);
		OS_SET_TIMER(&os_timer_t, 120000);
	} else if (val == SIMPLE_CONFIG_OFF) {	/* wps LED off */
		wps_led_blinking = 0;
		simple_config_led_state = SIMPLE_CONFIG_OFF;
		OS_CANCEL_TIMER(&os_timer_t);
		ath_gpio_out_val(WPS_LED_GPIO, initial_led_state);
	}
#endif	// CONFIG_SONOS_FILLMORE
	return count;
}
#endif	// CONFIG_PROC_FS

#ifndef CONFIG_SONOS_FILLMORE
void ap_usb_led_on(void)
{
#ifdef CONFIG_MACH_AR934x
#if !defined(CONFIG_I2S) && defined(AP_USB_LED_GPIO)
	unsigned int rddata;

	if (AP_USB_LED_GPIO == 4) {
		rddata = ath_reg_rd(ATH_GPIO_OUT_FUNCTION1); //87- for USB suspend
		rddata = rddata & 0xffffff00;
		rddata = rddata | ATH_GPIO_OUT_FUNCTION1_ENABLE_GPIO_4(0x0);
		ath_reg_wr(ATH_GPIO_OUT_FUNCTION1, rddata);
	} else if (AP_USB_LED_GPIO == 11) {
		rddata = ath_reg_rd(ATH_GPIO_OUT_FUNCTION2); //87- for USB suspend
		rddata = rddata & 0x00ffffff;
		rddata = rddata | ATH_GPIO_OUT_FUNCTION2_ENABLE_GPIO_11(0x0);
		ath_reg_wr(ATH_GPIO_OUT_FUNCTION2, rddata);
	}

	ath_reg_rmw_clear(ATH_GPIO_OE, (1<<AP_USB_LED_GPIO));
	ath_reg_rmw_clear(ATH_GPIO_OUT, (1<<AP_USB_LED_GPIO));
#endif
#else
#	ifdef AP_USB_LED_GPIO
	ath_gpio_config_output(AP_USB_LED_GPIO);
	ath_gpio_set_fn(AP_USB_LED_GPIO, 0);
	ath_gpio_out_val(AP_USB_LED_GPIO, USB_LED_ON);
#	endif
#endif
}

EXPORT_SYMBOL(ap_usb_led_on);
#endif	// CONFIG_SONOS_FILLMORE

#ifndef CONFIG_SONOS_FILLMORE
void ap_usb_led_off(void)
{
#ifdef CONFIG_MACH_AR934x
#if !defined(CONFIG_I2S) && defined(AP_USB_LED_GPIO)
	ath_reg_rmw_set(ATH_GPIO_OUT, (1<<AP_USB_LED_GPIO));
#endif
#else
#	ifdef AP_USB_LED_GPIO
	ath_gpio_out_val(AP_USB_LED_GPIO, USB_LED_OFF);
#	endif
#endif
}
EXPORT_SYMBOL(ap_usb_led_off);
#endif	// CONFIG_SONOS_FILLMORE

#ifdef AP_READY_LED_GPIO
void ap_ready_led_on(int led)
	ath_gpio_out_val(AP_READY_LED_GPIO, READY_LED_ON);
}
EXPORT_SYMBOL(ap_ready_led_on);
#endif

#ifdef CONFIG_SONOS_FILLMORE
void fillmore_led_on(int led)
{
	if (led == GPIO_LED_WHITE_PIN)
		ath_gpio_out_val(GPIO_LED_WHITE_PIN, FILLMORE_LED_ON);
	if (mdp_revision == MDP_REVISION_FILLMORE_P1) {
		if (led == GPIO_LED_RED_PIN)
			ath_gpio_out_val(GPIO_LED_RED_PIN, FILLMORE_LED_ON);
		if (led == GPIO_LED_GREEN_PIN)
			ath_gpio_out_val(GPIO_LED_GREEN_PIN, FILLMORE_LED_ON);
		if (led == GPIO_LED_AMBER_PIN)
			ath_gpio_out_val(GPIO_LED_AMBER_PIN, FILLMORE_LED_ON);
	}
	if (mdp_revision == MDP_REVISION_FILLMORE_BB2) {
		if (led == GPIO_LED_RED_PIN)
			ath_gpio_out_val(BB2_GPIO_LED_RED_PIN, FILLMORE_LED_OFF);
		if (led == GPIO_LED_GREEN_PIN)
			ath_gpio_out_val(BB2_GPIO_LED_GREEN_PIN, FILLMORE_LED_OFF);
		if (led == GPIO_LED_AMBER_PIN)
			ath_gpio_out_val(BB2_GPIO_LED_AMBER_PIN, FILLMORE_LED_OFF);
	}
}
EXPORT_SYMBOL(fillmore_led_on);
#endif

#ifdef AP_READY_LED_GPIO
void ap_ready_led_off(int led)
{
	ath_gpio_out_val(AP_READY_LED_GPIO, READY_LED_OFF);
}
EXPORT_SYMBOL(ap_ready_led_off);
#endif

#ifdef CONFIG_SONOS_FILLMORE
void fillmore_led_off(int led)
{
	if (led == GPIO_LED_WHITE_PIN)
		ath_gpio_out_val(GPIO_LED_WHITE_PIN, FILLMORE_LED_OFF);
	if (mdp_revision == MDP_REVISION_FILLMORE_P1) {
		if (led == GPIO_LED_RED_PIN)
			ath_gpio_out_val(GPIO_LED_RED_PIN, FILLMORE_LED_OFF);
		if (led == GPIO_LED_GREEN_PIN)
			ath_gpio_out_val(GPIO_LED_GREEN_PIN, FILLMORE_LED_OFF);
		if (led == GPIO_LED_AMBER_PIN)
			ath_gpio_out_val(GPIO_LED_AMBER_PIN, FILLMORE_LED_OFF);
		}
	if (mdp_revision == MDP_REVISION_FILLMORE_BB2) {
		if (led == GPIO_LED_RED_PIN)
			ath_gpio_out_val(BB2_GPIO_LED_RED_PIN, FILLMORE_LED_ON);
		if (led == GPIO_LED_GREEN_PIN)
			ath_gpio_out_val(BB2_GPIO_LED_GREEN_PIN, FILLMORE_LED_ON);
		if (led == GPIO_LED_AMBER_PIN)
			ath_gpio_out_val(BB2_GPIO_LED_AMBER_PIN, FILLMORE_LED_ON);
	}
}
EXPORT_SYMBOL(fillmore_led_off);

uint32_t fillmore_led_state(void)
{
	uint32_t leds = ath_reg_rd(ATH_GPIO_OUT);

	if (mdp_revision == MDP_REVISION_FILLMORE_BB2) {
		if (leds & GPIO_LED_AMBER)
			leds &= ~GPIO_LED_AMBER;
		else
			leds |= GPIO_LED_AMBER;
		if (leds & GPIO_LED_GREEN)
			leds &= ~GPIO_LED_GREEN;
		else
			leds |= GPIO_LED_GREEN;
		if (leds & GPIO_LED_RED)
			leds &= ~GPIO_LED_RED;
		else
			leds |= GPIO_LED_RED;
	}
	
	return leds;
}
EXPORT_SYMBOL(fillmore_led_state);

#endif

void ath_config_slave_mdio_gpios(void)
{
        unsigned int mask;

        /* Slave MDIO phy address setting */
        ath_reg_wr(MDIO_PHY_ADDR_ADDRESS, ATH_MDIO_PHY_ADDR);

        /* Set the Clock Divisor */
        mask = ath_reg_rd(ATH_MDIO_MAC_MII_MGMT_CFG) & ~(0xf);
        mask |= ATH_MDIO_MGMT_CFG_CLK_DIV_28;
        ath_reg_wr(ATH_MDIO_MAC_MII_MGMT_CFG, mask);

        /* Set External MDIO Multiplexing Register */
        mask = ath_reg_rd(ATH_GPIO_IN_ENABLE3) &
                      ~(GPIO_IN_ENABLE3_BOOT_EXT_MDC_MASK |
                        GPIO_IN_ENABLE3_BOOT_EXT_MDO_MASK);
        mask |= GPIO_IN_ENABLE3_BOOT_EXT_MDC_SET(ATH_MDC_GPIO_PIN) |
                GPIO_IN_ENABLE3_BOOT_EXT_MDO_SET(ATH_MDIO_GPIO_PIN);
        ath_reg_wr(ATH_GPIO_IN_ENABLE3, mask);

        /* Configure the Direction of GPIO Pins */
        ath_gpio_config_input(ATH_MDC_GPIO_PIN);
        ath_gpio_config_output(ATH_MDIO_GPIO_PIN);

        /* Configure GPIO Output function as GPIOs */
        ath_gpio_set_fn(ATH_MDIO_GPIO_PIN, ATH_MDIO_OUPUT_FUNC);
}
EXPORT_SYMBOL(ath_config_slave_mdio_gpios);

#ifdef POWER_ON_RLED_GPIO
static int power_on_finish_read(char *page, char **start, off_t off,
        int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", power_on_finish);
}

static int power_on_finish_write(struct file *file, const char *buf,
        unsigned long count, void *data)
{
	u_int32_t val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

    power_on_finish = 1;

	return count;
}

static OS_TIMER_FUNC(power_led_blink)
{
	static int power_led_status = POWER_LED_OFF, power_on_timeout = 0;

    OS_CANCEL_TIMER(&power_on_timer);

    if (power_on_finish) {
		ath_gpio_out_val(POWER_ON_GLED_GPIO, POWER_LED_ON);
    } else if (++power_on_timeout >= POWER_ON_TIMEOUT) {
        ath_gpio_out_val(POWER_ON_GLED_GPIO, POWER_LED_OFF);  
        ath_gpio_config_input(POWER_ON_GLED_GPIO);
        ath_gpio_config_output(POWER_ON_RLED_GPIO);
        ath_gpio_out_val(POWER_ON_RLED_GPIO, POWER_LED_ON);  
    } else {
		ath_gpio_out_val(POWER_ON_GLED_GPIO, power_led_status);
	    power_led_status = !power_led_status;
		OS_SET_TIMER(&power_on_timer, POWER_LED_BLINK_INTERVAL);
    }
}
#endif	// POWER_ON_RLED_GPIO

#ifdef CONFIG_PROC_FS
#ifdef CONFIG_SONOS_FILLMORE
static int sonos_button_proc_read(char *page, char **start, off_t off,
				  int count, int *eof, void *data)
{
	uint32_t val = ath_reg_rd(ATH_GPIO_IN);

	return sprintf(page,
		"Physical Button State=%s\n",
		(val & (1 << AP_RESET_GPIO)) ? "up" : "DOWN" );
}

void sonos_button_proc_simulate_key_done(unsigned long last)
{
	// Restore the keys to their pre-simulated state
	printk("Simulated key press done\n");

	button_event_send(button_queue, EVTSOURCE_BUTTON0, last ? EVTINFO_PRESSED : EVTINFO_RELEASED);
}

static inline void sonos_button_proc_simulate_key(char *duration_arg)
{
	int last = 0;
	unsigned long duration = 0;

	// Only one timer at a time
	if (timer_pending(&sonos_button_proc_simulate_key_timer)) {
		printk(KERN_WARNING "Key press simulation already in progress\n");
		return;
	}
	// Parse duration argument (empty strings and null = 0)
	if (duration_arg != NULL) {
		if (strict_strtoul(duration_arg, 10, &duration) != 0) {
			printk(KERN_ERR "Non-numerical argument '%s'\n", duration_arg);
			return;
		}
		if (duration > MAX_SIM_KEY_DURATION) {
			printk(KERN_ERR "Argument exceeds maximum (%d ms)\n", MAX_SIM_KEY_DURATION);
			return;
		}
	}

	button_event_send(button_queue, EVTSOURCE_BUTTON0, EVTINFO_PRESSED);

	// Schedule key up event based on timeout
	sonos_button_proc_simulate_key_timer.data = last;
	mod_timer(&sonos_button_proc_simulate_key_timer, (jiffies + MS_TO_JIFFIES(duration)));

	printk("Simulating key press 0x%02lx -> %s for %ld ms\n",
		sonos_button_proc_simulate_key_timer.data,
		"BUTTON0",
		JIFFIES_TO_MS(sonos_button_proc_simulate_key_timer.expires - jiffies));
}

// echo join=<msec> > button
// <msec> is the number of msec to hold the button down before releasing it.
static int sonos_button_proc_write(struct file *file, const char __user * buffer,
				   unsigned long count, void *data)
{
	char request[64];
	char *arg;

	if (count >= sizeof(request)) {
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
		sonos_button_proc_simulate_key(arg);
	}

	return count;
}
#endif // CONFIG_SONOS_FILLMORE

static int create_simple_config_led_proc_entry(void)
{
	if (simple_config_entry != NULL) {
		printk("Already have a proc entry for /proc/simple_config!\n");
		return -ENOENT;
	}

#ifndef CONFIG_SONOS_FILLMORE
	if (!simulate_push_button_entry)
		simulate_push_button_entry = create_proc_entry("driver/push_button", 0644, 0);
	if (!simulate_push_button_entry)
		return -ENOENT;

	simulate_push_button_entry->write_proc = push_button_write;
	simulate_push_button_entry->read_proc = push_button_read;
#endif // not CONFIG_SONOS_FILLMORE

	if (!simple_config_led_entry)
		simple_config_led_entry = create_proc_entry("driver/led_hw", 0644, 0);
	if (!simple_config_led_entry)
		return -ENOENT;

	simple_config_led_entry->write_proc = gpio_simple_config_led_write;
	simple_config_led_entry->read_proc = gpio_simple_config_led_read;

#ifdef CONFIG_SONOS_FILLMORE
	// Button simulation proc file and related timer
	sonos_button_proc_file = create_proc_read_entry("driver/button", (S_IRUSR|S_IWUSR), 0, sonos_button_proc_read, 0);
	if(!sonos_button_proc_file) {
		return -ENOENT;
	}
	sonos_button_proc_file->write_proc = sonos_button_proc_write;

	init_timer(&sonos_button_proc_simulate_key_timer);
	sonos_button_proc_simulate_key_timer.function = sonos_button_proc_simulate_key_done;
	sonos_button_proc_simulate_key_timer.data = 0;
#endif // CONFIG_SONOS_FILLMORE

	/* configure gpio as outputs */
	ath_gpio_config_output(WPS_LED_GPIO);

	/* switch off the led */
	ath_gpio_out_val(WPS_LED_GPIO, WPS_LED_OFF);

#ifdef POWER_ON_RLED_GPIO
	power_on_proc_entry = create_proc_entry("power_on_finish", 0644,
							simple_config_entry);
	if (!power_on_proc_entry)
		return -ENOENT;

	power_on_proc_entry->write_proc = power_on_finish_write;
	power_on_proc_entry->read_proc = power_on_finish_read;
#endif
	return 0;
}
#endif	// CONFIG_PROC_FS

/* ============================================ */

static int
athfr_open(struct inode *inode, struct file *file)
{
	if (MINOR(inode->i_rdev) != FACTORY_RESET_MINOR) {
		return -ENODEV;
	}

	if (ath_fr_opened) {
		return -EBUSY;
	}

#ifdef CONFIG_SONOS_FILLMORE
	button_event_queue_flush(button_queue);
#endif	// CONFIG_SONOS_FILLMORE

	ath_fr_opened = 1;
	return nonseekable_open(inode, file);
}

static int
athfr_close(struct inode *inode, struct file *file)
{
	if (MINOR(inode->i_rdev) != FACTORY_RESET_MINOR) {
		return -ENODEV;
	}

#ifdef CONFIG_SONOS_FILLMORE
	button_event_queue_flush(button_queue);
#endif	// CONFIG_SONOS_FILLMORE

	ath_fr_opened = 0;
	return 0;
}

static ssize_t
athfr_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
#ifdef CONFIG_SONOS_FILLMORE
    int ret = 0;
    ret = button_event_receive(button_queue, buf);
    return( ret );
#else
	return -ENOTSUPP;
#endif	// CONFIG_SONOS_FILLMORE
}

static ssize_t
athfr_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
	return -ENOTSUPP;
}

static int
athfr_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
    int ret = 0;
#ifdef CONFIG_SONOS_FILLMORE
    int val;
    switch(cmd) {
//	case AUDIOCTL_GET_REPEAT_TIME:
//	    printk("AUDIOCTL_GET_REPEAT_TIME -- NOT SUPPORTED\n");
//	    break;
//	case AUDIOCTL_SET_REPEAT_TIME:
//	    printk("AUDIOCTL_SET_REPEAT_TIME -- NOT SUPPORTED\n");
//          break;
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
                val = ath_reg_rd(ATH_GPIO_IN);
                /* Convert the GPIO mask to an API mask */
	        kernel_state[EVTSOURCE_BUTTON0] = (val & BUTTON_HOUSEHOLD_MASK) ? EVTINFO_RELEASED : EVTINFO_PRESSED;

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
			
#else	// CONFIG_SONOS_FILLMORE
	switch (cmd) {
	case ATH_FACTORY_RESET:
#ifndef AP_RESET_GPIO
		atomic_inc(&ath_fr_status);
#endif
		sleep_on(&ath_fr_wq);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
#endif	// CONFIG_SONOS_FILLMORE
}

static unsigned int
athfr_poll( struct file *fp, struct poll_table_struct *pt)
{
    unsigned int mask=0;

    poll_wait(fp,&button_queue_kwq,pt);
    if (!button_event_queue_empty(button_queue)) {
        mask = POLLIN | POLLRDNORM;
    }

    return mask;
}

static struct file_operations athfr_fops = {
	read:	athfr_read,
	write:	athfr_write,
	ioctl:	athfr_ioctl,
	open:	athfr_open,
	release:athfr_close,
	poll:	athfr_poll
};

static struct miscdevice athfr_miscdev = {
	FACTORY_RESET_MINOR,
	"Factory reset",
	&athfr_fops
};

int __init ath_simple_config_init(void)
{
#ifdef CONFIG_CUS100
	u32 mask = 0;
#endif
	int ret;
#ifdef AP_RESET_GPIO
	int req = 0;
	int req2;
#endif

	ret = misc_register(&athfr_miscdev);

	if (ret < 0) {
		printk("*** ath misc_register failed %d *** \n", ret);
		return -1;
	}

#ifdef CONFIG_SONOS_FILLMORE
	/* Default to latest revision in case the MDP is invalid */
	mdp_revision = MDP_REVISION_FILLMORE_P1;
	if ((sys_mdp.mdp_magic == MDP_MAGIC))
		mdp_revision = sys_mdp.mdp_revision;
	fillmore_led_off(GPIO_LED_RED_PIN);
	fillmore_led_off(GPIO_LED_GREEN_PIN);
	fillmore_led_off(GPIO_LED_AMBER_PIN);
	fillmore_led_on(GPIO_LED_WHITE_PIN);
	ath_gpio_set_fn(GPIO_LED_WHITE_PIN, 0);
	ath_gpio_set_fn(GPIO_LED_RED_PIN, 0);
	ath_gpio_set_fn(GPIO_LED_GREEN_PIN, 0);
	ath_gpio_set_fn(GPIO_LED_AMBER_PIN, 0);
	ath_gpio_config_output(GPIO_LED_WHITE_PIN);
	ath_gpio_config_output(GPIO_LED_RED_PIN);
	ath_gpio_config_output(GPIO_LED_GREEN_PIN);
	ath_gpio_config_output(GPIO_LED_AMBER_PIN);
	ath_gpio_config_input(GPIO_BUTTON_HOUSEHOLD);

	/* set up the button press ring */
	spin_lock_init(&button_queue_lock);
	init_waitqueue_head(&button_queue_kwq);
	button_queue = button_event_queue_create(16, &button_queue_kwq, &button_queue_lock);
	button_sim_init(button_queue, button_sim_matches);

#endif // CONFIG_SONOS_FILLMORE

#ifdef AP_RESET_GPIO
    ath_gpio_config_input(AP_RESET_GPIO);
    ath_gpio_config_int(AP_RESET_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);
#endif

#ifndef CONFIG_SONOS_FILLMORE
#ifdef CONFIG_CUS100
	mask = ath_reg_rd(ATH_MISC_INT_MASK);
	ath_reg_wr(ATH_MISC_INT_MASK, mask | (1 << 2));
	ath_gpio_config_int(JUMPSTART_GPIO, INT_TYPE_LEVEL,
				INT_POL_ACTIVE_HIGH);
	ath_gpio_intr_enable(JUMPSTART_GPIO);
	ath_gpio_config_input(JUMPSTART_GPIO);
#else
	ath_gpio_config_input(JUMPSTART_GPIO);
	/* configure Jumpstart GPIO as level triggered interrupt */
	ath_gpio_config_int(JUMPSTART_GPIO, INT_TYPE_LEVEL,
				INT_POL_ACTIVE_LOW);
	printk("%s (%s) JUMPSTART_GPIO: %d\n", __FILE__, __func__,
		JUMPSTART_GPIO);
#ifndef CONFIG_MACH_AR934x
	ath_reg_rmw_clear(ATH_GPIO_FUNCTIONS, (1 << 2));
	ath_reg_rmw_clear(ATH_GPIO_FUNCTIONS, (1 << 16));
	ath_reg_rmw_clear(ATH_GPIO_FUNCTIONS, (1 << 20));
#endif
#endif

	req = request_irq(ATH_GPIO_IRQn(JUMPSTART_GPIO), jumpstart_irq, 0,
#ifdef AP_RESET_GPIO
			"SW JUMPSTART", NULL
#else
			"SW JUMPSTART/FACTORY RESET", NULL
#endif
		);
	if (req != 0) {
		printk("request_irq for jumpstart failed (error %d)\n", req);
		misc_deregister(&athfr_miscdev);
		ath_gpio_intr_shutdown(ATH_GPIO_IRQn(JUMPSTART_GPIO));
		return -1;
	}
#endif	// CONFIG_SONOS_FILLMORE
#ifdef AP_RESET_GPIO
    req2 = request_irq(ATH_GPIO_IRQn(AP_RESET_GPIO), ath_reset_irq, 0,
            "FACTORY RESET", NULL);
    if (req2 != 0) {
        printk("request_irq for factory reset failed (error %d)\n", req);
        misc_deregister(&athfr_miscdev);
        free_irq(req, NULL);
        return -1;
    }
#endif
#if !defined(CONFIG_I2S) && defined(AP_USB_LED_GPIO)
	ath_gpio_config_output(AP_USB_LED_GPIO);
#endif

#ifdef AP_READY_LED_GPIO
    ath_gpio_config_output(AP_READY_LED_GPIO);
    ap_ready_led_on();
#endif
	init_waitqueue_head(&ath_fr_wq);

	create_simple_config_led_proc_entry();

#ifdef POWER_ON_GLED_GPIO
	printk("%s (%s) POWER_ON_GLED_GPIO: %d\n", __FILE__, __func__, POWER_ON_GLED_GPIO);
    ath_gpio_config_output(POWER_ON_GLED_GPIO);
    ath_gpio_out_val(POWER_ON_GLED_GPIO, POWER_LED_ON);
#endif

#ifdef POWER_ON_RLED_GPIO
	printk("%s (%s) POWER_ON_RLED_GPIO: %d\n", __FILE__, __func__, POWER_ON_RLED_GPIO);
    ath_gpio_config_output(POWER_ON_RLED_GPIO);
    ath_gpio_out_val(POWER_ON_RLED_GPIO, POWER_LED_OFF);
    OS_INIT_TIMER(NULL, &power_on_timer, power_led_blink, NULL);
    OS_SET_TIMER(&power_on_timer, POWER_LED_BLINK_INTERVAL);
#endif

	return 0;
}

/*
 * used late_initcall so that misc_register will succeed
 * otherwise, misc driver won't be in a initializated state
 * thereby resulting in misc_register api to fail.
 */
#if !defined(CONFIG_ATH_EMULATION)
late_initcall(ath_simple_config_init);
#endif
