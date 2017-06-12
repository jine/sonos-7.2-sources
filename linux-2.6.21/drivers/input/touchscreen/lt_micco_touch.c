/*
 *  linux/drivers/input/touchscreen/littleton.c
 *
 *  touch screen driver for Littleton Platform
 *
 *  Copyright (C) 2006, Marvell Corporation (fengwei.yin@Marvell.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>

#include <asm/irq.h>
#include <asm/hardware.h> 

#include <asm/arch/pxa-regs.h>
#include <asm/arch/ipmc.h>
#include <asm/arch/micco.h>
#include <asm/arch/pxa3xx_pmic.h>

enum
{
	TSI_EVENT_NONE = 0,
	TSI_EVENT_TOUCH = 1,
	TSI_EVENT_LIFT = 2
};

enum
{
	TSI_PEN_UNKNOW =0 ,
	TSI_PEN_DOWN = 1,
	TSI_PEN_UP = 2
};

struct lt_touch_data {
	spinlock_t			ts_lock;
	struct task_struct		*thread;
	int				suspended;
	int				pen_state;
	int				use_count; 
	wait_queue_head_t		ts_wait_queue;
	struct completion		thread_init;
	struct completion		thread_exit;
};

static struct lt_touch_data *lt_touch;
static struct input_dev *littleton_ts_input_dev;

#ifdef	CONFIG_LITTLETON_BACKLIGHT
extern void led_touch_press(void);
#endif

void lt_touch_init(void)
{
	return;
}

void lt_touch_deinit(void)
{
	return;
}

int lt_micco_touch_read(u16 *x,  u16 *y, int *pen_state)
{
	int ret;
	
	ret = micco_tsi_readxy(x, y, *pen_state);
	return ret;
}

/*
 * The callback function for micco touch.
 * Just turn on the LCD backligtht when PEN DOWN on touch.
 * Leave all the touch X,Y read to a kernel thread.
 * 
 * TODO:
 *	1. Move the backlight out of the touch driver.
 *	2. Make backlight align with kernel LCD backlight driver.
 *	3. Make user application handle the touch activity
 *	   turn on the LCD backlight.
 */
void lt_touch_interrupt(unsigned long event)
{
	if (lt_touch->use_count > 0)
		wake_up_interruptible(&lt_touch->ts_wait_queue);

#ifdef	CONFIG_LITTLETON_BACKLIGHT
	led_touch_press();		
#endif
	return 0;
}
EXPORT_SYMBOL(lt_touch_interrupt);

/*
 * The touchscreen sample reader thread
 */
static int ts_thread( void *d )
{
	struct lt_touch_data *lt_td = d;
	u16 tem_x = 0xffff;
	u16 tem_y = 0xffff;
	int ret = 0, state;
	u8 val;

	DEFINE_WAIT(ts_wait);
	/* set up thread context */

	lt_td->thread = current;	
	daemonize("lt_touch_thread");
	
	/* init is complete */
	complete(&lt_td->thread_init);

	/* touch reader loop */
	while (1) {	
		/* if the pen state is up, sleep */
		if (TSI_PEN_UP == lt_td->pen_state) {
			prepare_to_wait(&lt_touch->ts_wait_queue,
				&ts_wait, TASK_INTERRUPTIBLE);
				
			if (TSI_PEN_UP == lt_td->pen_state) {
				schedule();
			}
			finish_wait(&lt_td->ts_wait_queue, &ts_wait);
		}

		try_to_freeze();

		/* Now the pen state is down */
		ret = micco_read(MICCO_STATUS_A, &val); 

		if (val & 0x40) {	/* pen down */
			if (TSI_PEN_UP ==  lt_touch->pen_state) {
				lt_touch->pen_state = TSI_PEN_DOWN;
				pr_debug("%s: touch pen down!", __func__);
			}

			/* Enable the auto measure of the TSI.
			 * This will disable pen down interupt automatically.
			 */
			micco_tsi_enable_tsi(1);
			mdelay(1);
			micco_read(MICCO_EVENT_C, &val);
			while (!(val & 0x20)) {
				pr_debug("%s: micco c val = 0x%x",
					__func__, val);
				micco_read(MICCO_EVENT_C, &val);
				mdelay(1);
			}
			pr_debug("%s: micco c val = 0x%x when TSI ready",
					__func__, val);

			lt_micco_touch_read(&tem_x,&tem_y, &state);
			pr_debug("%s: tem_x:0x%x; tem_y:0x%x; pen_state:0x%x",
					__func__, tem_x, tem_y, state);

#ifdef CONFIG_IPM
			ipm_event_notify(IPM_EVENT_UI, IPM_EVENT_DEVICE_TSI, NULL, 0);
#endif
		}

		micco_tsi_enable_tsi(0);
		/* After disable the auto tsi, need wait a while to read the
		 * correct pen_down state. Currently, just wait for 1 ms. 
		 */

		mdelay(1);
		ret = micco_read(MICCO_STATUS_A, &val);
		pr_debug("%s: after tsi disable, before pen_down" \
				"enabled status a = 0x%x", __func__, val);

		if (val & 0x40) {
#ifdef CONFIG_LITTLETON_BACKLIGHT
			extern void led_touch_press(void);
			led_touch_press();
#endif
			/* Pen still down. The X, Y data is valid.
			 * Report it.
			 */
			input_report_abs(littleton_ts_input_dev, 
					ABS_X, tem_x & 0xfff);
			input_report_abs(littleton_ts_input_dev, 
					ABS_Y, tem_y & 0xfff);
			input_report_abs(littleton_ts_input_dev, 
					ABS_PRESSURE, 0xfff);
			msleep(8);

			/* don't enable the pen down interrupt because
			 * the pen down interrupt enabling for Micco will
			 * trigger an pen down interrupt. polling to get
			 * the next pen X, Y.
			 */
			continue;
		} else {	/* Pen is up now */
			/* Report a pen up event */	
			input_report_abs(littleton_ts_input_dev, ABS_PRESSURE, 0);
			pr_debug("%s: report pen up", __func__);
			lt_touch->pen_state = TSI_PEN_UP;
			micco_tsi_enable_pen(1);
		}

		if (!lt_td->thread)
			break;
	}

	/* Always enable the pen down detection before exit from thread */
	micco_tsi_enable_pen(1);
	complete_and_exit(&lt_td->thread_exit, 0);
	return 0;
}

static int littleton_ts_input_open(struct input_dev *idev)
{
	int ret = 0;
	unsigned long flags;

	pr_debug("%s: enter", __func__);

	if (lt_touch->suspended) {
		printk("touch has been suspended!\n");
		return -1;
	}

	spin_lock_irqsave(&lt_touch->ts_lock, flags);
	if (lt_touch->use_count++ ==0) {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);

		lt_touch_init();

		init_completion(&lt_touch->thread_init);
		ret = kernel_thread(ts_thread, lt_touch, 0);
		if (ret < 0)
			return ret;

		wait_for_completion(&lt_touch->thread_init); 
	} else {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);
	}
	
	return 0;
}

/* Kill the touchscreen thread and stop the touch digitiser. */
static void littleton_ts_input_close(struct input_dev *idev)
{
	unsigned long flags;

	pr_debug("%s: enter with use count = %d", __func__,lt_touch->use_count);

	spin_lock_irqsave(&lt_touch->ts_lock, flags);
	if (--lt_touch->use_count == 0) {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);

		pr_debug("%s: kill thread", __func__);
		/* kill thread */
		if (lt_touch->thread) {
			init_completion(&lt_touch->thread_exit);
			lt_touch->thread = NULL;
			wake_up_interruptible(&lt_touch->ts_wait_queue);
			wait_for_completion(&lt_touch->thread_exit);
		}

		lt_touch_deinit();
	} else {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);
	}
}

static int lt_ts_probe(struct platform_device *pdev)
{
	int ret;

	/* register input device */
	littleton_ts_input_dev = input_allocate_device();

      	littleton_ts_input_dev->name = "Littleton touchscreen";
	littleton_ts_input_dev->open = littleton_ts_input_open;
	littleton_ts_input_dev->close = littleton_ts_input_close;

	__set_bit(EV_ABS, littleton_ts_input_dev->evbit);
	__set_bit(ABS_X, littleton_ts_input_dev->absbit);
	__set_bit(ABS_Y, littleton_ts_input_dev->absbit);
	__set_bit(ABS_PRESSURE, littleton_ts_input_dev->absbit);

	input_register_device(littleton_ts_input_dev);

	/* The Littleton IRQ come from Micco. So we just implemate
	 * a callback IRQ function for Micco.
	 */
          
	lt_touch = kzalloc(sizeof(struct lt_touch_data), GFP_KERNEL);
	if (!lt_touch) {
		ret = -ENOMEM;
		goto lt_touch_out;
	}

	ret = pmic_callback_register(PMIC_EVENT_TOUCH, lt_touch_interrupt);
	if (ret < 0)
		goto pmic_cb_out;

	spin_lock_init(&lt_touch->ts_lock);
	init_waitqueue_head(&lt_touch->ts_wait_queue);
	lt_touch->pen_state = TSI_PEN_UP;
	lt_touch->suspended = 0;
	lt_touch->use_count = 0;

	/* Enable the touch IRQ here for lcd backlight */
	micco_enable_pen_down_irq(1);
	micco_tsi_poweron();	

	return 0;

pmic_cb_out:
	kfree(lt_touch);
lt_touch_out:
	input_unregister_device(&littleton_ts_input_dev);
	return ret;	
}

static int lt_ts_remove(struct platform_device *pdev)
{
	input_unregister_device(littleton_ts_input_dev);

	return 0;
}

#ifdef CONFIG_LITTLETON_BACKLIGHT
extern void lcd_led_on(void);
#else
static void lcd_led_on(void) {}
#endif

static int lt_ts_resume(struct platform_device *pdev)
{	
	micco_enable_pen_down_irq(1);
	micco_tsi_poweron();
	extern get_pm_state(void);
	extern void lcd_backlight_power_on(void);
	if (PM_SUSPEND_LCDREFRESH != get_pm_state())
		lcd_backlight_power_on();

	lt_touch->suspended = 0;
	return 0;
}

#ifdef	CONFIG_LITTLETON_BACKLIGHT
extern void lcd_led_dim(void);
#else
static void lcd_led_dim(void);
#endif

static int lt_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	lt_touch->suspended = 1; 

	extern get_pm_state(void);
	extern void lcd_backlight_power_off(void);
	if (PM_SUSPEND_LCDREFRESH != get_pm_state())
		lcd_backlight_power_off();
	
	micco_tsi_poweroff();
	micco_enable_pen_down_irq(0);
	return 0;
}

static struct platform_driver lt_ts_drv = {
	.driver = {
		.name 	= "lt_ts", 
	},
	.probe		= lt_ts_probe,
	.remove		= lt_ts_remove,
	.resume 	= lt_ts_resume,
	.suspend 	= lt_ts_suspend,
};

static int __init littleton_ts_init( void )
{
	return platform_driver_register(&lt_ts_drv);
}

static void __exit littleton_ts_exit( void )
{
	/* We move these codes here because we want to detect the
	 * pen down event even when touch driver is not opened.
	 */
	micco_tsi_poweroff();	
	micco_enable_pen_down_irq(0);
	pmic_callback_unregister(PMIC_EVENT_TOUCH, lt_touch_interrupt);

	platform_driver_unregister(&lt_ts_drv);
}

module_init(littleton_ts_init);
module_exit(littleton_ts_exit);

MODULE_AUTHOR("Yin, Fengwei <fengwei.yin@marvell.com>");
MODULE_DESCRIPTION("Littleton Platfrom touch screen driver");
MODULE_LICENSE("GPL");
