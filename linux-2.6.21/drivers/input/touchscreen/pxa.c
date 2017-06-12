/*
 *  linux/drivers/input/touchscreen/pxa.c
 *
 *  touch screen driver on zylonite Platform
 *
 *  Copyright (C) 2005, Intel Corporation (jingqing.xu@intel.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/hardware.h> 
#include <asm/arch/pxa-regs.h>
#include <asm/arch/ipmc.h>

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

/* ADCD */
#define TSI_SD              (0x1 << 31)
#define TSI_VAL             (0x1 << 30)
#define TSI_SV              (0x1 << 29)
#define MASK_TSI_ADC_DATA1  (0xfff << 15)
#define MASK_TSI_ADC_MUX0   (0x7 << 12)
#define MASK_TSI_ADC_DATA0  (0xfff)

/* ADCS */
#define TSI_RUN             (0x1 << 31)
#define TSI_CC              (0x1 << 30)
#define TSI_SD_EN           (0x1 << 29)
#define TSI_VAL_EN          (0x1 << 28)
#define TSI_XYM             (0x1 << 27)
/* 
 * This value too large may cause unwanted
 * pen down event after ADC conversion
 */
#define TSI_PRED            (0x5 << 20)
 
#define TSI_RESOU_6BIT      (0x1 << 19)
#define TSI_ADC_MUX         (0x7 << 16)
#define TSI_ADC_DELAY       (0xff)

/* ADCE */
#define TSI_BIAS_EN         (0x1 << 0)
#define TSI_P1              (0x1 << 7) 
#define TSI_P2              (0x1 << 8)

void tsi_pxa_init(void)
{
	u32 tem;
	ADCD = 0;
	ADCS = 0;

	/* Pen down interrput enabled.
	 * Conversion valid interrupt enabled.
	 * consectutive X and Y measurement enabled.
	 */
	ADCS = 	TSI_SD_EN | TSI_VAL_EN | TSI_XYM | TSI_PRED | TSI_ADC_DELAY;
	/* Read the registers to avoid the split transaction problem.
	 * Why need read two registers here (one should be OK)? --by yfw.
	 */
	tem = ADCD;
	tem = ADCS;
}

void tsi_pxa_poweron(int enable)
{
	u32 tem;
	if (enable) {
		ADCE &= ~TSI_BIAS_EN;		
	} else {
		ADCE |= TSI_BIAS_EN;
	}
	tem =  ADCE;	
}

int tsi_pxa_getpenevent(int *pPenEvent)
{
	u32 tem;

	if(ADCS & TSI_SD_EN){
		if(ADCD & TSI_SD){
			if(ADCD & TSI_SV){
				/* pen down */
				*pPenEvent = TSI_EVENT_TOUCH ;
			} else {
				/* pen lift */
				*pPenEvent = TSI_EVENT_LIFT;
 			}
			ADCD &= ~TSI_SD;
			/* read to avoid the spit transaction problem */
			tem = ADCD;
		} else {
			 /* pen event did not happend */
			*pPenEvent = TSI_EVENT_NONE;
		}
	} else {  /* pen down event detection is not enable */
		ADCD &= ~TSI_SD;
		/* read to avoid the spit transaction problem */
		tem = ADCD;
		return -ENXIO;
	}
	return 0;
}

int tsi_pxa_read(u16 *x, u16 *y, int *pPenState)
{
	int times=5000;

	*pPenState =(ADCD & TSI_SV) ? TSI_PEN_DOWN : TSI_PEN_UP;
	if(*pPenState == TSI_PEN_UP){
		*x =  0xffff;
		*y =  0xffff;
		return  -EIO ;
	}
	disable_irq(IRQ_TSI);
	ADCS &= ~TSI_SD_EN; /* disable the style down event detection */
	ADCD &= ~TSI_VAL;
	ADCS |= TSI_RUN; /* start the routine of sample */
	while(!(ADCD & TSI_VAL)) {
		mdelay(1);
		times--;
		if (times < 0){
			/* We will delay here 5s?? -- by yfw */
			*x = 0xffff;
			*y = 0xffff;
			
			/*
			 * FIXME: this TSI_RUN bit should be cleard
			 * automatically when synchronized to ADC clock.
			 * But it looks like the silicon doesn't clear
			 * this bit automatically. So clear it here.
			 */
			ADCS &= ~TSI_RUN;
			
			/* enable pen event dection */
			ADCS |= TSI_SD_EN;
			
			enable_irq(IRQ_TSI);
			return  -EIO;
		}
	}
	ADCD &= ~TSI_VAL;
	*x = (u16)(ADCD & MASK_TSI_ADC_DATA0);
	*y = (u16)((ADCD & MASK_TSI_ADC_DATA1) >> 15);
	/* wait for the pen detection circurit to be stable.
	 * without the delay, after the pen detection is enable
	 * unwanted interrupt may happens. 
	 */
	/* Delay 5ms here is not resonable. -- by yfw */
	mdelay(5);

	/*
	 * FIXME: this TSI_RUN bit should be cleard
	 * automatically when synchronized to ADC clock.
	 * But it looks like the silicon doesn't clear
	 * this bit automatically. So clear it here.
	 */
	ADCS &= ~TSI_RUN;

	/* enable the stylus down event dection */
	ADCS |= (TSI_SD_EN);
	
	enable_irq(IRQ_TSI);
	return 0;
}

MODULE_AUTHOR("Xu Jingqing <Jingqing.Xu@intel.com>");
MODULE_DESCRIPTION("Zylonite board touch screen driver");
MODULE_LICENSE("GPL");

/* #define TS_DEBUG */

static int use_count = 0; 
struct completion thread_init , thread_exit;
struct task_struct *thread_id = NULL  ;
static volatile int touch_suspend = 0;
spinlock_t ts_lock;

static DECLARE_WAIT_QUEUE_HEAD(ts_wait_queue);

static int pen_state = TSI_PEN_UP;

static struct input_dev zylonite_ts_input_dev;

static irqreturn_t ts_interrupt(int irq, void *ptr, struct pt_regs *regs) 
{
	
	int pen_event = TSI_EVENT_NONE;

	if (0 != tsi_pxa_getpenevent(&pen_event)){
		pr_debug("the pen event detection is not enable!");
		return IRQ_HANDLED;
	}
	
	if (TSI_EVENT_TOUCH == pen_event){
		if (TSI_PEN_UP ==  pen_state){
			pr_debug("touch!");
			pen_state = TSI_PEN_DOWN;
			wake_up_interruptible(&ts_wait_queue);
#ifdef CONFIG_IPM
			ipm_event_notify(IPM_EVENT_UI, IPM_EVENT_DEVICE_TSI, NULL, 0);
#endif
		}
	}

	if( TSI_EVENT_LIFT == pen_event){
		pr_debug("lift!");
		input_report_abs(&zylonite_ts_input_dev, ABS_PRESSURE, 0);		
	}
		
        return IRQ_HANDLED;
}

static int zb_ts_resume(struct device * dev)
{	
	pxa_set_cken(CKEN_TOUCH, 1);
	tsi_pxa_init();
	tsi_pxa_poweron(1);
	touch_suspend = 0;
	return 0;

}

static int zb_ts_suspend(struct device * dev, pm_message_t state)
{
	pxa_set_cken(CKEN_TOUCH, 0);
	tsi_pxa_poweron(0);
	touch_suspend = 1; 
	return 0;

}

static struct device_driver zb_ts_drv = {
	.bus = &platform_bus_type,
	.name = "zb_ts", 
	.resume = zb_ts_resume,
	.suspend = zb_ts_suspend,
};

/*
 * The touchscreen sample reader thread
 */
static int ts_thread( void *d )
{
	struct task_struct *tsk = current;
	u16 tem_x = 0xffff;
	u16 tem_y = 0xffff;
	int ret = 0;
	thread_id = tsk ;
	/* set up thread context */
	daemonize("ktsid");
	
	/* only want to receive SIGKILL and we will die when
	 * we receive SIGKILL.
	 */
	allow_signal(SIGKILL);

	/* init is complete */
	complete(&thread_init);

	/* touch reader loop */
	for (;;) {
		
		if (TSI_PEN_UP == pen_state) {
			DEFINE_WAIT(ts_wait);

			prepare_to_wait(&ts_wait_queue, &ts_wait, TASK_INTERRUPTIBLE);
			
			if (TSI_PEN_UP == pen_state){
				/* the pen is still up */
				schedule();
			}
			finish_wait(&ts_wait_queue, &ts_wait);
		}

		/* psuedo-signal is generated on suspend - trap it here */
		try_to_freeze();

		if(signal_pending(tsk))
			break;
			
		ret = tsi_pxa_read(&tem_x,&tem_y, &pen_state);
		
		if(0 == ret){
			static int pre_pressure =0 ;
			pr_debug("tem_x:0x%x; tem_y:0x%x; pen_state:0x%x",
					tem_x, tem_y, pen_state);
			input_report_abs(&zylonite_ts_input_dev,
				       	ABS_X, tem_x & 0xfff);
			input_report_abs(&zylonite_ts_input_dev, 
					ABS_Y, tem_y & 0xfff);
			if (pre_pressure) {
				input_report_abs(&zylonite_ts_input_dev, 
						ABS_PRESSURE, 0xfff);
				pre_pressure = 0;
			} else {
				input_report_abs(&zylonite_ts_input_dev, 
					ABS_PRESSURE, 0xff0);
				pre_pressure = 1;
			}		

			set_task_state(tsk, TASK_INTERRUPTIBLE);
			if (HZ >= 100)
				schedule_timeout(HZ/100);
			else
				schedule_timeout(1);
		}
		/* Why need 20ms delay here. -- by yfw */
		mdelay(20);
	}
		
	thread_id = NULL;
	complete_and_exit(&thread_exit, 0);

	return 0;
}

static int zylonite_ts_input_open(struct input_dev *idev)
{
	int ret = 0;
	unsigned long flags;

	if (touch_suspend) {
		printk(KERN_INFO "touch has been suspended!\n");
		return -1;
	}

	spin_lock_irqsave(&ts_lock, flags);
	if (use_count++ ==0) {
		spin_unlock_irqrestore(&ts_lock, flags);
		pxa_set_cken(CKEN_TOUCH, 1);
		tsi_pxa_init();
		tsi_pxa_poweron(1);
		init_completion(&thread_init);
		init_completion(&thread_exit);
		ret = kernel_thread(ts_thread, 0 , 0);
		if (ret >= 0)
			wait_for_completion(&thread_init); 
	} else {
		spin_unlock_irqrestore(&ts_lock, flags);
	}
	
	return 0;
}

/*
 * Kill the touchscreen thread and stop
 * the touch digitiser.
 */
static void zylonite_ts_input_close(struct input_dev *idev)
{
	unsigned long flags;

	if (touch_suspend){
		printk(KERN_INFO "touch has been suspended!\n");
		return -1;
	}

	spin_lock_irqsave(&ts_lock, flags);
	if (--use_count == 0) {
		spin_unlock_irqrestore(&ts_lock, flags);
		/* kill thread */
		init_completion(&thread_exit);
		if (thread_id) {
			send_sig(SIGKILL, thread_id, 1);
			wait_for_completion(&thread_exit);
		}

		pxa_set_cken(CKEN_TOUCH, 0);
		tsi_pxa_poweron(0);
	} else {
		spin_unlock_irqrestore(&ts_lock, flags);
	}
}

static int __init zylonite_ts_init( void )
{
        int ret;

	/* register input device */
	init_input_dev(&zylonite_ts_input_dev);
      	zylonite_ts_input_dev.name = "PXA touchscreen";
	zylonite_ts_input_dev.open = zylonite_ts_input_open;
	zylonite_ts_input_dev.close = zylonite_ts_input_close;
	__set_bit(EV_ABS, zylonite_ts_input_dev.evbit);
	__set_bit(ABS_X, zylonite_ts_input_dev.absbit);
	__set_bit(ABS_Y, zylonite_ts_input_dev.absbit);
	__set_bit(ABS_PRESSURE, zylonite_ts_input_dev.absbit);
	input_register_device(&zylonite_ts_input_dev);
	printk(KERN_INFO "input: zylonite touch screen device!\n");

	ret = request_irq (IRQ_TSI, ts_interrupt, 0, "touch screen", NULL);
	if (ret) {
		printk (KERN_CRIT "Can't register IRQ[%d] for touch"
			       "screen interface\n", IRQ_TSI);
		return -1;
	}
          
	driver_register(&zb_ts_drv);
	printk(KERN_INFO "platform bus: zylonite touch driver!\n");

	spin_lock_init(&ts_lock);
	return 0;
}

static void __exit zylonite_ts_exit( void )
{
	driver_unregister(&zb_ts_drv);
	input_unregister_device(&zylonite_ts_input_dev);
	free_irq(IRQ_TSI, NULL);
}
module_init(zylonite_ts_init);
module_exit(zylonite_ts_exit);


