/*
 *	Real Time Clock interface for Linux on Monahans.
 *
 *	Copyright (c) 2000 Nils Faerber
 *
 *	Based on rtc.c by Paul Gortmaker
 *	Date/time conversion routines taken from arch/arm/kernel/time.c
 *			by Linus Torvalds and Russel King
 *		and the GNU C Library
 *	( ... I love the GPL ... just take what you need! ;)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *
 *	1.00	2001-06-08	Nicolas Pitre <nico@cam.org>
 *	- added periodic timer capability using OSMR1
 *	- flag compatibility with other RTC chips
 *	- permission checks for ioctls
 *	- major cleanup, partial rewrite
 *
 *	0.03	2001-03-07	CIH <cih@coventive.com>
 *	- Modify the bug setups RTC clock.
 *
 *	0.02	2001-02-27	Nils Faerber <nils@@kernelconcepts.de>
 *	- removed mktime(), added alarm irq clear
 *
 *	0.01	2000-10-01	Nils Faerber <nils@@kernelconcepts.de>
 *	- initial release

 *(C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <asm/arch/pxa-regs.h>
#include <asm/hardware.h>
#include <asm/mach/time.h>
#include <asm/rtc.h>
#include <asm/arch/pxa3xx_rtc.h>


/*
 * Periodic Interrupt can be implemented by two ways. One is using OS Timer,
 * the other is using RTC Periodic Interrupt Hardware. The default resolution
 * is using OS Timer. It's determined by macro SOFT_IRQP.
 * Because RTC Periodic Interrupt Hardware is used by system suspend/resume.
 */
#define SOFT_IRQP		1
#define TIMER_FREQ		3250000

static DEFINE_SPINLOCK(pxa_rtc_lock);

static unsigned long rtc_freq = 1000;
static unsigned long rtc_epoch = 1900;

static irqreturn_t pxa_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned int rtsr;
	unsigned long num = 0, events = RTC_IRQF;

	spin_lock(&pxa_rtc_lock);

	rtsr = RTSR;

	/* clear interrupt sources */
	RTSR = 0;
	RTSR = (RTSR_AL | RTSR_HZ | RTSR_RDAL1 | RTSR_RDAL2 | RTSR_SWAL1 | 
		RTSR_SWAL2 | RTSR_PIAL);

	/* clear alarm interrupt if it has occurred */
#ifdef	CONFIG_PXA_RTC_WRISTWATCH
	if (rtsr & (RTSR_RDAL1)) {
		rtsr &= ~(RTSR_RDALE1 | RTSR_RDALE2);
		num++;
		events |= RTC_AF;
	}
#else
	if (rtsr & RTSR_AL) {
		rtsr &= ~RTSR_ALE;
		num++;
		events |= RTC_AF;
	}
#endif
	if (rtsr & RTSR_HZ) {
		/* rtsr &= ~RTSR_HZE; */
		num++;
		events |= RTC_UF;
	}
	if (rtsr & RTSR_SWAL1) {
		rtsr &= ~RTSR_SWALE1;
		num++;
		events |= RTC_SWF;
	}
	if (rtsr & RTSR_SWAL2) {
		rtsr &= ~RTSR_SWALE2;
		num++;
		events |= RTC_SWF;
	}
	if (rtsr & RTSR_PIAL) {
		if (PIAR == 0)
			rtsr &= ~RTSR_PIALE;
		num++;
		events |= RTC_PF;
	}
	RTSR = rtsr & (RTSR_ALE | RTSR_HZE | RTSR_RDALE1 | 
			RTSR_SWALE1 | RTSR_SWALE2 | RTSR_PIALE |RTSR_PICE |
			RTSR_SWCE);

/*
	printk(KERN_INFO "IRQ num:%d IRQ Events:0x%x\n", (int)num, 
		(unsigned int)events);
	printk(KERN_INFO "OSSR:0x%x\n", OSSR);
*/

	/* update irq data & counter */
	rtc_update_irq(&rtc->class_dev, num, events);

	spin_unlock(&pxa_rtc_lock);

	return IRQ_HANDLED;
}

#ifdef SOFT_IRQP
static irqreturn_t pxa_rtc_tickirq(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned long num = 0, events = RTC_IRQF;

	/*
	 * If we match for the first time, the periodic interrupt flag won't
	 * be set.  If it is, then we did wrap around (very unlikely but
	 * still possible) and compute the amount of missed periods.
	 * The match reg is updated only when the data is actually retrieved
	 * to avoid unnecessary interrupts.
	 */
	OSSR = OSSR_M1;	/* clear match on timer1 */
	OSMR1 = TIMER_FREQ/rtc_freq + OSCR;
	num++;
	events |= RTC_PF;

	/* update irq data & counter */
	rtc_update_irq(&rtc->class_dev, num, events);
	return IRQ_HANDLED;
}
#endif

#ifdef	CONFIG_PXA_RTC_WRISTWATCH
static void tm_to_wwtime(struct rtc_time *tm, unsigned int *dreg,
			unsigned int *yreg)
{
	unsigned int tmp;

	tmp = (tm->tm_mday << RTC_WW_MDAY_SHIFT) & RTC_WW_MDAY_MASK;
	tmp |= ((tm->tm_mon + 1) << RTC_WW_MON_SHIFT) & RTC_WW_MON_MASK;
	tmp |= ((tm->tm_year + 1900) << RTC_WW_YEAR_SHIFT) & RTC_WW_YEAR_MASK;
	*yreg = tmp;

	tmp = (tm->tm_sec << RTC_WW_SEC_SHIFT) & RTC_WW_SEC_MASK;
	tmp |= (tm->tm_min << RTC_WW_MIN_SHIFT) & RTC_WW_MIN_MASK;
	tmp |= (tm->tm_hour << RTC_WW_HOUR_SHIFT) & RTC_WW_HOUR_MASK;
	tmp |= (tm->tm_wday << RTC_WW_WDAY_SHIFT) & RTC_WW_WDAY_MASK;
	*dreg = tmp;
}

static int wwtime_to_tm(unsigned int dreg, unsigned int yreg,
			struct rtc_time *tm)
{
	tm->tm_sec = (dreg & RTC_WW_SEC_MASK) >> RTC_WW_SEC_SHIFT;
	tm->tm_min = (dreg & RTC_WW_MIN_MASK) >> RTC_WW_MIN_SHIFT;
	tm->tm_hour = (dreg & RTC_WW_HOUR_MASK) >> RTC_WW_HOUR_SHIFT;
	tm->tm_wday = (dreg & RTC_WW_WDAY_MASK) >> RTC_WW_WDAY_SHIFT;
	tm->tm_mday = (yreg & RTC_WW_MDAY_MASK) >> RTC_WW_MDAY_SHIFT;
	tm->tm_mon = (yreg & RTC_WW_MON_MASK) >> RTC_WW_MON_SHIFT;
	tm->tm_year = (yreg & RTC_WW_YEAR_MASK) >> RTC_WW_YEAR_SHIFT;

	if (tm->tm_year < 1900)
		return -EINVAL;
	tm->tm_year -= 1900;
	tm->tm_mon -= 1;

	return 0;
}
#endif

static unsigned int tm_to_swtime(struct sw_time *tm)
{
	unsigned int swreg;

	swreg = ((tm->tm_hour << RTC_SW_HOUR_SHIFT) & RTC_SW_HOUR_MASK) |
		((tm->tm_min << RTC_SW_MIN_SHIFT) & RTC_SW_MIN_MASK) |
		((tm->tm_sec << RTC_SW_SEC_SHIFT) & RTC_SW_SEC_MASK) |
		((tm->tm_hundreth << RTC_SW_HUNDR_SHIFT) & RTC_SW_HUNDR_MASK);
	return swreg;
}

static void swtime_to_tm(unsigned int swreg, struct sw_time *tm)
{
	tm->tm_min = (swreg & RTC_SW_MIN_MASK) >> RTC_SW_MIN_SHIFT;
	tm->tm_hour = (swreg & RTC_SW_HOUR_MASK) >> RTC_SW_HOUR_SHIFT;
	tm->tm_sec = (swreg & RTC_SW_SEC_MASK) >> RTC_SW_SEC_SHIFT;
	tm->tm_hundreth = (swreg & RTC_SW_HUNDR_MASK) >> RTC_SW_HUNDR_SHIFT;
}

static int pxa_sw_get(struct sw_time *tm, unsigned long cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	int ret;
	switch (cmd) {
	case RTC_RD_SWCNT:
		swtime_to_tm(SWCR, tm);
		break;
	case RTC_RD_SWAR1:
		swtime_to_tm(SWAR1, tm);
		break;
	case RTC_RD_SWAR2:
		swtime_to_tm(SWAR2, tm);
		break;
	}
	ret = copy_to_user(uarg, tm, sizeof(struct sw_time));
	if (ret)
		ret = -EFAULT;
	return ret;
}

static int pxa_sw_set(struct sw_time *tm, unsigned long cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	unsigned int tmp;
	int ret;

	ret = copy_from_user(tm, uarg, sizeof(struct sw_time));
	if (ret == 0) {
		switch (cmd) {
		case RTC_SET_SWCNT:
			SWCR = tm_to_swtime(tm);
			tmp = SWCR;
			break;
		case RTC_SET_SWAR1:
			SWAR1 = tm_to_swtime(tm);
			tmp = SWAR1;
			break;
		case RTC_SET_SWAR2:
			SWAR2 = tm_to_swtime(tm);
			tmp = SWAR2;
			break;
		}
		udelay(50);
	}
	else
		ret = -EFAULT;
	return ret;
}

#ifdef	CONFIG_PXA_RTC_WRISTWATCH
static int pxa_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	wwtime_to_tm(RDCR, RYCR, tm);
	return 0;
}

static int pxa_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *p = NULL;
	p = &(alrm->time);
	wwtime_to_tm(RDAR1, RYAR1, p);
	return 0;
}

static int pxa_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct rtc_time *p = NULL;
	struct rtc_time tmp;

	if ((tm->tm_year > (4095 - 1900)) || (tm->tm_year < 70))
		return -EINVAL;
	memcpy(&tmp, tm, sizeof(struct rtc_time));
	p = &tmp;
	tm_to_wwtime(p, (unsigned int *)&RDCR, (unsigned int *)&RYCR);
	udelay(50);
	return 0;
}

static int pxa_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *p = NULL;
	struct rtc_time tmp, now;

	if ((alrm->time.tm_year > (4095 - 1900))
		|| ((alrm->time.tm_year < 70) && (alrm->time.tm_year != -1)))
		return -EINVAL;
	memcpy(&tmp, &(alrm->time), sizeof(struct rtc_time));
	if ((tmp.tm_year == -1) && (tmp.tm_mon == -1)) {
		/* alarm in next 24 hours */
		pxa_rtc_gettime(dev, &now);
		rtc_merge_alarm(&now, &tmp);
	}
	p = &tmp;
	tm_to_wwtime(p, (unsigned int *)&RDAR1, (unsigned int *)&RYAR1);
	udelay(50);
	return 0;
}
#else
static int pxa_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	rtc_time_to_tm(RCNR, tm);
	return 0;
}

static int pxa_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	rtc_time_to_tm(RTAR, &(alrm->time));
	return 0;
}

static int pxa_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;

	/*
	 * max_days = 0xFFFFFFFF / 86400 = 49710
	 * max_years = 49710 / 365 = 136
	 */
	if ((tm->tm_year > 135 + 70) || (tm->tm_year < 70))
		return -EINVAL;
	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0) {
		RCNR = time;
		udelay(50);
	}
	return ret;
}

static int pxa_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long *reg = (unsigned long *)&RTAR;
	struct rtc_time tmp, now;

	/*
	 * max_days = 0xFFFFFFFF / 86400 = 49710
	 * max_years = 49710 / 365 = 136
	 */
	if ((alrm->time.tm_year > 135 + 70)
		|| ((alrm->time.tm_year < 70) && (alrm->time.tm_year != -1)))
		return -EINVAL;
	memcpy(&tmp, &(alrm->time), sizeof(struct rtc_time));
	if ((alrm->time.tm_year == -1) && (alrm->time.tm_mon == -1)) {
		/* alarm in next 24 hours */
		pxa_rtc_gettime(dev, &now);
		rtc_merge_alarm(&now, &tmp);
	}
	rtc_tm_to_time(&tmp, reg);
	udelay(50);
	return 0;
}
#endif


static int pxa_rtc_open(struct device *dev)
{
	int ret;

	/* find the IRQs */
	ret = request_irq(IRQ_RTC1Hz, pxa_rtc_interrupt, 
			SA_INTERRUPT, "RTC 1Hz", dev);
	if (ret) {
		printk(KERN_ERR "RTC:IRQ %d already in use.\n", IRQ_RTC1Hz);
		goto IRQ_RTC1Hz_failed;
	}
	ret = request_irq(IRQ_RTCAlrm, pxa_rtc_interrupt, 
			SA_INTERRUPT, "RTC Alrm", dev);
	if (ret) {
		printk(KERN_ERR "RTC:IRQ %d already in use.\n", IRQ_RTCAlrm);
		goto IRQ_RTCAlrm_failed;
	}
#ifdef SOFT_IRQP
	ret = request_irq (IRQ_OST1, pxa_rtc_tickirq, SA_INTERRUPT, "rtc timer", dev);
	if (ret) {
		printk(KERN_ERR "rtc: IRQ %d already in use.\n", IRQ_OST1);
		goto IRQ_OST1_failed;
	}
#endif
	return 0;

#ifdef SOFT_IRQP
IRQ_OST1_failed:
	free_irq (IRQ_RTCAlrm, NULL);
#endif
IRQ_RTCAlrm_failed:
	free_irq(IRQ_RTC1Hz, NULL);
IRQ_RTC1Hz_failed:
	return -EBUSY;
}

static void pxa_rtc_release(struct device *dev)
{
	spin_lock_irq(&pxa_rtc_lock);
	RTSR = 0;
#ifdef SOFT_IRQP
	OIER &= ~OIER_E1;
	OSSR = OSSR_M1;
#endif
	spin_unlock_irq(&pxa_rtc_lock);

#ifdef SOFT_IRQP
	free_irq (IRQ_OST1, dev);
#endif
	free_irq(IRQ_RTCAlrm, dev);
	free_irq(IRQ_RTC1Hz, dev);
}

static int pxa_rtc_ioctl(struct device *dev, unsigned int cmd,
			unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct sw_time	sw_tm;
	unsigned int tmp;
	unsigned long reg;
	int ret = 0;

	spin_lock_irq(&pxa_rtc_lock);
	switch (cmd) {
	case RTC_IRQP_READ:
		ret = copy_to_user(uarg, &rtc_freq, sizeof(unsigned long));
		break;
#ifdef SOFT_IRQP
	case RTC_IRQP_SET:
		ret = copy_from_user(&reg, uarg, sizeof(unsigned long));
		if (ret == 0) {
			if (reg < 1 || reg > TIMER_FREQ) {
				ret = -EINVAL;
				break;
			}
			if ((reg > 64) && (!capable(CAP_SYS_RESOURCE))) {
				ret = -EACCES;
				break;
			}
		}
		rtc_freq = reg;
		break;
	case RTC_PIE_OFF:
		OIER &= ~OIER_E1;
		break;
	case RTC_PIE_ON:
		if ((rtc_freq > 64) && !capable(CAP_SYS_RESOURCE)) {
			ret = -EACCES;
			break;
		}
		/* set the time interval */
		OSMR1 = TIMER_FREQ / rtc_freq + OSCR;
		OIER |= OIER_E1;
		break;
#else
	case RTC_IRQP_SET:
		ret = copy_from_user(&reg, uarg, sizeof(unsigned long));
		if (ret == 0) {
			if (reg < 1 || reg > 1000) {
				ret = -EINVAL;
				break;
			}
			if ((reg > 64) && (!capable(CAP_SYS_RESOURCE))) {
				ret = -EACCES;
				break;
			}
		}
		rtc_freq = reg;
		break;
	case RTC_PIE_OFF:
		RTSR &= ~(RTSR_PICE | RTSR_PIALE);
		break;
	case RTC_PIE_ON:
		if (rtc_freq == 0) {
			printk(KERN_WARNING "Periodic Frequency can't be 0Hz\n");
			ret = -EINVAL;
			break;
		}
		if (rtc_freq > 1000) {
			printk(KERN_WARNING "Periodic Frequency can't beyond 1KHz\n");
			ret = -EINVAL;
			break;
		}
		PIAR = 1000 / rtc_freq;
		udelay(10);
		RTSR |= (RTSR_PICE | RTSR_PIALE);
		break;
#endif
	case RTC_RD_SWCNT:
	case RTC_RD_SWAR1:
	case RTC_RD_SWAR2:
		ret = pxa_sw_get(&sw_tm, cmd, arg);
		break;
	case RTC_SET_SWCNT:
	case RTC_SET_SWAR1:
	case RTC_SET_SWAR2:
		ret = pxa_sw_set(&sw_tm, cmd, arg);
		break;

#ifdef	CONFIG_PXA_RTC_WRISTWATCH
	case RTC_AIE_OFF:
		RTSR &= ~RTSR_RDALE1;
		break;
	case RTC_AIE_ON:
		RTSR |= RTSR_RDALE1;
		break;
#else
	case RTC_AIE_OFF:
		RTSR &= ~RTSR_ALE;
		break;
	case RTC_AIE_ON:
		RTSR |= RTSR_ALE;
		break;
#endif
	case RTC_UIE_OFF:
		RTSR &= ~RTSR_HZE;
		break;
	case RTC_UIE_ON:
		RTSR |= RTSR_HZE;
		break;
	case RTC_SWAIE1_OFF:
		RTSR &= ~(RTSR_SWCE | RTSR_SWALE1);
		break;
	case RTC_SWAIE1_ON:
		RTSR |= (RTSR_SWCE | RTSR_SWALE1);
		/* wait hundreths field of SWCR to be synchronized with 
		 * second field
		 */
		do {
			tmp = SWCR & RTC_SW_HUNDR_MASK;
		} while (tmp);
		break;
	case RTC_SWAIE2_OFF:
		RTSR &= ~(RTSR_SWCE | RTSR_SWALE2);
		break;
	case RTC_SWAIE2_ON:
		RTSR |= (RTSR_SWCE | RTSR_SWALE2);
		do {
			tmp = SWCR & RTC_SW_HUNDR_MASK;
		} while (tmp);
		break;
	case RTC_SW_PAUSE:
		RTSR &= ~RTSR_SWCE;
		break;
	case RTC_SW_RESUME:
		RTSR |= RTSR_SWCE;
		break;
	case RTC_EPOCH_SET: 
		/* 
		 * There were no RTC clocks before 1900. 
		 */ 
		ret = copy_from_user(&reg, uarg, sizeof(unsigned long));
		if (ret == 0) {
			if ((reg < 1900) || (reg > 4000)) {
				ret = -EINVAL;
				break;
			}
			if (!capable(CAP_SYS_TIME)) {
				ret = -EACCES;
				break;
			}
			rtc_epoch = reg;
			break;
		}
        case RTC_EPOCH_READ: 
                ret = copy_to_user(uarg, &rtc_epoch, sizeof(unsigned long));
		break;
	default:
		ret = -ENOIOCTLCMD;
	}
	spin_unlock_irq(&pxa_rtc_lock);
	if (ret)
		return ret;
	else
		return 0;
}



/* print out resources */
static int pxa_rtc_read_proc(struct device *dev, struct seq_file *seq)
{
	struct sw_time	tm, *p = NULL;

	p = &tm;
	memset(p, 0, sizeof(struct sw_time));
	swtime_to_tm(SWCR, p);
	seq_printf(seq, "StopWatch time\t: %02d:%02d:%02d:%02d\n",
		p->tm_hour, p->tm_min, p->tm_sec, p->tm_hundreth);
	memset(p, 0, sizeof(struct sw_time));
	swtime_to_tm(SWAR1, p);
	seq_printf(seq, "StopWatch alarm1\t: %02d:%02d:%02d:%02d\n",
		p->tm_hour, p->tm_min, p->tm_sec, p->tm_hundreth);
	memset(p, 0, sizeof(struct sw_time));
	swtime_to_tm(SWAR2, p);
	seq_printf(seq, "StopWatch alarm2\t: %02d:%02d:%02d:%02d\n",
		p->tm_hour, p->tm_min, p->tm_sec, p->tm_hundreth);

	return 0;
}


static const struct rtc_class_ops pxa_rtc_ops = {
	.open		= pxa_rtc_open,
	.release	= pxa_rtc_release,
	.ioctl		= pxa_rtc_ioctl,
	.read_time	= pxa_rtc_gettime,
	.set_time	= pxa_rtc_settime,
	.read_alarm	= pxa_rtc_getalarm,
	.set_alarm	= pxa_rtc_setalarm,
	.proc		= pxa_rtc_read_proc,
};

static int pxa_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	/* initialization */
	RTSR = 0;
	
	rtc = rtc_device_register(pdev->name, &pdev->dev, &pxa_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
	platform_set_drvdata(pdev, rtc);

	return 0;

}

static int pxa_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	if (rtc)
		rtc_device_unregister(rtc);

	return 0;
}

unsigned long read_persistent_clock(void)
{
	struct rtc_time tm;
	unsigned long x;
	pxa_rtc_gettime(0,&tm);
	rtc_tm_to_time(&tm,&x);
	return x;
}

#define pxa_rtc_suspend NULL
#define pxa_rtc_resume NULL

#if 0

#ifdef CONFIG_PM

static struct timespec pxa_rtc_delta;

static int pxa_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_time tm;
	struct timespec time;

	memset(&time, 0, sizeof(struct timespec));

	pxa_rtc_gettime(&(pdev->dev), &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	save_time_delta(&pxa_rtc_delta, &time);

	return 0;
}

static int pxa_rtc_resume(struct platform_device *pdev)
{
	struct rtc_time tm;
	struct timespec time;

	memset(&time, 0, sizeof(struct timespec));

	pxa_rtc_gettime(&(pdev->dev), &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	restore_time_delta(&pxa_rtc_delta, &time);

	return 0;
}
#else
#define pxa_rtc_suspend	NULL
#define pxa_rtc_resume	NULL
#endif
#endif /*0*/

static struct platform_driver pxa_rtc_drv = {
	.driver = {
		.name	= "pxa-rtc",
	},
	.probe		= pxa_rtc_probe,
	.remove		= pxa_rtc_remove,
	.suspend	= pxa_rtc_suspend,
	.resume		= pxa_rtc_resume,
};

static int __init pxa_rtc_init(void)
{
	int ret;
	ret = platform_driver_register(&pxa_rtc_drv);
	if (ret) {
		printk(KERN_ERR "rtc: register error.\n");
	}
	pr_info("PXA Real Time Clock driver v" DRIVER_VERSION "\n");
	return 0;
}

static void __exit pxa_rtc_exit(void)
{
	platform_driver_unregister(&pxa_rtc_drv);
}


module_init(pxa_rtc_init);
module_exit(pxa_rtc_exit);

MODULE_AUTHOR("Nils Faerber <nils@@kernelconcepts.de>");
MODULE_DESCRIPTION("PXA3xx Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
