/* ibm405lp_pm.c:	Power management for the 405LP
 *
 * Copyright (C) 2002, 2003 Bishop Brock, Hollis Blanchard, IBM Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 ****************************************************************************
 *
 * Please see/update documentation in Documentation/powerpc/405LP-sleep.txt 
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk */
#include <linux/ctype.h>	/* isdigit */

#include <linux/serial_reg.h>	/* UART_LCR, etc */
#include <linux/pm.h>
#include <linux/sysctl.h>	/* ctl_table */
#include <linux/delay.h>	/* udelay */

#include <linux/tqueue.h>	/* Task queue stuff */

#include <asm/uaccess.h>	/* get_user */
#include <asm/ibm4xx.h>		/* mtdcr */
#include <asm/time.h>		/* get_tbu/l, tb_ticks_per_jiffy */
#include <asm/machdep.h>	/* ppc_md */
#include <asm/pgtable.h>	/* ptep stuff */
#include <asm/irq.h>		/* [dis|en]able_irq() */

#include <platforms/ibm405lp.h>			/* mtdcr_interlock */
#include <platforms/ibm405lp_pm.h>		/* struct wakeup_info */


#define WATCHDOG_CYCLES		64	/* number of RTC SQW cycles */
#define PREFIX		"405LP APM: "	/* for printk */

/* eventually disable MSR:EE and MSR:CE . Currently CE is 0 kernel-wide. */
#define save_and_crit_cli	save_and_cli

/* BIN_TO_BCD is stupid; can only use it on variables */
#define BIN2BCD(val)	((((val)/10)<<4) + (val)%10)
#define BCD2BIN(val)	(((val)&15) + ((val)>>4)*10)

/* inspiration from arch/arm/mach-sa1100/pm.c */
#define SPR_SAVE(array, reg)	\
	array[SLEEP_SAVE_##reg] = mfspr(SPRN_##reg)
#define SPR_RESTORE(array, reg)	\
	mtspr(SPRN_##reg, array[SLEEP_SAVE_##reg])
#define DCR_SAVE(array, reg)	\
	array[SLEEP_SAVE_##reg] = mfdcr(DCRN_##reg)
#define DCR_RESTORE(array, reg)	\
	mtdcr(DCRN_##reg, array[SLEEP_SAVE_##reg])
#define DCRI_SAVE(array, base, reg)	\
	array[SLEEP_SAVE_##base##_##reg] = mfdcri(DCRN_##base, reg)
#define DCRI_RESTORE(array, base, reg) \
	mtdcri(DCRN_##base, reg, array[SLEEP_SAVE_##base##_##reg])

enum apm_sleep_type {
	clocksuspend,
	powerdown,
	suspend,
	hibernate,
	standby,
	failsafe_powerdown,
};

const char *sleep_type_name[] = {
	[clocksuspend] = "clock-suspend",
	[powerdown] = "power-down",
	[suspend] = "suspend",
	[hibernate] = "hibernate",
	[standby] = "standby",
	[failsafe_powerdown] = "failsafe-power-down",
};

/* from head_4xx.S */
extern unsigned long ibm405lp_wakeup_info;

/* from ibm405lp_pmasm.S */
unsigned long ibm405lp_asm_suspend(unsigned long apm0_cfg,
				   unsigned long sleep_delay,
				   struct ibm405lp_wakeup_info *wakeup_info);
unsigned long ibm405lp_pm_command(unsigned long apm0_cfg,
				  unsigned long sleep_delay);
void do_rw_debug(void);
void do_rw_debug2(void);

#ifdef CONFIG_405LP_PM_BUTTON
static void pm_button_handler(unsigned long mode);
static void __init pm_button_init(void);
#else
static inline void pm_button_handler(unsigned long mode) {}
static inline void pm_button_init(void) {}
#endif /* CONFIG_405LP_PM_BUTTON */

static int rtc_freq_hz;	/* RTC input clock, in Hz */
static int cdiv;
static enum apm_sleep_type sleep_mode = suspend;
static int attempts;	/* debugging */
static int watchdog = 1;

enum {
	/* SPRs */
	SLEEP_SAVE_DAC1,
	SLEEP_SAVE_DAC2,
	SLEEP_SAVE_DBCR0,
	SLEEP_SAVE_DBCR1,
	SLEEP_SAVE_DCWR,
	SLEEP_SAVE_DVC1,
	SLEEP_SAVE_DVC2,
	SLEEP_SAVE_EVPR,
	SLEEP_SAVE_IAC1,
	SLEEP_SAVE_IAC2,
	SLEEP_SAVE_IAC3,
	SLEEP_SAVE_IAC4,
	SLEEP_SAVE_SGR,
	SLEEP_SAVE_SLER,
	SLEEP_SAVE_SPRG0,
	SLEEP_SAVE_SPRG1,
	SLEEP_SAVE_SPRG2,
	SLEEP_SAVE_SPRG3,
	SLEEP_SAVE_SPRG4,
	SLEEP_SAVE_SPRG5,
	SLEEP_SAVE_SPRG6,
	SLEEP_SAVE_SPRG7,
	SLEEP_SAVE_SRR0,
	SLEEP_SAVE_SRR1,
	SLEEP_SAVE_SRR2,
	SLEEP_SAVE_SRR3,
	SLEEP_SAVE_SU0R,

	/* DCRs */
	SLEEP_SAVE_PLB0_ACR,
	SLEEP_SAVE_CPC0_PLBAPR,
	SLEEP_SAVE_CPMER,
	SLEEP_SAVE_CPMFR,
	SLEEP_SAVE_UIC0_ER,
	SLEEP_SAVE_UIC0_CR,
	SLEEP_SAVE_UIC0_PR,
	SLEEP_SAVE_UIC0_TR,

	SLEEP_SAVE_EBC0_B0AP,
	SLEEP_SAVE_EBC0_B1AP,
	SLEEP_SAVE_EBC0_B2AP,
	SLEEP_SAVE_EBC0_B3AP,
	SLEEP_SAVE_EBC0_B4AP,
	SLEEP_SAVE_EBC0_B5AP,
	SLEEP_SAVE_EBC0_B6AP,
	SLEEP_SAVE_EBC0_B7AP,
	SLEEP_SAVE_EBC0_B0CR,
	SLEEP_SAVE_EBC0_B1CR,
	SLEEP_SAVE_EBC0_B2CR,
	SLEEP_SAVE_EBC0_B3CR,
	SLEEP_SAVE_EBC0_B4CR,
	SLEEP_SAVE_EBC0_B5CR,
	SLEEP_SAVE_EBC0_B6CR,
	SLEEP_SAVE_EBC0_B7CR,
	SLEEP_SAVE_EBC0_CFG,

	SLEEP_SAVE_RTC0_CR1,
};

#ifdef CONFIG_SERIAL_CONSOLE

#if defined(CONFIG_UART0_TTYS0)
#define CONSOLE_IO_BASE UART0_IO_BASE
#elif defined(CONFIG_UART0_TTYS1)
#define CONSOLE_IO_BASE UART1_IO_BASE
#else
#error "Unexpected configuration"
#endif /* CONFIG_UART0_TTYS0 */

/* A Hack to save/restore the UART state of the serial console device until(?)
   we have some kind of ordering mechanism for device save/restore.  The serial
   console must be saved/restored last/first to avoid losing messages during
   the suspend/resume process.  */

enum {
	Ulcr, Umcr, Uscr, Uier, Udll, Udlm
};

static u8 Uregs[6];
static void *Ubase;

static void suspend_console(void)
{
	Ubase = ioremap((long)CONSOLE_IO_BASE, 8);

	if (Ubase) {
		Uregs[Ulcr] = readb(Ubase+3);
		writeb(Uregs[Ulcr] & 0x7f, Ubase+3); /* DLAB=0 */
		Uregs[Umcr] = readb(Ubase+4);
		Uregs[Uscr] = readb(Ubase+7);
		Uregs[Uier] = readb(Ubase+1);
		writeb(Uregs[Ulcr] | 0x80, Ubase+3); /* DLAB=1 */
		Uregs[Udll] = readb(Ubase+0);
		Uregs[Udlm] = readb(Ubase+1);
		writeb(Uregs[Ulcr], Ubase+3); /* DLAB=orig */
	} else {
		printk(KERN_ERR "save_console: ioremap failed\n");
	}
}

static void resume_console(void)
{
	if (Ubase) {
		writeb(Uregs[Ulcr] & 0x7f, Ubase+3); /* DLAB=0 */
		writeb(Uregs[Umcr], Ubase+4);
		writeb(Uregs[Uscr], Ubase+7);
		writeb(Uregs[Uier], Ubase+1);
		writeb(0x07, Ubase+2);
		writeb(0x00, Ubase+2);
		(void)readb(Ubase+0);
		writeb(0x81, Ubase+2);
		writeb(Uregs[Ulcr] | 0x80, Ubase+3); /* DLAB=1 */
		writeb(Uregs[Udll], Ubase+0);
		writeb(Uregs[Udlm], Ubase+1);
		writeb(Uregs[Ulcr], Ubase+3); /* DLAB=orig */
		iounmap(Ubase);
		Ubase = NULL;
	}
}
#else  /* CONFIG_SERIAL_CONSOLE */

#define suspend_console() do {} while (0)
#define resume_console() do {} while (0)

#endif /* CONFIG_SERIAL_CONSOLE */

/* The assembly code that puts us to sleep needs to spin for at least 5
   (divided) RTC cycles after issuing the final APM0_CFG command in order to
   know that it has taken effect. */

unsigned
sleep_delay(apm0_cfg_t cfg)
{
	unsigned loops;
	unsigned cdiv_adj = ((cfg.fields.cdiv == 0) ? 1 : cfg.fields.cdiv * 2);

	loops = (5 * loops_per_jiffy * HZ * cdiv_adj) / rtc_freq_hz;
	return loops;
}

static void
save_sprs(unsigned long *sleep_save)
{
	SPR_SAVE(sleep_save, DAC1);
	SPR_SAVE(sleep_save, DAC2);
	SPR_SAVE(sleep_save, DBCR0);
	SPR_SAVE(sleep_save, DBCR1);
	SPR_SAVE(sleep_save, DCWR);
	SPR_SAVE(sleep_save, DVC1);
	SPR_SAVE(sleep_save, DVC2);
	SPR_SAVE(sleep_save, IAC1);
	SPR_SAVE(sleep_save, IAC2);
	SPR_SAVE(sleep_save, IAC3);
	SPR_SAVE(sleep_save, IAC4);
	SPR_SAVE(sleep_save, SGR);
	SPR_SAVE(sleep_save, SLER);
	SPR_SAVE(sleep_save, SPRG0);
	SPR_SAVE(sleep_save, SPRG1);
	SPR_SAVE(sleep_save, SPRG2);
	SPR_SAVE(sleep_save, SPRG3);
	SPR_SAVE(sleep_save, SPRG4);
	SPR_SAVE(sleep_save, SPRG5);
	SPR_SAVE(sleep_save, SPRG6);
	SPR_SAVE(sleep_save, SPRG7);
	SPR_SAVE(sleep_save, SRR0);
	SPR_SAVE(sleep_save, SRR1);
	SPR_SAVE(sleep_save, SRR2);
	SPR_SAVE(sleep_save, SRR3);
	SPR_SAVE(sleep_save, SU0R);
}		

static void
restore_sprs(unsigned long *sleep_save)
{
	SPR_RESTORE(sleep_save, DAC1);
	SPR_RESTORE(sleep_save, DAC2);
	SPR_RESTORE(sleep_save, DBCR0);
	SPR_RESTORE(sleep_save, DBCR1);
	SPR_RESTORE(sleep_save, DCWR);
	SPR_RESTORE(sleep_save, DVC1);
	SPR_RESTORE(sleep_save, DVC2);
	SPR_RESTORE(sleep_save, IAC1);
	SPR_RESTORE(sleep_save, IAC2);
	SPR_RESTORE(sleep_save, IAC3);
	SPR_RESTORE(sleep_save, IAC4);
	SPR_RESTORE(sleep_save, SGR);
	SPR_RESTORE(sleep_save, SLER);
	SPR_RESTORE(sleep_save, SPRG0);
	SPR_RESTORE(sleep_save, SPRG1);
	SPR_RESTORE(sleep_save, SPRG2);
	SPR_RESTORE(sleep_save, SPRG3);
	SPR_RESTORE(sleep_save, SPRG4);
	SPR_RESTORE(sleep_save, SPRG5);
	SPR_RESTORE(sleep_save, SPRG6);
	SPR_RESTORE(sleep_save, SPRG7);
	SPR_RESTORE(sleep_save, SRR0);
	SPR_RESTORE(sleep_save, SRR1);
	SPR_RESTORE(sleep_save, SRR2);
	SPR_RESTORE(sleep_save, SRR3);
	SPR_RESTORE(sleep_save, SU0R);
}	
	
static void
save_dcrs(unsigned long *sleep_save)
{
	DCRI_SAVE(sleep_save, EBC0, B0AP);
	DCRI_SAVE(sleep_save, EBC0, B1AP);
	DCRI_SAVE(sleep_save, EBC0, B2AP);
	DCRI_SAVE(sleep_save, EBC0, B3AP);
	DCRI_SAVE(sleep_save, EBC0, B4AP);
	DCRI_SAVE(sleep_save, EBC0, B5AP);
	DCRI_SAVE(sleep_save, EBC0, B6AP);
	DCRI_SAVE(sleep_save, EBC0, B7AP);
	DCRI_SAVE(sleep_save, EBC0, B0CR);
	DCRI_SAVE(sleep_save, EBC0, B1CR);
	DCRI_SAVE(sleep_save, EBC0, B2CR);
	DCRI_SAVE(sleep_save, EBC0, B3CR);
	DCRI_SAVE(sleep_save, EBC0, B4CR);
	DCRI_SAVE(sleep_save, EBC0, B5CR);
	DCRI_SAVE(sleep_save, EBC0, B6CR);
	DCRI_SAVE(sleep_save, EBC0, B7CR);
	DCRI_SAVE(sleep_save, EBC0, CFG);

	DCR_SAVE(sleep_save, PLB0_ACR);
	DCR_SAVE(sleep_save, CPC0_PLBAPR);
	DCR_SAVE(sleep_save, CPMER);
	DCR_SAVE(sleep_save, CPMFR);
	DCR_SAVE(sleep_save, UIC0_ER);
	DCR_SAVE(sleep_save, UIC0_CR);
	DCR_SAVE(sleep_save, UIC0_PR);
	DCR_SAVE(sleep_save, UIC0_TR);

	DCR_SAVE(sleep_save, RTC0_CR1);
}

static void
restore_dcrs(unsigned long *sleep_save)
{
	DCRI_RESTORE(sleep_save, EBC0, B0AP);
	DCRI_RESTORE(sleep_save, EBC0, B1AP);
	DCRI_RESTORE(sleep_save, EBC0, B2AP);
	DCRI_RESTORE(sleep_save, EBC0, B3AP);
	DCRI_RESTORE(sleep_save, EBC0, B4AP);
	DCRI_RESTORE(sleep_save, EBC0, B5AP);
	DCRI_RESTORE(sleep_save, EBC0, B6AP);
	DCRI_RESTORE(sleep_save, EBC0, B7AP);
	DCRI_RESTORE(sleep_save, EBC0, B0CR);
	DCRI_RESTORE(sleep_save, EBC0, B1CR);
	DCRI_RESTORE(sleep_save, EBC0, B2CR);
	DCRI_RESTORE(sleep_save, EBC0, B3CR);
	DCRI_RESTORE(sleep_save, EBC0, B4CR);
	DCRI_RESTORE(sleep_save, EBC0, B5CR);
	DCRI_RESTORE(sleep_save, EBC0, B6CR);
	DCRI_RESTORE(sleep_save, EBC0, B7CR);
	DCRI_RESTORE(sleep_save, EBC0, CFG);

	DCR_RESTORE(sleep_save, PLB0_ACR);
	DCR_RESTORE(sleep_save, CPC0_PLBAPR);
	DCR_RESTORE(sleep_save, CPMER);
	DCR_RESTORE(sleep_save, CPMFR);
	DCR_RESTORE(sleep_save, UIC0_ER);
	DCR_RESTORE(sleep_save, UIC0_CR);
	DCR_RESTORE(sleep_save, UIC0_PR);
	DCR_RESTORE(sleep_save, UIC0_TR);

	DCR_RESTORE(sleep_save, RTC0_CR1);

	/* Clear all pending external interrupt status, as any edge-triggered
	   status is bogus.  All true level interrupts, including the APM
	   interrupt that may have awoken the system will remain asserted. */

	mtdcr(DCRN_UIC0_SR, 0xffffffff);

	/* Restore CPM power management to its boot-time configuration.  Much
	   of this will be redone when the CPM registers are reloaded and
	   device drivers restore their states, however doing this here
	   guarantees that all obscure corners of the chip are properly power
	   managed even if they have no save/restore code. */

	ibm405lp_setup_cpm();
}

/* do_suspend()
 * Called for all modes except powerdown modes.  The clock-suspend is special
 * since it doesn't save/restore state, but it needs other things in here,
 * e.g., timebase save/restore.
 */
static int do_suspend(apm0_cfg_t apm0_cfg)
{
	u64 timebase;
	struct ibm405lp_wakeup_info my_wakeup_info = {
		.magic = IBM405LP_WAKEUP_MAGIC,
	};
	unsigned long *sleep_save;
	u32 pre_rtc_time, rtc_secs_elapsed;
	int ret;

	/* remember what time it is */
	timebase = ((u64)get_tbu() << 32) | get_tbl();
	pre_rtc_time = ppc_md.get_rtc_time();

	if (apm0_cfg.fields.sm == APM0_CFG_SM_CLOCK_SUSPEND) {
		ibm405lp_apm_dcr_delay();
		ret = ibm405lp_pm_command(apm0_cfg.reg, sleep_delay(apm0_cfg));
	} else {
		sleep_save = (unsigned long *)__get_free_page(GFP_ATOMIC); /* irqs are disabled */
		if (sleep_save == NULL) {
			printk(KERN_ERR PREFIX "__get_free_page failed!\n");
			return -ENOMEM;
		}

		/* SPRs are saved/restored by software even in standby mode.
		   In standby mode, the states of all DCRs will be preserved,
		   except for APM0_CFG and the SDRAM controller registers
		   touched by the firmware. */

		save_sprs(sleep_save);
		if (apm0_cfg.fields.sm == APM0_CFG_SM_POWER_DOWN) {
			suspend_console();
			save_dcrs(sleep_save);
		} else {
			BUG(); /* not currently supported */
		}
	
		/* the sleep call */
		ibm405lp_apm_dcr_delay();
		ret = ibm405lp_asm_suspend(apm0_cfg.reg, 
					   sleep_delay(apm0_cfg), 
					   &my_wakeup_info);
		/* wakeup() (called by the firmware on wake) returns here */

		restore_sprs(sleep_save);
		if (apm0_cfg.fields.sm == APM0_CFG_SM_POWER_DOWN) {
			restore_dcrs(sleep_save);
			resume_console();
		}

		/* free our memory */
		free_page((unsigned long)sleep_save);
	}

	/* Update timebase. We're stuck with 1-second accuracy from the RTC. */
	rtc_secs_elapsed = ppc_md.get_rtc_time() - pre_rtc_time;
	printk(KERN_DEBUG PREFIX "%u seconds elapsed\n", rtc_secs_elapsed);
	timebase += rtc_secs_elapsed * HZ * tb_ticks_per_jiffy;
	set_tb(timebase >> 32, (unsigned long)timebase);

	/* ?? Should jiffies be adjusted here ?? */

	return ret;
}

/* approximate the time required for cryo scan out + scan in */
static unsigned long calc_rtc_sqw(int cdiv, int rtc_freq)
{
	const int data_bits = 36400;    /* bits of data transferred (max 64Kbits) */
	int iic_freq;
	int transfer_ms;                /* time in ms to complete transfer */
	unsigned long rtc_rs;

	iic_freq = rtc_freq / (2 * cdiv);
	transfer_ms = 2 * data_bits * 1000 / iic_freq;
	transfer_ms = 2 * transfer_ms; /* double it to be safe */
	rtc_rs = __ilog2((transfer_ms * 0x10000) / (WATCHDOG_CYCLES * 1000));

	return rtc_rs;
}

static int ibm405lp_suspend(enum apm_sleep_type mode)
{
	apm0_cfg_t apm0_cfg;
	unsigned long flags;
	int	retval;

	printk(KERN_DEBUG PREFIX "attempt #%i\n", ++attempts);

	save_and_crit_cli(flags); /* disable critical exceptions too */

	apm0_cfg.reg = mfdcr(DCRN_APM0_CFG);
	apm0_cfg.fields.v = 0;
	apm0_cfg.fields.isp = 1;

	pm_button_handler(1);

	/* magic - do not touch */
	mtdcr_interlock(DCRN_APM0_SR, 0xffffffff, 0); /* vital: leave a 1 in SR */
	ibm405lp_apm_dcr_delay();
	/* end magic */

	printk(KERN_DEBUG PREFIX "IER = 0x%.8x\n", mfdcr(DCRN_APM0_IER));
	printk(KERN_DEBUG PREFIX "ISR = 0x%.8x\n", mfdcr(DCRN_APM0_ISR));

	switch (mode) {
		case clocksuspend:
			apm0_cfg.fields.sm = APM0_CFG_SM_CLOCK_SUSPEND;
			printk(KERN_DEBUG PREFIX "clocksuspend, CFG = 0x%.8x\n", 
			       apm0_cfg.reg);
			retval = do_suspend(apm0_cfg);
			break;

	        case failsafe_powerdown:
			mtdcr_interlock(DCRN_APM0_IER, 0, APM0_IRQ_MASK);
		case powerdown:	
			if (!(ppc_md.power_off)) {
				printk(KERN_ERR PREFIX 
				       "I don't know how to power off this"
				       "device - aborting");
				retval = -ENODEV;
			} else {
				apm0_cfg.fields.sm = APM0_CFG_SM_POWER_DOWN;
				printk(KERN_DEBUG PREFIX 
				       "%s, CFG = 0x%.8x\n",
				       sleep_type_name[mode],
				       apm0_cfg.reg);
				mtdcr(DCRN_RTC0_CEN,mfdcr(DCRN_RTC0_CEN) | IBM405LP_POWERDOWN_REBOOT);
				ibm405lp_apm_dcr_delay();
				ppc_md.power_off();
			}
			break;

		case suspend:
			apm0_cfg.fields.sm = APM0_CFG_SM_POWER_DOWN;
			printk(KERN_DEBUG PREFIX "suspend, CFG = 0x%.8x\n", apm0_cfg.reg);
			mtdcr(DCRN_RTC0_CEN,mfdcr(DCRN_RTC0_CEN) | IBM405LP_POWERDOWN_SUSPEND);
			retval = do_suspend(apm0_cfg);
			mtdcr(DCRN_RTC0_CEN,mfdcr(DCRN_RTC0_CEN) & ~IBM405LP_POWERDOWN_SUSPEND);
			break;

		case hibernate:
			apm0_cfg.fields.sm = APM0_CFG_SM_POWER_DOWN;
			printk(KERN_DEBUG PREFIX "hibernate, CFG = 0x%.8x\n", apm0_cfg.reg);
			mtdcr(DCRN_RTC0_CEN,mfdcr(DCRN_RTC0_CEN) | IBM405LP_POWERDOWN_HIBERNATE);
			retval = do_suspend(apm0_cfg);
			mtdcr(DCRN_RTC0_CEN,mfdcr(DCRN_RTC0_CEN) & ~IBM405LP_POWERDOWN_HIBERNATE);
			break;

		case standby:
			apm0_cfg.fields.sm = APM0_CFG_SM_CRYO_STANDBY;
			apm0_cfg.fields.rst = 1;

			/* set cdiv such that I2C freq is ~100 KHz */
			if (cdiv == 0) {
				if (rtc_freq_hz < 100000) {
					apm0_cfg.fields.cdiv = 0;
				} else {
					apm0_cfg.fields.cdiv = rtc_freq_hz / (100000 * 2);
					apm0_cfg.fields.cdiv++; /* round up to be safe */
				}
			} else {
				/* user override */
				apm0_cfg.fields.cdiv = cdiv;
			}

			/* enable cryo watchdog if user requested it */
			if (watchdog) {
				ibm405lp_set_rtc_sqw(calc_rtc_sqw(cdiv, rtc_freq_hz));
				apm0_cfg.fields.ewt = 1;
			}

			printk(KERN_DEBUG PREFIX "standby, CFG = 0x%.8x\n", apm0_cfg.reg);
			retval = do_suspend(apm0_cfg);
			break;
	}

	printk(KERN_DEBUG PREFIX "IER = 0x%.8x\n", mfdcr(DCRN_APM0_IER));
	printk(KERN_DEBUG PREFIX "ISR = 0x%.8x\n", mfdcr(DCRN_APM0_ISR));

	pm_button_handler(2);

	restore_flags(flags);

	return retval; /* success */
}

/* Device suspensions/resumptions are done here with interrupts enabled (?),
   before moving to the hard-core suspension code in ibm405lp_suspend. */

static int suspend_handler(enum apm_sleep_type mode)
{
	apm0_sr_t sr;
	int retval;

	/* If the APM unit is disabled any suspend commands will be ignored.
	   This is a hard disable that is strapped at APM reset - there is no
	   way for software to re-enable it. */

	sr.reg = mfdcr(DCRN_APM0_SR);
	if (!sr.fields.en) {
		printk(KERN_ERR PREFIX 
		       "The 405LP APM unit is hardware-disabled, "
		       "therefore all suspend commands are ignored\n");
		return -ENODEV;
	}

	switch (sleep_mode) {

	case powerdown:
	case failsafe_powerdown:
		/* No need to suspend anything, we're rebooting. */
		sr.reg = ibm405lp_suspend(mode);
		break;

	case standby:

		printk(KERN_WARNING PREFIX
		       "No 'standby' operating point is "
		       "defined for this platform.\n");
		printk(KERN_WARNING PREFIX
		       "Using 'clock-suspend' mode instead of "
		       "'standby'.\n");
		mode = clocksuspend;

	case clocksuspend:
		/* FIX ME: These should have a quickie clock-stop suspend, but
		   for now fall through and do the full suspend. */

	case suspend:
	case hibernate:
		/* Suspend all state to RAM and prepare for system
		   power-down. */

		/* suspend devices here */
		sr.reg = ibm405lp_suspend(mode);
		/* resume devices here */
		break;
	}
	printk(KERN_DEBUG "APM0_SR = 0x%.8x at resume\n", sr.reg);
	return 0;	/* success */
}


static int suspend_proc_handler(ctl_table *ctl, int write, struct file * filp,
				void *buffer, size_t *lenp)
{
	return suspend_handler(sleep_mode);
}

/* whose BRILLIANT idea was it to have the sysctl "strategy" routine NOT
 * called when accessed via /proc ? That renders it useless AFAICS.
 *
 * allow only valid modes to be written
 */
static int parse_mode(ctl_table *table, int write, struct file *filp,
		      void *buffer, size_t *lenp)
{
	size_t len;
	
	if (!table->maxlen || !*lenp || (filp->f_pos && !write)) {
		*lenp = 0;
		return 0;
	}

	if (write) {
		char string[table->maxlen];
		int i;

		len = *lenp;
		if (len > table->maxlen)
			len = table->maxlen;

		if (copy_from_user(string, buffer, len))
			return -EFAULT;

		for (i = 0; i < table->maxlen; i++) {
			/* Chop the string off at first nul or \n */
			if ( (i < len) && 
			     ( (string[i] == '\n') || (string[i] == '\0') ) )
				len = i;

			/* Clear the tail of the buffer */
			if (i >= len)
				string[i] = '\0';
		}

		for (i = 0; i < ARRAY_SIZE(sleep_type_name); i++) {
			const char *name = sleep_type_name[i];

			if (name && (strncmp(string, name, len) == 0)) {
				sleep_mode = i;
				filp->f_pos += *lenp;
				return 0;
			}
		}

		/* Didn't find it */
		return -EINVAL;
	} else {
		const char *name = sleep_type_name[sleep_mode];

		len = strlen(name);

		if (len > table->maxlen)
			len = table->maxlen;
		if (len > *lenp)
			len = *lenp;
		if (len > 0)
			if (copy_to_user(buffer, name, len))
				return -EFAULT;
		if (len < *lenp) {
			if (put_user('\n', ((char *) buffer) + len))
				return -EFAULT;
			len++;
		}
		*lenp = len;
		filp->f_pos += len;
	}

	return 0;
}

/* Implementation of the power management alarm function. This implementation
   assumes that the PM alarm is the only RTC interrupt in use. Global interrupt
   disable is not used due to the long latencies of APM/RTC register
   manipulations. Interrupt latencies for RTC and wakeup interrupts will be in
   the 100s of uS. */

enum alarm_status { clear, armed, expired } volatile alarm_status;

static spinlock_t pm_alarm_lock = SPIN_LOCK_UNLOCKED;

static void
_clear_alarm(enum alarm_status status)
{
	mtdcr(DCRN_RTC0_CR1, mfdcr(DCRN_RTC0_CR1) & ~RTC_AIE);
	ibm405lp_rtc_dcr_delay();
	mfdcr(DCRN_RTC0_CR2);	/* Clear status */
	ibm405lp_apm_irq_ack(APM0_IRQ_RTC);
	ibm405lp_apm_irq_disable(APM0_IRQ_RTC);
	alarm_status = status;
}

static void
clear_alarm(enum alarm_status status)
{
	spin_lock(&pm_alarm_lock);
	disable_irq(UIC_IRQ_APM);
	
	_clear_alarm(status);

	enable_irq(UIC_IRQ_APM);
	spin_unlock(&pm_alarm_lock);
}

static void
set_alarm(struct rtc_time *t)
{
	spin_lock(&pm_alarm_lock);
	disable_irq(UIC_IRQ_APM);

	ibm405lp_set_rtc_alm_time(t);
	mtdcr(DCRN_RTC0_CR1, mfdcr(DCRN_RTC0_CR1) | RTC_AIE);
	ibm405lp_rtc_dcr_delay();
	mfdcr(DCRN_RTC0_CR2);	/* Clear status */
	ibm405lp_rtc_dcr_delay();
	ibm405lp_apm_irq_enable(APM0_IRQ_RTC);
	ibm405lp_apm_irq_ack(APM0_IRQ_RTC);
	alarm_status = armed;

	enable_irq(UIC_IRQ_APM);
	spin_unlock(&pm_alarm_lock);
}

static void
pm_alarm_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	if (ibm405lp_apm_irq_status(APM0_IRQ_RTC)) {
		_clear_alarm(expired);
		printk(PREFIX 
		       "RTC alarm interrupt "
		       "[ISR: 0x%02x IER: 0x%02x].\n",
		       mfdcr(DCRN_APM0_ISR), mfdcr(DCRN_APM0_IER));
	}
}

static int
alarm_status_string(char *s, int len)
{
	struct rtc_time t;

	if (len < 17)
		return 0;
	
	spin_lock_irq(&pm_alarm_lock);
	ibm405lp_get_rtc_alm_time(&t);
	spin_unlock_irq(&pm_alarm_lock);

	return sprintf(s, "%02d:%02d:%02d %s\n",
		       (t.tm_hour < 24) ? t.tm_hour : 99,
		       (t.tm_min < 60) ? t.tm_min : 99,
		       (t.tm_sec < 60) ? t.tm_sec : 99,	
		       (alarm_status == armed) ? "armed" :
		       (alarm_status == expired) ? "expired" :
		       "clear");

}

static int pm_alarm_handler(ctl_table *table, int write, struct file *filp,
			    void *buffer, size_t *lenp)
{
	size_t len;
	char *s, *tok, *whitespace = " \t\r\n";
	struct rtc_time alm_time;

	if (!table->maxlen || !*lenp || (filp->f_pos && !write)) {
		*lenp = 0;
		return 0;
	}

	if (write) {
		char string[table->maxlen];

		len = *lenp;
		if (len >= table->maxlen)
			len = table->maxlen - 1;

		if (copy_from_user(string, buffer, len))
			return -EFAULT;
		string[len] = '\0';

		s = string + strspn(string, whitespace);
		tok = strsep(&s, whitespace);

		if (strcmp(tok, "clear") == 0) {
			clear_alarm(clear);
			return 0;
		} else if ((strlen(tok) == 8) &&	
			   (tok[2] == ':') && (tok[5] == ':')) {
			s = tok;
			tok = strsep(&s, ":");
			alm_time.tm_hour = simple_strtoul(tok, NULL, 10);
			tok = strsep(&s, ":");
			alm_time.tm_min = simple_strtoul(tok, NULL, 10);
			alm_time.tm_sec = simple_strtoul(s, NULL, 10);
			set_alarm(&alm_time);
			return 0;
		} else {
			return -EINVAL;
		}
	} else {
		char string[table->maxlen];

		len = alarm_status_string(string, table->maxlen);

		if (len > *lenp)
			len = *lenp;
		if (len > 0)
			if (copy_to_user(buffer, string, len))
				return -EFAULT;
		*lenp = len;
		filp->f_pos += len;

		return 0;
	}
}

static struct ctl_table pm_table[] =
{
	{
		.ctl_name = PM_405LP_SLEEP_CMD,
		.procname = "suspend",
		.mode = 0200, /* write-only to trigger sleep */
		.proc_handler = &suspend_proc_handler,
	},
	{
		.ctl_name = PM_405LP_SLEEP_MODE,
		.procname = "mode",
		.maxlen = 16,
		.mode = 0644,
		.proc_handler = &parse_mode,
	},
	{
		.ctl_name = PM_405LP_SLEEP_ALARM,
		.procname = "pm_alarm",
		.maxlen = 18,
		.mode = 0644,
		.proc_handler = &pm_alarm_handler,
	},
	{
		.ctl_name = PM_405LP_SLEEP_DEBUG_CDIV,
		.procname = "cdiv",
		.data = &cdiv,
		.maxlen = sizeof(int),
		.mode = 0644,
		.proc_handler = &proc_dointvec,
	},
	{
		.ctl_name = PM_405LP_SLEEP_DEBUG_WATCHDOG,
		.procname = "watchdog",
		.data = &watchdog,
		.maxlen = sizeof(int),
		.mode = 0644,
		.proc_handler = &proc_dointvec,
	},
	{0}
};

static struct ctl_table pm_dir_table[] =
{
	{
		.ctl_name = CTL_PM_405LP,
		.procname = "sleep",
		.mode = 0555,
		.child = pm_table,
	},
	{0}
};

#ifdef CONFIG_405LP_PM_BUTTON

/****************************************************************************
 * APM Power Button
 *****************************************************************************/

/* Implementation of a generic, kernel-level power button driver for a button
   attached to the 405LP APM unit.  Pressing and releasing the button will
   cause the system to suspend, using the current 'sleep_mode'.  Pressing the
   button on a sleeping system will awaken it.  If the button is pressed
   continuously for 5 seconds, the system will enter a failsafe_powerdown state
   with the APM wakeup mechanism disabled.  This will require an APM reset to
   restart the system.  The button works correctly in conjunction with all
   other mechanisms that can put the device to sleep and wake it back up. */

#define BTN_MAX   (5 * HZ)	/* After 5 sec. press, force powerdown */
#define BTN_DELAY (HZ / 10)	/* Poll 10 times per second to debounce */

enum pm_button_state { running, pressing, press_debouncing, initializing,
		       init_debouncing, failsafe, suspended };

static struct pm_button_struct {
	enum pm_button_state state; /* State of button */
	unsigned start;		/* Start of press in jiffies */
	struct timer_list timer; /* For debounce/advanced function */
} pm_button;

static struct tq_struct pm_button_task;

static void
button_suspend(void *data  __attribute__((unused)))
{
	suspend_handler(sleep_mode);
}

static void
button_powerdown(void *data  __attribute__((unused)))
{
	suspend_handler(failsafe_powerdown);
}

static void
pm_button_timer(void)
{
	pm_button.timer.expires = jiffies + BTN_DELAY;
	add_timer(&pm_button.timer);
}

/* This routine is written this way to work around limitations in the
   specification of the APM interrupt controller.  The only way to observe the
   state of the wakeup pin is to generate status, and the only way to see the
   status is to enable the interrupt.  This routine is always called with the
   IBM405LP_PM_IRQ disabled [ an invariant of the pm_button_handler() ], and
   quickly manipulates the enabled status of the IRQ to check on the status.
   Note that this may also generate status in the UIC. However, since the IRQ
   is disabled in the APM controller when the pm_button_irq() is later invoked,
   this bogus UIC status will have no effect other than a little overhead. */

static int
pm_button_get_status_and_ack(void)
{
	int status;

	disable_irq(UIC_IRQ_APM);
	ibm405lp_apm_irq_enable(IBM405LP_PM_IRQ);
	status = ibm405lp_apm_irq_status(IBM405LP_PM_IRQ);
	ibm405lp_apm_irq_disable(IBM405LP_PM_IRQ);
	if (status)
		ibm405lp_apm_irq_ack(IBM405LP_PM_IRQ);
	enable_irq(UIC_IRQ_APM);
	return status;
}

static void
pm_button_handler(unsigned long mode)
{
	switch (mode) {

	case 0:

		/* Called from initialization, IRQ handler and timer to handle
		   button pressing/debouncing. */

		switch (pm_button.state) {
		
		case running:
		
			/* The button is pressed for the first time in a
			   running system. */

			pm_button.state = pressing;
			pm_button.start = jiffies;
			ibm405lp_apm_irq_disable(IBM405LP_PM_IRQ);

			/* Fall through */

		case pressing:		
		case press_debouncing:

			/* The user is pressing the button to suspend the
                           system. */

			if (pm_button_get_status_and_ack()) {

				if ((jiffies - pm_button.start) > BTN_MAX) {

				/* Failsafe powerdown. The APM IRQ will _not_
				   be re-enabled.  */

					printk(KERN_CRIT PREFIX
					       "pm_button: Forcing fail-safe "
					       "power down\n");
					pm_button.state = failsafe;
					INIT_TQUEUE(&pm_button_task,
						    button_powerdown, 0);
					schedule_task(&pm_button_task);
					break;
				} else {
					pm_button.state = pressing;
				}
			} else if (pm_button.state == press_debouncing) {
				printk(PREFIX "pm_button: suspending\n");
				pm_button.state = suspended;
				INIT_TQUEUE(&pm_button_task,
					    button_suspend, 0);
				schedule_task(&pm_button_task);
				break;
			} else {
				pm_button.state = press_debouncing;
			}
			printk(PREFIX "pm_button: Polling\n");
			pm_button_timer();
			break;

		case suspended:

			/* Resume from suspend */

			ibm405lp_apm_irq_disable(IBM405LP_PM_IRQ);
			pm_button.state = initializing;

			/* Keep goin' */

		case initializing:
		case init_debouncing:

			/* Driver initialization, and resume from suspend.  The
			   APM0_IER bit for this PM IRQ is disabled here. The
			   user must let go of the button before we activate
			   the IRQ (again). */

			if (pm_button_get_status_and_ack()) {
				pm_button.state = initializing;
			} else if (pm_button.state == init_debouncing) {
				pm_button.state = running;
				ibm405lp_apm_irq_enable(IBM405LP_PM_IRQ);
				break;
			} else {
				pm_button.state = init_debouncing;
			}
			printk(PREFIX "Please let go of the button!\n");
			pm_button_timer();
			break;
			
		case failsafe:

			/* This is a bug */
			break;
		}
		return;

	case 1:
	       
		/* Called from ibm405lp_suspend() with interrupts and the PM
		   IRQ disabled, prior to suspend.  Arm the button for
		   wakeup. */

		switch (pm_button.state) {
		default: /* Many bizarre cases can cause this, e.g. /proc
			   suspend while pressing the button. */
			printk(KERN_WARNING PREFIX
			       "Unusual condition encountered by "
			       " pm_button_handler(1). State = [%d].\n", 
			       pm_button.state);
		case suspended: /* Suspended from button press */
		case running:	/* Suspended from /proc interface */
			ibm405lp_apm_irq_enable(IBM405LP_PM_IRQ);
		case failsafe:	/* Force powerdown without APM wakeup */
			pm_button.state = suspended;
			break;
		}
		del_timer_sync(&pm_button.timer);
		return;

	case 2:

		/* Called from ibm405lp_suspend() with interrupts disabled upon
		   resume.  If the power button did not cause the wakeup (maybe
		   it was RTC or another wakeup), set the state to 'running',
		   which arms the button to suspend the machine again. */

		if (!ibm405lp_apm_irq_status(IBM405LP_PM_IRQ))
			pm_button.state = running;
		return;
	}
}
	
static void
pm_button_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	if (ibm405lp_apm_irq_status(IBM405LP_PM_IRQ)) {
		if (pm_button.state == suspended)
			printk(PREFIX "pm_button: Resume from suspend\n");
		else if (pm_button.state != running)
			printk(KERN_CRIT PREFIX 
			       "Bug detected by pm_button_irq() [%d].\n",
			       pm_button.state
			       /* But let's not crash the system, maybe
				  everything will work out OK :-) */);
		pm_button_handler(0);
	}
}

static void __init
pm_button_init(void)
{
	int err;

	printk(PREFIX "IBM405LP PM Button Handler\n");

	ibm405lp_apm_irq_disable(IBM405LP_PM_IRQ);
	ibm405lp_apm_irq_setup(IBM405LP_PM_IRQ, 0, IBM405LP_PM_POLARITY); 

	if ((err = request_irq(UIC_IRQ_APM, pm_button_irq, SA_SHIRQ,
			       "ibm405lp_pm_button", NULL))) {
		printk(KERN_ERR PREFIX
		       "Request power button IRQ %d failed (%d)\n",
		       UIC_IRQ_APM, err);
		return;
	}

	pm_button.state = initializing;
	init_timer(&pm_button.timer);
	pm_button.timer.function = pm_button_handler;
	pm_button_handler(0);
}
#endif /* CONFIG_405LP_PM_BUTTON */

/*
 * Initialize power interface
 */
static int __init ibm405lp_pm_init(void)
{
	apm0_cfg_t apm0_cfg;
	int err;

	/* Since CDIV determines the speed of the entire APM unit, the
	 * delay required after an mtdcr (before it takes effect) is
	 * dependent on it.  We don't want to have to wait very long
	 * at all, so we *always* zero CDIV (causing the APM to run at
	 * RTC frequency), both at boot and in wakeup code.
	 */
	apm0_cfg.reg = mfdcr(DCRN_APM0_CFG) & 0xfffffffe;
	apm0_cfg.fields.cdiv = 0;
	mtdcr_interlock(DCRN_APM0_CFG, apm0_cfg.reg, 0xfffc0000);

	/* APM0_ID wasn't updated from 405LP 1.0 -> 1.1, so is
	 * probably useless (use PVR instead). OTOH it would be nice
	 * if it was used. :(
	 */
	printk(KERN_INFO PREFIX "APM version %x\n", mfdcr(DCRN_APM0_ID));

	/* we need to know the RTC frequency in a few places, so read
	 * it here */
	switch ((mfdcr(DCRN_RTC0_CR0) >> 4) & 0x7) {
		/* check DV0, DV1, DV3 */
		case RTC_DVBITS_4MHZ:
			rtc_freq_hz = 4194304;
			break;
		case RTC_DVBITS_33KHZ:
			rtc_freq_hz = 32768;
			break;
		default:
			printk(KERN_ERR PREFIX
					"unknown RTC clock rate! defaulting to 1048576 Hz\n");
			/* fall through */
		case RTC_DVBITS_1MHZ:
			rtc_freq_hz = 1048576;
			break;
	}
	/* we need to clear the shutdown flags from the RTC Century field in
	   case the firmware forgot to. */
	mtdcr(DCRN_RTC0_CEN, 
	      mfdcr(DCRN_RTC0_CEN) & ~(IBM405LP_POWERDOWN_SUSPEND ||
				       IBM405LP_POWERDOWN_HIBERNATE ||
				       IBM405LP_POWERDOWN_REBOOT));

	register_sysctl_table(pm_dir_table, 1);

	pm_button_init();

	clear_alarm(clear);
	if ((err = request_irq(UIC_IRQ_APM, pm_alarm_irq, SA_SHIRQ,
			       "ibm405lp_pm_alarm", NULL))) {
		printk(KERN_ERR PREFIX
		       "Request PM Alarm IRQ %d failed (%d)\n",
		       UIC_IRQ_APM, err);
		return -ENODEV;
	}

	return 0;
}

__initcall(ibm405lp_pm_init);

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
