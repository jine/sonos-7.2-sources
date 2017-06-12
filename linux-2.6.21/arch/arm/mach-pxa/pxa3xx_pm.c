/*
 * Monahans Power Management Routines
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#undef	DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/ipmc.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/pxa3xx_pm.h>
#ifdef CONFIG_MACH_WOODSTOCK
#include <asm/arch/pxa3xx_pmic.h>
#include <asm/arch/woodstock_ioctl.h>
#include "obmstub.h"
#endif

#include "sleepwkr.h"

/* mtd.h declares another DEBUG macro definition */
#undef DEBUG
#include <linux/mtd/mtd.h>

/* The first 32KB is reserved and can't be accessed by kernel.
 * This restrict is only valid on BootROM V2.
 */
#define ISRAM_START	0x5c000000

/* MOBM_START should be larger than SRAM_START */
/* MOBM_START is used on MOBM V2.
 * The address is 0x5c014000. It means MOBM will be copied on the address.
 * On MOBM V3, it will be copied on 0x5c013000.
 */
#define MOBM_START	0x5c014000
#ifdef CONFIG_MACH_WOODSTOCK
#define MOBM_SIZE	(sizeof(obmstub))
#else
#define MOBM_SIZE	(32 * 1024)
#endif
#define MOBM_OFFSET	8


#ifdef CONFIG_CPU_PXA320
#define ISRAM_SIZE	(128 * 6 * 1024)
#else
#define ISRAM_SIZE	(128 * 2 * 1024)
#endif

/* MOBM V2 is used on MhnP B0/B1/B2, MhnPL B1 and MhnL A0
 * MOBM V3 is used on MhnLV A0
 */
enum {
	PXA3xx_OBM_NULL,
	PXA3xx_OBM_V2,
	PXA3xx_OBM_V3,
	PXA3xx_OBM_INVAL,
};

enum pxa3xx_pm_mode {
	PXA3xx_PM_RUN = 0,
	PXA3xx_PM_IDLE = 1 ,
	PXA3xx_PM_LCDREFRESH = 2,
	PXA3xx_PM_STANDBY = 3,
	PXA3xx_PM_D0CS = 5,
	PXA3xx_PM_SLEEP = 6,
	PXA3xx_PM_DEEPSLEEP = 7,
};

struct pxa3xx_sleep_pin {
	unsigned int	gpioex0;
	unsigned int	gpioex1;
	unsigned int	gpioex0_saved;
	unsigned int	gpioex1_saved;
};

extern struct subsystem power_subsys;

static struct pxa3xx_sleep_pin pins = { 
	.gpioex0_saved = 0, 
	.gpioex1_saved = 0,
};

static unsigned sys_state;

pm_wakeup_src_t wakeup_src;	/* ipmc interface can set wakeup_src */
EXPORT_SYMBOL(wakeup_src);

static pm_wakeup_src_t waked;	/* It records the latest wakeup source */

/* How long we will in sleep mode if duty cycle. */
unsigned int  pm_sleeptime = 0;  /* In seconds. */
EXPORT_SYMBOL(pm_sleeptime);
unsigned int  pm_msleeptime = 0; /* In miliseconds. */

extern void woodstock_keypad_config(void);
extern void woodstock_keypad_sleep_config(void);
extern void woodstock_motionsensor_config(void);
extern void woodstock_motionsensor_sleep_config(void);
extern void woodstock_micco_pins_config(void);
// extern void woodstock_micco_sleep_pins_config(void);

#ifdef CONFIG_IPM

/* How long we will in run mode if duty cycle. */
unsigned int  pm_waketime = 2;  /* In seconds. */
EXPORT_SYMBOL(pm_waketime);

/* After pm_uitimeout long without input from keypad/touch, we will sleep. */
unsigned int  pm_uitimeout= 30; /* In seconds. */
EXPORT_SYMBOL(pm_uitimeout);

static struct timer_list pm_timer;
int pm_updatetimer(int);
#ifdef CONFIG_IPM_DEEPIDLE
int enable_deepidle = 1;
#else
int enable_deepidle = 0;
#endif
int save_deepidle = 0;
int deepidle_ticks = 2;
#endif

extern void pxa3xx_cpu_sleep(unsigned int, unsigned int);
extern void pxa3xx_cpu_resume(void);
extern void pxa3xx_cpu_standby(unsigned int);
extern void pxa3xx_cpu_deepsleep(unsigned int, unsigned int);
extern void pxa3xx_cpu_lcdrefresh(unsigned int);

void (*event_notify)(int, int, void *, unsigned int) = NULL;
EXPORT_SYMBOL(event_notify);

extern void pxa3xx_gpio_save(void);
extern void pxa3xx_gpio_restore(void);

static struct pxa3xx_pm_regs pxa3xx_pm_regs;

static unsigned long pm_state;

int get_pm_state(void)
{
	return pm_state;
}

/*************************************************************************/
/* workaround for bug 2140448 */
static int is_wkr_2140448(void)
{
#ifdef CONFIG_PXA3xx_D0CS
	unsigned int	cpuid;
	/* read CPU ID */
	__asm__ (
		"mrc p15, 0, %0, c0, c0, 0\n"
		: "=r" (cpuid)
	);
	if ((cpuid & 0xFFFF0000) != 0x69050000) {
		/* It's not xscale chip. */
		return 0;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		/* It's MhnP B1 */
		if ((cpuid & 0x0F) == 5)
			return 1;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		/* It's MhnL A0 */
		if ((cpuid & 0x0F) == 0)
			return 1;
	}
#endif
	return 0;
}

/* workaround for bootrom wrongly cleared OSCR on MHLV-A2 */
static int is_wkr_oscr(void)
{
	unsigned int	cpuid;

	/* read CPU ID */
	cpuid = read_cpuid(0) & 0xFFFF;
	/* It's Monahans LV A2 */
	if (cpuid == 0x6892)
		return 1;
	return 0;
}

/*
 * MOBM V2 is applied on chips taped out before MhnLV A0.
 * MOBM V3 is applied on chips taped out after MhnLV A0.
 * MOBM V3 is also applied on MhnLV A0.
 */
static int calc_obm_ver(void)
{
	unsigned int	cpuid;
	/* read CPU ID */
	__asm__ (
		"mrc p15, 0, %0, c0, c0, 0\n"
		: "=r" (cpuid)
	);
	if ((cpuid & 0xFFFF0000) != 0x69050000) {
		/* It's not xscale chip. */
		return PXA3xx_OBM_INVAL;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006420) {
		/* It's MhnP Ax */
		return PXA3xx_OBM_V2;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		/* It's MhnP Bx */
		if ((cpuid & 0x0F) <= 6)
			return PXA3xx_OBM_V2;
		else
			return PXA3xx_OBM_V3;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		/* It's MhnL Ax */
		if ((cpuid & 0x0F) == 0)
			return PXA3xx_OBM_V2;
		else
			return PXA3xx_OBM_V3;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006890) {
		/* It's MhnLV Ax */
		return PXA3xx_OBM_V3;
	}
	return PXA3xx_OBM_INVAL;
}

/* Return the address of OBM in RAM if successful.
 * Otherwise, return negative value.
 */
#ifdef CONFIG_MACH_WOODSTOCK
int pxa3xx_nand_read_obm(int,int,unsigned char *);
#endif
static int load_obm(void)
{
#ifdef CONFIG_MACH_WOODSTOCK
	return (int)obmstub;
#else
	void *addr = NULL;
	struct mtd_info *mtd = NULL;
	int obm_ver, retlen;

	mtd = get_mtd_device(NULL, 0);
	if (mtd == NULL)
		return -ENODEV;
	addr = kmalloc(MOBM_SIZE, GFP_KERNEL);
	if (!addr)
		return -ENOMEM;

	obm_ver = calc_obm_ver();
	if (obm_ver == PXA3xx_OBM_V2) {
		/* MOBM begins from 0x0000 */
#ifdef CONFIG_MACH_WOODSTOCK
                pxa3xx_nand_read_obm(0,MOBM_SIZE,addr);
#else
		if (mtd->writesize == 2048)
			mtd->read(mtd, 0x0, MOBM_SIZE, &retlen, addr);
		else {
#if (MOBM_SIZE > 16 * 1024)
			mtd->read(mtd, 0, 0x4000, &retlen, addr);
			mtd->read(mtd, 0x4000, MOBM_SIZE - 0x4000, &retlen,
				addr + 0x3e00);
#else
			mtd->read(mtd, 0, MOBM_SIZE, &retlen, addr);
#endif
		}
#endif /*!WOODSTOCK*/
		addr += MOBM_OFFSET;
	} else if (obm_ver == PXA3xx_OBM_V3) {
#ifdef CONFIG_MACH_WOODSTOCK
		// On Woodstock, we've squeezed the NTIM, OBM, and blob all
		// together into the first block.  OBM begins 1k in
		// (the first 1k is reserved for the NTIM)
                pxa3xx_nand_read_obm(0x400,MOBM_SIZE,addr);
#else
		/* MOBM begins from 0x20000 */
		if (mtd->writesize == 2048)
			mtd->read(mtd, 0x20000, MOBM_SIZE, &retlen, addr);

#endif /*!WOODSTOCK*/
	}
	pr_debug("load mobm into address: 0x%x\n", (unsigned int)addr);
	return (int)addr;
#endif
}

static void pxa3xx_intc_save(struct intc_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int temp, i;

	context->iccr = readl(base + ICCR);
	for (i = 0; i < 32; i++) {
		context->ipr[i] = readl(base + IPR0_OFF + (i << 2));
	}
	for (i = 0; i < 21; i++) {
		context->ipr2[i] = readl(base + IPR32_OFF + (i << 2));
	}

	/* load registers by accessing co-processor */
	__asm__("mrc\tp6, 0, %0, c1, c0, 0" : "=r" (temp));
	context->icmr = temp;
	__asm__("mrc\tp6, 0, %0, c7, c0, 0" : "=r" (temp));
	context->icmr2 = temp;
	__asm__("mrc\tp6, 0, %0, c2, c0, 0" : "=r" (temp));
	context->iclr = temp;
	__asm__("mrc\tp6, 0, %0, c8, c0, 0" : "=r" (temp));
	context->iclr2 = temp;
}

static void pxa3xx_intc_restore(struct intc_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int temp, i;

	writel(context->iccr, base + ICCR_OFF);
	for (i = 0; i < 32; i++) {
		writel(context->ipr[i], base + IPR0_OFF + (i << 2));
	}
	for (i = 0; i < 21; i++) {
		writel(context->ipr2[i], base + IPR32_OFF + (i << 2));
	}

	temp = context->icmr;
	__asm__("mcr\tp6, 0, %0, c1, c0, 0" : :"r"(temp));
	temp = context->icmr2;
	__asm__("mcr\tp6, 0, %0, c7, c0, 0" : :"r"(temp));
	temp = context->iclr;
	__asm__("mcr\tp6, 0, %0, c2, c0, 0" : :"r"(temp));
	temp = context->iclr2;
	__asm__("mcr\tp6, 0, %0, c8, c0, 0" : :"r"(temp));
}

static void pxa3xx_clk_save(struct clock_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	context->aicsr = readl(base + AICSR_OFF);
	context->ckena = readl(base + CKENA_OFF);
	context->ckenb = readl(base + CKENB_OFF);
	context->oscc = readl(base + OSCC_OFF);
	/* Disable the processor to use the ring oscillator output clock
	 * as a clock source when transitioning from any low-power mode
	 * to D0 mode.
	 */
	tmp = readl(base + ACCR_OFF);
	tmp &= ~ACCR_PCCE;
	writel(tmp, base + ACCR_OFF);
}

static void pxa3xx_clk_restore(struct clock_regs *context)
{
	unsigned char __iomem *base = context->membase;
	context->aicsr &= (AICSR_PCIE | AICSR_TCIE | AICSR_FCIE);
	writel(context->aicsr, base + AICSR_OFF);
	writel(context->ckena, base + CKENA_OFF);
	writel(context->ckenb, base + CKENB_OFF);
	writel(context->oscc, base + OSCC_OFF);
}

static void pxa3xx_ost_save(struct ost_regs *context)
{
	unsigned char __iomem *base = context->membase;
	if (is_wkr_oscr())
		context->oscr = readl(base + OSCR_OFF);
	context->oscr4 = readl(base + OSCR4_OFF);
	context->omcr4 = readl(base + OMCR4_OFF);
	context->oier = readl(base + OIER_OFF);
}

static void pxa3xx_ost_restore(struct ost_regs *context)
{
	unsigned char __iomem *base = context->membase;
	if (is_wkr_oscr())
		writel(context->oscr, base + OSCR_OFF);
	writel(context->oscr4, base + OSCR4_OFF);
	writel(context->omcr4, base + OMCR4_OFF);
	writel(context->oier, base + OIER_OFF);
}

static void pxa3xx_sysbus_init(struct pxa3xx_pm_regs *context)
{
	context->clock.membase = (unsigned char *)KSEG0(CLKBASE);
	context->intc.membase = (unsigned char *)KSEG0(INTCBASE);
	context->rtc.membase = (unsigned char *)KSEG0(RTCBASE);
	context->ost.membase = (unsigned char *)KSEG0(OSTBASE);
	context->pmu.membase = (unsigned char *)KSEG0(PMUBASE);
	context->smc.membase = (unsigned char *)KSEG1(SMCBASE);
	context->arb.membase = (unsigned char *)KSEG1(ARBBASE);

	context->sram_map = ioremap(ISRAM_START, ISRAM_SIZE);
	context->sram = vmalloc(ISRAM_SIZE);
	context->obm = (void *)load_obm();
	/* Two words begun from 0xC0000000 are used to store key information.
	 */
	context->data_pool = (unsigned char *)0xC0000000;
}

static void pxa3xx_sysbus_save(struct pxa3xx_pm_regs *context)
{
	unsigned char __iomem *base = NULL;
	unsigned int tmp;

	/* static memory controller */
	base = context->smc.membase;
	context->smc.msc0 = readl(base + MSC0_OFF);
	context->smc.msc1 = readl(base + MSC1_OFF);
	context->smc.sxcnfg = readl(base + SXCNFG_OFF);
	context->smc.memclkcfg = readl(base + MEMCLKCFG_OFF);
	context->smc.cscfg0 = readl(base + CSADRCFG0_OFF);
	context->smc.cscfg1 = readl(base + CSADRCFG1_OFF);
	context->smc.cscfg2 = readl(base + CSADRCFG2_OFF);
	context->smc.cscfg3 = readl(base + CSADRCFG3_OFF);
	context->smc.csmscfg = readl(base + CSMSADRCFG_OFF);
#ifdef CONFIG_CPU_PXA320
	context->smc.mecr = readl(base + MECR_OFF);
	context->smc.mcmem0 = readl(base + MCMEM0_OFF);
	context->smc.mcatt0 = readl(base + MCATT0_OFF);
	context->smc.mcio0 = readl(base + MCIO0_OFF);
	context->smc.cscfg_p = readl(base + CSADRCFG_P_OFF);
#endif
	/* system bus arbiters */
	base = context->arb.membase;
	context->arb.ctl1 = readl(base + ARBCTL1_OFF);
	context->arb.ctl2 = readl(base + ARBCTL2_OFF);

	/* pmu controller */
	base = context->pmu.membase;
	context->pmu.pecr = readl(base + PECR_OFF);
	context->pmu.pvcr = readl(base + PVCR_OFF);
	/* clear PSR */
	tmp = readl(base + PSR_OFF);
	tmp &= 0x07;
	writel(tmp, base + PSR_OFF);

	pxa3xx_intc_save(&(context->intc));
	pxa3xx_clk_save(&(context->clock));
	// XXX BT also don't save and restore the OST, also doesn't need it
	// pxa3xx_ost_save(&(context->ost));
	// XXX BT saving and restoring the MFPRs is harmful and wrong
	// XXX they are preserved during S2D3C4 anyways
	//pxa3xx_mfp_save();
	pxa3xx_gpio_save();
}

static void pxa3xx_sysbus_restore(struct pxa3xx_pm_regs *context)
{
	unsigned char __iomem *base = NULL;

	//XXX BT don't restore MFPRs, not necessary and could be bad
	//pxa3xx_mfp_restore();
	pxa3xx_gpio_restore();
	// pxa3xx_ost_restore(&(context->ost));
	pxa3xx_intc_restore(&(context->intc));
	pxa3xx_clk_restore(&(context->clock));

	/* PMU controller */
	base = context->pmu.membase;
	/* status information will be lost in PECR */
	writel(0xA0000000, base + PECR_OFF);
	writel((context->pmu.pecr | PECR_E1IS | PECR_E0IS), base + PECR_OFF);
	writel(context->pmu.pvcr, base + PVCR_OFF);

	/* system bus arbiters */
	base = context->arb.membase;
	writel(context->arb.ctl1, base + ARBCTL1_OFF);
	writel(context->arb.ctl2, base + ARBCTL2_OFF);

	/* static memory controller */
	base = context->smc.membase;
	writel(context->smc.msc0, base + MSC0_OFF);
	writel(context->smc.msc1, base + MSC1_OFF);
	writel(context->smc.sxcnfg, base + SXCNFG_OFF);
	writel(context->smc.memclkcfg, base + MEMCLKCFG_OFF);
	writel(context->smc.cscfg0, base + CSADRCFG0_OFF);
	writel(context->smc.cscfg1, base + CSADRCFG1_OFF);
	writel(context->smc.cscfg2, base + CSADRCFG2_OFF);
	writel(context->smc.cscfg3, base + CSADRCFG3_OFF);
	writel(context->smc.csmscfg, base + CSMSADRCFG_OFF);
#ifdef CONFIG_CPU_PXA320
	writel(context->smc.mecr, base + MECR_OFF);
	writel(context->smc.mcmem0, base + MCMEM0_OFF);
	writel(context->smc.mcatt0, base + MCATT0_OFF);
	writel(context->smc.mcio0, base + MCIO0_OFF);
	writel(context->smc.cscfg_p, base + CSADRCFG_P_OFF);
#endif
}

/* This function is used to set unit clock before system enters sleep.
 */
static void pxa3xx_pm_set_cken(void)
{
	/*
	 * turn off SMC, GPIO,INTC clocks to save power in sleep mode.
	 * they will be turn on by BLOB during wakeup
	 */
	pxa_set_cken(CKEN_SMC, 0);
	pxa_set_cken(CKEN_GPIO, 0);
	pxa_set_cken(CKEN_INTC, 0);

	/*
	 * turn on clocks used by bootrom during wakeup
	 * they will be turn off by BLOB during wakeup
	 * D0CKEN_A clocks: bootrom, No.19
	 */
	pxa_set_cken(CKEN_BOOT, 1);
	pxa_set_cken(CKEN_TPM, 1);
	/* This bit must be enabled before entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 1);

	/* Trusted parts */
	pxa_set_cken(CKEN_SMC, 1);
	pxa_set_cken(CKEN_MINI_LCD, 1);
}

/* This function is used to restore unit clock after system resumes.
 */
static void pxa3xx_pm_restore_cken(void)
{
	pxa_set_cken(CKEN_SMC, 1);
	pxa_set_cken(CKEN_GPIO, 1);
	pxa_set_cken(CKEN_INTC, 1);
	pxa_set_cken(CKEN_BOOT, 0);
	pxa_set_cken(CKEN_TPM, 0);
}

/* This function is used to clear power manager status.
 */
static void pxa3xx_clear_pm_status(unsigned char __iomem *base, int sys_level)
{
	unsigned int tmp;

	if (sys_level) {
		/* clear power manager status */
		tmp = readl(base + PSR_OFF);
		tmp &= PSR_MASK;
		writel(tmp, base + PSR_OFF);
	}
	/* clear application system status */
	tmp = readl(base + ASCR_OFF);
	tmp &= ASCR_MASK;
	writel(tmp, base + ASCR_OFF);
	/* clear all application subsystem reset status */
	tmp = readl(base + ARSR_OFF);
	writel(tmp, base + ARSR_OFF);
}

/* This function is used to set RTC time.
 * When it timeouts, it will wakeup system from low power mode.
 * There's limitation that only 65 seconds sleep time can be set by this way.
 * And user should avoid to use PIAR because it will be used as wakeup timer.
 *
 * Notice:
 * User can also choice use another RTC register to trigger wakeup event.
 * If so, keep pm_sleeptime as 0. Otherwise, those RTC registers event
 * will make user confused. System will only serve the first RTC event.
 */
static void pxa3xx_set_wakeup_sec(int sleeptime, struct rtc_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	if (sleeptime) {
		/* PIAR can not more than 65535 */
		if (sleeptime > 65)
			sleeptime = 65;
		pr_debug("Set RTC to wakeup system after %d sec\n",
			sleeptime);
		tmp = readl(base + RTSR_OFF);
		tmp &= ~(RTSR_PICE | RTSR_PIALE);
		writel(tmp, base + RTSR_OFF);
		/* set PIAR to sleep time, in ms */
		writel(sleeptime * 1000, base + PIAR_OFF);

		tmp = readl(base + RTSR_OFF);
		tmp |= RTSR_PICE;
		writel(tmp, base + RTSR_OFF);
	} else {
		/* Disable PIAR */
		tmp = readl(base + RTSR_OFF);
		tmp &= ~(RTSR_PICE | RTSR_PIALE);
		writel(tmp, base + RTSR_OFF);
	}
}

/* This function is used to set OS Timer4 time.
 * The time interval may not be accurate. Because it's derived from 32.768kHz
 * oscillator.
 */
static void pxa3xx_set_wakeup_msec(int msleeptime, struct ost_regs *context)
{
	// XXX BT enable OST4 for measuring sleep duration even if
	// XXX msleeptime is 0, but disable the interrupt if 
	unsigned char __iomem *base = context->membase;
	unsigned int tmp;
	pr_debug("Set OS Timer4 to wakeup system after %d msec\n",
		msleeptime);
	tmp = readl(base + OIER_OFF);
	tmp &= ~0x0010;
	writel(tmp, base + OIER_OFF);
	/* set the time interval of sleep */
	if (msleeptime) {
		writel(msleeptime, base + OSMR4_OFF);
	} else {
		writel(0xffffffff, base + OSMR4_OFF); //so it can't roll
	}
	/* use 32.768kHz oscillator when cpu is in low power mode */
	writel(0x0082, base + OMCR4_OFF);
	if (msleeptime) {
		tmp = readl(base + OIER_OFF);
		tmp |= 0x0010;
		writel(tmp, base + OIER_OFF);
	}
	/* kick off the OS Timer4 */
	writel(0, base + OSCR4_OFF);
}

#ifdef  CONFIG_MACH_ZYLONITE
static int set_mmc_wakeup_src(pm_wakeup_src_t src)
{
	int ret = 0;
#ifdef CONFIG_CPU_PXA320
#elif defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310)
	if (src.bits.mmc1_cd || src.bits.mmc2_cd) {
		if (!pins.gpioex0_saved) {
			pins.gpioex0 = MFP_REG(MFP_GPIO_EXP_0_N);
			pins.gpioex0_saved = 1;
		}
		pxa3xx_mfp_set_afds(MFP_GPIO_EXP_0_N, MFP_AF2, 0);
		pxa3xx_mfp_set_lpm(MFP_GPIO_EXP_0_N, MFP_PULL_HIGH);
		pxa3xx_mfp_set_edge(MFP_GPIO_EXP_0_N, MFP_EDGE_FALL);
		ret |= PXA3xx_PM_WE_SSP;
	}
	if (src.bits.mmc3_cd) {
		if (!pins.gpioex1_saved) {
			pins.gpioex1 = MFP_REG(MFP_GPIO_EXP_1_N);
			pins.gpioex1_saved = 1;
		}
		pxa3xx_mfp_set_afds(MFP_GPIO_EXP_1_N, MFP_AF2, 0);
		pxa3xx_mfp_set_lpm(MFP_GPIO_EXP_1_N, MFP_PULL_HIGH);
		pxa3xx_mfp_set_edge(MFP_GPIO_EXP_1_N, MFP_EDGE_FALL);
		ret |= PXA3xx_PM_WE_SSP2;
	}
	if (src.bits.mmc1_dat1) {
               pxa3xx_mfp_set_lpm(MFP_MMC_DAT1, MFP_LPM_PULL_HIGH);
               pxa3xx_mfp_set_edge(MFP_MMC_DAT1, MFP_EDGE_BOTH);
               ret |= PXA3xx_PM_WE_MMC1;
	}
	if (src.bits.mmc2_dat1) {
               pxa3xx_mfp_set_lpm(MFP_MMC2_DAT1, MFP_LPM_PULL_HIGH);
               pxa3xx_mfp_set_edge(MFP_MMC2_DAT1, MFP_EDGE_BOTH);
               ret |= PXA3xx_PM_WE_MMC2;
	}
#ifdef CONFIG_CPU_PXA310
	if (src.bits.mmc3_dat1) {
               pxa3xx_mfp_set_lpm(MFP_MMC3_DAT1, MFP_LPM_PULL_HIGH);
               pxa3xx_mfp_set_edge(MFP_MMC3_DAT1, MFP_EDGE_BOTH);
               ret |= PXA3xx_PM_WE_MMC3;
        }
#endif
#endif
	return ret;
}

static void clear_mmc_wakeup_src(pm_wakeup_src_t src)
{
#ifdef CONFIG_CPU_PXA320
#elif defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310)
	if (src.bits.mmc1_cd || src.bits.mmc2_cd) {
		pxa3xx_mfp_set_edge(MFP_GPIO_EXP_0_N, MFP_EDGE_NONE);
		if (pins.gpioex0_saved) {
			MFP_REG(MFP_GPIO_EXP_0_N) = pins.gpioex0;
			pins.gpioex0_saved = 0;
		}
	}
	if (src.bits.mmc3_cd) {
		pxa3xx_mfp_set_edge(MFP_GPIO_EXP_1_N, MFP_EDGE_NONE);
		if (pins.gpioex1_saved) {
			MFP_REG(MFP_GPIO_EXP_1_N) = pins.gpioex1;
			pins.gpioex1_saved = 0;
		}
	}
	if (src.bits.mmc1_dat1) {
               	pxa3xx_mfp_set_edge(MFP_MMC_DAT1, MFP_EDGE_NONE);
	}
	if (src.bits.mmc2_dat1) {
               	pxa3xx_mfp_set_edge(MFP_MMC2_DAT1, MFP_EDGE_NONE);
	}
#ifdef CONFIG_CPU_PXA310
	if (src.bits.mmc2_dat1) {
               	pxa3xx_mfp_set_edge(MFP_MMC3_DAT1, MFP_EDGE_NONE);
	}
#endif
#endif
}
#else
static int set_mmc_wakeup_src(pm_wakeup_src_t src) {return 0;}
static void clear_mmc_wakeup_src(pm_wakeup_src_t src) {}
#endif

/*
 * Clear the wakeup source event.
 */
static void pm_clear_wakeup_src(pm_wakeup_src_t src)
{
	/* set MFPR */
	if (src.bits.mkey) {
#ifndef CONFIG_MACH_WOODSTOCK
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_0, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_1, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_2, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_3, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_4, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_5, MFP_EDGE_NONE);
#ifdef	CONFIG_MACH_ZYLONITE
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_6, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_7, MFP_EDGE_NONE);
#endif
#endif /*!WOODSTOCK*/
	}
	if (src.bits.dkey) {
#ifdef CONFIG_MACH_WOODSTOCK
		woodstock_keypad_config();
#endif
#ifndef CONFIG_MACH_WOODSTOCK
		pxa3xx_mfp_set_edge(MFP_KP_DKIN_0, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_KP_DKIN_1, MFP_EDGE_NONE);
#endif
	}
#ifndef CONFIG_MACH_WOODSTOCK
	if (src.bits.uart1) {
		pxa3xx_mfp_set_edge(MFP_FFRXD, MFP_EDGE_NONE);
	}
#endif /*!WOODSTOCK*/
#ifdef CONFIG_WIFI_HOST_SLEEP
	if (src.bits.wifi) {
		/* wifi use FFDSR as wakeup source */
		pxa3xx_mfp_set_edge(MFP_WIFI_WAKEUP_HOST, MFP_EDGE_NONE);
	}
#endif
#ifndef CONFIG_MACH_WOODSTOCK
	if (src.bits.uart2) {
		pxa3xx_mfp_set_edge(MFP_BT_RXD, MFP_EDGE_NONE);
		pxa3xx_mfp_set_edge(MFP_BT_CTS, MFP_EDGE_NONE);
	}
	if (src.bits.uart3) {
		pxa3xx_mfp_set_edge(MFP_STD_RXD, MFP_EDGE_NONE);
	}
#endif /*0*/
	if (src.bits.mmc1_cd || src.bits.mmc2_cd || src.bits.mmc3_cd \
		|| src.bits.mmc1_dat1 || src.bits.mmc2_dat1 	\
		|| src.bits.mmc3_dat1)
		clear_mmc_wakeup_src(src);
	if (src.bits.ost) {
		unsigned char __iomem *base = pxa3xx_pm_regs.ost.membase;
		unsigned int tmp;
		writel(0x10, base + OSSR_OFF);
		tmp = readl(base + OIER_OFF);
		tmp &= ~0x10;
		writel(tmp, base + OIER_OFF);
		tmp = readl(base + OSCR4_OFF);
		// printk("slept for %u ms (%u jiffies)\n",tmp,tmp/10);
		tmp = tmp/10;
		// XXX BT jiffies should never advance by more than
		// XXX MAX_JIFFY_OFFSET at once or things could become
		// XXX confused.  Although there is no practical need to
		// XXX consider the overflow of jiffies_64, much kernel
		// XXX timekeeping depends on jiffies (the LS 32 bits)
		// XXX and there is a notion of jiffies comparison and
		// XXX whether a certain time in jiffies is "before" or
		// XXX "after" another.  Although jiffies is unsigned,
		// XXX before/after computation is done by casting to
		// XXX signed, subtracting, and comparing to 0.
		// XXX MAX_JIFFY_OFFSET is MAX_LONG/2 - 1 so this will
		// XXX work out correctly.  The -HZ above is a fudge factor
		// XXX in case we miss a soft interrupt or three (as long
		// XXX as it's less than HZ of them)
		if (tmp>(MAX_JIFFY_OFFSET-HZ)) tmp = MAX_JIFFY_OFFSET-HZ;
		write_seqlock(&xtime_lock);
		jiffies_64+=tmp;
		raise_softirq(TIMER_SOFTIRQ);
		write_sequnlock(&xtime_lock);
	}
	if (src.bits.motion) {
		woodstock_motionsensor_config();
	}
	if (src.bits.pmirq) {
		woodstock_micco_pins_config();
	}
}

static void pm_select_wakeup_src(enum pxa3xx_pm_mode lp_mode,
				pm_wakeup_src_t src,
				struct pmu_regs *context)
{
	unsigned char __iomem *base = context->membase;
	unsigned int tmp, reg_src = 0;
	struct pxa3xx_pm_regs *p = NULL;

	/* set MFPR */
		
	if (src.bits.mkey) {
#ifndef CONFIG_MACH_WOODSTOCK
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_0, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_1, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_2, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_3, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_4, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_5, MFP_EDGE_BOTH);
#ifdef	CONFIG_MACH_ZYLONITE
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_6, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_MKIN_7, MFP_EDGE_BOTH);
#endif
#endif /*!WOODSTOCK*/
		reg_src |= PXA3xx_PM_WE_MKEY;
	}
	if (src.bits.dkey) {
#ifdef CONFIG_MACH_WOODSTOCK
		woodstock_keypad_sleep_config();
#endif
#ifndef CONFIG_MACH_WOODSTOCK
		pxa3xx_mfp_set_edge(MFP_KP_DKIN_0, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_KP_DKIN_1, MFP_EDGE_BOTH);
#endif
		reg_src |= PXA3xx_PM_WE_DKEY;
	}
#ifndef CONFIG_MACH_WOODSTOCK
	if (src.bits.uart1) {
		pxa3xx_mfp_set_edge(MFP_FFRXD, MFP_EDGE_FALL);
		reg_src |= PXA3xx_PM_WE_UART1;
	}
#endif /*!WOODSTOCK*/
#ifdef CONFIG_WIFI_HOST_SLEEP
	if (src.bits.wifi) {
		pxa3xx_mfp_set_edge(MFP_WIFI_WAKEUP_HOST, MFP_EDGE_FALL);
		reg_src |= PXA3xx_PM_WE_UART1;
	}
#endif
	if (src.bits.mmc1_cd || src.bits.mmc2_cd || src.bits.mmc3_cd) {
		if ((lp_mode == PXA3xx_PM_STANDBY) || (lp_mode == PXA3xx_PM_LCDREFRESH))
			reg_src |= set_mmc_wakeup_src(src);
	}
	if (src.bits.mmc1_dat1 || src.bits.mmc2_dat1 || src.bits.mmc3_dat1) {
		reg_src |= set_mmc_wakeup_src(src);
	}
#ifndef CONFIG_MACH_WOODSTOCK
	if (src.bits.uart2) {
		pxa3xx_mfp_set_edge(MFP_BT_RXD, MFP_EDGE_BOTH);
		pxa3xx_mfp_set_edge(MFP_BT_CTS, MFP_EDGE_BOTH);
		reg_src |= PXA3xx_PM_WE_UART2;
	}
	if (src.bits.uart3) {
		pxa3xx_mfp_set_edge(MFP_STD_RXD, MFP_EDGE_BOTH);
		reg_src |= PXA3xx_PM_WE_UART3;
	}
#endif /*0*/
	if (src.bits.rtc) {
		p = container_of(context, struct pxa3xx_pm_regs, pmu);
		pxa3xx_set_wakeup_sec(pm_sleeptime, &(p->rtc));
		reg_src |= PXA3xx_PM_WE_RTC;
	}
	if (src.bits.ost) {
		p = container_of(context, struct pxa3xx_pm_regs, pmu);
		pxa3xx_set_wakeup_msec(pm_msleeptime, &(p->ost));
		reg_src |= PXA3xx_PM_WE_OST;
	}
	if (src.bits.ext0)
		reg_src |= PXA3xx_PM_WE_EXTERNAL0;
	if (src.bits.ext1)
		reg_src |= PXA3xx_PM_WE_EXTERNAL1;
#ifdef CONFIG_MACH_WOODSTOCK
	if (src.bits.motion) {
                //XXX BT this is called from mma7455l.c to avoid a race
		//woodstock_motionsensor_sleep_config();
		reg_src |= PXA3xx_PM_WE_MMC3;
	}
	if (src.bits.pmirq) {
		// XXX BT sleep_pins_config done in the micco suspend
		// XXX routine so as to avoid a potential race.
		// woodstock_micco_sleep_pins_config();
		reg_src |= PXA3xx_PM_WE_UART1;
	}
	if (src.bits.i2c) {
		reg_src |= PXA3xx_PM_WE_GENERIC9;
	}
#endif

	/* set wakeup register */
	if (lp_mode == PXA3xx_PM_SLEEP) {
		writel(0xFFFFFFFF, base + PWSR_OFF);
		writel(0, base + PWER_OFF);
		writel(0xFFFFFFFF, base + AD3SR_OFF);
		writel(0, base + AD3ER_OFF);

		tmp = readl(base + PWER_OFF);
		if (src.bits.rtc)
			tmp |= PWER_WERTC;
		if (src.bits.ext0)
			tmp |= (PWER_WER0 | PWER_WEF0);
		if (src.bits.ext1)
			tmp |= (PWER_WER1 | PWER_WEF1);
		writel(tmp, base + PWER_OFF);

		writel(reg_src & AD3ER_MASK, base + AD3ER_OFF);
	}
	if (lp_mode == PXA3xx_PM_DEEPSLEEP) {
		writel(0xFFFFFFFF, base + PWSR_OFF);
		writel(0, base + PWER_OFF);

		tmp = readl(base + PWER_OFF);
		if (src.bits.rtc)
			tmp |= PWER_WERTC;
		if (src.bits.ext0)
			tmp |= (PWER_WER0);
		if (src.bits.ext1)
			tmp |= (PWER_WER1 | PWER_WEF1);
		writel(tmp, base + PWER_OFF);
	}
	if (lp_mode == PXA3xx_PM_STANDBY) {
		writel(0xFFFFFFFF, base + AD2D0SR_OFF);
		writel(0, base + AD2D0ER_OFF);
		writel(reg_src & AD2D0ER_MASK, base + AD2D0ER_OFF);
	}
	if (lp_mode == PXA3xx_PM_LCDREFRESH) {
		writel(0xFFFFFFFF, base + AD1D0SR_OFF);
		writel(0, base + AD1D0ER_OFF);
		/* add the minilcd wakeup event */
		tmp = ((reg_src | PXA3xx_PM_WE_MLCD) & AD1D0ER_MASK);
		writel(tmp, base + AD1D0ER_OFF);
	}
}

static void update_wakeup_source(unsigned int reg_src, pm_wakeup_src_t *src)
{
	if (reg_src & PXA3xx_PM_WE_RTC)
		src->bits.rtc = 1;
	if (reg_src & PXA3xx_PM_WE_OST)
		src->bits.ost = 1;
#ifndef CONFIG_MACH_WOODSTOCK
	if (reg_src & PXA3xx_PM_WE_UART1) {
		src->bits.uart1 = 1;
#ifdef CONFIG_WIFI_HOST_SLEEP
		/* diff WIFI from UART1 */
		src->bits.wifi = 1;
#endif
	}
#endif
	if (reg_src & PXA3xx_PM_WE_UART2)
		src->bits.uart2 = 1;
	if (reg_src & PXA3xx_PM_WE_UART3)
		src->bits.uart3 = 1;
	if (reg_src & PXA3xx_PM_WE_MKEY)
		src->bits.mkey = 1;
	if (reg_src & PXA3xx_PM_WE_DKEY)
		src->bits.dkey = 1;
	if (reg_src & PXA3xx_PM_WE_TSI)
		src->bits.tsi = 1;
	if (reg_src & PXA3xx_PM_WE_MLCD)
		src->bits.mlcd = 1;
#ifdef CONFIG_MACH_ZYLONITE
	if (reg_src & PXA3xx_PM_WE_SSP) {
		if (wakeup_src.bits.mmc1_cd)
			src->bits.mmc1_cd = 1;
		if (wakeup_src.bits.mmc2_cd)
			src->bits.mmc2_cd = 1;
	}
	if (reg_src & PXA3xx_PM_WE_SSP2)
		src->bits.mmc3_cd = 1;
#endif
	if (reg_src & PXA3xx_PM_WE_MMC1) {
               	src->bits.mmc1_dat1 = 1;
#ifdef CONFIG_WIFI_HOST_SLEEP
		/* diff WIFI from UART1 */
		src->bits.wifi = 1;
#endif
	}
	if (reg_src & PXA3xx_PM_WE_MMC2) {
               	src->bits.mmc2_dat1 = 1;
#ifdef CONFIG_WIFI_HOST_SLEEP
		/* diff WIFI from UART1 */
		src->bits.wifi = 1;
#endif
	}
	if (reg_src & PXA3xx_PM_WE_MMC3) {
#ifdef CONFIG_MACH_WOODSTOCK
		src->bits.motion = 1;
#else
               	src->bits.mmc3_dat1 = 1;
#ifdef CONFIG_WIFI_HOST_SLEEP
		/* diff WIFI from UART1 */
		src->bits.wifi = 1;
#endif
#endif /*!WOODSTOCK*/
	}
	if (reg_src & PXA3xx_PM_WE_EXTERNAL0)
		src->bits.ext0 = 1;
	if (reg_src & PXA3xx_PM_WE_EXTERNAL1)
		src->bits.ext1 = 1;
#ifdef CONFIG_MACH_WOODSTOCK
	if (reg_src & PXA3xx_PM_WE_UART1)
		src->bits.pmirq = 1;
	if (reg_src & PXA3xx_PM_WE_GENERIC9)
		src->bits.i2c = 1;
#endif
}

static void pm_query_wakeup_src(struct pmu_regs *context)
{
	unsigned char __iomem *base = context->membase;
	struct pxa3xx_pm_regs *p = NULL;
	unsigned int tmp;

	memset(&waked, 0, sizeof(pm_wakeup_src_t));
	p = container_of(context, struct pxa3xx_pm_regs, pmu);
	tmp = readl(base + ASCR_OFF);
	writel(tmp, base + ASCR_OFF);

	if (tmp & 0x4) {
		/* check D1 wakeup source */
		tmp = readl(base + AD1D0SR_OFF);
		writel(tmp, base + AD1D0SR_OFF);
		update_wakeup_source(tmp, &waked);
	} else if (tmp & 0x2) {
		/* check D2 wakeup source */
		tmp = readl(base + AD2D0SR_OFF);
		writel(tmp, base + AD2D0SR_OFF);
		update_wakeup_source(tmp, &waked);
	} else if (tmp & 0x1) {
		/* check D3 wakeup source */
		tmp = readl(base + AD3SR_OFF);
		writel(tmp, base + AD3SR_OFF);
		update_wakeup_source(tmp, &waked);
		tmp = readl(base + PWSR_OFF);
		writel(tmp, base + PWSR_OFF);
		if (tmp & PWSR_EERTC)
			waked.bits.rtc = 1;
		if (tmp & PWSR_EDR0)
			waked.bits.ext0 = 1;
		if (tmp & PWSR_EDR1)
			waked.bits.ext1 = 1;
	} else {
		/* check S3 wakeup source */
		tmp = readl(base + ARSR_OFF);
		writel(tmp, base + ARSR_OFF);
		if (tmp & 0x4) {
			/* wakeup from S3 */
			tmp = readl(base + PWSR_OFF);
			writel(tmp, base + PWSR_OFF);
			if (tmp & PWSR_EERTC)
				waked.bits.rtc = 1;
			if (tmp & PWSR_EDR0)
				waked.bits.ext0 = 1;
			if (tmp & PWSR_EDR1)
				waked.bits.ext1 = 1;
		}
	}
}

/* set the default wakeup source */
static void pm_init_wakeup_src(pm_wakeup_src_t *src, struct pmu_regs *context)
{
#ifdef CONFIG_MACH_WOODSTOCK
	src->bits.mkey = 0;
	src->bits.dkey = 0;
	src->bits.ost = 1;
	src->bits.ext0 = 0; //XXX
	src->bits.motion = 0;
	src->bits.pmirq = 1; //XXX
	src->bits.i2c = 0;
#else /*WOODSTOCK*/
	src->bits.ext0 = 1;
	src->bits.rtc = 1;
	src->bits.mkey = 1;
	src->bits.dkey = 1;
	src->bits.mmc1_cd = 1;
	src->bits.mmc1_dat1 = 1;
	src->bits.mmc2_dat1 = 1;
#ifdef CONFIG_CPU_PXA310
	src->bits.mmc3_dat1 = 1;
#endif
#ifdef CONFIG_CPU_PXA320
	src->bits.tsi = 1;
#endif
#ifdef CONFIG_WIFI_HOST_SLEEP
	src->bits.wifi = 1;
#endif
#endif /*!WOODSTOCK*/

	/* clear the related wakeup source */
	pm_clear_wakeup_src(*src);
}

void pm_set_wakeups(unsigned int flags)
{
    if (flags&SF_MOTION) wakeup_src.bits.motion=1; else wakeup_src.bits.motion=0;
    if (flags&SF_KEY) wakeup_src.bits.dkey=1; else wakeup_src.bits.dkey=0;
    if (flags&SF_TOUCH) wakeup_src.bits.i2c=1; else wakeup_src.bits.i2c=0;
}

extern unsigned int micco_event;
int pm_wake_reason(void)
{
    if (waked.bits.dkey) return WAKE_REASON_KEY;
    if (waked.bits.ost) return WAKE_REASON_TIMEOUT;
    if (waked.bits.motion) return WAKE_REASON_MOTION;
    if ((waked.bits.ext0)||(waked.bits.pmirq)) {
        if ((micco_event&PMIC_EVENT_CHDET)||
            (micco_event&PMIC_EVENT_VBUS)) return WAKE_REASON_DOCK;
        if ((micco_event&PMIC_EVENT_TBAT)||
            (micco_event&PMIC_EVENT_VBATMON)||
            (micco_event&PMIC_EVENT_CH_CCTO)||
            (micco_event&PMIC_EVENT_CH_TCTO)) return WAKE_REASON_BAT;
    }
    if (waked.bits.i2c) return WAKE_REASON_TOUCH;
    return WAKE_REASON_NONE;
}

#if 0
static void dump_wakeup_src(pm_wakeup_src_t *src)
{
	printk(KERN_DEBUG "%s entry\n", __FUNCTION__);
	printk(KERN_DEBUG "wakeup source: ");
	if (src->bits.rtc)
		printk(KERN_DEBUG "rtc, ");
	if (src->bits.ost)
		printk(KERN_DEBUG "ost, ");
	if (src->bits.wifi)
		printk(KERN_DEBUG "wifi, ");
	if (src->bits.uart1)
		printk(KERN_DEBUG "uart1, ");
	if (src->bits.uart2)
		printk(KERN_DEBUG "uart2, ");
	if (src->bits.uart3)
		printk(KERN_DEBUG "uart3, ");
	if (src->bits.mkey)
		printk(KERN_DEBUG "mkey, ");
	if (src->bits.dkey)
		printk(KERN_DEBUG "dkey, ");
	if (src->bits.mlcd)
		printk(KERN_DEBUG "mlcd, ");
	if (src->bits.tsi)
		printk(KERN_DEBUG "tsi, ");
	if (src->bits.ext0)
		printk(KERN_DEBUG "ext0, ");
	if (src->bits.ext1)
		printk(KERN_DEBUG "ext1, ");
	if (src->bits.mmc1_cd)
		printk(KERN_DEBUG "mmc1 card detect, ");
	if (src->bits.mmc2_cd)
		printk(KERN_DEBUG "mmc2 card detect, ");
	if (src->bits.mmc3_cd)
		printk(KERN_DEBUG "mmc3 card detect, ");
	if (src->bits.mmc1_dat1)
               printk(KERN_DEBUG "mmc1 dat1, ");
	if (src->bits.mmc2_dat1)
               printk(KERN_DEBUG "mmc2 dat1, ");
	if (src->bits.mmc3_dat1)
               printk(KERN_DEBUG "mmc3 dat1, ");
	if (src->bits.motion)
		printk(KERN_DEBUG "motion, ");
	if (src->bits.pmirq)
		printk(KERN_DEBUG "pmirq, ");
	if (src->bits.i2c)
		printk(KERN_DEBUG "i2c, ");
}
#else
#define dump_wakeup_src(arg)
#endif

void get_wakeup_source(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	memcpy(src, &waked, sizeof(pm_wakeup_src_t));
}
EXPORT_SYMBOL(get_wakeup_source);

/*************************************************************************/
static void flush_cpu_cache(void)
{
	__cpuc_flush_kern_all();
	__cpuc_flush_l2cache_all();
}

struct os_header {
	int	version;
	int	identifier;
	int	address;
	int	size;
	int	reserved;
};

#ifdef CONFIG_MACH_WOODSTOCK
void woodstock_i2c_restore(void);
void woodstock_lcd_init(void);
#endif

#undef PROFILE_WAKE
#ifdef PROFILE_WAKE
void start_wake_profiler(void)
{
    OMCR5=0x80;
    OSMR5=0xFFFFFFFF;
    OSCR5=0;
    OMCR5=0x84;
}

unsigned int get_wake_profiler_time(void)
{
    return OSCR5;
}
#endif

void _pxa3xx_dmemc_init(void);

#ifdef CONFIG_MACH_WOODSTOCK
extern void mma_resume_ssp(void);
#endif

static int pxa3xx_pm_enter_sleep(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned char __iomem *base = pm_regs->pmu.membase;
	unsigned int tmp;

	pxa3xx_sysbus_save(pm_regs);

	if (is_wkr_2140448())
		sleep_wkr_start(0xf6f50084);

	pm_select_wakeup_src(PXA3xx_PM_SLEEP, wakeup_src, &(pm_regs->pmu));

	pxa3xx_pm_set_cken();

	/* should set:modeSaveFlags, areaAddress, flushFunc, psprAddress,
	 * extendedChecksumByteCount */
	pm_regs->pm_data.modeSaveFlags = 0x3f;	/* PM_MODE_SAVE_FLAG_SVC; */
	pm_regs->pm_data.flushFunc = flush_cpu_cache;
	pm_regs->pm_data.areaAddress = (unsigned int)&(pm_regs->pm_data);
	pm_regs->pm_data.psprAddress = (unsigned int)base + PSPR_OFF;
	pm_regs->pm_data.extendedChecksumByteCount =
		sizeof(struct pxa3xx_pm_regs) - sizeof(struct pm_save_data);
	pr_debug("ext size:%d, save size%d\n",
		pm_regs->pm_data.extendedChecksumByteCount,
		sizeof(struct pm_save_data));

	/* save the resume back address into SDRAM */
	pm_regs->word0 = readl(pm_regs->data_pool);
	pm_regs->word1 = readl(pm_regs->data_pool + 4);
	writel(virt_to_phys(pxa3xx_cpu_resume), pm_regs->data_pool);
	writel(virt_to_phys(&(pm_regs->pm_data)), pm_regs->data_pool + 4);

	pxa3xx_clear_pm_status(base, 1);

	/* make sure that sram bank 0 is not off */
	tmp = readl(base + AD3R_OFF);
	tmp |= 0x101;
	// tmp &= 0xfffffff6; // SRAM off
	writel(tmp, base + AD3R_OFF);

#ifdef CONFIG_MACH_ZYLONITE
	/*
	 * set MFP (MFP_SCL: 0x141, MFP_SDA: 0x141, MFP_AC97_nACRESET: 0x40)
	 */
	pxa3xx_mfp_set_edge(MFP_SCL, MFP_EDGE_NONE);
	pxa3xx_mfp_set_afds(MFP_SCL, MFP_AF1, 0);
	pxa3xx_mfp_set_lpm(MFP_SCL, 2);
	pxa3xx_mfp_set_edge(MFP_SDA, MFP_EDGE_NONE);
	pxa3xx_mfp_set_afds(MFP_SDA, MFP_AF1, 0);
	pxa3xx_mfp_set_lpm(MFP_SCL, 2);
	pxa3xx_mfp_set_edge(MFP_AC97_nACRESET, MFP_EDGE_NONE);
	pxa3xx_mfp_set_afds(MFP_AC97_nACRESET, MFP_AF0, 0);
	pxa3xx_mfp_set_lpm(MFP_AC97_nACRESET, 0);
	PCFR |= PCFR_L1DIS;
	PCFR &= ~PCFR_L0EN;
#endif

#ifdef CONFIG_MACH_WOODSTOCK
	PCFR |= PCFR_L1DIS;
	PCFR &= ~PCFR_L0EN;
	PVCR |= 0xC0000000; /*set PVE,FVE*/
#endif
	pr_debug("ready to sleep:0x%lx\n", virt_to_phys(&(pm_regs->pm_data)));

 	/* go to Zzzz */
	pxa3xx_cpu_sleep((unsigned int)&(pm_regs->pm_data),
			virt_to_phys(&(pm_regs->pm_data)));

	/* come back */
#ifdef CONFIG_MACH_WOODSTOCK
	PVCR &= 0x3FFFFFFF; /*clear PVE,FVE*/
#endif
	if (is_wkr_2140448())
		sleep_wkr_end(0xf6f50084);

	writel(pm_regs->word0, pm_regs->data_pool);
	writel(pm_regs->word1, pm_regs->data_pool + 4);

	_pxa3xx_dmemc_init();

	pxa3xx_pm_restore_cken();

#ifdef PROFILE_WAKE
	start_wake_profiler();
#endif
	pm_query_wakeup_src(&(pm_regs->pmu));
	dump_wakeup_src(&waked);
	// XXX BT moved pm_clear_wakeup_src up here because it sure
	// XXX seems to me more sensible to do this before the sysbus_restore
	// XXX rather than after since sysbus_restore may in fact clobber
	// XXX some stuff that pm_clear_wakeup_src wants - this is at
	// XXX least actually true for the OST, maybe other things...
	pm_clear_wakeup_src(wakeup_src);
	pxa3xx_sysbus_restore(pm_regs);
#ifdef CONFIG_MACH_WOODSTOCK
	// XXX BT this is required after pxa3xx_sysbus_restore because
	// XXX it's technically wrong to read the I2C data register and
	// XXX assume that the value you read from it is the value that
	// XXX was last written to it.  At least ordinarily, if the pin
	// XXX is set to be an output at the moment then this will be true
	// XXX however if it's set to be an input then what you read from
	// XXX the data register is the value of the pin, not the contents
	// XXX of the write latch.  It can happen that a particular GPIO
	// XXX is sometimes an input and sometimes an output and we care
	// XXX about the value in the write latch.  If the data register
	// XXX is saved while the pin is an input (normally true for the
	// XXX bit-banged I2C at least) then you lose.
	woodstock_i2c_restore();
	woodstock_lcd_init();
	// XXX BT also restore the SSP SPI interface to the accelerometer
	// before clearing RDH below
	mma_resume_ssp();
#endif

	pxa3xx_clear_pm_status(base, 1);

	/* clear RDH */
	tmp = readl(base + ASCR_OFF);
	tmp &= ~ASCR_RDH;
	writel(tmp, base + ASCR_OFF);

	// XXX BT used to pm_clear_wakeup_src(wakeup_src); here
	// XXX now done above before the sysbus_restore

	/* Clear this bit after returns from low power mode.
	 * Clear this bit can save power.
	 */
	pxa_set_cken(CKEN_HSIO2, 0);

#ifdef CONFIG_IPM
	pm_updatetimer(IPM_UITIMER);
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_SUSPEND_WAKEUP, PM_SUSPEND_MEM,
				&waked, sizeof(pm_wakeup_src_t));
	}
#endif

	pr_debug("Resume Back\n");
#ifdef PROFILE_WAKE
	printk("Took %u us to end of pxa3xx_pm_enter_sleep()\n",get_wake_profiler_time());
#endif

	return 0;
}

static int pxa3xx_pm_enter_deepsleep(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned char __iomem *base = pm_regs->pmu.membase;

	if (is_wkr_2140448()) {
		/*
		 * pr_debug("can't supported deepsleep under this stepping\n");
		 * return 0;
		 */
	}
	pm_select_wakeup_src(PXA3xx_PM_DEEPSLEEP, wakeup_src, &(pm_regs->pmu));

	pxa3xx_pm_set_cken();

	pm_regs->pm_data.modeSaveFlags = 0x3f;	/* PM_MODE_SAVE_FLAG_SVC; */
	pm_regs->pm_data.areaAddress = (unsigned long)&(pm_regs->pm_data);
	pm_regs->pm_data.flushFunc = flush_cpu_cache;
	pm_regs->pm_data.psprAddress = (unsigned int)base + PSPR_OFF;

	pxa3xx_clear_pm_status(base, 1);

	/* go to Zzzz */
	pxa3xx_cpu_deepsleep((unsigned int)&(pm_regs->pm_data),
			virt_to_phys(&(pm_regs->pm_data)));

	return 0;
}

static int pxa3xx_pm_enter_standby(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned char __iomem *base = pm_regs->pmu.membase;

	/* This bit must be enabled before entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 1);

	pxa3xx_clear_pm_status(base, 0);
	/* make sure that sram bank 0 is not off */
#ifdef CONFIG_CPU_PXA320
	writel(0x13F, base + AD2R_OFF);
#else
	writel(0x109, base + AD2R_OFF);
#endif
	if (is_wkr_2140448())
		sleep_wkr_start(0xf6f50084);

	pm_select_wakeup_src(PXA3xx_PM_STANDBY, wakeup_src, &(pm_regs->pmu));

	pxa3xx_cpu_standby((unsigned int)pm_regs->sram_map + 0x8000);

	if (is_wkr_2140448())
		sleep_wkr_end(0xf6f50084);

	pm_query_wakeup_src(&(pm_regs->pmu));
	dump_wakeup_src(&waked);

	pxa3xx_clear_pm_status(base, 0);
	pm_clear_wakeup_src(wakeup_src);
	/* This bit must be disabled after entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 0);

#ifdef CONFIG_IPM
	pm_updatetimer(IPM_UITIMER);
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_STANDBY_WAKEUP, PM_SUSPEND_STANDBY,
				&waked, sizeof(pm_wakeup_src_t));
	}
#endif
	pr_debug("*** made it back from standby\n");

	return 0;
}

#ifdef CONFIG_FB_PXA_MINILCD
extern int pxafb_minilcd_enter(void);
extern int pxafb_minilcd_exit (void);
#endif

static int pxa3xx_pm_enter_lcdrefresh(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned char __iomem *base = pm_regs->pmu.membase;

	/* This bit must be enabled before entering low power mode. */
	pxa_set_cken(CKEN_HSIO2, 1);

	/* enable miniLCD clock and WKUP_EN in miniLCD counter register.
	 * Without these operations, system can't enter D1 from D0.
	 */
	pxa_set_cken(CKEN_MINI_LCD, 1);
	pxa_set_cken(CKEN_MINI_IM, 1);
	
#if defined(CONFIG_FB_PXA_LCD_VGA)
	MLFRMCNT = 0x80000000;		/* Enable WKUP_EN */
#endif

#if defined(CONFIG_FB_PXA_MINILCD) && defined(CONFIG_FB_PXA_LCD_QVGA)
	pxafb_minilcd_enter();
#endif

	pxa3xx_clear_pm_status(base, 0);
	/* make sure that sram bank 0 is not off */
#ifdef CONFIG_CPU_PXA320
	writel(0x13F, base + AD1R_OFF);
#else
	writel(0x109, base + AD1R_OFF);
#endif
	if (is_wkr_2140448())
		sleep_wkr_start(0xf6f50084);

	pm_select_wakeup_src(PXA3xx_PM_LCDREFRESH, wakeup_src, &(pm_regs->pmu));

	pxa3xx_cpu_lcdrefresh((unsigned int)pm_regs->sram_map + 0x8000);

	if (is_wkr_2140448())
		sleep_wkr_end(0xf6f50084);

	pm_query_wakeup_src(&(pm_regs->pmu));
	dump_wakeup_src(&waked);
	pm_clear_wakeup_src(wakeup_src);

	pxa3xx_clear_pm_status(base, 0);
#if defined(CONFIG_FB_PXA_MINILCD) && defined(CONFIG_FB_PXA_LCD_QVGA)
	pxafb_minilcd_exit();
#endif
	/* This bit must be disabled after exiting low power mode. */
	pxa_set_cken(CKEN_HSIO2, 0);
	pxa_set_cken(CKEN_MINI_LCD, 0);
	pxa_set_cken(CKEN_MINI_IM, 0);

#ifdef CONFIG_IPM
	pm_updatetimer(IPM_UITIMER);
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_STANDBY_WAKEUP, PM_SUSPEND_LCDREFRESH,
				&waked, sizeof(pm_wakeup_src_t));
	}
#endif

	pr_debug("*** made it back from lcdrefresh\n");

	return 0;
}

#ifdef CONFIG_IPM
void pm_timeout_proc(unsigned long ptr)
{
	/* just tell policy maker that timer is timeout. */
	if (event_notify)
		event_notify(IPM_EVENT_UITIMER, 0, NULL, 0);
}

/*
 * Flag is used to say whether it was called from UI update or from
 * sleep call. In UI device call it with flag = 0, in sleep return
 * call it with flag = 1.
 */
int pm_updatetimer(int flag)
{
	if (flag == IPM_WAKEUPTIMER)
		mod_timer(&pm_timer, jiffies + pm_waketime*HZ);
	else if (flag == IPM_UITIMER)
		mod_timer(&pm_timer, jiffies + pm_uitimeout*HZ);
	else {
		/* not such timer */
	}
	return 0;
}

EXPORT_SYMBOL(pm_updatetimer);

#endif

static int pxa3xx_pm_enter(suspend_state_t state)
{
#if 0
#ifdef CONFIG_FB_PXA
	/* unless we are entering "lcdrefresh", turn off backlight */
	if (state != PM_SUSPEND_LCDREFRESH)
		pxa3xx_gpio_set_level(14, 0);
#endif
#endif
	if (state == PM_SUSPEND_MEM)
		return pxa3xx_pm_enter_sleep(&pxa3xx_pm_regs);
	else if (state == PM_SUSPEND_STANDBY)
		return pxa3xx_pm_enter_standby(&pxa3xx_pm_regs);
	else if (state == PM_SUSPEND_LCDREFRESH)
		return pxa3xx_pm_enter_lcdrefresh(&pxa3xx_pm_regs);
	else if (state == PM_SUSPEND_DEEPSLEEP)
		return pxa3xx_pm_enter_deepsleep(&pxa3xx_pm_regs);
	else
		return -EINVAL;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int pxa3xx_pm_prepare(suspend_state_t state)
{
	struct os_header header;
	int obm_ver;

	pr_debug("%s: enter\n", __FUNCTION__);
#ifdef CONFIG_IPM
	/* Disable deep idle when system enters low power mode */
	save_deepidle = enable_deepidle;
	enable_deepidle = 0;
#endif
	if (state == PM_SUSPEND_MEM) {
#if 1 /*XXX BT get away without this at the cost of reload latency?*/
		/* backup data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram, pxa3xx_pm_regs.sram_map, ISRAM_SIZE);
		obm_ver = calc_obm_ver();
		if (obm_ver == PXA3xx_OBM_V2) {
			/* load OBM into ISRAM
			 * The target address is 0x5c014000
			 */
/*
			memcpy(pxa3xx_pm_regs.sram_map + MOBM_START - ISRAM_START,
				pxa3xx_pm_regs.obm, MOBM_SIZE);
*/
			memcpy(pxa3xx_pm_regs.sram_map + 0x14000, pxa3xx_pm_regs.obm,
				MOBM_SIZE);
		} else if (obm_ver == PXA3xx_OBM_V3) {
			/* load OBM into ISRAM
			 * The target address is 0x5c013000
			 * The main purpose to load obm is to initialize DDR.
			 * When OBM found it's a resume process, it will jump
			 * to resume routine what resides in DDR.
			 */
			memset(&header, 0, sizeof(struct os_header));
			header.version = 3;
			header.identifier = 0x5265736D;		/* RESM */
			header.address = 0x5c013000;
			header.size = MOBM_SIZE;
			/* 0x5c008000 */
			memcpy(pxa3xx_pm_regs.sram_map + 0x8000, &header,
				sizeof(struct os_header));
			/* 0x5c013000 */
			memcpy(pxa3xx_pm_regs.sram_map + 0x13000, pxa3xx_pm_regs.obm,
				MOBM_SIZE);
		}
#endif
		pm_state = PM_SUSPEND_MEM;
	} else if (state == PM_SUSPEND_STANDBY) {
		/* FIXME: allocat SRAM to execute D1/D2 entry/exit code.
		 * Try not to use it in the future.
		 */
		/* backup data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram, pxa3xx_pm_regs.sram_map, 1024);
		pm_state = PM_SUSPEND_STANDBY;

	} else if (state == PM_SUSPEND_LCDREFRESH) {
		/* FIXME: allocat SRAM to execute D1/D2 entry/exit code.
		 * Try not to use it in the future.
		 */
		/* backup data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram, pxa3xx_pm_regs.sram_map, 1024);
		pm_state = PM_SUSPEND_LCDREFRESH;
	}

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static int pxa3xx_pm_finish(suspend_state_t state)
{
#ifdef CONFIG_IPM
	enable_deepidle = save_deepidle;
#endif
	if (state == PM_SUSPEND_MEM) {
		/* restore data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram_map, pxa3xx_pm_regs.sram, ISRAM_SIZE);
		pm_state = PM_SUSPEND_ON;
	} else if (state == PM_SUSPEND_STANDBY) {
		/* restore data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram_map, pxa3xx_pm_regs.sram, 1024);
		pm_state = PM_SUSPEND_ON;
	} else if (state == PM_SUSPEND_LCDREFRESH) {
		/* restore data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram_map, pxa3xx_pm_regs.sram, 1024);
		pm_state = PM_SUSPEND_ON;
	}

	return 0;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct pm_ops pxa3xx_pm_ops = {
	.pm_disk_mode	= PM_DISK_FIRMWARE,
	.prepare	= pxa3xx_pm_prepare,
	.enter		= pxa3xx_pm_enter,
	.finish		= pxa3xx_pm_finish,
};

#define pm_attr(_name, object) \
static ssize_t _name##_store(struct subsystem * subsys,			\
			const char * buf, size_t n)			\
{									\
	sscanf(buf, "%u", &object);					\
	return n;							\
}									\
static ssize_t _name##_show(struct subsystem * subsys, char * buf)	\
{									\
	return sprintf(buf, "%u\n", object);				\
}									\
static struct subsys_attribute _name##_attr = { 			\
        .attr   = {                             			\
                .name = __stringify(_name),     			\
                .mode = 0644,                   			\
        },                                      			\
        .show   = _name##_show,                 			\
        .store  = _name##_store,                			\
}

#if 0
extern int pxa3xx_fv_enter_low_op(void);

static ssize_t idle_forever_store(struct subsystem *subsys, const char *buf, size_t n)
{
    printk("Entering idle forever (you'll have to disconnect power to recover)\n");
    local_irq_disable();
    pxa3xx_fv_enter_low_op();
    while (1) ;
    /*NOTREACHED*/
    return 0;
}
static struct subsys_attribute idle_forever_attr = {
    .attr = {
        .name = "idle_forever",
        .mode = 0644,
    },
    .store = idle_forever_store,
    .show = 0,
};
#endif

pm_attr(sleeptime, pm_sleeptime);
pm_attr(msleeptime, pm_msleeptime);
#ifdef CONFIG_IPM
pm_attr(waketime, pm_waketime);
pm_attr(uitimeout, pm_uitimeout);
pm_attr(deepidle, enable_deepidle);
pm_attr(idle_ticks, deepidle_ticks);
#endif

extern void (*pm_power_off)(void);
void pxa3xx_power_off(void);

static int __init pxa3xx_pm_init(void)
{
	pm_set_ops(&pxa3xx_pm_ops);

#ifdef CONFIG_IPM
	init_timer(&pm_timer);
	pm_timer.function = pm_timeout_proc;

	if (pm_uitimeout)
		mod_timer(&pm_timer, jiffies + pm_uitimeout*HZ );

	if (sysfs_create_file(&power_subsys.kset.kobj, &waketime_attr.attr))
		return -1;

	if (sysfs_create_file(&power_subsys.kset.kobj, &uitimeout_attr.attr))
		return -1;

	if (sysfs_create_file(&power_subsys.kset.kobj, &deepidle_attr.attr))
		return -1;
	if (sysfs_create_file(&power_subsys.kset.kobj, &idle_ticks_attr.attr))
		return -1;
#endif

	if (sysfs_create_file(&power_subsys.kset.kobj, &sleeptime_attr.attr))
		return -1;

	if (sysfs_create_file(&power_subsys.kset.kobj, &msleeptime_attr.attr))
		return -1;
#if 0
	if (sysfs_create_file(&power_subsys.kset.kobj, &idle_forever_attr.attr))
		return -1;
#endif

	/* set memory base */
	pxa3xx_sysbus_init(&pxa3xx_pm_regs);
	/* set default wakeup src */
	pm_init_wakeup_src(&wakeup_src, &(pxa3xx_pm_regs.pmu));

	pm_power_off=pxa3xx_power_off;

	return 0;
}

#if 1
// XXX BT this doesn't work at least on Woodstock ES1 because the design of
// XXX the reset circuit is such that reset will (incorrectly) assert as
// XXX soon as the DA9034 shuts down and nBATT_FAULT asserts.  I may
// XXX eventually implement a minimum-power S2D3C4 alternative, with
// XXX the core and SRAM turned off and the DDR SDRAM not retaining state
// XXX - if there is a way to do that...
void pxa3xx_power_off(void)
{
    printk("Resetting wifi module and turning off 1.2V supply\n");
    pxa3xx_gpio_set_level(MFP_PIN_GPIO15, GPIO_LEVEL_LOW);
    mdelay(1);
    pxa3xx_gpio_set_level(MFP_PIN_GPIO1_2, GPIO_LEVEL_LOW);
    mdelay(50);
    wakeup_src.bits.rtc=0;
    wakeup_src.bits.ext0=1;
    wakeup_src.bits.ext1=0;
    printk("Entering S3D4C4 mode, goodbye!\n");
    local_irq_disable();
    pxa3xx_pm_enter(PM_SUSPEND_DEEPSLEEP);
    // local_irq_enable();
    printk("We returned from S3D4C4 mode?!\n");
}
#endif

late_initcall(pxa3xx_pm_init);

