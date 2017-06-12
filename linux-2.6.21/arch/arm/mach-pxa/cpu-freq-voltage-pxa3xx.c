/*
 * Core frequency change driver for Monahans
 *
 * Core frequency change driver for Monahans.
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#if 0
#undef DEBUG
#define DEBUG
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa3xx_pmic.h>

#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>


#ifdef DEBUG
#define PRINT_INFO(info)	do {					\
					pr_debug("%s: ", __FUNCTION__);	\
					print_info((info));		\
				} while (0)
#else
#define PRINT_INFO(info)	do {} while (0)
#endif

#define FREQ_CORE(xl, xn)	((xl)*(xn)*13)

#define FREQ_SRAM(sflfs)	(((sflfs) == 0x0)?104:	\
				((sflfs) == 0x1)?156:	\
				((sflfs) == 0x2)?208:312)

#define FREQ_STMM(smcfs)	(((smcfs) == 0x0)?78:	\
				((smcfs) == 0x2)?104:	\
				((smcfs) == 0x5)?208:0)

#define FREQ_DDR(dmcfs)		(((dmcfs) == 0x0)?26:	\
				((dmcfs) == 0x2)?208:	\
				((dmcfs) == 0x3)?260:0)

#define FREQ_HSS(hss)		(((hss) == 0x0)?104:	\
				((hss) == 0x1)?156:	\
				((hss) == 0x2)?208:0)

#define FREQ_DFCLK(smcfs, df_clkdiv)    				\
			(((df_clkdiv) == 0x1)?FREQ_STMM((smcfs)):	\
			((df_clkdiv) == 0x2)?FREQ_STMM((smcfs))/2:	\
			((df_clkdiv) == 0x3)?FREQ_STMM((smcfs))/4:0)

#define FREQ_EMPICLK(smcfs, empi_clkdiv)				\
			(((empi_clkdiv) == 0x1)?FREQ_STMM((smcfs)):	\
			((empi_clkdiv) == 0x2)?FREQ_STMM((smcfs))/2:	\
			((empi_clkdiv) == 0x3)?FREQ_STMM((smcfs))/4:0)

#define LPJ_PER_MHZ 4988

extern int pwri2c_flag;
static volatile u32 pvcr;

LIST_HEAD(pxa3xx_fv_notifier_list);
struct pxa3xx_fv_info pxa3xx_por_op[DVFM_MAX_OP] = {
	/*
	 *{ XL,	XN,	vcc_core, vcc_sram, smcfs,sflfs,
	 * hss, dmcfs, df_clk,
	 * empi_clk, d0cs, lpj
	 *}
	 */
	/* op 0:, ring oscillator mode */
	{0x0, 0x0, 1000, 1100, 0, 0,
	 0, 0, FV_DF_CLKDIV_1,
	 FV_EMPI_CLKDIV_1, 1, 293888
	}, /* those 0 values are meaningless for D0CS mode */

	/* op 1:, 104M, run mode */
	{0x08, 0x1, 1000, 1100, FV_ACCR_SMCFS_78M, FV_ACCR_SFLFS_104M,
	 FV_ACCR_HSS_104M, FV_ACCR_DMCFS_260M, FV_DF_CLKDIV_4,
	 FV_EMPI_CLKDIV_4, 0, 517120
	},

	/* op 2:, 208M, run mode */
	{0x10, 0x1, 1000, 1100, FV_ACCR_SMCFS_104M, FV_ACCR_SFLFS_156M,
	 FV_ACCR_HSS_104M, FV_ACCR_DMCFS_260M, FV_DF_CLKDIV_2,
	 FV_EMPI_CLKDIV_2, 0, 1036288
	},

	/* op 3:, 416M, turbo mode */
	{0x10, 0x2, 1100, 1200, FV_ACCR_SMCFS_104M, FV_ACCR_SFLFS_208M,
	 FV_ACCR_HSS_156M, FV_ACCR_DMCFS_260M, FV_DF_CLKDIV_2,
	 FV_EMPI_CLKDIV_2, 0, 2076672
	},

	/* op 4:, 624M, turbo mode */
	{0x18, 0x2, 1375, 1400, FV_ACCR_SMCFS_208M, FV_ACCR_SFLFS_312M,
	 FV_ACCR_HSS_208M, FV_ACCR_DMCFS_260M, FV_DF_CLKDIV_4,
	 FV_EMPI_CLKDIV_4, 0, 3112960
	},

#if defined(CONFIG_CPU_PXA320)
	/* op 5:, 806M, turbo mode */
   	{0x1F, 0x2, 1400, 1400, FV_ACCR_SMCFS_208M, FV_ACCR_SFLFS_312M,
	 FV_ACCR_HSS_208M, FV_ACCR_DMCFS_260M, FV_DF_CLKDIV_4,
	 FV_EMPI_CLKDIV_4, 0, 4020906 
	},
#endif

	/* END */
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	 0x0, 0x0, 0x0,
	 0x0, 0x0, 0
	},

	/* store the non-standard operating point that is configured at
	 * run time
	 */
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	 0x0, 0x0, 0x0,
	 0x0, 0x0, 0
	},

	/* store the operating point at boot */
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	 0x0, 0x0, 0x0,
	 0x0, 0x0, 0
	},
};

#if defined(CONFIG_DPM) || defined(CONFIG_IPM_DEEPIDLE)
static spinlock_t _dvfm_lock = SPIN_LOCK_UNLOCKED;
#define DVFM_LOCK       spin_lock
#define DVFM_UNLOCK     spin_unlock
#else
static DECLARE_MUTEX (_dvfm_lock);
#define DVFM_LOCK       down
#define DVFM_UNLOCK     up
#endif
/* max number of standard operating point */
static unsigned int max_op = 0;
static unsigned int cur_op = 0;
/* use 208M as default operating point */
static unsigned int def_op = 2;
/* D0CS operating point index */
static unsigned int d0cs_op = 0;
/* save the current op before entering sleep so to restore */
static unsigned int pm_save_op;
/* use lowest freq/vol as the operating point during sleep */
static unsigned int sleep_op = 4;
static int entry_low_op = -1;
#if !defined(CONFIG_FB_PXA_LCD_VGA) && defined(CONFIG_PXA3xx_D0CS)
static int low_op = 0;
#else
static int low_op = 1;		/* 104MHz */
#endif

#define DVFM_REC_NUM	16
#define DVFM_SEC_NUM	10
/* It's used to measure the time on the sequence */
struct fv_time_t {
	unsigned int start;
	unsigned int end;
};

/* It's used to record historical time costs */
struct dvfm_measure_t {
	unsigned int	freq_times;
	unsigned int	avg_time;
	unsigned int	last_time[DVFM_REC_NUM];
};

static struct dvfm_measure_t m_dvfm;

#ifdef CONFIG_DPM
/* Reserve 3 OP as END OP, NONSTD_OP and BOOT_OP */
struct dvfm_context_t {
	unsigned int max_op;
	struct pxa3xx_fv_info pxa3xx_op[DVFM_MAX_OP];
};

static struct dvfm_context_t dvfm_context;
#define END_OP		DVFM_MAX_OP - 3
#define NONSTD_OP	DVFM_MAX_OP - 2
#define BOOT_OP		DVFM_MAX_OP - 1
#else
#define NONSTD_OP (max_op + 1)
#define BOOT_OP (max_op + 2)
#endif


extern void pxa_clkcfg_write(u32);
extern int enter_d0cs_a(volatile u32 *, volatile u32 *);
extern int exit_d0cs_a(volatile u32 *, volatile u32 *);
extern unsigned long loops_per_jiffy;

#ifdef CONFIG_FB_PXA
extern void pxafb_set_pcd(unsigned int pcd);
extern unsigned int pxafb_get_op_pcd(unsigned int hss);
extern unsigned int pxafb_get_d0cs_pcd(void);
extern unsigned int pxafb_get_104m_pcd(void);
#endif

extern struct sysdev_class cpu_sysdev_class;
static int __set_fv(struct pxa3xx_fv_info *,
		struct pxa3xx_fv_info*, unsigned long);
static int find_op(void);

extern void pxa_u_wait(int us);


/*
 * This function is used to get the frequencies including
 * HSS frequency.
 */
int pm_freq_get(struct pxa3xx_fv_info* info,unsigned int *mode)
{
	volatile u32 acsr;
	volatile u32 memclkcfg;

	acsr = ACSR;
	memclkcfg = MEMCLKCFG;

	info->xl = ACCR_G_XL(acsr);
	info->xn = ACCR_G_XN(acsr);
	info->smcfs = ACCR_G_SMCFS(acsr);
	info->sflfs = ACCR_G_SFLFS(acsr);
	info->hss = ACCR_G_HSS(acsr);
	info->dmcfs = ACCR_G_DMCFS(acsr);
	info->df_clk = MEMCLKCFG_G_DFCLKDIV(memclkcfg);
	info->empi_clk = MEMCLKCFG_G_EMPICLKDIV(memclkcfg);
	info->d0cs = ACCR_G_D0CS(acsr);
	if (mode)
		*mode = ACCR_G_XSPCLK(acsr);

	return 0;
}

/*
 * get which frequencies and voltages will be changed
 */
unsigned int pm_freq_vol_preset(
			struct pxa3xx_fv_info* cur,
			struct pxa3xx_fv_info* next,
			unsigned int *pmode)
{
	unsigned int flag = 0, mode = *pmode;

	if (cur->vcc_core != next->vcc_core || cur->vcc_sram != next->vcc_sram)
		flag |= FV_SET_CV;
	if (cur->df_clk!= next->df_clk)
		flag |= FV_SET_DFCLK;
	if (cur->empi_clk!= next->empi_clk)
		flag |= FV_SET_EMPICLK;
	if (cur->d0cs != next->d0cs)
		flag |= FV_SET_D0CS;

	if (next->d0cs)
		goto exit;/* don't change ACCR frequencies in D0CS mode */

	if (cur->xl != next->xl || cur->xn != next->xn)
		flag |= FV_SET_CFS;
	if (cur->smcfs != next->smcfs)
		flag |= FV_SET_SMCFS;
	if (cur->sflfs != next->sflfs)
		flag |= FV_SET_SFLFS;
	if (cur->hss != next->hss)
		flag |= FV_SET_HSS;
	if (cur->dmcfs != next->dmcfs)
		flag |= FV_SET_DMCFS;

exit:
	if (!(flag & FV_SET_CFS))
		mode = (mode & ~FV_MODE_SET_CFS_XSPCLK_MASK)
			| FV_MODE_SET_CFS_XSPCLK_DIS;

	*pmode = mode;

	return flag;
}

/*
 * This function is used to get the VCC_CORE and VCC_SRAM.
 */
int pm_vol_get(unsigned int *vcc_core, unsigned int *vcc_sram)
{
	pxa3xx_pmic_get_voltage(VCC_CORE, vcc_core);
	pxa3xx_pmic_get_voltage(VCC_SRAM, vcc_sram);
	return 0;
}

/*
 * This function is used to change the VCC_CORE and VCC_SRAM.
 */
int pm_vol_change(unsigned int cur_vcc_core, unsigned int cur_vcc_sram,
		unsigned int vcc_core, unsigned int vcc_sram)
{
	/* if power i2c is enabled, voltage will be changed automatically */
	if (!pwri2c_flag) {
		if (vcc_core == cur_vcc_core) vcc_core= -1;
		if (vcc_sram == cur_vcc_sram) vcc_sram= -1;
		return pxa3xx_pmic_set_apps_and_sram(vcc_core,vcc_sram);
	}

	return 0;
}


/* 
 * Return 2 if MTS should be changed to 2.
 * Return 1 if MTS should be changed to 1.
 * Return 0 if MTS won't be changed.
 * In this function, the maxium MTS is 2.
 */
static int check_mts(int xn)
{
        int ret = 0; 
        unsigned int reg;

        reg = ASCR >> ASCR_MTS_S_OFFSET;
        reg = reg & 0x3;

        if ((reg == 1) && (xn == 2))
                ret = 2;
        if ((reg == 2) && (xn == 1))
                ret = 1;
        return ret;
}

static int set_mts(int xn)
{
        unsigned int reg;

        reg = ASCR;
        reg &= ~(3 << ASCR_MTS_OFFSET);
        reg |= (xn << ASCR_MTS_OFFSET);
        ASCR = reg;

        /* wait MTS is set */
        while (((ASCR >> ASCR_MTS_OFFSET) & 0x3)
                != ((ASCR >> ASCR_MTS_S_OFFSET) & 0x3));
        return 0;
}

/* Input: 208, 260MHz */
static int set_dmc(int ddr)
{
        int pll; 
        unsigned int reg;

        switch (ddr) {
        case 208:
                pll = 3;
                break;
        case 260:
        default:
                pll = 2;
                break;
        }
        for (;;) {
                reg = MDCNFG; 
                reg &= ~(3 << 28);
                reg |= (pll << 28);
                MDCNFG = reg;

                if (((MDCNFG >> 28) & 3) == pll)
                        break;
        }

        reg = DDR_HCAL;
        reg |= (1 << 31);
        DDR_HCAL = reg;

        return 0;
}

/* set DF and EMPI divider */
static int set_df(int smc)
{
        unsigned int reg;
	int fix_empi, cpuid;

	cpuid = read_cpuid(0);
	cpuid &= 0xFFFF;
	if (((cpuid >= 0x6880) && (cpuid <= 0x6881)) 
		|| ((cpuid >= 0x6890) && (cpuid <= 0x6892)))
		/* It's MonahansL or MonahansLV */
		fix_empi = 1;
	else
		fix_empi = 0;
	reg = MEMCLKCFG;
	reg &= ~((7 << 16) | 7);
	if (fix_empi) {
		reg |= (3 << MEMCLKCFG_EMPI_OFFSET);
		switch (smc) {
		case 208:
			/* divider -- 4 */
			reg |= (3 << MEMCLKCFG_DF_OFFSET);
			break;
		case 104:
			/* divider -- 2 */
			reg |= (2 << MEMCLKCFG_DF_OFFSET);
			break;
		case 78:
			/* divider -- 4 */
			reg |= (3 << MEMCLKCFG_DF_OFFSET);
			break;
		case 52:
			/* divider -- 1 */
			reg |= (1 << MEMCLKCFG_DF_OFFSET);
			break;
		}
	} else {
		switch (smc) {
		case 208:
			/* divider -- 4 */
			reg |= (3 << MEMCLKCFG_DF_OFFSET);
			reg |= (3 << MEMCLKCFG_EMPI_OFFSET);
			break;
		case 104:
			/* divider -- 2 */
			reg |= (2 << MEMCLKCFG_DF_OFFSET);
			reg |= (2 << MEMCLKCFG_EMPI_OFFSET);
			break;
		case 78:
			/* divider -- 4 */
			reg |= (3 << MEMCLKCFG_DF_OFFSET);
			reg |= (3 << MEMCLKCFG_EMPI_OFFSET);
			break;
		case 52:
			/* divider -- 1 */
			reg |= (1 << MEMCLKCFG_DF_OFFSET);
			reg |= (1 << MEMCLKCFG_EMPI_OFFSET);
			break;
		}
	}
	MEMCLKCFG = reg;
	return 0;
}

/* set bus frequency except HSS */
static int set_bus_freq(struct pxa3xx_fv_info *cur,
                        struct pxa3xx_fv_info *next)
{
	unsigned int reg, mask;
	int timeout;
	
	if (cur->dmcfs != next->dmcfs) {
		/* prepare DMEMC for frequency change */
		set_dmc(FREQ_DDR(next->dmcfs));
	}
	
	if (FREQ_STMM(next->smcfs) == 208) {
		/* set DFI clock divider to 4 */
		set_df(FREQ_STMM(next->smcfs));
	}
	
	reg = ACCR;
	mask = 0;
	if (cur->smcfs != next->smcfs) {
		reg &= ~ACCR_SMCFS_MASK;
		reg |= (next->smcfs << ACCR_SMCFS_OFFSET);
		mask |= ACCR_SMCFS_MASK;
	}
	if (cur->sflfs != next->sflfs) {
		reg &= ~ACCR_SFLFS_MASK;
		reg |= (next->sflfs << ACCR_SFLFS_OFFSET);
		mask |= ACCR_SFLFS_MASK;
	}
#if 0
        if (cur->hss != next->hss) {
        	reg &= ~ACCR_HSS_MASK;
        	reg |= (next->hss << ACCR_HSS_OFFSET);
        	mask |= ACCR_HSS_MASK;
        }
#endif
	if (cur->dmcfs != next->dmcfs) {
		reg &= ~ACCR_DMCFS_MASK;
		reg |= (next->dmcfs << ACCR_DMCFS_OFFSET);
		mask |= ACCR_DMCFS_MASK;
	}
	ACCR = reg;
	
	/* wait until ACSR is changed */
	while ((ACCR & mask) != (ACSR & mask));
	
	timeout = 10; 
	while (MDCNFG & (3 << 28)) {
		pxa_u_wait(1);
		if (--timeout == 0) {
			printk(KERN_WARNING "MDCNFG[29:28] isn't zero\n");
			break;
		}
	}
	
	if (FREQ_STMM(next->smcfs) == 104) {
		/* set DFI clock divider to 2 */
		set_df(FREQ_STMM(next->smcfs));
	}
	
	return 0;
}

/*
 * This function is used to change the frequencies except HSS.
 */
int pm_freq_change_nohss(struct pxa3xx_fv_info* cur,
			struct pxa3xx_fv_info* next,
			unsigned int mode)
{
	volatile unsigned int reg;

	/* enter 104M at first before entering any operating point */
	reg = ACCR;
	reg &= ~(ACCR_XL_MASK | ACCR_XN_MASK | ACCR_XSPCLK_MASK);
	reg |= (( 8 << ACCR_XL_OFFSET) | (1 << ACCR_XN_OFFSET)
		| (mode << ACCR_XSPCLK_OFFSET));
	ACCR = reg;
	reg = ACCR;
	reg += 1;

	/* delay 2 cycles of 13MHz clock */
	/* XXX BT this is NOT 2 cycles of the 13MHz clock, it's approximately
	 * 2us, which is much more.  I'm not going to mess with what
	 * Marvell put here, but I do believe that the documentation says
	 * you only have to wait 2 13MHz clocks
	 */
	pxa_u_wait(2);
	pxa_clkcfg_write(XCLKCFG_SET(1, 0));

	while ((ACCR & (ACCR_XL_MASK | ACCR_XN_MASK))
		!= (ACSR & (ACCR_XL_MASK | ACCR_XN_MASK)));

	// XXX BT I moved all the MTS setting logic up here.  The original
	// code had strange voodoo in which MTS setting was split into two
	// pieces at slightly different places (but with no intervening
	// frequency changes!), one for the MTS -> 1 case and one for the
	// MTS -> 2 case.  I cannot possibly see any good reason for that.
	// For that matter, when operating with FVE set to 0 at all times,
	// as we do, I can't imagine that there's actually any need to bother
	// setting MTS at all ever - because MTS only affects the operation
	// of the automatic voltage control on frequency change -
	// which is disabled when FVE is set to 0.  That being said, I'm
	// leaving the set_mts calls in.
	switch(check_mts(next->xn)) {
		case 1:
			set_mts(1);
			break;
		case 2:
			set_mts(2);
			break;
		default:
			;
	}
	// XXX BT all the following used to be executed unconditionally
	// but it's silly and a little time consuming to switch to
	// 104MHz mode twice in a row...
	if (next->xl!=8||next->xn!=1) {
		/* set xl and xn */
		reg = ACCR;
		reg &= ~(ACCR_XL_MASK | ACCR_XN_MASK | ACCR_XSPCLK_MASK);
		reg |= ((next->xl << ACCR_XL_OFFSET) | (next->xn << ACCR_XN_OFFSET)
			| (mode << ACCR_XSPCLK_OFFSET));
		ACCR = reg;
		reg = ACCR;
		reg += 1;
		/* delay 2 cycles of 13MHz clock */
		pxa_u_wait(2);
		
		/* set F or T bit */
		if (next->xl == 0x8 && next->xn != 0x1) {
			pxa_clkcfg_write(XCLKCFG_SET(0, 1));
		} else {
			pxa_clkcfg_write(XCLKCFG_SET(1, 0));
		}
		while ((ACCR & (ACCR_XL_MASK | ACCR_XN_MASK))
			!= (ACSR & (ACCR_XL_MASK | ACCR_XN_MASK)));
	}
	// XXX BT I doubt that the following pxa_u_wait is required
	pxa_u_wait(2);
	set_bus_freq(cur, next);
	
	
	/* XXX BT putting this here in hopes that nothing
	 * (like, oh, perhaps bit-banged I2C routines on Woodstock)
	 * will miscompute things like udelays
	 */
	loops_per_jiffy = next->lpj;
	return 0;
}

/*
 * This function is used to change HSS. It is used by
 * OS LCD driver's interrupt handler to call it when
 * handling LCD EOF interrupts.
 */
int pm_hss_change(u32 hss)
{
	volatile u32  accr;

	ACCR = ACCR_S_HSS(ACCR, hss);
	accr = ACCR;

	/*
	 * wait for HSS change completed
	 */
	while((ACSR & ACCR_HSS_MASK)!=
		(accr & ACCR_HSS_MASK));

	return 0;
}

/*
 * This function is used to change the frequencies,
 * including HSS. Voltage changes won't be performed here.
 */
int pm_freq_change(struct pxa3xx_fv_info* cur,
		struct pxa3xx_fv_info* next,
		unsigned int mode)
{
	int ret;
	unsigned int flag;
	unsigned int fv_mode = mode;

	ret = pm_freq_change_nohss(cur, next, mode);
	if (ret)
		return ret;

	/* determine if HSS or D0CS needs operation */
	flag = pm_freq_vol_preset(cur, next, &fv_mode);
	if (flag & FV_SET_HSS) {
		ret = pm_hss_change(next->hss);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * This function is used to change the frequencies except HSS.
 * Both frequencies and voltages changes will be performed here.
 */
int pm_freq_vol_change_nohss(struct pxa3xx_fv_info* cur,
				struct pxa3xx_fv_info* next,
				unsigned int mode)
{
	int ret;
	unsigned int fv_mode = mode;
	unsigned int flag;

	/* determine if voltages needs to be changed */
	flag = pm_freq_vol_preset(cur, next, &fv_mode);

	if ((flag & FV_SET_CV) && (next->vcc_core >= cur->vcc_core)) {
		ret = pm_vol_change(cur->vcc_core, cur->vcc_sram,
					next->vcc_core, next->vcc_sram);
		if (ret)
			return ret;
	}

	ret = pm_freq_change_nohss(cur, next, fv_mode);
	if (ret)
		return ret;

	if ((flag & FV_SET_CV) && (next->vcc_core < cur->vcc_core)) {
		ret = pm_vol_change(cur->vcc_core, cur->vcc_sram,
					next->vcc_core, next->vcc_sram);
		if (ret)
			return ret;
	}

	return 0;
}
int pm_freq_vol_get(struct pxa3xx_fv_info *info, unsigned int *mode)
{
	int ret;

	ret = pm_freq_get(info, mode);
	if (ret)
		return ret;

	ret = pm_vol_get(&info->vcc_core, &info->vcc_sram);
	if (ret)
		return ret;

	return 0;
}

/*
 * Put system into D0CS mode.
 */
int pm_enter_d0cs(struct pxa3xx_fv_info *info)
{
	volatile u32 mdrefr;
	volatile u32 accr;
	int ret = 0;
	volatile u32 reg;

	/* This bit must be set before entering/exiting D0CS mode. */
	pxa_set_cken(CKEN_HSIO2, 1);

	/* FIXME: disable FVE mode. If enable FVE mode, entering D0CS
	 * mode will take 800us. It just takes 26us if disable FVE mode.
	 * We will enabel FVE mode when exit D0CS mode.
	 */
	if (pwri2c_flag) {
		pvcr = PVCR;
		reg = pvcr;
		reg &= ~0x80000000;
		PVCR = reg;
		reg = PVCR;
	}

	/* enter D0CS mode*/
	if (enter_d0cs_a((volatile u32 *)&ACCR, (volatile u32 *)&MDCNFG)) {
		ret = -EIO;
		goto out;
	}

	info->xl = 0;
	info->xn = 0;
	info->d0cs = 1;

	/* disable system PLL and core PLL */
	/*
	 * delay 100us to ensure that ring oscillator is stable enough for
	 * D0CKEN_k and D0CKEN_B clocks before turning off two PLLs
	 */
	pxa_u_wait(100);

	accr = ACCR;
	accr = ACCR_S_SPDIS(accr, 0x1);
	accr = ACCR_S_XPDIS(accr, 0x1);
	ACCR = accr;
	accr = ACCR;
	while(ACSR & (ACCR_SPCLK_MASK | ACCR_XPCLK_MASK));

	/* refresh SDRAM after core frequency change*/
	mdrefr = MDREFR;
	MDREFR = mdrefr;

	pxa_set_cken(CKEN_HSIO2, 0);
	return 0;
out:
	pxa_set_cken(CKEN_HSIO2, 0);
	return ret;
}

int pm_exit_d0cs(struct pxa3xx_fv_info *info)
{
	volatile u32  mdrefr;
	volatile u32  accr;
	volatile u32  reg;
	int ret = 0;

	/* This bit must be set before entering/exiting D0CS mode. */
	pxa_set_cken(CKEN_HSIO2, 1);

	/* enable system PLL and core PLL */
	accr = ACCR;
	accr = ACCR_S_SPDIS(accr, 0x0);
	accr = ACCR_S_XPDIS(accr, 0x0);
	ACCR = accr;
	accr = ACCR;
	while ((ACSR & (ACCR_SPCLK_MASK | ACCR_XPCLK_MASK))
		!= (ACCR_SPCLK_MASK | ACCR_XPCLK_MASK));

	/* exit D0CS mode */
	if (exit_d0cs_a((volatile u32 *)&ACCR, (volatile u32 *)&MDCNFG)) {
		ret = -EIO;
		goto out;
	}

	/* FIXME: enable FVE mode. If enable FVE mode, entering D0CS
	 * mode will take 800us. It just takes 26us if disable FVE mode.
	 * So we disable FVE mode when enter D0CS mode and enable it here.
	 */
	if (pwri2c_flag) {
		PVCR = pvcr;
		reg = PVCR;
	}
	
	/* refresh SDRAM after core frequency change*/
	mdrefr = MDREFR;
	MDREFR = mdrefr;

	pxa_set_cken(CKEN_HSIO2, 0);
	return 0;
out:
	pxa_set_cken(CKEN_HSIO2, 0);
	return ret;
}

/*
 * This function is used to change the frequencies.
 * Both frequencies and voltages changes will be performed here.
 */
int pm_freq_vol_change(struct pxa3xx_fv_info* cur,
			struct pxa3xx_fv_info* next,
			unsigned int mode)
{
	int ret;
	unsigned int fv_mode = mode;
	unsigned int flag;

	/* determine if voltages needs to be changed */
	flag = pm_freq_vol_preset(cur, next, &fv_mode);

	if ((flag & FV_SET_CV) && (next->vcc_core >= cur->vcc_core)) {
		ret = pm_vol_change(cur->vcc_core, cur->vcc_sram,
					next->vcc_core, next->vcc_sram);
		if (ret)
			return ret;
	}

	ret = pm_freq_change(cur, next, fv_mode);
	if (ret)
		return ret;

	if ((flag & FV_SET_CV) && (next->vcc_core < cur->vcc_core)) {
		ret = pm_vol_change(cur->vcc_core, cur->vcc_sram,
					next->vcc_core, next->vcc_sram);
		if (ret)
			return ret;
	}

	return 0;
}

#ifdef DEBUG
static void print_info(struct pxa3xx_fv_info* info)
{
	pr_debug("xl:%lu, xn:%lu, vcc_core:%u, vcc_sram:%u, smcfs:%lu, "
		"sflfs:%lu, hss:%lu, dmsfs:%lu, df_clk:%lu, empi_clk:%lu, "
		"d0cs:%lu\n",
		info->xl, info->xn, info->vcc_core, info->vcc_sram,
		info->smcfs, info->sflfs, info->hss, info->dmcfs,
		info->df_clk, info->empi_clk, info->d0cs);
}
#endif

static ssize_t freq_show(struct sys_device *sys_dev, char *buf)
{
	int ret;
	struct pxa3xx_fv_info info;
	unsigned int mode;

	ret = pm_freq_get(&info, &mode);
	if (ret) {
		return sprintf(buf, "unable to get frequency info");
	}

	PRINT_INFO(&info);

	if (!info.d0cs){
		return sprintf(buf, "current frequency is %luMhz"
				" (XL: %lu, XN: %lu, %s) with\n"
				"  SMEM: %lu (%dMhz)\n"
				"  SRAM: %lu (%dMhz)\n"
				"  HSS: %lu (%dMhz)\n"
				"  DDR: %lu (%dMhz)\n"
				"  DFCLK: %lu (%dMhz)\n"
				"  EMPICLK: %lu (%dMhz)\n"
				"  D0CKEN_A: 0x%08x\n"
				"  D0CKEN_B: 0x%08x\n"
				"  ACCR: 0x%08x\n"
				"  ACSR: 0x%08x\n"
				"  OSCC: 0x%08x\n",
				FREQ_CORE(info.xl, info.xn), info.xl, info.xn,
				(info.xn != 0x1)? "Turbo Mode" : "Run Mode",
				info.smcfs, FREQ_STMM(info.smcfs),
				info.sflfs, FREQ_SRAM(info.sflfs),
				info.hss, FREQ_HSS(info.hss),
				info.dmcfs, FREQ_DDR(info.dmcfs),
				info.df_clk, FREQ_DFCLK(info.smcfs, info.df_clk),
				info.empi_clk, FREQ_EMPICLK(info.smcfs, info.empi_clk),
				CKENA, CKENB, ACCR, ACSR, OSCC);
	} else {
		return sprintf(buf, "current frequency is 60Mhz"
				" (ring oscillator mode) with\n"
				"  SMEM:15Mhz\n"
				"  SRAM:60Mhz\n"
				"  HSS:60Mhz\n"
				"  DDR:30Mhz\n"
				"  DFCLK:%sMhz\n"
				"  EMPICLK:%sMhz\n"
				"  D0CKEN_A: 0x%08x\n"
				"  D0CKEN_B: 0x%08x\n"
				"  ACCR: 0x%08x\n"
				"  ACSR: 0x%08x\n"
				"  OSCC: 0x%08x\n",
				(info.df_clk == 1)?"15":
					(info.df_clk == 2)?"7.5":
					(info.df_clk == 3)?"3.75":"0",
				(info.empi_clk == 1)?"15":
					(info.empi_clk == 2)?"7.5":
					(info.empi_clk == 3)?"3.75":"0",
				CKENA, CKENB, ACCR, ACSR, OSCC);
	}
}

/* Compare OPs without VCC and LPJ
 */
static int cmp_op_novcc(struct pxa3xx_fv_info *cur_info,
			struct pxa3xx_fv_info *next_info)
{
	int ret = 1;
	if (cur_info->xl != next_info->xl)
		return ret;
	if (cur_info->xn != next_info->xn)
		return ret;
	if (cur_info->smcfs != next_info->smcfs)
		return ret;
	if (cur_info->sflfs != next_info->sflfs)
		return ret;
	if (cur_info->hss != next_info->hss)
		return ret;
	if (cur_info->dmcfs != next_info->dmcfs)
		return ret;
	if (cur_info->df_clk != next_info->df_clk)
		return ret;
	if (cur_info->empi_clk != next_info->empi_clk)
		return ret;
	if (cur_info->d0cs != next_info->d0cs)
		return ret;
	return 0;
}

/* Set frequency.
 *
 * Notice: User can't add new OP by this way. If a new set of frequencies is
 * inputed by this way, it will only be treated as a non-standard op, not
 * a new op.
 */
static ssize_t freq_store(struct sys_device *sys_dev, const char *buf,
			size_t len)
{
	int ret, i;
	struct pxa3xx_fv_info next_info, cur_info, info;

	memset(&next_info, 0x0, sizeof(struct pxa3xx_fv_info));
	sscanf(buf, "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
		&next_info.xl, &next_info.xn, &next_info.smcfs,
		&next_info.sflfs, &next_info.hss, &next_info.dmcfs,
		&next_info.df_clk, &next_info.empi_clk);
	if ((next_info.xl == 0) || (next_info.xn == 0)) {
		next_info.xl = 0;
		next_info.xn = 0;
		next_info.d0cs = 1;
	}

	PRINT_INFO(&next_info);
	memcpy(&cur_info, &(pxa3xx_por_op[cur_op]), sizeof(struct pxa3xx_fv_info));
	for (i = 0; i < max_op; i++) {
		ret = cmp_op_novcc(&cur_info, &next_info);
		if (ret == 0)
			break;
	}
	if (ret) {
		/* set to non-standard op */
		/* Try to set the minum voltage on the NONSTANDARD OP */
		next_info.vcc_core = 1400;
		next_info.vcc_sram = 1400;
		for (i = 0; i < max_op; i++) {
			if (((pxa3xx_por_op[i].d0cs == 1) && (next_info.d0cs == 1))
				|| ((pxa3xx_por_op[i].xl == next_info.xl) &&
				(pxa3xx_por_op[i].xn == next_info.xn))) {
				next_info.vcc_core = pxa3xx_por_op[i].vcc_core;
				next_info.vcc_sram = pxa3xx_por_op[i].vcc_sram;
				break;
			}
		}
		DVFM_LOCK(&_dvfm_lock);
		ret = __set_fv(&cur_info, &next_info,
				FV_MODE_SET_CFS_XSPCLK_DIS);
		DVFM_UNLOCK(&_dvfm_lock);
		if (ret) {
			pr_debug("frequency change failed\n");
			return len;
		}
		pm_freq_vol_get(&info, NULL);
       		memcpy(&pxa3xx_por_op[NONSTD_OP], &info,
			sizeof(struct pxa3xx_fv_info));
		pxa3xx_por_op[NONSTD_OP].lpj = LPJ_PER_MHZ
			*FREQ_CORE(pxa3xx_por_op[NONSTD_OP].xl, pxa3xx_por_op[NONSTD_OP].xn);
		loops_per_jiffy = pxa3xx_por_op[NONSTD_OP].lpj;
	        cur_op = NONSTD_OP;
	} else {
		/* set to formal op */
		DVFM_LOCK(&_dvfm_lock);
		ret = __set_fv(&cur_info, &next_info,
				FV_MODE_SET_CFS_XSPCLK_DIS);
		DVFM_UNLOCK(&_dvfm_lock);
		if (ret) {
			pr_debug("frequency change failed\n");
			return len;
		}
		cur_op = i;
	}

	return len;
}

static ssize_t vol_show(struct sys_device *sys_dev, char *buf)
{
	int mv1, mv2, mv3, len;
	pxa3xx_pmic_get_voltage(VCC_CORE, &mv1);
	pxa3xx_pmic_get_voltage(VCC_SRAM, &mv2);
	pxa3xx_pmic_get_voltage(VCC_3V_APPS, &mv3);
	len = sprintf(buf, "VCC_CORE:%dmv;\tVCC_SRAM:%dmv;\tVCC_IO:%dmv;\t\t"
			"VCC_MSL:-1mv;\n", mv1, mv2, mv3);
	pxa3xx_pmic_get_voltage(VCC_LCD, &mv1);
	pxa3xx_pmic_get_voltage(VCC_USB, &mv2);
	pxa3xx_pmic_get_voltage(VCC_SDIO, &mv3);
	len += sprintf(buf + len, "VCC_LCD:%dmv;\t\tVCC_USB:%dmv;\t\tVCC_CARD0:%dmv;\t"
			"VCC_CARD1:%dmv;\n", mv1, mv2, mv3, mv3);
	pxa3xx_pmic_get_voltage(VCC_MEM, &mv1);
	pxa3xx_pmic_get_voltage(VCC_CAMERA_IO, &mv2);
	pxa3xx_pmic_get_voltage(VCC_TSI, &mv3);
	len += sprintf(buf + len, "VCC_MEM:%dmv;\t\tVCC_DF:-1mv;\t\tVCC_CI:%dmv;\t\t"
			"VCC_TSI:%dmv;\n", mv1, mv2, mv3);
	len += sprintf(buf + len, "entry_low_op:%d\n", entry_low_op);
	return len;
}

static ssize_t vol_store(struct sys_device *sys_dev, const char *buf,
			size_t len)
{
	int vcc_core, vcc_sram, vcc_io, vcc_msl,vcc_lcd, vcc_usb;
	int vcc_card0, vcc_card1, vcc_mem, vcc_df, vcc_ci, vcc_tsi;
	int ret;
	struct pxa3xx_fv_info next_info, cur_info, info;
	unsigned int mode;

	sscanf(buf, "%dmv,%dmv,%dmv,%dmv,%dmv,%dmv,"
		"%dmv,%dmv,%dmv,%dmv,%dmv,%dmv",
		&vcc_core, &vcc_sram, &vcc_io, &vcc_msl, &vcc_lcd, &vcc_usb,
	        &vcc_card0, &vcc_card1, &vcc_mem, &vcc_df, &vcc_ci, &vcc_tsi);

	pr_debug("want to set as:\n"
		"VCC_Core: %dmv;\t" "VCC_Sram: %dmv;\n"
		"VCC_IO:%dmv;\t" "VCC_MSL:%dmv;\n"
		"VCC_LCD:%dmv;\t" "VCC_USB:%dmv;\n"
		"VCC_CARD0:%dmv;\t" "VCC_CARD1:%dmv;\n"
		"VCC_MEM:%dmv;\t" "VCC_DF:%dmv;\n"
		"VCC_CI:%dmv;\t" "VCC_TSI:%dmv;\n",
		vcc_core, vcc_sram, vcc_io, vcc_msl, vcc_lcd, vcc_usb,
		vcc_card0, vcc_card1, vcc_mem, vcc_df, vcc_ci, vcc_tsi);

	memcpy(&next_info, &(pxa3xx_por_op[cur_op]), sizeof(struct pxa3xx_fv_info));
	memcpy(&cur_info, &(pxa3xx_por_op[cur_op]), sizeof(struct pxa3xx_fv_info));
	next_info.vcc_core = vcc_core;
	next_info.vcc_sram = vcc_sram;

	DVFM_LOCK(&_dvfm_lock);
	ret = __set_fv(&cur_info, &next_info, 0x3);
	DVFM_UNLOCK(&_dvfm_lock);

	if (ret)
	        pr_debug("voltage change failed\n");
	else
        	pr_debug("voltage change success\n");

	pxa3xx_pmic_set_voltage(VCC_3V_APPS, vcc_io);
	pxa3xx_pmic_set_voltage(VCC_LCD, vcc_lcd);
	pxa3xx_pmic_set_voltage(VCC_USB, vcc_usb);
	pxa3xx_pmic_set_voltage(VCC_SDIO, vcc_card0);
	pxa3xx_pmic_set_voltage(VCC_SDIO, vcc_card1);
	pxa3xx_pmic_set_voltage(VCC_MEM, vcc_mem);
	pxa3xx_pmic_set_voltage(VCC_CAMERA_IO, vcc_ci);
	pxa3xx_pmic_set_voltage(VCC_TSI, vcc_tsi);

	pm_freq_vol_get(&info, &mode);
	DVFM_LOCK(&_dvfm_lock);
	cur_op = find_op();

	/* This is trick, to set any possible value out of POR'ed operating points*/
	if (cur_op == max_op){
        	memcpy(&pxa3xx_por_op[NONSTD_OP], &info, sizeof(struct pxa3xx_fv_info));
		pxa3xx_por_op[NONSTD_OP].lpj = LPJ_PER_MHZ
			*FREQ_CORE(pxa3xx_por_op[NONSTD_OP].xl, pxa3xx_por_op[NONSTD_OP].xn);
		loops_per_jiffy = pxa3xx_por_op[NONSTD_OP].lpj;
 		cur_op = NONSTD_OP;
	}
	DVFM_UNLOCK(&_dvfm_lock);

	return len;
}

static ssize_t op_show(struct sys_device *sys_dev, char *buf)
{
	struct pxa3xx_fv_info *info;
	info = &pxa3xx_por_op[cur_op];

	return sprintf(buf, "op:%d, xl:%lu, xn:%lu,vcc_core:%u, vcc_sram:%u,"
			"smcfs:%lu, sflfs:%lu, hss:%lu, dmcfs:%lu, df_clk:%lu,"
			"empi_clk:%lu, d0cs:%lu, lpj:%lu\n",
			cur_op, info->xl, info->xn, info->vcc_core,
			info->vcc_sram, info->smcfs, info->sflfs, info->hss,
			info->dmcfs, info->df_clk, info->empi_clk, info->d0cs,
			info->lpj);
}

static ssize_t op_store(struct sys_device *sys_dev, const char *buf,
			size_t len)
{
	unsigned int op = max_op, mode = FV_MODE_SET_CFS_XSPCLK_MASK + 1;

	sscanf(buf, "%u,%u", &op, &mode);
	if (op < max_op) {
		if (mode > FV_MODE_SET_CFS_XSPCLK_MASK)
			mode = FV_MODE_SET_CFS_XSPCLK_DIS;
		pxa3xx_fv_set_op(op, mode);
	}

	return len;
}

static ssize_t switch_time_show(struct sys_device *sys_dev, char *buf)
{
	struct dvfm_measure_t s_dvfm;
	int len = 0, i;

	/* (x << 2) / 13 equals to (x * 100) / 325 */
	for (i = 0; i < DVFM_REC_NUM; i++) {
		s_dvfm.last_time[i] = (m_dvfm.last_time[i] << 2) / 13;
	}
	s_dvfm.avg_time = (m_dvfm.avg_time << 2) / 13;
	s_dvfm.freq_times = m_dvfm.freq_times;
	len = sprintf(buf, "Changed freq times:%d Costed average time:%dus\n",
				s_dvfm.freq_times, s_dvfm.avg_time);
	len += sprintf(buf + len, "The last 16 time:\n");
	for (i = DVFM_REC_NUM - 1; i > 0; i = i - 4) {
		len += sprintf(buf + len, "%dus %dus %dus %dus\n",
			s_dvfm.last_time[i], s_dvfm.last_time[i-1],
			s_dvfm.last_time[i-2], s_dvfm.last_time[i-3]);
	}
	return len;
}

static ssize_t lowop_store(struct sys_device *sys_dev, const char *buf,size_t len)
{
    sscanf(buf,"%d",&low_op);
    return len;
}

static ssize_t lowop_show(struct sys_device *sys_dev, char *buf)
{
    ssize_t len=0;
    len += sprintf(buf+len,"%d\n",low_op);
    return len;
}


SYSDEV_ATTR(frequency, 0644, freq_show, freq_store);
SYSDEV_ATTR(voltage, 0644, vol_show, vol_store);
SYSDEV_ATTR(op, 0644, op_show, op_store);
SYSDEV_ATTR(switch_time, 0444, switch_time_show, NULL);
SYSDEV_ATTR(low_op, 0644, lowop_show, lowop_store);

static struct attribute *pxa3xx_fv_attr[] = {
	&attr_frequency.attr,
	&attr_voltage.attr,
	&attr_op.attr,
	&attr_switch_time.attr,
	&attr_low_op.attr,
	NULL
};

static int pxa3xx_fv_add(struct sys_device *sys_dev)
{
	struct pxa3xx_fv_info info;
	unsigned int mode;
	int i = 0;

	if (sys_dev->id != 0)
		return -EINVAL;

	pm_freq_vol_get(&info, &mode);

	pr_debug("CPU%d:freqeuncy: %dMhz, Vcc_core: %dmv, "
		"Vcc_sram: %dmv\n", sys_dev->id,
		(unsigned int)(info.xl*info.xn*13), info.vcc_core,
		info.vcc_sram);
	while (pxa3xx_fv_attr[i]) {
		if (sysfs_create_file(&(sys_dev->kobj), pxa3xx_fv_attr[i]))
			pr_debug("sysfs_create_file failed for %s\n",
					pxa3xx_fv_attr[i]->name);
		i++;
	}
	return 0;
}

static int pxa3xx_fv_rm(struct sys_device *sysdev)
{
	int i = 0;

	while (pxa3xx_fv_attr[i]) {
		sysfs_remove_file(&(sysdev->kobj), pxa3xx_fv_attr[i]);
		i++;
	}
	return 0;
}

/* resume of DVFM is better done when each device is not resumed back. So the
   restore of frequeuncy and voltage is left do in main suspend/resume process.
   The driver only provide some interface for user to get information in sys
   file system.
 */

static struct sysdev_driver pxa3xx_fv_sysdev_driver = {
	.add		= pxa3xx_fv_add,
	.remove		= pxa3xx_fv_rm,
};


static struct pxa3xx_fv_notifier * pxa3xx_fv_notifier_call(int cmd,
					struct pxa3xx_fv_notifier_info *info)
{
	struct pxa3xx_fv_notifier *list;
	struct list_head *entry;
	int ret;

	list_for_each(entry, &pxa3xx_fv_notifier_list) {
		list = list_entry(entry, struct pxa3xx_fv_notifier,
				notifier_list);
		ret = list->notifier_call(cmd, list->client_data, info);
		list->ret_code = ret;
		if (ret && cmd == FV_NOTIFIER_QUERY_SET)
			return list;
	}
	return 0;
}

static int find_op(void)
{
	struct pxa3xx_fv_info info;
	unsigned int mode;
	int i = 0, ret;

	if (pm_freq_vol_get(&info, &mode)) {
		/* return -EIO; */
		printk(KERN_ERR "error: get frequency/voltage info!\n");
	}

	PRINT_INFO(&info);
	while ((pxa3xx_por_op[i].xl != 0x0) || (pxa3xx_por_op[i].d0cs != 0x0)) {
		ret = memcmp(&info, &(pxa3xx_por_op[i]),
			sizeof(struct pxa3xx_fv_info)  - sizeof(unsigned long));
		if (!ret)
			break;
		i++;
	}
	pr_debug("find_op:%d\n", i);
	return i;
}

static void measure_time(struct fv_time_t *meas, int i)
{
#ifdef CONFIG_DVFM_MEASUREMENT
	unsigned int tmp;
	if ((meas == NULL) || (i < 0)) {
		pr_debug("invalid paramter\n");
		return;
	}
	tmp = OSCR;
	if (i == 0) {
		(meas + i)->start = tmp;
	} else {
		(meas + i)->start = tmp;
		if ((meas + i - 1)->start == (meas + i - 1)->end) {
			(meas + i - 1)->end = tmp + 1;
		} else
			(meas + i - 1)->end = tmp;
	}
#endif
}
static void dump_measure_time(struct fv_time_t *meas, int len)
{
#ifdef CONFIG_DVFM_MEASUREMENT
	int i, out, sum = 0;
	pr_debug("#");
	for (i = 0; i < len; i++) {
		out = ((meas + i)->end > (meas + i)->start) ?
			(((meas + i)->end - (meas + i)->start) << 2) / 13 :
			((0xffffffff - (meas + i)->start + (meas + i)->end) *
			2) / 13;
		pr_debug("%dus ", out);
		sum += out;
	}
	pr_debug("T%dus\n", sum);
#endif
}

static int __set_fv(struct pxa3xx_fv_info* cur, struct pxa3xx_fv_info* next,
			unsigned long mode)
{
	int ret = 0, flag, i;
	struct pxa3xx_fv_notifier_info info;
	struct pxa3xx_fv_notifier *n;
	unsigned long flags;
	struct pxa3xx_fv_info temp;
	struct pxa3xx_fv_info* cur_info = &temp;
	unsigned int fv_mode;
	struct fv_time_t fv_time[DVFM_SEC_NUM];
#ifdef CONFIG_FB_PXA
	unsigned int next_pcd, d0cs_pcd, m104_pcd;
#endif
	const int wkr_104mhz = 1;

	PRINT_INFO(cur);
	PRINT_INFO(next);

	if ((next->d0cs == 0) && ((next->xl == 0) || next->xn == 0))
		return -EINVAL;

	memcpy(&(info.cur), cur, sizeof(struct pxa3xx_fv_info));
	memcpy(&(info.next), next, sizeof(struct pxa3xx_fv_info));
	memcpy(cur_info, cur, sizeof(struct pxa3xx_fv_info));

	info.mode = mode;

	local_irq_save(flags);

	for (i = 0; i < DVFM_SEC_NUM; i++) {
		fv_time[i].start = 0;
		fv_time[i].end = 1;
	}
	measure_time(fv_time, 0);

	n = pxa3xx_fv_notifier_call(FV_NOTIFIER_QUERY_SET, &info);

	measure_time(fv_time, 1);
	if (n) {
		pr_debug("The device(module): %s can not support freqeuncy "
			"change", n->name);
		local_irq_restore(flags);
		return n->ret_code;
	}

	pxa3xx_fv_notifier_call(FV_NOTIFIER_PRE_SET, &info);

	flag = pm_freq_vol_preset(cur_info, next, &fv_mode);

	measure_time(fv_time, 2);
#ifdef CONFIG_FB_PXA
	d0cs_pcd = pxafb_get_d0cs_pcd();
	m104_pcd = pxafb_get_104m_pcd();
	next_pcd = pxafb_get_op_pcd(next->hss);
#endif

	if (flag & FV_SET_D0CS) { /* enter/exit d0cs operating point */
		// XXX BT I'm generally concerned about how the LCD
		// controller clock and PCD setting is handled below
		// (for both the enter D0CS and exit D0CS cases.  Probably
		// this stuff really ought to be synchronized with the
		// frame so that we only change the pixel clock duration
		// between frames, not in the middle of a frame.  I'm
		// worried about glitching.  That said...  if I can get
		// away with it I'm going to leave this stuff as-is because
		// frame synchronization would be complex and slow and
		// seeing how this code is going to be called for entering
		// and exiting idle...  ugh...
		if (next->d0cs) { /*enter D0CS */
			/* Park the bus arbiter on the LCD controller. */
			ARB_CNTRL1 = (1<<25) | 0x0f00;

			measure_time(fv_time, 3);
			if (wkr_104mhz) {
				/* As a workaround, we always change 
				 * frequency to 104M before enter D0CS.
				 */
				flag = pm_freq_vol_preset(cur_info, 
					&pxa3xx_por_op[1], &fv_mode);
				measure_time(fv_time, 4);
				if (flag & FV_SET_HSS) {
#ifdef CONFIG_FB_PXA
					pxa_set_cken(CKEN_LCD, 0);
					pxafb_set_pcd(m104_pcd);
#endif
					pm_hss_change(pxa3xx_por_op[1].hss);
#ifdef CONFIG_FB_PXA
					pxa_set_cken(CKEN_LCD, 1);
#endif
					measure_time(fv_time, 4);
				}
				// Change the voltage here, while in 104MHz
				// mode, because this seems to freak out the
				// LCD controller less than changing it in
				// D0CS.  The voltages for 104MHz and D0CS
				// are the same.  - mycroft, 20080807
				pm_freq_vol_change_nohss(cur_info,
					&pxa3xx_por_op[1], mode);
			}

			/* FIXME: We try to change PCD value
			 * as fast as possible. So we change
			 * pcd value before enter d0cs.
			 */
#ifdef CONFIG_FB_PXA
			pxa_set_cken(CKEN_LCD, 0);
			pxafb_set_pcd(d0cs_pcd);
#endif
			pm_enter_d0cs(cur_info);
#ifdef CONFIG_FB_PXA
			pxa_set_cken(CKEN_LCD, 1);
#endif

			measure_time(fv_time, 4);
			measure_time(fv_time, 5);
			measure_time(fv_time, 6);
			measure_time(fv_time, 7);
		} else {			/* exit D0CS */

			measure_time(fv_time, 3);
			/* FIXME: We try to change PCD value
			 * as fast as possible. So we change
			 * pcd value after exit d0cs.
			 */
#ifdef CONFIG_FB_PXA
			pxa_set_cken(CKEN_LCD, 0);
#endif
			pm_exit_d0cs(next);
			/* We will be in 104M mode because we are in 104M mode
			 * before enter D0CS.
			 */
#ifdef CONFIG_FB_PXA
			pxafb_set_pcd(m104_pcd);
			pxa_set_cken(CKEN_LCD, 1);
#endif
			measure_time(fv_time, 4);

			if (wkr_104mhz) {
				// Change the voltage here, while in 104MHz
				// mode, because this seems to freak out the
				// LCD controller less than changing it in
				// D0CS.  The voltages for 104MHz and D0CS
				// are the same.  - mycroft, 20080807
				pm_freq_vol_change_nohss(&pxa3xx_por_op[1],
					next, mode);

				measure_time(fv_time, 5);
				measure_time(fv_time, 6);

				/* LCD clock is related to HSS */
				flag = pm_freq_vol_preset(&pxa3xx_por_op[1],
					next, &fv_mode);
				if (flag & FV_SET_HSS) {
					measure_time(fv_time, 6);
#ifdef CONFIG_FB_PXA
					pxa_set_cken(CKEN_LCD, 0);
#endif
					pm_hss_change(next->hss);
#ifdef CONFIG_FB_PXA
					pxafb_set_pcd(next_pcd);
					pxa_set_cken(CKEN_LCD, 1);
#endif
				}
			}
			measure_time(fv_time, 7);

			/* Park the bus arbiter on the CPU. */
			ARB_CNTRL1 = (1<<23);
		}
	} else {	/* Not enter/exit D0CS */
		/* We try to change the HSS/PCD as fast as possible, so
		 * HSS/PCD update need be done in higher frequency op.
		 */

		if (next->hss > cur_info->hss) {
			/* Next op is higher frequency, change freq first */
			pm_freq_vol_change_nohss(cur_info, next, mode);
		}

		measure_time(fv_time, 3);
		/* LCD clock is related to HSS */
		flag = pm_freq_vol_preset(cur_info, next, &fv_mode);
		measure_time(fv_time, 4);
		if (flag & FV_SET_HSS) {
#ifdef CONFIG_FB_PXA
			pxa_set_cken(CKEN_LCD, 0);
			pxafb_set_pcd(next_pcd);
#endif
			pm_hss_change(next->hss);
#ifdef CONFIG_FB_PXA
			pxa_set_cken(CKEN_LCD, 1);
#endif
 		}
		measure_time(fv_time, 5);

		if (cur_info->hss >= next->hss) {
			/* current op is higher frequency, change freq last */
			pm_freq_vol_change_nohss(cur_info, next, mode);
		}
		measure_time(fv_time, 6);

		measure_time(fv_time, 7);
	}

	pxa3xx_fv_notifier_call(FV_NOTIFIER_POST_SET, &info);

	measure_time(fv_time, 8);
	local_irq_restore(flags);
	/* dump out */
	dump_measure_time(fv_time, 8);

#if 0
	if (next->d0cs == 1) {
		pm_freq_get((struct pxa3xx_fv_info *)next, NULL);
	}
#endif

	return ret;
}

static int __pxa3xx_fv_set_op(unsigned int op, unsigned int mode)
{
	int ret;
	int save_op;

	pr_debug("set to op:%d, mode:%d\n", op, mode);
	pr_debug("op:%d,max_op:%d,cur_op:%d\n",op,max_op,cur_op);
	/* validate @op is in a valid range */
	/*
	if (op > max_op)
		pr_debug("warning: op(%d)>max_op(%d)!\n", op, max_op);
	*/

	if (op == cur_op)
		return 0;

	save_op = cur_op;
	ret = __set_fv(&(pxa3xx_por_op[cur_op]), &(pxa3xx_por_op[op]), mode);

	if (!ret) {
		if (!pxa3xx_por_op[op].lpj){
			pxa3xx_por_op[op].lpj = LPJ_PER_MHZ
				*FREQ_CORE(pxa3xx_por_op[op].xl, pxa3xx_por_op[op].xn);
		}
		loops_per_jiffy = pxa3xx_por_op[op].lpj;
		cur_op = op;
	} else {
		pr_debug("set invalid op: %d\n", op);
		return ret;
	}

	pr_debug("current op is :%d\n", cur_op);

	return ret;
}

#ifdef CONFIG_DPM
int pxa3xx_fv_add_op(struct pxa3xx_fv_info *info)
{
	int ret = 0;
	int save_op;

	/* It won't touch the reserved OPs */
	if (max_op == END_OP) {
		return -EINVAL;
	}
	DVFM_LOCK(&_dvfm_lock);
	pxa3xx_por_op[max_op].xl = info->xl;
	pxa3xx_por_op[max_op].xn = info->xn;
	pxa3xx_por_op[max_op].vcc_core = info->vcc_core;
	pxa3xx_por_op[max_op].vcc_sram = info->vcc_sram;
	pxa3xx_por_op[max_op].smcfs = info->smcfs;
	pxa3xx_por_op[max_op].sflfs = info->sflfs;
	pxa3xx_por_op[max_op].hss = info->hss;
	pxa3xx_por_op[max_op].dmcfs = info->dmcfs;
	pxa3xx_por_op[max_op].df_clk = info->df_clk;
	pxa3xx_por_op[max_op].empi_clk = info->empi_clk;
	pxa3xx_por_op[max_op].d0cs = info->d0cs;

	/* try the new op to caculate the loops_per_jiffy */
	save_op = cur_op;
	__pxa3xx_fv_set_op(max_op, 0x3);

	/* restore the original op */
	__pxa3xx_fv_set_op(save_op, 0x3);

	ret = max_op++;
	DVFM_UNLOCK(&_dvfm_lock);
	return ret;
}
EXPORT_SYMBOL(pxa3xx_fv_add_op);

void pxa3xx_fv_save(void)
{
	dvfm_context.max_op = max_op;

	memcpy(&(dvfm_context.pxa3xx_op), &(pxa3xx_por_op),
		sizeof(struct pxa3xx_fv_info) * DVFM_MAX_OP);
}

void pxa3xx_fv_restore(unsigned int saved_op)
{
	DVFM_LOCK(&_dvfm_lock);
	max_op = dvfm_context.max_op;
	memset(&(pxa3xx_por_op), 0x0, sizeof(struct pxa3xx_fv_info) * DVFM_MAX_OP);
	memcpy(&(pxa3xx_por_op), &(dvfm_context.pxa3xx_op),
		sizeof(struct pxa3xx_fv_info) * DVFM_MAX_OP);
	if ((saved_op == BOOT_OP) && (pxa3xx_por_op[BOOT_OP].xl == 0)) {
		/* Check whether BOOT_OP is set with valid parameter */
		cur_op = def_op;
	} else
		cur_op = saved_op;
	DVFM_UNLOCK(&_dvfm_lock);
	pxa3xx_fv_set_op(cur_op, 3);
}
EXPORT_SYMBOL(pxa3xx_fv_restore);
#endif

int pxa3xx_fv_set_op(unsigned int op, unsigned int mode)
{
	int ret;
#ifdef CONFIG_DVFM_MEASUREMENT
	int i;
	unsigned int fv_start_time;
	unsigned int fv_end_time;
	unsigned int fv_switch_time;
	static unsigned int sum = 0, old_sum = 0;
#endif

	DVFM_LOCK(&_dvfm_lock);
#ifdef CONFIG_DVFM_MEASUREMENT
	fv_start_time = OSCR;
#endif
	ret = __pxa3xx_fv_set_op(op, mode);
#ifdef CONFIG_DVFM_MEASUREMENT
	if (ret) {
		/* when fails to change op, time needn't be measured */
		DVFM_UNLOCK(&_dvfm_lock);
		return ret;
	}
	fv_end_time = OSCR;

	fv_switch_time = (fv_end_time > fv_start_time) ?
			(fv_end_time - fv_start_time) :
			(0xffffffff - fv_start_time + fv_end_time);
	for (i = 15; i > 0; i--) {
		m_dvfm.last_time[i] = m_dvfm.last_time[i - 1];
	}
	m_dvfm.last_time[0] = fv_switch_time;
	sum += fv_switch_time;
	if (sum < old_sum) {
		/* overflow on sum */
		sum = 0;
		m_dvfm.freq_times = 16;
		for (i = 0; i < m_dvfm.freq_times; i++)
			sum += m_dvfm.last_time[i];
		m_dvfm.avg_time = sum >> 4;

		old_sum = 0;
		DVFM_UNLOCK(&_dvfm_lock);
		return 0;
	}
	if (likely(++(m_dvfm.freq_times))) {
		m_dvfm.avg_time = sum / m_dvfm.freq_times;
	} else {
		/* overflow on freq_times */
		sum = 0;
		m_dvfm.freq_times = 16;
		for (i = 0; i < m_dvfm.freq_times; i++)
			sum += m_dvfm.last_time[i];
		m_dvfm.avg_time = sum >> 4;
	}
	old_sum = sum;
#endif
	DVFM_UNLOCK(&_dvfm_lock);

	return ret;
}
EXPORT_SYMBOL(pxa3xx_fv_set_op);

int is_low_op(void)
{
	if ((cur_op == d0cs_op) || (cur_op == low_op))
		return 1;
	return 0;
}
EXPORT_SYMBOL(is_low_op);

int pxa3xx_fv_enter_low_op(void)
{
	int ret = 1;

	entry_low_op = cur_op;
#if 0 /*XXX BT*/
#if !defined(CONFIG_FB_PXA_LCD_VGA) && defined(CONFIG_PXA3xx_D0CS)
	if (cur_op != d0cs_op) {
		/* D0CS can't support VGA pannel well because of
		 * hardware limitation
		 */
		ret = pxa3xx_fv_set_op(d0cs_op, 3);
	} else
#endif
#endif
	{
		/* use 208MHz instead of D0CS when D0CS is unavailable */
		if ((cur_op != low_op) && (cur_op != d0cs_op)
				&& (FREQ_CORE(pxa3xx_por_op[cur_op].xl,
				pxa3xx_por_op[cur_op].xn)
				> FREQ_CORE(pxa3xx_por_op[low_op].xl,
				pxa3xx_por_op[low_op].xn))) {
			ret = pxa3xx_fv_set_op(low_op, 3);
		}
	}
	/* can't enter low OP */
	if (ret)
		entry_low_op = -1;
	return ret;
}
EXPORT_SYMBOL(pxa3xx_fv_enter_low_op);

int pxa3xx_fv_exit_low_op(void)
{
	int ret = 0;
	if (entry_low_op != -1) {
		ret = pxa3xx_fv_set_op(entry_low_op, 3);
		if (ret == 0) {
			entry_low_op = -1;
			return 0;
		}
	}
	pr_debug("meet invalid op on exit\n");
	return ret;
}
EXPORT_SYMBOL(pxa3xx_fv_exit_low_op);

int pxa3xx_fv_get_op_count(void)
{
	return max_op;
}
EXPORT_SYMBOL(pxa3xx_fv_get_op_count);

int pxa3xx_fv_get_op_info(unsigned int op, struct pxa3xx_fv_info *info)
{
#ifdef CONFIG_DPM
	if (!info || (op > max_op && op <= END_OP))
#else
	if (!info || (op > ARRAY_SIZE(pxa3xx_por_op)))
#endif
		return -EINVAL;
	memcpy(info, &(pxa3xx_por_op[op]), sizeof(struct pxa3xx_fv_info));
	return 0;
}
EXPORT_SYMBOL(pxa3xx_fv_get_op_info);

int pxa3xx_fv_get_op(void)
{
	return cur_op;
}
EXPORT_SYMBOL(pxa3xx_fv_get_op);

int pxa3xx_fv_get_def_op(void)
{
	return def_op;
}
EXPORT_SYMBOL(pxa3xx_fv_get_def_op);

int pxa3xx_fv_register_notifier(struct pxa3xx_fv_notifier *n)
{
	struct pxa3xx_fv_notifier *list;
	struct list_head *entry;

	if (!n)
		return -EINVAL;

	list_for_each(entry, &pxa3xx_fv_notifier_list) {
		list = list_entry(entry, struct pxa3xx_fv_notifier,
				notifier_list);
		if(n->priority > list->priority)
			break;
	}
	list_add(&(n->notifier_list), entry);

	return 0;
}

int pxa3xx_fv_unregister_notifier(struct pxa3xx_fv_notifier *n)
{
	struct pxa3xx_fv_notifier *list;
	struct list_head *entry;

	if (!n)
		return -EINVAL;

	list_for_each(entry, &pxa3xx_fv_notifier_list) {
		list = list_entry(entry, struct pxa3xx_fv_notifier,
				notifier_list);
		if (list == n) {
			list_del(entry);
			return 0;
		}
	}
	return -ENOENT;
}

EXPORT_SYMBOL(pxa3xx_fv_register_notifier);
EXPORT_SYMBOL(pxa3xx_fv_unregister_notifier);

#ifdef	CONFIG_PM
/*
 * Suspend the pxa3xx_fv interface.
 */
static int pxa3xx_fv_suspend(struct platform_device *_dev, pm_message_t state)
{
	struct pxa3xx_fv_info curr_info;
	int ret = 0;

	pr_debug("pxa3xx_fv: pxa3xx_fv suspend\n");

	pm_save_op = cur_op;
	/* sleep_op shouldn't be D0CS op */
	if (sleep_op == d0cs_op) {
		printk(KERN_WARNING "can't suspend at this time\n");
		return -EFAULT;
	}
	if (pxa3xx_por_op[cur_op].d0cs == 1) {
		if (pm_vol_change(pxa3xx_por_op[cur_op].vcc_core,
				pxa3xx_por_op[cur_op].vcc_sram,
				pxa3xx_por_op[sleep_op].vcc_core,
				pxa3xx_por_op[sleep_op].vcc_sram)) {
			return -EFAULT;
		}
		memcpy(&curr_info, &pxa3xx_por_op[sleep_op],
			sizeof(struct pxa3xx_fv_info));
		pm_exit_d0cs(&curr_info);
	} else {
		if (cur_op != sleep_op)
			ret = pm_freq_vol_change_nohss(&pxa3xx_por_op[cur_op],
					&pxa3xx_por_op[sleep_op], 3);
	}
	if (ret)
		pr_debug("failed to change op before suspend\n");

	return 0;
}

/*
 * Resume the pxa3xx_fv interface.
 */
static int pxa3xx_fv_resume(struct platform_device *_dev)
{
	struct pxa3xx_fv_info curr_info;

	pr_debug("pxa3xx_fv: pxa3xx_fv resume\n");
	if (pxa3xx_por_op[pm_save_op].d0cs == 1) {
		memcpy(&curr_info, &pxa3xx_por_op[pm_save_op],
			sizeof(struct pxa3xx_fv_info));
		pm_enter_d0cs(&curr_info);
		pm_vol_change(curr_info.vcc_core, curr_info.vcc_sram,
				pxa3xx_por_op[pm_save_op].vcc_core,
				pxa3xx_por_op[pm_save_op].vcc_sram);
	} else {
		pm_freq_vol_get(&curr_info, NULL);
		pm_freq_vol_change_nohss(&curr_info, &pxa3xx_por_op[pm_save_op], 3);
		pm_hss_change(pxa3xx_por_op[pm_save_op].hss);
	}
	cur_op = pm_save_op;

	return 0;
}

#else				/*  */
#define	pxa3xx_fv_suspend		NULL
#define	pxa3xx_fv_resume		NULL
#endif				/*  */


static int set_ddr60(int flag)
{
	unsigned int reg, cpuid;

	cpuid = read_cpuid(0) & 0xFFFF;
	/* It's Monahans LV A2 */
	if (cpuid != 0x6892)
		return 0;
	reg = ACCR;
	if (flag) {
		reg |= 0x80;
	} else {
		reg &= ~0x80;
	}
		ACCR = reg;
		/* polling ACCR */
	for (;;) {
		if ((reg & 0x80) == (ACCR & 0x80))
		break;
	}
	low_op = 0;
	return 0;
}


/* OP tables are stored in DVFM.
 * Ring Oscillator Mode is also considered as an OP.
 */
static int pxa3xx_fv_probe(struct platform_device *_dev)
{
	int i = 0;
	unsigned int save_op;

	pr_debug("Frequency/Voltage Init\n");

	/* scan DVFM POR table */
	while ((pxa3xx_por_op[i].xl != 0x0) || (pxa3xx_por_op[i].d0cs != 0x0))
		i++;
#ifdef CONFIG_DPM
	if (i >= END_OP) {
		printk(KERN_ERR "The nubmer of operating point has exceeded \
				the max number!\n");
		return -EINVAL;
	}
#endif

	max_op = i;
	pr_debug("max_op:%d\n", max_op);
	cur_op = find_op();
	if (cur_op == max_op) {
		pm_freq_vol_get((struct pxa3xx_fv_info *)&pxa3xx_por_op[BOOT_OP],
				NULL);
		pxa3xx_por_op[BOOT_OP].lpj = LPJ_PER_MHZ
			*FREQ_CORE(pxa3xx_por_op[BOOT_OP].xl, pxa3xx_por_op[BOOT_OP].xn);
		loops_per_jiffy = pxa3xx_por_op[BOOT_OP].lpj;
		cur_op = BOOT_OP;
	}

	pr_info("Current operating point is %d\n", cur_op);
	sysdev_driver_register(&cpu_sysdev_class, &pxa3xx_fv_sysdev_driver);

	/* walk through the OP tables to calculate the proper
	 * loops_per_jiffy for each OP
	 */
	save_op = cur_op;

	for (i = 1; i < max_op; i++) {
		__pxa3xx_fv_set_op(i, 3);
	}
#ifdef CONFIG_PXA3xx_D0CS
	__pxa3xx_fv_set_op(0, 3);
#endif
	__pxa3xx_fv_set_op(save_op, 3);
#ifdef CONFIG_ZYLONITE_POWER_OPT
	/* op 3 is 416MHz, its VCC_CORE is 1.1v */
	sleep_op = 3;
#endif
	set_ddr60(1);

	return 0;
}

static int pxa3xx_fv_remove(struct platform_device *_dev)
{
	sysdev_driver_unregister(&cpu_sysdev_class, &pxa3xx_fv_sysdev_driver);
	return 0;
}

static struct platform_driver pxa3xx_fv_driver = {
	.driver = {
		.name	= "pxa3xx_fv",
	},
	.probe		= pxa3xx_fv_probe,
	.remove		= pxa3xx_fv_remove,
	.suspend	= pxa3xx_fv_suspend,
	.resume		= pxa3xx_fv_resume,
};

static int __init pxa3xx_fv_init(void)
{
	return platform_driver_register(&pxa3xx_fv_driver);
}

static void __exit pxa3xx_fv_exit(void)
{
	platform_driver_unregister(&pxa3xx_fv_driver);
}

#ifndef MODULE
late_initcall(pxa3xx_fv_init);
#else
module_init(pxa3xx_fv_init);
#endif
module_exit(pxa3xx_fv_exit);

MODULE_DESCRIPTION("Basic DVFM support for Monahans");
MODULE_LICENSE("GPL");

