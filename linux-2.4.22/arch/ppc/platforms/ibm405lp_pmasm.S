/*
 * arch/ppc/platforms/ibm405lp_pmasm.S 405lp-specific assembler routines
 *                                     to support power management
 *
 * Copyright (C) 2002, Bishop Brock & Hollis Blanchard, IBM Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 *
 * Bishop Brock
 * IBM Research, Austin Center for Low-Power Computing
 * bcbrock@us.ibm.com
 * March, 2002	 
 */

#include <linux/config.h>
#include <kernel/ppc_defs.h>
#include <asm/page.h>			/* KERNELBASE */
#include <asm/cache.h>			/* LG_L1_CACHE_LINE_SIZE */
#include <asm/processor.h>
#include <asm/ppc_asm.h>
#include <platforms/ibm405lp.h>
#include <platforms/ibm405lp_pm.h>

#define GPR2_SAVE		0
#define GPR13_SAVE		4
#define GPR14_SAVE		8
#define GPR15_SAVE		12
#define GPR16_SAVE		16
#define GPR17_SAVE		20
#define GPR18_SAVE		24
#define GPR19_SAVE		28
#define GPR20_SAVE		32
#define GPR21_SAVE		36
#define GPR22_SAVE		40
#define GPR23_SAVE		44
#define GPR24_SAVE		48
#define GPR25_SAVE		52
#define GPR26_SAVE		56
#define GPR27_SAVE		60
#define GPR28_SAVE		64
#define GPR29_SAVE		68
#define GPR30_SAVE		72
#define GPR31_SAVE		76
#define CR_SAVE			80
#define LR_SAVE			84
#define SP_SAVE			88
#define CCR0_SAVE		92
#define TCR_SAVE		96
#define PIT_SAVE		100
#define MSR_SAVE		104
#define CPC0_CR0_SAVE		108
#define CPC0_CR1_SAVE		112
#define ICCR_SAVE		116
#define DCCR_SAVE		120
#define EVPR_SAVE		124
#define PID_SAVE                128
#define ZPR_SAVE                132
#define TLB_SAVE		136	/* 64 entries*3 words = 768 bytes */
#define SLEEP_STACK_BYTES	904

	.macro	mtdcr_interlock, dcr, value, temp

	mfdcr	\temp, \dcr
	clrrwi	\temp, \temp, 1	/* write current without valid bit */
	mtdcr	\dcr, \temp

	clrrwi	\temp, \value, 1
	mtdcr	\dcr, \temp	/* write new without valid bit */

	ori	\temp, \value, 1
	mtdcr	\dcr, \temp	/* write new with valid bit */

	.endm

/* The routines in this section execute code prefetched into the I-Cache.  It
 * is imperative that no TLB misses occur during the prefetching, since ICBT is
 * a no-op if it would cause a TLB miss.  Therefore all entry points are forced
 * to occupy the same physical page.  By the time the critical entry point is
 * reached (ibm405lp_pm_command()) all I/D misses will have happened.
 */

 	.text	
	.align	PAGE_SHIFT
	
/* ibm405lp_asm_suspend()
 * r3 = command, passed to apm
 * r4 = delay, passed to apm
 * r5 = pointer to struct ibm405lp_wakeup_info (must save physical
 *	stack pointer)
 *
 * code specific to powerdown (rather than clock-freeze); calls 'apm'
 *	(below).
 *
 * LR, and r3/r4 are saved to r29-r31 because we call flush_dcache_all. LR
 * must be preserved because if a wakeup condition (interrupt) already exists
 * at the time of the sleep command, 'apm' could simply fall through and
 * return here, at which point 'powerdown' would need to return to its
 * caller.
 */

_GLOBAL(ibm405lp_asm_suspend)
	subi	r1, r1, SLEEP_STACK_BYTES	/* new stack frame */

	/* wakeup_info->wakeup_sp_phys = phys(r1) */
	tophys(r9,r1)
	stw	r9, WAKEUP_SP_OFFSET(r5)

	/* wakeup_info->wakeup_func_phys = phys(wakeup) */
	lis	r9, wakeup@h
	ori	r9, r9, wakeup@l
	tophys(r9,r9)
	stw	r9, WAKEUP_FUNC_OFFSET(r5)

	/* save non-volatile GPRs + LR onto stack (calling C function will
	 * reload volatiles including CTR and XER, so we can ignore them).
	 * r1 (stack pointer) is handled elsewhere.
	 */
	stw	r2, GPR2_SAVE(r1)
	stw	r13, GPR13_SAVE(r1)
	stw	r14, GPR14_SAVE(r1)
	stw	r15, GPR15_SAVE(r1)
	stw	r16, GPR16_SAVE(r1)
	stw	r17, GPR17_SAVE(r1)
	stw	r18, GPR18_SAVE(r1)
	stw	r19, GPR19_SAVE(r1)
	stw	r20, GPR20_SAVE(r1)
	stw	r21, GPR21_SAVE(r1)
	stw	r22, GPR22_SAVE(r1)
	stw	r23, GPR23_SAVE(r1)
	stw	r24, GPR24_SAVE(r1)
	stw	r25, GPR25_SAVE(r1)
	stw	r26, GPR26_SAVE(r1)
	stw	r27, GPR27_SAVE(r1)
	stw	r28, GPR28_SAVE(r1)
	stw	r29, GPR29_SAVE(r1)
	stw	r30, GPR30_SAVE(r1)
	stw	r31, GPR31_SAVE(r1)

	/* rescue from flush_dcache_all call below */
	mflr	r29
	mr	r31, r3
	mr	r30, r4

	/* save entire TLB to stack
	 *   TLB related CPU state is saved/restored
	 *   r9 = TLB index (+= 1 per iteration)
	 *   r10 = save address (+= 3 per iteration)
	 */
	mfspr	r9, SPRN_PID
	stw	r9, PID_SAVE(r1)
	mfspr	r9, SPRN_ZPR
	stw	r9, ZPR_SAVE(r1)

	li	r9, 64
	addi	r10, r1, TLB_SAVE-4
	mtctr	r9
save_tlbe:
	subi	r9, r9, 1
	tlbrelo	r11, r9
	stwu	r11, 4(r10)
	tlbrehi	r11, r9
	stwu	r11, 4(r10)
	mfspr	r11, SPRN_PID
	stwu	r11, 4(r10)
	bdnz	save_tlbe

	lwz	r9, PID_SAVE(r1)
	mtspr	SPRN_PID, r9
	isync

	/* save CR, LR, MSR, virtual SP */
	mfcr	r9
	stw	r9, CR_SAVE(r1)
	mflr	r9
	stw	r9, LR_SAVE(r1)			/* restored into SRR0 */
	mfmsr	r9
	stw	r9, MSR_SAVE(r1)		/* restored into SRR1 */
	stw	r1, SP_SAVE(r1)

	/* save EVPR (we need it before rfi in wakeup) */
	mfspr	r9, SPRN_EVPR
	stw	r9, EVPR_SAVE(r1)

	/* save timer regs: CCR0, TCR, PIT, CPC0_CR1 */
	mfspr	r9, SPRN_CCR0
	stw	r9, CCR0_SAVE(r1)
	mfspr	r9, SPRN_TCR
	stw	r9, TCR_SAVE(r1)
	mfdcr	r9, DCRN_CPC0_CR0
	stw	r9, CPC0_CR0_SAVE(r1)
	mfdcr	r9, DCRN_CPC0_CR1
	stw	r9, CPC0_CR1_SAVE(r1)
	mfspr	r9, SPRN_PIT
	stw	r9, PIT_SAVE(r1)

	/* save storage attribute regs: ICCR, DCCR */
	mfspr	r9, SPRN_ICCR
	stw	r9, ICCR_SAVE(r1)
	mfspr	r9, SPRN_DCCR
	stw	r9, DCCR_SAVE(r1)

	/* save physical struct ptr (r5) to &wakeup_info (0xfc) */

	tophys(r10,r5)
	/* disable MSR:DR */
	mfmsr	r12
	rlwinm	r11, r12, 0, ~MSR_DR
	mtmsr	r11
	/* do the write */
	lis	r9, ibm405lp_wakeup_info@h
	ori	r9, r9, ibm405lp_wakeup_info@l
	tophys(r9,r9)
	stw	r10, 0(r9)
	/* restore MSR:DR */
	mtmsr	r12

	eieio

	/* flush dcache to RAM or we lose all that data */
	bl	flush_dcache_all	/* leaves low addresses in cache! */
	lis	r4, ibm405lp_wakeup_info@h
	ori	r4, r4, ibm405lp_wakeup_info@l
	dcbf	r0, r4			/* flush that low-mem pointer */

	sync				/* force instruction completion */

	/* continue on to ibm405lp_pm_command */
	mr	r3, r31
	mr	r4, r30
	bl	ibm405lp_pm_command

	/* if we return to here, powerdown failed. try to recover. */

	/* r29-r31 are the nonvolatiles altered above */
	mtlr	r29
	lwz	r29, GPR29_SAVE(r1)
	lwz	r30, GPR30_SAVE(r1)
	lwz	r31, GPR31_SAVE(r1)
	addi	r1, r1, SLEEP_STACK_BYTES	/* surrender stack frame */
	blr


/* ibm405lp_pm_command(command, sleep_delay)
 *
 * Final shutdown (for clock-suspend, power-standby, and cryo modes).
 * Interrupts must be disabled because we put SDRAM into self-refresh.
 */
_GLOBAL(ibm405lp_pm_command)
	/* disable instruction prefetching to prevent PLB transactions while
	 * we're trying to sleep */
	mfspr	r7, SPRN_CCR0
	rlwinm	r9, r7, 0, 22, 19	/* mask off PFC, PFNC */
	mtspr	SPRN_CCR0, r9

	/* Touch the rest of this function into the I-cache */
	lis	r9, load_icache@h
	ori	r9, r9, load_icache@l
	li	r10, ((load_icache_end-load_icache-1)>>LG_L1_CACHE_LINE_SIZE)+1
	mtctr	r10
3:
	icbt	r0, r9
	addi	r9, r9, 32		/* next cacheline */
	bdnz	3b
	sync				/* force instruction completion */

	b	load_icache

	.align	LG_L1_CACHE_LINE_SIZE
load_icache:
	/* Start SDRAM self-refresh.  

	   We also set the EMDULR bit to force a valid value on the data bus,
	   which reduces standby leakage.  Unless this bit is set here we
	   observe HUGE standby leakage currents.  Whether or not EMDULR is set
	   during normal operation is a matter of PM policy. */
	  
	li	r9, DCRN_SDRAM0_CFG
	mtdcr	DCRN_SDRAM0_CFGADDR, r9
	mfdcr	r8, DCRN_SDRAM0_CFGDATA  /* R8 = Original SDRAM0_CFG */
	oris	r9, r8, 0x4020    
	mtdcr	DCRN_SDRAM0_CFGDATA, r9  /* Start self-refresh + EMDULR */

	/* Poll until self-refresh status */
	li	r9, DCRN_SDRAM0_STATUS
	mtdcr	DCRN_SDRAM0_CFGADDR, r9
1:
	mfdcr	r9, DCRN_SDRAM0_CFGDATA
	andis.	r9, r9, 0x4000
	beq	1b
	/* now in SDRAM self-refresh mode */

#ifdef CONFIG_ARCTIC2
	/* If this is a suspend-to-RAM mode, clamp down the CKE lines by
           bringing APM0_CFG[PSC] low at power-down. */
	mfdcr	r9, DCRN_RTC0_CEN
	andi.	r9, r9, IBM405LP_POWERDOWN_SUSPEND
	beq	2f
	rlwinm	r3, r3, 0, 25, 23  
2:	
#endif

	/* final sleep command */
	mtctr	r4	
	mtdcr_interlock	DCRN_APM0_CFG, r3, r10
	bdnz+	.

	/* back from sleep; Clear the cryo command, disable the APM watchdog
           and restore CDIV == 0 */ 
	rlwinm	r3, r3, 0, 18, 16               /* clear ISP */
	rlwinm	r3, r3, 0, 19, 17		/* clear EWT */
	rlwinm	r3, r3, 0, 31, 24		/* clear CDIV */
#ifdef CONFIG_ARCTIC2
	/* Unclamp the CKE lines by bringing APM0_CFG[PSC] high. */
	ori	r3, r3, 0x0080
#endif
	mtdcr_interlock DCRN_APM0_CFG, r3, r10
	mtctr   r4
        bdnz    .

	/* re-enable the SDRAM */
	li	r9, DCRN_SDRAM0_CFG		
	mtdcr	DCRN_SDRAM0_CFGADDR, r9
	mtdcr	DCRN_SDRAM0_CFGDATA, r8		/* remove self-refresh */

	/* Poll until status clear */
	li	r9, DCRN_SDRAM0_STATUS
	mtdcr	DCRN_SDRAM0_CFGADDR, r9
3:		
	mfdcr	r9, DCRN_SDRAM0_CFGDATA
	andis.	r9, r9, 0x4000
	bne	3b
load_icache_end:
	/* restore CCR0 from r7 where we left it */
	mtspr	SPRN_CCR0, r7

	mfdcr	r3, DCRN_APM0_SR		/* return status */
	blr

/* wakeup()
 * - The registers restored here are sensitive in some way and must be
 *   restored in assembly. All others are saved/restored in the calling C
 *   code.
 * - the following MSR bits are assumed disabled: ME CE EE IR DR
 *   Really no exceptions can be handled until this code completes...
 * - the I- and D-caches are assumed to have been invalidated
 */
wakeup:
	/* clear ISP, disable the APM watchdog and restore CDIV == 0 */
	mfdcr	r9, DCRN_APM0_CFG
	rlwinm	r9, r9, 0, 18, 16               /* clear ISP */
	rlwinm	r9, r9, 0, 19, 17		/* clear EWT */
	rlwinm	r9, r9, 0, 31, 24		/* clear CDIV */
	mtdcr_interlock DCRN_APM0_CFG, r9, r10
	
	/* load stack pointer */
	lis	r9, ibm405lp_wakeup_info@h
	ori	r9, r9, ibm405lp_wakeup_info@l
	tophys(r9,r9)		/* r9 = &ibm405lp_wakeup_info */
	lwz	r10, 0(r9)	/* r10 = *ibm405lp_wakeup_info */
	lwz	r1, WAKEUP_SP_OFFSET(r10)	/* restore stack ptr */

	/* zero ibm405lp_wakeup_info */
	lis	r10, 0
	stw	r10, 0(r9)

	/* restore ICCR/DCCR */
	/* do this before we load up the I-cache to modify CCR0 (below) */
	lwz	r9, ICCR_SAVE(r1)		/* load old ICCR */
	mtspr	SPRN_ICCR, r9			/* set ICCR */
	lwz	r9, DCCR_SAVE(r1)		/* load old DCCR */
	mtspr	SPRN_DCCR, r9			/* set DCCR */
	isync					/* discard prefetches */

	/* Touch the CCR0 part of this function into the I-cache */
	lis	r9, reload_CCR0@h
	ori	r9, r9, reload_CCR0@l
	li	r10, ((reload_CCR0_end-reload_CCR0-1)>>LG_L1_CACHE_LINE_SIZE)+1
	mtctr	r10
4:
	icbt	r0, r9
	addi	r9, r9, 32			/* next cacheline */
	bdnz	4b
	isync
	b	reload_CCR0

	.align	LG_L1_CACHE_LINE_SIZE
reload_CCR0:
	/* now that we're in the icache, restore CCR0 */
	lwz	r5, CCR0_SAVE(r1)
	mtspr	SPRN_CCR0, r5
	isync
reload_CCR0_end:

	/* restore timers: CPC0_CR1, TCR, PIT */
	lwz	r5, CPC0_CR0_SAVE(r1)
	mtdcr	DCRN_CPC0_CR0, r5
	lwz	r5, CPC0_CR1_SAVE(r1)
	mtdcr	DCRN_CPC0_CR1, r5
	lwz	r5, TCR_SAVE(r1)
	mtspr	SPRN_TCR, r5
	lwz	r6, PIT_SAVE(r1)
	andis.	r5, r5, 0x0040		/* check TCR Auto Reload Enable bit */
	/* if ARE=0, restore saved PIT because it was a one-time timer */
	beq-	reload_PIT
	/* otherwise, ignore saved PIT and reload tb_ticks_per_jiffy */
	lis	r5, tb_ticks_per_jiffy@h
	ori	r5, r5, tb_ticks_per_jiffy@l
	tophys(r5,r5)
	lwz	r6, 0(r5)		/* load tb_ticks_per_jiffy */
reload_PIT:
	mtspr	SPRN_PIT, r6

	/* restore entire TLB from stack */
	/*   Afterwords, restore TLB related CPU state */
	/*   r9 = TLB index (+= 1 per iteration) */
	/*   r10 = save address (+= 3 per iteration) */
	li	r9, 64
	addi	r10, r1, TLB_SAVE-4
	mtctr	r9
restore_tlbe:
	subi	r9, r9, 1
	lwzu	r11, 4(r10)
	tlbwelo	r11, r9
	lwzu	r11, 4(r10)
	lwzu	r12, 4(r10)		/* have to load PID before tlbwe */
	mtspr	SPRN_PID, r12
	tlbwehi	r11, r9
	bdnz	restore_tlbe

	lwz	r9, PID_SAVE(r1)
	mtspr	SPRN_PID, r9
	lwz	r9, ZPR_SAVE(r1)
	mtspr	SPRN_ZPR, r9
	isync

	/* load SRR0/SRR1 */
	lwz	r9, LR_SAVE(r1)		/* load LR */
	mtsrr0	r9			/* prepare to jump back to caller */
	lwz	r9, MSR_SAVE(r1)	/* load MSR */
	mtsrr1	r9			/* prepare new MSR */

	/* restore EVPR so our exceptions go to the right place when we rfi */
	lwz	r9, EVPR_SAVE(r1)
	mtspr	SPRN_EVPR, r9

	/* restore non-volatile regs */
	lwz	r2, GPR2_SAVE(r1)
	lwz	r13, GPR13_SAVE(r1)
	lwz	r14, GPR14_SAVE(r1)
	lwz	r15, GPR15_SAVE(r1)
	lwz	r16, GPR16_SAVE(r1)
	lwz	r17, GPR17_SAVE(r1)
	lwz	r18, GPR18_SAVE(r1)
	lwz	r19, GPR19_SAVE(r1)
	lwz	r20, GPR20_SAVE(r1)
	lwz	r21, GPR21_SAVE(r1)
	lwz	r22, GPR22_SAVE(r1)
	lwz	r23, GPR23_SAVE(r1)
	lwz	r24, GPR24_SAVE(r1)
	lwz	r25, GPR25_SAVE(r1)
	lwz	r26, GPR26_SAVE(r1)
	lwz	r27, GPR27_SAVE(r1)
	lwz	r28, GPR28_SAVE(r1)
	lwz	r29, GPR29_SAVE(r1)
	lwz	r30, GPR30_SAVE(r1)
	lwz	r31, GPR31_SAVE(r1)
	lwz	r1, SP_SAVE(r1)		/* restore virtual stack ptr */
	/* no more r1 loads/stores until after rfi! */
	addi	r1, r1, SLEEP_STACK_BYTES	/* surrender stack frame */
	mr	r3, r4			/* return APM0_SR from firmware */

	rfi				/* jump to 'powerdown' caller */
