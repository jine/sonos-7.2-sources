/*
 * linux/arch/arm/mm/proc-xsc3.S
 *
 * Original Author: Matthew Gilbert
 * Current Maintainer: Lennert Buytenhek <buytenh@wantstofly.org>
 *
 * Copyright 2004 (C) Intel Corp.
 * Copyright 2005 (C) MontaVista Software, Inc.
 * Copyright 2006 (C) Marvell International Ltd.  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * MMU functions for the Intel XScale3 Core (XSC3).  The XSC3 core is
 * an extension to Intel's original XScale core that adds the following
 * features:
 *
 * - ARMv6 Supersections
 * - Low Locality Reference pages (replaces mini-cache)
 * - 36-bit addressing
 * - L2 cache
 * - Cache coherency if chipset supports it
 *
 * Based on original XScale code by Nicolas Pitre.
 *
 * 2001 Aug 21:
 *	some contributions by Brett Gaines <brett.w.gaines@intel.com>
 *	Copyright 2001 by Intel Corp.
 *
 * 2001 Sep 08:
 *	Completely revisited, many important fixes
 *	Nicolas Pitre <nico@cam.org>
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/elf.h>
#include <asm/hardware.h>
#include <asm/pgtable.h>
#include <asm/pgtable-hwdef.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include "proc-macros.S"

/*
 * This is the maximum size of an area which will be flushed.  If the
 * area is larger than this, then we flush the whole cache.
 */
#define MAX_AREA_SIZE	32768

/*
 * The cache line size of the L1 I, L1 D and unified L2 cache.
 */
#define CACHELINESIZE	32

/*
 * The size of the L1 D cache.
 */
#define CACHESIZE	32768

/*
 * Virtual address used to allocate the cache when flushed
 *
 * This must be an address range which is _never_ used.  It should
 * apparently have a mapping in the corresponding page table for
 * compatibility with future CPUs that _could_ require it.  For instance we
 * don't care.
 *
 * This must be aligned on a 2*CACHESIZE boundary.  The code selects one of
 * the 2 areas in alternance each time the clean_d_cache macro is used.
 * Without this the XScale core exhibits cache eviction problems and no one
 * knows why.
 *
 * Reminder: the vector table is located at 0xffff0000-0xffff0fff.
 */
#define CLEAN_ADDR	0xfffe0000

/*
 * This macro is used to wait for a CP15 write and is needed when we
 * have to ensure that the last operation to the coprocessor was
 * completed before continuing with operation.
 */
	.macro	cpwait, rd
	mrc	p15, 0, \rd, c2, c0, 0		@ arbitrary read of cp15
	mov	\rd, \rd			@ wait for completion
	sub 	pc, pc, #4			@ flush instruction pipeline
	.endm

	.macro	cpwait_ret, lr, rd
	mrc	p15, 0, \rd, c2, c0, 0		@ arbitrary read of cp15
	sub	pc, \lr, \rd, LSR #32		@ wait for completion and
						@ flush instruction pipeline
	.endm

/*
 * This macro cleans & invalidates the entire monahans dcache by set & way.
 * The main loop has been unrolled to reduce loop overhead.
 * NSets = 256, Nways = 4.  One complete set per loop.
 * rd,rs,rx,ry are scratch register.
 */
        .macro  clean_monahans_d_cache, rd, rs, rx, ry

        @ clean L1 dcache by set & way
        mov \rd, #0x1F00
        orr \rd, \rd, #0x00E0        @ put NSets (256)-1 into bits 12:5, way 0
        add \rs, \rd, #0x40000000    @ index for way 1
        add \rx, \rs, #0x40000000    @ index for way 2
        add \ry, \rx, #0x40000000    @ index for way 3
1:
        mcr p15, 0, \rd, c7, c14, 2  @ clean&inv set/way0 of L1 dcache specified in \rd
        mcr p15, 0, \rs, c7, c14, 2  @ clean&inv set/way1 of L1 dcache specified in \rs
        mcr p15, 0, \rx, c7, c14, 2  @ clean&inv set/way2 of L1 dcache specified in \rx
        mcr p15, 0, \ry, c7, c14, 2  @ clean&inv set/way3 of L1 dcache specified in \ry

        subs \rd, \rd, #0x00000020   @ decrement the shifted set index (way0)
        sub  \rs, \rs, #0x00000020   @ decrement the shifted set index (way1)
        sub  \rx, \rx, #0x00000020   @ decrement the shifted set index (way2)
        sub  \ry, \ry, #0x00000020   @ decrement the shifted set index (way3)
        bpl 1b                       @ go to next set if not done with all sets

        .endm

/*
 * This macro cleans & invalidates the entire monahans dcache by set & way.
 * The main loop has been unrolled to reduce loop overhead.
 * NSets = 256, Nways = 4.  One complete set per loop.
 * rd is a scratch register.
 */
        .macro  clean_monahans_d_cache2, rd

        @ clean L1 dcache by set & way
        mov \rd, #0x1F00
        orr \rd, \rd, #0x00E0        @ put NSets (256)-1 into bits 12:5, way 0
1:
        mcr p15, 0, \rd, c7, c14, 2  @ clean&inv set/way0 of L1 dcache specified in \rd
        adds \rd, \rd, #0x40000000   @ increment way index
        bcc 1b

        subs \rd, \rd, #0x00000020   @ decrement the shifted set index
        bpl 1b                       @ go to next set if not done with all sets

        .endm

/*
 * This macro cleans the entire dcache using line allocate.
 * The main loop has been unrolled to reduce loop overhead.
 * rd and rs are two scratch registers.
 */
 	.macro  clean_d_cache rd, rs
	mov	\rd, #0x1f00
	orr	\rd, \rd, #0x00e0
1:	mcr	p15, 0, \rd, c7, c14, 2		@ clean/invalidate L1 D line
	adds	\rd, \rd, #0x40000000
	bcc	1b
	subs	\rd, \rd, #0x20
	bpl	1b
	.endm

#ifdef CONFIG_PXA3xx
       .macro  get_l2_present, rd
       mrc     p15, 1, \rd, c0, c0, 1  @ Read L2 Cache Type Register for
       ands    \rd, \rd, #0xF8         @ Associativity. See if the L2 Cache
                                       @ is absent.
       .endm

@ This macro returns  the following, based on L2 cache size (assuming 8-way)
@ L2 Cache Size                Return Value
@ ----------------------------------
@ 256 KB               0b0010
@ 512 KB               0b0011
       .macro  get_l2_size, rd
       mrc     p15, 1, \rd, c0, c0, 1  @ Read L2 Cache Type Register for
       and     \rd, \rd, #0xf00        @ way size. Isolate L2 cache-way size
       mov     \rd, \rd, lsr #8        @ Shift cache-way size encoding
                                       @ to bits 3:0
       .endm
#endif

	.data
clean_addr:	.word	CLEAN_ADDR

	.text

/*
 * cpu_xsc3_proc_init()
 *
 * Nothing too exciting at the moment
 */
ENTRY(cpu_xsc3_proc_init)
	mov	pc, lr

/*
 * cpu_xsc3_proc_fin()
 */
ENTRY(cpu_xsc3_proc_fin)
	str	lr, [sp, #-4]!
	mov	r0, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r0
	bl	xsc3_flush_kern_cache_all	@ clean caches
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1800			@ ...IZ...........
	bic	r0, r0, #0x0006			@ .............CA.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldr	pc, [sp], #4

/*
 * cpu_xsc3_reset(loc)
 *
 * Perform a soft reset of the system.  Put the CPU into the
 * same state as it would be if it had been reset, and branch
 * to what would be the reset vector.
 *
 * loc: location to jump to for soft reset
 */
	.align	5
ENTRY(cpu_xsc3_reset)
	mov	r1, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r1			@ reset CPSR
	mrc	p15, 0, r1, c1, c0, 0		@ ctrl register
	bic	r1, r1, #0x3900			@ ..VIZ..S........
	bic	r1, r1, #0x0086			@ ........B....CA.
	mcr	p15, 0, r1, c1, c0, 0		@ ctrl register
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate L1 caches and BTB
	bic	r1, r1, #0x0001			@ ...............M
	mcr	p15, 0, r1, c1, c0, 0		@ ctrl register
	@ CAUTION: MMU turned off from this point.  We count on the pipeline
	@ already containing those two last instructions to survive.
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
	mov	pc, r0

/*
 * cpu_xsc3_do_idle()
 *
 * Cause the processor to idle
 *
 * For now we do nothing but go to idle mode for every case
 *
 * XScale supports clock switching, but using idle mode support
 * allows external hardware to react to system state changes.
 */
	.align	5

ENTRY(cpu_xsc3_do_idle)
	mov	r0, #1
	mcr	p14, 0, r0, c7, c0, 0		@ go to idle
	mov	pc, lr

/* ================================= CACHE ================================ */

/*
 *     flush_l2_cache_all()
 *
 *     Invalidate all L2 cache entries.
 */
ENTRY(xsc3_flush_l2cache_all)
       mov     ip, #0
       stmfd   sp!, {r0, r1, r2, r3, lr}
#ifdef CONFIG_CPU_PXA3xx_L2CACHE
	get_l2_present r0
	mcrne   p15, 1, r0, c9, c5, 1   @ unlock all L2 cache lines, if L2 present
       cpwait  ip

        mcr     p15, 0, ip, c7, c10, 5  @ DMB
        get_l2_size r0
        mov     r1, #0xFFFFFFE0
        mov     r2, #19
        sub     r2, r2, r0
        mov     r0, r1, lsl r2
        mov     r0, r0, lsr r2
L2Cleanloop:
        mcr     p15, 1, r0, c7, c15, 2  @ clean and invalidate set/way in r0
        adds    r0, r0, #0x20000000
        bcc     L2Cleanloop
        subs    r0, r0, #0x00000020
        bpl     L2Cleanloop

        mcr     p15, 0, ip, c7, c10, 5  @ DMB

#endif
       ldmfd   sp!, {r0, r1, r2, r3, pc}


/*
 *	flush_user_cache_all()
 *
 *	Invalidate all cache entries in a particular address
 *	space.
 */
ENTRY(xsc3_flush_user_cache_all)
	/* FALLTHROUGH */

/*
 *	flush_kern_cache_all()
 *
 *	Clean and invalidate the entire cache.
 */
ENTRY(xsc3_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
	stmfd	sp!, {r0, r1, r2, r3, r4, lr}
	clean_d_cache r0, r1
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ data write barrier
#ifdef	CONFIG_PXA3xx
	mcrne	p15, 0, ip, c7, c5, 4		@ prefetch flush
#endif
	cpwait	ip
	ldmfd	sp!, {r0, r1, r2, r3, r4, pc} 

/*
 *	flush_user_cache_range(start, end, vm_flags)
 *
 *	Invalidate a range of cache entries in the specified
 *	address space.
 *
 *	- start - start address (may not be aligned)
 *	- end	- end address (exclusive, may not be aligned)
 *	- vma	- vma_area_struct describing address space
 */
	.align	5
ENTRY(xsc3_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #MAX_AREA_SIZE
	bhs	__flush_whole_cache

1:	tst	r2, #VM_EXEC
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate L1 I line
	mcr	p15, 0, r0, c7, c14, 1		@ clean/invalidate L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 6		@ invalidate BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ data write barrier
#ifdef	CONFIG_PXA3xx
	mcrne	p15, 0, ip, c7, c5, 4		@ prefetch flush
#endif
	cpwait	ip
	mov	pc, lr

/*
 *	coherent_kern_range(start, end)
 *
 *	Ensure coherency between the I cache and the D cache in the
 *	region described by start.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 *
 *	Note: single I-cache line invalidation isn't used here since
 *	it also trashes the mini I-cache used by JTAG debuggers.
 */
ENTRY(xsc3_coherent_kern_range)
/* FALLTHROUGH */

/*
 *	coherent_user_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 *
 *	Note: single I-cache line invalidation isn't used here since
 *	it also trashes the mini I-cache used by JTAG debuggers.
 */
ENTRY(xsc3_coherent_user_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
#ifdef	CONFIG_PXA3xx
	mcr	p15, 0, r0, c7, c5, 4		@ prefetch flush
#endif
	cpwait	ip
	mov	pc, lr

/*
 *	flush_kern_dcache_page(void *page)
 *
 *	Ensure no D cache aliasing occurs, either with itself or
 *	the I cache.
 *
 *	- addr	- page aligned address
 */
ENTRY(xsc3_flush_kern_dcache_page)
	add	r1, r0, #PAGE_SZ
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean/invalidate L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
#ifdef CONFIG_PXA3xx
	mcr	p15, 0, r0, c7, c5, 4		@ prefetch flush
#endif
	cpwait	ip
	mov	pc, lr

/*
 *	dma_inv_range(start, end)
 *
 *	Invalidate (discard) the specified virtual address range.
 *	May not write back any entries.  If 'start' or 'end'
 *	are not cache line aligned, those lines must be written
 *	back.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(xsc3_dma_inv_range)
	mrc	p15, 0, r2, c0, c0, 0		@ read ID
	eor	r2, r2, #0x69000000
	eor	r2, r2, #0x00052000
	bics	r2, r2, #1
	beq	xsc3_dma_flush_range

	tst	r0, #CACHELINESIZE - 1
	bic	r0, r0, #CACHELINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean L1 D line
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	mcrne	p15, 1, r0, c7, c11, 1		@ clean L2 line
#endif
	tst	r1, #CACHELINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean L1 D line
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	mcrne	p15, 1, r1, c7, c11, 1		@ clean L2 line
#endif
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate L1 D line
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	mcr	p15, 1, r0, c7, c7, 1		@ invalidate L2 line
#endif
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	cpwait	ip
	mov	pc, lr

/*
 *	dma_clean_range(start, end)
 *
 *	Clean the specified virtual address range.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(xsc3_dma_clean_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	mcr	p15, 1, r0, c7, c11, 1		@ clean L2 line
#endif
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	cpwait	ip
	mov	pc, lr

/*
 *	dma_flush_range(start, end)
 *
 *	Clean and invalidate the specified virtual address range.
 *
 *	- start  - virtual start address
 *	- end	 - virtual end address
 */
ENTRY(xsc3_dma_flush_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean/invalidate L1 D line
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	mcr	p15, 1, r0, c7, c11, 1		@ clean L2 line
	mcr	p15, 1, r0, c7, c7, 1		@ invalidate L2 line
#endif
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	cpwait	ip
	mov	pc, lr

ENTRY(xsc3_cache_fns)
	.long	xsc3_flush_kern_cache_all
	.long	xsc3_flush_user_cache_all
	.long	xsc3_flush_user_cache_range
	.long	xsc3_coherent_kern_range
	.long	xsc3_coherent_user_range
	.long	xsc3_flush_kern_dcache_page
	.long	xsc3_dma_inv_range
	.long	xsc3_dma_clean_range
	.long	xsc3_dma_flush_range

ENTRY(cpu_xsc3_dcache_clean_area)
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	add	r0, r0, #CACHELINESIZE
	subs	r1, r1, #CACHELINESIZE
	bhi	1b
	cpwait	ip
	mov	pc, lr

/* ================================ CACHE LOCKING============================
 *
 * The XScale MicroArchitecture implements support for locking entries into
 * the data and instruction cache.  The following functions implement the core
 * low level instructions needed to accomplish the locking.  The developer's
 * manual states that the code that performs the locking must be in non-cached
 * memory.  To accomplish this, the code in xscale-cache-lock.c copies the
 * following functions from the cache into a non-cached memory region that
 * is allocated through consistent_alloc().
 *
 */
	.align	5
/*
 * xsc3_icache_lock
 *
 * r0: starting address to lock
 * r1: end address to lock
 */
ENTRY(xsc3_icache_lock)
	bic	r0, r0, #CACHELINESIZE - 1
iLockLoop:
	mcr	p15, 0, r0, c9, c1, 0	@ lock into cache
	add	r0, r0, #CACHELINESIZE	@ advance to next cache line
	cmp	r0, r1			@ are we done?
	blo	iLockLoop
	cpwait	ip
	mov	pc, lr

/*
 * xsc3_icache_unlock
 */
ENTRY(xsc3_icache_unlock)
	stmfd	sp!, {r0, lr}
	mcr	p15, 0, r0, c9, c1, 1	@ Unlock icache
	cpwait	ip
	ldmfd	sp!, {r0, pc}
	@mov	pc, lr

/*
 * xsc3_dcache_lock
 *
 * r0: starting address to lock
 * r1: end address to lock
 */
ENTRY(xsc3_dcache_lock)
	mcr	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	r2, #1
	mcr	p15, 0, r2, c9, c2, 0	@ Put dcache in lock mode
	cpwait	ip			@ Wait for completion

	mrs	r2, cpsr
	orr	r3, r2, #PSR_F_BIT | PSR_I_BIT
dLockLoop:
	msr	cpsr_c, r3
	mcr	p15, 0, r0, c7, c10, 1	@ Write back line if it is dirty
	mcr	p15, 0, r0, c7, c6, 1	@ Flush/invalidate line
	msr	cpsr_c, r2
	ldr	ip, [r0], #CACHELINESIZE @ Preload 32 bytes into cache from
					@ location [r0]. Post-increment
					@ r3 to next cache line
	cmp	r0, r1			@ Are we done?
	blo	dLockLoop

	mcr	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	r2, #0
	mcr	p15, 0, r2, c9, c2, 0	@ Get out of lock mode
	cpwait_ret lr, ip

/*
 * xsc3_dcache_unlock
 */
ENTRY(xsc3_dcache_unlock)
	mcr	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mcr	p15, 0, ip, c9, c2, 1	@ Unlock cache
	cpwait	ip
	mov	pc, lr

#ifdef CONFIG_PXA3xx
/*
 * xsc3_icache_unlock_range
 *
 * r0: starting address to unlock
 * r1: end address to unlock
 */
ENTRY(xsc3_icache_unlock_range)
        bic     r0, r0, #CACHELINESIZE - 1
iUnLockLoop:
        mcr     p15, 0, r0, c7, c5, 1   @ unlock from cache
        add     r0, r0, #CACHELINESIZE  @ advance to next cache line
        cmp     r0, r1                  @ are we done?
        blo     iUnLockLoop
        mov     pc, lr

/*
 * xsc3_dcache_unlock_range
 *
 * r0: starting address to unlock
 * r1: end address to unlock
 */
ENTRY(xsc3_dcache_unlock_range)
        bic     r0, r0, #CACHELINESIZE - 1
dUnLockLoop:
        mcr     p15, 0, r0, c7, c14, 1  @ unlock from cache
        add     r0, r0, #CACHELINESIZE  @ advance to next cache line
        cmp     r0, r1                  @ are we done?
        blo     dUnLockLoop
        mov     pc, lr

/*
 * xsc3_l2_cache_fetch_and_lock
 *
 * r0: starting address to lock
 * r1: end address to lock
 */
ENTRY(xsc3_l2_cache_fetch_and_lock)
       bic     r0, r0, #CACHELINESIZE - 1
L2FLockLoop:
       mcr     p15, 1, r0, c9, c5, 0   @ fetch/lock into l2 cache
       add     r0, r0, #CACHELINESIZE  @ advance to next cache line
       cmp     r0, r1                  @ are we done?
       blo     L2FLockLoop
       mov     pc, lr

/*
 * xsc3_l2_cache_allocate_and_lock
 *
 * r0: starting address to allocate
 * r1: end address to allocate
 */
ENTRY(xsc3_l2_cache_allocate_and_lock)

       tst     r0, #CACHELINESIZE - 1
       mcrne   p15, 1, r0, c9, c5, 0   @ fetch/lock into l2 cache
       addne   r0, r0, #CACHELINESIZE
       bic     r0, r0, #CACHELINESIZE - 1
       tst     r1, #CACHELINESIZE - 1
       mcrne   p15, 1, r1, c9, c5, 0   @ fetch/lock into l2 cache
       subne   r1, r1, #CACHELINESIZE
L2ALockLoop:
       mcr     p15, 1, r0, c9, c5, 2   @ allocate/lock into l2 cache
       add     r0, r0, #CACHELINESIZE  @ advance to next cache line
       cmp     r0, r1                  @ are we done?
       blo     L2ALockLoop
       mov     pc, lr

/*
 * xsc3_l2_cache_unlock
 */
ENTRY(xsc3_l2_cache_unlock)
       mcr     p15, 1, r0, c9, c5, 1   @ Unlock l2 cache
       mov     pc, lr

/*
 * xsc3_l2_cache_unlock_range
 *
 * r0: starting address to unlock
 * r1: end address to unlock
 */
ENTRY(xsc3_l2_cache_unlock_range)
        bic     r0, r0, #CACHELINESIZE - 1
L2UnLockLoop:
        mcr     p15, 1, r0, c7, c11, 1  @ clean L2 cache line
       mcr     p15, 0, r0, c7, c10, 5  @ DMB
        mcr     p15, 1, r0, c7, c7, 1   @ invalidate/unlock L2 cache line
        add     r0, r0, #CACHELINESIZE  @ advance to next cache line
        cmp     r0, r1                  @ are we done?
        blo     L2UnLockLoop
        mov     pc, lr

/*
 * xsc3_l2_cache_invalidate_range
 *
 * r0: starting address to invalidate/unlock
 * r1: end address to invalidate/unlock
 */
ENTRY(xsc3_l2_cache_invalidate_range)

       tst     r0, #CACHELINESIZE - 1
       mcrne   p15, 1, r0, c7, c11, 1  @ clean L2 entry
       tst     r1, #CACHELINESIZE - 1
       mcrne   p15, 1, r1, c7, c11, 1  @ clean L2 entry
        bic     r0, r0, #CACHELINESIZE - 1
L2InvalidateLoop:
        mcr     p15, 1, r0, c7, c7, 1   @ invalidate/unlock L2 cache line
        add     r0, r0, #CACHELINESIZE  @ advance to next cache line
        cmp     r0, r1                  @ are we done?
        blo     L2InvalidateLoop
        mov     pc, lr

#endif /* #ifdef CONFIG_PXA3xx */

/*
 * Needed to determine the length of the code that needs to be copied.
 */
	.align	5
ENTRY(xsc3_cache_dummy)
	mov	pc, lr

/* ================================ TLB LOCKING==============================
 *
 * The XScale MicroArchitecture implements support for locking entries into
 * the Instruction and Data TLBs.  The following functions provide the
 * low level support for supporting these under Linux.  xscale-lock.c
 * implements some higher level management code.  Most of the following
 * is taken straight out of the Developer's Manual.
 */

/*
 * Lock I-TLB entry
 *
 * r0: Virtual address to translate and lock
 */
	.align	5
ENTRY(xsc3_itlb_lock)
	mrs	r2, cpsr
	orr	r3, r2, #PSR_F_BIT | PSR_I_BIT
	msr	cpsr_c, r3			@ Disable interrupts
	mcr	p15, 0, r0, c8, c5, 1		@ Invalidate I-TLB entry
	mcr	p15, 0, r0, c10, c4, 0		@ Translate and lock
	msr	cpsr_c, r2			@ Restore interrupts
	cpwait_ret lr, ip

/*
 * Lock D-TLB entry
 *
 * r0: Virtual address to translate and lock
 */
	.align	5
ENTRY(xsc3_dtlb_lock)
	mrs	r2, cpsr
	orr	r3, r2, #PSR_F_BIT | PSR_I_BIT
	msr	cpsr_c, r3			@ Disable interrupts
	mcr	p15, 0, r0, c8, c6, 1		@ Invalidate D-TLB entry
	mcr	p15, 0, r0, c10, c8, 0		@ Translate and lock
	msr	cpsr_c, r2			@ Restore interrupts
	cpwait_ret lr, ip

/*
 * Unlock all I-TLB entries
 */
	.align	5
ENTRY(xsc3_itlb_unlock)
	mcr	p15, 0, ip, c10, c4, 1		@ Unlock I-TLB
	mcr	p15, 0, ip, c8, c5, 0		@ Invalidate I-TLB
	cpwait_ret lr, ip

/*
 * Unlock all D-TLB entries
 */
ENTRY(xsc3_dtlb_unlock)
	mcr	p15, 0, ip, c10, c8, 1		@ Unlock D-TBL
	mcr	p15, 0, ip, c8, c6, 0		@ Invalidate D-TLB
	cpwait_ret lr, ip


/* =============================== PageTable ============================== */

#define PTE_CACHE_WRITE_ALLOCATE 0

/*
 * cpu_xsc3_switch_mm(pgd)
 *
 * Set the translation base pointer to be as described by pgd.
 *
 * pgd: new page tables
 */
	.align	5
ENTRY(cpu_xsc3_switch_mm)
	stmfd	sp!, {r3-r4, lr}
	clean_d_cache r1, r2
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcr	p15, 0, ip, c7, c10, 4		@ data write barrier
#ifdef	CONFIG_PXA3xx
	mcr	p15, 0, ip, c7, c5, 4		@ prefetch flush
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	orr	r0, r0, #0x18			@ cache the page table in L2
#endif
#endif
	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
	cpwait_ret lr, ip
	ldmfd	sp!, {r3-r4, pc}

/*
 * cpu_xsc3_set_pte_ext(ptep, pte, ext)
 *
 * Set a PTE and flush it out
 *
 * Errata 40: must set memory to write-through for user read-only pages.
 */
	.align	5
ENTRY(cpu_xsc3_set_pte_ext)
	str	r1, [r0], #-2048		@ linux version

	bic	r2, r1, #0xff0			@ keep C, B bits
	orr	r2, r2, #PTE_TYPE_EXT		@ extended page
	tst	r1, #L_PTE_SHARED		@ shared?
	orrne	r2, r2, #0x200

	eor	r3, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

	tst	r3, #L_PTE_USER			@ user?
	orrne	r2, r2, #PTE_EXT_AP_URO_SRW	@ yes -> user r/o, system r/w

	tst	r3, #L_PTE_WRITE | L_PTE_DIRTY	@ write and dirty?
	orreq	r2, r2, #PTE_EXT_AP_UNO_SRW	@ yes -> user n/a, system r/w
						@ combined with user -> user r/w

#if CONFIG_CPU_PXA3xx_L2CACHE
	@ If it's cacheable, it needs to be in L2 also.
	eor	ip, r1, #L_PTE_CACHEABLE
	tst	ip, #L_PTE_CACHEABLE
	orreq	r2, r2, #PTE_EXT_TEX(0x5)
#endif

	tst	r3, #L_PTE_PRESENT | L_PTE_YOUNG	@ present and young?
	movne	r2, #0				@ no -> fault

	str	r2, [r0]			@ hardware version
	mov	ip, #0
	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	mcr	p15, 0, ip, c7, c10, 4		@ data write barrier
	cpwait	ip
	mov	pc, lr

	.ltorg

	.align

	__INIT

	.type	__xsc3_setup, #function
__xsc3_setup:
	mov	r0, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate L1 caches and BTB
	mcr	p15, 0, ip, c7, c10, 4		@ data write barrier
#ifdef	CONFIG_PXA3xx
	mcr	p15, 0, ip, c7, c5, 4		@ prefetch flush
#endif
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
#ifdef	CONFIG_PXA3xx
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	mov	r0, #0x0400			@ LLR OC bits, outer write-back
#endif
	orr	r0, r0, #0x0030			@ LLR IC bits, inner write-back
	mcr	p15, 0, r0, c1, c0, 1		@ write aux control reg
#endif
#ifdef	CONFIG_CPU_PXA3xx_L2CACHE
	orr	r4, r4, #0x18			@ cache the page table in L2
#endif
	mcr	p15, 0, r4, c2, c0, 0		@ load page table pointer
	mov	r0, #0				@ don't allow CP access
	mcr	p15, 0, r0, c15, c1, 0		@ write CP access register
	mrc	p15, 0, r0, c1, c0, 1		@ get auxiliary control reg
	and	r0, r0, #2			@ preserve bit P bit setting
#if CONFIG_CPU_PXA3xx_L2CACHE
	orr	r0, r0, #(1 << 10)		@ enable L2 for LLR cache
#endif
	mcr	p15, 0, r0, c1, c0, 1		@ set auxiliary control reg

	adr	r5, xsc3_crval
	ldmia	r5, {r5, r6}
	mrc	p15, 0, r0, c1, c0, 0		@ get control register
	bic	r0, r0, r5			@ ..V. ..R. .... ..A.
	orr	r0, r0, r6			@ ..VI Z..S .... .C.M (mmu)
						@ ...I Z..S .... .... (uc)
#if CONFIG_CPU_PXA3xx_L2CACHE
	orr 	r0, r0, #0x04000000		@ L2 enable
#endif
	mov	pc, lr

	.size	__xsc3_setup, . - __xsc3_setup

	.type	xsc3_crval, #object
xsc3_crval:
	crval	clear=0x04002202, mmuset=0x00003905, ucset=0x00001900

	__INITDATA

/*
 * Purpose : Function pointers used to access above functions - all calls
 *	     come through these
 */

	.type	xsc3_processor_functions, #object
ENTRY(xsc3_processor_functions)
	.word	v5t_early_abort
	.word	cpu_xsc3_proc_init
	.word	cpu_xsc3_proc_fin
	.word	cpu_xsc3_reset
	.word	cpu_xsc3_do_idle
	.word	cpu_xsc3_dcache_clean_area
	.word	cpu_xsc3_switch_mm
	.word	cpu_xsc3_set_pte_ext
	.size	xsc3_processor_functions, . - xsc3_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv5te"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v5"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_xsc3_name, #object
cpu_xsc3_name:
	.asciz	"XScale-V3 based processor"
	.size	cpu_xsc3_name, . - cpu_xsc3_name

	.align

	.section ".proc.info.init", #alloc, #execinstr

	.type	__xsc3_proc_info,#object
__xsc3_proc_info:
	.long	0x69056000
	.long	0xffffe000
	.long	PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long	PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xsc3_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_xsc3_name
	.long	xsc3_processor_functions
	.long	v4wbi_tlb_fns
	.long	xsc3_mc_user_fns
	.long	xsc3_cache_fns
	.size	__xsc3_proc_info, . - __xsc3_proc_info
