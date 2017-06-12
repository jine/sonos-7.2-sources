#ifndef __PXAFB_H__
#define __PXAFB_H__

/*
 * linux/drivers/video/pxafb.h
 *    -- Intel PXA250/210 LCD Controller Frame Buffer Device
 *
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 *  2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	 - ported SA1100 code to PXA
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.

 *(C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 */

#ifdef CONFIG_FB_PXA_MINILCD

#include <asm/ioctl.h>

/* commands for pxafb_minilcd_ioctl() */

#define PXAFB_MINILCD_ENABLE		_IOW('F', 0x80, unsigned int)
#define PXAFB_MINILCD_BACKLIGHT		_IOW('F', 0x81, unsigned int)
#define PXAFB_MINILCD_WAKEUP		_IOW('F', 0x82, unsigned int)
#define PXAFB_MINILCD_FWAKEUP		_IOW('F', 0x83, unsigned int)
#define PXAFB_MINILCD_FRAMEDATA		_IOW('F', 0x84, void *)

/* Mini-LCD register definitions */

#define MLCCR0			__REG_2(0x46000000)
#define MLCCR1			__REG_2(0x46000004)
#define MLCCR2			__REG_2(0x46000008)
#define MLSADD			__REG_2(0x4600000C)
#define MLFRMCNT		__REG_2(0x46000010)

#define MLCCR0_OEP		(1 << 11)
#define MLCCR0_PCP		(1 << 10)
#define MLCCR0_VSP		(1 << 9)
#define MLCCR0_HSP		(1 << 8)
#define MLCCR0_PCD(d)		((d) & 0xff)

#define MLCCR1_BLW(n)		(((n) & 0xff) << 24)
#define MLCCR1_ELW(n)		(((n) & 0xff) << 16)
#define MLCCR1_HSW(n)		(((n) & 0x3f) << 10)
#define MLCCR1_PPL(n)		(((n) & 0x3ff)

#define MLCCR2_BFW(n)		(((n) & 0xff) << 24)
#define MLCCR2_EFW(n)		(((n) & 0xff) << 16)
#define MLCCR2_VSW(n)		(((n) & 0x3f) << 10)
#define MLCCR2_LPP(n)		(((n) & 0x3ff)

#define MLFRMCNT_WKUP		(1U << 31)
#define MLFRMCNT_FWKUP		(1U << 30)
#define MLFRMCNT_FRCOUNT(n)	((n) & 0x3ff)
#define MLFRMCNT_FRCOUNT_MASK	(0x3ff)

/* Shadows for Mini-LCD controller registers */
struct pxafb_minilcd_reg {
	uint32_t mlccr0;
	uint32_t mlccr1;
	uint32_t mlccr2;
	uint32_t mlsadd;
	uint32_t mlfrmcnt;
};

/* 
 * pxafb_minilcd_info - run-time information to enable mini-lcd
 * enable     - enable in low power mode (S0/D1/C2)
 * framecount - shadow of register MLFRMCNT
 * frameaddr  - shadow of register MLSADR
 * framedata  - points to the encoded data from user specified buffer,
 *              or NULL if the base frame buffer is going to be used.
 * framesize  - size of the encoded frame data if 'framedata' is not NULL
 */
struct pxafb_minilcd_info {
	unsigned int	enable;
	unsigned int	backlight;
	uint32_t	framecount;
	void *		framedata;
	size_t		framesize;

	uint32_t	sram_addr_phys; /* Physical address of the SRAM */
	void *		sram_addr_virt; /* Virtual address of the SRAM */
	void *		sram_save_to;	/* address to backup SRAM into */
	size_t		sram_save_size; /* size of saved SRAM */
};

extern int pxafb_minilcd_register(struct fb_info *);
extern int pxafb_minilcd_ioctl(struct fb_info *info, unsigned int cmd,
				unsigned long arg);

extern int pxafb_minilcd_enter(void);
extern int pxafb_minilcd_exit(void);
#endif

/* Shadows for LCD controller registers */
struct pxafb_lcd_reg {
	unsigned int lccr0;
	unsigned int lccr1;
	unsigned int lccr2;
	unsigned int lccr3;
};

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
/* PXA Overlay Framebuffer Support */
struct overlayfb_info
{
	struct fb_info	fb;
	
	struct fb_var_screeninfo old_var;

	struct semaphore mutex;
	unsigned long	 refcount;

	struct pxafb_info *basefb;

	unsigned long	map_cpu;
	unsigned long 	screen_cpu;
	unsigned long	palette_cpu;
	unsigned long 	map_size;
	unsigned long   palette_size;

	dma_addr_t 	screen_dma;
	dma_addr_t	map_dma;
	dma_addr_t	palette_dma;

	volatile u_char	state;

	/* overlay specific info */
	unsigned long	xpos;		/* screen position (x, y)*/
	unsigned long	ypos;		
	unsigned long 	format;
	unsigned int	buffer_num;
	unsigned int	buffer_index;
	unsigned int	ylen;
	unsigned int	cblen;
	unsigned int	crlen;
	unsigned int	yoff;
	unsigned int	cboff; 
	unsigned int	croff;

	/* additional */
	union {
		struct pxafb_dma_descriptor *dma0;
		struct pxafb_dma_descriptor *dma1;
		struct {
			struct pxafb_dma_descriptor *dma2;
			struct pxafb_dma_descriptor *dma3;
			struct pxafb_dma_descriptor *dma4;
		};
		struct {
			struct pxafb_dma_descriptor *dma5_pal;
			struct pxafb_dma_descriptor *dma5_frame;
		};
	};
};
#endif

/* PXA LCD DMA descriptor */
struct pxafb_dma_descriptor {
	unsigned int fdadr;
	unsigned int fsadr;
	unsigned int fidr;
	unsigned int ldcmd;
};

struct pxafb_info {
	struct fb_info		fb;
	struct device		*dev;

	u_int			max_bpp;
	u_int			max_xres;
	u_int			max_yres;

	/*
	 * These are the addresses we mapped
	 * the framebuffer memory region to.
	 */
	/* raw memory addresses */
	dma_addr_t		map_dma;	/* physical */
	u_char *		map_cpu;	/* virtual */
	u_int			map_size;

	/* addresses of pieces placed in raw buffer */
	u_char *		screen_cpu;	/* virtual address of frame buffer */
	dma_addr_t		screen_dma;	/* physical address of frame buffer */
	u16 *			palette_cpu;	/* virtual address of palette memory */
	dma_addr_t		palette_dma;	/* physical address of palette memory */
	u_int			palette_size;

	/* DMA descriptors */
	struct pxafb_dma_descriptor * 	dmadesc_fbfront_cpu;
	dma_addr_t		dmadesc_fbfront_dma;
	struct pxafb_dma_descriptor * 	dmadesc_fbback_cpu;
	dma_addr_t		dmadesc_fbback_dma;
	struct pxafb_dma_descriptor *	dmadesc_palette_cpu;
	dma_addr_t		dmadesc_palette_dma;

	dma_addr_t		fdadr0;

	u_int			lccr0;
	u_int			lccr3;
	u_int			cmap_inverse:1,
				cmap_static:1,
				unused:30;

	u_int			reg_lccr0;
	u_int			reg_lccr1;
	u_int			reg_lccr2;
	u_int			reg_lccr3;

	unsigned long	hsync_time;

	volatile u_char		state;
	volatile u_char		task_state;
	struct semaphore	ctrlr_sem;
	wait_queue_head_t	ctrlr_wait;
	struct work_struct	task;

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	/* PXA Overlay Framebuffer Support */
	struct overlayfb_info  *overlay1fb;
	struct overlayfb_info  *overlay2fb;
	struct overlayfb_info  *cursorfb;
	void (*set_overlay_ctrlr_state)(struct pxafb_info *, u_int);
#endif

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
	struct notifier_block	freq_policy;
#endif

#ifdef CONFIG_FB_PXA_MINILCD
	struct pxafb_minilcd_info minilcd_info;
#endif
};

#define TO_INF(ptr,member) container_of(ptr,struct pxafb_info,member)

/*
 * These are the actions for set_ctrlr_state
 */
#define C_DISABLE		(0)
#define C_ENABLE		(1)
#define C_DISABLE_CLKCHANGE	(2)
#define C_ENABLE_CLKCHANGE	(3)
#define C_REENABLE		(4)
#define C_DISABLE_PM		(5)
#define C_ENABLE_PM		(6)
#define C_STARTUP		(7)
#define C_BLANK  		(8)
#define C_UNBLANK		(9)


#define PXA_NAME	"PXA"

/*
 * Minimum X and Y resolutions
 */
#define MIN_XRES	64
#define MIN_YRES	64

#endif /* __PXAFB_H__ */
