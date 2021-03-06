/*
 *  linux/drivers/video/pxafb_overlay.c
 *
 *  Copyright (c) 2004, Intel Corporation
 *
 * 	Code Status:
 * 	2004/10/28: <yan.yin@intel.com>
 *      - Ported to 2.6 kernel
 *      - Made overlay driver a loadable module
 *      - Merged overlay optimized patch
 * 	2004/03/10: <stanley.cai@intel.com>
 *      - Fixed Bugs
 *      - Added workaround for overlay1&2
 * 	2003/08/27: <yu.tang@intel.com>
 *      - Added Overlay 1 & Overlay2 & Hardware Cursor support
 *
 *
 * 	This software program is licensed subject to the GNU Lesser General 
 * 	Public License (LGPL). Version 2.1, February 1999, available at 
 * 	http://www.gnu.org/copyleft/lesser.html
 *
 * 	Intel PXA27x LCD Controller Frame Buffer Overlay Driver
 *
 *
 * (C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/arch/bitfield.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/pxa-regs.h>

#include "pxafb.h"

/*Miangliang*/
#ifdef CONFIG_FB_PXA_SMART_PANEL
#include "oledfb.h"
#endif

/* LCD enhancement : Overlay 1 & 2 & Hardware Cursor */

/* 
 * LCD enhancement : Overlay 1 
 *
 * Features:
 * - support 16bpp (No palette)
 */
/*
 * debugging?
 */
#define DEBUG 0

#ifdef  DEBUG
#define dbg(fmt,arg...) printk(KERN_ALERT "%s(): " fmt "\n", __FUNCTION__, ##arg)
#else
#define dbg(fmt,arg...)
#endif

#ifdef CONFIG_FB_PXA_SMART_PANEL
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
static int SMART_OVERLAY1 = FALSE;
static int SMART_OVERLAY2 = FALSE;
#endif

static int overlay1fb_enable(struct fb_info *info);
static int overlay2fb_enable(struct fb_info *info);
static int cursorfb_enable(struct fb_info *info);

static int overlay1fb_disable(struct fb_info *info);
static int overlay2fb_disable(struct fb_info *info);
static int cursorfb_disable(struct fb_info *info);

static int overlay1fb_blank(int blank, struct fb_info *info);
static int overlay2fb_blank(int blank, struct fb_info *info);
static int cursorfb_blank(int blank, struct fb_info *info);

extern void set_ctrlr_state(struct pxafb_info *fbi, u_int state);
extern int pxafb_blank(int blank, struct fb_info *info);

#ifdef CONFIG_FB_PXA_SMART_PANEL
extern struct lcd_smart_info *smart_fbi;
extern void pxa_smartfb_set_ctrlr_state(struct lcd_smart_info *fbi, u_int state);
#endif
	
#define CLEAR_LCD_INTR(reg, intr) do {  \
	reg = (intr);			\
}while(0)

#define WAIT_FOR_LCD_INTR(reg,intr,timeout) ({	\
	int __done =0;				\
	int __t = timeout;			\
	while (__t) {				\
		__done = (reg) & (intr);	\
		if (__done) {			\
			(reg) = (intr);		\
			break;			\
		}				\
		mdelay(10);			\
		__t--;				\
	}					\
	if (!__t) dbg("wait " #intr " timeount");\
	__done;					\
})
	
#define DISABLE_OVERLAYS(fbi) do {	 				\
	if (fbi->overlay1fb && (fbi->overlay1fb->state == C_ENABLE)) {	\
		overlay1fb_disable((struct fb_info*)fbi->overlay1fb);	\
	}								\
	if (fbi->overlay2fb && (fbi->overlay2fb->state == C_ENABLE)) {	\
		overlay2fb_disable((struct fb_info*)fbi->overlay2fb);	\
	}								\
	if (fbi->cursorfb && (fbi->cursorfb->state == C_ENABLE)) {	\
		cursorfb_disable((struct fb_info*)fbi->cursorfb);	\
	}								\
}while(0)

#define ENABLE_OVERLAYS(fbi) do {					\
	if (fbi->overlay1fb && (fbi->overlay1fb->state == C_DISABLE)){ 	\
		overlay1fb_enable((struct fb_info*)fbi->overlay1fb);	\
	}								\
	if (fbi->overlay2fb && (fbi->overlay2fb->state == C_DISABLE)){	\
		overlay2fb_unblank_workaroundYUV420((struct fb_info*)fbi->overlay2fb);	\
		overlay2fb_enable((struct fb_info*)fbi->overlay2fb);	\
	}								\
	if (fbi->cursorfb && (fbi->cursorfb->state == C_DISABLE)){	\
		cursorfb_enable((struct fb_info*)fbi->cursorfb);	\
	}								\
}while(0)

#define BLANK_OVERLAYS(fbi) do {	 				\
	if (fbi->overlay1fb && (fbi->overlay1fb->state == C_ENABLE)) {	\
		overlay1fb_disable((struct fb_info*)fbi->overlay1fb);	\
		fbi->overlay1fb->state = C_BLANK;			\
	}								\
	if (fbi->overlay2fb && (fbi->overlay2fb->state == C_ENABLE)) {	\
		overlay2fb_disable((struct fb_info*)fbi->overlay2fb);	\
		fbi->overlay2fb->state = C_BLANK;			\
	}								\
	if (fbi->cursorfb && (fbi->cursorfb->state == C_ENABLE)) {	\
		cursorfb_disable((struct fb_info*)fbi->cursorfb);	\
		fbi->cursorfb->state = C_BLANK;			\
	}								\
}while(0)

#define UNBLANK_OVERLAYS(fbi) do {					\
	if (fbi->overlay1fb && (fbi->overlay1fb->state == C_BLANK)){ 	\
		overlay1fb_enable((struct fb_info*)fbi->overlay1fb);	\
		fbi->overlay1fb->state = C_ENABLE;			\
	}								\
	if (fbi->overlay2fb && (fbi->overlay2fb->state == C_BLANK)){	\
		overlay2fb_unblank_workaroundYUV420((struct fb_info*)fbi->overlay2fb);	\
		overlay2fb_enable((struct fb_info*)fbi->overlay2fb);	\
		fbi->overlay2fb->state = C_ENABLE;			\
	}								\
	if (fbi->cursorfb && (fbi->cursorfb->state == C_BLANK)){	\
		cursorfb_enable((struct fb_info*)fbi->cursorfb);	\
		fbi->cursorfb->state = C_ENABLE;			\
	}								\
}while(0)

static dma_addr_t	ov2_map_dma;	/* DMA address of overlay2 buffer */
static u_char		*ov2_map_cpu;	/* cpu address of overlay2 buffer */
/* max size of overlay2 buffer
 * RGB 25 bit. duel buffer. (640x480x4x2)
 */
static u_int ov2_map_size = 2457600;

static void* ov2_dma_alloc_writecombine(struct device *dev,
		size_t size, dma_addr_t *handle, gfp_t gfp)
{
	if (!ov2_map_cpu) {	/* No pre-alloc DMA buffer */
		return NULL;
	}

	*handle = ov2_map_dma;

	return ov2_map_cpu;
}

static void ov2_dma_free_writecombine(struct device *dev,
		size_t size, void *cpu_addr, dma_addr_t handle)
{
	cpu_addr = NULL;
	handle = ~0;
}

static int overlay1fb_open(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int ret = 0;

	/* if basefb is disable, enable fb. */
	if (fbi->basefb && fbi->basefb->state != C_ENABLE)
	{
		#ifdef CONFIG_FB_PXA_SMART_PANEL
		if(smart_fbi != NULL){
			if(smart_fbi->state != C_ENABLE){
				pxafb_blank(VESA_NO_BLANKING, (struct fb_info *)(fbi->basefb));
			}
			else{
				SMART_OVERLAY1 = TRUE;
			}
		}
		else{
			pxafb_blank(VESA_NO_BLANKING, (struct fb_info *)(fbi->basefb));
		}
		#else
		pxafb_blank(VESA_NO_BLANKING, (struct fb_info *)(fbi->basefb));
		#endif
	}
	
	down(&fbi->mutex);

	if (fbi->refcount) 
		ret = -EACCES;
	else
		fbi->refcount ++;

	up(&fbi->mutex);

	/* Initialize the variables in overlay1 framebuffer. */
	fbi->fb.var.xres = fbi->fb.var.yres = 0;
	fbi->fb.var.bits_per_pixel = 0;
	
	return ret;
}

static int overlay1fb_release(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	down(&fbi->mutex);

	if (fbi->refcount)
		fbi->refcount --;

	up(&fbi->mutex);
	/* disable overlay when released */
	overlay1fb_blank(1, info);

	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if(SMART_OVERLAY1 == TRUE)
	{
		SMART_OVERLAY1 =FALSE;
	}
	#endif
		
	return 0;
}

static int overlay1fb_map_video_memory(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	if (fbi->map_cpu) 
		dma_free_writecombine(NULL, fbi->map_size, (void*)fbi->map_cpu,  fbi->map_dma);
	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);

	fbi->map_cpu = (unsigned long)dma_alloc_writecombine(NULL, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL );

	if (!fbi->map_cpu) return -ENOMEM;

	fbi->screen_cpu = fbi->map_cpu + PAGE_SIZE;
	fbi->screen_dma = fbi->map_dma + PAGE_SIZE;
	
	fbi->fb.fix.smem_start = fbi->screen_dma;

	/* setup dma descriptor */
	fbi->dma1 = (struct pxafb_dma_descriptor*)
		(fbi->screen_cpu - sizeof(struct pxafb_dma_descriptor));

	fbi->dma1->fdadr = (fbi->screen_dma - sizeof(struct pxafb_dma_descriptor));
	fbi->dma1->fsadr = fbi->screen_dma;
	fbi->dma1->fidr  = 0;
	fbi->dma1->ldcmd = fbi->fb.fix.smem_len;

	return 0;
}

static int overlay1fb_enable(struct fb_info *info) 
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	unsigned long bpp1;

	if (!fbi->refcount) return 0;
	if (!fbi->map_cpu) return -EINVAL;

	switch(fbi->fb.var.bits_per_pixel){
	case 16:
		bpp1 = 0x4;
		break;
	case 18:
		bpp1 = 0x6;
		break;
	case 19:
		bpp1 = 0x8;
		break;
	case 24:
		bpp1 = 0x9;
		break;
	case 25:
		bpp1 = 0xa;
		break;
	default:
		return -EINVAL;
	}

	/* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM1 | LCCR5_BSM1 | LCCR5_EOFM1 | LCCR5_SOFM1);

	if (fbi->state == C_DISABLE || fbi->state == C_BLANK)
		FDADR1 = (fbi->dma1->fdadr);
	else 
		FBR1 = fbi->dma1->fdadr | 0x1;

	/* enable overlay 1 window */
	OVL1C2 = (fbi->ypos << 10) | fbi->xpos;
	OVL1C1 = OVL1C1_O1EN | (bpp1 << 20) | ((fbi->fb.var.yres-1)<<10) | (fbi->fb.var.xres-1);

	fbi->state = C_ENABLE;

	return 0;
}

static int overlay1fb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	int done;

	if ((fbi->state == C_DISABLE) || (fbi->state == C_BLANK)) 
               return 0;

	fbi->state = C_DISABLE;

	/* clear O1EN */
	OVL1C1 &= ~OVL1C1_O1EN;

	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if(SMART_OVERLAY1 == TRUE){
		FDADR1 = 0;			
		goto OUT_1;
	}
	
	LCCR5 |= (LCCR5_IUM1 | LCCR5_BSM1 | LCCR5_EOFM1 | LCCR5_SOFM1);

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS1);
	FBR1 = FDADR1 | 0x3;
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS1, 100);

	if (!done) {
		pr_debug(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}

OUT_1:
	
	#else
	CLEAR_LCD_INTR(LCSR1, LCSR1_BS1);
	FBR1 = FDADR1 | 0x3;
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS1, 100);

	if (!done) {
		pr_debug(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	#endif

	return 0;
}

static int overlay1fb_blank(int blank, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int err=0;

	switch(blank) 
	{
	case 0:
		err = overlay1fb_enable(info);
		if (err) {
			fbi->state = C_DISABLE;

			#ifdef CONFIG_FB_PXA_SMART_PANEL
			if(smart_fbi != NULL && SMART_OVERLAY1 == TRUE){
				pxa_smartfb_set_ctrlr_state(smart_fbi, C_REENABLE);
			}
			else{
				set_ctrlr_state(fbi->basefb, C_REENABLE);
			}
			#else
			set_ctrlr_state(fbi->basefb, C_REENABLE);
			#endif
		}
		break;
	case 1:
		err = overlay1fb_disable(info);
		if (err) {
			fbi->state = C_DISABLE;
			
			#ifdef CONFIG_FB_PXA_SMART_PANEL
			if(smart_fbi != NULL && SMART_OVERLAY1 == TRUE){
				//extern void pxa_smartfb_set_ctrlr_state(struct lcd_smart_info *fbi, u_int state);
				//pxa_smartfb_set_ctrlr_state(smart_fbi, C_REENABLE);
			}
			else{
				set_ctrlr_state(fbi->basefb, C_REENABLE);
			}
			#else
			set_ctrlr_state(fbi->basefb, C_REENABLE);
			#endif
		}
		break;
	default:
		break;
	}

	return err;
}

static int overlay1fb_check_var( struct fb_var_screeninfo *var, struct fb_info *info)
{
	int xpos, ypos;
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;

	/* must in base frame */
	xpos = (var->nonstd & 0x3ff);
	ypos = ((var->nonstd>>10) & 0x3ff);

	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if(smart_fbi != NULL && SMART_OVERLAY1 == TRUE){
		if ( (xpos + var->xres) > smart_fbi->fb.var.xres ) {
			return -EINVAL;
		}
		if ( (ypos + var->yres) > smart_fbi->fb.var.yres ) {
			return -EINVAL;
		}
	}
	else{
		if ( (xpos + var->xres) > fbi->basefb->fb.var.xres ) 
			return -EINVAL;

		if ( (ypos + var->yres) > fbi->basefb->fb.var.yres ) 
			return -EINVAL;
	}
	#else
	if ( (xpos + var->xres) > fbi->basefb->fb.var.xres ) 
		return -EINVAL;

	if ( (ypos + var->yres) > fbi->basefb->fb.var.yres ) 
		return -EINVAL;
	#endif

	switch (var->bits_per_pixel) {
	case 16:
		if ( var->xres & 0x1 ) {
			printk("xres should be a multiple of 2 pixels!\n");
			return -EINVAL;
		}
		break;
	case 18:
	case 19:
		if ( var->xres & 0x7 ) {
			printk("xres should be a multiple of 8 pixels!\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	fbi->old_var=*var;

	var->activate=FB_ACTIVATE_NOW;

	return 0;
}


static int overlay1fb_set_par(struct fb_info *info)
{
	int nbytes=0, err=0, pixels_per_line=0;

	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
		
	info->flags &= ~FBINFO_MISC_USEREVENT;

	if (fbi->state == C_BLANK)
		return 0;

	if (fbi->state == C_DISABLE)
		goto out1;

	/* only xpos & ypos change */
	if ( (var->xres == fbi->old_var.xres) &&
		(var->yres == fbi->old_var.yres) &&
		(var->bits_per_pixel == fbi->old_var.bits_per_pixel) ) 
		goto out2;

 out1:
	switch(var->bits_per_pixel) {
		case 16:
			/* 2 pixels per line */ 
			pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
			nbytes = 2;

        		var->red.offset   = 10; var->red.length   = 5;
	        	var->green.offset = 5;  var->green.length = 5;
        		var->blue.offset  = 0;  var->blue.length  = 5;
			var->transp.offset= 15; var->transp.length= 1;

			break;
		case 18:
			/* 8 pixels per line */
			pixels_per_line = (fbi->fb.var.xres + 0x7 ) & (~0x7);
			nbytes = 3;

        		var->red.offset   = 12; var->red.length   = 6;
	        	var->green.offset = 6;  var->green.length = 6;
        		var->blue.offset  = 0;  var->blue.length  = 6;
			var->transp.offset = var->transp.length = 0;

			break;
		case 19:
			/* 8 pixels per line */
			pixels_per_line = (fbi->fb.var.xres + 0x7 ) & (~0x7);
			nbytes = 3;

        		var->red.offset   = 12; var->red.length   = 6;
	        	var->green.offset = 6;  var->green.length = 6;
        		var->blue.offset  = 0;  var->blue.length  = 6;
			var->transp.offset= 18; var->transp.length= 1;

			break;
		case 24:
			pixels_per_line = fbi->fb.var.xres;
			nbytes = 4;

        		var->red.offset   = 16; var->red.length   = 7;
	        	var->green.offset = 8;  var->green.length = 8;
        		var->blue.offset  = 0;  var->blue.length  = 8;
			var->transp.offset= 23; var->transp.length= 1;

			break;
		case 25:
			pixels_per_line = fbi->fb.var.xres;
			nbytes = 4;

        		var->red.offset   = 16; var->red.length   = 8;
	        	var->green.offset = 8;  var->green.length = 8;
        		var->blue.offset  = 0;  var->blue.length  = 8;
			var->transp.offset= 24; var->transp.length= 1;

			break;
		}

		fbi->fb.fix.line_length = nbytes * pixels_per_line;
		fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres;

		err= overlay1fb_map_video_memory((struct fb_info*)fbi);

		if (err) return err;

out2:		
		fbi->xpos = var->nonstd & 0x3ff;
		fbi->ypos = (var->nonstd>>10) & 0x3ff;

		overlay1fb_enable(info);

		return 0;

}

static struct fb_ops overlay1fb_ops = {
	.owner			= THIS_MODULE,
	.fb_open		= overlay1fb_open,
	.fb_release		= overlay1fb_release,
	.fb_check_var 		= overlay1fb_check_var,
	.fb_set_par		= overlay1fb_set_par,
	.fb_blank		= overlay1fb_blank,	
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
};

 /* 
 * LCD enhancement : Overlay 2 
 *
 * Features:
 * - support planar YCbCr420/YCbCr422/YCbCr444;
 */ 
static int overlay2fb_open(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int ret = 0;

	/* if basefb is disable, enable fb. */
	if (fbi->basefb && fbi->basefb->state != C_ENABLE)
	{
		#ifdef CONFIG_FB_PXA_SMART_PANEL
		if(smart_fbi != NULL){
			if(smart_fbi->state != C_ENABLE){
				pxafb_blank(VESA_NO_BLANKING, (struct fb_info *)(fbi->basefb));
			}
			else{
				SMART_OVERLAY2 = TRUE;
			}
		}
		else{
			pxafb_blank(VESA_NO_BLANKING, (struct fb_info *)(fbi->basefb));
		}
		#else
		pxafb_blank(VESA_NO_BLANKING, (struct fb_info *)(fbi->basefb));
		#endif
	}

	down(&fbi->mutex);

	if (fbi->refcount) 
		ret = -EACCES;
	else
		fbi->refcount ++;

	up(&fbi->mutex);
	fbi->fb.var.xres = fbi->fb.var.yres = 0;

	return ret;
}

static int overlay2fb_release(struct fb_info *info, int user)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	down(&fbi->mutex);

	if (fbi->refcount)
		fbi->refcount --;

	up(&fbi->mutex);

	/* disable overlay when released */
	overlay2fb_blank(1, info);

	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if(SMART_OVERLAY2 == TRUE)
	{
		SMART_OVERLAY2 =FALSE;
	}
	#endif

	return 0;
}

static int pxa_create_dma(struct overlayfb_info *info, int buffer_index)
{    
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	fbi->screen_cpu = fbi->map_cpu + buffer_index * (fbi->map_size / fbi->buffer_num);
	fbi->screen_dma = fbi->map_dma + buffer_index * (fbi->map_size / fbi->buffer_num);

	if(fbi->format == 0){
		/* setup dma descriptor */
		fbi->dma2 = (struct pxafb_dma_descriptor*)
			(fbi->screen_cpu + PAGE_SIZE - sizeof(struct pxafb_dma_descriptor));

		fbi->dma2->fdadr = (fbi->screen_dma + PAGE_SIZE - sizeof(struct pxafb_dma_descriptor));
		fbi->dma2->fsadr = fbi->screen_dma + PAGE_SIZE;
		fbi->dma2->fidr  = 0;
		fbi->dma2->ldcmd = fbi->fb.fix.line_length * fbi->fb.var.yres;
	}else{
		/* setup dma for Planar format */
		fbi->dma2 = (struct pxafb_dma_descriptor*)(fbi->screen_cpu + 
			PAGE_SIZE - sizeof(struct pxafb_dma_descriptor));
		fbi->dma3 = fbi->dma2 - 1;
		fbi->dma4 = fbi->dma3 - 1;
		
		/* Y vector */
		fbi->dma2->fdadr = (fbi->screen_dma + PAGE_SIZE - sizeof(struct pxafb_dma_descriptor));
		fbi->dma2->fsadr = fbi->screen_dma + PAGE_SIZE + fbi->yoff;
		fbi->dma2->fidr  = 0;
		fbi->dma2->ldcmd = fbi->ylen;

		/* Cb vector */
		fbi->dma3->fdadr = (fbi->dma2->fdadr - sizeof(struct pxafb_dma_descriptor));
		fbi->dma3->fsadr = (fbi->screen_dma + PAGE_SIZE + fbi->cboff);
		fbi->dma3->fidr  = 0;
		fbi->dma3->ldcmd = fbi->cblen;

		/* Cr vector */
		fbi->dma4->fdadr = (fbi->dma3->fdadr - sizeof(struct pxafb_dma_descriptor));
		fbi->dma4->fsadr = (fbi->screen_dma + PAGE_SIZE + fbi->croff);
		fbi->dma4->fidr  = 0;
		fbi->dma4->ldcmd = fbi->crlen;
	}
	
	return 0;
}

static int overlay2fb_map_YUV_memory( struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	unsigned int aylen, acblen, acrlen;
	unsigned int xres,yres; 
	unsigned int nbytes;

	fbi->ylen = fbi->cblen = fbi->crlen = 0;
	fbi->yoff = fbi->cboff = fbi->croff = 0;
	aylen = acblen = acrlen = 0;

	if (fbi->map_cpu) 
		ov2_dma_free_writecombine(NULL, fbi->map_size, (void*)fbi->map_cpu,  fbi->map_dma);

	yres = fbi->fb.var.yres;

	switch(fbi->format) {
	case 0x4: /* YCbCr 4:2:0 planar */
		pr_debug("420 planar\n");
		/* 16 pixels per line */
		xres = (fbi->fb.var.xres + 0xf) & (~0xf);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		fbi->ylen = nbytes;
		fbi->cblen = fbi->crlen = (nbytes/4);

		break;
	case 0x3: /* YCbCr 4:2:2 planar */
		/* 8 pixles per line */
		pr_debug("422 planar\n");
		xres = (fbi->fb.var.xres + 0x7) & (~0x7);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		fbi->ylen  = nbytes;
		fbi->cblen = fbi->crlen = (nbytes/2);

		break;
	case 0x2: /* YCbCr 4:4:4 planar */
		/* 4 pixels per line */
		pr_debug("444 planar\n");
		xres = (fbi->fb.var.xres + 0x3) & (~0x3);
		fbi->fb.fix.line_length = xres;

		nbytes = xres * yres;
		fbi->ylen  = fbi->cblen = fbi->crlen = nbytes;
		break;
	}

	/* 16-bytes alignment for DMA */
	aylen  = (fbi->ylen + 0xf) & (~0xf);
	acblen = (fbi->cblen + 0xf) & (~0xf);
	acrlen = (fbi->crlen + 0xf) & (~0xf);

	/* offset */
	fbi->yoff = 0;
	fbi->cboff = aylen;
	fbi->croff = fbi->cboff + acblen;

	/* adjust for user */
	fbi->fb.var.red.length   = fbi->ylen;
	fbi->fb.var.red.offset   = fbi->yoff;
	fbi->fb.var.green.length = fbi->cblen;
	fbi->fb.var.green.offset = fbi->cboff;
	fbi->fb.var.blue.length  = fbi->crlen;
	fbi->fb.var.blue.offset  = fbi->croff;

	if(fbi->buffer_num == 1){
		fbi->fb.fix.smem_len = aylen + acblen + acrlen;
	}else{
		fbi->fb.fix.smem_len = PAGE_ALIGN(aylen + acblen + acrlen + PAGE_SIZE) * fbi->buffer_num;
	}
	
	/* alloc memory */
	fbi->map_size = PAGE_ALIGN(aylen + acblen + acrlen + PAGE_SIZE) * fbi->buffer_num;
	fbi->map_cpu = (unsigned long)ov2_dma_alloc_writecombine(NULL, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL );
	if (!fbi->map_cpu) {
		printk(KERN_INFO "%s: Allocate DMA buffer for Overlay2 failed\n",
				__func__);
		return -ENOMEM;
	}

	memset((void *)(fbi->map_cpu + PAGE_SIZE + fbi->yoff),  0x10, fbi->ylen);
	memset((void *)(fbi->map_cpu + PAGE_SIZE + fbi->cboff), 0x80, fbi->cblen);
	memset((void *)(fbi->map_cpu + PAGE_SIZE + fbi->croff), 0x80, fbi->crlen);

	fbi->fb.fix.smem_start = fbi->map_dma + PAGE_SIZE;
	
	pxa_create_dma(fbi, fbi->buffer_index);
     
	return 0;
};

static int overlay2fb_map_RGB_memory( struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	int pixels_per_line=0 , nbytes=0;

	if (fbi->map_cpu) 
		ov2_dma_free_writecombine(NULL,  fbi->map_size, (void*)fbi->map_cpu, fbi->map_dma);

	switch(var->bits_per_pixel) {
	case 16:
		/* 2 pixels per line */ 
		pixels_per_line = (fbi->fb.var.xres + 0x1) & (~0x1);
		nbytes = 2;

        	var->red.offset   = 10; var->red.length   = 5;
	        var->green.offset = 5;  var->green.length = 5;
        	var->blue.offset  = 0;  var->blue.length  = 5;
		var->transp.offset= 15; var->transp.length= 1;
		break;

	case 18:
		/* 8 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x7 ) & (~0x7);
		nbytes = 3;

        	var->red.offset   = 12; var->red.length   = 6;
	        var->green.offset = 6;  var->green.length = 6;
        	var->blue.offset  = 0;  var->blue.length  = 6;
		var->transp.offset = var->transp.length = 0;

		break;
	case 19:
		/* 8 pixels per line */
		pixels_per_line = (fbi->fb.var.xres + 0x7 ) & (~0x7);
		nbytes = 3;

        	var->red.offset   = 12; var->red.length   = 6;
	        var->green.offset = 6;  var->green.length = 6;
        	var->blue.offset  = 0;  var->blue.length  = 6;
		var->transp.offset= 18; var->transp.length= 1;

		break;
	case 24:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

        	var->red.offset   = 16; var->red.length   = 7;
	       	var->green.offset = 8;  var->green.length = 8;
        	var->blue.offset  = 0;  var->blue.length  = 8;
		var->transp.offset= 23; var->transp.length= 1;

		break;

	case 25:
		pixels_per_line = fbi->fb.var.xres;
		nbytes = 4;

        	var->red.offset   = 16; var->red.length   = 8;
	        var->green.offset = 8;  var->green.length = 8;
        	var->blue.offset  = 0;  var->blue.length  = 8;
		var->transp.offset= 24; var->transp.length= 1;

		break;
	}

	fbi->fb.fix.line_length = nbytes * pixels_per_line ;
	if(fbi->buffer_num == 1){
		fbi->fb.fix.smem_len = fbi->fb.fix.line_length * fbi->fb.var.yres;
	}else{
		fbi->fb.fix.smem_len = PAGE_ALIGN(fbi->fb.fix.line_length * fbi->fb.var.yres + PAGE_SIZE) * fbi->buffer_num;
	}
	
	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.line_length * fbi->fb.var.yres + PAGE_SIZE) * fbi->buffer_num;
	fbi->map_cpu = (unsigned long)ov2_dma_alloc_writecombine(NULL, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL );
	
	if (!fbi->map_cpu) return -ENOMEM;

	fbi->fb.fix.smem_start = fbi->map_dma + PAGE_SIZE;

	pxa_create_dma(fbi, fbi->buffer_index);

	return 0;
}

#ifdef CONFIG_PXA27x_E25
/* set xpos, ypos, PPL and LP to 0 */
static int overlay2fb_YUV420_workaround(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	struct pxafb_dma_descriptor *dma; 
	u32 map_dma, map_cpu;
	int done, ret=0;

	map_cpu = (u32)ov2_dma_alloc_writecombine(NULL, PAGE_SIZE, &map_dma, GFP_KERNEL );
	
	if (!map_cpu) return -1;

	dma = (struct pxafb_dma_descriptor*)((map_cpu + PAGE_SIZE) - sizeof(struct pxafb_dma_descriptor));
	dma->fdadr = map_dma + PAGE_SIZE - sizeof(struct pxafb_dma_descriptor);
	dma->fsadr = map_dma;
	dma->fidr  = 0;
	dma->ldcmd = LDCMD_EOFINT | 128;

	/* step 2.a - enable overlay 2 with RGB mode
         *
         * - (xpos,ypos) = (0,0); 
	 * - 64 pixels, 16bpp 
	 */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);
	OVL2C2 = 0;
	OVL2C1 = OVL2C1_O2EN | 0x04 <<20 | (63);

	CLEAR_LCD_INTR(LCSR1, LCSR1_EOF2);
	if (fbi->state == C_DISABLE || fbi->state == C_BLANK)
		FDADR2 = dma->fdadr;
	else
		FBR2 = dma->fdadr | 0x1;

	/* step 2.b - run at least 1 frame with overlay 2 */
	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_EOF2, 100);
	if (!done) goto err;

	/* step 2.c - disable overlay 2 */
	OVL2C1 &= ~OVL2C1_O2EN;

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);
	FBR2 = 0x3;

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);
	if (!done) goto err;

	/* step 2.d - Wait for base EOF interrupts */
	CLEAR_LCD_INTR(LCSR0, LCSR_EOF);
	done = WAIT_FOR_LCD_INTR(LCSR0, LCSR_EOF, 100);

	goto out;
err:
	ret = -1;
out:
	/* free buffer allocated */
	ov2_dma_free_writecombine(NULL, PAGE_SIZE, (void*)map_cpu,  map_dma);

	fbi->state == C_DISABLE;
	return ret;
}

static void overlay2fb_unblank_workaroundYUV420(struct fb_info *info)
{
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	int format = fbi->format;
	
	if ( (format == 0x4) && (fbi->state == C_BLANK) )
		overlay2fb_YUV420_workaround(info);
}
#else
static void overlay2fb_unblank_workaroundYUV420(struct fb_info *info) {}
#endif

#ifdef CONFIG_PXA27x_OV2_FASTBUS_WORKAROUND
/* workaround for overlay2 fast bus issue */
static int fastbus_set = 0;

static void overlay2fb_set_fastbus( void )
{
	unsigned long flags;

	local_irq_save(flags);
	if ( fastbus_set ) {
		__asm__ __volatile__ ("\n\
                    mrc p14, 0, r0, c6, c0, 0    \n\
                    orr r0, r0, #0x8              \n\
                    mcr p14, 0, r0, c6, c0, 0    \n\
                    mrc p14, 0, r0, c6, c0, 0    \n\
                    mov r0, r0                   \n\
                    nop                          \n\
                    nop" : : : "r0" );
	}
	local_irq_restore(flags);
}

static void overlay2fb_clear_fastbus( void )
{
	unsigned int cclkcfg;
	unsigned int unused;
	unsigned int flags;

	local_irq_save(flags);
	__asm__ __volatile__("\
                mrc p14, 0, %0, c6, c0, 0          \n\
                nop                                \n\
                nop                                \n\
              "
	      : "=r" (cclkcfg));
	printk("cclkcfg = 0x%08x\n", cclkcfg);

	fastbus_set = 0;
	/* If fast bus bit is set, clear it and set a flag */
	if ( cclkcfg | 0x8 ) {
		cclkcfg &= ~(0x8);
		__asm__ __volatile__ ( "\n\
                      mcr p14, 0, %1, c6, c0, 0    \n\
                      nop                          \n\
                      nop                          \n\
                    " : "=&r" (unused) : "r" (cclkcfg) );

		fastbus_set = 1;
	}
	local_irq_restore(flags);
}
#endif

static int overlay2fb_enable(struct fb_info *info) 
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	unsigned long bpp2;
	unsigned int xres, yres;

	if (!fbi->refcount) return 0;
	if (!fbi->map_cpu) return -EINVAL;

	switch(fbi->fb.var.bits_per_pixel) {
	case 16:
		bpp2 = 0x4;
		break;
	case 18:
		bpp2 = 0x6;
		break;
	case 19:
		bpp2 = 0x8;
		break;
	case 24:
		bpp2 = 0x9;
		break;
	case 25:
		bpp2 = 0xa;
		break;
	default:
		return -EINVAL;
	}

        /* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);

	if (fbi->format == 0) {
		/* overlay2 RGB resolution, RGB and YUV have different xres value*/
		xres = fbi->fb.var.xres;
		yres = fbi->fb.var.yres;

		OVL2C2 = (fbi->format << 20) | (fbi->ypos << 10) | fbi->xpos;
		OVL2C1 = OVL2C1_O2EN | (bpp2 << 20) | ((yres-1)<<10) | (xres-1);
		/* setup RGB DMA */
		if (fbi->state == C_DISABLE || fbi->state == C_BLANK)
			FDADR2 = fbi->dma2->fdadr;
		else
			FBR2 = fbi->dma2->fdadr | 0x1;
	} else {
		/* overlay2 YUV resolution */
		xres = fbi->fb.fix.line_length;
		yres = fbi->fb.var.yres;

		OVL2C2 = (fbi->format << 20) | (fbi->ypos << 10) | fbi->xpos;
		OVL2C1 = OVL2C1_O2EN | (bpp2 << 20) | ((yres-1)<<10) | (xres-1);
#ifdef	CONFIG_PXA27x_E25
		/* FIXME PXA27x E25 */
		if (fbi->format == 4){
			/* FIXME */
			/* Wait util fifo emtpy */
			CLEAR_LCD_INTR(LCSR1, LCSR1_IU2);
			WAIT_FOR_LCD_INTR(LCSR1, LCSR1_IU2, 100);
		}
#endif
		if (fbi->state == C_DISABLE || fbi->state == C_BLANK) {
			FDADR2 = fbi->dma2->fdadr;
			FDADR3 = fbi->dma3->fdadr;
			FDADR4 = fbi->dma4->fdadr;
		} else {
			FBR2 = fbi->dma2->fdadr | 0x01;
			FBR3 = fbi->dma3->fdadr | 0x01;
			FBR4 = fbi->dma4->fdadr | 0x01;
		}
	}

#ifdef	CONFIG_PXA3xx
	/* Turn off fetching Base frame to save DMA bandwidth on PXA3xx */
	if (likely(OVL2C2 & (0x7 << 20))) { /* We suppose YUV is used mostly */
		if (fbi->basefb->fb.var.xres <= fbi->fb.var.xres &&
			fbi->basefb->fb.var.yres <= fbi->fb.var.yres) {
			LCCR6 = (1 << 31);
		}
	}
#endif

	fbi->state = C_ENABLE;
	return 0;
}

static int overlay2fb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	int done;

	if (fbi->state == C_DISABLE) 
		return 0;
	if (fbi->state == C_BLANK) {
		fbi->state = C_DISABLE;
		return 0;
	}

#ifdef	CONFIG_PXA3xx
	/* Turn off fetching Base frame to save DMA bandwidth on PXA3xx */
	if (likely(OVL2C2 & (0x7 << 20))) { /* We suppose YUV is used mostly */
		if (fbi->basefb->fb.var.xres <= fbi->fb.var.xres &&
			fbi->basefb->fb.var.yres <= fbi->fb.var.yres) {
			FDADR0 = fbi->basefb->fdadr0;
			LCCR6 = 0;
		}
	}
#endif

	fbi->state = C_DISABLE;

	/* clear O2EN */
	OVL2C1 &= ~OVL2C1_O2EN;

/* This sentence was lost in opt.patch. 
 * That make overlay2 can't disable/enable 
 * correctly sometimes.
 */
	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if(SMART_OVERLAY2 == TRUE){
		if (fbi->format == 0) 
			FDADR2 = 0;
		else {
			FDADR2 = 0;
			FDADR3 = 0;
			FDADR4 = 0;
		}
		
		goto OUT_1;
	}
    /* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);	

	if (fbi->format == 0) 
		FBR2 = FDADR2 | 0x3;
	else {
		FBR2 = FDADR2 | 0x3;
		FBR3 = FDADR3 | 0x3;
		FBR4 = FDADR4 | 0x3;
	}

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);

	if (!done) {
		pr_debug(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}

OUT_1:
	
	#else
    /* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);
	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);	

	if (fbi->format == 0) 
		FBR2 = FDADR2 | 0x3;
	else {
		FBR2 = FDADR2 | 0x3;
		FBR3 = FDADR3 | 0x3;
		FBR4 = FDADR4 | 0x3;
	}

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);

	if (!done) {
		pr_debug(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}
	#endif

	return 0;
}

static int overlay2fb_blank(int blank, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	int err=0;

	switch(blank) 
	{
	case 0:
		err = overlay2fb_enable(info);
		if (err) {
			fbi->state = C_DISABLE;

			#ifdef CONFIG_FB_PXA_SMART_PANEL
			if(smart_fbi != NULL && SMART_OVERLAY2 == TRUE){
				pxa_smartfb_set_ctrlr_state(smart_fbi, C_REENABLE);
			}
			else{
				set_ctrlr_state(fbi->basefb, C_REENABLE);
			}
			#else
			set_ctrlr_state(fbi->basefb, C_REENABLE);
			#endif
		}
		break;
	case 1:
		err = overlay2fb_disable(info);
		if (err) {
			fbi->state = C_DISABLE;
			
			#ifdef CONFIG_FB_PXA_SMART_PANEL
			if(smart_fbi != NULL && SMART_OVERLAY2 == TRUE){
				//extern void pxa_smartfb_set_ctrlr_state(struct lcd_smart_info *fbi, u_int state);
				//pxa_smartfb_set_ctrlr_state(smart_fbi, C_REENABLE);
			}
			else{
				set_ctrlr_state(fbi->basefb, C_REENABLE);
			}
			#else
			set_ctrlr_state(fbi->basefb, C_REENABLE);
			#endif
		}
		break;
	default:
		/* reserved */
		break;
	}
	
	return err;
}


static int overlay2fb_check_var( struct fb_var_screeninfo *var, struct fb_info *info)
{
	int xpos, ypos, xres, yres;
	int format;
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;

	xres=yres=0;

	xpos = (var->nonstd & 0x3ff);
	ypos = (var->nonstd >> 10) & 0x3ff;
	format = (var->nonstd >>20) & 0x7;


	/* Palnar YCbCr444, YCbCr422, YCbCr420 */
	if ( (format != 0x4) && (format != 0x3) && (format != 0x2) && (format !=0x0)) 
		return -EINVAL;

	/* dummy pixels */
	switch(format) {
	case 0x0: /* RGB */
		xres = var->xres;
		break;
	case 0x2: /* 444 */
		xres = (var->xres + 0x3) & ~(0x3);
		break;
	case 0x3: /* 422 */
		xres = (var->xres + 0x7) & ~(0x7);
		break;
	case 0x4: /* 420 */
		xres = (var->xres + 0xf) & ~(0xf);
		break;
	}
	yres = var->yres;

	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if(smart_fbi != NULL && SMART_OVERLAY2 == TRUE){
		if ( (xpos + xres) > smart_fbi->fb.var.xres ) 
			return -EINVAL;

		if ( (ypos + yres) > smart_fbi->fb.var.yres ) 
			return -EINVAL;
	}
	else{
		if ( (xpos + xres) > fbi->basefb->fb.var.xres ) 
			return -EINVAL;

		if ( (ypos + yres) > fbi->basefb->fb.var.yres ) 
			return -EINVAL;
	}
	#else
	if ( (xpos + xres) > fbi->basefb->fb.var.xres ) 
		return -EINVAL;

	if ( (ypos + yres) > fbi->basefb->fb.var.yres ) 
		return -EINVAL;
	#endif

	fbi->old_var=*var;

	var->activate=FB_ACTIVATE_NOW;

	return 0;

}

static int pxafb_validate_var(struct fb_var_screeninfo *var)
{
	int ret = -EINVAL;
   
	var->xres_virtual =
	    var->xres_virtual < var->xres ? var->xres : (var->xres_virtual
        /var->xres)*var->xres;
	var->yres_virtual =
	    var->yres_virtual < var->yres ? var->yres : (var->yres_virtual
        /var->yres)*var->yres;

	var->xoffset = 0;
	var->yoffset = (var->yoffset/var->yres)*var->yres;
	if (var->yoffset >= var->yres_virtual) {
		var->yoffset = var->yres_virtual - var->yres;
	}
	ret = 0;
	return ret;
}

/*
 * overlay2fb_set_var()
 *
 * var.nonstd is used as YCbCr format.
 * var.red/green/blue is used as (Y/Cb/Cr) vector 
 */

static int overlay2fb_set_par(struct fb_info *info)
{
	unsigned int xpos, ypos;
	int format, err;

	struct overlayfb_info *fbi=(struct overlayfb_info*)info;	
	struct fb_var_screeninfo *var = &fbi->fb.var;

	pxafb_validate_var(var);
	fbi->buffer_num = var->yres_virtual/var->yres;
	fbi->buffer_index = var->yoffset/var->yres;

	info->flags &= ~FBINFO_MISC_USEREVENT;

	if (fbi->state == C_BLANK)
		return 0;
	
	if (fbi->state == C_DISABLE)
		goto out1;

	if ( (var->xres == fbi->old_var.xres) &&
		(var->yres == fbi->old_var.yres) &&
		(var->bits_per_pixel == fbi->old_var.bits_per_pixel) &&
		(((var->nonstd>>20) & 0x7) == fbi->format) ) 
		goto out2;

out1:	
	xpos = var->nonstd & 0x3ff;
	ypos = (var->nonstd>>10) & 0x3ff;
	format = (var->nonstd>>20) & 0x7;


	fbi->format = format;
	if ( fbi->format==0 ) 
		err = overlay2fb_map_RGB_memory(info);
	else
		err = overlay2fb_map_YUV_memory(info);

	if (err) return err;

out2:
#ifdef	CONFIG_PXA27x_E25
	/* FIXME PXA27x E25 */
	if (C_ENABLE == fbi->state)
		overlay2fb_disable(info);
	
	if ((format == 0x4 ) &&
	    ((fbi->state == C_DISABLE) || (fbi->format != format)) )
		overlay2fb_YUV420_workaround(info);
#endif

	/* position */
	fbi->xpos = var->nonstd & 0x3ff;
	fbi->ypos = (var->nonstd>>10) & 0x3ff;

	overlay2fb_enable(info);

	return 0;
}

int overlay2fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned long flags;
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;
	
	DECLARE_WAITQUEUE(wait, current);

	/* Checking if LCD frame buffer is turn on? */
	#ifdef CONFIG_FB_PXA_SMART_PANEL
	if ((fbi->basefb->state == C_DISABLE) ||
	    (fbi->basefb->state == C_DISABLE_CLKCHANGE)) {
		if (smart_fbi != NULL) {
			if (smart_fbi->state != C_ENABLE)
				return -EAGAIN;
		} else
			return -EAGAIN;
	}
	#else
	if ((fbi->basefb->state == C_DISABLE) ||
	    (fbi->basefb->state == C_DISABLE_CLKCHANGE))
		return -EAGAIN;
	#endif
	
	down(&fbi->basefb->ctrlr_sem);
	
	LCCR5 &= ~LCCR5_BSM2;

	local_irq_save(flags);
	pxafb_validate_var(&(fbi->fb.var));
	fbi->buffer_index = fbi->fb.var.yoffset / fbi->fb.var.yres;

	pxa_create_dma(fbi, fbi->buffer_index);

	if (fbi->format == 0)
		FBR2 = fbi->dma2->fdadr | 0x3; 
	else {
		FBR2 = fbi->dma2->fdadr | 0x3; 
		FBR3 = fbi->dma3->fdadr | 0x1; 
		FBR4 = fbi->dma4->fdadr | 0x1; 
	}
		
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&fbi->basefb->ctrlr_wait, &wait);

	local_irq_restore(flags);
		
	schedule();
	remove_wait_queue(&fbi->basefb->ctrlr_wait, &wait);

	LCCR5 |= LCCR5_BSM2;

	up(&fbi->basefb->ctrlr_sem);

	return 0;
}

static struct fb_ops overlay2fb_ops = {
	.owner			= THIS_MODULE,
	.fb_open		= overlay2fb_open,
	.fb_release		= overlay2fb_release,
	.fb_check_var 		= overlay2fb_check_var,
	.fb_set_par		= overlay2fb_set_par,
	.fb_blank		= overlay2fb_blank,	
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_pan_display		= overlay2fb_pan_display,
};

/* Hardware cursor */

/* Bulverde Cursor Modes */
struct cursorfb_mode{
	int xres;
	int yres;
	int bpp;
};

static struct cursorfb_mode cursorfb_modes[]={
	{ 32,  32, 2},
	{ 32,  32, 2},
	{ 32,  32, 2},
	{ 64,  64, 2},
	{ 64,  64, 2},
	{ 64,  64, 2},
	{128, 128, 1},
	{128, 128, 1}
};

static int cursorfb_enable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;

	if (!fbi->map_cpu) return -EINVAL;

	CCR &= ~CCR_CEN;

	/* set palette format 
	 *
	 * FIXME: if only cursor uses palette
	 */
	LCCR4 = (LCCR4 & (~(0x3<<15))) | (0x1<<15);

	/* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM5 | LCCR5_BSM5 | LCCR5_EOFM5 | LCCR5_SOFM5);

	/* load palette and frame data */
	if (fbi->state == C_DISABLE) {
		FDADR5 = fbi->dma5_pal->fdadr;
		udelay(1);
		FDADR5 = fbi->dma5_frame->fdadr;
		udelay(1);

	}
	else {
		FBR5 = fbi->dma5_pal->fdadr | 0x1;
		udelay(1);
		FBR5 = fbi->dma5_frame->fdadr | 0x1;
		udelay(1);
	}

	CCR = CCR_CEN | (fbi->ypos << 15) | (fbi->xpos << 5) | (fbi->format);

	fbi->state = C_ENABLE;

	return 0;
}

static int cursorfb_disable(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*)info;
	int done, ret = 0;

	fbi->state = C_DISABLE;

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS5, 100);
	if (!done) ret = -1;

	CCR &= ~CCR_CEN;

	return ret;
}

static int cursorfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info *)info;
	u_int val, ret = 1;
	u_int *pal=(u_int*) fbi->palette_cpu;

	/* 25bit with Transparcy for 16bpp format */
	if (regno < fbi->palette_size) {
		val = ((trans << 24)  & 0x1000000);
		val |= ((red << 16)  & 0x0ff0000);
		val |= ((green << 8 ) & 0x000ff00);
		val |= ((blue << 0) & 0x00000ff);

		pal[regno] = val;
		ret = 0;
	}
	return ret;
}

int cursorfb_blank(int blank, struct fb_info *info)
{
	switch(blank) 
	{
	case 0:
		cursorfb_enable(info);
		break;
	case 1:
		cursorfb_disable(info);
		break;
	default:
		/* reserved */
		break;
	}
	return 0;
}

static int cursorfb_check_var( struct fb_var_screeninfo *var, struct fb_info *info)
{
	int xpos, ypos, xres, yres;
	int mode;
	struct cursorfb_mode *cursor;
	struct overlayfb_info *fbi=(struct overlayfb_info*)info;	

	mode = var->nonstd & 0x7;
	xpos = (var->nonstd>>5) & 0x3ff;
	ypos = (var->nonstd>>15) & 0x3ff;

	if (mode>7 || mode <0 ) 
		return -EINVAL;

	cursor = cursorfb_modes + mode;

	xres = cursor->xres;
	yres = cursor->yres;

	if ( (xpos + xres) > fbi->basefb->fb.var.xres ) 
		return -EINVAL;

	if ( (ypos + yres) > fbi->basefb->fb.var.yres ) 
		return -EINVAL;

	return 0;

}

static int cursorfb_set_par(struct fb_info *info)
{
	struct overlayfb_info *fbi = (struct overlayfb_info*) info;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	struct cursorfb_mode *cursor;
	int mode, xpos, ypos;
	int err;

	info->flags &= ~FBINFO_MISC_USEREVENT;

	mode = var->nonstd & 0x7;
	xpos = (var->nonstd>>5) & 0x3ff;
	ypos = (var->nonstd>>15) & 0x3ff;

	if (mode != fbi->format) {
		cursor = cursorfb_modes + mode;

		/* update "var" info */
		fbi->fb.var.xres = cursor->xres;
		fbi->fb.var.yres = cursor->yres;
		fbi->fb.var.bits_per_pixel = cursor->bpp;

		/* alloc video memory 
		 *
		 * 4k is engouh for 128x128x1 cursor, 
		 * - 2k for cursor pixels, 
		 * - 2k for palette data, plus 2 dma descriptor
		 */
		if (!fbi->map_cpu) {
			fbi->map_size = PAGE_SIZE;
			fbi->map_cpu = (unsigned long)dma_alloc_writecombine(NULL, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL );
 			if (!fbi->map_cpu) return -ENOMEM;
		}

		cursor = cursorfb_modes + mode;

		/* update overlay & fix "info" */
		fbi->screen_cpu 	= fbi->map_cpu;
		fbi->palette_cpu 	= fbi->map_cpu + (PAGE_SIZE/2);
		fbi->screen_dma  	= fbi->map_dma;
		fbi->palette_dma 	= fbi->map_dma + (PAGE_SIZE/2);

		fbi->format 		= mode; 
		fbi->palette_size 	= (1<<cursor->bpp) ;
		fbi->fb.fix.smem_start 	= fbi->screen_dma;
		fbi->fb.fix.smem_len 	= cursor->xres * cursor->yres * cursor->bpp / 8;
		fbi->fb.fix.line_length = cursor->xres * cursor->bpp / 8 ;

		fbi->dma5_pal     	= (struct pxafb_dma_descriptor*)(fbi->map_cpu + PAGE_SIZE - 16 );
		fbi->dma5_pal->fdadr 	= (fbi->map_dma + PAGE_SIZE - 16);
		fbi->dma5_pal->fsadr 	= fbi->palette_dma;
		fbi->dma5_pal->fidr  	= 0;
		fbi->dma5_pal->ldcmd 	= (fbi->palette_size<<2) | LDCMD_PAL;

		fbi->dma5_frame   		= (struct pxafb_dma_descriptor*)(fbi->map_cpu + PAGE_SIZE - 32 );
		fbi->dma5_frame->fdadr 	= (fbi->map_dma + PAGE_SIZE - 32);
		fbi->dma5_frame->fsadr 	= fbi->screen_dma;
		fbi->dma5_frame->fidr  	= 0;
		fbi->dma5_frame->ldcmd 	= fbi->fb.fix.smem_len;

		/* alloc & set default cmap */
		err = fb_alloc_cmap(&fbi->fb.cmap, fbi->palette_size, 0);
		if (err) return err;
		err = fb_set_cmap(&fbi->fb.cmap, info);
		if (err) return err;
	}

	/* update overlay info */
	if( (xpos != fbi->xpos) || (ypos != fbi->ypos) ) {
		fbi->xpos = xpos;
		fbi->ypos = ypos;
	}

	cursorfb_enable(info);
	set_ctrlr_state(fbi->basefb, C_REENABLE);

	return 0;
}

static struct fb_ops cursorfb_ops = {
	.owner			= THIS_MODULE,
	.fb_check_var		= cursorfb_check_var,		
	.fb_set_par		= cursorfb_set_par,
	.fb_blank		= cursorfb_blank,		
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_setcolreg		= cursorfb_setcolreg,
};

static struct overlayfb_info * __init overlay1fb_init_fbinfo(void)
{
	struct overlayfb_info *fbi;

	fbi = kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info) );

	fbi->refcount = 0;
	init_MUTEX(&fbi->mutex);

	strcpy(fbi->fb.fix.id, "overlay1");

	fbi->fb.fix.type		= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux		= 0;
	fbi->fb.fix.xpanstep		= 0;
	fbi->fb.fix.ypanstep		= 0;
	fbi->fb.fix.ywrapstep		= 0;
	fbi->fb.fix.accel		= FB_ACCEL_NONE;

	fbi->fb.var.nonstd		= 0;
	fbi->fb.var.activate		= FB_ACTIVATE_NOW;
	fbi->fb.var.height		= -1;
	fbi->fb.var.width		= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode		= FB_VMODE_NONINTERLACED;


	fbi->fb.fbops			= &overlay1fb_ops;
	fbi->fb.flags			= FBINFO_FLAG_DEFAULT;
	fbi->fb.node			= -1;
	fbi->fb.pseudo_palette		= NULL;

	fbi->xpos   			= 0;
	fbi->ypos   			= 0;
	fbi->format 			= -1;
	fbi->state 			= C_DISABLE;

	return fbi;
}

static struct overlayfb_info * __init overlay2fb_init_fbinfo(void)
{
	struct overlayfb_info *fbi;

	fbi = kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info) );

	fbi->refcount = 0;
	init_MUTEX(&fbi->mutex);

	strcpy(fbi->fb.fix.id, "overlay2");

	fbi->fb.fix.type		= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux		= 0;
	fbi->fb.fix.xpanstep		= 0;
	fbi->fb.fix.ypanstep		= 1;
	fbi->fb.fix.ywrapstep		= 0;
	fbi->fb.fix.accel		= FB_ACCEL_NONE;

	fbi->fb.var.nonstd		= 0;
	fbi->fb.var.activate		= FB_ACTIVATE_NOW;
	fbi->fb.var.height		= -1;
	fbi->fb.var.width		= -1;
	fbi->fb.var.accel_flags		= 0;
	fbi->fb.var.vmode		= FB_VMODE_NONINTERLACED;

	fbi->fb.fbops			= &overlay2fb_ops;
	fbi->fb.flags			= FBINFO_FLAG_DEFAULT;
	fbi->fb.node			= -1;
	fbi->fb.pseudo_palette		= NULL;

	fbi->xpos   			= 0;
	fbi->ypos   			= 0;
	fbi->format 			= -1;
	fbi->state 			= C_DISABLE;

	return fbi;
}

static struct overlayfb_info * __init cursorfb_init_fbinfo(void)
{
	struct overlayfb_info *fbi;

	fbi = kmalloc(sizeof(struct overlayfb_info) + sizeof(u16) * 16, GFP_KERNEL);
	if (!fbi)
		return NULL;

	memset(fbi, 0, sizeof(struct overlayfb_info) );

	fbi->refcount = 0;
	init_MUTEX(&fbi->mutex);

	strcpy(fbi->fb.fix.id, "cursor");

	fbi->fb.fix.type		= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux		= 0;
	fbi->fb.fix.xpanstep		= 0;
	fbi->fb.fix.ypanstep		= 0;
	fbi->fb.fix.ywrapstep		= 0;
	fbi->fb.fix.accel		= FB_ACCEL_NONE;

	fbi->fb.var.nonstd		= 0;
	fbi->fb.var.activate		= FB_ACTIVATE_NOW;
	fbi->fb.var.height		= -1;
	fbi->fb.var.width		= -1;
	fbi->fb.var.accel_flags		= 0;
	fbi->fb.var.vmode		= FB_VMODE_NONINTERLACED;

	fbi->fb.fbops			= &cursorfb_ops;
	fbi->fb.flags			= FBINFO_FLAG_DEFAULT;
	fbi->fb.node			= -1;
	fbi->fb.pseudo_palette		= NULL;


	fbi->xpos   			= 0;
	fbi->ypos   			= 0;
	fbi->format 			= -1;
	fbi->state 			= C_DISABLE;

	return fbi;
}


void pxa_set_overlay_ctrlr_state(struct pxafb_info *fbi, u_int state)
{
	switch (state) {
	case C_DISABLE:
			DISABLE_OVERLAYS(fbi);
			break;
	case C_ENABLE:
			ENABLE_OVERLAYS(fbi); 
			break;
	case C_BLANK:
			BLANK_OVERLAYS(fbi);
			break;
	case C_UNBLANK:
			UNBLANK_OVERLAYS(fbi);
			break;
	default:
			break;
	}
}

struct callback_data_t{
	char  *name;
	struct device *dev;
};


static int find_dev(struct device *dev, void *callback_data)
{
	int found=0;
	struct callback_data_t * data=(struct callback_data_t *) callback_data;
	
	found = (strncmp(dev->kobj.name, data->name, KOBJ_NAME_LEN) == 0);
	if(found == 1){
		data->dev = dev;
	}

	return found;
}
struct device*  find_bus_device(struct bus_type *bus, char *name)
{
	struct callback_data_t callback_data;
	
	callback_data.name = name;
	callback_data.dev = NULL;
	bus_for_each_dev(bus, NULL, &callback_data, find_dev);
	
	return callback_data.dev;
}

static int __devinit pxafb_overlay_init(void)
{
	int ret;
	struct overlayfb_info *overlay1fb, *overlay2fb, *cursorfb;
	struct pxafb_info *fbi;
	struct device *dev;
	
	ret = -1;
	overlay1fb = overlay2fb = cursorfb = NULL;
	fbi=NULL;

	dev=find_bus_device(&platform_bus_type, "pxa2xx-fb");
	if(dev ==NULL){	
		printk(KERN_INFO "Base framebuffer not exists, failed to load overlay driver!\n");
		return ret;
	}
	
	fbi = dev_get_drvdata(dev);
	if(fbi ==NULL ){
		printk(KERN_INFO "Base framebuffer not initialized, failed to load overlay driver!\n");		
		return ret;
	}


	/* Overlay 1 windows */
	overlay1fb = overlay1fb_init_fbinfo();

	if(!overlay1fb) {
		ret = -ENOMEM;
		printk("overlay1fb_init_fbinfo failed\n");
		goto failed;
	}


	ret = register_framebuffer(&overlay1fb->fb);
	if (ret<0) goto failed;
	
	/* Overlay 2 window */
	overlay2fb = overlay2fb_init_fbinfo();

	if(!overlay2fb) {
		ret = -ENOMEM;
		printk("overlay2fb_init_fbinfo failed\n");
		goto failed;
	}

	ret = register_framebuffer(&overlay2fb->fb);
	if (ret<0) goto failed;

	ov2_map_cpu = dma_alloc_writecombine(NULL, ov2_map_size, 
			&ov2_map_dma, GFP_KERNEL);

	if (!ov2_map_cpu)
		goto failed;

	/* Hardware cursor window */
	cursorfb = cursorfb_init_fbinfo();

	if(!cursorfb) {
		ret = -ENOMEM;
		printk("cursorfb_init_fbinfo failed\n");
		goto failed;
	}

	ret = register_framebuffer(&cursorfb->fb);
	if (ret<0) goto failed;


	/* set refernce to Overlays  */
	fbi->overlay1fb  = overlay1fb;
	fbi->overlay2fb  = overlay2fb;
	fbi->cursorfb    = cursorfb;
	fbi->set_overlay_ctrlr_state=pxa_set_overlay_ctrlr_state;	

	/* set refernce to BaseFrame */
	overlay1fb->basefb = fbi;
	overlay2fb->basefb = fbi;
	cursorfb->basefb = fbi;

	printk(KERN_INFO "Load PXA Overlay driver successfully!\n");
	
	return 0;

failed:
	if (overlay1fb) 
		kfree(overlay1fb);
	if (overlay2fb) 
		kfree(overlay2fb);
	if (cursorfb)
		kfree(cursorfb);
	if (ov2_map_cpu)
		dma_free_writecombine(NULL, ov2_map_size,
			(void *)ov2_map_cpu, ov2_map_dma);
	printk(KERN_INFO "Load PXA Overlay driver failed!\n");	
	return ret;
}

static void __exit pxafb_overlay_exit(void)
{
	struct pxafb_info *fbi;
	struct device *dev;

	dev=find_bus_device(&platform_bus_type, "pxa2xx-fb");
	if(dev ==NULL){	
		return ;
	}
	
	fbi = dev_get_drvdata(dev);
	if(fbi ==NULL ){
		return ;
	}

	if (fbi->overlay1fb) {
		unregister_framebuffer(&(fbi->overlay1fb->fb));
		kfree(fbi->overlay1fb);
		fbi->overlay1fb  = NULL;		
	}		
	
	if (fbi->overlay2fb) {
		unregister_framebuffer(&(fbi->overlay2fb->fb));
		kfree(fbi->overlay2fb);
		fbi->overlay2fb  = NULL;		
	}

	if (fbi->cursorfb) {
		unregister_framebuffer(&(fbi->cursorfb->fb));
		kfree(fbi->cursorfb);
		fbi->cursorfb  = NULL;		
	}
	
	fbi->set_overlay_ctrlr_state = NULL;

	printk(KERN_INFO "Unload PXA Overlay driver successfully!\n");	
	return ;
}

	
module_init(pxafb_overlay_init);
module_exit(pxafb_overlay_exit);

MODULE_DESCRIPTION("Loadable framebuffer overlay driver for PXA");
MODULE_LICENSE("GPL");

