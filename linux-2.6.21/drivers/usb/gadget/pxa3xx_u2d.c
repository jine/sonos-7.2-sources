/*
 * linux/drivers/usb/gadget/pxa3xx_u2d.c
 * PXA3xx on-chip high speed USB device controllers
 *
 * Copyright (C) 2002 Intrinsyc, Inc. (Frank Becker)
 * Copyright (C) 2003 Robert Schwebel, Pengutronix
 * Copyright (C) 2003 Benedikt Spranger, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003 Joshua Wise
 * Copyright (C) 2004 Intel Corporation
 * Copyright (C) 2006 Marvell International Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * change log:
 * 2006.2.14	xiang,jing	modify for gpio-expander long interrupt lantency
 * 2007.5.16	brown,mark	added support for otg carkit mode
 */

/*
#define	DEBUG
#define	VERBOSE	
#undef	DBG_NOISY
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/unaligned.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/uaccess.h>
#include <asm/arch/mfp.h>
#include <asm/arch/gpio.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb_gadget.h>
#include <linux/usb/otg.h>

#include <asm/arch/cpu-freq-voltage-pxa3xx.h>

/*
 * This driver handles the USB 2.0 Device Controller (U2D) in Marvell's
 * PXA 3xx series processors.
 * Such controller drivers work with a gadget driver.  The gadget driver
 * returns descriptors, implements configuration and data protocols used
 * by the host to interact with this device, and allocates endpoints to
 * the different protocol interfaces.  The controller driver virtualizes
 * usb hardware so that the gadget drivers will be more portable.
 *
 * This U2D hardware wants to implement a bit too much USB protocol, so
 * it constrains the sorts of USB configuration change events that work.
 * The errata for these chips are misleading; some "fixed" bugs from
 * pxa250 a0/a1 b0/b1/b2 sure act like they're still there.
 */

#ifdef CONFIG_USB_COMPOSITE
#include <linux/kernel.h>
#endif

#undef U2D_USE_ISRAM
#ifdef U2D_USE_ISRAM
#include <asm/arch/imm.h>
#endif

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
#include "../otg/pxa3xx_otg.h"
#endif

#define DMA_SYNC
#include "pxa3xx_u2d.h"
#include <asm/hardware.h>

#define	DRIVER_VERSION	"01-Jan-2006"
#define	DRIVER_DESC	"Monahans USB 2.0 Device Controller driver"

static const char driver_name [] = "pxa3xx_u2d";
static const char ep0name [] = "ep0";

#ifdef CONFIG_PROC_FS
#define	U2D_PROC_FILE
#endif

#undef PATCH_TEMP
#undef USB_TASKLET
#undef	USE_SP

#ifndef CONFIG_CPU_PXA310
#define U2D_SOFT_DISCON
#endif

#ifdef CONFIG_CPU_PXA310
#define XCVR_RESET MFP_ULPI_RESET
#define XCVR_RESET_ECO
#define USE_SPEOREN
#else
#define XCVR_RESET MFP_U2D_RESET
#undef USE_SPEOREN
#endif

#ifdef CONFIG_EMBEDDED
/* few strings, and little code to use them */
#undef	DEBUG
#undef	U2D_PROC_FILE
#endif

static int u2d_bugs = 0;
static int connected = 0;
static int d0cs = 0;
static int out_d0cs = 0;
#ifdef CONFIG_DVFM
#include <asm-arm/arch-pxa/ipmc.h>
static int u2d_dvfm_notifier(unsigned int cmd, void* client_data, void *info);
static int is_op_below_624Mhz(struct pxa3xx_fv_info * info);
static int is_current_624Mhz(void);
#else
static int is_op_below_624Mhz(int a);
static int is_current_624Mhz(void);
#endif

#ifdef U2D_SOFT_DISCON 
static int soft_dis_en = 0;
#endif
static int skip_ep_num = 0;
static int num_ep_used = 1;
static int check_fifo = 0;
static int xcvr_init = 1;
static struct pxa3xx_u2d memory;

static void stop_gadget(struct pxa3xx_u2d *dev);
static void u2d_stop(int state);
static void __u2d_disable(struct pxa3xx_u2d *dev);
static void __u2d_enable(struct pxa3xx_u2d *dev);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
extern void  pxa3xx_otg_require_bus(int status);
extern void  pxa3xx_otg_reset(void);
extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);
#endif

#define	DMASTR " (dma support)"

#ifdef	CONFIG_USB_PXA27X_SMALL
#define SIZE_STR	" (small)"
#else
#define SIZE_STR	""
#endif

#define U2DINT_EP0	 0x7
#define U2DINT_MASK	 (U2DINT_SPACKETCMP 	\
		| U2DINT_PACKETCMP | U2DINT_FIFOERR)
#define U2DISR_MASK	 (0xFFFFFF)

#if defined(CONFIG_CPU_PXA310) && defined(USE_SPEOREN)
#define U2DCR_MASK	 (U2DCR_UDE | U2DCR_SPEOREN)
#else
#define U2DCR_MASK	 (U2DCR_UDE)
#endif

#ifdef U2D_USE_ISRAM
static u32 zylu2d_immid = 0;
#endif

#ifdef CONFIG_CPU_PXA310
static enum u2d_phy_mode xcvr_mode;
#endif

#ifdef DEBUG
#define dmsg(format, args...) printk(KERN_DEBUG "%s: " format "\n", \
			__FUNCTION__, ##args)
#else
#define dmsg(format, args...) do {} while (0)
#endif

/*
 * u2d_bug_check
 * U2D_BUG_INMASS:	MHN_P_B0, MHN_P_B1, MHN_L_A0
 * U2D_BUG_SETINTF:	MHN_P_B0, MHN_P_B1
 * U2D_BUG_STALL:	MHN_P_B0, MHN_P_B1, MHN_L_A0
 * DDR_BUG_DMA:		MHN_P_B0, MHN_P_B1, MHN_L_A0
 * U2D_BUG_UTMID:	MHN_LV_A0 
 * 	Both the U2DCR[UDE] and U2DOTGCR[ULE] register bits are 
 * 	set to 0 when touching UTMID
 * U2D_FIX_ULPI_STP:	MHN_LV_A2 and above
 * 	MHN_LV_A2 fixed the bug that ULPI_STP would be asserted
 *	after system goes out of Low Power mode
 */

static int u2d_bug_check(void)
{
        unsigned int    cpuid;

        /* read CPU ID */
        __asm__ (
                "mrc p15, 0, %0, c0, c0, 0\n"
                : "=r" (cpuid)
        );

        /* It's not xscale chip. */
        if ((cpuid & 0xFFFF0000) != 0x69050000)
		return U2D_BUG_NONE;

        /* It's MhnP Bx */
        if ((cpuid & 0x0000FFF0) == 0x00006820) {
                if (((cpuid & 0x0F) == 4) || ((cpuid & 0x0F) == 5))
       			/* MhnP B0, B1 */
			return U2D_BUG_INMASS | U2D_BUG_SETINTF | \
			      U2D_BUG_STALL | DDR_BUG_DMA;
                else
       			/* MhnP B2 */
			return U2D_BUG_NONE;
        }

        /* It's MhnL Ax */
        if ((cpuid & 0x0000FFF0) == 0x00006880) {
                if ((cpuid & 0x0F) == 0)
       			/* MhnL A0 */
                	return U2D_BUG_INMASS | U2D_BUG_STALL | \
				DDR_BUG_DMA;
                else
       			/* MhnL A1 and above */
			return U2D_BUG_NONE;
        }

        /* It's MhnLV Ax */
        if ((cpuid & 0x0000FFF0) == 0x00006890) {
                if ((cpuid & 0x0F) == 0)
			return U2D_BUG_UTMID;
		else
			return U2D_FIX_ULPI_STP; 
	}

	return U2D_BUG_NONE;
}

#ifdef CONFIG_IPM
static void ipm_notify(void)
{
	if(u2d_bugs & DDR_BUG_DMA) {
		pr_debug("send OUT416 event to ipmd\n");
		ipm_event_notify(IPM_EVENT_DEVICE,
			IPM_EVENT_DEVICE_OUT416, NULL, 0);
	} else {
		pr_debug("send OUTD0CS event to ipmd\n");
		ipm_event_notify(IPM_EVENT_DEVICE,
			IPM_EVENT_DEVICE_OUTD0CS, NULL, 0);
	}
}

static int dvfm_out_status(struct pxa3xx_u2d *dev)
{
	int retval;

	if (d0cs) {
		ipm_notify();
		retval = wait_event_interruptible_timeout(dev->delay_wait,
				(!(ACSR & 0x04000000)), HZ/100);

		if((retval<=0) && (ACSR & 0x04000000)){
			printk(KERN_DEBUG "%s retval = %d\n", __func__, retval);
			return -EAGAIN;
		}
	}
	return 0;
}
#else
static void ipm_notify(void) {}
static int dvfm_out_status(struct pxa3xx_u2d *dev)
{
	return 0;
}
#endif

#ifdef DEBUG
static void dump_buffer(char *buf, unsigned length)
{
        char *c = buf;
        int i;

        pr_debug("%s, buffer:%p, total length:%d\n",
		__func__, buf, length);
        for (i = 0; i < length; i++) {
                if (0 == i % 10)
                        pr_debug("\n");
                pr_debug(" %2x", c[i]);
        }
        pr_debug("\n");
}

static void led_on(void)
{
	GPDR0 |= ~(1<<18);
	GPDR0 |= ~(1<<4);
	GSDR0 |= 1<<18;
	GSDR0 |= 1<<4;

	GPCR0 &= ~(1<<18);
	GPCR0 &= ~(1<<4);
	GPSR0 |= 1<<4;
	GPSR0 |= 1<<18;
}

static void led_off(void)
{
	GSDR0 |= 1<<18;
	GSDR0 |= 1<<4;
	GPDR0 |= 1<<18;
	GPDR0 |= 1<<4;

	GPSR0 &= ~(1<<18);
	GPSR0 &= ~(1<<4);
	GPCR0 |= 1<<18;
	GPCR0 |= 1<<4;
}
#endif

void u2d_clk_set(int enable)
{
	struct pxa3xx_u2d *dev = &memory;

	if (enable) {
		pxa_set_cken(CKEN_USB2, 1);
		dev->u2d_clk_dis = 0;
	} else {
		pxa_set_cken(CKEN_USB2, 0);
		dev->u2d_clk_dis = 1;
	}
}

void u2d_clk_enable(void)
{
	if (!(CKENA & (1 << CKEN_USB2))) {
		pxa_set_cken(CKEN_USB2, 1);
	}
}

void u2d_clk_restore(void)
{
	struct pxa3xx_u2d *dev = &memory;
	 
	if (dev->u2d_clk_dis)
		pxa_set_cken(CKEN_USB2, 0);
}

static void u2d_irq_set(int en)
{
	struct pxa3xx_u2d *dev = &memory;
	unsigned long flags;

	local_irq_save(flags);
	if (!dev->u2d_irq_dis && !en) {
		disable_irq(IRQ_USB2);
		dev->u2d_irq_dis = 1;
	}

	if (dev->u2d_irq_dis && en) {
		enable_irq(IRQ_USB2);
		dev->u2d_irq_dis = 0;
	}
	local_irq_restore(flags);
}
/* platform related functions */

#ifdef CONFIG_MACH_ZYLONITE
/*
 * reset the external transceiver
 */
void reset_xcvr(void)
{
	xcvr_init = 1;
	pxa3xx_mfp_set_afds(XCVR_RESET, 0, 0);
	pxa3xx_gpio_set_direction(XCVR_RESET, GPIO_DIR_OUT);

	pxa3xx_gpio_set_level(XCVR_RESET, GPIO_LEVEL_LOW);
	mdelay(100);
	pxa3xx_gpio_set_level(XCVR_RESET, GPIO_LEVEL_HIGH);
#ifdef XCVR_RESET_ECO
	pxa3xx_gpio_set_direction(XCVR_RESET, GPIO_DIR_IN);
#endif
}
#elif defined(CONFIG_MACH_LITTLETON)
extern int max7320_read(void);
extern int max7320_write(u8 value);
void reset_xcvr(void)
{
	u8 val;
	printk(KERN_INFO "%s: enter!\n", __func__);

	xcvr_init = 1;
	if (0 == platform_id()) {	/* Littleton 1.x */
		val = max7320_read();
		val &= ~0x02;
		max7320_write(val);

		mdelay(5);
		val |= 0x02;
		max7320_write(val);
	} else {	/* Littleton 4.x */
		pxa3xx_mfp_set_afds(XCVR_RESET, 0, 0);
		pxa3xx_gpio_set_direction(XCVR_RESET, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(XCVR_RESET, GPIO_LEVEL_LOW);
		msleep(10);
		pxa3xx_gpio_set_level(XCVR_RESET, GPIO_LEVEL_HIGH);
	}
}
#endif /* #ifdef CONFIG_MACH_ZYLONITE */

/* detect USB cable attach and detach by GPIO46
 * 1 -- cable attached; 0 -- cable detached
 */
static int is_cable_attached(void)
{
	struct pxa3xx_u2d *dev = &memory;
	int ret = 1;	/* default is connected */

	if (!dev->driver || dev->stp_gpio_irq_en) {
		ret = 0;
		goto out;
	}

#ifdef CABLE_DETECT_GPIO
	{
		unsigned int value;
		value = pxa3xx_gpio_get_level(MFP_USB2_DETECT);
		ret = ((value == GPIO_LEVEL_HIGH) && dev->driver);
	}

#else	/* without OTG, we also should use this register to check whether the cable is attached.*/
	ret = ((U2DOTGUSR & U2DOTGUSR_VV)!=0);
#endif
out:
	connected = ret;
	/* DMSG("%s, ret %d\n", __func__, ret); */
	return ret;
}

/* u2d soft disconnection */
static int u2d_soft_dis(int enable)
{

#ifdef U2D_SOFT_DISCON 
	if (enable) {
		if (soft_dis_en) {
			return 1;
		}
		soft_dis_en = 1;
		pxa3xx_mfp_set_afds(MFP_U2D_TERM_SELECT, MFP_AF0, MFP_DS03X);
		pxa3xx_gpio_set_direction(MFP_U2D_TERM_SELECT, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_U2D_TERM_SELECT, GPIO_LEVEL_LOW);
	} else {
		if (!soft_dis_en) {
			return 1;
		}
		soft_dis_en = 0;
		pxa3xx_mfp_set_afds(MFP_U2D_TERM_SELECT, MFP_U2D_TERM_SELECT_AF,
			MFP_DS08X);
	}
	return 0;
#else
	if (enable) {
		if (U2DCR & U2DCR_ADD) {
			return 1;
		}
		U2DCR |= U2DCR_ADD;
		mdelay(3);
	} else {
		if (!(U2DCR & U2DCR_ADD)) {
			return 1;
		}
		U2DCR &= ~U2DCR_ADD;
		mdelay(3);
	}
#endif
	DMSG("soft dis %s, U2DCR %x\n", enable?"enable":"disable", U2DCR);
	return 0;
}

#ifdef CABLE_DETECT_GPIO
irqreturn_t cable_detect_interrupt(int irq, void *_dev,	struct pt_regs *r)
#else
int cable_detect_interrupt(void)
#endif
{
	struct pxa3xx_u2d *dev = &memory;
	unsigned long flags;

	DMSG("%s , connected %d, d0cs %d\n", __func__,
		is_cable_attached(), d0cs);
	local_irq_save(flags);
	if (dev->driver && is_cable_attached()) {
		/* U2D module has resided in kernel */
		if (!is_current_624Mhz() || d0cs) {
			ipm_notify();
		}
		if(!d0cs) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			pxa3xx_otg_require_bus(USBOTG_VBUS_VALID);
#else
			__u2d_enable(&memory);
#ifdef U2D_SOFT_DISCON
			u2d_soft_dis(0);
#endif
#endif
		}
		u2d_stop(0);
	} else {
		if (!dev->driver)
			goto out;
		/* cable detached */
		if (!d0cs) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			pxa3xx_otg_require_bus(0);
#else
#ifdef U2D_SOFT_DISCON
			u2d_soft_dis(1);
#endif
			__u2d_disable(&memory);
#endif
		}
		u2d_stop(1);
	}

out:
	local_irq_restore(flags);
	return IRQ_HANDLED;
}

#ifdef CABLE_DETECT_GPIO
static void cable_detect_init(void)
{
	int ret;

	/* clear the MFPR */
	pxa3xx_mfp_set_afds(MFP_USB2_DETECT, 0, 0);
	pxa3xx_gpio_set_direction(MFP_USB2_DETECT, GPIO_DIR_IN);

	/* Enable GPIO edge rising & falling detection */
	set_irq_type(IRQ_GPIO(MFP2GPIO(MFP_USB2_DETECT)), IRQT_BOTHEDGE);

	/* request irq */
	ret = request_irq(IRQ_GPIO(MFP2GPIO(MFP_USB2_DETECT)),
			(void *)cable_detect_interrupt,
			IRQF_DISABLED, "U2D cable detect", NULL);
	if (ret) {
		printk(KERN_ERR "Request IRQ for GPIO failed, return :%d\n", ret);
	}

#ifdef DEBUG
	DMSG("\n%s GPLR1:0x%02x, GRER1:0x%02x \n", __FUNCTION__, GPLR1, GRER1);
#endif
}

static void cable_detect_deinit(void)
{
	free_irq(IRQ_GPIO(MFP2GPIO(MFP_USB2_DETECT)), NULL);
}

#elif	!defined(CONFIG_USB_OTG_PXA3xx_U2D)	/*CPU is LV && on OTG*/
static void ulpi_phy_init(void);
static void cable_detect_init(void)
{
	ulpi_phy_init();
	U2DOTGICR = U2DOTGINT_SI | U2DOTGINT_RVV | U2DOTGINT_FVV;
}

static void cable_detect_deinit(void)
{	
	U2DOTGICR &= ~(U2DOTGINT_SI | U2DOTGINT_RVV | U2DOTGINT_FVV);
}

#else
static void cable_detect_init(void) {}
static void cable_detect_deinit(void) {}
#endif

#ifdef USB_TASKLET
/* Tasklet */
#define	TQUEUE_SIZE	40
struct submit_queue{
	struct pxa3xx_ep* ep;
	struct pxa3xx_request* req;
	int status;
}task_queue[TQUEUE_SIZE];

int submit_head = 0;
int submit_tail = 0;

/* Add a task to global queue. This function must always succeed. */
void queue_u2d_task(struct pxa3xx_ep* ep,struct pxa3xx_request* req,int status)
{
	task_queue[submit_tail].ep = ep;
	task_queue[submit_tail].req = req;
	task_queue[submit_tail].status = status;
	submit_tail++;
	if(submit_tail>=TQUEUE_SIZE)submit_tail=0;
}

/* Remove a task from queue. Return zero if queue is empty. In this case parameter
 * node won't be changed. Return non-zero if queue is not empty and a node is
 * removed from queue successfully. In this case node will be filled with the one
 * that's dequeue.
 */
int dequeue_udc_task(struct submit_queue* node)
{
	/* do nothing if queue is empty */
	if(submit_head==submit_tail)return 0;

	node->ep = task_queue[submit_head].ep;
	node->req = task_queue[submit_head].req;
	node->status = task_queue[submit_head].status;

	submit_head++;
	if(submit_head>=TQUEUE_SIZE)submit_head=0;
	return 1;
}

void submit_udc_packets(unsigned long param)
{
	unsigned long flags;
	struct submit_queue node;
	while(1){
		local_irq_save(flags);
		if(!dequeue_udc_task(&node)){
			local_irq_restore(flags);
			return;
		}
		local_irq_restore(flags);
		node.req->req.complete(&node.ep->ep,&node.req->req);
	}
}

DECLARE_TASKLET(complete_req,submit_udc_packets,0);
#endif

/* ---------------------------------------------------------------------------
 * 	endpoint related parts of the api to the usb controller hardware,
 *	used by gadget driver; and the inner talker-to-hardware core.
 * ---------------------------------------------------------------------------
 */

static void pxa3xx_ep_fifo_flush (struct usb_ep *ep);
static void nuke (struct pxa3xx_ep *, int status);

static int pxa3xx_dma_desc_alloc(struct pxa3xx_ep *ep)
{
	if (ep->dma_desc_virt && ep->dma_desc_phys != -1)
		return 0;
#ifdef U2D_USE_ISRAM
	if ((ep->dma_desc_virt = imm_malloc(DMA_DESC_SIZE,
		IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, zylu2d_immid)) != NULL){
		ep->dma_desc_phys = imm_get_physical((void *)ep->dma_desc_virt, zylu2d_immid);
	} else {
		printk(KERN_ERR "can't malloc ISRAM by IMM successfully \n");
		return -ENOMEM;
	}
#else
	ep->dma_desc_virt = dma_alloc_coherent (ep->dev->dev, 
				DMA_DESC_SIZE, &ep->dma_desc_phys,
				GFP_ATOMIC);
#endif
	if (!ep->dma_desc_virt) {
		printk(KERN_ERR"%s: failed to allocate dma desc buf\n", __FUNCTION__);
		return -ENOMEM;
	}
	ep->dma_desc_size = DMA_DESC_SIZE;

	return 0;
}

static int pxa3xx_dma_desc_free(struct pxa3xx_ep *ep)
{
#ifdef U2D_USE_ISRAM
	if(ep->dma_desc_virt)
		imm_free((void *)ep->dma_desc_virt, zylu2d_immid);
#else
	if (irqs_disabled()) {
		local_irq_enable();
		dma_free_coherent(ep->dev->dev, ep->dma_desc_size, \
			ep->dma_desc_virt, ep->dma_desc_phys);
		local_irq_disable();
	}
	else {
		dma_free_coherent(ep->dev->dev, ep->dma_desc_size, \
			ep->dma_desc_virt, ep->dma_desc_phys);
	}
#endif
	ep->dma_desc_virt = NULL;
	ep->dma_desc_phys = ep->dma_desc_size = -1 ;
	ep->dma = -1;
	return 0;
}

static int pxa3xx_dma_buf_alloc(struct pxa3xx_ep *ep)
{
#ifndef DMA_SYNC
#ifdef U2D_USE_ISRAM
	if ((ep->dma_buf_virt = imm_malloc(DMA_BUF_SIZE,
		IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, zylu2d_immid)) != NULL) {
		ep->dma_buf_phys = imm_get_physical((void *)ep->dma_buf_virt, zylu2d_immid);
	} else {
		printk(KERN_ERR "can't malloc ISRAM by IMM successfully \n");
		return -ENOMEM;
	}
#else
	ep->dma_buf_virt = dma_alloc_coherent (ep->dev->dev, 
				DMA_BUF_SIZE, &ep->dma_buf_phys,
				GFP_ATOMIC);
#endif
	if (!ep->dma_buf_virt) {
		printk(KERN_ERR"%s: failed to allocate dma buf\n", __FUNCTION__);
		return -ENOMEM;
	}
#endif
	ep->dma_buf_size = DMA_BUF_SIZE;
	return 0;
}	

static int pxa3xx_dma_buf_free(struct pxa3xx_ep *ep)
{
#ifndef DMA_SYNC
#ifdef U2D_USE_ISRAM
	if(ep->dma_buf_virt)
		imm_free((void *)ep->dma_buf_virt, zylu2d_immid);
#else
	if (irqs_disabled()) {
		local_irq_enable();
		dma_free_coherent(ep->dev->dev, ep->dma_buf_size, \
			ep->dma_buf_virt, ep->dma_buf_phys);
		local_irq_disable();
	}
	else {
		dma_free_coherent(ep->dev->dev, ep->dma_buf_size, \
			ep->dma_buf_virt, ep->dma_buf_phys);
	}
#endif
#endif
	ep->dma_buf_virt = NULL;
	ep->dma_buf_phys = ep->dma_buf_size = -1 ;
	
	return 0;
}

static int get_mps(int speed, __u8 bmAttributes)
{
	switch (bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_CONTROL:
		return(EP0_MPS);
		break;
	case USB_ENDPOINT_XFER_ISOC:
		return(ISO_MPS(speed));
		break;
	case USB_ENDPOINT_XFER_BULK:
		return(BULK_MPS(speed));
		break;
	case USB_ENDPOINT_XFER_INT:
		return(INT_MPS(speed));
		break;
	default:
		return 0;
		break;
	}
}

static void change_mps(enum usb_device_speed speed)
{
	unsigned i;
	struct pxa3xx_ep *pxa_ep = NULL;
	struct pxa3xx_u2d *dev = the_controller;

	DMSG("%s, speed = %s\n", __FUNCTION__, (speed==USB_SPEED_HIGH)?"high":"full");
	/* find all validate EPs and change the MPS */
	for (i = 1; i < U2D_EP_NUM; i++) {
		if(dev->ep[i].assigned) {
			pxa_ep = &dev->ep[i];
			switch (pxa_ep->ep_type & USB_ENDPOINT_XFERTYPE_MASK) {
			case USB_ENDPOINT_XFER_CONTROL:
				pxa_ep->ep.maxpacket = EP0_MPS;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				pxa_ep->ep.maxpacket = ISO_MPS(speed);
				break;
			case USB_ENDPOINT_XFER_BULK:
				pxa_ep->ep.maxpacket = BULK_MPS(speed);
				break;
			case USB_ENDPOINT_XFER_INT:
				pxa_ep->ep.maxpacket = INT_MPS(speed);
				break;
			default:
				break;
			}
		}
	}
}

/*
 * endpoint enable/disable
 *
 * we need to verify the descriptors used to enable endpoints.  since pxa27x
 * endpoint configurations are fixed, and are pretty much always enabled,
 * there's not a lot to manage here.
 *
 * because pxa27x can't selectively initialize bulk (or interrupt) endpoints,
 * (resetting endpoint halt and toggle), SET_INTERFACE is unusable except
 * for a single interface (with only the default altsetting) and for gadget
 * drivers that don't halt endpoints (not reset by set_interface).  that also
 * means that if you use ISO, you must violate the USB spec rule that all
 * iso endpoints must be in non-default altsettings.
 */
static int pxa3xx_ep_enable (struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct pxa3xx_ep        *ep;
	struct pxa3xx_u2d       *dev;

	ep = container_of (_ep, struct pxa3xx_ep, ep);
	if (!_ep || !desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| ep->fifo_size < le16_to_cpu(desc->wMaxPacketSize)) {
		DMSG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if( ep->ep_type != USB_ENDPOINT_XFER_BULK
#ifdef PATCH_TEMP
			&& ep->ep_type != USB_ENDPOINT_XFER_ISOC
#endif
			&& desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DMSG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
				&& le16_to_cpu (desc->wMaxPacketSize)
						>= BULK_FIFO_SIZE)
			|| !desc->wMaxPacketSize) {
		DMSG("%s, bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver ||
		dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	ep->desc = desc;
	ep->dma = ep->ep_num;
	ep->dma_buf_virt = NULL;
	ep->stopped = 0;
	ep->pio_irqs = ep->dma_irqs = 0;
	ep->ep.maxpacket = get_mps(dev->gadget.speed, desc->bmAttributes);

	/* flush fifo (mostly for OUT buffers) */
	pxa3xx_ep_fifo_flush (_ep);

	/* ... reset halt state too, if we could ... */

	/* for (some) bulk and ISO endpoints, try to get a DMA channel and
	 * bind it to the endpoint.  otherwise use PIO.
	 */
	DMSG("%s: called attributes=%d\n", __FUNCTION__, ep->ep_type);
	switch (ep->ep_type) {
	case USB_ENDPOINT_XFER_ISOC:
	case USB_ENDPOINT_XFER_INT:
		/* FIXME, is it necessary be 4B align
		 * if (le16_to_cpu(desc->wMaxPacketSize) % 32)
		 * 	break;
		 * 	 fall through
		 */
	case USB_ENDPOINT_XFER_BULK:
		/* request DMA descriptor buffer */
		pxa3xx_dma_desc_alloc(ep);
		/* request DMA buffer */
		pxa3xx_dma_buf_alloc(ep);
	
		DMSG("%s using dma%d\n", _ep->name, ep->dma);

		break;
	default:
		break;
	}

	DBG(DBG_VERBOSE, "enabled %s\n", _ep->name);
	return 0;
}

static int pxa3xx_ep_disable (struct usb_ep *_ep)
{
	struct pxa3xx_ep	*ep;
	u2d_clk_enable();

	ep = container_of (_ep, struct pxa3xx_ep, ep);
	if (!_ep || !ep->desc) {
		DMSG("%s, %s not enabled\n", __FUNCTION__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	nuke (ep, -ESHUTDOWN);

	if ((ep->dma >= 0) && (ep->dma_buf_virt)){
		
		/* free DMA buffer */
		pxa3xx_dma_buf_free(ep);
	
		/* free DMA descriptor buffer */
		pxa3xx_dma_desc_free(ep);

	}

	/* flush fifo (mostly for IN buffers) */
	pxa3xx_ep_fifo_flush (_ep);
	u2d_clk_restore();

	ep->desc = 0;
	ep->stopped = 1;

	DBG(DBG_VERBOSE, "%s disabled\n", _ep->name);
	return 0;
}

/*-------------------------------------------------------------------------*/

/* for the pxa27x, these can just wrap kmalloc/kfree.  gadget drivers
 * must still pass correctly initialized endpoints, since other controller
 * drivers may care about how it's currently set up (dma issues etc).
 */

/*
 * 	pxa3xx_ep_alloc_request - allocate a request data structure
 */
static struct usb_request *
pxa3xx_ep_alloc_request (struct usb_ep *_ep, unsigned gfp_flags)
{
	struct pxa3xx_request *req;

	req = kzalloc (sizeof *req, gfp_flags);
	if (!req)
		return 0;

	INIT_LIST_HEAD (&req->queue);
	return &req->req;
}


/*
 * 	pxa3xx_ep_free_request - deallocate a request data structure
 */
static void
pxa3xx_ep_free_request (struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa3xx_request *req;

	req = container_of(_req, struct pxa3xx_request, req);
	WARN_ON (!list_empty (&req->queue));
	kfree(req);
}


/* PXA cache needs flushing with DMA I/O (it's dma-incoherent), but there's
 * no device-affinity and the heap works perfectly well for i/o buffers.
 * It wastes much less memory than dma_alloc_coherent() would, and even
 * prevents cacheline (32 bytes wide) sharing problems.
 */
static void *
pxa3xx_ep_alloc_buffer(struct usb_ep *_ep, unsigned bytes,
	dma_addr_t *dma, unsigned gfp_flags)
{
	char			*retval;

	retval = kmalloc (bytes, gfp_flags & ~(__GFP_DMA|__GFP_HIGHMEM));
	if (retval)
		*dma = __virt_to_bus((unsigned long)retval);
	return retval;
}

static void
pxa3xx_ep_free_buffer(struct usb_ep *_ep, void *buf, dma_addr_t dma,
		unsigned bytes)
{
	kfree (buf);
}

/*-------------------------------------------------------------------------*/

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct pxa3xx_ep *ep, struct pxa3xx_request *req, int status)
{
 	DMSG("%s is called\n", __FUNCTION__);
	list_del_init(&req->queue);
	if (likely (req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN){
  		DBG(DBG_VERBOSE, "complete %s req %p stat %d len %u/%u\n",
		 	ep->ep.name, &req->req, status,
		 	req->req.actual, req->req.length);
	}

	/* don't modify queue heads during completion callback */
#ifdef PATCH_TEMP
	req->req.complete(&ep->ep, &req->req);
	/*
	queue_u2d_task(ep,req,0);
	tasklet_schedule(&complete_req);
	*/
#else
#ifndef USB_TASKLET
	if(req->req.complete)
		req->req.complete(&ep->ep, &req->req);
#else
	queue_u2d_task(ep,req,0);
	tasklet_schedule(&complete_req);
#endif
#endif
}


static inline void ep0_idle (struct pxa3xx_u2d *dev)
{
	dev->ep0state = EP0_IDLE;
	LED_EP0_OFF;
}

static int
write_packet(volatile u32 *uddr, struct pxa3xx_request *req, unsigned max)
{
	u32		*buf;
	int	length, count, remain;

	buf = (u32*)(req->req.buf + req->req.actual);
	prefetch(buf);

	/* how big will this packet be? */
	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	remain = length & 0x3;
	count = length & ~(0x3);

	while (likely(count)) {
		*uddr = *buf++;
		count -= 4;
	}

	if (remain) {
		if(remain == 3) *uddr = *buf;
		else if(remain == 2){
			 *(volatile unsigned short *)uddr = *(unsigned short *)buf;
		}
		else{
			 *(volatile unsigned char *)uddr = *(unsigned char *)buf;
		}
	}

	return length;
}

/* caller asserts req->pending (ep0 irq status nyet cleared); starts
 * ep0 data stage.  these chips want very simple state transitions.
 */
static inline
void ep0start(struct pxa3xx_u2d *dev, u32 flags, const char *tag)
{
	U2DCSR0 = flags/*|U2DCSR0_SA|U2DCSR0_OPC*/;

	dev->req_pending = 0;
	DBG(DBG_VERY_NOISY, "%s %s, %02x/%02x\n",
		__FUNCTION__, tag, U2DCSR0, flags);
}

static int
write_ep0_fifo (struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	unsigned	count;
	int		is_short, adjusted = 0;

	count = write_packet(&U2DDR0, req, EP0_FIFO_SIZE);
	ep->dev->stats.write.bytes += count;

	/* last packet "must be" short (or a zlp) */
	is_short = (count != EP0_FIFO_SIZE);
	adjusted = ((count%4) == 3);
	if(adjusted) adjusted = U2DCSR0_IPA;
	else adjusted = 0;

	DBG(DBG_VERY_NOISY, "ep0in %d bytes %d left %p, is_short %d req_pending %d\n", count,
		req->req.length - req->req.actual, &req->req, is_short, ep->dev->req_pending);

	/* Note: don't access any U2D registers between load FIFO and set IPR bit.
		otherwise, it may cause transfer wrong number of bytes. */
	if (is_short) {
		if (ep->dev->req_pending)
			ep0start(ep->dev, U2DCSR0_IPR | adjusted, "short IN");
		else
			U2DCSR0 = U2DCSR0_IPR | adjusted;

		count = req->req.length;
		done (ep, req, 0);
		ep0_idle(ep->dev);
		return is_short;
	} else if (ep->dev->req_pending) {
		ep0start(ep->dev, U2DCSR0_IPR, "IN");
		return is_short;
	}

	ep0start(ep->dev, U2DCSR0_IPR, "IN");	/* xj */
	return is_short;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int
read_ep0_fifo (struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	u32		*buf, word;
	unsigned	bufferspace;

	buf = (u32*) (req->req.buf + req->req.actual);
	bufferspace = req->req.length - req->req.actual;

	while (U2DCSR0 & U2DCSR0_RNE) {
	/* FIXME
	 * if setup data is not multiple of 4, this routing will read some
	 * extra bytes
	 */
		word = U2DDR0;

		if (unlikely (bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DMSG("%s overflow\n", ep->ep.name);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = word;
			req->req.actual += 4;
			bufferspace -= 4;
		}
	}
	U2DCSR0 = U2DCSR0_OPC;

	DMSG("%s, req.actual %d req.length %d req.status %d, u2dcsr0 %x\n",
		__func__, req->req.actual, req->req.length, req->req.status, U2DCSR0);

	/* completion */
	if (req->req.actual >= req->req.length){
		req->req.actual = req->req.length;
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

#ifdef	DEBUG
void dma_desc_dump(int ep_num)
{
	struct pxa3xx_u2d	* dev = the_controller;
	struct pxa3xx_ep	*ep;
	pxa_dma_desc *desc;
	int i;

	ep = &dev->ep [ep_num];
	if(!ep->desc) {
		printk(KERN_DEBUG "%s, desc not initialized, return\n", __FUNCTION__);
	}

		desc = ep->dma_desc_virt;
		for(i=0;i<3;i++){
			printk(KERN_DEBUG "\tdadr:%x, dsdr:%x, dtdr:%x, dcmd:%x \n",
				(unsigned)desc->ddadr, (unsigned)desc->dsadr,
				(unsigned)desc->dtadr, (unsigned)desc->dcmd);
			desc++;
		}

}

void dma_info_dump(void)
{
	struct pxa3xx_u2d	* dev = the_controller;
	struct pxa3xx_ep	*ep;
	const struct usb_endpoint_descriptor	*d;
	pxa_dma_desc *desc;
	int i, j;

	for (i = 1; i < U2D_EP_NUM; i++) {
			ep = &dev->ep [i];
			d = ep->desc;
			if (!d)
				continue;

		/* DMA desc chain */
		printk(KERN_DEBUG "ep %d:\n"	\
			"\tdesc %p(%p) num %d(%dB) buf %p(%p) size %uB\n", i,
			(int *)ep->dma_desc_phys, ep->dma_desc_virt,
			ep->dma_desc_size/16, ep->dma_desc_size,
			(int *)ep->dma_buf_phys, ep->dma_buf_virt,
			ep->dma_buf_size);

		desc = ep->dma_desc_virt;
		for(j=0;j<DMA_DESC_NUM;j++){
			if(j<5)printk(KERN_DEBUG "\tdadr:%x, dsdr:%x, dtdr:%x, dcmd:%x \n",
					(unsigned)desc->ddadr, (unsigned)desc->dsadr,
					(unsigned)desc->dtadr, (unsigned)desc->dcmd);
			desc++;
			if(desc->ddadr & U2DMADADR_STOP)
				printk(KERN_DEBUG "\tlast\n\tdadr:%x, dsdr:%x, dtdr:%x, dcmd:%x \n",
					(unsigned)desc->ddadr, (unsigned)desc->dsadr,
					(unsigned)desc->dtadr, (unsigned)desc->dcmd);
		}
	}
}
#endif

static int config_dma_desc(struct pxa3xx_ep *ep, struct dma_txfr_t *txfr)
{
	pxa_dma_desc *desc = ep->dma_desc_virt;
	unsigned desc_addr, buf_addr = ep->dma_buf_phys, i;
	unsigned packet_size = ep->ep.maxpacket, direct = ep->dir_in;
	volatile unsigned dumy;

	desc_addr = ep->dma_desc_phys;

	for(i=0;i<DMA_DESC_NUM;i++)	{
		desc_addr += sizeof(pxa_dma_desc);

		desc->ddadr = desc_addr;
		desc->dcmd  = 0;

		if(direct) { /* IN */
			desc->dsadr = buf_addr;
			desc->dcmd |= U2DMACMDR_XFRDIS|U2DMACMDR_PACKCOMP;
		} else { /* OUT */
			desc->dtadr = buf_addr;
	}

		if(txfr->len>packet_size) {
			desc->ddadr &= ~U2DMADADR_STOP;
			desc->dcmd |= packet_size;
		} else {
			desc->dcmd |= txfr->len;
			desc->ddadr |= U2DMADADR_STOP;
			if(txfr->end_irq_en) {
				desc->dcmd |= U2DMACMDR_ENDIRQEN;
			}

			if(direct) { /* IN */
				if(txfr->len>0) {
					/* send short package */
					/* Desc->dcmd |= U2DMACMDR_PACKCOMP; */
				}else
					if(txfr->is_zero) {
						/* Send Zero package */
						desc->dcmd |= U2DMACMDR_PACKCOMP;
					}
			}
		}

		/* read back to make sure ddadr have been writed before start DMA */
		dumy = desc->dcmd;
		dumy = desc->ddadr;
		dumy = desc->dsadr;
		dumy = desc->dtadr;

		buf_addr += packet_size;
		desc++;

		if(txfr->len >= packet_size)
			txfr->len -= packet_size;
		else
			break;
	}

	(desc-1)->ddadr |= U2DMADADR_STOP;
	/* read back to make sure ddadr have been writed before start DMA */
	dumy = (desc-1)->ddadr;

	U2DMADADR(ep->ep_num) = ep->dma_desc_phys;

	return 0;
}

static int kick_dma(struct pxa3xx_ep *ep, struct pxa3xx_request *req)
{
	u32	len = req->req.length, csr = U2DCSR(ep->ep_num);
	char*	buf = (char*)req->req.buf;
	struct dma_txfr_t txfr;
	int count = 1000;

	/* check whether FIFO is empty */
	if(ep->dir_in && check_fifo){
		do{
			csr = U2DCSR(ep->ep_num);
			if(csr & U2DCSR_BE) break;
			count--;
		}while(count);

		if(count <=0) {
			DMSG("%s, IN fifo not empty!!!\n", __FUNCTION__);
			start_watchdog(ep->dev);
			return -ENOBUFS;
		}
	}

	buf += req->req.actual;
	len -= req->req.actual;
	ep->dma_con = 0;

	U2DMACSR(ep->ep_num) &= ~U2DMACSR_RUN;
	DMSG("%s: req:0x%p, buf:%p, length:%d, actual:%d dma:%d\n",
	 		__FUNCTION__, &req->req, req->req.buf, req->req.length,
	 		req->req.actual,ep->dma);

	if(!(U2DCSR(ep->ep_num) & U2DCSR_DME)){
		/* should be added to ep_config, FIXME */
		U2DCSR(ep->ep_num) = U2DCSR_DME;
	}

	if ( len > ep->dma_buf_size)
		ep->dma_con = 1;


	len = min(len, (u32)ep->dma_buf_size);
#ifndef DMA_SYNC
	if(ep->dir_in) {
		memcpy(ep->dma_buf_virt, buf, len);
	}
#else
	ep->dma_buf_virt = req->req.buf + req->req.actual;
	ep->dma_buf_phys = virt_to_phys(ep->dma_buf_virt);
#endif

	/* init the params for transfer */
	txfr.end_irq_en = 1;
	txfr.is_zero = (req->req.zero)?1:0;
	txfr.len = len;

	config_dma_desc(ep, &txfr);

	if(ep->dir_in) {
		U2DMACSR(ep->ep_num) |= U2DMACSR_RUN | U2DMACSR_STOPIRQEN;
	} else {
#ifndef CONFIG_CPU_PXA310
		/* in Full speed mode, FIFO overrun when DMA not ready */
		/* U2DICR |= U2DINT(ep->ep_num, U2DINT_FIFOERR); */
#else
		/* strange, have to write twice for ep no.8-15 */
		if(ep->ep_num >= U2D_EP_NUM/2)
			U2DMACSR(ep->ep_num) = U2DMACSR_RUN | U2DMACSR_STOPIRQEN | U2DMACSR_EORSTOPEN;
#endif
		U2DMACSR(ep->ep_num) = U2DMACSR_RUN | U2DMACSR_STOPIRQEN | U2DMACSR_EORSTOPEN;
	}

	return 0;
}

/*
 * disable the desc, clear DME for the ep
*/
static void cancel_dma(struct pxa3xx_ep *ep)
{
	struct pxa3xx_request	*req;
	u32			tmp;

	if (U2DMACSR(ep->dma) == 0 || list_empty(&ep->queue))
		return;

	DMSG("%s, dma:%d,dcsr:0x%x\n", __FUNCTION__, ep->dma, U2DMACSR(ep->dma));
	U2DMACSR(ep->dma) = 0;

	while ((U2DMACSR(ep->dma) & U2DMACSR_STOPINTR) == 0)
		cpu_relax();

	req = list_entry(ep->queue.next, struct pxa3xx_request, queue);
	tmp = U2DMACMDR(ep->dma) & U2DMACMDR_LEN;
	if(req)req->req.actual = req->req.length - tmp;

	/* the last tx packet may be incomplete, so flush the fifo.
	 * FIXME correct req.actual if we can
	 */
	U2DCSR(ep->ep_num) = U2DCSR_FEF | U2DCSR_TRN | U2DCSR_PC;

}

static void u2dma_handler(int dmach, void *_ep)
{
	struct pxa3xx_ep	*ep = _ep;
	struct pxa3xx_request	*req, *req_next;
	u32			dcsr, dcmd, dadr, completed, remained;
	unsigned length, desc_num;
	unsigned long		flags;

	local_irq_save(flags);

	req = list_entry(ep->queue.next, struct pxa3xx_request, queue);

	ep->dma_irqs++;
	ep->dev->stats.irqs++;
	HEX_DISPLAY(ep->dev->stats.irqs);

	completed = 0;
	remained = req->req.length - req->req.actual;

	dcsr = U2DMACSR(dmach);
	dcmd = U2DMACMDR(dmach);
	dadr = U2DMADADR(dmach);
	U2DMACSR(ep->dma) &= ~(U2DMACSR_RUN | U2DMACSR_STOPIRQEN);

	DMSG("%s, buf:0x%p ch:%d dcsr:%x dcmd:%x dadr:%x u2dcsr:%x dma_con%d\n",
		__FUNCTION__, req->req.buf, dmach, dcsr, dcmd, U2DMADADR(dmach),
		U2DCSR(dmach), ep->dma_con);
	if (dcsr & U2DMACSR_BUSERRINTR) {
		printk(KERN_ERR " Bus Error\n");
		DMSG("dcsr:%x, ddadr:%x, dsadr:%x, dtadr:%x, dcmd:%x\n",
				U2DMACSR(dmach), U2DMADADR(dmach), U2DMASADR(dmach),
				U2DMATADR(dmach), U2DMACMDR(dmach));
		U2DMACSR(dmach) = U2DMACSR_BUSERRINTR;
		req->req.status = -EIO;
		completed = 1;
	} else if(dcsr & U2DMACSR_STARTINTR){
		U2DMACSR(dmach) = U2DMACSR_STARTINTR;
		goto done;
	} else if ( (dcsr & U2DMACSR_STOPINTR)||(dcsr & U2DMACSR_ENDINTR)){
		U2DMACSR(dmach) = dcsr & (U2DMACSR_ENDINTR|U2DMACSR_EORINTR);
		if (ep->dir_in) {
			/* There are still packets to transfer */
			if ( ep->dma_con) {
				DMSG("dma_con%s: more packets,length:%d,actual:%d\n",
				 	 __FUNCTION__,req->req.length,
				 	 req->req.actual);
					req->req.actual += ep->dma_buf_size;
			} else  { /* It is whole package*/
				/* FIXME Sent a ZLP? */
				completed = 1;
				req->req.actual = req->req.length;
				DMSG("%s: req->req.zero=%d, req->req.length=%d\n",
					__FUNCTION__,req->req.zero, req->req.length);
			}
		}
		else {	/* OUT */
			if( ep->dma_con) {
#ifndef DMA_SYNC
#ifdef USE_SPEOREN
				memcpy((char*)req->req.buf + req->req.actual,\
					ep->dma_buf_virt, ep->dma_buf_size);

				req->req.actual += ep->dma_buf_size;
#else
				memcpy((char*)req->req.buf + req->req.actual,\
					ep->dma_buf_virt + req->req.actual%ep->dma_buf_size, \
					ep->ep.maxpacket);
	
				req->req.actual += ep->ep.maxpacket;
#endif

#else

#ifdef USE_SPEOREN
				req->req.actual += ep->dma_buf_size;
#else
				req->req.actual += ep->ep.maxpacket;
#endif

#endif

				if(dcsr & U2DMACSR_ENDINTR){
					goto irq_done;
				}

				U2DMADADR(ep->dma) = dadr;
				U2DMACSR(ep->dma) |= U2DMACSR_RUN | \
					U2DMACSR_STOPIRQEN | U2DMACSR_EORSTOPEN;

				goto done;
			} else { /* for out endpoints */
			u32		u2dcsr=0;

			u2dcsr = (U2DCSR_SST | U2DCSR_TRN) & U2DCSR(ep->ep_num);

			/* 11.5.6.6, before clear SST, stop DMA */
			if(u2dcsr & U2DCSR_SST) {
				cancel_dma(ep);
			}

#ifdef USE_SPEOREN
			if((dcsr & U2DMACSR_EORINTR) || (dcsr & U2DMACSR_ENDINTR)){
#else
			if(dcsr & U2DMACSR_EORINTR){
#endif
				if((U2DMACMDR(dmach) & U2DMACMDR_LEN) || (dcsr & U2DMACSR_ENDINTR)){

				/* caculate the length */
				desc_num = ((U2DMADADR(dmach)&0xfffffff0) - ep->dma_desc_phys)/16;
				length = ep->ep.maxpacket*desc_num - (dcmd & U2DMACMDR_LEN);

				completed = 1;
#ifndef DMA_SYNC
				memcpy((char*)req->req.buf + req->req.actual, \
						ep->dma_buf_virt, length);
#endif
				req->req.actual += length;

				DMSG("\tfully data received, len=%d, completed:%d\n",
					req->req.actual, completed);

				goto irq_done;
				}

				U2DMADADR(ep->dma) = dadr;
				U2DMACSR(ep->dma) |= U2DMACSR_RUN | U2DMACSR_STOPIRQEN | U2DMACSR_EORSTOPEN;
				DMSG("\t csr %x dcsr %x dadr %x dcmd %x\n", U2DCSR(ep->dma), U2DMACSR(ep->dma),
					U2DMADADR(ep->dma), U2DMACMDR(ep->dma));
			}

			goto done;
			}
		}
	} else
		DMSG("%s: Others dma:%d DCSR:0x%x DCMD:0x%x\n",
				__FUNCTION__, dmach, U2DMACSR(dmach), U2DMACMDR(dmach));

irq_done:
	if (likely(completed)) {
		if (req->queue.next != &ep->queue) {
			req_next = list_entry(req->queue.next,
					struct pxa3xx_request, queue);
			kick_dma(ep, req_next);
		}
		done(ep, req, 0);
	} else {
		kick_dma(ep, req);
	}

done:
	local_irq_restore(flags);
}

/*-------------------------------------------------------------------------*/

static int
pxa3xx_ep_queue(struct usb_ep *_ep, struct usb_request *_req, unsigned gfp_flags)
{
	struct pxa3xx_ep	*ep;
	struct pxa3xx_request	*req;
	struct pxa3xx_u2d	*dev;
	unsigned long		flags;
	int count = 1000;
	u32 csr;

	req = container_of(_req, struct pxa3xx_request, req);
	if (unlikely (!_req || !_req->complete || !_req->buf||
			!list_empty(&req->queue))) {
		DMSG("%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	DMSG("%s, ep point %d is queue\n", __FUNCTION__, ep->ep_num);

	dev = ep->dev;
	if (unlikely (!dev->driver || ((dev->ep0state != EP0_IN_FAKE)
			&& (dev->gadget.speed == USB_SPEED_UNKNOWN)))) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	/* iso is always one packet per request, that's the only way
	 * we can report per-packet status.  that also helps with dma.
	 */
	if (unlikely (ep->ep_type == USB_ENDPOINT_XFER_ISOC
			&& req->req.length > le16_to_cpu
						(ep->desc->wMaxPacketSize)))
		return -EMSGSIZE;

#if 1
	/*	check whether FIFO is empty
	*/
	if(ep->dir_in && check_fifo){
		do{
			csr = U2DCSR(ep->ep_num);
			if(csr & U2DCSR_BE) break;
			count--;
		}while(count);

		if(count <=0) {
			return -ENOBUFS;
		}
	}
#endif

#ifdef DMA_SYNC
	if (ep->desc != 0)
		consistent_sync(req->req.buf, req->req.length, 
			(ep->dir_in)? DMA_TO_DEVICE : DMA_FROM_DEVICE);
#endif

	DBG(DBG_NOISY, "%s queue req %p, len %d buf %p\n",
	     _ep->name, _req, _req->length, _req->buf);

	u2d_clk_enable();

	local_irq_save(flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

 	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->desc == 0 /* ep0 */) {
			unsigned	length = _req->length;

			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE:
				dev->stats.write.ops++;
				DMSG("%s, dev->req_config = %d\n", __FUNCTION__, dev->req_config);
				if (dev->req_config) {
#ifdef CONFIG_USB_COMPOSITE
					if(dev->req_config > 1) {
						dev->req_config--;
						done(ep, req, 0);
						req=0;
						break;
					}
#endif
					DMSG("ep0: set config finished, u2dcsr0 %x\n", U2DCSR0);

					dev->req_config = 0;
					ep0_idle(dev);
					done(ep, req, 0);
					req=0;
				} else if (write_ep0_fifo(ep, req))
					req = 0;
				break;

			case EP0_OUT_DATA_PHASE:
				dev->stats.read.ops++;
				if (dev->req_pending)
					ep0start(dev, 0, "OUT");

				if (length == 0 || ((U2DCSR0 & U2DCSR0_RNE) != 0
						&& read_ep0_fifo(ep, req))) {
					ep0_idle(dev);

					done(ep, req, 0);
					ep0start(dev,U2DCSR0_IPR | U2DCSR0_FTF,"zero IN"); /* RNDIS */
					req = 0;
				}
				break;
			case EP0_NO_ACTION:
				ep0_idle(dev);
				req=0;
				break;
			case EP0_IN_FAKE:
				DMSG("%s: in EP0_IN_FAKE\n", __FUNCTION__);
				dev->config_length = _req->length;
				if(((__u8*)(_req->buf))[1] == USB_DT_CONFIG){
					if(dev->gadget.speed==USB_SPEED_HIGH){
						memcpy(dev->active_gadget->config_desc_hs, _req->buf, _req->length);
					}else{
						memcpy(dev->active_gadget->config_desc, _req->buf, _req->length);
					}
				}else if(((__u8*)(_req->buf))[1] == USB_DT_DEVICE){
					memcpy(dev->active_gadget->device_desc, _req->buf, _req->length);
				}
				ep0_idle(dev);
				req->req.actual=req->req.length;
				done(ep, req, 0);
				req = 0;
				break;
			default:
				DMSG("ep0 i/o, odd state %d\n", dev->ep0state);
				local_irq_restore (flags);
				return -EL2HLT;
			}
		/* either start dma or prime pio pump */
		} else if (ep->dma >= 0) {
			if ((_req->length == 0) && ep->dir_in) { /* ZLP */
				U2DCSR(ep->ep_num)=U2DCSR_SP | U2DCSR_DME;
				done(ep,req,0);
				req =0;
			} else{
				kick_dma(ep, req);
			}
		/* can the FIFO can satisfy the request immediately? */
		}

		DMSG("req:%p,ep->desc:%p,ep->dma:%d\n", req, ep->desc, ep->dma);
		if (likely (req && ep->desc)){
			/* should be removed, only DMA related interrupt is enabled
			 * PC and SP are not handled just clear the interrupt
			 */
			if(ep->ep_num){
				/* U2DICR |= U2DINT(ep->ep_num, U2DINT_SPACKETCMP); */
			} else {
				U2DICR |= U2DINT(ep->ep_num, U2DINT_SPACKETCMP|	\
					U2DINT_PACKETCMP);
			}
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (likely (req))
		list_add_tail(&req->queue, &ep->queue);
	local_irq_restore(flags);

	u2d_clk_restore();
	return 0;
}


/*
 * 	nuke - dequeue ALL requests
 *  called by: stop_activity
 */
static void nuke(struct pxa3xx_ep *ep, int status)
{
	struct pxa3xx_request *req;

	/* called with irqs blocked */
	if (ep->dma > 0 && !ep->stopped)
		cancel_dma(ep);

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pxa3xx_request, queue);
		done(ep, req, status);
	}
	if (ep->desc && ep->ep_num) {
		U2DMACSR(ep->ep_num) &= ~U2DMACSR_RUN;
		U2DCSR(ep->ep_num) &= ~U2DCSR_DME;
#ifdef CONFIG_CPU_PXA310
		if(ep->ep_num >= U2D_EP_NUM/2)
			U2DICR2 &= ~(U2DINT(ep->ep_num, U2DINT_SPACKETCMP|U2DINT_PACKETCMP|U2DINT_FIFOERR));
		else
#else
			U2DICR &= ~(U2DINT(ep->ep_num, U2DINT_SPACKETCMP|U2DINT_PACKETCMP|U2DINT_FIFOERR));
#endif
		U2DMACSR(ep->ep_num) &= ~(U2DMACSR_STOPIRQEN|U2DMACSR_EORIRQEN|	\
			U2DMACSR_EORJMPEN|U2DMACSR_EORSTOPEN|U2DMACSR_RASIRQEN);
	}
}


/* dequeue JUST ONE request */
static int pxa3xx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa3xx_ep	*ep;
	struct pxa3xx_request	*req;
	unsigned long		flags;

	u2d_clk_enable();
	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	local_irq_save(flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		local_irq_restore(flags);
		return -EINVAL;
	}

	if (ep->dma >= 0 && ep->queue.next == &req->queue && !ep->stopped) {
		cancel_dma(ep);
		done(ep, req, -ECONNRESET);
		/* restart i/o */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					struct pxa3xx_request, queue);
			kick_dma(ep, req);
		}
	} else
		done(ep, req, -ECONNRESET);

	local_irq_restore(flags);
	u2d_clk_restore();
	return 0;
}

/*-------------------------------------------------------------------------*/

static int pxa3xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct pxa3xx_ep	*ep;
	unsigned long		flags;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (unlikely (!_ep
			|| (!ep->desc && ep->ep.name != ep0name))
			|| ep->ep_type == USB_ENDPOINT_XFER_ISOC) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}
	if (value == 0) {
		/* this path (reset toggle+halt) is needed to implement
		 * SET_INTERFACE on normal hardware.  but it can't be
		 * done from software on the PXA U2D, and the hardware
		 * forgets to do it as part of SET_INTERFACE automagic.
		 */
		DMSG("only host can clear %s halt\n", _ep->name);
		return -EROFS;
	}

	if(!list_empty(&ep->queue) && ep->dir_in) {
		DMSG("%s, -EAGAIN\n", __FUNCTION__);
		return(-EAGAIN);
	}

	u2d_clk_enable();
	local_irq_save(flags);

	/* ep0 needs special care, may not necessary for U2D */
	if (!ep->desc) {
		U2DCSR0 |= U2DCSR0_FTF;
		U2DCSR0 |= U2DCSR0_FST;
		/* start_watchdog(ep->dev);	disabled for U2D test */
		ep->dev->req_pending = 0;
		ep->dev->ep0state = EP0_STALL;
		LED_EP0_OFF;

 	/* and bulk/intr endpoints like dropping stalls too */
 	} else {
 		unsigned i;

		/* flush endpoint FIFO & force STALL */
		U2DCSR(ep->ep_num) = (U2DCSR(ep->ep_num) & (U2DCSR_DME|U2DCSR_FST))	\
			| U2DCSR_FEF | U2DCSR_FST;

 		for (i = 0; i < 1000; i += 20) {
 			if (U2DCSR(ep->ep_num) & U2DCSR_SST)
 				break;

 			udelay(20);
 		}
  	}
 	local_irq_restore(flags);

	u2d_clk_restore();
	DBG(DBG_VERBOSE, "%s halt\n", _ep->name);
	return 0;
}

static int pxa3xx_ep_fifo_status(struct usb_ep *_ep)
{
	struct pxa3xx_ep        *ep;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -ENODEV;
	}
	/* pxa can't report unclaimed bytes from IN fifos */
	if (ep->dir_in)
		return -EOPNOTSUPP;
	if (ep->dev->gadget.speed == USB_SPEED_UNKNOWN
			|| (U2DCSR(ep->ep_num) & U2DCSR_FS) == 0)
		return 0;
	else
		return (U2DBCR(ep->ep_num) & 0xfff) + 1;
}

static void pxa3xx_ep_fifo_flush(struct usb_ep *_ep)
{
	struct pxa3xx_ep        *ep;

	ep = container_of(_ep, struct pxa3xx_ep, ep);
	if (!_ep || ep->ep.name == ep0name || !list_empty(&ep->queue)) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return;
	}

	/* toggle and halt bits stay unchanged */

	/* most IN status is the same, but ISO can't stall */
	/**ep->reg_u2dcsr = UDCCSR_PC|UDCCSR_FST|UDCCSR_TRN
		| (ep->ep_type == USB_ENDPOINT_XFER_ISOC)
			? 0 : UDCCSR_SST;*/

	u2d_clk_enable();
	/* see above, any necessary ops for ISO eps */
	U2DCSR(ep->ep_num) = (U2DCSR(ep->ep_num) & (U2DCSR_DME|U2DCSR_FST))	\
		| U2DCSR_FEF | U2DCSR_TRN | U2DCSR_PC;
	u2d_clk_restore();

}


static struct usb_ep_ops pxa3xx_ep_ops = {
	.enable		= pxa3xx_ep_enable,
	.disable	= pxa3xx_ep_disable,

	.alloc_request	= pxa3xx_ep_alloc_request,
	.free_request	= pxa3xx_ep_free_request,

	.alloc_buffer	= pxa3xx_ep_alloc_buffer,
	.free_buffer	= pxa3xx_ep_free_buffer,

	.queue		= pxa3xx_ep_queue,
	.dequeue	= pxa3xx_ep_dequeue,

	.set_halt	= pxa3xx_ep_set_halt,
	.fifo_status	= pxa3xx_ep_fifo_status,
	.fifo_flush	= pxa3xx_ep_fifo_flush,
};


/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int pxa3xx_u2d_get_frame(struct usb_gadget *_gadget)
{
	return (U2DFNR & 0x7FF);
}

static int pxa3xx_u2d_wakeup(struct usb_gadget *_gadget)
{
	struct pxa3xx_u2d *dev;

	u2d_clk_enable();
	dev = container_of(_gadget, struct pxa3xx_u2d, gadget);
	/* if remote wakeup is not enabled, call SRP */
	if ((U2DCR & U2DCR_DWRE) == 0)  {
	} else
		U2DCR = (U2DCR & (U2DCR_MASK)) | U2DCR_UDR;
	u2d_clk_restore();
	return 0;
}

static int pxa3xx_u2d_vbus_session(struct usb_gadget *_gadget, int is_active)
{
        struct pxa3xx_u2d *dev;

	u2d_clk_enable();
        dev = container_of(_gadget, struct pxa3xx_u2d, gadget);
        if(is_active){
                __u2d_enable(dev);
        } else {
                __u2d_disable(dev);
        }
	u2d_clk_restore();
	return 0;
}

#ifdef CONFIG_CPU_PXA310
static int
pxa3xx_u2d_pullup (struct usb_gadget *gadget, int is_on)
{
	u2d_clk_enable();
	if(is_on) {
		U2DCR &= ~U2DCR_ADD;
	} else {
		U2DCR |= U2DCR_ADD;
	}
	u2d_clk_restore();
	return 0;
}
#endif

static const struct usb_gadget_ops pxa3xx_u2d_ops = {
	.get_frame	 = pxa3xx_u2d_get_frame,
	.wakeup		 = pxa3xx_u2d_wakeup,
	/* current versions must always be self-powered */
	.vbus_session	 = pxa3xx_u2d_vbus_session,
#ifdef CONFIG_CPU_PXA310
	.pullup		 = pxa3xx_u2d_pullup,
#endif
};

#ifdef DEBUG
static void gadget_info_dump(void);
#endif
/*-------------------------------------------------------------------------*/

#ifdef U2D_PROC_FILE



static const char proc_node_name [] = "driver/u2d";
static const char none [] = "none";

static int
u2d_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char			*buf = page;
	struct pxa3xx_u2d	*dev = _dev;
	char			*next = buf;
	unsigned		size = count;
	unsigned long		flags;
	int			i, t;
	u32			tmp;
#ifdef CONFIG_USB_COMPOSITE
	char		*name = (char *)none;
	if(dev->driver) name = (char *)dev->driver->driver.name;
#endif

	if (off != 0)
		return 0;

	local_irq_save(flags);

	if(!dev->driver || !(CKENA & (1 << CKEN_USB2))) {
		t = scnprintf(next, size, "no gadget driver,"
			" or clock is disabled\n");
		size -= t;
		next += t;
		goto done;
	}

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n"
		"%s version: %s\nGadget driver: %s, speed: %s\n",
		driver_name, DRIVER_VERSION SIZE_STR DMASTR,
#ifndef CONFIG_USB_COMPOSITE
		dev->driver ? dev->driver->driver.name : "(none)",
#else
		(dev->driver_count > 1)? "(composite)" : name,
#endif
		(dev->gadget.speed==USB_SPEED_HIGH)?"high":"full");

	size -= t;
	next += t;

	/* registers for device and ep0 */
	t = scnprintf(next, size,
#ifndef CONFIG_CPU_PXA310
	        "uicr %02X, uisr %02X, ufnr %02X\n",
	                U2DICR, U2DISR, U2DFNR);
#else
                "uicr %02X uicr2 %02X, uisr %02X uisr2 %02X, ufnr %02X\n",
	                U2DICR, U2DICR2, U2DISR, U2DISR2, U2DFNR);
#endif
	size -= t;
	next += t;

	tmp = U2DCR;
#ifdef CONFIG_CPU_PXA310
        t = scnprintf(next, size,"u2dcr %02X =%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s, con=%d,inter=%d,altinter=%d\n", tmp,
                (tmp & U2DCR_NDC) ? " ndc":"",
#else
	t = scnprintf(next, size,"u2dcr %02X =%s%s%s%s%s%s%s%s%s%s%s%s%s, con=%d,inter=%d,altinter=%d\n", tmp,
#endif
                (tmp & U2DCR_HSTC_MASK) ? " hstc":"",
#ifdef CONFIG_CPU_PXA310
                (tmp & U2DCR_SPEOREN) ? " speoren":"",
#endif
                (tmp & U2DCR_FSTC_MASK) ? " fstc":"",
                (tmp & U2DCR_UCLKOVR) ? " uclkovr" : "",
                (tmp & U2DCR_ABP) ? " abp" : "",
                (tmp & U2DCR_ADD) ? " add" : "",
		(tmp & U2DCR_CC) ? " cc" : "",
		(tmp & U2DCR_HS) ? " hs" : "",
		(tmp & U2DCR_DWRE) ? " dwre" : "",
		(tmp & U2DCR_SMAC) ? " smac" : "",
		(tmp & U2DCR_EMCE) ? " emce" : "",
		(tmp & U2DCR_UDR) ? " udr" : "",
		(tmp & U2DCR_UDA) ? " uda" : "",
		(tmp & U2DCR_UDE) ? " ude" : "",
		(tmp & U2DCR_ACN) >> U2DCR_ACN_S,
		(tmp & U2DCR_AIN) >> U2DCR_AIN_S,
		(tmp & U2DCR_AAISN)>> U2DCR_AAISN_S );

	size -= t;
	next += t;

#ifdef CONFIG_CPU_PXA310
	tmp = U2DOTGCR;
	t = scnprintf(next, size,"u2dotgcr %02X =%s%s%s%s%s%s%s%s%s%s, xcvr_mode %d, ulpi_dat3_irq_en %d\n", tmp,
		(tmp & U2DOTGCR_OTGEN) ? " otgen":"",
		(tmp & U2DOTGCR_AALTHNP) ? " aalthnp":"",
		(tmp & U2DOTGCR_AHNP) ? " ahnp":"",
		(tmp & U2DOTGCR_BHNP) ? " bhnp":"",
		(tmp & U2DOTGCR_CKAF) ? " ckaf":"",
		(tmp & U2DOTGCR_UTMID) ? " utmid":"",
		(tmp & U2DOTGCR_ULAF) ? " ulaf":"",
		(tmp & U2DOTGCR_SMAF) ? " smaf":"",
		(tmp & U2DOTGCR_RTSM) ? " rtsm":"",
		(tmp & U2DOTGCR_ULE) ? " ule":"",
		xcvr_mode, dev->stp_gpio_irq_en);

	size -= t;
	next += t;

	/* registers for device and ep0 */
	t = scnprintf(next, size,
		"otgicr %08X otgisr %08X otgucr %08X otgusr %08X p3cr %08X\n",
		U2DOTGICR, U2DOTGISR, U2DOTGUCR, U2DOTGUSR, U2DP3CR);
	size -= t;
	next += t;

#endif

	tmp = U2DCSR0;
	t = scnprintf(next, size,
		"u2dcsr0 %02X =%s%s%s%s%s%s%s%s%s\n", tmp,
		(tmp & U2DCSR0_IPA) ? " ipa" : "",
		(tmp & U2DCSR0_SA) ? " sa" : "",
		(tmp & U2DCSR0_RNE) ? " rne" : "",
		(tmp & U2DCSR0_FST) ? " fst" : "",
		(tmp & U2DCSR0_SST) ? " sst" : "",
		(tmp & U2DCSR0_DME) ? " dme" : "",
		(tmp & U2DCSR0_FTF) ? " ftf" : "",
		(tmp & U2DCSR0_IPR) ? " ipr" : "",
		(tmp & U2DCSR0_OPC) ? " opc" : "");
	size -= t;
	next += t;

	if (!dev->driver)
		goto done;

	t = scnprintf(next, size, "ep0 IN %lu/%lu, OUT %lu/%lu\nirqs %lu\n\n",
		dev->stats.write.bytes, dev->stats.write.ops,
		dev->stats.read.bytes, dev->stats.read.ops,
		dev->stats.irqs);
	size -= t;
	next += t;

	/* dump endpoint queues */
	for (i = 0; i < U2D_EP_NUM; i++) {
		struct pxa3xx_ep	*ep = &dev->ep [i];
		struct pxa3xx_request	*req;
		int			t;

		if (i != 0) {
			const struct usb_endpoint_descriptor	*d;

			d = ep->desc;
			if (!d)
				continue;
			tmp = U2DCSR(ep->ep_num);
			t = scnprintf(next, size,
				"%s max %d %s u2dcsr %02x u2dcr:0x%x, u2denr:0x%x, intf=%d(%d)\n",
				ep->ep.name, le16_to_cpu (d->wMaxPacketSize),
				(ep->dma >= 0) ? "dma" : "pio", tmp,
				U2DEPCR(ep->ep_num), U2DEN(ep->ep_num),
				dev->ep[i].assigned_interface, dev->ep[i].interface);

			/* TODO translate all five groups of u2dcs bits! */

		} else /* ep0 should only have one transfer queued */
			t = scnprintf(next, size, "ep0 max 64 pio irqs %lu, u2den0:0x%x\n",
				ep->pio_irqs, U2DEN0);
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		/* DMA desc chain */
		t = scnprintf(next, size,
			"\tu2dmacsr %x desc %p(%p) num %d(%dB) buf %p(%p) size %uB\n", U2DMACSR(ep->dma),
			(int *)ep->dma_desc_phys, ep->dma_desc_virt,
			ep->dma_desc_size/16, ep->dma_desc_size,
			(int *)ep->dma_buf_phys, ep->dma_buf_virt,
			ep->dma_buf_size);
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		/* req queue */
		if (list_empty(&ep->queue)) {
			t = scnprintf(next, size, "\t(nothing queued)\n");
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
			continue;
		}
		list_for_each_entry(req, &ep->queue, queue) {
			if (ep->dma >= 0 && req->queue.prev == &ep->queue)
				t = scnprintf(next, size,
					"\treq %p len %d/%d "
					"buf %p \n "
					"\t(dma%d csr:%08x cmd:%08x da:%08x sa:%08x ta:%08x)\n",
					&req->req, req->req.actual,
					req->req.length, req->req.buf,
					ep->dma, U2DMACSR(ep->dma), U2DMACMDR(ep->dma),
					U2DMADADR(ep->dma), U2DMASADR(ep->dma), U2DMATADR(ep->dma)
					/* low 13 bits == bytes-to-go */
					);
			else
				t = scnprintf(next, size,
					"\treq %p len %d/%d buf %p\n",
					&req->req, req->req.actual,
					req->req.length, req->req.buf);
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
		}
	}

	t = scnprintf(next, size, "vbus level %x\n",
		is_cable_attached());
	size -= t;
	next += t;

done:
	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static int u2d_proc_write(struct file *filp, const char *buffer,
		unsigned long count, void *data)
{
	char kbuf[8];
	int index;
	struct pxa3xx_u2d *dev = the_controller;

	if (count >=8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	if(!dev->driver) {
		printk(KERN_ERR "no gadget driver, or clock is disabled\n");
		return -EINVAL;
	}

	switch(index) {
	default:
		return -EINVAL;
	}
	return count;
}

#ifndef CONFIG_PROC_FS
#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev)
#else
#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry(proc_node_name, 0, NULL);\
		if (ent) { \
			ent->data = dev; \
			ent->read_proc = u2d_proc_read; \
			ent->write_proc = u2d_proc_write; \
		} \
	}while(0);
#endif
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else	/* !U2D_PROC_FILE */
#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif	/* U2D_PROC_FILE */

/* "function" sysfs attribute */
static ssize_t
show_function (struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct pxa3xx_u2d	*dev = dev_get_drvdata (_dev);

	if (!dev->driver
			|| !dev->driver->function
			|| strlen (dev->driver->function) > PAGE_SIZE)
		return 0;
	return scnprintf (buf, PAGE_SIZE, "%s\n", dev->driver->function);
}
static DEVICE_ATTR (function, S_IRUGO, show_function, NULL);

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_CPU_PXA310 

#define MFP_ULPI_INT		MFP_ULPI_DATAOUT_3
#define MFP_ULPI_INT_AF 	MFP_ULPI_DATAOUT_3_AF
#define MFP_ULPI_INT_GPIO	MFP_PIN_GPIO33_AF_GPIO_33

static void ulpi_dat3_work(void)
{
	struct pxa3xx_u2d	*dev = the_controller;
	unsigned long flags;
	u32 u2dotgcr;

	//if (d0cs || !dev->ulpi_dat3_work) {
	if (!dev->ulpi_dat3_work) {
		return;
	}
	local_irq_save(flags);
	/* enable u2d function */
	u2d_clk_set(1);

	pxa3xx_mfp_set_afds(MFP_ULPI_INT, MFP_ULPI_INT_AF, MFP_DS08X);

	enable_oscc_pout();

	u2dotgcr = U2DOTGCR;
	u2dotgcr |= U2DOTGCR_ULE;
        U2DOTGCR = u2dotgcr;
	u2dotgcr = U2DOTGCR;

	DMSG("%s\n", __func__);
	u2d_irq_set(1);
	
	dev->ulpi_dat3_work = 0;
	local_irq_restore(flags);
}

static irqreturn_t ulpi_dat3_irq(int irq, void *devid)
{
	struct pxa3xx_u2d *dev = (struct pxa3xx_u2d *)devid;

	disable_irq(irq);
	dev->stp_gpio_irq_en = 0;
	dev->ulpi_dat3_work = 1;

	if (d0cs) {
		ipm_notify();
	}

	ulpi_dat3_work();
	/*INIT_DELAYED_WORK(&dev->stp_gpio_work, 
		(void (*)(void *))ulpi_dat3_work);
       	queue_delayed_work(dev->stp_work_queue, 
		&dev->stp_gpio_work, 20); */
	/*INIT_WORK(&dev->stp_gpio_work,
        	(void (*)(void *))ulpi_dat3_work);
        queue_work(dev->stp_work_queue, &dev->stp_gpio_work);*/

	DMSG("%s end\n", __func__);

	return IRQ_HANDLED;
}

static int ulpi_dat3_int_set(int enable)
{ 
	struct pxa3xx_u2d	*dev = the_controller;
	int irq = IRQ_GPIO(MFP2GPIO(MFP_ULPI_INT));
	unsigned long flags;
	u32 u2dotgcr;

	local_irq_save(flags);
	if (enable) {
		if (dev->stp_gpio_irq_en) {
			printk(KERN_ERR "re-enterance of %s\n", __func__);
			goto done;
		}
		
		pxa3xx_mfp_set_afds(MFP_ULPI_INT, MFP_ULPI_INT_GPIO, 0);
		pxa3xx_gpio_set_direction(MFP_ULPI_INT, GPIO_DIR_IN);	
		set_irq_type(irq, IRQT_RISING);

		enable_irq(irq);
		dev->stp_gpio_irq_en = 1;

		u2d_irq_set(0);
        	u2dotgcr = U2DOTGCR;
		u2dotgcr &= ~(U2DOTGCR_ULAF | U2DOTGCR_ULE);
        	U2DOTGCR = u2dotgcr;
		u2dotgcr = U2DOTGCR;

		u2d_clk_set(0);
	} else {
		if (!dev->stp_gpio_irq_en)
			goto done;
		disable_irq(irq);
		dev->ulpi_dat3_work = 1;
		ulpi_dat3_work();
		dev->stp_gpio_irq_en = 0;
	}
	DMSG("%s %d, orig %d\n", __func__, enable, dev->stp_gpio_irq_en);
	local_irq_restore(flags);

done:
	return 0;
}

int ulpi_rtsm(void)
{
        u32 u2dotgcr = U2DOTGCR;
        int count=100000;

	DMSG("%s, U2DOTGCR %x U2DOTGUSR %x\n", __func__, U2DOTGCR, U2DOTGUSR);
	u2dotgcr |= U2DOTGCR_UTMID;
	if (U2DOTGUSR & 0xf0000000) {
	
        /* switch to SYNCH mode first */
	u2dotgcr = U2DOTGCR;
	u2dotgcr |= U2DOTGCR_UTMID;
        u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF | U2DOTGCR_ULAF);
        U2DOTGCR = u2dotgcr;
	
	u2dotgcr = U2DOTGCR;
	u2dotgcr |= U2DOTGCR_RTSM;
        U2DOTGCR = u2dotgcr;
	
        u2dotgcr = U2DOTGCR;
        u2dotgcr |= U2DOTGCR_ULAF;
        u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF);
        U2DOTGCR = u2dotgcr;
	u2dotgcr = U2DOTGCR;

	}

        while((U2DOTGUSR & 0xf0000000) && (U2DOTGCR & U2DOTGCR_RTSM) && count) count--;

        if(count <=0) {
                printk(KERN_ALERT "%s time out, reset_xcvr!!! USR %x\n",
                        __func__, U2DOTGUSR);
                reset_xcvr();
        }
        xcvr_mode = PRE_SYNCH;
	DMSG("%s end, U2DOTGCR %x U2DOTGUSR %x\n", __func__, U2DOTGCR, U2DOTGUSR);

        return 0;
}

enum u2d_phy_mode ulpi_get_phymode(void)
{
	enum u2d_phy_mode state;

	ulpi_dat3_int_set(0);

	state = (U2DOTGUSR & 0xF0000000)>>28;
	/* in case when set UDE it would enter LOWPOWER mode automatically
	 * if no SOFs longer than 3ms */
	if(state == LOWPOWER)
		xcvr_mode = LOWPOWER;

	if((state != xcvr_mode) && (xcvr_mode != PRE_SYNCH)) {
		printk(KERN_DEBUG "ULPI mode %d not aligned, should be %d",
			state, xcvr_mode);
		xcvr_mode = state;
		if (state) 
			ulpi_rtsm();
	}

	return xcvr_mode;
}

int ulpi_reg_read(u8 reg, u8 *value)
{	int i=10000;
	enum u2d_phy_mode state = ulpi_get_phymode();

	if((state != SYNCH) && (state != PRE_SYNCH)) {
		dmsg(" not in SYNCH mode!!!");
		return -1;
	}

	U2DOTGUCR = U2DOTGUCR_RUN|U2DOTGUCR_RNW|(reg<<U2DOTGUCR_ADDR_S);

	while((U2DOTGUCR & U2DOTGUCR_RUN) && i--);

	if(i<=0){
		printk(KERN_DEBUG "Read ULPI register Time out,"
			       " reg %x otgucr %x, usr %x ucr %x\n", 
			       reg, U2DOTGUCR, U2DOTGUSR, U2DOTGCR);
		return -1;
	}

	*value = (u8)(U2DOTGUCR & U2DOTGUCR_RDATA);

        DMSG("read ulpi reg %x val %x\n", reg, *value);
	return 0;

}

int ulpi_reg_write(u8 reg, u8 value)
{	int i=10000;
	enum u2d_phy_mode state = ulpi_get_phymode();

	if((state != SYNCH) && (state != PRE_SYNCH)) {
		dmsg(": not in SYNCH mode!!!");
		return -1;
	}

	U2DOTGUCR = U2DOTGUCR_RUN | (reg<<U2DOTGUCR_ADDR_S) \
	       	| (value<<U2DOTGUCR_WDATA_S);

	while((U2DOTGUCR & U2DOTGUCR_RUN) && i--);

	if(i<=0){
		printk(KERN_DEBUG "Write ULPI register Time out,"
			       " reg %x val %x\n", reg, (int)value);
		return -1;
	}
        DMSG("write ulpi reg %x val %x\n", reg, (int)value);

	return 0;
}

int ulpi_set_phymode(enum u2d_phy_mode mode)
{
	u32 state;
	u32 u2dotgcr;
	u32 u2dp3cr;

	if(d0cs) {
		pr_debug("%s: in D0CS mode, please try set phy mode"
			" %d again\n", __func__, mode);
		return -EAGAIN;
	}

	ulpi_dat3_int_set(0);
	state = ulpi_get_phymode();
	u2dotgcr = U2DOTGCR;
	u2dp3cr = U2DP3CR;

	if((state == mode) && !xcvr_init) {
		if (mode == SYNCH)
			U2DOTGCR &= ~(U2DOTGCR_UTMID);
		return 0;
	}

	if(xcvr_init == 1)
		xcvr_init = 0;

	if((state != SYNCH) && (state != PRE_SYNCH)) {
		ulpi_rtsm();
	}

	switch(mode) {

	case SYNCH:
		/* disable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DMPULLDOWN);

			
		/* clear UDE and ULE before enable UTMI */
		U2DOTGICR = 0;
		U2DCR &= ~U2DCR_UDE;
		U2DOTGCR &= ~U2DOTGCR_ULE;

		/* enable the UTMI */
		u2dotgcr = U2DOTGCR;
		u2dotgcr &= ~(U2DOTGCR_UTMID|U2DOTGCR_SMAF);
		U2DOTGCR = u2dotgcr;
		u2dotgcr = U2DOTGCR;

		/* enable the ULE again */
		U2DOTGCR |= U2DOTGCR_ULE;
		U2DOTGICR = U2DOTGINT_DEFAULT;
		
		break;

	case SER_6PIN:
	case SER_3PIN:
		/* enable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DMPULLDOWN);

		/* switch to serial mode */
		u2dp3cr &= ~(U2DP3CR_P2SS);
		if(mode == SER_3PIN)
			u2dp3cr |= 0x1 << U2DP3CR_P2SS_S;
		U2DP3CR = u2dp3cr;

		/* set PHY into host mode */
		ulpi_reg_write(ULPI_FUNCTION_CONTROL_SET, 0x45);

		/* set ULPI PHY to serial mode */
		if(mode == SER_3PIN)
			ulpi_reg_write(ULPI_INTERFACE_CONTROL, ULPI_IC_3PIN);
		else
			ulpi_reg_write(ULPI_INTERFACE_CONTROL, ULPI_IC_6PIN);

		/* enable serial mode */
		u2dotgcr |= U2DOTGCR_SMAF;
		u2dotgcr &= ~(U2DOTGCR_ULAF | U2DOTGCR_CKAF);
		U2DOTGCR = u2dotgcr;
		dmsg("U2DOTGCR %08X, U2DOTGICR %08X, U2DP3CR %08x, U2DOTGUSR %08X\n",
			     	U2DOTGCR, U2DOTGICR, U2DP3CR, U2DOTGUSR);
		break;

	case LOWPOWER:
		/* enable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DMPULLDOWN);

		/* clear SuspendM in UPLI PHY  */
		ulpi_reg_write(ULPI_FUNCTION_CONTROL_CLEAR, ULPI_FC_SUSPENDM);
		

		if (u2d_bugs & U2D_FIX_ULPI_STP) { 
			/* disable oscc reference clock, and u2d clock */
			disable_oscc_pout();

			/* enable ULPI_DAT3 gpio interrupt */
			ulpi_dat3_int_set(1);
		}
		break;

	case CARKIT:
#if defined(CONFIG_CPU_PXA310) && defined(CONFIG_MACH_LITTLETON)
		/* Enable Carkit mode */

		/* disable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DMPULLDOWN);

		/* Enable ID pullup */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_IDPULLUP);

		/* Configure Carkit Interrupts */
		ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_SET, ULPI_CK_IDFLOATRISE);
		ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_SET, ULPI_CK_IDFLOATFALL);
		ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_SET, ULPI_CK_RIDINTEN);

		/* ULPI Switch to Carkit Mode */
		
		U2DOTGCR |= U2DOTGCR_UTMID;

		U2DOTGICR = U2DOTGINT_DEFAULT;

		U2DOTGCR |= U2DOTGCR_ULE;
		
		ulpi_reg_write(ULPI_INTERFACE_CONTROL, ULPI_IC_CARKIT); /* XCVR Carkit mode */ 

		/* Carkit Mode Alternate Function Select */
		u2dotgcr = U2DOTGCR;
		u2dotgcr &= ~(U2DOTGCR_SMAF);
		u2dotgcr &= ~(U2DOTGCR_ULAF);
		u2dotgcr |= U2DOTGCR_CKAF;
		U2DOTGCR = u2dotgcr;

		break;
#endif
	default:
		printk(KERN_ERR "unsupported ULPI operation modes %d\n", mode);
		return -1;
		break;
	}

	xcvr_mode = mode;
	/* printk(KERN_DEBUG "state %d to mode %d end, USR 0x%8x\n", 
		state, mode, U2DOTGUSR); */

	return 0;
}

#if defined(CONFIG_CPU_PXA310) && defined(CONFIG_MACH_LITTLETON)
/* Attempt to detect USB headset via USB Cable's Resitor ID */
int ulpi_detect_headset (void)
{
	u32 state = ulpi_get_phymode();
	u8 vendor_rid = 0;
	int ret = 0;
	int i = 0;
	
	if(xcvr_init)
		return 0;

	if(d0cs) {
		printk(KERN_DEBUG "%s: in D0CS mode, please try set phy"
			"again\n", __func__) ;
		return -EAGAIN;
	}

	if((state != SYNCH) && (state != PRE_SYNCH)) {
		ulpi_rtsm();
	}

	/* disable RidConversionDone interrupt and RID Short Cycle */
	ulpi_reg_write (ULPI_VENDOR_RID_CONV, 0);
	
	/* perform conversion */
	ulpi_reg_write (ULPI_VENDOR_RID_CONV_SET, RID_CONV_START);
	
	/* Wait until RID Conversion is complete */
	for(i=0; i<10000; i++)
	{
	  ulpi_reg_read(ULPI_VENDOR_RID_CONV, &vendor_rid);
	  if (vendor_rid & RID_CONV_DONE) break;
	}

	printk (KERN_NOTICE "USB Headset Vendor RID: 0x%08x\n", vendor_rid);

	vendor_rid &= RID_VALUE_MASK;

	if (vendor_rid == RID_100K)
	{
		printk (KERN_NOTICE "Headset detected RID = 100k ohm\n");
		/* May need adjustment based on headset */
		ret = USB_HEADSET_STEREO;
	}
		
	if (vendor_rid == RID_200K) 
	{
		printk (KERN_NOTICE "Headset detected RID = 200k ohm\n");
		/* May need adjustment based on headset */
		ret = USB_HEADSET_STEREO;
	}
		
	if (vendor_rid == RID_440K)
	{
		printk (KERN_NOTICE "Headset detected RID = 440k ohm\n");
		/* May need adjustment based on headset */
		ret = USB_HEADSET_MONO_MIC;
	}
	
	if (!ret)
	{
		printk (KERN_NOTICE "Headset configuration not available for Vendor RID: 0x%08x\n", vendor_rid);
	}
	
	return ret;
}
#endif

#ifndef CONFIG_USB_OTG_PXA3xx_U2D
static void ulpi_phy_init(void)
{
	u32 u2dotgcr = U2DOTGCR;

 	ulpi_rtsm();
	/* set to peripheral mode */
	U2DOTGCR = 0;
	u2dotgcr = U2DOTGCR;

	u2dotgcr |= U2DOTGCR_ULE;
	U2DOTGCR = u2dotgcr;

	u2dotgcr |= U2DOTGCR_ULAF;
	u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF);
	U2DOTGCR = u2dotgcr;
	u2dotgcr = U2DOTGCR;
}
#endif

void dump_ulpi_regs(void)
{	u8 val;

	ulpi_reg_read(ULPI_VENDOR_LOW, &val);
	printk(KERN_DEBUG "vendor ID low %02X\n", val);

	ulpi_reg_read(ULPI_VENDOR_HIGH, &val);
	printk(KERN_DEBUG "vendor ID high %02X\n", val);

	ulpi_reg_read(ULPI_PRODUCT_LOW, &val);
	printk(KERN_DEBUG "vendor PRODUCT low %02X\n", val);

	ulpi_reg_read(ULPI_PRODUCT_HIGH, &val);
	printk(KERN_DEBUG "vendor PRODUCT high %02X\n", val);

	ulpi_reg_read(ULPI_FUNCTION_CONTROL, &val);
	printk(KERN_DEBUG "function control %02X\n", val);

	ulpi_reg_read(ULPI_INTERFACE_CONTROL, &val);
	printk(KERN_DEBUG "interface control %02X\n", val);

	ulpi_reg_read(ULPI_OTG_CONTROL, &val);
	printk(KERN_DEBUG "otg control %02X\n", val);

	ulpi_reg_read(ULPI_INT_RISE, &val);
	printk(KERN_DEBUG "interrupt enable rising %02X\n", val);

	ulpi_reg_read(ULPI_INT_FALL, &val);
	printk(KERN_DEBUG "interrupt enable falling %02X\n", val);

	ulpi_reg_read(ULPI_INT_STATUS, &val);
	printk(KERN_DEBUG "interrupt status %02X\n", val);

	ulpi_reg_read(ULPI_INT_LATCH, &val);
	printk(KERN_DEBUG "interrupt latch %02X\n", val);

}
#else

#define MFP_ULPI_INT		MFP_RDY

static irqreturn_t ulpi_dat3_irq(int irq, void *devid) {}
static void ulpi_dat3_work(void) {}
#endif

/*-------------------------------------------------------------------*/
/*
 * get_extra_descriptor() finds a descriptor of specific type in the
 * extra field of the interface and endpoint descriptor structs.
 */
static int get_extra_descriptor(char *buffer, unsigned size,
	unsigned char type, void **ptr)
{
	struct usb_descriptor_header *header;

#ifdef DEBUG
	/* dump_buffer(buffer, size); */
#endif
	*ptr = buffer;
	while (size >= sizeof(struct usb_descriptor_header)) {
		header = (struct usb_descriptor_header *)buffer;

		if (header->bLength < 2) {
			DMSG("%s: descriptor, type %d length %d not found\n",
				__FUNCTION__,
				header->bDescriptorType,
				header->bLength);
			return -ENODATA;
		}

		if (header->bDescriptorType == type) {
			*ptr = header;
			return size;
		}

		buffer += header->bLength;
		size -= header->bLength;
	}
	return -ENODATA;
}


#ifdef CONFIG_USB_COMPOSITE
static void u2d_setup_complete (struct usb_ep *ep,
					struct usb_request *req);

#ifdef DEBUG
/*  dump the gadget_driver_info structure
 */
static void gadget_info_dump(void)
{
	struct pxa3xx_u2d	*dev = the_controller;
	struct gadget_driver_info *pInfo = dev->first_gadget;
	int i = 1;

	printk(KERN_DEBUG "%s, dev->interface_count= 0x%x\n", __FUNCTION__, dev->interface_count);
	while(pInfo){

		printk(KERN_DEBUG "i=%d, pInfo=%p\n", i, pInfo);
		printk(KERN_DEBUG "   next = 0x%x\n", (unsigned)pInfo->next);
		printk(KERN_DEBUG "   config = 0x%x\n", pInfo->config);
		printk(KERN_DEBUG "   assigned_intf_start = 0x%x\n", pInfo->assigned_intf_start);
		printk(KERN_DEBUG "   num_intfs = 0x%x\n", pInfo->num_intfs);
		printk(KERN_DEBUG "   ep_start = 0x%x\n", pInfo->ep_start);
		printk(KERN_DEBUG "   config_desc = 0x%x\n", (unsigned)pInfo->config_desc);
		printk(KERN_DEBUG "   driver = 0x%x\n", (unsigned)pInfo->driver);
		printk(KERN_DEBUG "   driver_data = 0x%x\n", (unsigned)pInfo->driver_data);

		pInfo = pInfo->next;
		i++;
	}
	printk(KERN_DEBUG "dev->first_gadget = %p\n", dev->first_gadget);
	printk(KERN_DEBUG "dev->active_gadget = %p\n", dev->active_gadget);
}
#endif

/* gadget_info_init
 * init the gadget_driver_info structure when the driver is registered
 * combined from several gadget driver, should be lager ??
 */
#define REQ_BUFSIZ 256
static int gadget_info_init(struct usb_gadget_driver *driver)
{
	struct pxa3xx_u2d *dev = the_controller;
	struct gadget_driver_info *info;
	struct gadget_driver_info *pInfo;

	/* set up the new gadget driver info */
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info){
		printk(KERN_ERR "kmalloc gadget_driver_info error\n");
		return -EBUSY;
	}

	info->driver = driver;
	info->next = NULL;

	info->ep_start = num_ep_used;
	info->num_eps = 0;
	info->num_intfs= 0;
	info->stopped = 1;

	if(dev->first_gadget) {
		/* find the last element */
		pInfo = dev->first_gadget;
		while(pInfo->next){
			pInfo = pInfo->next;
		}
		/* set up the struct.
		 * the last registered driver is always the active one
		 * before receive the set_interface request 
		 */
		pInfo->next = info;
		dev->active_gadget = info;
	} else {
		dev->first_gadget = dev->active_gadget = info;
		dev->interface_count = 0;
		dev->driver_count = 0;

		/* init ep0 control request queueand buffer */
		memset ((void *)&dev->ep0_req, 0, sizeof (dev->ep0_req));
		INIT_LIST_HEAD (&dev->ep0_req.queue);

		dev->ep0_req.req.complete = u2d_setup_complete;
		dev->ep0_req.req.buf = pxa3xx_ep_alloc_buffer(dev->gadget.ep0,
			REQ_BUFSIZ,	&dev->ep0_req.req.dma, GFP_KERNEL);
		if (!dev->ep0_req.req.buf) {
			usb_ep_free_request (dev->gadget.ep0, &dev->ep0_req.req);
			DMSG("%s, dev->ep0_req.req.buf malloc error\n", __FUNCTION__);
		}
	}

	return 0;
}

static int gadget_info_uninit(struct usb_gadget_driver *driver)
{
	struct pxa3xx_u2d *dev = the_controller;
	struct gadget_driver_info * pInfo = dev->first_gadget;
	struct gadget_driver_info * info = NULL;
	int i;

	do{
		/* find the gadget driver info to pInfo */
		if(pInfo->driver == driver) {
			/* for the first driver is being removed*/
			if(!info)
				info=pInfo->next;
			break;
		}
		/* save the previous one */
		info=pInfo;
		pInfo=pInfo->next;
	}while(pInfo);

	if(NULL==pInfo) {
		printk(KERN_ERR"%s, can't find driver!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* put the active one to the previous one */
	if(dev->first_gadget == pInfo){
		if(info)
			dev->first_gadget = info;
		else
			dev->first_gadget = pInfo->next;
	}
	if(dev->active_gadget == pInfo){
		if(info) 
			dev->active_gadget = info;
		else 
			dev->active_gadget = pInfo->next;
	}
	if((info) && (info!=pInfo->next))
		info->next = pInfo->next;

	if(dev->active_gadget)
		dev->driver = dev->active_gadget->driver;
	else
		dev->driver = 0; /* no drivers left */

	for (i=pInfo->ep_start; i<(pInfo->ep_start+pInfo->num_eps);i++){
		struct pxa3xx_ep *ep = &dev->ep[i];
		ep->assigned = 0;
		ep->desc = NULL;
		kfree(ep->ep.name);
	}

	kfree(pInfo);
	return 0;
}

#ifdef MULTI_P3
struct usb_interface_assoc_descriptor
iad_desc = {
	.bLength = sizeof iad_desc,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface = 0,
	.bInterfaceCount = 0,
	.bFunctionClass = 0,
	.bFunctionSubClass = 0,
	.bFunctionProtocol = 0,
	.iFunction = 0,
};

#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS
static void  set_iad_desc(struct usb_device_descriptor * device_desc,
		__u8 first_intf, __u8 num_intfs)
{
	iad_desc.bFirstInterface = first_intf;
	iad_desc.bInterfaceCount = num_intfs;

	iad_desc.bFunctionClass = device_desc->bDeviceClass;
	iad_desc.bFunctionSubClass = device_desc->bDeviceSubClass;
	iad_desc.bFunctionProtocol = device_desc->bDeviceProtocol;
}
#endif

static int gadget_get_device_desc(void)
{	struct pxa3xx_u2d *dev = the_controller;
	struct usb_ctrlrequest  req;
	int i;

	DMSG(KERN_DEBUG "%s\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_DEVICE<<8);
	req.wIndex = 0;
	req.wLength = sizeof (struct usb_device_descriptor);

	dev->ep0state = EP0_IN_FAKE;
	i=dev->driver->setup(&dev->gadget, &req);

	return 0;
}
#endif

#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS
/* combine_configuration
 * Combine the configuration descriptors for all gadget drivers registered.
 * Add the IAD descriptor if there are more than one interfaces within one
 * function.
 */
static int combine_configuration(int speed)
{	struct pxa3xx_u2d	*dev = the_controller;
	struct gadget_driver_info *pInfo = dev->first_gadget;
	struct usb_config_descriptor *config_desc;
#ifdef MULTI_P3
	struct usb_interface_assoc_descriptor *p_iad_desc;
#endif
	struct usb_config_descriptor *configs =
			(struct usb_config_descriptor *)dev->configs;
	int desc_length;

	DMSG("%s\n", __FUNCTION__);

	/* config desc, may diff between gadget drivers */
	configs->bLength = USB_DT_CONFIG_SIZE;
	configs->bDescriptorType = USB_DT_CONFIG;

	configs->wTotalLength = USB_DT_CONFIG_SIZE;
	configs->bNumInterfaces = 0;
	configs->bConfigurationValue = /* U2D_DEFAULT_CONFIG;*/
		((struct usb_config_descriptor *)pInfo->config_desc)->bConfigurationValue;
	configs->iConfiguration = 0;
	configs->bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
	configs->bMaxPower = 1;

	do{
		if(speed ==  USB_SPEED_HIGH)
			config_desc = (struct usb_config_descriptor *)(pInfo->config_desc_hs);
		else
			config_desc = (struct usb_config_descriptor *)(pInfo->config_desc);
		configs->bNumInterfaces += (u8)(config_desc->bNumInterfaces);

#ifdef MULTI_P3
		/* add the IAD descriptor if there are multiple interface in one
		   gadget driver */
		if(config_desc->bNumInterfaces>1){

			if((get_extra_descriptor ((char *)config_desc,
					config_desc->wTotalLength,	USB_DT_INTERFACE_ASSOCIATION,
					(void **) &p_iad_desc)) >= 0){
				p_iad_desc->bFirstInterface = pInfo->assigned_intf_start;
			} else {
				/* fill the iad_desc functionClass/subclass/protocol fields */
				set_iad_desc((struct usb_device_descriptor *)pInfo->device_desc,
					pInfo->assigned_intf_start, config_desc->bNumInterfaces);
				memcpy((u8*)configs + configs->wTotalLength,
					(u8*)&iad_desc, sizeof iad_desc);
				configs->wTotalLength += sizeof iad_desc;
			}
		}
#endif

		/* copy all descriptors except config desc */
		memcpy((u8*)configs + configs->wTotalLength,
				(u8 *)config_desc + USB_DT_CONFIG_SIZE,
				config_desc->wTotalLength -USB_DT_CONFIG_SIZE);

		/* modify the interface number to assigned interface number */
		desc_length = config_desc->wTotalLength - USB_DT_CONFIG_SIZE;

		configs->wTotalLength += (config_desc->wTotalLength-USB_DT_CONFIG_SIZE);
		pInfo = pInfo->next;
		DMSG("configs->wTotalLength = 0x%x\n", configs->wTotalLength);
	}while(pInfo!=NULL);

#ifdef DEBUG
	gadget_info_dump();
#endif

	return configs->wTotalLength;
}
#endif

/* set_eps
 * fill pxa_ep structure with their configuration, interface, alternate
 * settings, assigned interface number
 */
static int set_eps(__u8 num_eps, int ep_start_num,
		struct usb_endpoint_descriptor *p_ep_desc, int len,
		int config, int interface, int alt)
{	struct pxa3xx_u2d	*dev = the_controller;
	struct usb_endpoint_descriptor *ep_desc = p_ep_desc;
	int ep_desc_length = len;
	int j, k, ret;

	DMSG("  ----%s----\n", __FUNCTION__);
	DMSG("  num_eps=0x%x, p_ep_desc=0x%x\n",
				num_eps, (int)p_ep_desc);

	for(j=0;j<num_eps;j++){
		DMSG("  search eps: ep_start_num=%d, num_ep_used=%d\n",
				ep_start_num, num_ep_used);
		/* find the ep */
		if((ret = get_extra_descriptor ((char *)p_ep_desc,
				ep_desc_length,
				USB_DT_ENDPOINT, (void **) &ep_desc)) >= 0){
			DMSG("  ep_desc->bEndpointAddress = 0x%x, ep_desc=0x%x,"
				" p_ep_desc=0x%x\n",
				ep_desc->bEndpointAddress, (int)ep_desc, (int)p_ep_desc);
			/* compare with the ep in pxa27x, if match, fill the config,
			   interface and asin number fields */
			for(k=ep_start_num;k<U2D_EP_NUM;k++){
				if(dev->ep[k].ep_num == (ep_desc->bEndpointAddress & 0x0f)){
					dev->ep[k].assigned_interface = dev->interface_count;
					dev->ep[k].config = config;
					dev->ep[k].interface = interface;
					dev->ep[k].aisn = alt;
					dev->ep[k].driver_info = dev->active_gadget;
					DMSG("  found ep num = %d, old interface=%d,"
						" assigned_interface=%d\n",
						k, interface, dev->ep[k].assigned_interface);
					break;
				}
			}
		} else {
			DMSG("  ep desc not find, ep_desc_length=0x%x, p_ep_desc=0x%x\n",
				ep_desc_length, (int)p_ep_desc);
			return -EFAULT;
		}
		DMSG("    ep_desc_length = %d, ep_desc=0x%x, p_ep_desc=0x%x,"
			" search ep %d end\n",
				ep_desc->bEndpointAddress, (int)ep_desc, (int)p_ep_desc, j);

		ep_desc_length -= (int)ep_desc - (int)p_ep_desc + ep_desc->bLength;
		p_ep_desc = (struct usb_endpoint_descriptor *)((unsigned)ep_desc +
				ep_desc->bLength);
	}/* for(j=0;j<num_eps;j++) */

	return 0;
}

/* set_cdc_desc
 * modify the cdc union descriptor which include the master/slave interface
 * number
 */
static void set_cdc_desc(void)
{
	struct pxa3xx_u2d *dev = the_controller;
	struct usb_device_descriptor *device_desc =
		(struct usb_device_descriptor *)dev->active_gadget->device_desc;
	struct usb_config_descriptor *config_desc =
	       	(struct usb_config_descriptor *)dev->active_gadget->config_desc;
	struct usb_cdc_union_desc *union_desc;
	int config_desc_len = config_desc->wTotalLength, ret=0;

	if(device_desc->bDeviceClass != USB_CLASS_COMM) return;

	while((config_desc_len > 0) && (ret>=0)){
		if((ret = get_extra_descriptor ((char *)config_desc,
			config_desc_len,
			USB_DT_CS_INTERFACE, (void **) &union_desc)) >= 0){
			if(union_desc->bDescriptorSubType == USB_CDC_UNION_TYPE){
				DMSG("found cdc union desc, change to %d\n",
					dev->active_gadget->assigned_intf_start);
				union_desc->bMasterInterface0 =	dev->active_gadget->assigned_intf_start;
				union_desc->bSlaveInterface0 = dev->active_gadget->assigned_intf_start+1;
			}

			if(union_desc->bDescriptorSubType == USB_CDC_CALL_MANAGEMENT_TYPE){
				DMSG("found cdc call mgt desc, change to %d\n",
					dev->active_gadget->assigned_intf_start+1);
				((struct usb_cdc_call_mgmt_descriptor *)union_desc)->bDataInterface =
					dev->active_gadget->assigned_intf_start+1;
			}
			config_desc_len -= ((unsigned)union_desc -
			       	(unsigned)config_desc + union_desc->bLength);
			config_desc = (struct usb_config_descriptor *)	\
				((unsigned)union_desc + union_desc->bLength);
		}
	}
}

#endif

/* After driver is bound, send a fake get configuration command to
 * gadget driver to get the configuration information */
static int gadget_get_config_desc(void)
{
	struct pxa3xx_u2d *dev = the_controller;
	struct usb_ctrlrequest  req;
	struct usb_config_descriptor *config_desc;
	struct usb_interface_descriptor *interface_desc;
	unsigned config;
	int i;

	DMSG("----------%s------------\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_CONFIG<<8);
	req.wIndex = 0;
	req.wLength = MAX_CONFIG_LENGTH;

	dev->ep0state = EP0_IN_FAKE;
	dev->gadget.speed = USB_SPEED_FULL;
	i=dev->driver->setup(&dev->gadget, &req);
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	config_desc = (struct usb_config_descriptor*)dev->active_gadget->config_desc;

	if (config_desc->bDescriptorType == USB_DT_CONFIG) {
		config = config_desc->bConfigurationValue;
	} else {
		DMSG("wrong configuration\n");
		return -EFAULT;
	}


	{
	__u8 num_itfs = config_desc->bNumInterfaces;
	__u8 ep_num = dev->active_gadget->ep_start, cur_intf=0, last_intf=0;
	struct usb_config_descriptor * p_config_desc = config_desc;
	int config_desc_length = config_desc->wTotalLength, ret;

	DMSG("parse the config desc, assigned_intf_start=%d, num of intfs=%d\n",
		dev->interface_count, num_itfs);

	dev->active_gadget->assigned_intf_start = dev->interface_count;
	dev->active_gadget->config = config;
	dev->active_gadget->num_intfs = num_itfs;

	/* get every interface desc, fill the gadget_driver_info structure */
	for(i=0;i<num_itfs;i++){
	  	DMSG("\nparse interface %d, p_config_desc=%p, config_desc_length=%d\n",
			i, p_config_desc, config_desc_length);

	  	while(config_desc_length >=0){
			if((ret = get_extra_descriptor ((char *)p_config_desc,
						config_desc_length,
						USB_DT_INTERFACE, (void **) &interface_desc)) >= 0){
				cur_intf = interface_desc->bInterfaceNumber;
				DMSG("  cur_intf=%d, last_intf=%d, interface_desc=%p, "
					"config_desc_length=%d\n",
					cur_intf, last_intf, interface_desc, config_desc_length);

				config_desc_length -= (u32)interface_desc - (u32)p_config_desc;

				if(cur_intf!=last_intf) {
					p_config_desc = (struct usb_config_descriptor *)interface_desc;
					goto next_intf;
				}

				/* set interface number to assigned one */
				interface_desc->bInterfaceNumber =
						dev->active_gadget->assigned_intf_start + i;

#ifdef MULTI_P4
				/* set string desc id */
					{
						struct t_str_id * pStr = dev->str_id;
						int j=1;
						if(interface_desc->iInterface){
							if(pStr){
								j++;
								while(pStr->next) {
									pStr = pStr->next;
									j++;
								}
								pStr->next = kmalloc(sizeof(*pStr), GFP_KERNEL);
								pStr = pStr->next;
							}else{
								pStr = kmalloc(sizeof(*pStr), GFP_KERNEL);
								dev->str_id = pStr;
								j=1;
							}
							pStr->driver_info = dev->active_gadget;
							pStr->str_id = interface_desc->iInterface;
							pStr->next = NULL;
							interface_desc->iInterface = j;
						}
					}
#else
					interface_desc->iInterface = 0;
#endif

				/* search eps and fill the pxa3xx_ep_config struct */
				if(interface_desc->bNumEndpoints){
					set_eps(interface_desc->bNumEndpoints, ep_num,
						(struct usb_endpoint_descriptor *)interface_desc,
						config_desc_length,
						config, cur_intf, interface_desc->bAlternateSetting);
				}

				DMSG("  num_ep_used=%d, start from %d to %d, config=%d, "
					"intf=%d(assigned=%d), alt=%d\n\n",
					num_ep_used, ep_num, ep_num+interface_desc->bNumEndpoints,
					config, cur_intf, interface_desc->bInterfaceNumber,
					interface_desc->bAlternateSetting);
			} else {
				DMSG("    no more alt interfaces, config_desc_length=%d, "
					" goto next_intf\n", config_desc_length);
				goto next_intf;
			}/* if */

			p_config_desc = (struct usb_config_descriptor *)	\
				((struct usb_interface_descriptor *)interface_desc + 1);

			config_desc_length -= interface_desc->bLength;  /* yfw */
			DMSG("  p_config_desc=%p, interface_desc=%p, "
				"config_desc_length=%d\n",
				p_config_desc, interface_desc, config_desc_length);
	  	}/* while */

next_intf:
  	  	last_intf = cur_intf;
	  	dev->interface_count ++;

	  	DMSG("parse interface %d finished, dev->interface_count=%d\n",
			i, dev->interface_count);
	}

	/* set CDC union descriptors */
	set_cdc_desc();

	return 0;
	}
}

/* get the hs configuration desc, change the interface number */
static int	gadget_get_config_desc_hs(void)
{
	struct pxa3xx_u2d *dev = the_controller;
	struct usb_ctrlrequest  req;
	struct usb_config_descriptor *config_desc;
	struct usb_interface_descriptor *interface_desc;
	unsigned config;
	int i;

	DMSG("----------%s------------\n", __FUNCTION__);
	req.bRequestType = USB_RECIP_DEVICE | USB_DIR_IN;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = (USB_DT_CONFIG<<8);
	req.wIndex = 0;
	req.wLength = MAX_CONFIG_LENGTH;

	dev->ep0state = EP0_IN_FAKE;
	dev->gadget.speed = USB_SPEED_HIGH;
	i=dev->driver->setup(&dev->gadget, &req);
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	config_desc = (struct usb_config_descriptor*)dev->active_gadget->config_desc_hs;

	if (config_desc->bDescriptorType == USB_DT_CONFIG) {
		config = config_desc->bConfigurationValue;
	} else {
		DMSG("wrong configuration\n");
		return -EFAULT;
	}

	{
	__u8 num_itfs = config_desc->bNumInterfaces;
	__u8 cur_intf=0, last_intf=0;
	struct usb_config_descriptor * p_config_desc = config_desc;
	int config_desc_length = config_desc->wTotalLength, ret;

	DMSG("parse the config desc, assigned_intf_start=%d, num of intfs=%d\n",
		dev->interface_count, num_itfs);

	/* get every interface desc, fill the gadget_driver_info structure */
	for(i=0;i<num_itfs;i++){
	  	DMSG("\nparse interface %d, config_desc_length=%d\n",
			i, config_desc_length);

	  	while(config_desc_length >=0){
			if((ret = get_extra_descriptor ((char *)p_config_desc,
						config_desc_length,
						USB_DT_INTERFACE, (void **) &interface_desc)) >= 0){
				cur_intf = interface_desc->bInterfaceNumber;
				DMSG("  cur_intf=%d, last_intf=%d, config_desc_length=%d\n",
					cur_intf, last_intf, config_desc_length);

				config_desc_length -= (u32)interface_desc - (u32)p_config_desc;

				if(cur_intf!=last_intf) {
					p_config_desc = (struct usb_config_descriptor *)interface_desc;
					goto next_intf;
				}

				/* set interface number to assigned one */
				interface_desc->bInterfaceNumber =
						dev->active_gadget->assigned_intf_start + i;

#ifdef MULTI_P4
				/* set string desc id */
					{
						struct t_str_id * pStr = dev->str_id;
						int j=1;
						if(interface_desc->iInterface){
							if(pStr){
								j++;
								while(pStr->next) {
									pStr = pStr->next;
									j++;
								}
								pStr->next = kmalloc(sizeof(*pStr), GFP_KERNEL);
								pStr = pStr->next;
							}else{
								pStr = kmalloc(sizeof(*pStr), GFP_KERNEL);
								dev->str_id = pStr;
								j=1;
							}
							pStr->driver_info = dev->active_gadget;
							pStr->str_id = interface_desc->iInterface;
							pStr->next = NULL;
							interface_desc->iInterface = j;
						}
					}
#else
					interface_desc->iInterface = 0;
#endif
			} else {
				DMSG("    no more alt interfaces, config_desc_length=%d, "
					" goto next_intf\n", config_desc_length);
				goto next_intf;
			}/* if */

			p_config_desc = (struct usb_config_descriptor *)	\
				((struct usb_interface_descriptor *)interface_desc + 1);

			config_desc_length -= interface_desc->bLength;  /* yfw */
	  	}/* while */

next_intf:
  	  	last_intf = cur_intf;

	  	DMSG("parse interface %d finished, dev->interface_count=%d\n",
			i, dev->interface_count);
	}

	/* set CDC union descriptors */
	set_cdc_desc();

	return 0;
	}
}

/* u2d_eps_reset
 * clear the endpoint configuration and information register
 */
static void u2d_eps_reset(void)
{
	struct pxa3xx_u2d *dev = the_controller;
	int i;

	for(i=1;i<U2D_EP_NUM;i++){
		if(!dev->ep[i].assigned) continue;
		U2DEN(i) = i;
		U2DEPCR(i) = 0;
	}
}

/* u2d_eps_config
 * set the endpoint configuration and information register
 */
static int u2d_eps_config(int phase)
{
	struct pxa3xx_u2d *dev = the_controller;
	struct pxa3xx_ep *ep = NULL;
	unsigned config_num, intf_num, i;

	for (i=1; i < U2D_EP_NUM; i++) {
		if (!dev->ep[i].assigned)
			continue;

		ep = &dev->ep[i];
		config_num = ((struct usb_config_descriptor *)dev->	\
			     first_gadget->config_desc)->bConfigurationValue;
		intf_num = ep->assigned_interface;

		if (phase == 1)	{
			U2DEPCR(ep->ep_num) = (ep->fifo_size>>2) | U2DEPCR_EE;
		} else if (phase == 2) {
			if (u2d_bugs & U2D_BUG_SETINTF) {
				struct usb_device_descriptor *desc =
				(struct usb_device_descriptor *)dev->active_gadget->device_desc;

				if(((desc->idVendor == 0x0525) && (desc->idProduct == 0xa4a2)) || \
				  (((desc->idVendor == 0x0525) && (desc->idProduct == 0xa4a7)))){
						intf_num = 0;		/* FIXME rndis, cdc acm */
				}
			}
			U2DEN(ep->ep_num) = (ep->ep_num <<0 ) |	(ep->dir_in <<4) |
					    (ep->ep_type <<5) | (config_num <<7) |
					    (intf_num <<11)   | (ep->aisn <<15)  |
					    (ep->ep.maxpacket <<19) | (ep->hs_cmds <<30);
		}
	}

	return 0;
}

/*
 * 	__u2d_disable - disable USB device controller
 */
static void __u2d_disable(struct pxa3xx_u2d *dev)
{
#ifdef CONFIG_CPU_PXA310
	u32 u2dotgicr;
#endif

	u2d_clk_enable();

	/* soft disconnect & disable U2D */
	/*if(!(u2d_bug_check() & U2D_BUG_RECON))
	{
		u2d_soft_dis(1);
	}*/

	if (!(U2DCR & U2DCR_UDE)) 
		goto done;

	U2DCR &= ~U2DCR_UDE;

	/* clear U2D interrupts, include endpoints and U2DMAs */
	U2DICR = 0x00000000;
	U2DISR = 0xfeffffff;
#ifdef CONFIG_CPU_PXA310
	U2DICR2 = 0x00000000;
	U2DISR2 = 0x00ffffff;

	u2dotgicr = U2DOTGICR;
	U2DOTGICR = 0x00000000;

	U2DOTGCR &= ~U2DOTGCR_ULE;
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	U2DOTGCR = U2DOTGCR_UTMID|U2DOTGCR_OTGEN;
#else
	U2DOTGCR = U2DOTGCR_UTMID;
#endif
	U2DOTGCR |= U2DOTGCR_ULE;
	U2DOTGICR = u2dotgicr;
#endif

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->configuration = 0;
	dev->interface = 0;
	dev->alternate = 0;

done:
	u2d_clk_restore();
}

/*
 * 	u2d_reinit - initialize software state
 */
static void u2d_reinit(struct pxa3xx_u2d *dev)
{
	u32	i;

	u2d_clk_enable();
	dev->ep0state = EP0_IDLE;

	/* basic endpoint records init */
	for (i = 0; i < U2D_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];

		ep->stopped = 0;
		ep->pio_irqs = ep->dma_irqs = 0;
	}

	dev->configuration = 0;
	dev->interface = 0;
	dev->alternate = 0;
	/* the rest was statically initialized, and is read-only */
	u2d_clk_restore();
}

/* until it's enabled, this U2D should be completely invisible
 * to any USB host.
 */
static void __u2d_enable (struct pxa3xx_u2d *dev)
{
	int i;

	u2d_clk_enable();
	if (U2DCR & U2DCR_UDE)
		goto done;

	DMSG("%s\n", __func__);

	ep0_idle(dev);
	/* default speed, or should be unknown here? */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->stats.irqs = 0;

	/* check whether U2DCR[EMCE]=1 */
	if(U2DCR_EMCE & U2DCR) {
		printk(KERN_ERR "%s, Endpoint Memory Configuration Error\n", __FUNCTION__);
	}

	/* enable suspend/resume and reset irqs */
	U2DICR = U2DINT_CC | U2DINT_RU | U2DINT_SU | U2DINT_RS | U2DINT_DPE;

	/* enable ep0 irqs */
	U2DICR |= U2DINT(0, U2DINT_PACKETCMP);
	U2DICR |= U2DINT(0, U2DINT_SPACKETCMP);

	U2DCSR0 = U2DCSR0_FTF;

	U2DEN0 = 0x2000000;
	for(i=1;i<U2D_EP_NUM;i++){
		U2DEN(i) = i;
	}
	DMSG("%s: U2DCR = 0x%x, U2DEN0(%p) = 0x%x, U2DICR = 0x%x\n",
		__FUNCTION__, U2DCR, &U2DEN0, U2DEN0, U2DICR);

#ifdef EP0_OUT_DMA
	/* configure the DMA for OUT packets through EP0 */

	/* disable  */
	U2DICR &= ~(U2DINT(0, U2DINT_PACKETCMP));

#else
	/* receive FIFO not empty? */
	if(U2DCSR0 & U2DCSR0_RNE) {
		DMSG("%s, receive FIFO not empty? U2DCSR0=%x\n", __FUNCTION__, U2DCSR0);
		if(U2DCSR0 & U2DCSR0_SA)
			U2DCSR0 = U2DCSR0_SA|U2DCSR0_OPC;
		else
			U2DCSR0 = U2DCSR0_OPC;
	}
#endif	/* EP0_OUT_DMA */

	/* enable U2D */
	U2DCR = U2DCR_MASK;
done:
	u2d_clk_restore();
}

static void stop_activity(struct pxa3xx_u2d *dev, struct gadget_driver_info *pInfo);
/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct pxa3xx_u2d	*dev = the_controller;
	int			retval;

	if (!driver 	|| driver->speed != USB_SPEED_HIGH
			|| !driver->bind
			|| !driver->unbind
			|| !driver->disconnect
			|| !driver->setup
			)
		return -EINVAL;
	if (!dev)
		return -ENODEV;

#ifndef CONFIG_MULTIPLE_GADGET_DRIVERS
	if (dev->driver)
		return -EBUSY;
#endif

#ifdef CONFIG_DVFM
	if (dvfm_out_status(dev))
		return -EAGAIN;
#endif
	out_d0cs = 1;

	/* FIXME remove all modules before insert again */
	if ((dev->rm_flag) && dev->first_gadget) {
		printk(KERN_ERR "left modules may not work!  "
			"please remove all and insert again!!!\n");
		return -EBUSY;
	}

#if defined(CONFIG_CPU_PXA300) && defined(CONFIG_MACH_ZYLONITE)
	/* The pins of U2D are conflict with camera.
	 * When U2D gadget driver are loaded into kernel,
	 * camera should be disabled first.
	 */
	pxa3xx_gpio_set_direction(MFP_UTMI_SWITCH, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_UTMI_SWITCH, GPIO_LEVEL_HIGH);

	pxa3xx_gpio_set_direction(MFP_UTMI_TEST_EN, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_UTMI_TEST_EN, GPIO_LEVEL_LOW);
#endif

	pxa3xx_enable_u2d_pins();
#ifdef CONFIG_CPU_PXA310
	enable_oscc_pout();
#endif
	u2d_clk_set(1);

	cable_detect_init();
	
	local_irq_disable();
	__u2d_disable(dev);
	stop_gadget(dev);
   	local_irq_enable();

	pr_debug("register gadget driver \n");
	/* allocate gadget_driver_info and attach it to controller */
	gadget_info_init(driver);

	driver->driver.bus = NULL;

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	if (dev->driver_count == 0) {
		retval = device_add(&dev->gadget.dev);
		if (retval) {
			printk(KERN_ERR "cannot add device, ret: %d\n",
			       retval);
			goto err1;
		}

		retval = device_create_file(dev->dev, &dev_attr_function);
		if (retval) {
			printk(KERN_ERR "create device file failed, ret: %d\n",
			       retval);
			goto err2;
		}

		if (u2d_bugs & U2D_FIX_ULPI_STP) {
			int irq = IRQ_GPIO(MFP2GPIO(MFP_ULPI_INT));
			retval = request_irq(irq, ulpi_dat3_irq, 0, 
				"ULPI DAT3 gpio edge detect", (void *)dev);
			if (retval) {
				printk("ULPI: request DAT3 irq failed\n");
				return -EBUSY;
			}
			disable_irq(irq);
			dev->stp_gpio_irq_en = 0;
			dev->ulpi_dat3_work = 0;
			
			/* sprintf(dev->stp_work_name, "ULPI DAT3 work queue");
		        dev->stp_work_queue = create_workqueue(dev->stp_work_name);*/
		}
	}

	retval = driver->bind(&dev->gadget); /* will set_gadget_data */
	if (retval) {
		printk(KERN_ERR "bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		goto err3;
	}
	dev->active_gadget->driver_data = get_gadget_data(&dev->gadget);

	gadget_get_device_desc();

	/* default get FS functions */
	gadget_get_config_desc();

	/* configure the endpoint FIFO allocation in 8K SRAM */
	u2d_eps_config(1);

	/* get HS functions */
	gadget_get_config_desc_hs();

	check_fifo = 0;
	if(u2d_bugs & U2D_BUG_INMASS) {
		struct usb_device_descriptor * desc =
			(struct usb_device_descriptor *)dev->active_gadget->device_desc;
		if(((desc->idVendor == 0x0525) && (desc->idProduct == 0xa4a2)) ||
			((desc->idVendor == 0x0525) && (desc->idProduct == 0xa4a1)) ||
			((desc->idVendor == 0x049f) && (desc->idProduct == 0x505a)) ){
				check_fifo = 1;
		}
	}
	if(u2d_bugs & U2D_BUG_STALL) {
		skip_ep_num = 4;
	}

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 * NOTE:  this shouldn't power up until later.
	 */
	DMSG("registered gadget driver '%s'\n", driver->driver.name);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif

	if (is_cable_attached()) {
		if (!is_current_624Mhz() || d0cs)
			ipm_notify();

		if (!d0cs) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			pxa3xx_otg_require_bus(USBOTG_VBUS_VALID);
#else
			__u2d_enable(dev);
#endif
		}
	}

#ifdef U2D_SOFT_DISCON
	else {
		u2d_soft_dis(1);
	}
#endif

#ifndef CABLE_DETECT_GPIO
#ifdef CONFIG_CPU_PXA300
	__u2d_enable(dev);
#endif
	u2d_soft_dis(0);
#endif
	out_d0cs = 0;
	dump_state(dev);

#ifdef CONFIG_CPU_PXA310
	if (!is_cable_attached()) 
		ulpi_set_phymode(LOWPOWER);
#endif
	dev->driver_count ++;
	return 0;
err3:
	if (dev->driver_count == 0) {
		device_remove_file(dev->dev, &dev_attr_function);
	}
err2:
	if (dev->driver_count == 0) {
		device_del(&dev->gadget.dev);
	}
err1:
	gadget_info_uninit(driver);
	dev->driver = 0;
	dev->gadget.dev.driver = 0;

	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

static void
stop_activity(struct pxa3xx_u2d *dev, struct gadget_driver_info *pInfo)
{
	int i;

	DMSG("Trace path 1\n");
	
	u2d_clk_enable();

	/* don't disconnect drivers more than once */
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < U2D_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];

		nuke(ep, -ESHUTDOWN);
		ep->stopped = 1;
	}
	del_timer_sync(&dev->timer);

	if(!pInfo->stopped && pInfo->driver->disconnect)
		pInfo->driver->disconnect(&dev->gadget);
	pInfo->stopped = 1;

	/* re-init driver-visible data structures */
	u2d_reinit(dev);

	DMSG("%s end\n", __func__);
	u2d_clk_restore();
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	int i;
	struct pxa3xx_u2d	*dev = the_controller;
#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info * pInfo = dev->first_gadget;
	struct gadget_driver_info * info = NULL;
#ifdef MULTI_P4
	struct t_str_id *pStr = dev->str_id, *str;
#endif

	if (!dev)
		return -ENODEV;

	do{
		/* find the gadget driver info to pInfo */
		if(pInfo->driver == driver) {
			/* for the first driver is being removed*/
			if(!info) info=pInfo->next;
			break;
		}
		/* save the previous one */
		info=pInfo;
		pInfo=pInfo->next;
	}while(pInfo);

	if(NULL==pInfo) {
		printk(KERN_ERR"%s, can't find driver!\n", __FUNCTION__);
		return -EINVAL;
	}

#ifdef CONFIG_DVFM
	if (dvfm_out_status(dev))
		return -EAGAIN;
#endif
	out_d0cs = 1;

	u2d_clk_set(1);

	/* clear all non-zero EPs configurations */
	u2d_eps_reset();

	local_irq_disable();
	if(dev->driver_count == 1) {
		u2d_soft_dis(1);
		mdelay(6);
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
		otg_set_peripheral(dev->transceiver, NULL);
#endif
		__u2d_disable(dev);
		cable_detect_deinit();
		xcvr_init = 1;
	}
	set_gadget_data(&dev->gadget, pInfo->driver_data);
	stop_activity(dev, pInfo);
	local_irq_enable();

	driver->unbind(&dev->gadget);

	/* put the active one to the previous one */
	if(dev->first_gadget == pInfo){
		if(info) dev->first_gadget = info;
		else dev->first_gadget = pInfo->next;
	}
	if(dev->active_gadget == pInfo){
		if(info) dev->active_gadget = info;
		else dev->active_gadget = pInfo->next;
	}
	if((info) && (info!=pInfo->next))info->next = pInfo->next;


	if(dev->active_gadget) dev->driver = dev->active_gadget->driver;
	else dev->driver = 0; /* no drivers left */

	/* del the gadget abstract device */
	if(dev->driver_count == 1){
		device_del (&dev->gadget.dev);
		device_remove_file(dev->dev, &dev_attr_function);
	}

	for (i=pInfo->ep_start; i<(pInfo->ep_start+pInfo->num_eps);i++){
		struct pxa3xx_ep *ep = &dev->ep[i];
		ep->assigned = 0;
		ep->desc = NULL;
		kfree(ep->ep.name);
	}

	/* free the gadget_driver_info struct */
	kfree(pInfo);

#ifdef MULTI_P4
	while(pStr){
		str = pStr;
		pStr = str->next;
		kfree(str);
	}
	dev->str_id = NULL;
#endif

	dev->driver_count--;
	num_ep_used -= pInfo->num_eps;
	dev->interface_count -= pInfo->num_intfs;

	memset(dev->configs, 0, MAX_CONFIG_LENGTH);
	if(dev->driver_count != 0) {
		dev->rm_flag = 1;
		printk(KERN_WARNING "left modules may not work!  "
			"please remove all and insmod again!!!\n");
	} else {
		dev->rm_flag = 0;
		/* When all modules are removed, disable the USB 2.0 clock */
		u2d_clk_set(0);
		if (u2d_bugs & U2D_FIX_ULPI_STP) {
			int irq = IRQ_GPIO(MFP2GPIO(MFP_ULPI_INT));
			free_irq(irq, (void *)dev);
			/*destroy_workqueue(dev->stp_work_queue);*/
			if (dev->u2d_irq_dis) {
				enable_irq(IRQ_USB2);
				dev->u2d_irq_dis = 0;
			}
		}
	}

#else
	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	local_irq_disable();
	__u2d_disable(dev);
	stop_activity(dev, driver);
	local_irq_enable();

	u2d_clk_set(0);
#ifdef CONFIG_CPU_PXA310
	disable_oscc_pout();
#endif
	driver->unbind(&dev->gadget);
	dev->driver = 0;

	device_del (&dev->gadget.dev);
	device_remove_file(dev->dev, &dev_attr_function);

	for (i = 1; i < U2D_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];
		ep->assigned = 0;
		kfree(ep->ep.name);
	}
#endif

	out_d0cs = 0;
	DMSG("unregistered gadget driver '%s'\n", driver->driver.name);
	/* dump_state(dev); */
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

#ifndef	enable_disconnect_irq
#define	enable_disconnect_irq()		do {} while (0)
#define	disable_disconnect_irq()	do {} while (0)
#endif


/*-------------------------------------------------------------------------*/

static inline void clear_ep_state (struct pxa3xx_u2d *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 1; i < U2D_EP_NUM; i++)
		nuke(&dev->ep[i], -ECONNABORTED);
}

/*
 * Though FST will be cleared, the U2D will continue to respond to
 * subsequent accesses to Endpoint 0 with a Stall handshake until an
 * access to Endpoint 0 occurs with a SETUP packet id (that is, until a new
 * set-up transfer is initiated.)
*/
static void u2d_watchdog(unsigned long _dev)
{	struct pxa3xx_u2d	*dev = the_controller;
	struct pxa3xx_ep	*ep = &dev->ep[1];
	struct pxa3xx_request *req;
	unsigned csr, count = 1000;
	unsigned long flags;

	local_irq_save(flags);

	/*	check whether FIFO is empty
	*/
	if(ep->dir_in && check_fifo){
		do{
			csr = U2DCSR(ep->ep_num);
			if(csr & U2DCSR_BNF) break;
			count--;
		}while(count);

		if(count <=0) {
			DMSG("%s, IN fifo not empty!!!\n", __FUNCTION__);
			start_watchdog(ep->dev);
			goto done;
		}
	}

 	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		req = list_entry(ep->queue.next, struct pxa3xx_request, queue);
		kick_dma(ep, req);
	}
done:
	local_irq_restore(flags);
}

#ifdef CONFIG_USB_COMPOSITE
/* string desc not supported yet */
#define U2D_STRING_MANUFACTURER	0
#define U2D_STRING_PRODUCT		0

static struct usb_device_descriptor
usb_device_desc = {
	.bLength =		sizeof usb_device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16 (0x0200),
						/* USB_CLASS_COMM, for rndis */
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.bMaxPacketSize0 = EP0_MPS,
	.idVendor =		__constant_cpu_to_le16 (U2D_VENDOR_NUM),
	.idProduct =		__constant_cpu_to_le16 (U2D_PRODUCT_NUM),
	.iManufacturer =	U2D_STRING_MANUFACTURER,
	.iProduct =		U2D_STRING_PRODUCT,
	.bNumConfigurations =	1,
};

static struct usb_qualifier_descriptor
usb_qualifier_desc = {
	.bLength =		sizeof usb_qualifier_desc,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,

	.bcdUSB =		__constant_cpu_to_le16 (0x0200),
						/* USB_CLASS_COMM, for rndis */
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.bMaxPacketSize0 = EP0_MPS,
	.bNumConfigurations =	1,
};


static void u2d_setup_complete (struct usb_ep *ep,
			struct usb_request *req)
{
	if (req->status || req->actual != req->length)
		DMSG ("pseudo setup complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);
}

#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS
static int u2d_do_specific_requests(struct usb_ctrlrequest *ctrl,
		struct gadget_driver_info ** gadget_info)
{	struct pxa3xx_u2d *dev = the_controller;
	struct gadget_driver_info * pInfo = dev->first_gadget;

	if(((ctrl->bRequestType==0x21)&&(ctrl->bRequest==0x00)) ||
			((ctrl->bRequestType==0xa1)&&(ctrl->bRequest==0x01))){
		do{
			if(pInfo){
				if(strcmp(pInfo->driver->driver.name, "ether")==0) {
					break;
				}
			}
			pInfo=pInfo->next;
		}while(pInfo);
		if(pInfo == NULL) {
			printk(KERN_ERR "%s, eth not found????\n", __func__);
			return -1;
		}
		else {
			set_gadget_data(&dev->gadget, pInfo->driver_data);
		}
	}

	if(((ctrl->bRequestType==0xa1)&&(ctrl->bRequest==0xfe)) ||
		((ctrl->bRequestType==0x21)&&(ctrl->bRequest==0xff))){
		pInfo = dev->first_gadget;
		while((strcmp(pInfo->driver->driver.name,
						"g_file_storage") && pInfo) ){
			pInfo=pInfo->next;
		}
		if(pInfo == NULL) {
			printk(KERN_ERR "%s, mass not found????\n", __func__);
			return -1;
		}
		else set_gadget_data(&dev->gadget, pInfo->driver_data);
	}

	if(((ctrl->bRequestType==0xa1)&&(ctrl->bRequest==0x21)) ||
		((ctrl->bRequestType==0x21)&&(ctrl->bRequest==0x20)) ||
		((ctrl->bRequestType==0x21)&&(ctrl->bRequest==0x22))){
		pInfo = dev->first_gadget;
		while((strcmp(pInfo->driver->driver.name,
						"g_serial") && pInfo) ){
			pInfo=pInfo->next;
		}
		if(pInfo == NULL) {
			printk(KERN_ERR "%s, serial not found????\n", __func__);
			return -1;
		}
		else set_gadget_data(&dev->gadget, pInfo->driver_data);
	}

	*gadget_info = pInfo;
	return 0;
}
#endif

#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS
static int u2d_do_request(struct usb_ctrlrequest *ctrl, struct pxa3xx_ep *ep)
{	struct pxa3xx_u2d *dev = the_controller;
	struct usb_request *usb_req = &dev->ep0_req.req;
	int value = -EOPNOTSUPP;
	int pseudo = 0, ret = 0;

	DMSG("%s, ctrl->bRequest: 0x%x, ctrl->bRequestType: 0x%x, "
		"ctrl->wValue: 0x%x\n", __FUNCTION__, ctrl->bRequest,
		ctrl->bRequestType, ctrl->wValue);
	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;
		switch (ctrl->wValue >> 8) {

		case USB_DT_DEVICE:
			DMSG("%s, get device desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo device desc */
			value = min (ctrl->wLength, (u16) sizeof usb_device_desc);
			memcpy (usb_req->buf, &usb_device_desc, value);
			break;

		case USB_DT_CONFIG:
			DMSG("%s, get conf desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo configuration desc */
			value = combine_configuration(dev->gadget.speed);
			value = min ((int)ctrl->wLength, value);
			memcpy (usb_req->buf, &dev->configs, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			DMSG("%s, get other speed conf desc\n", __FUNCTION__);
			/* send the pseudo configuration desc */
			if(dev->gadget.speed == USB_SPEED_HIGH)
				value = combine_configuration(USB_SPEED_FULL);
			else if(dev->gadget.speed == USB_SPEED_FULL)
				value = combine_configuration(USB_SPEED_HIGH);
			else{
				printk(KERN_ERR "%s, unknown speed, error\n", __FUNCTION__);
				break;
			}
			pseudo = 1;
			value = min ((int)ctrl->wLength, value);
			memcpy (usb_req->buf, &dev->configs, value);
			break;

		case USB_DT_DEVICE_QUALIFIER:
			DMSG("%s, get device qualifier desc\n", __FUNCTION__);
			pseudo = 1;
			/* send the pseudo device desc */
			value = min (ctrl->wLength, (u16) sizeof usb_qualifier_desc);
			memcpy (usb_req->buf, &usb_qualifier_desc, value);
			break;

		default:
			break;
		}
	default:
		break;
	}

	if(pseudo){
		usb_req->length = value;
		usb_req->no_interrupt = 0;
		usb_req->zero = value < ctrl->wLength
				&& (value % ep->ep.maxpacket) == 0;
		usb_req->complete = u2d_setup_complete;

		ret = pxa3xx_ep_queue(&ep->ep, usb_req, GFP_KERNEL);
		if(!(ret == 0)){
			DMSG("%s, ep_queue error = 0x%x", __FUNCTION__, ret);
		}
		return value;
	} else {
		return -1;
	}
}
#endif
#endif

static void handle_ep0 (struct pxa3xx_u2d *dev)
{
	u32			u2dcsr0 = U2DCSR0;
	struct pxa3xx_ep	*ep = &dev->ep [0];
	struct pxa3xx_request	*req;
	union {
		struct usb_ctrlrequest	r;
		u8			raw [8];
		u32			word [2];
	} u;
	u32 i;

	DMSG("%s is called, ep0 state:%d\n", __FUNCTION__, dev->ep0state);

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct pxa3xx_request, queue);

	/* U2D needn't clear stall status by driver,
	   it would clear STALL until next SETUP cmd */
	if (u2dcsr0 & U2DCSR0_SST) {
		nuke(ep, -EPIPE);

		DMSG("%s EP0 in STALL, ep0 state:%d\n", __FUNCTION__, dev->ep0state);

		del_timer(&dev->timer);	/* maybe not necessary ????? */
		ep0_idle(dev);
	}

	/* previous request unfinished?  non-error iff back-to-back ... */
	if ((u2dcsr0 & U2DCSR0_SA) != 0 && dev->ep0state != EP0_IDLE) {
		DMSG("handle_ep0: Setup command again\n");
		nuke(ep, 0);
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	switch (dev->ep0state) {
	case EP0_NO_ACTION:
		printk(KERN_INFO"%s: Busy\n", __FUNCTION__);
		/*Fall through */
	case EP0_IDLE:
		/* late-breaking status? */
		u2dcsr0 = U2DCSR0;
		DMSG("%s EP0_IDLE u2dcsr0:0x%08x, u2dbcr 0x%x\n", __FUNCTION__, u2dcsr0, U2DBCR0);
		/* start control request? */
		if (likely((u2dcsr0 & (U2DCSR0_OPC|U2DCSR0_SA|U2DCSR0_RNE))
				== (U2DCSR0_OPC|U2DCSR0_SA|U2DCSR0_RNE))) {
			int i;

			nuke (ep, -EPROTO);
			u.word [0] = 0;
			u.word [1] = 0;
			/* read SETUP packet */
			for (i = 0; i < 2; i++) {
				if (unlikely(!(U2DCSR0 & U2DCSR0_RNE))) {
bad_setup:
					DMSG("SETUP %d!, U2DBCR0 %x\n", i, U2DBCR0);
					goto stall;
				}
				u.word [i] =  U2DDR0;
			}
			if (unlikely((U2DCSR0 & U2DCSR0_RNE) != 0))
				goto bad_setup;

			u2dcsr0 &= ~U2DCSR0_RNE;
			/* clear OPC */
			U2DCSR0 = u2dcsr0;

			le16_to_cpus (&u.r.wValue);
			le16_to_cpus (&u.r.wIndex);
			le16_to_cpus (&u.r.wLength);

			LED_EP0_ON;

			DBG(DBG_VERBOSE, "SETUP %02x.%02x v%04x i%04x l%04x\n",
				u.r.bRequestType, u.r.bRequest,
				u.r.wValue, u.r.wIndex, u.r.wLength);

			/* cope with automagic for some standard requests. */
			dev->req_std = (u.r.bRequestType & USB_TYPE_MASK)
						== USB_TYPE_STANDARD;
			dev->req_config = 0;
			dev->req_pending = 1;
#if 0
			switch (u.r.bRequest) {
			/* hardware was supposed to hide this */
			case USB_REQ_SET_CONFIGURATION:
			case USB_REQ_SET_INTERFACE:
			case USB_REQ_SET_ADDRESS:
				printk(KERN_ERR "Should not come here\n");
				break;
			}
#endif

			if (u.r.bRequestType & USB_DIR_IN)
				dev->ep0state = EP0_IN_DATA_PHASE;
			else
				dev->ep0state = EP0_OUT_DATA_PHASE;

			if (u.r.wLength == 0 )
				dev->ep0state = EP0_IN_DATA_PHASE;

#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS
		/* when only one driver is registered, do as original */
		if(dev->driver_count == 1) {
			i = dev->driver->setup(&dev->gadget, &u.r);
		}else
		if(dev->driver_count <=0){
			printk(KERN_ERR"%s, error: "
				"dev->driver_count = %d\n", __FUNCTION__, dev->driver_count);
			return;
		}else
		{
			struct gadget_driver_info * pInfo = dev->active_gadget;
			struct gadget_driver_info * pCurInfo = dev->active_gadget;

			i = u2d_do_request(&u.r, ep);

			/* class specfic requests needed to be set up */
			if(i<0){
				if((u.r.bRequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE){
					for(i=1;i<num_ep_used;i++){
						if(u.r.wIndex == dev->ep[i].assigned_interface){
							u.r.wIndex = dev->ep[i].interface;
							if(i) pInfo = dev->ep[i].driver_info;
							if(pInfo == NULL) {
								printk(KERN_ERR"driver not found, wrong req!!!\n");
								pInfo = pCurInfo;
							}
							set_gadget_data(&dev->gadget, pInfo->driver_data);
							break;
						}
					}
				}
				if((u.r.bRequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT){
					i = u.r.wIndex & 0xf;
					if(i) pInfo = dev->ep[i].driver_info;
					if(pInfo == NULL) {
						printk(KERN_ERR"driver not found, wrong req!!!\n");
						pInfo = pCurInfo;
					}
					set_gadget_data(&dev->gadget, pInfo->driver_data);
				}

#ifdef CONFIG_MULTIPLE_GADGET_DRIVERS /* FIXME */
				u2d_do_specific_requests(&u.r, &pInfo);
#endif

				/* for string desc for each interface, not supported yet */
				if((USB_REQ_GET_DESCRIPTOR==u.r.bRequestType) && \
						(USB_DT_STRING == (u.r.wValue >> 8))){
#ifdef MULTI_P4
					struct t_str_id * pStr = dev->str_id;
					int id_num = u.r.wValue&0xff, j;
					for(j=1;j<id_num;j++) {
						if(pStr) pStr = pStr->next;
						else{
							printk(KERN_ERR" string %d not find !\n", id_num);
						}
					}
					pInfo = pStr->driver_info;
					set_gadget_data(&dev->gadget, pInfo->driver_data);
					u.r.wValue = pStr->str_id;
				}
#else
					DMSG("  get string desc, %d, unsupported, "
						"will pass to current driver(info %p)\n", u.r.wValue, pInfo);
				}
#endif
				i = pInfo->driver->setup(&dev->gadget, &u.r);

				if(i < 0){
					pInfo = dev->first_gadget;
					do{
						set_gadget_data(&dev->gadget, pInfo->driver_data);
						i = pInfo->driver->setup(&dev->gadget, &u.r);
						pInfo=pInfo->next;
					}while((i == -EOPNOTSUPP) &&(pInfo));
					if(i == -EOPNOTSUPP)
						DMSG("%s, no correct driver found to respond to the req!!!\n",
									__FUNCTION__);
					set_gadget_data(&dev->gadget, pCurInfo->driver_data);
				}/* if(i) */
			}/* if(!i) */
		}/* if(dev->driver_count == 1) */
#else
			i = dev->driver->setup(&dev->gadget, &u.r);
#endif

			if (i < 0) {
				/* hardware automagic preventing STALL... */
				if (dev->req_config) {
					/* hardware sometimes neglects to tell
					 * tell us about config change events,
					 * so later ones may fail...
					 */
					WARN("config change %02x fail %d?\n",
						u.r.bRequest, i);
					return;
					/* TODO experiment:  if has_cfr,
					 * hardware didn't ACK; maybe we
					 * could actually STALL!
					 */
				}
				DBG(DBG_VERBOSE, "protocol STALL, "
					"u2dcsr0 %02x err %d\n", U2DCSR0, i);
stall:
				/* the watchdog timer helps deal with cases
				 * where udc seems to clear FST wrongly, and
				 * then NAKs instead of STALLing.
				 * 		watchdog not necessary for U2D ?????
				 */
				printk(KERN_DEBUG "\t ep0 stall\n");
				ep0start(dev, U2DCSR0_FST|U2DCSR0_FTF, "stall");
				/* start_watchdog(dev); */	/* del for U2D test */
				dev->ep0state = EP0_STALL;
				LED_EP0_OFF;

			/* deferred i/o == no response yet */
			} else if (dev->req_pending) {
				if (likely(dev->ep0state == EP0_IN_DATA_PHASE
						|| dev->req_std || u.r.wLength))
					ep0start(dev, 0, "defer");
				else
			/* Wait for client to send 0 length ep0 request */
					/* ep0start(dev, UDCCSR0_IPR, "defer/IPR"); */
					ep0start(dev, 0, "defer/IPR");
			}

			/* expect at least one data or status stage irq */
			return;

		} else {
			/* some random early IRQ:
			 * - we acked FST
			 * - IPR cleared
			 * - OPC got set, without SA (likely status stage)
			 */
			U2DCSR0 = u2dcsr0 & (U2DCSR0_SA|U2DCSR0_OPC); /* xjep0 */
		}
		break;
	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		if (u2dcsr0 & U2DCSR0_OPC) {
			U2DCSR0 = U2DCSR0_OPC|U2DCSR0_FTF;
			DBG(DBG_VERBOSE, "ep0in premature status\n");
			if (req)
				done(ep, req, 0);
			if (!dev->req_config)
				ep0_idle(dev);
			else
				printk(KERN_DEBUG "ep0in u2dcsr0 %x\n", u2dcsr0);
		} else /* irq was IPR clearing */ {
			if (req) {
				/* this IN packet might finish the request */
				(void) write_ep0_fifo(ep, req);
			} /* else IN token before response was written */
		}
		break;
	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		i = 0;
		while ((i < 10000) && !(U2DCSR0 & U2DCSR0_OPC))i++;
		if (U2DCSR0 & U2DCSR0_OPC) {

			if (req) {
 				/* this OUT packet might finish the request */
				DMSG("%s, U2DCSR0=%x\n", __func__, U2DCSR0);
				if (read_ep0_fifo(ep, req)){
					done(ep, req, 0);
					ep0start(dev,U2DCSR0_IPR | U2DCSR0_FTF,"zero 1 IN"); /* RNDIS */
				}
				/* else more OUT packets expected */
			} /* else OUT token before read was issued */
		} else /* irq was IPR clearing */ {
			DBG(DBG_VERBOSE, "ep0out premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		}
		break;
	case EP0_STALL:
		U2DCSR0 = U2DCSR0_FST;
		break;
	case EP0_IN_FAKE:
		printk(KERN_ERR"%s: impossible come here\n", __FUNCTION__);
		break;
	}
}

static void handle_ep(struct pxa3xx_ep *ep)
{
	struct pxa3xx_request	*req, *req_next;
	int			completed;
	u32			u2dcsr=0;

	DMSG("%s is called, ep num:%d, in:%d\n", __FUNCTION__, ep->ep_num, ep->dir_in);
	do {
		completed = 0;
		if (likely (!list_empty(&ep->queue))) {
			req = list_entry(ep->queue.next,
					struct pxa3xx_request, queue);
		} else
			req = 0;

#ifdef CONFIG_CPU_PXA310
		DMSG("%s: req:%p, u2disr:0x%x u2disr2:0x%x u2dcsr:0x%x\n", __func__,
				req, U2DISR, U2DISR2, U2DCSR(ep->ep_num));
#else
		DMSG("%s: req:%p, u2disr:0x%x u2dcsr:0x%x\n", __func__,
				req, U2DISR, U2DCSR(ep->ep_num));
#endif
		if (unlikely(ep->dir_in)) {
			u2dcsr = (U2DCSR_SST | U2DCSR_TRN) & U2DCSR(ep->ep_num);

			/* 11.5.6.6, before clear SST, stop DMA */
			if(u2dcsr & U2DCSR_SST) cancel_dma(ep);

			if (unlikely (u2dcsr))
				U2DCSR(ep->ep_num) = u2dcsr;
		} else { /* for out endpoints */
			u2dcsr = (U2DCSR_SST | U2DCSR_TRN) & U2DCSR(ep->ep_num);

			/* 11.5.6.6, before clear SST, stop DMA */
			if(u2dcsr & U2DCSR_SST) cancel_dma(ep);

			/* clear SST & TRN if necessary */
			if (unlikely(u2dcsr))
				U2DCSR(ep->ep_num) = u2dcsr;

#ifdef USE_SP
			if(U2DCSR(ep->ep_num) & U2DCSR_SP)
			{
				unsigned desc_num = 0, dcsr, dcmd, dadr, dcmd2, dadr2;
				unsigned dmach = ep->dma;

				dcsr = U2DMACSR(dmach);
				dcmd = U2DMACMDR(dmach);
				dadr = U2DMADADR(dmach);

			{
				int length;

				dcmd2 = U2DMACMDR(dmach);
				dadr2 = U2DMADADR(dmach);

				if(req){
					/* caculate the length */
					desc_num = ((/*U2DMADADR(dmach)*/dadr2&0xfffffff0) - ep->dma_desc_phys)/16;
					length = ep->ep.maxpacket*desc_num - (dcmd2 & U2DMACMDR_LEN);

					DMSG("req->req.length:%d, req->req.actual:%d, length:%x\n",
					req->req.length, req->req.actual, length);

					if((length <= DMA_BUF_SIZE) && (length >= 0))
						memcpy((char*)req->req.buf + req->req.actual, \
							ep->dma_buf_virt, length);
					else
						printk(KERN_DEBUG "%s,  error length %d, dcsr %x csr %x cmd %x dadr %x\n\n",
							__FUNCTION__, length, U2DMACSR(ep->dma), U2DCSR(ep->dma),
							U2DMACMDR(ep->dma), U2DMADADR(dmach));
					req->req.actual = length;

					/* clear SP & PC */
					if((length >= 0) && (length <= DMA_BUF_SIZE)){
						U2DCSR(ep->ep_num) = U2DCSR_PC | U2DCSR_DME;
					} else {
					}
				}
			}

			if(U2DMACSR(ep->dma) & U2DMACSR_STOPINTR)
				U2DMACSR(ep->ep_num) &= ~U2DMACSR_STOPIRQEN;

			}
#endif
			if (likely(req)) { /* should not come here, tail bytes handled by U2DMA */
				if (req->queue.next != &ep->queue) {
					req_next = list_entry(req->queue.next,
						struct pxa3xx_request, queue);
					kick_dma(ep, req_next);
				}
				done(ep, req, 0);
			} else {
#ifdef CONFIG_CPU_PXA310
				if (ep->ep_num >= U2D_EP_NUM/2) {
					U2DICR2 &= ~(U2DINT(ep->ep_num, U2DINT_PACKETCMP));
					U2DICR2 &= ~(U2DINT(ep->ep_num, U2DINT_SPACKETCMP));
					U2DICR2 &= ~(U2DINT(ep->ep_num, U2DINT_FIFOERR));
				} else
#else
				{
					U2DICR &= ~(U2DINT(ep->ep_num, U2DINT_PACKETCMP));
					U2DICR &= ~(U2DINT(ep->ep_num, U2DINT_SPACKETCMP));
					U2DICR &= ~(U2DINT(ep->ep_num, U2DINT_FIFOERR));
				}
#endif

				U2DCSR(ep->ep_num) = (U2DCSR(ep->ep_num) & (U2DCSR_DME|U2DCSR_FST)) | U2DCSR_FEF;

				U2DCSR(ep->ep_num) = U2DCSR_PC | U2DCSR_DME;
				DMSG("%s: no req for out data\n",
						__FUNCTION__);
			}
		}
		ep->pio_irqs++;
	} while (completed );
}

static void pxa3xx_change_configuration (struct pxa3xx_u2d *dev)
{
	struct usb_ctrlrequest req ;
	struct gadget_driver_info * pInfo = dev->first_gadget;

	req.bRequestType = 0;
	req.bRequest = USB_REQ_SET_CONFIGURATION;
	req.wValue = dev->configuration;
	req.wIndex = 0;
	req.wLength = 0;

	dev->ep0state = EP0_IN_DATA_PHASE;

	dev->req_config = dev->driver_count;
	do{
		set_gadget_data(&dev->gadget, pInfo->driver_data);
#ifndef MULTIPLE_CONFIGURATION
		/* switch to gadget driver's configuration */
		req.wValue = pInfo->config;
#endif
		pInfo->driver->setup(&dev->gadget, &req);
		pInfo->stopped = 0;
		pInfo = pInfo->next;
	}while(pInfo);
}

static void pxa3xx_change_interface (struct pxa3xx_u2d *dev)
{
	struct usb_ctrlrequest  req;
	int active_interface = (U2DCR&U2DCR_AIN)>>U2DCR_AIN_S;
	struct gadget_driver_info * pInfo = NULL;
	int ret, i;

	DMSG("%s, num_ep_used = %d\n", __FUNCTION__, num_ep_used);

	req.bRequestType = USB_RECIP_INTERFACE;
	req.bRequest = USB_REQ_SET_INTERFACE;
	req.wValue = dev->alternate;
	req.wIndex = dev->interface;
	req.wLength = 0;

	/* change the assigned interface to gadget interface */
	for(i=1;i<U2D_EP_NUM;i++){
		if(dev->ep[i].assigned_interface == active_interface){
			DMSG("dev->ep[%d].assigned_interface = %d, dev->ep[i].driver_info=0x%x\n",
				i, dev->ep[i].assigned_interface, (unsigned)(dev->ep[i].driver_info));
			pInfo = dev->ep[i].driver_info;
			req.wIndex = dev->ep[i].interface;
			DMSG("	req.wValue = %d, req.wIndex = %d\n",req.wValue,req.wIndex);
			break;
		}
	}

	if(pInfo == NULL) {
		printk(KERN_ERR "active interface not found, error\n");
	} else {
		dev->driver = pInfo->driver;
		dev->gadget.dev.driver = &((struct usb_gadget_driver *)(pInfo->driver))->driver;

		dev->active_gadget = pInfo;

		set_gadget_data(&dev->gadget, dev->active_gadget->driver_data);

		/* req.wValue = dev->ep[i].interface;
		req.wIndex = dev->ep[i]->aisn; */
		dev->interface = active_interface;

		dev->ep0state = EP0_IN_DATA_PHASE;

		ret = dev->driver->setup(&dev->gadget, &req);
		if(ret == -EOPNOTSUPP) {
			DMSG(" ret EOPNOTSUPP\n");
		}
	}
	return;
}

/*
 *	pxa3xx_u2d_irq - interrupt handler
 *
 * avoid delays in ep0 processing. the control handshaking isn't always
 * under software control (pxa250c0 and the pxa255 are better), and delays
 * could cause usb protocol errors.
 */
static irqreturn_t pxa3xx_u2d_irq(int irq, void *_dev)
{
	struct pxa3xx_u2d *dev = _dev;
	int handled;

	dev->stats.irqs++;
	HEX_DISPLAY(dev->stats.irqs);

	
	DMSG("\n");
	DBG(DBG_VERBOSE, "Interrupt, U2DICR:0x%08x, U2DISR:0x%08x, "
			"U2DCR:0x%08x, U2DMAINT:0x%x\n", U2DICR, U2DISR, U2DCR, U2DMAINT);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	DBG(DBG_VERBOSE, "\tU2DICR2:0x%08x U2DISR2:0x%08x U2DOTGCR:0x%08x,"
			"U2DOTGICR:0x%08x, U2DOTGISR:0x%08x,\n",
			U2DICR2, U2DISR2, U2DOTGCR, U2DOTGICR, U2DOTGISR);
#endif

	do {
		u32 u2dint, i, temp;

#ifndef CONFIG_CPU_PXA310
		if(U2DMAINT & 0xff)	{
#else
		if(U2DMAINT & 0xffff)	{
#endif
			for(i=1;i<U2D_EP_NUM;i++){
				if(U2DMAINT & (0x1<<i)) u2dma_handler(i, &dev->ep[i]);
			}
		}

		u2dint = U2DISR & 0xFE000000;
		handled = 0;

		/* SUSpend Interrupt Request */
		if (unlikely(u2dint & U2DINT_SU)) {
			U2DISR = U2DINT_SU;
			temp = U2DISR;
			handled = 1;
			DBG(DBG_VERBOSE, "USB suspend, u2dcr %x icr %x isr %x fnr %x dmacr %x int %x\n",
				U2DCR, U2DICR, U2DISR, U2DFNR, U2DMACR, U2DMAINT);
			if (dev->gadget.speed != USB_SPEED_UNKNOWN
					&& dev->driver
					&& dev->first_gadget) {
				struct gadget_driver_info * pInfo = dev->first_gadget;

				do{
					set_gadget_data(&dev->gadget, pInfo->driver_data);
					if (pInfo->driver->suspend)
						pInfo->driver->suspend(&dev->gadget);
					pInfo = pInfo->next;
				}while(pInfo);
			}
			ep0_idle (dev);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			otg_host_suspend(dev->transceiver);
			continue;
#endif
		}

		/* RESume Interrupt Request */
		if (unlikely(u2dint & U2DINT_RU)) {
			U2DISR = U2DINT_RU;
			temp = U2DISR;
			handled = 1;
			DBG(DBG_VERBOSE, "USB resume, u2dcr %x icr %x isr %x fnr %x dmacr %x int %x\n",
				U2DCR, U2DICR, U2DISR, U2DFNR, U2DMACR, U2DMAINT);

			if (dev->gadget.speed != USB_SPEED_UNKNOWN
					&& dev->driver
					&& dev->first_gadget) {
				struct gadget_driver_info * pInfo = dev->first_gadget;

				do{
					set_gadget_data(&dev->gadget, pInfo->driver_data);
					if (pInfo->driver->resume)
						pInfo->driver->resume(&dev->gadget);
					pInfo = pInfo->next;
				}while(pInfo);
			}
		}

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
		if (U2DOTGISR) {
			otg_interrupt(dev->transceiver);
			handled = 0;
			break;
		}
#elif	defined(CONFIG_CPU_PXA310)
		u32 otgisr, state;
		if (U2DOTGISR){
			handled = 1;
			otgisr = U2DOTGISR;
			state = ulpi_get_phymode();
			
			DMSG("\tU2DICR2:0x%08x U2DISR2:0x%08x U2DOTGCR:0x%08x,"
				"U2DOTGICR:0x%08x, U2DOTGISR:0x%08x,\n",
				U2DICR2, U2DISR2, U2DOTGCR, U2DOTGICR, U2DOTGISR);
			if (state != SYNCH)
				if (otgisr & U2DOTGINT_SI) {
					U2DOTGISR = U2DOTGINT_SI;
					ulpi_set_phymode(SYNCH);
				}

			otgisr = U2DOTGISR;
			U2DOTGISR = otgisr;
			
			if (otgisr & (U2DOTGINT_RVV | U2DOTGINT_FVV)) {
				DMSG("otgisr: 0x%x, otgusr: 0x%x\n", otgisr, U2DOTGUSR);
				cable_detect_interrupt();
				if (otgisr & U2DOTGINT_FVV) {
					ulpi_set_phymode(LOWPOWER);
					//extra_si = 1;
					handled = 0;
					break;
				} else if (state != SYNCH) {
					ulpi_set_phymode(SYNCH);
				}
					
				//DMSG("U2DOTGISR: 0x%x, otgusr: 0x%x\n", U2DOTGISR, U2DOTGUSR);
				//U2DOTGISR = U2DOTGINT_RVV | U2DOTGINT_FVV | U2DOTGISR_SI;
			}
		}
#endif

		if (unlikely(u2dint & U2DINT_CC)) {
			unsigned config, interface, alternate;

			handled = 1;
			DBG(DBG_VERBOSE, "USB SET_CONFIGURATION or "
				"SET_INTERFACE command received\n");

			config = (U2DCR & U2DCR_ACN) >> U2DCR_ACN_S;
			interface =  (U2DCR & U2DCR_AIN) >> U2DCR_AIN_S;
			alternate = (U2DCR & U2DCR_AAISN) >> U2DCR_AAISN_S;

			DBG(DBG_VERBOSE, "    config=%d,  interface=%d, alternate=%d, u2dcr %x\n",
				config, interface, alternate, U2DCR);

			if (dev->configuration != config) {
				dev->configuration = config;
				pxa3xx_change_configuration(dev) ;
			}
			else if ( (dev->interface != interface) || \
					(dev->interface == 0) || \
					(dev->alternate != alternate)){
				dev->interface = interface;
				dev->alternate = alternate;
				pxa3xx_change_interface(dev);
			}

			U2DCR = (U2DCR & (U2DCR_MASK)) | U2DCR_CC | U2DCR_SMAC;
			DMSG("%s: u2dcr:0x%1x, u2dcsr0:0x%1x\n",
				__FUNCTION__, U2DCR, U2DCSR0);

			while(U2DCR & U2DCR_SMAC);
			U2DISR = U2DINT_CC;
		}

		/* ReSeT Interrupt Request - USB reset */
		if (unlikely(u2dint & U2DINT_RS)) {
			U2DISR = U2DINT_RS;
			handled = 1;

			DBG(DBG_VERBOSE, "USB reset start\n");
			{
				struct gadget_driver_info * pInfo = dev->first_gadget;

				do{
					set_gadget_data(&dev->gadget, pInfo->driver_data);
					/* stop_activity(dev, pInfo); */
					pInfo = pInfo->next;
				}while(pInfo);
			}
			dev->gadget.speed = USB_SPEED_UNKNOWN;

			/* enable SOF/uSOF interrupt to detect bus speed */
			U2DICR |= (U2DINT_SOF|U2DINT_USOF);

			DMSG("USB reset, u2dcsra %x\n", U2DCSR(1));

			memset(&dev->stats, 0, sizeof dev->stats);
			u2d_reinit(dev);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			pxa3xx_otg_reset();
#endif
		}

		/* SOF/uSOF Interrupt, to detect bus speed */
		if (unlikely(U2DISR & (U2DINT_SOF|U2DINT_USOF))) {
			handled = 1;

			DBG(DBG_VERBOSE, "USB SOF/uSOF, u2disr %x, u2dcra %x, u2dcsra %x\n",
				U2DISR, U2DEPCR(1), U2DCSR(1));

			/* clear SOF/uSOF interrupt */
			U2DICR &= ~(U2DINT_SOF|U2DINT_USOF);
			if (U2DISR & U2DINT_USOF)
					U2DISR = U2DINT_USOF;
			if (U2DISR & U2DINT_SOF)
					U2DISR = U2DINT_SOF;

			if(dev->gadget.speed == USB_SPEED_UNKNOWN){
				if(U2DCR & U2DCR_HS){
					dev->gadget.speed = USB_SPEED_HIGH;
				} else{
					dev->gadget.speed = USB_SPEED_FULL;
				}
				}

				/* change the endpoint MPS */
				change_mps(dev->gadget.speed);

			/* set the endpoint information register */
			u2d_eps_config(2);
			DMSG("\t u2dcsra %x\n", U2DCSR(1));
			while(U2DCSR(1) & U2DCSR_PC)
				U2DCSR(1) = U2DCSR_PC | U2DCSR_TRN;
			DMSG("\t u2dcsra %x\n", U2DCSR(1));

		}
#ifdef CONFIG_CPU_PXA310
		if((U2DISR & U2DISR_MASK) || (U2DISR2 & U2DISR_MASK)) {
#else
		if(U2DISR & U2DISR_MASK) {
#endif
			u32	u2disr = U2DISR ;
			int	i;

			DBG(DBG_VERY_NOISY, "irq %02x\n", u2disr);

			/* control traffic */
			if (u2disr & U2DINT_EP0) {
				DMSG("handle_ep0: U2DISR:%x, U2DCSR0:%x\n",\
						U2DISR, U2DCSR0);

				U2DISR = U2DINT(0, U2DINT_MASK);

				if (u2disr & U2DINT_FIFOERR) {
					printk(KERN_WARNING"Endpoint 0 fifo Error\n");
					/* why delete originally????? */
					U2DISR = U2DINT(0, U2DINT_FIFOERR);
				}
				dev->ep[0].pio_irqs++;
				if ((u2disr & U2DINT_PACKETCMP) || (u2disr & U2DINT_SPACKETCMP)) {
					handle_ep0(dev);
					handled = 1;
				}
				/* why delete originally????? */
				/* UDCISR0 = UDCISR_INT(0, UDCISR_INT_MASK); */
			}

			u2disr >>= 3;
			/* endpoint data transfers */
			for (i = 1; u2disr!=0 && i < U2D_EP_NUM; u2disr>>=3,i++) {
#ifdef CONFIG_CPU_PXA310
				if(i >= U2D_EP_NUM/2) {
					u2disr = U2DISR2;
					U2DISR2 = U2DINT((i-U2D_EP_NUM/2), U2DINT_MASK);
				} else {
					U2DISR = U2DINT(i, U2DINT_MASK);
				}
#else
				U2DISR = U2DINT(i, U2DINT_MASK);
#endif

				if (u2disr & U2DINT_FIFOERR){
#ifdef DEBUG
					led_on();
#endif
					printk(KERN_WARNING" Endpoint %d Fifo error, csr:%x dcsr:%x da %x dcmd %x\n",
						i, U2DCSR(i), U2DMACSR(i), U2DMADADR(i), U2DMACMDR(i));
#ifdef CONFIG_CPU_PXA310
					if(i >= U2D_EP_NUM/2)
						U2DICR2 &= ~(U2DINT((i-U2D_EP_NUM/2), U2DINT_FIFOERR));
					else
						U2DICR &= ~(U2DINT(i, U2DINT_FIFOERR));
#else
					U2DICR &= ~(U2DINT(i, U2DINT_FIFOERR));
#endif

					if(U2DCSR(i) & U2DCSR_TRN)
						U2DCSR(i) = U2DCSR_DME|U2DCSR_TRN;
					U2DMACSR(i) &= ~U2DMACSR_RUN;
					U2DCSR(i) = U2DCSR_FEF | U2DCSR_PC;
				}

				if ((u2disr & U2DINT_PACKETCMP) \
					|| (u2disr & U2DINT_FIFOERR)){
					handle_ep(&dev->ep[i]);
					handled = 1;
				}
			}
		}
		/* we could also ask for 1 msec SOF (SIR) interrupts */
	} while (handled);

	DMSG("IRQ_HANDLED\n");
	return IRQ_HANDLED;
}

static void u2d_init_ep(struct pxa3xx_u2d *dev)
{
	int i;

	INIT_LIST_HEAD (&dev->gadget.ep_list);
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);

	for (i = 0; i < U2D_EP_NUM; i++) {
		struct pxa3xx_ep *ep = &dev->ep[i];

		ep->dma = -1;
		if (i != 0) {
			memset(ep, 0, sizeof(*ep));
		}
		INIT_LIST_HEAD (&ep->queue);
	}
}

#define NAME_SIZE 120

struct usb_ep* pxa3xx_ep_config(struct usb_gadget *gadget,
		struct usb_endpoint_descriptor *desc)
{
	unsigned i;
	char* name;
	struct usb_ep * ep = NULL;
	struct pxa3xx_ep *pxa_ep = NULL;
	struct pxa3xx_u2d *dev = the_controller;

	DMSG("pxa3xx_config_ep is called\n");
	DMSG(" usb endpoint descriptor is:\n"
		"	bLength:%d\n"
		"	bDescriptorType:%x\n"
		"	bEndpointAddress:%x\n"
		"	bmAttributes:%x\n"
		"	wMaxPacketSize:%d\n",
		desc->bLength,
		desc->bDescriptorType,desc->bEndpointAddress,
		desc->bmAttributes,desc->wMaxPacketSize);

	for (i = 1; i < U2D_EP_NUM; i++) {
		/* FIXME, does LV fix this bug */
		if(i==skip_ep_num) continue;
		if(!dev->ep[i].assigned) {
			pxa_ep = &dev->ep[i];
			pxa_ep->assigned = 1;
			pxa_ep->ep_num = i;
			break;
		}
	}
	if (unlikely(i == U2D_EP_NUM)) {
		printk(KERN_ERR"%s:Failed to find a spare endpoint\n",__FILE__);
		return NULL;
	}

	ep = &pxa_ep->ep;

	pxa_ep->dev = dev;
	pxa_ep->desc = desc;
	pxa_ep->pio_irqs = pxa_ep->dma_irqs = 0;
	pxa_ep->dma = -1;

	if (!(desc->bEndpointAddress & 0xF))
		desc->bEndpointAddress |= i;

	pxa_ep->dir_in = (desc->bEndpointAddress & USB_DIR_IN) ? 1 : 0;
	pxa_ep->ep_type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	pxa_ep->stopped = 1;
	pxa_ep->dma_con = 0;

	/* U2DMA related operations */
	pxa_ep->dma = i;

	/* Fill ep name*/
	name = kzalloc(NAME_SIZE, GFP_KERNEL);
	if (!name) {
		printk(KERN_ERR "%s: Error\n", __FUNCTION__);
		return NULL;
	}

	switch (pxa_ep->ep_type) {
	case USB_ENDPOINT_XFER_BULK:
		sprintf(name, "%s-bulk-%s-%d", gadget->dev.driver->name,
				(pxa_ep->dir_in ? "in":"out"), i);
		pxa_ep->fifo_size = BULK_FIFO_SIZE;
		pxa_ep->ep.maxpacket = BULK_MPS(dev->gadget.speed);
		break;
	case USB_ENDPOINT_XFER_INT:
		sprintf(name, "%s-interrupt-%s-%d", gadget->dev.driver->name,
				(pxa_ep->dir_in ? "in":"out"), i);
		pxa_ep->fifo_size = INT_FIFO_SIZE;
		pxa_ep->ep.maxpacket = INT_MPS(dev->gadget.speed);
		pxa_ep->hs_cmds = INT_HS_CMDS;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		sprintf(name, "%s-iso-%s-%d", gadget->dev.driver->name,
				(pxa_ep->dir_in ? "in":"out"), i);
		pxa_ep->fifo_size = ISO_FIFO_SIZE;
		pxa_ep->ep.maxpacket = ISO_MPS(dev->gadget.speed);
		pxa_ep->hs_cmds = ISO_HS_CMDS;
		break;
	default:
		sprintf(name, "endpoint-%s-%d", (pxa_ep->dir_in ? \
				"in":"out"), i);
		break;
	}

	if (!(desc->wMaxPacketSize)) {
		/* always FS MPS here */
		desc->wMaxPacketSize = pxa_ep->ep.maxpacket;
	}

	ep->name = name;

	ep->ops = &pxa3xx_ep_ops;
	ep->maxpacket = min((ushort)pxa_ep->fifo_size, desc->wMaxPacketSize);

	list_add_tail (&ep->ep_list, &gadget->ep_list);

	dev->active_gadget->num_eps++;
	num_ep_used++;
	DMSG("		ep->ep_num = 0x%x\n", pxa_ep->ep_num);
	DMSG("		num_ep_used = %d\n", num_ep_used);

	return ep;
}

EXPORT_SYMBOL(pxa3xx_ep_config);

/*-------------------------------------------------------------------------*/

static void nop_release (struct device *dev)
{
	DMSG("%s %s\n", __FUNCTION__, dev->bus_id);
}

/* this uses load-time allocation and initialization (instead of
 * doing it at run-time) to save code, eliminate fault paths, and
 * be more obviously correct.
 */
static struct pxa3xx_u2d memory = {
	.gadget = {
		.ops		= &pxa3xx_u2d_ops,
		.ep0		= &memory.ep[0].ep,
		.name		= driver_name,
		.dev = {
			.bus_id		= "gadget",
			.release	= nop_release,
		},
	},

	/* control endpoint */
	.ep[0] = {
		.ep = {
			.name		= ep0name,
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= EP0_FIFO_SIZE,
		},
		.dev		= &memory,
	}
#ifdef CONFIG_DVFM
    	,
	.dvfm_notifier = {
		.name = "pxa3xx-u2d",
		.priority = 0,
		.notifier_call = u2d_dvfm_notifier,
	},
#endif
};

#ifdef DEBUG
#define CP15R0_VENDOR_MASK	0xffffe000
#define CP15R0_XSCALE_VALUE	0x69054000	/* intel/arm/xscale */
#endif

/*
 * 	probe - binds to the platform device
 */
static int __init pxa3xx_u2d_probe(struct platform_device *pdev)
{
	struct pxa3xx_u2d *dev = &memory;
	struct device *_dev = &pdev->dev;
	int retval;

	/* other non-static parts of init */
	dev->dev = _dev;
	dev->mach = _dev->platform_data;

	init_timer(&dev->timer);
	dev->timer.function = u2d_watchdog;
	dev->timer.data = (unsigned long)dev;

	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = _dev;
	dev->gadget.dev.dma_mask = _dev->dma_mask;

	dev->gadget.is_dualspeed = 1;

	the_controller = dev;
	platform_set_drvdata(pdev, dev);

	/* __u2d_disable(dev); */
	u2d_init_ep(dev);
	u2d_reinit(dev);

	u2d_bugs = u2d_bug_check();

	/* reset the transceiver */
	reset_xcvr();

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	dev->gadget.is_otg = 1;
	dev->transceiver = otg_get_transceiver();
	if (!dev->transceiver) {
		DMSG("failed to get transceiver\n");
	}
#endif

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_USB2, pxa3xx_u2d_irq,
			SA_INTERRUPT, driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
			driver_name, IRQ_USB2, retval);
		return -EBUSY;
	}

	dev->got_irq = 1;
	dev->u2d_irq_dis = 0;

	create_proc_files();

#ifdef U2D_USE_ISRAM
	zylu2d_immid = imm_register_kernel("pxa3xx_u2d");
#endif

#ifdef CONFIG_DVFM
	dev->dvfm_notifier.client_data = NULL;
	pxa3xx_fv_register_notifier(&dev->dvfm_notifier);
	init_waitqueue_head(&dev->delay_wait);
#endif

	return 0;
}

static int pxa3xx_u2d_remove(struct platform_device *pdev)
{
	struct pxa3xx_u2d *dev = platform_get_drvdata(pdev);

	remove_proc_files();
	/* usb_gadget_unregister_driver(dev->driver); */

	if (dev->got_irq) {
		free_irq(IRQ_USB2, dev);
		dev->got_irq = 0;
		dev->u2d_irq_dis = 1;
	}
	if (machine_is_lubbock() && dev->got_disc) {
		free_irq(LUBBOCK_USB_DISC_IRQ, dev);
		dev->got_disc = 0;
	}
	platform_set_drvdata(pdev, NULL);
	the_controller = 0;

	if (machine_is_zylonite()) {
		free_irq(IRQ_USB2, dev);
		dev->u2d_irq_dis = 1;
	}
	
#ifdef CONFIG_DVFM
	pxa3xx_fv_unregister_notifier(&dev->dvfm_notifier);
#endif
	d0cs = 0;

	return 0;
}

static void stop_gadget(struct pxa3xx_u2d *dev)
{
	struct gadget_driver_info *pInfo = dev->first_gadget;

	while (pInfo) {
		set_gadget_data(&dev->gadget, pInfo->driver_data);
		stop_activity(dev, pInfo);
		pInfo = pInfo->next;
	}
}

/*
 * Interrupt comes from GPIO when usb cable attached or detached
 */
static void u2d_stop(int state)
{
#ifdef CONFIG_USB_COMPOSITE
	struct pxa3xx_u2d *dev = &memory;
	struct gadget_driver_info * pInfo = dev->first_gadget;

	if(dev->driver){
		if(state == 0){
			do {
				pInfo->stopped = 0;
				pInfo = pInfo->next;
			} while (pInfo);
		} else if (state == 1) {
			stop_gadget(dev);
		} else
			printk(KERN_ERR "stop state %d error\n", state);

	}
#endif
}

#ifdef CONFIG_PM
static void u2d_save_state(struct pxa3xx_u2d *dev)
{
	struct pxa3xx_ep *ep = NULL;
	int i;
	dev->u2dcr = U2DCR;
	dev->u2dicr = U2DICR;
	dev->u2dcsr0 = U2DCSR0;
	for (i = 1; i < U2D_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			ep = &dev->ep[i];
			ep->u2dcsr_value = U2DCSR(ep->ep_num);
			ep->u2dcr_value = U2DEPCR(ep->ep_num);
			ep->u2denr_value = U2DEN(ep->ep_num);
			DMSG("EP%d, u2dcsr:0x%x, u2dcr:0x%x, u2denr:0x%x\n",
				i, U2DCSR(ep->ep_num), U2DEPCR(ep->ep_num),
				U2DEN(ep->ep_num));
		}
	}
#ifdef CONFIG_CPU_PXA310
	dev->u2dotgcr = U2DOTGCR;
	dev->u2dotgicr = U2DOTGICR;
#endif
}

static void u2d_restore_state(struct pxa3xx_u2d *dev)
{
	struct pxa3xx_ep *ep = NULL;
	int i;
	
	U2DCSR0 = dev->u2dcsr0 & (U2DCSR0_FST | U2DCSR0_DME);
	U2DICR = dev->u2dicr;
	U2DCR = dev->u2dcr;
	for (i = 1; i < U2D_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			ep = &dev->ep[i];

			U2DCSR(ep->ep_num) = ep->u2dcsr_value;
			U2DEPCR(ep->ep_num) = ep->u2dcr_value;
			U2DEN(ep->ep_num)  = ep->u2denr_value;
			printk(KERN_DEBUG"EP%d, u2dcsr:0x%x, u2dcr:0x%x\n",
				i, U2DCSR(ep->ep_num), U2DEPCR(ep->ep_num));
		}
	}

#ifdef CONFIG_CPU_PXA310
	U2DOTGCR = dev->u2dotgcr;
	U2DOTGICR = dev->u2dotgicr;
#endif
}

static int pxa3xx_u2d_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa3xx_u2d *dev = (struct pxa3xx_u2d*)platform_get_drvdata(pdev);

	if (dev->driver) {
		if (dev->stp_gpio_irq_en) {
			int irq = IRQ_GPIO(MFP2GPIO(MFP_ULPI_INT));
			disable_irq(irq);
			dev->ulpi_dat3_work = 1;
			ulpi_dat3_work();
			dev->stp_gpio_irq_en = 0;
		}
		if (is_cable_attached()) {
			printk(KERN_ERR "Can't make system into suspend " \
					"state when USB cable is attached\n");
			return -EACCES;
		}
		u2d_save_state(dev);
		stop_gadget(dev);
		/* soft disconnect the device & disable UDE */
		if (dev->driver)
			u2d_soft_dis(1);
#ifndef U2D_SOFT_DISCON
		U2DCR &= ~U2DCR_UDE;
#endif

#ifdef CONFIG_USB_OTG
		otg_set_peripheral(dev->transceiver,NULL);
#endif 
		u2d_irq_set(0);
		u2d_clk_set(0);
	}

	return 0;
}

static int pxa3xx_u2d_resume(struct platform_device *pdev)
{
	struct pxa3xx_u2d *dev = (struct pxa3xx_u2d*)platform_get_drvdata(pdev);

	if (dev->driver) {
		u2d_clk_set(1);
		u2d_restore_state(dev);
		u2d_irq_set(1);
#ifdef U2D_SOFT_DISCON
		u2d_soft_dis(0);
#else
		U2DCR |= U2DCR_UDE;
#endif
		__u2d_enable(dev);
#ifdef CONFIG_USB_OTG
		otg_set_peripheral(dev->transceiver,&dev->gadget);
#endif
#ifdef CONFIG_CPU_PXA310
		if (!is_cable_attached())
			ulpi_set_phymode(LOWPOWER);
#endif
	}

	return 0;
}

#endif

#ifdef CONFIG_DVFM
/* Is the Fr which the system want to enter below 624Mhz?*/
static int is_op_below_624Mhz(struct pxa3xx_fv_info * info)
{
	if (u2d_bugs & DDR_BUG_DMA) {
		if (info && ((info->xl != 0x18)
			|| (info->xn != 0x2) || (info->d0cs != 0)))
			/* cable attached, and next op is not 624MHz */
			return 1;
	}
	return 0;
}

/* Is the Fr of the system 624Mhz*/
static int is_current_624Mhz(void)
{
	if (u2d_bugs & DDR_BUG_DMA) {
		if (((ACSR & 0x1f) == 0x18) && ((ACSR & 0x700) == 0x200))
		/* cable attached, and cur op is 624MHz */
			return 1;
		else
			return 0;
	}

	return 1;
}

/* for epx, check whether there are data transfered*/
static int is_data_transfering(struct pxa3xx_u2d *dev)
{
       	int i;

       	/* for ep0, check whether there are data transfered*/
       	if ((U2DCSR0 & U2DCSR0_IPR) || (U2DCSR0 & U2DCSR0_RNE))
               	return 1;

       	/* for epA-Z, check whether there are data transfered*/
       	for (i = 1; i < U2D_EP_NUM; i++){
               	if (dev->ep[i].assigned){
                       	if (!(U2DCSR(dev->ep[i].ep_num) & U2DCSR_BE))
                               	return 1;
        	}
       	}

       	return 0;
}

static int check_dvfm_status(struct pxa3xx_fv_notifier_info *notifier_info)
{
        struct pxa3xx_u2d *dev = the_controller;

	/* check whether clock is enabled */
	if ((!(CKENA & (1 << CKEN_USB2)) || !dev->driver) && !out_d0cs)
		return 0;
	
	/* for epx, check whether there are data transfered*/
       	if (is_data_transfering(dev))
               	return -EAGAIN;

        /* check whether clock is enabled */
	if ( (notifier_info->cur.d0cs != 1) || \
		(notifier_info->next.d0cs != 0))
		if(is_op_below_624Mhz(&notifier_info->next) && is_cable_attached())
			return -EAGAIN;

	if ((notifier_info->cur.d0cs == 0)
		&& (notifier_info->next.d0cs == 1)) {
		if (connected || out_d0cs) {
			pr_debug("USB 2.0 Client: unplug "\
				"cable and try D0CS mode again\n");
			return -EFAULT;
		}
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
		if (dev->transceiver->default_a) {
			pr_debug("USB OTG: unplug USB"\
				"cable and try D0CS mode again\n");
			return -EFAULT;
		}
#endif
	} else if ((notifier_info->cur.d0cs == 1)
		&& (notifier_info->next.d0cs == 0)) {
		if (is_cable_attached()) {
			pr_debug("USB 2.0 Client has to "\
				"be re-configured when exiting "\
				"D0CS mode\n");
		}
	}
	return 0;
}

static int
u2d_dvfm_notifier(unsigned int cmd, void* client_data, void *info)
{
	struct pxa3xx_fv_notifier_info *notifier_info = info;
	struct pxa3xx_u2d *dev = the_controller;

	switch (cmd) {
	case FV_NOTIFIER_QUERY_SET :
		return check_dvfm_status(notifier_info);
		break;

	case FV_NOTIFIER_PRE_SET :
		if ((notifier_info->cur.d0cs == 0)
			&& (notifier_info->next.d0cs == 1)) {
			if (dev->driver) {
				u2d_clk_enable();
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
				pxa3xx_otg_require_bus(0);
#endif
				u2d_soft_dis(1);
#ifdef CONFIG_CPU_PXA310
				ulpi_set_phymode(LOWPOWER);
#endif
				__u2d_disable(dev);
				u2d_clk_restore();
			}
			d0cs = 1;
		}
		break;

	case FV_NOTIFIER_POST_SET :
		if (d0cs && ((notifier_info->cur.d0cs == 1)
			&& (notifier_info->next.d0cs == 0))) {
			d0cs = 0;
			wake_up(&dev->delay_wait);
			if (!dev->stp_gpio_irq_en && dev->driver)
				ulpi_dat3_work();
			if (is_cable_attached()) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
				pxa3xx_otg_require_bus(USBOTG_VBUS_VALID |
							USBOTG_SRP_DETECT);
#else
				__u2d_enable(&memory);
				u2d_soft_dis(0);
#endif
				u2d_stop(0);
			}
		}
		break;
	}
	return 0;
}
#else
static int is_op_below_624Mhz(int a)
{
	return 0;
}

static int is_current_624Mhz(void)
{
	return 1;
}
#endif

/***************************************************************************/
static struct platform_driver u2d_driver = {
	.driver	= {
		.name	= "pxa3xx-u2d",
	},
	.probe		= pxa3xx_u2d_probe,
	.remove		= pxa3xx_u2d_remove,
#ifdef CONFIG_PM
	.suspend	= pxa3xx_u2d_suspend,
	.resume		= pxa3xx_u2d_resume,
#endif
};

static int __init u2d_init(void)
{
	return platform_driver_register(&u2d_driver);
}

static void __exit u2d_exit(void)
{
	platform_driver_unregister(&u2d_driver);
}

module_init(u2d_init);
module_exit(u2d_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Frank Becker, Robert Schwebel, David Brownell");
MODULE_LICENSE("GPL");



