/*
 * linux/drivers/usb/gadget/pxa3xx_u2d.h
 * PXA3xx on-chip high speed USB device controller
 *
 * Copyright (C) 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2006 Intel Corporation
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
 */


#ifndef __LINUX_USB_GADGET_PXA3xx_U2D_H
#define __LINUX_USB_GADGET_PXA3xx_U2D_H

#include <linux/types.h>
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>

#define CONFIG_USB_COMPOSITE

#ifdef CONFIG_USB_COMPOSITE
#undef MULTIPLE_CONFIGURATION		/* FIXME, RNDIS + CDC */
/* couldn't work in one module both with linux and windows machine */
#undef MULTI_P4				/* FIXME, string descriptor for interface support */
#define MULTI_P3			/*  add iad descriptor */
#define U2D_DEFAULT_CONFIG	1	/* FIXME for rndis */
#define U2D_VENDOR_NUM 		0x1286
#define U2D_PRODUCT_NUM 	0x8002
#endif

#ifdef CONFIG_CPU_PXA310
#undef CABLE_DETECT_GPIO
#else
#define CABLE_DETECT_GPIO
#endif

enum u2d_bug_type {
	U2D_BUG_NONE = 0,
	U2D_BUG_INMASS = 0x1,
	U2D_BUG_SETINTF = 0x2,
	U2D_BUG_STALL = 0x4,
	DDR_BUG_DMA = 0x8,
	U2D_BUG_UTMID = 0x10,
	U2D_FIX_ULPI_STP = 0x20,
};

#ifdef CONFIG_CPU_PXA310
enum u2d_phy_mode {
	SYNCH = 0,
	CARKIT = 0x1,
	SER_3PIN = 0x2,
	SER_6PIN = 0x4,
	LOWPOWER = 0x8,
	/* rtsm mode */
	PRE_SYNCH = 0x10,
};

#define ULPI_VENDOR_LOW 			0x0
#define ULPI_VENDOR_HIGH 			0x1
#define ULPI_PRODUCT_LOW 			0x2
#define ULPI_PRODUCT_HIGH 			0x3
#define ULPI_FUNCTION_CONTROL 			0x4
#define ULPI_FUNCTION_CONTROL_CLEAR 		0x6
#define ULPI_FUNCTION_CONTROL_SET 		0x5
#define ULPI_INTERFACE_CONTROL 			0x7
#define ULPI_INTERFACE_CONTROL_SET 		0x8
#define ULPI_INTERFACE_CONTROL_CLEAR 		0x9
#define ULPI_OTG_CONTROL 			0xA
#define ULPI_OTG_CONTROL_SET 			0xB
#define ULPI_OTG_CONTROL_CLEAR 			0xC
#define ULPI_INT_RISE 				0xD
#define ULPI_INT_RISE_SET 			0xE
#define ULPI_INT_RISE_CLEAR 			0xF
#define ULPI_INT_FALL 				0x10
#define ULPI_INT_FALL_SET 			0x11
#define ULPI_INT_FALL_CLEAR 			0x12
#define ULPI_INT_STATUS 			0x13
#define ULPI_INT_LATCH 				0x14
#define ULPI_DEBUG 				0x15
#define ULPI_SCRATCH 				0x16
#define ULPI_SCRATCH_SET 			0x17
#define ULPI_SCRATCH_CLEAR 			0x18

#ifdef CONFIG_MACH_LITTLETON
#define ULPI_CARKIT_CONTROL			0x19
#define ULPI_CARKIT_CONTROL_SET			0x1A
#define ULPI_CARKIT_CONTROL_CLEAR		0x1B
#define ULPI_CARKIT_INT_ENABLE			0x1D
#define ULPI_CARKIT_INT_ENABLE_SET		0x1E
#define ULPI_CARKIT_INT_ENABLE_CLEAR		0x1F
#define ULPI_CARKIT_INT_STATUS			0x20
#define ULPI_VENDOR_RID_CONV			0x36
#define ULPI_VENDOR_RID_CONV_SET		0x37
#define ULPI_VENDOR_RID_CONV_CLEAR		0x38
#endif 

#define ULPI_FC_RESET				(1 << 5) /* XCVR Reset */
#define ULPI_FC_SUSPENDM			(1 << 6) /* XCVR SuspendM, Low Power Mode */

#define ULPI_IC_6PIN				(1 << 0) /* XCVR 6 pin mode */
#define ULPI_IC_3PIN				(1 << 1) /* XCVR 3 pin mode */
#ifdef CONFIG_MACH_LITTLETON
#define ULPI_IC_CARKIT				(1 << 2) /* Carkit mode */
#endif
#define ULPI_IC_CLKSUSPENDM			(1 << 3) /* Active low clock suspend */

#define ULPI_OC_IDPULLUP			(1 << 0) /* ID Pull Up, enable sampling of ID line */
#define ULPI_OC_DPPULLDOWN			(1 << 1) /* Enable the 15K Ohm pull down resistor on D+ */
#define ULPI_OC_DMPULLDOWN			(1 << 2) /* Enable the 15K Ohm pull down resistor on D- */
#define ULPI_OC_DISCHRGVBUS			(1 << 3) /* Discharge Vbus */
#define ULPI_OC_CHRGVBUS			(1 << 4) /* Charge Vbus, for Vbus pulsing SRP */
#define ULPI_OC_DRVVBUS				(1 << 5) /* Drive 5V on Vbus */
#define ULPI_OC_DRVVBUSEXT			(1 << 6) /* Drive Vbus using external supply */

#define ULPI_INT_HOSTDISCON			(1 << 0) /* Host Disconnect */
#define ULPI_INT_VBUSVALID			(1 << 1) /* Vbus Valid */
#define ULPI_INT_SESSIONVALID			(1 << 2) /* Session Valid */
#define ULPI_INT_SESSIONEND			(1 << 3) /* Session End */
#define ULPI_INT_IDGND				(1 << 4) /* current status of IDGND */

#ifdef CONFIG_MACH_LITTLETON
/* Carkit Control Register Flags */
#define ULPI_CK_SPKLEFTEN			(1 << 4) /* Connect DM to Speaker Left Pin */
#define ULPI_CK_SPKRIGHTEN			(1 << 5) /* Connect DP to Speaker Right/Mic Pin */
#define ULPI_CK_SPKMICEN			(1 << 6) /* Connect DP to Speaker Right/Mic Pin */

/* Carkit Interrupt Enable Register Flags */
#define ULPI_CK_IDFLOATRISE			(1 << 0) /* Interrupt enabled when ID Pin transitions from Non-floating to Floating */
#define ULPI_CK_IDFLOATFALL			(1 << 1) /* Interrupt enabled when ID Pin transitions from Floating to Non-floating */
#define ULPI_CK_RIDINTEN			(1 << 5) /* Interrupt enabled when RidConvesionDone bit is asserted */

/* Carkit Rid Values */
#define RID_VALUE_MASK				0x7 /* Mask for RidValue Bits on Vendor RID Conversion Reg */
#define RID_0					0x0 /* Rid Value at 0K ohms */
#define RID_100K				0x2 /* Rid Value at 100K ohms */
#define RID_200K				0x3 /* Rid Value at 200K ohms */
#define RID_440K				0x4 /* Rid Value at 440K ohms */
#define RID_FLOATING				0x5 /* Rid Value indicates Floating */
#define RID_ERROR				0x7 /* Rid Value indicates Error */

/* Carkit Vendor Rid Conversion Register Flags */
#define RID_CONV_DONE				(1 << 3) /* Automatically Asserted when RID conversion is finished */
#define RID_CONV_START				(1 << 4) /* Flag to allow Rid ADC to start conversion process */

/* Carkit Interrupt Status Register Flags */
#define ULPI_CK_IDFLOAT				0x1 /* Asserted when ID pin is floating */
#endif

#ifndef CABLE_DETECT_GPIO
#define U2DOTGINT_DEFAULT	( U2DOTGINT_RID | U2DOTGINT_FID | U2DOTGINT_SI \
				| U2DOTGINT_RVV | U2DOTGINT_FVV | U2DOTGINT_SF)
#else
#define U2DOTGINT_DEFAULT	( U2DOTGINT_RID | U2DOTGINT_FID | U2DOTGINT_SI \
				| U2DOTGINT_SF )
#endif

#endif /* CONFIG_CPU_PXA310 */

enum hs_cmds_permf {
	HS_CMDS_0,
	HS_CMDS_1,
	HS_CMDS_2,
	HS_CMDS_3
};

#define ISO_HS_CMDS		HS_CMDS_0
#define INT_HS_CMDS		HS_CMDS_0

struct dma_txfr_t {
	unsigned int len;	/* Buffer Size */
	int is_zero;		/* Add zero package after transfer. Only valid for IN transfer. */
	int end_irq_en;		/* Enalbe/Disable End Interrupt at last descriptor */
};

struct pxa3xx_u2d;

struct pxa3xx_ep {
	struct usb_ep				ep;
	struct pxa3xx_u2d			*dev;

	const struct usb_endpoint_descriptor	*desc;
	struct list_head			queue;
	struct otg_transceiver			*trasceiver;
	unsigned long				pio_irqs;
	unsigned long				dma_irqs;

	unsigned				fifo_size;
	unsigned				ep_num;
	unsigned				ep_type;

	unsigned				stopped : 1;
	unsigned				dma_con : 1;
	unsigned				dir_in : 1;
	unsigned				assigned : 1;

	int						dma;
	void *					dma_desc_virt;
	dma_addr_t				dma_desc_phys;
	unsigned				dma_desc_size;

	void *					dma_buf_virt;
	dma_addr_t				dma_buf_phys;
	unsigned				dma_buf_size;

	unsigned				config;
	unsigned				interface;
	unsigned				aisn;

	/* HS only for ISO and interrupt endpoints */
	unsigned				hs_cmds;

#ifdef CONFIG_PM
	unsigned				u2dcsr_value;
	unsigned				u2dcr_value;
	unsigned				u2denr_value;
#endif
#ifdef CONFIG_USB_COMPOSITE
	unsigned	assigned_interface;		/*  actual interface number report to the host */
	struct gadget_driver_info * driver_info;	/*  pointer to corresponding gadget_driver_info */
#endif
};

struct pxa3xx_request {
	struct usb_request			req;
	struct list_head			queue;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_STALL,
	EP0_IN_FAKE,
	EP0_NO_ACTION
};

/* 8KB EP FIFO */
#define U2D_EPMEM_SIZE	((unsigned)8192)

#define EP0_MPS			((unsigned)64)
#define BULK_MPS(speed)	((speed)==USB_SPEED_HIGH)?((unsigned)512):((unsigned)64)
#define ISO_MPS(speed)	((speed)==USB_SPEED_HIGH)?((unsigned)1024):((unsigned)1023)
#define INT_MPS(speed)	((speed)==USB_SPEED_HIGH)?((unsigned)1024):((unsigned)8)

#define EP0_FIFO_SIZE	((unsigned)64)
#define BULK_FIFO_SIZE	(((unsigned)512 + 8)*2)
#define ISO_FIFO_SIZE	((unsigned)1024 + 8)
#define INT_FIFO_SIZE	((unsigned)100 + 8)

#define DMA_BUF_SIZE	PAGE_SIZE
#define DMA_DESC_SIZE	PAGE_SIZE
#define DMA_DESC_NUM	(DMA_DESC_SIZE/16)

struct u2d_stats {
	struct ep0stats {
		unsigned long		ops;
		unsigned long		bytes;
	} read, write;
	unsigned long			irqs;
};

#ifndef	U2D_EP_NUM
#ifndef CONFIG_CPU_PXA310
#define	U2D_EP_NUM	8
#else
#define	U2D_EP_NUM	16
#endif
#endif

#define MAX_CONFIG_LENGTH 256

#ifdef CONFIG_USB_COMPOSITE

struct gadget_driver_info
{
	struct gadget_driver_info *next;	/* point to next gadget_driver_info */
	unsigned config;			/* configuration number used by the driver */
	unsigned assigned_intf_start;		/* the first assigned interface number */
	unsigned num_intfs;			/* total interface number for the driver */
	unsigned ep_start;			/* the first ep number used in the ep list */
	unsigned num_eps;
#ifdef MULTI_P3
	unsigned char 	device_desc[18];	/* device desc for the driver */
#endif
	unsigned char	config_desc[MAX_CONFIG_LENGTH];	/* configuration desc for th driver */
	unsigned char 	config_desc_hs[MAX_CONFIG_LENGTH];

	struct 	usb_gadget_driver *driver;	/* store the driver pointer of gadget */
	void 	*driver_data;			/* pointer to the driver private data */
	unsigned stopped;			/* driver disconnected or not */
};

#ifdef MULTI_P4
struct t_str_id
{
	int str_id;				/* old string id */
	struct gadget_driver_info *driver_info; /* pointer to the driver info */
	struct t_str_id *next;
};
#endif
#endif

struct pxa3xx_u2d {
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
#ifdef CONFIG_USB_COMPOSITE
	struct gadget_driver_info 	*first_gadget;	/* head of a gadget_driver_info queue */
	struct gadget_driver_info 	*active_gadget;	/* currently active gadget */
	int	interface_count;			/* how many gadget already registered */
	int	driver_count;				/* how many ep already used */
	struct pxa3xx_request		ep0_req;	/* include the usb_req to respond to the get_desc request, etc */
	struct t_str_id 		*str_id;
	int	rm_flag;
#endif

	struct otg_transceiver		*transceiver;
	enum ep0_state			ep0state;
	struct u2d_stats		stats;
	unsigned			got_irq : 1,
					got_disc : 1,
					has_cfr : 1,
					req_pending : 1,
#ifndef CONFIG_USB_COMPOSITE
					req_std : 1,
					req_config : 1;
#else
					req_std : 1;
	unsigned 			req_config;
#endif

/* ????? is it still necessary for U2D */
#define start_watchdog(dev) mod_timer(&dev->timer, jiffies + (HZ/200))
	struct timer_list		timer;

	struct device			*dev;
	struct pxa3xx_u2d_mach_info	*mach;		/* where's the definition????? */
	u64				dma_mask;
	struct pxa3xx_ep		ep [U2D_EP_NUM];

	unsigned char			configs[MAX_CONFIG_LENGTH];
	unsigned			config_length;
	unsigned			configuration,
					interface,
					alternate;
#ifdef CONFIG_PM
	unsigned			u2dcr;
	unsigned			u2dicr;
	unsigned			u2dcsr0;
#ifdef CONFIG_CPU_PXA310
	unsigned			u2dotgcr;
	unsigned			u2dotgicr;
#endif
#endif

#ifdef CONFIG_DVFM
	struct pxa3xx_fv_notifier 	dvfm_notifier;
	wait_queue_head_t		delay_wait;
#endif
/* 	struct workqueue_struct 	*stp_work_queue;
 	struct delayed_work     	stp_gpio_work;
	char				stp_work_name[20];*/
	unsigned			stp_gpio_irq_en : 1,
					u2d_clk_dis: 1,
					u2d_irq_dis: 1,
					ulpi_dat3_work: 1;
};

/* LEDs are only for debug */
#ifndef HEX_DISPLAY
#define HEX_DISPLAY(n)		do {} while(0)
#endif

#ifndef LED_CONNECTED_ON
#define LED_CONNECTED_ON	do {} while(0)
#define LED_CONNECTED_OFF	do {} while(0)
#endif
#ifndef LED_EP0_ON
#define LED_EP0_ON		do {} while (0)
#define LED_EP0_OFF		do {} while (0)
#endif

static struct pxa3xx_u2d *the_controller;

/*-------------------------------------------------------------------------*/

/*
 * Debugging support vanishes in non-debug builds.  DBG_NORMAL should be
 * mostly silent during normal use/testing, with no timing side-effects.
 */
#define DBG_NORMAL	1	/* error paths, device state transitions */
#define DBG_VERBOSE	2	/* add some success path trace info */
#define DBG_NOISY	3	/* ... even more: request level */
#define DBG_VERY_NOISY	4	/* ... even more: packet level */

#ifdef DEBUG

static const char *state_name[] = {
	"EP0_IDLE",
	"EP0_IN_DATA_PHASE", "EP0_OUT_DATA_PHASE",
	"EP0_END_XFER", "EP0_STALL"
};

#define DMSG(stuff...) printk(KERN_DEBUG stuff)

#ifdef VERBOSE
#    define U2D_DEBUG DBG_VERY_NOISY
#else
#    define U2D_DEBUG DBG_NORMAL
#endif

static void __attribute__ ((__unused__))
dump_u2dcr(const char *label)
{
#if 0
	u32	u2dcr = U2DCR;
	DMSG("%s u2dcr %08X =%s%s%s%s%s%s%s%s%s%s%s%s%s, con=%d,inter=%d,altinter=%d\n",
		label, u2dcr,
		(u2dcr & U2DCR_HSTC_MASK) ? " hstc":"",
		(u2dcr & U2DCR_FSTC_MASK) ? " fstc":"",
		(u2dcr & U2DCR_UCLKOVR) ? " uclkovr" : "",
		(u2dcr & U2DCR_ABP) ? " abp" : "",
		(u2dcr & U2DCR_ADD) ? " add" : "",
		(u2dcr & U2DCR_CC) ? " cc" : "",
		(u2dcr & U2DCR_HS) ? " hs" : "",
		(u2dcr & U2DCR_DWRE) ? " dwre" : "",
		(u2dcr & U2DCR_SMAC) ? " smac" : "",
		(u2dcr & U2DCR_EMCE) ? " emce" : "",
		(u2dcr & U2DCR_UDR) ? " udr" : "",
		(u2dcr & U2DCR_UDA) ? " uda" : "",
		(u2dcr & U2DCR_UDE) ? " ude" : "",
		(u2dcr & U2DCR_ACN) >> U2DCR_ACN_S,
		(u2dcr & U2DCR_AIN) >> U2DCR_AIN_S,
		(u2dcr & U2DCR_AAISN)>> U2DCR_AAISN_S );
#endif
}

static void __attribute__ ((__unused__))
dump_u2dcsr0(const char *label)
{
#if 0
	u32		u2dcsr0 = U2DCSR0;

	DMSG("%s %s u2dcsr0 %08X =%s%s%s%s%s%s%s%s%s\n",
		label, state_name[the_controller->ep0state], u2dcsr0,
		(u2dcsr0 & U2DCSR0_IPA) ? " ipa" : "",
		(u2dcsr0 & U2DCSR0_SA) ? " sa" : "",
		(u2dcsr0 & U2DCSR0_RNE) ? " rne" : "",
		(u2dcsr0 & U2DCSR0_FST) ? " fst" : "",
		(u2dcsr0 & U2DCSR0_SST) ? " sst" : "",
		(u2dcsr0 & U2DCSR0_DME) ? " dme" : "",
		(u2dcsr0 & U2DCSR0_FTF) ? " ftf" : "",
		(u2dcsr0 & U2DCSR0_IPR) ? " ipr" : "",
		(u2dcsr0 & U2DCSR0_OPC) ? " opc" : "");
#endif
}

static void __attribute__ ((__unused__))
dump_state(struct pxa3xx_u2d *dev)
{
	unsigned	i;

	DMSG("%s, u2dicr %02X, u2disr %02X, u2dfnr %02X\n",
		state_name[dev->ep0state],
		U2DICR, U2DISR, U2DFNR);
	dump_u2dcr("u2dcr");

	if (!dev->driver) {
		DMSG("no gadget driver bound\n");
		return;
	} else
		DMSG("ep0 driver '%s'\n", dev->driver->driver.name);


	dump_u2dcsr0 ("u2dcsr0");
	DMSG("ep0 IN %lu/%lu, OUT %lu/%lu\n",
		dev->stats.write.bytes, dev->stats.write.ops,
		dev->stats.read.bytes, dev->stats.read.ops);

	for (i = 1; i < U2D_EP_NUM; i++) {
		if (dev->ep [i].desc == 0)
			continue;
		DMSG ("u2dcs%d = %02x\n", i, U2DCSR(i));
	}
}

#if 0
static void dump_regs(u8 ep)
{
	DMSG("EP:%d UDCCSR:0x%08x UDCBCR:0x%08x\n UDCCR:0x%08x\n",
		ep,UDCCSN(ep), UDCBCN(ep), UDCCN(ep));
}
static void dump_req (struct pxa3xx_request *req)
{
	struct usb_request *r = &req->req;

	DMSG("%s: buf:0x%08x length:%d dma:0x%08x actual:%d\n",
			__FUNCTION__, (unsigned)r->buf, r->length,
			r->dma,	r->actual);
}
#endif

#else

#define DMSG(stuff...)		do{}while(0)

#define	dump_u2dcr(x)	do{}while(0)
#define	dump_u2dcsr0(x)	do{}while(0)
#define	dump_state(x)	do{}while(0)

#define U2D_DEBUG ((unsigned)0)

#endif

#define DBG(lvl, stuff...) do{if ((lvl) <= U2D_DEBUG) DMSG(stuff);}while(0)

#define WARN(stuff...) printk(KERN_WARNING "u2d: " stuff)
#define INFO(stuff...) printk(KERN_INFO "u2d: " stuff)

#if defined(CONFIG_CPU_PXA310) && defined(CONFIG_MACH_LITTLETON)
/* USB headset macros */
#define USB_HEADSET_STEREO	1
#define USB_HEADSET_MONO_MIC	2
#endif

#endif /* __LINUX_USB_GADGET_PXA3xx_U2D_H */
