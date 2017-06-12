/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * Bus Glue for pxa3xx
 *
 * Written by Christopher Hoover <ch@hpl.hp.com>
 * Based on fragments of previous driver by Russell King et al.
 *
 * Modified for LH7A404 from ohci-sa1111.c
 *  by Durgesh Pattamatta <pattamattad@sharpsec.com>
 *
 * Modified for pxa27x from ohci-lh7a404.c
 *  by Nick Bane <nick@cecomputing.co.uk> 26-8-2004
 *
 * This file is licenced under the GPL.

 *(C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 */

#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/gpio.h>
#include <asm/arch/arava.h>
#include <asm/arch/ohci.h>
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>

#ifdef CONFIG_IPM
#include <asm/arch/ipmc.h>
#endif
/* #define ENABLE_PORT2 */
#define UHC_PROC_FILE
#define	dmsg(format, args...)	printk(KERN_DEBUG "%s: " format "\n", \
			__func__, ##args)

#define PXA_UHC_MAX_PORTNUM    3

/* #define UHCRHPS(x)              __REG2( 0x4C000050, (x)<<2 ) */

#if defined(CONFIG_DVFM) && defined(CONFIG_PXA3xx)
static int uhc_dvfm_notifier(unsigned int cmd, void *client_data, void *info);
static struct pxa3xx_fv_notifier dvfm_notifier_uhc = {
	.name = "mhn-uhc",
	.priority = 0,
	.notifier_call = uhc_dvfm_notifier,
};

static int pxa3xx_ohci_hub_status_data (struct usb_hcd *hcd, char *buf)
{
	int ret = ohci_hub_status_data(hcd, buf);

#ifdef	CONFIG_IPM
	if (ret && ((ACSR & 0x40000000) || (ACSR & 0x71F)==0x110)) {
		pr_debug("ACSR: 0x%x, sent event to ipmd\n", ACSR);
		ipm_event_notify(IPM_EVENT_DEVICE, IPM_EVENT_DEVICE_OUT208, NULL, 0);
		ret = 0;
	}
#endif
	return ret;
}
#endif

static int pxa3xx_ohci_pmm_state;

/*
  PMM_NPS_MODE -- PMM Non-power switching mode
      Ports are powered continuously.

  PMM_GLOBAL_MODE -- PMM global switching mode
      All ports are powered at the same time.

  PMM_PERPORT_MODE -- PMM per port switching mode
      Ports are powered individually.
 */
static int pxa3xx_ohci_select_pmm( int mode )
{
	pxa3xx_ohci_pmm_state = mode;

	switch ( mode ) {
	case PMM_NPS_MODE:
		UHCRHDA |= RH_A_NPS;
		break; 
	case PMM_GLOBAL_MODE:
		UHCRHDA &= ~(RH_A_NPS & RH_A_PSM);
		break;
	case PMM_PERPORT_MODE:
		UHCRHDA &= ~(RH_A_NPS);
		UHCRHDA |= RH_A_PSM;

		/* Set port power control mask bits, only 3 ports. */
		UHCRHDB |= (0x7<<17);
		break;
	default:
		printk( KERN_ERR
			"Invalid mode %d, set to non-power switch mode.\n", 
			mode );

		pxa3xx_ohci_pmm_state = PMM_NPS_MODE;
		UHCRHDA |= RH_A_NPS;
	}

	return 0;
}

extern int usb_disabled(void);

/*-------------------------------------------------------------------------*/

static int pxa3xx_start_hc(struct device *dev)
{
	int retval = 0;
	struct pxaohci_platform_data *inf;

	inf = dev->platform_data;

	pxa_set_cken(CKEN_USBH, 1);

	UHCHR |= UHCHR_FHR;
	udelay(11);
	UHCHR &= ~UHCHR_FHR;

	UHCHR |= UHCHR_FSBIR;
	while (UHCHR & UHCHR_FSBIR)
		cpu_relax();

	if (inf->init)
		retval = inf->init(dev);
	
	if (retval < 0)
		return retval;

	pxa3xx_ohci_pmm_state = inf->port_mode;

	/* Set the Power Control Polarity Low and Power Sense
	 * Polarity Low to active low. Supply power to USB ports 1/2.
	 */
	UHCHR = (UHCHR | UHCHR_PCPL | UHCHR_PSPL | UHCHR_SSEP3)
		& (~(UHCHR_SSEP1 | UHCHR_SSEP2 | UHCHR_SSE));
	UHCHIE = (UHCHIE_UPRIE | UHCHIE_RWIE);

	return 0;
}

static void pxa3xx_stop_hc(struct device *dev)
{
	struct pxaohci_platform_data *inf;

	inf = dev->platform_data;

	if (inf->exit)
		inf->exit(dev);

	UHCHR |= UHCHR_FHR;
	udelay(11);
	UHCHR &= ~UHCHR_FHR;

	/* set global power switch mode and clear global power */
	UHCRHDA &= ~(RH_A_NPS | RH_A_PSM);
	UHCRHS   = 0x1;

	UHCCOMS |= 1;
	udelay(10);

	pxa_set_cken(CKEN_USBH, 0);
}

#ifdef UHC_PROC_FILE

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

static const char proc_node_name [] = "driver/uhc";

static int
uhc_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	unsigned long		flags;
	int			t;
	struct usb_hcd *hcd 	= _dev;
	struct ohci_hcd *ohci 	= hcd_to_ohci(hcd);

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* frame number */
	t = scnprintf(next, size,
		"frame no: %d\n", ohci_frame_no(ohci));
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static int uhc_proc_write(struct file *filp, const char *buffer,
		unsigned long count, void *data)
{
	char kbuf[8];
	int index;
	struct usb_hcd *hcd = data;
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);

	if (count >=8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch(index) {
	case 1:
		ohci_dump(ohci, 1);
		break;
	default:
		return -EINVAL;
	}
	return count;
}

#ifndef CONFIG_PROC_FS
#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, uhc_proc_read, dev)
#else
#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry(proc_node_name, 0, NULL);\
		if (ent) { \
			ent->data = hcd; \
			ent->read_proc = uhc_proc_read; \
			ent->write_proc = uhc_proc_write; \
		} \
	}while(0);
#endif
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else	/* !UHC_PROC_FILE */
#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif	/* UHC_PROC_FILE */

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

#ifdef ENABLE_PORT2
/* Enable Vbus through control arava register */
void enable_vbus( void )
{
	int status;
	
	/* FIXME */
	/* mdelay(1000); FIXME: need removed */
	status = arava_write(ARAVA_USBPUMP, 0xc1);
	if (status) {
		printk(KERN_ERR"%s: Failed to write i2c\n", __FUNCTION__);
		return ;
	}
	arava_read(ARAVA_MISCB, &value);
	value |= 0x8;
	arava_write(ARAVA_MISCB, value);
}
#endif

/**
 * usb_hcd_pxa3xx_probe - initialize pxa3xx-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int usb_hcd_pxa3xx_probe (const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;

	if (pdev->resource[1].flags != IORESOURCE_IRQ) {
		pr_debug ("resource[1] is not IORESOURCE_IRQ");
		return -ENOMEM;
	}

	hcd = usb_create_hcd (driver, &pdev->dev, "pxa3xx");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err2;
	}

#ifdef ENABLE_PORT2
	UP2OCR = 0x0003000c;
	udelay(10);
#endif

	if ((retval = pxa3xx_start_hc(&pdev->dev)) < 0) {
		printk(KERN_ERR "pxa3xx_start_hc failed");
		goto err3;
	}

	/* Select Power Management Mode */
	pxa3xx_ohci_select_pmm(pxa3xx_ohci_pmm_state);

#ifdef ENABLE_PORT2
	enable_vbus();
#endif

#ifdef	CONFIG_USB_OTG
	hcd->self.otg_port = 2;
#endif

	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, pdev->resource[1].start, SA_INTERRUPT);
	if (retval == 0) {
		create_proc_files();
#ifdef CONFIG_DVFM
		dvfm_notifier_uhc.client_data = hcd_to_ohci(hcd);
		pxa3xx_fv_register_notifier(&dvfm_notifier_uhc);
#endif
		return retval;
	}

	pxa3xx_stop_hc(&pdev->dev);
 err3:
	iounmap(hcd->regs);
 err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
 err1:
	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_pxa3xx_remove - shutdown processing for pxa3xx-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_pxa3xx_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
void usb_hcd_pxa3xx_remove (struct usb_hcd *hcd, struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	pxa3xx_stop_hc(&pdev->dev);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
#ifdef CONFIG_DVFM
	pxa3xx_fv_unregister_notifier(&dvfm_notifier_uhc);
#endif
}

/*-------------------------------------------------------------------------*/

static int __devinit
ohci_pxa3xx_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	ohci_dbg (ohci, "ohci_pxa3xx_start, ohci:%p", ohci);

	/* The value of NDP in roothub_a is incorrect on this hardware */
	ohci->num_ports = 3;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		err ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

#ifdef CONFIG_USB_OTG
	ohci->transceiver = otg_get_transceiver();
	if (ohci->transceiver) {
		otg_set_host(ohci->transceiver, &hcd->self);
	} else {
		dev_err(hcd->self.controller, "can't find otg transceiver\n");
		return -ENODEV;
	}
#endif

	return 0;
}

#ifdef CONFIG_USB_OTG
static void start_hnp(struct ohci_hcd *ohci)
{
	struct usb_hcd *hcd = ohci_to_hcd(ohci);
	const unsigned  port = hcd->self.otg_port - 1;
	unsigned long   flags;

	otg_start_hnp(ohci->transceiver);

	local_irq_save(flags);
	ohci->transceiver->state = OTG_STATE_A_SUSPEND;
	writel (RH_PS_PSS, &ohci->regs->roothub.portstatus [port]);
	local_irq_restore(flags);
}

static int ohci_pxa_connect(struct usb_hcd *hcd, struct usb_device *udev)
{
	return otg_connect((hcd_to_ohci(hcd))->transceiver, udev);
}

static int ohci_pxa_disconnect(struct usb_hcd *hcd)
{
	return otg_disconnect((hcd_to_ohci(hcd))->transceiver);
}

#endif


/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_pxa3xx_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"PXA3xx OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_pxa3xx_start,
	.stop =			ohci_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
#if defined(CONFIG_DVFM)
	.hub_status_data =	pxa3xx_ohci_hub_status_data,
#else
	.hub_status_data =	ohci_hub_status_data,
#endif
	.hub_control =		ohci_hub_control,

#ifdef  CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif

#ifdef	CONFIG_USB_OTG
	.disconnect = 		ohci_pxa_disconnect,
	.connect = 		ohci_pxa_connect,
	.start_port_reset = 	ohci_start_port_reset,
#endif
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_pxa3xx_drv_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug ("In ohci_hcd_pxa3xx_drv_probe");

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_pxa3xx_probe(&ohci_pxa3xx_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_pxa3xx_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);

	usb_hcd_pxa3xx_remove(hcd, pdev);
	remove_proc_files();
	return 0;
}

#ifdef CONFIG_PM
extern int usb_suspend_both(struct usb_device *udev, pm_message_t msg);
extern int usb_resume_both(struct usb_device *udev);
static int ohci_hcd_pxa3xx_drv_suspend( struct platform_device *pdev,
					pm_message_t state)
{
	struct ohci_hcd *ohci = hcd_to_ohci(platform_get_drvdata(pdev));
	int i, temp, connected = 0;

	for (i = 0; i < 3; i++) {
		temp = ohci_readl(ohci, &ohci->regs->roothub.portstatus[i]);
		if (temp & RH_PS_CCS)
			connected = 1;
	}
	if (connected)
		return -EACCES;

#ifdef  CONFIG_USB_SUSPEND
	usb_suspend_both(ohci_to_hcd(ohci)->self.root_hub,
			state);
#else
	usb_lock_device(ohci_to_hcd(ohci)->self.root_hub);
	ohci_bus_suspend(ohci_to_hcd(ohci));
	usb_unlock_device(ohci_to_hcd(ohci)->self.root_hub);
#endif
	disable_irq(pdev->resource[1].start);
	pxa3xx_stop_hc(&pdev->dev);

	return 0;
}

static int ohci_hcd_pxa3xx_drv_resume(struct platform_device *pdev)
{
	struct ohci_hcd *ohci = hcd_to_ohci(platform_get_drvdata(pdev));
	int status = 0;

	pxa3xx_start_hc(&pdev->dev);
	enable_irq(pdev->resource[1].start);

#ifdef  CONFIG_USB_SUSPEND
	status = usb_resume_both(ohci_to_hcd(ohci)->self.root_hub);
#else
	usb_lock_device(ohci_to_hcd(ohci)->self.root_hub);
	status = ohci_bus_resume(ohci_to_hcd(ohci));
	usb_unlock_device(ohci_to_hcd(ohci)->self.root_hub);
#endif

	return status;
}
#endif

#ifdef CONFIG_DVFM

static int check_dvfm_status(struct pxa3xx_fv_notifier_info *info)
{	int ret = 0;

	/* if next OP is D0CS */
	if (info->cur.d0cs == 0 && info->next.d0cs == 1) 
		ret = 1;

	/* if next OP is 208MHz */
	if ((info->next.xl == 16 && info->next.xn == 1)
		&& (info->next.d0cs == 0)) 
		ret = 1;

	return ret;
}

static int uhc_dvfm_notifier(unsigned int cmd, void *client_data, void *info)
{
	struct ohci_hcd *ohci = (struct ohci_hcd *)client_data;
	struct pxa3xx_fv_notifier_info *notifier_info = info;
	unsigned temp = 0, connected = 0, i;
	static int count = 0;

	for (i = 0; i < 3; i++) {
		temp = ohci_readl(ohci, &ohci->regs->roothub.portstatus[i]);
		if (temp & RH_PS_CCS) {
			connected = 1;
			count = 3;
		}
	}

	switch (cmd) {
	case FV_NOTIFIER_QUERY_SET:
		if (check_dvfm_status(notifier_info)) {
			if (connected) {
				pr_debug("USB 1.1 Host: unplug the cable and try again\n");
				return -EAGAIN;
			} else {
				if (count > 0) {
					count--;
					pr_debug("count %d\n", count);
					return -EAGAIN;
				}
			}
		}

		break;
		
	case FV_NOTIFIER_PRE_SET:
		break;

	case FV_NOTIFIER_POST_SET:
		break;
	}

	return 0;
}
#endif

static struct platform_driver ohci_hcd_pxa3xx_driver = {
	.probe		= ohci_hcd_pxa3xx_drv_probe,
	.remove		= ohci_hcd_pxa3xx_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend	= ohci_hcd_pxa3xx_drv_suspend, 
	.resume		= ohci_hcd_pxa3xx_drv_resume, 
#endif
	.driver		= {
		.name	= "pxa3xx-ohci",
	},
};

