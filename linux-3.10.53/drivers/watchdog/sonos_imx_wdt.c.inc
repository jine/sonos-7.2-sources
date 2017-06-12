#include "sonos_lock.h"
#include "sonos_rollback.h"
static int kernel_service_count;
// Change to KERNEL_DEFAULT when userspace code is ready to service watchdog
static int watchdog_service_state = KERNEL_SERVICE;
static unsigned long last_service;

static inline void imx2_wdt_ping(void)
{
	__raw_writew(IMX2_WDT_SEQ1, imx2_wdt.base + IMX2_WDT_WSR);
	__raw_writew(IMX2_WDT_SEQ2, imx2_wdt.base + IMX2_WDT_WSR);
	last_service = jiffies;
}

static void imx2_wdt_timer_ping(unsigned long arg)
{
	__u32 s_since_service;
	/* ping it every (imx2_wdt.timeout / 2) seconds to prevent reboot */
	switch ( watchdog_service_state ) {
	case KERNEL_DEFAULT:
		if ( kernel_service_count < KERNEL_SERVICE_MAX ) {
			imx2_wdt_ping();
		} else {
			printk(KERN_WARNING "Watchdog state is KERNEL_DEFAULT and the kernel will stop servicing.\n");
		}
		kernel_service_count++;
		break;
	case KERNEL_SERVICE:
		imx2_wdt_ping();
		break;
	case USERSPACE_SERVICE:
		kernel_service_count = 0;
		s_since_service = (jiffies - last_service)/HZ;
		if ( s_since_service > 20 ) {
			printk(KERN_INFO "Userspace app has not serviced the interrupt for %d seconds...\n",s_since_service);
		}
		break;
	default:
		printk(KERN_ERR "bad watchdog state\n");
	}
	mod_timer(&imx2_wdt.timer, jiffies + imx2_wdt.timeout * HZ / WDT_PETS_PER_TIMEOUT);
}

void wdt2_set_service_state(int new_state)
{
	// This is the userspace app petting the dog...
	if ( new_state != watchdog_service_state ) {
		// Service state changing
		if ( new_state == USERSPACE_SERVICE ) {
			printk(KERN_INFO "Userspace app will service watchdog.\n");
		} else if ( new_state == KERNEL_DEFAULT ) {
			printk(KERN_INFO "Kernel will service watchdog temporarily.\n");
		} else if ( new_state == KERNEL_SERVICE ) {
			printk(KERN_INFO "Kernel will service watchdog indefinitely.\n");
		}
	}
	watchdog_service_state = new_state;
	imx2_wdt_ping();
}
EXPORT_SYMBOL(wdt2_set_service_state);

int wdt2_get_service_state(void)
{
	return watchdog_service_state;
}
EXPORT_SYMBOL(wdt2_get_service_state);

static int __init imx2_wdt_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	struct resource *res;
	u16 val;
	int service_needed = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imx2_wdt.base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(imx2_wdt.base))
		return PTR_ERR(imx2_wdt.base);

	imx2_wdt.clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(imx2_wdt.clk)) {
		dev_err(&pdev->dev, "can't get Watchdog clock\n");
		return PTR_ERR(imx2_wdt.clk);
	}

	val = __raw_readw(imx2_wdt.base + IMX2_WDT_WCR) ;
	/* Looking for WDE and not WDZST, indicates wdog is running */
	if ( (val & IMX2_WDT_WCR_WDE) && !(val & IMX2_WDT_WCR_WDZST) ) {
		/* uboot started the watchdog.  Figure out what it set as the time,
		 * and set up an event timer for n/WDT_PETS_PER_TIMEOUT to pat the dog.
		 */
		timeout = ((val >> 8) + 1) / 2;
		// Note that timeout is rounded down to integer seconds and
		// would notably be 0 here for 0.5s timeout which will
		// subsequently clamp to 1s below which would be longer
		// than the real timeout.  I'm not going to worry about
		// that case beyond this comment, though.
		printk(KERN_INFO "Watchdog running with a %d s timeout - will be serviced every %d ms\n",
				timeout, timeout * 1000 / WDT_PETS_PER_TIMEOUT);
		service_needed = 1;
		kernel_service_count = 0;
	} else {
		printk(KERN_INFO "watchdog timer not initialized\n");
	}

	imx2_wdt.timeout = clamp_t(unsigned, timeout, 1, IMX2_WDT_MAX_TIME);
	if (imx2_wdt.timeout != timeout)
		dev_warn(&pdev->dev, "Initial timeout out of range! "
			"Clamped from %u to %u\n", timeout, imx2_wdt.timeout);

	setup_timer(&imx2_wdt.timer, imx2_wdt_timer_ping, 0);
	if ( service_needed ) {
		/* Start the timer */
		mod_timer(&imx2_wdt.timer, jiffies + imx2_wdt.timeout * HZ / WDT_PETS_PER_TIMEOUT);
	}
	last_service = jiffies;

	imx2_wdt_miscdev.parent = &pdev->dev;
	ret = misc_register(&imx2_wdt_miscdev);
	if (ret)
		goto fail;

	dev_info(&pdev->dev,
		"IMX2+ Watchdog Timer driver registered. timeout=%ds (nowayout=%d)\n",
						imx2_wdt.timeout, nowayout);
	return 0;

fail:
	imx2_wdt_miscdev.parent = NULL;
	clk_put(imx2_wdt.clk);
	return ret;
}
