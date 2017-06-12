#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <asm/arch/pxa-regs.h>

static unsigned long woodstockbl_flags;
#define WOODSTOCKBL_SUSPENDED     0x01

static int woodstockbl_set_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	pxa_set_cken(CKEN_PWM0, 0);

	/* PWM clock is 13MHz.
	   This yields a period of 13MHz/(64*512), about 397Hz. */
	PWM_CTRL0 = 64-1;
	PWM_PWDUTY0 = intensity<<1;
	PWM_PERVAL0 = 512-1;

	if (bd->props.power == FB_BLANK_UNBLANK &&
	    bd->props.fb_blank == FB_BLANK_UNBLANK &&
	    !(woodstockbl_flags & WOODSTOCKBL_SUSPENDED))
		pxa_set_cken(CKEN_PWM0, 1);

	return 0;
}

#ifdef CONFIG_PM
static int woodstockbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	woodstockbl_flags |= WOODSTOCKBL_SUSPENDED;
	backlight_update_status(bd);
	return 0;
}

static int woodstockbl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	woodstockbl_flags &= ~WOODSTOCKBL_SUSPENDED;
	backlight_update_status(bd);
	return 0;
}
#else
#define woodstockbl_suspend	NULL
#define woodstockbl_resume	NULL
#endif

static int woodstockbl_get_intensity(struct backlight_device *bd)
{
	return PWM_PWDUTY0>>1;
}


static struct backlight_ops woodstockbl_ops = {
	.get_brightness = woodstockbl_get_intensity,
	.update_status  = woodstockbl_set_intensity,
};

static int woodstockbl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;

	bd = backlight_device_register ("woodstock-bl", &pdev->dev, NULL,
		&woodstockbl_ops);
	if (IS_ERR (bd))
		return PTR_ERR (bd);

	platform_set_drvdata(pdev, bd);

	bd->props.max_brightness = 256;
	bd->props.brightness = 256;
	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.fb_blank = FB_BLANK_UNBLANK;
	backlight_update_status(bd);

	printk("Woodstock Backlight Driver Initialized.\n");
	return 0;
}

static int woodstockbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.fb_blank = FB_BLANK_NORMAL;
	backlight_update_status(bd);

	backlight_device_unregister(bd);

	printk("Woodstock Backlight Driver Unloaded.\n");
	return 0;
}

static struct platform_driver woodstockbl_driver = {
	.probe		= woodstockbl_probe,
	.remove		= woodstockbl_remove,
	.suspend	= woodstockbl_suspend,
	.resume		= woodstockbl_resume,
	.driver		= {
		.name	= "woodstock-bl",
	},
};

static int __init woodstockbl_init(void)
{
	return platform_driver_register(&woodstockbl_driver);
}

static void __exit woodstockbl_exit(void)
{
	platform_driver_unregister(&woodstockbl_driver);
}

module_init(woodstockbl_init);
module_exit(woodstockbl_exit);

MODULE_AUTHOR("Charles Hannum <charles.hannum@sonos.com>");
MODULE_DESCRIPTION("Woodstock Backlight Driver");
