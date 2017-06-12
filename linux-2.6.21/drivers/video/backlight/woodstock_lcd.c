#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/lcd.h>
#include <asm/arch/pxa-regs.h>

static int woodstocklcd_set_power(struct lcd_device *ld, int on)
{
	return 0;
}

static int woodstocklcd_get_power(struct lcd_device *ld)
{
}

extern void woodstock_lcdspi_write(unsigned char, unsigned char);
extern unsigned char woodstock_lcdspi_read(unsigned char);

static int woodstocklcd_set_contrast(struct lcd_device *ld, int contrast)
{
	woodstock_lcdspi_write(0x1B, contrast);
	return 0;
}

static int woodstocklcd_get_contrast(struct lcd_device *ld)
{
	return woodstock_lcdspi_read(0x1B);
}

static struct lcd_ops woodstocklcd_ops = {
	.set_power	= woodstocklcd_set_power,
	.get_power	= woodstocklcd_get_power,
	.set_contrast	= woodstocklcd_set_contrast,
	.get_contrast	= woodstocklcd_get_contrast,
};

static int woodstocklcd_probe(struct platform_device *pdev)
{
	struct lcd_device *ld;

	ld = lcd_device_register ("woodstock-lcd", &pdev->dev,
		&woodstocklcd_ops);
	if (IS_ERR (ld))
		return PTR_ERR (ld);

	platform_set_drvdata(pdev, ld);

#if 0
	woodstocklcd_set_power(ld, FB_BLANK_UNBLANK);
#endif

	printk("Woodstock LCD Driver Initialized.\n");
	return 0;
}

static int woodstocklcd_remove(struct platform_device *pdev)
{
	struct lcd_device *ld = platform_get_drvdata(pdev);

#if 0
	woodstocklcd_set_power(ld, FB_BLANK_POWERDOWN);
#endif

	lcd_device_unregister(ld);

	printk("Woodstock LCD Driver Unloadled.\n");
	return 0;
}

static struct platform_driver woodstocklcd_driver = {
	.probe		= woodstocklcd_probe,
	.remove		= woodstocklcd_remove,
#if 0
	.suspend	= woodstocklcd_suspend,
	.resume		= woodstocklcd_resume,
#endif
	.driver		= {
		.name	= "woodstock-lcd",
	},
};


static int __init woodstocklcd_init(void)
{
	return platform_driver_register(&woodstocklcd_driver);
}

static void __exit woodstocklcd_exit(void)
{
	platform_driver_unregister(&woodstocklcd_driver);
}

module_init(woodstocklcd_init);
module_exit(woodstocklcd_exit);

MODULE_AUTHOR("Charles Hannum <charles.hannum@sonos.com>");
MODULE_DESCRIPTION("Woodstock LCD Driver");
