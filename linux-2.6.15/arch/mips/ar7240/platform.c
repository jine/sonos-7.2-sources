#include <linux/config.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#include <linux/console.h>
#include <asm/serial.h>

#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>

#include "mdp.h"
extern struct manufacturing_data_page sys_mdp;

#include <asm/mach-ar7240/ar7240.h>

extern uint32_t ar7240_ahb_freq;

/* 
 * OHCI (USB full speed host controller) 
 */
static struct resource ar7240_usb_ohci_resources[] = {
	[0] = {
		.start		= AR7240_USB_OHCI_BASE,
		.end		= AR7240_USB_OHCI_BASE + AR7240_USB_WINDOW - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= AR7240_CPU_IRQ_USB,
        .end	    = AR7240_CPU_IRQ_USB,
		.flags		= IORESOURCE_IRQ,
	},
};

/* 
 * The dmamask must be set for OHCI to work 
 */
static u64 ohci_dmamask = ~(u32)0;
static struct platform_device ar7240_usb_ohci_device = {
	.name		= "ar7240-ohci",
	.id		    = 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(ar7240_usb_ohci_resources),
	.resource	= ar7240_usb_ohci_resources,
};

/* 
 * EHCI (USB full speed host controller) 
 */
static struct resource ar7240_usb_ehci_resources[] = {
	[0] = {
		.start		= AR7240_USB_EHCI_BASE,
		.end		= AR7240_USB_EHCI_BASE + AR7240_USB_WINDOW - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= AR7240_CPU_IRQ_USB,
		.end		= AR7240_CPU_IRQ_USB,
		.flags		= IORESOURCE_IRQ,
	},
};

/* 
 * The dmamask must be set for EHCI to work 
 */
static u64 ehci_dmamask = ~(u32)0;

static struct platform_device ar7240_usb_ehci_device = {
	.name		= "ar7240-ehci",
	.id		    = 0,
	.dev = {
		.dma_mask		= &ehci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(ar7240_usb_ehci_resources),
	.resource	= ar7240_usb_ehci_resources,
};

static struct resource ar7240_uart_resources[] = {
	{
		.start = AR7240_UART_BASE,
		.end = AR7240_UART_BASE+0x0fff,
		.flags = IORESOURCE_MEM,
	},
};

static struct plat_serial8250_port ar7240_uart_data[] = {
	{
                .membase        = (char *)KSEG1ADDR(AR7240_UART_BASE),
                .irq            = AR7240_MISC_IRQ_UART,
                .flags          = STD_COM_FLAGS,
                .iotype         = UPIO_MEM32,
                .regshift       = 2,
                .uartclk        = 0, /* ar7240_ahb_freq, */
	},
        { },
};

static struct platform_device ar7240_uart = {
	 .name               = "serial8250",
        .id                 = 0,
        .dev.platform_data  = ar7240_uart_data,
        .num_resources      = 1, 
        .resource           = ar7240_uart_resources

};

static struct platform_device *ar7240_platform_devices[] __initdata = {
	&ar7240_usb_ohci_device,
	&ar7240_uart
};

int ar7240_platform_init(void)
{
        /* Unlock the console in any diag build, or if the appropriate flag
           is set, but just not passing the device to platform_add_devices */
        int bUnlockConsole = 1;
#ifndef CONFIG_SONOS_DIAGS
	if ((sys_mdp.mdp_flags&2)==0) bUnlockConsole = 0;
#endif

        /* need to set clock appropriately */
        ar7240_uart_data[0].uartclk = ar7240_ahb_freq; 
	return platform_add_devices(ar7240_platform_devices, 
                                    bUnlockConsole ? 2 : 1);
}

arch_initcall(ar7240_platform_init);
