/*
 *  linux/drivers/acorn/char/i2c.c
 *
 *  Copyright (C) 2000 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  based on ARM IOC/IOMD i2c driver.
 *
 *  On Acorn machines, the following i2c devices are on the bus:
 *	- PCF8583 real time clock & static RAM
 */
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/platform_device.h>

#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/mfp.h>

#define SCLGPIO0 21
#define SDAGPIO0 22

#define SCLGPIO1 104
#define SDAGPIO1 103

struct woodstock_bbi2c_data {
    int sclgpio;
    int sdagpio;
};

#ifndef CONFIG_I2C_PXA
static struct woodstock_bbi2c_data bbd0 = { SCLGPIO0, SDAGPIO0 };
#endif
static struct woodstock_bbi2c_data bbd1 = { SCLGPIO1, SDAGPIO1 };

static void ioc_setscl(void *data, int state)
{
	struct woodstock_bbi2c_data *d=data;
	int dir;
	if (state) dir=GPIO_DIR_IN; else dir=GPIO_DIR_OUT;
	pxa3xx_gpio_set_direction(d->sclgpio,dir);
}

static void ioc_setsda(void *data, int state)
{
	struct woodstock_bbi2c_data *d=data;
	int dir;
	if (state) dir=GPIO_DIR_IN; else dir=GPIO_DIR_OUT;
	pxa3xx_gpio_set_direction(d->sdagpio,dir);
}

static int ioc_getscl(void *data)
{
	struct woodstock_bbi2c_data *d=data;
	return pxa3xx_gpio_get_level(d->sclgpio);
}

static int ioc_getsda(void *data)
{
	struct woodstock_bbi2c_data *d=data;
	return pxa3xx_gpio_get_level(d->sdagpio);
}

#ifndef CONFIG_I2C_PXA
static struct i2c_algo_bit_data ioc_data0 = {
        .setsda         = ioc_setsda,
        .setscl         = ioc_setscl,
        .getsda         = ioc_getsda,
        .getscl         = ioc_getscl,
        .udelay         = 50,
        .timeout        = 100, /*jiffies*/
        .data           = &bbd0
};

static struct i2c_adapter ioc_ops0 = {
        .id                     = 0,
        .algo_data              = &ioc_data0,
};
#endif

static struct i2c_algo_bit_data ioc_data1 = {
	.setsda		= ioc_setsda,
	.setscl		= ioc_setscl,
	.getsda		= ioc_getsda,
	.getscl		= ioc_getscl,
	.udelay		= 3,
	.timeout	= 100, /*jiffies*/
	.data		= &bbd1
};

static struct i2c_adapter ioc_ops1 = {
	.id			= 1,
	.algo_data		= &ioc_data1,
};

struct pxa3xx_pin_config woodstock_i2c_bit_mfp_pins[] = {
	PXA3xx_MFP_CFG("SCL",MFP_PIN_GPIO104,MFP_PIN_GPIO104_AF_GPIO_104,MFP_DS04X,0,MFP_LPM_FLOAT,0),
	PXA3xx_MFP_CFG("SDA",MFP_PIN_GPIO103,MFP_PIN_GPIO103_AF_GPIO_103,MFP_DS04X,0,MFP_LPM_FLOAT,0),
#ifndef CONFIG_I2C_PXA
	PXA3xx_MFP_CFG("SCL0",MFP_PIN_GPIO21,MFP_PIN_GPIO21_AF_GPIO_21,MFP_DS04X,0,MFP_LPM_FLOAT,0),
	PXA3xx_MFP_CFG("SDA0",MFP_PIN_GPIO22,MFP_PIN_GPIO22_AF_GPIO_22,MFP_DS04X,0,MFP_LPM_FLOAT,0),
#endif
};

void woodstock_i2c_restore(void)
{
        pxa3xx_mfp_set_configs(woodstock_i2c_bit_mfp_pins,ARRAY_SIZE(woodstock_i2c_bit_mfp_pins));
#ifndef CONFIG_I2C_PXA
        pxa3xx_gpio_set_direction(SCLGPIO0,GPIO_DIR_IN);
        pxa3xx_gpio_set_level(SCLGPIO0,GPIO_LEVEL_LOW);
        pxa3xx_gpio_set_direction(SDAGPIO0,GPIO_DIR_IN);
        pxa3xx_gpio_set_level(SDAGPIO0,GPIO_LEVEL_LOW);
#endif
        pxa3xx_gpio_set_direction(SCLGPIO1,GPIO_DIR_IN);
        pxa3xx_gpio_set_level(SCLGPIO1,GPIO_LEVEL_LOW);
        pxa3xx_gpio_set_direction(SDAGPIO1,GPIO_DIR_IN);
        pxa3xx_gpio_set_level(SDAGPIO1,GPIO_LEVEL_LOW);

}

static int __init i2c_ioc_init(struct platform_device *dev)
{
	printk("woodstock i2c init\n");
	pxa3xx_mfp_set_configs(woodstock_i2c_bit_mfp_pins,ARRAY_SIZE(woodstock_i2c_bit_mfp_pins));
#ifndef CONFIG_I2C_PXA
	pxa3xx_gpio_set_direction(SCLGPIO0,GPIO_DIR_IN);
	pxa3xx_gpio_set_level(SCLGPIO0,GPIO_LEVEL_LOW);
	pxa3xx_gpio_set_direction(SDAGPIO0,GPIO_DIR_IN);
	pxa3xx_gpio_set_level(SDAGPIO0,GPIO_LEVEL_LOW);
#endif
        pxa3xx_gpio_set_direction(SCLGPIO1,GPIO_DIR_IN);
        pxa3xx_gpio_set_level(SCLGPIO1,GPIO_LEVEL_LOW);
        pxa3xx_gpio_set_direction(SDAGPIO1,GPIO_DIR_IN);
        pxa3xx_gpio_set_level(SDAGPIO1,GPIO_LEVEL_LOW);

#ifndef CONFIG_I2C_PXA
	i2c_bit_add_bus(&ioc_ops0);
#endif
	return i2c_bit_add_bus(&ioc_ops1);
}

static struct platform_driver i2c_woodstock_driver = {
        .probe          = i2c_ioc_init,
        .driver         = {
                .name   = "woodstock-i2c",
        },
};

static int __init i2c_adap_woodstock_init(void)
{
        return platform_driver_register(&i2c_woodstock_driver);
}


__initcall(i2c_adap_woodstock_init);
