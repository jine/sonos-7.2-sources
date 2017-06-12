
/*
 * linux/drivers/i2c/i2c-frodo.c
 *
 * Author: Abraham van der Merwe <abraham@2d3d.co.za>
 *
 * An I2C adapter driver for the 2d3D, Inc. StrongARM SA-1110
 * Development board (Frodo).
 *
 * This source code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <asm/mach-ar7240/ar7240.h>

#define CASBAH_SCL 0
#define CASBAH_SDA 1
#define CASBAH_AUTHIC_RESET 13

static void ar7240_gpio_config_output(int gpio)
{
    ar7240_reg_rmw_set(AR7240_GPIO_OE, (1 << gpio));
}

static void ar7240_gpio_config_input(int gpio)
{
    ar7240_reg_rmw_clear(AR7240_GPIO_OE, (1 << gpio));
}

static void ar7240_gpio_out_val(int gpio, int val)
{
    if (val) {
        ar7240_reg_rmw_set(AR7240_GPIO_OUT, (1 << gpio));
    }
    else {
        ar7240_reg_rmw_clear(AR7240_GPIO_OUT, (1 << gpio));
    }
}

static int ar7240_gpio_in_val(int gpio)
{
    return((1 << gpio) & (ar7240_reg_rd(AR7240_GPIO_IN)));
}

static void casbah_setsda (void *data,int state)
{
	if (state)
		ar7240_gpio_config_input(CASBAH_SDA);
	else {
      if (ar7240_reg_rd(AR7240_GPIO_OUT) & (1 << CASBAH_SDA)) {
         printk("Casbah-I2C: force SDA output back to 0\n");
         ar7240_gpio_out_val(CASBAH_SDA,0);         
      } 
		ar7240_gpio_config_output(CASBAH_SDA);
   }
}

static void casbah_setscl (void *data,int state)
{
	if (state)
		ar7240_gpio_config_input(CASBAH_SCL);
	else {
      if (ar7240_reg_rd(AR7240_GPIO_OUT) & (1 << CASBAH_SCL)) {
         printk("Casbah-I2C: force SCL output back to 0\n");
         ar7240_gpio_out_val(CASBAH_SCL,0);         
      }
		ar7240_gpio_config_output(CASBAH_SCL);
   }
}

static int casbah_getsda (void *data)
{
	return ((ar7240_gpio_in_val(CASBAH_SDA)) != 0);
}

static int casbah_getscl (void *data)
{
	return ((ar7240_gpio_in_val(CASBAH_SCL)) != 0);
}

static struct i2c_algo_bit_data bit_casbah_data = {
	.setsda		= casbah_setsda,
	.setscl		= casbah_setscl,
	.getsda		= casbah_getsda,
	.getscl		= casbah_getscl,
	.udelay		= 80,
	.mdelay		= 80,
	.timeout	= HZ
};

static struct i2c_adapter casbah_ops = {
	.owner			= THIS_MODULE,
	.name			= "Casbah I2C",
	.id			= I2C_HW_B_CASBAH,
	.algo_data		= &bit_casbah_data,
};

static int __init i2c_casbah_init (void)
{
	printk(KERN_INFO "Casbah I2C driver\n");
	ar7240_reg_rmw_clear(AR7240_GPIO_FUNCTIONS,0x78);
	ar7240_gpio_config_input(CASBAH_SCL);
	ar7240_gpio_config_input(CASBAH_SDA);
	ar7240_gpio_out_val(CASBAH_SCL,0);
	ar7240_gpio_out_val(CASBAH_SDA,0);
	ar7240_gpio_out_val(CASBAH_AUTHIC_RESET,0);
	ar7240_gpio_config_output(CASBAH_AUTHIC_RESET);
	mdelay(100);
	ar7240_gpio_out_val(CASBAH_AUTHIC_RESET,1);
	mdelay(100);
	printk(KERN_INFO "Reset authentication IC\n");
	printk(KERN_INFO "GPIO_FUNCTIONS=%08x GPIO_OE=%08x GPIO_IN=%08x GPIO_OUT=%08x\n",ar7240_reg_rd(AR7240_GPIO_FUNCTIONS),ar7240_reg_rd(AR7240_GPIO_OE),ar7240_reg_rd(AR7240_GPIO_IN),ar7240_reg_rd(AR7240_GPIO_OUT));
	return i2c_bit_add_bus(&casbah_ops);
}

static void __exit i2c_casbah_exit (void)
{
   printk("Casbah I2C driver exit\n");
	i2c_bit_del_bus(&casbah_ops);
}

MODULE_AUTHOR ("Sonos Inc.");
MODULE_DESCRIPTION ("I2C-Bus adapter routines for Casbah");
MODULE_LICENSE ("GPL");

module_init (i2c_casbah_init);
module_exit (i2c_casbah_exit);

