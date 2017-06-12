/*
 * drivers/i2c/chips/gpio-expander.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel_stat.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>

#ifdef CONFIG_PXA_MWB_12
#define GPIO_EXP_END            (GPIO_EXP2_END)
#else
#define GPIO_EXP_END		(GPIO_EXP1_END)
#endif

#define TO_idx(x)	((x) >> 4)
#define TO_bit(x)	((x) & 0xf)

struct pxa_gpio_exp {
	struct i2c_client i2c_client;
	struct delayed_work work;
	unsigned long rising_edge;
	unsigned long falling_edge;
	unsigned long input_level;
	int addr;
	int irq;
	int gpio_start;
	int gpio_end;
	int id;
	uint8_t regs[8];
};

static int nr = 0;
static struct pxa_gpio_exp *g_gpio_exp[GPIO_EXP_NUM];

extern spinlock_t irq_controller_lock;

static int gpio_exp_read(struct pxa_gpio_exp *gpio_exp, uint8_t reg, uint8_t *val);
static int gpio_exp_write(struct pxa_gpio_exp *gpio_exp, uint8_t reg, uint8_t val);

static void fastcall
pxa_gpio_ext_interrupt(unsigned int irq, struct irq_desc *desc)
{
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && g_gpio_exp[i]->irq == irq) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return;

	desc->chip->ack(irq);
	desc->chip->mask(irq);
	schedule_delayed_work(&gpio_exp->work, 20);
	desc->chip->unmask(irq);
	return;
}

static void gpio_exp_work(struct work_struct *work)
{
	struct pxa_gpio_exp *gpio_exp;
	struct delayed_work *dwork;
	unsigned int i;
	unsigned long flags;
	struct irq_desc *desc;
	u16 cur_gpio_ext_input;
	u8 tmp0, tmp1;
	int bit;
	int gpio_ext_irq_pending = 0;
	const unsigned int cpu = smp_processor_id();

	dwork = container_of(work, struct delayed_work, work);
	gpio_exp = container_of(dwork, struct pxa_gpio_exp, work);

	disable_irq(gpio_exp->irq);
	gpio_exp_read(gpio_exp, 0, &tmp0);
	gpio_exp_read(gpio_exp, 1, &tmp1);
	cur_gpio_ext_input = tmp0 | (tmp1 << 8);

	for (i = gpio_exp->gpio_start; i < gpio_exp->gpio_end; i++) {
		gpio_ext_irq_pending = 0;
		bit = TO_bit(i - gpio_exp->gpio_start);
		if (gpio_exp->rising_edge & (1 << bit)) {
			if ((!(gpio_exp->input_level & (1 << bit))) &&
			    (cur_gpio_ext_input & (1 << bit)))
				gpio_ext_irq_pending = 1;
		}

		if (gpio_exp->falling_edge & (1 << bit)) {
			if ((gpio_exp->input_level & (1 << bit)) &&
			    (!(cur_gpio_ext_input & (1 << bit))))
				gpio_ext_irq_pending = 1;
		}

		if (gpio_ext_irq_pending) {
			desc = irq_desc + GPIO_EXT_TO_IRQ(i);
			irq_enter();

			spin_lock_irqsave(&irq_controller_lock, flags);
		
			kstat_cpu(cpu).irqs[GPIO_EXT_TO_IRQ(i)]++;	
			desc->handle_irq(GPIO_EXT_TO_IRQ(i), desc);

			spin_unlock_irqrestore(&irq_controller_lock, flags);
			irq_exit();
		}
	}

	gpio_exp->input_level = cur_gpio_ext_input;

	enable_irq(gpio_exp->irq);
}

static int pxa_gpio_expander_irq_type(unsigned int irq, unsigned int type)
{
	int gpio_ext, gpio_ext_bit;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	gpio_ext = IRQ_TO_GPIO_EXT(irq);

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] &&
		    g_gpio_exp[i]->gpio_start <= gpio_ext &&
		    g_gpio_exp[i]->gpio_end >= gpio_ext) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;

	gpio_ext_bit = (gpio_ext - gpio_exp->gpio_start) & 0x0f;
	if (type == IRQT_PROBE) {
		if ((gpio_exp->rising_edge | gpio_exp->falling_edge)
		    & (1 << gpio_ext_bit))
			return 0;

		type = __IRQT_RISEDGE | __IRQT_FALEDGE;
	}

	if (type & __IRQT_RISEDGE) {
		__set_bit(gpio_ext_bit, &gpio_exp->rising_edge);
	} else
		__clear_bit(gpio_ext_bit, &gpio_exp->rising_edge);

	if (type & __IRQT_FALEDGE) {
		__set_bit(gpio_ext_bit, &gpio_exp->falling_edge);
	} else
		__clear_bit(gpio_ext_bit, &gpio_exp->falling_edge);

	return 0;
}

static void pxa_ack_gpio_expander(unsigned int irq)
{
	return;
}

static void pxa_mask_gpio_expander(unsigned int irq)
{
	return;
}

static void pxa_unmask_gpio_expander(unsigned int irq)
{
	return;
}

static struct irq_chip pxa_gpio_expander_chip = {
	.ack      = pxa_ack_gpio_expander,
	.mask     = pxa_mask_gpio_expander,
	.unmask   = pxa_unmask_gpio_expander,
	.set_type = pxa_gpio_expander_irq_type,
};

int gpio_exp_set_direction(int gpio_exp_id, int dir)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] &&
		    gpio_exp_id <= g_gpio_exp[i]->gpio_end &&
		    gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;

	bit = gpio_exp_id - gpio_exp->gpio_start;
	
	if (bit	< (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2)
		reg = 6;
	else {
		reg = 7;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	gpio_exp_read(gpio_exp, reg, &tmp);
	if (dir == GPIO_DIR_IN)
		tmp |=  (0x01u << bit);
	else
		tmp &= ~(0x01u << bit);

	gpio_exp_write(gpio_exp, reg, tmp);

	return 0;
}

int gpio_exp_get_direction(int gpio_exp_id)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] &&
		    gpio_exp_id <= g_gpio_exp[i]->gpio_end &&
		    gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;

	bit = gpio_exp_id - gpio_exp->gpio_start;
	
	if (bit	< (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2)
		reg = 6;
	else {
		reg = 7;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	gpio_exp_read(gpio_exp, reg, &tmp);

	if (tmp & (1u << bit))
		return GPIO_DIR_IN;
	else
		return GPIO_DIR_OUT;

}

int gpio_exp_set_level(int gpio_exp_id, int level)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] &&
		    gpio_exp_id <= g_gpio_exp[i]->gpio_end &&
		    gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;

	bit = gpio_exp_id - gpio_exp->gpio_start;
	
	if (bit	< (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2)
		reg = 2;
	else {
		reg = 3;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	gpio_exp_read(gpio_exp, reg, &tmp);

	if (level == GPIO_LEVEL_LOW)
		tmp &= ~(0x01 << bit);
	else
		tmp |=  (0x01 << bit);

	gpio_exp_write(gpio_exp, reg, tmp);

	return 0;
}

int gpio_exp_get_level(int gpio_exp_id)
{
	uint8_t tmp, bit, reg;
	int i = 0;
	struct pxa_gpio_exp *gpio_exp = NULL;

	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] &&
		    gpio_exp_id <= g_gpio_exp[i]->gpio_end &&
		    gpio_exp_id >= g_gpio_exp[i]->gpio_start) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -EINVAL;

	bit = gpio_exp_id - gpio_exp->gpio_start;
	
	if (bit	< (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2)
		reg = 0;
	else {
		reg = 1;
		bit -= (gpio_exp->gpio_end - gpio_exp->gpio_start + 1) / 2;
	}
	gpio_exp_read(gpio_exp, reg, &tmp);
	if (tmp & (1u << bit))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}

EXPORT_SYMBOL_GPL(gpio_exp_set_direction);
EXPORT_SYMBOL_GPL(gpio_exp_get_direction);
EXPORT_SYMBOL_GPL(gpio_exp_set_level);
EXPORT_SYMBOL_GPL(gpio_exp_get_level);

#ifdef CONFIG_PM

/*
 * GPIO Expander will be reset during suspend/resume, registers should
 * be saved/restored for proper operation
 */

static int gpio_exp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa_gpio_exp *gpio_exp = platform_get_drvdata(pdev);
	int i;

	/* skip saving input level register 0, 1
	 */
	for (i = 2; i < 8; i++)
		gpio_exp_read(gpio_exp, i, &gpio_exp->regs[i]);

	/* Currently USB_OTG_EN and USB_OTG_SR will generate unpredictable
	 * interrupts when system enters low power mode, changing them to 
	 * output temporarily solves this problem.
	 */
	if (gpio_exp->id == 0) {
		gpio_exp_write(gpio_exp, 7, 0x0F);
	}

	return 0;
}

static int gpio_exp_resume(struct platform_device *pdev)
{
	struct pxa_gpio_exp *gpio_exp = platform_get_drvdata(pdev);
	uint8_t tmp0, tmp1;
	int i;

	/* skip restoring input level register 0, 1
	 */
	for (i = 2; i < 8; i++)
		gpio_exp_write(gpio_exp, i, gpio_exp->regs[i]);

	/* read the input level registers to
	 * 1) clock the input to internal registers
	 * 2) clear interrupt if any
	 * 3) record the current level
	 */
	gpio_exp_read(gpio_exp, 0, &tmp0);
	gpio_exp_read(gpio_exp, 1, &tmp1);
	gpio_exp->input_level = tmp0 | (tmp1 << 8);

	return 0;
}
#endif

static int __init gpio_exp_probe(struct platform_device *pdev)
{
	uint8_t  tmp0, tmp1;
	struct resource *r;
	struct pxa_gpio_exp *gpio_exp = NULL;
	int i = 0, gpio, addr;

	printk(KERN_INFO "GPIO-expander driver for Monahans\n");

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENXIO;
	addr = r->start;
	while (i < GPIO_EXP_NUM) {
		if (g_gpio_exp[i] && g_gpio_exp[i]->addr == addr) {
			gpio_exp = g_gpio_exp[i];
			break;
		}
		i++;
	}
	if (!gpio_exp)
		return -ENXIO;


	/* Initialize the MFPR and GPIO settings of the two pins
	 * connected to the GPIO expander
	 */
	gpio_exp->irq = platform_get_irq(pdev, 0);
	/* Some GPIO expander never trigger interrupt */
	if (gpio_exp->irq < 0)
		gpio_exp->irq = 0;

	r = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!r)
		return -ENXIO;
	gpio_exp->gpio_start = r->start;
	gpio_exp->gpio_end = r->end;

	if (gpio_exp->irq == 0)
		goto exit;

	INIT_DELAYED_WORK(&gpio_exp->work, gpio_exp_work);

	/* read in the initial pin levels
	 */
	tmp0 = tmp1 = 0;
	gpio_exp_read(gpio_exp, 0, &tmp0);
	gpio_exp_read(gpio_exp, 1, &tmp1);
	gpio_exp->input_level = tmp0 | (tmp1 << 8);

	for (gpio = gpio_exp->gpio_start; gpio < gpio_exp->gpio_end; gpio++) {
		set_irq_chip(GPIO_EXT_TO_IRQ(gpio), &pxa_gpio_expander_chip);
		set_irq_handler(GPIO_EXT_TO_IRQ(gpio), handle_edge_irq);
		set_irq_flags(GPIO_EXT_TO_IRQ(gpio), IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(gpio_exp->irq, IRQT_FALLING);
	set_irq_chained_handler(gpio_exp->irq, pxa_gpio_ext_interrupt);
exit:
	platform_set_drvdata(pdev, gpio_exp);

	return 0;
}

static struct platform_driver gpio_exp_driver = {
	.probe		= gpio_exp_probe,
#ifdef CONFIG_PM
	.suspend	= gpio_exp_suspend,
	.resume		= gpio_exp_resume,
#endif
	.driver		= {
		.name	= "gpio-exp",
	},
};


/*****************************************************************************
 * GPIO Expander I2C Client Driver
 ****************************************************************************/
#include <linux/i2c.h>
static int i2c_gpio_exp_attach_adapter(struct i2c_adapter *adapter);
static int i2c_gpio_exp_detect_client(struct i2c_adapter *, int, int);
static int i2c_gpio_exp_detach_client(struct i2c_client *client);

struct i2c_driver i2c_gpio_exp_driver = {
	.driver		= {
		.name	= "gpio expander i2c client driver",
	},
	.attach_adapter	= &i2c_gpio_exp_attach_adapter,
	.detach_client	= &i2c_gpio_exp_detach_client,
};

/* Unique ID allocation */
#ifdef CONFIG_PXA_MWB_12
static unsigned short normal_i2c[] = {
	GPIO_EXP0_ADDRESS,
	GPIO_EXP1_ADDRESS,
	GPIO_EXP2_ADDRESS,
	I2C_CLIENT_END};
#else
static unsigned short normal_i2c[] = {
	GPIO_EXP0_ADDRESS,
	GPIO_EXP1_ADDRESS,
	I2C_CLIENT_END};
#endif
I2C_CLIENT_INSMOD_1(gpioexp);

static int gpio_exp_write(struct pxa_gpio_exp *gpio_exp,
			  uint8_t reg, uint8_t val)
{
	return i2c_smbus_write_byte_data(&gpio_exp->i2c_client, reg, val);
}

static int gpio_exp_read(struct pxa_gpio_exp *gpio_exp,
			 uint8_t reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(&gpio_exp->i2c_client, reg);
	if (ret >= 0) {
		*val = ret;
		return 0;
	} else
		return -EIO;
}

static int i2c_gpio_exp_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client,reg);
}

static int i2c_gpio_exp_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &i2c_gpio_exp_detect_client);
}

static int i2c_gpio_exp_detect_client(struct i2c_adapter *adapter,
				int address, int kind)
{
	struct pxa_gpio_exp *gpio_exp;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_INFO "byte op is not permited.\n");
		goto ERROR0;
	}

	if (nr >= GPIO_EXP_NUM) {
		err = -ENOMEM;
		goto ERROR0;
	}
	gpio_exp = kzalloc(sizeof(struct pxa_gpio_exp), GFP_KERNEL);

	if (!gpio_exp)  {
		err = -ENOMEM;
		goto ERROR0;
	}

	gpio_exp->i2c_client.addr = address;
 	gpio_exp->i2c_client.adapter = adapter;
	gpio_exp->i2c_client.driver = &i2c_gpio_exp_driver;
	gpio_exp->i2c_client.flags = 0;

	if (i2c_gpio_exp_read(&gpio_exp->i2c_client, 0) < 0)
		goto ERROR1;
	else
		printk(KERN_INFO "GPIO expander %d detected!\n", nr);
	
	sprintf(gpio_exp->i2c_client.name, "GPIO Expander %d", nr);

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(&gpio_exp->i2c_client)))
		goto ERROR1;

	gpio_exp->id = nr;
	gpio_exp->addr = address;
	g_gpio_exp[nr++] = gpio_exp;
	return 0;

ERROR1:
	kfree(gpio_exp);
ERROR0:
        return err;
}

static int i2c_gpio_exp_detach_client(struct i2c_client *client)
{
	int err;
	int i;
	struct pxa_gpio_exp *gpio_exp
		= container_of(client, struct pxa_gpio_exp, i2c_client);

  	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk(KERN_ERR "gpio_exp: client deregistration failed.\n");
	        return err;
	}

	for (i = 0; i < GPIO_EXP_NUM; i++){
		if (g_gpio_exp[i] == gpio_exp)
			g_gpio_exp[i] = NULL;
	}

	kfree(gpio_exp);
	return 0;
}


static int __init pxa_gpio_exp_init(void)
{
	int ret;
	
	if ((ret = i2c_add_driver(&i2c_gpio_exp_driver))) {
		printk(KERN_ERR "gpio_exp: driver registration failed.\n");
		return ret;
	}

	return platform_driver_register(&gpio_exp_driver);
}

static void __exit pxa_gpio_exp_exit(void)
{
	platform_driver_unregister(&gpio_exp_driver);
	if (i2c_del_driver(&i2c_gpio_exp_driver))
		printk(KERN_ERR "gpio_exp: driver unregistration failed.\n");
}

MODULE_DESCRIPTION("GPIO Expander Driver");
MODULE_LICENSE("GPL");

module_init(pxa_gpio_exp_init);
module_exit(pxa_gpio_exp_exit);

