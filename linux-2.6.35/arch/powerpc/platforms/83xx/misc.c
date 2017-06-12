/*
 * misc setup functions for MPC83xx
 *
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <asm/hw_irq.h>
#include <sysdev/fsl_soc.h>

/* Need the SUBMODEL defines to set up gpio correctly */
#include <mdp.h>

#include "mpc83xx.h"
#ifdef CONFIG_SONOS_FENWAY
#include <asm/fenway-gpio.h>
#include <asm/mpc83xx-gpio.h>
#endif

#ifdef CONFIG_SONOS_FENWAY
/*
 * DUART
 */
typedef struct duart83xx {
	u8 urbr_ulcr_udlb;	/* combined register for URBR, UTHR and UDLB */
	u8 uier_udmb;		/* combined register for UIER and UDMB */
	u8 uiir_ufcr_uafr;	/* combined register for UIIR, UFCR and UAFR */
	u8 ulcr;		/* line control register */
	u8 umcr;		/* MODEM control register */
	u8 ulsr;		/* line status register */
	u8 umsr;		/* MODEM status register */
	u8 uscr;		/* scratch register */
	u8 res0[8];
	u8 udsr;		/* DMA status register */
	u8 res1[3];
	u8 res2[0xEC];
} duart83xx_t;
#define FCR_FIFO_EN     0x01    /*fifo enable*/
#define FCR_RXSR        0x02    /*reciever soft reset*/
#define FCR_TXSR        0x04    /*transmitter soft reset*/
#define MCR_DTR         0x01
#define MCR_RTS         0x02
#define LSR_THRE    0x20    /* Xmit holding register empty */
#define LCR_BKSE    0x80    /* Bank select enable*/
#define LCR_SBRK    0x40    /* Set Break*/
#define LCR_8N1     0x03
#define LCRVAL  LCR_8N1				/* 8 data, 1 stop, no parity */
#define MCRVAL (MCR_DTR | MCR_RTS)		/* RTS/DTR */
#define FCRVAL (FCR_FIFO_EN | FCR_RXSR | FCR_TXSR) /* Clear & enable FIFOs */

extern int fenway_submodel;

static __be32 __iomem *restart_reg_base;
static __be32 __iomem *gpio_reg_base;
static __be32 __iomem *sysconf_reg_base;
static __be32 __iomem *sepnr_ptr;
#endif	// CONFIG_SONOS_FENWAY

static int __init mpc83xx_restart_init(void)
{
	/* map reset restart_reg_baseister space */
	restart_reg_base = ioremap(get_immrbase() + 0x900, 0xff);

	return 0;
}

arch_initcall(mpc83xx_restart_init);

#ifdef CONFIG_SONOS_FENWAY
u32 mpc83xx_read_sepnr(void)
{
   // XXX BT this is terribly hacky
   return in_be32(sepnr_ptr);
}
EXPORT_SYMBOL(mpc83xx_read_sepnr);

/* reads GPIO config and data registers */
u32 mpc83xx_read_gpio(int reg)
{

   if (gpio_reg_base == NULL) {
		printk (KERN_EMERG "Error: GPIO registers not mapped, read aborted!\n");
      return 0;
   }
   if ((reg & 0x3) || (reg > MPC8315_GPIO_GPICR)) { 
		printk (KERN_EMERG "Error: bad GPIO register %08x, read aborted!\n", reg);
      return 0;
   }

   return in_be32((gpio_reg_base + (reg >> 2)));
}
EXPORT_SYMBOL(mpc83xx_read_gpio);

/* writes GPIO config and data registers */
void mpc83xx_write_gpio(int reg, u32 val)
{
   if (gpio_reg_base == NULL) {
 		printk (KERN_EMERG "Error: GPIO registers not mapped, write aborted!\n");
      return;
   }
   if ((reg & 0x3) || (reg > MPC8315_GPIO_GPICR)) { 
		printk (KERN_EMERG "Error: bad GPIO register %08x, write aborted!\n", reg);
      return;
   }

   out_be32((gpio_reg_base + (reg >> 2)), val);
}
EXPORT_SYMBOL(mpc83xx_write_gpio);

void mpc83xx_set_sicr(u32 sicrl_mask, u32 sicrl_bits, 
                      u32 sicrh_mask, u32 sicrh_bits)
{
   u32 sicrl, sicrh;

   sicrl = in_be32((sysconf_reg_base + (0x14 >> 2)));
   sicrl &= ~sicrl_mask;
   sicrl |= (sicrl_bits & sicrl_mask);

   sicrh = in_be32((sysconf_reg_base + (0x18 >> 2)));
   sicrh &= ~sicrh_mask;
   sicrh |= (sicrh_bits & sicrh_mask);

   out_be32(sysconf_reg_base + (0x14 >> 2), sicrl);
   out_be32(sysconf_reg_base + (0x18 >> 2), sicrh);
}
EXPORT_SYMBOL(mpc83xx_set_sicr);

void mpc83xx_get_sicr(u32 *sicrl_bits, u32 *sicrh_bits)
{
   *sicrl_bits = in_be32((sysconf_reg_base + (0x14 >> 2)));
   *sicrh_bits = in_be32((sysconf_reg_base + (0x18 >> 2)));
}
EXPORT_SYMBOL(mpc83xx_get_sicr);

// Anvil green is silly
void initGreenUart(int on)
{  
   volatile duart83xx_t   *green_uart;
  int baud_divisor;

  green_uart = ioremap(get_immrbase() + 0x00004600, 0x20);

  baud_divisor = (125000000 / 16 / 7200);	
  green_uart->uier_udmb = 0x00;
  green_uart->ulcr = LCR_BKSE | LCRVAL;
  green_uart->urbr_ulcr_udlb = baud_divisor & 0xff;
  green_uart->uier_udmb = (baud_divisor >> 8) & 0xff;
  green_uart->ulcr = LCRVAL;
  green_uart->umcr = MCRVAL;
  green_uart->uiir_ufcr_uafr = FCRVAL;
  if (0 == on) // turn it off by driving break low
    green_uart->ulcr = LCR_SBRK;
}

void fenway_gpio_init(void)
{
   u32 temp;
   u32 gpio_input_mask;
   u32 gpio_output_mask;
   u32 sicrh_bits;
   u32 sicrl_bits;

   if (MDP_SUBMODEL_IS_ANVIL(fenway_submodel)) {	//Anvil
      printk("Configuring button/LED GPIOs for Anvil submodel\n");
      gpio_input_mask = ANVIL_GPIO_BUTTONS_MASK | ANVIL_GPIO_AMP_OTW_PIN;
      gpio_output_mask = ANVIL_GPIO_LEDS_MASK | ANVIL_GPIO_AMP_RESET_MASK |
         ANVIL_GPIO_DAC_RESET_PIN | ANVIL_GPIO_AMP_POWER_PIN | ANVIL_GPIO_DAC_SMUTE_MASK;
      sicrh_bits = ANVIL_SICRH;
      sicrl_bits = ANVIL_SICRL;
      initGreenUart(0);
   } else if (MDP_SUBMODEL_IS_AMOEBA(fenway_submodel)) {	//Amoeba
      printk("Configuring GPIOs for Amoeba\n");
      gpio_input_mask = AMOEBA_GPIO_BUTTONS_MASK | AMOEBA_GPIO_MISC;
      gpio_output_mask = AMOEBA_GPIO_LEDS_MASK | AMOEBA_GPIO_AMP_RESET_MASK | \
			AMOEBA_GPIO_AMP_PDN_MASK | AMOEBA_GPIO_AMP_HIZ_MASK;
      sicrh_bits = AMOEBA_SICRH;
      sicrl_bits = AMOEBA_SICRL;
   } else {
      /* default to fenway */
      if (MDP_SUBMODEL_IS_FENWAY(fenway_submodel)) {
         printk("Configuring button/LED GPIOs for Fenway submodel\n");
      }
      else {
         printk("Configuring button/LED GPIOs for Fenway by default\n");
      }
      gpio_input_mask = FENWAY_GPIO_BUTTONS_MASK;
      gpio_output_mask = FENWAY_GPIO_LEDS_MASK;
      sicrh_bits = FENWAY_SICRH;
      sicrl_bits = FENWAY_SICRL;
   }

   /* Set the SICR-L & SICR-H registers for Fenway GPIO pins */
   if (sysconf_reg_base == NULL) {
      /* map the System Config register space */
      sysconf_reg_base = ioremap(get_immrbase() + 0x100, 0x20);
   }
   mpc83xx_set_sicr(0xF000000C, sicrl_bits, 0xFFFFFF00, sicrh_bits);

   /* map the GPIO registers if it hasn't been done yet */
   if (gpio_reg_base == NULL) {
      gpio_reg_base = ioremap(get_immrbase() + MPC8315_GPIO_OFF, 0x20);
   }

   if (!sepnr_ptr) sepnr_ptr = ioremap(get_immrbase() + 0x72C, 0x04);

   /* Setup the direction register, Button pins are inputs, LEDs are outputs */
   temp = mpc83xx_read_gpio(MPC8315_GPIO_GPDIR);
   temp &= ~gpio_input_mask;
   temp |=  gpio_output_mask;
   mpc83xx_write_gpio(MPC8315_GPIO_GPDIR, temp);

   /* Fenway does not have any open drain pins */
   mpc83xx_write_gpio(MPC8315_GPIO_GPODR, 0);

   /* Now that the LED pins are defined as outputs we'll let the LED timer
      function take care of the actual LED values. Thus we don't need to
      touch the GPDAT register. */
   if (MDP_SUBMODEL_IS_ANVIL(fenway_submodel)||MDP_SUBMODEL_IS_AMOEBA(fenway_submodel)){
	   /* make sure the fixed white LED pin is always zero */
	   temp = mpc83xx_read_gpio(MPC8315_GPIO_GPDAT);
	   temp &= (MDP_SUBMODEL_IS_ANVIL(fenway_submodel))?\
		   (~ANVIL_GPIO_LED_WHITE_FIXED_MASK):(~AMOEBA_GPIO_LED_WHITE_FIXED_MASK);
	   mpc83xx_write_gpio(MPC8315_GPIO_GPDAT, temp);
	   printk("anvil/amoeba: set fixed white LED pin to 0 in gpdat\n");
   }
}
#endif	// CONFIG_SONOS_FENWAY


void mpc83xx_restart(char *cmd)
{
#define RST_OFFSET	0x00000900
#define RST_PROT_REG	0x00000018
#define RST_CTRL_REG	0x0000001c

#ifdef	CONFIG_SONOS_FENWAY
   printk("RESTART: %p\n", restart_reg_base);
#endif	// CONFIG_SONOS_FENWAY

	local_irq_disable();

	if (restart_reg_base) {
		/* enable software reset "RSTE" */
		out_be32(restart_reg_base + (RST_PROT_REG >> 2), 0x52535445);

		/* set software hard reset */
		out_be32(restart_reg_base + (RST_CTRL_REG >> 2), 0x2);
	} else {
		printk (KERN_EMERG "Error: Restart registers not mapped, spinning!\n");
	}

	for (;;) ;
}

long __init mpc83xx_time_init(void)
{
#define SPCR_OFFSET	0x00000110
#define SPCR_TBEN	0x00400000
	__be32 __iomem *spcr = ioremap(get_immrbase() + SPCR_OFFSET, 4);
	__be32 tmp;

	tmp = in_be32(spcr);
	out_be32(spcr, tmp | SPCR_TBEN);

	iounmap(spcr);

	return 0;
}
