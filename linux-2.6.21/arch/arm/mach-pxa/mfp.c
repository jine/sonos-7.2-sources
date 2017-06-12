/*
 *  arch/arm/mach-pxa/mfp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/spinlock.h>

#include <asm/io.h>
#include <asm/types.h>
#include <asm/hardware.h>
#include <asm/arch/mfp.h>

#define CONFIG_MFP_DEBUG

#define MAX_MFP_PINS	419

struct mfp_regs {
	unsigned char __iomem	*membase;
	unsigned int		mfp[MAX_MFP_PINS];
};

/* Writing to an MFP register is slow, a read-back of the register is
 * necessary for most functions here to make sure that the write is
 * finished. For configuration of multiple MFP pins, a read-back of the
 * last register is enough.
 */

/* mfp_spin_lock is used to ensure that MFP register configuration
 * (most likely a read-modify-write operation) is atomic.
 */
static spinlock_t mfp_spin_lock = SPIN_LOCK_UNLOCKED;

int pxa3xx_mfp_set_config(struct pxa3xx_pin_config *pin_config)
{
	unsigned int mfp_pin;
	uint32_t  mfp_reg;
	unsigned long flags;

	BUG_ON(pin_config == NULL);

	mfp_pin = pin_config->mfp_pin;
	mfp_reg = PIN2REG(pin_config);

	BUG_ON(MFP_OFFSET(mfp_pin) > PXA3xx_MAX_MFP_OFFSET);
	BUG_ON(MFP_OFFSET(mfp_pin) < PXA3xx_MIN_MFP_OFFSET);

	spin_lock_irqsave(&mfp_spin_lock, flags);
#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(mfp_pin)){
		spin_unlock_irqrestore(&mfp_spin_lock,flags);
		return 0;
	}
#endif

	MFP_REG(mfp_pin) = mfp_reg;
	wmb();

	spin_unlock_irqrestore(&mfp_spin_lock,flags);

	return 0;
}

int pxa3xx_mfp_set_configs(struct pxa3xx_pin_config *pin_configs, int n)
{
	int ret = 0;

	while (n--) {
		ret = pxa3xx_mfp_set_config(pin_configs++);

		if (ret < 0)
			break;
	}

	return ret;
}

int pxa3xx_mfp_set_afds(unsigned int pin, int af, int ds)
{
	uint32_t mfp_reg;
	unsigned long flags;

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif
	spin_lock_irqsave(&mfp_spin_lock, flags);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~(MFP_AF_MASK | MFP_DRV_MASK);
	mfp_reg |= (((af & 0x7) << MFPR_ALT_OFFSET) |
		    ((ds & 0x7) << MFPR_DRV_OFFSET));
	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	spin_unlock_irqrestore(&mfp_spin_lock, flags);

	return 0;
}

int pxa3xx_mfp_set_rdh(unsigned int pin, int rdh)
{
	uint32_t mfp_reg;
	unsigned long flags;

	BUG_ON(MFP_OFFSET(pin) > PXA3xx_MAX_MFP_OFFSET);
	BUG_ON(MFP_OFFSET(pin) < PXA3xx_MIN_MFP_OFFSET);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

	spin_lock_irqsave(&mfp_spin_lock, flags);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~MFP_RDH_MASK;

	if (likely(rdh))
		mfp_reg |= (1u << MFPR_SS_OFFSET);

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	spin_unlock_irqrestore(&mfp_spin_lock, flags);

	return 0;
}

int pxa3xx_mfp_set_lpm(unsigned int pin, int lpm)
{
	uint32_t mfp_reg;
	unsigned long flags;

	BUG_ON(MFP_OFFSET(pin) > PXA3xx_MAX_MFP_OFFSET);
	BUG_ON(MFP_OFFSET(pin) < PXA3xx_MIN_MFP_OFFSET);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

	spin_lock_irqsave(&mfp_spin_lock, flags);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~(MFP_LPM_MASK);

	if (lpm & 0x1) mfp_reg |= 1u << MFPR_SON_OFFSET;
	if (lpm & 0x2) mfp_reg |= 1u << MFPR_SD_OFFSET;
	if (lpm & 0x4) mfp_reg |= 1u << MFPR_PU_OFFSET;
	if (lpm & 0x8) mfp_reg |= 1u << MFPR_PD_OFFSET;
	if (lpm &0x10) mfp_reg |= 1u << MFPR_PS_OFFSET;

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	spin_unlock_irqrestore(&mfp_spin_lock, flags);

	return 0;
}

int pxa3xx_mfp_set_edge(unsigned int pin, int edge)
{
	uint32_t mfp_reg;
	unsigned long flags;

	BUG_ON(MFP_OFFSET(pin) > PXA3xx_MAX_MFP_OFFSET);
	BUG_ON(MFP_OFFSET(pin) < PXA3xx_MIN_MFP_OFFSET);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

	spin_lock_irqsave(&mfp_spin_lock, flags);

	mfp_reg = MFP_REG(pin);

	/* Clear bits - EDGE_CLEAR, EDGE_RISE_EN, EDGE_FALL_EN */
	mfp_reg &= ~(MFP_EDGE_MASK);

	switch (edge) {
	case MFP_EDGE_RISE:
		mfp_reg |= (1u << MFPR_ERE_OFFSET);
		break;
	case MFP_EDGE_FALL:
		mfp_reg |= (1u << MFPR_EFE_OFFSET);
		break;
	case MFP_EDGE_BOTH:
		mfp_reg |= (3u << MFPR_ERE_OFFSET);
		break;
	case MFP_EDGE_NONE:
		mfp_reg |= (1u << MFPR_EC_OFFSET);
		break;
	default:
		spin_unlock_irqrestore(&mfp_spin_lock, flags);
		return -EINVAL;
	}

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	spin_unlock_irqrestore(&mfp_spin_lock, flags);

	return 0;
}

/*
 * The pullup and pulldown state of the MFP pin is by default determined by
 * selected alternate function. In case some buggy devices need to override
 * this default behavior,  this function can be invoked by setting/clearing
 * bit PULL_SEL of MFPRxx.
 */
int pxa3xx_mfp_set_pull(unsigned int pin, int pull)
{
	uint32_t mfp_reg;
	unsigned long flags;

	BUG_ON(MFP_OFFSET(pin) > PXA3xx_MAX_MFP_OFFSET);
	BUG_ON(MFP_OFFSET(pin) < PXA3xx_MIN_MFP_OFFSET);

#if defined(CONFIG_PXA3xx_GPIOEX)
	if (IS_GPIO_EXP_PIN(pin))
		return 0;
#endif

	spin_lock_irqsave(&mfp_spin_lock, flags);

	mfp_reg = MFP_REG(pin);
	mfp_reg &= ~MFP_PULL_MASK;

	mfp_reg |= (pull & 0x7u) << MFPR_PD_OFFSET;

	MFP_REG(pin) = mfp_reg;
	mfp_reg = MFP_REG(pin);

	spin_unlock_irqrestore(&mfp_spin_lock, flags);

	return 0;
}

static struct mfp_regs context;
void pxa3xx_mfp_save(void)
{
	int i, offset;

	/* specify the membase */
	context.membase = (unsigned char *)&MFP_REG(0);

	for (i = 0; i < MAX_MFP_PINS; i++) {
		offset = i << 2;
		context.mfp[i] = readl(context.membase + offset);
	}
}

void pxa3xx_mfp_restore(void)
{
	int i, offset;

	/* check the membase */
	if (context.membase == NULL)
		return;

	for (i = 0; i < MAX_MFP_PINS; i++) {
		offset = i << 2;
		writel(context.mfp[i], context.membase + offset);
	}
}

EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_config);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_configs);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_afds);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_rdh);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_lpm);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_edge);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_set_pull);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_save);
EXPORT_SYMBOL_GPL(pxa3xx_mfp_restore);

