/*
 * include/asm-arm/arch-pxa/pxa3xx_gpio.h
 *
 * Copyright (C) 2006, Marvell International.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_PXA3xx_GPIO_H__
#define __ASM_ARCH_PXA3xx_GPIO_H__

#define GPIO_ID_START		0
#define GPIO_ID_END		127

#ifdef CONFIG_PXA3xx_GPIOEX

#define GPIO_EXP_START		(GPIO_ID_END + 1)
#define GPIO_EXP0_START		(GPIO_ID_END + 1)
#define GPIO_EXP0_HALF		(GPIO_EXP0_START + 8)
#define GPIO_EXP0_END		(GPIO_EXP0_START + 15)
#define GPIO_EXP1_START		(GPIO_EXP0_END + 1)
#define GPIO_EXP1_HALF		(GPIO_EXP1_START + 8)
#define GPIO_EXP1_END		(GPIO_EXP1_START + 15)

#define GPIO_EXP0_ADDRESS	0x74
#define GPIO_EXP1_ADDRESS	0x75

#ifdef CONFIG_PXA_MWB_12
#define GPIO_EXP_NUM          	3
#define GPIO_EXP2_ADDRESS	0x76
#define GPIO_EXP2_START         (GPIO_EXP1_END + 1)
#define GPIO_EXP2_HALF          (GPIO_EXP2_START + 8)
#define GPIO_EXP2_END           (GPIO_EXP2_START + 15)
#define GPIO_EXP_END		(GPIO_EXP2_END)
#else
#define GPIO_EXP_NUM         	2
#define GPIO_EXP_END		(GPIO_EXP1_END)
#endif

#define GPIO_ID_MAX		(GPIO_EXP_END)

extern int gpio_exp_set_direction(int gpio_exp_id, int direction);
extern int gpio_exp_get_direction(int gpio_exp_id);
extern int gpio_exp_set_level(int gpio_exp_id, int level);
extern int gpio_exp_get_level(int gpio_exp_id);

#else

#define GPIO_ID_MAX		GPIO_ID_END

#endif

#define GPIO_DIR_IN		0
#define GPIO_DIR_OUT		1
#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1

#define MFP2GPIO(mfp) ((mfp) & 0xFFFF)

extern int pxa3xx_gpio_set_direction(int gpio_id, int dir);
extern int pxa3xx_gpio_get_direction(int gpio_id);
extern int pxa3xx_gpio_set_level(int gpio_id, int level);
extern int pxa3xx_gpio_get_level(int gpio_id);
extern int pxa3xx_gpio_set_rising_edge_detect(int gpio_id, int enable);
extern int pxa3xx_gpio_get_rising_edge_detect(int gpio_id);
extern int pxa3xx_gpio_set_falling_edge_detect(int gpio_id, int enable);
extern int pxa3xx_gpio_get_falling_edge_detect(int gpio_id);
extern int pxa3xx_gpio_get_edge_detect_status(int gpio_id);
extern int pxa3xx_gpio_clear_edge_detect_status(int gpio_id);

extern void pxa3xx_gpio_save(void);
extern void pxa3xx_gpio_restore(void);

#endif /* __ASM_ARCH_PXA3xx_GPIO_H__ */
