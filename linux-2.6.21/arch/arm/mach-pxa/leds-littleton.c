/*
 * linux/arch/arm/mach-pxa/leds-littleton.c
 *
 * (C) Copyright 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/init.h>

#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/system.h>

#include <asm/arch/pxa-regs.h>
#include <asm/arch/littleton.h>
#include <asm/arch/mfp.h>
#include <asm/arch/gpio.h>

#include "leds.h"

#define LED_STATE_ENABLED	1
#define LED_STATE_CLAIMED	2

/* Define Littleton LEDs here */

#define LED_TIMER 1
#define LED_CPU 4

static unsigned int led_state;
static unsigned int led_timer_state;
static unsigned int led_cpu_state;

static inline void set_led(int led, int state)
{
	if (state) {
		mhn_gpio_set_level(led, GPIO_LEVEL_HIGH);
	} else {
		mhn_gpio_set_level(led, GPIO_LEVEL_LOW);
	}
}

void littleton_leds_event(led_event_t evt)
{
	unsigned long flags;

	local_irq_save(flags);

	switch(evt) {

		case led_start:

			/* 
			 * make sure the GPIOs for the 
			 * LEDS are set up as outputs
			 */
			mhn_gpio_set_direction(LED_TIMER, GPIO_DIR_OUT);
			mhn_gpio_set_direction(LED_CPU, GPIO_DIR_OUT);

			led_timer_state = 1;
			led_cpu_state = 0;
			led_state = LED_STATE_ENABLED;

			break;

		case led_stop:
			led_state &= ~LED_STATE_ENABLED;
			break;

		case led_claim:
			led_state |= LED_STATE_CLAIMED;
			led_timer_state = led_cpu_state = 0;
			break;

		case led_release:
			led_state &= ~LED_STATE_CLAIMED;
			led_timer_state = led_cpu_state = 0;
			break;

#ifdef CONFIG_LEDS_TIMER
		case led_timer:
			led_timer_state = (led_timer_state ? 0 : 1);
			break;
#endif

#ifdef CONFIG_LEDS_CPU
		case led_idle_start:
			led_cpu_state = 0;
			break;

		case led_idle_end:
			led_cpu_state = 1;
			break;
#endif  

		default:
			break;
	}
#if 0 
	/* update the LEDS */
	if (led_state & LED_STATE_ENABLED) {
		set_led(LED_TIMER, led_timer_state);
		set_led(LED_CPU, led_cpu_state);
	} else {
		set_led(LED_TIMER, 0);
		set_led(LED_CPU, 0);
	}
#endif
	local_irq_restore(flags);
}

