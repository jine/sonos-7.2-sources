/* drivers/char/backlight.c
 * 
 * Author:	Yin, Fengwei
 * Created:	Jan 12, 2007
 * Copyright:	Marvell International Ltd. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/micco.h>

#define KEYPAD_LED_MAX	0x7f
#define	KEYPAD_LED_MIN	0x0
#define	LCD_LED_MAX	0x7f
/* If the LCD backlight is set to 0, There is no any information on LCD.
 * So set it to 2 is OK for power saving.
 */
#define	LCD_LED_MIN	0x02

void lcd_led_timer_func(unsigned long par);
void keypad_led_timer_func(unsigned long par);

static struct timer_list lcd_led_timer =
		TIMER_INITIALIZER(lcd_led_timer_func, 0UL , 0UL);

static struct timer_list keypad_led_timer =
		TIMER_INITIALIZER(keypad_led_timer_func, 0UL , 0UL);

static int lcd_backlight_on = 0, keypad_backlight_on = 0;
static int lcd_bl_delay = 60 * HZ;
static int keypad_bl_delay = 60 * HZ;

/* Here is the usage mode of this driver:
 * 1. When keypad is press, the keypad and LCD backlight should be turned on.
 * 2. When touch is press, the LCD backlight should be turned on.
 * 3. If no activity on keypad/touch for a while, the backlight will be 
 *    turned off.
 */

void keypad_led_on(void)
{
	/* Workaround for keypad backlight */
	unsigned long flags;

	local_irq_save(flags);
	micco_write(0x90, 0x01);
	micco_write(0xB0, 0x08);

	/* Turn on the LED for keypad */
	micco_write(MICCO_LED1_CONTROL, KEYPAD_LED_MAX);
	micco_write(MICCO_LED2_CONTROL, KEYPAD_LED_MAX);
	keypad_backlight_on = 1;
	local_irq_restore(flags);
}

void keypad_led_off(void)
{
	unsigned long flags;

	local_irq_save(flags);
	/* Turn off the LED for keypad */
	micco_write(MICCO_LED1_CONTROL, 0x0);
	micco_write(MICCO_LED2_CONTROL, 0x0);

	/* Workaround for Keypad backlight */
	micco_write(0xB0, 0x00); /* Disable Boost on WLED drive circuit */
	micco_write(0x90, 0x00); /* Disable test registers */
	keypad_backlight_on = 0; 
	local_irq_restore(flags);
}

void lcd_led_on(void)
{
	unsigned long flags;

	local_irq_save(flags);
	micco_write(MICCO_WLED_CONTROL1, LCD_LED_MAX);
	lcd_backlight_on = 1;	
	local_irq_restore(flags);
}

void lcd_led_dim(void)
{
	unsigned long flags;

	local_irq_save(flags);
	micco_write(MICCO_WLED_CONTROL1, LCD_LED_MIN);
	lcd_backlight_on = 0;	
	local_irq_restore(flags);
}

void lcd_backlight_power_on(void)
{
	unsigned long flags;
	unsigned char val;

	local_irq_save(flags);
	/* turn on BOOST_EN to enable lcd backlight */
	micco_read(MICCO_WLED_CONTROL2, &val);
	val |= (1 << 5);
	micco_write(MICCO_WLED_CONTROL2, val);
	local_irq_restore(flags);
}

void lcd_backlight_power_off(void)
{
	unsigned long flags;
	unsigned char val;

	local_irq_save(flags);
	/* turn off BOOST_EN to disable lcd backlight */
	micco_read(MICCO_WLED_CONTROL2, &val);
	val &= ~(1 << 5);
	micco_write(MICCO_WLED_CONTROL2, val);
	local_irq_restore(flags);
}

void led_touch_press(void)
{
	if (!lcd_backlight_on)
		lcd_led_on();
	mod_timer(&lcd_led_timer, jiffies + lcd_bl_delay);
}

void led_keypad_press(void)
{
	if (!lcd_backlight_on)
		lcd_led_on();

	if (!keypad_backlight_on)
		keypad_led_on();

	mod_timer(&lcd_led_timer, jiffies + lcd_bl_delay);
	mod_timer(&keypad_led_timer, jiffies + keypad_bl_delay);
}

void lcd_led_timer_func(unsigned long par)
{
	lcd_led_dim();
}

void keypad_led_timer_func(unsigned long par)
{
	keypad_led_off();
}

static int __init backlight_init(void)
{
	init_timer(&lcd_led_timer);	
	init_timer(&keypad_led_timer);	

	/* Just turn on the LCD LED when initialization */
	led_touch_press();
	return 0;
}

static void __exit backlight_exit(void)
{
	del_timer(&keypad_led_timer);
	del_timer(&lcd_led_timer);
	lcd_led_timer_func(0);
	keypad_led_timer_func(0);
	return;
}

module_init(backlight_init);
module_exit(backlight_exit);
MODULE_LICENSE("GPL");
