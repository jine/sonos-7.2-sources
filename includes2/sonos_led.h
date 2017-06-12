/*
 * Copyright (c) 2014, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * SONOS LED control
 *
 * Data structures shared between the kernel and user space
 */
#ifndef SONOS_LED_H
#define SONOS_LED_H

struct led_step {
    uint8_t	r;
    uint8_t	g;
    uint8_t	b;
    uint8_t	reserved;	// Must be set to 0

    uint16_t	hold_time;	// How long to maintain this color, 0 for forever
    uint16_t	fade;		// How long to fade FROM the previous color, 0 for instant
};

struct led_pattern {
    uint16_t		flags;
    uint8_t		repeats;	// How many times to repeat, or 0 for forever
    uint8_t		num_steps;	// Number of colors, or 0 for all LEDs off
    struct led_step	*step_array;	// Must be last in this struct definition
};

/* Flags for use in the pattern.flags field */
/* LED_CTL_DEFAULT will replace the currently running pattern after
 * the current step finishes. LED_CTL_ENQUEUE will instruct the driver to
 * start the next pattern after the previous pattern finishes (i.e. runs
 * <repeats> times). If repeats=0, this will be ignored and default behavior
 * used instead. LED_CTL_INTERRUPT will stop the current step immediately
 * (for example, in a sigdetail event).
 * LED_CTL_AUDIODEV_WHITE is used for older products with an audiodev.ko
 * module where the white LED and green LED under the button can be on at
 * the same time.
 */
#define LED_CTL_DEFAULT			0x0000
#define LED_CTL_ENQUEUE			0x0001
#define LED_CTL_INTERRUPT		0x0002
#define LED_CTL_OVERRIDE_FEEDBACK	0x0004
#define LED_CTL_AUDIODEV_WHITE		0x0008

#define LED_CTL_MAX_STEPS		32

enum LED_COLOR {
    LED_RED_CLR         =   0xd0021b,
    LED_GREEN_CLR       =   0x55d000,
    LED_BLUE_CLR        =   0x0000ff,
    LED_AMBER_CLR       =   0xff5000,
    LED_YELLOW_CLR      =   0xffff55,
    LED_PURPLE_CLR      =   0xff55ff,
    LED_OFF             =   0x000000,
    LED_DIM_WHITE_CLR   =   0x6f6f6f,
    LED_WHITE_CLR       =   0xf3f3f3,
};

#endif /* LED_H */
