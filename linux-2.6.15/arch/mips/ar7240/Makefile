#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright © 2003 Atheros Communications, Inc.,  All Rights Reserved.
#

# Makefile for Atheros ar531x boards
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

#.S.s:
#	$(CPP) $(CFLAGS) $< -o $*.s
#.S.o:
#	$(CC) $(CFLAGS) -D__ASSEMBLY__ -c $< -o $*.o

ifdef CONFIG_SONOS
obj-y	+=	prom.o	\
			gpio.o	\
			intr.o	\
			irq.o	\
			setup.o	\
			ar7240.o \
			platform.o \
			wdt.o \
			button_event.o \
			button_sim.o \
			sonos-buttons.o \
			sonos-leds.o
else
obj-y	+=	prom.o	\
			gpio.o	\
			intr.o	\
			irq.o	\
			setup.o	\
			ar7240.o \
			platform.o \
			wdt.o
endif

ifdef CONFIG_CUS100
CFLAGS_gpio.o += -DCONFIG_CUS100=1
endif

ifeq ($(BOARD_TYPE), ap91fus)
CFLAGS_gpio.o += -DJUMPSTART_GPIO=12
else
CFLAGS_gpio.o += -DJUMPSTART_GPIO=12
endif

ifndef WPS_LED_GPIO_PIN
WPS_LED_GPIO_PIN=5
endif

CFLAGS_gpio.o += -DWPS_LED_GPIO=$(WPS_LED_GPIO_PIN)

#EXTRA_CFLAGS				+=	-fno-schedule-insns -fno-schedule-insns2
