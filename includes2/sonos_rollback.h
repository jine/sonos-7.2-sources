/*
 * Copyright (c) 2015, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * common/include/sonos_rollback.h
 */

#ifndef SONOS_ROLLBACK_H
#define SONOS_ROLLBACK_H

/*
 * Use SRC_GPR8 [0x20d803c] as the boot_counter register for auto-fallback
 * in case of corrupted installs.
 */

#define SRC_GPR_COMPATIBLE	"fsl,imx6q-src"
#define SRC_GPR_BASE		0x20d8000
#define BOOT_COUNTER_OFFSET	0x3c
#define BOOT_COUNTER_REG	(SRC_GPR_BASE + BOOT_COUNTER_OFFSET)

#define BC_FLAG_BAD_SIGNATURE	0x01
#define BC_FLAG_STOP_BOOT	0x02

#define BOOT_DEFAULT		0
#define FALLBACK_BOOT_COUNTER	1
#define FALLBACK_COMMAND	2
#define FALLBACK_CRC		3

typedef struct {
	uint8_t boot_section;
	uint8_t fallback_flags;
	uint8_t boot_state;
	uint8_t	boot_counter;
} BootCounterReg_t;

#endif



