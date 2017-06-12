/*
 * Copyright (c) 2014, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * The following file is used for prototypes and data structures for
 * sonos APIs embedded in the Linux kernel.
 *
 * See kernel/sonos.c.
 */

#ifndef SONOS_KERNEL_H
#define SONOS_KERNEL_H
#if defined(CONFIG_SONOS)

#if defined(CONFIG_SONOS_SECBOOT)
#define UBIFS_CRYPT_TYPE_NONE   0
#define UBIFS_CRYPT_TYPE_FIXED  4
#define UBIFS_CRYPT_TYPE_BLACK_KEY      5
#define UBIFS_CRYPT_TYPE_RED_KEY        6

#define	USE_RED	0
#define USE_BLACK 1
#define MAX_SLOT_SIZE	2048
#define ENCRYPT	0
#define DECRYPT 1
#define SECMEM_KEYMOD_LEN 8

struct crypt_operation {
	int	cmd;
	int	color;
	void	*input_buffer;
	int	input_length;
	void	*output_buffer;
	size_t	*output_length;
	int	original_length;
	char	keymod[8];
} ;

int sonos_key_encdec(int operation, int color, const void *in, int inlen,
		                void *out, size_t *outlen, const char *keymod);

extern int sonos_set_ubifs_key(__u32);
extern void sonos_set_proc_crypt(int);
#endif

#if defined(SONOS_ARCH_ENCORE)

extern void *sonos_orientation_register_callback(void (*function)(int orient, void *param), void *param);
extern int sonos_orientation_unregister_callback(void *entry);
extern void sonos_orientation_change_event(int orient);

#endif // SONOS_ARCH_ENCORE

// Monitor voltage on power rails
// Voltage rail chan==1 24V, chan==0 5V
#define RAIL_24VOLT	1
#define RAIL_5VOLT	0
/***********************************************************************
 * Voltage rail monitoring
 *    These numbers have been derived by feeding a constant known
 *    voltage into the board and reading the value that the ADC
 *    returns. Since we can't use floating point math in the kernel
 *    we multiply everything by 1000 and divide it out afterwards to
 *    get the voltage in mV.
 **********************************************************************/
#if defined(CONFIG_SONOS_SOLBASE)
#define VF610_SCALE_5VOLT(v)		((v * 1342) / 1000)
#define VF610_SCALE_24VOLT(v)		((v * 7088) / 1000)
#elif defined(CONFIG_SONOS_ROYALE)
// 7.007 (7007/1000) is millivolts of the "24V" supply per count of the ADC.
// This is determined from three things:
// 1.  The ratio of the 24V supply to ADC divider, which is 0.115
// 2.  The reference voltage of the ADC, which defines the input voltage
// that gives the full scale count, which is nominally 3.3V (the supply
// voltage at the V3P3 buck converter is designed to be slightly higher,
// but resistive drop should bring it down to very close to exactly 3.3 at
// the supply pins of the i.MX6
// 3.  The number of full-scale ADC counts, which, in 12 bit mode, is 4095
// The formula is (3.3 / 0.115) / 4095 = 0.007007V = 7.007mV
#define VF610_SCALE_24VOLT(v)		((v * 7007) / 1000)
#elif defined(CONFIG_SONOS_ENCORE)
#define VF610_SCALE_5VOLT(v)		((v * 1342) / 1000)
#define VF610_SCALE_24VOLT(v)		((v * 8905) / 1000)
#elif defined(CONFIG_SONOS_PARAMOUNT)
#define VF610_SCALE_5VOLT(v)		((v * 1342) / 1000)
#define VF610_SCALE_24VOLT(v)		((v * 8905) / 1000)
#endif

// returns completion status, passes back milli-volts
// see drivers/iio/adc/vf610_adc.c
extern int vf610_read_adc(int chan, int *mvolts);

// disable/enable NAND access during shutdown
// see drivers/mtd/nand/nand_base.c
extern int nand_shutdown_access(int);

#if defined(CONFIG_IMX_SDMA)
/*
 * The following are special Sonos functions retrofitted into the
 * imx-sdma driver.
 */

#include <linux/dmaengine.h>
#include <linux/types.h>

extern dma_addr_t sdma_sonos_swap_data_pointer(struct dma_chan *chan,
		u32 index, dma_addr_t data_phys);

#endif // CONFIG_IMX_SDMA

#endif // CONFIG_SONOS
#endif // SONOS_KERNEL_H
