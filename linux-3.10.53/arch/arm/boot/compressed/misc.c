/*
 * misc.c
 * 
 * This is a collection of several routines from gzip-1.0.3 
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Modified for ARM Linux by Russell King
 *
 * Nicolas Pitre <nico@visuaide.com>  1999/04/14 :
 *  For this code to run directly from Flash, all constant variables must
 *  be marked with 'const' and all other variables initialized at run-time 
 *  only.  This way all non constant variables will end up in the bss segment,
 *  which should point to addresses in RAM and cleared to 0 on start.
 *  This allows for a much quicker boot time.
 */

unsigned int __machine_arch_type;

#include <linux/compiler.h>	/* for inline */
#include <linux/types.h>
#include <linux/linkage.h>
#include <linux/kernel.h>

static void putstr(const char *ptr);
extern void error(char *x);

#ifdef CONFIG_SONOS_SECBOOT
#include "sonos_digest_data.h"
#include "sonos_rollback.h"

/* External tool will modify this data in the compressed binary prior to the
 * addition of the u-boot header.  After kernel decompression, it must be
 * copied to KERNEL_ROOTFS_DIGEST for use by the kernel.  The decompressor
 * discards the .data section, so keep it in .text.
 */
rootfs_digest_t rootfs_digest __attribute__ ((section (".text"))) = {
	{ cpu_to_be32(SONOS_DIGEST_MAGIC1), cpu_to_be32(SONOS_DIGEST_MAGIC2) },	// digest_magic
	0x87654321,				// digest_value_length
	0x12345678,				// rootfs_length
	{"Placeholder for computed digest" },	// digest_value
} ;

/* Include a copy of the whitelist in the uncompressed kernel with a different
 * magic number.
 */
#define SONOS_FWW_IN_UNCOMPRESSED_KERNEL
#include "../init/firmware_whitelist.c"
#undef SONOS_FWW_IN_UNCOMPRESSED_KERNEL
#endif

#include CONFIG_UNCOMPRESS_INCLUDE

#ifdef CONFIG_DEBUG_ICEDCC

#if defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_V6K) || defined(CONFIG_CPU_V7)

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c0, c1, 0" : "=r" (status));
	} while (status & (1 << 29));

	asm("mcr p14, 0, %0, c0, c5, 0" : : "r" (ch));
}


#elif defined(CONFIG_CPU_XSCALE)

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c14, c0, 0" : "=r" (status));
	} while (status & (1 << 28));

	asm("mcr p14, 0, %0, c8, c0, 0" : : "r" (ch));
}

#else

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c0, c0, 0" : "=r" (status));
	} while (status & 2);

	asm("mcr p14, 0, %0, c1, c0, 0" : : "r" (ch));
}

#endif

#define putc(ch)	icedcc_putc(ch)
#endif

static void putstr(const char *ptr)
{
	char c;

	while ((c = *ptr++) != '\0') {
		if (c == '\n')
			putc('\r');
		putc(c);
	}

	flush();
}

/*
 * gzip declarations
 */
extern char input_data[];
extern char input_data_end[];

unsigned char *output_data;

unsigned long free_mem_ptr;
unsigned long free_mem_end_ptr;

#ifndef arch_error
#define arch_error(x)
#endif

void error(char *x)
{
	arch_error(x);

	putstr("\n\n");
	putstr(x);
	putstr("\n\n -- System halted");

	while(1);	/* Halt */
}

asmlinkage void __div0(void)
{
	error("Attempting division by 0!");
}

extern int do_decompress(u8 *input, int len, u8 *output, void (*error)(char *x));


void
decompress_kernel(unsigned long output_start, unsigned long free_mem_ptr_p,
		unsigned long free_mem_ptr_end_p,
		int arch_id)
{
	int ret;

	output_data		= (unsigned char *)output_start;
	free_mem_ptr		= free_mem_ptr_p;
	free_mem_end_ptr	= free_mem_ptr_end_p;
	__machine_arch_type	= arch_id;

	arch_decomp_setup();

	putstr("Uncompressing Linux...");
	ret = do_decompress(input_data, input_data_end - input_data,
			    output_data, error);
	if (ret)
		error("decompressor returned an error");
	else
		putstr(" done, booting the kernel.\n");

#ifdef CONFIG_SONOS_SECBOOT
	/* Kernel is decompressed and ready to go - copy the calculated
	 * rootfs digest into its memory.
	 */
	{
		int		*pkern;
		int		*plocal;
		int		i;

		pkern = (int*)KERNEL_ROOTFS;
#if defined(CONFIG_SONOS_ROOTFS_ADJUSTMENT)
		/* During the initial implementation, we got lucky - the System.map
		 * happened to locate the rootfs structure exactly where this code
		 * needed to adjust it.  On other platforms, however, the physical
		 * memory location happens not to match the virtual, and we need to
		 * adjust the value.  We'll do that with a CONFIG.
		 */
		pkern = (int*)((int)pkern - (int)CONFIG_SONOS_ROOTFS_ADJUSTMENT);
#endif
		plocal = (int*)&rootfs_digest;

		for(i=0;i<sizeof(rootfs_digest)/sizeof(int);i++)
			*pkern++ = *plocal++;
	}
#endif
}
