#include <linux/init.h>
#include <asm/page.h>

	.section ".data.page_aligned"

	.globl sonos_dtb_start, sonos_dtb_end
	.balign PAGE_SIZE
sonos_dtb_start:
	.incbin "arch/powerpc/boot/fenway.dtb"
	.balign PAGE_SIZE
sonos_dtb_end:

	.previous
