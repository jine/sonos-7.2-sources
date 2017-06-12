/* Board information for the ADS8272, which should be generic for
 * all 82xx boards.  The IMMR is now given to us so the hard define
 * will soon be removed.  All of the clock values are computed from
 * the configuration SCMR and the Power-On-Reset word.
 */

#ifdef __KERNEL__
#ifndef __MACH_WEMBLEY_DEFS
#define __MACH_WEMBLEY_DEFS

#include <linux/config.h>
#include <asm/u-boot.h>

#define IMAP_ADDR               ((uint)0xf0000000)

#endif
#endif /* __KERNEL__ */
