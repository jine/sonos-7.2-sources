/* Board information for the ADS8272, which should be generic for
 * all 82xx boards.  The IMMR is now given to us so the hard define
 * will soon be removed.  All of the clock values are computed from
 * the configuration SCMR and the Power-On-Reset word.
 */

#ifdef __KERNEL__
#ifndef __MACH_ADS8272_DEFS
#define __MACH_ADS8272_DEFS

#include <linux/config.h>
#include <asm/u-boot.h>

#define IMAP_ADDR               ((uint)0xf0000000)
#define BCSR_ADDR               ((uint)0xf8000000)
#define BCSR_SIZE               ((uint)(32 * 1024))
#define BCSR0_LED0              ((uint)0x02000000)
#define BCSR0_LED1              ((uint)0x01000000)
#define BCSR1_FETHIEN           ((uint)0x08000000)
#define BCSR1_FETH_RST          ((uint)0x04000000)
#define BCSR1_RS232_EN1         ((uint)0x02000000)
#define BCSR1_RS232_EN2         ((uint)0x01000000)

#endif
#endif /* __KERNEL__ */
