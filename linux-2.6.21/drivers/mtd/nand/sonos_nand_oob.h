/*
 ******************************************************************************
 *
 * Include File Name: sonos_nand_oob.h
 *
 * Include File Description:
 *
 * Currently only used on Woodstock, but should be generic enough to use on
 * other products using the same scheme (i.e. things beyond Woodstock that use
 * logical blocks and have 8 bytes of space available in the oob).
 *
 * I used inline functions because I was in that kind of mood.  Feel free to
 * rewrite this as defines if it bothers you...
 *
 * Everything in here is little endian, while I think that the ZP has one of
 * these backwards.  Ths can obviously be fixed by setting the offsets
 * correctly (I think bootgen1 would be before bootgen0).
 *
 ******************************************************************************
 */

#ifndef SONOS_NAND_OOB_H
#define SONOS_NAND_OOB_H

/*
 * Platform customization
 */
#if defined(CONFIG_MACH_WOODSTOCK)
#define SONOS_NAND_OOB_MAGIC0	32
#define SONOS_NAND_OOB_MAGIC1	33
#define SONOS_NAND_OOB_BOOTGEN0	34
#define SONOS_NAND_OOB_BOOTGEN1	35
#define SONOS_NAND_OOB_LBLOCK0	36
#define SONOS_NAND_OOB_LBLOCK1	37
#define SONOS_NAND_OOB_LPAGE0	38
#define SONOS_NAND_OOB_LPAGE1	39
#endif

#ifndef SONOS_NAND_OOB_MAGIC0
#error Please select a set of OOB offsets
#endif

/*
 * All pages get one of these, last page in partition with valid data gets
 * _LAST
 */
#define SONOS_NAND_OOB_MAGIC		0x1974
#define SONOS_NAND_OOB_MAGIC_LAST	0x7419

#define NAND_OOB_MAGIC_OK(oob) \
    (nand_oob_get_magic(oob) == SONOS_NAND_OOB_MAGIC || \
     nand_oob_get_magic(oob) == SONOS_NAND_OOB_MAGIC_LAST)

#define NAND_OOB_MAGIC_LAST(oob) \
     nand_oob_get_magic(oob) == SONOS_NAND_OOB_MAGIC_LAST)

/*
 * Magic number
 *
 * If this isn't set correctly, the rest of the fields are undefined.  It is
 * either SONOS_NAND_OOB_MAGIC or SONOS_NAND_OOB_MAGIC_LAST, with
 * SONOS_NAND_OOB_MAGIC_LAST being used for the last page 
 */
inline unsigned short nand_oob_get_magic(const unsigned char *oob) {
    return (oob[SONOS_NAND_OOB_MAGIC0] << 8 | oob[SONOS_NAND_OOB_MAGIC1]);
}

inline void nand_oob_set_magic(unsigned char *oob, unsigned short magic) {
    oob[SONOS_NAND_OOB_MAGIC0] = (magic >> 8);
    oob[SONOS_NAND_OOB_MAGIC1] = (magic);
}

/*
 * Boot generation
 *
 * Basically goes up by one every time an upgrade is performed, and is written
 * to every partition that contains data related to the upgrade (i.e. kernel
 * and rootfs at the moment).
 *
 * Yes, bad things will happen with it wraps.  Specifically, the upgrade will
 * be ignored.  Not a huge problem since it requires 64K upgrades, but I may
 * work around it in the bootloader anyway...
 */
inline void nand_oob_set_bootgen(unsigned char *oob, unsigned short bootgen) {
    oob[SONOS_NAND_OOB_BOOTGEN0] = (bootgen >> 8);
    oob[SONOS_NAND_OOB_BOOTGEN1] = (bootgen);
}

inline unsigned short nand_oob_get_bootgen(const unsigned char *oob) {
    return (oob[SONOS_NAND_OOB_BOOTGEN0] << 8 | oob[SONOS_NAND_OOB_BOOTGEN1]);
}

/*
 * Logical block number  [ Currently Woodstock only ]
 *
 * The partition table uses logical blocks, and use this
 * to locate the physical block that contains the data for a given logical
 * block.  It is only looked at in the first page of every block, but setting
 * it everywhere can't hurt.
 *
 * It is written in manufacturing and by the MTD driver.  You can attempt to
 * set it from userspace all you want, but the MTD driver will override it
 * based on the offset into the device.
 *
 */
inline void nand_oob_set_lblock(unsigned char *oob, unsigned short lblock) {    
    oob[SONOS_NAND_OOB_LBLOCK0] = (lblock >> 8);
    oob[SONOS_NAND_OOB_LBLOCK1] = lblock;
}

inline unsigned short nand_oob_get_lblock(const unsigned char *oob) {
    return (oob[SONOS_NAND_OOB_LBLOCK0] << 8 | oob[SONOS_NAND_OOB_LBLOCK1]);
}

/*
 * Logical page   [ Who uses this? ]
 *
 * The current upgrade app sets it, but the MTD driver for the ZP ignores it.
 * Not sure if it is used, or how big it is in the cases where is it used.
 *
 * Should work fine for Woodstock, but the Woodstock bootloader/MTD driver
 * ignores it entirely.
 *
 * Note: Only allows for 64K pages per sector, which is roughly 1/2 the flash
 *       for Woodstock.  It will obviously wrap if it is checked, so be aware
 *       of it...
 */
inline void nand_oob_set_lpage(unsigned char *oob, unsigned short lpage) {    
    oob[SONOS_NAND_OOB_LPAGE0] = (lpage >> 8);
    oob[SONOS_NAND_OOB_LPAGE1] = lpage;
}

inline unsigned short nand_oob_get_lpage(const unsigned char *oob) {
    return (oob[SONOS_NAND_OOB_LPAGE0] << 8 | oob[SONOS_NAND_OOB_LPAGE1]);
}


#endif /* SONOS_NAND_OOB_H */
