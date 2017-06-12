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

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

#ifdef CONFIG_SONOS_LIMELIGHT
/* Limelight flash partition sizes */
#define LIMELIGHT_FLASH_UBOOT_BLOCKS          6
#define FLASH_UBOOT_BLOCKS		      LIMELIGHT_FLASH_UBOOT_BLOCKS
#define LIMELIGHT_FLASH_RESERVED_BLOCKS      12
#define LIMELIGHT_FLASH_FPGA_BLOCKS           1
#define LIMELIGHT_FLASH_KERNEL_BLOCKS        32
#define LIMELIGHT_FLASH_ROOTFS_BLOCKS       200
#define LIMELIGHT_FLASH_JFFS_BLOCKS         520
#define LIMELIGHT_FLASH_REPLACEMENT_BLOCKS   20
#define FLASH_REPLACEMENT_BLOCKS              LIMELIGHT_FLASH_REPLACEMENT_BLOCKS

/* the reserved partition is sub divided like this */
#define LIMELIGHT_FLASH_PTABLE_BLOCKS         1
#define LIMELIGHT_FLASH_MDP_BLOCKS            1
#define LIMELIGHT_FLASH_DIAG_BLOCKS          10

/* Limelight flash default partition block offsets. Only UBoot is unmapped. 
 * All other partitions are mapped. The replacement partion really isn't 
 * available so they don't exist in the virtual block map. Those blocks can 
 * be mapped to later to replace a physical block that has gone bad.*/
#define LIMELIGHT_FLASH_UBOOT_OFFSET          0 /* physical */

#define LIMELIGHT_FLASH_RESERVED_OFFSET       0 /* logical */

#define LIMELIGHT_FLASH_KERNEL0_OFFSET        (LIMELIGHT_FLASH_RESERVED_OFFSET + LIMELIGHT_FLASH_RESERVED_BLOCKS)
#define LIMELIGHT_FLASH_ROOTFS0_OFFSET        (LIMELIGHT_FLASH_KERNEL0_OFFSET + LIMELIGHT_FLASH_KERNEL_BLOCKS)
#define LIMELIGHT_FLASH_FPGA0_OFFSET          (LIMELIGHT_FLASH_ROOTFS0_OFFSET + LIMELIGHT_FLASH_ROOTFS_BLOCKS)

#define LIMELIGHT_FLASH_JFFS_OFFSET           (LIMELIGHT_FLASH_FPGA0_OFFSET + LIMELIGHT_FLASH_FPGA_BLOCKS)

#define LIMELIGHT_FLASH_KERNEL1_OFFSET        (LIMELIGHT_FLASH_JFFS_OFFSET + LIMELIGHT_FLASH_JFFS_BLOCKS)
#define LIMELIGHT_FLASH_ROOTFS1_OFFSET        (LIMELIGHT_FLASH_KERNEL1_OFFSET + LIMELIGHT_FLASH_KERNEL_BLOCKS)
#define LIMELIGHT_FLASH_FPGA1_OFFSET          (LIMELIGHT_FLASH_ROOTFS1_OFFSET + LIMELIGHT_FLASH_ROOTFS_BLOCKS)

#define LIMELIGHT_FLASH_LAST_OFFSET           (LIMELIGHT_FLASH_FPGA1_OFFSET + LIMELIGHT_FLASH_FPGA_BLOCKS)
#define FIRST_LOGICAL_BLOCK  0
#define LAST_LOGICAL_BLOCK   (LIMELIGHT_FLASH_LAST_OFFSET-1)

#define NUM_MAPPABLE_PHYSICAL_BLOCKS \
	(LIMELIGHT_FLASH_RESERVED_BLOCKS + ((LIMELIGHT_FLASH_FPGA_BLOCKS + \
		LIMELIGHT_FLASH_KERNEL_BLOCKS + LIMELIGHT_FLASH_ROOTFS_BLOCKS)<<1) + \
		LIMELIGHT_FLASH_JFFS_BLOCKS + LIMELIGHT_FLASH_REPLACEMENT_BLOCKS)
#define NUM_VIRTUAL_BLOCKS (NUM_MAPPABLE_PHYSICAL_BLOCKS - LIMELIGHT_FLASH_REPLACEMENT_BLOCKS)


/*
 * Platform customization
 */
#define SONOS_NAND_OOB_MAGIC0	   40
#define SONOS_NAND_OOB_MAGIC1	   41
#define SONOS_NAND_OOB_BOOTGEN0    42
#define SONOS_NAND_OOB_BOOTGEN1    43
#define SONOS_NAND_OOB_LBLOCK0     44
#define SONOS_NAND_OOB_LBLOCK1     45
#define SONOS_NAND_OOB_LPAGE0	   46
#define SONOS_NAND_OOB_LPAGE1	   47
#endif	// CONFIG_SONOS_LIMELIGHT

#ifdef CONFIG_SONOS_FENWAY
/* Fenway flash partition sizes */

// Small block NAND
#define FENWAY_FLASH_UBOOT_BLOCKS         48
#define FLASH_UBOOT_BLOCKS		   FENWAY_FLASH_UBOOT_BLOCKS
#define FENWAY_FLASH_RESERVED_BLOCKS      11
#define FENWAY_FLASH_PTABLE_BLOCKS         1
#define FENWAY_FLASH_MDP_BLOCKS            4
#define FENWAY_FLASH_KERNEL_BLOCKS       192
#define FENWAY_FLASH_ROOTFS_BLOCKS       640
#define FENWAY_FLASH_JFFS_BLOCKS        2288
#define FENWAY_FLASH_REPLACEMENT_BLOCKS   80
#define FLASH_REPLACEMENT_BLOCKS           FENWAY_FLASH_REPLACEMENT_BLOCKS

#define NUM_MAPPABLE_PHYSICAL_BLOCKS   \
   (FENWAY_FLASH_RESERVED_BLOCKS +     \
    FENWAY_FLASH_PTABLE_BLOCKS   +     \
    FENWAY_FLASH_MDP_BLOCKS      +     \
    FENWAY_FLASH_KERNEL_BLOCKS   +     \
    FENWAY_FLASH_KERNEL_BLOCKS   +     \
    FENWAY_FLASH_ROOTFS_BLOCKS   +     \
    FENWAY_FLASH_ROOTFS_BLOCKS   +     \
    FENWAY_FLASH_JFFS_BLOCKS     +     \
    FENWAY_FLASH_REPLACEMENT_BLOCKS)

#define NUM_VIRTUAL_BLOCKS  (NUM_MAPPABLE_PHYSICAL_BLOCKS - FENWAY_FLASH_REPLACEMENT_BLOCKS)

// Large block NAND
#define FENWAY_FLASH_UBOOT_BLOCKS_LP	8
#define FLASH_UBOOT_BLOCKS_LP		FENWAY_FLASH_UBOOT_BLOCKS_LP
#define FENWAY_FLASH_RESERVED_BLOCKS_LP	12
#define FENWAY_FLASH_PTABLE_BLOCKS_LP	1
#define FENWAY_FLASH_MDP_BLOCKS_LP	4
#define FENWAY_FLASH_KERNEL_BLOCKS_LP	24
#define FENWAY_FLASH_ROOTFS_BLOCKS_LP	80
#define FENWAY_FLASH_JFFS_BLOCKS_LP	286
#define FENWAY_FLASH_REPLACEMENT_BLOCKS_LP	20
#define FLASH_REPLACEMENT_BLOCKS_LP	FENWAY_FLASH_REPLACEMENT_BLOCKS_LP

#define NUM_MAPPABLE_PHYSICAL_BLOCKS_LP   \
   (FENWAY_FLASH_RESERVED_BLOCKS_LP +     \
    FENWAY_FLASH_PTABLE_BLOCKS_LP   +     \
    FENWAY_FLASH_MDP_BLOCKS_LP      +     \
    FENWAY_FLASH_KERNEL_BLOCKS_LP   +     \
    FENWAY_FLASH_KERNEL_BLOCKS_LP   +     \
    FENWAY_FLASH_ROOTFS_BLOCKS_LP   +     \
    FENWAY_FLASH_ROOTFS_BLOCKS_LP   +     \
    FENWAY_FLASH_JFFS_BLOCKS_LP     +     \
    FENWAY_FLASH_REPLACEMENT_BLOCKS_LP)

#define NUM_VIRTUAL_BLOCKS_LP  (NUM_MAPPABLE_PHYSICAL_BLOCKS_LP - FENWAY_FLASH_REPLACEMENT_BLOCKS_LP)


#define FIRST_LOGICAL_BLOCK  0

/*
 * Platform customization
 */

/* Small page NAND */
#define SONOS_NAND_OOB_MAGIC0	   10
#define SONOS_NAND_OOB_MAGIC1	   11
#define SONOS_NAND_OOB_LPAGE0	    0
#define SONOS_NAND_OOB_LPAGE1	    1
/* Large page NAND */
#define SONOS_NAND_OOB_MAGIC0_LP	6
#define SONOS_NAND_OOB_MAGIC1_LP	7
#define SONOS_NAND_OOB_LPAGE0_LP	1
#define SONOS_NAND_OOB_LPAGE1_LP	2
/* Common */
#define SONOS_NAND_OOB_BOOTGEN0    12
#define SONOS_NAND_OOB_BOOTGEN1    13
#define SONOS_NAND_OOB_LBLOCK0     14
#define SONOS_NAND_OOB_LBLOCK1     15

#endif	// CONFIG_SONOS_FENWAY

/* First physical block that can be mapped */
#define FIRST_MAPPABLE_PHYSICAL_BLOCK (FLASH_UBOOT_BLOCKS)
#define FIRST_MAPPABLE_PHYSICAL_BLOCK_LP (FLASH_UBOOT_BLOCKS_LP)

#define ERR		      0x100
#define EBADBLOCK	   (ERR + 1)
#define ECMDFAIL	   (ERR + 2)
#define ENOSPACE	   (ERR + 3)
#define ENOBLOCKi	   (ERR + 4)
#define ESIZE		   (ERR + 5)
#define ENOTABLE	   (ERR + 6)
#define ENOPARTITION	   (ERR + 6)
#define EBADBOOTGEN	   (ERR + 7)
#define ENOLAST		   (ERR + 8)
#define EBADMAGIC	   (ERR + 10)

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
     (nand_oob_get_magic(oob) == SONOS_NAND_OOB_MAGIC_LAST)


#ifdef CONFIG_SONOS_FENWAY
#define NAND_NUM_BLOCKS(info)  (info->size >> info->erasesize)
#endif
#if defined(CONFIG_SONOS_LIMELIGHT)
#define NAND_NUM_BLOCKS(info)  (info->size / (info->erasesize ? info->erasesize : (128 * 1024)))
#endif

#ifdef NAND_OOB_WITH_ECC
void nand_oob_set_ecc(unsigned char *oob, uint16_t virtualBlockNum);
int nand_oob_get_ecc(unsigned char *oob);
#endif	// NAND_OOB_WITH_ECC

#if defined(CONFIG_SONOS_LIMELIGHT)
unsigned short nand_oob_get_magic(const unsigned char *oob);
void nand_oob_set_magic(unsigned char *oob, unsigned short magic);
void nand_oob_set_bootgen(unsigned char *oob, unsigned short bootgen);
unsigned short nand_oob_get_bootgen(const unsigned char *oob);
void nand_oob_set_lblock(unsigned char *oob, unsigned short lblock);
unsigned short nand_oob_get_lblock(const unsigned char *oob);
void nand_oob_set_lpage(unsigned char *oob, unsigned short lpage);
unsigned short nand_oob_get_lpage(const unsigned char *oob);
int mark_badblock_in_ram(int block, struct mtd_info *info);
int mark_badblock(int block, struct mtd_info *info);
int is_badblock(int block, struct mtd_info *info);
int is_usedblock(int block, struct mtd_info *info);
int mark_usedblock(int block, struct mtd_info *info);
void set_lblock(struct mtd_info *info, int block, int logicalBlock);
void nand_print_map (struct mtd_info *mtd, int lblock);
unsigned short search_rel_block(int lblock, struct mtd_info *info);
int nand_erase_block(struct mtd_info *mtd, unsigned short physBlock);
unsigned short nand_get_physical_block(struct mtd_info *mtd, unsigned short logicalBlock);
#endif	// CONFIG_SONOS_LIMELIGHT

#endif /* SONOS_NAND_OOB_H */
