#define VNB_DEBUG_FORCE_ERRORS
#include "virtual_block.h"
#include "sonos_nand_oob.h"
#include <linux/proc_fs.h>
#include "mdp.h"
#include <linux/mtd/nand.h>
#ifdef NAND_OOB_WITH_ECC
#include <linux/mtd/nand_ecc.h>
#define NAND_OOB_ECC_TEST
#ifdef NAND_OOB_ECC_TEST
int nand_oob_ecc_test_flag;
int nand_oob_ecc_badeccbk;
static struct proc_dir_entry *proc_oobtest;
static struct proc_dir_entry *proc_badoobbk;
#endif // NAND_OOB_ECC_TEST
#endif	// NAND_OOB_WITH_ECC

/* Sonos Virtual NAND Block (VNB) support */

#define MDP_OFFSET 0x4000
#define MDP_OFFSET_LP 0x20000

extern int bootgeneration;
extern struct manufacturing_data_page sys_mdp;

static int firstBad = 1;
static int firstMap = 1;
/*
 * These used to be #defines, but they need to work with both
 * large and small page NAND, and are different, so we need to change
 * them to variables.
 */
static int NumVirtualBlocks;
static int SonosNandOobMagic0;
static int SonosNandOobMagic1;
static int SonosNandOobLpage0;
static int SonosNandOobLpage1;
static int FlashUbootBlocks;
static int FlashReplacementBlocks;
static int SonosOobPhysStart;
static int LastBlockMask;
static int NandBlockMask;
static int MdpOffset;
static int FirstMappablePhysicalBlock;

/*
 * Need to know, with Fenway, if it's a large or small page nand device.
 */
static bool FenwayLP = false;

#if defined(CONFIG_SONOS_LIMELIGHT)
extern unsigned int powerSaves;
#endif	// CONFIG_SONOS_LIMELIGHT

VIRTUAL_NAND_BLOCK_T *GlobalVnbDataPtr=NULL; /* used only for debug */

/* Small Page FLASH with FMR[ECCM] = 0 */
#if defined(CONFIG_SONOS_LIMELIGHT)
#define SONOS_OOBAVAIL 16
#define SONOS_OOB_PHYS_SKIP   14
#define SONOS_OOB_PHYS_START  (SONOS_OOB_PHYS_SKIP)

#define NAND_BLOCK_MASK  0xFFFC0000
#define NAND_OOB_SIZE    64
#ifdef DEBUG_JFFS
#define NAND_BLOCK_SHIFT 17
#define MAX_BLOCKS       2048
#define JFFS_OFFSET      (245 << NAND_BLOCK_SHIFT) // AJ: Please, stop the madness
#endif	// DEBUG_JFFS

static struct nand_ecclayout sonos_ecclayout = {
    .eccbytes = 32,
    .eccpos = {
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39
    },
    .oobavail = SONOS_OOBAVAIL,
    .oobfree = { {48, 16} },
};
#define MAX_PHYS_OOB_SIZE  (4 * 64)
#endif	// CONFIG_SONOS_LIMELIGHT

#ifdef CONFIG_SONOS_FENWAY
#define SONOS_OOBAVAIL 5
#define SONOS_OOB_PHYS_SKIP  7
#define SONOS_OOB_PHYS_START 0
#define SONOS_OOB_PHYS_START_LP 1

#define NAND_BLOCK_MASK  0xFFFFC000
#define NAND_BLOCK_MASK_LP  0xFFFE0000
#define NAND_OOB_SIZE    16
#ifdef DEBUG_JFFS
#define NAND_BLOCK_SHIFT 14
#define MAX_BLOCKS       4096
#define JFFS_OFFSET      0xd40000
#endif	// DEBUG_JFFS
static struct nand_ecclayout sonos_ecclayout_16 = {
	.eccbytes = 3,
	.eccpos = {6, 7, 8},
	.oobavail = SONOS_OOBAVAIL,
	.oobfree = { {0, 5}, {9, 1} },
};
static struct nand_ecclayout sonos_ecclayout_64 = {
	.eccbytes = 12,
	.eccpos = {8, 9, 10, 24, 25, 26, 40, 41, 42, 56, 57, 58},
	.oobavail = SONOS_OOBAVAIL,
	.oobfree = { {1, 7}, {11, 13}, {27, 13}, {43, 13}, {59, 5} }
};
#define MAX_PHYS_OOB_SIZE  (4 * 16)
#endif	// CONFIG_SONOS_FENWAY

int scancount;

static int vnb_phys_erase(struct mtd_info *pVirtualMtd, loff_t physicalOffset);

/*
 * Magic number
 *
 * If this isn't set correctly, the rest of the fields are undefined.  It is
 * either SONOS_NAND_OOB_MAGIC or SONOS_NAND_OOB_MAGIC_LAST, with
 * SONOS_NAND_OOB_MAGIC_LAST being used for the last page
 */
unsigned short nand_oob_get_magic(const unsigned char *oob) {
    return (oob[SonosNandOobMagic0] << 8 | oob[SonosNandOobMagic1]);
}

void nand_oob_set_magic(unsigned char *oob, unsigned short magic) {
    oob[SonosNandOobMagic0] = (magic >> 8);
    oob[SonosNandOobMagic1] = (magic);
}

/*
 * check that write page is same as "last block" page
 */
int nand_oob_is_last_block(loff_t virtualOffset, loff_t lbo) {
    return (
#ifdef CONFIG_SONOS_FENWAY
	/* last write block in erase block */
	((virtualOffset & LastBlockMask)==LastBlockMask) &&
#endif
        (lbo & NandBlockMask) == (virtualOffset & NandBlockMask));
}

/*
 * Boot generation
 *
 * Basically goes up by one every time an upgrade is performed, and is written
 * to every partition that contains data related to the upgrade (i.e. kernel
 * and rootfs at the moment).
 *
 * Yes, bad things will happen when it wraps.  Specifically, the upgrade will
 * be ignored.  Not a huge problem since it requires 64K upgrades, but I may
 * work around it in the bootloader anyway...
 */
void nand_oob_set_bootgen(unsigned char *oob, unsigned short bootgen) {
    oob[SONOS_NAND_OOB_BOOTGEN0] = (bootgen >> 8);
    oob[SONOS_NAND_OOB_BOOTGEN1] = (bootgen);
}

unsigned short nand_oob_get_bootgen(const unsigned char *oob) {
    return (oob[SONOS_NAND_OOB_BOOTGEN0] << 8 | oob[SONOS_NAND_OOB_BOOTGEN1]);
}
#ifdef NAND_OOB_WITH_ECC
/*
 * Protect OOB with ECC
 * Add two reserve bytes for future use
 * For ECC during write, set reserve bytes to 0xff, calculate ECC from magic
 * byte 0 to reserve bytes 1 and write 3 bytes ECC to OOB
 * For ECC during read, check ECC bytes of OOB, skip ECC calculation if all 3
 * bytes are 0xff ( for backward compatible), calculate ECC from magic byte 0
 * to reserve bytes 1, compare the calculated ECC bytes to the one on OOB
 * correct data if ECC bytes are different. if __nand_correct_data returns 1,
 * which means one bit is corrected, copy the data in the temp buffer to OOB
 * otherwise, just leave the original data in the OBB
 */
void nand_oob_set_ecc(unsigned char *oob, uint16_t virtualBlockNum)
{
    char buffer[256], ecc[3];
    oob[SONOS_NAND_OOB_RESERVE0] = 0xFF;
    oob[SONOS_NAND_OOB_RESERVE1] = 0xFF;
    memset(buffer, 0, 256);
    memcpy(buffer, &oob[SonosNandOobMagic0], SONOS_NAND_OOB_RESERVE1 -
       SonosNandOobMagic0 + 1);
    __nand_calculate_ecc(buffer, 256, ecc);
    memcpy(&oob[SONOS_NAND_OOB_ECC], ecc, 3);
#ifdef NAND_OOB_ECC_TEST
    if ( nand_oob_ecc_test_flag ) {
        char *flip_byte;
        int bk_to_flip = virtualBlockNum - nand_oob_ecc_badeccbk;
        int two_bits_flag = nand_oob_ecc_test_flag/100;
        int real_test_flag = nand_oob_ecc_test_flag - two_bits_flag * 100;
        int byte = real_test_flag / 8 + 1;
        int bit = real_test_flag % 8 - 1;
	int flip_flag = bk_to_flip == 0xc || bk_to_flip == 0x2c ||
            bk_to_flip == 0xf4 || bk_to_flip == 0x2fd || bk_to_flip == 0x31d ||
            bk_to_flip == 0x3e5;
        if ( !flip_flag ) return;

        if ( bit < 0 ) {
            byte--;
            bit = 7;
        }
	if ( byte > 8 )
            flip_byte = &oob[SONOS_NAND_OOB_ECC + byte - 9 ];
        else
            flip_byte = &oob[SonosNandOobMagic0 + byte - 1 ];
        if ( *flip_byte & ( 1 << bit ) )
            *flip_byte &= ~( 1 << bit );
        else
            *flip_byte |= ( 1 << bit );

        if ( two_bits_flag ) {
            if ( bit < 7 )
                bit++;
            else
                bit = 0;
            if ( *flip_byte & ( 1 << bit ) )
                *flip_byte &= ~( 1 << bit );
            else
                *flip_byte |= ( 1 << bit );
        }
    }
#endif
}

int nand_oob_get_ecc(unsigned char *oob)
{
    int ret;
    char buffer[256], ecc[3], read_ecc[3];

    if ( oob[SONOS_NAND_OOB_ECC] == 0xFF && oob[SONOS_NAND_OOB_ECC + 1] == 0xFF
      && oob[SONOS_NAND_OOB_ECC + 2] == 0xFF )
      return 0;
    memset(buffer, 0, 256);
    memcpy(buffer, &oob[SonosNandOobMagic0], SONOS_NAND_OOB_RESERVE1 -
       SonosNandOobMagic0 + 1);
    __nand_calculate_ecc(buffer, 256, ecc);
    memcpy(read_ecc, &oob[SONOS_NAND_OOB_ECC], 3);

    if ( memcmp(read_ecc, ecc, 3 ) == 0 )
      return 0;
    ret = __nand_correct_data(buffer, read_ecc, ecc, 256);
    if ( ret == 1 ) {
      memcpy(&oob[SonosNandOobMagic0], buffer, SONOS_NAND_OOB_RESERVE1 -
       SonosNandOobMagic0 + 1);
#ifdef NAND_OOB_ECC_TEST
      printk("One bit error corrected\n");
#endif
      return 0;
    }
#ifdef NAND_OOB_ECC_TEST
    printk("Can't corrected error in OOB\n");
#endif
    return 1;
}
#endif // NAND_OOB_WITH_ECC

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
void nand_oob_set_lblock(unsigned char *oob, unsigned short lblock) {
    oob[SONOS_NAND_OOB_LBLOCK0] = (lblock >> 8);
    oob[SONOS_NAND_OOB_LBLOCK1] = lblock;
}

unsigned short nand_oob_get_lblock(const unsigned char *oob) {
    return (oob[SONOS_NAND_OOB_LBLOCK0] << 8 | oob[SONOS_NAND_OOB_LBLOCK1]);
}

/* Is the physical block available? */
static inline int vnb_is_usedblock(int physBlock, VIRTUAL_NAND_BLOCK_T *pVirtualNandData)
{
    int byte = (physBlock / 8);
    u8  bit  = (1 << (physBlock % 8));

    return (pVirtualNandData->used_blocks[byte] & bit) ? 1 : 0;
}

/* make the physical block unavailable */
static inline void vnb_mark_usedblock(int physBlock, VIRTUAL_NAND_BLOCK_T *pVirtualNandData)
{
    int byte = (physBlock / 8);
    u8  bit  = (1 << (physBlock % 8));

    pVirtualNandData->used_blocks[byte] |= bit;
    pVirtualNandData->blocksUsed++;
}

/* make the physical block available */
static inline void vnb_mark_unusedblock(int physBlock, VIRTUAL_NAND_BLOCK_T *pVirtualNandData)
{
    int byte = (physBlock / 8);
    u8  bit  = (1 << (physBlock % 8));

    pVirtualNandData->used_blocks[byte] &= ~bit;
    pVirtualNandData->blocksUsed--;
}

static void vnb_set_rel_block(VIRTUAL_NAND_BLOCK_T *pVirtualNandData,
                              int virtualBlock, int physicalBlock)
{
    struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
    uint32_t numBlocks;

    numBlocks = mtd_div_by_eb(pPhysicalMtd->size, pPhysicalMtd);
    if ((physicalBlock < 0) || (physicalBlock >= numBlocks)) {
	printk(KERN_ERR "%s: physical block %X exceeds max %X\n",
	       __func__, physicalBlock, numBlocks);
    }

    if (virtualBlock < 0 || virtualBlock >= NumVirtualBlocks) {
	printk(KERN_ERR "%s: virtual block %X exceeds max %X\n",
	       __func__, virtualBlock, NumVirtualBlocks);
	return;
    }

   //printk("%s: virtual block %X mapped to physical block %X\n",
   //       __func__, virtualBlock, physicalBlock);

    vnb_mark_usedblock(physicalBlock, pVirtualNandData);
    pVirtualNandData->block_map[virtualBlock] = physicalBlock;
}

static void vnb_remove_mapping(VIRTUAL_NAND_BLOCK_T *pVirtualNandData, loff_t virtualOffset)
{
    struct mtd_info *pVirtualMtd = (struct mtd_info *)pVirtualNandData;
    uint16_t virtualBlockNum;

    virtualBlockNum = mtd_div_by_eb(virtualOffset, pVirtualMtd);
    if (virtualBlockNum <= pVirtualNandData->maxVirtualBlockNum)
	pVirtualNandData->block_map[virtualBlockNum] = 0;
}


/* Translate a virtual flash offset to a physical flash offset */
static int vnb_translateOffset(struct mtd_info *pVirtualMtd,
                               loff_t virtualOffset,  loff_t *pPhysicalOffset)
{
    VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
    uint16_t physicalBlockNum;
    uint16_t virtualBlockNum;
    uint32_t blockOffset;
    int error = 0;

    /* get the block number and the offset within the block */
    virtualBlockNum = mtd_div_by_eb(virtualOffset, pVirtualMtd);
    blockOffset = mtd_mod_by_eb(virtualOffset, pVirtualMtd);

    physicalBlockNum = pVirtualNandData->block_map[virtualBlockNum];
    if (physicalBlockNum == 0) {
	physicalBlockNum = 0xffff;
	error = -1;
    }

    *pPhysicalOffset = (physicalBlockNum * pVirtualMtd->erasesize) + blockOffset;

    return error;
}

/* Translate a virtual flash offset to a physical flash offset, allocate a
   physical block if virtual not already mapped */
static int vnb_translateOffset_alloc(struct mtd_info *pVirtualMtd,
                                     loff_t virtualOffset, loff_t *pPhysicalOffset)
{
    VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
    struct mtd_info *pPhysicalMtd;
    uint16_t physBlock, nextPhysBlock;
    uint16_t virtualBlockNum;
    uint32_t blockOffset;
    loff_t   physicalOffset;
    int error = 0;

    error = vnb_translateOffset(pVirtualMtd, virtualOffset, pPhysicalOffset);
    if (!error) {
	/* block is already mapped so we're done */
	return 0;
    }

    /* block is not mapped so let's go get one */
    pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
    blockOffset = mtd_mod_by_eb(virtualOffset, pVirtualMtd);
    virtualBlockNum = mtd_div_by_eb(virtualOffset, pVirtualMtd);

    /* pick up where we left off */
    physBlock = pVirtualNandData->nextPhysBlock;
    do {
	physicalOffset = physBlock * pVirtualMtd->erasesize;
	nextPhysBlock = physBlock + 1;
	if (nextPhysBlock > pVirtualNandData->lastPhysBlock)
	    nextPhysBlock = FirstMappablePhysicalBlock;

	if (!vnb_is_usedblock(physBlock, pVirtualNandData) &&
	    !pPhysicalMtd->block_isbad(pVirtualNandData->pPhysicalMtd, physicalOffset)) {
	    /* erase the physical block */
	    error = vnb_phys_erase(pVirtualMtd, physicalOffset);
	    if (!error) {
	        break;
	    }
	}
	physBlock = nextPhysBlock;

    } while (nextPhysBlock != pVirtualNandData->nextPhysBlock);

    if (nextPhysBlock != pVirtualNandData->nextPhysBlock) {
        /* physical block is good and available so grab it, we assume the block
         * will be written soon and that the OOB data is written then. */
	vnb_set_rel_block(pVirtualNandData, virtualBlockNum, physBlock);
	*pPhysicalOffset = physicalOffset + blockOffset;
	pVirtualNandData->nextPhysBlock = nextPhysBlock;
    } else {
	/* out of blocks */
	/* return something bad but not zero since that could result in trashing U-Boot */
	*pPhysicalOffset = 0xffffffff;
	error = -EIO;
    }

    return error;
}

static int vnb_read(struct mtd_info *pVirtualMtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd;
   uint32_t bytesRead, readLen;
   size_t vnbretlen;
   int error;
   loff_t virtualOffset = from;
   loff_t physicalOffset;
   uint32_t startOffset;
   int32_t extra;

   //printk("--vnb_read: from %llx, len %x\n", from, len);

   if (pVirtualMtd == NULL) {
      panic("vnb_read passed NULL virtual MTD\n");
   }
   pPhysicalMtd = pVirtualNandData->pPhysicalMtd;

   startOffset = mtd_mod_by_eb(from, pVirtualMtd);
#if 0
   if ((startOffset & (pPhysicalMtd->writesize - 1)) != 0) {
      printk("vnb_read: unaligned offset %llx, abort\n", from);
      *retlen = 0;
      return -1;
   }
#endif

   bytesRead = 0;
   while (bytesRead < len) {
      /* read one block at a time so the virtual to physical translation can be made */
      readLen = len - bytesRead;
      if (readLen > pVirtualMtd->erasesize) {
         readLen = pVirtualMtd->erasesize;
      }

      extra = readLen + startOffset - pVirtualMtd->erasesize;
      if (extra > 0) {
         /* don't cross block boundary on a single read */
         readLen -= extra;
      }

      error = vnb_translateOffset_alloc(pVirtualMtd, virtualOffset, &physicalOffset);
      if (error) {
         printk("vnb_read: virtual offset %llX cannot be mapped\n", virtualOffset);
         *retlen = 0;
         return error;
      }
      else {
         /* virtual block is mapped, so read flash */
         error = pPhysicalMtd->read(pPhysicalMtd, physicalOffset, readLen,
                                    &vnbretlen, &buf[bytesRead]);
         if (error) {
            printk("vnb_read: error %d at offset %llx\n", error, virtualOffset);
            *retlen = 0;
            return error;
         }
         if (vnbretlen != readLen) {
            printk("vnb_read: read %d bytes, requested %d\n", vnbretlen, readLen);
	    return -EIO;
         }
      }

      virtualOffset += readLen;
      bytesRead += readLen;
      startOffset = 0;
   }

   pVirtualNandData->pagesRead += (bytesRead + pVirtualMtd->writesize - 1) / pVirtualMtd->writesize;
   *retlen = bytesRead;
   return 0;
}

static int vnb_read_special(struct mtd_info *pVirtualMtd, struct mtd_special_info *rsi, void *buf)
{
    VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *) pVirtualMtd;
    struct mtd_info *pPhysicalMtd = (struct mtd_info *) pVirtualNandData->pPhysicalMtd;

    return pPhysicalMtd->read_special(pPhysicalMtd, rsi, buf);
}

struct mtd_info * vnb_init(struct mtd_info *pPhysicalMtd)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData;
   uint32_t numBlocks;
   uint32_t blockMapSize;

   scancount = 0;

//   printk("Virtual Block Init: enter\n");

   if ((pPhysicalMtd->size == 0) || (pPhysicalMtd->erasesize == 0)) {
      printk("MTD size not set yet, size %lld, erasesize %d\n",
             pPhysicalMtd->size, pPhysicalMtd->erasesize);
      return NULL;
   }

#ifdef CONFIG_SONOS_FENWAY
   if (pPhysicalMtd->erasesize == 131072) {
	   // This is a device with a large page NAND
	   printk(KERN_INFO "FenwayLP device\n");
	   FenwayLP = true;
   }

   if (FenwayLP) {
	   // Set up the remaining "constants"
	   NumVirtualBlocks = NUM_VIRTUAL_BLOCKS_LP;
	   SonosNandOobMagic0 = SONOS_NAND_OOB_MAGIC0_LP;
	   SonosNandOobMagic1 = SONOS_NAND_OOB_MAGIC1_LP;
	   SonosNandOobLpage0 = SONOS_NAND_OOB_LPAGE0_LP;
	   SonosNandOobLpage1 = SONOS_NAND_OOB_LPAGE1_LP;
	   FlashUbootBlocks = FLASH_UBOOT_BLOCKS_LP;
	   FlashReplacementBlocks = FLASH_REPLACEMENT_BLOCKS_LP;
	   SonosOobPhysStart = SONOS_OOB_PHYS_START_LP;
	   LastBlockMask = 0x1f800;
	   NandBlockMask = NAND_BLOCK_MASK_LP;
	   MdpOffset = MDP_OFFSET_LP;
   }
   else
#endif
   {
	   NumVirtualBlocks = NUM_VIRTUAL_BLOCKS;
	   SonosNandOobMagic0 = SONOS_NAND_OOB_MAGIC0;
	   SonosNandOobMagic1 = SONOS_NAND_OOB_MAGIC1;
	   SonosNandOobLpage0 = SONOS_NAND_OOB_LPAGE0;
	   SonosNandOobLpage1 = SONOS_NAND_OOB_LPAGE1;
	   FlashUbootBlocks = FLASH_UBOOT_BLOCKS;
	   FlashReplacementBlocks = FLASH_REPLACEMENT_BLOCKS;
	   SonosOobPhysStart = SONOS_OOB_PHYS_START;
	   LastBlockMask = 0x3E00;
	   NandBlockMask = NAND_BLOCK_MASK;
	   MdpOffset = MDP_OFFSET;
   }


   pVirtualNandData = kzalloc(sizeof(*pVirtualNandData), GFP_KERNEL);
   if (pVirtualNandData == NULL) {
      printk("unable to allocate virtual NAND data\n");
      return NULL;
   }

   numBlocks = mtd_div_by_eb(pPhysicalMtd->size, pPhysicalMtd);
   blockMapSize = sizeof(*pVirtualNandData->block_map) * numBlocks;
//   printk("allocating %d for virtual block map\n", blockMapSize);
   pVirtualNandData->block_map = kzalloc(blockMapSize, GFP_KERNEL);
   if (pVirtualNandData->block_map == NULL) {
      printk("unable to allocate block map\n");
      goto error_out;
   }
   pVirtualNandData->maxVirtualBlockNum =
      numBlocks - FlashUbootBlocks - FLASH_REPLACEMENT_BLOCKS;

   blockMapSize = sizeof(*pVirtualNandData->used_blocks) * numBlocks;
   pVirtualNandData->used_blocks = kzalloc(blockMapSize, GFP_KERNEL);
   if (pVirtualNandData->used_blocks == NULL) {
      printk("unable to allocate used_blocks map\n");
      goto error_out;
   }

   /* provide access to the physical MTD */
   pVirtualNandData->pPhysicalMtd = pPhysicalMtd;
   pPhysicalMtd->pVirtualNand = pVirtualNandData;
   GlobalVnbDataPtr = pVirtualNandData; /* save for debug */

//   printk("Virtual Block Init: virtual MTD %p\n", &pVirtualNandData->virtualMtd);
   return &pVirtualNandData->virtualMtd;

error_out:
   if (pVirtualNandData->block_map)
      kfree(pVirtualNandData->block_map);
   if (pVirtualNandData)
      kfree(pVirtualNandData);
   return NULL;
}


static int vnb_point(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, void **virt, resource_size_t *phys)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	if (from >= mtd->size)
		len = 0;
	else if (from + len > mtd->size)
		len = mtd->size - from;
	return part->master->point (part->master, from + part->offset,
				    len, retlen, virt, phys);
#else
   printk("%s not supported yet\n", __func__);
   return 0;
#endif
}

static void vnb_unpoint(struct mtd_info *mtd, loff_t from, size_t len)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);

	part->master->unpoint(part->master, from + part->offset, len);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
#endif
}

static int vnb_set_last_block(struct mtd_info *pVirtualMtd, loff_t ofs)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   printk("%s: last block offset set to %llx\n", __func__, ofs);
   pVirtualNandData->lastBlockOffset = ofs;
   return 0;
}

static int vnb_write_oob(struct mtd_info *pVirtualMtd, loff_t to, struct mtd_oob_ops *ops);

static int vnb_write(struct mtd_info *pVirtualMtd, loff_t to, size_t len,
                     size_t *retlen, const u_char *buf)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd;
   uint32_t bytesWritten, writeLen;
   int error;
   loff_t virtualOffset = to;
   uint32_t startOffset;

   struct mtd_oob_ops o;

   //printk("vnb_write: to %llx, len %x\n", to, len);

   if ((pVirtualMtd == NULL) || (pVirtualNandData->pPhysicalMtd == NULL)) {
       panic("vnb_write passed NULL virtual MTD %x %x\n", (unsigned int)pVirtualMtd,
	     (unsigned int)pVirtualNandData->pPhysicalMtd);
   }
   pPhysicalMtd = pVirtualNandData->pPhysicalMtd;

   startOffset = mtd_mod_by_eb(virtualOffset, pVirtualMtd);
   if ((startOffset & (pPhysicalMtd->writesize - 1)) != 0) {
      printk("vnb_write: unaligned offset %llx, abort\n", virtualOffset);
      *retlen = 0;
      return -EINVAL;
   }

   bytesWritten = 0;
   while (bytesWritten < len) {
      writeLen = len - bytesWritten;
      if (writeLen > pVirtualMtd->writesize) writeLen = pVirtualMtd->writesize;
      o.len=writeLen;
      o.datbuf=((u_char*)buf)+bytesWritten;
      o.oobbuf=0;
      o.ooblen=0;
      o.mode=MTD_OOB_AUTO;
      o.ooboffs=0;
      error=vnb_write_oob(pVirtualMtd,virtualOffset,&o);
      if (error) {
         printk("vnb_write calling vnb_write_oob error %d\n",error);
         return error;
      }
      bytesWritten += writeLen;
      virtualOffset += writeLen;
   }
   *retlen = bytesWritten;
   return 0;
}

static int vnb_panic_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	if (to >= mtd->size)
		len = 0;
	else if (to + len > mtd->size)
		len = mtd->size - to;
	return part->master->panic_write(part->master, to + part->offset,
				    len, retlen, buf);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_write_oob(struct mtd_info *pVirtualMtd, loff_t to, struct mtd_oob_ops *ops)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
   loff_t virtualOffset = to;
   loff_t physicalOffset;
   uint32_t startOffset;
   uint16_t virtualBlockNum;
   uint8_t  vnboob[NAND_OOB_SIZE];
   int error;
   struct mtd_oob_ops vnb_ops;
   uint16_t bootgen = bootgeneration+1;

   startOffset = mtd_mod_by_eb(virtualOffset, pVirtualMtd);
   // printk("%s: start.\n"i, __func__);
   if ((startOffset & (pPhysicalMtd->writesize - 1)) != 0) {
      printk("%s: unaligned offset %llx, abort\n", __func__, virtualOffset);
      return -EINVAL;
   }
   if ((ops->len)&&(ops->len!=pPhysicalMtd->writesize)) {
      printk("%s: surprising, in-band length is not writesize %i %i\n", __func__, ops->len, pPhysicalMtd->writesize);
   }

   /* We call the translate routine that allocs a new physical block if the
    * virtual block hasn't already been mapped. The mapping probably
    * already happened previously but this takes care of the case where
    * a bad block is replaced. */
   error = vnb_translateOffset_alloc(pVirtualMtd, virtualOffset, &physicalOffset);
   if (error) {
      printk("%s: virtual offset %llX not mapped, abort\n", __func__, virtualOffset);
      return error;
   }

   /* write the sonos VNB OOB data */
   virtualBlockNum = mtd_div_by_eb(virtualOffset, pVirtualMtd);
   memset(vnboob,0xff,NAND_OOB_SIZE);

   nand_oob_set_lblock(vnboob, virtualBlockNum);
   nand_oob_set_magic(vnboob, SONOS_NAND_OOB_MAGIC);
   // XXX BT should not have magic numbers in the following
   if(nand_oob_is_last_block(virtualOffset, pVirtualNandData->lastBlockOffset)) {
      nand_oob_set_magic(vnboob, SONOS_NAND_OOB_MAGIC_LAST);
   }

   nand_oob_set_bootgen(vnboob, bootgen);
#ifdef NAND_OOB_WITH_ECC
   nand_oob_set_ecc(vnboob, virtualBlockNum);
#endif

   if (ops->oobbuf) {
      if (ops->mode==MTD_OOB_AUTO) {
         int n=ops->ooblen;
#if defined(CONFIG_SONOS_LIMELIGHT)
         if (n>16) n=16;
         switch (n) {
            // BT: Fallthroughs below are intentional!
	 case 16:
	     vnboob[63]=ops->oobbuf[15];
	 case 15:
	     vnboob[62]=ops->oobbuf[14];
	 case 14:
	     vnboob[61]=ops->oobbuf[13];
	 case 13:
	     vnboob[60]=ops->oobbuf[12];
	 case 12:
	     vnboob[59]=ops->oobbuf[11];
	 case 11:
	     vnboob[58]=ops->oobbuf[10];
	 case 10:
	     vnboob[57]=ops->oobbuf[9];
	 case 9:
	     vnboob[56]=ops->oobbuf[8];
	 case 8:
	     vnboob[55]=ops->oobbuf[7];
	 case 7:
	     vnboob[54]=ops->oobbuf[6];
	 case 6:
	     vnboob[53]=ops->oobbuf[5];
	 case 5:
	     vnboob[52]=ops->oobbuf[4];
	 case 4:
	     vnboob[51]=ops->oobbuf[3];
	 case 3:
	     vnboob[50]=ops->oobbuf[2];
	 case 2:
	     vnboob[49]=ops->oobbuf[1];
	 case 1:
	     vnboob[48]=ops->oobbuf[0];
	 default:
	     ;
         }
#endif	// CONFIG_SONOS_LIMELIGHT
#ifdef CONFIG_SONOS_FENWAY
   // jffs2 is using a five byte magic number marker, and we need to not step on the
   // bad block marker, which is at offset 0 in the large page flash.  So map "logical bytes"
   // 0-4 onto "physical bytes" 1-5 for the large page devices in the Fenway family...
	if (n>6) n=6;
	if (!FenwayLP) {
		 switch (n) {
		     // BT: Fallthroughs below are intentional!
		 case 6:
		     vnboob[9]=ops->oobbuf[5];
		 case 5:
		     vnboob[4]=ops->oobbuf[4];
		 case 4:
		     vnboob[3]=ops->oobbuf[3];
		 case 3:
		     vnboob[2]=ops->oobbuf[2];
		 case 2:
		     vnboob[1]=ops->oobbuf[1];
		 case 1:
		     vnboob[0]=ops->oobbuf[0];
		 default:
		     ;
		 }
	 } else {
		 switch (n) {
		     // BT: Fallthroughs below are intentional!
		 case 6:
		     vnboob[9]=ops->oobbuf[10];
		 case 5:
		     vnboob[5]=ops->oobbuf[4];
		 case 4:
		     vnboob[4]=ops->oobbuf[3];
		 case 3:
		     vnboob[3]=ops->oobbuf[2];
		 case 2:
		     vnboob[2]=ops->oobbuf[1];
		 case 1:
		     vnboob[1]=ops->oobbuf[0];
		 default:
		     ;
	   }
	 }
#endif	// CONFIG_SONOS_FENWAY
      } else {
         int n;
	 printk("virtual_block: Nobody should ever call this without MTD_OOB_AUTO.\n");
         for (n=ops->ooboffs;n<(ops->ooboffs+ops->ooblen);n++) {
#ifdef CONFIG_SONOS_FENWAY
            if ((n<5)||(n==9))
#endif	// CONFIG_SONOS_FENWAY
			vnboob[n]=ops->oobbuf[n - ops->ooboffs];
         }
      }
   }

   firstMap = 0;
   vnb_ops.ooblen = NAND_OOB_SIZE;
   vnb_ops.ooboffs = 0;
   vnb_ops.datbuf = ops->datbuf;
   vnb_ops.len = ops->len;
   vnb_ops.mode = MTD_OOB_PLACE;
   vnb_ops.oobbuf = vnboob;
   error = pPhysicalMtd->write_oob(pPhysicalMtd, physicalOffset, &vnb_ops);
#ifdef VNB_DEBUG_FORCE_ERRORS
   if (!error && pVirtualNandData->forceWriteErrorOffset &&
       virtualOffset == pVirtualNandData->forceWriteErrorOffset) {
      /* only used to force an write error for block replacemnt testing */
      error = -97;
      pVirtualNandData->forceWriteErrorOffset = 0;
   }
#endif
   if (error) {
      int error2;
      // XXX BT could be argued need some fairly complicated updating
      // XXX of ops->retlen and opt->oobretlen here
      //printk("vnb_write_oob: physical write OOB (block num) error %d\n", error);
      //printk("%s: error %d  %llx %llx %x %x %x.\n",
      //	__func__, error, virtualOffset, physicalOffset, vnb_ops.len, vnb_ops.ooblen, vnb_ops.ooboffs);
      vnb_remove_mapping(pVirtualNandData, virtualOffset);
      error2 = pPhysicalMtd->block_markbad(pPhysicalMtd, physicalOffset);
      if (error2) {
         printk("%s: error %d marking physical block with offset %llX bad\n",
                __func__, error2, physicalOffset);
      }
      return error;
   }
   // XXX BT think the following is not 100% kosher but probably ok
   //   printk("%s: success %llx %llx %x %x %x.\n",
   //		__func__, virtualOffset, physicalOffset, vnb_ops.len, vnb_ops.ooblen, vnb_ops.ooboffs);
   pVirtualNandData->pagesWritten += ops->len / pPhysicalMtd->writesize;
   ops->retlen=ops->len;
   ops->oobretlen=ops->ooblen;
   return 0;
}


static int vnb_read_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->read_user_prot_reg(part->master, from,
					len, retlen, buf);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_get_user_prot_info(struct mtd_info *mtd,
		struct otp_info *buf, size_t len)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->get_user_prot_info(part->master, buf, len);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_read_fact_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->read_fact_prot_reg(part->master, from,
					len, retlen, buf);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_get_fact_prot_info(struct mtd_info *mtd, struct otp_info *buf,
		size_t len)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->get_fact_prot_info(part->master, buf, len);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}


static int vnb_write_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len, size_t *retlen, u_char *buf)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->write_user_prot_reg(part->master, from,
					len, retlen, buf);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_lock_user_prot_reg(struct mtd_info *mtd, loff_t from,
		size_t len)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->lock_user_prot_reg(part->master, from, len);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_writev(struct mtd_info *mtd, const struct kvec *vecs,
		unsigned long count, loff_t to, size_t *retlen)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	return part->master->writev(part->master, vecs, count,
					to + part->offset, retlen);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_read_oob(struct mtd_info *pVirtualMtd, loff_t from,
                        struct mtd_oob_ops *ops)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
   loff_t virtualOffset = from;
   loff_t physicalOffset;
   uint32_t startOffset;
   int error, i;
   char phys_oobbuf[MAX_PHYS_OOB_SIZE];
   struct mtd_oob_ops vnbops;

   startOffset = mtd_mod_by_eb(virtualOffset, pVirtualMtd);
   if ((startOffset & (pPhysicalMtd->writesize - 1)) != 0) {
      printk("%s: unaligned offset %llx, abort\n", __func__, virtualOffset);
      return -1;
   }

   error = vnb_translateOffset_alloc(pVirtualMtd, virtualOffset, &physicalOffset);
   if (error) {
       printk("%s: read OOB translate error %d %x %x %llx %i\n",
              __func__, error, ops->ooblen, ops->ooboffs, virtualOffset, ops->mode);
       return error;
   }

   /* OOB data is read into local buffer first */
   vnbops = *ops;
   vnbops.oobbuf = phys_oobbuf;
   vnbops.ooblen = pPhysicalMtd->oobavail * ((ops->ooblen + SONOS_OOBAVAIL - 1) / SONOS_OOBAVAIL);
   if (vnbops.ooblen > MAX_PHYS_OOB_SIZE) {
       vnbops.ooblen = MAX_PHYS_OOB_SIZE;
   }

   error = pPhysicalMtd->read_oob(pPhysicalMtd, physicalOffset, &vnbops);
   if (error) {
      printk("%s: physical read OOB error %d %x %x %llx %i\n", __func__, 
             error, ops->ooblen, ops->ooboffs, virtualOffset, ops->mode);
   }

   /* Now copy OOB data from local buffer to caller's buffer. */
   if (ops->mode == MTD_OOB_AUTO) {
       unsigned int call_index, phys_index, k;
       phys_index = SonosOobPhysStart;
       call_index = k = 0;
       while (call_index < ops->ooblen) {
           ops->oobbuf[call_index++] = phys_oobbuf[phys_index++];
           if (++k >= SONOS_OOBAVAIL) {
               /* skip over VNB data and unused OOB data */
               phys_index += SONOS_OOB_PHYS_SKIP;
               k = 0;
           }
           if (phys_index >= MAX_PHYS_OOB_SIZE) {
               break;
           }
       }
       ops->oobretlen = call_index;
   } else {
       unsigned int ooboff;
       ooboff = (ops->mode == MTD_OOB_PLACE) ? ops->ooboffs : 0;
       for (i = 0; i < ops->ooblen; i++) {
           if ((ooboff+i >= SonosNandOobMagic0) && (ooboff+i <= SONOS_NAND_OOB_LBLOCK1)) {
               ops->oobbuf[i] = 0xff;
           } else {
               ops->oobbuf[i] = phys_oobbuf[i];
           }
       }
       ops->oobretlen = ops->ooblen;
   }

   return error;
}

static void vnb_load_mdp(struct mtd_info *mtd)
{
#ifdef CONFIG_SONOS_FENWAY
    size_t len;
    vnb_read(mtd, MdpOffset, sizeof(struct manufacturing_data_page), &len, (u_char*)&sys_mdp);
#endif	// CONFIG_SONOS_FENWAY
}

static void vnb_sync(struct mtd_info *pVirtualMtd)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;

   pPhysicalMtd->sync(pPhysicalMtd);
}

static int vnb_suspend(struct mtd_info *mtd)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	return part->master->suspend(part->master);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static void vnb_resume(struct mtd_info *mtd)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	part->master->resume(part->master);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
#endif
}

static int vnb_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)mtd;
   int error;
   loff_t phys_ofs;

   error = vnb_translateOffset(mtd, ofs, &phys_ofs);
   if (error) {
      /* the virtual block didn't map to a physical block so it can't be bad */
      return 0;
   }

   return pVirtualNandData->pPhysicalMtd->block_isbad(pVirtualNandData->pPhysicalMtd, phys_ofs);
}

static int vnb_block_markbad(struct mtd_info *pVirtualMtd, loff_t virtualOffset)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
   int error=0;
   loff_t physicalOffset;
   uint16_t physicalBlockNum;
   uint16_t virtualBlockNum;

   virtualBlockNum = mtd_div_by_eb(virtualOffset, pVirtualMtd);
   physicalBlockNum = pVirtualNandData->block_map[virtualBlockNum];
   if (physicalBlockNum > 0) {
      /* remove mapping */
      pVirtualNandData->block_map[virtualBlockNum] = 0;

      /* and mark the physical block bad */
      physicalOffset = physicalBlockNum * pPhysicalMtd->erasesize;
      error = pPhysicalMtd->block_markbad(pPhysicalMtd, physicalOffset);
   }
   return error;
}

static int vnb_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	if ((len + ofs) > mtd->size)
		return -EINVAL;
	return part->master->lock(part->master, ofs + part->offset, len);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

static int vnb_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
#ifdef NOPE
	struct mtd_part *part = PART(mtd);
	if ((len + ofs) > mtd->size)
		return -EINVAL;
	return part->master->unlock(part->master, ofs + part->offset, len);
#else
   VNBDBG(printk("%s not supported yet\n", __func__));
   return 0;
#endif
}

typedef struct _VNB_ERASE_BLOCK {
   uint32_t errors;
   uint32_t totalBlocks;
   uint32_t blockCount;
   struct erase_info *pClientEraseInfo;
} VNB_ERASE_BLOCK;

/* can just an erase_info be enough? */
typedef struct _VNB_ERASE_INFO {
   struct erase_info  eraseInfo;
} VNB_ERASE_INFO;

/* Do the actual erase.
 * On error mark the block bad.
 */
static int vnb_phys_erase(struct mtd_info *pVirtualMtd, loff_t physicalOffset)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
   struct erase_info ei;
   int error, error1;

   //printk("%s: physicalOffset %llx\n", __func__, physicalOffset);

   if ((physicalOffset > pVirtualNandData->lastPhysBlock * pVirtualMtd->erasesize) ||
       (physicalOffset < (loff_t) FirstMappablePhysicalBlock * pVirtualMtd->erasesize))
	return -EPERM;

   /* erase flash */
   ei.mtd = pPhysicalMtd;
   ei.addr = physicalOffset;
   ei.len = pPhysicalMtd->erasesize;
   ei.callback = 0;
   ei.fail_addr = MTD_FAIL_ADDR_UNKNOWN;

   error = pPhysicalMtd->erase(pPhysicalMtd, &ei);
   if (error) {
       printk("%s: error %d at physical offset %llx\n", 
               __func__, error, physicalOffset);

       /* the physical block couldn't be erased so mark it bad... */
       error1 = pPhysicalMtd->block_markbad(pPhysicalMtd, physicalOffset);
       if (error1) {
           printk("%s: error %d marking physical block with offset %llX bad\n", 
                   __func__, error1, physicalOffset);
       }
   }

   return error;
}

static int vnb_erase(struct mtd_info *pVirtualMtd, struct erase_info *instr)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd;
   uint32_t bytesErased;
   int error;
   loff_t virtualOffset = instr->addr;
   loff_t physicalOffset;
   uint32_t startOffset;
   uint32_t totalBlocks;

   //printk("--vnb_erase offset %llx, len %x\n", instr->start, len);

   if (pVirtualMtd == NULL) {
      panic("vnb_erase passed NULL virtual MTD\n");
   }
   pPhysicalMtd = pVirtualNandData->pPhysicalMtd;

   startOffset = mtd_mod_by_eb(virtualOffset, pVirtualMtd);
   if (startOffset != 0) {
      printk("vnb_erase: unaligned offset %llx, abort\n", virtualOffset);
      return -EINVAL;
   }

   totalBlocks = mtd_div_by_eb(instr->len, pPhysicalMtd);

   /* allocate everything at once */
   bytesErased = 0;
   while (bytesErased < instr->len) {
      error = vnb_translateOffset(pVirtualMtd, virtualOffset, &physicalOffset);
      if (!error) {
         error = vnb_phys_erase(pVirtualMtd, physicalOffset);
#ifdef VNB_DEBUG_FORCE_ERRORS
         if (!error && pVirtualNandData->forceEraseErrorOffset && 
             virtualOffset == pVirtualNandData->forceEraseErrorOffset) {
             /* only used to force an erase error for block replacemnt testing */
             error = -99;
             pVirtualNandData->forceEraseErrorOffset = 0;
         }
#endif
         if (error) {
            vnb_remove_mapping(pVirtualNandData, virtualOffset);
	    /* Next pass will try this again and fall through to _alloc below */
            virtualOffset -= pVirtualMtd->erasesize;
            bytesErased -= pVirtualMtd->erasesize;
            pVirtualNandData->blocksErased--;
	 }
      } else {
        /* We call the translate routine that allocs a new physical block if the 
         * virtual block hasn't already been mapped. The mapping probably
         * already happened previously but this takes care of the case where
         * a bad block is replaced. The alloc also erases the block */
         error = vnb_translateOffset_alloc(pVirtualMtd, virtualOffset, &physicalOffset);
         if (error) {
            printk("vnb_erase: fatal error, virtual offset %llX not mapped, abort\n", virtualOffset);
            instr->fail_addr = virtualOffset; 
            return error;
         }
      }

      virtualOffset += pVirtualMtd->erasesize;
      bytesErased += pVirtualMtd->erasesize;
      pVirtualNandData->blocksErased++;
   }

   instr->state=MTD_ERASE_DONE;
   mtd_erase_callback(instr);

   return 0;
}

void vnb_MapBlock(struct mtd_info *pPhysicalMtd, loff_t offs, uint8_t *pOob);

void vnb_initMtd(struct mtd_info *pVirtualMtd)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData;
   struct mtd_info *master;

   pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   master = pVirtualNandData->pPhysicalMtd;
   if (master == NULL) {
      printk("%s: physical MTD is NULL\n", __func__);
      return;
   }

   /* set up the MTD object for this */
   pVirtualNandData->virtualMtd.type = master->type;
   pVirtualNandData->virtualMtd.flags = master->flags;
   pVirtualNandData->virtualMtd.size = master->size;
   pVirtualNandData->virtualMtd.writesize = master->writesize;
   pVirtualNandData->virtualMtd.oobsize = master->oobsize;
   pVirtualNandData->virtualMtd.oobavail = SONOS_OOBAVAIL;
   pVirtualNandData->virtualMtd.subpage_sft = master->subpage_sft;

   pVirtualNandData->virtualMtd.erasesize = master->erasesize;
   pVirtualNandData->virtualMtd.erasesize_shift = master->erasesize_shift;
   //pVirtualNandData->virtualMtd.erasesize_shift = 6;
   pVirtualNandData->virtualMtd.writesize_shift = master->writesize_shift;
   pVirtualNandData->virtualMtd.erasesize_mask = master->erasesize_mask;
   pVirtualNandData->virtualMtd.writesize_mask = master->writesize_mask;

   pVirtualNandData->virtualMtd.name = "Virtual Nand Block";
   pVirtualNandData->virtualMtd.owner = master->owner;

   pVirtualNandData->virtualMtd.read  = vnb_read;
   pVirtualNandData->virtualMtd.write = vnb_write;

   if (master->panic_write)
       pVirtualNandData->virtualMtd.panic_write = vnb_panic_write;

   if (master->point && master->unpoint) {
       pVirtualNandData->virtualMtd.point = vnb_point;
       pVirtualNandData->virtualMtd.unpoint = vnb_unpoint;
   }

   if (master->read_oob)
       pVirtualNandData->virtualMtd.read_oob = vnb_read_oob;
   if (master->write_oob)
       pVirtualNandData->virtualMtd.write_oob = vnb_write_oob;
   if (master->read_user_prot_reg)
       pVirtualNandData->virtualMtd.read_user_prot_reg = vnb_read_user_prot_reg;
   if (master->read_fact_prot_reg)
       pVirtualNandData->virtualMtd.read_fact_prot_reg = vnb_read_fact_prot_reg;
   if (master->write_user_prot_reg)
       pVirtualNandData->virtualMtd.write_user_prot_reg = vnb_write_user_prot_reg;
   if (master->lock_user_prot_reg)
       pVirtualNandData->virtualMtd.lock_user_prot_reg = vnb_lock_user_prot_reg;
   if (master->get_user_prot_info)
       pVirtualNandData->virtualMtd.get_user_prot_info = vnb_get_user_prot_info;
   if (master->get_fact_prot_info)
       pVirtualNandData->virtualMtd.get_fact_prot_info = vnb_get_fact_prot_info;
   if (master->sync)
       pVirtualNandData->virtualMtd.sync = vnb_sync;

   if (master->read_special)
       pVirtualNandData->virtualMtd.read_special = vnb_read_special;
   pVirtualNandData->virtualMtd.devid = master->devid;
   pVirtualNandData->virtualMtd.set_last_block = vnb_set_last_block;

   if (master->suspend && master->resume) {
			pVirtualNandData->virtualMtd.suspend = vnb_suspend;
			pVirtualNandData->virtualMtd.resume = vnb_resume;
	}
	if (master->writev)
		pVirtualNandData->virtualMtd.writev = vnb_writev;
	if (master->lock)
		pVirtualNandData->virtualMtd.lock = vnb_lock;
	if (master->unlock)
		pVirtualNandData->virtualMtd.unlock = vnb_unlock;
	if (master->block_isbad)
		pVirtualNandData->virtualMtd.block_isbad = vnb_block_isbad;
	if (master->block_markbad)
		pVirtualNandData->virtualMtd.block_markbad = vnb_block_markbad;
	pVirtualNandData->virtualMtd.erase = vnb_erase;

#ifdef CONFIG_SONOS_FENWAY
	if ( FenwayLP ) {
		pVirtualNandData->virtualMtd.ecclayout = &sonos_ecclayout_64;
		pVirtualNandData->nextPhysBlock = FIRST_MAPPABLE_PHYSICAL_BLOCK_LP;
		FirstMappablePhysicalBlock = FIRST_MAPPABLE_PHYSICAL_BLOCK_LP;
	} else {
		pVirtualNandData->virtualMtd.ecclayout = &sonos_ecclayout_16;
		pVirtualNandData->nextPhysBlock = FIRST_MAPPABLE_PHYSICAL_BLOCK;
		FirstMappablePhysicalBlock = FIRST_MAPPABLE_PHYSICAL_BLOCK;
	}
#else
	pVirtualNandData->virtualMtd.ecclayout = &sonos_ecclayout;
	pVirtualNandData->nextPhysBlock = FIRST_MAPPABLE_PHYSICAL_BLOCK;
#endif
	pVirtualNandData->pagesRead = pVirtualNandData->pagesWritten = pVirtualNandData->blocksErased = 0;

	pVirtualNandData->lastPhysBlock = mtd_div_by_eb(master->size, master) - 1;

	vnb_load_mdp(pVirtualMtd);

   printk("%s: VirtualMTD %p, size %llX, writesize %X, oobsize %X, oobavail %X,\n"
          "  erasesize %X, erasesize_shift %X \n",
          __func__, pVirtualMtd,
          pVirtualMtd->size, pVirtualMtd->writesize, pVirtualMtd->oobsize,
          pVirtualMtd->oobavail, pVirtualMtd->erasesize, pVirtualMtd->erasesize_shift);
}

/* called from the nand scan function during initialization */
void vnb_MapBlock(struct mtd_info *pPhysicalMtd, loff_t offs, uint8_t *pOob)
{
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData;
   uint16_t physicalBlockNum;
   uint16_t virtualBlockNum;

   if(scancount++ < FlashUbootBlocks) return;

   pVirtualNandData = pPhysicalMtd->pVirtualNand;
   if (pVirtualNandData == NULL) {
      printk("Physical MTD %p missing virutal NAND data\n", pPhysicalMtd);
      return;
   }

#ifdef NAND_OOB_WITH_ECC
   nand_oob_get_ecc(pOob);
#endif
   if (NAND_OOB_MAGIC_OK(pOob)) {
      virtualBlockNum = nand_oob_get_lblock(pOob);
      physicalBlockNum = mtd_div_by_eb(offs, pPhysicalMtd);

      if ((pVirtualNandData->block_map[virtualBlockNum] != 0) &&
          (pVirtualNandData->block_map[virtualBlockNum] != physicalBlockNum)) {
         printk("%s: replacing mapping of virtual block %X from %X to %X\n",
                __func__, virtualBlockNum,
                pVirtualNandData->block_map[virtualBlockNum], physicalBlockNum);
      }

      /* map it */
      vnb_set_rel_block(pVirtualNandData, virtualBlockNum, physicalBlockNum);
      //printk("Mapping block: %x %x.\n", physicalBlockNum, virtualBlockNum);
   } else {
       //printk("Block %llx unmapped.\n", offs);
   }
}

#ifdef DEBUG_JFFS
uint32_t vnb_getJffsPhysOffset(uint32_t ofs)
{
    uint32_t physoff, off = ofs + JFFS_OFFSET; /* AJ: Should be made sane somehow? */
    uint32_t vblock, pblock, vb;

    if (GlobalVnbDataPtr == NULL)
	return 0xffffff33;

    vblock = off >> NAND_BLOCK_SHIFT;
    if (vblock > MAX_BLOCKS)
	return 0xffffff44;

    pblock = GlobalVnbDataPtr->block_map[vblock];
    for (vb = 0; vb < MAX_BLOCKS; vb++) {
	if ((vb != vblock) && (pblock == GlobalVnbDataPtr->block_map[vb]))
	    printk("  ofs %X, vblock %X maps to phys block %X as does vb %X\n",
		   ofs, vblock, pblock, vb);
    }

    physoff = GlobalVnbDataPtr->block_map[vblock] << NAND_BLOCK_SHIFT;
    return physoff;
}
#endif	// DEBUG_JFFS

#ifdef CONFIG_PROC_FS
/*====================================================================*/
/* Support for /proc/vnb */

static int vnb_findNextAvailBlock(VIRTUAL_NAND_BLOCK_T *pVirtualNandData, 
                                  int *pNextAvailBlock)
{
   uint32_t physBlock;
   uint32_t physicalOffset;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;

   physBlock = FirstMappablePhysicalBlock;
   while (physBlock < pVirtualNandData->lastPhysBlock) {
      physicalOffset = physBlock * pPhysicalMtd->erasesize;

      if (!vnb_is_usedblock(physBlock, pVirtualNandData) &&
          !pPhysicalMtd->block_isbad(pVirtualNandData->pPhysicalMtd, physicalOffset)) {
         /* success */
         *pNextAvailBlock = physBlock;
         return 1;
      }
      physBlock++;
   }

   /* nothing available */
   *pNextAvailBlock = 0xffffffff;
   return 0;
}


static struct proc_dir_entry *proc_vnb;

static int vnb_write_proc(struct file *file, const char *buf, 
                          unsigned long count, void *data)
{
#ifdef VNB_DEBUG_FORCE_ERRORS
   char *erase0 = "erase0";
   char *write0 = "write0";
   char *erase1 = "erase1";
   char *write1 = "write1";
   char *translate = "translate";
   unsigned int addr;
   loff_t physoff;

   if (strncmp(buf, erase0, strlen(erase0)) == 0) {
      printk("VNB test: next erase to kernel0 section will fail\n");
      GlobalVnbDataPtr->forceEraseErrorOffset = 0x00040000;
   }
   else if (strncmp(buf, erase1, strlen(erase1)) == 0) {
      printk("VNB test: next erase to kernel1 section will fail\n");
      GlobalVnbDataPtr->forceEraseErrorOffset = 0x03100000;
   }
   else if (strncmp(buf, write0, strlen(write0)) == 0) {
      printk("VNB test: next write to kernel0 section will fail\n");
      GlobalVnbDataPtr->forceWriteErrorOffset = 0x00040000;
   }
   else if (strncmp(buf, write1, strlen(write1)) == 0) {
      printk("VNB test: next write to kernel1 section will fail\n");
      GlobalVnbDataPtr->forceWriteErrorOffset = 0x03100000;
   }
   else if (strncmp(buf, translate, 9) == 0) {
	  sscanf(buf + 10, "%x", &addr);
      //printk("VNB test: next write to kernel1 section will fail\n");
	  vnb_translateOffset(&GlobalVnbDataPtr->virtualMtd, addr, &physoff);
	  printk("Address %x = %llx.\n", addr, physoff);
   }
   else {
      printk("VNB test: unknown option\n");
   }
#else
   printk("VNB force errors disabled\n");
#endif

	return count;
}

static int vnb_read_proc(char *page, char **start, off_t off, int count,
                         int *eof, void *data_unused)
{
	int len = 0, found;
   uint32_t block, physicalBlockNum;
   uint32_t lowBlocks, highBlocks;
   struct mtd_ecc_stats *ecc_stats = &GlobalVnbDataPtr->pPhysicalMtd->ecc_stats;
   struct mtd_info *physMtd = GlobalVnbDataPtr->pPhysicalMtd;
   struct nand_chip *chip = physMtd->priv;

	len += sprintf(page+len, "First block 0x%x, ", FirstMappablePhysicalBlock);

   found = vnb_findNextAvailBlock(GlobalVnbDataPtr, &block);
   if (found)
      len += sprintf(page+len, "first available block 0x%x\n", block);
   else
      len += sprintf(page+len, "no available blocks\n");

	len += sprintf(page+len, "blocks used 0x%x\n", GlobalVnbDataPtr->blocksUsed);

   lowBlocks = highBlocks = 0;
   for (block = FIRST_LOGICAL_BLOCK; block < NumVirtualBlocks; block++) {
      physicalBlockNum = GlobalVnbDataPtr->block_map[block];
      if (physicalBlockNum != 0) {
         if (physicalBlockNum < FirstMappablePhysicalBlock) {
            lowBlocks++;
         }
         else if (physicalBlockNum > GlobalVnbDataPtr->lastPhysBlock) {
            highBlocks++;
         }
      }
   }
   if ((lowBlocks + highBlocks) > 0)
      len += sprintf(page+len,
                     "%d Virtual blocks mapped outside valid physical range (%d low, %d high)\n",
                     lowBlocks + highBlocks, lowBlocks, highBlocks);
   else
      len += sprintf(page+len, "All virtual blocks map to valid physical blocks\n");
   len += sprintf(page+len, "%i blocks scanned.\n", scancount);

   len += sprintf(page+len, "Blocks read/written/erased: %i/%i/%i.\n", GlobalVnbDataPtr->pagesRead, GlobalVnbDataPtr->pagesWritten, GlobalVnbDataPtr->blocksErased);
   len += sprintf(page+len, "Ecc status corrected/failed/bad: %i/%i/%i\n", ecc_stats->corrected, ecc_stats->failed, ecc_stats->badblocks);

   len += sprintf(page+len, "NAND write_page     %p\n", chip->write_page);
   len += sprintf(page+len, "NAND ecc.write_oob  %p\n", chip->ecc.write_oob);
   len += sprintf(page+len, "NAND ecc.write_page %p\n", chip->ecc.write_page);
   len += sprintf(page+len, "NAND Phys OOBavail  %u\n", physMtd->oobavail);

#if defined(CONFIG_SONOS_LIMELIGHT)
   len += sprintf(page+len, "Idle Power saves %u\n", powerSaves);
#endif	// CONFIG_SONOS_LIMELIGHT

   *eof = 1;
   return len;
}

static int vnb_uboot_read_proc(char *page, char **start, off_t off, int count,
							   int *eof, void *data_unused)
{
	int len = 0;

#if defined(CONFIG_SONOS_LIMELIGHT)
	size_t retlen=0;
	char uboot_stamp[64] = "failed";
    GlobalVnbDataPtr->pPhysicalMtd->read(GlobalVnbDataPtr->pPhysicalMtd, 0, 64, &retlen, uboot_stamp);
    len += sprintf(page+len, "%s\n", &uboot_stamp[4]);
#ifdef CONFIG_MTD_VNB_PHYSICAL_WRITE
    len += sprintf(page+len, "UBoot upgrade supported\n");
#else
    len += sprintf(page+len, "UBoot upgrade not supported\n");
#endif
#else
    len += sprintf(page+len, "UBoot upgrade not supported\n");
#endif	// CONFIG_SONOS_LIMELIGHT

    *eof = 1;
    return len;
}

#ifdef NAND_OOB_WITH_ECC
#ifdef NAND_OOB_ECC_TEST
static int proc_oobtest_read(char *page, char **start, off_t off, int count,
			   int *eof, void *data_unused)
{
	int len  = 0;
	len += sprintf(page+len, "This is used for hacking Nand OOB data "
		"to simulate OOB data corrupt\n");
	if ( nand_oob_ecc_test_flag == 0 )
		len += sprintf(page+len, "The Nand OOB data is under normal "
			"operation now\n");
	else {
		int two_bits_flag = nand_oob_ecc_test_flag/100;
		int real_test_flag = nand_oob_ecc_test_flag - two_bits_flag * 100;
		int byte = real_test_flag / 8 + 1;
		int bit = real_test_flag % 8 - 1;
		if ( bit < 0 ) {
			byte--;
			bit = 7;
		}
		len += sprintf(page+len, "The Nand OOB data is under test\n");
		len += sprintf(page+len, "The %d bit of %d byte will be "
			"flipped at virtual block %d\n", bit, byte,
			nand_oob_ecc_badeccbk);
		if ( two_bits_flag ) {
			if ( bit < 7 )
				len += sprintf(page+len, "The %d bit of %d "
					"byte will be flipped\n", bit + 1, byte);
			else
				len += sprintf(page+len, "The 0 bit of %d "
					"byte will be flipped\n", byte);
		}

		len += sprintf(page+len, "\tByte 1,2  are Magic\n");
		len += sprintf(page+len, "\tByte 3,4  are Bootgen\n");
		len += sprintf(page+len, "\tByte 5,6  are Logical block\n");
		len += sprintf(page+len, "\tByte 7,8  are Reserved\n");
		len += sprintf(page+len, "\tByte 9,11 are ECC\n");
	}
	return len;
}

static int proc_oobtest_write(struct file *file, const char *buf,
			   unsigned long count, void *data)
{
	unsigned long tmp;
	char *p = (char *) buf;

	tmp = simple_strtoul(p, &p, 10);
	if (!p || (*p && (*p != '\n')))
		return -EINVAL;
	if ( tmp == 0 ) {
		nand_oob_ecc_test_flag = 0;
		printk("The Nand OOB data is under normal operation now\n");
	} else if ( (tmp > 88 && tmp < 101 ) || (tmp > 188 )) {
		printk("Invalid input %ld(1..88 and 101..188)\n", tmp);
	}
	nand_oob_ecc_test_flag = tmp;
	return count;
}

static int proc_badoobbk_write(struct file *file, const char *buf,
			   unsigned long count, void *data)
{
	unsigned long tmp;
	char *p = (char *) buf;

	tmp = simple_strtoul(p, &p, 10);
	if (!p || (*p && (*p != '\n')))
		return -EINVAL;
	nand_oob_ecc_badeccbk = tmp;
	return count;
}
#endif
#endif

/* this function skips the usual VNB translatation and writes directly */
int vnb_writePhysical(struct mtd_info *pVirtualMtd, loff_t to, size_t len, u_char *buf)
{
   int error;
#ifdef CONFIG_MTD_VNB_PHYSICAL_WRITE
   size_t retlen=0;
   VIRTUAL_NAND_BLOCK_T *pVirtualNandData = (VIRTUAL_NAND_BLOCK_T *)pVirtualMtd;
   struct mtd_info *pPhysicalMtd = pVirtualNandData->pPhysicalMtd;
   struct erase_info ei;

   /* erase flash */
   ei.mtd = pPhysicalMtd;
   ei.addr = to;
   ei.len = len;
   ei.callback = 0;
   ei.fail_addr = MTD_FAIL_ADDR_UNKNOWN;

   error = pPhysicalMtd->erase(pPhysicalMtd, &ei);
   if (error) {
	   printk("%s: erase error %d\n", __func__, error);
	   return error;
   }
   error = pPhysicalMtd->write(pPhysicalMtd, to, len, &retlen, buf);
   if (error) {
	   printk("%s: write error %d\n", __func__, error);
   }
#else
   error = -EPERM;
#endif
   return error;
}

/*====================================================================*/
/* Init code */

static int __init init_vnb(void)
{
	if ((proc_vnb = create_proc_entry( "vnb", 0, NULL))) {
		proc_vnb->read_proc = vnb_read_proc;
		proc_vnb->write_proc = vnb_write_proc;
	}
	if ((proc_vnb = create_proc_entry( "uboot", 0, NULL))) {
		proc_vnb->read_proc = vnb_uboot_read_proc;
	}
#ifdef NAND_OOB_WITH_ECC
#ifdef NAND_OOB_ECC_TEST
	nand_oob_ecc_test_flag = 0;
	nand_oob_ecc_badeccbk = 0;
	if ((proc_oobtest = create_proc_entry( "oobtest", 0, NULL))) {
		proc_oobtest->read_proc = proc_oobtest_read;
		proc_oobtest->write_proc = proc_oobtest_write;
	}
	if ((proc_oobtest = create_proc_entry( "badoobbk", 0, NULL))) {
		proc_oobtest->read_proc = proc_oobtest_read;
		proc_oobtest->write_proc = proc_badoobbk_write;
	}
#endif
#endif

	return 0;
}

static void __exit cleanup_vnb(void)
{
   if (proc_vnb)
		remove_proc_entry( "vnb", NULL);
#ifdef NAND_OOB_WITH_ECC
#ifdef NAND_OOB_ECC_TEST
   if (proc_oobtest)
		remove_proc_entry( "oobtest", NULL);
   if (proc_badoobbk)
		remove_proc_entry( "badoobbk", NULL);
#endif
#endif
}

module_init(init_vnb);
module_exit(cleanup_vnb);

#endif /* CONFIG_PROC_FS */
