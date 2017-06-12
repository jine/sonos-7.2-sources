/* Virtual Nand Block support */


#ifndef VIRTUAL_BLOCK_H
#define VIRTUAL_BLOCK_H

#include <linux/mtd/mtd.h>

#ifdef CONFIG_VNB_DISABLE_UNSUPPORTED_PRINTKS
#define VNBDBG(x)
#else
#define VNBDBG(x) x
#endif


typedef struct _VIRTUAL_NAND_BLOCK
{
   struct mtd_info virtualMtd;      /* must be first */
   struct mtd_info *pPhysicalMtd;

   loff_t          lastBlockOffset;
   unsigned int    nextPhysBlock;
   unsigned int    lastPhysBlock;
   unsigned short *block_map; 
   unsigned char  *used_blocks;
   unsigned int    blocksUsed;
   unsigned short  maxVirtualBlockNum;
   unsigned int pagesRead, pagesWritten, blocksErased;

#ifdef VNB_DEBUG_FORCE_ERRORS
   unsigned int    forceEraseErrorOffset;
   unsigned int    forceWriteErrorOffset;
#endif

} VIRTUAL_NAND_BLOCK_T;


/* function prototypes */
struct mtd_info * vnb_init(struct mtd_info *pPhysicalMtd);
void vnb_initMtd(struct mtd_info *mtd);

#endif
