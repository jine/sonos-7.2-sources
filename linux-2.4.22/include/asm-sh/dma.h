#ifndef __ASM_SH_DMA_H
#define __ASM_SH_DMA_H

#include <linux/config.h>
#include <asm/io.h>		/* need byte IO */

#define MAX_DMA_CHANNELS 8
#define SH_MAX_DMA_CHANNELS 4

/* The maximum address that we can perform a DMA transfer to on this platform */
/* Don't define MAX_DMA_ADDRESS; it's useless on the SuperH and any
   occurrence should be flagged as an error.  */
/* But... */
/* XXX: This is not applicable to SuperH, just needed for alloc_bootmem */
#define MAX_DMA_ADDRESS      (PAGE_OFFSET+0x10000000)

#if defined(__sh3__)
#define SAR ((unsigned long[]){0xa4000020,0xa4000030,0xa4000040,0xa4000050})
#define DAR ((unsigned long[]){0xa4000024,0xa4000034,0xa4000044,0xa4000054})
#define DMATCR ((unsigned long[]){0xa4000028,0xa4000038,0xa4000048,0xa4000058})
#define CHCR ((unsigned long[]){0xa400002c,0xa400003c,0xa400004c,0xa400005c})
#define DMAOR 0xa4000060UL
#elif defined(__SH4__)
#define SAR ((unsigned long[]){0xbfa00000,0xbfa00010,0xbfa00020,0xbfa00030,0xbfa00050,0xbfa00060,0xbfa00070,0xbfa00080})
#define DAR ((unsigned long[]){0xbfa00004,0xbfa00014,0xbfa00024,0xbfa00034,0xbfa00054,0xbfa00064,0xbfa00074,0xbfa00084})
#define DMATCR ((unsigned long[]){0xbfa00008,0xbfa00018,0xbfa00028,0xbfa00038,0xbfa00058,0xbfa00068,0xbfa00078,0xbfa00088})
#define CHCR ((unsigned long[]){0xbfa0000c,0xbfa0001c,0xbfa0002c,0xbfa0003c,0xbfa0005C,0xbfa0006C,0xbfa0007C,0xbfa0008C})
#define DMAOR 0xbfa00040UL
#endif

#define DMTE_IRQ ((int[]){DMTE0_IRQ,DMTE1_IRQ,DMTE2_IRQ,DMTE3_IRQ,DMTE4_IRQ,DMTE5_IRQ,DMTE6_IRQ,DMTE7_IRQ})

#define DMA_MODE_READ	0x00	/* I/O to memory, no autoinit, increment, single mode */
#define DMA_MODE_WRITE	0x01	/* memory to I/O, no autoinit, increment, single mode */
#define DMA_AUTOINIT	0x10

#define REQ_L	0x00000000
#define REQ_E	0x00080000
#define RACK_H	0x00000000
#define RACK_L	0x00040000
#define ACK_R	0x00000000
#define ACK_W	0x00020000
#define ACK_H	0x00000000
#define ACK_L	0x00010000
#define DM_INC	0x00004000
#define DM_DEC	0x00008000
#define SM_INC	0x00001000
#define SM_DEC	0x00002000
#define RS_DUAL	0x00000000
#define RS_IN	0x00000200
#define RS_OUT	0x00000300
#define TM_BURST 0x0000080
#define TS_8	0x00000010
#define TS_16	0x00000020
#define TS_32	0x00000030
#define TS_64	0x00000000
#define TS_BLK	0x00000040
#define CHCR_DE 0x00000001
#define CHCR_TE 0x00000002
#define CHCR_IE 0x00000004

#define DMAOR_COD	0x00000008
#define DMAOR_AE	0x00000004
#define DMAOR_NMIF	0x00000002
#define DMAOR_DME	0x00000001

#define DMA1_STAT_REG		0x08
#define DMA_MODE_CASCADE	0xC0
#define DMA2_STAT_REG           0xD0    /* status register (r) */

// ------------------------------------------------------

/*
 * NOTES about DMA transfers:
 *
 *  controller 1: channels 0-3, byte operations, ports 00-1F
 *  controller 2: channels 4-7, word operations, ports C0-DF
 *
 *  - ALL registers are 8 bits only, regardless of transfer size
 *  - channel 4 is not used - cascades 1 into 2.
 *  - channels 0-3 are byte - addresses/counts are for physical bytes
 *  - channels 5-7 are word - addresses/counts are for physical words
 *  - transfers must not cross physical 64K (0-3) or 128K (5-7) boundaries
 *  - transfer count loaded to registers is 1 less than actual count
 *  - controller 2 offsets are all even (2x offsets for controller 1)
 *  - page registers for 5-7 don't use data bit 0, represent 128K pages
 *  - page registers for 0-3 use bit 0, represent 64K pages
 *
 * DMA transfers are limited to the lower 16MB of _physical_ memory.  
 * Note that addresses loaded into registers must be _physical_ addresses,
 * not logical addresses (which may differ if paging is active).
 *
 *  Address mapping for channels 0-3:
 *
 *   A23 ... A16 A15 ... A8  A7 ... A0    (Physical addresses)
 *    |  ...  |   |  ... |   |  ... |
 *    |  ...  |   |  ... |   |  ... |
 *    |  ...  |   |  ... |   |  ... |
 *   P7  ...  P0  A7 ... A0  A7 ... A0   
 * |    Page    | Addr MSB | Addr LSB |   (DMA registers)
 *
 *  Address mapping for channels 5-7:
 *
 *   A23 ... A17 A16 A15 ... A9 A8 A7 ... A1 A0    (Physical addresses)
 *    |  ...  |   \   \   ... \  \  \  ... \  \
 *    |  ...  |    \   \   ... \  \  \  ... \  (not used)
 *    |  ...  |     \   \   ... \  \  \  ... \
 *   P7  ...  P1 (0) A7 A6  ... A0 A7 A6 ... A0   
 * |      Page      |  Addr MSB   |  Addr LSB  |   (DMA registers)
 *
 * Again, channels 5-7 transfer _physical_ words (16 bits), so addresses
 * and counts _must_ be word-aligned (the lowest address bit is _ignored_ at
 * the hardware level, so odd-byte transfers aren't possible).
 *
 * Transfer count (_not # bytes_) is limited to 64K, represented as actual
 * count - 1 : 64K => 0xFFFF, 1 => 0x0000.  Thus, count is always 1 or more,
 * and up to 128K bytes may be transferred on channels 5-7 in one operation. 
 *
 */

/* 8237 DMA controllers */
#define IO_DMA1_BASE	0x00	/* 8 bit slave DMA, channels 0..3 */
#define IO_DMA2_BASE	0xC0	/* 16 bit master DMA, ch 4(=slave input)..7 */

/* DMA controller registers */
#define DMA1_CMD_REG		0x08	/* command register (w) */
#define DMA1_STAT_REG		0x08	/* status register (r) */
#define DMA1_REQ_REG            0x09    /* request register (w) */
#define DMA1_MASK_REG		0x0A	/* single-channel mask (w) */
#define DMA1_MODE_REG		0x0B	/* mode register (w) */
#define DMA1_CLEAR_FF_REG	0x0C	/* clear pointer flip-flop (w) */
#define DMA1_TEMP_REG           0x0D    /* Temporary Register (r) */
#define DMA1_RESET_REG		0x0D	/* Master Clear (w) */
#define DMA1_CLR_MASK_REG       0x0E    /* Clear Mask */
#define DMA1_MASK_ALL_REG       0x0F    /* all-channels mask (w) */

#define DMA2_CMD_REG		0xD0	/* command register (w) */
#define DMA2_STAT_REG		0xD0	/* status register (r) */
#define DMA2_REQ_REG            0xD2    /* request register (w) */
#define DMA2_MASK_REG		0xD4	/* single-channel mask (w) */
#define DMA2_MODE_REG		0xD6	/* mode register (w) */
#define DMA2_CLEAR_FF_REG	0xD8	/* clear pointer flip-flop (w) */
#define DMA2_TEMP_REG           0xDA    /* Temporary Register (r) */
#define DMA2_RESET_REG		0xDA	/* Master Clear (w) */
#define DMA2_CLR_MASK_REG       0xDC    /* Clear Mask */
#define DMA2_MASK_ALL_REG       0xDE    /* all-channels mask (w) */

#define DMA_ADDR_0              0x00    /* DMA address registers */
#define DMA_ADDR_1              0x02
#define DMA_ADDR_2              0x04
#define DMA_ADDR_3              0x06
#define DMA_ADDR_4              0xC0
#define DMA_ADDR_5              0xC4
#define DMA_ADDR_6              0xC8
#define DMA_ADDR_7              0xCC

#define DMA_CNT_0               0x01    /* DMA count registers */
#define DMA_CNT_1               0x03
#define DMA_CNT_2               0x05
#define DMA_CNT_3               0x07
#define DMA_CNT_4               0xC2
#define DMA_CNT_5               0xC6
#define DMA_CNT_6               0xCA
#define DMA_CNT_7               0xCE

#define DMA_PAGE_0              0x87    /* DMA page registers */
#define DMA_PAGE_1              0x83
#define DMA_PAGE_2              0x81
#define DMA_PAGE_3              0x82
#define DMA_PAGE_5              0x8B
#define DMA_PAGE_6              0x89
#define DMA_PAGE_7              0x8A

#define DMA_MODE_CASCADE 0xC0   /* pass thru DREQ->HRQ, DACK<-HLDA only */

//--------------------------------------------------------------------

struct dma_info_t {
	unsigned int chan;
	unsigned int mode_read;
	unsigned int mode_write;
	unsigned long dev_addr;
	unsigned int mode;
	unsigned long mem_addr;
	unsigned int count;
};

static __inline__ void clear_dma_ff(unsigned int dmanr){}

/* These are in arch/sh/kernel/dma.c: */
extern unsigned long claim_dma_lock(void);
extern void release_dma_lock(unsigned long flags);
extern void setup_dma(unsigned int dmanr, struct dma_info_t *info);
extern void enable_dma(unsigned int dmanr);
extern void disable_dma(unsigned int dmanr);
extern void set_dma_mode(unsigned int dmanr, char mode);
extern void set_dma_addr(unsigned int dmanr, unsigned int a);
extern void set_dma_count(unsigned int dmanr, unsigned int count);
extern int get_dma_residue(unsigned int dmanr);

#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy 	(0)
#endif


#endif /* __ASM_SH_DMA_H */
