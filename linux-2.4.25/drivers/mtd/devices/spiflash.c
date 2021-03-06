
/*
 * MTD driver for the SPI Flash Memory support.
 *
 * $Id: spiflash.c,v 1.1.1.1 2007/03/19 16:40:08 holmgren Exp $
 *
 *
 * Copyright (c) 2005-2006 Atheros Communications Inc.
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*===========================================================================
** !!!!  VERY IMPORTANT NOTICE !!!!  FLASH DATA STORED IN LITTLE ENDIAN FORMAT
**
** This module contains the Serial Flash access routines for the Atheros SOC.
** The Atheros SOC integrates a SPI flash controller that is used to access
** serial flash parts. The SPI flash controller executes in "Little Endian"
** mode. THEREFORE, all WRITES and READS from the MIPS CPU must be
** BYTESWAPPED! The SPI Flash controller hardware by default performs READ
** ONLY byteswapping when accessed via the SPI Flash Alias memory region
** (Physical Address 0x0800_0000 - 0x0fff_ffff). The data stored in the
** flash sectors is stored in "Little Endian" format.
**
** The spiflash_write() routine performs byteswapping on all write
** operations.
**===========================================================================*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/delay.h>
#include <asm/io.h>
#include "spiflash.h"
#define PAGE_WRITE

/* debugging */
/* #define SPIFLASH_DEBUG */

#ifndef __BIG_ENDIAN
#error This driver currently only works with big endian CPU.
#endif

static char module_name[] = "spiflash";

#define MIN(a,b)        ((a) < (b) ? (a) : (b))
#define FALSE 	0
#define TRUE 	1

#define ROOTFS_NAME	"rootfs"

static __u32 spiflash_regread32(int reg);
static void spiflash_regwrite32(int reg, __u32 data);
static __u32 spiflash_sendcmd (int op);

int __init spiflash_init (void);
void __exit spiflash_exit (void);
static int spiflash_probe (void);
static int spiflash_erase (struct mtd_info *mtd,struct erase_info *instr);
static int spiflash_read (struct mtd_info *mtd, loff_t from,size_t len,size_t *retlen,u_char *buf);
static int spiflash_write (struct mtd_info *mtd,loff_t to,size_t len,size_t *retlen,const u_char *buf);

/* Flash configuration table */
struct flashconfig {
    __u32 byte_cnt;
    __u32 sector_cnt;
    __u32 sector_size;
    __u32 cs_addrmask;
} flashconfig_tbl[MAX_FLASH] =
    {
        { 0, 0, 0, 0},
        { STM_1MB_BYTE_COUNT, STM_1MB_SECTOR_COUNT, STM_1MB_SECTOR_SIZE, 0x0},
        { STM_2MB_BYTE_COUNT, STM_2MB_SECTOR_COUNT, STM_2MB_SECTOR_SIZE, 0x0},
        { STM_4MB_BYTE_COUNT, STM_4MB_SECTOR_COUNT, STM_4MB_SECTOR_SIZE, 0x0},
        { STM_8MB_BYTE_COUNT, STM_8MB_SECTOR_COUNT, STM_8MB_SECTOR_SIZE, 0x0}
    };

/* Mapping of generic opcodes to STM serial flash opcodes */
struct opcodes {
    __u16 code;
    __s8 tx_cnt;
    __s8 rx_cnt;
} stm_opcodes[] = {
        {STM_OP_WR_ENABLE, 1, 0},
        {STM_OP_WR_DISABLE, 1, 0},
        {STM_OP_RD_STATUS, 1, 1},
        {STM_OP_WR_STATUS, 1, 0},
        {STM_OP_RD_DATA, 4, 4},
        {STM_OP_FAST_RD_DATA, 1, 0},
        {STM_OP_PAGE_PGRM, 8, 0},
        {STM_OP_SECTOR_ERASE, 4, 0},
        {STM_OP_BULK_ERASE, 1, 0},
        {STM_OP_DEEP_PWRDOWN, 1, 0},
        {STM_OP_RD_SIG, 4, 1}
};

/* Driver private data structure */
struct spiflash_data {
	struct 	mtd_info       *mtd;	
	struct 	mtd_partition  *parsed_parts;     /* parsed partitions */
	void 	*spiflash_readaddr; /* memory mapped data for read  */
	void 	*spiflash_mmraddr;  /* memory mapped register space */
#ifdef PAGE_WRITE
	void	*gpio_mmraddr;
#endif
};

static struct spiflash_data *spidata;

extern int parse_redboot_partitions(struct mtd_info *master, struct mtd_partition **pparts);

/***************************************************************************************************/

static __u32
spiflash_regread32(int reg)
{
	volatile __u32 *data = (__u32 *)(spidata->spiflash_mmraddr + reg);

	return (*data);
}

static void 
spiflash_regwrite32(int reg, __u32 data)
{
	volatile __u32 *addr = (__u32 *)(spidata->spiflash_mmraddr + reg);

	*addr = data;
	return;
}
#ifdef PAGE_WRITE
static void
set_cs()
{
	volatile __u32 *addr = (__u32 *)(spidata->gpio_mmraddr + GPIO_DO);
	__u32 x;
	long flags;
	local_irq_save(flags);
	x=*addr;
	x &= ~(1<<6);
	*addr=x;
	x=*addr;
	local_irq_restore(flags);
}

static void
clear_cs()
{
	volatile __u32 *addr = (__u32 *)(spidata->gpio_mmraddr + GPIO_DO);
	__u32 x;
	long flags;
	local_irq_save(flags);
	x=*addr;
	x |= (1<<6);
	*addr=x;
	x=*addr;
	local_irq_restore(flags);
}
static void
init_cs()
{
	volatile __u32 *addr = (__u32 *)(spidata->gpio_mmraddr + GPIO_DO);
	__u32 x;
	long flags;
	local_irq_save(flags);
	x=*addr;
	x |= (1<<6);
	*addr=x;
	addr = (__u32 *)(spidata->gpio_mmraddr + GPIO_CR);
	x=*addr;
	x |= (1<<6);
	*addr=x;
	addr = (__u32 *)(spidata->gpio_mmraddr + GPIO_DO);
	x=*addr;
	local_irq_restore(flags);
}
#endif

static __u32 
spiflash_sendcmd (int op)
{
	 __u32 reg;
	 __u32 mask;
	struct opcodes *ptr_opcode;

	ptr_opcode = &stm_opcodes[op];

	do {
		reg = spiflash_regread32(SPI_FLASH_CTL);
	} while (reg & SPI_CTL_BUSY);

	spiflash_regwrite32(SPI_FLASH_OPCODE, ptr_opcode->code);

	reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | ptr_opcode->tx_cnt |
        	(ptr_opcode->rx_cnt << 4) | SPI_CTL_START;

	spiflash_regwrite32(SPI_FLASH_CTL, reg);
 
	if (ptr_opcode->rx_cnt > 0) {
        	do {
          		reg = spiflash_regread32(SPI_FLASH_CTL);
        	} while (reg & SPI_CTL_BUSY);

        	reg = (__u32) spiflash_regread32(SPI_FLASH_DATA);

        	switch (ptr_opcode->rx_cnt) {
        	case 1:
            		mask = 0x000000ff;
            		break;
        	case 2:
            		mask = 0x0000ffff;
            		break;
        	case 3:
            		mask = 0x00ffffff;
            		break;
        	default:
            		mask = 0xffffffff;
            		break;
        	}

        	reg &= mask;
   	}
	else {
       		reg = 0;
    	}

	return reg;
}

/* Probe SPI flash device
 * Function returns 0 for failure.
 * and flashconfig_tbl array index for success.
 */
static int 
spiflash_probe (void)
{
	__u32 sig;
   	int flash_size;

   	/* Read the signature on the flash device */
   	sig = spiflash_sendcmd(SPI_RD_SIG);

   	switch (sig) {
   	case STM_8MBIT_SIGNATURE:
            	flash_size = FLASH_1MB;
        	break;
        case STM_16MBIT_SIGNATURE:
            	flash_size = FLASH_2MB;
            	break;
        case STM_32MBIT_SIGNATURE:
            	flash_size = FLASH_4MB;
            	break;
        case STM_64MBIT_SIGNATURE:
            	flash_size = FLASH_8MB;
            	break;
        default:
	    	printk (KERN_WARNING "%s: Read of flash device signature failed!\n", module_name);
            	return (0);
   	}

   	return (flash_size);
}


static int 
spiflash_erase (struct mtd_info *mtd,struct erase_info *instr)
{
	struct opcodes *ptr_opcode;
	__u32 temp, reg;
	int finished = FALSE;

#ifdef SPIFLASH_DEBUG
   	printk (KERN_DEBUG "%s(addr = 0x%.8x, len = %d)\n",__FUNCTION__,instr->addr,instr->len);
#endif

   	/* sanity checks */
   	if (instr->addr + instr->len > mtd->size) return (-EINVAL);

	ptr_opcode = &stm_opcodes[SPI_SECTOR_ERASE];

	temp = ((__u32)instr->addr << 8) | (__u32)(ptr_opcode->code);
	spiflash_sendcmd(SPI_WRITE_ENABLE);
	do {
		reg = spiflash_regread32(SPI_FLASH_CTL);
	} while (reg & SPI_CTL_BUSY);

	spiflash_regwrite32(SPI_FLASH_OPCODE, temp);

	reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | ptr_opcode->tx_cnt | SPI_CTL_START;
	spiflash_regwrite32(SPI_FLASH_CTL, reg);

	do {
		reg = spiflash_sendcmd(SPI_RD_STATUS);
		if (!(reg & SPI_STATUS_WIP)) {
			finished = TRUE;
		}
	} while (!finished);

   	instr->state = MTD_ERASE_DONE;
   	if (instr->callback) instr->callback (instr);

#ifdef SPIFLASH_DEBUG
   	printk (KERN_DEBUG "%s return\n",__FUNCTION__);
#endif
   	return (0);
}

static int 
spiflash_read (struct mtd_info *mtd, loff_t from,size_t len,size_t *retlen,u_char *buf)
{
	u_char	*read_addr;

#ifdef SPIFLASH_DEBUG
   	printk (KERN_DEBUG "%s(from = 0x%.8x, len = %d)\n",__FUNCTION__,(__u32) from,(int)len);  
#endif

   	/* sanity checks */
   	if (!len) return (0);
   	if (from + len > mtd->size) return (-EINVAL);
	

   	/* we always read len bytes */
   	*retlen = len;

	read_addr = (u_char *)(spidata->spiflash_readaddr + from);
	memcpy(buf, read_addr, len);

   	return (0);
}

static int 
spiflash_write (struct mtd_info *mtd,loff_t to,size_t len,size_t *retlen,const u_char *buf)
{
	int done = FALSE, page_offset, bytes_left, finished;
	__u32 xact_len, spi_data = 0, opcode, reg;
#ifdef PAGE_WRITE
	__u32 y;
	int do_address_cycle;
#endif

#ifdef SPIFLASH_DEBUG
   	printk (KERN_DEBUG "%s(to = 0x%.8x, len = %d)\n",__FUNCTION__,(__u32) to,len); 
#endif

   	*retlen = 0;
	
   	/* sanity checks */
   	if (!len) return (0);
   	if (to + len > mtd->size) return (-EINVAL);
	
	opcode = stm_opcodes[SPI_PAGE_PROGRAM].code;
	bytes_left = len;
	
	while (done == FALSE) {
#ifdef PAGE_WRITE
		xact_len = MIN(bytes_left, STM_PAGE_SIZE);
		do_address_cycle=1;
#else
		xact_len = MIN(bytes_left, sizeof(__u32));
#endif

		/* 32-bit writes cannot span across a page boundary
		 * (256 bytes). This types of writes require two page
		 * program operations to handle it correctly. The STM part
		 * will write the overflow data to the beginning of the
		 * current page as opposed to the subsequent page.
		 */
		page_offset = (to & (STM_PAGE_SIZE - 1)) + xact_len;

		if (page_offset > STM_PAGE_SIZE) {
			xact_len -= (page_offset - STM_PAGE_SIZE);
		}

		spiflash_sendcmd(SPI_WRITE_ENABLE);
		
#ifdef PAGE_WRITE
		y=MIN(xact_len,4);
nextpiece:
		if (do_address_cycle) {
			opcode=((stm_opcodes[SPI_PAGE_PROGRAM].code)&SPI_OPCODE_MASK)|((__u32)to<<8);
		} else {
			opcode=0;
		}
                do {
                        reg = spiflash_regread32(SPI_FLASH_CTL);
                } while (reg & SPI_CTL_BUSY);
		if (do_address_cycle) set_cs();
		switch (y) {
#else
		opcode=((stm_opcodes[SPI_PAGE_PROGRAM].code)&SPI_OPCODE_MASK)|((__u32)to<<8);
                do {
                        reg = spiflash_regread32(SPI_FLASH_CTL);
                } while (reg & SPI_CTL_BUSY);

		switch (xact_len) {
#endif
			case 1:
#ifdef PAGE_WRITE
				if (!do_address_cycle)
					opcode = buf[0];
				else
#endif
			 	spi_data = buf[0];
				break;
			case 2:
#ifdef PAGE_WRITE
				if (!do_address_cycle)
					opcode = buf[0] | (buf[1] << 24);
				else
#endif
				spi_data = (buf[1] << 8) | buf[0];
				break;
			case 3:
#ifdef PAGE_WRITE
				if (!do_address_cycle)
					opcode = buf[0] | (buf[1] << 24) | (buf[2] << 16);
				else
#endif
				spi_data = (buf[2] << 16) | (buf[1] << 8) | buf[0];
				break;
			case 4:
#ifdef PAGE_WRITE
				if (!do_address_cycle)
					opcode = buf[0] | (buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8);
				else
#endif
				spi_data = (buf[3] << 24) | (buf[2] << 16) | 
							(buf[1] << 8) | buf[0];
				break;
#ifdef PAGE_WRITE
			case 5:
				opcode = (buf[3] << 8) | (buf[2] << 16) |
							(buf[1] << 24) | buf[0];
				spi_data = buf[4];
				break;
			case 6:
				opcode = (buf[3] << 8) | (buf[2] << 16) |
							(buf[1] << 24) | buf[0];
				spi_data = (buf[5] << 8) | buf[4];
				break;
			case 7:
				opcode = (buf[3] << 8) | (buf[2] << 16) |
							(buf[1] << 24) | buf[0];
				spi_data = (buf[6] << 16) | (buf[5] << 8) |
							buf[4];
				break;
			case 8:
				opcode = (buf[3] << 8) | (buf[2] << 16) |
							(buf[1] << 24) | buf[0];
				spi_data = (buf[7] << 24) | (buf[6] << 16) |
							(buf[5] << 8) | buf[4];
				break;
#endif
			default:
				printk("spiflash_write: default case\n");
				break;
		}
#ifdef PAGE_WRITE
		spiflash_regwrite32(SPI_FLASH_DATA, spi_data);
		spiflash_regwrite32(SPI_FLASH_OPCODE, opcode);
		reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | ((do_address_cycle)?(y + 4):(y)) | SPI_CTL_START;
#else
		spiflash_regwrite32(SPI_FLASH_DATA, spi_data);
		spiflash_regwrite32(SPI_FLASH_OPCODE, opcode);
		reg = (reg & ~SPI_CTL_TX_RX_CNT_MASK) | (xact_len + 4) | SPI_CTL_START;
#endif
		spiflash_regwrite32(SPI_FLASH_CTL, reg);
		
#ifdef PAGE_WRITE
		bytes_left -= y;
		to += y;
		buf += y;
		*retlen += y;
		xact_len-=y;
		if (xact_len) {
			y=MIN(xact_len,8);
			do_address_cycle=0;
			goto nextpiece;
		}
                do {
                        reg = spiflash_regread32(SPI_FLASH_CTL);
                } while (reg & SPI_CTL_BUSY);
		clear_cs();
#else
		bytes_left -= xact_len;
		to += xact_len;
		buf += xact_len;

   		*retlen += xact_len;
#endif

                finished = FALSE;

                do {
                        udelay(1);
                        reg = spiflash_sendcmd(SPI_RD_STATUS);
                        if (!(reg & SPI_STATUS_WIP)) {
                                finished = TRUE;
                        }
                } while (!finished);


		if (bytes_left == 0) {
			done = TRUE;
		}
	}

   	return (0);
}


int __init 
spiflash_init (void)
{
   	int result, i;
   	int index, num_parts;
	struct mtd_info *mtd;
	struct  mtd_partition  *mtd_parts;

   	spidata = kmalloc(sizeof(struct spiflash_data), GFP_KERNEL);
   	if (!spidata)
		return (-ENXIO);

	spidata->spiflash_mmraddr = ioremap_nocache(SPI_FLASH_MMR, SPI_FLASH_MMR_SIZE);
	if (!spidata->spiflash_mmraddr) {
       		printk (KERN_WARNING "%s: Failed to map flash device\n", module_name);
		kfree(spidata);
       		return (-ENXIO);
	}
#ifdef PAGE_WRITE
	spidata->gpio_mmraddr = ioremap_nocache(GPIO_MMR, GPIO_MMR_SIZE);
	if (!spidata->gpio_mmraddr) {
		printk (KERN_WARNING "%s: Failed to map GPIO\n", module_name);
		kfree(spidata);
		return (-ENXIO);
	}
	init_cs();
#endif

   	mtd = kmalloc(sizeof(struct mtd_info), GFP_KERNEL);
   	if (!mtd) {
		kfree(spidata);
		return (-ENXIO);
	}
	
   	memset (mtd,0,sizeof (*mtd));
	
   	printk ("MTD driver for SPI flash.\n");
   	printk ("%s: Probing for Serial flash ...\n", module_name);
   	if (!(index = spiflash_probe ())) {
       		printk (KERN_WARNING "%s: Found no serial flash device\n", module_name);
		kfree(mtd);
		kfree(spidata);
       		return (-ENXIO);
   	}
   	printk ("%s: Found SPI serial Flash.\n", module_name);
   	printk ("%d: size\n", flashconfig_tbl[index].byte_cnt);

	spidata->spiflash_readaddr = ioremap_nocache(SPI_FLASH_READ, flashconfig_tbl[index].byte_cnt);
	if (!spidata->spiflash_readaddr) {
       		printk (KERN_WARNING "%s: Failed to map flash device\n", module_name);
		kfree(mtd);
		kfree(spidata);
       		return (-ENXIO);
	}

   	mtd->name = module_name;
   	mtd->type = MTD_NORFLASH;
   	mtd->flags = (MTD_CAP_NORFLASH|MTD_WRITEABLE);
   	mtd->size = flashconfig_tbl[index].byte_cnt;
   	mtd->erasesize = flashconfig_tbl[index].sector_size;
   	mtd->numeraseregions = 0;
   	mtd->eraseregions = NULL;
   	mtd->module = THIS_MODULE;
   	mtd->erase = spiflash_erase;
   	mtd->read = spiflash_read;
   	mtd->write = spiflash_write;
	
#ifdef SPIFLASH_DEBUG
	printk (KERN_DEBUG
		   "mtd->name = %s\n"
		   "mtd->size = 0x%.8x (%uM)\n"
		   "mtd->erasesize = 0x%.8x (%uK)\n"
		   "mtd->numeraseregions = %d\n",
		   mtd->name,
		   mtd->size, mtd->size / (1024*1024),
		   mtd->erasesize, mtd->erasesize / 1024,
		   mtd->numeraseregions);

   	if (mtd->numeraseregions) {
	 	for (result = 0; result < mtd->numeraseregions; result++) {
	   		printk (KERN_DEBUG
			   "\n\n"
			   "mtd->eraseregions[%d].offset = 0x%.8x\n"
			   "mtd->eraseregions[%d].erasesize = 0x%.8x (%uK)\n"
			   "mtd->eraseregions[%d].numblocks = %d\n",
			   result,mtd->eraseregions[result].offset,
			   result,mtd->eraseregions[result].erasesize,mtd->eraseregions[result].erasesize / 1024,
			   result,mtd->eraseregions[result].numblocks);
         	}
    	}
#endif

#ifndef CONFIG_BLK_DEV_INITRD
   	/* parse redboot partitions */
   	num_parts = parse_redboot_partitions(mtd, &spidata->parsed_parts);

#ifdef SPIFLASH_DEBUG
   	printk (KERN_DEBUG "Found %d redboot partitions\n", num_parts);
#endif

	if (num_parts) {
   		result = add_mtd_partitions(mtd, spidata->parsed_parts, num_parts);
		/* Find root partition */
		mtd_parts = spidata->parsed_parts;
		for (i=0; i < num_parts; i++) {
			if (!strcmp(mtd_parts[i].name, ROOTFS_NAME)) {
				/* Create root device */
        			ROOT_DEV = MKDEV(MTD_BLOCK_MAJOR, i);
				break;
			}
		}
	} else {
#ifdef SPIFLASH_DEBUG
   	printk (KERN_DEBUG "Did not find any redboot partitions\n");
#endif
		kfree(mtd);
		kfree(spidata);
       		return (-ENXIO);
	}
#endif

	spidata->mtd = mtd;

   	return (result);
}

void __exit 
spiflash_exit (void)
{
    	if (spidata && spidata->parsed_parts) {
        	del_mtd_partitions (spidata->mtd);
		kfree(spidata->mtd);
		kfree(spidata);
    	}
}

module_init (spiflash_init);
module_exit (spiflash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Atheros Communications Inc");
MODULE_DESCRIPTION("MTD driver for SPI Flash on Atheros SOC");

