/*
 * pxa_controller.h - MMC/SD/SDIO Controller driver header file
 *
 * Copyright (C) 2006 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *(C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 *
 */

#ifndef __PXA_CONTROLLER_H__
#define __PXA_CONTROLLER_H__

/* #include "common_protocol.h" */

#include <linux/mmc/mss_core.h>

#undef MMC_STRPCL
#undef MMC_STAT
#undef MMC_CLKRT
#undef MMC_SPI
#undef MMC_CMDAT
#undef MMC_RESTO
#undef MMC_RDTO
#undef MMC_BLKLEN
#undef MMC_NOB
#undef MMC_PRTBUF
#undef MMC_I_MASK
#undef END_CMD_RES
#undef MMC_I_REG
#undef MMC_CMD
#undef MMC_ARGH
#undef MMC_ARGL
#undef MMC_RES
#undef MMC_RXFIFO
#undef MMC_TXFIFO
#undef MMC_RDWAIT
#undef MMC_BLKS_REM

/* MMC_STRPCL */
#define MMC_STRPCL		0x0000
#define MMC_STRPCL_STOP_CLOCK    ( 1u << 0)
#define MMC_STRPCL_START_CLOCK   ( 1u << 1)   

/* MMC_STAT */
#define MMC_STAT			0x0004
#define MMC_STAT_READ_TIMEOUT       ( 1u << 0 )   
#define MMC_STAT_RESP_TIMEOUT       ( 1u << 1  )  
#define MMC_STAT_WRITE_CRC          ( 1u << 2 )
#define MMC_STAT_READ_CRC           ( 1u << 3 )   
#define MMC_STAT_SPI_ERR_TOKEN      ( 1u << 4 )   
#define MMC_STAT_RESP_CRC           ( 1u << 5  )
#define MMC_STAT_CLOCK_ON           ( 1u << 8  )
#define MMC_STAT_FLASH_ERR          ( 1u << 9  )
#define MMC_STAT_SPI_WRITE_ERR      ( 1u << 10 )
#define MMC_STAT_TRAN_DONE          ( 1u << 11 )
#define MMC_STAT_PRG_DONE           ( 1u << 12 )
#define MMC_STAT_END_CMD            ( 1u << 13 )
#define MMC_STAT_SDIO_RDSTALLED     ( 1u << 14 )
#define MMC_STAT_SDIO_INT           ( 1u << 15 )
#define MMC_STAT_SDIO_SUSPENDACK    ( 1u << 16 )

/* MMC_CLKRT */
#define MMC_CLKRT		0x0008		
#define CLKRT_26MHZ		(0x0007UL)
#define CLKRT_19_5MHZ		(0x0000UL)
#define CLKRT_9_75MHZ		(0x0001UL)
#define CLKRT_4_88MHZ		(0x0002UL)
#define CLKRT_2_44MHZ		(0x0003UL)
#define CLKRT_1_22MHZ		(0x0004UL)
#define CLKRT_0_609MHZ		(0x0005UL)
#define CLKRT_0_304MHZ		(0x0006UL)

/*MMC_SPI */
#define MMC_SPI			0x000c

/*MMC_CMDAT*/
#define MMC_CMDAT		0x0010
#define MMC_CMDAT_RESP_FMT_NONE	(0x0)
#define MMC_CMDAT_RESP_FMT_R1	(0x1)
#define MMC_CMDAT_RESP_FMT_R2	(0x2)
#define MMC_CMDAT_RESP_FMT_R3	(0x3)
#define MMC_CMDAT_DATA          (1u << 2)
#define MMC_CMDAT_WRITE         (1u << 3)
#define MMC_CMDAT_STREAM        (1u << 4)
#define MMC_CMDAT_BUSY          (1u << 5)
#define MMC_CMDAT_INIT          (1u << 6)
#define MMC_CMDAT_DMA           (1u << 7)
#define MMC_CMDAT_4DAT          (1u << 8)
#define MMC_CMDAT_STOP          (1u << 10)
#define MMC_CMDAT_SDIO_INT      (1u << 11)
#define MMC_CMDAT_SDIO_SUSPEND  (1u << 12)
#define MMC_CMDAT_SDIO_RESUME   (1u << 13)

/* MMC_RESTO */
#define MMC_RESTO		0x0014
#define RES_TIMEOUT_MAX		(0x007fUL) 

/* MMC_RDTO */
#define MMC_RDTO		0x0018
#define RD_TIMEOUT_MAX		(0xffffUL)

/* MMC_BLKLEN */
#define MMC_BLKLEN		0x001c

/* MMC_NUMBLK */
#define MMC_NUMBLK		0x0020

/*MMC_PRTBUF*/
#define MMC_PRTBUF		0x0024
#define BUF_PART_FULL		(1 << 0)

/*MMC_I_MASK   MMC_I_REG*/
#define MMC_I_MASK		0x0028
#define MMC_I_MASK_DATA_TRAN_DONE     (1u << 0) 
#define MMC_I_MASK_PRG_DONE           (1u << 1)
#define MMC_I_MASK_END_CMD_RES        (1u << 2) 
#define MMC_I_MASK_STOP_CMD           (1u << 3)
#define MMC_I_MASK_CLK_OFF            (1u << 4)
#define MMC_I_MASK_RXFIFO_RD_REQ      (1u << 5)
#define MMC_I_MASK_TXFIFO_WR_REQ      (1u << 6)
#define MMC_I_MASK_TINT               (1u << 7)
#define MMC_I_MASK_DAT_ERR            (1u << 8) 
#define MMC_I_MASK_RES_ERR            (1u << 9)
#define MMC_I_MASK_SDIO_RD_STALLED    (1u << 10)
#define MMC_I_MASK_SDIO_INT           (1u << 11)
#define MMC_I_MASK_SDIO_SUSPEND_ACK   (1u << 12)
#define MMC_MASK_ALL			(~0)

#define MMC_I_REG		0x002c
#define MMC_I_REG_DATA_TRAN_DONE      ( 1u << 0  )
#define MMC_I_REG_PRG_DONE            ( 1u << 1  )
#define MMC_I_REG_END_CMD_RES         ( 1u << 2  )
#define MMC_I_REG_STOP_CMD            ( 1u << 3  )
#define MMC_I_REG_CLK_OFF             ( 1u << 4  )
#define MMC_I_REG_RXFIFO_RD_REQ       ( 1u << 5  )
#define MMC_I_REG_TXFIFO_WR_REQ       ( 1u << 6  )
#define MMC_I_REG_TINT                ( 1u << 7  )
#define MMC_I_REG_DAT_ERR             ( 1u << 8  )
#define MMC_I_REG_RES_ERR             ( 1u << 9  )
#define MMC_I_REG_SDIO_RD_STALLED     ( 1u << 10 )
#define MMC_I_REG_SDIO_INT            ( 1u << 11 )
#define MMC_I_REG_SDIO_SUSPEND_ACK    ( 1u << 12 )

/* MMC_CMD */
#define MMC_CMD			0x0030

/* MMC_ARGH */
#define MMC_ARGH		0x0034

/* MMC_ARGL */
#define MMC_ARGL		0x0038

/* MMC_RES */
#define MMC_RES			0x003c

/* MMC_RXFIFO & MMC_TXFIFO */
#define MMC_RXFIFO		0x0040
#define MMC_TXFIFO		0x0044

/* MMC_RDWAIT */
#define MMC_RDWAIT		0x0048

/* MMC_BLKS_REM */
#define MMC_BLKS_REM		0x004c

#define MMC_DMA_BUF_SIZE	(MSS_SECTOR_SIZE * 256)

#define PXA_MSS_MAX_RESPONSE_SIZE	16

/* card detect irq */
#ifdef CONFIG_MACH_ZYLONITE
#if defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310)
#define MMC_CD0		(GPIO_EXT_TO_IRQ(128))
#define MMC_CD1		(GPIO_EXT_TO_IRQ(129))
#define MMC_CD3		(GPIO_EXT_TO_IRQ(158))
#else
#define MMC_CD0		IRQ_GPIO1
#define MMC_CD1		IRQ_GPIO(4)
#endif

#elif	defined (CONFIG_MACH_LITTLETON)
#define	MMC_CD0		IRQ_GPIO(15)
#else
#error	"Please select correct platform for build"
#endif

/* the controllers */
enum {
	PXA_MMC_1 = 0,
	PXA_MMC_2 = 1,
#ifdef CONFIG_CPU_PXA310
	PXA_MMC_3 = 2,
#endif
	PXA_MMC_MAX,
};

/* platform dependent structure for controller */
struct pxa_mss_host { 
	struct mss_host		*host;	/* point to upper controller */
	struct platform_device	*pdev;	/* point to contained platform device */
	
	int			irq;	/* IRQ for mmc controller */
	int			dat1_gpio_irq;/* DAT1/INT by GPIO edge detect */
	struct mss_ll_request	*llreq;
	struct mss_cmd		*cmd;
	struct mss_data		*data;
	struct mss_slot		*active_slot;
		
	u32			base;		/* register base */
	u32			phybase;
	u32			cken;
	u32			drcmrtx;
	u32			drcmrrx;
	
	int			dma;		/* DMA channel number */
	dma_addr_t		sg_dma;
	struct pxa_dma_desc	*sg_cpu;
	unsigned int		dma_len;
	unsigned int		sg_idx;
	unsigned int		dma_run;
#ifdef CONFIG_DVFM
	struct pxa3xx_fv_notifier dvfm_notifier;
#endif
	char			name[16];
	char			name2[16];
	struct workqueue_struct	*work_queue;		
	struct workqueue_struct	*sdio_work_queue;		
};


#endif
