/*
 * pxa3xx_controller.c - MMC/SD/SDIO Controller driver
 *
 * Copyright (C) 2006 Intel Corporation
 * Copyright (C) 2006 Marvell International Ltd  
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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/types.h>
#include <asm/dma-mapping.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>
#include <asm/arch/pxa3xx_pmic.h>
#include <asm/arch/arava.h>

#ifdef	CONFIG_MACH_ZYLONITE
#include <asm/arch/zylonite.h>
#elif	defined (CONFIG_MACH_LITTLETON)
#include <asm/arch/littleton.h>
#else
#error	"Please select correct Platform for build"
#endif
 
#include "pxa3xx_controller.h"

#ifdef CONFIG_PXA_MWB_12
extern void pxa3xx_enable_pxa_mwb_wifi(void);
#endif
 
#define PXA_SLOT_SCAN	1
#define PXA_SLOT_EXIT	2

struct pxa_slot {
	struct mss_slot *slot;
	u32 cd_irq;
	u32 cd_gpio;
	u32 wp_gpio;
	struct work_struct	sdio_int;
	struct delayed_work	card_change;
};

#if defined(CONFIG_CPU_PXA320) || defined(CONFIG_CPU_PXA300)
static u32 PXA_HOST_IRQ[PXA_MMC_MAX] = {IRQ_MMC, IRQ_MMC2};
static u32 PXA_HOST_DRCMRTX[PXA_MMC_MAX] 
			= {(u32)&(DRCMRTXMMC), (u32)&(DRCMRTXMMC2)};
static u32 PXA_HOST_DRCMRRX[PXA_MMC_MAX]
			= {(u32)&(DRCMRRXMMC), (u32)&(DRCMRRXMMC2)};
#ifdef CONFIG_MMC1_SLOT1
static u32 PXA_HOST_SLOTS[PXA_MMC_MAX] = {2, 1};
#else
static u32 PXA_HOST_SLOTS[PXA_MMC_MAX] = {1, 1};
#endif
static u32 PXA_HOST_BASE[PXA_MMC_MAX] = {(u32)&(__REG(0x41100000)), 
	(u32)&(__REG_2(0x42000000))};
static u32 PXA_HOST_PHYBASE[PXA_MMC_MAX] = {0x41100000, 0x42000000};
static u32 PXA_HOST_CKEN[PXA_MMC_MAX] = {CKEN_MMC1, CKEN_MMC2};

#elif defined(CONFIG_CPU_PXA310)
static u32 PXA_HOST_IRQ[PXA_MMC_MAX] = {IRQ_MMC, IRQ_MMC2, IRQ_MMC3};
static u32 PXA_HOST_DRCMRTX[PXA_MMC_MAX] 
			= {(u32)&(DRCMRTXMMC), (u32)&(DRCMRTXMMC2), 
				(u32)&(DRCMRTXMMC3)};
static u32 PXA_HOST_DRCMRRX[PXA_MMC_MAX]
			= {(u32)&(DRCMRRXMMC), (u32)&(DRCMRRXMMC2), 
				(u32)&(DRCMRRXMMC3)};
#ifdef CONFIG_MMC1_SLOT1
static u32 PXA_HOST_SLOTS[PXA_MMC_MAX] = {2, 1, 1};
#else
static u32 PXA_HOST_SLOTS[PXA_MMC_MAX] = {1, 1, 1};
#endif
static u32 PXA_HOST_BASE[PXA_MMC_MAX] = {(u32)&(__REG(0x41100000)),
	(u32)&(__REG_2(0x42000000)), (u32)&(__REG_4(0x42500000))};
static u32 PXA_HOST_PHYBASE[PXA_MMC_MAX] = {0x41100000, 0x42000000, 0x42500000};
static u32 PXA_HOST_CKEN[PXA_MMC_MAX] = {CKEN_MMC1, CKEN_MMC2, CKEN_MMC3};
#endif

/*  enable controller INT according to mask */
static void pxa_host_enable_int(struct pxa_mss_host *pxa_host, unsigned int mask)
{
	u32 i_reg = 0;
	unsigned long flags;

	local_irq_save(flags);
	i_reg = readl(pxa_host->base + MMC_I_MASK);
	i_reg &= ~mask;
	writel(i_reg, pxa_host->base + MMC_I_MASK);
	i_reg = readl(pxa_host->base + MMC_I_MASK);
	local_irq_restore(flags);
}

/*
 *  disable controller INT according to mask
 */
static void pxa_host_disable_int(struct pxa_mss_host *pxa_host, unsigned int mask)
{
	u32 i_reg;
	unsigned long flags;

	local_irq_save(flags);
	i_reg = readl(pxa_host->base + MMC_I_MASK);
	i_reg |= mask;
	writel(i_reg, pxa_host->base + MMC_I_MASK);
	i_reg = readl(pxa_host->base + MMC_I_MASK);
	local_irq_restore(flags);
}

static void pxa_host_start_busclock(struct pxa_mss_host *pxa_host)
{
	u32 retries = 0xff;
	
	writel(MMC_STRPCL_START_CLOCK, pxa_host->base + MMC_STRPCL);
	while (retries--) {
		if (readl(pxa_host->base + MMC_STAT) & MMC_STAT_CLOCK_ON);
			break;
		udelay(1);
	}
}

static void pxa_host_stop_busclock(struct pxa_mss_host *pxa_host)
{
	u32 retries = 0xff;
	
	writel(MMC_STRPCL_STOP_CLOCK, pxa_host->base + MMC_STRPCL);
	while (retries--) {
		if (readl(pxa_host->base + MMC_I_REG) & MMC_I_REG_CLK_OFF)
			break;
		udelay(1);
	}
}

/*
 *  setup DMA controller for data transfer 
 */
static void pxa_host_setup_data(struct pxa_mss_host *pxa_host,
		struct mss_data *data)
{
	u32 dcmd = 0;
	int i;
	struct mss_host *host = pxa_host->host;

	writel(data->blocks, pxa_host->base + MMC_NUMBLK);
	writel(data->blksz, pxa_host->base + MMC_BLKLEN);
	if (data->flags & MSS_DATA_READ) { /*read*/
		dcmd = DCMD_INCTRGADDR | DCMD_FLOWSRC;
		writel(0x0, pxa_host->drcmrtx);
		writel(pxa_host->dma | DRCMR_MAPVLD, pxa_host->drcmrrx);
	} else if (data->flags & MSS_DATA_WRITE ) { /*write*/
		dcmd = DCMD_INCSRCADDR | DCMD_FLOWTRG;
		writel(0x0, pxa_host->drcmrrx);
		writel(pxa_host->dma | DRCMR_MAPVLD, pxa_host->drcmrtx);
	} else {
		BUG();
	}

	dcmd |= DCMD_BURST32 | DCMD_WIDTH1;

	dbg("read to set up dma., sg_len:%d", data->sg_len);
	pxa_host->sg_idx = 0;
	pxa_host->dma_len = dma_map_sg(host->dev, data->sg, data->sg_len, 
			(data->flags & MSS_DATA_READ) ? 
			DMA_FROM_DEVICE : DMA_TO_DEVICE);

	for (i = 0; i < pxa_host->dma_len; i++) {
		data->bytes_xfered += sg_dma_len(&data->sg[i]);
		if (data->flags & MSS_DATA_READ) {
			pxa_host->sg_cpu[i].dsadr = 
				pxa_host->phybase + MMC_RXFIFO;
			pxa_host->sg_cpu[i].dtadr = 
				sg_dma_address(&data->sg[i]);
		}
		else {
			pxa_host->sg_cpu[i].dsadr = 
				sg_dma_address(&data->sg[i]);
			pxa_host->sg_cpu[i].dtadr = 
				pxa_host->phybase + MMC_TXFIFO;
		}
		pxa_host->sg_cpu[i].dcmd = dcmd | sg_dma_len(&data->sg[i]);
		pxa_host->sg_cpu[i].ddadr = pxa_host->sg_dma + (i + 1) *
					sizeof(struct pxa_dma_desc);
		dbg("sg:%d, dsadr:0x%x, dtadr:0x%x, dcmd:0x%x, ddadr:0x%x\n", i,
			pxa_host->sg_cpu[i].dsadr, pxa_host->sg_cpu[i].dtadr, 
			pxa_host->sg_cpu[i].dcmd, pxa_host->sg_cpu[i].ddadr);
	}
	pxa_host->sg_cpu[pxa_host->dma_len - 1].ddadr = DDADR_STOP;
	pxa_host->sg_cpu[pxa_host->dma_len - 1].dcmd |= DCMD_ENDIRQEN;
	wmb();
}

/*
 *  read response of MMC/SD/SDIO controller.
 */
static void pxa_host_get_response(struct pxa_mss_host *pxa_host,
		struct mss_cmd *cmd)
{

	int i;
	u32 mmc_res;

	for (i = 0; i < PXA_MSS_MAX_RESPONSE_SIZE; i = i + 2) {
		mmc_res = readl(pxa_host->base + MMC_RES);
		if (i < MSS_MAX_RESPONSE_SIZE) {
			cmd->response[i] = mmc_res >> 8;
			cmd->response[i + 1] = mmc_res & 0xff;
		}
	}
#ifdef CONFIG_MMC_DEBUG
	printk(KERN_DEBUG "Response for CMD 0x%x, RES:", cmd->opcode);
	for (i = 0; i < PXA_MSS_MAX_RESPONSE_SIZE; i++) 
		printk("0x%x,", cmd->response[i]);
	printk("\n");
#endif
}

/*
 *  set io_request result according to error status in MMC/SD/SDIO controller 
 *  status reg.
 */
static void pxa_host_set_error(struct mss_cmd *cmd, u32 stat)
{
	if (stat & MMC_STAT_RESP_TIMEOUT ) 
		cmd->error = MSS_ERROR_TIMEOUT;
	else if (stat & MMC_STAT_RESP_CRC) 
		cmd->error = MSS_ERROR_CRC;
	else if (stat & MMC_STAT_FLASH_ERR)
		cmd->error = MSS_ERROR_FLASH;
	else if (stat & MMC_STAT_READ_TIMEOUT)
		cmd->error = MSS_ERROR_TIMEOUT;
	else if (stat & (MMC_STAT_READ_CRC | MMC_STAT_WRITE_CRC))
		cmd->error = MSS_ERROR_CRC;
	dbg("Error %d for command: 0x%x\n", cmd->error, cmd->opcode);
}

static inline int get_slot_cd_irq(struct mss_slot *slot) 
{
	return ((struct pxa_slot *)slot->private)->cd_irq;
}

static inline void set_slot_cd_irq(struct mss_slot *slot, int cd_irq)
{
	((struct pxa_slot *)slot->private)->cd_irq = cd_irq;
}

static inline int get_slot_cd_gpio(struct mss_slot *slot) 
{
	return ((struct pxa_slot *)slot->private)->cd_gpio;
}

static inline void set_slot_cd_gpio(struct mss_slot *slot, int cd_gpio)
{
	((struct pxa_slot *)slot->private)->cd_gpio = cd_gpio;
}

static inline int get_slot_wp_gpio(struct mss_slot *slot) 
{
	return ((struct pxa_slot *)slot->private)->wp_gpio;
}

static inline void set_slot_wp_gpio(struct mss_slot *slot, int wp_gpio)
{
	((struct pxa_slot *)slot->private)->wp_gpio = wp_gpio;
}

#ifdef	CONFIG_MACH_ZYLONITE
static void pxa_mss_slot_select(struct mss_slot *slot)
{
	if (slot->host->id == PXA_MMC_1) {
		if (slot->id == 0) {
			pxa3xx_mfp_set_afds(MFP_MMC_CMD_0, MFP_AF4, MFP_DS03X);
			/* set to GPIO output high for CMD_1 pin */
			pxa3xx_mfp_set_afds(MFP_MMC_CMD_1, MFP_AF0, MFP_DS03X);
			pxa3xx_gpio_set_direction(MFP_MMC_CMD_1, GPIO_DIR_OUT);
			pxa3xx_gpio_set_level(MFP_MMC_CMD_1, GPIO_LEVEL_HIGH);
		} else if (slot->id == 1) {
			pxa3xx_mfp_set_afds(MFP_MMC_CMD_1, MFP_MMC_CMD_1_AF, 
					MFP_DS03X);
			/* set to GPIO output high for CMD_0 pin */
			pxa3xx_mfp_set_afds(MFP_MMC_CMD_0, MFP_AF0, MFP_DS03X);

			pxa3xx_gpio_set_direction(MFP_MMC_CMD_0, GPIO_DIR_OUT);
			pxa3xx_gpio_set_level(MFP_MMC_CMD_0, GPIO_LEVEL_HIGH);
		} else
			BUG();
	} else if(slot->host->id == PXA_MMC_2) {
		pxa3xx_mfp_set_afds(MFP_MMC2_CMD, MFP_AF4, MFP_DS08X);
	}
#if defined(CONFIG_CPU_PXA310)
	else if(slot->host->id == PXA_MMC_3) {
		pxa3xx_mfp_set_afds(MFP_MMC3_CMD, MFP_MMC3_CMD_AF, MFP_DS03X);
	}
#endif
	else
		BUG();
}

#elif	defined (CONFIG_MACH_LITTLETON)
static void pxa_mss_slot_select(struct mss_slot *slot)
{
	if (slot->host->id == PXA_MMC_1) {
		if (slot->id == 0) {
			pxa3xx_mfp_set_afds(MFP_MMC_CMD_0, MFP_AF4, MFP_DS03X);
		} else if (slot->id == 1) {
			/* set to GPIO output high for CMD_0 pin */ 
			pxa3xx_mfp_set_afds(MFP_MMC_CMD_0, MFP_AF0, MFP_DS03X);

			pxa3xx_gpio_set_direction(MFP_MMC_CMD_0, GPIO_DIR_OUT);
			pxa3xx_gpio_set_level(MFP_MMC_CMD_0, GPIO_LEVEL_HIGH);
		} else
			BUG();

	} else if(slot->host->id == PXA_MMC_2) {
		pxa3xx_mfp_set_afds(MFP_MMC2_CMD, MFP_AF4, MFP_DS08X);
	}
#if defined(CONFIG_CPU_PXA310)
	else if(slot->host->id == PXA_MMC_3) {
		pxa3xx_mfp_set_afds(MFP_MMC3_CMD, MFP_MMC3_CMD_AF, MFP_DS03X);
	}
#endif
	else
		BUG();
}

#else
#error	"Please select correct platform for build"
#endif

/*
 *  return value: 0 -- not write-protected, 1 -- write-protected
 */
static int pxa_mss_slot_is_wp(struct mss_slot *slot)
{
	u32 wp_gpio = get_slot_wp_gpio(slot);
	u32 level = GPIO_LEVEL_LOW;

	if (wp_gpio)
		level = pxa3xx_gpio_get_level(wp_gpio);
	
	dbg("host%d, slot%d wp is %s", slot->host->id, slot->id, 
			(level == GPIO_LEVEL_HIGH) ? "TRUE":"FALSE");
	return (level == GPIO_LEVEL_HIGH);
}

/*
 *  return value: 0 -- inserted, 1 -- empty.
 */
static int pxa_mss_slot_is_empty(struct mss_slot *slot)
{
	u32 cd_gpio = get_slot_cd_gpio(slot);

	if (cd_gpio) {
		int empty = (pxa3xx_gpio_get_level(cd_gpio) == GPIO_LEVEL_HIGH);
		return empty;	
	}
	
	return 0;
}
 
static int free_sdio_dat1_irq (struct pxa_mss_host *pxa_host)
{
	int irq = IRQ_GPIO(MFP2GPIO(MFP_MMC_DAT1));

	free_irq(irq, pxa_host);
	pxa_host->dat1_gpio_irq = 0;
	pxa3xx_mfp_set_afds(MFP_MMC_DAT1, MFP_MMC_DAT1_AF, MFP_DS03X);
	return 0;
}

static irqreturn_t pxa_sdio_dat1_irq(int irq, void *devid)
{
	struct pxa_mss_host *pxa_host;

	pxa_host = (struct pxa_mss_host *)devid;

	dbg("%s\n", __func__);
	free_sdio_dat1_irq(pxa_host);
	pxa_host_start_busclock(pxa_host);
	return IRQ_HANDLED;
}

static int set_sdio_dat1_irq (struct pxa_mss_host *pxa_host)
{ 
	int irq = IRQ_GPIO(MFP2GPIO(MFP_MMC_DAT1));
	int ret;

	dbg("%s\n", __func__);
	if (pxa_host->dat1_gpio_irq) {
		printk(KERN_ERR "re-enterance of %s\n", __func__);
		return -EFAULT;
	}
		
	pxa3xx_mfp_set_afds(MFP_MMC_DAT1, 0, 0);
	pxa3xx_gpio_set_direction(MFP_MMC_DAT1, GPIO_DIR_IN);	
	set_irq_type(irq, IRQT_BOTHEDGE);

	ret = request_irq(irq, pxa_sdio_dat1_irq, 0, 
		"SDIO DAT1/INT", (void *)pxa_host);

	if (ret)
		printk(KERN_ERR "SDIO: request irq %d fails, ret: %d\n", 
			irq, ret);
	else
		pxa_host->dat1_gpio_irq = irq;
	return ret;
}

static int pxa3xx_mmc_get_clockrate(struct mss_host *host, int clock)
{
	int ret;

	switch (clock) {
#if	defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310) 
	case 26000000:
		ret = CLKRT_26MHZ;
		break;
#endif
	case 19500000:
		ret = CLKRT_19_5MHZ;
		break;
	case  9750000:
		ret = CLKRT_9_75MHZ;
		break;
	case  4875000:
		ret = CLKRT_4_88MHZ;
		break;
	case  2437500:
		ret = CLKRT_2_44MHZ;
		break;
	case  1218750:
		ret = CLKRT_1_22MHZ;
		break;
	case   609375:
		ret = CLKRT_0_609MHZ;
		break;
	case   304000:
		ret = CLKRT_0_304MHZ;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void pxa_mss_set_ios(struct mss_host *host, struct mss_ios *ios)
{
	unsigned int clockrate;
	struct pxa_mss_host *pxa_host;
       
	pxa_host = host->private;
	
	dbg("clock now %d, to set %d, CLKRT:%x", host->ios.clock, ios->clock, 
			readl(pxa_host->base + MMC_CLKRT));
	if (ios->clock != host->ios.clock) {
#if	defined(CONFIG_CPU_PXA300) || defined(CONFIG_CPU_PXA310) 
		if (ios->clock >= 26000000) {
			host->ios.clock = 26000000;
		}
		else if (ios->clock >= 19500000)
#else
		if (ios->clock >= 19500000)
#endif
		{
			host->ios.clock = 19500000;
		}
		else if (ios->clock >= 9750000) {
			host->ios.clock = 9750000;
		}
		else if (ios->clock >= 4875000) {
			host->ios.clock = 4875000;
		}
		else if (ios->clock >= 2437500) {
			host->ios.clock = 2437500;
		}	
		else if (ios->clock >= 1218750) {
			host->ios.clock = 1218750;
		}
		else if (ios->clock >= 609375){
			host->ios.clock = 609375;
		}	
		else if ((ios->clock != MSS_CLOCK_START) &&
			 (ios->clock != MSS_CLOCK_STOP)) {
			host->ios.clock = 304000;
		}

		clockrate = pxa3xx_mmc_get_clockrate(host, host->ios.clock);
		pxa_host_stop_busclock(pxa_host);

		if (ios->clock != MSS_CLOCK_STOP) {
			if (pxa_host->dat1_gpio_irq)
				free_sdio_dat1_irq(pxa_host);
			writel(clockrate, pxa_host->base + MMC_CLKRT);
			pxa_host_start_busclock(pxa_host);
		} else if (host->active_card->card_type == MSS_SDIO_CARD) {
			/* enable the dat1 interrupt by setting to GPIO edge
			 * detion interrupt to detect interrupts, only 
			 * functional when MMC_CLK is stopped */
			if (ios->dat1_gpio_irq_en)
				set_sdio_dat1_irq(pxa_host);
		}
	}
	host->ios.bus_width = ios->bus_width;
	if (host->ios.bus_width > host->bus_width)
		host->ios.bus_width = host->bus_width;

}

#ifdef CONFIG_DVFM
static int pxa3xx_mmc_dvfm_notifier(unsigned cmd, void *client_data, void *info)
{
	struct pxa_mss_host *pxa_host = (struct pxa_mss_host *)client_data;

	switch (cmd) {
	case FV_NOTIFIER_QUERY_SET :
		if (pxa_host->dma_run)
			return -1;
		break;

	case FV_NOTIFIER_PRE_SET :
		break;

	case FV_NOTIFIER_POST_SET :
		break;
	}

	return 0;
}

#endif

/*
 *  send command by setting CMDAT reg of MMC/SD/SDIO controller.
 *  Note: to hide info about SD/SDIO.
 */
static void pxa_mss_handle_request(struct mss_host *host, 
		struct mss_ll_request *llreq)
{
	struct pxa_mss_host *pxa_host = host->private;
	struct mss_cmd *cmd;
	struct mss_card *card = host->active_card;
	u32 cmd_dat = 0;

	dbg("pxammc %d acitive slot %x\n", host->id + 1,
		(pxa_host->active_slot) ? pxa_host->active_slot->id : -1);
	if (pxa_host->active_slot != card->slot) {
		dbg("pxammc %d change acitive slot to %d\n", host->id + 1, 
				card->slot->id);
		pxa_mss_slot_select(card->slot);
		pxa_host->active_slot = card->slot;
	}
	pxa_host->cmd = llreq->cmd;
	pxa_host->data = llreq->data;
	pxa_host->llreq = llreq;
	cmd = pxa_host->cmd;

	dbg("Read to send CMD :0x%x, cmd flag 0x%x", cmd->opcode, cmd->flags);
	if (cmd->flags & MSS_CMD_INIT)
		cmd_dat |= MMC_CMDAT_INIT;
	if (cmd->flags & MSS_CMD_STOP)
		cmd_dat |= MMC_CMDAT_STOP;
	if (cmd->flags & MSS_CMD_SDIO_EN || host->sdio_int)
		cmd_dat |= MMC_CMDAT_SDIO_INT;
	if (llreq->data) {
		dbg("data flags 0x%x", llreq->data->flags);
		if (llreq->data->flags & MSS_DATA_WRITE)
			cmd_dat |= MMC_CMDAT_WRITE;
		pxa_host_setup_data(pxa_host, llreq->data);
		cmd_dat |= (MMC_CMDAT_DATA | MMC_CMDAT_DMA);
		cmd_dat &= ~MMC_CMDAT_BUSY;
		dbg("data nob %d, block_len %d", llreq->data->blocks, 
				llreq->data->blksz);
		if (llreq->data->flags & MSS_DATA_STREAM)
			cmd_dat |= MMC_CMDAT_STREAM;
	} else {
		writel(0, pxa_host->base + MMC_NUMBLK);
		writel(0, pxa_host->base + MMC_BLKLEN);
	}

	dbg("cmd rtype %d", cmd->rtype);
	switch (cmd->rtype) {
		case MSS_RESPONSE_NONE:
			cmd_dat |= MMC_CMDAT_RESP_FMT_NONE;
			break;
		case MSS_RESPONSE_R1B:
			cmd_dat |= MMC_CMDAT_BUSY;
			/* Fall through */
		case MSS_RESPONSE_R1:
		case MSS_RESPONSE_R4:
		case MSS_RESPONSE_R5:
		case MSS_RESPONSE_R6:
		case MSS_RESPONSE_R7:
			cmd_dat |= MMC_CMDAT_RESP_FMT_R1;
			break;
		case MSS_RESPONSE_R2_CID:
		case MSS_RESPONSE_R2_CSD:
			cmd_dat |= MMC_CMDAT_RESP_FMT_R2;
			break;
		case MSS_RESPONSE_R3:
			cmd_dat |= MMC_CMDAT_RESP_FMT_R3;
			break;
	}

	dbg("card bus width %d", card->bus_width);
	if (card->bus_width == MSS_BUSWIDTH_4BIT)
		cmd_dat |= MMC_CMDAT_4DAT;
	
	writel(cmd->opcode, pxa_host->base + MMC_CMD);
	writel(cmd->arg >> 16, pxa_host->base + MMC_ARGH);
	writel(cmd->arg & 0xffff, pxa_host->base + MMC_ARGL);
	writel(cmd_dat, pxa_host->base + MMC_CMDAT);
	
	dbg("Read out MMC_CMD 0x%x",readl(pxa_host->base+MMC_CMD));
	dbg("Read out MMC_ARGH 0x%x",readl(pxa_host->base+MMC_ARGH));
	dbg("Read out MMC_ARGL 0x%x",readl(pxa_host->base+MMC_ARGL));
	dbg("Read out MMC_CMDAT 0x%x",readl(pxa_host->base+MMC_CMDAT));
	dbg("Read out MMC_CLKRT 0x%x",readl(pxa_host->base+MMC_CLKRT));
	dbg("Read out MMC_NUMBLK 0x%x",readl(pxa_host->base+MMC_NUMBLK));
	dbg("Read out MMC_BLKSZ 0x%x",readl(pxa_host->base+MMC_BLKLEN));
	if (llreq->data && llreq->data->sg_len) {
		pxa_host->dma_run = 1;
		DDADR(pxa_host->dma) = pxa_host->sg_dma;
		/* start DMA */
		DCSR(pxa_host->dma) |= DCSR_RUN; 
		dbg("enable dma %d, DDADR: 0x%x, DCSR: 0x%x", pxa_host->dma, 
				DDADR(pxa_host->dma), DCSR(pxa_host->dma));
	}
	pxa_host_enable_int(pxa_host, MMC_I_MASK_END_CMD_RES | MMC_I_MASK_RES_ERR);	
}

static void pxa_mss_enable_sdio_int(struct mss_host *host, int enable)
{
	unsigned long flags;
	struct pxa_mss_host *pxa_host = host->private;
	
	local_irq_save(flags);
	if (enable) {
		host->sdio_int = MSS_SDIO_INT_EN;
		pxa_host_enable_int(pxa_host, MMC_I_MASK_SDIO_INT);
	}
	else {
		host->sdio_int = MSS_SDIO_INT_DIS;
		pxa_host_disable_int(pxa_host, MMC_I_MASK_SDIO_INT);
	}
	local_irq_restore(flags);
}

/*
 * DMA irq handler. devid represent pxa_mss_controller_device for dma interrupt
 * is to pxa_controller
 */
static void pxa_host_dma_irq(int dma, void *devid)
{
	unsigned int dcsr;
	struct pxa_mss_host *pxa_host;
	struct mss_card *card;
	struct mss_cmd *cmd;
	struct mss_data *data;

	dbg("DMA IRQ");
	pxa_host = (struct pxa_mss_host *)devid;
	card = pxa_host->host->active_card;
	data = pxa_host->data;
	if (!card) {
		printk(KERN_ERR "Can not find card\n");
		BUG();
	}
	cmd = pxa_host->cmd;

	dcsr = DCSR(dma);
	DCSR(dma) &= ~DCSR_STOPIRQEN;	
	dbg("dma int dcsr:0x%x, DSADR:0x%x, DCMD:0x%x", dcsr, DSADR(dma),
		       	DCMD(dma));
	if (dcsr & DCSR_BUSERR) {
		/* dbg("DCSR_BUSEER"); */
		cmd->error = MSS_ERROR_DMA;
		data->bytes_xfered = 0;
		pxa_host_disable_int(pxa_host, MMC_I_MASK_DATA_TRAN_DONE | 
				MMC_I_MASK_DAT_ERR | MMC_I_MASK_TINT);
		pxa_host->llreq->done(pxa_host->llreq);
	}
	else if (dcsr & DCSR_STOPSTATE) {
		if (data && (data->flags & MSS_DATA_WRITE))
			writel(BUF_PART_FULL, pxa_host->base + MMC_PRTBUF); 
		if (data && (data->flags & MSS_DATA_WRITE) && 
				cmd->rtype == MSS_RESPONSE_R1B)
			pxa_host_enable_int(pxa_host, MMC_I_REG_PRG_DONE);
		/* dbg("sg index:%d, dma_len:%d", pxa_host->sg_idx, 
				pxa_host->dma_len); */
		else if (data && (data->flags & 
					(MSS_DATA_WRITE | MSS_DATA_READ))) {
			pxa_host_enable_int(pxa_host, MMC_I_MASK_DATA_TRAN_DONE
				       	| MMC_I_MASK_TINT);
		} else {
			printk(KERN_ERR "ERROR in SLOT_DMA_IRQ, neither WRITE"
				       " nor READ or sg_idx exceed dma_len");
			dump_stack();
		}
	}
	pxa_host->dma_run = 0;	
	return;	
}

/*
 *  slot INT routine for commands with R1B response type
 */
static int pxa_host_r1b_irq(struct pxa_mss_host *pxa_host, 
		u32  ireg, u32 stat, u32 mask)
{
	struct mss_host *host;
	struct mss_cmd *cmd;

	/* dbg("host%d, I_REG: 0x%x, STAT: 0x%x, MASK: 0x%x\n", 
	pxa_host->host->id, (u32)ireg, (u32)stat, (u32)mask); */
	host = pxa_host->host;
	cmd = pxa_host->cmd;
	
	if ((ireg & MMC_I_REG_END_CMD_RES) 
			&& (!(mask & MMC_I_MASK_END_CMD_RES)) ) {
		pxa_host_disable_int(pxa_host, 
				MMC_I_MASK_END_CMD_RES | MMC_I_MASK_RES_ERR);
		pxa_host_get_response(pxa_host, cmd);
		if (ireg & MMC_I_REG_RES_ERR) {
			pxa_host_set_error(cmd, stat);
			/* dbg("in RES_ERROR, prepare to cmd_complete");*/
			pxa_host->llreq->done(pxa_host->llreq);
			return 1;
		}		
		pxa_host_enable_int(pxa_host, MMC_I_MASK_PRG_DONE);
		return 1;
	}
	if ((ireg & MMC_I_REG_PRG_DONE) && (!(mask & MMC_I_MASK_PRG_DONE)) ) {
		pxa_host_disable_int(pxa_host, MMC_I_MASK_PRG_DONE);
		/* dbg("in PRG_DONE, prepare to cmd_complete\n"); */
		pxa_host->llreq->done(pxa_host->llreq); 
	}
	return 1;
}

/*
 *  slot INT routine for write commands with data buffer
 */
static int pxa_host_write_irq(struct pxa_mss_host *pxa_host,
		u32 ireg, u32 stat, u32 mask)
{
	struct mss_host *host;
	struct mss_cmd *cmd;

	host = pxa_host->host;
	cmd = pxa_host->cmd;
	
	if ((ireg & MMC_I_REG_END_CMD_RES) && \
		(!(mask & MMC_I_MASK_END_CMD_RES)) ) {
		pxa_host_disable_int(pxa_host, 
			MMC_I_MASK_END_CMD_RES | MMC_I_MASK_RES_ERR);
		if (ireg & MMC_I_REG_RES_ERR) {
			pxa_host_set_error(cmd, stat);
			pxa_host->llreq->done(pxa_host->llreq);
			return 1;
		}	
		pxa_host_get_response(pxa_host, cmd);
		return 1;
	}

	if ((ireg & MMC_I_REG_TINT) && (!(mask & MMC_I_MASK_TINT))) {
		stat = readl(pxa_host->base + MMC_STAT);
		pxa_host_disable_int(pxa_host, \
			MMC_I_MASK_DATA_TRAN_DONE | \
			MMC_I_MASK_DAT_ERR | \
			MMC_I_MASK_TINT);
		pxa_host_set_error(cmd, stat);
		/* dbg("DATA/TIMEOUT error.  I_REG: 0x%x, STAT: 0x%x, 
		 I_MASK: 0x%x. prepare to cmd_complete", MMC_I_REG, MMC_STAT, 
		 MMC_I_MASK); */
		pxa_host->llreq->done(pxa_host->llreq);
		return 1;
	}	

	/* wait for DMA interrupt, DATA WRITE done once DATA_TRAN_DONE 
	 * interrupt occurred 
	 */
	if ((ireg & MMC_I_REG_DATA_TRAN_DONE) 
			&& (!(mask & MMC_I_MASK_DATA_TRAN_DONE)) ) {
		/* Note: must not disable DATA_ERROR interrupt, 
		 * for it may be occured again!!! 
		 */
		pxa_host_disable_int(pxa_host, 
				MMC_I_MASK_DATA_TRAN_DONE |
				MMC_I_MASK_DAT_ERR |
				MMC_I_MASK_TINT);
		pxa_host_enable_int(pxa_host, MMC_I_MASK_PRG_DONE); 
		return 1;
	}	
	
	if ((ireg & MMC_I_REG_PRG_DONE) && (!(mask & MMC_I_MASK_PRG_DONE)) ) {
		/* pxa_host_get_response(pxa_host, cmd); */
		pxa_host_disable_int(pxa_host, MMC_I_MASK_PRG_DONE);
		/* dbg("in PRG_DONE, prepare to cmd_complete"); */
		pxa_host->llreq->done(pxa_host->llreq); 
	}
	return 1;
}

/**
 *  pxa_slot_read_irq
 *  @card: mss_card_device
 *  @ireg: value of MMCx_I_REG
 *  @stat: value of MMCx_STAT
 *  @mask: value of MMCx_I_MASK
 *
 *  slot INT routine for read commands with data buffer
 */
static int pxa_host_read_irq(struct pxa_mss_host *pxa_host,
		u32 ireg, u32 stat, u32 mask)
{
	struct mss_host *host;
	struct mss_cmd *cmd;

	host = pxa_host->host;
	cmd = pxa_host->cmd;

	if ((ireg & MMC_I_REG_END_CMD_RES) 
			&& (!(mask & MMC_I_MASK_END_CMD_RES))) {
		pxa_host_disable_int(pxa_host, 
				MMC_I_MASK_END_CMD_RES | MMC_I_MASK_RES_ERR);
		if (ireg & MMC_I_REG_RES_ERR) {
			pxa_host_set_error(cmd, stat);
			pxa_host_enable_int(pxa_host, MMC_I_MASK_TINT);
			return 1;
	  	}
	  	pxa_host_get_response(pxa_host, cmd);
		/* DATA_TRAN_DONE is not enabled now */
		pxa_host_enable_int(pxa_host, MMC_I_MASK_TINT);
		return 1;
	}

	if ((ireg & MMC_I_REG_TINT) && (!(mask & MMC_I_MASK_TINT))) {
		stat = readl(pxa_host->base + MMC_STAT);
		pxa_host_disable_int(pxa_host, \
			MMC_I_MASK_DATA_TRAN_DONE | \
			MMC_I_MASK_DAT_ERR | \
			MMC_I_MASK_TINT);
		pxa_host_set_error(cmd, stat);
		pxa_host->llreq->done(pxa_host->llreq);
		return 1;
	}

	/* wait for DMA interrupt, DATA READ done 
	 * once DATA_TRAN_DONE interrupt occurred 
	 */
	if ((ireg & MMC_I_REG_DATA_TRAN_DONE) && \
		(!(mask & MMC_I_MASK_DATA_TRAN_DONE))) {
		pxa_host_disable_int(pxa_host, \
			MMC_I_MASK_DATA_TRAN_DONE | \
			MMC_I_MASK_DAT_ERR | \
			MMC_I_MASK_TINT);

		/* check data error */
                if (ireg & MMC_I_MASK_DAT_ERR) {
                        stat = readl(pxa_host->base + MMC_STAT);
                        pxa_host_set_error(cmd, stat);
                }

		/* dbg("in DATA_TRAN_DONE, prepare to cmd_complete");*/
		pxa_host->llreq->done(pxa_host->llreq); 
		return 1;
	}
	
	return 1;
}

/**
 *  pxa_slot_other_irq
 *  @card: mss_card_device
 *  @ireg: value of MMCx_I_REG
 *  @stat: value of MMCx_STAT
 *  @mask: value of MMCx_I_MASK
 *
 *  slot INT routine for commands other than R1B, write and read commands
 */
static int pxa_host_other_irq(struct pxa_mss_host *pxa_host, 
		u32  ireg, u32 stat, u32 mask)
{
	struct mss_cmd *cmd = pxa_host->cmd;

	if ((ireg & MMC_I_REG_RES_ERR) && (!(mask & MMC_I_MASK_RES_ERR))){
		pxa_host_disable_int(pxa_host, 
			MMC_I_MASK_END_CMD_RES | \
			MMC_I_MASK_RES_ERR);
		pxa_host_get_response(pxa_host, cmd);
		pxa_host_set_error(cmd, stat);
		/* dbg("in RES_ERROR, prepare to cmd_complete\n"); */
		pxa_host->llreq->done(pxa_host->llreq);
		return 1;
	} else if((ireg & MMC_I_REG_END_CMD_RES) && \
			(!(mask & MMC_I_MASK_END_CMD_RES))) {
		pxa_host_disable_int(pxa_host, \
			MMC_I_MASK_END_CMD_RES | \
			MMC_I_MASK_RES_ERR);
		pxa_host_get_response(pxa_host, cmd);
		pxa_host->llreq->done(pxa_host->llreq);
	}
	
	return 1;
}

static void sdio_interrupt_handler(struct work_struct *work)
{
 	struct pxa_slot * pxa_slot; 
 	struct pxa_mss_host * pxa_host; 
 	struct mss_card * card;
	struct mss_driver *drv; 
	
	pxa_slot = container_of(work, struct pxa_slot, sdio_int);
	if (pxa_slot && pxa_slot->slot && pxa_slot->slot->host)
		pxa_host = pxa_slot->slot->host->private;
	else
		goto out;

	if (pxa_host && pxa_host->active_slot)
		card = pxa_host->active_slot->card;
	else
		goto out;
	
	if(pxa_host && pxa_host->active_slot && pxa_host->active_slot->card)
	{
		drv = container_of(pxa_host->active_slot->card->dev.driver,
                                struct mss_driver, driver);
	}

	if (drv->sdio_int_handler)
		drv->sdio_int_handler(card);
	else
		return;
	
out:
	dbg("null pointer occurred\n");
	return;
}

/**
 *  pxa_slot_irq
 *  @irq: IRQ number
 *  @devid: pointer to pxa_mss_controller_device
 *  @regs: interrupt context
 *
 *  response interrupt and sdio interrupt handler. 
 *  devid represent pxa_mss_controller_device because 
 *  interrupt is to pxa_controller
 */
static irqreturn_t pxa_host_irq(int irq, void *devid)
{
	struct pxa_mss_host *pxa_host;
	struct mss_cmd *cmd;
	struct mss_data *data;
	struct mss_driver *drv;
	struct mss_card *card;
	u32 ireg, stat, mask;

	pxa_host = (struct pxa_mss_host *)devid;
	cmd = pxa_host->cmd;
	data = pxa_host->data;
	card = pxa_host->host->active_card;

	ireg = readl(pxa_host->base + MMC_I_REG);
	stat = readl(pxa_host->base + MMC_STAT);
	mask = readl(pxa_host->base + MMC_I_MASK);

	dbg("host%d irq, I_REG:0x%x, STAT:0x%x, I_MASK:0x%x, CLKRT:0x%x," 
			"RESTO:0x%x, RDTO:0x%x\n", 
			pxa_host->host->id, ireg, stat, mask, 
			readl(pxa_host->base + MMC_CLKRT), 
			readl(pxa_host->base + MMC_RESTO),
			readl(pxa_host->base + MMC_RDTO));
	if (data) {
		if (data->flags & MSS_DATA_WRITE) {
			pxa_host_write_irq(pxa_host, ireg, stat, mask);
		}
		if (data->flags & MSS_DATA_READ) {
			pxa_host_read_irq(pxa_host, ireg, stat, mask);
		}
	}
	else if ((cmd->rtype == MSS_RESPONSE_R1B) && (!data || !data->sg_len))
		pxa_host_r1b_irq(pxa_host, ireg, stat, mask);
	else
		pxa_host_other_irq(pxa_host, ireg, stat, mask);
		
	/* handle sdio interrupt, wakeup sdio int IRQ handler thread */
	if ((ireg & MMC_I_REG_SDIO_INT) && (!(mask & MMC_I_MASK_SDIO_INT))) {
		struct pxa_slot *pxa_slot = pxa_host->active_slot->private;

		dbg("SDIO interrupt ocurred");
		pxa_host_disable_int(pxa_host, MMC_I_MASK_SDIO_INT);
		pxa_host->host->sdio_int = MSS_SDIO_INT_DIS;

		if (!pxa_host->active_slot->card ||
		    !pxa_host->active_slot->card->dev.driver)
			return IRQ_HANDLED;
		drv = container_of(pxa_host->active_slot->card->dev.driver, 
				struct mss_driver, driver);

		/* application driver sdio_int_handler should 
		 * get card from work_struct. e.g,
		 * sdio_int_handler(struct work_struct *work)
		 * { 
		 * 	struct pxa_slot * pxa_slot = 
		 * 		contrainer_of(work, struct pxa_slot, sdio_int.work);
		 * 	struct pxa_mss_host * pxa_host = 
		 * 		pxa_slot->slot->host->private;
		 * 	struct mss_card * card = pxa_host->active_slot->card;
		 * 	...
		 * }
		 */
		INIT_WORK(&pxa_slot->sdio_int, 
				(void (*)(void *))sdio_interrupt_handler);
		queue_work(pxa_host->sdio_work_queue, &pxa_slot->sdio_int);
	}
	return IRQ_HANDLED;
}

/**
 *  pxa_slot_gpio_irq
 *  @irq: IRQ number
 *  @devid: pointer to mss_slot
 *  @regs: interrupt context
 *
 *  hot-plug interrupt handler. devid represent slot 
 *  because hot-plug is slot-specific. 
 */
static irqreturn_t pxa_slot_gpio_irq(int irq, void *devid)
{
	struct mss_slot *slot = (struct mss_slot *)devid;
	struct pxa_slot *pxa_slot = slot->private;
	struct pxa_mss_host *pxa_host = slot->host->private;

	dbg("Slot :%d, inttrupt", slot->id);
	queue_delayed_work(pxa_host->work_queue, &pxa_slot->card_change, 20);

	return IRQ_HANDLED;	
}

#if 0
static int pxa_slot_work_thread(void *data)
{
	struct mss_slot *slot = data;
	struct pxa_slot *pxa_slot = slot->private;
	DECLARE_WAITQUEUE(wait, current);
	
	current->flags |= PF_MEMALLOC;

	daemonize("pxaslot%d.%d", slot->host->id, slot->id);

	complete(&pxa_slot->thread_complete);
	
	down(&pxa_slot->thread_sem);
	add_wait_queue(&pxa_slot->thread_wq, &wait);
	do {
		try_to_freeze();
		if (pxa_slot->flags & PXA_SLOT_SCAN) {
			pxa_slot->flags &= ~PXA_SLOT_SCAN;
			mss_scan_slot(slot);
		}
		set_current_state(TASK_INTERRUPTIBLE);
		if (pxa_slot->flags & PXA_SLOT_EXIT)
			break;
		up(&pxa_slot->thread_sem);
		schedule();
		down(&pxa_slot->thread_sem);
		set_current_state(TASK_RUNNING);
	} while(1);
	remove_wait_queue(&pxa_slot->thread_wq, &wait);
	up(&pxa_slot->thread_sem);

	complete_and_exit(&pxa_slot->thread_complete, 0);

}
#endif

static void pxa_slot_card_change(struct work_struct *work)
{
	struct pxa_slot *pxa_slot = 
		container_of(work, struct pxa_slot, card_change.work);
	struct mss_slot *slot = pxa_slot->slot;
	struct mss_card *card = slot->card;

	while (card && card->state == MSS_CARD_SUSPENDED)
		schedule_timeout(20);
	mss_scan_slot(slot);
}

/* init slot, request IRQ for slot, register mss_slot to core, initialize card 
 * if inserted 
 */
static int pxa_slot_init(struct pxa_mss_host *pxa_host, int slotid)
{
	struct mss_slot *slot;
	struct pxa_slot *pxa_slot;
	struct mss_host *host = pxa_host->host;
	int ret = 0;
	int irq;
	
	slot = &host->slots[slotid];
	pxa_slot = slot->private;
	pxa_slot->slot = slot;

	INIT_DELAYED_WORK(&pxa_slot->card_change, 
		(void (*)(void *))pxa_slot_card_change);
	
	irq = get_slot_cd_irq(slot);
	if (!irq) {
		printk(KERN_INFO "no card detect IRQ for MMC%d, slot:%d.\n", 
			host->id + 1, slotid);
		goto skip_cd_irq;
	}

	ret = request_irq(irq, pxa_slot_gpio_irq, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, 
		"MMC card detection", (void *)slot);
	if (ret) {
		printk(KERN_ERR "MMC:%d, slot%d, request irq %d fails," 
				" ret: %d\n", host->id, slotid, irq, ret);
		goto exit;
	}

	dbg("request IRQ %d for host%d, slot%d", irq, host->id, slot->id);

skip_cd_irq:
	mss_scan_slot(slot);
	
	return 0;
exit:
	return ret;
}

/**
 *  cleanup a slot, unregister mss_slot form core, free IRQ of slot, 
 *  free mss_slot 
 */
static void pxa_slot_exit(struct mss_slot *slot)
{

/*	pxa_slot->flags = PXA_SLOT_EXIT;
	wake_up(&pxa_slot->thread_wq);
	wait_for_completion(&pxa_slot->thread_complete);
*/
	if (slot->card)
		mss_force_card_remove(slot->card);
	free_irq(get_slot_cd_irq(slot), slot);
}

#ifdef CONFIG_MACH_ZYLONITE
static void pxa_mss_host_init(struct pxa_mss_host *pxa_host)
{
	struct mss_host *host = pxa_host->host;
	
	pxa_host->base = PXA_HOST_BASE[pxa_host->host->id];
	pxa_host->phybase = PXA_HOST_PHYBASE[pxa_host->host->id];
	pxa_host->cken = PXA_HOST_CKEN[pxa_host->host->id];
	pxa_set_cken(pxa_host->cken, 1);

	if (pxa_host->host->id == PXA_MMC_1) {	
		pxa3xx_gpio_set_direction(MFP_MMC_CMD_0, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC_CMD_0, GPIO_LEVEL_HIGH);

		pxa3xx_gpio_set_direction(MFP_MMC_CMD_1, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC_CMD_1, GPIO_LEVEL_HIGH);

		/* set direction of CD/WP to IN */
		pxa3xx_gpio_set_direction(MFP_MMC_CD_0_GPIO, GPIO_DIR_IN);
		pxa3xx_gpio_set_direction(MFP_MMC_WP_0_N_GPIO, GPIO_DIR_IN);
		
		set_slot_cd_irq(&host->slots[0], MMC_CD0);
		set_slot_cd_gpio(&host->slots[0], MFP_MMC_CD_0_GPIO);
		set_slot_wp_gpio(&host->slots[0], MFP_MMC_WP_0_N_GPIO);

		if (PXA_HOST_SLOTS[PXA_MMC_1] > 1) {
			pxa3xx_gpio_set_direction(MFP_MMC_CD_1_GPIO, GPIO_DIR_IN);
			pxa3xx_gpio_set_direction(MFP_MMC_WP_1_N_GPIO,GPIO_DIR_IN);
	
			set_slot_cd_irq(&host->slots[1], MMC_CD1);
			set_slot_cd_gpio(&host->slots[1], MFP_MMC_CD_1_GPIO);
			set_slot_wp_gpio(&host->slots[1], MFP_MMC_WP_1_N_GPIO);
		}
	} else if (pxa_host->host->id == PXA_MMC_2) {
		pxa3xx_gpio_set_direction(MFP_MMC2_CMD, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC2_CMD, GPIO_LEVEL_HIGH);
			
		set_slot_cd_irq(&host->slots[0], 0);
		set_slot_cd_gpio(&host->slots[0], 0);
		set_slot_wp_gpio(&host->slots[0], 0);
	}
#ifdef CONFIG_CPU_PXA310
	else if (pxa_host->host->id == PXA_MMC_3) {
		pxa3xx_gpio_set_direction(MFP_MMC3_CMD, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC3_CMD, GPIO_LEVEL_HIGH);
	
		/* set direction of CD/WP to IN */
		pxa3xx_gpio_set_direction(MFP_MMC_CD_3_GPIO, GPIO_DIR_IN);
		pxa3xx_gpio_set_direction(MFP_MMC_WP_3_N_GPIO, GPIO_DIR_IN);

		set_slot_cd_irq(&host->slots[0], MMC_CD3);
		set_slot_cd_gpio(&host->slots[0], MFP_MMC_CD_3_GPIO);
		set_slot_wp_gpio(&host->slots[0], MFP_MMC_WP_3_N_GPIO);
	}
#endif
	writel(CLKRT_0_304MHZ, pxa_host->base + MMC_CLKRT);
	writel(RES_TIMEOUT_MAX, pxa_host->base + MMC_RESTO);
	writel(RD_TIMEOUT_MAX, pxa_host->base + MMC_RDTO);
	writel(MMC_MASK_ALL, pxa_host->base + MMC_I_MASK);
	writel(0, pxa_host->base + MMC_SPI);
	pxa_host_start_busclock(pxa_host);
}

#elif	defined(CONFIG_MACH_LITTLETON)
static void pxa_mss_host_init(struct pxa_mss_host *pxa_host)
{
	struct mss_host *host = pxa_host->host;
	
	pxa_host->base = PXA_HOST_BASE[pxa_host->host->id];
	pxa_host->phybase = PXA_HOST_PHYBASE[pxa_host->host->id];
	pxa_host->cken = PXA_HOST_CKEN[pxa_host->host->id];
	pxa_set_cken(pxa_host->cken, 1);

	if (pxa_host->host->id == PXA_MMC_1) {	
		pxa3xx_gpio_set_direction(MFP_MMC_CMD_0, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC_CMD_0, GPIO_LEVEL_HIGH);

		/* set direction of CD to IN */
		pxa3xx_gpio_set_direction(MFP_MMC_CD_0_GPIO, GPIO_DIR_IN);
		
		set_slot_cd_irq(&host->slots[0], MMC_CD0);
		set_slot_cd_gpio(&host->slots[0], MFP_MMC_CD_0_GPIO);

	} else if (pxa_host->host->id == PXA_MMC_2) {
		pxa3xx_gpio_set_direction(MFP_MMC2_CMD, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC2_CMD, GPIO_LEVEL_HIGH);
			
		set_slot_cd_irq(&host->slots[0], 0);
		set_slot_cd_gpio(&host->slots[0], 0);
		set_slot_wp_gpio(&host->slots[0], 0);
	}
#ifdef CONFIG_CPU_PXA310
	else if (pxa_host->host->id == PXA_MMC_3) {
		pxa3xx_gpio_set_direction(MFP_MMC3_CMD, GPIO_DIR_OUT);
		pxa3xx_gpio_set_level(MFP_MMC3_CMD, GPIO_LEVEL_HIGH);
	}
#endif
	writel(CLKRT_0_304MHZ, pxa_host->base + MMC_CLKRT);
	writel(RES_TIMEOUT_MAX, pxa_host->base + MMC_RESTO);
	writel(RD_TIMEOUT_MAX, pxa_host->base + MMC_RDTO);
	writel(MMC_MASK_ALL, pxa_host->base + MMC_I_MASK);
	writel(0, pxa_host->base + MMC_SPI);
	pxa_host_start_busclock(pxa_host);
}

#else
#error	"Please select correct platform for build"
#endif

static struct mss_host_ops pxa_mss_host_ops = {
	.enable_sdio_int= pxa_mss_enable_sdio_int,
	.request	= pxa_mss_handle_request,
	.set_ios	= pxa_mss_set_ios,
	.is_slot_empty	= pxa_mss_slot_is_empty,
	.is_slot_wp	= pxa_mss_slot_is_wp,
};

/**
  *  pxa_mss_controller_probe
  *  @dev:	device
  *  
  *  init controller, request CTROLLER IRQ, request DMA IRQ, create controller
  *  threads, invoke slot_init
  */

#ifdef	CONFIG_MACH_ZYLONITE
#define	PXA_IOS_VDD		MSS_VDD_32_33
#elif	defined (CONFIG_MACH_LITTLETON)
#define	PXA_IOS_VDD		MSS_VDD_28_29
#else
#error "Please select correct platform for build"
#endif

static int pxa_mss_host_probe(struct device *dev)
{
	int ret = 0, i = 0;
	struct platform_device *pdev;
	struct pxa_mss_host *pxa_host;
	struct mss_host *host;

	pdev = to_platform_device(dev);
	if (pdev->id > PXA_MMC_MAX)
		return -ENODEV;
	host = mss_alloc_host(PXA_HOST_SLOTS[pdev->id], \
		pdev->id, sizeof(struct pxa_mss_host) + \
		PXA_HOST_SLOTS[pdev->id] * sizeof(struct pxa_slot));
	if (NULL == host)
		return -ENOMEM;
	
	pxa_host = (struct pxa_mss_host *)host->private;
	pxa_host->host = host;
	pxa_host->irq = PXA_HOST_IRQ[pdev->id];
	pxa_host->dat1_gpio_irq = 0;
	pxa_host->drcmrtx = PXA_HOST_DRCMRTX[pdev->id];
	pxa_host->drcmrrx = PXA_HOST_DRCMRRX[pdev->id];
	pxa_host->dma_run = 0;

	/* Set host to be 3.2V */
#ifdef CONFIG_MACH_ZYLONITE
	pxa3xx_pmic_set_voltage(VCC_SDIO, 3200);
#endif
	host->dev = dev;
	host->ops = &pxa_mss_host_ops;
	host->vdd = host->ios.vdd = PXA_IOS_VDD;
	host->bus_width = MSS_BUSWIDTH_4BIT;
	host->ios.bus_width = MSS_BUSWIDTH_1BIT;
	host->f_min = 304000;
	host->f_max = 26000000;
	host->sd_spec = MSS_SD_SPEC_20;
	host->mmc_spec = MSS_MMC_SPEC_40_42;
	host->sdio_spec = MSS_SDIO_SPEC_11;
	host->high_capacity = 1;
	host->max_phys_segs = 32;
	host->max_hw_segs = 32;
	host->max_sectors = 256;
	host->max_seg_size = 0x1000;

	for (i = 0; i < PXA_HOST_SLOTS[pdev->id]; i++) {
		host->slots[i].private	= (void *)((unsigned int)(&pxa_host[1]) 
			+ i * sizeof(struct pxa_slot));
		dbg("host:%d, slot%d, private:0x%x", host->id, 
				host->slots[i].id, host->slots[i].private);
	}

	pxa_mss_host_init(pxa_host);

#ifdef CONFIG_PXA_MWB_12
	if (pxa_host->host->id == PXA_MMC_2) {
		pxa3xx_enable_pxa_mwb_wifi();
	}
#endif

	ret = register_mss_host(host);
	if (ret) {
		printk(KERN_ERR "register_mss_host error! ret: %d.\n", ret);
		goto free_host;
	}
	/* request controller IRQ */
	ret = request_irq(pxa_host->irq, pxa_host_irq, 0, \
		"Monahans-P MMC/SD/SDIO Controller", (void *)pxa_host);
	if (ret < 0) {
		printk(KERN_ERR "MMC%d request_irq error! ret: %d.\n", pdev->id, ret);
		goto unregister_host;
	}
	/* request DMA IRQ */
	pxa_host->dma = pxa_request_dma("Monahans mmc", DMA_PRIO_LOW, 
				pxa_host_dma_irq, (void *)pxa_host);
	dbg("DMA channel : %d, %x\n", pxa_host->dma, (u32)&pxa_host->dma);
	if (pxa_host->dma < 0) {
		ret = -EBUSY;
		printk(KERN_ERR "MMC%d request_dma error! ret: %d.\n", pdev->id, ret);
		goto free_host_irq;
	}
	pxa_host->sg_cpu = dma_alloc_coherent(dev, PAGE_SIZE, 
			&pxa_host->sg_dma, GFP_KERNEL);
	if (!pxa_host->sg_cpu) {
		ret = -ENOMEM;
		goto free_dma_irq;
	}
		
	sprintf(pxa_host->name, "PXA HOST %d", host->id);
	sprintf(pxa_host->name2, "SDIO INT PXA HOST %d", host->id);
	pxa_host->work_queue = create_workqueue(pxa_host->name);	
	pxa_host->sdio_work_queue = create_workqueue(pxa_host->name2);	
 
	for(i = 0; i < PXA_HOST_SLOTS[pdev->id]; i++) {
		ret = pxa_slot_init(pxa_host, i);
		if (ret)
			goto free_slot;	
	}	
#ifdef CONFIG_DVFM
	pxa_host->dvfm_notifier.name = pxa_host->name;
	pxa_host->dvfm_notifier.priority = 0,
	pxa_host->dvfm_notifier.notifier_call = pxa3xx_mmc_dvfm_notifier;
	pxa_host->dvfm_notifier.client_data = pxa_host;
	pxa3xx_fv_register_notifier(&(pxa_host->dvfm_notifier));
#endif
	return 0;
free_slot:
	while (--i >= 0) {
		pxa_slot_exit(&host->slots[i]);
	}
free_dma_irq:
	/* release DMA IRQ */
	pxa_free_dma(pxa_host->dma);

free_host_irq:
	/* release controller IRQ */
	free_irq(pxa_host->irq, pxa_host);

unregister_host:
	unregister_mss_host(host);

free_host:
	mss_free_host(host);
	return ret;
}


/**
 *  pxa_mss_controller_remove
 *  @dev: device
 *  
 *  cleanup controller, free CTROLLER IRQ, free DMA IRQ, destroy controller
 *  threads, invoke slot_exit
 */
static int pxa_mss_host_remove(struct device *dev)
{
	int ret = 0, i;
	struct platform_device *pdev;
	struct mss_host *host;
	struct pxa_mss_host *pxa_host;

	pdev = to_platform_device(dev);
	host = mss_find_host(pdev->id);
	if (!host)
		return -ENODEV;
	pxa_host = (struct pxa_mss_host *)host->private;
#ifdef CONFIG_DVFM
        pxa3xx_fv_unregister_notifier(&(pxa_host->dvfm_notifier));
#endif
	destroy_workqueue(pxa_host->work_queue);
	destroy_workqueue(pxa_host->sdio_work_queue);
	free_irq(pxa_host->irq, pxa_host);
	pxa_free_dma(pxa_host->dma);
	dma_free_coherent(dev, PAGE_SIZE, pxa_host->sg_cpu, pxa_host->sg_dma);

	for (i = 0; i < host->slot_num; i++) {
		pxa_slot_exit(&host->slots[i]);
	}

	unregister_mss_host(host);
	mss_free_host(host);
	return ret;
}

static void pxa_mss_host_shutdown(struct device *dev)
{
	return;
}

/**
 *  pxa_mss_controller_suspend
 *  @dev: device
 *  @state: suspend state
 *  @level: suspend level
 */
static int pxa_mss_host_suspend(struct device *dev, pm_message_t state)
{
	struct platform_device *pdev;
	struct pxa_mss_host *pxa_host;
	struct mss_host *host;

	pdev = to_platform_device(dev);
	host = mss_find_host(pdev->id);
	pxa_host = host->private;

	pxa_set_cken(pxa_host->cken, 0);

	return 0;
}

/**
 *  pxa_mss_controller_resume
 *  @dev: device
 */
extern int mss_init_card(struct mss_card *card);
static int pxa_mss_host_resume(struct device *dev)
{
	struct platform_device *pdev;
	struct pxa_mss_host *pxa_host;
	struct mss_host *host;
	int i, ret;
	struct mss_ios ios;

	pdev = to_platform_device(dev);
	host = mss_find_host(pdev->id);
	pxa_host = host->private;

	pxa_mss_host_init(pxa_host);	
	memcpy(&ios, &host->ios, sizeof(struct mss_ios));
	memset(&host->ios, 0x0, sizeof(struct mss_ios));
	pxa_mss_set_ios(host, &ios);
	if (host->sdio_int) {
		writel(0x0, pxa_host->base + MMC_CMD);
		writel(MMC_CMDAT_SDIO_INT, pxa_host->base + MMC_CMDAT);
	}
	pxa_mss_enable_sdio_int(host, host->sdio_int);
 
	for (i = 0; i < host->slot_num; i++) {
		struct mss_slot *slot = &host->slots[i];
		if (slot->card 
			&& slot->card->card_type == MSS_SDIO_CARD) {
#ifdef CONFIG_PXA_MWB_12
			int retry = 0;

#define MAX_RETRY	5
			pxa3xx_enable_pxa_mwb_wifi();
			do {
				ret = mss_init_card(slot->card);
			} while(ret && MAX_RETRY > retry++);
			if (!ret)
				slot->card->state |= MSS_CARD_INITED;
#endif
			return 0;
		}
 
		ret = mss_scan_slot(slot);
		if (ret) {	/* == MSS_ERROR_MISMATCH_CARD) { */
			struct pxa_slot *pxa_slot = slot->private;
			queue_delayed_work(pxa_host->work_queue, 
					&pxa_slot->card_change, 20);
		}
	}

	return 0;
}

/*****************************************************************************
 *
 *   object instances
 *
 ****************************************************************************/
static struct device_driver pxa_mss_host_driver = {
	.name		=	"mmc_controller",
	.bus		=	&platform_bus_type,
	.probe		= 	pxa_mss_host_probe,
	.remove		=	pxa_mss_host_remove,
	.shutdown	=	pxa_mss_host_shutdown,
	.suspend	=	pxa_mss_host_suspend,
	.resume		=	pxa_mss_host_resume,
};

static int pxa_mss_host_driver_init(void)
{
	int ret;

	/* register controller driver to PLATFORM bus */
	ret = driver_register(&pxa_mss_host_driver);
	if (ret)
		return ret;

 	return ret;	
}

static void pxa_mss_host_driver_exit(void)
{
	driver_unregister(&pxa_mss_host_driver);
}

module_init(pxa_mss_host_driver_init);
module_exit(pxa_mss_host_driver_exit);

MODULE_AUTHOR("Bridge Wu");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Controller driver for MMC/SD/SDIO card");
