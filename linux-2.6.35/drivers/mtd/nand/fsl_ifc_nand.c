/*
 * Freescale Integrated Flash Controller NAND driver
 *
 * Copyright 2011 Freescale Semiconductor, Inc
 *
 * Author: Dipen Dudhat <Dipen.Dudhat@freescale.com>
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
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/fsl_ifc.h>

#ifdef CONFIG_MTD_NAND_SONOS_VNB_MAPPING
#include "virtual_block.h"
#endif

#define ERR_BYTE		0xFF /* Value returned for read
					bytes when read failed	*/
#define IFC_TIMEOUT_MSECS	500  /* Maximum number of mSecs to wait
					for IFC NAND Machine	*/

struct fsl_ifc_ctrl;

/* mtd information per set */
struct fsl_ifc_mtd {
	struct mtd_info mtd;
	struct nand_chip chip;
	struct fsl_ifc_ctrl *ctrl;

	struct device *dev;
	int bank;		/* Chip select bank number		*/
	u8 __iomem *vbase;      /* Chip select base virtual address	*/
	unsigned int bufnum_mask;
};

/* overview of the fsl ifc controller */
struct fsl_ifc_nand_ctrl {
	struct nand_hw_control controller;
	struct fsl_ifc_mtd *chips[FSL_IFC_BANK_COUNT];

	u8 __iomem *addr;	/* Address of assigned IFC buffer	*/
	unsigned int page;	/* Last page written to / read from	*/
	unsigned int read_bytes;/* Number of bytes read during command	*/
	unsigned int column;	/* Saved column from SEQIN		*/
	unsigned int index;	/* Pointer to next byte to 'read'	*/
	unsigned int status;	/* status read after last op		*/
	unsigned int mdr;	/* IFC Data Register value		*/
	unsigned int use_mdr;	/* Non zero if the MDR is to be set	*/
	unsigned int oob;	/* Non zero if operating on OOB data	*/
	char *oob_poi;

	wait_queue_head_t		irq_wait;
};

static struct fsl_ifc_nand_ctrl *ifc_nand_ctrl;

int nand_get_device_exp(struct nand_chip *chip, struct mtd_info *mtd, int new_state);
void nand_release_device_exp(struct mtd_info *mtd);

/* These map to the positions used by the IFC NAND hardware ECC generator */
/* Small Page FLASH with 4-bit Mode ECC */
static struct nand_ecclayout fsl_ifc_oob_sp_eccm0 = {
	.eccbytes = 8,
	.eccpos = {8, 9, 10, 11, 12, 13, 14, 15},
	.oobfree = { {0, 5}, {7, 1} },
};

/* Large Page 2K Page size FLASH with 4-bit Mode ECC */
static struct nand_ecclayout fsl_ifc_oob_lp_2k_eccm0 = {
	.eccbytes = 32,
	.eccpos = {
		8, 9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31,
		32, 33, 34, 35, 36, 37, 38, 39
	},
	.oobfree = { {2, 6}, {40, 24} },
};

/* Large Page 2K Page size FLASH with 8-bit Mode ECC */
static struct nand_ecclayout fsl_ifc_oob_lp_2k_eccm1 = {
	.eccbytes = 64,
	.eccpos = {
		8, 9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31,
		32, 33, 34, 35, 36, 37, 38, 39,
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63,
		64, 65, 66, 67, 68, 69, 70, 71
	},
	.oobfree = { {2, 6}, {72, 56} },
};

/* Large Page 4K Page size FLASH with 4-bit Mode ECC */
static struct nand_ecclayout fsl_ifc_oob_lp_4k_eccm0 = {
	.eccbytes = 64,
	.eccpos = {
		8, 9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31,
		32, 33, 34, 35, 36, 37, 38, 39,
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63,
		64, 65, 66, 67, 68, 69, 70, 71
	},
	.oobfree = { {2, 6}, {72, 56} },
};

/*
 * Generic flash bbt descriptors
 */
static u8 bbt_pattern[] = {'B', 'b', 't', '0' };
static u8 mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	2,
	.len = 4,
	.veroffs = 6,
	.maxblocks = 4,
	.pattern = bbt_pattern,
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	2,
	.len = 4,
	.veroffs = 6,
	.maxblocks = 4,
	.pattern = mirror_pattern,
};

/*
 * Set up the IFC hardware block and page address fields, and the ifc nand
 * structure addr field to point to the correct IFC buffer in memory
 */
static void set_addr(struct mtd_info *mtd, int column,
				int page_addr, int oob) {
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	int buf_num;

	ifc_nand_ctrl->page = page_addr;
	/* Program ROW0/COL0 */
	out_be32(&ifc->ifc_nand.row0, page_addr);
	out_be32(&ifc->ifc_nand.col0, (oob ? IFC_NAND_COL_MS : 0) | column);

	/*if (mtd->writesize == 4096) AJ: sic
		buf_num = page_addr & 0x1;
	else if (mtd->writesize == 2048)
		buf_num = page_addr & 0x3;
	else
		buf_num = page_addr & 0xf;*/
	buf_num = page_addr & priv->bufnum_mask;

	ifc_nand_ctrl->addr = priv->vbase + buf_num * (mtd->writesize * 2);
	ifc_nand_ctrl->index = column;

	/* for OOB data point to the second half of the buffer */
	if (oob)
		ifc_nand_ctrl->index += mtd->writesize;
}

static int is_blank(struct mtd_info *mtd, struct fsl_ifc_ctrl *ctrl,
					unsigned int bufnum)
{
	struct nand_chip *chip = mtd->priv;

	u8 __iomem *addr = (u8*)ifc_nand_ctrl->addr;
	volatile u8 __iomem *oob = addr + mtd->writesize;
	volatile u32 __iomem *body = (u32*) ifc_nand_ctrl->addr;
	int i;

	for (i = 0; i < mtd->writesize / 4; i++) {
		if (body[i] != 0xffffffff) {
			return 0;
		}
	}

	for (i = 0; i < chip->ecc.layout->eccbytes; i++) {
		int pos = chip->ecc.layout->eccpos[i];
		if(oob[pos] != 0xff) {
			return 0;
		}
	}

	return 1;
}

static int check_read_ecc(struct mtd_info *mtd, struct fsl_ifc_ctrl *ctrl,
							u32 *eccstat, unsigned int bufnum)
{
	u32 reg = eccstat[bufnum >> 2];
	int errors = (reg >> ((3 - bufnum % 4) * 8)) & 15;

	if(errors == 15) {
		if(is_blank(mtd, ctrl, bufnum))
			return 1;
		ctrl->status |= IFC_NAND_EVTER_STAT_ECCER;
	} else if (errors >0) {
		mtd->ecc_stats.corrected += errors;
	}
	return 0;
}

/*
 * execute IFC NAND command and wait for it to complete
 */
static int fsl_ifc_run_command(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	int bufperpage = mtd->writesize / 512;
	int i, bufnum = (ifc_nand_ctrl->page & priv->bufnum_mask) * bufperpage;
	int bufnum_end = bufnum + bufperpage - 1;
	u32 eccstat[4];

	if (ifc_nand_ctrl->use_mdr)
		out_be32(&ifc->ifc_nand.nand_mdr, ifc_nand_ctrl->mdr);

	out_be32(&ifc->ifc_nand.ncfgr, 0x0);

	dev_vdbg(priv->dev,
			"%s: fir0=%08x fcr0=%08x\n",
			__func__,
			in_be32(&ifc->ifc_nand.nand_fir0),
			in_be32(&ifc->ifc_nand.nand_fcr0));

	ifc_nand_ctrl->status = 0;

	/* start read/write seq */
	out_be32(&ifc->ifc_nand.nandseq_strt, IFC_NAND_SEQ_STRT_FIR_STRT);

	/* wait for command complete flag or timeout */
	wait_event_timeout(ifc_nand_ctrl->irq_wait, ifc_nand_ctrl->status,
				IFC_TIMEOUT_MSECS * HZ/1000);

	out_be32(&ifc->csor_cs[0].csor, in_be32(&ifc->csor_cs[0].csor) & ~CSOR_NAND_ECC_ENC_EN);

	/* AJ: Leaving this here - freescale has an erratum regarding controller status reporting
	  printk("Timeout? %i %x.\n", ifc_nand_ctrl->status, IFC_TIMEOUT_MSECS * HZ/1000);
	*/

	/* store mdr value in case it was needed */
	if (ifc_nand_ctrl->use_mdr)
		ifc_nand_ctrl->mdr = in_be32(&ifc->ifc_nand.nand_mdr);

	ifc_nand_ctrl->use_mdr = 0;

	/* enable NAND Machine Interrupts which we have disabled in ISR */
	out_be32(&ifc->ifc_nand.nand_evter_intr_en,
			IFC_NAND_EVTER_INTR_OPCIR_EN |
			IFC_NAND_EVTER_INTR_FTOERIR_EN |
			IFC_NAND_EVTER_INTR_WPERIR_EN |
			IFC_NAND_EVTER_INTR_ECCERIR_EN);

	if (ctrl->eccread) {
		i = 0;
		eccstat[i++] = in_be32(&ifc->ifc_nand.nand_eccstat0);
		eccstat[i++] = in_be32(&ifc->ifc_nand.nand_eccstat1);
		eccstat[i++] = in_be32(&ifc->ifc_nand.nand_eccstat2);
		eccstat[i++] = in_be32(&ifc->ifc_nand.nand_eccstat3);

	   	for(i = bufnum; i <= bufnum_end; i++) {
			if(check_read_ecc(mtd, ctrl, eccstat, i)) {
				break;
			}
		}
	   	out_be32(&ifc->csor_cs[0].csor, (in_be32(&ifc->csor_cs[0].csor) & ~CSOR_NAND_ECC_DEC_EN));
	   	ctrl->eccread = 0;
	}

	if (ifc_nand_ctrl->status == IFC_NAND_EVTER_STAT_OPC)
		return 0;
   	dev_info(priv->dev,
   		"command failed: fir0: %x fcr0: %x"
   		"status: %x\n",
   		in_be32(&ifc->ifc_nand.nand_fir0),
   		in_be32(&ifc->ifc_nand.nand_fcr0),
   		ifc_nand_ctrl->status);
   	return -EIO;
}

static void fsl_ifc_do_read(struct nand_chip *chip,
			    int oob,
			    struct mtd_info *mtd)
{
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;

	/* Program FIR/IFC_NAND_FCR0 for Small/Large page */
	if (mtd->writesize > 512) {
		out_be32(&ifc->ifc_nand.nand_fir0,
			 (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
			 (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP1_SHIFT) |
			 (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP2_SHIFT) |
			 (IFC_FIR_OP_CMD1 << IFC_NAND_FIR0_OP3_SHIFT) |
			 (IFC_FIR_OP_RBCD << IFC_NAND_FIR0_OP4_SHIFT));
		out_be32(&ifc->ifc_nand.nand_fir1, 0x0);

		out_be32(&ifc->ifc_nand.nand_fcr0,
			(NAND_CMD_READ0 << IFC_NAND_FCR0_CMD0_SHIFT) |
			(NAND_CMD_READSTART << IFC_NAND_FCR0_CMD1_SHIFT));
	} else {
		out_be32(&ifc->ifc_nand.nand_fir0,
			 (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
			 (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP1_SHIFT) |
			 (IFC_FIR_OP_RA0  << IFC_NAND_FIR0_OP2_SHIFT) |
			 (IFC_FIR_OP_RBCD << IFC_NAND_FIR0_OP3_SHIFT));
		out_be32(&ifc->ifc_nand.nand_fir1, 0x0);

		if (oob)
			out_be32(&ifc->ifc_nand.nand_fcr0,
				 NAND_CMD_READOOB << IFC_NAND_FCR0_CMD0_SHIFT);
		else
			out_be32(&ifc->ifc_nand.nand_fcr0,
				NAND_CMD_READ0 << IFC_NAND_FCR0_CMD0_SHIFT);
	}

	if (!oob && (priv->chip.ecc.mode == NAND_ECC_HW)) {
		out_be32(&ifc->csor_cs[0].csor, (in_be32(&ifc->csor_cs[0].csor) | CSOR_NAND_ECC_DEC_EN));
		ctrl->eccread = 1;
	}
}

/* cmdfunc send commands to the IFC NAND Machine */
static void fsl_ifc_cmdfunc(struct mtd_info *mtd, unsigned int command,
			     int column, int page_addr) {
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;

	/* set the chip select for NAND Transaction */
	out_be32(&ifc->ifc_nand.nand_csel, priv->bank << IFC_NAND_CSEL_SHIFT);

	ifc_nand_ctrl->use_mdr = 0;

	/* clear the read buffer */
	ifc_nand_ctrl->read_bytes = 0;
	if (command != NAND_CMD_PAGEPROG)
		ifc_nand_ctrl->index = 0;

	switch (command) {
	/* READ0 read the entire buffer to use hardware ECC. */
	case NAND_CMD_READ0:
		out_be32(&ifc->ifc_nand.nand_fbcr, 0);
		set_addr(mtd, 0, page_addr, 0);

		ifc_nand_ctrl->read_bytes = mtd->writesize + mtd->oobsize;
		ifc_nand_ctrl->index += column;

		fsl_ifc_do_read(chip, 0, mtd);
		
		fsl_ifc_run_command(mtd);
		return;

	/* READOOB reads only the OOB because no ECC is performed. */
	case NAND_CMD_READOOB:
		out_be32(&ifc->ifc_nand.nand_fbcr, mtd->oobsize - column);
		set_addr(mtd, column, page_addr, 1);

		ifc_nand_ctrl->read_bytes = mtd->writesize + mtd->oobsize;

		fsl_ifc_do_read(chip, 1, mtd);
		fsl_ifc_run_command(mtd);

		return;

	/* READID must read all 8 possible bytes */
	case NAND_CMD_READID:
		out_be32(&ifc->ifc_nand.nand_fir0,
				(IFC_FIR_OP_CMD0 << IFC_NAND_FIR0_OP0_SHIFT) |
				(IFC_FIR_OP_UA  << IFC_NAND_FIR0_OP1_SHIFT) |
				(IFC_FIR_OP_RB << IFC_NAND_FIR0_OP2_SHIFT));
		out_be32(&ifc->ifc_nand.nand_fcr0,
				NAND_CMD_READID << IFC_NAND_FCR0_CMD0_SHIFT);
		/* 8 bytes for manuf, device and exts */
		out_be32(&ifc->ifc_nand.nand_fbcr, 8);
		ifc_nand_ctrl->read_bytes = 8;
		ifc_nand_ctrl->use_mdr = 0;
		ifc_nand_ctrl->mdr = 0;

		set_addr(mtd, 0, 0, 0);
		fsl_ifc_run_command(mtd);
		return;

	/* ERASE1 stores the block and page address */
	case NAND_CMD_ERASE1:
		set_addr(mtd, 0, page_addr, 0);
		return;

	/* ERASE2 uses the block and page address from ERASE1 */
	case NAND_CMD_ERASE2:
		out_be32(&ifc->ifc_nand.nand_fir0,
			 (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
			 (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP1_SHIFT) |
			 (IFC_FIR_OP_CMD1 << IFC_NAND_FIR0_OP2_SHIFT));

		out_be32(&ifc->ifc_nand.nand_fcr0,
			 (NAND_CMD_ERASE1 << IFC_NAND_FCR0_CMD0_SHIFT) |
			 (NAND_CMD_ERASE2 << IFC_NAND_FCR0_CMD1_SHIFT));

		out_be32(&ifc->ifc_nand.nand_fbcr, 0);
		ifc_nand_ctrl->read_bytes = 0;
		fsl_ifc_run_command(mtd);
		return;

	/* SEQIN sets up the addr buffer and all registers except the length */
	case NAND_CMD_SEQIN: {
		u32 nand_fcr0;
		ifc_nand_ctrl->column = column;
		ifc_nand_ctrl->oob = 0;

		if (mtd->writesize > 512) {
			nand_fcr0 =
				(NAND_CMD_SEQIN << IFC_NAND_FCR0_CMD0_SHIFT) |
				(NAND_CMD_PAGEPROG << IFC_NAND_FCR0_CMD1_SHIFT);

			out_be32(&ifc->ifc_nand.nand_fir0,
				 (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
				 (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP1_SHIFT) |
				 (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP2_SHIFT) |
				 (IFC_FIR_OP_WBCD  << IFC_NAND_FIR0_OP3_SHIFT) |
				 (IFC_FIR_OP_CW1 << IFC_NAND_FIR0_OP4_SHIFT));
		} else {
			nand_fcr0 = ((NAND_CMD_PAGEPROG <<
					IFC_NAND_FCR0_CMD1_SHIFT) |
				    (NAND_CMD_SEQIN <<
					IFC_NAND_FCR0_CMD2_SHIFT));

			out_be32(&ifc->ifc_nand.nand_fir0,
				 (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
				 (IFC_FIR_OP_CMD2 << IFC_NAND_FIR0_OP1_SHIFT) |
				 (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP2_SHIFT) |
				 (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP3_SHIFT) |
				 (IFC_FIR_OP_WBCD << IFC_NAND_FIR0_OP4_SHIFT));
			out_be32(&ifc->ifc_nand.nand_fir1,
				 (IFC_FIR_OP_CW1 << IFC_NAND_FIR1_OP5_SHIFT));

			if (column >= mtd->writesize) {
				/* OOB area --> READOOB */
				nand_fcr0 |= NAND_CMD_READOOB <<
						IFC_NAND_FCR0_CMD0_SHIFT;
			} else if (column < 256)
				/* First 256 bytes --> READ0 */
				nand_fcr0 |=
				NAND_CMD_READ0 << IFC_NAND_FCR0_CMD0_SHIFT;
			else
				/* Second 256 bytes --> READ1 */
				nand_fcr0 |=
				NAND_CMD_READ1 << IFC_NAND_FCR0_CMD0_SHIFT;
		}

        if (column >= mtd->writesize) {
            column -= mtd->writesize;
            ifc_nand_ctrl->oob = 1;
        }

		out_be32(&ifc->ifc_nand.nand_fcr0, nand_fcr0);
		set_addr(mtd, column, page_addr, ifc_nand_ctrl->oob);
		return;
	}

	/* PAGEPROG reuses all of the setup from SEQIN and adds the length */
	case NAND_CMD_PAGEPROG: {
		if (ifc_nand_ctrl->oob) {
            out_be32(&ifc->ifc_nand.nand_fbcr, ifc_nand_ctrl->index - ifc_nand_ctrl->column);
		} else {
			out_be32(&ifc->ifc_nand.nand_fbcr, 0);
			if(!ifc_nand_ctrl->index)
				out_be32(&ifc->csor_cs[0].csor, in_be32(&ifc->csor_cs[0].csor) | CSOR_NAND_ECC_ENC_EN);
		}

		fsl_ifc_run_command(mtd);

		ifc_nand_ctrl->oob_poi = NULL;
		return;
	}

	case NAND_CMD_STATUS:
		out_be32(&ifc->ifc_nand.nand_fir0,
				(IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
				(IFC_FIR_OP_RB << IFC_NAND_FIR0_OP1_SHIFT));
		out_be32(&ifc->ifc_nand.nand_fcr0,
				NAND_CMD_STATUS << IFC_NAND_FCR0_CMD0_SHIFT);
		out_be32(&ifc->ifc_nand.nand_fbcr, 1);
		set_addr(mtd, 0, 0, 0);
		ifc_nand_ctrl->read_bytes = 1;

		fsl_ifc_run_command(mtd);

		/*
		 * The chip always seems to report that it is
		 * write-protected, even when it is not.
		 */
		setbits8(ifc_nand_ctrl->addr, NAND_STATUS_WP);
		return;

	case NAND_CMD_RESET:
		out_be32(&ifc->ifc_nand.nand_fir0,
				IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT);
		out_be32(&ifc->ifc_nand.nand_fcr0,
				NAND_CMD_RESET << IFC_NAND_FCR0_CMD0_SHIFT);
		fsl_ifc_run_command(mtd);
		return;
		
	default:
		dev_err(ctrl->dev, "%s: error, unsupported command 0x%x.\n",
					__func__, command);
	}
}

static void fsl_ifc_select_chip(struct mtd_info *mtd, int chip)
{
	/* The hardware does not seem to support multiple
	 * chips per bank.
	 */
}

/*
 * Write buf to the IFC NAND Controller Data Buffer
 */
static void fsl_ifc_write_buf(struct mtd_info *mtd, const u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	unsigned int bufsize = mtd->writesize + mtd->oobsize;

	if ((priv->chip.ecc.mode == NAND_ECC_HW) && (!ifc_nand_ctrl->index)) {
		if ((len == mtd->writesize) || (len == bufsize)) {
		   	out_be32(&ifc->csor_cs[0].csor, (in_be32(&ifc->csor_cs[0].csor) | CSOR_NAND_ECC_ENC_EN));
			out_be32(&ifc->ifc_nand.nand_fbcr, ifc_nand_ctrl->index);
		}
	}

	if (len <= 0) {
		dev_err(priv->dev, "%s: write_buf of %d bytes", __func__, len);
		ifc_nand_ctrl->status = 0;
		return;
	}

	if ((unsigned int)len > bufsize - ifc_nand_ctrl->index) {
		dev_err(priv->dev,
			"%s: write_buf beyond end of buffer "
			"(%d requested, %u available)\n",
			__func__, len, bufsize - ifc_nand_ctrl->index);
		len = bufsize - ifc_nand_ctrl->index;
	}

	memcpy_toio(&ifc_nand_ctrl->addr[ifc_nand_ctrl->index], buf, len);
	ifc_nand_ctrl->index += len;
}

/*
 * Read two bytes from the IFC hardware buffer
 * read function for 16-bit buswith
 */
static uint8_t fsl_ifc_read_byte16(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	uint16_t data;

	/*
	 * If there are still bytes in the IFC buffer, then use the
	 * next byte.
	 */
	if (ifc_nand_ctrl->index < ifc_nand_ctrl->read_bytes) {
		data = in_be16((uint16_t *)&ifc_nand_ctrl->
					addr[ifc_nand_ctrl->index]);
		ifc_nand_ctrl->index += 2;
		return (uint8_t) data;
	}

	dev_err(priv->dev, "%s: read_byte16 beyond end of buffer\n", __func__);
	return ERR_BYTE;
}

/*
 * Read a byte from either the IFC hardware buffer
 * read function for 8-bit buswidth
 */
static uint8_t fsl_ifc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;

	/*
	 * If there are still bytes in the IFC buffer, then use the
	 * next byte.
	 */
	if (ifc_nand_ctrl->index < ifc_nand_ctrl->read_bytes)
		return in_8(&ifc_nand_ctrl->addr[ifc_nand_ctrl->index++]);

	dev_err(priv->dev, "%s: read_byte beyond end of buffer\n", __func__);
	return ERR_BYTE;
}

/*
 * Read from the IFC Controller Data Buffer
 */
static void fsl_ifc_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	int avail;

	if (len < 0)
		return;

	avail = min((unsigned int)len,
			ifc_nand_ctrl->read_bytes - ifc_nand_ctrl->index);
	memcpy_fromio(buf, &ifc_nand_ctrl->addr[ifc_nand_ctrl->index], avail);
	ifc_nand_ctrl->index += avail;

	if (len > avail)
		dev_err(priv->dev,
			"%s: read_buf beyond end of buffer "
			"(%d requested, %d available)\n",
			__func__, len, avail);
}

/*
 * Verify buffer against the IFC Controller Data Buffer
 */
static int fsl_ifc_verify_buf(struct mtd_info *mtd,
			       const u_char *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	int i;

	if (len < 0) {
		dev_err(priv->dev, "%s: write_buf of %d bytes", __func__, len);
		return -EINVAL;
	}

	if ((unsigned int)len >
			ifc_nand_ctrl->read_bytes - ifc_nand_ctrl->index) {
		dev_err(priv->dev,
			"%s: verify_buf beyond end of buffer "
			"(%d requested, %u available)\n", __func__,
		       len, ifc_nand_ctrl->read_bytes - ifc_nand_ctrl->index);

		ifc_nand_ctrl->index = ifc_nand_ctrl->read_bytes;
		return -EINVAL;
	}

	for (i = 0; i < len; i++)
		if (in_8(&ifc_nand_ctrl->addr[ifc_nand_ctrl->index + i]) !=
									buf[i])
			break;

	ifc_nand_ctrl->index += len;

	if (i != len)
		dev_err(priv->dev, "%s:%d: %d != %d\n", __func__, __LINE__, i, len);
	if (ifc_nand_ctrl->status != IFC_NAND_EVTER_STAT_OPC)
		dev_err(priv->dev, "%s:%d: x%x != x%x\n", __func__, __LINE__, \
				ifc_nand_ctrl->status, IFC_NAND_EVTER_STAT_OPC);

	return i == len && ifc_nand_ctrl->status == IFC_NAND_EVTER_STAT_OPC ?
			0 : -EIO;
}

/*
 * This function is called after Program and Erase Operations to
 * check for success or failure.
 */
static int fsl_ifc_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	u32 nand_fsr;

	/* Use READ_STATUS command, but wait for the device to be ready */
	ifc_nand_ctrl->use_mdr = 0;
	out_be32(&ifc->ifc_nand.nand_fir0,
		 (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
		 (IFC_FIR_OP_RDSTAT << IFC_NAND_FIR0_OP1_SHIFT));
	out_be32(&ifc->ifc_nand.nand_fcr0, NAND_CMD_STATUS <<
			IFC_NAND_FCR0_CMD0_SHIFT);
	out_be32(&ifc->ifc_nand.nand_fbcr, 1);
	set_addr(mtd, 0, 0, 0);
	ifc_nand_ctrl->read_bytes = 1;

	fsl_ifc_run_command(mtd);

	nand_fsr = in_be32(&ifc->ifc_nand.nand_fsr);

	/*
	 * The chip always seems to report that it is
	 * write-protected, even when it is not.
	 */
	return nand_fsr | NAND_STATUS_WP;
}

static int fsl_ifc_read_page(struct mtd_info *mtd,
			      struct nand_chip *chip,
			      uint8_t *buf, int page)
{
	fsl_ifc_read_buf(mtd, buf, mtd->writesize);
	fsl_ifc_read_buf(mtd, chip->oob_poi, mtd->oobsize);

	if (fsl_ifc_wait(mtd, chip) & NAND_STATUS_FAIL)
		mtd->ecc_stats.failed++;

	return 0;
}

/* ECC will be calculated automatically, and errors will be detected in
 * waitfunc.
 */
static void fsl_ifc_write_page(struct mtd_info *mtd,
				struct nand_chip *chip,
				const uint8_t *buf)
{
	fsl_ifc_write_buf(mtd, buf, mtd->writesize);
	fsl_ifc_write_buf(mtd, chip->oob_poi, mtd->oobsize);

	ifc_nand_ctrl->oob_poi = chip->oob_poi;
}

/* Spansion S34ML01G2 */
#define SPANSION_S34ML01G2_NAND_DEVID_0x01F1 0x01F1

static int fsl_ifc_read_special(struct mtd_info *mtd, struct mtd_special_info *rsi, void *buf)
{
	struct nand_chip *chip = (struct nand_chip*) mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	int x;

	nand_get_device_exp(chip, mtd, FL_READING);

	/*
		Okay... We have had issues with the SPANSION_S34ML01G2_NAND_DEVID_0x01F1
		FLASH device and reading its FSN. The first read of the FSN always returns
		zeros but does not fail the read. Sending a command reset to the pipeline
		seems to leave the FLASH device in a state where we can read the FSN properly.
		This is a workaround until we can determine through Spansion what truly needs
		to happen. Please see PLAYER-6148 for more information and SYSSW-860 for the
		on going investigation. UPDATE: PLAYER-6328 now contains root cause.
		The reset before reading FSN must stay.
	*/
	if (SPANSION_S34ML01G2_NAND_DEVID_0x01F1 == mtd->devid) {
		fsl_ifc_cmdfunc(mtd, NAND_CMD_RESET, 0, 0);
		fsl_ifc_wait(mtd, chip);
	}

	memset(buf, 0xff, rsi->datalen);

	if (rsi->preop_cmdlen) {
		for (x = 0; x < (rsi->preop_cmdlen); x++) {
			out_be32(&ifc->ifc_nand.nand_fir0,
				IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT);
			out_be32(&ifc->ifc_nand.nand_fcr0,
				rsi->preop_cmd[x] << IFC_NAND_FCR0_CMD0_SHIFT);
			fsl_ifc_run_command(mtd);
		}
	}

	if (rsi->alternate_read) {
	    /* AJ: Hi There!
	       Quick explanation:
	       Send alternate read command to nand chip, as though you were
	       sending a normal read command.
	       Then read 256 bytes, your special data should be in there.
	    */
	    set_addr(mtd, 0, 0, 0);

	    out_be32(&ifc->ifc_nand.nand_fir0,
		     (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
		     (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP1_SHIFT) |
		     (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP2_SHIFT) |
		     (IFC_FIR_OP_CMD1 << IFC_NAND_FIR0_OP3_SHIFT) |
		     (IFC_FIR_OP_RBCD << IFC_NAND_FIR0_OP4_SHIFT));
	    out_be32(&ifc->ifc_nand.nand_fir1, 0x0);		  
	    out_be32(&ifc->ifc_nand.nand_fcr0,
		     (rsi->alternate_read << IFC_NAND_FCR0_CMD0_SHIFT) |
		     (NAND_CMD_READSTART << IFC_NAND_FCR0_CMD1_SHIFT));
	    out_be32(&ifc->ifc_nand.nand_fbcr, rsi->datalen);
	    ifc_nand_ctrl->read_bytes = rsi->datalen;
	    fsl_ifc_run_command(mtd);
	} else {
	    set_addr(mtd, 0, 0, 0);
	    fsl_ifc_cmdfunc(mtd, NAND_CMD_READ0, 0, 0);
	}
	fsl_ifc_read_buf(mtd, buf, (rsi->datalen>256)?256:rsi->datalen);
	fsl_ifc_cmdfunc(mtd, NAND_CMD_RESET, 0, 0);
	fsl_ifc_wait(mtd, chip);

	nand_release_device_exp(mtd);

	return 0;
}

static int fsl_ifc_chip_init_tail(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_ifc_mtd *priv = chip->priv;
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	uint32_t csor;

	dev_dbg(priv->dev, "%s: nand->numchips = %d\n", __func__,
							chip->numchips);
	dev_dbg(priv->dev, "%s: nand->chipsize = %lld\n", __func__,
							chip->chipsize);
	dev_dbg(priv->dev, "%s: nand->pagemask = %8x\n", __func__,
							chip->pagemask);
	dev_dbg(priv->dev, "%s: nand->chip_delay = %d\n", __func__,
							chip->chip_delay);
	dev_dbg(priv->dev, "%s: nand->badblockpos = %d\n", __func__,
							chip->badblockpos);
	dev_dbg(priv->dev, "%s: nand->chip_shift = %d\n", __func__,
							chip->chip_shift);
	dev_dbg(priv->dev, "%s: nand->page_shift = %d\n", __func__,
							chip->page_shift);
	dev_dbg(priv->dev, "%s: nand->phys_erase_shift = %d\n", __func__,
							chip->phys_erase_shift);
	dev_dbg(priv->dev, "%s: nand->ecclayout = %p\n", __func__,
							chip->ecclayout);
	dev_dbg(priv->dev, "%s: nand->ecc.mode = %d\n", __func__,
							chip->ecc.mode);
	dev_dbg(priv->dev, "%s: nand->ecc.steps = %d\n", __func__,
							chip->ecc.steps);
	dev_dbg(priv->dev, "%s: nand->ecc.bytes = %d\n", __func__,
							chip->ecc.bytes);
	dev_dbg(priv->dev, "%s: nand->ecc.total = %d\n", __func__,
							chip->ecc.total);
	dev_dbg(priv->dev, "%s: nand->ecc.layout = %p\n", __func__,
							chip->ecc.layout);
	dev_dbg(priv->dev, "%s: mtd->flags = %08x\n", __func__, mtd->flags);
	dev_dbg(priv->dev, "%s: mtd->size = %lld\n", __func__, mtd->size);
	dev_dbg(priv->dev, "%s: mtd->erasesize = %d\n", __func__,
							mtd->erasesize);
	dev_dbg(priv->dev, "%s: mtd->writesize = %d\n", __func__,
							mtd->writesize);
	dev_dbg(priv->dev, "%s: mtd->oobsize = %d\n", __func__,
							mtd->oobsize);

	csor = in_be32(&ifc->csor_cs[priv->bank].csor);

	/* Change ECC setup for Large Page devices */
	if (mtd->writesize == 2048) {
		if ((csor & CSOR_NAND_ECC_ENC_EN) ||
			(csor & CSOR_NAND_ECC_DEC_EN)) {
			chip->ecc.steps = 4;
			if (csor & CSOR_NAND_ECC_MODE_8) {
				chip->ecc.layout = &fsl_ifc_oob_lp_2k_eccm1;
				chip->ecc.bytes = 16;
			} else {
				chip->ecc.layout = &fsl_ifc_oob_lp_2k_eccm0;
				chip->ecc.bytes = 8;
				priv->bufnum_mask = 3;
			}
		}
	} else if (mtd->writesize == 4096) {
		if ((csor & CSOR_NAND_ECC_ENC_EN) ||
			(csor & CSOR_NAND_ECC_DEC_EN)) {
			chip->ecc.steps = 8;
				chip->ecc.layout = &fsl_ifc_oob_lp_4k_eccm0;
				chip->ecc.bytes = 8;
		}
	}
	return 0;
}

static int fsl_ifc_chip_init(struct fsl_ifc_mtd *priv)
{
	struct fsl_ifc_ctrl *ctrl = priv->ctrl;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;
	struct nand_chip *chip = &priv->chip;

	dev_info(priv->dev, "IFC Set Information for bank %d\n", priv->bank);

	/* Fill in fsl_ifc_mtd structure */
	priv->mtd.priv = chip;
	priv->mtd.owner = THIS_MODULE;

	/* fill in nand_chip structure */
	/* set up function call table */
	if ((in_be32(&ifc->cspr_cs[priv->bank].cspr)) & CSPR_PORT_SIZE_16)
		chip->read_byte = fsl_ifc_read_byte16;
	else
		chip->read_byte = fsl_ifc_read_byte;

	chip->write_buf = fsl_ifc_write_buf;
	chip->read_buf = fsl_ifc_read_buf;
	chip->verify_buf = fsl_ifc_verify_buf;
	chip->select_chip = fsl_ifc_select_chip;
	chip->cmdfunc = fsl_ifc_cmdfunc;
	chip->waitfunc = fsl_ifc_wait;

	chip->bbt_td = &bbt_main_descr;
	chip->bbt_md = &bbt_mirror_descr;

	/* set up nand options */
#ifdef CONFIG_MTD_NAND_SONOS_VNB_MAPPING
	chip->options = NAND_NO_READRDY | NAND_NO_AUTOINCR;
#else
	chip->options = NAND_NO_READRDY | NAND_NO_AUTOINCR | NAND_USE_FLASH_BBT;
#endif

	chip->controller = &ifc_nand_ctrl->controller;
	chip->priv = priv;

	chip->ecc.read_page = fsl_ifc_read_page;
	chip->ecc.write_page = fsl_ifc_write_page;

	/* If Chip select selects full hardware ECC then use it */
	if (((in_be32(&ifc->csor_cs[priv->bank].csor))
				& CSOR_NAND_ECC_ENC_EN)
		|| ((in_be32(&ifc->csor_cs[priv->bank].csor)
				& CSOR_NAND_ECC_DEC_EN))) {
		chip->ecc.mode = NAND_ECC_HW;

		chip->ecc.size = 512;
		chip->ecc.steps = 1;
		chip->ecc.layout = &fsl_ifc_oob_sp_eccm0;
		chip->ecc.bytes = 8;
	} else {
		/* otherwise fall back to default software ECC */
		chip->ecc.mode = NAND_ECC_SOFT;
	}

	return 0;
}

static int fsl_ifc_chip_remove(struct fsl_ifc_mtd *priv)
{
	nand_release(&priv->mtd);

	kfree(priv->mtd.name);

	if (priv->vbase)
		iounmap(priv->vbase);

	ifc_nand_ctrl->chips[priv->bank] = NULL;
	kfree(priv);

	return 0;
}

/*
 * This interrupt is used to report ifc nand events of various kinds,
 * such as transaction complete, errors on the chipselects.
 */
static irqreturn_t fsl_ifc_nand_irq(int irqno, void *data)
{
	struct fsl_ifc_ctrl *ctrl = data;
	struct fsl_ifc_regs __iomem *ifc = ctrl->regs;

	/* disable the Interrupts */
	out_be32(&ifc->ifc_nand.nand_evter_intr_en, 0x0);

	ifc_nand_ctrl->status =	in_be32(&ifc->ifc_nand.nand_evter_stat);

	if (ifc_nand_ctrl->status != IFC_NAND_EVTER_STAT_OPC)
		ifc_nand_ctrl->status = ctrl->status;

	/* clear status events for NAND Machine */
	out_be32(&ifc->ifc_nand.nand_evter_stat, ifc_nand_ctrl->status);

	/* wake up */
	wake_up(&ifc_nand_ctrl->irq_wait);

	return IRQ_HANDLED;
}


static int __devinit fsl_ifc_nand_probe(struct of_device *dev,
					 const struct of_device_id *match)
{
	struct fsl_ifc_regs __iomem *ifc;
	struct fsl_ifc_mtd *priv;
	struct resource res;
#ifdef CONFIG_MTD_PARTITIONS
	static const char *part_probe_types[]
		= { "cmdlinepart", "RedBoot", NULL };
	struct mtd_partition *parts;
#endif
	int ret;
	int bank;
	struct device_node *node = dev->dev.of_node;
	struct nand_chip *chip;
	struct mtd_info *virt_mtd;

	dev_info(&dev->dev, "Freescale IFC NAND Machine Driver\n");

	if (!fsl_ifc_ctrl_dev || !fsl_ifc_ctrl_dev->regs)
		return -ENODEV;
	ifc = fsl_ifc_ctrl_dev->regs;

	/* get, allocate and map the memory resource */
	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		dev_err(fsl_ifc_ctrl_dev->dev, "%s: failed to get resource\n",
								__func__);
		return ret;
	}
	dev_info(&dev->dev, "resource gotten %x.\n", ret);

	/* find which chip select it is connected to */
	for (bank = 0; bank < FSL_IFC_BANK_COUNT; bank++) {
		if ((in_be32(&ifc->cspr_cs[bank].cspr) & CSPR_V) &&
			((in_be32(&ifc->cspr_cs[bank].cspr) & CSPR_MSEL)
							== CSPR_MSEL_NAND) &&
			(in_be32(&ifc->cspr_cs[bank].cspr) & CSPR_BA)
			== convert_ifc_address(res.start))
			break;
	}

	if (bank >= FSL_IFC_BANK_COUNT) {
		dev_err(fsl_ifc_ctrl_dev->dev, "%s: address did not match any "
			"chip selects\n", __func__);
		return -ENODEV;
	}                                                
	dev_info(&dev->dev, "chip select found %i.\n", bank);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (fsl_ifc_ctrl_dev->nand == NULL) {
		ifc_nand_ctrl = kzalloc(sizeof(*ifc_nand_ctrl), GFP_KERNEL);
		if (!ifc_nand_ctrl)
			return -ENOMEM;

		ifc_nand_ctrl->read_bytes = 0;
		ifc_nand_ctrl->index = 0;
		ifc_nand_ctrl->addr = NULL;
		fsl_ifc_ctrl_dev->nand = ifc_nand_ctrl;

		spin_lock_init(&ifc_nand_ctrl->controller.lock);
		init_waitqueue_head(&ifc_nand_ctrl->controller.wq);
	}

	init_waitqueue_head(&ifc_nand_ctrl->irq_wait);

	ifc_nand_ctrl->chips[bank] = priv;
	priv->bank = bank;
	priv->ctrl = fsl_ifc_ctrl_dev;
	priv->dev = fsl_ifc_ctrl_dev->dev;

	priv->vbase = ioremap(res.start, resource_size(&res));
	if (!priv->vbase) {
		dev_err(fsl_ifc_ctrl_dev->dev, "%s: failed to map chip"
						"region\n", __func__);
		ret = -ENOMEM;
		goto err;
	}
	dev_info(&dev->dev, "chip mapped %lx.\n", (unsigned long)priv->vbase);

	out_be32(&ifc->ifc_nand.nand_evter_en,
			IFC_NAND_EVTER_EN_OPC_EN |
			IFC_NAND_EVTER_EN_FTOER_EN |
			IFC_NAND_EVTER_EN_WPER_EN);

	/* enable NAND Machine Interrupts */
	out_be32(&ifc->ifc_nand.nand_evter_intr_en,
			IFC_NAND_EVTER_INTR_OPCIR_EN |
			IFC_NAND_EVTER_INTR_FTOERIR_EN |
			IFC_NAND_EVTER_INTR_WPERIR_EN |
			IFC_NAND_EVTER_INTR_ECCERIR_EN);

	ret = request_irq(fsl_ifc_ctrl_dev->nand_irq, fsl_ifc_nand_irq, 0,
				"fsl-ifc-nand", priv->ctrl);

	if (ret != 0) {
		dev_err(fsl_ifc_ctrl_dev->dev, "%s: failed to install"
			" irq (%d)\n",
			__func__, fsl_ifc_ctrl_dev->nand_irq);
		goto err;
	}
	dev_info(&dev->dev, "got irq %i.\n", fsl_ifc_ctrl_dev->nand_irq);

	priv->mtd.name = kasprintf(GFP_KERNEL, "%x.flash", (unsigned)res.start);
	if (!priv->mtd.name) {
		ret = -ENOMEM;
		goto err;
	}

	ret = fsl_ifc_chip_init(priv);
	if (ret)
		goto err;

	ret = nand_scan_ident(&priv->mtd, 1, NULL);
	if (ret)
		goto err;

	ret = fsl_ifc_chip_init_tail(&priv->mtd);
	if (ret)
		goto err;

#ifdef CONFIG_MTD_NAND_SONOS_VNB_MAPPING
	virt_mtd = vnb_init(&priv->mtd);
	if(virt_mtd == NULL)
		return -ENOMEM;
#else
		virt_mtd = &priv->mtd;
#endif

	chip = &priv->chip;

	priv->mtd.read_special = fsl_ifc_read_special;

	ret = nand_scan_tail(&priv->mtd);
	if (ret)
		goto err;

#ifdef CONFIG_MTD_PARTITIONS
	/* First look for RedBoot table or partitions on the command
	 * line, these take precedence over device tree information */
	//ret = parse_mtd_partitions(&priv->mtd, part_probe_types, &parts, 0);
	ret = parse_mtd_partitions(&priv->mtd, part_probe_types, &parts, 0);
	if (ret < 0)
		goto err;

#ifdef CONFIG_MTD_OF_PARTS
	if (ret == 0) {
		ret = of_mtd_parse_partitions(priv->dev, node, &parts);
		if (ret < 0)
			goto err;
	}
#endif

#ifdef CONFIG_MTD_NAND_SONOS_VNB_MAPPING
	vnb_initMtd(virt_mtd);
	printk("Sonos VNB Mapping init complete.\n");
#endif

	if (ret > 0)
		add_mtd_partitions(virt_mtd, parts, ret);
#endif
	add_mtd_device(virt_mtd);

	printk(KERN_INFO "IFC NAND device at 0x%llx, bank %d\n",
	       (unsigned long long)res.start, priv->bank);
	return 0;

err:
	fsl_ifc_chip_remove(priv);
	return ret;
}

static int fsl_ifc_nand_remove(struct of_device *ofdev)
{
	int i;

	for (i = 0; i < FSL_IFC_BANK_COUNT; i++)
		if (ifc_nand_ctrl->chips[i])
			fsl_ifc_chip_remove(ifc_nand_ctrl->chips[i]);

	fsl_ifc_ctrl_dev->nand = NULL;
	kfree(ifc_nand_ctrl);
	return 0;
}

static const struct of_device_id fsl_ifc_nand_match[] = {
	{
		.compatible = "fsl,ifc-nand",
	},
	{}
};

static struct of_platform_driver fsl_ifc_nand_driver = {
	.driver = {
		.name	= "fsl,ifc-nand",
		.owner = THIS_MODULE,
		.of_match_table = fsl_ifc_nand_match,
	},
	.probe       = fsl_ifc_nand_probe,
	.remove      = fsl_ifc_nand_remove,
};

static int __init fsl_ifc_nand_init(void)
{
	int ret;

	ret = of_register_platform_driver(&fsl_ifc_nand_driver);
	if (ret)
		printk(KERN_ERR "fsl-ifc: Failed to register platform"
				"driver\n");

	return ret;
}

static void __exit fsl_ifc_nand_exit(void)
{
	of_unregister_platform_driver(&fsl_ifc_nand_driver);
}

module_init(fsl_ifc_nand_init);
module_exit(fsl_ifc_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale");
MODULE_DESCRIPTION("Freescale Integrated Flash Controller MTD NAND driver");
