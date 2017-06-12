/* Core MMC driver functions
 *
 * Copyright (c) 2002 Hewlett-Packard Company
 *   
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *  
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *  
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Author:  Andrew Christian
 *          6 May 2002 

 *(C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 */

/*
 * mmc_protocol.c - MMC protocol driver
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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/sysctl.h>
#include <linux/mm.h>
#include <asm/scatterlist.h>

#include <linux/mmc/mss_core.h>
#include <linux/mmc/mmc_protocol.h>

/* internal functions */

#define KBPS 1
#define MBPS 1000

static u32 ts_exp[] = { 100*KBPS, 1*MBPS, 10*MBPS, 100*MBPS, 0, 0, 0, 0 };
static u32 ts_mul[] = { 0,    1000, 1200, 1300, 1500, 2000, 2500, 3000, 
			3500, 4000, 4500, 5000, 5500, 6000, 7000, 8000 };

static u32 mmc_tran_speed(u8 ts)
{
	u32 clock = ts_exp[(ts & 0x7)] * ts_mul[(ts & 0x78) >> 3];

	dbg("MMC clock is %d\n", clock);
	return clock;
}

static int mmc_unpack_r1(struct mss_cmd *cmd, struct mmc_response_r1 *r1, struct mmc_card *mmc_card)
{
	u8 *buf = cmd->response;

	mmc_card->errno = MMC_ERROR_NONE;
	r1->cmd    = unstuff_bits(buf, 40, 8, 6);
	r1->status = unstuff_bits(buf, 8, 32, 6);

	if (R1_STATUS(r1->status)) {
		if (r1->status & R1_OUT_OF_RANGE)
			mmc_card->errno = MMC_ERROR_OUT_OF_RANGE;
		if (r1->status & R1_ADDRESS_ERROR)
		  	mmc_card->errno = MMC_ERROR_ADDRESS;
		if (r1->status & R1_BLOCK_LEN_ERROR)
		    	mmc_card->errno = MMC_ERROR_BLOCK_LEN;
		if (r1->status & R1_ERASE_SEQ_ERROR)
		    	mmc_card->errno = MMC_ERROR_ERASE_SEQ;
		if (r1->status & R1_ERASE_PARAM)
			mmc_card->errno = MMC_ERROR_ERASE_PARAM;
		if (r1->status & R1_WP_VIOLATION)
		 	mmc_card->errno = MMC_ERROR_WP_VIOLATION;
		if (r1->status & R1_LOCK_UNLOCK_FAILED)
			mmc_card->errno = MMC_ERROR_LOCK_UNLOCK_FAILED;
		if (r1->status & R1_COM_CRC_ERROR)
		  	mmc_card->errno = MMC_ERROR_COM_CRC;
		if (r1->status & R1_ILLEGAL_COMMAND)
		    	mmc_card->errno = MMC_ERROR_ILLEGAL_COMMAND;
		if (r1->status & R1_CARD_ECC_FAILED)
		    	mmc_card->errno = MMC_ERROR_CARD_ECC_FAILED;
		if (r1->status & R1_CC_ERROR)
	     		mmc_card->errno = MMC_ERROR_CC;
		if (r1->status & R1_ERROR)
	  		mmc_card->errno = MMC_ERROR_GENERAL;
		if (r1->status & R1_UNDERRUN)
	     		mmc_card->errno = MMC_ERROR_UNDERRUN;
		if (r1->status & R1_OVERRUN)
      			mmc_card->errno = MMC_ERROR_OVERRUN;
		if (r1->status & R1_CID_CSD_OVERWRITE)
		      	mmc_card->errno = MMC_ERROR_CID_CSD_OVERWRITE;
	}

	if (buf[0] != cmd->opcode)
	       	mmc_card->errno = MMC_ERROR_HEADER_MISMATCH;
	/* This should be last - it's the least dangerous error */
	if (R1_CURRENT_STATE(r1->status) != mmc_card->state ) {
		dbg("state dismatch:r1->status:%x,state:%x\n",
				R1_CURRENT_STATE(r1->status),mmc_card->state);
		mmc_card->errno = MMC_ERROR_STATE_MISMATCH;
	}
	dbg("mmc card error %d", mmc_card->errno);
	if (mmc_card->errno)
		return MSS_ERROR_RESP_UNPACK;
	return 0;
}

static int mmc_unpack_r3(struct mss_cmd *cmd, struct mmc_response_r3 *r3, struct mmc_card *mmc_card)
{
	u8 *buf = cmd->response;

	mmc_card->errno = MMC_ERROR_NONE;
	r3->cmd = unstuff_bits(buf, 40, 8, 6);
	r3->ocr = unstuff_bits(buf, 8, 32, 6);

	if (r3->cmd != 0x3f) {  
		mmc_card->errno = MMC_ERROR_HEADER_MISMATCH;
		return MSS_ERROR_RESP_UNPACK;
	}
	return 0;
}

/**
 *  Fast I/O, support for CEATA devices.
 */
static int mmc_unpack_r4( struct mss_cmd *cmd, struct mmc_response_r4 *r4, struct mmc_card *mmc_card)
{
	u8 *buf = cmd->response;

	mmc_card->errno = MMC_ERROR_NONE;

	r4->cmd			= unstuff_bits(buf, 40, 8, 6);
	r4->rca			= unstuff_bits(buf, 24, 16, 6);
	r4->status		= unstuff_bits(buf, 23, 1, 6);
	r4->reg_addr		= unstuff_bits(buf, 16, 7, 6);
	r4->read_reg_contents	= unstuff_bits(buf, 8, 8, 6);
	
	if (r4->cmd != 0x27) {
		mmc_card->errno = MMC_ERROR_HEADER_MISMATCH;
		return MSS_ERROR_RESP_UNPACK;
	}
	return 0 ;
}

/**
 *  Interrupt request. not supported temporarily.
 */
#if 0
static int mmc_unpack_r5(struct mss_cmd *cmd, struct mmc_response_r5 *r5, struct mmc_card *mmc_card)
{
	u8 *buf = cmd->response;

	mmc_card->errno = MMC_ERROR_NONE;
	
	r5->cmd		= unstuff_bits(buf, 40, 8, 6);
	r5->rca		= unstuff_bits(buf, 24, 16, 6);
	r5->irq_data	= unstuff_bits(buf, 8, 16, 6);
	
	if (r5->cmd != 0x28) {
		mmc_card->errno = MMC_ERROR_HEADER_MISMATCH;
		return MSS_ERROR_RESP_UNPACK;
	}
	return 0 ;
}
#endif

static int mmc_unpack_cid(struct mss_cmd *cmd, struct mmc_cid *cid, struct mmc_card *mmc_card)
{
	u8 *buf = cmd->response;

	mmc_card->errno = MMC_ERROR_NONE;
	if (buf[0] != 0x3f)  {
	       mmc_card->errno = MMC_ERROR_HEADER_MISMATCH;
	       return MSS_ERROR_RESP_UNPACK;
	}
	buf = buf + 1;
	
	cid->mid	= unstuff_bits(buf, 120, 8, 16);
	cid->oid	= unstuff_bits(buf, 104, 16, 16);
	cid->pnm[0]	= unstuff_bits(buf, 96, 8, 16);
	cid->pnm[1]	= unstuff_bits(buf, 88, 8, 16);
	cid->pnm[2]	= unstuff_bits(buf, 80, 8, 16);
	cid->pnm[3]	= unstuff_bits(buf, 72, 8, 16);
	cid->pnm[4]	= unstuff_bits(buf, 64, 8, 16);
	cid->pnm[5]	= unstuff_bits(buf, 56, 8, 16);	
	cid->pnm[6]	= 0;
	cid->prv	= unstuff_bits(buf, 48, 8, 16);
	cid->psn	= unstuff_bits(buf, 16, 32, 16);
	cid->mdt	= unstuff_bits(buf, 8, 8, 16);
	
#if 0
	DEBUG(" mid=%d oid=%d pnm=%s prv=%d.%d psn=%08x mdt=%d/%d\n",
	      cid->mid, cid->oid, cid->pnm, 
	      (cid->prv>>4), (cid->prv&0xf), 
	      cid->psn, cid->mdt&&0xf, ((cid->mdt>>4)&0xff)+2000);
#endif
      	return 0;
}

static int mmc_unpack_csd(struct mss_cmd *cmd, struct mmc_csd *csd, struct mmc_card *mmc_card)
{
	u8 *buf = cmd->response;
	
	mmc_card->errno = MMC_ERROR_NONE;
	if (buf[0] != 0x3f)  {
		mmc_card->errno = MMC_ERROR_HEADER_MISMATCH;
		return MSS_ERROR_RESP_UNPACK;
	}
	buf = buf + 1;
	
	csd->csd_structure      = unstuff_bits(buf, 126, 2, 16);
	csd->spec_vers          = unstuff_bits(buf, 122, 4, 16);
	csd->taac               = unstuff_bits(buf, 112, 8, 16);
	csd->nsac               = unstuff_bits(buf, 104, 8, 16);
	csd->tran_speed         = unstuff_bits(buf, 96, 8, 16);
	csd->ccc                = unstuff_bits(buf, 84, 12, 16);;
	csd->read_bl_len        = unstuff_bits(buf, 80, 4, 16);
	csd->read_bl_partial    = unstuff_bits(buf, 79, 1, 16);
	csd->write_blk_misalign = unstuff_bits(buf, 78, 1, 16);
	csd->read_blk_misalign  = unstuff_bits(buf, 77, 1, 16);
	csd->dsr_imp            = unstuff_bits(buf, 76, 1, 16);
	csd->c_size             = unstuff_bits(buf, 62, 12, 16);
	csd->vdd_r_curr_min     = unstuff_bits(buf, 59, 3, 16);
	csd->vdd_r_curr_max     = unstuff_bits(buf, 56, 3, 16);
	csd->vdd_w_curr_min     = unstuff_bits(buf, 53, 3, 16);
	csd->vdd_w_curr_max     = unstuff_bits(buf, 50, 3, 16);
	csd->c_size_mult        = unstuff_bits(buf, 47, 3, 16);
	switch (csd->csd_structure ) {
		case CSD_STRUCT_1_0:
		case CSD_STRUCT_1_1:
			csd->erase.v22.sector_size    = 
				unstuff_bits(buf, 42, 5, 16);
			csd->erase.v22.erase_grp_size = 
				unstuff_bits(buf, 37, 5, 6);
		break;
		case CSD_STRUCT_1_2:
		default:
			csd->erase.v31.erase_grp_size = 
				unstuff_bits(buf, 42, 5, 16);
			csd->erase.v31.erase_grp_mult = 
				unstuff_bits(buf, 37, 5, 16);;
		break;
	}
	csd->wp_grp_size        = unstuff_bits(buf, 32, 5, 16);
	csd->wp_grp_enable      = unstuff_bits(buf, 31, 1, 16);
	csd->default_ecc        = unstuff_bits(buf, 29, 2, 16);
	csd->r2w_factor         = unstuff_bits(buf, 26, 3, 16);
	csd->write_bl_len       = unstuff_bits(buf, 22, 4, 16);
	csd->write_bl_partial   = unstuff_bits(buf, 21, 1, 16);
	csd->file_format_grp    = unstuff_bits(buf, 16, 1, 16);
	csd->copy               = unstuff_bits(buf, 14, 1, 16);
	csd->perm_write_protect = unstuff_bits(buf, 13, 1, 16);
	csd->tmp_write_protect  = unstuff_bits(buf, 12, 1, 16);
	csd->file_format        = unstuff_bits(buf, 10, 2, 16);
	csd->ecc                = unstuff_bits(buf, 8, 2, 16);
#if 0
	printk("csd_structure=%d  spec_vers=%d  taac=%02x  nsac=%02x  tran_speed=%02x\n"
	      "  ccc=%04x  read_bl_len=%d  read_bl_partial=%d  write_blk_misalign=%d\n"
	      "  read_blk_misalign=%d  dsr_imp=%d  c_size=%d  vdd_r_curr_min=%d\n"
	      "  vdd_r_curr_max=%d  vdd_w_curr_min=%d  vdd_w_curr_max=%d  c_size_mult=%d\n"
	      "  wp_grp_size=%d  wp_grp_enable=%d  default_ecc=%d  r2w_factor=%d\n"
	      "  write_bl_len=%d  write_bl_partial=%d  file_format_grp=%d  copy=%d\n"
	      "  perm_write_protect=%d  tmp_write_protect=%d  file_format=%d  ecc=%d\n",
	      csd->csd_structure, csd->spec_vers, 
	      csd->taac, csd->nsac, csd->tran_speed,
	      csd->ccc, csd->read_bl_len, 
	      csd->read_bl_partial, csd->write_blk_misalign,
	      csd->read_blk_misalign, csd->dsr_imp, 
	      csd->c_size, csd->vdd_r_curr_min,
	      csd->vdd_r_curr_max, csd->vdd_w_curr_min, 
	      csd->vdd_w_curr_max, csd->c_size_mult,
	      csd->wp_grp_size, csd->wp_grp_enable,
	      csd->default_ecc, csd->r2w_factor, 
	      csd->write_bl_len, csd->write_bl_partial,
	      csd->file_format_grp, csd->copy, 
	      csd->perm_write_protect, csd->tmp_write_protect,
	      csd->file_format, csd->ecc);
	switch ( csd->csd_structure ) {
	case CSD_STRUCT_1_0:
	case CSD_STRUCT_1_1:
		printk(" V22 sector_size=%d erase_grp_size=%d\n", 
		      csd->erase.v22.sector_size, 
		      csd->erase.v22.erase_grp_size);
		break;
	case CSD_STRUCT_1_2:
	default:
		printk(" V31 erase_grp_size=%d erase_grp_mult=%d\n", 
		      csd->erase.v31.erase_grp_size,
		      csd->erase.v31.erase_grp_mult);
		break;
		
	}
#endif
	return MSS_ERROR_NONE;
}

static int mmc_unpack_ext_csd(u8 *buf, struct mmc_ext_csd *ext_csd)
{
	ext_csd->s_cmd_set = buf[504];
	ext_csd->sec_count = ((u32)buf[212]) << 24 | ((u32)buf[213]) << 16 | ((u32)buf[214]) << 8 | ((u32)buf[215]) ;
	ext_csd->min_perf_w_8_52 = buf[210];
	ext_csd->min_perf_r_8_52 = buf[209];
	ext_csd->min_perf_w_26_4_52 = buf[208];
	ext_csd->min_perf_r_26_4_52 = buf[207];
	ext_csd->min_perf_w_4_26 = buf[206];
	ext_csd->min_perf_r_4_26 = buf[205];
	ext_csd->pwr_cl_26_360 = buf[203];
	ext_csd->pwr_cl_52_360 = buf[202];
	ext_csd->pwr_cl_26_195 = buf[201];
	ext_csd->pwr_cl_52_195 = buf[200];
	ext_csd->card_type = buf[196];
	ext_csd->csd_structure = buf[194];
	ext_csd->ext_csd_rev = buf[192];
	ext_csd->cmd_set = buf[191];
	ext_csd->cmd_set_rev = buf[189];
	ext_csd->power_class = buf[187];
	ext_csd->hs_timing = buf[185];
	ext_csd->erased_mem_cont = buf[183];

#if 0
	DEBUG("s_cmd_set:%d, sec_count:%d, min_perf_w_8_52:%d,\n"
	"min_perf_r_8_52:%d, min_perf_w_26_4_52:%d, min_perf_r_26_4_52:%d\n"
	"min_perf_w_4_26:%d, min_perf_r_4_26:%d, pwr_cl_26_360:%d, "
	"pwr_cl_52_360:%d, pwr_cl_26_195:%d, pwr_cl_52_195:%d, card_type:%d\n"
	"csd_structure:%d, ext_csd_rev:%d, cmd_set:%d, cmd_set_rev:%d\n"
	"power_class:%d, hs_timing:%d, erased_mem_cont:%d\n", ext_csd->s_cmd_set,ext_csd->sec_count,ext_csd->min_perf_w_8_52, ext_csd->min_perf_r_8_52,ext_csd->min_perf_w_26_4_52,ext_csd->min_perf_r_26_4_52, ext_csd->min_perf_w_4_26, ext_csd->min_perf_r_4_26, ext_csd->pwr_cl_26_360, ext_csd->pwr_cl_52_360, ext_csd->pwr_cl_26_195, ext_csd->pwr_cl_52_195, ext_csd->card_type, ext_csd->csd_structure, ext_csd->ext_csd_rev, ext_csd->cmd_set, ext_csd->cmd_set_rev, ext_csd->power_class, ext_csd->hs_timing, ext_csd->erased_mem_cont); 
#endif
	return 0;
}

#ifdef CONFIG_MMC_CEATA
static int mmc_unpack_ceata(u8 *buf, struct mmc_ceata *ceata)
{
/*	memcpy(ceata->sn, buf + 20, 20);
	memcpy(ceata->fw_ver, buf + 46, 8);
	memcpy(ceata->model_number, buf + 54, 40);
	ceata->major_ver = buf[160] << 8 | buf[161];
	ceata->max_lba = ((u64)buf[200] << 56) | ((u64)buf[201] << 48) | 
		((u64)buf[202] << 40) | ((u64)buf[203] << 32) | 
		((u64)buf[204] << 24) | ((u64)buf[205] << 16) | 
		((u64)buf[206] << 8) | (u64)(buf[207]);
	ceata->feature = buf[512] << 8 | buf[513];

	dbg("max_lbr : 0x%llx", ceata->max_lba);*/
	return 0;
}
#endif
/**
 *  The blocks requested by the kernel may or may not match what we can do.  
 *  Unfortunately, filesystems play fast and loose with block sizes, so we're 
 * stuck with this .
 */
static void mmc_fix_request_block_len(struct mss_card *card, int action, struct mss_rw_arg *arg)
{
	u16 block_len = 0;
	struct mmc_card *mmc_card = card->prot_card;
	struct mss_host *host = card->slot->host;

	switch(action) {
		case MSS_DATA_READ:
			block_len = 1 << mmc_card->csd.read_bl_len;
			break;
		case MSS_DATA_WRITE:
			block_len = 1 << mmc_card->csd.write_bl_len;
			break;
		default:
			return;
	}
	if (host->high_capacity 
			&& mmc_card->access_mode == MMC_ACCESS_MODE_SECTOR
			&& card->card_type == MSS_CEATA_CARD)
		block_len = 512;	
	if (block_len < arg->block_len) {
		int scale = arg->block_len / block_len;
		arg->block_len	= block_len;
		arg->block *= scale;
		arg->nob *= scale;
	}
}


/*****************************************************************************
 *
 *   protocol entry functions
 *
 ****************************************************************************/

static u32 mmc_make_cmd6_arg(u8 access, u8 index, u8 value, u8 cmdset)
{
        u32 ret;
        ret = (access << 24) | (index << 16) | (value << 8) | cmdset;
        return ret;
}
#if 0
static int mmc_simple_ll_req(struct mss_host *host, struct mmc_card *mmc_card, u32 opcode, u32 arg, u32 rtype, u32 flags)
{
	memset(&mmc_card->llreq, 0x0, sizeof(struct mss_ll_request));
	memset(&mmc_card->cmd, 0x0, sizeof(struct mss_cmd));

	mmc_card->cmd.opcode = opcode;
	mmc_card->cmd.arg = arg;
	mmc_card->cmd.rtype = rtype;
	mmc_card->cmd.flags = flags;

	mmc_card->llreq.cmd = &mmc_card->cmd;

	return mss_send_ll_req(host, &mmc_card->llreq);
}	
#endif
static int mmc_get_status(struct mss_card *card, u32 *status)
{
	struct mmc_response_r1 r1;
	struct mmc_card *mmc_card = card->prot_card;
	struct mss_host *host = card->slot->host;
	struct mss_ll_request *llreq = &mmc_card->llreq;
	struct mss_cmd *cmd = &mmc_card->cmd;
	int clock, ret, retries = 4;

	clock = mmc_tran_speed(mmc_card->csd.tran_speed);
	mss_set_clock(card->slot->host, clock);
	while (retries--) {
		ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SEND_STATUS, 
				mmc_card->rca << 16, MSS_RESPONSE_R1, 0);
		if (ret && !retries)
			return ret;
		else if (!ret) {
			ret = mmc_unpack_r1(cmd, &r1, mmc_card);
			if (ret) {
				if (mmc_card->errno==MMC_ERROR_STATE_MISMATCH) {
					mmc_card->state = 
						R1_CURRENT_STATE(r1.status);
					mmc_card->errno = MMC_ERROR_NONE;
				}
				else
					return ret;
			}
			else 
				break;
		}

		clock = host->ios.clock;
		clock = clock >> 1;
		if (clock < MMC_CARD_CLOCK_SLOW || retries == 1)
			clock = MMC_CARD_CLOCK_SLOW;
		mss_set_clock(host, clock);
	}

	*status = r1.status;
	
	return MSS_ERROR_NONE; 
}

static int mmc_recognize_card(struct mss_card *card)
{
	struct mmc_response_r3 r3;
	struct mmc_card *mmc_card = card->prot_card;
	struct mss_ios ios;
	struct mss_host *host = card->slot->host;
	struct mss_ll_request *llreq = &mmc_card->llreq;
	struct mss_cmd *cmd = &mmc_card->cmd;
	int ret;
	u32 ocr = host->vdd;

	mmc_card->state = CARD_STATE_IDLE;
	card->bus_width = MSS_BUSWIDTH_1BIT;
	card->card_type = MSS_UNKNOWN_CARD;
	
	memcpy(&ios, &host->ios, sizeof(struct mss_ios)); 
	ios.bus_mode = MSS_BUSMODE_OPENDRAIN;
	ios.clock = host->f_min;
	ios.bus_width = MSS_BUSWIDTH_1BIT;
	host->ops->set_ios(host, &ios);

	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_GO_IDLE_STATE, 0, 
			MSS_RESPONSE_NONE, MSS_CMD_INIT); 
	/* dbg("Sending GO_IDLE cmd, ret:%d\n", ret); */
	if (ret)
		return ret;
	if (host->high_capacity)
		ocr |= MMC_ACCESS_MODE_SECTOR;
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SEND_OP_COND, ocr, 
			MSS_RESPONSE_R3, 0); 
	/* dbg("Sending SEND_OP_COND cmd, arg:0x%x\n, ret:%d", ocr, ret); */
	if (ret)
		return ret;
	
	ret = mmc_unpack_r3(cmd, &r3, mmc_card); 
	dbg("unapck ret %d, SEND_OP_COND ocr:0x%x", ret, r3.ocr);
	if (!ret) {
		if (r3.ocr & host->vdd) {
			card->card_type = MSS_MMC_CARD;
		} else {
			printk(KERN_WARNING "uncompatible card\n");
			card->card_type = MSS_UNCOMPATIBLE_CARD;	
		}
		return ret;
	}
	card->card_type = MSS_UNKNOWN_CARD;

	return MSS_ERROR_NONE;
}

static int mmc_card_init(struct mss_card *card)
{
	struct mmc_response_r3 r3;
	struct mmc_response_r1 r1;
	struct mmc_cid cid;
	struct mmc_card *mmc_card = card->prot_card;
	struct mss_ios ios;
	struct mss_host *host;
	struct mss_ll_request *llreq = &mmc_card->llreq;
	struct mss_cmd *cmd = &mmc_card->cmd;
	struct mss_data *data = &mmc_card->data;
	int ret;
	u8 *g_buffer;
	u32 ocr, arg;
	struct scatterlist sg;
#ifdef CONFIG_MMC_CEATA
	struct mmc_response_r4 r4;
	u32 status, retries = 10;
#endif
       
	host = card->slot->host;
	ocr = host->vdd;

	mmc_card->state = CARD_STATE_IDLE;
	card->bus_width = MSS_BUSWIDTH_1BIT;
	
	memcpy(&ios, &host->ios, sizeof(struct mss_ios)); 
	ios.bus_mode = MSS_BUSMODE_OPENDRAIN;
	ios.bus_width = MSS_BUSWIDTH_1BIT;
	ios.clock = host->f_min;
	host->ops->set_ios(host, &ios);

	/* dbg("Sending GO_IDLE cmd, ret:%d\n", ret); */
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_GO_IDLE_STATE, 0, 
			MSS_RESPONSE_NONE, MSS_CMD_INIT); 
	if (ret)
		return ret;

	if (host->high_capacity)
		ocr |= MMC_ACCESS_MODE_SECTOR;
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SEND_OP_COND, ocr, 
			MSS_RESPONSE_R3, 0); 
	/* dbg("Sending SEND_OP_COND cmd, arg:0x%x\n, ret:%d", ocr, ret); */
	if (ret)
		return ret;
	
	ret = mmc_unpack_r3(cmd, &r3, mmc_card);
	/* dbg("unapck ret %d, SEND_OP_COND ocr:0x%x", ret, r3.ocr); */
	if (ret) {
		return ret;
	}

	while (!(r3.ocr & MMC_CARD_BUSY)) {
		mdelay(20);
		ret = mss_send_simple_ll_req(host, llreq, cmd, 
				MMC_SEND_OP_COND, ocr, MSS_RESPONSE_R3, 0); 
		if (ret)
			return ret;
		ret = mmc_unpack_r3(cmd, &r3, mmc_card);
		if (ret) {
			return ret;
		}
	}

	mmc_card->vdd = r3.ocr & MMC_VDD_MASK;
	mmc_card->access_mode = r3.ocr & MMC_ACCESS_MODE_MASK;
	mmc_card->state = CARD_STATE_READY;
	
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_ALL_SEND_CID, 0, 
			MSS_RESPONSE_R2_CID, 0); 
	/* dbg("Sending MMC_ALL_SEND_CID cmd, arg:0x%x\n, ret:%d", 0, ret); */
	if (ret)
		return ret;

	memset(&cid, 0x0, sizeof(struct mmc_cid));
	ret = mmc_unpack_cid(cmd, &cid, mmc_card);
	if (ret)
		return ret;

	/* Not first init */
	if (mmc_card->cid.mid != 0) {
		/* Card is changed */
		if (mmc_card->cid.mid != cid.mid || mmc_card->cid.oid != cid.oid
				|| mmc_card->cid.prv != cid.prv 
				|| mmc_card->cid.psn != cid.psn
				|| mmc_card->cid.mdt != cid.mdt
				|| memcmp(mmc_card->cid.pnm, cid.pnm, 6))
			return MSS_ERROR_MISMATCH_CARD;
	}
	else 
		memcpy(&mmc_card->cid, &cid, sizeof(struct mmc_cid));
	
	mmc_card->state = CARD_STATE_IDENT;

	mss_set_busmode(host, MSS_BUSMODE_PUSHPULL);

	mmc_card->rca = MMC_SLOT_RCA(card->slot);
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SET_RELATIVE_ADDR, 
			mmc_card->rca << 16, MSS_RESPONSE_R1, 0); 
	/* dbg("Sending SET_REL_ADDR cmd, arg:0x%x\n, ret:%d", 
			mmc_card->rca << 16, ret); */
	if (ret)
		return ret;

    	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret) {
		return ret;
	}
  	mmc_card->state = CARD_STATE_STBY;

	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SEND_CSD, 
			mmc_card->rca << 16, MSS_RESPONSE_R2_CSD, 0);
	/* dbg("Sending MMC_SEND_CSD cmd, arg:0x%x\n, ret:%d", 
			mmc_card->rca << 16, ret); */
	if (ret)
		return ret;
	ret = mmc_unpack_csd(cmd, &mmc_card->csd, mmc_card);
	if (ret) {
		return ret;
	}
#ifdef CONFIG_MMC_CEATA
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_FAST_IO, 
			(mmc_card->rca << 16) + (CEATA_LBA_MID << 8), 
			MSS_RESPONSE_R4, 0);
	if (ret == MSS_ERROR_TIMEOUT)
		goto no_ceata;
	if (ret)
		return ret;
	ret = mmc_unpack_r4(cmd, &r4, mmc_card);
	if (ret) {
		return ret;
	}
	if (r4.read_reg_contents != 0xCE)
		goto no_ceata;
	
	card->card_type = MSS_CEATA_CARD;
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_FAST_IO, 
			(mmc_card->rca << 16) + (CEATA_LBA_HIGH << 8), 
			MSS_RESPONSE_R4, 0);
	if (ret)
		return ret;
	ret = mmc_unpack_r4(cmd, &r4, mmc_card);
	if (ret) {
		return ret;
	}
	if (r4.read_reg_contents != 0xAA) {
		return MSS_ERROR_WRONG_CARD_TYPE;
	}
	retries = 10;
	do {
		mdelay(200);
		ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_FAST_IO, 
				(mmc_card->rca << 16) + 
				(CEATA_COMMAND_STATUS << 8), 
				MSS_RESPONSE_R4, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r4(cmd, &r4, mmc_card);
		if (ret) {
			return ret;
		}
		status = r4.read_reg_contents;
	} while((status & 0x40) != 0x40 && retries--);
	if (!retries)
		return MSS_ERROR_TIMEOUT;

no_ceata:
#endif	
	/*
	 * if it is MMC4.x-compatible card and how many bits are working.
	 */
	if (mmc_card->csd.spec_vers != CSD_SPEC_VERS_4 
			|| host->mmc_spec != MSS_MMC_SPEC_40_42) 
		goto exit;

	g_buffer = mmc_card->buf;
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SELECT_CARD, 
			mmc_card->rca << 16, MSS_RESPONSE_R1B, 0);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	/* 
	 * If CEATA is enabled, no-CEATA card will get 
	 * MMC_ERROR_ILLEGAL_COMMAND because it recieves the FAST_IO command 
	 * that it does not support 
	 */
#ifdef CONFIG_MMC_CEATA
	if (ret && mmc_card->errno != MMC_ERROR_ILLEGAL_COMMAND)
#else
	if (ret)
#endif
		return ret;
	mmc_card->state = CARD_STATE_TRAN;

	/*
	 * set 1-bus mode in init. arg:access=0b11, arg:index=0xb7, arg:value=0
	 * or CMD8 will not be responded with 1-bit/controller and 4-bit/card
	 * dev->bus_mode should be MSS_1_BIT before next command
	 * buffer = NULL;
	 */
	arg = mmc_make_cmd6_arg(0x3, 0xb7, 0, 0);
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SWITCH, arg, 
			MSS_RESPONSE_R1B, 0);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;
	
	card->bus_width = MSS_BUSWIDTH_1BIT;
	mss_set_buswidth(host, MSS_BUSWIDTH_1BIT);

	sg.page = virt_to_page(g_buffer);
	sg.offset = offset_in_page(g_buffer);
	sg.length = 512;

	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	MSS_INIT_CMD(cmd, MMC_SEND_EXT_CSD, 0, 0, MSS_RESPONSE_R1); 
	MSS_INIT_DATA(data, 1, 512, MSS_DATA_READ, 1, &sg, 0); 
	llreq->cmd = cmd;
	llreq->data = data;

	ret = mss_send_ll_req(host, &mmc_card->llreq);
	if (ret)
		return ret;
	ret = mmc_unpack_ext_csd(g_buffer, &mmc_card->ext_csd);
	if (ret)
		return ret;

/* 
 * Monahans MMC controller does not support BUS_R and BUS_W command correctly,
 * so we direclty set the card bus width to be 4 bit 
 */	
#if 0
	/*
	 *  use CMD19/CMD14 (BUSTEST_W/BUSTEST_R) to test supported bus mode.
	 */
	memset(g_buffer, 0xFF, 512);
	bus_width = host->bus_width;
	if (bus_width == MSS_BUSWIDTH_1BIT)
		goto exit;
	else if (bus_width == MSS_BUSWIDTH_4BIT) {
		card->bus_width = MSS_BUSWIDTH_4BIT;
		g_buffer[0] = 0xa5; /* 0b10100101 */
		g_buffer[1] = 0xFF; /* 0b11111111 */
	}
	else if (bus_width == MSS_BUSWIDTH_8BIT) {
		card->bus_width = MSS_BUSWIDTH_8BIT;
		g_buffer[0] = 0xaa; /* 0b10101010 */
		g_buffer[1] = 0x55; /* 0b01010101 */
	}
	else
		goto exit;
	
	mss_set_buswidth(host, bus_width);
	
	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	MSS_INIT_CMD(cmd, MMC_BUSTEST_W, 0, 0, MSS_RESPONSE_R1); 
	MSS_INIT_DATA(data, 1, 512, MSS_DATA_WRITE, 1, &sg, 0); 
	llreq->cmd = cmd;
	llreq->data = data;

	ret = mss_send_ll_req(host, &mmc_card->llreq);
	/* 
	 * FIXME: Monahans will result MSS_ERROR_FLASH, but it seems that the 
	 * output is correct
	 */
	if (ret && ret != MSS_ERROR_FLASH && ret != MSS_ERROR_CRC)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;

	mmc_card->state = CARD_STATE_BTST;
	memset(g_buffer, 0xFF, 512);

	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	MSS_INIT_CMD(cmd, MMC_BUSTEST_R, 0, 0, MSS_RESPONSE_R1); 
	MSS_INIT_DATA(data, 1, 512, MSS_DATA_READ, 1, &sg, 0); 
	llreq->cmd = cmd;
	llreq->data = data;
	
	/* 
	 * FIXME: Monahans will result MSS_ERROR_CRC, but it seems that the 
	 * output is correct
	 */
	ret = mss_send_ll_req(host, &mmc_card->llreq);
	if (ret && ret != MSS_ERROR_CRC && ret != MSS_ERROR_FLASH)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;

	if ((g_buffer[0] == 0x55/*0b01010101*/) && 
		(g_buffer[1] == 0xaa/*0b10101010*/)) {
		mmc_card->bus_width = MSS_BUSWIDTH_8BIT;
	}
	else if (g_buffer[0] == 0x5a /*0b01011010*/) {
		mmc_card->bus_width = MSS_BUSWIDTH_4BIT;
	}
	else { 
		mmc_card->bus_width = MSS_BUSWIDTH_1BIT;
	}
#else
	mss_set_buswidth(host, MSS_BUSWIDTH_4BIT);
	mmc_card->bus_width = MSS_BUSWIDTH_4BIT;
	
#endif
	mmc_card->state = CARD_STATE_TRAN;
	
	arg = mmc_make_cmd6_arg(0x3, 0xb7, mmc_card->bus_width, 0);
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SWITCH, arg, 
			MSS_RESPONSE_R1B, 0);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;
	
	card->bus_width = mmc_card->bus_width;
#ifdef CONFIG_MMC_CEATA
	if (card->card_type != MSS_CEATA_CARD) 
		goto continue_not_ceata;

	memset(g_buffer, 0x0, 16);
#ifndef CEATA_CSS_MODE
	g_buffer[CEATA_CONTROL] = 0x02; /* not using CSS */
#endif
	g_buffer[CEATA_COMMAND_STATUS] = CEATA_IDENTIFY_DATA;

	sg.page = virt_to_page(g_buffer);
	sg.offset = offset_in_page(g_buffer);
	sg.length = 16;

	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	MSS_INIT_CMD(cmd, CEATA_RW_MULTIPLE_REGISTER, 0x80000010, 0, 
			MSS_RESPONSE_R1B); 
	MSS_INIT_DATA(data, 1, 16, MSS_DATA_WRITE, 1, &sg, 0); 
	llreq->cmd = cmd;
	llreq->data = data;
	
	ret = mss_send_ll_req(host, &mmc_card->llreq);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;

	retries = 10;
	do {
		mdelay(200);
		ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_FAST_IO, 
				(mmc_card->rca << 16) + 
				(CEATA_COMMAND_STATUS << 8), 
				MSS_RESPONSE_R4, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r4(cmd, &r4, mmc_card);
		if (ret) {
			return ret;
		}
		status = r4.read_reg_contents;
	} while((status & 0x40) != 0x40 && retries--);
	if (!retries)
		return MSS_ERROR_TIMEOUT;

	mss_set_clock(host, mmc_tran_speed(mmc_card->csd.tran_speed));
	memset(g_buffer, 0x0, 512);
	sg.page = virt_to_page(g_buffer);
	sg.offset = offset_in_page(g_buffer);
	sg.length = 512;

	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	MSS_INIT_CMD(cmd, CEATA_RW_MULTIPLE_BLOCK, 0x1, 0, 
			MSS_RESPONSE_R1B); 
	MSS_INIT_DATA(data, 1, 512, MSS_DATA_READ, 1, &sg, 0); 
	llreq->cmd = cmd;
	llreq->data = data;
	
	ret = mss_send_ll_req(host, &mmc_card->llreq);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;
	mmc_unpack_ceata(g_buffer, &mmc_card->ceata);
continue_not_ceata:
#endif
	/** 
	 * use CMD6 to set high speed mode. arg:access=0b11, arg:index=0xb9, 
	 * arg:value=1 according to card/controller high speed timing  
	 * high speed mode for MMC-4.x compatible card.
	 */
	if (host->high_speed == MSS_HIGH_SPEED) {
		mmc_make_cmd6_arg(0x3, 0xb9, 1, 0);
		ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SWITCH, arg, 
				MSS_RESPONSE_R1B, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r1(cmd, &r1, mmc_card);
		if (ret)
			return ret;
		memcpy(&ios, &host->ios, sizeof(struct mss_ios)); 
		ios.high_speed = MSS_HIGH_SPEED;
		ios.clock = MMC_CARD_CLOCK_FAST;
		host->ops->set_ios(host, &ios);	
		if (host->ios.high_speed != MSS_HIGH_SPEED) {
			u32 clock = mmc_tran_speed(mmc_card->csd.tran_speed);
			mss_set_clock(host, clock);	
		}
	}
	else {
		/* change to the highest speed in normal mode */
		u32 clock = mmc_tran_speed(mmc_card->csd.tran_speed);
		mss_set_clock(host, clock);
	}
		
	/** 
	 * use CMD6 to set power class. arg:access=0b11, arg:index=0xbb, 
	 * arg:value=card power class code according to card/controller supported 
	 * power class. We read value from PWL_CL_ff_vvv and set it to POWER_CLASS
	 * according to supported voltage and clock rate.
	 */
#if 0
	/* Deselect the card */
	ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SELECT_CARD, 0, 
			MSS_RESPONSE_R1B, 0);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;
	mmc_card->state = CARD_STATE_STBY;
#endif
exit:
	return MSS_ERROR_NONE;
}

static int mmc_read_write_entry(struct mss_card *card, int action, struct mss_rw_arg *arg, struct mss_rw_result *result)
{
	struct mmc_card *mmc_card = card->prot_card;
	struct mss_host *host;
	struct mss_ios ios;
	struct mmc_response_r1 r1;
	struct mss_ll_request *llreq = &mmc_card->llreq;
	struct mss_cmd *cmd = &mmc_card->cmd;
	struct mss_data *data = &mmc_card->data;
	int ret, retries = 4, clock;
	u32 status = 0;
	int access_mode_sector = 0;
	u32 cmdarg = 0;
	u32 blklen, opcode, flags;
#ifdef CONFIG_MMC_CEATA
	char *g_buffer = mmc_card->buf;
	struct scatterlist sg;
	struct mmc_response_r4 r4;
#endif

	host = card->slot->host;

	ret = mmc_get_status(card, &status);
	if (ret)
		return ret;

	if (status & R1_CARD_IS_LOCKED)
		return MSS_ERROR_LOCKED;

	if (action == MSS_WRITE_MEM && host->ops->is_slot_wp && 
			host->ops->is_slot_wp(card->slot))
		return MSS_ERROR_WP;
	
	if (mmc_card->state == CARD_STATE_STBY) {
		ret = mss_send_simple_ll_req(host, llreq, cmd, MMC_SELECT_CARD, 
				mmc_card->rca << 16, MSS_RESPONSE_R1, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r1(cmd, &r1, mmc_card);
		if (ret)
			return ret;
	}
	mmc_card->state = CARD_STATE_TRAN;

	mmc_fix_request_block_len(card, action, arg);
	memcpy(&ios, &host->ios, sizeof(struct mss_ios)); 
#ifdef CONFIG_MMC_CEATA
	if (card->card_type == MSS_CEATA_CARD) {
		blklen = arg->block_len;
		goto ceata;
	}
#endif
	access_mode_sector = (mmc_card->access_mode == MMC_ACCESS_MODE_SECTOR)
	       	&& host->high_capacity;	
	if (access_mode_sector) {
		ios.access_mode = MSS_ACCESS_MODE_SECTOR;
		cmdarg = arg->block;
		blklen = arg->block_len;
	}
	else {
		if (arg->block_len != mmc_card->block_len) {
			ret = mss_send_simple_ll_req(host, llreq, cmd, 
					MMC_SET_BLOCKLEN, arg->block_len, 
					MSS_RESPONSE_R1, 0);
			if (ret)
				return ret;
			ret = mmc_unpack_r1(cmd, &r1, mmc_card); 
			if (ret)
				return ret;
			mmc_card->block_len = arg->block_len;
		}
		cmdarg = arg->block * arg->block_len;
		blklen = arg->block_len;
	}

#ifdef CONFIG_MMC_CEATA
ceata:
#endif
	if (mmc_card->csd.spec_vers == CSD_SPEC_VERS_4 && host->high_speed) {
		ios.high_speed = MSS_HIGH_SPEED;
		ios.clock = MMC_CARD_CLOCK_FAST;
	}
	else {
		ios.clock = mmc_tran_speed(mmc_card->csd.tran_speed);
	}
	host->ops->set_ios(host, &ios);

read_write_entry:
	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	llreq->cmd = cmd;
	llreq->data = data;
	
#ifdef CONFIG_MMC_CEATA
	if (card->card_type != MSS_CEATA_CARD) 
		goto no_ceata;
	memset(g_buffer, 0x0, 16);
#ifndef CEATA_CSS_MODE
	g_buffer[CEATA_CONTROL] = 0x02;
#endif
	g_buffer[CEATA_SECTOR_COUNT] = arg->nob;
	g_buffer[CEATA_LBA_LOW] = (arg->block * blklen / 512) & 0xFF;
	g_buffer[CEATA_LBA_MID] = ((arg->block * blklen / 512) & 0xFF00) >> 8;
	g_buffer[CEATA_LBA_HIGH] = (arg->block * blklen / 512)  >> 16;
	g_buffer[CEATA_DEVICE_HEAD] = 0x40;
	g_buffer[CEATA_COMMAND_STATUS] = (action == MSS_READ_MEM) ? 
		CEATA_READ_DMA_EXT : CEATA_WRITE_DMA_EXT;
	dbg("LBA_LOW:0x%x, LBR_MID:0x%x, LBR_HI:0x%x\n", g_buffer[CEATA_LBA_LOW], g_buffer[CEATA_LBA_MID], g_buffer[CEATA_LBA_HIGH]);
	sg.page = virt_to_page(g_buffer);
	sg.offset = offset_in_page(g_buffer);
	sg.length = 16;

	MSS_INIT_CMD(cmd, CEATA_RW_MULTIPLE_REGISTER, 0x80000010, 0, 
			MSS_RESPONSE_R1B); 
	MSS_INIT_DATA(data, 1, 16, MSS_DATA_WRITE, 1, &sg, 0); 

	ret = mss_send_ll_req(host, &mmc_card->llreq);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;
	retries = 10;
	do {
		mdelay(200);
		ret = mss_send_simple_ll_req(host, llreq, cmd, 
				MMC_FAST_IO, (mmc_card->rca << 16) 
				+ (CEATA_COMMAND_STATUS << 8), 
				MSS_RESPONSE_R4, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r4(cmd, &r4, mmc_card);
		if (ret) {
			return ret;
		}
		status = r4.read_reg_contents;
	} while((status & 0x48) != 0x48 && retries--);
	if (!retries)
		return MSS_ERROR_TIMEOUT;

	cmdarg = arg->nob;
	if (action == MSS_WRITE_MEM)
		cmdarg |= 1 << 31;
	memset(llreq, 0x0, sizeof(struct mss_ll_request));
	memset(cmd, 0x0, sizeof(struct mss_cmd));
	memset(data, 0x0, sizeof(struct mss_data));
	llreq->cmd = cmd;
	llreq->data = data;

	if (action == MSS_READ_MEM) {
		flags = MSS_DATA_READ | MSS_DATA_MULTI;
	}
	else {
		flags = MSS_DATA_WRITE | MSS_DATA_MULTI;
	}

	MSS_INIT_CMD(cmd, CEATA_RW_MULTIPLE_BLOCK, cmdarg, 0, 
			MSS_RESPONSE_R1B);
	MSS_INIT_DATA(data, arg->nob, blklen, flags, arg->sg_len, 
			arg->sg, 0);

	ret = mss_send_ll_req(host, &mmc_card->llreq);
	if (ret)
		return ret;

	ret = mmc_unpack_r1(cmd, &r1, mmc_card);
	if (ret)
		return ret;
	retries = 10;
	do {
		mdelay(200);
		ret = mss_send_simple_ll_req(host, llreq, cmd, 
				MMC_FAST_IO, (mmc_card->rca << 16) 
				+ (CEATA_COMMAND_STATUS << 8), 
				MSS_RESPONSE_R4, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r4(cmd, &r4, mmc_card);
		if (ret) {
			return ret;
		}
		status = r4.read_reg_contents;
	} while((status & 0x40) != 0x40 && retries--);
	if (!retries)
		return MSS_ERROR_TIMEOUT;

	mmc_card->state = CARD_STATE_TRAN;
	result->bytes_xfered = data->bytes_xfered;
	return MSS_ERROR_NONE;
no_ceata:
#endif
	if (arg->nob > 1) {
		if (action == MSS_READ_MEM) {
			opcode = MMC_READ_MULTIPLE_BLOCK;
			flags = MSS_DATA_READ | MSS_DATA_MULTI;
		}
		else {
			opcode = MMC_WRITE_MULTIPLE_BLOCK;
			flags = MSS_DATA_WRITE | MSS_DATA_MULTI;
		}
		
		MSS_INIT_CMD(cmd, opcode, cmdarg, 0, MSS_RESPONSE_R1);
		MSS_INIT_DATA(data, arg->nob, blklen, flags, arg->sg_len, 
				arg->sg, 0);
		
		ret = mss_send_ll_req(host, &mmc_card->llreq);
		if (!ret)
			ret = mmc_unpack_r1(cmd, &r1, mmc_card);
		
		mmc_card->state = (action == MSS_WRITE_MEM) ? CARD_STATE_RCV 
			: CARD_STATE_DATA;
		
		if (ret) {
			mss_send_simple_ll_req(host, llreq, cmd, 
					MMC_STOP_TRANSMISSION, 0, 
					(action == MSS_WRITE_MEM) ? 
					MSS_RESPONSE_R1B : MSS_RESPONSE_R1, 0);
			mmc_card->state = CARD_STATE_TRAN;
			
			if (--retries) {
				clock = host->ios.clock;
				clock = clock >> 1;
				if (clock < MMC_CARD_CLOCK_SLOW && retries == 1)
					clock = MMC_CARD_CLOCK_SLOW;
				mss_set_clock(host, clock);
				goto read_write_entry;
			}
			return ret;
		}
		ret = mss_send_simple_ll_req(host, llreq, cmd, 
				MMC_STOP_TRANSMISSION, 0, 
				(action == MSS_WRITE_MEM) ? 
				MSS_RESPONSE_R1B : MSS_RESPONSE_R1, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r1(cmd, &r1, mmc_card);
		mmc_card->state = CARD_STATE_TRAN;
		
		if (ret	&& (mmc_card->errno != MMC_ERROR_OUT_OF_RANGE)) 
			return ret;
	} else {
		if (action == MSS_READ_MEM) {
			opcode = MMC_READ_SINGLE_BLOCK;
			flags = MSS_DATA_READ;
		}
		else {
			opcode = MMC_WRITE_BLOCK;
			flags = MSS_DATA_WRITE;
		}
		MSS_INIT_CMD(cmd, opcode, cmdarg, 0, MSS_RESPONSE_R1);
		MSS_INIT_DATA(data, arg->nob, blklen, flags, arg->sg_len, 
				arg->sg, 0);
		
		ret = mss_send_ll_req(host, &mmc_card->llreq);
		if (!ret)
			ret = mmc_unpack_r1(cmd, &r1, mmc_card);
		if (ret) {
			if (--retries) {
				clock = host->ios.clock;
				clock = clock >> 1;
				if (clock < MMC_CARD_CLOCK_SLOW && retries == 1)
					clock = MMC_CARD_CLOCK_SLOW;
				mss_set_clock(host, clock);
				goto read_write_entry;
			}
			return ret;
		}
	}
	/* Deselect the card */
	/*mmc_simple_ll_req(host, mmc_card, MMC_SELECT_CARD, 
		0, MSS_RESPONSE_NONE, 0);
	if (ret)
		return ret;
	ret = mmc_unpack_r1(&mmc_card->cmd, &r1, mmc_card);
	mmc_card->state = CARD_STATE_STBY;
	if (ret)
		return ret;*/
	result->bytes_xfered = data->bytes_xfered;

	return MSS_ERROR_NONE;
}

#if 0
static int mmc_lock_unlock_entry(struct mss_card *card, int action, struct mss_lock_arg *arg)
{
	struct mmc_response_r1 r1;
	struct mmc_card *mmc_card = card->prot_card;
	int ret;
	u32 status = 0;

	ret = mmc_get_status(card, &status)
	if (ret) {
		/*printk(KERN_INFO "can not get card status");
		retval = MSS_LOCK_FAILED;
		goto lock_error;*/
		return ret;
	}

	if (mmc_card->state == CARD_STATE_STBY) {
		ret = mmc_simple_ll_req(host, mmc_card, MMC_SELECT_CARD, (host->slot->rca) << 16, RESPONSE_R1B, 0);
		if (ret)
			return ret;
		ret = mmc_unpack_r1(&mmc_card->cmd, &r1, mmc_card->state);
		if (ret)
			return ret;
	}
	mmc_card->state = CARD_STATE_TRAN;

	mmc_fix_request_block_len(arg);

	ret = mmc_simple_ll_req(host, mmc_card, MMC_SET_BLOCKLEN, arg->block_len, RESPONSE_R1, 0);
	ret = mmc_unpack_r1(&mmc_card->cmd, &r1, mmc_card->state); 
	if (ret)
		return ret;
	
/*
	mss_simple_cmd(dev, MMC_SET_BLOCKLEN, t->block_len, RESPONSE_R1, buffer);
	if ((retval = mmc_unpack_r1(dev->io_request, &r1, dev->state))) 
		goto lock_error;
	
	cmd_backup = dev->cmd_flag;
	dev->cmd_flag = MSS_WRITE;*/
	mss_send_cmd(dev, MMC_LOCK_UNLOCK, 0, 1, t->block_len, RESPONSE_R1B, dev->io_request->buffer);
	if ((retval = mmc_unpack_r1(dev->io_request, &r1, dev->state))) { 
		dev->flags |= MSS_SLOT_FLAG_LOCK_FAILED;
		goto lock_error;
	}
	dev->cmd_flag = cmd_backup;
	
	mss_simple_cmd(dev, MMC_SEND_STATUS, (dev->slot->rca) << 16, RESPONSE_R1, buffer);
	if ((retval = mmc_unpack_r1( dev->io_request, &r1, dev->state))) 
		goto lock_error;
	
	dev->flags &= ~MSS_SLOT_FLAG_LOCK_FAILED;	

	if ( r1.status & R1_CARD_IS_LOCKED ) 
		dev->flags |= MSS_SLOT_FLAG_LOCKED;
	else 
		dev->flags &= ~MSS_SLOT_FLAG_LOCKED;

	mss_finish_io_request(dev, 1);	
	return MSS_SUCCESS;

lock_error:
	dev->flags |= MSS_SLOT_FLAG_LOCK_FAILED;
	mss_finish_io_request(dev, 0);	
	return retval; 
	return 0;
	
}
#endif

/* count by 512 bytes */
static int mmc_get_capacity(struct mss_card *card, u32 *size)
{
	int c_size = 0;
	int c_size_mult = 0;
	struct mmc_card *mmc_card = card->prot_card;
	struct mss_host *host = card->slot->host;

	if (mmc_card->access_mode == MMC_ACCESS_MODE_SECTOR && host->high_capacity)
		*size = mmc_card->ext_csd.sec_count;
	else {
		c_size = mmc_card->csd.c_size;
		c_size_mult = mmc_card->csd.c_size_mult;
		*size = (c_size + 1) 
			<< (2 + c_size_mult + mmc_card->csd.read_bl_len - 9);
	}
#ifdef CONFIG_MMC_CEATA
	if (card->card_type == MSS_CEATA_CARD)
		*size = 4 * 2* 1024 * 1024; /* FIXME: Consider it as 4GB card */
#endif
	dbg("the capacity :0x%x", *size);
	return MSS_ERROR_NONE;
}


/*****************************************************************************
 *
 *   protocol driver interface functions
 *
 ****************************************************************************/

static int mmc_prot_entry(struct mss_card *card, unsigned int action, void *arg, void *result)
{
	u32 status;
	int ret;
	
	if (action != MSS_RECOGNIZE_CARD && (card->card_type != MSS_MMC_CARD 
				&& card->card_type != MSS_CEATA_CARD))
		return MSS_ERROR_WRONG_CARD_TYPE;
	switch (action) {
		case MSS_RECOGNIZE_CARD:
			ret = mmc_recognize_card(card);
			break;
		case MSS_INIT_CARD:
			ret = mmc_card_init(card);
			break;
		case MSS_READ_MEM:
		case MSS_WRITE_MEM:
			if (!arg)
				return -EINVAL;
			ret = mmc_read_write_entry(card, action, arg, result);
			break;
#if 0
		case MSS_SELECT_CARD:
		case MSS_DESELECT_CARD:
			ret = mmc_select_deselect_entry(dev);
			break;
		case MSS_SUSPEND_CARD:
			ret = mmc_suspend_entry(dev);
			break;
#endif
/*		case MSS_LOCK_UNLOCK:
			ret = mmc_lock_unlock_entry(dev);
			break;*/
		case MSS_QUERY_CARD:
			ret = mmc_get_status(card, &status);
			break;
		case MSS_GET_CAPACITY:
			ret = mmc_get_capacity(card, result);
			break;
		default:
			ret = MSS_ERROR_ACTION_UNSUPPORTED;
			//debug("Unknown protocol action!\n");
			break;
	}
	/* dbg("mmc protocol entry exit, ret: %d, action: %d\n", ret, action);*/
	return ret;
}

static int mmc_prot_attach_card(struct mss_card *card)
{
	struct mmc_card *mmc_card;

#define ALIGN32(x)	(((x) + 31) & (~31))
	mmc_card = kzalloc(ALIGN32(ALIGN32(sizeof(struct mmc_card))) + 512, 
			GFP_KERNEL);
	card->prot_card = mmc_card;
	if (mmc_card) {
		mmc_card->buf = (char *)ALIGN32((unsigned int)&mmc_card[1]);
		return 0;
	}
	return -ENOMEM;
}

static void mmc_prot_detach_card(struct mss_card *card)
{
	kfree(card->prot_card);
}

static int mmc_prot_get_errno(struct mss_card *card)
{
	struct mmc_card *mmc_card = card->prot_card;
	
	return mmc_card->errno;
}

#if 0
static int mmc_protocol_proc_read_device(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct mss_card_device *dev = (struct mss_card_device *)data;
	struct mmc_card_info *card = (struct mmc_card_info *)dev->card;
	char *p = page;
	int len = 0;

	if (!dev)
		return 0;

	p += sprintf(p, "Slot #%d\n", dev->slotid);
	p += sprintf(p, "  State %s (%d)\n", mmc_state_to_string(dev->state), dev->state);

	if ( dev->state != CARD_STATE_EMPTY ) {
		p += sprintf(p, "  Media %s\n", (dev->dev.driver ? dev->dev.driver->name : "unknown"));
		if(dev->card_type==MMC_CARD){
			p += sprintf(p, "  MMC card\n");
			p += sprintf(p, "  CID mid=%d\n", card->card_cid.mid);
			p += sprintf(p, "      oid=%d\n", card->card_cid.oid);
			p += sprintf(p, "      pnm=%s\n", card->card_cid.pnm);
			p += sprintf(p, "      prv=%d.%d\n", card->card_cid.prv>>4, card->card_cid.prv&0xf);
			p += sprintf(p, "      psn=0x%08x\n", card->card_cid.psn);
			p += sprintf(p, "      mdt=%d/%d\n", card->card_cid.mdt>>4, (card->card_cid.mdt&0xf)+1997);
    			p += sprintf(p, "  CSD csd_structure=%d\n", card->card_csd.csd_structure);
			p += sprintf(p, "      spec_vers=%d\n", card->card_csd.spec_vers);
			p += sprintf(p, "      taac=0x%02x\n", card->card_csd.taac);
			p += sprintf(p, "      nsac=0x%02x\n", card->card_csd.nsac);
			p += sprintf(p, "      tran_speed=0x%02x\n", card->card_csd.tran_speed);
			p += sprintf(p, "      ccc=0x%04x\n", card->card_csd.ccc);
			p += sprintf(p, "      read_bl_len=%d\n", card->card_csd.read_bl_len);
			p += sprintf(p, "      read_bl_partial=%d\n", card->card_csd.read_bl_partial);
			p += sprintf(p, "      write_blk_misalign=%d\n", card->card_csd.write_blk_misalign);
			p += sprintf(p, "      read_blk_misalign=%d\n", card->card_csd.read_blk_misalign);
			p += sprintf(p, "      dsr_imp=%d\n", card->card_csd.dsr_imp);
			p += sprintf(p, "      c_size=%d\n", card->card_csd.c_size);
			p += sprintf(p, "      vdd_r_curr_min=%d\n", card->card_csd.vdd_r_curr_min);
			p += sprintf(p, "      vdd_r_curr_max=%d\n", card->card_csd.vdd_r_curr_max);
			p += sprintf(p, "      vdd_w_curr_min=%d\n", card->card_csd.vdd_w_curr_min);
			p += sprintf(p, "      vdd_w_curr_max=%d\n", card->card_csd.vdd_w_curr_max);
			p += sprintf(p, "      c_size_mult=%d\n", card->card_csd.c_size_mult);
			p += sprintf(p, "      wp_grp_size=%d\n", card->card_csd.wp_grp_size);
			p += sprintf(p, "      wp_grp_enable=%d\n", card->card_csd.wp_grp_enable);
			p += sprintf(p, "      default_ecc=%d\n", card->card_csd.default_ecc);
			p += sprintf(p, "      r2w_factor=%d\n", card->card_csd.r2w_factor);
			p += sprintf(p, "      write_bl_len=%d\n", card->card_csd.write_bl_len);
			p += sprintf(p, "      write_bl_partial=%d\n", card->card_csd.write_bl_partial);
			p += sprintf(p, "      file_format_grp=%d\n", card->card_csd.file_format_grp);
			p += sprintf(p, "      copy=%d\n", card->card_csd.copy);
			p += sprintf(p, "      perm_write_protect=%d\n", card->card_csd.perm_write_protect);
			p += sprintf(p, "      tmp_write_protect=%d\n", card->card_csd.tmp_write_protect);
			p += sprintf(p, "      file_format=%d\n", card->card_csd.file_format);
			p += sprintf(p, "      ecc=%d\n", card->card_csd.ecc);	
			switch (card->card_csd.csd_structure) {
			case CSD_STRUCT_VER_1_0:
			case CSD_STRUCT_VER_1_1:
				p += sprintf(p, "      sector_size=%d\n", card->card_csd.erase.v22.sector_size);
				p += sprintf(p, "      erase_grp_size=%d\n", card->card_csd.erase.v22.erase_grp_size);
				break;
			case CSD_STRUCT_VER_1_2:
			default:
				p += sprintf(p, "      erase_grp_size=%d\n", card->card_csd.erase.v31.erase_grp_size);
				p += sprintf(p, "      erase_grp_mult=%d\n", card->card_csd.erase.v31.erase_grp_mult);
				break;
			}	  
	  	}
	}
	
  	len = (p - page) - off;
	*start = page + off;
  	return len;
}
#endif


static struct mss_prot_driver mmc_protocol = {
	.name			=	MMC_PROTOCOL,
	.prot_entry		=	mmc_prot_entry,
	.attach_card		=	mmc_prot_attach_card,
	.detach_card		=	mmc_prot_detach_card,
	.get_errno		=	mmc_prot_get_errno,
};

static int mmc_protocol_init(void)
{
	register_mss_prot_driver(&mmc_protocol);
	return 0;
}

static void mmc_protocol_exit(void)
{
	unregister_mss_prot_driver(&mmc_protocol);
}


module_init(mmc_protocol_init);
module_exit(mmc_protocol_exit);

MODULE_AUTHOR("Bridge Wu");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMC protocol driver");
