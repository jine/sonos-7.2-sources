/* 
 * drivers/mtd/nand/diskonchip.c
 *
 * (C) 2003 Red Hat, Inc.
 *
 * Author: David Woodhouse <dwmw2@infradead.org>
 *
 * Interface to generic NAND code for M-Systems DiskOnChip devices
 *
 * $Id: diskonchip.c,v 1.20 2004/06/28 16:27:43 dbrown Exp $
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/io.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/doc2000.h>
#include <linux/mtd/compatmac.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/inftl.h>

/* Where to look for the devices? */
#ifndef CONFIG_MTD_DOCPROBE_ADDRESS
#define CONFIG_MTD_DOCPROBE_ADDRESS 0
#endif

static unsigned long __initdata doc_locations[] = {
#if defined (__alpha__) || defined(__i386__) || defined(__x86_64__)
#ifdef CONFIG_MTD_DOCPROBE_HIGH
	0xfffc8000, 0xfffca000, 0xfffcc000, 0xfffce000, 
	0xfffd0000, 0xfffd2000, 0xfffd4000, 0xfffd6000,
	0xfffd8000, 0xfffda000, 0xfffdc000, 0xfffde000, 
	0xfffe0000, 0xfffe2000, 0xfffe4000, 0xfffe6000, 
	0xfffe8000, 0xfffea000, 0xfffec000, 0xfffee000,
#else /*  CONFIG_MTD_DOCPROBE_HIGH */
	0xc8000, 0xca000, 0xcc000, 0xce000, 
	0xd0000, 0xd2000, 0xd4000, 0xd6000,
	0xd8000, 0xda000, 0xdc000, 0xde000, 
	0xe0000, 0xe2000, 0xe4000, 0xe6000, 
	0xe8000, 0xea000, 0xec000, 0xee000,
#endif /*  CONFIG_MTD_DOCPROBE_HIGH */
#elif defined(__PPC__)
	0xe4000000,
#elif defined(CONFIG_MOMENCO_OCELOT)
	0x2f000000,
        0xff000000,
#elif defined(CONFIG_MOMENCO_OCELOT_G) || defined (CONFIG_MOMENCO_OCELOT_C)
        0xff000000,
##else
#warning Unknown architecture for DiskOnChip. No default probe locations defined
#endif
	0xffffffff };

static struct mtd_info *doclist = NULL;

struct doc_priv {
	unsigned long virtadr;
	unsigned long physadr;
	u_char ChipID;
	u_char CDSNControl;
	int chips_per_floor; /* The number of chips detected on each floor */
	int curfloor;
	int curchip;
	int mh0_page;
	int mh1_page;
	struct mtd_info *nextdoc;
};

/* Max number of eraseblocks to scan (from start of device) for the (I)NFTL
   MediaHeader.  The spec says to just keep going, I think, but that's just
   silly. */
#define MAX_MEDIAHEADER_SCAN 8

/* This is the syndrome computed by the HW ecc generator upon reading an empty
   page, one with all 0xff for data and stored ecc code. */
static u_char empty_read_syndrome[6] = { 0x26, 0xff, 0x6d, 0x47, 0x73, 0x7a };
/* This is the ecc value computed by the HW ecc generator upon writing an empty
   page, one with all 0xff for data. */
static u_char empty_write_ecc[6] = { 0x4b, 0x00, 0xe2, 0x0e, 0x93, 0xf7 };

#define INFTL_BBT_RESERVED_BLOCKS 4

#define DoC_is_Millennium(doc) ((doc)->ChipID == DOC_ChipID_DocMil)
#define DoC_is_2000(doc) ((doc)->ChipID == DOC_ChipID_Doc2k)

static void doc200x_hwcontrol(struct mtd_info *mtd, int cmd);
static void doc200x_select_chip(struct mtd_info *mtd, int chip);

static int debug=0;
MODULE_PARM(debug, "i");

static int try_dword=1;
MODULE_PARM(try_dword, "i");

static int no_ecc_failures=0;
MODULE_PARM(no_ecc_failures, "i");

static int no_autopart=0;
MODULE_PARM(no_autopart, "i");

static int inftl_bbt_write=0;
MODULE_PARM(inftl_bbt_write, "i");

static unsigned long doc_config_location = CONFIG_MTD_DOCPROBE_ADDRESS;
MODULE_PARM(doc_config_location, "l");
MODULE_PARM_DESC(doc_config_location, "Physical memory address at which to probe for DiskOnChip");

static void DoC_Delay(struct doc_priv *doc, unsigned short cycles)
{
	volatile char dummy;
	int i;
	
	for (i = 0; i < cycles; i++) {
		if (DoC_is_Millennium(doc))
			dummy = ReadDOC(doc->virtadr, NOP);
		else
			dummy = ReadDOC(doc->virtadr, DOCStatus);
	}
	
}
/* DOC_WaitReady: Wait for RDY line to be asserted by the flash chip */
static int _DoC_WaitReady(struct doc_priv *doc)
{
	unsigned long docptr = doc->virtadr;
	unsigned long timeo = jiffies + (HZ * 10);

	if(debug) printk("_DoC_WaitReady...\n");
	/* Out-of-line routine to wait for chip response */
	while (!(ReadDOC(docptr, CDSNControl) & CDSN_CTRL_FR_B)) {
		if (time_after(jiffies, timeo)) {
			printk("_DoC_WaitReady timed out.\n");
			return -EIO;
		}
		udelay(1);
		cond_resched();
	}

	return 0;
}

static inline int DoC_WaitReady(struct doc_priv *doc)
{
	unsigned long docptr = doc->virtadr;
	int ret = 0;

	DoC_Delay(doc, 4);

	if (!(ReadDOC(docptr, CDSNControl) & CDSN_CTRL_FR_B))
		/* Call the out-of-line routine to wait */
		ret = _DoC_WaitReady(doc);

	DoC_Delay(doc, 2);
	if(debug) printk("DoC_WaitReady OK\n");
	return ret;
}

static void doc2000_write_byte(struct mtd_info *mtd, u_char datum)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;

	if(debug)printk("write_byte %02x\n", datum);
	WriteDOC(datum, docptr, CDSNSlowIO);
	WriteDOC(datum, docptr, 2k_CDSN_IO);
}

static u_char doc2000_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	u_char ret;

	ReadDOC(docptr, CDSNSlowIO);
	DoC_Delay(doc, 2);
	ret = ReadDOC(docptr, 2k_CDSN_IO);
	if (debug) printk("read_byte returns %02x\n", ret);
	return ret;
}

static void doc2000_writebuf(struct mtd_info *mtd, 
			     const u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int i;
	if (debug)printk("writebuf of %d bytes: ", len);
	for (i=0; i < len; i++) {
		WriteDOC_(buf[i], docptr, DoC_2k_CDSN_IO + i);
		if (debug && i < 16)
			printk("%02x ", buf[i]);
	}
	if (debug) printk("\n");
}

static void doc2000_readbuf(struct mtd_info *mtd, 
			    u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
 	int i;

	if (debug)printk("readbuf of %d bytes: ", len);

	for (i=0; i < len; i++) {
		buf[i] = ReadDOC(docptr, 2k_CDSN_IO + i);
	}
}

static void doc2000_readbuf_dword(struct mtd_info *mtd, 
			    u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
 	int i;

	if (debug) printk("readbuf_dword of %d bytes: ", len);

	if (unlikely((((unsigned long)buf)|len) & 3)) {
		for (i=0; i < len; i++) {
			*(uint8_t *)(&buf[i]) = ReadDOC(docptr, 2k_CDSN_IO + i);
		}
	} else {
		for (i=0; i < len; i+=4) {
			*(uint32_t*)(&buf[i]) = readl(docptr + DoC_2k_CDSN_IO + i);
		}
	}
}

static int doc2000_verifybuf(struct mtd_info *mtd, 
			      const u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int i;

	for (i=0; i < len; i++)
		if (buf[i] != ReadDOC(docptr, 2k_CDSN_IO))
			return -EFAULT;
	return 0;
}

static uint16_t __init doc200x_ident_chip(struct mtd_info *mtd, int nr)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	uint16_t ret;

	doc200x_select_chip(mtd, nr);
	doc200x_hwcontrol(mtd, NAND_CTL_SETCLE);
	this->write_byte(mtd, NAND_CMD_READID);
	doc200x_hwcontrol(mtd, NAND_CTL_CLRCLE);
	doc200x_hwcontrol(mtd, NAND_CTL_SETALE);
	this->write_byte(mtd, 0);
	doc200x_hwcontrol(mtd, NAND_CTL_CLRALE);

	ret = this->read_byte(mtd) << 8;
	ret |= this->read_byte(mtd);

	if (doc->ChipID == DOC_ChipID_Doc2k && try_dword && !nr) {
		/* First chip probe. See if we get same results by 32-bit access */
		union {
			uint32_t dword;
			uint8_t byte[4];
		} ident;
		unsigned long docptr = doc->virtadr;

		doc200x_hwcontrol(mtd, NAND_CTL_SETCLE);
		doc2000_write_byte(mtd, NAND_CMD_READID);
		doc200x_hwcontrol(mtd, NAND_CTL_CLRCLE);
		doc200x_hwcontrol(mtd, NAND_CTL_SETALE);
		doc2000_write_byte(mtd, 0);
		doc200x_hwcontrol(mtd, NAND_CTL_CLRALE);

		ident.dword = readl(docptr + DoC_2k_CDSN_IO);
		if (((ident.byte[0] << 8) | ident.byte[1]) == ret) {
			printk(KERN_INFO "DiskOnChip 2000 responds to DWORD access\n");
			this->read_buf = &doc2000_readbuf_dword;
		}
	}
		
	return ret;
}

static void __init doc2000_count_chips(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	uint16_t mfrid;
	int i;

	/* Max 4 chips per floor on DiskOnChip 2000 */
	doc->chips_per_floor = 4;

	/* Find out what the first chip is */
	mfrid = doc200x_ident_chip(mtd, 0);

	/* Find how many chips in each floor. */
	for (i = 1; i < 4; i++) {
		if (doc200x_ident_chip(mtd, i) != mfrid)
			break;
	}
	doc->chips_per_floor = i;
	printk(KERN_DEBUG "Detected %d chips per floor.\n", i);
}

static int doc200x_wait(struct mtd_info *mtd, struct nand_chip *this, int state)
{
	struct doc_priv *doc = (void *)this->priv;

	int status;
	
	DoC_WaitReady(doc);
	this->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	DoC_WaitReady(doc);
	status = (int)this->read_byte(mtd);

	return status;
}

static void doc2001_write_byte(struct mtd_info *mtd, u_char datum)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;

	WriteDOC(datum, docptr, CDSNSlowIO);
	WriteDOC(datum, docptr, Mil_CDSN_IO);
	WriteDOC(datum, docptr, WritePipeTerm);
}

static u_char doc2001_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;

	//ReadDOC(docptr, CDSNSlowIO);
	/* 11.4.5 -- delay twice to allow extended length cycle */
	DoC_Delay(doc, 2);
	ReadDOC(docptr, ReadPipeInit);
	//return ReadDOC(docptr, Mil_CDSN_IO);
	return ReadDOC(docptr, LastDataRead);
}

static void doc2001_writebuf(struct mtd_info *mtd, 
			     const u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int i;

	for (i=0; i < len; i++)
		WriteDOC_(buf[i], docptr, DoC_Mil_CDSN_IO + i);
	/* Terminate write pipeline */
	WriteDOC(0x00, docptr, WritePipeTerm);
}

static void doc2001_readbuf(struct mtd_info *mtd, 
			    u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int i;

	/* Start read pipeline */
	ReadDOC(docptr, ReadPipeInit);

	for (i=0; i < len-1; i++)
		buf[i] = ReadDOC(docptr, Mil_CDSN_IO);

	/* Terminate read pipeline */
	buf[i] = ReadDOC(docptr, LastDataRead);
}

static int doc2001_verifybuf(struct mtd_info *mtd, 
			     const u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int i;

	/* Start read pipeline */
	ReadDOC(docptr, ReadPipeInit);

	for (i=0; i < len-1; i++)
		if (buf[i] != ReadDOC(docptr, Mil_CDSN_IO)) {
			ReadDOC(docptr, LastDataRead);
			return i;
		}
	if (buf[i] != ReadDOC(docptr, LastDataRead))
		return i;
	return 0;
}

static void doc200x_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int floor = 0;

	/* 11.4.4 -- deassert CE before changing chip */
	doc200x_hwcontrol(mtd, NAND_CTL_CLRNCE);

	if(debug)printk("select chip (%d)\n", chip);

	if (chip == -1)
		return;

	floor = chip / doc->chips_per_floor;
	chip -= (floor *  doc->chips_per_floor);

	WriteDOC(floor, docptr, FloorSelect);
	WriteDOC(chip, docptr, CDSNDeviceSelect);

	doc200x_hwcontrol(mtd, NAND_CTL_SETNCE);

	doc->curchip = chip;
	doc->curfloor = floor;
}

static void doc200x_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;

	switch(cmd) {
	case NAND_CTL_SETNCE:
		doc->CDSNControl |= CDSN_CTRL_CE;
		break;
	case NAND_CTL_CLRNCE:
		doc->CDSNControl &= ~CDSN_CTRL_CE;
		break;
	case NAND_CTL_SETCLE:
		doc->CDSNControl |= CDSN_CTRL_CLE;
		break;
	case NAND_CTL_CLRCLE:
		doc->CDSNControl &= ~CDSN_CTRL_CLE;
		break;
	case NAND_CTL_SETALE:
		doc->CDSNControl |= CDSN_CTRL_ALE;
		break;
	case NAND_CTL_CLRALE:
		doc->CDSNControl &= ~CDSN_CTRL_ALE;
		break;
	case NAND_CTL_SETWP:
		doc->CDSNControl |= CDSN_CTRL_WP;
		break;
	case NAND_CTL_CLRWP:
		doc->CDSNControl &= ~CDSN_CTRL_WP;
		break;
	}
	if (debug)printk("hwcontrol(%d): %02x\n", cmd, doc->CDSNControl);
	WriteDOC(doc->CDSNControl, docptr, CDSNControl);
	/* 11.4.3 -- 4 NOPs after CSDNControl write */
	DoC_Delay(doc, 4);
}

static int doc200x_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;

	/* 11.4.2 -- must NOP four times before checking FR/B# */
	DoC_Delay(doc, 4);
	if (!(ReadDOC(docptr, CDSNControl) & CDSN_CTRL_FR_B)) {
		if(debug)
			printk("not ready\n");
		return 0;
	}
	/* 11.4.2 -- Must NOP twice if it's ready */
	DoC_Delay(doc, 2);
	if (debug)printk("was ready\n");
	return 1; 
}	

static int doc200x_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	/* This is our last resort if we couldn't find or create a BBT.  Just
	   pretend all blocks are good. */
	return 0;
}

static void doc200x_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;

	/* Prime the ECC engine */
	switch(mode) {
	case NAND_ECC_READ:
		WriteDOC(DOC_ECC_RESET, docptr, ECCConf);
		WriteDOC(DOC_ECC_EN, docptr, ECCConf);
		break;
	case NAND_ECC_WRITE:
		WriteDOC(DOC_ECC_RESET, docptr, ECCConf);
		WriteDOC(DOC_ECC_EN | DOC_ECC_RW, docptr, ECCConf);
		break;
	}	
}

/* This code is only called on write */
static int doc200x_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				 unsigned char *ecc_code)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	int i;
	int emptymatch = 1;

	/* flush the pipeline */
	if (DoC_is_2000(doc)) {
		WriteDOC(doc->CDSNControl & ~CDSN_CTRL_FLASH_IO, docptr, CDSNControl);
		WriteDOC(0, docptr, 2k_CDSN_IO);
		WriteDOC(0, docptr, 2k_CDSN_IO);
		WriteDOC(0, docptr, 2k_CDSN_IO);
		WriteDOC(doc->CDSNControl, docptr, CDSNControl);
	} else {
		WriteDOC(0, docptr, NOP);
		WriteDOC(0, docptr, NOP);
		WriteDOC(0, docptr, NOP);
	}

	for (i = 0; i < 6; i++) {
		ecc_code[i] = ReadDOC_(docptr, DoC_ECCSyndrome0 + i);
		if (ecc_code[i] != empty_write_ecc[i])
			emptymatch = 0;
	}
	WriteDOC(DOC_ECC_DIS, docptr, ECCConf);
#if 0
	/* If emptymatch=1, we might have an all-0xff data buffer.  Check. */
	if (emptymatch) {
		/* Note: this somewhat expensive test should not be triggered
		   often.  It could be optimized away by examining the data in
		   the writebuf routine, and remembering the result. */
		for (i = 0; i < 512; i++) {
			if (dat[i] == 0xff) continue;
			emptymatch = 0;
			break;
		}
	}
	/* If emptymatch still =1, we do have an all-0xff data buffer.
	   Return all-0xff ecc value instead of the computed one, so
	   it'll look just like a freshly-erased page. */
	if (emptymatch) memset(ecc_code, 0xff, 6);
#endif
	return 0;
}

static int doc200x_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	int i, ret = 0;
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	unsigned long docptr = doc->virtadr;
	volatile u_char dummy;
	int emptymatch = 1;
	
	/* flush the pipeline */
	if (DoC_is_2000(doc)) {
		dummy = ReadDOC(docptr, 2k_ECCStatus);
		dummy = ReadDOC(docptr, 2k_ECCStatus);
		dummy = ReadDOC(docptr, 2k_ECCStatus);
	} else {
		dummy = ReadDOC(docptr, ECCConf);
		dummy = ReadDOC(docptr, ECCConf);
		dummy = ReadDOC(docptr, ECCConf);
	}
	
	/* Error occured ? */
	if (dummy & 0x80) {
		for (i = 0; i < 6; i++) {
			calc_ecc[i] = ReadDOC_(docptr, DoC_ECCSyndrome0 + i);
			if (calc_ecc[i] != empty_read_syndrome[i])
				emptymatch = 0;
		}
		/* If emptymatch=1, the read syndrome is consistent with an
		   all-0xff data and stored ecc block.  Check the stored ecc. */
		if (emptymatch) {
			for (i = 0; i < 6; i++) {
				if (read_ecc[i] == 0xff) continue;
				emptymatch = 0;
				break;
			}
		}
		/* If emptymatch still =1, check the data block. */
		if (emptymatch) {
		/* Note: this somewhat expensive test should not be triggered
		   often.  It could be optimized away by examining the data in
		   the readbuf routine, and remembering the result. */
			for (i = 0; i < 512; i++) {
				if (dat[i] == 0xff) continue;
				emptymatch = 0;
				break;
			}
		}
		/* If emptymatch still =1, this is almost certainly a freshly-
		   erased block, in which case the ECC will not come out right.
		   We'll suppress the error and tell the caller everything's
		   OK.  Because it is. */
		if (!emptymatch) ret = doc_decode_ecc (dat, calc_ecc);
		if (ret > 0)
			printk(KERN_ERR "doc200x_correct_data corrected %d errors\n", ret);
	}	
	WriteDOC(DOC_ECC_DIS, docptr, ECCConf);
	if (no_ecc_failures && (ret == -1)) {
		printk(KERN_ERR "suppressing ECC failure\n");
		ret = 0;
	}
	return ret;
}
		
//u_char mydatabuf[528];

static struct nand_oobinfo doc200x_oobinfo = {
        .useecc = MTD_NANDECC_AUTOPLACE,
        .eccbytes = 6,
        .eccpos = {0, 1, 2, 3, 4, 5},
        .oobfree = { {8, 8} }
};
 
/* Find the (I)NFTL Media Header, and optionally also the mirror media header.
   On sucessful return, buf will contain a copy of the media header for
   further processing.  id is the string to scan for, and will presumably be
   either "ANAND" or "BNAND".  If findmirror=1, also look for the mirror media
   header.  The page #s of the found media headers are placed in mh0_page and
   mh1_page in the DOC private structure. */
static int __init find_media_headers(struct mtd_info *mtd, u_char *buf,
				     const char *id, int findmirror)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	int offs, end = (MAX_MEDIAHEADER_SCAN << this->phys_erase_shift);
	int ret, retlen;

	end = min(end, mtd->size); // paranoia
	for (offs = 0; offs < end; offs += mtd->erasesize) {
		ret = mtd->read(mtd, offs, mtd->oobblock, &retlen, buf);
		if (retlen != mtd->oobblock) continue;
		if (ret) {
			printk(KERN_WARNING "ECC error scanning DOC at 0x%x\n",
				offs);
		}
		if (memcmp(buf, id, 6)) continue;
		printk(KERN_INFO "Found DiskOnChip %s Media Header at 0x%x\n", id, offs);
		if (doc->mh0_page == -1) {
			doc->mh0_page = offs >> this->page_shift;
			if (!findmirror) return 1;
			continue;
		}
		doc->mh1_page = offs >> this->page_shift;
		return 2;
	}
	if (doc->mh0_page == -1) {
		printk(KERN_WARNING "DiskOnChip %s Media Header not found.\n", id);
		return 0;
	}
	/* Only one mediaheader was found.  We want buf to contain a
	   mediaheader on return, so we'll have to re-read the one we found. */
	offs = doc->mh0_page << this->page_shift;
	ret = mtd->read(mtd, offs, mtd->oobblock, &retlen, buf);
	if (retlen != mtd->oobblock) {
		/* Insanity.  Give up. */
		printk(KERN_ERR "Read DiskOnChip Media Header once, but can't reread it???\n");
		return 0;
	}
	return 1;
}

static inline int __init nftl_partscan(struct mtd_info *mtd,
				struct mtd_partition *parts)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	u_char *buf = this->data_buf;
	struct NFTLMediaHeader *mh = (struct NFTLMediaHeader *) buf;
	const int psize = 1 << this->page_shift;
	int blocks, maxblocks;
	int offs, numheaders;

	if (!(numheaders=find_media_headers(mtd, buf, "ANAND", 1))) return 0;

//#ifdef CONFIG_MTD_DEBUG_VERBOSE
//	if (CONFIG_MTD_DEBUG_VERBOSE >= 2)
	printk(KERN_INFO "    DataOrgID        = %s\n"
			 "    NumEraseUnits    = %d\n"
			 "    FirstPhysicalEUN = %d\n"
			 "    FormattedSize    = %d\n"
			 "    UnitSizeFactor   = %d\n",
		mh->DataOrgID, mh->NumEraseUnits,
		mh->FirstPhysicalEUN, mh->FormattedSize,
		mh->UnitSizeFactor);
//#endif

	blocks = mtd->size >> this->phys_erase_shift;
	maxblocks = min(32768, mtd->erasesize - psize);

	if (mh->UnitSizeFactor == 0x00) {
		/* Auto-determine UnitSizeFactor.  The constraints are:
		   - There can be at most 32768 virtual blocks.
		   - There can be at most (virtual block size - page size)
		     virtual blocks (because MediaHeader+BBT must fit in 1).
		*/
		mh->UnitSizeFactor = 0xff;
		while (blocks > maxblocks) {
			blocks >>= 1;
			maxblocks = min(32768, (maxblocks << 1) + psize);
			mh->UnitSizeFactor--;
		}
		printk(KERN_WARNING "UnitSizeFactor=0x00 detected.  Correct value is assumed to be 0x%02x.\n", mh->UnitSizeFactor);
	}

	/* NOTE: The lines below modify internal variables of the NAND and MTD
	   layers; variables with have already been configured by nand_scan.
	   Unfortunately, we didn't know before this point what these values
	   should be.  Thus, this code is somewhat dependant on the exact
	   implementation of the NAND layer.  */
	if (mh->UnitSizeFactor != 0xff) {
		this->bbt_erase_shift += (0xff - mh->UnitSizeFactor);
		mtd->erasesize <<= (0xff - mh->UnitSizeFactor);
		printk(KERN_INFO "Setting virtual erase size to %d\n", mtd->erasesize);
		blocks = mtd->size >> this->bbt_erase_shift;
		maxblocks = min(32768, mtd->erasesize - psize);
	}

	if (blocks > maxblocks) {
		printk(KERN_ERR "UnitSizeFactor of 0x%02x is inconsistent with device size.  Aborting.\n", mh->UnitSizeFactor);
		return 0;
	}

	/* Skip past the media headers. */
	offs = max(doc->mh0_page, doc->mh1_page);
	offs <<= this->page_shift;
	offs += mtd->erasesize;

	//parts[0].name = " DiskOnChip Boot / Media Header partition";
	//parts[0].offset = 0;
	//parts[0].size = offs;

	parts[0].name = " DiskOnChip BDTL partition";
	parts[0].offset = offs;
	parts[0].size = (mh->NumEraseUnits - numheaders) << this->bbt_erase_shift;

	offs += parts[0].size;
	if (offs < mtd->size) {
		parts[1].name = " DiskOnChip Remainder partition";
		parts[1].offset = offs;
		parts[1].size = mtd->size - offs;
		return 2;
	}
	return 1;
}

/* This is a stripped-down copy of the code in inftlmount.c */
static inline int __init inftl_partscan(struct mtd_info *mtd,
				 struct mtd_partition *parts)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	u_char *buf = this->data_buf;
	struct INFTLMediaHeader *mh = (struct INFTLMediaHeader *) buf;
	struct INFTLPartition *ip;
	int numparts = 0;
	int blocks;
	int vshift, lastvunit = 0;
	int i;
	int end = mtd->size;

	if (inftl_bbt_write)
		end -= (INFTL_BBT_RESERVED_BLOCKS << this->phys_erase_shift);

	if (!find_media_headers(mtd, buf, "BNAND", 0)) return 0;
	doc->mh1_page = doc->mh0_page + (4096 >> this->page_shift);

	mh->NoOfBootImageBlocks = le32_to_cpu(mh->NoOfBootImageBlocks);
	mh->NoOfBinaryPartitions = le32_to_cpu(mh->NoOfBinaryPartitions);
	mh->NoOfBDTLPartitions = le32_to_cpu(mh->NoOfBDTLPartitions);
	mh->BlockMultiplierBits = le32_to_cpu(mh->BlockMultiplierBits);
	mh->FormatFlags = le32_to_cpu(mh->FormatFlags);
	mh->PercentUsed = le32_to_cpu(mh->PercentUsed);
 
//#ifdef CONFIG_MTD_DEBUG_VERBOSE
//	if (CONFIG_MTD_DEBUG_VERBOSE >= 2)
	printk(KERN_INFO "    bootRecordID          = %s\n"
			 "    NoOfBootImageBlocks   = %d\n"
			 "    NoOfBinaryPartitions  = %d\n"
			 "    NoOfBDTLPartitions    = %d\n"
			 "    BlockMultiplerBits    = %d\n"
			 "    FormatFlgs            = %d\n"
			 "    OsakVersion           = 0x%x\n"
			 "    PercentUsed           = %d\n",
		mh->bootRecordID, mh->NoOfBootImageBlocks,
		mh->NoOfBinaryPartitions,
		mh->NoOfBDTLPartitions,
		mh->BlockMultiplierBits, mh->FormatFlags,
		mh->OsakVersion, mh->PercentUsed);
//#endif

	vshift = this->phys_erase_shift + mh->BlockMultiplierBits;

	blocks = mtd->size >> vshift;
	if (blocks > 32768) {
		printk(KERN_ERR "BlockMultiplierBits=%d is inconsistent with device size.  Aborting.\n", mh->BlockMultiplierBits);
		return 0;
	}

	blocks = doc->chips_per_floor << (this->chip_shift - this->phys_erase_shift);
	if (inftl_bbt_write && (blocks > mtd->erasesize)) {
		printk(KERN_ERR "Writeable BBTs spanning more than one erase block are not yet supported.  FIX ME!\n");
		return 0;
	}

	/* Scan the partitions */
	for (i = 0; (i < 4); i++) {
		ip = &(mh->Partitions[i]);
		ip->virtualUnits = le32_to_cpu(ip->virtualUnits);
		ip->firstUnit = le32_to_cpu(ip->firstUnit);
		ip->lastUnit = le32_to_cpu(ip->lastUnit);
		ip->flags = le32_to_cpu(ip->flags);
		ip->spareUnits = le32_to_cpu(ip->spareUnits);
		ip->Reserved0 = le32_to_cpu(ip->Reserved0);

//#ifdef CONFIG_MTD_DEBUG_VERBOSE
//		if (CONFIG_MTD_DEBUG_VERBOSE >= 2)
		printk(KERN_INFO	"    PARTITION[%d] ->\n"
			"        virtualUnits    = %d\n"
			"        firstUnit       = %d\n"
			"        lastUnit        = %d\n"
			"        flags           = 0x%x\n"
			"        spareUnits      = %d\n",
			i, ip->virtualUnits, ip->firstUnit,
			ip->lastUnit, ip->flags,
			ip->spareUnits);
//#endif

/*
		if ((i == 0) && (ip->firstUnit > 0)) {
			parts[0].name = " DiskOnChip IPL / Media Header partition";
			parts[0].offset = 0;
			parts[0].size = mtd->erasesize * ip->firstUnit;
			numparts = 1;
		}
*/

		if (ip->flags & INFTL_BINARY)
			parts[numparts].name = " DiskOnChip BDK partition";
		else
			parts[numparts].name = " DiskOnChip BDTL partition";
		parts[numparts].offset = ip->firstUnit << vshift;
		parts[numparts].size = (1 + ip->lastUnit - ip->firstUnit) << vshift;
		numparts++;
		if (ip->lastUnit > lastvunit) lastvunit = ip->lastUnit;
		if (ip->flags & INFTL_LAST) break;
	}
	lastvunit++;
	if ((lastvunit << vshift) < end) {
		parts[numparts].name = " DiskOnChip Remainder partition";
		parts[numparts].offset = lastvunit << vshift;
		parts[numparts].size = end - parts[numparts].offset;
		numparts++;
	}
	return numparts;
}

static int __init nftl_scan_bbt(struct mtd_info *mtd)
{
	int ret, numparts;
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	struct mtd_partition parts[2];

	memset((char *) parts, 0, sizeof(parts));
	/* On NFTL, we have to find the media headers before we can read the
	   BBTs, since they're stored in the media header eraseblocks. */
	numparts = nftl_partscan(mtd, parts);
	if (!numparts) return -EIO;
	this->bbt_td->options = NAND_BBT_ABSPAGE | NAND_BBT_8BIT |
				NAND_BBT_SAVECONTENT | NAND_BBT_WRITE |
				NAND_BBT_VERSION;
	this->bbt_td->veroffs = 7;
	this->bbt_td->pages[0] = doc->mh0_page + 1;
	if (doc->mh1_page != -1) {
		this->bbt_md->options = NAND_BBT_ABSPAGE | NAND_BBT_8BIT |
					NAND_BBT_SAVECONTENT | NAND_BBT_WRITE |
					NAND_BBT_VERSION;
		this->bbt_md->veroffs = 7;
		this->bbt_md->pages[0] = doc->mh1_page + 1;
	} else {
		this->bbt_md = NULL;
	}

	/* It's safe to set bd=NULL below because NAND_BBT_CREATE is not set.
	   At least as nand_bbt.c is currently written. */
	if ((ret = nand_scan_bbt(mtd, NULL)))
		return ret;
	add_mtd_device(mtd);
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
	if (!no_autopart) add_mtd_partitions(mtd, parts, numparts);
#endif
	return 0;
}

static int __init inftl_scan_bbt(struct mtd_info *mtd)
{
	int ret, numparts;
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;
	struct mtd_partition parts[5];

	if (this->numchips > doc->chips_per_floor) {
		printk(KERN_ERR "Multi-floor INFTL devices not yet supported.\n");
		return -EIO;
	}

	if (mtd->size == (8<<20)) {
#if 0
/* This doesn't seem to work for me.  I get ECC errors on every page. */
		/* The Millenium 8M is actually an NFTL device! */
		mtd->name = "DiskOnChip Millennium 8M (NFTL)";
		return nftl_scan_bbt(mtd);
#endif
		printk(KERN_ERR "DiskOnChip Millennium 8M is not supported.\n");
		return -EIO;
	}

	this->bbt_td->options = NAND_BBT_LASTBLOCK | NAND_BBT_8BIT |
				NAND_BBT_VERSION;
	if (inftl_bbt_write)
		this->bbt_td->options |= NAND_BBT_WRITE;
	this->bbt_td->offs = 8;
	this->bbt_td->len = 8;
	this->bbt_td->veroffs = 7;
	this->bbt_td->maxblocks = INFTL_BBT_RESERVED_BLOCKS;
	this->bbt_td->reserved_block_code = 0x01;
	this->bbt_td->pattern = "MSYS_BBT";

	this->bbt_md->options = NAND_BBT_LASTBLOCK | NAND_BBT_8BIT |
				NAND_BBT_VERSION;
	if (inftl_bbt_write)
		this->bbt_md->options |= NAND_BBT_WRITE;
	this->bbt_md->offs = 8;
	this->bbt_md->len = 8;
	this->bbt_md->veroffs = 7;
	this->bbt_md->maxblocks = INFTL_BBT_RESERVED_BLOCKS;
	this->bbt_md->reserved_block_code = 0x01;
	this->bbt_md->pattern = "TBB_SYSM";

	/* It's safe to set bd=NULL below because NAND_BBT_CREATE is not set.
	   At least as nand_bbt.c is currently written. */
	if ((ret = nand_scan_bbt(mtd, NULL)))
		return ret;
	memset((char *) parts, 0, sizeof(parts));
	numparts = inftl_partscan(mtd, parts);
	/* At least for now, require the INFTL Media Header.  We could probably
	   do without it for non-INFTL use, since all it gives us is
	   autopartitioning, but I want to give it more thought. */
	if (!numparts) return -EIO;
	add_mtd_device(mtd);
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
	if (!no_autopart) add_mtd_partitions(mtd, parts, numparts);
#endif
	return 0;
}

static inline int __init doc2000_init(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;

	this->write_byte = doc2000_write_byte;
	this->read_byte = doc2000_read_byte;
	this->write_buf = doc2000_writebuf;
	this->read_buf = doc2000_readbuf;
	this->verify_buf = doc2000_verifybuf;
	this->scan_bbt = nftl_scan_bbt;

	doc->CDSNControl = CDSN_CTRL_FLASH_IO | CDSN_CTRL_ECC_IO;
	doc2000_count_chips(mtd);
	mtd->name = "DiskOnChip 2000 (NFTL Model)";
	return (4 * doc->chips_per_floor);
}

static inline int __init doc2001_init(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = (void *)this->priv;

	this->write_byte = doc2001_write_byte;
	this->read_byte = doc2001_read_byte;
	this->write_buf = doc2001_writebuf;
	this->read_buf = doc2001_readbuf;
	this->verify_buf = doc2001_verifybuf;
	this->scan_bbt = inftl_scan_bbt;

	ReadDOC(doc->virtadr, ChipID);
	ReadDOC(doc->virtadr, ChipID);
	ReadDOC(doc->virtadr, ChipID);
	if (ReadDOC(doc->virtadr, ChipID) != DOC_ChipID_DocMil) {
		/* It's not a Millennium; it's one of the newer
		   DiskOnChip 2000 units with a similar ASIC. 
		   Treat it like a Millennium, except that it
		   can have multiple chips. */
		doc2000_count_chips(mtd);
		mtd->name = "DiskOnChip 2000 (INFTL Model)";
		return (4 * doc->chips_per_floor);
	} else {
		/* Bog-standard Millennium */
		doc->chips_per_floor = 1;
		mtd->name = "DiskOnChip Millennium";
		return 1;
	}
}

static inline int __init doc_probe(unsigned long physadr)
{
	unsigned char ChipID;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	struct doc_priv *doc;
	unsigned long virtadr;
	unsigned char save_control;
	unsigned char tmp, tmpb, tmpc;
	int reg, len, numchips;
	int ret = 0;

	virtadr = (unsigned long)ioremap(physadr, DOC_IOREMAP_LEN);
	if (!virtadr) {
		printk(KERN_ERR "Diskonchip ioremap failed: 0x%x bytes at 0x%lx\n", DOC_IOREMAP_LEN, physadr);
		return -EIO;
	}

	/* It's not possible to cleanly detect the DiskOnChip - the
	 * bootup procedure will put the device into reset mode, and
	 * it's not possible to talk to it without actually writing
	 * to the DOCControl register. So we store the current contents
	 * of the DOCControl register's location, in case we later decide
	 * that it's not a DiskOnChip, and want to put it back how we
	 * found it. 
	 */
	save_control = ReadDOC(virtadr, DOCControl);

	/* Reset the DiskOnChip ASIC */
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_RESET, 
		 virtadr, DOCControl);
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_RESET, 
		 virtadr, DOCControl);

	/* Enable the DiskOnChip ASIC */
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_NORMAL, 
		 virtadr, DOCControl);
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_NORMAL, 
		 virtadr, DOCControl);

	ChipID = ReadDOC(virtadr, ChipID);

	switch(ChipID) {
	case DOC_ChipID_Doc2k:
		reg = DoC_2k_ECCStatus;
		break;
	case DOC_ChipID_DocMil:
		reg = DoC_ECCConf;
		break;
	default:
		ret = -ENODEV;
		goto notfound;
	}
	/* Check the TOGGLE bit in the ECC register */
	tmp  = ReadDOC_(virtadr, reg) & DOC_TOGGLE_BIT;
	tmpb = ReadDOC_(virtadr, reg) & DOC_TOGGLE_BIT;
	tmpc = ReadDOC_(virtadr, reg) & DOC_TOGGLE_BIT;
	if ((tmp == tmpb) || (tmp != tmpc)) {
		printk(KERN_WARNING "Possible DiskOnChip at 0x%lx failed TOGGLE test, dropping.\n", physadr);
		ret = -ENODEV;
		goto notfound;
	}

	for (mtd = doclist; mtd; mtd = doc->nextdoc) {
		nand = mtd->priv;
		doc = (void *)nand->priv;
		/* Use the alias resolution register to determine if this is
		   in fact the same DOC aliased to a new address.  If writes
		   to one chip's alias resolution register change the value on
		   the other chip, they're the same chip. */
		unsigned char oldval = ReadDOC(doc->virtadr, AliasResolution);
		unsigned char newval = ReadDOC(virtadr, AliasResolution);
		if (oldval != newval)
			continue;
		WriteDOC(~newval, virtadr, AliasResolution);
		oldval = ReadDOC(doc->virtadr, AliasResolution);
		WriteDOC(newval, virtadr, AliasResolution); // restore it
		newval = ~newval;
		if (oldval == newval) {
			//printk(KERN_DEBUG "Found alias of DOC at 0x%lx to 0x%lx\n", doc->physadr, physadr);
			goto notfound;
		}
	}

	printk(KERN_NOTICE "DiskOnChip found at 0x%lx\n", physadr);

	len = sizeof(struct mtd_info) +
	      sizeof(struct nand_chip) +
	      sizeof(struct doc_priv) +
	      (2 * sizeof(struct nand_bbt_descr));
	mtd = kmalloc(len, GFP_KERNEL);
	if (!mtd) {
		printk(KERN_ERR "DiskOnChip kmalloc (%d bytes) failed!\n", len);
		ret = -ENOMEM;
		goto fail;
	}
	memset(mtd, 0, len);

	nand			= (struct nand_chip *) (mtd + 1);
	doc			= (struct doc_priv *) (nand + 1);
	nand->bbt_td		= (struct nand_bbt_descr *) (doc + 1);
	nand->bbt_md		= nand->bbt_td + 1;

	mtd->priv		= (void *) nand;
	mtd->owner		= THIS_MODULE;

	nand->priv		= (void *) doc;
	nand->select_chip	= doc200x_select_chip;
	nand->hwcontrol		= doc200x_hwcontrol;
	nand->dev_ready		= doc200x_dev_ready;
	nand->waitfunc		= doc200x_wait;
	nand->block_bad		= doc200x_block_bad;
	nand->enable_hwecc	= doc200x_enable_hwecc;
	nand->calculate_ecc	= doc200x_calculate_ecc;
	nand->correct_data	= doc200x_correct_data;
	//nand->data_buf
	nand->autooob		= &doc200x_oobinfo;
	nand->eccmode		= NAND_ECC_HW6_512;
	nand->options		= NAND_USE_FLASH_BBT | NAND_HWECC_SYNDROME;

	doc->physadr		= physadr;
	doc->virtadr		= virtadr;
	doc->ChipID		= ChipID;
	doc->curfloor		= -1;
	doc->curchip		= -1;
	doc->mh0_page		= -1;
	doc->mh1_page		= -1;
	doc->nextdoc		= doclist;

	if (ChipID == DOC_ChipID_Doc2k)
		numchips = doc2000_init(mtd);
	else
		numchips = doc2001_init(mtd);

	if ((ret = nand_scan(mtd, numchips))) {
		/* DBB note: i believe nand_release is necessary here, as
		   buffers may have been allocated in nand_base.  Check with
		   Thomas. FIX ME! */
		/* nand_release will call del_mtd_device, but we haven't yet
		   added it.  This is handled without incident by
		   del_mtd_device, as far as I can tell. */
		nand_release(mtd);
		kfree(mtd);
		goto fail;
	}

	/* Success! */
	doclist = mtd;
	return 0;

notfound:
	/* Put back the contents of the DOCControl register, in case it's not
	   actually a DiskOnChip.  */
	WriteDOC(save_control, virtadr, DOCControl);
fail:
	iounmap((void *)virtadr);
	return ret;
}

int __init init_nanddoc(void)
{
	int i;

	if (doc_config_location) {
		printk(KERN_INFO "Using configured DiskOnChip probe address 0x%lx\n", doc_config_location);
		return doc_probe(doc_config_location);
	} else {
		for (i=0; (doc_locations[i] != 0xffffffff); i++) {
			doc_probe(doc_locations[i]);
		}
	}
	/* No banner message any more. Print a message if no DiskOnChip
	   found, so the user knows we at least tried. */
	if (!doclist) {
		printk(KERN_INFO "No valid DiskOnChip devices found\n");
		return -ENODEV;
	}
	return 0;
}

void __exit cleanup_nanddoc(void)
{
	struct mtd_info *mtd, *nextmtd;
	struct nand_chip *nand;
	struct doc_priv *doc;

	for (mtd = doclist; mtd; mtd = nextmtd) {
		nand = mtd->priv;
		doc = (void *)nand->priv;

		nextmtd = doc->nextdoc;
		nand_release(mtd);
		iounmap((void *)doc->virtadr);
		kfree(mtd);
	}
}
	
module_init(init_nanddoc);
module_exit(cleanup_nanddoc);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("M-Systems DiskOnChip 2000 and Millennium device driver\n");
