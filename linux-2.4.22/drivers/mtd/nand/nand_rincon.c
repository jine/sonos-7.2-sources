/*
 *  drivers/mtd/nand/nand_rincon.c
 *
 *  Copyright (c) 2003 Texas Instruments
 *
 *  Derived from drivers/mtd/autcpu12.c
 *
 *  Copyright (c) 2002 Thomas Gleixner <tgxl@linutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   TI fido board. It supports 32MiB and 64MiB cards
 *
 * $Id: nand_rincon.c,v 1.19.16.1 2006/09/12 19:38:28 tober Exp $
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>

#ifdef CONFIG_SH_SONOS
#include <asm-sh/machvec.h>
#endif

#ifdef CONFIG_PPC_SONOS
// #include <asm/mpc8260.h>
// #include <asm/immap_8260.h>
#include <asm/cpm_8260.h>
#endif

#include "mdp.h"
#include "ptable.h"

/*
 * MTD structure for TOTO board
 */
static struct nand_oobinfo nand_rincon_oobinfo = {
        .useecc = MTD_NANDECC_PLACEONLY,
        .eccbytes = 6,
        .eccpos = {0, 1, 2, 3, 6, 7},
        .oobfree = { {8, 8} }
};

static struct mtd_info *nand_rincon_mtd = NULL;

extern struct nand_oobinfo jffs2_oobinfo_swecc;

extern char root_device_name[64];

char nandroot[64]="/dev/mtdblock_autobadmap/3";
char nandopt[64]="/dev/mtdblock_autobadmap/4";
char nandjffs[64]="/dev/mtdblock/5";

/*
 * Define partitions for flash devices
 */

#define SZ_1M 0x100000

/*
 * The following table was once used to represent the runtime definition
 * of the device's partition table before the devices supported reading the 
 * table from NAND flash.  Now however their contents (save the name) is
 * overwritten by the on-flash partition table.
 * 
 * Technically the connectx version could be used for PPC devices too.
 */
#ifdef CONFIG_CONNECTX_SONOS_ZP
static struct mtd_partition nand_rincon_partition_info[10] = {
	{ .name =	"reserved" },
	{ .name =	"kernel0" },
	{ .name =	"rootfs0" },
	{ .name =	"jffs" },
	{ .name =	"kernel1" },
	{ .name =	"rootfs1" },
	{ .name =	"<unused>" },
	{ .name =	"<unused>" },
	{ .name = 	"<unused>" },
	{ .name =	"<unused>" },
	{ .name =	"<unused>" },
};
#else // PPC and SH4 devices
static struct mtd_partition nand_rincon_partition_info[10] = {
        { .name =       "P1",
          .offset =     0,
          .size =       256*1024 },
	{ .name =	"P2",
	  .offset =	256*1024,
	  .size	=	768 * 1024 },
	{ .name =       "P3",
	  .offset =     1 * SZ_1M,
	  .size =       2 * SZ_1M },
	{ .name =       "P4",
          .offset =     3 * SZ_1M,
	  .size =       4 * SZ_1M },
	{ .name =       "P5",
	  .offset =     7 * SZ_1M,
          .size =       9 * SZ_1M },
	{ .name =	"P6" },
	{ .name =	"P7" },
	{ .name = 	"P8" },
	{ .name =	"P9" },
	{ .name =	"P10" },
};
#endif // CONFIG_CONNECTX_SONOS_ZP

#define NAND_RINCON_NUM_PARTITIONS 10
int nand_rincon_num_partitions=NAND_RINCON_NUM_PARTITIONS;
/* 
 *	hardware specific access to control-lines
*/
#ifdef CONFIG_SH_SONOS
volatile unsigned char *scsmr1=(unsigned char *)0xFFE00000;
volatile unsigned char *scscr1=(unsigned char *)0xFFE00008;
volatile unsigned char *scsptr1=(unsigned char *)0xFFE0001C;
volatile unsigned short *scscr2=(unsigned short *)0xFFE80008;
volatile unsigned short *scsptr2=(unsigned short *)0xFFE80020;
#endif

#ifdef CONFIG_PPC_SONOS
static volatile immap_t *immap;
static volatile iop8260_t *iop;
#define ALE_PORT 0
#define ALE_PIN 23
#define CLE_PORT 0
#define CLE_PIN 22
#define CE_PORT 2
#define CE_PIN 28
#define RB_PORT 2
#define RB_PIN 29
#endif

#define CHIPDELAY 15 /*us*/

static void nand_rincon_init_alecle(void)
{
	unsigned long flags;
	local_irq_save(flags);
#ifdef CONFIG_SH_SONOS
	*scsmr1&=~(unsigned char)0x80; /*Clear SCI SCSMR1 C/A bit*/
	*scscr1&=~(unsigned char)0x03; /*Clear SCI SCSCR1 CKE1 and CKE0 bits*/
	*scsptr1&=~(unsigned char)0x0C; /*Clear SCI SCSPTR1 SPB1DT and SPB1IO bits*/
	*scsptr1|=0x08; /*Set SCI SCSPTR1 SPB1IO bit*/
	*scscr2&=~(unsigned short)0x03; /*Clear SCIF SCSCR2 CKE1 and CKE0 bits*/
	*scsptr2&=~(unsigned short)0x0C; /*Clear SCIF SCSPTR2 SCKDT and SCKIO bits*/
	*scsptr2|=0x08; /*Set SCIF SCSPTR2 SCKIO bit*/
#endif 

#ifdef CONFIG_PPC_SONOS
        m8260_port_set(ALE_PORT,ALE_PIN,PORT_SET_DIR|PORT_CLEAR_PAR|PORT_CLEAR_DAT);
        m8260_port_set(CLE_PORT,CLE_PIN,PORT_SET_DIR|PORT_CLEAR_PAR|PORT_CLEAR_DAT);
	if (ZPS5)
	  m8260_port_set(CE_PORT,CE_PIN,PORT_CLEAR_DIR|PORT_CLEAR_PAR|PORT_SET_DAT); /*turn to input for S5 ECO*/
	else
	  m8260_port_set(CE_PORT,CE_PIN,PORT_SET_DIR|PORT_CLEAR_PAR|PORT_SET_DAT); /*inactive high*/
        m8260_port_set(RB_PORT,RB_PIN,PORT_CLEAR_DIR|PORT_CLEAR_PAR);
	local_irq_restore(flags);
	udelay(1);
#endif
}
static inline void nand_rincon_set_ale(void)
{
#ifdef CONFIG_SH_SONOS
	unsigned long flags;
	local_irq_save(flags);
	*scsptr2|=0x04;
#endif
#ifdef CONFIG_PPC_SONOS
	m8260_port_set(ALE_PORT,ALE_PIN,PORT_SET_DAT);
#endif
#ifdef CONFIG_SH_SONOS
	local_irq_restore(flags);
#endif
}
static inline void nand_rincon_clear_ale(void)
{
#ifdef CONFIG_SH_SONOS
	unsigned long flags;
	local_irq_save(flags);
	*scsptr2&=~(unsigned short)0x04;
#endif
#ifdef CONFIG_PPC_SONOS
	m8260_port_set(ALE_PORT,ALE_PIN,PORT_CLEAR_DAT);
#endif
#ifdef CONFIG_SH_SONOS
	local_irq_restore(flags);
#endif
}
static inline void nand_rincon_set_cle(void)
{
#ifdef CONFIG_SH_SONOS
	unsigned long flags;
	local_irq_save(flags);
	*scsptr1|=0x04;
#endif
#ifdef CONFIG_PPC_SONOS
	m8260_port_set(CLE_PORT,CLE_PIN,PORT_SET_DAT);
#endif
#ifdef CONFIG_SH_SONOS
	local_irq_restore(flags);
#endif
}
static inline void nand_rincon_clear_cle(void)
{
#ifdef CONFIG_SH_SONOS
	unsigned long flags;
	local_irq_save(flags);
	*scsptr1&=~(unsigned char)0x04;
#endif
#ifdef CONFIG_PPC_SONOS
	m8260_port_set(CLE_PORT,CLE_PIN,PORT_CLEAR_DAT);
#endif
#ifdef CONFIG_SH_SONOS
	local_irq_restore(flags);
#endif
}

static inline void nand_rincon_set_nce(void)
{
#ifdef CONFIG_PPC_SONOS
  if (!ZPS5)
    m8260_port_set(CE_PORT,CE_PIN,PORT_CLEAR_DAT);
#endif
}
static inline void nand_rincon_clear_nce(void)
{
#ifdef CONFIG_PPC_SONOS
  if (!ZPS5)
    m8260_port_set(CE_PORT,CE_PIN,PORT_SET_DAT);
#endif
#ifdef CONFIG_SH_SONOS
    udelay(CHIPDELAY);
#endif
}

static void nand_rincon_hwcontrol(struct mtd_info *mtd, int cmd)
{

	udelay(1);
	switch(cmd){

		case NAND_CTL_SETCLE: nand_rincon_set_cle(); break;
		case NAND_CTL_CLRCLE: nand_rincon_clear_cle(); break;

		case NAND_CTL_SETALE: nand_rincon_set_ale(); break;
		case NAND_CTL_CLRALE: nand_rincon_clear_ale(); break;

		case NAND_CTL_SETNCE: nand_rincon_set_nce(); break;
		case NAND_CTL_CLRNCE: nand_rincon_clear_nce(); break;
	}
	udelay(1);
}

#ifdef CONFIG_PPC_SONOS
static int nand_rincon_dev_ready(struct mtd_info *mtd)
{
    return m8260_port_read(RB_PORT,RB_PIN);
}
#endif

void nand_get_chip (struct nand_chip *this, struct mtd_info *mtd, int new_state);
void nand_release_chip (struct mtd_info *mtd);

void nand_rincon_sleep(void)
{
	nand_get_chip((struct nand_chip *)(&nand_rincon_mtd[1]),nand_rincon_mtd,FL_READING);
}

void nand_rincon_wake(void)
{
	nand_rincon_init_alecle();
	nand_release_chip(nand_rincon_mtd);
}

extern struct manufacturing_data_page sys_mdp;

#ifdef CONFIG_PPC_SONOS
int nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code);
int nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc);
void early_read_mdp(void)
{
    unsigned char buf[528];
    unsigned char e[6];
    unsigned char re[6];
    int x,r;
    nand_rincon_init_alecle();
    nand_rincon_set_nce();
    nand_rincon_set_cle();
    udelay(1);
    writeb(0,0xF8000000);
    udelay(1);
    nand_rincon_clear_cle();
    nand_rincon_set_ale();
    udelay(1);
    writeb(0,0xF8000000);
    ndelay(50);
    writeb(0,0xF8000000);
    ndelay(50);
#ifdef CONFIG_CONNECTX_SONOS_ZP
    writeb(0,0xF8000000);
    ndelay(50);
#endif
    writeb(0,0xF8000000);
    udelay(1);
    nand_rincon_clear_ale();
    udelay(1);
    while (!nand_rincon_dev_ready(0)) ;
    for (x=0;x<528;x++) {
        buf[x]=readb(0xF8000000);
        ndelay(50);
    }
    re[0]=buf[512];
    re[1]=buf[513];
    re[2]=buf[514];
    re[3]=buf[515];
    re[4]=buf[518];
    re[5]=buf[519];
    nand_calculate_ecc(0,buf,e);
    nand_calculate_ecc(0,buf+256,e+3);
    nand_rincon_clear_nce();
    r=nand_correct_data(0,buf,re,e);
    if (r<0) {
        printk("early_read_mdp: uncorrectable ECC error (first half)\n");
        return;
    }
    if (r) {
        printk("early_read_mdp: corrected ECC error (first half)\n");
    }
    r=nand_correct_data(0,buf+256,re+3,e+3);
    if (r<0) {
        printk("early_read_mdp: uncorrectable ECC error (second half)\n");
        return;
    }
    if (r) {
        printk("early_read_mdp: corrected ECC error (second half)\n");
    }
    memcpy(&sys_mdp,buf,sizeof(sys_mdp));
    printk("early_read_mdp: success\n");
}
#endif
/*
 * Main initialization routine
 */
#undef NO_PROBE_NAND
int __init nand_rincon_init (void)
{
	struct nand_chip *this;
	int err = 0;
#ifndef NO_PROBE_NAND

	/* Allocate memory for MTD device structure and private data */
	nand_rincon_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!nand_rincon_mtd) {
		printk (KERN_WARNING "Unable to allocate Rincon NAND MTD device structure.\n");
		err = -ENOMEM;
		goto out;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&nand_rincon_mtd[1]);

	/* Initialize structures */
	memset((char *) nand_rincon_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	nand_rincon_mtd->priv = this;

	/* Set address of NAND IO lines */
	this->hwcontrol = nand_rincon_hwcontrol;
#ifdef CONFIG_SH_SONOS
	this->IO_ADDR_W=0xA8000000;
	this->IO_ADDR_R=0xA8000000;
#endif
#ifdef CONFIG_PPC_SONOS
	this->IO_ADDR_W=0xF8000000;
	this->IO_ADDR_R=0xF8000000;
#endif
#ifdef CONFIG_PPC_SONOS
	this->dev_ready = nand_rincon_dev_ready;
#endif
#ifdef CONFIG_SH_SONOS
	this->dev_ready = NULL;
#endif
	/* 15 us command delay time */
	this->chip_delay = CHIPDELAY;		
	this->eccmode = NAND_ECC_SOFT;
	/*nand_rincon_mtd->oobinfo = jffs2_oobinfo_swecc;*/
	nand_rincon_init_alecle();

	if (nand_scan (nand_rincon_mtd, 1)) {
		err = -ENXIO;
		goto out_mtd;
	}
	add_mtd_device(nand_rincon_mtd);
	nand_rincon_scan();
	/* Register the partitions */
	switch(nand_rincon_mtd->size){
		case SZ_1M*16:
		case SZ_1M*32:
#ifdef CONFIG_CONNECTX_SONOS_ZP
		case SZ_1M*64:
#endif // CONFIG_CONNECTX_SONOS_ZP
			add_mtd_partitions(nand_rincon_mtd, nand_rincon_partition_info, nand_rincon_num_partitions);
			break;
		default: {
			printk (KERN_WARNING "Unsupported Nand device\n"); 
			err = -ENXIO;
			goto out_buf;
		}
	}

	goto out;
    
out_buf:
	nand_release(nand_rincon_mtd);
out_mtd:
	kfree (nand_rincon_mtd);
#else
	err=-ENXIO;
#endif
out:
	if (err) {
		/* This didn't go well, so let's try for the NOR flash*/
		panic("Failed to detect any NAND flash on this device.\n");
		/*NOTREACHED*/
		return err;
	} else {
#ifdef CONFIG_SH_SONOS_ZP
		setup_mv_zoneplayer( RINCON_MODEL_ES2 );
#endif
#ifdef CONFIG_SH_SONOS_HH
		setup_mv_handheld( RINCON_MODEL_ES2 );
#endif
	}
	return err;
}

struct nand_badmap_ent {
	int pbase;
	int lbase;
	int nblocks;
} nand_rincon_badmap[36];

int nand_rincon_badmap_ents= -1;

/*
 * The following table was once used to represent the on-flash definition
 * of the device's partition table before the devices supported reading the 
 * table from NAND flash.  Now however their contents is overwritten by 
 * the on-flash partition table.
 * 
 * Technically the connectx version could be used for PPC devices too.
 */
#ifdef CONFIG_CONNECTX_SONOS_ZP
struct ptable sys_pt = {
	0, /*magic*/
	0, /*flags*/
	{ { 0, 0, 0 } }
};
#else // PPC and SH4 devices
struct ptable sys_pt = {
	0, /*magic*/
	0, /*flags*/
	{ { PE_TYPE_DIAGS, 0, 16 },
	  { PE_TYPE_KERNEL, 16, 48 },
	  { PE_TYPE_ROOT, 64, 128 },
	  { PE_TYPE_OPT, 192, 256 },
	  { PE_TYPE_JFFS, 448, 576 },
	  { PE_TYPE_END, 0, 0 } }
};
#endif // CONFIG_CONNECTX_SONOS_ZP

int nand_rincon_ltop(int b)
{
	int x;
	for (x=0;x<=nand_rincon_badmap_ents;x++) {
		if (b<(nand_rincon_badmap[x].lbase+nand_rincon_badmap[x].nblocks)) return nand_rincon_badmap[x].pbase+b-nand_rincon_badmap[x].lbase;
	}
	return -1;
}

int nand_rincon_check_partition(int start,int end)
{
	u_char buf[512];
	u_char oobbuf[16];
	int page;
	/*unsigned int lpage=0;*/
	/*unsigned int d;*/
	int g=0,h;
	int r;
	size_t rl;
	struct nand_chip *this=nand_rincon_mtd->priv;
	page=start<<5;
	while (page<=((end<<5)+31)) {
		if ((page&31)==0) {
			if ((nand_rincon_mtd->block_isbad(nand_rincon_mtd,page<<9))) {
				printk("nand_rincon_check_partition: block %d is bad\n",page>>5);
				page+=32;
				continue;
			}
		}
		r=nand_rincon_mtd->read_ecc(nand_rincon_mtd,page<<9,512,&rl,buf,oobbuf,&nand_rincon_oobinfo);
		if ((r!=0)||(rl!=512)) {printk("nand_rincon_check_partition: read failed\n");return -1;}
		if (! (((oobbuf[8]==0x19)&&(oobbuf[9]==0x74))||
		       ((oobbuf[8]==0x74)&&(oobbuf[9]==0x19)))) {
			printk("nand_rincon_check_partition: bad magic\n");
			return -1;
		}
		h=oobbuf[10]|(oobbuf[11]<<8);
		if (h==0) {printk("nand_rincon_check_partition: bad generation(0)\n");return -1;}
		if (g==0) g=h;
		if (g!=h) {printk("nand_rincon_check_partition: generation mismatch\n");return -1;}
		/*d=oobbuf[12]|(oobbuf[13]<<8)|(oobbuf[14]<<16)|(oobbuf[15]<<24);*/
		/*if (d!=lpage) {printk("nand_rincon_check_partition: bad sequence number\n");return -1;}*/
		if ((oobbuf[8]==0x74)&&(oobbuf[9]==0x19)) {printk("nand_rincon_check_partition: success (generation %u)\n",g);return g;}
		/*lpage++;*/
		if (sys_pt.pt_magic!=PT_MAGIC) return g; /*xxx*/
		if (page==((end<<5)+31)) break;
		page=(end<<5)+31;
	}
	printk("nand_rincon_check_partition: reached the end of the partition before the end of the file (%d %d %d)\n",start,end,page);
	return -1;
}

int bootsection=0;
int bootgeneration=0;

int nand_rincon_scan(void)
{
	int page,lblock=0;
	int lastbad=1;
	int r,x;
	int section=0;
	size_t rl;
	u_char buf[512];
	u_char oobbuf[16];
	char sbuf[64];
	struct ptable *pt;
	int n[2]={0,0};
	int g[10];
	int skip=0;
	int s,e;
	struct nand_chip *this=nand_rincon_mtd->priv;
	int boundary=0;
	
	nand_rincon_badmap_ents= -1;
	nand_rincon_badmap[0].lbase=0;
	nand_rincon_badmap[0].nblocks=0;
	for (page=0;page<((nand_rincon_mtd->size)>>9);page+=32) {
		if ((nand_rincon_mtd->block_isbad(nand_rincon_mtd,page<<9))) {
			lastbad=1;
		} else {
			if (lastbad) {
				nand_rincon_badmap_ents++;
				nand_rincon_badmap[nand_rincon_badmap_ents].pbase=page>>5;
				nand_rincon_badmap[nand_rincon_badmap_ents].lbase=lblock;
				nand_rincon_badmap[nand_rincon_badmap_ents].nblocks=0;
			}
			lastbad=0;
			nand_rincon_badmap[nand_rincon_badmap_ents].nblocks++;
			lblock++;
		}
	}
	for (x=0;x<=nand_rincon_badmap_ents;x++) {
		printk("NAND badmap entry %d: lbase %d pbase %d nblocks %d\n",x,nand_rincon_badmap[x].lbase,nand_rincon_badmap[x].pbase,nand_rincon_badmap[x].nblocks);
	}
	r=nand_rincon_mtd->read_ecc(nand_rincon_mtd,0,512,&rl,buf,oobbuf,&nand_rincon_oobinfo);
	if ((r!=0)||(rl!=512)) {
		printk("read MDP failed\n");
		goto read_pt;
	}
	if( sys_mdp.mdp_magic != MDP_MAGIC ) {
		printk("It looks like this system has no MDP\n");
		goto read_pt;
	}
read_pt:
	r=nand_rincon_mtd->read_ecc(nand_rincon_mtd,nand_rincon_ltop(1)<<14,512,&rl,buf,oobbuf,&nand_rincon_oobinfo);
	if ((r!=0)||(rl!=512)) {
		printk("Egads, read partition table failed.\n");
		goto fake_it;
	}
	pt=(struct ptable *)buf;
	if (pt->pt_magic!=PT_MAGIC) {
		printk("It looks like this system has no partition table\n");
		goto fake_it;
	}
	memcpy(&sys_pt,pt,sizeof(sys_pt));



fake_it:
	nand_rincon_num_partitions=10;
	for (x=0;x<10;x++) {
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_END) break;
		if (sys_pt.pt_flags&PTF_REL) {
			s=nand_rincon_ltop(sys_pt.pt_entries[x].pe_start);
			e=nand_rincon_ltop(sys_pt.pt_entries[x].pe_start+sys_pt.pt_entries[x].pe_nblocks-1)-s+1;
		} else {
			s=sys_pt.pt_entries[x].pe_start;
			e=sys_pt.pt_entries[x].pe_nblocks;
		}
		nand_rincon_partition_info[x].offset=s*16384;
		nand_rincon_partition_info[x].size=e*16384;
		printk("Partition %d: offset %u size %u\n",x,nand_rincon_partition_info[x].offset,nand_rincon_partition_info[x].size);
	}
	if (sys_pt.pt_magic!=PT_MAGIC) {
		printk("Skipping partition check\n");
		s=0;
		bootgeneration=nand_rincon_check_partition(16,63);
		goto skip_partition_check;
	}
	for (x=0;x<10;x++) {
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_END) break;
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_JFFS) {
			section++;
			boundary=x;
			skip=0;
			continue;
		}
		if (skip) continue;
		if ((sys_pt.pt_entries[x].pe_type!=PE_TYPE_KERNEL)&&
		    (sys_pt.pt_entries[x].pe_type!=PE_TYPE_ROOT)&&
		    (sys_pt.pt_entries[x].pe_type!=PE_TYPE_OPT)) continue;
		if (sys_pt.pt_flags&PTF_REL) {
			s=nand_rincon_ltop(sys_pt.pt_entries[x].pe_start);
			e=nand_rincon_ltop(sys_pt.pt_entries[x].pe_start+sys_pt.pt_entries[x].pe_nblocks-1);
		} else {
			s=sys_pt.pt_entries[x].pe_start;
			e=sys_pt.pt_entries[x].pe_start+sys_pt.pt_entries[x].pe_nblocks-1;
		}
		g[x]=nand_rincon_check_partition(s,e);
		if (g[x]<=0) {n[section]=0;skip=1;continue;}
		if (n[section]==0) n[section]=g[x];
		if (g[x]!=n[section]) {n[section]=0;skip=1;continue;}
	}
	s=0;
	if (n[1]>n[0]) {
		printk("Selected boot section 1 (generation %d)\n",n[1]);
		s=boundary;
		bootsection=1;
		bootgeneration=n[1];
        } else if (n[0]>0) {
                printk("Selected boot section 0 (generation %d)\n",n[0]);
		bootsection=0;
		bootgeneration=n[0];
        } else {
                panic("No good boot section\n");
        }
skip_partition_check:
	nandroot[0]='\0';
	nandopt[0]='\0';
	nandjffs[0]='\0';
	for (x=s;x<10;x++) {
		snprintf(sbuf, sizeof(sbuf), "/dev/mtdblock_autobadmap/%d",x+1);
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_END) break;
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_ROOT) {
			strcpy(nandroot,sbuf);
			printk("link: /dev/nandroot -> %s\n",sbuf);
			strcpy(root_device_name,sbuf);
		} else
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_OPT) {
			strcpy(nandopt,sbuf);
			printk("link: /dev/nandopt -> %s\n",sbuf);
		} else
		if (sys_pt.pt_entries[x].pe_type==PE_TYPE_JFFS) {
			snprintf(sbuf, sizeof(sbuf), "/dev/mtdblock/%d",x+1);
			strcpy(nandjffs,sbuf);
			printk("link: /dev/nandjffs -> %s\n",sbuf);
			if (x!=s) break;
		}
	}
	return 0;
}
	
module_init(nand_rincon_init);

/*
 * Clean up routine
 */
static void __exit nand_rincon_cleanup (void)
{
	nand_release(nand_rincon_mtd);
	kfree (nand_rincon_mtd);

}
module_exit(nand_rincon_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("B. Tober, based on nand_toto.c by Richard Woodruff <r-woodruff2@ti.com>");
MODULE_DESCRIPTION("Glue layer for NAND flash on Rincon devices");
