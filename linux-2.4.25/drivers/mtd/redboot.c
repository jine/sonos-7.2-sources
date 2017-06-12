/*
 * $Id: redboot.c,v 1.4 2007/06/04 15:59:35 rkuper Exp $
 *
 * Parse RedBoot-style Flash Image System (FIS) tables and
 * produce a Linux partition array to match.
 */
#define SONOS_HACK
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include "redboot_crc32.c"

struct fis_image_desc {
    unsigned char name[16];      // Null terminated name
    unsigned long flash_base;    // Address within FLASH of image
    unsigned long mem_base;      // Address in memory where it executes
    unsigned long size;          // Length of image
    unsigned long entry_point;   // Execution entry point
    unsigned long data_length;   // Length of actual data
    unsigned char _pad[256-(16+10*sizeof(unsigned long))];
    unsigned long block_cksum;
    unsigned long gen;
    unsigned long magic;
#define FIS_MAGIC 0x69cd79cc
    unsigned long desc_cksum;    // Checksum over image descriptor
    unsigned long file_cksum;    // Checksum over image data
};

struct fis_list {
	struct fis_image_desc *img;
	struct fis_list *next;
};

static inline int redboot_checksum(struct fis_image_desc *img)
{
	/* RedBoot doesn't actually write the desc_cksum field yet AFAICT */
	return 1;
}

#ifdef SONOS_HACK
#define BLOCKSIZE 65536
#define VIPER_REDBOOT_PTABLE_POS 0x7e0000
#define VIPER_REDBOOT_ALT_PTABLE_POS 0x7c0000
#define ERASED 0xffffffff
#endif

int parse_redboot_partitions(struct mtd_info *master, struct mtd_partition **pparts)
{
	int nrparts = 0;
	struct fis_image_desc *buf;
	struct mtd_partition *parts;
	struct fis_list *fl = NULL, *tmp_fl;
	int ret, i;
	size_t retlen;
	char *names;
	int namelen = 0;

        int config_table_found = 0;
        u_int32_t config_table_start = master->size - master->erasesize;

#ifdef SONOS_HACK
	buf = kmalloc(BLOCKSIZE, GFP_KERNEL);
#else
	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
#endif

	if (!buf)
		return -ENOMEM;

	/* Read the start of the last erase block */
        {
#ifdef SONOS_HACK
	unsigned long g;
	u_int32_t part_table_start = VIPER_REDBOOT_ALT_PTABLE_POS;
#else
        u_int32_t part_table_start = master->size - master->erasesize;
#if defined(CONFIG_MTD_END_RESERVED)
        part_table_start -= CONFIG_MTD_END_RESERVED;
#endif
#endif /*SONOS_HACK*/
#ifdef SONOS_HACK
	ret = master->read(master, part_table_start,
                           sizeof(struct fis_image_desc), &retlen, (void *)buf);
	if (ret)
		goto out;
	if (retlen != sizeof(struct fis_image_desc)) {
		ret=-EIO;
		goto out;
	}
	if ((buf->magic!=FIS_MAGIC)||(buf->gen==ERASED)) {
		printk("Alternate FIS is bad\n");
		part_table_start = VIPER_REDBOOT_PTABLE_POS;
		goto do_read;
	}
	g=buf->gen;
	part_table_start = VIPER_REDBOOT_PTABLE_POS;
        ret = master->read(master, part_table_start,
                           sizeof(struct fis_image_desc), &retlen, (void *)buf);
        if (ret)
                goto out;
        if (retlen != sizeof(struct fis_image_desc)) {
                ret=-EIO;
                goto out;
        }
	if ((buf->magic!=FIS_MAGIC)||(buf->gen==ERASED)||(g>buf->gen)) {
		part_table_start = VIPER_REDBOOT_ALT_PTABLE_POS;
	}
	printk("Prefer FIS @ %08x\n",part_table_start);
        ret = master->read(master, part_table_start,
                           BLOCKSIZE, &retlen, (void *)buf);
        if (ret)
                goto out;
        if (retlen != BLOCKSIZE) {
                ret=-EIO;
                goto out;
        }
	g=buf->block_cksum;
	buf->block_cksum=0;
	if (cyg_crc32((unsigned char *)buf,BLOCKSIZE)==g) {
		printk("Selected FIS @ %08x\n",part_table_start);
		goto do_process;
	}
	if (part_table_start==VIPER_REDBOOT_PTABLE_POS) {
		part_table_start=VIPER_REDBOOT_ALT_PTABLE_POS;
	} else {
		part_table_start=VIPER_REDBOOT_PTABLE_POS;
	}
        ret = master->read(master, part_table_start,
                           BLOCKSIZE, &retlen, (void *)buf);
        if (ret)
                goto out;
        if (retlen != BLOCKSIZE) {
                ret=-EIO;
                goto out;
        }
	g=buf->block_cksum;
	buf->block_cksum=0;
	if ((part_table_start==VIPER_REDBOOT_PTABLE_POS)||
            (cyg_crc32((unsigned char *)buf,BLOCKSIZE)==g)) {
		printk("Selected FIS @ %08x\n",part_table_start);
		goto do_process;
	}
	part_table_start=VIPER_REDBOOT_PTABLE_POS;
	printk("Selected primary FIS as last resort\n");
do_read:
#endif /*SONOS_HACK*/
	ret = master->read(master, part_table_start,
			   PAGE_SIZE, &retlen, (void *)buf);
        }

	if (ret)
		goto out;

	if (retlen != PAGE_SIZE) {
		ret = -EIO;
		goto out;
	}
#ifdef SONOS_HACK
do_process:
#endif
	/* RedBoot image could appear in any of the first three slots */
	for (i = 0; i < 3; i++) {
		if (!memcmp(buf[i].name, "RedBoot", 8))
			break;
	}
	if (i == 3) {
		/* Didn't find it */
		printk(KERN_NOTICE "No RedBoot partition table detected in %s\n",
		       master->name);
		ret = 0;
		goto out;
	}

	for (i = 0; i < PAGE_SIZE / sizeof(struct fis_image_desc); i++) {
		struct fis_list *new_fl, **prev;

		if (buf[i].name[0] == 0xff)
			break;
		if (!redboot_checksum(&buf[i]))
			break;

		new_fl = kmalloc(sizeof(struct fis_list), GFP_KERNEL);
		namelen += strlen(buf[i].name)+1;
		if (!new_fl) {
			ret = -ENOMEM;
			goto out;
		}
		new_fl->img = &buf[i];
		buf[i].flash_base &= master->size-1;

		/* I'm sure the JFFS2 code has done me permanent damage.
		 * I now think the following is _normal_
		 */
		prev = &fl;
		while(*prev && (*prev)->img->flash_base < new_fl->img->flash_base)
			prev = &(*prev)->next;
		new_fl->next = *prev;
		*prev = new_fl;

		nrparts++;
	}
	if (fl->img->flash_base) {
                if( fl->img->flash_base == config_table_start )
                    config_table_found++;
		nrparts++;
        }

	for (tmp_fl = fl; tmp_fl->next; tmp_fl = tmp_fl->next) {
		if (tmp_fl->img->flash_base + tmp_fl->img->size + master->erasesize < tmp_fl->next->img->flash_base)
			nrparts++;
	}

        if( config_table_found == 0 ) {   
            // if it fits - make sure we have a board config partition
            if( tmp_fl->img->flash_base + tmp_fl->img->size <= config_table_start ) {
                struct fis_list *new_fl;
                struct fis_image_desc *new_fd;
                new_fl = kmalloc(sizeof(struct fis_list)+sizeof(struct fis_image_desc), GFP_KERNEL);
                new_fd = (struct fis_image_desc *)(new_fl+1);
                new_fl->img = new_fd;

                new_fd->flash_base = config_table_start; 
                new_fd->size = master->erasesize;
                strcpy( new_fd->name,(char *)"Board Config" );
                namelen += strlen( new_fd->name )+1;

                new_fl->next = 0;
                tmp_fl->next = new_fl;
                nrparts++;
            }
        }
        nrparts++; /* to account for the whole flash partition */

	parts = kmalloc(sizeof(*parts)*nrparts + namelen, GFP_KERNEL);

	if (!parts) {
		ret = -ENOMEM;
		goto out;
	}
	names = (char *)&parts[nrparts];
	memset(parts, 0, sizeof(*parts)*nrparts + namelen);
	i=0;

	parts[0].name = "whole flash";
	parts[0].size = master->size;
	parts[0].offset = 0;
	i++;
	if (fl->img->flash_base) {
	       parts[i].name = "unallocated space";
	       parts[i].size = fl->img->flash_base;
	       parts[i].offset = 0;
	}
	for ( ; i<nrparts; i++) {
		parts[i].size = fl->img->size;
		parts[i].offset = fl->img->flash_base;
		parts[i].name = names;

		strcpy(names, fl->img->name);
		names += strlen(names)+1;

		if(fl->next && fl->img->flash_base + fl->img->size + master->erasesize < fl->next->img->flash_base) {
			i++;
			parts[i].offset = parts[i-1].size + parts[i-1].offset;
			parts[i].size = fl->next->img->flash_base - parts[i].offset;
			parts[i].name = "unallocated space";
		}
		tmp_fl = fl;
		fl = fl->next;
		kfree(tmp_fl);
	}
	ret = nrparts;
	*pparts = parts;
 out:
	while (fl) {
		struct fis_list *old = fl;
		fl = fl->next;
		kfree(old);
	}
	kfree(buf);
	return ret;
}

EXPORT_SYMBOL(parse_redboot_partitions);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Red Hat, Inc. - David Woodhouse <dwmw2@cambridge.redhat.com>");
MODULE_DESCRIPTION("Parsing code for RedBoot Flash Image System (FIS) tables");
