/*
 * $Id: mtdblock_robadmap.c,v 1.3 2004/09/17 01:26:37 holmgren Exp $
 *
 * (C) 2003 David Woodhouse <dwmw2@infradead.org>
 *
 * Simple read-only (writable only for RAM) mtdblock driver
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/mtd/nand.h>

static int mtdblock_readsect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	size_t retlen;
	if (dev->badmap) {
		if (dev->mtd->read(dev->mtd, dev->badmap[block>>(dev->bmshift-9)]+(block&((1<<(dev->bmshift-9))-1)) * 512, 512, &retlen, buf))
			return 1;
	} else {
		if (dev->mtd->read(dev->mtd, (block*512), 512, &retlen, buf))
			return 1;
	}
	return 0;
}

static int mtdblock_writesect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	return 1;
}

static void mtdblock_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct mtd_blktrans_dev *dev = kmalloc(sizeof(*dev), GFP_KERNEL);

	if (!dev)
		return;

	memset(dev, 0, sizeof(*dev));

	dev->mtd = mtd;
	dev->devnum = mtd->index;
	dev->blksize = 512;
	dev->size = mtd->size >> 9;
	dev->tr = tr;
	dev->readonly = 1;

	if (mtd->type==MTD_NANDFLASH) {
		int x;
		int j;
		unsigned char u;
		unsigned int rl;
		dev->bmshift=14; /*erase blocks are 16k*/
		dev->badmap=kmalloc(sizeof(unsigned int)*(mtd->size>>dev->bmshift),GFP_KERNEL);
		if (dev->badmap==0) {
			kfree(dev);
			return;
		}
		printk("Scanning NAND flash mtd %p (%u eraseblocks) for bad blocks\n",mtd,(mtd->size>>dev->bmshift));
		j=0;
		for (x=0;x<(mtd->size>>dev->bmshift);x++) {
			if ((mtd->block_isbad(mtd,(x<<dev->bmshift)) )) {
				printk("block %d is bad\n",x);
				continue;
			}
			dev->badmap[j++]=x<<dev->bmshift;
		}
		dev->size=j<<(dev->bmshift-9);
		printk("After accounting for bad blocks, mtd %p size (in 512 byte blocks) is %ld\n",mtd,dev->size);
	}


	add_mtd_blktrans_dev(dev);
}

static void mtdblock_remove_dev(struct mtd_blktrans_dev *dev)
{
	del_mtd_blktrans_dev(dev);
	kfree(dev);
}

struct mtd_blktrans_ops mtdblockab_tr = {
	.name		= "mtdblock_autobadmap",
	.major		= 31,
	.part_bits	= 0,
	.readsect	= mtdblock_readsect,
	.writesect	= mtdblock_writesect,
	.add_mtd	= mtdblock_add_mtd,
	.remove_dev	= mtdblock_remove_dev,
	.owner		= THIS_MODULE,
};

static int __init mtdblock_init(void)
{
	return register_mtd_blktrans(&mtdblockab_tr);
}

static void __exit mtdblock_exit(void)
{
	deregister_mtd_blktrans(&mtdblockab_tr);
}

module_init(mtdblock_init);
module_exit(mtdblock_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("Simple read-only block device emulation access to MTD devices");
