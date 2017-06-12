/*
 * A vitual block driver.
 *
 * (C) 2015 Sonos, Inc.
 * (C) 2015 liang.chai@sonos.com
 * Authors  Liang Chai
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/init.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/ubi.h>
#include "sect_upgrade_header.h"
#include "virtual_block_ioctl.h"

#define VIRTUAL_BLOCK_MINORS	1
#define SECTOR_SHIFT		9

static char *Version = "0.1";
static int major_num = VIRTUAL_BLOCK_MAJOR;
module_param(major_num, int, 0644);
static int nsectors = 147456; /* rootfs partition size */
module_param(nsectors, int, 0644);
static u32 rootfscrc = 0; /* rootfs partition crc */
module_param(rootfscrc, int, 0644);

extern void get_rootfs_ubi_info(struct mtd_info * mtd, int *vol_id, int *ubi_num);
extern struct ubi_volume_desc * rootfs_open(int vol_id, int ubi_num);
extern int rootfs_update(struct ubi_volume_desc *desc, int64_t bytes);
extern ssize_t rootfs_write(struct ubi_volume_desc *desc, const char *buf,
		size_t count, int flag);
extern int rootfs_release(struct ubi_volume_desc *desc);

extern void get_rootfs_ubi_info(struct mtd_info * mtd, int *vol_id, int *ubi_num);
extern int sonos_get_rootfs_key(int, char *);

/* buffer to save the LUKS header */
#define LUKS_HEADER_LEN	1024
char luks_header[LUKS_HEADER_LEN];

struct virtual_block {
	int init_flag;
	unsigned long size;
	unsigned long realsize;
	int64_t rootfs_size;
	int vol_id;
	int ubi_num;
	int ubi_count;
	int update_flag;
	spinlock_t lock;
	char ubi_name[ROOTFS_UBI_NAME_LEN];
	struct mtd_info * mtd;
	struct ubi_volume_desc *desc;
	struct request_queue *queue;
	struct gendisk *gd;
	struct proc_dir_entry *procfile;
	u8 *luks_data;
	u8 *rootfs;
	unsigned long data_size;
	unsigned long data_len;
	char key[KEY_LEN];
};

struct virtual_block *vbdev;
static struct proc_dir_entry *virtual_block_procdir;

static int virtual_block_ubi(struct virtual_block *dev)
{
	extern struct mtd_info *__mtd_next_device(int i);
	struct mtd_info * mtd;
	int i;

	vbdev->update_flag = 0;
	vbdev->ubi_count  = 0;
	vbdev->mtd = NULL;
	if ( dev->ubi_name[0] == '0' ) {
		printk("No rootfs ubi provided\n");
		return -EFAULT;
	}

	for ( i = 0; i < 15; i++ ) {
		mtd = __mtd_next_device(i);
		if ( mtd == NULL )
			return -EFAULT;
		if ( memcmp(mtd->name, dev->ubi_name, 7) == 0 )
			break;
	}
	if ( i == 15 ) {
		printk("virtual_block_ubi %s not found\n", dev->ubi_name);
		return -EFAULT;
	}
	dev->mtd = mtd;
	printk("************ mtd info *******************\n");
	printk("Name                %s\n", mtd->name);
	printk("Flag                %x\n", mtd->flags);
	printk("writesize           %x\n", mtd->writesize);
	printk("erasesize           %x\n", mtd->erasesize);
	printk("size                %lx\n", (long unsigned int)mtd->size);
	get_rootfs_ubi_info(mtd, &dev->vol_id, &dev->ubi_num);
	printk("vol_id              %d\n", dev->vol_id);
	printk("ubi_num             %d\n", dev->ubi_num);
	dev->init_flag = 1;
	return 0;
}

static int ubi_init(struct virtual_block *dev)
{
	if ( dev->desc )
		return 0;
	dev->desc = rootfs_open(dev->vol_id, dev->ubi_num);
	if (dev->desc == NULL ) {
		printk("ubi_init rootfs_open fail\n");
		return 1;
	}
	return rootfs_update(dev->desc, dev->rootfs_size);
}

static int ubi_finish(struct virtual_block *dev)
{
	if ( dev->desc ) {
		rootfs_release(dev->desc);
		dev->desc = NULL;
	}
	if ( dev->luks_data) {
		vfree(dev->luks_data);
		dev->luks_data = NULL;
	}
	if ( dev->rootfs) {
		vfree(dev->rootfs);
		dev->rootfs = NULL;
	}
	dev->init_flag = 0;
	return 0;
}

static unsigned long crc_table[256] = {
  0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
  0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
  0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
  0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
  0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
  0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
  0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
  0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
  0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
  0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
  0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
  0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
  0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
  0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
  0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
  0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
  0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
  0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
  0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
  0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
  0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
  0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
  0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
  0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
  0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
  0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
  0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
  0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
  0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
  0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
  0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
  0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
  0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
  0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
  0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
  0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
  0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
  0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
  0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
  0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
  0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
  0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
  0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
  0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
  0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
  0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
  0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
  0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
  0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
  0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
  0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
  0x2d02ef8dL
};

/* ========================================================================= */
#define DO1(buf) crc = crc_table[((int)crc ^ (*buf++)) & 0xff] ^ (crc >> 8);
#define DO2(buf)  DO1(buf); DO1(buf);
#define DO4(buf)  DO2(buf); DO2(buf);
#define DO8(buf)  DO4(buf); DO4(buf);

//----------------------------------------------------------------------
static unsigned int crc32(unsigned int crc, unsigned char *buf, size_t len)
{
    crc = crc ^ 0xffffffffL;
    while (len >= 8)
    {
      DO8(buf);
      len -= 8;
    }
    if (len) do {
      DO1(buf);
    } while (--len);
    return crc ^ 0xffffffffL;
}

static int ubi_commit(struct virtual_block *dev)
{
	int err;
	static u32 crc_calculated;

	if ( dev->update_flag == 0 ) {
		dev->update_flag = 1;
		printk("rootfs_write %ld\n", dev->realsize);
		err = rootfs_write(dev->desc, dev->luks_data, dev->realsize, 0);
		if ( err != dev->realsize ) {
			printk("rootfs_write fail\n");
			return 1;
		}
		crc_calculated = crc32(0, dev->luks_data, dev->realsize);
		dev->ubi_count = dev->realsize;
	}
	if ( dev->data_len ) {
		int len;
		size_t bytes_to_go = dev->data_len;
		char * p;
		p = dev->rootfs;
		printk("rootfs_write %ld\n", dev->data_len);
		while ( bytes_to_go ) {
			len = bytes_to_go;
			if ( len > 4096 )
				len  = 4096;
			err = rootfs_write(dev->desc, p, len, 0);
			if ( err != len ) {
				printk("rootfs_write fail\n");
				return 1;
			}
			crc_calculated = crc32(crc_calculated, p, len);
			bytes_to_go -= len;
			p += len;
		}
		dev->ubi_count += dev->data_len;
		dev->data_len = 0;
		rootfscrc = crc_calculated;
	}
	return 0;
}

static int virtual_block_open(struct block_device *bdev, fmode_t mode)
{
	return 0;
}

static void virtual_block_release(struct gendisk *disk, fmode_t mode)
{
}

static int virtual_block_ioctl(struct block_device *bdev, fmode_t mode,
		unsigned int cmd, unsigned long arg)
{
	struct virtual_block *dev = bdev->bd_disk->private_data;
	char __user *p = (char __user *)arg;
	struct virtual_block_ioctl msg;
	int err = 0;
	u32 crc_reported;

	switch (cmd) {
		case VIRTUAL_BLOCK_SET_CONFIG:
			if ( copy_from_user(&msg, p, sizeof(msg)) )
				return -EFAULT;
			dev->rootfs_size =  msg.rootfs_size + 0x200000;
			memset(dev->ubi_name, 0, ROOTFS_UBI_NAME_LEN);
			memcpy(dev->ubi_name, msg.rootfs_ubi_name,
				msg.rootfs_name_len);
			ubi_finish(dev);
			err = virtual_block_ubi(dev);
			if ( err ) {
				printk("cannot find mtd\n");
				return err;
			}
			err = ubi_init(dev);
			if ( err ) {
				printk("cannot init ubi %s, error %d\n",
					dev->mtd->name, err);
				return err;
			}
			dev->data_size = msg.rootfs_size;
			dev->data_len = 0;
			dev->rootfs = vmalloc(vbdev->data_size);
			if ( dev->rootfs == NULL ) {
				pr_err("kmalloc rootfs failed\n");
				return -ENOMEM;
			}
			/* 2M for temporary data */
			vbdev->realsize = 4096 << SECTOR_SHIFT;
			dev->luks_data = vmalloc(vbdev->realsize);
			if ( dev->luks_data == NULL ) {
				vfree(dev->rootfs);
				dev->rootfs = NULL;
				pr_err("kmalloc data failed\n");
				return -ENOMEM;
			}
			memset(dev->luks_data, 0, vbdev->realsize);
			break;
		case VIRTUAL_BLOCK_GET_CONFIG:
			if ( dev->init_flag == 0 ) {
				pr_err("device is not configured\n");
				return -EFAULT;
			}
			msg.rootfs_size = dev->rootfs_size - 0x200000;
			memset(msg.rootfs_ubi_name, 0, ROOTFS_UBI_NAME_LEN);
			memcpy(msg.rootfs_ubi_name, dev->ubi_name,
				ROOTFS_UBI_NAME_LEN);
			msg.rootfs_name_len = ROOTFS_UBI_NAME_LEN;
			memcpy(msg.key, dev->key, KEY_LEN);
			if ( copy_to_user(p, &msg, sizeof(msg)) )
				return -EFAULT;
			break;
		case VIRTUAL_BLOCK_COMMIT:
			if ( dev->init_flag == 0 ) {
				pr_err("device is not configured\n");
				return -EFAULT;
			}
			ubi_commit(dev);
			break;
		case VIRTUAL_BLOCK_CHECK:
			if ( dev->init_flag == 0 ) {
				pr_err("device is not configured\n");
				return -EFAULT;
			}
			if ( copy_to_user(p, &dev->data_len,
				sizeof(dev->data_len)) != 0 )
				err = -EFAULT;
			break;
		case VIRTUAL_BLOCK_COMPLETE:
			if ( dev->init_flag == 0 ) {
				pr_err("device is not configured\n");
				return -EFAULT;
			}
			ubi_finish(dev);
			break;
		case VIRTUAL_BLOCK_GET_CRC:
			crc_reported = 0;
			if ( dev->desc == NULL ) {
				crc_reported = rootfscrc;
			}
			if ( copy_to_user(p, &crc_reported, sizeof(u32)) != 0 )
				err = -EFAULT;
			break;
		default:
			printk("Command %x not supported\n", cmd);
			break;
	}
	return err;
}

static const struct block_device_operations virtual_block_ops = {
	.owner		= THIS_MODULE,
	.open		= virtual_block_open,
	.release	= virtual_block_release,
	.ioctl		= virtual_block_ioctl,
};

static void virtual_block_transfer(struct virtual_block *dev, unsigned long sector,
		unsigned long nsect, char *buffer, int write)
{
	unsigned long offset = sector << SECTOR_SHIFT;
	unsigned long nbytes = nsect << SECTOR_SHIFT;

	if (offset > dev->size || dev->size - offset < nbytes) {
		pr_notice("Beyond-end %s (%ld %ld)\n",
				write ? "write" : "read", offset, nbytes);
		return;
	}
	if (offset > dev->realsize || dev->realsize - offset <= nbytes) {
		if ( write ) {
			unsigned long temp_offset;

			if ( dev->rootfs == NULL )
				return;
			if ( offset < 0x200000 ) {
				printk("virtual_block_transfer invalid offset %ld\n", offset);
				return;
			}
			temp_offset = offset - 0x200000;
			if ( dev->data_size - temp_offset >= nbytes ) {
				memcpy(dev->rootfs + temp_offset, buffer, nbytes);
				dev->data_len += nbytes;
			}
		} else {
			memset(buffer, 0, nbytes);
		}
		return;
	}

	if ( dev->luks_data == NULL ) {
		if ( offset == 0 ) {
			int len = LUKS_HEADER_LEN;
			if ( len > nbytes )
				len = nbytes;
			memcpy(buffer, luks_header, len);
		}
		return;
	}
	if (write) {
		memcpy(dev->luks_data + offset, buffer, nbytes);
		if ( offset == 0 ) {
			int len = LUKS_HEADER_LEN;
			if ( len > nbytes )
				len = nbytes;
			memcpy(luks_header, buffer, len);
		}
	} else
		memcpy(buffer, dev->luks_data + offset, nbytes);
}

static int virtual_block_xfer_bio(struct virtual_block *dev, struct bio *bio)
{
	int i;
	struct bio_vec *bvec;
	sector_t sector = bio->bi_sector;

	bio_for_each_segment(bvec, bio, i) {
		char *buffer = __bio_kmap_atomic(bio, i, KM_USER0);
		unsigned len = bvec->bv_len >> SECTOR_SHIFT;

		virtual_block_transfer(dev, sector, len, buffer,
				bio_data_dir(bio) == WRITE);
		sector += len;
		__bio_kunmap_atomic(bio, KM_USER0);
	}
	return 0;
}

static void virtual_block_make_request(struct request_queue *q, struct bio *bio)
{
	struct virtual_block *dev = q->queuedata;
	int status = virtual_block_xfer_bio(dev, bio);
	bio_endio(bio, status);
}

static int proc_read_virtual_block(struct seq_file *m, void *v)
{
	struct virtual_block *dev =  m->private;

	seq_printf(m, "init status:\t\t\t%d\n", dev->init_flag);
	seq_printf(m, "Update status:\t\t\t%d\n", dev->update_flag);
	seq_printf(m, "Update UBI Volume:\t\t%s\n", dev->ubi_name[0] == '0' ? "": dev->ubi_name);
	seq_printf(m, "Virtual Block size:\t\t%ld\n", dev->size);
	seq_printf(m, "Virtual Block size(Buffer):\t\t%ld\n", dev->realsize);
	seq_printf(m, "Virtual Block size(rootfs):\t\t%lld\n", dev->rootfs_size);
	seq_printf(m, "Updated count:\t\t%d\n", dev->ubi_count);
	return 0;
}

static int proc_open_virtual_block(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_virtual_block, PDE_DATA(inode));
}

static const struct file_operations fops = {
	.open = proc_open_virtual_block,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int convert_key(char *sonos_key, char * buffer)
{
	char temp[3];
	int i;

	temp[2] = 0;
	for ( i = 0; i < KEY_LEN; i++ ) {
		long longtemp;
		memcpy(temp, &buffer[2*i], 2);
		if ( strict_strtol(temp, 16, &longtemp) != 0 )
			return -EIO;
		sonos_key[i] = (char )longtemp;
	}
	return 0;
}

static int __init virtual_block_init(void)
{
	char key[2 * KEY_LEN + 1] = {0};

	if (register_blkdev(major_num, "virtualblock") < 0 ) {
		printk("virtual_block: unable to get major number\n");
		return -EIO;
	}
	pr_info("Virtual Block %s: major: %d\n", Version, major_num);
	vbdev = kmalloc(sizeof(struct virtual_block), GFP_KERNEL);
	if ( vbdev == NULL ) {
		pr_err("kmalloc virtual_block failed\n");
		goto out_unregister;
	}

	memset(vbdev, 0x0, sizeof(struct virtual_block));
	vbdev->size = nsectors << SECTOR_SHIFT;
	vbdev->data_len = 0;

	spin_lock_init(&vbdev->lock);
	vbdev->queue = blk_alloc_queue(GFP_KERNEL);
	if (vbdev->queue == NULL) {
		pr_err("blk_alloc_queue failed\n");
		goto out_alloc_data;
	}

	blk_queue_make_request(vbdev->queue, virtual_block_make_request);
	vbdev->queue->queuedata = vbdev;

	vbdev->gd = alloc_disk(VIRTUAL_BLOCK_MINORS);
	if ( vbdev->gd == NULL ) {
		pr_err("alloc_disk failed\n");
		goto out_alloc_disk;
	}
	vbdev->gd->major = major_num;
	vbdev->gd->first_minor = VIRTUAL_BLOCK_MINORS;
	vbdev->gd->fops = &virtual_block_ops;
	vbdev->gd->queue = vbdev->queue;
	vbdev->gd->private_data = vbdev;
	snprintf(vbdev->gd->disk_name, 32, "virtualblock%d", VIRTUAL_BLOCK_MINORS);
	set_capacity(vbdev->gd, nsectors);
	add_disk(vbdev->gd);
	virtual_block_procdir = proc_mkdir("virtualblock", NULL);
	if ( virtual_block_procdir == NULL ) {
		pr_err("proc failed\n");
		goto out_proc_fail;
	}
	vbdev->procfile = proc_create_data("status", 0444, virtual_block_procdir,
			&fops, vbdev);
	if ( vbdev->procfile == NULL ) {
		pr_err("proc create data failed\n");
		goto cleanup;
	}
	/* following code will be removed if the key is not exposed to user
	 * the user space code will use this as place holder for table load
	 * the real table load in kernel ioctl call needs to get the right key
	 * for encryption
	 */
	sonos_get_rootfs_key(SECT_UPGRADE_ROOTFS_FORMAT_FIXED_KEY, key);
	if ( convert_key(vbdev->key, key) ) {
		pr_err("Not Valid key\n");
		goto wrong_key;
	}
	return 0;

wrong_key:
	if ( vbdev->procfile )
		remove_proc_entry("status", virtual_block_procdir);
cleanup:
	if ( virtual_block_procdir )
		remove_proc_entry("virtualblock", NULL);
out_proc_fail:
	if ( vbdev->gd )
		del_gendisk(vbdev->gd);
out_alloc_disk:
	blk_cleanup_queue(vbdev->queue);
	vbdev->queue = NULL;
out_alloc_data:
	kfree(vbdev);
out_unregister:
	unregister_blkdev(major_num, "virtualblock");
	return -ENOMEM;
}

static void __exit virtual_block_exit(void)
{
	ubi_finish(vbdev);
	if ( vbdev->procfile )
		remove_proc_entry("status", virtual_block_procdir);
	vbdev->procfile = NULL;
	if ( virtual_block_procdir )
		remove_proc_entry("virtualblock", NULL);
	virtual_block_procdir = NULL;
	if ( vbdev->gd )
		del_gendisk(vbdev->gd);
	if ( vbdev->queue )
		blk_cleanup_queue(vbdev->queue);
	if ( vbdev->luks_data)
		vfree(vbdev->luks_data);
	if ( vbdev->rootfs)
		vfree(vbdev->rootfs);
	kfree(vbdev);
	unregister_blkdev(major_num, "virtualblock");
	pr_info("Virtual Block exit: major: %d\n", major_num);
}

module_init(virtual_block_init);
module_exit(virtual_block_exit);
MODULE_LICENSE("GPL");
