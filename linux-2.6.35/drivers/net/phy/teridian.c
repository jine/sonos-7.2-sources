/*
 * Copyright (c) 2011, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * drivers/net/phy/teridian.c
 *
 * Driver for Teridian PHYs
 */
#include <linux/phy.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define TERIDIAN_PHYSR		01
#define TERIDIAN_PHYMR18         18
#define TERIDIAN_INER		17
#define TERIDIAN_INER_INIT	0x0400
#define TERIDIAN_INSR		17

#define TERIDIAN_100BT          0x400
#define TERIDIAN_FD             0x800

#define TERIDIAN_LINKUP         0x0004

MODULE_DESCRIPTION("Teridian PHY driver");
MODULE_LICENSE("GPL");

static int teridian_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, TERIDIAN_INSR);

	return (err < 0) ? err : 0;
}

static int teridian_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, TERIDIAN_INER,
				TERIDIAN_INER_INIT);
	else
		err = phy_write(phydev, TERIDIAN_INER, 0);

	return err;
}

static int
teridian_proc_read( char *page, char **start, off_t off, int count, int*eof, void *data)
{
	struct phy_device *phydev = (struct phy_device *)data;
	int len = 0, reg;
	*eof = 1;
	if (off != 0) {
		return 0;
	}

	for (reg = 0; reg <= 24; reg++) {
		int val;
		val = phy_read(phydev, reg);
		len += sprintf(page+len, "%04x ", val);
		if ((reg % 16) == 15) {
			len += sprintf(page+len, "\n");
		}
		else if ((reg % 8) == 7) {
			len += sprintf(page+len, "  ");
		}
	}
	len += sprintf(page+len, "\n");
	
	return len;
}

static int
teridian_proc_write(struct file *file, const char __user * buffer,
					unsigned long count, void *data)
{
	struct phy_device *phydev = (struct phy_device *)data;
	char buf[20];
	char *peq;
	int result, err;
	long longtemp;
	u32  regnum;
	u16  val;

	/* The following command writes register RR with the hex value VV. The register number RR 
	   must be a 2-digit hex number and the value VVVV a 4-digit hex value. */
	/* echo regRR=VVVV > /proc/driver/teridian-phy */

	if (count > 20)
		result = -EIO;
	else if (copy_from_user(buf, buffer, count)) {
		result = -EFAULT;
	}
	else {
		buf[count] = '\0';
		peq = strchr(buf, '=');
		if (peq != NULL) {
			*peq = 0;
			strict_strtol(peq+1, 16, &longtemp);
			val = longtemp;
			if (strncmp(buf, "reg", 3) == 0) {
				strict_strtol(peq-2, 16, &longtemp);
				regnum = longtemp;
				if (regnum <= 24) {
					err = phy_write(phydev, regnum, val);
					if (err < 0) {
						printk("%s: phy_write err %d\n", __func__, err);
					}
				}
				else {
					printk("%s: register num (0x%02x) too large\n", __func__, regnum);
				}
			}
		}		 

		result = count;
	}

	return result;
}

static int _setup_proc = 0;

static int teridian_config_init(struct phy_device *phydev)
{
	int ret;
	struct proc_dir_entry *proc;

	ret = teridian_config_intr(phydev);
	if (ret < 0) {
		return ret;
	}

	if (!_setup_proc) {
		proc = create_proc_read_entry("driver/teridian-phy", (S_IRUSR | S_IWUSR), 0, teridian_proc_read, phydev);
		if (proc == NULL) {
			printk("Unable to create driver/teridian-phy proc file\n");
			return( -EIO );
		}	
		proc->write_proc = (write_proc_t *) teridian_proc_write;
		_setup_proc = 1;
	}
		
	return 0;
}

static int teridian_remove(struct phy_device *phydev)
{
	/* REVIEW: This never seems to get called? */
	remove_proc_entry("driver/teridian-phy", 0);
	_setup_proc = 0;
	return 0;
}

static int teridian_read_status(struct phy_device *phydev)
{
	unsigned int ret = phy_read(phydev, TERIDIAN_PHYSR);

	if(ret & TERIDIAN_LINKUP) {
		ret = phy_read(phydev, TERIDIAN_PHYMR18);

		phydev->speed = (ret & TERIDIAN_100BT) ? 100 : 10;
		phydev->duplex = (ret & TERIDIAN_FD) ? DUPLEX_FULL : DUPLEX_HALF;
		phydev->link = 1;
	} else {
		phydev->speed = 0;
		phydev->duplex = 0;
		phydev->link = 0;
	}

	return 0;
}

/* TERIDIAN something-something-something */
static struct phy_driver teridian_driver = {
        .phy_id		= 0x000e7237,
	.name		= "Teridian 10/100 Ethernet",
	.phy_id_mask	= 0xfffffff0,
	.features	= PHY_BASIC_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &teridian_read_status,
	.ack_interrupt	= &teridian_ack_interrupt,
	.config_intr	= &teridian_config_intr,
        .config_init    = &teridian_config_init,
        .remove         = &teridian_remove,
	.driver		= { .owner = THIS_MODULE,},
};

static int __init teridian_init(void)
{
	int ret;

	ret = phy_driver_register(&teridian_driver);

	return ret;
}

static void __exit teridian_exit(void)
{
	phy_driver_unregister(&teridian_driver);
}

module_init(teridian_init);
module_exit(teridian_exit);
