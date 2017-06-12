/*
 * Copyright (c) 2011, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * drivers/net/phy/mv88e6020.c
 *
 * Driver for Marvell 88e6020 PHY (created by Sonos, not part of stock Linux)
 */
#include <linux/phy.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "mdp.h"
extern struct manufacturing_data_page sys_mdp;

#define MV88E6020_REG_PHY_CNTL        0x00
#define MV88E6020_REG_PHY_SPEC_CNTL   0x10
#define MV88E6020_REG_INT_MASK        0x12
#define MV88E6020_REG_INT_STATUS      0x13

#define MV88E6020_INT_LINK	      0x0400
#define MV88E6020_HWINT_NUM       5   /* P1014 maps the IRQ5 line to interrupt number 5 */

#define MV88E6020_REG_STATUS            0x11
#define MV88E6020_RTLINK	            0x0400
#define MV88E6020_PHY_STATUS_FULLDUPLEX 0x2000
#define MV88E6020_PHY_STATUS_100MBPS    0x4000

#define MV88E6020_PORT0_SWITCH_ADDR       0x18
#define MV88E6020_PORT1_SWITCH_ADDR       0x19
#define MV88E6020_PORT5_SWITCH_ADDR       0x1d
#define MV88E6020_PORT6_SWITCH_ADDR       0x1e
#define MV88E6020_SWITCH_REG_IDENT        0x03
#define MV88E6020_SWITCH_REG_PORT_CONTROL 0x04

#define MV88E6020_GLOBAL1_REGS_ADDR       0x1f
#define MV88E6020_REG_SWITCH_INTR_STATUS  0x00
#define MV88E6020_REG_SWITCH_INTR_MASK    0x04
#define MV88E6020_DEVICE_INT              0x0080

#define MV88E6020_GLOBAL2_REGS_ADDR     0x17
#define MV88E6020_PHY_INTR_STATUS_REG   0x00
#define MV88E6020_PHY_INTR_MASK_REG     0x01
#define MV88E6020_SMI_PHY_ADDR_REG      0x18
#define MV88E6020_SMI_PHY_DATA_REG      0x19
#define MV88E6020_SMI_PHY_ADDR_BASE     0x10
#define MV88E6020_SMI_PHY_ADDR_SHIFT    5
#define MV88E6020_SMI_PHY_ADDR_WRITE    0x9400
#define MV88E6020_SMI_PHY_ADDR_READ     0x9800
#define MV88E6020_SMIBUSY               0x8000
#define MV88E6020_GLOBAL2_PHY_INT(phy)  (1 << phy)
#define MV88E6020_PHY_TIMEOUT		10000

#define MV88E6020_PROC_DIR		"driver/mv88e6020"

#define MV88E6020_STATS_OP_REG          0x1d
#define MV88E6020_STATS_READ_REG_HI     0x1e
#define MV88E6020_STATS_READ_REG_LO     0x1f
#define MV88E6020_STATS_BUSY            0x8000
#define MV88E6020_STATS_READ            0x4000
#define MV88E6020_STATS_FLUSH           0x1000
#define MV88E6020_STATS_PORT(x)         (x + 1) << 5
#define MV88E6020_STATS_IN_GOOD_OCT     0x0000
#define MV88E6020_STATS_IN_BAD_OCT      0x0002
#define MV88E6020_STATS_IN_UNI          0x0004
#define MV88E6020_STATS_IN_BCAST        0x0006
#define MV88E6020_STATS_IN_MCAST        0x0007
#define MV88E6020_STATS_IN_PAUSE        0x0016
#define MV88E6020_STATS_IN_SMALL        0x0018
#define MV88E6020_STATS_IN_FRAG         0x0019
#define MV88E6020_STATS_IN_OVER         0x001a
#define MV88E6020_STATS_IN_JABBR        0x001b
#define MV88E6020_STATS_IN_RX_ERR       0x001c
#define MV88E6020_STATS_IN_FCS          0x001d
#define MV88E6020_STATS_OUT_OCT         0x000e
#define MV88E6020_STATS_OUT_UNI         0x0010
#define MV88E6020_STATS_OUT_BCAST       0x0013
#define MV88E6020_STATS_OUT_MCAST       0x0012
#define MV88E6020_STATS_OUT_PAUSE       0x0015
#define MV88E6020_STATS_OUT_COLL        0x001e
#define MV88E6020_STATS_OUT_DEFER       0x0005
#define MV88E6020_STATS_OUT_SINGL       0x0014
#define MV88E6020_STATS_OUT_MULTI       0x0017
#define MV88E6020_STATS_OUT_FCS         0x0003
#define MV88E6020_STATS_OUT_EXCS        0x0011
#define MV88E6020_STATS_OUT_LATE        0x001f
#define MV88E6020_STATS_NUM_COUNTERS    12

struct counter {
	int opcode;
	char not_set;
	unsigned long *element;
};

struct ethtool_cmd mv88e6020_status[2] = {{0}};

MODULE_DESCRIPTION("Mv88e6020 PHY driver");
MODULE_AUTHOR("Ed Perreault");
MODULE_LICENSE("GPL");

static struct phy_device *mv88e6020_phydev = NULL;
static struct net_device *mv88e6020_mac_device = NULL;
void (*mv88e6020_link_up)(struct net_device *ndev) = NULL;
static struct work_struct mv88e6020_link_isr_work;
static struct delayed_work mv88e6020_errata34_work;
int mv88e6020_irq;
static unsigned long mv88e6020_int_count=0;
static unsigned long mv88e6020_work_count=0;
static unsigned int  mv88e6020_max_pass=0;
static unsigned long mv88e6020_errata_passes=0;
static unsigned long mv88e6020_errata_count=0;

static struct proc_dir_entry *procdir;

static int swicth_to_cpu_link_status = 0;

typedef struct _mv88e6020_env {
	char	*name;
	int	id;
} mv88e6020_env_t;

static mv88e6020_env_t mv88e6020_named_entries[] = {
	{ "global1",	MV88E6020_GLOBAL1_REGS_ADDR},
	{ "global2",	MV88E6020_GLOBAL2_REGS_ADDR},
	{ "switch0",	MV88E6020_PORT0_SWITCH_ADDR},
	{ "switch1",	MV88E6020_PORT1_SWITCH_ADDR},
	{ "switch5",	MV88E6020_PORT5_SWITCH_ADDR},
	{ "switch6",	MV88E6020_PORT6_SWITCH_ADDR},
	{ NULL,		0},
};

static int mv88e6020_busychk(struct phy_device *phydev)
{
	u16 reg = 0;
	u32 timeout = MV88E6020_PHY_TIMEOUT;

	/* Poll till SMIBusy bit is clear */
	do {
		reg = mdiobus_read(phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
			MV88E6020_SMI_PHY_ADDR_REG);
		if (timeout-- == 0) {
			printk("SMI busy timeout\n");
			return -1;
		}
	} while (reg & MV88E6020_SMIBUSY);
	return 0;
}

/* reads a register in one of the internal PHY devices */
static int mv88e6020_phy_read(int phynum, u32 regnum)
{
	int err;
	u16 cmd;

	if ( mv88e6020_phydev == NULL) {
		printk("%s: phy not inited\n", __func__);
		return 0;
	}

	cmd = MV88E6020_SMI_PHY_ADDR_READ |
		((MV88E6020_SMI_PHY_ADDR_BASE | phynum) << 5) | regnum;

	if ( mv88e6020_busychk(mv88e6020_phydev) ) {
		printk("%s: switch is busy\n", __func__);
		return 0;
	}
	/* PHY registers accessed indirectly */
	err = mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
			MV88E6020_SMI_PHY_ADDR_REG, cmd);
	if (err < 0) {
		printk("%s: write error %d\n", __func__, err);
		return err;
	}

	if ( mv88e6020_busychk(mv88e6020_phydev) ) {
		printk("%s: switch is busy\n", __func__);
		return -EIO;
	}

	err = mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
			MV88E6020_SMI_PHY_DATA_REG);
	if (err < 0) {
		printk("%s: read error %d\n", __func__, err);
	}
	return err;
}

/* writes a register in one of the internal PHY devices */
static int mv88e6020_phy_write(int phynum, u32 regnum, u16 val)
{
	int err;
	u16 cmd;

	if ( mv88e6020_phydev == NULL) {
		printk("%s: phy not inited\n", __func__);
		return 0;
	}

	/* PHY registers accessed indirectly */
	err = mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
			MV88E6020_SMI_PHY_DATA_REG, val);
	if (err < 0) {
		printk("%s: write error %d\n", __func__, err);
		return err;
	}

	if ( mv88e6020_busychk(mv88e6020_phydev) ) {
		printk("%s: switch is busy\n", __func__);
		return -EIO;
	}

	cmd = MV88E6020_SMI_PHY_ADDR_WRITE |
		((MV88E6020_SMI_PHY_ADDR_BASE | phynum) << 5) | regnum;
	err = mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
			MV88E6020_SMI_PHY_ADDR_REG, cmd);
	if (err < 0) {
		printk("%s: write addr error %d\n", __func__, err);
	}

	if ( mv88e6020_busychk(mv88e6020_phydev) ) {
		printk("%s: switch is busy\n", __func__);
		return -EIO;
	}
	return err;
}

static irqreturn_t mv88e6020_isr(int irq, void *p)
{
	int ret = IRQ_NONE;

	if (irq == mv88e6020_irq) {
		mv88e6020_int_count++;

		disable_irq_nosync(irq);

		/* can't do MDIO accesses in ISR so schedule them */
		schedule_work(&mv88e6020_link_isr_work);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static unsigned long get_stat_value(char port, int opcode) {
	unsigned long ret = 0;
	int count = 0;

	// Make sure the status registers aren't busy.
	while((mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_OP_REG) & MV88E6020_STATS_BUSY) && count < 100) {
		udelay(10);
		count++;
	}
	if((mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_OP_REG) & MV88E6020_STATS_BUSY)) {
		printk(KERN_WARNING "mv88e6020: Status register busy!\n");
		return -EBUSY;
	}

	mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_OP_REG,
				MV88E6020_STATS_BUSY |
				MV88E6020_STATS_READ |
				MV88E6020_STATS_PORT(port) |
				opcode);
	ret = (unsigned long)((mdiobus_read(mv88e6020_phydev->bus,
				MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_READ_REG_HI) << 16) |
				mdiobus_read(mv88e6020_phydev->bus,
				MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_READ_REG_LO));

	return ret;
}

static int flush_stat_regs(void) {
	int ret = 0;
	int count = 0;

	// Make sure the status registers aren't busy.
	while((mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_OP_REG) & MV88E6020_STATS_BUSY) && count < 100) {
		udelay(10);
		count++;
	}
	if((mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_OP_REG) & MV88E6020_STATS_BUSY)) {
		printk(KERN_WARNING "mv88e6020: Status register busy!\n");
		return -EBUSY;
	}

	mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR, MV88E6020_STATS_OP_REG,
				MV88E6020_STATS_BUSY |
				MV88E6020_STATS_FLUSH);

	return ret;
}

/* Get all statistics from the mv88e6020.
 * No locking is used in this function since the device has a
 * busy bit built-in to the control register to ensure that the
 * read operation is finished before another one is started.
 * Also, the mdiobus_read and _write functions lock the bus themselves.
 */
int mv88e6020_get_stats(char port, struct net_device_stats *phy_stats) {
	int temp_stat = 0;
	int i = 0;
	int tries = 5;
	struct counter stats[MV88E6020_STATS_NUM_COUNTERS] = {
		{MV88E6020_STATS_IN_GOOD_OCT, 1, &(phy_stats->rx_bytes)},
		{MV88E6020_STATS_IN_BAD_OCT, 1, &(phy_stats->rx_bytes)},
		{MV88E6020_STATS_IN_MCAST, 1, &(phy_stats->multicast)},
		{MV88E6020_STATS_IN_SMALL, 1, &(phy_stats->rx_length_errors)},
		{MV88E6020_STATS_IN_OVER, 1, &(phy_stats->rx_length_errors)},
		{MV88E6020_STATS_IN_RX_ERR, 1, &(phy_stats->rx_errors)},
		{MV88E6020_STATS_IN_FCS, 1, &(phy_stats->rx_crc_errors)},
		{MV88E6020_STATS_OUT_OCT, 1, &(phy_stats->tx_bytes)},
		{MV88E6020_STATS_OUT_COLL, 1, &(phy_stats->collisions)},
		{MV88E6020_STATS_OUT_SINGL, 1, &(phy_stats->collisions)},
		{MV88E6020_STATS_OUT_MULTI, 1, &(phy_stats->collisions)},
		{MV88E6020_STATS_OUT_FCS, 1, &(phy_stats->tx_errors)},
	};

	memset(phy_stats, 0x00, sizeof(struct net_device_stats));

	/* This loop will attempt to read statistics from the registers.
	 * It will try 5 times to get the register. If it fails all 5 times
	 * on any register, it will give up and return -EBUSY.
	 */
	for(i = 0; i < MV88E6020_STATS_NUM_COUNTERS; i++) {
		tries = 5;
		do {
			temp_stat = get_stat_value(port, stats[i].opcode);
			if (temp_stat != -EBUSY) {
				*(stats[i].element) += temp_stat;
				stats[i].not_set = 0;
			} else {
				tries--;
				udelay(10);
			}
		} while (stats[i].not_set && tries);
		if (tries == 0) {		/* We're giving up, this is taking too long */
			memset(phy_stats, 0x00, sizeof(struct net_device_stats));
			return -EBUSY;
		}
	}

	return 0;
}

static void mv88e6020_check_link(void)
{
	int link, phy;

	/* report link as up if either PHY has link */
	link = 0;
	for (phy = 0; phy <= 1; phy++) {
		int val;
		unsigned int prev_speed = mv88e6020_status[phy].speed;

		val = mv88e6020_phy_read(phy, MV88E6020_REG_STATUS);
		if (val & MV88E6020_RTLINK) {
			link = 1;
			if (val & MV88E6020_PHY_STATUS_FULLDUPLEX)
				mv88e6020_status[phy].duplex = DUPLEX_FULL;
			else
				mv88e6020_status[phy].duplex = DUPLEX_HALF;
			if (val & MV88E6020_PHY_STATUS_100MBPS)
				mv88e6020_status[phy].speed = SPEED_100;
			else
				mv88e6020_status[phy].speed = SPEED_10;
		}
		else {
			mv88e6020_status[phy].duplex = 0;
			mv88e6020_status[phy].speed = 0;
		}

		if (mv88e6020_status[phy].speed != prev_speed) {
			if (mv88e6020_status[phy].speed == 0) {
				printk("mv88e6020: port %d down\n", phy);
			}
			else {
				printk("mv88e6020: port %d up, speed %d,"
					" %s duplex\n", phy,
					mv88e6020_status[phy].speed,
					(mv88e6020_status[phy].duplex ==
						DUPLEX_FULL) ? "full" : "half");
			}
		}
	}

	/* tell internal phy to report link*/
	swicth_to_cpu_link_status = link;

	if (mv88e6020_phydev == NULL) {
		printk("mv88e6020_phydev NULL\n");
	}
	else if (mv88e6020_link_up != NULL) {
		(*mv88e6020_link_up)(mv88e6020_mac_device);
	}
}

void mv88e6020_sonos_phy_connect(void (*link_up)(struct net_device *), struct net_device *ndev)
{
	mv88e6020_link_up = link_up;
	mv88e6020_mac_device = ndev;
	mv88e6020_check_link();
}

void mv88e6020_sonos_phy_disconnect(void)
{
	int phy;
	for (phy = 0; phy <= 1; phy++) {
		mv88e6020_status[phy].duplex = 0;
		mv88e6020_status[phy].speed = 0;
	}
	if (mv88e6020_link_up != NULL)
		(*mv88e6020_link_up)(mv88e6020_mac_device);
	mv88e6020_link_up = NULL;
	mv88e6020_mac_device = NULL;
	cancel_delayed_work(&mv88e6020_errata34_work);
	disable_irq_nosync(mv88e6020_irq);
	free_irq(mv88e6020_irq, NULL);
}

void mv88e6020_sonos_get_port_status(unsigned int port, struct ethtool_cmd *cmd)
{
	*cmd = mv88e6020_status[port];
}

/* Scheduled by ISR to do the actual work. MDIO accesses can't be done
   in the ISR so they're done here. */
static void mv88e6020_link_change(struct work_struct *work)
{
	int gb2int, pass=0;
	int mask = MV88E6020_GLOBAL2_PHY_INT(0) | MV88E6020_GLOBAL2_PHY_INT(1);

	mv88e6020_work_count++;

	gb2int = mask;
	while (gb2int != 0) {
		/* clear all PHY interrupts */
		mv88e6020_phy_read(0, MV88E6020_REG_INT_STATUS);
		mv88e6020_phy_read(1, MV88E6020_REG_INT_STATUS);

		/* report any changes */
		mv88e6020_check_link();

		/* did another PHY interrupt happen? */
		gb2int = mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
							  MV88E6020_PHY_INTR_STATUS_REG);
		gb2int &= mask;

		if (pass++ > 1000) {
			printk("%s: exceeded max attempts to clear interrupt\n", __func__);
		    mv88e6020_max_pass = pass;
			return; /* leave interrupt disabled since something is really wrong */
		}
	}

	enable_irq(mv88e6020_irq);

	if (pass > mv88e6020_max_pass) {
		mv88e6020_max_pass = pass;
	}
}

void mv88e6020_init_irq(void)
{
	int ret;
        struct device_node *node = NULL;

        node = of_find_compatible_node(NULL, NULL, "Sonos,mv88e6020_phy");
        if (node) {
                mv88e6020_irq = gpio_to_irq(of_get_named_gpio(node, "interrupts", NULL));
        }

	if ( mv88e6020_irq < 0 ) {
		printk("%s: no irq\n", __func__);
		return;
	}
	INIT_WORK(&mv88e6020_link_isr_work, mv88e6020_link_change);

	ret = request_irq(mv88e6020_irq, mv88e6020_isr, IRQF_TRIGGER_LOW,
                     "mv88e6020_isr", NULL);
	if (ret) {
		printk("%s: request_irq (virtual %d, hw %d) failed %d\n",
			__func__, mv88e6020_irq, MV88E6020_HWINT_NUM, ret);
	}
}

/* Description: on cable insertion or power on reset the PHYs may experience to link */
static void mv88e6020_errata34(struct work_struct *work)
{
	int phy;

	mv88e6020_errata_passes++;

	/* workaround from Marvell */
	for (phy = 0; phy <= 1; phy++) {
		int val;
		val = mv88e6020_phy_read(phy, MV88E6020_REG_STATUS);
		if ((val & (1<<10)) == 0) {
			val = mv88e6020_phy_read(phy, 0x05);
			if (val != 0) {
//XXX BT inside the #if 0 is Marvell's recommended workaround which doesn't
//XXX seem to actually work, so instead (see the #else) we reset the port,
//XXX which does appear to be effective.
#if 0
				mv88e6020_phy_write(phy, 0x1d, 0x0006);
				mv88e6020_phy_write(phy, 0x1f, 0x0232);
				mv88e6020_phy_write(phy, 0x1f, 0x0032);
#else
				mv88e6020_phy_write(phy, 0x00, 0xb100);
#endif
				mv88e6020_errata_count++;
			}
		}
	}

	/* do this again in a second */
	schedule_delayed_work(&mv88e6020_errata34_work, HZ);
}

static int mv88e6020_proc_read(struct seq_file *m, void *v)
{
	seq_printf(m, "Interupts %ld\n"
			"Interrupt Work %ld\n"
			"Max int pass %d\n"
			"Errata passes %ld\n"
			"Errata workarounds %ld\n",
			mv88e6020_int_count,
			mv88e6020_work_count,
			mv88e6020_max_pass,
			mv88e6020_errata_passes,
			mv88e6020_errata_count);
	return 0;
}

static int mv88e6020_proc_dump(struct seq_file *m, void *v)
{
	int devAddr = (int)m->private;
	int reg, val;

	for (reg = 0; reg <= 0x1f; reg++) {
		val = mdiobus_read(mv88e6020_phydev->bus, devAddr, reg);
		seq_printf(m, "%04x ", val);
		if ((reg & 0x0f) == 0x0f)
			seq_printf(m, "\n");
		else if ((reg & 0x07) == 0x07)
			seq_printf(m, "  ");
	}
	return 0;
}

static int mv88e6020_proc_phydump(struct seq_file *m, void *v)
{
	int phy = (int)m->private;
	int reg, val;

	for (reg = 0; reg <= 0x1f; reg++) {
		val = mv88e6020_phy_read(phy, reg);
		seq_printf(m, "%04x ", val);
		if ((reg & 0x0f) == 0x0f)
			seq_printf(m, "\n");
		else if ((reg & 0x07) == 0x07)
			seq_printf(m, "  ");
	}
	return 0;
}

static int mv88e6020_proc_test(struct seq_file *m, void *v)
{
	int err;
	/* turn off both PHYs */
	err  = mv88e6020_phy_write(0, 0, 0x3900);
	err += mv88e6020_phy_write(1, 0, 0x3900);

	/* turn on both PHYs */
	err += mv88e6020_phy_write(0, 0, 0xb100);
	err += mv88e6020_phy_write(1, 0, 0xb100);

	seq_printf(m, "both PHY links bounced, errors %d\n", err);
	return 0;
}

static int mv88e6020_data_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6020_proc_read, NULL);
}

static int mv88e6020_bounce_links_proc_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, mv88e6020_proc_test, NULL);
}

static int mv88e6020_port_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6020_proc_dump, PDE_DATA(inode));
}

static ssize_t mv88e6020_port_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	int devaddr = (int)PDE_DATA(file_inode(file));
	char buf[200];
	char *peq;
	int result = 0;
	long longtemp;
	u32  regnum;
	u16  val;

	if (count > 200)
		result = -EIO;
	else if (copy_from_user(buf, buffer, count)) {
		result = -EFAULT;
	}

	if ( result == 0 ) {
		buf[count] = '\0';
		peq = strchr(buf, '=');
		if (peq != NULL) {
			*peq = 0;
			if ( strict_strtol(peq+1, 16, &longtemp) != 0 )
				return -EIO;
			val = longtemp;
			if (strncmp(buf, "reg", 3) == 0) {
				if ( strict_strtol(buf+3, 16, &longtemp) != 0 )
					return -EIO;
				regnum = longtemp;
				mdiobus_write(mv88e6020_phydev->bus, devaddr,
					regnum, val);
			}
		}
		result = count;
	}
	return result;
}

static int mv88e6020_phy_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6020_proc_phydump, PDE_DATA(inode));
}

static ssize_t mv88e6020_phy_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	int phynum = (int)PDE_DATA(file_inode(file));
	char buf[200];
	char *peq;
	int result = 0;
	long longtemp;
	u32  regnum;
	u16  val;

	if (count > 200)
		result = -EIO;
	else if (copy_from_user(buf, buffer, count)) {
		result = -EFAULT;
	}

	if ( result == 0 ) {
		buf[count] = '\0';
		peq = strchr(buf, '=');
		if (peq != NULL) {
			*peq = 0;
			if ( strict_strtol(peq+1, 16, &longtemp) != 0 )
				return -EIO;
			val = longtemp;
			if (strncmp(buf, "reg", 3) == 0) {
				if (strict_strtol(buf+3, 16, &longtemp) != 0 )
					return -EIO;
				regnum = longtemp;
				mv88e6020_phy_write(phynum, regnum, val);
			}
		}
		result = count;
	}
	return result;
}

static const struct file_operations mv88e6020_data_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= mv88e6020_data_proc_open,
	.read		= seq_read,
	.release	= single_release,
};

static const struct file_operations mv88e6020_bounce_links_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= mv88e6020_bounce_links_proc_open,
	.read		= seq_read,
	.release	= single_release,
};

static const struct file_operations mv88e6020_port_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= mv88e6020_port_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= mv88e6020_port_proc_write,
};

static const struct file_operations mv88e6020_phy_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= mv88e6020_phy_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= mv88e6020_phy_proc_write,
};

static int m88e6020_config_init(struct phy_device *phydev)
{
	int phy, err, ident;

	mv88e6020_phydev = phydev;

	/* disable energy detection */
	for (phy = 0; phy <= 1; phy++) {
		int scr;
		scr = mv88e6020_phy_read(phy, MV88E6020_REG_PHY_SPEC_CNTL);
		if (scr & 0x4000) {
			printk("%s: phy%d: disabling energy detect only\n",
				__func__, phy);
                        mv88e6020_phy_write(phy, MV88E6020_REG_PHY_SPEC_CNTL,
				0x0138);
		} else {
			printk("%s: phy%d: energy detect already disabled\n",
				__func__, phy);
		}
	}

	/* disable switch port6 which is not used */
	err = mdiobus_write(mv88e6020_phydev->bus, MV88E6020_PORT6_SWITCH_ADDR,
			MV88E6020_SWITCH_REG_PORT_CONTROL, 0);
	if (err < 0) {
		 printk("%s: disable port6 write error %d\n", __func__, err);
	}

	for (phy = 0; phy <= 1; phy++) {
		int id2, id3;
		id2 = mv88e6020_phy_read(phy, MII_PHYSID1);
		id3 = mv88e6020_phy_read(phy, MII_PHYSID2);
		printk("%s: PHY%d: ID %04x %04x\n", __func__, phy, id2, id3);
	}

	ident = mdiobus_read(mv88e6020_phydev->bus, MV88E6020_PORT0_SWITCH_ADDR,
			MV88E6020_SWITCH_REG_IDENT);
	printk("switch port0: product number 0x%03x, revision 0x%1x\n",
			(ident >> 4), (ident & 0xf));
	ident = mdiobus_read(mv88e6020_phydev->bus, MV88E6020_PORT1_SWITCH_ADDR,
			MV88E6020_SWITCH_REG_IDENT);
	printk("switch port1: product number 0x%03x, revision 0x%1x\n",
			(ident >> 4), (ident & 0xf));
	ident = mdiobus_read(mv88e6020_phydev->bus, MV88E6020_PORT5_SWITCH_ADDR,
			MV88E6020_SWITCH_REG_IDENT);
	printk("switch port5: product number 0x%03x, revision 0x%1x\n",
			(ident >> 4), (ident & 0xf));

	/* clear all interrupts */
	mdiobus_read(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR,
			MV88E6020_REG_SWITCH_INTR_STATUS);
	mv88e6020_phy_read(0, MV88E6020_REG_INT_STATUS);
	mv88e6020_phy_read(1, MV88E6020_REG_INT_STATUS);

	for (phy = 0; phy <= 1; phy++) {
		/* enable PHY link interrupts in PHY */
		mv88e6020_phy_write(phy, MV88E6020_REG_INT_MASK,
			MV88E6020_INT_LINK);
	}

	/* enable interrupts from PHY0 and PHY1 */
	err = mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL2_REGS_ADDR,
			MV88E6020_PHY_INTR_MASK_REG,
			MV88E6020_GLOBAL2_PHY_INT(0) |
			MV88E6020_GLOBAL2_PHY_INT(1));
	if (err < 0) {
		printk("%s: write error %d\n", __func__, err);
	}

	/* enable device interrupts */
	err = mdiobus_write(mv88e6020_phydev->bus, MV88E6020_GLOBAL1_REGS_ADDR,
			MV88E6020_REG_SWITCH_INTR_MASK, MV88E6020_DEVICE_INT);
	if (err < 0) {
		printk("%s: write error %d\n", __func__, err);
	}

	/* hook interrupt with linux */
	mv88e6020_init_irq();

	/* get the current state of the links */
	mv88e6020_check_link();

	/*
	 * Clear statistics counters for all ports
	 * These counters are otherwise persistent across soft-reboot
	 * and are only cleared by a power cycle.
	 */
	if (flush_stat_regs())
		printk(KERN_WARNING "%s: Couldn't flush statistics registers.\n", __func__);

	/* perform the errata 3.4 workaround every second */
	INIT_DELAYED_WORK(&mv88e6020_errata34_work, mv88e6020_errata34);
	schedule_delayed_work(&mv88e6020_errata34_work, HZ);

	return 0;
}

static int m88e6020_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int m88e6020_genphy_read_status(struct phy_device *phydev)
{
	if ( swicth_to_cpu_link_status ) {
		phydev->speed = SPEED_100;
		phydev->duplex = DUPLEX_FULL;
		phydev->link = 1;
	} else {
		phydev->speed = 0;
		phydev->duplex = -1;
		phydev->link = 0;
	}
	return 0;
}

static int m88e6020_ack_interrupt(struct phy_device *phydev)
{
	return 0;
}

static int m88e6020_config_intr(struct phy_device *phydev)
{
	return 0;
}

static struct phy_driver marvell_drivers[] = {
	{
		.phy_id = 0x00000200,
		.phy_id_mask = 0x000fff00,
		.name = "Marvell 88E6020",
		.features = PHY_GBIT_FEATURES,
		.flags = PHY_HAS_INTERRUPT,
		.config_init = &m88e6020_config_init,
		.config_aneg = &m88e6020_config_aneg,
		.read_status = &m88e6020_genphy_read_status,
		.ack_interrupt = &m88e6020_ack_interrupt,
		.config_intr = &m88e6020_config_intr,
		.driver = { .owner = THIS_MODULE },
	},
};

static int __init mv88e6020_init(void)
{
	int ret = 0;
	mv88e6020_env_t *entry;
	int var_num;

#ifdef CONFIG_SONOS_SOLBASE
	if ( sys_mdp.mdp_revision >= MDP_REVISION_SOLBASE_PROTO4 ) {
		return 0;
	}
#endif

	mv88e6020_phydev = NULL;
	ret = phy_drivers_register(marvell_drivers,
		ARRAY_SIZE(marvell_drivers));

	if ( ret )
		goto err;

	swicth_to_cpu_link_status = 0;
	procdir = proc_mkdir(MV88E6020_PROC_DIR, NULL);
	if (procdir == NULL) {
		printk("Couldn't create base dir /proc/%s\n",
			MV88E6020_PROC_DIR);
		phy_drivers_unregister(marvell_drivers,
			ARRAY_SIZE(marvell_drivers));
		return -ENOMEM;
	}

	if (! proc_create_data("data", 0, procdir,
		&mv88e6020_data_proc_fops, 0)) {
		printk("mv88e6020/data not created\n");
		goto cleanup;
	}

	if ( !proc_create_data("bounce-links", 0, procdir,
		&mv88e6020_bounce_links_proc_fops, 0)) {
		printk("mv88e6020/bounce-links not created\n");
		goto cleanup;
	}

	/*
	 * Create all named nodes
	 */
	entry = mv88e6020_named_entries;
	while (entry->name && entry->id) {
		if (!proc_create_data(entry->name, 0644, procdir,
			&mv88e6020_port_proc_fops, (void *)entry->id))
			goto cleanup;
		entry++;
	}

	for (var_num = 0; var_num <= 1; var_num++) {
		char name[8];
		sprintf(name, "phy%d", var_num);
		if (!proc_create_data(name, 0644, procdir,
			&mv88e6020_phy_proc_fops, (void *)var_num))
			 goto cleanup;
	}
	return 0;
cleanup:
	phy_drivers_unregister(marvell_drivers,
			ARRAY_SIZE(marvell_drivers));
	remove_proc_subtree(MV88E6020_PROC_DIR, NULL);
err:
	return ret;
}

static void __exit mv88e6020_exit(void)
{
	printk("%s: done\n", __func__);
#ifdef CONFIG_SONOS_SOLBASE
	if ( sys_mdp.mdp_revision >= MDP_REVISION_SOLBASE_PROTO4 ) {
		return;
	}
#endif
	remove_proc_subtree(MV88E6020_PROC_DIR, NULL);
	phy_drivers_unregister(marvell_drivers,
			ARRAY_SIZE(marvell_drivers));
	mv88e6020_phydev = NULL;
}

#ifdef CONFIG_SONOS_PARAMOUNT
// stub it for now
int phy_power_down(struct phy_device *phydev, int on)
{
    return 0;
}
#endif

module_init(mv88e6020_init);
module_exit(mv88e6020_exit);

static struct mdio_device_id __maybe_unused marvell_tbl[] = {
	{ 0x00000200, 0x0000fff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, marvell_tbl);
