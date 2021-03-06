/*
 * Freescale board control FPGA.
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Tang Yuantian <b29983@freescale.com>
 *
 * Based on code wrote by Mingkai hu <Mingkai.hu@freescale.com>
 * Based on the bare code wrote by Chris Pettinato (ra5171@freescale.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <asm/machdep.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <asm/fsl_pixis.h>

static struct proc_dir_entry *pixis_proc_root;
static struct proc_dir_entry *pixis_proc_pm_ctrl;
static struct fsl_pixis *pixis;
static int ACK_TIMEOUT = 50;	/* unit:ms */

/* table for positive exponent */
static int pmbus_2power_pos[] = {
	1,					/* 0000_0 => 0 */
	2,					/* 0000_1 => 1 */
	4,					/* 0001_0 => 2 */
	8,					/* 0001_1 => 3 */
	16,					/* 0010_0 => 4 */
	32,					/* 0010_1 => 5 */
	64,					/* 0011_0 => 6 */
	128,				/* 0011_1 => 7 */
	256,				/* 0100_0 => 8 */
	512,				/* 0100_1 => 9 */
	1024,				/* 0101_0 => 10 */
	2048,				/* 0101_1 => 11 */
	4096,				/* 0110_0 => 12 */
	8192,				/* 0110_1 => 13 */
	16384,				/* 0111_0 => 14 */
	32678				/* 0111_1 => 15 */
};

/*
 * table for negative exponent.
 * amplified by 1000,000 times.
 */
static int pmbus_2power_neg[] = {
	15,					/* 1000_0 => 16 (-16) */
	30,					/* 1000_1 => 17 (-15) */
	61,					/* 1001_0 => 18 (-14) */
	122,				/* 1001_1 => 19 (-13) */
	244,				/* 1010_0 => 20 (-12) */
	488,				/* 1010_1 => 21 (-11) */
	976,				/* 1011_0 => 22 (-10) */
	1953,				/* 1011_1 => 23 (-9) */
	3906,				/* 1100_0 => 24 (-8) */
	7812,				/* 1100_1 => 25 (-7) */
	15625,				/* 1101_0 => 26 (-6) */
	31250,				/* 1101_1 => 27 (-5) */
	62500,				/* 1110_0 => 28 (-4) */
	125000, 			/* 1110_1 => 29 (-3) */
	250000, 			/* 1111_0 => 30 (-2) */
	500000 				/* 1111_1 => 31 (-1) */
};

/*
 * 1. send VOUT_MODE to get data mode and exponent.
 *	 VOUT is LINEAR mode with exponent value -13.
 * 2. send READ_VOUT to get mantissa.
 *
 * return unit: mv
 */
int pmbus_2volt(int v)
{
	int d;

	v &= 0xFFFF;
	d =  v * pmbus_2power_neg[3] / DIVIDE_FACTOR;

	return d;
}
EXPORT_SYMBOL(pmbus_2volt);

/*
 * IOUT is Literal data format.
 * 0x1111 1111 1111 1111
 *   ||   |||		   |
 *   | -+- | ----+-----
 *	 |  |  |	 +--> Mantissa X
 *	 |	|  +--------> Mantissa sign
 *	 |  +-----------> Exponent
 *	 +--------------> Exponent sign
 *
 * return unit: mA
 */
int pmbus_2cur(int a)
{
	int d;
	int n, y;

	n = (a >> 11) & 0xF;
	y = a & 0x7FF;

	if (a & 0x8000)
		d = y * pmbus_2power_neg[n] / DIVIDE_FACTOR;
	else
		d = y * pmbus_2power_pos[n] * DIVIDE_FACTOR;

	return d;
}
EXPORT_SYMBOL(pmbus_2cur);

/*
 * temperature is Literal data format.
 *
 * return unit: C
 */
static int pmbus_2temp(int t)
{
	int d;
	int n, y;

	n = (t >> 11) & 0xF;
	y = t & 0x7FF;

	if (t & 0x8000)
		d = y * pmbus_2power_neg[n] / DIVIDE_FACTOR / DIVIDE_FACTOR;
	else
		d = y * pmbus_2power_pos[n];

	return d;
}

static void pixis_setreg(u8 *reg, u8 val)
{
	out_8(reg, val);
}

static u8 pixis_getreg(u8 *reg)
{
	return in_8(reg);
}

static void gmsa_write_sram(u8 offset, u8 v)
{
	pixis_setreg(&pixis->base->addr, offset);
	pixis_setreg(&pixis->base->data, v);
}

static u8 gmsa_read_sram(u8 offset)
{
	pixis_setreg(&pixis->base->addr, offset);

	return pixis_getreg(&pixis->base->data);
}

static int gmsa_write_sram_block(u8 addr, int len, ...)
{
	int v;
	int n;
	va_list args;

	va_start(args, len);
	for (n = 0; n < len; n++) {
		v = va_arg(args, int);
		gmsa_write_sram(addr, (u8)v);
		addr++;
	}
	va_end(args);

	return n;
}

static int gmsa_sram_copy(int sram_addr, u8 *buf, int len)
{
	int i;

	if (sram_addr + len > 0xff) {
		pr_err("The addr is out of bound.\n");
		return 1;
	}

	for (i = 0; i < len; i++)
		buf[i] = gmsa_read_sram(sram_addr + i);

	return 0;
}

static int gmsa_check_may_send(void)
{
	u8 a, c;

	c = pixis_getreg(&pixis->base->ocmd);
	a = pixis_getreg(&pixis->base->mack);

	if ((c & PXOC_MSG) || (a & PXMA_ACK)) {
		pr_debug("PX_OCMD[%02X] or PX_MACK[%02X]"
				"is not ready.\n", c, a);
		return 1;
	}

	return 0;
}

/*
 * Start to execute the CMD in OCM.
 * @addr: the address from which the CMD serials are stored.
 */
static int gmsa_send_message(int addr)
{
	u8	v;
	int i;


	/* 1. set bit 0 of PX_OCMD to start CMD */
	pixis_setreg(&pixis->base->omsg, addr);
	pixis_setreg(&pixis->base->ocmd, PXOC_MSG);

	/* 2. wait for ack */
	for (i = 0; i < ACK_TIMEOUT; i++) {
		v = pixis_getreg(&pixis->base->mack);
		if (v & PXMA_ACK) {
			pr_debug("OCM acked @time=%d\n", i);
			break;
		}
		mdelay(1);
	}

	/* 3. check the error */
	if ((i == ACK_TIMEOUT) || (v & PXMA_ERR)) {
		pr_err("OCM Ack err or timeout.\n");
		return 1;
	}

	/* 4. clear the bit 0 of PX_OCMD */
	pixis_setreg(&pixis->base->ocmd, 0);

	/* 5. wait for OCM to stop */
	for (i = 0; i < ACK_TIMEOUT; i++) {
		v = pixis_getreg(&pixis->base->mack);
		if (!(v & PXMA_ACK)) {
			pr_debug("OCM ack clear @time=%d\n", i);
			break;
		}
		mdelay(1);
	}

	/* 6. check the error */
	if ((i == ACK_TIMEOUT) || (v & PXMA_ERR)) {
		pr_err("OCM Ack err or timeout.\n");
		return 1;
	}

	return 0;
}

static int pixis_set_timer(int rate)
{
	int v;

	v = gmsa_check_may_send();
	if (v) {
		pr_err("OCM is not ready.\n");
		return v;
	}

	gmsa_write_sram_block(0x0, 6, OM_TIMER, 0,
			rate & 0xff, 255, 255, OM_END);
	gmsa_send_message(0x0);

	return 0;
}

static int pixis_enable_channel(int mask)
{
	int v;

	v = gmsa_check_may_send();
	if (v) {
		pr_err("OCM is not ready.\n");
		return v;
	}

	gmsa_write_sram_block(0x0, 4, OM_ENABLE,
			((mask >> 8) & 0xFF), mask & 0xFF, OM_END);
	gmsa_send_message(0x0);

	return 0;
}

static int pixis_start_pm(void)
{
	u8	val;
	int v;

	val = gmsa_read_sram(0x02);
	if (val == OM_START)
		pr_info("PM is already running, Restart...\n");

	v = gmsa_check_may_send();
	if (v) {
		pr_err("OCM is not ready.\n");
		return v;
	}

	gmsa_write_sram_block(0x00, 4, OM_STOP, OM_SCLR, OM_START, OM_END);
	gmsa_send_message(0x0);

	return 0;
}

static int pixis_stop_pm(void)
{
	u8 val;
	int v;
	u8 *buf = (u8 *)&pixis->rec[0];

	val = gmsa_read_sram(0x02);
	if (val != OM_START) {
		pr_err("PM is not running.\n");
		return 0;
	}

	v = gmsa_check_may_send();
	if (v) {
		pr_err("OCM is not ready.\n");
		return v;
	}
	gmsa_write_sram_block(0x0, 4, OM_STOP, OM_GET, DATA_ADDR, OM_END);
	gmsa_send_message(0x0);
	gmsa_sram_copy(DATA_ADDR, buf, sizeof(struct crecord)*REC_NUM);

	return 0;
}

/*
 * These two functions are provided for Power Manamgement
 * code to call it in sleep mode.
 */
int pixis_start_pm_sleep(void)
{
	int status = 0;

	if (pixis->pm_cmd == PX_CMD_SLEEP) {
		pr_info("Start PM in sleep mode...\n");
		status = pixis_start_pm();
	}

	if (status)
		pr_err("PIXIS: fail to start PM in sleep mode.\n");

	return status;
}
EXPORT_SYMBOL(pixis_start_pm_sleep);

int pixis_stop_pm_sleep(void)
{
	int status = 0;

	if (pixis->pm_cmd == PX_CMD_SLEEP) {
		pr_info("Stop PM in sleep mode...\n");
		status = pixis_stop_pm();
	}

	pixis->pm_cmd = PX_CMD_STOP;
	if (status)
		pr_err("PIXIS: fail to stop PM in sleep mode.\n");

	return status;
}
EXPORT_SYMBOL(pixis_stop_pm_sleep);

static int pixis_proc_write_ctrl(struct file *file, const char __user *buffer,
				unsigned long count, void *data)
{
	u8 loc_buf[10];
	int err;
	unsigned long pm_cmd;

	if (count >= 10) {
		pr_err("Input string is too long.\n");
		return -EIO;
	}

	if (copy_from_user(loc_buf, buffer, count))
		return -EFAULT;

	loc_buf[count] = '\0';
	err = strict_strtoul(loc_buf, 10, &pm_cmd);
	if (err)
		return err;

	pixis->pm_cmd = (u32)pm_cmd;

	switch (pixis->pm_cmd) {
	case PX_CMD_START:
		memcpy(pixis->mode, "running", 8);
		pr_info("Start power monitoring...\n");
		pixis_start_pm();
		break;
	case PX_CMD_STOP:
		memcpy(pixis->mode, "stopped", 8);
		pr_info("Stop power monitoring...\n");
		pixis_stop_pm();
		break;
	case PX_CMD_SLEEP:
		memcpy(pixis->mode, "sleep", 6);
		pr_info("Prepare to start PM in sleep mode...\n");
		break;
	default:
		return -EIO;
	}

	return count;
}

/*
 * pm_status
 */
static int pixis_proc_show_status(struct seq_file *m, void *v)
{
	seq_putc(m, '\n');
	seq_printf(m, "  System ID       :    0x%02X\n", pixis_getreg(&pixis->base->id));
	seq_printf(m, "  System version  :    0x%02X\n", pixis_getreg(&pixis->base->arch));
	seq_printf(m, "  FPGA version    :    0x%02X\n", pixis_getreg(&pixis->base->scver));
	seq_printf(m, "  Board name      :    P1022DS\n");
	seq_putc(m, '\n');
	seq_printf(m, "  The PM is in %s mode.\n", pixis->mode);
	seq_putc(m, '\n');

	return 0;
}

static int pixis_proc_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, pixis_proc_show_status, NULL);
}

static const struct file_operations pixis_proc_status_fops = {
	.open		= pixis_proc_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * pm_results
 */
static int gmsa_snapshot_report(struct seq_file *m,
		int indx, int fmt, char *name, char *unit)
{
	int qty, samples, value, avg;

	qty = (pixis->rec[indx].qty1 << 8) | (pixis->rec[indx].qty2);
	seq_printf(m, "  %-6s  |  %5d    %04X  %04X  %8X  | ", name, qty,
			pixis->rec[indx].curr,
			pixis->rec[indx].max,
			pixis->rec[indx].acc);

	samples = qty ? qty : 1;
	switch (fmt) {
	case VOLT_FMT:
		value = pmbus_2volt(pixis->rec[indx].curr);
		avg = pmbus_2volt(pixis->rec[indx].acc / samples);
		break;
	case CURR_FMT:
		value = pmbus_2cur(pixis->rec[indx].curr);
		avg = pmbus_2cur(pixis->rec[indx].acc / samples);
		break;
	case TEMP_FMT:
		value = pmbus_2temp(pixis->rec[indx].curr);
		avg = pmbus_2temp(pixis->rec[indx].acc / samples);
		break;
	default:
		value = 0;
		avg = 0;
		break;
	}

	seq_printf(m, "%5d %s  %5d %s\n", value, unit, avg, unit);

	return 0;
}
static int pixis_proc_show_result(struct seq_file *m, void *v)
{
	int i;
	int   fmt[] = {0, 1, 0, 1, 0, 1, 0, 1, 2};
	char *name[] = {"VDD", "IVDD", "SVDD", "SIDD", "OVDD", "OIDD", "GVDD", "GIDD", "TEMP"};
	char *unit[] = {"mV", "mA", "mV", "mA", "mV", "mA", "mV", "mA", " C"};

	seq_putc(m, '\n');
	seq_printf(m, "  Name    |  Samples  CURR   MAX       SUM       Value      Avg.\n"
				  "  ======  |  =======  ====  ====   =======     =======   =======\n");

	for (i = 0; i < REC_NUM; i++)
		gmsa_snapshot_report(m, i, fmt[i], name[i], unit[i]);

	seq_putc(m, '\n');

	return 0;
}

static int pixis_proc_result_open(struct inode *inode, struct file *file)
{
	return single_open(file, pixis_proc_show_result, NULL);
}

static const struct file_operations pixis_proc_result_fops = {
	.open		= pixis_proc_result_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init fsl_pixis_init(void)
{
	struct resource r;
	struct device_node *np;

	pixis = kmalloc(sizeof(struct fsl_pixis), GFP_KERNEL);
	if (!pixis)
		return -ENOMEM;

	memset(pixis, 0, sizeof(*pixis));
	memcpy(pixis->mode, "stopped", 8);

	np = of_find_compatible_node(NULL, NULL, "fsl,p1022ds-fpga-pixis");
	if (!np) {
		pr_err("PIXIS: can't find device node"
				" 'fsl,p1022ds-fpga-pixis'\n");
		kfree(pixis);
		return -ENXIO;
	}
	of_address_to_resource(np, 0, &r);
	of_node_put(np);

	pixis->base = ioremap(r.start, r.end - r.start + 1);
	if (!pixis->base) {
		pr_err("PIXIS: can't map FPGA cfg register!\n");
		kfree(pixis);
		return -EIO;
	}

	/* Create the /proc entry */
	pixis_proc_root = proc_mkdir("pixis_ctrl", NULL);
	if (!pixis_proc_root) {
		pr_err("PIXIS: failed to create pixis_ctrl entry.\n");
		kfree(pixis);
		iounmap(pixis->base);
		return -ENOMEM;
	}

	pixis_proc_pm_ctrl = create_proc_entry("pm_control", S_IRUGO | S_IWUSR,
							pixis_proc_root);
	pixis_proc_pm_ctrl->write_proc = pixis_proc_write_ctrl;
	proc_create("pm_result", 0, pixis_proc_root, &pixis_proc_result_fops);
	proc_create("pm_status", 0, pixis_proc_root, &pixis_proc_status_fops);

	/* enable 9 channel */
	pixis_enable_channel(0x1FF);
	/* set timer 240, 1 sample/SEC */
	pixis_set_timer(240);

	return 0;
}

static void __exit fsl_pixis_exit(void)
{
	iounmap(pixis->base);
	kfree(pixis);
	remove_proc_entry("pm_control", pixis_proc_root);
	remove_proc_entry("pm_result", pixis_proc_root);
	remove_proc_entry("pm_status", pixis_proc_root);
	remove_proc_entry("pixis_ctrl", NULL);
}

MODULE_AUTHOR("Tang Yuantian <b29983@freescale.com>");
MODULE_DESCRIPTION("Freescale board control FPGA driver");
MODULE_LICENSE("GPL");

module_init(fsl_pixis_init);
module_exit(fsl_pixis_exit);
