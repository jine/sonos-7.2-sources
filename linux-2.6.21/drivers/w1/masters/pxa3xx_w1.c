/*
 *	pxa3xx_w1.c
 *
 * Copyright (c) 2006 Stanley Cai <stanley.cai@intel.com>
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include "../w1.h"
#include "../w1_int.h"

static struct w1_bus_master *pxa3xx_w1_bus_master;

static int pxa3xx_w1_wait_status(unsigned int active_status,
				 unsigned int deactive_status,
				 unsigned int timeout)
{
	volatile unsigned int intr;
	
	while (timeout--) {
		msleep(2);
		intr = W1INTR;
		if ((intr & active_status) == active_status &&
		   !(intr & deactive_status))
			return intr;
		pr_debug("timeout %d intr=0x%08x expect (active:0x%08x deactive:0x%08x)\n",
			timeout,intr,active_status,deactive_status);
	}
	return -ETIME;// impossible interrupt status
}

static int one_wire_analysis_search_rom_accelerator(unsigned char *receive_data,
						    unsigned char* transmit_data,
						    unsigned char * rom_id)
{
	int loop;
	int result = 1;
	unsigned char trom[64];   /* the transmit value being generated */
	unsigned char rrom[64];   /* the ROM recovered from the received data */
	unsigned char rdis[64];   /* the discrepancy bits in the received data */
	static unsigned char last_dis; /* last discrepancy pos */

	if(receive_data == NULL){
		/* first run initialize */
		last_dis=64; /*different from 0~ 63 is ok */
		memset(transmit_data,0,16);
		return 1;
	}
	/* de-interleave the received data into ROM code and the discrepancy bits */
	for (loop = 0; loop < 16; loop++){
		rrom[loop*4]=(receive_data[loop] & 0x02) ? 1:0;
		rrom[loop*4+1]=(receive_data[loop] & 0x08) ? 1:0;
		rrom[loop*4+2]=(receive_data[loop] & 0x20) ? 1:0;
		rrom[loop*4+3]=(receive_data[loop] & 0x80) ? 1:0;

		rdis[loop*4]=(receive_data[loop] & 0x01) ? 1:0;
		rdis[loop*4+1]=(receive_data[loop] & 0x04) ? 1:0;
		rdis[loop*4+2]=(receive_data[loop] & 0x10) ? 1:0;
		rdis[loop*4+3]=(receive_data[loop] & 0x40) ? 1:0;
	}	

	/* Generate ROM ID for next search if possible */
	memcpy(trom,rrom,64);
	for(loop = 63; loop >= 0; loop--){
		/* a new discrepancy bit */
		if ((rdis[loop]==1) && (loop != last_dis)){
			last_dis=loop;
			trom[loop]=trom[loop]?0:1; /* flip */
			result=0;
			break;
		}
	}

	/* Convert into SRA 16 bytes mode for next search
	 * It will be same rom id if now new rom found
	 */
	for (loop = 0; loop < 16; loop++)
		transmit_data[loop] = (trom[loop*4] << 1) +
				      (trom[loop*4+1]<<3) +
	                     	      (trom[loop*4+2]<<5) +
				      (trom[loop*4+3]<<7);

	/* Convert into ROM 64 bit mode */
	for (loop = 0; loop < 8; loop++){
		rom_id[loop] = (rrom[loop*8]) + (rrom[loop*8+1]<<1) +
	                (rrom[loop*8+2]<<2) +  (rrom[loop*8+3]<<3) +
			(rrom[loop*8+4]<<4) +  (rrom[loop*8+5]<<5) +
	                (rrom[loop*8+6]<<6) +  (rrom[loop*8+7]<<7);
	}
	return result;
}

static u8 pxa3xx_w1_reset(void *data)
{
	int status;

	W1CMDR = W1CMDR_RESET;
	status = pxa3xx_w1_wait_status(W1INTR_PD, W1INTR_PDR, 1);
	if (status == -ETIME)
		return 1;

	return status & W1INTR_PDR;
}

static void pxa3xx_w1_search(void *data, u8 id, w1_slave_found_callback cb)
{
	int loop1,loop2;
	int count=0;
	int ret;
	unsigned char tdata[16];
	unsigned char rdata[16];
	u64 rom_id;
	
	one_wire_analysis_search_rom_accelerator(NULL,tdata,NULL);

	/* run once for max rom number */
	for (loop1 = 0; loop1 < 10; loop1++){
		if (pxa3xx_w1_reset(data)){
			pr_debug("reset failed (no 1-wire responsed)!\n");
			break;
		}
		pr_debug("one wire search command 0xf0\n");
		W1TRR = W1_SEARCH;
		if (pxa3xx_w1_wait_status(W1INTR_TBE | W1INTR_TEMT, 0, 2) == -ETIME) {
			pr_debug("command 0xf0 failed\n");
			break;
		}
		pr_debug("one wire enter sra mode\n");
		/* enter Accelerator mode */
		W1CMDR |= W1CMDR_SRA;

		/* transmit the tdata and receive rdata */
		for (loop2 = 0; loop2 < 16; loop2++){
			/* pr_debug("write SRA data 0x%02x\n",(int)tdata[loop2]); */
			W1TRR = tdata[loop2];
			if (pxa3xx_w1_wait_status(W1INTR_RBF, 0, 2) == -ETIME){
				pr_debug("command SRA failed\n");
				break;
			}
			rdata[loop2] = W1TRR;
			/* pr_debug("read SRA 0x%02x\n",(int)rdata[loop2]); */
		}
		if (loop2 < 16){
			count = 0;
			break;
		}
		/* decode recovered ROM and generate next Search value
		 * if code reach here at least found one !
		 */
		count++;
		ret = one_wire_analysis_search_rom_accelerator
				(rdata, tdata,(unsigned char*)&rom_id);
		cb((void *)data, rom_id);
		if (ret)
			break;
	}
	pr_debug("totally found %d 1-wire rom(s)\n",count);
}

static u8 pxa3xx_w1_read_byte(void *data)
{
	u8 byte;

	W1TRR = 0xFF;
	if (pxa3xx_w1_wait_status(W1INTR_RBF, 0, 2) == -ETIME) {
		pr_debug("pxa3xx_w1_read_byte() time out!\n");
		return 0;
	}

	byte = W1TRR;

	return byte;
}

static void pxa3xx_w1_write_byte(void *data, u8 byte)
{
	W1TRR = byte;
	if (pxa3xx_w1_wait_status(W1INTR_TBE | W1INTR_TEMT, 0, 2) == -ETIME) {
		pr_debug("pxa3xx_w1_write_byte() time out!\n");

		return;
	}
}

static void pxa3xx_w1_write_bit(void *data, u8 bit)
{
	if (bit)
		W1CMDR |= (0x1 << 2);
	else
		W1CMDR &= ~(0x1 << 2);
}

static u8 pxa3xx_w1_read_bit(void *data)
{
	return W1CMDR & (0x1 << 3);
}

static int __devinit pxa3xx_w1_init(void)
{
	int err;

	pxa_set_cken(CKEN_1WIRE, 1);
	/* pxa3xx frequency divisor; */
	W1CDR = 3 | (2 << 2);
	W1IER = 0x00;
	pxa3xx_w1_bus_master = kmalloc(sizeof(*pxa3xx_w1_bus_master), GFP_KERNEL);
	if (!pxa3xx_w1_bus_master) {
		printk(KERN_ERR 
			"Failed to allocate pxa3xx 1-wire bus_master structure.\n");
		return -ENOMEM;
	}

	memset(pxa3xx_w1_bus_master, 0, sizeof(*pxa3xx_w1_bus_master));

	pxa3xx_w1_bus_master->read_byte	 = pxa3xx_w1_read_byte;
	pxa3xx_w1_bus_master->write_byte = pxa3xx_w1_write_byte;
	pxa3xx_w1_bus_master->reset_bus	 = pxa3xx_w1_reset;
	pxa3xx_w1_bus_master->read_bit	 = pxa3xx_w1_read_bit;
	pxa3xx_w1_bus_master->write_bit	 = pxa3xx_w1_write_bit;
	pxa3xx_w1_bus_master->search	 = pxa3xx_w1_search;
	pxa3xx_w1_bus_master->data	 = pxa3xx_w1_bus_master;

	err = w1_add_master_device(pxa3xx_w1_bus_master);
	if (err)
		goto err_out_free_bus_master;

	return 0;

err_out_free_bus_master:
	kfree(pxa3xx_w1_bus_master);

	return err;
}

static void __devexit pxa3xx_w1_exit(void)
{
	w1_remove_master_device(pxa3xx_w1_bus_master);
	kfree(pxa3xx_w1_bus_master);
}

module_init(pxa3xx_w1_init);
module_exit(pxa3xx_w1_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanley Cai <stanley.cai@intel.com>");
