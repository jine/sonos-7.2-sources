/*  pxa_mved - main file for mved driver
 *
 *  Copyright (C) 2006, Intel Corporation.
 *  Copyright (C) 2007, Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */ 
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/memory.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <asm/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/cpu-freq-voltage-pxa3xx.h>
#include <linux/mm.h>
#include <asm-arm/memory.h>
#include <asm-arm/arch-pxa/ipmc.h>
#include <linux/timer.h>
#include <asm/arch/pxa_mved.h>
 
#ifdef CONFIG_IMM
#include <asm/arch/imm.h>
#endif

#define MVED_REG(x)     	(*((volatile unsigned int *)(mved_base+((x)-0x56000000))))
#define MVED_DMA_REG(x)     	(*((volatile unsigned int *)(mved_dma_base+((x)-0x56100000))))
#define MVED_BPB2IMG_REG(x)	(*((volatile unsigned int *)(mved_bpb2img_base+((x)-0x56200000))))

#define IRQ_MVED		PXA_IRQ(54)
#define MVED_INTSTAT		MVED_REG(0x56000004)
#define MVED_INTENAB		MVED_REG(0x56000008)
#define MVED_INTCLEAR		MVED_REG(0x5600000c)
#define MVED_CLKGATE		MVED_REG(0x56000010)

#define IRQ_MVED_DMA		PXA_IRQ(48)
#define MVED_DMA_DCSR0		MVED_DMA_REG(0x56100000)
#define MVED_DMA_DCSR1		MVED_DMA_REG(0x56100004)
#define MVED_DMA_DCSR2		MVED_DMA_REG(0x56100008)
#define MVED_DMA_DALGN		MVED_DMA_REG(0x561000a0)
#define MVED_DMA_DPCSR		MVED_DMA_REG(0x561000a4)
#define MVED_DMA_DRQSR0		MVED_DMA_REG(0x561000e0)
#define MVED_DMA_DRQSR1		MVED_DMA_REG(0x561000e4)
#define MVED_DMA_DRQSR2		MVED_DMA_REG(0x561000e8)
#define MVED_DMA_DINT		MVED_DMA_REG(0x561000f0)
#define MVED_DMA_DRCMR0		MVED_DMA_REG(0x56100100)
#define MVED_DMA_DRCMR1		MVED_DMA_REG(0x56100104)
#define MVED_DMA_DRCMR2		MVED_DMA_REG(0x56100108)
#define MVED_DMA_DRCMR3		MVED_DMA_REG(0x5610010c)
#define MVED_DMA_DDADR0		MVED_DMA_REG(0x56100200)
#define MVED_DMA_DSADR0		MVED_DMA_REG(0x56100204)
#define MVED_DMA_DTADR0		MVED_DMA_REG(0x56100208)
#define MVED_DMA_DCMD0		MVED_DMA_REG(0x5610020c)
#define MVED_DMA_DDADR1		MVED_DMA_REG(0x56100210)
#define MVED_DMA_DSADR1		MVED_DMA_REG(0x56100214)
#define MVED_DMA_DTADR1		MVED_DMA_REG(0x56100218)
#define MVED_DMA_DCMD1		MVED_DMA_REG(0x5610021c)
#define MVED_DMA_DDADR2		MVED_DMA_REG(0x56100220)
#define MVED_DMA_DSADR2		MVED_DMA_REG(0x56100224)
#define MVED_DMA_DTADR2		MVED_DMA_REG(0x56100228)
#define MVED_DMA_DCMD2		MVED_DMA_REG(0x5610022c)

/**
 * MVED_DMA_DCSR Register 
 */
#define MVED_DMA_DCSR_BUS_ERR_INTR       (1U<<0)
#define MVED_DMA_DCSR_START_INTR         (1U<<1)
#define MVED_DMA_DCSR_END_INTR           (1U<<2)
#define MVED_DMA_DCSR_STOP_INTR          (1U<<3)
#define MVED_DMA_DCSR_RAS_INTR           (1U<<4)
#define MVED_DMA_DCSR_REQ_PEND           (1U<<8)
#define MVED_DMA_DCSR_EOR_INTR           (1U<<9)
#define MVED_DMA_DCSR_MASK_RUN           (1U<<22)
#define MVED_DMA_DCSR_RAS_EN             (1U<<23)
#define MVED_DMA_DCSR_EOR_STOP_EN        (1U<<26)
#define MVED_DMA_DCSR_EOR_JMP_EN         (1U<<27)
#define MVED_DMA_DCSR_EOR_IRQ_EN         (1U<<28)
#define MVED_DMA_DCSR_STOP_IRQ_EN        (1U<<29)
#define MVED_DMA_DCSR_NO_DESC_FETCH      (1U<<30)
#define MVED_DMA_DCSR_RUN                (1U<<31)

/**
 * Mask of all writable bits in DCSR; others must be written as 0
 */
#define MVED_DMA_DCSR_WRITABLES_MSK    (MVED_DMA_DCSR_BUS_ERR_INTR |\
                                    MVED_DMA_DCSR_START_INTR   |\
                                    MVED_DMA_DCSR_END_INTR     |\
                                    MVED_DMA_DCSR_RAS_INTR     |\
                                    MVED_DMA_DCSR_EOR_INTR     |\
                                    MVED_DMA_DCSR_RAS_EN       |\
                                    MVED_DMA_DCSR_EOR_STOP_EN  |\
                                    MVED_DMA_DCSR_EOR_JMP_EN   |\
                                    MVED_DMA_DCSR_EOR_IRQ_EN   |\
                                    MVED_DMA_DCSR_STOP_IRQ_EN  |\
                                    MVED_DMA_DCSR_NO_DESC_FETCH|\
                                    MVED_DMA_DCSR_RUN         )
                                    
#define MVED_DMA_DCSR_WRITE_ONE_TO_CLEAR_BITS    (MVED_DMA_DCSR_BUS_ERR_INTR|\
                                                   MVED_DMA_DCSR_START_INTR  |\
                                                   MVED_DMA_DCSR_END_INTR    |\
                                                   MVED_DMA_DCSR_RAS_INTR    |\
                                                   MVED_DMA_DCSR_EOR_INTR)

#define IRQ_MVED_BPB2IMG	PXA_IRQ(53)
#define BIMFSR			MVED_BPB2IMG_REG(0x56200000) 
#define BIMESR			MVED_BPB2IMG_REG(0x56200008) 
#define BIMDRMR			MVED_BPB2IMG_REG(0x56200010) 
#define BIMDBSR			MVED_BPB2IMG_REG(0x56200018) 
#define BIMISR			MVED_BPB2IMG_REG(0x56200020) 
#define BIMCRR			MVED_BPB2IMG_REG(0x56200028) 
#define BIMIER			MVED_BPB2IMG_REG(0x56200030) 
#define BIMSQE 			MVED_BPB2IMG_REG(0x56200038) 
#define BIMPMR1			MVED_BPB2IMG_REG(0x56200040) 
#define BIMPMR2			MVED_BPB2IMG_REG(0x56200048) 
#define BIMRES			MVED_BPB2IMG_REG(0x56200050) 

#define MVED_POWER_OFF		0
#define MVED_POWER_ON		1

static unsigned int		mved_base;
static unsigned int		mved_dma_base;
static unsigned int		mved_bpb2img_base;

static unsigned long		sys_res_p_start[2];
static unsigned long		sys_res_k_start[2];
static unsigned long		sys_res_len[2];
static unsigned long		sys_res_mmap[2];
static int 			major=245, minor;
static int 			int_en_last;
/**
 * who control the mved. in deepidle mode, the driver lost the control
 * and the notify call back get the control, vice versa.
 */
static struct semaphore 	mved_ctl_sem;	

static DECLARE_WAIT_QUEUE_HEAD(mved_dma_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_mtx_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_mvea_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_displaydma_wait);

static unsigned long 		mved_work_freq = 156;
static unsigned long 		mved_real_work_freq = 156;

static struct platform_driver	pxa_mved_driver;
static spinlock_t 		mved_dvfm_lock;
static int 			mved_open =0;
static volatile int 		mved_dvfm_enable = 1;
static volatile int 		mved_cur_power=MVED_POWER_OFF;
static unsigned int 		mved_suspend_count;
static unsigned int		mved_resume_count;
static unsigned int		mved_reject_d0cs;
static unsigned int		mved_accept_d0cs;

#ifdef CONFIG_IMM
static u32 			mved_immid = 0;
#endif

static int 			display_dma_ch = -1;
static volatile int 		display_dma_end = 0;
static pxa_dma_desc 		*display_dma_desc;
static unsigned long 		display_dma_desc_p;
static int 			is_display_dma_pending;


static irqreturn_t mved_dma_irq(int irq, void *ptr);
static irqreturn_t mved_irq(int irq, void *ptr);
static irqreturn_t mved_bpb2img_irq(int irq, void *ptr);
static void display_dma_irq(int channel, void *data);

static unsigned long get_cur_freq(void)
{
#ifdef CONFIG_DVFM
	int 	op;
	struct 	pxa3xx_fv_info fv_info;

	op = pxa3xx_fv_get_op();
	if (pxa3xx_fv_get_op_info(op, &fv_info))
		panic("invalid op!!\n");
	if (fv_info.d0cs)
		return 0;
	else
		return (fv_info.xl * fv_info.xn * 13);
#else
	return 624;
#endif
}

static int update_mved_freq(void)
{
	unsigned long 		cur_core_freq;

	cur_core_freq = get_cur_freq();
	if (cur_core_freq == 104 || cur_core_freq == 208) {
		mved_real_work_freq = 78;
		ACCR = (ACCR&(~(0x3<<28)))|(0x3 <<28); /*mved clock = 78MHz*/
	}else if (cur_core_freq == 416) {
		mved_real_work_freq = 104;
		ACCR = (ACCR&(~(0x3<<28)))|(0x0 <<28); /*mved clock = 104MHz*/
	}else if (cur_core_freq == 624) {
		mved_real_work_freq = 156;
		ACCR = (ACCR&(~(0x3<<28)))|(0x1 <<28); /*mved clock = 156MHz*/
	}else {
		return -1;
	}
	return 0;
}

static int query_core_freq(void)
{
#ifdef CONFIG_DVFM
	wait_queue_head_t	delay_wait;
	int 			err = 0;
	unsigned long 		cur_core_freq;

	init_waitqueue_head(&delay_wait);
	cur_core_freq = get_cur_freq();
	if (mved_work_freq == 78) {
		if (cur_core_freq < 104) {
			ipm_event_notify(IPM_EVENT_DEVICE, 
				IPM_EVENT_DEVICE_OUTD0CS, NULL, 0);
			err = wait_event_interruptible_timeout(delay_wait,
				(get_cur_freq()>=104), HZ/100);
			if (err<0)
				return err;
		}
	} else if (mved_work_freq == 104) {
		if (cur_core_freq < 416) {
			ipm_event_notify(IPM_EVENT_DEVICE, 
					IPM_EVENT_DEVICE_OUT208, NULL, 0);
			err = wait_event_interruptible_timeout(delay_wait,
						(get_cur_freq()>=416), HZ/100);
		if (err<0)
			return err;
		}
	} else if (mved_work_freq == 156) {
		if (cur_core_freq < 624) {
			ipm_event_notify(IPM_EVENT_DEVICE, 
				IPM_EVENT_DEVICE_OUT416, NULL, 0);
			err = wait_event_interruptible_timeout(delay_wait,
				(get_cur_freq()>=624), HZ/100);
		if (err<0)
			return err;
		}
	} else {
		panic("un-supported mved clock!!\n");
	}
#endif
	return 0;
}

static int display_do_dma(unsigned long srcphyaddr, 
			unsigned long desphyaddr, 
			unsigned long len)
{
#define MAX_DESC_NUM		0x1000
#define SINGLE_DESC_TRANS_MAX  	8000

	pxa_dma_desc 	*display_dma_desc_tmp;
	unsigned long 	display_dma_desc_p_tmp;
	unsigned long 	len_tmp;

	if (len > (MAX_DESC_NUM-2)*SINGLE_DESC_TRANS_MAX) {
		printk(KERN_ERR "display size is too large\n");
		return -1;
	}
	if (len & 0x1f) {
		printk(KERN_ERR "display size is not 32 bytes aligned\n");
		return -1;
	}

	if (display_dma_ch == -1) {
		display_dma_ch = pxa_request_dma("mved_display",
					DMA_PRIO_HIGH, 
					display_dma_irq, 
					NULL);
		if (display_dma_ch < 0) {
			printk(KERN_ERR 
				"MVED: Cann't request DMA for display\n");
			return -1;
		} 		
	}
	
	if (display_dma_desc == NULL) {
		display_dma_desc = dma_alloc_writecombine( NULL, 
					MAX_DESC_NUM * sizeof(pxa_dma_desc),
					(void *)&display_dma_desc_p, 
				GFP_KERNEL);
		if (display_dma_desc == NULL) {
			printk(KERN_ERR "display dma desc allocate error!!\n");
			return -1;
		}
	}

	display_dma_desc_tmp = display_dma_desc;
	display_dma_desc_p_tmp = display_dma_desc_p;
	while (len) {
		len_tmp = len > SINGLE_DESC_TRANS_MAX ? 
				SINGLE_DESC_TRANS_MAX : len;
	        display_dma_desc_tmp->ddadr = display_dma_desc_p_tmp
		       			+ sizeof(pxa_dma_desc);
		display_dma_desc_tmp->dsadr = srcphyaddr;
		display_dma_desc_tmp->dtadr = desphyaddr;
		display_dma_desc_tmp->dcmd = len_tmp | DCMD_INCSRCADDR 
					| DCMD_INCTRGADDR | DCMD_BURST32;
		len -= len_tmp;
		display_dma_desc_tmp ++;
		display_dma_desc_p_tmp += sizeof(pxa_dma_desc);
		srcphyaddr += len_tmp;
		desphyaddr += len_tmp;
	}
	
        display_dma_desc_tmp->ddadr = display_dma_desc_p_tmp 
					+ sizeof(pxa_dma_desc);
	display_dma_desc_tmp->dsadr = srcphyaddr;
	display_dma_desc_tmp->dtadr = desphyaddr;
	display_dma_desc_tmp->dcmd = 0 | DCMD_INCSRCADDR | DCMD_INCTRGADDR
	       				| DCMD_BURST32 | DCMD_ENDIRQEN;

	display_dma_end = 0;
        DDADR(display_dma_ch) = (int) display_dma_desc_p;
	is_display_dma_pending = 1;
        DCSR(display_dma_ch) |= DCSR_RUN;
	
	return 0;
}

static void display_dma_irq(int channel, void *data)
{
	DCSR(channel) = DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR;
	display_dma_end	= 1;
	wake_up_interruptible(&mved_displaydma_wait);

	return;
}

static int pxa_mved_open(struct inode *inode, struct file *file)
{
	int err;

	if (mved_open >= 2){
		printk(KERN_ERR "MVED driver is busy!!!!\n");
		return -EPERM;
	}

	if (mved_open == 0) {
		mved_open++;
		while(1) {
			mved_dvfm_enable=1;
			err = query_core_freq();
			if (err) {
				printk(KERN_ERR 
					"Can't goto normal power status !!!\n");
				mved_open--;
				return err;
			}
			if (down_interruptible(&mved_ctl_sem)) {
				printk(KERN_ERR "Wait mved free failed!!!\n");
				mved_open--;
				return err;
			}

			mved_dvfm_enable=0;
			err = update_mved_freq();
			if (!err) {
				break;
			}else{
				up(&mved_ctl_sem);
			}
		}
		
		pxa_set_cken(CKEN_MVED, 1);
		mved_resume_count++;
		mved_cur_power=MVED_POWER_ON;
		
		BIMRES = 0;
		schedule_timeout(10);
		BIMRES |= (1<<8);
		
		BIMFSR = 0x444;
		BIMESR = 0x3;
		BIMDRMR = 0x1;
		BIMDBSR = 0x2;
		BIMISR = 0x2;
		BIMCRR = 0x0;
		BIMIER = 0x02;
		BIMSQE = 1;
		BIMPMR1 = 0;
		BIMPMR2 = 0;

		MVED_DMA_DRCMR2=0x80;
		MVED_DMA_DRCMR3=0x81;

		err = request_irq(IRQ_MVED_DMA, mved_dma_irq, 0, 
				"mved_dma", NULL);
		disable_irq(IRQ_MVED_DMA);				
		int_en_last = 0;

		err = request_irq(IRQ_MVED, mved_irq, 0, "mved", NULL);
		disable_irq(IRQ_MVED);

		err = request_irq(IRQ_MVED_BPB2IMG, mved_bpb2img_irq, 0, 
				"mved_bpb2img", NULL);
		disable_irq(IRQ_MVED_BPB2IMG);

		pxa_set_cken(CKEN_MVED, 0);
		up(&mved_ctl_sem);
		mved_dvfm_enable=1;
		
		mved_cur_power=MVED_POWER_OFF;			
		mved_suspend_count=0;
		mved_resume_count=0;
		mved_reject_d0cs = 0;
		mved_accept_d0cs = 0;		
	} else {
		mved_open++;
	}
	return 0;
}

static int pxa_mved_close(struct inode *inode, struct file *file)
{
	mved_open --;
	if (mved_open < 0) {
		panic("Error Close Operation!!\n");
		return -EPERM;
	}
	
	if (mved_open == 0) {
		down_trylock(&mved_ctl_sem);
		up(&mved_ctl_sem);
		
		free_irq(IRQ_MVED_DMA, NULL);
		free_irq(IRQ_MVED, NULL);
		free_irq(IRQ_MVED_BPB2IMG, NULL);

		BIMRES = 0;
		pxa_set_cken(CKEN_MVED, 0);
		mved_dvfm_enable = 1;
		mved_cur_power=MVED_POWER_OFF;

		if (display_dma_desc != NULL) {
			dma_free_writecombine (NULL, 
		               	 MAX_DESC_NUM * sizeof (pxa_dma_desc),
	        		 display_dma_desc,
		                 (int)display_dma_desc_p);
			display_dma_desc = NULL;

		}
		if (display_dma_ch != -1) {
			pxa_free_dma(display_dma_ch);
			display_dma_ch = -1;

		}
		is_display_dma_pending = 0;		
	}

	return 0;
}

static ssize_t pxa_mved_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			mved_clock;

	t = scnprintf(next, size, "MVED Info: \n");
	size -= t;
	next += t;

	if (((ACCR & (0x3<<28)) >> 28) == 0)
		mved_clock = 104;
	else if (((ACCR & (0x3<<28)) >> 28) == 1)
		mved_clock = 156;
	else if (((ACCR & (0x3<<28)) >> 28) == 2)
		mved_clock = 208;
	else
		mved_clock = 78;

	t = scnprintf(next, size, "Clock:\t\t\t\t%dMHz\n", mved_clock);
	size -= t;
	next += t;

	if (mved_cur_power == MVED_POWER_ON)
		t = scnprintf(next, size, "Clock Gate:\t\t\t%dMHz\n", 
				MVED_CLKGATE);
	else
		t = scnprintf(next, size, "Clock Gate:\t\t\t???\n");
	size -= t;
	next += t;

	t = scnprintf(next, size, "mved_reject_d0cs:\t\t%d\n", 
			mved_reject_d0cs);
	size -= t;
	next += t;
	t = scnprintf(next, size, "mved_accept_d0cs:\t\t%d\n", 
			mved_accept_d0cs);
	size -= t;
	next += t;

	t = scnprintf(next, size, "Suspend Count:\t\t\t%d\n", 
			mved_suspend_count);
	size -= t;
	next += t;
	t = scnprintf(next, size, "Resume Count:\t\t\t%d\n", 
			mved_resume_count);
	size -= t;
	next += t;
	t = scnprintf(next, size, "Current Power Status:\t\t%s\n", 
			mved_cur_power==MVED_POWER_OFF?"off":"on");
	size -= t;
	next += t;
	t = scnprintf(next, size, "mved_dvfm_enable:\t\t%d\n", 
			mved_dvfm_enable);
	size -= t;
	next += t;
	
	t = scnprintf(next, size, "Help:\n");
	size -= t;
	next += t;

	t = scnprintf(next, size, 
		"echo 78 > /proc/driver/mved\t/*change clock to 78*/\n");
	size -= t;
	next += t;
	t = scnprintf(next, size, 
		"echo 104 > /proc/driver/mved\t/*change clock to 104MHz*/\n");
	size -= t;
	next += t;
	t = scnprintf(next, size, 
		"echo 156 > /proc/driver/mved\t/*change clock to 156*/\n");
	size -= t;
	next += t;

	*eof = 1;
	return count - size;
}

static int pxa_mved_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[1024];
	int mved_clock = 0;

	if (count >= 1024)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	sscanf(kbuf, "%d", &mved_clock);
	if (104 == mved_clock) {
		ACCR = (ACCR & (~(0x3<<28))) | (0x0<<28);
		mved_work_freq = 104;
	} else if (156 == mved_clock) {
		ACCR = (ACCR & (~(0x3<<28))) | (0x1<<28);
		mved_work_freq = 156;
	} else if (78 == mved_clock) {
		ACCR = (ACCR & (~(0x3<<28))) | (0x3<<28);
		mved_work_freq = 78;
	} else {
		printk(KERN_ERR "error!!\n");
		printk(KERN_ERR "\techo 78 > /proc/driver/mved\t\t/*Set 78MHz*/\n");
		printk(KERN_ERR "\techo 104 > /proc/driver/mved\t\t/*Set 104MHz*/\n");
		printk(KERN_ERR "\techo 156 > /proc/driver/mved\t\t/*Set 156MHz*/\n");
	}
	return count;
}

static int mved_usercopy(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg,
		int (*func)(struct inode *inode, struct file *file,
		unsigned int cmd, void *arg))
{
	char	sbuf[128];
	void    *mbuf = NULL;
	void	*parg = NULL;
	int	err  = -EINVAL;

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_NONE:
		parg = NULL;
		break;
	case _IOC_READ:
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
		if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
			parg = sbuf;
		} else {
			/* too big to allocate from stack */
			mbuf = kmalloc(_IOC_SIZE(cmd),GFP_KERNEL);
			if (NULL == mbuf)
				return -ENOMEM;
			parg = mbuf;
		}
		
		err = -EFAULT;
		if ((_IOC_DIR(cmd) & _IOC_WRITE) &&
		    (copy_from_user(parg, (void __user *)arg, _IOC_SIZE(cmd))))
				goto out;
		break;
	}

	/* call driver */
	err = func(inode, file, cmd, parg);
	if (err == -ENOIOCTLCMD)
		err = -EINVAL;
	if (err < 0)
		goto out;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd)) {
		case _IOC_READ:
		case (_IOC_WRITE | _IOC_READ):
			if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd)))
				err = -EFAULT;
			break;
	}

out:
	if (mbuf)
		kfree(mbuf);
	return err;
}

static unsigned long uva_to_pa(unsigned long addr)
{
        unsigned long ret = 0UL;
        pgd_t *pgd;
        pud_t *pud;
        pmd_t *pmd;
        pte_t *pte;

        pgd = pgd_offset(current->mm, addr);
        if (!pgd_none(*pgd)) {
                pud = pud_offset(pgd, addr);
                if (!pud_none(*pud)) {
                        pmd = pmd_offset(pud, addr);
                        if (!pmd_none(*pmd)) {
                                pte = pte_offset_map(pmd, addr);
                                if (!pte_none(*pte) && pte_present(*pte)) {
                                        ret = page_to_phys(pte_page(*pte));
                                        ret |= (addr & (PAGE_SIZE-1));
                                }
                        }
                }
        }
        return ret;
}

static int check_phy_contiguous(unsigned long addr, unsigned long len)
{
	unsigned long phyaddr;
	unsigned long lastphyaddr;
	
	if ((phyaddr = uva_to_pa(addr)) == 0UL)
		return 0;
	lastphyaddr = phyaddr;
	len = (len>PAGE_SIZE) ? len-PAGE_SIZE : 0;
	addr += PAGE_SIZE;
	while (len) {
		if ((phyaddr = uva_to_pa(addr)) == 0UL)
			return 0;
		if (lastphyaddr+PAGE_SIZE != phyaddr)
			return 0;
		lastphyaddr = phyaddr;
		addr += PAGE_SIZE;
		len = (len>PAGE_SIZE) ? len-PAGE_SIZE : 0;
	}
	return 1;
}

static int mved_do_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, void *arg)
{
	int retval = 0;

	switch (cmd) {
		case MVED_S_UPDATEMEMINFO:
		{
			struct MEMORY_INFO_TAG *meminfo = 
						(struct MEMORY_INFO_TAG *)arg;
			if (!(meminfo->flag & MM_FLAG_UVALID) 
				|| !(meminfo->flag & MM_FLAG_LVALID)) {
				printk(KERN_ERR 
					"User addr or len is not valid!!\n");
				break;
			}
			meminfo->p_addr = 
			    (void *)uva_to_pa((unsigned long)meminfo->u_addr);
			if (meminfo->p_addr != NULL);
				meminfo->flag |= MM_FLAG_PVALID;
			if (check_phy_contiguous((unsigned long)meminfo->u_addr,
						meminfo->len)) {
				meminfo->flag |= MM_FLAG_PHYCONT;
			} else {
				printk(KERN_ERR 
					"Memory is not phy contiguous!!\n");
				break;
			}
			break;
		}		
		case MVED_S_DISPLAY_DMA_ISSUE:
		{
			struct mem_dma_t *mem_dma = (struct mem_dma_t *)arg;
			unsigned long srcphyaddr, destphyaddr;

			srcphyaddr = uva_to_pa(mem_dma->srcuseraddr);
			destphyaddr = uva_to_pa(mem_dma->taruseraddr);

			if (srcphyaddr == 0 || destphyaddr == 0)
				return -1;
			retval = display_do_dma(srcphyaddr, destphyaddr, 
						mem_dma->len);
			break;
		}		
		case MVED_S_DISPLAY_DMA_SYNC:
		{
			int *poll = (int *)arg;
			
			DECLARE_WAITQUEUE(wait, current);

			if (!is_display_dma_pending)
				break;

			if (*poll) {
				while(!display_dma_end) {
					;
				}		
				is_display_dma_pending = 0;
				break;
			}
			
			add_wait_queue(&mved_displaydma_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			disable_irq(IRQ_DMA);
			if (display_dma_end) {
				enable_irq(IRQ_DMA);
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&mved_displaydma_wait, &wait);
				is_display_dma_pending = 0;
				break;
			}		
			enable_irq(IRQ_DMA);
			schedule();
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&mved_displaydma_wait, &wait);
			is_display_dma_pending = 0;
			break;
		}		
		case MVED_S_POWER_ON:
		{
			while(1) {
				mved_dvfm_enable=1;
				retval = query_core_freq();
				if (retval) {
					printk(KERN_ERR 
						"Can't goto normal status\n");
					return retval;
				}
				if (down_interruptible(&mved_ctl_sem)) {
					printk(KERN_ERR 
						"Wait mved free failed!!!\n");
					return -1;
				}
				mved_dvfm_enable=0;
				retval = update_mved_freq();
				if (!retval) {
					break;
				}else{
					up(&mved_ctl_sem);
				}
			}
			pxa_set_cken(CKEN_MVED, 1);
			mved_resume_count++;
			mved_cur_power=MVED_POWER_ON;
			break;
		}
		case MVED_S_POWER_OFF:
		{
			pxa_set_cken(CKEN_MVED, 0);
			mved_suspend_count++;
			mved_cur_power=MVED_POWER_OFF;			
			mved_dvfm_enable = 1;
			up(&mved_ctl_sem);
			break;
		}		
		
		case MVED_S_FLUSHCACHE:
		{
			struct MEMORY_INFO_TAG *meminfo = 
				(struct MEMORY_INFO_TAG *)arg;
			xsc3_flush_user_cache_range(
				(unsigned int)meminfo->u_addr, 
				(unsigned int)meminfo->u_addr+meminfo->len, 0);
			break;
		}		
		case MVED_S_MTX_WAIT:
		{
			unsigned int timeout = *((unsigned int *)arg);
			DECLARE_WAITQUEUE(wait, current);
			
			add_wait_queue(&mved_mtx_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);

			if (MVED_INTSTAT & (1<<2)) {
				MVED_INTCLEAR = 0xf;
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&mved_mtx_wait, &wait);
				break;
			}		
			enable_irq(IRQ_MVED);	
			*((unsigned int *)arg) = schedule_timeout(
						msecs_to_jiffies(timeout));
			disable_irq(IRQ_MVED);				
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&mved_mtx_wait, &wait);
			break;
		}
		case MVED_S_DMA_WAITLAST:
		{
			unsigned int timeout = *((unsigned int *)arg);
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&mved_dma_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);

			if (!((MVED_DMA_DCSR0&MVED_DMA_DCSR_END_INTR)==0 
				|| (MVED_DMA_DCSR1&MVED_DMA_DCSR_END_INTR)==0)){
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&mved_dma_wait, &wait);
				break;
			}		
			enable_irq(IRQ_MVED_DMA);	
			*((unsigned int *)arg) = schedule_timeout(
						msecs_to_jiffies(timeout));
			disable_irq(IRQ_MVED_DMA);				
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&mved_dma_wait, &wait);
			break;
		}
		default:
		{
			retval = -ENOIOCTLCMD;
			printk(KERN_ERR "IO Command 0x%x is not defined\n", cmd);
			break;
		}
	}
    
    return retval;
}

static int pxa_mved_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, unsigned long param)
{
	return mved_usercopy(inode, file, cmd, param, mved_do_ioctl);
}


static void pxa_mved_vma_open(struct vm_area_struct *vma)
{
}

static void pxa_mved_vma_close(struct vm_area_struct *vma)
{
	struct memblk_info *memblk = vma->vm_private_data;
	unsigned long addr, size;

	if (NULL == memblk) {
		return;
	}	

	if (MVED_MMAP_MALLOC == (memblk->type&MVED_MMAP_CMD_MASK)) {
		if (!((MVED_MMAP_RES0_MASK & memblk->type)
			|| (MVED_MMAP_RES1_MASK & memblk->type)
			|| (MVED_MMAP_SRAM_MASK & memblk->type))) {
			addr = (unsigned long)memblk->k_addr;
			size = (unsigned long)memblk->len;
			while(size > 0) {
				ClearPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
		}

		if (MVED_MMAP_SRAM_MASK&memblk->type) {
#ifdef CONFIG_IMM
			imm_free(memblk->k_addr, mved_immid);
#else
			addr = (unsigned long)memblk->k_addr;
			size = (unsigned long)memblk->len;
			while(size > 0) {
				ClearPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			free_pages((unsigned long)memblk->k_addr, 
					get_order(memblk->len));
#endif
		}else if (MVED_MMAP_RES0_MASK&memblk->type){
			sys_res_mmap[0] = 0;
		}else if (MVED_MMAP_RES1_MASK&memblk->type){
			sys_res_mmap[1] = 0;
		}else {
			free_pages((unsigned long)memblk->k_addr, 
					get_order(memblk->len));
		}
	}	
	kfree(memblk);
	memblk = NULL;
}

static struct vm_operations_struct pxa_mved_remap_vm_ops = {
	.open =  pxa_mved_vma_open,
	.close = pxa_mved_vma_close,
};


static int pxa_mved_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long map_size = vma->vm_end - vma->vm_start;
 	int type = (int)vma->vm_pgoff<<PAGE_SHIFT;
	struct memblk_info *memblk_new = NULL;
	struct page *page;
	unsigned long addr, size;
	unsigned int map_flag = 0;

	memblk_new = (struct memblk_info *)kmalloc(sizeof(struct memblk_info), 
							GFP_KERNEL);
	if (NULL == memblk_new)
		panic("No memory!!!!\n");
	memblk_new->len = map_size;
	memblk_new->type = type;
	
	if (MVED_MMAP_MVED_REGS == (type & MVED_MMAP_CMD_MASK)){
		if (remap_pfn_range(vma, (unsigned long)vma->vm_start, 
				(unsigned long)(MVED_REG_PHYBASE >> PAGE_SHIFT),
			       	map_size, pgprot_noncached(PAGE_SHARED))) 
			return -EAGAIN;
	} else if (MVED_MMAP_MVED_DMA_REGS == (type & MVED_MMAP_CMD_MASK)){
		if (remap_pfn_range(vma, (unsigned long)vma->vm_start, 
			(unsigned long)(MVED_DMA_REG_PHYBASE >> PAGE_SHIFT), 
			map_size, pgprot_noncached(PAGE_SHARED))) 
			return -EAGAIN;
	} else if (MVED_MMAP_BPB2IMG_REGS == (type & MVED_MMAP_CMD_MASK)){
		if (remap_pfn_range(vma, (unsigned long)vma->vm_start, 
			(unsigned long)(MVED_BPB2IMG_REG_PHYBASE >> PAGE_SHIFT),
		       	map_size, pgprot_noncached(PAGE_SHARED))) 
			return -EAGAIN;
	} else if (MVED_MMAP_MALLOC == (type & MVED_MMAP_CMD_MASK)) {
		if (MVED_MMAP_CACHE_MASK&type)
			map_flag = __pgprot(
				pgprot_val(PAGE_SHARED) & ~(L_PTE_BUFFERABLE));
		else
			map_flag = pgprot_noncached(PAGE_SHARED);
		
		if (MVED_MMAP_RES0_MASK & type) {
			if (!sys_res_k_start[0]) {
				printk(KERN_ERR 
					"Reserve memory before boot firstly\n");
				return -1;
			}
			if (sys_res_mmap[0]){
				printk(KERN_ERR "Reserve memory not avalid!\n");
				return -1;
			}
			sys_res_mmap[0] = 1;
				
			size = (unsigned long)memblk_new->len;
			if (size > sys_res_len[0]) {
				printk(KERN_ERR 
					"memory is lager than reserve size\n");
				return -1;
			}
			memblk_new->u_addr = (void *)vma->vm_start;
			memblk_new->k_addr = (void *)sys_res_k_start[0];
			memblk_new->p_addr = (void *)sys_res_p_start[0];

		} else if (MVED_MMAP_RES1_MASK & type) {
			if (!sys_res_k_start[1]) {
				printk(KERN_ERR 
					"Reserve memory before boot firstly\n");
				return -1;
			}
			if (sys_res_mmap[1]){
				printk(KERN_ERR "Reserve memory not avalid!!\n");
				return -1;
			}
			sys_res_mmap[1] = 1;
			size = (unsigned long)memblk_new->len;
			if (size > sys_res_len[1]) {
				printk(KERN_ERR 
					"memory size is too lage\n");
				return -1;
			}
			memblk_new->u_addr = (void *)vma->vm_start;
			memblk_new->k_addr = (void *)sys_res_k_start[1];
			memblk_new->p_addr = (void *)sys_res_p_start[1];

		} else if (MVED_MMAP_SRAM_MASK & type) {
#ifdef CONFIG_IMM
			memblk_new->u_addr = (void *)vma->vm_start;
			memblk_new->k_addr = imm_malloc(map_size, 
					IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, 
					mved_immid);
			if (NULL == memblk_new->k_addr) {
				printk(KERN_ERR "imm_malloc error\n");
				return -1;
			}
			memblk_new->p_addr = (void *)imm_get_physical(
							memblk_new->k_addr, 
							mved_immid);
#else
			page = alloc_pages(GFP_KERNEL, 
						get_order(memblk_new->len));
			if (page == NULL) {
				printk(KERN_ERR "No memory!!!!\n");
				return -1;
			}
			memblk_new->u_addr = (void *)vma->vm_start;
			memblk_new->k_addr = (void *)page_address(page);
			memblk_new->p_addr = (void *)virt_to_phys(
							memblk_new->k_addr);
			addr = (unsigned long)memblk_new->k_addr;
			size = (unsigned long)memblk_new->len;
			while(size > 0) {
				SetPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
#endif
		} else {
			page = alloc_pages(GFP_KERNEL, 
						get_order(memblk_new->len));
			if (page == NULL) {
				printk(KERN_ERR "No memory!!!!\n");
				return -1;
			}
			memblk_new->u_addr = (void *)vma->vm_start;
			memblk_new->k_addr = (void *)page_address(page);
			memblk_new->p_addr = (void *)virt_to_phys(
							memblk_new->k_addr);
			addr = (unsigned long)memblk_new->k_addr;
			size = (unsigned long)memblk_new->len;
			while(size > 0) {
				SetPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
		}

		if (remap_pfn_range(vma, (unsigned long)memblk_new->u_addr, 
				((unsigned long)memblk_new->p_addr>>PAGE_SHIFT),
			       	map_size, map_flag))
		{
			printk(KERN_ERR "remap error!\n");
			return -EAGAIN;
		}
		
		((unsigned long *)memblk_new->k_addr)[0] = 
					(unsigned long)memblk_new->k_addr;
		((unsigned long *)memblk_new->k_addr)[1] = 
					(unsigned long)memblk_new->u_addr;
		((unsigned long *)memblk_new->k_addr)[2] = 
					(unsigned long)memblk_new->p_addr;

		consistent_sync(memblk_new->k_addr, 
				(1<<get_order(memblk_new->len))<<PAGE_SHIFT, 
				DMA_BIDIRECTIONAL);
	} else {
		printk(KERN_ERR "command not support\n");
		return -EAGAIN;
	}
	vma->vm_ops = &pxa_mved_remap_vm_ops;
	vma->vm_private_data = memblk_new;

    return 0;
}

static int pxa_mved_resume(struct platform_device *pdev)
{	
	if (!mved_dvfm_enable)
		return -1;

	pxa_set_cken(CKEN_MVED, 1);
	mved_resume_count++;
	mved_cur_power=MVED_POWER_ON;
	return 0;
}

static int pxa_mved_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (!mved_dvfm_enable)
		return -1;

	pxa_set_cken(CKEN_MVED, 0);
	mved_suspend_count++;
	mved_cur_power=MVED_POWER_OFF;
	return 0;
}

static struct file_operations pxa_mved_fops = {
	.owner		= THIS_MODULE,
	.open     	= pxa_mved_open,
	.release	= pxa_mved_close,
	.ioctl        	= pxa_mved_ioctl,
	.mmap	= pxa_mved_mmap,
};


static struct platform_driver pxa_mved_driver = {
	.driver  = {
		.name    = "pxa2xx-mved",
	},
	.suspend = pxa_mved_suspend,
	.resume  = pxa_mved_resume,
};



static irqreturn_t mved_dma_irq(int irq, void *ptr)
{
	int dint = MVED_DMA_DINT;
	
	if (dint & (1<<0)) {
		if (MVED_DMA_DCSR0&MVED_DMA_DCSR_END_INTR)
				wake_up_interruptible(&mved_dma_wait);

		MVED_DMA_DCSR0 = MVED_DMA_DCSR_BUS_ERR_INTR
				|MVED_DMA_DCSR_START_INTR
				|MVED_DMA_DCSR_END_INTR;

	} else if (dint & (1<<1)) {
		if (MVED_DMA_DCSR1&MVED_DMA_DCSR_END_INTR)
				wake_up_interruptible(&mved_dma_wait);
	
		MVED_DMA_DCSR1 = MVED_DMA_DCSR_BUS_ERR_INTR
				|MVED_DMA_DCSR_START_INTR
				|MVED_DMA_DCSR_END_INTR;

	} else {
		if(dint)
			printk(KERN_ERR "unknow mved dma interrupt, dint:0x%x!!\n", dint);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mved_irq(int irq, void *ptr)
{
	unsigned int intstat = MVED_INTSTAT;

	if (intstat & (1<<3)) {
		printk(KERN_ERR "mtx halt interrupt!!\n");
	}
	if (intstat & (1<<2)) {
		wake_up_interruptible(&mved_mtx_wait);

	}
	MVED_INTCLEAR = 0xf;
	return IRQ_HANDLED;
}

static irqreturn_t mved_bpb2img_irq(int irq, void *ptr)
{
	printk(KERN_ERR "enter bpb2img interrupt!!\n");
	return IRQ_HANDLED;
}

#ifdef CONFIG_DVFM
static int pxa3xx_mved_dvfm_notifier(unsigned cmd, void *client_data, void *info);
static struct pxa3xx_fv_notifier dvfm_notifier = {
	.name 		= "mved",
	.priority 	= 0,
	.notifier_call 	= pxa3xx_mved_dvfm_notifier,
};

static int pxa3xx_mved_dvfm_notifier(unsigned cmd, void *client_data, void *info)
{
	int res = -1;

	if (!mved_open)
		return 0;
	switch (cmd) {
		case FV_NOTIFIER_QUERY_SET :
			if (!mved_dvfm_enable) {
				mved_reject_d0cs++;
				res = -1;
			} else {
				res = 0;;
			}
			break;
		case FV_NOTIFIER_PRE_SET :
			if (!down_trylock(&mved_ctl_sem)) {
				res = 0;
				mved_accept_d0cs++;
			} else{
				res = -1;
				mved_reject_d0cs++;
			}
			break;
		case FV_NOTIFIER_POST_SET :
			up(&mved_ctl_sem);
			res = 0;
			break;
		default:
			printk(KERN_ERR "dvfm notifier Error!!!\n");
			res = -1;
	}
	return res;
}
#endif

static int __devinit pxa_mved_init(void)
{
	int 		result = 0;
	struct proc_dir_entry *pxa_mved_proc_entry;	


	init_MUTEX(&mved_ctl_sem);
	spin_lock_init(&mved_dvfm_lock);
	
	register_chrdev(major, "mved", &pxa_mved_fops);
	if (major < 0) {
		printk(KERN_ERR 
			"mved: unable to be registered as a char device!!\n");
		return -major;
	}
	minor = 0;
	
	pr_info("Register MVED driver: major=%d, minor=%d\n", major, minor);

	mved_base = (unsigned int)ioremap_nocache(MVED_REG_PHYBASE, 1024);
	mved_dma_base = (unsigned int)ioremap_nocache(MVED_DMA_REG_PHYBASE, 
							1024);
	mved_bpb2img_base = (unsigned int)ioremap_nocache(
						MVED_BPB2IMG_REG_PHYBASE, 1024);

#ifdef CONFIG_IMM
	mved_immid = imm_register_kernel("mved");
#endif
	ACCR = (ACCR&(~(0x3<<28)))|(0x1 <<28); 
 
#ifdef CONFIG_MVED_RES_MEM
	struct page *page;
	unsigned long addr, size;

	page = alloc_pages(GFP_KERNEL, get_order(MVED_SURFACE_RES_MEM_LEN));
	if (page == NULL){
		printk(KERN_ERR "No memory!!!!\n");
		return -1;
	}

	sys_res_k_start[0] = (unsigned long)page_address(page);
	sys_res_p_start[0] = (unsigned long)virt_to_phys(
						(void *)sys_res_k_start[0]);
	sys_res_len[0] = MVED_SURFACE_RES_MEM_LEN;
	addr = sys_res_k_start[0];
	size = sys_res_len[0];
	while(size > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	page = alloc_pages(GFP_KERNEL, get_order(MVED_DEVICE_RES_MEM_LEN));
	if (page == NULL){
		printk(KERN_ERR "No memory!!!!\n");
		return -1;
	}

	sys_res_k_start[1] = (unsigned long)page_address(page);
	sys_res_p_start[1] = (unsigned long)virt_to_phys(
						(void *)sys_res_k_start[1]);
	sys_res_len[1] = MVED_DEVICE_RES_MEM_LEN;
	addr = sys_res_k_start[1];
	size = sys_res_len[1];
	while(size > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
#endif

	platform_driver_register(&pxa_mved_driver);
	
#ifdef CONFIG_DVFM
        dvfm_notifier.client_data = NULL;
        pxa3xx_fv_register_notifier(&dvfm_notifier);
#endif
	
	pxa_mved_proc_entry = create_proc_entry("driver/mved", 0, NULL);
	if (pxa_mved_proc_entry) { 
		pxa_mved_proc_entry->data = NULL; 
		pxa_mved_proc_entry->read_proc = pxa_mved_proc_read; 
		pxa_mved_proc_entry->write_proc = pxa_mved_proc_write; 
	} 

	pxa_set_cken(CKEN_MVED, 1);
	
	BIMRES = 0;
	schedule_timeout(10);
	BIMRES |= (1<<8);

	BIMFSR = 0x444;
	BIMESR = 0x3;
	BIMDRMR = 0x1;
	BIMDBSR = 0x2;
	BIMISR = 0x2;
	BIMCRR = 0x0;
	BIMIER = 0x02;
	BIMSQE = 1;
	BIMPMR1 = 0;
	BIMPMR2 = 0;

	MVED_DMA_DRCMR2=0x80;
	MVED_DMA_DRCMR3=0x81;
	BIMRES = 0;
	pxa_set_cken(CKEN_MVED, 0);
	mved_dvfm_enable = 1;
	mved_cur_power=MVED_POWER_OFF;
	
	return result;	
} 

static void __exit pxa_mved_exit(void)
{
	if (sys_res_k_start[0])
		__iounmap((void *)sys_res_k_start[0]);
	if (sys_res_k_start[1])
		__iounmap((void *)sys_res_k_start[1]);

	__iounmap((void *)mved_base);
	__iounmap((void *)mved_dma_base);
	__iounmap((void *)mved_bpb2img_base);
	
	unregister_chrdev(major, "mved");
#ifdef CONFIG_DVFM
        pxa3xx_fv_unregister_notifier(&dvfm_notifier);
#endif
	platform_driver_unregister(&pxa_mved_driver);
}

MODULE_DESCRIPTION("MVED Driver");
MODULE_LICENSE("GPL");

module_init(pxa_mved_init);
module_exit(pxa_mved_exit);

