/*
 * Woodstock TPK/Cypress touchscreen support
 *
 *
 * Copyright (C) 2008 Sonos (B. Tober)
 * based on i2c/chips/meptouch.c (which was based on i2c/chips/micco.c)
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <linux/input.h>
#include <linux/freezer.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/irqs.h>
#include <asm/arch/woodstock_ioctl.h>

#include <linux/string.h>
#include "cytouch_fw.h"

#undef VERBOSE
#undef DEBUGGING

#undef FORCE_REFLASH

#ifdef FORCE_REFLASH
static int cytouch_force_reflash=1;
#else
static int cytouch_force_reflash=0;
#endif

extern void lt_charger_set_charge_led(int enable, int blanking, unsigned long bitmask);

module_param(cytouch_force_reflash,bool,0);

struct cystate {
    struct completion compl;
    struct input_dev *idev;
    struct miscdevice md;
    struct mutex mu;
    int expect_reset;
    int suppress_touch;
    int wantmutex;
    int rawopen;
} *csp;

int cytouch_raw_open(struct inode *i,struct file *f);
int cytouch_raw_close(struct inode *i,struct file *f);
ssize_t cytouch_raw_read(struct file *f,char __user *u,size_t sz,loff_t *ofs);
ssize_t cytouch_raw_write(struct file *f,const char __user *u,size_t sz,loff_t *ofs);

static const struct file_operations cytouch_raw_fops = {
    .open = &cytouch_raw_open,
    .release = &cytouch_raw_close,
    .read = &cytouch_raw_read,
    .write = &cytouch_raw_write
};

struct cy_i2c {
    u8 control; //0
    u8 touched;
    u8 px[2];
    u8 py[2];

    u8 cursor;
    u8 raw[2];
    u8 baseline[2];
    u8 sig;
    u8 idac;

    u8 speed;
    u8 res;
    u8 calval[2];
    u8 pwmper;
    u8 pwmcmp;

    u8 nthresh;
    u8 fthresh;
    u8 fwver[2];
    u8 fthreshl;

#ifdef DEBUGGING
    u8 d_sig[22];
    u8 d_raw[44];
    u8 d_baseline[44];
#endif

}; //length 23 bytes
#if 0
static int cytouch_probe(struct platform_device *pdev)
{
	// int ret;

	return 0;
}

static int cytouch_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver cytouch_driver = {
	.driver = {
		.name	= "cytouch",
	},
	.probe		= cytouch_probe,
	.remove		= cytouch_remove,
};
#endif

/******************************************************************************
 *									      *
 *			MICCO I2C Client Driver				      *
 *									      *
 ******************************************************************************/
#include <linux/i2c.h>
static int i2c_cytouch_attach_adapter(struct i2c_adapter *adapter);
static int i2c_cytouch_detect_client(struct i2c_adapter *, int, int);
static int i2c_cytouch_detach_client(struct i2c_client *client);
static int cytouch_suspend(struct i2c_client *client,pm_message_t m);
static int cytouch_resume(struct i2c_client *client);

struct i2c_driver i2c_cytouch_driver  = 
{
	.driver = {
		.name	= "cytouch",
	},
	.attach_adapter	= &i2c_cytouch_attach_adapter, 
	.detach_client	= &i2c_cytouch_detach_client,
	.suspend = cytouch_suspend,
	.resume = cytouch_resume,
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = {0x20, I2C_CLIENT_END };
I2C_CLIENT_INSMOD_1(cytouch);

int cy_write(const unsigned char *,int);
int cy_read(unsigned char *,int);

int cytouch_raw_open(struct inode *i,struct file *f)
{
    if (csp->rawopen) return -EIO;
    csp->rawopen=1;
    csp->wantmutex=1;
    mutex_lock(&csp->mu);
    csp->wantmutex=0;
    return 0;
}

int cytouch_raw_close(struct inode *i,struct file *f)
{
    if (!csp->rawopen) return -EIO;
    mutex_unlock(&csp->mu);
    csp->rawopen=0;
    return 0;
}

ssize_t cytouch_raw_read(struct file *f,char __user *u,size_t sz,loff_t *ofs)
{
    unsigned char *b;
    int rr;
    int err=0;
    b=kmalloc(sz,GFP_KERNEL);
    if (!b) return -ENOMEM;
    while (pxa3xx_gpio_get_level(23)) wait_for_completion_timeout(&(csp->compl),HZ/10);
    rr=cy_read(b,sz);
    if (rr<0) {err= -EIO; goto out_free; }
    if (copy_to_user(u,b,sz)) err= -EFAULT;
out_free:
    kfree(b);
    if (err) return err; else return sz;
}

ssize_t cytouch_raw_write(struct file *f,const char __user *u,size_t sz,loff_t *ofs)
{
    unsigned char *b;
    int rr;
    int err=0;
    b=kmalloc(sz,GFP_KERNEL);
    if (!b) return -ENOMEM;
    if (copy_from_user(b,u,sz)) {err= -EFAULT; goto out_free;}
    while (pxa3xx_gpio_get_level(23)) wait_for_completion_timeout(&(csp->compl),HZ/10);
    rr=cy_write(b,sz);
    if ((rr<0)&&(rr!= -ECONNREFUSED)) err= -EIO; // XXX hack!
out_free:
    kfree(b);
    if (err) return err; else return sz;
}

void cy_decode(const unsigned char *b,unsigned *x,unsigned *y,int *touch)
{
    *y=(b[1]<<8)|b[2];
    *x=(b[3]<<8)|b[4];
    *touch=(b[0]&1);
}

int cythread(void *arg)
{
#ifdef DEBUGGING
    struct cy_i2c *ci;
    int a;
    unsigned long j=0;
    unsigned char b[sizeof(struct cy_i2c)];
#else
    unsigned char b[6];
#endif
    int r,i;
    int touched;
    unsigned int x,y;

    unsigned int last_x = 1000000, last_y = 1000000;
    int last_touched=0;

    daemonize("cythread");
    wait_for_completion_interruptible(&(csp->compl));
#ifdef DEBUGGING
    j=jiffies;
#endif
    while (1) {
	mutex_lock(&csp->mu);
        while (pxa3xx_gpio_get_level(23)) wait_for_completion_timeout(&(csp->compl),HZ/10);
#ifdef DEBUGGING
        r=cy_read(b,sizeof(struct cy_i2c));
#else
        r=cy_read(b,6);
#endif
        if (r<0) {
            printk("cythread: cy_read failed\n");
        } else {
            if ((!csp->expect_reset)&&(b[0]&1)) printk("cythread: unexpected device reset\n");
            if (csp->expect_reset) {
                if (!(b[0]&1)) printk("cythread: lack of reset when expected\n");
                csp->expect_reset=0;
            }
            // if (b[0]&2) printk("cythread: baseline reset\n");
#ifdef DEBUGGING
            ci=(struct cy_i2c *)b;
            if (time_after(jiffies,j)) {
                printk("Sig ");
                for (a=0;a<22;a++) {
                    printk("%d:%d ",a,ci->d_sig[a]);
                }
                printk("\nRaw ");
                for (a=0;a<22;a++) {
                    printk("%d:%d ",a,(ci->d_raw[a<<1]<<8)|ci->d_raw[(a<<1)|1]);
                }
                printk("\n");
                j=jiffies+HZ;
            }
#endif
            if ((b[1])&&(!csp->suppress_touch)) {
                x=(b[2]<<8)|b[3];
                y=(b[4]<<8)|b[5];
                if ((!last_touched)||(x!=last_x)||(y!=last_y)) {
                    input_report_abs(csp->idev,ABS_X,(int)x);
                    input_report_abs(csp->idev,ABS_Y,(int)y);
                    input_report_abs(csp->idev,ABS_PRESSURE,100);
                    input_sync(csp->idev);
                    last_x=x;
                    last_y=y;
                    last_touched=1;
                }
            } else {
                if ((last_touched)||(csp->suppress_touch)) {
                    csp->suppress_touch=0;
                    input_report_abs(csp->idev,ABS_PRESSURE,0);
                    input_sync(csp->idev);
                    last_touched=0;
                }
            }
        }
	mutex_unlock(&csp->mu);
	if (csp->wantmutex) yield(); //XXX
        if (signal_pending(current)) try_to_freeze();
    }
    return 0;
}

irqreturn_t cytouch_attn_int(int irq,void *arg)
{
    complete(&(csp->compl));
    return IRQ_HANDLED;
}

#define MAX_XFER 72
int cy_write(const unsigned char *p,int len)
{
    struct i2c_msg m;
    m.addr=g_client->addr;
    m.flags=0;
    m.len=len;
    m.buf=p;
    return i2c_transfer(g_client->adapter,&m,1);
}


int cy_read(unsigned char *p,int len)
{
    struct i2c_msg m;
    m.addr=g_client->addr;
    m.flags=I2C_M_RD;
    m.len=len;
    m.buf=p;
    return i2c_transfer(g_client->adapter,&m,1);
}

static int i2c_cytouch_attach_adapter(struct i2c_adapter *adap)
{	
	return i2c_probe(adap,&addr_data, &i2c_cytouch_detect_client);
}

struct pxa3xx_pin_config tp_programming_config[]={
    PXA3xx_MFP_CFG("XRES",MFP_PIN_GPIO24,MFP_AF0,MFP_DS10X,1,MFP_LPM_DRIVE_HIGH,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("SCL",MFP_PIN_GPIO21,MFP_AF0,MFP_DS10X,1,MFP_LPM_FLOAT,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("SDA",MFP_PIN_GPIO22,MFP_AF0,MFP_DS10X,1,MFP_LPM_FLOAT,MFP_EDGE_NONE),
};

struct pxa3xx_pin_config tp_sleep_config[]={
    PXA3xx_MFP_CFG("XRES",MFP_PIN_GPIO24,MFP_AF0,MFP_DS10X,1,MFP_LPM_DRIVE_LOW,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("SCL",MFP_PIN_GPIO21,MFP_AF1,MFP_DS10X,1,MFP_LPM_FLOAT,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("SDA",MFP_PIN_GPIO22,MFP_AF1,MFP_DS10X,1,MFP_LPM_FLOAT,MFP_EDGE_FALL),
};

#define XRESOUT do { pxa3xx_gpio_set_direction(24,GPIO_DIR_OUT); } while(0)
#define XRESLOW do { pxa3xx_gpio_set_level(24,GPIO_LEVEL_LOW); } while(0)
#define XRESHIGH do { pxa3xx_gpio_set_level(24,GPIO_LEVEL_HIGH); } while(0)
#define SCLOUT do { pxa3xx_gpio_set_direction(21,GPIO_DIR_OUT); } while (0)
#define SCLIN do { pxa3xx_gpio_set_direction(21,GPIO_DIR_IN); } while (0)
#define SCLLOW do { pxa3xx_gpio_set_level(21,GPIO_LEVEL_LOW); } while (0)
#define SCLHIGH do { pxa3xx_gpio_set_level(21,GPIO_LEVEL_HIGH); } while (0)
#define SDAIN do { pxa3xx_gpio_set_direction(22,GPIO_DIR_IN); } while (0)
#define SDAOUT do { pxa3xx_gpio_set_direction(22,GPIO_DIR_OUT); } while (0)
#define SDALOW do { pxa3xx_gpio_set_level(22,GPIO_LEVEL_LOW); } while (0)
#define SDAHIGH do { pxa3xx_gpio_set_level(22,GPIO_LEVEL_HIGH); } while (0)
#define SDADATA (pxa3xx_gpio_get_level(22))
#define SCLDATA (pxa3xx_gpio_get_level(21))
#define BITDELAY do { udelay(1); } while(0)

extern void pxa3xx_enable_i2c_pins(void);

extern unsigned int woodstock_sleepflags;

static int cytouch_suspend(struct i2c_client *client,pm_message_t m)
{
	unsigned char buf[1];
	if (woodstock_sleepflags&SF_TOUCH) {
		if (cy_read(buf,1)<0) {
			printk("cytouch_suspend: couldn't read the control register\n");
			return 0;
		}
		buf[0]|=0x10;
		if (cy_write(buf,1)<0) {
			printk("cytouch_suspend: couldn't write the control register\n");
			return 0;
		}
		XRESLOW;
		XRESOUT;
		SCLIN;
		SDAIN;
		pxa3xx_mfp_set_configs(tp_programming_config,ARRAY_SIZE(tp_programming_config));
		while (!(SDADATA && SCLDATA)) ;
		pxa3xx_mfp_set_configs(tp_sleep_config,ARRAY_SIZE(tp_sleep_config));
	}
	return 0;
}

static int cytouch_resume(struct i2c_client *client)
{
	// csp->expect_reset=1;
	if (woodstock_sleepflags&SF_TOUCH) {
		csp->suppress_touch=1;
		SDAIN;
		SCLIN;
		pxa3xx_mfp_set_configs(tp_programming_config,ARRAY_SIZE(tp_programming_config));
		SCLLOW;
		SCLOUT;
		while (pxa3xx_gpio_get_level(23)) ;
		SCLIN;
		SDAIN;
		pxa3xx_enable_i2c_pins();
	} else {
		csp->expect_reset=1;
	}
	return 0;
}

struct pxa3xx_pin_config tp_config[]={
    PXA3xx_MFP_CFG("ATTN",MFP_PIN_GPIO23,MFP_AF0,MFP_DS03X,0,MFP_LPM_FLOAT,MFP_EDGE_NONE),
};

static inline void cytouch_reflash_bitsout(const unsigned char *bvec, int bits)
{
    int x,y;
    SDAOUT;
    for (x=0;x<bits;x++) {
        y=bvec[(x>>3)]&(0x80>>(x&7));
        if (y) SDAHIGH; else SDALOW;
        BITDELAY;
        SCLHIGH;
        BITDELAY;
        SCLLOW;
        BITDELAY;
    }
}

static inline unsigned char cytouch_reflash_bytein(void)
{
    unsigned char r=0;
    int x;
    // bus turnaround write to read
    SDAIN;
    BITDELAY;
    SCLHIGH;
    BITDELAY;
    SCLLOW;
    BITDELAY;
    SCLHIGH;
    BITDELAY;

    for (x=0;x<8;x++) {
        r<<=1;
        SCLLOW;
        BITDELAY;
        SCLHIGH;
        BITDELAY;
        if (SDADATA) r|=1;
    }

    // bus turnaround read to write
    SCLLOW;
    BITDELAY;
    // XXX BT again Cypress's documentation and implementation conflict
    // on whether the end of read turnaround is half a clock or 1.5 clocks...
    // SCLHIGH;
    // BITDELAY;
    // SCLLOW;
    // BITDELAY;


    return r;
}

#define WAITACK_MAXSPIN 100000 // probably around a second

static int cytouch_reflash_waitack(void)
{
// XXX BT the algorithm that Cypress documents in their app note and
// XXX the algorithm they supply in their code are quite different...

    // const unsigned char fortyzeroes[]={0,0,0,0,0};
    int x;
    SCLLOW;
    SDAIN;
    BITDELAY;
    for (x=0;x<WAITACK_MAXSPIN;x++) {
        if (SDADATA) break;
        SCLHIGH;
        BITDELAY;
        SCLLOW;
        BITDELAY;
    }
    if (x==WAITACK_MAXSPIN) {
        printk("During waitack SDA never went high\n");
        return -1;
    }
    BITDELAY;
    for (x=0;x<WAITACK_MAXSPIN;x++) {
        if (!SDADATA) break;
        SCLHIGH;
        BITDELAY;
        SCLLOW;
        BITDELAY;
    }
    if (x==WAITACK_MAXSPIN) {
        printk("During waitack SDA never went low\n");
        return -1;
    }
    BITDELAY;
    // cytouch_reflash_bitsout(fortyzeroes,40);
    return 0;
}



static const unsigned char idread1vec[] = { 0xBF,0x00 };
static const int idread1vecl = 11;

static const unsigned char idread2vec[] = { 0xDF,0x90 };
static const int idread2vecl = 12;

static const unsigned char idread3vec[] = { 0x80 };
static const int idread3vecl = 1;

static const unsigned char checksumread1vec[] = { 0xBF, 0x20 };
static const int checksumread1vecl = 11;

static const unsigned char checksumread2vec[] = { 0xDF, 0x80 };
static const int checksumread2vecl = 12;

static const unsigned char checksumread3vec[] = { 0x80 };
static const int checksumread3vecl = 1;

static const int init1vecl = 396;
static const unsigned char init1vec[] =
    {
        0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0D, 0xEE, 0x01, 0xF7, 0xB0, 0x07, 0x9F, 0x07,
        0x5E, 0x7C, 0x81, 0xFD, 0xEA, 0x01, 0xF7, 0xA0,
        0x1F, 0x9F, 0x70, 0x1F, 0x7C, 0x98, 0x7D, 0xF4,
        0x81, 0xF7, 0x80, 0x4F, 0xDF, 0x00, 0x1F, 0x7F,
        0x89, 0x70
    };

static const int init2vecl = 286;
static const unsigned char init2vec[] =
        {
            0xDE, 0xE0, 0x1F, 0x7B, 0x00, 0x79, 0xF0, 0x75,
            0xE7, 0xC8, 0x1F, 0xDE, 0xA0, 0x1F, 0x7A, 0x01,
            0xF9, 0xF7, 0x01, 0xF7, 0xC9, 0x87, 0xDF, 0x48,
            0x1E, 0x7D, 0x00, 0xFD, 0xE0, 0x0D, 0xF7, 0xC0,
            0x07, 0xDF, 0xE2, 0x5C
};

static const int init3vecl = 836;
static const unsigned char init3vec[] =
        {
            0xDE, 0xE0, 0x1F, 0x7A, 0x01, 0xFD, 0xEA, 0x01,
            0xF7, 0xB0, 0x47, 0xDF, 0x0A, 0x3F, 0x7C, 0xFC,
            0x7D, 0xF4, 0x61, 0xF7, 0xF8, 0x97, 0x00, 0x00,
            0x03, 0x7B, 0x80, 0x7D, 0xE8, 0x07, 0xF7, 0xA8,
            0x07, 0xDE, 0xC1, 0x1F, 0x7C, 0x30, 0x7D, 0xF3,
            0xD5, 0xF7, 0xD1, 0x87, 0xDE, 0xE2, 0x1F, 0x7F,
            0x89, 0x70, 0x00, 0x00, 0x37, 0xB8, 0x07, 0xDE,
            0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC, 0x11, 0xF7,
            0xC2, 0x8F, 0xDF, 0x3F, 0x3F, 0x7D, 0x18, 0x7D,
            0xFE, 0x25, 0xC0, 0x00, 0x00, 0xDE, 0xE0, 0x1F,
            0x7A, 0x01, 0xFD, 0xEA, 0x01, 0xF7, 0xB0, 0x47,
            0xDF, 0x0C, 0x1F, 0x7C, 0xF4, 0x7D, 0xF4, 0x61,
            0xF7, 0xB8, 0x87, 0xDF, 0xE2, 0x5C, 0x00, 0x00,
            0x00
};

static const int idsetupvecl = 330;
static const unsigned char idsetupvec[] =
    {
        0xDE, 0xE2, 0x1F, 0x70, 0x01, 0x7D, 0xEE, 0x01,
        0xF7, 0xB0, 0x07, 0x9F, 0x07, 0x5E, 0x7C, 0x81,
        0xFD, 0xEA, 0x01, 0xF7, 0xA0, 0x1F, 0x9F, 0x70,
        0x1F, 0x7C, 0x98, 0x7D, 0xF4, 0x81, 0xE7, 0xD0,
        0x07, 0xDE, 0x00, 0xDF, 0x7C, 0x00, 0x7D, 0xFE,
        0x25, 0xC0
};

static const int pollendvecl = 40;
static const unsigned char pollendvec[] = { 0,0,0,0,0 };

static const int checksumvecl = 286;
static const unsigned char checksumvec[] =
    {
        0xDE, 0xE0, 0x1F, 0x7B, 0x00, 0x79, 0xF0, 0x75,
        0xE7, 0xC8, 0x1F, 0xDE, 0xA0, 0x1F, 0x7A, 0x01,
        0xF9, 0xF6, 0x01, 0xF7, 0xC9, 0x87, 0xDF, 0x48,
        0x1E, 0x7D, 0x40, 0x7D, 0xE0, 0x0F, 0xF7, 0xC0,
        0x07, 0xDF, 0xE2, 0x5C
};

static const int erasevecl = 308;
static const unsigned char erasevec[] =
    {
        0x9F, 0x82, 0xBE, 0x7F, 0x2B, 0x7D, 0xEE, 0x01,
        0xF7, 0xB0, 0x07, 0x9F, 0x07, 0x5E, 0x7C, 0x81,
        0xFD, 0xEA, 0x01, 0xF7, 0xA0, 0x1F, 0x9F, 0x70,
        0x1F, 0x7C, 0x98, 0x7D, 0xF4, 0x81, 0xF7, 0x80,
        0x2F, 0xDF, 0x00, 0x1F, 0x7F, 0x89, 0x70
};

static const int verifyvecl = 264;
static const unsigned char verifyvec[] =
    {
        0xDE, 0xE0, 0x1F, 0x7B, 0x00, 0x79, 0xF0, 0x75,
        0xE7, 0xC8, 0x1F, 0xDE, 0xA0, 0x1F, 0x7A, 0x01,
        0xF9, 0xF7, 0x01, 0xF7, 0xC9, 0x87, 0xDF, 0x48,
        0x1F, 0x78, 0x00, 0xFD, 0xF0, 0x01, 0xF7, 0xF8,
        0x97
};

static const int programvecl = 308;
static const unsigned char programvec[] =
    {
        0x9F, 0x8A, 0x9E, 0x7F, 0x2B, 0x7D, 0xEE, 0x01,
        0xF7, 0xB0, 0x07, 0x9F, 0x07, 0x5E, 0x7C, 0x81,
        0xFD, 0xEA, 0x01, 0xF7, 0xA0, 0x1F, 0x9F, 0x70,
        0x1F, 0x7C, 0x98, 0x7D, 0xF4, 0x81, 0xF7, 0x80,
        0x17, 0xDF, 0x00, 0x1F, 0x7F, 0x89, 0x70
};



static void cytouch_reflash_writebuf(unsigned char *b)
{
    int x;
    unsigned char n[3];
    for (x=0;x<64;x++) {
        n[0] = 0x90 | ((x&0x38)>>3);
        n[1] = ((x&0x07)<<5) | ((b[x]&0xf8) >> 3);
        n[2] = ((b[x]&0x07)<<5) | 0x1C;
        cytouch_reflash_bitsout(n,22);
    }
}

static void cytouch_reflash_readbuf(unsigned char *b)
{
    int x;
    unsigned char n[2];
    for (x=0;x<64;x++) {
        n[0]=0xb0 | ((x&0x38)>>3);
        n[1]=((x&0x07)<<5);
        cytouch_reflash_bitsout(n,11);
        b[x]=cytouch_reflash_bytein();
        n[0]=0x80;
        cytouch_reflash_bitsout(n,1);
    }
}

static void cytouch_reflash_setblock(int block)
{
    unsigned char n[3];
    n[0]=0x9F;
    n[1]=0x40|((block&0xF8)>>3);
    n[2]=((block&0x07)<<5)|0x1C;
    cytouch_reflash_bitsout(n,22);
    //udelay(10); // XXX maybe this requires some recovery time?
}

static int cytouch_reflash(void)
{
    unsigned char r1,r2;
    unsigned char buf[64];
    int x,y;

    pxa3xx_mfp_set_configs(tp_programming_config,ARRAY_SIZE(tp_programming_config));

    local_irq_disable();
    XRESHIGH;
    XRESOUT;
    SCLLOW;
    SCLOUT;
    SDALOW;
    SDAIN;
    udelay(30);
    XRESLOW;
    udelay(30);
    cytouch_reflash_bitsout(init1vec,init1vecl);
    if (cytouch_reflash_waitack()) {
        printk("init1 ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec,pollendvecl);
    cytouch_reflash_bitsout(init2vec,init2vecl);
    if (cytouch_reflash_waitack()) {
        printk("init2 ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec,pollendvecl);
    cytouch_reflash_bitsout(init3vec,init3vecl);
    cytouch_reflash_bitsout(pollendvec,pollendvecl);
    
    cytouch_reflash_bitsout(idsetupvec,idsetupvecl);
    if (cytouch_reflash_waitack()) {
        printk("idsetup ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec,pollendvecl);
    mdelay(100);
    cytouch_reflash_bitsout(idread1vec,idread1vecl);
    r1=cytouch_reflash_bytein();
    cytouch_reflash_bitsout(idread2vec,idread2vecl);
    r2=cytouch_reflash_bytein();
    cytouch_reflash_bitsout(idread3vec,idread3vecl);

    printk("Device silicon ID is %02x%02x\n",r1,r2);
    if ((r1!=0x06)||(r2!=0x38)) {
        printk("Unexpected silicon ID, cowardly refusing to reflash\n");
        goto fail;
    }

#if 0
    cytouch_reflash_bitsout(checksumvec, checksumvecl);
    if (cytouch_reflash_waitack()) {
        printk("checksum ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec, pollendvecl);
    cytouch_reflash_bitsout(checksumread1vec, checksumread1vecl);
    r1=cytouch_reflash_bytein();
    cytouch_reflash_bitsout(checksumread2vec, checksumread2vecl);
    r2=cytouch_reflash_bytein();
    cytouch_reflash_bitsout(checksumread3vec, checksumread3vecl);
    printk("Device checksum is %02x%02x\n",r1,r2);
#endif
#if 1
    cytouch_reflash_bitsout(erasevec, erasevecl);
    if (cytouch_reflash_waitack()) {
        printk("erase ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec, pollendvecl);
    mdelay(100);
    printk("TP IC has been erased\nProgram");
#endif

    for (x=0;x<64;x++) buf[x]=63-x;
    for (x=0;x<128;x++) {
    cytouch_reflash_writebuf(fwmain+(x<<6));
    mdelay(100);
    cytouch_reflash_setblock(x);
    mdelay(100);
    cytouch_reflash_bitsout(programvec,programvecl);
    if (cytouch_reflash_waitack()) {
        printk("program ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec, pollendvecl);
    mdelay(100);
    printk(".");
    }

    for (x=0;x<64;x++) buf[x]=0xAA;

    printk("\nVerify");
    for (x=0;x<128;x++) {
    cytouch_reflash_setblock(x);
    mdelay(100);
    cytouch_reflash_bitsout(verifyvec,verifyvecl);
    if (cytouch_reflash_waitack()) {
        printk("verify ack failed\n");
        goto fail;
    }
    cytouch_reflash_bitsout(pollendvec, pollendvecl);
    mdelay(100);
    cytouch_reflash_readbuf(buf);
    if (memcmp(buf,fwmain+(x<<6),64)!=0) {
        printk("verify failed in block %d\n",x);
        for (y=0;y<64;y++) {
            printk(" %02x",buf[y]);
            if ((y&7)==7) printk("\n");
        }
        goto fail;
    }
    printk(".");
    }
    printk("\nprogram ok!\n");
    local_irq_enable();
    XRESHIGH;
    pxa3xx_enable_i2c_pins();
    udelay(30);
    XRESLOW;
    mdelay(50);
    return 0;


fail:
    printk("\nprogram fail!\n");
    local_irq_enable();
    return -1;

}


static int i2c_cytouch_detect_client(struct i2c_adapter *adapter,
		int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0, ret;
        int tried_reprogram=0;
        unsigned char v;
        unsigned int fwv;
        struct cy_i2c ci2c;

#ifdef VERBOSE
	printk("Called i2c_cytouch_detect_client adapter %p address %d kind %d\n",adapter,address,kind);
#endif

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL );

	if ( !new_client )  {
		err = -ENOMEM;
		goto ERROR0;
	}

	new_client->addr = address;	
 	new_client->adapter = adapter;
	new_client->driver = &i2c_cytouch_driver;
	new_client->flags = 0;
	strcpy(new_client->name, "cytouch");
	g_client=new_client;

#if 0
	{
	int i;
	for (i=0;i<128;i++) {
		g_client->addr=i;
		if ((CY_WRITE1(0,v))==0) {
			printk("Successful probe on address %d\n",i);
		}
	} }
	printk("Finished address probing\n");
#endif /*0*/

        if (cytouch_force_reflash) {
            printk("cytouch: reprogram forced by kernel parameters\n");
            goto reprogram;
        }
reprobe:
        if (cy_read(&ci2c,sizeof(ci2c))<0) {
            printk("cytouch: Could not talk to the TP, going to try to reprogram it\n");
            goto reprogram;
        }

        if ((ci2c.control&0x01)==0) {
            printk("cytouch: panel not in reset state when expected\n");
            printk("cytouch: going to try to reprogram the TP\n");
            goto reprogram;
        }
        fwv=(ci2c.fwver[0]<<8)|ci2c.fwver[1];
        printk("cytouch panel firmware version %04x\n",fwv);
        if (fwv!=FWVER) {
            printk("cytouch: unexpected firmware version, going to try to reprogram the TP\n");
            goto reprogram;
        }


	strcpy(new_client->name, "cytouch");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	csp=kzalloc(sizeof(struct cystate),GFP_KERNEL);
	if (csp==0) goto ERROR1;
	init_completion(&(csp->compl));
        mutex_init(&csp->mu);
	csp->idev=input_allocate_device();
	if (csp->idev==0) goto ERROR1;
	csp->idev->name="cytouch";
	csp->idev->phys="cytouch/input0";
	csp->idev->private=csp;
	// set_bit(EV_KEY,csp->idev->evbit);
	set_bit(EV_ABS,csp->idev->evbit);
	// set_bit(BTN_TOUCH,csp->idev->keybit);
	set_bit(ABS_X,csp->idev->absbit);
	set_bit(ABS_Y,csp->idev->absbit);
	set_bit(ABS_PRESSURE,csp->idev->absbit);
	if ((input_register_device(csp->idev))<0) goto ERROR1;
	csp->md.minor=CYTOUCH_MISC_MINOR;
	csp->md.name="cytouch_raw";
	csp->md.fops=&cytouch_raw_fops;
	misc_register(&csp->md);
	kernel_thread(cythread,0,0);
	pxa3xx_gpio_set_direction(23,GPIO_DIR_IN);
	pxa3xx_mfp_set_configs(tp_config,ARRAY_SIZE(tp_config));
	request_irq(IRQ_GPIO(23),cytouch_attn_int,IRQF_TRIGGER_FALLING|IRQF_DISABLED,"cytouch_attn",g_client);
	complete(&(csp->compl));
	return 0;

reprogram:
        if (tried_reprogram) {
            printk("cytouch: Attempted to reprogram the panel previously, bailing out\n");
            err=-EIO;
            goto ERROR1;
        }
	lt_charger_set_charge_led(1, 0, 0x0000ffff);
	ret = cytouch_reflash();
	lt_charger_set_charge_led(0, 0, 0);
        if (ret < 0) {
            printk("cytouch: failed to program the TP flash, problem with the interface?\n");
        } else {
            printk("cytouch: Reprogrammed flash\n");
            tried_reprogram=1;
            goto reprobe;
        }
        err=-EIO;

ERROR1:
	if (csp) {
		if (csp->idev) kfree(csp->idev);
		kfree(csp);
	}
	g_client = NULL;
	kfree(new_client);
ERROR0:
	printk(KERN_ALERT "i2c detect client failed\n");
        return err;
}

static int i2c_cytouch_detach_client(struct i2c_client *client)
{
	int err = 0;

  	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk(KERN_WARNING "cytouch: Client deregistration failed,"
			" client not detached.\n");
	        return err;
	}

	/* Frees client data too, if allocated at the same time */
	kfree(client);
	g_client = NULL;
	return 0;
}

static int __init cytouch_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&i2c_cytouch_driver))) {
		printk(KERN_WARNING "cytouch: Driver registration failed,"
			" module not inserted.\n");
		return ret;
	}
	// ret = platform_driver_register(&cytouch_driver);

	return ret;
}

static void __exit cytouch_exit(void)
{
	// platform_driver_unregister(&cytouch_driver);

	if (i2c_del_driver(&i2c_cytouch_driver)) {
		printk(KERN_WARNING "cytouch: Driver registration failed,"
			"module not removed.\n");
	}
} 

module_init(cytouch_init);
module_exit(cytouch_exit);

MODULE_DESCRIPTION("cytouch driver");
