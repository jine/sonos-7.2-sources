/*
 * Synaptics MEP TouchPad support
 *
 *
 * Copyright (C) 2008 Sonos (B. Tober)
 * based on i2c/chips/micco.c
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

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/irqs.h>

struct mepstate {
    struct completion compl;
    struct input_dev *idev;
} *msp;

static int meptouch_probe(struct platform_device *pdev)
{
	// int ret;

	return 0;
}

static int meptouch_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver meptouch_driver = {
	.driver = {
		.name	= "meptouch",
	},
	.probe		= meptouch_probe,
	.remove		= meptouch_remove,
};

/******************************************************************************
 *									      *
 *			MICCO I2C Client Driver				      *
 *									      *
 ******************************************************************************/
#include <linux/i2c.h>
static int i2c_meptouch_attach_adapter(struct i2c_adapter *adapter);
static int i2c_meptouch_detect_client(struct i2c_adapter *, int, int);
static int i2c_meptouch_detach_client(struct i2c_client *client);

struct i2c_driver i2c_meptouch_driver  = 
{
	.driver = {
		.name	= "synaptics MEP touchpad i2c client driver",
	},
	.attach_adapter	= &i2c_meptouch_attach_adapter, 
	.detach_client	= &i2c_meptouch_detach_client,  
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = {0x20, I2C_CLIENT_END };
I2C_CLIENT_INSMOD_1(meptouch);

int mep_write(unsigned char *,int);
int mep_read(unsigned char *,int *);


#define NATIVEF_FINGER 1
#define NATIVEF_GESTURE 2
#define NATIVEF_RELPOSVLD 4

void meptouch_decode_native(char *b,unsigned *x,unsigned *y,unsigned *z,unsigned *flags,unsigned *w,unsigned *zoom)
{
    *w=(b[1]&0xf0)>>4;
    *flags=(b[1]&0x0f);
    *x=b[3]|((b[2]&0x1f)<<8);
    *y=b[5]|((b[4]&0x1f)<<8);
    *z=b[6];
    *zoom=(((b[2]&0xe0)>>5)|((b[4]&0xe0)>>2))<<6;
}

int mepthread(void *arg)
{
    unsigned char b[9];
    int l,r,i;

    static int last_x = 0, last_y = 0;

    daemonize("mepthread");
    while (1) {
        wait_for_completion_interruptible(&(msp->compl));
        if (signal_pending(current)) try_to_freeze(); else
        do {
            r=mep_read(b,&l);
            if (r<0) {
                printk("mep_read failed\n");
            }
#if 0
            printk("mep_read returned");
            for (i=0;i<l;i++) printk(" %02x",(unsigned int)b[i]);
            printk("\n");
#endif
            if (b[0]==0x0e) {
                unsigned x,y,z,f,w,zoom;
                meptouch_decode_native(b,&x,&y,&z,&f,&w,&zoom);
                printk("meptouch native x=%u y=%u z=%u flags=%01x(%s%s%s) w=%u",
                       x,y,z,
                       f,
                       ((f&NATIVEF_FINGER)?"Finger ":""),
                       ((f&NATIVEF_GESTURE)?"Gesture ":""),
                       ((f&NATIVEF_RELPOSVLD)?"RelPosVld ":""),
                       w);
                if (w==0) printk(" zoomdist=%u",zoom);
                printk("\n");
                if (f&NATIVEF_FINGER) {
                    input_report_abs(msp->idev,ABS_X,(int)x);
                    input_report_abs(msp->idev,ABS_Y,(int)y);
                    input_report_abs(msp->idev,ABS_PRESSURE,(int)100);
                    last_x = x;
                    last_y = y;
                } else {
                    input_report_abs(msp->idev,ABS_X,(int)last_x);
                    input_report_abs(msp->idev,ABS_Y,(int)last_y);
                    input_report_abs(msp->idev,ABS_PRESSURE,(int)0);
                }
                break;
            }

        } while ((l!=1)||((b[0]&0x1f)!=0));
    }
    return 0;
}

irqreturn_t meptouch_attn_int(int irq,void *arg)
{
    complete(&(msp->compl));
    return IRQ_HANDLED;
}
    

int mep_write(unsigned char *p,int len)
{
    struct i2c_msg m;
    unsigned char x;
    int i;
    x=(g_client->addr)<<1;
    for (i=0;i<len;i++) x+=p[i];
    p[len]=x;
    m.addr=g_client->addr;
    m.flags=0;
    m.len=len+1;
    m.buf=p;
    return i2c_transfer(g_client->adapter,&m,1);
}

int mep_read(unsigned char *p,int *len)
{
    struct i2c_msg m;
    unsigned char x;
    int ret,i,j;
    m.addr=g_client->addr;
    m.flags=I2C_M_RD|I2C_M_MEP;
    m.len=2;
    m.buf=p;
    ret=i2c_transfer(g_client->adapter,&m,1);
    if (ret<0) return ret;
    x=(((g_client->addr)<<1)|1);
    i=(p[0]&0x07)+1;
    *len=i;
    for (j=0;j<i;j++) x+=p[j];
    if (x!=p[i]) {
        printk("mep_read: PEC check failed, read PEC %02x calculated PEC %02x\n",(unsigned int)p[i],(unsigned int)x);
        return -1;
    }
    return 0;
}

static int i2c_meptouch_attach_adapter(struct i2c_adapter *adap)
{	
	return i2c_probe(adap,&addr_data, &i2c_meptouch_detect_client);
}

struct pxa3xx_pin_config tp_config[]={
    PXA3xx_MFP_CFG("ATTN",MFP_PIN_GPIO23,MFP_AF0,MFP_DS03X,0,MFP_LPM_PULL_HIGH|0x10,MFP_EDGE_NONE),
};

static int i2c_meptouch_detect_client(struct i2c_adapter *adapter,
		int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;
	int chip_id;
	unsigned char b[9];
	int l,i;
 
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL );

	if ( !new_client )  {
		err = -ENOMEM;
		goto ERROR0;
	}

	new_client->addr = address;	
 	new_client->adapter = adapter;
	new_client->driver = &i2c_meptouch_driver;
	new_client->flags = 0;
	strcpy(new_client->name, "meptouch");
	g_client=new_client;

	b[0]=0x08;
	if ((mep_write(b,1))<0) {
		printk("mep_write reset failed\n");
		goto ERROR1;
	}
	mdelay(1000);
	if ((mep_read(b,&l))<0) {
		printk("mep_read failed\n");
		goto ERROR1;
	}
	printk("mep reset response:");
	for (i=0;i<l;i++) printk(" %02x",b[i]);
	printk("\n");
	b[0]=0x01;
	b[1]=0x80;
	if ((mep_write(b,2))<0) {
		printk("mep_write query 0 failed\n");
		goto ERROR1;
	}
	if ((mep_read(b,&l))<0) {
		printk("mep_read failed\n");
		goto ERROR1;
	}
	printk("mep query 0 response:");
	for (i=0;i<l;i++) printk(" %02x",b[i]);
	printk("\n");
#if 1
	b[0]=0x05;
	b[1]=0x60;
	b[2]=0x00;
	b[3]=0x00;
	b[4]=0x0f;
	b[5]=0x00;
	if ((mep_write(b,6))<0) {
		printk("mep_write getset parameter 0x20 failed\n");
		goto ERROR1;
	}
	if ((mep_read(b,&l))<0) {
		printk("mep_read failed\n");
		goto ERROR1;
	}
        printk("mep getset parameter 0x20 response:");
        for (i=0;i<l;i++) printk(" %02x",b[i]);
        printk("\n");

#endif
	b[0]=0x01;
	b[1]=0x60;
	if ((mep_write(b,2))<0) {
		printk("mep_write get parameter 0x20 failed\n");
		goto ERROR1;
	}
	if ((mep_read(b,&l))<0) {
		printk("mep_read failed\n");
		goto ERROR1;
	}
	printk("mep get parameter 0x20 response:");
        for (i=0;i<l;i++) printk(" %02x",b[i]);
        printk("\n");
#if 0
	b[0]=0x05;
	b[1]=0x61;
	b[2]=0x00;
	b[3]=0x80;
	b[4]=0x00;
	b[5]=0x80;
        if ((mep_write(b,6))<0) {
                printk("mep_write getset parameter 0x21 failed\n");
                goto ERROR1;
        }
        if ((mep_read(b,&l))<0) {
                printk("mep_read failed\n");
                goto ERROR1;
        }
        printk("mep getset parameter 0x21 response:");
        for (i=0;i<l;i++) printk(" %02x",b[i]);
        printk("\n");
#endif
        b[0]=0x01;
        b[1]=0x61;
        if ((mep_write(b,2))<0) {
                printk("mep_write get parameter 0x21 failed\n");
                goto ERROR1;
        }
        if ((mep_read(b,&l))<0) {
                printk("mep_read failed\n");
                goto ERROR1;
        }
        printk("mep get parameter 0x21 response:");
        for (i=0;i<l;i++) printk(" %02x",b[i]);
        printk("\n");


	strcpy(new_client->name, "meptouch");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	msp=kmalloc(sizeof(struct mepstate),GFP_KERNEL);
	if (msp==0) goto ERROR1;
	init_completion(&(msp->compl));
	msp->idev=input_allocate_device();
	if (msp->idev==0) goto ERROR1;
	msp->idev->name="meptouch";
	msp->idev->phys="meptouch/input0";
	msp->idev->private=msp;
	set_bit(EV_KEY,msp->idev->evbit);
	set_bit(EV_ABS,msp->idev->evbit);
	set_bit(BTN_TOUCH,msp->idev->keybit);
	set_bit(ABS_X,msp->idev->absbit);
	set_bit(ABS_Y,msp->idev->absbit);
	set_bit(ABS_PRESSURE,msp->idev->absbit);
	if ((input_register_device(msp->idev))<0) goto ERROR1;
	kernel_thread(mepthread,0,0);
	pxa3xx_gpio_set_direction(23,GPIO_DIR_IN);
	pxa3xx_mfp_set_configs(tp_config,ARRAY_SIZE(tp_config));
	request_irq(IRQ_GPIO(23),meptouch_attn_int,IRQF_TRIGGER_FALLING,"meptouch_attn",g_client);
	complete(&(msp->compl));
	return 0;

ERROR1:
	if (msp) {
		if (msp->idev) kfree(msp->idev);
		kfree(msp);
	}
	g_client = NULL;
	kfree(new_client);
ERROR0:
	printk(KERN_ALERT "i2c detect client failed\n");
        return err;
}

static int i2c_meptouch_detach_client(struct i2c_client *client)
{
	int err = 0;

  	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk(KERN_WARNING "meptouch: Client deregistration failed,"
			" client not detached.\n");
	        return err;
	}

	/* Frees client data too, if allocated at the same time */
	kfree(client);
	g_client = NULL;
	return 0;
}

static int __init meptouch_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&i2c_meptouch_driver))) {
		printk(KERN_WARNING "meptouch: Driver registration failed,"
			" module not inserted.\n");
		return ret;
	}
	// ret = platform_driver_register(&meptouch_driver);

	return ret;
}

static void __exit meptouch_exit(void)
{
	// platform_driver_unregister(&meptouch_driver);

	if (i2c_del_driver(&i2c_meptouch_driver)) {
		printk(KERN_WARNING "meptouch: Driver registration failed,"
			"module not removed.\n");
	}
} 

module_init(meptouch_init);
module_exit(meptouch_exit);

MODULE_DESCRIPTION("meptouch driver");
