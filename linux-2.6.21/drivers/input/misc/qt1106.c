#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <asm/arch/ssp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/mfp.h>
#include <asm/arch/irqs.h>

#define IS_DRDY() (pxa3xx_gpio_get_level(77))
#define IS_CHANGE() (pxa3xx_gpio_get_level(78))


struct qt1106_state {
    spinlock_t lock;
    struct ssp_dev sspdev;
    struct completion compl;
    struct input_dev *idev;
    int mode; // operating mode
    int realmode; // mode of qt1106 at this moment, may vary from mode because
                  // for example in all modes (except sync?) qt1106 goes into
                  // freerun upon detection
#define MODE_FREERUN 0x0
#define MODE_LP200 0x1
#define MODE_LP280 0x2
#define MODE_LP440 0x3
#define MODE_LP760 0x4
#define MODE_SYNC 0x5
#define MODE_SLEEP 0x6
    int state;
#define STATE_IDLE 0
#define STATE_WAKING1 1 //waiting to send the wake pulse
#define STATE_WAKING2 2
#define STATE_COMM1 3 //waiting to start the (main) SPI transaction
#define STATE_COMM2 4 //waiting for the SPI transaction to finish
    unsigned char cthresh;
    int load_cthresh;
    unsigned char config[3];
    unsigned char dout[3];
#define DO1(CT,PROX,SLD,AKS) (((CT)<<7)|((PROX)<<4)|((SLD)<<3)|(AKS))
#define DO2(MOD,DI,LPB,MODE) (((MOD)<<5)|((DI)<<4)|((LPB)<<3)|(MODE))
#define DO3(RES,CALW,CALK,CKN) (((RES)<<5)|((CALW)<<4)|((CALK)<<3)|(CKN))
    unsigned char din[3];
    unsigned char last_din[3];
#define DI1_CW(DI1) ((DI1)>>7)
#define DI1_CK(DI1) (((DI1)&0x40)>>6)
#define DI1_EW(DI1) (((DI1)&0x20)>>5)
#define DI1_EK(DI1) (((DI1)&0x10)>>4)
#define DI1_LPS(DI1) (((DI1)&0x08)>>3)
#define DI1_QM(DI1) (((DI1)&0x04)>>2)
#define DI1_CTL(DI1) ((DI1)&0x01)
#define DI2_W(DI2) ((DI2)>>7)
#define DI2_KEYS(DI2) ((DI2)&0x7f)
    int bytesin;
   
}; 

static inline void qt1106_xmit_attempt(struct qt1106_state *qst)
{
    if (!IS_DRDY()) return; // will get called again from the drdy handler
    switch(qst->state) {
        case STATE_WAKING1:
            ssp_write_word(&(qst->sspdev),0);
            qst->state=STATE_WAKING2;
            break;
        case STATE_COMM1:
            ssp_write_word(&(qst->sspdev),(unsigned int)qst->dout[0]);
            ssp_write_word(&(qst->sspdev),(unsigned int)qst->dout[1]);
            ssp_write_word(&(qst->sspdev),(unsigned int)qst->dout[2]);
            disable_irq_nosync(IRQ_GPIO(77));
            qst->state=STATE_COMM2;
            break;
        default:
            printk("qt1106_xmit_attempt does not expect to be called in state %d\n",qst->state);
    }
}

static inline void qt1106_comm_start(struct qt1106_state *qst,int change)
{
    if (qst->state!=STATE_IDLE) {
        printk("qt1106_comm_start can't start from a state other than IDLE\n");
    }
    if (qst->load_cthresh) {
        qst->dout[0]=0x80;
        qst->dout[1]=qst->cthresh;
        qst->dout[2]=0;
        qst->load_cthresh=0;
    } else {
        memcpy(qst->dout,qst->config,3);
    }
    qst->bytesin=0;
    if (change) qst->realmode=MODE_FREERUN;

    if (qst->realmode!=MODE_FREERUN) {
        /* need to wake the chip */
        qst->state=STATE_WAKING1;
    } else {
        qst->state=STATE_COMM1;
    }
    enable_irq(IRQ_GPIO(77));
    qt1106_xmit_attempt(qst);
}

static irqreturn_t qt1106_change_int(int irq,void *arg)
{
    struct qt1106_state *qst=arg;
    // printk("qt1106_change_int\n");
    spin_lock(&qst->lock);
    if (qst->state!=STATE_IDLE) {
        // XXX if necessary this will be picked up the next time we go idle
        goto done;
    }
    qt1106_comm_start(qst,1);
done:
    spin_unlock(&qst->lock);
    return IRQ_HANDLED;
}

static irqreturn_t qt1106_drdy_int(int irq,void *arg)
{
    struct qt1106_state *qst=arg;
    // printk("qt1106_drdy_int\n");
    spin_lock(&qst->lock);
    switch(qst->state) {
        case STATE_IDLE:
            if (IS_CHANGE()) qt1106_comm_start(qst,1);
            break;
        case STATE_WAKING2:
        case STATE_COMM2:
            break;
        case STATE_WAKING1:
        case STATE_COMM1:
            qt1106_xmit_attempt(qst);
            break;
        default:
            printk("unknown state %d in qt1106_drdy_int\n",qst->state);
    }
    spin_unlock(&qst->lock);
    return IRQ_HANDLED;
}

static void qt1106_rx_handler(void *arg)
{
    unsigned int tmp;
    struct qt1106_state *qst=arg;
    spin_lock(&qst->lock);
    switch (qst->state) {
        case STATE_WAKING2:
            ssp_read_word(&(qst->sspdev),&tmp); // This "data" is meaningless
            qst->state=STATE_COMM1;
            qt1106_xmit_attempt(qst);
            break;
        case STATE_COMM2:
            while ((qst->bytesin<3)&&(ssp_rcv_ready(&(qst->sspdev)))) {
                ssp_read_word(&(qst->sspdev),&tmp);
                qst->din[qst->bytesin++]=(unsigned char)tmp;
            }
            if (qst->bytesin==3) {
                // printk("qt1106 data in %02x %02x %02x\n",(unsigned int)qst->din[0],(unsigned int)qst->din[1],(unsigned int)qst->din[2]);
                // input_report_key(qst->idev,BTN_TOUCH,DI2_W(qst->din[1]));
                if (DI2_W(qst->din[1])) input_report_abs(qst->idev,ABS_X,(int)qst->din[2]); else input_report_abs(qst->idev,ABS_X,-1);
                // input_sync(qst->idev);
                complete(&(qst->compl));
                qst->state=STATE_IDLE;
                // XXX BT I think it's possible we ignored a change
                // back when we weren't idle in which case do it now
                // if (IS_CHANGE()) qt1106_comm_start(qst,1);
            }
            break;
        default:
            printk("Unexpected call to qt1106_rx_handler in state %d\n",qst->state);
    }
    spin_unlock(&qst->lock);
}

extern void *ssp_rcv_callback_arg;
extern void (*ssp_rcv_callback)(void *);

struct pxa3xx_pin_config qt1106_config[]={
    PXA3xx_MFP_CFG("RESET",MFP_PIN_GPIO80,MFP_AF0,MFP_DS03X,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("DRDY",MFP_PIN_GPIO77,MFP_AF0,MFP_DS03X,0,MFP_LPM_PULL_LOW|0x10,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("CHANGE",MFP_PIN_GPIO78,MFP_AF0,MFP_DS03X,0,MFP_LPM_PULL_LOW|0x10,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("CLK",MFP_PIN_GPIO85,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_HIGH,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("SS",MFP_PIN_GPIO86,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_HIGH,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("MOSI",MFP_PIN_GPIO87,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_LOW,MFP_EDGE_NONE),
    PXA3xx_MFP_CFG("MISO",MFP_PIN_GPIO88,MFP_AF1,MFP_DS03X,0,MFP_LPM_PULL_LOW|0x10,MFP_EDGE_NONE),
};

int qt1106_init(void)
{
    struct qt1106_state *qst=0;
    struct input_dev *idev=0;
    int ret=0;
    qst=kzalloc(sizeof(struct qt1106_state),GFP_KERNEL);
    if (!qst) {ret=-ENOMEM;goto fail;}
    spin_lock_init(&(qst->lock));
    init_completion(&(qst->compl));
    qst->mode=0;
    qst->realmode=0;
    qst->state=STATE_IDLE;
    qst->config[0]=0x00;
    qst->config[1]=0x10;
    qst->config[2]=0xC0;
    qst->cthresh=10;
    qst->load_cthresh=1;
    idev=input_allocate_device();
    if (!idev) {ret=-ENOMEM;goto fail;}
    qst->idev=idev;
    idev->name="woodstock_qt1106";
    idev->phys="woodstock_qt1106/input0";
    idev->private=qst;
    set_bit(EV_KEY,idev->evbit);
    set_bit(EV_ABS,idev->evbit);
    set_bit(BTN_TOUCH,idev->keybit);
    set_bit(ABS_X,idev->absbit);
    ret=input_register_device(idev);
    if (ret<0) goto fail;

    pxa3xx_gpio_set_level(80,GPIO_LEVEL_LOW);
    pxa3xx_gpio_set_direction(80,GPIO_DIR_OUT);
    pxa3xx_gpio_set_direction(77,GPIO_DIR_IN);
    pxa3xx_gpio_set_direction(78,GPIO_DIR_IN);
    pxa3xx_mfp_set_configs(qt1106_config,ARRAY_SIZE(qt1106_config));

    request_irq(IRQ_GPIO(77),qt1106_drdy_int,IRQF_TRIGGER_RISING|IRQF_DISABLED,"qt1106_drdy",qst);
    disable_irq_nosync(IRQ_GPIO(77));
    request_irq(IRQ_GPIO(78),qt1106_change_int,IRQF_TRIGGER_RISING|IRQF_DISABLED,"qt1106_change",qst);
    ssp_rcv_callback=qt1106_rx_handler;
    ssp_rcv_callback_arg=(void *)qst;
    ssp_init(&(qst->sspdev),1,0);
    ssp_config(&(qst->sspdev),0x00023c07,0x00000019,0,0);
    ssp_enable(&(qst->sspdev));
    mdelay(100);
    pxa3xx_gpio_set_level(80,GPIO_LEVEL_HIGH);
    return 0;

fail:
    if (idev) input_free_device(idev);
    if (qst) kfree(qst);
    return ret;
    
}

module_init(qt1106_init);
