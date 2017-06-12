#undef MOTIONSENSOR
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include <asm/arch/ssp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/mfp.h>
#include <asm/arch/irqs.h>
#include <asm/arch/woodstock_motion_ioctl.h>

#define MMA7455L_SELFTEST_MIN_XY_DEFLECT 0
#define MMA7455L_SELFTEST_MAX_XY_DEFLECT 0x40
#define MMA7455L_SELFTEST_MIN_Z_DEFLECT 0x30
#define MMA7455L_SELFTEST_MAX_Z_DEFLECT 0x50


#define MMA7455L_REG_XOUTL       0x0
#define MMA7455L_REG_XOUTH       0x1
#define MMA7455L_REG_YOUTL       0x2
#define MMA7455L_REG_YOUTH       0x3
#define MMA7455L_REG_ZOUTL       0x4
#define MMA7455L_REG_ZOUTH       0x5
#define MMA7455L_REG_XOUT8       0x6
#define MMA7455L_REG_YOUT8       0x7
#define MMA7455L_REG_ZOUT8       0x8
#define MMA7455L_REG_STATUS      0x9
#define MMA7455L_REG_DETSRC      0xA
#define MMA7455L_REG_TOUT        0xB
#define MMA7455L_REG_RESERVED    0xC
#define MMA7455L_REG_I2CAD       0xD
#define MMA7455L_REG_USRINF      0xE
#define MMA7455L_REG_WHOAMI      0xF
#define MMA7455L_REG_XOFFL       0x10
#define MMA7455L_REG_XOFFH       0x11
#define MMA7455L_REG_YOFFL       0x12
#define MMA7455L_REG_YOFFH       0x13
#define MMA7455L_REG_ZOFFL       0x14
#define MMA7455L_REG_ZOFFH       0x15
#define MMA7455L_REG_MCTL        0x16
#define MMA7455L_REG_INTRST      0x17
#define MMA7455L_REG_CTL1        0x18
#define MMA7455L_REG_CTL2        0x19
#define MMA7455L_REG_LDTH        0x1A
#define MMA7455L_REG_PDTH        0x1B
#define MMA7455L_REG_PW          0x1C
#define MMA7455L_REG_LT          0x1D
#define MMA7455L_REG_TW          0x1E
#define MMA7455L_REG_RESERVED1   0x1F


#define IS_DRDY() (pxa3xx_gpio_get_level(83))
#define IS_CHANGE() (pxa3xx_gpio_get_level(84))
#define IS_MOTION() (pxa3xx_gpio_get_level(127))

//
// The input device state is relevant to applications that want to receive
// accelerometer and motion sensor input device events. Applications need to:
//   - open the device
//   - send ioctl MOTION_MISC_IOCTL_INPUTDEV_ENABLE
//   - keep the device open for the duration of its usage
// When the device is closed or goes to STANDBY, input device events will 
// stop being sent.
//
// TODO: When application comes back from STANDBY, input device should be
// reenabled and state should be moved back to INPUTDEV_STATE_MEASUREMENT.
//
typedef enum _inputdev_state_t {
    INPUTDEV_STATE_INIT,
    INPUTDEV_STATE_MEASUREMENT,
    INPUTDEV_STATE_STANDBY,
} inputdev_state_t;

void inputdev_timer_fn(unsigned long arg);

static struct timer_list inputdev_timer = 
    TIMER_INITIALIZER(inputdev_timer_fn, 0, 0);

// motion sensed event parameters
#define MOTION_THRESH_DX  0x10 // 0.25G
#define MOTION_THRESH_DY  0x10
#define MOTION_THRESH_DZ  0x10
#define MOTION_CLUSTER_LEN   (HZ/10) // 0.1 secs
// orientation event parameters
#define ORIENT_THRESH_DX  5
#define ORIENT_THRESH_DY  5
#define ORIENT_THRESH_DZ  5
// input device timer period
#define INPUTDEV_TIMER_PERIOD  (HZ/10) // 0.1 sec

struct mma7455l_state {
    spinlock_t lock;
    struct ssp_dev sspdev;
    struct completion compl;
    dev_t dev;
    struct cdev cdev;
    unsigned char last_mctl;
#define DO_MCTL(DRPD,SPI3W,STON,GLVL,MODE) (((DRPD)<<6)|((SPI3W)<<5)\
        |((STON)<<4)|((GLVL)<<2)|(MODE))
#define DO_INTRST(CLRINT2,CLRINT1) (((CLRINT2)<<1)|(CLRINT1))
#define DO_CTL1(DFBW,THOPT,ZDA,YDA,XDA,INTRG,INTPIN) (((DFBW)<<7)|\
        ((THOPT)<<6)|((ZDA)<<5)|((YDA)<<4)|((XDA)<<3)|((INTRG)<<1)|(INTPIN))
#define DO_CTL2(DRV0,PDPL,LDPL) (((DRV0)<<2)|((PDPL)<<1)|(LDPL))
    unsigned char dout[2];
    unsigned char regaddr;
    u32 din[2];
    u32 last_x;
    u32 last_y;
    u32 last_z;
    int bytesin;
    ////////////////////////////////////
    // input device related data
    inputdev_state_t inputdev_state;
    struct input_dev *inputdev;
    int inputdev_x;
    int inputdev_y;
    int inputdev_z;
    int motion_thresh_DX; // minimum delta X for motion sensed event
    int motion_thresh_DY; // minimum delta Y for motion sensed event
    int motion_thresh_DZ; // minimum delta Z for motion sensed event
    unsigned long motion_cluster_len;
    unsigned long last_motion_cluster; // start of cluster in jiffies
    int orient_thresh_DX; // minimum delta X for orientation event
    int orient_thresh_DY; // minimum delta Y for orientation event
    int orient_thresh_DZ; // minimum delta Z for orientation event
    wstk_orientation_t cur_orientation;
    int enable_motion_events;
}; 


#define EX_MODE(VAL) ((VAL)&3)
#define EX_GLVL(VAL) ((VAL>>2)&3)
#define EX_STON(VAL) ((VAL>>4)&1)
#define EX_SPI3W(VAL) ((VAL>>5)&1)
#define EX_DRPD(VAL) ((VAL>>6)&1)

struct mma7455l_state *g_pQst=0;

///////////////////////////////////////////////////////////////////////////////


static void inline mma7455l_reg_write (void *arg, unsigned char regaddr, 
                                       unsigned char data)
{
    int r;
    struct mma7455l_state *qst=arg;
    qst->bytesin = 0;
    qst->regaddr = regaddr << 1 | 0x80;
    qst->dout[0] = data;
    r=ssp_write_word (&(qst->sspdev), ((qst->regaddr<<8)|qst->dout[0]));
    if (!r) r=ssp_read_word (&(qst->sspdev), qst->din);
    if (r) printk("mma7455l_reg_write: ssp failed\n");
    udelay(100); // XXX guarantee chip select enough deassertion time
}

static void inline mma7455l_reg_read (void *arg, unsigned char regaddr, 
                                      unsigned char *data)
{
    int r;
    struct mma7455l_state *qst=arg;
    qst->regaddr = regaddr << 1;
    qst->bytesin = 0;
    r=ssp_write_word (&(qst->sspdev), (qst->regaddr<<8));
    if (!r) r=ssp_read_word (&(qst->sspdev), qst->din);
    if (r) printk("mma7455l_reg_read: ssp failed\n");
    *data = qst->din[0]&0xff;
    udelay(100); // XXX guarantee chip select enough deassertion time
}

///////////////////////////////////////////////////////////////////////////////

static void mma7455l_sign_extend (unsigned char* in, unsigned char* out,int elevenbit);

static void mma7455l_get_x_y_z (struct mma7455l_state *qst, 
                                int *x, int *y, int *z) 
{
    unsigned char val[4];
    mma7455l_reg_read (qst, MMA7455L_REG_XOUTL, val);
    mma7455l_reg_read (qst, MMA7455L_REG_XOUTH, val + 1);
    mma7455l_sign_extend (val, (unsigned char*)x,0);
    mma7455l_reg_read (qst, MMA7455L_REG_YOUTL, val);
    mma7455l_reg_read (qst, MMA7455L_REG_YOUTH, val + 1);
    mma7455l_sign_extend (val, (unsigned char*)y,0);
    mma7455l_reg_read (qst, MMA7455L_REG_ZOUTL, val);
    mma7455l_reg_read (qst, MMA7455L_REG_ZOUTH, val + 1); 
    mma7455l_sign_extend (val, (unsigned char*)z,0);
}
    
static wstk_orientation_t get_orientation (struct mma7455l_state *qst, 
                                           int x, int y, int z) 
{  
    int absX = (x > 0) ? x : -x;
    int absY = (y > 0) ? y : -y;
    int absZ = (z > 0) ? z : -z;
    if (absZ - absX > qst->orient_thresh_DZ && 
            absZ - absY > qst->orient_thresh_DZ)
        return z > 0 ? MOTION_ORIENT_BACK : MOTION_ORIENT_FRONT;
    if (absX - absZ > qst->orient_thresh_DX && 
            absX - absY > qst->orient_thresh_DX)
        return x > 0 ? MOTION_ORIENT_BOTTOM : MOTION_ORIENT_TOP;
    if (absY - absX > qst->orient_thresh_DY &&
            absY - absZ > qst->orient_thresh_DY)
        return y > 0 ? MOTION_ORIENT_LEFT : MOTION_ORIENT_RIGHT;
    
   return MOTION_ORIENT_UNKNOWN;
}

//
// inputdev_update()
//
// This function detects and sends motion sensed and orientation events
// when in STATE_MEASUREMENT. 
//
// @param qst the device state variable is assumed to be safe to read
//            and write. Locking/unlocking should be done before/after
//            calling this function
// @param force_or_event if this is non-zero, an orientation event is sent
//            regardless of whether there was a change
//
static wstk_orientation_t inputdev_update(struct mma7455l_state *qst,
                                          int force_or_event) 
{
    int x, y, z, absDX, absDY, absDZ;
    int bIsNotPartOfCluster = 0;
    wstk_orientation_t cur_or = MOTION_ORIENT_UNKNOWN;
    if (qst->inputdev_state == INPUTDEV_STATE_MEASUREMENT) {
        mma7455l_get_x_y_z(qst, &x, &y, &z);
        cur_or = get_orientation(qst, x, y, z);
        if (force_or_event || (cur_or != MOTION_ORIENT_UNKNOWN && 
                qst->cur_orientation != cur_or)) 
        {
            if (qst->enable_motion_events) {
                // send an orientation change event
                input_report_key(qst->inputdev, cur_or, 1);
                input_report_key(qst->inputdev, cur_or, 0);
                input_sync(qst->inputdev);
            }
            qst->cur_orientation = cur_or;
        }
        // send motion event if x,y,z changed past a threshold
        absDX = x - qst->inputdev_x;
        absDX = (absDX > 0) ? absDX : -absDX;
        absDY = y - qst->inputdev_y;
        absDY = (absDY > 0) ? absDY : -absDY;
        absDZ = z - qst->inputdev_z;
        absDZ = (absDZ > 0) ? absDZ : -absDZ;
        bIsNotPartOfCluster =
            (jiffies - qst->last_motion_cluster > qst->motion_cluster_len);
        if ((absDX > qst->motion_thresh_DX || absDY > qst->motion_thresh_DY ||
             absDZ > qst->motion_thresh_DZ) && bIsNotPartOfCluster) 
        {
            // don't send an event if first time through
            if (qst->last_motion_cluster && qst->enable_motion_events) {
                // send a motion sensed event
                input_report_key(qst->inputdev, MOTION_SENSED_EVENT, 1);
                input_report_key(qst->inputdev, MOTION_SENSED_EVENT, 0);
                input_sync(qst->inputdev);
            }
            // store time for this motion sensed event
            qst->last_motion_cluster = jiffies;
        }
        // update state
        qst->inputdev_x = x;
        qst->inputdev_y = y;
        qst->inputdev_z = z;
    }
    
    return cur_or;
}

void inputdev_start_timer(struct mma7455l_state *qst)
{
    inputdev_timer.data = (unsigned long)qst;    
    if (timer_pending(&inputdev_timer)) {
        mod_timer(&inputdev_timer, jiffies + INPUTDEV_TIMER_PERIOD);
    } else {
        inputdev_timer.expires = jiffies + INPUTDEV_TIMER_PERIOD;
        add_timer(&inputdev_timer);
    }
}

void inputdev_stop_timer(void)
{
    if (timer_pending(&inputdev_timer))
        del_timer(&inputdev_timer);
}

void inputdev_timer_fn(unsigned long arg)
{
    struct mma7455l_state *qst=(struct mma7455l_state *)arg;
    unsigned long flags;
    spin_lock_irqsave(&qst->lock, flags);
    inputdev_update(qst,0);
    inputdev_start_timer(qst);
    spin_unlock_irqrestore(&qst->lock, flags);
}

///////////////////////////////////////////////////////////////////////////////

#ifdef MOTIONSENSOR
static irqreturn_t motion_change_int(int irq,void *arg)
{
    struct mma7455l_state *qst=arg;
    spin_lock(&qst->lock);
    if (qst->last_mctl) {
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, qst->last_mctl);
        qst->last_mctl = 0;
    }
    inputdev_update(qst, 0);
    spin_unlock(&qst->lock);
    return IRQ_HANDLED;
}
#endif

/****************************************************************************
 * File System I/O operations
 ***************************************************************************/

static int mma7455l_open(struct inode *inode, struct file *filp)
{
    struct mma7455l_state *qst;
    qst = container_of(inode->i_cdev, struct mma7455l_state, cdev);
    filp->private_data = qst;    
    return 0;
}	

static ssize_t mma7455l_read(struct file *file, char __user *buf, 
                             size_t count, loff_t *ppos)
{
    return 0;
}

/* We assume that the user buf will not larger than kbuf size */
static ssize_t mma7455l_write(struct file *file, const char __user *buf,
                              size_t count, loff_t *ppos)
{
    return count;
}

static int mma7455l_release(struct inode *inode, struct file *filp)
{
    struct mma7455l_state *qst = filp->private_data;
    int bDisabled = 0;
    spin_lock(&qst->lock);
    // exit input device state
    if (qst->inputdev_state != INPUTDEV_STATE_INIT) {
        qst->inputdev_state = INPUTDEV_STATE_INIT;
        bDisabled = 1;
        inputdev_stop_timer();
    }
    spin_unlock(&qst->lock);
    if (bDisabled)
        printk(KERN_DEBUG "motion: input events disabled\n");
    return 0;
}

static void mma7455l_sign_extend (unsigned char* in, unsigned char* out, int elevenbit) 
{
    unsigned char sbit,mask;
    if (elevenbit) {sbit=0x04;mask=0x07;} else {sbit=0x02;mask=0x03;}
    if (in[1] & sbit) {
        out[3] = out[2] = 0xFF;
        out[1] = (~mask) | in[1];
        out[0] = in[0];
    } else {
        out[3] = out[2] = 0;
        out[1] = mask & in[1];
        out[0] = in[0];
    }
}

static void mma7455l_int_to_reg (unsigned char *in, unsigned char *out,int elevenbit)
{
    out[0]=in[0];
    if (elevenbit) out[1]=in[1]&0x07; else out[1]=in[1]&0x03;
}

static void mma7455l_adjust (struct mma7455l_state *qst, 
                             int offs_l, int offs_h, 
                             int val_l, int val_h, 
                             int target, int *saved_off) 
{
    int prev = 0, cur;
    unsigned char val[2];

    mma7455l_reg_read (qst, offs_l, val);
    mma7455l_reg_read (qst, offs_h, val + 1);
    mma7455l_sign_extend (val,(unsigned char *)&prev,1);
    if (saved_off) *saved_off=prev;
    mma7455l_reg_read (qst, val_l, val);
    mma7455l_reg_read (qst, val_h, val + 1);
    mma7455l_sign_extend (val,(unsigned char *)&cur,0);
    //  printk ("target %x vs current %x\n", target, cur);
    prev += -2*(cur-target);
    //  printk ("adjusting to %x\n", prev);
    mma7455l_int_to_reg ((unsigned char *)&prev, val,1);
    mma7455l_reg_write (qst, offs_l, val[0]);
    mma7455l_reg_write (qst, offs_h, val[1]);
}

static int mma7455l_ioctl (struct inode *inode, struct file *filp, 
                           unsigned int cmd, unsigned long arg)
{
    struct mma7455l_state *qst = filp->private_data;
    unsigned long flags;
    unsigned char buf[4];
    unsigned char n8bit;
    unsigned char val[4];
    int ret=0, diff, got, x, y, z;
    if (_IOC_SIZE(cmd)>4) return -EIO;
    if (_IOC_DIR(cmd)&_IOC_WRITE) {
        if (copy_from_user(buf,(void *)arg,_IOC_SIZE(cmd))) return -EFAULT;
    }
    spin_lock_irqsave(&(qst->lock),flags);
    buf[3] = buf[2] = 0;
    switch(cmd) {
    case MOTION_MISC_IOCTL_GET_X:
        mma7455l_reg_read (qst, MMA7455L_REG_XOUT8, &n8bit);
        *((int*)buf) = n8bit;
        break;
    case MOTION_MISC_IOCTL_GET_Y:
        mma7455l_reg_read (qst, MMA7455L_REG_YOUT8, &n8bit);
        *((int*)buf) = n8bit;
        break;
    case MOTION_MISC_IOCTL_GET_Z:
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUT8, &n8bit);
        *((int*)buf) = n8bit;
        break;
    case MOTION_MISC_IOCTL_SET_G_RANGE:
        mma7455l_reg_read (qst, MMA7455L_REG_MCTL, &n8bit);
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL,
                            DO_MCTL (EX_DRPD (n8bit), 
                                    EX_SPI3W (n8bit), 
                                    EX_STON (n8bit), 
                                    arg, 
                                    EX_MODE (n8bit)));
        break;
    case MOTION_MISC_IOCTL_SET_MODE: 
        mma7455l_reg_read (qst, MMA7455L_REG_MCTL, &n8bit);
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL,
                            DO_MCTL (EX_DRPD (n8bit), 
                                    EX_SPI3W (n8bit), 
                                    EX_STON (n8bit), 
                                    EX_GLVL (n8bit), 
                                    arg));
        break;
    case MOTION_MISC_IOCTL_GET_MCTL:
        mma7455l_reg_read (qst, MMA7455L_REG_MCTL, &n8bit);
        *((int*)buf) = n8bit;
        break;
    case MOTION_MISC_IOCTL_SET_MCTL:
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, arg);
        break;
    case MOTION_MISC_IOCTL_GET_X10:
        mma7455l_reg_read (qst, MMA7455L_REG_XOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_XOUTH, val + 1);
        mma7455l_sign_extend (val, buf,0);
        break;
    case MOTION_MISC_IOCTL_GET_Y10:
        mma7455l_reg_read (qst, MMA7455L_REG_YOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_YOUTH, val + 1);
        mma7455l_sign_extend (val, buf,0);
        break;
    case MOTION_MISC_IOCTL_GET_Z10:
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUTH, val + 1); 
        mma7455l_sign_extend (val, buf,0);
        break;
    case MOTION_MISC_IOCTL_GET_OFFSET_X:
        mma7455l_reg_read (qst, MMA7455L_REG_XOFFL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_XOFFH, val + 1);
        mma7455l_sign_extend (val, buf,1);
        break;
    case MOTION_MISC_IOCTL_GET_OFFSET_Y:
        mma7455l_reg_read (qst, MMA7455L_REG_YOFFL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_YOFFH, val + 1);
        mma7455l_sign_extend (val, buf,1);
        break;
    case MOTION_MISC_IOCTL_GET_OFFSET_Z:
        mma7455l_reg_read (qst, MMA7455L_REG_ZOFFL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_ZOFFH, val + 1); 
        mma7455l_sign_extend (val, buf,1);
        break;
    case MOTION_MISC_IOCTL_GET_STATUS:
        mma7455l_reg_read (qst, MMA7455L_REG_STATUS, &n8bit);
        *((int*)buf) = n8bit;
        break;
    case MOTION_MISC_IOCTL_SET_OFFSET_X:
        mma7455l_reg_write (qst, MMA7455L_REG_XOFFL, arg & 0xFF);
        mma7455l_reg_write (qst, MMA7455L_REG_XOFFH, (arg >> 8) & 0x7);
        break;
    case MOTION_MISC_IOCTL_SET_OFFSET_Y:
        mma7455l_reg_write (qst, MMA7455L_REG_YOFFL, arg & 0xFF);
        mma7455l_reg_write (qst, MMA7455L_REG_YOFFH, (arg >> 8) & 0x7);
        break;
    case MOTION_MISC_IOCTL_SET_OFFSET_Z:
        mma7455l_reg_write (qst, MMA7455L_REG_ZOFFL, arg & 0xFF);
        mma7455l_reg_write (qst, MMA7455L_REG_ZOFFH, (arg >> 8) & 0x7);
        break;
    case MOTION_MISC_IOCTL_WHICH_ENDS_UP:
        if (qst->inputdev_state == INPUTDEV_STATE_MEASUREMENT) {
            n8bit = inputdev_update(qst, 1);
        } else {
            mma7455l_get_x_y_z(qst, &x, &y, &z);
            n8bit = get_orientation (qst, x, y, z);
        }
        *((int*)buf) = n8bit;
        break;
    case MOTION_MISC_IOCTL_DO_SELFTEST:
        // read x,y,z values before self-test, set self-test mode control bit
        mma7455l_reg_read (qst, MMA7455L_REG_MCTL, &n8bit);
        if (n8bit == 0xFF) {
            printk ("motion: accelerometer power problem - reading mctl gave "
                    "0xFF\n");
            *((int*)buf) = 1;
            goto unlock_out;
        } else if ((n8bit & 0x3) == 0) {
            printk ("motion: accelerometer in stand-by mode\n");
            *((int*)buf) = 1;
            goto unlock_out;
        }
        mma7455l_reg_read (qst, MMA7455L_REG_XOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_XOUTH, val + 1);
        mma7455l_sign_extend (val,(unsigned char *)&(qst->last_x),0);
        mma7455l_reg_read (qst, MMA7455L_REG_YOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_YOUTH, val + 1);
        mma7455l_sign_extend (val,(unsigned char *)&(qst->last_y),0);
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUTH, val + 1);
        mma7455l_sign_extend (val,(unsigned char *)&(qst->last_z),0);
        mma7455l_reg_read (qst, MMA7455L_REG_MCTL, &(qst->last_mctl));
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, qst->last_mctl | 0x10);
        *((int*)buf) = 0;
        break;
    case MOTION_MISC_IOCTL_CHK_SELFTEST:
        // read x,y,z values before self-test, set self-test mode control bit
        mma7455l_reg_read (qst, MMA7455L_REG_XOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_XOUTH, val + 1);
        mma7455l_sign_extend (val,(unsigned char *)&got, 0);
        diff = got - qst->last_x; if (diff < 0) diff = -diff;
        if (diff > MMA7455L_SELFTEST_MAX_XY_DEFLECT) {
            printk ("motion: selftest deflected x axis too large, initial=%x, "
                    "selftest=%x\n", qst->last_x, got);
            *((int*)buf) = 1;
            goto unlock_out;
        }
        if (diff < MMA7455L_SELFTEST_MIN_XY_DEFLECT) {
            printk ("motion: selftest deflected x axis too small, initial=%x, "
                    "selftest=%x\n", qst->last_x, got);
            *((int*)buf) = 1;
            goto unlock_out;
        }
        mma7455l_reg_read (qst, MMA7455L_REG_YOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_YOUTH, val + 1);
        mma7455l_sign_extend (val,(unsigned char *)&got, 0);
        diff = got - qst->last_y; if (diff < 0) diff = -diff;
        if (diff > MMA7455L_SELFTEST_MAX_XY_DEFLECT) {
            printk ("motion: selftest deflected y axis too large, initial=%x, "
                    "selftest=%x\n", qst->last_y, got);
            *((int*)buf) = 1;
            goto unlock_out;
        }
        if (diff < MMA7455L_SELFTEST_MIN_XY_DEFLECT) {
            printk ("motion: selftest deflected y axis too small, initial=%x, "
                    "selftest=%x\n", qst->last_y, got);
            *((int*)buf) = 1;
            goto unlock_out;
        }
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUTL, val);
        mma7455l_reg_read (qst, MMA7455L_REG_ZOUTH, val + 1);
        mma7455l_sign_extend (val,(unsigned char *)&got, 0);
        diff = got - qst->last_z; if (diff < 0) diff = -diff;
        if (diff > MMA7455L_SELFTEST_MAX_Z_DEFLECT) {
            printk ("motion: selftest deflected z axis too large, initial=%x, "
                    "selftest=%x\n", qst->last_z, got);
            *((int*)buf) = 1;
            goto unlock_out;
        }
        if (diff < MMA7455L_SELFTEST_MIN_Z_DEFLECT) {
            printk ("motion: selftest deflected z axis too small, initial=%x, "
                    "selftest=%x\n", qst->last_z, got);
            *((int*)buf) = 1;
            goto unlock_out;
        }
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, qst->last_mctl);
	qst->last_mctl = 0;
        *((int*)buf) = 0;
        break;
    case MOTION_MISC_IOCTL_DO_CALIBRATE:
        // read 0 values, set offsets for X, Y to -2*value, for Z, 
        // set to -0x40 - 2*value
        mma7455l_adjust (qst, MMA7455L_REG_XOFFL, MMA7455L_REG_XOFFH, 
                         MMA7455L_REG_XOUTL, MMA7455L_REG_XOUTH, 0,0); 
        mma7455l_adjust (qst, MMA7455L_REG_YOFFL, MMA7455L_REG_YOFFH, 
                         MMA7455L_REG_YOUTL, MMA7455L_REG_YOUTH, 0,0); 
        mma7455l_adjust (qst, MMA7455L_REG_ZOFFL, MMA7455L_REG_ZOFFH, 
                         MMA7455L_REG_ZOUTL, MMA7455L_REG_ZOUTH, -0x40,0); 
        *((int*)buf) = 0;
        break;
    case MOTION_MISC_IOCTL_STANDBY:
        //
        // TODO: when coming back from standby, if inputdev_state is STANDBY
        // then input device events have to re-enabled and inputdev_state
        // should be moved back to INPUTDEV_STATE_MEASUREMENT.
        //
        if (qst->inputdev_state == INPUTDEV_STATE_MEASUREMENT) {
            qst->inputdev_state = INPUTDEV_STATE_STANDBY;
            inputdev_stop_timer();
        }
        // mma7455l_reg_read (qst, MMA7455L_REG_MCTL, &(qst->last_mctl));
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, 0);
        *((int*)buf) = 0;
        break;
    case MOTION_MISC_IOCTL_INPUTDEV_ENABLE:
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, 0x41);
        if (qst->inputdev_state != INPUTDEV_STATE_MEASUREMENT) {
            qst->inputdev_state = INPUTDEV_STATE_MEASUREMENT;
            inputdev_update(qst, 1);
            qst->enable_motion_events = 1;
            printk(KERN_DEBUG "motion: input events enabled\n");
#ifndef MOTIONSENSOR
            inputdev_start_timer(qst);
#endif
        }
        break;
    case MOTION_MISC_IOCTL_SET_MOTION_THRESH:
        qst->motion_thresh_DX = 
            qst->motion_thresh_DY = 
            qst->motion_thresh_DZ = arg; 
        printk(KERN_DEBUG "motion: set motion threshold to %lu\n", arg);
        break;
    case MOTION_MISC_IOCTL_SET_ORIENT_THRESH:
        qst->orient_thresh_DX = 
            qst->orient_thresh_DY = 
            qst->orient_thresh_DZ = arg; 
        printk(KERN_DEBUG "motion: set orientation threshold to %lu\n", arg);
        break;
    case MOTION_MISC_IOCTL_SET_CLUSTER_LEN_IN_MSECS:
        qst->motion_cluster_len = arg * HZ / 1000; 
        printk(KERN_DEBUG "motion: set motion cluster length to %lu msecs\n", arg);
        break;
    case MOTION_MISC_IOCTL_INPUTDEV_TIMER_CTL:
        if (arg) {
            inputdev_start_timer(qst);
            printk(KERN_DEBUG "motion: enable input device timer\n");
        } else {
            inputdev_stop_timer();
            printk(KERN_DEBUG "motion: disable input device timer\n");
        }
        break;
    case MOTION_MISC_IOCTL_ENABLE_MOTION_EVENTS:
        qst->enable_motion_events = arg;
        printk(KERN_DEBUG "motion: %s motion events\n", (arg) ? "enable" : 
              "disable");
        break;
        
    default:
        ret= -EIO;
        goto unlock_out;
    }
unlock_out:
    spin_unlock_irqrestore(&(qst->lock),flags);
    if (ret) return ret;
    
    if (_IOC_DIR(cmd)&_IOC_READ) {
        if (copy_to_user((void *)arg,buf,_IOC_SIZE(cmd))) return -EFAULT;
    }
    return 0;
}

static const struct file_operations mma7455l_fops = {
	.owner =	THIS_MODULE,
	.read =		mma7455l_read,
	.write =	mma7455l_write,
	.open =		mma7455l_open,
	.release =	mma7455l_release,
	.ioctl =	mma7455l_ioctl
};

extern void woodstock_motionsensor_sleep_config(void);

static int mma_suspend(struct platform_device *pdev, pm_message_t state)
{
    unsigned long flags;
    int x,y,z;
    int exclude=2;
    int limit=8;
    //unsigned char ldth,ctl1,ctl2,mctl,detsrc;
    struct mma7455l_state *qst = g_pQst;
    spin_lock_irqsave(&(qst->lock),flags);
    inputdev_stop_timer();
    if ((qst->inputdev_state == INPUTDEV_STATE_MEASUREMENT)&&(qst->enable_motion_events)) {
        mma7455l_get_x_y_z(qst, &x, &y, &z);
        // printk("entering standby, x=%d y=%d z=%d\n",x,y,z);
        if (x<0) x= -x;
        if (y<0) y= -y;
        if (z<0) z= -z;
        x>>=2;
        y>>=2;
        z>>=2;
        if ((x>y)&&(x>z)) {exclude=0; if (y>z) limit=y; else limit=z;}
        if ((y>x)&&(y>z)) {exclude=1; if (x>z) limit=x; else limit=z;}
        if ((z>x)&&(z>y)) {exclude=2; if (x>y) limit=x; else limit=y;}
        limit+=8;
        if (limit>15) limit=15;
        // printk("entering standby, exclude %d limit %d\n",exclude,limit);
        mma7455l_reg_write(qst, MMA7455L_REG_CTL1, 1<<(exclude+3));
        mma7455l_reg_write(qst, MMA7455L_REG_CTL2, 0x00);
        mma7455l_reg_write(qst, MMA7455L_REG_LDTH, limit);
        mma7455l_reg_write(qst, MMA7455L_REG_INTRST, 0x03);
        woodstock_motionsensor_sleep_config();
        mma7455l_reg_write(qst, MMA7455L_REG_INTRST, 0x00);
        mma7455l_reg_write(qst, MMA7455L_REG_MCTL, 0x42);
#if 0
        mma7455l_reg_read(qst, MMA7455L_REG_CTL1, &ctl1);
        mma7455l_reg_read(qst, MMA7455L_REG_CTL2, &ctl2);
        mma7455l_reg_read(qst, MMA7455L_REG_LDTH, &ldth);
        mma7455l_reg_read(qst, MMA7455L_REG_MCTL, &mctl);
        mma7455l_reg_read(qst, MMA7455L_REG_DETSRC, &detsrc);
        printk("readback regs CTL1=%02x CTL2=%02x LDTH=%02x MCTL=%02x DETSRC=%02x\n",ctl1,ctl2,ldth,mctl,detsrc);
#endif

    } else {
        mma7455l_reg_write(qst, MMA7455L_REG_MCTL, 0);
    }

#ifdef MOTIONSENSOR
    disable_irq(IRQ_GPIO(127));
#endif
    // disable_irq(IRQ_GPIO(83));
    //printk("mma_suspend done\n");
    spin_unlock_irqrestore(&(qst->lock),flags);
    return 0;
}

// XXX BT the following is called from the PM code, not the device restore
// XXX code below.  This is done because it is the PM code that releases the
// XXX pins from the sleep state, and this needs to be done before that in
// XXX order to guarantee a glitch-free transition out of standby
void mma_resume_ssp(void)
{
    struct mma7455l_state *qst=g_pQst;
    (void)ssp_config(&(qst->sspdev),0x00023c0f,0,0,0);
    ssp_enable(&(qst->sspdev));
}

static int mma_resume(struct platform_device *pdev)
{
    unsigned long flags;
    struct mma7455l_state *qst = g_pQst;
    //unsigned char detsrc,ldth,ctl1,ctl2,mctl,detsrc2;
    spin_lock_irqsave(&(qst->lock),flags);
    if (qst->inputdev_state == INPUTDEV_STATE_MEASUREMENT) {
#if 0
        mma7455l_reg_read(qst,MMA7455L_REG_DETSRC, &detsrc);
        mma7455l_reg_read(qst,MMA7455L_REG_LDTH, &ldth);
        mma7455l_reg_read(qst,MMA7455L_REG_CTL1, &ctl1);
        mma7455l_reg_read(qst,MMA7455L_REG_CTL2, &ctl2);
        mma7455l_reg_read(qst,MMA7455L_REG_MCTL, &mctl);
        mma7455l_reg_read(qst,MMA7455L_REG_DETSRC, &detsrc2);
        printk("mma_resume: DETSRC=%02x LDTH=%02x CTL1=%02x CTL2=%02x MCTL=%02x DETSRC2=%02x\n",detsrc,ldth,ctl1,ctl2,mctl,detsrc2);
#endif
        mma7455l_reg_write (qst, MMA7455L_REG_MCTL, 0x41);
        mma7455l_reg_write(qst, MMA7455L_REG_INTRST, 0x03);
        mma7455l_reg_write(qst, MMA7455L_REG_INTRST, 0x00);
        qst->last_mctl=0;
        inputdev_start_timer(qst);
    }
#ifdef MOTIONSENSOR
    enable_irq(IRQ_GPIO(127));
#endif
    // enable_irq(IRQ_GPIO(83));
    // if (qst->inputdev_state == INPUTDEV_STATE_MOTION) {
        // mma7455l_reg_write(qst, MMA7455L_REG_INTRST, 0x01);
        // mma7455l_reg_write(qst, MMA7455L_REG_INTRST, 0x00);
    // }
    spin_unlock_irqrestore(&(qst->lock),flags);
    return 0;

}

static int mma_probe(struct platform_device *dev)
{
    printk("called mma_probe\n");
    platform_set_drvdata(dev,g_pQst);
    return 0;
}

static struct platform_driver mma_driver = {
        .driver = {
                .name   = "mma7455l",
        },
	.probe		= mma_probe,
        .suspend        = mma_suspend,
        .resume         = mma_resume,
};

int mma7455l_setup(void)
{
    struct mma7455l_state *qst=0;
    int ret=0;
    struct input_dev *inputdev = NULL;
    unsigned char reg;
    g_pQst = qst = kzalloc(sizeof(struct mma7455l_state),GFP_KERNEL);
    if (!qst) {ret=-ENOMEM;goto fail;}
    spin_lock_init(&(qst->lock));
    init_completion(&(qst->compl));
    alloc_chrdev_region (&(qst->dev), 0, 1, "motion");
    cdev_init (&(qst->cdev), &mma7455l_fops);
    qst->cdev.owner = THIS_MODULE;
    ret = cdev_add (&(qst->cdev), qst->dev, 1);
    if (ret) {
        printk(KERN_ERR "motion: error %d adding mma7455l device!", ret);
        goto fail;
    }
    // initialize input device related data
    qst->inputdev_state = INPUTDEV_STATE_INIT;
    qst->motion_thresh_DX = MOTION_THRESH_DX;
    qst->motion_thresh_DY = MOTION_THRESH_DY;
    qst->motion_thresh_DZ = MOTION_THRESH_DZ;
    qst->orient_thresh_DX = ORIENT_THRESH_DX;
    qst->orient_thresh_DY = ORIENT_THRESH_DY;
    qst->orient_thresh_DZ = ORIENT_THRESH_DZ;
    qst->motion_cluster_len = MOTION_CLUSTER_LEN;
    qst->last_motion_cluster = 0;

    pxa3xx_gpio_set_direction(83,GPIO_DIR_IN);
    pxa3xx_gpio_set_direction(84,GPIO_DIR_IN);
    pxa3xx_gpio_set_direction(127,GPIO_DIR_IN);

#ifdef MOTIONSENSOR
    request_irq(IRQ_GPIO(127),motion_change_int,
                IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_DISABLED,"motion_change",qst);
#endif
#if 0
    request_irq(IRQ_GPIO(83),mma7455l_int1_int,
                IRQF_TRIGGER_RISING|IRQF_DISABLED,"mma7455l_int1",qst);
    request_irq(IRQ_GPIO(84),mma7455l_change_int,
                IRQF_TRIGGER_RISING|IRQF_DISABLED,"mma7455l_change",qst);
    disable_irq_nosync(IRQ_GPIO(84));
#endif

    // ssp_rcv_callback=mma7455l_rx_handler;
    // ssp_rcv_callback_arg=(void *)qst;
    ret = ssp_init(&(qst->sspdev),1,SSP_NO_IRQ);
    if (ret) printk ("motion: ssp_init returned %d\n", ret);
    ret = ssp_config(&(qst->sspdev),0x00023c0f,0,0,0);
    if (ret) printk ("motion: ssp_config returned %d\n", ret);
    ssp_enable(&(qst->sspdev));
    mdelay(100);
    mma7455l_reg_read(qst,MMA7455L_REG_I2CAD,&reg);
    //printk("mma7455l: I2CAD=%02x\n",reg);
    mma7455l_reg_write(qst,MMA7455L_REG_I2CAD,reg|0x80);
    //reg=0;
    //mma7455l_reg_read(qst,MMA7455L_REG_I2CAD,&reg);
    //printk("mma7455l: I2CAD readback=%02x\n",reg);
    
    complete(&(qst->compl));
    
    /* set up input device driver */
    inputdev = input_allocate_device();
    if (!inputdev) {
        ret = -ENOMEM;
        printk(KERN_ERR "motion: error allocating input device!\n");
        goto fail;
    }
    inputdev->name = "Woodstock Accelerometer and Motion Sensor";
    inputdev->evbit[0] = BIT(EV_KEY);
    set_bit(MOTION_ORIENT_FRONT, inputdev->keybit);
    set_bit(MOTION_ORIENT_BACK, inputdev->keybit);
    set_bit(MOTION_ORIENT_TOP, inputdev->keybit);
    set_bit(MOTION_ORIENT_BOTTOM, inputdev->keybit);
    set_bit(MOTION_ORIENT_LEFT, inputdev->keybit);
    set_bit(MOTION_ORIENT_RIGHT, inputdev->keybit);
    set_bit(MOTION_SENSED_EVENT, inputdev->keybit);
    
    ret = input_register_device(inputdev);
    if (ret) {
        goto fail_input_dev;
    }
    qst->inputdev = inputdev;

    platform_driver_register(&mma_driver);
    
    return 0;
    
fail_input_dev:
    input_free_device(inputdev);
    
fail:
    if (qst) kfree(qst);
    return ret;
}

static void mma7455l_teardown (void)
{
    free_irq(IRQ_GPIO(127),g_pQst);
    free_irq(IRQ_GPIO(84),g_pQst);
    input_unregister_device(g_pQst->inputdev);
    cdev_del(&(g_pQst->cdev));
    unregister_chrdev_region(g_pQst->dev, 1);
    ssp_disable (&(g_pQst->sspdev));
    ssp_exit (&(g_pQst->sspdev));
    if (g_pQst) kfree (g_pQst);
    g_pQst = NULL;
    printk(KERN_DEBUG "motion: mma7455l_teardown complete\n");
}

static int __devinit mma7455l_init(void)
{
    return mma7455l_setup();
}

static void __exit mma7455l_exit(void)
{
    mma7455l_teardown ();
}

module_init(mma7455l_init);
module_exit(mma7455l_exit);

MODULE_AUTHOR("Keith Williams <keith.williams@sonos.com>");
MODULE_DESCRIPTION("Woodstock mma7455l accelerometer Driver");
MODULE_LICENSE("GPL");

