/*
 * linux/arch/arm/mach-pxa/lt-charger.c
 *
 * The charger driver for Littleton Development Platform
 * 
 * Copyright (2006) Marvell International Ltd.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mman.h>
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/bitfield.h>
#include <asm/arch/micco.h>
#include <asm/arch/pxa3xx_pmic.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/woodstock_ioctl.h>

#define LDO_ADC_ALWAYS

static int bat_aval = 0;	/* Default no battary available*/
static int dc_present = 0;
static int charger_disabled = 0;
static spinlock_t charger_lock;

static int  charger_release(struct inode *inode, struct file *file);
static int  charger_open(struct inode *inode, struct file *file);
static void set_dc_present(int);
static void start_charging(void);

static void stop_charging(void)
{
	u8 val;

	micco_read(MICCO_CHARGE_CONTROL, &val);
	val &= ~0x80;
	micco_write(MICCO_CHARGE_CONTROL, val);
	micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
	// XXX BT the following constant used to be ~0x2C which was a problem.
	// XXX When stop_charging() is called from set_dc_present(0), it should
	// XXX disable the voltage monitor.
	val &= (~0x2E);
	if (dc_present && (!charger_disabled)) val |= 0x02; // arm the vbat monitor if it could be used to restart charging
	micco_write(MICCO_ADC_AUTO_CONTROL_1, val); // disable charging parameter measurement
#ifndef LDO_ADC_ALWAYS
	micco_read(MICCO_ADC_MAN_CONTROL, &val);
	val &= (~0x10);
	micco_write(MICCO_ADC_MAN_CONTROL, val); // disable the GPADC LDO to save power
#endif
	printk("Charger has been stopped\n");
	return;
}

static void set_dc_present(int dcp)
{
	u8 val;
	if (!bat_aval) return;
	if (dcp!=dc_present) {
		dc_present=dcp;
		if (dcp) {
			printk("charger: DC present\n");
			if (!charger_disabled) start_charging();
		} else {
			printk("charger: DC not present\n");
			stop_charging();
		}
	}
}

static void start_charging(void)
{
	u8 val;
	micco_write(MICCO_CCTR_CONTROL,0x17); // 184 minutes max const. current
	micco_write(MICCO_TCTR_CONTROL,0x04); // 4 hours max. charging
	micco_write(MICCO_TBATHIGHP, 25);
	micco_write(MICCO_TBATHIGHN, 28);
	micco_write(MICCO_TBATLOW, 235);

        /* Enable the TBAT, VCH, ICH and VBAT auto measurements */
        micco_read(MICCO_ADC_MAN_CONTROL, &val);
        val |= 0x10;
        micco_write(MICCO_ADC_MAN_CONTROL, val);

        micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
        val |= 0x2C;
	val &= (~0x02); // don't monitor vbat while running the charger, that's just silly
        micco_write(MICCO_ADC_AUTO_CONTROL_1, val);

	micco_write(MICCO_CHARGE_CONTROL, 0xD0); /* 4.1V 1000mA enable */
	printk("Charger has been started\n");
	return;
}

static int get_adc_channel(int channel)
{
	u8 val;
	int ldo_adc_wasoff=0;
	micco_read(MICCO_ADC_MAN_CONTROL,&val);
	if ((val&0x10)==0) {
		// Need to turn on LDO_ADC for the measurement
		// and will turn it off when done
		ldo_adc_wasoff=1;
		val|=0x10;
		micco_write(MICCO_ADC_MAN_CONTROL,val);
		udelay(100);
	}
	val=(channel&7)|0x18;
	micco_write(MICCO_ADC_MAN_CONTROL,val);
	udelay(100);
	micco_read(MICCO_ADC_MAN_CONTROL,&val);
	while (val&0x08) {
		udelay(100);
		micco_read(MICCO_ADC_MAN_CONTROL,&val);
	}
	micco_read(MICCO_MAN_RES_MSB,&val);
	if (ldo_adc_wasoff) micco_write(MICCO_ADC_MAN_CONTROL,0);
	return val;
}

static int get_vbat(void)
{
	int v;
	v=get_adc_channel(0);
	v*=2650;
	v/=255;
	v+=2650;
	return v;
}

static int get_ichg(void)
{
	int i;
	i=get_adc_channel(1);
	i*=1600;
	i/=255;
	return i;
}

static int get_vchg(void)
{
	int v;
	v=get_adc_channel(2);
	v*=15900;
	v/=255;
	return v;
}

static int get_tbat(void)
{
	u8 val;
	int r;
	micco_read(MICCO_MISC,&val);
	micco_write(MICCO_MISC,val|0x10);
	r=get_adc_channel(3);
	micco_write(MICCO_MISC,val);
	return r;
}

static int get_light(void)
{
	int ll;
	// XXX took out the GPIO control for now, the time constant of the
	// XXX sensing circuit is long so for the moment I'm going to leave
	// XXX it high at all times...
	// raise GPIO 105
	// pxa3xx_gpio_set_level(105,GPIO_LEVEL_HIGH);
	ll=get_adc_channel(5);
	// lower GPIO 105
	// pxa3xx_gpio_set_level(105,GPIO_LEVEL_LOW);
	return ll;
}

static int get_temp(void)
{
	int t;
	// raise GPIO 106
	pxa3xx_gpio_set_level(106,GPIO_LEVEL_HIGH);
	mdelay(10);
	t=get_adc_channel(6);
	// lower GPIO 106
	pxa3xx_gpio_set_level(106,GPIO_LEVEL_LOW);
	return t;
}

static void set_button_backlight(int dc)
{
	u8 val;
	if (dc<0) dc=0;
	if (dc>95) dc=95;
	val=dc;
	micco_write(MICCO_WLED_CONTROL1,val);
	if (dc) {
		micco_write(MICCO_LED2_CONTROL,0x5f);
		micco_write(MICCO_WLED_CONTROL2,0x2a);
	} else {
		micco_write(MICCO_WLED_CONTROL2,0);
		micco_write(MICCO_LED2_CONTROL,0);
	}
}

static int get_button_backlight(void)
{
	u8 val;
	micco_read(MICCO_WLED_CONTROL1,&val);
	return (val&0x7f);
}

static unsigned int get_status(void)
{
	u8 val,val2;
	unsigned int r=0;
	micco_read(MICCO_CHARGE_CONTROL, &val);
	if (val & 0x80)
		r |= DA9034_STATE_CHARGING;
	micco_read(MICCO_STATUS_A, &val);
	micco_read(MICCO_STATUS_B, &val2);
	if ((val & MICCO_STATUS_A_CHDET) || (val2 & MICCO_STATUS_B_USBDEV))
		r |= DA9034_STATE_DC_PRESENT;
	if (val & MICCO_STATUS_A_CHDET)
		r |= DA9034_STATE_HIGHDC_PRESENT;
	if (val & MICCO_STATUS_A_TBAT)
		r |= DA9034_STATE_TBAT_FAULT;
	if (charger_disabled)
		r |= DA9034_STATE_CHARGER_DISABLED;
	return r;
}

static void set_charge_led(int enable, int blanking, unsigned long bitmask)
{
	int byte;
	
	if (0 == enable) {
		
		micco_write(MICCO_LEDPC_CONTROL5,0);
		
	} else {

		for (byte = 0; byte < 4; byte++) {
			micco_write(MICCO_LEDPC_CONTROL1 + byte, bitmask & 0xff);
			bitmask >>= 8;
		}
		
		micco_write(MICCO_LEDPC_CONTROL5, (1 << 3) | (blanking & 7));				
	}	       
}

/*
 * KLUDGE: Hook to allow cytouch driver to flash LED when programming the
 *         touch panel firmware.   Since this is only used by the cytouch
 *         driver, and this driver isn't initialized when the cytouch driver is
 *         loading, there are no locks here.  If this changes, locks/whaetever
 *         can be added in this function.
 */
void lt_charger_set_charge_led(int enable, int blanking, unsigned long bitmask)
{
	set_charge_led(enable, blanking, bitmask);
}

static int charger_read_proc(char *page,char **start,off_t off,int count,int *eof,void *data)
{
	int t;
	unsigned long flags;
	unsigned int stat;
	spin_lock_irqsave(&charger_lock,flags);
	stat=get_status();
	t=sprintf(page,"DA9034 miscellaneous functions\n");
	t+=sprintf(page+t,"Battery voltage %dmV %sTBAT fault\n",get_vbat(),((stat&DA9034_STATE_TBAT_FAULT)?"":"No "));
	t+=sprintf(page+t,"Charger software state is %s\n",((stat&DA9034_STATE_CHARGER_DISABLED)?"disabled":"enabled"));
	t+=sprintf(page+t,"DC %spresent Charger %srunning\n",((stat&DA9034_STATE_DC_PRESENT)?"":"not "),((stat&DA9034_STATE_CHARGING)?"":"not "));
	if (stat&DA9034_STATE_DC_PRESENT) {
		t+=sprintf(page+t,"DC input in %s voltage range\n",((stat&DA9034_STATE_HIGHDC_PRESENT)?"high":"low"));
	}
	if (stat&DA9034_STATE_CHARGING) {
		t+=sprintf(page+t,"Charger current %dmA voltage %dmV\n",get_ichg(),get_vchg());
	}
	t+=sprintf(page+t,"Battery NTC %d Board NTC %d Light level %d\n",get_tbat(),get_temp(),get_light());
	t+=sprintf(page+t,"Button backlight level %d\n",get_button_backlight());
	spin_unlock_irqrestore(&charger_lock,flags);
	return t;
}

extern void pxa3xx_power_off(void);

void lt_charger_suspend(void)
{
    u8 val;
    if (dc_present) return;
    micco_write(MICCO_VBATMON,53);
    micco_read(MICCO_ADC_AUTO_CONTROL_1,&val);
    val|=0x02;
    micco_write(MICCO_ADC_AUTO_CONTROL_1,val);
}

static int dead_check=0;

void lt_charger_resume(void)
{
    u8 val;
    if (dc_present) return;
    dead_check=1;
    micco_read(MICCO_ADC_AUTO_CONTROL_1,&val);
    val&=0xFD;
    micco_write(MICCO_ADC_AUTO_CONTROL_1,val);
    micco_write(MICCO_VBATMON,120);
}

/****************************************************************************
 * Interrupt Handler
 ***************************************************************************/
/* Currently we don't enable the timeout for charging. So Just Event A is
 * enough.
 */
void lt_charger_interrupt(unsigned long event)
{
	u8 val;
	unsigned long flags;

	spin_lock_irqsave(&charger_lock,flags);
	printk("lt_charger_interrupt event %08lx\n",event);

	if (dead_check && (event & PMIC_EVENT_VBATMON)) {
		pxa3xx_power_off();
	}
	dead_check=0;
	if (event & ( PMIC_EVENT_REV_IOVER |
			PMIC_EVENT_IOVER)) {
		/* if Battery over temperature, reverse mode over current or
		 * Charger over current, turn off the charging because the 
		 * over temperature.
		 */
		stop_charging();
	} else if (event & PMIC_EVENT_CHDET) {	/* Charge detection */
		/* TODO: Notify user space appliction: charger is plugged */
		micco_read(MICCO_STATUS_A, &val);
		if (val & MICCO_STATUS_A_CHDET) {
			pr_debug("%s: 6V charger detected, start charging\n",
				__func__);
			set_dc_present(1);
		} else {
			pr_debug("%s: 6V charger removed!\n", __func__);
			set_dc_present(0);
		}
	} else if (event & PMIC_EVENT_VBATMON) { /* Charge voltage too low */
                if (dc_present) start_charging();
	} else if (event & PMIC_EVENT_VBUS) { /* USB cable detected */
		micco_read(MICCO_STATUS_B, &val);
		if (val & MICCO_STATUS_B_USBDEV) {
			pr_debug("%s: 5V charger detected, start charging\n",
				__func__);
			set_dc_present(1);
		} else {
			pr_debug("%s: 5V charger removed!\n", __func__);
			set_dc_present(0);
		}
	} else if (event & (PMIC_EVENT_CH_CCTO|PMIC_EVENT_CH_TCTO)) {
		printk("charging timer expired, disabling charger\n");
		stop_charging();
	}
	spin_unlock_irqrestore(&charger_lock,flags);
		
}

/****************************************************************************
 * File System I/O operations
 ***************************************************************************/

static int charger_open(struct inode *inode, struct file *file)
{
	return 0;
}	

static ssize_t charger_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}

/* We assume that the user buf will not larger than kbuf size */
static ssize_t charger_write(struct file *file, const char __user *buf,
                        size_t count, loff_t *ppos)
{
	return count;
}

static int charger_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int charger_ioctl(struct inode *inode, struct file *file,unsigned int cmd,unsigned long arg)
{
	unsigned long flags;
	unsigned char buf[4];
	int ret=0;
	if (_IOC_SIZE(cmd)>4) return -EIO;
	if (_IOC_DIR(cmd)&_IOC_WRITE) {
		if (copy_from_user(buf,(void *)arg,_IOC_SIZE(cmd))) return -EFAULT;
	}


	spin_lock_irqsave(&charger_lock,flags);
	switch(cmd) {
		case DA9034_MISC_IOCTL_GET_VBAT:
			*((int *)buf)=get_vbat();
			break;
		case DA9034_MISC_IOCTL_GET_ICHG:
			*((int *)buf)=get_ichg();
			break;
		case DA9034_MISC_IOCTL_GET_VCHG:
			*((int *)buf)=get_vchg();
			break;
		case DA9034_MISC_IOCTL_GET_TBAT:
			*((int *)buf)=get_tbat();
			break;
		case DA9034_MISC_IOCTL_GET_LIGHT:
			*((int *)buf)=get_light();
			break;
		case DA9034_MISC_IOCTL_GET_TEMP:
			*((int *)buf)=get_temp();
			break;
		case DA9034_MISC_IOCTL_SET_BUTTON_BACKLIGHT:
			set_button_backlight(arg);
			break;
		case DA9034_MISC_IOCTL_GET_BUTTON_BACKLIGHT:
			*((int *)buf)=get_button_backlight();
			break;
		case DA9034_MISC_IOCTL_GET_STATE:
			*((unsigned int *)buf)=get_status();
			break;
		case DA9034_MISC_IOCTL_SET_CHARGER:
			if (arg==DA9034_CHARGER_ENABLE) {
                            charger_disabled=0;
                            if (dc_present) start_charging();
                        }
                        if (arg==DA9034_CHARGER_DISABLE) {
                            charger_disabled=1;
                            stop_charging();
                        }
                        break;
		case DA9034_MISC_IOCTL_SET_CHARGE_LED:
			if (arg==DA9034_CHARGE_LED_ENABLE) {
				set_charge_led(1,0,0xffffffff);
                        }
                        if (arg==DA9034_CHARGE_LED_DISABLE) {
				set_charge_led(0,0,0);
                        }
                        break;
		case DA9034_MISC_IOCTL_SET_CHARGE_LED_EXTENDED:
			if (arg) set_charge_led(1,0,arg); else set_charge_led(0,0,0);
			break;
		default:
			ret= -EIO;
			goto unlock_out;
	}
unlock_out:
	spin_unlock_irqrestore(&charger_lock,flags);
	if (ret) return ret;

	if (_IOC_DIR(cmd)&_IOC_READ) {
		if (copy_to_user((void *)arg,buf,_IOC_SIZE(cmd))) return -EFAULT;
	}
	return 0;
}

static struct file_operations charger_fops = {
	.owner		= THIS_MODULE,
	.open		= charger_open,
	.release	= charger_release,
	.write		= charger_write,
	.read		= charger_read,
	.ioctl		= charger_ioctl,
};

static struct miscdevice charger_miscdev = {
	.minor		= DA9034_MISC_MINOR,
	.name		= "micco_charger",
	.fops		= &charger_fops,
};

static struct proc_dir_entry *charger_proc;

/****************************************************************************
 * Initialization / Registeration / Removal
 ***************************************************************************/
static int charger_probe(struct platform_device *pdev)
{
	u8 val;
	int ret;
	unsigned long flags;

	ret = misc_register(&charger_miscdev);
	if (ret < 0)
		return ret;

	spin_lock_init(&charger_lock);
	charger_proc=create_proc_read_entry("driver/micco_misc",0444,NULL,charger_read_proc,NULL);
	if (!charger_proc) {
		printk("Failed to create driver/micco_misc proc entry\n");
	}

	/* Charger should handle EVENT_CHARGER and VBUS */
	ret = pmic_callback_register((PMIC_EVENT_CHARGER | PMIC_EVENT_VBUS),
			lt_charger_interrupt);
	if (ret)  {
		misc_deregister(&charger_miscdev);
		return ret;
	}

	pr_info("Charger for Littleton initialized!\n");
	/* Enable interrupt:
	 * the over current on charger reverse,
	 * the VBAT less then VBATMON,
	 * the Battery over temperature,
	 * the Charger detection/removal.
	 */
	spin_lock_irqsave(&charger_lock,flags);
	micco_read(MICCO_IRQ_MASK_A, &val);
	val &= ~0x68; // no need to take TBAT interrupts
	micco_write(MICCO_IRQ_MASK_A, val);
	micco_read(MICCO_IRQ_MASK_B, &val);
	val &= ~0x07; //also enable interrupts for USB_DEV, CCTO, TCTO
	micco_write(MICCO_IRQ_MASK_B, val);

	/* TODO:
	 * Need set the registers value:
	 * TBATHIGHP, TBATHIGHN, TBATLOW.
	 * ICHMAX_RES, ICHMIN_RES, VCHMAX_RES, VCHMIN_RES
	 * --by yfw
	 */

	/* Enable the charging if there is a battery */
	micco_read(MICCO_MISC, &val);
	val |= MICCO_MISC_I_TBAT_ON;
	micco_write(MICCO_MISC, val);
	
	/* Enable ADC and setup input mux */
	micco_write(MICCO_ADC_MAN_CONTROL, 0x13);
 
	/* Enable manual conversion on TBAT voltage */
	micco_write(MICCO_ADC_MAN_CONTROL, 0x1b);
	mdelay(2);

	micco_read(MICCO_ADC_MAN_CONTROL, &val);
	while (val & MICCO_ADC_MAN_CONT_CONV) {
		mdelay(2);
		micco_read(MICCO_ADC_MAN_CONTROL, &val);
	}

	micco_read(MICCO_MAN_RES_MSB, &val);
	if (val < 0xf0) {	/* Bettery available */
		/* TODO: Need adjust the value according to
		 * the charger type (wall power or USB host).
		 * Currently, just support USB host.
		 */
		bat_aval = 1;
		printk("Battery appears to be present\n");
		//micco_write(MICCO_CCTR_CONTROL,0x07); // 56 minutes max const. current
		//micco_write(MICCO_TCTR_CONTROL,0x03); // 3 hours max. charging
		//start_charging();
	}

#ifdef LDO_ADC_ALWAYS
	micco_write(MICCO_ADC_MAN_CONTROL, 0x10);
#else
	micco_write(MICCO_ADC_MAN_CONTROL, 0x00);
#endif

	micco_read(MICCO_MISC, &val);
	val &= ~MICCO_MISC_I_TBAT_ON;
	micco_write(MICCO_MISC, val);
// XXX BT moved the following into start_charging()
#if 0
	/* Enable the TBAT, VCH, ICH and VBAT auto measurements */
	micco_read(MICCO_ADC_MAN_CONTROL, &val);
	val |= 0x10;
	micco_write(MICCO_ADC_MAN_CONTROL, val);

	micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
	val |= 0x2E;
	micco_write(MICCO_ADC_AUTO_CONTROL_1, val);
#else
	micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
	val |= 0x40;  // debounce VBATMON
	micco_write(MICCO_ADC_AUTO_CONTROL_1, val);
#endif
	micco_write(MICCO_VBATMON, 120); // low battery thresh ~3.9V
	if (bat_aval) {
		micco_read(MICCO_STATUS_A, &val);
		if (val & MICCO_STATUS_A_CHDET) {
			set_dc_present(1); //if charger present
		} else {
			micco_read(MICCO_STATUS_B, &val);
			if (val & MICCO_STATUS_B_USBDEV)
				set_dc_present(1);
		}
		// stop_charging(); // XXX for testing the low voltage restart stuff...
	}
	spin_unlock_irqrestore(&charger_lock,flags);
 
	return 0;
}

static int charger_remove(struct platform_device *pdev)
{
	/* Disable TBAT, VCH, ICH and VBAT auto measurements */
	u8 val;

	micco_read(MICCO_ADC_MAN_CONTROL, &val);
	val &= ~0x10;
	micco_write(MICCO_ADC_MAN_CONTROL, val);

	micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
	val &= ~0x2E;
	micco_write(MICCO_ADC_AUTO_CONTROL_1, val);

	/* Mask IRQ */
	micco_read(MICCO_IRQ_MASK_A, &val);
	val |= 0x78;
	micco_write(MICCO_IRQ_MASK_A, val);

	pmic_callback_unregister(PMIC_EVENT_CHARGER, lt_charger_interrupt);
	misc_deregister(&charger_miscdev);
	return 0;
}

static struct platform_driver charger_driver = {
	.driver = {
		.name	= "lt_charger",
	},
	.probe		= charger_probe,
	.remove		= charger_remove,
};

static int __devinit charger_init(void)
{
	return platform_driver_register(&charger_driver);
}

static void __exit charger_exit(void)
{
	platform_driver_unregister(&charger_driver);
}

module_init(charger_init);
module_exit(charger_exit);
MODULE_LICENSE("GPL");
