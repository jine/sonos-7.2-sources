/*
 *  drivers/input/touchscreen/zy_audio_touch.c.
 *
 *  Author:	bridge.wu@marvell.com
 *  Created:	Nov 17, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/mfp.h>
#include <asm/arch/ipmc.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <asm/arch/codec/ac97acodec.h>
#include <asm/arch/codec/wm9713.h>
#include <asm/arch/codec/acodec.h>
#include <asm/arch/pxa3xx_audio_plat.h>
#include <asm/arch/hardware.h>

#define ALSA_ZY_CARD_DEBUG
#undef ALSA_ZY_CARD_DEBUG

#ifdef ALSA_ZY_CARD_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG format, ##arg)
#else
#define dbg(format, arg...)
#endif

#define DEBUG
#undef DEBUG
unsigned int start_time;
unsigned int end_time;
#ifdef DEBUG
unsigned int time;
#define PRINT_TIME() do {\
        time = ((end_time > start_time))?\
                (end_time - start_time)*100/325:\
                (0xffffffff - start_time + end_time)*100/325;\
        printk("\n%s:%dus\n", __FUNCTION__, time);\
} while(0)
#endif

#define PEN_DOWN 		1
#define PEN_UP			0
#define TS_SAMPLE_INTERVAL 	1

static p_acodec_context_t p_zy_codec_ctxt = NULL;

int touch_client;

typedef struct {
	struct input_dev *idev;
	struct timer_list *timer;
	struct timer_list *reset_timer;
	int  use_count;
} codec_zy_ts_t;

codec_zy_ts_t codec_zy_ts;

static struct input_dev *codec_zy_ts_input;

#ifdef CONFIG_PM
static volatile int touch_suspend = 0 ;
#endif

extern acodec_error_t zy_acodec_init(acodec_context_t *codec_context, int hw_init);
extern acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context);

/*
 * add a touch event
 */
static int codec_zy_ts_evt_add(codec_zy_ts_t* ts, u16 pressure, u16 x, u16 y)
{
        /* add event and remove adc src bits */
        static u16 pre_press = 0;

        input_report_abs(ts->idev, ABS_X, x & 0xfff);
        input_report_abs(ts->idev, ABS_Y, y & 0xfff);
        if (pressure == pre_press){
                pressure--;
        }
        pre_press = pressure;
        input_report_abs(ts->idev, ABS_PRESSURE, pressure & 0xfff);
        return 0;
}

/*
 * add a pen up event
 */
static void codec_zy_ts_evt_release(codec_zy_ts_t* ts)
{
	input_report_abs(ts->idev, ABS_PRESSURE, 0);
#ifdef CONFIG_IPM
	ipm_event_notify(IPM_EVENT_UI, IPM_EVENT_DEVICE_TSI, NULL, 0);
#endif
	p_zy_codec_ctxt->event_ack(p_zy_codec_ctxt,EVENT_TYPE_PDN);
}

/*
 * Kill the touchscreen thread and stop
 * the touch digitiser.
 */
static void codec_zy_ts_input_close(struct input_dev *idev)
{
	codec_zy_ts_t *ts = (codec_zy_ts_t *) &codec_zy_ts;

#ifdef CONFIG_PM
	if(touch_suspend){
		pr_info("touch is suspended!\n");
		return;
	}
#endif
	dbg("close ts input!\n");
	if (--ts->use_count == 0) {
		del_timer(ts->timer);
		if (ts->timer != NULL)
			kfree(ts->timer);
		set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
 		p_zy_codec_ctxt->disable_touch(p_zy_codec_ctxt);
		set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	}
}

/*
 * Sample the touchscreen
 */
int ac97_poll_touch(codec_zy_ts_t *ts)
{
	unsigned short x=0, y=0;
	int if_down= 0;
	acodec_error_t status = ACODEC_SUCCESS;

#ifdef DEBUG
        start_time = OSCR;
#endif

	/* get x value */
	status = zy_acodec_get_adc_sample(p_zy_codec_ctxt, &x, ZY_TOUCH_SAMPLE_X, &if_down);
	if (ACODEC_SUCCESS != status ){
		dbg("failed to get x value, %d\n", status);
		return -EIO;
	}
	dbg("x:0x%x\n", x);

	/* the pen is up */
	if (1 != if_down){
		return PEN_UP;
	}

	/* get y vaule */
	status = zy_acodec_get_adc_sample(p_zy_codec_ctxt, &y, ZY_TOUCH_SAMPLE_Y, &if_down);
	if (ACODEC_SUCCESS != status ){
		dbg("failed to get y value, %d\n", status);
		return -EIO;
	}
	dbg("y:0x%x\n",y);

	/* the pen is up */
	if (1 != if_down){
		return PEN_UP;
	}

	/* the pen is down, can not get the pressure value,
	 * so if pen is down, give the max pressure value
	 */
	codec_zy_ts_evt_add(ts,0xfff, x, y);

#ifdef DEBUG
        end_time = OSCR;
        PRINT_TIME();
#endif

	return PEN_DOWN;
}

/*
 * Use new timer to implement delay operation.
 * Invoid to use mdelay() in touch_timer_handler().
 */
static void codec_reset_handler(unsigned long step)
{
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(p_zy_codec_ctxt->p_ctrl_reg);
	codec_zy_ts_t *ts = &codec_zy_ts;

	if (step > 1)
		step = 0;
	switch (step) {
	case 0:
		p_ac97_reg->gcr |= ZY_AC97_GCR_WARM_RESET_MSK;

		ts->reset_timer->expires = jiffies + 1;
		ts->reset_timer->data = ++step;
		add_timer(ts->reset_timer);
		break;
	case 1:
		zy_acodec_init(p_zy_codec_ctxt, 1);
		p_zy_codec_ctxt->enable_touch(p_zy_codec_ctxt);
		zy_ac97_acodec_write(p_zy_codec_ctxt, GPIO_PIN_CFG, 0xFFF6);
		zy_ac97_acodec_write(p_zy_codec_ctxt, GPIO_PIN_SHARING, 0xFFF6);
		zy_ac97_acodec_write(p_zy_codec_ctxt, GPIO_PIN_STICKY, 0x0008);

		ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
		add_timer(ts->timer);
		break;
	}
}

static void touch_timer_handler(unsigned long unused)
{
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(p_zy_codec_ctxt->p_ctrl_reg);
	codec_zy_ts_t *ts = &codec_zy_ts;
	int event;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	event = ac97_poll_touch(ts);

	if (event == PEN_DOWN) {
		dbg("pen down!\n");
		ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
		add_timer(ts->timer);
	} else if(event == PEN_UP) {
		dbg("pen up!\n");
		codec_zy_ts_evt_release(ts);
	} else if(event == -EIO) {
		printk(KERN_ERR "Access touch interface error!\n");

		/* execute warm reset on codec */
		/* set CLK_BPB */
		p_ac97_reg->gcr |= ZY_AC97_GCR_CLKBPB_MSK;
		/* clear CLK_BPB */
		p_ac97_reg->gcr &= ~ZY_AC97_GCR_CLKBPB_MSK;

		ts->reset_timer->expires = jiffies + 1;
		ts->reset_timer->data = 0;
		add_timer(ts->reset_timer);
	}
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	return;
}

/*
 * Start the touchscreen thread and
 * the touch digitiser.
 */
static int codec_zy_ts_input_open(struct input_dev *idev)
{
	codec_zy_ts_t *ts = (codec_zy_ts_t *) &codec_zy_ts;

#ifdef CONFIG_PM
	if(touch_suspend){
		pr_info("touch is suspended!\n");
		return -1;
	}
#endif

	if (ts->use_count++ > 0)
		return 0;

	dbg("Touch is opened. Use count: %d\n", ts->use_count);
	ts->idev = idev;
	ts->timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!ts->timer) {
		printk(KERN_ERR "Alloc memory error for timer!\n");
		return -ENOMEM;
	}

	init_timer(ts->timer);
	ts->timer->function = touch_timer_handler;
	ts->timer->data = 0;

	ts->reset_timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!ts->reset_timer) {
		printk(KERN_ERR "Alloc memroy error for timer!\n");
		return -ENOMEM;
	}

	init_timer(ts->reset_timer);
	ts->reset_timer->function = codec_reset_handler;
	ts->reset_timer->data = 0;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	p_zy_codec_ctxt->enable_touch(p_zy_codec_ctxt);

	/* Modified by Paul Shen */
	input_report_abs(ts->idev, ABS_PRESSURE, 0);
	p_zy_codec_ctxt->event_ack(p_zy_codec_ctxt,EVENT_TYPE_PDN);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	return 0;
}

/*
 * initilze the pxa touch screen
 */
static int alsa_ts_init(struct platform_device *pdev)
{
	int ret = -1;
	codec_zy_ts_t* ts = &codec_zy_ts;

	memset(ts, 0, sizeof(codec_zy_ts_t));

	/* tell input system what we events we accept and register */
	codec_zy_ts_input = input_allocate_device();
	if (codec_zy_ts_input == NULL) {
		printk("%s: failed to allocate input dev\n", __FUNCTION__);
		return -ENOMEM;
	}

	codec_zy_ts_input->name = "codec_ts";
	codec_zy_ts_input->phys = "codec_ts/input1";
	codec_zy_ts_input->cdev.dev = &pdev->dev;

	codec_zy_ts_input->open = codec_zy_ts_input_open;
	codec_zy_ts_input->close = codec_zy_ts_input_close;
	__set_bit(EV_ABS, codec_zy_ts_input->evbit);
	__set_bit(ABS_X, codec_zy_ts_input->absbit);
	__set_bit(ABS_Y, codec_zy_ts_input->absbit);
	__set_bit(ABS_PRESSURE, codec_zy_ts_input->absbit);

	ret = input_register_device(codec_zy_ts_input);
	if (ret) {
		printk("%s: unabled to register input device, ret = %d\n",
				__FUNCTION__, ret);
		return ret;
	}

	return 0;
}

static irqreturn_t pxa_touch_irq(int irq, void *dev)
{
	unsigned char event_type;

	dbg("%s: enter codec event handler\n", __FUNCTION__);
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	p_zy_codec_ctxt->get_event(p_zy_codec_ctxt, &event_type);
	switch (event_type) {
		case EVENT_TYPE_PDN:
		{
			codec_zy_ts_t *ts = &codec_zy_ts;
			/*if the touch is not open need not acknowledge the event*/
			if (ts->use_count <= 0)
				break;
			ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
			add_timer(ts->timer);
			break;
		}
		default:
			printk("unsupported codec event:0x%x\n", event_type);
	}
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	return IRQ_HANDLED;
}

extern int alsa_prepare_for_zy(p_acodec_context_t * p_p_zy_ctxt);
extern acodec_error_t zy_acodec_init(acodec_context_t *codec_context,
					int hw_init);
extern acodec_error_t zy_acodec_deinit(acodec_context_t *codec_context);
static int touch_codec_zy_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_card *card = NULL;
	acodec_error_t status;

	printk("%s:\n", __FUNCTION__);
	/* will increase codec context use count */
	ret = alsa_prepare_for_zy(&p_zy_codec_ctxt);
	if (ret)
		goto err;

	register_codec(&touch_client);
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	/* codec specific initialization, audio will do it either */
	if (1 == p_zy_codec_ctxt->use_count) {
		status = zy_acodec_init(p_zy_codec_ctxt, 1);
		if (ACODEC_SUCCESS != status) {
			printk(KERN_ERR "initialize codec error\n");
			ret = -EIO;
			goto err;
		}
#ifdef CONFIG_MACH_ZYLONITE
	/* power down the units of the acodec, sleep the acodec, zy_acodec_init()
	 * will open all the units' power of the codec while ALSA need all the codec
	 * units power down and the codec should sleep if it can.
	 * So on the zylonite platform we call below function to power down and sleep
	 * wm9713 codec.
	 */
	p_zy_codec_ctxt->codec_specific_dinit(p_zy_codec_ctxt);
#endif
	}

	alsa_ts_init(pdev);

	if (golden_board) {
		zy_ac97_acodec_write(p_zy_codec_ctxt, POWER_DOWN_1, 0x7FFF);
		zy_ac97_acodec_write(p_zy_codec_ctxt, GPIO_PIN_CFG, 0xFFF6);
		zy_ac97_acodec_write(p_zy_codec_ctxt, GPIO_PIN_SHARING, 0xFFF6);
		zy_ac97_acodec_write(p_zy_codec_ctxt, DIGITIZER_3_WM13, 0x8001);
		pxa3xx_mfp_set_afds(MFP_SSP_AUDIO_FRM, MFP_AF0, MFP_DEFAULT_DS);
		pxa3xx_mfp_set_pull(MFP_SSP_AUDIO_FRM, MFP_PULL_LOW);
		pxa3xx_gpio_set_direction(MFP2GPIO(MFP_SSP_AUDIO_FRM), GPIO_DIR_IN);
		pxa3xx_gpio_set_rising_edge_detect(MFP2GPIO(MFP_SSP_AUDIO_FRM), 1);

		set_irq_type(IRQ_GPIO(MFP2GPIO(MFP_SSP_AUDIO_FRM)), IRQT_RISING);
		ret = request_irq(IRQ_GPIO(MFP2GPIO(MFP_SSP_AUDIO_FRM)), pxa_touch_irq,
				SA_INTERRUPT, "wm9713 touch pendown interrupt", NULL);
		if (ret) {
			printk(KERN_ERR "Request IRQ for touch failed (%d).\n", ret);
			goto err;
		}
	} else {
		pxa3xx_mfp_set_afds(MFP_AC97_INT_N_GPIO,0,0);
		pxa3xx_gpio_set_direction(MFP_AC97_INT_N_GPIO, GPIO_DIR_IN);
		pxa3xx_gpio_clear_edge_detect_status(MFP_AC97_INT_N_GPIO);
		set_irq_type(IRQ_GPIO(MFP2GPIO(MFP_AC97_INT_N_GPIO)), IRQT_RISING);
		ret = request_irq(IRQ_GPIO(MFP2GPIO(MFP_AC97_INT_N_GPIO)),
				pxa_touch_irq,
				SA_INTERRUPT, "wm9713 touch event interrupt", NULL);
		if (ret) {
			printk(KERN_ERR "Request IRQ for touch failed (%d).\n", ret);
			goto err;
		}
	}
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	return 0;
err:
	if (p_zy_codec_ctxt && (!--p_zy_codec_ctxt->use_count)) {
		zy_acodec_deinit(p_zy_codec_ctxt);
		pxa_set_cken(CKEN_AC97, 0);
		kfree(p_zy_codec_ctxt);
		p_zy_codec_ctxt = NULL;
	}
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	if (card)
		snd_card_free(card);

	return ret;
}

extern void alsa_zy_codec_put(p_acodec_context_t p_acodectxt);
static int touch_codec_zy_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	input_unregister_device(codec_zy_ts_input);

	if (p_zy_codec_ctxt && (!--p_zy_codec_ctxt->use_count)) {
		alsa_zy_codec_put(p_zy_codec_ctxt);
		kfree(p_zy_codec_ctxt);
		p_zy_codec_ctxt = NULL;
	}

	if (card) {
		snd_card_free(card);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

#ifdef CONFIG_PM
extern int audio_codec_zy_do_suspend(struct snd_card *card, unsigned int state,
					p_acodec_context_t g_acodec_context);
extern int audio_codec_zy_do_resume(struct snd_card *card, unsigned int state,
					p_acodec_context_t g_acodec_context);
static int touch_codec_zy_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret=0;
	
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	ret = audio_codec_zy_do_suspend(NULL, SNDRV_CTL_POWER_D3cold, p_zy_codec_ctxt);
	touch_suspend = 1;
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_OFF);
	return ret;
}

static int touch_codec_zy_resume(struct platform_device *pdev)
{
	int ret = 0;
	
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	ret = audio_codec_zy_do_resume(NULL, SNDRV_CTL_POWER_D0, p_zy_codec_ctxt);
	touch_suspend = 0;
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	return ret;
}
#else
#define touch_codec_zy_suspend	NULL
#define touch_codec_zy_resume	NULL
#endif

static struct platform_driver touch_codec_zy_driver = {
	.driver = {
		.name 	= "pxa2xx-touch",
	},
	.probe		= touch_codec_zy_probe,
	.remove		= touch_codec_zy_remove,
	.suspend	= touch_codec_zy_suspend,
	.resume		= touch_codec_zy_resume,
};


#include <linux/fs.h>
#include <linux/proc_fs.h>

static unsigned int ac97_read(unsigned int reg)
{
	unsigned int value=0;

	zy_ac97_acodec_read(p_zy_codec_ctxt, reg, &value);
	return value;
}

static void ac97_write(unsigned int reg, unsigned int val)
{
	zy_ac97_acodec_write(p_zy_codec_ctxt, reg, &val);
}

static ssize_t wm9713_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			i;
	int 			reg;
	unsigned int sscr0, sscr1, sspsp, sstsa;
	unsigned int ssacd, ssacdd, ssrsa;

	t = scnprintf(next, size, "AC97 regs: GCR: 0x%x, GSR: 0x%x\n", 
			GCR, GSR);
	size -= t;
	next += t;

	t = scnprintf(next, size, "wm9713 regs: \n");
	size -= t;
	next += t;

	for(i=0; i<0x80; i+=8) {
		reg = ac97_read(i);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  ", i, reg);
		size -= t;
		next += t;
		reg = ac97_read(i+2);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  ", i+2, reg);
		size -= t;
		next += t;
		reg = ac97_read(i+4);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  ", i+4, reg);
		size -= t;
		next += t;
		reg = ac97_read(i+6);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  \n", i+6, reg);
		size -= t;
		next += t;
	}

	sscr0 = SSCR0_P(3);
	sscr1 = SSCR1_P(3);
	sspsp = SSPSP_P(3);
	sstsa = SSTSA_P(3);
	ssrsa = SSRSA_P(3);
	ssacd = SSACD_P(3);
	ssacdd= SSACDD_P(3);

	t = scnprintf(next, size, "SSP3 regs: \n");
	size -= t;
	next += t;
	t = scnprintf(next, size, "sscr0:0x%x \n", sscr0);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sscr1:0x%x \n", sscr1);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sspsp:0x%x \n", sspsp);
	size -= t;
	next += t;
	t = scnprintf(next, size, "sstsa:0x%x \n", sstsa);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssrsa:0x%x \n", ssrsa);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssacd:0x%x \n", ssacd);
	size -= t;
	next += t;
	t = scnprintf(next, size, "ssacdd:0x%x \n", ssacdd);
	size -= t;
	next += t;	

	*eof = 1;
	return count - size;
}
	
static int wm9713_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[4096];
	char *buf = kbuf;
	unsigned int	i, j, k, reg, reg2;
	char cmd;

	

	if (count >=4096)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%c 0x%x 0x%x", &cmd, &i, &reg);
	if('r' == cmd) {
		if(i>0x7e || (i&1)) {
			printk(KERN_INFO "invalid index!\n");
			goto error;
		}
		reg = ac97_read(i);
		printk(KERN_INFO "0x[%2x]=0x%4x\n", i, reg);
	}else if('w' == cmd) {
		if(i>0x7e || (i&1)) {
			printk(KERN_INFO "invalid index!\n");
			goto error;
		}
		if(reg>0xffff) {
			printk(KERN_INFO "invalid value!\n");
			goto error;
		}
		ac97_write(i, reg);
		reg2 = ac97_read(i);
		printk(KERN_INFO 
			"write 0x%4x to 0x[%2x], read back 0x%4x\n", 
			reg, i, reg2);
	}else if('W' == cmd) {
		__REG(i) = reg;
		printk(KERN_INFO 
			"Write 0x%x to 0x%x, read back 0x%x\n", 
			reg, i, __REG(i));
	} else if('g' == cmd) {
		printk(KERN_INFO "GPIO registers (base = 0x40E00000)\n");
		reg = 0x40E00000;
		while(reg <= 0x40E004AC) {
			printk(KERN_INFO "reg[0x%x] = 0x%x\n", reg, __REG(reg));
			reg += 4;
		}
	} else if('i' == cmd) {
		printk(KERN_INFO "Interrupt registers (base = 0x40D00000)\n");
		reg = 0x40D00000;
		while(reg < 0x40D00108) {
			printk(KERN_INFO "reg[0x%x] = 0x%x\n", reg, __REG(reg));
			reg += 4;
		}
	}else {
		printk(KERN_INFO "unknow opt!\n");
		goto error;
	}

	return count;
error:
	printk(KERN_INFO "r/w index(0x%%2x) value(0x%%4x)\n");
	return count;	
}

static int __init ac97codec_proc_init(void)
{
	struct proc_dir_entry *wm9713_proc_entry;	
	wm9713_proc_entry = create_proc_entry("driver/codec", 0, NULL);
	if (wm9713_proc_entry) { 
		wm9713_proc_entry->read_proc = wm9713_proc_read; 
		wm9713_proc_entry->write_proc = wm9713_proc_write; 	
	}

	return 0;
}  

static int __init touch_codec_zy_init(void)
{
	ac97codec_proc_init();
	return platform_driver_register(&touch_codec_zy_driver);
}

static void __exit touch_code_zy_exit(void)
{
	platform_driver_unregister(&touch_codec_zy_driver);
}

module_init(touch_codec_zy_init);
module_exit(touch_code_zy_exit);

EXPORT_SYMBOL(p_zy_codec_ctxt);

MODULE_AUTHOR("bridge.wu@marvell.com");
MODULE_DESCRIPTION("zylonite audio touch codec driver on ALSA");
MODULE_LICENSE("GPL");

