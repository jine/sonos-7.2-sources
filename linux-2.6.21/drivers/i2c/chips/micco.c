/*
 * Monahans Micco PMIC Management Routines
 *
 *
 * Copyright (C) 2006, Marvell Corporation(fengwei.yin@marvell.com).
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
#include <linux/freezer.h>

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_pmic.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/micco.h>

#define CONFIG_MICCO_IRQ_HOUSEKEEPING

#define	DEBUG

#ifdef CONFIG_MACH_WOODSTOCK
#define	IRQ_MICCO		IRQ_GPIO(102)
#else
#define	IRQ_MICCO		IRQ_GPIO(18)
#endif
#define MICCO_REG_NUM		(0xC0)

extern int get_pm_state(void);
static struct pxa3xx_pmic_regs micco_regs[MICCO_REG_NUM];
static int save_event=0;
unsigned int micco_event=0;

/* Make sure that Power I2C has been initialized before invoke this function */	
extern int pxa_i2c_set_speed(int speed);
extern int pxa_i2c_get_speed(void);
int micco_write(u8 reg, u8 val);
int micco_read(u8 reg, u8 *pval);
int micco_codec_disable_output(int type);

static int micco_initchip(void)
{
	int i;

	memset(&micco_regs, 0, (sizeof(struct pxa3xx_pmic_regs) * MICCO_REG_NUM));
	/* TODO: Mask all micco registers uncacheable now.
	 * We can do some optimization here later.
	 */
	for (i = 0; i < MICCO_REG_NUM; i++)
		micco_regs[i].mask = 1;

	return micco_write(MICCO_SYSCTRL_A, 0xE8);	
}

/* Micco TSI functions */
int micco_enable_pen_down_irq(int enable)
{
	int ret;
	u8 val;

	if (enable) {
		/* enable pen down IRQ */
		ret = micco_read(MICCO_IRQ_MASK_C, &val);
		val &= ~0x10;
		ret = micco_write(MICCO_IRQ_MASK_C, val);
	} else {
		/* disable pen down IRQ */
		ret = micco_read(MICCO_IRQ_MASK_C, &val);
		if (!(val & IRQ_MASK_C_PEN_DOWN)) {
			val |= 0x10;
			ret = micco_write(MICCO_IRQ_MASK_C, val);
		}
	}
	return ret;
}

int micco_tsi_poweron(void)
{
	int status;
	u8 val;

	val = 0x10;
	status = micco_write(MICCO_ADC_MAN_CONTROL, val);

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (!(val & MICCO_ADC_AUTO_2_PENDET_EN)) {
		val |= MICCO_ADC_AUTO_2_PENDET_EN;
		status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);
	}

	/* TSI_DEABY: 3 slot. TSI_SKIP: 3 slot */
	val = 0x33;
	status = micco_write(MICCO_TSI_CONTROL_1, val);

	val = 0x00;
	status = micco_write(MICCO_TSI_CONTROL_2, val);
	if (status)
		return -EIO;

	return 0;
}

int micco_tsi_poweroff(void)
{
	int status;
	u8 val;

	status =micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (status)
		return -EIO;

	val &= ~(MICCO_ADC_AUTO_2_TSI_EN | MICCO_ADC_AUTO_2_PENDET_EN);
	status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);

	if (status)
		return -EIO;

	return 0;
}

int micco_tsi_readxy(u16 *x, u16 *y, int pen_state)
{
	int status;
	u8 val;
	u16 mx, my, lxy;

	status = micco_read(MICCO_TSI_X_MSB, &val);
	if (status)
		return -EIO;
	mx = val;

	status = micco_read(MICCO_TSI_Y_MSB, &val);
	if (status)
		return -EIO;
	my = val;

	status = micco_read(MICCO_TSI_XY_MSB, &val);
	if (status)
		return -EIO;
	lxy = val;

	*x = ((mx << 2) & 0x3fc) + (lxy & 0x03);
	*y = ((my << 2) & 0x3fc) + ((lxy & 0x0c) >> 2);

	return 0;
}		

int micco_tsi_enable_pen(int pen_en)
{
	int status;
	u8 val;

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (status)
		return -EIO;

	if (pen_en)
		val |= MICCO_ADC_AUTO_2_PENDET_EN;
	else
		val &= ~MICCO_ADC_AUTO_2_PENDET_EN;

	status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);
	if (status)
		return -EIO;

	return 0;
}

int micco_tsi_enable_tsi(int tsi_en)
{
	int status;
	u8 val;

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (status)
		return -EIO;

	if (tsi_en)
		val |= MICCO_ADC_AUTO_2_TSI_EN;
	else
		val &= ~MICCO_ADC_AUTO_2_TSI_EN;

	status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);
	if (status)
		return -EIO;

	return 0;
}
/* Micco TSI functions end */

/* Micco Audio functions */
static volatile u8 micco_audio_regs[MICCO_AUDIO_REGS_NUM];

int micco_codec_read(u8 reg, u8 *val) 
{
	return micco_read(MICCO_AUDIO_REG_BASE + reg, val);
}

int micco_codec_write(u8 reg, u8 val)
{
	return micco_write(MICCO_AUDIO_REG_BASE + reg, val);
}

void micco_read_codec(void)
{
	int i;
	u8 val;

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++) {
		micco_codec_read(i, &val);
		micco_audio_regs[i] = val;
	}
}

void micco_dump_codec(void)
{
	int i;

	micco_read_codec();

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++) {
		printk(KERN_ALERT "%s: Micco_audio_reg[%d] = 0x%x\n",
			__func__, i, micco_audio_regs[i]);
	}
}
EXPORT_SYMBOL(micco_dump_codec);

int micco_audio_init(void)
{
	int i;

	/* The default setting for Micco */
	micco_audio_regs[MICCO_MUX_MONO] = 0x00; /* MONO */
	micco_audio_regs[MICCO_MUX_BEAR] = 0x00; /* BEAR */
	micco_audio_regs[MICCO_MUX_LINE_OUT] = 0x00; /* LINE OUT */
	micco_audio_regs[MICCO_MUX_STEREO_CH1] =
		0x00;	/* STEREO_CH1 */
	micco_audio_regs[MICCO_MUX_STEREO_CH2] = 
		0x00;	/* STEREO_CH2 */

	micco_audio_regs[MICCO_AUDIO_LINE_AMP] =
		MICCO_AUDIO_LINE_AMP_EN;	/* Enable the Line AMP */

	/* Gain for both channel controlled separately. Enable Setero */
	micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] =
		MICCO_STEREO_GAIN_SEPARATE | MICCO_STEREO_EN;

	/* Soft startup of the Stereo amplifiers */
	micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH2] = 0x0;
	
	micco_audio_regs[MICCO_HIFI_DAC_CONTROL] = 
		MICCO_HIFI_DAC_ON;

	micco_audio_regs[MICCO_MONO_VOL] = 
		MICCO_MONO_EN | 0xa;
	micco_audio_regs[MICCO_BEAR_VOL] =
		MICCO_BEAR_EN | 0xa;

	/* Micco as I2S slave. Use I2S MSB normal mode */
	micco_audio_regs[MICCO_I2S_CONTROL] =
		MICCO_I2S_MSB_JU_MODE;

	micco_audio_regs[MICCO_TX_PGA] =
		0x0c;	/* 0 dB */
	micco_audio_regs[MICCO_MIC_PGA] =
		MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		MICCO_MIC_PGA_SELMIC_2 | MICCO_MIC_PGA_AMP_EN |
		0x7;	/* 30 dB*/	

	micco_audio_regs[MICCO_TX_PGA_MUX] =
		0xFF;

	micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] =
		MICCO_VCODEC_ADC_ON_EN | 0x08;
	/* PCM_SDI normal operation, PCM_SDO enabled */
	micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] =
		MICCO_VDAC_ON | MICCO_VDAC_HPF_BYPASS;

	micco_audio_regs[MICCO_SIDETONE] =
		MICCO_SIDETONE_EN | MICCO_SIDETONE_GAIN_STEREO | 0x08;

	/* Enable AUX1,2. AUX1, 2 gain 0dB */
	micco_audio_regs[MICCO_PGA_AUX1_2] =
		MICCO_PGA_AUX1_EN | MICCO_PGA_AUX2_EN;
	
	/* Enable AUX3. AUX3 gain 0 dB */	
	micco_audio_regs[MICCO_PGA_AUX3] =
		MICCO_PGA_AUX3_EN;

	/* DAC1, 2, 3 gain 0dB */
	micco_audio_regs[MICCO_PGA_DACS] = 
		0x00;

	/*Soft start for MONO, BEAR LINE and STEREO is 61.5ms */
	micco_audio_regs[MICCO_SOFT_START_RAMP] =
		0x00;

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++)
		micco_codec_write(i, micco_audio_regs[i]);

	return 0;
}
EXPORT_SYMBOL(micco_audio_init);

/* FIXME: The Stereo have left and right channel. Need add it later */
int micco_codec_enable_output(int type)
{
	switch (type) {
	case CODEC_BEAR:
		if (!(micco_audio_regs[MICCO_BEAR_VOL] & MICCO_BEAR_EN)) {
			micco_audio_regs[MICCO_BEAR_VOL] |=  MICCO_BEAR_EN;
			micco_codec_write(MICCO_BEAR_VOL,
				 micco_audio_regs[MICCO_BEAR_VOL]);
		}
		break;	

	case CODEC_MONO:
		if (!(micco_audio_regs[MICCO_MONO_VOL] & MICCO_MONO_EN)) {
			micco_audio_regs[MICCO_MONO_VOL] |= MICCO_MONO_EN;
			micco_codec_write(MICCO_MONO_VOL,
				micco_audio_regs[MICCO_MONO_VOL]);
		}
		break;

	case CODEC_STEREO:
		if (!(micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] &
				MICCO_STEREO_EN)){
			micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] |= 
				MICCO_STEREO_EN;
			micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1,
				micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]);
		}
		break;

	case CODEC_LINE_OUT:
		if (!(micco_audio_regs[MICCO_AUDIO_LINE_AMP] & 
				MICCO_AUDIO_LINE_AMP_EN)) {
			micco_audio_regs[MICCO_AUDIO_LINE_AMP] |= 
				MICCO_AUDIO_LINE_AMP_EN;
			micco_codec_write(MICCO_AUDIO_LINE_AMP,
				micco_audio_regs[MICCO_AUDIO_LINE_AMP]);
		}
		break;

	case CODEC_ADC:
		if (!(micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] & 
				MICCO_VCODEC_ADC_ON_EN)) {
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] |= 
				MICCO_VCODEC_ADC_ON_EN;
			micco_codec_write(MICCO_VCODEC_ADC_CONTROL,
				micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_enable_output);

int micco_codec_disable_output(int type)
{
	switch (type) {
	case CODEC_BEAR:
		if (micco_audio_regs[MICCO_BEAR_VOL] & MICCO_BEAR_EN) {
			micco_audio_regs[MICCO_BEAR_VOL] &=  ~MICCO_BEAR_EN;
			micco_codec_write(MICCO_BEAR_VOL,
				 micco_audio_regs[MICCO_BEAR_VOL]);
		}
		break;	

	case CODEC_MONO:
		if (micco_audio_regs[MICCO_MONO_VOL] & MICCO_MONO_EN) {
			micco_audio_regs[MICCO_MONO_VOL] &= ~MICCO_MONO_EN;
			micco_codec_write(MICCO_MONO_VOL,
				micco_audio_regs[MICCO_MONO_VOL]);
		}
		break;

	case CODEC_STEREO:
		if (micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] &
				 MICCO_STEREO_EN){
			micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] &= 
				~MICCO_STEREO_EN;
			micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1,
				micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]);
		}
		break;

	case CODEC_LINE_OUT:
		if (micco_audio_regs[MICCO_AUDIO_LINE_AMP] & 
				MICCO_AUDIO_LINE_AMP_EN) {
			micco_audio_regs[MICCO_AUDIO_LINE_AMP] &= 
				~MICCO_AUDIO_LINE_AMP_EN;
			micco_codec_write(MICCO_AUDIO_LINE_AMP,
				micco_audio_regs[MICCO_AUDIO_LINE_AMP]);
		}
		break;

	case CODEC_ADC:
		if (micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] & 
				MICCO_VCODEC_ADC_ON_EN) {
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] &= 
				~MICCO_VCODEC_ADC_ON_EN;
			micco_codec_write(MICCO_VCODEC_ADC_CONTROL,
				micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* We don't check the paramater. The caller need make sure
 * that the vol is in legal range.
 */
int micco_codec_set_output_vol(int type, int vol)
{
	switch (type) {
	case CODEC_BEAR:
		micco_audio_regs[MICCO_BEAR_VOL] = (~0x3F & 
			micco_audio_regs[MICCO_BEAR_VOL]) | vol;
		micco_codec_write(MICCO_BEAR_VOL,
			micco_audio_regs[MICCO_BEAR_VOL]);
		break;

	case CODEC_MONO:
		micco_audio_regs[MICCO_MONO_VOL] = (~0x3F &
			micco_audio_regs[MICCO_MONO_VOL]) | vol;
		micco_codec_write(MICCO_MONO_VOL,
			micco_audio_regs[MICCO_MONO_VOL]);

	case CODEC_STEREO:
		micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] =
			(~0x3F & micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]) |
			vol;
		micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1, 
			micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]);
		break;

	case CODEC_LINE_OUT:
		micco_audio_regs[MICCO_AUDIO_LINE_AMP] =
			(~0x0F & micco_audio_regs[MICCO_AUDIO_LINE_AMP]) |
			vol;
		micco_codec_write(MICCO_AUDIO_LINE_AMP, 
			micco_audio_regs[MICCO_AUDIO_LINE_AMP]);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}	
EXPORT_SYMBOL(micco_codec_set_output_vol);


int micco_codec_enable_input(int type)
{
	switch (type) {
	case CODEC_AUX1:
		if (!(micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX1_EN)) {
			micco_audio_regs[MICCO_PGA_AUX1_2] |= MICCO_PGA_AUX1_EN;
			micco_codec_write(MICCO_PGA_AUX1_2, 
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX2:
		if (!(micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX2_EN)) {
			micco_audio_regs[MICCO_PGA_AUX1_2] |= MICCO_PGA_AUX2_EN;
			micco_codec_write(MICCO_PGA_AUX1_2, 
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX3:
		if (!(micco_audio_regs[MICCO_PGA_AUX3] & MICCO_PGA_AUX3_EN)) {
			micco_audio_regs[MICCO_PGA_AUX3] |= MICCO_PGA_AUX3_EN;
			micco_codec_write(MICCO_PGA_AUX3, 
				micco_audio_regs[MICCO_PGA_AUX3]);
		}
		break;

	case CODEC_MIC1:
		micco_audio_regs[MICCO_MIC_PGA] &= ~MICCO_MIC_PGA_SELMIC_2;
		micco_audio_regs[MICCO_MIC_PGA] =
		       micco_audio_regs[MICCO_MIC_PGA] |
		       MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		       MICCO_MIC_PGA_AMP_EN;
		micco_codec_write(MICCO_MIC_PGA, 
			micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_MIC2:
		micco_audio_regs[MICCO_MIC_PGA] |= MICCO_MIC_PGA_SELMIC_2;
		micco_audio_regs[MICCO_MIC_PGA] =
		       micco_audio_regs[MICCO_MIC_PGA] |
		       MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		       MICCO_MIC_PGA_AMP_EN;
		micco_codec_write(MICCO_MIC_PGA, 
			micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_PCM:
		if (!(micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &
				MICCO_VDAC_ON)) {
			micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] |=
				MICCO_VDAC_ON;
			micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &=
				~MICCO_VDAC_HPF_MUTE;
			micco_codec_write(MICCO_VCODEC_VDAC_CONTROL, 
				micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL]);
		}
		break;
	
	case CODEC_HIFI:
		if (!(micco_audio_regs[MICCO_HIFI_DAC_CONTROL] &
				MICCO_HIFI_DAC_ON)) {
			micco_audio_regs[MICCO_HIFI_DAC_CONTROL] |=
				MICCO_HIFI_DAC_ON;
			micco_codec_write(MICCO_HIFI_DAC_CONTROL,
				micco_audio_regs[MICCO_HIFI_DAC_CONTROL]);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_enable_input);

int micco_codec_disable_input(int type)
{
	switch (type) {
	case CODEC_AUX1:
		if (micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX1_EN) {
			micco_audio_regs[MICCO_PGA_AUX1_2] &=
				~MICCO_PGA_AUX1_EN;
			micco_codec_write(MICCO_PGA_AUX1_2, 
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX2:
		if (micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX2_EN) {
			micco_audio_regs[MICCO_PGA_AUX1_2] &=
				~MICCO_PGA_AUX2_EN;
			micco_codec_write(MICCO_PGA_AUX1_2, 
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX3:
		if (micco_audio_regs[MICCO_PGA_AUX3] & MICCO_PGA_AUX3_EN) {
			micco_audio_regs[MICCO_PGA_AUX3] &= ~MICCO_PGA_AUX3_EN;
			micco_codec_write(MICCO_PGA_AUX3, 
				micco_audio_regs[MICCO_PGA_AUX3]);
		}
		break;

	case CODEC_MIC1:
		micco_audio_regs[MICCO_MIC_PGA] =
			micco_audio_regs[MICCO_MIC_PGA] & 
			~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
				MICCO_MIC_PGA_AMP_EN);
		micco_codec_write(MICCO_MIC_PGA, 
				micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_MIC2:
		micco_audio_regs[MICCO_MIC_PGA] =
			micco_audio_regs[MICCO_MIC_PGA] & 
			~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
				MICCO_MIC_PGA_AMP_EN);
		micco_codec_write(MICCO_MIC_PGA, 
				micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_PCM:
		if (micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &
				MICCO_VDAC_ON) {
			micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &=
				~MICCO_VDAC_ON;
			micco_codec_write(MICCO_VCODEC_VDAC_CONTROL, 
				micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL]);
		}
		break;
	
	case CODEC_HIFI:
		if (micco_audio_regs[MICCO_HIFI_DAC_CONTROL] &
				MICCO_HIFI_DAC_ON) {
			micco_audio_regs[MICCO_HIFI_DAC_CONTROL] &=
				~MICCO_HIFI_DAC_ON;
			micco_codec_write(MICCO_HIFI_DAC_CONTROL,
				micco_audio_regs[MICCO_HIFI_DAC_CONTROL]);
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/* We don't check the paramater. The caller need make sure
 * that the vol is in legal range.
 */
int micco_codec_set_input_gain(int type, int gain)
{
	switch (type) {
	case CODEC_AUX1:
		micco_audio_regs[MICCO_PGA_AUX1_2] = ((~0x03 & 
			micco_audio_regs[MICCO_PGA_AUX1_2]) | gain);
		micco_codec_write(MICCO_PGA_AUX1_2, 
			micco_audio_regs[MICCO_PGA_AUX1_2]);
		break;

	case CODEC_AUX2:
		micco_audio_regs[MICCO_PGA_AUX1_2] = ((~0x30 & 
			micco_audio_regs[MICCO_PGA_AUX1_2]) | (gain << 4));
		micco_codec_write(MICCO_PGA_AUX1_2, 
			micco_audio_regs[MICCO_PGA_AUX1_2]);
		break;
	
	case CODEC_AUX3:
		micco_audio_regs[MICCO_PGA_AUX3] = ((~0x03 & 
			micco_audio_regs[MICCO_PGA_AUX3]) | gain);
		micco_codec_write(MICCO_PGA_AUX3, 
			micco_audio_regs[MICCO_PGA_AUX3]);
		break;

	case CODEC_MIC1:
	case CODEC_MIC2:
		micco_audio_regs[MICCO_MIC_PGA] = ((~0x07 &
			micco_audio_regs[MICCO_MIC_PGA]) | gain);
		micco_codec_write(MICCO_MIC_PGA,
			micco_audio_regs[MICCO_MIC_PGA]);
		break;
	
	case CODEC_PCM:	/* Need check whether HIFI and PCM support input? */
	case CODEC_HIFI:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int micco_codec_set_sample_rate(int port, int rate)
{
	switch (port) {
	case MICCO_VOICE_PORT:
		switch (rate) {
		case 8000:
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] = 
				(~(0x03 << 3) & 
				 micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
			break;

		case 16000:
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] =
				(~(0x03 << 3) &
				 micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]) |
				(0x01 << 3);
			break;

		case 32000:
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] |=(0x3 << 3);
			break;
		default:
			return -EINVAL;
		}
		micco_codec_write(MICCO_VCODEC_ADC_CONTROL,
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
		break;

	case MICCO_HIFI_PORT:
		switch (rate) {
		case 8000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			break;
		case 11025:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x01;
			break;
		case 12000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x02;
			break;
		case 16000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x03;
			break;
		case 22050:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x04;
			break;
		case 24000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x05;
			break;
		case 32000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x06;
			break;
		case 44100:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x07;
			break;
		case 48000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x0F;
			break;
		default:
			return -EINVAL;
		}
		micco_codec_write(MICCO_I2S_CONTROL,
			micco_audio_regs[MICCO_I2S_CONTROL]);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_set_sample_rate);

int micco_sidetone_enable(void)
{
	if (!(micco_audio_regs[MICCO_SIDETONE] & MICCO_SIDETONE_EN)) {
		micco_audio_regs[MICCO_SIDETONE] |= MICCO_SIDETONE_EN;
		micco_codec_write(MICCO_SIDETONE,
			micco_audio_regs[MICCO_SIDETONE]);
	}

	return 0;
}

int micco_sidetone_disable(void)
{
	if (micco_audio_regs[MICCO_SIDETONE] & MICCO_SIDETONE_EN) {
		micco_audio_regs[MICCO_SIDETONE] &= ~MICCO_SIDETONE_EN;
		micco_codec_write(MICCO_SIDETONE,
			micco_audio_regs[MICCO_SIDETONE]);
	}

	return 0;
}

/* The Micco route rule:
 * 1. DAC3 can't be route to TX_PGA.
 * 2. MIC1 and MIC2 can't be enabled at the same time.
 */
int micco_check_route(u16 *routemap)
{
/* TODO */
	return 0;
}
 
int micco_set_route(u16 *rmap, u16 * current_rmap)
{
	int ret;

	ret = micco_check_route(current_rmap);
	if (ret)
		return ret;

	return 0;
}
/* Micco Audio primitive end here */



/* Initialization functions for system */
/* LDO12 enabled for IO */
void micco_enable_LDO12(void)
{
	u8 val;

	micco_read(MICCO_LDO1312, &val);
	val = (0x0F & val);
	micco_write(MICCO_LDO1312, val);

	return;
}

/* Wrap functions for pmic read/write */
int pxa3xx_pmic_read(u8 reg, u8 *pval)
{
	return micco_read(reg, pval);
}
EXPORT_SYMBOL(pxa3xx_pmic_read);

int pxa3xx_pmic_write(u8 reg, u8 val)
{
	return micco_write(reg, val);
}
EXPORT_SYMBOL(pxa3xx_pmic_write);

/* USB Device/OTG functions */
static int micco_set_pump(int enable)
{
	int status;
	u8 val;
	unsigned long flags;

	local_irq_save(flags);
	if (enable) {
		status = micco_read(MICCO_MISC, &val);
		if (status)
			goto out;
		val |= MICCO_MISC_VBUS_COMPS_EN;
		status = micco_write(MICCO_MISC, val);
		if (status)
			goto out;

		/* FIXME: Littleton didn't connect EXTON as cable detection
		 * signal. We use USB_DEV detection in event B as cable
		 * detection.
		 */
		/* TODO: This is a platform related code. Need split
		 * to platform related code.
		 */
		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val &= ~IRQ_MASK_B_USB_DEV;
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;
	} else {
		status = micco_read(MICCO_MISC, &val);
		if (status)
			goto out;
		val &= ~MICCO_MISC_VBUS_COMPS_EN;
		status = micco_write(MICCO_MISC, val);
		if (status)
			goto out;

		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val |= IRQ_MASK_B_USB_DEV;
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;
	}
out:
	local_irq_restore(flags);
	return status;
}

/* 1. enable: 1, srp: 0, 100ma mode
 * 2. enable: 1, srp: 1, 10ma mode
 *
 * Micco before e: 1:s: 1, must e:0, s:1. 
 */
static int micco_set_vbus_supply(int enable, int srp)
{
	int ret;
	u8 val, tmp;

	ret = micco_read(MICCO_MISC, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	if (enable) {
		/* When enable the USB pump, need check the OTGCP_IOVER. */
		micco_read(MICCO_IRQ_MASK_B, &tmp);
		tmp &= ~IRQ_MASK_B_OTGCP_IOVER;
		micco_write(MICCO_IRQ_MASK_B, tmp);

		val |= MICCO_MISC_USBCP_EN;
		if (srp) {
			val |= MICCO_MISC_USBSR_EN;
		} else {
			val &= ~MICCO_MISC_USBSR_EN;
		}
	} else {
		/* When disable the USB pump, needn't check the OTGCP_IOVER. */
		micco_read(MICCO_IRQ_MASK_B, &tmp);
		tmp |= IRQ_MASK_B_OTGCP_IOVER;
		micco_write(MICCO_IRQ_MASK_B, tmp);

		val &= ~(MICCO_MISC_USBCP_EN | MICCO_MISC_USBSR_EN );
	}
	ret = micco_write(MICCO_MISC, val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
	}
	return ret;
}

static int micco_set_usbotg_a_mask(void)
{
	int ret;
	u8 val;

	ret = micco_read(MICCO_IRQ_MASK_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* Enable the interrupt for VUBS > 4.4V and Session valid 
	 * which A device care about.
	 *
	 * NOTESSSSSSSSS: We care about the SRP detection signal (0.8v ~ 2.0v)
	 * on Micco. Which map to SESSION_VALID_1_8.
	 */
	val |= (IRQ_MASK_B_VBUS_VALID_3_8 | IRQ_MASK_B_SRP_READY_0_6);
	val &= ~(IRQ_MASK_B_VBUS_VALID_4_55 | IRQ_MASK_B_SESSION_VALID_1_8);

	ret = micco_write(MICCO_IRQ_MASK_B, val);
	return ret;
}

static int micco_set_usbotg_b_mask(void)
{
	int ret;
	u8 val;

	ret = micco_read(MICCO_IRQ_MASK_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* Mask all USB VBUS interrupt for B device */
	val |= (IRQ_MASK_B_VBUS_VALID_3_8 | IRQ_MASK_B_SESSION_VALID_1_8 | \
		IRQ_MASK_B_VBUS_VALID_4_55 | IRQ_MASK_B_SRP_READY_0_6);

	ret = micco_write(MICCO_IRQ_MASK_B, val);

	return ret;
}

static int is_micco_vbus_assert(void)
{
	int ret;
	u8 val;

	ret = micco_read(MICCO_STATUS_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	if (val & MICCO_STATUS_B_USBDEV)
		return 1;
	else
		return 0;
}

#ifdef	CONFIG_LITTLETON_CHARGER
#define	CHARGER_HOOK
#else
#undef	CHARGER_HOOK
#endif

static unsigned int micco_event_change(void)
{
	unsigned int ret = 0;
	u8 val, mask;

	// XXX BT DA9034 manual says to read all of the EVENT registers
	// XXX together in page mode.  This isn't that, so as a result
	// XXX we might have a problem with missing interrupts if they
	// XXX arrive at exactly the wrong time.  Also...  even if that
	// XXX is fixed, I wonder whether the DA9034 will clear the
	// XXX interrupt line and then reassert it if an interrupt arrived
	// XXX while reading the EVENT registers or whether the line will
	// XXX just stay asserted.  If the latter, this too is a problem
	// XXX because of how the PXA3xx GPIO interrupt logic can only
	// XXX sense edges, not levels...
	micco_read(MICCO_EVENT_A, &val);
	if (val & MICCO_EA_CHDET)
		ret |= PMIC_EVENT_CHDET;

	if (val & MICCO_EA_REV_IOVER)
		ret |= PMIC_EVENT_REV_IOVER;

	if (val & MICCO_EA_IOVER)
		ret |= PMIC_EVENT_IOVER;

	if (val & MICCO_EA_TBAT)
		ret |= PMIC_EVENT_TBAT;

	if (val & MICCO_EA_VBATMON)
		ret |= PMIC_EVENT_VBATMON;

	micco_read(MICCO_EVENT_B, &val);
	if (val & MICCO_EB_USB_DEV)
		ret |= PMIC_EVENT_VBUS;

	if (val & MICCO_EB_SESSION_1P8) {
		micco_read(MICCO_IRQ_MASK_B, &mask);
		if (!(mask & IRQ_MASK_B_SESSION_VALID_1_8))
			ret |= PMIC_EVENT_VBUS; 
	}

	if (val & MICCO_EB_OTGCP_IOVER) {
		ret |= PMIC_EVENT_OTGCP_IOVER;
	}
	if (val & MICCO_EB_CH_CCTO) {
		ret |= PMIC_EVENT_CH_CCTO;
	}
	if (val & MICCO_EB_CH_TCTO) {
		ret |= PMIC_EVENT_CH_TCTO;
	}

	micco_read(MICCO_EVENT_C, &val);
	if (val & MICCO_EC_PEN_DOWN)
		ret |= PMIC_EVENT_TOUCH;
	
	micco_read(MICCO_EVENT_D, &val);

	return ret;
}

static int is_micco_avbusvld(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_VBUS_VALID_4_55)
		return 0;

	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_VBUS_V_4P4)
		return 1;
	else
		return 0;
}

static int is_micco_asessvld(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_SESSION_VALID_1_8)
		return 0;

	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_SESS_V_1P8)
		return 1;
	else
		return 0;
}

static int is_micco_bsessvld(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_VBUS_VALID_3_8)
		return 0;

	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_VBUS_V_3P8)
		return 1;
	else
		return 0;
}

static int is_micco_srp_ready(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_SESSION_VALID_1_8)
		return 0;

	/* FIXME: When cable detached, the SESS Valid
	 * of STATUA B can not change to 0 immediately.
	 * That will cause potential problems.
	 */
	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_SESS_V_1P8)
		return 1;
	else
		return 0;
}
/* USB Device/OTG functions end here*/

static int curr_apps_mv;
static int curr_sram_mv;

static int get_micco_voltage(int cmd, int *pmv)
{
	int status = 0;
	u8 val;

	*pmv = 0;

	start_calc_time();
	switch (cmd) {
	case VCC_CORE:	/* BUCK1 */
		*pmv = curr_apps_mv;
		status=0;
		break;

	case VCC_SRAM:	/* LDO2 */
		*pmv = curr_sram_mv;
		status=0;
		break;

	case VCC_MVT: /* We assume the MVT is controlled by MDTV1 */
		status = micco_read(MICCO_LDO1_MDTV1, &val);
		if (status)
			return status;

		val &= 0x0f;
		*pmv = val * MICCO_VLDO1_STEP + MICCO_VLDO1_BASE;
		break;

	case VCC_MEM: /* BUCK2, always 1.8v */
		*pmv = 1800;	/* Memory on Littleton is 1.8v */
		break;

	case VCC_3V_APPS:
	case VCC_USB:	/* LDO 3 for USB, 3.0V */
		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val &= 0x0f;
		*pmv = val * MICCO_VLDO3_STEP + MICCO_VLDO3_BASE;
		break;

	case VCC_CAMERA_ANA: /* LDO 6 for CAMERA ANALOG */
		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val = (val & 0xE0) >> 5;
		*pmv = val * MICCO_VLDO6_STEP + MICCO_VLDO6_BASE;
		break;

	case VCC_1P8V:	/* LDO 4 for VCC 1.8v */
		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		if (val & 0x10)
			*pmv = 2900;
		else
			*pmv = 1800;
		break;

	case VCC_LCD:	/* LDO 12 for LCD, LCD I/O */
		status = micco_read(MICCO_LDO1312, &val);
		if (status)
			return status;

		val &= 0x07;
		if (val & 0x08)
			*pmv = val * MICCO_VLDO12_STEP + MICCO_VLDO12_BASE1;
		else
			*pmv = val * MICCO_VLDO12_STEP + MICCO_VLDO12_BASE;
		break;

	case VCC_CAMERA_IO:	/* LDO 15 for CAMERA IO */
		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val = (val & 0xF0) >> 4;
		*pmv = val * MICCO_VLDO15_STEP + MICCO_VLDO15_BASE;
		break;

	case VCC_SDIO:	/* LDO 14 for SDIO */
		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val &= 0x0f;
		*pmv = val * MICCO_VLDO14_STEP + MICCO_VLDO14_BASE;
		break;

	case HDMI_TX:	/* LDO 9 for HDMI TX */
		status = micco_read(MICCO_LDO987, &val);
		if (status)
			return status;

		val = (val & 0xE0) >> 5;
		*pmv = val * MICCO_VLDO9_STEP + MICCO_VLDO9_BASE;
		break;

	case TECH_3V:	/* LDO 10 for TECH 3v */
		status = micco_read(MICCO_LDO987, &val);
		if (status)
			return status;

		val &= 0x07;
		*pmv = val * MICCO_VLDO10_STEP + MICCO_VLDO10_BASE;
		break;

	case TECH_1P8V:	/* LDO 11 for TECH 1.8v */
		status = micco_read(MICCO_LDO1110, &val);
		if (status)
			return status;

		val = (val & 0xf0) >> 4;
		*pmv = val * MICCO_VLDO11_STEP + MICCO_VLDO11_BASE;
		break;

	default:
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}
	end_calc_time();
	return status;
}

static int set_micco_apps_and_sram(int apps_mv, int sram_mv);

static int set_micco_voltage(int cmd, int mv)
{
	int status;
	u8 val;

	start_calc_time();
	switch (cmd) {
	case VCC_CORE:
		status = set_micco_apps_and_sram(mv,-1);
		break;

	case VCC_SRAM:	/* LDO 2 */
		status = set_micco_apps_and_sram(-1,mv);
		break;

	case VCC_MVT:	/* MDTV1 */
		if ((mv < MICCO_VLDO1_BASE) && (mv > MICCO_VLDO1_MAX))
			return -EINVAL;
		status = micco_read(MICCO_LDO1_MDTV1, &val);
		if (status)
			return status;

		val &= 0xf0;
		val = val | ((mv - MICCO_VLDO1_BASE) / MICCO_VLDO1_STEP);
		status = micco_write(MICCO_LDO1_MDTV1, val);
		break;

	case VCC_MEM:	/* BUCK2, always 1.8v */
		return 0;

	case VCC_3V_APPS:
	case VCC_USB:	/* LDO 3 */
		if ((mv < MICCO_VLDO3_BASE) && (mv > MICCO_VLDO3_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val &= 0xf0;
		val = val | ((mv - MICCO_VLDO3_BASE) / MICCO_VLDO3_STEP);
		status = micco_write(MICCO_LDO643, val);
		break;

	case VCC_CAMERA_ANA: /* LDO 6 for CAMERA ANALOG */
		if ((mv < MICCO_VLDO6_BASE) && (mv > MICCO_VLDO6_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val &= 0x1f;
		val = val | ((mv - MICCO_VLDO6_BASE) / MICCO_VLDO6_STEP);
		status = micco_write(MICCO_LDO643, val);
		break;

	case VCC_1P8V:	/* LDO 4 */
		if ((mv != 2900) && (mv !=1800))
			return -EINVAL;
		status = micco_read(MICCO_LDO643, &val);
		if (2900 == mv)
			val |= 0x10;
		else
			val &= ~0x10;
		status = micco_write(MICCO_LDO643, val);
		break;

	case VCC_LCD:/* LDO 12 */
		if ((mv < MICCO_VLDO12_BASE) && (mv > MICCO_VLDO12_MAX))
			return -EINVAL;


		status = micco_read(MICCO_LDO1312, &val);
		if (status)
			return status;

		val &= 0xf8;
		if (mv > MICCO_VLDO12_BASE1) {
			val = val | ((mv - MICCO_VLDO12_BASE1) /	\
				MICCO_VLDO12_STEP);
			val |= 0x08;
		} else {
			val = val | ((mv - MICCO_VLDO12_BASE) /		\
				MICCO_VLDO12_STEP);
			val &= ~0x08;
		}
		status = micco_write(MICCO_LDO1312, val);
		break;

	case VCC_CAMERA_IO:	/* LDO 15 */
		if ((mv < MICCO_VLDO15_BASE) && (mv > MICCO_VLDO15_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val &= 0x0f;
		val = val | ((mv - MICCO_VLDO15_BASE) / MICCO_VLDO15_STEP);
		status = micco_write(MICCO_LDO1514, val);
		break;

	case VCC_SDIO: /* LDO 14 */
		if ((mv < MICCO_VLDO14_BASE) && (mv > MICCO_VLDO14_MAX))
			return -EINVAL;
		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val &= 0xf0;
		val = val | ((mv - MICCO_VLDO14_BASE) / MICCO_VLDO14_STEP);
		status = micco_write(MICCO_LDO1514, val);
		break;

	case HDMI_TX:	/* LDO 9 */
		if ((mv < MICCO_VLDO9_BASE) && (mv > MICCO_VLDO9_MAX))
			return -EINVAL;
		status = micco_read(MICCO_LDO987, &val);
		if (status)
			return status;

		val &= 0x1f;
		val = val | ((mv - MICCO_VLDO9_BASE) / MICCO_VLDO9_STEP);
		status = micco_write(MICCO_LDO987, val);
		break;
	case TECH_3V:	/* LDO 10 */
		if ((mv < MICCO_VLDO10_BASE) && (mv > MICCO_VLDO10_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO1110, &val);
		if (status)
			return status;

		val &= 0xf8;
		val = val | ((mv - MICCO_VLDO10_BASE) / MICCO_VLDO10_STEP);
		status = micco_write(MICCO_LDO1110, val);
		break;
	
	case TECH_1P8V:	/* LDO 11 */
		if ((mv < MICCO_VLDO11_BASE) && (mv > MICCO_VLDO11_MAX))
			return -EINVAL;
		
		status = micco_read(MICCO_LDO1110, &val);
		if (status)
			return status;

		val &= 0x0f;
		val = val | ((mv - MICCO_VLDO11_BASE) / MICCO_VLDO11_STEP);
		status = micco_write(MICCO_LDO1110, val);
		break;

	default:
		printk(KERN_INFO "error command\n");
		return -EINVAL;
	}
	
	if (status != 0)
		return status;
	
	end_calc_time();
	return status;
}

static int set_micco_apps_and_sram(int apps_mv, int sram_mv)
{
	static int xappsmv[2]={-1,-1};
	static int xsrammv[2]={-1,-1};
	static int xappsn=0;
	static int xsramn=0;
	u8 val=0;
	u8 val2;
	int status;
	if (apps_mv==-1 && sram_mv==-1) return 0;
	if (apps_mv!=-1) {
		if ((apps_mv<MICCO_VBUCK1_BASE)||(apps_mv>MICCO_VBUCK1_MAX)) return -EINVAL;
		if (xappsmv[0]==apps_mv) {
			val|=0x01;
		} else if (xappsmv[1]==apps_mv) {
			val|=0x03;
		} else {
			val|=0x01;
			if (xappsn) val|=0x02;
			val2=((apps_mv-MICCO_VBUCK1_BASE)/MICCO_VBUCK1_STEP);
			val2&=0x1f;
			status=micco_write(MICCO_BUCK1_DVC1+xappsn,val2);
			if (status) return status;
			xappsmv[xappsn]=apps_mv;
			xappsn ^= 1;
		}
	}
	if (sram_mv!=-1) {
		if ((sram_mv<MICCO_VSRAM_BASE)||(sram_mv>MICCO_VSRAM_MAX)) return -EINVAL;
		if (xsrammv[0]==sram_mv) {
			val|=0x10;
		} else if (xsrammv[1]==sram_mv) {
			val|=0x30;
		} else {
			val|=0x10;
			if (xsramn) val|=0x20;
			val2=((sram_mv-MICCO_VSRAM_BASE)/MICCO_VSRAM_STEP);
			val2&=0x1f;
			status=micco_write(MICCO_SRAM_DVC1+xsramn,val2);
			if (status) return status;
			xsrammv[xsramn]=sram_mv;
			xsramn ^= 1;
		}
	}

	status = micco_write(MICCO_VCC1, val);
	if (status==0) {
		if (apps_mv!=-1) curr_apps_mv=apps_mv;
		if (sram_mv!=-1) curr_sram_mv=sram_mv;
	}
	return status;
}
	

/* 
 * Micco interrupt service routine. 
 * In the ISR we need to check the Status bits in Micco and according
 * to those bits to check which kind of IRQ had happened.
 */
static irqreturn_t micco_irq_handler(int irq, void *dev_id)
{
	unsigned int event;
	u8 val;

	event = micco_event_change();
	if (save_event) {
		micco_event=event;
		save_event=0;
	}
	pmic_event_handle(event);

	/* We don't put these codes to USB specific code because
	 * we need handle it even when no USB callback registered.
	 */
	if (event & PMIC_EVENT_OTGCP_IOVER) {
		/* According to Micco spec, when OTGCP_IOVER happen,
		 * Need clean the USBPCP_EN in MISC. and then set
		 * it again.
		 */
		micco_read(MICCO_MISC, &val);
		val &= ~MICCO_MISC_USBCP_EN;
		micco_write(MICCO_MISC, val);
		val |= MICCO_MISC_USBCP_EN;
		micco_write(MICCO_MISC, val);
	}

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
/*
 * Suspend the micco interface.
 * Add register save/restore here.
 */
extern void lt_charger_suspend(void);
static int micco_suspend(struct platform_device *pdev, pm_message_t state)
{
	unsigned char val;
	unsigned long flags;

	disable_irq(IRQ_MICCO);
	// XXX BT the strategy here is to put the interrupt pin into the
	// XXX sleep mode and then gratuitously run through the irq handler.
	// XXX in this way it should be the case that any interrupt raised
	// XXX on or after this last run of the handler will be caught by
	// XXX the mfp edge recognition logic and so any race around
	// XXX recognition of the interrupt with respect to standby should
	// XXX be avoided.
	woodstock_micco_sleep_pins_config();
	local_irq_save(flags);
	micco_irq_handler(IRQ_MICCO,NULL);
	local_irq_restore(flags);
        lt_charger_suspend();

#if 0
	if ((get_pm_state() == PM_SUSPEND_MEM)) {
		micco_read(MICCO_OVER1, &val);
		micco_regs[MICCO_OVER1].data = val;
		micco_read(MICCO_APP_OVER2, &val);
		micco_regs[MICCO_APP_OVER2].data = val;
		micco_read(MICCO_APP_OVER3, &val);
		micco_regs[MICCO_APP_OVER3].data = val;
		micco_read(MICCO_BUCK_SLEEP, &val);
		micco_regs[MICCO_BUCK_SLEEP].data = val;
		micco_read(MICCO_SYSCTRL_A, &val);
		micco_regs[MICCO_SYSCTRL_A].data = val;

		micco_write(MICCO_BUCK_SLEEP, 0x00); //6D
		micco_write(MICCO_SYSCTRL_A, 0xff); //XXX BT don't understand why this is necessary but it is

/*		micco_write(MICCO_OVER1, 0x04);
		micco_write(MICCO_APP_OVER2, 0x30);
		micco_write(MICCO_APP_OVER3, 0x7c);
*/
	}
#endif
	micco_read(MICCO_WLED_CONTROL2, &val);
	micco_regs[MICCO_WLED_CONTROL2].data = val;
	micco_write(MICCO_WLED_CONTROL2, 0);
#if 0
	micco_read(MICCO_AUDIO_REG_BASE+MICCO_MONO_VOL,&val);
	printk("At suspend, mono vol is %02x\n",val);
	micco_regs[MICCO_AUDIO_REG_BASE+MICCO_MONO_VOL].data = val;
	micco_write(MICCO_AUDIO_REG_BASE+MICCO_MONO_VOL,0);

        micco_read(MICCO_AUDIO_REG_BASE+MICCO_HIFI_DAC_CONTROL,&val);
	printk("At suspend, hifi dac control is %02x\n",val);
	micco_regs[MICCO_AUDIO_REG_BASE+MICCO_HIFI_DAC_CONTROL].data=val;
	micco_write(MICCO_AUDIO_REG_BASE+MICCO_HIFI_DAC_CONTROL,0);
#endif
	set_micco_apps_and_sram(1400,1400);
	return 0;
}

/*
 * Resume the Micco interface.
 */
extern void lt_charger_resume(void);
static int micco_resume(struct platform_device *pdev)
{
	int i;
	unsigned long flags;

	set_micco_apps_and_sram(1375,1375);
	for (i = 0; i < MICCO_REG_NUM; i++)
		micco_regs[i].hit = 0;
        lt_charger_resume();
	local_irq_save(flags);
	save_event=1;
	micco_irq_handler(IRQ_MICCO, NULL);
	local_irq_restore(flags);
	enable_irq(IRQ_MICCO);
#if 0
	if ((get_pm_state() == PM_SUSPEND_MEM)) {
		micco_write(MICCO_OVER1, micco_regs[MICCO_OVER1].data);
		micco_write(MICCO_APP_OVER2, micco_regs[MICCO_APP_OVER2].data);
		micco_write(MICCO_APP_OVER3, micco_regs[MICCO_APP_OVER3].data);
		micco_write(MICCO_BUCK_SLEEP, micco_regs[MICCO_BUCK_SLEEP].data);
		micco_write(MICCO_SYSCTRL_A, micco_regs[MICCO_SYSCTRL_A].data);
	}
#endif
	micco_write(MICCO_WLED_CONTROL2, micco_regs[MICCO_WLED_CONTROL2].data);

#if 0
	micco_write(MICCO_AUDIO_REG_BASE+MICCO_HIFI_DAC_CONTROL,micco_regs[MICCO_AUDIO_REG_BASE+MICCO_HIFI_DAC_CONTROL].data);
	micco_write(MICCO_AUDIO_REG_BASE+MICCO_MONO_VOL,micco_regs[MICCO_AUDIO_REG_BASE+MICCO_MONO_VOL].data);
#endif

	return 0;
}

#else				/*  */
#define	micco_suspend		NULL
#define	micco_resume		NULL
#endif				/*  */

static struct pmic_ops micco_pmic_ops = {
	.get_voltage		= get_micco_voltage,
	.set_voltage		= set_micco_voltage,

	.is_vbus_assert		= is_micco_vbus_assert,
	.is_avbusvld		= is_micco_avbusvld,
	.is_asessvld		= is_micco_asessvld,
	.is_bsessvld		= is_micco_bsessvld,
	.is_srp_ready		= is_micco_srp_ready,

	.set_pump		= micco_set_pump,
	.set_vbus_supply	= micco_set_vbus_supply,
	.set_usbotg_a_mask	= micco_set_usbotg_a_mask,
	.set_usbotg_b_mask	= micco_set_usbotg_b_mask,
	.set_apps_and_sram	= set_micco_apps_and_sram,
};

struct pxa3xx_pin_config lt_micco_pins[] = {
PXA3xx_MFP_CFG("Micco INT", MFP_PMIC_INT, MFP_AF0, MFP_DS03X, 0, MFP_LPM_PULL_HIGH|0x10, MFP_EDGE_NONE),
};

struct pxa3xx_pin_config lt_micco_sleep_pins[] = {
PXA3xx_MFP_CFG("Micco INT", MFP_PMIC_INT, MFP_AF1, MFP_DS03X, 1, MFP_LPM_PULL_HIGH|0x10, MFP_EDGE_FALL),
};

void woodstock_micco_pins_config(void)
{
	pxa3xx_mfp_set_configs(lt_micco_pins, ARRAY_SIZE(lt_micco_pins));
}

void woodstock_micco_sleep_pins_config(void)
{
	pxa3xx_mfp_set_configs(lt_micco_sleep_pins, ARRAY_SIZE(lt_micco_sleep_pins));
}


struct pxa3xx_pin_config wstk_tpxres_pin[] = {
PXA3xx_MFP_CFG("TP XRES", MFP_PIN_GPIO24, MFP_PIN_GPIO24_AF_GPIO_24, MFP_DS03X, 1, MFP_LPM_DRIVE_HIGH, MFP_EDGE_NONE),
};

#ifdef CONFIG_MICCO_IRQ_HOUSEKEEPING
/* NOTE: If the Micco interrupt is lost (the micco interrupt happen when irq 
 * is disable by irq_disable), the Micco interrupt will not happen because
 * no one clear the EVENT register.  This thread is use to clear the Micco EVENT
 * in case the micco interrupt is lost. the interval is about 1s.
 */
static int micco_thread(void *data)
{
	unsigned long flags;
	daemonize("Micco thread");
	while (1) {
		msleep_interruptible(1000);
		if (signal_pending(current)) try_to_freeze();
		local_irq_save(flags);
		micco_irq_handler(IRQ_MICCO, NULL);
		local_irq_restore(flags);
	}
	return 0;
}
#endif

#ifdef	CONFIG_PROC_FS
#define	MICCO_PROC_FILE	"driver/micco"
static struct proc_dir_entry *micco_proc_file;
static int index = 0;

static ssize_t micco_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if ((index < 1) || (index > MICCO_REG_NUM)) 
		return 0;

	micco_read(index, &reg_val);
	printk("register 0x%x: 0x%x\n",index, reg_val);
	return 0;
}

static ssize_t micco_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0] ) {	/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {			/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		micco_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations micco_proc_ops = {
	.read = micco_proc_read,
	.write = micco_proc_write,
};

static void create_micco_proc_file(void)
{
	micco_proc_file = create_proc_entry(MICCO_PROC_FILE, 0644, NULL);
	if (micco_proc_file) {
		micco_proc_file->owner = THIS_MODULE;
		micco_proc_file->proc_fops = &micco_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_micco_proc_file(void)
{
	remove_proc_entry(MICCO_PROC_FILE, &proc_root);
}

#endif

static int micco_probe(struct platform_device *pdev)
{
	int ret;
	u8 value;
	
	printk("micco_probe\n");
	ret = micco_initchip();
	if (ret != 0) {
		printk("Initialize Micco failed with ret 0x%x\n", ret);
	}

	pxa3xx_mfp_set_configs(lt_micco_pins, ARRAY_SIZE(lt_micco_pins));	
	pxa3xx_gpio_set_direction(MFP_PMIC_INT, GPIO_DIR_IN);
	ret = request_irq(IRQ_MICCO, micco_irq_handler, IRQF_TRIGGER_FALLING|IRQF_DISABLED,
			"Micco", NULL);
	if (ret) {
		printk("Request IRQ for Micco failed, return :%d\n", ret);
		return ret;
	}

#ifdef CONFIG_MICCO_IRQ_HOUSEKEEPING
	if ((ret = kernel_thread(&micco_thread, NULL, 0)) < 0) {
		free_irq(IRQ_MICCO, NULL);
		return ret;
	}
#endif
	
	/* Mask interrupts that are not needed */
	micco_write(MICCO_IRQ_MASK_A, 0xFE);
	micco_write(MICCO_IRQ_MASK_B, 0xFF);
	micco_write(MICCO_IRQ_MASK_C, 0xFF);
	micco_write(MICCO_IRQ_MASK_D, 0xFF);

	/* avoid SRAM power off during sleep*/
	micco_write(0x10, 0x05);
	micco_write(0x11, 0xff);
	micco_write(0x12, 0xff);

#ifndef CONFIG_MACH_WOODSTOCK
	/* Enable the ONKEY power down functionality */
	micco_write(MICCO_SYSCTRL_B, 0x20);
#endif
	micco_write(MICCO_SYSCTRL_A, 0x60);

#ifdef CONFIG_MACH_WOODSTOCK
	micco_write(0x14, 0x56); /*LDO7 up to 3V*/
	// micco_write(0x15, 0xC3); /*LDO11 up to 3V*/
	micco_write(0x11, 0x23); /*disable LDOs 8,9,10,12,13 (enable 6,7,11)*/
	micco_write(0x12, 0xF8); /*disable RF_GP, LDOs 15,14 */
	micco_write(0x18, 0x00); /*enable BUCK1 sleeps on nSLEEP */
	micco_write(0x0b, 0x60); /*enable nSLEEP pin*/
	micco_write(0x3b, 0x00); /*turn off LED_PC*/
	pxa3xx_mfp_set_configs(wstk_tpxres_pin, ARRAY_SIZE(wstk_tpxres_pin));
	pxa3xx_gpio_set_level(MFP_PIN_GPIO24,GPIO_LEVEL_HIGH);
	pxa3xx_gpio_set_direction(MFP_PIN_GPIO24,GPIO_DIR_OUT);
	mdelay(5);
	micco_write(0x13, 0xCD); /*LDO6 (accelerometer) to 2.8V, LDO3 (TP) to 3.1V*/
	mdelay(5);
	pxa3xx_gpio_set_level(MFP_PIN_GPIO24,GPIO_LEVEL_LOW);
	mdelay(50);
	micco_write(MICCO_APPS_AVRC,0x08); //6.25mV/us ramp rate
	micco_write(MICCO_SRAM_SVRC,0x08); //6.25mV/us ramp rate
	set_micco_apps_and_sram(1375,1375);
#endif
#ifdef CONFIG_SONOS_DIAGS
	printk("SONOS_DIAGS: Turning on VBOOST\n");
	micco_write(MICCO_WLED_CONTROL1,95);
	micco_write(MICCO_LED2_CONTROL,0x5f);
	micco_write(MICCO_WLED_CONTROL2,0x2a);
#endif

	/* IRQ is masked during the power-up sequence and will not be released 
	 * until they have been read for the first time */
	micco_read(MICCO_EVENT_A, &value);
	micco_read(MICCO_EVENT_B, &value);
	micco_read(MICCO_EVENT_C, &value);
	micco_read(MICCO_EVENT_D, &value);

	micco_read(MICCO_BUCK1_DVC2, &value);
	value&=0x1f;
	curr_apps_mv = (value * MICCO_VBUCK1_STEP) + MICCO_VBUCK1_BASE;
	micco_read(MICCO_SRAM_DVC2, &value);
	value&=0x1f;
	curr_sram_mv = (value * MICCO_VSRAM_STEP) + MICCO_VSRAM_BASE;

	//micco_enable_LDO12();

#ifdef	CONFIG_PROC_FS
	create_micco_proc_file();
#endif

	pmic_set_ops(&micco_pmic_ops);

	return 0;
}

static int micco_remove(struct platform_device *pdev)
{
	pmic_set_ops(NULL);

#ifdef	CONFIG_PROC_FS
	remove_micco_proc_file();
#endif

	free_irq(IRQ_MICCO, NULL);

	return 0;
}

static struct platform_driver micco_driver = {
	.driver = {
		.name	= "pxa3xx_pmic",
	},
	.probe		= micco_probe,
	.remove		= micco_remove,
	.suspend	= micco_suspend,
	.resume		= micco_resume,
};

/******************************************************************************
 *									      *
 *			MICCO I2C Client Driver				      *
 *									      *
 ******************************************************************************/
#include <linux/i2c.h>
static int i2c_micco_attach_adapter(struct i2c_adapter *adapter);
static int i2c_micco_detect_client(struct i2c_adapter *, int, int);
static int i2c_micco_detach_client(struct i2c_client *client);
#define I2C_DRIVERID_MICCO   0

struct i2c_driver i2c_micco_driver  = 
{
	.driver = {
		.name	= "micco i2c client driver",
	},
	.attach_adapter	= &i2c_micco_attach_adapter, 
	.detach_client	= &i2c_micco_detach_client,  
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = {MICCO_ADDRESS, I2C_CLIENT_END };
I2C_CLIENT_INSMOD_1(micco);

extern int i2c_pxa_set_speed(int speed);

int micco_read(u8 reg, u8 *pval)
{
	int ret;
	int pre_speed;
	unsigned long flags;	
	int status;
	
	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the Micco register. Disabled temporary */
#if	0
	if (micco_regs[reg].hit) {
		*pval = micco_regs[reg].data;
		return 0;
	}
#endif

	ret = i2c_smbus_read_byte_data(g_client, reg);
	if (ret >= 0) {
		*pval = ret;
		micco_regs[reg].hit = ~micco_regs[reg].mask;
		micco_regs[reg].data = ret;
		status = 0;
	}else
		status = -EIO;

	return status;
}

int micco_write(u8 reg, u8 val)
{
	int ret;
	int pre_speed;
	unsigned long flags;	
	int status;
	
	if(g_client == NULL)	/* No global client pointer? */
		return -1;

	ret = i2c_smbus_write_byte_data(g_client, reg, val);	
	if (ret == 0) {
		micco_regs[reg].hit = ~micco_regs[reg].mask;
		micco_regs[reg].data = val;
		status = 0;
	} else
		status = -EIO;
	
	return status;
}

static int i2c_micco_attach_adapter(struct i2c_adapter *adap)
{	
	return i2c_probe(adap,&addr_data, &i2c_micco_detect_client);
}

static int i2c_micco_detect_client(struct i2c_adapter *adapter,
		int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;
	int chip_id;
 
	/* Let's see whether this adapter can support what we need.
	       Please substitute the things you need here!  */
	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		err = -EPERM;
		goto ERROR0;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet.
	   But it allows us to access several i2c functions safely */
    
	/* Note that we reserve some space for micco_data too. If you don't
	   need it, remove it. We do it here to help to lessen memory
	   fragmentation. */

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL );

	if ( !new_client )  {
		err = -ENOMEM;
		goto ERROR0;
	}

	new_client->addr = address;	
 	new_client->adapter = adapter;
	new_client->driver = &i2c_micco_driver;
	new_client->flags = 0;

	chip_id = i2c_smbus_read_byte_data(new_client, MICCO_CHIP_ID);
	if (chip_id < 0){
		printk("micco unavailable!\n");
		goto ERROR1;
	} else {
		printk("micco(chip id:0x%02x) detected.\n", chip_id);
	}

	g_client = new_client;

	strcpy(new_client->name, "MICCO");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	return 0;

ERROR1:
	g_client = NULL;
	kfree(new_client);
ERROR0:
	printk(KERN_ALERT "i2c detect client failed\n");
        return err;
}

static int i2c_micco_detach_client(struct i2c_client *client)
{
	int err = 0;

  	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk(KERN_WARNING "Micco: Client deregistration failed,"
			" client not detached.\n");
	        return err;
	}

	/* Frees client data too, if allocated at the same time */
	kfree(client);
	g_client = NULL;
	return 0;
}

#ifdef CONFIG_MACH_WOODSTOCK
extern int i2c_adap_pxa_init(void);
#endif

static int __init micco_init(void)
{
	int ret;

	if ((ret = i2c_add_driver(&i2c_micco_driver))) {
		printk(KERN_WARNING "Micco: Driver registration failed,"
			" module not inserted.\n");
		return ret;
	}
	ret = platform_driver_register(&micco_driver);

#ifdef CONFIG_MACH_WOODSTOCK
        i2c_adap_pxa_init();
#endif
	return ret;
}

static void __exit micco_exit(void)
{
	platform_driver_unregister(&micco_driver);

	if (i2c_del_driver(&i2c_micco_driver)) {
		printk(KERN_WARNING "Micco: Driver registration failed,"
			"module not removed.\n");
	}
} 

module_init(micco_init);
module_exit(micco_exit);

MODULE_DESCRIPTION("Micco Driver");
MODULE_LICENSE("GPL");
