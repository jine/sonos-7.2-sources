/*
 * linux/sound/arm/codec/wm9713.c.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/kernel.h>

#include <asm/arch/codec/ac97acodec.h>
#include <asm/arch/codec/wm9713.h>
#include <asm/arch/codec/acodec.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pxa3xx_pmic.h>
#include <asm/arch/arava.h>

#ifdef ALSA_ZY_WM9713_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG "file: " __FILE__ "Line:(%d) FUNC:%s******\n" format "\n" , __LINE__ , __func__ , ##arg)
#else
#define dbg(format, arg...)
#endif

extern int codec_client;
extern int touch_client;

int golden_muted = 0;

static int current_mast_input_gain = 15;
static int current_headset_vol = 48;
static int current_speaker_vol = 48;

static const unsigned char zy_9713_unsave [] =
{
	0x00,
	0x28,
	0x54,
	0x68,
	0x6a,
	0x6c,
	0x6e,
	0x74,
	0x7a,
	0x7c,
	0x7e
};

acodec_error_t zy_wm9713_set_master_vol(acodec_context_t *p_device_context, unsigned short gain_in_db)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;
	unsigned char left_volume, right_volume, vol;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	/*take the unit of gain_in_db as 1 db*/
	left_volume = (unsigned char)(gain_in_db * 10 / 15);
	if (left_volume > ZY_AC97_WM9713_MAX_VOLUME) {
		left_volume = ZY_AC97_WM9713_MAX_VOLUME;
	}
	vol = right_volume = left_volume;

	if ((0 == left_volume) && (0 == right_volume)) {
		/* mute the output */
		value = ZY_AC97_WM9713_MVR_MM;
	} else {
		/* translate into attenuation value */
		left_volume = ZY_AC97_WM9713_MAX_VOLUME - left_volume;
		right_volume = ZY_AC97_WM9713_MAX_VOLUME - right_volume;
		value = left_volume << ZY_AC97_WM9713_MVR_ML_SHIFT;
		value |= right_volume;
	}

	status = zy_ac97_acodec_write (p_device_context, SPEAKER_VOLUME, value);
	if (status == ACODEC_SUCCESS) {
		current_speaker_vol = vol * 15 / 10;
	}
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);

	return status;
}

acodec_error_t zy_wm9713_set_master_input_gain(acodec_context_t *p_device_context, unsigned short gain_in_db)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned char left_volume, right_volume, vol;
	unsigned short value;

	/*take the unit of gain_in_db as 1 db*/
	left_volume = (unsigned char)(gain_in_db * 10 / 15);
	if (left_volume > ZY_AC97_WM9713_MAX_ADCGAIN) {
		left_volume = ZY_AC97_WM9713_MAX_ADCGAIN;
	}
	vol = left_volume;
	right_volume = left_volume;
	if ((0 == left_volume) && (0 == right_volume)) {
		/*mute the record*/
		value = ZY_AC97_WM9713_RGR_RM;
	} else {
		value = left_volume << ZY_AC97_WM9713_RGR_GL_SHIFT;
		value |= right_volume;
	}
	status = zy_ac97_acodec_write (p_device_context, REC_PGA_VOL, value);
	if (status == ACODEC_SUCCESS) {
   		current_mast_input_gain = vol * 15/10 ;
   	}
	return status;
}


acodec_error_t zy_wm9713_get_in_sample_rate(acodec_context_t *p_device_context, unsigned short * rate_in_hz)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	status = zy_ac97_acodec_read(p_device_context, AUDIO_ADC_RATE, &value);
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);

	if (ACODEC_SUCCESS != status)
	{
		return status;
	}

	switch (value)
	{
		case ZY_AC97_WM9713_DR_8000:
			*rate_in_hz = 8000;
			break;
		case ZY_AC97_WM9713_DR_11025:
			*rate_in_hz = 11025;
			break;
		 case ZY_AC97_WM9713_DR_12000:
			*rate_in_hz = 12000;
			break;
		case ZY_AC97_WM9713_DR_16000:
			*rate_in_hz = 16000;
			break;
		case ZY_AC97_WM9713_DR_22050:
			*rate_in_hz = 22050;
			break;
		case ZY_AC97_WM9713_DR_24000:
			*rate_in_hz = 24000;
			break;
		case ZY_AC97_WM9713_DR_32000:
			*rate_in_hz = 32000;
			break;
		case ZY_AC97_WM9713_DR_44100:
			*rate_in_hz = 44100;
			break;
		default:
			*rate_in_hz = 48000;
			break;
	}

	return status;
}

acodec_error_t zy_wm9713_get_out_sample_rate (acodec_context_t *p_device_context, unsigned short * rate_in_hz)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	status = zy_ac97_acodec_read(p_device_context, AUDIO_DAC_RATE, &value);

	if (ACODEC_SUCCESS != status)
	{
		return status;
	}

	switch (value)
	{
		case ZY_AC97_WM9713_DR_8000:
			*rate_in_hz = 8000;
			break;
		case ZY_AC97_WM9713_DR_11025:
			*rate_in_hz = 11025;
			break;
		case ZY_AC97_WM9713_DR_12000:
			*rate_in_hz = 12000;
			break;
		case ZY_AC97_WM9713_DR_16000:
			*rate_in_hz = 16000;
			break;
		case ZY_AC97_WM9713_DR_22050:
			*rate_in_hz = 22050;
			break;
		case ZY_AC97_WM9713_DR_32000:
			*rate_in_hz = 32000;
			break;
		 case ZY_AC97_WM9713_DR_24000:
			*rate_in_hz = 24000;
			break;
		case ZY_AC97_WM9713_DR_44100:
			*rate_in_hz = 44100;
			break;
		default:
			*rate_in_hz = 48000;
			break;
	}

	return status;
}

acodec_error_t zy_wm9713_voice_prepare(acodec_context_t *p_device_context)
{
	acodec_error_t status = ACODEC_SUCCESS;
	unsigned short reg = 0x8000;
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	/*master mode*/
	reg |= 0x4000;
	/*normal polarity*/
	/*output left ADC only*/
	reg |= 0x20;
	/*16bit DSP mode*/
	reg |= 0x3;

	reg |=(0x1 << 9);
	reg |=(0x0 << 8);

	status = zy_ac97_acodec_write(p_device_context, PCM_CODEC_CTRL, reg);
	status = zy_ac97_acodec_write(p_device_context, GPIO_PIN_CFG, 0xffd4);
	
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return status;
}

/*On zylonite board, the wm9713 is set as MAST mode*/
acodec_error_t zy_wm9713_set_voice_out_sample_rate(acodec_context_t *p_device_context, unsigned short rate_in_hz)
{
	acodec_error_t status = ACODEC_SUCCESS;
	unsigned short value, tem_value;
	/*
	 * reg 0x44 DIV, the PCM_CLK= AC97_CLK/DIV, the PCM_CLK should be 24.576Mhz.
	 * however, basing on the test, PCM_CLK = AC97_CLK/(DIV*2)
	 */
	switch (rate_in_hz)
	{
		case 8000:
			value = 0x5;
			break;
		case 16000:
			value = 0x2;
			break;
		case 48000:
			value = 0x0;
			break;
		default:
			dbg("sample rate:%d can not be supported\n", rate_in_hz);
			return ACODEC_SAMPLERATE_NOT_SUPPORTED;
	}
	status = zy_ac97_acodec_read(p_device_context, MCLK_PLL_CTRL_1,&tem_value);
	tem_value = (tem_value & 0xf0ff) | (value << 8);
	status = zy_ac97_acodec_write(p_device_context, MCLK_PLL_CTRL_1, tem_value);
	return status;
}

acodec_error_t zy_wm9713_set_in_sample_rate(acodec_context_t *p_device_context, unsigned short rate_in_hz)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	switch (rate_in_hz)
	{
		case 8000:
			value = ZY_AC97_WM9713_DR_8000;
			break;
		case 11025:
			value = ZY_AC97_WM9713_DR_11025;
			break;
		case 12000:
			value = ZY_AC97_WM9713_DR_12000;
			break;
		case 16000:
			value = ZY_AC97_WM9713_DR_16000;
			break;
		case 22050:
			value = ZY_AC97_WM9713_DR_22050;
			break;
		case 24000:
			value = ZY_AC97_WM9713_DR_24000;
			break;
		case 32000:
			value = ZY_AC97_WM9713_DR_32000;
			break;
		case 44100:
			value = ZY_AC97_WM9713_DR_44100;
			break;
		case 48000:
			value = ZY_AC97_WM9713_DR_48000;
			break;
		default:
			return ACODEC_SAMPLERATE_NOT_SUPPORTED;
	}
	/*enable VRA mode*/
	status = zy_ac97_acodec_write(p_device_context, ZY_AC97_CR_E_AUDIO_CTRL_STAT, ZY_AC97_WM9713_EASCR_VRA);
	if (ACODEC_SUCCESS != status)
	{
		return status;
	}

	status = zy_ac97_acodec_write(p_device_context, ZY_AC97_CR_E_ASR_PCM_LR_ADC_RT, value);
	return status;
}

acodec_error_t zy_wm9713_set_out_sample_rate (acodec_context_t *p_device_context, unsigned short rate_in_hz)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	switch (rate_in_hz)
	{
		case 8000:
			value = ZY_AC97_WM9713_DR_8000;
			break;
		case 11025:
			value = ZY_AC97_WM9713_DR_11025;
			break;
		case 12000:
			value = ZY_AC97_WM9713_DR_12000;
			break;
		case 16000:
			value = ZY_AC97_WM9713_DR_16000;
			break;
		case 22050:
			value = ZY_AC97_WM9713_DR_22050;
			break;
		case 24000:
			value = ZY_AC97_WM9713_DR_24000;
			break;
		case 32000:
			value = ZY_AC97_WM9713_DR_32000;
			break;
		case 44100:
			value = ZY_AC97_WM9713_DR_44100;
			break;
		case 48000:
			value = ZY_AC97_WM9713_DR_48000;
			break;
		default:
			return ACODEC_SAMPLERATE_NOT_SUPPORTED;
	}
	/*enable VRA mode*/
	status = zy_ac97_acodec_write(p_device_context, EXTENDED_AUD_STAT_CTRL, ZY_AC97_WM9713_EASCR_VRA);
	if (ACODEC_SUCCESS != status)
	{
		return status;
	}

	status = zy_ac97_acodec_write(p_device_context, AUDIO_DAC_RATE, value);
	return status;
}

acodec_error_t zy_wm9713_set_voice_in_sample_rate(acodec_context_t *p_device_context, unsigned short rate_in_hz)
{
	acodec_error_t status;
	/*set the PCM interface clock*/
	status = zy_wm9713_set_voice_out_sample_rate(p_device_context, rate_in_hz);
	/*set the ADCLR sample*/
	status = zy_wm9713_set_in_sample_rate(p_device_context, rate_in_hz);
	return status;
}

void zy_wm9713_get_event(acodec_context_t *p_device_context, unsigned char * event_type)
{
	unsigned short event_state = 0;
	zy_ac97_acodec_read(p_device_context, GPIO_PIN_STATUS, &event_state);
	if(event_state & ZY_AC97_WM9713_GPIO_PIN_PDN){
		*event_type = EVENT_TYPE_PDN;
		return;
	}
	return;
}

void zy_wm9713_event_ack(acodec_context_t *p_device_context, unsigned char event_type)
{
	unsigned short event_state = 0;
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	zy_ac97_acodec_read(p_device_context, GPIO_PIN_STATUS, &event_state);
	if( event_type == EVENT_TYPE_PDN){
		zy_ac97_acodec_write(p_device_context,
					GPIO_PIN_STATUS,
					(event_state & (~ZY_AC97_WM9713_GPIO_PIN_PDN)));
	}

	zy_ac97_acodec_read(p_device_context, GPIO_PIN_STATUS, &event_state);
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return;
}

/* This is platform specific
 * do later: not to enable recording route and playback route in this function,
 * leave it to driver to call other function
 */
acodec_error_t zy_wm9713_specific_init (acodec_context_t *p_device_context)
{
#ifdef CONFIG_AC97_EXTCLK
	unsigned short value;
#endif
	/* This assumes that the aclink is initialized
	 * wait some time and then do a warm reset to enabled the ACLINK,
	 * required for wm9713 (not wm9712 or ucb1400)
	 */

	/*pay attention: whether the workaround is still needed?*/
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(p_device_context->p_ctrl_reg);

	p_ac97_reg->gcr |= ZY_AC97_GCR_WARM_RESET_MSK;

	mdelay(5);

	/*power on all the necessary unit*/
	zy_ac97_acodec_write(p_device_context,POWERDOWN_CTRL_STAT, 0x000);/*26*/
	/* open left headphone mixer
	 * open right headphone mixer
	 * open right/left dac
	 * open right/left adc
	 * open temperature sensor
	 * enable reference generator
	 */
	zy_ac97_acodec_write(p_device_context,POWER_DOWN_1, 0xda00);	/*3c */
	/* open microphone bias
	 * open HPL output PGA
	 * open HPR output PGA
	 * open mic PGA MA
	 * open mic pre-amp MPA
	 */
	/* if here we enable SPKL and SPKR PGA, then Touch screen will doesn't
	 * work
	 */
	zy_ac97_acodec_write(p_device_context,POWER_DOWN_2,0xb9f5);   /*3e */

	/* recording route and microphone input
	 * microphone selection, now fixed to MIC1 input and mic bias output
	 * MIC1 only, MICBIAS enable
	 */
	zy_ac97_acodec_write (p_device_context, MIC_BIAS, 0xC440);	/*0x22h*/

	/* mic pga setting to mixer (side tone)
	 * comment the below code to make MICA/B play back volume gain + 0db
	 */
	/* zy_ac97_acodec_write (p_device_context, MIC_PGA_VOLUME, 0x0000); */	/*0x0eh*/

	/* recording side tone and ADC boost, now fixed to default (14h)
	 * recording volume  0dB
	 */
	zy_ac97_acodec_write(p_device_context, REC_PGA_VOL, 0x0); /*12*/

	/* hifi playback route and output mixer
	 * by default, fixed to enable headphone only
	 */

	/* comment the below code to make SPEAKER default MUTE */
	zy_ac97_acodec_write (p_device_context, SPEAKER_VOLUME, 0x0);		/*02*/

	/* comment the below code to make OUT3_OUT4 default MUTE */
	/*zy_ac97_acodec_write (p_device_context, OUT3_OUT4_VOLUME, 0x8000);*/    /*06*/

	/* remove all  the mute bit  volume gain + 0db */
	zy_ac97_acodec_write(p_device_context, HEADPHONE_VOLUME, 0x0);	/*04*/

	/* DAC route
	 * open DAC to headphone mixer path
	 * left DAC gain +0db
	 * right DAC gain +0db
	 */
	zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE,0x0808);	/*0c*/

	/* out3 configure, invert to HPMIXR */
	/* zy_ac97_acodec_write(p_device_context,DAC_3D_CTRL_INV_MUX_SEL, 0x8000); */	/*1e*/

	/* output control
	 * select HPMIXR HPMIXL out
	 * other out are all VIM
	 */
	zy_ac97_acodec_write(p_device_context,OUTPUT_PGA_MUX, 0x9BA8);	/*1c*/

	/* set sample rates
	 * enable variable rate conversion
	 */
	zy_ac97_acodec_write(p_device_context, EXTENDED_AUD_STAT_CTRL , 0x1); /*2a*/

	/* DAC 44kHZ */
	zy_ac97_acodec_write(p_device_context,AUDIO_DAC_RATE,0xac44); /*2c*/

	/* ADC 16KHZ */
	zy_ac97_acodec_write(p_device_context,AUDIO_ADC_RATE,0x3E80); /*32*/

	/* clock scheme, use external clock, it is 24MHZ from MCLK_A */

	/* the input Gain set to Max */
	zy_wm9713_set_master_input_gain(p_device_context,15);

	zy_wm9713_ear_speaker_enable(p_device_context);

#ifdef CONFIG_AC97_EXTCLK
	zy_ac97_acodec_read(p_device_context, MCLK_PLL_CTRL_1, &value);
	zy_ac97_acodec_write(p_device_context, MCLK_PLL_CTRL_1, value | 0x2);
#endif
	return ACODEC_SUCCESS;
}

static acodec_error_t  zy_wm9713_hifi_near_in_enable(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
   	 unsigned short value;

	/* turn DAC on, we don't change 26h setting. insteadly we use 3ch and
	 * 3eh to control specific power domain 3CH  bit 7 and 6
	 * power on left DAC and right DAC
	 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
 	value &= ~(ZY_AC97_9713_PWR_DACL | ZY_AC97_9713_PWR_DACR |ZY_AC97_9713_PWR_MBIAS);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	/* DAC route
	 * left DAC gain +0db
	 * right DAC gain +0db
	 */
	zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE, 0xe808);	/*0c*/

	return status;
}

acodec_error_t	zy_wm9713_hifi_stream_path_enable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* turn DAC on, we don't change 26h setting. insteadly we use 3ch and
	 * 3eh to control specific power domain 3CH  bit 7 and 6
	 * power on left DAC and right DAC
	 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
 	value &= ~(ZY_AC97_9713_PWR_DACL | ZY_AC97_9713_PWR_DACR |ZY_AC97_9713_PWR_MBIAS);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	/* DAC route
	 * open DAC to headphone mixer path
	 * left DAC gain +0db
	 * right DAC gain +0db
	 */
	zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE, 0x6808);	/*0c*/

	return status;
}
static acodec_error_t  zy_wm9713_voice_near_in_enable(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value &= ~ZY_AC97_9713_PWR_VXDAC;
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);
	return status;
}
static acodec_error_t zy_wm9713_close_voice_near_in(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= ZY_AC97_9713_PWR_VXDAC;
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);
	return status;
}

static acodec_error_t zy_wm9713_close_hifi_near_in(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mute DAC */
	zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE, 0xe808);	/*0c*/
	/* power DAC off, we don't change 26h setting. insteadly we use 3ch
	 * and 3eh to control specific power domain 3CH  bit 7 and 6
	 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_DACL | ZY_AC97_9713_PWR_DACR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	return status;
}
acodec_error_t	zy_wm9713_hifi_stream_path_disable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mute DAC */
	zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE, 0xe808);	/*0c*/
	/* power DAC off, we don't change 26h setting. insteadly we use 3ch
	 * and 3eh to control specific power domain 3CH  bit 7 and 6
	 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_DACL | ZY_AC97_9713_PWR_DACR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	return status;
}
static acodec_error_t zy_wm9713_open_hifi_near_out(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* turn ADC and MA pga on, we don't change 26h setting. insteadly we
	 * use 3ch and 3eh to control specific power domain 3CH  bit 5 and 4,
	 * 3eh bit 3 and 1
	 */
	/* open right/left ADC */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value &= ~(ZY_AC97_9713_PWR_ADCL | ZY_AC97_9713_PWR_ADCR |ZY_AC97_9713_PWR_MBIAS);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	/* select MICA into ADCL and ADCR */
	status = zy_ac97_acodec_write(p_device_context, REC_ROUTE_MUX_SEL, 0xc600);
	return status;
}

static acodec_error_t zy_wm9713_close_hifi_near_out(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* turn ADC and MA pga off, we don't change 26h setting. insteadly we
	 * use 3ch and 3eh to control specific power domain 3CH  bit 5 and 4,
	 * 3eh bit 3 and 1
	 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_ADCL | ZY_AC97_9713_PWR_ADCR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);
	return status;
}

acodec_error_t	zy_wm9713_recording_path_enable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* turn ADC and MA pga on, we don't change 26h setting. insteadly we
	 * use 3ch and 3eh to control specific power domain 3CH  bit 5 and 4,
	 * 3eh bit 3 and 1
	 */
	/* open right/left ADC */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value &= ~(ZY_AC97_9713_PWR_ADCL | ZY_AC97_9713_PWR_ADCR |ZY_AC97_9713_PWR_MBIAS);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	/* open mic PGA MA and mic pre-amp MPA */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value &= ~(ZY_AC97_9713_PWR_MA | ZY_AC97_9713_PWR_MPA);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	/* select MICA into ADCL and ADCR */
	status = zy_ac97_acodec_write(p_device_context, REC_ROUTE_MUX_SEL, 0x600);
	return status;
}

acodec_error_t	zy_wm9713_recording_path_disable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* turn ADC and MA pga off, we don't change 26h setting. insteadly we
	 * use 3ch and 3eh to control specific power domain 3CH  bit 5 and 4,
	 * 3eh bit 3 and 1
	 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_ADCL | ZY_AC97_9713_PWR_ADCR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value |= (ZY_AC97_9713_PWR_MA | ZY_AC97_9713_PWR_MPA);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t zy_wm9713_hpmixer_enable(acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* power 2ch; 3-2, 3eh: 10-9 */
	/* power on left/right headphone mix */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value &= ~(ZY_AC97_9713_PWR_HPLX | ZY_AC97_9713_PWR_HPRX);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);
	return status;
}

acodec_error_t zy_wm9713_hpmixer_disable(acodec_context_t *p_device_context)
{
	acodec_error_t status = ACODEC_SUCCESS;
	unsigned short value;

	/* power 3ch: 3-2, 3eh: 10-9 */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_HPLX | ZY_AC97_9713_PWR_HPRX);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);
	return status;
}

acodec_error_t	zy_wm9713_headset_enable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mixer route 0ch:15, this bit control belongs to hifipath, because
	 * on Zylonite all playback goes to headset mixer
	 */

	/* output control: 1ch:7-6 5-4 04h:15, 7 */
	/* select HPMIXR HPMIXL out */
	status = zy_ac97_acodec_read(p_device_context, OUTPUT_PGA_MUX, &value);
	value &= ~(0xf0);
	value |= 0xa0;
	status = zy_ac97_acodec_write(p_device_context, OUTPUT_PGA_MUX, value);

	/* remove mute bit of HPL/HPR */
	status = zy_ac97_acodec_read(p_device_context, HEADPHONE_VOLUME, &value);
	value &= ~(0x8080);
	status = zy_ac97_acodec_write(p_device_context, HEADPHONE_VOLUME, value);

	/* power on HPL/HPR output PGA */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value &= ~(ZY_AC97_9713_PWR_HPL | ZY_AC97_9713_PWR_HPR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t	zy_wm9713_headset_disable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mixer route 0ch:15, this bit control belongs to hifipath, because
	 * on Zylonite all playback goes to headset mixer
	 */

	/* output control: 1ch:7-6 5-4 04h:15, 7 */
	status = zy_ac97_acodec_read(p_device_context, OUTPUT_PGA_MUX, &value);
	value &= ~(0xf0);
	status = zy_ac97_acodec_write(p_device_context, OUTPUT_PGA_MUX, value);
	status = zy_ac97_acodec_read(p_device_context, HEADPHONE_VOLUME, &value);
	value |= 0x8080;
	status = zy_ac97_acodec_write(p_device_context, HEADPHONE_VOLUME, value);

	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value |= (ZY_AC97_9713_PWR_HPL | ZY_AC97_9713_PWR_HPR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t	zy_wm9713_ear_speaker_enable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* enable the below out putline
	 * HPMIXR -> INV1 ->OUT
	 * HPMIXL -> HPL
	 */

	/* mixer route 0ch:15, this bit control belongs to hifipath, because on
	 * Zylonite all playback goes to headset mixer
	 */

	/* out3 route: 1eh: 15-13 */
	status = zy_ac97_acodec_read(p_device_context, DAC_3D_CTRL_INV_MUX_SEL, &value);
	value &= ~(0xe000);
	value |= 0x8000;
	status = zy_ac97_acodec_write(p_device_context, DAC_3D_CTRL_INV_MUX_SEL, value);

	/* output control: 1ch:7-6 3-2, 06h: 7, 04h: 15 */
	status = zy_ac97_acodec_read(p_device_context, OUTPUT_PGA_MUX, &value);
	value &= ~(0xcc);
	value |= 0x88;
	status = zy_ac97_acodec_write(p_device_context, OUTPUT_PGA_MUX, value);

	status = zy_ac97_acodec_read(p_device_context, OUT3_OUT4_VOLUME, &value);
	value &= ~(0x80);
	status = zy_ac97_acodec_write(p_device_context, OUT3_OUT4_VOLUME, value);

	status = zy_ac97_acodec_read(p_device_context, HEADPHONE_VOLUME, &value);
	value &= ~(0x8000);
	status = zy_ac97_acodec_write(p_device_context, HEADPHONE_VOLUME, value);

	/* power on left/right mixer */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value &= ~(ZY_AC97_9713_PWR_HPLX | ZY_AC97_9713_PWR_HPRX);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	/* power on HPL output PGA */
	/* power on OUT3 output PGA */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value &= ~(ZY_AC97_9713_PWR_HPL | ZY_AC97_9713_PWR_OUT3);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t	zy_wm9713_ear_speaker_disable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mixer route 0ch:15, this bit control belongs to hifipath, because
	 * on Zylonite all playback goes to headset mixer
	 */

	/* out3 route: 1eh: 15-13 */
	status = zy_ac97_acodec_read(p_device_context, DAC_3D_CTRL_INV_MUX_SEL, &value);
	value &= ~(0xe000);
	status = zy_ac97_acodec_write(p_device_context, DAC_3D_CTRL_INV_MUX_SEL, value);

	/* output control: 1ch:7-6 3-2, 06h: 7, 04h: 15 */
	status = zy_ac97_acodec_read(p_device_context, OUTPUT_PGA_MUX, &value);
	value &= ~(0xcc);
	status = zy_ac97_acodec_write(p_device_context, OUTPUT_PGA_MUX, value);

	status = zy_ac97_acodec_read(p_device_context, OUT3_OUT4_VOLUME, &value);
	value |= (0x80);
	status = zy_ac97_acodec_write(p_device_context, OUT3_OUT4_VOLUME, value);

	status = zy_ac97_acodec_read(p_device_context, HEADPHONE_VOLUME, &value);
	value |= (0x8000);
	status = zy_ac97_acodec_write(p_device_context, HEADPHONE_VOLUME, value);

	/* power */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_HPLX | ZY_AC97_9713_PWR_HPRX);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value |= (ZY_AC97_9713_PWR_HPL | ZY_AC97_9713_PWR_OUT3);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t	zy_wm9713_louder_speaker_enable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mixer route 0ch:15, this bit control belongs to hifipath, because on
	 * Zylonite all playback goes to headset mixer
	 */

	/* output control: 1ch 13-11, 10-8, 02h: 15, 7
	 * HPMIXL->SPKL
	 * HPMIXR->SPKR
	 */
	status = zy_ac97_acodec_read(p_device_context, OUTPUT_PGA_MUX, &value);
	value &= ~(0x3f00);
	value |= 0x1200;
	status = zy_ac97_acodec_write(p_device_context, OUTPUT_PGA_MUX, value);

	/* remove mute bit of SPKR/SPKL */
	status = zy_ac97_acodec_read(p_device_context, SPEAKER_VOLUME, &value);
	value &= ~(0x8080);
	status = zy_ac97_acodec_write(p_device_context, SPEAKER_VOLUME, value);

	/* power on SPKL/SPKR output PGA */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value &= ~(ZY_AC97_9713_PWR_SPKL | ZY_AC97_9713_PWR_SPKR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t	zy_wm9713_louder_speaker_disable (acodec_context_t *p_device_context)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	/* mixer route 0ch:15, this bit control belongs to hifipath, because on
	 * Zylonite all playback goes to headset mixer
	 */

	/* output control: 1ch 13-11, 10-8, 02h: 15, 7 */
	status = zy_ac97_acodec_read(p_device_context, OUTPUT_PGA_MUX, &value);
	value &= ~(0x3f00);
	status = zy_ac97_acodec_write(p_device_context, OUTPUT_PGA_MUX, value);

	status = zy_ac97_acodec_read(p_device_context, SPEAKER_VOLUME, &value);
	value |= (0x8080);
	status = zy_ac97_acodec_write(p_device_context, SPEAKER_VOLUME, value);

	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &value);
	value |= (ZY_AC97_9713_PWR_SPKL | ZY_AC97_9713_PWR_SPKR);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, value);

	return status;
}

acodec_error_t	zy_wm9713_mic_enable (acodec_context_t *p_device_context, unsigned char mic_type)
{
	unsigned char val;

	/* open LDO13 (MIC_BIAS) */
	pxa3xx_pmic_read(ARAVA_REGCTRL1, &val);
	val |= 0x08;
	pxa3xx_pmic_write(ARAVA_REGCTRL1, val);

	/* mic input route: 22h: 13-12 */
	if (mic_type == HANDSET_MIC_MASK)
	{/* mic1 */
		zy_ac97_acodec_write (p_device_context, MIC_BIAS, 0x4040);	/*0x22h*/
	}
	else
	{/* mic2a */
		zy_ac97_acodec_write (p_device_context, MIC_BIAS, 0x5040);	/*0x22h*/
	}
	return ACODEC_SUCCESS;
}

acodec_error_t zy_wm9713_mic_disable (acodec_context_t *p_device_context, unsigned char mic_type)
{
	unsigned char val;

	/* shutdown LDO13 (MIC_BIAS) */
	pxa3xx_pmic_read(ARAVA_REGCTRL1, &val);
	val &= ~0x08;
	pxa3xx_pmic_write(ARAVA_REGCTRL1, val);
	return ACODEC_SUCCESS;
}

acodec_error_t zy_wm9713_set_headset_vlo(acodec_context_t *p_device_context, unsigned short gain_in_db)
 {
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;
	unsigned char left_volume, right_volume, vol;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	/*take the unit of gain_in_db as 1 db*/
	left_volume = (unsigned char)(gain_in_db * 10 / 15);
	if (left_volume > ZY_AC97_WM9713_MAX_VOLUME)
	{
		left_volume = ZY_AC97_WM9713_MAX_VOLUME;
	}
	right_volume = left_volume;
	vol = left_volume;
	if ((0 == left_volume) && (0 == right_volume))
	{
		if (!golden_board) {
			/*mute the HPL HPR*/
			value = ZY_AC97_WM9713_MVR_MM | (1<<7);
		}
	} else {
		/*translate into attenuation value*/
		left_volume = ZY_AC97_WM9713_MAX_VOLUME - left_volume;
		right_volume = ZY_AC97_WM9713_MAX_VOLUME - right_volume;
		value = left_volume << ZY_AC97_WM9713_MVR_ML_SHIFT;
		value |= right_volume;
		status = zy_ac97_acodec_write(p_device_context, HEADPHONE_VOLUME, value);
		if (status == ACODEC_SUCCESS) 
   			current_headset_vol = vol * 15/10;
   	}
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return status;
 }


acodec_error_t zy_wm9713_get_vol(acodec_context_t *p_device_context, vol_port_type_t port,  unsigned short *gain_in_db)
{

	switch(port){
		case MIC1:
		case MIC2:
			*gain_in_db = current_mast_input_gain;
			break;
		case HEADSET:
			*gain_in_db = current_headset_vol;
			break;
		case SPEAKER:
			*gain_in_db = current_speaker_vol;
			break;
		default:
			return ACODEC_FEATURE_NO_SUPPORTED;
	}
	return ACODEC_SUCCESS;
}

acodec_error_t zy_wm9713_set_vol(acodec_context_t *p_device_context, vol_port_type_t port,  unsigned short gain_in_db)
{
	acodec_error_t status = ACODEC_SUCCESS;
	/* the DB value is 1.5 step,
	 * if the gain_in_db == 1 DB will set as 0
	 * So set DB as 2 at this situation
	 */
	if (golden_board) {
		if (gain_in_db == 0) {
			/* mute */
			pr_debug("Let all path as shutdown\n");
			golden_muted = 1;
			return status;
		} else if (golden_muted) {
			/* unmute */
			pr_debug("Let all path as rework\n");
			golden_muted = 0;
		}
	}
	if(gain_in_db == 1){
		gain_in_db = 2;
	}

	switch(port){
		case MIC1:
		case MIC2:
			status = zy_wm9713_set_master_input_gain(p_device_context,gain_in_db);
			break;
		case HEADSET:
			status = zy_wm9713_set_headset_vlo(p_device_context,gain_in_db);
			break;
		case SPEAKER:
			status = zy_wm9713_set_master_vol(p_device_context, gain_in_db);
			break;
		default:
			status = ACODEC_FEATURE_NO_SUPPORTED;
	}
	return status;
}

static acodec_error_t  zy_wm9713_save_context (acodec_context_t *p_device_context, ZY_9713_CONTEXT_SAVE_T *p_codec_context)
{
	unsigned short value;
	unsigned short ii, jj, kk;

	jj = kk = 0;
	for (ii = 0; ii < 128; ii += 2)
	{
		if (ii == zy_9713_unsave[jj])
		{
			jj++;
			continue;
		}
		zy_ac97_acodec_read(p_device_context, ii, &value);
		p_codec_context->wm9713RegisterContext [kk++] = value;
	}
	return ACODEC_SUCCESS;
}

static acodec_error_t  zy_wm9713_restore_context (acodec_context_t *p_device_context, ZY_9713_CONTEXT_SAVE_T *p_codec_context)
{	/* check later: need special order to restore codec register? */
	unsigned short value;
	unsigned short ii, jj, kk;

	jj = kk = 0;
	for (ii = 0; ii < 128; ii += 2)
	{
		if (ii == zy_9713_unsave[jj])
		{
			jj++;
			continue;
		}
		value = p_codec_context->wm9713RegisterContext [kk++];
		zy_ac97_acodec_write(p_device_context, ii, value);
	}
	return ACODEC_SUCCESS;
}

/*After audio_codec boot up, it should be in sleep mode to save power.
 *So firstly, application should wake up audio codec
 *first_time used to show if it is the first time to wake up audio codec
 *if it is the first time, codec registers need not to be restored.
 *static int first_time = 1;
 *static acodec_state_t wm9713_codec_state =ZY_CODEC_SLEEP;

 *---------------------------------------------------------------------------------------------------------------
 * Function: zy_wm9713_sleep
 * Purpose: sleep the codec
 * Parameter:
 * Returns: acodec_error_t
 *operation: 1. make codec go to sleep
 *note: this function should substitutes zy_wm9713_save_context()
 *---------------------------------------------------------------------------------------------------------------
 */
acodec_error_t zy_wm9713_sleep(acodec_context_t *p_device_context)
{
	unsigned short value;
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	zy_wm9713_save_context(p_device_context, (ZY_9713_CONTEXT_SAVE_T *)(p_device_context->p_save_memory));
	/* make the codec into sleep mode */
	zy_ac97_acodec_read(p_device_context, POWERDOWN_CTRL_STAT, &value);
 	value |= 0x1000 ;
	zy_ac97_acodec_write(p_device_context, POWERDOWN_CTRL_STAT, value);
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
    	return ACODEC_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------------------------
 *  Function: zy_wm9713_wakeup
 *  Purpose:  wake up the codec and restore it to the original state
 *  Parameter:
 *  Returns:  acodec_error_t
 * operation: 1. warm reset to enabled the ACLINK
 *                 2. restore the state
 * note: this function should substitutes zy_wm9713_restore_context()
 * ---------------------------------------------------------------------------------------------------------------
 */
acodec_error_t zy_wm9713_wakeup(acodec_context_t *p_device_context)
{
	p_zy_ac97acodec_t p_ac97_reg = (p_zy_ac97acodec_t)(p_device_context->p_ctrl_reg);
	/* warm reset AClink to wake up the codec */
	p_ac97_reg->gcr |= ZY_AC97_GCR_WARM_RESET_MSK;
	mdelay(5);
	/* USE POWER_DOWN_1(2) to manage power
	 * zy_ac97_acodec_write(p_device_context, POWERDOWN_CTRL_STAT, 0x0);
	 */
	zy_wm9713_restore_context (p_device_context, (ZY_9713_CONTEXT_SAVE_T *)(p_device_context->p_save_memory));
	return ACODEC_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------------------------
 *  Function: zy_wm9713_specific_deinit
 *  Purpose: close all the power units and sleep codec
 *  Parameter:
 *  Returns:  0 do not support the control
 *               >0 support the control
 * operation: 1. save the state
 *                 2. make codec go to sleep
 * note: this function should substitutes zy_wm9713_save_context()
 * ---------------------------------------------------------------------------------------------------------------
 */
acodec_error_t zy_wm9713_specific_deinit (acodec_context_t *p_device_context)
{	/* do later: shut down all power */
	unsigned short value = 0;

	/* close the power of all units */
	zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, 0xffff);
	zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, 0xffff);
	zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
 	value &= ~(ZY_AC97_9713_PWR_MBIAS);
	zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	return ACODEC_SUCCESS;
}

static acodec_error_t support_scenario(unsigned short *rout_map)
{
	if(rout_map[INDEX_BT_HIFI_NEAR_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;

/*
	if(rout_map[INDEX_VOICE_NEAR_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;
*/
	if(rout_map[INDEX_BT_VOICE_NEAR_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_BT_MIC_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;

/*	if(rout_map[INDEX_MIC2_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;
*/

	if(rout_map[INDEX_FAR_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_MIDI_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_FM_IN_CONTROL])
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_HIFI_NEAR_IN_CONTROL] &
	~(STEREO_HEAD_SET_OUT_ROUTE|HIFI_NEAR_OUT_ROUTE|VOICE_NEAR_OUT_ROUTE|SPEAKER_OUT_ROUTE))
		return ACODEC_ROUTE_NO_SUPPORTED;

	if(rout_map[INDEX_MIC1_IN_CONTROL]&
	~(STEREO_HEAD_SET_OUT_ROUTE|HIFI_NEAR_OUT_ROUTE|VOICE_NEAR_OUT_ROUTE ))
		return ACODEC_ROUTE_NO_SUPPORTED;
	return ACODEC_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------------------------
 *  Function: zy_wm9713_set_route
 *  Purpose:  set the codec route
 *  Parameter:
 *  			rout_map__ the route map want to set
 * 			current_map__ the current rout map
 *  explain:   rout_map and current_map
 *                This function use the state of input devices state and ouput devices state to
 *                to show the codec rout map. More information please refer to spec
 *                [linux-2.6.9-zylonite-alpha2] [ALSA] Subsystem High Level Design.doc
 *                Every bit of the rout_map/current_map show one input/output device state.
 *                1 means open; 0 means close. The bit NO is the device index.
 *  Returns: ACODEC_SUCCESS
 *               ACODEC_ROUTE_NO_SUPPORTED
 * Operation: Function will first scan the supported route map to find that if rout_map is be supported. If the codec does not support the
 *                 route return ACODEC_ROUTE_NO_SUPPORTED; If the route can be supported. set the route according the rout_map,
 *                 current_map can be used to speed the route set process. If some route is the same as before,  then this route need not set again.
 *
 * ---------------------------------------------------------------------------------------------------------------
 */
acodec_error_t zy_wm9713_set_route(acodec_context_t *p_device_context, unsigned short *rout_map, unsigned short* current_map)
{
	unsigned short  hifi_near_in_change_state,voice_near_in_change_state;
	unsigned short  hifi_near_in, voice_near_in, current_out_state;
	unsigned short  out_state, rout_out_state;
	unsigned short  mic2_in, mic1_in, out_change_state;
	unsigned short  val = 0;
	int mic1_enable = 0;

	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	if(ACODEC_SUCCESS != support_scenario(rout_map)) {
		return(ACODEC_ROUTE_NO_SUPPORTED) ;
	}

	hifi_near_in = rout_map[INDEX_HIFI_NEAR_IN_CONTROL];
	voice_near_in = rout_map[INDEX_VOICE_NEAR_IN_CONTROL];

	mic1_in = rout_map[INDEX_MIC1_IN_CONTROL];
	mic2_in = rout_map[INDEX_MIC2_IN_CONTROL];

	hifi_near_in_change_state = rout_map[INDEX_HIFI_NEAR_IN_CONTROL]
			 ^ current_map[INDEX_HIFI_NEAR_IN_CONTROL];
	voice_near_in_change_state = rout_map[INDEX_VOICE_NEAR_IN_CONTROL]
			 ^ current_map[INDEX_VOICE_NEAR_IN_CONTROL];

	if( mic1_in | mic2_in ){
		/* open mic PGA MA and mic pre-amp MPA */
    		zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &val);
    		val &= ~(ZY_AC97_9713_PWR_MA | ZY_AC97_9713_PWR_MPA);
    		zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, val);
	}else {
		/* close mic PGA MA and mic pre-amp MPA */
		zy_ac97_acodec_read(p_device_context, POWER_DOWN_2, &val);
    		val |= (ZY_AC97_9713_PWR_MA | ZY_AC97_9713_PWR_MPA);
   		zy_ac97_acodec_write(p_device_context, POWER_DOWN_2, val);
	}
	if(mic1_in){
		/* open Mic1 */
		mic1_enable = 1;
		zy_wm9713_mic_enable(p_device_context, HANDSET_MIC_MASK);
	}else{
		/* close Mic1 */
		mic1_enable = 0;
		zy_wm9713_mic_disable(p_device_context, HANDSET_MIC_MASK);
	}
	if(!mic1_enable){
		if(mic2_in){
			zy_wm9713_mic_enable(p_device_context, HEADSET_MIC_MASK);
		}
		else{
			zy_ac97_acodec_read(p_device_context, REC_ROUTE_MUX_SEL, &val);
			val &= 0xffc0;
    			zy_ac97_acodec_write(p_device_context, REC_ROUTE_MUX_SEL, val);
		}
	}else
	{
		/* close the route */
	}

	if(hifi_near_in_change_state){
		if(current_map[INDEX_HIFI_NEAR_IN_CONTROL] && !rout_map[INDEX_HIFI_NEAR_IN_CONTROL]){
			/* from on to off */
			zy_wm9713_close_hifi_near_in(p_device_context);
		}else if(!current_map[INDEX_HIFI_NEAR_IN_CONTROL] && rout_map[INDEX_HIFI_NEAR_IN_CONTROL] )
		{
			/* from off to on */
			zy_wm9713_hifi_near_in_enable(p_device_context);
		}
	}/* end hifi_near_in_change_state */

	if(voice_near_in_change_state){
		if(current_map[INDEX_VOICE_NEAR_IN_CONTROL] && !rout_map[INDEX_VOICE_NEAR_IN_CONTROL]){
			/* from on to off */
			zy_wm9713_close_voice_near_in(p_device_context);
		}else if(!current_map[INDEX_VOICE_NEAR_IN_CONTROL] && rout_map[INDEX_VOICE_NEAR_IN_CONTROL] )
		{
			/* from off to on */
			zy_wm9713_voice_near_in_enable(p_device_context);
		}
	}/* end hifi_near_in_change_state */
	/* manage ouput device */
	out_state = rout_map[INDEX_HIFI_NEAR_IN_CONTROL] |rout_map[INDEX_MIC1_IN_CONTROL]|rout_map[INDEX_MIC2_IN_CONTROL]| rout_map[INDEX_VOICE_NEAR_IN_CONTROL];
	current_out_state = current_map[INDEX_HIFI_NEAR_IN_CONTROL] |current_map[INDEX_MIC1_IN_CONTROL] | current_map[INDEX_MIC2_IN_CONTROL]|current_map[INDEX_VOICE_NEAR_IN_CONTROL];
	rout_out_state = rout_map[INDEX_HIFI_NEAR_IN_CONTROL] |rout_map[INDEX_MIC1_IN_CONTROL] | rout_map[INDEX_MIC2_IN_CONTROL]|rout_map[INDEX_VOICE_NEAR_IN_CONTROL];

	out_change_state = out_state ^ current_out_state;

	if(out_change_state & STEREO_HEAD_SET_OUT_ROUTE) {
		if(current_out_state & STEREO_HEAD_SET_OUT_ROUTE ){
			/* change from open to close */
			zy_wm9713_headset_disable(p_device_context);
		}else {
			/* change from close to open */
			zy_wm9713_headset_enable(p_device_context);
		}

	}

	if(out_change_state & SPEAKER_OUT_ROUTE) {
		if(current_out_state & SPEAKER_OUT_ROUTE) {
			zy_wm9713_louder_speaker_disable(p_device_context);
		}else {
			zy_wm9713_louder_speaker_enable(p_device_context);
		}
	}

	if(out_change_state & (STEREO_HEAD_SET_OUT_ROUTE|SPEAKER_OUT_ROUTE)){
		if (out_state & (STEREO_HEAD_SET_OUT_ROUTE|SPEAKER_OUT_ROUTE)){
			zy_wm9713_hpmixer_enable(p_device_context);
		}else {
			zy_wm9713_hpmixer_disable(p_device_context);
		}
	}

	if(out_change_state & (HIFI_NEAR_OUT_ROUTE | VOICE_NEAR_OUT_ROUTE) ) {
		if((current_out_state & (HIFI_NEAR_OUT_ROUTE | VOICE_NEAR_OUT_ROUTE)) && !( rout_out_state & (HIFI_NEAR_OUT_ROUTE | VOICE_NEAR_OUT_ROUTE)  )  ){
			/* change from open to close */
			zy_wm9713_close_hifi_near_out(p_device_context);
		}else {
			/* change from close to open */
			zy_wm9713_open_hifi_near_out (p_device_context);
		}

	}
	/* manage path */
	if ((mic1_in | mic2_in) && (HIFI_NEAR_OUT_ROUTE | VOICE_NEAR_OUT_ROUTE))	{
		/* Mic1 to hifi_near and voice_near */
		unsigned short  tem=0;
		zy_ac97_acodec_read(p_device_context, REC_ROUTE_MUX_SEL, &tem);
		tem &= 0xffc0;
    		zy_ac97_acodec_write(p_device_context, REC_ROUTE_MUX_SEL, tem);
	} else {
	}
	if (mic1_in & STEREO_HEAD_SET_OUT_ROUTE) {
		/* only when mic1 enable the loop back will be enable.
		 * Open Mic1 to head set
		 */
		zy_ac97_acodec_read(p_device_context,REC_ROUTE_MUX_SEL,&val);
       		val &= 0x3fff;
       		zy_ac97_acodec_write(p_device_context,REC_ROUTE_MUX_SEL,val);
	} else {
		/* Close Mic1 to head set */
		zy_ac97_acodec_read(p_device_context,REC_ROUTE_MUX_SEL,&val);
        	val |= 0xc000;
       	 	zy_ac97_acodec_write(p_device_context,REC_ROUTE_MUX_SEL,val);
	}


	if(hifi_near_in & (STEREO_HEAD_SET_OUT_ROUTE | SPEAKER_OUT_ROUTE)){
		/* open hifi_near to head set */
		zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE, 0x6808);
	} else {
		/* close hifi_near to head set */
		zy_ac97_acodec_write(p_device_context, DAC_PGA_VOL_ROUTE, 0xe808);
	}

	if(voice_near_in & STEREO_HEAD_SET_OUT_ROUTE ){
		/* open voice_near to head set */
		unsigned short reg;
		zy_ac97_acodec_read(p_device_context, VXDAC_VOLUME_ROUTE, &reg);
		reg &= ~0x8000;
		zy_ac97_acodec_write(p_device_context, VXDAC_VOLUME_ROUTE, reg);
	} else {
		/* close voice_near to head set */
		unsigned short reg;
		zy_ac97_acodec_read(p_device_context, VXDAC_VOLUME_ROUTE, &reg);
		reg |= 0x8000;
		zy_ac97_acodec_write(p_device_context, VXDAC_VOLUME_ROUTE, reg);
	}
	set_codec_sub_state(codec_client, CODEC_SUB_LOWPOWER);
	return ACODEC_SUCCESS;
}


acodec_error_t zy_acodec_set_pen_down_interrupt(acodec_context_t *p_device_context, int enable)
{	/* disable/enable pen down interrupt in the codec. This function is not
	 * implemented for Wm9713 because the pen down detection could not be
	 * disabled in codec
	 */
	return ACODEC_SUCCESS;
}

acodec_error_t zy_wm9713_enable_touch(acodec_context_t *p_device_context)
{	/* enable touch functionality in the codec */
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	/* power setting */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value &= ~(ZY_AC97_9713_PWR_PADCPD);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);

	/* basic touch setting */
	status = zy_ac97_acodec_write(p_device_context, DIGITIZER_3_WM13, 0xc008);
	status = zy_ac97_acodec_write(p_device_context, DIGITIZER_2_WM13, 0x6);

	/* 9713 powerdown virtual gpio setting (polarity, sticky, wakeup)
	 * 9713 gpio 2(pin45) route to IRQ
	 * Notes: Can use defaults for IRQ polarity, PENDOWN polarity in IRQ,
	 * sticky for PENDOWN in IRQ and wakeup for PENDOWN.
	 */
	status = zy_ac97_acodec_read(p_device_context, GPIO_PIN_CFG, &value);
	value &= ~(0x4);
	status = zy_ac97_acodec_write(p_device_context, GPIO_PIN_CFG, value);

	status = zy_ac97_acodec_read(p_device_context, GPIO_PIN_SHARING, &value);
	value &= ~(0x4);
	status = zy_ac97_acodec_write(p_device_context, GPIO_PIN_SHARING, value);

	status = zy_ac97_acodec_read(p_device_context, GPIO_PIN_WAKEUP, &value);
	value |= (0x2000);
	status = zy_ac97_acodec_write(p_device_context, GPIO_PIN_WAKEUP, value);

	status = zy_ac97_acodec_read(p_device_context, GPIO_PIN_STICKY, &value);
	value |= (0x2000);
	status = zy_ac97_acodec_write(p_device_context, GPIO_PIN_STICKY, value);

	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	return status;
}

acodec_error_t zy_wm9713_disable_touch(acodec_context_t *p_device_context)
{	/* disable touch functionality in the codec */
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	/* power setting */
	status = zy_ac97_acodec_read(p_device_context, POWER_DOWN_1, &value);
	value |= (ZY_AC97_9713_PWR_PADCPD);
	status = zy_ac97_acodec_write(p_device_context, POWER_DOWN_1, value);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	return status;
}

acodec_error_t zy_acodec_get_adc_sample(acodec_context_t *p_device_context, unsigned short *p_sample_data, unsigned short adc_type, int *p_pen_down)
{
	acodec_error_t	status = ACODEC_SUCCESS;
	unsigned short value;
	unsigned long wait;

	if (adc_type == ZY_TOUCH_SAMPLE_X) {
		value = 0x202;
	} else {/* Y sample */
		value = 0x204;
	}

	status = zy_ac97_acodec_write(p_device_context, DIGITIZER_1_WM13, value);

	wait = 0;
	do {
		status = zy_ac97_acodec_read(p_device_context, DIGITIZER_1_WM13, &value);
		if ( !(value & 0x200 ) ) {
			break;
		}
	} while ( 100 > wait++ );

	status = zy_ac97_acodec_read(p_device_context, DIGITIZER_READ_BACK, &value);
	if (value & 0x8000) {	/* means pen down */
		*p_pen_down = 1;
	} else {
		*p_pen_down = 0;
	}
	*p_sample_data = value & 0xfff;

	return status;
}

