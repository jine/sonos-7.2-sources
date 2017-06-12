/*
   Copyright (C) 2006, Marvell Corporation.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 */ 

/*****************************************************************************
 * Abstract:  
 * 	contains all hardware related functions for OV5623
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/ 

#include "camera.h"
#include "ov5623_hw.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/hardware.h>

#define OV5623_DEBUG
#undef OV5623_DEBUG

#ifdef  OV5623_DEBUG
#define ov_dbg(fmt, arg...)    printk(KERN_INFO "%s(line %d): " fmt "\n",  \
                __FUNCTION__, __LINE__, ##arg)
#else 
#define ov_dbg(fmt, arg...)        do {} while (0)
#endif

#define DENOISE		1
#define	WB_CORRECTION	1
#define FREQUENCE50     1       /* banding filter for 50HZ 60HZ */

#undef	VFLIP
#undef	HMIRROR
/* #define VFLIP	1 */
/* #define HMIRROR	1 */

extern int platform_id(void);

/*****************************************************************************
 *  
 *****************************************************************************/
u32 field_pclk =CICLK;
u32 field_max_line =617;
u32 field_max_pixel =1300;

u8 field_gain = 0 ;
u32 field_exp = 0 ; 
u8 field_blueg = 0;
u8 field_redg = 0; 
		   

/*****************************************************************************
 *  Register Settings, Get from OmniVision
 ****************************************************************************/

const static u8 ov5623InitSetting[]=
{
	
	OV5623_COM8,     0x00,        
	OV5623_AEC,      0x1E,
	0x3B,		 0x07,

	0x5B,		 0x40,
	0x39,		 0x07,
	0x53,		 0x02,
	0x54,		 0x40,
	OV5623_REG04,    0x20,
	0x27,		 0x04,
	0x3D,		 0x40,

	0x36,		 0x00,
	0xC5,		 0x04,

	0x4E,		 0x00,
	0x4F,		 0x93,
	0x50,		 0x7B,
	0xCA,		 0x0C,
	0xCB,		 0x0F,

	0x39,		 0x07,

	0x4A,		 0x10,

	0x3E,		 0x09,
	0x3D,		 0x00,


#ifdef  HMIRROR
        #ifdef VFLIP
                0x04,   0xF0,
        #else
                0x04,   0xA0,
        #endif
#endif

#ifdef VFLIP
        #ifdef HMIRROR
                0x04,   0xF0,
        #else
                0x04,   0x70,
        #endif
#endif

#ifdef	WB_CORRECTION
	OV5623_DSP01,	 0x07,
#endif

	OV5623_REGEND,   0x00
};

const static u8 ov5623OtherSetting[]=
{
	OV5623_DSP09,	 0x20,
	OV5623_DGCTRL,	 0x80,
	0xB7,		 0x9D,
	0xB6,		 0x11,
	0xB5,		 0x55,
	0xB4,		 0x00,
	OV5623_TOPLMT,	 0xF0,
	OV5623_BOTLMT,	 0x0A,
	OV5623_REDLMT,	 0xF0,
	OV5623_GREENLMT, 0xF0,
	OV5623_BLUELMT,  0xF0,

        OV5623_COM8,     0xE7,
	OV5623_COM9,	 0x60,

	
	0x33,		 0x75,
	0x2C,		 0x00,
	OV5623_COM2,	 0x00,
	0x35,		 0x30,
	0x27,		 0x04,
	0x3C,		 0x07,	
	0x3A,		 0x0A,
	0x3B,		 0x07,
	OV5623_BLUE,	 0x40,
	OV5623_RED,	 0x40,
	OV5623_GREEN,	 0x40,
	0x52,		 0xB0,

	0x51,		 0x83,

	0x21,		 0xAB,	
	0x22,		 0x05,	
	0x23,		 0x00,	
	0x35,		 0x30,  

	0x20,		 0x90,
	0x28,		 0x30,
	0x73,		 0xE1,
	0x6C,		 0x00,
	0x6D,		 0x80,
	0x6E,		 0x00,
	0x70,		 0x04,
	0x71,		 0x00,	
	0x8D,		 0x04,
	0x64,		 0x00,
	0x65,		 0x00,
	0x66,		 0x00,
	0x67,		 0x00,
	0x68,		 0x00,	
	0x69,		 0x00,
	0x6A,		 0x00,
	0x6B,		 0x00,

	0xC5,		 0x03,

	0x71,		 0x93, 

	0x74,		 0x20,	
	0x8D,		 0x44,

	OV5623_DSPEN,	 0x09,
	OV5623_DSPCTRL,	 0xc0,

	0xD2,		 0x00,
	0xD3,		 0x00,
	0xD4,		 0x00,
	0xD5,		 0x00,
	0xD6,		 0x00,
	0xD7,		 0x00,


	OV5623_REGEND,   0x00
};

const static u8 ov5623_full[]=
{
	0x11,	0x00,
	0x12,	0x00,
	0x2a,	0x00,
	0x2b,	0x00,
	0x0c,	0x38,
	0x0d,	0x06,
	
	0x4e,	0x00,
	0x4f,	0x93,
	0x50,	0x7b,
	0xca,	0x0c,
	0xcb,	0x0f,

	0x17,	0x12,
	0x18,	0xb4,
	0x19,	0x01,
	0x1a,	0xf4,
	0x03,	0x4a,

	0x48,	0x08,
	0x46,	0x30,
	0x32,	0x00,
	0x53,	0x02,
	0x54,	0x00,

	0x60,	0x00,
	0x61,	0x00,
	0x38,	0x90,

	0x37,	0x82,
	0x44,	0x48,

	0x21,	0xab,
	0x22,	0x05,
	0x23,	0x00,
	0x35,	0x30,

	0x8d,	0x44,
	
	OV5623_REGEND,   0x00
};

const static u8 ov5623_sxga[]=
{
	0x11,	0x00,
	0x12,	0x40,
	0x2a,	0x10,
	0x2b,	0x86,
	0x0c,	0x0a,
	0x0d,	0xc6,

	0x4e,	0x10,
	0x4f,	0x24,
	0x50,	0xf4,
	0xca,	0x02,
	0xcb,	0x03,

	0x17,	0x12,
	0x19,	0x01,
	0x03,	0x42,
	
	0x48,	0x09,
	0x46,	0x11,

	0x60,	0x00,
	0x61,	0x00,
	0x38,	0x10,

	0x37,	0x80,
	0x44,	0x48,

	0x21,	0xab,
	0x22,	0x05,
	0x23,	0x00,	
	0x35,	0x30,

	0x8d,	0x64,

	OV5623_REGEND,   0x00
};

const static u8 ov5623_d1md[]=
{
	0x11,	0x40,
	0x12,	0x20,
	0x2a,	0x10,
	0x2b,	0xc4,
	0x0c,	0x0a,
	0x0d,	0x27,

	0x4e,	0x00,
	0x4f,	0xb8,
	0x50,	0x9a,
	0xca,	0x02,
	0xcb,	0x03,

	0x17,	0x11,
	0x19,	0x01,
	0x03,	0x42,
	
	0x48,	0x08,
	0x46,	0x11,

	0x32,	0x06,
	0x53,	0x02,
	0x54,	0x00,

	0x60,	0x00,
	0x61,	0x00,
	0x38,	0x10,

	0x37,	0x80,
	0x44,	0xc0,

	0x21,	0xbb,
	0x22,	0x15,
	0x23,	0x03,	
	0x35,	0x38,

	0x8d,	0x64,

	OV5623_REGEND,   0x00
};


const static u8 gSensorSlaveAddr = 0x30;
static int read_sensor_reg( const u8 subAddress, u8 *bufP );
static int write_sensor_reg( const u8 subAddress, u8 *bufP );

/*****************************************************************************
 *  Private/helper api
 *****************************************************************************/
#ifdef DEBUG_PARAM_CHECK
static int get_reg_value( u8 *regP, u8 regAddr, u8 *regValueP )
{
	unsigned int index = 0;
	u8 curReg = 0;

	while( curReg < OV5623_REGEND )
	{
		curReg = regP[index << 1];
		if( curReg == regAddr )
		{
			*regValueP = regP[(index << 1) + 1];
			return 0;
		}    
		index ++;
	} 

	return -EIO;

}

static int set_reg_value( u8 *regP, u8 regAddr, u8 regValue )
{
	unsigned int index = 0;
	u8 curReg = 0;

	while( curReg < OV5623_REGEND )	{
		curReg = regP[index << 1];
		if( curReg == regAddr )	{
			regP[(index << 1) + 1] = regValue;
			return 0;
		}    
		index ++;
	} 

	return -EIO;

}
#endif

#ifdef CONFIG_MACH_ZYLONITE
static void sensor_power_on(void)
{
	pxa3xx_gpio_set_direction(MFP_CIF_HI_PWDN_GPIO, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_CIF_HI_PWDN_GPIO, GPIO_LEVEL_LOW);
}

static void sensor_power_off(void)
{
	pxa3xx_gpio_set_level(MFP_CIF_HI_PWDN_GPIO, GPIO_LEVEL_HIGH);
	pxa3xx_gpio_set_direction(MFP_CIF_HI_PWDN_GPIO, GPIO_DIR_IN);
}
#elif CONFIG_MACH_LITTLETON
static void sensor_power_on(void)
{
	int val;
	u8 technology_boards;

	if (platform_id() == 0x0) /* Littleton 1.x */
	{
		technology_boards=Technology_board_detect();
		if (technology_boards & CAMERA_BOARD){
			val = Camera_board_read_max7321();
			val = 0x8E;
			Camera_board_write_max7321(val);
			val = Camera_board_read_max7321();
		}
	 	if (technology_boards & WLAN8385CAMERA_BOARD){
 			val = wlan8385camera_board_read_max7321();
 			val &= ~WLAN8385CAMERA_BOARD_CAMERA_MASK;
			/*
			 * P3: PWDR  H active
			 * P2: RESET L active
			 * 0b0100
			 */
	
	  		val |= 0x84;
 			wlan8385camera_board_write_max7321(val);
 			val = wlan8385camera_board_read_max7321();
 		}

		/*8688*/
	 	if (technology_boards & BTWLANCAMERA_BOARD){
 			val = btwlancamera_board_read_max7321();
 			val &= ~WLAN8385CAMERA_BOARD_CAMERA_MASK;
			/*
			 * P2: PWDR  H active
			 * P3: RESET L active
			 * 0b1000
			 */
  			val |= 0x88; /**/
	 		btwlancamera_board_write_max7321(val);
 			val = btwlancamera_board_read_max7321();
 		}
		/*8686*/
	 	if (technology_boards & WLAN8686CAMERA_BOARD){
			val = wlan8686camera_board_read_max7321();
			val &= ~WLAN8686CAMERA_BOARD_CAMERA_MASK;
			/*
			 * P2: PWDR  H active
			 * P3: RESET L active
			 * 0b1000
			 */
			val |= 0x88; //0x84;
			wlan8686camera_board_write_max7321(val);
			val = wlan8686camera_board_read_max7321();
		}
	}
}

static void sensor_power_off(void)
{
	int val = 0;
	u8 technology_boards;

        if (platform_id() == 0x0) /* Littleton 1.x */
        {
		read_sensor_reg( OV5623_COM4, &val );
		val &= ~0x06;
		write_sensor_reg( OV5623_COM4, &val);

		mdelay(150);

		technology_boards=Technology_board_detect();
		if (technology_boards & CAMERA_BOARD){
			val = Camera_board_read_max7321();
			val &= ~0x01;
			Camera_board_write_max7321(val);
		}
	 	if (technology_boards & WLAN8385CAMERA_BOARD){
 			val = wlan8385camera_board_read_max7321();
 			val &= ~WLAN8385CAMERA_BOARD_CAMERA_MASK;
	  		val |= 0x8C;
 			wlan8385camera_board_write_max7321(val);
 		}
		/*8688*/
	  	if (technology_boards & BTWLANCAMERA_BOARD){
 			val = btwlancamera_board_read_max7321();
 			val &= ~WLAN8385CAMERA_BOARD_CAMERA_MASK;
	  		val |= 0x8C;
 			btwlancamera_board_write_max7321(val);
	 	}
		/*8686*/
	 	if (technology_boards & WLAN8686CAMERA_BOARD){
			val = wlan8686camera_board_read_max7321();
			val &= ~WLAN8686CAMERA_BOARD_CAMERA_MASK;
			val |= 0x8C;
			wlan8686camera_board_write_max7321(val);
		}
  	}
}

#endif


/*****************************************************************************
 *  Sensor read/write 
 *****************************************************************************/

static int rmw_sensor_reg(const u8 subAddress, u8 *bufP, u8 andMask, u8 orMask)
{
	int status;
	status = read_sensor_reg( subAddress, bufP );
	if (!status) {
		*bufP &= andMask;
		*bufP |= orMask;
		status = write_sensor_reg( subAddress, bufP );
	}
	return status;
}

int ov5623hw_read_sensor_reg( const u8 subAddress, u8 *bufP )
{
	return read_sensor_reg(subAddress, bufP);
}

int ov5623hw_write_sensor_reg( const u8 subAddress, u8 *bufP )
{
	return write_sensor_reg(subAddress, bufP);
}

int ov5623hw_set_regs( const u8 *regP )
{
	u32 curReg = 0;
	int    status = 0;

	/* The list is a register number followed by the value */
	while( regP[curReg << 1] < OV5623_REGEND )
	{
		u8 regVal = regP[(curReg << 1) + 1];

		status = (write_sensor_reg( regP[curReg << 1], &regVal ) == 0)?
			0 : -EIO;

		if( curReg == 0 )
			ov5623hw_wait( 5 );

		curReg++;
	}
	

	return status;
}

int ov5623hw_read_all_regs( u8 *bufP, u32 numRegs )
{
	int curReg;

	for( curReg = 0; curReg < numRegs; curReg++, bufP++ )
		read_sensor_reg( (u8)curReg, bufP );


	return 0;
}

/*****************************************************************************
 *  Power & Reset
 *****************************************************************************/
void ov5623hw_power_down( u8 powerMode )
{
	/* OV5623 PWRDWN, 0 = NORMAL, 1=POWER DOWN */
	if( powerMode == CAMERA_POWER_OFF )
		sensor_power_off();
	else
		sensor_power_on();

	mdelay(100);
}

void ov5623hw_reset( )
{
	int val =0x80;
	ov_dbg("ov5623_hw_reset().\n");
	write_sensor_reg(0x12,&val);
	mdelay(5);
	write_sensor_reg(0x12,&val);
	mdelay(5);
	return;
}

void ov5623hw_wait( int ms )
{
	mdelay( ms );
}


/*****************************************************************************
 *  Settings
 *****************************************************************************/
int ov5623hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision)
{
	read_sensor_reg( OV5623_PIDH, pCmRevision );
	read_sensor_reg( OV5623_PIDL, pSensorRevision );
	return 0;
}

void ov5623hw_set_hsync()
{
}

void ov5623hw_auto_function_on()
{
	u8 val;
	ov_dbg("ov5623hw_auto_function_on().\n");
	read_sensor_reg( OV5623_COM8, &val );
	val |= 0x07;    /* don't disturb AWB */
	write_sensor_reg( OV5623_COM8, &val );
}

void ov5623hw_auto_function_off()
{
	u8 val;
	ov_dbg("ov5623hw_auto_function_off().\n");
	read_sensor_reg( OV5623_COM8, &val );
	val &= ~0x07;    /* don't disturb AWB */
	write_sensor_reg( OV5623_COM8, &val );
}

void ov5623hw_auto_AECAGC_on()
{
	u8 val;
	ov_dbg("ov5623hw_auto_function_on().\n");
	read_sensor_reg( OV5623_COM8, &val );
	val |= 0x05;    /* don't disturb AWB */
	write_sensor_reg( OV5623_COM8, &val );
}

void ov5623hw_auto_AECAGC_off()
{
	u8 val;
	ov_dbg("ov5623hw_auto_function_off().\n");
	read_sensor_reg( OV5623_COM8, &val );
	val &= ~0x05;    /* don't disturb AWB */
	write_sensor_reg( OV5623_COM8, &val );
}

/*****************************************************************************
 *  Viewfinder, still 
 *****************************************************************************/
int ov5623hw_view_finder_on()
{
	return OV_ERR_NONE;
}


int ov5623hw_view_finder_off()
{
	return OV_ERR_NONE;
}


int ov5623hw_halt_video_output()
{
	return OV_ERR_NONE;
}

int ov5623hw_resume_full_output_mode()
{
	return OV_ERR_NONE;
}

int ov5623hw_get_single_image()
{
	return OV_ERR_NONE;
}

/*****************************************************************************
 *Manual Exposure Control
 ****************************************************************************/
int ov5623hw_calculate_pclk(u32 ciclk)
{
	u8 pll_divider;
	u8 pll_multiplier;
	u8 clk_divider;
	u8 clk_divider_flag;

	u32 pixel_clk;

	read_sensor_reg(0x3d, &pll_divider);
	pll_divider &= 0x10;
	if (pll_divider > 0)
		pll_divider = 1;
	else if(pll_divider == 0)
		pll_divider = 10;


	read_sensor_reg(0x3e, &pll_multiplier);
	pll_multiplier &= 0x3f;
	
	read_sensor_reg(OV5623_CLKRC, &clk_divider);
	clk_divider_flag = clk_divider & 0x40;
	clk_divider_flag = clk_divider_flag >> 6;
	clk_divider &= 0x3f;

	if( clk_divider_flag == 0) {
		pixel_clk = ciclk  * pll_multiplier * 2 / pll_divider;
	} else if( clk_divider_flag == 1) {
		pixel_clk = ciclk * pll_multiplier * 2/
			(pll_divider * 2 *(clk_divider + 1));
	}
	
	ov_dbg("pixel_clk = %d\n", pixel_clk);

	return pixel_clk;
}

int  ov5623hw_field_protect(u32 captureWidth, u32 captureHeight)
{
	u8 gain_val,aechm_val,aech_val,aecl_val,blueg_val,redg_val;
	OV5623_MODE mode;
	
	ov_dbg("ov5623hw_field_protect()\n");

	field_pclk = ov5623hw_calculate_pclk(CICLK);

	/* let the sensor work on proper mode */
        if((captureWidth <= 864) && (captureHeight <= 600)) {
                mode = OV5623_D1MD;
        } else if((captureWidth <= 1280) && (captureHeight <= 960)) {
                mode = OV5623_SXGA;
        } else if((captureWidth <= 2560) && (captureHeight <= 1920)) {
                mode = OV5623_FULL;
        } else {
                return -EINVAL;
        }

        switch(mode)
        {
        case OV5623_D1MD:
                field_max_pixel = 1300;
                field_max_line = 617;
                break;
        case OV5623_SXGA:
                field_max_pixel = 1640;
                field_max_line = 977;
                break;
        case OV5623_FULL:
                field_max_pixel = 3252;
                field_max_line = 1968;
                break;
        default:
                printk("manual_exp_gain:Wrong still image size.\n");
                break;
	}

	read_sensor_reg(OV5623_GAIN, &gain_val);
	ov_dbg("OV5623_GAIN = %2x\n",gain_val);
	mdelay(5);
	read_sensor_reg(OV5623_REG45, &aechm_val);
	ov_dbg("OV5623_REG45 = %2x\n",aechm_val);
	mdelay(5);
	read_sensor_reg(OV5623_AEC, &aech_val);
	ov_dbg("OV5623_AEC = %2x\n",aech_val);
	mdelay(5);
	read_sensor_reg(OV5623_REG04, &aecl_val);
	ov_dbg("OV5623_REG04 = %2x\n",aecl_val);
	mdelay(5);
	read_sensor_reg(OV5623_BLUE, &blueg_val);
	ov_dbg("OV5623_BLUE = %2x\n",blueg_val);
	mdelay(5);
	read_sensor_reg(OV5623_RED, &redg_val);
	ov_dbg("OV5623_RED = %2x\n",redg_val);
	mdelay(5);

	field_gain = gain_val;
	field_exp = (((u32)(aechm_val & 0x3f)) << 11) +	\
		(((u32)(aech_val)) << 3) + (u32)(aecl_val & 0x07);
	field_blueg = blueg_val;
	field_redg = redg_val;

	ov_dbg("field_gain = %d\n", field_gain);
	ov_dbg("field_exp = %d\n", field_exp);

	return 0;
	
} 
	

int ov5623hw_calculate_exp(u32 captureWidth, u32 captureHeight)
{
	u8 aecl_val,aech_val,aechm_val;
	u32 capture_exp, preview_exp;
	u32 capture_max_line, preview_max_line;
	u32 capture_max_pixel, preview_max_pixel;
	u32 capture_pixel_clk, preview_pixel_clk;
	u32 capture_exp_gain;
	u32 time_unit;
	u32 capture_gain;
	u8  tmp_gain;
	OV5623_MODE mode;

	ov_dbg("ov5623hw_calculate_exp().\n");

	/* let the sensor work on proper mode */
        if((captureWidth <= 864) && (captureHeight <= 600)) {
                mode = OV5623_D1MD;
        } else if((captureWidth <= 1280) && (captureHeight <= 960)) {
                mode = OV5623_SXGA;
        } else if((captureWidth <= 2560) && (captureHeight <= 1920)) {
                mode = OV5623_FULL;
        } else {
                return -EINVAL;
        }

	switch(mode)
	{
	case OV5623_D1MD:
		capture_max_pixel = 1300; 
		capture_max_line = 617;
		break;
	case OV5623_SXGA:
		capture_max_pixel = 1640;
		capture_max_line = 977;
		break;
	case OV5623_FULL:
		capture_max_pixel = 3252;
		capture_max_line = 1968;
		break;
	default:
		printk("manual_exp_gain:Wrong still image size.\n");
		break;
	}

	capture_pixel_clk = ov5623hw_calculate_pclk(CICLK); 

	preview_exp = field_exp;
	preview_max_line = field_max_line;
	preview_max_pixel = field_max_pixel;
	preview_pixel_clk = field_pclk;

	
	ov_dbg("preview_exp = %d\n",preview_exp);
	ov_dbg("capture_pixel_clk = %d\n",capture_pixel_clk);
	ov_dbg("preview_max_pixel = %d\n",preview_max_pixel);
	ov_dbg("preview_pixel_clk = %d\n",preview_pixel_clk);
	ov_dbg("capture_max_pixel = %d\n",capture_max_pixel);

	capture_exp = preview_exp * capture_pixel_clk *	\
		preview_max_pixel /( preview_pixel_clk * capture_max_pixel);
	ov_dbg("capture_exp = %d\n",capture_exp);

	/*convert reg 0x00 to field_gain * 16*/
	capture_gain =(field_gain & 0x0f) + 16;


	if (field_gain & 0x10)
		capture_gain = capture_gain << 1;
	if (field_gain & 0x20)
		capture_gain = capture_gain << 1;
	if (field_gain & 0x40)
		capture_gain = capture_gain << 1;
	if (field_gain & 0x80)
		capture_gain = capture_gain << 1;

	capture_exp_gain = capture_exp * capture_gain;
	ov_dbg("OV5623: capture_exp_gain = %08x\n", capture_exp_gain);


	/*calculate banding filter value*/
	/* 1/100s for 50Hz, 1/120s for 60Hz*/
#ifdef FREQUENCE50
	time_unit = capture_pixel_clk *10000 / capture_max_pixel / 100;
#elif  FREQUENCE60
	time_unit = capture_pixel_clk *10000 / capture_max_pixel / 120;
#endif
	ov_dbg("time_unit = %d\n",time_unit);


	/* calculate field_gain, exposure*/

	if (capture_exp_gain < (capture_max_line * 16))
	{
		capture_exp = capture_exp_gain / 16;
		
		if (capture_exp > time_unit) 
		{
			capture_exp /= time_unit;
			capture_exp *= time_unit;
		}
		ov_dbg("capture_exp = %d\n",capture_exp);
	}
	else
	{
		capture_exp = capture_max_line;
		ov_dbg("capture_exp = %d\n",capture_exp);
	}

	if (capture_exp == 0)
		capture_exp = 1;

	capture_gain = (capture_exp_gain*100 + 50)/capture_exp/100;
	ov_dbg("capture_gain = %d\n",capture_gain);

	/* calculate reg0x00 from field_gain * 16*/

	tmp_gain = 0;
	if (capture_gain > 31)
	{
		tmp_gain |= 0x10;
		capture_gain = capture_gain >> 1;
	}	
	if (capture_gain > 31)
	{
		tmp_gain |= 0x20;
		capture_gain = capture_gain >> 1;
	}	
	if (capture_gain > 31)
	{
		tmp_gain |= 0x40;
		capture_gain = capture_gain >> 1;
	}	
	if (capture_gain > 31)
	{
		tmp_gain |= 0x80;
		capture_gain = capture_gain >> 1;
	}	

	if (capture_gain > 16)
		tmp_gain |= ((capture_gain -16) & 0x0f);
	
	capture_gain = tmp_gain;


	/*set values to reg*/
	read_sensor_reg(OV5623_REG04, &aecl_val);
	aecl_val &= 0xf8;
	aecl_val |= (u8)(capture_exp & 0x07);

	read_sensor_reg(OV5623_REG45, &aechm_val);
	aechm_val &= 0xc0;
	aechm_val |= (u8)((capture_exp & 0x1f800) >> 11);

	aech_val = (u8)((capture_exp & 0x7f8) >> 3);


	write_sensor_reg(OV5623_GAIN, 	&tmp_gain);
	write_sensor_reg(OV5623_BLUE,	&field_blueg);
	write_sensor_reg(OV5623_RED,  	&field_redg);
	write_sensor_reg(OV5623_REG45, 	&aechm_val);
	write_sensor_reg(OV5623_AEC, 	&aech_val);
	write_sensor_reg(OV5623_REG04, 	&aecl_val);
	
	ov_dbg("OV5623_GAIN = %2x\n", tmp_gain);
	ov_dbg("OV5623_REG45 = %2x\n", aechm_val);
	ov_dbg("OV5623_AEC = %2x\n", aech_val);
	ov_dbg("OV5623_REG04 = %2x\n", aecl_val);
	

	return 0;
		
}


/*****************************************************************************
 *  Format 
 *****************************************************************************/
int ov5623hw_set_format(u32 captureWidth, 
		u32 captureHeight, 
		u32 *winStartX, 
		u32 *winStartY, 
		u32 *winEndX, 
		u32 *winEndY)
{
	OV5623_MODE mode;
	unsigned char regVal;


	ov_dbg("ov5623hw_set_format().\n");

	/* let the sensor work on proper mode */
	if((captureWidth <= 864) && (captureHeight <= 600)) {
		mode = OV5623_D1MD;
	} else if((captureWidth <= 1280) && (captureHeight <= 960)) {
		mode = OV5623_SXGA;
	} else if((captureWidth <= 2560) && (captureHeight <= 1920)) {
		mode = OV5623_FULL;
	} else {
		return -EINVAL;
	}

	ov5623hw_set_regs(ov5623InitSetting);

	if (mode == OV5623_D1MD) {
		ov5623hw_set_regs(ov5623_d1md);
		ov_dbg("ov5623hw_set_regs(ov5623_d1md).\n");
	} else if (mode == OV5623_SXGA) {
		ov5623hw_set_regs(ov5623_sxga);
		ov_dbg("ov5623hw_set_regs(ov5623_sxga).\n");
	} else {
		ov5623hw_set_regs(ov5623_full);
		ov_dbg("ov5623hw_set_regs(ov5623_full).\n");
	}

	/* Special for OV5623 */
	ov5623hw_set_regs(ov5623OtherSetting);


	*winStartX = 0;
	*winStartY = 0;
	*winEndX   = captureWidth;
	*winEndY   = captureHeight;


	return 0;
}

/*****************************************************************************
 * Contrast
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 ContrastLowestSettings[] = {
	OV5623_REGEND,     0x00
};

const static u8 ContrastLowSettings[] = {
	OV5623_REGEND,     0x00        
};

const static u8 ContrastMiddleSettings[] = {
	OV5623_REGEND,     0x00        
};

const static u8 ContrastHighSettings[] = {
	OV5623_REGEND,     0x00        
};

const static u8 ContrastHighestSettings[] = {
	OV5623_REGEND,     0x00        
};

int ov5623hw_set_contrast(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch(value) {
		case SENSOR_CONTRAST_LOWEST:
			regP = ContrastLowestSettings;
			break;
		case SENSOR_CONTRAST_LOW:
			regP = ContrastLowSettings;
			break;
		case SENSOR_CONTRAST_MIDDLE:
			regP = ContrastMiddleSettings;
			break;
		case SENSOR_CONTRAST_HIGH:
			regP = ContrastHighSettings;
			break;
		case SENSOR_CONTRAST_HIGHEST:
			regP = ContrastHighestSettings;
			break;
		default:
			regP = ContrastMiddleSettings;
			break;
	}

	/* set hw */
	if (regP)        
		ov5623hw_set_regs(regP);

	return 0;
}

/*****************************************************************************
 * Exposure
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 ExposureSettings[] = {
};

int ov5623hw_set_exposure(u32 value)
{
	return 0;
}

/*****************************************************************************
 * Auto White Balance
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 AWBAuto[] = {
	OV5623_REGEND,     0x00        
};

const static u8 AWBFluorescent[] = {
	OV5623_REGEND,     0x00       
};

const static u8 AWBOutdoor[] = {
	OV5623_REGEND,     0x00        
};

const static u8 AWBIncandescent[] = {
	OV5623_REGEND,     0x00        
};


int ov5623hw_set_white_balance(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch(value) {
		case SENSOR_WHITEBALANCE_AUTO:                /* Auto */
			regP = AWBAuto;
			break;
		case SENSOR_WHITEBALANCE_INCANDESCENT:        /* Incandescent */
			regP = AWBIncandescent;
			break;
		case SENSOR_WHITEBALANCE_SUNNY:               /* Sunny */
			regP = AWBOutdoor;
			break;
		case SENSOR_WHITEBALANCE_FLUORESCENT:         /* Fluorescent */
			regP = AWBFluorescent;
			break;
		default:
			break;
	}

	/* set hw */
	if (regP) {
		ov5623hw_set_regs(regP);
	}
	return 0;
}

/******************************************************************************
 *                              OV5623 I2C Client Driver                      
 ******************************************************************************/
#include <linux/i2c.h>
static int i2c_ov5623_attach_adapter(struct i2c_adapter *adapter);
static int i2c_ov5623_detect_client(struct i2c_adapter *, int, int);
static int i2c_ov5623_detach_client(struct i2c_client *client);
#define I2C_DRIVERID_OV5623   (0)
#define	OV5623_ADDRESS	0x30

struct i2c_driver ov5623_driver  = 
{
	.driver	= {
		.name	= "ov5623 i2c client driver",
	},
	.id		= I2C_DRIVERID_OV5623,
	.attach_adapter	= &i2c_ov5623_attach_adapter, 
	.detach_client	= &i2c_ov5623_detach_client,  
};

/* Unique ID allocation */
static struct i2c_client *g_client;
static unsigned short normal_i2c[] = {OV5623_ADDRESS, I2C_CLIENT_END };
I2C_CLIENT_INSMOD_1(ov5623);

static int read_sensor_reg( const u8 subAddress, u8 *bufP )
{
	int ret;

	if( g_client == NULL )	/* No global client pointer? */
		return -1;

	ret = i2c_smbus_read_byte_data(g_client, subAddress);
	if (ret >= 0) {
		*bufP = ret;
	}
	return ret;
}

static int write_sensor_reg( const u8 subAddress, u8 *bufP )
{
	if( g_client == NULL )	/* No global client pointer? */
		return -1;

	return i2c_smbus_write_byte_data(g_client, subAddress, *bufP);	
}

static int i2c_ov5623_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client,reg);
}

static int i2c_ov5623_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &i2c_ov5623_detect_client);
}

static int i2c_ov5623_detect_client(struct i2c_adapter *adapter,
		int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;
	u8 pidh = 0, pidl = 0;

	/* Let's see whether this adapter can support what we need.
	 * Please substitute the things you need here!
	 */
	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		goto ERROR0;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 * But it allows us to access several i2c functions safely
	 */

	/* Note that we reserve some space for ov25623_data too. If you don't
	 * need it, remove it. We do it here to help to lessen memory
	 * fragmentation.
	 */

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL );

	if ( !new_client )  {
		err = -ENOMEM;
		goto ERROR0;
	}

	/* FIXME */
	new_client->addr = address;	
	new_client->adapter = adapter;
	new_client->driver = &ov5623_driver;
	new_client->flags = 0;

	/* detect OV5623 */
	pxa_set_cken(CKEN_CAMERA, 1);
	pxa3xx_mfp_set_afds(MFP_CIF_MCLK, MFP_AF0, MFP_DS04X);
	ci_set_clock(1, 1, CICLK);
	
	sensor_power_on();
	mdelay(1);

	pidh = i2c_ov5623_read(new_client, OV5623_PIDH);
	pidl = i2c_ov5623_read(new_client, OV5623_PIDL);
	if ((pidh != PID_OV56XX) || ((pidl != PID_5623 )&&(pidl != PID_5623_1))){
		ci_set_clock(0, 0, CICLK);
		printk("OV5623 detected failed.\n");
		sensor_power_off();
		pxa_set_cken(CKEN_CAMERA, 0);
		goto ERROR1;
	}		 
	else {
		extern int ov5623_detected;
		ov5623_detected = 1;
		/* To avoid the conflict between Ov5623 and ov7673 */
		i2c_smbus_write_byte_data(new_client, OV5623_COM4, 0x0);
		printk("OV5623 detected.\n");
	}
	ci_set_clock(0, 0, CICLK);
	sensor_power_off();
	pxa_set_cken(CKEN_CAMERA, 0);
	g_client = new_client;

	strcpy(new_client->name, "OV5623");

	/* Tell the i2c layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;

	return 0;

ERROR1:
	kfree(new_client);
ERROR0:
	return err;
}

static int i2c_ov5623_detach_client(struct i2c_client *client)
{
	int err;

	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
		printk("ov5623.o: Client deregistration failed,"
			"client not detached.\n");
		return err;
	}

	/* Frees client data too, if allocated at the same time */
	kfree(client);
	g_client = NULL;
	return 0;
}

static int __init i2c_ov5623_init(void)
{
	int ret;

	if ( (ret = i2c_add_driver(&ov5623_driver)) ) {
		printk("ov5623: Driver registration failed,"
			"module not inserted.\n");
		return ret;
	}

	return 0;
}

static void __exit i2c_ov5623_exit(void)
{
	if (i2c_del_driver(&ov5623_driver)) {
		printk("ov5623: Driver registration failed,"
			"module not removed.\n");
	}
}

MODULE_DESCRIPTION("I2C OV5623 driver");
MODULE_LICENSE("GPL");

module_init(i2c_ov5623_init);
module_exit(i2c_ov5623_exit);
