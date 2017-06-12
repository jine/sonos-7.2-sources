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

/*++

Abstract:  
      contains all OV5623 specific macros, typedefs, and prototypes.
    Declares no storage.

Notes:Only valid for processor code named Monahans.

--*/


#ifndef __CAM_OV5623_HW_HEADER__
#define __CAM_OV5623_HW_HEADER__

/**********************************************************************
 *
 * Constants & Structures
 *
 **********************************************************************/
// CICLK for QCI
#define CICLK	1300


// Revision constants
#define PID_OV56XX              0x56
#define PID_5620                0x20
#define PID_5623		0x20
#define PID_5623_1              0x21

/* Return codes */
#define OV_ERR_NONE             0x00
#define OV_ERR_TIMEOUT          -1
#define OV_ERR_PARAMETER        -2  
#define OV_COMM_ERR             -3

/*OV5620 Mode*/
typedef enum {
    OV5623_FULL = 0,		/* 2560X1920 */
    OV5623_UXGA,		/* 1600X1200 */
    OV5623_SXGA,		/* 1280X960  */
    OV5623_D1MD,		/* 864X600   */
    OV5623_VGA,			/* 640X480   */
    OV5623_INVALID
}OV5623_MODE;

/* Camera Mode */
#define VIEWFINDER_MODE         0x10
#define STILLFRAME_MODE         0x20

/* Others */
#define OV5623_TIMEOUT          1000    /* ms to timeout. */

#define OV5623_GAIN             0x00    /* AGC Gain Control */
#define OV5623_BLUE        	0x01    /* Blue Gain Control */
#define OV5623_RED 	        0x02    /* Red Gain Control */
#define OV5623_COM1             0x03    /* Common Control 1 */
#define OV5623_REG04            0x04    /* Register 04 */

/* 05-08 RSVD Reserved */

#define OV5623_COM2             0x09    /* Common control 2 */
#define OV5623_PIDH             0x0A    /* Product ID Number MSBs */
#define OV5623_PIDL             0x0B    /* Product ID Number LSBs */
#define OV5623_COM3             0x0C    /* Common control 3 */ 
#define OV5623_COM4             0x0D    /* Common control 4 */
#define OV5623_COM5             0x0E    /* Common control 5 */
#define OV5623_COM6             0x0F    /* Common control 6 */
#define OV5623_AEC              0x10    /* Automatic Exposure [10:3] */
#define OV5623_CLKRC            0x11    /* Clock Rate Control */
#define OV5623_COM7             0x12    /* Common Control 7 */
#define OV5623_COM8             0x13    /* Common control 8 */
#define OV5623_COM9             0x14    /* Common Control 9 */
#define OV5623_COM10            0x15    /* Common Control 10 */
#define OV5623_GREEN		0x16	/* Digital AWB Green Gain Control */
#define OV5623_HREFST           0x17    /* Horizontal Window Start */
#define OV5623_HREFEND          0x18    /* Horizontal window End */
#define OV5623_VSTRT            0x19    /* Vertical Window Line Start */
#define OV5623_VEND             0x1A    /* Vertical Window Line End */
#define OV5623_PSHFT            0x1B    /* Pixel Shift */
#define OV5623_MIDH             0x1C    /* Manufacturer ID Byte - High */
#define OV5623_MIDL             0x1D    /* Manufacturer ID Byte - Low */

/* 1E-23  RSVD Reserved */
/* Luminance Signal High Range for AEC/AGC Operation */
#define OV5623_AEW              0x24
/* Luminance Signal Low Range for AEC/AGC Operation */
#define OV5623_AEB              0x25
/* Fast Mode Large Step Range Threshold */
#define OV5623_VV               0x26

/* 27-29 RSVD Reserved */

#define OV5623_REG2A            0x2A    /* Register 2A */
/* 8 LSBs of EXHC - pixel count in horizontal blank */
#define OV5623_EXHCL            0x2B
/* 2C RSVD Reserved */

#define OV5623_ADDVSL           0x2D    /* VSYNC Pulse Width LSB 8 bits */
#define OV5623_ADDVSH           0x2E    /* VSYNC Pulse Width MSB 8 bits */
#define OV5623_YAVG             0x2F    /* Luminance Average */
/* HSYNC Position and Width Start Point LSB 8 bits */
#define OV5623_HSDY             0x30
/* HSYNC Position and Width End Lower 8 bits */
#define OV5623_HENY             0x31
#define OV5623_REG32            0x32    /* Register 32 */

/* 33-44 RSVD Reserved */

#define OV5623_REG45            0x45    /* Register 45 */
/* Number of vertical blanking Lines LSBs */
#define OV5623_DMLNL            0x46
/* Number of vertical blanking Lines MSBs */
#define OV5623_DMLNH            0x47
#define OV5623_ZOOMSL           0x48    /* Common Control 19 */
/* Zoom Mode Vertical Window Start Point 8 MSBs */
#define OV5623_ZOOMSH           0x49

/* 4A - 5E RSVD Reserved */

#define OV5623_COM30            0x5F    /* Common Control 30 */
#define OV5623_COM31            0x60    /* Common Control 31 */
#define OV5623_COM32            0x61    /* Common Control 32 */

/* 62 RSVD Reserved */

#define OV5623_COM34            0x63    /* Common Control 34 */

/* 64-7F RSVD Reserved */

#define OV5623_DSPEN            0x80    /* DSP Function Enable Control */
#define OV5623_DSP01            0x81    /* Sensor Internal Reference Control */


/* 82 RSVD Reserved */

#define OV5623_DGCTRL           0x83    /* Digital Gain Control */
#define OV5623_AWBBIAS          0x84    /* AWB Gain Bias Setting */
#define OV5623_DSPCTRL          0x85    /* DSP Control */

/* 86-88 RSVD Reserved */

#define OV5623_DSP09            0x89    /* DSP09 */

/* 8A RSVD Reserved */

#define OV5623_DSP0B            0x8B    /* DSP0B */

/* 8C-A7 RSVD Reserved */

#define OV5623_BOTLMT           0xA8    /* Pixel value Lower limit */
#define OV5623_TOPLMT           0xA9    /* Pixel value Upper limit */

/* AA-B7 RSVD Reserved */

#define OV5623_REDLMT		0xB8	/* Red Gain Limit */
#define OV5623_GREENLMT		0xB9	/* Green Gain Limit */
#define OV5623_BLUELMT		0xBA	/* Blue Gain Limit */

/* End of OV5623 register */
#define OV5623_REGEND         	0xFF



/**********************************************************************
 *
 * Function Prototype
 *
 **********************************************************************/
extern u32 field_pclk ;
extern u32 field_max_line ;
extern u32 field_max_pixel;

extern u8 field_gain ;
extern u32 field_exp ;
extern u8 field_blueg ;
extern u8 field_redg ;


int ov5623hw_set_regs( const u8 *regP );
int ov5623hw_read_all_regs( u8 *bufP, u32 numRegs );

void ov5623hw_power_down( u8 powerMode );
void ov5623hw_reset(void);
void ov5623hw_wait( int ms );

int  ov5623hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision);
void ov5623hw_set_hsync(void);
void ov5623hw_auto_function_on(void);
void ov5623hw_auto_function_off(void);

int ov5623hw_view_finder_on(void);
int ov5623hw_view_finder_off(void);
int ov5623hw_halt_video_output(void);
int ov5623hw_resume_full_output_mode(void);
int ov5623hw_get_single_image(void);

int ov5623hw_set_format(u32 captureWidth, 
                              u32 captureHeight, 
                              u32 *winStartX, 
                              u32 *winStartY, 
                              u32 *winEndX, 
                              u32 *winEndY);

int ov5623hw_read_sensor_reg( const u8 subAddress, u8 *bufP );
int ov5623hw_write_sensor_reg( const u8 subAddress, u8 *bufP );
int ov5623hw_set_contrast(u32 value);
int ov5623hw_set_exposure(u32 value);
int ov5623hw_set_white_balance(u32 value);

int ov5623hw_calculate_pclk(u32 ciclk);
int ov5623hw_field_protect(u32 captureWidth,u32 captureHeight);
int ov5623hw_calculate_exp(u32 captureWidth,u32 captureHeight);

#endif

