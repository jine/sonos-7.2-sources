/*
   Copyright (C) 2005, Intel Corporation.

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


 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

/*****************************************************************************
 * Abstract:
 *	contains all OV7673 specific macros, typedefs, and prototypes.
 *	Declares no storage.
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/

#ifndef __PXA3XX_CAM_OV7673_HW_HEADER__
#define __PXA3XX_CAM_OV7673_HW_HEADER__

/*****************************************************************************
 * Constants & Structures
 *****************************************************************************/
/* Revision constants */
#define PID_OV76XX			0x76
#define PID_7673			0x73

/* Return codes */
#define OV_ERR_NONE       	0x00
#define OV_ERR_TIMEOUT    	-1
#define OV_ERR_PARAMETER  	-2
#define OV_COMM_ERR			-3

/* Output Size & Format */
#define OV_SIZE_QQVGA			0x01
#define OV_SIZE_QVGA			( OV_SIZE_QQVGA << 1 )
#define OV_SIZE_VGA			( OV_SIZE_QQVGA << 2 )
#define OV_SIZE_QQCIF			0x10
#define OV_SIZE_QCIF			( OV_SIZE_QQCIF << 1 )
#define OV_SIZE_CIF			( OV_SIZE_QQCIF << 2 )
#define OV_FORMAT_YUV_422		1
#define OV_FORMAT_RGB_565		2
#define OV_FORMAT_RAW8   		3

/* Camera Mode */
#define VIEWFINDER_MODE     0x10
#define STILLFRAME_MODE     0x20

/* Others */
#define OV7673_TIMEOUT    1000    /* ms to timeout */

/* OV7673 Register Definitions */
#define OV7673_GAIN		0x0000
#define OV7673_BLUE		0x0001
#define OV7673_RED		0x0002
#define OV7673_VREF		0x0003
#define OV7673_COM1		0x0004
#define OV7673_BAVE		0x0005	/* U/B Average Level */
#define OV7673_GEAVE		0x0006	/* Y/Ge Average Level */
#define OV7673_GOAVE		0x0007	/* Y/Go Average Level */
#define OV7673_RAVE		0x0008	/* V/R Average level */
#define OV7673_COM2		0x0009	/* Common control 2 */
#define OV7673_PID		0x000A	/* Product ID */
#define OV7673_VER		0x000B	/* Version */
#define OV7673_COM3		0x000C
#define OV7673_COM4		0x000D
#define OV7673_COM5		0x000E
#define OV7673_COM6		0x000F
#define OV7673_AECH		0x0010
#define OV7673_CLKRC		0x0011
#define OV7673_COM7		0x0012
#define OV7673_COM8		0x0013
#define OV7673_COM9		0x0014
#define OV7673_COM10		0x0015
#define OV7673_WS		0x0016
#define OV7673_HSTART		0x0017
#define OV7673_HSTOP		0x0018
#define OV7673_VSTRT		0x0019
#define OV7673_VSTOP		0x001A
#define OV7673_PSHFT		0x001B
#define OV7673_MIDH		0x001C
#define OV7673_MIDL		0x001D
#define OV7673_DLY		0x001E
#define OV7673_LAEC		0x001F
#define OV7673_BOS		0x0020
#define OV7673_GBOS		0x0021
#define OV7673_GROS		0x0022
#define OV7673_ROS		0x0023
#define OV7673_AEW		0x0024
#define OV7673_AEB		0x0025
#define OV7673_VPT		0x0026
#define OV7673_BBIAS		0x0027
#define OV7673_GbBIAS		0x0028
#define OV7673_GrBIAS		0x0029
#define OV7673_EXHCH		0x002A
#define OV7673_EXHCL		0x002B
#define OV7673_RBIAS		0x002C
#define OV7673_ADVFL		0x002D
#define OV7673_ADVFH		0x002E
#define OV7673_YAVE		0x002F
#define OV7673_HSYST		0x0030
#define OV7673_HSYEN		0x0031
#define OV7673_HREF		0x0032
#define OV7673_CHLF		0x0033
#define OV7673_ARBLM		0x0034
#define OV7673_VRHL		0x0035
#define OV7673_VIDO		0x0036
#define OV7673_ADC		0x0037
#define OV7673_ACOM		0x0038
#define OV7673_OFON		0x0039
#define OV7673_TSLB		0x003A
#define OV7673_COM11		0x003B
#define OV7673_COM12		0x003C
#define OV7673_COM13		0x003D
#define OV7673_COM14		0x003E
#define OV7673_EDGE		0x003F
#define OV7673_COM15		0x0040
#define OV7673_COM16		0x0041
#define OV7673_COM17		0x0042
#define OV7673_AWBTH1		0x0043
#define OV7673_AWBTH2		0x0044
#define OV7673_AWBTH3		0x0045
#define OV7673_AWBTH4		0x0046
#define OV7673_AWBTH5		0x0047
#define OV7673_AWBTH6		0x0048
#define OV7673_RSVD49		0x0049
#define OV7673_MTX1		0x004F
#define OV7673_MTX2		0x0050
#define OV7673_MTX3		0x0051
#define OV7673_MTX4		0x0052
#define OV7673_MTX5		0x0053
#define OV7673_MTX6		0x0054
#define OV7673_MTX7		0x0055
#define OV7673_MTX8		0x0056
#define OV7673_MTX9		0x0057
#define OV7673_MTXS		0x0058
#define OV7673_AWBC1		0x0059
#define OV7673_AWBC2		0x005A
#define OV7673_AWBC3		0x005B
#define OV7673_AWBC4		0x005C
#define OV7673_AWBC5		0x005D
#define OV7673_AWBC6		0x005E
#define OV7673_AWBC7		0x005F
#define OV7673_AWBC8		0x0060
#define OV7673_AWBC9		0x0061
#define OV7673_LCC1		0x0062
#define OV7673_LCC2		0x0063
#define OV7673_LCC3		0x0064
#define OV7673_LCC4		0x0065
#define OV7673_LCC5		0x0066
#define OV7673_MANU		0x0067
#define OV7673_MANV		0x0068
#define OV7673_HV		0x0069
#define OV7673_MBD		0x006A
#define OV7673_DBLV		0x006B
#define OV7673_GSP0		0x006C
#define OV7673_GSP1		0x006D
#define OV7673_GSP2		0x006E
#define OV7673_GSP3		0x006F
#define OV7673_GSP4		0x0070
#define OV7673_GSP5		0x0071
#define OV7673_GSP6		0x0072
#define OV7673_GSP7		0x0073
#define OV7673_GSP8		0x0074
#define OV7673_GSP9		0x0075
#define OV7673_GSP10		0x0076
#define OV7673_GSP11		0x0077
#define OV7673_GSP12		0x0078
#define OV7673_GSP13		0x0079
#define OV7673_GSP14		0x007A
#define OV7673_GSP15		0x007B
#define OV7673_GST0		0x007C
#define OV7673_GST1		0x007D
#define OV7673_GST2		0x007E
#define OV7673_GST3		0x007F
#define OV7673_GST4		0x0080
#define OV7673_GST5		0x0081
#define OV7673_GST6		0x0082
#define OV7673_GST7		0x0083
#define OV7673_GST8		0x0084
#define OV7673_GST9		0x0085
#define OV7673_GST10		0x0086
#define OV7673_GST11		0x0087
#define OV7673_GST12		0x0088
#define OV7673_GST13		0x0089
#define OV7673_GST14		0x008A

/* End of OV7673 register */
#define OV7673_REGEND		( 0xCA + 1 )



/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
int ov7673hw_set_regs( const u8 *regP );
int ov7673hw_read_all_regs( u8 *bufP, u32 numRegs );

void ov7673hw_power_down( u8 powerMode );
void ov7673hw_reset(void);
void ov7673hw_wait( int ms );

int ov7673hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision);
void ov7673hw_set_hsync(void);
void ov7673hw_auto_function_on(void);
void ov7673hw_auto_function_off(void);

int ov7673hw_viewfinder_on(void);
int ov7673hw_viewfinder_off(void);
int ov7673hw_halt_video_output(void);
int ov7673hw_resumeto_full_output_mode(void);
int ov7673hw_get_single_image(void);

int ov7673hw_set_format( u32 captureSizeFormat, u32 colorFormat, u32 mode);
int ov7673hw_read_sensor_reg( const u8 subAddress, u8 *bufP );
int ov7673hw_write_sensor_reg( const u8 subAddress, u8 *bufP );
int ov7673hw_set_contrast(u32 value);
int ov7673hw_set_exposure(u32 value);
int ov7673hw_set_white_balance(u32 value);

#endif
