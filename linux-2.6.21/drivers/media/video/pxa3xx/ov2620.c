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
 *    contains all primitive functions for OV2620
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/

#include "camera.h"
#include "ci.h"
#include "ov2620.h"
#include "ov2620_hw.h"
#include <linux/delay.h>

#include <asm/errno.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>

/* LUT Table, TBD */
static unsigned char lut_table[] = {
	/* RED LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,

	/* BLUE LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,

	/* GREEN LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,
};

/* CMU conversion matrixes, they're sensor specific */
static CI_CMU_COE_MATRIX ov2620_cRGB_sRGB_COE = {
	RGB_FLOAT_TO_INT(1.780214),  RGB_FLOAT_TO_INT(-0.96883),
	RGB_FLOAT_TO_INT(0.188617),

	RGB_FLOAT_TO_INT(-0.7987),   RGB_FLOAT_TO_INT(1.790752),
	RGB_FLOAT_TO_INT(0.007949),

	RGB_FLOAT_TO_INT(-0.67645),  RGB_FLOAT_TO_INT(-1.60901),
	RGB_FLOAT_TO_INT(3.285467),
} ;/* This is not provided by OmniVision yet */

static CI_CMU_COE_MATRIX ov2620_cRGB_YUV_COE = {
	YUV_FLOAT_TO_INT(0.257),  YUV_FLOAT_TO_INT(0.504),
	YUV_FLOAT_TO_INT(0.098),

	YUV_FLOAT_TO_INT(-0.148), YUV_FLOAT_TO_INT(0.291),
	YUV_FLOAT_TO_INT(0.439),

	YUV_FLOAT_TO_INT(0.439),  YUV_FLOAT_TO_INT(0.368),
	YUV_FLOAT_TO_INT(0.071),
};/* This is not provided by OmniVision yet */

/* Bad pixel table, it's sensor specific, TBD */
#define END_MARKER -1
static int ov2620_badpixel_table[] = {
	/*  column,        row
	 *    0,           0,
	 *    1,           1,
	 *    1615,        1207,
	 */
	END_MARKER, END_MARKER
};

/*****************************************************************************
 * OV2620 Functions
 *****************************************************************************/
int ov2620_init( p_camera_context_t camera_context )
{
	u8 cm_rev, cm_pid;
	int timeout;
	int status;

	/* provide informat about the capabilities of the sensor */
	camera_context->sensor_status.caps |= SENSOR_CAP_MANUAL_CONTRAST |
		SENSOR_CAP_MANUAL_WHITEBALANCE |
		SENSOR_CAP_MANUAL_EXPOSURE;

	/* Configure CI according to OV2620's hardware
	 * master parallel with 8 data pins
	 */
	ci_set_mode( CI_MODE_MP, CI_DATA_WIDTH10);

	/* enable pixel clock(sensor will provide pclock)
	 * and master clock = 26MHZ
	 */
	ci_set_clock(1, 1, 2600);

	/* data sample on rising and h,vsync active high */
	ci_set_polarity( 0, 0, 0);

	/* fifo control */
	ci_set_fifo( 0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_2_TO_9);

	/* OV2620 Power on sequence
	 * Take out of Power down mode (GPIO_57), PWRDWN=1, NORMAL=0
	 * Assert Reset
	 * Delay
	 * Remove reset
	 * Delay
	 */
	ov2620hw_power_down(CAMERA_POWER_FULL );
	ov2620hw_reset();
	mdelay(1);

	/* read out version */
	timeout = 50;
	do
	{
		cm_pid = cm_rev = 0;
		status = ov2620hw_version_revision(&cm_pid, &cm_rev);

		/* Check to make sure we are working with an OV2620 */
		if( cm_pid != PID_OV26XX || cm_rev != PID_2620 )
		{
			ov2620hw_power_down(CAMERA_POWER_OFF );
			ov2620hw_power_down(CAMERA_POWER_FULL );
			ov2620hw_reset();
			mdelay(1);
		}
		if (--timeout == 0)
			return -EIO;
	}
	while( cm_pid != PID_OV26XX );

	/* turn sensor output off */
	ov2620hw_view_finder_off();


	return 0;
}

int ov2620_deinit(  p_camera_context_t camera_context )
{
	/* power off the external module */
	ov2620hw_power_down(CAMERA_POWER_OFF );

	return 0;
}

int ov2620_sleep(  p_camera_context_t camera_context )
{
	return ov2620_deinit( camera_context );
}

int ov2620_wake(  p_camera_context_t camera_context )
{
	return ov2620_init( camera_context );
}

int ov2620_set_capture_format(  p_camera_context_t camera_context )
{
	CI_MP_TIMING timing;
	u32 winStartX, winStartY;
	u32 winEndX, winEndY;
	int           *padPixelX, *padPixelY;
	int           badPixelNum;

	/* Set CMU Coe matrix, if necessary */
	if((camera_context->capture_input_format != CAMERA_IMAGE_FORMAT_RAW10)&&
		(camera_context->capture_input_format !=
			CAMERA_IMAGE_FORMAT_RAW8) &&
		(camera_context->capture_input_format !=
			CAMERA_IMAGE_FORMAT_RAW9))
		return -EINVAL;

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_YUV) {
		ci_cmu_set_color_correction_coe(&ov2620_cRGB_YUV_COE);
	}

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_RGB) {
		ci_cmu_set_color_correction_coe(&ov2620_cRGB_sRGB_COE);
	}

	ci_cmu_enable(camera_context->cmu_usage);

	/* Set OV2620 format */
	ov2620hw_set_format(
			camera_context->capture_input_width,
			camera_context->capture_input_height,
			&winStartX,
			&winStartY,
			&winEndX,
			&winEndY);

	/* configure PSU, if necessary */
	if (camera_context->psu_enable ) {
		ci_psu_enable(0);
	}

	padPixelX = ov2620_badpixel_table;
	padPixelY = ov2620_badpixel_table + 1;
	badPixelNum = 0;
	while ((*padPixelX != END_MARKER) && (*padPixelY != END_MARKER)){

		if (((u32)(*padPixelX) >= winStartX) &&
			((u32)(*padPixelX) <= winEndX) &&
			((u32)(*padPixelY) >= winStartY) &&
			((u32)(*padPixelY) <= winEndY)) {

			/* the very first pixel of each color
			 * can't be tagged as bad!!!
			 */
			if ((*padPixelX - winStartX > 1) ||
				(*padPixelY - winStartY > 1)){
				ci_psu_tag_bad_pixel(
					*padPixelX - winStartX,
					*padPixelY - winStartY);

				badPixelNum++;
				/* PSU only can substitute 128 bad pixels!! */
				if (badPixelNum == 128)
					break;
			}
		}

		padPixelX += 2;
		padPixelY += 2;
	}

	/* fill the last PSU RAM location with all 0's,
	 * signifying the end of the dead pixel addresses stored in the RAM
	 */
	if (badPixelNum < 128)
		ci_psu_tag_bad_pixel(0, 0);

	if (badPixelNum != 0){
		if ( !camera_context->psu_enable ){
			ci_psu_enable(1);
			camera_context->psu_enable = 1;
		}
	}

	/* set capture width/height and timing */
	timing.BFW = 0x00;
	timing.BLW = 0x01;   /*GRBG to RGGB*/
	ci_configure_mp(camera_context->capture_input_width-1,
		camera_context->capture_input_height-1, &timing);

	return 0;
}

int ov2620_start_capture(p_camera_context_t camera_context,
		unsigned int frames)
{
	(void) camera_context;

	if (camera_context->cgu_enable) {
		ci_cgu_load_lut_ram((unsigned int*)camera_context->
					histogram_lut_buffer_virtual,
				camera_context->histogram_lut_buffer_physical,
				(unsigned int*)camera_context->
					histogram_lut_dma_descriptors_virtual,
				camera_context->
					histogram_lut_dma_descriptors_physical,
				lut_table);
	}

	ci_cgu_enable(camera_context->cgu_enable);

	/* turn auto function on only doing continues capture */
	if (frames == 0) {
		ov2620hw_auto_function_on();
	} else {
		ov2620hw_auto_function_off();
	}

	/* turn viewfinder on */
	ov2620hw_view_finder_on();
	return 0;
}

int ov2620_stop_capture(p_camera_context_t camera_context)
{
	(void) camera_context;

	/* turn auto function off */
	ov2620hw_auto_function_off();

	/* turn viewfinder off */
	ov2620hw_view_finder_off();

	return 0;
}

int ov2620_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	(void) camera_context;

	ov2620hw_power_down(mode );
	return 0;
}

int ov2620_read_8bit(p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val)
{
	(void) camera_context;

	ov2620hw_read_sensor_reg( reg_addr, reg_val );
	return 0;
}

int ov2620_write_8bit( p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val)
{
	u8 buffer;
	int status;

	(void) camera_context;

	buffer = reg_val;
	status = (ov2620hw_write_sensor_reg( reg_addr, &buffer ) == 0) ?
		0 : -EIO;

	return status;
}

int ov2620_set_contrast( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_CONTRAST)
		return ov2620hw_set_contrast(value);
	else
		return 0;
}

int ov2620_set_exposure( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_EXPOSURE)
		return ov2620hw_set_exposure(value);
	else
		return 0;
}

int ov2620_set_white_balance( p_camera_context_t camera_context,
		u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_WHITEBALANCE)
		return ov2620hw_set_white_balance(value);
	else
		return 0;
}
