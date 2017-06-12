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
 *	contains all OV7673 function prototypes.
 *	Declares no storage.
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/

#ifndef __PXA3XX_CAM_OV7673_HEADER__
#define __PXA3XX_CAM_OV7673_HEADER__


#include "camera.h"

/******************************************************************************
 *	Prototypes
 ******************************************************************************/
int ov7673_init( p_camera_context_t );

int ov7673_deinit( p_camera_context_t );

int ov7673_sleep(  p_camera_context_t camera_context );

int ov7673_wake(  p_camera_context_t camera_context );

int ov7673_set_capture_format( p_camera_context_t );

int ov7673_start_capture( p_camera_context_t, unsigned int frames );

int ov7673_stop_capture( p_camera_context_t );

int ov7673_read_8bit(  p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val);

int ov7673_write_8bit(  p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val);

int ov7673_set_power_mode(p_camera_context_t camera_context, u8 mode);

int ov7673_set_contrast( p_camera_context_t camera_context,
		u8 mode, u32 value);

int ov7673_set_exposure( p_camera_context_t camera_context,
		u8 mode, u32 value);

int ov7673_set_white_balance( p_camera_context_t camera_context,
		u8 mode, u32 value);
#endif

