/*******************************************************************
 * pq2_usb_hw.h 2004/04/20
 *
 * Author: Dave Liu   (Daveliu@motorola.com)
 * Creation Date: 2004/04/20
 *
 * Copyright (c) 2004 Motorola
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *******************************************************************/

#ifndef PQ2_USB_HW_H
#define PQ2_USB_HW_H


#define PQ2_IMMR_ADDR            0xf0000000
#define BCSR_ADDR                0xf8000000
#define BCSR3_USB_ENABLE         0x80000000
#define BCSR3_USB_LOW_SPEED      0x40000000
#define BCSR3_USB_SUPPLY_VCC5V   0x20000000

#define USB_HIGHSPEED            1
#define USB_LOWSPEED             0
#define USB_VCC_SUPPLY           1
#define USB_VCC_NOT_SUPPLY       0
#define USB_ENABLE               1
#define USB_DISABLE              0

void pq2_usb_clock_config(void);
void pq2_usb_io_pin_config(void);
void pq2_board_usb_iface_config(void);
unsigned short pq2_dpram_offset(void *addr);
void pq2_board_usb_iface_deconfig(void);
void pq2_free_cpm_dpram(void);

#endif 
