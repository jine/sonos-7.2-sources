/*
 *  linux/include/asm-arm/arch-pxa/littleton.h
 *
 *  Author:	Yin Fengwei
 *  Created:	Jan 18, 2007
 *  Copyright:	(C) Copyright 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * (C) Copyright 2006 Marvell International Ltd.  
 * All Rights Reserved 
 */

#ifndef ASM_MACH_WOODSTOCK_H
#define ASM_MACH_WOODSTOCK_H
#include <asm/arch/mfp.h>

/* 
 *  Note: For the first instance of the Littleton, exact usage of many pins is
 *        not fully defined, or the headers to which they lead are not expected
 *        to be populated.  For those pins' symbols, the string "RSVD" is 
 *        placed after "_MFP_".  They should not be used by any module but only 
 *        set to an appropriate permanently initialized value by a single 
 *        system initialization function.
 * 
 *  Note: For each "RSVD" pin on the target platform, an unassigned Alternate
 *        Function code (based on internal silicon documents) has been selected
 *        so that there are no side effects and minimal power effects from 
 *        configuring the pin with that AF code.  It is, effectively, a null
 *        AF.  This permits simpler power-sensitive test board SW than would be
 * 	  required if GPIO were to be selected as the AF device for
 *	   unattached pins.
 */

/* Pick an unused AF on this pin for its NULL AF */

/*****************************************************************************
 * Beginning of Platform MFP Values and Alternate Function for Each Pin      *
 *									     *
 *									     *
 *									     *
 * Concept note:							     *
 *    If the XLLP modules and OS drivers are considered to be "Software ICs" *
 *    then the following set of constants can be considered as the	     *
 *    "Software PCB layer" in that it establishes the connection between the *
 *    processor pins and the target peripherals.			     *
 *****************************************************************************/

/* No specified use for GPIO 0.  It goes to a generic header */
//#define MFP_RDY					(MFP_PIN_GPIO0)
//#define MFP_RDY_AF				(MFP_PIN_GPIO0_AF_DF_RDY)

//#define MFP_FLASH_NOR_CS_N_GPIO			(MFP_PIN_GPIO1)
//#define MFP_FLASH_NOR_CS_N_GPIO_AF		(MFP_PIN_GPIO1_AF_nCS2)

//#define MFP_DEBUG_ETH_CS_N_GPIO			(MFP_PIN_GPIO2)
//#define	MFP_DEBUG_ETH_CS_N_GPIO_AF		(MFP_PIN_GPIO2_AF_nCS3)

#define MFP_MMC_DAT0				(MFP_PIN_GPIO3)
#define MFP_MMC_DAT0_AF				(MFP_PIN_GPIO3_AF_MM1_DAT0)

#define MFP_MMC_DAT1				(MFP_PIN_GPIO4)
#define MFP_MMC_DAT1_AF				(MFP_PIN_GPIO4_AF_MM1_DAT1)

#define MFP_MMC_DAT2				(MFP_PIN_GPIO5)
#define MFP_MMC_DAT2_AF				(MFP_PIN_GPIO5_AF_MM1_DAT2)

#define MFP_MMC_DAT3				(MFP_PIN_GPIO6)
#define MFP_MMC_DAT3_AF				(MFP_PIN_GPIO6_AF_MM1_DAT3)

#define MFP_MMC_CLK				(MFP_PIN_GPIO7)
#define MFP_MMC_CLK_AF				(MFP_PIN_GPIO7_AF_MM1_CLK)

#define MFP_MMC_CMD_0				(MFP_PIN_GPIO8)
#define MFP_MMC_CMD_0_AF			(MFP_PIN_GPIO8_AF_MM1_CMD)

//#define MFP_MMC2_DAT0				(MFP_PIN_GPIO9)
//#define MFP_MMC2_DAT0_AF			(MFP_PIN_GPIO9_AF_MM2_DAT0)

//#define MFP_MMC2_DAT1				(MFP_PIN_GPIO10)
//#define MFP_MMC2_DAT1_AF			(MFP_PIN_GPIO10_AF_MM2_DAT1)

//#define MFP_MMC2_DAT2_CS0			(MFP_PIN_GPIO11)
//#define MFP_MMC2_DAT2_CS0_AF			(MFP_PIN_GPIO11_AF_MM2_DAT2)

//#define MFP_MMC2_DAT3_CS1			(MFP_PIN_GPIO12)
//#define MFP_MMC2_DAT3_CS1_AF			(MFP_PIN_GPIO12_AF_MM2_DAT3)


//#define MFP_MMC2_CLK				(MFP_PIN_GPIO13)
//#define MFP_MMC2_CLK_AF				(MFP_PIN_GPIO13_AF_MM2_CLK)

//#define MFP_MMC2_CMD				(MFP_PIN_GPIO14)
//#define MFP_MMC2_CMD_AF				(MFP_PIN_GPIO14_AF_MM2_CMD)

//#define	MFP_MMC_CD_0_GPIO			(MFP_PIN_GPIO15)
//#define	MFP_MMC_CD_0_GPIO_AF			(MFP_PIN_GPIO15_AF_GPIO_15)

//#define MFP_CIR_ON_PWM				(MFP_PIN_GPIO16)
//#define MFP_CIR_ON_PWM_AF			(MFP_PIN_GPIO16_AF_CIR_OUT)

//#define MFP_SSP_2_LCD_CS			(MFP_PIN_GPIO17)
//#define MFP_SSP_2_LCD_CS_AF			(MFP_PIN_GPIO17_AF_SSP2_FRM)

#define	MFP_PMIC_INT				(MFP_PIN_GPIO102)
#define	MFP_PMIC_INT_AF				(MFP_PIN_GPIO102_AF_GPIO_102)

#define MFP_SCL					(MFP_PIN_GPIO21)
#define MFP_SCL_AF				(MFP_PIN_GPIO21_AF_I2C_SCL)

#define MFP_SDA					(MFP_PIN_GPIO22)
#define MFP_SDA_AF				(MFP_PIN_GPIO22_AF_I2C_SDA)

#define MFP_FFRXD                               (MFP_PIN_GPIO30)
#define MFP_FFRXD_AF                            (MFP_PIN_GPIO30_AF_UART1_RXD)

#define MFP_FFTXD                               (MFP_PIN_GPIO31)
#define MFP_FFTXD_AF                            (MFP_PIN_GPIO31_AF_UART1_TXD)

#define MFP_KP_DKIN_0				(MFP_PIN_GPIO107)
#define MFP_KP_DKIN_0_AF			(MFP_PIN_GPIO107_AF_KP_DKIN_0)
#define MFP_KP_DKIN_1				(MFP_PIN_GPIO108)
#define MFP_KP_DKIN_1_AF			(MFP_PIN_GPIO108_AF_KP_DKIN_1)
#define	MFP_KP_DKIN_2				(MFP_PIN_GPIO125)
#define	MFP_KP_DKIN_2_AF			(MFP_PIN_GPIO125_AF_KP_DKIN_2)
#define	MFP_KP_DKIN_3				(MFP_PIN_GPIO124)
#define	MFP_KP_DKIN_3_AF			(MFP_PIN_GPIO124_AF_KP_DKIN_3)


//
// End of Platform MFP Values for Each Pin 
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//
// Beginning of Platform Drive Strength Values  
//

//
// Note: The actual best values for the Littleton platform are not known.
//       These values are based on silicon team estimates for certain
//       voltages in ideal circumstances and the known voltage domains
//       on the Littleton.
//    

#define MFP_DEFAULT_DS			(MFP_DS03X)
#define MFP_USB2_DS			(MFP_DS08X)
#define MFP_CIF_DS			(MFP_DS04X)
#define MFP_LCD_DS			(MFP_DS04X)
#define MFP_SSP_DS			(MFP_DS04X)
#define MFP_MSL_DEFAULT_DS		(MFP_DS04X) //3.0V: 3 pins
#define MFP_MSL_SPECIAL_DS		(MFP_DS08X) //1.8V: 1 pin 
#define MFP_ND_DATA_DS			(MFP_DS08X)
#define MFP_ND_NON_DATA_DS		(MFP_DS10X)
#define MFP_AC97_DS			(MFP_DS04X)

// #define	LITTLETON_ETH_PHYS		0x30000000

#if 0
/* Littleton technology boards presented bits for "Technology_board_detect" */
#define	WLAN_BOARD		0x01
#define	CAMERA_BOARD		0x02
#define	UMTS_BOARD		0x04
#define	BTWLANCAMERA_BOARD	0x08
#define	WLAN8385CAMERA_BOARD	0x10
#define	SIEMENS_BOARD		0x11
#define	WLAN8686CAMERA_BOARD	0x12

/* Littleton combined WLAN and Camera board - 0x10 bit is presented in both
 * masks. It is used in writes to decrease the power consupmtion.
 */
#define	WLAN8385CAMERA_BOARD_CAMERA_MASK	0x8c
#define	WLAN8385CAMERA_BOARD_WLAN		0x83
#define	WLAN8686CAMERA_BOARD_CAMERA_MASK	0x8c
#define	WLAN8686CAMERA_BOARD_WLAN		0x83


void pxa3xx_enable_eth_pins(void);
void pxa3xx_enable_i2c_pins(void);
void pxa3xx_enable_dfc_pins(void);
void pxa3xx_enable_lcd_pins(void);
void pxa3xx_enable_cif_pins(void);
void pxa3xx_enable_keyp_pins(void);
void pxa3xx_enable_mmc1_pins(void);
void pxa3xx_enable_mmc2_pins(void);
void pxa3xx_enable_mmc3_pins(void);
void pxa3xx_enable_ffuart_pins(void);
void pxa3xx_enable_btuart_pins(void);
void pxa3xx_enable_stuart_pins(void);
void pxa3xx_enable_u2d_pins(void);

void pxa3xx_enable_1w_pins(void);
void pxa3xx_enable_otg_pins(void);
void pxa3xx_enable_ssp3_pins(void);
void pxa3xx_enable_ssp4_pins(void);
#endif /*0*/

#endif
