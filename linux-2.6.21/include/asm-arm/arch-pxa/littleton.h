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

#ifndef ASM_MACH_LITTLETON_H
#define ASM_MACH_LITTLETON_H
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
#define MFP_RDY					(MFP_PIN_GPIO0)
#define MFP_RDY_AF				(MFP_PIN_GPIO0_AF_DF_RDY)

#define MFP_FLASH_NOR_CS_N_GPIO			(MFP_PIN_GPIO1)
#define MFP_FLASH_NOR_CS_N_GPIO_AF		(MFP_PIN_GPIO1_AF_nCS2)

#define MFP_DEBUG_ETH_CS_N_GPIO			(MFP_PIN_GPIO2)
#define	MFP_DEBUG_ETH_CS_N_GPIO_AF		(MFP_PIN_GPIO2_AF_nCS3)

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

#define MFP_MMC2_DAT0				(MFP_PIN_GPIO9)
#define MFP_MMC2_DAT0_AF			(MFP_PIN_GPIO9_AF_MM2_DAT0)

#define MFP_MMC2_DAT1				(MFP_PIN_GPIO10)
#define MFP_MMC2_DAT1_AF			(MFP_PIN_GPIO10_AF_MM2_DAT1)

#define MFP_MMC2_DAT2_CS0			(MFP_PIN_GPIO11)
#define MFP_MMC2_DAT2_CS0_AF			(MFP_PIN_GPIO11_AF_MM2_DAT2)

#define MFP_MMC2_DAT3_CS1			(MFP_PIN_GPIO12)
#define MFP_MMC2_DAT3_CS1_AF			(MFP_PIN_GPIO12_AF_MM2_DAT3)


#define MFP_MMC2_CLK				(MFP_PIN_GPIO13)
#define MFP_MMC2_CLK_AF				(MFP_PIN_GPIO13_AF_MM2_CLK)

#define MFP_MMC2_CMD				(MFP_PIN_GPIO14)
#define MFP_MMC2_CMD_AF				(MFP_PIN_GPIO14_AF_MM2_CMD)

#define	MFP_MMC_CD_0_GPIO			(MFP_PIN_GPIO15)
#define	MFP_MMC_CD_0_GPIO_AF			(MFP_PIN_GPIO15_AF_GPIO_15)

#define MFP_CIR_ON_PWM				(MFP_PIN_GPIO16)
#define MFP_CIR_ON_PWM_AF			(MFP_PIN_GPIO16_AF_CIR_OUT)

#define MFP_SSP_2_LCD_CS			(MFP_PIN_GPIO17)
#define MFP_SSP_2_LCD_CS_AF			(MFP_PIN_GPIO17_AF_SSP2_FRM)

#define	MFP_PMIC_INT				(MFP_PIN_GPIO18)
#define	MFP_PMIC_INT_AF				(MFP_PIN_GPIO18_AF_GPIO_18)

#define MFP_RSVD_PWM_0				(MFP_PIN_GPIO19)
#define MFP_RSVD_PWM_0_AF			(MFP_PIN_GPIO19_AF_PWM0_OUT)

#define MFP_ONE_WIRE				(MFP_PIN_GPIO20)
#define MFP_ONE_WIRE_AF				(MFP_PIN_GPIO20_AF_OW_DQ_IN)

#define MFP_SCL					(MFP_PIN_GPIO21)
#define MFP_SCL_AF				(MFP_PIN_GPIO21_AF_I2C_SCL)

#define MFP_SDA					(MFP_PIN_GPIO22)
#define MFP_SDA_AF				(MFP_PIN_GPIO22_AF_I2C_SDA)

#define	MFP_HDMI_INT				(MFP_PIN_GPIO24)
#define	MFP_HDMI_INT_AF				(MFP_PIN_GPIO24_AF_GPIO_24)

#define MFP_SSP_2_CLK				(MFP_PIN_GPIO25)
#define MFP_SSP_2_CLK_AF			(MFP_PIN_GPIO25_AF_SSP2_SCLK)

#define MFP_SSP_2_FRM				(MFP_PIN_GPIO26)
#define MFP_SSP_2_FRM_AF			(MFP_PIN_GPIO26_AF_SSP2_FRM)

#define MFP_SSP_2_TXD				(MFP_PIN_GPIO27)
#define MFP_SSP_2_TXD_AF			(MFP_PIN_GPIO27_AF_SSP2_TXD)

#define MFP_SSP_2_RXD				(MFP_PIN_GPIO28)
#define MFP_SSP_2_RXD_AF			(MFP_PIN_GPIO28_AF_SSP2_RXD)

#define MFP_ULPI_STP				(MFP_PIN_ULPI_STP)
#define MFP_ULPI_STP_AF				(MFP_ULPI_STP_AF_STP)

#define MFP_ULPI_NXT				(MFP_PIN_ULPI_NXT)
#define MFP_ULPI_NXT_AF				(MFP_ULPI_NXT_AF_NXT)

#define MFP_ULPI_DIR				(MFP_PIN_ULPI_DIR)
#define MFP_ULPI_DIR_AF				(MFP_ULPI_DIR_AF_DIR)

#define MFP_ULPI_DATAOUT_0			(MFP_PIN_GPIO30)
#define MFP_ULPI_DATAOUT_0_AF			(MFP_PIN_GPIO30_AF_ULPI_DATA_OUT_0)

#define MFP_ULPI_DATAOUT_1			(MFP_PIN_GPIO31)
#define MFP_ULPI_DATAOUT_1_AF			(MFP_PIN_GPIO31_AF_ULPI_DATA_OUT_1)

#define MFP_ULPI_DATAOUT_2			(MFP_PIN_GPIO32)
#define MFP_ULPI_DATAOUT_2_AF			(MFP_PIN_GPIO32_AF_ULPI_DATA_OUT_2)

#define MFP_ULPI_DATAOUT_3			(MFP_PIN_GPIO33)
#define MFP_ULPI_DATAOUT_3_AF			(MFP_PIN_GPIO33_AF_ULPI_DATA_OUT_3)

#define MFP_ULPI_DATAOUT_4			(MFP_PIN_GPIO34)
#define MFP_ULPI_DATAOUT_4_AF			(MFP_PIN_GPIO34_AF_ULPI_DATA_OUT_4)

#define MFP_ULPI_DATAOUT_5			(MFP_PIN_GPIO35)
#define MFP_ULPI_DATAOUT_5_AF			(MFP_PIN_GPIO35_AF_ULPI_DATA_OUT_5)

#define MFP_ULPI_DATAOUT_6			(MFP_PIN_GPIO36)
#define MFP_ULPI_DATAOUT_6_AF			(MFP_PIN_GPIO36_AF_ULPI_DATA_OUT_6)

#define MFP_ULPI_DATAOUT_7			(MFP_PIN_GPIO37)
#define MFP_ULPI_DATAOUT_7_AF			(MFP_PIN_GPIO37_AF_ULPI_DATA_OUT_7)

#define MFP_ULPI_CLK				(MFP_PIN_GPIO38)
#define MFP_ULPI_CLK_AF				(MFP_PIN_GPIO38_AF_ULPI_CLK)

#define MFP_CIF_DD_0				(MFP_PIN_GPIO39)
#define MFP_CIF_DD_0_AF				(MFP_PIN_GPIO39_AF_CI_DD_0)

#define MFP_CIF_DD_1				(MFP_PIN_GPIO40)
#define MFP_CIF_DD_1_AF				(MFP_PIN_GPIO40_AF_CI_DD_1)

#define MFP_CIF_DD_2				(MFP_PIN_GPIO41)
#define MFP_CIF_DD_2_AF				(MFP_PIN_GPIO41_AF_CI_DD_2)

#define MFP_CIF_DD_3				(MFP_PIN_GPIO42)
#define MFP_CIF_DD_3_AF				(MFP_PIN_GPIO42_AF_CI_DD_3)

#define MFP_CIF_DD_4				(MFP_PIN_GPIO43)
#define MFP_CIF_DD_4_AF				(MFP_PIN_GPIO43_AF_CI_DD_4)

#define MFP_CIF_DD_5				(MFP_PIN_GPIO44)
#define MFP_CIF_DD_5_AF				(MFP_PIN_GPIO44_AF_CI_DD_5)

#define MFP_CIF_DD_6				(MFP_PIN_GPIO45)
#define MFP_CIF_DD_6_AF				(MFP_PIN_GPIO45_AF_CI_DD_6)

#define MFP_CIF_DD_7				(MFP_PIN_GPIO46)
#define MFP_CIF_DD_7_AF				(MFP_PIN_GPIO46_AF_CI_DD_7)

#define MFP_CIF_DD_8				(MFP_PIN_GPIO47)
#define MFP_CIF_DD_8_AF				(MFP_PIN_GPIO47_AF_CI_DD_8)

#define MFP_CIF_DD_9				(MFP_PIN_GPIO48)
#define MFP_CIF_DD_9_AF				(MFP_PIN_GPIO48_AF_CI_DD_9)

#define MFP_CIF_MCLK				(MFP_PIN_GPIO49)
#define MFP_CIF_MCLK_AF				(MFP_PIN_GPIO49_AF_CI_MCLK)

#define MFP_CIF_PCLK				(MFP_PIN_GPIO50)
#define MFP_CIF_PCLK_AF				(MFP_PIN_GPIO50_AF_CI_PCLK)

#define MFP_CIF_HSYNC				(MFP_PIN_GPIO51)
#define MFP_CIF_HSYNC_AF			(MFP_PIN_GPIO51_AF_CI_LV)

#define MFP_CIF_VSYNC				(MFP_PIN_GPIO52)
#define MFP_CIF_VSYNC_AF			(MFP_PIN_GPIO52_AF_CI_FV)

#define MFP_L_DD_0				(MFP_PIN_GPIO54)
#define MFP_L_DD_0_AF				(MFP_PIN_GPIO54_AF_LCD_LDD_0)

#define MFP_L_LP_DD_0				(MFP_PIN_GPIO54)
#define MFP_L_LP_DD_0_AF			(MFP_PIN_GPIO54_AF_LCD_LDD_0)

#define MFP_L_DD_1				(MFP_PIN_GPIO55)
#define MFP_L_DD_1_AF				(MFP_PIN_GPIO55_AF_LCD_LDD_1)

#define MFP_L_LP_DD_1				(MFP_PIN_GPIO55)
#define MFP_L_LP_DD_1_AF			(MFP_PIN_GPIO55_AF_LCD_LDD_1)

#define MFP_L_DD_2				(MFP_PIN_GPIO56)
#define MFP_L_DD_2_AF				(MFP_PIN_GPIO56_AF_LCD_LDD_2)

#define MFP_L_LP_DD_2				(MFP_PIN_GPIO56)
#define MFP_L_LP_DD_2_AF			(MFP_PIN_GPIO56_AF_LCD_LDD_2)

#define MFP_L_DD_3				(MFP_PIN_GPIO57)
#define MFP_L_DD_3_AF				(MFP_PIN_GPIO57_AF_LCD_LDD_3)

#define MFP_L_LP_DD_3				(MFP_PIN_GPIO57)
#define MFP_L_LP_DD_3_AF			(MFP_PIN_GPIO57_AF_LCD_LDD_3)

#define MFP_L_DD_4				(MFP_PIN_GPIO58)
#define MFP_L_DD_4_AF				(MFP_PIN_GPIO58_AF_LCD_LDD_4)

#define MFP_L_LP_DD_4				(MFP_PIN_GPIO58)
#define MFP_L_LP_DD_4_AF			(MFP_PIN_GPIO58_AF_LCD_LDD_4)

#define MFP_L_DD_5				(MFP_PIN_GPIO59)
#define MFP_L_DD_5_AF				(MFP_PIN_GPIO59_AF_LCD_LDD_5)

#define MFP_L_LP_DD_5				(MFP_PIN_GPIO59)
#define MFP_L_LP_DD_5_AF			(MFP_PIN_GPIO59_AF_LCD_LDD_5)

#define MFP_L_DD_6				(MFP_PIN_GPIO60)
#define MFP_L_DD_6_AF				(MFP_PIN_GPIO60_AF_LCD_LDD_6)

#define MFP_L_LP_DD_6				(MFP_PIN_GPIO60)
#define MFP_L_LP_DD_6_AF			(MFP_PIN_GPIO60_AF_LCD_LDD_6)

#define MFP_L_DD_7				(MFP_PIN_GPIO61)
#define MFP_L_DD_7_AF				(MFP_PIN_GPIO61_AF_LCD_LDD_7)

#define MFP_L_LP_DD_7				(MFP_PIN_GPIO61)
#define MFP_L_LP_DD_7_AF			(MFP_PIN_GPIO61_AF_LCD_LDD_7)

#define MFP_L_DD_8				(MFP_PIN_GPIO62)
#define MFP_L_DD_8_AF				(MFP_PIN_GPIO62_AF_LCD_LDD_8)

#define MFP_L_LP_DD_8				(MFP_PIN_GPIO62)
#define MFP_L_LP_DD_8_AF			(MFP_PIN_GPIO62_AF_LCD_LDD_8)

#define MFP_L_DD_9				(MFP_PIN_GPIO63)
#define MFP_L_DD_9_AF				(MFP_PIN_GPIO63_AF_LCD_LDD_9)

#define MFP_L_LP_DD_9				(MFP_PIN_GPIO63)
#define MFP_L_LP_DD_9_AF			(MFP_PIN_GPIO63_AF_LCD_LDD_9)

#define MFP_L_DD_10				(MFP_PIN_GPIO64)
#define MFP_L_DD_10_AF				(MFP_PIN_GPIO64_AF_LCD_LDD_10)

#define MFP_L_LP_DD_10				(MFP_PIN_GPIO64)
#define MFP_L_LP_DD_10_AF			(MFP_PIN_GPIO64_AF_LCD_LDD_10)

#define MFP_L_DD_11				(MFP_PIN_GPIO65)
#define MFP_L_DD_11_AF				(MFP_PIN_GPIO65_AF_LCD_LDD_11)

#define MFP_L_LP_DD_11				(MFP_PIN_GPIO65)
#define MFP_L_LP_DD_11_AF			(MFP_PIN_GPIO65_AF_LCD_LDD_11)

#define MFP_L_DD_12				(MFP_PIN_GPIO66)
#define MFP_L_DD_12_AF				(MFP_PIN_GPIO66_AF_LCD_LDD_12)

#define MFP_L_LP_DD_12				(MFP_PIN_GPIO66)
#define MFP_L_LP_DD_12_AF			(MFP_PIN_GPIO66_AF_LCD_LDD_12)

#define MFP_L_DD_13				(MFP_PIN_GPIO67)
#define MFP_L_DD_13_AF				(MFP_PIN_GPIO67_AF_LCD_LDD_13)

#define MFP_L_LP_DD_13				(MFP_PIN_GPIO67)
#define MFP_L_LP_DD_13_AF			(MFP_PIN_GPIO67_AF_LCD_LDD_13)

#define MFP_L_DD_14				(MFP_PIN_GPIO68)
#define MFP_L_DD_14_AF				(MFP_PIN_GPIO68_AF_LCD_LDD_14)

#define MFP_L_LP_DD_14				(MFP_PIN_GPIO68)
#define MFP_L_LP_DD_14_AF			(MFP_PIN_GPIO68_AF_LCD_LDD_14)

#define MFP_L_DD_15				(MFP_PIN_GPIO69)
#define MFP_L_DD_15_AF				(MFP_PIN_GPIO69_AF_LCD_LDD_15)

#define MFP_L_LP_DD_15				(MFP_PIN_GPIO69)
#define MFP_L_LP_DD_15_AF			(MFP_PIN_GPIO69_AF_LCD_LDD_15)

#define MFP_L_DD_16				(MFP_PIN_GPIO70)
#define MFP_L_DD_16_AF				(MFP_PIN_GPIO70_AF_LCD_LDD_16)

#define MFP_L_LP_DD_16				(MFP_PIN_GPIO70)
#define MFP_L_LP_DD_16_AF			(MFP_PIN_GPIO70_AF_LCD_LDD_16)

#define MFP_L_DD_17				(MFP_PIN_GPIO71)
#define MFP_L_DD_17_AF				(MFP_PIN_GPIO71_AF_LCD_LDD_17)

#define MFP_L_LP_DD_17				(MFP_PIN_GPIO71)
#define MFP_L_LP_DD_17_AF			(MFP_PIN_GPIO71_AF_LCD_LDD_17)

#define MFP_L_FCLK				(MFP_PIN_GPIO72)
#define MFP_L_FCLK_AF				(MFP_PIN_GPIO72_AF_LCD_L_FCLK)

#define MFP_L_LCLK				(MFP_PIN_GPIO73)
#define MFP_L_LCLK_AF				(MFP_PIN_GPIO73_AF_LCD_L_LCLK)

#define MFP_L_PCLK				(MFP_PIN_GPIO74)
#define MFP_L_PCLK_AF				(MFP_PIN_GPIO74_AF_LCD_L_PCLK)

#define MFP_L_BIAS				(MFP_PIN_GPIO75)
#define MFP_L_BIAS_AF				(MFP_PIN_GPIO75_AF_LCD_L_BIAS)

#define	MFP_TECH_WAKEUP0			(MFP_PIN_GPIO77)
#define	MFP_TECH_WAKEUP0_AF			(MFP_PIN_GPIO77_AF_GPIO_77)

#define MFP_TEST_PAD				(MFP_PIN_GPIO78)
#define MFP_TEST_PAD_AF				(MFP_PIN_GPIO78_AF_GPIO_78)

#define MFP_TECH_WAKEUP1			(MFP_PIN_GPIO79)
#define	MFP_TECH_WAKEUP1_AF			(MFP_PIN_GPIO79_AF_GPIO_79)

#define	MFP_FFDCD				(MFP_PIN_GPIO80)
#define	MFP_FFDCD_AF				(MFP_PIN_GPIO80_AF_UART1_DCD)

#define	MFP_FFDSR				(MFP_PIN_GPIO81)
#define	MFP_FFDSR_AF				(MFP_PIN_GPIO81_AF_UART1_DSR)

#define	MFP_FFRI				(MFP_PIN_GPIO82)
#define	MFP_FFRI_AF				(MFP_PIN_GPIO82_AF_UART1_RI)

#define	MFP_FFDTR				(MFP_PIN_GPIO83)
#define	MFP_FFDTR_AF				(MFP_PIN_GPIO83_AF_UART1_DTR)

#define	MFP_SSP_1_CLK				(MFP_PIN_GPIO85)	
#define	MFP_SSP_1_CLK_AF			(MFP_PIN_GPIO85_AF_SSP1_SCLK)

#define	MPF_SSP_1_FRM				(MFP_PIN_GPIO86)
#define	MFP_SSP_1_FRM_AF			(MFP_PIN_GPIO86_AF_SSP1_FRM)

#define	MFP_SSP_1_TXD				(MFP_PIN_GPIO87)
#define	MFP_SSP_1_TXD_AF			(MFP_PIN_GPIO87_AF_SSP1_TXD)

#define	MFP_SSP_1_RXD				(MFP_PIN_GPIO88)
#define	MFP_SSP_1_RXD_AF			(MFP_PIN_GPIO88_AF_SSP1_RXD)

#define MFP_TECH_WAKEUP2			(MFP_PIN_GPIO89)
#define MFP_TECH_WAKEUP2_AF			(MFP_PIN_GPIO89_AF_GPIO_89)

#define MFP_DEBUG_ETH_INT_GPIO			(MFP_PIN_GPIO90)
#define MFP_DEBUG_ETH_INT_GPIO_AF		(MFP_PIN_GPIO90_AF_GPIO_90)

#define MFP_DEBUG_ETH_INT_GPIO			(MFP_PIN_GPIO90)
#define MFP_DEBUG_ETH_INT_GPIO_AF		(MFP_PIN_GPIO90_AF_GPIO_90)

#define MFP_TECH_WAKEUP3			(MFP_PIN_GPIO90)
#define MFP_TECH_WAKEUP3_AF			(MFP_PIN_GPIO90_AF_GPIO_90)

#define	MFP_ULPI_RESET				(MFP_PIN_GPIO29)                        /* Littleton 4.x */
#define	MFP_ULPI_RESET_AF			(MFP_PIN_GPIO29_AF_GPIO_29)

#define	MFP_WLAN8686_PWDN_GPIO			(MFP_PIN_GPIO53)			/* Littleton 4.x */
#define	MFP_WLAN8686_PWDN_AF			(MFP_PIN_GPIO53_AF_GPIO_53)

#define	MFP_WL_WAKE_MH				(MFP_PIN_GPIO103)			/* Littleton 4.x */
#define	MFP_WL_WAKE_MH_AF			(MFP_PIN_GPIO103_AF_GPIO_103)

#define	MFP_MH_WAKE_WL				(MFP_PIN_GPIO105)			/* Littleton 4.x */
#define	MFP_MH_WAKE_WL_AF			(MFP_PIN_GPIO105_AF_GPIO_105)

#define MFP_SSP_AUDIO_SCLK			(MFP_PIN_GPIO91)
#define MFP_SSP_AUDIO_SCLK_AF			(MFP_PIN_GPIO91_AF_SSP3_SCLK)

#define MFP_SSP_AUDIO_FRM			(MFP_PIN_GPIO92)
#define MFP_SSP_AUDIO_FRM_AF			(MFP_PIN_GPIO92_AF_SSP3_FRM)

#define MFP_SSP_AUDIO_TXD			(MFP_PIN_GPIO93)
#define MFP_SSP_AUDIO_TXD_AF			(MFP_PIN_GPIO93_AF_SSP3_TXD)

#define MFP_SSP_AUDIO_RXD			(MFP_PIN_GPIO94)
#define MFP_SSP_AUDIO_RXD_AF			(MFP_PIN_GPIO94_AF_SSP3_RXD)

#define MFP_SSP_4_CLK				(MFP_PIN_GPIO95)
#define MFP_SSP_4_CLK_AF			(MFP_PIN_GPIO95_AF_SSP4_SCLK)

#define MFP_SSP_4_FRM				(MFP_PIN_GPIO96)
#define MFP_SSP_4_FRM_AF			(MFP_PIN_GPIO96_AF_SSP4_FRM)

#define MFP_SSP_4_TXD				(MFP_PIN_GPIO97)
#define MFP_SSP_4_TXD_AF			(MFP_PIN_GPIO97_AF_SSP4_TXD)

#define MFP_SSP_4_RXD				(MFP_PIN_GPIO98)
#define MFP_SSP_4_RXD_AF			(MFP_PIN_GPIO98_AF_SSP4_RXD)

#define MFP_FFRXD				(MFP_PIN_GPIO99)
#define MFP_FFRXD_AF				(MFP_PIN_GPIO99_AF_UART1_RXD)

#define MFP_FFTXD				(MFP_PIN_GPIO100)
#define MFP_FFTXD_AF				(MFP_PIN_GPIO100_AF_UART1_TXD)

#define MFP_FFCTS				(MFP_PIN_GPIO101)
#define MFP_FFCTS_AF				(MFP_PIN_GPIO101_AF_UART1_CTS)

#define MFP_MMC3_CLK				(MFP_PIN_GPIO103)
#define MFP_MMC3_CLK_AF				(MFP_PIN_GPIO103_AF_MM3_CLK)

#define MFP_WIFI_WAKEUP_HOST			(MFP_PIN_GPIO103)
#define MFP_WIFI_WAKEUP_HOST_AF			(MFP_PIN_GPIO103_AF_UART1_DSR)

#define MFP_MMC3_CMD				(MFP_PIN_GPIO105)
#define MFP_MMC3_CMD_AF				(MFP_PIN_GPIO105_AF_MM3_CMD)

#define MFP_FFRTS				(MFP_PIN_GPIO106)
#define MFP_FFRTS_AF				(MFP_PIN_GPIO106_AF_UART1_RTS)

#define MFP_KP_DKIN_0				(MFP_PIN_GPIO107)
#define MFP_KP_DKIN_0_AF			(MFP_PIN_GPIO107_AF_KP_DKIN_0)

#define MFP_KP_DKIN_1				(MFP_PIN_GPIO108)
#define MFP_KP_DKIN_1_AF			(MFP_PIN_GPIO108_AF_KP_DKIN_1)

#define MFP_STD_TXD				(MFP_PIN_GPIO109)
#define MFP_STD_TXD_AF				(MFP_PIN_GPIO109_AF_UART3_TXD)

#define MFP_STD_RXD				(MFP_PIN_GPIO110)
#define MFP_STD_RXD_AF				(MFP_PIN_GPIO110_AF_UART3_RXD)

#define MFP_BT_RTS				(MFP_PIN_GPIO111)
#define MFP_BT_RTS_AF				(MFP_PIN_GPIO111_AF_UART2_RTS)

#define MFP_BT_RXD				(MFP_PIN_GPIO112)
#define MFP_BT_RXD_AF				(MFP_PIN_GPIO112_AF_UART2_RXD)

#define MFP_BT_TXD				(MFP_PIN_GPIO113)
#define MFP_BT_TXD_AF				(MFP_PIN_GPIO113_AF_UART2_TXD)

#define MFP_BT_CTS				(MFP_PIN_GPIO114)
#define MFP_BT_CTS_AF				(MFP_PIN_GPIO114_AF_UART2_CTS)

#define MFP_KP_MKIN_0				(MFP_PIN_GPIO115)
#define MFP_KP_MKIN_0_AF			(MFP_PIN_GPIO115_AF_KP_MKIN_0)

#define MFP_KP_MKIN_1				(MFP_PIN_GPIO116)
#define MFP_KP_MKIN_1_AF			(MFP_PIN_GPIO116_AF_KP_MKIN_1)

#define MFP_KP_MKIN_2				(MFP_PIN_GPIO117)
#define MFP_KP_MKIN_2_AF			(MFP_PIN_GPIO117_AF_KP_MKIN_2)

#define MFP_KP_MKIN_3				(MFP_PIN_GPIO118)
#define MFP_KP_MKIN_3_AF			(MFP_PIN_GPIO118_AF_KP_MKIN_3)

#define MFP_KP_MKIN_4				(MFP_PIN_GPIO119)
#define MFP_KP_MKIN_4_AF			(MFP_PIN_GPIO119_AF_KP_MKIN_4)

#define MFP_KP_MKIN_5				(MFP_PIN_GPIO120)
#define MFP_KP_MKIN_5_AF			(MFP_PIN_GPIO120_AF_KP_MKIN_5)

#define MFP_KP_MKOUT_0				(MFP_PIN_GPIO121)
#define MFP_KP_MKOUT_0_AF			(MFP_PIN_GPIO121_AF_KP_MKOUT_0)

#define MFP_KP_MKOUT_1				(MFP_PIN_GPIO122)
#define MFP_KP_MKOUT_1_AF			(MFP_PIN_GPIO122_AF_KP_MKOUT_1)

#define MFP_KP_MKOUT_2				(MFP_PIN_GPIO123)
#define MFP_KP_MKOUT_2_AF			(MFP_PIN_GPIO123_AF_KP_MKOUT_2)

#define MFP_KP_MKOUT_3				(MFP_PIN_GPIO124)
#define MFP_KP_MKOUT_3_AF			(MFP_PIN_GPIO124_AF_KP_MKOUT_3)

#define MFP_KP_MKOUT_4				(MFP_PIN_GPIO125)
#define MFP_KP_MKOUT_4_AF			(MFP_PIN_GPIO125_AF_KP_MKOUT_4)

#define MFP_MMC3_DAT0				(MFP_PIN_GPIO7_2)
#define MFP_MMC3_DAT0_AF			(MFP_PIN_GPIO7_2_AF_MM3_DAT0)

#define MFP_MMC3_DAT1				(MFP_PIN_GPIO8_2)
#define MFP_MMC3_DAT1_AF			(MFP_PIN_GPIO8_2_AF_MM3_DAT1)

#define MFP_MMC3_DAT2				(MFP_PIN_GPIO9_2)
#define MFP_MMC3_DAT2_AF			(MFP_PIN_GPIO9_2_AF_MM3_DAT2)

#define MFP_MMC3_DAT3				(MFP_PIN_GPIO10_2)
#define MFP_MMC3_DAT3_AF			(MFP_PIN_GPIO10_2_AF_MM3_DAT3)

#define MFP_ND_CLE				(MFP_PIN_CLE_nOE)
#define MFP_ND_CLE_AF				(MFP_PIN_DF_CLE_AF_ND_CLE)

#define MFP_DF_ALE				(MFP_PIN_DF_ALE_nWE)
#define MFP_DF_ALE_AF				(MFP_PIN_DF_ALE_nWE1_AF_ND_ALE)

#define MFP_DF_NCS1				(MFP_PIN_DF_nCS1)
#define MFP_DF_NCS1_AF				(MFP_PIN_DF_nCS1_AF_DF_nCS1)

#define MFP_DF_SCLK_E				(MFP_PIN_DF_SCLK_E)
#define MFP_DF_SCLK_E_AF			(MFP_PIN_DF_SCLK_E_AF_DF_SCLK_E)

#define MFP_DF_SCLK_S				(MFP_PIN_DF_SCLK_S)
#define MFP_DF_SCLK_S_AF			(MFP_PIN_DF_SCLK_S_AF_DF_SCLK_S)

#define MFP_DEBUG_ETH_nBE0			(MFP_PIN_nBE0)
#define MFP_DEBUG_ETH_nBE0_AF			(MFP_PIN_nBE0_AF_DF_nBE0)

#define MFP_DEBUG_ETH_nBE1			(MFP_PIN_nBE1)
#define MFP_DEBUG_ETH_nBE1_AF			(MFP_PIN_nBE1_AF_DF_nBE1)

#define MFP_DF_INT_RnB				(MFP_PIN_DF_INT_RnB)
#define MFP_DF_INT_RnB_AF			(MFP_PIN_DF_INT_RnB_AF_INT_RnB)

#define MFP_nLLA				(MFP_PIN_nLLA)
#define MFP_nLLA_AF				(MFP_PIN_DF_nLLA_AF_DF_nLLA)

#define MFP_nLUA				(MFP_PIN_nLUA)
#define MFP_nLUA_AF				(MFP_PIN_DF_nLUA_AF_DF_nLUA)

#define MFP_DF_nWE				(MFP_PIN_DF_nWE)
#define MFP_DF_nWE_AF				(MFP_PIN_DF_nWE_AF_ND_WE)

#define MFP_DF_nOE				(MFP_PIN_DF_nRE_nOE)
#define MFP_DF_nOE_AF				(MFP_PIN_DF_nRE_AF_CD_OE)

#define MFP_DF_nRE				(MFP_PIN_DF_nRE)
#define MFP_DF_nRE_AF				(MFP_PIN_DF_nRE_AF_ND_RE)

#define MFP_RSVD_DF_ADDR0			(MFP_PIN_DF_ADDR0)
#define MFP_RSVD_DF_ADDR0_AF			(MFP_PIN_DF_ADDR0_AF_DF_ADDR0)

#define MFP_RSVD_DF_ADDR1			(MFP_PIN_DF_ADDR1)
#define MFP_RSVD_DF_ADDR1_AF			(MFP_PIN_DF_ADDR1_AF_DF_ADDR1)

#define MFP_RSVD_DF_ADDR2			(MFP_PIN_DF_ADDR2)
#define MFP_RSVD_DF_ADDR2_AF			(MFP_PIN_DF_ADDR2_AF_DF_ADDR2)

#define MFP_RSVD_DF_ADDR3			(MFP_PIN_DF_ADDR3)
#define MFP_RSVD_DF_ADDR3_AF			(MFP_PIN_DF_ADDR3_AF_DF_ADDR3)

#define MFP_DF_IO_0				(MFP_PIN_DF_IO0)
#define MFP_DF_IO_0_AF				(MFP_PIN_DF_IO_0_AF_ND)

#define MFP_DF_IO_1				(MFP_PIN_DF_IO1)
#define MFP_DF_IO_1_AF				(MFP_PIN_DF_IO_1_AF_ND)

#define MFP_DF_IO_2				(MFP_PIN_DF_IO2)
#define MFP_DF_IO_2_AF				(MFP_PIN_DF_IO_2_AF_ND)

#define MFP_DF_IO_3				(MFP_PIN_DF_IO3)
#define MFP_DF_IO_3_AF				(MFP_PIN_DF_IO_3_AF_ND)

#define MFP_DF_IO_4				(MFP_PIN_DF_IO4)
#define MFP_DF_IO_4_AF				(MFP_PIN_DF_IO_4_AF_ND)

#define MFP_DF_IO_5				(MFP_PIN_DF_IO5)
#define MFP_DF_IO_5_AF				(MFP_PIN_DF_IO_5_AF_ND)

#define MFP_DF_IO_6				(MFP_PIN_DF_IO6)
#define MFP_DF_IO_6_AF				(MFP_PIN_DF_IO_6_AF_ND)

#define MFP_DF_IO_7				(MFP_PIN_DF_IO7)
#define MFP_DF_IO_7_AF				(MFP_PIN_DF_IO_7_AF_ND)

#define MFP_DF_IO_8				(MFP_PIN_DF_IO8)
#define MFP_DF_IO_8_AF				(MFP_PIN_DF_IO_8_AF_ND)

#define MFP_DF_IO_9				(MFP_PIN_DF_IO9)
#define MFP_DF_IO_9_AF				(MFP_PIN_DF_IO_9_AF_ND)

#define MFP_DF_IO_10				(MFP_PIN_DF_IO10)
#define MFP_DF_IO_10_AF				(MFP_PIN_DF_IO_10_AF_ND)

#define MFP_DF_IO_11				(MFP_PIN_DF_IO11)
#define MFP_DF_IO_11_AF				(MFP_PIN_DF_IO_11_AF_ND)

#define MFP_DF_IO_12				(MFP_PIN_DF_IO12)
#define MFP_DF_IO_12_AF				(MFP_PIN_DF_IO_12_AF_ND)

#define MFP_DF_IO_13				(MFP_PIN_DF_IO13)
#define MFP_DF_IO_13_AF				(MFP_PIN_DF_IO_13_AF_ND)

#define MFP_DF_IO_14				(MFP_PIN_DF_IO14)
#define MFP_DF_IO_14_AF				(MFP_PIN_DF_IO_14_AF_ND)

#define MFP_DF_IO_15				(MFP_PIN_DF_IO15)
#define MFP_DF_IO_15_AF				(MFP_PIN_DF_IO_15_AF_ND)

#define MFP_DF_nADV1				(MFP_PIN_DF_ALE_nWE)
#define MFP_DF_nADV1_AF				(MFP_PIN_DF_ALE_nWE1_AF_ND_ALE)

#define MFP_DF_NCS0				(MFP_PIN_DF_nCS0)
#define MFP_DF_NCS0_AF				(MFP_PIN_DF_nCS0_AF_ND_nCS0)

#define MFP_RSVD_DF_NCS1			(MFP_PIN_DF_nCS1)
#define MFP_RSVD_DF_NCS1_AF			(MFP_PIN_DF_nCS1_AF_ND_nCS1)

#define MFP_SIR_TXD				(MFP_STD_TXD)
#define MFP_SIR_TXD_AF				(MFP_STD_TXD_AF)

#define MFP_SIR_RXD				(MFP_STD_RXD)
#define MFP_SIR_RXD_AF				(MFP_STD_RXD_AF)

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

#define	LITTLETON_ETH_PHYS		0x30000000
#endif

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
