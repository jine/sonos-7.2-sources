/*
 * linux/sound/arm/codec/wm9713.h.
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __ZY_WM9713_H__
#define __ZY_WM9713_H__
#include "acodec.h"


#define    ZY_AC97_CR_WM9713_IO_CTRL_STAT  0x5A  /* I/O pin level [0..9]. R/W. */
#define    ZY_AC97_CR_WM9713_IO_DIRN       0x5C  /* Sets In(0) or Out(1) for I/O pins */
#define    ZY_AC97_CR_WM9713_POS_INT_ENAB  0x5E  /* Enables intrpt sgnl on rising edge */
#define    ZY_AC97_CR_WM9713_NEG_INT_ENAB  0x60  /* Enables intrpt sgnl on falling edge */
#define    ZY_AC97_CR_WM9713_INT_CLR_STAT  0x62  /* Reports intrpt status, W clears stat */
#define    ZY_AC97_CR_WM9713_TS_CTRL       0x64  /* Touch Screen Control */
#define    ZY_AC97_CR_WM9713_ADC_CTRL      0x66  /* ADC Control */
#define    ZY_AC97_CR_WM9713_ADC_DATA      0x68  /* ADC Data */
#define    ZY_AC97_CR_WM9713_FTR_CTRL_STAT1  0x6A  /* Feature control + status reg 1 */
#define    ZY_AC97_CR_WM9713_FTR_CTRL_STAT2  0x6C  /* Feature control + status reg 2 */
#define    ZY_AC97_CR_WM9713_TEST_CTRL       0x6E  /* Only in Vendor Specific Test Mode */

#define    ZY_AC97_WM9713_RR_LOUDNESS ( 0x1 << 5 )    /* Loudness (bass boost) supported */
#define    ZY_AC97_WM9713_RR_20BITDAC ( 0x1 << 7 )    /* supports 20 bit DAC */
#define    ZY_AC97_WM9713_RR_20BITADC ( 0x1 << 9 )    /* supports 20 bit ADC */

/* Master Volume Register (MVR) definitions */

#define ZY_AC97_WM9713_MVR_MR_SHIFT    0               /* Volume Right, 6 bits wide */
/* Bits 6,7 Reserved */
#define ZY_AC97_WM9713_MVR_ML_SHIFT    8               /* Volume Left, 6 bits wide */
/* Bit 14 Reserved */
#define ZY_AC97_WM9713_MVR_MM          ( 0x1 << 15 )   /* Master Mute */

/* MIC Volume Register (MCVR) definitions */

/* Bits 0-5 Reserved */
#define ZY_AC97_WM9713_MCVR_20DB       ( 0x1 << 6 )    /* MIC Volume boosted by 20 dB */
/* Bits 7-15 Reserved */

/* Record Select Register (RSR) definitions */

#define ZY_AC97_WM9713_RSR_SR_SHIFT    0
#define ZY_AC97_WM9713_RSR_SR_CL       ( 0x0 << ZY_AC97_WM9713_RSR_SR_SHIFT ) /* copy from left */
#define ZY_AC97_WM9713_RSR_SR_LINE     ( 0x100 << ZY_AC97_WM9713_RSR_SR_SHIFT )

#define ZY_AC97_WM9713_RSR_SL_SHIFT    8
#define ZY_AC97_WM9713_RSR_SL_MIC      ( 0x0 << ZY_AC97_WM9713_RSR_SL_SHIFT )
#define ZY_AC97_WM9713_RSR_SL_LINE     ( 0x100 << ZY_AC97_WM9713_RSR_SL_SHIFT )

/* Record Gain Register (RGR) definitions */

#define ZY_AC97_WM9713_RGR_GR_SHIFT    0               /* Gain Right, 4 bits wide */
/* Bits 4-7 Reserved */

#define ZY_AC97_WM9713_RGR_GL_SHIFT    8               /* Gain Left, 4 bits wide */

/* Bits 12-14 Reserved */

#define ZY_AC97_WM9713_RGR_RM          ( 0x1 << 15 )   /* Record Mute */

/* General Purpose Register (GPR) definitions */

/* Bits 0-6 Reserved */
#define ZY_AC97_WM9713_GPR_LPBK        ( 0x1 << 7 )    /* ADC/DAC Loopback Mode */
/* Bits 8-15 Reserved */

/* Powerdown Control/Status Register (PCSR) definitions */

#define ZY_AC97_WM9713_PCSR_ADCR   ( 0x1 << 0 )    /* ADC ready to transmit data */
#define ZY_AC97_WM9713_PCSR_DAC    ( 0x1 << 1 )    /* DAC ready to accept data */
/* Bit 2 Reserved */
#define ZY_AC97_WM9713_PCSR_REF    ( 0x1 << 3 )    /* Vref is up to nominal level */
/* Bits 4-7 Reserved */
#define ZY_AC97_WM9713_PCSR_PR0    ( 0x1 << 8 )    /* ADC & input path powerdown */
#define ZY_AC97_WM9713_PCSR_PR1    ( 0x1 << 9 )    /* DAC & ouput path powerdown */
/* Bit 10 Reserved */
#define ZY_AC97_WM9713_PCSR_PR3    ( 0x1 << 11 )   /* Vref powerdown */
#define ZY_AC97_WM9713_PCSR_PR4    ( 0x1 << 12 )   /* Digital interface powerdown */
#define ZY_AC97_WM9713_PCSR_PR5    ( 0x1 << 13 )   /* Internal Clock disable */
/* Bits 14,15 Reserved */

/* Extended Audio ID Register (EAIDR) definitions */

#define ZY_AC97_WM9713_EAIDR_VRA   ( 0x1 << 0 )    /* Variable Rate PCM Audio supported */
/* Bits 1-13 Reserved */
#define ZY_AC97_WM9713_EAIDR_ID    ( 0x11 << 14 )  /* 2 bits wide, Always 0.  UCB1400 is a primary codec */

/* Extended Audio Status and Control Register (EASCR) definitions */

#define ZY_AC97_WM9713_EASCR_VRA   ( 0x1 << 0 )    /* Enable Variable Rate Audio mode */
/* Bits 1-15 Reserved */

/* Audio DAC & ADC Sample Rate Control Register (ADR & AAR) definitions */

#define ZY_AC97_WM9713_DR_8000     0x1F40  /*  8000 samples/sec */
#define ZY_AC97_WM9713_DR_11025    0x2B11  /* 11025 samples/sec */
#define ZY_AC97_WM9713_DR_12000    0x2EE0  /* 12000 samples/sec */
#define ZY_AC97_WM9713_DR_16000    0x3E80  /* 16000 samples/sec */
#define ZY_AC97_WM9713_DR_22050    0x5622  /* 22050 samples/sec */
#define ZY_AC97_WM9713_DR_24000    0x5DC0  /* 24000 samples/sec */
#define ZY_AC97_WM9713_DR_32000    0x7D00  /* 32000 samples/sec */
#define ZY_AC97_WM9713_DR_44100    0xAC44  /* 44100 samples/sec */
#define ZY_AC97_WM9713_DR_48000    0xBB80  /* 48000 samples/sec */

/* I/O Data Register (IODR) and I/O Data Direction (IODIRR) definitions */
#define ZY_AC97_WM9713_IO0 ( 0x1 << 0 )
#define ZY_AC97_WM9713_IO1 ( 0x1 << 1 )
#define ZY_AC97_WM9713_IO2 ( 0x1 << 2 )
#define ZY_AC97_WM9713_IO3 ( 0x1 << 3 )
#define ZY_AC97_WM9713_IO4 ( 0x1 << 4 )
#define ZY_AC97_WM9713_IO5 ( 0x1 << 5 )
#define ZY_AC97_WM9713_IO6 ( 0x1 << 6 )
#define ZY_AC97_WM9713_IO7 ( 0x1 << 7 )
#define ZY_AC97_WM9713_IO8 ( 0x1 << 8 )
#define ZY_AC97_WM9713_IO9 ( 0x1 << 9 )
/* Bits 10-15 Reserved */

/* Positive INT Enable Register (PIER) definitions */

#define ZY_AC97_WM9713_PIER_ION0   ( 0x1 << 0 )    /* enable falling edge interrupt for I/O pin 0 */
#define ZY_AC97_WM9713_PIER_ION1   ( 0x1 << 1 )    /* enable falling edge interrupt for I/O pin 1 */
#define ZY_AC97_WM9713_PIER_ION2   ( 0x1 << 2 )    /* enable falling edge interrupt for I/O pin 2 */
#define ZY_AC97_WM9713_PIER_ION3   ( 0x1 << 3 )    /* enable falling edge interrupt for I/O pin 3 */

#define ZY_AC97_WM9713_PIER_ION4   ( 0x1 << 4 )    /* enable falling edge interrupt for I/O pin 4 */
#define ZY_AC97_WM9713_PIER_ION5   ( 0x1 << 5 )    /* enable falling edge interrupt for I/O pin 5 */
#define ZY_AC97_WM9713_PIER_ION6   ( 0x1 << 6 )    /* enable falling edge interrupt for I/O pin 6 */
#define ZY_AC97_WM9713_PIER_ION7   ( 0x1 << 7 )    /* enable falling edge interrupt for I/O pin 7 */

#define ZY_AC97_WM9713_PIER_ION8   ( 0x1 << 8 )    /* enable falling edge interrupt for I/O pin 8 */
#define ZY_AC97_WM9713_PIER_ION9   ( 0x1 << 9 )    /* enable falling edge interrupt for I/O pin 9 */
#define ZY_AC97_WM9713_PIER_D10    ( 0x1 << 10 )   /* Reserved */
#define ZY_AC97_WM9713_PIER_ADCP   ( 0x1 << 11 )   /* enable falling edge interrupt for ADC Ready */

#define ZY_AC97_WM9713_PIER_TPXP   ( 0x1 << 12 )   /* enable falling edge interrupt for TSPX */
#define ZY_AC97_WM9713_PIER_TMXP   ( 0x1 << 13 )   /* enable falling edge interrupt for TSMX */
#define ZY_AC97_WM9713_PIER_D14    ( 0x1 << 14 )   /* Reserved */
#define ZY_AC97_WM9713_PIER_OVLP   ( 0x1 << 15 )   /* enable falling edge interrupt for OVFL */

/* Negative INT Enable Register (NIER) definitions */

#define ZY_AC97_WM9713_NIER_ION0   ( 0x1 << 0 )    /* enable falling edge interrupt for I/O pin 0 */
#define ZY_AC97_WM9713_NIER_ION1   ( 0x1 << 1 )    /* enable falling edge interrupt for I/O pin 1 */
#define ZY_AC97_WM9713_NIER_ION2   ( 0x1 << 2 )    /* enable falling edge interrupt for I/O pin 2 */
#define ZY_AC97_WM9713_NIER_ION3   ( 0x1 << 3 )    /* enable falling edge interrupt for I/O pin 3 */

#define ZY_AC97_WM9713_NIER_ION4   ( 0x1 << 4 )    /* enable falling edge interrupt for I/O pin 4 */
#define ZY_AC97_WM9713_NIER_ION5   ( 0x1 << 5 )    /* enable falling edge interrupt for I/O pin 5 */
#define ZY_AC97_WM9713_NIER_ION6   ( 0x1 << 6 )    /* enable falling edge interrupt for I/O pin 6 */
#define ZY_AC97_WM9713_NIER_ION7   ( 0x1 << 7 )    /* enable falling edge interrupt for I/O pin 7 */

#define ZY_AC97_WM9713_NIER_ION8   ( 0x1 << 8 )    /* enable falling edge interrupt for I/O pin 8 */
#define ZY_AC97_WM9713_NIER_ION9   ( 0x1 << 9 )    /* enable falling edge interrupt for I/O pin 9 */
#define ZY_AC97_WM9713_NIER_D10    ( 0x1 << 10 )   /* Reserved */
#define ZY_AC97_WM9713_NIER_ADCN   ( 0x1 << 11 )   /* enable falling edge interrupt for ADC Ready */

#define ZY_AC97_WM9713_NIER_TPXN   ( 0x1 << 12 )   /* enable falling edge interrupt for TSPX */
#define ZY_AC97_WM9713_NIER_TMXN   ( 0x1 << 13 )   /* enable falling edge interrupt for TSMX */
#define ZY_AC97_WM9713_NIER_D14    ( 0x1 << 14 )   /* Reserved */
#define ZY_AC97_WM9713_NIER_OVLN   ( 0x1 << 15 )   /* enable falling edge interrupt for OVFL */

/* INT Clear/Status Register (ICSR) definitions */

#define ZY_AC97_WM9713_ICSR_IOS0   ( 0x1 << 0 )    /* use to check or clear the int status for IO Bit 0 */
#define ZY_AC97_WM9713_ICSR_IOS1   ( 0x1 << 1 )    /* use to check or clear the int status for IO Bit 1 */
#define ZY_AC97_WM9713_ICSR_IOS2   ( 0x1 << 2 )    /* use to check or clear the int status for IO Bit 2 */
#define ZY_AC97_WM9713_ICSR_IOS3   ( 0x1 << 3 )    /* use to check or clear the int status for IO Bit 3 */

#define ZY_AC97_WM9713_ICSR_IOS4   ( 0x1 << 4 )    /* use to check or clear the int status for IO Bit 4 */
#define ZY_AC97_WM9713_ICSR_IOS5   ( 0x1 << 5 )    /* use to check or clear the int status for IO Bit 5 */
#define ZY_AC97_WM9713_ICSR_IOS6   ( 0x1 << 6 )    /* use to check or clear the int status for IO Bit 6 */
#define ZY_AC97_WM9713_ICSR_IOS7   ( 0x1 << 7 )    /* use to check or clear the int status for IO Bit 7 */

#define ZY_AC97_WM9713_ICSR_IOS8   ( 0x1 << 8 )    /* use to check or clear the int status for IO Bit 8 */
#define ZY_AC97_WM9713_ICSR_IOS9   ( 0x1 << 9 )    /* use to check or clear the int status for IO Bit 9 */
#define ZY_AC97_WM9713_ICSR_D10    ( 0x1 << 10 )   /* Reserved */
#define ZY_AC97_WM9713_ICSR_ADCS   ( 0x1 << 11 )   /* use to check or clear the int status for ADC ready */

#define ZY_AC97_WM9713_ICSR_TSPX   ( 0x1 << 12 )   /* use to check or clear the int status for TSPX */
#define ZY_AC97_WM9713_ICSR_TSMX   ( 0x1 << 13 )   /* use to check or clear the int status for TSMX */
#define ZY_AC97_WM9713_ICSR_D14    ( 0x1 << 14 )   /* Reserved */
#define ZY_AC97_WM9713_ICSR_OVLS   ( 0x1 << 15 )   /* use to check or clear the int status for OVFL */

/* Touch Screen Control Register (TSCR) defintions */

#define ZY_AC97_WM9713_TSCR_TSMX_POW   ( 0x1 << 0 )    /* TSMX pin is powered */
#define ZY_AC97_WM9713_TSCR_TSPX_POW   ( 0x1 << 1 )    /* TSPX pin is powered */
#define ZY_AC97_WM9713_TSCR_TSMY_POW   ( 0x1 << 2 )    /* TSMY pin is powered */
#define ZY_AC97_WM9713_TSCR_TSPY_POW   ( 0x1 << 3 )    /* TSPY pin is powered */

#define ZY_AC97_WM9713_TSCR_TSMX_GND   ( 0x1 << 4 )    /* TSMX pin is grounded */
#define ZY_AC97_WM9713_TSCR_TSPX_GND   ( 0x1 << 5 )    /* TSPX pin is grounded */
#define ZY_AC97_WM9713_TSCR_TSMY_GND   ( 0x1 << 6 )    /* TSMY pin is grounded */
#define ZY_AC97_WM9713_TSCR_TSPY_GND   ( 0x1 << 7 )    /* TSPY pin is grounded */

#define ZY_AC97_WM9713_TSCR_INTMO      ( 0x0 << 8 )    /* Interrupt Mode */
#define ZY_AC97_WM9713_TSCR_PREMO      ( 0x1 << 8 )    /* Pressure Measurement Mode */
#define ZY_AC97_WM9713_TSCR_POSMO      ( 0x2 << 8 )    /* Position Measurement Mode */
#define ZY_AC97_WM9713_TSCR_HYSD       ( 0x1 << 10 )   /* Hysteresis deactivated */
#define ZY_AC97_WM9713_TSCR_BIAS       ( 0x1 << 11 )   /* Bias circuitry activated */

#define ZY_AC97_WM9713_TSCR_PX         ( 0x1 << 12 )   /* Inverted state of TSPX pin */
#define ZY_AC97_WM9713_TSCR_MX         ( 0x1 << 13 )   /* Inverted state of TSMX pin */
#define ZY_AC97_WM9713_TSCR_D14        ( 0x1 << 14 )   /* Reserved */
#define ZY_AC97_WM9713_TSCR_D15        ( 0x1 << 15 )   /* Reserved */

/* ADC Control Register (ADCCR) definitions */

#define ZY_AC97_WM9713_ADCCR_ASE       ( 0x1 << 0 ) /* ADC is armed by AS bit and started by rising edge on ADCSYNC pin */
#define ZY_AC97_WM9713_ADCCR_D1        ( 0x1 << 1 ) /* Reserved */

#define ZY_AC97_WM9713_ADCCR_AI_SHIFT  2
#define ZY_AC97_WM9713_ADCCR_AI_TSPX   ( 0x0 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is TSPX */
#define ZY_AC97_WM9713_ADCCR_AI_TSMX   ( 0x1 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is TSMX */
#define ZY_AC97_WM9713_ADCCR_AI_TSPY   ( 0x2 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is TSPY */
#define ZY_AC97_WM9713_ADCCR_AI_TSMY   ( 0x3 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is TSMY */
#define ZY_AC97_WM9713_ADCCR_AI_AD0    ( 0x4 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is AD0 */
#define ZY_AC97_WM9713_ADCCR_AI_AD1    ( 0x5 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is AD1 */
#define ZY_AC97_WM9713_ADCCR_AI_AD2    ( 0x6 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is AD2 */
#define ZY_AC97_WM9713_ADCCR_AI_AD3    ( 0x7 << ZY_AC97_WM9713_ADCCR_AI_SHIFT )   /* ADC source is AD3 */

#define ZY_AC97_WM9713_ADCCR_D5        ( 0x1 << 5 )    /* Reserved */
#define ZY_AC97_WM9713_ADCCR_D6        ( 0x1 << 6 )    /* Reserved */
#define ZY_AC97_WM9713_ADCCR_AS        ( 0x1 << 7 )    /* Start the ADC conversion seq. */

#define ZY_AC97_WM9713_ADCCR_D8        ( 0x1 << 8 )    /* Reserved */
#define ZY_AC97_WM9713_ADCCR_D9        ( 0x1 << 9 )    /* Reserved */
#define ZY_AC97_WM9713_ADCCR_D10       ( 0x1 << 10 )   /* Reserved */
#define ZY_AC97_WM9713_ADCCR_D11       ( 0x1 << 11 )   /* Reserved */

#define ZY_AC97_WM9713_ADCCR_D12       ( 0x1 << 12 )   /* Reserved */
#define ZY_AC97_WM9713_ADCCR_D13       ( 0x1 << 13 )   /* Reserved */
#define ZY_AC97_WM9713_ADCCR_D14       ( 0x1 << 14 )   /* Reserved */
#define ZY_AC97_WM9713_ADCCR_AE        ( 0x1 << 15 )   /* ADC is activated */

/* ADC Data Register (ADCDR) definitions */

#define ZY_AC97_WM9713_ADCDR_MASK      0x3FF           /* ADC data register data mask */
#define ZY_AC97_WM9713_ADCDR_ADV       ( 0x1 << 15 )   /* Conversion complete */

/* Feature Control/Status Register 1 (FCSR1) definitions */

#define ZY_AC97_WM9713_FCSR1_OVFL      ( 0x1 << 0 )    /* ADC overflow status */
/* bit 1 is reserved */
#define ZY_AC97_WM9713_FCSR1_GIEN      ( 0x1 << 2 )    /* Enable interrupt/wakeup signaling */
#define ZY_AC97_WM9713_FCSR1_HIPS      ( 0x1 << 3 )    /* Activate ADC High Pass Filter */
#define ZY_AC97_WM9713_FCSR1_DC        ( 0x1 << 4 )    /* DC filter is enabled */
#define ZY_AC97_WM9713_FCSR1_DE        ( 0x1 << 5 )    /* De-emphasis is enabled */
#define ZY_AC97_WM9713_FCSR1_XTM       ( 0x1 << 6 )    /* Crystal Oscillator Powerdown Mode */

#define ZY_AC97_WM9713_FCSR1_M_SHIFT   7
#define ZY_AC97_WM9713_FCSR1_M_FLAT    ( 0x00 << ZY_AC97_WM9713_FCSR1_M_SHIFT )   /* Flat mode */
#define ZY_AC97_WM9713_FCSR1_M_MIN1    ( 0x1 << ZY_AC97_WM9713_FCSR1_M_SHIFT )    /* Minimum mode */
#define ZY_AC97_WM9713_FCSR1_M_MIN2    ( 0x2 << ZY_AC97_WM9713_FCSR1_M_SHIFT )    /* Minimum mode */
#define ZY_AC97_WM9713_FCSR1_M_MAX     ( 0x3 << ZY_AC97_WM9713_FCSR1_M_SHIFT )    /* Maximum mode */

#define ZY_AC97_WM9713_FCSR1_TR_SHIFT  9   /* 2 bits wide, Treble Boost */

#define ZY_AC97_WM9713_FCSR1_BB_SHIFT  11  /* 4 bits wide, Bass Boost */
/* Bit 15 Reserved */

/* Feature Control/Status Register 2 (FCSR2) definitions */

#define ZY_AC97_WM9713_FCSR2_EV_SHIFT   0
#define ZY_AC97_WM9713_FCSR2_EV_MASK    ( 0x7 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Mask for reading or clearing EV */
#define ZY_AC97_WM9713_FCSR2_EV_NORMOP  ( 0x0 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for normal operation */
#define ZY_AC97_WM9713_FCSR2_EV_ACLPBK  ( 0x1 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for ACLink loopback */
#define ZY_AC97_WM9713_FCSR2_EV_BSLPBK  ( 0x2 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for bitstr loopback */
#define ZY_AC97_WM9713_FCSR2_EV_DACEVAL ( 0x3 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for DAC bitstr eval */
#define ZY_AC97_WM9713_FCSR2_EV_ADCEVAL ( 0x4 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for ADC bitstr eval */
#define ZY_AC97_WM9713_FCSR2_EV_CLKEVAL ( 0x5 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for Clocks eval mode */
#define ZY_AC97_WM9713_FCSR2_EV_ADC10EV ( 0x6 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for 10 bit ADC eval mode */
#define ZY_AC97_WM9713_FCSR2_EV_NORMOP1 ( 0x7 << ZY_AC97_WM9713_FCSR2_EV_SHIFT ) /* Set EV for normal operation */
/* Bit 3 Reserved */

#define ZY_AC97_WM9713_FCSR2_SLP_SHIFT     4
#define ZY_AC97_WM9713_FCSR2_SLP_MASK      ( 0x3 << ZY_AC97_WM9713_FCSR2_SLP_SHIFT )  /* Mask for reading or clearing SLP */
#define ZY_AC97_WM9713_FCSR2_SLP_NSLP      ( 0x0 << ZY_AC97_WM9713_FCSR2_SLP_SHIFT )  /* No Smart Low Power Mode */
#define ZY_AC97_WM9713_FCSR2_SLP_SLPC      ( 0x1 << ZY_AC97_WM9713_FCSR2_SLP_SHIFT )  /* Smart Low Power Codec */
#define ZY_AC97_WM9713_FCSR2_SLP_SLPPLL    ( 0x2 << ZY_AC97_WM9713_FCSR2_SLP_SHIFT )  /* Smart Low Power PLL */
#define ZY_AC97_WM9713_FCSR2_SLP_SLPALL    ( 0x3 << ZY_AC97_WM9713_FCSR2_SLP_SHIFT )  /* Smart Low Power Codec & PLL */
/* Bits 6-15 Reserved */

/* Test Control Register (TCR) definitions */
#define ZY_AC97_WM9713_TCR_IDDQ    ( 0x1 << 0 )    /* IDDQ testing */
#define ZY_AC97_WM9713_TCR_ROM     ( 0x1 << 1 )    /* ROM testing */
#define ZY_AC97_WM9713_TCR_RAM     ( 0x1 << 2 )    /* RAM testing */
#define ZY_AC97_WM9713_TCR_VOH     ( 0x1 << 3 )    /* VOH testing */
#define ZY_AC97_WM9713_TCR_VOL     ( 0x1 << 4 )    /* VOL testing */
#define ZY_AC97_WM9713_TCR_TRI     ( 0x1 << 5 )    /* Tri-Sate testing */
/* Bits 7-15 Reserved */

#define ZY_AC97_WM9713_GPIO_PIN_PDN ( 0x1 << 13 )  /* Pen down */



#define ZY_AC97_WM9713_MAX_VOLUME    32
#define ZY_AC97_WM9713_MAX_ADCGAIN   15



/* 9713 specific
 *	Register Name			Index
 */
#define RESET				0X00
#define SPEAKER_VOLUME 			0X02
#define HEADPHONE_VOLUME		0X04
#define OUT3_OUT4_VOLUME  		0X06
#define MONOVOL_MONOINPGA_ROUTE		0X08
#define LINE_IN_PGA_VOL_ROUTE		0X0A
#define DAC_PGA_VOL_ROUTE		0X0C
#define MIC_PGA_VOLUME			0X0E
#define MIC_ROUTING			0X10
#define REC_PGA_VOL			0X12
#define REC_ROUTE_MUX_SEL		0X14
#define PCBEEP_VOL_ROUTE		0X16
#define VXDAC_VOLUME_ROUTE		0X18
#define AUX_DAC_VOL_ROUTE		0X1A
#define OUTPUT_PGA_MUX			0X1C
#define DAC_3D_CTRL_INV_MUX_SEL		0X1E
#define DAC_TONE_CTRL			0X20
#define MIC_BIAS			0X22
#define OUTPUT_VOL_MAPPING_JACK		0X24
#define POWERDOWN_CTRL_STAT		0X26
#define EXTENDED_AUD_ID			0X28
#define EXTENDED_AUD_STAT_CTRL  	0X2A
#define AUDIO_DAC_RATE			0X2C
#define AUX_DAC_RATE			0X2E
#define AUDIO_ADC_RATE			0X32
#define PCM_CODEC_CTRL			0X36
#define SPDIF_CTRL			0X3A
#define POWER_DOWN_1			0X3C
#define POWER_DOWN_2			0X3E
#define GENERAL_PURPOSE_WM_13		0X40
#define FAST_POWERUP_CTRL		0X42
#define MCLK_PLL_CTRL_1			0X44
#define MCLK_PLL_CTRL_2			0X46
#define GPIO_PIN_CFG			0X4C
#define GPIO_PIN_POL_TYPE		0X4E
#define GPIO_PIN_STICKY 		0X50
#define GPIO_PIN_WAKEUP			0X52
#define GPIO_PIN_STATUS			0X54
#define GPIO_PIN_SHARING		0X56
#define GPIO_PULL_UP_DOWN_CTRL		0X58
#define ADD_FUNC_1         		0X5A
#define ADD_FUNC_2        		0X5C
#define ALC_CTRL         		0X60
#define ALC_NOISE_GATE_CTRL		0X62
#define AUX_DAC_INPUT_CTRL 		0X64
#define TEST_REG_1 			0X68
#define TEST_REG_2 			0X6A
#define TEST_REG_3 			0X6C
#define TEST_REG_4 			0X6E
#define DIGITIZER_1_WM13	    	0x74
#define DIGITIZER_2_WM13	    	0x76
#define DIGITIZER_3_WM13	    	0x78
#define DIGITIZER_READ_BACK	    	0x7a
#define VENDOR_ID1	  		0x7c
#define VENDOR_ID2	  		0x7e

/*power enable bit in POWERDOWN_CTRL_STAT 26h */
#define ZY_AC97_9713_PR6    ( 0x1 << 14 )
#define ZY_AC97_9713_PR5    ( 0x1 << 13 )
#define ZY_AC97_9713_PR4    ( 0x1 << 12 )
#define ZY_AC97_9713_PR3    ( 0x1 << 11 )
#define ZY_AC97_9713_PR2    ( 0x1 << 10 )
#define ZY_AC97_9713_PR1    ( 0x1 << 9 )
#define ZY_AC97_9713_PR0    ( 0x1 << 8 )

/*power enable bit in 3ch and 3eh */
/*3ch */
#define ZY_AC97_9713_PWR_PADCPD   ( 0x1 << 15 )
#define ZY_AC97_9713_PWR_VMID     ( 0x1 << 14 )
#define ZY_AC97_9713_PWR_TSHUT    ( 0x1 << 13 )
#define ZY_AC97_9713_PWR_VXDAC    ( 0x1 << 12 )
#define ZY_AC97_9713_PWR_AUXDAC   ( 0x1 << 11 )
#define ZY_AC97_9713_PWR_MBIAS    ( 0x1 << 10 )
#define ZY_AC97_9713_PWR_PLL      ( 0x1 << 9 )
#define ZY_AC97_9713_PWR_DACL     ( 0x1 << 7 )
#define ZY_AC97_9713_PWR_DACR     ( 0x1 << 6 )
#define ZY_AC97_9713_PWR_ADCL     ( 0x1 << 5 )
#define ZY_AC97_9713_PWR_ADCR     ( 0x1 << 4 )
#define ZY_AC97_9713_PWR_HPLX     ( 0x1 << 3 )
#define ZY_AC97_9713_PWR_HPRX     ( 0x1 << 2 )
#define ZY_AC97_9713_PWR_SPKX     ( 0x1 << 1 )
#define ZY_AC97_9713_PWR_MX       ( 0x1 << 0 )

/*3EH */
#define ZY_AC97_9713_PWR_MCD      ( 0x1 << 15 )
#define ZY_AC97_9713_PWR_MICBIAS  ( 0x1 << 14 )
#define ZY_AC97_9713_PWR_MONO     ( 0x1 << 13 )
#define ZY_AC97_9713_PWR_OUT4     ( 0x1 << 12 )
#define ZY_AC97_9713_PWR_OUT3     ( 0x1 << 11 )
#define ZY_AC97_9713_PWR_HPL      ( 0x1 << 10 )
#define ZY_AC97_9713_PWR_HPR      ( 0x1 << 9 )
#define ZY_AC97_9713_PWR_SPKL     ( 0x1 << 8 )
#define ZY_AC97_9713_PWR_SPKR     ( 0x1 << 7 )
#define ZY_AC97_9713_PWR_LL       ( 0x1 << 6 )
#define ZY_AC97_9713_PWR_LR       ( 0x1 << 5 )
#define ZY_AC97_9713_PWR_MOIN     ( 0x1 << 4 )
#define ZY_AC97_9713_PWR_MA       ( 0x1 << 3 )
#define ZY_AC97_9713_PWR_MB       ( 0x1 << 2 )
#define ZY_AC97_9713_PWR_MPA      ( 0x1 << 1 )
#define ZY_AC97_9713_PWR_MPB      ( 0x1 << 0 )

#define VRA_ENABLED_MASK		0x1

/*00, 28h, 54h, 68h, 6ah, 6ch, 6eh, 74h, 7ah, 7ch, 7eh need not save */
#define WM9713_SAVE_REGISTER_NO (64-11)
typedef struct {
    unsigned short wm9713RegisterContext [WM9713_SAVE_REGISTER_NO + 1]; /* Fixed (data misalignment error) */
}ZY_9713_CONTEXT_SAVE_T;

#define CODEC_CONTEXT_SIZE (sizeof(ZY_9713_CONTEXT_SAVE_T))

#define HANDSET_MIC_MASK     1
#define HEADSET_MIC_MASK     2

#define ZY_TOUCH_SAMPLE_X 1
#define ZY_TOUCH_SAMPLE_Y 2
#define WM9713_HIFI_NEAR_IN_ROUTE                     (1 << ZY_INDEX_HIFI_NEAR_IN_CONTROL)
#define WM9713_VOICE_NEAR_IN_ROUTE                  (1 << ZY_INDEX_VOICE_NEAR_IN_CONTROL)
#define WM9713_BT_HIFI_NEAR_IN_ROUTE               (1 << ZY_INDEX_BT_HIFI_NEAR_IN_CONTROL)
#define WM9713_BT_VOICE_NEAR_IN_ROUTE             (1 << ZY_INDEX_BT_VOICE_NEAR_IN_CONTROL)
#define WM9713_BT_MIC_IN_ROUTE                           (1 << ZY_INDEX_BT_MIC_IN_CONTROL)
#define WM9713_MIC1_IN_ROUTE                               (1 << ZY_INDEX_MIC1_IN_CONTROL)
#define WM9713_MIC2_IN_ROUTE                               (1 << ZY_INDEX_MIC2_IN_CONTROL)
#define WM9713_FAR_IN_ROUTE                                 (1 << ZY_INDEX_FAR_IN_CONTROL)
#define WM9713_MIDI_IN_ROUTE                               (1 << ZY_INDEX_MIDI_IN_CONTROL)
#define WM9713_FM_IN_ROUTE                                 (1 << ZY_INDEX_FM_IN_CONTROL)
/*ouput route control */
#define WM9713_STEREO_HEAD_SET_OUT_ROUTE       (1<<(ZY_INDEX_STEREO_HEAD_SET_OUT_CONTROL +16))
#define WM9713_SPEAKER_OUT_ROUTE                        (1<<(ZY_INDEX_SPEAKER_OUT_CONTROL + 16))
#define WM9713_HAND_SET_OUT_ROUTE                     (1<<(ZY_INDEX_HAND_SET_OUT_CONTROL +16))
#define WM9713_BT_STEREO_HEAD_SET_OUT_ROUTE (1<<(ZY_INDEX_BT_STEREO_HEAD_SET_OUT_CONTROL +16))
#define WM9713_VOICE_NEAR_OUT_ROUTE                  (1<<(ZY_INDEX_VOICE_NEAR_OUT_CONTROL + 16))
#define WM9713_HIFI_NEAR_OUT_ROUTE                    (1<<(ZY_INDEX_HIFI_NEAR_OUT_CONTROL + 16))
#define WM9713_FAR_OUT_ROUTE                                (1<<(ZY_INDEX_FAR_OUT_CONTROL + 16))

/*WM9713 related functions*/
extern acodec_error_t zy_wm9713_set_master_vol(acodec_context_t *p_device_context, unsigned short gain_in_db);
extern acodec_error_t zy_wm9713_set_master_input_gain(acodec_context_t *p_device_context, unsigned short gain_in_db);
extern acodec_error_t zy_wm9713_get_in_sample_rate(acodec_context_t *p_device_context, unsigned short * rate_in_hz);
extern acodec_error_t zy_wm9713_get_out_sample_rate (acodec_context_t *p_device_context, unsigned short * rate_in_hz);
extern acodec_error_t zy_wm9713_set_in_sample_rate(acodec_context_t *p_device_context, unsigned short rate_in_hz);
extern acodec_error_t zy_wm9713_set_out_sample_rate (acodec_context_t *p_device_context, unsigned short rate_in_hz);
extern acodec_error_t zy_wm9713_specific_init (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_specific_deinit (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_hifi_stream_path_enable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_hifi_stream_path_disable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_recording_path_enable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_recording_path_disable (acodec_context_t *p_device_context);

extern acodec_error_t zy_wm9713_hpmixer_enable(acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_hpmixer_disable(acodec_context_t *p_device_context);

extern acodec_error_t zy_wm9713_headset_enable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_headset_disable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_ear_speaker_enable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_ear_speaker_disable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_louder_speaker_enable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_louder_speaker_disable (acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_mic_enable (acodec_context_t *p_device_context, unsigned char mic_type);
extern acodec_error_t zy_wm9713_mic_disable (acodec_context_t *p_device_context, unsigned char mic_type);
extern acodec_error_t zy_wm9713_wakeup(acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_sleep(acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_set_route(acodec_context_t * p_device_context, unsigned short * rout_map, unsigned short * current_map);
extern acodec_error_t zy_wm9713_set_vol(acodec_context_t * p_device_context, vol_port_type_t port, unsigned short gain_in_db);
extern acodec_error_t zy_wm9713_get_vol(acodec_context_t * p_device_context, vol_port_type_t port, unsigned short *gain_in_db);
extern acodec_error_t zy_acodec_set_pen_down_interrupt(acodec_context_t *p_device_context, int enable);
extern acodec_error_t zy_wm9713_enable_touch(acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_disable_touch(acodec_context_t *p_device_context);
extern acodec_error_t zy_acodec_get_adc_sample(acodec_context_t *p_device_context, unsigned short *p_sample_data, unsigned short adc_type, int *p_pen_down);
extern acodec_error_t zy_wm9713_set_voice_out_sample_rate(acodec_context_t *p_device_context, unsigned short rate_in_hz);
extern acodec_error_t zy_wm9713_voice_prepare(acodec_context_t *p_device_context);
extern acodec_error_t zy_wm9713_set_voice_in_sample_rate(acodec_context_t *p_device_context, unsigned short rate_in_hz);
extern void zy_wm9713_get_event(acodec_context_t *p_device_context, unsigned char * event_type);
extern void zy_wm9713_event_ack(acodec_context_t *p_device_context, unsigned char event_type);

#endif
