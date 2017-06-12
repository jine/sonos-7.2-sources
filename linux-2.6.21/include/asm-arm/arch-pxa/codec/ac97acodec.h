/*
 * linux/sound/arm/codec/ac97acodec.h
 *
 *  Author:	Jingqing.Xu@intel.com
 *  Created:	July 11, 2005
 *  Copyright:	Intel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __ZY_AC97ACODEC_H__
#define __ZY_AC97ACODEC_H__
#include "acodec.h"

/*
*******************************************************************************
    AC'97 Controller Registers Structure and Bit Definition
*******************************************************************************
*/

#define ZY_AC97_CODEC_REGS_NUM        0x40

typedef struct
{  //   Register symbol     // Usage
    volatile unsigned long pocr;           // PCM Out Control Register
    volatile unsigned long picr;           // PCM In Control Register
    volatile unsigned long mccr;           // Mic In Control Register
    volatile unsigned long gcr;            // Global Control Register
    volatile unsigned long posr;           // PCM Out Status Register
    volatile unsigned long pisr;           // PCM In Status Register
    volatile unsigned long mcsr;           // Mic In Status Register
    volatile unsigned long gsr;            // Global Status Register
    volatile unsigned long car;            // CODEC Access Register
    volatile unsigned long pcscr;          // PCM Surround Out Control
    volatile unsigned long pcssr;          // PCM Surround Out Status
    volatile unsigned long pcsdr;          // PCM Surround Out Data
    volatile unsigned long pcclcr;         // PCM Center/LFE Out Control
    volatile unsigned long pcclsr;         // PCM Center/LFE Out Status
    volatile unsigned long pccldr;         // PCM Center/LFE Out Data
    volatile unsigned long reserved1;       //
    volatile unsigned long pcdr;           // PCM FIFO Data Register
    volatile unsigned long reserved2 [0x7];      // 0x4050-0044 through 0x4050-005C
    volatile unsigned long mcdr;           // Mic-in FIFO Data Register
    volatile unsigned long reserved3 [0x27];   // 0x4050-0064 through 0x4050-00FC
    volatile unsigned long mocr;           // MODEM Out Control Register
    volatile unsigned long reserved4;
    volatile unsigned long micr;           // MODEM In Control Register
    volatile unsigned long reserved5;
    volatile unsigned long mosr;           // MODEM Out Status Register
    volatile unsigned long reserved6;
    volatile unsigned long misr;           // MODEM In Status Register
    volatile unsigned long reserved7 [0x9];      // 0x4050-011C through 0x4050-013C
    volatile unsigned long modr;           // MODEM FIFO Data Register
    volatile unsigned long reserved8 [0x2F];   // 0x4050-0144 through 0x4050-01FC
                            // Primary Audio CODEC registers access
    volatile unsigned long codec_regs_primary_aud   [ZY_AC97_CODEC_REGS_NUM];
                            // Secondary ID 01 Audio CODEC registers access
    volatile unsigned long codec_regs_secondary_aud [ZY_AC97_CODEC_REGS_NUM];
                            // Primary MODEM CODEC registers access
    volatile unsigned long codec_regs_primary_mdm   [ZY_AC97_CODEC_REGS_NUM];
                            // Secondary ID 01 MODEM CODEC registers access
    volatile unsigned long codec_regs_secondary_mdm [ZY_AC97_CODEC_REGS_NUM];
                            // Secondary ID 10 MODEM CODEC registers access
    volatile unsigned long codec_regs_third_mdm [ZY_AC97_CODEC_REGS_NUM];
                            // Secondary ID 11 MODEM CODEC registers access
    volatile unsigned long codec_regs_fouth_mdm [ZY_AC97_CODEC_REGS_NUM];
                            // Secondary ID 10 Audio CODEC registers access
    volatile unsigned long codec_regs_third_aud [ZY_AC97_CODEC_REGS_NUM];
                            // Secondary ID 11 Audio CODEC registers access
    volatile unsigned long codec_regs_fouth_aud [ZY_AC97_CODEC_REGS_NUM];
}  zy_ac97_acodec_t, *p_zy_ac97acodec_t ;


/* Constants for the Global Control Register and Global Status Register */

// AC97 Global Control Register bit mask constants

#define ZY_AC97_GCR_GIE_MSK          (1u << 0 )
#define ZY_AC97_GCR_COLD_RESET_MSK   (1u << 1 )
#define ZY_AC97_GCR_WARM_RESET_MSK   (1u << 2 )
#define ZY_AC97_GCR_LINK_OFF_MSK     (1u << 3 )
#define ZY_AC97_GCR_PCRSM_IEN_MSK    (1u << 4 )
#define ZY_AC97_GCR_SCRSM_IEN_MSK    (1u << 5 )
#define ZY_AC97_GCR_PCRDY_IEN_MSK    (1u << 8 )
#define ZY_AC97_GCR_SCRDY_IEN_MSK    (1u << 9 )
#define ZY_AC97_GCR_SDONE_IE_MSK     (1u << 18)
#define ZY_AC97_GCR_CDONE_IE_MSK     (1u << 19)
#define ZY_AC97_GCR_nDMAEN_MSK       (1u << 24)
#define ZY_AC97_GCR_CLKBPB_MSK       (1u << 31)
#define ZY_AC97_GCR_FRCRST_MSK       (1u << 30)
// Global Status Register bit mask constants

#define ZY_AC97_GSR_GSCI_MSK       (1u << 0 )
#define ZY_AC97_GSR_MIINT_MSK      (1u << 1 )
#define ZY_AC97_GSR_MOINT_MSK      (1u << 2 )
#define ZY_AC97_GSR_ACOFFD_MSK     (1u << 3 )
#define ZY_AC97_GSR_PIINT_MSK      (1u << 5 )
#define ZY_AC97_GSR_POINT_MSK      (1u << 6 )
#define ZY_AC97_GSR_MINT_MSK       (1u << 7 )
#define ZY_AC97_GSR_PCRDY_MSK      (1u << 8 )
#define ZY_AC97_GSR_SCRDY_MSK      (1u << 9 )
#define ZY_AC97_GSR_PCRSM_MSK      (1u << 10)
#define ZY_AC97_GSR_SCRSM_MSK      (1u << 11)
#define ZY_AC97_GSR_SLT12_BITS_MSK (7u << 12)
#define ZY_AC97_GSR_RCS_ERR_MSK    (1u << 15)
#define ZY_AC97_GSR_SDONE_MSK      (1u << 18)
#define ZY_AC97_GSR_CDONE_MSK      (1u << 19)


// Bit mask and values for CAIP bit in car register.
#define ZY_AC97_CAR_CAIP_MSK       (0x1<<0)
#define ZY_AC97_CAR_CAIP_LOCKED    (0x1<<0)
#define ZY_AC97_CAR_CAIP_CLEAR     (0<<0)

/* Constants for FIFO status reporting and control */

// One bit location is used to report FIFO error conditions and clear
//  interrupts on those conditions in the various non-global status registers.

// ZY_AC97_FIFOSTAT_FIFOE is used in:
                                                // posr
                                                // pisr
                                                // mcsr
                                                // mosr
                                                // misr

#define ZY_AC97_FIFOSTAT_FIFOE  (1u << 4)
#define ZY_AC97_FIFOSTAT_EOC	  (1u << 3)
#define ZY_AC97_FIFOSTAT_FSR	  (1u << 2)

// A different bit location is used to enable or disable interrupts based on
//  FIFO error conditions in the various non-global control registers.

// ZY_AC97_FIFOCTRL_FEIE is used in:
                                                // pocr
                                                // picr
                                                // mccr
                                                // mocr
                                                // micr

#define ZY_AC97_FIFOCTRL_FEIE  (1u << 3)
#define ZY_AC97_FIFOCTRL_FSRIE (1u << 1)

/*
*******************************************************************************
    AC'97 Codec Registers Location and Bit Definition
*******************************************************************************
*/

/* */

    // Includes symbolic values for certain proprietary register asssignments
    //   in AC'97 devices that might be used with ZY_AC97.

    // Valid for subset of R 2.1 specification.
    // Leading "e" in comment means it is an "expanded" register definition as
    //  found in one or more of the Appendices A-D of the R 2.1 specification.
    //  Appendix identifier will immediately follow the "e", such as "eA"
    // R/O indicates read-only
    // Registers not supported by the assumed controller will be commented out.

#define    ZY_AC97_CR_RESET_ID             0x00  // RESET CODEC TO DEFAULT, get ID info
#define    ZY_AC97_CR_MASTER_VOLUME        0x02  // LINE OUT VOLUME
#define    ZY_AC97_CR_HEADPHONE_VOLUME     0x04  //
#define    ZY_AC97_CR_MASTER_VOLUME_MONO   0x06  //
#define    ZY_AC97_CR_MASTER_TONE_R_L      0x08  //
#define    ZY_AC97_CR_PC_BEEP_VOLUME       0x0A  //
#define    ZY_AC97_CR_PHONE_VOLUME         0x0C  //
#define    ZY_AC97_CR_MIC_VOLUME           0x0E  //   micrOPHONE VOLUME/ AGC
#define    ZY_AC97_CR_LINE_IN_VOLUME       0x10  //   LINE IN VOLUME
#define    ZY_AC97_CR_CD_VOLUME            0x12  //
#define    ZY_AC97_CR_VIDEO_VOLUME         0x14  //
#define    ZY_AC97_CR_AUX_VOLUME           0x16  //
#define    ZY_AC97_CR_PCM_OUT_VOLUME       0x18  //
#define    ZY_AC97_CR_RECORD_SELECT        0x1A  //   SELECT LINE IN OR micrOPHONE
#define    ZY_AC97_CR_RECORD_GAIN          0x1C  //
#define    ZY_AC97_CR_RECORD_GAIN_MIC      0x1E  //
#define    ZY_AC97_CR_GENERAL_PURPOSE      0x20  //
#define    ZY_AC97_CR_CONTROL_3D           0x22  //
#define    ZY_AC97_CR_POWERDOWN_CTRL_STAT  0x26  //   POWER MANAGEMENT
#define    ZY_AC97_CR_E_AUDIO_ID           0x28  // eA Extended audio sprt info, R/O
#define    ZY_AC97_CR_E_AUDIO_CTRL_STAT    0x2A  // eA Extended audio stat + control

//
// Audio Sample Rate Control Registers, 0x2C - 0x34
//
                                           // eA PCM Front DAC rate control
#define    ZY_AC97_CR_E_ASR_PCM_FRNT_DAC_RT 0x2C  //  (output slots 3, 4, 6)
#define    ZY_AC97_CR_E_ASR_PCM_LR_ADC_RT   0x32  // eA PCM L+R ADC rate control (3+4)
#define    ZY_AC97_CR_E_ASR_MIC_ADC_RT      0x34  // eA PCM Mic ADC rate control (5)


#define    ZY_AC97_CR_E_MDM_GPIO_PIN_STAT     0x54
//
// 5Ah-7Ah: Vendor Reserved
//
//
// 7Ch-7Eh: Vendor ID registers.  Optional but standardized for Plug'n'Play
//
#define    ZY_AC97_CR_VENDOR_ID1         0x7C
#define    ZY_AC97_CR_VENDOR_ID2         0x7E

#define    ZY_AC97_CR_MAX               ZY_AC97_CR_VENDOR_ID2

#define    ZY_AC97_CR_END_OF_LIST       (ZY_AC97_CR_MAX +  2)



/* Other Constants */

// For accessing the Codec mixer registers, each increment of one 32-bit word
//  in processor space increments the addressed mixer register by two.
// This does not cause any ambiguities because only even mixer register
//  addresses are currently supported (AC '97 spec, R 2.2)
#define ZY_AC97_CODEC_REGS_PER_WORD   	2

/* Default timeout and holdtime settings */

// timeout in reading and writing codec registers through AC link
#define ZY_AC97_RW_TIMEOUT_DEF		200	//unit is us

// timeout in waiting for codec's ready signal during setup process
#define ZY_AC97_SETUP_TIMEOUT_DEF		500	//unit is us

// timeout in waiting for locking the link successfully
#define ZY_AC97_LOCK_TIMEOUT_DEF   		300	//unit is us

// timeout in shutting down the link
#define ZY_AC97_LINKOFF_TIMEOUT_DEF   	500	//unit is us

// holdtime for keeping nReset signal active(low) in AC link
#define ZY_AC97_COLD_HOLDTIME			100	//unit is us

/*
*******************************************************************************
  ZY AC97 data structure used in function interface
*******************************************************************************
*/

typedef struct
{
    unsigned long pocr;           // PCM Out Control Register
    unsigned long picr;           // PCM In Control Register
    unsigned long mccr;           // Mic In Control Register
    unsigned long gcr;            // Global Control Register
    unsigned long pcscr;          // PCM Surround Out Control
    unsigned long pcclcr;         // PCM Center/LFE Out Control
    unsigned long mocr;           // MODEM Out Control Register
    unsigned long micr;           // MODEM In Control Register
}zy_ac97_save_context_t;

#define AC97_SAVE_CONTEXT_SIZE (sizeof(zy_ac97_save_context_t))
/*
*******************************************************************************
  ZY ACODEC AC97 Functions
*******************************************************************************
*/

extern acodec_error_t zy_ac97_acodec_read(acodec_context_t *p_dev_context, unsigned short reg_addr,  unsigned short *p_reg_value);
extern acodec_error_t zy_ac97_acodec_write (acodec_context_t *p_dev_context, unsigned short reg_addr, unsigned short reg_value);
extern acodec_error_t zy_ac97_acodec_init (acodec_context_t *p_dev_context);
extern acodec_error_t zy_ac97_acodec_deinit (acodec_context_t *p_dev_context);
extern acodec_error_t zy_ac97_acodec_save_context (acodec_context_t *p_dev_context, zy_ac97_save_context_t *p_ac97_save_context);
extern acodec_error_t zy_ac97_acodec_restore_context (acodec_context_t *p_dev_context, zy_ac97_save_context_t *p_ac97_save_context);

#endif //__ZY_AC97ACODEC_H__
