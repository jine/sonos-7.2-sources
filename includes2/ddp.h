/*
 ******************************************************************************
 * Copyright (c) 2013, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * ddp.h
 *
 * Module Description:
 * 
 * Diagnotic Data Page structure and definitions
 *
 ******************************************************************************
 */


#define DDP_MAGIC   0xBEDAC0ED


/*
 * MRG
 * Only change the version if a change is needed that breaks
 * backwards compatibility.
 * Ideally the version should NEVER change.
 */
#define DDP_VERSION 2


typedef struct diagnostic_data_page
{

	unsigned long   ddpMagic;
	unsigned long   ddpVersion;

    unsigned long   dutAddr;
#define DDP_DUT_ADDR_STATIC     0x00000000  /* Use 169.254.1.1 */
#define DDP_DUT_ADDR_STATIC_N   0x00000001  /* Use 169.254.1.<dutNum> */
#define DDP_DUT_ADDR_DHCP       0x00000002  /* Use DHCP */
    unsigned long   dutNum; /* Used when DDP_DUT_ADDR_STATIC_N */

#define DDP_MAX_SERIALNUMS      16
#define DDP_MAX_SERIALNUM_LEN   32
    struct
    {
        char          serialnum[DDP_MAX_SERIALNUM_LEN]; /* ASCIIZ */
    } serialnums[DDP_MAX_SERIALNUMS];


#define DDP_MAX_RESULTS        64
#define DDP_MAX_RESULT_LEN     64
    struct
    {
        char          result[DDP_MAX_RESULT_LEN]; /* ASCIIZ */
    } results[DDP_MAX_RESULTS];

    /* DDP_VERSION 2 added CPU reset cause field  */
    unsigned long   reset_cause; /* uboot reset cause */

    /*
     * Add additional fields here.
     */
} DDP_ST, *DDP_PST;


/* API */
int ddpSetDUTAddressType(int type, int numDUT);
int ddpAddSerialNum(int index, char *serialnum);
int ddpAddStationResult(int index, char *result);
int SetResetCause(int cause);
