/*
 ******************************************************************************
 *
 * Copyright (c) 2013, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * Include File Name: sect_upgrade_header.h
 *
 * Include File Description:
 *  In targets that boot from nor-flash a header is placed at the beginning
 *  of the kernel partition that u-boot uses to determine whether to boot
 *  section 0 or 1.
 *  On targets that boot from nand-flash this information is in oob space.
 *
 ******************************************************************************
 */

#ifndef SECT_UPGRADE_HEADER_H
#define SECT_UPGRADE_HEADER_H

typedef struct _SECT_UPGRADE_HEADER
{
   unsigned int magic;
   unsigned int version;
   unsigned int kernelOffset;
   unsigned int kernelChecksum;
   unsigned int kernelLength;
   unsigned int rootfsOffset;
   unsigned int rootfsChecksum;
   unsigned int rootfsLength;
   unsigned int bootgen;
   unsigned int fpgaChecksum;
   unsigned int fpgaLength;
   unsigned int m4Checksum;
   unsigned int m4Length;
   unsigned int rootfsFormat;

#define SECT_UPGRADE_HEADER_PADS 2  /* make the overall size a power of 2 */
   unsigned int pad[SECT_UPGRADE_HEADER_PADS];
} SECT_UPGRADE_HEADER;


#define SECT_UPGRADE_HEADER_MAGIC           0x536F7821

#define SECT_UPGRADE_HEADER_VER_INITIAL     0
#define SECT_UPGRADE_HEADER_VER_ROOTFS_FMT  1
#define SECT_UPGRADE_HEADER_VERSION         SECT_UPGRADE_HEADER_VER_ROOTFS_FMT

// NB: The only code that should ever even consider using SECT_UPGRADE_KERNEL_HEADER_SIZE 
// or any other hardcoded size for this structure is the code in upgrade app
// that initializes and writes out this structure. It is expected to grow over
// time and be self-describing.
#define SECT_UPGRADE_KERNEL_HEADER_SIZE     sizeof(SECT_UPGRADE_HEADER)
#define SECT_UPGRADE_ROOTFS_HEADER_SIZE     0

#define SECT_UPGRADE_ROOTFS_FORMAT_PLAINTEXT    0
#define SECT_UPGRADE_ROOTFS_FORMAT_FIXED_KEY	1
#define SECT_UPGRADE_ROOTFS_FORMAT_RED_KEY	2
#define SECT_UPGRADE_ROOTFS_FORMAT_BLACK_KEY	3


#endif	// SECT_UPGRADE_HEADER_H
