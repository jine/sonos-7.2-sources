#ifndef _ARM_BYTEORDER_H
#define _ARM_BYTEORDER_H

#include <sonos/asm-arm-types.h>

#if !defined(__STRICT_ANSI__) || defined(__KERNEL__)
#  define __BYTEORDER_HAS_U64__
#  define __SWAB_64_THRU_32__
#endif

#include <linux/byteorder/little_endian.h>

#endif

