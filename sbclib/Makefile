# Copyright (c) 2010-2015, Sonos, Inc.  All rights reserved.

# This disables a number of pedantic warnings that would otherwise appear
NONRINCONCODE = 1

UPTO_ROOT := ../../..
include $(UPTO_ROOT)/mtools/master.mk

SUBDIRS =

ifeq ($(ARCH), sh4)
OPTIMIZE  += -fomit-frame-pointer -m4-single-only
endif

INCLUDES += -I$(UPTO_ROOT)/opr-os/minimath

DEFINES += -DRINCON -DSONOS_SBC_OPTIMIZED

LIB_CFILES :=  sbc.c \
               sbc_primitives.c

ifneq ($(ARCH), i386)
    LIB_NO_STATIC := 1
endif

LIB_NAME :=  libsbc
LIB_VER_MAJOR := 1
LIB_VER_MINOR := 0

include $(LIB_MK)

include $(BOTTOM_MK)
