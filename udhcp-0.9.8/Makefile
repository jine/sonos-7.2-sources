# Copyright (c) 2013, Rincon Networks, Inc.  All rights reserved.

UPTO_ROOT := ../../..
include $(UPTO_ROOT)/mtools/master.mk

SUBDIRS :=

# use the right Linux kernel headers
ifeq ($(ARCH),i386)
KERNEL_PATH  := /usr/src/linux
endif
ifneq ($(findstring $(ARCH),fenway limelight),)
XTRA_INC := -I$(KERNEL_PATH)/arch/powerpc/include
endif

INCLUDES += -I$(KERNEL_PATH)/include \
	    -I$(UPTO_ROOT)/cc/event_reporter \
	    $(XTRA_INC)

ifdef SONOS_ARCH_ATTR_SUPPORTS_EXTERNAL_EVENTS
LDFLAGS   += -L$(UPTO_ROOT)/cc/event_reporter/$(BLDDIR) -leventreporter
endif

# Uncomment this to output messages to syslog client
DEFINES += -DSYSLOG
# Uncomment this to output messages to stdout
# DEFINES += -DSTDOUTLOG
# Uncomment this to output messages to jffs
# DEFINES += -DJFFSFILELOG

VER := 0.9.8
DEFINES += -DVERSION='"$(VER)"'

PROGRAM_NAME        :=  udhcpc
PROGRAM_CFILES      :=  \
	options.c \
	socket.c \
	packet.c \
	pidfile.c \
	dhcpc.c \
	clientpacket.c \
	nlhelp.c \
	script.c \
	syslog_client.c \
	zeroconf.c

include $(PROGRAM_MK)

include $(BOTTOM_MK)

