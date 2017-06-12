# Copyright (c) 2015, Sonos, Inc.
#
# SPDX-License-Identifier: GPL-2.0

SMALL_OUTPUT := 1

UPTO_ROOT := ../..
include $(UPTO_ROOT)/mtools/master.mk

SUBDIRS = module
VPATH := libdm:libdm/datastruct:libdm/mm:cryptsetup/lib:cryptsetup/lib/luks1 \
	libdm/ioctl

INCLUDES += -DHAVE_STRERROR -D_GNU_SOURCE
INCLUDES += -DDM_DEVICE_UID=0 -DDM_DEVICE_GID=6 -DDM_DEVICE_MODE=0660 -DDM_IOCTLS
INCLUDES += -Imodule -Iinclude -Icryptsetup -Icryptsetup/lib -Icryptsetup/lib/luks1 \
            -Ilibdm/include -Ilibdm -Ilibdm/ioctl \
            $(KERNEL_INCLUDES)

TARGETS += prep
PROGRAM_NAME      := encryptfs

FSCRYPT_CFILES    := libdm/libdm-common.c \
	libdm/ioctl/libdm-iface.c \
	libdm/libdm-file.c \
	libdm/libdm-string.c \
	libdm/datastruct/list.c \
	libdm/datastruct/bitset.c \
	libdm/mm/pool.c \
	libdm/mm/dbg_malloc.c \
	encryptfs.c \
	sonos_luks.c \
	sonos_update.c \
	syslog_client.c \
	cryptsetup/lib/libdevmapper.c \
	cryptsetup/lib/setup.c  \
	cryptsetup/lib/utils.c \
	cryptsetup/lib/utils_crypt.c \
	cryptsetup/lib/utils_device.c \
	cryptsetup/lib/utils_devpath.c \
	cryptsetup/lib/utils_wipe.c \
	cryptsetup/lib/volumekey.c \
	cryptsetup/lib/luks1/keymanage.c \
	cryptsetup/lib/luks1/af.c

# Handle the fact the FSCRYPT_CFILES includes source files from multiple
# directories:
PROGRAM_OFILES=$(patsubst %.c,$(BLDDIR)/%.o,${notdir $(FSCRYPT_CFILES)})

include $(PROGRAM_MK)

include $(BOTTOM_MK)

syslog_client.c: $(UPTO_ROOT)/wifi/syslog_client.c
	cp -f $^ $@

syslog_client.h: $(UPTO_ROOT)/wifi/syslog_client.h
	cp -f $^ $@

prep: syslog_client.c syslog_client.h
