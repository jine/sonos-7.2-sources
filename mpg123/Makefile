# Copyright (c) 2015-2016, Sonos, Inc.  All rights reserved.

UPTO_ROOT := ../../..
include $(UPTO_ROOT)/mtools/master.mk

MODULE_NAME          := mpg123
LIBRARY_NAME         := lib$(MODULE_NAME)

VERSION_SUFFIX       := 1.22.4
MODULE_VER           := $(MODULE_NAME)-$(VERSION_SUFFIX)
LIBMPG123_VER        := $(LIBRARY_NAME)-$(VERSION_SUFFIX)
PATCH_DIR            := patches/$(VERSION_SUFFIX)

SANDBOX              := $(shell cd ../../../; pwd)

ifneq ($(ARCH), i386)
	LIB_NO_STATIC := 1
	MPG123_SHARED_OPTION := --enable-shared --disable-static
	ARCHIVE_SUFFIX       := so
else
	MPG123_SHARED_OPTION := --disable-shared
endif

ifdef SONOS_ARCH_ATTR_USES_NEON
	MPG123_CPU_TARGET := neon
else ifdef SONOS_ARCH_SH4
	MPG123_CPU_TARGET := generic_nofpu
	CROSS_CFLAGS  := $(CROSS_CFLAGS) -m4-single-only
	CROSS_LDFLAGS := $(CROSS_LDFLAGS) -m4-single-only
else
	MPG123_CPU_TARGET := generic_fpu
endif

ARLIBRARY_NAME  := $(LIBRARY_NAME).$(ARCHIVE_SUFFIX)
LIBRARY_ARCHIVE := $(BLDDIR)/src/libmpg123/.libs/$(ARLIBRARY_NAME)

TARGETS +=  $(LIBRARY_ARCHIVE)

$(LIBRARY_ARCHIVE): Makefile $(MODULE_VER).tar.bz2 $(PATCH_DIR)/*
	rm -rf $(BLDDIR)
	mkdir -p $(BLDDIR)
	tar xfj $(MODULE_VER).tar.bz2 -C $(BLDDIR)
	mv $(BLDDIR)/$(MODULE_VER)/* $(BLDDIR)
	rmdir $(BLDDIR)/$(MODULE_VER)
	cd $(BLDDIR) && \
	for p in `grep -v '^#' ../patches/$(VERSION_SUFFIX)/manifest`; do echo applying "$$p"; patch -b -p1 < "../patches/$(VERSION_SUFFIX)/$$p"; done
	cd $(BLDDIR) && \
	CC="$(CC)" CFLAGS="$(CROSS_CFLAGS) -DSONOS_MPG123_OPTIMIZED" LDFLAGS="$(CROSS_LDFLAGS)" \
		./configure \
		--host=$(HOST) \
		--with-cpu=$(MPG123_CPU_TARGET) \
		--disable-network --disable-ipv6 --disable-string \
		--disable-layer1 --disable-layer2 --disable-messages \
		--disable-feature_report --disable-largefile \
		--disable-gapless --disable-fifo --disable-buffer \
		--disable-ntom --disable-downsample --disable-icy \
		--disable-id3v2 --disable-lfs-alias --disable-feature-report \
		--with-seektable=0 \
		--enable-16bit --disable-8bit --disable-32bit --disable-real \
		$(MPG123_SHARED_OPTION)
	$(MAKE) -C $(BLDDIR)/src/libmpg123

include $(BOTTOM_MK)
