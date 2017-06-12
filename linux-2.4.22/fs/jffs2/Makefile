#
# Makefile for the linux Journalling Flash FileSystem (JFFS) routines.
#
# $Id: Makefile.common,v 1.4 2004/05/25 11:31:51 havasi Exp $
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now in the main makefile...


obj-$(CONFIG_JFFS2_FS) := jffs2.o

JRUBIN_OBJS-$(CONFIG_JFFS2_RUBIN) := compr_rubin.o
JRTIME_OBJS-$(CONFIG_JFFS2_RTIME) := compr_rtime.o
JZLIB_OBJS-$(CONFIG_JFFS2_ZLIB)  := compr_zlib.o
JLZO_OBJS-$(CONFIG_JFFS2_LZO)  := compr_lzo.o
JLZARI_OBJS-$(CONFIG_JFFS2_LZARI)  := compr_lzari.o
JPROC_OBJS-$(CONFIG_JFFS2_PROC)  := proc.o
JFFS2_OBJS	:= compr.o dir.o file.o ioctl.o nodelist.o malloc.o \
	read.o nodemgmt.o readinode.o write.o scan.o gc.o \
	symlink.o build.o erase.o background.o fs.o writev.o

BELOW25		:= $(shell echo $(PATCHLEVEL) | sed s/[1234]/y/)

ifeq ($(BELOW25),y)
LINUX_OBJS	:= super-v24.o crc32.o rbtree.o
else
LINUX_OBJS	:= super.o
endif

NAND_OBJS-$(CONFIG_JFFS2_FS_NAND)	:= wbuf.o

jffs2-objs := $(COMPR_OBJS) $(JFFS2_OBJS) $(VERS_OBJS) $(NAND_OBJS-y) \
	$(LINUX_OBJS) $(JRUBIN_OBJS-y) $(JRTIME_OBJS-y) $(JZLIB_OBJS-y) \
	$(JPROC_OBJS-y) $(JLZO_OBJS-y) $(JLZARI_OBJS-y)

# 2.4 build compatibility
ifeq ($(BELOW25),y)
obj-y := $(jffs2-objs)
O_TARGET := jffs2.o
include $(TOPDIR)/Rules.make
endif
