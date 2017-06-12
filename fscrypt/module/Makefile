MODULE := 1
UPTO_ROOT := ../../..
include $(UPTO_ROOT)/mtools/master.mk
PWD := $(shell pwd)
KM_NAME := virtual_block
KM_CFILES = virtual_block.c
VBTARGET := $(BLDDIR)/$(KM_NAME).$(OBJECT_SUFFIX)
VB_PATH    := $(shell pwd)

KERN_BLD := $(KERNEL_PATH)

TARGETS += ${VBTARGET}

$(BLDDIR)/%.c: %.c
	@$(MKDIR) -p $(BLDDIR)
	cp -f $< $@
	echo ${PLATFORMSPATH}

$(BLDDIR)/Makefile:
	@$(MKDIR) -p $(BLDDIR)
	@echo "obj-m += $(KM_NAME).o" > $@

${VBTARGET}: prep $(KM_CFILES:%.c=$(BLDDIR)/%.c) $(BLDDIR)/Makefile
	$(MAKE) -C $(KERN_BLD) O=${PLATFORMSPATH}/$(ARCH)/build \
	M=$(shell pwd)/$(BLDDIR) \
	ARCH=$(ARCH_CPU) CROSS_COMPILE=$(CROSSPATH)$(CROSS) \
	EXTRA_CFLAGS="-nostdlib -I${VB_PATH}" \
	modules

prep:
	$(RM) -f $(BLDDIR)/*.o
	$(RM) -f $(BLDDIR)/*.mod.*

include $(BOTTOM_MK)
