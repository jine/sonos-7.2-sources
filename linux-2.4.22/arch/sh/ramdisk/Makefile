#
# Makefile for a ramdisk image
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

# O_FORMAT = $(shell $(OBJDUMP) -i | head -2 | grep elf32)
O_FORMAT = elf32-sh-linux
ramdisk.o: rdisk ld.script
	echo "O_FORMAT:  " $(O_FORMAT)
	$(LD) -T ld.script -b binary --oformat $(O_FORMAT) -o $@ rdisk

include $(TOPDIR)/Rules.make

