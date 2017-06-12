#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1995, 1998, 2001, 2002 by Ralf Baechle
# Copyright (C) 2004  Maciej W. Rozycki
#

#
# Some DECstations need all possible sections of an ECOFF executable
#
ifdef CONFIG_MACH_DECSTATION
  E2EFLAGS = -a
else
  E2EFLAGS =
endif

#
# Drop some uninteresting sections in the kernel.
# This is only relevant for ELF kernels but doesn't hurt a.out
#
drop-sections	= .reginfo .mdebug .comment .note .pdr .options .MIPS.options
strip-flags	= $(addprefix --remove-section=,$(drop-sections))

VMLINUX = vmlinux

all: vmlinux.ecoff vmlinux.srec

vmlinux.ecoff: $(obj)/elf2ecoff $(VMLINUX)
	$(obj)/elf2ecoff $(VMLINUX) $(obj)/vmlinux.ecoff $(E2EFLAGS)

$(obj)/elf2ecoff: $(obj)/elf2ecoff.c
	$(HOSTCC) -o $@ $^

ENTRY=`readelf -a ${VMLINUX}|grep "Entry"|head -1|cut -d":" -f 2`
LDADDR=`readelf -a ${VMLINUX}|grep "\[ 1\]"|head -1|cut -d" " -f 26`

# SONOS NOTE:
#	u-boot requires that LZMA files have their uncompressed size in
#	the file header.  However most modern LZMA utilities leave this field
#	undefined and use an EOF marker instead.  To LZMA compress a kernel
#	you must specify which binary to use via the LZMA_TOOL environment
#	variable.
#	The fillmore u-boot explicitly states that LZMA 4.57 is the correct version.

vmlinux.bin: $(VMLINUX)
	$(OBJCOPY) -O binary $(strip-flags) $(VMLINUX) $(obj)/vmlinux.bin
	$(LZMA_TOOL) e $(obj)/vmlinux.bin $(obj)/vmlinux.bin.lzma 
	./mkimage \
		-A mips -O linux -T kernel -C lzma \
		-a 0x${LDADDR} -e ${ENTRY} -n "Linux Kernel Image" \
		-d $(obj)/vmlinux.bin.lzma $(obj)/vmlinux.bin.lzma.uImage
	cp $(obj)/vmlinux.bin.lzma.uImage vmlinux.uImage

vmlinux.srec: $(VMLINUX)
	$(OBJCOPY) -S -O srec $(strip-flags) $(VMLINUX) $(obj)/vmlinux.srec

clean-files += elf2ecoff \
	       vmlinux.bin \
	       vmlinux.ecoff \
	       vmlinux.srec
