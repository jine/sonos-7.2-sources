#
# linux/drivers/nand/Makefile
#
# $Id: Makefile.common,v 1.8 2004/06/17 02:43:24 dbrown Exp $

ifeq ($(PATCHLEVEL),4)
O_TARGET			:= nandlink.o
export-objs			:= nand_base.o nand_bbt.o nand_ecc.o nand_ids.o
list-multi			:= nand.o
endif

obj-$(CONFIG_MTD_NAND)		+= nand_ecc.o nand.o
obj-$(CONFIG_MTD_NAND_SPIA)	+= spia.o
obj-$(CONFIG_MTD_NAND_TOTO)	+= toto.o
obj-$(CONFIG_MTD_NAND_AUTCPU12)	+= autcpu12.o
obj-$(CONFIG_MTD_NAND_EDB7312)  += edb7312.o
obj-$(CONFIG_MTD_NAND_TX4925NDFMC)  += tx4925ndfmc.o
obj-$(CONFIG_MTD_NAND_TX4938NDFMC)  += tx4938ndfmc.o
obj-$(CONFIG_MTD_NAND_AU1550)	  += au1550nd.o
obj-$(CONFIG_MTD_NAND_IDS)	+= nand_ids.o
obj-$(CONFIG_MTD_NAND_PPCHAMELEONEVB)   += ppchameleonevb.o
obj-$(CONFIG_MTD_NAND_DISKONCHIP) += diskonchip.o
obj-$(CONFIG_MTD_NAND_RINCON) += nand_rincon.o

nand-objs = nand_base.o nand_bbt.o

-include $(TOPDIR)/Rules.make

nand.o: $(nand-objs)
	$(LD) -r -o $@ $(nand-objs)
