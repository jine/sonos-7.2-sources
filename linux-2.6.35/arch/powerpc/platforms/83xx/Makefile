#
# Makefile for the PowerPC 83xx linux kernel.
#
obj-y				:= misc.o usb.o
obj-$(CONFIG_SUSPEND)		+= suspend.o suspend-asm.o
obj-$(CONFIG_MCU_MPC8349EMITX)	+= mcu_mpc8349emitx.o
obj-$(CONFIG_MPC831x_RDB)	+= mpc831x_rdb.o
obj-$(CONFIG_MPC832x_RDB)	+= mpc832x_rdb.o
obj-$(CONFIG_MPC834x_MDS)	+= mpc834x_mds.o
obj-$(CONFIG_MPC834x_ITX)	+= mpc834x_itx.o
obj-$(CONFIG_MPC836x_MDS)	+= mpc836x_mds.o
obj-$(CONFIG_MPC836x_RDK)	+= mpc836x_rdk.o
obj-$(CONFIG_MPC832x_MDS)	+= mpc832x_mds.o
obj-$(CONFIG_MPC837x_MDS)	+= mpc837x_mds.o
obj-$(CONFIG_SBC834x)		+= sbc834x.o
obj-$(CONFIG_MPC837x_RDB)	+= mpc837x_rdb.o
obj-$(CONFIG_ASP834x)		+= asp834x.o
obj-$(CONFIG_KMETER1)		+= kmeter1.o
obj-$(CONFIG_SONOS)     	+= dtb-wrapper.o

# Force dependency (incbin is bad)
$(obj)/dtb-wrapper.o : arch/powerpc/boot/fenway.dtb

arch/powerpc/boot/fenway.dtb : arch/powerpc/boot/dts/fenway.dts
	scripts/dtc/dtc -I dts -O dtb -R 8 -S 20000 arch/powerpc/boot/dts/fenway.dts > arch/powerpc/boot/fenway.dtb

