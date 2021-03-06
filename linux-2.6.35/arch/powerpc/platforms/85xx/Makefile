#
# Makefile for the PowerPC 85xx linux kernel.
#
obj-$(CONFIG_SMP)         += smp.o
obj-$(CONFIG_HOTPLUG_CPU) += bootpage.o
obj-$(CONFIG_SUSPEND)     += suspend-asm.o
obj-$(CONFIG_MPC85xx_CPUFREQ) += cpufreq.o

obj-$(CONFIG_MPC8540_ADS) += mpc85xx_ads.o
obj-$(CONFIG_MPC8560_ADS) += mpc85xx_ads.o
obj-$(CONFIG_MPC85xx_CDS) += mpc85xx_cds.o
obj-$(CONFIG_MPC8536_DS)  += mpc8536_ds.o
obj-$(CONFIG_MPC85xx_DS)  += mpc85xx_ds.o
obj-$(CONFIG_MPC85xx_MDS) += mpc85xx_mds.o
obj-$(CONFIG_MPC85xx_RDB) += mpc85xx_rdb.o
obj-$(CONFIG_P1010_RDB)    += p1010rdb.o
obj-$(CONFIG_P1022_DS)    += p1022_ds.o
obj-$(CONFIG_P4080_DS)    += p4080_ds.o corenet_ds.o
obj-$(CONFIG_STX_GP3)	  += stx_gp3.o
obj-$(CONFIG_TQM85xx)	  += tqm85xx.o
obj-$(CONFIG_SBC8560)     += sbc8560.o
obj-$(CONFIG_SBC8548)     += sbc8548.o
obj-$(CONFIG_SOCRATES)    += socrates.o socrates_fpga_pic.o
obj-$(CONFIG_KSI8560)	  += ksi8560.o
obj-$(CONFIG_XES_MPC85xx) += xes_mpc85xx.o
obj-$(CONFIG_FSL_PMC)     += uli8259_suspend.o
obj-$(CONFIG_SONOS_LIMELIGHT) += dtb-wrapper.o

# Force dependency (incbin is bad)
ifeq ($(CONFIG_SONOS_LIMELIGHT),y)
$(obj)/dtb-wrapper.o : arch/powerpc/boot/limelight-p1.dtb
endif     



#MRG: For diags, I reserve the last 2MB of physical memory for use by diagapp.
ifeq ($(DIAGS),1)

ifeq ($(CONFIG_SONOS_LIMELIGHT),y)
arch/powerpc/boot/limelight-p1.dtb : arch/powerpc/boot/dts/limelight_diag.dts
	scripts/dtc/dtc -I dts -O dtb -R 8 -S 20000 arch/powerpc/boot/dts/limelight_diag.dts > arch/powerpc/boot/limelight-p1.dtb
endif     

else

ifeq ($(CONFIG_SONOS_LIMELIGHT),y)
arch/powerpc/boot/limelight-p1.dtb : arch/powerpc/boot/dts/limelight-p1.dts
	scripts/dtc/dtc -I dts -O dtb -R 8 -S 20000 arch/powerpc/boot/dts/limelight-p1.dts > arch/powerpc/boot/limelight-p1.dtb
endif

endif
