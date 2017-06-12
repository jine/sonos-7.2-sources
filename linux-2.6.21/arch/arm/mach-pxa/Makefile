#
# Makefile for the linux kernel.
#

# Common support (must be linked before board specific support)
obj-y += clock.o generic.o irq.o dma.o time.o
obj-$(CONFIG_PXA25x) += pxa25x.o
obj-$(CONFIG_PXA27x) += pxa27x.o
obj-$(CONFIG_PXA3xx) += pxa3xx.o mfp.o pxa3xx_gpio.o

# Specific board support
obj-$(CONFIG_ARCH_LUBBOCK) += lubbock.o
obj-$(CONFIG_MACH_LOGICPD_PXA270) += lpd270.o
obj-$(CONFIG_MACH_MAINSTONE) += mainstone.o
obj-$(CONFIG_ARCH_PXA_IDP) += idp.o
obj-$(CONFIG_MACH_TRIZEPS4)	+= trizeps4.o
obj-$(CONFIG_PXA_SHARP_C7xx)	+= corgi.o corgi_ssp.o corgi_lcd.o sharpsl_pm.o corgi_pm.o
obj-$(CONFIG_PXA_SHARP_Cxx00)	+= spitz.o corgi_ssp.o corgi_lcd.o sharpsl_pm.o spitz_pm.o
obj-$(CONFIG_MACH_AKITA)	+= akita-ioexp.o
obj-$(CONFIG_MACH_POODLE)	+= poodle.o corgi_ssp.o
obj-$(CONFIG_MACH_TOSA)         += tosa.o
obj-$(CONFIG_MACH_ZYLONITE)	+= zylonite.o
obj-$(CONFIG_MACH_LITTLETON)	+= littleton.o
obj-$(CONFIG_MACH_WOODSTOCK)	+= woodstock.o

# Support for blinky lights
led-y := leds.o
led-$(CONFIG_ARCH_LUBBOCK) += leds-lubbock.o
led-$(CONFIG_MACH_MAINSTONE) += leds-mainstone.o
led-$(CONFIG_ARCH_PXA_IDP) += leds-idp.o
led-$(CONFIG_MACH_TRIZEPS4) += leds-trizeps4.o
led-$(CONFIG_MACH_ZYLONITE) += leds-zylonite.o
led-$(CONFIG_MACH_LITTLETON) += leds-littleton.o

obj-$(CONFIG_LEDS) += $(led-y)

# Misc features
obj-$(CONFIG_PXA_SSP) += ssp.o

obj-$(CONFIG_PXA3xx_PMIC)	+= pxa3xx_pmic.o

obj-$(CONFIG_IMM) += imm/

ifeq ($(CONFIG_PXA27x),y)
obj-$(CONFIG_PM) += standby.o
endif

obj-$(CONFIG_PXA3xx_PMIC)	+= pxa3xx_pmic.o

ifeq ($(CONFIG_PXA3xx),y)
  obj-y += clkmgr.o pmu.o pmu_gnu.o
  obj-$(CONFIG_PM)		+= pxa3xx_pm.o pxa3xx_suspend.o sleepwkr.o
  obj-$(CONFIG_DVFM)		+= cpu-freq-voltage-pxa3xx.o pm_dvfm.o
  obj-$(CONFIG_PRM)		+= prm.o
  obj-$(CONFIG_IPM)		+= ipmc.o ipm-profiler.o ipm-pmu.o
else
  obj-$(CONFIG_PM)		+= pm.o sleep.o
endif
