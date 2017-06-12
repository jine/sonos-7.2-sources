#
# Makefile for the kernel mmc device drivers.
#

#
# Bus driver
#
ifdef CONFIG_MMC_OPEN_SOURCE
obj-$(CONFIG_MMC)		+= mmc.o mmc_queue.o mmc_sysfs.o
else
obj-$(CONFIG_MMC)		+= mss_core.o
endif

#
# Media drivers
#
ifdef CONFIG_MMC_OPEN_SOURCE
obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o
else
obj-$(CONFIG_MMC_BLOCK)		+= mss_block.o
endif

#
# MMC protocol driver
# 
ifndef CONFIG_MMC_OPEN_SOURCE
obj-$(CONFIG_MMC_PROTOCOL)	+= mmc_protocol.o
obj-$(CONFIG_SD_PROTOCOL)	+= sd_protocol.o
obj-$(CONFIG_SDIO_PROTOCOL)	+= sdio_protocol.o
endif

#
# Host drivers
#
obj-$(CONFIG_MMC_ARMMMCI)	+= mmci.o
ifdef CONFIG_MMC_OPEN_SOURCE
obj-$(CONFIG_MMC_PXA)		+= pxamci.o
else
obj-$(CONFIG_MMC_PXA)		+= pxa3xx_controller.o
endif
obj-$(CONFIG_MMC_IMX)		+= imxmmc.o
obj-$(CONFIG_MMC_SDHCI)		+= sdhci.o
obj-$(CONFIG_MMC_WBSD)		+= wbsd.o
obj-$(CONFIG_MMC_AU1X)		+= au1xmmc.o
obj-$(CONFIG_MMC_OMAP)		+= omap.o
obj-$(CONFIG_MMC_AT91)		+= at91_mci.o
obj-$(CONFIG_MMC_TIFM_SD)	+= tifm_sd.o

ifeq ($(CONFIG_MMC_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
