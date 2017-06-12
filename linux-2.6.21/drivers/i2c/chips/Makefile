#
# Makefile for miscellaneous I2C chip drivers.
#

obj-$(CONFIG_SENSORS_DS1337)	+= ds1337.o
obj-$(CONFIG_SENSORS_DS1374)	+= ds1374.o
obj-$(CONFIG_SENSORS_EEPROM)	+= eeprom.o
obj-$(CONFIG_SENSORS_MAX6875)	+= max6875.o
obj-$(CONFIG_SENSORS_M41T00)	+= m41t00.o
obj-$(CONFIG_SENSORS_PCA9539)	+= pca9539.o
obj-$(CONFIG_SENSORS_PCF8574)	+= pcf8574.o
obj-$(CONFIG_SENSORS_PCF8591)	+= pcf8591.o
obj-$(CONFIG_ISP1301_OMAP)	+= isp1301_omap.o
obj-$(CONFIG_TPS65010)		+= tps65010.o

obj-$(CONFIG_PXA3xx_ARAVA)	+= arava.o
obj-$(CONFIG_PXA3xx_MICCO)	+= micco.o
obj-$(CONFIG_PXA3xx_GPIOEX)	+= gpio-expander.o
obj-$(CONFIG_PXA3xx_MAX7320)	+= max7320.o
obj-$(CONFIG_PXA3xx_MAX7321)	+= max7321.o
obj-$(CONFIG_MEPTOUCH)		+= meptouch.o
obj-$(CONFIG_CYTOUCH)		+= cytouch.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

