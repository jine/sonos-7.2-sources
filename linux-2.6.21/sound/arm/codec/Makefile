obj-$(CONFIG_CODEC_AC97_ZY)	+= zy-ac97-codec.o
ifeq ($(CONFIG_BASE_BAND_AUDIO), y)
zy-ac97-codec-objs		+= ac97acodec.o ac97acodecplat.o acodec.o wm9713_bb.o
else
zy-ac97-codec-objs		+= ac97acodec.o ac97acodecplat.o acodec.o wm9713.o
endif

obj-$(CONFIG_CODEC_MICCO_LT)	+= lt-micco-codec.o
lt-micco-codec-objs	+= lt_micco.o lt_micco_acodec.o
