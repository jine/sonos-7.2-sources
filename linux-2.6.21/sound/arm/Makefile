#
# Makefile for ALSA
#

obj-$(CONFIG_SND_SA11XX_UDA1341) += snd-sa11xx-uda1341.o 
snd-sa11xx-uda1341-objs		:= sa11xx-uda1341.o

obj-$(CONFIG_SND_ARMAACI)	+= snd-aaci.o
snd-aaci-objs			:= aaci.o devdma.o

obj-$(CONFIG_SND_PXA2XX_PCM)	+= snd-pxa2xx-pcm.o
snd-pxa2xx-pcm-objs		:= pxa2xx-pcm.o

obj-$(CONFIG_SND_PXA2XX_AC97)	+= snd-pxa2xx-ac97.o
snd-pxa2xx-ac97-objs		:= pxa2xx-ac97.o

obj-$(CONFIG_SND_ZYLONITE_CORE)		+= zylonite_snd.o

ifeq ($(CONFIG_BASE_BAND_AUDIO), y)
zylonite_snd-objs			+= pxa3xx_audio_plat_bb.o pxa3xx_audio_card.o
else
zylonite_snd-objs			+= pxa3xx_audio_plat.o pxa3xx_audio_card.o
endif

obj-$(CONFIG_SND_ZYLONITE)		+= pxa3xx_audio_pcm.o  pxa3xx_audio_control.o \
					   pxa3xx_audio_hifi_pcm.o pxa3xx_audio_voice_pcm.o


obj-$(CONFIG_SND_LITTLETON_CORE)	+= littleton_snd.o
littleton_snd-objs			+= lt_audio.o lt_audio_card.o \
					   lt_audio_pcm.o pxa3xx_audio_control.o \
					   pxa3xx_audio_hifi_pcm.o pxa3xx_audio_voice_pcm.o

obj-$(CONFIG_SND_CODEC)			+= codec/

