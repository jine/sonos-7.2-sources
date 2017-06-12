ifeq ($(CONFIG_PXA3xx),y)
ifeq ($(CONFIG_CPU_PXA310),y)
   zreladdr-y	:= 0xb0008000
else
   zreladdr-y	:= 0x80008000
endif
else
   zreladdr-y	:= 0xa0008000
endif
