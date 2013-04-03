#
# Makefile for the s3c framebuffer driver
#

ifeq ($(CONFIG_FB_S3C),y)
obj-y				+= s3cfb.o
obj-$(CONFIG_ARCH_S5PV210)	+= s3cfb_fimd6x.o

obj-$(CONFIG_FB_S3C_LTN101NT02)	+= s3cfb_ltn101nt02.o
obj-$(CONFIG_FB_BYD_8688)	    += s3cfb_byd8688.o

obj-$(CONFIG_FB_S3C_TL2796)     += s3cfb_tl2796.o
obj-$(CONFIG_FB_S3C_HT101HD1)	+= s3cfb_ht101hd1.o
endif

ifeq ($(CONFIG_FB_S3C_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
