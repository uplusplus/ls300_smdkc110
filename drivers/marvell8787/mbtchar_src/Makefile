SHELL = bash

CONFIG_FILE := $(PWD)/.config
-include $(CONFIG_FILE)

EXTRA_CFLAGS += -I$(PWD)

export EXTRA_CFLAGS

# KERNELDIR point to the installed kernel directory.
# KERNELDIR can be set on the command line,
# make KERNELDIR=/usr/src/arm/<arch-bsp-path>
# Alternatively KERNELDIR can be set in the environment.
# Default value for KERNELDIR is set below.
KERNELDIR ?= ../../../

# CROSS_COMPILE specify the prefix used for all executables used
# during compilation. Only gcc and related bin-utils executables
# CROSS_COMPILE can be set on the command line 
# make CROSS_COMPILE=</usr/local/arm/4.1.1/bin/>arm-linux-
# Alternatively CROSS_COMPILE can be set in the environment.
# Default value for CROSS_COMPILE is set below.
CROSS_COMPILE ?= /usr/local/arm/arm-2009q3/bin/arm-none-linux-gnueabi-

# INSTALLDIR specify the path to install the kernel module after
# succesful compilation.
# INSTALLDIR can be set on the command line 
# make INSTALLDIR=/tftpboot/<rootfs>
# Alternatively INSTALLDIR can be set in the environment.
# Default value for INSTALL is set below.
INSTALLDIR ?= ~/project/bin

# ARCH specifies the architecture of the target processor, this kernel
# module will run.
# ARCH can be set on the command line
# make ARCH=<arm/i386>
# Alternatively ARCH can be set in the environment
# Default values of ARCH for specific platform are set below.
ARCH ?= arm

MBT_CHAR_OBJS := mbt_char.o hci_wrapper.o
obj-m := mbtchar.o
mbtchar-objs := $(MBT_CHAR_OBJS)

BINDIR = ../bin_mbtchar

PWD   := $(shell pwd)

default:
ifeq ($(CONFIG_PXA9XX), y)
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
else
ifeq ($(CONFIG_MOORESTOWN), y)
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
else
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
endif

build: default
	@if [ ! -d $(BINDIR) ]; then \
		mkdir $(BINDIR); \
	fi 
	@cp -f mbtchar.ko $(BINDIR)
	@cp -f README $(BINDIR)

clean:
	echo "pwd" $(PWD)
	-find . -name "*.o" -exec rm {} \;
	-find . -name "*.ko" -exec rm {} \;
	-find . -name ".*.cmd" -exec rm {} \;
	-find . -name "*.mod.c" -exec rm {} \;
	-find . -name "*.symvers" -exec rm {} \;
	-find . -name "modules.order" -exec rm {} \;
	-rm -rf .tmp_versions

PXA9XX PXA950 PXA920 X86 MOORESTOWN:
	@echo CONFIG_$@=y > $(CONFIG_FILE)

2.6.25:
	@echo "KMINORVER=25" >> $(CONFIG_FILE)

2.6.32:
	@echo "KMINORVER=32" >> $(CONFIG_FILE)

