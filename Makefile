//MODULE_NAME = ch341-gpio
//obj-m += ch341-gpio.o

MODULE_NAME = ch341-bridge
obj-m += $(MODULE_NAME).o
$(MODULE_NAME)-objs += ch341-core.o
$(MODULE_NAME)-objs += ch341-gpio.o
$(MODULE_NAME)-objs += ch341-spi.o

# Path to the kernel source tree
KDIR ?= /lib/modules/$(shell uname -r)/build

PWD := $(shell pwd)

default: module

module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)

probe:
	# modprobe -r -f $(MODULE_NAME) || true
	rmmod $(MODULE_NAME).ko || true
	dmesg -c > /dev/null
	insmod $(MODULE_NAME).ko
	#modprobe $(MODULE_NAME)
	#dmesg -W


olddefconfig:
	$(MAKE) -C $(KDIR) olddefconfig

prepare:
	$(MAKE) -C $(KDIR) modules_prepare 

help:
	$(MAKE) -C $(KDIR) help
