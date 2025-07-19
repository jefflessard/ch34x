MODULE_NAME = ch341-gpio
obj-m += ch341-gpio.o

# Path to the kernel source tree
KDIR ?= /lib/modules/$(shell uname -r)/build

PWD := $(shell pwd)

default: module

module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install INSTALL_MOD_PATH=$(INSTALL_MOD_PATH)
	depmod -a

probe:
	# modprobe -r -f $(MODULE_NAME) || true
	rmmod ch341-gpio.ko || true
	dmesg -c > /dev/null
	insmod ch341-gpio.ko
	#modprobe $(MODULE_NAME)
	#dmesg -W


olddefconfig:
	$(MAKE) -C $(KDIR) olddefconfig

prepare:
	$(MAKE) -C $(KDIR) modules_prepare 

help:
	$(MAKE) -C $(KDIR) help
