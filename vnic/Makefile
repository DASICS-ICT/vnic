NAME = vnic
obj-m += $(NAME).o
TARGET := $(NAME).ko
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
WORK_DIR := /home/lgf/vnic

KBUILD_EXTRA_SYMBOLS := $(WORK_DIR)/xdma/xdma/Module.symvers

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
