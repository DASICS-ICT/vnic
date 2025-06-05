NAME = vnic
obj-m += $(NAME).o
TARGET := $(NAME).ko
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
XDMA_DIR := $(PWD)/xdma

KBUILD_EXTRA_SYMBOLS := $(XDMA_DIR)/xdma/Module.symvers

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
