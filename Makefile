# Makefile for kernel module

KERNEL_VERSION:=$(shell uname -r)
KERNEL_PATH:=/lib/modules/$(KERNEL_VERSION)/build
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

obj-m = litepcie.o
litepcie-objs = main.o

all: litepcie.ko

litepcie.ko: main.c
	make -C $(KERNEL_PATH) M=$(ROOT_DIR) modules

clean:
	make -C $(KERNEL_PATH) M=$(ROOT_DIR) clean
	rm -f *~
	
.PHONY: clean
