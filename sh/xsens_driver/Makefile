obj-m+=xsens_mt.o
ifndef KDIR
KDIR:=/lib/modules/$(shell uname -r)/build
endif
PWD:=$(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules