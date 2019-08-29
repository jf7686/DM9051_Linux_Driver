##================================================================
##     Davicom Semiconductor Inc.       For DM9620 V0.00
##   --------------------------------------------------------
## Description:
##              Compile driver dm9051.c to dm9051.ko
##
## 05/02/2014   by Stone Shyr
##================================================================
# Comment/uncomment the following line to disable/enable debugging
# DEBUG = y

# Add your debugging flag (or not) to CFLAGS
#ifeq ($(DEBUG),y)
#  DEBFLAGS = -O -g # "-O" is needed to expand inlines
#else
#  DEBFLAGS = -O2
#endif

#CFLAGS += $(DEBFLAGS) -I$(LDDINCDIR)

#ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m   := dm9051.o

#else
MODULE_INSTALDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/net/usb
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

#endif



clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

install:
	#modprobe -r dm9051
	install -c -m 0644 dm9051.ko $(MODULE_INSTALDIR)
	depmod -a -e

uninstall:
	rm -rf $(MODULE_INSTALDIR)/dm9051.ko

#depend .depend dep:
#       $(CC) $(CFLAGS) -M *.c > .depend


#ifeq (.depend,$(wildcard .depend))
#include .depend
#endif
