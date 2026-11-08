obj-m += imx06a.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all: modules imx06a.dtbo

modules:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean
	rm -f imx06a.dtbo

%.dtbo: %-overlay.dts
	dtc -@ -Wno-unit_address_vs_reg -I dts -O dtb -o $@ $<

.PHONY: all modules clean
