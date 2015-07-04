obj-m += axi-csi-rx.o ov5647.o

KSRC ?= "/lib/modules/$(shell uname -r)/build"

all:
	make -C ${KSRC} M=$(PWD) modules

clean:
	make -C ${KSRC} M=$(PWD) clean
