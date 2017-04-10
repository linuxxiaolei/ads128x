obj-m := pt100.o
KERNEL_DIR :=/linux-3.6
PWD := $(shell pwd)
all:
	make ARCH=arm CROSS_COMPILE=arm-linux- -C /linux-3.6 M=$(PWD) modules
clean:
	rm *.o *.ko
