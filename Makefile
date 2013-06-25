KERN_SRC=/home/flo/armLinux2/stable-kernel/KERNEL
KERN_COMPILER=/home/flo/armLinux2/stable-kernel/dl/gcc-linaro-arm-linux-gnueabihf-4.7-2013.04-20130415_linux/bin
 
obj-m := dali_drv.o
 
all:
	make -C $(KERN_SRC) ARCH=arm CROSS_COMPILE=$(KERN_COMPILER)/arm-linux-gnueabihf- M=`pwd` modules
 
clean:
	make -C $(KERN_SRC) ARCH=arm CROSS_COMPILE=$(KERN_COMPILER)/arm-linux-gnueabihf- M=`pwd` clean
