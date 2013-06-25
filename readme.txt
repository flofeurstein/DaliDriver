-make
cmd "make" in shell

-compile
/home/flo/armLinux2/stable-kernel/dl/gcc-linaro-arm-linux-gnueabihf-4.7-2013.04-20130415_linux/bin/arm-linux-gnueabihf-gcc dali_app.c -o dali_app

-insert module in system and make it accessible by changing group and permission (put this code into /etc/rc.local or execute it as su)
insmod /home/ubuntu/drivers/dali/dali_drv.ko
chgrp dialout /dev/dali_drv
chmod 660 /dev/dali_drv

