#
# Makefile for GM12U320-based devices driver
#

obj-m += gm12u320.o

all:
	make -C /lib/modules/`uname -r`/build M=`pwd` modules

modules_install:
	make -C /lib/modules/`uname -r`/build M=`pwd` modules_install

clean:
	make -C /lib/modules/`uname -r`/build M=`pwd` clean

