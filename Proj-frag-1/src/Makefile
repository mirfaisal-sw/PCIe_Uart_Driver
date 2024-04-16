obj-m += mpci_driver.o

ARCH=arm64
CROSS_COMPILE=aarch64-linux-gnu-

all:
	make ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C /home/mir/My_Workspace/RaspberryPi_4_Kernel/linux M=`pwd` modules
 
clean:
	make ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C /home/mir/My_Workspace/RaspberryPi_4_Kernel/linux M=`pwd` clean
