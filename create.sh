#!/bin/bash

kernelDir=/home/nicolas/project/kernel
#kernelDir=/home/development/Desktop/Raspberry/kernel/kernel_openplotter
targetDir=/home/nicolas/project/naviDev

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- all KDIR=$kernelDir

if [ $? -eq 0 ]
then
echo "Copy kernel modules to the target rootfs"
cp *.ko $targetDir
else
echo "Error occured during make"
exit 1
fi

echo "************ SUCCESSFULL ************"
exit 0
