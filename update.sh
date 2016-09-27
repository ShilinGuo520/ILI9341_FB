#!/bin/bash

echo "start"

#git clone --depth=1 https://github.com/raspberrypi/linux
#sudo apt-get install bc

cd linux
KERNEL=kernel7
make bcm2709_defconfig

#make clean

make -j8 zImage modules dtbs
sudo make modules_install
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm/boot/dts/overlays/README /boot/overlays/
sudo scripts/mkknlimg arch/arm/boot/zImage /boot/$KERNEL.img

echo "end"
sudo reboot

