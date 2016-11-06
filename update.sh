#!/bin/bash

# Reference https://www.raspberrypi.org/documentation/linux/kernel/building.md

echo "start"

start_time=`date +%Y%m%d%H%M%S`

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

end_time=`date +%Y%m%d%H%M%S`


echo $start_time 
echo $end_time


cd ..
echo $start_time > update_time
echo $end_time >> update_time
echo `expr $time1 - $time` >> update_time

sudo reboot

