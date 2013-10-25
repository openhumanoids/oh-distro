#! /bin/bash

echo "Building Palm Bootloader"
make clean
make conf/ARMH_palm.conf.mk
if [ $? -eq 0 ] # if no error
then
    cp xboot-boot.hex ../palm/
    mv xboot-boot.hex ../palm/bootloader-palm.hex
else
    echo "ERROR, stopping"
    exit 1
fi

echo "Building Tactile Bootloader"
make clean
make conf/ARMH_tactile.conf.mk
if [ $? -eq 0 ] # if no error
then
    cp xboot-boot.hex ../tactile/
    mv xboot-boot.hex ../tactile/bootloader-tactile.hex
else
    echo "ERROR, stopping"
    exit 1
fi

echo "Building Motor Bootloader"
make clean
make conf/ARMH_motor.conf.mk
if [ $? -eq 0 ] # if no error
then
    cp xboot-boot.hex ../motor/
    mv xboot-boot.hex ../motor/bootloader-motor.hex
else
    echo "ERROR, stopping"
    exit 1
fi

echo "Building Proximal Bootloader"
make clean
make conf/ARMH_proximal.conf.mk
if [ $? -eq 0 ] # if no error
then
    cp xboot-boot.hex ../proximal/
    mv xboot-boot.hex ../proximal/bootloader-proximal.hex
else
    echo "ERROR, stopping"
    exit 1
fi

echo "Building Distal Bootloader"
make clean
make conf/ARMH_distal.conf.mk
if [ $? -eq 0 ] # if no error
then
    cp xboot-boot.hex ../distal/
    mv xboot-boot.hex ../distal/bootloader-distal.hex
else
    echo "ERROR, stopping"
    exit 1
fi
