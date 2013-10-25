#! /bin/bash

set -o errexit

cd palm
make clean
make
if [ $? -ne 0 ]
then
    exit 1
fi

cd ../tactile
make clean
make
if [ $? -ne 0 ]
then
    exit 1
fi

cd ../motor
make clean
make
if [ $? -ne 0 ]
then
    exit 1
fi

cd ../proximal
make clean
make
if [ $? -ne 0 ]
then
    exit 1
fi

cd ../distal
make clean
make
if [ $? -ne 0 ]
then
    exit 1
fi

cd ../bootloader
make clean
make
if [ $? -ne 0 ]
then
    exit 1
fi

cd ..
