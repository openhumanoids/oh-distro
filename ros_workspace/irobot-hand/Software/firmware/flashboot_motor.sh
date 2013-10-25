#! /bin/bash


sudo avrdude -px32a4 -P usb -c avrispmkII -e

cd motor
../bootloader/prgx32
../bootloader/prgx32
cd ..

