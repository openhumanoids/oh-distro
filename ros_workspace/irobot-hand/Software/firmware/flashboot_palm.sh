#! /bin/bash

sudo avrdude -px128a1 -P usb -c avrispmkII -e

cd palm
../bootloader/prgx128
../bootloader/prgx128
cd ..

