#!/bin/bash



DIRECTORY=`lsb_release -cs`

echo "Starting"
sudo apt-get install libusb-1.0-0-dev -y

cd /tmp/ros

make clean
make debian
mkdir -p $DIRECTORY

cp *.deb $DIRECTORY
make clean


