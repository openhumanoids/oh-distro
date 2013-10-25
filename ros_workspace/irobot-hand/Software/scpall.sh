#! /bin/bash

cd firmware
./scpfirmware.sh $1
cd ../handle/handle_controller
./scpall.sh $1
cd ../..

