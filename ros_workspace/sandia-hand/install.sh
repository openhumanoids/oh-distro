#!/bin/bash

cd `dirname $0`

rm -rf build
mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=$PWD  ../
make -j6 install
