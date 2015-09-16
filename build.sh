#!/bin/bash
# Compiles the ipab-distro from the ground up

# Source environment
. ~/ipab-distro/drc/software/config/drc_environment.sh
. ~/ipab-distro/ipab-ros-workspace/devel/setup.bash

# DRC
## externals
cd drc/software/externals
make -j6 $1 # $1 so we can overwrite -j6 by -j1 in case parallel build is broken
# might need to:
# cd opencv/pod-build
# ccmake .
# WITH_CUDA = OFF, then [c] and [e]

## drc
cd ..
make -j6 $1

# ipab-ros-workspace
cd ../../ipab-ros-workspace
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -j6 $1