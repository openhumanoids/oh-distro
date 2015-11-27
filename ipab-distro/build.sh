#!/bin/bash
# Updates/compiles all folders sequentially

# Source environment
. ../software/config/drc_environment.sh
. ../catkin_ws/devel/setup.bash

# DRC
## externals
cd ../software/externals
make -j8 $1

## drc
cd ..
make -j8 $1

## ROS/ catkin workspace
cd ../catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -j8 $1