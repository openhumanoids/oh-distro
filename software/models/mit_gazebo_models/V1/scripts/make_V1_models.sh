#!/bin/bash
V1_PACKAGE=V1
source $(rospack find $V1_PACKAGE)/scripts/xacro_to_urdf.bash

MODEL=V1			# Name of ROS package
XACROS=(hw sim gazebo wbc)	# List of xacros to build

make_xacros $MODEL ${XACROS[@]}	# XACRO to URDF conversion
