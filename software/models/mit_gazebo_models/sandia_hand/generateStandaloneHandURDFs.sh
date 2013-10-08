#!/bin/bash

echo "Generating: 1/2 Left Sandia Standalone"
rosrun xacro xacro.py xacro/sandia_left_hand_standalone.urdf.xacro > model_LS_standalone.urdf
echo "Generating: 2/2 Right Sandia Standalone"
rosrun xacro xacro.py xacro/sandia_right_hand_standalone.urdf.xacro > model_RS_standalone.urdf

echo ''
echo "NB: To complete the standalone urdf:"
echo "manually insert the [l/r]_hand link into the top of the urdf e.g."
echo '<robot name="atlas" xmlns:xacro="http://www.ros.org/wiki/xacro">'
echo '  <link name="r_hand"/>'
echo '  <link name="right_palm">'
echo ''
