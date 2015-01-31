#!/bin/bash

echo "Generating: 1/2 Left iRobot Standalone"
rosrun xacro xacro.py xacro/irobot_left_hand_standalone.urdf.xacro > model_LI_standalone.urdf
echo "Generating: 2/2 Right iRobot Standalone"
rosrun xacro xacro.py xacro/irobot_right_hand_standalone.urdf.xacro > model_RI_standalone.urdf

echo ''
echo ''
echo ''
echo '###################################################################'
echo "NB: To complete the standalone urdf:"
echo "manually insert the [l/r]_hand link into the top of the urdf e.g."
echo '<robot name="atlas" xmlns:xacro="http://www.ros.org/wiki/xacro">'
echo '  <link name="l_hand"/>'
echo '  <joint name="left_finger[0]/joint_flex" type="revolute">'
echo '###################################################################'
