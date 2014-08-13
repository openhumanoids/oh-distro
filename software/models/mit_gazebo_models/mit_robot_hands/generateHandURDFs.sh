#!/bin/bash
 
echo "Generate Robot Model Configurations:"

echo "sandia_hand_left"
rosrun xacro xacro.py xacro/sandia_hand_left.urdf.xacro > sandia_hand_left.urdf
rosrun xacro xacro.py xacro/sandia_hand_right.urdf.xacro > sandia_hand_right.urdf

echo "irobot hands"
rosrun xacro xacro.py xacro/irobot_hand_left.urdf.xacro > irobot_hand_left.urdf
rosrun xacro xacro.py xacro/irobot_hand_right.urdf.xacro > irobot_hand_right.urdf

echo "robotiq hands"
rosrun xacro xacro.py xacro/robotiq_hand_left.urdf.xacro > robotiq_hand_left.urdf
rosrun xacro xacro.py xacro/robotiq_hand_right.urdf.xacro > robotiq_hand_right.urdf

echo "pointer hands"
rosrun xacro xacro.py xacro/pointer_hand_left.urdf.xacro > pointer_hand_left.urdf
rosrun xacro xacro.py xacro/pointer_hand_right.urdf.xacro > pointer_hand_right.urdf

# Valkyrie hands are not generated from xacro
