#!/bin/bash

# 5 common (real) robot configurations:
echo "Generating: 1/9 stumps"
rosrun xacro xacro.py xacro/atlas_stumps.urdf.xacro > model_stumps.urdf
echo "Generating: 2/9 Left Sandia Right Stump"
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf.xacro > model_LS_RN.urdf
echo "Generating: 3/9 Left Stump  Right Sandia"
rosrun xacro xacro.py xacro/atlas_irobot_hands.urdf.xacro > model_LN_RS.urdf

echo "Generating: 4/9 Left Sandia Right Sandia"
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf.xacro > model_LS_RS.urdf
echo "Generating: 5/9 Left iRobot Right iRobot"
rosrun xacro xacro.py xacro/atlas_irobot_hands.urdf.xacro > model_LI_RI.urdf


echo "Generating: 6/9 Left iRobot Right Sandia"
rosrun xacro xacro.py xacro/atlas_irobot_left_sandia_right.urdf.xacro > model_LI_RS.urdf
echo "Generating: 7/9 Left Sandia Right iRobot"
rosrun xacro xacro.py xacro/atlas_sandia_left_irobot_right.urdf.xacro > model_LS_RI.urdf

# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "Generating: 8/9 sim"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "Generating: 9/9 gazebo"
./renameSimGazeboURDF.sh
