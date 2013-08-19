#!/bin/bash

# 5 common (real) robot configurations:
echo "Generating: 1/7 stumps"
rosrun xacro xacro.py xacro/atlas_stumps.urdf.xacro > model_stumps.urdf
echo "Generating: 2/7 LS_RS"
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf.xacro > model_LS_RS.urdf
echo "Generating: 3/7 LI_RI"
rosrun xacro xacro.py xacro/atlas_irobot_hands.urdf.xacro > model_LI_RI.urdf


echo "Generating: 4/7 LI_RS"
rosrun xacro xacro.py xacro/atlas_irobot_left_sandia_right.urdf.xacro > model_LI_RS.urdf
echo "Generating: 5/7 LS_RI"
rosrun xacro xacro.py xacro/atlas_sandia_left_irobot_right.urdf.xacro > model_LS_RI.urdf

# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "Generating: 6/7 sim"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "Generating: 7/7 gazebo"
./renameSimGazeboURDF.sh
