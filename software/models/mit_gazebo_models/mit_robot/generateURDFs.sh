#!/bin/bash

echo "Generate Robot Model Configurations:"
# 9 real robot configurations:
echo "1/11 LI RI"
#rosrun xacro xacro.py xacro/atlas_LI_RI.urdf.xacro > model_LI_RI.urdf
echo "2/11 LI RN"
#rosrun xacro xacro.py xacro/atlas_LI_RN.urdf.xacro > model_LI_RN.urdf
echo "3/11 LI RS"
#rosrun xacro xacro.py xacro/atlas_LI_RS.urdf.xacro > model_LI_RS.urdf

echo "4/11 LN RI"
#rosrun xacro xacro.py xacro/atlas_LN_RI.urdf.xacro > model_LN_RI.urdf
echo "5/11 LN RN"
#rosrun xacro xacro.py xacro/atlas_LN_RN.urdf.xacro > model_LN_RN.urdf
echo "6/11 LN RS"
#rosrun xacro xacro.py xacro/atlas_LN_RS.urdf.xacro > model_LN_RS.urdf

echo "7/11 LS RI"
#rosrun xacro xacro.py xacro/atlas_LS_RI.urdf.xacro > model_LS_RI.urdf
echo "8/11 LS RN"
#rosrun xacro xacro.py xacro/atlas_LS_RN.urdf.xacro > model_LS_RN.urdf
echo "9/11 LS RS"
#rosrun xacro xacro.py xacro/atlas_LS_RS.urdf.xacro > model_LS_RS.urdf

echo "10/11 LH RI"
rosrun xacro xacro.py xacro/atlas_LH_RI.urdf.xacro > model_LH_RI.urdf


# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "10/11 sim - MIT Facing"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "11/11 sim - Gazebo Facing"
./renameSimGazeboURDF.sh
