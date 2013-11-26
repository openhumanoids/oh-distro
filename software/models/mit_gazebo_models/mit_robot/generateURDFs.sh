#!/bin/bash

echo "Generate Robot Model Configurations:"
# 9 real robot configurations:
echo "1/15 LI RI"
#rosrun xacro xacro.py xacro/atlas_LI_RI.urdf.xacro > model_LI_RI.urdf
echo "2/15 LI RN"
#rosrun xacro xacro.py xacro/atlas_LI_RN.urdf.xacro > model_LI_RN.urdf
echo "3/15 LI RS"
#rosrun xacro xacro.py xacro/atlas_LI_RS.urdf.xacro > model_LI_RS.urdf

echo "4/15 LN RI"
#rosrun xacro xacro.py xacro/atlas_LN_RI.urdf.xacro > model_LN_RI.urdf
echo "5/15 LN RN"
#rosrun xacro xacro.py xacro/atlas_LN_RN.urdf.xacro > model_LN_RN.urdf
echo "6/15 LN RS"
#rosrun xacro xacro.py xacro/atlas_LN_RS.urdf.xacro > model_LN_RS.urdf
echo "7/15 LN RR"
rosrun xacro xacro.py xacro/atlas_LN_RR.urdf.xacro > model_LN_RR.urdf
echo "8/15 LR RN"
rosrun xacro xacro.py xacro/atlas_LR_RN.urdf.xacro > model_LR_RN.urdf

echo "9/15 LS RI"
#rosrun xacro xacro.py xacro/atlas_LS_RI.urdf.xacro > model_LS_RI.urdf
echo "10/15 LS RN"
#rosrun xacro xacro.py xacro/atlas_LS_RN.urdf.xacro > model_LS_RN.urdf
echo "11/15 LS RS"
#rosrun xacro xacro.py xacro/atlas_LS_RS.urdf.xacro > model_LS_RS.urdf

echo "12/15 LH RI"
rosrun xacro xacro.py xacro/atlas_LH_RI.urdf.xacro > model_LH_RI.urdf

echo "13/15 LH RR"
rosrun xacro xacro.py xacro/atlas_LH_RR.urdf.xacro > model_LH_RR.urdf

# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "14/15 sim - MIT Facing"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "15/15 sim - Gazebo Facing"
./renameSimGazeboURDF.sh
