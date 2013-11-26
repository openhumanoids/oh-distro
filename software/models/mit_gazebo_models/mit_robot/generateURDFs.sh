#!/bin/bash

echo "Generate Robot Model Configurations:"
# 9 real robot configurations:
echo "1/18 LI RI"
#rosrun xacro xacro.py xacro/atlas_LI_RI.urdf.xacro > model_LI_RI.urdf
echo "2/18 LI RN"
#rosrun xacro xacro.py xacro/atlas_LI_RN.urdf.xacro > model_LI_RN.urdf
echo "3/18 LI RS"
#rosrun xacro xacro.py xacro/atlas_LI_RS.urdf.xacro > model_LI_RS.urdf

echo "4/18 LN RI"
#rosrun xacro xacro.py xacro/atlas_LN_RI.urdf.xacro > model_LN_RI.urdf
echo "5/18 LN RN"
#rosrun xacro xacro.py xacro/atlas_LN_RN.urdf.xacro > model_LN_RN.urdf
echo "6/18 LN RS"
#rosrun xacro xacro.py xacro/atlas_LN_RS.urdf.xacro > model_LN_RS.urdf
echo "7/18 LN RR"
rosrun xacro xacro.py xacro/atlas_LN_RR.urdf.xacro > model_LN_RR.urdf
echo "8/18 LR RN"
rosrun xacro xacro.py xacro/atlas_LR_RN.urdf.xacro > model_LR_RN.urdf

echo "9/18 LS RI"
#rosrun xacro xacro.py xacro/atlas_LS_RI.urdf.xacro > model_LS_RI.urdf
echo "10/18 LS RN"
#rosrun xacro xacro.py xacro/atlas_LS_RN.urdf.xacro > model_LS_RN.urdf
echo "11/18 LS RS"
#rosrun xacro xacro.py xacro/atlas_LS_RS.urdf.xacro > model_LS_RS.urdf

echo "12/18 LH RI"
rosrun xacro xacro.py xacro/atlas_LH_RI.urdf.xacro > model_LH_RI.urdf
echo "13/18 LH RN"
rosrun xacro xacro.py xacro/atlas_LH_RN.urdf.xacro > model_LH_RN.urdf
echo "14/18 LN RH"
rosrun xacro xacro.py xacro/atlas_LN_RH.urdf.xacro > model_LN_RH.urdf
echo "15/18 LH RH"
rosrun xacro xacro.py xacro/atlas_LH_RH.urdf.xacro > model_LH_RH.urdf

echo "16/18 LH RR"
rosrun xacro xacro.py xacro/atlas_LH_RR.urdf.xacro > model_LH_RR.urdf


# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "17/18 sim - MIT Facing"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "18/18 sim - Gazebo Facing"
./renameSimGazeboURDF.sh
