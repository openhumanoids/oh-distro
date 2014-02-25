#!/bin/bash
 
echo "Generate Robot Model Configurations:"
# Robot configurations:
#echo "[I]Robot, [N]one, [S]andia, [H]ook, [P]ointer, [R]obotiq"

echo "bdi hooks"
rosrun xacro xacro.py xacro/atlas_LH_RH.urdf.xacro > model_LH_RH.urdf

echo "pointers"
rosrun xacro xacro.py xacro/atlas_LP_RP.urdf.xacro > model_LP_RP.urdf

echo "IRobot"
rosrun xacro xacro.py xacro/atlas_LI_RI.urdf.xacro > model_LI_RI.urdf

echo "Robotiq"
rosrun xacro xacro.py xacro/atlas_LR_RR.urdf.xacro > model_LR_RR.urdf


 
# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "17/18 sim - MIT Facing"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "18/18 sim - Gazebo Facing"
./renameSimGazeboURDF.sh

echo "19/18 sim - MIT Facing [2014]"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim_v3_v1_inertia.urdf.xacro > model_sim_v3_v1_inertia.urdf

