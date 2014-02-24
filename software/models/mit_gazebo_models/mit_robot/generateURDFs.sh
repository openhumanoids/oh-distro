#!/bin/bash
 
echo "Generate Robot Model Configurations:"
# 9 real robot configurations:
#echo "[I]Robot, [N]one, [S]andia, [H]ook, [P]ointer, [R]obotiq"



echo "1. left pointer, right robotiq: valve"
rosrun xacro xacro.py xacro/atlas_LP_RR.urdf.xacro > model_LP_RR.urdf

echo "2. left irobot, right robotiq: hose, walking"
rosrun xacro xacro.py xacro/atlas_LI_RR.urdf.xacro > model_LI_RR.urdf

echo "3. left irobot, right robotiq, extenders: debris"
rosrun xacro xacro.py xacro/atlas_LI_RR_extender.urdf.xacro > model_LI_RR_extender.urdf

echo "4. bdi hooks: ladder"
rosrun xacro xacro.py xacro/atlas_LH_RH.urdf.xacro > model_LH_RH.urdf

echo "5. pointers with hook ends: door, driving"
rosrun xacro xacro.py xacro/atlas_LP_RP.urdf.xacro > model_LP_RP.urdf

echo "6. left robotiq, right pointer, special rotation in urdf: drill"
rosrun xacro xacro.py xacro/atlas_LQ_RP.urdf.xacro > model_LQ_RP.urdf
# was LQ_RP


echo "IRobot left combinations - 1"
rosrun xacro xacro.py xacro/atlas_LI_RI.urdf.xacro > model_LI_RI.urdf
#rosrun xacro xacro.py xacro/atlas_LI_RN.urdf.xacro > model_LI_RN.urdf
#rosrun xacro xacro.py xacro/atlas_LI_RS.urdf.xacro > model_LI_RS.urdf

#echo "None left combinations - 5"
#rosrun xacro xacro.py xacro/atlas_LN_RI.urdf.xacro > model_LN_RI.urdf
#rosrun xacro xacro.py xacro/atlas_LN_RN.urdf.xacro > model_LN_RN.urdf
#rosrun xacro xacro.py xacro/atlas_LN_RS.urdf.xacro > model_LN_RS.urdf
#rosrun xacro xacro.py xacro/atlas_LN_RR.urdf.xacro > model_LN_RR.urdf
#rosrun xacro xacro.py xacro/atlas_LN_RH.urdf.xacro > model_LN_RH.urdf

#echo "Sandia left combinations - 3"
#rosrun xacro xacro.py xacro/atlas_LS_RI.urdf.xacro > model_LS_RI.urdf
#rosrun xacro xacro.py xacro/atlas_LS_RN.urdf.xacro > model_LS_RN.urdf
#rosrun xacro xacro.py xacro/atlas_LS_RS.urdf.xacro > model_LS_RS.urdf

#echo "Hook left combinations - 5"
#rosrun xacro xacro.py xacro/atlas_LH_RI.urdf.xacro > model_LH_RI.urdf
#rosrun xacro xacro.py xacro/atlas_LH_RN.urdf.xacro > model_LH_RN.urdf
#rosrun xacro xacro.py xacro/atlas_LH_RH.urdf.xacro > model_LH_RH.urdf
#rosrun xacro xacro.py xacro/atlas_LH_RR.urdf.xacro > model_LH_RR.urdf
#rosrun xacro xacro.py xacro/atlas_LH_RQ.urdf.xacro > model_LH_RQ.urdf

#echo "Pointer left combinations - 0"

echo "Robotiq left combinations - 1"
rosrun xacro xacro.py xacro/atlas_LR_RR.urdf.xacro > model_LR_RR.urdf
#rosrun xacro xacro.py xacro/atlas_LR_RN.urdf.xacro > model_LR_RN.urdf
#rosrun xacro xacro.py xacro/atlas_LQ_RH.urdf.xacro > model_LQ_RH.urdf
#rosrun xacro xacro.py xacro/atlas_LQ_RI.urdf.xacro > model_LQ_RI.urdf

#echo "IRobot left - extender"
 
 
# the simulated robot configuration - LCM facing and ROS/gazbo facing versions
echo "17/18 sim - MIT Facing"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim.urdf.xacro > model_sim.urdf
echo "18/18 sim - Gazebo Facing"
./renameSimGazeboURDF.sh

echo "19/18 sim - MIT Facing [2014]"
rosrun xacro xacro.py xacro/atlas_sandia_hands_sim_v3_v1_inertia.urdf.xacro > model_sim_v3_v1_inertia.urdf

