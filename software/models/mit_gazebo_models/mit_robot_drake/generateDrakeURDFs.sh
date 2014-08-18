#!/bin/bash
 
echo "Generate Drake Robot Model Configurations:"

echo "Generating various minimal contact versions"
rosrun xacro xacro.py xacro/atlas.urdf.xacro > model_minimal_contact.urdf
rosrun xacro xacro.py xacro/atlas_full_contact_no_hands.urdf.xacro > model.urdf
rosrun xacro xacro.py xacro/atlas_full_contact_irobot_hands.urdf.xacro > model_irobot_hands.urdf
rosrun xacro xacro.py xacro/atlas_full_contact_no_hands.urdf.xacro > model_no_hands.urdf
rosrun xacro xacro.py xacro/atlas_fixedjoint_hands.urdf.xacro > model_minimal_contact_fixedjoint_hands.urdf
rosrun xacro xacro.py xacro/atlas_point_sandia_hands.urdf.xacro > model_minimal_contact_point_hands.urdf
rosrun xacro xacro.py xacro/atlas_point_stumps.urdf.xacro > model_minimal_contact_point_stumps.urdf 

echo "Generating various combinations e.g. LI_RN etc"
rosrun xacro xacro.py xacro/atlas_LN_RS.urdf.xacro > model_LN_RS.urdf
rosrun xacro xacro.py xacro/atlas_LS_RN.urdf.xacro > model_LS_RN.urdf
rosrun xacro xacro.py xacro/atlas_LS_RI.urdf.xacro > model_LS_RI.urdf
rosrun xacro xacro.py xacro/atlas_LI_RS.urdf.xacro > model_LI_RS.urdf
rosrun xacro xacro.py xacro/atlas_LI_RN.urdf.xacro > model_LI_RN.urdf
rosrun xacro xacro.py xacro/atlas_LN_RI.urdf.xacro > model_LN_RI.urdf
rosrun xacro xacro.py xacro/atlas_LI_RI.urdf.xacro > model_LI_RI.urdf
rosrun xacro xacro.py xacro/atlas_LN_RN.urdf.xacro > model_LN_RN.urdf


#rosrun xacro xacro.py xacro/sandia_hand_left.urdf.xacro > sandia_hand_left.urdf
#rosrun xacro xacro.py xacro/sandia_hand_right.urdf.xacro > sandia_hand_right.urdf

#echo "irobot hands"
#rosrun xacro xacro.py xacro/irobot_hand_left.urdf.xacro > irobot_hand_left.urdf
#rosrun xacro xacro.py xacro/irobot_hand_right.urdf.xacro > irobot_hand_right.urdf

#echo "robotiq hands"
#rosrun xacro xacro.py xacro/robotiq_hand_left.urdf.xacro > robotiq_hand_left.urdf
#rosrun xacro xacro.py xacro/robotiq_hand_right.urdf.xacro > robotiq_hand_right.urdf

#echo "pointer hands"
#rosrun xacro xacro.py xacro/pointer_hand_left.urdf.xacro > pointer_hand_left.urdf
#rosrun xacro xacro.py xacro/pointer_hand_right.urdf.xacro > pointer_hand_right.urdf

# Valkyrie hands are not generated from xacro
