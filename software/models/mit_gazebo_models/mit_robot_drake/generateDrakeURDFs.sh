#!/bin/bash
 
echo "Generate Drake Robot Model Configurations:"

echo "Generating minimal contact versions"
rosrun xacro xacro.py xacro/atlas.urdf.xacro > model_minimal_contact.urdf
rosrun xacro xacro.py xacro/atlas_point_robotiq_hands.urdf.xacro > model_minimal_contact_point_hands.urdf
rosrun xacro xacro.py xacro/atlas_point_stumps.urdf.xacro > model_minimal_contact_point_stumps.urdf 

echo "Generating convex hull versions"
rosrun xacro xacro.py xacro/atlas_convex_hull_no_hands.urdf.xacro > model_convex_hull_no_hands.urdf
rosrun xacro xacro.py xacro/atlas_convex_hull_robotiq_hands.urdf.xacro > model_convex_hull_robotiq_hands.urdf
rosrun xacro xacro.py xacro/atlas_convex_hull_closed_robotiq_hands.urdf.xacro > model_convex_hull_closed_robotiq_hands.urdf
