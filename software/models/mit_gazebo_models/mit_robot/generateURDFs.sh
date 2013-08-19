rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf.xacro > model.urdf
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf_sim.xacro > model_sim.urdf
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf_sim.xacro > model_sim_gazebo.urdf
./renameSimGazeboURDF.sh
