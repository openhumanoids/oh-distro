rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf.xacro > model/model.urdf
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf_sim.xacro > model/model_sim.urdf
rosrun xacro xacro.py xacro/atlas_sandia_hands.urdf_sim.xacro > model/model_sim_gazebo.urdf
renameSimGazeboURDF.sh

