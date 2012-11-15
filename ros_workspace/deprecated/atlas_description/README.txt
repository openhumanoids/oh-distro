
To generate URDF from xacro files, run the following terminal command:
rosrun xacro xacro.py atlas_robot.urdf.xacro > atlas_robot.urdf


To investigate the kinematic properties of the model in Rviz, run the following from a terminal:
roscd atlas_description/urdf
roslaunch atlas_description displayInRviz.launch model:=atlas_robot.urdf.xacro gui:=True

To spawn the URDF in gazebo, run:
roslaunch atlas_description atlas_gazebo.launch
- 
Alternatively, one could run the following commands in multiple terminals (more stable).
1) roslaunch gazebo_worlds empty_world.launch
2) rosservice call /gazebo/pause_physics
3) roslaunch atlas_decription atlas_spawn_gazebo.launch


Tested under ROS Electric + Ubuntu 11.10
Sisir 2012

