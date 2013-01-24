
Changes to get working within drake.

1) Mesh files are found from
$(find mit_drcsim_scripts)/models/xxxx_hand
instead of
package://xxxx_hand

2) dae and stl files need to be converted to wrl files with
the meshlab shell scripts in /model/xxxx_hand/meshes

3) Run the following xacro commands in the xacro folder to generate drake urdfs (These urdfs are not checked as the the paths are resolved for each machine by the $(find mit_drcsim_scripts) command in the c)

rosrun xacro xacro.py sandia_hand_left_drake.urdf.xacro > ../sandia_hand_left_drake.sdf
rosrun xacro xacro.py sandia_hand_right_drake.urdf.xacro > ../sandia_hand_right_drake.sdf
rosrun xacro xacro.py irobot_hand_right_drake.urdf.xacro > ../irobot_hand_drake.sdf



NOTE: Altered the sandia_hand_right_drake.urdf.xacro and the sadia_hand_drake.xacro to basically include a floating base to the hand, added sandia_hand_floating_base.xacro in drake_urdfs/xacro for this purpose. No alteration has been made to the sandia_hand_left_drake.xacro so that should still work as before however the sdf produced is wrong and will not load in rviz or gazebo

to create sdf file run as before:

rosrun xacro xacro.py sandia_hand_right_drake.urdf.xacro > ../sandia_hand_right_drake.sdf


