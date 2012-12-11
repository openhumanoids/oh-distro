
* This pod contains all local copies of the robot and different robot hand urdf files provided by osrf. 
* see WHOS_ROBOT to see who is maintaining which local copy.

A few rules to minimize effort in updating custom urdf descriptions when drcsim is updated to newer versions.

Maintaining local copies of meshes.
============================================
1) The meshes for all derived atlas robot sdf's are specified w.r.t. the mit_robot/meshes folder. This will be the one folder where latest mesh files need to placed and then converted to .obj and .wrl files via the provided shell scripts,  

2) The meshes for different osrf hands are in their respective folders in the mit_gazebo_models pod. E.g. sandia_hand/meshes, irobot_hand/meshes. This will make the robot sdfs for gazebo, drake and viewer all refer to the hand meshes via package://xxxxx_hand which is universal. These folders need to be also synced up with the latest mesh files from osrf and then converted to .obj and .wrl files via the meshlab shell scripts.

Maintaining sdf descriptions.
============================================
3) Every specific model has a xacro folder that contains custom xacro scripts which can to run to regenerate sdf files with any updated geometric information from osrf.

4) Some manual changes are necessary to update drake related urdfs. Convert .stl or .dae to .wrl and remove the top level comments in the .wrl which drake can't parse for now.

Maintainers: Maurice, Sisir and Scott.
