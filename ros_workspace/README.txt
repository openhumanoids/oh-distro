 This stack contains four ROS nodes.

MAINTAINERS: Sisir K.

======== INSTALLATION =======

NOTE: MANUAL INSTALLATION IS REQUIRED FOR SOME NODES:
1. Add paths to shell:
echo "export PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:\$HOME/drc/software/build/lib/pkgconfig/" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$HOME/drc/ros_workspace" >> ~/.bashrc
. ~/.bashrc

2. run rosmake for:
cd ~/drc/ros_workspace/atlas_gazebo_plugins
rosmake 
cd ~/drc/ros_workspace/keyboard_teleop
rosmake
cd ~/drc/ros_workspace/ROS2LCM_translator
rosmake
cd ~/drc/ros_workspace/LCM2ROS_translator
rosmake


======== DESCRIPTION =======
1) atlas_description: contains the urdf and the xacro files for generating the urdf of a stick model of the atlas biped. 

2) atlas_gazebo_plugins: Contains a plugin that transmits true_robot_state from gazebo. The gazebo plugin is generated as a shared lib within atlas_gazebo_plugins/lib folder.  Must run rosmake within the atlas_gazebo_plugin folder. This is essential for atlas biped to be launched within gazebo. 

3) atlas_gazebo_plugins_pre_fuerte: Same as above, but plugin api is compatible with pre Fuerte Ros releases. Must run rosmake within the  folder. 

4) atlas_gazebo_msgs: Builds ROS msgs used in gazebo plugins seperately to avoid dependency issues with fuerte and pre-fuerte versions.

4) ROS2LCM_translator: This node contains a translator that links to lcm and drc_lcmtypes pkg-config files. Add this following line to your .bashrc file, so that ros can successfully build this node.

Run rosmake within this folder, generates an executable in ROS2LCM_translator/bin.

5) sample_pr2_sensing: contains launch scripts to load a pr2 in gazebo with a stereo sensor and a scanning laser. No build is necessary for this node.

6)keyboard_teleop: contains an executable that sends twist commands based on keyboard inputs. Used for teleoperation of diff drive robots. But can be used to command body twist (desired CoM vel and turnrate) messages for Bipeds.

