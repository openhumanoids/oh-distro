 This stack contains four nodes.

NOTE: MANUAL INSTALLATION IS REQUIRED FOR SOME NODES:

add 
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$HOME/drc/software/build/lib/pkgconfig/
to .bashrc file

and run rosmake for
- atlas_gazebo_plugins
- ROS2LCM_translator

MAINTAINERS: Sisir K.

DESCRIPTION:

1) atlas_description: contains the urdf and the xacro files for generating the urdf of a stick model of the atlas biped. 

2) atlas_gazebo_plugins: Contains a plugin that transmits true_robot_state from gazebo. The gazebo plugin is generated as a shared lib within atlas_gazebo_plugins/lib folder.  Must run rosmake within the atlas_gazebo_plugin folder. This is essential for atlas biped to be launched within gazebo. 

3) ROS2LCM_translator: This node contains a translator that links to lcm and drc_lcmtypes pkg-config files. Add this following line to your .bashrc file, so that ros can successfully build this node.

export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$HOME/drc/software/build/lib/pkgconfig/

Run rosmake within this folder, generates an executable in ROS2LCM_translator/bin.

4) sample_pr2_sensing: contains launch scripts to load a pr2 in gazebo with a stereo sensor and a scanning laser. No build is necessary for this node.


