
This package has a launch file to do spawn a PR2 in gazebo with a table and a coffee cup.  RVIZ is also launched to visualize the tilting laser scans and stereo point clouds. 

Run 
0) roscore
1) roslaunch sample_pr2_sensing sample_pr2_world.launch
2) roslaunch pr2_teleop teleop_keyboard.launch (to teleop pr2 from the keyboard, NOTE: this term has to be the active window for the keyboard commands to be published)

- 
Alternatively, one could run the following commands in multiple terminals (if the former approach fails for some reason).
0) roscore
1) roslaunch gazebo_worlds empty_world.launch
2) roslaunch pr2_gazebo pr2.launch
3) roslaunch gazebo_worlds table.launch
4) roslaunch gazebo_worlds coffee_cup.launch
5) rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
6) rosrun rviz rviz -d display_config_pr2_laserscan_stereopointcloud.vcg 
7) roslaunch pr2_teleop teleop_keyboard.launch


Run 'rostopic list' in a terminal to see all the published messages. There should a \tilt_scan msg and a 
/wide_stereo/points for stereo point cloud, and a /wide_stereo/left/image_color for a color image.

Tested under ROS Electric + Ubuntu 11.10
Sisir 2012

