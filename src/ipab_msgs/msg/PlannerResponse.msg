# A planning response message
# A translation of the data to the robot_plan_w_keyframes_t lcm message
Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
int32 info
string robot
int32 base_type
#0 fixed base
#10 floating base
#20 planar base
