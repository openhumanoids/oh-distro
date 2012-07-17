# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string robot_name
geometry_msgs/Pose body_pose
geometry_msgs/Twist body_twist
string[] joint_name
float64[] joint_position
float64[] joint_velocity

