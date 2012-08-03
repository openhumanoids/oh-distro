# This represent the actuator command on each joint
Header header
string robot_name
string[] joint_name
float64[] joint_effort
# Also need durations (usually dt, after which torques expire).
