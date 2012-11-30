#!/usr/bin/env python

import roslib; roslib.load_manifest('porterbot_scripts')
import rospy, math

from std_msgs.msg import Float64


def jointStateCommand():
	# Setup the publishers for each joint

	torso = rospy.Publisher('/torso_joint_position_controller/command', Float64)
	velodyne = rospy.Publisher('/velodyne_joint_position_controller/command', Float64)
	l_finger1 = rospy.Publisher('/l_finger1_joint_position_controller/command', Float64)
	l_finger2 = rospy.Publisher('/l_finger2_joint_position_controller/command', Float64)
	r_finger1 = rospy.Publisher('/r_finger1_joint_position_controller/command', Float64)
	r_finger2 = rospy.Publisher('/r_finger2_joint_position_controller/command', Float64)

 	l_j1 = rospy.Publisher('/l_j1_position_controller/command', Float64)
	l_j2 = rospy.Publisher('/l_j2_position_controller/command', Float64)
	l_j3 = rospy.Publisher('/l_j3_position_controller/command', Float64) 
	l_j4 = rospy.Publisher('/l_j4_position_controller/command', Float64)
	l_j5 = rospy.Publisher('/l_j5_position_controller/command', Float64)
	l_j6 = rospy.Publisher('/l_j6_position_controller/command', Float64)
	l_j7 = rospy.Publisher('/l_j7_position_controller/command', Float64)

	r_j1 = rospy.Publisher('/r_j1_position_controller/command', Float64)
	r_j2 = rospy.Publisher('/r_j2_position_controller/command', Float64)
	r_j3 = rospy.Publisher('/r_j3_position_controller/command', Float64) 
	r_j4 = rospy.Publisher('/r_j4_position_controller/command', Float64)
	r_j5 = rospy.Publisher('/r_j5_position_controller/command', Float64)
	r_j6 = rospy.Publisher('/r_j6_position_controller/command', Float64)
	r_j7 = rospy.Publisher('/r_j7_position_controller/command', Float64)


	# Initialize the node
	rospy.init_node('gripper_test')

	#Initial pose



	# Sleep for 2 second to wait for the initialsing of the node
	rospy.sleep(2)

	#This while loop will continue until ROS tells it to shutdown
	while not rospy.is_shutdown():

		t = 4 * rospy.get_time()
		l_finger = 0.0175 + 0.0175 * math.sin(t)

		rospy.sleep(0.01)

		l_finger1.publish(l_finger)
		l_finger2.publish(l_finger)

 



if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass
