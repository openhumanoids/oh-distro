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
	rospy.init_node('homeposition')

	#Initial pose



	# Sleep for 2 second to wait for the initialsing of the node
	rospy.sleep(2)


	a=0
	l_finger1.publish(0)
	l_finger2.publish(0)
	r_j1.publish(a)
	r_j2.publish(a)
	r_j3.publish(a)
	r_j4.publish(a)
	r_j5.publish(a)
	r_j6.publish(a)
	r_j7.publish(a)
	torso.publish(0)
	velodyne.publish(0)
	r_finger1.publish(0)
	r_finger2.publish(0)

	l_j1.publish(0)
	l_j2.publish(0)
	l_j3.publish(0)
	l_j4.publish(0)
	l_j5.publish(0)
	l_j6.publish(0)
	l_j7.publish(0)



if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass
