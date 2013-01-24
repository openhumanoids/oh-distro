#!/usr/bin/env python

import roslib; roslib.load_manifest('porterbot_scripts')
import rospy, math

from std_msgs.msg import Float64


def jointStateCommand():
	# Setup the publishers for each joint

	torso = rospy.Publisher('/torso_joint_position_controller/command', Float64)
	l_j1 = rospy.Publisher('/l_j1_position_controller/command', Float64)
	l_j2 = rospy.Publisher('/l_j2_position_controller/command', Float64)
	l_j3 = rospy.Publisher('/l_j3_position_controller/command', Float64) 
	l_j4 = rospy.Publisher('/l_j4_position_controller/command', Float64)
	l_j5 = rospy.Publisher('/l_j5_position_controller/command', Float64)
	l_j6 = rospy.Publisher('/l_j6_position_controller/command', Float64)
	l_j7 = rospy.Publisher('/l_j7_position_controller/command', Float64)


	# Initialize the node
	rospy.init_node('safetyposition')

	#Initial pose



	# Sleep for 2 second to wait for the initialsing of the node
	rospy.sleep(2)

	torso.publish(-0.3)
	l_j1.publish(-0.1)
	l_j2.publish(-1.3)
	l_j3.publish(0)
	l_j4.publish(0)
	l_j5.publish(0)
	l_j6.publish(0)
	l_j7.publish(0)


	rospy.sleep(2)

	torso.publish(0.3)
	l_j1.publish(-0.1)
	l_j2.publish(1.3)
	l_j3.publish(0)
	l_j4.publish(0)
	l_j5.publish(0)
	l_j6.publish(0)
	l_j7.publish(0)


	rospy.sleep(2)

	torso.publish(0)


if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass
