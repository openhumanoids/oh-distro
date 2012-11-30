#!/usr/bin/env python

import roslib; roslib.load_manifest('porterbot_scripts')
import rospy, math

from std_msgs.msg import Float64



def base(deg):
	# move the base
	torso = rospy.Publisher('/torso_joint_position_controller/command', Float64)
	rospy.init_node('base')
	print ( 'My base will move to position of ' + str(deg) + ' Degrees')
	# Sleep for 2 second to wait for the initialsing of the node
	rospy.sleep(2)
	rad=deg*3.141/180
	torso.publish(rad)
