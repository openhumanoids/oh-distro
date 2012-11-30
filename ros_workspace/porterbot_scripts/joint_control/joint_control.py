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
	rospy.init_node('home_position')

	#Initial pose

        print "input2"

	# Sleep for 2 second to wait for the initialsing of the node
	rospy.sleep(2)

	print "input"
	#!/usr/bin/env python


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
	rospy.init_node('joint_control')

	#Initial pose

	print "Wait 2 seconds for initialising the node"
	# Sleep for 2 second to wait for the initialsing of the node
	rospy.sleep(2)
	while not rospy.is_shutdown():
		b=input ("Desired Joint Position in Degree: ")
		a=b*3.141/180

		l_finger1.publish(a)
		l_finger2.publish(a)
		r_j1.publish(a)
		r_j2.publish(a)
		r_j3.publish(a)
		r_j4.publish(a)
		r_j5.publish(a)
		r_j6.publish(a)
		r_j7.publish(a)
		torso.publish(a)
		velodyne.publish(a)
		r_finger1.publish(a)
		r_finger2.publish(a)

		#l_j1.publish(a)
		#l_j2.publish(a)
		#l_j3.publish(a)
		#l_j4.publish(a)
		#l_j5.publish(a)
		#l_j6.publish(a)
		#l_j7.publish(a)





if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass

