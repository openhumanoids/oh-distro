#!/usr/bin/env python

import roslib; roslib.load_manifest('porterbot_scripts')
import rospy, math

from std_msgs.msg import Float64

def jointStateCommand():
    # Setup the publishers for each joint
    l_j1 = rospy.Publisher('/l_j1_position_controller/command', Float64)
    l_j2 = rospy.Publisher('/l_j2_position_controller/command', Float64)
    l_j3 = rospy.Publisher('/l_j3_position_controller/command', Float64) 
    l_j4 = rospy.Publisher('/l_j4_position_controller/command', Float64)
    l_j5 = rospy.Publisher('/l_j5_position_controller/command', Float64)
    l_j6 = rospy.Publisher('/l_j6_position_controller/command', Float64)
    l_j7 = rospy.Publisher('/l_j7_position_controller/command', Float64)

    # Initialize the node
    rospy.init_node('scan_left')

    # Sleep for 1 second to wait for intit the node
    rospy.sleep(1)

    #Initial pose
    l_j1.publish(-0.4)
    l_j2.publish(0.6)
    l_j3.publish(0)
    l_j4.publish(0.3)
    l_j5.publish(0)
    l_j6.publish(0.3)
    l_j7.publish(0)
    # Sleep for 3 second to wait for the scan position
    rospy.sleep(3)

    #This while loop will continue until ROS tells it to shutdown
    while not rospy.is_shutdown():
        t = 2 * rospy.get_time()
        l_scan_move = -0 + 0.15 * math.cos(t)

	l_j5.publish(l_scan_move)


        # Wait 0.01 second
        rospy.sleep(0.01)



if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass

