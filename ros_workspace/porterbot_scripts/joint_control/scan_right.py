#!/usr/bin/env python

import roslib; roslib.load_manifest('porterbot_scripts')
import rospy, math

from std_msgs.msg import Float64

def jointStateCommand():
    # Setup the publishers for each joint
    r_j1 = rospy.Publisher('/r_j1_position_controller/command', Float64)
    r_j2 = rospy.Publisher('/r_j2_position_controller/command', Float64)
    r_j3 = rospy.Publisher('/r_j3_position_controller/command', Float64) 
    r_j4 = rospy.Publisher('/r_j4_position_controller/command', Float64)
    r_j5 = rospy.Publisher('/r_j5_position_controller/command', Float64)
    r_j6 = rospy.Publisher('/r_j6_position_controller/command', Float64)
    r_j7 = rospy.Publisher('/r_j7_position_controller/command', Float64)

    # Initialize the node
    rospy.init_node('scan_right')

    # Sleep for 1 second to wait for intit the node
    rospy.sleep(1)

    #Initial pose
    r_j1.publish(.8)
    r_j2.publish(1.2)
    r_j3.publish(1)
    r_j4.publish(-1.3)
    r_j5.publish(-1.5)
    r_j6.publish(0)
    r_j7.publish(0)
    # Sleep for 3 second to wait for the scan position
    rospy.sleep(3)

    #This while loop will continue until ROS tells it to shutdown
    while not rospy.is_shutdown():
        t = 2 * rospy.get_time()
        r_scan_move = -1.3 + 0.5 * math.cos(t)

	r_j4.publish(r_scan_move)


        # Wait 0.01 second
        rospy.sleep(0.01)



if __name__ == '__main__':
    try:
        jointStateCommand()
    except rospy.ROSInterruptException: pass

