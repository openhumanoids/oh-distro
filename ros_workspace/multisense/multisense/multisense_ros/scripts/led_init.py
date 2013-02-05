#!/usr/bin/env python

import roslib; roslib.load_manifest('multisense_ros')
import rospy
import actionlib
from multisense_msgs.msg import LedControlAction, LedControlGoal


def main():
    rospy.init_node("led_init")
    ac = actionlib.SimpleActionClient("led_control", LedControlAction)
    while not rospy.is_shutdown() and not ac.wait_for_server(rospy.Duration(1.0)):
        print "Still waiting for action server 'led_control'"
    ac.send_goal_and_wait(LedControlGoal(flashing=False, intensities=[0.0, 0.0, 0.0, 0.0]), rospy.Duration(60.0), rospy.Duration(60.0))
    print "Finished initializing led"
    return


if __name__ == "__main__":
    main()
