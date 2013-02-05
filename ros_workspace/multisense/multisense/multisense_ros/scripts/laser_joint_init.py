#!/usr/bin/env python

import roslib; roslib.load_manifest('multisense_ros')
import rospy
import actionlib
from multisense_msgs.msg import SpindleControlAction, SpindleControlGoal


def main():
    rospy.init_node("laser_joint_init")
    ac = actionlib.SimpleActionClient("laser_joint/spindle_control", SpindleControlAction)
    while not rospy.is_shutdown() and not ac.wait_for_server(rospy.Duration(1.0)):
        print "Still waiting for action server 'laser_joint/spindle_control'"
    ac.send_goal_and_wait(SpindleControlGoal(rotational_speed=0.0), rospy.Duration(60.0), rospy.Duration(60.0))
    print "Finished initializing laser joint"
    return


if __name__ == "__main__":
    main()
