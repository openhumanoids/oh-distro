#!/usr/bin/env python
# '''
# Created on Oct 30, 2013
# 
# @author: Twan
# '''

import roslib; roslib.load_manifest('handle_launch')
import rospy

from handle_msgs.msg import HandleControl

def do_control(current):
    motor_index = 0;
    command_message = HandleControl()
    command_message.valid[motor_index] = True
    command_message.type[motor_index] = command_message.CURRENT
    command_message.value[motor_index] = current
    return command_message

if __name__ == '__main__':
    node_name = "close_finger_current_control"
    publisher_name = "irobot_hands/r_hand/control"
    ros_rate = 100.0  # todo: something smarter
    current = 1 # amps

    rospy.init_node(node_name)
    rate = rospy.Rate(ros_rate)
    publisher = rospy.Publisher(publisher_name, HandleControl)
    
    while not rospy.is_shutdown():
        command_message = do_control(current)
        publisher.publish(command_message)
        rate.sleep()


    