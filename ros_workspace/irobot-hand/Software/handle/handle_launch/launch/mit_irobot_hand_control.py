#!/usr/bin/env python
'''
Created on Oct 30, 2013

@author: Twan
'''

import roslib; roslib.load_manifest('handle_launch')
import rospy
import argparse

from handle_msgs.msg import HandleControl

motor_indices = range(3) #range(3)

def set_command_message(command_message, motor_indices, control_type, value):
    for motor_index in motor_indices:
        command_message.valid[motor_index] = True
        command_message.type[motor_index] = control_type
        command_message.value[motor_index] = value

def loop_control(rospy, publisher, rate, max_time, control):
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < max_time and not rospy.is_shutdown():
        command_message = HandleControl()
        control(command_message)
        publisher.publish(command_message)
        rate.sleep()

def zero_current(publisher):
    no_current_message = HandleControl()
    set_command_message(no_current_message, motor_indices, HandleControl.CURRENT, 0)
    publisher.publish(no_current_message)

def close_hand_current_control(rospy, publisher, rate):
    grasp_time = 5
    finger_close_current = 1 # milliamps?
    def control(command_message):
        set_command_message(command_message, motor_indices, HandleControl.CURRENT, finger_close_current)

    loop_control(rospy, publisher, rate, grasp_time, control)

def open_hand(rospy, publisher, rate):
    open_time = 5
    open_angle = 10
    def control(command_message):
        set_command_message(command_message, motor_indices, HandleControl.ANGLE, open_angle)
        
    loop_control(rospy, publisher, rate, open_time, control)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for interacting with iRobot hand')
    parser.add_argument('commands', nargs='+', help='a list of commands, each one being CLOSE or OPEN')
    args = parser.parse_args()
    commands = args.commands
    print "Commands: " + ", ".join(commands)
    
    node_name = "mit_irobot_hand_control"
    publisher_name = "irobot_hands/r_hand/control"
    ros_rate = 100.0  # todo: something smarter
    
    rospy.init_node(node_name)
    rate = rospy.Rate(ros_rate)
    publisher = rospy.Publisher(publisher_name, HandleControl)
    
    for command in commands:
        print "Executing command: " + command
        if command == 'CLOSE':
            close_hand_current_control(rospy, publisher, rate)
        elif command == 'OPEN':
            open_hand(rospy, publisher, rate)
        else:
            raise RuntimeError("Command not recognized: " + command + "\n")

    print "Sending zero current message"
    zero_current(publisher)
