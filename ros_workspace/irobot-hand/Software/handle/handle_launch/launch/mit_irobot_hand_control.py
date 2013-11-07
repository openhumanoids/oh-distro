#!/usr/bin/env python

# Created on Oct 30, 2013
# 
# @author: Twan



import roslib; roslib.load_manifest('handle_launch')
import rospy
import argparse

from handle_msgs.msg import HandleControl
from handle_msgs.msg import HandleSensors
from IRobotHandConfigParser import IRobotHandConfigParser

motor_indices = range(3)

def set_command_message(command_message, motor_indices, control_type, value):
    for motor_index in motor_indices:
        command_message.valid[motor_index] = True
        command_message.type[motor_index] = control_type
        command_message.value[motor_index] = value

def loop_control(publisher, rate, max_time, control):
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < max_time and not rospy.is_shutdown():
        command_message = HandleControl()
        control(command_message)
        publisher.publish(command_message)
        rate.sleep()

class IRobotHandController(object):

    def __init__(self):
        self.sensors = HandleSensors()
        self.config_parser = IRobotHandConfigParser(side)
        self.config_parser.load()
    
        node_name = "mit_irobot_hand_control"
        message_base_name = "irobot_hands/" + side + "_hand/"
        publisher_name = message_base_name + "control"
        subscriber_name = message_base_name + "sensors/raw"
        ros_rate = 100.0  # todo: something smarter
        
        rospy.init_node(node_name)
        self.rate = rospy.Rate(ros_rate)
        self.publisher = rospy.Publisher(publisher_name, HandleControl)
        self.subscriber = rospy.Subscriber(subscriber_name, HandleSensors, self.sensor_data_callback)
    
    def sensor_data_callback(self, data):
        self.sensors = data
    

    def zero_current(self):
        no_current_message = HandleControl()
        set_command_message(no_current_message, motor_indices, HandleControl.CURRENT, 0)
        self.publisher.publish(no_current_message)

    def close_hand_current_control(self):
        grasp_time = 5
        finger_close_current = 200 # milliamps
        def control(command_message):
            set_command_message(command_message, motor_indices, HandleControl.CURRENT, finger_close_current)
    
        loop_control(self.publisher, self.rate, grasp_time, control)
    
    def open_hand(self):
        open_time = 5
        open_angle = 10
        def control(command_message):
            set_command_message(command_message, motor_indices, HandleControl.ANGLE, open_angle)
            
        loop_control(self.publisher, self.rate, open_time, control)
    
    def clear_config(self):
        self.config_parser.clear()
        
    def calibrate_motor_encoder_offsets(self):
        self.close_hand_current_control()
        for motor_index in motor_indices:
            current_value = self.sensors.motorHallEncoder[motor_index]
            self.config_parser.set_motor_encoder_offset(motor_index, current_value)

    def exit(self):
        self.config_parser.save()

def parseArguments():
    parser = argparse.ArgumentParser(description='Script for interacting with iRobot hand')
    parser.add_argument('side', help='hand side, l or r')
    parser.add_argument('commands', nargs='+', help='a list of commands, each one being CLOSE or OPEN')
    args = parser.parse_args()
    print "Commands: " + ", ".join(args.commands)
    return (args.side, args.commands)

if __name__ == '__main__':
    side, commands = parseArguments()
    controller = IRobotHandController()

    for command in commands:
        print "Executing command: " + command
        if command == 'CLOSE':
            controller.close_hand_current_control()
        elif command == 'OPEN':
            controller.open_hand()
        elif command == 'CLEAR_CONFIG':
            controller.clear_config()
        elif command == 'CALIBRATE':
            controller.calibrate_motor_encoder_offsets()
        else:
            raise RuntimeError("Command not recognized: " + command + "\n")

    print "Sending zero current message"
    controller.zero_current()
    controller.exit()
