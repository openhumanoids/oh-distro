#!/usr/bin/env python
'''
Created on Oct 30, 2013

@author: Twan
'''

import roslib; roslib.load_manifest('handle_launch')
import rospy

from handle_msgs.msg import HandleControl

def do_control(command_message, motor_indices, control_type, value):
    for motor_index in motor_indices:
        command_message.valid[motor_index] = True
        command_message.type[motor_index] = control_type
        command_message.value[motor_index] = value
    

if __name__ == '__main__':
    node_name = "close_finger_current_control"
    publisher_name = "irobot_hands/r_hand/control"
    ros_rate = 100.0  # todo: something smarter

    rospy.init_node(node_name)
    rate = rospy.Rate(ros_rate)
    publisher = rospy.Publisher(publisher_name, HandleControl)

    start_time = rospy.get_time()
    
    
    open_time = 5
    open_angle = 10


    finger_close_current = 1 # milliamps?
    motor_indices = range(3) #range(3)


    done = False
    close_started = False
    open_started = False
    
    while not rospy.is_shutdown() and not done:
        time = rospy.get_time() - start_time
        
        command_message = HandleControl()
        if time < open_time:
            if not open_started:
                print("Opening")
                open_started = True
            do_control(command_message, motor_indices, HandleControl.ANGLE, open_angle)
        else:
            done = True

#         print command_message
        publisher.publish(command_message)
        rate.sleep()
    
    print("sending command: no current")
    no_current_message = HandleControl()
    do_control(no_current_message, motor_indices, HandleControl.CURRENT, 0)
    publisher.publish(no_current_message)
    rospy.spin()
