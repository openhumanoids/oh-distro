#!/usr/bin/env python

# Created on Oct 30, 2013
# 
# @author: Twan, Maurice



import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import rospy

from sandia_hand_msgs.msg import SimpleGrasp
from IRobotHandController import IRobotHandController


def simple_cmd_callback(controller, data):
    command = data.name
    if command in ['cylindrical','prismatic','spherical'] :
        closed_amount = data.closed_amount
        if epsilonEquals(closed_amount, 1):
            print("Close (current control)")
            controller.close_hand_current_control(800)
        else:
            print("Close (motor tendon excursion control; fraction: %s)" % closed_amount)
            controller.motor_excursion_control_close_fraction(closed_amount)
    elif command in ['twofinger_cylindrical','twofinger_prismatic','twofinger_spherical'] :
        if epsilonEquals(closed_amount, 1):
            print "Close two finger"
            # ... Close the two fingers only
        else:
            print "open two finger"
            # ... open the two fingers only
    elif data.name == 'calibrate_jig':
        print("Jig")
        controller.calibrate_motor_encoder_offsets(True)
    elif data.name == 'calibrate_no_jig':
        print("No Jig")
        controller.calibrate_motor_encoder_offsets(False)
    elif data.name == 'prismatic_spread':
        # 90 degrees
        print "Move to prismatic_spread"
    elif data.name == 'cylinderical_spread':
        # 60 degrees
        print "Move to cylinderical_spread"
    elif data.name == 'spherical_spread':
        # 0 degrees turn
        print "Move to spherical_spread"        
    else:
        print "Message not understood "+data.name
        return
    
    #print "Sending zero current message"
    controller.zero_current()
    #print "Finished"

def epsilonEquals(a, b):
    return abs(a - b) < 1e-3

def parseArguments():
    sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
    parser = argparse.ArgumentParser(description='Script for interacting with iRobot hand')
    parser.add_argument('side', help='hand side, l/L or r/R')
    args = parser.parse_args()
    side = args.side.lower()
    if side not in ['r', 'l']:
        raise RuntimeError("Side not recognized: " + side)
    return (side)

if __name__ == '__main__':
    side = parseArguments()
    controller = IRobotHandController(side)
    
    subscriber_simple_name = "simple_command"
    callback = lambda data : simple_cmd_callback(controller, data)
    subscriber_simple = rospy.Subscriber(subscriber_simple_name, SimpleGrasp, callback)

    rospy.spin()
