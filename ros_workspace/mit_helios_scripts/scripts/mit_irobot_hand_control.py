#!/usr/bin/env python

# Created on Oct 30, 2013
# 
# @author: Twan, Maurice



import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import rospy

from mit_helios_scripts.msg import MITIRobotHandCalibrate
from mit_helios_scripts.msg import MITIRobotHandCurrentControlClose
from mit_helios_scripts.msg import MITIRobotHandPositionControlClose
from mit_helios_scripts.msg import MITIRobotHandSpread

from IRobotHandController import IRobotHandController

def validIndices(validArray):
    return [i for i, valid in enumerate(validArray) if valid]

def calibrate_callback(controller, message):
    print("Calibrate: in jig: %s" % message.in_jig)
    controller.calibrate_motor_encoder_offsets(message.in_jig)
    controller.zero_current()

def current_control_close_callback(controller, message):
    indices = validIndices(message.valid)
    controller.close_hand_current_control(message.current_milliamps, indices)
    controller.zero_current()

def position_control_close_callback(controller, message):
    indices = validIndices(message.valid)
    controller.motor_excursion_control_close_fraction(message.close_fraction, indices)
    controller.zero_current()

def spread_callback(controller, message):
    controller.spread_angle_control(message.angle_radians)
    controller.zero_current()

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
    
    rospy.Subscriber("mit_calibrate", MITIRobotHandCalibrate, lambda message : calibrate_callback(controller, message))
    rospy.Subscriber("mit_current_control_close", MITIRobotHandCurrentControlClose, lambda message : current_control_close_callback(controller, message))
    rospy.Subscriber("mit_position_control_close", MITIRobotHandPositionControlClose, lambda message : position_control_close_callback(controller, message))
    rospy.Subscriber("mit_spread", MITIRobotHandSpread, lambda message : spread_callback(controller, message))
    
    rospy.spin()
