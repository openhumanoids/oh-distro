#!/usr/bin/env python

import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import rospy
import math
import numpy

from mit_helios_scripts.msg import MITIRobotHandCalibrate
from mit_helios_scripts.msg import MITIRobotHandCurrentControlClose
from mit_helios_scripts.msg import MITIRobotHandPositionControlClose
from mit_helios_scripts.msg import MITIRobotHandSpread

def lower_case_side_string(side):
    if side == 'r':
        return 'right'
    elif side == 'l':
        return 'left'
    else:
        raise RuntimeError("Side not recognized: " + side)

def parseArguments():
    sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
    parser = argparse.ArgumentParser(description='Script for interacting with iRobot hand')

    # TODO: add help per argument
    
    defaultIndices = range(0, 3)
    
    parser.add_argument('side', help='side')
    parser.add_argument('--calibrate', metavar='IN_JIG', type=bool)
    parser.add_argument('--current', type=float)
    parser.add_argument('--position', type=float)
    parser.add_argument('--indices', nargs='*', metavar='INDEX', type=int, default=defaultIndices) # TODO: get default indices from somewhere
    parser.add_argument('--spread', metavar='SPREAD_ANGLE_DEG', type=float)
    parser.add_argument('--open', action='store_true')
    parser.add_argument('--close', action='store_true')

    args = parser.parse_args()
    side = args.side.lower()
    if side not in ['r', 'l']:
        raise RuntimeError("Side not recognized: " + side)
    
    if args.open:
        args.position = 0
    
    if args.close:
        args.current = 800
    
    if args.calibrate is not None:
        message = MITIRobotHandCalibrate()
        message.in_jig = args.calibrate
        publish(side, 'mit_calibrate', message)
    
    if args.current is not None:
        message = MITIRobotHandCurrentControlClose()
        message.current_milliamps = args.current
        message.valid = indicesToValid(args.indices)
        publish(side, 'mit_current_control_close', message)
    
    if args.position is not None:
        message = MITIRobotHandPositionControlClose()
        message.close_fraction = args.position
        message.valid = indicesToValid(args.indices)
        publish(side, 'mit_position_control_close', message)
    
    if args.spread is not None:
        message = MITIRobotHandSpread()
        message.angle_radians = numpy.deg2rad(args.spread)
        publish(side, 'mit_spread', message)

def publish(side, topic, message):
    fulltopic = '/irobot_hands/' + side + '_hand/' + topic
    publisher = rospy.Publisher(fulltopic, type(message))
    rospy.init_node('mit_irobot_hand_command_line')
    rospy.sleep(0.5)
    publisher.publish(message)
    rospy.sleep(0.5)

def indicesToValid(indices):
    ret = [False]*4
    for index in indices:
        ret[index] = True
    
    return ret

if __name__ == '__main__':
    parseArguments()
