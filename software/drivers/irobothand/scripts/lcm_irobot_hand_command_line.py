#!/usr/bin/env python

import argparse, sys
import math, os
import numpy
import lcm

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from irobothand.current_control_close_t import current_control_close_t
from irobothand.position_control_close_t import position_control_close_t
from irobothand.spread_t import spread_t
from irobothand.calibrate_t import calibrate_t

def lower_case_side_string(side):
    if side == 'r':
        return 'right'
    elif side == 'l':
        return 'left'
    else:
        raise RuntimeError("Side not recognized: " + side)

def parseArguments():
    #sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
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
      
    if (side == 'r'):
      side = "RIGHT"
    elif (side == 'l'):
      side = "LEFT"
      
    
    if args.open:
        args.position = 0
    
    if args.close:
        args.current = 800
    
    if args.calibrate is not None:
        message = calibrate_t()
        message.in_jig = args.calibrate
        publish(side, 'CALIBRATE', message)
    
    if args.current is not None:
        message = current_control_close_t()
        message.current_milliamps = args.current
        message.valid = indicesToValid(args.indices)
        publish(side, 'CURRENT_CONTROL_CLOSE', message)
    
    if args.position is not None:
        message = position_control_close_t()
        message.close_fraction = args.position
        message.valid = indicesToValid(args.indices)
        publish(side, 'POSITION_CONTROL_CLOSE', message)
    
    if args.spread is not None:
        message = spread_t()
        message.angle_radians = numpy.deg2rad(args.spread)
        publish(side, 'SPREAD', message)

def publish(side, topic, message):
    fulltopic = 'IROBOT_' + side + '_' + topic
    lc = lcm.LCM()
    lc.publish(fulltopic, message.encode())

def indicesToValid(indices):
    ret = [False]*4
    for index in indices:
        ret[index] = True
    
    return ret

if __name__ == '__main__':
    parseArguments()
