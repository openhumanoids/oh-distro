#!/usr/bin/env python

import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import rospy
import numpy

from mit_helios_scripts.msg import MITIRobotHandCalibrate
from mit_helios_scripts.msg import MITIRobotHandCurrentControlClose
from mit_helios_scripts.msg import MITIRobotHandPositionControlClose
from mit_helios_scripts.msg import MITIRobotHandSpread
from IRobotHandController import IRobotHandController

default_open_fraction = 0
default_close_current = 800

def lower_case_side_string(side):
    if side == 'r':
        return 'right'
    elif side == 'l':
        return 'left'
    else:
        raise RuntimeError("Side not recognized: " + side)

def createParser():
    progname = sys.argv[0]
    examples = 'To open right hand, set spread to zero, calibrate in jig, close thumb 50%, close hand fully, and open again:\n'
    examples += progname + ' r --open\n'
    examples += progname + ' r --spread 0\n'
    examples += progname + ' r --calibrate 1\n'
    examples += progname + ' r --position 0.5 -i 2\n'
    examples += progname + ' r --close\n'
    examples += progname + ' r --open\n'
    
    parser = argparse.ArgumentParser(description='Command line tool for interacting with iRobot hand.',
                                     usage='%(prog)s side [options]',
                                     epilog=examples, formatter_class=argparse.RawDescriptionHelpFormatter)

    defaultIndices = IRobotHandController.get_close_hand_motor_indices()
    indicesChoice = IRobotHandController.get_nonspread_motor_indices()

    parser.add_argument('side', 
                        help='Side of hand to which command will be sent.',
                        choices=['r', 'R', 'l', 'L'])

    parser.add_argument('--calibrate', 
                        help='Calibrate motor tendon excursion encoders. Argument specifies whether hand is in calibration jig.', 
                        metavar='IN_JIG', type=bool)

    parser.add_argument('--current', 
                        help='Do current control. Argument is current in milliamps.',
                        type=float)
    
    parser.add_argument('--position', 
                        help='Do position control. Argument is number between 0 and 1 representing how far to close fingers.', 
                        metavar='FRACTION', type=float)
    
    parser.add_argument('--indices', '-i',
                        help='Finger indices on which control action should operate.', 
                        nargs='*', type=int, default=defaultIndices, choices=indicesChoice)
    
    parser.add_argument('--spread', 
                        help='Open spread DoF to desired angle in degrees.', 
                        metavar='DEGREES', type=float)
    
    parser.add_argument('--open', 
                        help='Fully open fingers. Shortcut for --position %s.' % default_open_fraction, 
                        action='store_true')
    
    parser.add_argument('--close', 
                        help='Current control close fingers. Shortcut for --current %s' % default_close_current, 
                        action='store_true')
    return parser


def publish(side, topic, message):
    fulltopic = '/irobot_hands/' + side + '_hand/' + topic
    publisher = rospy.Publisher(fulltopic, type(message))
    rospy.init_node('mit_irobot_hand_command_line')
    rospy.sleep(0.5)
    publisher.publish(message)
    rospy.sleep(0.5)

def indicesToBooleans(indices):
    ret = [False]*4
    for index in indices:
        ret[index] = True
    
    return ret

if __name__ == '__main__':
    sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
    parser = createParser()
    args = parser.parse_args()
    side = args.side.lower()
    
    if args.open:
        args.position = default_open_fraction
    
    if args.close:
        args.current = default_close_current
    
    if args.calibrate is not None:
        message = MITIRobotHandCalibrate()
        message.in_jig = args.calibrate
        publish(side, 'mit_calibrate', message)
    
    if args.current is not None:
        message = MITIRobotHandCurrentControlClose()
        message.current_milliamps = args.current
        message.valid = indicesToBooleans(args.indices)
        publish(side, 'mit_current_control_close', message)
    
    if args.position is not None:
        message = MITIRobotHandPositionControlClose()
        message.close_fraction = args.position
        message.valid = indicesToBooleans(args.indices)
        publish(side, 'mit_position_control_close', message)
    
    if args.spread is not None:
        message = MITIRobotHandSpread()
        message.angle_radians = numpy.deg2rad(args.spread)
        publish(side, 'mit_spread', message)
