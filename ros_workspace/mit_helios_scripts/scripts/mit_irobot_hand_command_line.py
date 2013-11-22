#!/usr/bin/env python

import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import numpy
import lcm

from irobothand.calibrate_t import calibrate_t
from irobothand.current_control_close_t import current_control_close_t
from irobothand.position_control_close_t import position_control_close_t
from irobothand.spread_t import spread_t
from irobothand.calibrate_tactile_t import calibrate_tactile_t

from IRobotHandController import IRobotHandController
import mit_irobot_hand_control

default_open_fraction = 0
default_close_current = 800

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
    indicesChoice = IRobotHandController.get_non_spread_motor_indices()

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
    
    parser.add_argument('--calibrate_tactile', 
                    help='Current control close fingers. Shortcut for --current %s' % default_close_current, 
                    action='store_true')
    
    return parser


def publish(side, topic, message):
    fulltopic = mit_irobot_hand_control.get_lcm_channel(side, topic)
    lc = lcm.LCM()
    lc.publish(fulltopic, message.encode())

def indicesToBooleans(indices):
    ret = [False]* len(IRobotHandController.get_non_spread_motor_indices())
    for index in indices:
        ret[index] = True
    
    return ret

if __name__ == '__main__':
#     sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
    parser = createParser()
    args = parser.parse_args()
    side = args.side.lower()
    
    if args.open:
        args.position = default_open_fraction
    
    if args.close:
        args.current = default_close_current
    
    if args.calibrate_tactile:
        message = calibrate_tactile_t()
        publish(side, mit_irobot_hand_control.calibrate_tactile_channel, message)
    
    if args.calibrate is not None:
        message = calibrate_t()
        message.in_jig = args.calibrate
        publish(side, mit_irobot_hand_control.calibrate_channel, message)
    
    if args.current is not None:
        message = current_control_close_t()
        message.current_milliamps = args.current
        message.valid = indicesToBooleans(args.indices)
        publish(side, mit_irobot_hand_control.current_control_channel, message)
    
    if args.position is not None:
        message = position_control_close_t()
        message.close_fraction = args.position
        message.valid = indicesToBooleans(args.indices)
        publish(side, mit_irobot_hand_control.position_control_channel, message)
    
    if args.spread is not None:
        message = spread_t()
        message.angle_radians = numpy.deg2rad(args.spread)
        publish(side, mit_irobot_hand_control.spread_channel, message)
