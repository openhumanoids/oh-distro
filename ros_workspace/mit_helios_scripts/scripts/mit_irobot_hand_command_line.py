#!/usr/bin/env python
import rospy

import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import rospy

from std_msgs.msg import String
from sandia_hand_msgs.msg import SimpleGrasp


def parseArguments():
    sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
    parser = argparse.ArgumentParser(description='Script for interacting with iRobot hand')
    parser.add_argument('side', help='side')    
    parser.add_argument('name', help='name')
    parser.add_argument('closed_amount', help='closed_amount')
    args = parser.parse_args()
    side = args.side.lower()
    if side not in ['r', 'l']:
        raise RuntimeError("Side not recognized: " + side)
    name = args.name.lower()
    closed_amount = float(args.closed_amount)
    return (side,name,closed_amount)

if __name__ == '__main__':
    side,name,closed_amount = parseArguments()

    if (side == 'r'):
      pub = rospy.Publisher('/irobot_hands/r_hand/simple_command', SimpleGrasp)
    else:
      pub = rospy.Publisher('/irobot_hands/l_hand/simple_command', SimpleGrasp)
      
    rospy.init_node('mit_irobot_hand_command_line')
    str = "hello world"
    rospy.loginfo(str)

    sg = SimpleGrasp()
    sg.name = name
    sg.closed_amount = closed_amount
  
    pub.publish(sg)

