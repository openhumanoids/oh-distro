#!/usr/bin/env python

import roslib; roslib.load_manifest('handle_macros')
import rospy

from handle_msgs.msg import HandleCollisions
from std_msgs.msg import String

frames = ['palm', 'finger0', 'finger1', 'finger2']
