#!/usr/bin/env python
#
# Software License Agreement (Apache License)
#
# Copyright 2013 Open Source Robotics Foundation
# Author: Brian Gerkey
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import roslib; roslib.load_manifest('sandia_hand_teleop')
import rospy, sys
from sensor_msgs.msg import Joy
from sandia_hand_msgs.msg import SimpleGrasp

USAGE = 'joy_to_simple_grasp.py {l|r|lr} [scene]'

# Axis to grasp name; activate button for each axis is axis number * 2
GRASP_MAPPING = {
  6 : 'cylindrical',
  7 : 'spherical',
  8 : 'prismatic'
}

# Scene button comes last
SCENE_BUTTON_INDEX = -1

class Grasper:
    def __init__(self, argv):
        self.srv_name = 'sandia_hands/%s/simple_grasp'
        self.hands = []
        self.pubs = []
        if len(argv) < 2 or len(argv) > 3:
            self.usage()
        if argv[1] == 'l':
            self.hands.append(self.srv_name%('l_hand'))
        elif argv[1] == 'r':
            self.hands.append(self.srv_name%('r_hand'))
        elif argv[1] == 'lr' or argv[1] == 'rl':
            self.hands.append(self.srv_name%('l_hand'))
            self.hands.append(self.srv_name%('r_hand'))
        else:
            self.usage()
        # If specified, we'll only do anything when the scene number matches
        self.scene = None
        if len(argv) == 3:
            self.scene = int(argv[2])

        rospy.init_node('grasp_teleop', anonymous=True)

        for s in self.hands:
            self.pubs.append(rospy.Publisher(s, SimpleGrasp))

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb)

    def usage(self):
        print USAGE
        sys.exit(1)

    def joy_cb(self, msg):
      if (self.scene is not None and 
          msg.buttons[SCENE_BUTTON_INDEX] != self.scene):
          return
      grasp = None
      amount = None
      for k,v in GRASP_MAPPING.iteritems():
          if msg.buttons[k*2] == 1:
              grasp = v
              amount = msg.axes[k]
              break
      if grasp is None:
          return
      for s in self.pubs:
          s.publish(SimpleGrasp(grasp, amount))

if __name__ == '__main__':
    g = Grasper(rospy.myargv())
    rospy.spin()
