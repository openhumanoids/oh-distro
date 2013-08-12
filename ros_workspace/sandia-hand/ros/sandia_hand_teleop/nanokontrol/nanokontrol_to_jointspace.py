#!/usr/bin/env python
#
# Software License Agreement (Apache License)
#
# Copyright 2012 Open Source Robotics Foundation
# Author: Morgan Quigley
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
from sensor_msgs.msg import JointState

g_js_pub = None
g_js = JointState()

def joy_cb(msg):
  global g_js_pub, g_js
  x = msg.axes[0]
  if x < 0:
    x = 0
  elif x > 1:
    x = 1
  origin = [0] * 12
  g0 = [0] * 12
  grasp = 0
  if len(msg.buttons) >= 24:
    grasp = msg.buttons[24]
  if (grasp == 0):
    g0 = [0,1.5,1.7, 0,1.5,1.7, 0,1.5,1.7, -0.2,.8,1.2]
  elif (grasp == 1):
    origin = [0.7,0,0, -0.1,0,0, -0.7,0,0, 0,0,0]
    g0 = [0,1.4,1.4, 0,1.4,1.4, 0,1.4,1.4, 0,0.7,0.7]
  elif (grasp == 2):
    origin = [0,1.4,0, 0,1.4,0, 0,1.4,0, -0.1,0.8,-0.8]
    g0 = [0,0,1.4, 0,0,1.4, 0,0,1.4, 0,0,1.4]
    pass
  g_js.position = [0] * 12
  for i in xrange(0,12):
    g_js.position[i] = origin[i] + g0[i] * x
  g_js_pub.publish(g_js)

if __name__ == '__main__':
  rospy.init_node('nanokontrol_to_jointspace')
  g_js_pub = rospy.Publisher('joint_targets', JointState)
  g_js.name = ["f0_j0", "f0_j1", "f0_j2",
               "f1_j0", "f1_j1", "f1_j2",
               "f2_j0", "f2_j1", "f2_j2",
               "f3_j0", "f3_j1", "f3_j2"]
  g_js.position = [0] * 12
  rate = rospy.Rate(50.0) # max frequency of sending target joint positions
  joy_sub = rospy.Subscriber('joy', Joy, joy_cb)
  while not rospy.is_shutdown():
    # do something brilliant here someday
    rate.sleep()
