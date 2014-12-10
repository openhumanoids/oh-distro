#!/usr/bin/env python
#
# Software License Agreement (Apache License)
#
# Copyright 2013 Open Source Robotics Foundation
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

import sys,os
import lcm
base_dir =os.getenv("DRC_BASE")
sys.path.append(base_dir + "/software/build/lib/python2.7/dist-packages")
sys.path.append(base_dir + "/software/build/lib/python2.7/site-packages")
from drc.sandia_simple_grasp_t import sandia_simple_grasp_t
from drc.joint_command_t import joint_command_t

g_jc_pub = None


left_names = ["left_finger[0]/joint_base_rotation", "left_finger[0]/joint_base",
        "left_finger[0]/joint_flex", "left_finger[1]/joint_base_rotation", 
        "left_finger[1]/joint_base", "left_finger[1]/joint_flex",
        "left_finger[2]/joint_base", "left_finger[2]/joint_flex"]

right_names = ["right_finger[0]/joint_base_rotation", "right_finger[0]/joint_base",
        "right_finger[0]/joint_flex", "right_finger[1]/joint_base_rotation", 
        "right_finger[1]/joint_base", "right_finger[1]/joint_flex",
        "right_finger[2]/joint_base", "right_finger[2]/joint_flex"]


jc = joint_command_t()
jc.robot_name = "irobot"
jc.num_joints = 8
jc.name = ["correct_joint_name_here"]*8
jc.position = [0] * 8
jc.velocity = [0] * 8
jc.effort = [0] * 8

jc.kp_position = [0] * 8
jc.ki_position = [0] * 8
jc.kd_position = [0] * 8
jc.kp_velocity = [0] * 8

jc.i_effort_min = [0] * 8
jc.i_effort_max = [0] * 8

# From lcm2ros_hands.cpp
# hall = position * 3500/(0.5*M_PI)
#0->1 in 0.25
# 1 0
# 2 1500
# 3 3000
# 4 4500
# 5 6000

def grasp_cb(msg):
  global g_jc_pub, g_jc
  print "request: grasp [%s] amount [%f]" % (msg.name, msg.closed_amount)
  # save some typing
  gn = msg.name
  x = msg.closed_amount
  if x < 0:
    x = 0
  elif x > 1:
    x = 1
  g0 = [2.7] * 12

  jc.position = [0] * 8
  for i in xrange(0, 8):
    jc.position[i] = g0[i] *x
  
  # joint commands to produce equivalent angle spreads in hall values:
  if (gn == "cylindrical"):
    jc.position[0] = 0
    jc.position[3] = 0
  elif (gn == "spherical"):
    jc.position[0] = 1.08
    jc.position[3] = 1.08
  elif (gn == "prismatic"):
    jc.position[0] = 1.57
    jc.position[3] = 1.57
  else:
    return None # bogus

  print "joint state: %s" % (str(jc.position))

def on_simple_grasp_left(channel, data):
  m = sandia_simple_grasp_t.decode(data)
  print "L"
  grasp_cb(m)
  jc.name = left_names
  lc.publish("L_HAND_JOINT_COMMANDS", jc.encode())
  

def on_simple_grasp_right(channel, data):
  m = sandia_simple_grasp_t.decode(data)
  print "R"
  grasp_cb(m)
  jc.name = right_names
  lc.publish("R_HAND_JOINT_COMMANDS", jc.encode())

lc = lcm.LCM()
print "iRobot simple grasp service is now running"
lc.subscribe("IROBOT_LEFT_SIMPLE_GRASP", on_simple_grasp_left)
lc.subscribe("IROBOT_RIGHT_SIMPLE_GRASP", on_simple_grasp_right)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

if __name__ == '__main__':
  g_jc.position = [0] * 12
  print "iRobot simple grasp service is now running."
