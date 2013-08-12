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

import roslib; roslib.load_manifest('sandia_hand_teleop')
import rospy, sys, os
import pygame
from pygame.locals import *
from sandia_hand_msgs.msg import SimpleGrasp

#os.environ["SDL_VIDEODRIVER"] = "dummy"
g_right_grasp_pub = None
g_left_grasp_pub = None
g_key_map = { '1': [ 'L', 'cylindrical', 0.00],
              '2': [ 'L', 'cylindrical', 0.25],
              '3': [ 'L', 'cylindrical', 0.50],
              '4': [ 'L', 'cylindrical', 0.75],
              '5': [ 'L', 'cylindrical', 0.95],
              '6': [ 'R', 'cylindrical', 0.00],
              '7': [ 'R', 'cylindrical', 0.25],
              '8': [ 'R', 'cylindrical', 0.50],
              '9': [ 'R', 'cylindrical', 0.75],
              '0': [ 'R', 'cylindrical', 0.95],
              'q': [ 'L', 'spherical'  , 0.00],
              'w': [ 'L', 'spherical'  , 0.25],
              'e': [ 'L', 'spherical'  , 0.50],
              'r': [ 'L', 'spherical'  , 0.75],
              't': [ 'L', 'spherical'  , 0.95],
              'y': [ 'R', 'spherical'  , 0.00],
              'u': [ 'R', 'spherical'  , 0.25],
              'i': [ 'R', 'spherical'  , 0.50],
              'o': [ 'R', 'spherical'  , 0.75],
              'p': [ 'R', 'spherical'  , 0.95],
              'a': [ 'L', 'prismatic'  , 0.00],
              's': [ 'L', 'prismatic'  , 0.25],
              'd': [ 'L', 'prismatic'  , 0.50],
              'f': [ 'L', 'prismatic'  , 0.75],
              'g': [ 'L', 'prismatic'  , 0.95],
              'h': [ 'R', 'prismatic'  , 0.00],
              'j': [ 'R', 'prismatic'  , 0.25],
              'k': [ 'R', 'prismatic'  , 0.50],
              'l': [ 'R', 'prismatic'  , 0.75],
              ';': [ 'R', 'prismatic'  , 0.95],
            }

def keypress(c):
  print "handling keypress: %s" % c
  if c in g_key_map:
    pub = None
    if g_key_map[c][0] == 'L':
      pub = g_left_grasp_pub
    elif g_key_map[c][0] == 'R':
      pub = g_right_grasp_pub
    else:
      return # wtf
    sg = SimpleGrasp(g_key_map[c][1], g_key_map[c][2])
    pub.publish(sg)

def print_usage():
  print "Each row corresponds to a canonical grasp:"
  print "  top row of letters    = cylindrical (power)"
  print "  second row of letters = spherical"
  print "  third row of letters  = prismatic"
  print ""
  print "Each column corresponds to an amount to open/close the grasp:"
  print "  leftmost column = fully open"
  print "  second column   = 25%  closed"
  print "  third column    = 50%  closed"
  print "  fourth column   = 75%  closed"
  print "  fifth column    = 100% closed"
  print ""
  print "The left side of the keyboard controls the left (or only) hand."
  print "Right side of the keyboard, starting at 'Y', controls the right hand."
  print ""
  print "Example 1: a fully open cylindrical grasp for the left/only hand "
  print "is commnanded by pressing 'q'. For the right hand, press 'y'."
  print ""
  print "Example 2: fully closed spherical grasp for the left/only hand "
  print "is commanded by pressing 'g'. For the right hand, press ';'."
  print ""
  print "Press [escape] to quit."

def draw_string(screen, font, x, y, string):
  text = font.render(string, 1, (255,255,255))
  screen.blit(text, (x, y))

if __name__ == '__main__':
  pygame.init()
  screen = pygame.display.set_mode((640, 480))
  pygame.display.set_caption('Minimalist Dual-Hand Keyboard Teleop')
  font = pygame.font.Font(None, 18)
  draw_string(screen, font, 10, 10, "This window must have focus")
  draw_string(screen, font, 10, 30, "Press [escape] to quit.")
  pygame.display.flip()
  print_usage()
  rospy.init_node("simple_grasp_keyboard_teleop")
  g_left_grasp_pub = rospy.Publisher("left_hand/simple_grasp", SimpleGrasp)
  g_right_grasp_pub = rospy.Publisher("right_hand/simple_grasp", SimpleGrasp)
  rate = rospy.Rate(100.0) # todo: something smarter
  done = False
  while not done and not rospy.is_shutdown():
    for event in pygame.event.get():
      if (event.type == KEYDOWN):
        if event.key == K_ESCAPE:
          done = True
        elif event.key < 128:
          keypress(chr(event.key))
    rate.sleep()
