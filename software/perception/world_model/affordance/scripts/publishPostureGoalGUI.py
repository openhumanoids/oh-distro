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


import sys, os
import pygame, time
from pygame.locals import *
import lcm
from lcm import LCM

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from drc.joint_angles_t import joint_angles_t

def flipArms(name,position):
  #print name
  #print position
  # nb: l back rotation and r back rotation are a simple sign change
  # "back_bkx", "back_bky" shouldn't be used as non-zeros
  lname = ["l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx","back_bkz","back_bkx", "back_bky"]
  rname = ["r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx","back_bkz","back_bkx", "back_bky"]
  flip  = [1          , -1         , 1          , -1         , 1          , -1         ,-1        ,1         ,1          ]
  
  for i in range(len(name)):
    #print i
    j = rname.index(name[i])
    #print lname[j]
    name[i] = lname[j]
    position[i] = flip[j]*position[i]

  #print "after"
  #print name
  #print position
  return [name,position]


def timestamp_now (): return int (time.time () * 1000000)

def getCradle():
  position=[-2.1563617329, 0.100071714551, 0.0580735206604, -0.943417046462, 2.93858361244, 2.2]
  name = ["r_arm_elx", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy", "r_arm_ely"]
  return [position,name]

def getHang():
  position=[-1.03245399544, 1.79775392296, -1.17321766434, 0.101622587872, -1.228967243, 0.0]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getFarHang():
  position=[-0.281868335103, 1.79775392296, -1.17321766434, 0.351412541103, -1.0226471182, 0.0]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getShooter():
  position=[-2.35619, 1.35674083233, 0.0874569192529, 0.270734280348, 0.785398, 0.286707222462]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getShooterBack():
  position=[0,0,0,-2.35619, 1.35674083233, 0.0874569192529, 0.270734280348, 0.785398, 0.286707222462]
  name=["back_bkx", "back_bky", "back_bkz","r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPullDown():
  # right one finger close to chest
  position=[-1.66505864662, 2.12239654256, 0.423257291317, 0.664812326431, -0.596253812313, 3.14159]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]

  # right two fingers close to chest
  #position=[-1.40425169468, 1.94562590122, -0.584710803126, 0.789787143469, -0.927295863628, 0.546333581209]
  #name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getHandDown():
  position=[-1.01134300232, 1.96587055922, 0.0517824155395, 1.38765757637, 0.239791974425, 0.009811]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]


def getCrane():
  position=[-1.61829870194, 2.56434223056, -1.10402787743, -0.796537280083, -0.773799479008, 0.727126836777]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getTurnBack():
  position=[-0.663225]
  name=["back_bkz"]
  return [position,name]

def getZeroBack():
  position=[0.0, 0.0, 0.0]
  name=["back_bkx", "back_bky", "back_bkz"]
  return [position,name]

def getRetract():
  position=[-0.663225, -0.0359195768833, 1.35790967941, 0.734279990196, 0.168820291758, 0.767885386944, 0.0597184896469]
  name=["back_bkz", "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getHoseMate():
  position=[-0.899628996849, 3.14159, -0.848963201046, -0.0953866541386, -0.739547431469, 0.199334427714]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position,name]

def getPoolQueue():
  position=[-0.6583873,-0.493037, 3.08459508499, 1.1543736, 0.276259120, 0.785398000, 2.769477100]
  name=["back_bkz", "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position, name]

def getHoseMateSuckerPunch():
  position=[0.100242577493, -0.0315173082054, 0.322634994984, -1.70546746254, 1.28341770172, -0.111346885562, 1.54854994373, 0.487143397331, 0.0]
  name=["back_bkx", "back_bky", "back_bkz", "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position, name]


def getHosePsycho():
  position=[-1.74607777596, 0.65927785635, 0.224538683891, 0.808890104294, -0.590336024761, 1.1200705735]
  name=["r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy"]
  return [position, name]

def keypress(input_char):
  global posture_side
  print "handling keypress: %s" % input_char

  lc = lcm.LCM()
  msg = joint_angles_t()
  msg.utime = timestamp_now ()

  if (input_char == "h"):
    if (posture_side == "l"):
      posture_side = 'r'
    elif (posture_side == "r"):
      posture_side = 'l'
    return

  if (input_char == "1"):
    [msg.joint_position, msg.joint_name] = getCradle()
  elif (input_char == "2"):
    [msg.joint_position, msg.joint_name] = getHang()
  elif (input_char == "3"):
    [msg.joint_position, msg.joint_name] = getFarHang()
  elif (input_char == "4"):
    [msg.joint_position, msg.joint_name] = getPullDown()
  elif (input_char == "5"):
    [msg.joint_position, msg.joint_name] = getHoseMate()
  elif (input_char == "6"):
    [msg.joint_position, msg.joint_name] = getHoseMateSuckerPunch()
  elif (input_char == "7"):
    [msg.joint_position, msg.joint_name] = getHosePsycho()
  elif (input_char == "q"):
    [msg.joint_position, msg.joint_name] = getHandDown()
  elif (input_char == "w"):
    [msg.joint_position, msg.joint_name] = getZeroBack()
  elif (input_char == "a"):
    [msg.joint_position, msg.joint_name] = getShooter()
  elif (input_char == "s"):
    [msg.joint_position, msg.joint_name] = getShooterBack()
  elif (input_char == "d"):
    [msg.joint_position, msg.joint_name] = getCrane()
  elif (input_char == "z"):
    [msg.joint_position, msg.joint_name] = getPoolQueue()
  elif (input_char == "x"):
    [msg.joint_position, msg.joint_name] = getTurnBack()
  elif (input_char == "c"):
    [msg.joint_position, msg.joint_name] = getRetract()

  if (posture_side == "l"):
    [msg.joint_name,msg.joint_position] = flipArms(msg.joint_name,msg.joint_position)


  draw_string(screen, font, 300, 10, posture_side)
  msg.num_joints = len(msg.joint_name)
  lc.publish("POSTURE_GOAL", msg.encode())

def draw_string(screen, font, x, y, string):
  text = font.render(string, 1, (255,255,255))
  screen.blit(text, (x, y))

if __name__ == '__main__':
  pygame.init()
  screen = pygame.display.set_mode((800, 640))
  pygame.display.set_caption('Minimal Dual-Hand Keyboard Teleop')
  font = pygame.font.SysFont("monospace", 20)
  draw_string(screen, font, 10, 30,  "1 cradle      - close to robot, hand upside down")
  draw_string(screen, font, 10, 50,  "2 hang        - hand just about cradle. thumb in")
  draw_string(screen, font, 10, 70,  "3 farhang     - above, further away")
  draw_string(screen, font, 10, 90,  "4 pulldown    - lower config below the current")
  draw_string(screen, font, 10, 110, "5 hosemate    - config mate")
  draw_string(screen, font, 10, 130, "6 hosemate    - sucker punch")
  draw_string(screen, font, 10, 150, "7 hose pyscho - raise arm up after pulldown")

  draw_string(screen, font, 10, 190, "q handdown    - handdown")
  draw_string(screen, font, 10, 210, "w zero back   - zero the back joint")

  draw_string(screen, font, 10, 250,  "a shooter     - hand up and finger away")
  draw_string(screen, font, 10, 270, "s shooterback - hand up and finger away, back straight")
  draw_string(screen, font, 10, 290, "d crane       - hand pointing down from in front")

  draw_string(screen, font, 10, 330, "z poolqueue   - draw hand back with face down")
  draw_string(screen, font, 10, 350, "x turn back   - turn the back z joint")
  draw_string(screen, font, 10, 370, "c retract     - turn back and retract hand")



  draw_string(screen, font, 10, 410, "h switch hands")
  posture_side = 'l'

  pygame.display.flip()
  done = False
  while not done:
    for event in pygame.event.get():
      if (event.type == KEYDOWN):
        if event.key == K_ESCAPE:
          done = True
        elif event.key < 128:
          keypress(chr(event.key))
    time.sleep(0.1)
