#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from bot_core.image_t import image_t
from kinect.frame_msg_t import frame_msg_t
from drc.robot_state_t import robot_state_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

# mode 0 - visual
# mode 1 - for matlab
mode = 0
global mode

def on_pose(channel, data):

  m = pose_t.decode(data)
  if (mode is 0):
    print m.utime, " bdi"
  else:
    print m.utime,",0,0"

def on_image(channel, data):
  m = frame_msg_t.decode(data)
  if (mode is 0):
    print "                        ", m.timestamp, " kin"
  else:
    print "0,", m.timestamp, ",0"

def on_robot_state(channel, data):
  m = robot_state_t.decode(data)
  if (mode is 0):
    print "                                                  ",m.utime, " ERS"
  else:
    print "0,0,",m.utime

#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("POSE_BODY_ALT", on_pose)
lc.subscribe("KINECT_FRAME", on_image)
lc.subscribe("EST_ROBOT_STATE", on_robot_state)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

