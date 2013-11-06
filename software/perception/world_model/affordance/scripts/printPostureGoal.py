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
from drc.joint_angles_t import joint_angles_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

def on_posture_goal(channel, data):
  m = joint_angles_t.decode(data)
  print "Joint Position",str(len(m.joint_position))
  strout = ""
  for x in m.joint_position:
    strout += str(x) + ", "
  print strout[:-2] # skip last ", "
  
  print "Joint Name",str(len(m.joint_name))
  strout = ""
  for x in m.joint_name:
    strout += str(x) + ", "
  print strout[:-2] # skip last ", "

#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("POSTURE_GOAL", on_posture_goal)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

