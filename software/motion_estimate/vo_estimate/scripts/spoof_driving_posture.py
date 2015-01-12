#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
import math
import numpy as np

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from multisense.images_t import images_t
from drc.atlas_state_t import atlas_state_t
########################################################################################


def send_stuff(t):
  print "Sending State %d" %(t)
  o = atlas_state_t()
  o.utime = t
  
  o.num_joints = 30
  o.joint_position = [0] * o.num_joints
  o.joint_position[1] = 0.10
  o.joint_position[2] = -0.1
  o.joint_position[3] = -0.1 # neck

  o.joint_position[6] = -1.60
  o.joint_position[7] = 1.55
  o.joint_position[12] = -1.6
  o.joint_position[13] = 1.55

  # larm
  o.joint_position[16] = -np.pi/4
  o.joint_position[18] =  np.pi/2
  o.joint_position[19] =  np.pi/4

  o.joint_position[24] = np.pi/2
  o.joint_position[25] = np.pi/2
  o.joint_position[26] = -np.pi/2
  o.joint_position[28] = -np.pi/5
#  o.joint_position[24] =  np.pi/2
#  o.joint_position[25] =  np.pi/4

  o.joint_velocity = [0] * o.num_joints
  o.joint_effort   = [0] * o.num_joints
  lc.publish("ATLAS_STATE", o.encode())    

def on_camera(channel, data):
  m = images_t.decode(data)
  send_stuff(m.utime)

####################################################################
lc = lcm.LCM()
print "started: spoof ATLAS_STATE"
sub1 = lc.subscribe("CAMERA", on_camera)
while True:
  lc.handle()

lc.unsubscribe(sub)


