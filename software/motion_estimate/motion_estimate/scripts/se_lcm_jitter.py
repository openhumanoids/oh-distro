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
from drc.robot_state_t import robot_state_t
from drc.atlas_raw_imu_batch_t import atlas_raw_imu_batch_t
from drc.atlas_state_t import atlas_state_t
########################################################################################

def on_atlas_state(channel, data):
  m = atlas_state_t.decode(data)
  print "1, ",m.utime
 
def on_imu_batch(channel, data):
  m = atlas_raw_imu_batch_t.decode(data)
  print "2, ",m.utime

def on_est_robot_state(channel, data):
  m = robot_state_t.decode(data)
  print "3, ",m.utime
  
#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("EST_ROBOT_STATE", on_est_robot_state)
lc.subscribe("ATLAS_IMU_BATCH", on_imu_batch)
lc.subscribe("ATLAS_STATE", on_atlas_state)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

