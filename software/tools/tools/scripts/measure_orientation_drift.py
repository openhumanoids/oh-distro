#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

from threading import Thread
import threading

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from drc.robot_state_t import robot_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


class State:
  def __init__(self):
    self.prev_utime = 0
    self.prev_rpy = [0,0,0]
    self.prev_xyz = [0,0,0]

def quat_to_euler(q) :
  
  roll_a = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  roll_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  roll = math.atan2 (roll_a, roll_b);

  pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  pitch = math.asin (pitch_sin);

  yaw_a = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  yaw_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);  
  yaw = math.atan2 (yaw_a, yaw_b);
  return [roll,pitch,yaw]

def doPrint():
  print "#utime, utime_delta, velocity, yaw_delta"

def on_ers(channel, data):
  m = robot_state_t.decode(data)

  delta_time = (m.utime - s.prev_utime)/1E6
  if (delta_time < 1):
    return

  rpy = quat_to_euler([m.pose.rotation.w, m.pose.rotation.x, m.pose.rotation.y, m.pose.rotation.z] )
  xyz = [m.pose.translation.x, m.pose.translation.y, m.pose.translation.z]
  measure_drift(delta_time, m.utime, xyz, rpy)

def on_pose(channel, data):
  m = pose_t.decode(data)

  delta_time = (m.utime - s.prev_utime)/1E6
  if (delta_time < 1):
    return

  rpy = quat_to_euler(m.orientation )
  xyz = m.pos
  measure_drift(delta_time, m.utime, xyz, rpy)

def measure_drift(delta_time,utime, xyz, rpy):

  delta_xyz=[0,0,0]
  velocity = math.sqrt(pow(xyz[0]-s.prev_xyz[0],2) + pow(xyz[1]-s.prev_xyz[1],2) + pow(xyz[2]-s.prev_xyz[2],2) )/delta_time

  delta_rpy=[0,0,0]
  delta_rpy[0] = (rpy[0] - s.prev_rpy[0])
  delta_rpy[1] = (rpy[1] - s.prev_rpy[1])
  delta_rpy[2] = (rpy[2] - s.prev_rpy[2])

  if (delta_time < 100000): # (to avoid capturing spikes)
    print utime , ", ", delta_time , ", " , velocity  , ", ", rpy[2]*180/math.pi  , ", ", delta_rpy[2]*180/math.pi # drift rate per second
    sys.stdout.flush()

  s.prev_utime = utime
  s.prev_rpy = rpy
  s.prev_xyz = xyz

####################################################################
lc = lcm.LCM()
#print "started"

s = State()

#either ERS and robot_state_t or a pose channel and pose_t

channel = "EST_ROBOT_STATE"
mode = 0

if (len(sys.argv)	 >= 2):
  channel = sys.argv[1]
  mode = 1
  #print channel, " as a pose_t"
#else:
#  print channel, " as a robot_state_t"  





if (mode == 0):
  sub1 = lc.subscribe(channel, on_ers)
else:
  sub1 = lc.subscribe(channel, on_pose)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)



