#!/usr/bin/python
# TODO: check at count is entirely mono-tonic and no missing packets

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

from drc.atlas_raw_imu_batch_t import atlas_raw_imu_batch_t
from microstrain.ins_t import ins_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

class State:
  def __init__(self):
    self.last_packet = -1

def publishIMUPacket(msg):
  lc.publish("ATLAS_IMU_PACKET",msg.encode())

def printIMUPacker(msg):
  s = str(msg.utime) + ', ' + str(msg.delta_rotation[0]) + ', ' + str(msg.delta_rotation[1]) + ', ' + str(msg.delta_rotation[2])  \
      + ', ' + str(msg.linear_acceleration[0]) + ', ' + str(msg.linear_acceleration[1]) + ', ' + str(msg.linear_acceleration[2])
  print s

def on_atlas_imu_batch(channel, data):
  global state
  m = atlas_raw_imu_batch_t.decode(data)

  num_new = 0
  verbose = False
  for i in range(m.num_packets-1,-1,-1):
    if (m.raw_imu[i].packet_count > state.last_packet):
      if verbose: print "new ", state.last_packet, i, m.raw_imu[i].packet_count
      state.last_packet = m.raw_imu[i].packet_count
      num_new= num_new+1
      publishIMUPacket(m.raw_imu[i])
      printIMUPacker(m.raw_imu[i])
    else:
      if verbose: print "old ", state.last_packet, i, m.raw_imu[i].packet_count

  if verbose: print num_new , "---"

def a():  
  ins = ins_t()
  ins.utime = m.utime
  
  dt = 0.001
  older_logs = 1
  if (older_logs ==1):
    dt = 0.001
  else:
    dt = (m.raw_imu[0].utime - m.raw_imu[1].utime)*1E-6

  ins.device_time = m.raw_imu[0].utime
  ins.accel[0] = m.raw_imu[0].linear_acceleration[0]
  ins.accel[1] = m.raw_imu[0].linear_acceleration[1]
  ins.accel[2] = m.raw_imu[0].linear_acceleration[2]

  
  ins.gyro[0] = (m.raw_imu[0].delta_rotation[0])/dt
  ins.gyro[1] = (m.raw_imu[0].delta_rotation[1])/dt
  ins.gyro[2] = (m.raw_imu[0].delta_rotation[2])/dt

  print dt
  if (1==0):
    print "======"
    print "old"
    print m.raw_imu[1].utime
    print m.raw_imu[1].delta_rotation
    print "new"
    print m.raw_imu[0].utime
    print m.raw_imu[0].delta_rotation
    print "diff"
    print dt
    print ins.gyro

  lc.publish("MICROSTRAIN_INS",ins.encode())

####################################################################
lc = lcm.LCM()
print "started"

state = State()

sub1 = lc.subscribe("ATLAS_IMU_BATCH", on_atlas_imu_batch)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



