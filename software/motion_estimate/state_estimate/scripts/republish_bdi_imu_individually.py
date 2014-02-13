#!/usr/bin/python
# TODO: check at count is entirely mono-tonic and no missing packets

import os,sys
import lcm
import time
from lcm import LCM
from math import *
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
    self.last_packet_utime =0
    self.publishAt1000Hz=0
    self.printPacketAt1000Hz=0
    self.publishMicrostrainPacketAt1000Hz=1
    self.publishMicrostrainPacketAt330Hz=0

    # in addition, manually convert MS gyro to body frame (hacky but works)
    self.gyroToBody =1

def publishIMUPacket(msg):
  lc.publish("ATLAS_IMU_PACKET",msg.encode())

def printIMUPacket(msg):
  s = str(msg.utime) + ', ' + str(msg.delta_rotation[0]) + ', ' + str(msg.delta_rotation[1]) + ', ' + str(msg.delta_rotation[2])  \
      + ', ' + str(msg.linear_acceleration[0]) + ', ' + str(msg.linear_acceleration[1]) + ', ' + str(msg.linear_acceleration[2])
  print s


def publishMicrostrainPacketAt330Hz(m):
  global state
  ins = ins_t()
  ins.utime = m.utime
  
  dt = 0.001
  older_logs = 0
  if (older_logs ==1):
    dt = 0.001
  else:
    dt = (m.raw_imu[0].utime - m.raw_imu[1].utime)*1E-6

  ins.device_time = m.raw_imu[0].utime
  ins.accel[0] = m.raw_imu[0].linear_acceleration[0]
  ins.accel[1] = m.raw_imu[0].linear_acceleration[1]
  ins.accel[2] = m.raw_imu[0].linear_acceleration[2]

  x=(m.raw_imu[0].delta_rotation[0])/dt
  y=(m.raw_imu[0].delta_rotation[1])/dt
  z=(m.raw_imu[0].delta_rotation[2])/dt
  ins.gyro[0] = x
  ins.gyro[1] = y
  ins.gyro[2] = z
  if (state.gyroToBody):
    rads = pi*45.0/180.0
    ins.gyro[0] = -sin(rads)*x + cos(rads)*y
    ins.gyro[1] = cos(rads)*x + sin(rads)*y
    ins.gyro[2] = -z

  # initialize mag values as zero - can be used in mav estimator but typically unused
  ins.mag[0] = 0
  ins.mag[1] = 0
  ins.mag[2] = 0
  
  if (1==0):
    print dt
    print "======"
    print "rotation"
    print m.raw_imu[0].utime
    print m.raw_imu[1].utime
    print m.raw_imu[1].delta_rotation
    print "rotation rate"
    print dt
    print ins.gyro

  lc.publish("MICROSTRAIN_INS",ins.encode())


def publishMicrostrainPacketAt1000Hz(m, packet_utime, i):
  global state
  ins = ins_t()
  # simple algorithm to
  ins.utime = packet_utime - i*1000
  #print i
  #print ins.utime
  
  dt = 0.001
  older_logs = 0
  if (older_logs ==1):
    dt = 0.001
  else:
    dt = (m.utime - state.last_packet_utime)*1E-6

  ins.device_time = m.utime
  ins.accel[0] = m.linear_acceleration[0]
  ins.accel[1] = m.linear_acceleration[1]
  ins.accel[2] = m.linear_acceleration[2]

  x=(m.delta_rotation[0])/dt
  y=(m.delta_rotation[1])/dt
  z=(m.delta_rotation[2])/dt
  ins.gyro[0] = x
  ins.gyro[1] = y
  ins.gyro[2] = z
  if (state.gyroToBody):
    rads = pi*45.0/180.0
    ins.gyro[0] = -sin(rads)*x + cos(rads)*y
    ins.gyro[1] = cos(rads)*x + sin(rads)*y
    ins.gyro[2] = -z

  # initialize mag values as zero - can be used in mav estimator but typically unused
  ins.mag[0] = 0
  ins.mag[1] = 0
  ins.mag[2] = 0
  
  if (1==0):
    print dt
    print "======"
    print "rotation"
    print m.raw_imu[0].utime
    print m.raw_imu[1].utime
    print m.raw_imu[1].delta_rotation
    print "rotation rate"
    print dt
    print ins.gyro

  lc.publish("MICROSTRAIN_INS",ins.encode())


def on_atlas_imu_batch(channel, data):
  global state
  m = atlas_raw_imu_batch_t.decode(data)

  num_new = 0
  verbose = False
  counter_for_microstrain=0
  for i in range(m.num_packets-1,-1,-1):
    if (m.raw_imu[i].packet_count > state.last_packet):
      if verbose: print "new ", state.last_packet, i, m.raw_imu[i].packet_count
      state.last_packet = m.raw_imu[i].packet_count
      num_new= num_new+1
      if (state.publishAt1000Hz):
        publishIMUPacket(m.raw_imu[i])
      if (state.printPacketAt1000Hz):
        printIMUPacket(m.raw_imu[i])
      if (state.publishMicrostrainPacketAt1000Hz):
        publishMicrostrainPacketAt1000Hz(m.raw_imu[i], m.utime, i)
      state.last_packet_utime = m.raw_imu[i].utime
    else:
      if verbose: print "old ", state.last_packet, i, m.raw_imu[i].packet_count


  if (state.publishMicrostrainPacketAt330Hz):
    publishMicrostrainPacketAt330Hz(m)

  if verbose: print num_new , "---"


####################################################################
lc = lcm.LCM()
print "started"

state = State()

sub1 = lc.subscribe("ATLAS_IMU_BATCH", on_atlas_imu_batch)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



