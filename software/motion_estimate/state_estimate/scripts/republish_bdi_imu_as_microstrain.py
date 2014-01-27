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

from drc.atlas_raw_imu_batch_t import atlas_raw_imu_batch_t
from microstrain.ins_t import ins_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)



def on_atlas_imu_batch(channel, data):
  m = atlas_raw_imu_batch_t.decode(data)

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

  ins.gyro[0] = (m.raw_imu[0].delta_rotation[0])/dt
  ins.gyro[1] = (m.raw_imu[0].delta_rotation[1])/dt
  ins.gyro[2] = (m.raw_imu[0].delta_rotation[2])/dt

  # initialize mag values as zero - can be used in mav estimator
  # but typically unused
  ins.mag[0] = 0
  ins.mag[1] = 0
  ins.mag[2] = 0
  
  print dt
  if (1==1):
    print "======"
    print "rotation"
    print m.raw_imu[0].utime
    print m.raw_imu[1].utime
    print m.raw_imu[0].delta_rotation
    print "rotation rate"
    print dt
    print ins.gyro

  lc.publish("MICROSTRAIN_INS",ins.encode())

####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("ATLAS_IMU_BATCH", on_atlas_imu_batch)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



