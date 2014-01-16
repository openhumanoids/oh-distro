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

  ins.device_time = m.raw_imu[0].utime
  ins.accel = m.raw_imu[0].linear_acceleration

  dt = (m.raw_imu[0].utime - m.raw_imu[1].utime)*1E-6
  ins.gyro[0] = (m.raw_imu[0].delta_rotation[0] - m.raw_imu[1].delta_rotation[0])/dt
  ins.gyro[1] = (m.raw_imu[0].delta_rotation[1] - m.raw_imu[1].delta_rotation[1])/dt
  ins.gyro[2] = (m.raw_imu[0].delta_rotation[2] - m.raw_imu[1].delta_rotation[2])/dt

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

sub1 = lc.subscribe("ATLAS_IMU_BATCH", on_atlas_imu_batch)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



