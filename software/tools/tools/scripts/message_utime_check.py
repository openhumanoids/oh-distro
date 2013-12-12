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

from drc.robot_state_t import robot_state_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


class State:
  def __init__(self):
    self.prev_utime = 0


def on_ers(channel, data):
  m = robot_state_t.decode(data)

  delta = m.utime - s.prev_utime
  print m.utime, "| delta:", delta/1E6
  s.prev_utime = m.utime

####################################################################
lc = lcm.LCM()
print "started"

s = State()


sub1 = lc.subscribe("EST_ROBOT_STATE", on_ers)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



