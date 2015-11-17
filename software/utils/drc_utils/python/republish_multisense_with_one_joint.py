#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from pronto.multisense_state_t import multisense_state_t

def on_msg(channel, data):
  m = multisense_state_t.decode(data)
  m.num_joints = 1
  lc.publish("MULTISENSE_STATE",m.encode())
  print m.utime

lc = lcm.LCM()
print "started"
sub1 = lc.subscribe("MULTISENSE_STATE_AS_LOGGED", on_msg)
while True:
  lc.handle()

lc.unsubscribe(sub1)



