#!/usr/bin/python

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

from drc.atlas_state_t import atlas_state_t
from drc.robot_state_t import robot_state_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

# ely = 18

class State:
  def __init__(self):
    self.pot_angle = None
    self.pot_utime = None
    self.out_angle = None
    self.out_utime = None

def on_atlas_state(channel, data):
  global state
  m = atlas_state_t.decode(data)
  state.pot_angle = m.joint_position[18]
  state.pot_utime = m.utime

def on_robot_state(channel, data):
  global state
  m = robot_state_t.decode(data)
  state.out_angle = m.joint_position[18]
  state.out_utime = m.utime

  if (state.pot_angle is not None):
    print state.out_utime, state.out_angle, state.pot_angle


####################################################################
lc = lcm.LCM()
print "started"

state = State()

sub1 = lc.subscribe("ATLAS_STATE", on_atlas_state)
sub1 = lc.subscribe("EST_ROBOT_STATE", on_robot_state)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



