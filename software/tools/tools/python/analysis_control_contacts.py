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

from drake.lcmt_qp_controller_input import lcmt_qp_controller_input
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

class State:
  def __init__(self):
    self.l_foot_sup = None
    self.r_foot_sup = None

def on_qci(channel, data):
  global state
  state.l_foot_sup = None
  state.r_foot_sup = None

  m = lcmt_qp_controller_input.decode(data)
  for sup in m.support_data:
    if (sup.body_id == 25):
      state.l_foot_sup = sup
    if (sup.body_id == 12):
      state.r_foot_sup = sup

  n_contacts_l = 0
  if (state.l_foot_sup is not None):
    n_contacts_l = state.l_foot_sup.num_contact_pts
  n_contacts_r = 0
  if (state.r_foot_sup is not None):
    n_contacts_r = state.r_foot_sup.num_contact_pts

  print str(n_contacts_l), " | ", str(n_contacts_r)


####################################################################
lc = lcm.LCM()
print "started"
state = State()
sub1 = lc.subscribe("QP_CONTROLLER_INPUT", on_qci)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)



