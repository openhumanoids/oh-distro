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
from drc_utils import *

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.deprecated_footstep_plan_t import deprecated_footstep_plan_t
from bot_core.pose_t import pose_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


class State:
  def __init__(self):
    self.last_utime =0
    self.got_pose_bdi =False
    self.got_pose_body =False
    self.pose_bdi = BotTrans()
    self.pose_body = BotTrans()

def on_pose_bdi(channel, data):
  m = pose_t.decode(data)
  if (state.got_pose_bdi == False): print "Received",channel
  state.got_pose_bdi = True
  state.pose_bdi = getBotCorePose3dAsBotTrans(m)

  #temp = trans_invert( state.pose_body ) ;
  #q = trans_apply_trans( state.pose_bdi,temp   );    
  #q.print_out()


def on_pose_body(channel, data):
  m = pose_t.decode(data)
  if (state.got_pose_body == False): print "Received",channel
  state.got_pose_body = True
  state.pose_body = getBotCorePose3dAsBotTrans(m)
  
    
def on_deprecated_footstep_plan(channel, data):
  if ( (state.got_pose_bdi == False) or (state.got_pose_body == False) ):
    print "havent got POSE_BDI and POSE_BODY yet"
    return
    
  m = deprecated_footstep_plan_t.decode(data)
  print m.utime,"Republishing", m.num_steps, "steps"
  temp = trans_invert( state.pose_body ) ;
  q = trans_apply_trans( state.pose_bdi,temp   );    
  
  for i in range(0,m.num_steps):
    p = getDrcPosition3dAsBotTrans(m.footstep_goals[i].pos)

    temp = trans_invert( state.pose_body ) ;
    body_to_step = trans_apply_trans( p,temp   );    
    #body_to_step.print_out()

    q2 = trans_apply_trans( body_to_step,state.pose_bdi )
    m.footstep_goals[i].pos = getBotTransAsDrcPosition3d(q2)
  lc.publish("CANDIDATE_BDI_FOOTSTEP_PLAN",m.encode())


####################################################################
lc = lcm.LCM()
print "started"

state = State()

sub1 = lc.subscribe("POSE_BDI", on_pose_bdi)
sub2 = lc.subscribe("POSE_BODY", on_pose_body)
sub3 = lc.subscribe("CANDIDATE_BDI_FOOTSTEP_PLAN_MIT_FRAME", on_deprecated_footstep_plan)

while True:
  lc.handle()
