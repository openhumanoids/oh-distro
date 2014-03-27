#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/trials/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/trials/software/build/lib/python2.7/dist-packages")

from trials.utime_t import utime_t
from trials.twist_timed_t import twist_timed_t
from trials.vector_3d_t import vector_3d_t
from trials.data_request_t import data_request_t
from trials.data_request_list_t import data_request_list_t
from trials.sensor_request_t import sensor_request_t
from trials.neck_pitch_t import neck_pitch_t

##
from trials.robot_plan_w_keyframes_t import robot_plan_w_keyframes_t
from trials.robot_plan_t import robot_plan_t
from trials.footstep_plan_t import footstep_plan_t
from trials.robot_state_t import robot_state_t
from trials.subimage_request_t import subimage_request_t
from trials.atlas_status_t import atlas_status_t
from trials.walking_goal_t import walking_goal_t
from trials.affordance_t import affordance_t
from trials.affordance_plus_t import affordance_plus_t
from robotiqhand.command_t import command_t
from trials.drill_control_t import drill_control_t
from trials.bandwidth_stats_t import bandwidth_stats_t

########################################################################################

class State:
  def __init__(self):
    self.last_utime = 0
    self.behavior = -1

def on_ers(channel, data):
  m = robot_state_t.decode(data)
  state.last_utime = m.utime

def on_candrp(channel, data):
  m = robot_plan_w_keyframes_t.decode(data)
  print state.last_utime,channel # utime inside main message not pupulated

def on_crp(channel, data):
  m = robot_plan_t.decode(data)
  print m.utime,channel

def on_cfp(channel, data):
  m = footstep_plan_t.decode(data)
  print m.utime,channel

def on_utime(channel, data):
  m = utime_t.decode(data)
  print m.utime,channel

def on_subimage_request(channel, data):
  m = subimage_request_t.decode(data)
  print state.last_utime,channel # no utime

def on_walking_goal(channel, data):
  m = walking_goal_t.decode(data)
  print m.utime,channel

def on_affordance_fit(channel, data):
  m = affordance_plus_t.decode(data)
  print state.last_utime,channel # utime inside main message not pupulated

def on_affordance_track(channel, data):
  m = affordance_t.decode(data)
  print state.last_utime,channel # utime inside main message not pupulated

def on_atlas_status(channel, data):
  m = atlas_status_t.decode(data)
  if (m.behavior != state.behavior):
    state.behavior = m.behavior
    beh = "BEHAVIOR_CHANGE_"+str(state.behavior)
    print m.utime,beh

def on_robotiqhand_command(channel, data):
  m = command_t.decode(data)
  print m.utime,channel

def on_drill_control(channel, data):
  m = drill_control_t.decode(data)
  print m.utime,channel


##### ##### ##### ##### ##### #####
def on_crp_duration(channel, data):
  m = robot_plan_t.decode(data)
  #print m.num_states
  print m.utime,m.plan[m.num_states-1].utime

##### ##### ##### ##### ##### #####
def on_bandwidth_stats(channel, data):
  m = bandwidth_stats_t.decode(data)

  #mode=1
  # print channel names:
  #if (mode==0): # channels
  #  strout_channels = str(m.sent_channels[0])
  #  for i in range(1, m.num_sent_channels):
  #    strout_channels += str(" " + str(m.sent_channels[i]))
  #  print m.utime, strout_channels
  #elif (mode==2):
  #  strout = str(m.received_channels[0])
  #  for i in range(1, m.num_received_channels):
  #    strout += str(" " + str(m.received_channels[i]))
  #  print m.utime, strout

  if (state.mode==2):
    strout = str(m.sent_bytes[0])
    for i in range(1, m.num_sent_channels):
      strout += str(" " + str(m.sent_bytes[i]))
    print m.utime, strout

  elif (state.mode==3):
    strout = str(m.received_bytes[0])
    for i in range(1, m.num_received_channels):
      strout += str(" " + str(m.received_bytes[i]))
    print m.utime, strout

####################################################################
 
state = State()

# Get the arguments list 
nargs = len(sys.argv)
if (nargs<2):
  state.mode =0
else:
  state.mode = int(sys.argv[1])

#print mode




#python log_analysis.py 0 > ~/Desktop/drill/driving/important_messages.txt
# just manip plans and their duration:
#python log_analysis.py 1 > ~/Desktop/drill/driving/task_anatomy_plans.txt
#python log_analysis.py 2 > ~/Desktop/drill/driving/base_sent_data.txt
#python log_analysis.py 3 > ~/Desktop/drill/driving/base_received_data.txt

#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/drill/lcmlog-2013-12-20-12-57-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/debris/lcmlog-2013-12-20-07-56-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/walking_1/lcmlog-2013-12-21-10-34-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/walking_2/lcmlog-2013-12-21-15-11-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/hose/lcmlog-2013-12-20-09-53-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/climbing/lcmlog-2013-12-21-11-49-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/valves/lcmlog-2013-12-20-11-05-base?speed=0")
#lc = lcm.LCM("file:///home/mfallon/Desktop/drill/doors/lcmlog-2013-12-20-14-45-base?speed=0")

lc = lcm.LCM("file:///home/mfallon/Desktop/drill/driving/lcmlog-2013-12-21-08-10-base?speed=0")



if (state.mode==0): # typical
  lc.subscribe("EST_ROBOT_STATE", on_ers)
  lc.subscribe("COMMITTED_ROBOT_PLAN", on_crp)
  lc.subscribe("CANDIDATE_MANIP_PLAN", on_candrp)
  lc.subscribe("COMMITTED_FOOTSTEP_PLAN", on_cfp)
  lc.subscribe("SUBIMAGE_REQUEST", on_subimage_request)
  lc.subscribe("ROBOTIQ_LEFT_COMMAND", on_robotiqhand_command)
  lc.subscribe("ATLAS_STATUS", on_atlas_status)
  lc.subscribe("WALKING_GOAL", on_walking_goal)
  lc.subscribe("AFFORDANCE_FIT", on_affordance_fit)
  #lc.subscribe("AFFORDANCE_TRACK", on_affordance_track)
  lc.subscribe("DRILL_CONTROL", on_drill_control)
elif (state.mode==1): # commited robot plan duration 
  lc.subscribe("COMMITTED_ROBOT_PLAN", on_crp_duration)
elif (state.mode==2): 
  lc.subscribe("BASE_BW_STATS", on_bandwidth_stats)
elif (state.mode==3): # receieved data
  lc.subscribe("BASE_BW_STATS", on_bandwidth_stats)
else:
  print "mode not understood"
  sys.exit()

while True:
  lc.handle()


