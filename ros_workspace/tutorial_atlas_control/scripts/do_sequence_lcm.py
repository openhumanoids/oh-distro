#!/usr/bin/python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
import csv_parse
import carry_out_sequence 

import os,sys
import lcm
import time
from lcm import LCM
import math
import csv_parse
import carry_out_sequence 


home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.recovery_t import recovery_t
from drc.system_status_t import system_status_t
from drc.frequency_t import frequency_t

filenames = ['projectial_ingress','faceup_to_facedown_wriggle','flat_out', 
             'knee_rise_set','knee_rise','knee_rise_finish']

real_time_percent = 0.8

def on_frequency_lcm(channel, data):
  global real_time_percent
  m = frequency_t.decode(data)  
  real_time_percent = int( m.real_time_percent )
  print "got real_time_percent: %d" % (real_time_percent) 

def on_recovery(channel, data):
  global real_time_percent
  print "bbb %d" %( real_time_percent)
  
  msg = system_status_t() 
  msg.system=3
  msg.importance=0
  msg.frequency=0

  m = recovery_t.decode(data)  
  mode = int(m.mode)
  filename = filenames[mode] 
  print "Doing [%d: %s]" %(mode, filename)
  msg.value = "start: " + filename
  lc.publish("SYSTEM_STATUS", msg.encode())
  full_filename = home_dir + "/drc/ros_workspace/tutorial_atlas_control/scripts/" + filename + ".csv"
  #print full_filename
  carry_out_sequence.carry_out_sequence(full_filename, 0, real_time_percent)
  print "done with sequence\n\n"
  msg.value = "finish: " + filename
  lc.publish("SYSTEM_STATUS", msg.encode())

  


####################################################################
lc = lcm.LCM()
print "started projectile process"
sub1 = lc.subscribe("RECOVERY_CMD", on_recovery)
sub2 = lc.subscribe("FREQUENCY_LCM", on_frequency_lcm)
while True:
  lc.handle()

lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
