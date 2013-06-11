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

filenames = ['projectial_ingress','faceup_to_facedown_wriggle','flat_out', 
             'knee_rise_set','knee_rise','knee_rise_finish']

def on_recovery(channel, data):
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
  carry_out_sequence.carry_out_sequence(full_filename, 0)
  print "done with sequence\n\n"
  msg.value = "finish: " + filename
  lc.publish("SYSTEM_STATUS", msg.encode())

  


####################################################################
lc = lcm.LCM()
print "started projectile process"
sub1 = lc.subscribe("RECOVERY_CMD", on_recovery)
while True:
  lc.handle()

lc.unsubscribe(sub)
