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

filenames = ['projectial_ingress_ready', 'projectial_ingress',  #0 1
             'faceup_to_facedown_wriggle', 'facedown_to_faceup',         #2 3
             'flat_out', 'flat_out_slow', # 4 5
             'flat_out_knees_out','faceup_to_facedown',    #67
             'nothing','nothing', #89
             'knee_rise_set','knee_rise', #10 11
             'knee_rise_finish','nothing',   # 12 13
             'nothing', 'nothing', # 14 15
             'nothing', 'nothing', # 16 17
             'nothing', 'nothing', # 18 19
             'jaguar_crawl_set','jaguar_crawl', # 20 21
             'jaguar_crawl_left','jaguar_crawl_left_large', # 22 23
             'jaguar_crawl_right','jaguar_crawl_right_large'] # 24 25


real_time_percent_ma = 50


def on_frequency_lcm(channel, data):
  global real_time_percent, real_time_percent_ma
  m = frequency_t.decode(data)  
  real_time_percent = float( m.real_time_percent )
  real_time_percent_ma = 0.85*real_time_percent_ma + 0.15*real_time_percent
  print "got real_time: %f | moving average: %f" % (real_time_percent, real_time_percent_ma) 

def on_recovery(channel, data):
  m = recovery_t.decode(data)  
  mode = int(m.mode)
  controller = int(m.controller)
  if (controller!=0):
    print "this recovery message is not for me, returning"
    return

  global real_time_percent_ma
  print "rtp ma: %f" %( real_time_percent_ma)

  msg = system_status_t() 
  msg.system=3
  msg.importance=0
  msg.frequency=0

  print "Reading [%d]" %(mode)
  filename = filenames[mode] 
  if (filename == 'nothing'):
    print "ignoring nothing string"
    return
  
  print "Doing [%d: %s]" %(mode, filename)
  msg.value = "start: " + filename
  lc.publish("SYSTEM_STATUS", msg.encode())
  full_filename = home_dir + "/drc/ros_workspace/tutorial_atlas_control/scripts/" + filename + ".csv"
  #print full_filename
  carry_out_sequence.carry_out_sequence(full_filename, 0, real_time_percent_ma)
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
