#!/usr/bin/python
import os,sys
home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
from drc.atlas_command_t import atlas_command_t
import time

msg = atlas_command_t()
# todo: use bot time:
msg.utime = 0
#msg.desired_controller_period_ms = 0
msg.k_effort =""

lc = lcm.LCM()
while (1):
  msg.utime = msg.utime +1
  lc.publish("ATLAS_COMMAND", msg.encode())
  print "ATLAS_COMMAND sent"
  #time.sleep(0.0033)
  time.sleep(0.005)  


