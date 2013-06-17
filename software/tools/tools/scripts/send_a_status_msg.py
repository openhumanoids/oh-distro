#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from drc.system_status_t import system_status_t
import time
#print 'Argument List:', str(sys.argv)
if len(sys.argv)>1:
  system = int(sys.argv[1])
else:
  print 'No mode specified!'
  times= 400.0

if len(sys.argv)>2:
  message = int(sys.argv[2])
else:
  message = "calm down scott. everything will be all right"


msg = system_status_t()
msg.utime = 0
msg.system = system
msg.importance = 0
msg.frequency = 0
msg.value = message

lc = lcm.LCM()
lc.publish("SYSTEM_STATUS", msg.encode())
  

