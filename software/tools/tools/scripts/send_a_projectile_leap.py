#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from drc.recovery_t import recovery_t
import time

if len(sys.argv)>1:
  msg = recovery_t() 
  msg.mode = int(sys.argv[1])
  lc = lcm.LCM()
  lc.publish("RECOVERY_CMD", msg.encode())
  print "One small leap for a robot..."
else:
  print 'No mode specified!'
  times= 1.0




