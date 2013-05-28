#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from bot_core.pose_t import pose_t
import time
#print 'Argument List:', str(sys.argv)
if len(sys.argv)>1:
  times = float(sys.argv[1])
else:
  print 'No mode specified!'
  times= 400.0

msg = pose_t()
msg.utime = 12345
msg.pos = ( times, 0.0, 0.0)
msg.orientation = (1.0, 0.0, 0.0, 0.0)

lc = lcm.LCM()
while (1):
  lc.publish("POSE_SET", msg.encode())
  print "Commanding simulator to set hose pose to mode %f" % times
  print "[0=on hand 1= near hand 2= at hand 3 = at pipe 4 = in pipe]"
  time.sleep(0.001)
  

