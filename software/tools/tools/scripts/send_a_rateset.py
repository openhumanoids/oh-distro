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
  times= 1.0

#times = 1.0/20.0
if (times > 1.0):
  print "ignoring fraction quicker than 1 - can't do miracles"
  times=1.0
elif (times <= 0):
  print "don't be an idiot"
  times=1.0

val = (1/ float(times) )-1

msg = pose_t()
msg.utime = 12345
msg.pos = ( val, 0, 0)
msg.orientation = (1, 0, 0, 0)

lc = lcm.LCM()
while (1):
  lc.publish("GAZEBO_RATESET", msg.encode())
  print "Commanding simulator to run at %f times real time (tops)" % times
  time.sleep(0.25)
  

