#!/usr/bin/python
# Script to turn on 
import os,sys
home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
from drc.atlas_power_hands_t import atlas_power_hands_t
import time
#print 'Argument List:', str(sys.argv)
if len(sys.argv)>1:
  mode = sys.argv[1]

  msg = atlas_power_hands_t()
  msg.utime = 0
  if (mode == "both"):
    msg.power_left = 1
    msg.power_right = 1
  elif (mode=="left"):
    msg.power_left = 1
    msg.power_right = 0
  elif (mode=="right"):
    msg.power_left = 0
    msg.power_right = 1
  elif (mode=="none"):
    msg.power_left = 0
    msg.power_right = 0
  else:
    print 'Mode not recognised: ',mode
    sys.exit(0)

  print 'Sending Power Mode: ',mode
  lc = lcm.LCM()
  lc.publish("ATLAS_POWER_HANDS", msg.encode())
  

else:
  print 'No mode specified, usage:'
  print 'drc-power-hands [both|left|right|none]'



