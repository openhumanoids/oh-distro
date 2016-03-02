#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
#sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
#sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from bot_core.pose_t import pose_t
import time

def timestamp_now (): return int (time.time () * 1000000)
time.sleep(1)

msg = pose_t()
msg.utime = timestamp_now()
msg.pos = [1, 2,99999]

lc = lcm.LCM()
lc.publish("IHMC_FOOTSTEP_PARAMS", msg.encode())
print "Sending Footstep params to IHMC controller"
