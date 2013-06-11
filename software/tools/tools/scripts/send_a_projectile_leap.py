#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from drc.utime_t import utime_t
import time

msg = utime_t()
msg.utime = 12345

lc = lcm.LCM()
lc.publish("PROJECTILE_LEAP", msg.encode())
print "One small leap for a robot..."
