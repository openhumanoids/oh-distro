#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")


import lcm
from bot_core.pose_t import pose_t

msg = pose_t()
msg.utime = 0
msg.pos = (0, 0, 0)
msg.orientation = (1, 0, 0, 0)

lc = lcm.LCM()
lc.publish("POSE_HEAD", msg.encode())
