#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from bot_core.image_t import image_t
import time

msg = image_t()
msg.utime = 12345
msg.size =0
msg.data="dfds"
msg.nmetadata=0
msg.metadata=[]
lc = lcm.LCM()
lc.publish("KINTINUOUS_COMMAND", msg.encode())
print "KINTINUOUS_COMMAND SENT"
  

