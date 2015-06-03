#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from multisense.command_t import command_t
import time

def timestamp_now (): return int (time.time () * 1000000)
time.sleep(5)

msg = command_t()
msg.utime = timestamp_now()
msg.rpm = 5
msg.fps = -1
msg.gain = -1
msg.agc = -1

lc = lcm.LCM()
lc.publish("MULTISENSE_COMMAND", msg.encode())
print "Publishing Multisense command to spin at 5rpm"
