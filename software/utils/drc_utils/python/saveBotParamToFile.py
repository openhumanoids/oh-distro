#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from bot_param.update_t import update_t

########################################################################################


def on_param(channel, data):
  m = update_t.decode(data)
  t = m.utime
  print t
  text_file = open("output.cfg", "w")
  text_file.write("%s" % m.params)
  text_file.close()
  print "done writing output.cfg"

####################################################################
lc = lcm.LCM()
print "started"
sub = lc.subscribe("PARAM_UPDATE", on_param)
while True:
  lc.handle()

lc.unsubscribe(sub)
