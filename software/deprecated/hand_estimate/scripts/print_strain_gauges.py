#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")
from drc.atlas_strain_gauges_t import atlas_strain_gauges_t

def on_wsg(channel, data):
  m = atlas_strain_gauges_t.decode(data)  
  s = str(m.utime)
  for x in m.strain_gauges:
    s += "," + ('%.7f' % x)
  print s
#################################################################################

lc = lcm.LCM()
sub0 = lc.subscribe("WRIST_STRAIN_GAUGES", on_wsg) 
while True:
  lc.handle()
lc.unsubscribe(sub0)

