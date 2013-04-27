#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.utime_t import utime_t
from drc.twist_timed_t import twist_timed_t
from drc.vector_3d_t import vector_3d_t

from drc.data_request_t import data_request_t
from drc.data_request_list_t import data_request_list_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

def on_utime(channel, data):
  m = utime_t.decode(data)
  t = m.utime
  #print t
  if ( abs(t - 300000) < 5000): # after .3secs send a spin rate
    print "spin the bloody laser already - %d" %(t)
    o = twist_timed_t()
    o.utime = t
    l = vector_3d_t()
    a = vector_3d_t()
    a.x = 0.733038
    o.linear_velocity =a
    o.angular_velocity =a
    lc.publish("SCAN_RATE_CMD", o.encode())    
  if (abs(t - 3000000) < 5000): # after 4secs send the data request
    print "gimmie the data or else - %d" %(t)
    o = data_request_list_t()
    o.utime = t
    r = data_request_t()
    r.type = 1
    r.period = 10
    o.num_requests= 3
    o.requests.append(r)
    r.type =2
    o.requests.append(r)
    r.type =6
    o.requests.append(r)
    lc.publish("DATA_REQUEST", o.encode())    

####################################################################
lc = lcm.LCM()
print "started"

sub1 = lc.subscribe("ROBOT_UTIME", on_utime)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub)
