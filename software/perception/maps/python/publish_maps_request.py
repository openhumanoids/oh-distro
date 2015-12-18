#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from drc.data_request_list_t import data_request_list_t
from drc.data_request_t import data_request_t

def timestamp_now (): return int (time.time () * 1000000)

lc = lcm.LCM()
print "started"


m = data_request_list_t()
m.utime = timestamp_now()
m.num_requests = 1

r = data_request_t()
#r.type = 21 #OCTREE_SCENE
r.type = 22 #OCTREE_WORKSPACE
r.period = 10 # 0 one shot, 10 for 1Hz, 20 for 0.5Hz

m.requests = [r]
lc.publish("DATA_REQUEST",m.encode())


