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
from drc.planner_request_t import planner_request_t

import ddapp.thirdparty.numpyjsoncoder as nje
import json

def ConstraintDecoder(dct):
    return nje.NumpyDecoder(dct)

def decodeConstraints(dataStream):
    return json.loads(dataStream, object_hook=ConstraintDecoder)

def on_msg(channel, data):
  m = planner_request_t.decode(data)

  print m.utime
  p = decodeConstraints(m.constraints)
  print "raw string:"
  print m.constraints
  print type(p)
  print p[0]
  print "----------- Some of the Elements of Constraint [0] --------------"

  print p[0]['joints']
  print p[0]['enabled']
  print type(p[0]['enabled'])
  print p[0]['jointsLowerBound']

  
lc = lcm.LCM()
print "started"
sub1 = lc.subscribe("PLANNER_REQUEST", on_msg)
while True:
  lc.handle()

lc.unsubscribe(sub1)
