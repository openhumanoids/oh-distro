#!/usr/bin/python
# system to show the status on the viewer screen
# usage: python renderer_status.py
# mfallon. august 2011

import sys
import lcm
# for bottime:
import time
import os

import random


home_dir=  os.environ['HOME']
sys.path.append("/home/mfallon/marine/projects/collabmap/build/lib/python2.6/site-packages")

from vs.system_status_t import system_status_t

def timestamp_now (): return int (time.time () * 1000000)
########################################################################################################
lc = lcm.LCM()

types_list= ['VODOM', 'SLAM', 'LOOP']


for i in range(1,100):
  stat = system_status_t()
  stat.utime = timestamp_now()
  stat.name = types_list[random.randrange(0,3)]
  stat.level = random.randrange(0,6)
  stat.value = "Example message %d" % random.randrange(0,100)
  lc.publish("SYSTEM_STATUS", stat.encode())
  time.sleep(0.1)
  print "%d %s" % (i,stat.name)  