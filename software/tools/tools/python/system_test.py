#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
base_dir = os.getenv("DRC_BASE")
sys.path.append(base_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(base_dir + "/software/build/lib/python2.7/dist-packages")
from drc.robot_state_t import robot_state_t
from drc.utime_t import utime_t
from drc.neck_pitch_t import neck_pitch_t
def timestamp_now (): return int (time.time () * 1000000)


def on_msg1(channel, data):
  m = robot_state_t.decode(data)
  print "got ers"
  neckmsg = neck_pitch_t()
  neckmsg.utime = timestamp_now()
  neckmsg.pitch = 0.43
  lc.publish('DESIRED_NECK_PITCH', neckmsg.encode() )
  time.sleep(10) # for the robot to drop onto the ground

  msg = utime_t()
  msg.utime = timestamp_now()
  lc.publish('START_MIT_STAND', msg.encode() )
  print "Sent START_MIT_STAND"
  lc.unsubscribe(sub1)

def on_msg2(channel, data):
  msg = utime_t()
  msg.utime = timestamp_now()
  lc.publish('AUTONOMOUS_TEST_VALVE', msg.encode() )
  print "Sent AUTONOMOUS_TEST_VALVE"
  lc.unsubscribe(sub2)


def on_msg3(channel, data):
  m = utime_t.decode(data)
  print "got AUTONOMOUS_TEST_VALVE_DONE"
  sys.exit()


lc = lcm.LCM()
print "started test"
sub1 = lc.subscribe("EST_ROBOT_STATE", on_msg1)
sub2 = lc.subscribe("ATLAS_COMMAND", on_msg2)
sub3 = lc.subscribe("AUTONOMOUS_TEST_VALVE_DONE", on_msg3)
while True:
  lc.handle()
lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
lc.unsubscribe(sub3)
