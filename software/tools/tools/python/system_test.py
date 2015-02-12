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
  global test_init, test_stop
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
  global test_init, test_stop
  msg = utime_t()
  msg.utime = timestamp_now()
  lc.publish(test_init, msg.encode() )
  print "Sent init message"
  print test_init
  lc.unsubscribe(sub2)


def on_msg3(channel, data):
  global test_init, test_stop
  m = utime_t.decode(data)
  print "got termination message"
  print test_stop
  sys.exit()


if ((len(sys.argv)) < 2):
  print "no test specified, doing valve test"
  test_name = 'VALVE'
else:
  print sys.argv[1].upper()
  test_name = sys.argv[1].upper()

test_init = "AUTONOMOUS_TEST_" + test_name
test_stop = "AUTONOMOUS_TEST_" + test_name + "_DONE"
print test_init
print test_stop

lc = lcm.LCM()
print "started test"
sub1 = lc.subscribe("EST_ROBOT_STATE", on_msg1)
sub2 = lc.subscribe("ATLAS_COMMAND", on_msg2)
sub3 = lc.subscribe(test_stop, on_msg3)
while True:
  lc.handle()
lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
lc.unsubscribe(sub3)
