#!/usr/bin/python
import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.utime_t import utime_t
from drc.twist_timed_t import twist_timed_t
from drc.vector_3d_t import vector_3d_t

from drc.data_request_t import data_request_t
from drc.data_request_list_t import data_request_list_t

from drc.sensor_request_t import sensor_request_t
from drc.neck_pitch_t import neck_pitch_t

########################################################################################


def send_stuff(t):
  print "Set lidar spinning at 7 RPM and Cameras to 5 FPS and Compression to Low | %d" %(t)
  o = sensor_request_t()
  o.utime = t
  o.spindle_rpm = 7 # 0.733038 rad/sec
  o.head_fps = 5
  o.hand_fps = 5
  o.camera_compression = 1  
  lc.publish("SENSOR_REQUEST", o.encode())    
  print "Set pitch of neck to 35degees %d" %(t)
  p = neck_pitch_t()
  p.utime = t
  p.pitch = 0.61 # 35 degrees seems about right
  lc.publish("DESIRED_NECK_PITCH", p.encode())    

def on_utime(channel, data):
  m = utime_t.decode(data)
  t = m.utime
  #print t
  if ( abs(t - 500000) < 10000): # after 0.5secs
    send_stuff(t)

  if ( abs(t - 3000000) < 10000): # after 3secs
    send_stuff(t)
    
  if ( abs(t - 5000000) < 10000): # after 5secs
    send_stuff(t)
  
  if ( abs(t - 7000000) < 10000): # after 7secs
    send_stuff(t)

####################################################################
lc = lcm.LCM()
print "started"
sub1 = lc.subscribe("ROBOT_UTIME", on_utime)
while True:
  lc.handle()

lc.unsubscribe(sub)
