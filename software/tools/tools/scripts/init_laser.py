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


def on_utime(channel, data):
  m = utime_t.decode(data)
  t = m.utime
  #print t
  if ( abs(t - 300000) < 5000): # after 3secs send a spin rate
    print "Set lidar spinning at 7 RPM and Cameras to 5 FPS and Compression to Low | %d" %(t)
    o = sensor_request_t()
    o.utime = t
    o.spindle_rpm = 7 # 0.733038 rad/sec
    o.head_fps = 5
    o.hand_fps = 5
    o.camera_compression = 1  
    lc.publish("SENSOR_REQUEST", o.encode())    

  if ( abs(t - 9000000) < 5000): # after 3secs send a spin rate
    print "Set pitch of neck to 35degees %d" %(t)
    p = neck_pitch_t()
    p.utime = t
    p.pitch = 0.61 # 35 degrees seems about right
    lc.publish("DESIRED_NECK_PITCH", p.encode())    
    
  #if (abs(t - 3000000) < 5000): # after 4secs send the data request
  #  print "gimmie the data or else - %d" %(t)
  #  o = data_request_list_t()
  #  o.utime = t
  #  r1 = data_request_t()
  #  r1.type = 1
  #  r1.period = 10
  #  o.num_requests= 3
  #  o.requests.append(r1)
  #  r2 = data_request_t()
  #  r2.period = 10
  #  r2.type =2
  #  o.requests.append(r2)
  #  r3 = data_request_t()
  #  r3.period = 10
  #  r3.type =6
  #  o.requests.append(r3)
  #  lc.publish("DATA_REQUEST", o.encode())    

####################################################################
lc = lcm.LCM()
print "started"
sub1 = lc.subscribe("ROBOT_UTIME", on_utime)
while True:
  lc.handle()

lc.unsubscribe(sub)
