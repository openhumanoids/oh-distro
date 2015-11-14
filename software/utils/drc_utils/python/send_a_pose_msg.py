#!/usr/bin/python
import os,sys
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import math
import numpy as np
import lcm
from bot_core.pose_t import pose_t
import time

def euler_to_quat(rpy):
  roll =  rpy[0]
  pitch = rpy[1]
  yaw =   rpy[2]

  sy = math.sin(yaw*0.5);
  cy = math.cos(yaw*0.5);
  sp = math.sin(pitch*0.5);
  cp = math.cos(pitch*0.5);
  sr = math.sin(roll*0.5);
  cr = math.cos(roll*0.5);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
  return np.array([w,x,y,z])


lc = lcm.LCM()
print "Send POSE_BODY..."
msg = pose_t();

msg.utime = 0;
msg.pos = [0,0,1.2]


msg.orientation = euler_to_quat([0, 17.0*np.pi/180.0,0])
lc.publish("POSE_BODY", msg.encode())

